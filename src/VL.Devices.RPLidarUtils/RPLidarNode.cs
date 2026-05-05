using Stride.Core.Mathematics;
using System.IO.Ports;
using System.Reactive.Subjects;
using VL.Core;
using VL.Core.Import;
using VL.Lib.Collections;
using ComPort = VL.Lib.IO.Ports.ComPort;

namespace Devices.RPLidar;

/// <summary>Connects to an RPLIDAR device and streams 360° point-cloud scans.</summary>
[ProcessNode(Name = "RPLidar")]
public sealed class RPLidarNode : IDisposable
{
    // ── Background thread resources ──────────────────────────────────────────
    private SerialPort? _port;
    private Thread? _readThread;
    private CancellationTokenSource _cts = new();

    // ── Reactive scan output — stable for the lifetime of this node instance
    private readonly Subject<Spread<Vector2>> _scanSubject = new();

    // ── Live scaling (written by main thread, read by background thread) ─────
    private volatile float _scaling = 1f;

    // ── Connection change-detection ──────────────────────────────────────────
    private string _lastPort = string.Empty;
    private int _lastBaud = -1;
    private int _lastTimeout = -1;
    private bool _lastEnabled = false;
    private bool _lastScan = false;
    private volatile bool _isScanning = false;

    // ── Status (written by bg thread + main thread, read by main thread) ─────
    private readonly object _statusLock = new();
    private volatile bool _isConnected = false;
    private string _deviceInfo = "";
    private string _statusText = "";
    private int _errorCode = 0;

    // ── Parsed device info (set on connect, reset on disconnect) ─────────────
    private Protocol.DeviceInfo _parsedDeviceInfo;

    // ────────────────────────────────────────────────────────────────────────
    // Update — called every frame by vvvv on the main thread
    // ────────────────────────────────────────────────────────────────────────

    /// <summary>
    /// Manages the device connection and emits a reactive stream of 360° scans.
    /// </summary>
    public void Update(
        out IObservable<Spread<Vector2>> result,
        out string deviceInfo,
        out bool isConnected,
        out string status,
        out int errorCode,
        ComPort portName = default!,
        RPLidarBaudRate baudRate = RPLidarBaudRate.ASeries,
        int timeout = 2000,
        bool scan = true,
        bool enabled = false,
        float scaling = 1f)
    {
        // Always update scaling so the background thread picks it up immediately.
        _scaling = scaling;

        string port = portName?.Value ?? string.Empty;
        int baud = (int)baudRate;

        bool needReconnect =
            enabled != _lastEnabled ||
            port != _lastPort ||
            baud != _lastBaud ||
            timeout != _lastTimeout;

        bool scanChanged = scan != _lastScan;

        if (needReconnect)
        {
            FullDisconnect();
            _lastEnabled = enabled;
            _lastPort = port;
            _lastBaud = baud;
            _lastTimeout = timeout;
            _lastScan = scan;

            if (enabled && TryOpenAndIdentify(port, baud, timeout) && scan)
                TryStartScanning(timeout);
        }
        else if (scanChanged && enabled)
        {
            _lastScan = scan;
            if (scan)
            {
                // StopScanning closes the port to unblock Read() quickly.
                // If the port was closed, reconnect before starting the scan.
                if (!_isConnected)
                    TryOpenAndIdentify(_lastPort, _lastBaud, _lastTimeout);
                TryStartScanning(timeout);
            }
            else
            {
                StopScanning();
            }
        }

        result = _scanSubject;
        lock (_statusLock) { deviceInfo = _deviceInfo; status = _statusText; errorCode = _errorCode; }
        isConnected = _isConnected;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Connection lifecycle
    // ────────────────────────────────────────────────────────────────────────

    /// <summary>Opens the port and runs GET_INFO + GET_HEALTH. Does NOT start scanning.</summary>
    private bool TryOpenAndIdentify(string portName, int baudRate, int timeout)
    {
        try
        {
            _cts = new CancellationTokenSource();

            _port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                // Generous read timeout for the initial handshake.
                ReadTimeout = Math.Max(timeout, 3000),
                WriteTimeout = 1000
                // DtrEnable intentionally left at OS default (false on Windows).
                // A-series devices use DTR LOW to enable the motor; setting it HIGH
                // during init causes the USB-UART chip to emit spurious bytes that
                // confuse the GET_INFO response scanner.
            };
            _port.Open();
            _port.DiscardInBuffer();

            // Identify the device without sending CMD_STOP first.
            // CMD_STOP before GET_INFO breaks the A1 (causes it to stop responding).
            // The C1/S-series do not need it here — CMD_STOP is only required when
            // stopping an active scan, which FullDisconnect/StopScanning handle.
            if (!TryGetInfo(timeout))
            {
                SetStatus("Device info failed — check baud rate and connection");
                Cleanup();
                return false;
            }

            TryGetHealth(timeout);
            _isConnected = true;
            return true;
        }
        catch (Exception ex)
        {
            SetStatus("Connect failed: " + ex.Message);
            Cleanup();
            return false;
        }
    }

    /// <summary>Sends CMD_SCAN, enables the motor via DTR, and starts the read thread.</summary>
    private void TryStartScanning(int timeout)
    {
        if (_port?.IsOpen != true || _isScanning) return;
        try
        {
            // Motor on.
            // A2/A3-series: DTR enables the motor circuit AND a PWM command sets the speed.
            // A1-series: DTR alone is sufficient (no PWM command supported).
            // C/S-series: built-in motor controller, DTR and PWM are irrelevant.
            _port.DtrEnable = false;
            if (_parsedDeviceInfo.NeedsPwmMotorControl)
            {
                SendMotorPwm(Protocol.DEFAULT_MOTOR_PWM);
                Thread.Sleep(50); // give the device time to process the PWM command
            }

            SendCmd(Protocol.CMD_SCAN);
            var desc = ReadDescriptor(timeout);
            if (!desc.Valid || desc.DataType != Protocol.RESP_SCAN)
            {
                SetStatus($"Unexpected scan response type 0x{desc.DataType:X2}");
                _port.DtrEnable = true; // motor back off on failure
                return;
            }

            _port.ReadTimeout = timeout;
            _isScanning = true;

            _readThread = new Thread(() => ScanLoop(_cts.Token))
            {
                IsBackground = true,
                Name = "RPLidar.ScanLoop"
            };
            _readThread.Start();
        }
        catch (Exception ex)
        {
            SetStatus("Scan start failed: " + ex.Message);
            try { _port!.DtrEnable = true; } catch { }
        }
    }

    /// <summary>Stops scanning but keeps the port open (device remains identified).</summary>
    private void StopScanning()
    {
        if (!_isScanning) return;
        _cts.Cancel();
        if (_port?.IsOpen == true)
        {
            try { SendCmd(Protocol.CMD_STOP); Thread.Sleep(50); } catch { }
            if (_parsedDeviceInfo.NeedsPwmMotorControl)
                try { SendMotorPwm(Protocol.STOP_MOTOR_PWM); } catch { }
            try { _port.DtrEnable = true; } catch { } // motor off
        }
        // Close the port BEFORE joining: the background thread may be blocked in
        // Read() with a long timeout; closing the port causes Read() to throw an
        // IOException which the thread catches and exits immediately.
        var savedPort = _port;
        _port = null;
        _isConnected = false;
        try { savedPort?.DiscardInBuffer(); } catch { }
        try { savedPort?.Close(); } catch { }
        try { savedPort?.Dispose(); } catch { }
        _readThread?.Join(millisecondsTimeout: 1000);
        _readThread = null;
        _cts = new CancellationTokenSource(); // fresh token for next TryStartScanning
        _isScanning = false;
    }

    /// <summary>Stops scanning and closes the port entirely.</summary>
    private void FullDisconnect()
    {
        if (_isScanning)
        {
            _cts.Cancel();
            if (_port?.IsOpen == true)
            {
                try { SendCmd(Protocol.CMD_STOP); Thread.Sleep(50); } catch { }
                if (_parsedDeviceInfo.NeedsPwmMotorControl)
                    try { SendMotorPwm(Protocol.STOP_MOTOR_PWM); } catch { }
                try { _port.DtrEnable = true; } catch { } // motor off
            }
            // Close port before joining — interrupts any blocked Read() in the
            // background thread so Join() returns in milliseconds, not seconds.
            Cleanup();
            _readThread?.Join(millisecondsTimeout: 1000);
            _readThread = null;
            _isScanning = false;
        }
        else
        {
            Cleanup();
        }
        _parsedDeviceInfo = default;
        lock (_statusLock) { _deviceInfo = ""; _statusText = ""; _errorCode = 0; }
    }

    private void Cleanup()
    {
        if (_port is null) return;
        try { _port.DiscardInBuffer(); } catch { }
        try { _port.Close(); } catch { }
        try { _port.Dispose(); } catch { }
        _port = null;
        _isConnected = false;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Background scan loop
    // ────────────────────────────────────────────────────────────────────────

    private void ScanLoop(CancellationToken ct)
    {
        var rawBuf = new byte[Protocol.SCAN_UNIT];
        // Pre-allocate a list sized for one RPLIDAR C1 scan (~5 000 points).
        var pending = new List<Vector2>(5500);
        bool started = false;

        // The RPLIDAR waits for its motor to reach stable speed before emitting
        // any data. Use a long initial timeout so this spin-up phase is covered;
        // once data is flowing switch to a tighter watchdog.
        const int SpinUpTimeoutMs = 15_000;
        const int RunningTimeoutMs = 5_000;

        while (!ct.IsCancellationRequested)
        {
            try
            {
                if (!started)
                {
                    SetStatus("Waiting for motor…");
                    _port!.ReadTimeout = SpinUpTimeoutMs;
                }

                ReadExact(rawBuf, 0, rawBuf.Length, ct);
                if (ct.IsCancellationRequested) break;

                if (!started)
                {
                    started = true;
                    _port!.ReadTimeout = RunningTimeoutMs;
                    SetStatus("Scanning");
                }

                if (!Protocol.TryParseMeasurement(rawBuf, out var m)) continue;

                // When the start-of-scan flag arrives, publish the completed scan.
                if (m.IsNewScan && pending.Count > 0)
                {
                    PublishScan(pending);
                    pending.Clear();
                }

                // Skip zero-distance (no-object) measurements.
                if (m.Distance < 0.01f) continue;

                float s = _scaling;
                float rad = m.Angle * (MathF.PI / 180f);
                float distMetr = m.Distance * 0.001f * s;   // mm → m, apply scaling
                pending.Add(new Vector2(
                    distMetr * MathF.Cos(rad),
                    distMetr * MathF.Sin(rad)));
            }
            catch (OperationCanceledException) { break; }
            catch when (ct.IsCancellationRequested) { break; }
            catch (TimeoutException) when (!ct.IsCancellationRequested)
            {
                SetStatus(started
                    ? "Scan stopped — no data for 5 s (device disconnected?)"
                    : "Motor did not start within 15 s — check device power");
                break;
            }
            catch (Exception ex)
            {
                SetStatus("Scan error: " + ex.Message);
                break;
            }
        }

        _isConnected = false;
        _isScanning = false;
    }

    private void PublishScan(List<Vector2> src)
    {
        var builder = new SpreadBuilder<Vector2>(src.Count);
        foreach (var v in src) builder.Add(v);
        _scanSubject.OnNext(builder.ToSpread());
    }

    // ────────────────────────────────────────────────────────────────────────
    // Protocol helpers
    // ────────────────────────────────────────────────────────────────────────

    private void SendCmd(byte cmd)
    {
        if (_port?.IsOpen != true) return;
        var p = Protocol.SimpleCommand(cmd);
        _port.Write(p, 0, p.Length);
    }

    /// <summary>Sends CMD_SET_MOTOR_PWM for A2/A3-series devices. No-ops on other series.</summary>
    private void SendMotorPwm(ushort pwm)
    {
        if (_port?.IsOpen != true) return;
        var p = Protocol.MotorPwmCommand(pwm);
        _port.Write(p, 0, p.Length);
    }

    /// <summary>
    /// Read a 7-byte response descriptor, scanning byte-by-byte for the 0xA5 0x5A
    /// sync pattern. This tolerates stray bytes that may precede the real response.
    /// </summary>
    private Protocol.Descriptor ReadDescriptor(int timeoutMs)
    {
        var buf = new byte[Protocol.DESC_SIZE];
        var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);

        byte prev = 0;
        while (true)
        {
            if (DateTime.UtcNow > deadline) return default;
            byte cur = (byte)_port!.ReadByte();   // throws TimeoutException on ReadTimeout
            if (prev == Protocol.RESP_SYNC1 && cur == Protocol.RESP_SYNC2)
            {
                buf[0] = prev;
                buf[1] = cur;
                break;
            }
            prev = cur;
        }

        int read = 2;
        while (read < buf.Length)
        {
            if (DateTime.UtcNow > deadline) return default;
            read += _port!.Read(buf, read, buf.Length - read);
        }
        return Protocol.ParseDescriptor(buf);
    }

    /// <summary>Read exactly <paramref name="count"/> bytes, blocking until done or cancelled.</summary>
    private void ReadExact(byte[] buf, int offset, int count, CancellationToken ct)
    {
        while (count > 0)
        {
            ct.ThrowIfCancellationRequested();
            int n = _port!.Read(buf, offset, count);
            offset += n;
            count -= n;
        }
    }

    private bool TryGetInfo(int timeoutMs)
    {
        try
        {
            SendCmd(Protocol.CMD_GET_INFO);
            var desc = ReadDescriptor(timeoutMs);
            if (!desc.Valid || desc.DataType != Protocol.RESP_DEVINFO) return false;

            var data = new byte[Protocol.DEVINFO_SIZE];
            ReadBytes(data, timeoutMs);

            var info = Protocol.ParseDeviceInfo(data);
            _parsedDeviceInfo = info;

            string hint = info.IsHighSpeedDevice && _lastBaud < 460800
                ? "\n⚠  C1/S-series detected — set Baud Rate to C And S Series (460800)"
                : string.Empty;

            lock (_statusLock)
                _deviceInfo =
                    $"Model:    {info.ModelName}\n" +
                    $"Firmware: {info.FirmwareMajor}.{info.FirmwareMinor}\n" +
                    $"Hardware: v{info.Hardware}\n" +
                    $"S/N:      {info.SerialNumber}" +
                    hint;
            return true;
        }
        catch { return false; }
    }

    private bool TryGetHealth(int timeoutMs)
    {
        try
        {
            SendCmd(Protocol.CMD_GET_HEALTH);
            var desc = ReadDescriptor(timeoutMs);
            if (!desc.Valid) return false;

            var data = new byte[Protocol.DEVHEALTH_SIZE];
            ReadBytes(data, timeoutMs);

            var (st, ec) = Protocol.ParseHealth(data);
            lock (_statusLock)
            {
                _errorCode = ec;
                _statusText = st switch { 0 => "Healthy", 1 => "Warning", _ => $"Error 0x{ec:X4}" };
            }
            return true;
        }
        catch { return false; }
    }

    /// <summary>Read exactly <paramref name="buf"/>.Length bytes with a wall-clock deadline.</summary>
    private void ReadBytes(byte[] buf, int timeoutMs)
    {
        var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);
        int read = 0;
        while (read < buf.Length)
        {
            if (DateTime.UtcNow > deadline) throw new TimeoutException("RPLIDAR read timed out");
            read += _port!.Read(buf, read, buf.Length - read);
        }
    }

    private void SetStatus(string msg)
    {
        lock (_statusLock) _statusText = msg;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Enums
    // ────────────────────────────────────────────────────────────────────────

    /// <summary>Serial baud rate for the connected RPLIDAR model.</summary>
    public enum RPLidarBaudRate
    {
        /// <summary>A1, A2, A3 series — 115200 baud.</summary>
        ASeries = 115200,
        /// <summary>Some A2/A3 variants — 256000 baud.</summary>
        ASeriesHighSpeed = 256000,
        /// <summary>C1, S1, S2, S3 series — 460800 baud.</summary>
        CAndSSeries = 460800
    }

    // ────────────────────────────────────────────────────────────────────────
    // IDisposable
    // ────────────────────────────────────────────────────────────────────────

    /// <summary>Stops the scan, releases the serial port, and completes the scan observable.</summary>
    public void Dispose()
    {
        FullDisconnect();
        _scanSubject.OnCompleted();
    }
}
