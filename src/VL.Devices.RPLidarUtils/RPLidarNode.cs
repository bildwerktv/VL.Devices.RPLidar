using Stride.Core.Mathematics;
using System.IO.Ports;
using VL.Core;
using VL.Core.Import;
using VL.Lib.Collections;
using ComPort = VL.Lib.IO.Ports.ComPort;

namespace Devices.RPLidar;

[ProcessNode(Name = "RPLidar")]
public sealed class RPLidarNode : IDisposable
{
    // ── Background thread resources ──────────────────────────────────────────
    private SerialPort? _port;
    private Thread? _readThread;
    private CancellationTokenSource _cts = new();

    // ── Scan output (shared between background thread and Update) ────────────
    private readonly object _scanLock = new();
    private Spread<Vector2> _latestScan = Spread<Vector2>.Empty;

    // ── Live scaling (written by main thread, read by background thread) ─────
    private volatile float _scaling = 1f;

    // ── Connection change-detection ──────────────────────────────────────────
    private string _lastPort = string.Empty;
    private int _lastBaud = -1;
    private int _lastTimeout = -1;
    private bool _lastEnabled = false;

    // ── Status (written by background thread, read by main thread) ───────────
    private readonly object _statusLock = new();
    private volatile bool _isConnected = false;
    private string _deviceInfo = "";
    private string _statusText = "";
    private int _errorCode = 0;

    // ────────────────────────────────────────────────────────────────────────
    // Update — called every frame by vvvv on the main thread
    // ────────────────────────────────────────────────────────────────────────

    /// <summary>
    /// Manages the device connection and returns the most recent 360° scan.
    /// </summary>
    public void Update(
        out Spread<Vector2> points,
        out string deviceInfo,
        out bool isConnected,
        out string status,
        out int errorCode,
        ComPort portName = default!,
        RPLidarBaudRate baudRate = RPLidarBaudRate.ASeries,
        int timeout = 2000,
        bool enabled = false,
        float scaling = 1f)
    {
        // Always update scaling so the background thread picks it up immediately.
        _scaling = scaling;

        string port = portName?.Value ?? string.Empty;

        bool needReconnect =
            enabled != _lastEnabled ||
            port != _lastPort ||
            (int)baudRate != _lastBaud ||
            timeout != _lastTimeout;

        if (needReconnect)
        {
            Disconnect();
            // Store the new values before TryConnect so that internal helpers
            // (e.g. the C1 baud-rate hint in TryGetInfo) see the correct baud rate.
            _lastEnabled = enabled;
            _lastPort = port;
            _lastBaud = (int)baudRate;
            _lastTimeout = timeout;
            if (enabled) TryConnect(port, (int)baudRate, timeout);
        }

        lock (_scanLock) points = _latestScan;
        lock (_statusLock) { deviceInfo = _deviceInfo; status = _statusText; errorCode = _errorCode; }
        isConnected = _isConnected;
    }

    // ────────────────────────────────────────────────────────────────────────
    // Connection lifecycle
    // ────────────────────────────────────────────────────────────────────────

    private void TryConnect(string portName, int baudRate, int timeout)
    {
        try
        {
            _cts = new CancellationTokenSource();

            _port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                // Use a generous read timeout for the initial handshake.
                // The configured timeout applies to the scan loop later.
                ReadTimeout = Math.Max(timeout, 3000),
                WriteTimeout = 1000
            };
            _port.Open();
            _port.DiscardInBuffer();

            // Send STOP and flush. Do NOT send CMD_RESET: the C1 (and S-series)
            // output a multi-second boot banner after reset that corrupts the
            // response descriptor for the subsequent GET_INFO command.
            SendCmd(Protocol.CMD_STOP);
            Thread.Sleep(300);
            _port.DiscardInBuffer();
            Thread.Sleep(100);      // catch any trailing bytes still arriving
            _port.DiscardInBuffer();

            // Identify the device (also emits a baud-rate hint for C1).
            if (!TryGetInfo(timeout))
            {
                SetStatus("Device info failed — verify baud rate (C1 needs 460800)");
                Cleanup();
                return;
            }

            TryGetHealth(timeout);

            // Start the standard scan (works on every RPLIDAR model).
            SendCmd(Protocol.CMD_SCAN);
            var desc = ReadDescriptor(timeout);
            if (!desc.Valid || desc.DataType != Protocol.RESP_SCAN)
            {
                SetStatus($"Unexpected scan response type 0x{desc.DataType:X2}");
                Cleanup();
                return;
            }

            // Restore user-configured timeout for the ongoing scan loop.
            _port.ReadTimeout = timeout;

            _isConnected = true;
            SetStatus("Scanning");

            _readThread = new Thread(() => ScanLoop(_cts.Token))
            {
                IsBackground = true,
                Name = "RPLidar.ScanLoop"
            };
            _readThread.Start();
        }
        catch (Exception ex)
        {
            SetStatus("Connect failed: " + ex.Message);
            Cleanup();
        }
    }

    public enum RPLidarBaudRate
    {
        ASeries = 115200,    // → "A Series"
        CAndSSeries = 460800 // → "C And S Series"
    }

    private void Disconnect()
    {
        _cts.Cancel();

        // Tell the device to stop scanning before closing the port.
        if (_port?.IsOpen == true)
        {
            try { SendCmd(Protocol.CMD_STOP); Thread.Sleep(50); } catch { }
        }

        _readThread?.Join(millisecondsTimeout: 1500);
        _readThread = null;
        Cleanup();

        lock (_scanLock) _latestScan = Spread<Vector2>.Empty;
        lock (_statusLock) { _deviceInfo = ""; _statusText = ""; _errorCode = 0; }
    }

    private void Cleanup()
    {
        try { _port?.Close(); } catch { }
        _port?.Dispose();
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
        bool started = false;   // true once the first valid byte has arrived

        // The RPLIDAR waits for its motor to reach stable speed before emitting
        // any data.  Use a long initial timeout so this spin-up phase is covered;
        // once data is flowing switch to a tighter watchdog.
        const int SpinUpTimeoutMs = 15_000;   // generous for cold-start
        const int RunningTimeoutMs = 5_000;   // watchdog once streaming

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
    }

    private void PublishScan(List<Vector2> src)
    {
        // Build Spread outside the lock to minimise contention.
        var builder = new SpreadBuilder<Vector2>(src.Count);
        foreach (var v in src) builder.Add(v);
        var scan = builder.ToSpread();

        lock (_scanLock) _latestScan = scan;
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

    /// <summary>
    /// Read a 7-byte response descriptor, scanning for the 0xA5 0x5A sync
    /// pattern first.  This tolerates stray bytes (boot banner fragments, STOP
    /// echoes) that may precede the real response.
    /// </summary>
    private Protocol.Descriptor ReadDescriptor(int timeoutMs)
    {
        var buf = new byte[Protocol.DESC_SIZE];
        var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);

        // Scan byte-by-byte for the 0xA5 0x5A sync header.
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

        // Read the remaining 5 bytes of the descriptor.
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

            string hint = info.IsHighSpeedDevice && _lastBaud < 460800
                ? "\n⚠  C1/S-series detected — set Baud Rate to 460800 for full performance"
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
    // IDisposable
    // ────────────────────────────────────────────────────────────────────────

    public void Dispose() => Disconnect();
}
