namespace Devices.RPLidar;

/// <summary>
/// Low-level RPLIDAR serial protocol constants, packet building, and parsing.
/// Covers the A/S/C series protocol documented in Slamtec's LR001/LR002 spec sheets.
/// </summary>
internal static class Protocol
{
    // ── Request framing ──────────────────────────────────────────────────────
    public const byte SYNC = 0xA5;

    // ── Commands (no payload) ────────────────────────────────────────────────
    public const byte CMD_STOP       = 0x25;
    public const byte CMD_RESET      = 0x40;
    public const byte CMD_SCAN       = 0x20;   // standard scan, all models
    public const byte CMD_GET_INFO   = 0x50;
    public const byte CMD_GET_HEALTH = 0x52;

    // ── Commands (with payload) ──────────────────────────────────────────────
    // A2/A3 series only — sets the motor PWM speed (0 = stop, 660 = default).
    public const byte   CMD_SET_MOTOR_PWM   = 0xF0;
    public const ushort DEFAULT_MOTOR_PWM   = 660;
    public const ushort STOP_MOTOR_PWM      = 0;

    // ── Response framing ─────────────────────────────────────────────────────
    public const byte RESP_SYNC1 = 0xA5;
    public const byte RESP_SYNC2 = 0x5A;
    public const int  DESC_SIZE  = 7;   // response descriptor length in bytes

    // ── Response data-type codes ─────────────────────────────────────────────
    public const byte RESP_DEVINFO   = 0x04;
    public const byte RESP_DEVHEALTH = 0x06;
    public const byte RESP_SCAN      = 0x81;   // standard scan measurement stream

    // ── Payload sizes ────────────────────────────────────────────────────────
    public const int DEVINFO_SIZE   = 20;  // bytes
    public const int DEVHEALTH_SIZE = 3;   // bytes
    public const int SCAN_UNIT      = 5;   // bytes per measurement

    // ── Packet building ──────────────────────────────────────────────────────

    /// <summary>Build a simple two-byte command request (no payload).</summary>
    public static byte[] SimpleCommand(byte cmd) => new[] { SYNC, cmd };

    /// <summary>
    /// Build a payload command: [0xA5] [cmd] [len] [payload...] [checksum].
    /// Checksum is the XOR of ALL preceding bytes (SYNC + CMD + len + payload),
    /// matching the Slamtec protocol spec and the RPLidar4Net reference implementation.
    /// </summary>
    public static byte[] PayloadCommand(byte cmd, byte[] payload)
    {
        var pkt = new byte[3 + payload.Length + 1];
        pkt[0] = SYNC;
        pkt[1] = cmd;
        pkt[2] = (byte)payload.Length;
        Array.Copy(payload, 0, pkt, 3, payload.Length);
        byte checksum = 0;
        for (int i = 0; i < 3 + payload.Length; i++) checksum ^= pkt[i];
        pkt[3 + payload.Length] = checksum;
        return pkt;
    }

    /// <summary>Build a CMD_SET_MOTOR_PWM packet for A2/A3-series devices.</summary>
    public static byte[] MotorPwmCommand(ushort pwm)
        => PayloadCommand(CMD_SET_MOTOR_PWM, new[] { (byte)(pwm & 0xFF), (byte)(pwm >> 8) });

    // ── Response descriptor ──────────────────────────────────────────────────

    public readonly struct Descriptor
    {
        public readonly uint DataLength;
        public readonly byte SendMode;   // 0 = single response, 1+ = continuous stream
        public readonly byte DataType;
        public readonly bool Valid;

        internal Descriptor(uint len, byte mode, byte type)
        { DataLength = len; SendMode = mode; DataType = type; Valid = true; }
    }

    /// <summary>Parse a 7-byte response descriptor from the start of <paramref name="buf"/>.</summary>
    public static Descriptor ParseDescriptor(ReadOnlySpan<byte> buf)
    {
        if (buf[0] != RESP_SYNC1 || buf[1] != RESP_SYNC2)
            return default;

        // Bytes 2-5: 30-bit data-length | 2-bit send-mode (little-endian)
        uint combined = (uint)buf[2]
                      | ((uint)buf[3] << 8)
                      | ((uint)buf[4] << 16)
                      | ((uint)buf[5] << 24);

        return new Descriptor(
            len:  combined & 0x3FFF_FFFF,
            mode: (byte)((combined >> 30) & 0x03),
            type: buf[6]);
    }

    // ── Standard scan measurement ─────────────────────────────────────────────

    public readonly struct Measurement
    {
        /// <summary>Bearing in degrees, 0–360, 0 = forward.</summary>
        public readonly float Angle;
        /// <summary>Radial distance in millimetres. 0 = no object detected.</summary>
        public readonly float Distance;
        /// <summary>Measurement quality 0–63 (higher = better).</summary>
        public readonly byte Quality;
        /// <summary>True on the first point of each new 360° scan.</summary>
        public readonly bool IsNewScan;

        internal Measurement(float a, float d, byte q, bool s)
        { Angle = a; Distance = d; Quality = q; IsNewScan = s; }
    }

    /// <summary>
    /// Try to parse one 5-byte standard-scan measurement unit.
    /// Returns false when the packet fails its built-in check bits.
    /// </summary>
    public static bool TryParseMeasurement(ReadOnlySpan<byte> buf, out Measurement m)
    {
        m = default;

        // Byte 0: quality[7:2] | S[1] | ¬S[0]  — S and ¬S must differ
        bool s    = (buf[0] & 0x01) != 0;
        bool sInv = (buf[0] & 0x02) != 0;
        if (s == sInv) return false;

        // Byte 1, bit 0: check-bit must be 1
        if ((buf[1] & 0x01) == 0) return false;

        byte  quality = (byte)(buf[0] >> 2);

        // Angle: 15-bit fixed-point Q6 (÷64 → degrees)
        //   lower 7 bits from buf[1][7:1], upper 8 bits from buf[2]
        float angle = ((buf[1] >> 1) | (buf[2] << 7)) / 64.0f;

        // Distance: 16-bit fixed-point Q2 (÷4 → mm)
        float dist = (buf[3] | (buf[4] << 8)) / 4.0f;

        m = new Measurement(angle, dist, quality, s);
        return true;
    }

    // ── Device info ───────────────────────────────────────────────────────────

    public readonly struct DeviceInfo
    {
        /// <summary>
        /// Raw first byte of the response.
        /// A-series: full 8-bit model ID.
        /// S/C-series: upper nibble = MajorModel, lower nibble = SubModel.
        /// </summary>
        public readonly byte ModelByte;
        public readonly byte FirmwareMinor;
        public readonly byte FirmwareMajor;
        public readonly byte Hardware;
        /// <summary>16-byte serial number formatted as a 32-character hex string.</summary>
        public readonly string SerialNumber;

        // S/C series nibble accessors
        public byte MajorModel => (byte)(ModelByte >> 4);
        public byte SubModel   => (byte)(ModelByte & 0x0F);

        /// <summary>
        /// Human-readable model name.
        /// Uses the known A-series IDs from Slamtec docs; falls back to nibble-decoded
        /// S/C model for newer devices (MajorModel ≥ 4).
        /// </summary>
        public string ModelName => ModelByte switch
        {
            0x18 => "A1M8",
            0x28 => "A2M6",
            0x2B => "A2M8",
            0x2A => "A2M4",
            0x2C => "A2M12",
            0x61 => "A3M1",
            _ when MajorModel == 4 => $"C1 (sub-model {SubModel})",
            _ when MajorModel == 6 => $"S1 (sub-model {SubModel})",
            _ when MajorModel == 7 => $"S2 (sub-model {SubModel})",
            _ when MajorModel == 8 => $"S3 (sub-model {SubModel})",
            _                      => $"Unknown (0x{ModelByte:X2})"
        };

        /// <summary>True for C1 / S-series devices that require 460800 baud.</summary>
        public bool IsHighSpeedDevice => MajorModel >= 4;

        /// <summary>
        /// True for A2-series devices (MajorModel == 2) that require CMD_SET_MOTOR_PWM
        /// to actually spin the motor. A1 devices use DTR alone; C/S devices have a
        /// built-in motor controller and ignore the PWM command.
        /// </summary>
        public bool NeedsPwmMotorControl => MajorModel == 2;

        internal DeviceInfo(byte model, byte fwMin, byte fwMaj, byte hw, string sn)
        { ModelByte = model; FirmwareMinor = fwMin; FirmwareMajor = fwMaj; Hardware = hw; SerialNumber = sn; }
    }

    /// <summary>Parse a 20-byte GET_INFO response payload.</summary>
    public static DeviceInfo ParseDeviceInfo(ReadOnlySpan<byte> buf)
    {
        // buf[0]     = model / MajorModel+SubModel
        // buf[1]     = firmware minor
        // buf[2]     = firmware major
        // buf[3]     = hardware revision
        // buf[4..19] = 16-byte serial number, LSB-first
        string sn = BitConverter.ToString(buf.Slice(4, 16).ToArray()).Replace("-", "");
        return new DeviceInfo(buf[0], buf[1], buf[2], buf[3], sn);
    }

    // ── Device health ─────────────────────────────────────────────────────────

    /// <summary>
    /// Parse a 3-byte GET_HEALTH response payload.
    /// Returns (status, errorCode) where status 0=Good, 1=Warning, 2=Error.
    /// </summary>
    public static (byte status, int errorCode) ParseHealth(ReadOnlySpan<byte> buf)
        => (buf[0], buf[1] | (buf[2] << 8));
}
