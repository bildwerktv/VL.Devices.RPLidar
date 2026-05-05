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

    // Express scan — A2/A3/C/S series (FW ≥ 1.17). Returns 84-byte capsules.
    public const byte CMD_EXPRESS_SCAN     = 0x82;
    public const byte RESP_EXPRESS_SCAN    = 0x82;   // Standard Express Capsule (A2/A3)
    public const byte RESP_DENSE_CAPSULE   = 0x85;   // Dense Capsule (C1/S-series)
    public const int  EXPRESS_CAPSULE_SIZE = 84;     // bytes per capsule (both formats)
    public const byte EXP_SYNC_1          = 0xA;    // upper nibble of capsule byte 0
    public const byte EXP_SYNC_2          = 0x5;    // upper nibble of capsule byte 1

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

    /// <summary>
    /// Build a CMD_EXPRESS_SCAN request.
    /// Payload: working_mode=0, working_flags=0 (u16 LE), param=0 (u16 LE) — 5 zero bytes.
    /// </summary>
    public static byte[] ExpressScanCommand()
        => PayloadCommand(CMD_EXPRESS_SCAN, new byte[5]);

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

    // ── Express scan capsule ─────────────────────────────────────────────────
    //
    // Capsule layout (84 bytes, sl_lidar_response_capsule_measurement_nodes_t):
    //   byte[0]    s_checksum_1  — upper nibble = EXP_SYNC_1 (0xA)
    //   byte[1]    s_checksum_2  — upper nibble = EXP_SYNC_2 (0x5)
    //   byte[2..3] start_angle_sync_q6 (uint16 LE):
    //                bit15  = new-scan flag
    //                [14:0] = startAngle × 64  (÷64 → degrees)
    //   bytes[4..83] 16 × Cabin (5 bytes each):
    //     [0..1] distance_angle_1 (uint16 LE): bits[15:2]=distQ2, bits[1:0]=angleLo
    //     [2..3] distance_angle_2 (uint16 LE): same for second measurement
    //     [4]    offset_angles_q3: upper nibble = angleHi for meas-A, lower = angleHi for meas-B

    internal readonly struct ExpressCabin
    {
        public readonly ushort DistAngle1;    // raw uint16 as described above
        public readonly ushort DistAngle2;
        public readonly byte   OffsetAnglesQ3;

        internal ExpressCabin(ushort da1, ushort da2, byte oa)
        { DistAngle1 = da1; DistAngle2 = da2; OffsetAnglesQ3 = oa; }
    }

    internal readonly struct ExpressCapsule
    {
        public readonly bool          IsNewScan;
        public readonly float         StartAngleDeg;   // 0–360
        public readonly ExpressCabin[] Cabins;          // length 16
        public readonly bool          Valid;

        internal ExpressCapsule(bool isNew, float angle, ExpressCabin[] cabins)
        { IsNewScan = isNew; StartAngleDeg = angle; Cabins = cabins; Valid = true; }
    }

    /// <summary>
    /// Parse an 84-byte express-scan capsule buffer.
    /// Returns an invalid capsule if sync bytes are missing.
    /// </summary>
    public static ExpressCapsule ParseExpressCapsule(ReadOnlySpan<byte> buf)
    {
        if ((buf[0] >> 4) != EXP_SYNC_1 || (buf[1] >> 4) != EXP_SYNC_2)
            return default;

        ushort startWord  = (ushort)(buf[2] | (buf[3] << 8));
        bool   isNewScan  = (startWord >> 15) != 0;
        float  startAngle = (startWord & 0x7FFF) / 64.0f;

        var cabins = new ExpressCabin[16];
        int offset = 4;
        for (int i = 0; i < 16; i++, offset += 5)
        {
            ushort da1 = (ushort)(buf[offset]     | (buf[offset + 1] << 8));
            ushort da2 = (ushort)(buf[offset + 2] | (buf[offset + 3] << 8));
            byte   oa  = buf[offset + 4];
            cabins[i]  = new ExpressCabin(da1, da2, oa);
        }
        return new ExpressCapsule(isNewScan, startAngle, cabins);
    }

    /// <summary>
    /// Decode the 32 measurements carried by <paramref name="prev"/> using the
    /// start-angle of <paramref name="curr"/> for linear angle interpolation.
    /// Appends decoded <see cref="Measurement"/> values to <paramref name="output"/>.
    /// Zero-distance measurements are skipped.
    /// </summary>
    public static void DecodeCapsulePair(
        in ExpressCapsule prev, in ExpressCapsule curr,
        List<Measurement> output)
    {
        float a0 = prev.StartAngleDeg;
        float a1 = curr.StartAngleDeg;
        // Handle wrap-around at 360°
        float dA = a1 - a0;
        if (dA < 0) dA += 360f;

        int idx = 0;
        for (int c = 0; c < 16; c++)
        {
            var cabin = prev.Cabins[c];
            // Per Slamtec SDK handler_capsules.cpp:
            //   angle_offset1_q3 = (offset_angles_q3 & 0xF)  | ((distance_angle_1 & 0x3) << 4)
            //   angle_offset2_q3 = (offset_angles_q3 >> 4)   | ((distance_angle_2 & 0x3) << 4)
            // DA1 uses the LOWER nibble of offset_angles_q3; DA2 uses the UPPER nibble.
            ProcessHalf(cabin.DistAngle1, (byte)(cabin.OffsetAnglesQ3 & 0xF), idx,     a0, dA, output);
            ProcessHalf(cabin.DistAngle2, (byte)(cabin.OffsetAnglesQ3 >> 4),  idx + 1, a0, dA, output);
            idx += 2;
        }
    }

    private static void ProcessHalf(
        ushort distAngle, byte angleLo4, int idx,
        float a0, float dA, List<Measurement> output)
    {
        // Distance: clear the 2 angle bits then interpret as Q2 (mm × 4).
        // SDK: dist_q2 = distance_angle & 0xFFFC; hqNode.dist_mm_q2 = dist_q2
        // → dist_mm = (distAngle & 0xFFFC) / 4.0
        float dist = (distAngle & 0xFFFC) / 4.0f;
        if (dist < 0.01f) return;   // no-object reading

        // 6-bit Q3 angle delta:
        //   bits[3:0] = lower nibble of offset_angles_q3 (passed as angleLo4)
        //   bits[5:4] = lower 2 bits of distance_angle (the angle-offset bits)
        // SDK: angle_offset_q3 = angleLo4 | ((distAngle & 0x3) << 4)
        int   dAngleQ3  = angleLo4 | ((distAngle & 0x3) << 4);
        float dAngleDeg = dAngleQ3 / 8.0f;

        // Interpolate base angle then subtract the cabin delta
        float angle = a0 + dA * (idx / 32.0f) - dAngleDeg;
        angle = ((angle % 360f) + 360f) % 360f;   // normalise to [0, 360)

        output.Add(new Measurement(angle, dist, 15, false));
    }

    // ── Dense Capsule (C1 / S-series express scan, DataType 0x85) ────────────
    //
    // Capsule layout (84 bytes, sl_lidar_response_dense_capsule_measurement_nodes_t):
    //   byte[0]    s_checksum_1  — upper nibble = EXP_SYNC_1 (0xA)
    //   byte[1]    s_checksum_2  — upper nibble = EXP_SYNC_2 (0x5)
    //   byte[2..3] start_angle_sync_q6 (uint16 LE):
    //                bit15  = new-scan flag
    //                [14:0] = startAngle × 64  (÷64 → degrees)
    //   bytes[4..83] 40 × Cabin (2 bytes each):
    //     [0..1] distance (uint16 LE): raw distance in mm (no Q encoding)
    //
    // Angles are evenly distributed across the capsule's angular span —
    // no per-measurement angle offsets unlike the standard Express Capsule.

    internal readonly struct DenseCapsule
    {
        public readonly bool      IsNewScan;
        public readonly float     StartAngleDeg;  // 0–360
        public readonly ushort[]  Distances;       // length 40, raw mm
        public readonly bool      Valid;

        internal DenseCapsule(bool isNew, float angle, ushort[] distances)
        { IsNewScan = isNew; StartAngleDeg = angle; Distances = distances; Valid = true; }
    }

    /// <summary>
    /// Parse an 84-byte dense-capsule buffer (C1/S-series express scan response 0x85).
    /// Returns an invalid capsule if sync nibbles are missing.
    /// </summary>
    public static DenseCapsule ParseDenseCapsule(ReadOnlySpan<byte> buf)
    {
        if ((buf[0] >> 4) != EXP_SYNC_1 || (buf[1] >> 4) != EXP_SYNC_2)
            return default;

        ushort startWord  = (ushort)(buf[2] | (buf[3] << 8));
        bool   isNewScan  = (startWord >> 15) != 0;
        float  startAngle = (startWord & 0x7FFF) / 64.0f;

        var distances = new ushort[40];
        int offset = 4;
        for (int i = 0; i < 40; i++, offset += 2)
            distances[i] = (ushort)(buf[offset] | (buf[offset + 1] << 8));

        return new DenseCapsule(isNewScan, startAngle, distances);
    }

    /// <summary>
    /// Decode the 40 measurements carried by <paramref name="prev"/> using the
    /// start-angle of <paramref name="curr"/> for linear angle interpolation.
    /// Angles are evenly distributed — no per-measurement offsets.
    /// Zero-distance measurements are skipped.
    /// </summary>
    public static void DecodeDenseCapsulePair(
        in DenseCapsule prev, in DenseCapsule curr,
        List<Measurement> output)
    {
        float a0 = prev.StartAngleDeg;
        float a1 = curr.StartAngleDeg;
        float dA = a1 - a0;
        if (dA < 0) dA += 360f;

        const int cabins = 40;
        for (int i = 0; i < cabins; i++)
        {
            float dist = prev.Distances[i];   // raw mm directly (SDK: dist_q2 = dist << 2)
            if (dist < 0.01f) continue;        // no-object reading

            float angle = a0 + dA * (i / (float)cabins);
            angle = ((angle % 360f) + 360f) % 360f;
            output.Add(new Measurement(angle, dist, 15, false));
        }
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
            // A-series — fixed 8-bit model IDs (not nibble-encoded)
            0x18 => "A1M8",
            0x28 => "A2M6",
            0x2B => "A2M8",
            0x2A => "A2M4",
            0x2C => "A2M12",
            0x61 => "A3M1",  // must precede the MajorModel-6 wildcard below

            // C/S-series — upper nibble = family, lower nibble = sub-model
            _ when MajorModel == 4 => $"C1 (sub-model {SubModel})",
            _ when MajorModel == 5 => $"S1 (sub-model {SubModel})",
            _ when MajorModel == 7 => $"S2 (sub-model {SubModel})",
            _ when MajorModel == 8 => $"S3 (sub-model {SubModel})",

            _                      => $"Unknown (0x{ModelByte:X2})"
        };

        /// <summary>
        /// True when the device is an A3M1.
        /// A3M1 has model byte 0x61 (MajorModel nibble = 6), which would otherwise be
        /// misidentified as a C/S-series device. It is in fact an A-series unit that
        /// uses 256000 baud, needs PWM motor control, and supports standard express scan.
        /// </summary>
        public bool IsA3M1 => ModelByte == 0x61;

        /// <summary>
        /// True for C1 / S-series devices that require 460800 baud and use the Dense
        /// Capsule express-scan format (0x85). A3M1 is explicitly excluded — its model
        /// byte 0x61 has MajorModel nibble 6 but it is an A-series triangulation device.
        /// </summary>
        public bool IsHighSpeedDevice => MajorModel >= 4 && !IsA3M1;

        /// <summary>
        /// True for A2 and A3-series devices that require CMD_SET_MOTOR_PWM to spin the
        /// motor. A1 uses DTR alone; C/S devices have a built-in motor controller.
        /// </summary>
        public bool NeedsPwmMotorControl => MajorModel == 2 || IsA3M1;

        /// <summary>
        /// True for all devices that support CMD_EXPRESS_SCAN (FW ≥ 1.17).
        /// A2/A3 respond with Standard Express Capsule (0x82, 16 cabins × 5 bytes).
        /// C1/S-series respond with Dense Capsule (0x85, 40 cabins × 2 bytes).
        /// Both formats share the same 84-byte packet size and sync nibbles.
        /// A1 series does not support express scan.
        /// </summary>
        public bool SupportsExpressScan => MajorModel >= 2;

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
