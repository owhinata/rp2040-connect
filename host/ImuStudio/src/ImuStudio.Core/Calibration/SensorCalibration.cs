// SPDX-License-Identifier: Apache-2.0
using System.Buffers.Binary;
using System.Numerics;

namespace ImuStudio.Core.Calibration;

public enum SensorKind { Accel = 0, Gyro = 1 }

/// <summary>
/// Conversion of one sensor from raw LSB to m/s² or rad/s:
/// <c>out = M · (raw − bias)</c>. The coefficients are the integers the
/// board keeps (drivers/lsm6dso/lsm6dso_cal.h), so a value stored on the
/// board converts here as it does there.
/// </summary>
public sealed class SensorCalibration
{
    public const int BlobSize = 56;
    public const int BlobVersion = 1;
    public const int BiasFraction = 8;
    /// <summary>Fraction bits of M: 22 of the board's output unit plus 14 of the coefficient.</summary>
    private const int MatrixFraction = 36;

    private readonly double[] _m = new double[9];

    public SensorKind Kind { get; }
    /// <summary>Sensitivity this applies to: µg/LSB or µdps/LSB.</summary>
    public uint Gain { get; }
    /// <summary>True for a calibration stored on the board, false for the data sheet sensitivity.</summary>
    public bool IsStored { get; }
    /// <summary>Bias in LSB times 2^<see cref="BiasFraction"/>.</summary>
    public int[] Bias { get; } = new int[3];

    private SensorCalibration(SensorKind kind, uint gain, bool stored, ReadOnlySpan<int> m, ReadOnlySpan<int> bias)
    {
        Kind = kind;
        Gain = gain;
        IsStored = stored;
        for (int i = 0; i < 9; i++)
        {
            _m[i] = m[i] / (double)(1L << MatrixFraction);
        }
        bias.CopyTo(Bias);
    }

    public static string Key(SensorKind kind, uint gain) =>
        $"imu.cal.{(kind == SensorKind.Accel ? "accel" : "gyro")}.{gain}";

    public static SensorCalibration Nominal(SensorKind kind, uint gain)
    {
        // Output units per µg (standard gravity) and per µdps, times 2^56, as on the board.
        ulong perGain = kind == SensorKind.Accel ? 706643604572UL : 1257642267UL;
        int diagonal = (int)((gain * perGain + (1UL << 19)) >> 20);
        Span<int> m = stackalloc int[9];
        m[0] = m[4] = m[8] = diagonal;
        return new SensorCalibration(kind, gain, false, m, stackalloc int[3]);
    }

    /// <summary>Decodes a value of the board's KV store. Null if it is not one for this sensor and sensitivity.</summary>
    public static SensorCalibration? TryDecode(ReadOnlySpan<byte> blob, SensorKind kind, uint gain)
    {
        if (blob.Length != BlobSize || blob[0] != BlobVersion || blob[2] != 0 || blob[3] != 0 ||
            blob[1] != (byte)kind || BinaryPrimitives.ReadUInt32LittleEndian(blob[4..]) != gain)
        {
            return null;
        }
        Span<int> values = stackalloc int[12];
        for (int i = 0; i < 12; i++)
        {
            values[i] = BinaryPrimitives.ReadInt32LittleEndian(blob[(8 + 4 * i)..]);
        }
        return new SensorCalibration(kind, gain, true, values[..9], values[9..]);
    }

    public Vector3 Apply(short x, short y, short z) => Apply(x, y, z, Bias);

    /// <summary>Converts with another bias (LSB times 2^<see cref="BiasFraction"/>), such as a tracked one.</summary>
    public Vector3 Apply(short x, short y, short z, ReadOnlySpan<int> bias)
    {
        const double scale = 1.0 / (1 << BiasFraction);
        double a = x - bias[0] * scale;
        double b = y - bias[1] * scale;
        double c = z - bias[2] * scale;
        return new Vector3(
            (float)(_m[0] * a + _m[1] * b + _m[2] * c),
            (float)(_m[3] * a + _m[4] * b + _m[5] * c),
            (float)(_m[6] * a + _m[7] * b + _m[8] * c));
    }
}
