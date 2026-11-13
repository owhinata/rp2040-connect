// SPDX-License-Identifier: Apache-2.0
using System.Buffers.Binary;
using System.Text;

namespace ImuStudio.Core.Tests;

internal static class TestData
{
    public const string Session = "000102030405060708090a0b0c0d0e0f";

    public static string Info(uint accelGain = 244, uint gyroGain = 70000, int odr = 6667, string session = Session) =>
        $$"""{"v":1,"session":"{{session}}","accel_odr_hz":{{odr}},"gyro_odr_hz":{{odr}},"accel_ug_per_lsb":{{accelGain}},"gyro_udps_per_lsb":{{gyroGain}},"tick_hz_nominal":40000,"freq_fine":0}""";

    public static byte[] InfoBytes(uint accelGain = 244, uint gyroGain = 70000, int odr = 6667) =>
        Encoding.UTF8.GetBytes(Info(accelGain, gyroGain, odr));

    /// <summary>A batch of alternating accel and gyro words, all with the same values.</summary>
    public static byte[] Batch(ulong sequence, ulong firstIndex, int pairs, (short X, short Y, short Z) accel,
                               (short X, short Y, short Z) gyro, ulong? tick = null, uint epoch = 0)
    {
        int count = pairs * 2;
        int bitmapLength = (count + 7) / 8;
        byte[] p = new byte[96 + bitmapLength + count * 6];
        "IMU1"u8.CopyTo(p);
        p[4] = 2;
        p[5] = (byte)(tick is null ? 0 : 3);
        BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(6), 96);
        Convert.FromHexString(Session).CopyTo(p, 8);
        BinaryPrimitives.WriteUInt64LittleEndian(p.AsSpan(24), sequence);
        BinaryPrimitives.WriteUInt64LittleEndian(p.AsSpan(32), firstIndex);
        BinaryPrimitives.WriteUInt64LittleEndian(p.AsSpan(40), firstIndex);
        BinaryPrimitives.WriteUInt64LittleEndian(p.AsSpan(48), tick ?? 0);
        BinaryPrimitives.WriteUInt64LittleEndian(p.AsSpan(56), tick ?? 0);
        BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(64), (ushort)count);
        BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(66), (ushort)bitmapLength);
        BinaryPrimitives.WriteUInt32LittleEndian(p.AsSpan(68), epoch);
        for (int i = 0; i < count; i++)
        {
            bool isAccel = i % 2 == 0;
            if (isAccel)
            {
                p[96 + i / 8] |= (byte)(1 << (i % 8));
            }
            (short x, short y, short z) = isAccel ? accel : gyro;
            Span<byte> word = p.AsSpan(96 + bitmapLength + i * 6);
            BinaryPrimitives.WriteInt16LittleEndian(word, x);
            BinaryPrimitives.WriteInt16LittleEndian(word[2..], y);
            BinaryPrimitives.WriteInt16LittleEndian(word[4..], z);
        }
        return p;
    }

    /// <summary>A stored calibration with a diagonal matrix.</summary>
    public static byte[] CalibrationBlob(byte kind, uint gain, double unitsPerLsb, (double X, double Y, double Z) biasLsb)
    {
        byte[] blob = new byte[56];
        blob[0] = 1;
        blob[1] = kind;
        BinaryPrimitives.WriteUInt32LittleEndian(blob.AsSpan(4), gain);
        int m = (int)Math.Round(unitsPerLsb * Math.Pow(2, 36));
        foreach (int i in new[] { 0, 4, 8 })
        {
            BinaryPrimitives.WriteInt32LittleEndian(blob.AsSpan(8 + 4 * i), m);
        }
        double[] bias = [biasLsb.X, biasLsb.Y, biasLsb.Z];
        for (int i = 0; i < 3; i++)
        {
            BinaryPrimitives.WriteInt32LittleEndian(blob.AsSpan(44 + 4 * i), (int)Math.Round(bias[i] * 256));
        }
        return blob;
    }
}
