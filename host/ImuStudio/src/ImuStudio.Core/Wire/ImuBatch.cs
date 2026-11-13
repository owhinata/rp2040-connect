// SPDX-License-Identifier: Apache-2.0
using System.Buffers.Binary;
using System.Numerics;

namespace ImuStudio.Core.Wire;

[Flags]
public enum ImuBatchFlags : byte
{
    None = 0,
    AccelTick = 1,
    GyroTick = 2,
    AccelUnix = 4,
    GyroUnix = 8,
}

/// <summary>
/// One IMU1 version 2 payload (design document, section 6): a run of FIFO
/// words, each a raw int16 x, y, z of either sensor, in the order the sensor
/// produced them.
/// </summary>
public sealed class ImuBatch
{
    public const int HeaderSize = 96;
    public const int Version = 2;
    public const int MaxWords = 384;
    public const ulong NoIndex = ulong.MaxValue;

    private readonly byte[] _payload;
    private readonly int _bitmapOffset;
    private readonly int _wordsOffset;

    private ImuBatch(byte[] payload, int count, int accelCount)
    {
        _payload = payload;
        Count = count;
        AccelCount = accelCount;
        _bitmapOffset = HeaderSize;
        _wordsOffset = HeaderSize + (count + 7) / 8;
    }

    public ImuBatchFlags Flags => (ImuBatchFlags)_payload[5];
    public ReadOnlySpan<byte> Session => _payload.AsSpan(8, 16);
    public ulong Sequence => U64(24);
    /// <summary>Index of the first acceleration sample, or <see cref="NoIndex"/> if there is none.</summary>
    public ulong AccelIndex => U64(32);
    public ulong GyroIndex => U64(40);
    /// <summary>Sensor tick of the first sample; valid when the flag is set.</summary>
    public ulong AccelTick => U64(48);
    public ulong GyroTick => U64(56);
    public uint Epoch => BinaryPrimitives.ReadUInt32LittleEndian(_payload.AsSpan(68));
    public ulong AccelUnixMicros => U64(80);
    public ulong GyroUnixMicros => U64(88);
    /// <summary>Number of words.</summary>
    public int Count { get; }
    public int AccelCount { get; }
    public int GyroCount => Count - AccelCount;
    public ReadOnlySpan<byte> Payload => _payload;

    public bool IsAccel(int word) => (_payload[_bitmapOffset + (word >> 3)] >> (word & 7) & 1) != 0;

    public void Read(int word, out short x, out short y, out short z)
    {
        ReadOnlySpan<byte> p = _payload.AsSpan(_wordsOffset + word * 6, 6);
        x = BinaryPrimitives.ReadInt16LittleEndian(p);
        y = BinaryPrimitives.ReadInt16LittleEndian(p[2..]);
        z = BinaryPrimitives.ReadInt16LittleEndian(p[4..]);
    }

    private ulong U64(int offset) => BinaryPrimitives.ReadUInt64LittleEndian(_payload.AsSpan(offset));

    /// <summary>
    /// Checks a payload against the format. Returns null and the reason when
    /// it does not fit. The batch keeps a reference to <paramref name="payload"/>.
    /// </summary>
    public static ImuBatch? TryParse(byte[] payload, out string? error)
    {
        ReadOnlySpan<byte> p = payload;
        error = null;
        if (p.Length < HeaderSize || !p[..4].SequenceEqual("IMU1"u8))
        {
            error = "magic";
            return null;
        }
        if (p[4] != Version)
        {
            error = $"version {p[4]}";
            return null;
        }
        byte flags = p[5];
        if ((flags & ~15) != 0 || ((flags & 4) != 0 && (flags & 1) == 0) || ((flags & 8) != 0 && (flags & 2) == 0))
        {
            error = $"flags 0x{flags:x2}";
            return null;
        }
        if (BinaryPrimitives.ReadUInt16LittleEndian(p[6..]) != HeaderSize ||
            BinaryPrimitives.ReadUInt32LittleEndian(p[76..]) != 0)
        {
            error = "header";
            return null;
        }
        int count = BinaryPrimitives.ReadUInt16LittleEndian(p[64..]);
        int bitmapLength = BinaryPrimitives.ReadUInt16LittleEndian(p[66..]);
        if (count < 1 || count > MaxWords || bitmapLength != (count + 7) / 8 ||
            p.Length != HeaderSize + bitmapLength + count * 6)
        {
            error = "length";
            return null;
        }
        ReadOnlySpan<byte> bitmap = p.Slice(HeaderSize, bitmapLength);
        if (count % 8 != 0 && bitmap[^1] >> (count % 8) != 0)
        {
            error = "bitmap padding";
            return null;
        }
        int accelCount = 0;
        foreach (byte b in bitmap)
        {
            accelCount += BitOperations.PopCount(b);
        }
        ImuBatch batch = new(payload, count, accelCount);
        if ((accelCount == 0) != (batch.AccelIndex == NoIndex) ||
            (accelCount == count) != (batch.GyroIndex == NoIndex))
        {
            error = "first index";
            return null;
        }
        if (((flags & 1) == 0 && batch.AccelTick != 0) || ((flags & 2) == 0 && batch.GyroTick != 0) ||
            ((flags & 4) == 0 && batch.AccelUnixMicros != 0) || ((flags & 8) == 0 && batch.GyroUnixMicros != 0))
        {
            error = "value without its flag";
            return null;
        }
        return batch;
    }
}
