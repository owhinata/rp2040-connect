// SPDX-License-Identifier: Apache-2.0
using FluentAssertions;
using ImuStudio.Core.Wire;
using Xunit;

namespace ImuStudio.Core.Tests;

public class WireTests
{
    [Fact]
    public void Parses_a_batch()
    {
        byte[] payload = TestData.Batch(7, 100, 3, (1, 2, 3), (-4, -5, -6), tick: 600);
        ImuBatch? batch = ImuBatch.TryParse(payload, out string? error);
        error.Should().BeNull();
        batch!.Sequence.Should().Be(7);
        batch.Count.Should().Be(6);
        batch.AccelCount.Should().Be(3);
        batch.GyroIndex.Should().Be(100);
        batch.GyroTick.Should().Be(600);
        batch.Flags.Should().Be(ImuBatchFlags.AccelTick | ImuBatchFlags.GyroTick);
        batch.IsAccel(0).Should().BeTrue();
        batch.IsAccel(1).Should().BeFalse();
        batch.Read(1, out short x, out short y, out short z);
        (x, y, z).Should().Be(((short)-4, (short)-5, (short)-6));
    }

    [Fact]
    public void Rejects_what_does_not_fit()
    {
        byte[] good = TestData.Batch(0, 0, 2, (0, 0, 0), (0, 0, 0));
        Check(p => p[0] = (byte)'X', "magic");
        Check(p => p[4] = 1, "version 1");
        Check(p => p[5] = 0x10, "flags 0x10");
        Check(p => p[5] = 4, "flags 0x04");
        Check(p => p[76] = 1, "header");
        Check(p => p[64] = 3, "length");
        Check(p => p[96] |= 0x10, "bitmap padding");
        Check(p => p[48] = 1, "value without its flag");
        ImuBatch.TryParse(good[..^1], out string? error).Should().BeNull();
        error.Should().Be("length");

        void Check(Action<byte[]> damage, string expected)
        {
            byte[] p = (byte[])good.Clone();
            damage(p);
            ImuBatch.TryParse(p, out string? e).Should().BeNull();
            e.Should().Be(expected);
        }
    }

    [Fact]
    public void Parses_info()
    {
        StreamInfo? info = StreamInfo.TryParse(TestData.InfoBytes());
        info!.Session.Should().Be(TestData.Session);
        info.GyroOdrHz.Should().Be(6667);
        info.AccelMicroGPerLsb.Should().Be(244);
        info.GyroMicroDpsPerLsb.Should().Be(70000);
        StreamInfo.TryParse("{}"u8).Should().BeNull();
        StreamInfo.TryParse("nonsense"u8).Should().BeNull();
    }
}
