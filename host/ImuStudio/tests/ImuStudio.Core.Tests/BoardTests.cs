// SPDX-License-Identifier: Apache-2.0
using FluentAssertions;
using ImuStudio.Core.Board;
using Xunit;

namespace ImuStudio.Core.Tests;

public class BoardTests
{
    [Fact]
    public void Filter_drops_telnet_commands_and_escape_sequences()
    {
        TerminalFilter filter = new();
        byte[] data = [0xFF, 0xFB, 0x01, (byte)'a', 0x1B, (byte)'[', (byte)'1', (byte)';', (byte)'3', (byte)'2',
                       (byte)'m', (byte)'b', (byte)'\r', (byte)'\n', 0xFF, 0xFA, 0x18, 0x01, 0xFF, 0xF0, (byte)'c'];
        filter.Feed(data).Should().Be("ab\nc");
    }

    [Fact]
    public void Filter_keeps_its_state_across_calls()
    {
        TerminalFilter filter = new();
        string text = filter.Feed([(byte)'x', 0xFF]) + filter.Feed([0xFD]) + filter.Feed([0x03, 0x1B]) +
                      filter.Feed([(byte)'[']) + filter.Feed([(byte)'m', (byte)'y']);
        text.Should().Be("xy");
    }

    [Fact]
    public void Output_lines_leave_out_the_echo_and_the_prompt()
    {
        string captured = "kv get imu.cal.accel.244\n0100000000f40000\nabcdef\nuart:~$ ";
        BoardShell.OutputLines(captured, "kv get imu.cal.accel.244").Should().Equal("0100000000f40000", "abcdef");
    }
}
