// SPDX-License-Identifier: Apache-2.0
using System.Text;

namespace ImuStudio.Core.Board;

/// <summary>
/// Turns the bytes of a Telnet shell into plain text: drops Telnet commands,
/// ANSI escape sequences and carriage returns. Sequences may be split across
/// calls.
/// </summary>
public sealed class TerminalFilter
{
    private enum State { Text, Iac, IacOption, Subnegotiation, SubnegotiationIac, Escape, Csi }

    private readonly Decoder _decoder = Encoding.UTF8.GetDecoder();
    private readonly byte[] _one = new byte[1];
    private readonly char[] _chars = new char[4];
    private State _state;

    public string Feed(ReadOnlySpan<byte> data)
    {
        StringBuilder text = new(data.Length);
        foreach (byte b in data)
        {
            switch (_state)
            {
                case State.Text:
                    if (b == 0xFF)
                    {
                        _state = State.Iac;
                    }
                    else if (b == 0x1B)
                    {
                        _state = State.Escape;
                    }
                    else if (b != (byte)'\r' && b != 0)
                    {
                        Emit(b, text);
                    }
                    break;
                case State.Iac:
                    _state = b switch
                    {
                        >= 0xFB and <= 0xFE => State.IacOption,
                        0xFA => State.Subnegotiation,
                        _ => State.Text,
                    };
                    break;
                case State.IacOption:
                    _state = State.Text;
                    break;
                case State.Subnegotiation:
                    if (b == 0xFF)
                    {
                        _state = State.SubnegotiationIac;
                    }
                    break;
                case State.SubnegotiationIac:
                    _state = b == 0xF0 ? State.Text : State.Subnegotiation;
                    break;
                case State.Escape:
                    _state = b == (byte)'[' ? State.Csi : State.Text;
                    break;
                case State.Csi:
                    // Parameters and intermediates are 0x20-0x3F; the final byte ends it.
                    if (b is >= 0x40 and <= 0x7E)
                    {
                        _state = State.Text;
                    }
                    break;
            }
        }
        return text.ToString();
    }

    private void Emit(byte b, StringBuilder text)
    {
        _one[0] = b;
        int n = _decoder.GetChars(_one, 0, 1, _chars, 0, flush: false);
        text.Append(_chars, 0, n);
    }
}
