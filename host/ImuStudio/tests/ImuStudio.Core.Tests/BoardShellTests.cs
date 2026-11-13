// SPDX-License-Identifier: Apache-2.0
using System.Net;
using System.Net.Sockets;
using System.Text;
using FluentAssertions;
using ImuStudio.Core.Board;
using Xunit;

namespace ImuStudio.Core.Tests;

public class BoardShellTests
{
    private const string Prompt = "uart:~$ ";

    /// <summary>A shell that answers each line with <paramref name="reply"/>, like the board's.</summary>
    private static (TcpListener Listener, Task Server) FakeBoard(bool echo, Func<string, string> reply)
    {
        TcpListener listener = new(IPAddress.Loopback, 0);
        listener.Start();
        Task server = Task.Run(async () =>
        {
            using TcpClient client = await listener.AcceptTcpClientAsync();
            NetworkStream stream = client.GetStream();
            // Negotiation and the first prompt, as a Telnet shell sends them.
            await stream.WriteAsync(new byte[] { 0xFF, 0xFB, 0x01, 0xFF, 0xFB, 0x03 });
            await stream.WriteAsync(Encoding.ASCII.GetBytes("\r\n\x1b[1;32m" + Prompt + "\x1b[m"));
            StringBuilder line = new();
            byte[] one = new byte[1];
            while (await stream.ReadAsync(one) == 1)
            {
                if (one[0] == '\n')
                {
                    string text = line.ToString().TrimEnd('\r');
                    line.Clear();
                    string output = (echo ? text : "") + "\r\n" + reply(text) + "\x1b[1;32m" + Prompt + "\x1b[m";
                    // In pieces, as the network delivers them.
                    byte[] bytes = Encoding.ASCII.GetBytes(output);
                    for (int i = 0; i < bytes.Length; i += 7)
                    {
                        await stream.WriteAsync(bytes.AsMemory(i, Math.Min(7, bytes.Length - i)));
                        await stream.FlushAsync();
                    }
                }
                else
                {
                    line.Append((char)one[0]);
                }
            }
        });
        return (listener, server);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public async Task Returns_what_a_command_printed(bool echo)
    {
        (TcpListener listener, Task server) = FakeBoard(echo, line => line == "kv get k" ? "0102\r\n0304\r\n" : "Saved\r\n");
        try
        {
            using BoardShell shell = await BoardShell.ConnectAsync("127.0.0.1", ((IPEndPoint)listener.LocalEndpoint).Port);
            StringBuilder shown = new();
            shell.Output += text => { lock (shown) { shown.Append(text); } };
            await shell.SettleAsync();
            string first = await shell.ExecuteAsync("kv get k", TimeSpan.FromSeconds(5));
            BoardShell.OutputLines(first, "kv get k").Should().Equal("0102", "0304");
            string second = await shell.ExecuteAsync("kv set a b", TimeSpan.FromSeconds(5));
            BoardShell.OutputLines(second, "kv set a b").Should().Equal("Saved");
            lock (shown)
            {
                shown.ToString().Should().NotContain("kv get k", "the echo is left to whoever shows the line");
                shown.ToString().Should().Contain("0102\n0304\n" + Prompt);
            }
        }
        finally
        {
            listener.Stop();
            await Task.WhenAny(server, Task.Delay(1000));
        }
    }

    [Fact]
    public async Task Gives_up_after_the_wait_and_reports_the_end_of_the_connection()
    {
        TcpListener listener = new(IPAddress.Loopback, 0);
        listener.Start();
        try
        {
            Task<TcpClient> accept = listener.AcceptTcpClientAsync();
            using BoardShell shell = await BoardShell.ConnectAsync("127.0.0.1", ((IPEndPoint)listener.LocalEndpoint).Port);
            TaskCompletionSource<string?> closed = new();
            shell.Closed += reason => closed.TrySetResult(reason);
            using TcpClient peer = await accept;
            (await shell.ExecuteAsync("silent", TimeSpan.FromMilliseconds(200))).Should().BeEmpty();
            peer.Close();
            (await closed.Task.WaitAsync(TimeSpan.FromSeconds(5))).Should().NotBeNull();
        }
        finally
        {
            listener.Stop();
        }
    }

    [Fact]
    public async Task Says_when_it_cannot_connect()
    {
        TcpListener listener = new(IPAddress.Loopback, 0);
        listener.Start();
        int port = ((IPEndPoint)listener.LocalEndpoint).Port;
        listener.Stop();
        Func<Task> connect = () => BoardShell.ConnectAsync("127.0.0.1", port);
        await connect.Should().ThrowAsync<BoardException>();
    }
}
