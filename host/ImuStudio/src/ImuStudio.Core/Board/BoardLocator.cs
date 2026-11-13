// SPDX-License-Identifier: Apache-2.0
using System.Diagnostics;
using System.Net.Sockets;
using System.Text.Json;

namespace ImuStudio.Core.Board;

public sealed class BoardException(string message) : Exception(message);

/// <summary>Finds the board among the stations of the access point this PC runs.</summary>
public static class BoardLocator
{
    /// <summary>Address NetworkManager gives a shared connection (script/wifi_ap.sh).</summary>
    public const string DefaultGateway = "10.42.0.1";
    public const int TelnetPort = 23;

    public static async Task<string> FindAsync(string gateway = DefaultGateway, CancellationToken ct = default)
    {
        string? ifname = null;
        using (JsonDocument addr = await IpJsonAsync("-4 addr", ct).ConfigureAwait(false))
        {
            foreach (JsonElement iface in addr.RootElement.EnumerateArray())
            {
                if (!iface.TryGetProperty("addr_info", out JsonElement infos))
                {
                    continue;
                }
                foreach (JsonElement a in infos.EnumerateArray())
                {
                    if (a.TryGetProperty("local", out JsonElement local) && local.GetString() == gateway)
                    {
                        ifname = iface.GetProperty("ifname").GetString();
                    }
                }
            }
        }
        if (ifname is null)
        {
            throw new BoardException($"No interface has {gateway}. Start the access point or enter the board address.");
        }
        List<string> found = new();
        using (JsonDocument neigh = await IpJsonAsync($"-4 neigh show dev {ifname}", ct).ConfigureAwait(false))
        {
            foreach (JsonElement n in neigh.RootElement.EnumerateArray())
            {
                if (!n.TryGetProperty("lladdr", out _) || !n.TryGetProperty("dst", out JsonElement dst))
                {
                    continue;
                }
                bool dead = n.TryGetProperty("state", out JsonElement state) &&
                            state.EnumerateArray().Any(s => s.GetString() is "FAILED" or "INCOMPLETE");
                if (!dead && dst.GetString() is string ip)
                {
                    found.Add(ip);
                }
            }
        }
        if (found.Count > 1)
        {
            List<string> answering = new();
            foreach (string ip in found)
            {
                if (await AnswersTelnetAsync(ip, ct).ConfigureAwait(false))
                {
                    answering.Add(ip);
                }
            }
            found = answering;
        }
        return found.Count switch
        {
            1 => found[0],
            0 => throw new BoardException($"No station found on {ifname}. Check the board's power and Wi-Fi, or enter its address."),
            _ => throw new BoardException($"Several stations on {ifname} answer Telnet ({string.Join(", ", found)}). Enter the address."),
        };
    }

    private static async Task<bool> AnswersTelnetAsync(string ip, CancellationToken ct)
    {
        using TcpClient client = new();
        using CancellationTokenSource timeout = CancellationTokenSource.CreateLinkedTokenSource(ct);
        timeout.CancelAfter(TimeSpan.FromSeconds(1));
        try
        {
            await client.ConnectAsync(ip, TelnetPort, timeout.Token).ConfigureAwait(false);
            return true;
        }
        catch (Exception e) when (e is SocketException or OperationCanceledException)
        {
            ct.ThrowIfCancellationRequested();
            return false;
        }
    }

    private static async Task<JsonDocument> IpJsonAsync(string arguments, CancellationToken ct)
    {
        ProcessStartInfo info = new("ip", "-j " + arguments)
        {
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
        };
        try
        {
            using Process process = Process.Start(info) ?? throw new BoardException("Cannot run ip.");
            string text = await process.StandardOutput.ReadToEndAsync(ct).ConfigureAwait(false);
            await process.WaitForExitAsync(ct).ConfigureAwait(false);
            if (process.ExitCode != 0)
            {
                throw new BoardException($"ip {arguments} failed.");
            }
            return JsonDocument.Parse(string.IsNullOrWhiteSpace(text) ? "[]" : text);
        }
        catch (Exception e) when (e is System.ComponentModel.Win32Exception or JsonException)
        {
            throw new BoardException($"Cannot list network neighbours: {e.Message}");
        }
    }
}
