// SPDX-License-Identifier: Apache-2.0
using System.Net;
using System.Net.Sockets;
using System.Text;

namespace ImuStudio.Core.Mqtt;

/// <summary>
/// The smallest MQTT 3.1.1 endpoint that the board can publish to. It is not
/// a broker: it accepts one client, answers CONNECT, PUBLISH (QoS 0/1),
/// SUBSCRIBE, PINGREQ and DISCONNECT, and hands each PUBLISH to the caller.
/// A new connection replaces the current one.
/// </summary>
public sealed class MqttEndpoint : IDisposable
{
    private readonly Socket _listener;
    private readonly Thread _thread;
    private readonly List<byte> _buffer = new();
    private volatile bool _stop;
    private Socket? _client;

    /// <summary>Raised on the receiving thread for every PUBLISH.</summary>
    public event Action<string, byte[]>? Published;
    /// <summary>Raised on the receiving thread when the client connects or goes away.</summary>
    public event Action? ConnectionChanged;

    public int Port { get; }
    public int Connects { get; private set; }
    public int Disconnects { get; private set; }
    public string? Peer { get; private set; }
    public bool HasClient { get; private set; }
    public string? ProtocolError { get; private set; }

    public MqttEndpoint(int port, IPAddress? bind = null)
    {
        _listener = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        try
        {
            _listener.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
            _listener.Bind(new IPEndPoint(bind ?? IPAddress.Any, port));
            _listener.Listen(1);
        }
        catch
        {
            _listener.Dispose();
            throw;
        }
        Port = ((IPEndPoint)_listener.LocalEndPoint!).Port;
        _thread = new Thread(Run) { IsBackground = true, Name = "mqtt" };
        _thread.Start();
    }

    public void Dispose()
    {
        _stop = true;
        _thread.Join();
        _client?.Dispose();
        _listener.Dispose();
    }

    private void Run()
    {
        byte[] chunk = new byte[65536];
        List<Socket> readable = new(2);
        while (!_stop)
        {
            readable.Clear();
            readable.Add(_listener);
            if (_client is not null)
            {
                readable.Add(_client);
            }
            try
            {
                Socket.Select(readable, null, null, 100_000);
            }
            catch (SocketException)
            {
                DropClient();
                continue;
            }
            if (readable.Contains(_listener))
            {
                Socket accepted = _listener.Accept();
                DropClient();
                accepted.NoDelay = true;
                _client = accepted;
                Peer = (accepted.RemoteEndPoint as IPEndPoint)?.Address.ToString();
                HasClient = true;
                ConnectionChanged?.Invoke();
            }
            if (_client is not null && readable.Contains(_client))
            {
                try
                {
                    int n = _client.Receive(chunk);
                    if (n == 0)
                    {
                        DropClient();
                        continue;
                    }
                    _buffer.AddRange(chunk.AsSpan(0, n));
                    if (!Drain())
                    {
                        DropClient();
                    }
                }
                catch (Exception e) when (e is SocketException or InvalidDataException or IndexOutOfRangeException or ArgumentException)
                {
                    ProtocolError = $"{e.GetType().Name}: {e.Message}";
                    DropClient();
                }
            }
        }
    }

    private void DropClient()
    {
        if (_client is null)
        {
            return;
        }
        _client.Dispose();
        _client = null;
        _buffer.Clear();
        Disconnects++;
        HasClient = false;
        ConnectionChanged?.Invoke();
    }

    /// <summary>Handles the complete packets in the buffer. False ends the connection.</summary>
    private bool Drain()
    {
        int pos = 0;
        bool keep = true;
        while (keep && _buffer.Count - pos >= 2)
        {
            int length = 0, shift = 0, i = pos + 1;
            bool complete = false;
            while (i < _buffer.Count && i < pos + 5)
            {
                byte b = _buffer[i++];
                length |= (b & 0x7F) << shift;
                shift += 7;
                if ((b & 0x80) == 0)
                {
                    complete = true;
                    break;
                }
            }
            if (!complete)
            {
                if (i >= pos + 5)
                {
                    throw new InvalidDataException("malformed remaining length");
                }
                break;
            }
            if (_buffer.Count - i < length)
            {
                break;
            }
            byte[] body = new byte[length];
            _buffer.CopyTo(i, body, 0, length);
            keep = Handle(_buffer[pos], body);
            pos = i + length;
        }
        _buffer.RemoveRange(0, pos);
        return keep;
    }

    private bool Handle(byte header, byte[] body)
    {
        Socket client = _client!;
        switch (header >> 4)
        {
            case 1:
                client.Send([0x20, 0x02, 0x00, 0x00]);
                Connects++;
                break;
            case 3:
            {
                int qos = (header >> 1) & 3;
                if (body.Length < 2)
                {
                    throw new InvalidDataException("short PUBLISH");
                }
                int n = body[0] << 8 | body[1];
                int offset = 2 + n;
                if (offset + (qos != 0 ? 2 : 0) > body.Length)
                {
                    throw new InvalidDataException("short PUBLISH");
                }
                string topic = Encoding.UTF8.GetString(body, 2, n);
                if (qos != 0)
                {
                    client.Send([0x40, 0x02, body[offset], body[offset + 1]]);
                    offset += 2;
                }
                Published?.Invoke(topic, body[offset..]);
                break;
            }
            case 8:
            {
                int pos = 2, count = 0;
                while (pos + 2 <= body.Length)
                {
                    pos += 2 + (body[pos] << 8 | body[pos + 1]) + 1;
                    count++;
                }
                byte[] ack = new byte[4 + count];
                ack[0] = 0x90;
                ack[1] = (byte)(2 + count);
                ack[2] = body[0];
                ack[3] = body[1];
                client.Send(ack);
                break;
            }
            case 12:
                client.Send([0xD0, 0x00]);
                break;
            case 14:
                return false;
            default:
                throw new InvalidDataException($"unexpected MQTT packet type {header >> 4}");
        }
        return true;
    }
}
