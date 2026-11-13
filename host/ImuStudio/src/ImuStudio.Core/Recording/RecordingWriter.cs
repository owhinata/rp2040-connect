// SPDX-License-Identifier: Apache-2.0
using System.Buffers.Binary;
using System.Text;
using System.Text.Json;

namespace ImuStudio.Core.Recording;

/// <summary>
/// Writes a recording in the layout of <c>imu-host record</c>, so either tool
/// can read what the other wrote. A recording is a directory that holds what
/// was received, unprocessed:
/// <list type="bullet">
/// <item><c>raw.bin</c>: the RAW payloads, each preceded by its length (u32le)</item>
/// <item><c>messages.jsonl</c>: info, status and other messages, one per line</item>
/// <item><c>meta.json</c>: start and end time, counters, and the calibration in use</item>
/// </list>
/// </summary>
public sealed class RecordingWriter : IDisposable
{
    private readonly object _lock = new();
    private readonly FileStream _raw;
    private readonly StreamWriter _messages;
    private readonly double _started = UnixNow();
    private bool _closed;

    public string Path { get; }
    public long RawPayloads { get; private set; }

    public RecordingWriter(string path)
    {
        if (Directory.Exists(path) || File.Exists(path))
        {
            throw new IOException($"{path} already exists");
        }
        Directory.CreateDirectory(path);
        Path = path;
        _raw = new FileStream(System.IO.Path.Combine(path, "raw.bin"), FileMode.CreateNew, FileAccess.Write,
                              FileShare.Read, 1 << 16);
        _messages = new StreamWriter(System.IO.Path.Combine(path, "messages.jsonl"), false, new UTF8Encoding(false));
    }

    private static double UnixNow() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

    public void Add(string topic, byte[] payload)
    {
        lock (_lock)
        {
            if (_closed)
            {
                return;
            }
            if (topic.EndsWith("/raw", StringComparison.Ordinal))
            {
                Span<byte> length = stackalloc byte[4];
                BinaryPrimitives.WriteUInt32LittleEndian(length, (uint)payload.Length);
                _raw.Write(length);
                _raw.Write(payload);
                RawPayloads++;
                return;
            }
            using MemoryStream line = new();
            using (Utf8JsonWriter json = new(line))
            {
                json.WriteStartObject();
                json.WriteNumber("t", UnixNow());
                json.WriteString("topic", topic);
                json.WritePropertyName("payload");
                try
                {
                    using JsonDocument body = JsonDocument.Parse(payload);
                    body.RootElement.WriteTo(json);
                }
                catch (JsonException)
                {
                    json.WriteStringValue(Encoding.UTF8.GetString(payload));
                }
                json.WriteEndObject();
            }
            _messages.WriteLine(Encoding.UTF8.GetString(line.GetBuffer(), 0, (int)line.Length));
        }
    }

    /// <summary>Closes the files and writes <c>meta.json</c>.</summary>
    /// <param name="summary">Counters to keep with the recording.</param>
    /// <param name="calibration">KV key to value of the calibration the board had.</param>
    public void Close(IReadOnlyDictionary<string, object?> summary, IReadOnlyDictionary<string, byte[]> calibration)
    {
        lock (_lock)
        {
            if (_closed)
            {
                return;
            }
            _closed = true;
            _raw.Dispose();
            _messages.Dispose();
            using FileStream meta = File.Create(System.IO.Path.Combine(Path, "meta.json"));
            using (Utf8JsonWriter json = new(meta, new JsonWriterOptions { Indented = true }))
            {
                json.WriteStartObject();
                json.WriteNumber("started", _started);
                json.WriteNumber("ended", UnixNow());
                json.WritePropertyName("summary");
                JsonSerializer.Serialize(json, summary);
                json.WriteStartObject("calibration");
                foreach ((string key, byte[] blob) in calibration.OrderBy(p => p.Key, StringComparer.Ordinal))
                {
                    json.WriteString(key, Convert.ToHexString(blob).ToLowerInvariant());
                }
                json.WriteEndObject();
                json.WriteEndObject();
            }
            meta.WriteByte((byte)'\n');
        }
    }

    public void Dispose()
    {
        lock (_lock)
        {
            if (!_closed)
            {
                _closed = true;
                _raw.Dispose();
                _messages.Dispose();
            }
        }
    }
}
