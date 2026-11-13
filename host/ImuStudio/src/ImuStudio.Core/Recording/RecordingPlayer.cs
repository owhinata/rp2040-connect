// SPDX-License-Identifier: Apache-2.0
using System.Buffers.Binary;
using System.Diagnostics;
using System.Text;
using System.Text.Json;
using ImuStudio.Core.Wire;

namespace ImuStudio.Core.Recording;

/// <summary>
/// Replays a recording as the messages the board sent, at the pace the
/// samples were taken. The receiver cannot tell it from a live board, apart
/// from the topic names. Playing can be paused and moved to another time.
/// </summary>
public sealed class RecordingPlayer : IDisposable
{
    private const string Topic = "imu/replay/";

    private readonly string _path;
    private readonly List<(string Session, int OdrHz, byte[] Payload)> _infos = new();
    private readonly List<(ulong Tick, string Session, byte[] Payload)> _statuses = new();
    private readonly List<long> _offsets = new();     // of each payload's length prefix in raw.bin
    private readonly List<double> _times = new();     // seconds from the start to each payload
    private readonly CancellationTokenSource _stop = new();
    private readonly object _lock = new();
    private Thread? _thread;
    private int _seekTo = -1;
    private bool _paused;
    private double _position;

    /// <summary>Calibration saved with the recording: KV key to value. Empty for recordings made without it.</summary>
    public IReadOnlyDictionary<string, byte[]> Calibration { get; }
    /// <summary>Length in seconds, by the number of samples.</summary>
    public double Duration { get; }
    /// <summary>Seconds from the start to what is played now.</summary>
    public double Position
    {
        get { lock (_lock) { return _position; } }
    }

    public bool IsPaused
    {
        get { lock (_lock) { return _paused; } }
        set { lock (_lock) { _paused = value; } }
    }

    /// <summary>Raised on the playing thread for each message.</summary>
    public event Action<string, byte[]>? Published;
    /// <summary>
    /// Raised on the playing thread before the first message and after each
    /// <see cref="Seek"/>: what follows is not continuous with what was played.
    /// </summary>
    public event Action? Restarted;
    /// <summary>Raised on the playing thread when the end is reached or playing is stopped.</summary>
    public event Action? Finished;

    public RecordingPlayer(string path)
    {
        _path = path;
        string raw = Path.Combine(path, "raw.bin");
        if (!File.Exists(raw))
        {
            throw new FileNotFoundException($"{path} is not a recording: raw.bin is missing");
        }
        Calibration = ReadCalibration(Path.Combine(path, "meta.json"));
        string messages = Path.Combine(path, "messages.jsonl");
        if (File.Exists(messages))
        {
            ReadMessages(messages);
        }
        Duration = Index(raw);
    }

    private static Dictionary<string, byte[]> ReadCalibration(string metaPath)
    {
        Dictionary<string, byte[]> result = new();
        try
        {
            using JsonDocument doc = JsonDocument.Parse(File.ReadAllBytes(metaPath));
            if (doc.RootElement.TryGetProperty("calibration", out JsonElement cal) &&
                cal.ValueKind == JsonValueKind.Object)
            {
                foreach (JsonProperty p in cal.EnumerateObject())
                {
                    if (p.Value.GetString() is string hex)
                    {
                        result[p.Name] = Convert.FromHexString(hex);
                    }
                }
            }
        }
        catch (Exception e) when (e is IOException or JsonException or FormatException or InvalidOperationException)
        {
        }
        return result;
    }

    private void ReadMessages(string path)
    {
        foreach (string line in File.ReadLines(path))
        {
            try
            {
                using JsonDocument doc = JsonDocument.Parse(line);
                JsonElement r = doc.RootElement;
                string topic = r.GetProperty("topic").GetString() ?? "";
                JsonElement payload = r.GetProperty("payload");
                if (payload.ValueKind != JsonValueKind.Object)
                {
                    continue;
                }
                byte[] bytes = Encoding.UTF8.GetBytes(payload.GetRawText());
                if (topic.EndsWith("/info", StringComparison.Ordinal))
                {
                    if (StreamInfo.TryParse(bytes) is StreamInfo info)
                    {
                        _infos.Add((info.Session, Math.Max(1, info.GyroOdrHz), bytes));
                    }
                }
                else if (topic.EndsWith("/status", StringComparison.Ordinal) &&
                         payload.TryGetProperty("tick", out JsonElement tick) &&
                         payload.TryGetProperty("session", out JsonElement session))
                {
                    _statuses.Add((tick.GetUInt64(), session.GetString() ?? "", bytes));
                }
            }
            catch (Exception e) when (e is JsonException or KeyNotFoundException or InvalidOperationException or FormatException)
            {
            }
        }
    }

    private int OdrOf(string session)
    {
        foreach ((string infoSession, int odr, _) in _infos)
        {
            if (infoSession == session)
            {
                return odr;
            }
        }
        return 6667;
    }

    /// <summary>Notes where each payload is and when it plays. Returns the total time.</summary>
    private double Index(string rawPath)
    {
        using FileStream raw = new(rawPath, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 16);
        byte[] header = new byte[4];
        double time = 0;
        string? session = null;
        int odr = 6667;
        while (raw.Read(header) == 4)
        {
            int n = (int)BinaryPrimitives.ReadUInt32LittleEndian(header);
            if (n <= 0 || n > 1 << 20)
            {
                break;
            }
            byte[] payload = new byte[n];
            if (raw.Read(payload) != n)
            {
                break;
            }
            _offsets.Add(raw.Position - n - 4);
            _times.Add(time);
            if (ImuBatch.TryParse(payload, out _) is ImuBatch batch)
            {
                string batchSession = Convert.ToHexString(batch.Session).ToLowerInvariant();
                if (batchSession != session)
                {
                    session = batchSession;
                    odr = OdrOf(session);
                }
                time += batch.GyroCount / (double)odr;
            }
        }
        return time;
    }

    public void Start()
    {
        if (_thread is not null)
        {
            throw new InvalidOperationException("already started");
        }
        _thread = new Thread(Run) { IsBackground = true, Name = "replay" };
        _thread.Start();
    }

    /// <summary>
    /// Continues from the payload at <paramref name="seconds"/>. While paused,
    /// that one payload is played so that the receiver shows the new place.
    /// </summary>
    public void Seek(double seconds)
    {
        if (_times.Count == 0)
        {
            return;
        }
        int index = _times.BinarySearch(Math.Clamp(seconds, 0, Duration));
        if (index < 0)
        {
            index = Math.Max(0, ~index - 1);
        }
        lock (_lock)
        {
            _seekTo = Math.Min(index, _times.Count - 1);
        }
    }

    private void Run()
    {
        try
        {
            Play();
        }
        catch (IOException)
        {
        }
        Finished?.Invoke();
    }

    private void Play()
    {
        using FileStream raw = new(Path.Combine(_path, "raw.bin"), FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 16);
        byte[] header = new byte[4];
        string? session = null;
        int odr = 6667;
        int nextStatus = 0;
        int index = 0;
        bool restart = true, wasPaused = false;
        Stopwatch watch = Stopwatch.StartNew();
        double due = 0;
        while (!_stop.IsCancellationRequested && index < _offsets.Count)
        {
            bool paused;
            lock (_lock)
            {
                if (_seekTo >= 0)
                {
                    index = _seekTo;
                    _seekTo = -1;
                    restart = true;
                }
                paused = _paused;
            }
            if (paused && !restart)
            {
                wasPaused = true;
                _stop.Token.WaitHandle.WaitOne(20);
                continue;
            }
            if (restart || wasPaused)
            {
                // The pace starts anew from here.
                watch.Restart();
                due = 0;
                wasPaused = false;
            }
            raw.Position = _offsets[index];
            if (raw.Read(header) != 4)
            {
                break;
            }
            byte[] payload = new byte[BinaryPrimitives.ReadUInt32LittleEndian(header)];
            if (raw.Read(payload) != payload.Length)
            {
                break;
            }
            ImuBatch? batch = ImuBatch.TryParse(payload, out _);
            if (restart)
            {
                restart = false;
                session = null;
                Restarted?.Invoke();
            }
            if (batch is not null)
            {
                string batchSession = Convert.ToHexString(batch.Session).ToLowerInvariant();
                if (batchSession != session)
                {
                    // The receiver handles one run: announce the one this data belongs to.
                    session = batchSession;
                    odr = OdrOf(session);
                    foreach ((string infoSession, _, byte[] info) in _infos)
                    {
                        if (infoSession == session)
                        {
                            Published?.Invoke(Topic + "info", info);
                        }
                    }
                    // Statuses from before this place are not played again.
                    nextStatus = 0;
                    if ((batch.Flags & ImuBatchFlags.GyroTick) != 0)
                    {
                        while (nextStatus < _statuses.Count &&
                               (_statuses[nextStatus].Session != session || _statuses[nextStatus].Tick <= batch.GyroTick))
                        {
                            nextStatus++;
                        }
                    }
                }
                else if ((batch.Flags & ImuBatchFlags.GyroTick) != 0)
                {
                    for (; nextStatus < _statuses.Count; nextStatus++)
                    {
                        (ulong tick, string statusSession, byte[] status) = _statuses[nextStatus];
                        if (statusSession != session)
                        {
                            continue;
                        }
                        if (tick > batch.GyroTick)
                        {
                            break;
                        }
                        Published?.Invoke(Topic + "status", status);
                    }
                }
            }
            Published?.Invoke(Topic + "raw", payload);
            index++;
            lock (_lock)
            {
                _position = index < _times.Count ? _times[index] : Duration;
            }
            due += (batch?.GyroCount ?? 0) / (double)odr;
            double ahead = due - watch.Elapsed.TotalSeconds;
            if (ahead > 0.002)
            {
                _stop.Token.WaitHandle.WaitOne(TimeSpan.FromSeconds(ahead));
            }
        }
    }

    public void Dispose()
    {
        _stop.Cancel();
        if (_thread is not null && _thread != Thread.CurrentThread)
        {
            _thread.Join();
        }
        _stop.Dispose();
    }
}
