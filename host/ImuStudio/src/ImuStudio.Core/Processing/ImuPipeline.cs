// SPDX-License-Identifier: Apache-2.0
using System.Numerics;
using System.Text.Json;
using ImuStudio.Core.Calibration;
using ImuStudio.Core.Wire;

namespace ImuStudio.Core.Processing;

/// <summary>Counters and latest values of the pipeline, copied at one instant.</summary>
public sealed record PipelineSnapshot(
    string? Session,
    StreamInfo? Info,
    string? BoardState,
    long Batches,
    long MissingBatches,
    long InvalidPayloads,
    long EpochChanges,
    long IndexJumps,
    long AccelSamples,
    long GyroSamples,
    long BatchesWithUnixTime,
    bool AccelCalibrated,
    bool GyroCalibrated,
    Vector3 Accel,
    Vector3 Gyro,
    Vector3 GyroBiasDps,
    uint BiasUpdates,
    uint BiasRejected,
    double SamplePeriod,
    bool TickRateMeasured);

/// <summary>
/// Takes the board's MQTT messages and turns the RAW words into physical
/// samples for the sink in use: checks the batches, converts with the
/// calibration that matches the announced sensitivity, tracks the gyro bias
/// at rest and derives the sample period from the sensor's ticks.
///
/// Messages may come from the receiving thread while other threads change
/// the sink or the calibration; one lock serialises them. Sink methods run
/// under that lock on the thread that delivers the message.
/// </summary>
public sealed class ImuPipeline
{
    private readonly object _lock = new();
    private readonly Dictionary<string, byte[]> _stored = new();
    private ISampleSink? _sink;
    private StreamInfo? _info;
    private byte[]? _session;
    private string? _boardState;
    private SensorCalibration? _accelCal, _gyroCal;
    private GyroZeroTracker? _zero;
    private SampleClock? _clock;
    private int[]? _gyroBias;
    private ulong _nextSequence, _nextAccel, _nextGyro;
    private uint _epoch;
    private bool _haveBatch;
    private long _batches, _missing, _invalid, _epochChanges, _indexJumps, _accelSamples, _gyroSamples, _unixBatches;
    private Vector3 _accel, _gyro;

    /// <summary>Raised under the lock when a run announces itself; handlers must not block.</summary>
    public event Action<StreamInfo>? InfoReceived;

    public ISampleSink? Sink
    {
        get { lock (_lock) { return _sink; } }
        set
        {
            lock (_lock)
            {
                _sink = value;
                _sink?.Reset();
            }
        }
    }

    /// <summary>Makes a value of the board's KV store available; used when its sensitivity is announced.</summary>
    public void SetStored(string key, byte[]? blob)
    {
        lock (_lock)
        {
            if (blob is null)
            {
                _stored.Remove(key);
            }
            else
            {
                _stored[key] = blob;
            }
            if (_info is not null)
            {
                SelectCalibration();
            }
        }
    }

    public IReadOnlyDictionary<string, byte[]> Stored
    {
        get { lock (_lock) { return new Dictionary<string, byte[]>(_stored); } }
    }

    /// <summary>Forgets the run; the stored calibrations are kept.</summary>
    public void Clear()
    {
        lock (_lock)
        {
            _info = null;
            _session = null;
            _boardState = null;
            StartRun();
        }
    }

    public PipelineSnapshot Snapshot()
    {
        lock (_lock)
        {
            Vector3 bias = default;
            if (_gyroCal is not null && _gyroBias is not null)
            {
                float dpsPerLsb = _gyroCal.Gain * 1e-6f / (1 << SensorCalibration.BiasFraction);
                bias = new Vector3(_gyroBias[0], _gyroBias[1], _gyroBias[2]) * dpsPerLsb;
            }
            return new PipelineSnapshot(
                _session is null ? null : Convert.ToHexString(_session).ToLowerInvariant(),
                _info, _boardState, _batches, _missing, _invalid, _epochChanges, _indexJumps,
                _accelSamples, _gyroSamples, _unixBatches,
                _accelCal?.IsStored ?? false, _gyroCal?.IsStored ?? false,
                _accel, _gyro, bias, _zero?.Updates ?? 0, _zero?.Rejected ?? 0,
                _clock?.Period ?? 0, _clock?.TickRateMeasured ?? false);
        }
    }

    public void OnMessage(string topic, byte[] payload)
    {
        lock (_lock)
        {
            if (topic.EndsWith("/raw", StringComparison.Ordinal))
            {
                OnRaw(payload);
            }
            else if (topic.EndsWith("/info", StringComparison.Ordinal))
            {
                // The board repeats the info of a run after it reconnects.
                if (StreamInfo.TryParse(payload) is StreamInfo info && info != _info)
                {
                    _info = info;
                    _session = null;
                    StartRun();
                    InfoReceived?.Invoke(info);
                }
            }
            else if (topic.EndsWith("/status", StringComparison.Ordinal))
            {
                OnStatus(payload);
            }
        }
    }

    private void StartRun()
    {
        _haveBatch = false;
        _batches = _missing = _invalid = _epochChanges = _indexJumps = _accelSamples = _gyroSamples = _unixBatches = 0;
        _accel = _gyro = default;
        _clock = _info is null ? null : new SampleClock(_info.GyroOdrHz, _info.TickHzNominal, _info.FreqFine);
        SelectCalibration();
        _sink?.Reset();
    }

    private void SelectCalibration()
    {
        if (_info is null)
        {
            _accelCal = _gyroCal = null;
            _zero = null;
            _gyroBias = null;
            return;
        }
        _accelCal = Select(SensorKind.Accel, _info.AccelMicroGPerLsb);
        SensorCalibration gyro = Select(SensorKind.Gyro, _info.GyroMicroDpsPerLsb);
        // As on the board, the tracked bias starts over when the stored calibration changes.
        if (_gyroCal is null || _zero is null || gyro.IsStored != _gyroCal.IsStored || gyro.Gain != _gyroCal.Gain ||
            !gyro.Bias.AsSpan().SequenceEqual(_gyroCal.Bias))
        {
            _zero = new GyroZeroTracker(gyro.Gain, _accelCal.Gain, gyro.IsStored ? gyro.Bias : default,
                                        (uint)Math.Max(0, _info.GyroOdrHz));
            _gyroBias = (int[])gyro.Bias.Clone();
        }
        _gyroCal = gyro;
    }

    private SensorCalibration Select(SensorKind kind, uint gain)
    {
        if (_stored.TryGetValue(SensorCalibration.Key(kind, gain), out byte[]? blob) &&
            SensorCalibration.TryDecode(blob, kind, gain) is SensorCalibration stored)
        {
            return stored;
        }
        return SensorCalibration.Nominal(kind, gain);
    }

    private void OnStatus(byte[] payload)
    {
        try
        {
            using JsonDocument doc = JsonDocument.Parse(payload);
            JsonElement r = doc.RootElement;
            if (r.TryGetProperty("state", out JsonElement state))
            {
                _boardState = state.GetString();
            }
            if (_clock is not null && _info is not null &&
                r.TryGetProperty("session", out JsonElement session) && session.GetString() == _info.Session &&
                r.TryGetProperty("tick", out JsonElement tick) && r.TryGetProperty("tick_mono_us", out JsonElement mono))
            {
                _clock.Status(tick.GetUInt64(), mono.GetUInt64());
            }
        }
        catch (Exception e) when (e is JsonException or InvalidOperationException or FormatException)
        {
        }
    }

    private void OnRaw(byte[] payload)
    {
        ImuBatch? batch = ImuBatch.TryParse(payload, out _);
        if (batch is null)
        {
            _invalid++;
            return;
        }
        // Without the info of this run the sensitivity is unknown.
        if (_info is null || _accelCal is null || _gyroCal is null || _zero is null || _clock is null || _gyroBias is null)
        {
            return;
        }
        if (_session is null || !batch.Session.SequenceEqual(_session))
        {
            if (!string.Equals(Convert.ToHexString(batch.Session), _info.Session, StringComparison.OrdinalIgnoreCase))
            {
                return; // a run whose info has not been seen
            }
            _session = batch.Session.ToArray();
        }
        bool discontinuous = false;
        if (_haveBatch)
        {
            if (batch.Sequence != _nextSequence)
            {
                if (batch.Sequence > _nextSequence)
                {
                    _missing += (long)(batch.Sequence - _nextSequence);
                }
                discontinuous = true;
            }
            bool epochChanged = batch.Epoch != _epoch;
            if (epochChanged)
            {
                // The board lost an unknown number of samples and numbers them anew:
                // sample numbers before and after no longer measure the same time.
                _epochChanges++;
                _clock = new SampleClock(_info.GyroOdrHz, _info.TickHzNominal, _info.FreqFine);
                discontinuous = true;
            }
            // Sample numbers that jump although no batch is missing.
            if (!discontinuous && ((batch.AccelCount > 0 && batch.AccelIndex != _nextAccel) ||
                                   (batch.GyroCount > 0 && batch.GyroIndex != _nextGyro)))
            {
                _indexJumps++;
                discontinuous = true;
            }
        }
        if (discontinuous)
        {
            // Across missing batches the sample numbers and ticks still line up, as
            // the board keeps counting, but the sink must know that time has passed.
            _sink?.OnGap();
        }
        _haveBatch = true;
        _nextSequence = batch.Sequence + 1;
        _epoch = batch.Epoch;
        if (batch.AccelCount > 0)
        {
            _nextAccel = batch.AccelIndex + (ulong)batch.AccelCount;
        }
        if (batch.GyroCount > 0)
        {
            _nextGyro = batch.GyroIndex + (ulong)batch.GyroCount;
            if ((batch.Flags & ImuBatchFlags.GyroTick) != 0)
            {
                _clock.Anchor(batch.GyroIndex, batch.GyroTick);
            }
        }
        if ((batch.Flags & (ImuBatchFlags.AccelUnix | ImuBatchFlags.GyroUnix)) != 0)
        {
            _unixBatches++;
        }
        _batches++;
        _accelSamples += batch.AccelCount;
        _gyroSamples += batch.GyroCount;

        double dt = _clock.Period;
        ISampleSink? sink = _sink;
        for (int i = 0; i < batch.Count; i++)
        {
            batch.Read(i, out short x, out short y, out short z);
            if (batch.IsAccel(i))
            {
                _zero.Accel(x, y, z);
                _accel = _accelCal.Apply(x, y, z);
                sink?.OnAccel(_accel);
            }
            else
            {
                if (_zero.Gyro(x, y, z))
                {
                    _zero.Bias.CopyTo(_gyroBias, 0);
                }
                _gyro = _gyroCal.Apply(x, y, z, _gyroBias);
                sink?.OnGyro(_gyro, dt);
            }
        }
    }
}
