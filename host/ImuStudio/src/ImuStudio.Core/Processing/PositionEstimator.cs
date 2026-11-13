// SPDX-License-Identifier: Apache-2.0
using System.Diagnostics;
using System.Numerics;

namespace ImuStudio.Core.Processing;

/// <summary>Where the estimate stands, copied at one instant.</summary>
public sealed record PositionSnapshot(
    Vector3 Position,
    Vector3 VelocityBody,
    Vector3 VelocityWorld,
    Quaternion Orientation,
    double Distance,
    double Seconds,
    long Estimates,
    long Skipped,
    double InferenceMilliseconds,
    double EstimatesPerSecond,
    int Windows,
    string? Error);

/// <summary>
/// Estimates the position by dead reckoning on a learned velocity.
///
/// The samples are reduced to 200 Hz (each value the mean of 5 ms), which is
/// the rate the model's training data had. About sixty times a second the
/// last one to ten seconds go to the model, laid out as it was trained: one
/// second windows, every fifth sample. The model answers with the mean
/// velocity over the last second in the sensor's frame. A Madgwick filter
/// run on every sample gives the attitude that turns it into the world
/// frame, and the position is the sum of velocity × time.
///
/// The model runs on its own thread, so receiving is never held up by it. If
/// it cannot keep up, requests are skipped and the time they covered is
/// still accounted for at the next one.
/// </summary>
public sealed class PositionEstimator : ISampleSink, IDisposable
{
    private const double SlotSeconds = 1.0 / 200;
    private const int SlotsPerWindow = 200;
    private const int Decimation = SlotsPerWindow / IVelocityModel.WindowSamples;
    private const int RingSlots = SlotsPerWindow * IVelocityModel.MaxWindows;
    private const double EstimateSeconds = 1.0 / 60;
    private const int TrackLimit = 36_000;       // an hour at ten points a second
    private const double TrackSeconds = 0.1;

    private readonly IVelocityModel _model;
    private readonly AttitudeEstimator _attitude = new();
    private readonly float[][] _ring = new float[IVelocityModel.Rows][];
    private readonly Thread _worker;
    private readonly object _lock = new();
    private readonly SemaphoreSlim _pending = new(0);
    private readonly List<Vector3> _track = new();
    private volatile bool _stop;

    // Receiving side.
    private Vector3 _gyroSum, _accelSum;
    private int _gyroCount, _accelCount;
    private double _slotTime, _sinceEstimate, _time;
    private long _slots;
    private int _filled;

    // Handed to the worker: the newest request replaces one not yet taken.
    private float[]? _request;
    private int _requestWindows;
    private Quaternion _requestOrientation;
    private double _requestTime;

    // Result, under _lock.
    private Vector3 _position, _velocityBody, _velocityWorld;
    private Quaternion _orientation = Quaternion.Identity;
    private double _distance, _lastTime = double.NaN, _lastTrackTime, _inferenceMs, _rate;
    private long _estimates, _skipped;
    private int _windows;
    private int _generation;
    private string? _error;

    public PositionEstimator(IVelocityModel model)
    {
        _model = model;
        for (int row = 0; row < _ring.Length; row++)
        {
            _ring[row] = new float[RingSlots];
        }
        _worker = new Thread(Work) { IsBackground = true, Name = "position" };
        _worker.Start();
    }

    /// <summary>Gain of the attitude filter; see <see cref="AttitudeEstimator.Beta"/>.</summary>
    public float Beta
    {
        get => _attitude.Beta;
        set => _attitude.Beta = value;
    }

    public PositionSnapshot Snapshot()
    {
        lock (_lock)
        {
            return new PositionSnapshot(_position, _velocityBody, _velocityWorld, _orientation, _distance,
                                        double.IsNaN(_lastTime) ? 0 : _lastTime, _estimates, _skipped, _inferenceMs,
                                        _rate, _windows, _error);
        }
    }

    /// <summary>Copies the path so far, about ten points a second, oldest first.</summary>
    public Vector3[] Track()
    {
        lock (_lock)
        {
            return _track.ToArray();
        }
    }

    /// <summary>Back to the origin, with the attitude and everything learned from the past dropped.</summary>
    public void Reset()
    {
        _attitude.Reset();
        ClearInput();
        _time = 0;
        lock (_lock)
        {
            _generation++;
            _request = null;
            _position = _velocityBody = _velocityWorld = Vector3.Zero;
            _orientation = Quaternion.Identity;
            _distance = 0;
            _lastTime = double.NaN;
            _lastTrackTime = 0;
            _estimates = _skipped = 0;
            _windows = 0;
            _rate = 0;
            _track.Clear();
            _track.Add(Vector3.Zero);
        }
    }

    /// <summary>
    /// The windows must be continuous: they fill again from here. The position
    /// and the attitude stay; nothing is added for the time without windows.
    /// </summary>
    public void OnGap()
    {
        _attitude.OnGap();
        ClearInput();
        lock (_lock)
        {
            _request = null;
            _lastTime = double.NaN;
        }
    }

    private void ClearInput()
    {
        _gyroSum = _accelSum = Vector3.Zero;
        _gyroCount = _accelCount = 0;
        _slotTime = _sinceEstimate = 0;
        _slots = 0;
        _filled = 0;
    }

    public void OnAccel(Vector3 metersPerSecond2)
    {
        _attitude.OnAccel(metersPerSecond2);
        _accelSum += metersPerSecond2;
        _accelCount++;
    }

    public void OnGyro(Vector3 radiansPerSecond, double dt)
    {
        _attitude.OnGyro(radiansPerSecond, dt);
        _gyroSum += radiansPerSecond;
        _gyroCount++;
        _time += dt;
        _slotTime += dt;
        if (_slotTime < SlotSeconds)
        {
            return;
        }
        _slotTime -= SlotSeconds;
        if (_accelCount == 0)
        {
            return; // nothing to pair the angular rate with yet
        }
        Vector3 gyro = _gyroSum / _gyroCount;
        Vector3 accel = _accelSum / _accelCount;
        _gyroSum = _accelSum = Vector3.Zero;
        _gyroCount = _accelCount = 0;
        int at = (int)(_slots % RingSlots);
        _ring[0][at] = gyro.X;
        _ring[1][at] = gyro.Y;
        _ring[2][at] = gyro.Z;
        _ring[3][at] = accel.X;
        _ring[4][at] = accel.Y;
        _ring[5][at] = accel.Z;
        _slots++;
        _filled = Math.Min(RingSlots, _filled + 1);

        // Capped, so that the wait for the first window is not made up for by a burst.
        _sinceEstimate = Math.Min(_sinceEstimate + SlotSeconds, 2 * EstimateSeconds);
        int windows = _filled / SlotsPerWindow;
        if (windows == 0 || _sinceEstimate < EstimateSeconds - 1e-9)
        {
            return;
        }
        _sinceEstimate -= EstimateSeconds;
        float[] input = BuildInput(windows);
        lock (_lock)
        {
            if (_request is not null)
            {
                _skipped++;
            }
            else
            {
                _pending.Release();
            }
            _request = input;
            _requestWindows = windows;
            _requestOrientation = _attitude.Orientation;
            _requestTime = _time;
        }
    }

    /// <summary>
    /// The last <paramref name="windows"/> seconds as the model was trained on
    /// them: consecutive windows of 200 slots, of which every fifth is used,
    /// starting with the window's first.
    /// </summary>
    private float[] BuildInput(int windows)
    {
        const int n = IVelocityModel.WindowSamples;
        float[] input = new float[windows * IVelocityModel.Rows * n];
        long first = _slots - (long)windows * SlotsPerWindow;
        for (int w = 0; w < windows; w++)
        {
            for (int row = 0; row < IVelocityModel.Rows; row++)
            {
                float[] ring = _ring[row];
                int offset = (w * IVelocityModel.Rows + row) * n;
                for (int i = 0; i < n; i++)
                {
                    long slot = first + (long)w * SlotsPerWindow + i * Decimation;
                    input[offset + i] = ring[(int)(slot % RingSlots)];
                }
            }
        }
        return input;
    }

    private void Work()
    {
        Stopwatch watch = new();
        double rateWindowStart = 0;
        int rateCount = 0;
        while (true)
        {
            _pending.Wait();
            if (_stop)
            {
                return;
            }
            float[]? input;
            int windows, generation;
            Quaternion orientation;
            double time;
            lock (_lock)
            {
                input = _request;
                _request = null;
                windows = _requestWindows;
                orientation = _requestOrientation;
                time = _requestTime;
                generation = _generation;
            }
            if (input is null)
            {
                continue; // withdrawn by a reset or a gap
            }
            Vector3 body;
            watch.Restart();
            try
            {
                body = _model.Predict(input, windows);
            }
            catch (Exception e)
            {
                lock (_lock)
                {
                    _error = e.Message;
                }
                continue;
            }
            double ms = watch.Elapsed.TotalMilliseconds;
            Vector3 world = Vector3.Transform(body, orientation);
            lock (_lock)
            {
                if (generation != _generation)
                {
                    continue; // reset while the model was running
                }
                _error = null;
                if (!double.IsNaN(_lastTime) && time > _lastTime)
                {
                    float dt = (float)(time - _lastTime);
                    _position += world * dt;
                    _distance += world.Length() * dt;
                }
                _lastTime = time;
                _velocityBody = body;
                _velocityWorld = world;
                _orientation = orientation;
                _windows = windows;
                _estimates++;
                _inferenceMs += (ms - _inferenceMs) * 0.05;
                if (time - _lastTrackTime >= TrackSeconds || time < _lastTrackTime)
                {
                    _lastTrackTime = time;
                    if (_track.Count >= TrackLimit)
                    {
                        _track.RemoveRange(0, TrackLimit / 10);
                    }
                    _track.Add(_position);
                }
                rateCount++;
                if (time - rateWindowStart >= 1 || time < rateWindowStart)
                {
                    _rate = time > rateWindowStart ? rateCount / (time - rateWindowStart) : 0;
                    rateWindowStart = time;
                    rateCount = 0;
                }
            }
        }
    }

    public void Dispose()
    {
        _stop = true;
        _pending.Release();
        _worker.Join();
        _pending.Dispose();
        _model.Dispose();
    }
}
