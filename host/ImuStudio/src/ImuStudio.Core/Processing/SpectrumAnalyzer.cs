// SPDX-License-Identifier: Apache-2.0
using System.Numerics;

namespace ImuStudio.Core.Processing;

/// <summary>The six spectra at one instant.</summary>
/// <param name="Db">
/// Per channel, the amplitude of each frequency bin in dB: re 1 m/s² for the
/// acceleration channels, re 1 dps for the angular rate channels. A sine of
/// amplitude A reads 20·log10(A) at its frequency.
/// </param>
/// <param name="BinHz">Width of a bin; bin i is at i × BinHz.</param>
/// <param name="PeakHz">Per channel, the frequency of the strongest bin above DC.</param>
/// <param name="PeakDb">Per channel, the level of that bin.</param>
public sealed record SpectrumColumn(float[][] Db, double BinHz, float[] PeakHz, float[] PeakDb);

/// <summary>
/// Turns the samples into amplitude spectra, about sixty times a second.
///
/// The samples are first expressed in a frame set at the start: one axis is
/// the direction of gravity seen while the sensor rests (the mean of the
/// first quarter second of acceleration), the other two are horizontal, the
/// first one being the sensor's X axis laid flat. The frame is fixed to the
/// sensor: it stays level only as long as the sensor is not tilted.
///
/// Each spectrum is over the last <see cref="Size"/> samples with a Hann
/// window, after removing their mean. The window moves by a sixtieth of a
/// second between spectra, so they overlap heavily: the picture updates at
/// 60 Hz while each column still covers the whole window.
/// </summary>
public sealed class SpectrumAnalyzer : ISampleSink
{
    public const int Channels = 6;
    public static readonly string[] ChannelNames =
    [
        "Accel vertical", "Accel horizontal 1", "Accel horizontal 2",
        "Gyro vertical", "Gyro horizontal 1", "Gyro horizontal 2",
    ];
    public static readonly int[] Sizes = [1024, 2048, 4096, 8192, 16384];

    private const double ColumnsPerSecond = 60;
    private const double LevelSeconds = 0.25;
    private const int QueueLimit = 240;
    private const float RadToDps = 180f / MathF.PI;

    private readonly object _lock = new();
    private readonly Queue<SpectrumColumn> _columns = new();
    private volatile int _requestedSize = 4096;
    private int _size;
    private Fft? _fft;
    private float[] _window = [];
    private float _windowSum;
    private float[][] _ring = [];
    private float[] _re = [], _im = [];
    private int _accelWrite, _gyroWrite, _accelFilled, _gyroFilled, _sinceColumn;
    // The frame: rows are the vertical and the two horizontal axes, in sensor coordinates.
    private Vector3 _vertical, _horizontal1, _horizontal2, _levelSum;
    private int _levelCount, _levelNeeded;
    private bool _levelled;

    /// <summary>Samples per spectrum; one of <see cref="Sizes"/>. May be set from another thread.</summary>
    public int Size
    {
        get => _requestedSize;
        set
        {
            if (Array.IndexOf(Sizes, value) < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(value));
            }
            _requestedSize = value;
        }
    }

    /// <summary>The direction taken as vertical, in sensor coordinates; zero until it is known.</summary>
    public Vector3 Vertical
    {
        get { lock (_lock) { return _levelled ? _vertical : Vector3.Zero; } }
    }

    /// <summary>Takes the spectra made since the last call, oldest first.</summary>
    public List<SpectrumColumn> Drain()
    {
        lock (_lock)
        {
            List<SpectrumColumn> result = new(_columns);
            _columns.Clear();
            return result;
        }
    }

    public void Reset()
    {
        lock (_lock)
        {
            _levelled = false;
            _levelCount = 0;
            _levelNeeded = 0;
            _levelSum = Vector3.Zero;
            _columns.Clear();
        }
        Restart();
    }

    /// <summary>The windows must not span the lost time: they fill again from here.</summary>
    public void OnGap() => Restart();

    private void Restart()
    {
        _accelWrite = _gyroWrite = _accelFilled = _gyroFilled = _sinceColumn = 0;
    }

    private void Configure(int size)
    {
        _size = size;
        _fft = new Fft(size);
        _window = new float[size];
        _windowSum = 0;
        for (int i = 0; i < size; i++)
        {
            _window[i] = 0.5f - 0.5f * MathF.Cos(2 * MathF.PI * i / size);
            _windowSum += _window[i];
        }
        _ring = new float[Channels][];
        for (int c = 0; c < Channels; c++)
        {
            _ring[c] = new float[size];
        }
        _re = new float[size];
        _im = new float[size];
        Restart();
    }

    public void OnAccel(Vector3 metersPerSecond2)
    {
        if (!_levelled)
        {
            _levelSum += metersPerSecond2;
            _levelCount++;
            TryLevel();
            return;
        }
        if (_size != _requestedSize)
        {
            Configure(_requestedSize);
        }
        _ring[0][_accelWrite] = Vector3.Dot(metersPerSecond2, _vertical);
        _ring[1][_accelWrite] = Vector3.Dot(metersPerSecond2, _horizontal1);
        _ring[2][_accelWrite] = Vector3.Dot(metersPerSecond2, _horizontal2);
        _accelWrite = (_accelWrite + 1) % _size;
        _accelFilled = Math.Min(_size, _accelFilled + 1);
    }

    public void OnGyro(Vector3 radiansPerSecond, double dt)
    {
        if (!_levelled)
        {
            if (_levelNeeded == 0 && dt > 0)
            {
                _levelNeeded = Math.Max(1, (int)Math.Round(LevelSeconds / dt));
                TryLevel();
            }
            return;
        }
        if (_size != _requestedSize)
        {
            Configure(_requestedSize);
        }
        Vector3 dps = radiansPerSecond * RadToDps;
        _ring[3][_gyroWrite] = Vector3.Dot(dps, _vertical);
        _ring[4][_gyroWrite] = Vector3.Dot(dps, _horizontal1);
        _ring[5][_gyroWrite] = Vector3.Dot(dps, _horizontal2);
        _gyroWrite = (_gyroWrite + 1) % _size;
        _gyroFilled = Math.Min(_size, _gyroFilled + 1);

        int hop = Math.Max(1, (int)Math.Round(1 / (dt * ColumnsPerSecond)));
        if (++_sinceColumn >= hop && _gyroFilled == _size && _accelFilled == _size)
        {
            _sinceColumn = 0;
            Emit(1 / (dt * _size));
        }
    }

    private void TryLevel()
    {
        if (_levelNeeded == 0 || _levelCount < _levelNeeded || _levelSum.Length() < 1e-6f)
        {
            return;
        }
        Vector3 vertical = Vector3.Normalize(_levelSum);
        // The sensor's X laid flat; its Y if X happens to be the vertical.
        Vector3 reference = MathF.Abs(vertical.X) < 0.9f ? Vector3.UnitX : Vector3.UnitY;
        Vector3 horizontal1 = Vector3.Normalize(reference - Vector3.Dot(reference, vertical) * vertical);
        lock (_lock)
        {
            _vertical = vertical;
            _horizontal1 = horizontal1;
            _horizontal2 = Vector3.Cross(vertical, horizontal1);
            _levelled = true;
        }
    }

    private void Emit(double binHz)
    {
        int bins = _size / 2;
        float[][] db = new float[Channels][];
        float[] peakHz = new float[Channels], peakDb = new float[Channels];
        float scale = 2f / _windowSum;
        for (int c = 0; c < Channels; c++)
        {
            float[] ring = _ring[c];
            int start = c < 3 ? _accelWrite : _gyroWrite; // the oldest sample
            float mean = 0;
            for (int i = 0; i < _size; i++)
            {
                mean += ring[i];
            }
            mean /= _size;
            for (int i = 0; i < _size; i++)
            {
                _re[i] = (ring[(start + i) % _size] - mean) * _window[i];
                _im[i] = 0;
            }
            _fft!.Transform(_re, _im);
            float[] column = new float[bins];
            int peak = 2;
            float peakPower = -1;
            for (int i = 0; i < bins; i++)
            {
                float power = _re[i] * _re[i] + _im[i] * _im[i];
                column[i] = 10f * MathF.Log10(MathF.Max(power * scale * scale, 1e-24f));
                // The first bins hold what is left of the mean and of slow drift.
                if (i >= 2 && power > peakPower)
                {
                    peakPower = power;
                    peak = i;
                }
            }
            db[c] = column;
            peakHz[c] = (float)(peak * binHz);
            peakDb[c] = column[peak];
        }
        lock (_lock)
        {
            if (_columns.Count >= QueueLimit)
            {
                _columns.Dequeue();
            }
            _columns.Enqueue(new SpectrumColumn(db, binHz, peakHz, peakDb));
        }
    }
}
