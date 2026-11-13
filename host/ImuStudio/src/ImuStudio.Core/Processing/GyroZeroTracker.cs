// SPDX-License-Identifier: Apache-2.0
namespace ImuStudio.Core.Processing;

/// <summary>
/// Tracks the gyro bias from the samples that pass while the sensor is at
/// rest. This is the board's tracker (drivers/lsm6dso/lsm6dso_zero.c) with
/// the same integer arithmetic, because the RAW stream does not carry what
/// the board tracked.
///
/// Samples are grouped into segments of an eighth of a second. The sensor
/// counts as resting once eight consecutive segments have the same mean
/// angular rate and the same mean acceleration within the limits. The mean
/// rate of that window is blended into the estimate: a plain average over
/// the first windows, then an exponential one. A window whose mean is far
/// from the reference bias is rejected, which bounds what a slow steady
/// turn about the gravity axis can do.
/// </summary>
public sealed class GyroZeroTracker
{
    public const int Segments = 8;
    public const int BiasFraction = 8;
    public const int Warmup = 20;
    private const int One = 1 << BiasFraction;
    private const long GyroSpreadMicroDps = 300_000;
    private const long AccelSpreadMicroG = 2_000;
    private const long GyroRangeCalibratedMicroDps = 1_000_000;
    private const long GyroRangeNominalMicroDps = 5_000_000;

    private readonly int _gyroSpread, _accelSpread, _gyroRange;
    private readonly int[] _reference = new int[3];
    private readonly int[] _gyroSum = new int[3], _accelSum = new int[3];
    private readonly int[] _firstGyro = new int[3], _firstAccel = new int[3];
    private readonly long[] _windowGyro = new long[3];
    private uint _segmentSamples, _gyroCount, _accelCount;
    private int _segments;

    /// <summary>Bias in LSB times 2^<see cref="BiasFraction"/>; valid when <see cref="Updates"/> is not zero.</summary>
    public int[] Bias { get; } = new int[3];
    public uint Updates { get; private set; }
    /// <summary>Resting windows whose mean was too far from the reference.</summary>
    public uint Rejected { get; private set; }

    /// <param name="gyroGain">µdps/LSB</param>
    /// <param name="accelGain">µg/LSB</param>
    /// <param name="reference">The stored bias to stay close to, or null for zero (no calibration).</param>
    public GyroZeroTracker(uint gyroGain, uint accelGain, ReadOnlySpan<int> reference, uint gyroHz)
    {
        bool calibrated = !reference.IsEmpty;
        if (gyroGain != 0 && accelGain != 0)
        {
            _gyroSpread = (int)(GyroSpreadMicroDps * One / gyroGain);
            _accelSpread = (int)(AccelSpreadMicroG * One / accelGain);
            _gyroRange = (int)((calibrated ? GyroRangeCalibratedMicroDps : GyroRangeNominalMicroDps) * One / gyroGain);
        }
        if (calibrated)
        {
            reference.CopyTo(_reference);
        }
        // Below about 26 Hz a segment is too short to average anything.
        uint samples = gyroHz / Segments;
        _segmentSamples = samples < 3 || gyroGain == 0 || accelGain == 0 ? 0 : samples;
    }

    public void Accel(short x, short y, short z)
    {
        _accelSum[0] += x;
        _accelSum[1] += y;
        _accelSum[2] += z;
        _accelCount++;
    }

    /// <summary>Returns true when this sample completed a resting window and changed the bias.</summary>
    public bool Gyro(short x, short y, short z)
    {
        if (_segmentSamples == 0)
        {
            return false;
        }
        _gyroSum[0] += x;
        _gyroSum[1] += y;
        _gyroSum[2] += z;
        if (++_gyroCount < _segmentSamples)
        {
            return false;
        }
        // Without acceleration there is nothing to tell a tilt from a bias.
        if (_accelCount == 0)
        {
            RestartSegment();
            RestartWindow();
            return false;
        }
        Span<int> gyro = stackalloc int[3];
        Span<int> accel = stackalloc int[3];
        for (int axis = 0; axis < 3; axis++)
        {
            gyro[axis] = (int)((long)_gyroSum[axis] * One / (int)_gyroCount);
            accel[axis] = (int)((long)_accelSum[axis] * One / (int)_accelCount);
        }
        RestartSegment();
        if (_segments > 0 && (!Within(gyro, _firstGyro, _gyroSpread) || !Within(accel, _firstAccel, _accelSpread)))
        {
            // Moved: this segment starts a new window.
            RestartWindow();
        }
        if (_segments == 0)
        {
            gyro.CopyTo(_firstGyro);
            accel.CopyTo(_firstAccel);
        }
        for (int axis = 0; axis < 3; axis++)
        {
            _windowGyro[axis] += gyro[axis];
        }
        bool changed = false;
        if (++_segments == Segments)
        {
            changed = Accept();
            RestartWindow();
        }
        return changed;
    }

    private bool Accept()
    {
        Span<int> mean = stackalloc int[3];
        for (int axis = 0; axis < 3; axis++)
        {
            mean[axis] = (int)(_windowGyro[axis] / Segments);
        }
        if (!Within(mean, _reference, _gyroRange))
        {
            Rejected++;
            return false;
        }
        int weight = (int)(Updates < Warmup ? Updates + 1 : Warmup);
        for (int axis = 0; axis < 3; axis++)
        {
            Bias[axis] += (mean[axis] - Bias[axis]) / weight;
        }
        Updates++;
        return true;
    }

    private static bool Within(ReadOnlySpan<int> a, ReadOnlySpan<int> b, int limit) =>
        Math.Abs(a[0] - b[0]) <= limit && Math.Abs(a[1] - b[1]) <= limit && Math.Abs(a[2] - b[2]) <= limit;

    private void RestartSegment()
    {
        _gyroCount = 0;
        _accelCount = 0;
        Array.Clear(_gyroSum);
        Array.Clear(_accelSum);
    }

    private void RestartWindow()
    {
        _segments = 0;
        Array.Clear(_windowGyro);
    }
}
