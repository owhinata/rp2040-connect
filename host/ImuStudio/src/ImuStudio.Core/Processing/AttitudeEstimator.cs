// SPDX-License-Identifier: Apache-2.0
using System.Numerics;
using ImuStudio.Core.Filters;

namespace ImuStudio.Core.Processing;

/// <summary>
/// Runs the Madgwick filter on every gyro sample with the latest
/// acceleration. The first acceleration sets the starting tilt so the view
/// does not spend seconds converging from the identity.
/// </summary>
public sealed class AttitudeEstimator : ISampleSink
{
    private readonly MadgwickFilter _filter = new();
    private readonly object _lock = new();
    private Vector3 _accel;
    private bool _haveAccel, _started;
    private Quaternion _orientation = Quaternion.Identity;
    private float _beta = 0.05f;

    /// <summary>Filter gain; may be set from another thread.</summary>
    public float Beta
    {
        get { lock (_lock) { return _beta; } }
        set { lock (_lock) { _beta = value; } }
    }

    /// <summary>Rotation from the sensor frame to the world frame (Z up); safe to read from another thread.</summary>
    public Quaternion Orientation
    {
        get { lock (_lock) { return _orientation; } }
    }

    public void Reset()
    {
        _haveAccel = false;
        _started = false;
        _filter.Reset();
        lock (_lock)
        {
            _orientation = Quaternion.Identity;
        }
    }

    /// <summary>
    /// Keeps the attitude. What was lost is typically a few tens of
    /// milliseconds: the rotation in that time is a smaller error than
    /// starting over, which would also throw the heading away.
    /// </summary>
    public void OnGap()
    {
    }

    public void OnAccel(Vector3 metersPerSecond2)
    {
        _accel = metersPerSecond2;
        _haveAccel = true;
    }

    public void OnGyro(Vector3 radiansPerSecond, double dt)
    {
        if (!_haveAccel)
        {
            return;
        }
        if (!_started)
        {
            _filter.SetOrientation(TiltFrom(_accel));
            _started = true;
        }
        float beta;
        lock (_lock)
        {
            beta = _beta;
        }
        _filter.Beta = beta;
        _filter.Update(_accel, radiansPerSecond, (float)dt);
        Quaternion q = _filter.Orientation;
        lock (_lock)
        {
            _orientation = q;
        }
    }

    /// <summary>The rotation with no heading change that takes the measured "up" to world Z.</summary>
    public static Quaternion TiltFrom(Vector3 accel)
    {
        float length = accel.Length();
        if (length < 1e-6f)
        {
            return Quaternion.Identity;
        }
        Vector3 up = accel / length;
        float dot = up.Z;
        if (dot < -0.999999f)
        {
            return new Quaternion(1f, 0f, 0f, 0f); // upside down: half a turn about X
        }
        Vector3 axis = Vector3.Cross(up, Vector3.UnitZ);
        return Quaternion.Normalize(new Quaternion(axis, 1f + dot));
    }
}
