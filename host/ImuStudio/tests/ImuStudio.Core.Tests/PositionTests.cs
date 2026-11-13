// SPDX-License-Identifier: Apache-2.0
using System.Numerics;
using FluentAssertions;
using ImuStudio.Core.Processing;
using Xunit;

namespace ImuStudio.Core.Tests;

public class PositionTests
{
    /// <summary>Stands in for the model: a fixed velocity, and a record of what it was given.</summary>
    private sealed class FakeModel(Vector3 velocity) : IVelocityModel
    {
        public List<(float[] Imu, int Windows)> Calls { get; } = new();

        public Vector3 Predict(ReadOnlySpan<float> imu, int windows)
        {
            lock (Calls)
            {
                Calls.Add((imu.ToArray(), windows));
            }
            return velocity;
        }

        public void Dispose()
        {
        }
    }

    private const double Rate = 6400;

    /// <summary>
    /// Feeds samples faster than real time. Requests the estimator's thread
    /// does not get to are skipped, which costs no distance: each estimate
    /// covers the time since the one before.
    /// </summary>
    private static void Feed(PositionEstimator e, double seconds, Func<double, Vector3> gyro, ref double t)
    {
        double dt = 1 / Rate;
        int n = (int)Math.Round(seconds * Rate);
        for (int i = 0; i < n; i++)
        {
            e.OnAccel(new Vector3(0, 0, 9.8f));
            e.OnGyro(gyro(t), dt);
            t += dt;
            if (i % 32 == 0)
            {
                Thread.Yield();
            }
        }
        Thread.Sleep(30);
    }

    [Fact]
    public void Moves_along_the_velocity_the_model_gives()
    {
        FakeModel model = new(new Vector3(1, 0, 0));
        using PositionEstimator e = new(model);
        e.Reset();
        double t = 0;
        Feed(e, 0.9, _ => Vector3.Zero, ref t);
        e.Snapshot().Estimates.Should().Be(0, "a window takes a second to fill");
        Feed(e, 2.1, _ => Vector3.Zero, ref t);
        Thread.Sleep(50);
        PositionSnapshot s = e.Snapshot();
        // Estimates from 1 s to 3 s: two seconds at 1 m/s, less the first interval.
        s.Position.X.Should().BeApproximately(2f, 0.1f);
        s.Position.Y.Should().BeApproximately(0f, 1e-3f);
        s.Distance.Should().BeApproximately(2.0, 0.1);
        s.Windows.Should().Be(3);
        (s.Estimates + s.Skipped).Should().BeInRange(118, 124, "sixty requests a second from 1 s to 3 s");
        e.Track().Length.Should().BeInRange(2, 23);
    }

    [Fact]
    public void Turns_the_velocity_with_the_attitude()
    {
        FakeModel model = new(new Vector3(1, 0, 0));
        using PositionEstimator e = new(model) { Beta = 0 };
        e.Reset();
        double t = 0;
        // A quarter turn to the left in the first second, then straight on: the
        // sensor's forward now points along the world's Y.
        Feed(e, 1.0, _ => new Vector3(0, 0, MathF.PI / 2), ref t);
        Feed(e, 2.0, _ => Vector3.Zero, ref t);
        Thread.Sleep(50);
        PositionSnapshot s = e.Snapshot();
        s.Position.Y.Should().BeApproximately(2f, 0.1f);
        s.Position.X.Should().BeApproximately(0f, 0.05f);
        s.VelocityWorld.Y.Should().BeApproximately(1f, 1e-2f);
        s.VelocityBody.X.Should().Be(1f);
    }

    [Fact]
    public void Lays_the_samples_out_as_the_model_was_trained()
    {
        FakeModel model = new(Vector3.Zero);
        using PositionEstimator e = new(model);
        e.Reset();
        double dt = 1 / Rate;
        // The gyro's X is the time in seconds, so a value tells which sample it is.
        for (int i = 0; i < (int)(2.5 * Rate); i++)
        {
            e.OnAccel(new Vector3(1, 2, 3));
            e.OnGyro(new Vector3((float)(i * dt), 0, 0), dt);
        }
        Thread.Sleep(100);
        (float[] imu, int windows) = model.Calls[^1];
        windows.Should().Be(2);
        imu.Length.Should().Be(2 * 6 * 40);
        // Within a row the samples are 25 ms apart; the two windows follow each other.
        (imu[1] - imu[0]).Should().BeApproximately(0.025f, 1e-4f);
        (imu[39] - imu[0]).Should().BeApproximately(0.975f, 1e-4f);
        (imu[240] - imu[0]).Should().BeApproximately(1f, 1e-4f);
        // The newest sample used is 20 ms (four 5 ms slots) before the newest slot.
        imu[240 + 39].Should().BeInRange(2.5f - 0.05f, 2.5f - 0.02f);
        // Rows 3 to 5 are the acceleration.
        imu[3 * 40].Should().Be(1f);
        imu[4 * 40].Should().Be(2f);
        imu[5 * 40 + 39].Should().Be(3f);
    }

    [Fact]
    public void Starts_over_on_reset_and_keeps_the_position_across_a_gap()
    {
        FakeModel model = new(new Vector3(0, 1, 0));
        using PositionEstimator e = new(model);
        e.Reset();
        double t = 0;
        Feed(e, 2.0, _ => Vector3.Zero, ref t);
        Thread.Sleep(50);
        float before = e.Snapshot().Position.Y;
        before.Should().BeGreaterThan(0.8f);
        e.OnGap();
        Feed(e, 0.5, _ => Vector3.Zero, ref t);
        Thread.Sleep(50);
        e.Snapshot().Position.Y.Should().Be(before, "no window is full again yet");
        Feed(e, 1.0, _ => Vector3.Zero, ref t);
        Thread.Sleep(50);
        e.Snapshot().Position.Y.Should().BeGreaterThan(before);
        e.Reset();
        e.Snapshot().Position.Should().Be(Vector3.Zero);
        e.Track().Should().Equal(Vector3.Zero);
    }
}
