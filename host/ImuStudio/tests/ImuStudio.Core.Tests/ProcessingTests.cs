// SPDX-License-Identifier: Apache-2.0
using System.Numerics;
using System.Text;
using FluentAssertions;
using ImuStudio.Core.Calibration;
using ImuStudio.Core.Processing;
using Xunit;

namespace ImuStudio.Core.Tests;

public class ProcessingTests
{
    private const uint Hz = 800;

    private static bool Feed(GyroZeroTracker t, int samples, short gx, short ax = 0)
    {
        bool changed = false;
        for (int i = 0; i < samples; i++)
        {
            t.Accel(ax, 0, 4098);
            changed |= t.Gyro(gx, 0, 0);
        }
        return changed;
    }

    [Fact]
    public void Tracker_takes_the_mean_rate_of_a_resting_second()
    {
        GyroZeroTracker t = new(70000, 244, default, Hz);
        Feed(t, 799, 10).Should().BeFalse();
        Feed(t, 1, 10).Should().BeTrue();
        t.Updates.Should().Be(1);
        t.Bias[0].Should().Be(10 << 8);
    }

    [Fact]
    public void Tracker_ignores_a_window_in_which_the_sensor_moved()
    {
        GyroZeroTracker t = new(70000, 244, default, Hz);
        Feed(t, 400, 10);
        Feed(t, 400, 10, ax: 100).Should().BeFalse(); // 24 mg of tilt
        t.Updates.Should().Be(0);
        Feed(t, 800, 10, ax: 100).Should().BeTrue();
    }

    [Fact]
    public void Tracker_rejects_a_rate_far_from_the_stored_bias()
    {
        int[] reference = [0, 0, 0];
        GyroZeroTracker t = new(70000, 244, reference, Hz);
        Feed(t, 800, 20).Should().BeFalse(); // 1.4 dps, limit 1 dps
        t.Rejected.Should().Be(1);
        Feed(t, 800, 10).Should().BeTrue();
    }

    [Fact]
    public void Tracker_needs_acceleration()
    {
        GyroZeroTracker t = new(70000, 244, default, Hz);
        bool changed = false;
        for (int i = 0; i < 1600; i++)
        {
            changed |= t.Gyro(10, 0, 0);
        }
        changed.Should().BeFalse();
    }

    [Fact]
    public void Clock_starts_nominal_and_takes_measurements()
    {
        SampleClock clock = new(6667, 40000, 0);
        clock.Period.Should().BeApproximately(40000.0 / 6667 / 40000, 1e-12);
        clock.Anchor(0, 1000);
        clock.Anchor(10000, 1000 + 60100);
        clock.TicksPerSample.Should().BeApproximately(6.01, 1e-9);
        clock.Status(0, 0);
        clock.Status(40000, 1_000_000);
        clock.TickRateMeasured.Should().BeFalse("one second is too short to measure the rate");
        clock.Status(400000, 9_950_000);
        clock.TickRateMeasured.Should().BeTrue();
        clock.SecondsPerTick.Should().BeApproximately(24.875e-6, 1e-12);
    }

    [Fact]
    public void Clock_ignores_an_implausible_measurement()
    {
        SampleClock clock = new(6667, 40000, 4);
        double nominal = clock.SecondsPerTick;
        nominal.Should().BeApproximately(1 / 40240.0, 1e-12);
        clock.Status(0, 0);
        clock.Status(400000, 20_000_000);
        clock.SecondsPerTick.Should().Be(nominal);
    }

    [Theory]
    [InlineData(0f, 0f, 1f)]
    [InlineData(1f, 0f, 0f)]
    [InlineData(0.3f, -0.5f, 0.8f)]
    [InlineData(0f, 0f, -1f)]
    public void Tilt_takes_the_measured_up_to_world_z(float x, float y, float z)
    {
        Vector3 accel = new Vector3(x, y, z) * 9.8f;
        Vector3 world = Vector3.Transform(Vector3.Normalize(accel), AttitudeEstimator.TiltFrom(accel));
        world.X.Should().BeApproximately(0f, 1e-5f);
        world.Y.Should().BeApproximately(0f, 1e-5f);
        world.Z.Should().BeApproximately(1f, 1e-5f);
    }

    [Fact]
    public void Estimator_starts_at_the_tilt_and_follows_a_rotation()
    {
        AttitudeEstimator e = new();
        const double dt = 1.0 / 6667;
        // Lying on its side: the sensor's X points up.
        e.OnAccel(new Vector3(9.8f, 0, 0));
        e.OnGyro(Vector3.Zero, dt);
        Vector3.Transform(Vector3.UnitX, e.Orientation).Z.Should().BeApproximately(1f, 1e-4f);

        // Flat, then a quarter turn about Z in one second.
        e.Reset();
        e.Beta = 0f;
        e.OnAccel(new Vector3(0, 0, 9.8f));
        for (int i = 0; i < 6667; i++)
        {
            e.OnGyro(new Vector3(0, 0, MathF.PI / 2), dt);
        }
        Vector3 x = Vector3.Transform(Vector3.UnitX, e.Orientation);
        x.X.Should().BeApproximately(0f, 2e-3f);
        x.Y.Should().BeApproximately(1f, 2e-3f);
    }

    private sealed class Recorder : ISampleSink
    {
        public int Resets, Gaps, Accels, Gyros;
        public Vector3 Accel, Gyro;
        public double Dt;
        public void Reset() => Resets++;
        public void OnGap() => Gaps++;
        public void OnAccel(Vector3 a) { Accels++; Accel = a; }
        public void OnGyro(Vector3 g, double dt) { Gyros++; Gyro = g; Dt = dt; }
    }

    [Fact]
    public void Pipeline_converts_with_the_nominal_sensitivity_and_counts()
    {
        ImuPipeline p = new();
        Recorder sink = new();
        p.Sink = sink;
        p.OnMessage("imu/x/raw", TestData.Batch(0, 0, 4, (0, 0, 4098), (1000, 0, 0)));
        sink.Gyros.Should().Be(0, "the sensitivity is unknown before the info");
        p.OnMessage("imu/x/info", TestData.InfoBytes());
        p.OnMessage("imu/x/raw", TestData.Batch(0, 0, 4, (0, 0, 4098), (1000, 0, 0)));
        p.OnMessage("imu/x/raw", TestData.Batch(1, 4, 4, (0, 0, 4098), (1000, 0, 0)));
        sink.Accels.Should().Be(8);
        sink.Gyros.Should().Be(8);
        sink.Accel.Z.Should().BeApproximately(9.806f, 1e-2f);
        sink.Gyro.X.Should().BeApproximately(70f * MathF.PI / 180f, 1e-5f);
        sink.Dt.Should().BeApproximately(1.0 / 6667, 1e-9);
        PipelineSnapshot s = p.Snapshot();
        s.Batches.Should().Be(2);
        s.MissingBatches.Should().Be(0);
        s.GyroCalibrated.Should().BeFalse();
        s.Session.Should().Be(TestData.Session);
    }

    [Fact]
    public void Pipeline_counts_missing_batches_and_tells_the_sink()
    {
        ImuPipeline p = new();
        Recorder sink = new();
        p.Sink = sink;
        p.OnMessage("imu/x/info", TestData.InfoBytes());
        int resets = sink.Resets;
        p.OnMessage("imu/x/raw", TestData.Batch(0, 0, 4, (0, 0, 4098), (0, 0, 0)));
        p.OnMessage("imu/x/raw", TestData.Batch(3, 12, 4, (0, 0, 4098), (0, 0, 0)));
        p.OnMessage("imu/x/raw", new byte[10]);
        PipelineSnapshot s = p.Snapshot();
        s.MissingBatches.Should().Be(2);
        s.InvalidPayloads.Should().Be(1);
        sink.Resets.Should().Be(resets, "a gap is not a new run");
        sink.Gaps.Should().Be(1);
    }

    [Fact]
    public void Pipeline_uses_a_stored_calibration_for_the_announced_sensitivity()
    {
        ImuPipeline p = new();
        Recorder sink = new();
        p.Sink = sink;
        p.SetStored("imu.cal.gyro.70000", TestData.CalibrationBlob(1, 70000, 0.002, (100, 0, 0)));
        p.SetStored("imu.cal.accel.122", TestData.CalibrationBlob(0, 122, 0.5, (0, 0, 0)));
        p.OnMessage("imu/x/info", TestData.InfoBytes());
        p.OnMessage("imu/x/raw", TestData.Batch(0, 0, 1, (0, 0, 4098), (1100, 0, 0)));
        sink.Gyro.X.Should().BeApproximately(2f, 1e-5f);
        sink.Accel.Z.Should().BeApproximately(9.806f, 1e-2f, "the stored value is for another range");
        PipelineSnapshot s = p.Snapshot();
        s.GyroCalibrated.Should().BeTrue();
        s.AccelCalibrated.Should().BeFalse();
        s.GyroBiasDps.X.Should().BeApproximately(7f, 1e-4f);
    }

    [Fact]
    public void Pipeline_tracks_the_gyro_bias_at_rest()
    {
        ImuPipeline p = new();
        Recorder sink = new();
        p.Sink = sink;
        p.OnMessage("imu/x/info", Encoding.UTF8.GetBytes(TestData.Info(odr: 800)));
        for (ulong i = 0; i < 10; i++)
        {
            p.OnMessage("imu/x/raw", TestData.Batch(i, i * 100, 100, (0, 0, 4098), (10, -5, 0)));
        }
        PipelineSnapshot s = p.Snapshot();
        s.BiasUpdates.Should().Be(1);
        s.GyroBiasDps.X.Should().BeApproximately(0.7f, 1e-4f);
        sink.Gyro.Length().Should().BeLessThan(1e-6f);
    }

    [Fact]
    public void Pipeline_keeps_a_run_when_its_info_is_repeated()
    {
        ImuPipeline p = new();
        p.Sink = new Recorder();
        p.OnMessage("imu/x/info", TestData.InfoBytes());
        p.OnMessage("imu/x/raw", TestData.Batch(0, 0, 4, (0, 0, 4098), (0, 0, 0)));
        p.OnMessage("imu/x/info", TestData.InfoBytes());
        p.Snapshot().Batches.Should().Be(1);
    }
}
