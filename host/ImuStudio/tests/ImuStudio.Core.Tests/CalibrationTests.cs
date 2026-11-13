// SPDX-License-Identifier: Apache-2.0
using System.Numerics;
using FluentAssertions;
using ImuStudio.Core.Calibration;
using Xunit;

namespace ImuStudio.Core.Tests;

public class CalibrationTests
{
    [Fact]
    public void Nominal_uses_the_data_sheet_sensitivity()
    {
        // 244 µg/LSB: 4098 LSB is 1 g. 70000 µdps/LSB: 1000 LSB is 70 dps.
        Vector3 accel = SensorCalibration.Nominal(SensorKind.Accel, 244).Apply(0, 0, 4098);
        accel.Z.Should().BeApproximately(4098 * 244e-6f * 9.80665f, 1e-4f);
        Vector3 gyro = SensorCalibration.Nominal(SensorKind.Gyro, 70000).Apply(1000, -1000, 0);
        gyro.X.Should().BeApproximately(70f * MathF.PI / 180f, 1e-5f);
        gyro.Y.Should().BeApproximately(-70f * MathF.PI / 180f, 1e-5f);
    }

    [Fact]
    public void Decodes_a_stored_value()
    {
        byte[] blob = TestData.CalibrationBlob(1, 70000, 0.001, (10.5, -2, 0));
        SensorCalibration? cal = SensorCalibration.TryDecode(blob, SensorKind.Gyro, 70000);
        cal!.IsStored.Should().BeTrue();
        Vector3 v = cal.Apply(1010, 98, 5);
        v.X.Should().BeApproximately(0.9995f, 1e-6f);
        v.Y.Should().BeApproximately(0.1f, 1e-6f);
        v.Z.Should().BeApproximately(0.005f, 1e-6f);
    }

    [Fact]
    public void Does_not_decode_a_value_for_another_sensor_or_sensitivity()
    {
        byte[] blob = TestData.CalibrationBlob(1, 70000, 0.001, (0, 0, 0));
        SensorCalibration.TryDecode(blob, SensorKind.Accel, 70000).Should().BeNull();
        SensorCalibration.TryDecode(blob, SensorKind.Gyro, 35000).Should().BeNull();
        SensorCalibration.TryDecode(blob.AsSpan(0, 55), SensorKind.Gyro, 70000).Should().BeNull();
        blob[0] = 2;
        SensorCalibration.TryDecode(blob, SensorKind.Gyro, 70000).Should().BeNull();
    }

    [Fact]
    public void Key_matches_the_board()
    {
        SensorCalibration.Key(SensorKind.Accel, 244).Should().Be("imu.cal.accel.244");
        SensorCalibration.Key(SensorKind.Gyro, 70000).Should().Be("imu.cal.gyro.70000");
    }
}
