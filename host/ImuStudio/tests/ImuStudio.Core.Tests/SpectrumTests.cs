// SPDX-License-Identifier: Apache-2.0
using System.Numerics;
using FluentAssertions;
using ImuStudio.Core.Processing;
using Xunit;

namespace ImuStudio.Core.Tests;

public class SpectrumTests
{
    [Fact]
    public void Fft_finds_a_sine()
    {
        const int n = 256;
        float[] re = new float[n], im = new float[n];
        for (int i = 0; i < n; i++)
        {
            re[i] = 3f * MathF.Cos(2 * MathF.PI * 10 * i / n) + 1f;
        }
        new Fft(n).Transform(re, im);
        MathF.Sqrt(re[10] * re[10] + im[10] * im[10]).Should().BeApproximately(3f * n / 2, 1e-2f);
        re[0].Should().BeApproximately(n, 1e-2f);
        MathF.Sqrt(re[11] * re[11] + im[11] * im[11]).Should().BeLessThan(1e-2f);
    }

    [Fact]
    public void Fft_needs_a_power_of_two()
    {
        Action make = () => _ = new Fft(1000);
        make.Should().Throw<ArgumentException>();
    }

    /// <summary>Feeds seconds of samples; the sensor lies with its X axis up.</summary>
    private static List<SpectrumColumn> Run(SpectrumAnalyzer a, double seconds, double rate = 6400,
                                            int startSample = 0, bool drain = true)
    {
        double dt = 1 / rate;
        for (int i = startSample; i < startSample + (int)(seconds * rate); i++)
        {
            double t = i * dt;
            // Gravity on X with a 200 Hz vibration of 0.5 m/s² along it; a 50 Hz
            // wobble of 3 dps about the sensor's Z.
            a.OnAccel(new Vector3(9.8f + 0.5f * (float)Math.Sin(2 * Math.PI * 200 * t), 0, 0));
            a.OnGyro(new Vector3(0, 0, 3f * MathF.PI / 180f * (float)Math.Sin(2 * Math.PI * 50 * t)), dt);
        }
        return drain ? a.Drain() : [];
    }

    [Fact]
    public void Analyzer_shows_each_motion_on_its_axis()
    {
        SpectrumAnalyzer a = new() { Size = 4096 };
        a.Reset();
        List<SpectrumColumn> columns = Run(a, 2.0);
        a.Vertical.X.Should().BeApproximately(1f, 1e-3f);
        SpectrumColumn last = columns[^1];
        last.BinHz.Should().BeApproximately(6400.0 / 4096, 1e-9);
        last.Db[0].Length.Should().Be(2048);
        // Vertical acceleration: 0.5 m/s² at 200 Hz is -6 dB.
        last.PeakHz[0].Should().BeApproximately(200f, 1.6f);
        last.PeakDb[0].Should().BeApproximately(-6.02f, 0.3f);
        // The sensor's Z is horizontal axis 2 here (vertical X, horizontal 1 is Y).
        last.PeakHz[5].Should().BeApproximately(50f, 1.6f);
        last.PeakDb[5].Should().BeApproximately(9.54f, 0.3f);
        // Nothing on the other axes.
        last.PeakDb[1].Should().BeLessThan(-100f);
        last.PeakDb[3].Should().BeLessThan(-100f);
    }

    [Fact]
    public void Analyzer_makes_about_sixty_columns_a_second_once_the_window_is_full()
    {
        SpectrumAnalyzer a = new() { Size = 1024 };
        a.Reset();
        // 0.25 s to find the vertical, 0.16 s to fill the window.
        Run(a, 0.3).Should().BeEmpty();
        int count = Run(a, 1.0, startSample: 1920).Count;
        count.Should().BeInRange(50, 61);
        int more = Run(a, 1.0, startSample: 8320).Count;
        more.Should().BeInRange(59, 61);
    }

    [Fact]
    public void Analyzer_refills_the_window_after_a_gap_and_keeps_the_frame()
    {
        SpectrumAnalyzer a = new() { Size = 1024 };
        a.Reset();
        Run(a, 1.0).Should().NotBeEmpty();
        a.OnGap();
        Run(a, 0.1, startSample: 6400).Should().BeEmpty("the window is not full again yet");
        Run(a, 0.2, startSample: 7040).Should().NotBeEmpty();
        a.Vertical.X.Should().BeApproximately(1f, 1e-3f);
    }

    [Fact]
    public void Analyzer_follows_a_change_of_size()
    {
        SpectrumAnalyzer a = new() { Size = 1024 };
        a.Reset();
        Run(a, 1.0)[^1].Db[0].Length.Should().Be(512);
        a.Size = 2048;
        Run(a, 1.0, startSample: 6400)[^1].Db[0].Length.Should().Be(1024);
        Action bad = () => a.Size = 1000;
        bad.Should().Throw<ArgumentOutOfRangeException>();
    }
}
