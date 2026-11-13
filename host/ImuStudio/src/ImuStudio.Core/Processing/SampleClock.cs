// SPDX-License-Identifier: Apache-2.0
namespace ImuStudio.Core.Processing;

/// <summary>
/// Estimates the time between two samples of one run.
///
/// The period is (sensor ticks per sample) × (seconds per tick). Both start
/// from what the board announces and are replaced by measurements once
/// enough of the run has passed: ticks per sample from the ticks attached to
/// batches, seconds per tick from the status messages, which pair a sensor
/// tick with the board's crystal clock.
/// </summary>
public sealed class SampleClock
{
    private const double StatusSpanMicros = 5_000_000;
    private const double Tolerance = 0.05;

    private readonly double _nominalTicksPerSample, _nominalSecondsPerTick;
    private readonly ulong _anchorSpanSamples;
    private bool _haveAnchor, _haveStatus;
    private ulong _firstIndex, _firstTick, _firstStatusTick, _firstStatusMicros;

    public double TicksPerSample { get; private set; }
    public double SecondsPerTick { get; private set; }
    public double Period => TicksPerSample * SecondsPerTick;
    public bool TickRateMeasured { get; private set; }

    public SampleClock(int odrHz, int tickHzNominal, int freqFine)
    {
        // The sensor's oscillator trim: its real rates are 0.15 % per step off nominal.
        double tickHz = tickHzNominal * (1.0 + 0.0015 * freqFine);
        _nominalSecondsPerTick = SecondsPerTick = 1.0 / tickHz;
        _nominalTicksPerSample = TicksPerSample = tickHzNominal / (double)Math.Max(1, odrHz);
        _anchorSpanSamples = (ulong)Math.Max(1, odrHz);
    }

    /// <summary>A batch said that sample <paramref name="index"/> was taken at <paramref name="tick"/>.</summary>
    public void Anchor(ulong index, ulong tick)
    {
        if (!_haveAnchor || index < _firstIndex || tick < _firstTick)
        {
            _haveAnchor = true;
            _firstIndex = index;
            _firstTick = tick;
            return;
        }
        if (index - _firstIndex >= _anchorSpanSamples)
        {
            double measured = (tick - _firstTick) / (double)(index - _firstIndex);
            if (Math.Abs(measured / _nominalTicksPerSample - 1.0) <= Tolerance)
            {
                TicksPerSample = measured;
            }
        }
    }

    /// <summary>A status message said that the sensor tick was <paramref name="tick"/> at board time <paramref name="monoMicros"/>.</summary>
    public void Status(ulong tick, ulong monoMicros)
    {
        if (!_haveStatus || tick < _firstStatusTick || monoMicros < _firstStatusMicros)
        {
            _haveStatus = true;
            _firstStatusTick = tick;
            _firstStatusMicros = monoMicros;
            return;
        }
        double span = monoMicros - _firstStatusMicros;
        if (span >= StatusSpanMicros && tick > _firstStatusTick)
        {
            double measured = span * 1e-6 / (tick - _firstStatusTick);
            if (Math.Abs(measured / _nominalSecondsPerTick - 1.0) <= Tolerance)
            {
                SecondsPerTick = measured;
                TickRateMeasured = true;
            }
        }
    }
}
