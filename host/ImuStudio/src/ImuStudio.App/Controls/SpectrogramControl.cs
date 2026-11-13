// SPDX-License-Identifier: Apache-2.0
using System.Globalization;
using System.Runtime.InteropServices;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Platform;

namespace ImuStudio.App.Controls;

/// <summary>
/// One channel's spectrogram: time runs to the right, frequency upwards, the
/// colour is the level. A strip on the right shows the latest spectrum as a
/// curve. Columns are written into a bitmap used as a ring, so adding one
/// costs one column of pixels, whatever the history length.
/// </summary>
public sealed class SpectrogramControl : Control
{
    private const int HistoryColumns = 600;   // ten seconds at 60 columns a second
    private const int Rows = 256;
    private const double AxisWidth = 46;
    private const double CurveWidth = 72;
    private const double LogFloorHz = 2;

    private static readonly uint[] s_palette = MakePalette();
    private static readonly IBrush s_background = new SolidColorBrush(Color.FromRgb(0x12, 0x12, 0x16));
    private static readonly IBrush s_text = new SolidColorBrush(Color.FromArgb(0xD0, 0xE0, 0xE0, 0xE0));
    private static readonly IPen s_grid = new Pen(new SolidColorBrush(Color.FromArgb(0x38, 0xFF, 0xFF, 0xFF)), 1);
    private static readonly IPen s_curve = new Pen(new SolidColorBrush(Color.FromRgb(0xF2, 0xC1, 0x4E)), 1.2);
    private static readonly Typeface s_typeface = new("monospace");

    private readonly WriteableBitmap _bitmap = new(new PixelSize(HistoryColumns, Rows), new Vector(96, 96),
                                                   PixelFormat.Bgra8888, AlphaFormat.Opaque);
    private readonly int[] _pixels = new int[Rows];
    private readonly float[] _latest = new float[Rows];
    private readonly int[] _rowFirstBin = new int[Rows], _rowEndBin = new int[Rows];
    private int _write, _bins;
    private double _binHz, _maxHz = double.PositiveInfinity, _shownMaxHz;
    private bool _log, _haveLatest;
    private float _minDb = -70, _maxDb = 20;
    private string _peak = "";

    public string Title { get; set; } = "";
    public string Unit { get; set; } = "";

    public SpectrogramControl()
    {
        ClipToBounds = true;
        Clear();
    }

    /// <summary>The frequency axis: its upper end (infinity for all there is) and its scale.</summary>
    public void SetAxis(double maxHz, bool log)
    {
        if (maxHz != _maxHz || log != _log)
        {
            _maxHz = maxHz;
            _log = log;
            _bins = 0; // mapped again at the next column
            Clear();
        }
    }

    /// <summary>The levels shown as the darkest and the brightest colour.</summary>
    public void SetRange(float minDb, float maxDb)
    {
        _minDb = minDb;
        _maxDb = Math.Max(minDb + 1, maxDb);
        InvalidateVisual();
    }

    public void Clear()
    {
        using (ILockedFramebuffer frame = _bitmap.Lock())
        {
            int[] line = new int[HistoryColumns];
            Array.Fill(line, unchecked((int)s_palette[0]));
            for (int y = 0; y < Rows; y++)
            {
                Marshal.Copy(line, 0, frame.Address + y * frame.RowBytes, HistoryColumns);
            }
        }
        _write = 0;
        _haveLatest = false;
        _peak = "";
        InvalidateVisual();
    }

    /// <summary>Adds columns, oldest first. All must be of one spectrum size.</summary>
    public void Add(IReadOnlyList<float[]> columns, double binHz, float peakHz, float peakDb)
    {
        if (columns.Count == 0)
        {
            return;
        }
        if (columns[0].Length != _bins || binHz != _binHz)
        {
            Map(columns[0].Length, binHz);
        }
        using (ILockedFramebuffer frame = _bitmap.Lock())
        {
            float scale = (s_palette.Length - 1) / (_maxDb - _minDb);
            foreach (float[] db in columns)
            {
                for (int row = 0; row < Rows; row++)
                {
                    float level = float.NegativeInfinity;
                    for (int bin = _rowFirstBin[row]; bin < _rowEndBin[row]; bin++)
                    {
                        level = MathF.Max(level, db[bin]);
                    }
                    _latest[row] = level;
                    int index = (int)Math.Clamp((level - _minDb) * scale, 0, s_palette.Length - 1);
                    _pixels[row] = unchecked((int)s_palette[index]);
                }
                // Row 0 is the lowest frequency; the bitmap's first line is the top.
                for (int row = 0; row < Rows; row++)
                {
                    Marshal.WriteInt32(frame.Address, (Rows - 1 - row) * frame.RowBytes + _write * 4, _pixels[row]);
                }
                _write = (_write + 1) % HistoryColumns;
            }
        }
        _haveLatest = true;
        _peak = string.Create(CultureInfo.InvariantCulture, $"peak {peakHz,7:F1} Hz {peakDb,6:F1} dB");
        InvalidateVisual();
    }

    /// <summary>Works out which bins each row of pixels shows.</summary>
    private void Map(int bins, double binHz)
    {
        _bins = bins;
        _binHz = binHz;
        _shownMaxHz = Math.Min(_maxHz, bins * binHz);
        for (int row = 0; row < Rows; row++)
        {
            int first = (int)Math.Floor(Frequency(row / (double)Rows) / binHz);
            int end = (int)Math.Ceiling(Frequency((row + 1) / (double)Rows) / binHz);
            first = Math.Clamp(first, 0, bins - 1);
            _rowFirstBin[row] = first;
            _rowEndBin[row] = Math.Clamp(end, first + 1, bins);
        }
    }

    /// <summary>Frequency at a height from 0 (bottom) to 1 (top).</summary>
    private double Frequency(double height) =>
        _log ? LogFloorHz * Math.Pow(_shownMaxHz / LogFloorHz, height) : height * _shownMaxHz;

    private double HeightOf(double hz) =>
        _log ? Math.Log(hz / LogFloorHz) / Math.Log(_shownMaxHz / LogFloorHz) : hz / _shownMaxHz;

    public override void Render(DrawingContext context)
    {
        Rect bounds = new(Bounds.Size);
        context.FillRectangle(s_background, bounds);
        Rect image = new(AxisWidth, 2, Math.Max(1, bounds.Width - AxisWidth - CurveWidth - 4), Math.Max(1, bounds.Height - 4));
        Rect curve = new(image.Right + 4, image.Y, CurveWidth, image.Height);

        // The ring: the oldest column is at the write position.
        double split = image.Width * (HistoryColumns - _write) / HistoryColumns;
        if (_write < HistoryColumns)
        {
            context.DrawImage(_bitmap, new Rect(_write, 0, HistoryColumns - _write, Rows),
                              new Rect(image.X, image.Y, split, image.Height));
        }
        if (_write > 0)
        {
            context.DrawImage(_bitmap, new Rect(0, 0, _write, Rows),
                              new Rect(image.X + split, image.Y, image.Width - split, image.Height));
        }

        if (_bins > 0)
        {
            foreach (double hz in Ticks())
            {
                double y = image.Bottom - HeightOf(hz) * image.Height;
                if (y < image.Y - 0.5 || y > image.Bottom + 0.5)
                {
                    continue;
                }
                context.DrawLine(s_grid, new Point(image.X, y), new Point(curve.Right, y));
                FormattedText label = Text(hz >= 1000 ? $"{hz / 1000:0.#}k" : $"{hz:0}");
                context.DrawText(label, new Point(AxisWidth - 4 - label.Width,
                                                  Math.Clamp(y - label.Height / 2, 0, bounds.Height - label.Height)));
            }
        }

        if (_haveLatest)
        {
            StreamGeometry geometry = new();
            using (StreamGeometryContext g = geometry.Open())
            {
                for (int row = 0; row < Rows; row++)
                {
                    double level = Math.Clamp((_latest[row] - _minDb) / (_maxDb - _minDb), 0, 1);
                    Point p = new(curve.X + level * curve.Width, curve.Bottom - (row + 0.5) / Rows * curve.Height);
                    if (row == 0)
                    {
                        g.BeginFigure(p, false);
                    }
                    else
                    {
                        g.LineTo(p);
                    }
                }
                g.EndFigure(false);
            }
            context.DrawGeometry(null, s_curve, geometry);
        }

        FormattedText title = Text($"{Title} [{Unit}]");
        context.DrawText(title, new Point(image.X + 6, image.Y + 3));
        if (_peak.Length > 0)
        {
            context.DrawText(Text(_peak), new Point(image.X + 6, image.Y + 4 + title.Height));
        }
    }

    private IEnumerable<double> Ticks()
    {
        if (_log)
        {
            for (double decade = 1; decade <= _shownMaxHz; decade *= 10)
            {
                foreach (double m in new[] { 1.0, 2.0, 5.0 })
                {
                    if (decade * m >= LogFloorHz * 2 && decade * m <= _shownMaxHz)
                    {
                        yield return decade * m;
                    }
                }
            }
            yield break;
        }
        // About five steps of 1, 2 or 5 times a power of ten.
        double raw = _shownMaxHz / 5;
        double power = Math.Pow(10, Math.Floor(Math.Log10(raw)));
        double step = power * (raw / power < 1.5 ? 1 : raw / power < 3.5 ? 2 : raw / power < 7.5 ? 5 : 10);
        for (double hz = step; hz <= _shownMaxHz + step * 0.01; hz += step)
        {
            yield return hz;
        }
    }

    private static FormattedText Text(string text) =>
        new(text, CultureInfo.InvariantCulture, FlowDirection.LeftToRight, s_typeface, 11, s_text);

    /// <summary>Dark blue through purple and orange to pale yellow: brighter is stronger.</summary>
    private static uint[] MakePalette()
    {
        (double At, int R, int G, int B)[] stops =
        [
            (0.00, 0, 0, 4), (0.15, 30, 12, 80), (0.35, 100, 22, 120), (0.55, 180, 54, 90),
            (0.75, 240, 110, 30), (0.90, 250, 190, 50), (1.00, 252, 255, 180),
        ];
        uint[] palette = new uint[256];
        for (int i = 0; i < palette.Length; i++)
        {
            double t = i / (double)(palette.Length - 1);
            int s = 0;
            while (s < stops.Length - 2 && t > stops[s + 1].At)
            {
                s++;
            }
            double f = (t - stops[s].At) / (stops[s + 1].At - stops[s].At);
            uint r = (uint)(stops[s].R + f * (stops[s + 1].R - stops[s].R));
            uint g = (uint)(stops[s].G + f * (stops[s + 1].G - stops[s].G));
            uint b = (uint)(stops[s].B + f * (stops[s + 1].B - stops[s].B));
            palette[i] = 0xFF000000 | r << 16 | g << 8 | b;
        }
        return palette;
    }
}
