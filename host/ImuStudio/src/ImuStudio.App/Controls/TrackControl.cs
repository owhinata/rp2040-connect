// SPDX-License-Identifier: Apache-2.0
using System.Globalization;
using System.Numerics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;

namespace ImuStudio.App.Controls;

/// <summary>
/// The path seen from above: world X to the right, world Y up, a grid in
/// metres. The scale follows the path so that all of it stays in view.
/// </summary>
public sealed class TrackControl : Control
{
    private static readonly IBrush s_background = new SolidColorBrush(Color.FromRgb(0x12, 0x12, 0x16));
    private static readonly IBrush s_text = new SolidColorBrush(Color.FromArgb(0xD0, 0xE0, 0xE0, 0xE0));
    private static readonly IPen s_grid = new Pen(new SolidColorBrush(Color.FromArgb(0x30, 0xFF, 0xFF, 0xFF)), 1);
    private static readonly IPen s_axes = new Pen(new SolidColorBrush(Color.FromArgb(0x70, 0xFF, 0xFF, 0xFF)), 1);
    private static readonly IPen s_path = new Pen(new SolidColorBrush(Color.FromRgb(0xF2, 0xC1, 0x4E)), 1.6);
    private static readonly IBrush s_start = new SolidColorBrush(Color.FromRgb(0x58, 0xC4, 0x60));
    private static readonly IBrush s_now = new SolidColorBrush(Color.FromRgb(0xEB, 0x4D, 0x4B));
    private static readonly IPen s_heading = new Pen(s_now, 2);
    private static readonly Typeface s_typeface = new("monospace");

    private Vector3[] _track = [];
    private Vector3 _position;
    private Quaternion _orientation = Quaternion.Identity;
    private double _halfSpan = 1; // metres from the centre to the nearer edge

    public TrackControl()
    {
        ClipToBounds = true;
    }

    /// <summary>The path so far; kept until the next call.</summary>
    public void SetTrack(Vector3[] track)
    {
        _track = track;
        InvalidateVisual();
    }

    public void SetCurrent(Vector3 position, Quaternion orientation)
    {
        _position = position;
        _orientation = orientation;
        InvalidateVisual();
    }

    public override void Render(DrawingContext context)
    {
        Rect bounds = new(Bounds.Size);
        context.FillRectangle(s_background, bounds);
        if (bounds.Width < 10 || bounds.Height < 10)
        {
            return;
        }

        // Everything in view, with room to spare; the scale only steps so the picture does not breathe.
        double minX = Math.Min(0, _position.X), maxX = Math.Max(0, _position.X);
        double minY = Math.Min(0, _position.Y), maxY = Math.Max(0, _position.Y);
        foreach (Vector3 p in _track)
        {
            minX = Math.Min(minX, p.X);
            maxX = Math.Max(maxX, p.X);
            minY = Math.Min(minY, p.Y);
            maxY = Math.Max(maxY, p.Y);
        }
        double centreX = (minX + maxX) / 2, centreY = (minY + maxY) / 2;
        double needed = Math.Max(0.5, Math.Max(maxX - minX, maxY - minY) / 2 * 1.25);
        if (needed > _halfSpan || needed < _halfSpan / 4)
        {
            _halfSpan = Math.Pow(2, Math.Ceiling(Math.Log2(needed)));
        }
        double scale = Math.Min(bounds.Width, bounds.Height) / 2 / _halfSpan; // pixels per metre
        Point Map(double x, double y) =>
            new(bounds.Width / 2 + (x - centreX) * scale, bounds.Height / 2 - (y - centreY) * scale);

        // Grid lines every 1, 2 or 5 × 10ⁿ metres, about eight across.
        double raw = _halfSpan / 4;
        double power = Math.Pow(10, Math.Floor(Math.Log10(raw)));
        double step = power * (raw / power < 1.5 ? 1 : raw / power < 3.5 ? 2 : raw / power < 7.5 ? 5 : 10);
        double left = centreX - bounds.Width / 2 / scale, right = centreX + bounds.Width / 2 / scale;
        double bottom = centreY - bounds.Height / 2 / scale, top = centreY + bounds.Height / 2 / scale;
        for (double x = Math.Ceiling(left / step) * step; x <= right; x += step)
        {
            Point p = Map(x, 0);
            context.DrawLine(Math.Abs(x) < step / 2 ? s_axes : s_grid, new Point(p.X, 0), new Point(p.X, bounds.Height));
            context.DrawText(Text(Label(x, step)), new Point(p.X + 3, bounds.Height - 16));
        }
        for (double y = Math.Ceiling(bottom / step) * step; y <= top; y += step)
        {
            Point p = Map(0, y);
            context.DrawLine(Math.Abs(y) < step / 2 ? s_axes : s_grid, new Point(0, p.Y), new Point(bounds.Width, p.Y));
            context.DrawText(Text(Label(y, step)), new Point(3, p.Y - 15));
        }
        context.DrawText(Text($"grid {Label(step, step)} m   x →  y ↑"), new Point(bounds.Width - 190, 4));

        if (_track.Length > 0)
        {
            StreamGeometry geometry = new();
            using (StreamGeometryContext g = geometry.Open())
            {
                g.BeginFigure(Map(_track[0].X, _track[0].Y), false);
                for (int i = 1; i < _track.Length; i++)
                {
                    g.LineTo(Map(_track[i].X, _track[i].Y));
                }
                g.LineTo(Map(_position.X, _position.Y));
                g.EndFigure(false);
            }
            context.DrawGeometry(null, s_path, geometry);
        }

        context.DrawEllipse(s_start, null, Map(0, 0), 4, 4);
        Point now = Map(_position.X, _position.Y);
        // The sensor's X axis, as far as it lies in the horizontal plane.
        Vector3 forward = Vector3.Transform(Vector3.UnitX, _orientation);
        context.DrawLine(s_heading, now, new Point(now.X + forward.X * 22, now.Y - forward.Y * 22));
        context.DrawEllipse(s_now, null, now, 5, 5);
    }

    private static string Label(double value, double step) =>
        (Math.Abs(value) < step / 1000 ? 0 : value).ToString(step < 1 ? "0.##" : "0", CultureInfo.InvariantCulture);

    private static FormattedText Text(string text) =>
        new(text, CultureInfo.InvariantCulture, FlowDirection.LeftToRight, s_typeface, 11, s_text);
}
