using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>A reusable, conservative cutter-center height field. Each node bounds all
/// adjacent cells against the original image and physical cutter. Interpolation therefore
/// stays above the source clearance envelope even when detail sampling is coarser than pixels.</summary>
internal sealed class CarvingClearanceField
{
    private readonly double[] _heights;
    private readonly int _width, _height;
    private readonly double _stepX, _stepY;
    internal int Width => _width;
    internal int Height => _height;
    internal CarvingClearanceField(HeightmapSurface surface, double radius, bool ball,
        CarvingCutterProfile? cutter, CarvingOptions options, CancellationToken token, Action<double, string>? progress)
    {
        double width = options.SamplingPercent > 0
            ? Math.Max(2, Math.Ceiling(surface.Grid.Width * options.SamplingPercent / 100))
            : Math.Ceiling(surface.Placement.Width / options.SamplingSpacing) + 1;
        double height = options.SamplingPercent > 0
            ? Math.Max(2, Math.Ceiling(surface.Grid.Height * options.SamplingPercent / 100))
            : Math.Ceiling(surface.Placement.Height / options.SamplingSpacing) + 1;
        if (!double.IsFinite(width) || !double.IsFinite(height) || width * height > 4_000_000)
            throw new InvalidOperationException(options.SamplingPercent > 0
                ? "Carving detail sampling exceeds four million field nodes; lower Sampling (%)."
                : "Carving detail sampling exceeds four million field nodes; increase legacy Sampling Spacing.");
        _width = checked((int)width); _height = checked((int)height);
        _stepX = surface.Placement.Width / (_width - 1); _stepY = surface.Placement.Height / (_height - 1);
        _heights = new double[_width * _height];
        long started = System.Diagnostics.Stopwatch.GetTimestamp();
        int completed = 0; var gate = new object();
        progress?.Invoke(0, $"Cutter clearance field 0/{_height:N0}");
        Parallel.For(0, _height, new ParallelOptions { CancellationToken = token,
            MaxDegreeOfParallelism = Math.Clamp(Environment.ProcessorCount - 2, 1, 16) }, y =>
        {
            for (int x = 0; x < _width; x++)
            {
                token.ThrowIfCancellationRequested();
                var a = new PointD((x - 1) * _stepX, (y - 1) * _stepY);
                var b = new PointD((x + 1) * _stepX, (y + 1) * _stepY);
                _heights[y * _width + x] = surface.CutterUpperBound(a, b, radius, ball,
                    options.Tolerance / 8, token, options.OutputRoundingMargin, cutter);
            }
            lock (gate) progress?.Invoke((double)++completed / _height, $"Cutter clearance field {completed:N0}/{_height:N0}");
        });
        Console.WriteLine($"[Carving.Field] source={surface.Grid.Width}x{surface.Grid.Height} grid={_width}x{_height} " +
            $"samplingPercent={options.SamplingPercent:R} spacing={_stepX:R},{_stepY:R} radius={radius:R} " +
            $"elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
    }

    internal double Sample(PointD p)
    {
        double x = Math.Clamp(p.x / _stepX, 0, _width - 1), y = Math.Clamp(p.y / _stepY, 0, _height - 1);
        int ix = Math.Min((int)x, _width - 2), iy = Math.Min((int)y, _height - 2);
        double tx = x - ix, ty = y - iy; int i = iy * _width + ix;
        return (_heights[i] * (1 - tx) + _heights[i + 1] * tx) * (1 - ty)
            + (_heights[i + _width] * (1 - tx) + _heights[i + _width + 1] * tx) * ty;
    }

    /// <summary>Exact maximum above a chord: within each crossed bilinear cell the
    /// height along a straight line is quadratic. No physical cutter queries or recursive
    /// pixel subdivision are needed once the field has been prepared.</summary>
    internal double ChordLift(PointD a, double za, PointD b, double zb, double tolerance)
    {
        double dx = b.x - a.x, dy = b.y - a.y;
        double Next(double value, double delta, double step)
        {
            if (Math.Abs(delta) < 1e-12) return double.PositiveInfinity;
            double boundary = delta > 0 ? Math.Floor(value / step + 1e-10) + 1 : Math.Ceiling(value / step - 1e-10) - 1;
            return (boundary * step - value) / delta;
        }
        double tx = Next(a.x, dx, _stepX), ty = Next(a.y, dy, _stepY);
        double dtX = Math.Abs(dx) > 1e-12 ? _stepX / Math.Abs(dx) : double.PositiveInfinity;
        double dtY = Math.Abs(dy) > 1e-12 ? _stepY / Math.Abs(dy) : double.PositiveInfinity;
        double from = 0, maximum = 0;
        double Residual(double t) => Sample(new PointD(a.x + dx * t, a.y + dy * t)) - za - (zb - za) * t;
        double first = Residual(0);
        while (from < 1)
        {
            double to = Math.Min(1, Math.Min(tx, ty));
            double last = Residual(to), middle = Residual((from + to) / 2);
            double c = 2 * (first + last - 2 * middle), linear = last - first - c;
            maximum = Math.Max(maximum, Math.Max(first, last));
            if (c < 0)
            {
                double t = -linear / (2 * c);
                if (t > 0 && t < 1) maximum = Math.Max(maximum, first + linear * t + c * t * t);
            }
            if (maximum > tolerance) return double.PositiveInfinity;
            if (tx <= to + 1e-12) tx += dtX;
            if (ty <= to + 1e-12) ty += dtY;
            from = to; first = last;
        }
        return maximum + 1e-9;
    }
}
