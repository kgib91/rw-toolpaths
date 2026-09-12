using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>A sampled cutter-contact field used only to guide raster subdivision.
/// Nodes are point contact heights, so sampling does not spread adjacent raised features.
/// Interpolation is not a clearance certificate; accepted chords must be checked against
/// the original surface and physical cutter.</summary>
internal sealed class CarvingClearanceField
{
    // 32 MiB of doubles. Match the importer's 4,194,304-pixel source budget:
    // a valid 2048-square image must also fit when percentage rounding approaches 100%.
    internal const int MaximumNodes = 4_194_304;
    private readonly double[] _heights;
    private readonly int _width, _height;
    private readonly double _stepX, _stepY;
    private readonly double _originX, _originY, _maximumX, _maximumY;
    private readonly Func<PointD, double> _outsideHeight;
    private long _outsideSamples, _outsideChords;
    internal int Width => _width;
    internal int Height => _height;
    internal CarvingClearanceField(HeightmapSurface surface, double radius, bool ball,
        CarvingCutterProfile? cutter, CarvingOptions options, CancellationToken token, Action<double, string>? progress)
    {
        var region = options.SamplingRegion ?? new CarvingSamplingRegion(0, 0,
            surface.Placement.Width, surface.Placement.Height, surface.Grid.Width, surface.Grid.Height);
        if (!double.IsFinite(region.X + region.Y + region.Width + region.Height)
            || region.X < -1e-8 || region.Y < -1e-8 || region.Width <= 0 || region.Height <= 0
            || region.X + region.Width > surface.Placement.Width + 1e-8
            || region.Y + region.Height > surface.Placement.Height + 1e-8
            || region.Columns < 2 || region.Rows < 2)
            throw new ArgumentException("The carving sampling region must lie within the source surface.", nameof(options));
        double width = options.SamplingPercent > 0
            ? Math.Max(2, Math.Ceiling(region.Columns * options.SamplingPercent / 100))
            : Math.Ceiling(region.Width / options.SamplingSpacing) + 1;
        double height = options.SamplingPercent > 0
            ? Math.Max(2, Math.Ceiling(region.Rows * options.SamplingPercent / 100))
            : Math.Ceiling(region.Height / options.SamplingSpacing) + 1;
        double nodes = width * height;
        bool fits = double.IsFinite(nodes) && nodes <= MaximumNodes;
        Console.WriteLine($"[Carving.FieldBudget] native={region.Columns}x{region.Rows} requested={width:R}x{height:R} " +
            $"nodes={nodes:R} limit={MaximumNodes} samplingPercent={options.SamplingPercent:R} " +
            $"legacySpacing={options.SamplingSpacing:R} accepted={fits}");
        if (!fits)
            throw new InvalidOperationException(options.SamplingPercent > 0
                ? $"Carving detail sampling exceeds {MaximumNodes:N0} field nodes; lower Sampling (%)."
                : $"Carving detail sampling exceeds {MaximumNodes:N0} field nodes; increase legacy Sampling Spacing.");
        _width = checked((int)width); _height = checked((int)height);
        _originX = Math.Max(0, region.X); _originY = Math.Max(0, region.Y);
        _maximumX = Math.Min(surface.Placement.Width, region.X + region.Width);
        _maximumY = Math.Min(surface.Placement.Height, region.Y + region.Height);
        if (_maximumX <= _originX || _maximumY <= _originY)
            throw new ArgumentException("The carving sampling region must have positive physical extent.", nameof(options));
        _stepX = (_maximumX - _originX) / (_width - 1); _stepY = (_maximumY - _originY) / (_height - 1);
        _outsideHeight = p => surface.CutterUpperBound(p, p, radius, ball,
            options.Tolerance / 8, token, options.OutputRoundingMargin, cutter);
        _heights = new double[_width * _height];
        long started = System.Diagnostics.Stopwatch.GetTimestamp();
        int completed = 0; var gate = new object();
        progress?.Invoke(0, $"Cutter clearance samples 0/{_height:N0}");
        Parallel.For(0, _height, new ParallelOptions { CancellationToken = token,
            MaxDegreeOfParallelism = Math.Clamp(Environment.ProcessorCount - 2, 1, 16) }, y =>
        {
            for (int x = 0; x < _width; x++)
            {
                token.ThrowIfCancellationRequested();
                var p = new PointD(_originX + x * _stepX, _originY + y * _stepY);
                _heights[y * _width + x] = surface.CutterUpperBound(p, p, radius, ball,
                    options.Tolerance / 8, token, options.OutputRoundingMargin, cutter);
            }
            lock (gate) progress?.Invoke((double)++completed / _height, $"Cutter clearance samples {completed:N0}/{_height:N0}");
        });
        Console.WriteLine($"[Carving.Field] mode=point-guide source={surface.Grid.Width}x{surface.Grid.Height} native={region.Columns}x{region.Rows} grid={_width}x{_height} " +
            $"region={_originX:R},{_originY:R},{_maximumX - _originX:R},{_maximumY - _originY:R} " +
            $"samplingPercent={options.SamplingPercent:R} spacing={_stepX:R},{_stepY:R} radius={radius:R} " +
            $"elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
    }

    internal double Sample(PointD p)
    {
        if (!Contains(p))
        {
            Interlocked.Increment(ref _outsideSamples);
            return _outsideHeight(p);
        }
        double x = Math.Clamp((p.x - _originX) / _stepX, 0, _width - 1);
        double y = Math.Clamp((p.y - _originY) / _stepY, 0, _height - 1);
        int ix = Math.Min((int)x, _width - 2), iy = Math.Min((int)y, _height - 2);
        double tx = x - ix, ty = y - iy; int i = iy * _width + ix;
        return (_heights[i] * (1 - tx) + _heights[i + 1] * tx) * (1 - ty)
            + (_heights[i + _width] * (1 - tx) + _heights[i + _width + 1] * tx) * ty;
    }

    internal bool Contains(PointD p) => p.x >= _originX && p.x <= _maximumX
        && p.y >= _originY && p.y <= _maximumY;

    internal void ReportFallbacks()
    {
        if (_outsideSamples != 0 || _outsideChords != 0)
            Console.WriteLine($"[Carving.FieldFallback] samples={_outsideSamples} chords={_outsideChords} " +
                "clearance=original-source-full-profile");
    }

    /// <summary>Exact guide-field maximum above a chord: the interpolated height is
    /// quadratic within each crossed cell. This cheaply rejects unsuitable candidates;
    /// passing this test does not certify clearance against the source surface.</summary>
    internal double ChordLift(PointD a, double za, PointD b, double zb, double tolerance)
    {
        // A restricted image guide has no interpolant outside its own rectangle.
        // Defer such chords to the mandatory full-source physical proof; their point
        // samples are exact rather than clamped heights from the nearest guide edge.
        if (!Contains(a) || !Contains(b))
        {
            Interlocked.Increment(ref _outsideChords);
            return 0;
        }
        double dx = b.x - a.x, dy = b.y - a.y;
        double Next(double value, double delta, double step)
        {
            if (Math.Abs(delta) < 1e-12) return double.PositiveInfinity;
            double boundary = delta > 0 ? Math.Floor(value / step + 1e-10) + 1 : Math.Ceiling(value / step - 1e-10) - 1;
            return (boundary * step - value) / delta;
        }
        double tx = Next(a.x - _originX, dx, _stepX), ty = Next(a.y - _originY, dy, _stepY);
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

