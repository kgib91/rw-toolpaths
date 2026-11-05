using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Original image domain in surface-local coordinates and its native sample counts.
/// Synthetic edge aprons do not consume the image's guide sampling budget.</summary>
public sealed record CarvingSamplingRegion(double X, double Y, double Width, double Height, int Columns, int Rows);

public sealed record CarvingOptions
{
    /// <summary>Common machine rates, transfer plane and optional earlier-stock proof.
    /// Null retains geometric routing when machine context is unavailable.</summary>
    public MillingMotionSettings? Motion { get; init; }
    public double StepOver { get; init; } = 0.1;
    public double SurfaceAllowance { get; init; }
    public double RadialAllowance { get; init; }
    public double ScanAngle { get; init; }
    public bool BothWays { get; init; } = true;
    public bool InsideOut { get; init; }
    public double Tolerance { get; init; } = 0.02;
    /// <summary>Explicit permitted upward detail filtering; zero disables extra lift.</summary>
    public double MicrotextureLiftAllowanceMm { get; init; }
    /// <summary>Legacy physical clearance-field spacing in millimeters. Used only when SamplingPercent is zero.</summary>
    public double SamplingSpacing { get; init; }
    /// <summary>Percentage of each source image dimension used for the raster guide field.
    /// Values between zero and 100 scale independently onto the physical placement; 100 uses the exact
    /// source-query path. Zero preserves legacy SamplingSpacing behavior. Original pixels remain the
    /// authority for cutter clearance at every percentage.</summary>
    public double SamplingPercent { get; init; }
    public CarvingSamplingRegion? SamplingRegion { get; init; }
    public double OutputRoundingMargin { get; init; }
    public RampSettings Ramp { get; init; } = RampSettings.FromRatio(2);
    public IReadOnlyList<PointD>? StockBoundary { get; init; }
    /// <summary>Return a 2D roughing swept-area summary for callers that consume it.</summary>
    public bool CalculateSweptArea { get; init; } = true;
}

public static class CarvingRoughingToolpaths
{
    public static ToolpathPlan Generate(HeightmapSurface surface, ToolGeometry tool, double depthPerPass,
        CarvingOptions options, CancellationToken cancellationToken = default, Action<double, string>? progress = null)
    {
        Validate(tool, options);
        if (!double.IsFinite(depthPerPass) || depthPerPass <= 0)
            throw new ArgumentException("Carving depth per pass must be positive.");
        double maximum = surface.SurfaceZ - surface.Minimum - options.SurfaceAllowance;
        if (maximum <= 0) return ToolpathPlan.Empty;
        double levels = Math.Ceiling(maximum / depthPerPass);
        if (levels > 10000) throw new InvalidOperationException("Carving requires more than 10000 depth levels; increase depth per pass.");
        int count = (int)levels;
        if (Math.Min(surface.Placement.Width, surface.Placement.Height) / (4 * tool.Radius * options.StepOver) > 9999)
            throw new InvalidOperationException("Carving exceeds the pocket ring budget; increase roughing stepover or cutter diameter.");
        var paths = new List<TaggedToolpath>();
        var swept = new List<List<PointD>>();
        long totalPoints = 0;
        var progressGate = new object();
        var passProgress = new double[count];
        double completedWork = 0;
        void ReportPass(int pass, double fraction, string stage)
        {
            if (progress == null) return;
            lock (progressGate)
            {
                fraction = Math.Clamp(fraction, passProgress[pass], 1);
                completedWork += fraction - passProgress[pass];
                passProgress[pass] = fraction;
                progress(Math.Clamp(completedWork / count, 0, 1), $"Depth {pass + 1}/{count}: {stage}");
            }
        }
        ToolpathPlan GeneratePass(int pass)
        {
            cancellationToken.ThrowIfCancellationRequested();
            long sliceStarted = System.Diagnostics.Stopwatch.GetTimestamp();
            double previous = Math.Min(maximum, pass * depthPerPass);
            double depth = Math.Min(maximum, (pass + 1) * depthPerPass);
            double z = surface.SurfaceZ - depth;
            // The final slice targets the exact source minimum. Reconstructing it
            // by subtracting the allowance after the depth schedule can round a
            // few ULPs below that minimum and discard every flat floor contour.
            double rawSliceLimit = z - options.SurfaceAllowance;
            double sliceLimit = depth == maximum ? surface.Minimum : rawSliceLimit;
            Console.WriteLine($"[Carving.SliceDepth] level={pass + 1}/{count} z={z:R} allowance={options.SurfaceAllowance:R} " +
                $"sourceLimit={sliceLimit:R} minimum={surface.Minimum:R} floorSnap={sliceLimit - rawSliceLimit:R}");
            ReportPass(pass, 0, "Tracing heightmap");
            PathsD rings;
            int boundaryPatches;
            if (!HeightmapSliceContours.TryBuild(surface, sliceLimit, options.Tolerance,
                cancellationToken, out rings, out boundaryPatches,
                (done, total) => ReportPass(pass, 0.3 * done / total, $"Tracing image rows {done:N0}/{total:N0}")))
            {
            // Union bounded batches as we go. Pixel triangles are intermediate fragments, not machining regions.
            var rectangles = new PathsD();
            var sliceBands = new PathsD();
            long fragmentCount = 0, bandPointCount = 0;
            int unionBatches = 0;
            boundaryPatches = 0;
            void FlushFragments()
            {
                if (rectangles.Count == 0) return;
                cancellationToken.ThrowIfCancellationRequested();
                var merged = Clipper.Union(rectangles, new PathsD(), FillRule.NonZero, 6);
                bandPointCount += merged.Sum(ring => (long)ring.Count);
                if (bandPointCount > 4_000_000)
                    throw new InvalidOperationException("Carving slice exceeds four million contour points; increase tolerance or simplify the image.");
                sliceBands.AddRange(merged);
                rectangles.Clear(); unionBatches++;
                cancellationToken.ThrowIfCancellationRequested();
            }
            void AddFragment(PathD polygon)
            {
                if (++fragmentCount > 16_000_000)
                    throw new InvalidOperationException("Carving slice exceeds sixteen million boundary fragments; increase tolerance or simplify the image.");
                rectangles.Add(polygon);
                if (rectangles.Count >= 16384) FlushFragments();
            }
            void AddRectangle(double x0, double y0, double x1, double y1)
            {
                AddFragment(new PathD(new[] { surface.Placement.ToWorld(x0, y0), surface.Placement.ToWorld(x1, y0),
                    surface.Placement.ToWorld(x1, y1), surface.Placement.ToWorld(x0, y1) }));
            }
            void Boundary(double x0, double y0, double x1, double y1, double a, double b, double c, double d)
            {
                if ((++boundaryPatches & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                if (boundaryPatches > 32_000_000)
                    throw new InvalidOperationException("Carving slice exceeds its subdivision budget; increase tolerance or simplify the image.");
                double limit = sliceLimit;
                if (Math.Max(Math.Max(a, b), Math.Max(c, d)) <= limit)
                { AddRectangle(x0, y0, x1, y1); return; }
                if (Math.Min(Math.Min(a, b), Math.Min(c, d)) >= limit) return;
                // Pick the upper diagonal of the bilinear saddle. These planes already
                // bound the patch from above, and agree exactly on neighboring cell edges.
                // A per-cell vertical lift would create artificial gaps at those edges.
                double saddle = a - b - c + d;
                double error = Math.Abs(saddle) / 4;
                if (error <= options.Tolerance / 2)
                {
                    void Triangle((double x, double y, double h)[] points)
                    {
                        var polygon = new PathD();
                        for (int i = 0; i < 3; i++)
                        {
                            var p = points[i]; var q = points[(i + 1) % 3];
                            bool pin = p.h <= limit, qin = q.h <= limit;
                            if (pin) polygon.Add(surface.Placement.ToWorld(p.x, p.y));
                            if (pin != qin)
                            {
                                double t = (limit - p.h) / (q.h - p.h);
                                polygon.Add(surface.Placement.ToWorld(p.x + t * (q.x - p.x), p.y + t * (q.y - p.y)));
                            }
                        }
                        if (polygon.Count >= 3) AddFragment(polygon);
                    }
                    if (saddle >= 0)
                    {
                        Triangle(new[] { (x0, y0, a), (x1, y0, b), (x1, y1, d) });
                        Triangle(new[] { (x0, y0, a), (x1, y1, d), (x0, y1, c) });
                    }
                    else
                    {
                        Triangle(new[] { (x0, y0, a), (x1, y0, b), (x0, y1, c) });
                        Triangle(new[] { (x1, y0, b), (x1, y1, d), (x0, y1, c) });
                    }
                    return;
                }
                double mx = (x0 + x1) / 2, my = (y0 + y1) / 2;
                double ab = (a + b) / 2, ac = (a + c) / 2, bd = (b + d) / 2, cd = (c + d) / 2, center = (a + b + c + d) / 4;
                Boundary(x0, y0, mx, my, a, ab, ac, center); Boundary(mx, y0, x1, my, ab, b, center, bd);
                Boundary(x0, my, mx, y1, ac, center, c, cd); Boundary(mx, my, x1, y1, center, bd, cd, d);
            }
            for (int y = 0; y < surface.Grid.Height - 1; y++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                ReportPass(pass, 0.3 * y / (surface.Grid.Height - 1), "Building slice contours");
                int start = -1;
                for (int x = 0; x <= surface.Grid.Width - 1; x++)
                {
                    bool include = x < surface.Grid.Width - 1 && surface.CellIncluded(x, y)
                        && surface.CellMaximum(x, y) <= sliceLimit;
                    if (!include && x < surface.Grid.Width - 1 && surface.CellIncluded(x, y))
                    {
                        double x0 = x * surface.PixelX, y0 = y * surface.PixelY;
                        double x1 = (x + 1) * surface.PixelX, y1 = (y + 1) * surface.PixelY;
                        Boundary(x0, y0, x1, y1, surface.Sample(x0, y0), surface.Sample(x1, y0), surface.Sample(x0, y1), surface.Sample(x1, y1));
                    }
                    if (include && start < 0) start = x;
                    if (!include && start >= 0)
                    {
                        AddRectangle(start * surface.PixelX, y * surface.PixelY, x * surface.PixelX, (y + 1) * surface.PixelY);
                        start = -1;
                    }
                }
            }
            FlushFragments();
            rings = Clipper.Union(sliceBands, new PathsD(), FillRule.NonZero, 6);
            Console.WriteLine($"[Carving.Slice] level={pass + 1}/{count} source={surface.Grid.Width}x{surface.Grid.Height} fragments={fragmentCount} batches={unionBatches} contourPoints={rings.Sum(ring => (long)ring.Count)} rings={rings.Count} elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(sliceStarted).TotalMilliseconds:F1}");
            }
            if (options.StockBoundary != null)
                rings = Clipper.Intersect(rings, new PathsD { new PathD(options.StockBoundary) }, FillRule.NonZero, 6);
            cancellationToken.ThrowIfCancellationRequested();
            // Move the simplified boundary inward before intersecting it with the exact safe
            // slice. Without this guard, intersection reintroduces most of the pixel fragments.
            // The final intersection remains the authority for all protected holes and peaks.
            long rawPoints = rings.Sum(ring => (long)ring.Count);
            ReportPass(pass, 0.35, "Simplifying slice contours");
            double reductionTolerance = rawPoints > 8192 ? options.Tolerance / 4 : 0;
            var simplified = new PathsD(rings.Select(ring => {
                var reduced = new PathD(reductionTolerance > 0 ? PolygonSimplify.RdpSegmentBounded(ring, reductionTolerance)
                    : PolygonSimplify.Rdp(ring, options.Tolerance / 2));
                return reduced.Count >= 3 && Clipper.Area(reduced) * Clipper.Area(ring) > 0 ? reduced : ring;
            }));
            var inset = simplified;
            if (reductionTolerance > 0)
            {
                var insetter = new ClipperOffset(2, reductionTolerance * PathUtils.Scale / 4);
                insetter.AddPaths(new Paths64(simplified.Select(ring => new Path64(ring.Select(p =>
                    new Point64((long)Math.Round(p.x * PathUtils.Scale), (long)Math.Round(p.y * PathUtils.Scale)))))),
                    JoinType.Round, EndType.Polygon);
                var insetPaths = new Paths64();
                insetter.Execute(-reductionTolerance * PathUtils.Scale - 2, insetPaths);
                inset = new PathsD(insetPaths.Select(ring => new PathD(ring.Select(p =>
                    new PointD(p.X / PathUtils.Scale, p.Y / PathUtils.Scale)))));
            }
            rings = Clipper.Intersect(rings, inset, FillRule.NonZero, 6);
            Console.WriteLine($"[Carving.SliceSimplify] level={pass + 1}/{count} before={rawPoints} simplified={simplified.Sum(ring => (long)ring.Count)} inset={inset.Sum(ring => (long)ring.Count)} after={rings.Sum(ring => (long)ring.Count)} insetGuard={reductionTolerance:R}");
            // Reuse Clipper's topology tree instead of repeatedly scanning every contour for every hole.
            var regions = PolygonOps.DifferenceToComponents(rings, Array.Empty<IReadOnlyList<PointD>>())
                .Where(component => component.Outer.Max(p => p.x) - component.Outer.Min(p => p.x) >= 2 * (tool.Radius + options.RadialAllowance)
                    && component.Outer.Max(p => p.y) - component.Outer.Min(p => p.y) >= 2 * (tool.Radius + options.RadialAllowance))
                .Select(component => new MillingRegion(component.Outer,
                    component.Holes.Select(hole => (IReadOnlyList<PointD>)hole).ToList())).ToList();
            cancellationToken.ThrowIfCancellationRequested();
            // One interval only: rebuilding the full depth schedule for every slice repeats cuts.
            var plan = PocketToolpaths.Generate(regions, tool,
                new DepthSchedule(depth - previous, depth - previous) { SurfaceZ = surface.SurfaceZ - previous },
                new PocketOptions { Motion=options.Motion,
                    StepOver = options.StepOver, StockToLeave = options.RadialAllowance,
                    InsideOut = options.InsideOut, Ramp = options.Ramp,
                    Tolerances = new GeometryTolerances { ArcTolerance = options.Tolerance / 4, SimplifyTolerance = 0 },
                    // Independent rings retain unambiguous clearance across evolving slice islands.
                    SpiralChaining = false,
                    DetectCurves = false,
                    // Carving slices keep the established complete angular medial emitter.
                    BoundOriginalSourceMedial = false,
                    // Slice fills keep the established mutual raw/compact proof.
                    AdaptiveCompactionProposals = false,
                    TrialFilledSweepProposals = false,
                    ParentLocalRescueProposals = false,
                    ExactFirstCoverageProposals = false,
                    DenseFirstArcConstruction = false,
                    TwoOriginConstruction = false,
                    SeparateOriginalFirstConstruction = false,
                    CalculateSweptArea = options.CalculateSweptArea,
                }, cancellationToken, (fraction, stage) => ReportPass(pass, 0.4 + 0.59 * fraction, stage));
            if (Interlocked.Add(ref totalPoints, plan.Toolpaths.Sum(p => (long)p.Points.Count)) > 2_000_000)
                throw new InvalidOperationException("Carving roughing exceeds two million points; increase tolerance or stepover.");
            Console.WriteLine($"[Carving.Rough] level={pass + 1}/{count} z={z:R} regions={regions.Count} boundaryPatches={boundaryPatches} paths={plan.Toolpaths.Count} allowance={options.SurfaceAllowance:R}");
            ReportPass(pass, 1, "Depth complete");
            return plan;
        }
        // Slice geometry and its one-depth pocket are independent. Limit concurrent memory
        // to two levels, then publish the resulting cuts strictly in machining depth order.
        var passPlans = new ToolpathPlan[count];
        int workers = Math.Min(2, Math.Min(count, Environment.ProcessorCount));
        Console.WriteLine($"[Carving.RoughSchedule] levels={count} workers={workers} curves=False sweptArea={options.CalculateSweptArea}");
        try
        {
            Parallel.For(0, count, new ParallelOptions { MaxDegreeOfParallelism = workers,
                CancellationToken = cancellationToken }, pass => passPlans[pass] = GeneratePass(pass));
        }
        catch (AggregateException error)
        {
            var failures = error.Flatten().InnerExceptions;
            System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(
                failures.FirstOrDefault(failure => failure is not OperationCanceledException) ?? failures[0]).Throw();
            throw;
        }
        for (int pass = 0; pass < count; pass++)
        {
            foreach (var path in passPlans[pass].Toolpaths)
                paths.Add(path with { Category = "carving-rough", DepthPassIndex = pass, VisitIdentity = paths.Count });
            swept.AddRange(passPlans[pass].SweptArea);
        }
        paths = ToolpathContinuationPlanner.Reconnect(paths,"carving-rough",2*tool.Radius,cancellationToken,motion:options.Motion,surface:surface.SurfaceZ);
        return new ToolpathPlan(paths, swept, new ToolpathDiagnostics { PathCount = paths.Count, DepthPassCount = count });
    }

    internal static void Validate(ToolGeometry tool, CarvingOptions options)
    {
        if (options.SamplingRegion is { } region
            && (!double.IsFinite(region.X) || !double.IsFinite(region.Y)
                || !double.IsFinite(region.Width) || !double.IsFinite(region.Height)
                || region.Width <= 0 || region.Height <= 0 || region.Columns < 2 || region.Rows < 2))
            throw new ArgumentException("Invalid carving sampling region.", nameof(options));
        if (!double.IsFinite(options.SamplingPercent) || options.SamplingPercent < 0 || options.SamplingPercent > 100)
            throw new ArgumentException("Carving sampling must be between 0 and 100 percent; zero preserves legacy spacing.", nameof(options));
        if (!double.IsFinite(tool.Radius) || tool.Radius <= 0 || !double.IsFinite(options.StepOver)
            || options.StepOver <= 0 || options.StepOver > 1 || !double.IsFinite(options.Tolerance) || options.Tolerance < 0.001
            || !double.IsFinite(options.SurfaceAllowance) || options.SurfaceAllowance < 0
            || !double.IsFinite(options.OutputRoundingMargin) || options.OutputRoundingMargin < 0
            || !double.IsFinite(options.SamplingSpacing) || options.SamplingSpacing < 0
            || !double.IsFinite(options.RadialAllowance) || options.RadialAllowance < 0 || !double.IsFinite(options.ScanAngle))
            throw new ArgumentException("Invalid carving cutter, stepover, allowance, angle or tolerance (minimum 0.001 mm).");
    }


}

public static class CarvingFinishingToolpaths
{
    public static ToolpathPlan Generate(HeightmapSurface surface, ToolGeometry tool, bool ballNose,
        CarvingOptions options, CancellationToken cancellationToken = default, Action<double, string>? progress = null,
        CarvingCutterProfile? cutter = null)
    {
        CarvingRoughingToolpaths.Validate(tool, options);
        bool useField = options.SamplingPercent > 0 ? options.SamplingPercent < 100 : options.SamplingSpacing > 0;
        if (useField && ballNose && cutter == null)
        {
            // Sampled guides still require a physical swept-chord proof. Represent a
            // plain ball nose explicitly so that proof uses its curved tip, not a disk.
            cutter = new CarvingCutterProfile(
                new[] { new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, tool.Radius, tool.Radius) },
                Math.Max(tool.Radius, surface.SurfaceZ - surface.Minimum));
        }
        double r = cutter?.MaximumRadius ?? tool.Radius;
        double tipRadius = cutter?.TipRadius ?? tool.Radius;
        var field = useField ? new CarvingClearanceField(surface, r, ballNose, cutter,
            options, cancellationToken, (fraction, stage) => progress?.Invoke(0.15 * fraction, stage)) : null;
        void Report(double fraction, string stage) => progress?.Invoke(field == null ? fraction : 0.15 + 0.85 * fraction, stage);
        double Height(PointD p) => field?.Sample(p) ?? surface.CutterUpperBound(p, p, r, ballNose,
            options.Tolerance / 8, cancellationToken, options.OutputRoundingMargin, cutter);
        bool slopedProof = field != null || !ballNose || cutter != null;
        double step = 2 * tipRadius * options.StepOver;
        // Keep the rounded footprint strictly inside the image at row endpoints.
        // Exact tangency to a straight ball shaft creates a discontinuous exterior
        // contact constraint, which cannot be represented by a finite sloping chord.
        double inset = tipRadius + options.OutputRoundingMargin + Math.Max(1e-7, tipRadius * 1e-10);
        if (surface.Placement.Width <= 2 * inset || surface.Placement.Height <= 2 * inset)
            throw new InvalidOperationException("The carving image is smaller than the cutter diameter.");
        double angle = options.ScanAngle * Math.PI / 180, dx = Math.Cos(angle), dy = Math.Sin(angle);
        var corners = new[] { surface.Placement.ToWorld(inset, inset), surface.Placement.ToWorld(surface.Placement.Width - inset, inset),
            surface.Placement.ToWorld(surface.Placement.Width - inset, surface.Placement.Height - inset),
            surface.Placement.ToWorld(inset, surface.Placement.Height - inset) };
        double v0 = corners.Min(p => -dy * p.x + dx * p.y), v1 = corners.Max(p => -dy * p.x + dx * p.y);
        double u0 = corners.Min(p => dx * p.x + dy * p.y), u1 = corners.Max(p => dx * p.x + dy * p.y);
        double rowCount = Math.Ceiling((v1 - v0) / step);
        if (rowCount > 100000) throw new InvalidOperationException("Carving exceeds 100000 scanlines; increase stepover.");
        int rows = Math.Max(1, (int)rowCount);
        var paths = new List<TaggedToolpath>();
        int totalPoints = 0;
        long totalSamples = 0;
        long smoothedPoints = 0, proofBudgetSkips = 0, originalReversals = 0, filteredReversals = 0;
        long exactPointQueries = 0, physicalChordQueries = 0, guideRejects = 0, physicalRejects = 0;
        long failedProofs = 0, rejectedValleys = 0, discontinuityBridges = 0;
        long surfaceClampedPoints = 0, surfaceIntersections = 0;
        var rowMaximumBridgeSpan = new double[rows + 1];
        var rowMaximumLift = new double[rows + 1];
        // This is an automatic microtexture allowance, separate from numerical
        // chord accuracy. Limit it to 5% of the ball-tip diameter and 0.05 mm;
        // only sub-tip valleys can be bridged, always above the proven path.
        double microtextureSpan = ballNose ? 2 * tipRadius : 0;
        if(!double.IsFinite(options.MicrotextureLiftAllowanceMm)||options.MicrotextureLiftAllowanceMm<0)throw new ArgumentOutOfRangeException(nameof(options.MicrotextureLiftAllowanceMm));
        double microtextureLift = ballNose ? options.MicrotextureLiftAllowanceMm : 0;
        long started = System.Diagnostics.Stopwatch.GetTimestamp();
        int workers = Math.Clamp(Environment.ProcessorCount - 2, 1, 16);
        Console.WriteLine($"[Carving.Finish] start rows={rows + 1} workers={workers} tolerance={options.Tolerance:R}");
        (List<Point3D[]> Runs, int Points) GenerateRow(int row)
        {
            cancellationToken.ThrowIfCancellationRequested();
            double v = Math.Min(v1, v0 + row * step);
            var wa = new PointD(dx * u0 - dy * v, dy * u0 + dx * v);
            var wb = new PointD(dx * u1 - dy * v, dy * u1 + dx * v);
            var a = surface.Placement.ToLocal(wa.x, wa.y); var b = surface.Placement.ToLocal(wb.x, wb.y);
            double low = 0, high = 1;
            if (!Clip(a.x, b.x, inset, surface.Placement.Width - inset, ref low, ref high)
                || !Clip(a.y, b.y, inset, surface.Placement.Height - inset, ref low, ref high)) return (new(), 0);
            if (options.StockBoundary is { Count: > 0 } stock
                && (!Clip(wa.x, wb.x, stock.Min(p => p.x) + inset, stock.Max(p => p.x) - inset, ref low, ref high)
                    || !Clip(wa.y, wb.y, stock.Min(p => p.y) + inset, stock.Max(p => p.y) - inset, ref low, ref high))) return (new(), 0);
            var start = Mix(a, b, low); var end = Mix(a, b, high);
            if (options.BothWays && (row & 1) == 1) (start, end) = (end, start);
            return GenerateTrace(start, end, row);
        }
        // Raster rows and their turns use the same source, full physical cutter,
        // adaptive chord proof and bounded simplification.
        (List<Point3D[]> Runs, int Points) GenerateTrace(PointD start, PointD end, int row)
        {
            var samples = new List<(PointD p, double z)>();
            long rowExact = 0, rowPhysical = 0, rowGuideReject = 0, rowPhysicalReject = 0;
            long rowFailedProofs = 0, rowRejectedValleys = 0, rowBridges = 0;
            var exactCache = new Dictionary<(double X, double Y), double>();
            double ExactHeight(PointD p)
            {
                if (exactCache.TryGetValue((p.x, p.y), out double cached)) return cached;
                rowExact++;
                double result = surface.CutterUpperBound(p, p, r, ballNose, options.Tolerance / 8,
                    cancellationToken, options.OutputRoundingMargin, cutter);
                exactCache[(p.x, p.y)] = result;
                return result;
            }
            double za = Height(start), zb = Height(end);
            samples.Add((start, field == null ? za : ExactHeight(start)));
            void Refine(PointD p, double zp, PointD q, double zq, int level)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (totalSamples + samples.Count > 16_000_000) throw new InvalidOperationException("Carving exceeds sixteen million clearance samples; increase tolerance or stepover.");
                var mid = Mix(p, q, 0.5);
                double? middleHeight = null;
                if (field != null)
                {
                    // Reject long chords cheaply using the sampled guide. Its heights
                    // cannot certify safety or replace source-derived contact heights.
                    double guideLift = field.ChordLift(p, zp, q, zq, options.Tolerance * 0.5);
                    middleHeight = Height(mid);
                    // Exact apron heights need not meet the interpolated guide at its
                    // boundary. Only test the guide's valley where it owns both ends;
                    // the full-source proof below still tests the actual midpoint.
                    bool guideValley = !field.Contains(p) || !field.Contains(q)
                        || (zp + zq) / 2 - middleHeight.Value <= options.Tolerance * 0.25;
                    if (!double.IsFinite(guideLift) || !guideValley)
                    {
                        rowGuideReject++;
                        if (level >= 24)
                        {
                            Console.Error.WriteLine($"[Carving.Guide] row={row} p=({p.x:R},{p.y:R},{zp:R}) " +
                                $"q=({q.x:R},{q.y:R},{zq:R}) lift={guideLift:R} followsValley={guideValley}");
                            throw new InvalidOperationException("Carving raster guide did not converge.");
                        }
                        Refine(p, zp, mid, middleHeight.Value, level + 1);
                        Refine(mid, middleHeight.Value, q, zq, level + 1);
                        return;
                    }
                }
                // Keep guide endpoints for recursive guide tests. Feeding exact values
                // back into interpolation can make a guide reject its own endpoints.
                double guideZp = zp, guideZq = zq;
                if (field != null)
                {
                    zp = ExactHeight(p);
                    zq = ExactHeight(q);
                }
                rowPhysical++;
                double lift = !slopedProof
                    ? Math.Max(0, surface.CutterUpperBound(p, q, r, true, options.Tolerance / 8, cancellationToken, options.OutputRoundingMargin) - Math.Min(zp, zq))
                    : surface.FlatCutterChordLift(p, zp, q, zq, r, options.Tolerance * 0.5, cancellationToken, options.OutputRoundingMargin, cutter);
                if (slopedProof && !double.IsFinite(lift)
                    && Math.Max(Math.Abs(p.x - q.x), Math.Abs(p.y - q.y))
                        <= Math.Max(surface.PixelX, surface.PixelY) / 8)
                {
                    // The chord proof projects the rounding square into scan coordinates.
                    // At an angle its enclosing box is wider than the original square, so
                    // that bound need not converge to the point-height bound on steep relief.
                    // For short rejected chords, use the same rounded XY footprint as the
                    // point query. Its horizontal upper bound proves the complete chord.
                    lift = Math.Max(0, surface.CutterUpperBound(p, q, r, false, options.Tolerance / 8,
                        cancellationToken, options.OutputRoundingMargin, cutter) - Math.Min(zp, zq));
                    if (lift > options.Tolerance * 0.5) lift = double.PositiveInfinity;
                }
                bool followsValley = true;
                if (slopedProof && lift <= options.Tolerance * 0.75)
                {
                    double middleExact = field == null ? Height(mid) : ExactHeight(mid);
                    if (field == null) middleHeight = middleExact;
                    followsValley = (zp + zq) / 2 + options.Tolerance * 0.5 - middleExact <= options.Tolerance * 0.75;
                }
                if (lift <= options.Tolerance * 0.75 && followsValley)
                {
                    // Lift the complete chord by its proven swept-clearance deficit.
                    // The flat-cutter proof is bounded by half the tolerance. Use that
                    // same allowance on every chord instead of imprinting variations in
                    // the proof's conservative bound onto an otherwise smooth surface.
                    double pz = !slopedProof ? Math.Max(zp, Math.Min(zp, zq) + lift) : zp + options.Tolerance * 0.5;
                    double qz = !slopedProof ? Math.Max(zq, Math.Min(zp, zq) + lift) : zq + options.Tolerance * 0.5;
                    samples[^1] = (p, Math.Max(samples[^1].z, pz));
                    samples.Add((q, qz));
                    return;
                }
                if (level >= 24)
                {
                    double span = Math.Sqrt(Math.Pow(q.x - p.x, 2) + Math.Pow(q.y - p.y, 2));
                    if (lift > options.Tolerance * 0.75
                        && span <= Math.Max(options.OutputRoundingMargin, 1e-7))
                    {
                        // An abrupt wider shaft can create a real jump in required tip
                        // height. At sub-output precision, no sloping chord can follow
                        // that discontinuity. Prove a horizontal bridge over the entire
                        // rounded XY rectangle, retaining vertical motion at each end.
                        double upper = surface.CutterUpperBound(p, q, r, ballNose, options.Tolerance / 8,
                            cancellationToken, options.OutputRoundingMargin, cutter);
                        if (double.IsFinite(upper))
                        {
                            double pz = ExactHeight(p) + options.Tolerance * 0.5;
                            double qz = ExactHeight(q) + options.Tolerance * 0.5;
                            double bridgeZ = Math.Max(upper + options.Tolerance * 0.5,
                                Math.Max(samples[^1].z, Math.Max(pz, qz)));
                            // Preserve the preceding proven endpoint. Raising it in place
                            // would turn the previous chord into an unrelated long ramp.
                            samples.Add((p, bridgeZ));
                            samples.Add((q, bridgeZ));
                            if (qz < bridgeZ) samples.Add((q, qz));
                            rowBridges++;
                            rowMaximumBridgeSpan[row] = Math.Max(rowMaximumBridgeSpan[row], span);
                            return;
                        }
                    }
                    Console.Error.WriteLine($"[Carving.Clearance] row={row} p=({p.x:R},{p.y:R},{zp:R}) q=({q.x:R},{q.y:R},{zq:R}) lift={lift:R} middle={middleHeight:R} followsValley={followsValley} tolerance={options.Tolerance:R}");
                    throw new InvalidOperationException("Carving clearance did not converge; inspect image discontinuities or increase tolerance.");
                }
                double zm = middleHeight ?? Height(mid);
                rowPhysicalReject++;
                if (lift > options.Tolerance * 0.75) rowFailedProofs++;
                else rowRejectedValleys++;
                Refine(p, guideZp, mid, zm, level + 1); Refine(mid, zm, q, guideZq, level + 1);
            }
            Refine(start, za, end, zb, 0);
            // White and masked regions are stock at SurfaceZ. Keep the proven
            // approach, traverse at the top, and continue down the next slope.
            var runs = new List<Point3D[]>();
            var run = new List<Point3D>();
            void Flush()
            {
                // Point bound (1/8), chord lift (1/2), and upward simplification
                // (2 * 3/16) together consume at most the supplied vertical tolerance.
                if (run.Count > 1)
                {
                    IReadOnlyList<Point3D> filtered = run;
                    if (ballNose)
                    {
                        var result = RasterMicrotextureFilter.Apply(run, microtextureSpan,
                            microtextureLift, cancellationToken);
                        filtered = result.Points;
                        Interlocked.Add(ref smoothedPoints, result.RemovedPoints);
                        Interlocked.Add(ref proofBudgetSkips, result.ProofBudgetSkips);
                        Interlocked.Add(ref originalReversals, result.OriginalReversals);
                        Interlocked.Add(ref filteredReversals, result.ResultReversals);
                        rowMaximumLift[row] = Math.Max(rowMaximumLift[row], result.MaximumLift);
                    }
                    var simplified = SimplifyRasterRun(filtered,
                        options.Tolerance * (slopedProof ? 3d / 16 : 1d / 16), cancellationToken);
                    runs.Add(ClampRasterToSurface(simplified, surface.SurfaceZ + options.OutputRoundingMargin,
                        out int clamped, out int intersections));
                    Interlocked.Add(ref surfaceClampedPoints, clamped);
                    Interlocked.Add(ref surfaceIntersections, intersections);
                }
                run.Clear();
            }
            foreach (var sample in samples)
            {
                var world = surface.Placement.ToWorld(sample.p.x, sample.p.y);
                var point = new Point3D(world.x, world.y, sample.z + options.OutputRoundingMargin);
                run.Add(point);
            }
            Flush();
            Interlocked.Add(ref exactPointQueries, rowExact);
            Interlocked.Add(ref physicalChordQueries, rowPhysical);
            Interlocked.Add(ref guideRejects, rowGuideReject);
            Interlocked.Add(ref physicalRejects, rowPhysicalReject);
            Interlocked.Add(ref failedProofs, rowFailedProofs);
            Interlocked.Add(ref rejectedValleys, rowRejectedValleys);
            Interlocked.Add(ref discontinuityBridges, rowBridges);
            return (runs, samples.Count);
        }
        // Keep workers occupied when neighboring rows have very different relief costs.
        // Results contain only compact runs; commit in scan order after the workers finish
        // so scheduling cannot change direction, linking or path IDs.
        var rasterRows = new (List<Point3D[]> Runs, int Points)[rows + 1];
        int completedRows = 0;
        var progressGate = new object();
        Report(0, $"Finishing scanlines 0/{rows + 1:N0}");
        try
        {
            Parallel.For(0, rows + 1, new ParallelOptions { MaxDegreeOfParallelism = workers,
                CancellationToken = cancellationToken }, row =>
            {
                var result = GenerateRow(row);
                if (Interlocked.Add(ref totalSamples, result.Points) > 16_000_000)
                    throw new InvalidOperationException("Carving exceeds sixteen million clearance samples; increase tolerance or stepover.");
                if (Interlocked.Add(ref totalPoints, result.Runs.Sum(run => run.Length)) > 4_000_000)
                    throw new InvalidOperationException("Carving finishing exceeds four million points; increase tolerance or stepover.");
                rasterRows[row] = result;
                int completed;
                lock (progressGate)
                {
                    completed = ++completedRows;
                    Report(0.9 * completed / (rows + 1), $"Finishing scanlines {completed:N0}/{rows + 1:N0}");
                }
                if ((completed & 31) == 0) Console.WriteLine($"[Carving.Finish] completedRows={completed}/{rows + 1} samples={Interlocked.Read(ref totalSamples)} points={Volatile.Read(ref totalPoints)} ball={ballNose} angle={options.ScanAngle:R} elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
            });
        }
        catch (AggregateException error)
        {
            var failures = error.Flatten().InnerExceptions;
            System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(
                failures.FirstOrDefault(failure => failure is not OperationCanceledException) ?? failures[0]).Throw();
            throw;
        }
        if (!rasterRows.Any(row => row.Runs.Any(run =>
            run.Any(point => point.Z < surface.SurfaceZ + options.OutputRoundingMargin - 1e-7))))
        {
            Console.WriteLine("[Carving.Finish] empty reason=no-reachable-relief-below-stock-top");
            Report(1, "No reachable relief below stock top");
            return ToolpathPlan.Empty;
        }
        var rowLinks = new Point3D[rows + 1][];
        int linkedRows = 0, previousRow = -1;
        double maximumLinkLength = 0;
        for (int row = 0; row <= rows; row++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (rasterRows[row].Runs.Count == 0) continue;
            if (options.BothWays && previousRow == row - 1 && previousRow >= 0)
            {
                var previous = rasterRows[previousRow].Runs[^1];
                var current = rasterRows[row].Runs[0];
                var from = previous[^1]; var to = current[0];
                var linkResult = GenerateTrace(surface.Placement.ToLocal(from.X, from.Y),
                    surface.Placement.ToLocal(to.X, to.Y), row);
                var link = linkResult.Runs.Single();
                // Different adjacent chords may carry different conservative lifts.
                // Share the higher endpoint; raising either proven chord preserves
                // clearance and avoids a vertical seam at the row junction.
                link[0] = previous[^1] = new Point3D(from.X, from.Y, Math.Max(from.Z, link[0].Z));
                link[^1] = current[0] = new Point3D(to.X, to.Y, Math.Max(to.Z, link[^1].Z));
                rowLinks[row] = link;
                linkedRows++;
                maximumLinkLength = Math.Max(maximumLinkLength,
                    Math.Sqrt(Math.Pow(to.X - from.X, 2) + Math.Pow(to.Y - from.Y, 2)));
                totalSamples += linkResult.Points;
                totalPoints += link.Length - 1;
                if (totalSamples > 16_000_000 || totalPoints > 4_000_000)
                    throw new InvalidOperationException("Carving finishing links exceed the sample/point budget; increase stepover.");
            }
            previousRow = row;
            Report(0.9 + 0.08 * (row + 1) / (rows + 1), $"Finishing row links {row + 1:N0}/{rows + 1:N0}");
        }
        for (int row = 0; row <= rows; row++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            foreach (var run in rasterRows[row].Runs)
            {
                var link = rowLinks[row];
                var points = link == null ? run : link.Take(link.Length - 1).Concat(run).ToArray();
                int cutStart = link == null ? 0 : link.Length - 1;
                var spans = new List<ToolpathSpan>();
                if (cutStart > 0) spans.Add(new(0, cutStart, ToolpathSpanKind.Link));
                spans.Add(new(cutStart, points.Length - 1, ToolpathSpanKind.Cut));
                paths.Add(new TaggedToolpath(points, spans, row, "carving-finish", null, paths.Count, detectCurves: false));
            }
            rasterRows[row] = default;
        }
        Console.WriteLine($"[Carving.Continuity] bothWays={options.BothWays} linkedRows={linkedRows} " +
            $"maximumLinkLength={maximumLinkLength:R} clearance=original-source-full-profile");
        Console.WriteLine($"[Carving.SurfaceClamp] surface={surface.SurfaceZ:R} roundingReserve={options.OutputRoundingMargin:R} " +
            $"clampedPoints={surfaceClampedPoints} intersections={surfaceIntersections} peakRetracts=0");
        if (ballNose)
            Console.WriteLine($"[Carving.Microtexture] tipDiameter={microtextureSpan:R} allowance={microtextureLift:R} " +
                $"maximumLift={rowMaximumLift.Max():R} removedPoints={smoothedPoints} " +
                $"reversals={originalReversals}->{filteredReversals} proofBudgetSkips={proofBudgetSkips} " +
                $"clearance=upper-envelope");
        Report(1, "Finishing scanlines complete");
        Console.WriteLine($"[Carving.Finish] complete rows={rows + 1} samples={totalSamples} points={totalPoints} paths={paths.Count} tolerance={options.Tolerance:R} elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
        if (discontinuityBridges > 0)
            Console.WriteLine($"[Carving.Discontinuity] bridges={discontinuityBridges} maximumSpan={rowMaximumBridgeSpan.Max():R} " +
                $"clearance=original-source-rectangle");
        Console.WriteLine($"[Carving.SourceProof] method=correlated-section-support exactPointQueries={exactPointQueries} physicalChordQueries={physicalChordQueries} " +
            $"guideRejects={guideRejects} physicalRejects={physicalRejects} failedProofs={failedProofs} rejectedValleys={rejectedValleys}");
        field?.ReportFallbacks();
        paths = ToolpathContinuationPlanner.Reconnect(paths,"carving-finish",2*tool.Radius,cancellationToken,motion:options.Motion,surface:surface.SurfaceZ);
        return new ToolpathPlan(paths, new List<List<PointD>>(), new ToolpathDiagnostics { PathCount = paths.Count, DepthPassCount = 1 });
    }
    internal static Point3D[] ClampRasterToSurface(IReadOnlyList<Point3D> points, double top,
        out int clamped, out int intersections)
    {
        clamped = intersections = 0;
        var result = new List<Point3D>(points.Count);
        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i];
            if (i > 0)
            {
                var a = points[i - 1];
                if ((a.Z < top && p.Z > top) || (a.Z > top && p.Z < top))
                {
                    double t = (top - a.Z) / (p.Z - a.Z);
                    result.Add(new Point3D(a.X + t * (p.X - a.X), a.Y + t * (p.Y - a.Y), top));
                    intersections++;
                }
            }
            if (p.Z > top) clamped++;
            result.Add(new Point3D(p.X, p.Y, Math.Min(p.Z, top)));
        }
        // Clip the complete proven polyline, not just its vertices: lowering an
        // above-top endpoint alone can pull the approaching chord into a peak.
        // Every cutter section lies above its tip and the entire target (including
        // masks/exterior) is bounded by SurfaceZ. Retain output rounding clearance.
        return result.ToArray();
    }
    private static Point3D[] SimplifyRasterRun(IReadOnlyList<Point3D> points, double tolerance, CancellationToken cancellationToken)
    {
        if (points.Count <= 2) return points.ToArray();
        var keep = new bool[points.Count]; keep[0] = keep[^1] = true;
        var pending = new Stack<(int Start, int End)>(); pending.Push((0, points.Count - 1));
        while (pending.TryPop(out var interval))
        {
            cancellationToken.ThrowIfCancellationRequested();
            var a = points[interval.Start]; var b = points[interval.End];
            double dx = b.X - a.X, dy = b.Y - a.Y, length2 = dx * dx + dy * dy;
            if (length2 <= 1e-24)
            {
                for (int i = interval.Start + 1; i < interval.End; i++) keep[i] = true;
                continue;
            }
            int split = -1; double greatest = tolerance;
            for (int i = interval.Start + 1; i < interval.End; i++)
            {
                var p = points[i];
                double t = ((p.X - a.X) * dx + (p.Y - a.Y) * dy) / length2;
                double error = Math.Abs(p.Z - (a.Z + t * (b.Z - a.Z)));
                if (error > greatest) { greatest = error; split = i; }
            }
            if (split >= 0)
            {
                keep[split] = true;
                pending.Push((interval.Start, split)); pending.Push((split, interval.End));
            }
        }
        // A raster is monotone on one XY line. Bounding vertical error at its old vertices
        // bounds it along every old chord. Raising retained vertices by the same error keeps
        // every new chord above the original cutter-clearance envelope (at most 2*error higher).
        return points.Where((_, i) => keep[i]).Select(p => new Point3D(p.X, p.Y, p.Z + tolerance)).ToArray();
    }
    private static PointD Mix(PointD a, PointD b, double t) => new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
    private static bool Clip(double a, double b, double min, double max, ref double low, ref double high)
    {
        if (min > max) return false;
        double d = b - a;
        if (Math.Abs(d) < 1e-12) return a >= min - 1e-8 && a <= max + 1e-8;
        double t0 = (min - a) / d, t1 = (max - a) / d;
        low = Math.Max(low, Math.Min(t0, t1)); high = Math.Min(high, Math.Max(t0, t1));
        return high - low > 1e-10;
    }
}
