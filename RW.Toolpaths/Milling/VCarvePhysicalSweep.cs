using Clipper2Lib;
using System.Diagnostics;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>
/// Surface footprints of penetrating cutting chords, using the actual cutter profile.
/// Lower footprints are inscribed; upper footprints include profile interpolation,
/// circle tessellation and integer-geometry reserves. They do not include retracts.
/// Coordinates are used as supplied: callers auditing machine output must supply its
/// rounded coordinates rather than treating a nominal sweep as a machine certificate.
/// </summary>
internal static class VCarvePhysicalSweep
{
    private const double GridReserve = 4 / PathUtils.Scale;
    private const double PostUnionReserve = 6 / PathUtils.Scale;
    private const double PostUnionArcTolerance = .25 / PathUtils.Scale;
    private const int UnionBatchSize = 32;
    private const int FlatRunChunkChords = 256;
    private const int MaximumUnionHierarchyDepth = 2;
    private const double ConsolidationMaximumPlanarCoordinate = 10_000;
    private const double ConsolidationMaximumZCoordinate = 100;
    private const double ConsolidationMaximumDepth = 100;
    private const double ConsolidationMaximumSphereRadius = 100;
    private const double ConsolidationDepthRoundoff = 5e-12;

    private enum ConsolidationRefusal { None, Profile, SphereRadius, PlanarCoordinate, WorldZ, Depth, SectionJoin, TipReserve }

    internal static double EffectiveTolerance(double requested)
    {
        if (!double.IsFinite(requested) || requested <= 0)
            throw new ArgumentOutOfRangeException(nameof(requested));
        return Math.Max(requested, 32 / PathUtils.Scale);
    }

    internal static VCarveCutterProfile ProfileFor(ToolGeometry tool, double maximumDepth)
    {
        ArgumentNullException.ThrowIfNull(tool);
        if (tool.VCarveProfile is { } physical) return physical;
        if (!double.IsFinite(maximumDepth) || maximumDepth <= 0)
            throw new ArgumentOutOfRangeException(nameof(maximumDepth));
        double bottom = Math.Max(0, tool.BottomRadius ?? 0);
        double height = tool.ConeLength ?? maximumDepth;
        double top = tool.TopRadius ?? bottom + Math.Tan((tool.TipAngleRadians ?? 0) / 2) * height;
        if (!(height > 0) || !(top > 0) || top < bottom)
            throw new ArgumentException("A physical V-carve cutter profile is required.", nameof(tool));
        return new VCarveCutterProfile(
            new[] { new CarvingCutterSection(CarvingCutterSectionKind.Linear, bottom, top, height) }, Math.Min(maximumDepth, height));
    }

    internal static List<List<PointD>> Build(
        IReadOnlyList<TaggedToolpath> paths, ToolGeometry tool, double maximumDepth,
        double outputSurfaceZ, double spatialTolerance,
        CancellationToken cancellationToken = default, bool upperBound = false,
        bool accelerateConstantDepthRuns = true, bool orderedDiscHulls = true,
        bool consolidateHemisphereChords = false)
        => Build(paths, ProfileFor(tool, maximumDepth), outputSurfaceZ, spatialTolerance,
            cancellationToken, upperBound, accelerateConstantDepthRuns, orderedDiscHulls, consolidateHemisphereChords);

    internal static List<List<PointD>> Build(
        IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile,
        double outputSurfaceZ, double spatialTolerance,
        CancellationToken cancellationToken = default, bool upperBound = false,
        bool accelerateConstantDepthRuns = true, bool orderedDiscHulls = true,
        bool consolidateHemisphereChords = false)
        => BuildCore(paths, profile, outputSurfaceZ, spatialTolerance, cancellationToken,
            upperBound, accelerateConstantDepthRuns, orderedDiscHulls, null, consolidateHemisphereChords);

    // Diagnostic nomination only: this result cannot establish cutter coverage.
    internal sealed record UntrustedLowerNomination(List<List<PointD>> Paths)
    {
        internal bool CoverageAuthority => false;
        internal bool RequiresWholeTargetPhysicalCertificate => true;
    }

    internal static UntrustedLowerNomination BuildNomination(
        IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile,
        double outputSurfaceZ, double spatialTolerance, Func<Paths64, Paths64> closedUnion,
        CancellationToken cancellationToken = default,
        bool accelerateConstantDepthRuns = true, bool orderedDiscHulls = true)
    {
        ArgumentNullException.ThrowIfNull(closedUnion);
        return new(BuildCore(paths, profile, outputSurfaceZ, spatialTolerance, cancellationToken,
            false, accelerateConstantDepthRuns, orderedDiscHulls, closedUnion));
    }

    internal static List<List<PointD>> BuildWithTaperedRadiusRuns(
        IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile,
        double outputSurfaceZ, double spatialTolerance,
        CancellationToken cancellationToken = default, bool upperBound = false,
        bool accelerateConstantDepthRuns = true, bool orderedDiscHulls = true,
        bool consolidateHemisphereChords = false, bool enableTaperedRadiusRuns = false)
        => BuildCore(paths, profile, outputSurfaceZ, spatialTolerance, cancellationToken,
            upperBound, accelerateConstantDepthRuns, orderedDiscHulls, null,
            consolidateHemisphereChords, enableTaperedRadiusRuns);

    // Same physical primitives, 32-path leaf batches and ordered parent unions.
    // Parallelization changes execution only; nomination delegates stay serial.
    internal static List<List<PointD>> BuildWithParallelLeafUnions(
        IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile,
        double outputSurfaceZ, double spatialTolerance,
        CancellationToken cancellationToken = default, bool upperBound = false,
        bool accelerateConstantDepthRuns = true, bool orderedDiscHulls = true,
        bool consolidateHemisphereChords = false, bool enableTaperedRadiusRuns = false)
        => BuildCore(paths, profile, outputSurfaceZ, spatialTolerance, cancellationToken,
            upperBound, accelerateConstantDepthRuns, orderedDiscHulls, null,
            consolidateHemisphereChords, enableTaperedRadiusRuns, true);

    private static List<List<PointD>> BuildCore(
        IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile,
        double outputSurfaceZ, double spatialTolerance, CancellationToken cancellationToken,
        bool upperBound, bool accelerateConstantDepthRuns, bool orderedDiscHulls,
        Func<Paths64, Paths64>? nominationUnion, bool consolidateHemisphereChords = false, bool enableTaperedRadiusRuns = false,
        bool enableParallelLeafUnions = false)
    {
        ArgumentNullException.ThrowIfNull(paths);
        ArgumentNullException.ThrowIfNull(profile);
        if (!double.IsFinite(outputSurfaceZ)) throw new ArgumentOutOfRangeException(nameof(outputSurfaceZ));
        double tolerance = EffectiveTolerance(spatialTolerance);
        double interpolationBudget = tolerance / 4, circleBudget = tolerance / 4;
        var sectionStarts = new List<double> { 0 };
        foreach (var section in profile.Sections) sectionStarts.Add(sectionStarts[^1] + section.Height);
        var pending = new Paths64();
        var batches = new Paths64();
        var unionGroups = new List<Paths64>();
        var circles = new Dictionary<int, (double X, double Y)[]>();
        using var orderedHulls = orderedDiscHulls ? new VCarveOrderedDiscHull() : null;
        long chords = 0, pieces = 0, subdivisions = 0, ignored = 0;
        long flatRuns = 0, flatChords = 0, flatChunks = 0;
        var taperedWorkspace = enableTaperedRadiusRuns && !upperBound && nominationUnion is null && accelerateConstantDepthRuns
            ? new TaperedRadiusRun(profile, outputSurfaceZ, cancellationToken) : null;
        bool taperedMode = taperedWorkspace?.Eligible == true;
        long taperedRuns = 0, taperedChords = 0;
        double taperedFitMilliseconds = 0;
        long consolidatedChords = 0, consolidatedPieces = 0, consolidatedVertices = 0;
        long consolidationRequests = 0, consolidationEligible = 0;
        long consolidationProfileRefusals = 0, consolidationRadiusRefusals = 0, consolidationPlanarRefusals = 0;
        long consolidationWorldZRefusals = 0, consolidationDepthRefusals = 0, consolidationTipRefusals = 0;
        long consolidationJoinRefusals = 0;
        List<Point64>? convexWitnesses = null;
        int convexPieces = 0;
        bool mayConsolidate = false;
        long started = Stopwatch.GetTimestamp(), reported = started;
        double offsetMilliseconds = 0, unionMilliseconds = 0;
        long sourceChords = paths.Sum(path => (long)Math.Max(0, path.Points.Count - 1));
        string bound = nominationUnion is not null ? "untrusted-lower-nomination" : upperBound ? "upper" : "lower";
        Console.WriteLine($"[VCarve.PhysicalSweep.Stage] event=start bound={bound} paths={paths.Count} " +
            $"sourceChords={sourceChords} constantDepthRuns={accelerateConstantDepthRuns} effectiveTolerance={tolerance:R} " +
            $"hullUnionBatchSize={UnionBatchSize} maximumUnionHierarchyDepth={MaximumUnionHierarchyDepth} " +
            $"hemisphereConsolidationRequested={consolidateHemisphereChords} lowerOnly=True " +
            $"taperedRunsRequested={enableTaperedRadiusRuns} taperedRunsEnabled={taperedMode} " +
            $"taperedMaximumChords={TaperedRadiusRun.MaximumChords} taperedLossBudget={interpolationBudget:R} " +
            "taperedAuthority=vertex-disc-domination-and-same-section-concavity taperedArithmetic=directed-enclosures " +
            $"consolidationMaximumXY={ConsolidationMaximumPlanarCoordinate:R} " +
            $"consolidationMaximumWorldZ={ConsolidationMaximumZCoordinate:R} " +
            $"consolidationMaximumDepth={ConsolidationMaximumDepth:R} " +
            $"consolidationMaximumSphereRadius={ConsolidationMaximumSphereRadius:R} " +
            $"consolidationSphereJoinReserve={ConsolidationDepthRoundoff:R} " +
            "consolidationRefusal=literal-original-pieces originalUpperUnchanged=True");
        double maximumRadialError = 0;

        Paths64 Union(Paths64 source) => nominationUnion is null
            ? Clipper.Union(source, FillRule.NonZero) : nominationUnion(source);

        using var parallelUnions = enableParallelLeafUnions && nominationUnion is null
            && GeometryParallelism.WorkerLimit > 1
            ? new VCarveOrderedUnionQueue(cancellationToken, Math.Min(4, GeometryParallelism.WorkerLimit)) : null;
        var orderedUnionSlots = parallelUnions is null ? null : new List<VCarveOrderedUnionQueue.Slot>();

        void Progress(string stage, bool force = false)
        {
            if (!force && Stopwatch.GetElapsedTime(reported).TotalSeconds < 2) return;
            reported = Stopwatch.GetTimestamp();
            Console.WriteLine($"[VCarve.PhysicalSweep.Stage] event={stage} bound={bound} " +
                $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} processedChords={chords + ignored} " +
                $"sourceChords={sourceChords} flatRuns={flatRuns} flatChords={flatChords} flatChunks={flatChunks} " +
                $"taperedRuns={taperedRuns} taperedChords={taperedChords} taperedFitMs={taperedFitMilliseconds:F1} " +
                $"hullPieces={pieces} batchContours={batches.Count} offsetMs={offsetMilliseconds:F1} unionMs={unionMilliseconds:F1}");
        }

        void Flush()
        {
            if (pending.Count == 0) return;
            cancellationToken.ThrowIfCancellationRequested();
            if (parallelUnions is not null)
            {
                orderedUnionSlots!.Add(parallelUnions.Submit(pending));
                pending.Clear();
                Progress("hull-batch");
                return;
            }
            long unionStarted = Stopwatch.GetTimestamp();
            var group = Union(pending);
            batches.AddRange(group);
            unionGroups.Add(group);
            unionMilliseconds += Stopwatch.GetElapsedTime(unionStarted).TotalMilliseconds;
            pending.Clear();
            Progress("hull-batch");
        }

        void ConstantRun(IReadOnlyList<Point3D> points, int first, int last, double depth)
        {
            double radius = profile.PhysicalRadiusAtDepth(depth);
            double arcTolerance = tolerance / 8;
            // Clipper's round arcs are polygonal. Two arc-error reserves plus
            // four integer quanta cover arc chords, input-center rounding,
            // offset vertices and subsequent Boolean intersections. Use round
            // caps on genuinely open chunks, including closed source traces:
            // a Joined offset can invert a vanished inner loop into fake stock.
            double margin = 2 * arcTolerance + GridReserve;
            double offsetRadius = upperBound ? radius + margin : radius - margin;
            if (offsetRadius <= GridReserve)
            {
                // Retain the old individual-disc proof at sub-grid engagement.
                for (int i = first + 1; i <= last; i++) Emit(points[i - 1], points[i], radius, radius, 0);
                return;
            }
            int chordCount = last - first;
            bool closed = points[first].X == points[last].X && points[first].Y == points[last].Y;
            int chunkLength = closed ? Math.Min(FlatRunChunkChords, Math.Max(1, chordCount / 2)) : FlatRunChunkChords;
            flatRuns++;
            flatChords += chordCount;
            for (int start = first; start < last; start += chunkLength)
            {
                cancellationToken.ThrowIfCancellationRequested();
                int finish = Math.Min(last, start + chunkLength);
                var centerline = new Path64(finish - start + 1);
                for (int i = start; i <= finish; i++)
                {
                    Point3D point = points[i];
                    if (!double.IsFinite(point.X + point.Y + point.Z))
                        throw new ArgumentException("Physical sweeps require finite cutting coordinates.", nameof(paths));
                    var integer = new Point64((long)Math.Round(point.X * PathUtils.Scale), (long)Math.Round(point.Y * PathUtils.Scale));
                    if (centerline.Count == 0 || centerline[^1] != integer) centerline.Add(integer);
                }
                if (centerline.Count < 2)
                {
                    Emit(points[start], points[finish], radius, radius, 0);
                    continue;
                }
                var offset = new ClipperOffset(miterLimit: 2, arcTolerance: arcTolerance * PathUtils.Scale,
                    preserveCollinear: false, reverseSolution: false);
                offset.AddPath(centerline, JoinType.Round, EndType.Round);
                var band = new Paths64();
                long offsetStarted = Stopwatch.GetTimestamp();
                if (parallelUnions is null) offset.Execute(offsetRadius * PathUtils.Scale, band);
                else
                {
                    using var producerGeometry = parallelUnions.EnterProducerGeometry();
                    offsetStarted = Stopwatch.GetTimestamp();
                    offset.Execute(offsetRadius * PathUtils.Scale, band);
                }
                offsetMilliseconds += Stopwatch.GetElapsedTime(offsetStarted).TotalMilliseconds;
                batches.AddRange(band);
                if (parallelUnions is null) unionGroups.Add(band);
                else orderedUnionSlots!.Add(VCarveOrderedUnionQueue.Slot.Completed(band));
                flatChunks++;
                Progress("constant-depth-offset");
            }
        }

        (double X, double Y)[] Circle(int count)
        {
            if (!circles.TryGetValue(count, out var result))
            {
                result = Enumerable.Range(0, count).Select(i =>
                    (Math.Cos(2 * Math.PI * i / count), Math.Sin(2 * Math.PI * i / count))).ToArray();
                circles.Add(count, result);
            }
            return result;
        }

        void Emit(Point3D a, Point3D b, double ra, double rb, double radialError)
        {
            if (++pieces > 2_000_000)
                throw new InvalidOperationException("V-carve physical sweep exceeded two million certified chord pieces.");
            maximumRadialError = Math.Max(maximumRadialError, radialError);
            if (!upperBound)
            {
                // A pointed endpoint need not itself lie on the integer lattice.
                // Truncate only its sub-grid-width tip, retaining a real inset
                // disc instead of either snapping outside the cut or dropping
                // the entire long tapered hull when that endpoint has no disc.
                const double firstRadius = 2 * GridReserve;
                if (ra < firstRadius && rb > firstRadius)
                {
                    a = Interpolate(a, b, (firstRadius - ra) / (rb - ra));
                    ra = firstRadius;
                }
                else if (rb < firstRadius && ra > firstRadius)
                {
                    b = Interpolate(b, a, (firstRadius - rb) / (ra - rb));
                    rb = firstRadius;
                }
            }
            double reserve = upperBound ? radialError + GridReserve : -GridReserve;
            double largest = Math.Max(ra, rb) + Math.Max(0, reserve);
            int count = 8;
            while (largest * (1 / Math.Cos(Math.PI / count) - 1) > circleBudget)
                count *= 2;
            var unitCircle = Circle(count);
            var vertices = orderedHulls is null ? new List<Point64>(2 * count) : null;
            orderedHulls?.BeginCircle(unitCircle);
            void Disc(Point3D center, double physicalRadius)
            {
                double radius = Math.Max(0, physicalRadius + reserve);
                if (upperBound) radius /= Math.Cos(Math.PI / count);
                if (radius <= 0)
                {
                    var point = new Point64((long)Math.Round(center.X * PathUtils.Scale),
                        (long)Math.Round(center.Y * PathUtils.Scale));
                    double dx = point.X / PathUtils.Scale - center.X, dy = point.Y / PathUtils.Scale - center.Y;
                    // Exact zero-radius endpoints still define a tapered sweep's
                    // tip. Sub-grid discs without a contained lattice point vanish.
                    if (dx * dx + dy * dy <= physicalRadius * physicalRadius)
                    { if (orderedHulls is null) vertices!.Add(point); else orderedHulls.AddPoint(point); }
                    return;
                }
                if (orderedHulls is not null) orderedHulls.AddDisc(center, radius);
                else foreach (var unit in unitCircle)
                    vertices!.Add(new Point64(
                        (long)Math.Round((center.X + radius * unit.X) * PathUtils.Scale),
                        (long)Math.Round((center.Y + radius * unit.Y) * PathUtils.Scale)));
            }
            double dx = b.X - a.X, dy = b.Y - a.Y;
            // A sufficiently steep/deepening chord has one endpoint disc inside
            // the other. The interpolation reserve still bounds a spherical
            // bulge between them; sorting the redundant disc adds no coverage.
            if ((ra - rb) * (ra - rb) >= dx * dx + dy * dy)
            {
                if (ra >= rb) Disc(a, ra); else Disc(b, rb);
            }
            else { Disc(a, ra); Disc(b, rb); }
            var hull = orderedHulls is null ? ConvexHull(vertices!) : orderedHulls.FinishHull();
            if (hull.Count >= 3)
            {
                if (convexWitnesses is not null)
                { convexWitnesses.AddRange(hull); convexPieces++; }
                else pending.Add(hull);
            }
            if (pending.Count >= UnionBatchSize) Flush();
        }

        void Refine(Point3D a, Point3D b, int sectionIndex, int recursion)
        {
            cancellationToken.ThrowIfCancellationRequested();
            double da = Math.Max(0, outputSurfaceZ - a.Z), db = Math.Max(0, outputSurfaceZ - b.Z);
            double ra = profile.PhysicalRadiusAtDepth(da), rb = profile.PhysicalRadiusAtDepth(db);
            double error = 0;
            if (sectionIndex < profile.Sections.Count
                && profile.Sections[sectionIndex].Kind == CarvingCutterSectionKind.Hemisphere)
            {
                double radius = profile.Sections[sectionIndex].TopRadius;
                double ha = Math.Clamp(Math.Min(da, db) - sectionStarts[sectionIndex], 0, radius);
                double hb = Math.Clamp(Math.Max(da, db) - sectionStarts[sectionIndex], 0, radius);
                if (hb > ha)
                {
                    double radiusRange = Math.Abs(rb - ra);
                    if (radiusRange <= interpolationBudget)
                        error = radiusRange; // Also avoids a singular derivative at subnormal tip depths.
                    else
                    {
                        double lowRadius = Math.Min(ra, rb), slope = radiusRange / (hb - ha);
                        double root = Math.Sqrt(1 + slope * slope);
                        // Exact maximum of the concave spherical radius above its secant.
                        double contact = Math.Clamp(radius / (root * (root + slope)), ha, hb);
                        double contactRadius = Math.Sqrt(Math.Max(0, contact * (2 * radius - contact)));
                        error = Math.Max(0, contactRadius - lowRadius - slope * (contact - ha));
                    }
                }
            }
            if (error > interpolationBudget)
            {
                if (recursion >= 48)
                    throw new InvalidOperationException($"V-carve sweep could not certify profile interpolation: error={error:R}, tolerance={tolerance:R}.");
                subdivisions++;
                if (recursion == 0 && mayConsolidate) convexWitnesses = new List<Point64>();
                var mid = Interpolate(a, b, .5);
                Refine(a, mid, sectionIndex, recursion + 1);
                Refine(mid, b, sectionIndex, recursion + 1);
            }
            else Emit(a, b, ra, rb, error);
        }

        foreach (var path in paths)
        {
            cancellationToken.ThrowIfCancellationRequested();
            bool Active(int end) => path.Spans.Count == 0 || path.Spans.Any(span => span.StartIndex < end
                && span.EndIndex >= end && span.Kind != ToolpathSpanKind.Retract);
            for (int i = 1; i < path.Points.Count; i++)
            {
                if (!Active(i)) { ignored++; continue; }
                var a = path.Points[i - 1]; var b = path.Points[i];
                if (!double.IsFinite(a.X + a.Y + a.Z + b.X + b.Y + b.Z))
                    throw new ArgumentException("Physical sweeps require finite cutting coordinates.", nameof(paths));
                double da = outputSurfaceZ - a.Z, db = outputSurfaceZ - b.Z;
                if (Math.Max(da, db) <= 0) { ignored++; continue; }
                if (accelerateConstantDepthRuns && a.Z == b.Z)
                {
                    int last = i;
                    while (last + 1 < path.Points.Count && path.Points[last + 1].Z == a.Z && Active(last + 1)) last++;
                    if (last - i + 1 >= 3)
                    {
                        chords += last - i + 1;
                        ConstantRun(path.Points, i - 1, last, da);
                        i = last;
                        continue;
                    }
                }
                if (taperedMode)
                {
                    long fitStarted = Stopwatch.GetTimestamp();
                    bool accepted = taperedWorkspace!.TryFind(path.Points, i - 1, Active, interpolationBudget,
                        cancellationToken, out var tapered);
                    taperedFitMilliseconds += Stopwatch.GetElapsedTime(fitStarted).TotalMilliseconds;
                    if (accepted)
                    {
                        int consumed = tapered.Last - tapered.First;
                        chords += consumed;taperedChords += consumed;taperedRuns++;
                        convexWitnesses = null;mayConsolidate = false;
                        // Affine-radius disc sweep equals the endpoint-disc convex hull.
                        // Existing Emit retains its circle/grid and later Boolean reserves.
                        Emit(path.Points[tapered.First], path.Points[tapered.Last], tapered.Radius0, tapered.Radius1, tapered.MaximumLoss);
                        i = tapered.Last;continue;
                    }
                }
                chords++;
                var breaks = new List<double> { 0, 1 };
                if (da != db)
                    foreach (double sectionDepth in sectionStarts)
                    {
                        double t = (sectionDepth - da) / (db - da);
                        if (t > 0 && t < 1) breaks.Add(t);
                    }
                breaks.Sort();
                for (int part = 1; part < breaks.Count; part++)
                {
                    double from = breaks[part - 1], to = breaks[part];
                    double middleDepth = da + (db - da) * ((from + to) / 2);
                    if (middleDepth <= 0) continue;
                    int sectionIndex = 0;
                    while (sectionIndex < profile.Sections.Count && middleDepth >= sectionStarts[sectionIndex + 1]) sectionIndex++;
                    // For one affine XY/depth chord wholly in the first sphere,
                    // r(h)=sqrt(h(2R-h)) is concave. Every convex combination of
                    // real sampled-disc witnesses therefore remains in its true
                    // continuous physical sweep. Combine the existing reserved
                    // lower pieces without performing internal Boolean unions.
                    // The original upper target, sampling error, integer reserves
                    // and postunion bound remain unchanged. Never span an XY bend
                    // or a section transition with this convexity argument.
                    Point3D partA = Interpolate(a, b, from), partB = Interpolate(a, b, to);
                    mayConsolidate = false;
                    if (consolidateHemisphereChords && !upperBound)
                    {
                        consolidationRequests++;
                        var refusal = ConsolidationGuard(a, b, partA, partB, sectionIndex, profile, outputSurfaceZ);
                        mayConsolidate = refusal == ConsolidationRefusal.None;
                        switch (refusal)
                        {
                            case ConsolidationRefusal.None: consolidationEligible++; break;
                            case ConsolidationRefusal.Profile: consolidationProfileRefusals++; break;
                            case ConsolidationRefusal.SphereRadius: consolidationRadiusRefusals++; break;
                            case ConsolidationRefusal.PlanarCoordinate: consolidationPlanarRefusals++; break;
                            case ConsolidationRefusal.WorldZ: consolidationWorldZRefusals++; break;
                            case ConsolidationRefusal.Depth: consolidationDepthRefusals++; break;
                            case ConsolidationRefusal.SectionJoin: consolidationJoinRefusals++; break;
                            case ConsolidationRefusal.TipReserve: consolidationTipRefusals++; break;
                        }
                    }
                    convexWitnesses = null;
                    convexPieces = 0;
                    Refine(partA, partB, sectionIndex, 0);
                    if (convexWitnesses is not null)
                    {
                        cancellationToken.ThrowIfCancellationRequested();
                        var hull = ConvexHull(convexWitnesses);
                        if (hull.Count >= 3) pending.Add(hull);
                        consolidatedChords++;
                        consolidatedPieces += convexPieces;
                        consolidatedVertices += convexWitnesses.Count;
                        convexWitnesses = null;
                        if (pending.Count >= UnionBatchSize) Flush();
                    }
                }
            }
        }
        Flush();
        if (parallelUnions is not null)
        {
            parallelUnions.Complete();
            foreach (var slot in orderedUnionSlots!) unionGroups.Add(parallelUnions.Read(slot));
        }
        cancellationToken.ThrowIfCancellationRequested();
        Progress("union-start", true);
        double serialLeafUnionMilliseconds = unionMilliseconds;
        long finalUnionStarted = Stopwatch.GetTimestamp();
        // Nearby chunks overlap heavily. Canonicalize bounded groups before the
        // final union so very long traces do not send every redundant cap into
        // one monolithic intersection sweep. Limit the rounding chain: one hull
        // vertex rounding, one pending-batch union, two hierarchy unions and the
        // final union move boundary points by at most 5*sqrt(.5) grid quanta,
        // within the four-quantum radial reserve. A third hierarchy level would
        // exceed that reserve. Offset runs have their additional arc reserve.
        int hierarchyDepth = 0;
        while (unionGroups.Count > 64 && hierarchyDepth < MaximumUnionHierarchyDepth)
        {
            var next = new List<Paths64>();
            for (int first = 0; first < unionGroups.Count; first += 32)
            {
                cancellationToken.ThrowIfCancellationRequested();
                // Keep each canonical group's outer rings and holes together.
                // Splitting a hole from its outer ring before union would turn
                // the isolated negative contour into falsely covered material.
                var source = new Paths64();
                for (int i = first; i < Math.Min(unionGroups.Count, first + 32); i++) source.AddRange(unionGroups[i]);
                next.Add(Union(source));
                Progress("union-batch");
            }
            unionGroups = next;
            hierarchyDepth++;
        }
        batches = new Paths64();
        foreach (var group in unionGroups) batches.AddRange(group);
        var combined = Union(batches);
        cancellationToken.ThrowIfCancellationRequested();
        long postUnionStarted = Stopwatch.GetTimestamp();
        var result = PathUtils.FromClipper(BoundBooleanResult(combined, upperBound));
        double postUnionMilliseconds = Stopwatch.GetElapsedTime(postUnionStarted).TotalMilliseconds;
        cancellationToken.ThrowIfCancellationRequested();
        unionMilliseconds += Stopwatch.GetElapsedTime(finalUnionStarted).TotalMilliseconds;
        // Includes hull-vertex and all Boolean rounding, plus offset-vertex,
        // offset-internal-union and final set-operation rounding. At the minimum
        // tolerance this is less than 32 integer quanta (31.907 at depth two).
        double uncertaintyBound = interpolationBudget + circleBudget + GridReserve
            + (hierarchyDepth + 3) * Math.Sqrt(.5) / PathUtils.Scale
            + PostUnionReserve + PostUnionArcTolerance + 3 * Math.Sqrt(.5) / PathUtils.Scale;
        Console.WriteLine($"[VCarve.PhysicalSweep] bound={bound} paths={paths.Count} " +
            $"chords={chords} pieces={pieces} subdivisions={subdivisions} ignored={ignored} contours={result.Count} " +
            $"requestedTolerance={spatialTolerance:R} effectiveTolerance={tolerance:R} radialError={maximumRadialError:R} " +
            $"gridReserve={GridReserve:R} unionHierarchyDepth={hierarchyDepth} " +
            $"hullRoundingBound={(hierarchyDepth + 3) * Math.Sqrt(.5) / PathUtils.Scale:R} " +
            $"postUnionReserve={PostUnionReserve:R} postUnionArcTolerance={PostUnionArcTolerance:R} " +
            $"completeUncertaintyBound={uncertaintyBound:R} postUnionMs={postUnionMilliseconds:F1} " +
            $"surfaceZ={outputSurfaceZ:R} physicalProfile=True coordinates=provided retracts=False " +
            $"coverageAuthority={nominationUnion is null} wholeTargetCertificateRequired={nominationUnion is not null} " +
            $"flatRuns={flatRuns} flatChords={flatChords} flatChunks={flatChunks} offsetMs={offsetMilliseconds:F1} " +
            $"taperedRuns={taperedRuns} taperedChords={taperedChords} taperedFitMs={taperedFitMilliseconds:F3} " +
            $"taperedRequests={taperedWorkspace?.Metrics.Requests ?? 0} taperedSourcePoints={taperedWorkspace?.Metrics.SourcePoints ?? 0} " +
            $"taperedFitPoints={taperedWorkspace?.Metrics.FitPoints ?? 0} taperedCandidateFits={taperedWorkspace?.Metrics.CandidateFits ?? 0} " +
            $"taperedSphereBulges={taperedWorkspace?.Metrics.SphereBulges ?? 0} taperedEnclosureCalls={taperedWorkspace?.Metrics.EnclosureArithmeticCalls ?? 0} taperedSqrtChecks={taperedWorkspace?.Metrics.SqrtEnclosureChecks ?? 0} " +
            $"taperedTokenBarriers={taperedWorkspace?.Metrics.TokenBarriers ?? 0} " +
            $"taperedMaximumLoss={taperedWorkspace?.Metrics.MaximumAcceptedLoss ?? 0:R} " +
            $"taperedMaximumReduction={taperedWorkspace?.Metrics.MaximumAcceptedReduction ?? 0:R} " +
            $"taperedMaximumRadiusSpread={taperedWorkspace?.Metrics.MaximumAcceptedRadiusSpread ?? 0:R} " +
            $"taperedLastRejectedLoss={taperedWorkspace?.Metrics.LastRejectedLoss ?? 0:R} " +
            $"taperedRefusals={(taperedWorkspace is null ? "disabled" : string.Join(",", Enum.GetValues<TaperedRadiusRun.Refusal>().Select(reason => reason + ":" + taperedWorkspace.Metrics.Refusals[(int)reason])))} " +
            $"unionMs={unionMilliseconds:F1} orderedDiscHulls={orderedDiscHulls} " +
            $"serialLeafUnionMs={serialLeafUnionMilliseconds:F3} hierarchyFinalAndPostUnionMs={unionMilliseconds - serialLeafUnionMilliseconds:F3} " +
            $"parallelLeafUnionsRequested={enableParallelLeafUnions} parallelLeafUnionWorkers={parallelUnions?.WorkerCount ?? 0} " +
            $"parallelLeafUnionBatches={parallelUnions?.Submitted ?? 0} parallelLeafUnionLiteralFallbacks={parallelUnions?.LiteralFallbacks ?? 0} " +
            $"parallelLeafUnionComputeMs={parallelUnions?.ComputeMilliseconds ?? 0:F3} parallelLeafUnionWaitMs={parallelUnions?.WaitMilliseconds ?? 0:F3} " +
            $"parallelLeafUnionPeakInFlight={parallelUnions?.PeakInFlight ?? 0} parallelLeafUnionPeakInputBytes={parallelUnions?.PeakRetainedInputBytes ?? 0} " +
            $"parallelLeafUnionMaximumInputBytes={VCarveOrderedUnionQueue.MaximumRetainedInputBytes} unionMsScope=serial-union-only parallelComputeScope=sum-worker-inclusive " +
            $"hemisphereConsolidation={consolidateHemisphereChords && !upperBound} " +
            $"consolidationRequests={consolidationRequests} consolidationEligibleChords={consolidationEligible} " +
            $"consolidationProfileRefusals={consolidationProfileRefusals} consolidationRadiusRefusals={consolidationRadiusRefusals} " +
            $"consolidationPlanarRefusals={consolidationPlanarRefusals} " +
            $"consolidationWorldZRefusals={consolidationWorldZRefusals} consolidationDepthRefusals={consolidationDepthRefusals} " +
            $"consolidationJoinRefusals={consolidationJoinRefusals} " +
            $"consolidationTipReserveRefusals={consolidationTipRefusals} " +
            $"consolidatedChords={consolidatedChords} consolidatedPieces={consolidatedPieces} " +
            $"consolidatedWitnessVertices={consolidatedVertices} consolidationAuthority=single-affine-chord-concavity " +
            $"orderedHullCalls={orderedHulls?.Calls ?? 0} orderedHullVertices={orderedHulls?.Vertices ?? 0} " +
            $"orderedHullTieGroups={orderedHulls?.TieGroups ?? 0} orderedHullTiePairs={orderedHulls?.TiePairs ?? 0} " +
            $"orderedHullPairSwaps={orderedHulls?.PairSwaps ?? 0} orderedHullMaxTieGroup={orderedHulls?.MaximumTieGroup ?? 0} " +
            $"orderedHullSortFallbacks={orderedHulls?.SortFallbacks ?? 0} orderedHullDuplicates={orderedHulls?.DuplicatesSkipped ?? 0} " +
            $"orderedHullCircleOrderBuilds={orderedHulls?.CircleOrderBuilds ?? 0} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
        return result;
    }

    private static ConsolidationRefusal ConsolidationGuard(Point3D originalA, Point3D originalB,
        Point3D partA, Point3D partB, int sectionIndex, VCarveCutterProfile profile, double surfaceZ)
    {
        var sphere = profile.Sections[0];
        if (sectionIndex != 0 || sphere.Kind != CarvingCutterSectionKind.Hemisphere
            || sphere.BottomRadius != 0 || sphere.Height != sphere.TopRadius)
            return ConsolidationRefusal.Profile;
        if (!(sphere.TopRadius > 0) || !double.IsFinite(sphere.TopRadius)
            || sphere.TopRadius > ConsolidationMaximumSphereRadius)
            return ConsolidationRefusal.SphereRadius;
        static bool Ordinary(double value, double limit) => double.IsFinite(value) && Math.Abs(value) <= limit;
        if (!Ordinary(originalA.X, ConsolidationMaximumPlanarCoordinate)
            || !Ordinary(originalA.Y, ConsolidationMaximumPlanarCoordinate)
            || !Ordinary(originalB.X, ConsolidationMaximumPlanarCoordinate)
            || !Ordinary(originalB.Y, ConsolidationMaximumPlanarCoordinate)
            || !Ordinary(partA.X, ConsolidationMaximumPlanarCoordinate)
            || !Ordinary(partA.Y, ConsolidationMaximumPlanarCoordinate)
            || !Ordinary(partB.X, ConsolidationMaximumPlanarCoordinate)
            || !Ordinary(partB.Y, ConsolidationMaximumPlanarCoordinate))
            return ConsolidationRefusal.PlanarCoordinate;
        if (!Ordinary(surfaceZ, ConsolidationMaximumZCoordinate)
            || !Ordinary(originalA.Z, ConsolidationMaximumZCoordinate)
            || !Ordinary(originalB.Z, ConsolidationMaximumZCoordinate)
            || !Ordinary(partA.Z, ConsolidationMaximumZCoordinate)
            || !Ordinary(partB.Z, ConsolidationMaximumZCoordinate))
            return ConsolidationRefusal.WorldZ;
        double originalDepthA = surfaceZ - originalA.Z, originalDepthB = surfaceZ - originalB.Z;
        double depthA = surfaceZ - partA.Z, depthB = surfaceZ - partB.Z;
        if (!Ordinary(originalDepthA, ConsolidationMaximumDepth)
            || !Ordinary(originalDepthB, ConsolidationMaximumDepth)
            || !double.IsFinite(depthA + depthB) || Math.Min(depthA, depthB) < 0
            || Math.Max(depthA, depthB) > sphere.Height)
            return ConsolidationRefusal.Depth;
        // At an exactly rounded join, a later sub-ulp section height can leave
        // MaxRadiusBelow's accumulated start unchanged and expose a subsequent
        // larger BottomRadius. Arbitrarily steep later sections also make a
        // join's depth roundoff impossible to bound by the sphere alone. Keep
        // every actual/ideal sample strictly within the first section. The
        // outward comparisons pay the same <5e-12 interpolation allowance below;
        // ambiguous joins retain the original ordered pieces and physical model.
        double sphereInteriorEnd = Math.BitDecrement(sphere.Height - ConsolidationDepthRoundoff);
        if (Math.BitIncrement(Math.Max(depthA, depthB)) > sphereInteriorEnd)
            return ConsolidationRefusal.SectionJoin;
        // An unreserved snapped tip is retained by the original piece builder.
        // Consolidation requires real inset-disc vertices throughout its span.
        // Radius is nondecreasing in first-sphere depth, so this also ensures
        // Refine's tip truncation never introduces an unreserved witness here.
        if (profile.PhysicalRadiusAtDepth(Math.Min(depthA, depthB)) < 2 * GridReserve)
            return ConsolidationRefusal.TipReserve;
        // A vertex has at most 48 Refine interpolations, one section split and
        // one Emit interpolation. With |Z|<=100 their 50*8u*100 roundoff plus
        // surface subtraction is <5e-12 mm. The sphere's near-tip square-root
        // error is then <sqrt(2*100*5e-12+8u*100^2)<3.2e-5 mm. XY roundoff is
        // <50*8u*10000<4.5e-10 mm; lattice rounding is sqrt(.5)/Scale<2.2e-5 mm.
        // Their sum stays below the unchanged 4/Scale=1.2207e-4 mm reserve.
        // Refusal preserves the original ordered pieces, budgets and tolerances.
        return ConsolidationRefusal.None;
    }

    /// <summary>
    /// Encloses accumulated integer Boolean uncertainty in the correct direction.
    /// A tiny false interior hole has negligible area but can put a boundary deep
    /// inside a physically covered cutter disc. Vertex reserves alone do not
    /// remove that topological artifact. A bounded dilation/erosion handles both
    /// holes and slivers geometrically, without classifying them by their area.
    /// </summary>
    internal static Paths64 BoundBooleanResult(Paths64 source, bool upperBound)
    {
        if (source.Count == 0) return new Paths64();
        var offset = new ClipperOffset(miterLimit: 2,
            arcTolerance: PostUnionArcTolerance * PathUtils.Scale,
            preserveCollinear: false, reverseSolution: false);
        offset.AddPaths(source, JoinType.Round, EndType.Polygon);
        var enclosed = new Paths64();
        offset.Execute((upperBound ? PostUnionReserve : -PostUnionReserve) * PathUtils.Scale, enclosed);
        // Retain the known filled set on the outer side and forbid any collapsed
        // inward offset from adding material beyond the original lower set.
        return upperBound ? Clipper.Union(source, enclosed, FillRule.NonZero)
            : Clipper.Intersect(source, enclosed, FillRule.NonZero);
    }

    private static Point3D Interpolate(Point3D a, Point3D b, double t)
        => new(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);

    private static Path64 ConvexHull(List<Point64> points)
    {
        points.Sort((a, b) => a.X == b.X ? a.Y.CompareTo(b.Y) : a.X.CompareTo(b.X));
        var unique = points.Distinct().ToArray();
        if (unique.Length < 3) return new Path64(unique);
        static Int128 Cross(Point64 a, Point64 b, Point64 c)
            => (Int128)(b.X - a.X) * (c.Y - a.Y) - (Int128)(b.Y - a.Y) * (c.X - a.X);
        var hull = new Path64();
        foreach (var point in unique)
        {
            while (hull.Count >= 2 && Cross(hull[^2], hull[^1], point) <= 0) hull.RemoveAt(hull.Count - 1);
            hull.Add(point);
        }
        int lower = hull.Count;
        for (int i = unique.Length - 2; i >= 0; i--)
        {
            var point = unique[i];
            while (hull.Count > lower && Cross(hull[^2], hull[^1], point) <= 0) hull.RemoveAt(hull.Count - 1);
            hull.Add(point);
        }
        hull.RemoveAt(hull.Count - 1);
        return hull;
    }
}
