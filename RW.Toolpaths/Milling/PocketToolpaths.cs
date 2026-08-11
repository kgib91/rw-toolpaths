using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Configuration for concentric-offset pocket clearing.</summary>
public sealed record PocketOptions
{
    /// <summary>Radial engagement as a fraction of tool diameter; 0.4 removes 40% per pass.</summary>
    public double StepOver { get; init; } = 0.4;

    /// <summary>Material deliberately left on the walls for a later finishing pass.</summary>
    public double StockToLeave { get; init; }

    /// <summary>Work from the centre outward instead of from the wall inward.</summary>
    public bool InsideOut { get; init; }

    /// <summary>Cap on concentric rings per region.</summary>
    public int MaxRings { get; init; } = RingOffsetEngine.AbsoluteRingLimit;

    public MillingDirection Direction { get; init; } = MillingDirection.Climb;

    public RampSettings Ramp { get; init; } = RampSettings.FromRatio(2.0);

    public GeometryTolerances Tolerances { get; init; } = GeometryTolerances.Default;

    /// <summary>
    /// Cut nested rings as one continuous spiral rather than lifting between them.
    /// Turn off when something downstream needs each ring as a discrete loop.
    /// </summary>
    public bool SpiralChaining { get; init; } = true;

    /// <summary>Optional boundary that rings are trimmed to, normally the stock extents.</summary>
    public IReadOnlyList<PointD>? ClipBoundary { get; init; }

    /// <summary>
    /// Vets every ring-to-ring link before it is cut. Left null, pocketing derives one from the
    /// region it is clearing; supply an implementation to widen it with knowledge of what earlier
    /// operations already removed.
    /// </summary>
    public ILinkClearance? LinkClearance { get; init; }
}

/// <summary>Counters describing what a strategy produced, for logging and regression checks.</summary>
public sealed record ToolpathDiagnostics
{
    public int RingCount { get; init; }
    public int IslandCount { get; init; }
    public int PathCount { get; init; }
    public int DepthPassCount { get; init; }

    /// <summary>Total distance travelled while cutting.</summary>
    public double CutLength { get; init; }

    /// <summary>Straight-line distance between the end of one path and the start of the next.</summary>
    public double LinkLength { get; init; }

    /// <summary>Ring-to-ring links refused by the clearance test and turned into a lift.</summary>
    public int LinkLifts { get; init; }

    /// <summary>Potential ring-entry points checked by the link-clearance policy.</summary>
    public int LinkCandidatesTested { get; init; }

    /// <summary>Stay-down links recovered by selecting a safe point other than the nearest candidate.</summary>
    public int AlternativeLinks { get; init; }

    public int PlungeEntries { get; init; }
    public int LinearRampEntries { get; init; }
    public int HelicalEntries { get; init; }

    /// <summary>Closed profile loops taken through multiple scheduled depths without retracting.</summary>
    public int DepthFirstSpirals { get; init; }

    /// <summary>Total scheduled depth passes represented by those continuous loops.</summary>
    public int PassesCombined { get; init; }

    /// <summary>Between-pass retracts removed by continuous depth-first loops.</summary>
    public int AvoidedRetracts { get; init; }
}

/// <summary>Everything one strategy produced for one operation.</summary>
public sealed class ToolpathPlan
{
    internal ToolpathPlan(
        List<TaggedToolpath> toolpaths,
        List<List<PointD>> sweptArea,
        ToolpathDiagnostics diagnostics,
        HoldingTabReport? tabReport = null)
    {
        Toolpaths = toolpaths;
        SweptArea = sweptArea;
        Diagnostics = diagnostics;
        TabReport = tabReport;
    }

    /// <summary>Continuous paths in cut order. Callers insert their own rapids between them.</summary>
    public IReadOnlyList<TaggedToolpath> Toolpaths { get; }

    /// <summary>Area the cutter sweeps, for stock tracking by later operations.</summary>
    public IReadOnlyList<List<PointD>> SweptArea { get; }

    public ToolpathDiagnostics Diagnostics { get; }

    /// <summary>Holding-tab outcome; <c>null</c> when the operation does not use tabs.</summary>
    public HoldingTabReport? TabReport { get; }

    internal static ToolpathPlan Empty { get; } =
        new(new List<TaggedToolpath>(), new List<List<PointD>>(), new ToolpathDiagnostics());
}

/// <summary>
/// Clears the interior of closed regions with concentric offset rings.
///
/// <para>
/// Depth passes are nested inside islands rather than the other way round: an island is taken all
/// the way to final depth before the cutter moves on. Sweeping every island at one depth before
/// stepping down would mean re-entering each pocket once per pass, and chips would be pushed back
/// under the cutter on every re-entry.
/// </para>
/// </summary>
public static class PocketToolpaths
{
    /// <summary>Category tag applied to pocket passes.</summary>
    public const string Category = "pocket";

    public static ToolpathPlan Generate(
        IReadOnlyList<MillingRegion> regions,
        ToolGeometry tool,
        DepthSchedule depth,
        PocketOptions options,
        CancellationToken cancellationToken = default)
    {
        long t0 = PerfLog.Start();

        if (regions.Count == 0 || tool.Radius <= 1e-6 || depth.Depth <= 0)
            return ToolpathPlan.Empty;

        double stepOver = tool.Radius * 2.0 * options.StepOver;
        if (stepOver <= 1e-6)
            stepOver = tool.Radius;

        var rings = RingOffsetEngine.BuildRings(
            regions,
            new RingOffsetOptions(tool.Radius + options.StockToLeave, stepOver)
            {
                RingLimit = options.MaxRings,
                Tolerances = options.Tolerances,
            },
            cancellationToken);

        if (options.ClipBoundary is { Count: >= 3 })
            rings = ClipRings(rings, options.ClipBoundary);

        if (rings.Count == 0)
            return ToolpathPlan.Empty;

        RingOffsetEngine.BuildNesting(rings);
        var traversal = RingOffsetEngine.BuildTraversal(rings, options.InsideOut);

        var guarded = options with
        {
            LinkClearance = options.LinkClearance ?? BuildTravelClearance(rings),
        };

        var plan = Emit(rings, traversal, depth, guarded, tool, Category, cancellationToken);

        PerfLog.Stop("PocketToolpaths.Generate", t0,
            $"regions={regions.Count} rings={rings.Count} paths={plan.Toolpaths.Count} " +
            $"passes={depth.PassCount} cut={plan.Diagnostics.CutLength:F1} " +
            $"link={plan.Diagnostics.LinkLength:F1} lifts={plan.Diagnostics.LinkLifts}");

        return plan;
    }

    /// <summary>
    /// The cutter centre may go anywhere the outermost ring can be cut. Level-zero rings are
    /// already the region eroded by cutter radius plus stock to leave, including clipping, so
    /// re-offsetting the source here would repeat the most expensive geometry operation.
    /// </summary>
    private static ILinkClearance BuildTravelClearance(
        IReadOnlyList<OffsetRing> rings)
    {
        long t0 = PerfLog.Start();
        var travel = rings
            .Where(ring => ring.Level == 0)
            .Select(ring =>
            {
                var points = new List<PointD>(ring.Points);
                if (ring.IsHole)
                    points.Reverse();
                return points;
            })
            .ToList();

        // Union resolves overlap between source regions while preserving Clipper's hole winding.
        if (rings.Select(ring => ring.RegionIndex).Distinct().Take(2).Count() > 1)
            travel = PolygonOps.Union(travel);

        PerfLog.Stop("PocketToolpaths.BuildTravelClearance", t0,
            $"contours={travel.Count} holes={travel.Count(path => PolygonOps.SignedArea(path) < 0)}");
        return LinkClearance.WithinRegion(travel);
    }

    /// <summary>
    /// Walks islands in travel-optimised order, taking each one to full depth before moving on.
    /// Shared by pocketing and profiling; profiling supplies its own pre-built rings.
    /// </summary>
    internal static ToolpathPlan Emit(
        IReadOnlyList<OffsetRing> rings,
        RingTraversal traversal,
        DepthSchedule depth,
        PocketOptions options,
        ToolGeometry tool,
        string category,
        CancellationToken cancellationToken,
        Func<int, List<Point3D>, List<ToolpathSpan>, (List<Point3D> Points, List<ToolpathSpan> Spans)>? applyTabs = null)
    {
        long t0 = PerfLog.Start();
        var toolpaths = new List<TaggedToolpath>();

        double cutLength = 0;
        int emittedPointCount = 0;
        int plunges = 0, linearRamps = 0, helicals = 0;

        var islandOrder = OrderIslands(rings, traversal, cancellationToken);
        Point3D? previousEnd = null;
        double linkLength = 0;
        int linkLifts = 0;
        int linkCandidatesTested = 0;
        int alternativeLinks = 0;
        int radialDepthRamps = 0;
        int planarRingTransitions = 0;
        int straightTransitionFallbacks = 0;
        var routedRingIndices = new HashSet<int>();

        var regionRoutes = islandOrder
            .GroupBy(islandIndex =>
            {
                var (start, _) = traversal.Islands[islandIndex];
                return rings[traversal.Order[start]].RegionIndex;
            })
            .Select(group => group.ToList())
            .ToList();

        foreach (var regionIslands in regionRoutes)
        {
            cancellationToken.ThrowIfCancellationRequested();

            var chains = new List<RingChain>();
            foreach (int islandIndex in regionIslands)
            {
                var (start, count) = traversal.Islands[islandIndex];
                var islandRings = new List<int>(count);
                for (int i = 0; i < count; i++)
                    islandRings.Add(traversal.Order[start + i]);

                chains.AddRange(options.SpiralChaining
                    ? RingOffsetEngine.BuildChains(rings, islandRings)
                    : islandRings.Select(r => new RingChain(new List<int> { r })));
            }

            var reversedChains = chains
                .AsEnumerable()
                .Reverse()
                .Select(chain => new RingChain(chain.RingIndices
                    .AsEnumerable()
                    .Reverse()
                    .ToList()))
                .ToList();
            bool rampAcrossPassSeam = chains.Sum(chain => chain.RingIndices.Count) == 2;
            foreach (var chain in chains)
            {
                foreach (int ringIndex in chain.RingIndices)
                    routedRingIndices.Add(ringIndex);
            }
            Point3D? regionEnd = previousEnd;

            for (int pass = 0; pass < depth.PassCount; pass++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                double topZ = depth.PassTopZ(pass);
                double bottomZ = depth.PassBottomZ(pass);
                IReadOnlyList<RingChain> passChains = rampAcrossPassSeam || pass % 2 == 0
                    ? chains
                    : reversedChains;

                for (int chainIndex = 0; chainIndex < passChains.Count; chainIndex++)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    RingChain passChain = passChains[chainIndex];

                    foreach (var built in BuildChainPaths(
                                 rings, passChain, topZ, bottomZ, regionEnd,
                                 requireEntry: pass == 0 && chainIndex == 0 && previousEnd is not null,
                                 options, tool,
                                 ref linkLifts, ref linkCandidatesTested, ref alternativeLinks,
                                 ref radialDepthRamps, ref planarRingTransitions,
                                 ref straightTransitionFallbacks))
                    {
                        if (built.Points.Count < 2)
                            continue;

                        if (built.HasEntry)
                        {
                            switch (built.Strategy)
                            {
                                case RampStrategy.Helical: helicals++; break;
                                case RampStrategy.Linear: linearRamps++; break;
                                default: plunges++; break;
                            }
                        }

                        var points = built.Points;
                        var spans = built.Spans;
                        if (applyTabs is not null)
                            (points, spans) = applyTabs(passChain.RingIndices[0], points, spans);

                        if (points.Count < 2)
                            continue;

                        if (previousEnd is { } previous)
                        {
                            double dx = points[0].X - previous.X;
                            double dy = points[0].Y - previous.Y;
                            linkLength += Math.Sqrt(dx * dx + dy * dy);
                        }
                        previousEnd = points[^1];
                        regionEnd = points[^1];

                        cutLength += CutLength(points, spans);
                        emittedPointCount += points.Count;

                        toolpaths.Add(new TaggedToolpath(
                            points,
                            spans,
                            rings[passChain.RingIndices[0]].RegionIndex,
                            category,
                            pass,
                            visitIdentity: toolpaths.Count));
                    }
                }
            }
        }

        int routeExcursions = 0;
        double routeAirSaved = 0;
        if (toolpaths.Count > 1)
        {
            (toolpaths, routeExcursions, routeAirSaved) = InterleaveNearbyRoutes(
                toolpaths,
                depth.PassCount);
            linkLength = BoundaryLinkLength(toolpaths);
            emittedPointCount = toolpaths.Sum(path => path.Points.Count);
        }

        long sweptAreaStart = PerfLog.Start();
        var sweptCenterlines = rings
            .Select(ring => (IReadOnlyList<PointD>)ring.Points)
            .ToList();
        var linkKeys = new HashSet<(long StartX, long StartY, long EndX, long EndY)>();
        int linkCenterlineCount = 0;
        foreach (var toolpath in toolpaths)
        {
            foreach (var span in toolpath.Spans.Where(span => span.Kind == ToolpathSpanKind.Link))
            {
                for (int pointIndex = span.StartIndex + 1;
                     pointIndex <= span.EndIndex && pointIndex < toolpath.Points.Count;
                     pointIndex++)
                {
                    Point3D start = toolpath.Points[pointIndex - 1];
                    Point3D end = toolpath.Points[pointIndex];
                    long startX = (long)Math.Round(start.X * PathUtils.Scale);
                    long startY = (long)Math.Round(start.Y * PathUtils.Scale);
                    long endX = (long)Math.Round(end.X * PathUtils.Scale);
                    long endY = (long)Math.Round(end.Y * PathUtils.Scale);
                    var key = (startX, startY, endX, endY);
                    var reverseKey = (endX, endY, startX, startY);
                    if (linkKeys.Contains(key) || linkKeys.Contains(reverseKey))
                        continue;

                    linkKeys.Add(key);
                    sweptCenterlines.Add(new List<PointD>
                    {
                        new(start.X, start.Y),
                        new(end.X, end.Y),
                    });
                    linkCenterlineCount++;
                }
            }
        }

        int bufferedPointCount = sweptCenterlines.Sum(centerline => centerline.Count);
        var sweptArea = toolpaths.Count > 0
            ? PolygonOps.BufferCenterlines(
                sweptCenterlines,
                tool.Radius,
                options.Tolerances.ArcTolerance)
            : new List<List<PointD>>();
        PerfLog.Stop("PocketToolpaths.SweptArea", sweptAreaStart,
            $"rings={rings.Count} links={linkCenterlineCount} emittedPoints={emittedPointCount} " +
            $"bufferedPoints={bufferedPointCount} polygons={sweptArea.Count}");

        var diagnostics = new ToolpathDiagnostics
        {
            RingCount = rings.Count,
            IslandCount = traversal.Islands.Count,
            PathCount = toolpaths.Count,
            DepthPassCount = depth.PassCount,
            CutLength = cutLength,
            LinkLength = linkLength,
            LinkLifts = linkLifts,
            LinkCandidatesTested = linkCandidatesTested,
            AlternativeLinks = alternativeLinks,
            PlungeEntries = plunges,
            LinearRampEntries = linearRamps,
            HelicalEntries = helicals,
        };

        string levelTrace = string.Join(",", rings
            .GroupBy(ring => ring.Level)
            .OrderBy(group => group.Key)
            .Take(32)
            .Select(group => $"{group.Key}:{group.Count()}"));
        int missingRings = rings.Count - routedRingIndices.Count;
        PerfLog.Stop("PocketToolpaths.Emit", t0,
            $"category={category} paths={toolpaths.Count} links={linkLength:F3} lifts={linkLifts} " +
            $"routedRings={routedRingIndices.Count}/{rings.Count} missingRings={missingRings} " +
            $"levels=[{levelTrace}] " +
            $"routeExcursions={routeExcursions} routeAirSaved={routeAirSaved:F3} " +
            $"linkCandidates={linkCandidatesTested} alternatives={alternativeLinks} " +
            $"radialDepthRamps={radialDepthRamps} " +
            $"planarRingTransitions={planarRingTransitions} " +
            $"straightTransitionFallbacks={straightTransitionFallbacks}");

        return new ToolpathPlan(toolpaths, sweptArea, diagnostics);
    }

    private sealed class RouteBlock
    {
        internal RouteBlock(
            int routeId,
            IReadOnlyList<TaggedToolpath> paths,
            double cutLength)
        {
            RouteId = routeId;
            Paths = paths;
            CutLength = cutLength;
        }

        internal int RouteId { get; }
        internal IReadOnlyList<TaggedToolpath> Paths { get; }
        internal double CutLength { get; }
        internal List<RouteAttachment> Attachments { get; } = new();
    }

    private sealed record RouteAttachment(
        RouteBlock Child,
        int HostPathIndex,
        RoutePause Pause);

    private readonly record struct RoutePause(
        int SegmentEndIndex,
        double SegmentPosition,
        double ExcursionLength);

    private static (List<TaggedToolpath> Paths, int Excursions, double AirSaved)
        InterleaveNearbyRoutes(
            IReadOnlyList<TaggedToolpath> source,
            int scheduledPasses)
    {
        List<RouteBlock> routes = BuildRouteBlocks(source);
        var attachedRoutes = new HashSet<int>();
        foreach (RouteBlock guest in routes.OrderBy(route => route.CutLength))
        {
            if (!IsSpiralRoute(guest, scheduledPasses))
            {
                continue;
            }

            Point3D guestStart = guest.Paths[0].Points[0];
            Point3D guestEnd = guest.Paths[^1].Points[^1];
            (RouteBlock Host, int PathIndex, RoutePause Pause)? best = null;
            foreach (RouteBlock host in routes)
            {
                if (host.RouteId == guest.RouteId
                    || !IsCarrierRoute(host, guest))
                {
                    continue;
                }

                for (int pathIndex = 0; pathIndex < host.Paths.Count; pathIndex++)
                {
                    if (!TryFindRoutePause(
                            host.Paths[pathIndex],
                            guestStart,
                            guestEnd,
                            out RoutePause pause)
                        || (best is not null
                            && pause.ExcursionLength >= best.Value.Pause.ExcursionLength))
                    {
                        continue;
                    }

                    best = (host, pathIndex, pause);
                }
            }

            if (best is null)
                continue;

            best.Value.Host.Attachments.Add(new RouteAttachment(
                guest,
                best.Value.PathIndex,
                best.Value.Pause));
            attachedRoutes.Add(guest.RouteId);
        }

        var output = new List<TaggedToolpath>();
        foreach (RouteBlock root in routes.Where(route => !attachedRoutes.Contains(route.RouteId)))
            EmitRouteBlock(root, output);

        double airSaved = BoundaryLinkLength(source) - BoundaryLinkLength(output);
        return (output, attachedRoutes.Count, airSaved);
    }

    private static List<RouteBlock> BuildRouteBlocks(IReadOnlyList<TaggedToolpath> source)
    {
        var routes = new List<RouteBlock>();
        var paths = new List<TaggedToolpath>();
        foreach (TaggedToolpath path in source)
        {
            if (paths.Count > 0
                && !PointsCoincide(paths[^1].Points[^1], path.Points[0]))
            {
                AddRoute();
            }
            paths.Add(path);
        }
        AddRoute();
        return routes;

        void AddRoute()
        {
            if (paths.Count == 0)
                return;
            var routePaths = paths.ToList();
            routes.Add(new RouteBlock(
                routes.Count,
                routePaths,
                routePaths.Sum(path => CutLength(path.Points, path.Spans))));
            paths.Clear();
        }
    }

    private static bool IsSpiralRoute(RouteBlock route, int scheduledPasses)
    {
        if (route.Paths.Count == 0
            || !IsRampedEntry(route.Paths[0])
            || route.Paths.Any(path => path.Spans.Any(span => span.Kind == ToolpathSpanKind.Link)))
        {
            return false;
        }

        var passes = route.Paths
            .Select(path => path.DepthPassIndex)
            .Where(pass => pass.HasValue)
            .Select(pass => pass!.Value)
            .Distinct()
            .Order()
            .ToArray();
        return passes.SequenceEqual(Enumerable.Range(0, scheduledPasses));
    }

    private static bool IsCarrierRoute(RouteBlock host, RouteBlock guest)
        => host.CutLength > guest.CutLength + 1e-6
            && (host.CutLength >= guest.CutLength * 2
                || host.Paths.Any(path => path.Spans.Any(
                    span => span.Kind == ToolpathSpanKind.Link)));

    private static void EmitRouteBlock(RouteBlock route, List<TaggedToolpath> output)
    {
        for (int pathIndex = 0; pathIndex < route.Paths.Count; pathIndex++)
        {
            TaggedToolpath path = route.Paths[pathIndex];
            List<RouteAttachment> attachments = route.Attachments
                .Where(attachment => attachment.HostPathIndex == pathIndex)
                .ToList();
            if (attachments.Count == 0)
            {
                output.Add(path);
                continue;
            }

            TaggedToolpath withPauses = InsertRoutePauses(
                path,
                attachments,
                out List<(RouteAttachment Attachment, int PointIndex)> pauses);
            int cursor = 0;
            foreach (var pause in pauses.OrderBy(entry => entry.PointIndex))
            {
                if (pause.PointIndex > cursor)
                    output.Add(SliceRoute(withPauses, cursor, pause.PointIndex));
                EmitRouteBlock(pause.Attachment.Child, output);
                cursor = pause.PointIndex;
            }
            if (cursor < withPauses.Points.Count - 1)
                output.Add(SliceRoute(withPauses, cursor, withPauses.Points.Count - 1));
        }
    }

    private static TaggedToolpath InsertRoutePauses(
        TaggedToolpath source,
        IReadOnlyList<RouteAttachment> attachments,
        out List<(RouteAttachment Attachment, int PointIndex)> pauses)
    {
        var points = new List<Point3D>(source.Points.Count + attachments.Count);
        var moveKinds = new List<ToolpathSpanKind>(source.Points.Count + attachments.Count);
        pauses = new List<(RouteAttachment Attachment, int PointIndex)>(attachments.Count);

        points.Add(source.Points[0]);
        var bySegment = attachments
            .GroupBy(attachment => attachment.Pause.SegmentEndIndex)
            .ToDictionary(
                group => group.Key,
                group => group.OrderBy(attachment => attachment.Pause.SegmentPosition).ToList());

        for (int segmentEnd = 1; segmentEnd < source.Points.Count; segmentEnd++)
        {
            Point3D start = source.Points[segmentEnd - 1];
            Point3D end = source.Points[segmentEnd];
            ToolpathSpanKind kind = MoveKindAt(source.Spans, segmentEnd);
            if (bySegment.TryGetValue(segmentEnd, out var segmentPauses))
            {
                foreach (RouteAttachment attachment in segmentPauses)
                {
                    double position = attachment.Pause.SegmentPosition;
                    if (position <= 1e-7)
                    {
                        pauses.Add((attachment, points.Count - 1));
                        continue;
                    }
                    if (position >= 1 - 1e-7)
                        continue;

                    points.Add(Interpolate(start, end, position));
                    moveKinds.Add(kind);
                    pauses.Add((attachment, points.Count - 1));
                }
            }

            points.Add(end);
            moveKinds.Add(kind);
            if (bySegment.TryGetValue(segmentEnd, out segmentPauses))
            {
                foreach (RouteAttachment attachment in segmentPauses)
                {
                    if (attachment.Pause.SegmentPosition >= 1 - 1e-7)
                        pauses.Add((attachment, points.Count - 1));
                }
            }
        }

        return new TaggedToolpath(
            points,
            BuildSpansFromMoves(moveKinds),
            source.RegionIndex,
            source.Category,
            source.DepthPassIndex,
            source.VisitIdentity);
    }

    private static ToolpathSpanKind MoveKindAt(
        IReadOnlyList<ToolpathSpan> spans,
        int moveEndIndex)
    {
        foreach (ToolpathSpan span in spans)
        {
            if (moveEndIndex > span.StartIndex && moveEndIndex <= span.EndIndex)
                return span.Kind;
        }
        return ToolpathSpanKind.Cut;
    }

    private static List<ToolpathSpan> BuildSpansFromMoves(
        IReadOnlyList<ToolpathSpanKind> moveKinds)
    {
        var spans = new List<ToolpathSpan>();
        if (moveKinds.Count == 0)
            return spans;

        int start = 0;
        ToolpathSpanKind kind = moveKinds[0];
        for (int move = 1; move < moveKinds.Count; move++)
        {
            if (moveKinds[move] == kind)
                continue;

            spans.Add(new ToolpathSpan(start, move, kind));
            start = move;
            kind = moveKinds[move];
        }
        spans.Add(new ToolpathSpan(start, moveKinds.Count, kind));
        return spans;
    }

    private static bool IsRampedEntry(TaggedToolpath path)
        => path.Points.Count >= 2
            && path.Spans.Any(span => span.Kind == ToolpathSpanKind.Ramp)
            && path.Spans.Any(span => span.Kind == ToolpathSpanKind.Cut);

    private static bool TryFindRoutePause(
        TaggedToolpath host,
        Point3D guestStart,
        Point3D guestEnd,
        out RoutePause pause)
    {
        pause = default;
        double best = double.MaxValue;
        foreach (ToolpathSpan span in host.Spans)
        {
            if (span.Kind != ToolpathSpanKind.Cut)
                continue;

            int firstEnd = Math.Max(1, span.StartIndex + 1);
            int lastEnd = Math.Min(host.Points.Count - 1, span.EndIndex);
            for (int segmentEnd = firstEnd; segmentEnd <= lastEnd; segmentEnd++)
            {
                Point3D start = host.Points[segmentEnd - 1];
                Point3D end = host.Points[segmentEnd];
                double position = MinimizeExcursion(start, end, guestStart, guestEnd);
                if (position <= 1e-7 && segmentEnd == 1
                    || position >= 1 - 1e-7 && segmentEnd == host.Points.Count - 1)
                {
                    continue;
                }
                Point3D candidate = Interpolate(start, end, position);
                double length = PlanarDistance(candidate, guestStart)
                    + PlanarDistance(guestEnd, candidate);
                if (length >= best)
                    continue;

                best = length;
                pause = new RoutePause(segmentEnd, position, length);
            }
        }
        return best < double.MaxValue;
    }

    private static double MinimizeExcursion(
        Point3D segmentStart,
        Point3D segmentEnd,
        Point3D guestStart,
        Point3D guestEnd)
    {
        double low = 0;
        double high = 1;
        for (int iteration = 0; iteration < 32; iteration++)
        {
            double first = (low * 2 + high) / 3;
            double second = (low + high * 2) / 3;
            double firstCost = ExcursionAt(first);
            double secondCost = ExcursionAt(second);
            if (firstCost <= secondCost)
                high = second;
            else
                low = first;
        }
        return (low + high) * 0.5;

        double ExcursionAt(double position)
        {
            Point3D point = Interpolate(segmentStart, segmentEnd, position);
            return PlanarDistance(point, guestStart) + PlanarDistance(guestEnd, point);
        }
    }

    private static TaggedToolpath SliceRoute(TaggedToolpath source, int start, int end)
    {
        var points = source.Points.Skip(start).Take(end - start + 1).ToList();
        var spans = new List<ToolpathSpan>();
        foreach (ToolpathSpan span in source.Spans)
        {
            int spanStart = Math.Max(start, span.StartIndex);
            int spanEnd = Math.Min(end, span.EndIndex);
            if (spanEnd <= spanStart)
                continue;
            spans.Add(new ToolpathSpan(
                spanStart - start,
                spanEnd - start,
                span.Kind));
        }
        return new TaggedToolpath(
            points,
            spans,
            source.RegionIndex,
            source.Category,
            source.DepthPassIndex,
            source.VisitIdentity);
    }

    private static double BoundaryLinkLength(IReadOnlyList<TaggedToolpath> paths)
    {
        double total = 0;
        for (int index = 1; index < paths.Count; index++)
            total += PlanarDistance(paths[index - 1].Points[^1], paths[index].Points[0]);
        return total;
    }

    private static bool PointsCoincide(Point3D first, Point3D second)
        => Math.Abs(first.X - second.X) <= 1e-6
            && Math.Abs(first.Y - second.Y) <= 1e-6
            && Math.Abs(first.Z - second.Z) <= 1e-6;

    private static Point3D Interpolate(Point3D start, Point3D end, double position)
        => new(
            start.X + (end.X - start.X) * position,
            start.Y + (end.Y - start.Y) * position,
            start.Z + (end.Z - start.Z) * position);

    private static double PlanarDistance(Point3D first, Point3D second)
    {
        double dx = second.X - first.X;
        double dy = second.Y - first.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>Sequences islands so the cutter crosses as little empty space as possible.</summary>
    private static int[] OrderIslands(
        IReadOnlyList<OffsetRing> rings,
        RingTraversal traversal,
        CancellationToken cancellationToken)
    {
        if (traversal.Islands.Count <= 1)
            return traversal.Islands.Count == 0 ? Array.Empty<int>() : new[] { 0 };

        long t0 = PerfLog.Start();
        var islandsByComponent = new Dictionary<int, List<int>>();
        var componentIndices = new List<int>();
        for (int islandIndex = 0; islandIndex < traversal.Islands.Count; islandIndex++)
        {
            int componentIndex = rings[traversal.Order[traversal.Islands[islandIndex].StartIndex]].RegionIndex;
            if (!islandsByComponent.TryGetValue(componentIndex, out var componentIslands))
            {
                componentIslands = new List<int>();
                islandsByComponent.Add(componentIndex, componentIslands);
                componentIndices.Add(componentIndex);
            }

            componentIslands.Add(islandIndex);
        }

        // The outer-to-outer hop between decals may be shorter than the hop to a cutout in the
        // current decal, but leaving a decal half-cut costs a long rapid to return. Optimise the
        // decal groups first, then optimise their local contours without allowing re-entry.
        var componentBoundaries = new List<IReadOnlyList<PointD>>(componentIndices.Count);
        foreach (int componentIndex in componentIndices)
        {
            var boundary = new List<PointD>();
            foreach (int islandIndex in islandsByComponent[componentIndex])
                boundary.AddRange(rings[traversal.Order[traversal.Islands[islandIndex].StartIndex]].Points);
            componentBoundaries.Add(boundary);
        }

        var optimizedComponents = TravelOptimizer.Order(
            componentBoundaries, start: null, label: "pocket-components", cancellationToken);
        var order = new List<int>(traversal.Islands.Count);
        var componentOrder = new List<int>(optimizedComponents.Length);
        foreach (int componentPosition in optimizedComponents)
        {
            cancellationToken.ThrowIfCancellationRequested();

            int componentIndex = componentIndices[componentPosition];
            componentOrder.Add(componentIndex);
            var componentIslands = islandsByComponent[componentIndex];
            if (componentIslands.Count == 1)
            {
                order.Add(componentIslands[0]);
                continue;
            }

            var islandBoundaries = new List<IReadOnlyList<PointD>>(componentIslands.Count);
            foreach (int islandIndex in componentIslands)
                islandBoundaries.Add(rings[traversal.Order[traversal.Islands[islandIndex].StartIndex]].Points);

            foreach (int islandPosition in TravelOptimizer.Order(
                         islandBoundaries, start: null, $"pocket-component-{componentIndex}", cancellationToken))
            {
                order.Add(componentIslands[islandPosition]);
            }
        }

        string componentTrace = string.Join(",", componentOrder.Take(16));
        if (componentOrder.Count > 16)
            componentTrace += ",...";

        PerfLog.Stop("PocketToolpaths.OrderIslands", t0,
            $"islands={traversal.Islands.Count} components={componentOrder.Count} reentries=0 " +
            $"componentOrder=[{componentTrace}]");

        return order.ToArray();
    }

    private readonly record struct ChainPath(
        List<Point3D> Points,
        List<ToolpathSpan> Spans,
        RampStrategy Strategy,
        bool HasEntry);

    private readonly record struct LinkCandidate(
        PointD Point,
        int SegmentIndex,
        double SegmentPosition,
        double DistanceSquared);

    /// <summary>
    /// Walks a chain: ramp into the first ring, then step through the rest at depth, rotating
    /// each so it starts where the previous one finished.
    ///
    /// <para>
    /// A step is only taken when the clearance test can prove the straight move removes nothing
    /// the operation has to leave standing. Where it cannot, the chain is cut in two and the
    /// second half ramps in afresh, which costs a lift but never drags the cutter through an
    /// island. Ordering and nesting are heuristics, so this is what keeps a bad guess from
    /// becoming a scrapped part.
    /// </para>
    /// </summary>
    private static List<ChainPath> BuildChainPaths(
        IReadOnlyList<OffsetRing> rings,
        RingChain chain,
        double topZ,
        double bottomZ,
        Point3D? entryTarget,
        bool requireEntry,
        PocketOptions options,
        ToolGeometry tool,
        ref int linkLifts,
        ref int linkCandidatesTested,
        ref int alternativeLinks,
        ref int radialDepthRamps,
        ref int planarRingTransitions,
        ref int straightTransitionFallbacks)
    {
        var clearance = options.LinkClearance ?? LinkClearance.Unrestricted;
        var paths = new List<ChainPath>(1);

        var firstRing = RingOffsetEngine.Orient(rings[chain.RingIndices[0]].Points, options.Direction);
        bool hasEntry = true;
        bool linkAtDepth = false;
        bool continuesFromPreviousPass = false;
        bool rampFromPreviousPass = false;
        double ringEntryZ = topZ;
        if (entryTarget is { } target)
        {
            if (requireEntry)
            {
                linkLifts++;
                firstRing = PathUtils.RebaseNear(
                    firstRing,
                    new PointD(target.X, target.Y));
            }
            else
            {
                bool continuesPreviousPass = Math.Abs(target.Z - topZ) <= 1e-6;
                double entryZ = continuesPreviousPass ? topZ : bottomZ;
                if (TryFindSafeRebase(
                        firstRing,
                        target,
                        entryZ,
                        clearance,
                        tool.Radius,
                        out var entryRing,
                        out int candidatesTested))
                {
                    firstRing = entryRing;
                    linkAtDepth = !continuesPreviousPass;
                    hasEntry = !linkAtDepth;
                    linkCandidatesTested += candidatesTested;
                    if (candidatesTested > 1)
                        alternativeLinks++;

                    if (continuesPreviousPass)
                    {
                        continuesFromPreviousPass = true;
                        double dx = firstRing[0].x - target.X;
                        double dy = firstRing[0].y - target.Y;
                        double connectorLength = Math.Sqrt(dx * dx + dy * dy);
                        double connectorDrop = RampDropForDistance(
                            options.Ramp,
                            connectorLength,
                            topZ - bottomZ);
                        ringEntryZ = topZ - connectorDrop;
                        rampFromPreviousPass = connectorDrop > 1e-9;
                        if (rampFromPreviousPass && connectorLength > 1e-9)
                            radialDepthRamps++;
                    }
                }
                else
                {
                    linkCandidatesTested += candidatesTested;
                    linkLifts++;
                    firstRing = PathUtils.RebaseNear(
                        firstRing,
                        new PointD(target.X, target.Y));
                }
            }
        }
        var entry = RampPlanner.PlanClosedRing(
            firstRing,
            linkAtDepth ? bottomZ : ringEntryZ,
            bottomZ,
            options.Ramp);

        var points = entry.Points;
        var spans = entry.Spans;
        var strategy = rampFromPreviousPass ? RampStrategy.Linear : entry.Strategy;
        if (entryTarget is { } transitionStart
            && (linkAtDepth || continuesFromPreviousPass))
        {
            PrependTransition(
                points,
                spans,
                transitionStart,
                rampFromPreviousPass ? ToolpathSpanKind.Ramp : ToolpathSpanKind.Link,
                clearance,
                tool.Radius,
                ref planarRingTransitions,
                ref straightTransitionFallbacks);
        }

        for (int i = 1; i < chain.RingIndices.Count; i++)
        {
            var ring = RingOffsetEngine.Orient(rings[chain.RingIndices[i]].Points, options.Direction);

            if (points.Count > 0)
            {
                var last = points[^1];
                if (TryFindSafeRebase(
                        ring,
                        last,
                        bottomZ,
                        clearance,
                        tool.Radius,
                        out var rebased,
                        out int candidatesTested))
                {
                    linkCandidatesTested += candidatesTested;
                    if (candidatesTested > 1)
                        alternativeLinks++;
                    AppendRingAtDepth(
                        points,
                        spans,
                        rebased,
                        bottomZ,
                        clearance,
                        tool.Radius,
                        ref planarRingTransitions,
                        ref straightTransitionFallbacks);
                    continue;
                }

                linkCandidatesTested += candidatesTested;
                rebased = PathUtils.RebaseNear(ring, new PointD(last.X, last.Y));

                linkLifts++;
                paths.Add(new ChainPath(points, spans, strategy, hasEntry));

                var reentry = RampPlanner.PlanClosedRing(rebased, topZ, bottomZ, options.Ramp);
                points = reentry.Points;
                spans = reentry.Spans;
                strategy = reentry.Strategy;
                hasEntry = true;
                continue;
            }

            AppendRingAtDepth(
                points,
                spans,
                ring,
                bottomZ,
                clearance,
                tool.Radius,
                ref planarRingTransitions,
                ref straightTransitionFallbacks);
        }

        paths.Add(new ChainPath(points, spans, strategy, hasEntry));
        return paths;
    }

    private static void PrependTransition(
        List<Point3D> points,
        List<ToolpathSpan> spans,
        Point3D from,
        ToolpathSpanKind kind,
        ILinkClearance clearance,
        double toolRadius,
        ref int planarRingTransitions,
        ref int straightTransitionFallbacks)
    {
        if (points.Count == 0)
            return;

        Point3D first = points[0];
        if (Math.Abs(first.X - from.X) <= 1e-9
            && Math.Abs(first.Y - from.Y) <= 1e-9
            && Math.Abs(first.Z - from.Z) <= 1e-9)
        {
            points[0] = from;
            return;
        }

        Point3D? destinationNext = points.Count > 1 ? points[1] : null;
        bool smoothed = TryBuildPlanarTransition(
            from,
            sourcePrevious: null,
            first,
            destinationNext,
            clearance,
            toolRadius,
            out var transition);
        if (smoothed)
            planarRingTransitions++;
        else
            straightTransitionFallbacks++;

        int insertedCount = smoothed ? transition.Count - 1 : 1;
        if (smoothed)
            points.InsertRange(0, transition.Take(transition.Count - 1));
        else
            points.Insert(0, from);
        for (int spanIndex = 0; spanIndex < spans.Count; spanIndex++)
        {
            ToolpathSpan span = spans[spanIndex];
            spans[spanIndex] = new ToolpathSpan(
                span.StartIndex + insertedCount,
                span.EndIndex + insertedCount,
                span.Kind);
        }
        spans.Insert(0, new ToolpathSpan(0, insertedCount, kind));
    }

    private static bool TryBuildPlanarTransition(
        Point3D from,
        Point3D? sourcePrevious,
        Point3D to,
        Point3D? destinationNext,
        ILinkClearance clearance,
        double toolRadius,
        out List<Point3D> transition)
    {
        transition = new List<Point3D> { from, to };
        if (PlanarDistance(from, to) <= 1e-6)
            return false;

        return clearance.IsTravelSafe(from, to, toolRadius);
    }

    private static double RampDropForDistance(
        RampSettings settings,
        double horizontalDistance,
        double requestedDrop)
    {
        if (settings.Strategy == RampStrategy.Plunge
            || settings.AngleRadians <= 0
            || horizontalDistance <= 1e-9
            || requestedDrop <= 1e-9)
        {
            return 0;
        }

        return Math.Min(requestedDrop, horizontalDistance * Math.Tan(settings.AngleRadians));
    }

    internal static bool TryFindSafeRebase(
        IReadOnlyList<PointD> ring,
        Point3D from,
        double bottomZ,
        ILinkClearance clearance,
        double toolRadius,
        out List<PointD> rebased,
        out int candidatesTested)
    {
        var loop = RampPlanner.Normalize(ring);
        rebased = loop;
        candidatesTested = 0;
        if (loop.Count < 2)
            return false;

        var candidates = new List<LinkCandidate>(loop.Count * 2);
        var seen = new HashSet<(long X, long Y)>();
        for (int segmentIndex = 0; segmentIndex < loop.Count; segmentIndex++)
        {
            PointD start = loop[segmentIndex];
            PointD end = loop[(segmentIndex + 1) % loop.Count];
            AddCandidate(start, segmentIndex, 0);

            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double lengthSquared = dx * dx + dy * dy;
            if (lengthSquared <= 1e-18)
                continue;

            double position = Math.Clamp(
                ((from.X - start.x) * dx + (from.Y - start.y) * dy) / lengthSquared,
                0,
                1);
            AddCandidate(
                new PointD(start.x + dx * position, start.y + dy * position),
                segmentIndex,
                position);
        }

        candidates.Sort((first, second) =>
        {
            int comparison = first.DistanceSquared.CompareTo(second.DistanceSquared);
            if (comparison != 0)
                return comparison;
            comparison = first.SegmentIndex.CompareTo(second.SegmentIndex);
            return comparison != 0
                ? comparison
                : first.SegmentPosition.CompareTo(second.SegmentPosition);
        });

        foreach (LinkCandidate candidate in candidates)
        {
            candidatesTested++;
            if (!clearance.IsTravelSafe(
                    from,
                    new Point3D(candidate.Point.x, candidate.Point.y, bottomZ),
                    toolRadius))
            {
                continue;
            }

            rebased = RebaseAtCandidate(loop, candidate);
            return true;
        }

        return false;

        void AddCandidate(PointD point, int segmentIndex, double segmentPosition)
        {
            var key = (
                (long)Math.Round(point.x * PathUtils.Scale),
                (long)Math.Round(point.y * PathUtils.Scale));
            if (!seen.Add(key))
                return;

            double pointDx = point.x - from.X;
            double pointDy = point.y - from.Y;
            candidates.Add(new LinkCandidate(
                point,
                segmentIndex,
                segmentPosition,
                pointDx * pointDx + pointDy * pointDy));
        }
    }

    private static List<PointD> RebaseAtCandidate(
        IReadOnlyList<PointD> ring,
        LinkCandidate candidate)
    {
        int startIndex;
        bool insertPoint;
        if (candidate.SegmentPosition <= 1e-12)
        {
            startIndex = candidate.SegmentIndex;
            insertPoint = false;
        }
        else if (candidate.SegmentPosition >= 1 - 1e-12)
        {
            startIndex = (candidate.SegmentIndex + 1) % ring.Count;
            insertPoint = false;
        }
        else
        {
            startIndex = (candidate.SegmentIndex + 1) % ring.Count;
            insertPoint = true;
        }

        var rebased = new List<PointD>(ring.Count + (insertPoint ? 1 : 0));
        if (insertPoint)
            rebased.Add(candidate.Point);
        for (int offset = 0; offset < ring.Count; offset++)
            rebased.Add(ring[(startIndex + offset) % ring.Count]);
        return rebased;
    }

    private static void AppendRingAtDepth(
        List<Point3D> points,
        List<ToolpathSpan> spans,
        IReadOnlyList<PointD> ring,
        double bottomZ,
        ILinkClearance clearance,
        double toolRadius,
        ref int planarRingTransitions,
        ref int straightTransitionFallbacks)
    {
        if (ring.Count == 0)
            return;

        int linkStart = points.Count - 1;
        var ringStart = new Point3D(ring[0].x, ring[0].y, bottomZ);
        int cutStart = 0;
        if (linkStart >= 0)
        {
            Point3D? sourcePrevious = points.Count > 1 ? points[^2] : null;
            Point3D? destinationNext = ring.Count > 1
                ? new Point3D(ring[1].x, ring[1].y, bottomZ)
                : null;
            bool smoothed = TryBuildPlanarTransition(
                points[^1],
                sourcePrevious,
                ringStart,
                destinationNext,
                clearance,
                toolRadius,
                out var transition);
            if (smoothed)
            {
                planarRingTransitions++;
                points.AddRange(transition.Skip(1));
            }
            else
            {
                straightTransitionFallbacks++;
                points.Add(ringStart);
            }

            cutStart = points.Count - 1;
            spans.Add(new ToolpathSpan(linkStart, cutStart, ToolpathSpanKind.Link));
        }
        else
        {
            points.Add(ringStart);
        }

        for (int pointIndex = 1; pointIndex < ring.Count; pointIndex++)
        {
            PointD point = ring[pointIndex];
            points.Add(new Point3D(point.x, point.y, bottomZ));
        }
        points.Add(new Point3D(ring[0].x, ring[0].y, bottomZ));

        spans.Add(new ToolpathSpan(cutStart, points.Count - 1, ToolpathSpanKind.Cut));
    }

    /// <summary>Trims rings to a boundary, preserving each ring's level and region.</summary>
    private static List<OffsetRing> ClipRings(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<PointD> boundary)
    {
        var clipped = new List<OffsetRing>(rings.Count);
        foreach (var ring in rings)
        {
            foreach (var fragment in PolygonOps.Clip(new[] { ring.Points }, boundary))
            {
                if (fragment.Count >= 3)
                    clipped.Add(RingOffsetEngine.CreateRing(
                        fragment, ring.Level, ring.RegionIndex, ring.IsHole));
            }
        }
        return clipped;
    }

    private static double CutLength(IReadOnlyList<Point3D> points, IReadOnlyList<ToolpathSpan> spans)
    {
        double total = 0;
        foreach (var span in spans)
        {
            if (span.Kind is not (ToolpathSpanKind.Cut or ToolpathSpanKind.Link))
                continue;

            for (int i = span.StartIndex + 1; i <= span.EndIndex && i < points.Count; i++)
            {
                double dx = points[i].X - points[i - 1].X;
                double dy = points[i].Y - points[i - 1].Y;
                total += Math.Sqrt(dx * dx + dy * dy);
            }
        }
        return total;
    }
}
