using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Holding-tab configuration for a profile operation.</summary>
public sealed record TabOptions
{
    /// <summary>Tabs per retained piece.</summary>
    public int Count { get; init; } = 4;

    /// <summary>Tab length along the cut path.</summary>
    public double Length { get; init; } = 5.0;

    /// <summary>Height of retained material measured up from the bottom of the cut.</summary>
    public double Thickness { get; init; } = 2.0;

    /// <summary>
    /// Plan tabs in material space so every piece that would otherwise come loose is bridged,
    /// instead of just spacing tabs evenly around each loop. Costlier, but the only mode that
    /// can prove no offcut is left free to move under the cutter.
    /// </summary>
    public bool ConstrainFloating { get; init; }

    /// <summary>Remnants thinner than this are treated as swarf rather than parts to retain.</summary>
    public double IgnoredSliverThickness { get; init; } = 0.25;
}

/// <summary>Configuration for contour/profile milling.</summary>
public sealed record ProfileOptions
{
    public ProfileSide Side { get; init; } = ProfileSide.Outside;

    /// <summary>
    /// Whether an outside profile cuts each material component's outer boundary.
    /// </summary>
    public bool IncludeOuterEnvelopes { get; init; } = true;

    /// <summary>
    /// Whether an outside profile omits boundaries around interior voids.
    /// </summary>
    public bool ExcludeInnerEnvelopes { get; init; }

    /// <summary>Material deliberately left on the wall for a later finishing pass.</summary>
    public double StockToLeave { get; init; }

    /// <summary>Extra passes at final depth that remove the deflection the first pass left.</summary>
    public int SpringPasses { get; init; }

    public MillingDirection Direction { get; init; } = MillingDirection.Climb;

    public RampSettings Ramp { get; init; } = RampSettings.FromRatio(2.0);

    public GeometryTolerances Tolerances { get; init; } = GeometryTolerances.Default;

    /// <summary>Holding tabs; <c>null</c> cuts straight through.</summary>
    public TabOptions? Tabs { get; init; }

    /// <summary>Stock frame used by material-space tab planning; falls back to a padded bounding box.</summary>
    public IReadOnlyList<PointD>? StockOutline { get; init; }

    /// <summary>
    /// Whether an inside profile may extend one cutter radius through a coincident stock edge to
    /// clear a square corner that a fully enclosed inside offset cannot reach.
    /// </summary>
    public bool ExtendThroughStockEdge { get; init; } = true;

    /// <summary>Optional boundary that rings are trimmed to.</summary>
    public IReadOnlyList<PointD>? ClipBoundary { get; init; }

    /// <summary>
    /// Whether to omit edge loops by emitting profile centerlines only within
    /// <see cref="ClipBoundary"/>.
    /// </summary>
    public bool OmitEdgeLoops { get; init; }
}

/// <summary>
/// Cuts a single contour around each region rather than clearing its interior.
///
/// <para>
/// Regions are first decomposed into connected components so tab planning can tell genuine
/// material from the voids punched through it. Offsetting a region without that step would let a
/// tab be placed on a bridge that does not exist in the source design.
/// </para>
/// </summary>
public static class ProfileToolpaths
{
    private readonly record struct StockEdgeContact(
        int RegionIndex,
        PointD Start,
        PointD End,
        PointD OutwardNormal);

    /// <summary>Category tag applied to profile passes.</summary>
    public const string Category = "profile";

    public static ToolpathPlan Generate(
        IReadOnlyList<MillingRegion> regions,
        ToolGeometry tool,
        DepthSchedule depth,
        ProfileOptions options,
        CancellationToken cancellationToken = default)
    {
        long t0 = PerfLog.Start();

        if (regions.Count == 0 || tool.Radius <= 1e-6 || depth.Depth <= 0)
            return ToolpathPlan.Empty;

        double offset = options.Side switch
        {
            ProfileSide.Outside => tool.Radius + options.StockToLeave,
            ProfileSide.Inside => -(tool.Radius + options.StockToLeave),
            _ => 0.0,
        };

        var rings = new List<OffsetRing>();
        var sourceMaterial = new List<List<PointD>>();
        var sourceComponents = new List<ProfileSourceComponent>();
        int componentIndex = 0;
        bool filterOutsideBoundaries = options.Side == ProfileSide.Outside;
        bool includeOuter = !filterOutsideBoundaries || options.IncludeOuterEnvelopes;
        bool includeInner = !filterOutsideBoundaries || !options.ExcludeInnerEnvelopes;

        foreach (var region in regions)
        {
            cancellationToken.ThrowIfCancellationRequested();

            var components = PolygonOps.DifferenceToComponents(
                new[] { region.Outer },
                region.Holes);

            foreach (var component in components)
            {
                var material = new List<IReadOnlyList<PointD>>(1 + component.Holes.Count) { component.Outer };
                material.AddRange(component.Holes);

                sourceMaterial.Add(component.Outer);
                sourceMaterial.AddRange(component.Holes);
                sourceComponents.Add(new ProfileSourceComponent(
                    componentIndex,
                    material.Select(boundary => (IReadOnlyList<PointD>)boundary).ToList()));

                var contours = Math.Abs(offset) < 1e-9
                    ? material.Select(boundary => new List<PointD>(boundary)).ToList()
                    : PolygonOps.Offset(
                        material,
                        offset,
                        options.Tolerances.ArcTolerance,
                        options.Tolerances.SimplifyTolerance);

                foreach (var contour in contours)
                {
                    if (contour.Count < 3)
                        continue;

                    rings.Add(RingOffsetEngine.CreateRing(contour, level: 0, componentIndex));
                }

                componentIndex++;
            }
        }

        // Compensation needs every material boundary: omitting an outer boundary before
        // PolygonOps.Offset reinterprets its holes as positive islands. Filter the final rings
        // instead, where IsHole preserves each compensated contour's material polarity.
        if (filterOutsideBoundaries)
            rings = rings.Where(ring => ring.IsHole ? includeInner : includeOuter).ToList();

        bool omitEdgeLoops = options.OmitEdgeLoops
            && options.ClipBoundary is { Count: >= 3 };
        if (options.ClipBoundary is { Count: >= 3 } clipBoundary)
        {
            // Standard clipping retains closed rings by intersecting their filled area. Outside
            // clipping instead slices the emitted centreline, preserving only interior spans.
            if (!omitEdgeLoops)
                rings = ClipRings(rings, clipBoundary);
        }

        int edgeBypassCount = 0;
        if (options.ExtendThroughStockEdge
            && options.Side == ProfileSide.Inside
            && options.StockOutline is { Count: >= 3 })
        {
            rings = BypassStockEdgeFillets(
                rings,
                sourceComponents,
                options.StockOutline,
                tool.Radius,
                options.StockToLeave,
                out edgeBypassCount);
            if (edgeBypassCount > 0)
            {
                Console.WriteLine(
                    $"[ProfileToolpaths.EdgeBypass] side=inside segments={edgeBypassCount} "
                    + $"maxOverrun={tool.Radius:R} stockToLeave={options.StockToLeave:R}");
            }
        }

        if (rings.Count == 0)
            return ToolpathPlan.Empty;

        HoldingTabReport? tabReport = null;
        var tabHook = BuildTabHook(rings, sourceMaterial, tool, depth, options, out tabReport);

        ProfileRouteForest route = ProfileRoutePlanner.Plan(
            rings,
            sourceComponents,
            tool.Radius,
            cancellationToken);
        ToolpathPlan plan = ProfileRouteMaterializer.Emit(
            rings,
            route,
            depth,
            options,
            tool,
            cancellationToken,
            tabHook);

        int interiorFragmentCount = plan.Toolpaths.Count;
        if (omitEdgeLoops)
        {
            plan = ClipToolpathsToInterior(
                plan,
                options.ClipBoundary!,
                tool,
                options.Tolerances,
                out interiorFragmentCount);
        }

        PerfLog.Stop("ProfileToolpaths.Generate", t0,
            $"regions={regions.Count} side={options.Side} outer={includeOuter} inner={includeInner} rings={rings.Count} " +
            $"omitEdgeLoops={omitEdgeLoops} edgeBypasses={edgeBypassCount} " +
            $"interiorFragments={interiorFragmentCount} paths={plan.Toolpaths.Count} " +
            $"tabs={(options.Tabs is null ? "off" : tabReport?.Summary ?? "on")}");

        return new ToolpathPlan(
            plan.Toolpaths.ToList(),
            plan.SweptArea.ToList(),
            plan.Diagnostics,
            tabReport);
    }

    private static List<OffsetRing> BypassStockEdgeFillets(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<ProfileSourceComponent> sourceComponents,
        IReadOnlyList<PointD> stockOutline,
        double toolRadius,
        double stockToLeave,
        out int bypassCount)
    {
        bypassCount = 0;
        double compensatedOffset = toolRadius + Math.Max(0, stockToLeave);
        if (compensatedOffset <= 1e-6)
            return rings.ToList();

        var contactsByRegion = BuildStockEdgeContacts(sourceComponents, stockOutline);
        if (contactsByRegion.Count == 0)
            return rings.ToList();

        var output = new List<OffsetRing>(rings.Count);
        foreach (var ring in rings)
        {
            if (!contactsByRegion.TryGetValue(ring.RegionIndex, out var contacts))
            {
                output.Add(ring);
                continue;
            }

            int ringBypassCount = 0;
            var points = new List<PointD>(ring.Points.Count + contacts.Count * 2);
            for (int pointIndex = 0; pointIndex < ring.Points.Count; pointIndex++)
            {
                PointD start = ring.Points[pointIndex];
                PointD end = ring.Points[(pointIndex + 1) % ring.Points.Count];
                points.Add(start);

                if (!TryFindInsideStockOffsetSegment(
                        start,
                        end,
                        contacts,
                        compensatedOffset,
                        out StockEdgeContact contact))
                {
                    continue;
                }

                points.Add(ProjectOutsideStock(start, contact, toolRadius));
                points.Add(ProjectOutsideStock(end, contact, toolRadius));
                ringBypassCount++;
            }

            bypassCount += ringBypassCount;
            output.Add(ringBypassCount == 0
                ? ring
                : RingOffsetEngine.CreateRing(points, ring.Level, ring.RegionIndex, ring.IsHole));
        }

        return output;
    }

    private static Dictionary<int, List<StockEdgeContact>> BuildStockEdgeContacts(
        IReadOnlyList<ProfileSourceComponent> sourceComponents,
        IReadOnlyList<PointD> stockOutline)
    {
        const double tolerance = 1e-6;
        double stockArea = PolygonOps.SignedArea(stockOutline);
        var contactsByRegion = new Dictionary<int, List<StockEdgeContact>>();

        foreach (var source in sourceComponents)
        {
            foreach (var boundary in source.Boundaries)
            {
                for (int sourceIndex = 0; sourceIndex < boundary.Count; sourceIndex++)
                {
                    PointD sourceStart = boundary[sourceIndex];
                    PointD sourceEnd = boundary[(sourceIndex + 1) % boundary.Count];
                    for (int stockIndex = 0; stockIndex < stockOutline.Count; stockIndex++)
                    {
                        PointD stockStart = stockOutline[stockIndex];
                        PointD stockEnd = stockOutline[(stockIndex + 1) % stockOutline.Count];
                        if (!TryBuildStockEdgeContact(
                                source.RegionIndex,
                                sourceStart,
                                sourceEnd,
                                stockStart,
                                stockEnd,
                                stockArea,
                                tolerance,
                                out StockEdgeContact contact))
                        {
                            continue;
                        }

                        if (!contactsByRegion.TryGetValue(source.RegionIndex, out var contacts))
                        {
                            contacts = new List<StockEdgeContact>();
                            contactsByRegion.Add(source.RegionIndex, contacts);
                        }
                        contacts.Add(contact);
                    }
                }
            }
        }

        return contactsByRegion;
    }

    private static bool TryBuildStockEdgeContact(
        int regionIndex,
        PointD sourceStart,
        PointD sourceEnd,
        PointD stockStart,
        PointD stockEnd,
        double stockArea,
        double tolerance,
        out StockEdgeContact contact)
    {
        contact = default;
        double stockX = stockEnd.x - stockStart.x;
        double stockY = stockEnd.y - stockStart.y;
        double stockLength = Math.Sqrt(stockX * stockX + stockY * stockY);
        if (stockLength <= tolerance)
            return false;

        double sourceX = sourceEnd.x - sourceStart.x;
        double sourceY = sourceEnd.y - sourceStart.y;
        double sourceLength = Math.Sqrt(sourceX * sourceX + sourceY * sourceY);
        if (sourceLength <= tolerance)
            return false;

        if (Math.Abs(sourceX * stockY - sourceY * stockX) > tolerance * sourceLength * stockLength)
            return false;

        double startOffsetX = sourceStart.x - stockStart.x;
        double startOffsetY = sourceStart.y - stockStart.y;
        if (Math.Abs(startOffsetX * stockY - startOffsetY * stockX) > tolerance * stockLength)
            return false;

        double sourceStartProjection = (startOffsetX * stockX + startOffsetY * stockY) / stockLength;
        double sourceEndProjection = ((sourceEnd.x - stockStart.x) * stockX
            + (sourceEnd.y - stockStart.y) * stockY) / stockLength;
        if (Math.Min(sourceStartProjection, sourceEndProjection) >= stockLength - tolerance
            || Math.Max(sourceStartProjection, sourceEndProjection) <= tolerance)
        {
            return false;
        }

        double tangentX = stockX / stockLength;
        double tangentY = stockY / stockLength;
        PointD outwardNormal = stockArea >= 0
            ? new PointD(tangentY, -tangentX)
            : new PointD(-tangentY, tangentX);
        contact = new StockEdgeContact(regionIndex, stockStart, stockEnd, outwardNormal);
        return true;
    }

    private static bool TryFindInsideStockOffsetSegment(
        PointD start,
        PointD end,
        IReadOnlyList<StockEdgeContact> contacts,
        double compensatedOffset,
        out StockEdgeContact contact)
    {
        double segmentX = end.x - start.x;
        double segmentY = end.y - start.y;
        double segmentLength = Math.Sqrt(segmentX * segmentX + segmentY * segmentY);
        double tolerance = Math.Max(0.02, compensatedOffset * 1e-3);
        if (segmentLength <= tolerance)
        {
            contact = default;
            return false;
        }

        PointD midpoint = new((start.x + end.x) * 0.5, (start.y + end.y) * 0.5);
        foreach (var candidate in contacts)
        {
            double stockX = candidate.End.x - candidate.Start.x;
            double stockY = candidate.End.y - candidate.Start.y;
            double stockLength = Math.Sqrt(stockX * stockX + stockY * stockY);
            if (stockLength <= tolerance)
                continue;

            if (Math.Abs(segmentX * stockY - segmentY * stockX)
                > tolerance * segmentLength * stockLength)
            {
                continue;
            }

            double midpointX = midpoint.x - candidate.Start.x;
            double midpointY = midpoint.y - candidate.Start.y;
            double tangentDistance = (midpointX * stockX + midpointY * stockY) / stockLength;
            if (tangentDistance <= -tolerance || tangentDistance >= stockLength + tolerance)
                continue;

            double signedDistance = midpointX * candidate.OutwardNormal.x
                + midpointY * candidate.OutwardNormal.y;
            if (signedDistance >= -tolerance
                || Math.Abs(signedDistance + compensatedOffset) > tolerance)
            {
                continue;
            }

            contact = candidate;
            return true;
        }

        contact = default;
        return false;
    }

    private static PointD ProjectOutsideStock(
        PointD point,
        StockEdgeContact contact,
        double toolRadius)
    {
        double signedDistance = (point.x - contact.Start.x) * contact.OutwardNormal.x
            + (point.y - contact.Start.y) * contact.OutwardNormal.y;
        double distanceToOutsideLimit = toolRadius - signedDistance;
        return new PointD(
            point.x + contact.OutwardNormal.x * distanceToOutsideLimit,
            point.y + contact.OutwardNormal.y * distanceToOutsideLimit);
    }

    private static ToolpathPlan ClipToolpathsToInterior(
        ToolpathPlan source,
        IReadOnlyList<PointD> clipBoundary,
        ToolGeometry tool,
        GeometryTolerances tolerances,
        out int interiorFragmentCount)
    {
        var fragments = new List<TaggedToolpath>();
        int sourceMoveCount = 0;
        int retainedMoveCount = 0;
        foreach (var toolpath in source.Toolpaths)
        {
            var pathFragments = ClipToolpathToInterior(
                toolpath,
                clipBoundary,
                out int pathSourceMoveCount,
                out int pathRetainedMoveCount);
            fragments.AddRange(pathFragments);
            sourceMoveCount += pathSourceMoveCount;
            retainedMoveCount += pathRetainedMoveCount;
        }

        interiorFragmentCount = fragments.Count;
        var sweptCenterlines = new List<IReadOnlyList<PointD>>();
        foreach (var fragment in fragments)
        {
            if (!HasPlanarMotion(fragment.Points))
                continue;

            sweptCenterlines.Add(fragment.Points
                .Select(point => new PointD(point.X, point.Y))
                .ToList());
        }

        var sweptArea = sweptCenterlines.Count == 0
            ? new List<List<PointD>>()
            : PolygonOps.Clip(
                PolygonOps.BufferCenterlines(
                    sweptCenterlines,
                    tool.Radius,
                    tolerances.ArcTolerance),
                clipBoundary);
        var diagnostics = source.Diagnostics with
        {
            PathCount = fragments.Count,
            CutLength = MeasureLength(fragments, ToolpathSpanKind.Cut),
            LinkLength = MeasureLength(fragments, ToolpathSpanKind.Link),
        };

        Console.WriteLine(
            $"[ProfileToolpaths.ClipOutside] sourcePaths={source.Toolpaths.Count} "
            + $"sourceMoves={sourceMoveCount} retainedMoves={retainedMoveCount} "
            + $"interiorFragments={fragments.Count} sweptPolygons={sweptArea.Count}");
        return new ToolpathPlan(fragments, sweptArea, diagnostics, source.TabReport);
    }

    private static List<TaggedToolpath> ClipToolpathToInterior(
        TaggedToolpath source,
        IReadOnlyList<PointD> clipBoundary,
        out int sourceMoveCount,
        out int retainedMoveCount)
    {
        sourceMoveCount = Math.Max(0, source.Points.Count - 1);
        retainedMoveCount = 0;
        var fragments = new List<TaggedToolpath>();
        var moveKinds = BuildMoveKinds(source);
        var points = new List<Point3D>();
        var kinds = new List<ToolpathSpanKind>();

        void FlushFragment()
        {
            if (points.Count >= 2 && kinds.Count > 0)
            {
                fragments.Add(new TaggedToolpath(
                    points.ToList(),
                    BuildSpans(kinds),
                    source.RegionIndex,
                    source.Category,
                    source.DepthPassIndex,
                    source.VisitIdentity));
            }

            points.Clear();
            kinds.Clear();
        }

        for (int pointIndex = 1; pointIndex < source.Points.Count; pointIndex++)
        {
            Point3D start = source.Points[pointIndex - 1];
            Point3D end = source.Points[pointIndex];
            ToolpathSpanKind kind = moveKinds[pointIndex - 1];
            bool retainedThisMove = false;
            foreach (var (startT, endT) in FindInteriorIntervals(start, end, clipBoundary))
            {
                Point3D clippedStart = Interpolate(start, end, startT);
                Point3D clippedEnd = Interpolate(start, end, endT);
                if (PointsCoincide(clippedStart, clippedEnd))
                    continue;

                if (points.Count == 0)
                {
                    points.Add(clippedStart);
                }
                else if (!PointsCoincide(points[^1], clippedStart))
                {
                    FlushFragment();
                    points.Add(clippedStart);
                }

                points.Add(clippedEnd);
                kinds.Add(kind);
                retainedThisMove = true;
            }

            if (retainedThisMove)
            {
                retainedMoveCount++;
            }
            else
            {
                FlushFragment();
            }
        }

        FlushFragment();
        return fragments;
    }

    private static IEnumerable<(double Start, double End)> FindInteriorIntervals(
        Point3D start,
        Point3D end,
        IReadOnlyList<PointD> boundary)
    {
        const double tolerance = 1e-10;
        var splitPoints = new List<double> { 0, 1 };
        var startPoint = new PointD(start.X, start.Y);
        var endPoint = new PointD(end.X, end.Y);
        for (int boundaryIndex = 0; boundaryIndex < boundary.Count; boundaryIndex++)
        {
            PointD boundaryStart = boundary[boundaryIndex];
            PointD boundaryEnd = boundary[(boundaryIndex + 1) % boundary.Count];
            if (TryGetIntersectionParameter(
                    startPoint, endPoint, boundaryStart, boundaryEnd, out double parameter))
            {
                splitPoints.Add(parameter);
            }
        }

        splitPoints.Sort();
        var uniqueSplitPoints = new List<double>(splitPoints.Count);
        foreach (double parameter in splitPoints)
        {
            double clamped = Math.Clamp(parameter, 0, 1);
            if (uniqueSplitPoints.Count == 0
                || clamped - uniqueSplitPoints[^1] > tolerance)
            {
                uniqueSplitPoints.Add(clamped);
            }
        }

        for (int splitIndex = 1; splitIndex < uniqueSplitPoints.Count; splitIndex++)
        {
            double intervalStart = uniqueSplitPoints[splitIndex - 1];
            double intervalEnd = uniqueSplitPoints[splitIndex];
            if (intervalEnd - intervalStart <= tolerance)
                continue;

            double midpoint = (intervalStart + intervalEnd) * 0.5;
            Point3D midpointPoint = Interpolate(start, end, midpoint);
            if (IsPointStrictlyInsidePolygon(new PointD(midpointPoint.X, midpointPoint.Y), boundary))
                yield return (intervalStart, intervalEnd);
        }
    }

    private static bool TryGetIntersectionParameter(
        PointD segmentStart,
        PointD segmentEnd,
        PointD boundaryStart,
        PointD boundaryEnd,
        out double parameter)
    {
        const double tolerance = 1e-10;
        double segmentX = segmentEnd.x - segmentStart.x;
        double segmentY = segmentEnd.y - segmentStart.y;
        double boundaryX = boundaryEnd.x - boundaryStart.x;
        double boundaryY = boundaryEnd.y - boundaryStart.y;
        double denominator = segmentX * boundaryY - segmentY * boundaryX;
        if (Math.Abs(denominator) <= tolerance)
        {
            parameter = 0;
            return false;
        }

        double offsetX = boundaryStart.x - segmentStart.x;
        double offsetY = boundaryStart.y - segmentStart.y;
        double segmentParameter = (offsetX * boundaryY - offsetY * boundaryX) / denominator;
        double boundaryParameter = (offsetX * segmentY - offsetY * segmentX) / denominator;
        if (segmentParameter < -tolerance || segmentParameter > 1 + tolerance
            || boundaryParameter < -tolerance || boundaryParameter > 1 + tolerance)
        {
            parameter = 0;
            return false;
        }

        parameter = Math.Clamp(segmentParameter, 0, 1);
        return true;
    }

    private static bool IsPointStrictlyInsidePolygon(PointD point, IReadOnlyList<PointD> polygon)
    {
        bool inside = false;
        for (int currentIndex = 0, previousIndex = polygon.Count - 1;
             currentIndex < polygon.Count;
             previousIndex = currentIndex++)
        {
            PointD current = polygon[currentIndex];
            PointD previous = polygon[previousIndex];
            if (PointIsOnSegment(point, previous, current))
                return false;

            bool crossesRay = (current.y > point.y) != (previous.y > point.y)
                && point.x < (previous.x - current.x) * (point.y - current.y)
                    / (previous.y - current.y) + current.x;
            if (crossesRay)
                inside = !inside;
        }

        return inside;
    }

    private static bool PointIsOnSegment(PointD point, PointD segmentStart, PointD segmentEnd)
    {
        const double tolerance = 1e-9;
        double cross = (segmentEnd.x - segmentStart.x) * (point.y - segmentStart.y)
            - (segmentEnd.y - segmentStart.y) * (point.x - segmentStart.x);
        if (Math.Abs(cross) > tolerance)
            return false;

        return point.x >= Math.Min(segmentStart.x, segmentEnd.x) - tolerance
            && point.x <= Math.Max(segmentStart.x, segmentEnd.x) + tolerance
            && point.y >= Math.Min(segmentStart.y, segmentEnd.y) - tolerance
            && point.y <= Math.Max(segmentStart.y, segmentEnd.y) + tolerance;
    }

    private static Point3D Interpolate(Point3D start, Point3D end, double parameter)
        => new(
            start.X + (end.X - start.X) * parameter,
            start.Y + (end.Y - start.Y) * parameter,
            start.Z + (end.Z - start.Z) * parameter);

    private static bool PointsCoincide(Point3D first, Point3D second)
        => Math.Abs(first.X - second.X) <= 1e-9
            && Math.Abs(first.Y - second.Y) <= 1e-9
            && Math.Abs(first.Z - second.Z) <= 1e-9;

    private static ToolpathSpanKind[] BuildMoveKinds(TaggedToolpath path)
    {
        var kinds = new ToolpathSpanKind[Math.Max(0, path.Points.Count - 1)];
        Array.Fill(kinds, ToolpathSpanKind.Cut);
        foreach (var span in path.Spans)
        {
            int firstPoint = Math.Max(1, span.StartIndex + 1);
            int lastPoint = Math.Min(path.Points.Count - 1, span.EndIndex);
            for (int pointIndex = firstPoint; pointIndex <= lastPoint; pointIndex++)
                kinds[pointIndex - 1] = span.Kind;
        }

        return kinds;
    }

    private static List<ToolpathSpan> BuildSpans(IReadOnlyList<ToolpathSpanKind> moveKinds)
    {
        var spans = new List<ToolpathSpan>();
        if (moveKinds.Count == 0)
            return spans;

        int startPointIndex = 0;
        ToolpathSpanKind kind = moveKinds[0];
        for (int moveIndex = 1; moveIndex < moveKinds.Count; moveIndex++)
        {
            if (moveKinds[moveIndex] == kind)
                continue;

            spans.Add(new ToolpathSpan(startPointIndex, moveIndex, kind));
            startPointIndex = moveIndex;
            kind = moveKinds[moveIndex];
        }

        spans.Add(new ToolpathSpan(startPointIndex, moveKinds.Count, kind));
        return spans;
    }

    private static bool HasPlanarMotion(IReadOnlyList<Point3D> points)
    {
        for (int pointIndex = 1; pointIndex < points.Count; pointIndex++)
        {
            double deltaX = points[pointIndex].X - points[pointIndex - 1].X;
            double deltaY = points[pointIndex].Y - points[pointIndex - 1].Y;
            if (deltaX * deltaX + deltaY * deltaY > 1e-18)
                return true;
        }

        return false;
    }

    private static double MeasureLength(
        IReadOnlyList<TaggedToolpath> paths,
        ToolpathSpanKind targetKind)
    {
        double total = 0;
        foreach (var path in paths)
        {
            ToolpathSpanKind[] moveKinds = BuildMoveKinds(path);
            for (int pointIndex = 1; pointIndex < path.Points.Count; pointIndex++)
            {
                if (moveKinds[pointIndex - 1] != targetKind)
                    continue;

                Point3D start = path.Points[pointIndex - 1];
                Point3D end = path.Points[pointIndex];
                double deltaX = end.X - start.X;
                double deltaY = end.Y - start.Y;
                total += Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
            }
        }

        return total;
    }

    /// <summary>
    /// Resolves tab zones per ring, then returns a hook that lifts each emitted path over them.
    /// </summary>
    private static Func<int, List<Point3D>, List<ToolpathSpan>, (List<Point3D>, List<ToolpathSpan>)>?
        BuildTabHook(
            IReadOnlyList<OffsetRing> rings,
            IReadOnlyList<List<PointD>> sourceMaterial,
            ToolGeometry tool,
            DepthSchedule depth,
            ProfileOptions options,
            out HoldingTabReport? report)
    {
        report = null;

        var tabs = options.Tabs;
        if (tabs is null || tabs.Count < 1 || tabs.Length <= 0)
            return null;

        double thickness = Math.Clamp(tabs.Thickness, 0, depth.Depth);
        double tabZ = depth.SurfaceZ - depth.Depth + thickness;
        if (thickness <= 1e-6)
            return null;

        var zonesByRing = new Dictionary<int, TabZone[]>();

        if (tabs.ConstrainFloating)
        {
            var ringPoints = rings.Select(r => r.Points).ToList();
            var perimeters = rings.Select(r => r.Perimeter).ToList();

            var plan = HoldingTabPlanner.Plan(
                ringPoints,
                perimeters,
                options.StockOutline is null ? null : new List<PointD>(options.StockOutline),
                sourceMaterial.Select(m => new List<PointD>(m)).ToList(),
                tool.Radius,
                tabs.Count,
                tabs.Length,
                tabZ,
                tabs.IgnoredSliverThickness);

            report = plan.Report;

            foreach (var (ringIndex, intervals) in plan.RingZones)
            {
                var zones = intervals.Select(i => new TabZone(i.Start, i.End)).ToList();
                var normalized = TabZonePlanner.NormalizeProjected(
                    zones, rings[ringIndex].Perimeter);
                if (normalized is null)
                    continue;

                zonesByRing[ringIndex] = normalized;
            }
        }
        else
        {
            // Keep a cutter-width of clearance either side so the tab is not undercut as the
            // tool rolls onto and off it.
            double protectedLength = tabs.Length + tool.Radius * 2;
            var bridgesMaterial = BuildBridgeTest(
                rings, sourceMaterial, tool.Radius, tabs.Length, options.StockOutline);

            int seated = 0;
            int starvedRings = 0;
            for (int i = 0; i < rings.Count; i++)
            {
                if (!TabZonePlanner.TryBuildEvenlySpaced(
                        rings[i].Points, rings[i].Perimeter, tabs.Count, protectedLength, tool.Radius,
                        out var zones, bridgesMaterial))
                {
                    starvedRings++;
                    continue;
                }

                var normalized = TabZonePlanner.Normalize(zones, rings[i].Perimeter);
                if (normalized is null)
                {
                    starvedRings++;
                    continue;
                }

                zonesByRing[i] = normalized;
                seated += normalized.Length;
            }

            PerfLog.Stop("ProfileToolpaths.TabSeats", PerfLog.Start(),
                $"rings={rings.Count} tabbedRings={zonesByRing.Count} starvedRings={starvedRings} " +
                $"seated={seated}/{rings.Count * tabs.Count} " +
                $"bridgeTest={(bridgesMaterial is null ? "unavailable" : "on")}");
        }

        if (zonesByRing.Count == 0)
            return null;

        return (ringIndex, points, spans) =>
        {
            if (!zonesByRing.TryGetValue(ringIndex, out var zones))
                return (points, spans);

            double seamArc = ArcPositionOnRing(
                rings[ringIndex].Points,
                new PointD(points[0].X, points[0].Y));
            TabZone[] rebasedZones = options.Direction == MillingDirection.Conventional
                ? zones.Select(zone => new TabZone(
                    seamArc - zone.End,
                    seamArc - zone.Start)).ToArray()
                : zones.Select(zone => new TabZone(
                    zone.Start - seamArc,
                    zone.End - seamArc)).ToArray();

            double pathCutZ = points.Min(point => point.Z);
            var lifted = TabZonePlanner.ApplyToClosedPath(
                points, spans, rebasedZones, rings[ringIndex].Perimeter, pathCutZ, tabZ);
            return (lifted.Points, lifted.Spans);
        };
    }

    private static double ArcPositionOnRing(
        IReadOnlyList<PointD> ring,
        PointD target)
    {
        var loop = RampPlanner.Normalize(ring);
        double bestDistanceSquared = double.MaxValue;
        double bestArc = 0;
        double arc = 0;

        for (int i = 0; i < loop.Count; i++)
        {
            var start = loop[i];
            var end = loop[(i + 1) % loop.Count];
            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double lengthSquared = dx * dx + dy * dy;
            double length = Math.Sqrt(lengthSquared);
            if (length <= 1e-12)
                continue;

            double position = Math.Clamp(
                ((target.x - start.x) * dx + (target.y - start.y) * dy) / lengthSquared,
                0,
                1);
            double closestX = start.x + dx * position;
            double closestY = start.y + dy * position;
            double targetDx = target.x - closestX;
            double targetDy = target.y - closestY;
            double distanceSquared = targetDx * targetDx + targetDy * targetDy;
            if (distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;
                bestArc = arc + length * position;
            }
            arc += length;
        }

        return bestArc;
    }

    /// <summary>
    /// Builds the "is this seat actually a bridge?" test for evenly spaced tabs.
    ///
    /// <para>
    /// A tab only retains a part if stock survives the cut on <b>both</b> sides of the kerf, so
    /// the test probes across the cut into the material model that is left once every ring has
    /// been swept. Spacing alone cannot tell a bridge from a seat on a ring running along the
    /// edge of the stock, where the tab is machined into air and holds nothing.
    /// </para>
    /// </summary>
    private static Func<PointD, PointD, bool>? BuildBridgeTest(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<List<PointD>> sourceMaterial,
        double toolRadius,
        double tabLength,
        IReadOnlyList<PointD>? stockOutline)
    {
        var ringPolys = new List<List<PointD>>(rings.Count);
        foreach (var ring in rings)
            if (ring.Points.Count >= 3)
                ringPolys.Add(ring.Points);

        var cutRegion = PolygonOps.BufferCenterlines(ringPolys, toolRadius);
        if (cutRegion.Count == 0)
            return null;

        var material = sourceMaterial.Select(m => new List<PointD>(m)).ToList();
        var stock = HoldingTabPlanner.BuildStockOutline(
            stockOutline is null ? null : new List<PointD>(stockOutline),
            cutRegion,
            material,
            toolRadius,
            tabLength);
        if (stock.Count < 3)
            return null;

        var remaining = PolygonOps.Difference(new List<List<PointD>> { stock }, cutRegion);
        if (remaining.Count == 0)
            return null;

        // Clear the kerf wall by a machinable web before calling the far side material: a sliver
        // narrower than this tears off with the tab still attached to it.
        double reach = toolRadius + Math.Max(0.25, toolRadius * 0.2);

        return (point, normal) =>
            PolygonOps.Contains(remaining, point.x + normal.x * reach, point.y + normal.y * reach)
            && PolygonOps.Contains(remaining, point.x - normal.x * reach, point.y - normal.y * reach);
    }

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
}
