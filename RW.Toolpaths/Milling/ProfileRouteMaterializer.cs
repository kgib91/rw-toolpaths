using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Materializes a profile route-inspection forest into executable toolpaths.</summary>
internal static class ProfileRouteMaterializer
{
    private readonly record struct PreparedPath(
        int RingIndex,
        int PassIndex,
        List<Point3D> Points,
        List<ToolpathSpan> Spans,
        RampStrategy Strategy,
        int PlungeEntries,
        int LinearRampEntries,
        int HelicalEntries);

    private sealed class DepthFirstState
    {
        internal HashSet<int> CompletedRings { get; } = new();
        internal int SpiralCount { get; set; }
        internal int PassesCombined { get; set; }
        internal int AvoidedRetracts { get; set; }
        internal int LinkCandidates { get; set; }
        internal int LinksAccepted { get; set; }
        internal int LinksRejectedAboveDepth { get; set; }
        internal int LinksRejectedByDistance { get; set; }
        internal int LinksRejectedByProtection { get; set; }
        internal double MinimumLinkDistance { get; set; } = double.MaxValue;
        internal double MaximumLinkDistance { get; set; }
    }

    internal static ToolpathPlan Emit(
        IReadOnlyList<OffsetRing> rings,
        ProfileRouteForest route,
        DepthSchedule depth,
        ProfileOptions options,
        ToolGeometry tool,
        CancellationToken cancellationToken,
        Func<int, List<Point3D>, List<ToolpathSpan>,
            (List<Point3D> Points, List<ToolpathSpan> Spans)>? applyTabs)
    {
        long t0 = PerfLog.Start();
        var toolpaths = new List<TaggedToolpath>();
        Point3D? previousEnd = null;
        double cutLength = 0;
        double linkLength = 0;
        int emittedPointCount = 0;
        int linkLifts = 0;
        int plunges = 0, linearRamps = 0, helicals = 0;
        var depthFirst = new DepthFirstState();

        foreach (var tree in route.Trees)
        {
            int visitCount = depth.PassCount + options.SpringPasses;
            for (int visit = 0; visit < visitCount; visit++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (depthFirst.CompletedRings.Contains(tree.Root.RingIndex))
                    continue;

                int pass = Math.Min(visit, depth.PassCount - 1);

                PointD? rootEntry = previousEnd is { } previous
                    ? new PointD(previous.X, previous.Y)
                    : null;
                var paths = BuildNodePaths(
                    tree.Root,
                    entryTarget: rootEntry,
                    rings,
                    depth,
                    pass,
                    options,
                    tool,
                    cancellationToken,
                    applyTabs,
                    depthFirst,
                    allowDepthFirst: options.SpringPasses == 0);

                foreach (var path in paths)
                {
                    if (path.Points.Count < 2)
                        continue;

                    if (previousEnd is { } from)
                    {
                        if (!PointsCoincide(from, path.Points[0]))
                        {
                            linkLength += PlanarDistance(from, path.Points[0]);
                            linkLifts++;
                        }
                    }
                    previousEnd = path.Points[^1];

                    plunges += path.PlungeEntries;
                    linearRamps += path.LinearRampEntries;
                    helicals += path.HelicalEntries;

                    cutLength += CutLength(path.Points, path.Spans);
                    emittedPointCount += path.Points.Count;
                    int visitIdentity = checked(visit * rings.Count + path.RingIndex);
                    toolpaths.Add(new TaggedToolpath(
                        path.Points,
                        path.Spans,
                        rings[path.RingIndex].RegionIndex,
                        ProfileToolpaths.Category,
                        path.PassIndex,
                        visitIdentity));
                }
            }
        }

        long sweptAreaStart = PerfLog.Start();
        var sweptCenterlines = rings
            .Select(ring => (IReadOnlyList<PointD>)ring.Points)
            .ToList();
        var linkKeys = new HashSet<(long StartX, long StartY, long EndX, long EndY)>();
        int depthLinkCount = 0;
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
                    linkLength += PlanarDistance(start, end);

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
                    depthLinkCount++;
                }
            }
        }

        int sweptCenterlinePoints = sweptCenterlines.Sum(centerline => centerline.Count);
        var sweptArea = toolpaths.Count > 0
            ? PolygonOps.BufferCenterlines(
                sweptCenterlines,
                tool.Radius,
                options.Tolerances.ArcTolerance)
            : new List<List<PointD>>();
        PerfLog.Stop("ProfileRouteMaterializer.SweptArea", sweptAreaStart,
            $"rings={rings.Count} depthLinks={depthLinkCount} emittedPoints={emittedPointCount} " +
            $"bufferedPoints={sweptCenterlinePoints} polygons={sweptArea.Count}");
        var diagnostics = new ToolpathDiagnostics
        {
            RingCount = rings.Count,
            IslandCount = route.Trees.Count,
            PathCount = toolpaths.Count,
            DepthPassCount = depth.PassCount,
            CutLength = cutLength,
            LinkLength = linkLength,
            LinkLifts = linkLifts,
            PlungeEntries = plunges,
            LinearRampEntries = linearRamps,
            HelicalEntries = helicals,
            DepthFirstSpirals = depthFirst.SpiralCount,
            PassesCombined = depthFirst.PassesCombined,
            AvoidedRetracts = depthFirst.AvoidedRetracts,
        };

        PerfLog.Stop("ProfileRouteMaterializer.Emit", t0,
            $"trees={route.Trees.Count} portals={route.PortalCount} paths={toolpaths.Count} " +
            $"passes={depth.PassCount} depthFirstSpirals={depthFirst.SpiralCount} " +
            $"passesCombined={depthFirst.PassesCombined} " +
            $"avoidedRetracts={depthFirst.AvoidedRetracts} " +
            $"depthLinks={depthLinkCount} lifts={linkLifts} " +
            $"linkCandidates={depthFirst.LinkCandidates} " +
            $"linkAccepted={depthFirst.LinksAccepted} " +
            $"linkRejectedDepth={depthFirst.LinksRejectedAboveDepth} " +
            $"linkRejectedDistance={depthFirst.LinksRejectedByDistance} " +
            $"linkRejectedProtection={depthFirst.LinksRejectedByProtection} " +
            $"linkDistanceRange={FormatDistanceRange(depthFirst)} " +
            $"cut={cutLength:F3} linkXY={linkLength:F3}");
        return new ToolpathPlan(toolpaths, sweptArea, diagnostics);
    }

    private static List<PreparedPath> BuildNodePaths(
        ProfileRouteNode node,
        PointD? entryTarget,
        IReadOnlyList<OffsetRing> rings,
        DepthSchedule depth,
        int pass,
        ProfileOptions options,
        ToolGeometry tool,
        CancellationToken cancellationToken,
        Func<int, List<Point3D>, List<ToolpathSpan>,
            (List<Point3D> Points, List<ToolpathSpan> Spans)>? applyTabs,
        DepthFirstState depthFirst,
        bool enterAtDepth = false,
        bool allowDepthFirst = true)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var activeBranches = node.Children
            .Where(branch => !depthFirst.CompletedRings.Contains(branch.Child.RingIndex))
            .ToList();
        var ring = RingOffsetEngine.Orient(rings[node.RingIndex].Points, options.Direction);
        foreach (var branch in activeBranches)
            ring = InsertPoint(ring, branch.Portal.FromPoint, rotate: false);
        if (entryTarget is { } target)
            ring = InsertPoint(ring, target, rotate: true);

        double bottomZ = depth.PassBottomZ(pass);
        var ramped = RampPlanner.PlanClosedRing(
            ring,
            enterAtDepth ? bottomZ : depth.PassTopZ(pass),
            bottomZ,
            options.Ramp);
        var points = ramped.Points;
        var spans = ramped.Spans;
        if (applyTabs is not null)
            (points, spans) = applyTabs(node.RingIndex, points, spans);

        var prepared = CreatePreparedPath(
            node.RingIndex,
            pass,
            points,
            spans,
            ramped.Strategy,
            countEntry: !enterAtDepth);

        if (allowDepthFirst
            && node.Children.Count == 0
            && pass < depth.PassCount - 1)
        {
            PreparedPath combined = BuildDepthFirstPath(
                node.RingIndex,
                ring,
                pass,
                prepared,
                depth,
                options,
                cancellationToken,
                applyTabs);
            depthFirst.CompletedRings.Add(node.RingIndex);
            depthFirst.SpiralCount++;
            depthFirst.PassesCombined += depth.PassCount - pass;
            depthFirst.AvoidedRetracts += depth.PassCount - pass - 1;
            return new List<PreparedPath> { combined };
        }

        if (activeBranches.Count == 0)
            return new List<PreparedPath> { prepared };

        int splitStart = spans
            .Where(span => span.Kind == ToolpathSpanKind.Cut)
            .Select(span => span.StartIndex)
            .DefaultIfEmpty(-1)
            .Min();
        if (enterAtDepth && splitStart >= 0)
            splitStart = Math.Min(points.Count - 1, Math.Max(1, splitStart));
        bool allowTabLift = splitStart < 0;
        if (allowTabLift)
        {
            splitStart = spans
                .Where(span => span.Kind == ToolpathSpanKind.TabLift)
                .Select(span => span.StartIndex)
                .DefaultIfEmpty(-1)
                .Min();
            if (splitStart < 0)
            {
                throw new InvalidOperationException(
                    $"Profile ring {node.RingIndex} has no safe portal span on pass {pass}.");
            }
        }

        var branchesBySplit = activeBranches
            .Select(branch => (
                Branch: branch,
                Split: FindSplitIndex(
                    points,
                    spans,
                    splitStart,
                    depth.PassBottomZ(pass),
                    branch.Portal.FromPoint,
                    allowTabLift)))
            .GroupBy(entry => entry.Split)
            .OrderBy(group => group.Key)
            .ToList();

        var output = new List<PreparedPath>();
        int cursor = 0;
        bool firstFragment = true;
        bool resumeAtDepth = false;
        foreach (var group in branchesBySplit)
        {
            int split = group.Key;
            if (split > cursor)
            {
                output.Add(Slice(
                    node.RingIndex,
                    pass,
                    points,
                    spans,
                    ramped.Strategy,
                    cursor,
                    split,
                    firstFragment,
                    resumeAtDepth,
                    enterAtDepth,
                    depth.PassTopZ(pass)));
                firstFragment = false;
                resumeAtDepth = true;
            }

            Point3D pause = points[split];
            foreach (var entry in group)
            {
                if (TryBuildLinkedBranch(
                        entry.Branch,
                        pause,
                        rings,
                        depth,
                        pass,
                        options,
                        tool,
                        cancellationToken,
                        applyTabs,
                        depthFirst,
                        out var linkedPaths))
                {
                    output.AddRange(linkedPaths);
                    resumeAtDepth = true;
                    continue;
                }

                var childEntry = ClosestPointOnRing(
                    rings[entry.Branch.Child.RingIndex].Points,
                    new PointD(pause.X, pause.Y));
                output.AddRange(BuildNodePaths(
                    entry.Branch.Child,
                    childEntry,
                    rings,
                    depth,
                    pass,
                    options,
                    tool,
                    cancellationToken,
                    applyTabs,
                    depthFirst,
                    enterAtDepth: false,
                    allowDepthFirst));
                resumeAtDepth = false;
            }
            cursor = split;
        }

        if (cursor < points.Count - 1)
        {
            output.Add(Slice(
                node.RingIndex,
                pass,
                points,
                spans,
                ramped.Strategy,
                cursor,
                points.Count - 1,
                firstFragment,
                resumeAtDepth,
                enterAtDepth,
                depth.PassTopZ(pass)));
        }
        return output;
    }

    private static bool TryBuildLinkedBranch(
        ProfileRouteBranch branch,
        Point3D pause,
        IReadOnlyList<OffsetRing> rings,
        DepthSchedule depth,
        int pass,
        ProfileOptions options,
        ToolGeometry tool,
        CancellationToken cancellationToken,
        Func<int, List<Point3D>, List<ToolpathSpan>,
            (List<Point3D> Points, List<ToolpathSpan> Spans)>? applyTabs,
        DepthFirstState depthFirst,
        out List<PreparedPath> linkedPaths)
    {
        linkedPaths = new List<PreparedPath>();
        depthFirst.LinkCandidates++;
        depthFirst.MinimumLinkDistance = Math.Min(
            depthFirst.MinimumLinkDistance,
            branch.Portal.Distance);
        depthFirst.MaximumLinkDistance = Math.Max(
            depthFirst.MaximumLinkDistance,
            branch.Portal.Distance);

        double bottomZ = depth.PassBottomZ(pass);
        if (Math.Abs(pause.Z - bottomZ) > 1e-6)
        {
            depthFirst.LinksRejectedAboveDepth++;
            return false;
        }

        var childEntry = ClosestPointOnRing(
            rings[branch.Child.RingIndex].Points,
            new PointD(pause.X, pause.Y));
        var childStart = new Point3D(childEntry.x, childEntry.y, bottomZ);
        double maximumLinkDistance = tool.Radius + LinkClearance.DefaultTolerance;
        if (PlanarDistance(pause, childStart)
            > maximumLinkDistance)
        {
            depthFirst.LinksRejectedByDistance++;
            return false;
        }

        var candidate = BuildNodePaths(
            branch.Child,
            childEntry,
            rings,
            depth,
            pass,
            options,
            tool,
            cancellationToken,
            applyTabs,
            depthFirst,
            enterAtDepth: true,
            allowDepthFirst: false);
        if (candidate.Count == 0
            || candidate[0].Points.Count == 0
            || candidate[^1].Points.Count == 0)
        {
            depthFirst.LinksRejectedByProtection++;
            return false;
        }

        Point3D actualStart = candidate[0].Points[0];
        Point3D actualEnd = candidate[^1].Points[^1];
        if (Math.Abs(actualStart.Z - bottomZ) > 1e-6
            || Math.Abs(actualEnd.Z - bottomZ) > 1e-6
            || PlanarDistance(pause, actualStart)
                > maximumLinkDistance
            || PlanarDistance(actualEnd, pause)
                > maximumLinkDistance)
        {
            depthFirst.LinksRejectedByProtection++;
            return false;
        }

        candidate[0] = PrependLink(candidate[0], pause);
        candidate[^1] = AppendLink(candidate[^1], pause);
        depthFirst.LinksAccepted++;
        linkedPaths = candidate;
        return true;
    }

    private static PreparedPath BuildDepthFirstPath(
        int ringIndex,
        IReadOnlyList<PointD> ring,
        int firstPass,
        PreparedPath firstPath,
        DepthSchedule depth,
        ProfileOptions options,
        CancellationToken cancellationToken,
        Func<int, List<Point3D>, List<ToolpathSpan>,
            (List<Point3D> Points, List<ToolpathSpan> Spans)>? applyTabs)
    {
        var points = new List<Point3D>(firstPath.Points);
        var spans = new List<ToolpathSpan>(firstPath.Spans);
        int plunges = firstPath.PlungeEntries;
        int linearRamps = firstPath.LinearRampEntries;
        int helicals = firstPath.HelicalEntries;

        for (int pass = firstPass + 1; pass < depth.PassCount; pass++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            Point3D previousEnd = points[^1];
            var passRing = InsertPoint(
                ring,
                new PointD(previousEnd.X, previousEnd.Y),
                rotate: true);
            var ramped = RampPlanner.PlanClosedRing(
                passRing,
                depth.PassTopZ(pass),
                depth.PassBottomZ(pass),
                options.Ramp);
            var passPoints = ramped.Points;
            var passSpans = ramped.Spans;
            if (applyTabs is not null)
                (passPoints, passSpans) = applyTabs(ringIndex, passPoints, passSpans);

            AppendPass(points, spans, passPoints, passSpans);
            AddEntry(ramped.Strategy, ref plunges, ref linearRamps, ref helicals);
        }

        return new PreparedPath(
            ringIndex,
            depth.PassCount - 1,
            points,
            spans,
            firstPath.Strategy,
            plunges,
            linearRamps,
            helicals);
    }

    private static void AppendPass(
        List<Point3D> points,
        List<ToolpathSpan> spans,
        IReadOnlyList<Point3D> passPoints,
        IReadOnlyList<ToolpathSpan> passSpans)
    {
        if (points.Count == 0 || passPoints.Count == 0)
            return;

        Point3D previous = points[^1];
        Point3D next = passPoints[0];
        if (Math.Abs(previous.X - next.X) > 1e-9
            || Math.Abs(previous.Y - next.Y) > 1e-9
            || Math.Abs(previous.Z - next.Z) > 1e-9)
        {
            throw new InvalidOperationException(
                "Consecutive profile depth passes do not share a stable seam.");
        }

        int indexOffset = points.Count - 1;
        for (int index = 1; index < passPoints.Count; index++)
            points.Add(passPoints[index]);
        foreach (var span in passSpans)
        {
            spans.Add(new ToolpathSpan(
                indexOffset + span.StartIndex,
                indexOffset + span.EndIndex,
                span.Kind));
        }
    }

    private static PreparedPath CreatePreparedPath(
        int ringIndex,
        int pass,
        List<Point3D> points,
        List<ToolpathSpan> spans,
        RampStrategy strategy,
        bool countEntry = true)
    {
        int plunges = 0, linearRamps = 0, helicals = 0;
        if (countEntry)
            AddEntry(strategy, ref plunges, ref linearRamps, ref helicals);
        return new PreparedPath(
            ringIndex,
            pass,
            points,
            spans,
            strategy,
            plunges,
            linearRamps,
            helicals);
    }

    private static void AddEntry(
        RampStrategy strategy,
        ref int plunges,
        ref int linearRamps,
        ref int helicals)
    {
        switch (strategy)
        {
            case RampStrategy.Helical: helicals++; break;
            case RampStrategy.Linear: linearRamps++; break;
            default: plunges++; break;
        }
    }

    private static PreparedPath Slice(
        int ringIndex,
        int pass,
        List<Point3D> sourcePoints,
        List<ToolpathSpan> sourceSpans,
        RampStrategy sourceStrategy,
        int start,
        int end,
        bool firstFragment,
        bool resumeAtDepth,
        bool enteredAtDepth,
        double entryZ)
    {
        var points = sourcePoints.GetRange(start, end - start + 1);
        int offset = 0;
        bool addEntry = !firstFragment && !resumeAtDepth;
        if (addEntry)
        {
            Point3D first = points[0];
            points.Insert(0, new Point3D(first.X, first.Y, entryZ));
            offset = 1;
        }

        var spans = new List<ToolpathSpan>();
        if (addEntry)
            spans.Add(new ToolpathSpan(0, 1, ToolpathSpanKind.Ramp));
        foreach (var span in sourceSpans)
        {
            int spanStart = Math.Max(start, span.StartIndex);
            int spanEnd = Math.Min(end, span.EndIndex);
            if (spanEnd <= spanStart)
                continue;
            spans.Add(new ToolpathSpan(
                spanStart - start + offset,
                spanEnd - start + offset,
                span.Kind));
        }

        return CreatePreparedPath(
            ringIndex,
            pass,
            points,
            spans,
            firstFragment || !addEntry ? sourceStrategy : RampStrategy.Plunge,
            countEntry: firstFragment ? !enteredAtDepth : addEntry);
    }

    private static PreparedPath PrependLink(PreparedPath path, Point3D from)
    {
        if (PointsCoincide(from, path.Points[0]))
        {
            path.Points[0] = from;
            return path;
        }

        var points = new List<Point3D>(path.Points.Count + 1) { from };
        points.AddRange(path.Points);
        var spans = new List<ToolpathSpan>(path.Spans.Count + 1)
        {
            new(0, 1, ToolpathSpanKind.Link),
        };
        spans.AddRange(path.Spans.Select(span => new ToolpathSpan(
            span.StartIndex + 1,
            span.EndIndex + 1,
            span.Kind)));
        return path with { Points = points, Spans = spans };
    }

    private static PreparedPath AppendLink(PreparedPath path, Point3D to)
    {
        if (PointsCoincide(path.Points[^1], to))
        {
            path.Points[^1] = to;
            return path;
        }

        var points = new List<Point3D>(path.Points.Count + 1);
        points.AddRange(path.Points);
        int linkStart = points.Count - 1;
        points.Add(to);
        var spans = new List<ToolpathSpan>(path.Spans)
        {
            new(linkStart, linkStart + 1, ToolpathSpanKind.Link),
        };
        return path with { Points = points, Spans = spans };
    }

    private static int FindSplitIndex(
        IReadOnlyList<Point3D> points,
        IReadOnlyList<ToolpathSpan> spans,
        int splitStart,
        double cutZ,
        PointD target,
        bool allowTabLift = false)
    {
        var eligible = new bool[points.Count];
        foreach (var span in spans)
        {
            bool isFullDepthCut = span.Kind == ToolpathSpanKind.Cut;
            bool isProtectedPause = allowTabLift && span.Kind == ToolpathSpanKind.TabLift;
            if (!isFullDepthCut && !isProtectedPause)
                continue;
            for (int index = Math.Max(splitStart, span.StartIndex);
                 index <= span.EndIndex && index < points.Count;
                 index++)
            {
                if (isProtectedPause || Math.Abs(points[index].Z - cutZ) <= 1e-6)
                    eligible[index] = true;
            }
        }

        int bestIndex = -1;
        double bestDistanceSquared = double.MaxValue;
        for (int index = splitStart; index < points.Count; index++)
        {
            if (!eligible[index])
                continue;
            double dx = points[index].X - target.x;
            double dy = points[index].Y - target.y;
            double distanceSquared = dx * dx + dy * dy;
            if (distanceSquared >= bestDistanceSquared)
                continue;
            bestDistanceSquared = distanceSquared;
            bestIndex = index;
        }

        if (bestIndex < 0)
            throw new InvalidOperationException("No safe profile point is available for an excursion.");
        return bestIndex;
    }

    private static List<PointD> InsertPoint(
        IReadOnlyList<PointD> ring,
        PointD target,
        bool rotate)
    {
        var loop = RampPlanner.Normalize(ring);
        int segmentIndex = 0;
        double segmentPosition = 0;
        PointD closest = loop[0];
        double bestDistanceSquared = double.MaxValue;
        for (int index = 0; index < loop.Count; index++)
        {
            PointD start = loop[index];
            PointD end = loop[(index + 1) % loop.Count];
            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double lengthSquared = dx * dx + dy * dy;
            double position = lengthSquared <= 1e-18
                ? 0
                : Math.Clamp(
                    ((target.x - start.x) * dx + (target.y - start.y) * dy) / lengthSquared,
                    0,
                    1);
            PointD candidate = new(start.x + dx * position, start.y + dy * position);
            double targetX = candidate.x - target.x;
            double targetY = candidate.y - target.y;
            double distanceSquared = targetX * targetX + targetY * targetY;
            if (distanceSquared >= bestDistanceSquared)
                continue;
            bestDistanceSquared = distanceSquared;
            segmentIndex = index;
            segmentPosition = position;
            closest = candidate;
        }

        int pointIndex;
        if (segmentPosition <= 1e-9)
        {
            pointIndex = segmentIndex;
        }
        else if (segmentPosition >= 1 - 1e-9)
        {
            pointIndex = (segmentIndex + 1) % loop.Count;
        }
        else
        {
            pointIndex = segmentIndex + 1;
            loop.Insert(pointIndex, closest);
        }
        return rotate ? RampPlanner.RotateTo(loop, pointIndex) : loop;
    }

    private static PointD ClosestPointOnRing(IReadOnlyList<PointD> ring, PointD target)
        => InsertPoint(ring, target, rotate: true)[0];

    private static bool PointsCoincide(Point3D first, Point3D second)
        => Math.Abs(first.X - second.X) <= 1e-6
            && Math.Abs(first.Y - second.Y) <= 1e-6
            && Math.Abs(first.Z - second.Z) <= 1e-6;

    private static double PlanarDistance(Point3D first, Point3D second)
    {
        double dx = second.X - first.X;
        double dy = second.Y - first.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    private static string FormatDistanceRange(DepthFirstState state)
        => state.LinkCandidates == 0
            ? "n/a"
            : $"{state.MinimumLinkDistance:F3}-{state.MaximumLinkDistance:F3}";

    private static double CutLength(
        IReadOnlyList<Point3D> points,
        IReadOnlyList<ToolpathSpan> spans)
    {
        double total = 0;
        foreach (var span in spans)
        {
            if (span.Kind is not (ToolpathSpanKind.Cut or ToolpathSpanKind.Link))
                continue;
            for (int index = span.StartIndex + 1;
                 index <= span.EndIndex && index < points.Count;
                 index++)
            {
                double dx = points[index].X - points[index - 1].X;
                double dy = points[index].Y - points[index - 1].Y;
                total += Math.Sqrt(dx * dx + dy * dy);
            }
        }
        return total;
    }
}