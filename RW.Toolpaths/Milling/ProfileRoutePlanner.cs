using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Source geometry owned by one profile-region index.</summary>
internal sealed record ProfileSourceComponent(
    int RegionIndex,
    IReadOnlyList<IReadOnlyList<PointD>> Boundaries);

/// <summary>A closest-point connector between two required contour cycles.</summary>
internal readonly record struct ProfilePortal(
    int FromRingIndex,
    PointD FromPoint,
    int ToRingIndex,
    PointD ToPoint,
    double Distance)
{
    internal ProfilePortal Reverse()
        => new(ToRingIndex, ToPoint, FromRingIndex, FromPoint, Distance);
}

/// <summary>One doubled-tree branch in the route-inspection solution.</summary>
internal sealed record ProfileRouteBranch(ProfilePortal Portal, ProfileRouteNode Child);

/// <summary>One required contour cycle and the excursions reached from it.</summary>
internal sealed class ProfileRouteNode
{
    internal ProfileRouteNode(int ringIndex) => RingIndex = ringIndex;

    internal int RingIndex { get; }

    internal List<ProfileRouteBranch> Children { get; } = new();
}

/// <summary>A connected required-edge component rooted at its largest contour.</summary>
internal sealed record ProfileRouteTree(ProfileRouteNode Root, IReadOnlyList<int> RingIndices);

/// <summary>The ordered route-inspection forest for one profile operation.</summary>
internal sealed record ProfileRouteForest(
    IReadOnlyList<ProfileRouteTree> Trees,
    int PortalCount,
    double SourceProximity);

/// <summary>
/// Plans profile travel as a metric Rural Postman approximation.
///
/// <para>
/// Each closed contour is already an Eulerian required component. Source components within one
/// tool radius are clustered, exact closest-point portals form the complete connector graph, and
/// Prim's minimum spanning tree selects the connector network. Doubling those tree connectors
/// makes every portal degree even; a depth-first traversal is therefore the same Euler tour that
/// Hierholzer's algorithm would produce on the augmented graph. The result naturally pauses a
/// large contour, services a nearby loop, returns through the paired portal, and resumes.
/// </para>
/// </summary>
internal static class ProfileRoutePlanner
{
    private readonly record struct BoundaryBounds(
        double MinX,
        double MinY,
        double MaxX,
        double MaxY);

    internal static ProfileRouteForest Plan(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<ProfileSourceComponent> sourceComponents,
        double toolRadius,
        CancellationToken cancellationToken)
    {
        long t0 = PerfLog.Start();
        if (rings.Count == 0)
            return new ProfileRouteForest(Array.Empty<ProfileRouteTree>(), 0, toolRadius);

        var ringClusters = BuildRingClusters(
            rings,
            sourceComponents,
            toolRadius + LinkClearance.DefaultTolerance,
            cancellationToken,
            out int sourcePairCandidates,
            out int sourcePairSolves);
        var trees = new List<ProfileRouteTree>(ringClusters.Count);
        int portalCount = 0;
        int portalSolves = 0;
        int portalFrontierChecks = 0;
        foreach (var ringCluster in ringClusters)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var tree = BuildMinimumSpanningTree(
                rings,
                ringCluster,
                cancellationToken,
                out int treePortalSolves,
                out int treePortalFrontierChecks);
            trees.Add(tree);
            portalCount += Math.Max(0, ringCluster.Count - 1);
            portalSolves += treePortalSolves;
            portalFrontierChecks += treePortalFrontierChecks;
        }

        if (trees.Count > 1)
        {
            var boundaries = trees
                .Select(tree => (IReadOnlyList<PointD>)tree.RingIndices
                    .SelectMany(index => rings[index].Points)
                    .ToList())
                .ToList();
            int[] order = TravelOptimizer.Order(
                boundaries,
                start: null,
                label: "profile-route-forest",
                cancellationToken);
            trees = order.Select(index => trees[index]).ToList();
        }

        PerfLog.Stop("ProfileRoutePlanner.Plan", t0,
            $"rings={rings.Count} clusters={trees.Count} portals={portalCount} " +
            $"sourcePairCandidates={sourcePairCandidates} sourcePairSolves={sourcePairSolves} " +
            $"portalSolves={portalSolves} portalFrontierChecks={portalFrontierChecks} " +
            $"sourceProximity={toolRadius:F3}");
        return new ProfileRouteForest(trees, portalCount, toolRadius);
    }

    private static List<List<int>> BuildRingClusters(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<ProfileSourceComponent> sourceComponents,
        double proximity,
        CancellationToken cancellationToken,
        out int sourcePairCandidates,
        out int sourcePairSolves)
    {
        sourcePairCandidates = 0;
        sourcePairSolves = 0;
        var regionIndices = rings
            .Select(ring => ring.RegionIndex)
            .Distinct()
            .OrderBy(index => index)
            .ToList();
        var regionPosition = regionIndices
            .Select((regionIndex, position) => (regionIndex, position))
            .ToDictionary(entry => entry.regionIndex, entry => entry.position);
        var sourceByRegion = sourceComponents.ToDictionary(component => component.RegionIndex);
        var sourceBoundsByRegion = sourceComponents.ToDictionary(
            component => component.RegionIndex,
            component => GetBounds(component.Boundaries));
        var parent = Enumerable.Range(0, regionIndices.Count).ToArray();

        int Find(int index)
        {
            while (parent[index] != index)
            {
                parent[index] = parent[parent[index]];
                index = parent[index];
            }
            return index;
        }

        void Union(int first, int second)
        {
            int firstRoot = Find(first);
            int secondRoot = Find(second);
            if (firstRoot != secondRoot)
                parent[secondRoot] = firstRoot;
        }

        double proximitySquared = proximity * proximity;
        for (int first = 0; first < regionIndices.Count; first++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (!sourceByRegion.TryGetValue(regionIndices[first], out var firstSource))
                continue;

            for (int second = first + 1; second < regionIndices.Count; second++)
            {
                if (!sourceByRegion.TryGetValue(regionIndices[second], out var secondSource))
                    continue;
                sourcePairCandidates++;
                if (BoundsDistanceSquared(
                        sourceBoundsByRegion[regionIndices[first]],
                        sourceBoundsByRegion[regionIndices[second]]) > proximitySquared)
                {
                    continue;
                }
                sourcePairSolves++;
                if (ClosestBoundaryDistanceSquared(firstSource.Boundaries, secondSource.Boundaries)
                    <= proximitySquared)
                {
                    Union(first, second);
                }
            }
        }

        var clustersByRoot = new Dictionary<int, List<int>>();
        foreach (int regionIndex in regionIndices)
        {
            int root = Find(regionPosition[regionIndex]);
            if (!clustersByRoot.TryGetValue(root, out var cluster))
            {
                cluster = new List<int>();
                clustersByRoot.Add(root, cluster);
            }
            cluster.AddRange(Enumerable.Range(0, rings.Count)
                .Where(ringIndex => rings[ringIndex].RegionIndex == regionIndex));
        }

        return clustersByRoot.Values
            .Select(cluster => cluster.Distinct().OrderBy(index => index).ToList())
            .OrderBy(cluster => cluster.Min())
            .ToList();
    }

    private static BoundaryBounds GetBounds(
        IReadOnlyList<IReadOnlyList<PointD>> boundaries)
    {
        double minX = double.MaxValue;
        double minY = double.MaxValue;
        double maxX = double.MinValue;
        double maxY = double.MinValue;
        foreach (var point in boundaries.SelectMany(boundary => boundary))
        {
            minX = Math.Min(minX, point.x);
            minY = Math.Min(minY, point.y);
            maxX = Math.Max(maxX, point.x);
            maxY = Math.Max(maxY, point.y);
        }
        return new BoundaryBounds(minX, minY, maxX, maxY);
    }

    private static double BoundsDistanceSquared(BoundaryBounds first, BoundaryBounds second)
    {
        double dx = Math.Max(0, Math.Max(first.MinX - second.MaxX, second.MinX - first.MaxX));
        double dy = Math.Max(0, Math.Max(first.MinY - second.MaxY, second.MinY - first.MaxY));
        return dx * dx + dy * dy;
    }

    private static ProfileRouteTree BuildMinimumSpanningTree(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<int> ringIndices,
        CancellationToken cancellationToken,
        out int portalSolves,
        out int portalFrontierChecks)
    {
        portalSolves = 0;
        portalFrontierChecks = 0;
        int rootRingIndex = ringIndices
            .OrderByDescending(index => rings[index].Area)
            .ThenBy(index => index)
            .First();
        var nodes = ringIndices.ToDictionary(index => index, index => new ProfileRouteNode(index));
        var visited = new HashSet<int> { rootRingIndex };
        var frontier = new Dictionary<int, ProfilePortal>();
        int addedRingIndex = rootRingIndex;

        while (visited.Count < ringIndices.Count)
        {
            cancellationToken.ThrowIfCancellationRequested();
            foreach (int toRingIndex in ringIndices.Where(index => !visited.Contains(index)))
            {
                ProfilePortal candidate = FindClosestPortal(
                    addedRingIndex,
                    rings[addedRingIndex].Points,
                    toRingIndex,
                    rings[toRingIndex].Points);
                portalSolves++;
                if (!frontier.TryGetValue(toRingIndex, out var current)
                    || IsBetterPortal(candidate, current))
                {
                    frontier[toRingIndex] = candidate;
                }
            }

            ProfilePortal? bestPortal = null;
            foreach (int toRingIndex in ringIndices.Where(index => !visited.Contains(index)))
            {
                ProfilePortal candidate = frontier[toRingIndex];
                portalFrontierChecks++;
                if (bestPortal is null || IsBetterPortal(candidate, bestPortal.Value))
                    bestPortal = candidate;
            }

            if (bestPortal is null)
                throw new InvalidOperationException("Profile connector graph is unexpectedly disconnected.");

            ProfilePortal portal = bestPortal.Value;
            nodes[portal.FromRingIndex].Children.Add(
                new ProfileRouteBranch(portal, nodes[portal.ToRingIndex]));
            visited.Add(portal.ToRingIndex);
            frontier.Remove(portal.ToRingIndex);
            addedRingIndex = portal.ToRingIndex;
        }

        return new ProfileRouteTree(nodes[rootRingIndex], ringIndices.ToList());
    }

    private static bool IsBetterPortal(ProfilePortal candidate, ProfilePortal current)
        => candidate.Distance < current.Distance - 1e-9
            || (Math.Abs(candidate.Distance - current.Distance) <= 1e-9
                && (candidate.FromRingIndex < current.FromRingIndex
                    || (candidate.FromRingIndex == current.FromRingIndex
                        && candidate.ToRingIndex < current.ToRingIndex)));

    internal static ProfilePortal FindClosestPortal(
        int firstRingIndex,
        IReadOnlyList<PointD> first,
        int secondRingIndex,
        IReadOnlyList<PointD> second)
    {
        PointD bestFirst = first[0];
        PointD bestSecond = second[0];
        double bestDistanceSquared = double.MaxValue;

        for (int firstSegment = 0; firstSegment < first.Count; firstSegment++)
        {
            PointD firstStart = first[firstSegment];
            PointD firstEnd = first[(firstSegment + 1) % first.Count];
            for (int secondSegment = 0; secondSegment < second.Count; secondSegment++)
            {
                PointD secondStart = second[secondSegment];
                PointD secondEnd = second[(secondSegment + 1) % second.Count];

                if (TrySegmentIntersection(
                        firstStart,
                        firstEnd,
                        secondStart,
                        secondEnd,
                        out double firstPosition,
                        out _))
                {
                    PointD intersection = Interpolate(firstStart, firstEnd, firstPosition);
                    return new ProfilePortal(
                        firstRingIndex,
                        intersection,
                        secondRingIndex,
                        intersection,
                        0);
                }

                Consider(firstStart, Project(firstStart, secondStart, secondEnd));
                Consider(firstEnd, Project(firstEnd, secondStart, secondEnd));
                Consider(Project(secondStart, firstStart, firstEnd), secondStart);
                Consider(Project(secondEnd, firstStart, firstEnd), secondEnd);
            }
        }

        return new ProfilePortal(
            firstRingIndex,
            bestFirst,
            secondRingIndex,
            bestSecond,
            Math.Sqrt(bestDistanceSquared));

        void Consider(PointD firstPoint, PointD secondPoint)
        {
            double dx = firstPoint.x - secondPoint.x;
            double dy = firstPoint.y - secondPoint.y;
            double distanceSquared = dx * dx + dy * dy;
            if (distanceSquared >= bestDistanceSquared)
                return;
            bestDistanceSquared = distanceSquared;
            bestFirst = firstPoint;
            bestSecond = secondPoint;
        }
    }

    private static double ClosestBoundaryDistanceSquared(
        IReadOnlyList<IReadOnlyList<PointD>> first,
        IReadOnlyList<IReadOnlyList<PointD>> second)
    {
        double best = double.MaxValue;
        foreach (var firstBoundary in first)
        {
            foreach (var secondBoundary in second)
            {
                ProfilePortal portal = FindClosestPortal(0, firstBoundary, 1, secondBoundary);
                best = Math.Min(best, portal.Distance * portal.Distance);
            }
        }
        return best;
    }

    private static PointD Project(PointD point, PointD start, PointD end)
    {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared <= 1e-18)
            return start;
        double position = Math.Clamp(
            ((point.x - start.x) * dx + (point.y - start.y) * dy) / lengthSquared,
            0,
            1);
        return Interpolate(start, end, position);
    }

    private static bool TrySegmentIntersection(
        PointD firstStart,
        PointD firstEnd,
        PointD secondStart,
        PointD secondEnd,
        out double firstPosition,
        out double secondPosition)
    {
        double firstX = firstEnd.x - firstStart.x;
        double firstY = firstEnd.y - firstStart.y;
        double secondX = secondEnd.x - secondStart.x;
        double secondY = secondEnd.y - secondStart.y;
        double denominator = firstX * secondY - firstY * secondX;
        if (Math.Abs(denominator) < 1e-12)
        {
            firstPosition = 0;
            secondPosition = 0;
            return false;
        }

        double offsetX = secondStart.x - firstStart.x;
        double offsetY = secondStart.y - firstStart.y;
        firstPosition = (offsetX * secondY - offsetY * secondX) / denominator;
        secondPosition = (offsetX * firstY - offsetY * firstX) / denominator;
        return firstPosition >= 0 && firstPosition <= 1
            && secondPosition >= 0 && secondPosition <= 1;
    }

    private static PointD Interpolate(PointD start, PointD end, double position)
        => new(
            start.x + (end.x - start.x) * position,
            start.y + (end.y - start.y) * position);
}