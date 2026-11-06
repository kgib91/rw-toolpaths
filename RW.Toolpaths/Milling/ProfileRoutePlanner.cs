using Clipper2Lib;
using System.Diagnostics;

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
    internal sealed class PortalSearchMetrics
    {
        public long SegmentPairsConsidered;
        public long SegmentPairsPruned;
        public long SourceSegmentPairsConsidered,SourceSegmentPairsPruned,SourcePortalSolveTicks;
        public long IndexBuilds,IndexQueries,IndexNodeVisits,IndexCandidates,IndexRejectedPairs,IndexFallbackSolves;
        public long IndexReservedBytes,IndexBuildTicks,PortalSolveTicks;
    }

    private readonly record struct BoundaryBounds(
        double MinX,
        double MinY,
        double MaxX,
        double MaxY);

    internal const long MaximumSegmentIndexBytes=64L<<20;
    private const int MinimumIndexedSegments=32;

    // Portal geometry is literal double precision. The shared polygon index
    // quantizes to integer machining coordinates, so it cannot own these ties.
    private sealed class SegmentIndex
    {
        private readonly record struct Entry(BoundaryBounds Bounds,int Ordinal);
        private readonly record struct Node(BoundaryBounds Bounds,int First,int Count,int Left,int Right);
        internal BoundaryBounds[] OriginalBounds{get;}
        private readonly Entry[] entries;
        private readonly Node[] nodes;
        internal static long Reservation(int count)=>256L+128L*count;
        internal SegmentIndex(IReadOnlyList<PointD> points)
        {
            OriginalBounds=new BoundaryBounds[points.Count];entries=new Entry[points.Count];
            for(int i=0;i<points.Count;i++)
            {
                OriginalBounds[i]=GetSegmentBounds(points[i],points[(i+1)%points.Count]);
                entries[i]=new(PortalArithmeticEnvelope(OriginalBounds[i]),i);
            }
            var built=new List<Node>();Build(0,entries.Length);nodes=built.ToArray();
            int Build(int first,int count)
            {
                var bounds=entries[first].Bounds;
                for(int i=first+1;i<first+count;i++)bounds=Union(bounds,entries[i].Bounds);
                int index=built.Count;built.Add(default);
                if(count<=16){built[index]=new(bounds,first,count,-1,-1);return index;}
                bool x=bounds.MaxX-bounds.MinX>=bounds.MaxY-bounds.MinY;
                Array.Sort(entries,first,count,Comparer<Entry>.Create((a,b)=>{
                    double av=x?a.Bounds.MinX/2+a.Bounds.MaxX/2:a.Bounds.MinY/2+a.Bounds.MaxY/2;
                    double bv=x?b.Bounds.MinX/2+b.Bounds.MaxX/2:b.Bounds.MinY/2+b.Bounds.MaxY/2;
                    int order=av.CompareTo(bv);return order!=0?order:a.Ordinal.CompareTo(b.Ordinal);
                }));
                int half=count/2,left=Build(first,half),right=Build(first+half,count-half);
                built[index]=new(bounds,first,0,left,right);return index;
            }
        }
        internal void Query(BoundaryBounds first,double best,List<int> ordinals,PortalSearchMetrics? metrics)
        {
            ordinals.Clear();if(metrics is not null)metrics.IndexQueries++;
            Visit(0);ordinals.Sort();
            if(metrics is not null){metrics.IndexCandidates+=ordinals.Count;metrics.IndexRejectedPairs+=entries.Length-ordinals.Count;}
            void Visit(int ordinal)
            {
                if(metrics is not null)metrics.IndexNodeVisits++;
                var node=nodes[ordinal];
                if(ConservativeBoundsDistanceSquared(first,node.Bounds)>=best)return;
                if(node.Count==0){Visit(node.Left);Visit(node.Right);return;}
                for(int i=node.First;i<node.First+node.Count;i++)
                    if(ConservativeBoundsDistanceSquared(first,entries[i].Bounds)<best)ordinals.Add(entries[i].Ordinal);
            }
        }
        private static BoundaryBounds Union(BoundaryBounds a,BoundaryBounds b)=>new(
            Math.Min(a.MinX,b.MinX),Math.Min(a.MinY,b.MinY),Math.Max(a.MaxX,b.MaxX),Math.Max(a.MaxY,b.MaxY));
    }

    private sealed class PortalIndexOwner(long limit,PortalSearchMetrics metrics)
    {
        private sealed class Entry
        {
            internal SegmentIndex? Index;
            internal bool Attempted,CheckedDomain,Eligible;
        }
        private readonly Dictionary<IReadOnlyList<PointD>,Entry> entries=new(ReferenceEqualityComparer.Instance);
        internal readonly List<int> Scratch=new();
        private Entry? GetEntry(IReadOnlyList<PointD> points)
        {
            if(entries.TryGetValue(points,out var entry))return entry;
            // Covers metadata objects and dictionary resize scratch as well.
            if(256>limit-metrics.IndexReservedBytes)return null;
            entry=new();entries.Add(points,entry);metrics.IndexReservedBytes+=256;return entry;
        }
        internal bool Eligible(IReadOnlyList<PointD> points)
        {
            var entry=GetEntry(points);if(entry is null)return false;
            if(!entry.CheckedDomain){entry.Eligible=FinitePortalDomain(points);entry.CheckedDomain=true;}
            return entry.Eligible;
        }
        internal SegmentIndex? Get(IReadOnlyList<PointD> points)
        {
            var entry=GetEntry(points);if(entry is null)return null;
            if(entry.Attempted)return entry.Index;entry.Attempted=true;
            long bytes=SegmentIndex.Reservation(points.Count);
            if(points.Count<MinimumIndexedSegments||!Eligible(points)||bytes>limit-metrics.IndexReservedBytes)return null;
            long started=Stopwatch.GetTimestamp();var index=new SegmentIndex(points);
            metrics.IndexBuildTicks+=Stopwatch.GetTimestamp()-started;metrics.IndexBuilds++;metrics.IndexReservedBytes+=bytes;
            return entry.Index=index;
        }
    }

    private static bool FinitePortalDomain(IReadOnlyList<PointD> points)=>points.Count>0
        &&points.All(p=>double.IsFinite(p.x)&&double.IsFinite(p.y)&&Math.Abs(p.x)<=1e12&&Math.Abs(p.y)<=1e12);

    private static BoundaryBounds PortalArithmeticEnvelope(BoundaryBounds bounds)
    {
        // This bounds the ORIGINAL leaf-box predicate, not an ideal projection.
        // Outward boxes plus monotone nonnegative arithmetic and downward
        // rounding cannot exceed that predicate for any enclosed leaf. The
        // unchanged leaf predicate therefore skips every omitted pair too.
        // The finite domain also keeps the legacy best distance finite and
        // nonincreasing; unsupported domains retain the literal full scan.
        return new(Math.BitDecrement(bounds.MinX),Math.BitDecrement(bounds.MinY),
            Math.BitIncrement(bounds.MaxX),Math.BitIncrement(bounds.MaxY));
    }
    private static double ConservativeBoundsDistanceSquared(BoundaryBounds a,BoundaryBounds b)
    {
        static double Down(double x)=>x>0&&double.IsFinite(x)?Math.Max(0,Math.BitDecrement(x)):0;
        double x=Math.Max(Down(a.MinX-b.MaxX),Down(b.MinX-a.MaxX));
        double y=Math.Max(Down(a.MinY-b.MaxY),Down(b.MinY-a.MaxY));
        return Down(Down(x*x)+Down(y*y));
    }

    internal static ProfileRouteForest Plan(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<ProfileSourceComponent> sourceComponents,
        double toolRadius,
        CancellationToken cancellationToken,bool useSegmentIndex=true,long maximumSegmentIndexBytes=MaximumSegmentIndexBytes)
    {
        if(maximumSegmentIndexBytes<0||maximumSegmentIndexBytes>MaximumSegmentIndexBytes)throw new ArgumentOutOfRangeException(nameof(maximumSegmentIndexBytes));
        long t0 = PerfLog.Start();
        if (rings.Count == 0)
            return new ProfileRouteForest(Array.Empty<ProfileRouteTree>(), 0, toolRadius);

        var portalSearchMetrics = new PortalSearchMetrics();
        PortalIndexOwner? portalIndices=null;
        if(useSegmentIndex&&maximumSegmentIndexBytes>=256)
        {
            portalSearchMetrics.IndexReservedBytes=256;
            portalIndices=new(maximumSegmentIndexBytes,portalSearchMetrics);
        }
        var ringClusters = BuildRingClusters(
            rings,
            sourceComponents,
            toolRadius + LinkClearance.DefaultTolerance,
            cancellationToken,
            portalIndices,portalSearchMetrics,
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
                portalSearchMetrics,portalIndices,
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
            $"segmentPairs={portalSearchMetrics.SegmentPairsConsidered} " +
            $"segmentPairsPruned={portalSearchMetrics.SegmentPairsPruned} " +
            $"sourceSegmentPairs={portalSearchMetrics.SourceSegmentPairsConsidered} sourceSegmentPairsPruned={portalSearchMetrics.SourceSegmentPairsPruned} sourcePortalSolveMs={portalSearchMetrics.SourcePortalSolveTicks*1000.0/Stopwatch.Frequency:F3} " +
            $"segmentIndexBuilds={portalSearchMetrics.IndexBuilds} segmentIndexQueries={portalSearchMetrics.IndexQueries} segmentIndexNodeVisits={portalSearchMetrics.IndexNodeVisits} " +
            $"segmentIndexCandidates={portalSearchMetrics.IndexCandidates} segmentIndexRejectedPairs={portalSearchMetrics.IndexRejectedPairs} segmentIndexFallbackSolves={portalSearchMetrics.IndexFallbackSolves} " +
            $"segmentIndexBuildMs={portalSearchMetrics.IndexBuildTicks*1000.0/Stopwatch.Frequency:F3} portalSolveMs={portalSearchMetrics.PortalSolveTicks*1000.0/Stopwatch.Frequency:F3} segmentIndexReservedBytes={portalSearchMetrics.IndexReservedBytes} segmentIndexByteLimit={maximumSegmentIndexBytes} segmentOrdinalOrder=original physicalAuthorityUnchanged=True " +
            $"sourceProximity={toolRadius:F3}");
        return new ProfileRouteForest(trees, portalCount, toolRadius);
    }

    private static List<List<int>> BuildRingClusters(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<ProfileSourceComponent> sourceComponents,
        double proximity,
        CancellationToken cancellationToken,
        PortalIndexOwner? portalIndices,PortalSearchMetrics portalSearchMetrics,
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
                if (ClosestBoundaryDistanceSquared(firstSource.Boundaries, secondSource.Boundaries,portalIndices,portalSearchMetrics)
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
        PortalSearchMetrics portalSearchMetrics,PortalIndexOwner? portalIndices,
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
                var secondIndex=portalIndices is not null&&portalIndices.Eligible(rings[addedRingIndex].Points)?portalIndices.Get(rings[toRingIndex].Points):null;
                ProfilePortal candidate = FindClosestPortalCore(
                    addedRingIndex,
                    rings[addedRingIndex].Points,
                    toRingIndex,
                    rings[toRingIndex].Points,
                    portalSearchMetrics,secondIndex,portalIndices?.Scratch);
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
        IReadOnlyList<PointD> second,
        PortalSearchMetrics? portalSearchMetrics = null,bool useSegmentIndex=true,
        long maximumSegmentIndexBytes=MaximumSegmentIndexBytes)
    {
        if(maximumSegmentIndexBytes<0||maximumSegmentIndexBytes>MaximumSegmentIndexBytes)throw new ArgumentOutOfRangeException(nameof(maximumSegmentIndexBytes));
        SegmentIndex? index=null;
        if(useSegmentIndex&&second.Count>=MinimumIndexedSegments&&FinitePortalDomain(first)&&FinitePortalDomain(second)
            &&SegmentIndex.Reservation(second.Count)<=maximumSegmentIndexBytes)
        {
            long started=Stopwatch.GetTimestamp();index=new(second);
            if(portalSearchMetrics is not null){portalSearchMetrics.IndexBuildTicks+=Stopwatch.GetTimestamp()-started;
                portalSearchMetrics.IndexBuilds++;portalSearchMetrics.IndexReservedBytes+=SegmentIndex.Reservation(second.Count);}
        }
        return FindClosestPortalCore(firstRingIndex,first,secondRingIndex,second,portalSearchMetrics,index,null);
    }

    private static ProfilePortal FindClosestPortalCore(int firstRingIndex,IReadOnlyList<PointD> first,
        int secondRingIndex,IReadOnlyList<PointD> second,PortalSearchMetrics? portalSearchMetrics,
        SegmentIndex? index,List<int>? scratch,bool sourceClustering=false)
    {
        long solveStarted=Stopwatch.GetTimestamp();
        try
        {
        PointD bestFirst = first[0];
        PointD bestSecond = second[0];
        double bestDistanceSquared = double.MaxValue;
        var secondSegments=index?.OriginalBounds??new BoundaryBounds[second.Count];
        if(index is null)
        {
            if(portalSearchMetrics is not null)portalSearchMetrics.IndexFallbackSolves++;
            for (int secondSegment = 0; secondSegment < second.Count; secondSegment++)
            {
                PointD secondStart = second[secondSegment];
                PointD secondEnd = second[(secondSegment + 1) % second.Count];
                secondSegments[secondSegment] = GetSegmentBounds(secondStart, secondEnd);
            }
        }
        else scratch??=new List<int>();

        for (int firstSegment = 0; firstSegment < first.Count; firstSegment++)
        {
            PointD firstStart = first[firstSegment];
            PointD firstEnd = first[(firstSegment + 1) % first.Count];
            BoundaryBounds firstBounds = GetSegmentBounds(firstStart, firstEnd);
            if(index is not null)index.Query(PortalArithmeticEnvelope(firstBounds),bestDistanceSquared,scratch!,portalSearchMetrics);
            int nominations=index is null?second.Count:scratch!.Count;
            for (int candidateOrdinal = 0; candidateOrdinal < nominations; candidateOrdinal++)
            {
                int secondSegment=index is null?candidateOrdinal:scratch![candidateOrdinal];
                PointD secondStart = second[secondSegment];
                PointD secondEnd = second[(secondSegment + 1) % second.Count];

                if (portalSearchMetrics != null)
                {if(sourceClustering)portalSearchMetrics.SourceSegmentPairsConsidered++;else portalSearchMetrics.SegmentPairsConsidered++;}
                if (BoundsDistanceSquared(firstBounds, secondSegments[secondSegment]) >= bestDistanceSquared)
                {
                    if (portalSearchMetrics != null)
                    {if(sourceClustering)portalSearchMetrics.SourceSegmentPairsPruned++;else portalSearchMetrics.SegmentPairsPruned++;}
                    continue;
                }

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
        finally{if(portalSearchMetrics is not null){long ticks=Stopwatch.GetTimestamp()-solveStarted;
            if(sourceClustering)portalSearchMetrics.SourcePortalSolveTicks+=ticks;else portalSearchMetrics.PortalSolveTicks+=ticks;}}
    }

    private static double ClosestBoundaryDistanceSquared(
        IReadOnlyList<IReadOnlyList<PointD>> first,
        IReadOnlyList<IReadOnlyList<PointD>> second,PortalIndexOwner? portalIndices,PortalSearchMetrics metrics)
    {
        double best = double.MaxValue;
        foreach (var firstBoundary in first)
        {
            foreach (var secondBoundary in second)
            {
                var index=portalIndices is not null&&portalIndices.Eligible(firstBoundary)?portalIndices.Get(secondBoundary):null;
                ProfilePortal portal = FindClosestPortalCore(0,firstBoundary,1,secondBoundary,metrics,index,
                    portalIndices?.Scratch,sourceClustering:true);
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

    private static BoundaryBounds GetSegmentBounds(PointD start, PointD end)
        => new(
            Math.Min(start.x, end.x),
            Math.Min(start.y, end.y),
            Math.Max(start.x, end.x),
            Math.Max(start.y, end.y));

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
