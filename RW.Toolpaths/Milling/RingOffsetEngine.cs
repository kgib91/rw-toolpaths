using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>One closed ring produced by the offset engine, plus its place in the nesting tree.</summary>
public sealed class OffsetRing
{
    /// <summary>Takes ownership of <paramref name="points"/> and normalizes it to CCW winding.</summary>
    internal OffsetRing(List<PointD> points, int level, int regionIndex, bool? isHole = null)
    {
        double area = PolygonOps.SignedArea(points);
        IsHole = isHole ?? area < 0;
        if (area < 0)
        {
            points.Reverse();
            area = -area;
        }

        Points = points;
        Level = level;
        RegionIndex = regionIndex;
        Area = area;
        Perimeter = PolygonOps.Perimeter(points);
        Centroid = PolygonOps.Centroid(points);
    }

    public List<PointD> Points { get; }

    /// <summary>Inward offset steps from the source boundary; 0 is the outermost ring.</summary>
    public int Level { get; }

    /// <summary>Index of the source <see cref="MillingRegion"/> this ring came from.</summary>
    public int RegionIndex { get; }

    /// <summary>
    /// Whether this contour bounded a hole before its cutting direction was normalized.
    /// </summary>
    public bool IsHole { get; }

    public double Perimeter { get; }

    public double Area { get; }

    public PointD Centroid { get; }

    /// <summary>Index of the enclosing ring one level out, or -1 when this ring is a root.</summary>
    public int Parent { get; internal set; } = -1;

    public List<int> Children { get; } = new();
}

/// <summary>Ring visit order plus the index ranges that make up each independent island.</summary>
public sealed class RingTraversal
{
    internal RingTraversal(IReadOnlyList<int> order, IReadOnlyList<(int StartIndex, int Count)> islands)
    {
        Order = order;
        Islands = islands;
    }

    public IReadOnlyList<int> Order { get; }

    /// <summary>Slices of <see cref="Order"/>, one per root ring, i.e. one per island.</summary>
    public IReadOnlyList<(int StartIndex, int Count)> Islands { get; }
}

/// <summary>
/// A run of rings that nest one inside the next with no branching, so the cutter can spiral
/// through them continuously instead of retracting and re-entering for each ring.
/// </summary>
public sealed class RingChain
{
    internal RingChain(List<int> ringIndices) => RingIndices = ringIndices;

    public List<int> RingIndices { get; }
}

/// <summary>Inputs to <see cref="RingOffsetEngine.BuildRings"/>.</summary>
/// <param name="FirstOffset">Offset of the first ring from the boundary, normally tool radius + stock to leave.</param>
/// <param name="StepOver">Radial distance between successive rings.</param>
public sealed record RingOffsetOptions(double FirstOffset, double StepOver)
{
    /// <summary>Hard cap on rings per region.</summary>
    public int RingLimit { get; init; } = 10_000;

    /// <summary>Offset direction; negative shrinks inward (pocketing), positive grows outward.</summary>
    public double OffsetSign { get; init; } = -1.0;

    public GeometryTolerances Tolerances { get; init; } = GeometryTolerances.Default;
}

/// <summary>
/// Builds the concentric ring set that both pocketing and profiling are made of, works out how
/// the rings nest, and decides what order to cut them in.
///
/// <para>
/// Rings are offset from the <em>original</em> boundary at increasing distance rather than by
/// re-offsetting the previous ring. Re-offsetting compounds Clipper's rounding at every step, so
/// ring spacing slowly drifts away from the requested stepover and cutter engagement drifts with
/// it. Offsetting from the source keeps every ring exactly where it was asked to be.
/// </para>
/// </summary>
public static class RingOffsetEngine
{
    /// <summary>Absolute ceiling on rings per region, independent of caller configuration.</summary>
    public const int AbsoluteRingLimit = 10_000;

    // --- Ring construction ----------------------------------------------------

    /// <summary>
    /// Generates concentric rings for every region. Stops a region early when its rings stop
    /// shrinking, which is the signal that Clipper has collapsed the geometry rather than
    /// produced a genuine inner ring.
    /// </summary>
    public static List<OffsetRing> BuildRings(
        IReadOnlyList<MillingRegion> regions,
        RingOffsetOptions options,
        CancellationToken cancellationToken = default)
    {
        long t0 = PerfLog.Start();
        var rings = new List<OffsetRing>();

        double stepOver = options.StepOver;
        int ringLimit = Math.Clamp(options.RingLimit, 1, AbsoluteRingLimit);
        if (stepOver <= 1e-9)
            ringLimit = 1;

        double minArea = stepOver > 1e-9 ? stepOver * stepOver * 0.01 : 0;

        for (int regionIndex = 0; regionIndex < regions.Count; regionIndex++)
        {
            long regionStart = PerfLog.Start();
            cancellationToken.ThrowIfCancellationRequested();

            var region = regions[regionIndex];
            var preparedOffset = PolygonOps.PrepareBoundaryOffset(
                region.Outer,
                region.Holes,
                options.Tolerances.ArcTolerance,
                options.Tolerances.SimplifyTolerance);
            int regionRingStart = rings.Count;
            int generatedLevels = 0;
            string stopReason = "limit";
            double previousLevelArea = double.MaxValue;
            int areaGrowthLevels = 0;
            double maximumAreaGrowth = 0;

            for (int level = 0; level < ringLimit; level++)
            {
                cancellationToken.ThrowIfCancellationRequested();

                double distance = options.FirstOffset + level * stepOver;
                var polygons = preparedOffset.Execute(options.OffsetSign * distance);

                if (polygons.Count == 0)
                {
                    stopReason = "consumed";
                    break;
                }

                double levelArea = Math.Abs(polygons.Sum(PolygonOps.SignedArea));

                // Filled material must shrink monotonically as we move inward. Hole contours
                // expand during an inward offset, so summing each contour's absolute area can
                // grow even while the machinable region is shrinking normally.
                double areaTolerance = previousLevelArea == double.MaxValue
                    ? 0
                    : Math.Max(minArea, previousLevelArea * 1e-9);
                if (levelArea > previousLevelArea + areaTolerance)
                {
                    areaGrowthLevels++;
                    maximumAreaGrowth = Math.Max(
                        maximumAreaGrowth,
                        levelArea - previousLevelArea);
                }
                previousLevelArea = levelArea;
                generatedLevels++;

                foreach (var polygon in polygons)
                {
                    if (polygon.Count < 3)
                        continue;
                    if (Math.Abs(PolygonOps.SignedArea(polygon)) < minArea)
                        continue;

                    rings.Add(new OffsetRing(polygon, level, regionIndex));
                }
            }

            PerfLog.Stop("RingOffsetEngine.RegionOffsets", regionStart,
                $"region={regionIndex} sourcePaths={preparedOffset.SourcePathCount} " +
                $"rings={rings.Count - regionRingStart} levels={generatedLevels} " +
                $"stop={stopReason} netArea={previousLevelArea:F3} " +
                $"areaGrowthLevels={areaGrowthLevels} maxAreaGrowth={maximumAreaGrowth:F3} " +
                $"sourcePoints={preparedOffset.SourcePointCount} offsets={preparedOffset.OffsetCount} " +
                $"fallbacks={preparedOffset.FallbackCount} " +
                $"prepareMs={preparedOffset.PreparationMilliseconds:F2} " +
                $"executeMs={preparedOffset.OffsetExecutionMilliseconds:F2} " +
                $"validateMs={preparedOffset.ValidationMilliseconds:F2} " +
                $"fallbackMs={preparedOffset.FallbackMilliseconds:F2} " +
                $"materializeMs={preparedOffset.MaterializationMilliseconds:F2}");
        }

        PerfLog.Stop("RingOffsetEngine.BuildRings", t0,
            $"regions={regions.Count} rings={rings.Count} step={stepOver:F4} limit={ringLimit}");

        return rings;
    }

    /// <summary>
    /// Wraps a pre-built polygon as a ring without offsetting, for callers that generate their
    /// rings elsewhere (profile passes and spring passes).
    /// </summary>
    public static OffsetRing CreateRing(
        IReadOnlyList<PointD> polygon,
        int level,
        int regionIndex,
        bool? isHole = null)
        => new(new List<PointD>(polygon), level, regionIndex, isHole);

    // --- Nesting --------------------------------------------------------------

    /// <summary>
    /// Links each ring to the enclosing ring exactly one level out in the same region, choosing
    /// the closest candidate by boundary proximity. Rings at the same level in the same region
    /// never nest inside each other, so proximity resolves the parent unambiguously and far more
    /// cheaply than a full containment test.
    /// </summary>
    public static void BuildNesting(IReadOnlyList<OffsetRing> rings)
    {
        long t0 = PerfLog.Start();

        foreach (var ring in rings)
        {
            ring.Parent = -1;
            ring.Children.Clear();
        }

        var ringsByRegionAndLevel = new Dictionary<(int RegionIndex, int Level), List<int>>();
        for (int ringIndex = 0; ringIndex < rings.Count; ringIndex++)
        {
            var ring = rings[ringIndex];
            var key = (ring.RegionIndex, ring.Level);
            if (!ringsByRegionAndLevel.TryGetValue(key, out var levelRings))
            {
                levelRings = new List<int>();
                ringsByRegionAndLevel.Add(key, levelRings);
            }
            levelRings.Add(ringIndex);
        }

        long candidateChecks = 0;

        for (int i = 0; i < rings.Count; i++)
        {
            var ring = rings[i];
            if (ring.Level == 0)
                continue;

            if (!ringsByRegionAndLevel.TryGetValue(
                    (ring.RegionIndex, ring.Level - 1),
                    out var parentCandidates))
            {
                continue;
            }

            int bestParent = -1;
            double bestDistanceSq = double.MaxValue;

            foreach (int j in parentCandidates)
            {
                var candidate = rings[j];
                candidateChecks++;
                double distanceSq = ClosestApproachSq(ring.Points, candidate.Points);
                if (distanceSq < bestDistanceSq)
                {
                    bestDistanceSq = distanceSq;
                    bestParent = j;
                }
            }

            ring.Parent = bestParent;
            if (bestParent >= 0)
                rings[bestParent].Children.Add(i);
        }

        PerfLog.Stop("RingOffsetEngine.BuildNesting", t0,
            $"rings={rings.Count} candidateChecks={candidateChecks}");
    }

    /// <summary>Sampled closest approach between two rings; exact distance is not required to rank parents.</summary>
    private static double ClosestApproachSq(IReadOnlyList<PointD> a, IReadOnlyList<PointD> b)
    {
        int stepA = Math.Max(1, a.Count / 8);
        int stepB = Math.Max(1, b.Count / 8);
        double best = double.MaxValue;

        for (int i = 0; i < a.Count; i += stepA)
        {
            for (int j = 0; j < b.Count; j += stepB)
            {
                double dx = a[i].x - b[j].x;
                double dy = a[i].y - b[j].y;
                double d = dx * dx + dy * dy;
                if (d < best)
                    best = d;
            }
        }

        return best;
    }

    // --- Traversal ------------------------------------------------------------

    /// <summary>
    /// Orders rings depth-first so every disconnected pocket is finished before the next one
    /// starts, keeping the cutter local instead of hopping between islands.
    ///
    /// <para>
    /// With <paramref name="insideOut"/> the post-order emits children before their parent, so
    /// enclosed material is removed while it is still supported. Cutting the enclosing ring first
    /// would leave the island floating and free to move under the cutter.
    /// </para>
    /// </summary>
    public static RingTraversal BuildTraversal(IReadOnlyList<OffsetRing> rings, bool insideOut)
    {
        long t0 = PerfLog.Start();

        var order = new List<int>(rings.Count);
        var islands = new List<(int StartIndex, int Count)>();

        var roots = new List<int>();
        for (int i = 0; i < rings.Count; i++)
        {
            if (rings[i].Parent == -1)
                roots.Add(i);
        }

        if (roots.Count > 1)
        {
            // Inside-out starts from the tightest island; outside-in from the largest.
            roots.Sort((a, b) => insideOut
                ? rings[a].Area.CompareTo(rings[b].Area)
                : rings[b].Area.CompareTo(rings[a].Area));
        }

        double lastX = double.NaN, lastY = double.NaN;

        void SortChildrenByProximity(List<int> children)
        {
            if (children.Count <= 1)
                return;

            if (double.IsNaN(lastX))
            {
                children.Sort((a, b) => rings[b].Area.CompareTo(rings[a].Area));
                return;
            }

            double curX = lastX, curY = lastY;
            for (int i = 0; i < children.Count - 1; i++)
            {
                int best = i;
                double bestDistance = double.MaxValue;
                for (int j = i; j < children.Count; j++)
                {
                    var c = rings[children[j]].Centroid;
                    double d = (c.x - curX) * (c.x - curX) + (c.y - curY) * (c.y - curY);
                    if (d < bestDistance)
                    {
                        bestDistance = d;
                        best = j;
                    }
                }
                if (best != i)
                    (children[i], children[best]) = (children[best], children[i]);

                curX = rings[children[i]].Centroid.x;
                curY = rings[children[i]].Centroid.y;
            }
        }

        void Visit(int index)
        {
            if (insideOut)
            {
                SortChildrenByProximity(rings[index].Children);
                foreach (int child in rings[index].Children)
                    Visit(child);
                order.Add(index);
            }
            else
            {
                order.Add(index);
                SortChildrenByProximity(rings[index].Children);
                foreach (int child in rings[index].Children)
                    Visit(child);
            }

            lastX = rings[index].Centroid.x;
            lastY = rings[index].Centroid.y;
        }

        var pending = new List<int>(roots);
        while (pending.Count > 0)
        {
            int pick = 0;
            if (!double.IsNaN(lastX) && pending.Count > 1)
            {
                double bestDistance = double.MaxValue;
                for (int i = 0; i < pending.Count; i++)
                {
                    var c = rings[pending[i]].Centroid;
                    double d = (c.x - lastX) * (c.x - lastX) + (c.y - lastY) * (c.y - lastY);
                    if (d < bestDistance)
                    {
                        bestDistance = d;
                        pick = i;
                    }
                }
            }

            int root = pending[pick];
            pending.RemoveAt(pick);

            int start = order.Count;
            Visit(root);
            islands.Add((start, order.Count - start));
        }

        PerfLog.Stop("RingOffsetEngine.BuildTraversal", t0,
            $"rings={rings.Count} islands={islands.Count} insideOut={insideOut}");

        return new RingTraversal(order, islands);
    }

    // --- Spiral chaining ------------------------------------------------------

    /// <summary>
    /// Groups consecutive rings in the traversal into chains that can be cut without lifting.
    /// A chain continues while each ring leads to exactly one nested neighbour; branching or a
    /// dead end closes it. Fewer chains means fewer plunges, retracts and entry marks.
    /// </summary>
    public static List<RingChain> BuildChains(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<int> order)
    {
        var chains = new List<RingChain>();
        if (order.Count == 0)
            return chains;

        var current = new List<int> { order[0] };

        for (int i = 1; i < order.Count; i++)
        {
            int previous = order[i - 1];
            int next = order[i];

            // Continuous only when the two rings are directly nested and the outer one has no
            // other branch competing for the cutter.
            bool nested =
                (rings[next].Parent == previous && rings[previous].Children.Count == 1) ||
                (rings[previous].Parent == next && rings[next].Children.Count == 1);

            if (nested)
            {
                current.Add(next);
            }
            else
            {
                chains.Add(new RingChain(current));
                current = new List<int> { next };
            }
        }

        chains.Add(new RingChain(current));
        return chains;
    }

    /// <summary>
    /// Flattens a chain into one continuous polyline, rotating each ring so it begins near where
    /// the previous ring ended. Without that the link between rings would be a long chord across
    /// the pocket instead of a short step-over move.
    /// </summary>
    public static List<PointD> MaterializeChain(
        IReadOnlyList<OffsetRing> rings,
        RingChain chain,
        MillingDirection direction)
    {
        var points = new List<PointD>();

        for (int i = 0; i < chain.RingIndices.Count; i++)
        {
            var ring = Orient(rings[chain.RingIndices[i]].Points, direction);

            if (points.Count > 0)
                ring = PathUtils.RebaseNear(ring, points[^1]);

            points.AddRange(ring);

            // Close each ring back onto its own start before stepping to the next one.
            if (ring.Count > 0)
                points.Add(ring[0]);
        }

        return points;
    }

    /// <summary>Applies milling direction to a ring, returning a copy.</summary>
    public static List<PointD> Orient(IReadOnlyList<PointD> ring, MillingDirection direction)
    {
        if (direction == MillingDirection.Default)
            return new List<PointD>(ring);

        bool wantCcw = direction == MillingDirection.Climb;
        bool isCcw = PolygonOps.SignedArea(ring) > 0;
        if (isCcw == wantCcw)
            return new List<PointD>(ring);

        var reversed = new List<PointD>(ring);
        reversed.Reverse();
        return reversed;
    }
}
