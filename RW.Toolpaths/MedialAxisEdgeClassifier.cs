using Clipper2Lib;
using System.Diagnostics;

namespace RW.Toolpaths;

internal static partial class MedialAxisEdgeClassifier
{
    internal static class Colors
    {
        public const int Unclassified = 11;
        public const int Secondary = 1;
        public const int OuterPrimary = 2;
        public const int InnerPrimary = 3;
        public const int HolePrimary = 4;
    }

    private const double Epsilon = 1e-5;

    internal sealed record class EdgePoint(double X, double Y);

    internal sealed record class SegmentSite(EdgePoint Low, EdgePoint High);

    internal sealed record class CellSite(bool ContainsPoint, EdgePoint? Point, SegmentSite? Segment);

    internal sealed record class EdgeData(
        int Index,
        int TwinIndex,
        bool IsPrimary,
        bool IsSecondary,
        bool IsInfinite,
        int PrevIndex,
        int NextIndex,
        EdgePoint? Vertex0,
        EdgePoint? Vertex1,
        CellSite Cell,
        CellSite TwinCell)
    {
        public int Color { get; set; }
        public bool IsLinear { get; set; }
        public bool IsCurved { get; set; }
    }

    internal static void ClassifyEdges(
        IReadOnlyList<EdgeData> edges,
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        CancellationToken cancellationToken = default, Action<string>? diagnostic = null,
        Action<double>? progress = null)
        => ClassifyEdgesCore(edges, boundary, holes, cancellationToken, diagnostic, progress,
            reference: string.Equals(Environment.GetEnvironmentVariable("RW_TOOLPATHS_CLASSIFICATION_REFERENCE"),
                "1", StringComparison.Ordinal));

    // Diagnostic original traversal, with the same boundary and finite-site
    // predicates. It never changes native sites, edge order or output ownership.
    internal static void ClassifyEdgesReference(
        IReadOnlyList<EdgeData> edges, IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        CancellationToken cancellationToken = default, Action<string>? diagnostic = null,
        Action<double>? progress = null)
        => ClassifyEdgesCore(edges, boundary, holes, cancellationToken, diagnostic, progress, reference: true);

    private static void ClassifyEdgesCore(
        IReadOnlyList<EdgeData> edges, IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes, CancellationToken cancellationToken,
        Action<string>? diagnostic, Action<double>? progress, bool reference)
    {
        cancellationToken.ThrowIfCancellationRequested();
        long preparationAt = Stopwatch.GetTimestamp();
        var preparedBoundary = new PreparedRingContainment(boundary, cancellationToken);
        var preparedHoles = holes.Select(hole => new PreparedRingContainment(hole, cancellationToken)).ToArray();
        var containment = new PreparedRegionContainment(preparedBoundary, preparedHoles, cancellationToken,
            useHoleIndex: !reference, memoLimit: reference ? 0 : PreparedRegionContainment.DefaultMemoLimit);
        double preparationMs = Stopwatch.GetElapsedTime(preparationAt).TotalMilliseconds;
        var candidates = new List<EdgeData>();

        foreach (var edge in edges)
        {
            if (edge.IsPrimary)
            {
                if (edge.IsInfinite)
                {
                    edge.Color = Colors.OuterPrimary;
                }
                else
                {
                    edge.Color = Colors.Unclassified;
                    candidates.Add(edge);
                }
            }
            else
            {
                edge.Color = Colors.Secondary;
            }
        }

        int insideCount = 0;
        int outsideBoundaryCount = 0;
        int insideHoleCount = 0;

        long progressAt = Stopwatch.GetTimestamp();
        int completed = 0;
        foreach (var edge in candidates)
        {
            if ((completed++ & 1023) == 0)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if ((diagnostic is not null || progress is not null) && Stopwatch.GetElapsedTime(progressAt).TotalMilliseconds >= 250)
                {
                    diagnostic?.Invoke($"stage=voronoi-classification event=progress completed={completed} total={candidates.Count}");
                    progress?.Invoke((double)completed / Math.Max(1, candidates.Count));
                    progressAt = Stopwatch.GetTimestamp();
                }
            }
            if (edge.Color == Colors.InnerPrimary)
            {
                continue;
            }

            var nonBorderPoint = GetNonBorderPoint(edge);
            if (containment.Contains(nonBorderPoint, out bool isInsideHole))
            {
                insideCount++;
                edge.Color = Colors.InnerPrimary;

                if (edge.TwinIndex >= 0 && edge.TwinIndex < edges.Count)
                {
                    edges[edge.TwinIndex].Color = Colors.InnerPrimary;
                }
            }
            else if (isInsideHole)
            {
                insideHoleCount++;
            }
            else
            {
                outsideBoundaryCount++;
            }
        }

        cancellationToken.ThrowIfCancellationRequested();
        progress?.Invoke(1);
        diagnostic?.Invoke($"stage=voronoi-classification event=complete completed={candidates.Count} total={candidates.Count} " +
            $"inside={insideCount} outside={outsideBoundaryCount} holes={insideHoleCount} " +
            $"sourceEdges={boundary.Count + holes.Sum(hole => hole.Count)} " +
            $"edgeTests={preparedBoundary.EdgeTests + preparedHoles.Sum(hole => hole.EdgeTests)}");
        string evidence = $"mode={(reference ? "original-all-holes" : "exact-hole-bounds-and-immutable-point-memo")} "
            + $"holes={holes.Count} queries={containment.Queries} memoHits={containment.MemoHits} "
            + $"memoEntries={containment.MemoCount} memoLimit={containment.MemoLimit} memoClears={containment.MemoClears} "
            + $"holeNodeVisits={containment.HoleNodeVisits} holeBoundsTests={containment.HoleBoundsTests} "
            + $"holePredicateTests={containment.HolePredicateTests} preparationMs={preparationMs:F2} "
            + $"holeIndexBuildMs={containment.HoleIndexBuildMilliseconds:F2} "
            + $"membershipMs={containment.QueryMilliseconds:F2} originalRayPredicate=True "
            + "inclusiveBounds=True originalSites=True sourceOrder=unchanged";
        diagnostic?.Invoke("stage=voronoi-classification event=containment " + evidence);
        Console.WriteLine("[BoostVoronoi.ClassificationContainment] " + evidence);
        if (IsMedialDebugEnabled())
        {
            Console.Error.WriteLine(
                $"[medial-axis][classify] candidates={candidates.Count} inside={insideCount} " +
                $"outsideBoundary={outsideBoundaryCount} insideHole={insideHoleCount} holes={holes.Count}");
        }
    }

    internal static List<int> SelectInnerPrimaryTraversal(
        IReadOnlyList<EdgeData> edges,
        double filteringAngle, CancellationToken cancellationToken = default,
        Action<string>? diagnostic = null, Action<double>? progress = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var innerPrimary = edges
            .Where(edge => edge.Color == Colors.InnerPrimary)
            .ToList();

        // Edge indices are their positions in the provider's edge array. The old
        // walk scanned that entire array for the first remaining edge whose
        // predecessor had disappeared. Maintain the same ordered eligible set and
        // update only dependents of removed edges (including removed twins).
        var remaining = new bool[edges.Count];
        var firstFollower = new int[edges.Count];
        var nextFollower = new int[edges.Count];
        Array.Fill(firstFollower, -1);
        Array.Fill(nextFollower, -1);
        foreach (var edge in innerPrimary) remaining[edge.Index] = true;
        var eligible = new SortedSet<int>();
        int remainingCount = innerPrimary.Count, smallestRemaining = 0;
        foreach (var edge in innerPrimary)
        {
            if ((edge.Index & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
            var previous = GetPreviousThroughSecondary(edges, edge);
            if (previous is null || !remaining[previous.Index]) eligible.Add(edge.Index);
            if (previous is not null)
            {
                nextFollower[edge.Index] = firstFollower[previous.Index];
                firstFollower[previous.Index] = edge.Index;
            }
        }

        void Remove(int index)
        {
            if (index < 0 || index >= remaining.Length || !remaining[index]) return;
            remaining[index] = false;
            remainingCount--;
            eligible.Remove(index);
            for (int dependent = firstFollower[index]; dependent >= 0; dependent = nextFollower[dependent])
                if (remaining[dependent]) eligible.Add(dependent);
        }

        var walked = new List<int>();
        long progressAt = Stopwatch.GetTimestamp();
        while (remainingCount > 0)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if ((diagnostic is not null || progress is not null) && Stopwatch.GetElapsedTime(progressAt).TotalMilliseconds >= 250)
            {
                diagnostic?.Invoke($"stage=voronoi-traversal event=progress completed={innerPrimary.Count - remainingCount} total={innerPrimary.Count}");
                progress?.Invoke((double)(innerPrimary.Count - remainingCount) / Math.Max(1, innerPrimary.Count));
                progressAt = Stopwatch.GetTimestamp();
            }
            while (smallestRemaining < remaining.Length && !remaining[smallestRemaining]) smallestRemaining++;
            var start = edges[eligible.Count > 0 ? eligible.Min : smallestRemaining];
            var path = new List<int>();
            EdgeData? current = start;
            while (current is not null && remaining[current.Index])
            {
                if ((walked.Count & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
                walked.Add(current.Index);
                path.Add(current.Index);
                Remove(current.Index);
                current = GetNextThroughSecondary(edges, current);
            }

            foreach (int edgeIndex in path) Remove(edges[edgeIndex].TwinIndex);
        }

        progress?.Invoke(1);
        return walked
            .Where(index =>
            {
                double? angle = GetCellsAngle(edges[index]);
                return angle is null || angle <= filteringAngle;
            })
            .ToList();
    }

    // Preserve the ray-crossing predicate exactly, including its asymmetric boundary
    // convention. Y buckets eliminate edges that cannot cross this ray; strict bounds
    // eliminate distant holes without changing equality behavior on a contour.
    internal sealed class PreparedRingContainment
    {
        private readonly IReadOnlyList<PointD> _ring;
        private readonly int[][] _buckets;
        private readonly double _minX, _maxX, _minY, _maxY, _bucketHeight;
        internal long EdgeTests { get; private set; }
        internal bool IsEmpty => _ring.Count == 0;
        internal (double MinX, double MinY, double MaxX, double MaxY) Bounds
            => (_minX, _minY, _maxX, _maxY);

        internal PreparedRingContainment(IReadOnlyList<PointD> ring, CancellationToken cancellationToken = default)
        {
            _ring = ring;
            if (ring.Count == 0)
            {
                _buckets = Array.Empty<int[]>();
                return;
            }
            _minX = ring.Min(p => p.x); _maxX = ring.Max(p => p.x);
            _minY = ring.Min(p => p.y); _maxY = ring.Max(p => p.y);
            int count = Math.Clamp((int)Math.Sqrt(ring.Count), 1, 512);
            _bucketHeight = _maxY > _minY ? (_maxY - _minY) / count : 1;
            var buckets = Enumerable.Range(0, count).Select(_ => new List<int>()).ToArray();
            for (int index = 0, previous = ring.Count - 1; index < ring.Count; previous = index++)
            {
                if ((index & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
                double y0 = ring[index].y, y1 = ring[previous].y;
                if (y0 == y1) continue;
                int first = Bucket(Math.Min(y0, y1), count), last = Bucket(Math.Max(y0, y1), count);
                for (int bucket = first; bucket <= last; bucket++) buckets[bucket].Add(index);
            }
            _buckets = buckets.Select(bucket => bucket.ToArray()).ToArray();
        }

        private int Bucket(double y, int count) => Math.Clamp((int)((y - _minY) / _bucketHeight), 0, count - 1);

        internal bool Contains(EdgePoint point)
        {
            if (_ring.Count == 0 || point.X < _minX || point.X > _maxX || point.Y < _minY || point.Y > _maxY)
                return false;
            bool inside = false;
            var candidates = _buckets[Bucket(point.Y, _buckets.Length)];
            EdgeTests += candidates.Length;
            foreach (int index in candidates)
            {
                var current = _ring[index];
                var prior = _ring[index == 0 ? _ring.Count - 1 : index - 1];
                bool crosses = (current.y > point.Y) != (prior.y > point.Y);
                if (!crosses) continue;
                double xAtY = (prior.x - current.x) * (point.Y - current.y) /
                              (prior.y - current.y) + current.x;
                if (xAtY > point.X) inside = !inside;
            }
            return inside;
        }
    }

    private static EdgePoint GetNonBorderPoint(EdgeData edge)
    {
        var vertex0 = edge.Vertex0 ?? throw new InvalidOperationException("Expected finite edge vertex0");
        var vertex1 = edge.Vertex1 ?? throw new InvalidOperationException("Expected finite edge vertex1");

        if (edge.Cell.ContainsPoint)
        {
            var point = edge.Cell.Point ?? throw new InvalidOperationException("Expected point site");
            if (!EqualPoints(vertex0, point))
            {
                return vertex0;
            }
        }
        else
        {
            var segment = edge.Cell.Segment ?? throw new InvalidOperationException("Expected segment site");
            if (!EqualPoints(vertex0, segment.Low) && !EqualPoints(vertex0, segment.High))
            {
                return vertex0;
            }
        }

        return vertex1;
    }

    private static EdgeData? GetPreviousThroughSecondary(IReadOnlyList<EdgeData> edges, EdgeData edge)
    {
        var current = GetEdge(edges, edge.PrevIndex);
        while (current is not null && current.IsSecondary && current.Index != edge.Index)
        {
            var twin = GetEdge(edges, current.TwinIndex);
            current = twin is null ? null : GetEdge(edges, twin.PrevIndex);
        }

        return current;
    }

    private static EdgeData? GetNextThroughSecondary(IReadOnlyList<EdgeData> edges, EdgeData edge)
    {
        var current = GetEdge(edges, edge.NextIndex);
        while (current is not null && current.IsSecondary && current.Index != edge.Index)
        {
            var twin = GetEdge(edges, current.TwinIndex);
            current = twin is null ? null : GetEdge(edges, twin.NextIndex);
        }

        return current;
    }

    private static double? GetCellsAngle(EdgeData edge)
    {
        if (edge.Cell.ContainsPoint || edge.TwinCell.ContainsPoint)
        {
            return null;
        }

        var left = edge.Cell.Segment ?? throw new InvalidOperationException("Expected segment site on cell");
        var right = edge.TwinCell.Segment ?? throw new InvalidOperationException("Expected segment site on twin cell");

        if (EqualPoints(left.Low, right.Low))
        {
            return Angle(left.High, left.Low, right.High);
        }

        if (EqualPoints(left.High, right.Low))
        {
            return Angle(left.Low, left.High, right.High);
        }

        if (EqualPoints(left.Low, right.High))
        {
            return Angle(left.High, left.Low, right.Low);
        }

        if (EqualPoints(left.High, right.High))
        {
            return Angle(left.Low, left.High, right.Low);
        }

        return null;
    }

    private static double Angle(EdgePoint a, EdgePoint pivot, EdgePoint b)
    {
        double ax = a.X - pivot.X;
        double ay = a.Y - pivot.Y;
        double bx = b.X - pivot.X;
        double by = b.Y - pivot.Y;

        double dot = ax * bx + ay * by;
        double denom = Math.Sqrt((ax * ax + ay * ay) * (bx * bx + by * by));
        if (denom <= 0)
        {
            return 0;
        }

        return Math.Acos(Math.Clamp(dot / denom, -1.0, 1.0));
    }

    internal static bool EqualPoints(EdgePoint a, EdgePoint b) =>
        Math.Abs(a.X - b.X) < Epsilon && Math.Abs(a.Y - b.Y) < Epsilon;

    private static bool IsMedialDebugEnabled() =>
        string.Equals(
            Environment.GetEnvironmentVariable("RW_TOOLPATHS_MEDIAL_DEBUG"),
            "1",
            StringComparison.Ordinal);

    private static EdgeData? GetEdge(IReadOnlyList<EdgeData> edges, int index) =>
        index >= 0 && index < edges.Count ? edges[index] : null;
}
