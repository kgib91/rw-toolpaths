namespace RW.Toolpaths.Geometry;

/// <summary>
/// An immutable XY snapshot with incremental, exact distance ordering. Original
/// ordinals break ties, matching stable OrderBy even when points share XY or Z.
/// Every enumeration owns its queue; partial enumeration does not sort all points.
/// </summary>
internal sealed class PreparedPointIndex
{
    private const int LeafSize = 12;
    private readonly record struct Entry(Point3D Point, int Ordinal);
    private readonly record struct Node(double MinX, double MinY, double MaxX, double MaxY,
        int First, int Count, int MinimumOrdinal, int Left, int Right, double MinimumZ);
    private readonly record struct Work(int Node, int Entry);
    private readonly Entry[] _entries;
    private readonly Node[] _nodes;
    private readonly bool _finite;

    internal int Count => _entries.Length;
    internal bool SupportsBounds => _finite;

    /// <summary>Optional caller-owned counters; no counters are collected by default.</summary>
    internal sealed class QueryDiagnostics
    {
        internal long Queries, NodeBounds, PointDistances, ReturnedPoints, FallbackQueries;
        internal long BuildPartitions, BuildPartitionVisits, RobustPartitions;
        internal long RangeQueries, RangeNodeBounds, RangePointTests, RangeReturnedOrdinals, RangeFallbackQueries;
        internal int PeakQueue;

        internal string Describe() => $"queries={Queries} nodeBounds={NodeBounds} pointDistances={PointDistances} " +
            $"returnedPoints={ReturnedPoints} peakQueue={PeakQueue} fallbackQueries={FallbackQueries} " +
            $"buildPartitions={BuildPartitions} buildPartitionVisits={BuildPartitionVisits} robustPartitions={RobustPartitions} " +
            $"rangeQueries={RangeQueries} rangeNodeBounds={RangeNodeBounds} rangePointTests={RangePointTests} " +
            $"rangeReturnedOrdinals={RangeReturnedOrdinals} rangeFallbackQueries={RangeFallbackQueries} " +
            "ordering=exact-squared-distance-then-original-ordinal immutableSnapshot=True";
    }

    internal PreparedPointIndex(IReadOnlyList<Point3D> points, CancellationToken cancellationToken = default,
        QueryDiagnostics? diagnostics = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        _entries = new Entry[points.Count];
        _finite = true;
        for (int i = 0; i < points.Count; i++)
        {
            if ((i & 1023) == 0) cancellationToken.ThrowIfCancellationRequested();
            _entries[i] = new(points[i], i);
            _finite &= double.IsFinite(points[i].X) && double.IsFinite(points[i].Y);
        }
        if (!_finite || _entries.Length == 0) { _nodes = Array.Empty<Node>(); return; }
        var nodes = new List<Node>();
        Build(0, _entries.Length);
        _nodes = nodes.ToArray();

        int Build(int first, int count)
        {
            cancellationToken.ThrowIfCancellationRequested();
            double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
            double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
            int ordinal = int.MaxValue;
            double minimumZ = double.PositiveInfinity;
            for (int i = first; i < first + count; i++)
            {
                Point3D point = _entries[i].Point;
                minX = Math.Min(minX, point.X); minY = Math.Min(minY, point.Y);
                maxX = Math.Max(maxX, point.X); maxY = Math.Max(maxY, point.Y);
                ordinal = Math.Min(ordinal, _entries[i].Ordinal);
                minimumZ = Math.Min(minimumZ, point.Z);
            }
            int index = nodes.Count;
            nodes.Add(default);
            int left = -1, right = -1;
            if (count > LeafSize)
            {
                bool x = maxX - minX >= maxY - minY;
                int half = count / 2;
                Partition(first, first + count - 1, first + half, x);
                left = Build(first, half); right = Build(first + half, count - half);
            }
            nodes[index] = new(minX, minY, maxX, maxY, first, count, ordinal, left, right, minimumZ);
            return index;
        }

        void Partition(int lo, int hi, int median, bool x)
        {
            // Median selection avoids sorting every recursive subtree. Axis and
            // ordinal form a total order, including coincident input points.
            long remainingWork = 3L * (hi - lo + 1);
            while (lo < hi)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (diagnostics is not null)
                {
                    diagnostics.BuildPartitions++;
                    diagnostics.BuildPartitionVisits += hi - lo + 1;
                }
                remainingWork -= hi - lo + 1;
                int pivotIndex;
                if (remainingWork < 0)
                {
                    if (diagnostics is not null) diagnostics.RobustPartitions++;
                    // Ordered rings can defeat median-of-three. After a linear
                    // work budget, exact median-of-five-group medians guarantees
                    // bounded partitions without a full-subtree sorting fallback.
                    int groups = 0;
                    for (int group = lo; group <= hi; group += 5)
                    {
                        int end = Math.Min(group + 4, hi);
                        for (int i = group + 1; i <= end; i++)
                            for (int j = i; j > group && Compare(_entries[j], _entries[j - 1], x) < 0; j--)
                                Swap(j, j - 1);
                        Swap(lo + groups++, group + (end - group) / 2);
                    }
                    pivotIndex = lo + groups / 2;
                    Partition(lo, lo + groups - 1, pivotIndex, x);
                }
                else
                {
                    pivotIndex = lo + (hi - lo) / 2;
                    if (Compare(_entries[pivotIndex], _entries[lo], x) < 0) Swap(pivotIndex, lo);
                    if (Compare(_entries[hi], _entries[lo], x) < 0) Swap(hi, lo);
                    if (Compare(_entries[hi], _entries[pivotIndex], x) < 0) Swap(hi, pivotIndex);
                }
                Entry pivot = _entries[pivotIndex];
                Swap(pivotIndex, hi);
                int split = lo;
                for (int i = lo; i < hi; i++)
                    if (Compare(_entries[i], pivot, x) < 0) Swap(i, split++);
                Swap(split, hi);
                if (split == median) return;
                if (split < median) lo = split + 1; else hi = split - 1;
            }
        }

        void Swap(int a, int b) => (_entries[a], _entries[b]) = (_entries[b], _entries[a]);
    }

    /// <summary>
    /// Returns at most maximumResults; omit that limit for a lazy search whose
    /// validity predicate or candidate budget determines when to stop.
    /// </summary>
    internal IEnumerable<Point3D> Nearest(double x, double y, int maximumResults = int.MaxValue,
        CancellationToken cancellationToken = default, QueryDiagnostics? diagnostics = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        if (diagnostics is not null) diagnostics.Queries++;
        if (maximumResults <= 0 || Count == 0) yield break;
        if (!_finite || !double.IsFinite(x) || !double.IsFinite(y))
        {
            // Preserve the stable double comparer for malformed/nonfinite input
            // as well. Such coordinates cannot use conservative box bounds.
            if (diagnostics is not null) diagnostics.FallbackQueries++;
            int returned = 0;
            foreach (Entry entry in _entries.OrderBy(entry => DistanceSquared(entry.Point, x, y))
                .ThenBy(entry => entry.Ordinal))
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (diagnostics is not null) diagnostics.ReturnedPoints++;
                yield return entry.Point;
                if (++returned == maximumResults) yield break;
            }
            yield break;
        }
        var queue = new PriorityQueue<Work, (double Distance, int Ordinal, int Kind)>();
        EnqueueNode(0);
        int visits = 0, emitted = 0;
        while (queue.TryDequeue(out Work work, out _))
        {
            if ((++visits & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            if (work.Node >= 0)
            {
                Node node = _nodes[work.Node];
                if (node.Left >= 0) { EnqueueNode(node.Left); EnqueueNode(node.Right); }
                else for (int i = node.First; i < node.First + node.Count; i++)
                {
                    Entry entry = _entries[i];
                    if (diagnostics is not null) diagnostics.PointDistances++;
                    queue.Enqueue(new(-1, i), (DistanceSquared(entry.Point, x, y), entry.Ordinal, 1));
                }
                if (diagnostics is not null) diagnostics.PeakQueue = Math.Max(diagnostics.PeakQueue, queue.Count);
            }
            else
            {
                if (diagnostics is not null) diagnostics.ReturnedPoints++;
                yield return _entries[work.Entry].Point;
                if (++emitted == maximumResults) yield break;
            }
        }

        void EnqueueNode(int index)
        {
            Node node = _nodes[index];
            double dx = Math.Max(0, Math.Max(node.MinX - x, x - node.MaxX));
            double dy = Math.Max(0, Math.Max(node.MinY - y, y - node.MaxY));
            // The coordinate interval bounds are monotone under IEEE subtraction,
            // squaring and addition. Round the result downward too so equality
            // cannot let a point pass an unexpanded subtree with an earlier tie.
            double lower = Math.Max(0, Math.BitDecrement(dx * dx + dy * dy));
            if (diagnostics is not null) diagnostics.NodeBounds++;
            queue.Enqueue(new(index, -1), (lower, node.MinimumOrdinal, 0));
        }
    }

    /// <summary>Finds a real pose satisfying a bounded XY disk and maximum Z.
    /// This positive query avoids sorting nearest points when any physical witness suffices.</summary>
    internal bool AnyWithin(double x, double y, double radius, double maximumZ,
        CancellationToken token = default, Func<Point3D, bool>? predicate = null,
        Func<double,double>? radiusAtDepth = null)
    {
        token.ThrowIfCancellationRequested();
        if (radius < 0 || double.IsNaN(radius)) return false;
        double radius2 = radius * radius;
        if (!_finite)
        {
            for (int i = 0; i < _entries.Length; i++)
            {
                if ((i & 63) == 0) token.ThrowIfCancellationRequested();
                var p = _entries[i].Point;
                if (p.Z <= maximumZ && DistanceSquared(p, x, y) <= radius2 && (predicate is null || predicate(p))) return true;
            }
            return false;
        }
        if (_nodes.Length == 0) return false;
        // Median partitioning halves every subtree: even Int32.MaxValue entries
        // need fewer than 32 pending siblings. Keep this positive query on stack.
        Span<int> pending = stackalloc int[64];
        int count = 1, visits = 0; pending[0] = 0;
        while (count > 0)
        {
            if ((++visits & 63) == 0) token.ThrowIfCancellationRequested();
            var node = _nodes[pending[--count]];
            if (node.MinimumZ > maximumZ) continue;
            // A monotone physical radius can tighten a positive-witness query
            // with this subtree's actual deepest pose. The final point predicate
            // remains authoritative; a bound never establishes a witness.
            double nodeRadius=radiusAtDepth is null?radius:Math.Min(radius,radiusAtDepth(maximumZ-node.MinimumZ));
            double dx = Math.Max(0, Math.Max(node.MinX - x, x - node.MaxX));
            double dy = Math.Max(0, Math.Max(node.MinY - y, y - node.MaxY));
            if (Math.BitDecrement(dx * dx + dy * dy) > Math.BitIncrement(nodeRadius*nodeRadius)) continue;
            if (node.Left < 0)
            {
                for (int i = node.First; i < node.First + node.Count; i++)
                {
                    var p = _entries[i].Point;
                    if (p.Z <= maximumZ && DistanceSquared(p, x, y) <= radius2 && (predicate is null || predicate(p))) return true;
                }
                continue;
            }
            var left = _nodes[node.Left]; var right = _nodes[node.Right];
            double lx = Math.Max(0, Math.Max(left.MinX - x, x - left.MaxX));
            double ly = Math.Max(0, Math.Max(left.MinY - y, y - left.MaxY));
            double rx = Math.Max(0, Math.Max(right.MinX - x, x - right.MaxX));
            double ry = Math.Max(0, Math.Max(right.MinY - y, y - right.MaxY));
            bool leftFirst = lx * lx + ly * ly <= rx * rx + ry * ry;
            pending[count++] = leftFirst ? node.Right : node.Left;
            pending[count++] = leftFirst ? node.Left : node.Right;
        }
        return false;
    }

    /// <summary>
    /// Returns original ordinals in the inclusive XY rectangle. Traversal order
    /// is unspecified; callers restore their established candidate order.
    /// </summary>
    internal List<int> IntersectingOrdinals(double minX, double minY, double maxX, double maxY,
        CancellationToken cancellationToken = default, QueryDiagnostics? diagnostics = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        if (diagnostics is not null) diagnostics.RangeQueries++;
        var result = new List<int>();
        if (Count == 0) return result;
        if (!_finite || !double.IsFinite(minX) || !double.IsFinite(minY)
            || !double.IsFinite(maxX) || !double.IsFinite(maxY))
        {
            if (diagnostics is not null) diagnostics.RangeFallbackQueries++;
            for (int i = 0; i < _entries.Length; i++)
            {
                if ((i & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                Consider(_entries[i]);
            }
        }
        else Visit(0);
        return result;

        void Visit(int index)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (diagnostics is not null) diagnostics.RangeNodeBounds++;
            Node node = _nodes[index];
            if (node.MaxX < minX || node.MinX > maxX || node.MaxY < minY || node.MinY > maxY) return;
            if (node.Left >= 0) { Visit(node.Left); Visit(node.Right); }
            else for (int i = node.First; i < node.First + node.Count; i++) Consider(_entries[i]);
        }

        void Consider(Entry entry)
        {
            if (diagnostics is not null) diagnostics.RangePointTests++;
            Point3D point = entry.Point;
            if (point.X >= minX && point.X <= maxX && point.Y >= minY && point.Y <= maxY)
            {
                result.Add(entry.Ordinal);
                if (diagnostics is not null) diagnostics.RangeReturnedOrdinals++;
            }
        }
    }

    private static int Compare(Entry a, Entry b, bool x)
    {
        int coordinate = (x ? a.Point.X : a.Point.Y).CompareTo(x ? b.Point.X : b.Point.Y);
        return coordinate != 0 ? coordinate : a.Ordinal.CompareTo(b.Ordinal);
    }

    private static double DistanceSquared(Point3D point, double x, double y)
        => (point.X - x) * (point.X - x) + (point.Y - y) * (point.Y - y);
}
