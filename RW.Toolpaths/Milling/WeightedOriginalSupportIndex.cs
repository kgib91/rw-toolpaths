using RW.Toolpaths;
namespace RW.Toolpaths.Milling.Artifacts;

/// <summary>
/// An immutable original-command endpoint/disc snapshot with incremental physical gap ordering. Original
/// ordinals break ties, matching stable OrderBy even when points share XY or Z.
/// AABB minus maximum ORIGINAL physical radius bounds each node. Candidate directions only; no coverage credit.
/// </summary>
internal sealed class WeightedOriginalSupportIndex
{
    private const int LeafSize = 12;
    private readonly record struct Entry(Point3D Point, int Ordinal, double Radius);
    private readonly record struct Node(double MinX, double MinY, double MaxX, double MaxY,
        int First, int Count, int MinimumOrdinal, int Left, int Right, double MaximumRadius);
    private readonly record struct Work(int Node, int Entry);
    internal readonly record struct Hit(int OriginalOrdinal, Point3D Point, double OriginalPhysicalRadius, double PhysicalGap);
    private readonly Entry[] _entries;
    private readonly Node[] _nodes;
    private readonly bool _finite;

    internal int Count => _entries.Length;
    internal bool SupportsBounds => _finite;

    /// <summary>Optional caller-owned counters; no counters are collected by default.</summary>
    internal sealed class QueryDiagnostics
    {
        internal long Queries, NodeBounds, PointDistances, ReturnedPoints;
        internal long BuildPartitions, BuildPartitionVisits, RobustPartitions;
        internal int PeakQueue; internal long BoundedQueries;
        internal string LastQueryStatus = "unqueried-NOM-only";

        internal string Describe() => $"queries={Queries} nodeBounds={NodeBounds} pointDistances={PointDistances} " +
            $"returnedPoints={ReturnedPoints} peakQueue={PeakQueue} allSourceFallbackQueries=0 " +
            $"buildPartitions={BuildPartitions} buildPartitionVisits={BuildPartitionVisits} robustPartitions={RobustPartitions} " +
            $"ordering=original-physical-disc-gap-then-original-ordinal immutableSnapshot=True queryVisitCap=65536 boundedQueries={BoundedQueries} lastQueryStatus={LastQueryStatus} negativeCredit=False";
    }

    internal WeightedOriginalSupportIndex(IReadOnlyList<Point3D> points, Func<double, double> originalPhysicalRadius, CancellationToken cancellationToken = default,
        QueryDiagnostics? diagnostics = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        _entries = new Entry[points.Count];
        _finite = true;
        for (int i = 0; i < points.Count; i++)
        {
            if ((i & 1023) == 0) cancellationToken.ThrowIfCancellationRequested();
            double radius = originalPhysicalRadius(-points[i].Z);
            if (!double.IsFinite(radius) || radius < 0 || !double.IsFinite(points[i].Z)) throw new ArgumentOutOfRangeException(nameof(points));
            _entries[i] = new(points[i], i, radius);
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
            int ordinal = int.MaxValue; double maximumRadius = 0;
            for (int i = first; i < first + count; i++)
            {
                Point3D point = _entries[i].Point;
                minX = Math.Min(minX, point.X); minY = Math.Min(minY, point.Y);
                maxX = Math.Max(maxX, point.X); maxY = Math.Max(maxY, point.Y);
                ordinal = Math.Min(ordinal, _entries[i].Ordinal);
                maximumRadius = Math.Max(maximumRadius, _entries[i].Radius);
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
            nodes[index] = new(minX, minY, maxX, maxY, first, count, ordinal, left, right, maximumRadius);
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
    internal IEnumerable<Hit> Nearest(double x, double y, int maximumResults = int.MaxValue,
        CancellationToken cancellationToken = default, QueryDiagnostics? diagnostics = null, int maximumQueryVisits = 65536)
    {
        cancellationToken.ThrowIfCancellationRequested();
        if (maximumQueryVisits is < 1 or > 65536) throw new ArgumentOutOfRangeException(nameof(maximumQueryVisits));
        if (diagnostics is not null) diagnostics.Queries++;
        if (diagnostics is not null) diagnostics.LastQueryStatus = "query-running-NOM-only";
        if (maximumResults <= 0 || Count == 0)
        { if (diagnostics is not null) diagnostics.LastQueryStatus = "empty-NOM-no-negative-credit"; yield break; }
        if (!_finite || !double.IsFinite(x) || !double.IsFinite(y))
        {
            throw new ArgumentOutOfRangeException(nameof(x), "Unsupported weighted nomination query cannot scan or sort every source point.");
        }
        var queue = new PriorityQueue<Work, (double Distance, int Ordinal, int Kind)>();
        EnqueueNode(0);
        int visits = 0, emitted = 0;
        while (queue.TryDequeue(out Work work, out _))
        {
            if (++visits > maximumQueryVisits)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (diagnostics is not null) { diagnostics.BoundedQueries++; diagnostics.LastQueryStatus = "UNKNOWN-query-visit-bound-no-negative-credit"; }
                yield break;
            }
            if ((visits & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            if (work.Node >= 0)
            {
                Node node = _nodes[work.Node];
                if (node.Left >= 0) { EnqueueNode(node.Left); EnqueueNode(node.Right); }
                else for (int i = node.First; i < node.First + node.Count; i++)
                {
                    Entry entry = _entries[i];
                    if (diagnostics is not null) diagnostics.PointDistances++;
                    queue.Enqueue(new(-1, i), (Math.Sqrt(DistanceSquared(entry.Point, x, y)) - entry.Radius, entry.Ordinal, 1));
                }
                if (diagnostics is not null) diagnostics.PeakQueue = Math.Max(diagnostics.PeakQueue, queue.Count);
            }
            else
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (diagnostics is not null) diagnostics.ReturnedPoints++;
                Entry entry = _entries[work.Entry];
                yield return new(entry.Ordinal, entry.Point, entry.Radius,
                    Math.Sqrt(DistanceSquared(entry.Point, x, y)) - entry.Radius);
                if (++emitted == maximumResults)
                { if (diagnostics is not null) diagnostics.LastQueryStatus = "requested-prefix-complete-NOM-only"; yield break; }
            }
        }
        if (diagnostics is not null) diagnostics.LastQueryStatus = "enumeration-complete-NOM-only";

        void EnqueueNode(int index)
        {
            Node node = _nodes[index];
            double dx = Math.Max(0, Math.Max(node.MinX - x, x - node.MaxX));
            double dy = Math.Max(0, Math.Max(node.MinY - y, y - node.MaxY));
            // The coordinate interval bounds are monotone under IEEE subtraction,
            // squaring and addition. Round the result downward too so equality
            // cannot let a point pass an unexpanded subtree with an earlier tie.
            double squaredLower = Math.Max(0, Math.BitDecrement(dx * dx + dy * dy));
            double lower = Math.BitDecrement(Math.BitDecrement(Math.Sqrt(squaredLower)) - node.MaximumRadius);
            if (diagnostics is not null) diagnostics.NodeBounds++;
            queue.Enqueue(new(index, -1), (lower, node.MinimumOrdinal, 0));
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
