using System.Diagnostics;

namespace RW.Toolpaths;

internal static partial class MedialAxisEdgeClassifier
{
    // Classification receives immutable native vertices and complete original
    // rings. Neither this index nor its bounded memo can add geometric authority:
    // every uncached answer uses the existing asymmetric ray-crossing predicate.
    internal sealed class PreparedRegionContainment
    {
        internal const int DefaultMemoLimit = 262144;
        private readonly PreparedRingContainment _boundary;
        private readonly IReadOnlyList<PreparedRingContainment> _holes;
        private readonly CancellationToken _token;
        private readonly Dictionary<EdgePoint, byte>? _memo;
        private readonly HoleEntry[]? _entries;
        private readonly HoleNode? _root;

        private readonly record struct HoleEntry(PreparedRingContainment Ring,
            double MinX, double MinY, double MaxX, double MaxY, int OriginalOrdinal);
        private sealed record HoleNode(double MinX, double MinY, double MaxX, double MaxY,
            int Start, int Count, HoleNode? First = null, HoleNode? Second = null);

        internal long Queries { get; private set; }
        internal long MemoHits { get; private set; }
        internal int MemoClears { get; private set; }
        internal int MemoCount => _memo?.Count ?? 0;
        internal int MemoLimit { get; }
        internal long HoleNodeVisits { get; private set; }
        internal long HoleBoundsTests { get; private set; }
        internal long HolePredicateTests { get; private set; }
        internal double HoleIndexBuildMilliseconds { get; }
        internal double QueryMilliseconds { get; private set; }

        internal PreparedRegionContainment(PreparedRingContainment boundary,
            IReadOnlyList<PreparedRingContainment> holes, CancellationToken token = default,
            bool useHoleIndex = true, int memoLimit = DefaultMemoLimit)
        {
            token.ThrowIfCancellationRequested();
            if (memoLimit < 0) throw new ArgumentOutOfRangeException(nameof(memoLimit));
            _boundary = boundary; _holes = holes; _token = token; MemoLimit = memoLimit;
            // Reference identity certifies the exact same immutable native point.
            // A clear retains dictionary storage, bounds retained vertex references
            // and simply recomputes future answers through the original predicates.
            if (memoLimit > 0) _memo = new(ReferenceEqualityComparer.Instance);
            long started = Stopwatch.GetTimestamp();
            if (useHoleIndex && holes.Count > 8)
            {
                var entries = new List<HoleEntry>(holes.Count);
                bool finite = true;
                for (int ordinal = 0; ordinal < holes.Count; ordinal++)
                {
                    if ((ordinal & 255) == 0) token.ThrowIfCancellationRequested();
                    var hole = holes[ordinal];
                    if (hole.IsEmpty) continue;
                    var bounds = hole.Bounds;
                    if (!double.IsFinite(bounds.MinX) || !double.IsFinite(bounds.MinY)
                        || !double.IsFinite(bounds.MaxX) || !double.IsFinite(bounds.MaxY))
                    { finite = false; break; }
                    entries.Add(new(hole, bounds.MinX, bounds.MinY, bounds.MaxX, bounds.MaxY, ordinal));
                }
                // Nonfinite legacy inputs retain the entire legacy path. No new
                // bounds convention or inferred absence replaces its predicate.
                if (finite && entries.Count > 0)
                {
                    _entries = entries.ToArray();
                    _root = Build(0, _entries.Length);
                }
            }
            token.ThrowIfCancellationRequested();
            HoleIndexBuildMilliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        }

        internal bool Contains(EdgePoint point, out bool isInsideHole)
        {
            _token.ThrowIfCancellationRequested();
            long started = Stopwatch.GetTimestamp();
            try
            {
                Queries++;
                if (_memo is not null && _memo.TryGetValue(point, out byte cached))
                {
                    MemoHits++;
                    isInsideHole = cached == 2;
                    return cached == 1;
                }
                byte result;
                if (!_boundary.Contains(point)) result = 0;
                else if (_root is not null ? ContainsHole(_root, point) : ContainsAnyHole(point)) result = 2;
                else result = 1;
                if (_memo is not null)
                {
                    if (_memo.Count == MemoLimit) { _memo.Clear(); MemoClears++; }
                    _memo.Add(point, result);
                }
                isInsideHole = result == 2;
                return result == 1;
            }
            finally { QueryMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds; }
        }

        private bool ContainsAnyHole(EdgePoint point)
        {
            foreach (var hole in _holes)
            {
                HoleBoundsTests++; HolePredicateTests++;
                if (hole.Contains(point)) return true;
            }
            return false;
        }

        private bool ContainsHole(HoleNode node, EdgePoint point)
        {
            HoleNodeVisits++;
            if (point.X < node.MinX || point.X > node.MaxX
                || point.Y < node.MinY || point.Y > node.MaxY) return false;
            if (node.Count == 0)
                return ContainsHole(node.First!, point) || ContainsHole(node.Second!, point);
            for (int index = node.Start; index < node.Start + node.Count; index++)
            {
                var entry = _entries![index];
                HoleBoundsTests++;
                if (point.X < entry.MinX || point.X > entry.MaxX
                    || point.Y < entry.MinY || point.Y > entry.MaxY) continue;
                HolePredicateTests++;
                if (entry.Ring.Contains(point)) return true;
            }
            return false;
        }

        private HoleNode Build(int start, int count)
        {
            _token.ThrowIfCancellationRequested();
            double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
            double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
            for (int index = start; index < start + count; index++)
            {
                var entry = _entries![index];
                minX = Math.Min(minX, entry.MinX); minY = Math.Min(minY, entry.MinY);
                maxX = Math.Max(maxX, entry.MaxX); maxY = Math.Max(maxY, entry.MaxY);
            }
            if (count <= 8) return new(minX, minY, maxX, maxY, start, count);
            bool x = maxX - minX >= maxY - minY;
            Array.Sort(_entries!, start, count, Comparer<HoleEntry>.Create((a, b) =>
            {
                int compare = (x ? a.MinX + a.MaxX : a.MinY + a.MaxY)
                    .CompareTo(x ? b.MinX + b.MaxX : b.MinY + b.MaxY);
                return compare != 0 ? compare : a.OriginalOrdinal.CompareTo(b.OriginalOrdinal);
            }));
            _token.ThrowIfCancellationRequested();
            int first = count / 2;
            return new(minX, minY, maxX, maxY, 0, 0, Build(start, first), Build(start + first, count - first));
        }
    }
}
