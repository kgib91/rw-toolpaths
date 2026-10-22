using System.Diagnostics;
using Clipper2Lib;

namespace RW.Toolpaths.Milling.Acceleration;

internal readonly record struct TargetWindingSegment(double Ax, double Ay, double Bx, double By);

/// <summary>Invocation-owned exact reuse of the literal boundary winding test.
/// Dyadic certificate boxes repeat Y rows. A prepared row counts rays strictly
/// separated from outward crossing intervals with prefix sums; ambiguous edges
/// execute the original floating-point cross expression, without approximation.
/// Unsupported inputs return false so the caller retains its original BVH.</summary>
internal sealed class PreparedTargetWindingRows
{
    private const double MaximumCoordinate = 10_000;
    private const double MinimumRayDelta = 1e-100;
    private const double CrossingReserve = 1e-8;
    private readonly TargetWindingSegment[] _segments;
    private readonly List<YNode> _nodes = new();
    private readonly Dictionary<long, Row> _rows = new();
    private readonly Queue<long> _rowOrder = new();
    private readonly CancellationToken _cancellation;
    private readonly Action<string>? _diagnostic;
    private readonly Action? _workCheckpoint;
    private readonly int _maximumRows;
    private readonly long _maximumBytes;
    private readonly bool _supported;
    private long _lastDiagnostic = Stopwatch.GetTimestamp();
    internal long Hits { get; private set; }
    internal long Misses { get; private set; }
    internal long AmbiguousEvaluations { get; private set; }
    internal long RefusedRows { get; private set; }
    internal long Evictions { get; private set; }
    internal long CacheBytes { get; private set; }
    internal long RowEdges { get; private set; }
    internal double PreparationMilliseconds { get; }
    internal double RowBuildMilliseconds { get; private set; }
    internal int Rows => _rows.Count;

    internal PreparedTargetWindingRows(TargetWindingSegment[] segments, CancellationToken cancellationToken,
        Action<string>? diagnostic = null, int maximumRows = 65536, long maximumBytes = 64L * 1024 * 1024,
        Action? workCheckpoint = null)
    {
        ArgumentNullException.ThrowIfNull(segments);
        if (maximumRows < 1 || maximumBytes < 64) throw new ArgumentOutOfRangeException(nameof(maximumRows));
        _cancellation = cancellationToken; _diagnostic = diagnostic; _workCheckpoint = workCheckpoint;
        _maximumRows = maximumRows; _maximumBytes = maximumBytes;
        Check(); long started = Stopwatch.GetTimestamp();
        _supported = segments.All(s => Ordinary(s.Ax) && Ordinary(s.Ay) && Ordinary(s.Bx) && Ordinary(s.By));
        _segments = _supported ? segments.Where(s => s.Ay != s.By).ToArray() : Array.Empty<TargetWindingSegment>();
        if (_segments.Length != 0) Build(0, _segments.Length);
        PreparationMilliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        _diagnostic?.Invoke($"stage=smooth-target-winding event=prepared sourceEdges={segments.Length} rayEdges={_segments.Length} " +
            $"nodes={_nodes.Count} supported={_supported} maximumRows={maximumRows} maximumBytes={maximumBytes} " +
            $"preparationMs={PreparationMilliseconds:F2} authority=literal-cross-and-outward-row-intervals");
    }

    internal bool TryIsInside(PointD q, out bool inside)
    {
        inside = false; Check();
        if (!_supported || !Ordinary(q.x) || !Ordinary(q.y)) { RefusedRows++; return false; }
        long key = BitConverter.DoubleToInt64Bits(q.y);
        if (_rows.TryGetValue(key, out Row? row)) Hits++;
        else
        {
            Misses++; long started = Stopwatch.GetTimestamp();
            row = PrepareRow(q.y); RowBuildMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            if (row is null) { RefusedRows++; return false; }
            while (_rows.Count >= _maximumRows || CacheBytes > _maximumBytes - row.Bytes)
            {
                if (!_rowOrder.TryDequeue(out long oldest)) { RefusedRows++; return false; }
                CacheBytes -= _rows[oldest].Bytes; _rows.Remove(oldest); Evictions++;
            }
            _rows.Add(key, row); _rowOrder.Enqueue(key); CacheBytes += row.Bytes;
        }
        Entry[] entries = row.Entries;
        int firstAmbiguous = LowerBoundHigh(entries, q.x);
        int firstRight = UpperBoundLow(entries, q.x);
        int winding = row.Prefix[^1] - row.Prefix[firstRight];
        for (int i = firstAmbiguous; i < firstRight; i++)
        {
            if ((i & 1023) == 0) Check();
            TargetWindingSegment s = _segments[entries[i].Ordinal];
            double cross = (s.Bx - s.Ax) * (q.y - s.Ay) - (q.x - s.Ax) * (s.By - s.Ay);
            if (s.Ay <= q.y && s.By > q.y && cross > 0) winding++;
            else if (s.Ay > q.y && s.By <= q.y && cross < 0) winding--;
            AmbiguousEvaluations++;
        }
        inside = winding != 0;
        if (Stopwatch.GetElapsedTime(_lastDiagnostic).TotalSeconds >= 2)
        {
            _lastDiagnostic = Stopwatch.GetTimestamp(); ReportDiagnostics("progress");
        }
        return true;
    }

    internal void ReportDiagnostics(string eventName = "complete") => _diagnostic?.Invoke(
        $"stage=smooth-target-winding event={eventName} hits={Hits} misses={Misses} rows={Rows} cacheBytes={CacheBytes} " +
        $"rowEdges={RowEdges} ambiguousEvaluations={AmbiguousEvaluations} refusedRows={RefusedRows} evictions={Evictions} " +
        $"preparationMs={PreparationMilliseconds:F2} rowBuildMs={RowBuildMilliseconds:F2}");

    private Row? PrepareRow(double y)
    {
        var entries = new List<Entry>(); bool refused = false;
        if (_nodes.Count != 0) Visit(0);
        if (refused) return null;
        Entry[] sorted = entries.ToArray();
        Array.Sort(sorted, static (a, b) =>
        {
            int order = a.Low.CompareTo(b.Low); return order == 0 ? a.High.CompareTo(b.High) : order;
        });
        var prefix = new int[sorted.Length + 1];
        double previousHigh = double.NegativeInfinity;
        for (int i = 0; i < sorted.Length; i++)
        {
            if ((i & 1023) == 0) Check();
            // All intervals use the same reserve and monotone outward steps.
            // Refuse if platform arithmetic ever violates sorted-high order.
            if (sorted[i].High < previousHigh) return null;
            previousHigh = sorted[i].High; prefix[i + 1] = prefix[i] + sorted[i].Sign;
        }
        RowEdges += sorted.Length;
        return new(sorted, prefix);

        void Visit(int index)
        {
            Check(); YNode node = _nodes[index];
            if (y < node.Min || y >= node.Max || refused) return;
            if (node.Left >= 0) { Visit(node.Left); Visit(node.Right); return; }
            for (int i = node.First; i < node.First + node.Count; i++)
            {
                TargetWindingSegment s = _segments[i];
                int sign = s.Ay <= y && s.By > y ? 1 : s.Ay > y && s.By <= y ? -1 : 0;
                if (sign == 0) continue;
                double dy = s.By - s.Ay;
                if (!double.IsFinite(dy) || Math.Abs(dy) < MinimumRayDelta) { refused = true; return; }
                // Treat the first product as the literal double used by the
                // original cross expression, not an ideal interpolated edge.
                double left = (s.Bx - s.Ax) * (y - s.Ay);
                double quotient = left / dy, crossing = s.Ax + quotient;
                if (!double.IsFinite(left) || !double.IsFinite(quotient) || !double.IsFinite(crossing)
                    || Math.Abs(quotient) > 3 * MaximumCoordinate) { refused = true; return; }
                // For |x|,|Ax|<=S, subtraction error is <=2uS; multiplication
                // divided by |dy| adds <=3uS (including subnormals, dy>=1e-100).
                // Division and addition add <=5uS. The 1e-8 enclosure exceeds
                // 128uS (1.5e-10) and is stepped outward once more. Outside it,
                // the two finite literal products are strictly ordered, so
                // their floating subtraction has the same strict cross sign.
                // Inside it the exact original expression above is evaluated.
                double low = Math.BitDecrement(crossing - CrossingReserve);
                double high = Math.BitIncrement(crossing + CrossingReserve);
                if (!double.IsFinite(low) || !double.IsFinite(high)) { refused = true; return; }
                entries.Add(new(low, high, i, sign));
                if (160L + 24L * entries.Count + 4L * (entries.Count + 1) > _maximumBytes)
                { refused = true; return; }
            }
        }
    }

    private int Build(int first, int count)
    {
        Check(); double min = double.PositiveInfinity, max = double.NegativeInfinity;
        for (int i = first; i < first + count; i++)
        {
            if ((i & 2047) == 0) Check();
            TargetWindingSegment s = _segments[i];
            min = Math.Min(min, Math.Min(s.Ay, s.By)); max = Math.Max(max, Math.Max(s.Ay, s.By));
        }
        int index = _nodes.Count; _nodes.Add(default);
        int left = -1, right = -1;
        if (count > 32)
        {
            int half = count / 2; Partition(first, first + count - 1, first + half);
            left = Build(first, half); right = Build(first + half, count - half);
        }
        _nodes[index] = new(min, max, first, count, left, right); return index;
    }

    private void Partition(int lo, int hi, int middle)
    {
        static double Key(TargetWindingSegment s) => s.Ay / 2 + s.By / 2;
        while (lo < hi)
        {
            Check(); double pivot = Key(_segments[lo + (hi - lo) / 2]); int i = lo, j = hi;
            while (i <= j)
            {
                while (Key(_segments[i]) < pivot) i++;
                while (Key(_segments[j]) > pivot) j--;
                if (i <= j) { (_segments[i], _segments[j]) = (_segments[j], _segments[i]); i++; j--; }
            }
            if (middle <= j) hi = j; else if (middle >= i) lo = i; else break;
        }
    }

    private static int LowerBoundHigh(Entry[] entries, double x)
    {
        int lo = 0, hi = entries.Length;
        while (lo < hi) { int middle = lo + (hi - lo) / 2; if (entries[middle].High < x) lo = middle + 1; else hi = middle; }
        return lo;
    }
    private static int UpperBoundLow(Entry[] entries, double x)
    {
        int lo = 0, hi = entries.Length;
        while (lo < hi) { int middle = lo + (hi - lo) / 2; if (entries[middle].Low <= x) lo = middle + 1; else hi = middle; }
        return lo;
    }
    private void Check() { _cancellation.ThrowIfCancellationRequested(); _workCheckpoint?.Invoke(); }
    private static bool Ordinary(double value) => double.IsFinite(value) && Math.Abs(value) <= MaximumCoordinate;
    private readonly record struct YNode(double Min, double Max, int First, int Count, int Left, int Right);
    private readonly record struct Entry(double Low, double High, int Ordinal, int Sign);
    private sealed record Row(Entry[] Entries, int[] Prefix)
    {
        // Includes conservative row/array/dictionary/queue metadata allowance,
        // keeping the byte cap meaningful when tens of thousands of tiny rows fit.
        internal long Bytes => 160L + 24L * Entries.Length + 4L * Prefix.Length;
    }
}
