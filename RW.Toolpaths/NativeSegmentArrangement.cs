using System.Diagnostics;

namespace RW.Toolpaths;

// Boost requires disjoint segment interiors. Integer Boolean boundaries may
// backtrack over a one-tick subsegment or meet another segment in its interior.
// Split only exact contacts: the complete finite-site union and the separate
// original signed-ring classifier remain identical, with no geometry allowance.
internal sealed partial class NativeSegmentArrangement
{
    internal readonly record struct Point(int X, int Y);
    internal readonly record struct Segment(Point A, Point B);
    internal sealed record Atom(Segment Segment, int OriginalOrdinal, int[] AdditionalOriginalOrdinals);
    internal sealed class UnsupportedContactException : InvalidOperationException
    {
        internal string Reason { get; }
        internal int FirstOriginal { get; }
        internal int SecondOriginal { get; }
        internal UnsupportedContactException(string message, string reason, int firstOriginal, int secondOriginal)
            : base(message)
        {
            Reason = reason; FirstOriginal = firstOriginal; SecondOriginal = secondOriginal;
        }
    }
    internal sealed record Result(IReadOnlyList<Atom>? Atoms, int OriginalSites, int SplitOriginalSites,
        int DuplicateAtoms, long CandidatePairs, long NodeVisits, double BuildMilliseconds,
        double QueryMilliseconds, double CertificateMilliseconds)
    {
        internal bool Changed => Atoms is not null;
        internal int AtomicSites => Atoms?.Count ?? OriginalSites;
    }
    private readonly record struct Bounds(int MinX, int MinY, int MaxX, int MaxY)
    {
        internal bool Meets(Bounds other) => MaxX >= other.MinX && MinX <= other.MaxX
            && MaxY >= other.MinY && MinY <= other.MaxY;
    }
    private sealed record Node(Bounds Bounds, int Start, int Count, Node? First = null, Node? Second = null);
    private sealed class AtomBuilder
    {
        internal readonly Segment Segment;
        internal readonly int OriginalOrdinal;
        internal List<int>? AdditionalOriginalOrdinals;
        internal AtomBuilder(Segment segment, int originalOrdinal)
        {
            Segment = segment;
            OriginalOrdinal = originalOrdinal;
        }
    }
    private readonly Segment[] _source;
    private readonly Bounds[] _bounds;
    private readonly int[] _order;
    private readonly CancellationToken _token;
    private readonly long _pairLimit, _nodeLimit;
    private readonly Dictionary<int, HashSet<Point>> _splits = new();
    private long _candidatePairs, _nodeVisits;

    private NativeSegmentArrangement(IReadOnlyList<Segment> source, CancellationToken token, long? pairLimit)
    {
        token.ThrowIfCancellationRequested();
        _source = source.ToArray(); _token = token;
        _pairLimit = pairLimit ?? Math.Max(1_000_000, 64L * source.Count);
        _nodeLimit = Math.Max(1_000_000, 128L * source.Count);
        _bounds = new Bounds[source.Count]; _order = new int[source.Count];
        for (int i = 0; i < source.Count; i++)
        {
            if ((i & 1023) == 0) token.ThrowIfCancellationRequested();
            Segment s = _source[i];
            if (s.A == s.B) throw new InvalidOperationException($"Native segment arrangement received a zero-length original site: {i}.");
            _bounds[i] = new(Math.Min(s.A.X, s.B.X), Math.Min(s.A.Y, s.B.Y), Math.Max(s.A.X, s.B.X), Math.Max(s.A.Y, s.B.Y));
            _order[i] = i;
        }
    }

    internal static Result Prepare(IReadOnlyList<Segment> source, CancellationToken token = default,
        long? candidatePairLimit = null)
    {
        var arrangement = new NativeSegmentArrangement(source, token, candidatePairLimit);
        long started = Stopwatch.GetTimestamp();
        Node? root = source.Count == 0 ? null : arrangement.Build(0, source.Count);
        double buildMs = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        started = Stopwatch.GetTimestamp();
        for (int i = 0; i < source.Count; i++)
        {
            if ((i & 255) == 0) token.ThrowIfCancellationRequested();
            if (root is not null) arrangement.Visit(root, i);
        }
        double queryMs = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        started = Stopwatch.GetTimestamp();
        if (arrangement._splits.Count == 0)
            return new(null, source.Count, 0, 0, arrangement._candidatePairs, arrangement._nodeVisits, buildMs, queryMs, 0);
        var atoms = arrangement.CertifyCompleteUnion(out int duplicateAtoms);
        token.ThrowIfCancellationRequested();
        return new(atoms, source.Count, arrangement._splits.Count, duplicateAtoms,
            arrangement._candidatePairs, arrangement._nodeVisits, buildMs, queryMs,
            Stopwatch.GetElapsedTime(started).TotalMilliseconds);
    }

    private Node Build(int start, int count)
    {
        _token.ThrowIfCancellationRequested();
        int minX = int.MaxValue, minY = int.MaxValue, maxX = int.MinValue, maxY = int.MinValue;
        for (int i = start; i < start + count; i++)
        {
            Bounds b = _bounds[_order[i]];
            minX = Math.Min(minX, b.MinX); minY = Math.Min(minY, b.MinY);
            maxX = Math.Max(maxX, b.MaxX); maxY = Math.Max(maxY, b.MaxY);
        }
        var bounds = new Bounds(minX, minY, maxX, maxY);
        if (count <= 8) return new(bounds, start, count);
        bool x = (long)maxX - minX >= (long)maxY - minY;
        Partition(start, count, x);
        int first = count / 2;
        return new(bounds, 0, 0, Build(start, first), Build(start + first, count - first));
    }

    private void Partition(int start, int count, bool x)
    {
        int left = start, right = start + count - 1, median = start + count / 2;
        int budget = 2 * (int)Math.Log2(count) + 2;
        int Compare(int a, int b)
        {
            Bounds aa = _bounds[a], bb = _bounds[b];
            long av = x ? (long)aa.MinX + aa.MaxX : (long)aa.MinY + aa.MaxY;
            long bv = x ? (long)bb.MinX + bb.MaxX : (long)bb.MinY + bb.MaxY;
            int order = av.CompareTo(bv); return order != 0 ? order : a.CompareTo(b);
        }
        while (left < right)
        {
            _token.ThrowIfCancellationRequested();
            if (--budget < 0)
            {
                Array.Sort(_order, left, right - left + 1, Comparer<int>.Create(Compare));
                _token.ThrowIfCancellationRequested(); break;
            }
            int middle = left + (right - left) / 2;
            if (Compare(_order[middle], _order[left]) < 0) (_order[middle], _order[left]) = (_order[left], _order[middle]);
            if (Compare(_order[right], _order[left]) < 0) (_order[right], _order[left]) = (_order[left], _order[right]);
            if (Compare(_order[right], _order[middle]) < 0) (_order[right], _order[middle]) = (_order[middle], _order[right]);
            int pivot = _order[middle], first = left, last = right;
            while (first <= last)
            {
                while (Compare(_order[first], pivot) < 0) first++;
                while (Compare(_order[last], pivot) > 0) last--;
                if (first > last) break;
                (_order[first], _order[last]) = (_order[last], _order[first]); first++; last--;
            }
            if (median <= last) right = last;
            else if (median >= first) left = first;
            else break;
        }
    }

    private void Visit(Node node, int ordinal)
    {
        if (++_nodeVisits > _nodeLimit) Fail("node-work-budget", ordinal, -1);
        if ((_nodeVisits & 4095) == 0) _token.ThrowIfCancellationRequested();
        if (!node.Bounds.Meets(_bounds[ordinal])) return;
        if (node.Count == 0) { Visit(node.First!, ordinal); Visit(node.Second!, ordinal); return; }
        for (int i = node.Start; i < node.Start + node.Count; i++)
        {
            int other = _order[i];
            if (other <= ordinal || !_bounds[ordinal].Meets(_bounds[other])) continue;
            if (++_candidatePairs > _pairLimit) Fail("pair-work-budget", ordinal, other);
            Contact(ordinal, other);
        }
    }

    private void Contact(int first, int second)
    {
        Segment a = _source[first], b = _source[second];
        Int128 c = Cross(a.A, a.B, b.A), d = Cross(a.A, a.B, b.B);
        Int128 e = Cross(b.A, b.B, a.A), f = Cross(b.A, b.B, a.B);
        if (Opposite(c, d) && Opposite(e, f))
        {
            long dx = (long)a.B.X - a.A.X, dy = (long)a.B.Y - a.A.Y;
            long ex = (long)b.B.X - b.A.X, ey = (long)b.B.Y - b.A.Y;
            Int128 denominator = (Int128)dx * ey - (Int128)dy * ex;
            Int128 numerator = (Int128)((long)b.A.X - a.A.X) * ey - (Int128)((long)b.A.Y - a.A.Y) * ex;
            Int128 xn = (Int128)a.A.X * denominator + (Int128)dx * numerator;
            Int128 yn = (Int128)a.A.Y * denominator + (Int128)dy * numerator;
            if (xn % denominator != 0 || yn % denominator != 0)
            {
                if (!_allowRational) Fail("noninteger-proper-crossing", first, second, unsupportedContact: true);
                AddRationalSplit(first, second, xn, yn, denominator);
                return;
            }
            Int128 x = xn / denominator, y = yn / denominator;
            if (x < int.MinValue || x > int.MaxValue || y < int.MinValue || y > int.MaxValue)
                throw new OverflowException("Exact native contact lies outside Int32 coordinates.");
            // Range is explicit: C# 10 cannot select checked Int128 operators.
            var intersection = new Point((int)x, (int)y);
            AddSplit(first, intersection); AddSplit(second, intersection);
        }
        if (c == 0) AddSplit(first, b.A);
        if (d == 0) AddSplit(first, b.B);
        if (e == 0) AddSplit(second, a.A);
        if (f == 0) AddSplit(second, a.B);
    }

    private void AddSplit(int ordinal, Point point)
    {
        Segment s = _source[ordinal];
        if (point == s.A || point == s.B || Cross(s.A, s.B, point) != 0) return;
        Int128 projection = Dot(s, point), length = Dot(s, s.B);
        if (projection <= 0 || projection >= length) return;
        if (!_splits.TryGetValue(ordinal, out var points)) _splits.Add(ordinal, points = new());
        points.Add(point);
    }

    private IReadOnlyList<Atom> CertifyCompleteUnion(out int duplicateAtoms)
    {
        var ordered = new List<AtomBuilder>(_source.Length + _splits.Sum(p => p.Value.Count));
        var unique = new Dictionary<(Point, Point), AtomBuilder>(ordered.Capacity);
        duplicateAtoms = 0;
        for (int ordinal = 0; ordinal < _source.Length; ordinal++)
        {
            if ((ordinal & 255) == 0) _token.ThrowIfCancellationRequested();
            Segment original = _source[ordinal];
            Point[] points = _splits.TryGetValue(ordinal, out var knots)
                ? knots.OrderBy(p => Dot(original, p)).Prepend(original.A).Append(original.B).ToArray()
                : new[] { original.A, original.B };
            Point cursor = original.A;
            for (int i = 1; i < points.Length; i++)
            {
                Point next = points[i];
                // Every atom lies on its complete original finite segment; the
                // directed, strictly increasing subchain covers A through B.
                // Shared atoms have all original owners, even in reverse order.
                if (Cross(original.A, original.B, next) != 0 || Dot(original, next) <= Dot(original, cursor)
                    || Dot(original, next) > Dot(original, original.B)) Fail("subchain-certificate", ordinal, -1);
                var key = Compare(cursor, next) <= 0 ? (cursor, next) : (next, cursor);
                if (unique.TryGetValue(key, out var atom))
                {
                    atom.AdditionalOriginalOrdinals ??= new();
                    atom.AdditionalOriginalOrdinals.Add(ordinal); duplicateAtoms++;
                }
                else
                {
                    atom = new(new(cursor, next), ordinal); unique.Add(key, atom); ordered.Add(atom);
                }
                cursor = next;
            }
            if (cursor != original.B) Fail("subchain-endpoint", ordinal, -1);
        }
        _token.ThrowIfCancellationRequested();
        return ordered.Select(a => new Atom(a.Segment, a.OriginalOrdinal,
            a.AdditionalOriginalOrdinals?.ToArray() ?? Array.Empty<int>())).ToArray();
    }
    private static int Compare(Point a, Point b) => a.X != b.X ? a.X.CompareTo(b.X) : a.Y.CompareTo(b.Y);
    private static Int128 Cross(Point a, Point b, Point c) => (Int128)((long)b.X - a.X) * ((long)c.Y - a.Y)
        - (Int128)((long)b.Y - a.Y) * ((long)c.X - a.X);
    private static Int128 Dot(Segment s, Point p) => (Int128)((long)s.B.X - s.A.X) * ((long)p.X - s.A.X)
        + (Int128)((long)s.B.Y - s.A.Y) * ((long)p.Y - s.A.Y);
    private static bool Opposite(Int128 a, Int128 b) => a < 0 && b > 0 || a > 0 && b < 0;
    private void Fail(string reason, int first, int second, bool unsupportedContact = false)
    {
        Console.WriteLine($"[BoostVoronoi.SegmentArrangement] result=rejected reason={reason} firstOriginal={first} secondOriginal={second} "
            + $"first={_source[first]} second={(second >= 0 ? _source[second].ToString() : "none")} pairs={_candidatePairs} nodeVisits={_nodeVisits} originalSites={_source.Length}");
        string message = $"Native medial segment arrangement could not preserve the complete original finite-site union: {reason}, original sites {first}/{second}.";
        if (unsupportedContact) throw new UnsupportedContactException(message, reason, first, second);
        throw new InvalidOperationException(message);
    }
}
