using System.Numerics;

namespace RW.Toolpaths;

internal sealed partial class NativeSegmentArrangement
{
    internal readonly record struct WidePoint(long X, long Y);
    internal sealed record WideAtom(WidePoint A, WidePoint B, int OriginalOrdinal, int[] AdditionalOriginalOrdinals);
    internal sealed record WideResult(IReadOnlyList<WideAtom> Atoms, long Scale, int SplitOriginalSites, int DuplicateAtoms);
    private bool _allowRational;
    private readonly Dictionary<int, List<(BigInteger X, BigInteger Y, BigInteger Denominator)>> _rationalSplits = new();

    // Retry only unsupported fractional contacts. All arithmetic through native
    // submission is exact; no original segment or signed ring is simplified.
    internal static WideResult PrepareWide(IReadOnlyList<Segment> source, CancellationToken token = default)
    {
        var arrangement = new NativeSegmentArrangement(source, token, null) { _allowRational = true };
        Node? root = source.Count == 0 ? null : arrangement.Build(0, source.Count);
        for (int i = 0; i < source.Count; i++)
        {
            if ((i & 255) == 0) token.ThrowIfCancellationRequested();
            if (root is not null) arrangement.Visit(root, i);
        }
        return arrangement.CertifyWideUnion();
    }

    private void AddRationalSplit(int first, int second, Int128 xn, Int128 yn, Int128 denominator)
    {
        BigInteger x = (BigInteger)xn, y = (BigInteger)yn, divisor = (BigInteger)denominator;
        if (divisor.Sign < 0) { x = -x; y = -y; divisor = -divisor; }
        BigInteger gcd = BigInteger.GreatestCommonDivisor(BigInteger.GreatestCommonDivisor(x, y), divisor);
        var point = (x / gcd, y / gcd, divisor / gcd);
        if (!_rationalSplits.TryGetValue(first, out var a)) _rationalSplits.Add(first, a = new());
        if (!_rationalSplits.TryGetValue(second, out var b)) _rationalSplits.Add(second, b = new());
        a.Add(point); b.Add(point);
    }

    private WideResult CertifyWideUnion()
    {
        BigInteger scale = BigInteger.One;
        foreach (var splits in _rationalSplits.Values)
            foreach (var point in splits)
            {
                _token.ThrowIfCancellationRequested();
                scale = scale / BigInteger.GreatestCommonDivisor(scale, point.Denominator) * point.Denominator;
                if (scale > (BigInteger.One << 50)) Fail("rational-scale-range", 0, -1);
            }
        var atoms = new List<WideAtom>();
        var unique = new Dictionary<(WidePoint, WidePoint), int>();
        int duplicates = 0, splitSites = 0;
        for (int ordinal = 0; ordinal < _source.Length; ordinal++)
        {
            if ((ordinal & 255) == 0) _token.ThrowIfCancellationRequested();
            Segment original = _source[ordinal];
            WidePoint a = Promote(original.A), b = Promote(original.B);
            var points = new HashSet<WidePoint> { a, b };
            if (_splits.TryGetValue(ordinal, out var integral))
                foreach (var p in integral) points.Add(Promote(p));
            if (_rationalSplits.TryGetValue(ordinal, out var fractional))
                foreach (var p in fractional)
                    points.Add(Wide(p.X * (scale / p.Denominator), p.Y * (scale / p.Denominator)));
            if (points.Count > 2) splitSites++;
            BigInteger dx = (BigInteger)b.X - a.X, dy = (BigInteger)b.Y - a.Y;
            BigInteger Projection(WidePoint p) => dx * ((BigInteger)p.X - a.X) + dy * ((BigInteger)p.Y - a.Y);
            BigInteger length = Projection(b);
            WidePoint cursor = a;
            foreach (var next in points.OrderBy(Projection).Skip(1))
            {
                if (dx * ((BigInteger)next.Y - a.Y) != dy * ((BigInteger)next.X - a.X)
                    || Projection(next) <= Projection(cursor) || Projection(next) > length)
                    Fail("rational-subchain-certificate", ordinal, -1);
                var key = cursor.X < next.X || cursor.X == next.X && cursor.Y < next.Y ? (cursor, next) : (next, cursor);
                if (unique.TryGetValue(key, out int existing))
                {
                    var atom = atoms[existing];
                    atoms[existing] = atom with { AdditionalOriginalOrdinals = atom.AdditionalOriginalOrdinals.Append(ordinal).ToArray() };
                    duplicates++;
                }
                else
                {
                    unique.Add(key, atoms.Count);
                    atoms.Add(new(cursor, next, ordinal, Array.Empty<int>()));
                }
                cursor = next;
            }
            if (cursor != b) Fail("rational-subchain-endpoint", ordinal, -1);
        }
        _token.ThrowIfCancellationRequested();
        Console.WriteLine($"[BoostVoronoi.SegmentArrangement] mode=exact-rational-finite-union originalSites={_source.Length} "
            + $"atomicSites={atoms.Count} scale={scale} fractionalSites={_rationalSplits.Count} splitOriginalSites={splitSites} duplicateAtoms={duplicates} "
            + $"completeOriginalFiniteUnion=True originalSubchainsVerified=True originalOwnerMappings=True originalSignedRings=True geometryDisplacement=0");
        return new(atoms, (long)scale, splitSites, duplicates);

        WidePoint Promote(Point p) => Wide(p.X * scale, p.Y * scale);
        WidePoint Wide(BigInteger x, BigInteger y)
        {
            // Native wide predicates admit at most 50 coordinate bits, keeping
            // every submitted integer exactly representable by double as well.
            BigInteger limit = BigInteger.One << 50;
            if (BigInteger.Abs(x) > limit || BigInteger.Abs(y) > limit) Fail("rational-coordinate-range", 0, -1);
            return new((long)x, (long)y);
        }
    }
}
