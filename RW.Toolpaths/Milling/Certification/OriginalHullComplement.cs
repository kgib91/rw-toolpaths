using System.Diagnostics;
using System.Numerics;
using Clipper2Lib;

namespace RW.Toolpaths.Milling.Certification;

// No rounded Boolean subtraction is allowed to decide what
// support disappears. Each removed exact convex piece must fit one literal
// finite ORIGINAL capsule; its exact complementary halfplane stays present.
internal static class OriginalHullComplement
{
    internal readonly record struct Site(Point64 Start, Point64 End);
    private readonly record struct Q(BigInteger X, BigInteger Y, BigInteger D)
    {
        internal static Q Make(BigInteger x, BigInteger y, BigInteger d)
        {
            if (d.IsZero) throw new ArithmeticException("Zero rational divisor");
            if (d.Sign < 0) { x = -x; y = -y; d = -d; }
            var common = BigInteger.GreatestCommonDivisor(BigInteger.GreatestCommonDivisor(BigInteger.Abs(x), BigInteger.Abs(y)), d);
            return new(x / common, y / common, d / common);
        }
    }
    internal sealed class Statistics
    {
        public int Hulls, FullyExcluded, PartialExcluded, Unchanged, Refused, Cuts, ExactVertexTests, MaximumDenominatorBits;
        public double Milliseconds;
        public string LastReason = "not-started";
    }
    private sealed class Capsule
    {
        private readonly Site site;
        private readonly BigInteger dx, dy, radial, factor;
        internal Capsule(Site source, long radius, int bits)
        {
            site = source;
            dx = (BigInteger)site.End.X - site.Start.X; dy = (BigInteger)site.End.Y - site.Start.Y;
            radial = (BigInteger)radius * radius; factor = BigInteger.One << (2 * bits);
        }
        internal bool Contains(Q p)
        {
            BigInteger wx = p.X - site.Start.X * p.D, wy = p.Y - site.Start.Y * p.D;
            BigInteger length = dx * dx + dy * dy, projection = wx * dx + wy * dy;
            BigInteger limit = radial * p.D * p.D;
            if (length.IsZero || projection.Sign <= 0) return (wx * wx + wy * wy) * factor <= limit;
            if (projection >= length * p.D)
            {
                BigInteger vx = p.X - site.End.X * p.D, vy = p.Y - site.End.Y * p.D;
                return (vx * vx + vy * vy) * factor <= limit;
            }
            BigInteger cross = wx * dy - wy * dx;
            return cross * cross * factor <= limit * length;
        }
    }

    internal static bool TryTrim(Path64 input, IReadOnlyList<Site> nominated, Func<Site, bool> isOriginal,
        long radius, int bits, CancellationToken token, Action checkpoint, Statistics statistics, out Paths64 retained)
    {
        long started = Stopwatch.GetTimestamp();
        statistics.Hulls++;
        retained = new();
        token.ThrowIfCancellationRequested(); checkpoint();
        // Bound allocation before cloning any untrusted proposed hull.
        if (input.Count is < 3 or > 32 || nominated.Count is < 1 or > 4)
        { statistics.Refused++; statistics.LastReason = "hull-or-nomination-count-bound"; return false; }
        retained.Add(new(input));
        try
        {
            token.ThrowIfCancellationRequested(); checkpoint();
            if (radius <= 0 || bits is < 0 or > 24 || nominated.Count is < 1 or > 4 || !Normalize(input, out var hull)
                || nominated.Any(s => !InRange(s.Start) || !InRange(s.End) || !isOriginal(s)))
            { statistics.Refused++; statistics.LastReason = "invalid-input-or-original-owner"; return false; }
            var polygon = hull.Select(p => Q.Make(p.X, p.Y, 1)).ToList();
            bool changed = false;
            foreach (var site in nominated.Distinct())
            {
                token.ThrowIfCancellationRequested(); checkpoint();
                var capsule = new Capsule(site, radius, bits);
                bool Inside(Q p) { statistics.ExactVertexTests++; return capsule.Contains(p); }
                if (polygon.All(Inside))
                {
                    retained = new(); statistics.FullyExcluded++; statistics.LastReason = "whole-exact-convex-piece-in-strict-original-capsule";
                    return true;
                }
                // Floating arithmetic only nominates an integer normal. All
                // supporting halfplanes and capsule acceptances below are exact.
                double x = polygon.Average(p => (double)p.X / (double)p.D), y = polygon.Average(p => (double)p.Y / (double)p.D);
                double dx = site.End.X - site.Start.X, dy = site.End.Y - site.Start.Y, length = dx * dx + dy * dy;
                double t = length == 0 ? 0 : Math.Clamp(((x - site.Start.X) * dx + (y - site.Start.Y) * dy) / length, 0, 1);
                double nx = x - site.Start.X - t * dx, ny = y - site.Start.Y - t * dy, norm = Math.Sqrt(nx * nx + ny * ny);
                if (!(norm > 0)) { nx = -dy; ny = dx; norm = Math.Sqrt(length); }
                if (!(norm > 0)) { nx = 1; ny = 0; norm = 1; }
                long ax = (long)Math.Round(nx / norm * (1 << 20)), ay = (long)Math.Round(ny / norm * (1 << 20));
                if (ax == 0 && ay == 0) continue;
                BigInteger Dot(Q p) => (BigInteger)ax * p.X + (BigInteger)ay * p.Y;
                BigInteger low = polygon.Min(p => Floor(Dot(p), p.D)) - 1;
                BigInteger high = polygon.Max(p => Ceiling(Dot(p), p.D)) + 1;
                BigInteger? accepted = null;
                for (int trial = 0; trial < 64 && high - low > 1; trial++)
                {
                    token.ThrowIfCancellationRequested(); checkpoint();
                    BigInteger middle = (low + high) >> 1;
                    var excluded = Clip(polygon, ax, ay, middle);
                    if (excluded.All(Inside)) { low = middle; if (excluded.Count > 0) accepted = middle; }
                    else high = middle;
                }
                if (high - low > 1) { statistics.Refused++; statistics.LastReason = "integer-normal-search-budget"; return false; }
                if (accepted is not { } constant) continue;
                // Recheck the full excluded convex piece immediately before
                // changing support. Boundary overlap is intentionally retained.
                var piece = Clip(polygon, ax, ay, constant);
                if (piece.Count == 0 || !piece.All(Inside)) throw new InvalidOperationException("Exact supporting-cut certificate changed");
                polygon = Clip(polygon, -ax, -ay, -constant);
                if (polygon.Count > 36 || polygon.Any(p => p.D.GetBitLength() > 256))
                { statistics.Refused++; statistics.LastReason = "exact-complement-size-budget"; return false; }
                if (polygon.Count == 0) throw new InvalidOperationException("Unproved empty complement");
                statistics.Cuts++; changed = true;
            }
            if (!changed) { statistics.Unchanged++; statistics.LastReason = "no-supported-excluding-cut"; return true; }
            // Every exact rational vertex lies inside its four integer corners.
            // The hull of ALL corners encloses the entire retained convex set.
            // No inward snap or rounded Boolean operation discards thin pieces.
            var corners = new Path64();
            foreach (var p in polygon)
            {
                statistics.MaximumDenominatorBits = Math.Max(statistics.MaximumDenominatorBits, checked((int)p.D.GetBitLength()));
                long x0 = checked((long)Floor(p.X, p.D)), x1 = checked((long)Ceiling(p.X, p.D));
                long y0 = checked((long)Floor(p.Y, p.D)), y1 = checked((long)Ceiling(p.Y, p.D));
                corners.Add(new(x0,y0)); corners.Add(new(x0,y1)); corners.Add(new(x1,y0)); corners.Add(new(x1,y1));
            }
            var outer = ConvexHull(corners);
            if (outer.Count < 3)
            {
                long x0 = corners.Min(p => p.X) - 1, x1 = corners.Max(p => p.X) + 1;
                long y0 = corners.Min(p => p.Y) - 1, y1 = corners.Max(p => p.Y) + 1;
                outer = new() { new(x0,y0),new(x1,y0),new(x1,y1),new(x0,y1) };
            }
            if (outer.Count > 32 || outer.Any(p => !InRange(p)) || polygon.Any(p => !InsideConvex(outer,p)))
            { statistics.Refused++; statistics.LastReason = "outward-support-envelope-refused"; return false; }
            retained = new() { outer }; statistics.PartialExcluded++; statistics.LastReason = "exact-complement-outward-envelope-certified";
            return true;
        }
        finally { statistics.Milliseconds += (Stopwatch.GetTimestamp() - started) * 1000d / Stopwatch.Frequency; }
    }

    private static List<Q> Clip(List<Q> input, long nx, long ny, BigInteger constant)
    {
        var output = new List<Q>(input.Count + 2);
        for (int i = 0, previous = input.Count - 1; i < input.Count; previous = i++)
        {
            Q a = input[previous], b = input[i];
            BigInteger sa = (BigInteger)nx * a.X + (BigInteger)ny * a.Y - constant * a.D;
            BigInteger sb = (BigInteger)nx * b.X + (BigInteger)ny * b.Y - constant * b.D;
            if ((sa.Sign <= 0) != (sb.Sign <= 0))
                output.Add(Q.Make(a.X * sb - b.X * sa, a.Y * sb - b.Y * sa, a.D * sb - b.D * sa));
            if (sb.Sign <= 0) output.Add(b);
        }
        return output.Distinct().ToList();
    }
    private static BigInteger Floor(BigInteger n, BigInteger d)
    { BigInteger q = BigInteger.DivRem(n,d,out var r); return r.Sign < 0 ? q - 1 : q; }
    private static BigInteger Ceiling(BigInteger n, BigInteger d) => -Floor(-n,d);
    private static bool InRange(Point64 p) => p.X > -(1L<<30) && p.X < 1L<<30 && p.Y > -(1L<<30) && p.Y < 1L<<30;
    private static Int128 Cross(Point64 a, Point64 b, Point64 p) => (Int128)(b.X-a.X)*(p.Y-a.Y)-(Int128)(b.Y-a.Y)*(p.X-a.X);
    private static bool InsideConvex(Path64 hull, Q p)
    {
        for (int i=0;i<hull.Count;i++)
        {
            Point64 a=hull[i], b=hull[(i+1)%hull.Count];
            if (((BigInteger)b.X-a.X)*(p.Y-a.Y*p.D)-((BigInteger)b.Y-a.Y)*(p.X-a.X*p.D) < 0) return false;
        }
        return true;
    }
    private static bool Normalize(Path64 input, out Path64 hull)
    {
        hull = new(input);
        if (hull.Count > 1 && hull[0] == hull[^1]) hull.RemoveAt(hull.Count-1);
        if (hull.Count is <3 or >32 || hull.Any(p=>!InRange(p)) || hull.Distinct().Count()!=hull.Count) return false;
        Int128 area=0; for(int i=0;i<hull.Count;i++) area+=(Int128)hull[i].X*hull[(i+1)%hull.Count].Y-(Int128)hull[i].Y*hull[(i+1)%hull.Count].X;
        if(area==0)return false; if(area<0)hull.Reverse();
        var copied=hull; return copied.All(p=>InsideConvex(copied,Q.Make(p.X,p.Y,1)));
    }
    private static Path64 ConvexHull(Path64 input)
    {
        var sorted=input.Distinct().OrderBy(p=>p.X).ThenBy(p=>p.Y).ToArray();
        if(sorted.Length<3)return new(sorted);
        var lower=new Path64(); var upper=new Path64();
        foreach(var p in sorted){while(lower.Count>1&&Cross(lower[^2],lower[^1],p)<=0)lower.RemoveAt(lower.Count-1);lower.Add(p);}
        for(int i=sorted.Length-1;i>=0;i--){var p=sorted[i];while(upper.Count>1&&Cross(upper[^2],upper[^1],p)<=0)upper.RemoveAt(upper.Count-1);upper.Add(p);}
        lower.RemoveAt(lower.Count-1);upper.RemoveAt(upper.Count-1);lower.AddRange(upper);return lower;
    }

}
