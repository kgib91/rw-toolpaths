using System.Numerics;
using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>
/// Proves the rectangular part of a capsule inside a union of earlier capsules.
/// The caller separately proves its two endpoint discs from actual posted poses.
/// Every accepted triangle lies in ONE earlier convex finite-segment capsule;
/// dyadic subdivision covers the complete rectangle, including its boundary.
/// </summary>
internal static class PocketExactCapsuleCoverage
{
    internal readonly record struct Segment(long AX, long AY, long BX, long BY);
    internal readonly record struct Result(bool Proven, int Triangles, int DistanceChecks,
        int Refinements, bool BudgetExhausted, int NegativeCandidates = 0,
        int NegativeProbes = 0, int NegativeWitnesses = 0, int NegativeDistanceChecks = 0,
        int NegativeBudgetRefusals = 0, double NegativeMilliseconds = 0,
        int NegativePreparedVertices = 0, int NegativeAxisProofs = 0, int NegativeFullDistanceChecks = 0);
    private readonly record struct Vertex(Number X, Number Y);
    private readonly record struct PreparedVertex(BigInteger XA, BigInteger XB, BigInteger YA,
        BigInteger YB, BigInteger Denominator, BigInteger RadiusNumeratorSquaredDenominatorSquared);
    private readonly record struct Triangle(Vertex A, Vertex B, Vertex C, int Depth);

    // One shared quadratic field Q(sqrt(D)). Denominators are positive.
    // Query normals are (-dy,dx)*sqrt(D)/D, hence exact binary64 radius,
    // F3 coordinates and every dyadic subdivision stay in this field.
    private readonly record struct Number(BigInteger A, BigInteger B, BigInteger Denominator);
    private sealed class Field
    {
        internal readonly BigInteger D;
        private readonly double _root;
        internal Field(BigInteger d) { D = d; _root = Math.Sqrt((double)d); }
        internal Number Make(BigInteger a, BigInteger b, BigInteger denominator)
        {
            if (denominator.Sign <= 0) throw new ArgumentOutOfRangeException(nameof(denominator));
            if (a.IsZero && b.IsZero) return new(0, 0, 1);
            var gcd = BigInteger.GreatestCommonDivisor(BigInteger.Abs(a), BigInteger.Abs(b));
            gcd = BigInteger.GreatestCommonDivisor(gcd, denominator);
            return new(a / gcd, b / gcd, denominator / gcd);
        }
        internal Number Integer(BigInteger value) => new(value, 0, 1);
        internal Number Add(Number a, Number b) => Make(
            a.A * b.Denominator + b.A * a.Denominator,
            a.B * b.Denominator + b.B * a.Denominator, a.Denominator * b.Denominator);
        internal Number Subtract(Number a, Number b) => Make(
            a.A * b.Denominator - b.A * a.Denominator,
            a.B * b.Denominator - b.B * a.Denominator, a.Denominator * b.Denominator);
        internal Number Scale(Number a, BigInteger value) => Make(a.A * value, a.B * value, a.Denominator);
        internal Number Divide(Number a, BigInteger value) => Make(a.A, a.B, a.Denominator * value);
        internal Number Square(Number a) => Make(a.A * a.A + a.B * a.B * D,
            2 * a.A * a.B, a.Denominator * a.Denominator);
        internal int Sign(Number number)
        {
            if (number.A.IsZero) return number.B.Sign;
            if (number.B.IsZero || number.A.Sign == number.B.Sign) return number.A.Sign;
            int comparison = (number.A * number.A).CompareTo(number.B * number.B * D);
            return number.A.Sign > 0 ? comparison : -comparison;
        }
        internal double Approximate(Number a) => (double)a.A / (double)a.Denominator
            + (double)a.B / (double)a.Denominator * _root;
        internal Vertex Midpoint(Vertex a, Vertex b) => new(Divide(Add(a.X, b.X), 2), Divide(Add(a.Y, b.Y), 2));
    }

    internal static Result Prove(Segment query, IReadOnlyList<Segment> earlier, double radius,
        CancellationToken token, int maximumTriangles = 512, int maximumDistanceChecks = 4096,
        int maximumNegativeProbes = 8, int maximumNegativeDistanceChecks = 2048,
        bool usePreparedNegativeArithmetic = true)
    {
        token.ThrowIfCancellationRequested();
        if (!double.IsFinite(radius) || radius <= 0 || earlier.Count == 0
            || maximumTriangles < 2 || maximumDistanceChecks < 3) return new(false, 0, 0, 0, false);
        BigInteger dx = (BigInteger)query.BX - query.AX, dy = (BigInteger)query.BY - query.AY;
        BigInteger squaredLength = dx * dx + dy * dy;
        if (squaredLength.IsZero) return new(true, 0, 0, 0, false);
        var field = new Field(squaredLength);
        // Decode the declared physical radius BEFORE multiplication by1000;
        // performing that multiplication in binary64 would change the proof.
        long bits = BitConverter.DoubleToInt64Bits(radius);
        int exponent = (int)((bits >> 52) & 0x7ff);
        BigInteger numerator = bits & 0x000f_ffff_ffff_ffffL;
        int shift;
        if (exponent == 0) shift = -1074;
        else { numerator += BigInteger.One << 52; shift = exponent - 1023 - 52; }
        numerator *= 1000;
        BigInteger denominator = BigInteger.One;
        if (shift >= 0) numerator <<= shift; else denominator <<= -shift;
        var gcd = BigInteger.GreatestCommonDivisor(numerator, denominator);
        numerator /= gcd; denominator /= gcd;
        var radiusSquared = field.Make(numerator * numerator, 0, denominator * denominator);
        BigInteger normalDenominator = denominator * squaredLength;
        var normalX = field.Make(0, -dy * numerator, normalDenominator);
        var normalY = field.Make(0, dx * numerator, normalDenominator);
        Vertex Corner(long x, long y, bool positive) => new(
            positive ? field.Add(field.Integer(x), normalX) : field.Subtract(field.Integer(x), normalX),
            positive ? field.Add(field.Integer(y), normalY) : field.Subtract(field.Integer(y), normalY));
        var a = Corner(query.AX, query.AY, true); var b = Corner(query.BX, query.BY, true);
        var c = Corner(query.BX, query.BY, false); var d = Corner(query.AX, query.AY, false);
        var stack = new Stack<Triangle>();
        stack.Push(new(a, b, c, 0)); stack.Push(new(a, c, d, 0));
        int triangles = 0, checks = 0, refinements = 0;
        int negativeCandidates = 0, negativeProbes = 0, negativeWitnesses = 0;
        int negativeChecks = 0, negativeBudgetRefusals = 0;
        int negativePreparedVertices = 0, negativeAxisProofs = 0, negativeFullDistanceChecks = 0;
        double negativeMilliseconds = 0;
        bool initialNegativeCandidatesTried = false, negativeBudgetRefused = false;
        var negativeCandidatesTried = new HashSet<Vertex>();
        double radiusGrid = radius * 1000;
        while (stack.Count > 0)
        {
            token.ThrowIfCancellationRequested();
            if (++triangles > maximumTriangles) return Complete(false, true);
            var triangle = stack.Pop();
            bool covered = false;
            foreach (var segment in earlier)
            {
                // Approximation ONLY rejects this particular witness choice.
                // Every positive containment is rechecked by exact field signs.
                if (!Plausible(triangle.A, segment) || !Plausible(triangle.B, segment)
                    || !Plausible(triangle.C, segment)) continue;
                if (checks + 3 > maximumDistanceChecks) return Complete(false, true);
                if (!Inside(triangle.A, segment) || !Inside(triangle.B, segment) || !Inside(triangle.C, segment)) continue;
                covered = true;
                break;
            }
            if (covered) continue;
            // A negative witness only rejects this bounded SUBSET certificate.
            // It never rejects the complete prior-stock union or supplies any
            // stock authority: the caller still runs its unchanged INNER proof.
            // These center, side and dyadic points belong to the whole query
            // rectangle. Doubles merely nominate; an outside certificate checks
            // EVERY selected finite capsule with the exact quadratic predicate.
            if (!initialNegativeCandidatesTried)
            {
                initialNegativeCandidatesTried = true;
                if (OutsideSelectedCapsules(field.Midpoint(a, c))
                    || OutsideSelectedCapsules(field.Midpoint(a, b))
                    || OutsideSelectedCapsules(field.Midpoint(c, d))) return Complete(false, false);
            }
            if (OutsideSelectedCapsules(triangle.A) || OutsideSelectedCapsules(triangle.B)
                || OutsideSelectedCapsules(triangle.C)) return Complete(false, false);
            if (triangle.Depth >= 14) return Complete(false, true);
            refinements++;
            double ab = LengthSquared(triangle.A, triangle.B), bc = LengthSquared(triangle.B, triangle.C);
            double ca = LengthSquared(triangle.C, triangle.A);
            int nextDepth = triangle.Depth + 1;
            if (ab >= bc && ab >= ca)
            {
                var middle = field.Midpoint(triangle.A, triangle.B);
                if (OutsideSelectedCapsules(middle)) return Complete(false, false);
                stack.Push(new(triangle.A, middle, triangle.C, nextDepth));
                stack.Push(new(middle, triangle.B, triangle.C, nextDepth));
            }
            else if (bc >= ca)
            {
                var middle = field.Midpoint(triangle.B, triangle.C);
                if (OutsideSelectedCapsules(middle)) return Complete(false, false);
                stack.Push(new(triangle.A, triangle.B, middle, nextDepth));
                stack.Push(new(triangle.A, middle, triangle.C, nextDepth));
            }
            else
            {
                var middle = field.Midpoint(triangle.C, triangle.A);
                if (OutsideSelectedCapsules(middle)) return Complete(false, false);
                stack.Push(new(triangle.A, triangle.B, middle, nextDepth));
                stack.Push(new(middle, triangle.B, triangle.C, nextDepth));
            }
        }
        return Complete(true, false);

        Result Complete(bool proven, bool budgetExhausted) => new(proven, triangles, checks, refinements,
            budgetExhausted, negativeCandidates, negativeProbes, negativeWitnesses, negativeChecks,
            negativeBudgetRefusals, negativeMilliseconds, negativePreparedVertices,
            negativeAxisProofs, negativeFullDistanceChecks);

        bool OutsideSelectedCapsules(Vertex vertex)
        {
            token.ThrowIfCancellationRequested();
            // The candidate cap also bounds failed double nominations. An
            // exhausted negative budget leaves the original subdivision intact.
            if (maximumNegativeProbes <= 0 || maximumNegativeDistanceChecks <= 0
                || negativeCandidates >= 32 || negativeBudgetRefused
                || !negativeCandidatesTried.Add(vertex)) return false;
            long started = Stopwatch.GetTimestamp();
            try
            {
                negativeCandidates++;
                double x = field.Approximate(vertex.X), y = field.Approximate(vertex.Y);
                if (!double.IsFinite(x) || !double.IsFinite(y)) return false;
                int nominationChecks = 0;
                foreach (var segment in earlier)
                {
                    if ((nominationChecks++ & 31) == 0) token.ThrowIfCancellationRequested();
                    if (PlausiblePoint(x, y, segment)) return false;
                }
                if (negativeProbes >= maximumNegativeProbes
                    || (long)negativeChecks + earlier.Count > maximumNegativeDistanceChecks)
                {
                    negativeBudgetRefused = true;
                    negativeBudgetRefusals++;
                    return false;
                }
                negativeProbes++;
                var prepared = usePreparedNegativeArithmetic ? PrepareNegativeVertex(vertex) : default;
                foreach (var segment in earlier)
                {
                    if ((negativeChecks & 31) == 0) token.ThrowIfCancellationRequested();
                    negativeChecks++;
                    if (usePreparedNegativeArithmetic)
                    {
                        if (InsidePreparedNegative(prepared, segment)) return false;
                    }
                    else
                    {
                        negativeFullDistanceChecks++;
                        if (InsideExact(vertex, segment)) return false;
                    }
                }
                // All exact distance comparisons were STRICTLY outside. A
                // boundary equality counts as inside, including finite endcaps.
                negativeWitnesses++;
                return true;
            }
            finally { negativeMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds; }
        }

        PreparedVertex PrepareNegativeVertex(Vertex vertex)
        {
            // ONE common rational denominator per witness. Every selected old
            // capsule sees the same immutable quadratic XY and physical radius.
            BigInteger common = vertex.X.Denominator / BigInteger.GreatestCommonDivisor(
                vertex.X.Denominator, vertex.Y.Denominator) * vertex.Y.Denominator;
            BigInteger xScale = common / vertex.X.Denominator, yScale = common / vertex.Y.Denominator;
            negativePreparedVertices++;
            return new(vertex.X.A * xScale, vertex.X.B * xScale, vertex.Y.A * yScale,
                vertex.Y.B * yScale, common, numerator * numerator * common * common);
        }
        int SignRaw(BigInteger a, BigInteger b)
        {
            // Reduction/GCD cannot affect a sign. The positive common
            // denominator is omitted from every comparison below.
            if (a.IsZero) return b.Sign;
            if (b.IsZero || a.Sign == b.Sign) return a.Sign;
            int comparison = (a * a).CompareTo(b * b * field.D);
            return a.Sign > 0 ? comparison : -comparison;
        }
        bool InsidePreparedNegative(PreparedVertex vertex, Segment segment)
        {
            // A finite capsule is contained in its centerline AABB enlarged by
            // EXACT R. A strict exact sign outside one axis excludes that
            // capsule; equality is unresolved and keeps the full predicate.
            // This is only the selected negative scan, never stock authority.
            BigInteger common = vertex.Denominator;
            BigInteger radiusCommon = numerator * common;
            if (SignRaw((vertex.XA - Math.Max(segment.AX, segment.BX) * common) * denominator - radiusCommon,
                    vertex.XB * denominator) > 0
                || SignRaw((Math.Min(segment.AX, segment.BX) * common - vertex.XA) * denominator - radiusCommon,
                    -vertex.XB * denominator) > 0
                || SignRaw((vertex.YA - Math.Max(segment.AY, segment.BY) * common) * denominator - radiusCommon,
                    vertex.YB * denominator) > 0
                || SignRaw((Math.Min(segment.AY, segment.BY) * common - vertex.YA) * denominator - radiusCommon,
                    -vertex.YB * denominator) > 0)
            { negativeAxisProofs++; return false; }
            negativeFullDistanceChecks++;
            // With XY=(A+B*sqrt(D))/common, projection and squared finite
            // distance use the same signs as InsideExact. Multiply only by
            // positive denominators; avoid per-operation GCD normalization.
            BigInteger xa = vertex.XA - segment.AX * common, ya = vertex.YA - segment.AY * common;
            BigInteger xb = vertex.XB, yb = vertex.YB;
            BigInteger sx = (BigInteger)segment.BX - segment.AX, sy = (BigInteger)segment.BY - segment.AY;
            BigInteger length = sx * sx + sy * sy;
            BigInteger dotA = xa * sx + ya * sy, dotB = xb * sx + yb * sy;
            if (!length.IsZero && SignRaw(dotA, dotB) > 0
                && SignRaw(dotA - length * common, dotB) < 0)
            {
                BigInteger crossA = ya * sx - xa * sy, crossB = yb * sx - xb * sy;
                return SignRaw((crossA * crossA + crossB * crossB * field.D) * denominator * denominator
                        - vertex.RadiusNumeratorSquaredDenominatorSquared * length,
                    2 * crossA * crossB * denominator * denominator) <= 0;
            }
            if (!length.IsZero && SignRaw(dotA, dotB) > 0)
            {
                xa = vertex.XA - segment.BX * common; ya = vertex.YA - segment.BY * common;
            }
            return SignRaw((xa * xa + xb * xb * field.D + ya * ya + yb * yb * field.D)
                    * denominator * denominator - vertex.RadiusNumeratorSquaredDenominatorSquared,
                2 * (xa * xb + ya * yb) * denominator * denominator) <= 0;
        }

        bool Inside(Vertex vertex, Segment segment)
        {
            checks++;
            return InsideExact(vertex, segment);
        }
        bool InsideExact(Vertex vertex, Segment segment)
        {
            var x = field.Subtract(vertex.X, field.Integer(segment.AX));
            var y = field.Subtract(vertex.Y, field.Integer(segment.AY));
            BigInteger sx = (BigInteger)segment.BX - segment.AX, sy = (BigInteger)segment.BY - segment.AY;
            BigInteger length = sx * sx + sy * sy;
            var dot = field.Add(field.Scale(x, sx), field.Scale(y, sy));
            if (!length.IsZero && field.Sign(dot) > 0
                && field.Sign(field.Subtract(dot, field.Integer(length))) < 0)
            {
                var cross = field.Subtract(field.Scale(y, sx), field.Scale(x, sy));
                return field.Sign(field.Subtract(field.Square(cross), field.Scale(radiusSquared, length))) <= 0;
            }
            if (!length.IsZero && field.Sign(dot) > 0)
            {
                x = field.Subtract(vertex.X, field.Integer(segment.BX));
                y = field.Subtract(vertex.Y, field.Integer(segment.BY));
            }
            return field.Sign(field.Subtract(field.Add(field.Square(x), field.Square(y)), radiusSquared)) <= 0;
        }
        bool Plausible(Vertex vertex, Segment segment)
        {
            double x = field.Approximate(vertex.X), y = field.Approximate(vertex.Y);
            if (!double.IsFinite(x) || !double.IsFinite(y)) return false;
            return PlausiblePoint(x, y, segment);
        }
        bool PlausiblePoint(double x, double y, Segment segment)
        {
            double sx = (double)segment.BX - segment.AX, sy = (double)segment.BY - segment.AY;
            double length = sx * sx + sy * sy;
            double t = length == 0 ? 0 : Math.Clamp(((x - segment.AX) * sx + (y - segment.AY) * sy) / length, 0, 1);
            double ex = x - segment.AX - t * sx, ey = y - segment.AY - t * sy;
            // This slack merely keeps near-equality choices for exact checking.
            return ex * ex + ey * ey <= Math.Pow(radiusGrid + Math.Max(1e-5, radiusGrid * 1e-8), 2);
        }
        double LengthSquared(Vertex first, Vertex second)
        {
            double x = field.Approximate(field.Subtract(first.X, second.X));
            double y = field.Approximate(field.Subtract(first.Y, second.Y));
            return x * x + y * y;
        }
    }
}
