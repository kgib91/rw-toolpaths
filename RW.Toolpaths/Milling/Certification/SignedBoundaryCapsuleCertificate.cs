using Clipper2Lib;
using System.Diagnostics;
using System.Numerics;

namespace RW.Toolpaths.Milling.Certification;

/// <summary>
/// Proves the complete NonZero support of an arbitrary signed boundary is
/// contained in ORIGINAL finite-segment capsules. Triangulation is an untrusted
/// choice aid: exact signed edge-chain equality proves the winding identity,
/// including negative/exterior slivers and repaired self-crossings. Every main
/// and correction triangle must independently fit one strict original capsule.
/// </summary>
internal static class SignedBoundaryCapsuleCertificate
{
    // Optional nomination policy. Absent policy preserves the established
    // all-support certificate and its integer-choice traversal. Every retained
    // enclosure is the entire unresolved exact triangle, rounded outward.
    internal sealed class PartitionPolicy
    {
        internal int MaximumDepth { get; init; } = 20;
        internal int SmallHullSizeTicks { get; init; } = 64;
        internal int MaximumHulls { get; init; } = 4096;
        internal Paths64 Enclosures { get; } = new();
    }

    internal sealed class Diagnostics
    {
        internal bool Certified;
        internal bool UntrustedIntersectionRepairSkipped;
        internal string Reason = "not-started";
        internal int InputPoints, MainTriangles, CorrectionTriangles, ResidualEdges, FinalResidualEdges;
        internal bool SupportOutsideEnclosuresCertified, OriginalOwnerRegistryEnforced;
        internal int UnresolvedPieces;
        internal int TriangleTasks, CertifiedPieces, RefinedPieces, BigIntegerFallbacks;
        internal int OriginalHintEdges, OriginalSiteRegistryEdges, OriginalEdgeHintTrials, OriginalEdgeHintCertificates,
            InheritedSiteTrials, InheritedSiteCertificates, OriginalSiteChoices, MaximumPendingTasks;
        internal double TriangulationMilliseconds, ChainMilliseconds, CapsuleMilliseconds;
        internal Point64? FailedWitness;
        internal double[][]? FailedTriangle;
    }

    private sealed class OriginalSiteChoiceException : Exception { }

    private readonly record struct EdgeKey(Point64 A, Point64 B);
    // Retained as an adapter for frozen reflected chain controls. Production
    // decomposition is owned solely by ExactSignedBoundaryDecomposition.
    private readonly record struct Triangle(Point64 A, Point64 B, Point64 C);
    private readonly record struct OriginalSite(Point64 Start, Point64 End);
    private readonly record struct DyadicPoint(Int128 X, Int128 Y)
    {
        internal DyadicPoint Twice() => new((X * 2), (Y * 2));
        internal Point64 ChoicePoint(int shift) => new((long)(X / ((Int128)1 << shift)), (long)(Y / ((Int128)1 << shift)));
    }
    private readonly record struct Task(DyadicPoint A, DyadicPoint B, DyadicPoint C, int Shift, int Depth = 0,
        OriginalSite? PreferredSite = null, bool InheritedSite = false);

    internal static bool TryCertify(Paths64 signedBoundary,
        Func<Point64, (Point64 Start, Point64 End)> originalSiteChoice,
        long strictRadius, CancellationToken token, out Diagnostics diagnostics,
        int maximumTriangleTasks = 2_000_000, bool useDelaunay = false, Action? workCheckpoint = null,
        Paths64? originalBoundary = null, bool repairIntersections = false, int radiusFractionBits = 0,
        PartitionPolicy? partition = null,
        Func<double, double, (Point64 Start, Point64 End)>? fractionalOriginalSiteChoice = null,
        bool useOriginalEdgeHints = true, Paths64? triangulationAidBoundary = null)
    {
        token.ThrowIfCancellationRequested();
        workCheckpoint?.Invoke();
        var result = new Diagnostics { InputPoints = signedBoundary.Sum(path => path.Count) };
        diagnostics = result;
        if (partition is not null && (partition.Enclosures.Count != 0
                || partition.MaximumDepth is < 0 or > 48 || partition.SmallHullSizeTicks is < 0 or > 32768
                || partition.MaximumHulls is < 1 or > 4096))
        { result.Reason = "original-registry-or-partition-bound"; return false; }
        if (strictRadius <= 0 || radiusFractionBits is < 0 or > 24
            || result.InputPoints == 0 || result.InputPoints > 500_000
            || signedBoundary.SelectMany(path => path).Any(point => Math.Abs((double)point.X) >= (1L << 30)
                || Math.Abs((double)point.Y) >= (1L << 30)))
        { result.Reason = "input-or-integer-bound"; return false; }
        if (originalBoundary is null)
        { result.Reason = "missing-original-site-registry"; return false; }
        try
        {
            bool decomposed = ExactSignedBoundaryDecomposition.TryCreateAfterInitialCheckpoint(
                signedBoundary, token, workCheckpoint, out var triangles, out var decomposition,
                useDelaunay, repairIntersections, triangulationAidBoundary);
            result.UntrustedIntersectionRepairSkipped = decomposition.UntrustedIntersectionRepairSkipped;
            result.MainTriangles = decomposition.MainTriangles;
            result.CorrectionTriangles = decomposition.CorrectionTriangles;
            result.ResidualEdges = decomposition.ResidualEdges;
            result.FinalResidualEdges = decomposition.FinalResidualEdges;
            result.TriangulationMilliseconds = decomposition.TriangulationMilliseconds;
            result.ChainMilliseconds = decomposition.ChainMilliseconds;
            if (!decomposed) { result.Reason = decomposition.Reason; return false; }
            var timer = Stopwatch.StartNew();
            result.Reason = "finite-capsule-support-in-progress";
            // Only exact edges of the SAME canonical original boundary are
            // choices. An untrusted triangulator edge never manufactures a
            // site; every selected finite capsule still proves all vertices.
            var hints = OriginalEdgeHints(originalBoundary, token, workCheckpoint);
            result.OriginalSiteRegistryEdges = hints.Count;
            result.OriginalHintEdges = useOriginalEdgeHints ? hints.Count : 0;
            result.OriginalOwnerRegistryEnforced = true;
            HashSet<Point64>? originalVertices = null;
            OriginalSite Registered((Point64 Start, Point64 End) selected)
            {
                // Every capsule choice requires the literal complete ORIGINAL
                // finite-owner registry, including a hintless reference call.
                bool registered = selected.Start == selected.End
                    ? (originalVertices ??= originalBoundary.SelectMany(path => path).ToHashSet()).Contains(selected.Start)
                    : hints.ContainsKey(Key(selected.Start, selected.End));
                if (!registered) throw new OriginalSiteChoiceException();
                return new(selected.Start, selected.End);
            }
            var pending = new Stack<Task>(49);
            // Preserve the previous reverse main-triangle traversal, while
            // retaining only the local depth-first subdivision stack.
            for (int triangleIndex = triangles.Length - 1; triangleIndex >= 0; triangleIndex--)
            {
                var triangle = triangles[triangleIndex];
                OriginalSite? preferred = null;
                if (useOriginalEdgeHints && (hints.TryGetValue(Key(triangle.A, triangle.B), out var edge)
                    || hints.TryGetValue(Key(triangle.B, triangle.C), out edge)
                    || hints.TryGetValue(Key(triangle.C, triangle.A), out edge))) preferred = edge;
                pending.Push(new(new(triangle.A.X, triangle.A.Y), new(triangle.B.X, triangle.B.Y),
                    new(triangle.C.X, triangle.C.Y), 0, PreferredSite: preferred));
                while (pending.TryPop(out var current))
                {
                    result.MaximumPendingTasks = Math.Max(result.MaximumPendingTasks, pending.Count + 1);
                    if ((++result.TriangleTasks & 255) == 0)
                    { token.ThrowIfCancellationRequested(); workCheckpoint?.Invoke(); }
                    if (result.TriangleTasks > maximumTriangleTasks)
                    { result.Reason = "capsule-work-budget"; result.CapsuleMilliseconds = timer.Elapsed.TotalMilliseconds; return false; }
                    Int128 scale = (Int128)1 << current.Shift;
                    var center = new Point64((long)((current.A.X + current.B.X + current.C.X) / (3 * scale)),
                        (long)((current.A.Y + current.B.Y + current.C.Y) / (3 * scale)));
                    bool Fits(OriginalSite site) => FitsTriangle(current, site, strictRadius, result, radiusFractionBits);
                    OriginalSite Choice(Point64 point)
                    {
                        result.OriginalSiteChoices++;
                        var selected = originalSiteChoice(point);
                        return Registered(selected);
                    }
                    OriginalSite FractionalChoice(double x, double y)
                    {
                        if (fractionalOriginalSiteChoice is null) return Choice(new Point64((long)x, (long)y));
                        result.OriginalSiteChoices++;
                        return Registered(fractionalOriginalSiteChoice(x, y));
                    }
                    OriginalSite CenterChoice() => fractionalOriginalSiteChoice is null ? Choice(center)
                        : FractionalChoice((double)(current.A.X + current.B.X + current.C.X) / (3 * (double)scale),
                            (double)(current.A.Y + current.B.Y + current.C.Y) / (3 * (double)scale));
                    OriginalSite VertexChoice(DyadicPoint point) => fractionalOriginalSiteChoice is null
                        ? Choice(point.ChoicePoint(current.Shift))
                        : FractionalChoice((double)point.X / (double)scale, (double)point.Y / (double)scale);
                    bool preferredFits = false;
                    if (current.PreferredSite is { } hint)
                    {
                        if (current.InheritedSite) result.InheritedSiteTrials++; else result.OriginalEdgeHintTrials++;
                        preferredFits = Fits(hint);
                        if (preferredFits)
                        {
                            if (current.InheritedSite) result.InheritedSiteCertificates++; else result.OriginalEdgeHintCertificates++;
                        }
                    }
                    // A failed choice can still be useful to a child. It grants
                    // no credit until all three exact child vertices fit it.
                    var site = current.PreferredSite ?? CenterChoice();
                    bool centerFits = false;
                    if (!preferredFits)
                    {
                        var centerSite = current.PreferredSite.HasValue ? CenterChoice() : site;
                        centerFits = (!current.PreferredSite.HasValue || centerSite != current.PreferredSite.Value) && Fits(centerSite);
                        if (fractionalOriginalSiteChoice is not null || !current.PreferredSite.HasValue) site = centerSite;
                    }
                    if (preferredFits || centerFits || Fits(VertexChoice(current.A))
                        || Fits(VertexChoice(current.B)) || Fits(VertexChoice(current.C)))
                    { result.CertifiedPieces++; continue; }
                    bool smallHull = partition is { SmallHullSizeTicks: > 0 }
                        && Int128.Max(current.A.X, Int128.Max(current.B.X, current.C.X)) - Int128.Min(current.A.X, Int128.Min(current.B.X, current.C.X)) <= partition.SmallHullSizeTicks * scale
                        && Int128.Max(current.A.Y, Int128.Max(current.B.Y, current.C.Y)) - Int128.Min(current.A.Y, Int128.Min(current.B.Y, current.C.Y)) <= partition.SmallHullSizeTicks * scale;
                    if (partition is not null && (current.Depth >= partition.MaximumDepth || smallHull))
                    {
                        if (partition.Enclosures.Count >= partition.MaximumHulls)
                        { result.Reason = "unresolved-enclosure-budget"; result.CapsuleMilliseconds = timer.Elapsed.TotalMilliseconds; return false; }
                        token.ThrowIfCancellationRequested(); workCheckpoint?.Invoke();
                        long BoundedLong(Int128 value)
                        {
                            // C# 10 cannot select Int128's checked conversion.
                            // Check the exact value before the ordinary cast.
                            if (value < long.MinValue || value > long.MaxValue) throw new OverflowException();
                            return (long)value;
                        }
                        long Floor(Int128 numerator)
                        {
                            Int128 q = numerator / scale, r = numerator % scale;
                            // Convex dyadic subdivision retains the bounded
                            // original coordinate range; this adjustment cannot
                            // approach Int128 limits. The cast is checked above.
                            return BoundedLong(q - (r < 0 ? 1 : 0));
                        }
                        long Ceiling(Int128 numerator)
                        {
                            Int128 q = numerator / scale, r = numerator % scale;
                            return BoundedLong(q + (r > 0 ? 1 : 0));
                        }
                        var points = new List<Point64>(12);
                        foreach (var vertex in new[] { current.A, current.B, current.C })
                        {
                            long left = Floor(vertex.X), right = Ceiling(vertex.X), top = Floor(vertex.Y), bottom = Ceiling(vertex.Y);
                            points.Add(new(left - 1, top - 1)); points.Add(new(right + 1, top - 1));
                            points.Add(new(right + 1, bottom + 1)); points.Add(new(left - 1, bottom + 1));
                        }
                        var ordered = points.Distinct().OrderBy(p => p.X).ThenBy(p => p.Y).ToArray();
                        static Int128 Cross(Point64 a, Point64 b, Point64 c)
                            => (Int128)(b.X-a.X)*(c.Y-a.Y)-(Int128)(b.Y-a.Y)*(c.X-a.X);
                        var hull = new Path64();
                        foreach (var point in ordered)
                        { while (hull.Count >= 2 && Cross(hull[^2], hull[^1], point) <= 0) hull.RemoveAt(hull.Count-1); hull.Add(point); }
                        int lowerCount = hull.Count;
                        for (int i=ordered.Length-2;i>=0;i--)
                        { while(hull.Count>lowerCount&&Cross(hull[^2],hull[^1],ordered[i])<=0)hull.RemoveAt(hull.Count-1);hull.Add(ordered[i]); }
                        hull.RemoveAt(hull.Count-1);
                        partition.Enclosures.Add(hull); result.UnresolvedPieces++;
                        continue;
                    }
                    if (current.Depth == 48)
                    {
                        result.Reason = "capsule-subdivision-unresolved";
                        result.FailedWitness = center;
                        result.FailedTriangle = new[] { current.A, current.B, current.C }
                            .Select(point => new[] { (double)point.X / (double)scale, (double)point.Y / (double)scale }).ToArray();
                        result.CapsuleMilliseconds = timer.Elapsed.TotalMilliseconds;
                        return false;
                    }
                    // Bisect the longest edge, so a very thin monotone triangle
                    // does not reproduce four thin triangles on every iteration.
                    // Floating length selects a split ONLY; exact dyadic vertices
                    // and full capsule inequalities remain the authority.
                    double Length(DyadicPoint a, DyadicPoint b)
                    { double x = (double)(a.X - b.X), y = (double)(a.Y - b.Y); return x * x + y * y; }
                    double abLength = Length(current.A, current.B), bcLength = Length(current.B, current.C), caLength = Length(current.C, current.A);
                    var a2 = current.A.Twice(); var b2 = current.B.Twice(); var c2 = current.C.Twice();
                    int shift = current.Shift + 1, depth = current.Depth + 1;
                    if (abLength >= bcLength && abLength >= caLength)
                    {
                        var middle = new DyadicPoint(current.A.X + current.B.X, current.A.Y + current.B.Y);
                        pending.Push(Normalize(new(a2, middle, c2, shift, depth, site, true)));
                        pending.Push(Normalize(new(middle, b2, c2, shift, depth, site, true)));
                    }
                    else if (bcLength >= caLength)
                    {
                        var middle = new DyadicPoint(current.B.X + current.C.X, current.B.Y + current.C.Y);
                        pending.Push(Normalize(new(a2, b2, middle, shift, depth, site, true)));
                        pending.Push(Normalize(new(a2, middle, c2, shift, depth, site, true)));
                    }
                    else
                    {
                        var middle = new DyadicPoint(current.C.X + current.A.X, current.C.Y + current.A.Y);
                        pending.Push(Normalize(new(a2, b2, middle, shift, depth, site, true)));
                        pending.Push(Normalize(new(middle, b2, c2, shift, depth, site, true)));
                    }
                    result.RefinedPieces++;
                }
            }
            result.CapsuleMilliseconds = timer.Elapsed.TotalMilliseconds;
            if (partition is not null && partition.Enclosures.Count != 0)
            {
                result.SupportOutsideEnclosuresCertified = true;
                result.Reason = "complete-signed-support-in-original-capsules-or-recorded-enclosures";
                return false;
            }
            result.Certified = true;
            result.Reason = "exact-signed-chain-and-complete-finite-capsule-support";
            return true;
        }
        catch (OriginalSiteChoiceException)
        { result.Reason = "unregistered-original-site-choice"; return false; }
        catch (TriangulationBudgetException)
        { result.Reason = "untrusted-triangulator-work-budget"; return false; }
        catch (InsufficientExecutionStackException)
        { result.Reason = "untrusted-triangulator-stack-budget"; return false; }
        catch (OverflowException)
        { result.Reason = "bounded-chain-arithmetic-overflow"; return false; }
        catch (InvalidOperationException)
        { result.Reason = "untrusted-decomposition-or-chain-invariant"; return false; }
    }

    internal static bool IsBoundedProposedTriangulation(Paths64 proposed)
        => ExactSignedBoundaryDecomposition.IsBoundedProposedTriangulation(proposed);

    private static Task Normalize(Task triangle)
    {
        while (triangle.Shift > 0 && ((triangle.A.X | triangle.A.Y | triangle.B.X | triangle.B.Y | triangle.C.X | triangle.C.Y) & 1) == 0)
            triangle = new(new(triangle.A.X >> 1, triangle.A.Y >> 1), new(triangle.B.X >> 1, triangle.B.Y >> 1),
                new(triangle.C.X >> 1, triangle.C.Y >> 1), triangle.Shift - 1, triangle.Depth,
                triangle.PreferredSite, triangle.InheritedSite);
        return triangle;
    }

    private static EdgeKey Key(Point64 a, Point64 b)
        => a.X < b.X || (a.X == b.X && a.Y < b.Y) ? new(a, b) : new(b, a);

    private static Dictionary<EdgeKey, OriginalSite> OriginalEdgeHints(Paths64? originalBoundary,
        CancellationToken token, Action? checkpoint)
    {
        var hints = new Dictionary<EdgeKey, OriginalSite>();
        if (originalBoundary is null) return hints;
        int points = 0;
        foreach (var path in originalBoundary)
        for (int i = 0; i < path.Count; i++)
        {
            if ((++points & 1023) == 0) { token.ThrowIfCancellationRequested(); checkpoint?.Invoke(); }
            Point64 a = path[i], b = path[(i + 1) % path.Count];
            if (points > 500_000 || Math.Abs((double)a.X) >= 1L << 30 || Math.Abs((double)a.Y) >= 1L << 30)
                throw new InvalidOperationException("Original site choices exceed the exact arithmetic range.");
            if (a != b) hints.TryAdd(Key(a, b), new(a, b));
        }
        return hints;
    }

    private static bool FitsTriangle(Task triangle, OriginalSite site, long radius, Diagnostics statistics, int radiusFractionBits)
    {
        try
        {
            // The finite-site projection, scaled endpoints and squared
            // radius are identical for all three vertices. Compute them once.
            var capsule = new CapsuleParameters(site, radius, triangle.Shift, radiusFractionBits);
            return capsule.Contains(triangle.A) && capsule.Contains(triangle.B) && capsule.Contains(triangle.C);
        }
        catch (OverflowException)
        {
            var capsule = new BigCapsuleParameters(site, radius, triangle.Shift, radiusFractionBits);
            bool Contains(DyadicPoint point) { statistics.BigIntegerFallbacks++; return capsule.Contains(point); }
            return Contains(triangle.A) && Contains(triangle.B) && Contains(triangle.C);
        }
    }

    /// <summary>
    /// Exact positive containment of all three dyadic vertices in ONE finite
    /// original-edge capsule. The caller supplies an edge from its immutable
    /// original registry and a strictly reserved radius numerator; this helper
    /// does not nominate sites or classify target winding. A capsule is convex,
    /// so its three exact vertex inequalities contain the complete triangle.
    /// </summary>
    internal static bool ContainsDyadicTriangle(Point64 edgeA, Point64 edgeB,
        Int128 ax, Int128 ay, Int128 bx, Int128 by, Int128 cx, Int128 cy,
        int shift, long radiusNumerator, int radiusFractionBits = 24)
        => ContainsDyadicTriangle(edgeA, edgeB, ax, ay, bx, by, cx, cy, shift,
            radiusNumerator, out _, radiusFractionBits);

    internal static bool ContainsDyadicTriangle(Point64 edgeA, Point64 edgeB,
        Int128 ax, Int128 ay, Int128 bx, Int128 by, Int128 cx, Int128 cy,
        int shift, long radiusNumerator, out int bigIntegerFallbacks, int radiusFractionBits = 24)
    {
        bigIntegerFallbacks = 0;
        const long bound = ExactSignedBoundaryDecomposition.CoordinateLimit;
        if (shift is < 0 or > 48 || radiusFractionBits is < 0 or > 24 || radiusNumerator <= 0
            || edgeA.X <= -bound || edgeA.X >= bound || edgeA.Y <= -bound || edgeA.Y >= bound
            || edgeB.X <= -bound || edgeB.X >= bound || edgeB.Y <= -bound || edgeB.Y >= bound) return false;
        Int128 scaledBound = (Int128)bound << shift;
        bool Bounded(Int128 value) => value > -scaledBound && value < scaledBound;
        if (!Bounded(ax) || !Bounded(ay) || !Bounded(bx) || !Bounded(by) || !Bounded(cx) || !Bounded(cy)) return false;
        var a = new DyadicPoint(ax, ay); var b = new DyadicPoint(bx, by); var c = new DyadicPoint(cx, cy);
        try
        {
            var capsule = new CapsuleParameters(new(edgeA, edgeB), radiusNumerator, shift, radiusFractionBits);
            return capsule.Contains(a) && capsule.Contains(b) && capsule.Contains(c);
        }
        catch (OverflowException)
        {
            var capsule = new BigCapsuleParameters(new(edgeA, edgeB), radiusNumerator, shift, radiusFractionBits);
            int fallbacks = 0;
            bool Contains(DyadicPoint point) { fallbacks++; return capsule.Contains(point); }
            bool contained = Contains(a) && Contains(b) && Contains(c);
            bigIntegerFallbacks = fallbacks;
            return contained;
        }
    }

    /// <summary>Exact dyadic point containment in a finite ORIGINAL capsule.</summary>
    private static bool Capsule(DyadicPoint point, (Point64 Start, Point64 End) site,
        long radius, int shift, Diagnostics statistics)
        => CapsuleWithDyadicRadius(point, site, radius, shift, statistics, 0);

    // Distinct name retains the established integer-radius reflection hook.
    private static bool CapsuleWithDyadicRadius(DyadicPoint point, (Point64 Start, Point64 End) site,
        long radius, int shift, Diagnostics statistics, int radiusFractionBits)
    {
        try
        {
            return new CapsuleParameters(new(site.Start, site.End), radius, shift, radiusFractionBits).Contains(point);
        }
        catch (OverflowException)
        {
            statistics.BigIntegerFallbacks++;
            return new BigCapsuleParameters(new(site.Start, site.End), radius, shift, radiusFractionBits).Contains(point);
        }
    }

    private readonly struct CapsuleParameters
    {
        private readonly Int128 dx, dy, length, maximumProjection, radiusSquared, interiorLimit;
        private readonly Int128 startX, startY, endX, endY;
        private readonly Int128 denominatorSquared;
        internal CapsuleParameters(OriginalSite site, long radius, int shift, int radiusFractionBits)
        {
            Int128 scale = (Int128)1 << shift;
            dx = (Int128)site.End.X - site.Start.X; dy = (Int128)site.End.Y - site.Start.Y;
            length = CheckedInt128.Add(CheckedInt128.Multiply(dx, dx), CheckedInt128.Multiply(dy, dy));
            maximumProjection = CheckedInt128.Multiply(length, scale);
            Int128 radial = CheckedInt128.Multiply(radius, scale);
            radiusSquared = CheckedInt128.Multiply(radial, radial);
            denominatorSquared = (Int128)1 << (2 * radiusFractionBits);
            interiorLimit = MultiplyNonnegative(radiusSquared, length);
            startX = CheckedInt128.Multiply(site.Start.X, scale); startY = CheckedInt128.Multiply(site.Start.Y, scale);
            endX = CheckedInt128.Multiply(site.End.X, scale); endY = CheckedInt128.Multiply(site.End.Y, scale);
        }
        internal bool Contains(DyadicPoint point)
        {
            Int128 wx = CheckedInt128.Subtract(point.X, startX), wy = CheckedInt128.Subtract(point.Y, startY);
            Int128 projection = CheckedInt128.Add(CheckedInt128.Multiply(wx, dx), CheckedInt128.Multiply(wy, dy));
            if (length == 0 || projection <= 0)
                return CheckedInt128.Multiply(CheckedInt128.Add(CheckedInt128.Multiply(wx, wx), CheckedInt128.Multiply(wy, wy)), denominatorSquared) <= radiusSquared;
            if (projection >= maximumProjection)
            {
                Int128 vx = CheckedInt128.Subtract(point.X, endX), vy = CheckedInt128.Subtract(point.Y, endY);
                return CheckedInt128.Multiply(CheckedInt128.Add(CheckedInt128.Multiply(vx, vx), CheckedInt128.Multiply(vy, vy)), denominatorSquared) <= radiusSquared;
            }
            Int128 cross = CheckedInt128.Subtract(CheckedInt128.Multiply(dx, wy), CheckedInt128.Multiply(dy, wx));
            return CheckedInt128.Multiply(CheckedInt128.Multiply(cross, cross), denominatorSquared) <= interiorLimit;
        }
    }

    private static Int128 MultiplyNonnegative(Int128 a, Int128 b)
    {
        if (a < 0 || b < 0) return CheckedInt128.Multiply(a, b);
        if (a == 0 || b == 0) return 0;
        int Bits(Int128 value)
        {
            ulong high = (ulong)((UInt128)value >> 64);
            return high != 0 ? 128 - BitOperations.LeadingZeroCount(high)
                : 64 - BitOperations.LeadingZeroCount((ulong)value);
        }
        // Positive operands with at most 127 combined bits have product
        // strictly below 2^127. All other cases retain checked overflow and
        // the exact BigInteger capsule fallback.
        if (Bits(a) + Bits(b) <= 127) return a * b;
        return CheckedInt128.Multiply(a, b);
    }

    private sealed class BigCapsuleParameters
    {
        private readonly BigInteger dx, dy, length, maximumProjection, radiusSquared, interiorLimit;
        private readonly BigInteger startX, startY, endX, endY;
        private readonly BigInteger denominatorSquared;
        internal BigCapsuleParameters(OriginalSite site, long radius, int shift, int radiusFractionBits)
        {
            BigInteger scale = BigInteger.One << shift;
            dx = (BigInteger)site.End.X - site.Start.X; dy = (BigInteger)site.End.Y - site.Start.Y;
            length = dx * dx + dy * dy; maximumProjection = length * scale;
            BigInteger radial = radius * scale; radiusSquared = radial * radial; interiorLimit = radiusSquared * length;
            denominatorSquared = BigInteger.One << (2 * radiusFractionBits);
            startX = site.Start.X * scale; startY = site.Start.Y * scale;
            endX = site.End.X * scale; endY = site.End.Y * scale;
        }
        internal bool Contains(DyadicPoint point)
        {
            BigInteger wx = (BigInteger)point.X - startX, wy = (BigInteger)point.Y - startY;
            BigInteger projection = wx * dx + wy * dy;
            if (length.IsZero || projection.Sign <= 0) return (wx * wx + wy * wy) * denominatorSquared <= radiusSquared;
            if (projection >= maximumProjection)
            {
                BigInteger vx = (BigInteger)point.X - endX, vy = (BigInteger)point.Y - endY;
                return (vx * vx + vy * vy) * denominatorSquared <= radiusSquared;
            }
            BigInteger cross = dx * wy - dy * wx;
            return cross * cross * denominatorSquared <= interiorLimit;
        }
    }

    // Frozen reflected controls keep their original private Triangle signature;
    // the shared decomposition owns all chain arithmetic and correction order.
    private static List<(Point64 Start, Point64 End)> Residual(Paths64 boundary,
        IReadOnlyList<Triangle> triangles, CancellationToken token, Action? workCheckpoint)
        => ExactSignedBoundaryDecomposition.Residual(boundary,
            triangles.Select(triangle => new SignedTriangle(triangle.A, triangle.B, triangle.C)).ToArray(),
            token, workCheckpoint);

    private static bool AppendSignedCorrections(IReadOnlyList<(Point64 Start, Point64 End)> residual,
        List<Triangle> triangles, CancellationToken token, Action? workCheckpoint, out int corrections)
    {
        var shared = triangles.Select(triangle => new SignedTriangle(triangle.A, triangle.B, triangle.C)).ToList();
        int originalCount = shared.Count;
        bool completed = ExactSignedBoundaryDecomposition.AppendSignedCorrections(residual, shared,
            token, workCheckpoint, out corrections, out _);
        for (int i = originalCount; i < shared.Count; i++)
            triangles.Add(new(shared[i].A, shared[i].B, shared[i].C));
        return completed;
    }
}
