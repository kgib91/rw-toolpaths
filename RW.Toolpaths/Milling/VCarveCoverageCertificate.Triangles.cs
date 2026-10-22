using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Milling.Acceleration;
using RW.Toolpaths.Milling.Certification;

namespace RW.Toolpaths.Milling;

internal static partial class VCarveCoverageCertificate
{
    // Numerators remain exact through every partition. GPU/CPU floating vertices
    // only nominate positive physical witnesses; they never replace this domain.
    private readonly record struct DyadicTriangle(Int128 Ax, Int128 Ay, Int128 Bx, Int128 By,
        Int128 Cx, Int128 Cy, int Shift, int BoundaryHint)
    {
        internal GpuTriangleQuery Query(double desired)
        {
            double scale = Math.ScaleB(PathUtils.Scale, Shift);
            return new((double)Ax / scale, (double)Ay / scale, (double)Bx / scale,
                (double)By / scale, (double)Cx / scale, (double)Cy / scale, desired);
        }

        internal void Split(GpuTriangleQuery q, out DyadicTriangle first, out DyadicTriangle second)
        {
            // Choosing an edge needs no authority. Its midpoint and both closed
            // children are exact, even when the rounded edge lengths are tied.
            double ab = Square(q.Ax - q.Bx) + Square(q.Ay - q.By);
            double bc = Square(q.Bx - q.Cx) + Square(q.By - q.Cy);
            double ca = Square(q.Cx - q.Ax) + Square(q.Cy - q.Ay);
            if (ab >= bc && ab >= ca)
            {
                first = new(Ax * 2, Ay * 2, Ax + Bx, Ay + By, Cx * 2, Cy * 2, Shift + 1, BoundaryHint);
                second = new(Ax + Bx, Ay + By, Bx * 2, By * 2, Cx * 2, Cy * 2, Shift + 1, BoundaryHint);
            }
            else if (bc >= ca)
            {
                first = new(Ax * 2, Ay * 2, Bx * 2, By * 2, Bx + Cx, By + Cy, Shift + 1, BoundaryHint);
                second = new(Ax * 2, Ay * 2, Bx + Cx, By + Cy, Cx * 2, Cy * 2, Shift + 1, BoundaryHint);
            }
            else
            {
                first = new(Ax * 2, Ay * 2, Bx * 2, By * 2, Cx + Ax, Cy + Ay, Shift + 1, BoundaryHint);
                second = new(Cx + Ax, Cy + Ay, Bx * 2, By * 2, Cx * 2, Cy * 2, Shift + 1, BoundaryHint);
            }
            first = first.Normalize(); second = second.Normalize();
        }

        private DyadicTriangle Normalize()
        {
            var t = this;
            while (t.Shift > 0 && ((t.Ax | t.Ay | t.Bx | t.By | t.Cx | t.Cy) & 1) == 0)
                t = new(t.Ax / 2, t.Ay / 2, t.Bx / 2, t.By / 2, t.Cx / 2, t.Cy / 2, t.Shift - 1, t.BoundaryHint);
            return t;
        }
    }

    private sealed class TriangleRun
    {
        private readonly Limits limits;
        private readonly Budget work;
        private readonly Tree boundary, mandatory;
        private readonly Tree? optional;
        private readonly VCarveGpuDiscWitnessContext device;
        private readonly VCarveGpuDiscWitnessContext? optionalDevice;
        private readonly VCarveCutterProfile profile;
        private readonly double gap;
        private readonly bool[]? owners;
        private readonly Action<string>? diagnostic;
        internal TriangleRun(Limits limits, Budget work, Tree boundary, Tree mandatory, Tree? optional,
            VCarveGpuDiscWitnessContext device, VCarveGpuDiscWitnessContext? optionalDevice,
            VCarveCutterProfile profile, double gap, bool[]? owners, Action<string>? diagnostic)
        {
            this.limits = limits; this.work = work; this.boundary = boundary; this.mandatory = mandatory;
            this.optional = optional; this.device = device; this.optionalDevice = optionalDevice;
            this.profile = profile; this.gap = gap; this.owners = owners; this.diagnostic = diagnostic;
        }
        internal bool Used;
        internal string Reason = "preparing";
        internal string? GpuFailureReason;
        internal int SourceTriangles, CorrectionTriangles;
        internal double PreparationMilliseconds;
        internal bool PreparationReused;
        internal long Visited, Queries, Witnesses, OptionalQueries, OptionalWitnesses;
        internal long BoundaryWitnesses, Subdivisions, FallbackQueries, BigIntegerFallbacks;
        internal long DiscQueries, DiscWitnesses, OptionalDiscQueries, OptionalDiscWitnesses;
        internal bool BoundaryGpuAvailable;
        internal string BoundaryGpuReason = "disabled";
        internal long BoundaryGpuQueries, BoundaryGpuWitnesses, BoundaryGpuMisses, BoundaryGpuRefusals;
        internal PointD? Unresolved;
        private readonly Stack<DyadicTriangle> _pending = new();
        private int _outstanding;
        internal int Remaining => _pending.Count + _outstanding;

        internal bool? TryCertify(IReadOnlyList<IReadOnlyList<PointD>> target)
        {
            long started = Stopwatch.GetTimestamp();
            try
            {
                if (!device.SupportsConvexTriangleWitnesses || gap <= ConvexBoxArithmeticReserve)
                { Reason = "unsupported-convex-profile"; return null; }
                if (target.Sum(ring => (long)ring.Count) > ExactSignedBoundaryDecomposition.MaximumInputPoints)
                { Reason = "target-point-budget"; return null; }
                var grid = new Paths64(target.Count);
                foreach (var ring in target)
                {
                    var path = new Path64(ring.Count);
                    foreach (PointD point in ring)
                    {
                        work.Check();
                        if (!Ordinary(point.x) || !Ordinary(point.y))
                        { Reason = "target-coordinate-domain"; return null; }
                        double x = point.x * PathUtils.Scale, y = point.y * PathUtils.Scale;
                        long ix = (long)Math.Round(x), iy = (long)Math.Round(y);
                        if (BitConverter.DoubleToInt64Bits(ix / PathUtils.Scale) != BitConverter.DoubleToInt64Bits(point.x)
                            || BitConverter.DoubleToInt64Bits(iy / PathUtils.Scale) != BitConverter.DoubleToInt64Bits(point.y))
                        { Reason = "target-not-exact-grid"; return null; }
                        path.Add(new(ix, iy));
                    }
                    grid.Add(path);
                }
                var preparation = limits.TriangleDomainPreparation ?? new PreparedTriangleDomain();
                if (!preparation.TryCreateOrReuse(grid, limits.UseDelaunayTriangleDomain, work.Cancellation, work.CheckNow,
                    out var triangles, out var decomposition, out PreparationReused))
                { Reason = decomposition.Reason; return null; }
                SourceTriangles = decomposition.MainTriangles;
                CorrectionTriangles = decomposition.CorrectionTriangles;
                boundary.PrepareBoundaryTriangleHints();
                foreach (var triangle in triangles)
                {
                    work.Check();
                    int hint = boundary.FindBoundaryTriangleHint(triangle);
                    _pending.Push(new(triangle.A.X, triangle.A.Y, triangle.B.X, triangle.B.Y,
                        triangle.C.X, triangle.C.Y, 0, hint));
                }
                Used = true; Reason = "exact-signed-boundary-chain";
                diagnostic?.Invoke($"stage=smooth-physical-certificate event=triangle-selection used=True " +
                    $"sourceTriangles={SourceTriangles} correctionTriangles={CorrectionTriangles} " +
                    $"delaunay={limits.UseDelaunayTriangleDomain} preparationReused={PreparationReused} " +
                    $"chainReason={decomposition.Reason} triangulationMs={decomposition.TriangulationMilliseconds:F3} " +
                    $"chainMs={decomposition.ChainMilliseconds:F3}");
            }
            finally
            {
                PreparationMilliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                diagnostic?.Invoke($"stage=smooth-physical-certificate event=triangle-prepare used={Used} reason={Reason} " +
                    $"prepareMs={PreparationMilliseconds:F3}");
            }
            return Run();
        }

        private bool Run()
        {
            int size = limits.GpuBatchSize;
            var tasks = new DyadicTriangle[size];
            var queries = new GpuTriangleQuery[size];
            var nominations = new GpuTriangleNomination[size];
            var compactQueries = new GpuTriangleQuery[size];
            var compactNominations = new GpuTriangleNomination[size];
            var ordinals = new int[size];
            var accepted = new bool[size];
            var witnessOwners = new int[size];
            var discQueries = new GpuDiscQuery[size];
            var compactDiscQueries = new GpuDiscQuery[size];
            var discNominations = new GpuDiscNomination[size];
            var centers = new PointD[size];
            var radii = new double[size];
            var boundaryQueries = new GpuBoundaryQuery[size];
            var boundaryNominations = new GpuBoundarySampleNomination[size];
            var boundarySamples = new GpuBoundarySampleNomination[size];
            var boundaryCompleted = new bool[size];
            VCarveGpuDiscWitnessContext.BoundarySampleContext? boundaryGpu = null;
            if (limits.UseGpuBoundaryNominations)
            {
                BoundaryGpuAvailable = boundary.TryCreateBoundaryGpu(diagnostic, out boundaryGpu, out BoundaryGpuReason);
                if (BoundaryGpuAvailable) BoundaryGpuReason = "ready";
            }
            using var boundaryGpuOwner = boundaryGpu;
            if (limits.UseGpuBoundaryNominations)
            {
                diagnostic?.Invoke($"stage=smooth-physical-certificate event=boundary-device-selection available={BoundaryGpuAvailable} "
                    + $"reason={BoundaryGpuReason} authority=exact-original-finite-capsule misses=nomination-only");
            }
            bool boundaryHealthy = BoundaryGpuAvailable;
            Action discCheckpoint = work.Disc;
            bool healthy = true, optionalHealthy = optionalDevice is not null;
            // Floor and decrement keep this exact finite capsule strictly inside
            // the original gap. A failed hint carries no negative authority.
            double scaled = Math.BitDecrement((gap - ConvexBoxArithmeticReserve) * PathUtils.Scale * (1L << 24));
            long capsuleRadius = scaled > 0 && scaled < long.MaxValue ? (long)Math.Floor(scaled) : 0;
            long reported = Stopwatch.GetTimestamp();
            while (_pending.Count != 0)
            {
                int count = 0;
                while (count < size && _pending.TryPop(out var task))
                {
                    work.CheckNow();
                    if (Visited >= limits.MaximumBoxes)
                    { _pending.Push(task); Reason = "box-budget"; return false; }
                    Visited++;
                    GpuTriangleQuery query = task.Query(gap - ConvexBoxArithmeticReserve);
                    if (capsuleRadius > 0)
                    {
                        bool boundaryCovered = boundary.TryBoundaryTriangleWitness(task, query, capsuleRadius,
                            nearest: false, out _, out int big);
                        BigIntegerFallbacks += big;
                        if (boundaryCovered) { BoundaryWitnesses++; continue; }
                    }
                    tasks[count] = task; queries[count] = query;
                    var center = new PointD(query.Ax / 3 + query.Bx / 3 + query.Cx / 3,
                        query.Ay / 3 + query.By / 3 + query.Cy / 3);
                    centers[count] = center;
                    double radius = Math.Sqrt(Math.Max(Square(query.Ax - center.x) + Square(query.Ay - center.y),
                        Math.Max(Square(query.Bx - center.x) + Square(query.By - center.y),
                            Square(query.Cx - center.x) + Square(query.Cy - center.y))));
                    radii[count] = radius;
                    discQueries[count] = new(center.x, center.y, gap - radius - ConvexBoxArithmeticReserve);
                    count++; _outstanding++;
                }
                if (count == 0) continue;
                Array.Clear(accepted, 0, count);
                if (healthy)
                {
                    if (device.TryNominateTriangles(queries, nominations, count, work.Cancellation, out string reason))
                    {
                        Queries += count;
                        for (int i = 0; i < count; i++)
                        {
                            accepted[i] = device.TryValidateTriangleNomination(queries[i], nominations[i],
                                out witnessOwners[i], discCheckpoint);
                            if (accepted[i]) Witnesses++;
                        }
                    }
                    else { healthy = false; ReportFallback("mandatory-triangle", reason); }
                    work.CheckNow();
                }
                // Complete mandatory nomination first, including tips and strict
                // section joins refused by the convex span guard. Optional
                // terminal credit must not precede an available retained disc.
                int discCount = CompactDiscs();
                if (healthy && discCount > 0)
                {
                    if (device.TryNominate(compactDiscQueries, discNominations, discCount, work.Cancellation, out string reason))
                    {
                        DiscQueries += discCount;
                        for (int j = 0; j < discCount; j++)
                        {
                            int i = ordinals[j];
                            accepted[i] = mandatory.TryValidateGpuNomination(centers[i], discQueries[i].Desired,
                                discNominations[j], profile, out witnessOwners[i]);
                            if (accepted[i]) DiscWitnesses++;
                        }
                    }
                    else { healthy = false; ReportFallback("mandatory-disc", reason); }
                    work.CheckNow();
                }
                if (healthy && optionalHealthy)
                {
                    int compactCount = 0;
                    for (int i = 0; i < count; i++) if (!accepted[i])
                    { compactQueries[compactCount] = queries[i]; ordinals[compactCount++] = i; }
                    if (compactCount > 0)
                    {
                        if (optionalDevice!.TryNominateTriangles(compactQueries, compactNominations, compactCount,
                            work.Cancellation, out string reason))
                        {
                            OptionalQueries += compactCount;
                            for (int j = 0; j < compactCount; j++)
                            {
                                int i = ordinals[j];
                                accepted[i] = optionalDevice.TryValidateTriangleNomination(queries[i], compactNominations[j],
                                    out witnessOwners[i], discCheckpoint);
                                if (accepted[i]) OptionalWitnesses++;
                            }
                        }
                        else { optionalHealthy = false; ReportFallback("optional-triangle", reason); }
                        work.CheckNow();
                    }
                }
                discCount = CompactDiscs();
                if (healthy && optionalHealthy && discCount > 0)
                {
                    if (optionalDevice!.TryNominate(compactDiscQueries, discNominations, discCount, work.Cancellation, out string reason))
                    {
                        OptionalDiscQueries += discCount;
                        for (int j = 0; j < discCount; j++)
                        {
                            int i = ordinals[j];
                            accepted[i] = optional!.TryValidateGpuNomination(centers[i], discQueries[i].Desired,
                                discNominations[j], profile, out witnessOwners[i]);
                            if (accepted[i]) OptionalDiscWitnesses++;
                        }
                    }
                    else { optionalHealthy = false; ReportFallback("optional-disc", reason); }
                    work.CheckNow();
                }
                Array.Clear(boundaryCompleted, 0, count);
                if (boundaryHealthy && capsuleRadius > 0)
                {
                    int boundaryCount = 0;
                    for (int i = 0; i < count; i++) if (!accepted[i])
                    {
                        // Rounded centroids choose an original edge only. The
                        // exact three-vertex capsule predicate owns all credit.
                        boundaryQueries[boundaryCount] = new((long)Math.Round(centers[i].x * PathUtils.Scale),
                            (long)Math.Round(centers[i].y * PathUtils.Scale));
                        ordinals[boundaryCount++] = i;
                    }
                    if (boundaryCount > 0)
                    {
                        if (boundaryGpu!.TryNominate(boundaryQueries, boundaryNominations, boundaryCount,
                            work.Cancellation, out string reason))
                        {
                            BoundaryGpuQueries += boundaryCount;
                            for (int j = 0; j < boundaryCount; j++)
                            {
                                int i = ordinals[j];
                                boundarySamples[i] = boundaryNominations[j];
                                boundaryCompleted[i] = boundary.IsOriginalBoundaryNomination(boundarySamples[i]);
                                if (!boundaryCompleted[i]) BoundaryGpuRefusals++;
                            }
                        }
                        else
                        {
                            boundaryHealthy = false; BoundaryGpuReason = reason;
                            diagnostic?.Invoke($"stage=smooth-physical-certificate event=boundary-device-fallback reason={reason}");
                        }
                        work.CheckNow();
                    }
                }
                for (int i = 0; i < count; i++)
                {
                    work.CheckNow();
                    if (accepted[i])
                    { if (owners is not null) owners[witnessOwners[i]] = true; _outstanding--; continue; }
                    var task = tasks[i];
                    int hint = task.BoundaryHint, big = 0;
                    bool boundaryCovered;
                    if (capsuleRadius > 0 && boundaryCompleted[i])
                    {
                        hint = boundarySamples[i].SegmentOrdinal;
                        boundaryCovered = boundary.TryBoundaryGpuTriangleWitness(task, queries[i], capsuleRadius,
                            boundarySamples[i], out big);
                        if (boundaryCovered) BoundaryGpuWitnesses++; else BoundaryGpuMisses++;
                        // A miss merely declines this fringe nomination. This
                        // task still requires a real disc, exact subdivision,
                        // or an inconclusive result; it never excludes stock.
                    }
                    else
                        boundaryCovered = capsuleRadius > 0 && boundary.TryBoundaryTriangleWitness(task, queries[i], capsuleRadius,
                            nearest: true, out hint, out big);
                    BigIntegerFallbacks += big;
                    if (boundaryCovered) { BoundaryWitnesses++; _outstanding--; continue; }
                    task = task with { BoundaryHint = hint };
                    if (!healthy || radii[i] <= gap / 4 || optional is not null && !optionalHealthy)
                    {
                        FallbackQueries++;
                        if (mandatory.HasDiscWitness(centers[i], discQueries[i].Desired, profile, out int owner)
                            || optional is not null && optional.HasDiscWitness(centers[i], discQueries[i].Desired, profile, out owner))
                        { if (owners is not null) owners[owner] = true; _outstanding--; continue; }
                    }
                    if (radii[i] <= limits.MinimumBoxRadius || task.Shift >= 48)
                    { Unresolved = centers[i]; Reason = task.Shift >= 48 ? "triangle-coordinate-resolution" : "unresolved-box"; return false; }
                    task.Split(queries[i], out var first, out var second);
                    _pending.Push(first); _pending.Push(second); Subdivisions++; _outstanding--;
                }
                if (Stopwatch.GetElapsedTime(reported).TotalSeconds >= 2)
                {
                    reported = Stopwatch.GetTimestamp();
                    diagnostic?.Invoke($"stage=smooth-physical-certificate event=triangle-progress visitedTriangles={Visited} " +
                        $"pending={Remaining} triangleQueries={Queries} triangleWitnesses={Witnesses} optionalTriangleWitnesses={OptionalWitnesses} " +
                        $"boundaryWitnesses={BoundaryWitnesses} subdivisions={Subdivisions} fallbackQueries={FallbackQueries} " +
                        $"discEvaluations={work.DiscEvaluations} elapsedSeconds={work.Elapsed:R}");
                }
                int CompactDiscs()
                {
                    int compactCount = 0;
                    for (int i = 0; i < count; i++) if (!accepted[i])
                    { compactDiscQueries[compactCount] = discQueries[i]; ordinals[compactCount++] = i; }
                    return compactCount;
                }
            }
            Reason = "whole-target-certified"; return true;
        }

        private void ReportFallback(string stage, string reason)
        {
            if (stage.StartsWith("mandatory", StringComparison.Ordinal)) GpuFailureReason = reason;
            diagnostic?.Invoke($"stage=smooth-physical-certificate event=triangle-device-fallback queryStage={stage} reason={reason}");
        }
    }

    private sealed partial class Tree
    {
        private readonly record struct BoundaryEdgeKey(long Ax, long Ay, long Bx, long By)
        {
            internal static BoundaryEdgeKey Create(Point64 a, Point64 b)
                => a.X < b.X || a.X == b.X && a.Y <= b.Y ? new(a.X, a.Y, b.X, b.Y) : new(b.X, b.Y, a.X, a.Y);
        }
        private Dictionary<BoundaryEdgeKey, int>? _boundaryTriangleHints;

        internal void PrepareBoundaryTriangleHints()
        {
            _boundaryTriangleHints = new(_segments.Length);
            for (int i = 0; i < _segments.Length; i++)
            {
                _work.Check(); Segment s = _segments[i];
                _boundaryTriangleHints.TryAdd(BoundaryEdgeKey.Create(Grid(s.A), Grid(s.B)), i);
            }
        }

        internal int FindBoundaryTriangleHint(SignedTriangle triangle)
        {
            if (_boundaryTriangleHints!.TryGetValue(BoundaryEdgeKey.Create(triangle.A, triangle.B), out int ordinal)
                || _boundaryTriangleHints.TryGetValue(BoundaryEdgeKey.Create(triangle.B, triangle.C), out ordinal)
                || _boundaryTriangleHints.TryGetValue(BoundaryEdgeKey.Create(triangle.C, triangle.A), out ordinal)) return ordinal;
            return -1;
        }

        private static Point64 Grid(Point3D point)
            => new((long)Math.Round(point.X * PathUtils.Scale), (long)Math.Round(point.Y * PathUtils.Scale));

        internal bool TryBoundaryTriangleWitness(DyadicTriangle task, GpuTriangleQuery q, long radiusNumerator,
            bool nearest, out int hint, out int bigIntegerFallbacks)
        {
            hint = task.BoundaryHint; bigIntegerFallbacks = 0;
            if ((uint)hint < (uint)_segments.Length && Capsule(hint, task, q, radiusNumerator, out bigIntegerFallbacks)) return true;
            if (!nearest) return false;
            var center = new PointD(q.Ax / 3 + q.Bx / 3 + q.Cx / 3, q.Ay / 3 + q.By / 3 + q.Cy / 3);
            // A covering finite capsule must contain the triangle centroid.
            // Nominate only original edges in that tiny neighborhood instead of
            // finding a global nearest edge for every uncut interior triangle.
            // Floating broad-phase misses merely lose a nomination. Acceptance
            // still requires the original exact dyadic three-point predicate.
            double radius = radiusNumerator / (PathUtils.Scale * (1L << 24)) + ConvexBoxArithmeticReserve;
            double radiusSquared = radius * radius;
            int nomination = hint, fallbacks = 0, visited = 0, tested = 0;
            bool nominationBudgetExhausted = false;
            bool covered = Visit(0);
            hint = nomination; bigIntegerFallbacks += fallbacks;
            return covered;
            bool Visit(int index)
            {
                if (nominationBudgetExhausted) return false;
                _work.Check(); BoundaryTriangleNominationVisits++;
                if (++visited > 4096)
                { nominationBudgetExhausted = true; BoundaryTriangleNominationRefusals++; return false; }
                Node node = _nodes[index];
                double dx = Math.Max(0, Math.Max(node.Bounds.X0 - center.x, center.x - node.Bounds.X1));
                double dy = Math.Max(0, Math.Max(node.Bounds.Y0 - center.y, center.y - node.Bounds.Y1));
                if (dx * dx + dy * dy > radiusSquared) return false;
                if (node.Left >= 0) return Visit(node.Left) || Visit(node.Right);
                for (int i = node.First; i < node.First + node.Count; i++)
                {
                    if (i == task.BoundaryHint) continue;
                    Segment s = _segments[i];
                    double parameter = Project(center, s);
                    Point3D p = At(s.A, s.B, parameter);
                    if (Square(p.X - center.x) + Square(p.Y - center.y) > radiusSquared) continue;
                    BoundaryTriangleNominationCandidates++;
                    if (++tested > 64)
                    { nominationBudgetExhausted = true; BoundaryTriangleNominationRefusals++; return false; }
                    if (nomination == task.BoundaryHint) nomination = i;
                    bool accepted = Capsule(i, task, q, radiusNumerator, out int big);
                    fallbacks += big;
                    if (accepted) { nomination = i; return true; }
                }
                return false;
            }
        }

        private bool Capsule(int ordinal, DyadicTriangle task, GpuTriangleQuery q, long radius, out int big)
        {
            _work.Check(); Segment s = _segments[ordinal];
            // A cheap rejection only removes a nomination. Exact integer
            // capsule arithmetic remains the sole positive fringe authority.
            big = 0;
            double nominationRadius = radius / (PathUtils.Scale * (1L << 24)) + ConvexBoxArithmeticReserve;
            if (Distance(new(q.Ax, q.Ay), s) > nominationRadius
                || Distance(new(q.Bx, q.By), s) > nominationRadius
                || Distance(new(q.Cx, q.Cy), s) > nominationRadius) return false;
            return SignedBoundaryCapsuleCertificate.ContainsDyadicTriangle(Grid(s.A), Grid(s.B),
                task.Ax, task.Ay, task.Bx, task.By, task.Cx, task.Cy, task.Shift, radius, out big);
        }

        internal long BoundaryTriangleNominationVisits, BoundaryTriangleNominationCandidates, BoundaryTriangleNominationRefusals;
    }
}
