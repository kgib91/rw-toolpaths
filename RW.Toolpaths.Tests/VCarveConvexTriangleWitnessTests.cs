using System.Numerics;
using System.Reflection;
using System.Runtime.InteropServices;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.Acceleration;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

// Numeric-mode environment variables are process global. Reuse the existing
// nonparallel collection rather than introducing an overlapping policy owner.
[Collection("Pocket construction policy controls")]
public sealed class VCarveConvexTriangleWitnessTests(ITestOutputHelper output)
{
    private const double OriginalGap = .004;
    private static double ReservedGap => OriginalGap - GpuConvexTriangleWitnessValidator.GeometryReserve;

    [Fact]
    public void TriangleAbiKeepsOriginalDoubleCoordinatesAndParameters()
    {
        Assert.Equal(56, Marshal.SizeOf<GpuTriangleQuery>());
        Assert.Equal(32, Marshal.SizeOf<GpuTriangleNomination>());
        Assert.Equal(0, Marshal.OffsetOf<GpuTriangleQuery>(nameof(GpuTriangleQuery.Ax)).ToInt32());
        Assert.Equal(48, Marshal.OffsetOf<GpuTriangleQuery>(nameof(GpuTriangleQuery.Desired)).ToInt32());
        Assert.Equal(0, Marshal.OffsetOf<GpuTriangleNomination>(nameof(GpuTriangleNomination.T0)).ToInt32());
        Assert.Equal(24, Marshal.OffsetOf<GpuTriangleNomination>(nameof(GpuTriangleNomination.SegmentOrdinal)).ToInt32());
        Assert.Equal(28, Marshal.OffsetOf<GpuTriangleNomination>(nameof(GpuTriangleNomination.Status)).ToInt32());
    }

    [Fact]
    public void LongCapsuleTrianglePassesWhenOneCircumballDiscCannotCoverIt()
    {
        GpuDiscSegment[] segments = [new(-50, 0, 1, 50, 0, 1, 1, 17)];
        var query = new GpuTriangleQuery(-40, -.9, 40, -.9, 0, .9, ReservedGap);
        var nomination = new GpuTriangleNomination(0, .1, .9, .5, 1);
        Assert.True(-1 > OriginalGap - 40 - VCarveCoverageCertificate.ArithmeticReserve);
        int charges = 0;
        Assert.True(GpuConvexTriangleWitnessValidator.TryValidate(segments, Cone(), query, nomination,
            out int owner, () => charges++));
        Assert.Equal(17, owner); Assert.Equal(3, charges);
        Sample(query, (x, y) => Assert.True(PointSegmentDistance(x, y, segments[0]) < 1));
        output.WriteLine("[VCarve.Triangle.Test] longCapsule=True singleDiscCircumball=False sameInterval=True sampledInteriorPhysical=True charges=3");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void VariableFirstHemisphereTriangleCoversSampledActualContinuousSweep(bool reverse)
    {
        var segment = new GpuDiscSegment(-3, 0, .002, 3, 0, .9, 1, 29);
        if (reverse) segment = new(segment.Bx, segment.By, segment.Bz, segment.Ax, segment.Ay, segment.Az, 1, 29);
        GpuDiscSegment[] segments = [segment];
        var query = new GpuTriangleQuery(-2, -.35, 2, -.35, 0, .35, ReservedGap);
        var nomination = new GpuTriangleNomination(0, reverse ? 5d / 6 : 1d / 6,
            reverse ? 1d / 6 : 5d / 6, .5, 1);
        Assert.True(GpuConvexTriangleWitnessValidator.TryValidate(segments, Sphere(), query, nomination, out int owner));
        Assert.Equal(29, owner);
        Sample(query, (x, y) => Assert.True(SphereCenterSegmentDistance(x, y, segment) < 1,
            $"Actual continuous hemisphere sweep misses {x:R},{y:R}."));
        output.WriteLine($"[VCarve.Triangle.Test] firstHemisphere=True reverse={reverse} sampledInteriorPhysical=True");
    }

    [Fact]
    public void SeparateIntervalsWithOneOwnerCannotCombineVerticesAcrossAnOmittedHole()
    {
        GpuDiscSegment[] perimeter =
        [
            new(-5, -5, .25, 5, -5, .25, .25, 7),
            new(5, -5, .25, 5, 5, .25, .25, 7),
            new(5, 5, .25, -5, 5, .25, .25, 7),
            new(-5, 5, .25, -5, -5, .25, .25, 7),
        ];
        var query = new GpuTriangleQuery(-5, -5, 5, -5, 5, 5, ReservedGap);
        foreach ((double x, double y) in new[] { (query.Ax, query.Ay), (query.Bx, query.By), (query.Cx, query.Cy) })
            Assert.Contains(perimeter, s => PointSegmentDistance(x, y, s) <= .25);
        double centerX = (query.Ax + query.Bx + query.Cx) / 3, centerY = (query.Ay + query.By + query.Cy) / 3;
        Assert.DoesNotContain(perimeter, s => PointSegmentDistance(centerX, centerY, s) <= .25 + OriginalGap);
        for (int ordinal = 0; ordinal < perimeter.Length; ordinal++)
        {
            Assert.False(GpuConvexTriangleWitnessValidator.TryValidate(perimeter, Cone(), query,
                new(ordinal, 0, 1, .5, 1), out int owner));
            Assert.Equal(-1, owner);
        }
        output.WriteLine("[VCarve.Triangle.Test] verticesCoveredByDifferentIntervals=True omittedCenter=True sameOwnerHasNoCombinedAuthority=True");
    }

    [Fact]
    public void CoveredCentroidCannotAuthorizeMissedVertices()
    {
        GpuDiscSegment[] segments = [new(-.1, 0, .5, .1, 0, .5, .5, 11)];
        var query = new GpuTriangleQuery(-2, -.1, 2, -.1, 0, .1, ReservedGap);
        Assert.True(PointSegmentDistance(0, -.1 / 3, segments[0]) < .5);
        Assert.False(GpuConvexTriangleWitnessValidator.TryValidate(segments, Cone(), query,
            new(0, 0, 1, .5, 1), out int owner));
        Assert.Equal(-1, owner);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ThreePoseSpanCrossingAProfileJoinHasNoConcavityAuthority(bool hemisphereJoin)
    {
        var profile = hemisphereJoin
            ? new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
                new(CarvingCutterSectionKind.Linear, 1, 2, 1)], 2)
            : new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 1, 1),
                new(CarvingCutterSectionKind.Linear, 1, 1.2, 1)], 2);
        GpuDiscSegment[] segments = [new(-.5, 0, .5, .5, 0, 1.5, 2, 31)];
        int charges = 0;
        Assert.False(GpuConvexTriangleWitnessValidator.TryValidate(segments, profile,
            new(-.1, -.1, .1, -.1, 0, .1, ReservedGap), new(0, .2, .8, .5, 1), out int owner, () => charges++));
        Assert.Equal(-1, owner); Assert.Equal(0, charges);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.0005)]
    [InlineData(.000999999)]
    public void UnengagedAndSubminimumDepthsCannotAuthorizeTriangle(double depth)
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 1, 1, 2)], 2);
        GpuDiscSegment[] segments = [new(-1, 0, depth, 1, 0, depth, 1, 42)];
        Assert.False(GpuConvexTriangleWitnessValidator.TryValidate(segments, profile,
            new(-.5, -.2, .5, -.2, 0, .2, ReservedGap), new(0, .25, .75, .5, 1), out int owner));
        Assert.Equal(-1, owner);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(-.00000002, true)]
    [InlineData(.00000002, false)]
    public void OriginalGapBoundaryRetainsItsArithmeticReserve(double offset, bool expected)
    {
        GpuDiscSegment[] segments = [new(-1, 0, 1, 1, 0, 1, 1, 9)];
        var query = new GpuTriangleQuery(-.1, -.2, .1, -.2, 0, 1 + OriginalGap + offset, ReservedGap);
        Assert.Equal(expected, GpuConvexTriangleWitnessValidator.TryValidate(segments, Cone(), query,
            new(0, .45, .55, .5, 1), out int owner));
        Assert.Equal(expected ? 9 : -1, owner);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void DeepDyadicVertexConversionFitsInsideItsAllocatedReserve(int sign)
    {
        const int shift = 48, denominatorPower = shift + 15;
        // One exact dyadic tick beyond an IEEE halfway tie approaches the
        // worst admitted conversion error rather than an exactly stored point.
        Int128 numerator = sign * (((Int128)9999 * 32768 << shift) + ((Int128)1 << 23) + 1);
        double converted = (double)numerator / Math.ScaleB(32768, shift);
        BigInteger doubleTicks = ExactDoubleTimesPowerOfTwo(converted, denominatorPower);
        BigInteger errorTicks = BigInteger.Abs((BigInteger)numerator - doubleTicks);
        Assert.True(errorTicks <= BigInteger.One << (denominatorPower - 40));
        Assert.True(Math.Sqrt(2) * Math.ScaleB(1, -40) < GpuConvexTriangleWitnessValidator.DyadicCoordinateReserve);
        Assert.True(1e-9 + GpuConvexTriangleWitnessValidator.DyadicCoordinateReserve
            < GpuConvexTriangleWitnessValidator.GeometryReserve);
        output.WriteLine($"[VCarve.Triangle.Test] dyadicShift={shift} world={converted:R} exactErrorTicks={errorTicks} reserve={GpuConvexTriangleWitnessValidator.DyadicCoordinateReserve:R}");
    }

    [Fact]
    public void MalformedNominationsAndUnsupportedDomainsStayInconclusive()
    {
        GpuDiscSegment[] segments = [new(-1, 0, 1, 1, 0, 1, 1, 11)];
        var query = new GpuTriangleQuery(-.1, -.1, .1, -.1, 0, .1, ReservedGap);
        GpuTriangleNomination[] invalid =
        [
            new(-1, .45, .55, .5, 1), new(1, .45, .55, .5, 1),
            new(0, double.NaN, .55, .5, 1), new(0, -.1, .55, .5, 1),
            new(0, .45, 1.1, .5, 1), new(0, .45, .55, double.PositiveInfinity, 1),
            new(0, .45, .55, .5, 0), new(0, .45, .55, .5, 2),
        ];
        foreach (var nomination in invalid)
            Assert.False(GpuConvexTriangleWitnessValidator.TryValidate(segments, Cone(), query, nomination, out _));
        var valid = new GpuTriangleNomination(0, .45, .55, .5, 1);
        GpuTriangleQuery[] unsupported =
        [
            new(double.NaN, -.1, .1, -.1, 0, .1, ReservedGap),
            new(-.1, -.1, .1, double.PositiveInfinity, 0, .1, ReservedGap),
            new(-.1, -.1, .1, -.1, 10001, .1, ReservedGap),
            new(-.1, -.1, .1, -.1, 0, .1, -1),
            new(-.1, -.1, .1, -.1, 0, .1, double.NaN),
        ];
        foreach (var malformed in unsupported)
            Assert.False(GpuConvexTriangleWitnessValidator.TryValidate(segments, Cone(), malformed, valid, out _));
        Assert.False(GpuConvexTriangleWitnessValidator.TryValidate([], Cone(), query, valid, out _));
        Assert.False(GpuConvexTriangleWitnessValidator.TryValidate([new(-1, 0, 1, 1, 0, 1, 1, -1)], Cone(), query, valid, out _));
        Assert.False(GpuConvexTriangleWitnessValidator.TryValidate(segments,
            new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 2, .01)], .01), query, valid, out _));
        // A later physical section beyond the nominal operation cap can be
        // discontinuous. Actual overdepth must retain the strict span refusal.
        var laterHemisphere = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, .25, .25, 1),
            new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)], 1);
        Assert.False(GpuConvexTriangleWitnessValidator.TryValidate([new(-1, 0, 1.5, 1, 0, 1.5, 1, 11)],
            laterHemisphere, query, valid, out _));
    }

    [Fact]
    public void CallbackCancellationAndDisposedContextCannotPublishTriangleOwner()
    {
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        GpuDiscSegment[] segments = [new(-1, 0, 1, 1, 0, 1, 1, 11)];
        var query = new GpuTriangleQuery(-.1, -.1, .1, -.1, 0, .1, ReservedGap);
        Assert.Throws<OperationCanceledException>(() => GpuConvexTriangleWitnessValidator.TryValidate(segments, Cone(), query,
            new(0, .45, .55, .5, 1), out _, cancellation.Token.ThrowIfCancellationRequested));
        var constructor = typeof(VCarveGpuDiscWitnessContext).GetConstructors(BindingFlags.Instance | BindingFlags.NonPublic).Single();
        var snapshot = (VCarveGpuDiscWitnessContext)constructor.Invoke(new object?[] { segments,
            new[] { new GpuDiscNode(-1, 0, 1, 0, 1, 0, 1, -1, -1) }, Cone(), null });
        int charges = 0;
        Assert.True(snapshot.TryValidateTriangleNomination(query, new(0, .45, .55, .5, 1), out int owner, () => charges++));
        Assert.Equal(11, owner); Assert.Equal(3, charges);
        snapshot.Dispose();
        Assert.False(snapshot.TryNominateTriangles([query], new GpuTriangleNomination[1], 1, default, out string reason));
        Assert.Equal("context-disposed", reason);
        Assert.Throws<OperationCanceledException>(() => snapshot.TryNominateTriangles([query], new GpuTriangleNomination[1],
            1, cancellation.Token, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RealCudaTriangleBatchesRetainOriginalCpuAuthorityInBothNumericModes(bool fp32)
    {
        using var mode = new NumericMode(fp32);
        const double origin = 400;
        GpuDiscSegment[] segments = [new(origin, origin, .5, origin + 1, origin, .5, 1, 19)];
        GpuDiscNode[] nodes = [new(origin, origin, origin + 1, origin, 1, 0, 1, -1, -1)];
        if (!Create(segments, nodes, Sphere(), out var context)) return;
        using (context)
        {
            Assert.Equal(fp32, context!.Fp32Enabled);
            Assert.True(context.SupportsConvexTriangleWitnesses);
            var queries = new GpuTriangleQuery[VCarveGpuDiscWitnessContext.MaximumBatchSize + 1];
            for (int index = 0; index < queries.Length; index++)
                queries[index] = (index % 3) switch
                {
                    0 => new(origin, origin - .8, origin + 1, origin - .8, origin + .5, origin + .8, ReservedGap),
                    1 => new(origin, origin + 2, origin + 1, origin + 2, origin + .5, origin + 3, ReservedGap),
                    _ => new(origin - 2, origin - .1, origin + 3, origin - .1, origin + .5, origin + .1, ReservedGap),
                };
            var nominations = new GpuTriangleNomination[queries.Length];
            Assert.True(context.TryNominateTriangles(queries, nominations, queries.Length, default, out string reason), reason);
            Assert.Equal(2, context.Batches); Assert.Equal(queries.Length, context.TriangleQueryCount);
            Assert.Equal(context.QueryCount, context.PositiveNominations + context.NominationMisses + context.TraversalRefusals);
            for (int index = 0; index < queries.Length; index++)
            {
                if (index % 3 == 0)
                {
                    Assert.True(nominations[index].HasWitness);
                    int charges = 0;
                    Assert.True(context.TryValidateTriangleNomination(queries[index], nominations[index], out int owner, () => charges++));
                    Assert.Equal(19, owner); Assert.Equal(3, charges);
                }
                else Assert.False(context.TryValidateTriangleNomination(queries[index], nominations[index], out _));
            }
            Assert.Equal(fp32 ? queries.Length : 0, context.Fp32Queries);
            GpuTriangleQuery[] unsupported = [new(origin, origin, origin + 1, origin, origin + .5, origin, 101)];
            var fallback = new GpuTriangleNomination[1];
            Assert.True(context.TryNominateTriangles(unsupported, fallback, 1, default, out reason), reason);
            Assert.False(context.TryValidateTriangleNomination(unsupported[0], fallback[0], out _));
            Assert.Equal(fp32 ? 1 : 0, context.Fp64FallbackBatches);
            output.WriteLine($"[VCarve.Triangle.Test] actualCuda=True fp32={fp32} queries={context.TriangleQueryCount} kernelMs={context.TriangleKernelMilliseconds:F3} cpuAuthority=True");
        }
    }

    [Fact]
    public void FloatTriangleNomineeCanBeOptimisticAndStillFailsLiteralCpuValidation()
    {
        using var mode = new NumericMode(true);
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, .15, .15, 1)], 1);
        if (!Create([new(0, 0, .1, 1, 0, .1, .15, 7)], [new(0, 0, 1, 0, .15, 0, 1, -1, -1)], profile, out var context)) return;
        using (context)
        {
            Assert.True(context!.Fp32Enabled, context.Fp32RefusalReason);
            // The original .1500000025 and .15 both round to the same float.
            // A degenerate closed triangle isolates that optimistic rounding:
            // double relative XY cancels the query offset, while segment-local
            // Y rounds as above. Its result still requires all 3 actual checks.
            GpuTriangleQuery[] queries = [new(.5, .1500000025, .5, .1500000025, .5, .1500000025, 0)];
            var nominations = new GpuTriangleNomination[1];
            Assert.True(context.TryNominateTriangles(queries, nominations, 1, default, out string reason), reason);
            Assert.True(nominations[0].HasWitness, "Control requires an actual optimistic float nominee, rather than a miss.");
            Assert.False(context.TryValidateTriangleNomination(queries[0], nominations[0], out int owner));
            Assert.Equal(-1, owner);
            output.WriteLine("[VCarve.Triangle.Test] actualOptimisticFp32Nominee=True originalCpuPredicateRejected=True");
        }
    }

    [Fact]
    public void DeviceTraversalAndBroadPhaseRefusalsNeverAcquireNegativeProofAuthority()
    {
        using var mode = new NumericMode(false);
        GpuDiscSegment[] segments = [new(0, 0, .5, 1, 0, .5, 1, 19)];
        var query = new GpuTriangleQuery(0, -.8, 1, -.8, .5, .8, ReservedGap);
        if (!Create(segments, [new(0, 0, 1, 0, 1, 0, 0, 0, 0)], Sphere(), out var cyclic)) return;
        using (cyclic)
        {
            var nominations = new GpuTriangleNomination[1];
            Assert.True(cyclic!.TryNominateTriangles([query], nominations, 1, default, out string reason), reason);
            Assert.Equal(2, nominations[0].Status); Assert.False(nominations[0].HasWitness);
            Assert.True(cyclic.TryValidateTriangleNomination(query, new(0, 0, 1, .5, 1), out _));
        }
        // Malformed nomination-only bounds can suppress a valid witness, but
        // cannot certify stock. The untouched CPU geometry still proves it.
        if (!Create(segments, [new(50, 50, 51, 51, 1, 0, 1, -1, -1)], Sphere(), out var badBounds)) return;
        using (badBounds)
        {
            var nominations = new GpuTriangleNomination[1];
            Assert.True(badBounds!.TryNominateTriangles([query], nominations, 1, default, out string reason), reason);
            Assert.False(nominations[0].HasWitness);
            Assert.True(badBounds.TryValidateTriangleNomination(query, new(0, 0, 1, .5, 1), out _));
            Assert.False(badBounds.TryNominateTriangles([new(double.NaN, 0, 1, 0, .5, .1, ReservedGap)],
                nominations, 1, default, out reason));
            Assert.Contains("nonfinite-triangle-query", reason);
            Assert.False(badBounds.TryNominateTriangles([query], nominations, 1, default, out reason));
            Assert.Equal(badBounds.FailureReason, reason);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AllVertexBroadPhaseSkipsDenseCentroidIntervalsBeforeTheTrueWitnessBranch(bool fp32)
    {
        using var mode = new NumericMode(fp32);
        const int irrelevant = 600;
        var segments = new GpuDiscSegment[irrelevant + 1];
        for (int index = 0; index < irrelevant; index++)
            segments[index] = new(-.05, 0, 1, .05, 0, 1, 1, 3);
        segments[irrelevant] = new(-50, 0, 1, 50, 0, 1, 1, 19);
        GpuDiscNode[] nodes =
        [
            new(-50, 0, 50, 0, 1, 0, segments.Length, 1, 2),
            new(-.05, 0, .05, 0, 1, 0, irrelevant, -1, -1),
            new(-50, 0, 50, 0, 1, irrelevant, 1, -1, -1),
        ];
        var query = new GpuTriangleQuery(-40, -.9, 40, -.9, 0, .9, ReservedGap);
        // Both branches reach the centroid and tie the original centroid lower
        // bound, so it visits the irrelevant left branch first. That branch
        // exceeds the 512 interval budget before the true right witness. It
        // cannot reach the far vertices and must be skipped before leaf search.
        Assert.True(PointSegmentDistance(0, -.3, segments[0]) < 1);
        Assert.True(PointSegmentDistance(query.Ax, query.Ay, segments[0]) > 1 + OriginalGap);
        if (!Create(segments, nodes, Cone(), out var context)) return;
        using (context)
        {
            Assert.Equal(fp32, context!.Fp32Enabled);
            var nominations = new GpuTriangleNomination[1];
            Assert.True(context.TryNominateTriangles([query], nominations, 1, default, out string reason), reason);
            Assert.True(nominations[0].HasWitness, "Dense centroid-only intervals must not exhaust the traversal before the real all-vertex branch.");
            Assert.Equal(irrelevant, nominations[0].SegmentOrdinal);
            Assert.Equal(0, context.TraversalRefusals);
            Assert.True(context.TryValidateTriangleNomination(query, nominations[0], out int owner));
            Assert.Equal(19, owner);
            output.WriteLine($"[VCarve.Triangle.Test] actualCuda=True allVertexBroadPhase=True fp32={fp32} irrelevantCenterIntervals={irrelevant} trueWitnessOwner={owner} traversalRefusals={context.TraversalRefusals}");
        }
    }

    private bool Create(GpuDiscSegment[] segments, GpuDiscNode[] nodes, VCarveCutterProfile profile,
        out VCarveGpuDiscWitnessContext? context)
    {
        bool created = VCarveGpuDiscWitnessContext.TryCreate(segments, nodes, profile, default, output.WriteLine,
            out context, out string reason);
        if (!created)
        {
            Assert.Null(context); Assert.False(string.IsNullOrWhiteSpace(reason));
            output.WriteLine($"[VCarve.Triangle.Test] actualCuda=False portableFallbackReason={reason}");
        }
        return created;
    }

    private static VCarveCutterProfile Cone() => new([new(CarvingCutterSectionKind.Linear, 0, 2, 2)], 2);
    private static VCarveCutterProfile Sphere() => new([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)], 1);
    private static void Sample(GpuTriangleQuery q, Action<double, double> assert)
    {
        const int samples = 48;
        for (int i = 0; i <= samples; i++)
        for (int j = 0; j <= samples - i; j++)
        {
            double b = (double)i / samples, c = (double)j / samples;
            assert(q.Ax + (q.Bx - q.Ax) * b + (q.Cx - q.Ax) * c,
                q.Ay + (q.By - q.Ay) * b + (q.Cy - q.Ay) * c);
        }
    }
    private static double PointSegmentDistance(double x, double y, GpuDiscSegment s)
    {
        double dx = s.Bx - s.Ax, dy = s.By - s.Ay, square = dx * dx + dy * dy;
        double t = square > 0 ? Math.Clamp(((x - s.Ax) * dx + (y - s.Ay) * dy) / square, 0, 1) : 0;
        double vx = x - s.Ax - dx * t, vy = y - s.Ay - dy * t;
        return Math.Sqrt(vx * vx + vy * vy);
    }
    private static double SphereCenterSegmentDistance(double x, double y, GpuDiscSegment s)
    {
        double ax = s.Ax - x, ay = s.Ay - y, az = 1 - s.Az;
        double dx = s.Bx - s.Ax, dy = s.By - s.Ay, dz = s.Az - s.Bz;
        double square = dx * dx + dy * dy + dz * dz;
        double t = Math.Clamp(-(ax * dx + ay * dy + az * dz) / square, 0, 1);
        ax += dx * t; ay += dy * t; az += dz * t;
        return Math.Sqrt(ax * ax + ay * ay + az * az);
    }
    private static BigInteger ExactDoubleTimesPowerOfTwo(double value, int power)
    {
        long bits = BitConverter.DoubleToInt64Bits(value);
        int exponent = (int)((bits >> 52) & 2047) - 1023 - 52 + power;
        BigInteger mantissa = (bits & 0x000fffffffffffffL) | (1L << 52);
        if (bits < 0) mantissa = -mantissa;
        return exponent >= 0 ? mantissa << exponent : mantissa >> -exponent;
    }
    private sealed class NumericMode : IDisposable
    {
        private readonly string? _previous = Environment.GetEnvironmentVariable("RW_TOOLPATHS_VCARVE_GPU_FP32");
        internal NumericMode(bool enabled) => Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_GPU_FP32", enabled ? "1" : null);
        public void Dispose() => Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_GPU_FP32", _previous);
    }
}
