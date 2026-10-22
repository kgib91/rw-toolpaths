using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.Acceleration;

namespace RW.Toolpaths.Tests;

public sealed class VCarveConvexBoxWitnessTests
{
    private const double OriginalGap = .004;
    private static double ReservedGap => OriginalGap - GpuConvexBoxWitnessValidator.GeometryReserve;

    [Fact]
    public void LongCapsuleCoversBoxWhoseCircumballCannotFitAnySinglePhysicalDisc()
    {
        var profile = Cone();
        GpuDiscSegment[] segments = [new(-50, 0, 1, 50, 0, 1, 1, 17)];
        var box = new GpuBoxQuery(-40, -.9, 40, .9, ReservedGap);
        var nomination = new GpuBoxNomination(0, .1, .9, .9, .1, 1);
        double circumball = Math.Sqrt(40 * 40 + .9 * .9);
        // At the center the deepest possible literal signed disc distance is -1.
        // The existing circumball witness asks for about -40 and must subdivide.
        Assert.True(-1 > OriginalGap - circumball - VCarveCoverageCertificate.ArithmeticReserve);
        Assert.True(GpuConvexBoxWitnessValidator.TryValidate(segments, profile, box, nomination, out int owner));
        Assert.Equal(17, owner);
        for (int ix = 0; ix <= 40; ix++)
        for (int iy = 0; iy <= 12; iy++)
        {
            double x = box.X0 + (box.X1 - box.X0) * ix / 40;
            double y = box.Y0 + (box.Y1 - box.Y0) * iy / 12;
            Assert.True(PointSegmentDistance(x, y, segments[0]) <= 1);
        }
        Console.WriteLine($"[VCarve.ConvexBox.Test] shape=long-capsule circumballRadius={circumball:R} "
            + "singleDiscCircumball=False fourCornerSameInterval=True sampledInteriorPhysical=True owner=17");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void VariableSphereCornersCertifyInteriorOfActualContinuousCommandSweep(bool reverse)
    {
        var profile = Sphere(1);
        var segment = new GpuDiscSegment(-3, 0, .002, 3, 0, .9, 1, 29);
        if (reverse) segment = new(segment.Bx, segment.By, segment.Bz, segment.Ax, segment.Ay, segment.Az, 1, 29);
        GpuDiscSegment[] segments = [segment];
        var box = new GpuBoxQuery(-2, -.4, 2, .4, ReservedGap);
        double first = reverse ? 5d / 6 : 1d / 6, last = reverse ? 1d / 6 : 5d / 6;
        var nomination = new GpuBoxNomination(0, first, last, last, first, 1);
        Assert.True(GpuConvexBoxWitnessValidator.TryValidate(segments, profile, box, nomination, out int owner));
        Assert.Equal(29, owner);
        double smallestMargin = double.PositiveInfinity;
        for (int ix = 0; ix <= 48; ix++)
        for (int iy = 0; iy <= 20; iy++)
        {
            double x = box.X0 + (box.X1 - box.X0) * ix / 48;
            double y = box.Y0 + (box.Y1 - box.Y0) * iy / 20;
            // A moving first-hemisphere cutter is a constant-radius sphere
            // capsule intersected by the stock halfspace. This independent 3D
            // capsule distance checks every sampled surface point.
            double distance = SphereCenterSegmentDistance(x, y, segment, 1);
            Assert.True(distance < 1, $"Continuous sphere sweep misses interior at {x:R},{y:R}.");
            smallestMargin = Math.Min(smallestMargin, 1 - distance);
        }
        Console.WriteLine($"[VCarve.ConvexBox.Test] shape=variable-sphere reverse={reverse} "
            + $"smallestSampledSphereMargin={smallestMargin:R} sameInterval=True sampledInteriorPhysical=True");
    }

    [Fact]
    public void DifferentIntervalsWithOnePathOwnerCannotCombineCornerCoverageAcrossAnUncutCenter()
    {
        var profile = Cone();
        GpuDiscSegment[] perimeter =
        [
            new(-5, -5, .25, 5, -5, .25, .25, 7),
            new(5, -5, .25, 5, 5, .25, .25, 7),
            new(5, 5, .25, -5, 5, .25, .25, 7),
            new(-5, 5, .25, -5, -5, .25, .25, 7),
        ];
        double[] x = [-5, 5, 5, -5], y = [-5, -5, 5, 5];
        for (int corner = 0; corner < 4; corner++)
            Assert.Contains(perimeter, segment => PointSegmentDistance(x[corner], y[corner], segment) <= .25);
        Assert.DoesNotContain(perimeter, segment => PointSegmentDistance(0, 0, segment) <= .25 + OriginalGap);
        var box = new GpuBoxQuery(-5, -5, 5, 5, ReservedGap);
        for (int ordinal = 0; ordinal < perimeter.Length; ordinal++)
        {
            var nomination = new GpuBoxNomination(ordinal, 0, 1, 1, 0, 1);
            Assert.False(GpuConvexBoxWitnessValidator.TryValidate(perimeter, profile, box, nomination, out int owner));
            Assert.Equal(-1, owner);
        }
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CornerPoseSpanCrossingProfileJoinHasNoSingleSectionConvexityAuthority(bool hemisphereJoin)
    {
        var profile = hemisphereJoin
            ? new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
                new(CarvingCutterSectionKind.Linear, 1, 2, 1)], 2)
            : new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 1, 1),
                new(CarvingCutterSectionKind.Linear, 1, 1.2, 1)], 2);
        GpuDiscSegment[] segments = [new(-.5, 0, .5, .5, 0, 1.5, 2, 31)];
        var box = new GpuBoxQuery(-.1, -.1, .1, .1, ReservedGap);
        // Each nominated disc reaches its corner, but the pose span crosses the
        // section join. A local radius formula cannot authorize the whole span.
        Assert.True(profile.PhysicalRadiusAtDepth(.7) > Math.Sqrt(.2 * .2 + .1 * .1));
        Assert.True(profile.PhysicalRadiusAtDepth(1.3) > Math.Sqrt(.2 * .2 + .1 * .1));
        Assert.False(GpuConvexBoxWitnessValidator.TryValidate(segments, profile, box,
            new(0, .2, .8, .8, .2, 1), out int owner));
        Assert.Equal(-1, owner);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.0005)]
    [InlineData(.000999999)]
    public void FlatBottomRadiusAtUnengagedOrSubminimumDepthCannotAuthorizeBox(double depth)
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 1, 1, 2)], 2);
        GpuDiscSegment[] segments = [new(-1, 0, depth, 1, 0, depth, 1, 42)];
        Assert.False(GpuConvexBoxWitnessValidator.TryValidate(segments, profile,
            new(-.5, -.2, .5, .2, ReservedGap), new(0, .25, .75, .75, .25, 1), out int owner));
        Assert.Equal(-1, owner);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(-.00000002, true)]
    [InlineData(.00000002, false)]
    public void OriginalGapBoundaryRetainsTheOneSidedArithmeticReserve(double offset, bool expected)
    {
        GpuDiscSegment[] segments = [new(-1, 0, 1, 1, 0, 1, 1, 9)];
        var box = new GpuBoxQuery(-.1, -.2, .1, 1 + OriginalGap + offset, ReservedGap);
        Assert.Equal(expected, GpuConvexBoxWitnessValidator.TryValidate(segments, Cone(), box,
            new(0, .45, .55, .55, .45, 1), out int owner));
        Assert.Equal(expected ? 9 : -1, owner);
    }

    [Fact]
    public void OrdinaryGuardIncludesTheLargestSphereAtMinimumNominatedDepth()
    {
        var profile = Sphere(100);
        GpuDiscSegment[] segments = [new(9999, 0, .001, 9999.8, 0, .002, 1, 83)];
        var query = new GpuBoxQuery(9999.15, -.4, 9999.65, .4, ReservedGap);
        double first = (query.X0 - segments[0].Ax) / (segments[0].Bx - segments[0].Ax);
        double last = (query.X1 - segments[0].Ax) / (segments[0].Bx - segments[0].Ax);
        Assert.True(GpuConvexBoxWitnessValidator.TryValidate(segments, profile, query,
            new(0, first, last, last, first, 1), out int owner));
        Assert.Equal(83, owner);
    }

    [Fact]
    public void MalformedNominationsAndUnboundedArithmeticDomainsRemainInconclusive()
    {
        GpuDiscSegment[] segments = [new(-1, 0, 1, 1, 0, 1, 1, 11)];
        var profile = Cone(); var query = new GpuBoxQuery(-.1, -.1, .1, .1, ReservedGap);
        GpuBoxNomination[] invalid =
        [
            new(-1, .45, .55, .55, .45, 1), new(1, .45, .55, .55, .45, 1),
            new(0, double.NaN, .55, .55, .45, 1), new(0, -.1, .55, .55, .45, 1),
            new(0, .45, 1.1, .55, .45, 1), new(0, .45, .55, .55, .45, 0),
            new(0, .45, .55, .55, .45, 2),
        ];
        foreach (var nomination in invalid)
        {
            Assert.False(GpuConvexBoxWitnessValidator.TryValidate(segments, profile, query, nomination, out int owner));
            Assert.Equal(-1, owner);
        }
        var valid = new GpuBoxNomination(0, .45, .55, .55, .45, 1);
        GpuBoxQuery[] invalidBoxes =
        [
            new(.1, -.1, -.1, .1, ReservedGap), new(-.1, .1, .1, -.1, ReservedGap),
            new(double.NaN, -.1, .1, .1, ReservedGap), new(-.1, -.1, .1, .1, double.PositiveInfinity),
            new(-.1, -.1, 10001, .1, ReservedGap), new(-.1, -.1, .1, .1, -1),
        ];
        foreach (var box in invalidBoxes)
            Assert.False(GpuConvexBoxWitnessValidator.TryValidate(segments, profile, box, valid, out _));
        Assert.False(GpuConvexBoxWitnessValidator.TryValidate([], profile, query, valid, out _));
        Assert.False(GpuConvexBoxWitnessValidator.TryValidate(
            [new(-1, 0, 1, 1, 0, 1, 1, -1)], profile, query, valid, out _));
        Assert.False(GpuConvexBoxWitnessValidator.TryValidate(segments,
            new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 2, .01)], .01), query, valid, out _));
    }

    [Fact]
    public void PhysicalDiscCheckpointPropagatesCancellationInsteadOfPublishingAnOwner()
    {
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        GpuDiscSegment[] segments = [new(-1, 0, 1, 1, 0, 1, 1, 11)];
        Assert.Throws<OperationCanceledException>(() => GpuConvexBoxWitnessValidator.TryValidate(segments, Cone(),
            new(-.1, -.1, .1, .1, ReservedGap), new(0, .45, .55, .55, .45, 1), out _,
            cancellation.Token.ThrowIfCancellationRequested));
    }

    private static VCarveCutterProfile Cone() => new([new(CarvingCutterSectionKind.Linear, 0, 2, 2)], 2);
    private static VCarveCutterProfile Sphere(double radius) => new([new(CarvingCutterSectionKind.Hemisphere,
        0, radius, radius)], radius);

    private static double PointSegmentDistance(double x, double y, GpuDiscSegment s)
    {
        double dx = s.Bx - s.Ax, dy = s.By - s.Ay, square = dx * dx + dy * dy;
        double t = square > 0 ? Math.Clamp(((x - s.Ax) * dx + (y - s.Ay) * dy) / square, 0, 1) : 0;
        double vx = x - s.Ax - dx * t, vy = y - s.Ay - dy * t;
        return Math.Sqrt(vx * vx + vy * vy);
    }

    private static double SphereCenterSegmentDistance(double x, double y, GpuDiscSegment s, double radius)
    {
        double ax = s.Ax - x, ay = s.Ay - y, az = radius - s.Az;
        double dx = s.Bx - s.Ax, dy = s.By - s.Ay, dz = s.Az - s.Bz;
        double square = dx * dx + dy * dy + dz * dz;
        double t = square > 0 ? Math.Clamp(-(ax * dx + ay * dy + az * dz) / square, 0, 1) : 0;
        double vx = ax + dx * t, vy = ay + dy * t, vz = az + dz * t;
        return Math.Sqrt(vx * vx + vy * vy + vz * vz);
    }
}
