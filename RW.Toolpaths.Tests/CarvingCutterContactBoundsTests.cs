using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class CarvingCutterContactBoundsTests
{
    [Fact]
    public void SlopedBallChordUsesTheActualContactInsteadOfClosestXY()
    {
        var cutter = Ball(1, 1);
        var source = new PointD(0.8, 0.5);
        double az = 1 + Math.Sqrt(0.11), bz = 1 + Math.Sqrt(0.71), dz = bz - az;
        double t = 0.8 - dz * Math.Sqrt(0.75 / (1 + dz * dz));
        double expected = az + dz * t + 1 - Math.Sqrt(1 - Math.Pow(0.8 - t, 2) - 0.25);
        double actual = CarvingCutterContactBounds.MinimumTipAndBottom(cutter, source,
            new PointD(0, 0), az, new PointD(1, 0), bz, 0);
        Assert.Equal(expected, actual, 12);
        double wrongNearestXY = az + dz * 0.8 + 1 - Math.Sqrt(0.75);
        Assert.True(wrongNearestXY - actual > 0.1,
            $"Expected the contact witness gap that stalled patch convergence; gap={wrongNearestXY - actual:R}.");
    }

    [Fact]
    public void ConeContactKeepsTheSlopedTipAndRadialDistanceCorrelated()
    {
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0, 3, 6) }, 6);
        double t = 0.45 - Math.Sqrt(0.36 / 60);
        double expected = 1 + t + 2 * Math.Sqrt(Math.Pow(0.9 - 2 * t, 2) + 0.36);
        Assert.Equal(expected, CarvingCutterContactBounds.MinimumTipAndBottom(cutter,
            new PointD(0.9, 0.6), new PointD(0, 0), 1, new PointD(2, 0), 2, 0), 12);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(0.2, 0.8)]
    public void DiskDomainExitIncludesTheSquareRoundingReserve(double reserve, double expected)
    {
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1, 1, 4) }, 2);
        Assert.Equal(expected, CarvingCutterContactBounds.MinimumTipAndBottom(cutter,
            new PointD(0, 0), new PointD(-2, 0), 4, new PointD(2, 0), 0, reserve), 12);
    }

    [Fact]
    public void SphereReachIsClippedAtThePhysicalContactHeight()
    {
        var cutter = Ball(1, 0.2);
        // The reachable disk radius is 0.6. The descending chord exits that disk
        // at t=0.8, where both the underside and tip slope contribute.
        Assert.Equal(0.6, CarvingCutterContactBounds.MinimumTipAndBottom(cutter,
            new PointD(0, 0), new PointD(-1, 0), 2, new PointD(1, 0), 0, 0), 12);
    }

    [Fact]
    public void ADiscontinuousWiderShaftDoesNotHideTheLowerDiskDomainBoundary()
    {
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, .7, .7, .6),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.6, 1.6, 4) }, 2);
        double expected = .9 - Math.Sqrt(.49 - .16);
        Assert.Equal(expected, CarvingCutterContactBounds.MinimumTipAndBottom(cutter,
            new PointD(.1, .4), new PointD(-1, 0), 2, new PointD(1, 0), 0, 0), 12);
    }

    [Fact]
    public void ReservedSquareReachesContactThatUnreservedNearestXYWouldReject()
    {
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, .5, .5, 4) }, 2);
        Assert.True(Math.Sqrt(.5) > cutter.MaximumRadius);
        double expected = 1 - 2 * Math.Sqrt(.035);
        Assert.Equal(expected, CarvingCutterContactBounds.MinimumTipAndBottom(cutter,
            new PointD(1, 0), new PointD(0, 0), 2, new PointD(1, 1), 0, .2), 12);
    }

    [Fact]
    public void ReservedObliqueSphereEquatorPatchConvergesToItsExactPointWitness()
    {
        // Rotation of saved source16931. The previous relief dual upper leaves a
        // permanent gap although the source rectangle shrinks to a single point.
        const double dx = .306610107421875, dy = .003265380859375;
        const double nose = 1.5875, reserve = .001119, tipZ = 6.35, height = 12.7;
        double length = Math.Sqrt(dx * dx + dy * dy);
        var point = new PointD(dx / 2 - dy / length * 1.588, dy / 2 + dx / length * 1.588);
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, nose, nose),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, nose, 3, 20) }, 6.35);
        var start = new PointD(0, 0); var end = new PointD(dx, dy);
        double witness = height - CarvingCutterContactBounds.MinimumTipAndBottom(cutter, point,
            start, tipZ, end, tipZ, reserve);
        const double halfWidth = 1e-9;
        double oldUpper = cutter.PatchUpperBound(point.x - halfWidth, point.y - halfWidth,
            point.x + halfWidth, point.y + halfWidth, height, height, height, height,
            start.x, start.y, tipZ, end.x, end.y, tipZ, reserve);
        Assert.True(oldUpper - witness > .04,
            $"The witness must expose the permanent relief tangent gap: upper={oldUpper:R} witness={witness:R}.");
        double upper = CarvingCutterContactBounds.ChordLiftUpperBound(cutter,
            point.x - halfWidth, point.y - halfWidth, point.x + halfWidth, point.y + halfWidth,
            height, start, tipZ, end, tipZ, reserve);
        Assert.InRange(upper - witness, 0, 1e-6);
    }

    [Fact]
    public void RectangularSourceExtentsAreAddedIndependentlyToTheReservedAxes()
    {
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1, 1, 4) }, 2);
        double upper = CarvingCutterContactBounds.ChordLiftUpperBound(cutter,
            -.2, -.1, .2, .1, 2, new PointD(-2, 0), 4, new PointD(2, 0), 0, 0);
        Assert.Equal(1.2 + 1e-8, upper, 12);
    }

    [Fact]
    public void RectangularSlopedContactUpperBoundsIndependentPhysicalSweepSamples()
    {
        var profiles = new[] {
            Ball(1, 1), Ball(1, .37),
            new CarvingCutterProfile(new[] {
                new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, .6, .6),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, .6, 2, 4),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 2, 2, 10) }, 6),
            new CarvingCutterProfile(new[] {
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, .7, .7, .6),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.6, 1.6, 4) }, 2) };
        var random = new Random(835691);
        foreach (var cutter in profiles)
        for (int sample = 0; sample < 24; sample++)
        {
            var start = new PointD(100, 300);
            var end = new PointD(100 + 3 * (random.NextDouble() - .5), 300 + 3 * (random.NextDouble() - .5));
            double x0 = 100 + 3 * (random.NextDouble() - .5), y0 = 300 + 3 * (random.NextDouble() - .5);
            double x1 = x0 + .01 + random.NextDouble(), y1 = y0 + .01 + random.NextDouble();
            double az = 3 * random.NextDouble(), bz = 3 * random.NextDouble(), reserve = .2 * random.NextDouble();
            const double height = 4;
            double upper = CarvingCutterContactBounds.ChordLiftUpperBound(cutter, x0, y0, x1, y1,
                height, start, az, end, bz, reserve);
            for (int step = 0; step <= 96; step++)
            for (int iy = 0; iy <= 12; iy++)
            for (int ix = 0; ix <= 12; ix++)
            {
                double t = step / 96d, x = x0 + (x1 - x0) * ix / 12, y = y0 + (y1 - y0) * iy / 12;
                double rx = Math.Max(0, Math.Abs(x - (start.x + t * (end.x - start.x))) - reserve);
                double ry = Math.Max(0, Math.Abs(y - (start.y + t * (end.y - start.y))) - reserve);
                double radius = Math.Sqrt(rx * rx + ry * ry);
                if (radius > cutter.MaximumRadius) continue;
                double contact = height - az - t * (bz - az) - cutter.Bottom(radius);
                Assert.True(upper + 1e-7 >= contact,
                    $"Rectangle sweep underbound: sample={sample} t={t:R} source=({x:R},{y:R}) upper={upper:R} contact={contact:R}.");
            }
        }
    }

    [Fact]
    public void SquareReserveBreaksAndStackJunctionsBoundIndependentContactSamples()
    {
        var profiles = new[] {
            Ball(1, 1), Ball(1, 0.37),
            new CarvingCutterProfile(new[] {
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0, 3, 2) }, 2),
            new CarvingCutterProfile(new[] {
                new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 0.4, 0.4),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0.4, 1.3, 1.7),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.3, 1.3, 4) }, 6),
            new CarvingCutterProfile(new[] {
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0.7, 0.7, 0.6),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.6, 1.6, 1),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.6, 0.3, 1),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0.3, 2, 2) }, 2.4) };
        var random = new Random(982371);
        foreach (var cutter in profiles)
        for (int sample = 0; sample < 36; sample++)
        {
            var start = new PointD(100, 300);
            var end = new PointD(100 + 4 * (random.NextDouble() - 0.5), 300 + 4 * (random.NextDouble() - 0.5));
            var point = new PointD(100 + 4 * (random.NextDouble() - 0.5), 300 + 4 * (random.NextDouble() - 0.5));
            double az = 4 * random.NextDouble(), bz = 4 * random.NextDouble(), reserve = random.NextDouble() * 0.3;
            double exact = CarvingCutterContactBounds.MinimumTipAndBottom(cutter, point, start, az, end, bz, reserve);
            double sampled = double.PositiveInfinity;
            for (int step = 0; step <= 24000; step++)
            {
                double t = step / 24000d;
                double rx = Math.Max(0, Math.Abs(point.x - (start.x + t * (end.x - start.x))) - reserve);
                double ry = Math.Max(0, Math.Abs(point.y - (start.y + t * (end.y - start.y))) - reserve);
                double radius = Math.Sqrt(rx * rx + ry * ry);
                if (radius <= cutter.MaximumRadius)
                    sampled = Math.Min(sampled, az + t * (bz - az) + cutter.Bottom(radius));
            }
            if (double.IsFinite(sampled))
            {
                Assert.True(exact <= sampled + 1e-9,
                    $"Contact minimum missed a physical sample: profile={Array.IndexOf(profiles, cutter)} sample={sample} exact={exact:R} sampled={sampled:R}.");
                Assert.True(sampled - exact < 0.005,
                    $"Contact minimum escaped the physical domain: sample={sample} exact={exact:R} sampled={sampled:R}.");
            }
            else Assert.True(double.IsPositiveInfinity(exact), $"Unexpected finite contact: sample={sample} exact={exact:R}.");
        }
    }

    private static CarvingCutterProfile Ball(double radius, double depth) => new(new[] {
        new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, radius, radius) }, depth);
}
