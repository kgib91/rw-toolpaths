using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class AdaptiveRingOffsetTests
{
    [Fact]
    public void CircularFrontsKeepTheAuthoredPitchWhenTheirSweepsMeet()
    {
        var rings = RingOffsetEngine.BuildRings(new[] { new MillingRegion(Circle(30)) },
            Options(3));

        Assert.Equal(3, rings.Count);
        foreach (var ring in rings)
        {
            double expectedRadius = 30 - 3 - 6 * ring.Level;
            Assert.All(ring.Points, point => Assert.InRange(
                Math.Abs(Math.Sqrt(point.x * point.x + point.y * point.y) - expectedRadius), 0, .003));
            Assert.Equal(3 + 6 * ring.Level, ring.OffsetDistance);
        }
    }

    [Fact]
    public void PhysicalShellProofRetainsHoleWindingAcrossAcceptedLevels()
    {
        var region = new MillingRegion(Circle(60), new[] { Circle(20) });
        var rings = RingOffsetEngine.BuildRings(new[] { region }, Options(2));

        Assert.Equal(4, rings.Count);
        foreach (var level in rings.GroupBy(ring => ring.Level))
        {
            var outside = Assert.Single(level.Where(ring => !ring.IsHole));
            var hole = Assert.Single(level.Where(ring => ring.IsHole));
            double inset = 3 + 6 * level.Key;
            Assert.All(outside.Points, point => Assert.InRange(
                Math.Abs(Math.Sqrt(point.x * point.x + point.y * point.y) - (60 - inset)), 0, .003));
            Assert.All(hole.Points, point => Assert.InRange(
                Math.Abs(Math.Sqrt(point.x * point.x + point.y * point.y) - (20 + inset)), 0, .003));
            Assert.Equal(inset, outside.OffsetDistance);
            Assert.Equal(inset, hole.OffsetDistance);
        }
    }

    [Fact]
    public void ADisappearingAttachedArmGetsACoherentFrontInsteadOfMedialStubs()
    {
        var region = new MillingRegion(new PointD[]
        {
            new(0, 0), new(60, 0), new(60, 27), new(90, 27),
            new(90, 43), new(60, 43), new(60, 70), new(0, 70),
        });
        int terminalCalls = 0;
        var acceptedInsets = new List<double>();
        var rings = RingOffsetEngine.BuildRings(new[] { region }, Options(4) with
        {
            TerminalPaths = (_, _) => { terminalCalls++; return Array.Empty<IReadOnlyList<PointD>>(); },
            LevelProgress = (_, inset) => acceptedInsets.Add(inset),
        });

        Assert.Equal(4, rings.Count);
        Assert.All(rings, ring => Assert.True(ring.IsClosed));
        Assert.Equal(0, terminalCalls);
        Assert.True(acceptedInsets.Zip(acceptedInsets.Skip(1), (a, b) => b > a).All(value => value));
        Assert.Contains(rings.Skip(1), ring => ring.Points.Max(point => point.x) > 80);

        // These points lie in the legal arm interior but beyond the first
        // contour's physical sweep. Test cutter distance directly, independently
        // of the collector's boolean shell proof and its accepted inset values.
        for (int x = 65; x <= 85; x++)
        for (int y = 31; y <= 39; y++)
        {
            double distance = rings.Min(ring => DistanceToClosedRing(new PointD(x, y), ring.Points));
            Assert.True(distance <= 3 + .003 + 4 / PathUtils.Scale,
                $"Attached arm stock is unreachable from the accepted fronts at ({x},{y}); cutter distance={distance:R}.");
        }
    }

    [Fact]
    public void AnAlreadyCutThinComponentStopsBeforeANeedlessNextContour()
    {
        var region = new MillingRegion(new PointD[]
        {
            new(0, 0), new(6.01, 0), new(6.01, 40), new(0, 40),
        });
        var rings = RingOffsetEngine.BuildRings(new[] { region }, Options(4) with { StepOver = .003 });

        var first = Assert.Single(rings);
        Assert.Equal(0, first.Level);
        Assert.True(first.IsClosed);
        Assert.True(DistanceToClosedRing(new PointD(3.005, 20), first.Points) < 3);
    }

    [Fact]
    public void CompletingAnAnnularComponentRemovesItsOuterAndHoleContoursTogether()
    {
        var region = new MillingRegion(Circle(10), new[] { Circle(3) });
        var rings = RingOffsetEngine.BuildRings(new[] { region }, Options(4) with { StepOver = .2 });

        Assert.Equal(2, rings.Count);
        Assert.Single(rings.Where(ring => ring.IsHole));
        Assert.Single(rings.Where(ring => !ring.IsHole));
        Assert.All(rings, ring => Assert.Equal(0, ring.Level));
    }

    [Fact]
    public void AThinPositiveFilledComponentRemainsWhenItsPhysicalCutIsStillNeeded()
    {
        var region = new MillingRegion(Circle(60), new[] { Circle(47.99) });
        var rings = RingOffsetEngine.BuildRings(new[] { region }, Options(2) with { StepOver = 3 });

        Assert.Equal(4, rings.Count);
        var last = rings.Where(ring => ring.Level == 1).ToArray();
        Assert.Equal(2, last.Length);
        Assert.InRange(last.Single(ring => !ring.IsHole).Area - last.Single(ring => ring.IsHole).Area, 0, 5);
        var witness = new PointD(53.994, 0);
        Assert.True(rings.Where(ring => ring.Level == 0).Min(ring => DistanceToClosedRing(witness, ring.Points)) > 3);
        Assert.True(last.Min(ring => DistanceToClosedRing(witness, ring.Points)) < .02);
    }

    [Fact]
    public void AFirstFrontBelowTheAuthoredAreaCutoffCannotSupplyGhostCoverage()
    {
        var rings = RingOffsetEngine.BuildRings(new[] { new MillingRegion(Circle(9.02)) },
            Options(4) with { MinimumRingArea = 200 });

        Assert.Empty(rings);
    }

    private static RingOffsetOptions Options(int levels) => new(3, 6)
    {
        RingLimit = levels,
        MinimumRingArea = 0,
        PhysicalSweepRadius = 3,
        Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = 0 },
    };

    private static List<PointD> Circle(double radius) => Enumerable.Range(0, 512).Select(index =>
    {
        double angle = index * Math.PI * 2 / 512;
        return new PointD(radius * Math.Cos(angle), radius * Math.Sin(angle));
    }).ToList();

    private static double DistanceToClosedRing(PointD point, IReadOnlyList<PointD> ring)
    {
        double closest = double.PositiveInfinity;
        for (int index = 0; index < ring.Count; index++)
        {
            var from = ring[index];
            var to = ring[(index + 1) % ring.Count];
            double dx = to.x - from.x, dy = to.y - from.y;
            double lengthSquared = dx * dx + dy * dy;
            double t = lengthSquared == 0 ? 0 : Math.Clamp(((point.x - from.x) * dx + (point.y - from.y) * dy)
                / lengthSquared, 0, 1);
            closest = Math.Min(closest, Math.Sqrt(Math.Pow(point.x - from.x - t * dx, 2)
                + Math.Pow(point.y - from.y - t * dy, 2)));
        }
        return closest;
    }
}
