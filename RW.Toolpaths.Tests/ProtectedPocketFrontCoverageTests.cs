using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ProtectedPocketFrontCoverageTests
{
    [Fact]
    public void ExplicitClosingChordsCoverSmallContourWithoutAnInvertedInnerHole()
    {
        // Reduced from the saved protected-pocket front. Its three short round
        // shoulder chords make a joined offset retain an inverted inner fold.
        var ring = new PointD[]
        {
            new(3.20697021484375,-.303314208984375), new(3.280059814453125,-.20477294921875),
            new(3.343017578125,-.099700927734375), new(3.395538330078125,.0113525390625),
            new(.561492919921875,1.52874755859375), new(-.156005859375,.37060546875),
            new(.777618408203125,-.66510009765625), new(1.79901123046875,-.79595947265625),
        };
        const double radius = 3.174999952316284;
        var witness = new PointD(.8775041393723, 1.3595110232357);
        var ordinary = RingOffsetEngine.BufferClosedSweeps(new[] { ring }, radius, .001,
            CancellationToken.None, explicitChords: false);
        var protectedSweep = RingOffsetEngine.BufferClosedSweeps(new[] { ring }, radius, .001,
            CancellationToken.None, explicitChords: true);

        Assert.True(Distance(witness, ring) < .31);
        Assert.False(Contains(ordinary, witness));
        Assert.True(Contains(protectedSweep, witness));
        // Independently construct points on every commanded segment capsule,
        // including the explicit final-to-first chord and stationary endpoints.
        for (int edge = 0; edge < ring.Length; edge++)
        for (int sample = 0; sample <= 8; sample++)
        for (int angle = 0; angle < 32; angle++)
        {
            var a = ring[edge]; var b = ring[(edge + 1) % ring.Length];
            double t = sample / 8d, theta = 2 * Math.PI * angle / 32;
            var point = new PointD(a.x + t * (b.x - a.x) + (radius - .005) * Math.Cos(theta),
                a.y + t * (b.y - a.y) + (radius - .005) * Math.Sin(theta));
            Assert.True(Contains(protectedSweep, point), $"Missing cutter contact at edge={edge}, t={t:R}, angle={angle}.");
        }
    }

    [Fact]
    public void CompletedChildNeverReappearsAfterItsAcceptedSweepLeavesTheActiveFront()
    {
        var source = new MillingRegion(new PointD[]
        {
            new(0,0), new(40,0), new(40,19), new(50,19), new(50,15), new(70,15),
            new(70,25), new(50,25), new(50,21), new(40,21), new(40,40), new(0,40),
        });
        var options = new RingOffsetOptions(3, .2)
        {
            RingLimit = 6, MinimumRingArea = 0, PhysicalSweepRadius = 3,
            Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = 0 },
            RetainEmittedSweepHistory = true,
        };
        var rings = RingOffsetEngine.BuildRings(new[] { source }, options);
        var child = Assert.Single(rings.Where(ring => ring.Points.Max(point => point.x) > 45));
        Assert.Equal(0, child.Level);
        Assert.Equal(6, rings.Count(ring => ring.Points.Max(point => point.x) < 45));
        for (double x = 53.1; x <= 66.9; x += .5)
        for (double y = 18.1; y <= 21.9; y += .5)
            Assert.True(Distance(new PointD(x, y), child.Points) < 3);
    }

    [Fact]
    public void ClippedAwayChordsCannotAuthorizeProtectedCoverage()
    {
        var ring = new PointD[] { new(-10,-1), new(10,-1), new(10,1), new(-10,1) };
        var clip = new PointD[] { new(0,-20), new(20,-20), new(20,20), new(0,20) };
        var sweep = RingOffsetEngine.BufferClosedSweeps(new[] { ring }, 2, .001,
            CancellationToken.None, explicitChords: true, clip);
        Assert.False(Contains(sweep, new PointD(-5, 0)));
        Assert.True(Contains(sweep, new PointD(5, 0)));
    }

    private static bool Contains(IEnumerable<IReadOnlyList<PointD>> paths, PointD point)
    {
        var scaled = new Point64((long)Math.Round(point.x * PathUtils.Scale), (long)Math.Round(point.y * PathUtils.Scale));
        return PathUtils.ToClipper(paths).Sum(ring => Clipper.PointInPolygon(scaled, ring) == PointInPolygonResult.IsOutside
            ? 0 : Clipper.IsPositive(ring) ? 1 : -1) != 0;
    }

    private static double Distance(PointD point, IReadOnlyList<PointD> ring)
    {
        double minimum = double.PositiveInfinity;
        for (int edge = 0; edge < ring.Count; edge++)
        {
            var a = ring[edge]; var b = ring[(edge + 1) % ring.Count];
            double dx = b.x - a.x, dy = b.y - a.y, lengthSquared = dx * dx + dy * dy;
            double t = lengthSquared == 0 ? 0 : Math.Clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / lengthSquared, 0, 1);
            minimum = Math.Min(minimum, Math.Sqrt(Math.Pow(point.x - a.x - t * dx, 2) + Math.Pow(point.y - a.y - t * dy, 2)));
        }
        return minimum;
    }
}
