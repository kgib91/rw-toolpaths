using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveIntrinsicGeometryTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PinchedOffsetCyclesShareOneVisitWithoutLosingChordsOrHoleWinding(bool hole)
    {
        // Exact 603 lobe from b660: area = 1.5 Clipper grid cells squared.
        // Size is irrelevant: connection and physical coverage decide the route.
        Point3D shared = new(51.360626220703125, 61.4869384765625, -6.35);
        var lobe = new List<Point3D>
        {
            shared, new(51.36065673828125, 61.4862060546875, -6.35),
            new(51.360626220703125, 61.487030029296875, -6.35), shared
        };
        var main = new List<Point3D>
        {
            shared, new(shared.X + 2, shared.Y, shared.Z),
            new(shared.X + 2, shared.Y + 3, shared.Z),
            new(shared.X, shared.Y + 3, shared.Z), shared
        };
        if (hole) main.Reverse();
        var paths = new List<List<Point3D>> { main, lobe };
        var joined = MedialAxisToolpaths.JoinTouchingClearingCycles(paths);
        var circuit = Assert.Single(joined);
        Assert.Equal(circuit[0], circuit[^1]);
        Assert.Equal(Chords(paths), Chords(joined));
        double Area(IEnumerable<Point3D> points) => Clipper.Area(new PathD(points.Select(p => new PointD(p.X, p.Y))));
        Assert.Equal(Area(main) + Area(lobe), Area(circuit), 8);
    }

    [Fact]
    public void IdenticalDerivedCyclesHaveOneFloorSweep()
    {
        var ring = new List<Point3D> { new(0, 0, -1), new(2, 0, -1), new(2, 2, -1), new(0, 2, -1), new(0, 0, -1) };
        var reverse = ring.AsEnumerable().Reverse().ToList();
        var joined = MedialAxisToolpaths.JoinTouchingClearingCycles([ring, reverse]);
        Assert.Equal(ring, Assert.Single(joined));
    }

    [Fact]
    public void CompleteChordProofFindsAnIslandBetweenClearEndpoints()
    {
        var outer = Rectangle(-10, -10, 10, 10);
        var hole = Rectangle(-.5, -.5, .5, .5).Reverse().ToArray();
        var index = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper([outer, hole]));
        Point64 P(double x, double y) => new((long)Math.Round(x * PathUtils.Scale), (long)Math.Round(y * PathUtils.Scale));
        var start = P(-3, 0); var end = P(3, 0);
        Assert.True(index.IsInside(start));
        Assert.True(index.IsInside(end));
        Assert.True(index.DistanceSquared(start) > PathUtils.Scale * PathUtils.Scale);
        Assert.True(index.DistanceSquared(end) > PathUtils.Scale * PathUtils.Scale);
        Assert.NotNull(index.ClearanceViolation(start, end, PathUtils.Scale));
    }

    [Fact]
    public void SourceProofKeepsARealNarrowCenterRegionAndItsLargerNeighbor()
    {
        IReadOnlyList<PointD>[] source = [Rectangle(0, 0, 10, 10), Rectangle(211, 74, 211.204, 74.4)];
        var inset = PolygonOps.CutterInset(source, .1);
        Assert.Contains(inset, ring => ring.All(p => p.x > 210));
        Assert.Contains(inset, ring => ring.Any(p => p.x < 1));
        var narrow = Assert.Single(inset.Where(ring => ring.All(p => p.x > 210)));
        Assert.True(narrow.Max(p => p.x) - narrow.Min(p => p.x) < .004);
        foreach (var p in narrow)
        {
            double x = Math.Round((float)p.x, 3), y = Math.Round((float)p.y, 3);
            Assert.InRange(x, 211.1, 211.104);
            Assert.InRange(y, 74.1, 74.3);
        }
    }

    [Fact]
    public void SurfaceVisitsAreRejectedButRealShallowRampsAndSpringCutsSurvive()
    {
        const double surface = 12.7;
        bool Cuts(params double[] depths) => VCarveToolpaths.HasCommandedPenetration(
            depths.Select((depth, i) => new Point3D(i, 0, surface - depth)).ToList(), surface);
        Assert.False(Cuts(0, 0, 0));
        Assert.False(Cuts(0, .0004, 0));
        Assert.True(Cuts(0, .0006, 0));
        Assert.True(Cuts(0, .1, .1));
        Assert.True(Cuts(.1, .1, .1));
    }

    [Theory]
    [InlineData(.0002, false)]
    [InlineData(.0006, true)]
    public void PublicStrategyEmitsVisitsOnlyWhenTheMachineCommandsPenetration(double depth, bool cuts)
    {
        var tool = new ToolGeometry(.5)
        {
            BottomRadius = 0, TopRadius = .5, ConeLength = 1, TipAngleRadians = 2 * Math.Atan(.5)
        };
        var plan = VCarveToolpaths.Generate([new MillingRegion(Rectangle(0, 0, 4, 3))], tool,
            new DepthSchedule(depth, depth) { SurfaceZ = 12.7 },
            new VCarveOptions { IncludeInteriorFill = false, CalculateSweptArea = false, DetectCurves = false },
            new EmptyProvider());
        Assert.Equal(cuts, plan.Toolpaths.Count > 0);
        Assert.Equal(cuts ? 1 : 0, plan.Diagnostics.PathCount);
        Assert.All(plan.Toolpaths, path => Assert.True(VCarveToolpaths.HasCommandedPenetration(path.Points, 12.7)));
    }

    [Theory]
    [InlineData(true, 0)]
    [InlineData(true, 12.7)]
    [InlineData(false, 0)]
    [InlineData(false, 12.7)]
    public void ShallowClearingChordsFitTheActualF3BallOrSteepConeFootprint(bool ball, double surface)
    {
        const double depth = .0006;
        CarvingCutterSection section = ball
            ? new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)
            : new(CarvingCutterSectionKind.Linear, 0, 30, 1);
        // The operation's nominal cap is deliberately below its rounded command.
        var profile = new VCarveCutterProfile([section], depth);
        var source = Rectangle(0, 0, 1, 1);
        var paths = MedialAxisToolpaths.GenerateClearingPassesTagged([source], 0, depth, 0, depth,
            includeInteriorFill: false, detectCurves: false, cutterProfile: profile, outputSurfaceZ: surface);
        Assert.NotEmpty(paths);
        double commandDepth = Math.Round((float)surface - (float)(surface - depth), 3,
            MidpointRounding.AwayFromZero);
        Assert.Equal(.001, commandDepth);
        double radius = ball ? Math.Sqrt(commandDepth * (2 - commandDepth)) : 30 * commandDepth;
        Assert.True(radius - profile.RadiusAtDepth(depth) > .006);
        var sourceIndex = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper([source]));
        foreach (var path in paths)
        for (int i = 1; i < path.Points.Count; i++)
        {
            Assert.Equal(-depth, path.Points[i].Z);
            var a = path.Points[i - 1]; var b = path.Points[i];
            Point64 Command(Point3D p) => new(
                (long)Math.Round(Math.Round((float)p.X, 3, MidpointRounding.AwayFromZero) * PathUtils.Scale),
                (long)Math.Round(Math.Round((float)p.Y, 3, MidpointRounding.AwayFromZero) * PathUtils.Scale));
            Assert.Null(sourceIndex.ClearanceViolation(Command(a), Command(b), radius * PathUtils.Scale));
        }
    }

    [Fact]
    public void F3MidpointFormattingDoesNotInventADeeperFootprintAndDropARealNarrowRegion()
    {
        const double depth = .0625;
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 8, 1)], depth);
        float relativeZ = (float)-depth;
        double commandDepth = -double.Parse(relativeZ.ToString("F3", System.Globalization.CultureInfo.InvariantCulture),
            System.Globalization.CultureInfo.InvariantCulture);
        double expectedRadius = 8 * Math.Max(depth, commandDepth);
        Assert.Equal(expectedRadius, MedialAxisToolpaths.CommandedCuttingRadius(profile, 0, 8, 0, depth, 0));
        // This feasible center region is small, but its radius and commanded
        // penetration satisfy the physical tool; no size heuristic may omit it.
        var paths = MedialAxisToolpaths.GenerateClearingPassesTagged(
            [Rectangle(0, 0, 2 * expectedRadius + .004, 2)], 0, depth, 0, depth,
            includeInteriorFill: false, detectCurves: false, cutterProfile: profile);
        Assert.NotEmpty(paths);
    }

    private sealed class EmptyProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle = 3 * Math.PI / 4, bool useBigIntegers = true) => [];
    }

    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];

    private static string[] Chords(IEnumerable<List<Point3D>> paths)
        => paths.SelectMany(path => path.Zip(path.Skip(1), (a, b) => $"{a.X:R},{a.Y:R},{a.Z:R}>{b.X:R},{b.Y:R},{b.Z:R}"))
            .OrderBy(value => value, StringComparer.Ordinal).ToArray();
}
