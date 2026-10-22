using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

/// <summary>
/// Full-diameter offset steps can collapse the next ring while leaving cutter-reachable
/// material along a pocket's medial strip. These checks use physical cutter discs and
/// complete boundary segments, independently of the offset and medial implementations.
/// </summary>
public sealed class PocketStripCompletionTests
{
    private const double Radius = 3;
    private const double ClearanceSlack = .0012;
    private readonly ITestOutputHelper _output;

    public PocketStripCompletionTests(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData(false, 16)]
    [InlineData(true, 16)]
    public void FullDiameterStepoverCutsTheStripLeftWhenTheNextRingCollapses(bool insideOut, double width)
    {
        var region = new MillingRegion(Rectangle(0, 0, width, 60));
        var plan = Generate(region, insideOut);
        Report($"rectangle width={width:R} insideOut={insideOut}", plan);

        Assert.NotEmpty(plan.Toolpaths);
        Assert.Equal(1, plan.Diagnostics.RingCount);
        Assert.Equal(1, plan.Diagnostics.StripCount);
        Assert.Equal(2, plan.Toolpaths.Select(path => path.VisitIdentity).Distinct().Count());
        // A parent ring may pause and resume around its native terminal child.
        Assert.InRange(plan.Toolpaths.Count, 2, 3);
        AssertContinuousClearance(plan, region, Radius);
        AssertCovered(plan, Grid(6, width - 6, 6, 54, .5), -1,
            $"collapsed rectangle width={width:R}, insideOut={insideOut}");

        // The middle is cleared along an open centre traverse, rather than inventing
        // another closed ring inside the strip. Wall-finishing rings remain useful.
        var centreMoves = CuttingMoves(plan, -1)
            .Where(move => Math.Abs(move.A.x - width / 2) < .001
                && Math.Abs(move.B.x - width / 2) < .001)
            .ToArray();
        double centreLength = centreMoves.Sum(move => Math.Abs(move.B.y - move.A.y));
        Assert.True(centreLength >= 40,
            $"Missing long open centre strip: width={width:R}, insideOut={insideOut}, centreLength={centreLength:R}.");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharpCapsNeedASecondCoherentFrontAndKeepOneChainedEntryAtEachDepth(bool insideOut)
    {
        const double width = 18, height = 60;
        var region = new MillingRegion(Rectangle(0, 0, width, height));
        var capWitnesses = new PointD[]
        {
            new(6.5, 6.5), new(11.5, 6.5), new(6.5, 53.5), new(11.5, 53.5),
        };
        var firstContourAndSpine = ClosedEdges(Rectangle(Radius, Radius, width - Radius, height - Radius))
            .Append((new PointD(width / 2, width / 2), new PointD(width / 2, height - width / 2)))
            .ToArray();
        foreach (var witness in capWitnesses)
        {
            double clearance = Math.Sqrt(ClosedEdges(region.Outer)
                .Min(wall => PointSegmentDistanceSquared(witness, wall.A, wall.B)));
            Assert.True(clearance >= Radius, $"Cap witness is not cutter-reachable: {witness}.");
            double distance = Math.Sqrt(firstContourAndSpine
                .Min(move => PointSegmentDistanceSquared(witness, move.Item1, move.Item2)));
            Assert.True(distance > Radius + .49,
                $"The first contour and exact medial spine unexpectedly cover cap witness ({witness.x:R},{witness.y:R}); distance={distance:R}.");
        }

        // At width18 the next nominal inset is line-valued, but collapsing it
        // directly to the medial spine leaves the independently witnessed cap
        // gaps above. Retain an earlier complete front instead of corner stabs.
        var depth = new DepthSchedule(2, 1);
        var plan = PocketToolpaths.Generate([region], ToolGeometry.Flat(Radius), depth,
            Options(insideOut) with { SpiralChaining = true, Ramp = RampSettings.FromRatio(4) });
        Report($"sharp caps insideOut={insideOut}", plan);

        Assert.Equal(2, plan.Diagnostics.RingCount);
        Assert.Equal(0, plan.Diagnostics.StripCount);
        Assert.Equal(depth.PassCount, plan.Diagnostics.LinearRampEntries
            + plan.Diagnostics.PlungeEntries + plan.Diagnostics.HelicalEntries);
        Assert.Equal(0, plan.Diagnostics.LinkLifts);
        AssertContinuousClearance(plan, region, Radius);
        for (int pass = 0; pass < depth.PassCount; pass++)
        {
            var path = Assert.Single(plan.Toolpaths.Where(path => path.DepthPassIndex == pass));
            Assert.Equal(depth.PassTopZ(pass), path.Points[0].Z, 9);
            Assert.Equal(depth.PassBottomZ(pass), path.Points[^1].Z, 9);
            AssertCovered(plan, capWitnesses.Concat(Grid(3.5, 14.5, 3.5, 56.5, .5)),
                depth.PassBottomZ(pass), $"sharp caps pass={pass}, insideOut={insideOut}");
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactToolWidthCorridorCutsOneOpenStripWithoutAConcentricRing(bool insideOut)
    {
        var region = new MillingRegion(Rectangle(0, 0, 6, 60));
        var plan = Generate(region, insideOut);
        Report($"exact tool width insideOut={insideOut}", plan);

        Assert.Equal(0, plan.Diagnostics.RingCount);
        var path = Assert.Single(plan.Toolpaths);
        var cut = Assert.Single(path.Spans.Where(span => span.Kind == ToolpathSpanKind.Cut));
        Assert.True(cut.MoveCount >= 1);
        Assert.All(path.Points, point => Assert.InRange(point.X, 3 - .001, 3 + .001));
        Assert.True(Math.Abs(path.Points[cut.EndIndex].Y - path.Points[cut.StartIndex].Y) >= 53.99,
            $"Exact-width strip did not traverse the corridor: first={path.Points[cut.StartIndex]}, last={path.Points[cut.EndIndex]}.");
        AssertContinuousClearance(plan, region, Radius);
        AssertCovered(plan, Grid(.5, 5.5, 3, 57, .5), -1,
            $"exact tool width, insideOut={insideOut}");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CorridorNarrowerThanTheToolRemainsUncut(bool insideOut)
    {
        var plan = Generate(new MillingRegion(Rectangle(0, 0, 5.9, 60)), insideOut);
        Report($"undersize corridor insideOut={insideOut}", plan);
        Assert.Empty(plan.Toolpaths);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StripOrderingFollowsInsideOutOrOutsideInAtEveryDepth(bool insideOut)
    {
        var region = new MillingRegion(Rectangle(0, 0, 16, 60));
        var depth = new DepthSchedule(2, 1);
        var plan = PocketToolpaths.Generate([region], ToolGeometry.Flat(Radius), depth, Options(insideOut));
        Report($"strip ordering insideOut={insideOut}", plan);

        for (int pass = 0; pass < depth.PassCount; pass++)
        {
            var layer = plan.Toolpaths.Select((path, index) => (Path: path, Index: index))
                .Where(item => item.Path.DepthPassIndex == pass).ToArray();
            int stripIndex = Assert.Single(layer.Where(item =>
                CentreTraverseLength(item.Path, 8, depth.PassBottomZ(pass)) >= 40)).Index;
            int wallIndex = Assert.Single(layer.Where(item =>
                item.Path.Points.Min(point => point.X) < 3.001
                && item.Path.Points.Max(point => point.X) > 12.999
                && item.Path.Points.Max(point => point.Y) - item.Path.Points.Min(point => point.Y) > 53.99)).Index;
            Assert.True(insideOut ? stripIndex < wallIndex : stripIndex > wallIndex,
                $"Wrong strip/ring ordering: insideOut={insideOut}, pass={pass}, stripIndex={stripIndex}, wallIndex={wallIndex}.");
            AssertCovered(plan, Grid(6.5, 9.5, 8, 52, .5), depth.PassBottomZ(pass),
                $"strip order pass={pass}, insideOut={insideOut}");
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RegionWideEnoughForAnotherRingRetainsThatNecessaryRing(bool insideOut)
    {
        var region = new MillingRegion(Rectangle(0, 0, 19, 60));
        var plan = Generate(region, insideOut);
        Report($"second ring fits insideOut={insideOut}", plan);

        Assert.Equal(2, plan.Diagnostics.RingCount);
        AssertContinuousClearance(plan, region, Radius);
        AssertCovered(plan, Grid(8, 11, 10, 50, .5), -1,
            $"second ring remains necessary, insideOut={insideOut}");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BentCorridorRetainsReachableStripsOnBothSidesOfTheBend(bool insideOut)
    {
        var region = new MillingRegion(new List<PointD>
        {
            new(0, 0), new(16, 0), new(16, 44), new(56, 44), new(56, 60), new(0, 60),
        });
        var plan = Generate(region, insideOut);
        Report($"bent corridor insideOut={insideOut}", plan);

        AssertContinuousClearance(plan, region, Radius);
        AssertCovered(plan,
            Grid(6.5, 9.5, 8, 44, .5).Concat(Grid(16, 48, 50.5, 53.5, .5)), -1,
            $"bent corridor, insideOut={insideOut}");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CorridorOpeningIntoARoomKeepsTheStemAndRoomBranchesReachable(bool insideOut)
    {
        var region = new MillingRegion(new List<PointD>
        {
            new(0, 0), new(16, 0), new(16, 44), new(32, 44), new(32, 76),
            new(-16, 76), new(-16, 44), new(0, 44),
        });
        var plan = Generate(region, insideOut);
        Report($"branched room insideOut={insideOut}", plan);

        AssertContinuousClearance(plan, region, Radius);
        AssertCovered(plan,
            Grid(6.5, 9.5, 8, 36, .5).Concat(Grid(-7, 23, 58.5, 61.5, .5)), -1,
            $"corridor with room branches, insideOut={insideOut}");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StripCompletionOnBothSidesOfAnIslandPreservesItsEntireBoundary(bool insideOut)
    {
        var hole = Rectangle(13, 20, 19, 40);
        var region = new MillingRegion(Rectangle(0, 0, 32, 60), new[] { hole });
        var plan = Generate(region, insideOut);
        Report($"protected island insideOut={insideOut}", plan);

        AssertContinuousClearance(plan, region, Radius);
        AssertCovered(plan,
            Grid(6.25, 6.75, 25, 35, .25).Concat(Grid(25.25, 25.75, 25, 35, .25)), -1,
            $"protected island, insideOut={insideOut}");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void StripCompletionRespectsStockToLeaveAndTheClipBoundary(bool insideOut)
    {
        var region = new MillingRegion(Rectangle(0, 0, 18, 60));
        var clip = Rectangle(0, 10, 18, 50);
        var plan = Generate(region, insideOut, options => options with
        {
            StockToLeave = 1,
            ClipBoundary = clip,
        });
        Report($"stock and clipping insideOut={insideOut}", plan);

        AssertContinuousClearance(plan, region, Radius + 1);
        Assert.All(plan.Toolpaths.SelectMany(path => path.Points), point =>
        {
            Assert.InRange(point.X, 0, 18);
            Assert.InRange(point.Y, 10 - .000002, 50 + .000002);
        });
        AssertCovered(plan, Grid(7.5, 10.5, 13, 47, .5), -1,
            $"stockToLeave=1 and clip y=10..50, insideOut={insideOut}");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactWidthStripReachesEveryDepthAndRampsFromTheKnownFloor(bool insideOut)
    {
        var region = new MillingRegion(Rectangle(0, 0, 6, 60));
        var depth = new DepthSchedule(4, 1) { SurfaceZ = 11, StartDepth = 1.25 };
        var plan = PocketToolpaths.Generate([region], ToolGeometry.Flat(Radius), depth,
            Options(insideOut) with { Ramp = RampSettings.FromRatio(4) });
        Report($"exact-width depth/ramp insideOut={insideOut}", plan);

        Assert.Equal(3, plan.Diagnostics.DepthPassCount);
        Assert.Equal(new[] { 0, 1, 2 }, plan.Toolpaths
            .Select(path => path.DepthPassIndex!.Value).Distinct().Order().ToArray());
        AssertContinuousClearance(plan, region, Radius);
        Assert.All(plan.Toolpaths.SelectMany(path => path.Points), point =>
            Assert.InRange(point.Z, 7 - 1e-9, 9.75 + 1e-9));

        for (int pass = 0; pass < depth.PassCount; pass++)
        {
            var paths = plan.Toolpaths.Where(path => path.DepthPassIndex == pass).ToArray();
            var ramps = paths.SelectMany(path => path.Spans
                .Where(span => span.Kind == ToolpathSpanKind.Ramp)
                .Select(span => (Path: path, Span: span))).ToArray();
            Assert.NotEmpty(ramps);
            Assert.Equal(depth.PassTopZ(pass), ramps[0].Path.Points[ramps[0].Span.StartIndex].Z, 9);
            Assert.Equal(depth.PassBottomZ(pass), ramps[0].Path.Points[ramps[0].Span.EndIndex].Z, 9);
            foreach (var (path, ramp) in ramps)
            for (int index = ramp.StartIndex + 1; index <= ramp.EndIndex; index++)
            {
                Point3D a = path.Points[index - 1], b = path.Points[index];
                double travel = Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
                Assert.True(travel > 1e-12, $"Vertical strip ramp at pass={pass}: {a} -> {b}.");
                Assert.InRange(a.Z - b.Z, -1e-9, travel / 4 + 1e-8);
            }
            AssertCovered(plan, Grid(.5, 5.5, 3, 57, .5), depth.PassBottomZ(pass),
                $"exact-width depth pass={pass}, insideOut={insideOut}");
        }
    }

    private static ToolpathPlan Generate(MillingRegion region, bool insideOut,
        Func<PocketOptions, PocketOptions>? adapt = null)
    {
        PocketOptions options = Options(insideOut);
        return PocketToolpaths.Generate([region], ToolGeometry.Flat(Radius), new DepthSchedule(1, 1),
            adapt is null ? options : adapt(options));
    }

    private static PocketOptions Options(bool insideOut) => new()
    {
        StepOver = 1,
        InsideOut = insideOut,
        SpiralChaining = false,
        Ramp = RampSettings.Plunge,
        // Ring arcs already allow their chord tolerance. Keeping that tolerance below
        // the independent clearance slack lets one oracle cover both rings and strips.
        Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = 0 },
        DetectCurves = false,
        CalculateSweptArea = false,
    };

    private void Report(string fixture, ToolpathPlan plan)
    {
        _output.WriteLine($"{fixture}: paths={plan.Toolpaths.Count}, rings={plan.Diagnostics.RingCount}, strips={plan.Diagnostics.StripCount}, "
            + $"passes={plan.Diagnostics.DepthPassCount}, points={plan.Toolpaths.Sum(path => path.Points.Count)}.");
        foreach (var path in plan.Toolpaths)
            _output.WriteLine($"  pass={path.DepthPassIndex}, points={path.Points.Count}, "
                + $"first={path.Points[0]}, last={path.Points[^1]}, "
                + $"spans={string.Join(',', path.Spans.Select(span => $"{span.Kind}:{span.StartIndex}..{span.EndIndex}"))}");
    }

    private static void AssertCovered(ToolpathPlan plan, IEnumerable<PointD> witnesses, double floorZ, string fixture)
    {
        var moves = CuttingMoves(plan, floorZ).ToArray();
        Assert.NotEmpty(moves);
        foreach (PointD witness in witnesses)
        {
            double distance = Math.Sqrt(moves.Min(move => PointSegmentDistanceSquared(witness, move.A, move.B)));
            Assert.True(distance <= Radius + .002,
                $"Reachable material remains: {fixture}, witness=({witness.x:R},{witness.y:R}), "
                + $"floorZ={floorZ:R}, nearest cut centre={distance:R}, toolRadius={Radius:R}.");
        }
    }

    private static void AssertContinuousClearance(ToolpathPlan plan, MillingRegion region, double required)
    {
        var source = new[] { region.Outer }.Concat(region.Holes).ToArray();
        var walls = source.SelectMany(ClosedEdges).ToArray();
        double requiredSquared = Math.Pow(required - ClearanceSlack, 2);
        foreach (var path in plan.Toolpaths)
        {
            Assert.NotEmpty(path.Points);
            foreach (Point3D point in path.Points)
                Assert.True(Contains(source, new PointD(point.X, point.Y)),
                    $"Cutter centre left the pocket or entered its island: {point}.");
            for (int index = 1; index < path.Points.Count; index++)
            {
                Point3D first = path.Points[index - 1], last = path.Points[index];
                var a = new PointD(first.X, first.Y);
                var b = new PointD(last.X, last.Y);
                foreach (var (c, d) in walls)
                {
                    double actualSquared = SegmentDistanceSquared(a, b, c, d);
                    Assert.True(actualSquared >= requiredSquared,
                        $"Insufficient complete-segment wall/island clearance: pass={path.DepthPassIndex}, "
                        + $"move=({a.x:R},{a.y:R})->({b.x:R},{b.y:R}), "
                        + $"boundary=({c.x:R},{c.y:R})->({d.x:R},{d.y:R}), "
                        + $"required={required:R}, actual={Math.Sqrt(actualSquared):R}.");
                }
            }
        }
    }

    private static IEnumerable<(PointD A, PointD B)> CuttingMoves(ToolpathPlan plan, double floorZ)
    {
        foreach (var path in plan.Toolpaths)
        foreach (var span in path.Spans.Where(span => span.Kind is ToolpathSpanKind.Cut or ToolpathSpanKind.Link))
        for (int index = span.StartIndex + 1; index <= span.EndIndex; index++)
        {
            Point3D a = path.Points[index - 1], b = path.Points[index];
            if (Math.Abs(a.Z - floorZ) < 1e-8 && Math.Abs(b.Z - floorZ) < 1e-8)
                yield return (new PointD(a.X, a.Y), new PointD(b.X, b.Y));
        }
    }

    private static double CentreTraverseLength(TaggedToolpath path, double centreX, double floorZ)
    {
        double length = 0;
        foreach (var span in path.Spans.Where(span => span.Kind == ToolpathSpanKind.Cut))
        for (int index = span.StartIndex + 1; index <= span.EndIndex; index++)
        {
            Point3D a = path.Points[index - 1], b = path.Points[index];
            if (Math.Abs(a.X - centreX) < .001 && Math.Abs(b.X - centreX) < .001
                && Math.Abs(a.Z - floorZ) < 1e-8 && Math.Abs(b.Z - floorZ) < 1e-8)
                length += Math.Abs(b.Y - a.Y);
        }
        return length;
    }

    private static IEnumerable<PointD> Grid(double minX, double maxX, double minY, double maxY, double step)
    {
        for (double y = minY; y <= maxY + 1e-9; y += step)
        for (double x = minX; x <= maxX + 1e-9; x += step)
            yield return new PointD(x, y);
    }

    private static List<PointD> Rectangle(double minX, double minY, double maxX, double maxY) =>
        [new(minX, minY), new(maxX, minY), new(maxX, maxY), new(minX, maxY)];

    private static IEnumerable<(PointD A, PointD B)> ClosedEdges(IReadOnlyList<PointD> path)
    {
        for (int index = 0; index < path.Count; index++)
            yield return (path[index], path[(index + 1) % path.Count]);
    }

    private static bool Contains(IEnumerable<IReadOnlyList<PointD>> contours, PointD point)
    {
        bool inside = false;
        foreach (var contour in contours)
        foreach (var (a, b) in ClosedEdges(contour))
            if ((a.y > point.y) != (b.y > point.y)
                && point.x < a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y))
                inside = !inside;
        return inside;
    }

    private static double SegmentDistanceSquared(PointD a, PointD b, PointD c, PointD d)
    {
        double abx = b.x - a.x, aby = b.y - a.y, cdx = d.x - c.x, cdy = d.y - c.y;
        double denominator = abx * cdy - aby * cdx;
        if (Math.Abs(denominator) > 1e-20)
        {
            double acx = c.x - a.x, acy = c.y - a.y;
            double t = (acx * cdy - acy * cdx) / denominator;
            double u = (acx * aby - acy * abx) / denominator;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
                return 0;
        }
        return Math.Min(Math.Min(PointSegmentDistanceSquared(a, c, d), PointSegmentDistanceSquared(b, c, d)),
            Math.Min(PointSegmentDistanceSquared(c, a, b), PointSegmentDistanceSquared(d, a, b)));
    }

    private static double PointSegmentDistanceSquared(PointD point, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y;
        double lengthSquared = dx * dx + dy * dy;
        double t = lengthSquared == 0 ? 0
            : Math.Clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / lengthSquared, 0, 1);
        double x = point.x - a.x - t * dx, y = point.y - a.y - t * dy;
        return x * x + y * y;
    }
}
