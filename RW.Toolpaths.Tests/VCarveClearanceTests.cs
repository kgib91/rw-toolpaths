using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveClearanceTests
{
    private const double BottomRadius = 0.05;
    private const double RadiusSlope = 0.3125;

    [Theory]
    [InlineData(0.2)]
    [InlineData(2.0)]
    [InlineData(3.5)]
    public void ObservedTinyContourNeverReappearsAsAnInvertedDeepClearingLoop(double depthPerPass)
    {
        IReadOnlyList<PointD>[] boundary = [ObservedTinyContour()];
        var paths = Clearing(boundary, depthPerPass);
        if (depthPerPass < 0.3)
            Assert.NotEmpty(paths); // The shallow, physically reachable pass survives.
        else
            Assert.Empty(paths);
        AssertClearingSweepsContained(boundary, paths);
    }

    [Fact]
    public void CollapsedClearingKeepsReachableShallowMedialDetail()
    {
        var contour = ObservedTinyContour();
        var plan = VCarveToolpaths.Generate(
            [new MillingRegion(contour)],
            new ToolGeometry(3.175)
            {
                BottomRadius = BottomRadius, TopRadius = 3.175,
                ConeLength = 10, TipAngleRadians = 2 * Math.Atan(RadiusSlope)
            },
            new DepthSchedule(6.35, 3.5),
            new VCarveOptions { IncludeInteriorFill = false, DetectCurves = false },
            BoostVoronoiProvider.CreateDefault());

        Assert.DoesNotContain(plan.Toolpaths, path => path.Category == "clearing");
        var detail = plan.Toolpaths.Where(path => path.Category == "final-carve").ToArray();
        Assert.NotEmpty(detail);
        Assert.Contains(detail.SelectMany(path => path.Points), point => point.Z < -0.01);
        double maximumPossibleDepth = ((contour.Max(p => p.x) - contour.Min(p => p.x)) / 2 - BottomRadius) / RadiusSlope;
        Assert.All(detail.SelectMany(path => path.Points),
            point => Assert.InRange(-point.Z, 0, maximumPossibleDepth + 0.001));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TinyComponentDoesNotRemoveReachableNeighborsOrProtectedHoles(bool includeInteriorFill)
    {
        // A large ring has a protected hole with an independently reachable nested island.
        // The observed tiny component is disconnected and must vanish only at its own limit.
        IReadOnlyList<PointD>[] boundary =
        [
            Rectangle(0, 0, 30, 30),
            Rectangle(5, 5, 25, 25).AsEnumerable().Reverse().ToArray(),
            Rectangle(10, 10, 20, 20),
            ObservedTinyContour()
        ];
        var paths = Clearing(boundary, 2, includeInteriorFill);
        Assert.NotEmpty(paths);
        Assert.Contains(paths, path => path.Points.All(point => point.X > 10 && point.X < 20));
        Assert.Contains(paths, path => path.Points.Any(point => point.X < 5));
        Assert.DoesNotContain(paths, path => path.Points.Any(point => point.X > 200));
        AssertClearingSweepsContained(boundary, paths);
    }

    private static List<TaggedToolpath> Clearing(
        IReadOnlyList<IReadOnlyList<PointD>> boundary, double depthPerPass, bool includeInteriorFill = false)
        => MedialAxisToolpaths.GenerateClearingPassesTagged(
            boundary, 0, 6.35, 2 * Math.Atan(RadiusSlope), depthPerPass,
            bottomRadiusOverride: BottomRadius, topRadiusOverride: 3.175, coneLengthOverride: 10,
            includeInteriorFill: includeInteriorFill, detectCurves: false);

    private static void AssertClearingSweepsContained(
        IReadOnlyList<IReadOnlyList<PointD>> boundary, IReadOnlyList<TaggedToolpath> paths)
    {
        var rings = boundary.Select(ring => new PathD(ring)).ToArray();
        foreach (var path in paths)
        for (int index = 1; index < path.Points.Count; index++)
        {
            var a = path.Points[index - 1];
            var b = path.Points[index];
            Assert.Equal(a.Z, b.Z);
            var start = new PointD(a.X, a.Y);
            var end = new PointD(b.X, b.Y);
            int winding = rings.Sum(ring => Clipper.PointInPolygon(start, ring) == PointInPolygonResult.IsOutside
                ? 0 : Math.Sign(Clipper.Area(ring)));
            Assert.NotEqual(0, winding);
            double clearance = double.PositiveInfinity;
            foreach (var ring in rings)
            for (int j = 0; j < ring.Count; j++)
                clearance = Math.Min(clearance, SegmentDistance(start, end, ring[j], ring[(j + 1) % ring.Count]));
            double required = BottomRadius - a.Z * RadiusSlope;
            Assert.True(clearance + 0.01 >= required,
                $"Cutter footprint crosses the boundary: clearance={clearance:R}, radius={required:R}, depth={-a.Z:R}, start=({a.X:R},{a.Y:R}).");
        }
    }

    private static double SegmentDistance(PointD a, PointD b, PointD c, PointD d)
    {
        static double Cross(PointD p, PointD q, PointD r)
            => (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
        if (Cross(a, b, c) * Cross(a, b, d) < 0 && Cross(c, d, a) * Cross(c, d, b) < 0)
            return 0;
        static double Distance(PointD p, PointD s, PointD e)
        {
            double dx = e.x - s.x, dy = e.y - s.y;
            double lengthSquared = dx * dx + dy * dy;
            double t = lengthSquared == 0 ? 0 : Math.Clamp(((p.x - s.x) * dx + (p.y - s.y) * dy) / lengthSquared, 0, 1);
            double x = p.x - s.x - t * dx, y = p.y - s.y - t * dy;
            return Math.Sqrt(x * x + y * y);
        }
        return Math.Min(Math.Min(Distance(a, c, d), Distance(b, c, d)),
            Math.Min(Distance(c, a, b), Distance(d, a, b)));
    }

    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];

    // The exact quantized 0.3167 x 0.5338 mm contour from the reported artwork.
    // Its unvalidated 3.5 mm clearing pass inverted into a loop almost 2 mm outside.
    private static List<PointD> ObservedTinyContour() => new()
    {
        new(211.37313842773438, 74.59634399414062),
        new(211.38323974609375, 74.59634399414062),
        new(211.39337158203125, 74.59634399414062),
        new(211.40347290039062, 74.599609375),
        new(211.41357421875, 74.60284423828125),
        new(211.42367553710938, 74.60610961914062),
        new(211.43377685546875, 74.61260986328125),
        new(211.447265625, 74.619140625),
        new(211.45736694335938, 74.62564086914062),
        new(211.46746826171875, 74.63540649414062),
        new(211.48095703125, 74.64517211914062),
        new(211.49105834960938, 74.658203125),
        new(211.50115966796875, 74.66796875),
        new(211.51129150390625, 74.68096923828125),
        new(211.52139282226562, 74.69400024414062),
        new(211.52813720703125, 74.70703125),
        new(211.53485107421875, 74.72003173828125),
        new(211.54159545898438, 74.73306274414062),
        new(211.54833984375, 74.74609375),
        new(211.55508422851562, 74.76235961914062),
        new(211.55844116210938, 74.775390625),
        new(211.56182861328125, 74.79165649414062),
        new(211.565185546875, 74.8046875),
        new(211.56854248046875, 74.81768798828125),
        new(211.57192993164062, 74.833984375),
        new(211.57192993164062, 74.84698486328125),
        new(211.57192993164062, 74.86328125),
        new(211.57192993164062, 74.87954711914062),
        new(211.57192993164062, 74.892578125),
        new(211.56854248046875, 74.90884399414062),
        new(211.565185546875, 74.921875),
        new(211.56182861328125, 74.93487548828125),
        new(211.55844116210938, 74.951171875),
        new(211.55508422851562, 74.96417236328125),
        new(211.54833984375, 74.98046875),
        new(211.54159545898438, 74.99346923828125),
        new(211.53485107421875, 75.00650024414062),
        new(211.52813720703125, 75.01953125),
        new(211.52139282226562, 75.03253173828125),
        new(211.51129150390625, 75.04556274414062),
        new(211.50115966796875, 75.05859375),
        new(211.49105834960938, 75.068359375),
        new(211.48095703125, 75.08135986328125),
        new(211.46746826171875, 75.09112548828125),
        new(211.45736694335938, 75.10089111328125),
        new(211.447265625, 75.107421875),
        new(211.43377685546875, 75.11392211914062),
        new(211.42367553710938, 75.12042236328125),
        new(211.41357421875, 75.12368774414062),
        new(211.40347290039062, 75.126953125),
        new(211.39337158203125, 75.13018798828125),
        new(211.38323974609375, 75.13018798828125),
        new(211.37313842773438, 75.13018798828125),
        new(211.363037109375, 75.13018798828125),
        new(211.35293579101562, 75.126953125),
        new(211.34619140625, 75.12368774414062),
        new(211.33609008789062, 75.1171875),
        new(211.329345703125, 75.11392211914062),
        new(211.32260131835938, 75.107421875),
        new(211.3125, 75.09765625),
        new(211.30575561523438, 75.087890625),
        new(211.29901123046875, 75.078125),
        new(211.29229736328125, 75.068359375),
        new(211.28890991210938, 75.05532836914062),
        new(211.28216552734375, 75.04229736328125),
        new(211.27880859375, 75.029296875),
        new(211.27206420898438, 75.01626586914062),
        new(211.26870727539062, 75),
        new(211.26531982421875, 74.98370361328125),
        new(211.261962890625, 74.96417236328125),
        new(211.25860595703125, 74.94464111328125),
        new(211.25860595703125, 74.92510986328125),
        new(211.25521850585938, 74.90557861328125),
        new(211.25521850585938, 74.88604736328125),
        new(211.25521850585938, 74.86328125),
        new(211.25521850585938, 74.84048461914062),
        new(211.25521850585938, 74.82095336914062),
        new(211.25860595703125, 74.80142211914062),
        new(211.25860595703125, 74.78189086914062),
        new(211.261962890625, 74.76235961914062),
        new(211.26531982421875, 74.74282836914062),
        new(211.26870727539062, 74.7265625),
        new(211.27206420898438, 74.71026611328125),
        new(211.27880859375, 74.697265625),
        new(211.28216552734375, 74.68423461914062),
        new(211.28890991210938, 74.67120361328125),
        new(211.29229736328125, 74.658203125),
        new(211.29901123046875, 74.6484375),
        new(211.30575561523438, 74.638671875),
        new(211.3125, 74.62890625),
        new(211.32260131835938, 74.619140625),
        new(211.329345703125, 74.61260986328125),
        new(211.33609008789062, 74.609375),
        new(211.34619140625, 74.60284423828125),
        new(211.35293579101562, 74.599609375),
        new(211.363037109375, 74.59634399414062),
    };
}

