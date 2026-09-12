using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public class DenseOffsetCoverageTests
{
    private const double FirstInset = 3.1765567;
    private const double Stepover = 2.5412454;
    private const double ArcTolerance = 0.004174415;
    // The established offset proof allows 5% of arc tolerance plus integer rounding.
    private const double ClearanceSlack = ArcTolerance * 0.05 + 0.000002;

    [Theory]
    [InlineData(0, 10240, 20, 160)]
    [InlineData(113, 10240, 20, 160)]
    [InlineData(0, 2560, 40, 40)]
    [InlineData(0, 5120, 20, 80)]
    public void DenseNoisyPocketRetainsReachableRoomsAcrossInsetLevelsAndChunkSeams(
        int rotation, int outerPoints, int holeCount, int holePoints)
    {
        var (outer, holes, holeCenters) = CreatePocket(outerPoints, holeCount, holePoints);
        outer = outer.Skip(rotation).Concat(outer.Take(rotation)).ToList();
        var source = new[] { outer }.Concat(holes).ToArray();
        var segments = source.SelectMany(Edges).ToArray();
        var prepared = PolygonOps.PrepareBoundaryOffset(outer, holes, ArcTolerance, simplifyTolerance: 0);
        double previousArea = double.PositiveInfinity;

        for (int level = 0; level < 4; level++)
        {
            double inset = FirstInset + Stepover * level;
            var contours = prepared.Execute(-inset);
            double area = Math.Abs(contours.Sum(PolygonOps.SignedArea));
            Assert.True(area > 1000, $"Level {level} lost a large, reachable pocket: area={area:R}.");
            Assert.True(area < previousArea, $"Increasing inset grew the pocket: level={level}, previous={previousArea:R}, area={area:R}.");
            previousArea = area;

            // A witness with this source-boundary clearance admits the entire cutter disc.
            // This checks coverage independently of the offsetter, including separate rooms
            // left between the off-center ring of islands and the corrugated outer wall.
            int reachableWitnesses = 0;
            for (int y = -40; y <= 40; y += 10)
            for (int x = -40; x <= 40; x += 10)
            {
                var witness = new PointD(x, y);
                if (!Contains(source, witness) || PointClearanceSquared(segments, witness) < Math.Pow(inset + 0.1, 2))
                    continue;
                reachableWitnesses++;
                Assert.True(Contains(contours, witness),
                    $"Reachable room disappeared: level={level}, witness=({x},{y}), inset={inset:R}.");
            }
            Assert.True(reachableWitnesses >= 9);
            foreach (var center in holeCenters)
                Assert.False(Contains(contours, center), $"Protected island was filled at level {level}.");

            AssertContinuousClearance(contours, source, segments, inset - ClearanceSlack, level);
        }
    }

    [Fact]
    public void LargeLateInsetsRetainReachableMaterialAndProtectIslands()
    {
        var (outer, holes, holeCenters) = CreatePocket();
        List<PointD> Enlarge(List<PointD> path) => path.Select(p => new PointD(p.x * 3, p.y * 3)).ToList();
        outer = Enlarge(outer);
        holes = holes.Select(Enlarge).ToList();
        holeCenters = Enlarge(holeCenters);
        var source = new[] { outer }.Concat(holes).ToArray();
        var segments = source.SelectMany(Edges).ToArray();
        var prepared = PolygonOps.PrepareBoundaryOffset(outer, holes, ArcTolerance, simplifyTolerance: 0);
        double previousArea = double.PositiveInfinity;
        foreach (int level in new[] { 0, 5, 17 })
        {
            double inset = FirstInset + Stepover * level;
            var contours = prepared.Execute(-inset);
            double area = contours.Sum(PolygonOps.SignedArea);
            Assert.True(area > 1000 && area < previousArea, $"Invalid area at level {level}: {area:R}");
            previousArea = area;
            int witnesses = 0;
            for (int y = -60; y <= 60; y += 20)
            for (int x = -60; x <= 60; x += 20)
            {
                var p = new PointD(x, y);
                if (!Contains(source, p) || PointClearanceSquared(segments, p) < Math.Pow(inset + 0.1, 2)) continue;
                witnesses++;
                Assert.True(Contains(contours, p), $"Lost reachable room at level {level}: ({x},{y})");
            }
            Assert.True(witnesses >= 9);
            foreach (var center in holeCenters) Assert.False(Contains(contours, center));
            AssertContinuousClearance(contours, source, segments, inset - ClearanceSlack, level);
        }
    }

    private static (List<PointD> Outer, List<List<PointD>> Holes, List<PointD> Centers) CreatePocket(
        int outerPoints = 10240, int holeCount = 20, int holePoints = 160)
    {
        var random = new Random(0);
        var outer = new List<PointD>(outerPoints);
        for (int i = 0; i < outerPoints; i++)
        {
            double a = 2 * Math.PI * i / outerPoints;
            double r = 55 + 0.05 * (2 * random.NextDouble() - 1) + 7 * Math.Sin(7 * a);
            outer.Add(new PointD(r * Math.Cos(a), r * Math.Sin(a)));
        }

        var holes = new List<List<PointD>>();
        var centers = new List<PointD>();
        for (int h = 0; h < holeCount; h++)
        {
            double a = 2 * Math.PI * h / holeCount;
            var center = new PointD(31 * Math.Cos(a) + 3, 31 * Math.Sin(a) - 2);
            var hole = new List<PointD>(holePoints);
            for (int i = 0; i < holePoints; i++)
            {
                double b = 2 * Math.PI * i / holePoints;
                double r = 1.4 + 0.1 * (2 * random.NextDouble() - 1);
                hole.Add(new PointD(center.x + r * Math.Cos(b), center.y + r * Math.Sin(b)));
            }
            holes.Add(hole);
            centers.Add(center);
        }
        return (outer, holes, centers);
    }

    private static void AssertContinuousClearance(
        IReadOnlyList<List<PointD>> contours,
        IReadOnlyList<List<PointD>> source,
        (PointD A, PointD B)[] sourceSegments,
        double required,
        int level)
    {
        double requiredSquared = required * required;
        foreach (var contour in contours)
        {
            Assert.True(Contains(source, contour[0]), $"Inset left the source pocket at level {level}.");
            foreach (var (a, b) in Edges(contour))
            foreach (var (c, d) in sourceSegments)
            {
                // Cheap independent AABB rejection; every remaining pair is checked over
                // both complete line segments, rather than checking only contour vertices.
                double dx = Math.Max(0, Math.Max(Math.Min(a.x, b.x) - Math.Max(c.x, d.x), Math.Min(c.x, d.x) - Math.Max(a.x, b.x)));
                double dy = Math.Max(0, Math.Max(Math.Min(a.y, b.y) - Math.Max(c.y, d.y), Math.Min(c.y, d.y) - Math.Max(a.y, b.y)));
                if (dx * dx + dy * dy >= requiredSquared)
                    continue;
                double clearanceSquared = SegmentDistanceSquared(a, b, c, d);
                if (!(clearanceSquared >= requiredSquared))
                    Assert.Fail(
                        $"Cutter crosses the clearance envelope at level {level}: required={required:R}, actual={Math.Sqrt(clearanceSquared):R}, segment=({a.x:R},{a.y:R})->({b.x:R},{b.y:R}).");
            }
        }
    }

    private static IEnumerable<(PointD A, PointD B)> Edges(IReadOnlyList<PointD> path)
    {
        for (int i = 0; i < path.Count; i++)
            yield return (path[i], path[(i + 1) % path.Count]);
    }

    private static double PointClearanceSquared((PointD A, PointD B)[] source, PointD p)
        => source.Min(segment => PointSegmentDistanceSquared(p, segment.A, segment.B));

    private static double SegmentDistanceSquared(PointD a, PointD b, PointD c, PointD d)
    {
        double abx = b.x - a.x, aby = b.y - a.y, cdx = d.x - c.x, cdy = d.y - c.y;
        double denominator = abx * cdy - aby * cdx;
        if (Math.Abs(denominator) > 1e-20)
        {
            double acx = c.x - a.x, acy = c.y - a.y;
            double t = (acx * cdy - acy * cdx) / denominator;
            double u = (acx * aby - acy * abx) / denominator;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return 0;
        }
        return Math.Min(Math.Min(PointSegmentDistanceSquared(a, c, d), PointSegmentDistanceSquared(b, c, d)),
            Math.Min(PointSegmentDistanceSquared(c, a, b), PointSegmentDistanceSquared(d, a, b)));
    }

    private static double PointSegmentDistanceSquared(PointD p, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y;
        double lengthSquared = dx * dx + dy * dy;
        double t = lengthSquared == 0 ? 0 : Math.Clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSquared, 0, 1);
        double x = p.x - a.x - t * dx, y = p.y - a.y - t * dy;
        return x * x + y * y;
    }

    private static bool Contains(IEnumerable<IReadOnlyList<PointD>> contours, PointD p)
    {
        bool inside = false;
        foreach (var contour in contours)
        foreach (var (a, b) in Edges(contour))
            if ((a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y))
                inside = !inside;
        return inside;
    }
}
