using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class OffsetClearanceRepairTests
{
    [Theory]
    [InlineData(-2)]
    [InlineData(2)]
    public void LocalClearanceRepairPreservesDistantBoundariesAndProtectedHole(double delta)
    {
        var source = new Paths64 { Rectangle(0, 40), Rectangle(15, 25, hole: true) };
        double min = -delta, max = 40 + delta;
        // Model the observed union defect: one invalid indentation remains next to a
        // source edge while the distant portion of this connected contour is valid.
        double badX = delta < 0 ? 1.8 : -1.8;
        var candidate = new Paths64
        {
            new() { Point(min, min), Point(max, min), Point(max, max), Point(min, max), Point(badX, 20) },
            Rectangle(15 + delta, 25 - delta, hole: true)
        };
        var repaired = PolygonOps.RepairConservativeOffset(new PolygonOps.BoundarySegmentIndex(source),
            source, candidate, delta * PathUtils.Scale, .01, .2 * PathUtils.Scale);

        Assert.Equal(2, repaired.Count);
        Assert.False(Contains(repaired, Point(20, 20)));
        Assert.Equal(max, repaired.SelectMany(p => p).Max(p => p.X) / PathUtils.Scale, 8);
        Assert.True(Contains(repaired, Point(max - .001, 20)));
        Assert.Equal(delta > 0, Contains(repaired, Point(delta > 0 ? -1.9 : 1.9, 20)));

        // Independent full segment-to-segment distance oracle, including the closing
        // edges, verifies physical clearance rather than only the repair's vertices.
        double required = 2 - .0005 - 2 / PathUtils.Scale;
        foreach (var a in repaired)
        foreach (var (a0, a1) in Edges(a))
        foreach (var b in source)
        foreach (var (b0, b1) in Edges(b))
            Assert.True(SegmentDistance(a0, a1, b0, b1) >= required,
                $"Insufficient repaired clearance for delta={delta}: ({a0.X},{a0.Y})->({a1.X},{a1.Y}).");
    }

    private static Point64 Point(double x, double y) =>
        new((long)Math.Round(x * PathUtils.Scale), (long)Math.Round(y * PathUtils.Scale));

    private static Path64 Rectangle(double min, double max, bool hole = false)
    {
        var path = new Path64 { Point(min, min), Point(max, min), Point(max, max), Point(min, max) };
        if (hole) path.Reverse();
        return path;
    }

    private static bool Contains(Paths64 paths, Point64 point) =>
        paths.Count(path => Clipper.PointInPolygon(point, path) != PointInPolygonResult.IsOutside) % 2 != 0;

    private static IEnumerable<(Point64 A, Point64 B)> Edges(Path64 path)
    {
        for (int i = 0; i < path.Count; i++) yield return (path[i], path[(i + 1) % path.Count]);
    }

    private static double SegmentDistance(Point64 a, Point64 b, Point64 c, Point64 d)
    {
        var p = new PointD(a.X / PathUtils.Scale, a.Y / PathUtils.Scale);
        var q = new PointD(b.X / PathUtils.Scale, b.Y / PathUtils.Scale);
        var r = new PointD(c.X / PathUtils.Scale, c.Y / PathUtils.Scale);
        var s = new PointD(d.X / PathUtils.Scale, d.Y / PathUtils.Scale);
        double abx = q.x - p.x, aby = q.y - p.y, cdx = s.x - r.x, cdy = s.y - r.y;
        double det = abx * cdy - aby * cdx;
        if (Math.Abs(det) > 1e-20)
        {
            double acx = r.x - p.x, acy = r.y - p.y;
            double t = (acx * cdy - acy * cdx) / det;
            double u = (acx * aby - acy * abx) / det;
            if (t is >= 0 and <= 1 && u is >= 0 and <= 1) return 0;
        }
        static double PointDistance(PointD point, PointD start, PointD end)
        {
            double dx = end.x - start.x, dy = end.y - start.y;
            double length2 = dx * dx + dy * dy;
            double t = length2 == 0 ? 0 : Math.Clamp(((point.x - start.x) * dx + (point.y - start.y) * dy) / length2, 0, 1);
            double x = point.x - start.x - t * dx, y = point.y - start.y - t * dy;
            return Math.Sqrt(x * x + y * y);
        }
        return Math.Min(Math.Min(PointDistance(p, r, s), PointDistance(q, r, s)),
            Math.Min(PointDistance(r, p, q), PointDistance(s, p, q)));
    }
}
