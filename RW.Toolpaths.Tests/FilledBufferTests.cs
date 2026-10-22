using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class FilledBufferTests
{
    [Fact]
    public void DilationGrowsTheOuterAndShrinksItsSignedHoleAsOneFilledGroup()
    {
        var outer = Circle(20);
        var hole = Circle(10);
        hole.Reverse();
        var dilated = PolygonOps.Buffer(new[] { outer, hole }, 2, .001);

        Assert.Equal(2, dilated.Count);
        var outside = Assert.Single(dilated.Where(path => PolygonOps.SignedArea(path) > 0));
        var inside = Assert.Single(dilated.Where(path => PolygonOps.SignedArea(path) < 0));
        Assert.All(outside, point => Assert.InRange(Math.Abs(Math.Sqrt(point.x * point.x + point.y * point.y) - 22), 0, .003));
        Assert.All(inside, point => Assert.InRange(Math.Abs(Math.Sqrt(point.x * point.x + point.y * point.y) - 8), 0, .003));
    }

    private static List<PointD> Circle(double radius) => Enumerable.Range(0, 512).Select(index =>
    {
        double angle = index * Math.PI * 2 / 512;
        return new PointD(radius * Math.Cos(angle), radius * Math.Sin(angle));
    }).ToList();
}
