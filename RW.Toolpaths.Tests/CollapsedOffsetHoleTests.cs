using Clipper2Lib;
using RW.Toolpaths.Geometry;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class CollapsedOffsetHoleTests
{
    [Theory]
    [InlineData(1.0)]
    [InlineData(2.0)]
    [InlineData(4.0)]
    public void CertifiedCollapsedHolesRetainTheLiteralPositiveOffset(double radius)
    {
        var source = new List<IReadOnlyList<PointD>> { Circle(0, 0, 100, 5200) };
        for (int row = -4; row <= 4; row++)
        for (int column = -4; column <= 4; column++)
            source.Add(Circle(column * 12, row * 12, 0.5 + Math.Abs(row) * .65, 40).Reverse().ToArray());
        // A filled island nested inside a disappearing hole must also survive
        // the same complete signed fill, rather than being treated as waste.
        source.Add(Circle(0, 0, .2, 20));
        Exact(source, radius);
    }

    [Fact]
    public void NegativeDisjointFillAndUnresolvedTangencyKeepTheirLiteralOperands()
    {
        var source = new List<IReadOnlyList<PointD>> { Circle(0, 0, 20, 5200),
            Circle(0, 0, 2, 40).Reverse().ToArray(),
            Circle(50, 0, .4, 40).Reverse().ToArray() };
        Exact(source, 2);
    }

    private static void Exact(IEnumerable<IReadOnlyList<PointD>> source, double radius)
    {
        var expected = PolygonOps.Offset(source, radius, .0001, 0);
        var actual = PolygonOps.Offset(source, radius, .0001, 0, pruneCollapsedHoles:true);
        Assert.Equal(expected.Count, actual.Count);
        for (int path = 0; path < expected.Count; path++) Assert.Equal(expected[path], actual[path]);
    }

    private static PointD[] Circle(double x, double y, double radius, int count)
        => Enumerable.Range(0,count).Select(i => new PointD(x+radius*Math.Cos(2*Math.PI*i/count),
            y+radius*Math.Sin(2*Math.PI*i/count))).ToArray();
}
