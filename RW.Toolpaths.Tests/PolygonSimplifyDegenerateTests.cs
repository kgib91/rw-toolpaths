using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class PolygonSimplifyDegenerateTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void DenseValidSliverKeepsEveryAuthoredChordWhenSimplificationCollapses(
        bool repeatedClosure, bool clockwise)
    {
        var source = ThinRectangle();
        if (repeatedClosure) source.Add(source[0]);
        if (clockwise) source.Reverse();
        var collapsed = PolygonSimplify.Rdp(source, .25);
        Assert.Equal(repeatedClosure ? 3 : 2, collapsed.Count);
        Assert.Equal(0, PolygonOps.SignedArea(collapsed));
        Assert.NotEqual(0, PolygonOps.SignedArea(source));

        var retained = Assert.Single(PolygonSimplify.RdpAll(new[] { source }, .25));
        Assert.NotSame(source, retained);
        Assert.Equal(source, retained);
        Assert.Equal(clockwise ? -.02 : .02, PolygonOps.SignedArea(retained), 10);
        // Exact sequence equality includes each original edge and the existing
        // closing edge. No center chord replaces either side of this narrow ring.
        Assert.Equal(source.Count, retained.Count);
    }

    [Theory]
    [InlineData(3)]
    [InlineData(8)]
    public void AlreadyCollinearSourceDoesNotGainAFallbackContour(int vertices)
    {
        var source = Enumerable.Range(0, vertices).Select(index => new PointD(0, index)).ToList();
        Assert.Empty(PolygonSimplify.RdpAll(new[] { source }, .25));
    }

    [Fact]
    public void ZeroAreaSelfCrossingSourceDoesNotGainAFallbackContour()
    {
        var source = new List<PointD> { new(0, 0), new(2, 2), new(0, 2), new(2, 0) };
        Assert.Empty(PolygonSimplify.RdpAll(new[] { source }, .25));
    }

    [Fact]
    public void NonfiniteSourceDoesNotGainAFallbackContour()
    {
        var source = ThinRectangle();
        source[1] = new PointD(double.NaN, 0);
        Assert.Empty(PolygonSimplify.RdpAll(new[] { source }, .25));
    }

    [Fact]
    public void OrdinaryCollinearWallFacetsStillSimplify()
    {
        var source = new List<PointD>
        {
            new(0, 0), new(2, 0), new(4, 0), new(6, 0), new(8, 0), new(10, 0), new(10, 5), new(0, 5),
        };
        var simplified = Assert.Single(PolygonSimplify.RdpAll(new[] { source }, .25));
        Assert.Equal(4, simplified.Count);
        Assert.Equal(50, PolygonOps.SignedArea(simplified), 10);
    }

    private static List<PointD> ThinRectangle()
    {
        var source = new List<PointD> { new(0, 0), new(.0005, 0) };
        for (int y = 5; y <= 40; y += 5) source.Add(new PointD(.0005, y));
        for (int y = 40; y >= 5; y -= 5) source.Add(new PointD(0, y));
        return source;
    }
}
