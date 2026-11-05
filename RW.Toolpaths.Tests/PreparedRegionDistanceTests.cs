using Clipper2Lib;
using RW.Toolpaths.Geometry;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class PreparedRegionDistanceTests
{
    [Fact]
    public void DirectAndCachedDistancesMatchIndependentEdgesAndFilledPolygons()
    {
        var rectangle=Rect(-2.123456,-3.987654,4.567891,6.345678);
        double angle=.417;
        var rotated=rectangle.Select(p=>new PointD(p.x*Math.Cos(angle)-p.y*Math.Sin(angle),
            p.x*Math.Sin(angle)+p.y*Math.Cos(angle))).ToArray();
        var hole=Rect(0,0,1,1);Array.Reverse(hole);
        foreach(var source in new IReadOnlyList<PointD>[][]{[rectangle],[rotated],[rectangle,hole]})
        {
            var canonical=Clipper.Union(PathUtils.ToClipper(source),FillRule.NonZero);
            var region=new PreparedRegionDistance(source);
            var worker=region.CreateQueryWorker();
            for(int ix=-16;ix<=20;ix++)
            for(int iy=-20;iy<=24;iy++)
            {
                var point=new PointD(ix*.337+.1/PathUtils.Scale,iy*.319-.1/PathUtils.Scale);
                var q=new Point64((long)Math.Round(point.x*PathUtils.Scale),(long)Math.Round(point.y*PathUtils.Scale));
                double nearest=double.PositiveInfinity;bool inside=false;
                foreach(var ring in canonical)
                {
                    if(Clipper.PointInPolygon(q,ring)==PointInPolygonResult.IsInside)inside=!inside;
                    for(int i=0;i<ring.Count;i++)
                    {
                        var a=ring[i];var b=ring[(i+1)%ring.Count];
                        double dx=b.X-a.X,dy=b.Y-a.Y;
                        double t=Math.Clamp(((q.X-a.X)*dx+(q.Y-a.Y)*dy)/(dx*dx+dy*dy),0,1);
                        nearest=Math.Min(nearest,Math.Sqrt(Math.Pow(q.X-a.X-t*dx,2)+Math.Pow(q.Y-a.Y-t*dy,2))/PathUtils.Scale);
                    }
                }
                double expected=nearest<=PreparedRegionDistance.Uncertainty?0:inside?nearest:-nearest;
                Assert.Equal(expected,region.SignedDistanceUncached(point),9);
                Assert.Equal(expected,region.SignedDistance(point),9);
                Assert.Equal(expected,worker.SignedDistanceUncached(point),9);
                Assert.Equal(expected,worker.SignedDistance(point),9);
                Assert.Equal(expected,worker.SignedDistance(point),9);
            }
        }
    }

    [Fact]
    public void CanonicalFilledDistanceRespectsHoleAndUncertainBoundary()
    {
        var outer = Rect(0, 0, 10, 10);
        var hole = Rect(4, 4, 6, 6); Array.Reverse(hole);
        var region = new PreparedRegionDistance(new[] { (IReadOnlyList<PointD>)outer, hole });
        Assert.Equal(1, region.SignedDistance(new PointD(1, 5)), 6);
        Assert.Equal(-1, region.SignedDistance(new PointD(5, 5)), 6);
        Assert.Equal(-2, region.SignedDistance(new PointD(-2, 5)), 6);
        Assert.Equal(0, region.SignedDistance(new PointD(0, 5)));
        Assert.Equal(0, region.SignedDistance(new PointD(PreparedRegionDistance.Uncertainty / 2, 5)));
        Assert.True(double.IsNegativeInfinity(new PreparedRegionDistance(Array.Empty<IReadOnlyList<PointD>>())
            .SignedDistance(new PointD(1, 1))));
    }

    [Fact]
    public void RepeatedQuantizedAndConcurrentQueriesRetainBoundaryAndHoleClassification()
    {
        var outer = Rect(0, 0, 10, 10);
        var hole = Rect(4, 4, 6, 6); Array.Reverse(hole);
        var region = new PreparedRegionDistance(new[] { (IReadOnlyList<PointD>)outer, hole });
        var points = new[] { new PointD(1, 5), new PointD(5, 5), new PointD(0, 5) };
        var expected = new[] { 1d, -1d, 0d };
        Parallel.For(0, 1000, index =>
        {
            int item = index % points.Length;
            Assert.Equal(expected[item], region.SignedDistance(points[item]));
        });
        long queries = region.BoundaryQueries;
        Assert.Equal(1, region.SignedDistance(new PointD(1 + .1 / PathUtils.Scale, 5)));
        Assert.Equal(queries, region.BoundaryQueries);
        Assert.Equal(3, region.CachedQueryCount);
        Assert.True(region.CachedQueryHits > 900);
    }

    private static PointD[] Rect(double x0, double y0, double x1, double y1) => new[] {
        new PointD(x0, y0), new PointD(x1, y0), new PointD(x1, y1), new PointD(x0, y1) };
}
