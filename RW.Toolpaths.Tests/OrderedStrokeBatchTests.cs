using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class OrderedStrokeBatchTests
{
    [Theory]
    [InlineData(.00318310546875)] [InlineData(.5)]
    public void ParallelBatchesRetainTheLiteralSerialIntegerContoursAndHoles(double radius)
    {
        IReadOnlyList<PointD> Ring(double x,double r)=>Enumerable.Range(0,4097)
            .Select(i=>new PointD(x+r*Math.Cos(2*Math.PI*i/4096),r*Math.Sin(2*Math.PI*i/4096))).ToArray();
        var lines=new[]{Ring(0,10),Ring(24,8)};
        var parallel=PolygonOps.BufferOpenCenterlines(lines,radius,.001);
        List<List<PointD>> serial;
        using(GeometryParallelism.EnterWorker())serial=PolygonOps.BufferOpenCenterlines(lines,radius,.001);
        Assert.Equal(PathUtils.ToClipper(serial).SelectMany((p,i)=>p.Select(v=>(i,v.X,v.Y))),
            PathUtils.ToClipper(parallel).SelectMany((p,i)=>p.Select(v=>(i,v.X,v.Y))));
        Assert.True(new PreparedRegionDistance(parallel).SignedDistance(new(0,0))<0);
    }
}
