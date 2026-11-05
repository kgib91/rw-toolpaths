using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class PocketStockReplayTests
{
    [Fact]
    public void SelectiveFloorUnionsKeepNestedSignedContoursAndCapturedPrefixes()
    {
        PointD[] Circle(double x,double radius)=>Enumerable.Range(0,64).Select(i=>new PointD(
            x+radius*Math.Cos(i*Math.PI/32),radius*Math.Sin(i*Math.PI/32))).ToArray();
        var source=new List<IReadOnlyList<PointD>> {new PointD[]{new(-100,-20),new(100,-20),new(100,20),new(-100,20)}};
        for(int x=-80;x<=80;x+=10)
        {
            source.Add(Circle(x,3).Reverse().ToArray());
            source.Add(Circle(x,1.5));source.Add(Circle(x,.5).Reverse().ToArray());
        }
        var literal=new PocketContinuationRouter.ClearedStock(.4,default,knownEmpty:source,knownEmptyFloor:-1,
            selectiveFloorUnions:false);
        var selective=new PocketContinuationRouter.ClearedStock(.4,default,knownEmpty:source,knownEmptyFloor:-1);
        var oldLiteral=literal.PrepareAtFloor(-1);var oldSelective=selective.PrepareAtFloor(-1);
        for(int x=-80;x<=80;x+=10)
        {
            var stroke=new Point3D[]{new(x-3.1,0,-1),new(x+3.1,0,-1)};
            literal.Record(stroke);selective.Record(stroke);
            Equal(literal.AtFloor(-1),selective.AtFloor(-1));
        }
        Equal(oldLiteral.Materialize(),oldSelective.Materialize());
        Assert.True(selective.FloorUnionRetainedVertices>0);
        Assert.True(selective.FloorUnionSubmittedVertices>0);
    }

    [Fact]
    public void RepeatedNativeChordsDoNotRequireAnotherPhysicalSweepAtTheSameFloor()
    {
        var literal = new PocketContinuationRouter.ClearedStock(2, default, deduplicateProjectedSweeps:false,selectiveFloorUnions:false);
        var reused = new PocketContinuationRouter.ClearedStock(2, default);
        var loop = Enumerable.Range(0,129).Select(i => new Point3D(10*Math.Cos(i*Math.PI/64),
            10*Math.Sin(i*Math.PI/64),-1)).ToArray();
        void Record(IReadOnlyList<Point3D> points) { literal.Record(points); reused.Record(points); }
        Record(loop);
        Equal(literal.AtFloor(-1), reused.AtFloor(-1));
        var capturedLiteral = literal.PrepareAtFloor(-2);
        var capturedReuse = reused.PrepareAtFloor(-2);
        Record(loop.Reverse().ToArray());
        Equal(literal.AtFloor(-1), reused.AtFloor(-1));
        Record(loop.Select(p=>p with { Z=-2 }).ToArray());
        Equal(literal.AtFloor(-1), reused.AtFloor(-1));
        Equal(literal.AtFloor(-2), reused.AtFloor(-2));
        Assert.Empty(capturedLiteral.Materialize());
        Assert.Empty(capturedReuse.Materialize());
        Assert.Equal(literal.RecordedStrokes, reused.RecordedStrokes);
        Assert.True(reused.FloorUnionBatches < literal.FloorUnionBatches);
    }

    [Fact]
    public void NewSourceFollowingARetracedPrefixStillAddsItsCompleteClearedStock()
    {
        var literal = new PocketContinuationRouter.ClearedStock(2, default, deduplicateProjectedSweeps:false,selectiveFloorUnions:false);
        var reused = new PocketContinuationRouter.ClearedStock(2, default);
        foreach (var stroke in new[] {
            new Point3D[] { new(0,0,-1),new(10,0,-1),new(10,10,-1) },
            new Point3D[] { new(10,10,-1),new(10,0,-1),new(0,0,-1),new(-10,0,-1),new(-10,10,-1) } })
        {
            literal.Record(stroke); reused.Record(stroke);
            Equal(literal.AtFloor(-1), reused.AtFloor(-1));
        }
    }

    private static void Equal(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Empty(Clipper.Xor(PathUtils.ToClipper(expected),PathUtils.ToClipper(actual),FillRule.NonZero));
    }
}
