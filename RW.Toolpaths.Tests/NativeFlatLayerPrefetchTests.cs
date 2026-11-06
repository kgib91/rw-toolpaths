using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeFlatLayerPrefetchTests
{
    private static readonly PointD[][] Target=[[new(-.2,-3),new(.2,-3),new(.2,3),new(-.2,3)]];
    private static readonly PointD[] Part=[new(-10,-10),new(10,-10),new(10,10),new(-10,10)];
    private static readonly VCarveRoughingChord[] Columns=[new(new(-2,0,-2),new(2,0,-2),.5,.5,.5),
        new(new(2,0,-1),new(2,2,-1),.5,.5,.5)];
    private static VCarveClearedStock Stock()=>new(Columns.ToArray(),[1,2],Part.ToArray());
    private static Paths64 Canonical(IEnumerable<IReadOnlyList<PointD>> target)=>Clipper.Union(PathUtils.ToClipper(target),FillRule.NonZero);
    private static string Encode(object value)=>JsonSerializer.Serialize(value,new JsonSerializerOptions{IncludeFields=true});
    private static VCarveMaterialSurface.FlatLayerGeometry[] Literal(VCarveClearedStock stock)=>VCarveMaterialSurface.BuildFlatLayerGeometry(
        Canonical(Target),PathUtils.ToClipper(new[]{stock.PartBoundary}),VCarveMaterialSurface.AdmittedPostedColumns(stock.Chords),
        VCarveMaterialSurface.FlatFloorSchedule(stock.FloorDepths,2),default,null).ToArray();

    [Fact]
    public void CompleteGeometryIsLiteralOrderedSingleConsumptionWithOnePrivateWorker()
    {
        if(GeometryParallelism.HardwareWorkerLimit<=1)return;
        var stock=Stock();var logs=new List<string>();int workers=0;
        using var owner=new VCarveFlatLayerPrefetch(diagnostic:s=>{lock(logs)logs.Add(s);});
        Assert.True(owner.Start(_=>{workers=GeometryParallelism.WorkerLimit;return Target;},stock,2));
        var actual=owner.Take(VCarveMaterialSurface.AdmittedPostedColumns(stock.Chords),Canonical(Target),stock.PartBoundary,stock.FloorDepths,2,default);
        Assert.NotNull(actual);Assert.Equal(1,workers);Assert.Equal(Encode(Literal(stock)),Encode(actual));
        Assert.Null(owner.Take(Columns,Canonical(Target),Part,[1,2],2,default));
        Assert.Contains(logs,s=>s.Contains("event=hit")&&s.Contains("exactOrderedInputs=True")&&s.Contains("queryProofsShared=False"));
    }

    [Theory]
    [InlineData("column-order")]
    [InlineData("column-bits")]
    [InlineData("floors")]
    [InlineData("part")]
    [InlineData("depth")]
    [InlineData("target")]
    public void EveryLiteralOperandMismatchUsesTheOriginalFallback(string change)
    {
        if(GeometryParallelism.HardwareWorkerLimit<=1)return;
        var stock=Stock();using var owner=new VCarveFlatLayerPrefetch();Assert.True(owner.Start(_=>Target,stock,2));
        var columns=Columns.ToArray();var part=Part.ToArray();double[] floors=[1,2];double depth=2;var target=Canonical(Target);
        switch(change)
        {
            case "column-order":Array.Reverse(columns);break;
            case "column-bits":columns[0]=columns[0] with{A=columns[0].A with{Y=BitConverter.Int64BitsToDouble(long.MinValue)}};break;
            case "floors":floors=[2,1];break;
            case "part":part[0]=new(-9,-10);break;
            case "depth":depth=1.5;break;
            case "target":target[0][0]=new(target[0][0].X+1,target[0][0].Y);break;
        }
        Assert.Null(owner.Take(columns,target,part,floors,depth,default));
        // A mismatched request never consumes or mutates the original snapshot.
        Assert.NotNull(owner.Take(Columns,Canonical(Target),Part,[1,2],2,default));
    }

    [Fact]
    public void StockSnapshotsAreDetachedAndLaterZeroRadiusProfilesDoNotChangeFlatInputs()
    {
        if(GeometryParallelism.HardwareWorkerLimit<=1)return;
        var columns=Columns.ToArray();var part=Part.ToArray();double[] floors=[1,2];var stock=new VCarveClearedStock(columns,floors,part);
        using var owner=new VCarveFlatLayerPrefetch();Assert.True(owner.Start(_=>Target,stock,2));
        columns[0]=columns[0] with{ClearedColumnRadius=.25};part[0]=new(-9,-9);floors[0]=.5;
        var complete=Columns.Append(new VCarveRoughingChord(new(0,0,-2),new(1,1,-2),0,0,0)).ToArray();
        Assert.NotNull(owner.Take(VCarveMaterialSurface.AdmittedPostedColumns(complete),Canonical(Target),Part,[1,2],2,default));
        Assert.Equal(0,complete[^1].ClearedColumnRadius);
    }

    [Fact]
    public void ParentCancellationJoinsAndCannotPublishPartialGeometry()
    {
        if(GeometryParallelism.HardwareWorkerLimit<=1)return;
        using var cancellation=new CancellationTokenSource();using var entered=new ManualResetEventSlim();using var ended=new ManualResetEventSlim();
        var owner=new VCarveFlatLayerPrefetch(cancellation.Token);
        Assert.True(owner.Start(token=>{entered.Set();try{token.WaitHandle.WaitOne();token.ThrowIfCancellationRequested();return Target;}finally{ended.Set();}},Stock(),2));
        Assert.True(entered.Wait(TimeSpan.FromSeconds(10)));cancellation.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(()=>owner.Dispose());Assert.True(ended.IsSet);
    }

    [Fact]
    public void OptionalFailureAndNestedWorkerUseLiteralConstruction()
    {
        if(GeometryParallelism.HardwareWorkerLimit>1)
        {
            using var owner=new VCarveFlatLayerPrefetch();Assert.True(owner.Start(_=>throw new InvalidOperationException("known-prefetch-failure"),Stock(),2));
            Assert.Null(owner.Take(Columns,Canonical(Target),Part,[1,2],2,default));Assert.NotEmpty(Literal(Stock()));
        }
        using(var worker=GeometryParallelism.EnterWorker())using(var owner=new VCarveFlatLayerPrefetch())
            Assert.False(owner.Start(_=>throw new InvalidOperationException("must-not-start"),Stock(),2));
    }

    [Fact]
    public void OversizedInputNeverStartsGeometryWork()
    {
        if(GeometryParallelism.HardwareWorkerLimit<=1)return;
        var many=Enumerable.Repeat(Columns[0],400_001).ToArray();bool invoked=false;
        using var owner=new VCarveFlatLayerPrefetch();
        Assert.False(owner.Start(_=>{invoked=true;return Target;},new(many,[1,2],Part),2));Assert.False(invoked);
    }
}
