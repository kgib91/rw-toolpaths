using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class RasterPreparationLookaheadTests
{
    private static PointD[] Box(double x,double y,double z,double w)=>[new(x,y),new(z,y),new(z,w),new(x,w)];
    private static MillingRegion Region()=>new(Box(-6,-5,6,5),[Box(-1,-1,1,1).Reverse().ToArray()]);
    private static PocketOptions Options()=>new(){SmoothRoughing=true,IndependentRasterEntries=true,
        ProtectedBoundary=[Box(-.5,-.5,.5,.5)],StepOver=.5,DetectCurves=false,CalculateSweptArea=false,
        Ramp=RampSettings.FromRatio(0),Tolerances=GeometryTolerances.Default with{ArcTolerance=.001,SimplifyTolerance=.001},Motion=new(200,75,1000,3)};
    private static T Mode<T>(bool enabled,Func<T> work)
    {
        var previous=ScanlinePocketToolpaths.RasterLookaheadEnabled.Value;
        try{ScanlinePocketToolpaths.RasterLookaheadEnabled.Value=enabled;return work();}
        finally{ScanlinePocketToolpaths.RasterLookaheadEnabled.Value=previous;}
    }

    [Theory] [InlineData(0)] [InlineData(17)] [InlineData(90)]
    public void CompleteScanlineGeometryOrderingOptionsAndMachineCostMatchSerialControl(double angle)
    {
        var regions=new[]{Region()};var tool=new ToolGeometry(.5);var depth=new DepthSchedule(.5,.25);var options=Options();
        var expected=Mode(false,()=>ScanlinePocketToolpaths.Generate(regions,tool,depth,options,angle));
        var actual=Mode(true,()=>ScanlinePocketToolpaths.Generate(regions,tool,depth,options,angle));
        Assert.NotEmpty(actual.Toolpaths);Assert.Equal(expected.Toolpaths.Count,actual.Toolpaths.Count);
        for(int i=0;i<expected.Toolpaths.Count;i++)
        {
            Assert.Equal(expected.Toolpaths[i].Points,actual.Toolpaths[i].Points);Assert.Equal(expected.Toolpaths[i].Spans,actual.Toolpaths[i].Spans);
            Assert.Equal(expected.Toolpaths[i].RegionIndex,actual.Toolpaths[i].RegionIndex);Assert.Equal(expected.Toolpaths[i].VisitIdentity,actual.Toolpaths[i].VisitIdentity);
            Assert.Equal(expected.Toolpaths[i].Category,actual.Toolpaths[i].Category);Assert.Equal(expected.Toolpaths[i].DepthPassIndex,actual.Toolpaths[i].DepthPassIndex);
        }
        Assert.Equal(expected.Diagnostics.CutLength,actual.Diagnostics.CutLength);Assert.Equal(expected.Diagnostics.LinkLength,actual.Diagnostics.LinkLength);
        Assert.Equal(new ToolpathMotionCost(options.Motion!,0).SequenceSeconds(null,expected.Toolpaths,null),
            new ToolpathMotionCost(options.Motion!,0).SequenceSeconds(null,actual.Toolpaths,null));
    }

    [Theory] [InlineData(-.75,.001,0)] [InlineData(-.25,.002,.001)] [InlineData(.1,.001,.001)]
    public void LookaheadExecutesTheLiteralOrderedDifferenceOffsetAndClip(double inset,double arc,double simplify)
    {
        var source=Region();var clip=Box(-5,-4,5,4);
        var literal=PolygonOps.Intersect(PolygonOps.Offset(PolygonOps.Difference(new[]{source.Outer},source.Holes),inset,arc,simplify),new[]{clip});
        using var owner=new ScanlinePocketToolpaths.RasterInsetLookahead(3,
            token=>ScanlinePocketToolpaths.PrepareRasterInset(source,inset,arc,simplify,clip,token),default);
        Assert.Null(owner.Take(2,default));var actual=owner.Take(3,default)!;
        Assert.Equal(Encode(literal),Encode(actual.Inset));Assert.True(owner.Consumed);Assert.True(owner.Joined);
        Assert.InRange(owner.RetainedBytes,1,ScanlinePocketToolpaths.RasterInsetLookahead.MaximumBytes);Assert.Null(owner.Take(3,default));
    }

    [Fact]
    public void SnapshotRetainsExactOperandsAndDoesNotPublishMutatedCallerLists()
    {
        var outer=Box(-6,-5,6,5);var clip=Box(-5,-4,5,4);var source=new MillingRegion(outer);
        var options=Options() with{ClipBoundary=clip};var tool=new ToolGeometry(.5);
        var literal=ScanlinePocketToolpaths.PrepareRasterInset(new MillingRegion(outer.ToArray()),-.5,
            options.Tolerances.ArcTolerance,options.Tolerances.SimplifyTolerance,clip.ToArray(),default);
        Mode(true,()=>
        {
            using var owner=ScanlinePocketToolpaths.BeginRasterLookahead([source],tool,options,default)!;
            Assert.NotNull(owner);outer[0]=new(-99,-99);clip[0]=new(99,99);
            Assert.Equal(Encode(literal.Inset),Encode(owner.Take(0,default)!.Inset));return 0;
        });
    }

    [Fact]
    public async Task SinglePrivateWorkerAndOrderedFailureRemainJoined()
    {
        var started=new TaskCompletionSource<int>(TaskCreationOptions.RunContinuationsAsynchronously);
        using var release=new ManualResetEventSlim();var failure=new InvalidOperationException("selected-raster-region");
        using var owner=new ScanlinePocketToolpaths.RasterInsetLookahead(4,token=>
        {started.SetResult(GeometryParallelism.WorkerLimit);release.Wait(token);throw failure;},default);
        Assert.Equal(1,await started.Task.WaitAsync(TimeSpan.FromSeconds(10)));Assert.Null(owner.Take(0,default));
        release.Set();Assert.Same(failure,Assert.Throws<InvalidOperationException>(()=>owner.Take(4,default)));Assert.True(owner.Joined);
    }

    [Fact]
    public async Task EarlierFailureOrCancellationCancelsAndJoinsUnconsumedGeometry()
    {
        var started=new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);int active=0;
        var owner=new ScanlinePocketToolpaths.RasterInsetLookahead(1,token=>
        {
            Interlocked.Increment(ref active);started.SetResult();
            try{token.WaitHandle.WaitOne();token.ThrowIfCancellationRequested();throw new InvalidOperationException();}
            finally{Interlocked.Decrement(ref active);}
        },default);
        await started.Task.WaitAsync(TimeSpan.FromSeconds(10));owner.Dispose();
        Assert.True(owner.Joined);Assert.Equal(0,active);Assert.False(owner.Consumed);
    }

    [Fact]
    public void OversizedCompleteResultRetainsNothingAndRequestsOriginalSerialGeometry()
    {
        using var owner=new ScanlinePocketToolpaths.RasterInsetLookahead(0,
            _=>new([Box(-1,-1,1,1).ToList()],0,0,0),default,maximumBytes:16);
        Assert.Null(owner.Take(0,default));Assert.True(owner.Oversized);Assert.Equal(0,owner.RetainedBytes);
        Assert.False(owner.Consumed);Assert.True(owner.Joined);
    }

    [Fact]
    public void NestedBudgetAndOpaqueOrIneligibleContextsKeepOriginalSerialPreparation()
    {
        Mode(true,()=>
        {
            var options=Options();var source=new[]{Region()};var tool=new ToolGeometry(.5);int calls=0;
            using(GeometryParallelism.EnterWorker())Assert.Null(ScanlinePocketToolpaths.BeginRasterLookahead(source,tool,options,default));
            Assert.Null(ScanlinePocketToolpaths.BeginRasterLookahead(source,tool,options with{SmoothRoughing=false},default));
            Assert.Null(ScanlinePocketToolpaths.BeginRasterLookahead(source,tool,options with{IndependentRasterEntries=false},default));
            Assert.Null(ScanlinePocketToolpaths.BeginRasterLookahead(source,tool,options with{ProtectedBoundary=null},default));
            Assert.Null(ScanlinePocketToolpaths.BeginRasterLookahead(source,tool,options with{Motion=options.Motion! with
                {ClearedTravelGuard=(_,_)=>{calls++;throw new InvalidOperationException();}}},default));
            Assert.Equal(0,calls);return 0;
        });
    }
    private static byte[] Encode(IReadOnlyList<List<PointD>> paths)
    {
        using var stream=new MemoryStream();using var writer=new BinaryWriter(stream);
        writer.Write(paths.Count);foreach(var path in paths){writer.Write(path.Count);foreach(var point in path){writer.Write(point.x);writer.Write(point.y);}}
        return stream.ToArray();
    }
}
