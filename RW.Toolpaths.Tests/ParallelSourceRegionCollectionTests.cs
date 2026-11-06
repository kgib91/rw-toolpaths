using System.Collections;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ParallelSourceRegionCollectionTests
{
    private static IReadOnlyList<PointD> Box(double x0,double y0,double x1,double y1)
        =>[new(x0,y0),new(x1,y0),new(x1,y1),new(x0,y1)];
    private static RingOffsetOptions Options(bool parallel)=>new(1,1.5){RingLimit=3,
        MinimumRingArea=0,ParallelSourceRegions=parallel,
        Tolerances=new GeometryTolerances{ArcTolerance=.001,SimplifyTolerance=0}};
    private static string[] Words(IEnumerable<OffsetRing> rings)=>rings.Select(r=>
        $"{r.RegionIndex}/{r.Level}/{r.IsHole}/{r.IsClosed}/{r.OffsetDistance:R}/{r.Area:R}/{r.Perimeter:R}/"
        +string.Join(";",r.Points.Select(p=>$"{p.x:R},{p.y:R}"))).ToArray();

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void ParallelMainFrontsRetainExactGeometryIdsProgressNestingAndTraversal(bool insideOut)
    {
        var regions=new[]{new MillingRegion(Box(0,0,15,13),[Box(5,5,8,8)]),
            new MillingRegion(Box(25,-10,42,8)),new MillingRegion(Box(-20,4,-7,18))};
        var serialProgress=new List<(double,string)>();var parallelProgress=new List<(double,string)>();
        var serial=RingOffsetEngine.BuildRings(regions,Options(false),progress:(x,s)=>serialProgress.Add((x,s)));
        var parallel=RingOffsetEngine.BuildRings(regions,Options(true),progress:(x,s)=>parallelProgress.Add((x,s)));
        Assert.Equal(Words(serial),Words(parallel));Assert.Equal(serialProgress,parallelProgress);
        foreach(var rings in new[]{serial,parallel})RingOffsetEngine.BuildNesting(rings);
        Assert.Equal(serial.Select(r=>r.Parent),parallel.Select(r=>r.Parent));
        Assert.Equal(serial.Select(r=>string.Join(',',r.Children)),parallel.Select(r=>string.Join(',',r.Children)));
        var first=RingOffsetEngine.BuildTraversal(serial,insideOut);var second=RingOffsetEngine.BuildTraversal(parallel,insideOut);
        Assert.Equal(first.Order,second.Order);Assert.Equal(first.Islands,second.Islands);
        Assert.Equal(RingOffsetEngine.BuildChains(serial,first.Order).Select(c=>string.Join(',',c.RingIndices)),
            RingOffsetEngine.BuildChains(parallel,second.Order).Select(c=>string.Join(',',c.RingIndices)));
    }

    [Fact]
    public void ExactFirstCollapseAdditionsStillFollowAllMainRegionResults()
    {
        var regions=new[]{new MillingRegion(Box(0,0,4,20)),new MillingRegion(Box(20,0,60,40))};
        var options=new RingOffsetOptions(2,4){RingLimit=2,MinimumRingArea=0,BoundOriginalSourceMedial=true,
            Tolerances=new GeometryTolerances{ArcTolerance=.001,SimplifyTolerance=0}};
        var serial=MedialAxisGeometry.BuildConcentricFill(regions,options,2,true);
        var parallel=MedialAxisGeometry.BuildConcentricFill(regions,options with{ParallelSourceRegions=true},2,true);
        Assert.Equal(Words(serial),Words(parallel));
        Assert.Contains(parallel,r=>r.RegionIndex==0&&!r.IsClosed);
        int lastMain=parallel.FindLastIndex(r=>r.RegionIndex==1&&r.IsClosed);
        int firstExact=parallel.FindIndex(r=>r.RegionIndex==0&&!r.IsClosed);
        Assert.True(firstExact>lastMain);
    }

    [Theory]
    [InlineData("level")] [InlineData("front")] [InlineData("terminal")]
    public void OpaqueConstructionCallbacksRetainSerialOrderAndCallerContext(string kind)
    {
        int caller=Environment.CurrentManagedThreadId;var observed=new List<int>();
        var sources=Enumerable.Range(0,2).Select(i=>new MillingRegion(new ProbeRing(Box(i*20,0,i*20+10,10),
            ()=>{Assert.Equal(caller,Environment.CurrentManagedThreadId);observed.Add(i);}))).ToArray();
        var options=Options(true) with {MaximumInset=1.5};
        options=kind switch {
            "level"=>options with{LevelProgress=(_,_)=>{}},
            "front"=>options with{MaximumInsetFront=r=>[r.Outer]},
            _=>options with{TerminalPaths=(_,_)=>Array.Empty<IReadOnlyList<PointD>>()}};
        RingOffsetEngine.BuildRings(sources,options);
        Assert.Equal(new[]{0,1},observed);
    }

    [Fact]
    public void RegionWorkersAreBoundedAndNestedGeometryIsSerial()
    {
        int expected=Math.Min(2,GeometryParallelism.WorkerLimit),active=0,peak=0;
        using var arrivals=new CountdownEvent(expected);
        var sources=Enumerable.Range(0,2).Select(i=>new MillingRegion(new ProbeRing(Box(i*20,0,i*20+10,10),()=>{
            if(expected>1)Assert.Equal(1,GeometryParallelism.WorkerLimit);
            int count=Interlocked.Increment(ref active),old;
            do{old=Volatile.Read(ref peak);}while(count>old&&Interlocked.CompareExchange(ref peak,count,old)!=old);
            try{if(arrivals.CurrentCount>0)arrivals.Signal();Assert.True(arrivals.Wait(TimeSpan.FromSeconds(10)));}
            finally{Interlocked.Decrement(ref active);}
        }))).ToArray();
        var rings=RingOffsetEngine.BuildRings(sources,Options(true));
        Assert.NotEmpty(rings);Assert.Equal(expected,peak);Assert.Equal(0,active);
        Assert.All(rings.GroupBy(r=>r.RegionIndex),group=>Assert.All(group,r=>Assert.InRange(r.Level,0,2)));
    }

    [Fact]
    public void WorkerFailureJoinsActiveRegionsAndPublishesNoProgressOrPartialCollection()
    {
        if(GeometryParallelism.WorkerLimit<2)return;
        int active=0;bool slowFinished=false;using var slowStarted=new ManualResetEventSlim();var progress=new List<string>();
        var sources=new[]{new MillingRegion(new ProbeRing(Box(0,0,10,10),()=>{
                Assert.True(slowStarted.Wait(TimeSpan.FromSeconds(10)));throw new InvalidOperationException("region failed");})),
            new MillingRegion(new ProbeRing(Box(20,0,30,10),()=>{
                Interlocked.Increment(ref active);slowStarted.Set();
                try{Thread.Sleep(40);slowFinished=true;}finally{Interlocked.Decrement(ref active);}}))};
        var failure=Assert.Throws<InvalidOperationException>(()=>RingOffsetEngine.BuildRings(sources,Options(true),progress:(_,s)=>progress.Add(s)));
        Assert.Contains("region failed",failure.ToString());Assert.Equal(0,active);Assert.True(slowFinished);Assert.Empty(progress);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void FinalProgressCancellationCannotReturnACompletedCollection(bool parallel)
    {
        using var cancel=new CancellationTokenSource();
        var sources=new[]{new MillingRegion(Box(0,0,10,10)),new MillingRegion(Box(20,0,30,10))};
        Assert.ThrowsAny<OperationCanceledException>(()=>RingOffsetEngine.BuildRings(sources,Options(parallel),cancel.Token,
            (fraction,_)=>{if(fraction==1)cancel.Cancel();}));
    }

    [Fact]
    public void ActiveCancellationJoinsWorkersAndPreCancellationAdmitsNoSource()
    {
        using var cancel=new CancellationTokenSource();int expected=Math.Min(2,GeometryParallelism.WorkerLimit),active=0;
        using var arrivals=new CountdownEvent(expected);var progress=new List<string>();
        var sources=Enumerable.Range(0,2).Select(i=>new MillingRegion(new ProbeRing(Box(i*20,0,i*20+10,10),()=>{
            Interlocked.Increment(ref active);
            try{if(arrivals.CurrentCount>0)arrivals.Signal();Assert.True(arrivals.Wait(TimeSpan.FromSeconds(10)));cancel.Cancel();cancel.Token.ThrowIfCancellationRequested();}
            finally{Interlocked.Decrement(ref active);}
        }))).ToArray();
        Assert.ThrowsAny<OperationCanceledException>(()=>RingOffsetEngine.BuildRings(sources,Options(true),cancel.Token,(_,s)=>progress.Add(s)));
        Assert.Equal(0,active);if(expected>1)Assert.Empty(progress);
        int calls=0;var untouched=new[]{new MillingRegion(new ProbeRing(Box(0,0,10,10),()=>calls++))};
        Assert.ThrowsAny<OperationCanceledException>(()=>RingOffsetEngine.BuildRings(untouched,Options(true),cancel.Token));Assert.Equal(0,calls);
    }

    private sealed class ProbeRing(IReadOnlyList<PointD> points,Action firstRead):IReadOnlyList<PointD>
    {
        private int observed;
        private void Observe(){if(Interlocked.Exchange(ref observed,1)==0)firstRead();}
        public int Count=>points.Count;
        public PointD this[int index]{get{Observe();return points[index];}}
        public IEnumerator<PointD> GetEnumerator(){Observe();return points.GetEnumerator();}
        IEnumerator IEnumerable.GetEnumerator()=>GetEnumerator();
    }
}
