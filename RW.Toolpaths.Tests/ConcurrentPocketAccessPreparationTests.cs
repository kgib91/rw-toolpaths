using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ConcurrentPocketAccessPreparationTests
{
    private static IReadOnlyList<PointD> Box(double x0,double y0,double x1,double y1)
        =>[new(x0,y0),new(x1,y0),new(x1,y1),new(x0,y1)];

    [Theory]
    [InlineData(2)] [InlineData(3)]
    public void IndependentRecipesKeepLiteralSerialGeometryAndAuthority(int workers)
    {
        var target=new[]{new MillingRegion(Box(0,0,20,18),[Box(7,5,12,11)]),
            new MillingRegion(Box(24,3,29,12))};
        var access=new MillingBoundaryAccess([new MillingRegion(Box(7,5,12,11))],Box(0,0,32,22));
        var tool=ToolGeometry.Flat(.7);var depth=new DepthSchedule(1,1);
        MillingBoundaryAccess.Resolved Resolve()=>access.Resolve(target,tool,depth,false,30,0,default);
        var serial=Resolve().PreparePocket(.7,0,PocketContinuationRouter.KnownAirReserve,default,1);
        var owner=Resolve();
        var parallel=owner.PreparePocket(.7,0,PocketContinuationRouter.KnownAirReserve,default,workers);
        Assert.Equal(RegionWords(serial.Regions),RegionWords(parallel.Regions));
        Assert.Equal(RingWords(serial.EmptyBoundary),RingWords(parallel.EmptyBoundary));
        Assert.True(owner.CuttingRegionsPrepared);
        var empty=new PreparedRegionDistance(parallel.EmptyBoundary);
        Assert.True(empty.SignedDistance(new(9,8))>0); // Explicit approved clearance.
        Assert.True(empty.SignedDistance(new(3,3))<0); // Cutting target never gains air credit.
        static string[] RegionWords(IReadOnlyList<MillingRegion> rs)=>rs.SelectMany(r=>
            new[]{"outer"}.Concat(RingWords([r.Outer])).Concat(["holes"]).Concat(RingWords(r.Holes))).ToArray();
        static string[] RingWords(IEnumerable<IReadOnlyList<PointD>> rs)=>rs.Select(r=>
            string.Join(";",r.Select(p=>$"{p.x:R},{p.y:R}"))).ToArray();
    }

    [Theory]
    [InlineData(1)] [InlineData(2)] [InlineData(3)]
    public void RecipesRespectSlotBoundAndSerializeNestedGeometry(int workers)
    {
        int active=0,peak=0,arrivals=0;var log=new List<string>();
        using var ready=new CountdownEvent(workers);
        void Recipe(CancellationToken token)
        {
            Assert.Equal(1,GeometryParallelism.WorkerLimit);
            int count=Interlocked.Increment(ref active),old;
            do{old=Volatile.Read(ref peak);}while(count>old&&Interlocked.CompareExchange(ref peak,count,old)!=old);
            try
            {
                if(Interlocked.Increment(ref arrivals)<=workers)ready.Signal();
                Assert.True(ready.Wait(TimeSpan.FromSeconds(10),token));
            }
            finally{Interlocked.Decrement(ref active);}
        }
        var prepared=MillingBoundaryAccess.Resolved.PreparePocketRecipes(Recipe,
            t=>{Recipe(t);return Array.Empty<MillingRegion>();},
            t=>{Recipe(t);return Array.Empty<IReadOnlyList<PointD>>();},workers,default,log.Add);
        Assert.Empty(prepared.Regions);Assert.Empty(prepared.EmptyBoundary);
        Assert.Equal(workers,peak);Assert.Equal(0,active);Assert.Equal(3,arrivals);
        Assert.Contains(log,s=>s.Contains($"workers={workers} ")&&s.Contains("helpersJoined=True ")
            &&s.Contains("completed=True ")&&s.Contains("commandDomainMs=")&&s.Contains("emptyBoundaryMs="));
    }

    [Fact]
    public void HelperFailureCancelsAndJoinsCallerAndSiblingBeforeThrowing()
    {
        int active=0;using var started=new CountdownEvent(2);
        void Waiting(CancellationToken token)
        {
            Interlocked.Increment(ref active);started.Signal();
            try{token.WaitHandle.WaitOne();token.ThrowIfCancellationRequested();}
            finally{Interlocked.Decrement(ref active);}
        }
        var error=Assert.Throws<InvalidOperationException>(()=>MillingBoundaryAccess.Resolved.PreparePocketRecipes(
            _=>{Assert.True(started.Wait(TimeSpan.FromSeconds(10)));throw new InvalidOperationException("command recipe failed");},
            t=>{Waiting(t);return Array.Empty<MillingRegion>();},
            t=>{Waiting(t);return Array.Empty<IReadOnlyList<PointD>>();},3,default));
        Assert.Equal("command recipe failed",error.Message);Assert.Equal(0,active);
    }

    [Fact]
    public void CallerFailureJoinsEveryAdmittedHelperAndPublishesNoResult()
    {
        int active=0;using var started=new CountdownEvent(2);
        void Waiting(CancellationToken token)
        {
            Interlocked.Increment(ref active);started.Signal();
            try{token.WaitHandle.WaitOne();token.ThrowIfCancellationRequested();}
            finally{Interlocked.Decrement(ref active);}
        }
        Assert.Throws<InvalidOperationException>(()=>MillingBoundaryAccess.Resolved.PreparePocketRecipes(Waiting,
            _=>{Assert.True(started.Wait(TimeSpan.FromSeconds(10)));throw new InvalidOperationException("guide recipe failed");},
            t=>{Waiting(t);return Array.Empty<IReadOnlyList<PointD>>();},3,default));
        Assert.Equal(0,active);
    }

    [Fact]
    public void ExternalCancellationJoinsActiveHelpersAndPreCancellationAdmitsNothing()
    {
        using var cancellation=new CancellationTokenSource();int calls=0,active=0;
        using var started=new CountdownEvent(2);
        void Waiting(CancellationToken token)
        {
            Interlocked.Increment(ref calls);Interlocked.Increment(ref active);started.Signal();
            try{token.WaitHandle.WaitOne();token.ThrowIfCancellationRequested();}
            finally{Interlocked.Decrement(ref active);}
        }
        Assert.ThrowsAny<OperationCanceledException>(()=>MillingBoundaryAccess.Resolved.PreparePocketRecipes(Waiting,
            _=>{Assert.True(started.Wait(TimeSpan.FromSeconds(10)));cancellation.Cancel();cancellation.Token.ThrowIfCancellationRequested();return Array.Empty<MillingRegion>();},
            t=>{Waiting(t);return Array.Empty<IReadOnlyList<PointD>>();},3,cancellation.Token));
        Assert.Equal(0,active);Assert.Equal(2,calls);
        Assert.ThrowsAny<OperationCanceledException>(()=>MillingBoundaryAccess.Resolved.PreparePocketRecipes(Waiting,
            _=>throw new Exception("must not run"),_=>throw new Exception("must not run"),3,cancellation.Token));
        Assert.Equal(2,calls);
    }

    [Theory]
    [InlineData(0)] [InlineData(4)]
    public void InvalidWorkerBudgetAdmitsNoRecipes(int workers)
        =>Assert.Throws<ArgumentOutOfRangeException>(()=>MillingBoundaryAccess.Resolved.PreparePocketRecipes(
            _=>throw new Exception("must not run"),_=>throw new Exception("must not run"),
            _=>throw new Exception("must not run"),workers,default));
}
