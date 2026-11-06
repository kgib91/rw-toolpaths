using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class RingPortalAuditWorkerTests
{
    private static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Linear,.1,.1,4)],4);
    private static readonly PointD[] Boundary=[new(-10,-10),new(10,-10),new(10,10),new(-10,10)];
    private static VCarveSmoothing.CommandClearance Wall(double surface,CancellationToken token=default)=>new([Boundary],Profile,surface,token,null);
    private static VCarveSmoothing.FloorField Floor(CancellationToken token=default)=>new(
        [new TaggedToolpath(new Point3D[]{new(-9,0,-1.5),new(9,0,-1.5)},Array.Empty<ToolpathSpan>(),0,"final-carve",null,detectCurves:false)],
        Profile,.5,4,token);
    private static TaggedToolpath[] Source()=>Enumerable.Range(0,4).Select(owner=>
    {
        double r=4-owner*.25;var points=new List<Point3D>();
        for(int side=0;side<4;side++)for(int i=0;i<8;i++)
        {
            double t=i/8d;points.Add(side switch{0=>new(-r+2*r*t,-r,-.5),1=>new(r,-r+2*r*t,-.5),
                2=>new(r-2*r*t,r,-.5),_=>new(-r,r-2*r*t,-.5)});
        }
        points.Add(points[0]);return new TaggedToolpath(points,[new(0,points.Count-1,ToolpathSpanKind.Cut)],0,"clearing",owner%2,101+owner,detectCurves:false);
    }).ToArray();

    [Theory]
    [InlineData(0)] [InlineData(12.7)] [InlineData(-12.7)]
    public void PrivateFinalAuditsKeepLiteralNominationsPortalsAndCompletePhysicalSource(double surface)
    {
        var source=Source();var serialWall=Wall(surface);var serialFloor=Floor();
        var serialGuard=VCarveSmoothing.CreateContinuationGuard(serialFloor,serialWall,surface,.5,default);
        var serialCalls=new List<(Point3D,Point3D)>();IReadOnlyList<ToolpathContinuationComponents.Edge>? expectedPortals=null;
        var expected=ToolpathSourceTraversal.RingPortals(source,.8,p=>p,(a,b)=>{serialCalls.Add((a,b));return serialGuard(a,b);},
            default,capturePortals:p=>expectedPortals=p);
        var wall=Wall(surface);var floor=Floor();var guard=VCarveSmoothing.CreateContinuationGuard(floor,wall,surface,.5,default);
        var nominalCalls=new List<(Point3D,Point3D)>();IReadOnlyList<ToolpathContinuationComponents.Edge>? actualPortals=null;
        var messages=new List<string>();int factories=0,active=0,peak=0;
        var actual=ToolpathSourceTraversal.RingPortals(source,.8,p=>p,(a,b)=>{nominalCalls.Add((a,b));return guard(a,b);},default,messages.Add,
            p=>{Assert.Equal(0,active);actualPortals=p;},
            ()=>{factories++;return VCarveSmoothing.CreateContinuationQueryWorker(floor,wall,surface,.5,default);},
            (_,_)=>{int current=Interlocked.Increment(ref active);int old;
                do{old=Volatile.Read(ref peak);}while(current>old&&Interlocked.CompareExchange(ref peak,current,old)!=old);
                Interlocked.Decrement(ref active);return true;},maximumWorkers:4);
        Assert.InRange(factories,2,4);Assert.InRange(peak,1,factories);Assert.Equal(0,active);
        Assert.True(nominalCalls.Count<serialCalls.Count);
        Assert.Equal(serialCalls.Take(nominalCalls.Count),nominalCalls);
        Equal(expected,actual);Assert.Equal(expectedPortals,actualPortals);Assert.NotEmpty(actualPortals!);
        Assert.True(actual.Where((p,i)=>p.Points.Count>source[i].Points.Count).Count()>=2);
        var physical=new VCarveNativeContainment([Boundary],Profile);
        for(int owner=0;owner<actual.Count;owner++)
        {
            Assert.InRange(actual[owner].Points.Count,source[owner].Points.Count,source[owner].Points.Count+8);
            Assert.All(actual[owner].Points.Zip(actual[owner].Points.Skip(1)),e=>Assert.True(physical.Legal(e.First,e.Second)));
        }
        Assert.Contains(messages,m=>m.Contains("stage=ring-span-portals ")&&m.Contains("privateQueryOwners=True")
            &&m.Contains("allWorkersJoined=True")&&m.Contains("nominationOrderAndBudgetsUnchanged=True"));
    }

    [Theory] [InlineData(false)] [InlineData(true)]
    public void OpaqueAndUnavailableFactoriesPreserveEveryOriginalCallbackInOrder(bool unavailable)
    {
        var source=Source();var expectedCalls=new List<(Point3D,Point3D)>();var actualCalls=new List<(Point3D,Point3D)>();
        bool Accept(Point3D a,Point3D b)=>Math.Abs(a.X-b.X)+Math.Abs(a.Y-b.Y)<2;
        var expected=ToolpathSourceTraversal.RingPortals(source,.8,p=>p,(a,b)=>{expectedCalls.Add((a,b));return Accept(a,b);},default);
        int factories=0;var messages=new List<string>();
        var actual=ToolpathSourceTraversal.RingPortals(source,.8,p=>p,(a,b)=>{actualCalls.Add((a,b));return Accept(a,b);},default,messages.Add,
            workerFactory:unavailable?()=>{factories++;return null;}:null,
            privateWorkerPrefix:(_,_)=>throw new InvalidOperationException("Opaque callback gained a private capability."),maximumWorkers:4);
        Equal(expected,actual);Assert.Equal(expectedCalls,actualCalls);Assert.Equal(unavailable?1:0,factories);
        Assert.Contains(messages,m=>m.Contains("privateQueryOwners=False")&&m.Contains("finalAuditWorkers=1"));
    }

    [Theory] [InlineData(false)] [InlineData(true)]
    public void PrivateAuditFailureOrCancellationJoinsWorkersBeforeReturningWithoutPublishing(bool cancel)
    {
        using var cancellation=new CancellationTokenSource();var source=Source();var wall=Wall(0,cancellation.Token);var floor=Floor(cancellation.Token);
        var guard=VCarveSmoothing.CreateContinuationGuard(floor,wall,0,.5,cancellation.Token);
        int factories=0,queries=0,active=0,captures=0;long before=0;
        void Run()=>ToolpathSourceTraversal.RingPortals(source,.8,p=>p,guard,cancellation.Token,capturePortals:_=>captures++,
            workerFactory:()=>{if(factories++==0)before=wall.LiteralChordQueries;
                return VCarveSmoothing.CreateContinuationQueryWorker(floor,wall,0,.5,cancellation.Token);},
            privateWorkerPrefix:(_,_)=>
            {
                Interlocked.Increment(ref active);
                try{if(Interlocked.Increment(ref queries)==1){if(cancel)cancellation.Cancel();else throw new InvalidOperationException("portal-private-failure");}return true;}
                finally{Interlocked.Decrement(ref active);}
            },maximumWorkers:4);
        if(cancel)Assert.ThrowsAny<OperationCanceledException>(Run);
        else Assert.Equal("portal-private-failure",Assert.Throws<InvalidOperationException>(Run).Message);
        Assert.True(factories>1);Assert.True(queries>0);Assert.Equal(0,active);Assert.Equal(0,captures);
        Assert.Equal(before,wall.LiteralChordQueries);Assert.All(source,p=>Assert.Equal(33,p.Points.Count));
    }

    [Fact]
    public void PrivatePrefixRetainsTheSamePerOwnerFinalRefusalAndPortalFiltering()
    {
        var source=Source();var wall=Wall(0);var floor=Floor();var guard=VCarveSmoothing.CreateContinuationGuard(floor,wall,0,.5,default);
        bool Prefix(Point3D a,Point3D b)=>Math.Abs(a.X-b.X)+Math.Abs(a.Y-b.Y)<.99;
        IReadOnlyList<ToolpathContinuationComponents.Edge>? expectedPortals=null,actualPortals=null;
        var expected=ToolpathSourceTraversal.RingPortals(source,.8,p=>p,(a,b)=>Prefix(a,b)&&guard(a,b),default,capturePortals:p=>expectedPortals=p);
        var actual=ToolpathSourceTraversal.RingPortals(source,.8,p=>p,(a,b)=>Prefix(a,b)&&guard(a,b),default,capturePortals:p=>actualPortals=p,
            workerFactory:()=>VCarveSmoothing.CreateContinuationQueryWorker(floor,wall,0,.5,default),privateWorkerPrefix:Prefix,maximumWorkers:4);
        Equal(expected,actual);Assert.Equal(expectedPortals,actualPortals);
        Assert.Contains(actual.Where((p,i)=>ReferenceEquals(p.Points,source[i].Points)),p=>p.VisitIdentity==101);
    }

    [Fact]
    public void NoEligibleSourceNeverCreatesPrivateOwnersOrCapturesPortals()
    {
        var source=Source().Select(p=>p with{Category="final-carve"}).ToArray();int factories=0,captures=0;
        var actual=ToolpathSourceTraversal.RingPortals(source,.8,p=>p,(_,_)=>throw new InvalidOperationException(),default,
            capturePortals:_=>captures++,workerFactory:()=>{factories++;return null;},maximumWorkers:4);
        Equal(source,actual);Assert.Equal(0,factories);Assert.Equal(0,captures);
    }

    [Fact]
    public void ExplicitMaximumCannotEscapeTheCurrentGeometryWorkerOwner()
    {
        var source=Source();var calls=new List<(Point3D,Point3D)>();var expectedCalls=new List<(Point3D,Point3D)>();
        var expected=ToolpathSourceTraversal.RingPortals(source,.8,p=>p,(a,b)=>{expectedCalls.Add((a,b));return true;},default);
        var messages=new List<string>();int factories=0;
        using var owner=GeometryParallelism.EnterWorker();
        var actual=ToolpathSourceTraversal.RingPortals(source,.8,p=>p,(a,b)=>{calls.Add((a,b));return true;},default,messages.Add,
            workerFactory:()=>{factories++;throw new InvalidOperationException("Nested owner admitted extra proof workers.");},
            privateWorkerPrefix:(_,_)=>throw new InvalidOperationException("Nested owner ran a private proof."),maximumWorkers:2);
        Equal(expected,actual);Assert.Equal(expectedCalls,calls);Assert.Equal(0,factories);
        Assert.Contains(messages,m=>m.Contains("finalAuditWorkers=1")&&m.Contains("privateQueryOwners=False"));
    }

    [Theory]
    [InlineData(0)] [InlineData(-1)] [InlineData(int.MinValue)]
    public void NonpositiveWorkerLimitsAreRejectedBeforeAnyGeometryOrCallback(int maximum)
    {
        int calls=0;
        var error=Assert.Throws<ArgumentOutOfRangeException>(()=>ToolpathSourceTraversal.RingPortals([], .8,
            p=>{calls++;return p;},(_,_)=>{calls++;return true;},default,workerFactory:()=>{calls++;return null;},maximumWorkers:maximum));
        Assert.Equal("maximumWorkers",error.ParamName);Assert.Equal(0,calls);
    }

    [Fact]
    public void CaptureCancellationPreventsReturningTheRebuiltSource()
    {
        using var cancellation=new CancellationTokenSource();var source=Source();var wall=Wall(0,cancellation.Token);var floor=Floor(cancellation.Token);
        var guard=VCarveSmoothing.CreateContinuationGuard(floor,wall,0,.5,cancellation.Token);int captures=0;
        Assert.ThrowsAny<OperationCanceledException>(()=>ToolpathSourceTraversal.RingPortals(source,.8,p=>p,guard,cancellation.Token,
            capturePortals:_=>{captures++;cancellation.Cancel();},workerFactory:()=>VCarveSmoothing.CreateContinuationQueryWorker(floor,wall,0,.5,cancellation.Token),maximumWorkers:4));
        Assert.Equal(1,captures);
    }

    private static void Equal(IReadOnlyList<TaggedToolpath> expected,IReadOnlyList<TaggedToolpath> actual)
    {
        Assert.Equal(expected.Count,actual.Count);for(int i=0;i<expected.Count;i++)
        {Assert.Equal(expected[i].Points,actual[i].Points);Assert.Equal(expected[i].Spans,actual[i].Spans);
            Assert.Equal(expected[i].VisitIdentity,actual[i].VisitIdentity);Assert.Equal(expected[i].RegionIndex,actual[i].RegionIndex);
            Assert.Equal(expected[i].Category,actual[i].Category);Assert.Equal(expected[i].DepthPassIndex,actual[i].DepthPassIndex);
            Assert.Same(expected[i].ConcentricSource,actual[i].ConcentricSource);}
    }
}
