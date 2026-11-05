using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveSpatialMixedSourcesTests
{
    private static TaggedToolpath Other(int owner,double x,int pass=0)=>new([new(x,0,-1),new(x+4,0,-1)],
        [new(0,1,ToolpathSpanKind.Cut)],0,"detail",pass,detectCurves:false)
        {ContinuationSourceOrdinal=owner,ContinuationComponentIndex=10};
    private static TaggedToolpath Ring(int owner,double x,VCarveRingOrder.Plan plan,int ring=0,int pass=0)
        =>Other(owner,x,pass) with{ConcentricSource=new(plan,ring)};
    private static readonly ToolGeometry Tool=new(1){VCarveProfile=new([new(CarvingCutterSectionKind.Linear,0,1,2)],2)};
    private static readonly VCarveOptions Options=new(){SmoothVCarving=true,OptimizeContourConnections=true,
        Motion=new(1000,500,5000,5),Ramp=new(Math.Atan(.5)),StepOver=.45,SmoothRampAngleDegrees=50,DetectCurves=false};
    private static readonly IReadOnlyList<IReadOnlyList<PointD>> Boundary=[new PointD[]{new(-20,-20),new(200,-20),new(200,40),new(-20,40)}];

    private static (List<TaggedToolpath> Paths,List<string> Log) Route(TaggedToolpath[] source,bool mixed=true,DepthSchedule? schedule=null,
        Func<IReadOnlyList<TaggedToolpath>,Point3D?,List<TaggedToolpath>>? finish=null)
    {
        var sweepBefore=VCarveRingOrder.SpatialSweepOverride.Value;
        var mixedBefore=VCarveRingRouter.SpatialMixedSourcesOverride.Value;
        var retraceBefore=VCarvePathRouter.CutRetraceOptimizationOverride.Value;
        try{
            VCarveRingOrder.SpatialSweepOverride.Value=new(VCarveRingOrder.SpatialAxis.X,32);
            VCarveRingRouter.SpatialMixedSourcesOverride.Value=mixed;VCarvePathRouter.CutRetraceOptimizationOverride.Value=false;
            var log=new List<string>();var depth=schedule??new DepthSchedule(2,2);
            var actual=VCarveRingRouter.Route(source,Tool,depth,Options,null,(_,_)=>false,null,default,log.Add,
                finish??((other,cursor)=>VCarvePathRouter.Route(other,Boundary,Tool,depth,Options,0,cursor,default,log.Add,(_,_)=>false)));
            return(actual,log);
        }finally{
            VCarveRingOrder.SpatialSweepOverride.Value=sweepBefore;VCarveRingRouter.SpatialMixedSourcesOverride.Value=mixedBefore;
            VCarvePathRouter.CutRetraceOptimizationOverride.Value=retraceBefore;
        }
    }
    private static VCarveRingSpatialDispatch Dispatch(params TaggedToolpath[] paths)
    {
        var old=VCarveRingRouter.SpatialMixedSourcesOverride.Value;
        try{VCarveRingRouter.SpatialMixedSourcesOverride.Value=true;
            return VCarveRingSpatialDispatch.Create(paths,paths,new(2,2),Options,new(VCarveRingOrder.SpatialAxis.X,32),default,null)!;
        }finally{VCarveRingRouter.SpatialMixedSourcesOverride.Value=old;}
    }

    [Fact]
    public void MixedDispatchFinishesLeftOtherWorkBeforeTheFirstRightRing()
    {
        var plan=new VCarveRingOrder.Plan([-1,-1]);
        var source=new[]{Ring(0,60,plan),Ring(1,100,plan,1),Other(2,0),Other(3,50),Other(4,90)};
        var control=Route(source,mixed:false);var actual=Route(source);
        Assert.Equal(new[]{0,1},control.Paths.Take(2).Select(p=>p.ContinuationSourceOrdinal!.Value));
        Assert.Equal(new[]{2,3,4},control.Paths.Skip(2).Select(p=>p.ContinuationSourceOrdinal!.Value).Order());
        Assert.Equal(new[]{2,3,0,4,1},actual.Paths.Select(p=>p.ContinuationSourceOrdinal!.Value));
        AssertSourcesRemain(source,actual.Paths);
        Assert.Contains(actual.Log,l=>l.Contains("stage=ring-spatial-dispatch event=complete")
            &&l.Contains("dispatchedSources=3 ")&&l.Contains("pendingBundles=0 ")&&l.Contains("globalFrontierReversals=0 "));
    }

    [Fact]
    public void SeparateOwnersInOneLargeComponentRemainSeparateSpatialJobs()
    {
        var paths=new[]{Other(0,0),Other(1,100)};
        var plan=new ToolpathContinuationComponents.Plan([],paths,default);
        var dispatch=Dispatch(paths.Select(p=>p with{ContinuationPlan=plan}).ToArray());
        Assert.Equal(2,dispatch.Bundles.Count);
        Assert.All(dispatch.Bundles,b=>Assert.Single(b.Sources));
    }

    [Fact]
    public void IdenticalNumericOwnersInDifferentPlansDoNotMerge()
    {
        var a=Other(0,0);var b=Other(0,100);
        var first=new ToolpathContinuationComponents.Plan([],[a],default);
        var second=new ToolpathContinuationComponents.Plan([],[b],default);
        Assert.Equal(2,Dispatch(a with{ContinuationPlan=first},b with{ContinuationPlan=second}).Bundles.Count);
        Assert.Single(Dispatch(a with{ContinuationPlan=first},b with{ContinuationPlan=first}).Bundles);
    }

    [Fact]
    public void ASharedTypedDefinitionStaysAnIntactBundleAcrossWindows()
    {
        Point3D[] nodes=[new(0,0,-1),new(1,0,-1)];
        var edges=new List<ToolpathCircuitBuilder.Edge>{new(0,1,nodes,1,1),new(0,1,nodes,1,1)};
        var definition=ToolpathCircuitBuilder.Definition.FromGraph(nodes,edges,[new(){0,1},new(){0,1}],0,default);
        var a=Other(0,0) with{ContinuationCircuit=true,ContinuationCircuitDefinition=definition};
        var b=Other(1,100) with{ContinuationCircuit=true,ContinuationCircuitDefinition=definition};
        var bundle=Assert.Single(Dispatch(a,b).Bundles);
        Assert.Equal(new[]{a,b},bundle.Sources);Assert.Same(definition,bundle.Sources[1].ContinuationCircuitDefinition);
        Assert.True(bundle.Extent>32);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MultiplePhysicalOrAssignedPassesRetainTheOriginalSourceDispatch(bool physical)
    {
        var plan=new VCarveRingOrder.Plan([-1]);
        var source=new[]{Ring(0,60,plan),Other(1,0,physical?0:1)};
        var actual=Route(source,schedule:physical?new DepthSchedule(2,1):new DepthSchedule(2,2));
        Assert.Equal(new[]{0,1},actual.Paths.Select(p=>p.ContinuationSourceOrdinal!.Value));
        Assert.Contains(actual.Log,l=>l.Contains("stage=ring-spatial-dispatch event=retained reason=multiple-depth-layers"));
    }

    [Fact]
    public void TrailingNonCutMotionCannotSeedAStaleRingReturnBody()
    {
        var plan=new VCarveRingOrder.Plan([-1]);var other=Other(1,0);
        var actual=Route([Ring(0,60,plan),other],finish:(paths,_)=>paths.Select(p=>p with{
            Points=p.Points.Append(new(p.Points[^1].X+1,0,-1)).ToArray(),
            Spans=[new(0,1,ToolpathSpanKind.Cut),new(1,2,ToolpathSpanKind.Link)]}).ToList());
        Assert.Equal(new[]{1,0},actual.Paths.Select(p=>p.ContinuationSourceOrdinal!.Value));
        Assert.Contains(actual.Log,l=>l.Contains("stage=ring-spatial-dispatch event=complete")&&l.Contains("endpointOnlyResets=1 "));
        Assert.Contains(actual.Log,l=>l.Contains("stage=ring-routing event=complete")&&l.Contains("returnedVisits=0 "));
    }

    [Fact]
    public void AnEmptyEnteredBundleFallsBackWithoutDroppingOriginalSources()
    {
        var plan=new VCarveRingOrder.Plan([-1]);var source=new[]{Ring(0,60,plan),Other(1,0)};int calls=0;
        var actual=Route(source,finish:(paths,cursor)=>++calls==1?[]:
            VCarvePathRouter.Route(paths,Boundary,Tool,new(2,2),Options,0,cursor,default,null,(_,_)=>false));
        Assert.Equal(2,calls);Assert.Equal(new[]{0,1},actual.Paths.Select(p=>p.ContinuationSourceOrdinal!.Value));
        AssertSourcesRemain(source,actual.Paths);
        Assert.Contains(actual.Log,l=>l.Contains("reason=empty-entered-bundle-output")&&l.Contains("trialOutputsDiscarded=True"));
    }

    [Fact]
    public void MixedDispatchRequiresAnExplicitOptIn()
    {
        var old=VCarveRingRouter.SpatialMixedSourcesOverride.Value;
        try{VCarveRingRouter.SpatialMixedSourcesOverride.Value=null;
            Assert.Null(VCarveRingSpatialDispatch.Create([Other(0,0)],[Other(0,0)],new(2,2),Options,
                new(VCarveRingOrder.SpatialAxis.X,32),default,null));
        }finally{VCarveRingRouter.SpatialMixedSourcesOverride.Value=old;}
    }

    private static void AssertSourcesRemain(IReadOnlyList<TaggedToolpath> source,IReadOnlyList<TaggedToolpath> actual)
    {
        foreach(var original in source){
            var path=Assert.Single(actual.Where(p=>p.ContinuationSourceOrdinal==original.ContinuationSourceOrdinal));
            var kinds=ToolpathSpanKinds.Expand(path);var a=original.Points[0];var b=original.Points[^1];
            Assert.Contains(Enumerable.Range(1,path.Points.Count-1),i=>kinds[i-1]==ToolpathSpanKind.Cut
                &&(path.Points[i-1]==a&&path.Points[i]==b||path.Points[i-1]==b&&path.Points[i]==a));
        }
    }
}
