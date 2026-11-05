using System.Text.RegularExpressions;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveIndependentContinuationTests
{
    private static TaggedToolpath Cut(int owner,VCarveRingOrder.Plan plan,int ring,params Point3D[] points)
        =>new(points,[new(0,points.Length-1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false)
        {ContinuationSourceOrdinal=owner,ConcentricSource=new(plan,ring)};

    private static TaggedToolpath[] Sources(TaggedToolpath? target=null,bool child=false)
    {
        var first=new VCarveRingOrder.Plan([-1,0]);var other=new VCarveRingOrder.Plan(child?[-1,0]:[-1]);
        var source=new List<TaggedToolpath>{
            Cut(0,first,0,new(0,0,-1),new(2,0,-1),new(2,2,-1),new(0,2,-1),new(0,0,-1)),
            Cut(1,first,1,new(100,0,-1.1),new(102,0,-1.1),new(102,2,-1.1),new(100,2,-1.1),new(100,0,-1.1)),
            target??Cut(2,other,0,new(2.2,0,-1),new(2.2,2,-1))};
        if(child)source.Add(Cut(3,other,1,new(2.1,.1,-.9),new(2.1,2.1,-.9)));
        // Keep the chain-tour postpass out of this scheduling fixture.
        source.Add(new([new Point3D(200,200,-1),new(201,200,-1)],[],0,"support",0,detectCurves:false));
        return source.ToArray();
    }

    private static (List<TaggedToolpath> Paths,List<string> Log) Route(bool? enable,
        Func<Point3D,Point3D,bool>? guard=null,TaggedToolpath[]? source=null,CancellationToken token=default)
    {
        var old=VCarveRingRouter.IndependentEntryContinuationOverride.Value;
        var oldRetrace=VCarvePathRouter.CutRetraceOptimizationOverride.Value;
        try{
            VCarveRingRouter.IndependentEntryContinuationOverride.Value=enable;
            VCarvePathRouter.CutRetraceOptimizationOverride.Value=false;
            var log=new List<string>();
            var paths=VCarveRingRouter.Route(source??Sources(),new ToolGeometry(3){
                    VCarveProfile=new([new(CarvingCutterSectionKind.Linear,0,3,6)],3)},
                new(3,3),new VCarveOptions{SmoothVCarving=true,OptimizeContourConnections=true,
                    Motion=new(1000,500,5000,5),Ramp=new(Math.Atan(.5)),StepOver=.45,
                    SmoothRampAngleDegrees=50,DetectCurves=false},null,guard??((_,_)=>true),null,token,log.Add,
                (other,_)=>other.ToList());
            return(paths,log);
        }finally{
            VCarveRingRouter.IndependentEntryContinuationOverride.Value=old;
            VCarvePathRouter.CutRetraceOptimizationOverride.Value=oldRetrace;
        }
    }

    [Fact]
    public void AReadyGlobalWinnerGetsAProvedStepBeforeASeparateRampEntry()
    {
        var before=Route(false);var after=Route(true);
        var old=before.Paths.Single(p=>p.ContinuationSourceOrdinal==2);
        var next=after.Paths.Single(p=>p.ContinuationSourceOrdinal==2);
        var previous=after.Paths[after.Paths.IndexOf(next)-1];
        Assert.Equal(0,old.Points[0].Z);
        Assert.Equal(previous.Points[^1],next.Points[0]);
        Assert.Contains(next.Spans,s=>s.Kind==ToolpathSpanKind.Link);
        Assert.Contains(after.Log,l=>l.Contains("independentWinnerOutsideLocal=1 ")&&l.Contains("independentWinnerAccepted=1 "));
        var motion=new ToolpathMotionCost(new(1000,500,5000,5),0);
        Assert.True(motion.SequenceSeconds(null,after.Paths,null)<motion.SequenceSeconds(null,before.Paths,null));
        AssertSourcesRemain(after.Paths,Sources());
    }

    [Fact]
    public void TheDefaultPolicyMatchesTheVerifiedContinuationPolicy()
    {
        var prior=Route(true);var current=Route(null);
        Assert.Equal(prior.Paths.SelectMany(p=>p.Points),current.Paths.SelectMany(p=>p.Points));
        Assert.Equal(prior.Paths.SelectMany(p=>p.Spans),current.Paths.SelectMany(p=>p.Spans));
        Assert.Contains(current.Log,l=>l.Contains("independentWinnerContinuation=True ")&&!l.Contains("independentWinnerAttempts=0 "));
    }

    [Fact]
    public void CachedImmutableBodyCostsKeepTheExactUncachedRouteAndScores()
    {
        var previous=VCarveRingRouter.BodyCostCacheOverride.Value;
        try{
            VCarveRingRouter.BodyCostCacheOverride.Value=false;var control=Route(true);
            VCarveRingRouter.BodyCostCacheOverride.Value=null;var cached=Route(true);
            Assert.Equal(control.Paths.SelectMany(p=>p.Points),cached.Paths.SelectMany(p=>p.Points));
            Assert.Equal(control.Paths.SelectMany(p=>p.Spans),cached.Paths.SelectMany(p=>p.Spans));
            Assert.Equal(control.Paths.Select(p=>(p.ContinuationSourceOrdinal,p.DepthPassIndex,p.RegionIndex)),
                cached.Paths.Select(p=>(p.ContinuationSourceOrdinal,p.DepthPassIndex,p.RegionIndex)));
            string before=Assert.Single(control.Log.Where(l=>l.Contains("stage=ring-routing event=complete")));
            string after=Assert.Single(cached.Log.Where(l=>l.Contains("stage=ring-routing event=complete")));
            foreach(string metric in new[]{"transferDistance","selectedSavingSec","independentWinnerSavingSec"})
                Assert.Equal(Regex.Match(before,$@"{metric}=([^ ]+)").Groups[1].Value,Regex.Match(after,$@"{metric}=([^ ]+)").Groups[1].Value);
            Assert.True(int.Parse(Regex.Match(after,@"bodyCostCacheHits=(\d+)").Groups[1].Value)>0);
        }finally{VCarveRingRouter.BodyCostCacheOverride.Value=previous;}
    }

    [Fact]
    public void ARefusedPhysicalCorridorKeepsTheSelectedIndependentEntry()
    {
        bool Guard(Point3D a,Point3D b)=>!(Math.Min(a.X,b.X)<2.1&&Math.Max(a.X,b.X)>2.1);
        var result=Route(true,Guard);
        var target=result.Paths.Single(p=>p.ContinuationSourceOrdinal==2);
        Assert.Equal(0,target.Points[0].Z);
        Assert.Contains(result.Log,l=>l.Contains("independentWinnerAccepted=0 ")&&l.Contains("independentWinnerOutsideLocal=1 "));
        AssertSourcesRemain(result.Paths,Sources());
    }

    [Fact]
    public void ACloseInteriorDoorCannotWinByHidingItsExtraArmTraversal()
    {
        var plan=new VCarveRingOrder.Plan([-1]);
        var target=Cut(2,plan,0,new(-30,.2,-1),new(30,.2,-1));
        var source=Sources(target);var result=Route(true,source:source);
        Assert.Equal(0,result.Paths.Single(p=>p.ContinuationSourceOrdinal==2).Points[0].Z);
        Assert.Contains(result.Log,l=>l.Contains("independentWinnerAccepted=0 "));
        AssertSourcesRemain(result.Paths,source);
    }

    [Fact]
    public void TheGlobalWinnerCannotSkipAnUnfinishedOffsetPredecessor()
    {
        var source=Sources(child:true);var result=Route(true,source:source);
        int parent=result.Paths.FindIndex(p=>p.ContinuationSourceOrdinal==2);
        int child=result.Paths.FindIndex(p=>p.ContinuationSourceOrdinal==3);
        Assert.True(parent>=0&&child>parent);
        AssertSourcesRemain(result.Paths,source);
    }

    [Fact]
    public void ARefusedDenseTargetKeepsTheFiniteLandingBudget()
    {
        var plan=new VCarveRingOrder.Plan([-1]);
        var points=Enumerable.Range(0,129).Select(i=>new Point3D(2.8+.1*Math.Cos(2*Math.PI*(i%128)/128),
            .1*Math.Sin(2*Math.PI*(i%128)/128),-1)).ToArray();
        var source=Sources(Cut(2,plan,0,points));var result=Route(true,(_,_)=>false,source);
        string summary=Assert.Single(result.Log.Where(l=>l.Contains("stage=ring-routing event=complete")));
        int count=int.Parse(Regex.Match(summary,@"independentWinnerLandings=(\d+)").Groups[1].Value);
        int attempts=int.Parse(Regex.Match(summary,@"independentWinnerAttempts=(\d+)").Groups[1].Value);
        Assert.InRange(count,1,24*attempts);
        Assert.Contains("independentWinnerBudgetRefusals=1 ",summary);
        AssertSourcesRemain(result.Paths,source);
    }

    [Fact]
    public void CancellationDuringTheWinnerProofPropagates()
    {
        using var cancel=new CancellationTokenSource();bool reached=false;
        bool Guard(Point3D a,Point3D b){
            if(Math.Min(a.X,b.X)<2.1&&Math.Max(a.X,b.X)>2.1){reached=true;cancel.Cancel();}
            return true;
        }
        Assert.ThrowsAny<OperationCanceledException>(()=>Route(true,Guard,token:cancel.Token));
        Assert.True(reached);
    }

    private static void AssertSourcesRemain(IReadOnlyList<TaggedToolpath> routed,IReadOnlyList<TaggedToolpath> source)
    {
        foreach(var original in source.Where(p=>p.ConcentricSource is not null)){
            var actual=Assert.Single(routed.Where(p=>p.ContinuationSourceOrdinal==original.ContinuationSourceOrdinal));
            var kinds=ToolpathSpanKinds.Expand(actual);
            for(int i=1;i<original.Points.Count;i++){
                var a=original.Points[i-1];var b=original.Points[i];
                Assert.Contains(Enumerable.Range(1,actual.Points.Count-1),j=>kinds[j-1]==ToolpathSpanKind.Cut
                    &&(actual.Points[j-1]==a&&actual.Points[j]==b||actual.Points[j-1]==b&&actual.Points[j]==a));
            }
        }
    }
}
