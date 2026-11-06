using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class EnteredVisitCostMemoTests
{
    private static TaggedToolpath Path(double surface)=>new(new Point3D[]{new(0,0,surface-.2),new(.0004,0,surface-.23),
        new(1,0,surface-.8),new(2,.2,surface-.9),new(3,.2,surface-.91),new(3,.2,surface+1),new(4,.2,surface+1)},
        new ToolpathSpan[]{new(0,2,ToolpathSpanKind.Ramp),new(2,4,ToolpathSpanKind.Cut),
            new(4,5,ToolpathSpanKind.TabLift),new(5,6,ToolpathSpanKind.ClearedTravel)},0,"memo-price",0,detectCurves:false);
    private static long Bits(double value)=>BitConverter.DoubleToInt64Bits(value);

    [Theory]
    [InlineData(0)] [InlineData(12.7)]
    public void ResetModalVisitPriceIsBitExactAndEveryTransferCallbackStillRuns(double surface)
    {
        var path=Path(surface);var origins=new[]{new Point3D(5,5,surface-.7),new(-2,3,surface+2),new(8,-2,surface-.2),new(1,1,surface+3)};
        var oldLog=new List<(Point3D,Point3D)>();var newLog=new List<(Point3D,Point3D)>();
        MillingMotionSettings Settings(List<(Point3D,Point3D)> log)=>new(100.7,37.2,1200.25,surface+5,surface){
            ClearedTravelGuard=(a,b)=>{log.Add((a,b));return log.Count%2==0;}};
        var literal=new ToolpathMotionCost(Settings(oldLog),surface);
        var cost=new ToolpathMotionCost(Settings(newLog),surface);var memo=new ToolpathEnteredVisitMemo(cost);
        var entry=memo.Store(1,4,surface,path,path);
        foreach(var origin in origins)
            Assert.Equal(Bits(literal.CompleteVisitSeconds(origin,path)),Bits(memo.CompleteVisitSeconds(origin,entry)));
        Assert.Equal(oldLog,newLog);Assert.NotEmpty(oldLog);
        Assert.Equal(1,memo.PricedMisses);Assert.Equal(origins.Length-1,memo.PricedHits);
        Assert.Equal((origins.Length-1)*(path.Points.Count-1),memo.ChordsAvoided);
    }

    [Fact]
    public void ExactEntryTopBitsAndSourceIndexBindTheMemoAndEvictionRemainsBounded()
    {
        var path=Path(0);var memo=new ToolpathEnteredVisitMemo(new(new(200,80,1200,5),0),maximumBytes:4096,maximumEntries:1);
        memo.Store(2,0,0,path,path);
        Assert.True(memo.TryGet(2,0,0,out _));Assert.False(memo.TryGet(2,0,-0d,out _));
        Assert.False(memo.TryGet(2,1,0,out _));Assert.False(memo.TryGet(3,0,0,out _));
        memo.Store(2,1,Math.BitIncrement(0d),path,path);
        Assert.False(memo.TryGet(2,0,0,out _));Assert.True(memo.TryGet(2,1,Math.BitIncrement(0d),out _));
        Assert.Equal(1,memo.Evictions);Assert.Equal(1,memo.Count);Assert.InRange(memo.BytesRetained,1,4096);
        var none=new ToolpathEnteredVisitMemo(new(new(200,80,1200,5),0),maximumBytes:0);
        none.Store(0,0,0,path,path);Assert.Equal(0,none.Count);Assert.Equal(1,none.BudgetRefusals);
    }

    [Fact]
    public void CachedGeometryIsDetachedAndForeignPriceOwnersUseTheirOwnLiteralRate()
    {
        var source=Path(0);var points=source.Points.ToArray();var entered=source with{Points=points};
        var firstCost=new ToolpathMotionCost(new(200,80,1200,5),0);var first=new ToolpathEnteredVisitMemo(firstCost);
        var entry=first.Store(0,0,0,source,entered);var old=entry.Entered.Points[1];points[1]=new(100,100,100);
        Assert.Equal(old,entry.Entered.Points[1]);
        var secondCost=new ToolpathMotionCost(new(400,100,1800,5),0);var second=new ToolpathEnteredVisitMemo(secondCost);
        Point3D origin=new(3,3,2);
        Assert.Equal(Bits(secondCost.CompleteVisitSeconds(origin,entry.Entered)),Bits(second.CompleteVisitSeconds(origin,entry)));
        Assert.Equal(0,second.PricedHits);Assert.Equal(0,second.PricedMisses);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void RouteMemoPreservesGeometryAndOpaqueHeightTravelAndPhysicalCallOrder(bool variableHeight)
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,6)],3);
        TaggedToolpath Cut(int owner,VCarveRingOrder.Plan plan,int ring,params Point3D[] points)
            =>new(points,[new(0,points.Length-1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false)
                {ContinuationSourceOrdinal=owner,ConcentricSource=new(plan,ring)};
        var first=new VCarveRingOrder.Plan([-1,0]);
        var source=new[]{Cut(0,first,0,new(0,0,-1),new(2,0,-1),new(2,2,-1),new(0,2,-1),new(0,0,-1)),
            Cut(1,first,1,new(20,0,-1.1),new(22,0,-1.1),new(22,2,-1.1),new(20,2,-1.1),new(20,0,-1.1)),
            Cut(2,new([-1]),0,Enumerable.Range(0,41).Select(i=>new Point3D(i-20,.2,-1)).ToArray()),
            new TaggedToolpath([new(100,100,-1),new(101,100,-1)],[],0,"support",0,detectCurves:false)};
        (List<TaggedToolpath> Paths,List<(string,Point3D,Point3D)> Calls,long Hits,long PricedHits,long ReturnLookups,long ReturnPriceHits) Run(bool enabled,bool refuseConnections=false)
        {
            bool? old=VCarveRingRouter.IndependentEntryMemoOverride.Value,oldRetrace=VCarvePathRouter.CutRetraceOptimizationOverride.Value;
            try
            {
                VCarveRingRouter.IndependentEntryMemoOverride.Value=enabled;VCarvePathRouter.CutRetraceOptimizationOverride.Value=false;
                var calls=new List<(string,Point3D,Point3D)>();var logs=new List<string>();int heights=0;
                var motion=new MillingMotionSettings(600,100,3000,5){ClearedTravelGuard=(a,b)=>{calls.Add(("travel",a,b));return false;}};
                var result=VCarveRingRouter.Route(source,new ToolGeometry(3){VCarveProfile=profile},new(3,3),new VCarveOptions{
                    SmoothVCarving=true,OptimizeContourConnections=true,DetectCurves=false,Motion=motion,
                    MaximumContourSeamCandidates=2,MaximumContourReturnCandidates=2,Ramp=new(Math.Atan(.5)),StepOver=.45,SmoothRampAngleDegrees=50},
                    null,(a,b)=>{calls.Add(("guard",a,b));return !refuseConnections;},p=>{calls.Add(("height",p,p));return variableHeight&&++heights%3==0?.05:0;},
                    default,logs.Add,(other,_)=>other.ToList());
                long Counter(string name)=>enabled?long.Parse(System.Text.RegularExpressions.Regex.Match(
                    Assert.Single(logs.Where(s=>s.StartsWith("stage=entered-visit-cost-memo "))),$@"(?:^| ){name}=(\d+)(?: |$)").Groups[1].Value):0;
                long ReturnCounter(string name)=>long.Parse(System.Text.RegularExpressions.Regex.Match(
                    Assert.Single(logs.Where(s=>s.StartsWith("stage=ring-routing event=complete "))),$@"(?:^| ){name}=(\d+)(?: |$)").Groups[1].Value);
                return(result,calls,Counter("hits"),Counter("pricedHits"),ReturnCounter("returnBudgetEntryLookups"),ReturnCounter("returnBudgetPriceHits"));
            }
            finally{VCarveRingRouter.IndependentEntryMemoOverride.Value=old;VCarvePathRouter.CutRetraceOptimizationOverride.Value=oldRetrace;}
        }
        var literal=Run(false);var cached=Run(true);Assert.Equal(literal.Calls,cached.Calls);
        Assert.Equal(literal.Paths.Count,cached.Paths.Count);
        for(int i=0;i<literal.Paths.Count;i++)
        {
            Assert.Equal(literal.Paths[i].Points,cached.Paths[i].Points);Assert.Equal(literal.Paths[i].Spans,cached.Paths[i].Spans);
            Assert.Equal(literal.Paths[i].RegionIndex,cached.Paths[i].RegionIndex);Assert.Equal(literal.Paths[i].DepthPassIndex,cached.Paths[i].DepthPassIndex);
            Assert.Equal(literal.Paths[i].ContinuationSourceOrdinal,cached.Paths[i].ContinuationSourceOrdinal);
        }
        if(!variableHeight)
        {
            // Refused new connectors make a still-pending source compete from
            // multiple departures, so the production memo hit is compulsory.
            var oldEntries=Run(false,true);var reusedEntries=Run(true,true);
            Assert.Equal(oldEntries.Calls,reusedEntries.Calls);
            Assert.Equal(oldEntries.Paths.Select(p=>p.Points),reusedEntries.Paths.Select(p=>p.Points));
            Assert.Equal(oldEntries.Paths.Select(p=>p.Spans),reusedEntries.Paths.Select(p=>p.Spans));
            Assert.True(reusedEntries.Hits>0);Assert.True(reusedEntries.PricedHits>0);
            Assert.Equal(oldEntries.ReturnLookups,reusedEntries.ReturnLookups);
            Assert.True(reusedEntries.ReturnLookups>0);Assert.True(reusedEntries.ReturnPriceHits>0);
        }
    }

    [Fact]
    public void InvalidBudgetsAreRejected()
    {
        var cost=new ToolpathMotionCost(new(200,80,1200,5),0);
        Assert.Throws<ArgumentOutOfRangeException>(()=>new ToolpathEnteredVisitMemo(cost,-1));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new ToolpathEnteredVisitMemo(cost,maximumEntries:-1));
    }
}
