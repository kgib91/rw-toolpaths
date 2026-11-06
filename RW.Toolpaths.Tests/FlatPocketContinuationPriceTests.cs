using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class FlatPocketContinuationPriceTests
{
    private static TaggedToolpath Cut(params Point3D[] points)=>new(points,[],0,"clearing",0,detectCurves:false);
    private static ToolpathMotionCost Price()=>new(new MillingMotionSettings(60,60,600,2),0);

    [Theory]
    [InlineData(false,1)]
    [InlineData(true,0)]
    public void UnaffordableDirectNominationKeepsEntryWithoutStockProof(bool early,int expectedProofs)
    {
        int proofs=0;var log=new List<string>();
        var planner=new ToolpathContinuationPlanner("flat-price",20,default,log.Add);
        var result=planner.Connect(new(0,0,-1),[new(100,0,-1)],(_,_)=>{proofs++;return true;},
            targetCosts:[0],maximumCost:1,motionCost:Price(),costBeforeNewChord:early);
        Assert.Null(result);Assert.Equal(expectedProofs,proofs);
        planner.Report();
        Assert.Contains(log,line=>line.Contains($"costFirstDirectProofSkips={(early?1:0)} "));
    }

    [Fact]
    public void GoalLowerBoundSkipsOnlyUnaffordableNewChords()
    {
        Point3D from=new(0,0,-1),portal=new(9,0,-1),target=new(10,0,-1);
        int proofs=0;var log=new List<string>();
        var planner=new ToolpathContinuationPlanner("flat-price",20,default,log.Add);
        planner.Record(Cut(from,portal));
        var result=planner.Connect(from,[target],(_,_)=>{proofs++;return true;},
            targetCosts:[0],preferredEntryPortals:[portal],maximumCost:1,motionCost:Price(),costBeforeNewChord:true);
        Assert.Null(result);Assert.Equal(0,proofs);planner.Report();
        Assert.Contains(log,line=>line.Contains("costFirstDirectProofSkips=1 ")
            &&!line.Contains("costFirstGoalProofSkips=0 "));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(2)]
    public void CompetitiveGraphRouteHasSameCompletePathAndStillRequiresStockProof(double tailCost)
    {
        Point3D from=new(0,0,-1),portal=new(3,0,-1),target=new(4,0,-1);
        var reference=Run(false);var filtered=Run(true);
        Assert.Equal(new[]{from,portal,target},reference.Points);
        Assert.Equal(reference.Points,filtered.Points);Assert.Equal(reference.Target,filtered.Target);

        ToolpathContinuationPlanner.Connection Run(bool early)
        {
            var planner=new ToolpathContinuationPlanner("flat-price",5,default);
            planner.Record(Cut(from,portal));int admittedProofs=0;
            var result=planner.Connect(from,[target],(a,b)=>
            {
                bool accepted=a==portal&&b==target;if(accepted)admittedProofs++;return accepted;
            },targetCosts:[tailCost],maximumCost:5+tailCost,motionCost:Price(),costBeforeNewChord:early);
            Assert.True(admittedProofs>0);
            return Assert.IsType<ToolpathContinuationPlanner.Connection>(result);
        }
    }

    [Fact]
    public void ExactEntryCostTieStillReceivesItsFullProof()
    {
        int proofs=0;Point3D from=new(0,0,-1),target=new(4,0,-1);
        var result=new ToolpathContinuationPlanner("flat-price",5,default).Connect(from,[target],
            (_,_)=>{proofs++;return true;},targetCosts:[2],maximumCost:6,motionCost:Price(),costBeforeNewChord:true);
        Assert.NotNull(result);Assert.Equal(new[]{from,target},result!.Points);Assert.Equal(1,proofs);
    }

    [Fact]
    public void FloatPricedCompetitiveGoalIsNotPrunedByLargerNativeDoubleDistance()
    {
        Point3D from=new(1000.00004,0,-1),portal=new(1001.00002,0,-1),target=new(1002,0,-1);
        var cost=new ToolpathMotionCost(new MillingMotionSettings(60,60,60,2),0);
        double maximum=cost.ChordSeconds(from,portal)+cost.ChordSeconds(portal,target);
        double nativeLower=portal.X-from.X+cost.ChordSeconds(portal,target);
        Assert.True(nativeLower>maximum+1e-9);
        var checkedChords=new List<(Point3D,Point3D)>();
        var planner=new ToolpathContinuationPlanner("flat-float-price",5,default);
        planner.Record(Cut(from,portal));
        planner.Connect(from,[target],(a,b)=>
        {
            checkedChords.Add((a,b));return a==portal&&b==target;
        },targetCosts:[0],preferredEntryPortals:[portal],maximumCost:maximum,
            motionCost:cost,costBeforeNewChord:true);
        // This regression concerns proof admission. The existing generic A*
        // native-coordinate heuristic remains an independent, unchanged policy.
        Assert.Contains((portal,target),checkedChords);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OpaqueTravelCallbackRetainsOriginalGuardFirstTraversal(bool early)
    {
        var calls=new List<string>();var log=new List<string>();
        var planner=new ToolpathContinuationPlanner("opaque-travel",5,default,log.Add);
        Assert.Null(planner.Connect(new(0,0,-1),[new(10,0,-1)],
            (_,_)=>{calls.Add("stock");return true;},
            positiveTravelClearance:(_,_)=>{calls.Add("travel");return true;},
            targetCosts:[0],maximumCost:.01,motionCost:Price(),costBeforeNewChord:early));
        Assert.Equal(new[]{"stock","travel"},calls);planner.Report();
        Assert.Contains(log,line=>line.Contains($"costFirstTravelBypasses={(early?1:0)} "));
    }

    [Fact]
    public void WithoutMachinePriceTheOriginalProofOrderRemains()
    {
        int proofs=0;
        var result=new ToolpathContinuationPlanner("no-price",5,default).Connect(new(0,0,-1),[new(1,0,-1)],
            (_,_)=>{proofs++;return true;},targetCosts:[0],costBeforeNewChord:true);
        Assert.NotNull(result);Assert.Equal(1,proofs);
    }

    [Fact]
    public void RefusedConnectionsKeepEveryDirectedCutAndIndependentEntry()
    {
        var first=Cut(new(0,0,-1),new(1,0,-1));
        var second=new TaggedToolpath(new Point3D[]{new(5,0,0),new(5,0,-1),new(6,0,-1)},
            [new(0,1,ToolpathSpanKind.Ramp),new(1,2,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);
        var recorded=new List<TaggedToolpath>();
        var result=ToolpathContinuationPlanner.Reconnect([first,second],"flat-price",5,default,
            allowsNewChord:(_,_,_)=>false,removeSubsumedEntries:true,diagnostic:_=>{},
            recordExecuted:recorded.Add,motion:new(60,60,600,2),surface:0,costBeforeNewChord:true);
        Assert.Equal(new[]{first,second},result);Assert.Equal(result,recorded);
        Assert.Equal(ToolpathSpanKind.Ramp,result[1].Spans[0].Kind);
        Assert.Equal(second.Points,result[1].Points);Assert.Equal(second.Spans,result[1].Spans);
    }
}
