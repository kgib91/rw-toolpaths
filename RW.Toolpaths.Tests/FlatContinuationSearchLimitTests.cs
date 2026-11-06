using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class FlatContinuationSearchLimitTests
{
    private static TaggedToolpath Cut(params Point3D[] points)=>new(points,[],0,"clearing",0,detectCurves:false);
    private static Point3D[] Graph()=>Enumerable.Range(0,13).Select(i=>new Point3D(i,0,-1)).ToArray();

    [Fact]
    public void LibraryDefaultsKeepTheSameOpaqueProofTraversalAndOutput()
    {
        Assert.Equal(48,new PocketOptions().FlatContinuationNearbyGoals);
        Assert.Equal(100_000,new PocketOptions().FlatContinuationVisibilityProofs);
        var original=Run(null);var defaults=Run(new(48,100_000));
        Assert.Equal(original.Points,defaults.Points);Assert.Equal(original.Proofs,defaults.Proofs);
        (IReadOnlyList<Point3D> Points,List<(Point3D,Point3D)> Proofs) Run(ToolpathContinuationSearchLimits? limits)
        {
            var points=Graph();var proofs=new List<(Point3D,Point3D)>();
            var planner=new ToolpathContinuationPlanner("default",20,default,searchLimits:limits);planner.Record(Cut(points));
            var result=planner.Connect(points[^1],[points[0]],(a,b)=>{proofs.Add((a,b));return false;});
            return(Assert.IsType<ToolpathContinuationPlanner.Connection>(result).Points,proofs);
        }
    }

    [Fact]
    public void NearbyGoalLimitStopsBeforeAdditionalCutterProofCalls()
    {
        int proofs=0;var log=new List<string>();
        var planner=new ToolpathContinuationPlanner("nearby",20,default,log.Add,new(8,0));
        Point3D from=new(-1,0,-1);planner.Record(Cut(from,new(-.5,0,-1)));
        for(int i=0;i<60;i++)planner.Record(Cut(new(i*.05,.1,-1),new(i*.05,.15,-1)));
        Assert.Null(planner.Connect(from,[new(2.2,.12,-1)],(_,_)=>{proofs++;return false;}));
        Assert.Equal(9,proofs); // One unchanged direct nomination plus eight nearby goals.
        planner.Report();
        Assert.Contains(log,s=>s.Contains("nearbyGoalsPerTarget=8 ")&&s.Contains("nearbyGoalLimitExcluded=40 "));
    }

    [Fact]
    public void OperationVisibilityLimitIsSharedAndExactNativeGoalsStillRetrace()
    {
        int proofs=0;var log=new List<string>();var points=Graph();
        var planner=new ToolpathContinuationPlanner("visibility",20,default,log.Add,new(0,2));planner.Record(Cut(points));
        for(int i=0;i<2;i++)
        {
            var result=planner.Connect(points[^1],[points[0]],(_,_)=>{proofs++;return false;});
            Assert.NotNull(result);Assert.Equal(points.Reverse(),result!.Points);
        }
        Assert.Equal(4,proofs); // Two direct proofs and exactly two operation-wide visibility proofs.
        planner.Report();
        Assert.Contains(log,s=>s.Contains("visibilityProofs=2 ")&&s.Contains("operationVisibilityProofLimit=2 ")
            &&s.Contains("searchNodesPerQuery=4096 ")&&!s.Contains("visibilityLimitStoppedPasses=0 "));
    }

    [Fact]
    public void ZeroOptionalSearchRetainsDirectWholeProofAndCoincidentGoals()
    {
        int proofs=0;Point3D a=new(0,0,-1),b=new(.25,0,-1);
        var planner=new ToolpathContinuationPlanner("direct",2,default,searchLimits:new(0,0));
        var direct=planner.Connect(a,[b],(_,_)=>{proofs++;return true;});
        Assert.NotNull(direct);Assert.Equal(new[]{a,b},direct!.Points);Assert.Equal(1,proofs);
        var coincident=planner.Connect(a,[a],(_,_)=>throw new InvalidOperationException("Coincident goal needs no new chord."));
        Assert.NotNull(coincident);Assert.Equal(new[]{a},coincident!.Points);
    }

    [Fact]
    public void ZeroDiscoverySkipsPortalGatheringButRetainsLiteralNativeRetrace()
    {
        var points=Graph();var log=new List<string>();int proofs=0;
        var planner=new ToolpathContinuationPlanner("zero",20,default,log.Add,new(0,0));
        planner.Record(Cut(points));
        var result=planner.Connect(points[^1],[points[0]],(_,_)=>{proofs++;return false;});
        Assert.NotNull(result);Assert.Equal(points.Reverse(),result!.Points);Assert.Equal(1,proofs);
        planner.Report();
        Assert.Contains(log,s=>s.Contains("nearbyGoalNominationPassesSkipped=1 ")
            &&s.Contains("visibilityProofs=0 ")&&s.Contains("connectDirectMs=")
            &&s.Contains("connectGoalMs=")&&s.Contains("connectAStarMs="));
    }

    [Fact]
    public void ReconnectKeepsEverySourceCutAndItsFinalConnectionGuard()
    {
        Point3D a=new(0,0,-1),b=new(1,0,-1),c=new(1.1,0,-1),d=new(2,0,-1);
        var source=new[]{Cut(a,b),Cut(c,d)};
        foreach(bool guardAllows in new[]{true,false})
        {
            int proofs=0,guards=0;
            var result=ToolpathContinuationPlanner.Reconnect(source,"bounded",3,default,
                allowsNewChord:(_,_,_)=>{proofs++;return true;},diagnostic:_=>{},
                validateConnection:(_,_)=>{guards++;return guardAllows;},searchLimits:new(0,0));
            Assert.True(proofs>0);Assert.True(guards>0);
            var cutChords=result.SelectMany(path=>Enumerable.Range(1,path.Points.Count-1)
                .Where(i=>ToolpathSpanKinds.Expand(path)[i-1]==ToolpathSpanKind.Cut)
                .Select(i=>(path.Points[i-1],path.Points[i]))).ToArray();
            Assert.Contains((a,b),cutChords);Assert.Contains((c,d),cutChords);
            if(!guardAllows)Assert.Equal(source[1],result[^1]);
        }
    }

    [Theory]
    [InlineData(-1,0)]
    [InlineData(49,0)]
    [InlineData(0,-1)]
    [InlineData(0,100001)]
    public void SearchLimitsAreExplicitlyValidated(int goals,int visibility)
        =>Assert.Throws<ArgumentOutOfRangeException>(()=>new ToolpathContinuationSearchLimits(goals,visibility));

    [Fact]
    public void BoundedSearchCannotSwallowCancellation()
    {
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        var planner=new ToolpathContinuationPlanner("cancel",2,cancel.Token,searchLimits:new(0,0));
        Assert.ThrowsAny<OperationCanceledException>(()=>planner.Connect(new(0,0,-1),[new(1,0,-1)],(_,_)=>true));
    }
}
