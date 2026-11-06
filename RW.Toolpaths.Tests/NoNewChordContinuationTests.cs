using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NoNewChordContinuationTests
{
    private static TaggedToolpath Cut(params Point3D[] points)=>new(points,[],0,"profile",0,detectCurves:false);
    private static readonly Point3D A=new(0,0,-1),B=new(0,5,-1),C=new(5,5,-1);

    [Fact]
    public void AbsentAuthorityKeepsTheExactNativeRetraceAndSkipsNewChordDiscovery()
    {
        var control=Run(true);var absent=Run(false);
        Assert.Equal(new[]{C,B,A},control.Connection.Points);
        Assert.Equal(control.Connection.Points,absent.Connection.Points);
        Assert.Equal(control.Connection.Target,absent.Connection.Target);
        Assert.Equal(control.Connection.Kind,absent.Connection.Kind);
        Assert.True(control.Proofs>0);Assert.Equal(0,absent.Proofs);
        Assert.Contains(absent.Log,s=>s.Contains("noNewChordQueries=1 ")&&s.Contains("visibilityProofs=0 ")
            &&!s.Contains("noNewChordGoalNominationPassesSkipped=0 ")
            &&!s.Contains("noNewChordVisibilityNominationPassesSkipped=0 "));

        (ToolpathContinuationPlanner.Connection Connection,int Proofs,List<string> Log) Run(bool capability)
        {
            int proofs=0;var log=new List<string>();var planner=new ToolpathContinuationPlanner("no-new",10,default,log.Add);
            planner.Record(Cut(A,B,C));
            var result=planner.Connect(C,[A],(_,_)=>{proofs++;return false;},
                preferredEntryPortals:[B],newChordAuthorityAvailable:capability);
            planner.Report();return(Assert.IsType<ToolpathContinuationPlanner.Connection>(result),proofs,log);
        }
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CoincidentExactGoalsNeedNoNewChordPermission(bool capability)
    {
        int proofs=0;var planner=new ToolpathContinuationPlanner("coincident",2,default);
        var result=planner.Connect(A,[A],(_,_)=>{proofs++;return false;},newChordAuthorityAvailable:capability);
        Assert.NotNull(result);Assert.Equal(new[]{A},result!.Points);Assert.Equal("coincident",result.Kind);Assert.Equal(0,proofs);
    }

    [Theory]
    [InlineData(.1,false)]
    [InlineData(100,true)]
    public void LiteralGraphCostsAndRetainedTargetPricesStayAuthoritative(double maximum,bool expected)
    {
        var reference=Run(true);var absent=Run(false);
        Assert.Equal(expected,reference is not null);Assert.Equal(expected,absent is not null);
        if(expected)Assert.Equal(reference!.Points,absent!.Points);
        ToolpathContinuationPlanner.Connection? Run(bool capability)
        {
            var planner=new ToolpathContinuationPlanner("cost",10,default);planner.Record(Cut(A,B,C));
            return planner.Connect(C,[A],(_,_)=>false,targetCosts:[2],maximumCost:maximum,
                motionCost:new(new MillingMotionSettings(60,60,60,2),0),newChordAuthorityAvailable:capability);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NearbyOrSameXYDifferentDepthNetworksCannotBecomeLiteralConnections(bool changeDepth)
    {
        Point3D a=new(0,0,-1),b=new(1,0,-1),c=changeDepth?new(1,0,-2):new(1.01,0,-1),
            d=changeDepth?new(2,0,-2):new(2,0,-1);
        foreach(bool capability in new[]{true,false})
        {
            var planner=new ToolpathContinuationPlanner("disconnected",3,default);planner.Record(Cut(a,b));planner.Record(Cut(c,d));
            Assert.Null(planner.Connect(b,[d],(_,_)=>false,newChordAuthorityAvailable:capability));
        }
    }

    [Fact]
    public void OmittedXYCursorDriftCannotCreateAnExactExecutedGoal()
    {
        Point3D a=new(2,0,0),unreached=new(2+1e-9,0,-1),c=new(4,0,-1);
        foreach(bool capability in new[]{true,false})
        {
            var planner=new ToolpathContinuationPlanner("partial-axis",3,default);planner.Record(Cut(a,unreached,c));
            Assert.Null(planner.Connect(c,[unreached],(_,_)=>false,newChordAuthorityAvailable:capability));
        }
    }

    [Fact]
    public void ReconnectInfersCapabilityOnlyFromAnActuallyAbsentAdapterAndPreservesAllOutput()
    {
        var paths=new[]{Cut(A,B,C),Cut(A,new(1,0,-1)),Cut(new(20,0,-1),new(21,0,-1))};
        var control=Run(false);var absent=Run(true);
        Assert.Equal(control.Output.Count,absent.Output.Count);
        for(int i=0;i<control.Output.Count;i++)
        {
            Assert.Equal(control.Output[i].Points,absent.Output[i].Points);Assert.Equal(control.Output[i].Spans,absent.Output[i].Spans);
            Assert.Equal(control.Output[i].RegionIndex,absent.Output[i].RegionIndex);
            Assert.Equal(control.Output[i].DepthPassIndex,absent.Output[i].DepthPassIndex);
            Assert.Equal(control.Output[i].VisitIdentity,absent.Output[i].VisitIdentity);
        }
        Assert.True(control.Proofs>0);Assert.Equal(0,absent.Proofs);
        Assert.Contains(control.Log,s=>s.Contains("event=complete owner=null-adapter")&&s.Contains("noNewChordQueries=0 "));
        Assert.Contains(absent.Log,s=>s.Contains("[Toolpath.Continuation.Perf]")&&s.Contains("newChordAuthorityAvailable=False")
            &&s.Contains("enteredDetoursMs=")&&s.Contains("connectMs=")&&s.Contains("recordMs="));

        (List<TaggedToolpath> Output,int Proofs,List<string> Log) Run(bool missing)
        {
            int proofs=0;var log=new List<string>();var recorded=new List<TaggedToolpath>();
            Func<TaggedToolpath,Point3D,Point3D,bool>? adapter=missing?null:(_,_,_)=>{proofs++;return false;};
            var output=ToolpathContinuationPlanner.Reconnect(paths,"null-adapter",10,default,adapter,
                diagnostic:log.Add,recordExecuted:recorded.Add,motion:new(1000,500,3000,2));
            Assert.Equal(output,recorded);return(output,proofs,log);
        }
    }

    [Fact]
    public void ExistingConnectionGuardCanStillRefuseALiteralRetrace()
    {
        var paths=new[]{Cut(A,B,C),Cut(A,new(1,0,-1))};
        var log=new List<string>();var checkedChords=new List<(Point3D,Point3D)>();
        var output=ToolpathContinuationPlanner.Reconnect(paths,"guarded-null",10,default,
            diagnostic:log.Add,validateConnection:(from,to)=>{checkedChords.Add((from,to));return to!=A;});
        Assert.Contains((B,A),checkedChords);Assert.Equal(paths[1],output[^1]);
        Assert.Contains(log,s=>s.Contains("event=connection-guard-refused"));
    }

    [Fact]
    public void CancellationStillPreventsPublishingTheAbsenceOptimizedRoute()
    {
        using var canceled=new CancellationTokenSource();canceled.Cancel();
        var planner=new ToolpathContinuationPlanner("canceled",2,canceled.Token);
        Assert.ThrowsAny<OperationCanceledException>(()=>planner.Connect(A,[A],(_,_)=>false,newChordAuthorityAvailable:false));
        using var during=new CancellationTokenSource();int recorded=0;
        Assert.ThrowsAny<OperationCanceledException>(()=>ToolpathContinuationPlanner.Reconnect(
            [Cut(A,B,C),Cut(new(20,0,-1),new(21,0,-1))],"cancel-record",10,during.Token,
            diagnostic:_=>{},recordExecuted:_=>{recorded++;during.Cancel();}));
        Assert.Equal(1,recorded);
    }
}
