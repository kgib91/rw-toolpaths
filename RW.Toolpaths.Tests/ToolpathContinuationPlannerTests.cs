using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ToolpathContinuationPlannerTests
{
    private static TaggedToolpath Cut(params Point3D[] p) => new(p,[],0,"clearing",0,detectCurves:false);

    [Fact]
    public void BlockedDirectTravelRetracesOnlyExecutedNativeChords()
    {
        var planner=new ToolpathContinuationPlanner("test",2,default);
        Point3D a=new(0,0,-1),b=new(0,5,-1),c=new(5,5,-1);
        Assert.Null(planner.Connect(c,[a],(_,_)=>false));
        planner.Record(Cut(a,b,c));
        var connected=Assert.IsType<ToolpathContinuationPlanner.Connection>(planner.Connect(c,[a],(_,_)=>false));
        Assert.Equal(new[]{c,b,a},connected.Points);
        Assert.Equal("executed-native-retrace",connected.Kind);
    }

    [Fact]
    public void ExecutedRetraceStillRequiresThePostedCutterProof()
    {
        Point3D a=new(0,0,-1),b=new(0,5,-1),c=new(5,5,-1);
        var prior=Cut(a,b,c);
        var next=Cut(a,new(1,0,-1));
        var log=new List<string>();
        var checkedChords=new List<(Point3D,Point3D)>();
        var result=ToolpathContinuationPlanner.Reconnect([prior,next],"test",2,default,
            diagnostic:log.Add,validateConnection:(from,to)=>
            {
                checkedChords.Add((from,to));
                // A native execution graph is not the final authority for a
                // material-aware operation's independently posted cutter sweep.
                return to!=a;
            });
        Assert.Contains((b,a),checkedChords);
        Assert.Equal(next,result[1]);
        Assert.Contains(log,l=>l.Contains("event=connection-guard-refused")
            &&l.Contains("sourceEntryRetained=True"));
    }

    [Fact]
    public void XYOverlapDoesNotConnectDifferentHeightsOrFutureVisits()
    {
        var planner=new ToolpathContinuationPlanner("test",2,default);
        Point3D a=new(0,0,-1),b=new(1,0,-1),deepA=new(0,0,-2),deepB=new(1,0,-2);
        planner.Record(Cut(a,b));
        Assert.Null(planner.Connect(b,[deepB],(_,_)=>false));
        Assert.Null(planner.Connect(deepA,[a],(_,_)=>false));
        planner.RecordProvedAir(deepA,deepB,(_,_)=>false);
        Assert.Null(planner.Connect(b,[deepB],(_,_)=>false));
    }

    [Fact]
    public void AProvedAirNetworkCanShortenAnExecutedDetour()
    {
        var planner=new ToolpathContinuationPlanner("test",3,default);
        Point3D a=new(0,0,-1),b=new(0,20,-1),c=new(2,20,-1),d=new(2,0,-1);
        planner.Record(Cut(a,b,c,d));
        var connection=planner.Connect(d,[a],(x,y)=>x==d&&y==a);
        Assert.NotNull(connection);Assert.Equal(new[]{d,a},connection.Points);
    }

    [Fact]
    public void RemovingEntryRequiresFullLaterCutDominanceAndPreservesAllCutChords()
    {
        Point3D a=new(0,0,-1),b=new(2,0,-1);
        var prior=Cut(a,b);
        var next=new TaggedToolpath(new[]{new Point3D(2,0,0),new Point3D(0,0,-1),a,b},
            new[]{new ToolpathSpan(0,1,ToolpathSpanKind.Ramp),new ToolpathSpan(1,3,ToolpathSpanKind.Cut)},
            0,"clearing",0,detectCurves:false);
        var result=ToolpathContinuationPlanner.Reconnect([prior,next],"test",3,default,
            (_,_,_)=>true,removeSubsumedEntries:true,diagnostic:_=>{});
        Assert.Equal(prior.Points[^1],result[1].Points[0]);
        Assert.DoesNotContain(result[1].Spans,s=>s.Kind==ToolpathSpanKind.Ramp);
        Assert.Equal(next.Points.Skip(1),result[1].Points.TakeLast(3));
    }

    [Fact]
    public void EntryWithAnUncoveredProjectionCannotBeDiscarded()
    {
        var prior=Cut(new(0,0,-1),new(1,0,-1));
        var next=new TaggedToolpath(new[]{new Point3D(5,5,0),new Point3D(3,0,-1),new Point3D(4,0,-1)},
            new[]{new ToolpathSpan(0,1,ToolpathSpanKind.Ramp),new ToolpathSpan(1,2,ToolpathSpanKind.Cut)},
            0,"clearing",0,detectCurves:false);
        var result=ToolpathContinuationPlanner.Reconnect([prior,next],"test",3,default,
            (_,_,_)=>true,removeSubsumedEntries:true,diagnostic:_=>{});
        Assert.Contains(result[1].Spans,s=>s.Kind==ToolpathSpanKind.Ramp);
    }

    [Fact]
    public void ConnectedVCarveRingsAndTheirDepthLayersNeedOnlyOneSurfaceEntry()
    {
        TaggedToolpath Loop(double inset,double z,int pass) => new(new Point3D[]{
            new(inset,inset,z),new(20-inset,inset,z),new(20-inset,20-inset,z),
            new(inset,20-inset,z),new(inset,inset,z)},[],0,"clearing",pass,detectCurves:false);
        TaggedToolpath[] source=[Loop(2,-1,0),Loop(2.2,-1,0),Loop(2,-2,1),Loop(2.2,-2,1)];
        var log=new List<string>();
        var result=VCarvePathRouter.Route(source,[new PointD[]{new(0,0),new(20,0),new(20,20),new(0,20)}],
            new ToolGeometry(2){TipAngleRadians=2*Math.Atan(.2),BottomRadius=0,TopRadius=2,ConeLength=10},
            new DepthSchedule(2,1),new VCarveOptions{SmoothVCarving=true,SmoothRampAngleDegrees=30,
                DetectCurves=false},0,null,diagnostic:log.Add,continuationCutGuard:(_,_)=>true);
        Assert.Equal(0,result[0].Points[0].Z);
        for(int i=1;i<result.Count;i++)Assert.Equal(result[i-1].Points[^1],result[i].Points[0]);
        Assert.Contains(log,l=>l.Contains("event=complete owner=vcarve")&&l.Contains("entries=1 "));
        var executed=result.SelectMany(p=>p.Spans.Where(s=>s.Kind==ToolpathSpanKind.Cut)
            .SelectMany(s=>Enumerable.Range(s.StartIndex,s.MoveCount).Select(i=>(p.Points[i],p.Points[i+1])))).ToArray();
        foreach(var original in source)
        for(int i=1;i<original.Points.Count;i++)Assert.Contains((original.Points[i-1],original.Points[i]),executed);
    }

    [Fact]
    public void DisconnectedVCarveAreasKeepReasonedEntries()
    {
        var log=new List<string>();
        var sources=new[]{Cut(new(0,0,-1),new(1,0,-1)),Cut(new(20,0,-1),new(21,0,-1))};
        var result=VCarvePathRouter.Route(sources,[new PointD[]{new(-5,-5),new(30,-5),new(30,5),new(-5,5)}],
            new ToolGeometry(2){TipAngleRadians=2*Math.Atan(.2),BottomRadius=0,TopRadius=2,ConeLength=10},
            new DepthSchedule(1,1),new VCarveOptions{SmoothVCarving=true,DetectCurves=false},0,null,
            diagnostic:log.Add,continuationCutGuard:(_,_)=>false);
        Assert.All(result,p=>Assert.Equal(0,p.Points[0].Z));
        Assert.Contains(log,l=>l.Contains("event=reentry")&&l.Contains("no-proved-entry-connection"));
    }

    [Fact]
    public void CancellationIsNotReportedAsARequiredRetract()
    {
        using var stop=new CancellationTokenSource();stop.Cancel();
        var planner=new ToolpathContinuationPlanner("test",1,stop.Token);
        Assert.Throws<OperationCanceledException>(()=>planner.Connect(new(0,0,0),[new(1,0,0)],(_,_)=>true));
    }

    [Fact]
    public void SubFloatDriftCannotInventDistinctExecutedXYPoses()
    {
        var planner=new ToolpathContinuationPlanner("test",1,default);
        Point3D a=new(2,0,0),unreached=new(2+1e-9,0,-1),c=new(4,0,-1);
        planner.Record(Cut(a,unreached,c));
        Assert.Null(planner.Connect(c,[unreached],(_,_)=>false));
        Assert.Empty(planner.ClearedEntries(unreached,0,true));
    }

    [Fact]
    public void SmallSlopedCommandsKeepTheirExecutedPlanarPoseForRetracing()
    {
        var planner=new ToolpathContinuationPlanner("test",1,default);
        Point3D a=new(0,0,0),reached=new(.0008,0,-.0016),c=new(2,0,-.0016);
        Assert.True(planner.Record(Cut(a,reached,c)));
        Assert.NotNull(planner.Connect(c,[reached],(_,_)=>false));
    }

    [Fact]
    public void CertifiedComponentsFinishAllTheirDepthsBeforeAnotherComponent()
    {
        var source=new[]{Cut(new(0,0,-2),new(1,0,-2)),Cut(new(1.1,0,-2),new(2,0,-2)),
            Cut(new(20,0,-2),new(21,0,-2))};
        var assigned=ToolpathContinuationComponents.Assign(source,p=>p with{Z=Math.Max(-1,p.Z)},
            (_,_)=>true,.2,default);
        Assert.Equal(assigned[0].ContinuationComponentIndex,assigned[1].ContinuationComponentIndex);
        Assert.NotEqual(assigned[0].ContinuationComponentIndex,assigned[2].ContinuationComponentIndex);
        var scheduled=VCarveToolpaths.ScheduleFreshStockVisits(assigned,new DepthSchedule(2,1),default,null);
        var groups=scheduled.Select(p=>p.ContinuationComponentIndex).ToArray();
        Assert.Equal(groups.Order(),groups);
        Assert.All(scheduled.GroupBy(p=>p.ContinuationComponentIndex),g=>
            Assert.Equal(new[]{0,1},g.Select(p=>p.DepthPassIndex!.Value).Distinct()));
    }

    [Fact]
    public void TravelCostCanPreferAClearedDetourOverASlowerTouchingShortcut()
    {
        Point3D a=new(0,0,-2),b=new(0,1,-1.8),c=new(10,1,-1.8),d=new(10,0,-2);
        var planner=new ToolpathContinuationPlanner("test",2,default);
        planner.Record(Cut(a,b,c,d));
        var route=planner.Connect(a,[d],(x,y)=>x==a&&y==d,
            (x,y)=>x.Y==1&&y.Y==1,clearedTravelCostRatio:.2);
        Assert.NotNull(route);Assert.Equal(new[]{a,b,c,d},route.Points);
        Assert.Equal(new[]{false,true,false},route.ClearedTravel);
    }

    [Theory]
    [InlineData(3,true)]
    [InlineData(0,false)]
    [InlineData(.1,false)]
    public void FasterVCarveTravelRequiresFullBodyColumnsAndPositiveClearance(double columnRadius,bool faster)
    {
        var source=new[]{Cut(new(0,0,-2),new(1,0,-2)),Cut(new(10,0,-2),new(11,0,-2))};
        var result=VCarvePathRouter.Route(source,[new PointD[]{new(-5,-5),new(30,-5),new(30,5),new(-5,5)}],
            new ToolGeometry(2){TipAngleRadians=2*Math.Atan(.2),BottomRadius=0,TopRadius=2,ConeLength=10},
            new DepthSchedule(2,2),new VCarveOptions{SmoothVCarving=true,DetectCurves=false,
                CompactCollinearCommands=true,ClearedTravelCostRatio=.2,
                RoughingRestChords=[new(new(0,0,-3),new(20,0,-3),3,columnRadius,3)]},0,null);
        Assert.Equal(faster,result.Any(p=>p.Spans.Any(s=>s.Kind==ToolpathSpanKind.ClearedTravel)));
    }

    [Fact]
    public void AForestPortalSurvivesCompactionAndReachesTheInteriorOfACarrier()
    {
        var parent=Cut(new(2,2,-1),new(10,2,-1),new(18,2,-1),new(18,18,-1),new(2,18,-1),new(2,2,-1));
        var child=Cut(new(10,3,-1),new(10,2.1,-1));
        var assigned=ToolpathContinuationComponents.Assign([child,parent],p=>p with{Z=Math.Max(-1,p.Z)},
            (_,_)=>true,.4,default);
        // Isolate entry through a carrier body from the independent depth-ramp
        // regression above; a long exact carrier retrace must still obey cost.
        var source=VCarveToolpaths.ScheduleFreshStockVisits(assigned,new DepthSchedule(1,1),default,null);
        var log=new List<string>();
        var result=VCarvePathRouter.Route(source,[new PointD[]{new(0,0),new(20,0),new(20,20),new(0,20)}],
            new ToolGeometry(2){TipAngleRadians=2*Math.Atan(.2),BottomRadius=0,TopRadius=2,ConeLength=10},
            new DepthSchedule(1,1),new VCarveOptions{SmoothVCarving=true,DetectCurves=false,
                CompactCollinearCommands=true},0,null,diagnostic:log.Add,continuationCutGuard:(_,_)=>true);
        for(int i=1;i<result.Count;i++)Assert.Equal(result[i-1].Points[^1],result[i].Points[0]);
        Assert.Contains(log,l=>l.Contains("event=complete owner=vcarve")&&l.Contains("entries=1 "));
    }

    [Fact]
    public void StepdownEvidenceRequiresOneEarlierWholeChordAndBothCoordinateDomains()
    {
        var planner=new ToolpathContinuationPlanner("test",2,default);
        Point3D a=new(0,0,-2),b=new(1,0,-2);
        Point3D Command(Point3D p)=>p with{Z=Math.Round(p.Z,3)};
        Assert.False(planner.HasStepdownPrecursor(a,b,1,Command));
        planner.Record(Cut(a with{Z=-1},b with{Z=0}));
        planner.Record(Cut(a with{Z=0},b with{Z=-1}));
        Assert.False(planner.HasStepdownPrecursor(a,b,1,Command));
        planner.Record(Cut(a with{Z=-1},b with{Z=-1}));
        Assert.True(planner.HasStepdownPrecursor(a,b,1,Command));
        Assert.False(planner.HasStepdownPrecursor(a with{Z=-2.0004},b,1,Command));
        Assert.False(planner.HasStepdownPrecursor(a with{X=.0001},b,1,Command));
        Assert.False(planner.HasStepdownPrecursor(a,b,1,p=>p.Z==-2 ? p with{Z=-2.001} : p));
    }

    [Fact]
    public void AContinuationMustFitTheMotionCostOfTheEntryItReplaces()
    {
        var planner=new ToolpathContinuationPlanner("test",2,default);
        Point3D a=new(0,0,-1),b=new(0,10,-1),c=new(1,10,-1),d=new(1,0,-1);
        planner.Record(Cut(a,b,c,d));
        Assert.Null(planner.Connect(d,[a],(_,_)=>false,targetCosts:[5],maximumCost:10));
        Assert.Equal("no-proved-route-within-replaced-entry-cost",planner.LastFailure);
        Assert.NotNull(planner.Connect(d,[a],(_,_)=>false,targetCosts:[5],maximumCost:26));
    }

    [Fact]
    public void ReplacedEntryCostIncludesBothVerticalLegsAndTheSafeHeightTraverse()
    {
        Assert.Equal(3.4,ToolpathContinuationPlanner.SafeHeightTransferCost(new(0,0,-3),new(4,0,0),5,.2),10);
        Assert.Equal(0,ToolpathContinuationPlanner.SafeHeightTransferCost(new(0,0,-3),new(4,0,0),double.NaN,.2));
    }

    [Fact]
    public void EarlierStockCanProveARaisedEntryAwayFromTheRougherCenterline()
    {
        var log=new List<string>();
        var prior=Cut(new(0,1.1,-1.5),new(1,1.1,-1.5));
        var next=Cut(new(2.5,1.1,-3),new(3.5,1.1,-3));
        var adapter=new VCarveContinuationAdapter(
            new ToolGeometry(2){TipAngleRadians=2*Math.Atan(.4),BottomRadius=0,TopRadius=2,ConeLength=5},
            new DepthSchedule(3,3),new VCarveOptions{SmoothVCarving=true,DetectCurves=false,
                SmoothRampAngleDegrees=30,CompactCollinearCommands=true,ClearedTravelCostRatio=.1,
                RoughingRestChords=[new(new(0,0,-4),new(20,0,-4),2,2,4)]},(_,_)=>false,default,log.Add,[prior,next]);
        adapter.Record(prior);
        var connection=adapter.Connect(prior.Points[^1],ref next);
        Assert.NotNull(connection);
        Assert.Equal(new Point3D(2.5,1.1,-1.5),connection.Points[^1]);
        Assert.All(connection.ClearedTravel!,p=>Assert.True(p));
        adapter.Report();
        Assert.Contains(log,l=>l.Contains("stockProvedEntryPoses=")&&!l.Contains("stockProvedEntryPoses=0 "));
    }

    [Fact]
    public void ABodyPortalUsesItsExecutedCurrentDepthEvenWhenCoarseNominationMissesIt()
    {
        Point3D portal=new(1.1,0,-1),childPortal=new(1.1,.1,-1);
        var parent=Cut(new(0,0,-2),new(1.01,0,-2),portal with{Z=-2},new(0,0,-2));
        var child=Cut(childPortal with{Z=-2},new(2,.1,-2));
        var plan=new ToolpathContinuationComponents.Plan(
            [new(new(0,portal),new(1,childPortal))],[parent,child],default);
        parent=parent with{ContinuationPlan=plan,ContinuationSourceOrdinal=0};
        child=child with{ContinuationPlan=plan,ContinuationSourceOrdinal=1};
        var adapter=new VCarveContinuationAdapter(
            new ToolGeometry(2){TipAngleRadians=2*Math.Atan(.2),BottomRadius=0,TopRadius=2,ConeLength=10},
            new DepthSchedule(2,1),new VCarveOptions{SmoothVCarving=true,DetectCurves=false},
            (_,_)=>true,default,null,[parent,child]);
        // The complete shallow connector was really executed. Its parent body
        // pose at -2 shares a nomination voxel with 1.01, but remains a literal
        // graph node. The next layer can reuse it without climbing back to -1.
        adapter.Record(Cut(portal,childPortal));
        adapter.Record(parent);
        var connection=adapter.Connect(parent.Points[^1],ref child);
        Assert.NotNull(connection);
        Assert.All(connection.Points,p=>Assert.Equal(-2,p.Z));
        Assert.Contains(portal with{Z=-2},connection.Points);
    }
}
