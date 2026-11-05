using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveSpatialSweepTests
{
    private static TaggedToolpath Row(VCarveRingOrder.Plan plan,int ring,double x,double y=0,int pass=0,int? owner=null,int wall=-1,double length=.2)
        =>new([new(x,y,-pass-1),new(x+length,y,-pass-1)],[new(0,1,ToolpathSpanKind.Cut)],0,"clearing",pass,detectCurves:false)
        {ConcentricSource=new(plan,ring,Wall:wall),ContinuationSourceOrdinal=owner??ring};

    [Fact]
    public void NoSpatialSettingRetainsTheOrdinaryReadyPool()
    {
        var plan=new VCarveRingOrder.Plan([-1,-1,-1]);
        var paths=new[]{Row(plan,0,100),Row(plan,1,0),Row(plan,2,50)};
        var order=new VCarveRingOrder(paths,false,default,.5);
        Assert.Equal(new[]{0,1,2},order.IndependentCandidates());
        Assert.Equal(new[]{0},order.Candidates(null));Assert.Equal("spatialSweep=None",order.SpatialSummary());
    }

    [Theory]
    [InlineData(0,32)]
    [InlineData(0,64)]
    [InlineData(1,32)]
    [InlineData(1,64)]
    public void ReadyWindowsAdvanceAcrossTheRequestedAxis(int axisValue,double window)
    {
        var axis=(VCarveRingOrder.SpatialAxis)axisValue;
        var plan=new VCarveRingOrder.Plan([-1,-1,-1,-1]);
        double[] keys=[100,0,20,70];
        var paths=keys.Select((key,i)=>axis==VCarveRingOrder.SpatialAxis.X?Row(plan,i,key):Row(plan,i,0,key)).ToArray();
        var order=new VCarveRingOrder(paths,false,default,.5,new(axis,window));
        Assert.Equal(new[]{1,2},order.IndependentCandidates());
        order.Complete(1);Assert.Equal(window==64?new[]{2,3}:new[]{2},order.IndependentCandidates());
        order.Complete(2);Assert.Equal(new[]{0,3},order.IndependentCandidates());
        order.Complete(3);Assert.Equal(new[]{0},order.IndependentCandidates());order.Complete(0);
        Assert.Empty(order.IndependentCandidates());
        Assert.Contains("spatialFrontierAdvances=3 ",order.SpatialSummary());
        Assert.Contains("spatialDependencyEscapes=0 ",order.SpatialSummary());
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ABlockedSpatialFrontierUsesItsRealOffsetPredecessor(bool bottomUp)
    {
        var plan=new VCarveRingOrder.Plan([-1,0,-1]);
        var paths=new[]{Row(plan,0,0,length:2),Row(plan,1,bottomUp?.5:-.5,length:2),Row(plan,2,100)};
        var baseline=new VCarveRingOrder(paths,bottomUp,default,1);
        var order=new VCarveRingOrder(paths,bottomUp,default,1,new(VCarveRingOrder.SpatialAxis.X,.1));
        Assert.Equal(baseline.Dependencies().Order(),order.Dependencies().Order());
        int prerequisite=bottomUp?1:0,blocked=bottomUp?0:1;
        Assert.False(order.IsReady(blocked));Assert.Equal(new[]{prerequisite},order.IndependentCandidates());
        order.Complete(prerequisite);baseline.Complete(prerequisite);
        Assert.Equal(new[]{blocked},order.IndependentCandidates());
        order.Complete(blocked);baseline.Complete(blocked);
        Assert.Equal(new[]{2},order.IndependentCandidates());order.Complete(2);baseline.Complete(2);
        Assert.Contains("spatialDependencyEscapes=1 ",order.SpatialSummary());
        Assert.Contains("spatialDependencyFallbacks=0 ",order.SpatialSummary());
    }

    [Fact]
    public void WallLevelPrerequisitesCanSupportABlockedWindowWithoutRelaxation()
    {
        var plan=new VCarveRingOrder.Plan([-1,0]);
        var paths=new[]{Row(plan,0,50,wall:0),Row(plan,1,0,wall:0)};
        var order=new VCarveRingOrder(paths,false,default,.5,new(VCarveRingOrder.SpatialAxis.X,32));
        Assert.Equal(new[]{(0,1)},order.Dependencies());
        Assert.False(order.IsReady(1));Assert.Equal(new[]{0},order.IndependentCandidates());
        order.Complete(0);Assert.Equal(new[]{1},order.IndependentCandidates());order.Complete(1);
        Assert.Contains("spatialDependencyEscapes=1 ",order.SpatialSummary());
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SpatialSchedulingPreservesEveryDepthPassAndOffsetEdge(bool bottomUp)
    {
        var plan=new VCarveRingOrder.Plan([-1,0,-1]);
        var paths=new[]{Row(plan,0,0,pass:0),Row(plan,1,.3,pass:0),Row(plan,2,50,pass:0),
            Row(plan,0,0,pass:1),Row(plan,1,.3,pass:1),Row(plan,2,50,pass:1)};
        var baseline=new VCarveRingOrder(paths,bottomUp,default,.5);
        var order=new VCarveRingOrder(paths,bottomUp,default,.5,new(VCarveRingOrder.SpatialAxis.X,32));
        Assert.Equal(baseline.Dependencies().Order(),order.Dependencies().Order());var visited=new HashSet<int>();
        while(order.IndependentCandidates() is {Length:>0} ready){
            foreach(int candidate in ready)Assert.True(baseline.IsReady(candidate));
            int selected=ready[0];Assert.True(visited.Add(selected));order.Complete(selected);baseline.Complete(selected);
        }
        Assert.Equal(paths.Length,visited.Count);
    }

    [Fact]
    public void ACloseLocalContinuationStaysStickyInsideTheWindow()
    {
        var plan=new VCarveRingOrder.Plan([-1]);
        var paths=new[]{Row(plan,0,0,owner:0,length:1),Row(plan,0,1.1,owner:1),Row(plan,0,3,owner:2)};
        var order=new VCarveRingOrder(paths,false,default,.2,new(VCarveRingOrder.SpatialAxis.X,32));
        order.Complete(0);Assert.Equal(new[]{1,2},order.IndependentCandidates());
        Assert.Equal(new[]{1},order.Candidates(new(1,0,-1)));
        Assert.Contains("spatialStickyLocalPools=1 ",order.SpatialSummary());
    }

    [Fact]
    public void ALocalCandidateOutsideTheWindowCannotPullTheSweepBackAcrossTheBoard()
    {
        var plan=new VCarveRingOrder.Plan([-1]);
        var paths=new[]{Row(plan,0,100,owner:0,length:1),Row(plan,0,101.1,owner:1),Row(plan,0,0,owner:2)};
        var order=new VCarveRingOrder(paths,false,default,.2,new(VCarveRingOrder.SpatialAxis.X,32));
        order.Complete(0);Assert.Equal(new[]{2},order.Candidates(new(101,0,-1)));
        Assert.Equal(new[]{2},order.IndependentCandidates());
    }

    [Fact]
    public void ASourceSpanningSeveralWindowsIsRetainedAsOneCompleteJob()
    {
        var plan=new VCarveRingOrder.Plan([-1,-1]);
        var paths=new[]{Row(plan,0,0,length:200),Row(plan,1,50)};
        var order=new VCarveRingOrder(paths,false,default,.5,new(VCarveRingOrder.SpatialAxis.X,32));
        Assert.Equal(new[]{0},order.IndependentCandidates());order.Complete(0);
        Assert.Equal(new[]{1},order.IndependentCandidates());order.Complete(1);
        Assert.Equal(2,order.Count);Assert.Contains("spatialWindowSpanningJobs=1 ",order.SpatialSummary());
    }

    [Fact]
    public void ALongPrerequisiteChainHasABoundedSearchAndKeepsActualReadiness()
    {
        const int count=300;
        var plan=new VCarveRingOrder.Plan(Enumerable.Range(0,count).Select(i=>i-1));
        var paths=Enumerable.Range(0,count).Select(i=>Row(plan,i,count-i)).ToArray();
        var order=new VCarveRingOrder(paths,false,default,1,new(VCarveRingOrder.SpatialAxis.X,.1));
        Assert.Equal(new[]{0},order.IndependentCandidates());Assert.True(order.IsReady(0));Assert.False(order.IsReady(count-1));
        Assert.Contains("spatialDependencyBoundRefusals=1 ",order.SpatialSummary());
        Assert.Contains("spatialDependencyFallbacks=1 ",order.SpatialSummary());
        Assert.Contains("spatialSupportNodes=256 ",order.SpatialSummary());
    }

    [Fact]
    public void TheSpatialReadyQueryPropagatesCancellation()
    {
        using var cancel=new CancellationTokenSource();var plan=new VCarveRingOrder.Plan([-1]);
        var order=new VCarveRingOrder([Row(plan,0,0)],false,cancel.Token,.5,new(VCarveRingOrder.SpatialAxis.X,32));
        cancel.Cancel();Assert.ThrowsAny<OperationCanceledException>(()=>order.IndependentCandidates());
    }

    [Fact]
    public void RouterUsesTheSpatialWindowWithoutAddingUnguardedCutConnections()
    {
        var plan=new VCarveRingOrder.Plan([-1,-1,-1]);
        var source=new[]{Row(plan,0,100,length:4),Row(plan,1,0,length:4),Row(plan,2,50,length:4)};
        var oldSweep=VCarveRingOrder.SpatialSweepOverride.Value;
        var oldRetrace=VCarvePathRouter.CutRetraceOptimizationOverride.Value;
        try{
            VCarveRingOrder.SpatialSweepOverride.Value=new(VCarveRingOrder.SpatialAxis.X,32);
            VCarvePathRouter.CutRetraceOptimizationOverride.Value=false;
            var log=new List<string>();
            var actual=VCarveRingRouter.Route(source,new ToolGeometry(1){VCarveProfile=new([new(CarvingCutterSectionKind.Linear,0,1,2)],2)},
                new(2,2),new VCarveOptions{SmoothVCarving=true,OptimizeContourConnections=true,
                    Motion=new(1000,500,5000,5),Ramp=new(Math.Atan(.5)),StepOver=.45,
                    SmoothRampAngleDegrees=50,DetectCurves=false},new(101,0,0),(_,_)=>false,null,default,log.Add,(other,_)=>other.ToList());
            Assert.Equal(new[]{1,2,0},actual.Select(p=>p.ContinuationSourceOrdinal!.Value));
            Assert.DoesNotContain(actual.SelectMany(p=>p.Spans),s=>s.Kind==ToolpathSpanKind.Link);
            foreach(var original in source){
                var path=Assert.Single(actual.Where(p=>p.ContinuationSourceOrdinal==original.ContinuationSourceOrdinal));
                var kinds=ToolpathSpanKinds.Expand(path);var a=original.Points[0];var b=original.Points[^1];
                Assert.Contains(Enumerable.Range(1,path.Points.Count-1),i=>kinds[i-1]==ToolpathSpanKind.Cut
                    &&(path.Points[i-1]==a&&path.Points[i]==b||path.Points[i-1]==b&&path.Points[i]==a));
            }
            Assert.Contains(log,l=>l.Contains("stage=ring-spatial-sweep event=complete")&&l.Contains("chainTourSuppressed=True"));
            Assert.DoesNotContain(log,l=>l.Contains("stage=ring-tour"));
        }
        finally{VCarveRingOrder.SpatialSweepOverride.Value=oldSweep;VCarvePathRouter.CutRetraceOptimizationOverride.Value=oldRetrace;}
    }
}
