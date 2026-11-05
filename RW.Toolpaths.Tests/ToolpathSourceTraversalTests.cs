using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ToolpathSourceTraversalTests
{
    private static TaggedToolpath Cut(params Point3D[] p)=>new(p,[],0,"clearing",0,detectCurves:false);

    [Fact]
    public void LiteralCircuitPortalsRetainBothEndsOfAShortBranchConnector()
    {
        var carrier=Cut(new(0,0,-1),new(5,0,-1),new(10,0,-1),new(10,10,-1),new(0,10,-1),new(0,0,-1));
        var branch=Cut(new(5,.00001,-1),new(5,3,-1),new(5,.00001,-1));
        var link=Cut(new(5,0,-1),new(5,.00001,-1),new(5,0,-1));
        var logs=new List<string>();
        var route=Assert.Single(ToolpathCircuitBuilder.Build([carrier,branch,link],(_,_)=>false,.001,default,logs.Add,
            preserveSourceOrder:true,literalPortalsOnly:true));
        Assert.Equal(carrier.Points[0],route.Points[0]);
        Assert.Equal(route.Points[0],route.Points[^1]);
        Assert.Contains(new Point3D(5,3,-1),route.Points);
        double Length(TaggedToolpath p)=>Enumerable.Range(1,p.Points.Count-1)
            .Sum(i=>ToolpathContinuationPlanner.Distance(p.Points[i-1],p.Points[i]));
        Assert.Equal(Length(carrier)+Length(branch)+Length(link),Length(route),8);
        Assert.Contains(logs,l=>l.Contains("parityDuplicates=0 "));
    }

    [Fact]
    public void LiteralCircuitModeDoesNotBridgeNearbyDisjointStrokes()
    {
        var first=Cut(new(0,0,-1),new(1,0,-1),new(0,0,-1));
        var second=Cut(new(1,.00001,-1),new(2,0,-1),new(1,.00001,-1));
        int proposals=0;
        var routes=ToolpathCircuitBuilder.Build([first,second],(_,_)=>{proposals++;return true;},.001,default,
            preserveSourceOrder:true,literalPortalsOnly:true);
        Assert.Equal(2,routes.Count);Assert.Equal(0,proposals);
    }

    [Fact]
    public void SeparateOutwardAndReturnSourcesCompleteOneBranchExcursion()
    {
        Point3D portal=new(5,0,-1),firstStem=new(5,.00001,-1),secondStem=new(5,.000011,-1),tip=new(5,3,-1);
        var carrier=Cut(new(0,0,-1),portal,new(10,0,-1),new(10,10,-1),new(0,10,-1),new(0,0,-1));
        var route=Assert.Single(ToolpathCircuitBuilder.Build([carrier,Cut(firstStem,tip),Cut(tip,secondStem),
            Cut(portal,firstStem,portal),Cut(portal,secondStem,portal)],(_,_)=>false,.001,default,
            preserveSourceOrder:true,literalPortalsOnly:true));
        int Count(Point3D a,Point3D b)=>Enumerable.Range(1,route.Points.Count-1).Count(i=>
            route.Points[i-1]==a&&route.Points[i]==b||route.Points[i-1]==b&&route.Points[i]==a);
        Assert.Equal(1,Count(firstStem,tip));Assert.Equal(1,Count(tip,secondStem));
        Assert.Equal(1,Count(carrier.Points[0],portal));
        Assert.Equal(carrier.Points[0],route.Points[0]);Assert.Equal(route.Points[0],route.Points[^1]);
    }

    [Fact]
    public void InteriorBranchIsServicedBeforeCarrierLeavesWithNoInventedChord()
    {
        var carrier=Cut(new(0,0,-1),new(10,0,-1),new(10,10,-1),new(0,10,-1),new(0,0,-1));
        var branch=Cut(new(5,0,-1),new(5,3,-1));
        var log=new List<string>();
        var route=Assert.Single(ToolpathSourceTraversal.Detours([carrier,branch],default,log.Add));
        var points=route.Points.ToList();
        int portal=points.IndexOf(new(5,0,-1));
        Assert.True(portal>=0);
        Assert.Equal(new Point3D(5,3,-1),points[portal+1]);
        Assert.Equal(new Point3D(5,0,-1),points[portal+2]);
        Assert.Equal(carrier.Points[0],points[0]);Assert.Equal(points[0],points[^1]);
        Assert.All(Enumerable.Range(1,points.Count-1),i=>Assert.True(
            points[i-1].Y==0&&points[i].Y==0 || points[i-1].X==10&&points[i].X==10
            ||points[i-1].Y==10&&points[i].Y==10||points[i-1].X==0&&points[i].X==0
            ||points[i-1].X==5&&points[i].X==5));
        Assert.Contains(log,l=>l.Contains("introducedChords=0")&&l.Contains("merged=1 "));
    }

    [Fact]
    public void CollinearContactUsesOriginalStrokesAndDoesNotLoseTheCarrierExit()
    {
        var a=Cut(new(0,0,-1),new(10,0,-1));var b=Cut(new(5,0,-1),new(7,0,-1));
        var route=Assert.Single(ToolpathSourceTraversal.Detours([a,b],default));
        Assert.Equal(a.Points[0],route.Points[0]);Assert.Equal(a.Points[^1],route.Points[^1]);
        Assert.Contains(b.Points[0],route.Points);Assert.Contains(b.Points[^1],route.Points);
    }

    [Fact]
    public void DifferentHeightsAndRefusedPostedSplitsStayDisconnected()
    {
        var carrier=Cut(new(0,0,-1),new(10,0,-1));
        Assert.Equal(2,ToolpathSourceTraversal.Detours([carrier,Cut(new(5,0,-2),new(5,3,-2))],default).Count);
        Assert.Equal(2,ToolpathSourceTraversal.Detours([carrier,Cut(new(5,0,-1),new(5,3,-1))],default,
            allowsSplit:(_,_)=>false).Count);
    }

    [Fact]
    public void ABranchCannotJoinTwoClosedCarriersAndChangeRingOrder()
    {
        var first=Cut(new(0,0,-1),new(10,0,-1),new(10,10,-1),new(0,10,-1),new(0,0,-1));
        var second=Cut(new(1,1,-1),new(9,1,-1),new(9,9,-1),new(1,9,-1),new(1,1,-1));
        var branch=Cut(new(5,0,-1),new(5,1,-1));
        var routes=ToolpathSourceTraversal.Detours([first,second,branch],default);
        Assert.Equal(2,routes.Count);Assert.Equal(first.Points[0],routes[0].Points[0]);
        Assert.Equal(second.Points,routes[1].Points);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(12.7)]
    public void CommonFinalRouterKeepsCarrierRampAndEliminatesBranchAirEntry(double surface)
    {
        var carrier=new TaggedToolpath(new Point3D[]{new(0,0,0),new(0,0,-1),new(10,0,-1)},
            new ToolpathSpan[]{new(0,1,ToolpathSpanKind.Ramp),new(1,2,ToolpathSpanKind.Cut)},0,"clearing",0,7,detectCurves:false);
        var branch=new TaggedToolpath(new Point3D[]{new(5,0,0),new(5,0,-1),new(5,3,-1)},
            new ToolpathSpan[]{new(0,1,ToolpathSpanKind.Ramp),new(1,2,ToolpathSpanKind.Cut)},0,"clearing",0,8,detectCurves:false);
        TaggedToolpath Shift(TaggedToolpath p)=>new(p.Points.Select(v=>v with{Z=v.Z+surface}).ToArray(),p.Spans,
            p.RegionIndex,p.Category,p.DepthPassIndex,p.VisitIdentity,detectCurves:false);
        carrier=Shift(carrier);branch=Shift(branch);
        var result=Assert.Single(ToolpathContinuationPlanner.Reconnect([carrier,branch],"test",1,default));
        Assert.Equal(carrier.Points[0],result.Points[0]);Assert.Equal(7,result.VisitIdentity);
        Assert.Equal(carrier.Points[^1],result.Points[^1]);
        Assert.Single(result.Spans.Where(s=>s.Kind==ToolpathSpanKind.Ramp));
        Assert.Contains(new Point3D(5,3,surface-1),result.Points);Assert.DoesNotContain(new Point3D(5,0,surface),result.Points);
    }

    [Fact]
    public void SpatialDetourSearchAvoidsAllPairsOnSparseSourcesAndCancels()
    {
        var source=Enumerable.Range(0,4000).Select(i=>Cut(new(i*10,0,-1),new(i*10+1,0,-1))).ToArray();
        var log=new List<string>();Assert.Equal(source.Length,ToolpathSourceTraversal.Detours(source,default,log.Add).Count);
        Assert.Contains(log,l=>l.Contains("candidates=0 ")&&l.Contains("cappedQueries=0 "));
        using var stop=new CancellationTokenSource();stop.Cancel();
        Assert.Throws<OperationCanceledException>(()=>ToolpathSourceTraversal.Detours(source,stop.Token));
    }

    [Fact]
    public void ParallelRingSpansConnectWhenDiagonalCornersExceedStepover()
    {
        var outer=Cut(new(0,0,-1),new(10,0,-1),new(10,10,-1),new(0,10,-1),new(0,0,-1));
        var inner=Cut(new(.2,.2,-1),new(9.8,.2,-1),new(9.8,9.8,-1),new(.2,9.8,-1),new(.2,.2,-1));
        var portals=ToolpathSourceTraversal.RingPortals([outer,inner],.21,p=>p,(_,_)=>true,default);
        Assert.Contains(portals[0].Points,p=>p==new Point3D(.2,0,-1)||p==new Point3D(0,.2,-1));
        var components=ToolpathSourceTraversal.Components(portals,p=>p,(_,_)=>true,.21,default);
        Assert.Equal(components[0].ContinuationComponentIndex,components[1].ContinuationComponentIndex);
        var refused=ToolpathSourceTraversal.RingPortals([outer,inner],.21,p=>p,(_,_)=>false,default);
        Assert.Equal(outer.Points,refused[0].Points);Assert.Equal(inner.Points,refused[1].Points);
    }

    [Fact]
    public void InterleavedClosedFamiliesFinishEveryRingAndDepthBeforeTheNextIsland()
    {
        TaggedToolpath Ring(double x,double inset)=>Cut(new(x+inset,inset,-2),new(x+10-inset,inset,-2),
            new(x+10-inset,10-inset,-2),new(x+inset,10-inset,-2),new(x+inset,inset,-2));
        var original=new[]{Ring(0,0),Ring(50,0),Ring(0,.1),Ring(50,.1)};
        var grouped=ToolpathSourceTraversal.Components(original,p=>p with{Z=-1},(_,_)=>true,.3,default);
        var visits=VCarveToolpaths.ScheduleFreshStockVisits(grouped,new(2,1),default,null,preserveSourceOrder:true);
        var components=visits.Select(p=>p.ContinuationComponentIndex).ToArray();
        Assert.Equal(components.Order(),components);
        Assert.Equal(2,visits.GroupBy(p=>p.ContinuationComponentIndex).Count());
        foreach(var island in visits.GroupBy(p=>p.ContinuationComponentIndex))
        {
            Assert.Equal(new[]{0,1,0,1},island.Select(p=>p.DepthPassIndex!.Value));
            Assert.Equal(new[]{0d,0d,.1,.1},island.Select(p=>Math.Round(p.Points[0].Y,1)));
        }
    }

    [Fact]
    public void FreshEngagementIsRelativeToExecutedPoseAndAlsoChecksPostedMotion()
    {
        Point3D a=new(0,0,-4),b=new(.2,0,-4.05);
        Assert.True(ToolpathEngagementLimits.Allows(a,b,.3,.1,.3,p=>p));
        Assert.False(ToolpathEngagementLimits.Allows(a,b,.1,.1,.3,p=>p));
        Assert.False(ToolpathEngagementLimits.Allows(a,b,.3,.01,.3,p=>p));
        Assert.False(ToolpathEngagementLimits.Allows(a,b,.3,.1,.3,p=>p==b?p with{Z=-4.2}:p));
    }
}
