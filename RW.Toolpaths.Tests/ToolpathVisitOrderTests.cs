using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ToolpathVisitOrderTests
{
    private static TaggedToolpath Cut(params Point3D[] points)=>new(points,Array.Empty<ToolpathSpan>(),0,"clearing",null,detectCurves:false);

    [Fact]
    public void WholeRegionOrderingKeepsEveryDepthAndRampTraceTogether()
    {
        var source=new[]{Cut(new(0,0,0),new(1,0,-1)) with{RegionIndex=0},
            Cut(new(90,0,0),new(91,0,-1)) with{RegionIndex=1},
            Cut(new(91,0,-1),new(92,0,-2)) with{RegionIndex=1},
            Cut(new(2,0,0),new(3,0,-1)) with{RegionIndex=2}}.ToList();
        var routed=VCarvePathRouter.OrderRegions(source,default,null);
        Assert.Equal(new[]{0,2,1,1},routed.Select(p=>p.RegionIndex));
        Assert.Equal(source[1].Points,routed[2].Points);
        Assert.Equal(source[2].Points,routed[3].Points);
        foreach(var path in source)Assert.Contains(routed,p=>p.Points.SequenceEqual(path.Points)&&p.Spans.SequenceEqual(path.Spans));
    }

    [Fact]
    public void OpenVisitsChooseTheNextPathFromTheirActualFarExit()
    {
        var paths=new[]{Cut(new(0,0,-1),new(100,0,-1)),
            Cut(new(2,0,-1),new(3,0,-1)),Cut(new(101,0,-1),new(102,0,-1))};
        Assert.Equal(new[]{0,2,1},ToolpathVisitOrder.Order(paths,new[]{true,true,true},new Point3D(0,0,0),default));
    }

    [Fact]
    public void CircuitEntryRetainsItsEstablishedRampSeamAndEveryFloorChord()
    {
        var source=Cut(new(0,0,-1),new(100,0,-1),new(100,10,-1),new(0,10,-1),new(0,0,-1));
        var circuits=ToolpathCircuitBuilder.Build(new[]{source},(_,_)=>false,1,default);
        var log=new List<string>();
        var result=ToolpathCircuitExecutor.Execute(circuits,.001,1,new(1,1),new(Math.Atan(.5)),
            Array.Empty<(double,double)>(),(_,_)=>false,default,log.Add,start:new Point3D(100,10,0));
        Assert.Equal(new Point3D(0,0,0),result[0].Points[0]);
        for(int i=1;i<source.Points.Count;i++)
            Assert.Contains(result,p=>Enumerable.Range(1,p.Points.Count-1).Any(j=>
                p.Points[j-1]==source.Points[i-1]&&p.Points[j]==source.Points[i]));
        Assert.Contains(log,l=>l.Contains("rootPolicy=source-circuit-seam"));
    }

    [Fact]
    public void BlockedNearbyRingEntryDoesNotFallBackToOppositeSide()
    {
        var ring=new PointD[]{new(0,0),new(100,0),new(100,10),new(0,10)};
        var clearance=new FarSideOnly();
        Assert.False(PocketToolpaths.TryFindSafeRebase(ring,new(0,1,-1),-1,clearance,1,out _,out int tested,true,localEntriesOnly:true));
        Assert.InRange(tested,1,2);
        Assert.All(clearance.Targets,p=>Assert.True(p.X<10));
    }

    [Fact]
    public void ManyCompletedSamplesKeepACompleteDeterministicVisitPermutation()
    {
        var paths=Enumerable.Range(0,600).Select(i=>Cut(new(i,0,-1),new(i,1,-1))).ToArray();
        var first=ToolpathVisitOrder.Order(paths,Enumerable.Repeat(true,paths.Length).ToArray(),new Point3D(0,0,0),default);
        Assert.Equal(Enumerable.Range(0,600),first.Order());
        Assert.Equal(first,ToolpathVisitOrder.Order(paths,Enumerable.Repeat(true,paths.Length).ToArray(),new Point3D(0,0,0),default));
    }

    private sealed class FarSideOnly : ILinkClearance
    {
        internal readonly List<Point3D> Targets=new();
        public bool IsTravelSafe(Point3D from,Point3D to,double radius){Targets.Add(to);return to.X>90;}
    }
}
