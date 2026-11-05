using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveFractionalReturnTests
{
    private static TaggedToolpath Cut(params Point3D[] p)=>new(p,[new(0,p.Length-1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ALongParentEdgeNeedsAnExplicitSubdivisionProof(bool allowSplit)
    {
        var parent=Cut(new(0,0,-.5),new(100,0,-.5),new(100,1,-.5),new(0,1,-.5),new(0,0,-.5));
        var child=Cut(new(0,.5,-1),new(50,.5,-1));var original=parent.Points.ToArray();
        var returns=new VCarveRingReturns(default,splitGuard:allowSplit?(_,_)=>true:(_,_)=>false);
        returns.Record(parent,null);returns.Record(child,[parent.Points[0],child.Points[0]]);
        var cost=new ToolpathMotionCost(new(1000,500,1000,5),0);
        bool Step(Point3D a,Point3D b)=>a==child.Points[^1]&&Math.Abs(b.Z+.5)<1e-12&&Math.Pow(b.X-a.X,2)+Math.Pow(b.Y-a.Y,2)<=.714*.714;
        var choices=returns.Departures(Cut(new(50,.4,-.6),new(51,.4,-.6)),cost,.2,.714,Step).ToArray();
        var shortReturns=choices.Where(p=>p.Anchor.Source==parent&&p.PortalFraction>0&&p.ProvedParentSteps==1).ToArray();
        if(allowSplit){
            var shortest=shortReturns.MinBy(p=>p.Seconds)!;Assert.Equal(new[]{child.Points[^1],new Point3D(50,0,-.5)},shortest.Points);
            Assert.Equal(.5,shortest.PortalFraction,12);Assert.True(shortest.Seconds<.05);
        }else{Assert.Empty(shortReturns);Assert.True(returns.ParentProjectionRefusals>0);}
        Assert.Equal(original,parent.Points);
    }

    [Fact]
    public void ARecordedFractionalDoorIsTheExactStartingPoseOfTheLaterParentWalk()
    {
        var parent=Cut(new(0,0,-.5),new(100,0,-.5),new(100,1,-.5),new(0,1,-.5),new(0,0,-.5));
        var child=Cut(new(0,.5,-1),new(50,.5,-1));
        var returns=new VCarveRingReturns(default,splitGuard:(_,_)=>true);
        returns.Record(parent,null);returns.Record(child,[parent.Points[0],child.Points[0]]);
        var cost=new ToolpathMotionCost(new(1000,500,1000,5),0);
        bool Step(Point3D a,Point3D b)=>a==child.Points[^1]&&Math.Abs(b.Z+.5)<1e-12&&Math.Pow(b.X-a.X,2)+Math.Pow(b.Y-a.Y,2)<=.714*.714;
        var door=returns.Departures(Cut(new(50,.4,-.6),new(51,.4,-.6)),cost,.2,.714,Step)
            .Where(d=>d.Anchor.Source==parent&&d.PortalFraction>0).MinBy(d=>d.Seconds)!;
        var point=door.Points[^1];var next=Cut(new(50,.3,-.7),new(70,.3,-.7));
        returns.Record(next,[point,next.Points[0]],door.Anchor,door.Portal,door.PortalFraction);
        var later=returns.Departures(Cut(new(49,.1,-.5),new(51,.1,-.5)),cost,10,.714)
            .Where(d=>d.Anchor.Source==parent).MinBy(d=>d.Seconds)!;
        Assert.Equal(new[]{next.Points[^1],next.Points[0],point},later.Points.Take(3));
        Assert.InRange(later.Points[^1].X,49,51);Assert.DoesNotContain(parent.Points[0],later.Points);
        Assert.Equal(5,parent.Points.Count);
    }

    [Fact]
    public void ALongCompletedContourSearchesNearbyFiniteEdgesWithoutScanningTheWholeBody()
    {
        const int count=4096;const double radius=100;
        var points=Enumerable.Range(0,count).Select(i=>new Point3D(radius*Math.Cos(2*Math.PI*i/count),radius*Math.Sin(2*Math.PI*i/count),-.5)).ToList();
        points.Add(points[0]);var parent=Cut(points.ToArray());
        var child=Cut(new(radius-.5,0,-1),new(radius-.5,.02,-1));
        var returns=new VCarveRingReturns(default,splitGuard:(_,_)=>true);
        returns.Record(parent,null);returns.Record(child,[parent.Points[0],child.Points[0]]);
        var cost=new ToolpathMotionCost(new(1000,500,1000,5),0);
        bool Step(Point3D a,Point3D b)=>Math.Abs(b.Z+.5)<1e-12&&Math.Pow(a.X-b.X,2)+Math.Pow(a.Y-b.Y,2)<=.714*.714;
        var choices=returns.Departures(Cut(new(radius-.1,0,-.6),new(radius-.1,.01,-.6)),cost,1,.714,Step).ToArray();
        Assert.Contains(choices,d=>d.Anchor.Source==parent&&d.PortalFraction>0&&d.ProvedParentSteps>0);
        Assert.InRange(returns.ParentProjectionEdgesExamined,1,512);
        Assert.Equal(count+1,parent.Points.Count);
        Console.WriteLine($"[VCarve.ParentDoorIndex.Test] bodyEdges={count} examined={returns.ParentProjectionEdgesExamined} finiteDoorFound=True");
    }
}
