using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveSourcePortalTests
{
    [Fact]
    public void RejectedCombinedSubdivisionsPublishNoStalePortals()
    {
        var source=new[]{Path(new(0,0,-1),new(10,0,-1),new(10,5,-1),new(0,5,-1),new(0,0,-1)),
            Path(new(1,.2,-1),new(2,.2,-1),new(2,1.2,-1),new(1,1.2,-1),new(1,.2,-1))};
        bool Guard(Point3D a,Point3D b)=>!(a.Y==0&&b.Y==0&&a.X>0&&a.X<10&&b.X>0&&b.X<10);
        IReadOnlyList<ToolpathContinuationComponents.Edge>? captured=null;
        var result=ToolpathSourceTraversal.RingPortals(source,2,p=>p,Guard,default,capturePortals:p=>captured=p);
        Assert.Equal(source[0].Points,result[0].Points);
        Assert.NotNull(captured);Assert.Empty(captured!);
    }

    [Fact]
    public void LiteralCoincidentVerticesKeepPhysicalRegionsSeparate()
    {
        var source=new[]{Path(new(0,0,0),new(1,0,0)),Path(new(0,0,0),new(1,0,0)) with{RegionIndex=1}};
        var result=ToolpathContinuationComponents.Assign(source,p=>p,(_,_)=>true,2,default,literalPortalsOnly:true);
        Assert.Equal(2,result.Select(p=>p.ContinuationComponentIndex).Distinct().Count());
        Assert.Empty(result[0].ContinuationPlan!.Edges);
    }

    [Fact]
    public void AnInsertedLiteralDoorSurvivesTheCoarseNeighbourSampler()
    {
        var a=new Point3D(.1,.1,0);var b=new Point3D(.1,.2,0);
        var source=new[]{Path(new(0,0,0),a,new(10,0,0)),Path(b,new(.1,1,0))};
        bool Guard(Point3D x,Point3D y)=>(x==a&&y==b)||(x==b&&y==a);
        var sampled=ToolpathContinuationComponents.Assign(source,p=>p,Guard,10,default);
        Assert.Equal(2,sampled.Select(p=>p.ContinuationComponentIndex).Distinct().Count());
        var logs=new List<string>();
        var retained=ToolpathContinuationComponents.Assign(source,p=>p,Guard,10,default,logs.Add,
            nominatedPortals:[new(new(0,a),new(1,b))]);
        Assert.Single(retained.Select(p=>p.ContinuationComponentIndex).Distinct());
        Assert.Equal(source[0].Points,retained[0].Points);Assert.Equal(source[1].Points,retained[1].Points);
        Assert.Contains(logs,l=>l.Contains("nominatedAccepted=1"));
        Console.WriteLine("[VCarve.SourcePortal.Test] coarseCellWouldDiscardDoor=True physicalChordReproved=True originalSourcePreserved=True");
    }

    [Fact]
    public void BothPhysicalDoorsSurviveEvenWhenTheirOwnersAreAlreadyConnected()
    {
        var source=new[]{Path(new(0,0,0),new(10,0,0)),Path(new(0,1,0),new(10,1,0))};
        var edges=new[]{new ToolpathContinuationComponents.Edge(new(0,source[0].Points[0]),new(1,source[1].Points[0])),
            new ToolpathContinuationComponents.Edge(new(0,source[0].Points[1]),new(1,source[1].Points[1]))};
        var logs=new List<string>();
        var result=ToolpathContinuationComponents.Assign(source,p=>p,(a,b)=>a.X==b.X,20,default,logs.Add,nominatedPortals:edges);
        Assert.Equal(2,result[0].ContinuationPlan!.Edges.Count);
        Assert.Equal(2,result[0].ContinuationPlan!.Entries(0).Count());
        Assert.Contains(logs,l=>l.Contains("cyclePortals=1")&&l.Contains("nominatedAccepted=2"));
    }

    [Theory]
    [InlineData(false,false,false)]
    [InlineData(true,true,false)]
    [InlineData(true,false,true)]
    public void RefusedUnownedOrCrossRegionDoorCannotJoinSources(bool proof,bool unowned,bool acrossRegion)
    {
        var source=new[]{Path(new(0,0,0),new(1,0,0)),Path(new(0,1,0),new(1,1,0)) with{RegionIndex=acrossRegion?1:0}};
        var a=unowned?new Point3D(.5,0,0):source[0].Points[0];var b=source[1].Points[0];
        bool Guard(Point3D x,Point3D y)=>proof&&((x==a&&y==b)||(y==a&&x==b));
        var result=ToolpathContinuationComponents.Assign(source,p=>p,Guard,2,default,
            nominatedPortals:[new(new(0,a),new(1,b))]);
        Assert.Equal(2,result.Select(p=>p.ContinuationComponentIndex).Distinct().Count());
        Assert.Empty(result[0].ContinuationPlan!.Edges);
    }

    private static TaggedToolpath Path(params Point3D[] points)=>new(points,Array.Empty<ToolpathSpan>(),0,"clearing",0,detectCurves:false);
}
