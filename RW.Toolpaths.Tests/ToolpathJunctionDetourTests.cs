using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ToolpathJunctionDetourTests
{
    private static TaggedToolpath Path(params Point3D[] points)=>new(points,[],0,"clearing",null,detectCurves:false);
    [Fact]
    public void AttachLiteralMultiChordJunctionAndReturnToSameCarrierVertex()
    {
        var carrier=Path(new(0,0,-1),new(1,0,-1),new(2,0,-1));
        var junction=Path(new(1,.1,-1),new(1,.4,-1),new(1,.6,-1));
        var result=ToolpathSourceTraversal.JunctionDetours([carrier],[junction],.2,(_,_)=>true,(_,_)=>true,default,Console.WriteLine,out int attached);
        Assert.Equal(1,attached);var points=Assert.Single(result).Points;
        Assert.Equal(carrier.Points[0],points[0]);Assert.Equal(carrier.Points[^1],points[^1]);
        var edges=points.Zip(points.Skip(1)).ToArray();
        foreach(var e in carrier.Points.Zip(carrier.Points.Skip(1)).Concat(junction.Points.Zip(junction.Points.Skip(1))))Assert.Contains(e,edges);
        foreach(var e in junction.Points.Zip(junction.Points.Skip(1)))Assert.Contains((e.Second,e.First),edges);
        Assert.Equal(new[]{carrier.Points[0],carrier.Points[1],junction.Points[0],junction.Points[1],junction.Points[2],junction.Points[1],junction.Points[0],carrier.Points[1],carrier.Points[2]},points);
    }
    [Fact]
    public void RefusedReverseChordRetainsIndependentJunction()
    {
        var carrier=Path(new(0,0,-1),new(1,0,-1),new(2,0,-1));
        var junction=Path(new(1,.1,-1),new(1,.4,-1));
        var result=ToolpathSourceTraversal.JunctionDetours([carrier],[junction],.2,(_,_)=>true,(a,b)=>a.Y<=b.Y,default,Console.WriteLine,out int attached);
        Assert.Equal(0,attached);Assert.Equal(new[]{carrier,junction},result);
    }
    [Fact]
    public void ARequiredJunctionCannotAttachToAnotherConstructedRow()
    {
        var carrier=Path(new(0,0,-1),new(1,0,-1),new(2,0,-1));
        var junction=Path(new(1,.1,-1),new(1,.4,-1));
        var result=ToolpathSourceTraversal.JunctionDetours([carrier],[junction],.2,(_,_)=>false,(_,_)=>true,default,Console.WriteLine,out int attached);
        Assert.Equal(0,attached);Assert.Equal(new[]{carrier,junction},result);
    }
}
