using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class ContourDemandTests
{
    [Fact]
    public void ShortGapsStayOnTheirContourButLongExcursionsRemainSeparate()
    {
        Point3D[] p=Enumerable.Range(0,10).Select(i=>new Point3D(i,0,0)).ToArray();
        bool[] needed={true,false,true,false,false,false,true,false,true,false};
        Assert.Equal(2,ContourDemand.BridgeGaps(p,needed,false,2));
        Assert.Equal(new[]{true,true,true,false,false,false,true,true,true,false},needed);
    }

    [Fact]
    public void ClosedContourBridgesAcrossItsSeamButOpenContourDoesNot()
    {
        Point3D[] p={new(0,0,0),new(1,0,0),new(1,1,0),new(0,1,0)};
        bool[] closed={false,true,false,true},open=(bool[])closed.Clone();
        Assert.Equal(2,ContourDemand.BridgeGaps(p,closed,true,2));
        Assert.All(closed,p=>Assert.True(p));
        Assert.Equal(1,ContourDemand.BridgeGaps(p,open,false,2));
        Assert.False(open[0]);
    }

    [Fact]
    public void CostUsesRealContourLengthIncludingTheRamp()
    {
        Point3D[] p={new(0,0,0),new(.1,0,1),new(.2,0,0)};
        bool[] needed={true,false,true};
        Assert.Equal(0,ContourDemand.BridgeGaps(p,needed,false,1));
        Assert.False(needed[1]);
    }
}
