using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class CylindricalFootprintRefusalTests
{
    [Theory]
    [InlineData(0,.5)]
    [InlineData(1,.5)]
    [InlineData(2,.5)]
    [InlineData(0,3.175)]
    [InlineData(1,3.175)]
    [InlineData(2,3.175)]
    public void ActualCapsuleWitnessAgreesWithCompleteIndependentColumnProof(int direction,double radius)
    {
        Point3D a=new(0,0,-3),b=direction switch{0=>new(10,0,-3),1=>new(0,10,-3),_=>new(10,10,-3)};
        var source=new[]{new ClearedColumnStock.Column(a,b,radius)};
        var bounds=Bounds(source);
        var from=a with{Z=-1};var to=b with{Z=-1};double demanded=radius+.1;
        Assert.True(bounds.TryUncoveredWitness(from,to,demanded,out var witness));
        Assert.True(DistanceToSegment(witness,a,b)>radius);
        Assert.False(new ClearedColumnStock(source,default).Contains(from,to,_=>demanded));
        Assert.True(bounds.TryUncoveredWitness(to,from,demanded,out _));
    }

    [Fact]
    public void JointlyClearedLanesRetainTheirFullPositiveColumnProof()
    {
        ClearedColumnStock.Column[] source=[new(new(-2,-.4,-3),new(12,-.4,-3),.5),
            new(new(-2,.4,-3),new(12,.4,-3),.5)];
        Point3D from=new(0,0,-1),to=new(10,0,-1);
        Assert.False(Bounds(source).TryUncoveredWitness(from,to,.6,out _));
        Assert.True(new ClearedColumnStock(source,default).Contains(from,to,_=>.6));
    }

    [Fact]
    public void GapInsideOuterBoundsRemainsUnresolvedAndNeedsTheOriginalProof()
    {
        ClearedColumnStock.Column[] source=[new(new(-5,-2,-3),new(15,-2,-3),1),
            new(new(-5,2,-3),new(15,2,-3),1)];
        Point3D from=new(0,0,-1),to=new(10,0,-1);
        Assert.False(Bounds(source).TryUncoveredWitness(from,to,.5,out _));
        Assert.False(new ClearedColumnStock(source,default).Contains(from,to,_=>.5));
    }

    [Theory]
    [InlineData(0,false)]
    [InlineData(.5,false)]
    [InlineData(1,false)]
    [InlineData(2,true)]
    public void QuantizationReserveAndBoundaryEqualityCannotCreateRefusal(double reserveFactor,bool expected)
    {
        var bounds=Bounds([new(new(0,0,-3),new(10,0,-3),1)]);
        double protrusion=reserveFactor*MillingLocalTravel.CylindricalFootprintBounds.Reserve;
        Point3D centre=new(-.5-protrusion,0,-1);
        Assert.Equal(expected,bounds.TryUncoveredWitness(centre,centre,.5,out _));
    }

    [Fact]
    public void MissingNonfiniteOrOverflowingEvidenceKeepsTheCompleteProof()
    {
        var bounds=new MillingLocalTravel.CylindricalFootprintBounds();
        Assert.False(bounds.TryUncoveredWitness(new(100,0,-1),new(101,0,-1),1,out _));
        bounds.Add(new(new(0,0,-3),new(10,0,-3),1));
        Assert.False(bounds.TryUncoveredWitness(new(double.NaN,0,-1),new(101,0,-1),1,out _));
        Assert.False(bounds.TryUncoveredWitness(new(100,0,-1),new(101,0,-1),double.PositiveInfinity,out _));
        bounds.Add(new(new(double.NaN,0,-3),new(10,0,-3),1));
        Assert.False(bounds.IsUsable);
        Assert.False(bounds.TryUncoveredWitness(new(100,0,-1),new(101,0,-1),1,out _));
        bounds.Reset();bounds.Add(new(new(double.MaxValue,0,-3),new(double.MaxValue,0,-3),double.MaxValue));
        Assert.False(bounds.IsUsable);
    }

    [Fact]
    public void LocalTravelRefusalSkipsFloorUnionButRetainsHeightOnlyTransfer()
    {
        var log=new List<string>();var local=Local(log);
        local.RecordExecutedCut(new(-5,0,-3),new(5,0,-3));
        Assert.Null(local.Connect(new(4.9,0,7),new(5,0,7),new(4.9,0,-3),15));
        var height=local.Connect(new(4.9,0,7),new(4.9,0,6),new(4.9,0,-3),15);
        Assert.NotNull(height);Assert.Single(height!);
        Assert.Equal(new Point3D(4.9,0,6),height![0].End);
        local.Report();
        Assert.Contains(log,line=>line.Contains("event=column-footprint-uncovered")&&line.Contains("decision=refuse-only"));
        Assert.Contains(log,line=>line.Contains("event=summary")&&line.Contains("floorUnionBuilds=0 ")
            &&line.Contains("heightOnly=1 ")&&!line.Contains("footprintBoundsRefusals=0 "));
    }

    [Fact]
    public void NewRegionDiscardsEarlierBoundsAndCreditsOnlyItsStoredCuts()
    {
        var log=new List<string>();var local=Local(log);
        local.RecordExecutedCut(new(-100,-100,-3),new(100,100,-3));
        local.BeginRegion(1);local.RecordExecutedCut(new(0,0,-3),new(1,0,-3));
        Assert.Null(local.Connect(new(0,0,7),new(1,0,7),new(0,0,-3),15));
        local.Report();
        Assert.Contains(log,line=>line.Contains("event=column-footprint-uncovered region=1 ")&&line.Contains("storedColumns=1 "));
        Assert.Contains(log,line=>line.Contains("event=summary")&&line.Contains("floorUnionBuilds=0 "));
    }

    private static MillingLocalTravel Local(List<string> log)
    {
        var result=new MillingLocalTravel(new([new(CarvingCutterSectionKind.Linear,.5,.5,8)],5),10,
            diagnostic:log.Add);result.BeginRegion(0);return result;
    }
    private static MillingLocalTravel.CylindricalFootprintBounds Bounds(IEnumerable<ClearedColumnStock.Column> source)
    {
        var bounds=new MillingLocalTravel.CylindricalFootprintBounds();foreach(var column in source)bounds.Add(column);return bounds;
    }
    private static double DistanceToSegment(Point3D p,Point3D a,Point3D b)
    {
        double dx=b.X-a.X,dy=b.Y-a.Y,square=dx*dx+dy*dy;
        double t=square==0?0:Math.Clamp(((p.X-a.X)*dx+(p.Y-a.Y)*dy)/square,0,1);
        return Math.Sqrt(Math.Pow(p.X-a.X-t*dx,2)+Math.Pow(p.Y-a.Y-t*dy,2));
    }
}
