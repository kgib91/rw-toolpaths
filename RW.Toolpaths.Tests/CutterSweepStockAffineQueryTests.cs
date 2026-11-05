using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class CutterSweepStockAffineQueryTests
{
    private const double Allowance=.03/1.4142135623730951;
    private static VCarveCutterProfile Cone=>new([new(CarvingCutterSectionKind.Linear,0,3,6)],5);
    private static CutterSweepStock Stock(params (Point3D A,Point3D B)[] donors)
    {
        var stock=new CutterSweepStock(Cone,p=>p,0,default);
        foreach(var donor in donors)stock.RecordExecuted(donor.A,donor.B);
        stock.Seal();return stock;
    }
    private static bool Covered(CutterSweepStock stock,Point3D a,Point3D b)
        =>stock.Contains(a,b,Allowance,-Allowance);

    // Refine the mathematical query only. These exact affine midpoints are
    // never float-cast, F3-rounded, emitted, or recorded as stock evidence.
    private static bool CompleteQuery(CutterSweepStock stock,Point3D a,Point3D b,
        out int maximumDepth,out int nodes)
    {
        maximumDepth=nodes=0;
        if(Covered(stock,a,b))return true;
        if(!Covered(stock,a,a)||!Covered(stock,b,b))return false;
        int deepest=0,visited=0;
        bool Refine(Point3D x,Point3D y,int depth){
            deepest=Math.Max(deepest,depth);visited++;
            if(Covered(stock,x,y))return true;
            if(depth==6)return false;
            var middle=new Point3D((x.X+y.X)/2,(x.Y+y.Y)/2,(x.Z+y.Z)/2);
            return Refine(x,middle,depth+1)&&Refine(middle,y,depth+1);
        }
        bool covered=Refine(a,b,0);maximumDepth=deepest;nodes=visited;return covered;
    }

    [Fact]
    public void AffineRefinementRecoversCorrelatedAxialAndRadialConeClearance()
    {
        var stock=Stock((new(0,0,-2),new(10,0,-4)));
        var a=new Point3D(0,.05,-1.9);var b=new Point3D(10,.4,-3);
        // Cone slope=.5. Both endpoint physical margins are positive, and
        // their affine interpolation remains positive along the whole query.
        double marginA=.5*(a.Z-(-2)+Allowance)+Allowance-a.Y;
        double marginB=.5*(b.Z-(-4)+Allowance)+Allowance-b.Y;
        Assert.True(marginA>0);Assert.True(marginB>0);
        // The direct certificate combines the MINIMUM axial difference at A
        // with the MAXIMUM lateral difference at B and conservatively refuses.
        Assert.False(Covered(stock,a,b));Assert.True(Covered(stock,a,a));Assert.True(Covered(stock,b,b));
        Assert.True(CompleteQuery(stock,a,b,out int depth,out int nodes));
        Assert.InRange(depth,1,6);Assert.InRange(nodes,3,127);
        Assert.Equal(1,stock.Chords); // Proof subdivision creates no new stock.
    }

    [Fact]
    public void AnUncoveredEndpointCannotGainPermissionFromSubdivision()
    {
        var stock=Stock((new(0,0,-2),new(10,0,-4)));
        var a=new Point3D(0,.2,-1.9);var b=new Point3D(10,.4,-3);
        Assert.False(Covered(stock,a,a));Assert.True(Covered(stock,b,b));
        Assert.False(CompleteQuery(stock,a,b,out int depth,out int nodes));
        Assert.Equal(0,depth);Assert.Equal(0,nodes);
    }

    [Fact]
    public void ProvedEndpointsDoNotFillAnUncutInteriorGap()
    {
        var stock=Stock((new(0,0,-2),new(4,0,-2)),(new(6,0,-2),new(10,0,-2)));
        var a=new Point3D(0,0,-1.9);var b=new Point3D(10,0,-1.9);
        Assert.True(Covered(stock,a,a));Assert.True(Covered(stock,b,b));
        Assert.False(Covered(stock,new(5,0,-1.9),new(5,0,-1.9)));
        Assert.False(CompleteQuery(stock,a,b,out int depth,out int nodes));
        Assert.InRange(depth,1,6);Assert.InRange(nodes,1,127);
        Assert.Equal(2,stock.Chords);
    }
}
