using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Tests;
public sealed class VCarveRemovalUnionTests
{
    private static readonly VCarveCutterProfile Ball=new([new(CarvingCutterSectionKind.Hemisphere,0,1,1),new(CarvingCutterSectionKind.Linear,1,1,10)],2);
    private static TaggedToolpath Line(double y,double z)=>new([new(-2,y,z),new(12,y,z)],[],0,"proof",0,detectCurves:false);
    [Fact]
    public void NeighboringSweepsJointlyCoverWhatNoSingleSweepContains()
    {
        var cuts=new[]{Line(-.4,-1),Line(.4,-1)};var old=new CutterSweepStock(Ball,p=>p,0,default);
        foreach(var p in cuts)old.RecordExecuted(p.Points[0],p.Points[1]);old.Seal();
        var a=new Point3D(0,0,-.8);var b=new Point3D(10,0,-.8);double t=.03/Math.Sqrt(2);
        Assert.False(old.Contains(a,b,t,-t));
        var union=new VCarveRemovalUnion(Ball,cuts,.001,default);
        Assert.True(union.Contains(a,b,t,t));Assert.True(union.Cells>1);
    }
    [Fact]
    public void AGapBetweenSweepsIsNotFilledByTheirBounds()
    {
        var union=new VCarveRemovalUnion(Ball,[Line(-1.1,-1),Line(1.1,-1)],.001,default);
        Assert.False(union.Contains(new(0,0,-.8),new(10,0,-.8),.02,.02));Assert.True(union.UnprovedPointRefusals>0);
    }
    [Fact]
    public void ExhaustingTheCellBudgetRefusesTheCertificate()
    {
        var union=new VCarveRemovalUnion(Ball,[Line(-.4,-1),Line(.4,-1)],.001,default,maximumCells:1);
        Assert.False(union.Contains(new(0,0,-.8),new(10,0,-.8),.02,.02));Assert.True(union.BudgetRefusals>0);
    }
    [Fact]
    public void ClearedTravelCannotBecomeRemovalEvidence()
    {
        var cut=Line(0,-1) with{Spans=[new(0,1,ToolpathSpanKind.ClearedTravel)]};
        var union=new VCarveRemovalUnion(Ball,[cut],.001,default);
        Assert.False(union.Contains(new(0,0,-.8),new(10,0,-.8),.02,.02));Assert.Equal(0,union.Chords);
    }
    [Fact]
    public void FirstContactAtACylinderRadiusIsItsBottomNotItsTop()
    {
        var cylinder=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,1,1,10)],2);
        var union=new VCarveRemovalUnion(cylinder,[Line(-.4,-1),Line(.4,-1)],.001,default);
        Assert.True(union.Contains(new(0,0,-.8),new(10,0,-.8),.02,.02));
    }
    [Fact]
    public void CoordinateOverflowCannotTurnAnUnresolvedRegionIntoAir()
    {
        var union=new VCarveRemovalUnion(Ball,[Line(0,-1)],.001,default);
        Assert.False(union.Contains(new(1e200,0,-.8),new(2e200,0,-.8),.02,.02));
    }
    [Fact]
    public void AcceptedBallUnionAlsoPassesIndependentPhysicalSphereSamples()
    {
        var union=new VCarveRemovalUnion(Ball,[Line(-.4,-1),Line(.4,-1)],.001,default);
        Assert.True(union.Contains(new(0,0,-.8),new(10,0,-.8),.02,.02));
        double Height(double radius)=>radius>1?double.PositiveInfinity:1-Math.Sqrt(Math.Max(0,1-radius*radius));
        for(int ix=0;ix<=120;ix++)for(int iy=0;iy<=100;iy++){
            double x=-1+12d*ix/120,y=-1+2d*iy/100;
            double dx=Math.Max(0,Math.Max(-x,x-10));
            double required=-.8+.02+Height(Math.Sqrt(dx*dx+y*y)+.02);
            if(required>=0)continue;
            double donor=Math.Min(-1+Height(Math.Abs(y-.4)),-1+Height(Math.Abs(y+.4)));
            Assert.True(donor<=required+1e-10,$"Uncovered sphere sample ({x},{y}): donor={donor}, required={required}");
        }
    }
}
