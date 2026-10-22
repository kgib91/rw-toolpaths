using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class CutterSweepStockTests
{
    [Fact]
    public void MedianIndexesMatchTheOriginalSortedStockAcrossCommittedAdditions()
    {
        var indexed=new CutterSweepStock(Profile,p=>p,0,default);
        var reference=new CutterSweepStock(Profile,p=>p,0,default,useMedianPartition:false);
        var original=Cut(Enumerable.Range(0,1001).Select(i=>new Point3D(-100+i/10d,0,-3)).ToArray());
        foreach(var stock in new[]{indexed,reference}){stock.AddProtected([original]);stock.Seal();}
        var actual=new List<(double X,double Depth)>{(-1,3)};
        for(int batch=0;batch<40;batch++)
        {
            double x=10+batch*10,depth=3+batch%4;
            var cut=Cut(new(x-.1,0,-depth),new(x,0,-depth),new(x+.1,0,-depth));
            foreach(var stock in new[]{indexed,reference}){stock.AddProtected([cut]);stock.Seal();}
            actual.Add((x,depth));
            foreach(var stock in new[]{indexed,reference})
            {
                foreach(var pose in actual)
                {
                    Assert.True(stock.CoversFloorTarget(new(pose.X,0,1-pose.Depth),0));
                    Assert.True(stock.Contains(new(pose.X,0,1-pose.Depth),new(pose.X+.01,0,1-pose.Depth),0));
                    Assert.False(stock.CoversFloorTarget(new(pose.X,0,-pose.Depth-.01),0));
                    Assert.False(stock.CoversFloorTarget(new(pose.X,8,1-pose.Depth),0));
                }
                Assert.False(stock.CoversFloorTarget(new(x+10,0,-1),0));
            }
        }
        Assert.InRange(indexed.MaximumBlocks,1,8);Assert.True(indexed.MergeCount>0);
        Assert.True(indexed.PartitionVisits>0);
        Assert.Equal(0,reference.PartitionVisits);
        Assert.True(reference.ReindexedChords>4000);
    }

    [Fact]
    public void PriorWitnessesAreReprovedAtTheNewLocationAndDepth()
    {
        var stock=new CutterSweepStock(Profile,p=>p,0,default);
        stock.AddProtected([Cut(new(-2,0,-4),new(-1,0,-4),new(2,0,-4))]);stock.Seal();
        Assert.True(stock.Contains(new(0,0,-2),new(.5,0,-2),0));
        Assert.True(stock.Contains(new(.001,0,-2),new(.501,0,-2),0));
        Assert.True(stock.PriorWholeChordProofs>0);
        Assert.False(stock.Contains(new(20,0,-2),new(21,0,-2),0));
        Assert.False(stock.Contains(new(0,0,-5),new(.5,0,-5),0));
        Assert.True(stock.CoversFloorTarget(new(0,0,-2),0));
        Assert.True(stock.CoversFloorTarget(new(.001,0,-2),0));
        Assert.True(stock.PriorFloorPoseProofs>0);
        Assert.False(stock.CoversFloorTarget(new(20,0,-2),0));
        Assert.False(stock.CoversFloorTarget(new(0,0,-4.01),0));
        stock.AddProtected([Cut(new(19,0,-6),new(20,0,-6),new(21,0,-6))]);stock.Seal();
        Assert.False(stock.CoversFloorTarget(new(0,0,-5),0));
        Assert.True(stock.CoversFloorTarget(new(20,0,-5),0));
    }

    [Fact]
    public void FloorWitnessUsesBoundedSurfaceDistanceOnASteepFlank()
    {
        var stock=new CutterSweepStock(Profile,p=>p,0,default);
        stock.AddProtected([Cut(new(0,0,-3),new(0,1,-3))]);stock.Seal();
        Assert.False(stock.CoversFloorTarget(new(1.18,1,-1),.1));
        Assert.True(stock.CoversFloorTarget(new(1.18,1,-1),.1,true));
        Assert.False(stock.CoversFloorTarget(new(1.21,1,-1),.1,true));
    }

    [Theory]
    [InlineData(.03)]
    [InlineData(.1)]
    public void NormalOffsetMatchesIndependentPhysicalSectionSearch(double clearance)
    {
        var profile=Profile;var stock=new CutterSweepStock(profile,p=>p,0,default);
        foreach(double height in new[]{-.02,0,.02,.6,.99,1,1.03,2.98,3,3.02,4,6})
        {
            double lo=Math.Max(0,height-clearance),hi=height+clearance,lower=0;
            for(int i=0;i<=20000;i++)
            {
                double at=lo+(hi-lo)*i/20000,delta=height-at;
                lower=Math.Max(lower,profile.PhysicalRadiusAtDepth(at)
                    +Math.Sqrt(Math.Max(0,clearance*clearance-delta*delta)));
            }
            double actual=stock.OffsetRadius(height,clearance);
            Assert.InRange(actual-lower,-1e-12,2e-5);
        }
    }

    [Fact]
    public void ContinuousContactRingCanUseSeveralActualPosesButCannotFillAGap()
    {
        var stock=new CutterSweepStock(Profile,p=>p,0,default);
        stock.AddProtected([Cut(new(-.4,0,-1),new(-.4,0,-1)),Cut(new(.4,0,-1),new(.4,0,-1))]);stock.Seal();
        Assert.True(stock.CoversFloorContactRing(new(0,0,-.5),.5,0));
        Assert.False(stock.CoversFloorContactRing(new(0,0,-.5),1,0));
    }

    private static TaggedToolpath Cut(params Point3D[] p)=>new(p,[],0,"clearing",null,detectCurves:false);
    private static VCarveCutterProfile Profile=>new(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1,1),
        new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,1.2,2),
        new CarvingCutterSection(CarvingCutterSectionKind.Linear,1.2,1.5,4)},6);

    [Fact]
    public void TipWitnessCannotEraseAStillUncutCutterFlank()
    {
        var profile=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,1,4)},3);
        var stock=new CutterSweepStock(profile,p=>p,0,default);
        stock.AddProtected(new[]{Cut(new(.8,-5,-2),new(.8,0,-2),new(.8,5,-2))});stock.Seal();
        Assert.True(2-profile.PhysicalProfile.Bottom(.8)>1.5);
        Assert.False(stock.Contains(new(0,1,-1.5),new(0,2,-1.5),.025));
        Assert.True(stock.CoversFloorTarget(new(0,1,-1.5),.025));
    }

    [Fact]
    public void AffineOldChordContainsTheWholeMovingQueryProfile()
    {
        var profile=Profile;var stock=new CutterSweepStock(profile,p=>p,0,default);
        stock.AddProtected(new[]{Cut(new(-10,0,-3),new(0,0,-3),new(10,0,-4))});stock.Seal();
        var a=new Point3D(2,.005,-1);var b=new Point3D(8,.005,-1.5);
        Assert.True(stock.Contains(a,b,0));
        for(int i=0;i<=64;i++)
        {
            double t=i/64d,x=a.X+(b.X-a.X)*t,depth=1+.5*t,old=3+x/10;
            for(int j=0;j<=64;j++)
            {
                double h=depth*j/64;
                Assert.True(.005+profile.PhysicalRadiusAtDepth(h)<=profile.PhysicalRadiusAtDepth(h+old-depth));
            }
        }
    }

    [Fact]
    public void DeepEndpointsCannotInventTheMissingSweepAcrossAShallowValley()
    {
        var stock=new CutterSweepStock(Profile,p=>p,0,default);
        stock.AddProtected(new[]{Cut(new(-10,0,-4),new(0,0,-4),new(5,0,-.1),new(10,0,-4))});stock.Seal();
        Assert.False(stock.Contains(new(0,0,-2),new(10,0,-2),.025));
        Assert.False(stock.CoversFloorTarget(new(5,0,-2),.025));
    }

    [Fact]
    public void AdjacentCertifiedOldChordsCoverALongerQueryWithoutInventingGaps()
    {
        var stock=new CutterSweepStock(Profile,p=>p,0,default);
        stock.AddProtected(new[]{Cut(new(-1,0,-3),new(0,0,-3),new(1,0,-3),new(2,0,-3),new(3,0,-3),new(4,0,-3))});stock.Seal();
        Assert.True(stock.Contains(new(.2,.0005,-2.99),new(3.8,.0005,-2.99),.025));
        Assert.Equal(1,stock.UnionAccepted);
        var gap=new CutterSweepStock(Profile,p=>p,0,default);
        gap.AddProtected(new[]{Cut(new(-1,0,-3),new(0,0,-3),new(1,0,-3)),
            Cut(new(1.9,0,-3),new(2,0,-3),new(3,0,-3),new(4,0,-3))});gap.Seal();
        Assert.False(gap.Contains(new(.2,.0005,-2.99),new(3.8,.0005,-2.99),.025));
    }

    [Fact]
    public void SurfaceDistanceAllowanceDoesNotTurnIntoAnUnboundedFlankTolerance()
    {
        var profile=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,1,4)},3);
        var stock=new CutterSweepStock(profile,p=>p,0,default);
        stock.AddProtected(new[]{Cut(new(0,-1,-2),new(0,0,-2),new(0,4,-2))});stock.Seal();
        double allowance=.03/Math.Sqrt(2);
        Assert.False(stock.Contains(new(.01,1,-2),new(.01,2,-2),allowance));
        Assert.True(stock.Contains(new(.01,1,-2),new(.01,2,-2),allowance,-allowance));
        Assert.False(stock.Contains(new(.05,1,-2),new(.05,2,-2),allowance,-allowance));
    }

    [Fact]
    public void RadialMarginIncludesTheSphereConeStationaryPoint()
    {
        var profile=Profile;var stock=new CutterSweepStock(profile,p=>p,0,default);
        foreach(var (delta,depth) in new[]{(.2,1d),(.1,3d),(1d,4d),(3d,2d)})
        {
            double bound=stock.RadialMargin(delta,depth),sampled=double.PositiveInfinity;
            for(int i=0;i<=10000;i++)
            {
                double h=depth*i/10000;
                sampled=Math.Min(sampled,profile.PhysicalRadiusAtDepth(h+delta)-profile.PhysicalRadiusAtDepth(h));
            }
            Assert.True(bound<=sampled+1e-12);Assert.True(sampled-bound<1e-5);
        }
        Assert.True(stock.RadialMargin(.2,1)<profile.PhysicalRadiusAtDepth(1.2)-profile.PhysicalRadiusAtDepth(1));
    }
}
