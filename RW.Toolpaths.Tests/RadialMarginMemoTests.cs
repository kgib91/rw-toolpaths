using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class RadialMarginMemoTests
{
    private static VCarveCutterProfile Profile=>new([
        new(CarvingCutterSectionKind.Hemisphere,0,1,1),
        new(CarvingCutterSectionKind.Linear,1,1.2,2),
        new(CarvingCutterSectionKind.Linear,1.2,1.5,4)],6);

    [Fact]
    public void ExactSectionMinimumBitsSurviveRepeatedArgumentsAndFifoEviction()
    {
        static Point3D NoCommand(Point3D p)=>throw new InvalidOperationException("A numeric profile minimum must not invoke a command callback.");
        var literal=new CutterSweepStock(Profile,NoCommand,0,default);
        var memo=new CutterSweepStock(Profile,NoCommand,0,default,radialMarginMemoEntries:4096);
        (double Delta,double Depth)[] exceptional=[(0,0),(-0d,-0d),(-.1,1),(.1,-1),(10,10),
            (double.NaN,1),(1,double.NaN),(double.PositiveInfinity,1),(1,double.NegativeInfinity)];
        foreach(var (delta,depth) in exceptional)Equal(delta,depth);
        for(int i=0;i<5000;i++)
        {
            double delta=(i+1)/100000d,depth=.5+i%7*.25;
            Equal(delta,depth);Equal(delta,depth);
        }
        foreach(var (delta,depth) in exceptional)Equal(delta,depth);
        Assert.Equal(4096,memo.RadialMarginMemoEntries);
        Assert.True(memo.RadialMarginMemoEvictions>0);
        Assert.True(memo.RadialMarginMemoHits>=5000);
        Assert.Equal(0,literal.RadialMarginMemoEntries);
        void Equal(double delta,double depth)=>Assert.Equal(
            BitConverter.DoubleToInt64Bits(literal.RadialMargin(delta,depth)),
            BitConverter.DoubleToInt64Bits(memo.RadialMargin(delta,depth)));
    }

    [Fact]
    public void CachedNumericMinimaPreserveWholeCutterAnswersAsActualStockGrows()
    {
        var literal=new CutterSweepStock(Profile,p=>p,0,default,mergeIncrementally:true);
        var memo=new CutterSweepStock(Profile,p=>p,0,default,mergeIncrementally:true,radialMarginMemoEntries:4096);
        var targetA=new Point3D(.1,0,-1);var targetB=new Point3D(.8,0,-1);
        Assert.False(literal.Contains(targetA,targetB,0));Assert.False(memo.Contains(targetA,targetB,0));
        for(int batch=0;batch<4;batch++)
        {
            double y=batch*4;
            var a=new Point3D(-2,y,-4);var b=new Point3D(2,y,-4);
            literal.RecordExecuted(a,b);memo.RecordExecuted(a,b);literal.Seal();memo.Seal();
            foreach(double height in new[]{-.9,-1d,-3.999,-4.01})
            foreach(double offset in new[]{0d,.01,.1,.5})
            {
                var x=new Point3D(-.5,y+offset,height);var z=new Point3D(.5,y+offset,height);
                for(int repeat=0;repeat<2;repeat++)
                {
                    Assert.Equal(literal.Contains(x,z,.01,-.01),memo.Contains(x,z,.01,-.01));
                    Assert.Equal(literal.ContainsTravel(x,z,.1),memo.ContainsTravel(x,z,.1));
                }
            }
            Assert.Equal(literal.Queries,memo.Queries);Assert.Equal(literal.Accepted,memo.Accepted);
            Assert.Equal(literal.PriorWholeChordTrials,memo.PriorWholeChordTrials);
            Assert.Equal(literal.PriorWholeChordProofs,memo.PriorWholeChordProofs);
        }
        Assert.True(memo.Contains(targetA,targetB,0));
        Assert.True(memo.RadialMarginMemoHits>0);
    }
}
