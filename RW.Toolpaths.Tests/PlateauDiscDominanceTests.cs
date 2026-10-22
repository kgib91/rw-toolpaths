using RW.Toolpaths.Milling;
using Clipper2Lib;

namespace RW.Toolpaths.Tests;

public sealed class PlateauDiscDominanceTests
{
    [Fact]
    public void IndexMatchesTheCompleteOriginalRetainedDiscWalkIncludingNearTangencies()
    {
        var random=new Random(89271);
        var discs=new List<(double X,double Y,double Radius)>();
        for(int i=0;i<3000;i++)discs.Add((random.NextDouble()*80,random.NextDouble()*80,.2+random.NextDouble()*10));
        foreach(double shift in new[]{0d,1000000d})
        foreach(double epsilon in new[]{-1e-7,0,1e-7})
        {
            discs.Add((shift,shift,10));discs.Add((shift+5+epsilon,shift,5));
            discs.Add((shift,shift,5));discs.Add((shift,shift,5));
        }
        var ordered=discs.OrderByDescending(d=>d.Radius).ToArray();
        var indexed=new PlateauDiscDominance(ordered,default);
        var retained=new List<(double X,double Y,double Radius)>();
        for(int i=0;i<ordered.Length;i++)
        {
            var d=ordered[i];
            bool literal=retained.Any(other=>other.Radius-d.Radius>1e-10
                &&Math.Pow(other.X-d.X,2)+Math.Pow(other.Y-d.Y,2)<=Math.Pow(other.Radius-d.Radius-1e-10,2));
            Assert.Equal(literal,indexed.IsCovered(i));
            if(!literal){retained.Add(d);indexed.Keep(i);}
        }
        Assert.True(indexed.Candidates<ordered.Length*(long)ordered.Length/20);
    }

    [Fact]
    public void IntegerSubtreeBoundsMatchIndependentFullDiscContainment()
    {
        var random=new Random(21777);
        var discs=Enumerable.Range(0,2000).Select(_=>new PlateauDiscCoverage.Disc(
            new Point64(random.Next(-50000,50000),random.Next(-50000,50000)),random.Next(1,20000))).ToArray();
        var index=new PlateauDiscCoverage(discs,default);
        var first=discs[0];
        for(int i=0;i<4;i++)Assert.True(index.ContainsHull(first.Center,first.Radius/2,first.Center,first.Radius/2));
        Assert.True(index.PriorDiscWins>0);
        Assert.False(index.ContainsHull(new(1000000,1000000),1,new(1000001,1000000),1));
        for(int i=0;i<4000;i++)
        {
            var a=new Point64(random.Next(-60000,60000),random.Next(-60000,60000));
            var b=new Point64(a.X+random.Next(-5000,5000),a.Y+random.Next(-5000,5000));
            long ra=random.Next(0,15000),rb=random.Next(0,15000);
            bool literal=discs.Any(d=>d.Radius>=ra&&d.Radius>=rb&&Fits(d,a,ra)&&Fits(d,b,rb));
            Assert.Equal(literal,index.ContainsHull(a,ra,b,rb));
        }
        static bool Fits(PlateauDiscCoverage.Disc d,Point64 p,long radius)
        {
            Int128 dx=(Int128)p.X-d.Center.X,dy=(Int128)p.Y-d.Center.Y,gap=d.Radius-radius;
            return dx*dx+dy*dy<=gap*gap;
        }
    }
}
