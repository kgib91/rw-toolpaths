using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeContainmentBreakpointTests
{
    private static IReadOnlyList<IReadOnlyList<PointD>> Boundary(int count=96)
        =>[Enumerable.Range(0,count).Select(i=>new PointD(3*Math.Cos(2*Math.PI*i/count),3*Math.Sin(2*Math.PI*i/count))).ToArray()];

    private static VCarveCutterProfile Profile(bool rounded,int extra=0)
    {
        var sections=new List<CarvingCutterSection>();
        sections.Add(rounded?new(CarvingCutterSectionKind.Hemisphere,0,.3,.3):new(CarvingCutterSectionKind.Linear,.1,.3,.3));
        sections.Add(new(CarvingCutterSectionKind.Linear,.3,.8,.7));
        for(int i=0;i<extra;i++)sections.Add(new(CarvingCutterSectionKind.Linear,.8,.8,.02));
        sections.Add(new(CarvingCutterSectionKind.Linear,.8,.8,2));
        return new(sections,2);
    }

    [Theory]
    [InlineData(false,0)] [InlineData(true,0)] [InlineData(true,70)]
    public void LiteralBoundaryAndProfileProofsHaveIdenticalBits(bool rounded,int extra)
    {
        var outer=Boundary()[0];
        PointD[] hole=[new(-.3,-.3),new(-.3,.3),new(.3,.3),new(.3,-.3)];
        var contexts=new[]{new VCarveNativeContainment([outer],Profile(rounded,extra)),new VCarveNativeContainment([outer,hole],Profile(rounded,extra))};
        var random=new Random(720183);
        foreach(var context in contexts)
        {
            var endpoints=new List<(Point3D A,Point3D B)>
            {
                (new(0,0,-0d),new(1,0,-2)),(new(1,0,-2),new(0,0,-0d)),
                (new(-2,0,-.3),new(2,0,-.3)),(new(0,2,-1),new(0,-2,-1)),
                (new(0,0,1),new(0,0,1)),(new(0,0,-.3),new(0,0,-.3)),
                (new(2,0,-1),new(2,0,-.3)),(new(2,0,-.3),new(2,0,-1)),
                (new(-3,0,-1),new(3,0,-1)),(new(0,0,0),new(0,0,0))
            };
            for(int i=0;i<250;i++)endpoints.Add((Next(),Next()));
            Point3D Next()=>new(6*random.NextDouble()-3,6*random.NextDouble()-3,.2-2.2*random.NextDouble());
            foreach(var (a,b) in endpoints)
                Assert.Equal(BitConverter.DoubleToInt64Bits(context.MinimumSquaredSeparationLiteral(a,b)),BitConverter.DoubleToInt64Bits(context.MinimumSquaredSeparation(a,b)));
        }
    }

    [Fact]
    public void StockCallbacksKeepTheirOriginalOrderAndCompleteAuthority()
    {
        var calls=new List<(Point3D,Point3D)>();
        var context=new VCarveNativeContainment(Boundary(),Profile(true),stockChord:(a,b)=>{calls.Add((a,b));return a.X==0;},stockCurve:_=>true);
        Point3D a=new(0,0,-1),b=new(1,0,-1);
        Assert.Equal(0,context.MinimumSquaredSeparation(a,b));
        Assert.Equal(double.NegativeInfinity,context.MinimumSquaredSeparation(b,a));
        Assert.Equal(new[]{(a,b),(b,a)},calls);
    }

    [Fact]
    public void EdgeScratchAvoidsRepeatedPartitionAndSectionEnumeratorAllocation()
    {
        var context=new VCarveNativeContainment(Boundary(256),Profile(true));
        Point3D a=new(-2.8,.1,-.2),b=new(2.8,-.1,-1.8);
        for(int i=0;i<10;i++){context.MinimumSquaredSeparation(a,b);context.MinimumSquaredSeparationLiteral(a,b);}
        long before=GC.GetAllocatedBytesForCurrentThread();
        for(int i=0;i<100;i++)context.MinimumSquaredSeparationLiteral(a,b);
        long literal=GC.GetAllocatedBytesForCurrentThread()-before;
        before=GC.GetAllocatedBytesForCurrentThread();
        for(int i=0;i<100;i++)context.MinimumSquaredSeparation(a,b);
        long prepared=GC.GetAllocatedBytesForCurrentThread()-before;
        Assert.True(prepared<literal/3,$"prepared={prepared} literal={literal}");
    }
}
