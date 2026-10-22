using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class FloorDirectionalBoundsTests
{
    [Fact]
    public void EveryPlaneBoundsIndependentLiteralKernelsAlongCompleteAffineSources()
    {
        var random=new Random(70231);
        for(int trial=0;trial<400;trial++)
        {
            double grade=trial%7==0?4:4*Math.Pow(10,-6*random.NextDouble()),radius=trial%9==0?100:.1+99.9*random.NextDouble();
            if(trial%11==0){grade=Math.Tan(Math.PI/6);radius=1.5875;}
            Point3D Point()=>new(2048*random.NextDouble()-1024,2048*random.NextDouble()-1024,256*random.NextDouble()-128);
            var a=Point();var b=Point();var c=Point();
            var left=FloorDirectionalBounds.Empty;left.Add(a,grade);left.Add(b,grade);
            var right=FloorDirectionalBounds.Empty;right.Add(c,grade);
            var all=FloorDirectionalBounds.Merge(left,right);Assert.True(all.Eligible);
            for(int query=0;query<12;query++)
            {
                var p=query==0?a:Point();
                Assert.True(FloorDirectionalBounds.TryPrepare(p.X,p.Y,grade,radius,out var prepared));
                double lower=all.Lower(prepared);
                for(int i=0;i<=64;i++)
                {
                    double t=i/64d;
                    var s=new Point3D(a.X+(b.X-a.X)*t,a.Y+(b.Y-a.Y)*t,a.Z+(b.Z-a.Z)*t);
                    double distance=Math.Sqrt(Math.Pow(s.X-p.X,2)+Math.Pow(s.Y-p.Y,2));
                    double radial=Math.Min(distance,radius*grade/Math.Sqrt(1+grade*grade));
                    double bottom=radial*radial/(radius+Math.Sqrt(Math.Max(0,radius*radius-radial*radial)));
                    double literal=-s.Z+Math.Max(0,Math.Min(grade*distance,bottom+grade*(distance-radial)));
                    Assert.True(lower<=literal,$"lower={lower:R}, literal={literal:R}, m={grade:R}, R={radius:R}, t={t:R}");
                }
            }
        }
        Assert.False(FloorDirectionalBounds.TryPrepare(1025,0,.5,1,out _));
        Assert.False(FloorDirectionalBounds.TryPrepare(0,0,4.01,1,out _));
        Assert.False(FloorDirectionalBounds.TryPrepare(0,0,.5,double.NaN,out _));
        var unsupported=FloorDirectionalBounds.Empty;unsupported.Add(new(1025,0,0),.5);Assert.False(unsupported.Eligible);
    }
}
