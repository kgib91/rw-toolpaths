using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeGeometryRejectionScratchTests
{
    [Fact]
    public void FixedScratchRetainsEveryOriginalEnclosureAndRefusal()
    {
        var random=new Random(98121);
        for(int trial=0;trial<400;trial++)
        {
            Point3D Point()=>new(random.NextDouble()*20-10,random.NextDouble()*20-10,random.NextDouble()*4-2);
            var move=ToolMove.CubicBezier(Point(),Point(),Point(),Point(),0);
            var source=Enumerable.Range(0,12).Select(i=>{
                var p=VCarveNativeCurveMath.At(move,i/11d);
                return p with{Y=p.Y+(i==trial%12?(trial%3)*.04:0)};
            }).ToArray();
            foreach(double allowance in new[]{0d,.03,.3})
                Assert.Equal(Original(move,source,allowance),
                    VCarveNativeGeometryRejection.Excluded(move,source,0,source.Length-1,allowance));
        }
        var straight=ToolMove.CubicBezier(new(0,0,0),new(1,0,0),new(2,0,0),new(3,0,0),0);
        var points=new Point3D[]{new(0,0,0),new(1,.3,0),new(3,0,0)};
        foreach(double value in new[]{double.NaN,double.PositiveInfinity,-1,0,.3})
            Assert.Equal(Original(straight,points,value),VCarveNativeGeometryRejection.Excluded(straight,points,0,2,value));
        foreach(var control in new Point3D[]{new(double.NaN,0,0),new(double.PositiveInfinity,0,0)})
        {
            var invalid=straight with{Control1=control};
            Assert.Equal(Original(invalid,points,.3),VCarveNativeGeometryRejection.Excluded(invalid,points,0,2,.3));
        }
    }

    [Fact]
    public void RepeatedEnclosuresAllocateNoManagedCandidateScratch()
    {
        var move=ToolMove.CubicBezier(new(0,0,0),new(1,.2,0),new(2,.2,0),new(3,0,0),0);
        var source=new Point3D[]{move.Start,new(1,.1,0),move.End};
        for(int i=0;i<100;i++)VCarveNativeGeometryRejection.Excluded(move,source,0,2,.3);
        long start=GC.GetAllocatedBytesForCurrentThread();
        for(int i=0;i<1000;i++)VCarveNativeGeometryRejection.Excluded(move,source,0,2,.3);
        Assert.Equal(0,GC.GetAllocatedBytesForCurrentThread()-start);
    }

    // Original allocating enclosure, independent of the fixed-scratch helper.
    private static bool Original(ToolMove move,IReadOnlyList<Point3D> source,double allowance)
    {
        var controls=new[]{move.Start,move.Control1,move.Control2,move.End};
        double scale=controls.Max(p=>Math.Max(Math.Abs(p.X),Math.Max(Math.Abs(p.Y),Math.Abs(p.Z))));
        if(!double.IsFinite(scale)||!double.IsFinite(allowance)||allowance<0)return false;
        double squared=Math.Pow(allowance+1e-9*Math.Max(1,scale),2);
        var boxes=new List<(Point3D Low,Point3D High)>();
        void Cover(Point3D a,Point3D b,Point3D c,Point3D d,int level)
        {
            if(level==4)
            {
                boxes.Add((new(Math.Min(Math.Min(a.X,b.X),Math.Min(c.X,d.X)),Math.Min(Math.Min(a.Y,b.Y),Math.Min(c.Y,d.Y)),Math.Min(Math.Min(a.Z,b.Z),Math.Min(c.Z,d.Z))),
                    new(Math.Max(Math.Max(a.X,b.X),Math.Max(c.X,d.X)),Math.Max(Math.Max(a.Y,b.Y),Math.Max(c.Y,d.Y)),Math.Max(Math.Max(a.Z,b.Z),Math.Max(c.Z,d.Z)))));
                return;
            }
            var ab=VCarveNativeCurveMath.Lerp(a,b,.5);var bc=VCarveNativeCurveMath.Lerp(b,c,.5);var cd=VCarveNativeCurveMath.Lerp(c,d,.5);
            var abc=VCarveNativeCurveMath.Lerp(ab,bc,.5);var bcd=VCarveNativeCurveMath.Lerp(bc,cd,.5);var mid=VCarveNativeCurveMath.Lerp(abc,bcd,.5);
            Cover(a,ab,abc,mid,level+1);Cover(mid,bcd,cd,d,level+1);
        }
        Cover(controls[0],controls[1],controls[2],controls[3],0);
        foreach(var point in source)
        {
            bool possible=false;
            foreach(var box in boxes)
            {
                double x=Math.Max(0,Math.Max(box.Low.X-point.X,point.X-box.High.X));
                double y=Math.Max(0,Math.Max(box.Low.Y-point.Y,point.Y-box.High.Y));
                double z=Math.Max(0,Math.Max(box.Low.Z-point.Z,point.Z-box.High.Z));
                if(x*x+y*y+z*z<=squared){possible=true;break;}
            }
            if(!possible)return true;
        }
        return false;
    }
}
