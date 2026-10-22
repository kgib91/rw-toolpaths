using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ConservativeSurfaceMaskTests
{
    [Fact]
    public void DenseExclusionKeepsAllProtectedStockIncludingTinyIslandsWithinEightMicrons()
    {
        Path64 Circle(double radius,int count,bool reverse=false)
        {
            var p=new Path64(Enumerable.Range(0,count).Select(i=>{
                double a=2*Math.PI*i/count;return new Point64(Math.Cos(a)*radius*PathUtils.Scale,Math.Sin(a)*radius*PathUtils.Scale);
            }));if(reverse)p.Reverse();return p;
        }
        var tiny=new Path64(new[]{new PointD(30,30),new PointD(30.002,30),new PointD(30.002,30.002),new PointD(30,30.002)}
            .Select(p=>new Point64(p.x*PathUtils.Scale,p.y*PathUtils.Scale)));
        var original=new Paths64{Circle(10,6000),Circle(2,1200,true),tiny};
        var compact=ConservativeSurfaceMask.Compact(original,default);
        Assert.True(compact.Sum(p=>p.Count)<original.Sum(p=>p.Count)/3);
        Assert.Empty(Clipper.Difference(original,compact,FillRule.NonZero));
        var grow=new ClipperOffset(2,.000125*PathUtils.Scale);grow.AddPaths(original,JoinType.Round,EndType.Polygon);
        var upper=new Paths64();grow.Execute(.008*PathUtils.Scale,upper);
        Assert.Empty(Clipper.Difference(compact,upper,FillRule.NonZero));
    }

    [Fact]
    public void CancellationCannotPublishLessProtection()
    {
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        Assert.Throws<OperationCanceledException>(()=>ConservativeSurfaceMask.Compact([],cancel.Token));
    }
}
