using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class VCarveArtworkFringeGoalTests
{
    static List<PointD> Square(double half)=>new(){new(-half,-half),new(half,-half),new(half,half),new(-half,half)};
    static Paths64 Native(IEnumerable<IReadOnlyList<PointD>> r)=>new(r.Select(p=>new Path64(p.Select(v=>new Point64((long)Math.Round(v.x*PathUtils.Scale),(long)Math.Round(v.y*PathUtils.Scale))))));
    static bool Contains(IEnumerable<IReadOnlyList<PointD>> rings,double x,double y)
    {
        var p=new Point64((long)Math.Round(x*PathUtils.Scale),(long)Math.Round(y*PathUtils.Scale));
        return Native(rings).Sum(r=>Clipper.PointInPolygon(p,r)==PointInPolygonResult.IsInside?Math.Sign(Clipper.Area(r)):0)!=0;
    }
    [Fact]
    public void OriginalArtworkMaskRetainsAnInteriorMissingStockHole()
    {
        var goal=new List<List<PointD>>{Square(10)};var artwork=new List<IReadOnlyList<PointD>>{Square(10)};var logs=new List<string>();
        var masked=VCarveArtworkFringeGoal.ApplyToFixedTarget(goal,artwork,true,default,logs.Add);
        Assert.NotSame(goal,masked);Assert.True(Contains(masked,9.99,0));Assert.False(Contains(masked,9.999,0));
        Assert.Empty(Clipper.Difference(Native(masked),Native(goal),FillRule.NonZero));
        Assert.Empty(Clipper.Difference(Native(masked),Native(artwork),FillRule.NonZero));
        // The actual cut has a large interior stock island. Its boundary is
        // NOT an original artwork boundary, so the demand remains in the goal.
        var hole=Square(.5);hole.Reverse();var actual=new List<IReadOnlyList<PointD>>{Square(10),hole};
        var missing=Clipper.Difference(Native(masked),Native(actual),FillRule.NonZero);
        Assert.True(Contains(missing.Select(r=>(IReadOnlyList<PointD>)r.Select(p=>new PointD(p.X/PathUtils.Scale,p.Y/PathUtils.Scale)).ToList()),0,0));
        Assert.Contains(logs,s=>s.Contains("applied=True")&&s.Contains("globalVisibilityAccepted=False"));
    }
    [Fact]
    public void ArtificialTargetEdgesDoNotCreateIgnoredMaterialFringes()
    {
        var goal=new List<List<PointD>>{Square(.5)};var artwork=new List<IReadOnlyList<PointD>>{Square(10)};
        var masked=VCarveArtworkFringeGoal.ApplyToFixedTarget(goal,artwork,true);
        Assert.True(Contains(masked,.4999,0));
        Assert.Empty(Clipper.Difference(Native(goal),Native(masked),FillRule.NonZero));
        Assert.Empty(Clipper.Difference(Native(masked),Native(goal),FillRule.NonZero));
    }
    [Fact]
    public void DisabledUnsupportedAndCancellationKeepTheWholeOriginalGoal()
    {
        var goal=new List<List<PointD>>{Square(10)};var invalid=new List<IReadOnlyList<PointD>>{Square(10),new List<PointD>{new(double.NaN,0),new(1,0),new(0,1)}};
        Assert.Same(goal,VCarveArtworkFringeGoal.ApplyToFixedTarget(goal,invalid,false));
        Assert.Same(goal,VCarveArtworkFringeGoal.ApplyToFixedTarget(goal,invalid,true));
        Assert.Same(goal,VCarveArtworkFringeGoal.ApplyToFixedTarget(goal,new List<IReadOnlyList<PointD>>{Square(.00001)},true));
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        Assert.Throws<OperationCanceledException>(()=>VCarveArtworkFringeGoal.ApplyToFixedTarget(goal,invalid,true,cancel.Token));
    }
}
