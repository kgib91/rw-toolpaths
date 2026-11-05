using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class NativeLongSpanPlanningTests
{
    [Fact]
    public void DenseSamplesDoNotLimitNativeCurveToSixteenIntervals()
    {
        var points=Enumerable.Range(0,2001).Select(i=>{double t=i/2000d;return new Point3D(8*t,Math.Sin(t*Math.PI)*.7,-1);}).ToArray();
        var path=new TaggedToolpath(points,[new ToolpathSpan(0,2000,ToolpathSpanKind.Cut)],0,"dense",0,detectCurves:false);
        IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> boundaries=[new IReadOnlyList<PointD>[] {new PointD[] {new(-100,-100),new(100,-100),new(100,100),new(-100,100)}}];
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875)],1.5);
        var plan=VCarveNativeCurvePlanning.Nominate([path],boundaries,profile,0,limits:new(MaximumCandidateEdges:100),settings:new(.3,.3))[0];
        Assert.True(plan.NewCurveChecksPassed);
        Assert.False(plan.BudgetExhausted);
        Assert.True(plan.Primitives.Count<30,$"Dense curve retained {plan.Primitives.Count} primitives");
        Assert.Contains(plan.Primitives,p=>p.SourceLast-p.SourceFirst>16&&p.Move.Kind==ToolMoveKind.CubicBezier);
        Assert.All(plan.Primitives,p=>Assert.InRange(p.Move.MaxError,0,.3));
        Assert.Equal(2000,plan.Primitives.Sum(p=>p.SourceLast-p.SourceFirst));
        Assert.Equal(points,path.Points);
    }
}
