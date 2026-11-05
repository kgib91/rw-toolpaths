using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class NativeMinimumPathTests
{
    static VCarveNativeFit Edge(int a,int b,double? certifiedLength=null)
        =>new(ToolMove.Line(new(a,0,0),new(b,0,0)),a,b,b-a,certifiedLength??b-a,certifiedLength??b-a,0,0);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TwoPointSourceCanEarnACubicForItsPrescribedJoin(bool refuseStock)
    {
        Point3D[] source=[new(0,0,-8),new(.02,0,-1)];var original=source.ToArray();
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.01,.01,10)],10);
        Clipper2Lib.PointD[][] boundary=[[new(-20,-20),new(20,-20),new(20,20),new(-20,20)]];
        var guard=refuseStock?new VCarveNativeContainment(boundary,profile,stockChord:(a,b)=>false,stockCurve:m=>false)
            :new VCarveNativeContainment(boundary,profile);
        var fit=VCarveNativeEdgeSearch.Find(source,0,1,VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[1],source[0])),new(1,0,0),
            VCarveNativeCurveMath.Distance(source[0],source[1]),guard,profile,out _);
        Assert.Equal(original,source);
        if(refuseStock){Assert.Null(fit);return;}
        Assert.NotNull(fit);Assert.Equal(ToolMoveKind.CubicBezier,fit.Move.Kind);Assert.True(fit.LengthLower>=.3);
        Assert.True(VCarveNativeGeometricProof.Prove(fit.Move,source,0,1,.3).Certified);
        Assert.True(VCarveNativeCutterProof.Certified(fit.Move,guard,profile,out _));
    }

    [Fact] public void FindsCompleteAlternativeAfterLongestPrefixReachesDeadEnd()
    {
        var legal=new HashSet<(int,int)>{(0,4),(0,2),(2,5)};
        var result=VCarveNativeMinimumPath.Search([0,1,2,3,4,5],[0,1,2,3,4,5],.3,2048,
            (a,b)=>legal.Contains((a,b))?Edge(a,b):null,default,out var stats);
        Assert.NotNull(result);
        Assert.Equal(new[]{(0,2),(2,5)},result.Select(e=>(e.First,e.Last)));
        Assert.True(stats.FailedNodes>0);
    }

    [Fact] public void RefusesPartialPrefixWhenNoCompletePathExists()
    {
        var result=VCarveNativeMinimumPath.Search([0,1,2,3],[0,1,2,3],.3,2048,
            (a,b)=>a==0&&b==2?Edge(a,b):null,default,out _);
        Assert.Null(result);
    }

    [Fact] public void SuccessfulLongEdgeDoesNotExploreUnreachableSuffixStarts()
    {
        int tried=0;
        var result=VCarveNativeMinimumPath.Search([0,1,2,3,4],[0,1,2,3,4],.3,2048,
            (a,b)=>{tried++;return Edge(a,b);},default,out _);
        Assert.Single(result!);Assert.Equal(1,tried);
    }

    [Theory]
    [InlineData(.299999)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void ConstructorRequiresCertifiedMinimumLength(double length)
    {
        var result=VCarveNativeMinimumPath.Search([0,1],[0,1],.3,2048,
            (a,b)=>Edge(a,b,length),default,out _);
        Assert.Null(result);
    }

    [Fact] public void SourceArcCannotHideAnUnconstructableShortTail()
    {
        var attempted=new List<(int,int)>();
        var result=VCarveNativeMinimumPath.Search([0,1,2],[0,1,1.1],.3,2048,
            (a,b)=>{attempted.Add((a,b));return a==0&&b==1?Edge(a,b):null;},default,out _);
        Assert.Null(result);Assert.DoesNotContain((0,1),attempted);
    }

    [Fact] public void CancellationAndSharedBudgetRefusalPropagate()
    {
        using var canceled=new CancellationTokenSource();canceled.Cancel();
        Assert.Throws<OperationCanceledException>(()=>VCarveNativeMinimumPath.Search([0,1],[0,1],.3,2048,
            (a,b)=>Edge(a,b),canceled.Token,out _));
        var budget=new VCarveNativeSearchBudget(default,1);
        Assert.Throws<VCarveNativeBudgetException>(()=>VCarveNativeMinimumPath.Search([0,1,2],[0,1,2],.3,2048,
            (a,b)=>{budget.CheckCandidate();return a==1?Edge(a,b):null;},default,out _));
    }
}
