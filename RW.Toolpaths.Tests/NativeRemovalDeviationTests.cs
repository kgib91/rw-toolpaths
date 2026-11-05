using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Clipper2Lib;
using RW.Toolpaths.Milling.ReviewedSource;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class NativeRemovalDeviationTests
{
    static readonly PointD[] Part=[new(-10,-10),new(10,-10),new(10,20),new(-10,20)];
    static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),new(CarvingCutterSectionKind.Linear,1.5875,3,20)],12.7);
    [Fact] public void ContinuousGaugeDistinguishesBoundedRemovalFromMissingOrDisplacedCut()
    {
        Point3D[] old=[new(0,0,-4),new(0,5,-4)];
        var same=ToolMove.CubicBezier(old[0],new(0,1,-4),new(0,4,-4),old[1],0);
        var stock=new VCarveClearedStock([],[],Part);
        Assert.True(VCarveNativeCurvePlanning.VerifyRemovalDeviation(old,[same],stock,Profile,.3));
        Assert.False(VCarveNativeCurvePlanning.VerifyRemovalDeviation(old,[],stock,Profile,.3));
        var far=same with{Start=same.Start with{X=1},End=same.End with{X=1},Control1=same.Control1 with{X=1},Control2=same.Control2 with{X=1}};
        Assert.False(VCarveNativeCurvePlanning.VerifyRemovalDeviation(old,[far],stock,Profile,.3));
        Assert.Throws<ArgumentOutOfRangeException>(()=>VCarveNativeCurvePlanning.VerifyRemovalDeviation(old,[same],stock,Profile,0));
        Assert.Equal(0,old[0].X);Assert.Equal(-4,old[0].Z);
    }
    [Fact] public void ContactOptimizationHandlesBothDepthPlanesWithoutFeasibilityOnlyAcceptance()
    {
        var result=VCarveContactNativeConstruction.SolveContactQp(new double[,]{{1}},[1],
            [new([1],-.2),new([-1],0)]);
        Assert.True(result.Converged);Assert.InRange(result.Values[0],-1e-8,1e-8);
        var impossible=VCarveContactNativeConstruction.SolveContactQp(new double[,]{{1}},[0],
            [new([1],1),new([-1],0)]);
        Assert.False(impossible.Converged);
    }
}
