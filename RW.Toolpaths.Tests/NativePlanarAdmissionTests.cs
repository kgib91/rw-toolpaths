using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativePlanarAdmissionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RecordedTestDesignsRegionTwoRoundedHandleIsRejectedBeforePublication(bool reverse)
    {
        // Native-only replay of frozen v16 reproduced both original Cut179–180
        // and its cached Link15445–15446. The entire .3mm source certificate
        // passed; one saturated handle rounded3.635e-15mm beyond the unchanged
        // constant-X reserve. Retain actual neighborhood coordinates/ordinals.
        const double z=-1.9073486257070726e-7;
        Point3D[] source=[new(118.656,335.33,z),new(118.656,335.717,z),new(118.656,335.719,z),
            new(118.657,336.11,z),new(118.657,343.717,z),new(118.655,343.781,z),
            new(118.647,343.886,z),new(118.63,343.991,z)];
        var move=ToolMove.CubicBezier(source[3],new(118.65699999899999,336.11000001751074,z),
            new(118.6570000000237,343.7169999824607,z),source[4],2.0000036354540497e-9);
        if(reverse){source=source.Reverse().ToArray();move=VCarveNativeCurveMath.Reverse(move);}
        Assert.True(VCarveNativeGeometricProof.Prove(move,source,3,4,.3).Certified);
        Assert.False(VCarveNativeCurvePlanning.PreservesPlanarProgress(move,source,3,4,out var reason));
        Assert.Equal("source-constant-planar-axis-X",reason);
        Assert.False(VCarveNativePlanarProgress.CreateAdmission(source,3,4).Preserved(move,out _));
        Assert.True(Math.Abs(move.Control1.X-source[3].X)>VCarveNativeGeometricProof.ArithmeticReserve
            ||Math.Abs(move.Control2.X-source[3].X)>VCarveNativeGeometricProof.ArithmeticReserve);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CanonicalCacheAdmissionChecksBothLiteralSourceOrigins(bool swapAxes)
    {
        const double reserve=VCarveNativeGeometricProof.ArithmeticReserve;
        Point3D P(double x,double y)=>swapAxes?new(y,x,-.5):new(x,y,-.5);
        Point3D[] source=[P(0,0),P(.75*reserve,1)];
        var move=ToolMove.CubicBezier(source[0],P(-.5*reserve,1d/3),P(.5*reserve,2d/3),source[1],0);
        Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(move,source,0,1,out _));
        var reverse=VCarveNativeCurveMath.Reverse(move);var reversed=source.Reverse().ToArray();
        Assert.False(VCarveNativeCurvePlanning.PreservesPlanarProgress(reverse,reversed,0,1,out _));
        Assert.False(VCarveNativePlanarProgress.CreateAdmission(source,0,1).Preserved(move,out var reason));
        Assert.StartsWith("reversed-occurrence:source-constant-planar-axis",reason);
    }

    [Fact]
    public void LiteralRoundedControlIsCheckedAfterHandleParameterSelection()
    {
        Point3D[] source=[new(4096,0,-.5),new(4096,1,-.5)];
        var u=new Point3D(1,1,0);var v=new Point3D(0,1,0);double alpha=1,beta=1;
        var oldDomain=VCarveNativePlanarProgress.Create(source,0,1);
        Assert.True(oldDomain.TrySelectHandles(source[0],source[1],u,v,ref alpha,ref beta));
        var move=ToolMove.CubicBezier(source[0],VCarveNativeCurveMath.Add(source[0],VCarveNativeCurveMath.Mul(u,alpha)),
            VCarveNativeCurveMath.Sub(source[1],VCarveNativeCurveMath.Mul(v,beta)),source[1],0);
        Assert.False(oldDomain.Preserved(move,out var reason));
        Assert.Equal("source-constant-planar-axis-X",reason);
        Assert.False(VCarveNativePlanarProgress.CreateAdmission(source,0,1).Preserved(move,out _));
    }

    [Fact]
    public void RoundedPlanarRefusalHappensBeforeAnyPhysicalCandidateCallback()
    {
        Point3D[] source=[new(4096,0,-.5),new(4096,1,-.5)];
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,1)],1);
        PointD[][] boundary=[[new(4094,-2),new(4098,-2),new(4098,2),new(4094,2)]];
        int physical=0;var guard=new VCarveNativeContainment(boundary,profile,
            stockChord:(a,b)=>{physical++;return true;},stockCurve:m=>{physical++;return true;});
        var u=VCarveNativeCurveMath.Unit(new(1,1,0));var v=new Point3D(0,1,0);
        var fit=VCarveNativeEdgeSearch.Find(source,0,1,u,v,1,guard,profile,out var audit,
            new VCarveNativeFitSettings(.3,.3){PreservePlanarSourceProgress=true,CandidateSelection=VCarveNativeCandidateSelection.FirstVerified});
        Assert.Null(fit);Assert.True(audit.PlanarProgressRejected>0);
        Assert.Equal(0,audit.SourceGeometryProofAttempts);Assert.Equal(0,physical);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FeasibleNearConstantHandlesAreSelectedInsideBothOccurrenceDomains(bool reverse)
    {
        const double reserve=VCarveNativeGeometricProof.ArithmeticReserve;
        Point3D[] source=[new(0,0,-.5),new(.75*reserve,1,-.5)];
        if(reverse)source=source.Reverse().ToArray();
        var direction=reverse?-1d:1d;
        var u=new Point3D(-reserve,direction,0);var v=new Point3D(reserve,direction,0);
        double alpha=.5,beta=.5;var domain=VCarveNativePlanarProgress.CreateAdmission(source,0,1);
        Assert.True(domain.TrySelectHandles(source[0],source[1],u,v,ref alpha,ref beta));
        var move=ToolMove.CubicBezier(source[0],VCarveNativeCurveMath.Add(source[0],VCarveNativeCurveMath.Mul(u,alpha)),
            VCarveNativeCurveMath.Sub(source[1],VCarveNativeCurveMath.Mul(v,beta)),source[1],0);
        Assert.True(domain.Preserved(move,out var reason),reason);
        Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(move,source,0,1,out _));
        Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(VCarveNativeCurveMath.Reverse(move),source.Reverse().ToArray(),0,1,out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullEdgeAdmissionNeverReturnsACubicThatFailsEitherLiteralAudit(bool reverse)
    {
        const double reserve=VCarveNativeGeometricProof.ArithmeticReserve;
        Point3D[] source=[new(0,0,-.5),new(.25*reserve,.25,-.5),new(.5*reserve,.5,-.5),new(.75*reserve,1,-.5)];
        if(reverse)source=source.Reverse().ToArray();
        var original=source.ToArray();
        PointD[][] boundary=[[new(-2,-2),new(2,-2),new(2,2),new(-2,2)]];
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,1)],1);
        var u=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[1],source[0]));
        var v=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[^1],source[^2]));
        double length=Enumerable.Range(1,source.Length-1).Sum(i=>VCarveNativeCurveMath.Distance(source[i-1],source[i]));
        var fit=VCarveNativeEdgeSearch.Find(source,0,source.Length-1,u,v,length,new(boundary,profile),profile,out _,
            new VCarveNativeFitSettings(.3,.3){PreservePlanarSourceProgress=true,CandidateSelection=VCarveNativeCandidateSelection.FirstVerified});
        Assert.NotNull(fit);
        Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(fit!.Move,source,0,source.Length-1,out _));
        Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(VCarveNativeCurveMath.Reverse(fit.Move),source.Reverse().ToArray(),0,source.Length-1,out _));
        Assert.True(VCarveNativeGeometricProof.Prove(fit.Move,source,0,source.Length-1,.3).Certified);
        Assert.True(VCarveNativeCutterProof.Certified(fit.Move,new(boundary,profile),profile,out _));
        Assert.Equal(original,source);
    }
}
