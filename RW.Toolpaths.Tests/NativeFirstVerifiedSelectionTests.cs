using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeFirstVerifiedSelectionTests
{
    private static readonly VCarveCutterProfile Cutter=new([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
    private static readonly PointD[][] Target=[[new(-2,-2),new(11,-2),new(11,2),new(-2,2)]];
    private static Point3D[] Source()
    {
        var source=ToolMove.CubicBezier(new(0,0,-.5),new(3,1,-.5),new(6,1,-.5),new(9,0,-.5),0);
        return Enumerable.Range(0,97).Select(i=>VCarveNativeCurveMath.At(source,i/96d)).ToArray();
    }
    private static (VCarveNativeFit? Fit,VCarveNativeEdgeAudit Audit) Fit(Point3D[] source,VCarveNativeContainment guard,
        VCarveNativeCandidateSelection selection=VCarveNativeCandidateSelection.FirstVerified)
    {
        var settings=new VCarveNativeFitSettings(.3,.3){CandidateSelection=selection,PreservePlanarSourceProgress=true};
        double length=0;for(int i=1;i<source.Length;i++)length+=VCarveNativeCurveMath.Distance(source[i-1],source[i]);
        var u=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[1],source[0]));
        var v=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[^1],source[^2]));
        var fit=VCarveNativeEdgeSearch.Find(source,0,source.Length-1,u,v,length,guard,Cutter,out var audit,settings);
        return(fit,audit);
    }
    private static void IndependentlyCheck(VCarveNativeFit fit,Point3D[] source,VCarveNativeContainment guard)
    {
        Assert.Equal(source[0],fit.Move.Start);Assert.Equal(source[^1],fit.Move.End);
        Assert.True(VCarveNativeGeometricProof.Prove(fit.Move,source,0,source.Length-1,.3).Certified);
        Assert.True(VCarveNativeCutterProof.Certified(fit.Move,guard,Cutter,out _));
        Assert.True(fit.LengthLower>=.3);Assert.InRange(fit.Move.MaxError,0,.3);
        Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(fit.Move,source,0,source.Length-1,out _));
        var expectedStart=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[1],source[0]));
        var expectedEnd=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[^1],source[^2]));
        Assert.InRange(VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(expectedStart,
            VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(fit.Move,0)))),0,1e-12);
        Assert.InRange(VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(expectedEnd,
            VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(fit.Move,1)))),0,1e-12);
        var range=VCarveNativeCurveMath.ZRange(fit.Move);
        Assert.InRange(range.Min,source.Min(p=>p.Z)-1e-9,source.Max(p=>p.Z)+1e-9);
        Assert.InRange(range.Max,source.Min(p=>p.Z)-1e-9,source.Max(p=>p.Z)+1e-9);
    }

    [Fact]
    public void FirstFullyVerifiedCandidateAvoidsProvingTheRestOfTheInventory()
    {
        var source=Source();var original=source.ToArray();
        var first=Fit(source,new(Target,Cutter));
        var ranked=Fit(source,new(Target,Cutter),VCarveNativeCandidateSelection.MinimumError);
        Assert.NotNull(first.Fit);Assert.NotNull(ranked.Fit);
        IndependentlyCheck(first.Fit!,source,new(Target,Cutter));
        IndependentlyCheck(ranked.Fit!,source,new(Target,Cutter));
        Assert.Equal(VCarveNativeCandidateSelection.FirstVerified,first.Audit.CandidateSelection);
        Assert.Equal(1,first.Audit.VisitedNominees);Assert.Equal(1,first.Audit.GeometryProofs);
        Assert.True(first.Audit.VisitedNominees<first.Audit.CandidateInventory);
        Assert.True(first.Audit.GeometryProofs<ranked.Audit.GeometryProofs);
        Assert.Equal(1,first.Audit.GeometryWorkers);Assert.Equal(0,first.Audit.BufferedGeometryCandidates);
        Assert.Equal(original,source);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ARefusedPoseOrContinuousProofContinuesToTheNextCandidate(bool refusePose)
    {
        int poses=0,curves=0;
        var physical=new VCarveNativeContainment(Target,Cutter);
        var guard=new VCarveNativeContainment(Target,Cutter,
            stockChord:(a,b)=>{poses++;return !(refusePose&&poses==1)&&physical.Legal(a,b);},
            stockCurve:move=>{curves++;return !(!refusePose&&curves==1)&&VCarveNativeCutterProof.Certified(move,physical,Cutter,out _);});
        var source=Source();var result=Fit(source,guard);
        Assert.NotNull(result.Fit);IndependentlyCheck(result.Fit!,source,new(Target,Cutter));
        Assert.True(result.Audit.VisitedNominees>=2);Assert.True(result.Audit.GeometryProofs>=2);
        if(refusePose){Assert.Equal(1,result.Audit.PoseRejected);Assert.Equal(0,result.Audit.ContinuousRejected);Assert.Equal(1,curves);}
        else{Assert.Equal(1,result.Audit.ContinuousRejected);Assert.Equal(2,curves);}
    }

    [Fact]
    public void ActualPriorStockStillSuppliesTheCompletePhysicalCertificate()
    {
        PointD[][] target=[[new(-.05,-.05),new(.05,-.05),new(.05,.05),new(-.05,.05)]];
        PointD[] part=[new(-10,-10),new(20,-10),new(20,10),new(-10,10)];
        var stock=new VCarveClearedStock([new(new(4.5,0,-1),new(4.5,0,-1),7,7)],[1],part);
        VCarveMaterialSurface Owner(VCarveClearedStock prior)=>new(target,prior,Cutter,1,0,default,null);
        var owner=Owner(stock);var source=Source();
        var result=Fit(source,new(target,Cutter,stockChord:owner.IsLegalNativeChord,stockCurve:owner.IsLegalNativeCurve));
        Assert.NotNull(result.Fit);
        var independent=Owner(stock);
        Assert.True(independent.IsLegalNativeCurve(result.Fit!.Move));
        Assert.True(VCarveNativeGeometricProof.Prove(result.Fit.Move,source,0,source.Length-1,.3).Certified);
        var unremoved=Owner(new([],[],part));
        Assert.False(unremoved.IsLegalNativeCurve(result.Fit.Move));
    }

    [Fact]
    public void ProtectedMaterialCannotBeWaivedByTheSelectionPolicy()
    {
        PointD[][] target=[Target[0],[new(3.5,-1),new(3.5,2),new(5.5,2),new(5.5,-1)]];
        var result=Fit(Source(),new(target,Cutter));
        Assert.Null(result.Fit);Assert.True(result.Audit.PoseRejected+result.Audit.ContinuousRejected>0);
        Assert.Equal(result.Audit.CandidateInventory,result.Audit.VisitedNominees);
    }

    [Fact]
    public void LibraryDefaultRetainsExistingRankingAndRejectsUnknownPolicies()
    {
        var defaults=new VCarveNativeFitSettings();
        Assert.Equal(VCarveNativeCandidateSelection.MinimumError,defaults.CandidateSelection);defaults.Validate();
        Assert.Throws<ArgumentOutOfRangeException>(()=>(defaults with{CandidateSelection=(VCarveNativeCandidateSelection)123}).Validate());
    }
}
