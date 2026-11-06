using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeFirstVerifiedEffortTests
{
    private static readonly VCarveCutterProfile Cutter=new([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
    private static readonly PointD[][] Target=[[new(-2,-2),new(11,-2),new(11,2),new(-2,2)]];
    private static Point3D[] Source()
    {
        var move=ToolMove.CubicBezier(new(0,0,-.5),new(3,1,-.5),new(6,1,-.5),new(9,0,-.5),0);
        return Enumerable.Range(0,97).Select(i=>VCarveNativeCurveMath.At(move,i/96d)).ToArray();
    }
    private static VCarveNativeFitSettings Settings(int maximum)=>new(.3,.3)
        {CandidateSelection=VCarveNativeCandidateSelection.FirstVerified,MaximumCandidateGeometryProofs=maximum,PreservePlanarSourceProgress=true};
    private static (VCarveNativeFit? Fit,VCarveNativeEdgeAudit Audit) Find(Point3D[] source,
        VCarveNativeContainment guard,VCarveNativeFitSettings settings,Point3D? start=null,Point3D? end=null)
    {
        double length=0;for(int i=1;i<source.Length;i++)length+=VCarveNativeCurveMath.Distance(source[i-1],source[i]);
        var u=start??VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[1],source[0]));
        var v=end??VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[^1],source[^2]));
        var fit=VCarveNativeEdgeSearch.Find(source,0,source.Length-1,u,v,length,guard,Cutter,out var audit,settings,
            parallelGeometryProofs:false);
        return(fit,audit);
    }

    [Fact]
    public void SuccessfulFirstCandidateRetainsItsIndependentFullCertificates()
    {
        var source=Source();var unchanged=source.ToArray();
        var result=Find(source,new(Target,Cutter),Settings(1));
        Assert.NotNull(result.Fit);var fit=result.Fit!;
        Assert.True(VCarveNativeGeometricProof.Prove(fit.Move,source,0,source.Length-1,.3).Certified);
        Assert.True(VCarveNativeCutterProof.Certified(fit.Move,new(Target,Cutter),Cutter,out _));
        Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(fit.Move,source,0,source.Length-1,out _));
        Assert.InRange(fit.Move.MaxError,0,.3);Assert.True(fit.LengthLower>=.3);
        Assert.Equal(source[0],fit.Move.Start);Assert.Equal(source[^1],fit.Move.End);
        Assert.Equal(1,result.Audit.SourceGeometryProofAttempts);
        Assert.False(result.Audit.CandidateGeometryLimitReached);
        Assert.Equal(unchanged,source);
    }

    [Theory]
    [InlineData(8)]
    [InlineData(32)]
    public void FailedStockPosesStopAfterTheConfiguredSourceProofs(int maximum)
    {
        var result=Find(Source(),new(Target,Cutter,stockChord:(_,_)=>false,stockCurve:_=>false),Settings(maximum));
        Assert.Null(result.Fit);
        Assert.Equal(maximum,result.Audit.SourceGeometryProofAttempts);
        Assert.Equal(maximum,result.Audit.MaximumCandidateGeometryProofs);
        Assert.True(result.Audit.CandidateGeometryLimitReached);
        Assert.Equal(1,result.Audit.CandidateGeometryLimitRefusals);
        Assert.Equal("candidate-geometry-proof-limit",result.Audit.Outcome);
        Assert.True(result.Audit.PoseRejected>0);
        Assert.Equal(0,result.Audit.ContinuousRejected);
    }

    [Fact]
    public void FailedCompleteCutterCertificateCannotBecomeAValidMoveAtTheLimit()
    {
        var physical=new VCarveNativeContainment(Target,Cutter);
        var guard=new VCarveNativeContainment(Target,Cutter,
            stockChord:physical.Legal,stockCurve:_=>false);
        var result=Find(Source(),guard,Settings(1));
        Assert.Null(result.Fit);Assert.Equal(1,result.Audit.SourceGeometryProofAttempts);
        Assert.Equal(1,result.Audit.ContinuousRejected);Assert.Equal(0,result.Audit.PoseRejected);
        Assert.True(result.Audit.CandidateGeometryLimitReached);
    }

    [Fact]
    public void CappedLongSpanUsesOrdinarySubdivisionAndEmitsOnlyFullyVerifiedMoves()
    {
        var source=Source();var physical=new VCarveNativeContainment(Target,Cutter);
        int longCurveRefusals=0;
        bool Chord(Point3D a,Point3D b)=>VCarveNativeCurveMath.Distance(a,b)<=3&&physical.Legal(a,b);
        bool Curve(ToolMove move)
        {
            if(VCarveNativeCurveMath.Distance(move.Start,move.End)>3){longCurveRefusals++;return false;}
            return VCarveNativeCutterProof.Certified(move,physical,Cutter,out _);
        }
        var path=new TaggedToolpath(source,[],0,"effort-cap-subdivision",0,detectCurves:false);
        var plan=Assert.Single(VCarveNativeCurvePlanning.Nominate([path],[Target],Cutter,0,
            settings:Settings(1),stockChord:Chord,stockCurve:Curve));
        Assert.True(longCurveRefusals>0);
        Assert.True(plan.CandidateVerified);Assert.False(plan.BudgetExhausted);
        Assert.True(plan.SearchRejections.GetValueOrDefault("candidate-geometry-proof-limit")>0);
        Assert.True(plan.Primitives.Count>1);
        foreach(var primitive in plan.Primitives)
        {
            Assert.True(primitive.LengthLowerMm>=.3);Assert.InRange(primitive.Move.MaxError,0,.3);
            Assert.True(VCarveNativeGeometricProof.Prove(primitive.Move,source,primitive.SourceFirst,primitive.SourceLast,.3).Certified);
            Assert.True(VCarveNativeCutterProof.Certified(primitive.Move,new(Target,Cutter,stockChord:Chord,stockCurve:Curve),Cutter,out _));
        }
    }

    [Fact]
    public void CheapDepthRefusalsDoNotConsumeSourceGeometryEffort()
    {
        var result=Find(Source(),new(Target,Cutter),Settings(1),
            VCarveNativeCurveMath.Unit(new(1,0,1)),VCarveNativeCurveMath.Unit(new(1,0,-1)));
        Assert.Null(result.Fit);Assert.True(result.Audit.DepthRejected>0);
        Assert.Equal(0,result.Audit.SourceGeometryProofAttempts);
        Assert.False(result.Audit.CandidateGeometryLimitReached);
    }

    [Fact]
    public void TheStraightLineProbeSharesTheSamePerFindLimit()
    {
        Point3D[] source=[new(0,0,-.5),new(1,0,-.5),new(2,0,-.5)];
        var result=Find(source,new(Target,Cutter,stockChord:(_,_)=>false,stockCurve:_=>false),Settings(1));
        Assert.Null(result.Fit);Assert.Equal(1,result.Audit.SourceGeometryProofAttempts);
        Assert.Equal(0,result.Audit.GeometryProofs);Assert.True(result.Audit.CandidateGeometryLimitReached);
    }

    [Fact]
    public void MinimumErrorIgnoresTheEffortCapAndLibraryFirstVerifiedDefaultsRemainUnbounded()
    {
        Assert.Equal(int.MaxValue,new VCarveNativeFitSettings().MaximumCandidateGeometryProofs);
        Assert.Equal(int.MaxValue,(new VCarveNativeFitSettings() with{CandidateSelection=VCarveNativeCandidateSelection.FirstVerified}).MaximumCandidateGeometryProofs);
        var source=Source();
        var original=Find(source,new(Target,Cutter),new(.3,.3));
        var ignored=Find(source,new(Target,Cutter),new(.3,.3){MaximumCandidateGeometryProofs=1});
        Assert.Equal(original.Fit,ignored.Fit);
        Assert.Equal(original.Audit.SourceGeometryProofAttempts,ignored.Audit.SourceGeometryProofAttempts);
        Assert.True(ignored.Audit.SourceGeometryProofAttempts>1);
        Assert.Equal(int.MaxValue,ignored.Audit.MaximumCandidateGeometryProofs);
        Assert.False(ignored.Audit.CandidateGeometryLimitReached);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void NonpositiveEffortLimitsAreRejected(int limit)
    {
        var settings=Settings(limit);
        Assert.Throws<ArgumentOutOfRangeException>(settings.Validate);
        Assert.Throws<ArgumentOutOfRangeException>(()=>Find(Source(),new(Target,Cutter),settings));
    }
}
