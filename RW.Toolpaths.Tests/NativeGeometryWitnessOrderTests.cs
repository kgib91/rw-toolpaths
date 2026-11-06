using System.Text.Json;
using System.Text.RegularExpressions;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class NativeGeometryWitnessOrderTests(ITestOutputHelper output)
{
    private static long[] Bits(IEnumerable<double> values)=>values.Select(BitConverter.DoubleToInt64Bits).ToArray();
    private static (ToolMove Move,Point3D[] Source) LaterProjectionFixture()
    {
        var move=ToolMove.CubicBezier(new(0,0,0),new(0,1,0),new(0,2,0),new(10,3,10),0);
        var source=Enumerable.Range(0,101).Select(i=>VCarveNativeCurveMath.At(move,i/100d)).ToArray();
        source[1]=source[1] with{Z=source[1].Z+.29};
        return(move,source);
    }
    private static void SameFailure(VCarveNativeGeometryCertificate original,VCarveNativeGeometryCertificate actual)
    {
        Assert.False(original.Certified);Assert.False(actual.Certified);Assert.Equal(original.Reason,actual.Reason);
        Assert.Equal(BitConverter.DoubleToInt64Bits(original.ErrorBound),BitConverter.DoubleToInt64Bits(actual.ErrorBound));
        Assert.Equal(Bits(original.Parameters),Bits(actual.Parameters));
    }
    private static void CheckCertificate(ToolMove move,Point3D[] source,double allowance,VCarveNativeGeometryCertificate proof)
    {
        Assert.True(proof.Certified,proof.Reason);Assert.InRange(proof.ErrorBound,0,allowance);
        Assert.Equal(0,proof.Parameters[0]);Assert.Equal(1,proof.Parameters[^1]);
        Assert.True(proof.Parameters.Zip(proof.Parameters.Skip(1)).All(p=>p.First<=p.Second));
        // Independent unchanged array-based Bernstein arithmetic replays the
        // selected witness over the COMPLETE original source, not samples.
        Assert.True(VCarveNativeGeometricProof.CorrespondenceForOracle(move,source,0,source.Length-1,
            allowance,proof.Parameters,true).Certified);
    }

    [Fact]
    public void LaterCheapProjectionAvoidsTheEarlierAlternativeHookAndStillHasACompleteCertificate()
    {
        var (move,source)=LaterProjectionFixture();
        VCarveNativeGeometryCertificate original,actual;VCarveNativeGeometricProof.WorkSnapshot oldWork,newWork;
        using(var capture=VCarveNativeGeometricProof.BeginWorkCapture())
        {original=VCarveNativeGeometricProof.ProveOriginalOrder(move,source,0,100,.3);oldWork=capture.Snapshot();}
        using(var capture=VCarveNativeGeometricProof.BeginWorkCapture())
        {actual=VCarveNativeGeometricProof.ProveCheapFirst(move,source,0,100,.3);newWork=capture.Snapshot();}
        CheckCertificate(move,source,.3,original);CheckCertificate(move,source,.3,actual);
        Assert.True(oldWork.ClosestCalls>0);Assert.Equal(0,newWork.ClosestCalls);Assert.Equal(0,newWork.MinimaxCalls);
        Assert.Equal(1,newWork.CheapWinsBeforePriorAlternatives);
        output.WriteLine("original "+oldWork.Describe());output.WriteLine("cheapFirst "+newWork.Describe());
    }

    [Fact]
    public void AllFailedWitnessesRetainTheOriginalReasonErrorBitsParametersAndStrictFailureTies()
    {
        var (move,source)=LaterProjectionFixture();
        foreach(double allowance in new[]{.001,.1,.28})
            SameFailure(VCarveNativeGeometricProof.ProveOriginalOrder(move,source,0,100,allowance),
                VCarveNativeGeometricProof.ProveCheapFirst(move,source,0,100,allowance));
        Point3D[] reversed=[new(0,0,0),new(2,0,0),new(.2,0,0),new(3,0,0)];
        var line=ToolMove.Line(reversed[0],reversed[^1]);
        SameFailure(VCarveNativeGeometricProof.ProveOriginalOrder(line,reversed,0,3,.3),
            VCarveNativeGeometricProof.ProveCheapFirst(line,reversed,0,3,.3));
        var loop=ToolMove.CubicBezier(new(0,0,0),new(3,2,0),new(-2,-2,0),new(1,0,0),0);
        Point3D[] straight=[loop.Start,loop.End];
        SameFailure(VCarveNativeGeometricProof.ProveOriginalOrder(loop,straight,0,1,.3),
            VCarveNativeGeometricProof.ProveCheapFirst(loop,straight,0,1,.3));
    }

    private sealed record Recorded(int Region,Point3D[] Source,ToolMove Move);
    [Fact]
    public void RecordedCurvesFreeJoinsAndInteriorAnchorsPreserveCertificationWithFullOriginalProofs()
    {
        var assembly=typeof(NativeGeometryWitnessOrderTests).Assembly;
        using(var input=assembly.GetManifestResourceStream("RW.Toolpaths.Tests.Fixtures.reviewed-geometric-correspondence.json")!)
        {
            var cases=JsonSerializer.Deserialize<Recorded[]>(input,new JsonSerializerOptions{IncludeFields=true})!;
            foreach(var item in cases)foreach(double allowance in new[]{.3,.28,.01})
            {
                var old=VCarveNativeGeometricProof.ProveOriginalOrder(item.Move,item.Source,0,item.Source.Length-1,allowance);
                var actual=VCarveNativeGeometricProof.ProveCheapFirst(item.Move,item.Source,0,item.Source.Length-1,allowance);
                Assert.Equal(old.Certified,actual.Certified);
                if(actual.Certified)CheckCertificate(item.Move,item.Source,allowance,actual);else SameFailure(old,actual);
            }
        }
        using(var input=assembly.GetManifestResourceStream(assembly.GetManifestResourceNames().Single(n=>n.EndsWith("contact-ordered-correspondence.json")))!)
        using(var json=JsonDocument.Parse(input))
        {
            var options=new JsonSerializerOptions{IncludeFields=true};var move=JsonSerializer.Deserialize<ToolMove>(json.RootElement.GetProperty("Move"),options);
            var source=JsonSerializer.Deserialize<Point3D[]>(json.RootElement.GetProperty("Reference"),options)!;
            foreach(double allowance in new[]{.3,.28})
            {
                var old=VCarveNativeGeometricProof.ProveFreeJoinOriginalOrder(move,source,0,source.Length-1,allowance);
                var actual=VCarveNativeGeometricProof.ProveFreeJoinCheapFirst(move,source,0,source.Length-1,allowance);
                Assert.Equal(old.Certified,actual.Certified);
                if(actual.Certified)CheckCertificate(move,source,allowance,actual);else SameFailure(old,actual);
            }
        }
        var (anchored,_) = LaterProjectionFixture();
        var points=Enumerable.Range(0,101).Select(i=>VCarveNativeCurveMath.At(anchored,i/100d)).ToArray();
        VCarveNativeInteriorPose[] poses=[new(50,.5)];
        var proof=VCarveNativeGeometricProof.ProveCheapFirst(anchored,points,0,100,.3,poses);
        CheckCertificate(anchored,points,.3,proof);Assert.Equal(.5,proof.Parameters[50]);
    }

    [Fact]
    public void WorkCaptureIsThreadPrivateAndNestedScopesRestoreTheirPreviousOwner()
    {
        var (move,source)=LaterProjectionFixture();
        using var outer=VCarveNativeGeometricProof.BeginWorkCapture();
        VCarveNativeGeometricProof.ProveCheapFirst(move,source,0,100,.3);
        using(var inner=VCarveNativeGeometricProof.BeginWorkCapture())
        {
            Parallel.For(0,24,_=>Assert.True(VCarveNativeGeometricProof.ProveCheapFirst(move,source,0,100,.3).Certified));
            Assert.Equal(24,inner.Snapshot().Proofs);
        }
        VCarveNativeGeometricProof.ProveCheapFirst(move,source,0,100,.3);
        Assert.Equal(2,outer.Snapshot().Proofs);
    }

    [Theory]
    [InlineData(VCarveNativeCandidateSelection.MinimumError,null,false)]
    [InlineData(VCarveNativeCandidateSelection.FirstVerified,null,false)]
    [InlineData(VCarveNativeCandidateSelection.MinimumError,false,false)]
    [InlineData(VCarveNativeCandidateSelection.FirstVerified,false,false)]
    [InlineData(VCarveNativeCandidateSelection.MinimumError,true,true)]
    [InlineData(VCarveNativeCandidateSelection.FirstVerified,true,true)]
    public void NominationKeepsOriginalDefaultAndHonorsExplicitDiagnosticOrder(VCarveNativeCandidateSelection selection,bool? diagnosticOverride,bool expectedCheap)
    {
        var previous=VCarveNativeGeometricProof.CheapProjectionsFirstOverride.Value;
        try
        {
            VCarveNativeGeometricProof.CheapProjectionsFirstOverride.Value=diagnosticOverride;
            var source=new TaggedToolpath([new(0,0,-.2),new(1,0,-.2),new(2,0,-.2)],[],0,"scope",0,detectCurves:false);
            var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
            var logs=new List<string>();
            var plans=VCarveNativeCurvePlanning.Nominate([source],[[new PointD[]{new(-10,-10),new(10,-10),new(10,10),new(-10,10)}]],
                profile,0,diagnostic:logs.Add,settings:new(.3,.3){CandidateSelection=selection});
            Assert.All(plans,p=>Assert.True(p.CandidateVerified));
            string summary=Assert.Single(logs.Where(s=>s.StartsWith("[VCarve.NativeGeometryWork]")));
            long Number(string name)=>long.Parse(Regex.Match(summary,$@"\b{name}=(\d+)").Groups[1].Value);
            Assert.True(Number("proofCalls")>0);Assert.Equal(expectedCheap?Number("proofCalls"):0,Number("cheapFirstProofs"));
            Assert.Equal(diagnosticOverride,VCarveNativeGeometricProof.CheapProjectionsFirstOverride.Value);
            using var capture=VCarveNativeGeometricProof.BeginWorkCapture();
            VCarveNativeGeometricProof.Prove(ToolMove.Line(source.Points[0],source.Points[^1]),source.Points,0,2,.3);
            Assert.Equal(diagnosticOverride is true?1:0,capture.Snapshot().CheapFirstProofs);
        }
        finally{VCarveNativeGeometricProof.CheapProjectionsFirstOverride.Value=previous;}
    }

    private static void NominateForSummaryFailure(Action<string> diagnostic,CancellationToken token=default,
        Func<Point3D,Point3D,bool>? stockChord=null)
    {
        var source=new TaggedToolpath([new(0,0,-.2),new(1,0,-.2),new(2,0,-.2)],[],0,"scope-failure",0,detectCurves:false);
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
        VCarveNativeCurvePlanning.Nominate([source],[[new PointD[]{new(-10,-10),new(10,-10),new(10,10),new(-10,10)}]],
            profile,0,token:token,diagnostic:diagnostic,settings:new(.3,.3){CandidateSelection=VCarveNativeCandidateSelection.FirstVerified},
            stockChord:stockChord,stockCurve:stockChord is null?null:_=>true);
    }
    [Fact]
    public void ThrowingSummaryCannotReplaceTheOriginalStockAuthorityFailure()
    {
        var previous=VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value;
        try
        {
            VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value=false;
            var original=new ApplicationException("stock authority fixture");
            var error=Assert.Throws<ApplicationException>(()=>NominateForSummaryFailure(
                line=>{if(line.StartsWith("[VCarve.NativeGeometryWork]"))throw new InvalidOperationException("summary fixture");},
                stockChord:(_,_)=>throw original));
            Assert.Same(original,error);Assert.Equal(false,VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value);
        }
        finally{VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value=previous;}
    }
    [Fact]
    public void ThrowingSummaryCannotReplaceCancellationAndStillRestoresWitnessContext()
    {
        var previous=VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value;
        try
        {
            VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value=false;
            using var canceled=new CancellationTokenSource();canceled.Cancel();
            var error=Assert.ThrowsAny<OperationCanceledException>(()=>NominateForSummaryFailure(
                line=>{if(line.StartsWith("[VCarve.NativeGeometryWork]"))throw new InvalidOperationException("summary fixture");},canceled.Token));
            Assert.Equal(canceled.Token,error.CancellationToken);Assert.Equal(false,VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value);
        }
        finally{VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value=previous;}
    }
    [Fact]
    public void SummaryFailureAfterSuccessfulBodyPropagatesWithRestoredWitnessContext()
    {
        var previous=VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value;
        try
        {
            VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value=false;
            var summary=new ApplicationException("summary fixture");
            var error=Assert.Throws<ApplicationException>(()=>NominateForSummaryFailure(
                line=>{if(line.StartsWith("[VCarve.NativeGeometryWork]"))throw summary;}));
            Assert.Same(summary,error);Assert.Equal(false,VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value);
        }
        finally{VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value=previous;}
    }
}
