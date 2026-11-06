using System.Text.RegularExpressions;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeMinimaxDistanceElisionTests
{
    [Fact]
    public void CapabilityCopiesByIdentityWithoutChangingSemanticSettingsContracts()
    {
        var (_,source)=Fixture(17);var copy=source.ToArray();
        var a=new VCarveNativeFitSettings(.03,.3){CandidateSelection=VCarveNativeCandidateSelection.FirstVerified,MaximumCandidateGeometryProofs=32};
        var b=a with{OwnedMinimaxSource=VCarveNativeGeometricProof.OwnedMinimaxSource.CapturePrivateSnapshot(source)};
        var c=a with{OwnedMinimaxSource=VCarveNativeGeometricProof.OwnedMinimaxSource.CapturePrivateSnapshot(copy)};
        Assert.Equal(a,b);Assert.Equal(b,c);Assert.Equal(a.GetHashCode(),b.GetHashCode());Assert.Equal(b.GetHashCode(),c.GetHashCode());
        Assert.Equal(a.ToString(),b.ToString());Assert.Equal(System.Text.Json.JsonSerializer.Serialize(a),System.Text.Json.JsonSerializer.Serialize(b));
        Assert.Same(b.OwnedMinimaxSource,(b with{PreservePlanarSourceProgress=true}).OwnedMinimaxSource);
        Assert.True(b.OwnedMinimaxSource!.Matches(source));Assert.False(b.OwnedMinimaxSource.Matches(copy));
        Assert.NotEqual(a,a with{FitErrorMm=.04});Assert.NotEqual(a,a with{MinimumPrimitiveLengthMm=.4});
        Assert.NotEqual(a,a with{PreservePlanarSourceProgress=true});Assert.NotEqual(a,a with{MaximumCandidateGeometryProofs=33});
        Assert.NotEqual(a,a with{CandidateSelection=VCarveNativeCandidateSelection.MinimumError});
    }
    private static long[] Bits(double[] p)=>p.Select(BitConverter.DoubleToInt64Bits).ToArray();
    private static void Same(double[]? expected,double[]? actual)
    {if(expected is null){Assert.Null(actual);return;}Assert.NotNull(actual);Assert.Equal(Bits(expected),Bits(actual!));}
    private static void Same(VCarveNativeGeometryCertificate a,VCarveNativeGeometryCertificate b)
    {Assert.Equal(a.Certified,b.Certified);Assert.Equal(a.Reason,b.Reason);Assert.Equal(BitConverter.DoubleToInt64Bits(a.ErrorBound),BitConverter.DoubleToInt64Bits(b.ErrorBound));Assert.Equal(Bits(a.Parameters),Bits(b.Parameters));}
    private static (ToolMove Move,Point3D[] Source) Fixture(int count)
    {
        var move=ToolMove.CubicBezier(new(0,0,-1),new(1,.5,-1),new(3,-.5,-.5),new(4,0,-.5),0);
        return(move,Enumerable.Range(0,count).Select(i=>VCarveNativeCurveMath.At(move,i/(double)(count-1))).ToArray());
    }
    [Theory]
    [InlineData(2)] [InlineData(17)] [InlineData(33)] [InlineData(65)]
    public void OwnedFiniteGridRetainsExactEagerWitnessBitsIncludingSlabBoundaries(int count)
    {
        var (move,source)=Fixture(count);var cap=VCarveNativeGeometricProof.OwnedMinimaxSource.CapturePrivateSnapshot(source);
        Assert.NotNull(cap);long elided=0;
        var previous=VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value;
        try
        {
            VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value=true;
            foreach(double allowance in new[]{-0d,0d,.0001,.005,.03,.3,10d,double.NaN,double.PositiveInfinity,double.NegativeInfinity,-1d})
            {
                var expected=VCarveNativeGeometricProof.MinimaxOrderedParametersEager(move,source,0,count-1,allowance);
                using var work=VCarveNativeGeometricProof.BeginWorkCapture();
                var actual=VCarveNativeGeometricProof.MinimaxOrderedParameters(move,source,0,count-1,allowance,cap);
                Same(expected,actual);elided+=work.Snapshot().MinimaxDistanceSquareRootsElided;
                if(allowance==.03)Assert.NotNull(actual);
            }
            Assert.True(elided>0);
        }
        finally{VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value=previous;}
    }

    [Fact]
    public void OutwardThresholdNeverRejectsAnOriginallyAcceptedRoundedSquareRoot()
    {
        foreach(double allowance in new[]{-0d,0d,double.Epsilon,Math.Sqrt(double.Epsilon),1e-150,.001,.03,.3,1d,1e6,1e150})
        {
            Assert.True(VCarveNativeGeometricProof.TryMinimaxSquaredThreshold(allowance,out double upper));
            double square=allowance*allowance;
            foreach(double x in new[]{0d,double.Epsilon,Math.BitDecrement(square),square,Math.BitIncrement(square),
                Math.BitDecrement(upper),upper,Math.BitIncrement(upper),double.PositiveInfinity})
                if(x>=0&&x>upper)Assert.True(Math.Sqrt(x)>allowance,$"s={allowance:R} x={x:R} upper={upper:R}");
            // Direct original sqrt controls surround the LAST possibly
            // accepted binary64 distance, including exponent transitions.
            double cursor=upper;
            for(int i=0;i<16;i++){cursor=Math.BitIncrement(cursor);Assert.True(Math.Sqrt(cursor)>allowance);}
        }
        foreach(double allowance in new[]{double.NaN,double.PositiveInfinity,double.NegativeInfinity,-1d,double.MaxValue})
            Assert.False(VCarveNativeGeometricProof.TryMinimaxSquaredThreshold(allowance,out _));
    }

    [Theory]
    [InlineData(0)] [InlineData(1)] [InlineData(2)]
    public void GenericSourcesMismatchedSnapshotsAndExplicitOffKeepLiteralGetterOrder(int mode)
    {
        var (move,array)=Fixture(33);var expectedReads=new RecordingSource(array);var reads=new RecordingSource(array);
        var expected=VCarveNativeGeometricProof.MinimaxOrderedParametersEager(move,expectedReads,0,32,.01);
        var prior=VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value;
        try
        {
            VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value=mode==2?false:true;
            var cap=VCarveNativeGeometricProof.OwnedMinimaxSource.CapturePrivateSnapshot(mode==1?array.ToArray():array);
            using var capture=VCarveNativeGeometricProof.BeginWorkCapture();
            var actual=VCarveNativeGeometricProof.MinimaxOrderedParameters(move,reads,0,32,.01,cap);
            Same(expected,actual);Assert.Equal(expectedReads.Reads,reads.Reads);
            Assert.Equal(0,capture.Snapshot().MinimaxDistanceSquareRootsElided);
            Assert.Equal(0,capture.Snapshot().MinimaxDistanceElisionCalls);
            // Even a raw array, without the matching private capability,
            // must not infer immutability from its runtime type.
            using var arrayCapture=VCarveNativeGeometricProof.BeginWorkCapture();
            Same(expected,VCarveNativeGeometricProof.MinimaxOrderedParameters(move,array,0,32,.01,mode==1?cap:null));
            Assert.Equal(0,arrayCapture.Snapshot().MinimaxDistanceSquareRootsElided);
            if(mode==2)
            {
                Same(expected,VCarveNativeGeometricProof.MinimaxOrderedParameters(move,array,0,32,.01,cap));
                Assert.Equal(0,arrayCapture.Snapshot().MinimaxDistanceElisionCalls);
            }
        }
        finally{VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value=prior;}
    }

    [Fact]
    public void NonfiniteOutOfBoundsAndOverflowUseTheOriginalArithmetic()
    {
        var (move,source)=Fixture(17);var cap=VCarveNativeGeometricProof.OwnedMinimaxSource.CapturePrivateSnapshot(source)!;
        foreach(double value in new[]{double.NaN,double.PositiveInfinity,double.NegativeInfinity,1_000_001d})
        {
            var invalid=source.ToArray();invalid[1]=invalid[1] with{Y=value};
            Assert.Null(VCarveNativeGeometricProof.OwnedMinimaxSource.CapturePrivateSnapshot(invalid));
        }
        foreach(double allowance in new[]{double.NaN,double.PositiveInfinity,double.NegativeInfinity,-1d,double.MaxValue})
        {
            using var capture=VCarveNativeGeometricProof.BeginWorkCapture();
            Same(VCarveNativeGeometricProof.MinimaxOrderedParametersEager(move,source,0,16,allowance),
                VCarveNativeGeometricProof.MinimaxOrderedParameters(move,source,0,16,allowance,cap));
            Assert.Equal(0,capture.Snapshot().MinimaxDistanceSquareRootsElided);
        }
        foreach(double value in new[]{double.NaN,double.PositiveInfinity,1_000_001d})
        {
            var invalid=move with{Control1=new(value,0,0)};
            using var capture=VCarveNativeGeometricProof.BeginWorkCapture();
            Same(VCarveNativeGeometricProof.MinimaxOrderedParametersEager(invalid,source,0,16,.3),
                VCarveNativeGeometricProof.MinimaxOrderedParameters(invalid,source,0,16,.3,cap));
            Assert.Equal(0,capture.Snapshot().MinimaxDistanceElisionCalls);
        }
    }

    [Fact]
    public void CompleteOwnedProofMatchesOriginalCertificateAndActuallyExercisesElision()
    {
        var move=ToolMove.CubicBezier(new(0,0,0),new(0,1,0),new(0,2,0),new(10,3,10),0);
        var source=Enumerable.Range(0,101).Select(i=>VCarveNativeCurveMath.At(move,i/100d)).ToArray();source[1]=source[1] with{Z=source[1].Z+.29};
        var cap=VCarveNativeGeometricProof.OwnedMinimaxSource.CapturePrivateSnapshot(source)!;
        var old=VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value;
        var order=VCarveNativeGeometricProof.CheapProjectionsFirstOverride.Value;long skipped=0,calls=0;
        try
        {
            foreach(bool cheap in new[]{false,true})foreach(double allowance in new[]{.001,.1,.28,.3})
            {
                VCarveNativeGeometricProof.CheapProjectionsFirstOverride.Value=cheap;
                VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value=false;
                var expected=VCarveNativeGeometricProof.ProveOwned(move,source,0,100,allowance,cap);
                VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value=true;
                using var capture=VCarveNativeGeometricProof.BeginWorkCapture();
                var actual=VCarveNativeGeometricProof.ProveOwned(move,source,0,100,allowance,cap);Same(expected,actual);
                var work=capture.Snapshot();skipped+=work.MinimaxDistanceSquareRootsElided;calls+=work.MinimaxDistanceElisionCalls;
                if(actual.Certified)Assert.True(VCarveNativeGeometricProof.CorrespondenceForOracle(move,source,0,100,allowance,actual.Parameters,true).Certified);
            }
            Assert.True(calls>0);Assert.True(skipped>0);
        }
        finally{VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value=old;VCarveNativeGeometricProof.CheapProjectionsFirstOverride.Value=order;}
    }

    [Fact]
    public void NominatePrivateSnapshotKeepsFullVerifiedPlansAndSourceWhileExercisingOwnedStorage()
    {
        // The first/last two points are exactly horizontal. This makes the
        // full-span straight probe obligatory before cheap cubic exclusion.
        // Interior waves force that probe's ordered-correspondence DP to fail;
        // its unchanged literal source-line fallback remains fully legal and
        // comfortably longer than the configured primitive minimum.
        var points=Enumerable.Range(0,65).Select(i=>{double t=i/64d;return new Point3D(10*t,
            i is 0 or 1 or 63 or 64?0:.4*Math.Sin(4*Math.PI*t),-.5);}).ToArray();
        var path=new TaggedToolpath(points,[new(0,64,ToolpathSpanKind.Cut)],0,"owned-sqrt",0,0,detectCurves:false);
        var before=points.ToArray();var logs=new List<string>();
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,2,2)],2);
        IReadOnlyList<VCarveNativeCurvePlan> Run()=>VCarveNativeCurvePlanning.Nominate([path],[[new PointD[]{new(-20,-20),new(20,-20),new(20,20),new(-20,20)}]],
            profile,0,limits:new(3000),diagnostic:logs.Add,settings:new(.001,.001){CandidateSelection=VCarveNativeCandidateSelection.FirstVerified,MaximumCandidateGeometryProofs=32},
            selectedSourceOrdinals:[new[]{0,64}]);
        var old=VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value;
        try
        {
            VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value=false;var expected=Run();logs.Clear();
            VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value=true;var actual=Run();
            Assert.Equal(expected.SelectMany(p=>p.Primitives),actual.SelectMany(p=>p.Primitives));
            Assert.Equal(expected.Select(p=>(p.Status,p.CandidateVerified,p.BudgetExhausted)),actual.Select(p=>(p.Status,p.CandidateVerified,p.BudgetExhausted)));
            Assert.All(actual,p=>Assert.True(p.CandidateVerified,p.Status));Assert.Equal(before,points);
            long skipped=logs.Where(l=>l.StartsWith("[VCarve.NativeGeometryWork]")).Select(l=>Regex.Match(l,@"minimaxDistanceSquareRootsElided=(\d+)"))
                .Where(m=>m.Success).Sum(m=>long.Parse(m.Groups[1].Value));
            Assert.True(skipped>0,string.Join(Environment.NewLine,logs.Where(l=>l.StartsWith("[VCarve.NativeGeometryWork]"))));
        }
        finally{VCarveNativeGeometricProof.MinimaxDistanceElisionOverride.Value=old;}
    }

    private sealed class RecordingSource(Point3D[] source):IReadOnlyList<Point3D>
    {
        internal readonly List<int> Reads=[];public int Count=>source.Length;
        public Point3D this[int i]{get{Reads.Add(i);return source[i];}}
        public IEnumerator<Point3D> GetEnumerator()=>((IEnumerable<Point3D>)source).GetEnumerator();
        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()=>GetEnumerator();
    }
}
