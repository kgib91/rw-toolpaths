using System.Runtime.CompilerServices;
using System.Text.Json;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class NativeGeometryValueQuadTests(ITestOutputHelper output)
{
    private static long[] Bits(IEnumerable<double> values)=>values.Select(BitConverter.DoubleToInt64Bits).ToArray();
    private static long[] Bits(IEnumerable<Point3D> values)=>Bits(values.SelectMany(p=>new[]{p.X,p.Y,p.Z}));
    private static void Same(VCarveNativeGeometryCertificate literal,VCarveNativeGeometryCertificate actual)
    {
        Assert.Equal(literal.Certified,actual.Certified);Assert.Equal(literal.Reason,actual.Reason);
        Assert.Equal(BitConverter.DoubleToInt64Bits(literal.ErrorBound),BitConverter.DoubleToInt64Bits(actual.ErrorBound));
        Assert.Equal(Bits(literal.Parameters),Bits(actual.Parameters));
    }
    private static ToolMove Curve=>ToolMove.CubicBezier(new(0,0,-1),new(1,1,-1.5),new(2,-1,-.7),new(3,0,-1.1),0);

    [Theory]
    [InlineData(2)] [InlineData(16)] [InlineData(128)] [InlineData(1024)]
    public void CompleteCertificatesKeepBitsForRegularReversedRepeatedAndFreeJoinSources(int divisions)
    {
        var curve=Curve;
        var original=Enumerable.Range(0,divisions+1).Select(i=>VCarveNativeCurveMath.At(curve,i/(double)divisions)).ToArray();
        foreach(bool reverse in new[]{false,true})foreach(bool repeated in new[]{false,true})
        {
            var move=reverse?ToolMove.CubicBezier(curve.End,curve.Control2,curve.Control1,curve.Start,0):curve;
            var ordered=reverse?original.Reverse().ToArray():original;
            var source=repeated?ordered.SelectMany((p,i)=>i%3==0?new[]{p,p}:new[]{p}).ToArray():ordered.ToArray();
            var before=Bits(source);
            foreach(double allowance in new[]{.3,.003})
            {
                Same(VCarveNativeGeometricProof.ProveLiteral(move,source,0,source.Length-1,allowance),
                    VCarveNativeGeometricProof.Prove(move,source,0,source.Length-1,allowance));
                var free=move with{Start=move.Start with{Y=move.Start.Y+.001},End=move.End with{Z=move.End.Z-.001}};
                Same(VCarveNativeGeometricProof.ProveFreeJoinLiteral(free,source,0,source.Length-1,allowance),
                    VCarveNativeGeometricProof.ProveFreeJoin(free,source,0,source.Length-1,allowance));
            }
            Assert.Equal(before,Bits(source));
        }
    }

    [Fact]
    public void RequiredInteriorAnchorsAndEscapingParameterArraysKeepTheirOriginalOwnership()
    {
        var move=Curve;var source=Enumerable.Range(0,129).Select(i=>VCarveNativeCurveMath.At(move,i/128d)).ToArray();
        foreach(var anchors in new[]{new[]{new VCarveNativeInteriorPose(64,.5)},
            new[]{new VCarveNativeInteriorPose(64,.6)},new[]{new VCarveNativeInteriorPose(96,.75),new VCarveNativeInteriorPose(64,.5)}})
            Same(VCarveNativeGeometricProof.ProveLiteral(move,source,0,128,.3,anchors),
                VCarveNativeGeometricProof.Prove(move,source,0,128,.3,anchors));
        var literal=VCarveNativeGeometricProof.ProveLiteral(move,source,0,128,.3);
        var actual=VCarveNativeGeometricProof.Prove(move,source,0,128,.3);
        Same(literal,actual);Assert.True(actual.Certified);Assert.NotSame(literal.Parameters,actual.Parameters);
        double prior=literal.Parameters[1];actual.Parameters[1]=900;Assert.Equal(prior,literal.Parameters[1]);
        var parameters=Enumerable.Range(0,129).Select(i=>i/128d).ToArray();var before=Bits(parameters);
        var direct=VCarveNativeGeometricProof.CorrespondenceForOracle(move,source,0,128,.3,parameters,false);
        Assert.Same(parameters,direct.Parameters);Assert.Equal(before,Bits(parameters));
    }

    private sealed record Recorded(int Region,Point3D[] Source,ToolMove Move);
    [Fact]
    public void RecordedActualCurvesAndContactCorrespondencesKeepCompleteCertificateBits()
    {
        var assembly=typeof(NativeGeometryValueQuadTests).Assembly;
        using(var input=assembly.GetManifestResourceStream("RW.Toolpaths.Tests.Fixtures.reviewed-geometric-correspondence.json")!)
        {
            var cases=JsonSerializer.Deserialize<Recorded[]>(input,new JsonSerializerOptions{IncludeFields=true})!;
            Assert.Equal(4,cases.Length);
            foreach(var item in cases)foreach(double allowance in new[]{.3,.28})
                Same(VCarveNativeGeometricProof.ProveLiteral(item.Move,item.Source,0,item.Source.Length-1,allowance),
                    VCarveNativeGeometricProof.Prove(item.Move,item.Source,0,item.Source.Length-1,allowance));
        }
        using(var input=assembly.GetManifestResourceStream(assembly.GetManifestResourceNames().Single(n=>n.EndsWith("contact-ordered-correspondence.json")))!)
        using(var json=JsonDocument.Parse(input))
        {
            var options=new JsonSerializerOptions{IncludeFields=true};var move=JsonSerializer.Deserialize<ToolMove>(json.RootElement.GetProperty("Move"),options);
            var source=JsonSerializer.Deserialize<Point3D[]>(json.RootElement.GetProperty("Reference"),options)!;
            foreach(double allowance in new[]{.3,.28})
                Same(VCarveNativeGeometricProof.ProveFreeJoinLiteral(move,source,0,source.Length-1,allowance),
                    VCarveNativeGeometricProof.ProveFreeJoin(move,source,0,source.Length-1,allowance));
        }
    }

    [Fact]
    public void LiteralBoundOracleKeepsNaNPayloadsInfinitySignedZeroDepthAndBudgetTransitions()
    {
        double nanA=BitConverter.Int64BitsToDouble(0x7ff8000000000001L),nanB=BitConverter.Int64BitsToDouble(unchecked((long)0xfff8000000000002UL));
        Point3D[][] cases=[
            [new(0,0,0),new(.39,0,0),new(-.39,0,0),new(0,0,0)],
            [new(-0d,0,0),new(0,-0d,0),new(-0d,-0d,0),new(0,0,-0d)],
            [new(nanA,0,0),new(.3,0,0),new(nanB,0,0),new(.2,0,0)],
            [new(nanA,0,0),new(nanB,0,0),new(nanA,0,0),new(nanB,0,0)],
            [new(double.PositiveInfinity,0,0),new(1,0,0),new(-1,0,0),new(double.NegativeInfinity,0,0)],
            [new(0,0,0),new(.3-1e-9,0,0),new(.3-1e-9,0,0),new(0,0,0)]];
        foreach(var controls in cases)foreach(double allowance in new[]{.3,Math.BitDecrement(.3),Math.BitIncrement(.3),double.NaN,double.PositiveInfinity})
        foreach(int depth in new[]{0,13,14})foreach(int remaining in new[]{-1,0,1,4})
        {
            var expected=VCarveNativeGeometricProof.BoundForOracle(controls,allowance,depth,remaining,true);
            var actual=VCarveNativeGeometricProof.BoundForOracle(controls,allowance,depth,remaining,false);
            Assert.Equal(expected.Certified,actual.Certified);Assert.Equal(expected.Remaining,actual.Remaining);
            Assert.Equal(BitConverter.DoubleToInt64Bits(expected.Bound),BitConverter.DoubleToInt64Bits(actual.Bound));
        }
        Point3D[] needingSplit=[new(0,0,0),new(.39,0,0),new(-.39,0,0),new(0,0,0)];
        var exhausted=VCarveNativeGeometricProof.BoundForOracle(needingSplit,.1,0,0,false);
        Assert.False(exhausted.Certified);Assert.Equal(-1,exhausted.Remaining);
    }

    [Fact]
    public void LinesUnsupportedMovesClosedAndReversingSourcesKeepTheirRefusalReasons()
    {
        Point3D[][] sources=[[new(0,0,0),new(1,0,0),new(.999,0,0)],
            [new(0,0,0),new(2,0,0),new(.2,0,0),new(3,0,0)],[new(0,0,0),new(1,0,0),new(0,0,0)]];
        foreach(var source in sources)
        {
            var line=ToolMove.Line(source[0],source[^1]);var loop=ToolMove.CubicBezier(source[0],new(3,2,0),new(-2,-2,0),source[^1],0);
            foreach(var move in new[]{line,loop,line with{Kind=ToolMoveKind.ArcCw}})
                Same(VCarveNativeGeometricProof.ProveLiteral(move,source,0,source.Length-1,.3),
                    VCarveNativeGeometricProof.Prove(move,source,0,source.Length-1,.3));
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static double Repeat(ToolMove move,Point3D[] source,int count,bool literal)
    {
        double result=0;for(int i=0;i<count;i++)
            result+=(literal?VCarveNativeGeometricProof.ProveLiteral(move,source,0,source.Length-1,.3)
                :VCarveNativeGeometricProof.Prove(move,source,0,source.Length-1,.3)).ErrorBound;
        return result;
    }
    [Fact]
    public void WarmedCompleteProofAllocationsFallWhileCertificateBitsStayIdentical()
    {
        var move=Curve;var source=Enumerable.Range(0,257).Select(i=>VCarveNativeCurveMath.At(move,i/256d)).ToArray();
        for(int warm=0;warm<8;warm++){Repeat(move,source,1,true);Repeat(move,source,1,false);}
        long before=GC.GetAllocatedBytesForCurrentThread();double literal=Repeat(move,source,16,true);long literalBytes=GC.GetAllocatedBytesForCurrentThread()-before;
        before=GC.GetAllocatedBytesForCurrentThread();double actual=Repeat(move,source,16,false);long actualBytes=GC.GetAllocatedBytesForCurrentThread()-before;
        output.WriteLine($"completeProofs=16 sourceChords={source.Length-1} literalAllocatedBytes={literalBytes} valueQuadAllocatedBytes={actualBytes} savedBytes={literalBytes-actualBytes}");
        Assert.Equal(BitConverter.DoubleToInt64Bits(literal),BitConverter.DoubleToInt64Bits(actual));
        Assert.True(actualBytes*4<literalBytes,$"Expected transient quad removal; literal={literalBytes}, value={actualBytes}.");
    }
}
