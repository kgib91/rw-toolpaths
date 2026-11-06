using System.Reflection;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class RecursiveFloorEnclosureReuseTests
{
    private const double Grade=.5;
    private static readonly VCarveCutterProfile Cutter=new(
        [new(CarvingCutterSectionKind.Linear,.1,.1,4)],4);

    [Theory]
    [InlineData(false,0d)]
    [InlineData(true,0d)]
    [InlineData(false,4096d)]
    [InlineData(true,-4096d)]
    public void SharedEndpointsRetainLiteralFirstQueryOrderAndEveryFloorAnswer(bool reverse,double offset)
    {
        var source=Path([new(offset-10,0,-1),new(offset+10,0,-1)]);
        var literalField=Field(source);var reusedField=Field(source);
        var literal=Proof(literalField,false);var reused=Proof(reusedField,true);
        var a=new Point3D(offset-1.75,0,-.9);var b=new Point3D(offset+1.75,0,-.9);
        if(reverse)(a,b)=(b,a);
        Assert.True(literal.Prove(a,b));Assert.True(reused.Prove(a,b));
        // This fixture requires continuous interior subdivision, not just an
        // endpoint check. The independently known floor is the plane z=-1.
        // A 3.5 mm chord requires four binary levels under the literal Lipschitz
        // certificate; the shorter fixture closed after only seven nodes.
        Assert.True(literal.Pieces>7);
        EqualProofAndQueries(literal,reused,literalField,reusedField);
        Assert.Equal(2*literal.Pieces,literal.ComputedEndpointValues);
        Assert.Equal(literal.ComputedEndpointValues,
            reused.ComputedEndpointValues+reused.ReusedEndpointValues);
        Assert.True(reused.ComputedEndpointValues*2<literal.ComputedEndpointValues);
        Assert.Contains("endpointValueReuse=True",reused.Diagnostics());
    }

    [Fact]
    public void InteriorRefusalStillShortCircuitsBeforeAnyRightChildFieldQuery()
    {
        var source=Path([new(-.01,0,-.5),new(.01,0,-.5)]);
        var literalField=Field(source);var reusedField=Field(source);
        var literal=Proof(literalField,false);var reused=Proof(reusedField,true);
        var a=new Point3D(-2,0,-.8);var b=new Point3D(2,0,-.8);
        Assert.False(literal.Prove(a,b));Assert.False(reused.Prove(a,b));
        EqualProofAndQueries(literal,reused,literalField,reusedField);
        Assert.Equal(2,literal.Pieces);
        Assert.Equal(new[]{(-2d,0d),(2d,0d),(0d,0d)},Cache(reusedField).Select(p=>p.Key).ToArray());
        Assert.Equal(3,reused.ComputedEndpointValues);Assert.Equal(1,reused.ReusedEndpointValues);
    }

    [Fact]
    public void MaximumSubdivisionRefusalKeepsTheOriginalDepthAndVisitChecks()
    {
        var source=Path([new(-10,0,-1),new(10,0,-1)]);
        var literalField=Field(source);var reusedField=Field(source);
        var literal=Proof(literalField,false);var reused=Proof(reusedField,true);
        var a=new Point3D(-.005,0,-(1-1e-12));var b=new Point3D(.005,0,a.Z);
        Assert.False(literal.Prove(a,b));Assert.False(reused.Prove(a,b));
        EqualProofAndQueries(literal,reused,literalField,reusedField);
        Assert.Equal(29,literal.Pieces);
        Assert.Equal(1,Counter(literal,"_unresolvedRefusals"));
        Assert.True(reused.ReusedEndpointValues>0);
    }

    [Theory]
    [InlineData(0d)]
    [InlineData(12.7)]
    [InlineData(-12.7)]
    public void NativeAndPostedChordsAndRoundedBoundaryRefusalsMatchTheLiteralProof(double surface)
    {
        var source=Path([new(-10,0,-.9997),new(10,0,-.9997)]);
        var literalField=Field(source);var reusedField=Field(source);
        var literal=Proof(literalField,false);var reused=Proof(reusedField,true);
        Point3D Q(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),
            ToolpathCommandRounding.Round((float)p.Y),
            ToolpathCommandRounding.Round((float)(surface+p.Z)-(float)surface));
        (Point3D A,Point3D B)[] queries=[
            (new(-.4,0,-.9),new(.4,0,-.9)),
            (new(-.01,0,-.9996),new(.01,0,-.9996)),
            (new(-.01,0,-1),new(.01,0,-1)),
            (new(Math.BitDecrement(.001),0,-.8),new(Math.BitIncrement(.002),0,-.8))];
        var answers=new List<bool>();
        foreach(var (a,b) in queries)
        {
            bool expected=literal.Prove(a,b)&&literal.Prove(Q(a),Q(b));
            bool actual=reused.Prove(a,b)&&reused.Prove(Q(a),Q(b));
            Assert.Equal(expected,actual);answers.Add(actual);
        }
        Assert.Contains(true,answers);Assert.Contains(false,answers);
        // A nominal chord above the floor rounds below it at F3 precision.
        Assert.False(answers[1]);
        EqualProofAndQueries(literal,reused,literalField,reusedField);
    }

    [Fact]
    public void PreCancellationComputesNoEnclosureAndRetainsNoFloorCertificate()
    {
        var source=Path([new(-10,0,-1),new(10,0,-1)]);
        var literalField=Field(source);var reusedField=Field(source);
        using var cancellation=new CancellationTokenSource();cancellation.Cancel();
        var literal=new VCarveSmoothing.ActualF3CompactionFloorProof(literalField,Grade,cancellation.Token,false);
        var reused=new VCarveSmoothing.ActualF3CompactionFloorProof(reusedField,Grade,cancellation.Token,true);
        Assert.Throws<OperationCanceledException>(()=>literal.Prove(new(-1,0,-.9),new(1,0,-.9)));
        Assert.Throws<OperationCanceledException>(()=>reused.Prove(new(-1,0,-.9),new(1,0,-.9)));
        Assert.Equal(0,literal.ComputedEndpointValues);Assert.Equal(0,reused.ComputedEndpointValues);
        Assert.Empty(Cache(literalField));Assert.Empty(Cache(reusedField));
    }

    private static VCarveSmoothing.ActualF3CompactionFloorProof Proof(VCarveSmoothing.FloorField field,bool reuse)
        =>new(field,Grade,default,reuse);
    private static VCarveSmoothing.FloorField Field(TaggedToolpath path)=>new([path],Cutter,Grade,4);
    private static TaggedToolpath Path(Point3D[] points)=>new(points,[],0,"final-carve",null,detectCurves:false);
    private static KeyValuePair<(double,double),double>[] Cache(VCarveSmoothing.FloorField field)
        =>((Dictionary<(double,double),double>)typeof(VCarveSmoothing.FloorField)
            .GetField("_cache",BindingFlags.Instance|BindingFlags.NonPublic)!.GetValue(field)!).ToArray();
    private static long Counter(VCarveSmoothing.ActualF3CompactionFloorProof proof,string name)
        =>(long)typeof(VCarveSmoothing.ActualF3CompactionFloorProof)
            .GetField(name,BindingFlags.Instance|BindingFlags.NonPublic)!.GetValue(proof)!;
    private static void EqualProofAndQueries(VCarveSmoothing.ActualF3CompactionFloorProof literal,
        VCarveSmoothing.ActualF3CompactionFloorProof reused,VCarveSmoothing.FloorField literalField,
        VCarveSmoothing.FloorField reusedField)
    {
        foreach(string name in new[]{"_queries","_pieces","_endpointRefusals","_unresolvedRefusals",
            "_arithmeticRefusals","_originalQueryBudgetRefusals"})Assert.Equal(Counter(literal,name),Counter(reused,name));
        var expected=Cache(literalField);var actual=Cache(reusedField);Assert.Equal(expected.Length,actual.Length);
        for(int i=0;i<expected.Length;i++)
        {
            Assert.Equal(BitConverter.DoubleToInt64Bits(expected[i].Key.Item1),BitConverter.DoubleToInt64Bits(actual[i].Key.Item1));
            Assert.Equal(BitConverter.DoubleToInt64Bits(expected[i].Key.Item2),BitConverter.DoubleToInt64Bits(actual[i].Key.Item2));
            Assert.Equal(BitConverter.DoubleToInt64Bits(expected[i].Value),BitConverter.DoubleToInt64Bits(actual[i].Value));
        }
        Assert.True(Counter(reused,"_enclosureOperations")<=Counter(literal,"_enclosureOperations"));
    }
}
