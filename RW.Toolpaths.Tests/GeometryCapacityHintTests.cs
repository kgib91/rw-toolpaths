using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class GeometryCapacityHintTests(ITestOutputHelper output)
{
    private static readonly FieldInfo RoundHints=Field("_roundCapacityHints"),RawPath=Field("pathOut"),RoundResizes=Field("_roundCapacityHintResizes");
    private static readonly object Enabled=true,Disabled=false;
    private static FieldInfo Field(string name)=>typeof(ClipperOffset).GetField(name,BindingFlags.Instance|BindingFlags.NonPublic)!;

    [Fact]
    public void ConvexSingleRowClosingVertexNoLongerDoublesItsReturnedBacking()
    {
        var circle=Circle(128);using var literal=new VCarveOrderedDiscHull(literalHullCapacity:true);
        using var hinted=new VCarveOrderedDiscHull();
        Path64 Build(VCarveOrderedDiscHull owner){owner.BeginCircle(circle);owner.AddDisc(new(0,0,0),10);return owner.FinishHull();}
        var expected=Build(literal);var actual=Build(hinted);
        Assert.Equal(expected.ToArray(),actual.ToArray());Assert.Equal(128,actual.Count);
        Assert.Equal(256,expected.Capacity);Assert.Equal(129,actual.Capacity);
        Assert.Equal(1,literal.HullCapacityResizes);Assert.Equal(0,hinted.HullCapacityResizes);
        long Measure(VCarveOrderedDiscHull owner)
        {
            long before=GC.GetAllocatedBytesForCurrentThread();
            for(int i=0;i<64;i++)_ = Build(owner);
            return GC.GetAllocatedBytesForCurrentThread()-before;
        }
        _ = Measure(literal);_ = Measure(hinted);long oldBytes=Measure(literal),newBytes=Measure(hinted);
        output.WriteLine($"single-row-hulls=64 literalAllocatedBytes={oldBytes} hintedAllocatedBytes={newBytes}");
        Assert.True(newBytes<oldBytes);
    }

    [Theory]
    [InlineData(8)] [InlineData(128)] [InlineData(1024)]
    public void CapacityOnlyHullHintKeepsEveryOrderedVertexAcrossPairsAndExtremeTies(int count)
    {
        var circle=Circle(count);using var literal=new VCarveOrderedDiscHull(literalHullCapacity:true);
        using var hinted=new VCarveOrderedDiscHull();var random=new Random(77123+count);
        var rows=new List<(Point3D,double)>{(new(0,0,0),1e-12),(new(1e15,-1e15,0),2),
            (new(.5/PathUtils.Scale,-.5/PathUtils.Scale,0),1e-7)};
        for(int i=0;i<32;i++)rows.Add((new(random.NextDouble()*10,random.NextDouble()*10,0),random.NextDouble()*2));
        for(int i=0;i<rows.Count;i++)
        {
            foreach(var owner in new[]{literal,hinted})
            {owner.BeginCircle(circle);owner.AddDisc(rows[i].Item1,rows[i].Item2);if(i%2==0)owner.AddDisc(rows[(i+1)%rows.Count].Item1,rows[(i+1)%rows.Count].Item2);}
            Assert.Equal(literal.FinishHull().ToArray(),hinted.FinishHull().ToArray());
        }
        Assert.Equal(literal.Vertices,hinted.Vertices);Assert.Equal(literal.DiscRowCacheHits,hinted.DiscRowCacheHits);
        Assert.Equal(literal.SortFallbacks,hinted.SortFallbacks);
    }

    [Theory]
    [InlineData(false,4d)] [InlineData(true,4d)]
    [InlineData(false,32d)] [InlineData(true,32d)]
    public void RoundReservationKeepsRawAndCleanedVerticesAndExactDeltaCallbackOrder(bool callback,double arc)
    {
        Paths64 source=[new(){new(-500,-100),new(-100,-100),new(100,300),new(500,-100)},
            new(){new(0,0),new(400,0),new(400,400),new(0,400),new(0,0)}];
        (Paths64 Cleaned,Path64 Raw,(int,int)[] Calls,int Resizes) Execute(bool enabled)
        {
            var owner=new ClipperOffset(2,arc);RoundHints.SetValue(owner,enabled?Enabled:Disabled);
            var calls=new List<(int,int)>();
            if(callback)owner.DeltaCallback=(path,normals,j,k)=>{calls.Add((j,k));return 200+(j%3)*.5;};
            owner.AddPaths(source,JoinType.Round,EndType.Round);var result=new Paths64();owner.Execute(200,result);
            return(result,new((Path64)RawPath.GetValue(owner)!),calls.ToArray(),(int)RoundResizes.GetValue(owner)!);
        }
        var literal=Execute(false);var hinted=Execute(true);
        EqualPaths(literal.Cleaned,hinted.Cleaned);Assert.Equal(literal.Raw.ToArray(),hinted.Raw.ToArray());
        Assert.Equal(literal.Calls,hinted.Calls);Assert.Equal(0,literal.Resizes);Assert.True(hinted.Resizes>0);
        Assert.Equal(new Point64(-500,-100),source[0][0]);
    }

    [Fact]
    public void LongRoundCapsAllocateLessWithTheSameCompleteManagedOffset()
    {
        Path64 source=[new(0,0),new(20000,10000)];
        Paths64 Execute(bool enabled)
        {
            var owner=new ClipperOffset(2,4);RoundHints.SetValue(owner,enabled?Enabled:Disabled);
            owner.AddPath(source,JoinType.Round,EndType.Round);var result=new Paths64();owner.Execute(10000,result);return result;
        }
        EqualPaths(Execute(false),Execute(true));
        long Measure(bool enabled)
        {
            long before=GC.GetAllocatedBytesForCurrentThread();
            for(int i=0;i<64;i++)_ = Execute(enabled);
            return GC.GetAllocatedBytesForCurrentThread()-before;
        }
        _ = Measure(false);_ = Measure(true);long literal=Measure(false),hinted=Measure(true);
        output.WriteLine($"managed-round-offsets=64 literalAllocatedBytes={literal} hintedAllocatedBytes={hinted}");
        Assert.True(hinted<literal);
    }

    [Fact]
    public void OversizedOverflowAndNonpositiveHintsDoNotAllocateOrChangeTheRawPath()
    {
        var owner=new ClipperOffset();var path=new Path64(4){new(7,8)};RawPath.SetValue(owner,path);
        var reserve=typeof(ClipperOffset).GetMethod("ReserveRoundCapacity",BindingFlags.Instance|BindingFlags.NonPublic)!;
        foreach(int remaining in new[]{int.MaxValue,1_000_000,0,-1})reserve.Invoke(owner,[remaining]);
        Assert.Equal(4,path.Capacity);Assert.Equal(new[]{new Point64(7,8)},path.ToArray());
        Assert.Equal(0,(int)RoundResizes.GetValue(owner)!);
    }

    [Fact]
    public void DeltaCallbackFailureStillPrecedesAnyRoundHintOrPublication()
    {
        foreach(bool enabled in new[]{false,true})
        {
            var owner=new ClipperOffset();RoundHints.SetValue(owner,enabled?Enabled:Disabled);int calls=0;
            owner.AddPath([new(0,0),new(100,0)],JoinType.Round,EndType.Round);
            owner.DeltaCallback=(path,normals,j,k)=>{calls++;throw new FormatException("original callback failure");};
            Assert.Equal("original callback failure",Assert.Throws<FormatException>(()=>owner.Execute(20,new Paths64())).Message);
            Assert.Equal(1,calls);Assert.Equal(0,(int)RoundResizes.GetValue(owner)!);
        }
    }

    private static (double X,double Y)[] Circle(int count)=>Enumerable.Range(0,count)
        .Select(i=>(Math.Cos(2*Math.PI*i/count),Math.Sin(2*Math.PI*i/count))).ToArray();
    private static void EqualPaths(Paths64 expected,Paths64 actual)
    {Assert.Equal(expected.Count,actual.Count);for(int i=0;i<expected.Count;i++)Assert.Equal(expected[i].ToArray(),actual[i].ToArray());}
}
