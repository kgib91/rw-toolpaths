using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class BoundarySegmentAllocationTests
{
    [Fact]
    public void DirectStorageKeepsEveryLiteralClosingEdgeAndOriginalOrdinal()
    {
        var boundaries=new Paths64
        {
            new(),new(){new(7,-2)},new(){new(-3,4),new(8,9)},
            new(){new(0,0),new(100,0),new(100,100),new(100,100),new(0,100),new(0,0)}
        };
        var direct=new PolygonOps.BoundarySegmentIndex(boundaries);
        var literal=new PolygonOps.BoundarySegmentIndex(boundaries,useLiteralListConstruction:true);
        var expected=new List<(Point64 Start,Point64 End)>();
        foreach(var boundary in boundaries)
            for(int end=0,start=boundary.Count-1;end<boundary.Count;start=end++)
                expected.Add((boundary[start],boundary[end]));
        Assert.Equal(expected.Count,direct.SegmentCount);
        EqualSegments(expected,direct);EqualSegments(expected,literal);
        Assert.Equal(literal.PartitionCount,direct.PartitionCount);
        Assert.Equal(literal.PartitionSortFallbacks,direct.PartitionSortFallbacks);
        // Captured points are values, never aliases of caller-owned Path64.
        boundaries[1][0]=new(999,999);boundaries.Clear();
        EqualSegments(expected,direct);
    }

    [Fact]
    public void QueryTraversalDistanceTiesAndLegacyWitnessesMatchTheLiteralStorageControl()
    {
        var random=new Random(871122);
        for(int trial=0;trial<30;trial++)
        {
            var input=new Paths64();
            for(int ring=0;ring<6;ring++)
            {
                int x=random.Next(-500,500),y=random.Next(-500,500),w=random.Next(1,300),h=random.Next(1,300);
                var path=new Path64{new(x,y),new(x+w,y),new(x+w,y+h),new(x,y+h)};
                if((trial+ring)%2==0)path.Reverse();
                if(ring%3==0)path.Add(path[0]);
                input.Add(path);
            }
            if(trial%2==0)input.Reverse();
            var direct=new PolygonOps.BoundarySegmentIndex(input);
            var literal=new PolygonOps.BoundarySegmentIndex(input,useLiteralListConstruction:true);
            for(int query=0;query<12;query++)
            {
                var a=new Point64(random.Next(-700,700),random.Next(-700,700));
                var b=new Point64(random.Next(-700,700),random.Next(-700,700));
                var da=new List<int>();var la=new List<int>();
                Assert.Equal(literal.QueryBounds(-1000,-1000,1000,1000,la,1000,CancellationToken.None),
                    direct.QueryBounds(-1000,-1000,1000,1000,da,1000,CancellationToken.None));
                Assert.Equal(la,da);
                Assert.Equal(BitConverter.DoubleToInt64Bits(literal.DistanceSquared(a)),BitConverter.DoubleToInt64Bits(direct.DistanceSquared(a)));
                Assert.Equal(literal.IsInside(a),direct.IsInside(a));
                Assert.Equal(literal.NearestSegment(a),direct.NearestSegment(a));
                Assert.Equal(literal.NearestSegmentForCertificate(a),direct.NearestSegmentForCertificate(a));
                Assert.Equal(literal.ClearanceViolation(a,b,17),direct.ClearanceViolation(a,b,17));
            }
            Assert.True(direct.LegacyWitnessIndexBuilt);
            Assert.True(literal.LegacyWitnessIndexBuilt);
        }
    }

    [Fact]
    public void DenseConstructionAvoidsTemporaryListGrowthAllocationWithoutChangingTheTree()
    {
        const int count=8192;
        var ring=new Path64(count);
        for(int i=0;i<count;i++)
        {
            double angle=2*Math.PI*i/count;
            ring.Add(new Point64(Math.Round(1_000_000*Math.Cos(angle)),Math.Round(1_000_000*Math.Sin(angle))));
        }
        var input=new Paths64{ring};
        _=new PolygonOps.BoundarySegmentIndex(input);
        _=new PolygonOps.BoundarySegmentIndex(input,true);
        long start=GC.GetAllocatedBytesForCurrentThread();
        var direct=new PolygonOps.BoundarySegmentIndex(input);
        long directBytes=GC.GetAllocatedBytesForCurrentThread()-start;
        start=GC.GetAllocatedBytesForCurrentThread();
        var literal=new PolygonOps.BoundarySegmentIndex(input,true);
        long literalBytes=GC.GetAllocatedBytesForCurrentThread()-start;
        Assert.Equal(literal.SegmentCount,direct.SegmentCount);
        Assert.Equal(literal.PartitionCount,direct.PartitionCount);
        Assert.Equal(literal.PartitionSortFallbacks,direct.PartitionSortFallbacks);
        Assert.True(literalBytes-directBytes>=count*16L,$"Direct={directBytes}; literal={literalBytes}; segments={count}");
        GC.KeepAlive(direct);GC.KeepAlive(literal);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(-2)]
    [InlineData(-50)]
    public void GenericOffsetReusesOnlyItsFinalOriginalGroupAndMatchesIndependentOwners(double delta)
    {
        IReadOnlyList<PointD>[] source=
        [
            new PointD[]{new(-20,-20),new(20,-20),new(20,20),new(-20,20)},
            new PointD[]{new(-4,-4),new(-4,4),new(4,4),new(4,-4)},
            // Positive 2mm dilation bridges this 3mm gap. The old x=22 front
            // intersects a tessellated arc around (23,10), producing an actual
            // Boolean intersection vertex inside that source corner's 2mm disc.
            // Unlike an isolated round join, this exercises vertex validation.
            new PointD[]{new(23,-10),new(33,-10),new(33,10),new(23,10)}
        ];
        var snapshot=source.Select(p=>p.ToArray()).ToArray();
        int enumerations=0;
        IEnumerable<IReadOnlyList<PointD>> OneShot()
        {
            Assert.Equal(0,enumerations++);
            foreach(var ring in source)yield return ring;
        }
        var actual=PolygonOps.OffsetWithBoundaryIndexControl(OneShot(),delta,.25,0,JoinType.Round,false,true,out var used);
        var expected=PolygonOps.OffsetWithBoundaryIndexControl(source,delta,.25,0,JoinType.Round,false,false,out var control);
        Same(expected,actual);
        Assert.Equal(1,enumerations);
        Assert.Equal(control.Requests,used.Requests);
        Assert.Equal(control.SourceSegments,used.SourceSegments);
        Assert.Equal(control.Requests,control.Builds);Assert.Equal(0,control.Reuses);
        Assert.Equal(used.Requests==0?0:1,used.Builds);
        Assert.Equal(used.Requests-used.Builds,used.Reuses);
        if(delta>0)
        {
            Assert.True(used.Requests>=2,$"Expected joined offset-arc intersection fallback, got {used}.");
            Assert.True(used.FinalArraySlots<control.FinalArraySlots);
        }
        for(int i=0;i<source.Length;i++)Assert.Equal(snapshot[i],source[i]);
    }

    [Fact]
    public void EmptyOffsetAndCancelledIndexQueryRetainTheirOriginalContracts()
    {
        Assert.Empty(PolygonOps.OffsetWithBoundaryIndexControl([],2,.01,0,JoinType.Round,false,true,out var empty));
        Assert.Equal(default,empty);
        var input=new Paths64();
        for(int i=0;i<2048;i++)input.Add(new(){new(i*7,0),new(i*7+3,0),new(i*7+3,3),new(i*7,3)});
        var index=new PolygonOps.BoundarySegmentIndex(input);
        using var canceled=new CancellationTokenSource();canceled.Cancel();
        Assert.Throws<OperationCanceledException>(()=>index.QueryBounds(long.MinValue,long.MinValue,long.MaxValue,long.MaxValue,
            new(),int.MaxValue,canceled.Token));
    }

    private static void EqualSegments(IReadOnlyList<(Point64 Start,Point64 End)> expected,PolygonOps.BoundarySegmentIndex index)
    {
        var storage=(Array)typeof(PolygonOps.BoundarySegmentIndex).GetField("_segments",BindingFlags.Instance|BindingFlags.NonPublic)!.GetValue(index)!;
        Assert.Equal(expected.Count,storage.Length);
        foreach(object segment in storage)
        {
            Type type=segment.GetType();int ordinal=(int)type.GetProperty("Ordinal")!.GetValue(segment)!;
            Assert.Equal(expected[ordinal].Start,(Point64)type.GetProperty("Start")!.GetValue(segment)!);
            Assert.Equal(expected[ordinal].End,(Point64)type.GetProperty("End")!.GetValue(segment)!);
        }
    }

    private static void Same(IReadOnlyList<List<PointD>> expected,IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<expected.Count;i++)
        {
            Assert.Equal(expected[i].Count,actual[i].Count);
            for(int j=0;j<expected[i].Count;j++)
            {
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected[i][j].x),BitConverter.DoubleToInt64Bits(actual[i][j].x));
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected[i][j].y),BitConverter.DoubleToInt64Bits(actual[i][j].y));
            }
        }
    }
}
