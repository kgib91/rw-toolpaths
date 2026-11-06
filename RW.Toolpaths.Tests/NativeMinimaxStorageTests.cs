using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeMinimaxStorageTests
{
    [Fact]
    public void CompactRowsRetainTheOriginalWitnessIncludingTiesAndFailedPaths()
    {
        var random=new Random(84022);
        for(int trial=0;trial<120;trial++)
        {
            var move=ToolMove.CubicBezier(new(0,0,-1),
                new(1,random.NextDouble()*2-1,-random.NextDouble()),
                new(3,random.NextDouble()*2-1,-random.NextDouble()),new(4,0,-.5),0);
            int count=2+(trial%4)*13;
            var points=Enumerable.Range(0,count).Select(i=>{
                double t=i/(double)(count-1);
                if(i>0&&i<count-1&&trial%3==0)t=Math.Round(t*4)/4;
                var point=VCarveNativeCurveMath.At(move,t);
                return point with{Y=point.Y+(i==count/2?(trial%5)*.05:0)};
            }).ToArray();
            if(trial%7==0)Array.Reverse(points,1,points.Length-2);
            var padded=new Point3D[]{new(-10,-10,-10)}.Concat(points).Append(new(10,10,10)).ToArray();
            foreach(double allowance in new[]{0d,.03,.3,10d})
                Assert.Equal(Original(move,padded,1,count,allowance),
                    VCarveNativeGeometricProof.MinimaxOrderedParameters(move,padded,1,count,allowance));
        }
    }

    [Fact]
    public void SuccessfulFullSearchUsesLessThanHalfTheOriginalManagedScratch()
    {
        var move=ToolMove.CubicBezier(new(0,0,-1),new(1,.5,-1),new(3,.5,-.5),new(4,0,-.5),0);
        var source=Enumerable.Range(0,129).Select(i=>VCarveNativeCurveMath.At(move,i/128d)).ToArray();
        Original(move,source,0,128,10);
        VCarveNativeGeometricProof.MinimaxOrderedParameters(move,source,0,128,10);
        long before=GC.GetAllocatedBytesForCurrentThread();
        var expected=Original(move,source,0,128,10);
        long originalBytes=GC.GetAllocatedBytesForCurrentThread()-before;
        before=GC.GetAllocatedBytesForCurrentThread();
        var actual=VCarveNativeGeometricProof.MinimaxOrderedParameters(move,source,0,128,10);
        long actualBytes=GC.GetAllocatedBytesForCurrentThread()-before;
        Assert.NotNull(expected);Assert.Equal(expected,actual);
        Assert.True(actualBytes<originalBytes/2,$"originalBytes={originalBytes} compactBytes={actualBytes}");
    }

    private static long[] Bits(double[] values)=>values.Select(BitConverter.DoubleToInt64Bits).ToArray();
    private static void Same(double[]? expected,double[]? actual)
    {
        if(expected is null){Assert.Null(actual);return;}
        Assert.NotNull(actual);Assert.Equal(Bits(expected),Bits(actual!));
    }

    [Fact]
    public void LazySlabsRetainEagerParameterBitsAcrossSlabEdgesAndExceptionalArithmetic()
    {
        var move=ToolMove.CubicBezier(new(0,0,-1),new(1,.5,-1),new(3,-.5,-.5),new(4,0,-.5),0);
        foreach(int count in new[]{1,2,16,17,18,32,33,34,165,166,337})
        {
            var source=Enumerable.Range(0,count).Select(i=>VCarveNativeCurveMath.At(move,count==1?0:i/(double)(count-1))).ToArray();
            if(count>3)source[count/2]=source[count/2-1];
            foreach(double allowance in new[]{-1d,-0d,0d,.01,10d,double.NaN,double.PositiveInfinity,double.NegativeInfinity})
                Same(VCarveNativeGeometricProof.MinimaxOrderedParametersEager(move,source,0,count-1,allowance),
                    VCarveNativeGeometricProof.MinimaxOrderedParameters(move,source,0,count-1,allowance));
        }
        foreach(double abnormal in new[]{double.NaN,double.PositiveInfinity,double.NegativeInfinity,-0d})
        {
            Point3D[] source=[move.Start,new(1,abnormal,-.5),new(1,abnormal,-.5),move.End];
            foreach(double allowance in new[]{.1,double.NaN,double.PositiveInfinity})
                Same(VCarveNativeGeometricProof.MinimaxOrderedParametersEager(move,source,0,3,allowance),
                    VCarveNativeGeometricProof.MinimaxOrderedParameters(move,source,0,3,allowance));
        }
    }

    [Theory]
    [InlineData(1,0,0)]
    [InlineData(17,1,16)]
    [InlineData(33,2,32)]
    public void FirstRefusedRowDoesNotAllocateTheNextSlabOrAnyFutureRows(int failedRow,int slabs,int retained)
    {
        var move=ToolMove.Line(new(0,0,0),new(1,0,0));
        var source=Enumerable.Repeat(move.Start,1025).ToArray();source[failedRow]=new(0,100,0);
        VCarveNativeGeometricProof.WorkSnapshot stats;
        using(var capture=VCarveNativeGeometricProof.BeginWorkCapture())
        {
            Assert.Null(VCarveNativeGeometricProof.MinimaxOrderedParameters(move,source,0,1024,.01));
            stats=capture.Snapshot();
        }
        Assert.Equal(1024,stats.MinimaxRowsRequested);Assert.Equal(failedRow,stats.MinimaxRowsVisited);
        Assert.Equal(slabs,stats.MinimaxParentSlabs);Assert.Equal(retained,stats.MinimaxParentRowsRetained);
        Assert.Equal((long)slabs*16*257,stats.MinimaxParentCells);
        Assert.Equal(1025L*257,stats.MinimaxEagerParentCellsRequested);
        Assert.Equal(1,stats.MinimaxParentLohAllocationsAvoided);
        Same(VCarveNativeGeometricProof.MinimaxOrderedParametersEager(move,source,0,1024,.01),
            VCarveNativeGeometricProof.MinimaxOrderedParameters(move,source,0,1024,.01));
    }

    [Fact]
    public void EarlyRefusalAllocatesLessThanOneTenthTheEagerMatrixWithoutChangingReadOrder()
    {
        var move=ToolMove.Line(new(0,0,0),new(1,0,0));
        var source=Enumerable.Repeat(move.Start,2049).ToArray();source[1]=new(0,100,0);
        VCarveNativeGeometricProof.MinimaxOrderedParametersEager(move,source,0,2048,.01);
        VCarveNativeGeometricProof.MinimaxOrderedParameters(move,source,0,2048,.01);
        long before=GC.GetAllocatedBytesForCurrentThread();
        Assert.Null(VCarveNativeGeometricProof.MinimaxOrderedParametersEager(move,source,0,2048,.01));
        long eager=GC.GetAllocatedBytesForCurrentThread()-before;
        before=GC.GetAllocatedBytesForCurrentThread();
        Assert.Null(VCarveNativeGeometricProof.MinimaxOrderedParameters(move,source,0,2048,.01));
        long lazy=GC.GetAllocatedBytesForCurrentThread()-before;
        Assert.True(lazy<eager/10,$"eagerBytes={eager} lazyBytes={lazy}");
        var expected=new RecordingSource(source);var actual=new RecordingSource(source);
        Same(VCarveNativeGeometricProof.MinimaxOrderedParametersEager(move,expected,0,2048,.01),
            VCarveNativeGeometricProof.MinimaxOrderedParameters(move,actual,0,2048,.01));
        Assert.Equal(expected.Reads,actual.Reads);Assert.Equal(new[]{0,1},actual.Reads);
    }

    [Fact]
    public void CompleteCertificatesAndFailureReasonsMatchWithExplicitEagerStorageControl()
    {
        var move=ToolMove.CubicBezier(new(0,0,0),new(0,1,0),new(0,2,0),new(10,3,10),0);
        var source=Enumerable.Range(0,101).Select(i=>VCarveNativeCurveMath.At(move,i/100d)).ToArray();
        source[1]=source[1] with{Z=source[1].Z+.29};
        long eagerCalls=0,slabCalls=0;
        foreach(bool cheap in new[]{false,true})foreach(double allowance in new[]{.001,.1,.28,.3})
        {
            var oldOverride=VCarveNativeGeometricProof.EagerMinimaxParentsOverride.Value;
            try
            {
                VCarveNativeGeometricProof.EagerMinimaxParentsOverride.Value=true;
                using var eagerCapture=VCarveNativeGeometricProof.BeginWorkCapture();
                var expected=cheap?VCarveNativeGeometricProof.ProveCheapFirst(move,source,0,100,allowance)
                    :VCarveNativeGeometricProof.ProveOriginalOrder(move,source,0,100,allowance);
                eagerCalls+=eagerCapture.Snapshot().MinimaxCalls;
                eagerCapture.Dispose();
                VCarveNativeGeometricProof.EagerMinimaxParentsOverride.Value=false;
                using var slabCapture=VCarveNativeGeometricProof.BeginWorkCapture();
                var actual=cheap?VCarveNativeGeometricProof.ProveCheapFirst(move,source,0,100,allowance)
                    :VCarveNativeGeometricProof.ProveOriginalOrder(move,source,0,100,allowance);
                slabCalls+=slabCapture.Snapshot().MinimaxCalls;
                Assert.Equal(expected.Certified,actual.Certified);Assert.Equal(expected.Reason,actual.Reason);
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected.ErrorBound),BitConverter.DoubleToInt64Bits(actual.ErrorBound));
                Assert.Equal(Bits(expected.Parameters),Bits(actual.Parameters));
            }
            finally{VCarveNativeGeometricProof.EagerMinimaxParentsOverride.Value=oldOverride;}
        }
        Assert.True(eagerCalls>0,$"eagerMinimaxCalls={eagerCalls}");
        Assert.Equal(eagerCalls,slabCalls);
    }

    private sealed class RecordingSource(Point3D[] points):IReadOnlyList<Point3D>
    {
        internal readonly List<int> Reads=[];
        public int Count=>points.Length;
        public Point3D this[int index]{get{Reads.Add(index);return points[index];}}
        public IEnumerator<Point3D> GetEnumerator()=>((IEnumerable<Point3D>)points).GetEnumerator();
        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()=>GetEnumerator();
    }

    // Retained original minimax recurrence, row allocation and tie-breaking.
    private static double[]? Original(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,double allowance)
    {
        const int divisions=256;int count=last-first+1;
        var positions=Enumerable.Range(0,divisions+1).Select(i=>VCarveNativeCurveMath.At(move,i/(double)divisions)).ToArray();
        var previous=Enumerable.Repeat(double.PositiveInfinity,divisions+1).ToArray();
        previous[0]=VCarveNativeCurveMath.Distance(move.Start,source[first]);
        var parents=new int[count,divisions+1];
        for(int ordinal=1;ordinal<count;ordinal++)
        {
            var current=new double[divisions+1];double best=double.PositiveInfinity;int predecessor=0;
            for(int j=0;j<=divisions;j++)
            {
                if(previous[j]<best){best=previous[j];predecessor=j;}
                current[j]=Math.Max(best,VCarveNativeCurveMath.Distance(positions[j],source[first+ordinal]));
                parents[ordinal,j]=predecessor;
            }
            previous=current;
        }
        if(previous[divisions]>allowance)return null;
        var result=new double[count];int cursor=divisions;
        for(int ordinal=count-1;ordinal>0;ordinal--){result[ordinal]=cursor/(double)divisions;cursor=parents[ordinal,cursor];}
        return result;
    }
}
