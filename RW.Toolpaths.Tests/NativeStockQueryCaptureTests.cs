using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class NativeStockQueryCaptureTests
{
    [Fact]
    public void CaptureRetainsActualIndexInputsAndCompletedParentResultsOnly()
    {
        var old=VCarveNativeStockTrace.QueryObserverFactory.Value;
        var rows=new List<VCarveNativeStockQuery>();VCarveNativeStockIndexEvidence? evidence=null;
        try
        {
            VCarveNativeStockTrace.QueryObserverFactory.Value=value=>{evidence=value;return rows.Add;};
            var profile=Profile();
            var columns=new[]{new VCarveRoughingChord(new(-3,0,-2),new(3,0,-2),1,1,1)};
            var sweeps=new[]{new VCarveClearedSweep(profile,
                [new(new(-2,1,-2),new(2,1,-1),0,0,0)])};
            var stock=new VCarveProjectedStock(columns,Square(8),Square(12)[0],profile,4,CancellationToken.None,sweeps);
            Assert.NotNull(evidence);Assert.Same(profile,evidence!.Profile);
            Assert.Same(profile,evidence.ProfileSweeps[0].Profile);
            Assert.Equal(columns,evidence.Columns);Assert.Equal(stock.IndexedChords,evidence.IndexedChords);
            var queries=new[]{(0d,0d,.1,true),(0d,1d,0d,false),(1d,1d,.03,true),(20d,20d,0d,true)};
            foreach(var q in queries)
            {
                double depth=stock.ClearedDepth(q.Item1,q.Item2,q.Item3,q.Item4);
                var row=rows[^1];Assert.Equal(q.Item1,row.X);Assert.Equal(q.Item2,row.Y);
                Assert.Equal(q.Item3,row.Radius);Assert.Equal(q.Item4,row.InnerReserve);
                Assert.Equal(BitConverter.DoubleToInt64Bits(depth),BitConverter.DoubleToInt64Bits(row.ExpectedDepth));
            }
            Assert.Equal(queries.Length,rows.Count);
            var worker=stock.CreateQueryWorker();worker.ClearedDepth(0,1,0);
            Assert.Equal(queries.Length,rows.Count);
        }
        finally{VCarveNativeStockTrace.QueryObserverFactory.Value=old;}
    }

    [Fact]
    public void DiagnosticSinkDoesNotChangeDepthBitsOrTraversalCounts()
    {
        var old=VCarveNativeStockTrace.QueryObserverFactory.Value;
        try
        {
            var profile=Profile();var rows=new List<VCarveNativeStockQuery>();
            var sweeps=new[]{new VCarveClearedSweep(profile,
                [new(new(-3,0,-3),new(3,0,-1),0,0,0),new(new(-3,1,-1),new(3,1,-3),0,0,0)])};
            VCarveNativeStockTrace.QueryObserverFactory.Value=null;
            var original=new VCarveProjectedStock([],Square(8),Square(12)[0],profile,4,CancellationToken.None,sweeps);
            VCarveNativeStockTrace.QueryObserverFactory.Value=_=>rows.Add;
            var captured=new VCarveProjectedStock([],Square(8),Square(12)[0],profile,4,CancellationToken.None,sweeps);
            for(int i=0;i<80;i++)
            {
                double x=-3+i*.075,y=(i%3)*.35,radius=(i%4)*.02;bool reserve=i%2==0;
                Assert.Equal(BitConverter.DoubleToInt64Bits(original.ClearedDepth(x,y,radius,reserve)),
                    BitConverter.DoubleToInt64Bits(captured.ClearedDepth(x,y,radius,reserve)));
            }
            Assert.Equal(80,rows.Count);Assert.Equal(original.ColumnQueries,captured.ColumnQueries);
            Assert.Equal(original.ColumnTests,captured.ColumnTests);Assert.Equal(original.ColumnNodeVisits,captured.ColumnNodeVisits);
            Assert.Equal(original.PriorPoseSeedTrials,captured.PriorPoseSeedTrials);
            Assert.Equal(original.SpatialDepthTieReorders,captured.SpatialDepthTieReorders);
            Assert.All(rows,row=>Assert.True(double.IsFinite(row.ExpectedDepth)));
        }
        finally{VCarveNativeStockTrace.QueryObserverFactory.Value=old;}
    }

    [Fact]
    public void EmptyIndexStillCapturesTheCompletedZeroQuery()
    {
        var old=VCarveNativeStockTrace.QueryObserverFactory.Value;var rows=new List<VCarveNativeStockQuery>();
        try
        {
            VCarveNativeStockTrace.QueryObserverFactory.Value=_=>rows.Add;
            var stock=new VCarveProjectedStock([],Square(8),Square(12)[0],Profile(),4,CancellationToken.None);
            Assert.Equal(0,stock.ClearedDepth(0,0,.2));
            var row=Assert.Single(rows);Assert.Equal(-1,row.SeedBefore);Assert.Equal(-1,row.SeedAfter);
            Assert.Equal(0,row.ExpectedDepth);Assert.Equal(.2,row.Radius);
        }
        finally{VCarveNativeStockTrace.QueryObserverFactory.Value=old;}
    }

    private static VCarveCutterProfile Profile()=>new([new(CarvingCutterSectionKind.Linear,0,2,4)],4);
    private static PointD[][] Square(double radius)=>[[new(-radius,-radius),new(radius,-radius),new(radius,radius),new(-radius,radius)]];
}
