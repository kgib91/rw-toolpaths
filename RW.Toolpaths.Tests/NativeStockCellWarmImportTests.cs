using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeStockCellWarmImportTests
{
    private const int Limit=1_048_576;
    private static readonly VCarveCutterProfile Cutter=new([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
    private static PointD[] Box(double left,double bottom,double right,double top)
        =>[new(left,bottom),new(right,bottom),new(right,top),new(left,top)];
    private static VCarveProjectedStock Source(CancellationToken token=default)
        =>new([new(new(0,0,-1),new(0,0,-1),2,2)],[Box(-.04,-.04,.04,.04)],Box(-4,-4,4,4),Cutter,1,token);
    private static FieldInfo Field(string name)=>typeof(VCarveProjectedStock).GetField(name,BindingFlags.NonPublic|BindingFlags.Instance)!;
    private static VCarveStockCellFacts Facts(VCarveProjectedStock owner)=>(VCarveStockCellFacts)Field("stockCells").GetValue(owner)!;
    private static VCarveStockCellFacts.Key[] Keys(VCarveProjectedStock owner)
        =>Facts(owner).Entries.Select(row=>row.Key).ToArray();
    private static string[] Snapshot(VCarveProjectedStock owner)
    {
        var facts=Facts(owner);
        return Keys(owner).Select(key=>{
            var value=facts.Existing(key);
            return key.Packed.ToString("X16")+":"+string.Join(',',new[]{value.PartDistance,value.TargetDistance,value.Clearance,value.PointClearance,value.SeedClearance}
                .Select(number=>BitConverter.DoubleToInt64Bits(number).ToString("X16")));
        }).ToArray();
    }
    private static void Warm(VCarveProjectedStock source)
    {
        Assert.True(source.Prove(new(0,0,-.5),new(.3,.1,-.5)));
        Assert.True(source.Prove(new(.5,0,-.5),new(.7,.1,-.5)));
        Assert.True(source.CachedStockCellFacts>3);
    }

    [Fact]
    public void NativeWarmImportCopiesExactFifoValueBitsIncludingUnknownNanPayloads()
    {
        var source=Source();Warm(source);var dictionary=Facts(source);var key=Keys(source)[0];
        // These remain unknown (NaN), never fabricated positive stock facts.
        double Nan(int index)=>BitConverter.Int64BitsToDouble(unchecked((long)(0x7ff8000000000100UL+(ulong)index)));
        dictionary.Existing(key)=new(){PartDistance=Nan(0),TargetDistance=Nan(1),Clearance=Nan(2),PointClearance=Nan(3),SeedClearance=Nan(4)};
        var before=Snapshot(source);long queries=source.Queries,cells=source.Cells;
        var worker=source.CreateQueryWorker();Assert.Equal(0,worker.CachedStockCellFacts);
        Assert.Equal(3,worker.ImportStockCellFacts(source,3));Assert.Equal(3,worker.CachedStockCellFacts);
        Assert.Equal(before.Take(3),Snapshot(worker));Assert.Equal(before,Snapshot(source));
        Assert.Equal(queries,source.Queries);Assert.Equal(cells,source.Cells);
        // Mutation of this private value cannot affect the original cache.
        var workerFacts=Facts(worker);var workerKey=Keys(worker)[0];
        workerFacts.Existing(workerKey).SeedClearance=BitConverter.Int64BitsToDouble(unchecked((long)0x7ff8000000000fffUL));
        Assert.Equal(before,Snapshot(source));
    }

    [Fact]
    public void WarmAndColdPrivateQueriesKeepIndependentExactPhysicalCapsuleAnswers()
    {
        var source=Source();Warm(source);var before=Snapshot(source);
        var warm=source.CreateQueryWorker();warm.ImportStockCellFacts(source);
        var cold=source.CreateQueryWorker();Assert.Equal(0,cold.CachedStockCellFacts);
        foreach(double offset in new[]{-1.5,-1d,-.5,0,.5,1,1.5,2.5})
        {
            var a=new Point3D(offset,-.1,-.5);var b=new Point3D(offset,.1,-.5);
            // A constant-radius cutter swept on a straight line is a capsule.
            // Convexity makes endpoint-radius containment in the real disk
            // sufficient for its ENTIRE physical volume here, with wide reserve.
            bool analytic=Math.Max(double.Hypot(a.X,a.Y),double.Hypot(b.X,b.Y))+.1<2;
            Assert.Equal(analytic,cold.Prove(a,b));Assert.Equal(analytic,warm.Prove(a,b));
        }
        Assert.False(warm.Prove(new(5,0,-.5),new(5,.1,-.5))); // Physical Part still owns its boundary.
        Assert.Equal(before,Snapshot(source));
    }

    [Fact]
    public void PositiveSeedClearanceFromARealPriorConeKeepsTheAnalyticWholeCapsuleOracle()
    {
        // A unit-slope prior cone at tip Z=-2 removes depth 2-rho at
        // radial distance rho. The current flat-radius .1 cutter at depth .5
        // therefore fits exactly inside the radius1.5 cone cross-section.
        var prior=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,2,2)],2);
        VCarveProjectedStock ConeStock()=>new([], [Box(-.04,-.04,.04,.04)],Box(-4,-4,4,4),Cutter,1,default,
            profileSweeps:[new(prior,[new(new(0,0,-2),new(0,0,-2),0)])]);
        var source=ConeStock();Assert.Equal(1,source.ProfileChords);
        // This real chord crosses four dyadic quadrants. The first physical
        // disk proof finds the prior cone; subsequent cells can seed from it.
        Assert.True(source.Prove(new(-.05,-.1,-.5),new(.05,.1,-.5)));
        Assert.True(source.SeededCellTrials>0);
        int positiveSeeds=Facts(source).Entries.Count(row=>double.IsFinite(row.Value.SeedClearance)&&row.Value.SeedClearance>0);
        Assert.True(positiveSeeds>0,"Real prior-profile queries must populate a finite positive SeedClearance.");
        var before=Snapshot(source);long sourceQueries=source.Queries,sourceCells=source.Cells;
        var warm=source.CreateQueryWorker();Assert.Equal(source.CachedStockCellFacts,warm.ImportStockCellFacts(source));
        Assert.Equal(before,Snapshot(warm));
        var cold=source.CreateQueryWorker();var independent=ConeStock();
        foreach(double offset in new[]{-1.75,-1.25,-.5,0,.5,1.25,1.75,2.5})
        {
            var a=new Point3D(offset,-.1,-.5);var b=new Point3D(offset,.1,-.5);
            // The flat cylindrical query's deepest circular cross-section
            // dominates its entire volume in the upward-widening prior cone;
            // convexity makes both endpoint-radius tests exact for the capsule.
            // Samples stay well away from all numerical posting/inner reserves.
            bool analytic=Math.Max(double.Hypot(a.X,a.Y),double.Hypot(b.X,b.Y))+.1<1.5;
            Assert.Equal(analytic,independent.Prove(a,b));
            Assert.Equal(analytic,cold.Prove(a,b));Assert.Equal(analytic,warm.Prove(a,b));
        }
        Assert.Equal(before,Snapshot(source));
        Assert.Equal(sourceQueries,source.Queries);Assert.Equal(sourceCells,source.Cells);
    }

    [Fact]
    public void LimitsAndSameDomainCustodyRemainExplicit()
    {
        var source=Source();Warm(source);
        var zero=source.CreateQueryWorker();Assert.Equal(0,zero.ImportStockCellFacts(source,0));Assert.Equal(0,zero.CachedStockCellFacts);
        Assert.Throws<ArgumentOutOfRangeException>(()=>zero.ImportStockCellFacts(source,-1));
        Assert.Throws<ArgumentOutOfRangeException>(()=>zero.ImportStockCellFacts(source,Limit+1));
        Assert.Throws<ArgumentException>(()=>Source().ImportStockCellFacts(source));
        var complete=source.CreateQueryWorker();
        Assert.Equal(source.CachedStockCellFacts,complete.ImportStockCellFacts(source,Limit));
        Assert.Equal(Snapshot(source),Snapshot(complete));
        Assert.Throws<InvalidOperationException>(()=>complete.ImportStockCellFacts(source,1));
    }

    [Fact]
    public void CancellationPreventsPublishingACellInventory()
    {
        using var cancellation=new CancellationTokenSource();var source=Source(cancellation.Token);Warm(source);
        var worker=source.CreateQueryWorker();cancellation.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(()=>worker.ImportStockCellFacts(source));
        Assert.Equal(0,worker.CachedStockCellFacts);
    }
}
