using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Facts=RW.Toolpaths.Milling.VCarveStockCellFacts.Facts;
using Key=RW.Toolpaths.Milling.VCarveStockCellFacts.Key;

namespace RW.Toolpaths.Tests;

public sealed class StockCellFactStoreTests
{
    // Independent literal original Dictionary+Queue admission and ref-update
    // implementation. Queries do not move a retained entry in this oracle.
    private sealed class Original(int limit)
    {
        private readonly Dictionary<ulong,Facts> values=new();
        private readonly Queue<ulong> order=new();
        internal long Hits,Misses,Evictions;
        internal IEnumerable<(ulong Key,Facts Value)> Entries=>order.Select(key=>(key,values[key]));
        internal ref Facts Get(ulong key,out bool found)
        {
            ref var prior=ref CollectionsMarshal.GetValueRefOrNullRef(values,key);
            if(!Unsafe.IsNullRef(ref prior)){found=true;Hits++;return ref prior;}
            found=false;Misses++;
            if(values.Count==limit){values.Remove(order.Dequeue());Evictions++;}
            ref var added=ref CollectionsMarshal.GetValueRefOrAddDefault(values,key,out _);
            added=Facts.Unknown();order.Enqueue(key);return ref added;
        }
    }

    private static long[] Bits(Facts value)=>[BitConverter.DoubleToInt64Bits(value.PartDistance),
        BitConverter.DoubleToInt64Bits(value.TargetDistance),BitConverter.DoubleToInt64Bits(value.Clearance),
        BitConverter.DoubleToInt64Bits(value.PointClearance),BitConverter.DoubleToInt64Bits(value.SeedClearance)];
    private static Facts Value(int index)
    {
        double Nan(int n)=>BitConverter.Int64BitsToDouble(unchecked((long)(0x7ff8000000000100UL+(uint)n)));
        return new(){PartDistance=-index*.125,TargetDistance=Nan(index%31),Clearance=index*.25,
            PointClearance=BitConverter.Int64BitsToDouble(long.MinValue),SeedClearance=Nan(index%17)};
    }

    [Theory]
    [InlineData(1,0)] [InlineData(1,4096)] [InlineData(3,0)] [InlineData(3,4096)]
    [InlineData(4097,0)] [InlineData(4097,4096)]
    public void LiteralAdmissionRefUpdatesFifoAndEveryValueBitMatchOriginal(int limit,int fast)
    {
        var original=new Original(limit);var candidate=new VCarveStockCellFacts(limit,fast);
        int operations=limit*3+300;
        for(int i=0;i<operations;i++)
        {
            ulong packed=(ulong)(i%5==0?Math.Max(0,i-1):i%7==0?0:i);
            ref var old=ref original.Get(packed,out bool found);
            ref var actual=ref candidate.GetOrAdd(new(packed),out var lookup);
            Assert.Equal(found,lookup!=VCarveStockCellFacts.Lookup.New);Assert.Equal(Bits(old),Bits(actual));
            if(i%3!=0){old=Value(i);actual=Value(i);}
            else{old.SeedClearance=i*.125;actual.SeedClearance=i*.125;}
            if(i%127==0||i==operations-1)
            {
                Assert.Equal(original.Entries.Select(row=>row.Key),candidate.Entries.Select(row=>row.Key.Packed));
                Assert.Equal(original.Entries.Select(row=>string.Join(',',Bits(row.Value))),candidate.Entries.Select(row=>string.Join(',',Bits(row.Value))));
            }
        }
        Assert.Equal(original.Hits,candidate.FastHits+candidate.SafeDictionaryHits);
        Assert.Equal(original.Misses,candidate.Misses);Assert.Equal(original.Evictions,candidate.Evictions);
        Assert.InRange(candidate.Count,1,limit);Assert.InRange(candidate.AllocatedSlots,1,limit);
        if(fast==4096)Assert.True(candidate.FastHits>0);
        else Assert.Equal(0,candidate.FastHits);
    }

    [Fact]
    public void StaleDirectSlotAndHashCollisionsNeverAliasAnOccupiedFact()
    {
        var a=new Key(0);int lane=a.GetHashCode()&4095;
        var same=Enumerable.Range(1,200000).Select(i=>new Key((ulong)i)).First(k=>(k.GetHashCode()&4095)==lane);
        var different=Enumerable.Range(1,20000).Select(i=>new Key((ulong)i)).First(k=>(k.GetHashCode()&4095)!=lane);
        var collisions=new VCarveStockCellFacts(3,4096);
        collisions.GetOrAdd(a,out _)=Value(1);collisions.GetOrAdd(same,out _)=Value(2);
        Assert.Equal(Bits(Value(1)),Bits(collisions.GetOrAdd(a,out var collisionLookup)));
        Assert.Equal(VCarveStockCellFacts.Lookup.Dictionary,collisionLookup);
        var wrapped=new VCarveStockCellFacts(1,4096);
        wrapped.GetOrAdd(a,out _)=Value(3);wrapped.GetOrAdd(different,out _)=Value(4);
        Assert.All(Bits(wrapped.GetOrAdd(a,out var staleLookup)),bits=>Assert.True(double.IsNaN(BitConverter.Int64BitsToDouble(bits))));
        Assert.Equal(VCarveStockCellFacts.Lookup.New,staleLookup);Assert.Equal(2,wrapped.Evictions);
    }

    [Fact]
    public void PageGrowthKeepsExistingManagedReferencesAndAllocatesOnlyTouchedSlots()
    {
        var store=new VCarveStockCellFacts(4097,4096);
        Assert.Equal(0,store.AllocatedSlots);
        ref var first=ref store.GetOrAdd(new(0),out _);first=Value(7);
        Assert.Equal(4096,store.AllocatedSlots);Assert.Equal(1,store.AllocatedPages);
        for(ulong i=1;i<4097;i++)store.GetOrAdd(new(i),out _).Clearance=i;
        first.SeedClearance=17;
        Assert.Equal(17,store.Existing(new(0)).SeedClearance);
        Assert.Equal(4097,store.AllocatedSlots);Assert.Equal(2,store.AllocatedPages);
    }

    [Fact]
    public void WrappedFifoImportCopiesLiteralNanBitsWithoutWarmingQueriesOrSharingValues()
    {
        var source=new VCarveStockCellFacts(3,4096);
        for(ulong i=0;i<8;i++)source.GetOrAdd(new(i),out _)=Value((int)i);
        var before=source.Entries.Select(row=>(row.Key.Packed,Bits:string.Join(',',Bits(row.Value)))).ToArray();
        Assert.Equal(new ulong[]{5,6,7},before.Select(row=>row.Packed));
        var destination=new VCarveStockCellFacts(3,4096);destination.EnsureCapacity(3);
        foreach(var row in source.Entries)destination.AddImported(row.Key,row.Value);
        Assert.Equal(before,destination.Entries.Select(row=>(row.Key.Packed,Bits:string.Join(',',Bits(row.Value)))).ToArray());
        Assert.Equal(0,destination.FastHits+destination.SafeDictionaryHits+destination.Misses+destination.Evictions);
        destination.GetOrAdd(new(5),out var lookup).SeedClearance=123;
        Assert.Equal(VCarveStockCellFacts.Lookup.Dictionary,lookup);
        Assert.Equal(before,source.Entries.Select(row=>(row.Key.Packed,Bits:string.Join(',',Bits(row.Value)))).ToArray());
        Assert.Throws<InvalidOperationException>(()=>destination.AddImported(new(8),Value(8)));
    }

    [Fact]
    public void OutsidePackedCoordinateDomainRetainsFreshUncachedValuesAndOriginalMissCounts()
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
        PointD[] box=[new(-4,-4),new(4,-4),new(4,4),new(-4,4)];
        var stock=new VCarveProjectedStock([], [box],box,profile,1,default,cellCacheLimit:3,stockCellFastEntries:4096);
        const BindingFlags flags=BindingFlags.NonPublic|BindingFlags.Instance;
        var method=typeof(VCarveProjectedStock).GetMethod("GetCellFacts",flags)!;
        object cell=Activator.CreateInstance(typeof(VCarveProjectedStock).GetNestedType("Cell",flags)!)!;
        var uncached=typeof(VCarveProjectedStock).GetField("uncachedCellFacts",flags)!;
        foreach(double x in new[]{(int.MaxValue+1d)/2048,(int.MinValue-1d)/2048})
        {
            uncached.SetValue(stock,Value(9));
            var facts=(Facts)method.Invoke(stock,[cell,x,0d])!;
            Assert.All(Bits(facts),bits=>Assert.True(double.IsNaN(BitConverter.Int64BitsToDouble(bits))));
        }
        Assert.Equal(0,stock.CachedStockCellFacts);Assert.Equal(2,stock.CellGeometryCacheMisses);
        Assert.Equal(0,stock.CellGeometryCacheHits);Assert.Equal(0,stock.StockCellCacheEvictions);
    }

    [Fact]
    public void StoreAndDiagnosticFastCapacityValidationRemainExplicit()
    {
        Assert.Throws<ArgumentOutOfRangeException>(()=>new VCarveStockCellFacts(0));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new VCarveStockCellFacts(1048577));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new VCarveStockCellFacts(3,1));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new VCarveStockCellFacts(3,-1));
    }

    [Fact]
    public void ActualWholeCapsuleProofsAndAllLogicalCacheCountsMatchWithDirectMapDisabledOrEnabled()
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
        PointD[] Box(double r)=>[new(-r,-r),new(r,-r),new(r,r),new(-r,r)];
        VCarveProjectedStock Stock(int fast)=>new([new(new(0,0,-1),new(0,0,-1),2,2)],[Box(.04)],Box(4),profile,1,default,
            cellCacheLimit:3,stockCellFastEntries:fast);
        var without=Stock(0);var with=Stock(4096);
        foreach(double offset in new[]{-2.5,-1.75,-1.25,-.5,0,.5,1.25,1.75,2.5,0,-.5,.5})
        {
            var a=new Point3D(offset,-.1,-.5);var b=new Point3D(offset,.1,-.5);
            bool expected=Math.Max(double.Hypot(a.X,a.Y),double.Hypot(b.X,b.Y))+.1<2;
            Assert.Equal(expected,without.Prove(a,b));Assert.Equal(expected,with.Prove(a,b));
        }
        Assert.Equal(without.Cells,with.Cells);Assert.Equal(without.ColumnQueries,with.ColumnQueries);
        Assert.Equal(without.ColumnTests,with.ColumnTests);Assert.Equal(without.CellGeometryCacheHits,with.CellGeometryCacheHits);
        Assert.Equal(without.CellGeometryCacheMisses,with.CellGeometryCacheMisses);
        Assert.Equal(without.StockCellCacheEvictions,with.StockCellCacheEvictions);
        Assert.True(with.StockCellCacheEvictions>0);
    }

    [Fact]
    public void DiagnosticOverrideIsValidatedAndPrivateWorkersInheritTheResolvedSetting()
    {
        var previous=VCarveProjectedStock.StockCellFastEntriesOverride.Value;
        try
        {
            var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
            PointD[] box=[new(-4,-4),new(4,-4),new(4,4),new(-4,4)];
            VCarveProjectedStock Source(int? fast=null)=>new([], [box],box,profile,1,default,stockCellFastEntries:fast);
            VCarveProjectedStock.StockCellFastEntriesOverride.Value=4096;
            var source=Source();VCarveProjectedStock.StockCellFastEntriesOverride.Value=0;
            Assert.Contains("cellFactFastEntries=4096",source.CreateQueryWorker().CellFactStoreDiagnostics);
            VCarveProjectedStock.StockCellFastEntriesOverride.Value=4096;
            Assert.Contains("cellFactFastEntries=0",Source(0).CellFactStoreDiagnostics);
            VCarveProjectedStock.StockCellFastEntriesOverride.Value=1;
            Assert.Throws<ArgumentOutOfRangeException>(()=>Source());
        }
        finally{VCarveProjectedStock.StockCellFastEntriesOverride.Value=previous;}
    }
}
