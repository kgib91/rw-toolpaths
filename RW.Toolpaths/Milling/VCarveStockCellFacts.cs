using System.Runtime.CompilerServices;

namespace RW.Toolpaths.Milling;

/// <summary>Owner-private exact FIFO facts. Slot pages never move or resize;
/// only actual stored value bits are returned. This is no permission policy.</summary>
internal sealed class VCarveStockCellFacts
{
    internal struct Facts
    {
        internal double PartDistance,TargetDistance,Clearance,PointClearance,SeedClearance;
        internal static Facts Unknown()=>new(){PartDistance=double.NaN,TargetDistance=double.NaN,
            Clearance=double.NaN,PointClearance=double.NaN,SeedClearance=double.NaN};
    }
    internal readonly record struct Key(ulong Packed)
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()=>HashCode.Combine((uint)Packed,(uint)(Packed>>32));
    }
    internal enum Lookup { New,Dictionary,Fast }
    internal const int PageSize=4096;
    private const int PageBits=12,PageMask=PageSize-1;
    private readonly int limit;
    private readonly Dictionary<Key,int> slots=new();
    private Facts[]?[]? factPages;
    private Key[]?[]? keyPages;
    private ulong[]? fastKeys;
    private int[]? fastSlotsPlusOne;
    private int head,allocatedSlots,allocatedPages;
    internal int Count {get;private set;}
    internal int FastEntries {get;}
    internal long FastHits,SafeDictionaryHits,Misses,Evictions;
    internal int AllocatedSlots=>allocatedSlots;
    internal int AllocatedPages=>allocatedPages;
    internal long EstimatedManagedBytes=>128L+28L*slots.EnsureCapacity(0)+48L*allocatedSlots
        +48L*allocatedPages+(factPages?.LongLength??0)*16+12L*(fastKeys?.LongLength??0);

    internal VCarveStockCellFacts(int limit,int fastEntries=0)
    {
        if(limit<1||limit>1048576)throw new ArgumentOutOfRangeException(nameof(limit));
        if(fastEntries is not(0 or 4096))throw new ArgumentOutOfRangeException(nameof(fastEntries));
        this.limit=limit;FastEntries=fastEntries;
    }

    internal void EnsureCapacity(int count)=>slots.EnsureCapacity(Math.Min(limit,count));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ref Facts At(int slot)=>ref factPages![slot>>PageBits]![slot&PageMask];
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Key KeyAt(int slot)=>keyPages![slot>>PageBits]![slot&PageMask];

    internal ref Facts Existing(Key key)=>ref At(slots[key]);
    internal IEnumerable<(Key Key,Facts Value)> Entries
    {
        get
        {
            int slot=head;
            for(int i=0;i<Count;i++)
            {
                var value=factPages![slot>>PageBits]![slot&PageMask];
                yield return(KeyAt(slot),value);
                if(++slot==limit)slot=0;
            }
        }
    }

    private void EnsurePage(int slot)
    {
        if(factPages is null)
        {
            int pages=(limit+PageMask)>>PageBits;
            factPages=new Facts[pages][];keyPages=new Key[pages][];
        }
        int page=slot>>PageBits;
        if(factPages[page] is not null)return;
        int length=Math.Min(PageSize,limit-(page<<PageBits));
        factPages[page]=new Facts[length];keyPages![page]=new Key[length];
        allocatedSlots+=length;allocatedPages++;
    }

    private void PublishFast(Key key,int slot,int lane)
    {
        if(FastEntries==0)return;
        fastKeys![lane]=key.Packed;fastSlotsPlusOne![lane]=slot+1;
    }

    internal ref Facts GetOrAdd(Key key,out Lookup lookup)
    {
        int lane=0;
        if(FastEntries!=0)
        {
            fastKeys??=new ulong[FastEntries];fastSlotsPlusOne??=new int[FastEntries];
            lane=key.GetHashCode()&(FastEntries-1);
            int plus=fastSlotsPlusOne[lane];
            // A direct entry can outlive FIFO eviction. Its exact key AND the
            // current occupied slot key must match before any fact is read.
            if(plus>0&&plus<=Count&&fastKeys[lane]==key.Packed&&KeyAt(plus-1)==key)
            {FastHits++;lookup=Lookup.Fast;return ref At(plus-1);}
        }
        if(slots.TryGetValue(key,out int slot))
        {
            SafeDictionaryHits++;lookup=Lookup.Dictionary;PublishFast(key,slot,lane);return ref At(slot);
        }
        Misses++;lookup=Lookup.New;
        slot=Count<limit?Count:head;EnsurePage(slot);
        if(Count==limit){slots.Remove(KeyAt(slot));Evictions++;}
        At(slot)=Facts.Unknown();keyPages![slot>>PageBits]![slot&PageMask]=key;
        slots.Add(key,slot);
        if(Count<limit)Count++;else if(++head==limit)head=0;
        // All five unknown facts and the exact occupied key are initialized
        // before this slot can be observed through the direct map.
        PublishFast(key,slot,lane);return ref At(slot);
    }

    // Import is a literal FIFO value copy, not a query. It neither warms the
    // direct cache nor changes hit/miss counters or the source owner.
    internal void AddImported(Key key,Facts value)
    {
        if(Count>=limit)throw new InvalidOperationException("Imported facts exceed the destination limit.");
        int slot=Count;EnsurePage(slot);slots.Add(key,slot);
        At(slot)=value;keyPages![slot>>PageBits]![slot&PageMask]=key;Count++;
    }
}
