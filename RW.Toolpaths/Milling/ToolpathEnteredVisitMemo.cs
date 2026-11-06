namespace RW.Toolpaths.Milling;

/// <summary>Route-owned immutable entry geometry and pure reset-modal visit
/// pricing. Entry-height and transfer/stock callbacks remain with the caller.</summary>
internal sealed class ToolpathEnteredVisitMemo
{
    internal const long MaximumBytes=32L<<20;
    internal const int MaximumEntries=4096;
    private readonly record struct Key(int Owner,int Index,long EntryTopBits);
    internal sealed class Entry
    {
        internal readonly ToolpathEnteredVisitMemo Owner;
        internal readonly TaggedToolpath Source,Entered;
        internal readonly long Bytes,Points;
        internal bool Priced;
        internal double VisitSeconds;
        internal Entry(ToolpathEnteredVisitMemo owner,TaggedToolpath source,TaggedToolpath entered,long bytes)
        {
            Owner=owner;Source=source;Bytes=bytes;Points=source.Points.Count+(long)entered.Points.Count;
            // The entry planner's mutable construction lists do not escape into
            // the memo. Downstream callers retain immutable value ownership.
            var points=entered.Points.ToArray();
            Entered=entered with{Points=points,Moves=new LineToolMoves(points),Spans=entered.Spans.ToArray()};
        }
    }
    private readonly ToolpathMotionCost cost;
    private readonly long byteLimit;
    private readonly int entryLimit;
    private readonly Dictionary<Key,Entry> entries=new();
    private readonly Queue<Key> order=new();
    internal long Hits{get;private set;}
    internal long Misses{get;private set;}
    internal long Evictions{get;private set;}
    internal long BudgetRefusals{get;private set;}
    internal long PricedHits{get;private set;}
    internal long PricedMisses{get;private set;}
    internal long ChordsAvoided{get;private set;}
    internal long PointsRetained{get;private set;}
    internal long BytesRetained{get;private set;}
    internal long PeakBytes{get;private set;}
    internal int Count=>entries.Count;
    internal ToolpathEnteredVisitMemo(ToolpathMotionCost cost,long maximumBytes=MaximumBytes,int maximumEntries=MaximumEntries)
    {
        if(maximumBytes<0||maximumBytes>MaximumBytes)throw new ArgumentOutOfRangeException(nameof(maximumBytes));
        if(maximumEntries<0||maximumEntries>MaximumEntries)throw new ArgumentOutOfRangeException(nameof(maximumEntries));
        this.cost=cost;byteLimit=maximumBytes;entryLimit=maximumEntries;
    }
    internal bool TryGet(int owner,int index,double entryTop,out Entry? value)
    {
        if(entries.TryGetValue(new(owner,index,BitConverter.DoubleToInt64Bits(entryTop)),out value)){Hits++;return true;}
        Misses++;return false;
    }
    internal Entry Store(int owner,int index,double entryTop,TaggedToolpath source,TaggedToolpath entered)
    {
        long bytes=256L+64L*(source.Points.Count+(long)entered.Points.Count+source.Spans.Count+entered.Spans.Count);
        var entry=new Entry(this,source,entered,bytes);
        if(entryLimit==0||bytes>byteLimit){BudgetRefusals++;return entry;}
        while(entries.Count>=entryLimit||bytes>byteLimit-BytesRetained)
        {
            var key=order.Dequeue();var old=entries[key];entries.Remove(key);
            BytesRetained-=old.Bytes;PointsRetained-=old.Points;Evictions++;
        }
        var identity=new Key(owner,index,BitConverter.DoubleToInt64Bits(entryTop));
        entries.Add(identity,entry);order.Enqueue(identity);BytesRetained+=bytes;PointsRetained+=entry.Points;
        PeakBytes=Math.Max(PeakBytes,BytesRetained);return entry;
    }
    internal double CompleteVisitSeconds(Point3D from,Entry entry)
    {
        if(!ReferenceEquals(entry.Owner,this))return cost.CompleteVisitSeconds(from,entry.Entered);
        // Preserve the original Transfer + Visit evaluation and addition order.
        // Transfer can call opaque current-stock/context delegates every time.
        double transfer=cost.TransferSeconds(from,entry.Entered.Points[0]);
        if(!entry.Priced){entry.VisitSeconds=cost.VisitSeconds(entry.Entered);entry.Priced=true;PricedMisses++;}
        else{PricedHits++;ChordsAvoided+=Math.Max(0,entry.Entered.Points.Count-1);}
        return transfer+entry.VisitSeconds;
    }
    internal void Report(Action<string>? log)=>log?.Invoke($"stage=entered-visit-cost-memo hits={Hits} misses={Misses} evictions={Evictions} "
        +$"budgetRefusals={BudgetRefusals} pricedHits={PricedHits} pricedMisses={PricedMisses} chordsAvoided={ChordsAvoided} "
        +$"entries={Count} retainedPoints={PointsRetained} retainedBytes={BytesRetained} peakBytes={PeakBytes} byteLimit={byteLimit} entryLimit={entryLimit} "
        +"key=route-source-owner-index-exact-entry-top-bits entryHeightCallsRetained=True transferCallbacksReplayed=True "
        +"visitModalReset=NaN visitSumStartsAtZero=True originalAdditionOrder=True cachedClearanceAuthority=False");
}
