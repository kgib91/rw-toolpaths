using System.Buffers;
using Clipper2Lib;

namespace RW.Toolpaths.Milling;

// Operation-owned scratch only. Returned hulls copy value coordinates and never
// reference rentals. Physical radius/reserve/zero-tip decisions remain callers'.
internal sealed class VCarveOrderedDiscHull : IDisposable
{
    private static readonly IComparer<Point64> XY = Comparer<Point64>.Create(Compare);
    private static readonly IComparer<Point64> Y = Comparer<Point64>.Create((a, b) => a.Y.CompareTo(b.Y));
    private readonly Dictionary<int, int[]> _orders = new();
    private (double X, double Y)[] _circle = Array.Empty<(double, double)>();
    private int[] _order = Array.Empty<int>();
    private Point64[]? _first, _second, _merged;
    private int _rows, _firstCount, _secondCount;
    private DiscRowKey? _firstKey, _secondKey;
    private bool _finiteCircle=true;
    private readonly bool literalFinalRowValidation;
    private readonly bool literalHullCapacity;
    private sealed record DiscRowSnapshot(DiscRowKey Key,Point64[] Row);
    private readonly Dictionary<DiscRowKey,DiscRowSnapshot>? extendedRows;
    private readonly Queue<DiscRowSnapshot>? extendedRowOrder;
    private readonly int extendedRowLimit;
    internal const long MaximumExtendedRowBytes=1024L*1024;
    private long extendedRowBytes;
    private readonly record struct DiscRowKey((double X,double Y)[] Circle,long X,long Y,long Radius,int Count);
    internal long Calls, Vertices, TieGroups, TiePairs, PairSwaps, SortFallbacks, DuplicatesSkipped;
    internal long DiscRowCacheHits, DiscRowCacheMisses, DiscRowCacheRefusals, DiscRowBufferSwaps, DiscVerticesAvoided;
    internal long FinalRowValidationComparisons,FinalRowValidationComparisonsAvoided;
    internal long HullCapacityHints,HullCapacityResizes;
    internal long ExtendedRowHits,ExtendedRowMisses,ExtendedRowEvictions,ExtendedRowBudgetRefusals,ExtendedRowBufferReuses;
    internal long ExtendedRowPeakBytes;
    internal int ExtendedRowEntries=>extendedRows?.Count??0;
    internal long ExtendedRowBytes=>extendedRowBytes;
    internal int MaximumRetainedRows=>extendedRowLimit+2;
    internal int MaximumTieGroup, CircleOrderBuilds;

    // Internal oracle only: retain the former defensive pass when requested.
    // Production rows use the X-order and equal-X Y-order certificates below.
    internal VCarveOrderedDiscHull(bool literalFinalRowValidation=false,int retainedDiscRows=2,bool literalHullCapacity=false)
    {
        if(retainedDiscRows is <2 or >32)throw new ArgumentOutOfRangeException(nameof(retainedDiscRows));
        this.literalFinalRowValidation=literalFinalRowValidation;this.literalHullCapacity=literalHullCapacity;extendedRowLimit=retainedDiscRows-2;
        if(extendedRowLimit>0){extendedRows=new();extendedRowOrder=new();}
    }

    internal void BeginCircle((double X, double Y)[] circle)
    {
        // Internal unit-circle arrays are immutable. Retain only the two row
        // keys, and never identify a custom circle by its vertex count alone.
        if(!ReferenceEquals(_circle,circle))_finiteCircle=circle.All(p=>double.IsFinite(p.X)&&double.IsFinite(p.Y));
        _circle = circle; _rows = _firstCount = _secondCount = 0;
        if (!_orders.TryGetValue(circle.Length, out var order))
        {
            order = Enumerable.Range(0, circle.Length).ToArray();
            Array.Sort(order, (a, b) => circle[a].X == circle[b].X
                ? circle[a].Y.CompareTo(circle[b].Y) : circle[a].X.CompareTo(circle[b].X));
            _orders.Add(circle.Length, order); CircleOrderBuilds++;
        }
        _order = order;
    }

    internal void AddDisc(Point3D center, double radius)
    {
        int ordinal = _rows++;
        if(ordinal>1)throw new InvalidOperationException("A chord hull has at most two endpoint rows.");
        bool reusable=_finiteCircle&&double.IsFinite(center.X)&&double.IsFinite(center.Y)&&double.IsFinite(radius)&&radius>=0;
        var key=new DiscRowKey(_circle,BitConverter.DoubleToInt64Bits(center.X),BitConverter.DoubleToInt64Bits(center.Y),
            BitConverter.DoubleToInt64Bits(radius),_circle.Length);
        var prior=ordinal==0?_firstKey:_secondKey;
        if(reusable&&prior!=key&&ordinal==0&&_secondKey==key)
        {
            // Only the first row may take the previous second row. Swapping
            // during ordinal one would overwrite this hull's current first.
            (_first,_second)=(_second,_first);(_firstKey,_secondKey)=(_secondKey,_firstKey);
            prior=_firstKey;DiscRowBufferSwaps++;
        }
        if(reusable&&prior==key)
        {
            SetCount(ordinal,_circle.Length);Vertices+=_circle.Length;
            DiscRowCacheHits++;DiscVerticesAvoided+=_circle.Length;return;
        }
        if(reusable&&extendedRows is not null)
        {
            if(extendedRows.TryGetValue(key,out var captured))
            {
                Array.Copy(captured.Row,Row(ordinal,captured.Row.Length),captured.Row.Length);
                SetCount(ordinal,captured.Row.Length);SetKey(ordinal,key);Vertices+=captured.Row.Length;
                ExtendedRowHits++;DiscRowCacheHits++;DiscVerticesAvoided+=captured.Row.Length;return;
            }
            ExtendedRowMisses++;
        }
        if(reusable)DiscRowCacheMisses++;else DiscRowCacheRefusals++;
        SetKey(ordinal,null);
        var row = Row(ordinal, _circle.Length);
        for (int index = 0; index < _order.Length; index++)
        {
            var unit = _circle[_order[index]];
            // Literal old IEEE expression; changing addition/multiply/round order
            // would change the integer input to the physical footprint union.
            row[index] = new Point64((long)Math.Round((center.X + radius * unit.X) * PathUtils.Scale),
                (long)Math.Round((center.Y + radius * unit.Y) * PathUtils.Scale));
        }
        SetCount(ordinal, _circle.Length); Vertices += _circle.Length;
        OrderQuantizedRow(row, _circle.Length, !double.IsFinite(center.X)
            || !double.IsFinite(center.Y) || !double.IsFinite(radius) || radius < 0);
        if(reusable){SetKey(ordinal,key);RetainRow(key,row,_circle.Length);}
    }

    private void RetainRow(DiscRowKey key,Point64[] row,int count)
    {
        if(extendedRows is null)return;
        long bytes=32L+16L*count;
        if(bytes>MaximumExtendedRowBytes){ExtendedRowBudgetRefusals++;return;}
        Point64[]? backing=null;
        while(extendedRows.Count>=extendedRowLimit||extendedRowBytes+bytes>MaximumExtendedRowBytes)
        {
            var old=extendedRowOrder!.Dequeue();extendedRows.Remove(old.Key);
            extendedRowBytes-=32L+16L*old.Row.Length;ExtendedRowEvictions++;
            if(backing is null&&old.Row.Length==count){backing=old.Row;ExtendedRowBufferReuses++;}
        }
        backing??=new Point64[count];Array.Copy(row,backing,count);
        var snapshot=new DiscRowSnapshot(key,backing);extendedRows.Add(key,snapshot);extendedRowOrder!.Enqueue(snapshot);
        extendedRowBytes+=bytes;ExtendedRowPeakBytes=Math.Max(ExtendedRowPeakBytes,extendedRowBytes);
    }

    internal void AddPoint(Point64 point)
    {
        int ordinal = _rows++;SetKey(ordinal,null);var row = Row(ordinal, 1);
        row[0] = point; SetCount(ordinal, 1); Vertices++;
    }

    internal Path64 FinishHull()
    {
        Calls++;
        int total = checked(_firstCount + _secondCount);
        if (total == 0) return new Path64();
        var merged = Ensure(ref _merged, total);
        int first = 0, second = 0, count = 0;
        while (first < _firstCount || second < _secondCount)
        {
            var point = second >= _secondCount || first < _firstCount && Compare(_first![first], _second![second]) <= 0
                ? _first![first++] : _second![second++];
            if (count != 0 && EqualityComparer<Point64>.Default.Equals(merged[count - 1], point))
            { DuplicatesSkipped++; continue; }
            merged[count++] = point;
        }
        if (count <= 2)
        { var small = new Path64(count); for (int index = 0; index < count; index++) small.Add(merged[index]); return small; }
        // Andrew's upper chain temporarily appends the closing first vertex.
        // A convex single row therefore needs count+1 before the final removal.
        int capacity=count;
        if(!literalHullCapacity&&count<int.MaxValue){capacity++;HullCapacityHints++;}
        var hull = new Path64(capacity);
        for (int index = 0; index < count; index++)
        {
            var point = merged[index];
            while (hull.Count >= 2 && Cross(hull[^2], hull[^1], point) <= 0) hull.RemoveAt(hull.Count - 1);
            hull.Add(point);
        }
        int lower = hull.Count;
        for (int index = count - 2; index >= 0; index--)
        {
            var point = merged[index];
            while (hull.Count > lower && Cross(hull[^2], hull[^1], point) <= 0) hull.RemoveAt(hull.Count - 1);
            hull.Add(point);
        }
        hull.RemoveAt(hull.Count - 1);
        if(hull.Capacity>capacity)HullCapacityResizes++;
        return hull;
    }

    private void OrderQuantizedRow(Point64[] row, int count, bool forceSort)
    {
        for (int index = 1; !forceSort && index < count; index++) forceSort = row[index - 1].X > row[index].X;
        if (forceSort) { Array.Sort(row, 0, count, XY); SortFallbacks++; return; }
        for (int first = 0; first < count;)
        {
            int last = first + 1;
            while (last < count && row[last].X == row[first].X) last++;
            int group = last - first;
            if (group > 1)
            {
                TieGroups++; MaximumTieGroup = Math.Max(MaximumTieGroup, group);
                if (group == 2)
                { TiePairs++; if (row[first].Y > row[first + 1].Y) { (row[first], row[first + 1]) = (row[first + 1], row[first]); PairSwaps++; } }
                else Array.Sort(row, first, group, Y);
            }
            first = last;
        }
        // The first pass already certified nondecreasing X (or full-sorted and
        // returned). Only equal-X groups were subsequently reordered, and each
        // is now nondecreasing Y. The complete row is therefore lexicographic;
        // rechecking every pair cannot discover another fallback.
        if(literalFinalRowValidation)
        {
            for (int index = 1; index < count; index++)
            {
                FinalRowValidationComparisons++;
                if (Compare(row[index - 1], row[index]) > 0)
                { Array.Sort(row, 0, count, XY); SortFallbacks++; return; }
            }
        }
        else FinalRowValidationComparisonsAvoided+=Math.Max(0,count-1);
    }

    private Point64[] Row(int ordinal, int count) => ordinal switch
    { 0 => Ensure(ref _first, count), 1 => Ensure(ref _second, count), _ => throw new InvalidOperationException("A chord hull has at most two endpoint rows.") };
    private void SetCount(int ordinal, int count) { if (ordinal == 0) _firstCount = count; else _secondCount = count; }
    private void SetKey(int ordinal,DiscRowKey? key)
    {
        if(ordinal==0)_firstKey=key;else if(ordinal==1)_secondKey=key;
        else throw new InvalidOperationException("A chord hull has at most two endpoint rows.");
    }
    private static Point64[] Ensure(ref Point64[]? buffer, int count)
    {
        if (buffer is null || buffer.Length < count)
        { if (buffer is not null) ArrayPool<Point64>.Shared.Return(buffer); buffer = ArrayPool<Point64>.Shared.Rent(count); }
        return buffer;
    }
    private static int Compare(Point64 a, Point64 b) => a.X == b.X ? a.Y.CompareTo(b.Y) : a.X.CompareTo(b.X);
    // Preserve the old subtraction/cast order, including its extreme-input behavior.
    private static Int128 Cross(Point64 a, Point64 b, Point64 c)
        => (Int128)(b.X - a.X) * (c.Y - a.Y) - (Int128)(b.Y - a.Y) * (c.X - a.X);
    public void Dispose()
    {
        if (_first is not null) ArrayPool<Point64>.Shared.Return(_first);
        if (_second is not null) ArrayPool<Point64>.Shared.Return(_second);
        if (_merged is not null) ArrayPool<Point64>.Shared.Return(_merged);
        _first = _second = _merged = null;
        _firstKey=_secondKey=null;
        extendedRows?.Clear();extendedRowOrder?.Clear();extendedRowBytes=0;
    }
}
