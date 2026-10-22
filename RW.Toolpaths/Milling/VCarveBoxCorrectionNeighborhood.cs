using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
namespace RW.Toolpaths.Milling;
// UNTRUSTED component axis-box NOM envelope. The complete signed original
// artwork/legal target and cutter guards retain all physical authority.
internal sealed class VCarveBoxCorrectionNeighborhood
{
    private readonly VCarveSmoothing.PreparedCorrectionNeighborhood? _default;
    private readonly IReadOnlyList<IReadOnlyList<PointD>>? _originalWorld;
    private const long MaximumSnapshotPoints=4_000_000;
    private readonly Rect64 _bounds;
    private VCarveSmoothing.PreparedCorrectionNeighborhood? _fallback;
    private readonly bool _prepared;
    private int _queries, _selectionQueries, _disjointSkips;
    private double _selectionMilliseconds;
    internal bool EnvelopePrepared => _prepared;
    internal double PreparationMilliseconds { get; }
    internal double OffsetMilliseconds => _default?.OffsetMilliseconds ?? _fallback?.OffsetMilliseconds ?? 0;
    internal double ConversionMilliseconds => _default?.ConversionMilliseconds ?? _fallback?.ConversionMilliseconds ?? 0;
    internal double BoxMilliseconds { get; private set; }
    internal int Queries => _default?.Queries ?? _queries + (_fallback?.Queries ?? 0);
    internal int SelectionQueries => _default?.SelectionQueries ?? _selectionQueries + (_fallback?.SelectionQueries ?? 0);
    internal int DisjointSkips => _default?.DisjointSkips ?? _disjointSkips + (_fallback?.DisjointSkips ?? 0);
    internal long ContactProbes => _default?.ContactProbes ?? _fallback?.ContactProbes ?? 0;
    internal double SelectionMilliseconds => _default?.SelectionMilliseconds ?? _selectionMilliseconds + (_fallback?.SelectionMilliseconds ?? 0);
    internal int BoxQueries { get; private set; }
    internal int OriginalFallbackQueries { get; private set; }
    internal const long ExtraNativeTicks=16;
    // Bounded old round joins need at most~2300 rotations per full circle here;
    // this excludes huge deltas rather than borrowing a guessed drift reserve.
    // Actual authored3mm body/terminal nomination reaches are well inside it.
    internal const double MaximumNativeReach=1_000_000;
    static bool Bounded(Rect64 r)=>Math.Abs((double)r.left)<=1e12&&Math.Abs((double)r.right)<=1e12
        &&Math.Abs((double)r.top)<=1e12&&Math.Abs((double)r.bottom)<=1e12;
    internal VCarveBoxCorrectionNeighborhood(IEnumerable<IReadOnlyList<PointD>> error,CancellationToken token=default,bool useBoxSelector=false)
    {
        token.ThrowIfCancellationRequested();long start=Stopwatch.GetTimestamp();
        if(!useBoxSelector){_default=new(error,token);PreparationMilliseconds=Stopwatch.GetElapsedTime(start).TotalMilliseconds;return;}
        // Retain literal ordered WORLD input for fallback. Feeding the
        // canonical result back through Union would introduce a second Boolean.
        var copied=new List<IReadOnlyList<PointD>>();long points=0;
        using var iterator=error.GetEnumerator();
        while(iterator.MoveNext())
        {
            token.ThrowIfCancellationRequested();var ring=iterator.Current;
            if(ring.Count>MaximumSnapshotPoints-points)
            {
                // Snapshot quota is not a geometry refusal or truncation. The
                // unchanged old constructor consumes the complete remaining input
                // immediately, in literal order, then retains its own source.
                IEnumerable<IReadOnlyList<PointD>> OriginalRemaining()
                {foreach(var prior in copied)yield return prior;yield return ring;while(iterator.MoveNext())yield return iterator.Current;}
                _default=new(OriginalRemaining(),token);PreparationMilliseconds=Stopwatch.GetElapsedTime(start).TotalMilliseconds;return;
            }
            points+=ring.Count;copied.Add(ring.ToArray());
        }
        _originalWorld=copied;
        var source=Clipper.Union(new Paths64(_originalWorld.Select(r=>new Path64(r.Select(p=>new Point64((long)Math.Round(p.x*PathUtils.Scale),(long)Math.Round(p.y*PathUtils.Scale)))))),FillRule.NonZero);
        _bounds=Clipper.GetBounds(source);
        _prepared=source.Count!=0&&Bounded(_bounds);token.ThrowIfCancellationRequested();
        if(!_prepared)_fallback=new(_originalWorld,token);
        PreparationMilliseconds=Stopwatch.GetElapsedTime(start).TotalMilliseconds;
    }
    private VCarveSmoothing.PreparedCorrectionNeighborhood Literal(CancellationToken token)=>_default??(_fallback??=new(_originalWorld!,token));
    internal List<List<PointD>> Expand(double reach,CancellationToken token=default)
    {
        token.ThrowIfCancellationRequested();if(_default is not null)return _default.Expand(reach,token);
        double scaled=reach*PathUtils.Scale+4;
        if(!_prepared||!double.IsFinite(reach)||reach<0||!double.IsFinite(scaled)||scaled>MaximumNativeReach)
        {OriginalFallbackQueries++;return Literal(token).Expand(reach,token);}
        long start=Stopwatch.GetTimestamp();long padding=checked((long)Math.Ceiling(scaled)+ExtraNativeTicks);
        // The original signed canonical source bbox is retained. This box is
        // only an outer NOM selector and changes no raw physical inset/collector.
        long x0=checked(_bounds.left-padding),x1=checked(_bounds.right+padding),y0=checked(_bounds.top-padding),y1=checked(_bounds.bottom+padding);
        var output=new List<List<PointD>>{new(){new(x0/PathUtils.Scale,y0/PathUtils.Scale),new(x1/PathUtils.Scale,y0/PathUtils.Scale),new(x1/PathUtils.Scale,y1/PathUtils.Scale),new(x0/PathUtils.Scale,y1/PathUtils.Scale)}};
        BoxMilliseconds+=Stopwatch.GetElapsedTime(start).TotalMilliseconds;token.ThrowIfCancellationRequested();BoxQueries++;_queries++;return output;
    }
    internal List<List<PointD>> SelectCenters(IReadOnlyList<List<PointD>> legal,double reach,CancellationToken token=default)
    {
        token.ThrowIfCancellationRequested();if(!double.IsFinite(reach)||reach<0)throw new ArgumentOutOfRangeException(nameof(reach));
        if(_default is not null)return _default.SelectCenters(legal,reach,token);
        // Unsupported production selection delegates the WHOLE literal owner,
        // including MayIntersect, offset and final intersection in its order.
        // Qualified eligible box math and its active counters stay unchanged.
        double scaled=reach*PathUtils.Scale+4;
        if(!_prepared||!double.IsFinite(scaled)||scaled>MaximumNativeReach)
        {OriginalFallbackQueries++;return Literal(token).SelectCenters(legal,reach,token);}
        _selectionQueries++;
        // Retain the original component/finite-boundary proximity predicate.
        // A box corner alone must not end the original depth search early.
        long started=Stopwatch.GetTimestamp();
        bool possible=Literal(token).MayIntersect(legal,reach,token);
        _selectionMilliseconds+=Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        if(!possible){_disjointSkips++;return new List<List<PointD>>();}
        return PolygonOps.Intersect(legal,Expand(reach,token));
    }
}
