using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

/// <summary>Indexed signed distance to a canonical filled region. Positive is inside.
/// The uncertainty covers stored-boundary and query quantization; callers must retain
/// uncertain boundary patches when proving that material is protected.</summary>
public sealed class PreparedRegionDistance
{
    public const double Uncertainty = 5.0 / PathUtils.Scale;
    private readonly PolygonOps.BoundarySegmentIndex _boundary;
    private readonly (long Left,long Bottom,long Right,long Top)? _rectangle;
    private const int MaximumCachedQueries = 131_072;
    private readonly Dictionary<(long X,long Y), double> _queries = new();
    private readonly Queue<(long X,long Y)> _queryOrder = new();
    private readonly object _queryLock = new();
    private readonly bool _threadConfined;
    private long _cachedQueryHits, _boundaryQueries, _cachedQueryEvictions;
    public long CachedQueryHits => Interlocked.Read(ref _cachedQueryHits);
    public long BoundaryQueries => Interlocked.Read(ref _boundaryQueries);
    public long CachedQueryEvictions => Interlocked.Read(ref _cachedQueryEvictions);
    public int CachedQueryCount { get { lock (_queryLock) return _queries.Count; } }

    public PreparedRegionDistance(IEnumerable<IReadOnlyList<PointD>> region)
    {
        ArgumentNullException.ThrowIfNull(region);
        var canonical=Clipper.Union(PathUtils.ToClipper(region), FillRule.NonZero);
        _boundary = new PolygonOps.BoundarySegmentIndex(canonical);
        if(canonical.Count==1&&canonical[0].Count==4)
        {
            var ring=canonical[0];long left=ring.Min(p=>p.X),right=ring.Max(p=>p.X);
            long bottom=ring.Min(p=>p.Y),top=ring.Max(p=>p.Y);
            if(left<right&&bottom<top&&ring.All(p=>(p.X==left||p.X==right)&&(p.Y==bottom||p.Y==top))
                &&ring.Select(p=>(p.X,p.Y)).Distinct().Count()==4)
                _rectangle=(left,bottom,right,top);
        }
    }

    // Only immutable geometry is shared. A bounded geometry worker owns this
    // wrapper and its query memo exclusively, avoiding a common cache lock in
    // millions of independent layer/stock queries.
    internal PreparedRegionDistance CreateQueryWorker() => new(this);
    private PreparedRegionDistance(PreparedRegionDistance source)
    {
        _boundary = source._boundary; _rectangle = source._rectangle;
        _threadConfined = true;
    }

    /// <summary>Literal closest boundary site for a separate physical proof or
    /// conservative nomination refusal; this query grants no clearance.</summary>
    internal PointD? NearestBoundaryPoint(PointD point)
    {
        if (_boundary.SegmentCount == 0) return null;
        var edge = _boundary.NearestSegmentForCertificate(point.x * PathUtils.Scale, point.y * PathUtils.Scale);
        double ax = edge.Start.X / PathUtils.Scale, ay = edge.Start.Y / PathUtils.Scale;
        double vx = (edge.End.X - edge.Start.X) / PathUtils.Scale, vy = (edge.End.Y - edge.Start.Y) / PathUtils.Scale;
        double length = vx * vx + vy * vy;
        double t = length > 0 ? Math.Clamp(((point.x - ax) * vx + (point.y - ay) * vy) / length, 0, 1) : 0;
        return new(ax + vx * t, ay + vy * t);
    }

    public double SignedDistance(PointD point)
    {
        if (!double.IsFinite(point.x) || !double.IsFinite(point.y))
            throw new ArgumentException("Region query coordinates must be finite.", nameof(point));
        var quantized = new Point64((long)Math.Round(point.x * PathUtils.Scale),
            (long)Math.Round(point.y * PathUtils.Scale));
        var key=(quantized.X,quantized.Y);
        if (_boundary.SegmentCount == 0) return double.NegativeInfinity;
        if (_threadConfined)
        {
            if (_queries.TryGetValue(key, out double cached)) { _cachedQueryHits++; return cached; }
            double distance = ComputeDistance(quantized);
            RememberQuery(key, distance);
            return distance;
        }
        // The distance and sign depend only on this exact stored query coordinate.
        // Adaptive stock patches and repeated ramp laps revisit these coordinates.
        lock (_queryLock)
        {
            if (_queries.TryGetValue(key, out double cached))
            {
                Interlocked.Increment(ref _cachedQueryHits);
                return cached;
            }
        }
        double result=ComputeDistance(quantized);
        lock (_queryLock)
        {
            // An independent reader may have completed the same immutable query.
            if (_queries.ContainsKey(key)) return result;
            RememberQuery(key, result);
        }
        return result;
    }

    private void RememberQuery((long X,long Y) key, double result)
    {
        if (_queries.Count >= MaximumCachedQueries)
        {
            _queries.Remove(_queryOrder.Dequeue());
            if (_threadConfined) _cachedQueryEvictions++;
            else Interlocked.Increment(ref _cachedQueryEvictions);
        }
        _queries.Add(key, result); _queryOrder.Enqueue(key);
    }

    /// <summary>The same literal distance without a second cache. Callers such
    /// as immutable stock cells already own the exact-coordinate cache.</summary>
    internal double SignedDistanceUncached(PointD point)
    {
        if(!double.IsFinite(point.x)||!double.IsFinite(point.y))
            throw new ArgumentException("Region query coordinates must be finite.",nameof(point));
        if(_boundary.SegmentCount==0)return double.NegativeInfinity;
        return ComputeDistance(new((long)Math.Round(point.x*PathUtils.Scale),(long)Math.Round(point.y*PathUtils.Scale)));
    }

    private double ComputeDistance(Point64 point)
    {
        if (_threadConfined) _boundaryQueries++; else Interlocked.Increment(ref _boundaryQueries);
        if(_rectangle is { } box)
        {
            double x=point.X,y=point.Y;
            double dx=Math.Max(0,Math.Max(box.Left-x,x-box.Right));
            double dy=Math.Max(0,Math.Max(box.Bottom-y,y-box.Top));
            double signed=dx>0||dy>0?-Math.Sqrt(dx*dx+dy*dy)/PathUtils.Scale
                :Math.Min(Math.Min(x-box.Left,box.Right-x),Math.Min(y-box.Bottom,box.Top-y))/PathUtils.Scale;
            return Math.Abs(signed)<=Uncertainty?0:signed;
        }
        double distance=Math.Sqrt(_boundary.DistanceSquared(point))/PathUtils.Scale;
        return distance<=Uncertainty?0:_boundary.IsInside(point)?distance:-distance;
    }
}
