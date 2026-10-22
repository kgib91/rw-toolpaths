using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling.Certification;

// A bounded nomination cache over one immutable complete-original index.
// Exact finite capsule tests and registered owner checks grant all authority.
internal sealed class PreparedFractionalSiteChoice
{
    private readonly PolygonOps.BoundarySegmentIndex _original;
    internal PreparedFractionalSiteChoice(PolygonOps.BoundarySegmentIndex original) => _original = original;
    private const int MaximumMemoEntries = 262_144;
    private readonly Dictionary<(long X, long Y), (Point64 Start, Point64 End)> _memo = new();
    internal long Queries, MemoHits, MemoClears;
    internal (Point64 Start, Point64 End) Choose(Point64 p) => Choose(p.X, p.Y);
    internal (Point64 Start, Point64 End) Choose(double x, double y)
    {
        Queries++;
        var key = (BitConverter.DoubleToInt64Bits(x), BitConverter.DoubleToInt64Bits(y));
        if (_memo.TryGetValue(key, out var saved)) { MemoHits++; return saved; }
        var site = _original.NearestSegmentForCertificate(x, y);
        if (_memo.Count >= MaximumMemoEntries) { _memo.Clear(); MemoClears++; }
        _memo.TryAdd(key, site);
        return site;
    }
}
