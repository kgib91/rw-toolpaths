using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

/// <summary>Reusable, conservative circular-sweep containment in a canonical polygon region.</summary>
public sealed class PreparedSweepRegion
{
    private readonly PolygonOps.BoundarySegmentIndex _boundary;

    public PreparedSweepRegion(IEnumerable<PathD> region)
    {
        var canonical = Clipper.Union(new PathsD(region), new PathsD(), FillRule.NonZero, 6);
        _boundary = new PolygonOps.BoundarySegmentIndex(new Paths64(canonical.Select(path =>
            new Path64(path.Select(p => Quantize(p))))));
    }

    public bool Contains(PointD start, PointD end, double radius)
    {
        if (!double.IsFinite(radius) || radius < 0) throw new ArgumentOutOfRangeException(nameof(radius));
        if (!double.IsFinite(start.x) || !double.IsFinite(start.y) || !double.IsFinite(end.x) || !double.IsFinite(end.y))
            throw new ArgumentException("Sweep coordinates must be finite.");
        return _boundary.ContainsCapsule(Quantize(start), Quantize(end), radius * PathUtils.Scale);
    }

    private static Point64 Quantize(PointD point) => new((long)Math.Round(point.x * PathUtils.Scale),
        (long)Math.Round(point.y * PathUtils.Scale));
}
