using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

internal readonly record struct ContactIntervalQueryStatistics(bool MidpointClassified, int CandidateEdges,
    bool BoundaryClassified = false);

/// <summary>
/// Certifies a complete circular cutter footprint against protected polygon material.
/// Analytic segment distances cover the whole commanded chord, including stationary plunges.
/// </summary>
public sealed class PreparedSweepExclusion
{
    private readonly PolygonOps.BoundarySegmentIndex _boundary;

    public PreparedSweepExclusion(IEnumerable<IReadOnlyList<PointD>> protectedBoundary)
    {
        var canonical = PolygonOps.Union(protectedBoundary);
        _boundary = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(canonical));
    }

    public bool IsClear(PointD start, PointD end, double radius)
    {
        if (!double.IsFinite(radius) || radius < 0)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!double.IsFinite(start.x) || !double.IsFinite(start.y)
            || !double.IsFinite(end.x) || !double.IsFinite(end.y))
            throw new ArgumentException("Sweep coordinates must be finite.");
        var a = Quantize(start);
        var b = Quantize(end);
        // Include the established integer coordinate reserve. Tangency inside
        // that uncertainty is rejected instead of certifying rounded contact.
        return !_boundary.IsInside(a) && !_boundary.IsInside(b)
            && _boundary.ClearanceViolation(a, b, radius * PathUtils.Scale + 4) == null;
    }

    /// <summary>
    /// Chord-parameter intervals whose centers contact the protected region
    /// expanded by radius. Uses boundary capsules and filled-region winding,
    /// with the same coordinate reserve as complete-footprint exclusion.
    /// </summary>
    public IReadOnlyList<(double Start, double End)> ContactIntervals(PointD start, PointD end,
        double radius, CancellationToken cancellationToken = default)
        => ContactIntervals(start, end, radius, cancellationToken, out _);

    internal IReadOnlyList<(double Start, double End)> ContactIntervals(PointD start, PointD end,
        double radius, CancellationToken cancellationToken, out ContactIntervalQueryStatistics statistics)
    {
        if (!double.IsFinite(radius) || radius < 0)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!double.IsFinite(start.x) || !double.IsFinite(start.y)
            || !double.IsFinite(end.x) || !double.IsFinite(end.y))
            throw new ArgumentException("Sweep coordinates must be finite.");
        return _boundary.ContactIntervals(Quantize(start), Quantize(end),
            radius * PathUtils.Scale + 4, cancellationToken, out statistics);
    }

    private static Point64 Quantize(PointD point) => new(
        (long)Math.Round(point.x * PathUtils.Scale),
        (long)Math.Round(point.y * PathUtils.Scale));
}
