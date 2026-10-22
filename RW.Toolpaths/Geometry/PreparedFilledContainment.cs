using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

/// <summary>A strict containment proof for already canonical NonZero regions.
/// The snapshot includes every signed contour. Touching/quantized boundaries
/// return null so the caller retains its existing Boolean boundary policy.</summary>
internal sealed class PreparedFilledContainment
{
    private readonly Paths64 _region;
    private readonly PolygonOps.BoundarySegmentIndex _boundary;
    private readonly bool _largeCoordinates;
    internal int Queries { get; private set; }
    internal int Contained { get; private set; }
    internal int Outside { get; private set; }
    internal int Ambiguous { get; private set; }
    internal long EdgeQueries { get; private set; }
    internal long EnclosedHoleQueries { get; private set; }

    internal PreparedFilledContainment(Paths64 canonicalRegion)
    {
        _region = new Paths64(canonicalRegion.Select(path => new Path64(path)));
        _largeCoordinates = HasLargeCoordinates(_region);
        _boundary = new PolygonOps.BoundarySegmentIndex(_region);
    }

    internal bool? ContainsCanonical(Paths64 canonicalCandidate, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        Queries++;
        bool? Complete(bool? result)
        {
            if (result is true) Contained++;
            else if (result is false) Outside++;
            else Ambiguous++;
            return result;
        }
        if (canonicalCandidate.Count == 0) return Complete(true);
        if (_region.Count == 0) return Complete(false);
        // Beyond this range, double segment distances can lose the reserved
        // integer uncertainty. Preserve Clipper's decision for those inputs.
        if (_largeCoordinates || HasLargeCoordinates(canonicalCandidate)) return Complete(null);
        foreach (var contour in canonicalCandidate)
        {
            if (contour.Count < 3 || Clipper.Area(contour) == 0) return Complete(null);
            for (int end = 0, start = contour.Count - 1; end < contour.Count; start = end++)
            {
                if ((end & 255) == 0) token.ThrowIfCancellationRequested();
                EdgeQueries++;
                if (_boundary.BoundaryMayMeet(contour[start], contour[end])) return Complete(null);
            }
        }
        // With disjoint boundaries, each connected candidate outer lies wholly
        // on one side. Its holes do not authorize containment of filled material.
        foreach (var contour in canonicalCandidate)
            if (Clipper.IsPositive(contour) && !_boundary.IsInside(contour[0])) return Complete(false);

        PolygonOps.BoundarySegmentIndex? candidateBoundary = null;
        foreach (var contour in _region)
        {
            if (Clipper.IsPositive(contour)) continue;
            token.ThrowIfCancellationRequested();
            EnclosedHoleQueries++;
            candidateBoundary ??= new PolygonOps.BoundarySegmentIndex(canonicalCandidate);
            // Candidate vertices can all be inside while enclosing a protected
            // hole. No boundary crossing is required for this failure case.
            if (candidateBoundary.IsInside(contour[0])) return Complete(false);
        }
        return Complete(true);
    }

    private static bool HasLargeCoordinates(Paths64 paths)
        => paths.Any(path => path.Any(point => point.X < -(1L << 50) || point.X > (1L << 50)
            || point.Y < -(1L << 50) || point.Y > (1L << 50)));
}
