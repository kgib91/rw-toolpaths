using Clipper2Lib;

namespace RW.Toolpaths;

/// <summary>Finite-chord certificates for complete or local original-site parabola pieces.
/// A quadratic control hull lies in a convex endpoint-chord tube; recursive
/// subdivision never accepts an infinite-line flatness test or truncated edge.</summary>
internal static class BoundedLocalMedialEmitter
{
    private const int MaximumPieces = 20000;

    // This internal eligibility is confined to OffsetPocket full-source graphs.
    // Every certificate is evaluated against the actual binary64 world command.
    internal readonly record struct SourceCoordinates(double OriginX, double OriginY, double Scale);

    internal static bool TryEmit(MedialAxisEdgeClassifier.EdgeData edge, RectD window,
        double maximumError, double arithmeticReserve, List<MedialSegment> output,
        CancellationToken token)
        => TryEmitCore(edge, window, maximumError, arithmeticReserve, null, output, token,
            out _, out _, out _);

    internal static bool TryEmitFull(MedialAxisEdgeClassifier.EdgeData edge, SourceCoordinates coordinates,
        double maximumWorldError, List<MedialSegment> output, CancellationToken token,
        out string failure, out int visited, out double maximumAcceptedWorldBound,
        int maximumPieces = MaximumPieces)
        => TryEmitCore(edge, null, maximumWorldError, 0, coordinates, output, token,
            out failure, out visited, out maximumAcceptedWorldBound, maximumPieces);

    private static bool TryEmitCore(MedialAxisEdgeClassifier.EdgeData edge, RectD? window, double maximumError,
        double arithmeticReserve, SourceCoordinates? coordinates, List<MedialSegment> output,
        CancellationToken token, out string failure, out int visited, out double maximumAcceptedBound,
        int maximumPieces = MaximumPieces)
    {
        token.ThrowIfCancellationRequested();
        failure = ""; visited = 0; maximumAcceptedBound = 0;
        if (!double.IsFinite(maximumError) || maximumError <= 0) { failure = "invalid-error"; return false; }
        if (edge.Vertex0 is not { } nativeA || edge.Vertex1 is not { } nativeB) { failure = "nonfinite-edge"; return false; }
        if (coordinates is { } conversion && (!double.IsFinite(conversion.OriginX)
            || !double.IsFinite(conversion.OriginY) || !double.IsFinite(conversion.Scale) || conversion.Scale <= 0))
        { failure = "invalid-world-conditioning"; return false; }
        if (!edge.IsCurved)
        {
            if (coordinates is not null)
            {
                maximumAcceptedBound = Math.Max(
                    DistanceUpper(CertificatePosition(new(OutwardInterval.Exact(nativeA.X), OutwardInterval.Exact(nativeA.Y))), CertificatePoint(nativeA), CertificatePoint(nativeA)),
                    DistanceUpper(CertificatePosition(new(OutwardInterval.Exact(nativeB.X), OutwardInterval.Exact(nativeB.Y))), CertificatePoint(nativeB), CertificatePoint(nativeB)));
                if (maximumAcceptedBound > maximumError) { failure = "linear-world-conversion"; return false; }
            }
            if (Meets(nativeA, nativeB))
                return MedialAxisSegmentEmitter.TryCreateOutputSegments(edge, output,
                    noParabola: false, showSites: false, rejectIncompleteDiscretization: true);
            return true;
        }
        var focus = edge.Cell.ContainsPoint ? edge.Cell.Point : edge.TwinCell.Point;
        var directrix = edge.Cell.ContainsPoint ? edge.TwinCell.Segment : edge.Cell.Segment;
        if (focus is null || directrix is null) { failure = "missing-focus-directrix"; return false; }
        // Original Boost focus/directrix coordinates are integer sites. Compute
        // the cancellation-sensitive dot/cross coefficients exactly before
        // enclosing every subsequent floating operation.
        if (!IntegerSite(focus) || !IntegerSite(directrix.Low) || !IntegerSite(directrix.High)) { failure = "noninteger-site"; return false; }
        Int128 integerDx = (long)directrix.High.X - (long)directrix.Low.X,
            integerDy = (long)directrix.High.Y - (long)directrix.Low.Y;
        Int128 focusDx = (long)focus.X - (long)directrix.Low.X,
            focusDy = (long)focus.Y - (long)directrix.Low.Y;
        Int128 integerLengthSquared = integerDx * integerDx + integerDy * integerDy;
        Int128 integerProjectedFocus = integerDx * focusDx + integerDy * focusDy;
        Int128 integerNormalFocus = integerDx * focusDy - integerDy * focusDx;
        if (integerLengthSquared <= 0 || integerNormalFocus == 0) { failure = "degenerate-focus-directrix"; return false; }
        var dx = OutwardInterval.Integer(integerDx); var dy = OutwardInterval.Integer(integerDy);
        var lengthSquared = OutwardInterval.Integer(integerLengthSquared);
        var projectedFocus = OutwardInterval.Integer(integerProjectedFocus);
        var normalFocus = OutwardInterval.Integer(integerNormalFocus);
        var start = Projection(nativeA); var end = Projection(nativeB);
        var initialA = Evaluate(start); var initialB = Evaluate(end);
        if (!initialA.Finite || !initialB.Finite
            || DistanceUpper(CertificatePosition(initialA), CertificatePoint(nativeA), CertificatePoint(nativeA)) > maximumError
            || DistanceUpper(CertificatePosition(initialB), CertificatePoint(nativeB), CertificatePoint(nativeB)) > maximumError) { failure = "native-endpoint-fit"; return false; }
        var temporary = new List<MedialSegment>();
        var stack = new Stack<(OutwardInterval Start, OutwardInterval End, MedialAxisEdgeClassifier.EdgePoint A,
            MedialAxisEdgeClassifier.EdgePoint B, int Depth)>();
        stack.Push((start, end, nativeA, nativeB, 0));
        while (stack.TryPop(out var piece))
        {
            if ((++visited & 255) == 0) token.ThrowIfCancellationRequested();
            if (visited > maximumPieces) { failure = "piece-budget"; return false; }
            var midParameter = (piece.Start + piece.End) * OutwardInterval.Exact(.5);
            var analyticStart = Evaluate(piece.Start); var analyticEnd = Evaluate(piece.End);
            var midpointBounds = Evaluate(midParameter);
            var control = new Position(midpointBounds.X * OutwardInterval.Exact(2)
                    - (analyticStart.X + analyticEnd.X) * OutwardInterval.Exact(.5),
                midpointBounds.Y * OutwardInterval.Exact(2)
                    - (analyticStart.Y + analyticEnd.Y) * OutwardInterval.Exact(.5));
            if (!analyticStart.Finite || !analyticEnd.Finite || !midpointBounds.Finite || !control.Finite) { failure = "nonfinite-analytic-enclosure"; return false; }
            if (!HullMeets(analyticStart, analyticEnd, control)) continue;
            var commandA = CertificatePoint(piece.A); var commandB = CertificatePoint(piece.B);
            double bound = Math.Max(DistanceUpper(CertificatePosition(analyticStart), commandA, commandB),
                Math.Max(DistanceUpper(CertificatePosition(analyticEnd), commandA, commandB),
                    DistanceUpper(CertificatePosition(control), commandA, commandB)));
            if (bound <= maximumError)
            {
                maximumAcceptedBound = Math.Max(maximumAcceptedBound, bound);
                temporary.Add(new(new(piece.A.X, piece.A.Y, Distance(piece.A, focus)),
                    new(piece.B.X, piece.B.Y, Distance(piece.B, focus))));
                continue;
            }
            if (piece.Depth >= 48 || midParameter == piece.Start || midParameter == piece.End) { failure = "unresolved-subdivision"; return false; }
            var midpoint = new MedialAxisEdgeClassifier.EdgePoint(midpointBounds.X.Midpoint, midpointBounds.Y.Midpoint);
            if (!Finite(midpoint)) { failure = "nonfinite-midpoint"; return false; }
            stack.Push((midParameter, piece.End, midpoint, piece.B, piece.Depth + 1));
            stack.Push((piece.Start, midParameter, piece.A, midpoint, piece.Depth + 1));
        }
        output.AddRange(temporary);
        return true;

        OutwardInterval Projection(MedialAxisEdgeClassifier.EdgePoint p)
            => dx * (OutwardInterval.Exact(p.X) - OutwardInterval.Exact(directrix.Low.X))
                + dy * (OutwardInterval.Exact(p.Y) - OutwardInterval.Exact(directrix.Low.Y));
        Position Evaluate(OutwardInterval projection)
        {
            var delta = projection - projectedFocus;
            var normal = (delta.Square() + normalFocus.Square()) / (OutwardInterval.Exact(2) * normalFocus);
            return new((dx * projection - dy * normal) / lengthSquared + OutwardInterval.Exact(directrix.Low.X),
                (dy * projection + dx * normal) / lengthSquared + OutwardInterval.Exact(directrix.Low.Y));
        }
        Position CertificatePosition(Position native)
            => coordinates is { } placement
                ? new(native.X / OutwardInterval.Exact(placement.Scale) + OutwardInterval.Exact(placement.OriginX),
                    native.Y / OutwardInterval.Exact(placement.Scale) + OutwardInterval.Exact(placement.OriginY))
                : native;
        MedialAxisEdgeClassifier.EdgePoint CertificatePoint(MedialAxisEdgeClassifier.EdgePoint native)
            => coordinates is { } placement
                ? new(native.X / placement.Scale + placement.OriginX, native.Y / placement.Scale + placement.OriginY)
                : native;
        bool HullMeets(Position a, Position b, Position c)
            => window is not { } query || Math.Min(a.X.Low, Math.Min(b.X.Low, c.X.Low)) <= query.right
                && Math.Max(a.X.High, Math.Max(b.X.High, c.X.High)) >= query.left
                && Math.Min(a.Y.Low, Math.Min(b.Y.Low, c.Y.Low)) <= query.bottom
                && Math.Max(a.Y.High, Math.Max(b.Y.High, c.Y.High)) >= query.top;
        bool Meets(MedialAxisEdgeClassifier.EdgePoint a, MedialAxisEdgeClassifier.EdgePoint b)
        {
            if (window is not { } query) return true;
            double extra = maximumError + arithmeticReserve;
            return Math.Min(a.X, b.X) - extra <= query.right && Math.Max(a.X, b.X) + extra >= query.left
                && Math.Min(a.Y, b.Y) - extra <= query.bottom && Math.Max(a.Y, b.Y) + extra >= query.top;
        }


    }

    private readonly record struct Position(OutwardInterval X, OutwardInterval Y)
    { internal bool Finite => X.IsFinite && Y.IsFinite; }

    private static bool IntegerSite(MedialAxisEdgeClassifier.EdgePoint p)
        => Finite(p) && p.X == Math.Round(p.X) && p.Y == Math.Round(p.Y)
            && Math.Abs(p.X) <= int.MaxValue && Math.Abs(p.Y) <= int.MaxValue;

    private static double DistanceUpper(Position enclosure, MedialAxisEdgeClassifier.EdgePoint a,
        MedialAxisEdgeClassifier.EdgePoint b)
    {
        // Distance to a finite segment is convex; bounding the four rectangle
        // corners encloses the complete uncertain quadratic control point.
        double bound = 0;
        for (int xi = 0; xi < 2; xi++)
        for (int yi = 0; yi < 2; yi++)
        {
            double x = xi == 0 ? enclosure.X.Low : enclosure.X.High;
            double y = yi == 0 ? enclosure.Y.Low : enclosure.Y.High;
            double dx = b.X - a.X, dy = b.Y - a.Y, squared = dx * dx + dy * dy;
            double t = squared == 0 ? 0 : Math.Clamp(((x - a.X) * dx + (y - a.Y) * dy) / squared, 0, 1);
            // The chosen binary64 t is a real point ON the finite command
            // segment. It need not be the exact nearest projection: an outward
            // distance to this one point is already a conservative upper bound.
            var fraction = OutwardInterval.Exact(t);
            var complement = OutwardInterval.Exact(1) - fraction;
            var commandX = OutwardInterval.Exact(a.X) * complement + OutwardInterval.Exact(b.X) * fraction;
            var commandY = OutwardInterval.Exact(a.Y) * complement + OutwardInterval.Exact(b.Y) * fraction;
            double upper = ((OutwardInterval.Exact(x) - commandX).Square()
                + (OutwardInterval.Exact(y) - commandY).Square()).SqrtUpper();
            if (!double.IsFinite(upper)) return double.PositiveInfinity;
            bound = Math.Max(bound, upper);
        }
        return bound;
    }

    internal static double FiniteChordDistanceSquared(MedialAxisEdgeClassifier.EdgePoint p,
        MedialAxisEdgeClassifier.EdgePoint a, MedialAxisEdgeClassifier.EdgePoint b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, squared = dx * dx + dy * dy;
        double t = squared == 0 ? 0 : Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy) / squared, 0, 1);
        double ex = p.X - a.X - t * dx, ey = p.Y - a.Y - t * dy;
        return ex * ex + ey * ey;
    }

    private static bool Finite(MedialAxisEdgeClassifier.EdgePoint p) => double.IsFinite(p.X) && double.IsFinite(p.Y);
    private static double Distance(MedialAxisEdgeClassifier.EdgePoint a, MedialAxisEdgeClassifier.EdgePoint b)
        => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
}
