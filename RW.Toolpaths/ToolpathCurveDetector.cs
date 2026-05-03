using Clipper2Lib;

namespace RW.Toolpaths;

public enum ToolMoveKind
{
    Line,
    ArcCw,
    ArcCcw,
    CubicBezier
}

public readonly record struct ToolMove
{
    private ToolMove(
        ToolMoveKind kind,
        Point3D start,
        Point3D end,
        Point3D control1,
        Point3D control2,
        PointD arcCenter,
        double arcRadius,
        double maxError)
    {
        Kind = kind;
        Start = start;
        End = end;
        Control1 = control1;
        Control2 = control2;
        ArcCenter = arcCenter;
        ArcRadius = arcRadius;
        MaxError = maxError;
    }

    public ToolMoveKind Kind { get; init; }
    public Point3D Start { get; init; }
    public Point3D End { get; init; }
    public Point3D Control1 { get; init; }
    public Point3D Control2 { get; init; }
    public PointD ArcCenter { get; init; }
    public double ArcRadius { get; init; }
    public double MaxError { get; init; }

    public static ToolMove Line(Point3D start, Point3D end, double maxError = 0.0) =>
        new(ToolMoveKind.Line, start, end, default, default, default, 0.0, maxError);

    public static ToolMove Arc(Point3D start, Point3D end, PointD center, double radius, bool clockwise, double maxError) =>
        new(clockwise ? ToolMoveKind.ArcCw : ToolMoveKind.ArcCcw, start, end, default, default, center, radius, maxError);

    public static ToolMove CubicBezier(Point3D start, Point3D control1, Point3D control2, Point3D end, double maxError) =>
        new(ToolMoveKind.CubicBezier, start, end, control1, control2, default, 0.0, maxError);
}

public static class ToolpathCurveDetector
{
    public const double DefaultTolerance = 0.00025;

    private const int MinArcPointCount = 4;
    private const int MinBezierPointCount = 5;
    private const int MaxCandidatePointCount = 96;
    private const double MaxArcInteriorTurnRad = Math.PI / 4.0;
    private const double MaxBezierInteriorTurnRad = Math.PI / 4.0;
    private const double SpatialDriftToleranceMultiplier = 1.0;
    private const double BoundsToleranceMultiplier = 8.0;

    public static IReadOnlyList<ToolMove> DetectMoves(
        IReadOnlyList<Point3D> points,
        double tolerance = DefaultTolerance,
        bool closePath = false)
    {
        if (points is null) throw new ArgumentNullException(nameof(points));
        if (tolerance < 0) throw new ArgumentOutOfRangeException(nameof(tolerance));
        if (points.Count < 2) return Array.Empty<ToolMove>();

        if (closePath && !SamePoint(points[0], points[^1]))
        {
            var closed = new List<Point3D>(points.Count + 1);
            closed.AddRange(points);
            closed.Add(points[0]);
            points = closed;
        }

        if (closePath && TryDetectClosedArcRing(points, tolerance, out var closedArcMoves))
            return closedArcMoves;

        var moves = new List<ToolMove>();
        int index = 0;
        while (index < points.Count - 1)
        {
            var line = FindBestLine(points, index, tolerance);
            var arc = FindBestArc(points, index, tolerance);
            var bezier = FindBestBezier(points, index, tolerance);

            Candidate best = line;
            if (arc.IsValid && arc.EndIndex >= best.EndIndex)
                best = arc;
            if (bezier.IsValid && bezier.EndIndex > best.EndIndex + 2)
                best = bezier;
            else if (bezier.IsValid && !arc.IsValid && bezier.EndIndex > best.EndIndex)
                best = bezier;

            moves.Add(best.Move);
            index = Math.Max(index + 1, best.EndIndex);
        }

        return moves;
    }

    public static IReadOnlyList<Point3D> SampleMove(ToolMove move, double chordTolerance = DefaultTolerance)
    {
        if (chordTolerance <= 0) chordTolerance = DefaultTolerance;

        if (move.Kind == ToolMoveKind.Line)
            return new[] { move.Start, move.End };

        int segments = move.Kind is ToolMoveKind.ArcCw or ToolMoveKind.ArcCcw
            ? ArcSampleCount(move, chordTolerance)
            : BezierSampleCount(move, chordTolerance);

        var points = new List<Point3D>(segments + 1);
        for (int i = 0; i <= segments; i++)
        {
            double t = i / (double)segments;
            points.Add(move.Kind switch
            {
                ToolMoveKind.ArcCw or ToolMoveKind.ArcCcw => EvaluateArc(move, t),
                ToolMoveKind.CubicBezier => EvaluateCubic(move.Start, move.Control1, move.Control2, move.End, t),
                _ => t == 0 ? move.Start : move.End,
            });
        }

        return points;
    }

    private static Candidate FindBestLine(IReadOnlyList<Point3D> points, int start, double tolerance)
    {
        int endLimit = Math.Min(points.Count - 1, start + MaxCandidatePointCount - 1);
        int bestEnd = start + 1;
        double bestError = 0.0;

        for (int end = start + 2; end <= endLimit; end++)
        {
            double error = MaxLineError(points, start, end);
            if (error > tolerance)
                break;
            bestEnd = end;
            bestError = error;
        }

        return new Candidate(bestEnd, ToolMove.Line(points[start], points[bestEnd], bestError));
    }

    private static bool TryDetectClosedArcRing(
        IReadOnlyList<Point3D> points,
        double tolerance,
        out IReadOnlyList<ToolMove> moves)
    {
        moves = Array.Empty<ToolMove>();
        if (points.Count < 9 || !SamePoint(points[0], points[^1]))
            return false;

        int last = points.Count - 1;
        foreach (int segmentCount in new[] { 2, 3, 4, 6, 8 })
        {
            if (last < segmentCount * (MinArcPointCount - 1))
                continue;

            var candidate = new List<ToolMove>(segmentCount);
            int start = 0;
            bool allArcs = true;
            for (int segment = 1; segment <= segmentCount; segment++)
            {
                int end = (int)Math.Round(segment * last / (double)segmentCount);
                if (end - start < MinArcPointCount - 1
                    || !TryFitArc(points, start, end, tolerance, out var move))
                {
                    allArcs = false;
                    break;
                }

                candidate.Add(move);
                start = end;
            }

            if (allArcs)
            {
                moves = candidate;
                return true;
            }
        }

        return false;
    }

    private static Candidate FindBestArc(IReadOnlyList<Point3D> points, int start, double tolerance)
    {
        int endLimit = Math.Min(points.Count - 1, start + MaxCandidatePointCount - 1);
        Candidate best = default;

        for (int end = start + MinArcPointCount - 1; end <= endLimit; end++)
        {
            if (TryFitArc(points, start, end, tolerance, out var move))
                best = new Candidate(end, move);
            else if (best.IsValid)
                break;
        }

        return best;
    }

    private static Candidate FindBestBezier(IReadOnlyList<Point3D> points, int start, double tolerance)
    {
        int endLimit = Math.Min(points.Count - 1, start + MaxCandidatePointCount - 1);
        Candidate best = default;

        for (int end = start + MinBezierPointCount - 1; end <= endLimit; end++)
        {
            if (TryFitCubicBezier(points, start, end, tolerance, out var move))
                best = new Candidate(end, move);
            else if (best.IsValid)
                break;
        }

        return best;
    }

    private static bool TryFitArc(
        IReadOnlyList<Point3D> points,
        int start,
        int end,
        double tolerance,
        out ToolMove move)
    {
        move = default;
        int mid = (start + end) / 2;
        var p0 = points[start];
        var pm = points[mid];
        var p1 = points[end];

        if (HasSharpInteriorTurn(points, start, end, MaxArcInteriorTurnRad))
            return false;

        if (!TryCircumcenter(p0, pm, p1, out var center, out double radius))
            return false;

        double a0 = Math.Atan2(p0.Y - center.y, p0.X - center.x);
        double am = Math.Atan2(pm.Y - center.y, pm.X - center.x);
        double a1 = Math.Atan2(p1.Y - center.y, p1.X - center.x);
        double ccwDelta = NormalizeCcw(a1 - a0);
        double ccwMid = NormalizeCcw(am - a0);
        bool clockwise;
        double totalDelta;

        if (ccwMid <= ccwDelta + 1e-12)
        {
            clockwise = false;
            totalDelta = ccwDelta;
        }
        else
        {
            clockwise = true;
            totalDelta = NormalizeCcw(a0 - a1);
        }

        if (totalDelta < 1e-9 || totalDelta > Math.PI * 2.0 - 1e-9)
            return false;

        double maxError = 0.0;
        double previousAlong = 0.0;
        for (int i = start; i <= end; i++)
        {
            var p = points[i];
            double angle = Math.Atan2(p.Y - center.y, p.X - center.x);
            double along = clockwise ? NormalizeCcw(a0 - angle) : NormalizeCcw(angle - a0);
            if (along + 1e-9 < previousAlong || along > totalDelta + 1e-9)
                return false;
            previousAlong = along;

            double radialError = Math.Abs(Hypot(p.X - center.x, p.Y - center.y) - radius);
            double zAtAngle = Lerp(p0.Z, p1.Z, totalDelta <= 0 ? 0.0 : along / totalDelta);
            double zError = Math.Abs(p.Z - zAtAngle);
            maxError = Math.Max(maxError, Hypot(radialError, zError));
            if (maxError > tolerance)
                return false;
        }

        if (!IsArcSpatiallyConservative(points, start, end, center, radius, clockwise, p0, p1, totalDelta, tolerance, out double spatialError))
            return false;

        maxError = Math.Max(maxError, spatialError);
        move = ToolMove.Arc(p0, p1, center, radius, clockwise, maxError);
        return true;
    }

    private static bool IsArcSpatiallyConservative(
        IReadOnlyList<Point3D> points,
        int start,
        int end,
        PointD center,
        double radius,
        bool clockwise,
        Point3D p0,
        Point3D p1,
        double totalDelta,
        double tolerance,
        out double maxSpatialError)
    {
        maxSpatialError = 0.0;
        double sourceLength = PolylineLength(points, start, end);
        if (sourceLength <= 1e-12)
            return false;

        if (radius > sourceLength * 8.0)
            return false;

        var bounds = Bounds(points, start, end);
        double margin = SpatialMargin(tolerance);

        int samples = Math.Clamp((end - start) * 4, 24, 128);
        double allowedCurveError = SpatialDriftTolerance(tolerance);
        double startAngle = Math.Atan2(p0.Y - center.y, p0.X - center.x);
        double signedSweep = clockwise ? -totalDelta : totalDelta;
        for (int i = 1; i < samples; i++)
        {
            double t = i / (double)samples;
            double angle = startAngle + signedSweep * t;
            var sample = new Point3D(
                center.x + radius * Math.Cos(angle),
                center.y + radius * Math.Sin(angle),
                Lerp(p0.Z, p1.Z, t));

            if (!InsideExpandedBounds(sample, bounds, margin))
                return false;
            double spatialError = DistanceToPolyline(sample, points, start, end);
            maxSpatialError = Math.Max(maxSpatialError, spatialError);
            if (spatialError > allowedCurveError)
                return false;
        }

        return true;
    }

    private static bool TryFitCubicBezier(
        IReadOnlyList<Point3D> points,
        int start,
        int end,
        double tolerance,
        out ToolMove move)
    {
        move = default;
        int count = end - start + 1;
        if (count < MinBezierPointCount)
            return false;

        if (HasSharpInteriorTurn(points, start, end, MaxBezierInteriorTurnRad))
            return false;

        var parameters = ChordLengthParameters(points, start, end);
        var p0 = points[start];
        var p3 = points[end];

        double c00 = 0.0;
        double c01 = 0.0;
        double c11 = 0.0;
        double rx1 = 0.0, ry1 = 0.0, rz1 = 0.0;
        double rx2 = 0.0, ry2 = 0.0, rz2 = 0.0;

        for (int local = 1; local < count - 1; local++)
        {
            double t = parameters[local];
            double mt = 1.0 - t;
            double b0 = mt * mt * mt;
            double b1 = 3.0 * mt * mt * t;
            double b2 = 3.0 * mt * t * t;
            double b3 = t * t * t;
            var p = points[start + local];
            double qx = p.X - (b0 * p0.X + b3 * p3.X);
            double qy = p.Y - (b0 * p0.Y + b3 * p3.Y);
            double qz = p.Z - (b0 * p0.Z + b3 * p3.Z);

            c00 += b1 * b1;
            c01 += b1 * b2;
            c11 += b2 * b2;
            rx1 += b1 * qx;
            ry1 += b1 * qy;
            rz1 += b1 * qz;
            rx2 += b2 * qx;
            ry2 += b2 * qy;
            rz2 += b2 * qz;
        }

        double determinant = c00 * c11 - c01 * c01;
        if (Math.Abs(determinant) < 1e-18)
            return false;

        var c1 = new Point3D(
            (rx1 * c11 - rx2 * c01) / determinant,
            (ry1 * c11 - ry2 * c01) / determinant,
            (rz1 * c11 - rz2 * c01) / determinant);
        var c2 = new Point3D(
            (c00 * rx2 - c01 * rx1) / determinant,
            (c00 * ry2 - c01 * ry1) / determinant,
            (c00 * rz2 - c01 * rz1) / determinant);

        double maxError = 0.0;
        for (int local = 0; local < count; local++)
        {
            var actual = points[start + local];
            var expected = EvaluateCubic(p0, c1, c2, p3, parameters[local]);
            double error = Distance(actual, expected);
            maxError = Math.Max(maxError, error);
            if (maxError > tolerance)
                return false;
        }

        if (!IsBezierSpatiallyConservative(points, start, end, p0, c1, c2, p3, tolerance, out double spatialError))
            return false;

        maxError = Math.Max(maxError, spatialError);
        move = ToolMove.CubicBezier(p0, c1, c2, p3, maxError);
        return true;
    }

    private static bool HasSharpInteriorTurn(
        IReadOnlyList<Point3D> points,
        int start,
        int end,
        double maxTurnRad)
    {
        for (int i = start + 1; i < end; i++)
        {
            var prev = points[i - 1];
            var current = points[i];
            var next = points[i + 1];
            double ax = current.X - prev.X;
            double ay = current.Y - prev.Y;
            double az = current.Z - prev.Z;
            double bx = next.X - current.X;
            double by = next.Y - current.Y;
            double bz = next.Z - current.Z;
            double al = Math.Sqrt(ax * ax + ay * ay + az * az);
            double bl = Math.Sqrt(bx * bx + by * by + bz * bz);
            if (al <= 1e-12 || bl <= 1e-12)
                continue;

            double dot = (ax * bx + ay * by + az * bz) / (al * bl);
            double turn = Math.Acos(Math.Clamp(dot, -1.0, 1.0));
            if (turn > maxTurnRad)
                return true;
        }

        return false;
    }

    private static bool IsBezierSpatiallyConservative(
        IReadOnlyList<Point3D> points,
        int start,
        int end,
        Point3D p0,
        Point3D c1,
        Point3D c2,
        Point3D p3,
        double tolerance,
        out double maxSpatialError)
    {
        maxSpatialError = 0.0;
        var bounds = Bounds(points, start, end);
        double margin = SpatialMargin(tolerance);
        if (!InsideExpandedBounds(c1, bounds, margin) || !InsideExpandedBounds(c2, bounds, margin))
            return false;

        int samples = Math.Clamp((end - start) * 4, 24, 128);
        double allowedCurveError = SpatialDriftTolerance(tolerance);
        for (int i = 1; i < samples; i++)
        {
            double t = i / (double)samples;
            var sample = EvaluateCubic(p0, c1, c2, p3, t);
            double spatialError = DistanceToPolyline(sample, points, start, end);
            maxSpatialError = Math.Max(maxSpatialError, spatialError);
            if (spatialError > allowedCurveError)
                return false;
        }

        return true;
    }

    private static double[] ChordLengthParameters(IReadOnlyList<Point3D> points, int start, int end)
    {
        int count = end - start + 1;
        var result = new double[count];
        double total = 0.0;
        for (int i = 1; i < count; i++)
        {
            total += Distance(points[start + i - 1], points[start + i]);
            result[i] = total;
        }

        if (total <= 1e-18)
            return result;

        for (int i = 1; i < count; i++)
            result[i] /= total;
        result[^1] = 1.0;
        return result;
    }

    private static Point3D EvaluateArc(ToolMove move, double t)
    {
        double startAngle = Math.Atan2(move.Start.Y - move.ArcCenter.y, move.Start.X - move.ArcCenter.x);
        double endAngle = Math.Atan2(move.End.Y - move.ArcCenter.y, move.End.X - move.ArcCenter.x);
        double sweep = move.Kind == ToolMoveKind.ArcCw
            ? -NormalizeCcw(startAngle - endAngle)
            : NormalizeCcw(endAngle - startAngle);
        double angle = startAngle + sweep * t;
        return new Point3D(
            move.ArcCenter.x + move.ArcRadius * Math.Cos(angle),
            move.ArcCenter.y + move.ArcRadius * Math.Sin(angle),
            Lerp(move.Start.Z, move.End.Z, t));
    }

    private static Point3D EvaluateCubic(Point3D p0, Point3D c1, Point3D c2, Point3D p3, double t)
    {
        double mt = 1.0 - t;
        double b0 = mt * mt * mt;
        double b1 = 3.0 * mt * mt * t;
        double b2 = 3.0 * mt * t * t;
        double b3 = t * t * t;
        return new Point3D(
            b0 * p0.X + b1 * c1.X + b2 * c2.X + b3 * p3.X,
            b0 * p0.Y + b1 * c1.Y + b2 * c2.Y + b3 * p3.Y,
            b0 * p0.Z + b1 * c1.Z + b2 * c2.Z + b3 * p3.Z);
    }

    private static bool TryCircumcenter(Point3D a, Point3D b, Point3D c, out PointD center, out double radius)
    {
        double ax = a.X;
        double ay = a.Y;
        double bx = b.X;
        double by = b.Y;
        double cx = c.X;
        double cy = c.Y;
        double d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
        if (Math.Abs(d) < 1e-18)
        {
            center = default;
            radius = 0.0;
            return false;
        }

        double ax2ay2 = ax * ax + ay * ay;
        double bx2by2 = bx * bx + by * by;
        double cx2cy2 = cx * cx + cy * cy;
        double ux = (ax2ay2 * (by - cy) + bx2by2 * (cy - ay) + cx2cy2 * (ay - by)) / d;
        double uy = (ax2ay2 * (cx - bx) + bx2by2 * (ax - cx) + cx2cy2 * (bx - ax)) / d;
        center = new PointD(ux, uy);
        radius = Hypot(ax - ux, ay - uy);
        return radius > 1e-12 && double.IsFinite(radius);
    }

    private static double MaxLineError(IReadOnlyList<Point3D> points, int start, int end)
    {
        var a = points[start];
        var b = points[end];
        double abx = b.X - a.X;
        double aby = b.Y - a.Y;
        double abz = b.Z - a.Z;
        double ab2 = abx * abx + aby * aby + abz * abz;
        double max = 0.0;

        for (int i = start + 1; i < end; i++)
        {
            var p = points[i];
            double error;
            if (ab2 < 1e-24)
            {
                error = Distance(a, p);
            }
            else
            {
                double apx = p.X - a.X;
                double apy = p.Y - a.Y;
                double apz = p.Z - a.Z;
                double t = Math.Clamp((apx * abx + apy * aby + apz * abz) / ab2, 0.0, 1.0);
                var projected = new Point3D(a.X + t * abx, a.Y + t * aby, a.Z + t * abz);
                error = Distance(p, projected);
            }

            max = Math.Max(max, error);
        }

        return max;
    }

    private static double NormalizeCcw(double angle)
    {
        double result = angle % (Math.PI * 2.0);
        if (result < 0) result += Math.PI * 2.0;
        return result;
    }

    private static double Distance(Point3D a, Point3D b)
    {
        double dx = a.X - b.X;
        double dy = a.Y - b.Y;
        double dz = a.Z - b.Z;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static double DistanceToPolyline(
        Point3D point,
        IReadOnlyList<Point3D> polyline,
        int start,
        int end)
    {
        double best = double.PositiveInfinity;
        for (int i = start + 1; i <= end; i++)
            best = Math.Min(best, DistanceToSegment(point, polyline[i - 1], polyline[i]));
        return best;
    }

    private static double PolylineLength(IReadOnlyList<Point3D> polyline, int start, int end)
    {
        double length = 0.0;
        for (int i = start + 1; i <= end; i++)
            length += Distance(polyline[i - 1], polyline[i]);
        return length;
    }

    private static double DistanceToSegment(Point3D point, Point3D a, Point3D b)
    {
        double abx = b.X - a.X;
        double aby = b.Y - a.Y;
        double abz = b.Z - a.Z;
        double ab2 = abx * abx + aby * aby + abz * abz;
        if (ab2 <= 1e-24)
            return Distance(point, a);

        double apx = point.X - a.X;
        double apy = point.Y - a.Y;
        double apz = point.Z - a.Z;
        double t = Math.Clamp((apx * abx + apy * aby + apz * abz) / ab2, 0.0, 1.0);
        return Distance(point, new Point3D(a.X + abx * t, a.Y + aby * t, a.Z + abz * t));
    }

    private static Bounds3D Bounds(IReadOnlyList<Point3D> points, int start, int end)
    {
        double minX = double.PositiveInfinity;
        double minY = double.PositiveInfinity;
        double minZ = double.PositiveInfinity;
        double maxX = double.NegativeInfinity;
        double maxY = double.NegativeInfinity;
        double maxZ = double.NegativeInfinity;

        for (int i = start; i <= end; i++)
        {
            var p = points[i];
            minX = Math.Min(minX, p.X);
            minY = Math.Min(minY, p.Y);
            minZ = Math.Min(minZ, p.Z);
            maxX = Math.Max(maxX, p.X);
            maxY = Math.Max(maxY, p.Y);
            maxZ = Math.Max(maxZ, p.Z);
        }

        return new Bounds3D(minX, minY, minZ, maxX, maxY, maxZ);
    }

    private static bool InsideExpandedBounds(Point3D point, Bounds3D bounds, double margin) =>
        point.X >= bounds.MinX - margin
        && point.X <= bounds.MaxX + margin
        && point.Y >= bounds.MinY - margin
        && point.Y <= bounds.MaxY + margin
        && point.Z >= bounds.MinZ - margin
        && point.Z <= bounds.MaxZ + margin;

    private static double SpatialDriftTolerance(double tolerance) =>
        Math.Max(tolerance * SpatialDriftToleranceMultiplier, 1e-9);

    private static double SpatialMargin(double tolerance) =>
        Math.Max(tolerance * BoundsToleranceMultiplier, 1e-9);

    private static double Hypot(double x, double y) => Math.Sqrt(x * x + y * y);

    private static double Lerp(double a, double b, double t) => a + (b - a) * t;

    private static int ArcSampleCount(ToolMove move, double chordTolerance)
    {
        double startAngle = Math.Atan2(move.Start.Y - move.ArcCenter.y, move.Start.X - move.ArcCenter.x);
        double endAngle = Math.Atan2(move.End.Y - move.ArcCenter.y, move.End.X - move.ArcCenter.x);
        double sweep = move.Kind == ToolMoveKind.ArcCw
            ? NormalizeCcw(startAngle - endAngle)
            : NormalizeCcw(endAngle - startAngle);
        if (sweep <= 1e-12 || move.ArcRadius <= 1e-12)
            return 1;

        double maxAngle = 2.0 * Math.Acos(Math.Clamp(1.0 - chordTolerance / move.ArcRadius, -1.0, 1.0));
        if (maxAngle <= 1e-6 || double.IsNaN(maxAngle))
            maxAngle = Math.PI / 32.0;
        return Math.Clamp((int)Math.Ceiling(sweep / maxAngle), 1, 256);
    }

    private static int BezierSampleCount(ToolMove move, double chordTolerance)
    {
        double controlLength = Distance(move.Start, move.Control1)
            + Distance(move.Control1, move.Control2)
            + Distance(move.Control2, move.End);
        double chordLength = Distance(move.Start, move.End);
        double excess = Math.Max(0.0, controlLength - chordLength);
        return Math.Clamp((int)Math.Ceiling(Math.Sqrt(excess / chordTolerance)) * 4, 8, 128);
    }

    private static bool SamePoint(Point3D a, Point3D b) =>
        Math.Abs(a.X - b.X) <= 1e-12
        && Math.Abs(a.Y - b.Y) <= 1e-12
        && Math.Abs(a.Z - b.Z) <= 1e-12;

    private readonly record struct Candidate(int EndIndex, ToolMove Move)
    {
        public bool IsValid => EndIndex > 0;
    }

    private readonly record struct Bounds3D(
        double MinX,
        double MinY,
        double MinZ,
        double MaxX,
        double MaxY,
        double MaxZ);
}