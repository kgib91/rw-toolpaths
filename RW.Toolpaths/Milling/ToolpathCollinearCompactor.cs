namespace RW.Toolpaths.Milling;

/// <summary>
/// Removes subdivision points only when both the native trace and the posted
/// float/F3 trace follow the same directed straight chord. The commanded cutter
/// poses, and therefore removed stock at every height, are unchanged.
/// </summary>
internal static class ToolpathCollinearCompactor
{
    private readonly record struct GridPoint(long X, long Y, long Z);

    internal static TaggedToolpath Compact(TaggedToolpath path, double surfaceZ,
        CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var source = path.Points;
        if (source.Count < 3 || path.DetectCurves) return path;
        if (path.Spans.Any(span => span.StartIndex < 0 || span.EndIndex < span.StartIndex
            || span.EndIndex >= source.Count || span.Kind is ToolpathSpanKind.TabLift or ToolpathSpanKind.Retract)) return path;

        // A span boundary can change feed or cutting authority, even when its
        // adjacent geometric chords are collinear. Keep every original boundary.
        var boundaries = path.Spans.SelectMany(span => new[] { span.StartIndex, span.EndIndex })
            .Distinct().OrderBy(index => index).ToArray();
        List<Point3D>? points = null;
        List<int>? ordinals = null;
        for (int i = 2; i < source.Count; i++)
        {
            if ((i & 1023) == 0) cancellationToken.ThrowIfCancellationRequested();
            var candidate = source[i];
            while ((points?.Count ?? i) >= 2)
            {
                int count = points?.Count ?? i;
                var first = points is null ? source[count - 2] : points[count - 2];
                var middle = points is null ? source[count - 1] : points[count - 1];
                int firstOrdinal = ordinals is null ? count - 2 : ordinals[count - 2];
                int middleOrdinal = ordinals is null ? count - 1 : ordinals[count - 1];
                // The first nominal point is not an emitted XYZ command. A
                // previous Y-only command also need not reach its nominal XY.
                // Retain the next explicit XYZ endpoint before combining chords;
                // its actual posted position then supplies the straight-line proof.
                int incomingMode = firstOrdinal == 0 ? -1
                    : MotionMode(source[firstOrdinal - 1], source[firstOrdinal], surfaceZ);
                int outgoingMode = MotionMode(first, middle, surfaceZ);
                if (incomingMode is < 0 or > 1 || outgoingMode is < 0 or > 1
                    || Array.BinarySearch(boundaries, middleOrdinal) >= 0
                    || !NativeBetween(first, middle, candidate)
                    || !TryGrid(first, surfaceZ, out var a)
                    || !TryGrid(middle, surfaceZ, out var b)
                    || !TryGrid(candidate, surfaceZ, out var c)
                    || !GridBetween(a, b, c)
                    || !SameMotionMode(first, middle, candidate, surfaceZ)) break;

                if (points is null)
                {
                    points = new List<Point3D>(source.Count);
                    ordinals = new List<int>(source.Count);
                    for (int copy = 0; copy < i; copy++)
                    {
                        points.Add(source[copy]);
                        ordinals.Add(copy);
                    }
                }
                points.RemoveAt(points.Count - 1);
                ordinals!.RemoveAt(ordinals.Count - 1);
            }
            if (points is not null)
            {
                points.Add(candidate);
                ordinals!.Add(i);
            }
        }
        if (points is null) return path;
        var spans = path.Spans.Select(span => new ToolpathSpan(
            ordinals!.BinarySearch(span.StartIndex), ordinals.BinarySearch(span.EndIndex), span.Kind)).ToArray();
        if (spans.Any(span => span.StartIndex < 0 || span.EndIndex < 0))
            throw new InvalidOperationException("Collinear compaction lost a toolpath span boundary.");
        return new TaggedToolpath(points, spans, path.RegionIndex, path.Category,
            path.DepthPassIndex, path.VisitIdentity, detectCurves: false)
            {ContinuationComponentIndex=path.ContinuationComponentIndex,ContinuationPlan=path.ContinuationPlan,
                ContinuationSourceOrdinal=path.ContinuationSourceOrdinal,ConcentricSource=path.ConcentricSource};
    }

    private static bool NativeBetween(Point3D a, Point3D b, Point3D c)
    {
        if (!double.IsFinite(a.X + a.Y + a.Z + b.X + b.Y + b.Z + c.X + c.Y + c.Z)) return false;
        double dx = c.X - a.X, dy = c.Y - a.Y, dz = c.Z - a.Z;
        double largest = Math.Max(Math.Abs(dx), Math.Max(Math.Abs(dy), Math.Abs(dz)));
        if (largest == 0) return a == b;
        double t = Math.Abs(dx) == largest ? (b.X - a.X) / dx
            : Math.Abs(dy) == largest ? (b.Y - a.Y) / dy : (b.Z - a.Z) / dz;
        if (t < 0 || t > 1) return false;
        // Only floating arithmetic noise around a native straight line is
        // permitted here. Command-space collinearity below is exact.
        const double roundingUnits = 32 * 2.2204460492503131e-16;
        static bool Same(double value, double expected, double start, double end)
            => Math.Abs(value - expected) <= roundingUnits * Math.Max(1,
                Math.Max(Math.Abs(value), Math.Max(Math.Abs(start), Math.Abs(end))));
        return Same(b.X, a.X + t * dx, a.X, c.X)
            && Same(b.Y, a.Y + t * dy, a.Y, c.Y)
            && Same(b.Z, a.Z + t * dz, a.Z, c.Z);
    }

    private static bool TryGrid(Point3D point, double surfaceZ, out GridPoint result)
    {
        float x = (float)point.X, y = (float)point.Y, z = (float)point.Z - (float)surfaceZ;
        // At this bound binary32*1000 is exactly representable in binary64;
        // nearest-even integer rounding matches Single.ToString("F3"). Keep
        // extreme/nonfinite coordinates unchanged rather than approximate them.
        if (!float.IsFinite(x) || !float.IsFinite(y) || !float.IsFinite(z)
            || Math.Abs(x) > 1e9 || Math.Abs(y) > 1e9 || Math.Abs(z) > 1e9)
        { result = default; return false; }
        result = new GridPoint((long)Math.Round((double)x * 1000, MidpointRounding.ToEven),
            (long)Math.Round((double)y * 1000, MidpointRounding.ToEven),
            (long)Math.Round((double)z * 1000, MidpointRounding.ToEven));
        return true;
    }

    private static bool GridBetween(GridPoint a, GridPoint b, GridPoint c)
    {
        long lx = c.X - a.X, ly = c.Y - a.Y, lz = c.Z - a.Z;
        long mx = b.X - a.X, my = b.Y - a.Y, mz = b.Z - a.Z;
        const long fastBound = 1_000_000_000;
        if (Math.Abs(lx) <= fastBound && Math.Abs(ly) <= fastBound && Math.Abs(lz) <= fastBound
            && Math.Abs(mx) <= fastBound && Math.Abs(my) <= fastBound && Math.Abs(mz) <= fastBound)
        {
            // Three products bounded by 1e18 fit in Int64. Keep exact Int128
            // fallback for larger coordinates; ordinary jobs avoid its cost.
            if (mx * ly != my * lx || mx * lz != mz * lx || my * lz != mz * ly) return false;
            if (lx == 0 && ly == 0 && lz == 0) return mx == 0 && my == 0 && mz == 0;
            long scalar = mx * lx + my * ly + mz * lz;
            return scalar >= 0 && scalar <= lx * lx + ly * ly + lz * lz;
        }
        Int128 dx = c.X - a.X, dy = c.Y - a.Y, dz = c.Z - a.Z;
        Int128 bx = b.X - a.X, by = b.Y - a.Y, bz = b.Z - a.Z;
        if (bx * dy != by * dx || bx * dz != bz * dx || by * dz != bz * dy) return false;
        if (dx == 0 && dy == 0 && dz == 0) return bx == 0 && by == 0 && bz == 0;
        Int128 dot = bx * dx + by * dy + bz * dz;
        return dot >= 0 && dot <= dx * dx + dy * dy + dz * dz;
    }

    private static bool SameMotionMode(Point3D a, Point3D b, Point3D c, double surfaceZ)
    {
        // The desktop post classifies its float world chords before F3:
        // ToolpathMotionClassifier's .001 mm vertical/planar/surface thresholds
        // and IlProgramBuilder's partial-axis vertical commands are part of this
        // conversion contract. Do not turn a sequence of XYZ commands into a
        // Y-only plunge/retract, or combine an air rapid with a cutting feed.
        int mode = MotionMode(a, b, surfaceZ);
        return mode is 0 or 1 && mode == MotionMode(b, c, surfaceZ) && mode == MotionMode(a, c, surfaceZ);
    }

    private static int MotionMode(Point3D from, Point3D to, double surface)
    {
        float ax = (float)from.X, ay = (float)from.Y, az = (float)from.Z;
        float bx = (float)to.X, by = (float)to.Y, bz = (float)to.Z;
        float dx = bx - ax, dy = by - ay, dz = bz - az;
        float run = MathF.Sqrt(dx * dx + dy * dy);
        if (!float.IsFinite(ax) || !float.IsFinite(ay) || !float.IsFinite(az)
            || !float.IsFinite(bx) || !float.IsFinite(by) || !float.IsFinite(bz)
            || !float.IsFinite(run) || !float.IsFinite(dz) || !float.IsFinite((float)surface)) return -1;
        if (run > .001f && MathF.Min(az, bz) >= (float)surface - .001f) return 1; // XYZ rapid.
        if (MathF.Abs(dz) > .001f && run <= .001f)
            return dz > 0 ? 2 : bz >= (float)surface ? 3 : 4; // Y-only retract, rapid, plunge.
        return 0;
    }
}
