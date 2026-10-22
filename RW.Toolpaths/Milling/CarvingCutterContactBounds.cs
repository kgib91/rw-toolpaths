using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Analytic physical cutter contact against points and height-bounded
/// source rectangles, keeping sloped tip motion and both reserved axes correlated.</summary>
public static class CarvingCutterContactBounds
{
    /// <summary>Lowest tip Z plus physical cutter underside over an entire sloped
    /// chord at a fixed source point. The XY reserve is a square, matching independent
    /// rounding of the two machine axes. Positive infinity means no contact.</summary>
    public static double MinimumTipAndBottom(CarvingCutterProfile cutter,
        PointD protectedPoint, PointD start, double startZ, PointD end, double endZ,
        double horizontalReserve)
        => MinimumTipAndBottom(cutter, protectedPoint, start, startZ, end, endZ,
            horizontalReserve, horizontalReserve);

    // The parameter is only a nominee. Callers must evaluate their own physical
    // predicate at this pose; a missing or rounded contact is not negative proof.
    internal static double MinimumTipAndBottom(CarvingCutterProfile cutter,
        PointD protectedPoint, PointD start, double startZ, PointD end, double endZ,
        double horizontalReserve, out double parameter)
        => MinimumTipAndBottom(cutter, protectedPoint, start, startZ, end, endZ,
            horizontalReserve, horizontalReserve, out parameter);

    private static double MinimumTipAndBottom(CarvingCutterProfile cutter,
        PointD protectedPoint, PointD start, double startZ, PointD end, double endZ,
        double reserveX, double reserveY)
        => MinimumTipAndBottom(cutter, protectedPoint, start, startZ, end, endZ,
            reserveX, reserveY, out _);

    private static double MinimumTipAndBottom(CarvingCutterProfile cutter,
        PointD protectedPoint, PointD start, double startZ, PointD end, double endZ,
        double reserveX, double reserveY, out double parameter)
    {
        ArgumentNullException.ThrowIfNull(cutter);
        if (!double.IsFinite(protectedPoint.x) || !double.IsFinite(protectedPoint.y)
            || !double.IsFinite(start.x) || !double.IsFinite(start.y) || !double.IsFinite(startZ)
            || !double.IsFinite(end.x) || !double.IsFinite(end.y) || !double.IsFinite(endZ)
            || !double.IsFinite(reserveX) || reserveX < 0 || !double.IsFinite(reserveY) || reserveY < 0)
            throw new ArgumentException("A cutter contact chord requires finite coordinates and a nonnegative reserve.");

        double dx = end.x - start.x, dy = end.y - start.y, dz = endZ - startZ;
        double sx = protectedPoint.x - start.x, sy = protectedPoint.y - start.y;
        double minimum = double.PositiveInfinity;
        double selectedParameter = double.NaN;
        void Record(double t, double value)
        {
            // Retain the original Math.Min operation, including NaN and tied
            // values. The first strictly improving pose is sufficient to nominate.
            if (value < minimum) selectedParameter = t;
            minimum = Math.Min(minimum, value);
        }
        void Add(double t)
        {
            double rx = Math.Max(0, Math.Abs(sx - t * dx) - reserveX);
            double ry = Math.Max(0, Math.Abs(sy - t * dy) - reserveY);
            double radius = Math.Sqrt(rx * rx + ry * ry);
            // A radial-domain intersection can round a few ulps outside its disk.
            // This allowance is far below the existing contact upper-bound reserve.
            double radialRoundoff = 1e-12 * Math.Max(1, cutter.MaximumRadius);
            if (radius > cutter.MaximumRadius + radialRoundoff) return;
            double bottom = cutter.Bottom(Math.Min(radius, cutter.MaximumRadius));
            Record(t, startZ + t * dz + bottom);
        }

        // On each interval the distance from the source to the reserved center
        // square is the norm of an affine vector. Its active axes change only here.
        Span<double> breaks = stackalloc double[6];
        int count = 2;
        breaks[0] = 0; breaks[1] = 1;
        void Break(double source, double delta, double reserve, Span<double> values, ref int size)
        {
            if (delta == 0) return;
            double t0 = (source - reserve) / delta;
            double t1 = (source + reserve) / delta;
            if (t0 > 0 && t0 < 1) values[size++] = t0;
            if (t1 > 0 && t1 < 1) values[size++] = t1;
        }
        Break(sx, dx, reserveX, breaks, ref count); Break(sy, dy, reserveY, breaks, ref count);
        breaks[..count].Sort();
        for (int interval = 1; interval < count; interval++)
        {
            double lo = breaks[interval - 1], hi = breaks[interval];
            if (lo == hi) continue;
            Add(lo); Add(hi);
            double midpoint = (lo + hi) / 2;
            double px = sx - midpoint * dx, py = sy - midpoint * dy;
            double vx = dx, vy = dy;
            if (Math.Abs(px) <= reserveX) { px = 0; vx = 0; }
            else px -= Math.CopySign(reserveX, px);
            if (Math.Abs(py) <= reserveY) { py = 0; vy = 0; }
            else py -= Math.CopySign(reserveY, py);
            double length2 = vx * vx + vy * vy;
            if (length2 == 0) continue; // Constant radius, linear tip Z.
            double nearest = (px * vx + py * vy) / length2;
            double cross = px * vy - py * vx;
            double across2 = cross * cross / length2;
            void Candidate(double offset, double? radialBoundary = null)
            {
                double t = midpoint + offset;
                if (t < lo || t > hi) return;
                if (radialBoundary is { } boundary)
                {
                    // Evaluate the exact domain junction, rather than recomputing
                    // a radius a few ulps above a discontinuous disk underside.
                    Record(t, startZ + t * dz + cutter.Bottom(boundary));
                }
                else Add(t);
            }
            void Radius(double radius)
            {
                if (radius < 0 || radius > cutter.MaximumRadius) return;
                double difference = radius * radius - across2;
                if (difference < 0) return;
                double offset = Math.Sqrt(difference / length2);
                Candidate(nearest - offset, radius); Candidate(nearest + offset, radius);
            }
            Candidate(nearest);
            Radius(cutter.MaximumRadius);
            double sectionZ = 0;
            foreach (var section in cutter.Sections)
            {
                if (sectionZ > cutter.MaximumContactHeight) break;
                double height = Math.Min(section.Height, cutter.MaximumContactHeight - sectionZ);
                Radius(section.BottomRadius);
                if (section.Kind == CarvingCutterSectionKind.Hemisphere)
                {
                    double r = section.TopRadius;
                    Radius(Math.Sqrt(Math.Max(0, height * (2 * r - height))));
                    double difference = r * r - across2;
                    if (difference >= 0)
                        Candidate(nearest - dz * Math.Sqrt(difference / (length2 * (length2 + dz * dz))));
                }
                else
                {
                    double outer = section.BottomRadius
                        + (section.TopRadius - section.BottomRadius) * height / section.Height;
                    Radius(outer);
                    if (section.TopRadius > section.BottomRadius)
                    {
                        double slope = section.Height / (section.TopRadius - section.BottomRadius);
                        double denominator = slope * slope * length2 - dz * dz;
                        if (denominator > 0)
                            Candidate(nearest - dz * Math.Sqrt(across2 / (length2 * denominator)));
                    }
                }
                sectionZ += section.Height;
            }
        }
        parameter = double.IsFinite(minimum) ? selectedParameter : double.NaN;
        return minimum;
    }

    /// <summary>Conservative additional tip lift needed by every point of a sloped
    /// chord over a rectangular source patch bounded by <paramref name="height"/>.</summary>
    public static double ChordLiftUpperBound(CarvingCutterProfile cutter,
        double x0, double y0, double x1, double y1, double height,
        PointD start, double startZ, PointD end, double endZ, double horizontalReserve)
    {
        ArgumentNullException.ThrowIfNull(cutter);
        if (!double.IsFinite(x0) || !double.IsFinite(y0) || !double.IsFinite(x1) || !double.IsFinite(y1)
            || x1 < x0 || y1 < y0 || !double.IsFinite(height)
            || !double.IsFinite(horizontalReserve) || horizontalReserve < 0)
            throw new ArgumentException("A cutter contact patch requires finite ordered bounds and a nonnegative reserve.");
        // Minimize over the rectangle and reserved cutter axis together. Their
        // independent axis extents add exactly; for each chord parameter, the
        // resulting distance is the norm of one affine, axis-clipped vector.
        // Keeping that parameter correlated with tip Z eliminates the fixed gap
        // left by relief tangent planes at a reserved, oblique sphere-equator contact.
        var center = new PointD(x0 + (x1 - x0) / 2, y0 + (y1 - y0) / 2);
        double minimum = MinimumTipAndBottom(cutter, center, start, startZ, end, endZ,
            horizontalReserve + (x1 - x0) / 2, horizontalReserve + (y1 - y0) / 2);
        return height - minimum + 1e-8;
    }
}
