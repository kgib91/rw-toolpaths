using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling.Certification;

// The target is the mathematical UNION of separately retained
// signed CENTER and every full convex unresolved H. Boolean output is merely
// an OUTER proposal. No summed winding, Boolean center containment, or extra
// center inflation grants enclosure credit.
internal static class PotentialCenterFootprint
{

    internal static bool Certify(Paths64 originalCenters, IReadOnlyList<Path64> hulls,
        Paths64 footprint, double cutterRadius, CancellationToken token, Action checkpoint,
        out int chords, out int enclosedVertices)
    {
        chords = 0; enclosedVertices = 0;
        token.ThrowIfCancellationRequested(); checkpoint();
        if (!double.IsFinite(cutterRadius) || cutterRadius <= 0 || footprint.Count == 0
            || hulls.Count > 4096 || originalCenters.Sum(p => p.Count) > 500_000
            || footprint.Sum(p => p.Count) > 500_000
            || originalCenters.Concat(footprint).Concat(hulls).SelectMany(p => p).Any(p => !InRange(p))
            || hulls.Any(h => !PositiveConvex(h))) return false;
        checkpoint();
        var stock = new PolygonOps.BoundarySegmentIndex(footprint);
        var centers = new PolygonOps.BoundarySegmentIndex(originalCenters);
        checkpoint();
        // Same literal net physical radius + one arithmetic guard tick as the
        // frozen production gate. ContainsCapsule adds four ticks internally.
        double radiusArgument = cutterRadius * PathUtils.Scale - 3;
        foreach (var path in originalCenters.Concat(hulls))
        for (int i = 0; i < path.Count; i++)
        {
            if ((++chords & 63) == 0) { token.ThrowIfCancellationRequested(); checkpoint(); }
            if (!stock.ContainsCapsule(path[i], path[(i + 1) % path.Count], radiusArgument)) return false;
        }
        var vertices = footprint.SelectMany(p => p).OrderBy(p => p.X).ThenBy(p => p.Y).ToArray();
        for (int i = 0; i < vertices.Length; i++)
        {
            if ((i & 255) == 0) { token.ThrowIfCancellationRequested(); checkpoint(); }
            if (centers.IsInside(vertices[i])) { enclosedVertices++; return false; }
        }
        // Every H is an independently checked convex positive polygon. A
        // footprint boundary wholly swallowed by ANY union constituent is
        // forbidden. Inclusive exact halfplanes block tangency too. Bounds
        // merely nominate vertices; they never authorize stock credit.
        int work = 0;
        foreach (var hull in hulls)
        {
            token.ThrowIfCancellationRequested(); checkpoint();
            long minX = hull.Min(p => p.X), maxX = hull.Max(p => p.X);
            long minY = hull.Min(p => p.Y), maxY = hull.Max(p => p.Y);
            for (int i = LowerBound(vertices, minX); i < vertices.Length && vertices[i].X <= maxX; i++)
            {
                if ((++work & 255) == 0) { token.ThrowIfCancellationRequested(); checkpoint(); }
                var p = vertices[i];
                if (p.Y < minY || p.Y > maxY) continue;
                if (ContainsConvex(hull, p)) { enclosedVertices++; return false; }
            }
        }
        checkpoint(); return true;
    }

    private static bool InRange(Point64 p) => Math.Abs((double)p.X) < 1L << 30
        && Math.Abs((double)p.Y) < 1L << 30;

    private static Int128 Cross(Point64 a, Point64 b, Point64 p)
    {
        // Every coordinate is bounded before calling. Differences <2^31,
        // products <2^62, subtraction <2^63: exact Int128 with ample range.
        return (Int128)(b.X - a.X) * (p.Y - a.Y) - (Int128)(b.Y - a.Y) * (p.X - a.X);
    }

    private static bool PositiveConvex(Path64 hull)
    {
        if (hull.Count < 3 || hull.Count > 32 || hull.Any(p => !InRange(p))
            || hull.Distinct().Count() != hull.Count) return false;
        bool positive = false;
        for (int i = 0; i < hull.Count; i++)
        foreach (var p in hull)
        {
            var cross = Cross(hull[i], hull[(i + 1) % hull.Count], p);
            if (cross < 0) return false;
            positive |= cross > 0;
        }
        return positive;
    }

    private static bool ContainsConvex(Path64 hull, Point64 p)
    {
        for (int i = 0; i < hull.Count; i++)
            if (Cross(hull[i], hull[(i + 1) % hull.Count], p) < 0) return false;
        return true;
    }

    private static int LowerBound(Point64[] vertices, long x)
    {
        int left = 0, right = vertices.Length;
        while (left < right)
        { int middle = left + (right - left) / 2; if (vertices[middle].X < x) left = middle + 1; else right = middle; }
        return left;
    }

}
