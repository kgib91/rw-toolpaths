using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Optional negative authority for one linear physical-profile interval.
/// A certified unit direction supplies a Cauchy lower bound over the whole chord.
/// The approximate optimizer only selects that direction; it supplies no proof.</summary>
internal static class VCarveAffineRadiusBounds
{
    internal const double MaximumMagnitude = 10_000;
    internal const double MaximumSlope = 10_000;
    internal const double MinimumSectionHeight = .0001;
    internal const double JunctionReserve = 1e-8;
    internal const double EvaluationReserve = 1e-5;

    internal static bool TryLowerBound(PointD q, Point3D a, Point3D b,
        VCarveCutterProfile profile, out double lower)
    {
        lower = double.NegativeInfinity;
        if (!Ordinary(q.x) || !Ordinary(q.y)
            || !Ordinary(a.X) || !Ordinary(a.Y) || !Ordinary(a.Z)
            || !Ordinary(b.X) || !Ordinary(b.Y) || !Ordinary(b.Z)
            || profile.Sections.Count is < 1 or > 8) return false;

        double minDepth = Math.Min(a.Z, b.Z), maxDepth = Math.Max(a.Z, b.Z);
        double sectionZ = 0;
        bool linearInterval = false;
        foreach (var section in profile.Sections)
        {
            if (!Ordinary(section.Height) || section.Height < MinimumSectionHeight
                || !Ordinary(section.BottomRadius) || !Ordinary(section.TopRadius)) return false;
            double endZ = sectionZ + section.Height;
            if (!Ordinary(endZ)) return false;
            if (minDepth > sectionZ + JunctionReserve && maxDepth < endZ - JunctionReserve)
            {
                if (section.Kind != CarvingCutterSectionKind.Linear) return false;
                // Bound the exact slope of the same rounded radial difference
                // used by PhysicalRadiusAtDepth, rather than a nominal cap.
                double slopeUpper = Math.BitIncrement(Math.Abs(section.TopRadius - section.BottomRadius) / section.Height);
                if (!double.IsFinite(slopeUpper) || slopeUpper > MaximumSlope) return false;
                linearInterval = true;
            }
            sectionZ = endZ;
        }
        if (!linearInterval) return false;

        double r0 = profile.PhysicalRadiusAtDepth(a.Z), r1 = profile.PhysicalRadiusAtDepth(b.Z);
        if (!Ordinary(r0) || !Ordinary(r1)) return false;
        double vx = b.X - a.X, vy = b.Y - a.Y, v2 = vx * vx + vy * vy;
        if (!(v2 > 0) || !double.IsFinite(v2)) return false;
        double wx = a.X - q.x, wy = a.Y - q.y, dr = r1 - r0;
        double t;
        if (dr * dr >= v2) t = dr >= 0 ? 1 : 0;
        else
        {
            double cross = wx * vy - wy * vx;
            double dot = wx * vx + wy * vy;
            t = (dr * Math.Abs(cross) / Math.Sqrt(v2 - dr * dr) - dot) / v2;
            if (!double.IsFinite(t)) return false;
            t = Math.Clamp(t, 0, 1);
        }

        double px = a.X + vx * t - q.x, py = a.Y + vy * t - q.y;
        double length = Math.Sqrt(px * px + py * py);
        if (!(length > 0) || !double.IsFinite(length)) return false;
        // Shrinking normally makes the following outward test inexpensive. Only
        // that test, not the normalization or stationary formula, certifies n.
        const double shrink = 1 - 1e-12;
        double nx = px / length * shrink, ny = py / length * shrink;
        double normUpper = Math.BitIncrement(Math.BitIncrement(nx * nx) + Math.BitIncrement(ny * ny));
        if (!double.IsFinite(normUpper) || normUpper > 1) return false;

        // Within the guarded section, the ideal physical radius is the maximum
        // of one affine radius and constants from completed sections. That is
        // convex, so the secant through endpoint radii is an upper envelope.
        // For any certified |n|<=1, |P(t)-q| >= n.(P(t)-q); subtracting that
        // secant leaves an affine function whose minimum is at an endpoint.
        //
        // Cover the literal binary Value arithmetic, not just this ideal model.
        // With u=2^-52, S=1e4 and slope<=1e4: At's three operations contribute
        // <=8uS per axis/depth; subtraction, norm and the endpoint dot contribute
        // <64uS; physical radius multiply/divide/add and its depth perturbation,
        // plus both endpoint radius errors, contribute <32uS(1+slope).
        // Even the looser combined 100uS(1+slope) is <2.23e-6mm. The 1e-5mm
        // deduction covers those errors, underflow, and final subtraction.
        // JunctionReserve exceeds 32uS and keeps every literal interpolated depth
        // within this section. Completed sections are the same fixed constants
        // at both endpoints and every pose. Any failed premise returns no bound.
        double first = nx * (a.X - q.x) + ny * (a.Y - q.y) - r0;
        double last = nx * (b.X - q.x) + ny * (b.Y - q.y) - r1;
        lower = Math.BitDecrement(Math.Min(first, last) - EvaluationReserve);
        if (!double.IsFinite(lower)) { lower = double.NegativeInfinity; return false; }
        return true;
    }

    private static bool Ordinary(double value) => double.IsFinite(value) && Math.Abs(value) <= MaximumMagnitude;
}
