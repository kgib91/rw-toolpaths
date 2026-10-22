using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Negative-only enclosure of first-hemisphere disc witnesses by a
/// sphere around the source point. Approximate projection selects a direction;
/// only its outward-certified unit norm and the enclosure supply authority.</summary>
internal static class VCarveSphereDiscBounds
{
    internal const double UnitRoundoff = 2.220446049250313e-16; // 2^-52, deliberately twice the usual unit roundoff.
    internal const double MaximumMagnitude = 10_000;
    internal const double MaximumSphereRadius = 100;
    internal const double GeometryReserve = 1e-8;
    private const double ValueRoundoff = 128 * UnitRoundoff * MaximumMagnitude;

    internal static bool TryBounds(PointD q, Point3D a, Point3D b,
        VCarveCutterProfile profile, double desired, out double lowerDistance, out double enclosingRadius)
    {
        lowerDistance = double.NegativeInfinity;
        enclosingRadius = double.PositiveInfinity;
        if (!Ordinary(q.x) || !Ordinary(q.y) || !Ordinary(a.X) || !Ordinary(a.Y)
            || !Ordinary(b.X) || !Ordinary(b.Y) || !Ordinary(desired)
            || !double.IsFinite(a.Z) || !double.IsFinite(b.Z) || a.Z < 0 || b.Z < 0) return false;
        var sections = profile.PhysicalProfile.Sections;
        if (sections.Length == 0) return false;
        var first = sections[0];
        if (first.Kind != CarvingCutterSectionKind.Hemisphere || first.BottomRadius != 0
            || first.Height != first.TopRadius || !(first.TopRadius > 0) || first.TopRadius > MaximumSphereRadius) return false;
        double radius = first.TopRadius, minDepth = Math.Min(a.Z, b.Z), maxDepth = Math.Max(a.Z, b.Z);
        // Literal At depth differs from exact endpoint interpolation by <=8uD.
        // Keeping the upper end away from R also keeps later sections inactive.
        double depthError = Up(8 * UnitRoundoff * maxDepth);
        double lowDepth = Math.Max(0, Down(minDepth - depthError)), highDepth = Up(maxDepth + depthError);
        if (!(highDepth < radius - GeometryReserve)) return false;

        // Real hemispherical radius is increasing on [0,R]. Directed endpoint
        // expressions bound it; +/-4uR additionally enclose literal multiply,
        // subtract and sqrt roundoff, including nonmonotone one-ulp variations.
        double twiceRadius = 2 * radius;
        double radiusRoundoff = Up(4 * UnitRoundoff * radius);
        double minSquared = Math.Max(0, Down(lowDepth * Down(twiceRadius - lowDepth)));
        double maxSquared = Up(highDepth * Up(twiceRadius - highDepth));
        double minRadius = Math.Max(0, Down(Down(Math.Sqrt(minSquared)) - radiusRoundoff));
        double maxRadius = Up(Up(Math.Sqrt(maxSquared)) + radiusRoundoff);
        double effectiveDesired = Up(desired + ValueRoundoff);

        // For literal penetrating depth h and radius r, the physical evaluation
        // obeys r²+(R-h)²<=R²+64uR². This avoids differentiating sqrt at h=0.
        // Value<=desired implies exact horizontal distance<=r+effectiveDesired.
        // Positive d uses r<=rMax. Negative d=-a additionally requires r>=a,
        // hence r>=max(a,rMin); subtract a LOWER 2a*r product for an upper bound.
        double sphereSquared = Up(radius * radius);
        double identityError = Up(64 * UnitRoundoff * sphereSquared);
        double effectiveSquared = Up(sphereSquared + identityError);
        if (effectiveDesired >= 0)
            effectiveSquared = Up(effectiveSquared + Up(Up(2 * effectiveDesired) * maxRadius));
        else
        {
            double erosion = -effectiveDesired, minimumFeasibleRadius = Math.Max(erosion, minRadius);
            double productLower = Down(Down(2 * erosion) * minimumFeasibleRadius);
            effectiveSquared = Up(effectiveSquared - productLower);
        }
        effectiveSquared = Up(effectiveSquared + Up(effectiveDesired * effectiveDesired));
        if (!(effectiveSquared >= 0) || !double.IsFinite(effectiveSquared)) return false;

        // Relative to (q.x,q.y,0), the ideal sphere centers interpolate between
        // (A.X-q.x,A.Y-q.y,R-A.Z) and the corresponding B endpoint in 3D.
        double ax = a.X - q.x, ay = a.Y - q.y, az = radius - a.Z;
        double bx = b.X - q.x, by = b.Y - q.y, bz = radius - b.Z;
        double vx = b.X - a.X, vy = b.Y - a.Y, vz = a.Z - b.Z;
        double lengthSquared = vx * vx + vy * vy + vz * vz;
        double t = lengthSquared > 0 ? Math.Clamp(-(ax * vx + ay * vy + az * vz) / lengthSquared, 0, 1) : 0;
        if (!double.IsFinite(t)) return false;
        double px = ax + vx * t, py = ay + vy * t, pz = az + vz * t;
        double length = Math.Sqrt(px * px + py * py + pz * pz);
        if (!(length > 0) || !double.IsFinite(length)) return false;
        const double shrink = 1 - 1e-12;
        double nx = px / length * shrink, ny = py / length * shrink, nz = pz / length * shrink;
        double normUpper = Up(Up(Up(nx * nx) + Up(ny * ny)) + Up(nz * nz));
        if (!double.IsFinite(normUpper) || normUpper > 1) return false;

        // Cauchy gives |C(t)|>=n.C(t); the latter is affine and has its minimum
        // at an endpoint. No accuracy assumption about the projected pose is
        // needed after certifying |n|<=1. Literal center interpolation and these
        // endpoint dot operations contribute <128uS; GeometryReserve covers
        // them for S<=1e4. ValueRoundoff separately covers XY distance and final
        // subtraction in Value. The first-section guard implies R>1e-8, making
        // underflow negligible compared with 4uR and 64uR² above.
        double firstDot = nx * ax + ny * ay + nz * az, lastDot = nx * bx + ny * by + nz * bz;
        double lower = Down(Math.Min(firstDot, lastDot) - GeometryReserve);
        double upper = Up(Math.Sqrt(effectiveSquared));
        if (!double.IsFinite(lower) || !double.IsFinite(upper)) return false;
        lowerDistance = lower; enclosingRadius = upper;
        return true;
    }

    private static double Up(double value) => Math.BitIncrement(value);
    private static double Down(double value) => Math.BitDecrement(value);
    private static bool Ordinary(double value) => double.IsFinite(value) && Math.Abs(value) <= MaximumMagnitude;
}
