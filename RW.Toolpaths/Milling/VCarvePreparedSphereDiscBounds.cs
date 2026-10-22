using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>The same first-hemisphere enclosure with interval work prepared once.
/// Query authority uses unnormalized Cauchy and outward squared bounds; no sqrt,
/// section traversal or radius evaluation occurs during a query.</summary>
internal static class VCarvePreparedSphereDiscBounds
{
    private const double U = VCarveSphereDiscBounds.UnitRoundoff;
    private const double S = VCarveSphereDiscBounds.MaximumMagnitude;
    private const double GeometryReserve = VCarveSphereDiscBounds.GeometryReserve;
    private const double ValueRoundoff = 128 * U * S;

    internal readonly record struct PreparedProfile(double Radius, double RadiusRoundoff, double SphereSquaredUpper);
    // Seven doubles,56 bytes. Only variable-depth intervals need a slot.
    internal readonly record struct PreparedInterval(double DeltaX, double DeltaY, double DeltaZ,
        double LengthSquared, double CenterZ0, double MinRadius, double MaxRadius);

    internal static bool TryPrepareProfile(VCarveCutterProfile profile, out PreparedProfile prepared)
    {
        prepared = default;
        var sections = profile.PhysicalProfile.Sections;
        if (sections.Length == 0) return false;
        var first = sections[0];
        double radius = first.TopRadius;
        if (first.Kind != CarvingCutterSectionKind.Hemisphere || first.BottomRadius != 0
            || first.Height != radius || !(radius > GeometryReserve)
            || radius > VCarveSphereDiscBounds.MaximumSphereRadius) return false;
        double squared = Up(radius * radius);
        prepared = new(radius, Up(4 * U * radius), Up(squared + Up(64 * U * squared)));
        return true;
    }

    internal static bool TryPrepareInterval(Point3D a, Point3D b, in PreparedProfile profile, out PreparedInterval prepared)
    {
        prepared = default;
        if (!Ordinary(a.X) || !Ordinary(a.Y) || !Ordinary(b.X) || !Ordinary(b.Y)
            || !double.IsFinite(a.Z) || !double.IsFinite(b.Z) || a.Z < 0 || b.Z < 0) return false;
        double minDepth = Math.Min(a.Z, b.Z), maxDepth = Math.Max(a.Z, b.Z);
        double depthError = Up(8 * U * maxDepth);
        double lowDepth = Math.Max(0, Down(minDepth - depthError)), highDepth = Up(maxDepth + depthError);
        if (!(highDepth < profile.Radius - GeometryReserve)) return false;
        double twiceRadius = 2 * profile.Radius;
        double minSquared = Math.Max(0, Down(lowDepth * Down(twiceRadius - lowDepth)));
        double maxSquared = Up(highDepth * Up(twiceRadius - highDepth));
        double minRadius = Math.Max(0, Down(Down(Math.Sqrt(minSquared)) - profile.RadiusRoundoff));
        double maxRadius = Up(Up(Math.Sqrt(maxSquared)) + profile.RadiusRoundoff);
        double dx = b.X - a.X, dy = b.Y - a.Y, dz = a.Z - b.Z;
        double lengthSquared = dx * dx + dy * dy + dz * dz;
        if (!double.IsFinite(lengthSquared)) return false;
        prepared = new(dx, dy, dz, lengthSquared, profile.Radius - a.Z, minRadius, maxRadius);
        return true;
    }

    internal static bool TryReject(PointD q, Point3D a, in PreparedInterval interval,
        double sphereSquaredUpper, double desired, out bool rejected)
    {
        rejected = false;
        if (!Ordinary(q.x) || !Ordinary(q.y) || !Ordinary(desired)) return false;
        double effectiveDesired = Up(desired + ValueRoundoff);
        double effectiveSquared = sphereSquaredUpper;
        if (effectiveDesired >= 0)
            effectiveSquared = Up(effectiveSquared + Up(Up(2 * effectiveDesired) * interval.MaxRadius));
        else
        {
            double erosion = -effectiveDesired, minimumRadius = Math.Max(erosion, interval.MinRadius);
            effectiveSquared = Up(effectiveSquared - Down(Down(2 * erosion) * minimumRadius));
        }
        effectiveSquared = Up(effectiveSquared + Up(effectiveDesired * effectiveDesired));
        if (!(effectiveSquared >= 0) || !double.IsFinite(effectiveSquared)) return false;

        double ax = a.X - q.x, ay = a.Y - q.y, az = interval.CenterZ0;
        double t = interval.LengthSquared > 0 ? Math.Clamp(-(ax * interval.DeltaX + ay * interval.DeltaY
            + az * interval.DeltaZ) / interval.LengthSquared, 0, 1) : 0;
        if (!double.IsFinite(t)) return false;
        // This vector only nominates a Cauchy direction. Its normalization and
        // the accuracy of the projected pose are unnecessary for negative proof.
        double ux = ax + interval.DeltaX * t, uy = ay + interval.DeltaY * t, uz = az + interval.DeltaZ * t;
        double normSquaredUpper = Up(Up(Up(ux * ux) + Up(uy * uy)) + Up(uz * uz));
        double l1Upper = Up(Up(Math.Abs(ux) + Math.Abs(uy)) + Math.Abs(uz));
        double firstDot = ux * ax + uy * ay + uz * az;
        double deltaDot = ux * interval.DeltaX + uy * interval.DeltaY + uz * interval.DeltaZ;
        // The interval's stored deltas/end-center and these endpoint dot sums
        // differ from ideal and literal centers by <128uS per component.
        // The 1e-8*|u|_1 deduction covers that error, as in the unit-vector helper.
        double dotLower = Down(Math.Min(firstDot, firstDot + deltaDot) - Up(GeometryReserve * l1Upper));
        if (!double.IsFinite(normSquaredUpper) || !double.IsFinite(dotLower)) return false;
        if (dotLower > 0)
        {
            // For any hypothetical witness C, u.C<=|u|*|C|. The positive lower
            // dot cannot exceed that product, so square only after requiring>0.
            double leftLower = Down(dotLower * dotLower), rightUpper = Up(effectiveSquared * normSquaredUpper);
            if (!double.IsFinite(leftLower) || !double.IsFinite(rightUpper)) return false;
            rejected = leftLower > rightUpper;
        }
        return true;
    }

    private static double Up(double value) => Math.BitIncrement(value);
    private static double Down(double value) => Math.BitDecrement(value);
    private static bool Ordinary(double value) => double.IsFinite(value) && Math.Abs(value) <= S;
}
