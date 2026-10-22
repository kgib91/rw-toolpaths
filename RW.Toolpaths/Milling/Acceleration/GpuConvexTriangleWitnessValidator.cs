using System.Runtime.InteropServices;

namespace RW.Toolpaths.Milling.Acceleration;

[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuTriangleQuery
{
    internal readonly double Ax, Ay, Bx, By, Cx, Cy, Desired;
    internal GpuTriangleQuery(double ax, double ay, double bx, double by, double cx, double cy, double desired)
    { Ax = ax; Ay = ay; Bx = bx; By = by; Cx = cx; Cy = cy; Desired = desired; }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuTriangleNomination
{
    internal readonly double T0, T1, T2;
    internal readonly int SegmentOrdinal, Status;
    internal GpuTriangleNomination(int segmentOrdinal, double t0, double t1, double t2, int status)
    { SegmentOrdinal = segmentOrdinal; T0 = t0; T1 = t1; T2 = t2; Status = status; }
    internal bool HasWitness => Status == 1;
}

/// <summary>Positive coverage of one closed triangle through three real poses
/// on the SAME eligible concave physical interval. The original double CPU
/// profile and inequality retain authority. The swept-disc gap expansion is
/// convex, so covering all vertices covers their complete convex hull.
/// Triangle signs and exact target decomposition are separate caller authority.
/// Desired must be the original gap minus GeometryReserve, as for boxes.</summary>
internal static class GpuConvexTriangleWitnessValidator
{
    internal const double GeometryReserve = GpuConvexBoxWitnessValidator.GeometryReserve;
    internal const double DyadicCoordinateReserve = 1e-10;

    // Exact longest-edge dyadic vertices have |integer numerator|<2^78 and
    // shift<=48. Convert (double)numerator / (32768 * 2^shift), a normal power-
    // of-two scale with no under/overflow. At |world XY|<=1e4 a correctly
    // rounded coordinate differs by at most 2^-40; planar error<1.3e-12.
    // Signed disc distance is 1-Lipschitz, so DyadicCoordinateReserve covers
    // the conversion. It is part of, rather than added to, the same 1e-8 total
    // gap reserve: existing physical arithmetic<1e-9 plus 1e-10 remains below
    // 1e-8. The caller retains the original exact dyadic triangle as domain.
    internal static bool TryValidate(GpuDiscSegment[] segments, VCarveCutterProfile profile,
        GpuTriangleQuery query, GpuTriangleNomination nomination, out int sourcePathIndex,
        Action? discCheckpoint = null)
        => TryValidate(segments, GpuConvexBoxWitnessValidator.Prepare(profile), query, nomination,
            out sourcePathIndex, discCheckpoint);

    internal static bool TryValidate(GpuDiscSegment[] segments, GpuConvexBoxWitnessValidator.PreparedProfile prepared,
        GpuTriangleQuery query, GpuTriangleNomination nomination, out int sourcePathIndex,
        Action? discCheckpoint = null)
    {
        sourcePathIndex = -1;
        if (!nomination.HasWitness || !GpuConvexBoxWitnessValidator.Ordinary(query.Ax, 10000)
            || !GpuConvexBoxWitnessValidator.Ordinary(query.Ay, 10000)
            || !GpuConvexBoxWitnessValidator.Ordinary(query.Bx, 10000)
            || !GpuConvexBoxWitnessValidator.Ordinary(query.By, 10000)
            || !GpuConvexBoxWitnessValidator.Ordinary(query.Cx, 10000)
            || !GpuConvexBoxWitnessValidator.Ordinary(query.Cy, 10000)
            || !GpuConvexBoxWitnessValidator.Ordinary(query.Desired, 100) || query.Desired < 0
            || !GpuConvexBoxWitnessValidator.Pose(nomination.T0) || !GpuConvexBoxWitnessValidator.Pose(nomination.T1)
            || !GpuConvexBoxWitnessValidator.Pose(nomination.T2)) return false;
        double first = Math.Min(nomination.T0, Math.Min(nomination.T1, nomination.T2));
        double last = Math.Max(nomination.T0, Math.Max(nomination.T1, nomination.T2));
        if (!GpuConvexBoxWitnessValidator.TryPrepareSpan(segments, prepared, nomination.SegmentOrdinal,
            first, last, out GpuDiscSegment s)) return false;
        if (!GpuConvexBoxWitnessValidator.Vertex(s, prepared, query.Ax, query.Ay, nomination.T0, query.Desired, discCheckpoint)
            || !GpuConvexBoxWitnessValidator.Vertex(s, prepared, query.Bx, query.By, nomination.T1, query.Desired, discCheckpoint)
            || !GpuConvexBoxWitnessValidator.Vertex(s, prepared, query.Cx, query.Cy, nomination.T2, query.Desired, discCheckpoint)) return false;
        sourcePathIndex = s.SourcePathIndex; return true;
    }
}
