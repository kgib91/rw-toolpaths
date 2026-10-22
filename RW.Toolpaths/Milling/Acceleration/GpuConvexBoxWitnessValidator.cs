using System.Runtime.InteropServices;

namespace RW.Toolpaths.Milling.Acceleration;

[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuBoxQuery
{
    internal readonly double X0, Y0, X1, Y1, Desired;
    internal GpuBoxQuery(double x0, double y0, double x1, double y1, double desired)
    { X0 = x0; Y0 = y0; X1 = x1; Y1 = y1; Desired = desired; }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuBoxNomination
{
    internal readonly double T0, T1, T2, T3;
    internal readonly int SegmentOrdinal, Status;
    internal GpuBoxNomination(int segmentOrdinal, double t0, double t1, double t2, double t3, int status)
    { SegmentOrdinal = segmentOrdinal; T0 = t0; T1 = t1; T2 = t2; T3 = t3; Status = status; }
    internal bool HasWitness => Status == 1;
}

/// <summary>Whole-box positive proof through one convex physical swept interval.
/// Discs with affine centers and concave nonnegative radii have a convex union:
/// mixed offsets have norm at most the mixed radii, which concavity bounds by
/// the radius at the mixed pose. Its positive-gap expansion is convex too, so
/// four literal corner witnesses from the SAME interval cover the whole box.
/// Guards and the caller's 1e-8 gap reserve cover center/depth/profile roundoff.</summary>
internal static class GpuConvexBoxWitnessValidator
{
    internal const double GeometryReserve = 1e-8;
    private const double U = 1.1102230246251565e-16;

    internal sealed record PreparedProfile(VCarveCutterProfile Profile, PreparedSection[] Sections, bool Supported);
    internal readonly record struct PreparedSection(double MinimumDepth, double MaximumDepth);

    internal static PreparedProfile Prepare(VCarveCutterProfile profile)
    {
        var sections = new List<PreparedSection>();
        double height = 0, previousRadius = profile.Sections[0].BottomRadius;
        foreach (CarvingCutterSection section in profile.Sections)
        {
            if (section.BottomRadius != previousRadius || section.TopRadius < section.BottomRadius
                || section.TopRadius > 100 || section.Height > 100
                || (section.Kind == CarvingCutterSectionKind.Hemisphere
                    && (height != 0 || section.BottomRadius != 0 || section.Height != section.TopRadius))
                || section.Kind is not (CarvingCutterSectionKind.Linear or CarvingCutterSectionKind.Hemisphere)
                || (section.Kind == CarvingCutterSectionKind.Linear
                    && (section.TopRadius - section.BottomRadius) / section.Height > 100))
                return new(profile, Array.Empty<PreparedSection>(), false);
            double end = height + section.Height;
            if (end > 100) return new(profile, Array.Empty<PreparedSection>(), false);
            double startError = 64 * U * 100;
            sections.Add(new(Math.BitIncrement(height + startError), Math.BitDecrement(end - startError)));
            height = end; previousRadius = section.TopRadius;
        }
        return new(profile, sections.ToArray(), true);
    }

    internal static bool TryValidate(GpuDiscSegment[] segments, VCarveCutterProfile profile,
        GpuBoxQuery query, GpuBoxNomination nomination, out int sourcePathIndex, Action? discCheckpoint = null)
        => TryValidate(segments, Prepare(profile), query, nomination, out sourcePathIndex, discCheckpoint);

    internal static bool TryValidate(GpuDiscSegment[] segments, PreparedProfile prepared,
        GpuBoxQuery query, GpuBoxNomination nomination, out int sourcePathIndex, Action? discCheckpoint = null)
    {
        sourcePathIndex = -1;
        if (!prepared.Supported || !nomination.HasWitness || (uint)nomination.SegmentOrdinal >= (uint)segments.Length
            || !Ordinary(query.X0, 10000) || !Ordinary(query.X1, 10000)
            || !Ordinary(query.Y0, 10000) || !Ordinary(query.Y1, 10000)
            || query.X0 > query.X1 || query.Y0 > query.Y1 || !Ordinary(query.Desired, 100) || query.Desired < 0
            || !Pose(nomination.T0) || !Pose(nomination.T1) || !Pose(nomination.T2) || !Pose(nomination.T3)) return false;
        double first = Math.Min(Math.Min(nomination.T0, nomination.T1), Math.Min(nomination.T2, nomination.T3));
        double last = Math.Max(Math.Max(nomination.T0, nomination.T1), Math.Max(nomination.T2, nomination.T3));
        if (!TryPrepareSpan(segments, prepared, nomination.SegmentOrdinal, first, last, out GpuDiscSegment s)) return false;
        if (!Vertex(s, prepared, query.X0, query.Y0, nomination.T0, query.Desired, discCheckpoint)
            || !Vertex(s, prepared, query.X1, query.Y0, nomination.T1, query.Desired, discCheckpoint)
            || !Vertex(s, prepared, query.X1, query.Y1, nomination.T2, query.Desired, discCheckpoint)
            || !Vertex(s, prepared, query.X0, query.Y1, nomination.T3, query.Desired, discCheckpoint)) return false;
        sourcePathIndex = s.SourcePathIndex; return true;
    }

    // Boxes and exact target triangles share the same immutable profile,
    // guarded single concave pose span, and literal actual CPU vertex predicate.
    // Span preparation performs no physical-disc evaluation or budget charge.
    internal static bool TryPrepareSpan(GpuDiscSegment[] segments, PreparedProfile prepared, int segmentOrdinal,
        double first, double last, out GpuDiscSegment s)
    {
        s = default;
        if (!prepared.Supported || (uint)segmentOrdinal >= (uint)segments.Length
            || !Pose(first) || !Pose(last) || first > last) return false;
        s = segments[segmentOrdinal];
        if (!Ordinary(s.Ax, 10000) || !Ordinary(s.Ay, 10000) || !Ordinary(s.Bx, 10000) || !Ordinary(s.By, 10000)
            || !Ordinary(s.Az, 100) || !Ordinary(s.Bz, 100) || s.SourcePathIndex < 0) return false;
        double za = s.Az + (s.Bz - s.Az) * first, zb = s.Az + (s.Bz - s.Az) * last;
        double minimum = Math.Min(za, zb), maximum = Math.Max(za, zb);
        if (!(minimum >= .001) || !double.IsFinite(maximum)) return false;
        double depthError = 32 * U * Math.Max(1, Math.Abs(s.Az) + Math.Abs(s.Bz));
        double minimumIdeal = Math.BitDecrement(minimum - depthError), maximumIdeal = Math.BitIncrement(maximum + depthError);
        if (!(minimumIdeal > 0)) return false;
        bool supportedSpan = false;
        foreach (PreparedSection section in prepared.Sections)
        {
            if (minimumIdeal >= section.MinimumDepth && maximumIdeal <= section.MaximumDepth) { supportedSpan = true; break; }
        }
        if (!supportedSpan) return false;
        // With endpoint depth<=100, depth error<=7.2e-13; hemisphere derivative
        // at h>=.001,R<=100 is <225, while linear slopes<=100. Together with
        // coordinate interpolation error<1e-11 and profile arithmetic, total
        // corner error is <1e-9, strictly below the caller's 1e-8 reserve.
        return true;
    }

    internal static bool Vertex(GpuDiscSegment s, PreparedProfile prepared, double x, double y, double t,
        double desired, Action? discCheckpoint)
    {
        discCheckpoint?.Invoke();
        double depth = s.Az + (s.Bz - s.Az) * t;
        if (!(depth >= .001)) return false;
        double dx = s.Ax + (s.Bx - s.Ax) * t - x, dy = s.Ay + (s.By - s.Ay) * t - y;
        return Math.Sqrt(dx * dx + dy * dy) - prepared.Profile.PhysicalRadiusAtDepth(depth) <= desired;
    }

    internal static bool Ordinary(double value, double maximum) => double.IsFinite(value) && Math.Abs(value) <= maximum;
    internal static bool Pose(double value) => double.IsFinite(value) && value >= 0 && value <= 1;
}
