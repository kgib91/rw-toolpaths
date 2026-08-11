using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Direction of cutter rotation relative to the material.</summary>
public enum MillingDirection
{
    /// <summary>Leave the natural winding produced by the offset engine.</summary>
    Default = 0,

    /// <summary>Cutter rotation opposes feed; lower forces and a better finish.</summary>
    Climb = 1,

    /// <summary>Cutter rotation matches feed; more predictable on thin walls and soft stock.</summary>
    Conventional = 2,
}

/// <summary>Which side of the contour the cutter runs on.</summary>
public enum ProfileSide
{
    /// <summary>Cutter runs inside the contour, leaving the outline as a hole.</summary>
    Inside = 0,

    /// <summary>Cutter runs outside the contour, leaving the outline as a part.</summary>
    Outside = 1,

    /// <summary>Cutter centre traces the contour exactly (engraving).</summary>
    OnLine = 2,
}

/// <summary>
/// One closed machining region: an outer boundary and the holes immediately inside it.
/// Coordinates are workspace units (mm), pre-tessellated.
/// </summary>
public sealed class MillingRegion
{
    public MillingRegion(IReadOnlyList<PointD> outer, IReadOnlyList<IReadOnlyList<PointD>>? holes = null)
    {
        Outer = outer;
        Holes = holes ?? Array.Empty<IReadOnlyList<PointD>>();
    }

    public IReadOnlyList<PointD> Outer { get; }

    public IReadOnlyList<IReadOnlyList<PointD>> Holes { get; }

    /// <summary>Outer boundary followed by every hole, as one flat ring group.</summary>
    public List<IReadOnlyList<PointD>> AsRingGroup()
    {
        var group = new List<IReadOnlyList<PointD>>(1 + Holes.Count) { Outer };
        group.AddRange(Holes);
        return group;
    }
}

/// <summary>
/// Cutting geometry of the tool. Only <see cref="Radius"/> matters for flat-bottom milling;
/// the cone fields describe V-bits and tapered cutters for depth-dependent width.
/// </summary>
public sealed record ToolGeometry(double Radius)
{
    /// <summary>Included tip angle in radians for a V-bit; <c>null</c> for flat/ball cutters.</summary>
    public double? TipAngleRadians { get; init; }

    /// <summary>Radius at the tip; 0 for a true point, non-zero for a flattened V-bit.</summary>
    public double? BottomRadius { get; init; }

    /// <summary>Radius where the cone meets the shank.</summary>
    public double? TopRadius { get; init; }

    /// <summary>Axial length of the conical section.</summary>
    public double? ConeLength { get; init; }

    public static ToolGeometry Flat(double radius) => new(radius);
}

/// <summary>
/// Depth passes for one operation. Z is negative into the material, matching
/// <see cref="Point3D"/> conventions across the library.
/// </summary>
/// <param name="Depth">Total cut depth as a positive magnitude below <see cref="SurfaceZ"/>.</param>
/// <param name="DepthPerPass">Maximum depth removed per pass; clamped to <paramref name="Depth"/> when larger.</param>
public sealed record DepthSchedule(double Depth, double DepthPerPass)
{
    /// <summary>Z of the material surface, normally 0.</summary>
    public double SurfaceZ { get; init; }

    /// <summary>Number of depth passes required, always at least one.</summary>
    public int PassCount
    {
        get
        {
            double perPass = DepthPerPass > 0 ? DepthPerPass : Depth;
            if (perPass <= 0)
                return 1;
            return Math.Max(1, (int)Math.Ceiling(Depth / perPass - 1e-9));
        }
    }

    /// <summary>Absolute Z at the bottom of pass <paramref name="passIndex"/> (zero-based).</summary>
    public double PassBottomZ(int passIndex)
    {
        double perPass = DepthPerPass > 0 ? DepthPerPass : Depth;
        double cut = Math.Min(Depth, perPass * (passIndex + 1));
        return SurfaceZ - cut;
    }

    /// <summary>Absolute Z the cutter starts pass <paramref name="passIndex"/> from.</summary>
    public double PassTopZ(int passIndex)
        => passIndex == 0 ? SurfaceZ : PassBottomZ(passIndex - 1);
}

/// <summary>Geometric fidelity knobs shared by every strategy. Values are in mm.</summary>
public sealed record GeometryTolerances
{
    /// <summary>Chord tolerance for round-join arcs emitted by the offset engine.</summary>
    public double ArcTolerance { get; init; } = 0.25;

    /// <summary>RDP tolerance applied to offset output to keep the point count machinable.</summary>
    public double SimplifyTolerance { get; init; } = 0.25;

    public static readonly GeometryTolerances Default = new();
}

internal static class MillingDirectionExtensions
{
    /// <summary>Maps to the string form understood by <see cref="PathUtils.OrientPath"/>.</summary>
    public static string? ToPathUtilsToken(this MillingDirection direction) => direction switch
    {
        MillingDirection.Climb => "climb",
        MillingDirection.Conventional => "conventional",
        _ => null,
    };
}
