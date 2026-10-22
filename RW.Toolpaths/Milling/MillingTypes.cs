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
/// the optional physical profile describes depth-dependent V-carving width. The
/// cone fields remain available for callers describing a single conical section.
/// </summary>
public sealed record ToolGeometry(double Radius)
{
    /// <summary>Physical cutting sections from the actual tip, including a rounded nose.</summary>
    public VCarveCutterProfile? VCarveProfile { get; init; }

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
    private int _skipDepthPasses;
    private double _startDepth;

    /// <summary>Z of the material surface, normally 0.</summary>
    public double SurfaceZ { get; init; }

    /// <summary>
    /// Depth of the already cleared material floor, measured positively below the
    /// unchanged physical stock surface. New passes advance below this floor by
    /// DepthPerPass; a floor equal to Depth retains one finishing pass at that floor.
    /// </summary>
    public double StartDepth
    {
        get => _startDepth;
        init
        {
            if (!double.IsFinite(value) || value < 0 || value > Depth)
                throw new ArgumentOutOfRangeException(nameof(value), "Starting depth must be finite and between zero and the final depth.");
            _startDepth = value;
        }
    }

    /// <summary>
    /// Number of leading nominal depth passes omitted because an earlier operation
    /// already roughed them. A truncated local/tool depth always retains its final pass.
    /// This does not change the material surface or claim entry clearance.
    /// </summary>
    public int SkipDepthPasses
    {
        get => _skipDepthPasses;
        init
        {
            ArgumentOutOfRangeException.ThrowIfNegative(value);
            _skipDepthPasses = value;
        }
    }

    /// <summary>Number of passes before leading passes are omitted, always at least one.</summary>
    public int NominalPassCount
    {
        get
        {
            double remaining = Depth - StartDepth;
            double perPass = DepthPerPass > 0 ? DepthPerPass : remaining;
            if (perPass <= 0)
                return 1;
            return Math.Max(1, (int)Math.Ceiling(remaining / perPass - 1e-9));
        }
    }

    /// <summary>Nominal index of the first emitted pass, preserving the final target.</summary>
    public int FirstPassIndex => Math.Min(SkipDepthPasses, NominalPassCount - 1);

    /// <summary>Number of emitted depth passes, always at least one.</summary>
    public int PassCount => NominalPassCount - FirstPassIndex;

    /// <summary>Absolute Z at the bottom of emitted pass <paramref name="passIndex"/> (zero-based).</summary>
    public double PassBottomZ(int passIndex)
    {
        double perPass = DepthPerPass > 0 ? DepthPerPass : Depth - StartDepth;
        double cut = Math.Min(Depth, StartDepth + perPass * ((double)FirstPassIndex + passIndex + 1));
        return SurfaceZ - cut;
    }

    /// <summary>
    /// Absolute Z the cutter starts emitted pass <paramref name="passIndex"/> from.
    /// The first retained pass enters from the known cleared material floor. Omitting
    /// nominal passes never lowers that entry floor; subsequent passes enter from the
    /// preceding emitted depth. SurfaceZ continues to describe the physical stock top.
    /// </summary>
    public double PassTopZ(int passIndex)
        => passIndex == 0 ? SurfaceZ - StartDepth : PassBottomZ(passIndex - 1);
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
