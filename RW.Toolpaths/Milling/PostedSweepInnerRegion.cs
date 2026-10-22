using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Conservative inner union of real posted circular-cutter sweeps.
/// Shared by Pocket stepdown proof and preceding-rougher stock. Input chains
/// contain only already-executed chords; no joining shortcut earns stock.</summary>
internal static class PostedSweepInnerRegion
{
    internal const double Reserve = 16 / PathUtils.Scale;
    internal const double ArcTolerance = .001;

    internal static PolygonOps.BoundarySegmentIndex Build(Paths64 chains, double radius,
        CancellationToken token)
        => new(BuildPaths(chains, radius, token));

    internal static Paths64 BuildPaths(Paths64 chains, double radius, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        var offsetter = new ClipperOffset(2, ArcTolerance * PathUtils.Scale);
        offsetter.AddPaths(chains, JoinType.Round, EndType.Round);
        var swept = new Paths64();
        offsetter.Execute(Math.Max(0, radius - Reserve) * PathUtils.Scale, swept);
        token.ThrowIfCancellationRequested();
        return Clipper.Union(swept, FillRule.NonZero);
    }
}
