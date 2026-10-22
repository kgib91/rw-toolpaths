using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>The authored material surface is a constant-grade offset of the
/// retained crest, capped at the next plateau. Medial branches nominate cutter
/// sites; their endpoints do not define additional cones in that surface.</summary>
internal static class VCarveRampSurface
{
    internal static List<TaggedToolpath> Sources(IReadOnlyList<IReadOnlyList<PointD>> boundary,
        VCarveOptions options, CancellationToken token)
    {
        if (options.RampOutsideEdges && options.AccessTarget is { } target)
            boundary = PolygonOps.Intersect(boundary, target.Target);
        var edges = !options.RampOutsideEdges && options.AccessTarget is { } access
            ? access.RetainedRampEdges(boundary, token)
            : boundary.SelectMany(ring => Enumerable.Range(0, ring.Count)
                .Select(i => (A: ring[i], B: ring[(i + 1) % ring.Count]))).ToArray();
        var result = new List<TaggedToolpath>(edges.Count);
        foreach (var (a, b) in edges)
        {
            token.ThrowIfCancellationRequested();
            if (a.x == b.x && a.y == b.y) continue;
            result.Add(new TaggedToolpath(new Point3D[] { new(a.x, a.y, 0), new(b.x, b.y, 0) }, Array.Empty<ToolpathSpan>(),
                0, "final-carve", null, detectCurves: false));
        }
        return result;
    }

    internal static (List<TaggedToolpath> Limited, List<TaggedToolpath> Upper) FitMedial(
        IReadOnlyList<TaggedToolpath> source, IReadOnlyList<TaggedToolpath> upper,
        IReadOnlyList<IReadOnlyList<PointD>> boundary, VCarveOptions options,
        VCarveCutterProfile profile, double maximumDepth, CancellationToken token)
    {
        var crests = Sources(boundary, options, token);
        var field = crests.Count == 0 ? null : new VCarveSmoothing.FloorField(crests, profile,
            Math.Tan(options.SmoothRampAngleDegrees * Math.PI / 180), maximumDepth, token);
        List<TaggedToolpath> Fit(IReadOnlyList<TaggedToolpath> paths, bool isUpper)
            => paths.Select(path => path with { Points = path.Points.Select(p => {
                token.ThrowIfCancellationRequested();
                double cap = field?.Depth(p.X, p.Y) ?? maximumDepth;
                if (isUpper) cap += MachiningAccuracy.FloorSearch;
                return p with { Z = -Math.Min(-p.Z, cap) };
            }).ToArray() }).ToList();
        return (Fit(source, false), Fit(upper, true));
    }

    internal static List<IReadOnlyList<PointD>> Plateau(
        IReadOnlyList<IReadOnlyList<PointD>> boundary, VCarveOptions options,
        double floor, double grade, CancellationToken token)
    {
        var crests = Sources(boundary, options, token);
        // This mask bounds MATERIAL, before the rougher's one compensation.
        // Finishing cutter support belongs to FloorField, not to this mask.
        return crests.Count == 0 ? new List<IReadOnlyList<PointD>>() : SmoothRoughingPlateau.Build(crests, boundary,
            floor, grade, token, target: options.AccessTarget?.Target);
    }
}
