using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Configuration for medial-axis V-carving.</summary>
public sealed record VCarveOptions
{
    /// <summary>Material left at the boundary for a later finish pass.</summary>
    public double StockToLeave { get; init; }

    /// <summary>Fraction of the effective V-bit diameter used for clearing passes.</summary>
    public double StepOver { get; init; } = 0.4;

    /// <summary>Medial-axis parabola discretisation tolerance in workspace units.</summary>
    public double Tolerance { get; init; } = 0.03;

    /// <summary>Whether broad areas receive inward V-bit clearing paths.</summary>
    public bool IncludeInteriorFill { get; init; } = true;
}

/// <summary>
/// Produces V-carve toolpaths from pre-tessellated, world-space milling regions. The caller owns
/// document-model adaptation and output-machine coordinate conversion; this strategy owns all
/// generic geometry, depth, routing, and swept-area decisions.
/// </summary>
public static class VCarveToolpaths
{
    public const string Category = "vcarve";

    public static ToolpathPlan Generate(
        IReadOnlyList<MillingRegion> regions,
        ToolGeometry tool,
        DepthSchedule depth,
        VCarveOptions options,
        IMedialAxisProvider provider,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(regions);
        ArgumentNullException.ThrowIfNull(tool);
        ArgumentNullException.ThrowIfNull(depth);
        ArgumentNullException.ThrowIfNull(options);
        ArgumentNullException.ThrowIfNull(provider);

        if (regions.Count == 0
            || tool.Radius <= 1e-6
            || depth.Depth <= 0
            || tool.TipAngleRadians is not > 1e-6
            || tool.ConeLength is not > 1e-6)
        {
            return ToolpathPlan.Empty;
        }

        double maximumDepth = Math.Min(depth.Depth, tool.ConeLength.Value);
        if (maximumDepth <= 1e-6)
            return ToolpathPlan.Empty;

        double depthPerPass = depth.DepthPerPass > 0
            ? Math.Min(depth.DepthPerPass, maximumDepth)
            : maximumDepth;
        var effectiveDepth = new DepthSchedule(maximumDepth, depthPerPass)
        {
            SurfaceZ = depth.SurfaceZ,
        };

        var carveRegions = new List<(int RegionIndex, List<IReadOnlyList<PointD>> Rings)>(regions.Count);
        for (int regionIndex = 0; regionIndex < regions.Count; regionIndex++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var rings = BuildBoundary(regions[regionIndex], options.StockToLeave);
            if (rings.Count > 0)
                carveRegions.Add((regionIndex, rings));
        }

        if (carveRegions.Count == 0)
            return ToolpathPlan.Empty;

        int[] visitOrder = TravelOptimizer.Order(
            carveRegions.Select(region => (IReadOnlyList<PointD>)region.Rings[0]).ToList(),
            label: "vcarve",
            cancellationToken: cancellationToken);

        var toolpaths = new List<TaggedToolpath>();
        int sourcePointCount = 0;
        int compactedPointCount = 0;
        foreach (int visitSlot in visitOrder)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var (regionIndex, boundary) = carveRegions[visitSlot];
            List<TaggedToolpath> generated = MedialAxisToolpaths.GenerateVCarveTagged(
                provider,
                boundary,
                startDepth: 0.0,
                endDepth: maximumDepth,
                radianTipAngle: tool.TipAngleRadians.Value,
                depthPerPass: effectiveDepth.DepthPerPass,
                stepOver: options.StepOver,
                tolerance: options.Tolerance,
                regionIndex: regionIndex,
                bottomRadiusOverride: tool.BottomRadius,
                topRadiusOverride: tool.TopRadius,
                coneLengthOverride: tool.ConeLength,
                cancellationToken: cancellationToken,
                includeInteriorFill: options.IncludeInteriorFill);

            foreach (TaggedToolpath path in generated)
            {
                if (path.Points.Count < 2)
                    continue;

                sourcePointCount += path.Points.Count;
                List<Point3D> compacted = CompactAndClamp(
                    path.Points,
                    depth.SurfaceZ,
                    maximumDepth);
                compactedPointCount += compacted.Count;
                if (compacted.Count < 2)
                    continue;

                toolpaths.Add(new TaggedToolpath(
                    compacted,
                    path.Spans,
                    path.RegionIndex,
                    path.Category,
                    path.DepthPassIndex,
                    visitIdentity: toolpaths.Count));
            }
        }

        var centrelines = toolpaths
            .Select(path => (IReadOnlyList<PointD>)path.Points
                .Select(point => new PointD(point.X, point.Y))
                .ToList())
            .ToList();
        List<List<PointD>> sweptArea = PolygonOps.BufferOpenCenterlines(centrelines, tool.Radius);

        var diagnostics = new ToolpathDiagnostics
        {
            RingCount = carveRegions.Sum(region => region.Rings.Count),
            IslandCount = carveRegions.Count,
            PathCount = toolpaths.Count,
            DepthPassCount = effectiveDepth.PassCount,
            CutLength = toolpaths.Sum(path => PathLength(path.Points)),
        };
        Console.WriteLine(
            $"[VCarveToolpaths] regions={carveRegions.Count} paths={toolpaths.Count} " +
            $"sourcePoints={sourcePointCount} compactedPoints={compactedPointCount} " +
            $"depth={maximumDepth:F4} passes={effectiveDepth.PassCount} " +
            $"sweptPolygons={sweptArea.Count}");

        return new ToolpathPlan(toolpaths, sweptArea, diagnostics);
    }

    private static List<IReadOnlyList<PointD>> BuildBoundary(MillingRegion region, double stockToLeave)
    {
        List<PointD> outer = NormalizeRing(region.Outer, counterClockwise: true);
        if (outer.Count < 3)
            return new List<IReadOnlyList<PointD>>();

        var holes = new List<IReadOnlyList<PointD>>(region.Holes.Count);
        foreach (IReadOnlyList<PointD> hole in region.Holes)
        {
            List<PointD> normalizedHole = NormalizeRing(hole, counterClockwise: false);
            if (normalizedHole.Count >= 3)
                holes.Add(normalizedHole);
        }

        if (stockToLeave <= 0.0001)
        {
            var original = new List<IReadOnlyList<PointD>>(1 + holes.Count) { outer };
            original.AddRange(holes);
            return original;
        }

        return PolygonOps.OffsetBoundary(
                outer,
                holes,
                delta: -stockToLeave,
                arcTolerance: 0.25,
                simplifyTolerance: 0.25)
            .Where(ring => ring.Count >= 3)
            .Select(ring => (IReadOnlyList<PointD>)ring)
            .ToList();
    }

    private static List<PointD> NormalizeRing(IReadOnlyList<PointD> source, bool counterClockwise)
    {
        int count = source.Count;
        if (count > 1 && SamePoint(source[0], source[^1]))
            count--;

        var ring = new List<PointD>(count);
        for (int index = 0; index < count; index++)
            ring.Add(source[index]);

        if (ring.Count >= 3 && (Clipper.Area(new PathD(ring)) > 0) != counterClockwise)
            ring.Reverse();

        return ring;
    }

    private static List<Point3D> CompactAndClamp(
        IReadOnlyList<Point3D> source,
        double surfaceZ,
        double maximumDepth)
    {
        var compacted = new List<Point3D>(source.Count);
        foreach (Point3D point in source)
        {
            var candidate = new Point3D(
                point.X,
                point.Y,
                surfaceZ + Math.Clamp(point.Z, -maximumDepth, 0.0));
            while (compacted.Count >= 2
                && IsRedundant(compacted[^2], compacted[^1], candidate))
            {
                compacted[^1] = candidate;
            }

            if (compacted.Count == 0 || !SamePoint(compacted[^1], candidate))
                compacted.Add(candidate);
        }

        return compacted;
    }

    private static bool IsRedundant(Point3D start, Point3D middle, Point3D end)
    {
        double dx = end.X - start.X;
        double dy = end.Y - start.Y;
        double dz = end.Z - start.Z;
        double lengthSquared = dx * dx + dy * dy + dz * dz;
        if (lengthSquared <= 1e-24)
            return false;

        double mx = middle.X - start.X;
        double my = middle.Y - start.Y;
        double mz = middle.Z - start.Z;
        double projection = (mx * dx + my * dy + mz * dz) / lengthSquared;
        if (projection <= 1e-12 || projection >= 1.0 - 1e-12)
            return false;

        double closestX = start.X + projection * dx;
        double closestY = start.Y + projection * dy;
        double closestZ = start.Z + projection * dz;
        double errorX = middle.X - closestX;
        double errorY = middle.Y - closestY;
        double errorZ = middle.Z - closestZ;
        double tolerance = 1e-8 * Math.Max(1.0, Math.Sqrt(lengthSquared));
        return errorX * errorX + errorY * errorY + errorZ * errorZ <= tolerance * tolerance;
    }

    private static double PathLength(IReadOnlyList<Point3D> points)
    {
        double length = 0;
        for (int index = 1; index < points.Count; index++)
        {
            double dx = points[index].X - points[index - 1].X;
            double dy = points[index].Y - points[index - 1].Y;
            double dz = points[index].Z - points[index - 1].Z;
            length += Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        return length;
    }

    private static bool SamePoint(PointD first, PointD second)
        => Math.Abs(first.x - second.x) <= 1e-12
            && Math.Abs(first.y - second.y) <= 1e-12;

    private static bool SamePoint(Point3D first, Point3D second)
        => Math.Abs(first.X - second.X) <= 1e-12
            && Math.Abs(first.Y - second.Y) <= 1e-12
            && Math.Abs(first.Z - second.Z) <= 1e-12;
}