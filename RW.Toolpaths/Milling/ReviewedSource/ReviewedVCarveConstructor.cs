using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Milling.ReviewedSource;

// Conventional medial-axis-transform V-carving, with a flat-depth cap.
// Boost enumerates maximal disks; the saved physical cutter supplies depth.
// The source is constructed here. No old V-carve generator/fitter/ring owner
// is used. See design.md for the source and numerical contracts.
public static class ReviewedVCarveConstructor
{
    public const double Grid = 32768;

    private const double OffsetGrid = 32768;
    internal const double CommandClearance = .003;
    private sealed record RingEdge(PointD A, PointD B, List<(double T, PointD P)> Cuts);
    public sealed record Edge(int A, int B, string Owner);
    public sealed record Result(int Region, Point3D[] Nodes, Edge[] Edges, int RawMedialSegments,
        int CappedMedialIntervals, int CommandCollapsedIntervals, int Subdivisions, double ElapsedMs);

    public static Result Construct(ReviewedVCarveRegion region, VCarveCutterProfile profile, double depth, double surface, double depthPerPass, ReviewedVCarveSourceSettings settings, CancellationToken token=default, string? evidenceOutput = null, IReadOnlyList<IReadOnlyList<PointD>>? contactDepthFront = null)
    {
        token.ThrowIfCancellationRequested();
        if(!double.IsFinite(settings.SourceCurveToleranceMm)||settings.SourceCurveToleranceMm<=Math.Sqrt(2)/(2*Grid)
            ||!double.IsFinite(settings.MedialCurveCentralAngleRadians)||settings.MedialCurveCentralAngleRadians<=0||settings.MedialCurveCentralAngleRadians>Math.PI
            ||!double.IsFinite(settings.GeneratingSiteCornerAngleRadians)||settings.GeneratingSiteCornerAngleRadians<0||settings.GeneratingSiteCornerAngleRadians>Math.PI)
            throw new ArgumentOutOfRangeException(nameof(settings));
        double BoundaryTolerance=settings.SourceCurveToleranceMm;
        long started = Stopwatch.GetTimestamp();
        var physical = new PreparedRegionDistance(region.Boundary);
        // One actual site polygon owns medial, cap and nominal contact depth.
        // Quantize once, then give these SAME sites to both geometric kernels.
        var preparedBoundary = region.PreparedBoundary;
        PointD Up(PointD p) => new(Math.Floor(p.x * Grid + .5), Math.Floor(p.y * Grid + .5));
        IReadOnlyList<IReadOnlyList<PointD>> boundary = preparedBoundary.Select(r => (IReadOnlyList<PointD>)r.Select(p => {
            var q = Up(p); return new PointD(q.x / Grid, q.y / Grid);
        }).ToArray()).ToArray();
        var constructionDistance = new PreparedRegionDistance(boundary);
        // A numerical enclosure of float/F3 XY and the reused guard shell.
        // This is logged separately from the requested StockToLeave=0.
        double capRadius = profile.RadiusAtDepth(depth) + CommandClearance + .001;
        List<List<PointD>> capRings;
        if(contactDepthFront is not null)
        {
            capRings=contactDepthFront.Select(r=>r.ToList()).ToList();
            if(capRings.Any(r=>r.Count<3||r.Any(p=>!double.IsFinite(p.x+p.y))))
                throw new ArgumentException("STOCK_CONTACT_FRONT_INVALID");
        }
        else
        {
            capRings=new();
            var clip = PathUtils.ToClipper(boundary);
            var offsetter = new ClipperOffset(2, PathUtils.Scale / OffsetGrid);
            offsetter.AddPaths(clip, JoinType.Round, EndType.Polygon);
            var offset = new Paths64(); offsetter.Execute(-capRadius * PathUtils.Scale, offset);
            foreach (var ring in PathUtils.FromClipper(offset))
            {
                var distances = ring.Select(constructionDistance.SignedDistance).ToArray();
                double required = capRadius - 8 / PathUtils.Scale;
                if (distances.All(d => d >= required)) capRings.Add(ring);
                else if (distances.All(d => d < required))
                    Console.WriteLine($"[StandardInset] region={region.Id} event=collapsed-offset-component vertices={ring.Count} minimumClearance={distances.Min():R} maximumClearance={distances.Max():R} required={required:R} sourceMedialRetained=True");
                else throw new InvalidOperationException($"stage=depth-cap region={region.Id} reason=offset-loop-fails-inset-contract min={distances.Min():R} max={distances.Max():R} required={required:R}");
            }
        }
        var capDistance = new PreparedRegionDistance(capRings);
        var capEdges = capRings.SelectMany(r => Enumerable.Range(0, r.Count).Select(i =>
            new RingEdge(r[i], r[(i + 1) % r.Count], [(0, r[i]), (1, r[(i + 1) % r.Count])]))).ToArray();
        // The original design's single bounded polygon is used directly here;
        // the separate legacy nominal-layer preparation is not an input.
        // Supplied reference C0 explicitly selects 135 degrees between the
        // GENERATING adjacent segment sites. This is its construction policy,
        // not a length/angle test on already generated branches.
        IReadOnlyList<MedialSegment> Diagram(double siteCornerAngle) => BoostVoronoiProvider.CreateDefault().ConstructNominalMedialAxis(boundary[0].Select(Up).ToArray(),
            boundary.Skip(1).Select(r => (IReadOnlyList<PointD>)r.Select(Up).ToArray()).ToArray(),
            settings.MedialCurveCentralAngleRadians, profile.RadiusAtDepth(Math.Min(depth, depthPerPass)) * Grid, siteCornerAngle, token,
            s => Console.WriteLine($"[StandardVoronoi] region={region.Id} {s}"), null);
        var medial = Diagram(settings.GeneratingSiteCornerAngleRadians);
        var designGuard = new VCarveNativeContainment(region.Boundary, profile,token);
        var nodes = new List<Point3D>(); var nodeIndex = new Dictionary<Point3D, int>();
        var edges = new List<Edge>(); var edgeKeys = new HashSet<(int, int)>();
        int capped = 0, collapsed = 0, subdivisions = 0, physicalDepthLimits = 0;
        double maximumPhysicalLift = 0;
        Point3D Pose(PointD p)
        {
            double x = VCarveCommandRounding.Round((float)p.x), y = VCarveCommandRounding.Round((float)p.y);
            var xy = new PointD(x, y);
            double nominalRadius = Math.Max(0, constructionDistance.SignedDistance(xy) - CommandClearance);
            double physicalRadius = Math.Max(0, physical.SignedDistance(xy) - CommandClearance);
            double target = Math.Min(depth, profile.DepthAtRadius(nominalRadius));
            double d = Math.Min(target, profile.DepthAtRadius(physicalRadius));
            if (target - d > 1e-9) { physicalDepthLimits++; maximumPhysicalLift = Math.Max(maximumPhysicalLift, target - d); }
            // Downward quantization error never deepens the requested cap.
            d = Math.Max(0, Math.Floor(d * 1000 + 1e-8) / 1000);
            return new(x, y, -d);
        }
        int Node(Point3D p)
        {
            if (nodeIndex.TryGetValue(p, out int i)) return i;
            nodeIndex.Add(p, nodes.Count); nodes.Add(p); return nodes.Count - 1;
        }
        void Add(PointD a, PointD b, string owner, int level = 0)
        {
            token.ThrowIfCancellationRequested();
            var pa = Pose(a); var pb = Pose(b);
            if (pa == pb) { collapsed++; return; }
            // A commanded segment entirely on the start plane removes no
            // volume. Keep positive-depth midpoint work even when both rounded
            // endpoints are at zero; otherwise account it as command-collapsed.
            if (pa.Z == 0 && pb.Z == 0 && Pose(new((a.x + b.x) / 2, (a.y + b.y) / 2)).Z == 0)
            { collapsed++; return; }
            if (!designGuard.Legal(pa, pb))
            {
                if (level == 22 || Math.Sqrt(ReviewedVCarveMath.DistanceSquared(a, b)) < .00001)
                    throw new InvalidOperationException($"stage=source-chord region={region.Id} owner={owner} a={pa} b={pb} reason=unproved-design-clearance level={level}");
                var mid = new PointD((a.x + b.x) / 2, (a.y + b.y) / 2); subdivisions++;
                Add(a, mid, owner, level + 1); Add(mid, b, owner, level + 1); return;
            }
            int ia = Node(pa), ib = Node(pb); var key = (Math.Min(ia, ib), Math.Max(ia, ib));
            if (edgeKeys.Add(key)) edges.Add(new(ia, ib, owner));
        }
        for (int m = 0; m < medial.Count; m++)
        {
            var s = medial[m]; var a = new PointD(s.Point0.X / Grid, s.Point0.Y / Grid); var b = new PointD(s.Point1.X / Grid, s.Point1.Y / Grid);
            var cuts = new List<(double T, PointD P)> { (0, a), (1, b) };
            foreach (var ring in capEdges)
                if (Intersect(a, b, ring.A, ring.B) is { } hit)
                {
                    cuts.Add((hit.T, hit.P)); ring.Cuts.Add((hit.U, hit.P));
                }
            var sorted = cuts.OrderBy(p => p.T).DistinctBy(p => Math.Round(p.T, 12)).ToArray();
            for (int j = 1; j < sorted.Length; j++)
            {
                var first = sorted[j - 1].P; var last = sorted[j].P;
                var mid = new PointD((first.x + last.x) / 2, (first.y + last.y) / 2);
                if (capRings.Count > 0 && capDistance.SignedDistance(mid) > 0) { capped++; continue; }
                Add(first, last, $"medial:{m}");
            }
        }
        for (int i = 0; i < capEdges.Length; i++)
        {
            var sorted = capEdges[i].Cuts.OrderBy(p => p.T).DistinctBy(p => Math.Round(p.T, 12)).ToArray();
            for (int j = 1; j < sorted.Length; j++) Add(sorted[j - 1].P, sorted[j].P, $"cap:{i}");
        }
        // A circular component narrower than the depth-cap diameter has a
        // point MAT. Preserve that real maximal-disc task when the selected
        // diagram contains no finite source edge; a zero-path region is not
        // successful completion. The unselected diagram supplies its vertex.
        if (edges.Count == 0)
        {
            var complete = Diagram(Math.PI);
            var peak = complete.SelectMany(e => new[] { e.Point0, e.Point1 }).MaxBy(p => p.Radius);
            var deepest = Pose(new(peak.X / Grid, peak.Y / Grid)); var top = deepest with { Z = 0 };
            if (deepest.Z >= 0 || !designGuard.Legal(top, deepest))
                throw new InvalidOperationException($"stage=point-medial region={region.Id} reason=no-commandable-maximal-disc");
            edges.Add(new(Node(top), Node(deepest), "medial-point"));
        }
        Console.WriteLine($"[StandardConstruction] region={region.Id} source=authored-shape nominalVertices={boundary.Sum(r => r.Count)} physicalVertices={region.Boundary.Sum(r => r.Count)} " +
            $"rawMedial={medial.Count} cappedInteriorIntervals={capped} nodes={nodes.Count} uniqueRequiredEdges={edges.Count} commandCollapsed={collapsed} subdivisions={subdivisions} " +
            $"commandClearanceMm={CommandClearance:R} sourceSiteCornerDegrees=135 stockToLeave=0 stockDerivedDepthFront={contactDepthFront is not null} oldGenerationCalls=0");
        Console.WriteLine($"[ConstructionConsistency] region={region.Id} sharedBoundary=bounded-original-design medialGridMm={1 / Grid:R} capSource={(contactDepthFront is null?"nominal-original-stock":"current-stock-full-cutter")} nominalDepthFromSameSites=True physicalDepthLimits={physicalDepthLimits} maximumPhysicalLiftMm={maximumPhysicalLift:R}");
        if (evidenceOutput is not null)
            File.WriteAllText(Path.Combine(evidenceOutput, $"construction-region-{region.Id}.json"), System.Text.Json.JsonSerializer.Serialize(new {
                region = region.Id, boundaryToleranceMm = BoundaryTolerance, siteGridPerMm = Grid, boundary = boundary.Select(r => r.Select(p => new[] { p.x, p.y })),
                capRadius, capRings = capRings.Select(r => r.Select(p => new[] { p.x, p.y })),
                medial = medial.Select(s => new[] { new[] { s.Point0.X / Grid, s.Point0.Y / Grid, s.Point0.Radius / Grid }, new[] { s.Point1.X / Grid, s.Point1.Y / Grid, s.Point1.Radius / Grid } }),
                physicalDepthLimits, maximumPhysicalLiftMm = maximumPhysicalLift,
                physicalBoundaryWitnessErrorMm = boundary.SelectMany(r => r).Max(p => Math.Abs(physical.SignedDistance(p)))
            }));
        return new(region.Id, nodes.ToArray(), edges.ToArray(), medial.Count, capped, collapsed, subdivisions, Stopwatch.GetElapsedTime(started).TotalMilliseconds);
    }

    private static (double T, double U, PointD P)? Intersect(PointD a, PointD b, PointD c, PointD d)
    {
        var r = ReviewedVCarveMath.Sub(b, a); var s = ReviewedVCarveMath.Sub(d, c);
        double denominator = ReviewedVCarveMath.Cross(r, s); if (Math.Abs(denominator) < 1e-18) return null;
        var q = ReviewedVCarveMath.Sub(c, a); double t = ReviewedVCarveMath.Cross(q, s) / denominator, u = ReviewedVCarveMath.Cross(q, r) / denominator;
        if (t < 0 || t > 1 || u < 0 || u > 1) return null;
        return (t, u, new(a.x + t * r.x, a.y + t * r.y));
    }
}
