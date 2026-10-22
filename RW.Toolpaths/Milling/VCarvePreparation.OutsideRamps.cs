using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

public sealed partial class VCarvePreparation
{
    private VCarvePreparation RelimitOutsidePolicy(double angle, bool ramps, CancellationToken token)
    {
        var profile = VCarvePhysicalSweep.ProfileFor(_tool, _depth.Depth);
        var options = _options with { SmoothRampAngleDegrees = angle, RampOutsideEdges = ramps };
        var regions = Regions.Select(region => {
            var upper = region.OriginalMedialPaths.Select((path, index) => CopyDepths(path,
                path.Points.Select((p, i) => p with { Z = -Math.Min(_depth.Depth,
                    profile.DepthAtRadius(region.SourceClearancePaths[index][i].Radius + GeometricErrorBound)) }).ToArray())).ToArray();
            var pair = VCarveRampSurface.FitMedial(region.OriginalMedialPaths, upper,
                region.Boundary, options, profile, _depth.Depth, token);
            return region with { SmoothedMedialPaths = pair.Limited, UpperMedialPaths = pair.Upper };
        }).ToArray();
        return new(OriginalRegions, _tool, _depth, options, regions);
    }

    // This changes the analytical grade field only. The cached finishing
    // preparation, source XY graph, clearing demand and physical access stay owned
    // by their operations. No rougher permission is granted to the finishing tool.
    private VCarvePreparation OpenOutsideRampAnchors(MillingBoundaryAccess outsideAccess,
        double angle, CancellationToken token)
    {
        var access = outsideAccess.Resolve(OriginalRegions, _tool, _depth, true, angle, 0, token);
        var physical = VCarvePhysicalSweep.ProfileFor(_tool, _depth.Depth);
        double grade = Math.Tan(angle * Math.PI / 180);
        var updated = new List<PreparedRegion>(Regions.Count);
        long lifted = 0, retainedEdges = 0,wallQueries=0,wallCacheHits=0,wallCacheResets=0,reusedRegions=0;
        foreach (var region in Regions)
        {
            token.ThrowIfCancellationRequested();
            var edges = access.RetainedRampEdges(VCarveToolpaths.BuildBoundary(OriginalRegions[region.RegionIndex], 0), token,out bool allRetained);
            retainedEdges += edges.Count;
            if(allRetained && _options.StockToLeave==0 && !_options.RampOutsideEdges
                && angle==_options.SmoothRampAngleDegrees)
            {
                // Exact open clipping proved that this policy removes no
                // original wall segment. Its source distances and grade field
                // are already prepared; no million-point re-query is needed.
                // The new scope below still owns its own access and witnesses.
                updated.Add(region);reusedRegions++;continue;
            }
            var distance = edges.Count == 0 ? null : new MedialAxisToolpaths.SpatialSegmentIndex(edges.ToList());
            double unbounded = physical.PhysicalRadiusAtDepth(_depth.Depth) + _depth.Depth / grade + 1;
            var wallDistances=new Dictionary<(double X,double Y),double>();
            double WallDistance(Point3D p)
            {
                if(distance==null)return unbounded;
                if(wallDistances.TryGetValue((p.X,p.Y),out double value)){wallCacheHits++;return value;}
                value=Math.Max(0,distance.QueryDistance(p.X,p.Y)-_options.StockToLeave);wallQueries++;
                if(wallDistances.Count==65536){wallDistances.Clear();wallCacheResets++;}
                wallDistances.Add((p.X,p.Y),value);return value;
            }
            var raw = new List<TaggedToolpath>(); var upper = new List<TaggedToolpath>();
            var clearances = new List<IReadOnlyList<MedialPoint>>();
            for (int pathIndex = 0; pathIndex < region.OriginalMedialPaths.Count; pathIndex++)
            {
                var source = region.OriginalMedialPaths[pathIndex];
                var points = new List<Point3D>(); var high = new List<Point3D>(); var radii = new List<MedialPoint>();
                for (int i = 0; i < source.Points.Count; i++)
                {
                    token.ThrowIfCancellationRequested();
                    var p = source.Points[i];
                    double radius = Math.Max(region.SourceClearancePaths[pathIndex][i].Radius,
                        WallDistance(p));
                    double depth = Math.Max(-p.Z, Math.Min(_depth.Depth, physical.DepthAtRadius(radius)));
                    if (depth > -p.Z + GeometricErrorBound) lifted++;
                    points.Add(p with { Z = -depth });
                    high.Add(p with { Z = -Math.Min(_depth.Depth, physical.DepthAtRadius(radius + GeometricErrorBound)) });
                    radii.Add(new(p.X, p.Y, radius));
                }
                raw.Add(CopyDepths(source, points)); upper.Add(CopyDepths(source, high)); clearances.Add(radii);
            }
            var surfaceOptions = _options with { AccessTarget = access, RampOutsideEdges = false, SmoothRampAngleDegrees = angle };
            var limited = VCarveRampSurface.FitMedial(raw, upper, region.Boundary,
                surfaceOptions, physical, _depth.Depth, token);
            updated.Add(region with { OriginalMedialPaths = raw, SourceClearancePaths = clearances,
                SmoothedMedialPaths = limited.Limited, UpperMedialPaths = limited.Upper });
        }
        var options = _options with { AccessTarget = access, RampOutsideEdges = false, SmoothRampAngleDegrees = angle };
        Log(options, $"event=outside-ramp-anchors enabled=False liftedSourcePoints={lifted} retainedWallSegments={retainedEdges} "
            +$"wallQueries={wallQueries} wallCacheHits={wallCacheHits} wallCacheResets={wallCacheResets} wallCacheLimit=65536 exactXYKeys=True "
            +$"unchangedWallRegionsReused={reusedRegions} unchangedWallProof=exact-open-clipping "
            + "originalGraphXYUnchanged=True finishingPermissionUnchanged=True physicalGuardsUnchanged=True");
        return new(OriginalRegions, _tool, _depth, options, updated);
    }

    internal static IReadOnlyList<TaggedToolpath> RampSources(IReadOnlyList<TaggedToolpath> source,
        VCarveCutterProfile profile, VCarveOptions options, CancellationToken token, bool upper = false)
    {
        if (!options.RampOutsideEdges || options.AccessTarget is not { } access) return source;
        var target = new PreparedRegionDistance(PolygonOps.Offset(access.Target, -options.StockToLeave, .0001, 0));
        long changed = 0;
        var result = source.Select(path => CopyDepths(path, path.Points.Select(p => {
            token.ThrowIfCancellationRequested();
            double radius = Math.Max(0, target.SignedDistance(new(p.X, p.Y))) + (upper ? GeometricErrorBound : 0);
            double depth = Math.Min(-p.Z, profile.DepthAtRadius(radius));
            if (depth < -p.Z - GeometricErrorBound) changed++;
            return p with { Z = -depth };
        }).ToArray())).ToArray();
        Log(options, $"event=outside-ramp-anchors enabled=True cappedSourcePoints={changed} upper={upper} "
            + "anchor=original-cutting-target physicalGuardsUnchanged=True");
        return result;
    }

    private static TaggedToolpath CopyDepths(TaggedToolpath source, IReadOnlyList<Point3D> points)
        => new(points, Array.Empty<ToolpathSpan>(), source.RegionIndex, source.Category, source.DepthPassIndex, detectCurves: false);
}
