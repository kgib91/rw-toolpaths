using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>The authored material surface is a constant-grade offset of the
/// retained crest, capped at the next plateau. Medial branches nominate cutter
/// sites; their endpoints do not define additional cones in that surface.</summary>
internal static class VCarveRampSurface
{
    // The complete slope is additive. Fill Limit selects only the subsequent
    // floor cleanup; it never truncates a ramp. These remain nominations: the
    // existing floor/wall/posted-chord guards own Z and the full cutter envelope.
    internal static List<TaggedToolpath> Contours(IReadOnlyList<IReadOnlyList<PointD>> boundary,
        VCarveOptions options, VCarveCutterProfile profile, double depth, int regionIndex,
        CancellationToken token, Action<string>? diagnostic)
    {
        token.ThrowIfCancellationRequested();
        long started = Stopwatch.GetTimestamp();
        // The physical domain already removes approved outside walls. Offsetting
        // its intersection with Target would put those walls back into every
        // concentric front. Keep the physical wall fronts and clip their visits
        // to demand only after offsetting. Explicit outside ramps opt back in.
        if (options.RampOutsideEdges && options.AccessTarget is { } access)
            boundary = PolygonOps.Intersect(boundary, access.Target);
        if (MillingGeometryTrace.Observer.Value is not null)
            MillingGeometryTrace.Capture("outline-source", regionIndex, boundary.Select(r =>
                new TaggedToolpath(r.Select(p => new Point3D(p.x,p.y,0)).ToArray(),
                    Array.Empty<ToolpathSpan>(),regionIndex,"outline-source",null,detectCurves:false)).ToArray());
        string contourSource = options.AccessTarget != null && !options.RampOutsideEdges
            ? "retained-physical-walls" : "cutting-outline";
        var rampBand=ConcentricBandSchedule.Ramp(profile,depth,options);
        double grade=rampBand.Grade,diameter=2*profile.PhysicalRadiusAtDepth(depth);
        double surfacePitch=rampBand.SurfacePitch,pitch=rampBand.Pitch,first=rampBand.First;
        var regions = PolygonOps.DifferenceToComponents(boundary, Array.Empty<IReadOnlyList<PointD>>())
            .Select(c => new MillingRegion(c.Outer, c.Holes)).ToArray();
        // No inset survives beyond half the smaller source box span. Bound
        // shallow-angle work by geometry before checking the ring inventory.
        double geometricReach = regions.Select(r => Math.Min(r.Outer.Max(p => p.x) - r.Outer.Min(p => p.x),
            r.Outer.Max(p => p.y) - r.Outer.Min(p => p.y)) / 2).DefaultIfEmpty(0).Max();
        double requiredReach = rampBand.Reach;
        // Interior clearing starts at the completed slope's last offset front,
        // exactly where the blue cleanup rows continue the red slope rows. Do
        // not restart distance measurement at the theoretical surface lip
        // outside that front: its cutter-support collar is already slope work.
        double floorInset = requiredReach;
        double fillLimit = options.IncludeInteriorFill ? double.PositiveInfinity : options.EdgeFillWidth;
        // Fill Limit is a centre-to-nearest-floor-edge DISTANCE. Eroding the
        // source by rampReach + L is the same as eroding the completed slope
        // front by L. The shared offset lattice therefore selects this band
        // directly, including complete narrow pockets, without a disc-opening
        // mask that would discard their inner rings. Never halve this distance.
        bool restFill=options.ExcludeDiscSweep&&(options.IncludeInteriorFill||fillLimit>0);
        double reach = Math.Min(geometricReach, restFill ? requiredReach : options.IncludeInteriorFill ? geometricReach
            : requiredReach + fillLimit);
        if (reach < first)
        {
            diagnostic?.Invoke($"stage=outline-ramps event=empty region={regionIndex} reason=no-commandable-closed-inset "
                + $"firstInset={first:R} geometricReach={geometricReach:R} medialDetailRetained=True");
            return new();
        }
        // Fit to the authored band once, independent of a component's size.
        // Smaller components consume a prefix of that same offset lattice.
        double intervals = Math.Floor((reach - first) / pitch + 1e-10);
        if (!double.IsFinite(intervals) || pitch <= 0 || intervals >= RingOffsetEngine.AbsoluteRingLimit)
        {
            diagnostic?.Invoke($"stage=outline-ramps event=failed reason=ring-budget reach={reach:R} pitch={pitch:R} intervals={intervals:R}");
            throw new InvalidOperationException("Full-outline V-carve ramp exceeds the concentric ring inventory.");
        }
        double scheduledInset = intervals * pitch;
        diagnostic?.Invoke($"stage=outline-ramps event=start region={regionIndex} sourceRings={boundary.Count} "
            + $"contourSource={contourSource} outsideAnchors={options.RampOutsideEdges} "
            + $"firstInset={first:R} reach={reach:R} lastInset={first+scheduledInset:R} pitch={pitch:R} surfacePitch={surfacePitch:R} grade={grade:R} "
            + $"authoredFillLimit={options.EdgeFillWidth:R} floorInset={floorInset:R} requiredRampReach={requiredReach:R} "
            + "rampTruncated=False fillLimitAppliesTo=post-slope-nearest-edge-distance fillLimitMeasure=distance-not-diameter medialGraphReplaced=False");
        var rings = MedialAxisGeometry.BuildConcentricFill(regions,
            new RingOffsetOptions(first, pitch) {
                MaximumInset = scheduledInset, RingLimit = (int)intervals + 1,
                MinimumRingArea = 0,
                Tolerances = GeometryTolerances.Default with { ArcTolerance = .001, SimplifyTolerance = 0 },
            }, sweepRadius: 0, preserveTerminalPaths: false, cancellationToken: token);
        double ringsMs = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        // Keep the slope inventory literal. For rest filling, subtract AREAS
        // first and offset their boundaries, including the actual rougher hole.
        // Clipping the old floor rings would lose those closed pocket loops.
        VCarveDiscSweepExclusion? disc = null;
        var restRings=new HashSet<OffsetRing>();
        var floor = rings.Where(r => r.IsClosed && Math.Abs(
                r.OffsetDistance.GetValueOrDefault(first + r.Level * pitch) - floorInset) <= 1 / PathUtils.Scale)
            .Select(r => r.IsHole ? (IReadOnlyList<PointD>)r.Points.AsEnumerable().Reverse().ToArray() : r.Points).ToArray();
        if (restFill&&floor.Length>0)
        {
            // Fill Limit is measured on the finisher's supported centre front,
            // but a roughing disc fits the MATERIAL floor. Testing it inside the
            // smaller centre front compensates for the finishing cutter again
            // and can reject a disc that the real rougher has already swept.
            // Use the same continuous material plateau as Pocket analysis; its
            // physical reserves and outside-edge policy stay in that owner.
            var protectedSlope = Plateau(boundary,options,depth,grade,token);
            var materialFloor = PolygonOps.Difference(boundary,protectedSlope);
            disc = new(materialFloor,options.DiscRadius,token,diagnostic,"shared-material-plateau");
            diagnostic?.Invoke($"stage=disc-sweep event=domain region={regionIndex} "
                + $"centreFrontArea={Math.Abs(floor.Sum(PolygonOps.SignedArea)):R} "
                + $"materialFloorArea={Math.Abs(materialFloor.Sum(PolygonOps.SignedArea)):R} "
                + "domainOwner=VCarveRampSurface.Plateau sharedWith=Pocket.AnalyzeSourcePlateaus "
                + "finisherCompensationAppliedToDisc=False fillLimitReference=completed-slope-front");
            var stock=options.RestFillStock??new VCarvePostedFloorSweep(
                options.RoughingRestChords.Concat(options.ClearedStock?.Chords??Array.Empty<VCarveRoughingChord>()));
            var cleared=stock.AtDepth(depth,token,diagnostic);
            IReadOnlyList<IReadOnlyList<PointD>> band=double.IsPositiveInfinity(fillLimit) ? floor
                : PolygonOps.Difference(floor,PolygonOps.Offset(floor,-fillLimit,.001,0));
            var cleanup=disc.Fill(band,cleared,pitch,fillLimit,token,diagnostic);
            int firstLevel=rings.Max(r=>r.Level)+1;
            var owners=regions.Select(r=>new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(
                new[]{r.Outer}.Concat(r.Holes)))).ToArray();
            foreach(var ring in cleanup)
            {
                var p=ring.Points[0];
                var q=new Point64((long)Math.Round(p.x*PathUtils.Scale),(long)Math.Round(p.y*PathUtils.Scale));
                int owner=Array.FindIndex(owners,index=>index.IsInside(q));
                if(owner<0)throw new InvalidOperationException("Rest fill escaped its original source component.");
                var appended=new OffsetRing(ring.Points,firstLevel+ring.Level,owner,ring.IsHole,true,ring.OffsetDistance);
                rings.Add(appended);restRings.Add(appended);
            }
        }
        if (MillingGeometryTrace.Observer.Value is not null)
        {
            // Read-only diagnosis of the same completed front. Restore hole
            // winding for consumers that inspect the enclosed residual floor.
            MillingGeometryTrace.Capture("post-slope-front",regionIndex,floor.Select(r =>
                new TaggedToolpath(r.Select(p=>new Point3D(p.x,p.y,-depth)).ToArray(),
                    Array.Empty<ToolpathSpan>(),regionIndex,"post-slope-front",null,detectCurves:false)).ToArray());
        }
        long selectionStarted = Stopwatch.GetTimestamp();
        var result = new List<TaggedToolpath>(rings.Count);
        int rampRings = 0, floorRings = 0, floorSpans = 0;
        double selectedFloorLength = 0, lastFloorDistance = 0;
        var floorIndex=restRings.Count>0 ? new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(floor)) : null;
        RingOffsetEngine.BuildNesting(rings);
        var traversal=RingOffsetEngine.BuildTraversal(rings,options.ProfileDirection==VCarveProfileDirection.BottomUp);
        var ringPlan=new VCarveRingOrder.Plan(rings.Select(r=>r.Parent).ToArray(),traversal.Order,
            options.ProfileDirection==VCarveProfileDirection.BottomUp,rings.Select(r=>r.Level).ToArray(),rings.Select(r=>r.RegionIndex).ToArray(),walls:boundary);
        foreach (int ringIndex in traversal.Order)
        {
            token.ThrowIfCancellationRequested();
            var ring=rings[ringIndex];
            if (ring.Points.Count < 3) continue;
            var points = new List<PointD>(ring.Points);
            if (ring.IsClosed && points[^1] != points[0]) points.Add(points[0]);
            double inset = ring.OffsetDistance.GetValueOrDefault(first + ring.Level * pitch);
            bool isRest=restRings.Contains(ring);
            bool isRamp = !isRest&&inset <= requiredReach + 1 / PathUtils.Scale;
            if (isRamp) rampRings++; else floorRings++;
            var stroke=points.Select(p => new Point3D(p.x,p.y,-depth)).ToArray();
            var selected = new List<List<Point3D>> {stroke.ToList()};
            foreach (var fragment in selected)
                result.Add(new TaggedToolpath(fragment,Array.Empty<ToolpathSpan>(),regionIndex,"clearing",null,detectCurves:false)
                    {ConcentricSource=new(ringPlan,ringIndex)});
            if (!isRamp)
            {
                floorSpans+=selected.Count;
                selectedFloorLength += selected.Sum(p=>OpenLength(p.Select(v=>new PointD(v.X,v.Y)).ToArray()));
                if(!isRest)lastFloorDistance = Math.Max(lastFloorDistance, inset - requiredReach);
                else foreach(var p in points)
                {
                    double distance=Math.Sqrt(floorIndex!.DistanceSquared(new Point64(
                        (long)Math.Round(p.x*PathUtils.Scale),(long)Math.Round(p.y*PathUtils.Scale))))/PathUtils.Scale;
                    lastFloorDistance=Math.Max(lastFloorDistance,distance);
                    if(distance>fillLimit+.003)
                        throw new InvalidOperationException("Rest fill exceeded the authored distance from the completed slope front.");
                }
                if (!isRest&&inset > requiredReach + fillLimit + 1 / PathUtils.Scale)
                    throw new InvalidOperationException("Post-slope cleanup exceeded its nearest-edge Fill Limit.");
            }
        }
        if (options.AccessTarget is { } target)
            result = target.Trim(result, profile.PhysicalRadiusAtDepth, token);
        MillingGeometryTrace.Capture("outline-selected",regionIndex,result);
        diagnostic?.Invoke($"stage=outline-ramps event=nominated region={regionIndex} paths={result.Count} "
            + $"points={result.Sum(p => (long)p.Points.Count)} direction={options.ProfileDirection} "
            + "spacingOwner=uniform-pitch orderOwner=shared-ring-tree physicalProtectionRequired=True");
        diagnostic?.Invoke($"stage=post-slope-fill event=nominated region={regionIndex} clearInterior={options.IncludeInteriorFill} "
            + $"nearestEdgeLimit={fillLimit:R} floorInset={floorInset:R} centreRampReach={requiredReach:R} "
            + $"lastFloorDistance={lastFloorDistance:R} rampRings={rampRings} floorRings={floorRings} floorSpans={floorSpans} "
            + $"selectedFloorLength={selectedFloorLength:R} "
            + $"excludeDiscSweep={options.ExcludeDiscSweep} discRadius={options.DiscRadius:R} "
            + $"restAreaContours={restRings.Count} ringClipping=False "
            + $"selection=nearest-edge-offset-band diameterConversion=False openingMask={disc is not null} stockAllowanceAlreadyInBoundary=True "
            + $"ringsMs={ringsMs:R} selectionAndTargetTrimMs={Stopwatch.GetElapsedTime(selectionStarted).TotalMilliseconds:R} "
            + "residualBoundary=complete-last-slope-front theoreticalFloorLipNotUsed=True "
            + "limitOwner=shared-concentric-schedule slopeAdditive=True physicalProtectionRequired=True");
        return result;
    }

    private static double OpenLength(IReadOnlyList<PointD> points)
        => Enumerable.Range(1, Math.Max(0, points.Count - 1)).Sum(i =>
            Math.Sqrt(Math.Pow(points[i].x - points[i - 1].x, 2) + Math.Pow(points[i].y - points[i - 1].y, 2)));

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
        // Prepare only literal source XY queries with the existing bounded,
        // joined field workers. Unprepared/refused sites keep the same serial
        // Depth kernel below; contour order and physical depth arithmetic stay.
        if (field is not null && options.ParallelFloorQueryPreparation)
        {
            long inputSites = source.Sum(path => (long)path.Points.Count)
                + upper.Sum(path => (long)path.Points.Count);
            int prepared = field.PrewarmQueries(source.Concat(upper).SelectMany(path => path.Points)
                .Select(p => new PointD(p.X, p.Y)), inputSites,
                message => Console.WriteLine($"[VCarve.RampSurface] operation='{options.DiagnosticContext}' {message}"),
                "medial-surface-fit");
            Console.WriteLine($"[VCarve.RampSurface] operation='{options.DiagnosticContext}' stage=medial-floor-sites "
                + $"inputSites={inputSites} preparedUniqueSites={prepared} preparationBound=literal-input-inventory "
                + $"remainingQueries=original-serial-kernel sourceXYAndOrderUnchanged=True");
        }
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
