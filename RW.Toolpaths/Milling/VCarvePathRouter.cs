using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Orders independent visits without changing their cutting chords or depth schedule.</summary>
internal static class VCarvePathRouter
{
    private const double LinkBoundaryTolerance = .001;
    internal static List<TaggedToolpath> OrderRegions(List<TaggedToolpath> paths,CancellationToken token,Action<string>? log)
    {
        if(paths.Count<2)return paths;
        var blocks=new List<List<TaggedToolpath>>();
        foreach(var path in paths)
        {
            if(blocks.Count==0||blocks[^1][0].RegionIndex!=path.RegionIndex)blocks.Add(new());
            blocks[^1].Add(path);
        }
        if(blocks.Count<2)return paths;
        var visits=blocks.Select(b=>new TaggedToolpath(new[]{b[0].Points[0],b[^1].Points[^1]},
            Array.Empty<ToolpathSpan>(),0,"region-transfer",null,detectCurves:false)).ToArray();
        var order=ToolpathVisitOrder.Order(visits,Enumerable.Repeat(false,visits.Length).ToArray(),null,token);
        double before=TransferDistance(visits,null),after=TransferDistance(order.Select(i=>visits[i]),null);
        bool accepted=after<before-1e-6;
        log?.Invoke($"stage=region-travel blocks={blocks.Count} beforeTransfer={before:R} afterTransfer={(accepted?after:before):R} "
            +$"accepted={accepted} completeRegionBlocks=True literalCutRampAndReturnTracesPreserved=True");
        if(!accepted)return paths;
        return order.SelectMany(i=>blocks[i]).Select((p,i)=>p with{VisitIdentity=i}).ToList();
    }
    internal static List<TaggedToolpath> Route(
        IReadOnlyList<TaggedToolpath> paths,
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        ToolGeometry tool,
        DepthSchedule depth,
        VCarveOptions options,
        double localEdgeFillWidth,
        Point3D? start,
        CancellationToken cancellationToken = default,
        Action<string>? diagnostic = null,
        Func<Point3D, Point3D, bool>? continuationCutGuard = null,
        Func<Point3D, double>? materialEntryHeight = null)
    {
        if(options.SmoothVCarving && continuationCutGuard is not null && paths.Count>0
            && paths.All(p=>p.ContinuationCircuitDefinition is not null))
        {
            var profile=VCarvePhysicalSweep.ProfileFor(tool,depth.Depth);
            double step=depth.DepthPerPass>0 ? depth.DepthPerPass : depth.Depth;
            // A small positive engagement preserves actual cutter-accessible
            // topology. This is a transit construction scale, not a visual
            // omission tolerance or a relaxation of the authored stepdown.
            double transit=Math.Min(step,VCarveCommandRounding.CommandIncrement);
            bool Allows(Point3D a,Point3D b)=>continuationCutGuard(a with{Z=a.Z-depth.SurfaceZ},b with{Z=b.Z-depth.SurfaceZ});
            var ramp=options.Ramp with{Strategy=RampStrategy.Linear,
                AngleRadians=Math.Min(options.Ramp.AngleRadians>0 ? options.Ramp.AngleRadians : double.PositiveInfinity,
                    options.SmoothRampAngleDegrees*Math.PI/180)};
            Point3D Command(Point3D p)=>new(VCarveCommandRounding.Round((float)p.X),VCarveCommandRounding.Round((float)p.Y),
                VCarveCommandRounding.Round((float)p.Z-(float)depth.SurfaceZ));
            var completedStock=new CompletedCircuitStock(paths,profile,Command,cancellationToken);
            bool ClearedCarrier(int owner,Point3D a,Point3D b)=>options.CompactCollinearCommands
                && completedStock.ContainsRaisedChord(owner,a,b,.1);
            var executed=ToolpathCircuitExecutor.Execute(paths,transit,Math.Max(.1,2*options.StepOver*profile.ClearingDiameter),
                depth,ramp,VCarveToolpaths.FreshStockLayers(paths,depth,cancellationToken,depth.SurfaceZ),Allows,cancellationToken,diagnostic,
                Math.Tan(options.SmoothRampAngleDegrees*Math.PI/180),ClearedCarrier,start);
            diagnostic?.Invoke($"stage=completed-circuit-stock event=complete queries={completedStock.Queries} accepted={completedStock.Accepted} "
                +$"indexedCircuits={completedStock.IndexedCircuits} positiveClearanceMm=0.1 fullInscribedCylinder=True lazySourceChordIndex=True");
            return executed;
        }
        if (options.SmoothVCarving && paths.Count > 0 && paths.All(p => p.ContinuationCircuit))
            return RouteCircuits(paths, depth, options, cancellationToken, diagnostic);
        var result = new List<TaggedToolpath>(paths.Count);
        PolygonOps.BoundarySegmentIndex? sourceBoundary = null;
        // Source seams define the legacy order, orientation, and entry decisions.
        // A shortened closed tail changes only the physical exit used by emission.
        Point3D? routingPrevious = start;
        Point3D? actualPrevious = start;
        var continuation = options.SmoothVCarving && (continuationCutGuard is not null || options.RoughingRestChords.Count>0)
            ? new VCarveContinuationAdapter(tool, depth, options, continuationCutGuard, cancellationToken, diagnostic,paths) : null;
        for (int first = 0; first < paths.Count;)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var representative = paths[first];
            int end = first + 1;
            while (end < paths.Count && paths[end].RegionIndex == representative.RegionIndex
                && paths[end].Category == representative.Category
                && paths[end].ContinuationComponentIndex == representative.ContinuationComponentIndex
                && paths[end].DepthPassIndex == representative.DepthPassIndex)
                end++;
            var group = paths.Skip(first).Take(end - first).ToArray();
            bool[] movable = group.Select(path => CanChangeStart(path)
                || continuation is not null && path.Points.Count >= 2 && Closed(path)
                    && path.Spans.All(span => span.Kind == ToolpathSpanKind.Cut)).ToArray();
            var samples = group.Select((path, index) => RoutingSamples(path, movable[index])).ToArray();
            var incoming=continuation is not null ? actualPrevious : routingPrevious;
            // Large stroke sets use the spatial directed candidate below; an
            // all-pairs tour plus repeated local search is unnecessary there.
            int[] order = group.Length <= 128 ? TravelOptimizer.Order(samples,
                incoming is { } p ? new PointD(p.X, p.Y) : null,
                label: $"vcarve-paths region={representative.RegionIndex} category={representative.Category} depth={representative.DepthPassIndex}",
                cancellationToken: cancellationToken) : Enumerable.Range(0,group.Length).ToArray();
            var routed = Orient(group, movable, order, incoming, cancellationToken);
            double before = TransferDistance(group, incoming);
            double after = TransferDistance(routed, incoming);
            var directed = Orient(group, movable, ToolpathVisitOrder.Order(group,movable,incoming,cancellationToken),
                incoming,cancellationToken);
            double directedDistance=TransferDistance(directed,incoming);
            if(directedDistance<after){routed=directed;after=directedDistance;}
            // Closest-approach TSP is an approximation for open paths with fixed exits.
            // Keep the original order if its realized travel is better, while still
            // considering safe seam changes in that order.
            if (after > before + 1e-6)
            {
                routed = Orient(group, movable, Enumerable.Range(0, group.Length), incoming, cancellationToken);
                after = TransferDistance(routed, incoming);
                if (after > before + 1e-6) { routed = group.ToList(); after = before; }
            }
            // A trailing single-point entry can produce no physical path. Keep
            // the last real visit's seam so that such an empty entry cannot leave
            // the next group's incoming cursor at a shortened closed exit.
            int finalMultiPointVisit = routed.FindLastIndex(path => path.Points.Count > 1);

            ILinkClearance? clearance = null;
            double pitch = 0, radius = 0, layerDepth = 0;
            bool linkableLayer = representative.Category == "clearing"
                && !options.SmoothVCarving
                && group.All(CanChangeStart)
                && group.All(path => path.Points[0].Z == group[0].Points[0].Z)
                && (options.IncludeInteriorFill || localEdgeFillWidth > 0);
            if (linkableLayer)
            {
                layerDepth = depth.SurfaceZ - group[0].Points[0].Z;
                radius = RadiusAtDepth(tool, depth.Depth, layerDepth);
                bool bottom = Math.Abs(layerDepth - depth.Depth) <= 1e-5;
                pitch = tool.VCarveProfile is { RoundedTip: true } physical
                    ? options.StepOver * Math.Min(physical.ClearingDiameter, 2 * radius)
                    : (bottom ? .05 : options.StepOver) * 2 * radius;
                // A band shallower than the mandatory perimeter adds no clearing.
                if (!options.IncludeInteriorFill && localEdgeFillWidth <= radius) pitch = 0;
            }
            int links = 0, rejected = 0;
            double linkLength = 0;
            double actualTransfer = 0;
            long redundantPointsRemoved = 0;
            double redundantCutTravelRemoved = 0;
            int entryExitChanges = 0;
            int partialAxisFallbacks = 0;
            int groupBoundaryExitGuards = 0;
            int legacyContinuationExitGuards = 0;
            int physicalCoincidenceWithoutLegacyContinuation = 0;
            TaggedToolpath? priorInGroup = null;
            for (int visit = 0; visit < routed.Count; visit++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var path = routed[visit];
                bool linkedAtDepth = (continuation is not null ? actualPrevious : routingPrevious) == path.Points[0];
                ToolpathContinuationPlanner.Connection? continued = null;
                if (!linkedAtDepth && actualPrevious == path.Points[0])
                    physicalCoincidenceWithoutLegacyContinuation++;
                if (priorInGroup is { } prior && pitch > 0)
                {
                    Point3D from = prior.Points[^1], to = path.Points[0];
                    double distance = Distance(from, to);
                    if (distance > 1e-9 && distance <= pitch + 1e-6)
                    {
                        if (clearance is null)
                        {
                            var allowed = PolygonOps.Offset(boundary, -radius, arcTolerance: 0, simplifyTolerance: 0);
                            if (!options.IncludeInteriorFill)
                            {
                                var core = PolygonOps.Offset(boundary, -localEdgeFillWidth, arcTolerance: 0, simplifyTolerance: 0);
                                allowed = PolygonOps.Difference(allowed, core);
                            }
                            clearance = LinkClearance.WithinRegion(allowed);
                        }
                        // Offset contours alone approximate circular corner clearance.
                        // Also certify the complete physical cutter capsule against the
                        // original canonical boundary, retaining the link policy's micron
                        // endpoint tolerance for offset/coordinate quantization.
                        sourceBoundary ??= new PolygonOps.BoundarySegmentIndex(
                            Clipper.Union(PathUtils.ToClipper(boundary), FillRule.NonZero));
                        if (clearance.IsTravelSafe(from, to, radius)
                            && sourceBoundary.ContainsCapsule(ToClipper(from), ToClipper(to),
                                Math.Max(0, radius - LinkBoundaryTolerance) * PathUtils.Scale))
                        {
                            result.Add(new TaggedToolpath(new[] { from, to },
                                new[] { new ToolpathSpan(0, 1, ToolpathSpanKind.Link) }, path.RegionIndex,
                                path.Category, path.DepthPassIndex, detectCurves: options.DetectCurves));
                            links++;
                            linkLength += distance;
                            linkedAtDepth = true;
                        }
                        else rejected++;
                    }
                }
                if ((options.SmoothVCarving || depth.StartDepth > 0 || depth.SkipDepthPasses > 0) && !linkedAtDepth)
                {
                    if (continuation is not null && actualPrevious is { } previousExit)
                        continued = continuation.Connect(previousExit, ref path);
                    if (continuation is not null && continued is null)
                        continuation.Reentry(actualPrevious, path.Points[0]);
                    // The cleared starting floor changes entry, not the physical
                    // stock/cutter geometry. Shallow detail above this floor must
                    // start at its own target height rather than plunge below it.
                    // Smooth visits explicitly carry their complete descent from
                    // the stock surface. This also keeps the downstream emitter's
                    // vertical approach out of material, independently of the
                    // authored starting-floor declaration or stock pruning.
                    double entryZ = continued is not null ? continued.Points[^1].Z : options.SmoothVCarving ? depth.SurfaceZ
                        : Math.Max(depth.PassTopZ(0), path.Points[0].Z);
                    if (materialEntryHeight is not null && continued is null)
                        entryZ = Math.Max(entryZ, materialEntryHeight(path.Points[0]));
                    var entryRamp = options.SmoothVCarving
                        ? options.Ramp with
                        {
                            AngleRadians = Math.Min(options.Ramp.AngleRadians > 0
                                ? options.Ramp.AngleRadians : double.PositiveInfinity,
                                options.SmoothRampAngleDegrees * Math.PI / 180),
                            Strategy = RampStrategy.Linear,
                        }
                        : options.Ramp;
                    bool reductionEnabled = options.CompactCollinearCommands && pitch <= 0;
                    bool closedSource = path.Points.Count > 1 && Closed(path);
                    // A final group exit feeds the next group's order/orientation,
                    // and the final region exit feeds VCarveToolpaths' next region.
                    // Retain that seam rather than invent a new routing contract.
                    bool groupBoundaryGuard = continuation is null && closedSource
                        && (visit == routed.Count - 1 || visit == finalMultiPointVisit);
                    // The legacy continuation had no surface entry. Shortening
                    // its predecessor would add a different at-depth approach.
                    bool continuationGuard = continuation is null && closedSource && visit + 1 < routed.Count
                        && path.Points[^1] == routed[visit + 1].Points[0];
                    if (reductionEnabled && groupBoundaryGuard) groupBoundaryExitGuards++;
                    if (reductionEnabled && continuationGuard) legacyContinuationExitGuards++;
                    var entry = RampPlanner.PlanPathEntry(path.Points, entryZ,
                        entryRamp, cancellationToken,
                        detail => diagnostic?.Invoke($"region={path.RegionIndex} category={path.Category} " +
                            $"depthPass={path.DepthPassIndex?.ToString() ?? "none"} startingDepth={depth.StartDepth:R} " +
                            $"startingFloorZ={depth.PassTopZ(0):R} entryZ={entryZ:R} " +
                            $"skipDepthPasses={depth.SkipDepthPasses} smooth={options.SmoothVCarving} " +
                            $"maximumRampAngleDegrees={options.SmoothRampAngleDegrees:R} {detail}"),
                        landAtSourceVertex: options.SmoothVCarving || materialEntryHeight is not null,
                        preserveClosedExit: !reductionEnabled || groupBoundaryGuard || continuationGuard);
                    var entered = new TaggedToolpath(entry.Points, entry.Spans,
                        path.RegionIndex, path.Category, path.DepthPassIndex,
                        path.VisitIdentity, options.DetectCurves) {ContinuationComponentIndex=path.ContinuationComponentIndex,
                        ContinuationPlan=path.ContinuationPlan,ContinuationSourceOrdinal=path.ContinuationSourceOrdinal};
                    result.Add(continued is null ? entered : ToolpathContinuationPlanner.Prepend(entered,
                        continued.Points,clearedTravel:continued.ClearedTravel));
                    redundantPointsRemoved += entry.RedundantPointsRemoved;
                    redundantCutTravelRemoved += entry.RedundantCutTravelRemoved;
                    if (entry.PartialAxisFallback) partialAxisFallbacks++;
                    if (entry.Points.Count > 0 && entry.Points[^1] != path.Points[^1]) entryExitChanges++;
                }
                else result.Add(path);
                // Entry may complete a closed trace at a different source vertex.
                // Continuation and certified links must start where the tool ends.
                var actualPath = result[^1];
                continuation?.Record(actualPath);
                if (!linkedAtDepth && actualPrevious is { } exit && actualPath.Points.Count > 0)
                    actualTransfer += Distance(exit, actualPath.Points[0]);
                priorInGroup = actualPath.Points.Count > 0 ? actualPath : null;
                if (actualPath.Points.Count > 0)
                {
                    actualPrevious = actualPath.Points[^1];
                    routingPrevious = path.Points[^1];
                }
            }
            string finalExit = actualPrevious is { } finalPoint
                ? $"({finalPoint.X:R},{finalPoint.Y:R},{finalPoint.Z:R})" : "none";
            string routingExit = routingPrevious is { } routingPoint
                ? $"({routingPoint.X:R},{routingPoint.Y:R},{routingPoint.Z:R})" : "none";
            diagnostic?.Invoke($"stage=path-routing region={representative.RegionIndex} " +
                $"category={representative.Category} depthPass={representative.DepthPassIndex?.ToString() ?? "none"} " +
                $"paths={group.Length} beforeTransfer={before:R} afterTransfer={after - linkLength:R} " +
                $"transferEstimate=source-visits actualTransfer={actualTransfer:R} " +
                $"orderPolicy=actual-exit-spatial matrixAdmission={group.Length<=128} " +
                $"links={links} linkLength={linkLength:R} rejectedLinks={rejected} " +
                $"redundantPointsRemoved={redundantPointsRemoved} redundantCutTravelRemoved={redundantCutTravelRemoved:R} " +
                $"entryExitChanges={entryExitChanges} partialAxisFallbacks={partialAxisFallbacks} finalExit={finalExit} " +
                $"legacyRoutingExit={routingExit} groupBoundaryExitGuards={groupBoundaryExitGuards} " +
                $"legacyContinuationExitGuards={legacyContinuationExitGuards} " +
                $"physicalCoincidenceWithoutLegacyContinuation={physicalCoincidenceWithoutLegacyContinuation} " +
                $"layerDepth={layerDepth:R} surfaceRadius={radius:R} maxLinkDistance={pitch:R} " +
                $"edgeFillWidth={localEdgeFillWidth:R} capsuleTolerance={LinkBoundaryTolerance:R} sourceChordsPreserved=true");
            first = end;
        }
        continuation?.Report();
        if (options.SmoothVCarving)
        {
            double permitted = Math.Tan(options.SmoothRampAngleDegrees * Math.PI / 180);
            double maximumGrade = 0, maximumExcess = 0;
            long chords = 0;
            foreach (var path in result)
            for (int i = 1; i < path.Points.Count; i++)
            {
                if ((++chords & 1023) == 0) cancellationToken.ThrowIfCancellationRequested();
                var a = path.Points[i - 1]; var b = path.Points[i];
                double run = Distance(a, b), rise = Math.Abs(b.Z - a.Z);
                double excess = rise - permitted * run;
                maximumExcess = Math.Max(maximumExcess, excess);
                if (run > 1e-12) maximumGrade = Math.Max(maximumGrade, rise / run);
                if (excess > 1e-8)
                {
                    diagnostic?.Invoke($"stage=smooth-routing event=reject region={path.RegionIndex} " +
                        $"category={path.Category} chord={i - 1} rise={rise:R} run={run:R} excess={excess:R} " +
                        $"from=({a.X:R},{a.Y:R},{a.Z:R}) to=({b.X:R},{b.Y:R},{b.Z:R})");
                    throw new InvalidOperationException("Smooth V-carve routing exceeded the requested ramp angle.");
                }
            }
            diagnostic?.Invoke($"stage=smooth-routing event=certified paths={result.Count} chords={chords} " +
                $"maximumGrade={maximumGrade:R} requestedGrade={permitted:R} maximumExcess={maximumExcess:R} " +
                "independentEntries=reasoned continuityPolicy=shared-executed-stock coordinateSpace=native");
        }
        return options.SmoothVCarving ? result : ToolpathContinuationPlanner.Reconnect(result,
            "vcarve-legacy", Math.Max(.1, tool.Radius), cancellationToken, diagnostic: diagnostic,
            validateConnection: materialEntryHeight is not null ? continuationCutGuard : null);
    }

    private static List<TaggedToolpath> RouteCircuits(IReadOnlyList<TaggedToolpath> paths,
        DepthSchedule depth, VCarveOptions options, CancellationToken token, Action<string>? log)
    {
        var result = new List<TaggedToolpath>(paths.Count);
        var ramp = options.Ramp with { Strategy = RampStrategy.Linear,
            AngleRadians = Math.Min(options.Ramp.AngleRadians > 0 ? options.Ramp.AngleRadians : double.PositiveInfinity,
                options.SmoothRampAngleDegrees * Math.PI / 180) };
        int entries = 0, continuedLayers = 0, partialAxisTails = 0; long tailReturnPoints=0;
        Point3D? previous = null; int? component = null;
        foreach (var path in paths)
        {
            token.ThrowIfCancellationRequested();
            bool continues = component == path.ContinuationComponentIndex && previous is { } p
                && p.X == path.Points[0].X && p.Y == path.Points[0].Y;
            double entryZ = continues ? previous!.Value.Z : depth.SurfaceZ;
            var entry = RampPlanner.PlanPathEntry(path.Points, entryZ, ramp, token,
                landAtSourceVertex: true, preserveClosedExit: false, diagnostic: null);
            if(entry.PartialAxisFallback)partialAxisTails++;
            // The required cycle ends at its ramp contact. Return over the
            // already-cut short source prefix to the depth circuit's seam;
            // appending another complete circuit is unnecessary.
            if(entry.Points[^1]!=path.Points[0])
            {
                int contact=-1;
                for(int i=0;i<path.Points.Count;i++)if(path.Points[i]==entry.Points[^1]){contact=i;break;}
                if(contact<0)throw new InvalidOperationException("A circuit ramp did not land on a retained source vertex.");
                int from=entry.Points.Count-1;
                for(int i=contact-1;i>=0;i--){entry.Points.Add(path.Points[i]);tailReturnPoints++;}
                entry.Spans.Add(new(from,entry.Points.Count-1,ToolpathSpanKind.Link));
            }
            var entered = new TaggedToolpath(entry.Points, entry.Spans, path.RegionIndex, path.Category,
                path.DepthPassIndex, path.VisitIdentity, detectCurves: false)
                { ContinuationCircuit = true, ContinuationComponentIndex = path.ContinuationComponentIndex };
            if (continues && entered.Points[0] != previous)
                throw new InvalidOperationException("A connected depth circuit lost its executed entry pose.");
            double grade = Math.Tan(options.SmoothRampAngleDegrees * Math.PI / 180);
            for (int i = 1; i < entered.Points.Count; i++)
            {
                if ((i & 1023) == 0) token.ThrowIfCancellationRequested();
                var a = entered.Points[i - 1]; var b = entered.Points[i];
                if (Math.Abs(b.Z - a.Z) > grade * Distance(a, b) + 1e-8)
                    throw new InvalidOperationException("A constructed V-carve circuit exceeded the authored ramp grade.");
            }
            result.Add(entered); previous = entered.Points[^1]; component = path.ContinuationComponentIndex;
            if (continues) continuedLayers++; else entries++;
        }
        log?.Invoke($"stage=construction-routing event=complete circuits={entries} surfaceEntries={entries} "
            + $"continuousDepthPasses={continuedLayers} outputPaths={result.Count} "
            + $"partialAxisTails={partialAxisTails} seamReturnPoints={tailReturnPoints} "
            + "sameXYDepthPrecursor=True sourceChordsRetained=True emittedGraphSearch=False oneEntryPerCertifiedComponent=True");
        return result;
    }

    private static bool CanChangeStart(TaggedToolpath path)
        => path.Points.Count >= 2
            && path.Spans.All(span => span.Kind == ToolpathSpanKind.Cut)
            && path.Points.All(point => point.Z == path.Points[0].Z);

    private static bool Closed(TaggedToolpath path) => path.Points[0] == path.Points[^1];

    private static IReadOnlyList<PointD> RoutingSamples(TaggedToolpath path, bool movable)
    {
        if (!movable) return new[] { new PointD(path.Points[0].X, path.Points[0].Y) };
        if (!Closed(path)) return new[] { new PointD(path.Points[0].X, path.Points[0].Y), new PointD(path.Points[^1].X, path.Points[^1].Y) };
        int count = path.Points.Count - 1, take = Math.Min(16, count);
        return Enumerable.Range(0, take).Select(i => path.Points[(int)((long)i * count / take)])
            .Select(point => new PointD(point.X, point.Y)).ToArray();
    }

    private static List<TaggedToolpath> Orient(TaggedToolpath[] paths, bool[] movable,
        IEnumerable<int> order, Point3D? start, CancellationToken cancellationToken)
    {
        var result = new List<TaggedToolpath>(paths.Length);
        Point3D? previous = start;
        foreach (int index in order)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var path = paths[index];
            if (movable[index] && previous is { } from)
            {
                List<Point3D>? points = null;
                if (Closed(path))
                {
                    int best = 0;
                    double bestDistance = Distance(from, path.Points[0]);
                    for (int i = 1; i < path.Points.Count - 1; i++)
                    {
                        double distance = Distance(from, path.Points[i]);
                        if (distance < bestDistance) { best = i; bestDistance = distance; }
                    }
                    if (best != 0)
                    {
                        int count = path.Points.Count - 1;
                        points = Enumerable.Range(0, count).Select(i => path.Points[(best + i) % count]).ToList();
                        points.Add(points[0]);
                    }
                }
                else if (Distance(from, path.Points[^1]) < Distance(from, path.Points[0]))
                    points = path.Points.Reverse().ToList();
                if (points is not null)
                    path = new TaggedToolpath(points, Array.Empty<ToolpathSpan>(), path.RegionIndex,
                        path.Category, path.DepthPassIndex, path.VisitIdentity, path.DetectCurves)
                        {ContinuationComponentIndex=path.ContinuationComponentIndex,ContinuationPlan=path.ContinuationPlan,
                            ContinuationSourceOrdinal=path.ContinuationSourceOrdinal};
            }
            result.Add(path);
            previous = path.Points[^1];
        }
        return result;
    }

    private static double TransferDistance(IEnumerable<TaggedToolpath> paths, Point3D? start)
    {
        double distance = 0;
        Point3D? previous = start;
        foreach (var path in paths)
        {
            if (previous is { } from) distance += Distance(from, path.Points[0]);
            previous = path.Points[^1];
        }
        return distance;
    }

    private static double Distance(Point3D a, Point3D b)
        => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

    private static Point64 ToClipper(Point3D point)
        => new((long)Math.Round(point.X * PathUtils.Scale), (long)Math.Round(point.Y * PathUtils.Scale));

    private static double RadiusAtDepth(ToolGeometry tool, double maximumDepth, double depth)
    {
        if (tool.VCarveProfile is { } physical) return physical.RadiusAtDepth(depth);
        double bottom = Math.Max(0, tool.BottomRadius ?? 0);
        double top = Math.Max(bottom, tool.TopRadius ?? bottom + Math.Tan((tool.TipAngleRadians ?? 0) / 2) * maximumDepth);
        double slope = Math.Max(1e-9, (top - bottom) / Math.Max(1e-9, tool.ConeLength ?? maximumDepth));
        return Math.Min(top, bottom + slope * depth);
    }
}
