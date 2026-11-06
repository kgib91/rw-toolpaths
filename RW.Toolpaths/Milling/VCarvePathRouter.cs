using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Orders independent visits without changing their cutting chords or depth schedule.</summary>
internal static partial class VCarvePathRouter
{
    // Read-only per-execution capture of the actual protected routing request.
    // Diagnostic observers may retain it for bounded, fixed-source comparisons;
    // no caller-supplied route or clearance result enters production generation.
    internal sealed record Request(IReadOnlyList<TaggedToolpath> Paths,
        IReadOnlyList<IReadOnlyList<PointD>> Boundary, ToolGeometry Tool, DepthSchedule Depth,
        VCarveOptions Options, double EdgeFillWidth, Point3D? Start,
        Func<Point3D, Point3D, bool>? CutGuard, Func<Point3D, double>? EntryHeight);
    internal static readonly AsyncLocal<Action<Request>?> Observer = new();
    // Controlled same-source routing comparisons in friend diagnostics/tests.
    // This can select the legacy heuristic; it grants no geometry/stock proof.
    internal static readonly AsyncLocal<bool?> ConnectionOptimizationOverride = new();
    internal static readonly AsyncLocal<bool?> CutRetraceOptimizationOverride = new();
    internal static readonly AsyncLocal<bool?> AirRampFeedOptimizationOverride = new();
    // Opt-in ordering of complete nonconcentric clearing sources. Ring jobs
    // retain their constructor order; typed fixed-seam tours retain legacy.
    internal static readonly AsyncLocal<bool?> SpatialIndependentSourcesOverride = new();
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
        int unanchoredFastEntries=0;
        for(int i=1;i<order.Length;i++){
            var block=blocks[order[i]];
            if(!block.Any(p=>p.Spans.Any(s=>s.Kind==ToolpathSpanKind.ClearedTravel)))continue;
            var from=blocks[order[i-1]][^1].Points[^1];var to=block[0].Points[0];
            if(!MillingTransitionPlanner.RequiresPlanarTraverse(from,to)
                &&((float)from.X!=(float)to.X||(float)from.Y!=(float)to.Y))unanchoredFastEntries++;
        }
        bool accepted=after<before-1e-6&&unanchoredFastEntries==0;
        log?.Invoke($"stage=region-travel blocks={blocks.Count} beforeTransfer={before:R} afterTransfer={(accepted?after:before):R} "
            +$"accepted={accepted} unanchoredFastEntries={unanchoredFastEntries} completeRegionBlocks=True literalCutRampAndReturnTracesPreserved=True");
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
        if (Observer.Value is { } observer)
            observer(new(paths, boundary, tool, depth, options, localEdgeFillWidth, start,
                continuationCutGuard, materialEntryHeight));
        var roughingPreparation=options.SmoothVCarving&&continuationCutGuard is not null
            &&options.RoughingRestChords.Count>0&&options.OwnedRouteRoughingValues is not null
            &&VCarveContinuationAdapter.RoughingPreparationEnabled.Value!=false
            ?new VCarveContinuationAdapter.RoughingPreparation(options.RoughingRestChords,ownedSnapshot:options.OwnedRouteRoughingValues):null;
        bool routeCompleted=false;
        try
        {
        if(options.SmoothVCarving&&continuationCutGuard is not null&&paths.Any(p=>p.ConcentricSource is not null)
            &&paths.All(p=>p.RegionIndex==paths[0].RegionIndex))
        {
            var previousSweep=VCarveRingOrder.SpatialSweepOverride.Value;
            var previousMixed=VCarveRingRouter.SpatialMixedSourcesOverride.Value;
            var previousIndependent=SpatialIndependentSourcesOverride.Value;
            var automatic=previousSweep is null?AutomaticSpatialSweep(paths,depth,options,cancellationToken):null;
            try{
                if(automatic is not null){
                    VCarveRingOrder.SpatialSweepOverride.Value=automatic;
                    VCarveRingRouter.SpatialMixedSourcesOverride.Value=previousMixed??true;
                    SpatialIndependentSourcesOverride.Value=previousIndependent??true;
                    diagnostic?.Invoke($"stage=spatial-routing-policy event=selected owner=automatic-large-single-pass-region axis={automatic.Axis} windowMm={automatic.WindowMm:R} "
                        +"scope=complete-source-region sourceSplitting=False configuredDepthDirectionUnchanged=True geometricOrTravelAuthority=False");
                }
                var ringRouted=VCarveRingRouter.Route(paths,tool,depth,options,start,continuationCutGuard,materialEntryHeight,
                    cancellationToken,diagnostic,(other,cursor)=>RouteCore(other,boundary,tool,depth,options,
                        localEdgeFillWidth,cursor,cancellationToken,diagnostic,continuationCutGuard,materialEntryHeight,roughingPreparation));
                routeCompleted=true;return ringRouted;
            }finally{
                VCarveRingOrder.SpatialSweepOverride.Value=previousSweep;
                VCarveRingRouter.SpatialMixedSourcesOverride.Value=previousMixed;
                SpatialIndependentSourcesOverride.Value=previousIndependent;
            }
        }
        var sourceRouted=RouteCore(paths,boundary,tool,depth,options,localEdgeFillWidth,start,cancellationToken,diagnostic,
            continuationCutGuard,materialEntryHeight,roughingPreparation);
        routeCompleted=true;return sourceRouted;
        }
        finally
        {
            if(routeCompleted)roughingPreparation?.Report(diagnostic);
            else try{roughingPreparation?.Report(diagnostic);}catch{/* Keep the original routing/cancellation failure. */}
        }
    }
    private static List<TaggedToolpath> RouteCore(IReadOnlyList<TaggedToolpath> paths,
        IReadOnlyList<IReadOnlyList<PointD>> boundary,ToolGeometry tool,DepthSchedule depth,VCarveOptions options,
        double localEdgeFillWidth,Point3D? start,CancellationToken cancellationToken,Action<string>? diagnostic,
        Func<Point3D,Point3D,bool>? continuationCutGuard,Func<Point3D,double>? materialEntryHeight,
        VCarveContinuationAdapter.RoughingPreparation? roughingPreparation)
    {
        bool branchFinishing=!options.SmoothVCarving&&!options.DetectCurves&&options.Motion is not null
            &&continuationCutGuard is not null&&materialEntryHeight is not null
            &&depth.StartDepth>=depth.Depth&&depth.Depth>0;
        if(branchFinishing)paths=paths.Select((p,i)=>p with{ContinuationSourceOrdinal=i}).ToArray();
        var result = new List<TaggedToolpath>(paths.Count);
        var motionCost=options.Motion is null?null:new ToolpathMotionCost(options.Motion,depth.SurfaceZ);
        int entryCostRefusals=0;double entryCostSaving=0;
        PolygonOps.BoundarySegmentIndex? sourceBoundary = null;
        // Ordinary routing retains its legacy seam cursor. Smooth routing uses
        // the literal completed exit for following entries and connections.
        Point3D? routingPrevious = start;
        Point3D? actualPrevious = start;
        VCarveContinuationAdapter? continuation=null;
        for (int first = 0; first < paths.Count;)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var representative = paths[first];
            if(options.SmoothVCarving && continuationCutGuard is not null && (first==0
                || paths[first-1].RegionIndex!=representative.RegionIndex
                || paths[first-1].ContinuationComponentIndex!=representative.ContinuationComponentIndex))
            {
                continuation?.Report();
                var family=paths.Skip(first).TakeWhile(p=>p.RegionIndex==representative.RegionIndex
                    &&p.ContinuationComponentIndex==representative.ContinuationComponentIndex).ToArray();
                continuation=new(tool,depth,options,continuationCutGuard,cancellationToken,diagnostic,family,roughingPreparation);
                if(result.Count>0)continuation.Record(result[^1]);
            }
            int end = first + 1;
            while (end < paths.Count && paths[end].RegionIndex == representative.RegionIndex
                && paths[end].Category == representative.Category
                && paths[end].ContinuationComponentIndex == representative.ContinuationComponentIndex
                && paths[end].DepthPassIndex == representative.DepthPassIndex)
                end++;
            var group = paths.Skip(first).Take(end - first).ToArray();
            bool fixedTypedCircuit=group.Any(path=>(path.ContinuationCircuitDefinition is not null||path.ContinuationCircuit)
                &&!CanChangeStart(path));
            bool independentClearingOrder=SpatialIndependentSourcesOverride.Value==true&&options.SmoothVCarving
                &&representative.Category=="clearing"&&group.All(path=>path.ConcentricSource is null)&&!fixedTypedCircuit;
            bool[] movable = group.Select(path => CanChangeStart(path)
                || options.SmoothVCarving && path.Points.Count >= 2 && Closed(path)
                    && path.Spans.All(span => span.Kind == ToolpathSpanKind.Cut)).ToArray();
            var samples = group.Select((path, index) => RoutingSamples(path, movable[index])).ToArray();
            var incoming=options.SmoothVCarving ? actualPrevious : routingPrevious;
            // Large stroke sets use the spatial directed candidate below; an
            // all-pairs tour plus repeated local search is unnecessary there.
            bool keepOffsetOrder = options.SmoothVCarving && representative.Category == "clearing"&&!independentClearingOrder;
            int[] order = !keepOffsetOrder && group.Length <= 128 ? TravelOptimizer.Order(samples,
                incoming is { } p ? new PointD(p.X, p.Y) : null,
                label: $"vcarve-paths region={representative.RegionIndex} category={representative.Category} depth={representative.DepthPassIndex}",
                cancellationToken: cancellationToken) : Enumerable.Range(0,group.Length).ToArray();
            var routed = Orient(group, movable, order, incoming, cancellationToken);
            double before = TransferDistance(group, incoming);
            double after = TransferDistance(routed, incoming);
            if (!keepOffsetOrder)
            {
                var directed = Orient(group, movable, ToolpathVisitOrder.Order(group,movable,incoming,cancellationToken),
                    incoming,cancellationToken);
                double directedDistance=TransferDistance(directed,incoming);
                if(directedDistance<after){routed=directed;after=directedDistance;}
            }
            // Closest-approach TSP is an approximation for open paths with fixed exits.
            // Keep the original order if its realized travel is better, while still
            // considering safe seam changes in that order.
            if (!keepOffsetOrder && after > before + 1e-6)
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
                var independentSource=path;
                bool linkedAtDepth = (options.SmoothVCarving?actualPrevious:routingPrevious) == path.Points[0];
                ToolpathContinuationPlanner.Connection? continued=null;
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
                    RampedPath? baselineEntry=null;TaggedToolpath? independentEntry=null;
                    if(continuation is not null && motionCost is not null)
                    {
                        double baselineZ=materialEntryHeight is null ? depth.SurfaceZ : Math.Max(path.Points[0].Z,materialEntryHeight(path.Points[0]));
                        try
                        {
                            baselineEntry=RampPlanner.PlanPathEntry(path.Points,baselineZ,
                                options.Ramp with{AngleRadians=Math.Min(options.Ramp.AngleRadians>0?options.Ramp.AngleRadians:double.PositiveInfinity,
                                    options.SmoothRampAngleDegrees*Math.PI/180),Strategy=RampStrategy.Linear},cancellationToken,null,
                                landAtSourceVertex:true,preserveClosedExit:false,preserveOpenExit:false);
                            independentEntry=new TaggedToolpath(baselineEntry.Points,baselineEntry.Spans,path.RegionIndex,path.Category,
                                path.DepthPassIndex,path.VisitIdentity,options.DetectCurves)
                                {ContinuationComponentIndex=path.ContinuationComponentIndex,ContinuationPlan=path.ContinuationPlan,
                                    ContinuationSourceOrdinal=path.ContinuationSourceOrdinal};
                        }
                        catch(InvalidOperationException error)
                        {
                            // An unavailable independent entry cannot disqualify
                            // a separately proved continuation. If that also fails,
                            // normal entry construction below retains its error.
                            diagnostic?.Invoke($"stage=entry-cost alternative=unavailable reason={error.Message} provedContinuationStillRequired=True");
                        }
                    }
                    if(continuation is not null && actualPrevious is { } from)
                        continued=continuation.Connect(from,ref path,independentEntry);
                    if(continuation is not null && continued is null)continuation.Reentry(actualPrevious,path.Points[0]);
                    // The cleared starting floor changes entry, not the physical
                    // stock/cutter geometry. Shallow detail above this floor must
                    // start at its own target height rather than plunge below it.
                    // With prior-removal evidence, cutting entry uses certified local contact.
                    // Otherwise original stock top remains the datum. Safe transfer/retract
                    // clearance is owned independently by the downstream emitter.
                    double entryZ = continued is not null?continued.Points[^1].Z:materialEntryHeight is not null ? Math.Max(path.Points[0].Z,materialEntryHeight(path.Points[0])) : options.SmoothVCarving ? depth.SurfaceZ
                        : Math.Max(depth.PassTopZ(0), path.Points[0].Z);
                    if (materialEntryHeight is not null)
                    {
                        double required=materialEntryHeight(path.Points[0]);
                        if(continued is not null&&required>entryZ)
                        {
                            diagnostic?.Invoke("stage=continuation event=reentry owner=vcarve reason=material-entry-height sourceEntryRetained=True");
                            continued=null;path=independentSource;entryZ=Math.Max(path.Points[0].Z,required);
                        }
                        entryZ=Math.Max(entryZ,required);
                    }
                    var entryRamp = options.SmoothVCarving
                        ? options.Ramp with
                        {
                            AngleRadians = Math.Min(options.Ramp.AngleRadians > 0
                                ? options.Ramp.AngleRadians : double.PositiveInfinity,
                                options.SmoothRampAngleDegrees * Math.PI / 180),
                            Strategy = RampStrategy.Linear,
                        }
                        : options.Ramp;
                    bool reductionEnabled = options.SmoothVCarving || options.CompactCollinearCommands && pitch <= 0;
                    bool closedSource = path.Points.Count > 1 && Closed(path);
                    // Ordinary routing still has consumers of its original seam.
                    // Smooth continuation and following groups use the actual exit.
                    bool groupBoundaryGuard = !options.SmoothVCarving && closedSource
                        && (visit == routed.Count - 1 || visit == finalMultiPointVisit);
                    // The legacy continuation had no surface entry. Shortening
                    // its predecessor would add a different at-depth approach.
                    bool continuationGuard = !options.SmoothVCarving && closedSource && visit + 1 < routed.Count
                        && path.Points[^1] == routed[visit + 1].Points[0];
                    if (reductionEnabled && groupBoundaryGuard) groupBoundaryExitGuards++;
                    if (reductionEnabled && continuationGuard) legacyContinuationExitGuards++;
                    var entry = continued is null && baselineEntry is not null ? baselineEntry : RampPlanner.PlanPathEntry(path.Points, entryZ,
                        entryRamp, cancellationToken,
                        detail => diagnostic?.Invoke($"region={path.RegionIndex} category={path.Category} " +
                            $"depthPass={path.DepthPassIndex?.ToString() ?? "none"} startingDepth={depth.StartDepth:R} " +
                            $"startingFloorZ={depth.PassTopZ(0):R} entryZ={entryZ:R} " +
                            $"skipDepthPasses={depth.SkipDepthPasses} smooth={options.SmoothVCarving} " +
                            $"maximumRampAngleDegrees={options.SmoothRampAngleDegrees:R} {detail}"),
                        landAtSourceVertex: options.SmoothVCarving || materialEntryHeight is not null,
                        preserveClosedExit: !reductionEnabled || groupBoundaryGuard || continuationGuard,
                        preserveOpenExit: !options.SmoothVCarving);
                    var entered = new TaggedToolpath(entry.Points, entry.Spans,
                        path.RegionIndex, path.Category, path.DepthPassIndex,
                        path.VisitIdentity, options.DetectCurves) {ContinuationComponentIndex=path.ContinuationComponentIndex,
                        ContinuationPlan=path.ContinuationPlan,ContinuationSourceOrdinal=path.ContinuationSourceOrdinal};
                    var selected=continued is null?entered:ToolpathContinuationPlanner.Prepend(entered,continued.Points,
                        clearedTravel:continued.ClearedTravel);
                    if(continued is not null && independentEntry is {} baseline && actualPrevious is {} departure)
                    {
                        if(!ToolpathMotionCost.PreferContinuation(motionCost,departure,baseline,selected,out double beforeCost,out double afterCost))
                        {
                            if(++entryCostRefusals<=8)diagnostic?.Invoke($"stage=entry-cost decision=reenter continuationSec={afterCost:R} entrySec={beforeCost:R} completeRetainedTrace=True policyOwner=RW.Toolpaths");
                            selected=baseline;entry=baselineEntry!;path=independentSource;
                        }
                        else entryCostSaving+=beforeCost-afterCost;
                    }
                    result.Add(selected);
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
                $"component={representative.ContinuationComponentIndex} category={representative.Category} depthPass={representative.DepthPassIndex?.ToString() ?? "none"} " +
                $"paths={group.Length} beforeTransfer={before:R} afterTransfer={after - linkLength:R} " +
                $"transferEstimate=source-visits actualTransfer={actualTransfer:R} " +
                $"orderPolicy=actual-exit-spatial matrixAdmission={group.Length<=128} " +
                $"independentClearingOrder={independentClearingOrder} fixedTypedCircuitOrderRetained={keepOffsetOrder&&fixedTypedCircuit} " +
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
        diagnostic?.Invoke($"stage=entry-cost event=complete owner=vcarve machineContext={motionCost is not null} refused={entryCostRefusals} selectedSavingSec={entryCostSaving:R} completeBridgeEntryAndCut=True policyOwner=RW.Toolpaths");
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
                "continuityPolicy=shared-executed-stock cuttingConnectors=runtime-certified coordinateSpace=native");
        }
        if(options.SmoothVCarving)return result;
        var independentVisits=ToolpathContinuationPlanner.Reconnect(result,
            "vcarve-legacy", Math.Max(.1, tool.Radius), cancellationToken, diagnostic: diagnostic,
            validateConnection: materialEntryHeight is not null ? continuationCutGuard : null,
            motion:options.Motion,surface:depth.SurfaceZ);
        return branchFinishing?ToolpathSourceTraversal.FinishingBranches(paths,result,independentVisits,tool.Radius,
            Math.Tan(options.Ramp.AngleRadians),continuationCutGuard!,materialEntryHeight!,options.Motion!,depth.SurfaceZ,
            start,cancellationToken,diagnostic):independentVisits;
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
