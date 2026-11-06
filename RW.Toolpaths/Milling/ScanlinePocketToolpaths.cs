using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Flat-depth raster pockets, bounded by the same physical-cutter inset as offset pockets.</summary>
public static class ScanlinePocketToolpaths
{
    internal static readonly AsyncLocal<bool?> RasterLookaheadEnabled=new();
    internal sealed record RasterInsetPreparation(List<List<PointD>> Inset,long DifferenceTicks,long OffsetTicks,long ClipTicks)
    {
        internal long EstimatedBytes=>256L+64L*Inset.Count+32L*Inset.Sum(p=>(long)p.Count);
    }
    // One complete owned geometry result, never stock/proof or query history.
    // Errors are observed only at the original region's ordered consumption;
    // an earlier perimeter failure cancels and joins this optional work first.
    internal sealed class RasterInsetLookahead:IDisposable
    {
        internal const long MaximumBytes=64L<<20;
        private readonly CancellationTokenSource lifetime;
        private Task<RasterInsetPreparation?>? task;
        private readonly int region;
        private bool taken,disposed;
        internal long ComputeTicks,WaitTicks,RetainedBytes;
        internal bool Oversized,Consumed,Joined;
        internal RasterInsetLookahead(int region,Func<CancellationToken,RasterInsetPreparation> prepare,
            CancellationToken token,long maximumBytes=MaximumBytes)
        {
            if(maximumBytes<0)throw new ArgumentOutOfRangeException(nameof(maximumBytes));
            this.region=region;lifetime=CancellationTokenSource.CreateLinkedTokenSource(token);
            task=Task.Run(()=>
            {
                using var owned=GeometryParallelism.EnterWorker();long started=Stopwatch.GetTimestamp();
                try
                {
                    lifetime.Token.ThrowIfCancellationRequested();var value=prepare(lifetime.Token);
                    lifetime.Token.ThrowIfCancellationRequested();long bytes=value.EstimatedBytes;
                    if(bytes>maximumBytes){Oversized=true;return null;}
                    RetainedBytes=bytes;return value;
                }
                finally{ComputeTicks=Stopwatch.GetTimestamp()-started;}
            },lifetime.Token);
        }
        internal RasterInsetPreparation? Take(int index,CancellationToken token)
        {
            token.ThrowIfCancellationRequested();if(index!=region||taken)return null;
            taken=true;long started=Stopwatch.GetTimestamp();
            try
            {
                var value=task!.GetAwaiter().GetResult();Joined=true;token.ThrowIfCancellationRequested();
                Consumed=value is not null;return value;
            }
            finally{WaitTicks+=Stopwatch.GetTimestamp()-started;Joined=true;task=null;}
        }
        public void Dispose()
        {
            if(disposed)return;disposed=true;lifetime.Cancel();
            try{task?.GetAwaiter().GetResult();}
            catch{/* Earlier ordered work/caller cancellation owns the public error. */}
            finally
            {
                task=null;Joined=true;lifetime.Dispose();
                Console.WriteLine($"[Pocket.Scanline.Lookahead] event=closed region={region} consumed={Consumed} oversized={Oversized} "
                    +$"retainedBytes={RetainedBytes} computeMs={1000d*ComputeTicks/Stopwatch.Frequency:F3} waitMs={1000d*WaitTicks/Stopwatch.Frequency:F3} "
                    +"workers=1 workersJoined=True computeTimeOverlapsPerimeter=True errorPublication=original-region-order");
            }
        }
    }
    internal static RasterInsetPreparation PrepareRasterInset(MillingRegion region,double insetDistance,
        double arcTolerance,double simplifyTolerance,IReadOnlyList<PointD>? clip,CancellationToken token)
    {
        token.ThrowIfCancellationRequested();long started=Stopwatch.GetTimestamp();
        var material=PolygonOps.Difference(new[]{region.Outer},region.Holes);
        long difference=Stopwatch.GetTimestamp()-started;token.ThrowIfCancellationRequested();started=Stopwatch.GetTimestamp();
        var inset=PolygonOps.Offset(material,insetDistance,arcTolerance,simplifyTolerance);
        long offset=Stopwatch.GetTimestamp()-started;token.ThrowIfCancellationRequested();started=Stopwatch.GetTimestamp();
        if(clip is {Count:>=3})inset=PolygonOps.Intersect(inset,new[]{clip});
        long clipping=Stopwatch.GetTimestamp()-started;token.ThrowIfCancellationRequested();
        return new(inset,difference,offset,clipping);
    }
    internal static RasterInsetLookahead? BeginRasterLookahead(IReadOnlyList<MillingRegion> regions,ToolGeometry tool,
        PocketOptions options,CancellationToken token)
    {
        if(RasterLookaheadEnabled.Value is false||GeometryParallelism.WorkerLimit<=1||!options.SmoothRoughing
            ||!options.IndependentRasterEntries||options.ProtectedBoundary is not {Count:>0}
            ||options.LinkClearance is not null||options.Motion?.ClearedTravelGuard is not null)return null;
        long minimum=RasterLookaheadEnabled.Value is true?1:4096;int selected=-1;long most=0;
        for(int i=0;i<regions.Count;i++)
        {
            token.ThrowIfCancellationRequested();long points=regions[i].Outer.Count+regions[i].Holes.Sum(p=>(long)p.Count);
            if(points>=minimum&&points<=500_000&&points>most){most=points;selected=i;}
        }
        if(selected<0)return null;
        var original=regions[selected];
        var captured=new MillingRegion(original.Outer.ToArray(),original.Holes.Select(p=>(IReadOnlyList<PointD>)p.ToArray()).ToArray());
        var clip=options.ClipBoundary?.ToArray();double distance=-(tool.Radius+options.StockToLeave);
        double arc=options.Tolerances.ArcTolerance,simplify=options.Tolerances.SimplifyTolerance;
        Console.WriteLine($"[Pocket.Scanline.Lookahead] event=start region={selected} inputPoints={most} workers=1 maximumBytes={RasterInsetLookahead.MaximumBytes} "
            +"phase=literal-difference-offset-clip perimeterOverlap=True originalOperandSnapshot=True opaqueCallbacks=False nestedGeometryWorkers=1");
        return new(selected,ct=>PrepareRasterInset(captured,distance,arc,simplify,clip,ct),token);
    }
    public static ToolpathPlan Generate(IReadOnlyList<MillingRegion> regions, ToolGeometry tool,
        DepthSchedule depth, PocketOptions options, double angleDegrees = 0,
        CancellationToken cancellationToken = default)
    {
        if (!double.IsFinite(options.StockToLeave))
            throw new ArgumentOutOfRangeException(nameof(options), "Pocket edge offset must be finite.");
        var requestedSearchLimits=new ToolpathContinuationSearchLimits(options.FlatContinuationNearbyGoals,options.FlatContinuationVisibilityProofs);
        var searchLimits=options.FlatContinuationNearbyGoals==ToolpathContinuationSearchLimits.DefaultNearbyGoals
            &&options.FlatContinuationVisibilityProofs==ToolpathContinuationSearchLimits.DefaultVisibilityProofs?null:requestedSearchLimits;
        if (regions.Count == 0 || tool.Radius <= 0 || depth.Depth <= 0) return ToolpathPlan.Empty;
        long generateStarted=Stopwatch.GetTimestamp(),stamp;
        long accessSetupTicks=0,protectionSubtractTicks=0,perimeterTicks=0,rasterDifferenceTicks=0,rasterOffsetTicks=0,
            rasterClipTicks=0,rasterPreparationTicks=0,rasterRowsTicks=0,legalRunsTicks=0,entryLayoutTicks=0,orderingTicks=0,
            accessTrimTicks=0,simplifyTicks=0,sourceClearanceTicks=0,reconnectTicks=0,finalAccessVerifyTicks=0,
            protectionVerifyTicks=0,sweptTicks=0,rasterLookaheadWaitTicks=0;
        bool completed=false;string activeStage="access-setup";int inputRegions=regions.Count,outputPaths=0;
        try
        {
        // Raster endpoints and the closing perimeter share the physical inset.
        // Coarse display tolerances can chord through a protected island corner.
        options = options with { Tolerances = options.Tolerances with {
            ArcTolerance = Math.Min(options.Tolerances.ArcTolerance, MachiningAccuracy.ArtworkChord),
            SimplifyTolerance = Math.Min(options.Tolerances.SimplifyTolerance, MachiningAccuracy.ArtworkChord) } };
        stamp=Stopwatch.GetTimestamp();
        var access = options.BoundaryAccess?.Resolve(regions,tool,depth,options.SmoothRoughing,
            options.SmoothRampAngleDegrees,options.StockToLeave,cancellationToken);
        if(access!=null)
        {
            var prepared=access.PreparePocket(tool.Radius,options.StockToLeave,PocketContinuationRouter.KnownAirReserve,cancellationToken);
            regions=prepared.Regions;
            options=options with {KnownEmptyBoundary=prepared.EmptyBoundary,
                KnownEmptyFloor=depth.SurfaceZ-depth.Depth, ClipBoundary=options.ClipBoundary ?? access.PartBoundary};
        }
        accessSetupTicks=Stopwatch.GetTimestamp()-stamp;
        activeStage="protection-subtract";stamp=Stopwatch.GetTimestamp();
        regions = PocketProtection.Subtract(regions, options, cancellationToken,preparedScanlineDifference:true);
        protectionSubtractTicks=Stopwatch.GetTimestamp()-stamp;
        if (regions.Count == 0){completed=true;return ToolpathPlan.Empty;}
        double pitch = tool.Radius * 2 * options.StepOver;
        if (!double.IsFinite(pitch) || pitch <= 0) throw new ArgumentException("Scanline stepover must be positive.");
        Console.WriteLine($"[PocketToolpaths.EdgeOffset] mode=scanline toolRadius={tool.Radius:R} "
            + $"edgeOffset={options.StockToLeave:R} firstInset={tool.Radius + options.StockToLeave:R} "
            + $"stepOver={pitch:R} regions={regions.Count} arcTolerance={options.Tolerances.ArcTolerance:R} "
            + $"simplifyTolerance={options.Tolerances.SimplifyTolerance:R}");
        double angle = angleDegrees * Math.PI / 180, cos = Math.Cos(angle), sin = Math.Sin(angle);
        PointD Rotate(PointD p) => new(p.x * cos + p.y * sin, -p.x * sin + p.y * cos);
        PointD World(PointD p) => new(p.x * cos - p.y * sin, p.x * sin + p.y * cos);
        var paths = new List<TaggedToolpath>();
        var planarRuns = new List<List<PointD>>();
        // The compensated perimeter closes the scalloped strip left at raster endpoints.
        Console.WriteLine($"[Pocket.Scanline.Perimeter] independentEntries={options.IndependentRasterEntries} "
            +$"owner={(access!=null&&!options.IndependentRasterEntries?"concentric-first-front":"inside-profile")} actualAccessAndProtectionAuditRequired=True");
        using var rasterLookahead=BeginRasterLookahead(regions,tool,options,cancellationToken);
        activeStage="perimeter";stamp=Stopwatch.GetTimestamp();
        var boundary = access != null && !options.IndependentRasterEntries
            ? PocketToolpaths.Generate(regions,tool,depth,options with { BoundaryAccess=null,MaxRings=1,
                InsideOut=false,SpiralChaining=false },cancellationToken)
            : ProfileToolpaths.Generate(regions, tool, depth, new ProfileOptions { Motion=options.Motion,DetectCurves=false,
            Side = ProfileSide.Inside, StockToLeave = options.StockToLeave, Ramp = options.Ramp,
            Tolerances = options.Tolerances, ClipBoundary = options.ClipBoundary,
            CalculateSweptArea = options.CalculateSweptArea }, cancellationToken);
        perimeterTicks=Stopwatch.GetTimestamp()-stamp;
        paths.AddRange(boundary.Toolpaths);
        int boundaryVisits = paths.Count;
        int rows = 0;
        for (int regionIndex = 0; regionIndex < regions.Count; regionIndex++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            activeStage="raster-difference";stamp=Stopwatch.GetTimestamp();
            long beforeWait=rasterLookahead?.WaitTicks??0;
            var preparedInset=rasterLookahead?.Take(regionIndex,cancellationToken);
            rasterLookaheadWaitTicks+=(rasterLookahead?.WaitTicks??0)-beforeWait;
            if(preparedInset is null)
            {
                preparedInset=PrepareRasterInset(regions[regionIndex],-(tool.Radius+options.StockToLeave),
                    options.Tolerances.ArcTolerance,options.Tolerances.SimplifyTolerance,options.ClipBoundary,cancellationToken);
                rasterDifferenceTicks+=preparedInset.DifferenceTicks;rasterOffsetTicks+=preparedInset.OffsetTicks;rasterClipTicks+=preparedInset.ClipTicks;
            }
            else Console.WriteLine($"[Pocket.Scanline.Lookahead] event=hit region={regionIndex} retainedBytes={rasterLookahead!.RetainedBytes} "
                +$"computeMs={1000d*rasterLookahead.ComputeTicks/Stopwatch.Frequency:F3} waitMs={1000d*rasterLookahead.WaitTicks/Stopwatch.Frequency:F3} "
                +"sameOrderedIntegerGeometry=True sourceOrderUnchanged=True proofCallbacksDeferred=True computeTimeOverlapsPerimeter=True");
            var inset=preparedInset.Inset;
            activeStage="raster-preparation";stamp=Stopwatch.GetTimestamp();
            var polygons = inset.Select(p => p.Select(Rotate).ToList()).ToArray();
            if (polygons.Length == 0){rasterPreparationTicks+=Stopwatch.GetTimestamp()-stamp;continue;}
            double minY = polygons.SelectMany(p => p).Min(p => p.y), maxY = polygons.SelectMany(p => p).Max(p => p.y);
            int rowCount = Math.Max(1, (int)Math.Ceiling((maxY - minY) / pitch));
            if (rowCount > 100000) throw new InvalidOperationException("Pocket exceeds 100000 scanlines; increase stepover.");
            var runs = new List<(PointD A, PointD B)>();
            rasterPreparationTicks+=Stopwatch.GetTimestamp()-stamp;
            activeStage="raster-intersections";long rowsStarted=Stopwatch.GetTimestamp();
            for (int row = 0; row <= rowCount; row++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                double y = minY + (maxY - minY) * row / rowCount;
                var intersections = new List<double>();
                foreach (var polygon in polygons)
                for (int i = 0; i < polygon.Count; i++)
                {
                    var a = polygon[i]; var b = polygon[(i + 1) % polygon.Count];
                    if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y))
                        intersections.Add(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
                }
                intersections.Sort();
                for (int i = 0; i + 1 < intersections.Count; i += 2)
                {
                    if (intersections[i + 1] - intersections[i] <= 1e-6) continue;
                    var a = World(new PointD(intersections[i], y)); var b = World(new PointD(intersections[i + 1], y));
                    if(access==null) runs.Add(row % 2 == 0 ? (a, b) : (b, a));
                    else
                    {
                        long legalStarted=Stopwatch.GetTimestamp();
                        foreach(var run in access.LegalRuns(a,b,tool.Radius,options.StockToLeave,cancellationToken))
                        runs.Add(row%2==0?run:(run.B,run.A));
                        legalRunsTicks+=Stopwatch.GetTimestamp()-legalStarted;
                    }
                }
            }
            rasterRowsTicks+=Stopwatch.GetTimestamp()-rowsStarted;
            rows += runs.Count;
            activeStage="raster-ordering";stamp=Stopwatch.GetTimestamp();
            if(options.IndependentRasterEntries && runs.Count>1)
            {
                var carrier=boundary.Toolpaths.LastOrDefault(p=>p.RegionIndex==regionIndex&&p.Points.Count>0);
                var seed=carrier.Points is {Count:>0}?new PointD(carrier.Points[^1].X,carrier.Points[^1].Y):runs[0].A;
                runs=RouteLocalRows(runs,seed,regionIndex,cancellationToken);
            }
            orderingTicks+=Stopwatch.GetTimestamp()-stamp;
            activeStage="entry-layout";stamp=Stopwatch.GetTimestamp();
            if (options.CalculateSweptArea)
                planarRuns.AddRange(runs.Select(run => new List<PointD> { run.A, run.B }));
            int firstRaster = paths.Count;
            var rowBlocks = runs.Select(_ => new List<TaggedToolpath>()).ToArray();
            for (int pass = 0; pass < depth.PassCount; pass++)
            {
            bool reverse = options.IndependentRasterEntries && (pass & 1) != 0;
            IEnumerable<int> passRows = reverse ? Enumerable.Range(0, runs.Count).Reverse()
                : Enumerable.Range(0, runs.Count);
            foreach (int row in passRows)
            {
                var run = runs[row];
                if (reverse) run = (run.B, run.A);
                cancellationToken.ThrowIfCancellationRequested();
                // Local open entries recut their ramped prefix and exit at the far endpoint.
                RampedPath ramp;
                if(!options.IndependentRasterEntries)
                    ramp=RampPlanner.PlanClosedRing(new[] { run.A,run.B },depth.PassTopZ(pass),depth.PassBottomZ(pass),options.Ramp);
                else if(options.Ramp.Strategy==RampStrategy.Plunge || options.Ramp.AngleRadians<=0)
                    ramp=new RampedPath(new List<Point3D>{new(run.A.x,run.A.y,depth.PassTopZ(pass)),
                        new(run.A.x,run.A.y,depth.PassBottomZ(pass)),new(run.B.x,run.B.y,depth.PassBottomZ(pass))},
                        new List<ToolpathSpan>{new(0,1,ToolpathSpanKind.Ramp),new(1,2,ToolpathSpanKind.Cut)},RampStrategy.Plunge,0);
                else
                    ramp=RampPlanner.PlanPathEntry(new Point3D[]{new(run.A.x,run.A.y,depth.PassBottomZ(pass)),
                        new(run.B.x,run.B.y,depth.PassBottomZ(pass))},depth.PassTopZ(pass),options.Ramp,cancellationToken);
                var path = new TaggedToolpath(ramp.Points, ramp.Spans, regionIndex, PocketToolpaths.Category,
                    pass, paths.Count, detectCurves: false);
                paths.Add(path);
                rowBlocks[row].Add(path);
            }
            }
            entryLayoutTicks+=Stopwatch.GetTimestamp()-stamp;
            activeStage="row-block-ordering";stamp=Stopwatch.GetTimestamp();
            if (options.IndependentRasterEntries && depth.PassCount > 1 && runs.Count > 1)
            {
                var carrier = boundary.Toolpaths.LastOrDefault(p => p.RegionIndex == regionIndex && p.Points.Count > 0);
                var seed = carrier.Points is { Count: > 0 } ? carrier.Points[^1] : rowBlocks[0][0].Points[0];
                var ordered = OrderRowBlocks(paths.Skip(firstRaster).ToArray(), rowBlocks, seed, depth.SurfaceZ,
                    regionIndex, cancellationToken);
                paths.RemoveRange(firstRaster, paths.Count - firstRaster);
                paths.AddRange(ordered);
            }
            orderingTicks+=Stopwatch.GetTimestamp()-stamp;
        }
        Console.WriteLine($"[Pocket.Scanline] regions={regions.Count} rows={rows} passes={depth.PassCount} paths={paths.Count} pitch={pitch:R} angle={angleDegrees:R}");
        // Finish one source component before the next. The boundary carrier
        // supplies the same executed retrace network used by offset pocketing.
        activeStage="global-ordering";stamp=Stopwatch.GetTimestamp();
        paths = paths.Select((path,index) => (path,index)).OrderBy(p => p.path.RegionIndex)
            .ThenBy(p => p.index < boundaryVisits ? 0 : 1)
            .ThenBy(p => p.index < boundaryVisits || !options.IndependentRasterEntries ? p.path.DepthPassIndex ?? 0 : 0)
            .Select(p => p.path).ToList();
        orderingTicks+=Stopwatch.GetTimestamp()-stamp;
        activeStage="access-trim";stamp=Stopwatch.GetTimestamp();
        if(access!=null) paths=access.TrimOwnedFlatRadius(paths,tool.Radius,cancellationToken,surface:depth.SurfaceZ,
            diagnostic:message=>Console.WriteLine($"[Pocket.Scanline] {message}"));
        accessTrimTicks=Stopwatch.GetTimestamp()-stamp;
        double verifiedAllowance=options.StockToLeave;
        activeStage="source-simplification";stamp=Stopwatch.GetTimestamp();
        if(options.IndependentRasterEntries&&options.StockToLeave>=.02)
        {
            // The rougher may spend four microns of its positive finishing
            // allowance instead of emitting a million incidental clip vertices.
            // Endpoints/motion kinds stay; the full original physical masks are
            // verified after replacement. The finishing target never changes.
            paths=SurfaceToolpathSimplifier.Nominate(paths,.004,cancellationToken,
                message=>Console.WriteLine($"[Pocket.Scanline] {message}"),"raster-final-cutter-trajectory");
            verifiedAllowance=Math.Max(0,options.StockToLeave-.006);
            Console.WriteLine($"[Pocket.Scanline.Allowance] authored={options.StockToLeave:R} minimumVerified={verifiedAllowance:R} "
                +$"maximumTrajectoryChangeMm=0.004 originalPhysicalWallsAndRampExclusionRequired=True finishingTargetChanged=False");
        }
        simplifyTicks=Stopwatch.GetTimestamp()-stamp;
        {
        activeStage="source-clearance";stamp=Stopwatch.GetTimestamp();
        var sourceClearance = new PocketToolpaths.FillSourceClearance(regions,
            Math.Max(0,tool.Radius+options.StockToLeave), options.LinkClearance, options.ClipBoundary);
        var continuation = new FlatPocketContinuationAdapter(tool,depth,sourceClearance,cancellationToken,
            options.KnownEmptyBoundary,options.KnownEmptyFloor,options.FlatContinuationAdditionalWitnessSites);
        sourceClearanceTicks=Stopwatch.GetTimestamp()-stamp;
        activeStage="reconnect";stamp=Stopwatch.GetTimestamp();
        paths = ToolpathContinuationPlanner.Reconnect(paths,"scanline-pocket",2*tool.Radius,
            cancellationToken,continuation.AllowsNewChord,removeSubsumedEntries:continuation.Supported,
            recordExecuted:continuation.Record,motion:options.Motion,surface:depth.SurfaceZ,
            costBeforeNewChord:true,searchLimits:searchLimits);
        reconnectTicks=Stopwatch.GetTimestamp()-stamp;
        continuation.Report();
        sourceClearance.Report();
        }
        Console.WriteLine($"[Pocket.Scanline.LocalEntries] paths={paths.Count} rows={rows} "
            +$"commonSourceContinuation=True fullCutterProofRequired=True unknownRetainsEntry=True");
        activeStage="final-access-verify";stamp=Stopwatch.GetTimestamp();
        access?.Verify(paths,tool,depth,verifiedAllowance,cancellationToken);
        finalAccessVerifyTicks=Stopwatch.GetTimestamp()-stamp;
        activeStage="protection-verify";stamp=Stopwatch.GetTimestamp();
        PocketProtection.Verify(paths, tool.Radius, depth.SurfaceZ,
            options.ProtectedBoundary, cancellationToken);
        protectionVerifyTicks=Stopwatch.GetTimestamp()-stamp;
        activeStage="swept-area";stamp=Stopwatch.GetTimestamp();
        var swept = options.CalculateSweptArea
            ? PolygonOps.Union(boundary.SweptArea.Concat(PolygonOps.BufferCenterlines(planarRuns, tool.Radius)).ToList())
            : new List<List<PointD>>();
        sweptTicks=Stopwatch.GetTimestamp()-stamp;
        Console.WriteLine($"[Pocket.Scanline.SweptArea] calculateSweptArea={options.CalculateSweptArea} polygons={swept.Count}");
        completed=true;activeStage="complete";outputPaths=paths.Count;
        return new ToolpathPlan(paths, swept, new ToolpathDiagnostics {
            PathCount = paths.Count, DepthPassCount = depth.PassCount, IslandCount = regions.Count });
        }
        finally
        {
            long total=Stopwatch.GetTimestamp()-generateStarted;
            long measuredRowTicks=Math.Max(rasterRowsTicks,legalRunsTicks);
            long accounted=accessSetupTicks+protectionSubtractTicks+perimeterTicks+rasterDifferenceTicks+rasterOffsetTicks
                +rasterClipTicks+rasterPreparationTicks+measuredRowTicks+entryLayoutTicks+orderingTicks+accessTrimTicks+simplifyTicks
                +sourceClearanceTicks+reconnectTicks+finalAccessVerifyTicks+protectionVerifyTicks+sweptTicks+rasterLookaheadWaitTicks;
            static double Ms(long ticks)=>1000d*ticks/Stopwatch.Frequency;
            Console.WriteLine($"[Pocket.Scanline.Perf] completed={completed} activeStage={activeStage} inputRegions={inputRegions} outputPaths={outputPaths} "
                +$"totalMs={Ms(total):F3} accessSetupMs={Ms(accessSetupTicks):F3} protectionSubtractMs={Ms(protectionSubtractTicks):F3} perimeterMs={Ms(perimeterTicks):F3} "
                +$"rasterDifferenceMs={Ms(rasterDifferenceTicks):F3} rasterOffsetMs={Ms(rasterOffsetTicks):F3} rasterClipMs={Ms(rasterClipTicks):F3} rasterPreparationMs={Ms(rasterPreparationTicks):F3} rasterLookaheadWaitMs={Ms(rasterLookaheadWaitTicks):F3} "
                +$"rowIntersectionsMs={Ms(measuredRowTicks-legalRunsTicks):F3} legalRunsMs={Ms(legalRunsTicks):F3} entryLayoutMs={Ms(entryLayoutTicks):F3} orderingMs={Ms(orderingTicks):F3} "
                +$"accessTrimMs={Ms(accessTrimTicks):F3} sourceSimplificationMs={Ms(simplifyTicks):F3} sourceClearanceMs={Ms(sourceClearanceTicks):F3} reconnectMs={Ms(reconnectTicks):F3} "
                +$"finalAccessVerifyMs={Ms(finalAccessVerifyTicks):F3} protectionVerifyMs={Ms(protectionVerifyTicks):F3} sweptAreaMs={Ms(sweptTicks):F3} otherMs={Ms(Math.Max(0,total-accounted)):F3} "
                +"timing=owned-wall-disjoint-stages legalRunsExcludedFromRowIntersections=True incompleteStageTimeInOther=True geometryAndGuardsUnchanged=True");
        }
    }

    private static IReadOnlyList<TaggedToolpath> OrderRowBlocks(IReadOnlyList<TaggedToolpath> original,
        IReadOnlyList<List<TaggedToolpath>> blocks, Point3D seed, double surface, int region, CancellationToken token)
    {
        // Complete a row's existing stepdowns as one block. The next reversed
        // entry starts at the literal preceding exit; no connector or stock
        // credit is invented. Keep the perimeter and every directed source chord.
        var visits = blocks.Select(b => new TaggedToolpath(new[] { b[0].Points[0], b[^1].Points[^1] },
            Array.Empty<ToolpathSpan>(), region, "row-block", null, detectCurves: false)).ToArray();
        var order = ToolpathVisitOrder.Order(visits, Enumerable.Repeat(false, visits.Length).ToArray(), seed, token);
        var candidate = order.SelectMany(i => blocks[i]).ToArray();
        (int Count, double Planar, double Up, double Down) Cost(IReadOnlyList<TaggedToolpath> paths)
        {
            var from = seed; int count = 0; double planar = 0, up = 0, down = 0;
            foreach (var path in paths)
            {
                token.ThrowIfCancellationRequested(); var to = path.Points[0];
                if (from != to)
                {
                    count++; planar += Math.Sqrt(Math.Pow(to.X-from.X,2)+Math.Pow(to.Y-from.Y,2));
                    up += Math.Max(0,surface-from.Z); down += Math.Max(0,surface-to.Z);
                }
                from = path.Points[^1];
            }
            return (count,planar,up,down);
        }
        var before = Cost(original); var after = Cost(candidate);
        // Componentwise dominance holds for any positive approach/retract/feed
        // rates and any nonnegative safe clearance, without guessing machine feeds.
        bool accept = after.Count <= before.Count && after.Planar <= before.Planar + 1e-8
            && after.Up <= before.Up + 1e-8 && after.Down <= before.Down + 1e-8;
        Console.WriteLine($"[Pocket.Scanline.RowBlocks] region={region} blocks={blocks.Count} accepted={accept} "
            + $"beforeTransfers={before.Count} candidateTransfers={after.Count} beforePlanarMm={before.Planar:R} candidatePlanarMm={after.Planar:R} "
            + $"beforeRetractToSurfaceMm={before.Up:R} candidateRetractToSurfaceMm={after.Up:R} "
            + $"beforeApproachFromSurfaceMm={before.Down:R} candidateApproachFromSurfaceMm={after.Down:R} "
            + $"owner=shared-actual-exit-spatial-order completeRowStepdowns=True directedSourcesUnchanged=True newCutLinks=0 componentwiseTransitionCostRequired=True");
        return accept ? candidate : original;
    }

    private static List<(PointD A,PointD B)> RouteLocalRows(List<(PointD A,PointD B)> source,
        PointD seed,int region,CancellationToken token)
    {
        // A few hundred rows need only cheap endpoint arithmetic. Large inputs
        // retain the linear row order instead of admitting quadratic work.
        if(source.Count>2048)return source;
        double Square(PointD a,PointD b)=>(a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
        double Travel(List<(PointD A,PointD B)> paths)
        {
            var at=seed;double sum=0;foreach(var p in paths){sum+=Math.Sqrt(Square(at,p.A));at=p.B;}return sum;
        }
        var used=new bool[source.Count];var ordered=new List<(PointD A,PointD B)>(source.Count);var current=seed;
        for(int count=0;count<source.Count;count++)
        {
            token.ThrowIfCancellationRequested();int next=-1;bool reverse=false;double best=double.PositiveInfinity;
            for(int i=0;i<source.Count;i++)if(!used[i])
            {
                double a=Square(current,source[i].A),b=Square(current,source[i].B);
                if(a<best){best=a;next=i;reverse=false;}
                if(b<best){best=b;next=i;reverse=true;}
            }
            used[next]=true;var row=source[next];if(reverse)row=(row.B,row.A);ordered.Add(row);current=row.B;
        }
        double before=Travel(source),after=Travel(ordered);bool accept=after<=before;
        Console.WriteLine($"[Pocket.Scanline.RowRouting] region={region} rows={source.Count} beforeTransferMm={before:R} "
            +$"afterTransferMm={(accept?after:before):R} accepted={accept} maximumQuadraticRows=2048 "
            +$"actualRowExit=far-end interLayerOrder=reverse priorOrFutureStockCredit=none safeRetracts=True");
        return accept?ordered:source;
    }
}
