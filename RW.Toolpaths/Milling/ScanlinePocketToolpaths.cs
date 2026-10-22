using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Flat-depth raster pockets, bounded by the same physical-cutter inset as offset pockets.</summary>
public static class ScanlinePocketToolpaths
{
    public static ToolpathPlan Generate(IReadOnlyList<MillingRegion> regions, ToolGeometry tool,
        DepthSchedule depth, PocketOptions options, double angleDegrees = 0,
        CancellationToken cancellationToken = default)
    {
        if (!double.IsFinite(options.StockToLeave))
            throw new ArgumentOutOfRangeException(nameof(options), "Pocket edge offset must be finite.");
        if (regions.Count == 0 || tool.Radius <= 0 || depth.Depth <= 0) return ToolpathPlan.Empty;
        var access = options.BoundaryAccess?.Resolve(regions,tool,depth,options.SmoothRoughing,
            options.SmoothRampAngleDegrees,options.StockToLeave,cancellationToken);
        if(access!=null)
        {
            regions=access.PocketRegions(tool.Radius,options.StockToLeave,cancellationToken);
            options=options with {KnownEmptyBoundary=access.CertifiedEmptyBoundary(PocketContinuationRouter.KnownAirReserve,cancellationToken),
                KnownEmptyFloor=depth.SurfaceZ-depth.Depth, ClipBoundary=options.ClipBoundary ?? access.PartBoundary};
        }
        regions = PocketProtection.Subtract(regions, options, cancellationToken);
        if (regions.Count == 0) return ToolpathPlan.Empty;
        double pitch = tool.Radius * 2 * options.StepOver;
        if (!double.IsFinite(pitch) || pitch <= 0) throw new ArgumentException("Scanline stepover must be positive.");
        Console.WriteLine($"[PocketToolpaths.EdgeOffset] mode=scanline toolRadius={tool.Radius:R} "
            + $"edgeOffset={options.StockToLeave:R} firstInset={tool.Radius + options.StockToLeave:R} "
            + $"stepOver={pitch:R} regions={regions.Count}");
        double angle = angleDegrees * Math.PI / 180, cos = Math.Cos(angle), sin = Math.Sin(angle);
        PointD Rotate(PointD p) => new(p.x * cos + p.y * sin, -p.x * sin + p.y * cos);
        PointD World(PointD p) => new(p.x * cos - p.y * sin, p.x * sin + p.y * cos);
        var paths = new List<TaggedToolpath>();
        var planarRuns = new List<List<PointD>>();
        // The compensated perimeter closes the scalloped strip left at raster endpoints.
        Console.WriteLine($"[Pocket.Scanline.Perimeter] independentEntries={options.IndependentRasterEntries} "
            +$"owner={(access!=null&&!options.IndependentRasterEntries?"concentric-first-front":"inside-profile")} actualAccessAndProtectionAuditRequired=True");
        var boundary = access != null && !options.IndependentRasterEntries
            ? PocketToolpaths.Generate(regions,tool,depth,options with { BoundaryAccess=null,MaxRings=1,
                InsideOut=false,SpiralChaining=false },cancellationToken)
            : ProfileToolpaths.Generate(regions, tool, depth, new ProfileOptions {
            Side = ProfileSide.Inside, StockToLeave = options.StockToLeave, Ramp = options.Ramp,
            Tolerances = options.Tolerances, ClipBoundary = options.ClipBoundary,
            CalculateSweptArea = options.CalculateSweptArea }, cancellationToken);
        paths.AddRange(boundary.Toolpaths);
        int boundaryVisits = paths.Count;
        int rows = 0;
        for (int regionIndex = 0; regionIndex < regions.Count; regionIndex++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var material = PolygonOps.Difference(new[] { regions[regionIndex].Outer }, regions[regionIndex].Holes);
            var inset = PolygonOps.Offset(material, -(tool.Radius + options.StockToLeave),
                options.Tolerances.ArcTolerance, options.Tolerances.SimplifyTolerance);
            if (options.ClipBoundary is { Count: >= 3 }) inset = PolygonOps.Intersect(inset, new[] { options.ClipBoundary });
            var polygons = inset.Select(p => p.Select(Rotate).ToList()).ToArray();
            if (polygons.Length == 0) continue;
            double minY = polygons.SelectMany(p => p).Min(p => p.y), maxY = polygons.SelectMany(p => p).Max(p => p.y);
            int rowCount = Math.Max(1, (int)Math.Ceiling((maxY - minY) / pitch));
            if (rowCount > 100000) throw new InvalidOperationException("Pocket exceeds 100000 scanlines; increase stepover.");
            var runs = new List<(PointD A, PointD B)>();
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
                    else foreach(var run in access.LegalRuns(a,b,tool.Radius,options.StockToLeave,cancellationToken))
                        runs.Add(row%2==0?run:(run.B,run.A));
                }
            }
            rows += runs.Count;
            if(options.IndependentRasterEntries && runs.Count>1)
            {
                var carrier=boundary.Toolpaths.LastOrDefault(p=>p.RegionIndex==regionIndex&&p.Points.Count>0);
                var seed=carrier.Points is {Count:>0}?new PointD(carrier.Points[^1].X,carrier.Points[^1].Y):runs[0].A;
                runs=RouteLocalRows(runs,seed,regionIndex,cancellationToken);
            }
            if (options.CalculateSweptArea)
                planarRuns.AddRange(runs.Select(run => new List<PointD> { run.A, run.B }));
            for (int pass = 0; pass < depth.PassCount; pass++)
            {
            IEnumerable<(PointD A,PointD B)> passRuns=options.IndependentRasterEntries&&(pass&1)!=0
                ? runs.AsEnumerable().Reverse().Select(r=>(A:r.B,B:r.A)):runs;
            foreach (var run in passRuns)
            {
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
                paths.Add(new TaggedToolpath(ramp.Points, ramp.Spans, regionIndex, PocketToolpaths.Category,
                    pass, paths.Count, detectCurves: false));
            }
            }
        }
        Console.WriteLine($"[Pocket.Scanline] regions={regions.Count} rows={rows} passes={depth.PassCount} paths={paths.Count} pitch={pitch:R} angle={angleDegrees:R}");
        // Finish one source component before the next. The boundary carrier
        // supplies the same executed retrace network used by offset pocketing.
        paths = paths.Select((path,index) => (path,index)).OrderBy(p => p.path.RegionIndex)
            .ThenBy(p => p.index < boundaryVisits ? 0 : 1).ThenBy(p => p.path.DepthPassIndex ?? 0)
            .Select(p => p.path).ToList();
        if(access!=null) paths=access.Trim(paths,_=>tool.Radius,cancellationToken,surface:depth.SurfaceZ);
        double verifiedAllowance=options.StockToLeave;
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
                +"maximumTrajectoryChangeMm=0.004 originalPhysicalWallsAndRampExclusionRequired=True finishingTargetChanged=False");
        }
        if(!options.IndependentRasterEntries)
        {
        var sourceClearance = new PocketToolpaths.FillSourceClearance(regions,
            Math.Max(0,tool.Radius+options.StockToLeave), options.LinkClearance, options.ClipBoundary);
        var continuation = new FlatPocketContinuationAdapter(tool,depth,sourceClearance,cancellationToken,
            options.KnownEmptyBoundary,options.KnownEmptyFloor);
        paths = ToolpathContinuationPlanner.Reconnect(paths,"scanline-pocket",2*tool.Radius,
            cancellationToken,continuation.AllowsNewChord,removeSubsumedEntries:continuation.Supported,
            recordExecuted:continuation.Record);
        sourceClearance.Report();
        }
        else Console.WriteLine($"[Pocket.Scanline.LocalEntries] paths={paths.Count} rows={rows} "
            +"wholeCutterAndAccessVerificationRequired=True safeRetractsBetweenVisits=True rowExit=far-end "
            +"executedStockNetworkReconstruction=False previousLayerCredit=none newInterVisitCutLinks=0");
        access?.Verify(paths,tool,depth,verifiedAllowance,cancellationToken);
        PocketProtection.Verify(paths, tool.Radius, depth.SurfaceZ,
            options.ProtectedBoundary, cancellationToken);
        var swept = options.CalculateSweptArea
            ? PolygonOps.Union(boundary.SweptArea.Concat(PolygonOps.BufferCenterlines(planarRuns, tool.Radius)).ToList())
            : new List<List<PointD>>();
        Console.WriteLine($"[Pocket.Scanline.SweptArea] calculateSweptArea={options.CalculateSweptArea} polygons={swept.Count}");
        return new ToolpathPlan(paths, swept, new ToolpathDiagnostics {
            PathCount = paths.Count, DepthPassCount = depth.PassCount, IslandCount = regions.Count });
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
            +"actualRowExit=far-end interLayerOrder=reverse priorOrFutureStockCredit=none safeRetracts=True");
        return accept?ordered:source;
    }
}
