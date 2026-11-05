using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Actual preceding posted cutter columns, in stock-relative coordinates.
/// Floor depths nominate slices only; a slice earns clearance exclusively from
/// chords whose two ends reached it. PartBoundary bounds the physical platform.</summary>
public sealed record VCarveClearedStock(
    IReadOnlyList<VCarveRoughingChord> Chords,
    IReadOnlyList<double> FloorDepths,
    IReadOnlyList<PointD> PartBoundary,
    IReadOnlyList<VCarveClearedSweep>? ProfileSweeps = null);

/// <summary>Actual earlier posted motion of a complete monotone physical cutter,
/// including rounded V-carvers that cannot supply a full tip-radius column.</summary>
public sealed record VCarveClearedSweep(VCarveCutterProfile Profile, IReadOnlyList<VCarveRoughingChord> Chords);

/// <summary>Height-dependent cutter fit against remaining stock. The original
/// target still owns every medial XY branch. Unions here describe clearance at
/// particular heights, never geometry from which to generate a new medial axis.</summary>
internal sealed partial class VCarveMaterialSurface
{
    private sealed record Layer(double TopDepth, PreparedRegionDistance Distance, PreparedRegionDistance ClearedDistance,
        MedialAxisToolpaths.SpatialSegmentIndex Boundary, IReadOnlyList<IReadOnlyList<PointD>> FreeRegion);
    private readonly List<Layer> layers = new();
    private readonly VCarveCutterProfile profile;
    private readonly VCarveProjectedStock projectedStock;
    private readonly PreparedRegionDistance originalTarget;
    private readonly double maximumDepth, surface;
    private readonly CancellationToken token;
    private readonly Action<string>? diagnostic;
    private readonly bool parallelMedialFits;
    private long sourcePoints, outputPoints, loweredPoints, chordChecks, subdivisions, retreats, refusedSources, shortRefusals, fittedRetreats;
    private double largestDescent;
    private long verticalSweepsReduced;
    private long raisedSourceProofs, qualifiedSourceChords;
    internal long RaisedSourceProofs => raisedSourceProofs;
    private readonly Dictionary<(double, double, double, double), List<(double A, double B)>> qualifiedSources = new();
    // Layer geometry and cutter profile never change in this owner. Repeated
    // native and posted poses retain the literal layer answer,
    // including refusals, without transferring any executed-stock certificate.
    private const int LayerPoseCacheLimit = 65536;
    private readonly Dictionary<Point3D, bool> layerPoseProofs = new();
    private readonly Queue<Point3D> layerPoseOrder = new();
    private long layerPoseCacheHits, layerPoseCacheMisses, layerPoseCacheEvictions;
    private const int LegalPoseCacheLimit=131072;
    private readonly Dictionary<(double X,double Y),double> deepestLegalPoses=new();
    private readonly Queue<(double X,double Y)> legalPoseOrder=new();
    private long raisedPoseProofHits,raisedPoseProofPublications;
    private long postedChordRefusals,nativeChordRefusals;
    private long pointDiscLayerProofs;
    // Accounts for F3 planar posting and integer boundary construction. Every
    // resulting chord is additionally proved with the complete physical profile.
    internal const double PlanarReserve = .002;

    internal VCarveMaterialSurface(IEnumerable<IReadOnlyList<PointD>> target,
        VCarveClearedStock stock, VCarveCutterProfile profile, double maximumDepth,
        double surface, CancellationToken token, Action<string>? diagnostic,
        bool parallelMedialFits = true)
    {
        this.parallelMedialFits = parallelMedialFits;
        this.profile = profile; this.maximumDepth = maximumDepth; this.surface = surface;
        this.token = token; this.diagnostic = diagnostic;
        long started = Stopwatch.GetTimestamp();
        var original = Clipper.Union(PathUtils.ToClipper(target), FillRule.NonZero);
        originalTarget = new PreparedRegionDistance(PathUtils.FromClipper(original));
        VCarveNativeStockTrace.Observer.Value?.Invoke(new(PathUtils.FromClipper(original),stock,profile,maximumDepth,surface));
        if (stock.PartBoundary.Count < 3 || stock.PartBoundary.Any(p => !double.IsFinite(p.x + p.y)))
            throw new ArgumentException("Cleared V-carve stock requires a finite physical Part boundary.", nameof(stock));
        var part = PathUtils.ToClipper(new[] { stock.PartBoundary });
        bool Posted(Point3D p) => double.IsFinite(p.X + p.Y + p.Z)
            && Math.Abs(p.X) <= 10000 && Math.Abs(p.Y) <= 10000 && Math.Abs(p.Z) <= 100
            && p.X == Math.Round(p.X * 1000) / 1000d && p.Y == Math.Round(p.Y * 1000) / 1000d
            && p.Z == Math.Round(p.Z * 1000) / 1000d;
        var executed = stock.Chords.Where(c => Posted(c.A) && Posted(c.B)
            && c.ClearedColumnRadius > 0 && double.IsFinite(c.ClearedColumnRadius)).ToArray();
        projectedStock = new(executed, PathUtils.FromClipper(original), stock.PartBoundary,
            profile, maximumDepth, token, stock.ProfileSweeps,diagnostic);
        diagnostic?.Invoke($"stage=material-surface event=continuous-index flatChords={executed.Length} "
            +$"profileChords={projectedStock.ProfileChords} indexedChords={projectedStock.IndexedChords} "
            +"compaction=literal-F3-directed-collinearity-and-same-trace-depth-dominance unionGeometryPreserved=True");
        // Coalescing shallow slices into a deeper one is conservative: fewer
        // executed columns qualify. Keep work bounded for long operation lists.
        double[] floors = stock.FloorDepths.Where(double.IsFinite)
            .Select(d => Math.Min(maximumDepth, Math.Floor(d * 1000 + 1e-7) / 1000))
            .Where(d => d > 0).Append(maximumDepth).Distinct().Order().TakeLast(8).ToArray();
        double top = 0;
        foreach (double floor in floors)
        {
            token.ThrowIfCancellationRequested();
            var swept = new Paths64();
            int credited = 0;
            foreach (var group in executed.Where(c => Math.Max(c.A.Z, c.B.Z) <= -floor + 1e-9)
                .GroupBy(c => c.ClearedColumnRadius))
            {
                var columns = group.ToArray();
                credited += columns.Length;
                var segments = columns.Select(c => new PocketPostedChordChains.Segment(
                    new((long)Math.Round(c.A.X * 1000), (long)Math.Round(c.A.Y * 1000)),
                    new((long)Math.Round(c.B.X * 1000), (long)Math.Round(c.B.Y * 1000)))).ToArray();
                var chains = PocketPostedChordChains.Build(segments, token);
                var paths = new Paths64(chains.Paths.Select(path => new Path64(path.Select(p =>
                    new Point64((long)Math.Round(p.X / 1000d * PathUtils.Scale),
                        (long)Math.Round(p.Y / 1000d * PathUtils.Scale))))));
                paths.AddRange(columns.Where(c => c.A.X == c.B.X && c.A.Y == c.B.Y)
                    .Select(c => new Path64 { new((long)Math.Round(c.A.X * PathUtils.Scale),
                        (long)Math.Round(c.A.Y * PathUtils.Scale)) }));
                if (paths.Count > 0)
                    swept.AddRange(PostedSweepInnerRegion.BuildPaths(paths, group.Key, token));
            }
            swept = Clipper.Intersect(Clipper.Union(swept, FillRule.NonZero), part, FillRule.NonZero);
            var free = Clipper.Intersect(Clipper.Union(new Paths64(original.Concat(swept)), FillRule.NonZero), part, FillRule.NonZero);
            var rings = PathUtils.FromClipper(free).Select(r => (IReadOnlyList<PointD>)r).ToList();
            var edges = rings.SelectMany(ring => Enumerable.Range(0, ring.Count)
                .Select(i => (ring[i], ring[(i + 1) % ring.Count]))).ToList();
            layers.Add(new(top, new PreparedRegionDistance(rings), new PreparedRegionDistance(PathUtils.FromClipper(swept)),
                new MedialAxisToolpaths.SpatialSegmentIndex(edges), rings));
            diagnostic?.Invoke($"stage=material-surface layerTopDepth={top:R} layerBottomDepth={floor:R} "
                + $"postedColumnChords={credited} boundaryEdges={edges.Count} "
                + "targetBranchesUnchanged=True credit=entire-executed-column platformClipped=True");
            top = floor;
        }
        diagnostic?.Invoke($"stage=material-surface event=prepared layers={layers.Count} "
            + $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
            + $"receivedChords={stock.Chords.Count} admittedPostedColumns={executed.Length} "
            + $"finalTipDepth={maximumDepth:R} startingDepthIsNotGlobalCutterDatum=True");
    }

    private VCarveMaterialSurface(VCarveMaterialSurface source)
    {
        layers = source.layers.Select(layer => layer with {
            Distance = layer.Distance.CreateQueryWorker(),
            ClearedDistance = layer.ClearedDistance.CreateQueryWorker() }).ToList();
        profile = source.profile;
        projectedStock = source.projectedStock.CreateQueryWorker();
        originalTarget = source.originalTarget.CreateQueryWorker();
        maximumDepth = source.maximumDepth;
        surface = source.surface; token = source.token;
        // Apply publishes/qualifies sources after every fitting worker has joined.
        // This dictionary is read-only for the complete parallel query interval.
        qualifiedSources = source.qualifiedSources;
        diagnostic = null; parallelMedialFits=false;
    }

    internal double AllowedDepth(double x, double y)
        => AllowedDepth(x, y, clearedOnly: false);

    private double AllowedDepth(double x, double y, bool clearedOnly)
    {
        double result = maximumDepth, unreserved = maximumDepth;
        PreparedRegionDistance? limiting = null;
        double limitingClearance = 0;
        foreach (var layer in layers)
        {
            double clearance = (clearedOnly ? layer.ClearedDistance : layer.Distance).SignedDistance(new(x, y));
            // Outside a layer, the tip may reach its upper plane but cannot
            // enter the protected material below that plane.
            double penetration = clearance <= PlanarReserve ? 0
                : profile.DepthAtRadius(clearance - PlanarReserve);
            result = Math.Min(result, layer.TopDepth + penetration);
            double upper = layer.TopDepth + (clearance <= 0 ? 0
                : profile.DepthAtRadius(clearance + PreparedRegionDistance.Uncertainty));
            if (upper < unreserved)
            {
                unreserved = upper; limiting = clearedOnly ? layer.ClearedDistance : layer.Distance;
                limitingClearance = clearance;
            }
        }
        PointD? witness = result < maximumDepth && limiting is not null
            ? limitingClearance <= 0 ? new PointD(x,y) : limiting.NearestBoundaryPoint(new(x,y)) : null;
        return projectedStock.RefineDepth(x, y, result, unreserved, clearedOnly, witness);
    }

    internal double EntryHeight(Point3D absolutePoint)
    {
        var command = Command(new(absolutePoint.X, absolutePoint.Y, absolutePoint.Z - surface));
        double clearedDepth = Math.Min(AllowedDepth(absolutePoint.X, absolutePoint.Y, true),
            AllowedDepth(command.X, command.Y, true));
        return surface - Math.Max(0, clearedDepth - .001);
    }

    private Point3D Command(Point3D p) => new(VCarveCommandRounding.Round((float)p.X),
        VCarveCommandRounding.Round((float)p.Y),
        VCarveCommandRounding.Round((float)(surface + p.Z) - (float)surface));

    private Point3D Fit(Point3D source, bool preserveDepthCap = false)
    {
        var command = Command(source);
        double cap = preserveDepthCap ? Math.Min(maximumDepth, Math.Max(0, -source.Z)) : maximumDepth;
        double depth = Math.Min(cap, Math.Min(AllowedDepth(source.X, source.Y), AllowedDepth(command.X, command.Y)));
        // Reserve axial posting error; never exceed the authored absolute cap.
        depth = depth >= cap ? cap : Math.Max(0, depth - .001);
        var fitted = new Point3D(source.X, source.Y, -depth);
        if (!IsLegalChord(fitted, fitted))
        {
            if (++fittedRetreats <= 8)
                diagnostic?.Invoke($"stage=material-surface event=fit-retreat point={fitted} commanded={Command(fitted)} "
                    + "layers=" + string.Join(";", layers.Select(l => $"{l.TopDepth:R}:{l.Distance.SignedDistance(new(fitted.X,fitted.Y)):R}/{l.Boundary.QueryDistance(fitted.X,fitted.Y):R}")));
            double low=0, high=depth;
            for(int i=0;i<20;i++)
            {
                double candidate=(low+high)/2;
                var point=new Point3D(source.X,source.Y,-candidate);
                if(IsLegalChord(point,point))low=candidate;else high=candidate;
            }
            fitted=new(source.X,source.Y,-low);
        }
        if (-fitted.Z > -source.Z + .002) { loweredPoints++; largestDescent = Math.Max(largestDescent, source.Z - fitted.Z); }
        return fitted;
    }

    internal bool IsLegalChord(Point3D a, Point3D b, bool postingReserve = true)
    {
        chordChecks++;
        // Refuse an unpostable chord before constructing its otherwise
        // unused native proof.
        // Every positive still requires BOTH unchanged complete authorities.
        var ca=Command(a);var cb=Command(b);
        if(!Legal(ca,cb,postingReserve)){postedChordRefusals++;return false;}
        if(ca==a && cb==b)return true;
        if(!Legal(a,b,postingReserve)){nativeChordRefusals++;return false;}
        return true;
    }

    private bool Legal(Point3D a, Point3D b, bool postingReserve = true)
    {
        // For the supported nondecreasing radial profile, withdrawing at the
        // same XY sweeps no additional stock below the original top. The whole
        // deepest cutter pose is therefore the exact vertical-sweep authority.
        if (a.X == b.X && a.Y == b.Y && Math.Max(-a.Z,-b.Z) <= profile.MaximumDepth)
        {
            if (a.Z != b.Z) verticalSweepsReduced++;
            a = b = a.Z < b.Z ? a : b;
        }
        bool pose=a==b && -a.Z<=profile.MaximumDepth;
        var key=(a.X,a.Y);
        if(pose && deepestLegalPoses.TryGetValue(key,out double provedZ) && a.Z>=provedZ)
        {raisedPoseProofHits++;return true;}
        bool legal=ContainsQualifiedRaisedChord(a,b) || projectedStock.TryPositiveProof(a,b,postingReserve)
            || LayerLegal(a, b) || projectedStock.Prove(a, b, postingReserve: postingReserve);
        // With this immutable stock/target and nondecreasing physical profile,
        // every shallower pose at IDENTICAL XY is a subset of a proved deeper
        // pose. Retain only positive complete proofs, including the posting
        // shell; a proof without that shell never enters this stronger memo.
        if(pose && legal && postingReserve)
        {
            if(deepestLegalPoses.TryGetValue(key,out double priorZ))
                deepestLegalPoses[key]=Math.Min(priorZ,a.Z);
            else
            {
                if(deepestLegalPoses.Count==LegalPoseCacheLimit)
                    deepestLegalPoses.Remove(legalPoseOrder.Dequeue());
                deepestLegalPoses.Add(key,a.Z);legalPoseOrder.Enqueue(key);
            }
            raisedPoseProofPublications++;
        }
        return legal;
    }

    private static (double, double, double, double) SourceKey(ref Point3D a, ref Point3D b)
    {
        if (a.X > b.X || a.X == b.X && a.Y > b.Y) (a,b)=(b,a);
        return (a.X,a.Y,b.X,b.Y);
    }

    private bool ContainsQualifiedRaisedChord(Point3D a, Point3D b)
    {
        var key=SourceKey(ref a,ref b);
        if (!qualifiedSources.TryGetValue(key,out var depths)) return false;
        foreach (var source in depths)
        {
            // Exact planar endpoints in this coordinate domain, with both new
            // endpoints at or above an independently proved whole source chord.
            // At every stock height the monotone cutter cross-section is then
            // a subset of the source sweep. This credits clearance, not future
            // removal. No interpolation, endpoint snapping or tolerance is used.
            if (a.Z < source.A || b.Z < source.B) continue;
            raisedSourceProofs++;
            return true;
        }
        return false;
    }

    private void QualifySourceChords(IReadOnlyList<TaggedToolpath> paths)
    {
        void Add(Point3D a,Point3D b)
        {
            if (Math.Max(-a.Z,-b.Z)>profile.MaximumDepth) return;
            // Establish authority directly from original stock, without using
            // any previously registered source certificate.
            if (!projectedStock.TryPositiveProof(a,b) && !LayerLegal(a,b) && !projectedStock.Prove(a,b)) return;
            var key=SourceKey(ref a,ref b);
            if (!qualifiedSources.TryGetValue(key,out var depths))
                qualifiedSources.Add(key,depths=new());
            if (depths.Any(d=>d.A<=a.Z && d.B<=b.Z)) return;
            depths.RemoveAll(d=>d.A>=a.Z && d.B>=b.Z);
            depths.Add((a.Z,b.Z));
            qualifiedSourceChords++;
        }
        foreach (var path in paths)
        for (int i=1;i<path.Points.Count;i++)
        {
            token.ThrowIfCancellationRequested();
            var a=path.Points[i-1];var b=path.Points[i];
            Add(a,b);Add(Command(a),Command(b));
        }
    }

    private bool LayerLegal(Point3D a, Point3D b)
    {
        if (a != b) return LayerLegalCore(a, b);
        if (layerPoseProofs.TryGetValue(a, out bool cached))
        {
            layerPoseCacheHits++;
            return cached;
        }
        layerPoseCacheMisses++;
        bool result = LayerLegalCore(a, b);
        if (layerPoseProofs.Count == LayerPoseCacheLimit)
        {
            layerPoseProofs.Remove(layerPoseOrder.Dequeue());
            layerPoseCacheEvictions++;
        }
        layerPoseProofs.Add(a, result);
        layerPoseOrder.Enqueue(a);
        return result;
    }

    private bool LayerLegalCore(Point3D a, Point3D b)
    {
        foreach (var layer in layers)
        {
            if (Math.Max(-a.Z, -b.Z) <= layer.TopDepth + 1e-10) continue;
            if(a==b)
            {
                double clearance=layer.Distance.SignedDistance(new(a.X,a.Y));
                double radius=profile.PhysicalRadiusAtDepth(-a.Z-layer.TopDepth);
                // A signed-distance lower bound contains the ENTIRE physical
                // cross-section disc at this layer's top. All deeper sections
                // of the monotone cutter are smaller. Uncertain/tangent poses
                // retain the original exact finite-boundary test below.
                if(radius<=clearance-PreparedRegionDistance.Uncertainty)
                {pointDiscLayerProofs++;continue;}
            }
            foreach (var point in new[] { a, b })
            {
                double penetration = -point.Z - layer.TopDepth;
                if (penetration <= 1e-10) continue;
                double clearance = layer.Distance.SignedDistance(new(point.X, point.Y));
                if (clearance < 0 || profile.PhysicalRadiusAtDepth(penetration) > clearance + 1e-9) return false;
            }
            if (layer.Boundary.TryFindDepthViolation(a, b, -layer.TopDepth,
                0, 1, 0, out double witness, profile))
            {
                if (Math.Pow(a.X-b.X,2)+Math.Pow(a.Y-b.Y,2)<1e-8 && ++shortRefusals <= 8)
                {
                    double x=a.X+(b.X-a.X)*witness, y=a.Y+(b.Y-a.Y)*witness;
                    double d=-(a.Z+(b.Z-a.Z)*witness)-layer.TopDepth;
                    diagnostic?.Invoke($"stage=material-surface event=short-chord-witness a={a} b={b} top={layer.TopDepth:R} t={witness:R} "
                        + $"penetration={d:R} radius={profile.PhysicalRadiusAtDepth(d):R} nearest={layer.Boundary.QueryDistance(x,y):R} signed={layer.Distance.SignedDistance(new(x,y)):R}");
                }
                return false;
            }
        }
        return true;
    }

    internal List<TaggedToolpath> Apply(List<TaggedToolpath> paths)
    {
        long started=System.Diagnostics.Stopwatch.GetTimestamp();
        var result = new List<TaggedToolpath>(paths.Count);
        var medial=paths.Select((path,index)=>(Path:path,Index:index)).Where(p=>p.Path.Category=="final-carve"
            &&p.Path.Points.Count>=2).ToArray();
        const int medialBlockEdges=16;
        var blocks=new List<(int Path,int First,int Last)>();
        foreach(var item in medial)
            for(int at=0;at<item.Path.Points.Count-1;at+=medialBlockEdges)
                blocks.Add((item.Index,at,Math.Min(item.Path.Points.Count-1,at+medialBlockEdges)));
        var fitted=new List<Point3D>?[blocks.Count];
        int medialWorkers=parallelMedialFits && medial.Sum(p=>(long)p.Path.Points.Count)>=512
            ? Math.Min(GeometryParallelism.StockFitWorkerLimit,blocks.Count) : 1;
        if(medialWorkers>1)
        {
            long medialStarted=Stopwatch.GetTimestamp();
            var workers=new VCarveMaterialSurface[medialWorkers];
            Parallel.For(0,medialWorkers,new ParallelOptions { CancellationToken=token,
                MaxDegreeOfParallelism=medialWorkers }, index=>{
                using var scope=GeometryParallelism.EnterWorker();
                var worker=workers[index]=new VCarveMaterialSurface(this);
                for(int block=index;block<blocks.Count;block+=medialWorkers)
                {
                    var item=blocks[block];
                    fitted[block]=worker.FitMedialRange(paths[item.Path],item.First,item.Last);
                }
            });
            token.ThrowIfCancellationRequested();
            long imported=0;
            foreach(var worker in workers)
            {
                sourcePoints+=worker.sourcePoints;outputPoints+=worker.outputPoints;loweredPoints+=worker.loweredPoints;
                largestDescent=Math.Max(largestDescent,worker.largestDescent);
                chordChecks+=worker.chordChecks;subdivisions+=worker.subdivisions;retreats+=worker.retreats;
                refusedSources+=worker.refusedSources;shortRefusals+=worker.shortRefusals;fittedRetreats+=worker.fittedRetreats;
                verticalSweepsReduced+=worker.verticalSweepsReduced;raisedSourceProofs+=worker.raisedSourceProofs;
                raisedPoseProofHits+=worker.raisedPoseProofHits;raisedPoseProofPublications+=worker.raisedPoseProofPublications;
                postedChordRefusals+=worker.postedChordRefusals;nativeChordRefusals+=worker.nativeChordRefusals;
                pointDiscLayerProofs+=worker.pointDiscLayerProofs;
                layerPoseCacheHits+=worker.layerPoseCacheHits;layerPoseCacheMisses+=worker.layerPoseCacheMisses;
                layerPoseCacheEvictions+=worker.layerPoseCacheEvictions;
                imported+=projectedStock.ImportPositiveProofs(worker.projectedStock);
            }
            diagnostic?.Invoke($"stage=material-surface event=medial-fits paths={medial.Length} sourcePoints={medial.Sum(p=>p.Path.Points.Count)} "
                +$"workers={medialWorkers} blockEdges={medialBlockEdges} elapsedMs={Stopwatch.GetElapsedTime(medialStarted).TotalMilliseconds:F1} "
                +$"workerProofQueries={workers.Sum(w=>w.projectedStock.Queries)} workerProofCells={workers.Sum(w=>w.projectedStock.Cells)} workerInheritedCellProofs={workers.Sum(w=>w.projectedStock.InheritedStockCellProofs)} workerReusedCellIntersections={workers.Sum(w=>w.projectedStock.ReusedCellIntersectionPredicates)} workerInheritedRegionQueriesAvoided={workers.Sum(w=>w.projectedStock.InheritedRegionDistanceQueriesAvoided)} "
                +$"importedPositiveProofs={imported} workerRefusalWitnessHits={workers.Sum(w=>w.projectedStock.RefusalWitnessHits)} "
                +"immutableStockShared=True privateFitHints=True publication=original-source-order fullNativeAndF3Guards=True");
        }
        for(int pathIndex=0;pathIndex<paths.Count;pathIndex++)
        {
            var path=paths[pathIndex];
            // Preserve native clearing moves literally; fit only medial Z.
            if (path.Category != "final-carve" || path.Points.Count < 2) { result.Add(path); continue; }
            result.Add(medialWorkers>1 ? AssembleMedialPath(path,pathIndex,blocks,fitted) : FitMedialPath(path));
        }
        long qualificationStarted=Stopwatch.GetTimestamp();
        long beforeQualificationCells=projectedStock.Cells;
        QualifySourceChords(result);
        diagnostic?.Invoke($"stage=material-surface event=source-qualification paths={result.Count} "
            +$"elapsedMs={Stopwatch.GetElapsedTime(qualificationStarted).TotalMilliseconds:F1} "
            +$"proofCells={projectedStock.Cells-beforeQualificationCells} authority=original-complete-stock-and-layer-proofs");
        diagnostic?.Invoke($"stage=material-surface event=fitted-region paths={paths.Count} originalMedialPoints={sourcePoints} retainedMedialPoints={outputPoints} "
            +$"qualifiedSourceChords={qualifiedSourceChords} sourceCertificatePolicy=exact-XY-axially-raised-native-and-F3 "
            +"supplementalShoulder=False originalClearingPreserved=True physicalProofToleranceUnchanged=True "
            +$"inheritedStockCellProofs={projectedStock.InheritedStockCellProofs} seededStockCellProofs={projectedStock.SeededCellProofs} seededStockCellTrials={projectedStock.SeededCellTrials} seedCellAuthority=whole-prior-pose-disk-and-current-full-cutter "
            +$"nearbyDepthTrials={projectedStock.NearbyDepthTrials} nearbyDepthFits={projectedStock.NearbyDepthFits} nearbyDepthRefusals={projectedStock.NearbyDepthRefusals} rejectedHighDepthTrials={projectedStock.RejectedHighDepthTrials} intermediateDepthFits={projectedStock.IntermediateDepthFits} "
            +$"stockCellCacheHits={projectedStock.StockCellCacheHits} stockCellCacheMisses={projectedStock.StockCellCacheMisses} stockCellCacheEvictions={projectedStock.StockCellCacheEvictions} cellGeometryCacheHits={projectedStock.CellGeometryCacheHits} cellGeometryCacheMisses={projectedStock.CellGeometryCacheMisses} cachedSweepProofs={projectedStock.CachedSweepProofs} sweepProofCacheLimit={VCarveProjectedStock.SweepProofCacheLimit} sweepProofCacheEvictions={projectedStock.SweepProofCacheEvictions} reversedSweepProofHits={projectedStock.ReversedSweepProofHits} strongerSweepProofHits={projectedStock.StrongerSweepProofHits} depthWitnessQueries={projectedStock.DepthWitnessQueries} depthWitnessBounds={projectedStock.DepthWitnessBounds} depthWitnessSeedFits={projectedStock.DepthWitnessSeedFits} "
            +$"profileCandidatesPruned={projectedStock.ProfileCandidatesPruned} profileNodesPruned={projectedStock.ProfileNodesPruned} priorPoseSeedTrials={projectedStock.PriorPoseSeedTrials} priorPoseSeedPositive={projectedStock.PriorPoseSeedPositive} elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} deletedBranches=0");
        return result;
    }

    private List<Point3D> FitMedialRange(TaggedToolpath path,int first,int last)
    {
        token.ThrowIfCancellationRequested();
        var points=new List<Point3D>{Fit(path.Points[first])};
        // Close against the actual first output pose after all blocks join.
        if(last==path.Points.Count-1 && path.Points[0]==path.Points[^1])last--;
        for(int i=first+1;i<=last;i++)
            Append(path.Points[i-1],path.Points[i],points[^1],Fit(path.Points[i]),points,0);
        return points;
    }

    private TaggedToolpath AssembleMedialPath(TaggedToolpath path,int pathIndex,
        List<(int Path,int First,int Last)> blocks,List<Point3D>?[] fitted)
    {
        var points=new List<Point3D>();int raisedSeams=0;double maximumLift=0;
        for(int block=0;block<blocks.Count;block++)
        {
            if(blocks[block].Path!=pathIndex)continue;
            token.ThrowIfCancellationRequested();
            var next=fitted[block]!;
            if(points.Count==0){points.AddRange(next);continue;}
            var a=points[^1];var b=next[0];
            if(a.X!=b.X||a.Y!=b.Y)
                throw new InvalidOperationException($"Medial block lost its original XY interface: path={pathIndex} block={block} a={a} b={b}");
            // Raising either endpoint of a proved chord keeps its complete
            // monotone-cutter sweep inside that original proof. Use the same
            // shallower interface pose on both sides; add no connecting motion.
            double z=Math.Max(a.Z,b.Z);
            if(a.Z!=b.Z){raisedSeams++;maximumLift=Math.Max(maximumLift,Math.Abs(a.Z-b.Z));}
            points[^1]=a with{Z=z};next[0]=b with{Z=z};points.AddRange(next.Skip(1));
        }
        if(path.Points[0]==path.Points[^1])
            Append(path.Points[^2],path.Points[^1],points[^1],points[0],points,0);
        sourcePoints+=path.Points.Count;outputPoints+=points.Count;
        if(raisedSeams>0)diagnostic?.Invoke($"stage=material-surface event=medial-block-interfaces region={path.RegionIndex} "
            +$"raisedSeams={raisedSeams} maximumLift={maximumLift:R} originalXY=True newConnectingMoves=0 "
            +"authority=pointwise-axial-subset-of-complete-proved-monotone-cutter-chords finalQualificationRequired=True");
        return new TaggedToolpath(points,Array.Empty<ToolpathSpan>(),path.RegionIndex,path.Category,
            path.DepthPassIndex,path.VisitIdentity,detectCurves:false);
    }

    private TaggedToolpath FitMedialPath(TaggedToolpath path)
    {
        token.ThrowIfCancellationRequested();
        sourcePoints+=path.Points.Count;
        var points=new List<Point3D>(path.Points.Count){Fit(path.Points[0])};
        for(int i=1;i<path.Points.Count;i++)
            Append(path.Points[i-1],path.Points[i],points[^1],
                i==path.Points.Count-1 && path.Points[^1]==path.Points[0] ? points[0] : Fit(path.Points[i]),points,0);
        outputPoints+=points.Count;
        return new TaggedToolpath(points,Array.Empty<ToolpathSpan>(),path.RegionIndex,path.Category,
            path.DepthPassIndex,path.VisitIdentity,detectCurves:false);
    }

    private void Append(Point3D sourceA, Point3D sourceB, Point3D a, Point3D b,
        List<Point3D> output, int level)
    {
        token.ThrowIfCancellationRequested();
        var midSource = new Point3D((sourceA.X + sourceB.X) / 2,
            (sourceA.Y + sourceB.Y) / 2, (sourceA.Z + sourceB.Z) / 2);
        var mid = Fit(midSource);
        double lengthSquared = Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2);
        double sourceLengthSquared = Math.Pow(sourceA.X - sourceB.X, 2) + Math.Pow(sourceA.Y - sourceB.Y, 2);
        bool legal = IsLegalChord(a, b);
        if (!legal && level == 0 && ++refusedSources <= 8)
            diagnostic?.Invoke($"stage=material-surface event=refine-refused-chord a={a} b={b} "
                + $"nativeA={Legal(a,a)} nativeB={Legal(b,b)} postedA={Legal(Command(a),Command(a))} postedB={Legal(Command(b),Command(b))} "
                + $"layers=" + string.Join(";", layers.Select(l => $"{l.TopDepth:R}:{l.Distance.SignedDistance(new(a.X,a.Y)):R}/{l.Boundary.QueryDistance(a.X,a.Y):R}")));
        // F3 planar-cell boundaries make the posted clearance field discrete.
        // A proved chord at command resolution must not keep subdividing to
        // chase that discontinuity; doing so adds no machinable XY detail.
        if (legal && (sourceLengthSquared <= .000004
            || lengthSquared <= .25 && Math.Abs(mid.Z - (a.Z + b.Z) / 2) <= .01
                && Math.Abs(mid.X - (a.X + b.X) / 2) <= .002
                && Math.Abs(mid.Y - (a.Y + b.Y) / 2) <= .002))
        { output.Add(b); return; }
        if (level < 16 && sourceLengthSquared > .00000001)
        {
            subdivisions++;
            Append(sourceA, midSource, a, mid, output, level + 1);
            Append(midSource, sourceB, output[^1], b, output, level + 1);
            return;
        }
        // Never delete the branch or credit an unproved chord. Very short
        // posted XY transitions may need a shallower common tool pose.
        double low = 0, high = Math.Min(-a.Z, -b.Z);
        for (int i = 0; i < 24; i++)
        {
            double candidate = (low + high) / 2;
            if (IsLegalChord(new(a.X, a.Y, -candidate), new(b.X, b.Y, -candidate))) low = candidate;
            else high = candidate;
        }
        var safeA = new Point3D(a.X, a.Y, -low);
        var safeB = new Point3D(b.X, b.Y, -low);
        // The extra axial segment only retracts inside the already-proved
        // cutter envelope at A. B has no following descent into unproved stock.
        output.Add(safeA); output.Add(safeB);
        if (++retreats <= 8)
            diagnostic?.Invoke($"stage=material-surface event=quantized-chord-retreat xy=({a.X:R},{a.Y:R})-({b.X:R},{b.Y:R}) depth={low:R}");
    }

    internal void Verify(IReadOnlyList<TaggedToolpath> paths)
    {
        foreach (var path in paths)
        for (int i = 1; i < path.Points.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var a = path.Points[i - 1]; var b = path.Points[i];
            if (!IsLegalChord(new(a.X, a.Y, a.Z - surface), new(b.X, b.Y, b.Z - surface), postingReserve:false))
                throw new InvalidOperationException($"Height-aware V-carve cutter clearance failed: region={path.RegionIndex}, category={path.Category}, chord={i-1}, spans={string.Join(';',path.Spans)}, a={a}, b={b}.");
        }
        diagnostic?.Invoke($"stage=material-surface event=verified originalMedialPoints={sourcePoints} "
            + $"retainedMedialPoints={outputPoints} loweredSamples={loweredPoints} maximumAdditionalDepth={largestDescent:R} "
            + $"chordChecks={chordChecks} verticalSweepsReduced={verticalSweepsReduced} qualifiedSourceChords={qualifiedSourceChords} raisedSourceProofs={raisedSourceProofs} subdivisions={subdivisions} quantizedRetreats={retreats} fittedRetreats={fittedRetreats} "
            + $"continuousQueries={projectedStock.Queries} continuousCells={projectedStock.Cells} "
            + $"continuousColumnQueries={projectedStock.ColumnQueries} continuousColumnTests={projectedStock.ColumnTests} "
            + $"continuousImprovedSites={projectedStock.ImprovedSites} continuousMaximumImprovement={projectedStock.LargestImprovement:R} "
            + $"continuousCacheHits={projectedStock.CacheHits} continuousRefusals={projectedStock.Refusals} "
            + $"flatWitnessRefusals={projectedStock.FlatWitnessRefusals} priorProfileChords={projectedStock.ProfileChords} "
            + $"stockCellCacheHits={projectedStock.StockCellCacheHits} stockCellCacheMisses={projectedStock.StockCellCacheMisses} stockCellCacheEvictions={projectedStock.StockCellCacheEvictions} cellGeometryCacheHits={projectedStock.CellGeometryCacheHits} cellGeometryCacheMisses={projectedStock.CellGeometryCacheMisses} cachedSweepProofs={projectedStock.CachedSweepProofs} sweepProofCacheLimit={VCarveProjectedStock.SweepProofCacheLimit} sweepProofCacheEvictions={projectedStock.SweepProofCacheEvictions} reversedSweepProofHits={projectedStock.ReversedSweepProofHits} strongerSweepProofHits={projectedStock.StrongerSweepProofHits} depthWitnessQueries={projectedStock.DepthWitnessQueries} depthWitnessBounds={projectedStock.DepthWitnessBounds} depthWitnessSeedFits={projectedStock.DepthWitnessSeedFits} stockCellRootWidth=16 stockCellCacheLimit=1048576 "
            + $"nearbyDepthTrials={projectedStock.NearbyDepthTrials} nearbyDepthFits={projectedStock.NearbyDepthFits} nearbyDepthRefusals={projectedStock.NearbyDepthRefusals} rejectedHighDepthTrials={projectedStock.RejectedHighDepthTrials} intermediateDepthFits={projectedStock.IntermediateDepthFits} "
            + $"raisedPoseProofHits={raisedPoseProofHits} raisedPoseProofPublications={raisedPoseProofPublications} raisedPoseProofLimit={LegalPoseCacheLimit} "
            + $"postedChordRefusals={postedChordRefusals} nativeChordRefusals={nativeChordRefusals} "
            + $"pointDiscLayerProofs={pointDiscLayerProofs} pointDiscReserve={PreparedRegionDistance.Uncertainty:R} "
            + $"refusalWitnessHits={projectedStock.RefusalWitnessHits} refusalWitnessPublications={projectedStock.RefusalWitnessPublications} refusalWitnessTrials={projectedStock.RefusalWitnessTrials} "
            + "refusalWitnessAuthority=upper-prior-removal-plus-original-posting-shell-collision "
            + "stockCellMemoKey=exact-dyadic-centre-pair stockCellEviction=FIFO "
            + "deletedBranches=0 nativeAndActualF3=True fullPhysicalProfile=True");
    }
}
