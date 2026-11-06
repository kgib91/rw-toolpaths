using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>A falsifiable decision on one clearance basin in the original artwork graph.</summary>
public sealed record VCarveBasinDecision(int RegionIndex, int BasinIndex, PointD Peak,
    double ClearanceRadius, double OriginalPeakDepth, double SmoothedPeakDepth,
    double BoundedMaximumDepth, double RoughingFloor, double Uncertainty,
    bool OmitRoughing, string Reason, double ProtectedArea, bool EntireSourceRegion,
    bool IsAttachment = false);

/// <summary>Stock that the coordinated rougher must preserve for the smooth V-carver.</summary>
public sealed class VCarveRoughingAnalysis
{
    internal VCarveRoughingAnalysis(VCarvePreparation preparation,
        IReadOnlyList<IReadOnlyList<PointD>> boundary, IReadOnlyList<VCarveBasinDecision> basins,
        IReadOnlySet<int> protectedSources, IReadOnlySet<int> entireSources,
        Func<IReadOnlyList<VCarveBasinDecision>>? completeBasinDiagnostics = null)
    {
        Preparation = preparation;
        ProtectedBoundary = boundary;
        _basins = basins;
        _completeBasins = completeBasinDiagnostics is null ? null : new(completeBasinDiagnostics,
            LazyThreadSafetyMode.ExecutionAndPublication);
        ProtectedSourceRegionIndices = protectedSources;
        EntireProtectedSourceRegionIndices = entireSources;
        ProtectedRegions = PolygonOps.DifferenceToComponents(boundary, Array.Empty<IReadOnlyList<PointD>>())
            .Select(component => new MillingRegion(component.Outer, component.Holes)).ToArray();
        ProtectedArea = Math.Abs(boundary.Sum(ring => Clipper.Area(new PathD(ring))));
    }

    public VCarvePreparation Preparation { get; }
    public IReadOnlyList<IReadOnlyList<PointD>> ProtectedBoundary { get; }
    public IReadOnlyList<MillingRegion> ProtectedRegions { get; }
    private readonly IReadOnlyList<VCarveBasinDecision> _basins;
    private readonly Lazy<IReadOnlyList<VCarveBasinDecision>>? _completeBasins;
    public IReadOnlyList<VCarveBasinDecision> Basins => _completeBasins?.Value ?? _basins;
    /// <summary>Summary counts require no detailed depth-field materialization.</summary>
    public int DecisionCount => _basins.Count;
    public int OmittedDecisionCount => _basins.Count(basin => basin.OmitRoughing);
    public IReadOnlySet<int> ProtectedSourceRegionIndices { get; }
    public IReadOnlySet<int> EntireProtectedSourceRegionIndices { get; }
    public double ProtectedArea { get; }

    private VCarveRoughingAnalysis(VCarveRoughingAnalysis source, VCarvePreparation preparation)
    {
        Preparation = preparation;
        ProtectedBoundary = source.ProtectedBoundary;
        ProtectedRegions = source.ProtectedRegions;
        _basins = source._basins;
        _completeBasins = source._completeBasins;
        ProtectedSourceRegionIndices = source.ProtectedSourceRegionIndices;
        EntireProtectedSourceRegionIndices = source.EntireProtectedSourceRegionIndices;
        ProtectedArea = source.ProtectedArea;
    }

    /// <summary>
    /// Reuses the complete analytical decision with a separate compilation's finish-proof owner.
    /// No basin analysis, polygon construction or protected geometry is repeated.
    /// </summary>
    public VCarveRoughingAnalysis WithPreparation(VCarvePreparation preparation)
    {
        ArgumentNullException.ThrowIfNull(preparation);
        if (preparation.Fingerprint != Preparation.Fingerprint
            || !ReferenceEquals(preparation.Regions, Preparation.Regions))
            throw new ArgumentException("An analysis can only use an independent finish scope of its original preparation.", nameof(preparation));
        return new VCarveRoughingAnalysis(this, preparation);
    }

}

/// <summary>
/// Completed original clearance graph and physical depth preparation. The graph remains independent
/// of roughing masks, path routing, compensation, stock pruning and machine command reserves.
/// </summary>
public sealed partial class VCarvePreparation
{
    // Preserve the original ambient budget for optional pure prefetch builders.
    // Only an explicit false selects the one-worker diagnostic comparison.
    // Exact request keys retain the caller's ambient policy in either case.
    internal static readonly AsyncLocal<bool?> InitialOutlinePrefetchAmbientWorkers=new();
    internal readonly record struct OutlinePrefetchWorkerBudget(string Phase,int AmbientGeometryWorkers,
        int AmbientStockFitWorkers,int EffectiveGeometryWorkers,int EffectiveStockFitWorkers,
        int RestoredGeometryWorkers,int RestoredStockFitWorkers);
    // Reuse the established smooth-coverage geometric error; this is not a pocket-size cutoff.
    public const double GeometricErrorBound = VCarveSmoothing.CoverageTolerance;
    private const double MaximumSampleSpacing = .25;
    private readonly ToolGeometry _tool;
    private readonly DepthSchedule _depth;
    private readonly VCarveOptions _options;
    private const long MaximumOutlineBytes=64L<<20;
    private sealed class OutlineState
    {
        internal readonly FilledFrontEvidenceCache.Cache<OutlineSnapshot> Cache=new(MaximumOutlineBytes,4);
        internal readonly object Gate=new();
        internal OutlineFlight? Flight;
        internal long Constructions;
        internal OutlinePrefetchWorkerBudget? LastWorkerBudget;
    }
    private sealed class OutlineFlight(FilledFrontEvidenceCache.Key key)
    {
        internal readonly FilledFrontEvidenceCache.Key Key=key;
        internal Task Work=Task.CompletedTask;
    }
    private sealed class OutlineSnapshot
    {
        // The entire original graph stays private until Cache.Take atomically
        // transfers its sole ownership. Reconstructing OffsetRing would repeat
        // winding/area arithmetic; sharing mutable Points/Children is forbidden.
        internal readonly List<OffsetRing> Rings;
        internal readonly RestFillPrefetch? RestFill;
        internal OutlineSnapshot(List<OffsetRing> rings,RestFillPrefetch? restFill=null)
        {Rings=rings;RestFill=restFill;}
    }
    internal sealed class RestFillKey
    {
        private readonly FilledFrontEvidenceCache.Key boundary,crests,target;
        private readonly string policy;
        private readonly ulong hash;
        internal long Bytes=>boundary.Bytes+crests.Bytes+target.Bytes+64L+2L*policy.Length;
        internal ulong Hash=>hash;
        private RestFillKey(FilledFrontEvidenceCache.Key boundary,FilledFrontEvidenceCache.Key crests,
            FilledFrontEvidenceCache.Key target,string policy)
        {
            this.boundary=boundary;this.crests=crests;this.target=target;this.policy=policy;
            hash=unchecked((boundary.Hash*1099511628211UL^crests.Hash)*1099511628211UL^target.Hash);
            foreach(char word in policy)hash=unchecked((hash^word)*1099511628211UL);
        }
        internal bool Same(RestFillKey other)=>policy==other.policy&&boundary.Same(other.boundary)
            &&crests.Same(other.crests)&&target.Same(other.target);
        internal static RestFillKey? Capture(VCarveRampSurface.RestFillRequest request,CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            // Sources currently construct exactly two crest endpoints at Z=0.
            // Refuse unsupported future source layouts rather than omit a
            // consumed ordinate from this exact-input key.
            if(request.Crests.Any(p=>p.Points.Any(q=>BitConverter.DoubleToInt64Bits(q.Z)!=0)))return null;
            var boundary=FilledFrontEvidenceCache.Key.Capture(request.Boundary,request.Depth,request.Grade,request.DiscRadius,1,null,token);
            var crests=FilledFrontEvidenceCache.Key.Capture(request.Crests.Select(p=>(IReadOnlyList<PointD>)p.Points
                .Select(q=>new PointD(q.X,q.Y)).ToArray()),request.Depth,request.Grade,request.DiscRadius,2,null,token);
            var target=FilledFrontEvidenceCache.Key.Capture(request.Target??Array.Empty<IReadOnlyList<PointD>>(),
                request.Depth,request.Grade,request.DiscRadius,request.Target is null?0:1,null,token);
            return boundary is null||crests is null||target is null?null:new(boundary,crests,target,OutlinePolicy(request.PlateauConstantOffsetLeafVertices));
        }
    }
    internal sealed class RestFillPrefetch(RestFillKey key,VCarveRampSurface.RestFillPreparation value)
    {
        private int consumed;
        internal long Bytes=>96+key.Bytes+value.EstimatedBytes;
        internal VCarveRampSurface.RestFillPreparation? Take(VCarveRampSurface.RestFillRequest request,
            CancellationToken token,Action<string>? diagnostic)
        {
            token.ThrowIfCancellationRequested();var actual=RestFillKey.Capture(request,token);
            bool hit=Interlocked.Exchange(ref consumed,1)==0&&actual is not null&&key.Same(actual);
            diagnostic?.Invoke($"stage=rest-fill-prefetch event={(hit?"hit":"miss")} keyHash={key.Hash:X16} "
                +$"actualKeyHash={(actual is null?"unavailable":actual.Hash.ToString("X16",CultureInfo.InvariantCulture))} exactOrderedInputs=True consumedOnce={hit} "
                +$"retainedBytes={Bytes} plateauBuildMs={value.PlateauMilliseconds:F3} discBuildMs={value.DiscMilliseconds:F3} "
                +"stockRead=False footprintCredit=False physicalGuardsUnchanged=True");
            token.ThrowIfCancellationRequested();return hit?value:null;
        }
    }
    private readonly OutlineState outlines;
    internal FilledFrontEvidenceCache.State InitialOutlineCacheState=>outlines.Cache.Inspect();
    internal long InitialOutlineConstructions=>Interlocked.Read(ref outlines.Constructions);
    internal OutlinePrefetchWorkerBudget? InitialOutlinePrefetchBudget
    {get{lock(outlines.Gate)return outlines.LastWorkerBudget;}}
    private CompletedFinish? _completedFinish;
    // The consumer permits .001mm comparison tolerance. Withhold that amount
    // from an actual commanded physical witness so tolerance cannot add stock.
    private const double FinishComparisonReserve = .0010000001;
    internal IReadOnlyList<PreparedRegion> Regions { get; }
    public string Fingerprint { get; }
    /// <summary>Original source geometry and region mapping, independent of machining settings.</summary>
    public string OriginalGeometryFingerprint { get; }
    public IReadOnlyList<MillingRegion> OriginalRegions { get; }

    /// <summary>Global slope bound of the conservative commanded physical removal certificate.</summary>
    public double PreservedFinishDepthLipschitz => PhysicalWitnessSlope();

    /// <summary>Whether an entire completed actual command batch owns the removal proof.</summary>
    public bool HasCompletedFinish => Volatile.Read(ref _completedFinish) is not null;

    /// <summary>
    /// Creates an unbound finish-proof owner for one compilation while sharing the completed
    /// original graph and immutable analytical inputs. A cached preparation's actual finish proof
    /// is never inherited or cleared by this scope.
    /// </summary>
    public VCarvePreparation CreateIndependentFinishScope()
    {
        var scope = new VCarvePreparation(this);
        Log(_options, "event=finish-scope-created analyticalPreparationReused=True originalGraphReused=True "
            + "completedFinishInherited=False owner=independent-compilation");
        return scope;
    }

    /// <summary>Discards removal proof if its containing compilation fails or is canceled.</summary>
    public void ClearCompletedFinish()
    {
        Volatile.Write(ref _completedFinish, null);
        Log(_options, "event=finish-certificate-cleared owner=failed-or-canceled-compilation removalProof=0");
    }

    /// <summary>
    /// Binds completed actual machine chords, after smoothing, routing, stock filtering and F3
    /// posting. XY are commanded planar coordinates; Z is commanded stock-relative tip height
    /// (negative below the surface). A canceled or incomplete build never publishes a certificate.
    /// </summary>
    public void BindCompletedFinish(IEnumerable<(Point3D Start, Point3D End)> commandedChords,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(commandedChords);
        double slope = PhysicalWitnessSlope();
        if (!double.IsFinite(slope) || slope <= 0)
            throw new InvalidOperationException("The physical cutter cannot provide a finite finish-removal witness slope.");
        var elapsed = System.Diagnostics.Stopwatch.StartNew();
        Log(_options, "event=finish-certificate-start owner=completed-actual-F3-physical-sweeps publication=staged");
        double maximumDepth = Math.Min(_depth.Depth,
            _tool.VCarveProfile?.MaximumDepth ?? _tool.ConeLength ?? _depth.Depth);
        var segments = new List<FinishSegment>();
        foreach (var chord in commandedChords)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (!double.IsFinite(chord.Start.X + chord.Start.Y + chord.Start.Z)
                || !double.IsFinite(chord.End.X + chord.End.Y + chord.End.Z))
                throw new ArgumentException("A completed finish chord contains a nonfinite coordinate.", nameof(commandedChords));
            if (Math.Max(-chord.Start.Z, -chord.End.Z) <= 0) continue;
            // A nominal/tool cap can only lower the witness of actual F3 contact.
            var a = chord.Start with { Z = Math.Max(chord.Start.Z, -maximumDepth) };
            var b = chord.End with { Z = Math.Max(chord.End.Z, -maximumDepth) };
            segments.Add(new FinishSegment(a, b));
        }
        var completed = new CompletedFinish(segments.ToArray(), slope, cancellationToken);
        cancellationToken.ThrowIfCancellationRequested();
        Volatile.Write(ref _completedFinish, completed);
        Log(_options, $"event=finish-certificate-bound commandedChords={segments.Count} "
            + $"witnessSlope={completed.Slope:R} axialComparisonReserve={FinishComparisonReserve:R} elapsedMs={elapsed.Elapsed.TotalMilliseconds:R} "
            + "owner=completed-actual-F3-physical-sweeps surfaceEntriesAndStockFilteringIncluded=True incompletePublished=False");
    }

    /// <summary>
    /// Conservative physically removed depth from completed actual commanded chords. Analytical
    /// smoothed ceilings do not prove removal after entry ramps and command-wall repairs. An unbound
    /// preparation therefore certifies zero removal. Each witness uses an enclosing conical underside
    /// with finite slope; another cutter must check every point of its complete contact footprint.
    /// </summary>
    public double MaximumPreservedFinishDepth(int sourceRegion, double x, double y,
        CancellationToken cancellationToken = default)
    {
        if (sourceRegion < 0 || sourceRegion >= Regions.Count) throw new ArgumentOutOfRangeException(nameof(sourceRegion));
        if (!double.IsFinite(x) || !double.IsFinite(y)) throw new ArgumentOutOfRangeException(nameof(x));
        cancellationToken.ThrowIfCancellationRequested();
        var completed = Volatile.Read(ref _completedFinish);
        return completed is null ? 0 : Math.Max(0, completed.MaximumDepth(x, y, cancellationToken) - FinishComparisonReserve);
    }

    private readonly record struct FinishSegment(Point3D A, Point3D B)
    {
        internal double MaximumDepth => Math.Max(-A.Z, -B.Z);
    }

    private sealed class CompletedFinish
    {
        private sealed record Node(double X0, double Y0, double X1, double Y1, double Depth,
            int First, int Count, Node? Left, Node? Right);
        private readonly FinishSegment[] _segments;
        private readonly Node? _root;
        internal double Slope { get; }

        internal CompletedFinish(FinishSegment[] segments, double slope, CancellationToken token)
        {
            _segments = segments; Slope = slope;
            _root = segments.Length == 0 ? null : Build(0, segments.Length);

            Node Build(int first, int count)
            {
                token.ThrowIfCancellationRequested();
                double x0 = double.PositiveInfinity, y0 = double.PositiveInfinity;
                double x1 = double.NegativeInfinity, y1 = double.NegativeInfinity, depth = 0;
                for (int i = first; i < first + count; i++)
                {
                    var s = segments[i];
                    x0 = Math.Min(x0, Math.Min(s.A.X, s.B.X)); y0 = Math.Min(y0, Math.Min(s.A.Y, s.B.Y));
                    x1 = Math.Max(x1, Math.Max(s.A.X, s.B.X)); y1 = Math.Max(y1, Math.Max(s.A.Y, s.B.Y));
                    depth = Math.Max(depth, s.MaximumDepth);
                }
                if (count <= 16) return new Node(x0, y0, x1, y1, depth, first, count, null, null);
                bool xAxis = x1 - x0 >= y1 - y0;
                Array.Sort(segments, first, count, Comparer<FinishSegment>.Create((a, b) =>
                    (xAxis ? a.A.X + a.B.X : a.A.Y + a.B.Y)
                    .CompareTo(xAxis ? b.A.X + b.B.X : b.A.Y + b.B.Y)));
                int half = count / 2;
                return new Node(x0, y0, x1, y1, depth, first, count,
                    Build(first, half), Build(first + half, count - half));
            }
        }

        internal double MaximumDepth(double x, double y, CancellationToken token)
        {
            double best = 0;
            if (_root is not null) Visit(_root);
            return best;

            double Upper(Node node)
            {
                double dx = Math.Max(0, Math.Max(node.X0 - x, x - node.X1));
                double dy = Math.Max(0, Math.Max(node.Y0 - y, y - node.Y1));
                return node.Depth - Slope * Math.Sqrt(dx * dx + dy * dy);
            }

            void Visit(Node node)
            {
                token.ThrowIfCancellationRequested();
                if (Upper(node) <= best) return;
                if (node.Left is { } left && node.Right is { } right)
                {
                    if (Upper(left) >= Upper(right)) { Visit(left); Visit(right); }
                    else { Visit(right); Visit(left); }
                    return;
                }
                for (int i = node.First; i < node.First + node.Count; i++)
                {
                    var s = _segments[i];
                    double dx = s.B.X - s.A.X, dy = s.B.Y - s.A.Y, length = Math.Sqrt(dx * dx + dy * dy);
                    if (length <= 1e-15)
                    {
                        best = Math.Max(best, s.MaximumDepth - Slope * Math.Sqrt(Math.Pow(x - s.A.X, 2) + Math.Pow(y - s.A.Y, 2)));
                        continue;
                    }
                    double k = (s.A.Z - s.B.Z) / length;
                    double along = ((x - s.A.X) * dx + (y - s.A.Y) * dy) / length;
                    double across = Math.Abs((x - s.A.X) * dy - (y - s.A.Y) * dx) / length;
                    double at = k >= Slope ? length : k <= -Slope ? 0
                        : Math.Clamp(along + k * across / Math.Sqrt(Math.Max(1e-30, Slope * Slope - k * k)), 0, length);
                    best = Math.Max(best, -s.A.Z + k * at - Slope * Math.Sqrt(Math.Pow(at - along, 2) + across * across));
                }
            }
        }
    }

    private double PhysicalWitnessSlope()
    {
        var physical = VCarvePhysicalSweep.ProfileFor(_tool,
            Math.Min(_depth.Depth, _tool.VCarveProfile?.MaximumDepth ?? _tool.ConeLength ?? _depth.Depth));
        double slope = 1, height = 0;
        foreach (CarvingCutterSection section in physical.Sections)
        {
            if (section.BottomRadius > 0) slope = Math.Max(slope, height / section.BottomRadius);
            if (section.TopRadius > 0) slope = Math.Max(slope, (height + section.Height) / section.TopRadius);
            height += section.Height;
        }
        return slope;
    }

    internal sealed record PreparedRegion
    {
        public int RegionIndex { get; init; }
        public IReadOnlyList<IReadOnlyList<PointD>> Boundary { get; init; }
        public IReadOnlyList<MedialSegment> OriginalMedialGraph { get; init; }
        private IReadOnlyList<IReadOnlyList<MedialPoint>>? _clearance;
        private IReadOnlyList<TaggedToolpath>? _original, _smooth, _upper;
        private double _maximumChordLength;
        private DeferredRegionDepths? _deferred;
        public IReadOnlyList<IReadOnlyList<MedialPoint>> SourceClearancePaths
        { get => _clearance ?? Complete(default).SourceClearancePaths; init { ResolveForEdit();_clearance = value; } }
        public IReadOnlyList<TaggedToolpath> OriginalMedialPaths
        { get => _original ?? Complete(default).OriginalMedialPaths; init { ResolveForEdit();_original = value; } }
        public IReadOnlyList<TaggedToolpath> SmoothedMedialPaths
        { get => _smooth ?? Complete(default).SmoothedMedialPaths; init { ResolveForEdit();_smooth = value; } }
        public IReadOnlyList<TaggedToolpath> UpperMedialPaths
        { get => _upper ?? Complete(default).UpperMedialPaths; init { ResolveForEdit();_upper = value; } }
        public double MaximumChordLength
        { get => _deferred is null ? _maximumChordLength : Complete(default).MaximumChordLength; init { ResolveForEdit();_maximumChordLength = value; } }
        internal bool HasDeferredDepths => _deferred is not null;
        internal bool HasCompletedDepths => _deferred?.HasCompleted ?? true;
        internal double? SampledUpperWitness => _deferred?.SampledUpperWitness;
        internal PreparedRegion Complete(CancellationToken token) => _deferred?.Complete(token) ?? this;
        private void ResolveForEdit()
        {
            if(_deferred is null)return;
            var complete=Complete(default);
            _clearance=complete._clearance;_original=complete._original;_smooth=complete._smooth;
            _upper=complete._upper;_maximumChordLength=complete._maximumChordLength;
            _deferred=null;
        }

        internal PreparedRegion(int regionIndex,IReadOnlyList<IReadOnlyList<PointD>> boundary,
            IReadOnlyList<MedialSegment> originalMedialGraph,IReadOnlyList<IReadOnlyList<MedialPoint>> sourceClearancePaths,
            IReadOnlyList<TaggedToolpath> originalMedialPaths,IReadOnlyList<TaggedToolpath> smoothedMedialPaths,
            IReadOnlyList<TaggedToolpath> upperMedialPaths,double maximumChordLength)
        {
            RegionIndex=regionIndex;Boundary=boundary;OriginalMedialGraph=originalMedialGraph;
            _clearance=sourceClearancePaths;_original=originalMedialPaths;_smooth=smoothedMedialPaths;
            _upper=upperMedialPaths;_maximumChordLength=maximumChordLength;
        }
        internal PreparedRegion(int regionIndex,IReadOnlyList<IReadOnlyList<PointD>> boundary,
            IReadOnlyList<MedialSegment> originalMedialGraph,DeferredRegionDepths deferred)
        {RegionIndex=regionIndex;Boundary=boundary;OriginalMedialGraph=originalMedialGraph;_deferred=deferred;}
    }

    private VCarvePreparation(IReadOnlyList<MillingRegion> originals, ToolGeometry tool,
        DepthSchedule depth, VCarveOptions options, IReadOnlyList<PreparedRegion> regions,OutlineState? outlineState=null)
    {
        OriginalRegions = originals; _tool = tool; _depth = depth; _options = options;
        Regions = regions; Fingerprint = ComputeFingerprint(originals, tool, depth, options);
        OriginalGeometryFingerprint = ComputeOriginalGeometryFingerprint(originals);
        outlines=outlineState??new();
    }

    private VCarvePreparation(VCarvePreparation source)
    {
        OriginalRegions = source.OriginalRegions;
        _tool = source._tool;
        _depth = source._depth;
        _options = source._options;
        Regions = source.Regions;
        Fingerprint = source.Fingerprint;
        OriginalGeometryFingerprint = source.OriginalGeometryFingerprint;
        outlines=source.outlines;
        // CompletedFinish remains null: it is owned by this compilation, never by the cache.
    }

    /// <summary>Overlaps stock-independent outline/rest-fill nominations with this compilation's
    /// preceding pocket work. Dispose cancels and joins unfinished optional work;
    /// caller cancellation remains observable. No actual stock/protection work runs here.</summary>
    public IDisposable? PrefetchInitialOutline(CancellationToken token=default)=>PrefetchInitialOutline(token,4096);
    internal OutlinePrefetchLease? PrefetchInitialOutline(CancellationToken token,int minimumBoundaryPoints)
    {
        token.ThrowIfCancellationRequested();
        lock(outlines.Gate)if(outlines.Flight is not null)return null;
        var region=Regions.Where(r=>r.Boundary.Sum(p=>(long)p.Count)>=minimumBoundaryPoints)
            .OrderByDescending(r=>r.Boundary.Sum(p=>(long)p.Count)).FirstOrDefault();
        if(region is null)return null;
        double depth=Math.Min(_depth.Depth,_tool.VCarveProfile?.MaximumDepth??_tool.ConeLength??0);
        var boundary=VCarveRampSurface.OutlineBoundary(region.Boundary,_options);
        var request=VCarveRampSurface.PrepareInitialOutline(boundary,_options,VCarvePhysicalSweep.ProfileFor(_tool,depth),depth,token);
        if(request.Construction is null)return null;
        var (key,policy)=OutlineKey(request,token);
        if(key is null||outlines.Cache.Find(key,token) is not null)return null;
        var lifetime=CancellationTokenSource.CreateLinkedTokenSource(token);
        OutlineFlight flight;
        lock(outlines.Gate)
        {
            if(outlines.Flight is not null){lifetime.Dispose();return null;}
            flight=new(key);outlines.Flight=flight;
            flight.Work=Task.Run(()=>
            {
                try
                {
                    Log(_options,$"event=outline-prefetch state=started region={region.RegionIndex} keyHash={key.Hash:X16} stockRead=False prefetchTasks=1 geometryWorkers={GeometryParallelism.WorkerLimit} stockFitWorkers={GeometryParallelism.StockFitWorkerLimit} ambientWorkerControl={InitialOutlinePrefetchAmbientWorkers.Value is not false} ambientWorkerControlRaw={InitialOutlinePrefetchAmbientWorkers.Value?.ToString()??"default"} pureBuilderWorkers={(InitialOutlinePrefetchAmbientWorkers.Value is false?1:GeometryParallelism.WorkerLimit)}");
                    ConstructInitialOutline(request,key,policy,lifetime.Token,prefetch:true);
                }
                catch(Exception error)
                {
                    Log(_options,$"event=outline-prefetch state={(lifetime.IsCancellationRequested?"canceled":"failed")} region={region.RegionIndex} keyHash={key.Hash:X16} detail='{error}' publishedPartial=False");
                    throw;
                }
                finally{lock(outlines.Gate)if(ReferenceEquals(outlines.Flight,flight))outlines.Flight=null;}
            },CancellationToken.None);
        }
        return new(lifetime,flight.Work,token,message=>Log(_options,message));
    }
    private sealed class PrefetchWorkerScope:IDisposable
    {
        private readonly VCarvePreparation owner;
        private readonly string phase;
        private readonly int ambientGeometry,ambientStock,effectiveGeometry,effectiveStock;
        private readonly IDisposable? scope;
        private int disposed;
        internal PrefetchWorkerScope(VCarvePreparation owner,string phase)
        {
            this.owner=owner;this.phase=phase;
            ambientGeometry=GeometryParallelism.WorkerLimit;ambientStock=GeometryParallelism.StockFitWorkerLimit;
            scope=InitialOutlinePrefetchAmbientWorkers.Value is false?GeometryParallelism.EnterWorker():null;
            effectiveGeometry=GeometryParallelism.WorkerLimit;effectiveStock=GeometryParallelism.StockFitWorkerLimit;
            try{Log(owner._options,$"event=outline-prefetch-budget state=started phase={phase} ambientGeometryWorkers={ambientGeometry} ambientStockFitWorkers={ambientStock} effectiveGeometryWorkers={effectiveGeometry} effectiveStockFitWorkers={effectiveStock} sourceAndUnionOperands=unchanged");}
            catch{scope?.Dispose();throw;}
        }
        public void Dispose()
        {
            if(Interlocked.Exchange(ref disposed,1)!=0)return;
            scope?.Dispose();
            int restoredGeometry=GeometryParallelism.WorkerLimit,restoredStock=GeometryParallelism.StockFitWorkerLimit;
            lock(owner.outlines.Gate)owner.outlines.LastWorkerBudget=new(phase,ambientGeometry,ambientStock,
                effectiveGeometry,effectiveStock,restoredGeometry,restoredStock);
            Log(owner._options,$"event=outline-prefetch-budget state=restored phase={phase} effectiveGeometryWorkers={effectiveGeometry} effectiveStockFitWorkers={effectiveStock} restoredGeometryWorkers={restoredGeometry} restoredStockFitWorkers={restoredStock} keyPolicy=original-ambient workerScopeJoined=True");
        }
    }
    internal sealed class OutlinePrefetchLease(CancellationTokenSource lifetime,Task work,CancellationToken callerToken,Action<string> log):IDisposable
    {
        private int disposed;
        internal Task Completion=>work;
        public void Dispose()
        {
            if(Interlocked.Exchange(ref disposed,1)!=0)return;
            lifetime.Cancel();
            try{work.GetAwaiter().GetResult();}
            catch(OperationCanceledException)when(lifetime.IsCancellationRequested){}
            catch(Exception error){log($"event=outline-prefetch state=joined-failure detail='{error}' actualConstructionStillRequired=True");}
            finally{lifetime.Dispose();}
            log($"event=outline-prefetch state=joined completed={work.IsCompletedSuccessfully} detachedWorker=False");
            callerToken.ThrowIfCancellationRequested();
        }
    }
    private static string OutlinePolicy(int plateauConstantOffsetLeafVertices)
    {
        // Generic offset construction has development-only runtime selectors.
        // Capture their complete names/values, rather than loosen the key when
        // a diagnostic process changes a selector between preparation and use.
        var values=Environment.GetEnvironmentVariables().Cast<System.Collections.DictionaryEntry>()
            .Where(e=>e.Key is string name&&name.StartsWith("RW_TOOLPATHS_",StringComparison.Ordinal))
            .OrderBy(e=>(string)e.Key,StringComparer.Ordinal);
        return $"geometryWorkers={GeometryParallelism.WorkerLimit}\0stockFitWorkers={GeometryParallelism.StockFitWorkerLimit}\0"
            +string.Concat(values.Select(e=>$"{e.Key}\0{e.Value}\0"))
            +(plateauConstantOffsetLeafVertices==SmoothRoughingPlateau.LegacyConstantOffsetLeafVertices?string.Empty
                :$"plateauConstantOffsetLeafVertices={plateauConstantOffsetLeafVertices}\0");
    }
    private static (FilledFrontEvidenceCache.Key? Key,string Policy) OutlineKey(VCarveRampSurface.InitialOutline request,CancellationToken token)
    {
        string policy=OutlinePolicy(request.PlateauConstantOffsetLeafVertices);var construction=request.Construction!;
        var policyWords=policy.Select(c=>new PointD(c,0)).ToArray();
        var key=FilledFrontEvidenceCache.Key.Capture(request.Boundary.Append(policyWords),construction.FirstOffset,
            construction.StepOver,construction.MaximumInset,construction.RingLimit,null,token);
        return(key,policy);
    }
    internal List<OffsetRing> InitialOutlineRings(VCarveRampSurface.InitialOutline request,CancellationToken token,Action<string>? diagnostic)
        =>InitialOutlineRings(request,token,diagnostic,out _);
    internal List<OffsetRing> InitialOutlineRings(VCarveRampSurface.InitialOutline request,CancellationToken token,Action<string>? diagnostic,
        out RestFillPrefetch? restFill)
    {
        restFill=null;
        if(request.Construction is null){token.ThrowIfCancellationRequested();return VCarveRampSurface.BuildInitialOutline(request,token);}
        token.ThrowIfCancellationRequested();var (key,policy)=OutlineKey(request,token);
        var cached=outlines.Cache.Take(key,token);
        if(cached is null&&key is not null)
        {
            Task? pending;
            lock(outlines.Gate)pending=outlines.Flight is {} flight&&flight.Key.Same(key)?flight.Work:null;
            if(pending is not null)
            {
                diagnostic?.Invoke($"stage=outline-prefetch event=waiting keyHash={key.Hash:X16} exactOrderedInputs=True stockRead=False");
                try{pending.WaitAsync(token).GetAwaiter().GetResult();}
                catch(Exception)when(!token.IsCancellationRequested){/* Optional work failed/canceled; the actual original builder remains authoritative. */}
                token.ThrowIfCancellationRequested();
            }
            // Recheck also closes publication between the first lookup and
            // acquisition of the single-flight owner.
            cached=outlines.Cache.Take(key,token);
        }
        var state=outlines.Cache.Inspect();
        diagnostic?.Invoke($"stage=outline-prefetch event={(cached is null?"miss":"hit")} keyHash={(key is null?"unavailable":key.Hash.ToString("X16",CultureInfo.InvariantCulture))} "
            +$"exactOrderedInputs=True consumedOnce={cached is not null} entries={state.Entries} retainedBytes={state.Bytes} maximumBytes={MaximumOutlineBytes} stockRead=False");
        restFill=cached?.RestFill;
        return cached?.Rings??ConstructInitialOutline(request,key,policy,token);
    }
    private List<OffsetRing> ConstructInitialOutline(VCarveRampSurface.InitialOutline request,FilledFrontEvidenceCache.Key? key,string policy,CancellationToken token,bool prefetch=false)
    {
        long started=System.Diagnostics.Stopwatch.GetTimestamp();Interlocked.Increment(ref outlines.Constructions);
        List<OffsetRing> result;
        using(var worker=prefetch?new PrefetchWorkerScope(this,"outline"):null)
            result=VCarveRampSurface.BuildInitialOutline(request,token);
        token.ThrowIfCancellationRequested();
        long bytes=256L+256L*result.Count+32L*result.Sum(r=>(long)r.Points.Count)+8L*result.Sum(r=>(long)r.Children.Count);
        RestFillPrefetch? restFill=null;
        if(prefetch&&key is not null&&key.Bytes+bytes+96<=MaximumOutlineBytes&&VCarveRampSurface.HasRestFillFront(result,request))
        {
            var restRequest=VCarveRampSurface.CaptureRestFillRequest(request.Boundary,_options,request.Depth,request.Band.Grade,token);
            var restKey=RestFillKey.Capture(restRequest,token);
            if(restKey is not null)
            {
                VCarveRampSurface.RestFillPreparation helper;
                using(var worker=new PrefetchWorkerScope(this,"rest-fill"))
                    helper=VCarveRampSurface.BuildRestFillPreparation(restRequest,token,null);
                var candidate=new RestFillPrefetch(restKey,helper);
                bool fits=key.Bytes+bytes+candidate.Bytes+96<=MaximumOutlineBytes;
                if(fits){restFill=candidate;bytes+=candidate.Bytes;}
                Log(_options,$"event=rest-fill-prefetch state={(fits?"prepared":"raw-only-budget")} keyHash={restKey.Hash:X16} "
                    +$"retainedBytes={candidate.Bytes} plateauBuildMs={helper.PlateauMilliseconds:F3} discBuildMs={helper.DiscMilliseconds:F3} "
                    +"exactOrderedInputs=True stockRead=False footprintCredit=False completedOnly=True");
            }
            else Log(_options,"event=rest-fill-prefetch state=raw-only-key-refusal exactOrderedInputs=True stockRead=False");
        }
        bool admitted=prefetch&&key is not null&&key.Bytes+bytes+96<=MaximumOutlineBytes&&policy==OutlinePolicy(request.PlateauConstantOffsetLeafVertices);
        int ringCount=result.Count;long pointCount=result.Sum(r=>(long)r.Points.Count);
        if(admitted)outlines.Cache.Publish(key,new OutlineSnapshot(result,restFill),bytes,token);
        var state=outlines.Cache.Inspect();
        Log(_options,$"event=outline-prefetch state={(admitted?"published":prefetch?"uncached":"literal")} keyHash={(key is null?"unavailable":key.Hash.ToString("X16",CultureInfo.InvariantCulture))} "
            +$"rings={ringCount} points={pointCount} elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
            +$"entries={state.Entries} retainedBytes={state.Bytes} maximumBytes={MaximumOutlineBytes} exactOrderedInputs=True stockRead=False completedOnly=True");
        return result;
    }

    private sealed class SourceRadiusMemo
    {
        internal readonly Dictionary<(double X,double Y),double> Values=new();
        internal long Queries,Hits,Clears;
    }

    internal static VCarvePreparation Create(IReadOnlyList<MillingRegion> regions, ToolGeometry tool,
        DepthSchedule depth, VCarveOptions options, IMedialAxisProvider provider,
        CancellationToken cancellationToken, bool deferBoundedFinishDepths=false)
    {
        ArgumentNullException.ThrowIfNull(regions); ArgumentNullException.ThrowIfNull(tool);
        ArgumentNullException.ThrowIfNull(depth); ArgumentNullException.ThrowIfNull(options);
        ArgumentNullException.ThrowIfNull(provider);
        // Retained preparation and its deferred consumers must use the policy
        // captured before this request starts, even if the ambient trial changes.
        options=options with{PlateauConstantOffsetLeafVertices=SmoothRoughingPlateau.CaptureConstantOffsetLeafVertices(options)};
        if (provider is NativeMedialAxisProvider)
            throw new InvalidOperationException("The approximate sampled medial provider cannot certify automatic roughing omission; keep ordinary roughing.");
        if (!options.SmoothVCarving || !double.IsFinite(options.SmoothRampAngleDegrees)
            || options.SmoothRampAngleDegrees <= 0 || options.SmoothRampAngleDegrees >= 90)
            throw new ArgumentException("Preparation requires valid smooth V-carving settings.", nameof(options));
        double maximumDepth = Math.Min(depth.Depth, tool.VCarveProfile?.MaximumDepth ?? tool.ConeLength ?? 0);
        if (!(maximumDepth > 0)) throw new ArgumentException("Preparation requires a physical cutting depth.", nameof(depth));
        var physical = VCarvePhysicalSweep.ProfileFor(tool, maximumDepth);
        double grade = Math.Tan(options.SmoothRampAngleDegrees * Math.PI / 180);
        var prepared = new List<PreparedRegion>(regions.Count);
        var originals = regions.Select(region => new MillingRegion(region.Outer.ToArray(),
            region.Holes.Select(hole => (IReadOnlyList<PointD>)hole.ToArray()).ToArray())).ToArray();
        for (int regionIndex = 0; regionIndex < originals.Length; regionIndex++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            long regionStarted=System.Diagnostics.Stopwatch.GetTimestamp();
            var boundary = VCarveToolpaths.BuildBoundary(originals[regionIndex], options.StockToLeave);
            var raw = new List<MedialSegment>();
            foreach (var component in PolygonOps.DifferenceToComponents(boundary, Array.Empty<IReadOnlyList<PointD>>()))
            {
                double contact = options.PreparationContactRadius;
                double span = Math.Min(component.Outer.Max(p => p.x) - component.Outer.Min(p => p.x),
                    component.Outer.Max(p => p.y) - component.Outer.Min(p => p.y));
                if (contact > 0 && span / 2 + 2 * GeometricErrorBound < contact)
                {
                    Log(options, $"event=unreachable-component region={regionIndex} proof=enclosing-box "
                        + $"inradiusUpper={span / 2 + 2 * GeometricErrorBound:R} contactRadius={contact:R} "
                        + "originalSourcePreserved=True graphNotRequired=True");
                    continue;
                }
                raw.AddRange(MedialAxisGeometry.Build(component.Outer, component.Holes, provider,
                    options.Tolerance, maxRadius: 0, cancellationToken: cancellationToken,
                    diagnostic: message => Log(options, $"region={regionIndex} {message}"),
                    maximumGeometricError: GeometricErrorBound / 4, preserveNativeGrid: options.PreparationContactRadius > 0));
            }
            double medialMs=System.Diagnostics.Stopwatch.GetElapsedTime(regionStarted).TotalMilliseconds;
            int capturedRegion=regionIndex;
            PreparedRegion Complete(CancellationToken token)=>PrepareRegionDepths(capturedRegion,boundary,raw,
                options,physical,maximumDepth,medialMs,token);
            if(deferBoundedFinishDepths&&raw.Count>=4096
                &&TrySampledUpperWitness(regionIndex,boundary,raw,options,physical,maximumDepth,grade,cancellationToken,out double witness))
                prepared.Add(new PreparedRegion(regionIndex,boundary,raw,
                    new DeferredRegionDepths(Complete,witness,options,regionIndex)));
            else prepared.Add(Complete(cancellationToken));
        }
        cancellationToken.ThrowIfCancellationRequested();
        return new VCarvePreparation(originals, tool, depth, options, prepared);
    }

    /// <summary>Strict bounded floor comparison; equality, incompatible depth and uncertainty keep roughing.</summary>
    public VCarveRoughingAnalysis AnalyzeRoughing(double roughingFloor,
        CancellationToken cancellationToken = default)
        => AnalyzeRoughingCore(roughingFloor, cancellationToken, plateauOnly: false);

    // Pocket owns its plateau independently of a later operation. The public
    // V-carve basin analysis retains its separate historical contract.
    internal VCarveRoughingAnalysis AnalyzePlateauRoughing(double roughingFloor,
        CancellationToken cancellationToken = default)
        => AnalyzeRoughingCore(roughingFloor, cancellationToken, plateauOnly: true);

    /// <summary>Full-depth Pocket plateau from this finishing surface. Cutter compensation
    /// remains in the Pocket engine; allowance never moves the source ramp anchors.</summary>
    public VCarveRoughingAnalysis AnalyzePocketPlateau(double floor, double rampAngleDegrees,
        double stockAllowance = 0, CancellationToken cancellationToken = default, bool retainBasinDiagnostics = false,
        MillingBoundaryAccess? outsideAccess = null, bool? rampOutsideEdges = null)
    {
        if (!double.IsFinite(rampAngleDegrees) || rampAngleDegrees <= 0 || rampAngleDegrees >= 90)
            throw new ArgumentOutOfRangeException(nameof(rampAngleDegrees));
        if (!double.IsFinite(stockAllowance)) throw new ArgumentOutOfRangeException(nameof(stockAllowance));
        bool ramps = rampOutsideEdges ?? _options.RampOutsideEdges;
        var preparation = outsideAccess != null && !ramps
            ? OpenOutsideRampAnchors(outsideAccess, rampAngleDegrees, cancellationToken) : this;
        if (rampAngleDegrees != preparation._options.SmoothRampAngleDegrees
            || ramps != preparation._options.RampOutsideEdges && preparation._options.AccessTarget != null)
            preparation = preparation.RelimitOutsidePolicy(rampAngleDegrees, ramps, cancellationToken);
        var result = retainBasinDiagnostics ? preparation.AnalyzePlateauRoughing(floor, cancellationToken)
            : preparation.AnalyzeSourcePlateaus(floor, cancellationToken);
        if (stockAllowance >= 0 || result.ProtectedBoundary.Count == 0) return result;
        // Positive allowance is already applied once by the Pocket inset.
        // Negative allowance explicitly permits overlap. Its routing mask moves
        // inward once; existing protection cancels a second negative wall offset.
        double arc = GeometricErrorBound / 4;
        var shifted = PolygonOps.Offset(result.ProtectedBoundary, stockAllowance, arc, 0);
        Log(_options, $"event=plateau-overlap allowance={stockAllowance:R} "
            + $"beforeArea={result.ProtectedArea:R} afterArea={Math.Abs(shifted.Sum(r => Clipper.Area(new PathD(r)))):R} offsetApplications=1");
        return new VCarveRoughingAnalysis(preparation, shifted, result.Basins,
            result.ProtectedSourceRegionIndices, result.EntireProtectedSourceRegionIndices);
    }

    /// <summary>Conservative boundary reach for finishing stock a matching flat rougher cannot enter.
    /// This requests nominations only; every finish retains its own floor, wall and grade guards.</summary>
    public double PocketFinishingReach(double floor, double roughAngleDegrees, double roughRadius, double allowance)
    {
        if (!double.IsFinite(floor) || floor <= 0 || floor > _depth.Depth
            || !double.IsFinite(roughAngleDegrees) || roughAngleDegrees <= 0 || roughAngleDegrees >= 90
            || !double.IsFinite(roughRadius) || roughRadius <= 0 || !double.IsFinite(allowance))
            throw new ArgumentOutOfRangeException(nameof(floor));
        double grade = Math.Tan(Math.Min(roughAngleDegrees, _options.SmoothRampAngleDegrees) * Math.PI / 180);
        var profile = VCarvePhysicalSweep.ProfileFor(_tool, floor);
        double contact = profile.PhysicalRadiusAtDepth(floor);
        // A raised graph point traces to a shallow original anchor. Its graph
        // travel plus remaining grade reach is bounded by floor/grade. The
        // shallow anchor is within the finish contact radius of artwork.
        // Any point beyond another roughRadius+leave admits a rougher centre.
        return contact + SmoothRoughingPlateau.ExclusionRadius(floor, grade, profile)
            + roughRadius + Math.Max(0, allowance) + 2 * GeometricErrorBound;
    }

    private VCarveRoughingAnalysis AnalyzeSourcePlateaus(double floor, CancellationToken token,bool forceComplete=false)
    {
        if (!double.IsFinite(floor) || floor <= 0 || floor > _depth.Depth
            || floor > (_tool.VCarveProfile?.MaximumDepth ?? _tool.ConeLength ?? 0))
            throw new ArgumentOutOfRangeException(nameof(floor));
        double grade = Math.Tan(_options.SmoothRampAngleDegrees * Math.PI / 180);
        var protection = new List<IReadOnlyList<PointD>>();
        var decisions = new List<VCarveBasinDecision>();
        var sources = new HashSet<int>(); var entire = new HashSet<int>();
        bool usedWitness=false;
        foreach (var pendingRegion in Regions)
        {
            token.ThrowIfCancellationRequested();
            bool witnessed=!forceComplete&&pendingRegion.SampledUpperWitness is {} witness&&witness>=floor;
            var region=witnessed?pendingRegion:pendingRegion.Complete(token);
            var boundary = VCarveToolpaths.BuildBoundary(OriginalRegions[region.RegionIndex], 0);
            double peak = witnessed?double.NaN:Peak(region.SmoothedMedialPaths);
            double upper = witnessed?double.NaN:Peak(region.UpperMedialPaths) + grade * (region.MaximumChordLength / 2 + 2 * GeometricErrorBound);
            bool whole = !witnessed&&(region.OriginalMedialPaths.Count == 0 || upper < floor);
            var mask = whole ? boundary : VCarveRampSurface.Plateau(boundary, _options, floor, grade, token);
            mask = RestrictProtectionToTarget(mask);
            double area = Math.Abs(mask.Sum(ring => Clipper.Area(new PathD(ring))));
            if (area > 0) { protection.AddRange(mask); sources.Add(region.RegionIndex); }
            if (whole) entire.Add(region.RegionIndex);
            decisions.Add(new VCarveBasinDecision(region.RegionIndex, 0, default, 0, witnessed?double.NaN:Peak(region.OriginalMedialPaths),
                peak, upper, floor, upper - peak, whole, whole ? "complete-source-below-floor" : "continuous-full-depth-plateau", area, whole));
            usedWitness|=witnessed;
            if(witnessed)Log(_options,$"event=source-plateau region={region.RegionIndex} sampledUpperWitness={region.SampledUpperWitness:R} "
                +$"floor={floor:R} entireSource=False protectedArea={area:R} originalPlateauMask=True "
                +"wholeClassificationExact=True fullPeakStatistics=deferred-until-diagnostic-request originalGraphRetained=True");
            else Log(_options, $"event=source-plateau region={region.RegionIndex} smoothPeak={peak:R} boundedPeak={upper:R} "
                + $"floor={floor:R} entireSource={whole} protectedArea={area:R} watershedRequired=False "
                + "sourceAggregateBoundEquivalent=True graphAndGradeReused=True");
        }
        var canonical = PathUtils.CanonicalizeRings(protection).Select(r => (IReadOnlyList<PointD>)r).ToList();
        return new VCarveRoughingAnalysis(this,
            RestrictProtectionToTarget(canonical), decisions, sources, entire,
            usedWitness?()=>AnalyzeSourcePlateaus(floor,default,forceComplete:true).Basins:null);
    }

    private VCarveRoughingAnalysis AnalyzeRoughingCore(double roughingFloor,
        CancellationToken cancellationToken, bool plateauOnly)
    {
        if (!double.IsFinite(roughingFloor) || roughingFloor <= 0)
            throw new ArgumentOutOfRangeException(nameof(roughingFloor));
        var decisions = new List<VCarveBasinDecision>();
        var protection = new List<IReadOnlyList<PointD>>();
        var protectedSources = new HashSet<int>(); var entireSources = new HashSet<int>();
        double grade = Math.Tan(_options.SmoothRampAngleDegrees * Math.PI / 180);
        double toolDepth = _tool.VCarveProfile?.MaximumDepth ?? _tool.ConeLength ?? 0;
        string? incompatible = _depth.Depth < roughingFloor ? "authored-depth-below-roughing-floor"
            : toolDepth < roughingFloor ? "physical-tool-too-short-for-roughing-floor" : null;
        foreach (PreparedRegion pendingRegion in Regions)
        {
            var region=pendingRegion.Complete(cancellationToken);
            cancellationToken.ThrowIfCancellationRequested();
            Graph graph = BuildGraph(region, cancellationToken);
            if (graph.Nodes.Count == 0)
            {
                bool protectUnknown = plateauOnly && incompatible is null;
                double unknownArea = 0;
                if (protectUnknown)
                {
                    var originalBoundary = RestrictProtectionToTarget(
                        VCarveToolpaths.BuildBoundary(OriginalRegions[region.RegionIndex], 0));
                    protection.AddRange(originalBoundary);
                    protectedSources.Add(region.RegionIndex); entireSources.Add(region.RegionIndex);
                    unknownArea = Math.Abs(originalBoundary.Sum(ring => Clipper.Area(new PathD(ring))));
                    Log(_options, $"event=plateau-omitted region={region.RegionIndex} reason=no-proven-original-graph protectedArea={unknownArea:R}");
                }
                decisions.Add(new VCarveBasinDecision(region.RegionIndex, 0, default, 0, 0, 0,
                    double.PositiveInfinity, roughingFloor, double.PositiveInfinity, protectUnknown,
                    "inconclusive-empty-original-graph", unknownArea, protectUnknown));
                continue;
            }
            int[] labels = Basins(graph, cancellationToken);
            // Each graph node belongs to one basin. Group it once rather than
            // rescanning the complete graph for every local maximum.
            var basinMembers = new Dictionary<int, List<int>>();
            for (int node = 0; node < labels.Length; node++)
            {
                if ((node & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                if (!basinMembers.TryGetValue(labels[node], out var members))
                    basinMembers.Add(labels[node], members = new List<int>());
                members.Add(node);
            }
            int[] basinIds = basinMembers.Keys.OrderBy(id => id).ToArray();
            var qualifying = new HashSet<int>();
            var provisional = new List<VCarveBasinDecision>();
            foreach (int basin in basinIds)
            {
                var members = basinMembers[basin];
                Node peak = graph.Nodes[members.MaxBy(index => graph.Nodes[index].Radius)];
                double originalPeak = members.Max(index => graph.Nodes[index].OriginalDepth);
                double smoothPeak = members.Max(index => graph.Nodes[index].SmoothDepth);
                // Lipschitz continuity bounds the entire unsampled chord, not only its endpoint depths.
                double boundedPeak = members.Max(index => graph.Nodes[index].UpperDepth)
                    + grade * (region.MaximumChordLength / 2 + 2 * GeometricErrorBound);
                double uncertainty = boundedPeak - smoothPeak;
                bool omit = incompatible is null && boundedPeak < roughingFloor;
                string reason = incompatible ?? (omit ? "geometry-limited-smoothed-basin-below-floor"
                    : smoothPeak >= roughingFloor ? "smoothed-basin-reaches-floor" : "floor-comparison-inconclusive");
                if (omit) qualifying.Add(basin);
                provisional.Add(new VCarveBasinDecision(region.RegionIndex, basin, new PointD(peak.X, peak.Y),
                    peak.Radius, originalPeak, smoothPeak, boundedPeak, roughingFloor, uncertainty,
                    omit, reason, 0, false, IsAttachment: basin >= graph.Nodes.Count));
            }
            bool entire = qualifying.Count == basinIds.Length;
            if (!entire)
            {
                // A saddle plateau joins distinct surviving maxima; it is a throat/attachment,
                // not part of either terminal basin. Keep its stock and disc priority whenever
                // the complete source cannot be protected. No authored length or size cutoff.
                for (int feature = 0; feature < provisional.Count; feature++)
                {
                    var attachment = provisional[feature];
                    if (!attachment.IsAttachment) continue;
                    qualifying.Remove(attachment.BasinIndex);
                    provisional[feature] = attachment with
                    {
                        OmitRoughing = false,
                        Reason = incompatible ?? (attachment.SmoothedPeakDepth >= roughingFloor
                            ? "floor-reaching-neck-attachment"
                            : attachment.BoundedMaximumDepth >= roughingFloor
                                ? "neck-attachment-floor-comparison-inconclusive"
                                : "neck-attachment-retained-with-ocean"),
                    };
                }
            }
            List<IReadOnlyList<PointD>> regionProtection;
            if (entire)
            {
                // Preserve the whole isolated authored source, including its acute corners and holes.
                regionProtection = VCarveToolpaths.BuildBoundary(OriginalRegions[region.RegionIndex], 0);
                entireSources.Add(region.RegionIndex);
            }
            else if (plateauOnly && incompatible is null)
            {
                regionProtection = VCarveRampSurface.Plateau(
                    VCarveToolpaths.BuildBoundary(OriginalRegions[region.RegionIndex], 0),
                    _options, roughingFloor, grade, cancellationToken);
            }
            else if (qualifying.Count > 0)
            {
                // Retained/ocean disc unions take precedence in overlaps at the basin mouth.
                var bay = DiscUnion(graph, labels, qualifying, cancellationToken);
                var retained = DiscUnion(graph, labels, basinIds.Where(id => !qualifying.Contains(id)).ToHashSet(), cancellationToken, upperBound: true);
                regionProtection = PolygonOps.Intersect(PolygonOps.Difference(bay, retained),
                    VCarveToolpaths.BuildBoundary(OriginalRegions[region.RegionIndex], 0))
                    .Select(ring => (IReadOnlyList<PointD>)ring).ToList();
            }
            else regionProtection = new();
            regionProtection = RestrictProtectionToTarget(regionProtection);
            double area = Math.Abs(regionProtection.Sum(ring => Clipper.Area(new PathD(ring))));
            if (_options.AccessTarget != null)
                Log(_options,$"event=plateau-target region={region.RegionIndex} branch=basin-diagnostics "
                    + $"protectedArea={area:R} clearanceIsProtectedDemand=False boundedTargetUnion=True");
            if (area > 0) { protection.AddRange(regionProtection); protectedSources.Add(region.RegionIndex); }
            foreach (VCarveBasinDecision decision in provisional)
            {
                var completed = decision with { ProtectedArea = decision.OmitRoughing ? area : 0,
                    EntireSourceRegion = entire && decision.OmitRoughing };
                decisions.Add(completed);
                Log(_options, $"event=decision feature=source-{decision.RegionIndex}/basin-{decision.BasinIndex} " +
                    $"peak=({decision.Peak.x:R},{decision.Peak.y:R}) clearanceRadius={decision.ClearanceRadius:R} " +
                    $"originalPeakDepth={decision.OriginalPeakDepth:R} smoothedPeakDepth={decision.SmoothedPeakDepth:R} " +
                    $"boundedMaximumDepth={decision.BoundedMaximumDepth:R} roughingFloor={roughingFloor:R} uncertainty={decision.Uncertainty:R} " +
                    $"decision={(decision.OmitRoughing ? "omit" : "keep")} reason={decision.Reason} protectedArea={completed.ProtectedArea:R} " +
                    "protectedAreaScope=source-aggregate " +
                    $"attachment={completed.IsAttachment} entireSource={completed.EntireSourceRegion} " +
                    $"effectiveStartingDepth={(decision.OmitRoughing ? 0 : _depth.StartDepth):R}");
            }
        }
        cancellationToken.ThrowIfCancellationRequested();
        var canonical = PathUtils.CanonicalizeRings(protection).Select(ring => (IReadOnlyList<PointD>)ring).ToList();
        return new VCarveRoughingAnalysis(this, RestrictProtectionToTarget(canonical), decisions, protectedSources, entireSources);
    }

    private List<IReadOnlyList<PointD>> RestrictProtectionToTarget(List<IReadOnlyList<PointD>> paths)
        => _options.AccessTarget is { } access && paths.Count>0
            ? PolygonOps.Intersect(paths,access.Target).Select(r=>(IReadOnlyList<PointD>)r).ToList()
            : paths;

    internal bool Matches(IReadOnlyList<MillingRegion> regions, ToolGeometry tool, DepthSchedule depth,
        VCarveOptions options) => Fingerprint == ComputeFingerprint(regions, tool, depth, options);

    private sealed record Node(double X, double Y, double Radius, double OriginalDepth, double SmoothDepth, double UpperDepth)
    {
        internal HashSet<int> Neighbors { get; } = new();
    }
    private sealed record Graph(List<Node> Nodes);

    private static Graph BuildGraph(PreparedRegion region, CancellationToken token)
    {
        var nodes = new List<Node>(); var ids = new Dictionary<(double X, double Y), int>();
        for (int path = 0; path < region.OriginalMedialPaths.Count; path++)
        {
            token.ThrowIfCancellationRequested();
            var points = region.OriginalMedialPaths[path].Points;
            int previous = -1;
            for (int i = 0; i < points.Count; i++)
            {
                Point3D point = points[i];
                if (!ids.TryGetValue((point.X, point.Y), out int id))
                {
                    id = nodes.Count; ids.Add((point.X, point.Y), id);
                    double radius = region.SourceClearancePaths[path][i].Radius;
                    nodes.Add(new Node(point.X, point.Y, radius, -point.Z,
                        -region.SmoothedMedialPaths[path].Points[i].Z, -region.UpperMedialPaths[path].Points[i].Z));
                }
                if (previous >= 0 && previous != id) { nodes[id].Neighbors.Add(previous); nodes[previous].Neighbors.Add(id); }
                previous = id;
            }
        }
        return new Graph(nodes);
    }

    // Watershed ascent assigns corner spokes to their ocean. A superlevel flood supplies saddle
    // prominence for merging numerical/local maxima within the shared geometric error bound.
    private static int[] Basins(Graph graph, CancellationToken token)
    {
        int count = graph.Nodes.Count;
        int[] plateau = Enumerable.Range(0, count).ToArray();
        int Find(int[] sets, int node) { while (sets[node] != node) { sets[node] = sets[sets[node]]; node = sets[node]; } return node; }
        foreach (int node in Enumerable.Range(0, count))
        foreach (int next in graph.Nodes[node].Neighbors)
            if (Math.Abs(graph.Nodes[node].Radius - graph.Nodes[next].Radius) <= 1e-10)
            { int a = Find(plateau, node), b = Find(plateau, next); if (a != b) plateau[Math.Max(a, b)] = Math.Min(a, b); }
        var groups = Enumerable.Range(0, count).GroupBy(node => Find(plateau, node)).ToDictionary(group => group.Key, group => group.ToArray());
        var ascent = new Dictionary<int, int>();
        foreach (var group in groups)
        {
            double radius = group.Value.Max(node => graph.Nodes[node].Radius);
            int best = group.Key; double bestRadius = radius;
            foreach (int next in group.Value.SelectMany(node => graph.Nodes[node].Neighbors))
            {
                double candidate = graph.Nodes[next].Radius;
                if (candidate > bestRadius + 1e-10 || (candidate > radius + 1e-10 && candidate == bestRadius && Find(plateau, next) < best))
                { best = Find(plateau, next); bestRadius = candidate; }
            }
            ascent[group.Key] = best;
        }
        int Summit(int node) { int root = Find(plateau, node); while (ascent[root] != root) root = ascent[root]; return root; }
        int[] labels = Enumerable.Range(0, count).Select(Summit).ToArray();
        Console.WriteLine($"[VCarve.Basins] event=watershed nodes={count} maxima={labels.Distinct().Count()} " +
            $"peaks={string.Join(",", labels.Distinct().Select(index => $"{index}:{graph.Nodes[index].Radius:R}"))}");
        int[] flood = Enumerable.Range(0, count).ToArray(); int[] peak = Enumerable.Range(0, count).ToArray();
        int[] merges = Enumerable.Range(0, count).ToArray(); bool[] active = new bool[count];
        foreach (int node in Enumerable.Range(0, count).OrderByDescending(index => graph.Nodes[index].Radius).ThenBy(index => index))
        {
            token.ThrowIfCancellationRequested(); active[node] = true;
            foreach (int next in graph.Nodes[node].Neighbors)
            {
                if (!active[next]) continue;
                int a = Find(flood, node), b = Find(flood, next); if (a == b) continue;
                int high = peak[a], low = peak[b];
                if (graph.Nodes[low].Radius > graph.Nodes[high].Radius
                    || (graph.Nodes[low].Radius == graph.Nodes[high].Radius && low < high)) (high, low) = (low, high);
                // A newly activated saddle/plateau fragment may ascend to a summit that
                // is not yet connected in this flood component. Its local component maximum
                // is not that basin's peak; measure prominence against the actual summit.
                if (graph.Nodes[labels[low]].Radius - graph.Nodes[node].Radius <= GeometricErrorBound)
                {
                    int lowSummit = Find(merges, labels[low]), highSummit = Find(merges, labels[high]);
                    if (lowSummit != highSummit) merges[lowSummit] = highSummit;
                }
                flood[b] = a; peak[a] = high;
            }
        }
        var survivingSummits = labels.Select(label => Find(merges, label)).ToArray();
        var completed = survivingSummits.ToArray();
        int attachments = 0;
        foreach (var group in groups)
        {
            token.ThrowIfCancellationRequested();
            double radius = graph.Nodes[group.Value[0]].Radius;
            var uphillSummits = group.Value.SelectMany(node => graph.Nodes[node].Neighbors)
                .Where(next => graph.Nodes[next].Radius > radius + 1e-10)
                .Select(next => survivingSummits[next]).Distinct().Take(2).ToArray();
            if (uphillSummits.Length < 2) continue;
            // Evaluate this geometrically identifiable neck independently. Wholesale ascent of
            // its equal-radius plateau would otherwise let a deep ocean throat contaminate a
            // shallow bay's bounded maximum, even for mirrored copies of the same artwork.
            int attachment = count + group.Key;
            foreach (int node in group.Value) completed[node] = attachment;
            attachments++;
        }
        Console.WriteLine($"[VCarve.Basins] event=prominence-complete basins={completed.Distinct().Count()} errorBound={GeometricErrorBound:R}");
        Console.WriteLine($"[VCarve.Basins] event=attachments-excluded attachments={attachments} " +
            $"owner=distinct-surviving-uphill-maxima oceanDiscPriority=True arbitraryCutoff=False");
        return completed;
    }

    private static List<IReadOnlyList<PointD>> DiscUnion(Graph graph, int[] labels, HashSet<int> basins,
        CancellationToken token, bool upperBound = false)
    {
        var discs = new Paths64(); var batches = new Paths64();
        for (int node = 0; node < graph.Nodes.Count; node++)
        {
            if (!basins.Contains(labels[node])) continue;
            token.ThrowIfCancellationRequested(); Node center = graph.Nodes[node];
            double radius = Math.Max(0, center.Radius + (upperBound ? GeometricErrorBound : -GeometricErrorBound));
            if (radius <= 4 / PathUtils.Scale) continue;
            int count = Math.Max(16, (int)Math.Ceiling(Math.PI / Math.Acos(Math.Clamp(1 - GeometricErrorBound / radius, -1, 1))));
            if (upperBound) radius = (radius + 2 / PathUtils.Scale) / Math.Cos(Math.PI / count);
            var disc = new Path64(count);
            for (int i = 0; i < count; i++)
            {
                double angle = 2 * Math.PI * i / count;
                disc.Add(new Point64((long)Math.Round((center.X + radius * Math.Cos(angle)) * PathUtils.Scale),
                    (long)Math.Round((center.Y + radius * Math.Sin(angle)) * PathUtils.Scale)));
            }
            discs.Add(disc);
            if (discs.Count >= 64) { batches.AddRange(Clipper.Union(discs, FillRule.NonZero)); discs.Clear(); }
        }
        batches.AddRange(discs);
        return PathUtils.FromClipper(Clipper.Union(batches, FillRule.NonZero)).Select(ring => (IReadOnlyList<PointD>)ring).ToList();
    }

    internal static string ComputeFingerprint(IReadOnlyList<MillingRegion> regions, ToolGeometry tool,
        DepthSchedule depth, VCarveOptions options)
    {
        var builder = new StringBuilder();
        builder.Append("continuous-retained-crest-basins-forward-owned-roughing-finish-v6|");
        void Value(double value) => builder.Append(value.ToString("R", CultureInfo.InvariantCulture)).Append('|');
        Value(tool.Radius); Value(tool.BottomRadius ?? -1); Value(tool.TopRadius ?? -1);
        Value(tool.ConeLength ?? -1); Value(tool.TipAngleRadians ?? -1);
        if (tool.VCarveProfile is { } profile)
        {
            Value(profile.MaximumDepth);
            foreach (var section in profile.Sections) { builder.Append(section.Kind).Append('|'); Value(section.BottomRadius); Value(section.TopRadius); Value(section.Height); }
        }
        Value(depth.Depth); Value(depth.DepthPerPass); Value(depth.SurfaceZ); Value(depth.StartDepth); Value(depth.SkipDepthPasses);
        Value(options.SmoothRampAngleDegrees); Value(options.StockToLeave); Value(options.StepOver); Value(options.Tolerance);
        Value(options.PreparationContactRadius);
        Value(options.EdgeFillWidth); Value(options.Ramp.AngleRadians);
        builder.Append(options.IncludeInteriorFill).Append('|').Append(options.SmoothVCarving).Append('|');
        builder.Append(options.RampOutsideEdges).Append('|');
        builder.Append(options.AccessTarget?.Signature).Append('|');
        int plateauLeafVertices=SmoothRoughingPlateau.CaptureConstantOffsetLeafVertices(options);
        if(plateauLeafVertices!=SmoothRoughingPlateau.LegacyConstantOffsetLeafVertices)
            builder.Append("plateau-constant-offset-leaf-vertices-v1:").Append(plateauLeafVertices).Append('|');
        foreach (var region in regions)
        {
            builder.Append("region|");
            foreach (var ring in region.AsRingGroup())
            { builder.Append("ring|"); foreach (var point in ring) { Value(point.x); Value(point.y); } }
        }
        return Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(builder.ToString())));
    }

    private static string ComputeOriginalGeometryFingerprint(IReadOnlyList<MillingRegion> regions)
    {
        var builder = new StringBuilder("original-source-region-mapping-v1|");
        builder.Append(regions.Count).Append('|');
        foreach (var region in regions)
        {
            var rings = region.AsRingGroup();
            builder.Append("region|").Append(rings.Count).Append('|');
            foreach (var ring in rings)
            {
                builder.Append("ring|").Append(ring.Count).Append('|');
                foreach (var point in ring)
                    builder.Append(point.x.ToString("R", CultureInfo.InvariantCulture)).Append('|')
                        .Append(point.y.ToString("R", CultureInfo.InvariantCulture)).Append('|');
            }
        }
        return Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(builder.ToString())));
    }

    private static double Distance(MedialPoint a, MedialPoint b) => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
    private static double Peak(IReadOnlyList<TaggedToolpath> paths) => paths.SelectMany(path => path.Points).Select(point => -point.Z).DefaultIfEmpty(0).Max();
    internal static void Log(VCarveOptions options, string message) => Console.WriteLine($"[VCarve.Coordination] operation='{options.DiagnosticContext}' {message}");
}
