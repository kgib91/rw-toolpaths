using System.Security.Cryptography;
using System.Text;
using System.Diagnostics;
using System.Runtime.ExceptionServices;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
namespace RW.Toolpaths.Milling;

/// <summary>Real native primitive and exact source/motion ownership. A fit is a
/// design-fit candidate. Machine posting remains a separate capability/rounding check.</summary>
public sealed record VCarveNativeShortFeature(int First,int Last);
public sealed record VCarveNativePrimitive(ToolMove Move,int SourceFirst,int SourceLast,
    double LengthLowerMm,double LengthUpperMm,ToolpathSpanKind Kind)
{
    public VCarveNativeShortFeature? ShortSourceFeature {get;init;}
    public string? SourceBranchIdentity {get;init;}
    public int BranchPrimitiveIndex {get;init;}
    public int BranchPrimitiveCount {get;init;}=1;
}
public sealed record VCarveNativeCurvePlan(int RegionIndex,int? VisitIdentity,int SourceSegments,
    IReadOnlyList<VCarveNativePrimitive> Primitives,bool NewCurveChecksPassed,bool BudgetExhausted,string Status)
{
    public bool CandidateVerified => NewCurveChecksPassed && !BudgetExhausted && Primitives.All(p=>p.LengthLowerMm>=MinimumPrimitiveLengthMm||p.ShortSourceFeature is not null);
    public IReadOnlyDictionary<string,int> SearchRejections { get; init; } = new Dictionary<string,int>();
    public bool MaterialRemovalCoverageCertified => false;
    public double MinimumPrimitiveLengthMm { get; init; } = .3;
    public int StockContactEnvelopeCurves {get;init;}
    public double MaximumSourceAxialDirectionChangeDegrees {get;init;}
    public int BelowMinimum => Primitives.Count(p=>p.LengthUpperMm<MinimumPrimitiveLengthMm);
}
public sealed record VCarveNativeFitLimits(int MaximumCandidateEdges=50_000,int MaximumBlockPoints=2048,int MaximumSourcePoints=200_000);
internal sealed class VCarveNativeBudgetException:Exception { }
internal sealed class VCarveNativeSearchBudget(CancellationToken token,int maximum)
{
    private readonly object gate=new();
    private int _attempts,_fastProbeAttempts;
    internal int Attempts=>Volatile.Read(ref _attempts);
    internal int Remaining=>Math.Max(0,maximum-Attempts);
    internal int FastProbeAttempts=>Volatile.Read(ref _fastProbeAttempts);
    internal int FastProbeLimit=>Math.Max(0,maximum/20);
    internal void CheckCandidate()
    {
        token.ThrowIfCancellationRequested();
        lock(gate)
        {
            if(_attempts>=maximum)throw new VCarveNativeBudgetException();
            _attempts++;
        }
    }
    // Optional nominations consume the SAME operation budget, never free work.
    // Reserve the last half for ordinary construction/fallback and bound all
    // fast probes across this shared owner to five percent of its inventory.
    internal bool TryCheckFastCandidate()
    {
        token.ThrowIfCancellationRequested();
        lock(gate)
        {
            if(_fastProbeAttempts>=FastProbeLimit||maximum-_attempts<=Math.Max(1,maximum/2))return false;
            _attempts++;_fastProbeAttempts++;return true;
        }
    }
}
/// <summary>Selected fitting policy; independent of primitive length and the
/// operation configuration. Inconclusive design candidates are diagnostic evidence only;
/// unsupported machine consumers reject typed native IL rather than retessellate it.</summary>
public enum VCarveNativeCandidateSelection
{
    MinimumError,
    FirstVerified
}
public sealed record VCarveNativeFitSettings(double FitErrorMm=.3,double MinimumPrimitiveLengthMm=.3)
{
 internal bool PreservePlanarSourceProgress {get;init;}
 internal VCarveNativeGeometricProof.OwnedMinimaxSource? OwnedMinimaxSource {get;init;}
 // Proof-storage custody is implementation metadata, not a semantic fitting
 // setting. Keep the record's prior five-field equality/hash contract.
 public bool Equals(VCarveNativeFitSettings? other)=>other is not null
  &&FitErrorMm.Equals(other.FitErrorMm)&&MinimumPrimitiveLengthMm.Equals(other.MinimumPrimitiveLengthMm)
  &&PreservePlanarSourceProgress==other.PreservePlanarSourceProgress&&CandidateSelection==other.CandidateSelection
  &&MaximumCandidateGeometryProofs==other.MaximumCandidateGeometryProofs;
 public override int GetHashCode()
 {
  unchecked
  {
   int hash=EqualityComparer<Type>.Default.GetHashCode(typeof(VCarveNativeFitSettings));
   hash=hash*-1521134295+EqualityComparer<double>.Default.GetHashCode(FitErrorMm);
   hash=hash*-1521134295+EqualityComparer<double>.Default.GetHashCode(MinimumPrimitiveLengthMm);
   hash=hash*-1521134295+EqualityComparer<bool>.Default.GetHashCode(PreservePlanarSourceProgress);
   hash=hash*-1521134295+EqualityComparer<VCarveNativeCandidateSelection>.Default.GetHashCode(CandidateSelection);
   return hash*-1521134295+EqualityComparer<int>.Default.GetHashCode(MaximumCandidateGeometryProofs);
  }
 }
 public VCarveNativeCandidateSelection CandidateSelection {get;init;}=VCarveNativeCandidateSelection.MinimumError;
 /// <summary>Full source-geometry attempts per edge Find in FirstVerified mode.
 /// Includes the straight-line probe; cheap nomination refusals do not consume
 /// it. Reaching the limit requests ordinary subdivision/fallback, never weaker
 /// acceptance. MinimumError ignores this effort bound.</summary>
 public int MaximumCandidateGeometryProofs {get;init;}=int.MaxValue;
 public void Validate(){if(!double.IsFinite(FitErrorMm)||FitErrorMm<=0||!double.IsFinite(MinimumPrimitiveLengthMm)||MinimumPrimitiveLengthMm<=0)throw new ArgumentOutOfRangeException(nameof(FitErrorMm));if(!Enum.IsDefined(CandidateSelection))throw new ArgumentOutOfRangeException(nameof(CandidateSelection));if(MaximumCandidateGeometryProofs<=0)throw new ArgumentOutOfRangeException(nameof(MaximumCandidateGeometryProofs));}
}
public static partial class VCarveNativeCurvePlanning
{
    public static IReadOnlyList<IReadOnlyList<PointD>> PrepareBoundary(MillingRegion region,double stockToLeave)
        => VCarveToolpaths.BuildBoundary(region,stockToLeave);
    // Optional read-only evidence snapshot for diagnostic harnesses. Async-local;
    // unset in the client. Never changes fitting or grants a safety certificate.
    public static readonly AsyncLocal<Action<IReadOnlyList<TaggedToolpath>,IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>>,VCarveCutterProfile,double>?> EvidenceObserver=new();
    // Diagnostic candidates only; snapshots cannot mutate the returned geometry.
    public static readonly AsyncLocal<Action<IReadOnlyList<VCarveNativeCurvePlan>>?> CandidateObserver=new();
    // Immutable per-path diagnostic checkpoint; unset in normal client use.
    public static readonly AsyncLocal<Action<int,VCarveNativeCurvePlan>?> RegionObserver=new();
    // An isolated serial control for the explicit query-worker path. Opaque
    // callback owners never opt into parallel execution through this setting.
    internal static readonly AsyncLocal<int?> RegionWorkerLimitOverride=new();
    // Control for the original region dispatcher. This only selects scheduling;
    // opaque callbacks and the literal one-worker control remain sequential.
    internal static readonly AsyncLocal<bool?> MandatoryIntervalWorkersEnabled=new();
    public const string Policy="native-geometric-order-clearance-and-short-feature-preservation-v11";
    public const double FitErrorMm=.3;
    public const double MinimumPrimitiveLengthMm=.3;
    public static IReadOnlyList<VCarveNativeCurvePlan> Nominate(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> regionBoundaries,VCarveCutterProfile profile,
        double surfaceZ,CancellationToken token=default,VCarveNativeFitLimits? limits=null,Action<string>? diagnostic=null,VCarveNativeFitSettings? settings=null,
        Func<Point3D,Point3D,bool>? stockChord=null,Func<ToolMove,bool>? stockCurve=null,
        IReadOnlyList<IReadOnlyList<int>>? selectedSourceOrdinals=null,IReadOnlyList<IReadOnlyList<int>>? requiredSourceOrdinals=null,
        VCarveNativeSourceContract sourceContract=VCarveNativeSourceContract.PreparedSpatialDirections,VCarveNativeTopologyContext? canonicalTopology=null,
        IReadOnlyList<VCarveNativeNominationFamily>? nominationAlternatives=null,
        Func<VCarveNativeQueryWorker>? stockQueryWorkerFactory=null)
    {
        using var geometryWork=diagnostic is null?null:VCarveNativeGeometricProof.BeginWorkCapture();
        bool? previousProjectionOrder=VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value;
        // Cheap-first witness order remains an explicit diagnostic experiment;
        // same-binary replay did not establish a benefit over the original order.
        VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value=false;
        bool bodyFailed=false;
        try
        {
        ArgumentNullException.ThrowIfNull(source);
        int workers=Math.Min(6,RegionWorkerLimitOverride.Value??GeometryParallelism.StockFitWorkerLimit);
        if(stockQueryWorkerFactory is not null&&workers>1&&MandatoryIntervalWorkersEnabled.Value is not false)
        {
            var intervals=TryNominateIntervals(source,regionBoundaries,profile,surfaceZ,token,limits,diagnostic,settings,
                stockChord,stockCurve,selectedSourceOrdinals,requiredSourceOrdinals,sourceContract,canonicalTopology,
                nominationAlternatives,stockQueryWorkerFactory,workers);
            if(intervals is not null)return intervals;
        }
        if(stockQueryWorkerFactory is null||workers<=1||source.Select(p=>p.RegionIndex).Distinct().Take(2).Count()<2)
            return NominateCore(source,regionBoundaries,profile,surfaceZ,token,limits,diagnostic,settings,
                stockChord,stockCurve,selectedSourceOrdinals,requiredSourceOrdinals,sourceContract,canonicalTopology,nominationAlternatives,null,null);
        return NominateRegions(source,regionBoundaries,profile,surfaceZ,token,limits,diagnostic,settings,
            stockChord,stockCurve,selectedSourceOrdinals,requiredSourceOrdinals,sourceContract,canonicalTopology,
            nominationAlternatives,stockQueryWorkerFactory,workers);
        }
        catch{bodyFailed=true;throw;}
        finally
        {
            VCarveNativeGeometricProof.ScopedCheapProjectionsFirst.Value=previousProjectionOrder;
            try
            {
                if(geometryWork is not null)diagnostic?.Invoke("[VCarve.NativeGeometryWork] "+geometryWork.Snapshot().Describe()
                    +" regularityAndContinuousBernsteinProofsUnchanged=True defaultWitnessOrder=original cheapFirst=explicit-diagnostic-only");
            }
            catch when(bodyFailed){/* Preserve the original failure or cancellation. */}
        }
    }

    private readonly record struct IntervalOccurrence(int Visit,int Block);
    private sealed record IntervalWorkGroup(string GeometryKey,List<IntervalOccurrence> Occurrences);

    private static IReadOnlyList<VCarveNativeCurvePlan>? TryNominateIntervals(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> regionBoundaries,VCarveCutterProfile profile,
        double surfaceZ,CancellationToken token,VCarveNativeFitLimits? limits,Action<string>? diagnostic,VCarveNativeFitSettings? settings,
        Func<Point3D,Point3D,bool>? stockChord,Func<ToolMove,bool>? stockCurve,
        IReadOnlyList<IReadOnlyList<int>>? selectedSourceOrdinals,IReadOnlyList<IReadOnlyList<int>>? requiredSourceOrdinals,
        VCarveNativeSourceContract sourceContract,VCarveNativeTopologyContext? canonicalTopology,
        IReadOnlyList<VCarveNativeNominationFamily>? nominationAlternatives,
        Func<VCarveNativeQueryWorker> workerFactory,int maximumWorkers)
    {
        (settings,limits)=ValidateInvocation(source,regionBoundaries,profile,surfaceZ,limits,settings,
            stockChord,stockCurve,selectedSourceOrdinals,requiredSourceOrdinals,sourceContract,nominationAlternatives);
        if(stockChord is null||stockCurve is null)
            throw new ArgumentException("Native query workers require the complete original chord and curve authority.");
        token.ThrowIfCancellationRequested();
        var topology=canonicalTopology??VCarveNativeTopologyContext.Create(source,token);
        if(source.Sum(p=>(long)p.Points.Count)>limits.MaximumSourcePoints||topology.SourcePointCount>limits.MaximumSourcePoints)return null;
        var visits=new PreparedNativeVisit?[source.Count];int maximumIntervals=0;
        for(int i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();var path=source[i];
            var p=path.Points.Select(q=>q with{Z=q.Z-surfaceZ}).ToArray();
            if(p.Any(q=>!double.IsFinite(q.X)||!double.IsFinite(q.Y)||!double.IsFinite(q.Z)))
                throw new ArgumentException("Native source coordinates must be finite.");
            if(p.Length<2||path.RegionIndex<0||path.RegionIndex>=regionBoundaries.Count||path.Moves.Any(m=>m.Kind!=ToolMoveKind.Line))continue;
            var visit=PrepareVisit(path,p,topology,requiredSourceOrdinals?[i],settings);visits[i]=visit;
            if(!visit.PreservedClosedFeature)maximumIntervals=Math.Max(maximumIntervals,visit.Anchors.Length-1);
        }
        // Small visits retain the already-tested region scheduler and avoid
        // manufacturing a large private-owner inventory for trivial fits.
        if(maximumIntervals<4)return null;
        var groups=new List<IntervalWorkGroup>();var groupByGeometry=new Dictionary<string,IntervalWorkGroup>();
        for(int i=0;i<source.Count;i++)
        {
            if(visits[i] is not {} visit||visit.PreservedClosedFeature)
            {Add($"whole-visit:{source[i].RegionIndex}",new(i,0));continue;}
            for(int h=1;h<visit.Anchors.Length;h++)
            {
                token.ThrowIfCancellationRequested();Add(IntervalGeometryKey(visit,h),new(i,h));
            }
        }
        void Add(string key,IntervalOccurrence occurrence)
        {
            if(!groupByGeometry.TryGetValue(key,out var group))
            {group=new(key,[]);groupByGeometry.Add(key,group);groups.Add(group);}
            group.Occurrences.Add(occurrence);
        }
        if(groups.Count<2)return null;
        EvidenceObserver.Value?.Invoke(source.Select(p=>new TaggedToolpath(p.Points.ToArray(),p.Spans.ToArray(),
            p.RegionIndex,p.Category,p.DepthPassIndex,p.VisitIdentity,detectCurves:false)).ToArray(),regionBoundaries,profile,surfaceZ);
        int workerCount=Math.Min(maximumWorkers,groups.Count);var workers=new VCarveNativeQueryWorker[workerCount];
        long started=Stopwatch.GetTimestamp();
        for(int i=0;i<workers.Length;i++)
        {
            token.ThrowIfCancellationRequested();workers[i]=workerFactory();
            if(workers[i] is null||workers[i].Chord is null||workers[i].Curve is null)
                throw new ArgumentException("Native query worker factory returned an incomplete authority pair.");
            diagnostic?.Invoke($"[VCarve.NativeIntervals] event=worker worker={i} provenance={workers[i].ImmutableDomainProvenance} "
                +$"importedPositiveSweepProofs={workers[i].ImportedPositiveSweepProofs} importedPositivePoseProofs={workers[i].ImportedPositivePoseProofs} "
                +$"importedPositiveLayerPoses={workers[i].ImportedPositiveLayerPoses} estimatedImportedProofBytes={workers[i].EstimatedImportedProofBytes} "
                +$"importedStockCellFacts={workers[i].ImportedStockCellFacts} estimatedImportedStockCellBytes={workers[i].EstimatedImportedStockCellBytes} "
                +$"startupMs={workers[i].StartupMilliseconds:F3} creation=sequential originalOwnerQuiescent=True mutableHintsShared=False");
        }
        var solved=visits.Select(v=>v is null?Array.Empty<NativeIntervalResult?>():new NativeIntervalResult?[v.Anchors.Length-1]).ToArray();
        var wholePlans=new VCarveNativeCurvePlan?[source.Count];
        var messages=visits.Select(v=>Enumerable.Range(0,v is null||v.PreservedClosedFeature?1:v.Anchors.Length-1)
            .Select(_=>new List<string>()).ToArray()).ToArray();var errors=new ExceptionDispatchInfo?[groups.Count];
        var elapsed=new double[groups.Count];var assigned=new int[groups.Count];
        var groupStarted=new bool[groups.Count];
        var cacheEntries=new int[workerCount];var budget=new VCarveNativeSearchBudget(token,limits.MaximumCandidateEdges);
        int next=-1,failed=0;
        diagnostic?.Invoke($"[VCarve.NativeIntervals] event=start workers={workerCount} groups={groups.Count} visits={source.Count} "
            +$"intervals={groups.Sum(g=>g.Occurrences.Count)} maximumVisitIntervals={maximumIntervals} sourcePoints={topology.SourcePointCount} "
            +$"candidateLimit={limits.MaximumCandidateEdges} factoryMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} "
            +"fullVisitTopologyAndMaps=True sharedOperationBudget=True geometryGroupsOmitKnots=True exactFitCacheKeysRetainKnots=True nestedGeometryWorkers=1 sourceProjectionParallel=False");
        Parallel.For(0,workerCount,new ParallelOptions{MaxDegreeOfParallelism=workerCount},workerIndex=>
        {
            using var geometryWorker=GeometryParallelism.EnterWorker();
            var evidenceObserver=EvidenceObserver.Value;var candidateObserver=CandidateObserver.Value;var regionObserver=RegionObserver.Value;
            var cache=new Dictionary<string,List<VCarveNativeFit>>();var guards=new Dictionary<int,VCarveNativeContainment>();bool exhausted=false;
            try
            {
                EvidenceObserver.Value=null;CandidateObserver.Value=null;RegionObserver.Value=null;
                while(!token.IsCancellationRequested&&Volatile.Read(ref failed)==0)
                {
                    int job=Interlocked.Increment(ref next);if(job>=groups.Count)break;
                    assigned[job]=workerIndex;groupStarted[job]=true;long jobStarted=Stopwatch.GetTimestamp();
                    try
                    {
                        foreach(var occurrence in groups[job].Occurrences)
                        {
                            token.ThrowIfCancellationRequested();int index=occurrence.Visit;
                            var trace=messages[index][Math.Max(0,occurrence.Block-1)];
                            long intervalStarted=Stopwatch.GetTimestamp();int previousCacheEntries=cache.Count;
                            if(occurrence.Block==0)
                            {
                                IReadOnlyList<IReadOnlyList<int>>? Subset(IReadOnlyList<IReadOnlyList<int>>? maps)=>maps is null?null:[maps[index]];
                                var alternatives=nominationAlternatives?.Select(f=>new VCarveNativeNominationFamily(f.Name,[f.SelectedSourceOrdinals[index]])).ToArray();
                                wholePlans[index]=NominateCore([source[index]],regionBoundaries,profile,surfaceZ,token,limits,trace.Add,settings,
                                    workers[workerIndex].Chord,workers[workerIndex].Curve,Subset(selectedSourceOrdinals),Subset(requiredSourceOrdinals),
                                    sourceContract,topology,alternatives,budget,cache).Single();
                                exhausted|=wholePlans[index]!.BudgetExhausted;continue;
                            }
                            var visit=visits[index]!;int region=visit.Path.RegionIndex;
                            if(!guards.TryGetValue(region,out var guard))guards[region]=guard=new(regionBoundaries[region],profile,token,
                                workers[workerIndex].Chord,workers[workerIndex].Curve);
                            var interval=SolveInterval(visit,occurrence.Block,index,regionBoundaries,profile,surfaceZ,token,limits,trace.Add,settings,
                                workers[workerIndex].Chord,workers[workerIndex].Curve,selectedSourceOrdinals,sourceContract,topology,
                                nominationAlternatives,budget,cache,guard,exhausted);
                            solved[index][occurrence.Block-1]=interval;exhausted|=interval.BudgetExhausted;
                            trace.Add($"[VCarve.NativeIntervals] event=interval visitIndex={index} region={region} block={occurrence.Block}/{visit.Anchors.Length-1} "
                                +$"first={visit.Anchors[occurrence.Block-1]} last={visit.Anchors[occurrence.Block]} worker={workerIndex} "
                                +$"cacheEntriesBefore={previousCacheEntries} cacheEntriesAfter={cache.Count} elapsedMs={Stopwatch.GetElapsedTime(intervalStarted).TotalMilliseconds:F3} "
                                +$"completionProcessor={Thread.GetCurrentProcessorId()} "
                                +"fullVisitContext=True exactFitCacheKeysRetainKnots=True");
                        }
                    }
                    catch(Exception error){errors[job]=ExceptionDispatchInfo.Capture(error);Interlocked.Exchange(ref failed,1);}
                    finally{elapsed[job]=Stopwatch.GetElapsedTime(jobStarted).TotalMilliseconds;}
                }
            }
            finally
            {cacheEntries[workerIndex]=cache.Count;EvidenceObserver.Value=evidenceObserver;CandidateObserver.Value=candidateObserver;RegionObserver.Value=regionObserver;}
        });
        // Worker diagnostics are replayed in the original visit/interval order,
        // including a refused or canceled run, only after every query has joined.
        foreach(var visitMessages in messages)foreach(var intervalMessages in visitMessages)
            foreach(var message in intervalMessages)diagnostic?.Invoke(message);
        for(int job=0;job<groups.Count;job++)
            diagnostic?.Invoke($"[VCarve.NativeIntervals] event=group group={job} worker={assigned[job]} occurrences={groups[job].Occurrences.Count} "
                +$"geometry={groups[job].GeometryKey} elapsedMs={elapsed[job]:F3} started={groupStarted[job]} completed={groupStarted[job]&&errors[job] is null} "
                +"fullVisitContext=True diagnosticsReplayedAfterJoin=True");
        token.ThrowIfCancellationRequested();foreach(var error in errors)error?.Throw();
        var result=new VCarveNativeCurvePlan[source.Count];var finalGuards=new Dictionary<int,VCarveNativeContainment>();
        for(int i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();
            if(wholePlans[i] is {} whole){result[i]=whole;continue;}
            var visit=visits[i]!;var output=new List<VCarveNativePrimitive>();var rejections=new Dictionary<string,int>();
            int tangent=0,contacts=0;double axial=0;bool exhausted=false;
            foreach(var interval in solved[i])
            {
                if(interval is null)throw new InvalidOperationException("Native mandatory interval was not completed before ordered publication.");
                output.AddRange(interval.Primitives);foreach(var pair in interval.Rejections)rejections[pair.Key]=rejections.GetValueOrDefault(pair.Key)+pair.Value;
                tangent+=interval.BoundaryTangentMismatches;contacts+=interval.ContactCurves;axial=Math.Max(axial,interval.MaximumAxialChange);exhausted|=interval.BudgetExhausted;
            }
            int region=visit.Path.RegionIndex;
            if(!finalGuards.TryGetValue(region,out var guard))finalGuards[region]=guard=new(regionBoundaries[region],profile,token,workers[0].Chord,workers[0].Curve);
            // Original complete-visit retained-line, planar-progress and
            // protected-tangent audits run in source order after ALL workers join.
            result[i]=FinalizeVisit(visit,output,rejections,tangent,contacts,axial,exhausted,false,sourceContract,guard,surfaceZ,settings,diagnostic);
        }
        for(int i=0;i<result.Length;i++)
        {token.ThrowIfCancellationRequested();RegionObserver.Value?.Invoke(i,Snapshot(result[i]));token.ThrowIfCancellationRequested();}
        CandidateObserver.Value?.Invoke(result.Select(Snapshot).ToArray());token.ThrowIfCancellationRequested();
        for(int i=0;i<workers.Length;i++)if(workers[i].DiagnosticCounters is {} counters)
            diagnostic?.Invoke($"[VCarve.NativeIntervals] event=worker-query-counters worker={i} {counters()} allWorkersJoined=True");
        diagnostic?.Invoke($"[VCarve.NativeIntervals] event=complete workers={workerCount} groups={groups.Count} visits={source.Count} "
            +$"candidateAttempts={budget.Attempts} candidateLimit={limits.MaximumCandidateEdges} fastProbeAttempts={budget.FastProbeAttempts} fastProbeLimit={budget.FastProbeLimit} "
            +$"privateFitCacheEntries={cacheEntries.Sum()} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} "
            +"stableSourceOrder=True allWorkersJoined=True fullSourceAndGlobalTopology=True opaqueCallbacksParallel=False finalVisitAuditsSerial=True");
        token.ThrowIfCancellationRequested();return result;
    }

    private static string IntervalGeometryKey(PreparedNativeVisit visit,int h)
    {
        int first=visit.Anchors[h-1],last=visit.Anchors[h];bool reverse=false;var p=visit.Points;
        for(int i=0;i<=last-first;i++){int cmp=Compare(p[first+i],p[last-i]);if(cmp!=0){reverse=cmp>0;break;}}
        using var bytes=new MemoryStream();using(var writer=new BinaryWriter(bytes,Encoding.UTF8,true))
        {
            writer.Write(visit.Path.RegionIndex);var kind=visit.Kinds[first];
            writer.Write((int)(kind is ToolpathSpanKind.Cut or ToolpathSpanKind.Link?ToolpathSpanKind.Cut:kind));
            for(int i=0;i<=last-first;i++){var q=p[reverse?last-i:first+i];writer.Write(q.X);writer.Write(q.Y);writer.Write(q.Z);}
        }
        return Convert.ToHexString(SHA256.HashData(bytes.ToArray()));
    }

    private static IReadOnlyList<VCarveNativeCurvePlan> NominateRegions(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> regionBoundaries,VCarveCutterProfile profile,
        double surfaceZ,CancellationToken token,VCarveNativeFitLimits? limits,Action<string>? diagnostic,VCarveNativeFitSettings? settings,
        Func<Point3D,Point3D,bool>? stockChord,Func<ToolMove,bool>? stockCurve,
        IReadOnlyList<IReadOnlyList<int>>? selectedSourceOrdinals,IReadOnlyList<IReadOnlyList<int>>? requiredSourceOrdinals,
        VCarveNativeSourceContract sourceContract,VCarveNativeTopologyContext? canonicalTopology,
        IReadOnlyList<VCarveNativeNominationFamily>? nominationAlternatives,
        Func<VCarveNativeQueryWorker> workerFactory,int maximumWorkers)
    {
        (settings,limits)=ValidateInvocation(source,regionBoundaries,profile,surfaceZ,limits,settings,
            stockChord,stockCurve,selectedSourceOrdinals,requiredSourceOrdinals,sourceContract,nominationAlternatives);
        if(stockChord is null||stockCurve is null)
            throw new ArgumentException("Native query workers require the complete original chord and curve authority.");
        var topology=canonicalTopology??VCarveNativeTopologyContext.Create(source,token);
        // Preserve the operation-wide source budget before any partition or
        // factory side effect. A private region is never a new topology owner.
        if(source.Sum(p=>(long)p.Points.Count)>limits.MaximumSourcePoints||topology.SourcePointCount>limits.MaximumSourcePoints)
            return NominateCore(source,regionBoundaries,profile,surfaceZ,token,limits,diagnostic,settings,
                stockChord,stockCurve,selectedSourceOrdinals,requiredSourceOrdinals,sourceContract,topology,nominationAlternatives,null,null);
        EvidenceObserver.Value?.Invoke(source.Select(p=>new TaggedToolpath(p.Points.ToArray(),p.Spans.ToArray(),
            p.RegionIndex,p.Category,p.DepthPassIndex,p.VisitIdentity,detectCurves:false)).ToArray(),regionBoundaries,profile,surfaceZ);
        var jobs=source.Select((path,index)=>(path.RegionIndex,index)).GroupBy(p=>p.RegionIndex)
            .Select(group=>group.Select(p=>p.index).ToArray()).ToArray();
        int workerCount=Math.Min(maximumWorkers,jobs.Length);
        var workers=new VCarveNativeQueryWorker[workerCount];
        long started=Stopwatch.GetTimestamp();
        // Original owner is quiescent while complete positive proofs are
        // imported. No mutable query memo crosses into a worker callback.
        for(int i=0;i<workers.Length;i++)
        {
            token.ThrowIfCancellationRequested();workers[i]=workerFactory();
            if(workers[i] is null||workers[i].Chord is null||workers[i].Curve is null)
                throw new ArgumentException("Native query worker factory returned an incomplete authority pair.");
            diagnostic?.Invoke($"[VCarve.NativeRegions] event=worker worker={i} provenance={workers[i].ImmutableDomainProvenance} "
                +$"importedPositiveSweepProofs={workers[i].ImportedPositiveSweepProofs} importedPositivePoseProofs={workers[i].ImportedPositivePoseProofs} "
                +$"importedPositiveLayerPoses={workers[i].ImportedPositiveLayerPoses} estimatedImportedProofBytes={workers[i].EstimatedImportedProofBytes} "
                +$"importedStockCellFacts={workers[i].ImportedStockCellFacts} estimatedImportedStockCellBytes={workers[i].EstimatedImportedStockCellBytes} "
                +$"startupMs={workers[i].StartupMilliseconds:F3} creation=sequential originalOwnerQuiescent=True mutableHintsShared=False");
        }
        diagnostic?.Invoke($"[VCarve.NativeRegions] event=start workers={workerCount} regions={jobs.Length} visits={source.Count} "
            +$"sourcePoints={topology.SourcePointCount} candidateLimit={limits.MaximumCandidateEdges} factoryMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} "
            +"globalTopology=True sharedOperationBudget=True privateQueryOwners=True nestedGeometryWorkers=1 sourceProjectionParallel=False");
        var result=new VCarveNativeCurvePlan[source.Count];
        var messages=jobs.Select(_=>new List<string>()).ToArray();
        var errors=new ExceptionDispatchInfo?[jobs.Length];
        var elapsed=new double[jobs.Length];var assigned=new int[jobs.Length];
        var budget=new VCarveNativeSearchBudget(token,limits.MaximumCandidateEdges);
        int next=-1;
        Parallel.For(0,workerCount,new ParallelOptions{MaxDegreeOfParallelism=workerCount},workerIndex=>
        {
            using var geometryWorker=GeometryParallelism.EnterWorker();
            var evidenceObserver=EvidenceObserver.Value;var candidateObserver=CandidateObserver.Value;var regionObserver=RegionObserver.Value;
            try
            {
                EvidenceObserver.Value=null;CandidateObserver.Value=null;RegionObserver.Value=null;
                while(true)
                {
                    int job=Interlocked.Increment(ref next);if(job>=jobs.Length)break;
                    int[] indices=jobs[job];assigned[job]=workerIndex;long jobStarted=Stopwatch.GetTimestamp();
                    try
                    {
                        token.ThrowIfCancellationRequested();
                        var paths=indices.Select(i=>source[i]).ToArray();
                        IReadOnlyList<IReadOnlyList<int>>? Subset(IReadOnlyList<IReadOnlyList<int>>? maps)
                            =>maps is null?null:indices.Select(i=>maps[i]).ToArray();
                        var alternatives=nominationAlternatives?.Select(f=>new VCarveNativeNominationFamily(f.Name,
                            indices.Select(i=>f.SelectedSourceOrdinals[i]).ToArray())).ToArray();
                        var fitted=NominateCore(paths,regionBoundaries,profile,surfaceZ,token,limits,messages[job].Add,settings,
                            workers[workerIndex].Chord,workers[workerIndex].Curve,Subset(selectedSourceOrdinals),Subset(requiredSourceOrdinals),
                            sourceContract,topology,alternatives,budget,new Dictionary<string,List<VCarveNativeFit>>());
                        for(int i=0;i<indices.Length;i++)result[indices[i]]=fitted[i];
                    }
                    catch(Exception error){errors[job]=ExceptionDispatchInfo.Capture(error);}
                    finally{elapsed[job]=Stopwatch.GetElapsedTime(jobStarted).TotalMilliseconds;}
                }
            }
            finally
            {
                EvidenceObserver.Value=evidenceObserver;CandidateObserver.Value=candidateObserver;RegionObserver.Value=regionObserver;
            }
        });
        // All private queries have joined before any external callback or
        // original-owner branch recheck. Preserve original region/path order.
        for(int job=0;job<jobs.Length;job++)
        {
            foreach(string message in messages[job])diagnostic?.Invoke(message);
            diagnostic?.Invoke($"[VCarve.NativeRegions] event=region region={source[jobs[job][0]].RegionIndex} worker={assigned[job]} "
                +$"visits={jobs[job].Length} elapsedMs={elapsed[job]:F3} completed={errors[job] is null} diagnosticsReplayedAfterJoin=True");
        }
        token.ThrowIfCancellationRequested();
        foreach(var error in errors)error?.Throw();
        for(int i=0;i<result.Length;i++)
        {
            token.ThrowIfCancellationRequested();RegionObserver.Value?.Invoke(i,Snapshot(result[i]));
            token.ThrowIfCancellationRequested();
        }
        CandidateObserver.Value?.Invoke(result.Select(Snapshot).ToArray());
        token.ThrowIfCancellationRequested();
        for(int i=0;i<workers.Length;i++)if(workers[i].DiagnosticCounters is {} counters)
            diagnostic?.Invoke($"[VCarve.NativeRegions] event=worker-query-counters worker={i} {counters()} allWorkersJoined=True");
        diagnostic?.Invoke($"[VCarve.NativeRegions] event=complete workers={workerCount} regions={jobs.Length} visits={source.Count} "
            +$"candidateAttempts={budget.Attempts} candidateLimit={limits.MaximumCandidateEdges} fastProbeAttempts={budget.FastProbeAttempts} fastProbeLimit={budget.FastProbeLimit} "
            +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} stableSourceOrder=True allWorkersJoined=True opaqueCallbacksParallel=False");
        token.ThrowIfCancellationRequested();
        return result;
    }

    private static VCarveNativeCurvePlan Snapshot(VCarveNativeCurvePlan plan)=>plan with
    {Primitives=plan.Primitives.ToArray(),SearchRejections=new System.Collections.ObjectModel.ReadOnlyDictionary<string,int>(plan.SearchRejections.ToDictionary(x=>x.Key,x=>x.Value))};

    private static bool ValidSourceMap(IReadOnlyList<int> map,int pointCount)
        =>pointCount==1?map.Count==1&&map[0]==0
            :map.Count>=2&&map[0]==0&&map[^1]==pointCount-1
                &&!map.Where((n,i)=>n<0||n>=pointCount||i>0&&n<=map[i-1]).Any();

    private static (VCarveNativeFitSettings Settings,VCarveNativeFitLimits Limits) ValidateInvocation(
        IReadOnlyList<TaggedToolpath> source,IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> regionBoundaries,VCarveCutterProfile profile,
        double surfaceZ,VCarveNativeFitLimits? limits,VCarveNativeFitSettings? settings,Func<Point3D,Point3D,bool>? stockChord,Func<ToolMove,bool>? stockCurve,
        IReadOnlyList<IReadOnlyList<int>>? selectedSourceOrdinals,IReadOnlyList<IReadOnlyList<int>>? requiredSourceOrdinals,
        VCarveNativeSourceContract sourceContract,IReadOnlyList<VCarveNativeNominationFamily>? nominationAlternatives)
    {
        ArgumentNullException.ThrowIfNull(source);ArgumentNullException.ThrowIfNull(regionBoundaries);ArgumentNullException.ThrowIfNull(profile);
        if(!double.IsFinite(surfaceZ))throw new ArgumentOutOfRangeException(nameof(surfaceZ));
        if(selectedSourceOrdinals is not null&&selectedSourceOrdinals.Count!=source.Count||requiredSourceOrdinals is not null&&requiredSourceOrdinals.Count!=source.Count)
            throw new ArgumentException("Native source-map visit count does not match immutable source.");
        for(int index=0;index<source.Count;index++)
        foreach(var map in new[]{selectedSourceOrdinals?[index],requiredSourceOrdinals?[index]})
            if(map is not null && !ValidSourceMap(map,source[index].Points.Count))
                throw new ArgumentException("Native source maps must contain strictly ordered original ordinals including both endpoints.");
        if(!Enum.IsDefined(sourceContract))throw new ArgumentOutOfRangeException(nameof(sourceContract));
        if(sourceContract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth&&(stockChord is null||stockCurve is null))
            throw new InvalidOperationException("STOCK_CONTACT_DIRECTION_REQUIRES_ACTUAL_STOCK");
        settings=(settings??new()) with{PreservePlanarSourceProgress=sourceContract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth};settings.Validate();
        limits??=new();if(limits.MaximumCandidateEdges<0||limits.MaximumBlockPoints<2||limits.MaximumSourcePoints<2)throw new ArgumentOutOfRangeException(nameof(limits));
        if(nominationAlternatives is {Count:>0}&&(sourceContract!=VCarveNativeSourceContract.MedialPlanarWithStockContactDepth
            ||nominationAlternatives.Any(f=>f.SelectedSourceOrdinals.Count!=source.Count)))
            throw new ArgumentException("Alternative nominations require the complete stock-contact source map.");
        return(settings,limits);
    }

    private static IReadOnlyList<VCarveNativeCurvePlan> NominateCore(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> regionBoundaries,VCarveCutterProfile profile,
        double surfaceZ,CancellationToken token,VCarveNativeFitLimits? limits,Action<string>? diagnostic,VCarveNativeFitSettings? settings,
        Func<Point3D,Point3D,bool>? stockChord,Func<ToolMove,bool>? stockCurve,
        IReadOnlyList<IReadOnlyList<int>>? selectedSourceOrdinals,IReadOnlyList<IReadOnlyList<int>>? requiredSourceOrdinals,
        VCarveNativeSourceContract sourceContract,VCarveNativeTopologyContext? canonicalTopology,
        IReadOnlyList<VCarveNativeNominationFamily>? nominationAlternatives,VCarveNativeSearchBudget? sharedBudget,
        Dictionary<string,List<VCarveNativeFit>>? sharedCache)
    {
        ArgumentNullException.ThrowIfNull(source);ArgumentNullException.ThrowIfNull(regionBoundaries);ArgumentNullException.ThrowIfNull(profile);
        if(!double.IsFinite(surfaceZ))throw new ArgumentOutOfRangeException(nameof(surfaceZ));
        if(selectedSourceOrdinals is not null&&selectedSourceOrdinals.Count!=source.Count||requiredSourceOrdinals is not null&&requiredSourceOrdinals.Count!=source.Count)
            throw new ArgumentException("Native source-map visit count does not match immutable source.");
        for(int index=0;index<source.Count;index++)
        foreach(var map in new[]{selectedSourceOrdinals?[index],requiredSourceOrdinals?[index]})
            if(map is not null && !ValidSourceMap(map,source[index].Points.Count))
                throw new ArgumentException("Native source maps must contain strictly ordered original ordinals including both endpoints.");

        if(!Enum.IsDefined(sourceContract))throw new ArgumentOutOfRangeException(nameof(sourceContract));
        if(sourceContract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth&&(stockChord is null||stockCurve is null))
            throw new InvalidOperationException("STOCK_CONTACT_DIRECTION_REQUIRES_ACTUAL_STOCK");
        settings=(settings??new()) with{PreservePlanarSourceProgress=sourceContract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth};settings.Validate();limits??=new();if(limits.MaximumCandidateEdges<0||limits.MaximumBlockPoints<2||limits.MaximumSourcePoints<2)throw new ArgumentOutOfRangeException(nameof(limits));
        if(EvidenceObserver.Value is {} observer)
        {
            var copied=source.Select(p=>new TaggedToolpath(p.Points.ToArray(),p.Spans.ToArray(),p.RegionIndex,p.Category,p.DepthPassIndex,p.VisitIdentity,detectCurves:false)).ToArray();
            observer(copied,regionBoundaries,profile,surfaceZ);
        }
        var topology=canonicalTopology??VCarveNativeTopologyContext.Create(source,token);
        if(source.Sum(p=>(long)p.Points.Count)>limits.MaximumSourcePoints||topology.SourcePointCount>limits.MaximumSourcePoints)
            return source.Select(p=>new VCarveNativeCurvePlan(p.RegionIndex,p.VisitIdentity,Math.Max(0,p.Points.Count-1),[],false,true,"source-point-budget-exceeded")).ToArray();
        if(nominationAlternatives is {Count:>0}&&(sourceContract!=VCarveNativeSourceContract.MedialPlanarWithStockContactDepth
            ||nominationAlternatives.Any(f=>f.SelectedSourceOrdinals.Count!=source.Count)))
            throw new ArgumentException("Alternative nominations require the complete stock-contact source map.");
        var guards=new Dictionary<int,VCarveNativeContainment>();var cache=sharedCache??new Dictionary<string,List<VCarveNativeFit>>();var result=new List<VCarveNativeCurvePlan>();var budget=sharedBudget??new VCarveNativeSearchBudget(token,limits.MaximumCandidateEdges);bool exhausted=false;
        for(int pathIndex=0;pathIndex<source.Count;pathIndex++)
        {
            var path=source[pathIndex];
            token.ThrowIfCancellationRequested();var p=path.Points.Select(p=>p with{Z=p.Z-surfaceZ}).ToArray();
            if(p.Any(q=>!double.IsFinite(q.X)||!double.IsFinite(q.Y)||!double.IsFinite(q.Z)))throw new ArgumentException("Native source coordinates must be finite.");
            if(p.Length<2){result.Add(new(path.RegionIndex,path.VisitIdentity,0,[],false,false,"empty-source"));continue;}
            if(path.RegionIndex<0||path.RegionIndex>=regionBoundaries.Count){result.Add(new(path.RegionIndex,path.VisitIdentity,p.Length-1,[],false,false,"unresolved-region"));continue;}
            if(path.Moves.Any(m=>m.Kind!=ToolMoveKind.Line)){result.Add(new(path.RegionIndex,path.VisitIdentity,p.Length-1,[],false,false,"existing-native-source-needs-adapter"));continue;}
            if(!guards.TryGetValue(path.RegionIndex,out var guard))guards[path.RegionIndex]=guard=new(regionBoundaries[path.RegionIndex],profile,token,stockChord,stockCurve);
            var visit=PrepareVisit(path,p,topology,requiredSourceOrdinals?[pathIndex],settings);
            var kinds=visit.Kinds;var anchors=visit.Anchors;var closedLengths=visit.ClosedLengths;
            bool preservedClosedFeature=visit.PreservedClosedFeature;
            var output=new List<VCarveNativePrimitive>();bool localBudget=false;
            int boundaryTangentMismatches=0,contactCurves=0;double maximumAxialChange=0;
            if(preservedClosedFeature)
            {
                for(int i=0;i<closedLengths.Length;i++)output.Add(new(ToolMove.Line(path.Points[i],path.Points[i+1]),
                    i,i+1,closedLengths[i],closedLengths[i],kinds[i]){ShortSourceFeature=new(0,p.Length-1)});
                diagnostic?.Invoke($"[VCarve.NativeShortVisit] region={path.RegionIndex} visit={path.VisitIdentity} closed=True sourceLengthMm={closedLengths.Sum():R} primitives={output.Count} exactSource=True clearanceWaived=False");
            }
            var rejectionCounts=new Dictionary<string,int>();
            for(int h=1;!preservedClosedFeature&&h<anchors.Length;h++)
            {
                var solved=SolveInterval(visit,h,pathIndex,regionBoundaries,profile,surfaceZ,token,limits,diagnostic,settings,
                    stockChord,stockCurve,selectedSourceOrdinals,sourceContract,topology,nominationAlternatives,budget,cache,guard,exhausted);
                output.AddRange(solved.Primitives);foreach(var pair in solved.Rejections)
                    rejectionCounts[pair.Key]=rejectionCounts.GetValueOrDefault(pair.Key)+pair.Value;
                boundaryTangentMismatches+=solved.BoundaryTangentMismatches;contactCurves+=solved.ContactCurves;
                maximumAxialChange=Math.Max(maximumAxialChange,solved.MaximumAxialChange);
                localBudget|=solved.BudgetExhausted;exhausted|=solved.BudgetExhausted;
            }
            var plan=FinalizeVisit(visit,output,rejectionCounts,boundaryTangentMismatches,contactCurves,maximumAxialChange,
                localBudget,exhausted,sourceContract,guard,surfaceZ,settings,diagnostic);result.Add(plan);
            RegionObserver.Value?.Invoke(pathIndex,Snapshot(plan));
        }
        CandidateObserver.Value?.Invoke(result.Select(p=>p with{Primitives=p.Primitives.ToArray(),SearchRejections=new System.Collections.ObjectModel.ReadOnlyDictionary<string,int>(p.SearchRejections.ToDictionary(x=>x.Key,x=>x.Value))}).ToArray());
        diagnostic?.Invoke($"[VCarve.NativeBudget] visits={source.Count} candidateAttempts={budget.Attempts} candidateLimit={limits.MaximumCandidateEdges} sharedBudget={sharedBudget is not null} unverifiedPlans={result.Count(p=>!p.CandidateVerified)} exhaustedPlans={result.Count(p=>p.BudgetExhausted)}");
        return result;
    }
    private sealed record PreparedNativeVisit(TaggedToolpath Path,Point3D[] Points,ToolpathSpanKind[] Kinds,
        int[] Anchors,double[] ClosedLengths,bool PreservedClosedFeature);
    private sealed record NativeIntervalResult(List<VCarveNativePrimitive> Primitives,Dictionary<string,int> Rejections,
        int BoundaryTangentMismatches,int ContactCurves,double MaximumAxialChange,bool BudgetExhausted);

    private static PreparedNativeVisit PrepareVisit(TaggedToolpath path,Point3D[] p,VCarveNativeTopologyContext topology,
        IReadOnlyList<int>? requiredSourceOrdinals,VCarveNativeFitSettings settings)
    {
            var kinds=Enumerable.Repeat(ToolpathSpanKind.Cut,p.Length-1).ToArray();var required=new SortedSet<int>{0,p.Length-1};
            foreach(var span in path.Spans)
            {
                if(span.StartIndex<0||span.EndIndex>=p.Length||span.EndIndex<span.StartIndex)throw new ArgumentException("Invalid native source span.");
                for(int i=span.StartIndex;i<span.EndIndex;i++)kinds[i]=span.Kind;
            }
            for(int i=1;i<kinds.Length;i++)if(kinds[i]!=kinds[i-1])required.Add(i);
            for(int i=0;i<kinds.Length;i++)if(kinds[i]is not (ToolpathSpanKind.Cut or ToolpathSpanKind.Link)){required.Add(i);required.Add(i+1);}
            for(int i=0;i<p.Length;i++)if(topology.IsVisitPort(path.Points[i])||topology.Degree(path.Points[i])!=2)required.Add(i);
            if(p[0]==p[^1]){int far=1;for(int i=2;i<p.Length-1;i++)if(VCarveNativeCurveMath.Distance(p[0],p[i])>VCarveNativeCurveMath.Distance(p[0],p[far]))far=i;required.Add(far);}
            if(requiredSourceOrdinals is not null)foreach(int ordinal in requiredSourceOrdinals)required.Add(ordinal);
            var closedLengths=p[0]==p[^1]?Enumerable.Range(0,p.Length-1)
                .Select(i=>VCarveNativeCurveMath.Distance(path.Points[i],path.Points[i+1])).ToArray():[];
            bool preservedClosedFeature=closedLengths.Length>0
                &&closedLengths.All(l=>double.IsFinite(l)&&l>0)
                &&closedLengths.Sum()<settings.MinimumPrimitiveLengthMm
                &&kinds.All(k=>k==kinds[0])&&kinds[0] is (ToolpathSpanKind.Cut or ToolpathSpanKind.Link)
                &&Enumerable.Range(1,p.Length-2).All(i=>!topology.IsVisitPort(path.Points[i])&&topology.Degree(path.Points[i])==2);
        return new(path,p,kinds,required.ToArray(),closedLengths,preservedClosedFeature);
    }

    // One EXISTING mandatory interval, with the full immutable visit and global
    // topology still in scope. No artificial visit ports, kinds, source maps or
    // authored endpoint tangents are manufactured by scheduling this unit.
    private static NativeIntervalResult SolveInterval(PreparedNativeVisit visit,int h,int pathIndex,
        IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> regionBoundaries,VCarveCutterProfile profile,double surfaceZ,
        CancellationToken token,VCarveNativeFitLimits limits,Action<string>? diagnostic,VCarveNativeFitSettings settings,
        Func<Point3D,Point3D,bool>? stockChord,Func<ToolMove,bool>? stockCurve,
        IReadOnlyList<IReadOnlyList<int>>? selectedSourceOrdinals,VCarveNativeSourceContract sourceContract,
        VCarveNativeTopologyContext topology,IReadOnlyList<VCarveNativeNominationFamily>? nominationAlternatives,
        VCarveNativeSearchBudget budget,Dictionary<string,List<VCarveNativeFit>> cache,VCarveNativeContainment guard,bool exhausted)
    {
        var path=visit.Path;var p=visit.Points;var kinds=visit.Kinds;var anchors=visit.Anchors;
        var output=new List<VCarveNativePrimitive>();var rejectionCounts=new Dictionary<string,int>();
        bool localBudget=false;int boundaryTangentMismatches=0,contactCurves=0;double maximumAxialChange=0;

                diagnostic?.Invoke($"[VCarve.NativeSpan] region={path.RegionIndex} block={h}/{anchors.Length-1} first={anchors[h-1]} last={anchors[h]} utc={DateTime.UtcNow:O}");
                token.ThrowIfCancellationRequested();int first=anchors[h-1],last=anchors[h];var raw=p[first..(last+1)];bool reverse=false;
                for(int i=0;i<raw.Length;i++){int cmp=Compare(raw[i],raw[raw.Length-1-i]);if(cmp!=0){reverse=cmp>0;break;}}
                var canonical=reverse?raw.Reverse().ToArray():raw;
                // This slice/reverse is private immutable source custody, never
                // an externally supplied array or observer-visible alias.
                settings=settings with{OwnedMinimaxSource=VCarveNativeGeometricProof.OwnedMinimaxSource.CapturePrivateSnapshot(canonical)};
                var incumbent=Enumerable.Range(0,canonical.Length-1).Select(i=>{double length=VCarveNativeCurveMath.Distance(canonical[i],canonical[i+1]);return new VCarveNativeFit(ToolMove.Line(canonical[i],canonical[i+1]),i,i+1,length,length,length,0,0,true);}).ToArray();
                var proposedKnots=selectedSourceOrdinals is null?null:selectedSourceOrdinals[pathIndex].Where(i=>i>=first&&i<=last).Select(i=>reverse?last-i:i-first).Append(0).Append(canonical.Length-1).Distinct().Order().ToArray();
                using var bytes=new MemoryStream();using(var writer=new BinaryWriter(bytes,Encoding.UTF8,true)){writer.Write(path.RegionIndex);// Cut/Link share geometric constraints and operation feed metadata;
                // their provenance is attached per occurrence after canonical fitting.
                writer.Write((int)(kinds[first] is ToolpathSpanKind.Cut or ToolpathSpanKind.Link?ToolpathSpanKind.Cut:kinds[first]));foreach(var q in canonical){writer.Write(q.X);writer.Write(q.Y);writer.Write(q.Z);}writer.Write(proposedKnots?.Length??-1);if(proposedKnots is not null)foreach(int ordinal in proposedKnots)writer.Write(ordinal);}
                string key=Convert.ToHexString(SHA256.HashData(bytes.ToArray()));
                bool PhysicalBoundary(int at)=>topology.IsVisitPort(path.Points[at])||topology.Degree(path.Points[at])!=2||(at>0&&at<kinds.Length&&kinds[at-1]!=kinds[at]);
                double sourceFeatureLength=incumbent.Sum(f=>f.SourceLength);
                // A whole genuine source feature below the target is preserved
                // deliberately before fitting. It is not a search-failure fallback.
                // Every retained line still receives the continuous stock proof.
                if(!cache.ContainsKey(key)
                    &&kinds[first] is (ToolpathSpanKind.Cut or ToolpathSpanKind.Link)
                    &&PhysicalBoundary(first)&&PhysicalBoundary(last)&&sourceFeatureLength>0&&sourceFeatureLength<settings.MinimumPrimitiveLengthMm)
                    cache[key]=incumbent.Select(f=>f with{PreservedShortFeature=true}).ToList();
                // A complete depth-limit contour branch is derived contact geometry.
                // Its junction positions remain exact, but polygon subdivision
                // directions are not authored tangents. A direct native line must
                // earn the same all-source fit and whole-cutter stock certificate.
                if(!cache.ContainsKey(key)&&!exhausted
                    &&sourceContract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth
                    &&topology.Degree(path.Points[first])!=2&&topology.Degree(path.Points[last])!=2
                    &&VCarveNativeDepthLimitFit.IsDepthLimitBranch(canonical,profile))
                {
                    try { budget.CheckCandidate();
                        var contour=VCarveNativeDepthLimitFit.Line(canonical,guard,profile,settings);
                        if(contour is not null)cache[key]=[contour];
                    } catch(VCarveNativeBudgetException){exhausted=true;localBudget=true;}
                }
                if(!cache.TryGetValue(key,out var selected))
                {
                    selected=incumbent.ToList();
                    if(!exhausted)
                    {
                        var knots=proposedKnots;
                        selected=knots is null?FitLongSpans(canonical,incumbent,guard,profile,budget,settings,limits.MaximumBlockPoints,
                            token,out bool blockExhausted,out var blockRejections):FitPreparedSourceSpans(canonical,knots,incumbent,guard,profile,budget,settings,limits.MaximumBlockPoints,token,out blockExhausted,out blockRejections,
                            sourceContract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth
                                &&topology.Degree(path.Points[first])!=2&&topology.Degree(path.Points[last])!=2
                                ?VCarveNativeSourceContract.MedialPlanarWithStockContactDepth:VCarveNativeSourceContract.PreparedSpatialDirections);
                        foreach(var pair in blockRejections)rejectionCounts[pair.Key]=rejectionCounts.GetValueOrDefault(pair.Key)+pair.Value;
                        if(blockExhausted){exhausted=true;localBudget=true;}
                        // Only an entire true topological branch may use the contact
                        // depth contract. Degree-two smooth/state joins retain their
                        // spatial continuity contract; no free fitting joint is added.
                        if(!blockExhausted&&sourceContract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth
                            &&canonical.Length>=3&&canonical.Length<=limits.MaximumBlockPoints
                            &&topology.Degree(path.Points[first])!=2&&topology.Degree(path.Points[last])!=2
                            &&selected.Any(f=>f.LengthLower<settings.MinimumPrimitiveLengthMm))
                        {
                            try { budget.CheckCandidate();
                            int next=knots is null?1:knots[1],prior=knots is null?canonical.Length-2:knots[^2];
                            var start=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(canonical[next],canonical[0]));
                            var end=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(canonical[^1],canonical[prior]));
                            var contact=VCarveNativeStockContactFit.Find(canonical,start,end,guard,profile,settings,token,out var contactStatus);
                            if(contact is not null)selected=[contact];
                            else rejectionCounts[contactStatus]=rejectionCounts.GetValueOrDefault(contactStatus)+1;
                            } catch(VCarveNativeBudgetException){exhausted=true;localBudget=true;}
                        }
                    }
                    else localBudget=true;
                    cache[key]=selected;
                }
                bool alternativeChosen=false;
                if(!exhausted&&nominationAlternatives is {Count:>0}
                    &&sourceContract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth
                    &&kinds[first] is (ToolpathSpanKind.Cut or ToolpathSpanKind.Link)
                    &&topology.Degree(path.Points[first])!=2&&topology.Degree(path.Points[last])!=2
                    &&(selected.Count==0||selected.Any(f=>f.LengthLower<settings.MinimumPrimitiveLengthMm&&!f.PreservedShortFeature)
                        ||BoundaryTangentMismatches(canonical,proposedKnots,selected)>0))
                {
                    var tried=new HashSet<string>();
                    foreach(var family in nominationAlternatives)
                    {
                        token.ThrowIfCancellationRequested();
                        var alternateKnots=family.SelectedSourceOrdinals[pathIndex].Where(i=>i>=first&&i<=last)
                            .Select(i=>i-first).Append(0).Append(last-first).Distinct().Order().ToArray();
                        var canonicalKnots=alternateKnots.Select(i=>reverse?last-first-i:i).Order().ToArray();
                        if(proposedKnots is not null&&canonicalKnots.SequenceEqual(proposedKnots)
                            ||!tried.Add(string.Join(',',canonicalKnots)))continue;
                        var branch=new TaggedToolpath(path.Points.Skip(first).Take(last-first+1).ToArray(),
                            [new(0,last-first,kinds[first])],path.RegionIndex,path.Category,path.DepthPassIndex,path.VisitIdentity,detectCurves:false);
                        var sourceObserver=EvidenceObserver.Value;var candidateObserver=CandidateObserver.Value;var regionObserver=RegionObserver.Value;
                        VCarveNativeCurvePlan alternate;
                        diagnostic?.Invoke($"[VCarve.NativeNomination] event=try family={family.Name} region={path.RegionIndex} first={first} last={last} sameBudget=True candidateAttempts={budget.Attempts} candidateLimit={limits.MaximumCandidateEdges} sourceGeometryUnchanged=True");
                        try
                        {
                            EvidenceObserver.Value=null;CandidateObserver.Value=null;RegionObserver.Value=null;
                            alternate=NominateCore([branch],regionBoundaries,profile,surfaceZ,token,limits,diagnostic,settings,
                                stockChord,stockCurve,[alternateKnots],[[0,last-first]],sourceContract,topology,null,budget,cache).Single();
                        }
                        finally{EvidenceObserver.Value=sourceObserver;CandidateObserver.Value=candidateObserver;RegionObserver.Value=regionObserver;}
                        foreach(var rejection in alternate.SearchRejections)
                        {
                            string reason=$"nomination:{family.Name}:{rejection.Key}";
                            rejectionCounts[reason]=rejectionCounts.GetValueOrDefault(reason)+rejection.Value;
                        }
                        if(alternate.BudgetExhausted){exhausted=true;localBudget=true;break;}
                        if(!alternate.CandidateVerified||alternate.Primitives.Any(f=>!double.IsFinite(f.LengthLowerMm)||!double.IsFinite(f.LengthUpperMm)
                            ||f.LengthLowerMm<=0||f.LengthUpperMm<f.LengthLowerMm
                            ||f.LengthLowerMm<settings.MinimumPrimitiveLengthMm&&f.ShortSourceFeature is null))continue;
                        output.AddRange(alternate.Primitives.Select(f=>f with{SourceFirst=f.SourceFirst+first,SourceLast=f.SourceLast+first,
                            ShortSourceFeature=f.ShortSourceFeature is {} feature?new(feature.First+first,feature.Last+first):null}));
                        contactCurves+=alternate.StockContactEnvelopeCurves;
                        maximumAxialChange=Math.Max(maximumAxialChange,alternate.MaximumSourceAxialDirectionChangeDegrees);
                        diagnostic?.Invoke($"[VCarve.NativeNomination] event=selected family={family.Name} region={path.RegionIndex} first={first} last={last} primitives={alternate.Primitives.Count} sameBudget=True candidateAttempts={budget.Attempts} candidateLimit={limits.MaximumCandidateEdges} sourceGeometryUnchanged=True completeCertificates=True");
                        alternativeChosen=true;break;
                    }
                }
                if(alternativeChosen)return new(output,rejectionCounts,boundaryTangentMismatches,contactCurves,maximumAxialChange,localBudget||exhausted);
                boundaryTangentMismatches+=BoundaryTangentMismatches(canonical,proposedKnots,selected);
                contactCurves+=selected.Count(f=>f.StockContactAxialEnvelope);
                maximumAxialChange=Math.Max(maximumAxialChange,selected.Where(f=>f.StockContactAxialEnvelope).Select(f=>Math.Max(f.TangentStart,f.TangentEnd)).DefaultIfEmpty().Max());
                diagnostic?.Invoke($"[VCarve.NativeSpanResult] region={path.RegionIndex} block={h}/{anchors.Length-1} first={first} last={last} primitives={selected.Count} unacceptedShorts={selected.Count(f=>f.LengthLower<settings.MinimumPrimitiveLengthMm&&!f.PreservedShortFeature)} preservedShortPrimitives={selected.Count(f=>f.PreservedShortFeature)} budgetExhausted={localBudget||exhausted}");
                foreach(var f in reverse?selected.AsEnumerable().Reverse():selected)
                {
                    int a=first+(reverse?raw.Length-1-f.Last:f.First),b=first+(reverse?raw.Length-1-f.First:f.Last);var m=reverse?VCarveNativeCurveMath.Reverse(f.Move):f.Move;
                    Point3D World(Point3D q)=>q with{Z=q.Z+surfaceZ};m=m.Kind==ToolMoveKind.CubicBezier?ToolMove.CubicBezier(World(m.Start),World(m.Control1),World(m.Control2),World(m.End),m.MaxError):ToolMove.Line(World(m.Start),World(m.End),m.MaxError);
                    if(f.PreservedShortFeature)m=ToolMove.Line(path.Points[a],path.Points[b]);
                    output.Add(new(m,a,b,f.LengthLower,f.LengthUpper,kinds[a]){ShortSourceFeature=f.PreservedShortFeature?new(first,last):null});
                }
        return new(output,rejectionCounts,boundaryTangentMismatches,contactCurves,maximumAxialChange,localBudget||exhausted);
    }

    private static VCarveNativeCurvePlan FinalizeVisit(PreparedNativeVisit visit,List<VCarveNativePrimitive> output,
        Dictionary<string,int> rejectionCounts,int boundaryTangentMismatches,int contactCurves,double maximumAxialChange,
        bool localBudget,bool exhausted,VCarveNativeSourceContract sourceContract,VCarveNativeContainment guard,
        double surfaceZ,VCarveNativeFitSettings settings,Action<string>? diagnostic)
    {
        var path=visit.Path;var p=visit.Points;

            int uncertifiedLines=0;
            foreach(var primitive in output.Where(x=>x.Move.Kind==ToolMoveKind.Line))
            {
                var a=primitive.Move.Start with{Z=primitive.Move.Start.Z-surfaceZ};
                var b=primitive.Move.End with{Z=primitive.Move.End.Z-surfaceZ};
                if(!guard.Legal(a,b))uncertifiedLines++;
            }
            int planarProgressFailures=sourceContract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth?output.Count(f=>!VCarveNativePlanarProgress.Preserved(f.Move,path.Points,f.SourceFirst,f.SourceLast,out _)):0;
            if(planarProgressFailures>0)rejectionCounts["source-planar-progress-contract-unresolved"]=planarProgressFailures;
            if(uncertifiedLines>0)rejectionCounts["retained-line-not-clearance-certified"]=uncertifiedLines;
            if(boundaryTangentMismatches>0)rejectionCounts["protected-boundary-tangent-contract-unresolved"]=boundaryTangentMismatches;
            var plan=new VCarveNativeCurvePlan(path.RegionIndex,path.VisitIdentity,p.Length-1,output,uncertifiedLines==0&&boundaryTangentMismatches==0&&planarProgressFailures==0,localBudget||exhausted,uncertifiedLines>0?"native-retained-line-clearance-unverified":boundaryTangentMismatches>0?"native-protected-boundary-tangent-unresolved":planarProgressFailures>0?"native-source-planar-progress-unresolved":"native-design-fit-and-clearance-checked"){MinimumPrimitiveLengthMm=settings.MinimumPrimitiveLengthMm,SearchRejections=rejectionCounts,StockContactEnvelopeCurves=contactCurves,MaximumSourceAxialDirectionChangeDegrees=maximumAxialChange};
            diagnostic?.Invoke($"[VCarve.NativeFit] region={path.RegionIndex} sourceSegments={p.Length-1} nativeCandidates={output.Count} cubics={output.Count(x=>x.Move.Kind==ToolMoveKind.CubicBezier)} belowMinimum={plan.BelowMinimum} preservedFeaturePrimitives={output.Count(p=>p.ShortSourceFeature is not null)} fitErrorMm={settings.FitErrorMm:R} minimumLengthMm={settings.MinimumPrimitiveLengthMm:R} sourceContract={sourceContract} contactEnvelopeCurves={contactCurves} maxSourceAxialDirectionChangeDegrees={maximumAxialChange:R} materialCoverageMeasured=False candidateVerified={plan.CandidateVerified} budgetExhausted={plan.BudgetExhausted}");
            var shortKinds=output.Where(x=>x.LengthLowerMm<settings.MinimumPrimitiveLengthMm).GroupBy(x=>x.Kind).ToDictionary(x=>x.Key.ToString(),x=>x.Count());
            diagnostic?.Invoke($"[VCarve.NativeAudit] region={path.RegionIndex} anchors={visit.Anchors.Length} sourceIntervals={p.Length-1} shortKinds={System.Text.Json.JsonSerializer.Serialize(shortKinds)} candidateSearchRejections={System.Text.Json.JsonSerializer.Serialize(rejectionCounts)} rejectedCandidatesAreNotNecessityProof=True");
        return plan;
    }

    // Propose complete long spans before subdividing. Raw sample density must
    // not cap a curve to sixteen tiny source intervals. Every accepted span
    // still uses ALL source samples in the continuous bound and the unchanged
    // continuous cutter proof. On exhaustion retain certified progress plus
    // original chords as diagnostic candidates. Strict transport rejects any
    // unresolved short, unverified or exhausted plan; this is not output fallback.
    private static List<VCarveNativeFit> FitLongSpans(Point3D[] points,VCarveNativeFit[] original,
        VCarveNativeContainment guard,VCarveCutterProfile profile,VCarveNativeSearchBudget budget,
        VCarveNativeFitSettings settings,int maximumBlockPoints,CancellationToken token,out bool exhausted,out IReadOnlyDictionary<string,int> searchRejections)
    {
        var result=new List<VCarveNativeFit>();bool stopped=false;var failures=new Dictionary<string,int>();
        var cumulative=new double[points.Length];
        for(int i=1;i<points.Length;i++)cumulative[i]=cumulative[i-1]+VCarveNativeCurveMath.Distance(points[i-1],points[i]);
        void Visit(int first,int last)
        {
            token.ThrowIfCancellationRequested();
            if(last-first==1){result.Add(original[first]);return;}
            double length=cumulative[last]-cumulative[first];
            if(stopped||length<settings.MinimumPrimitiveLengthMm)
            {for(int i=first;i<last;i++)result.Add(original[i]);return;}
            if(last-first+1<=maximumBlockPoints)
            {
                try
                {
                    budget.CheckCandidate();
                    var u=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[first+1],points[first]));
                    var v=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[last],points[last-1]));
                    var fit=VCarveNativeEdgeSearch.Find(points,first,last,u,v,length,guard,profile,out var edgeAudit,settings:settings);
                    if(fit is null)failures[edgeAudit.Outcome]=failures.GetValueOrDefault(edgeAudit.Outcome)+1;
                    if(fit is not null){result.Add(fit);return;}
                }
                catch(VCarveNativeBudgetException){stopped=true;}
            }
            int middle=(first+last)/2;
            Visit(first,middle);Visit(middle,last);
        }
        Visit(0,points.Length-1);exhausted=stopped;searchRejections=failures;return result;
    }
    private static int BoundaryTangentMismatches(Point3D[] canonical,int[]? knots,IReadOnlyList<VCarveNativeFit> selected)
    {
        if(knots is null||selected.Count==0||selected.All(f=>f.DepthLimitContour||f.PreservedShortFeature))return 0;
        var expectedStart=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(canonical[knots[1]],canonical[0]));
        var expectedEnd=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(canonical[^1],canonical[knots[^2]]));
        var actualStart=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(selected[0].Move,0));
        var actualEnd=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(selected[^1].Move,1));
        if(selected.Any(f=>f.StockContactAxialEnvelope))
        {
            expectedStart=VCarveNativeStockContactFit.Planar(expectedStart);expectedEnd=VCarveNativeStockContactFit.Planar(expectedEnd);
            actualStart=VCarveNativeStockContactFit.Planar(actualStart);actualEnd=VCarveNativeStockContactFit.Planar(actualEnd);
        }
        return (VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(actualStart,expectedStart))>1e-8?1:0)
            +(VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(actualEnd,expectedEnd))>1e-8?1:0);
    }
    private static int Compare(Point3D a,Point3D b){int x=a.X.CompareTo(b.X);if(x!=0)return x;int y=a.Y.CompareTo(b.Y);return y!=0?y:a.Z.CompareTo(b.Z);}
}
