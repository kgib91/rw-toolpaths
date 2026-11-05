using System.Security.Cryptography;
using System.Text;
using Clipper2Lib;
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
    private int _attempts;
    internal int Attempts=>_attempts;
    internal void CheckCandidate(){token.ThrowIfCancellationRequested();if(++_attempts>maximum)throw new VCarveNativeBudgetException();}
}
/// <summary>Selected fitting policy; independent of primitive length and the
/// operation configuration. Inconclusive design candidates are diagnostic evidence only;
/// unsupported machine consumers reject typed native IL rather than retessellate it.</summary>
public sealed record VCarveNativeFitSettings(double FitErrorMm=.3,double MinimumPrimitiveLengthMm=.3)
{
 internal bool PreservePlanarSourceProgress {get;init;}
 public void Validate(){if(!double.IsFinite(FitErrorMm)||FitErrorMm<=0||!double.IsFinite(MinimumPrimitiveLengthMm)||MinimumPrimitiveLengthMm<=0)throw new ArgumentOutOfRangeException(nameof(FitErrorMm));}
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
    public const string Policy="native-geometric-order-clearance-and-short-feature-preservation-v11";
    public const double FitErrorMm=.3;
    public const double MinimumPrimitiveLengthMm=.3;
    public static IReadOnlyList<VCarveNativeCurvePlan> Nominate(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> regionBoundaries,VCarveCutterProfile profile,
        double surfaceZ,CancellationToken token=default,VCarveNativeFitLimits? limits=null,Action<string>? diagnostic=null,VCarveNativeFitSettings? settings=null,
        Func<Point3D,Point3D,bool>? stockChord=null,Func<ToolMove,bool>? stockCurve=null,
        IReadOnlyList<IReadOnlyList<int>>? selectedSourceOrdinals=null,IReadOnlyList<IReadOnlyList<int>>? requiredSourceOrdinals=null,
        VCarveNativeSourceContract sourceContract=VCarveNativeSourceContract.PreparedSpatialDirections,VCarveNativeTopologyContext? canonicalTopology=null,
        IReadOnlyList<VCarveNativeNominationFamily>? nominationAlternatives=null)
        =>NominateCore(source,regionBoundaries,profile,surfaceZ,token,limits,diagnostic,settings,
            stockChord,stockCurve,selectedSourceOrdinals,requiredSourceOrdinals,sourceContract,canonicalTopology,nominationAlternatives,null,null);

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
            if(map is not null && (map.Count<2||map[0]!=0||map[^1]!=source[index].Points.Count-1||map.Where((n,i)=>n<0||n>=source[index].Points.Count||i>0&&n<=map[i-1]).Any()))
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
            if(requiredSourceOrdinals is not null)foreach(int ordinal in requiredSourceOrdinals[pathIndex])required.Add(ordinal);
            var output=new List<VCarveNativePrimitive>();var anchors=required.ToArray();bool localBudget=false;int boundaryTangentMismatches=0,contactCurves=0;double maximumAxialChange=0;
            // A complete closed visit is a genuine feature too. Its farthest
            // point below is only a fitting split, not a new topology boundary.
            // Preserve a sub-target visit before search, with one whole-source
            // identity; the ordinary retained-line clearance audit still runs.
            var closedLengths=p[0]==p[^1]?Enumerable.Range(0,p.Length-1)
                .Select(i=>VCarveNativeCurveMath.Distance(path.Points[i],path.Points[i+1])).ToArray():[];
            bool preservedClosedFeature=closedLengths.Length>0
                &&closedLengths.All(l=>double.IsFinite(l)&&l>0)
                &&closedLengths.Sum()<settings.MinimumPrimitiveLengthMm
                &&kinds.All(k=>k==kinds[0])&&kinds[0] is (ToolpathSpanKind.Cut or ToolpathSpanKind.Link)
                &&Enumerable.Range(1,p.Length-2).All(i=>!topology.IsVisitPort(path.Points[i])&&topology.Degree(path.Points[i])==2);
            if(preservedClosedFeature)
            {
                for(int i=0;i<closedLengths.Length;i++)output.Add(new(ToolMove.Line(path.Points[i],path.Points[i+1]),
                    i,i+1,closedLengths[i],closedLengths[i],kinds[i]){ShortSourceFeature=new(0,p.Length-1)});
                diagnostic?.Invoke($"[VCarve.NativeShortVisit] region={path.RegionIndex} visit={path.VisitIdentity} closed=True sourceLengthMm={closedLengths.Sum():R} primitives={output.Count} exactSource=True clearanceWaived=False");
            }
            var rejectionCounts=new Dictionary<string,int>();
            for(int h=1;!preservedClosedFeature&&h<anchors.Length;h++)
            {
                diagnostic?.Invoke($"[VCarve.NativeSpan] region={path.RegionIndex} block={h}/{anchors.Length-1} first={anchors[h-1]} last={anchors[h]} utc={DateTime.UtcNow:O}");
                token.ThrowIfCancellationRequested();int first=anchors[h-1],last=anchors[h];var raw=p[first..(last+1)];bool reverse=false;
                for(int i=0;i<raw.Length;i++){int cmp=Compare(raw[i],raw[raw.Length-1-i]);if(cmp!=0){reverse=cmp>0;break;}}
                var canonical=reverse?raw.Reverse().ToArray():raw;var incumbent=Enumerable.Range(0,canonical.Length-1).Select(i=>{double length=VCarveNativeCurveMath.Distance(canonical[i],canonical[i+1]);return new VCarveNativeFit(ToolMove.Line(canonical[i],canonical[i+1]),i,i+1,length,length,length,0,0,true);}).ToArray();
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
                if(alternativeChosen)continue;
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
            }
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
            var plan=new VCarveNativeCurvePlan(path.RegionIndex,path.VisitIdentity,p.Length-1,output,uncertifiedLines==0&&boundaryTangentMismatches==0&&planarProgressFailures==0,localBudget||exhausted,uncertifiedLines>0?"native-retained-line-clearance-unverified":boundaryTangentMismatches>0?"native-protected-boundary-tangent-unresolved":planarProgressFailures>0?"native-source-planar-progress-unresolved":"native-design-fit-and-clearance-checked"){MinimumPrimitiveLengthMm=settings.MinimumPrimitiveLengthMm,SearchRejections=rejectionCounts,StockContactEnvelopeCurves=contactCurves,MaximumSourceAxialDirectionChangeDegrees=maximumAxialChange};result.Add(plan);
            RegionObserver.Value?.Invoke(pathIndex,plan with{Primitives=plan.Primitives.ToArray(),SearchRejections=new System.Collections.ObjectModel.ReadOnlyDictionary<string,int>(plan.SearchRejections.ToDictionary(x=>x.Key,x=>x.Value))});
            diagnostic?.Invoke($"[VCarve.NativeFit] region={path.RegionIndex} sourceSegments={p.Length-1} nativeCandidates={output.Count} cubics={output.Count(x=>x.Move.Kind==ToolMoveKind.CubicBezier)} belowMinimum={plan.BelowMinimum} preservedFeaturePrimitives={output.Count(p=>p.ShortSourceFeature is not null)} fitErrorMm={settings.FitErrorMm:R} minimumLengthMm={settings.MinimumPrimitiveLengthMm:R} sourceContract={sourceContract} contactEnvelopeCurves={contactCurves} maxSourceAxialDirectionChangeDegrees={maximumAxialChange:R} materialCoverageMeasured=False candidateVerified={plan.CandidateVerified} budgetExhausted={plan.BudgetExhausted}");
            var shortKinds=output.Where(x=>x.LengthLowerMm<settings.MinimumPrimitiveLengthMm).GroupBy(x=>x.Kind).ToDictionary(x=>x.Key.ToString(),x=>x.Count());
            diagnostic?.Invoke($"[VCarve.NativeAudit] region={path.RegionIndex} anchors={required.Count} sourceIntervals={p.Length-1} shortKinds={System.Text.Json.JsonSerializer.Serialize(shortKinds)} candidateSearchRejections={System.Text.Json.JsonSerializer.Serialize(rejectionCounts)} rejectedCandidatesAreNotNecessityProof=True");
        }
        CandidateObserver.Value?.Invoke(result.Select(p=>p with{Primitives=p.Primitives.ToArray(),SearchRejections=new System.Collections.ObjectModel.ReadOnlyDictionary<string,int>(p.SearchRejections.ToDictionary(x=>x.Key,x=>x.Value))}).ToArray());
        diagnostic?.Invoke($"[VCarve.NativeBudget] visits={source.Count} candidateAttempts={budget.Attempts} candidateLimit={limits.MaximumCandidateEdges} sharedBudget={sharedBudget is not null} unverifiedPlans={result.Count(p=>!p.CandidateVerified)} exhaustedPlans={result.Count(p=>p.BudgetExhausted)}");
        return result;
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
