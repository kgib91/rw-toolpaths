namespace RW.Toolpaths.Milling;

/// <summary>Experimental intact-source jobs for a mixed XY sweep. Definitions
/// have already been depth-materialized. This owner only groups and schedules
/// them; the established path router still constructs every actual entry.</summary>
internal sealed class VCarveRingSpatialDispatch
{
    internal sealed class Bundle
    {
        internal readonly TaggedToolpath[] Sources;
        internal readonly int Ordinal;
        internal readonly double Key,Extent;
        internal bool Done;
        internal Bundle(TaggedToolpath[] sources,int ordinal,VCarveRingOrder.SpatialAxis axis)
        {
            Sources=sources;Ordinal=ordinal;
            double min=double.PositiveInfinity,max=double.NegativeInfinity;
            foreach(var path in sources)foreach(var point in path.Points){
                double value=axis==VCarveRingOrder.SpatialAxis.X?point.X:point.Y;min=Math.Min(min,value);max=Math.Max(max,value);
            }
            Key=min;Extent=max-min;
        }
        internal double AccessDistanceSquared(Point3D? from)
        {
            if(from is not {} p)return 0;
            double best=double.PositiveInfinity;
            foreach(var path in Sources){
                var a=path.Points[0];var b=path.Points[^1];
                best=Math.Min(best,Math.Min(Math.Pow(p.X-a.X,2)+Math.Pow(p.Y-a.Y,2),Math.Pow(p.X-b.X,2)+Math.Pow(p.Y-b.Y,2)));
            }
            return best;
        }
    }
    private readonly Bundle[] bundles;
    private readonly VCarveRingOrder.SpatialSweep sweep;
    private int next,completed,completedSources,actualOutputs,emptyOutputs,rootResets,endpointOnlyResets;
    private double firstFrontier=double.NaN,lastFrontier=double.NaN;
    private int advances,reversals;
    internal IReadOnlyList<Bundle> Bundles=>bundles;
    internal bool Pending=>completed<bundles.Length;
    internal double Frontier
    {
        get{while(next<bundles.Length&&bundles[next].Done)next++;return next<bundles.Length?bundles[next].Key:double.PositiveInfinity;}
    }
    private VCarveRingSpatialDispatch(TaggedToolpath[] sources,VCarveRingOrder.SpatialSweep sweep,CancellationToken token)
    {
        this.sweep=sweep;int[] parents=Enumerable.Range(0,sources.Length).ToArray();
        int Root(int index){while(parents[index]!=index){parents[index]=parents[parents[index]];index=parents[index];}return index;}
        void Union(int first,int second){int a=Root(first),b=Root(second);if(a!=b)parents[b]=a;}
        var definitions=new Dictionary<ToolpathCircuitBuilder.Definition,int>(ReferenceEqualityComparer.Instance);
        var owners=new Dictionary<(ToolpathContinuationComponents.Plan,int),int>();
        for(int i=0;i<sources.Length;i++){
            token.ThrowIfCancellationRequested();var path=sources[i];
            if(path.ContinuationCircuitDefinition is {} definition){
                if(definitions.TryGetValue(definition,out int previous))Union(i,previous);else definitions.Add(definition,i);
            }
            if(path.ContinuationPlan is {} plan&&path.ContinuationSourceOrdinal is {} owner){
                if(owners.TryGetValue((plan,owner),out int previous))Union(i,previous);else owners.Add((plan,owner),i);
            }
        }
        bundles=Enumerable.Range(0,sources.Length).GroupBy(Root)
            .Select(g=>new Bundle(g.Select(i=>sources[i]).ToArray(),g.Min(),sweep.Axis)).OrderBy(b=>b.Key).ThenBy(b=>b.Ordinal).ToArray();
    }
    internal static VCarveRingSpatialDispatch? Create(TaggedToolpath[] remaining,IReadOnlyList<TaggedToolpath> all,
        DepthSchedule depth,VCarveOptions options,VCarveRingOrder.SpatialSweep? sweep,CancellationToken token,Action<string>? log)
    {
        if(VCarveRingRouter.SpatialMixedSourcesOverride.Value!=true)return null;
        string? reason=sweep is null?"no-spatial-sweep":!options.SmoothVCarving||!options.OptimizeContourConnections?"smooth-optimized-routing-required"
            :depth.DepthPerPass>0&&depth.DepthPerPass<depth.Depth-1e-9||all.Any(p=>p.DepthPassIndex is >0)?"multiple-depth-layers"
            :all.Any(p=>p.DepthPassIndex is <0)?"unsupported-depth-pass"
            :remaining.Length==0?"no-other-sources"
            :remaining.Any(p=>p.Points.Count<2||p.Points.Any(v=>!double.IsFinite(v.X)||!double.IsFinite(v.Y)||!double.IsFinite(v.Z)))?"degenerate-or-nonfinite-source":null;
        if(reason is not null){log?.Invoke($"stage=ring-spatial-dispatch event=retained reason={reason} originalAllRingsThenOtherOrder=True");return null;}
        var result=new VCarveRingSpatialDispatch(remaining,sweep!,token);
        log?.Invoke($"stage=ring-spatial-dispatch event=start {result.Summary()} typedDefinitionsMaterializedBeforeRouting=True wholeComponentsNotAtomic=True sourceSplitting=False");
        return result;
    }
    internal Bundle? Select(double frontier,Point3D? cursor,CancellationToken token)
    {
        if(double.IsNaN(firstFrontier))firstFrontier=frontier;
        else if(frontier>lastFrontier+1e-9)advances++;
        else if(frontier<lastFrontier-1e-9)reversals++;
        lastFrontier=frontier;
        Bundle? best=null;double distance=double.PositiveInfinity;
        foreach(var bundle in bundles){
            token.ThrowIfCancellationRequested();if(bundle.Done)continue;if(bundle.Key>frontier+sweep.WindowMm)break;
            double score=bundle.AccessDistanceSquared(cursor);
            if(best is null||score<distance-1e-9){best=bundle;distance=score;}
        }
        return best;
    }
    internal void Complete(Bundle bundle,int outputs,bool anchoredCut)
    {
        if(bundle.Done)throw new InvalidOperationException("A mixed spatial source bundle was dispatched twice.");
        bundle.Done=true;completed++;completedSources+=bundle.Sources.Length;actualOutputs+=outputs;rootResets++;
        if(!anchoredCut)endpointOnlyResets++;
    }
    internal void EmptyOutput()=>emptyOutputs++;
    internal string Summary()=>$"mixedSpatialSources=True spatialAxis={sweep.Axis} spatialWindowMm={sweep.WindowMm:R} "
        +$"bundles={bundles.Length} dispatchedBundles={completed} dispatchedSources={completedSources} actualOutputs={actualOutputs} pendingBundles={bundles.Length-completed} "
        +$"typedDefinitionBundles={bundles.Count(b=>b.Sources.Any(p=>p.ContinuationCircuitDefinition is not null))} maximumBundleSources={bundles.Max(b=>b.Sources.Length)} "
        +$"windowSpanningBundles={bundles.Count(b=>b.Extent>sweep.WindowMm)} globalFrontierFirst={firstFrontier:R} globalFrontierLast={lastFrontier:R} "
        +$"globalFrontierAdvances={advances} globalFrontierReversals={reversals} emptyBundleOutputs={emptyOutputs} rootReturnResets={rootResets} endpointOnlyResets={endpointOnlyResets} "
        +"eligibility=single-physical-depth-pass-zero grouping=definition-reference-or-exact-plan-source-identity selection=active-window-nearest-original-endpoint futureCutCredit=False";
}
