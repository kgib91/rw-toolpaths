namespace RW.Toolpaths.Milling;

// Construct a complete path of certified minimum-length edges. A long prefix
// is never accepted unless its suffix also has a complete solution. Candidate
// generation and certification use the existing owners and shared work budget.
internal static class VCarveNativeMinimumPath
{
    internal sealed record SearchStats(int VisitedNodes,int TriedEdges,int FailedNodes);

    internal static List<VCarveNativeFit>? Search(int[] knots,double[] cumulative,double minimum,int maximumBlockPoints,
        Func<int,int,VCarveNativeFit?> find,CancellationToken token,out SearchStats stats)
    {
        const int lookahead=48; // Same prepared-knot horizon as the existing fitter.
        int count=knots.Length,last=count-1,visited=0,tried=0,failed=0;
        var state=new byte[count];state[last]=1;
        var next=Enumerable.Range(0,count).Select(i=>Math.Min(last,i+lookahead)).ToArray();
        var pending=Enumerable.Repeat(-1,count).ToArray();var stops=new int[count];
        var pendingFit=new VCarveNativeFit?[count];
        var chosen=new VCarveNativeFit?[count];var stack=new Stack<int>();stack.Push(0);
        while(stack.Count>0)
        {
            token.ThrowIfCancellationRequested();int at=stack.Peek();
            if(state[at]!=0){stack.Pop();continue;}
            if(pending[at]>=0)
            {
                int suffix=pending[at];pending[at]=-1;
                if(state[suffix]==1)
                {chosen[at]=pendingFit[at];stops[at]=suffix;state[at]=1;stack.Pop();}
                pendingFit[at]=null;
                continue;
            }
            int stop=next[at]--;
            if(stop<=at){state[at]=2;failed++;stack.Pop();continue;}
            if(next[at]==Math.Min(last,at+lookahead)-1)visited++;
            int a=knots[at],b=knots[stop];
            if(state[stop]==2||b-a+1>maximumBlockPoints||cumulative[b]-cumulative[a]<minimum
                ||stop<last&&cumulative[^1]-cumulative[b]<minimum)continue;
            tried++;var fit=find(knots[at],knots[stop]);
            if(fit is null||!double.IsFinite(fit.LengthLower)||!double.IsFinite(fit.LengthUpper)
                ||fit.LengthLower<minimum||fit.LengthUpper<fit.LengthLower)continue;
            if(state[stop]==0)
            {pending[at]=stop;pendingFit[at]=fit;stack.Push(stop);continue;}
            chosen[at]=fit;stops[at]=stop;state[at]=1;stack.Pop();
        }
        stats=new(visited,tried,failed);
        if(state[0]!=1)return null;
        var result=new List<VCarveNativeFit>();
        for(int at=0;at<last;at=stops[at])result.Add(chosen[at]??throw new InvalidOperationException("Incomplete minimum-length path"));
        return result;
    }

    internal static List<VCarveNativeFit> Fit(Point3D[] points,int[] knots,VCarveNativeFit[] diagnosticSource,
        VCarveNativeContainment guard,VCarveCutterProfile profile,VCarveNativeSearchBudget budget,
        VCarveNativeFitSettings settings,int maximumBlockPoints,CancellationToken token,
        out bool exhausted,out IReadOnlyDictionary<string,int> searchRejections,
        VCarveNativeSourceContract contract=VCarveNativeSourceContract.MedialPlanarWithStockContactDepth)
    {
        var watch=System.Diagnostics.Stopwatch.StartNew();
        var failures=new Dictionary<string,int>();var cumulative=new double[points.Length];
        for(int i=1;i<points.Length;i++)cumulative[i]=cumulative[i-1]+VCarveNativeCurveMath.Distance(points[i-1],points[i]);
        var index=knots.Select((point,i)=>(point,i)).ToDictionary(x=>x.point,x=>x.i);
        bool sharedJoins=false,planarJoins=false,endpointChords=false;int sourceSide=0;
        IReadOnlyList<double>? handleScales=null;
        Point3D SharedDirection(int ordinal)
        {
            if(ordinal==0)return VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[knots[1]],points[0]));
            if(ordinal==points.Length-1)return VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[^1],points[knots[^2]]));
            if(endpointChords)
            {
                double start=Math.Pow(points[ordinal].X-points[0].X,2)+Math.Pow(points[ordinal].Y-points[0].Y,2);
                double end=Math.Pow(points[ordinal].X-points[^1].X,2)+Math.Pow(points[ordinal].Y-points[^1].Y,2);
                double reach=settings.MinimumPrimitiveLengthMm*settings.MinimumPrimitiveLengthMm;
                if(start<=reach&&start<=end)return VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[ordinal],points[0]));
                if(end<=reach)return VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[^1],points[ordinal]));
            }
            if(sourceSide!=0)return VCarveNativeCurveMath.Unit(sourceSide>0
                ?VCarveNativeCurveMath.Sub(points[ordinal+1],points[ordinal])
                :VCarveNativeCurveMath.Sub(points[ordinal],points[ordinal-1]));
            int found=Array.BinarySearch(knots,ordinal),after=found>=0?found+1:~found,before=found>=0?found-1:after-1;
            var direction=VCarveNativeCurveMath.Sub(points[knots[after]],points[knots[before]]);
            return VCarveNativeCurveMath.Unit(planarJoins?direction with{Z=0}:direction);
        }
        void Reject(string reason)=>failures[reason]=failures.GetValueOrDefault(reason)+1;
        VCarveNativeFit? Find(int a,int b)
        {
            token.ThrowIfCancellationRequested();int at=index[a],stop=index[b];
            var u=sharedJoins?SharedDirection(a):VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[knots[at+1]],points[a]));
            var v=sharedJoins?SharedDirection(b):VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[b],points[knots[stop-1]]));
            budget.CheckCandidate();
            var fit=VCarveNativeEdgeSearch.Find(points,a,b,u,v,cumulative[b]-cumulative[a],guard,profile,out var audit,settings,handleScales);
            if(fit is not null)return fit;
            Reject(audit.Outcome);
            var freedom=(a==0?VCarveNativeAxialFreedom.Start:VCarveNativeAxialFreedom.None)
                |(b==points.Length-1?VCarveNativeAxialFreedom.End:VCarveNativeAxialFreedom.None);
            if(contract!=VCarveNativeSourceContract.MedialPlanarWithStockContactDepth||freedom==VCarveNativeAxialFreedom.None)return null;
            budget.CheckCandidate();
            var contact=VCarveNativeStockContactFit.Find(points[a..(b+1)],u,v,guard,profile,settings,token,out var outcome,freedom);
            if(contact is not null)return contact with{First=a,Last=b};
            Reject(outcome);return null;
        }
        try
        {
            var result=Search(knots,cumulative,settings.MinimumPrimitiveLengthMm,maximumBlockPoints,Find,token,out var stats);
            void Log(string family,int proposalCount)=>Console.WriteLine($"[VCarve.NativeMinimumPath] family={family} sourceContract={contract} axialFreedomAllowed={contract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth} preservePlanarSourceProgress={settings.PreservePlanarSourceProgress} solved={result is not null} elapsedMs={watch.Elapsed.TotalMilliseconds:F1} sourcePoints={points.Length} knots={proposalCount} visitedNodes={stats.VisitedNodes} triedEdges={stats.TriedEdges} failedNodes={stats.FailedNodes} candidateAttempts={budget.Attempts} minimumMm={settings.MinimumPrimitiveLengthMm:R} completePathRequired=True");
            Log("prepared-directions",knots.Length);
            if(result is null)
            {
                // The established repair stage nominates source positions near
                // one/two minimum lengths around an unresolved short. Include
                // those positions in the COMPLETE path search too. Interior
                // free joins use one shared spatial direction on both sides;
                // true block endpoint directions remain exactly prepared.
                var proposals=new SortedSet<int>(knots);double minimum=settings.MinimumPrimitiveLengthMm;
                foreach(var shortFit in diagnosticSource.Where(f=>f.LengthLower<minimum))
                foreach(double target in new[]{cumulative[shortFit.First]-minimum,cumulative[shortFit.First]-2*minimum,
                    cumulative[shortFit.Last]+minimum,cumulative[shortFit.Last]+2*minimum})
                {
                    int near=Array.BinarySearch(cumulative,target);if(near<0)near=~near;
                    for(int delta=-1;delta<=1;delta++)if(near+delta>0&&near+delta<points.Length-1)proposals.Add(near+delta);
                }
                var expanded=proposals.ToArray();index=expanded.Select((point,i)=>(point,i)).ToDictionary(x=>x.point,x=>x.i);
                sharedJoins=true;
                result=Search(expanded,cumulative,minimum,maximumBlockPoints,Find,token,out stats);
                Log("shared-free-joins",expanded.Length);
                if(result is null)
                {
                    planarJoins=true;
                    result=Search(expanded,cumulative,minimum,maximumBlockPoints,Find,token,out stats);
                    Log("shared-planar-free-joins",expanded.Length);
                }
                // A steep contact transition can require the literal incoming
                // or outgoing source chord at a free join. An averaged tangent
                // may cross protected stock even when both source legs fit.
                // Each family uses ONE direction at each join on BOTH incident
                // moves. True block endpoints, the candidate budget and every
                // continuous source/cutter certificate remain unchanged.
                foreach(int side in new[]{1,-1})
                {
                    if(result is not null)break;
                    sourceSide=side;planarJoins=false;
                    result=Search(expanded,cumulative,minimum,maximumBlockPoints,Find,token,out stats);
                    Log(side>0?"shared-outgoing-source-joins":"shared-incoming-source-joins",expanded.Length);
                }
                if(result is null)
                {
                    // Contact refinement can split a nearly axial endpoint leg
                    // into several tiny planar samples. Its complete chord is
                    // another shared tangent proposal, with the same geometric
                    // target as nomination reach and no source replacement.
                    sourceSide=0;endpointChords=true;
                    result=Search(expanded,cumulative,minimum,maximumBlockPoints,Find,token,out stats);
                    Log("shared-endpoint-chord-joins",expanded.Length);
                }
                if(result is null)
                {
                    // A nearly axial derivative may need a longer handle to
                    // finish withdrawing before XY crosses a stock shoulder.
                    // Keep every previously certified construction first. This
                    // bounded inventory is used only after all ordinary shared
                    // join families fail; it grants no fit/clearance credit.
                    handleScales=[1,.85,1.15,.7,1.3,.5,.25,.1,.03,.01,.003,2,3,4];
                    result=Search(expanded,cumulative,minimum,maximumBlockPoints,Find,token,out stats);
                    Log("shared-endpoint-chord-long-handles",expanded.Length);
                }
            }
            exhausted=false;searchRejections=failures;
            if(result is not null)return result;
            Reject("minimum-length-construction-unresolved");
        }
        catch(VCarveNativeBudgetException)
        {
            exhausted=true;searchRejections=failures;Reject("minimum-length-construction-budget-exhausted");
            Console.WriteLine($"[VCarve.NativeMinimumPath] solved=False budgetExhausted=True elapsedMs={watch.Elapsed.TotalMilliseconds:F1} sourcePoints={points.Length} knots={knots.Length} candidateAttempts={budget.Attempts}");
        }
        // Diagnostic reference only. Strict transport still rejects unresolved
        // shorts/unverified geometry; no partial constructed prefix is emitted.
        return diagnosticSource.ToList();
    }
}
