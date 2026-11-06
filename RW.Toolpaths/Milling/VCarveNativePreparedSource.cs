namespace RW.Toolpaths.Milling;
public static partial class VCarveNativeCurvePlanning
{
    private static List<VCarveNativeFit> FitPreparedSourceSpans(Point3D[] points,int[] knots,VCarveNativeFit[] original,
        VCarveNativeContainment guard,VCarveCutterProfile profile,VCarveNativeSearchBudget budget,
        VCarveNativeFitSettings settings,int maximumBlockPoints,CancellationToken token,out bool exhausted,out IReadOnlyDictionary<string,int> searchRejections,VCarveNativeSourceContract contract)
    {
        var result=new List<VCarveNativeFit>();var failures=new Dictionary<string,int>();bool stopped=false;
        var cum=new double[points.Length];for(int i=1;i<points.Length;i++)cum[i]=cum[i-1]+VCarveNativeCurveMath.Distance(points[i-1],points[i]);
        if(settings.CandidateSelection==VCarveNativeCandidateSelection.FirstVerified&&budget.FastProbeLimit>0)
        {
            var watch=System.Diagnostics.Stopwatch.StartNew();
            var fast=VCarveNativePreparedSourceProbe.Search(points,knots,cum,settings.MinimumPrimitiveLengthMm,
                settings.FitErrorMm,maximumBlockPoints,(at,stop)=>{
                    int a=knots[at],b=knots[stop];
                    var u=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[knots[at+1]],points[a]));
                    var v=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[b],points[knots[stop-1]]));
                    return VCarveNativeEdgeSearch.Find(points,a,b,u,v,cum[b]-cum[a],guard,profile,out _,settings:settings);
                },budget.TryCheckFastCandidate,token,out var probe);
            Console.WriteLine($"[VCarve.NativePreparedProbe] sourcePoints={points.Length} knots={knots.Length} attemptedEdges={probe.AttemptedEdges} provisionalEdges={probe.ProvisionalEdges} maximumBlockAttempts={probe.MaximumEdges} maximumEndpointsPerStart=5 outcome={probe.Outcome} accepted={fast is not null} "
                +$"elapsedMs={watch.Elapsed.TotalMilliseconds:F3} globalFastAttempts={budget.FastProbeAttempts} globalFastLimit={budget.FastProbeLimit} globalCandidateAttempts={budget.Attempts} globalRemaining={budget.Remaining} incompleteFastPathDiscarded=True fallback=original-48-greedy-repair-complete-path physicalProofsUnchanged=True");
            if(fast is not null)
            {
                exhausted=false;searchRejections=failures;
                return fast;
            }
            failures["prepared-coarse:"+probe.Outcome]=1;
        }
        // The original forty-eight-endpoint greedy pass and every subsequent
        // repair/complete-path family remain the fallback without memoized fits
        // or refused candidates from the optional coarse probe.
        for(int at=0;at<knots.Length-1;)
        {
            token.ThrowIfCancellationRequested();VCarveNativeFit? best=null;int bestStop=at+1;
            if(!stopped)
            for(int stop=Math.Min(knots.Length-1,at+48);stop>at;stop--)
            {
                int a=knots[at],b=knots[stop];double length=cum[b]-cum[a];if(length<settings.MinimumPrimitiveLengthMm)continue;
                if(b-a+1>maximumBlockPoints)continue;
                try
                {
                    budget.CheckCandidate();
                    var u=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[knots[at+1]],points[a]));
                    var v=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[b],points[knots[stop-1]]));
                    var fit=VCarveNativeEdgeSearch.Find(points,a,b,u,v,length,guard,profile,out var audit,settings:settings);
                    if(fit is not null){best=fit;bestStop=stop;break;}
                    failures[audit.Outcome]=failures.GetValueOrDefault(audit.Outcome)+1;
                }
                catch(VCarveNativeBudgetException){stopped=true;break;}
            }
            if(best is not null)result.Add(best);
            else for(int i=knots[at];i<knots[at+1];i++)result.Add(original[i]);
            at=bestStop;
        }
        if(!stopped&&result.Any(f=>f.LengthLower<settings.MinimumPrimitiveLengthMm))result=VCarveNativeShortSpanRepair.Repair(points,result,knots,guard,profile,budget,settings,maximumBlockPoints,token,failures,out stopped,
            VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[knots[1]],points[0])),
            VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[^1],points[knots[^2]])),contract);
        // Retain complete certified constructions from the established candidate
        // families. Only unresolved fragments need the complete-path search;
        // its additional candidates share the operation's remaining budget.
        if(!stopped&&result.Any(f=>f.LengthLower<settings.MinimumPrimitiveLengthMm))
        {
            // Newly constructed spatial alternatives must not add planar reversals.
            // This nomination constraint leaves completed incumbent fits unchanged.
            var minimumSettings=settings with{PreservePlanarSourceProgress=true};
            result=VCarveNativeMinimumPath.Fit(points,knots,result.ToArray(),guard,profile,budget,minimumSettings,maximumBlockPoints,token,out stopped,out var constructionRejections,contract);
            foreach(var rejection in constructionRejections)failures[rejection.Key]=failures.GetValueOrDefault(rejection.Key)+rejection.Value;
        }
        exhausted=stopped;searchRejections=failures;return result;
    }
}

// Nominate a COMPLETE provisional path. Every callback result still comes from
// the ordinary full edge certifier; no original short chord is admitted here.
// This is deliberately separate from MinimumPath's complete 48-knot graph.
internal static class VCarveNativePreparedSourceProbe
{
    internal sealed record Audit(int AttemptedEdges,int ProvisionalEdges,int MaximumEdges,string Outcome);
    internal static IReadOnlyList<int> Endpoints(int at,int last,int[] knots,double[] cumulative,
        double minimum,int maximumBlockPoints)
    {
        int furthest=Math.Min(last,at+48),reach=furthest-at,smallest=-1;
        for(int stop=at+1;stop<=furthest;stop++)
            if(knots[stop]-knots[at]+1<=maximumBlockPoints&&cumulative[knots[stop]]-cumulative[knots[at]]>=minimum
                &&(stop==last||cumulative[^1]-cumulative[knots[stop]]>=minimum))
            {smallest=stop;break;}
        return new[]{furthest,at+reach/2,at+reach/4,at+reach/8,smallest}
            .Where(stop=>stop>at&&stop<=furthest).Distinct().ToArray();
    }

    internal static List<VCarveNativeFit>? Search(Point3D[] points,int[] knots,double[] cumulative,
        double minimum,double fitError,int maximumBlockPoints,Func<int,int,VCarveNativeFit?> find,
        Func<bool> admitCandidate,CancellationToken token,out Audit audit,int maximumEdges=32)
    {
        if(!double.IsFinite(minimum)||minimum<=0||!double.IsFinite(fitError)||fitError<=0
            ||maximumBlockPoints<2||maximumEdges<1||knots.Length<2||knots[0]!=0||knots[^1]!=points.Length-1)
            throw new ArgumentOutOfRangeException(nameof(maximumEdges));
        int attempts=0;var result=new List<VCarveNativeFit>();
        List<VCarveNativeFit>? Stop(string outcome,out Audit report)
        {report=new(attempts,result.Count,maximumEdges,outcome);return null;}
        for(int at=0;at<knots.Length-1;)
        {
            token.ThrowIfCancellationRequested();int selected=-1;
            foreach(int stop in Endpoints(at,knots.Length-1,knots,cumulative,minimum,maximumBlockPoints))
            {
                token.ThrowIfCancellationRequested();int a=knots[at],b=knots[stop];
                if(b-a+1>maximumBlockPoints||cumulative[b]-cumulative[a]<minimum
                    ||stop<knots.Length-1&&cumulative[^1]-cumulative[b]<minimum)continue;
                if(attempts>=maximumEdges)return Stop("block-fast-attempt-limit",out audit);
                if(!admitCandidate())return Stop("shared-fast-budget-or-headroom",out audit);
                attempts++;var fit=find(at,stop);
                if(fit is null)continue;
                var u=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[knots[at+1]],points[a]));
                var v=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[b],points[knots[stop-1]]));
                double startError=VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(fit.Move,0)),u));
                double endError=VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(fit.Move,1)),v));
                if(fit.First!=a||fit.Last!=b||fit.Move.Start!=points[a]||fit.Move.End!=points[b]
                    ||!double.IsFinite(fit.LengthLower)||!double.IsFinite(fit.LengthUpper)
                    ||fit.LengthLower<minimum||fit.LengthUpper<fit.LengthLower
                    ||!double.IsFinite(fit.Move.MaxError)||fit.Move.MaxError<0||fit.Move.MaxError>fitError
                    ||!double.IsFinite(startError)||!double.IsFinite(endError)||startError>1e-8||endError>1e-8)
                    return Stop("certified-edge-contract-mismatch",out audit);
                result.Add(fit);selected=stop;break;
            }
            if(selected<0)return Stop("no-complete-coarse-path",out audit);
            at=selected;
        }
        token.ThrowIfCancellationRequested();
        audit=new(attempts,result.Count,maximumEdges,"complete-certified-coarse-path");return result;
    }
}
