namespace RW.Toolpaths.Milling;
public static partial class VCarveNativeCurvePlanning
{
    private static List<VCarveNativeFit> FitPreparedSourceSpans(Point3D[] points,int[] knots,VCarveNativeFit[] original,
        VCarveNativeContainment guard,VCarveCutterProfile profile,VCarveNativeSearchBudget budget,
        VCarveNativeFitSettings settings,int maximumBlockPoints,CancellationToken token,out bool exhausted,out IReadOnlyDictionary<string,int> searchRejections,VCarveNativeSourceContract contract)
    {
        var result=new List<VCarveNativeFit>();var failures=new Dictionary<string,int>();bool stopped=false;
        var cum=new double[points.Length];for(int i=1;i<points.Length;i++)cum[i]=cum[i-1]+VCarveNativeCurveMath.Distance(points[i-1],points[i]);
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
