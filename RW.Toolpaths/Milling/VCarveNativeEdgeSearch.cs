using RW.Toolpaths;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Milling;

internal sealed record VCarveNativeFreeJoinEndpoints(Point3D Start,Point3D End,double MinimumZ,double MaximumZ);
internal sealed class VCarveNativeEdgeAudit
{
    public int DepthRejected,OrderRejected,FitRejected,PoseRejected,LengthRejected,ContinuousRejected,ProofBudget,Qualified,LowerBoundRejected,GeometryProofs,PlanarProgressRejected;
    public string Outcome="no-candidate";
}

// Shared, region-independent candidate family. Sampling rejects; continuous
// source and cutter enclosures alone authorize a graph edge.
internal static class VCarveNativeEdgeSearch
{
    internal static VCarveNativeFit? Find(Point3D[] points,int first,int last,Point3D u,Point3D v,double sourceLength,VCarveNativeContainment guard,VCarveCutterProfile profile,out VCarveNativeEdgeAudit audit,VCarveNativeFitSettings? settings=null,IReadOnlyList<double>? handleScales=null,VCarveNativeFreeJoinEndpoints? freeJoin=null)
    {
        var watch=System.Diagnostics.Stopwatch.StartNew();
        var result=FindCore(points,first,last,u,v,sourceLength,guard,profile,out audit,settings,handleScales,freeJoin);
        if(watch.ElapsedMilliseconds>=250)
            Console.WriteLine($"[VCarve.NativeEdgeCost] elapsedMs={watch.Elapsed.TotalMilliseconds:F1} sourcePoints={last-first+1} first={first} last={last} lowerBoundRejected={audit.LowerBoundRejected} planarProgressRejected={audit.PlanarProgressRejected} geometryProofs={audit.GeometryProofs} qualified={audit.Qualified} poseRejected={audit.PoseRejected} continuousRejected={audit.ContinuousRejected} outcome={audit.Outcome}");
        return result;
    }

    private static VCarveNativeFit? FindCore(Point3D[] points,int first,int last,Point3D u,Point3D v,double sourceLength,VCarveNativeContainment guard,VCarveCutterProfile profile,out VCarveNativeEdgeAudit audit,VCarveNativeFitSettings? settings,IReadOnlyList<double>? handleScales,VCarveNativeFreeJoinEndpoints? freeJoin)
    {
        settings??=new();audit=new();var a=freeJoin?.Start??points[first];var b=freeJoin?.End??points[last];
        VCarveNativeGeometryCertificate Prove(ToolMove m)=>freeJoin is null?VCarveNativeGeometricProof.Prove(m,points,first,last,settings.FitErrorMm):VCarveNativeGeometricProof.ProveFreeJoin(m,points,first,last,settings.FitErrorMm);
        if(sourceLength<settings.MinimumPrimitiveLengthMm){audit.Outcome="source-span-below-target";return null;}
        if(VCarveNativeCurveMath.Distance(a,b)<1e-9){audit.Outcome="closed-or-degenerate-endpoints";return null;}
        var chord=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(b,a));
        if(VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(u,chord))<1e-8&&VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(v,chord))<1e-8)
        {
            var line=ToolMove.Line(a,b);var certificate=Prove(line);double error=certificate.ErrorBound;
            double length=VCarveNativeCurveMath.Distance(a,b);
            if(certificate.Certified&&length>=settings.MinimumPrimitiveLengthMm&&guard.Legal(a,b))
            {audit.Outcome="certified-line";return new(line with{MaxError=error},first,last,sourceLength,length,length,0,0);}
        }
        // A two-point source owns a complete straight chord. Prescribed join
        // directions can still require a cubic approximating that chord. The
        // default handle seed below covers the underdetermined least-squares
        // case; the same full continuous proof remains mandatory.
        var ts=VCarveNativeCurveMath.Parameters(points,first,last);double aa=0,ab=0,bb=0,ar=0,br=0;
        for(int i=1;i<ts.Length-1;i++)
        {
            double t=ts[i],s=1-t,k1=3*s*s*t,k2=-3*s*t*t;
            var basis=VCarveNativeCurveMath.Add(VCarveNativeCurveMath.Mul(a,1-3*t*t+2*t*t*t),VCarveNativeCurveMath.Mul(b,3*t*t-2*t*t*t));var rhs=VCarveNativeCurveMath.Sub(points[first+i],basis);
            aa+=k1*k1;ab+=k1*k2*VCarveNativeCurveMath.Dot(u,v);bb+=k2*k2;ar+=k1*VCarveNativeCurveMath.Dot(u,rhs);br+=k2*VCarveNativeCurveMath.Dot(v,rhs);
        }
        var seeds=new List<(double Alpha,double Beta)>();double det=aa*bb-ab*ab;
        if(det>1e-20){double alpha=(ar*bb-br*ab)/det,beta=(br*aa-ar*ab)/det;if(alpha>1e-10&&beta>1e-10)seeds.Add((alpha,beta));}
        seeds.Add((sourceLength/3,sourceLength/3));
        double low=double.PositiveInfinity,high=double.NegativeInfinity;for(int i=first;i<=last;i++){low=Math.Min(low,points[i].Z);high=Math.Max(high,points[i].Z);}
        if(freeJoin is not null){low=freeJoin.MinimumZ;high=freeJoin.MaximumZ;}
        // Dimensionless handle nominations only. Small handles preserve the
        // exact endpoint direction while allowing a sharp local turn to fit;
        // minimum primitive length and every acceptance bound stay unchanged.
        var scales=handleScales??new[]{1d,.85,1.15,.7,1.3,.5,.25,.1,.03,.01,.003};
        var choices=new List<(ToolMove Move,double Margin)>();
        var planarProgress=settings.PreservePlanarSourceProgress?VCarveNativePlanarProgress.Create(points,first,last):null;
        foreach(var (alpha,beta) in seeds)foreach(double sa in scales)foreach(double sb in scales)
        {
            guard.Token.ThrowIfCancellationRequested();
            double handleA=alpha*sa,handleB=beta*sb;
            if(planarProgress is not null&&!planarProgress.TrySelectHandles(a,b,u,v,ref handleA,ref handleB)){audit.PlanarProgressRejected++;continue;}
            var m=ToolMove.CubicBezier(a,VCarveNativeCurveMath.Add(a,VCarveNativeCurveMath.Mul(u,handleA)),VCarveNativeCurveMath.Sub(b,VCarveNativeCurveMath.Mul(v,handleB)),b,0);
            var z=VCarveNativeCurveMath.ZRange(m);if(z.Min<low-1e-9||z.Max>high+1e-9){audit.DepthRejected++;continue;}
            if(VCarveNativeGeometryRejection.Excluded(m,points,first,last,settings.FitErrorMm)){audit.LowerBoundRejected++;continue;}
            audit.GeometryProofs++;
            var certificate=Prove(m);
            if(!certificate.Certified){if(certificate.Reason.StartsWith("curve-",StringComparison.Ordinal))audit.OrderRejected++;else audit.FitRejected++;continue;}
            double error=certificate.ErrorBound;
            audit.Qualified++;m=m with{MaxError=error};
            // A stock certificate has boolean margin zero. Keep the identical
            // error ordering, but do not query poses of candidates never tried.
            if(guard.StockAware){choices.Add((m,0));continue;}
            double margin=double.PositiveInfinity;
            foreach(double t in Enumerable.Range(0,17).Select(i=>i/16d)){var p=VCarveNativeCurveMath.At(m,t);margin=Math.Min(margin,guard.MinimumSquaredSeparation(p,p));if(margin<-1e-10)break;}
            if(margin<-1e-10){audit.PoseRejected++;continue;}
            choices.Add((m,margin));
        }
        int poseQualified=0;
        foreach(var item in choices.OrderByDescending(x=>x.Margin).ThenBy(x=>x.Move.MaxError))
        {
            if(guard.StockAware && !Enumerable.Range(0,17).All(i=>{var q=VCarveNativeCurveMath.At(item.Move,i/16d);return guard.Legal(q,q);})){audit.PoseRejected++;continue;}
            poseQualified++;
            var length=VCarveNativeCurveMath.Length(item.Move);if(length.Lower<settings.MinimumPrimitiveLengthMm){audit.LengthRejected++;continue;}
            if(!VCarveNativeCutterProof.Certified(item.Move,guard,profile,out var reason,9,2048)){audit.ContinuousRejected++;if(reason=="continuous-proof-budget")audit.ProofBudget++;continue;}
            audit.Outcome="certified-cubic";return new(item.Move,first,last,sourceLength,length.Lower,length.Upper,0,0);
        }
        audit.Outcome=audit.Qualified==0?"no-geometry-depth-order-fit-in-family":poseQualified==0?(guard.StockAware?"stock-pose-not-certified":"all-qualified-candidates-have-actual-pose-violation"):audit.ProofBudget>0?"uncertified-continuous-alternative":"continuous-cutter-or-length-rejection";
        return null;
    }
}
