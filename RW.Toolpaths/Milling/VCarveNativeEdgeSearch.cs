using RW.Toolpaths;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Geometry;
using System.Diagnostics;

namespace RW.Toolpaths.Milling;

internal sealed record VCarveNativeFreeJoinEndpoints(Point3D Start,Point3D End,double MinimumZ,double MaximumZ);
internal sealed class VCarveNativeEdgeAudit
{
    public int DepthRejected,OrderRejected,FitRejected,PoseRejected,LengthRejected,ContinuousRejected,ProofBudget,Qualified,LowerBoundRejected,GeometryProofs,PlanarProgressRejected;
    public string Outcome="no-candidate";
    public int GeometryWorkers=1,BufferedGeometryCandidates;
    public int VisitedNominees,CandidateInventory;
    public int SourceGeometryProofAttempts,MaximumCandidateGeometryProofs=int.MaxValue,CandidateGeometryLimitRefusals;
    public bool CandidateGeometryLimitReached;
    public VCarveNativeCandidateSelection CandidateSelection;
    public double NominationMilliseconds,GeometryProofMilliseconds,StockSelectionMilliseconds;
}

// Shared, region-independent candidate family. Sampling rejects; continuous
// source and cutter enclosures alone authorize a graph edge.
internal static class VCarveNativeEdgeSearch
{
    private const int MaximumGeometryBatch=242,MinimumParallelSourcePoints=64,MinimumParallelNominees=32;
    private readonly record struct GeometryResult(bool Certified,double ErrorBound,string Reason)
    {
        internal static GeometryResult From(VCarveNativeGeometryCertificate certificate)
            =>new(certificate.Certified,certificate.ErrorBound,certificate.Reason);
    }
    internal static VCarveNativeFit? Find(Point3D[] points,int first,int last,Point3D u,Point3D v,double sourceLength,VCarveNativeContainment guard,VCarveCutterProfile profile,out VCarveNativeEdgeAudit audit,VCarveNativeFitSettings? settings=null,IReadOnlyList<double>? handleScales=null,VCarveNativeFreeJoinEndpoints? freeJoin=null,bool parallelGeometryProofs=true)
    {
        var watch=System.Diagnostics.Stopwatch.StartNew();
        var result=FindCore(points,first,last,u,v,sourceLength,guard,profile,out audit,settings,handleScales,freeJoin,parallelGeometryProofs);
        if(watch.ElapsedMilliseconds>=250||audit.CandidateSelection==VCarveNativeCandidateSelection.FirstVerified)
            Console.WriteLine($"[VCarve.NativeEdgeCost] elapsedMs={watch.Elapsed.TotalMilliseconds:F1} sourcePoints={last-first+1} first={first} last={last} lowerBoundRejected={audit.LowerBoundRejected} planarProgressRejected={audit.PlanarProgressRejected} geometryProofs={audit.GeometryProofs} qualified={audit.Qualified} poseRejected={audit.PoseRejected} continuousRejected={audit.ContinuousRejected} outcome={audit.Outcome} "
                +$"candidateSelection={audit.CandidateSelection} visitedNominees={audit.VisitedNominees} candidateInventory={audit.CandidateInventory} geometryWorkers={audit.GeometryWorkers} bufferedGeometryCandidates={audit.BufferedGeometryCandidates} nominationMs={audit.NominationMilliseconds:F3} geometryProofWallMs={audit.GeometryProofMilliseconds:F3} stockSelectionMs={audit.StockSelectionMilliseconds:F3} stockCallbacks=serial candidateOrder={(audit.CandidateSelection==VCarveNativeCandidateSelection.FirstVerified?"seed-scale":"clearance-error")} "
                +$"maximumCandidateGeometryProofs={audit.MaximumCandidateGeometryProofs} sourceGeometryProofAttempts={audit.SourceGeometryProofAttempts} limitReached={audit.CandidateGeometryLimitReached} limitRefusals={audit.CandidateGeometryLimitRefusals} effortScope=one-edge-Find onLimit=ordinary-subdivision-fallback physicalProofsUnchanged=True");
        return result;
    }

    private static VCarveNativeFit? FindCore(Point3D[] points,int first,int last,Point3D u,Point3D v,double sourceLength,VCarveNativeContainment guard,VCarveCutterProfile profile,out VCarveNativeEdgeAudit audit,VCarveNativeFitSettings? settings,IReadOnlyList<double>? handleScales,VCarveNativeFreeJoinEndpoints? freeJoin,bool parallelGeometryProofs)
    {
        settings??=new();settings.Validate();
        bool firstVerified=settings.CandidateSelection==VCarveNativeCandidateSelection.FirstVerified;
        audit=new(){CandidateSelection=settings.CandidateSelection,
            MaximumCandidateGeometryProofs=firstVerified?settings.MaximumCandidateGeometryProofs:int.MaxValue};
        var a=freeJoin?.Start??points[first];var b=freeJoin?.End??points[last];
        var report=audit;
        bool CandidateLimitReached()
        {
            if(!firstVerified||report.SourceGeometryProofAttempts<report.MaximumCandidateGeometryProofs)return false;
            guard.Token.ThrowIfCancellationRequested();
            report.CandidateGeometryLimitReached=true;report.CandidateGeometryLimitRefusals++;
            report.Outcome="candidate-geometry-proof-limit";
            return true;
        }
        VCarveNativeGeometryCertificate RawProof(ToolMove m)=>VCarveNativeGeometricProof.ProveOwned(m,points,first,last,
            settings.FitErrorMm,settings.OwnedMinimaxSource,freeJoin is not null);
        GeometryResult Prove(ToolMove move)
        {
            long started=Stopwatch.GetTimestamp();
            report.SourceGeometryProofAttempts++;
            var certificate=GeometryResult.From(RawProof(move));
            report.GeometryProofMilliseconds+=Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            return certificate;
        }
        if(sourceLength<settings.MinimumPrimitiveLengthMm){audit.Outcome="source-span-below-target";return null;}
        if(VCarveNativeCurveMath.Distance(a,b)<1e-9){audit.Outcome="closed-or-degenerate-endpoints";return null;}
        var chord=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(b,a));
        if(VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(u,chord))<1e-8&&VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(v,chord))<1e-8)
        {
            var line=ToolMove.Line(a,b);var certificate=Prove(line);double error=certificate.ErrorBound;
            double length=VCarveNativeCurveMath.Distance(a,b);
            if(certificate.Certified&&length>=settings.MinimumPrimitiveLengthMm)
            {
                long stockStarted=Stopwatch.GetTimestamp();bool legal=guard.Legal(a,b);
                report.StockSelectionMilliseconds+=Stopwatch.GetElapsedTime(stockStarted).TotalMilliseconds;
                if(legal){audit.Outcome="certified-line";return new(line with{MaxError=error},first,last,sourceLength,length,length,0,0);}
            }
            if(CandidateLimitReached())return null;
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
        audit.CandidateInventory=seeds.Count*scales.Count*scales.Count;
        var choices=new List<(ToolMove Move,double Margin)>();
        var planarProgress=settings.PreservePlanarSourceProgress?VCarveNativePlanarProgress.CreateAdmission(points,first,last):null;
        int geometryWorkers=!firstVerified&&parallelGeometryProofs&&last-first+1>=MinimumParallelSourcePoints?GeometryParallelism.WorkerLimit:1;
        var nominees=geometryWorkers>1?new List<ToolMove>(MaximumGeometryBatch):null;
        int firstPoseQualified=0;
        VCarveNativeFit? VerifyFirst(ToolMove move,GeometryResult certificate)
        {
            if(!certificate.Certified){if(certificate.Reason.StartsWith("curve-",StringComparison.Ordinal))report.OrderRejected++;else report.FitRejected++;return null;}
            report.Qualified++;move=move with{MaxError=certificate.ErrorBound};
            long stockStarted=Stopwatch.GetTimestamp();
            try
            {
                // Candidate preference changes only when to stop searching.
                // Every accepted move retains all existing pose, actual-length
                // and complete continuous physical-cutter requirements.
                bool legalPoses=true;
                foreach(double t in Enumerable.Range(0,17).Select(i=>i/16d))
                {
                    var point=VCarveNativeCurveMath.At(move,t);
                    if(guard.StockAware?!guard.Legal(point,point):guard.MinimumSquaredSeparation(point,point)<-1e-10)
                    {legalPoses=false;break;}
                }
                if(!legalPoses){report.PoseRejected++;return null;}
                firstPoseQualified++;
                var length=VCarveNativeCurveMath.Length(move);
                if(length.Lower<settings.MinimumPrimitiveLengthMm){report.LengthRejected++;return null;}
                if(!VCarveNativeCutterProof.Certified(move,guard,profile,out var reason,9,2048))
                {report.ContinuousRejected++;if(reason=="continuous-proof-budget")report.ProofBudget++;return null;}
                return new(move,first,last,sourceLength,length.Lower,length.Upper,0,0);
            }
            finally{report.StockSelectionMilliseconds+=Stopwatch.GetElapsedTime(stockStarted).TotalMilliseconds;}
        }
        void KeepCertified(ToolMove move,GeometryResult certificate)
        {
            if(!certificate.Certified){if(certificate.Reason.StartsWith("curve-",StringComparison.Ordinal))report.OrderRejected++;else report.FitRejected++;return;}
            report.Qualified++;move=move with{MaxError=certificate.ErrorBound};
            // These callbacks remain on the calling thread in the identical
            // nomination order, even when source-only proofs ran in parallel.
            if(guard.StockAware){choices.Add((move,0));return;}
            long stockStarted=Stopwatch.GetTimestamp();
            double margin=double.PositiveInfinity;
            foreach(double t in Enumerable.Range(0,17).Select(i=>i/16d)){var point=VCarveNativeCurveMath.At(move,t);margin=Math.Min(margin,guard.MinimumSquaredSeparation(point,point));if(margin<-1e-10)break;}
            report.StockSelectionMilliseconds+=Stopwatch.GetElapsedTime(stockStarted).TotalMilliseconds;
            if(margin<-1e-10){report.PoseRejected++;return;}
            choices.Add((move,margin));
        }
        long nominationStarted=Stopwatch.GetTimestamp();
        double geometryBefore=report.GeometryProofMilliseconds,stockBefore=report.StockSelectionMilliseconds;
        void FinishNominationTiming()=>report.NominationMilliseconds=Stopwatch.GetElapsedTime(nominationStarted).TotalMilliseconds
            -(report.GeometryProofMilliseconds-geometryBefore)-(report.StockSelectionMilliseconds-stockBefore);
        foreach(var (alpha,beta) in seeds)foreach(double sa in scales)foreach(double sb in scales)
        {
            guard.Token.ThrowIfCancellationRequested();
            audit.VisitedNominees++;
            double handleA=alpha*sa,handleB=beta*sb;
            if(planarProgress is not null&&!planarProgress.TrySelectHandles(a,b,u,v,ref handleA,ref handleB)){audit.PlanarProgressRejected++;continue;}
            var m=ToolMove.CubicBezier(a,VCarveNativeCurveMath.Add(a,VCarveNativeCurveMath.Mul(u,handleA)),VCarveNativeCurveMath.Sub(b,VCarveNativeCurveMath.Mul(v,handleB)),b,0);
            // The handle-domain calculation precedes rounded control-point
            // construction. Admit the literal move using the same predicate as
            // the final full-source audit, in both cached occurrence directions.
            if(planarProgress is not null&&!planarProgress.Preserved(m,out _)){audit.PlanarProgressRejected++;continue;}
            var z=VCarveNativeCurveMath.ZRange(m);if(z.Min<low-1e-9||z.Max>high+1e-9){audit.DepthRejected++;continue;}
            if(VCarveNativeGeometryRejection.Excluded(m,points,first,last,settings.FitErrorMm)){audit.LowerBoundRejected++;continue;}
            if(nominees is not null){nominees.Add(m);continue;}
            if(CandidateLimitReached()){FinishNominationTiming();return null;}
            audit.GeometryProofs++;
            var certificate=Prove(m);
            if(firstVerified)
            {
                var verified=VerifyFirst(m,certificate);
                if(verified is null)
                {
                    if(CandidateLimitReached()){FinishNominationTiming();return null;}
                    continue;
                }
                audit.Outcome="certified-cubic";FinishNominationTiming();return verified;
            }
            KeepCertified(m,certificate);
        }
        FinishNominationTiming();
        if(firstVerified)
        {
            audit.Outcome=audit.Qualified==0?"no-geometry-depth-order-fit-in-family":firstPoseQualified==0?(guard.StockAware?"stock-pose-not-certified":"all-qualified-candidates-have-actual-pose-violation"):audit.ProofBudget>0?"uncertified-continuous-alternative":"continuous-cutter-or-length-rejection";
            return null;
        }
        if(nominees is not null)
        {
            report.BufferedGeometryCandidates=nominees.Count;
            if(nominees.Count<MinimumParallelNominees)
                foreach(var move in nominees){guard.Token.ThrowIfCancellationRequested();report.GeometryProofs++;KeepCertified(move,Prove(move));}
            else
            {
                // The source proof is pure: no target/stock caches, callbacks
                // or candidate-budget state is read or published by workers.
                // Extended handle families keep all nominations in bounded
                // batches; certificate parameter arrays are not retained.
                for(int batch=0;batch<nominees.Count;batch+=MaximumGeometryBatch)
                {
                    guard.Token.ThrowIfCancellationRequested();
                    int start=batch,count=Math.Min(MaximumGeometryBatch,nominees.Count-batch);
                    var certificates=new GeometryResult[count];
                    int workers=Math.Min(geometryWorkers,count);report.GeometryWorkers=Math.Max(report.GeometryWorkers,workers);
                    report.GeometryProofs+=count;
                    report.SourceGeometryProofAttempts+=count;
                    long geometryStarted=Stopwatch.GetTimestamp();
                    Parallel.For(0,workers,new ParallelOptions{CancellationToken=guard.Token,MaxDegreeOfParallelism=workers},worker=>
                    {
                        using var scope=GeometryParallelism.EnterWorker();
                        for(int index=worker;index<count;index+=workers)
                        {
                            guard.Token.ThrowIfCancellationRequested();
                            certificates[index]=GeometryResult.From(RawProof(nominees[start+index]));
                        }
                    });
                    report.GeometryProofMilliseconds+=Stopwatch.GetElapsedTime(geometryStarted).TotalMilliseconds;
                    guard.Token.ThrowIfCancellationRequested();
                    for(int index=0;index<count;index++)
                    {
                        guard.Token.ThrowIfCancellationRequested();
                        KeepCertified(nominees[start+index],certificates[index]);
                    }
                }
            }
        }
        int poseQualified=0;
        long selectionStarted=Stopwatch.GetTimestamp();
        try
        {
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
        finally{report.StockSelectionMilliseconds+=Stopwatch.GetElapsedTime(selectionStarted).TotalMilliseconds;}
    }
}
