using Clipper2Lib;
namespace RW.Toolpaths.Milling.ReviewedSource;
public static class ReviewedVCarvePlan
{
 public static ToolpathPlan AttachClearance(IReadOnlyList<TaggedToolpath> raw,IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> boundaries,
  VCarveCutterProfile profile,DepthSchedule depth,VCarveClearedStock? stock,CancellationToken token=default,Action<string>? diagnostic=null)
 {
  var owner=stock is null?null:new VCarveMaterialSurface(boundaries.SelectMany(r=>r),stock,profile,depth.Depth,depth.SurfaceZ,token,diagnostic);
  return AttachClearance(raw,boundaries,profile,depth.SurfaceZ,owner,token,diagnostic,reusedAuthority:false);
 }

 internal static ToolpathPlan AttachClearance(IReadOnlyList<TaggedToolpath> raw,IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> boundaries,
  VCarveCutterProfile profile,double surface,VCarveMaterialSurface? owner,CancellationToken token,Action<string>? diagnostic,bool reusedAuthority)
 {
  // Actual stock already owns the complete target. Building separate solid-stock
  // guards here cannot contribute to that branch and duplicates its indexes.
  VCarveNativeContainment[] guards=owner is null?boundaries.Select(b=>new VCarveNativeContainment(b,profile,token)).ToArray():[];
  owner?.ReportClearanceAttachment(diagnostic,reusedAuthority);
  Point3D Relative(Point3D p)=>p with{Z=p.Z-surface};
  bool NativeChord(Point3D a,Point3D b)=>owner is not null?owner.IsLegalNativeChord(Relative(a),Relative(b)):guards.Any(g=>g.Legal(Relative(a),Relative(b)));
  bool NativeCurve(ToolMove m)
  {
   var r=VCarveNativeCurveMath.TranslateZ(m,-surface);
   return owner is not null?owner.IsLegalNativeCurve(r):guards.Any(g=>VCarveNativeCutterProof.Certified(r,g,profile,out _));
  }
  VCarveNativeQueryWorker QueryWorker()
  {
   var worker=owner!.CreateNativeQueryWorker(out var startup);
   return new((a,b)=>worker.IsLegalNativeChord(Relative(a),Relative(b)),
       move=>worker.IsLegalNativeCurve(VCarveNativeCurveMath.TranslateZ(move,-surface)))
   {
    ImportedPositiveSweepProofs=startup.SweepProofs,ImportedPositivePoseProofs=startup.PoseProofs,
    ImportedPositiveLayerPoses=startup.LayerPoses,EstimatedImportedProofBytes=startup.EstimatedBytes,
    ImportedStockCellFacts=startup.StockCellFacts,EstimatedImportedStockCellBytes=startup.StockCellBytes,
    StartupMilliseconds=startup.Milliseconds,ImmutableDomainProvenance=startup.Domain,
    DiagnosticCounters=()=>worker.ProfileDepthDiagnostics
   };
  }
  return new(raw.ToList(),[],new())
  {
   NativeReplacementChordGuard=NativeChord,NativeReplacementCurveGuard=NativeCurve,
   NativeReplacementQueryWorkerFactory=owner is null?null:QueryWorker,
   ReplacementChordGuard=(a,b)=>owner is not null?owner.IsLegalChord(Relative(a),Relative(b)):NativeChord(a,b)
  };
 }
}
