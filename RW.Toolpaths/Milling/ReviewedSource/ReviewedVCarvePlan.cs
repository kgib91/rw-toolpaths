using Clipper2Lib;
namespace RW.Toolpaths.Milling.ReviewedSource;
public static class ReviewedVCarvePlan
{
 public static ToolpathPlan AttachClearance(IReadOnlyList<TaggedToolpath> raw,IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> boundaries,
  VCarveCutterProfile profile,DepthSchedule depth,VCarveClearedStock? stock,CancellationToken token=default,Action<string>? diagnostic=null)
 {
  var guards=boundaries.Select(b=>new VCarveNativeContainment(b,profile,token)).ToArray();
  var owner=stock is null?null:new VCarveMaterialSurface(boundaries.SelectMany(r=>r),stock,profile,depth.Depth,depth.SurfaceZ,token,diagnostic);
  Point3D Relative(Point3D p)=>p with{Z=p.Z-depth.SurfaceZ};
  bool NativeChord(Point3D a,Point3D b)=>owner is not null?owner.IsLegalNativeChord(Relative(a),Relative(b)):guards.Any(g=>g.Legal(Relative(a),Relative(b)));
  bool NativeCurve(ToolMove m)
  {
   var r=VCarveNativeCurveMath.TranslateZ(m,-depth.SurfaceZ);
   return owner is not null?owner.IsLegalNativeCurve(r):guards.Any(g=>VCarveNativeCutterProof.Certified(r,g,profile,out _));
  }
  return new(raw.ToList(),[],new())
  {
   NativeReplacementChordGuard=NativeChord,NativeReplacementCurveGuard=NativeCurve,
   ReplacementChordGuard=(a,b)=>owner is not null?owner.IsLegalChord(Relative(a),Relative(b)):NativeChord(a,b)
  };
 }
}
