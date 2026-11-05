namespace RW.Toolpaths.Milling;

// Native fitting for a complete branch of the derived depth-limit contour.
// The caller owns the branch's exact topological endpoints. This does not apply
// at a degree-two smooth join, to authored spatial tangents, or away from the cap.
internal static class VCarveNativeDepthLimitFit
{
 internal static bool IsDepthLimitBranch(IReadOnlyList<Point3D> source,VCarveCutterProfile profile)
  =>source.Count>=2&&source.All(p=>p.Z==-profile.MaximumDepth);
 internal static VCarveNativeFit? Line(Point3D[] source,VCarveNativeContainment guard,
  VCarveCutterProfile profile,VCarveNativeFitSettings settings)
 {
  if(!guard.StockAware||!IsDepthLimitBranch(source,profile))return null;
  var move=ToolMove.Line(source[0],source[^1]);
  double length=VCarveNativeCurveMath.Distance(move.Start,move.End);
  if(length<settings.MinimumPrimitiveLengthMm)return null;
  var proof=VCarveNativeGeometricProof.Prove(move,source,0,source.Length-1,settings.FitErrorMm);
  if(!proof.Certified||!guard.Legal(move.Start,move.End))return null;
  double sourceLength=Enumerable.Range(1,source.Length-1).Sum(i=>VCarveNativeCurveMath.Distance(source[i-1],source[i]));
  return new(move with{MaxError=proof.ErrorBound},0,source.Length-1,sourceLength,length,length,0,0){DepthLimitContour=true};
 }
}
