using RW.Toolpaths.Milling;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class NativeFreeJoinProofTests
{
 [Fact] public void FreeJoinDisplacementIsMeasuredWithoutMovingProtectedOuterEnds()
 {
  Point3D[] source=[new(0,0,0),new(.5,0,0),new(1,0,0),new(1.5,0,0),new(2,0,0)];
  var join=new Point3D(1,0,.05);
  var left=ToolMove.CubicBezier(source[0],new(.3,0,0),new(.7,0,.05),join,0);
  var right=ToolMove.CubicBezier(join,new(1.3,0,.05),new(1.7,0,0),source[^1],0);
  Assert.False(VCarveNativeGeometricProof.Prove(left,source,0,2,.1).Certified);
  Assert.True(VCarveNativeGeometricProof.ProveFreeJoin(left,source,0,2,.1).Certified);
  Assert.True(VCarveNativeGeometricProof.ProveFreeJoin(right,source,2,4,.1).Certified);
  Assert.Equal(left.End,right.Start);Assert.Equal(source[0],left.Start);Assert.Equal(source[^1],right.End);
  Assert.Equal(VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(left,1)),VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(right,0)));
  Assert.False(VCarveNativeGeometricProof.ProveFreeJoin(left,source,0,2,.01).Certified);
 }
 [Fact] public void FreeJoinCannotHideInteriorExcursionOrBacktracking()
 {
  Point3D[] source=[new(0,0,0),new(.5,0,0),new(1,0,0)];
  var excursion=ToolMove.CubicBezier(source[0],new(.3,0,1),new(.7,0,1),new(1,0,.01),0);
  Assert.False(VCarveNativeGeometricProof.ProveFreeJoin(excursion,source,0,2,.1).Certified);
  var reverse=ToolMove.CubicBezier(source[0],new(-1,0,0),new(2,0,0),new(1,0,.01),0);
  Assert.False(VCarveNativeGeometricProof.ProveFreeJoin(reverse,source,0,2,.1).Certified);
 }
}
