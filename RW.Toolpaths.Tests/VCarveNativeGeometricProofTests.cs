using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Tests;
public sealed class VCarveNativeGeometricProofTests
{
 static Point3D P(double x,double y=0,double z=0)=>new(x,y,z);
 [Fact] public void TinyRawReversalMayPauseWithoutReversingTheNativeLine()
 {
  var source=new[]{P(0),P(1),P(.999)};var proof=VCarveNativeGeometricProof.Prove(ToolMove.Line(source[0],source[^1]),source,0,2,.3);
  Assert.True(proof.Certified);Assert.InRange(proof.ErrorBound,.001,.00100001);Assert.Equal(proof.Parameters[1],proof.Parameters[2]);
 }
 [Fact] public void LargeReversalCannotBeErasedByAForwardShortcut()
 {
  var source=new[]{P(0),P(2),P(.2),P(3)};var proof=VCarveNativeGeometricProof.Prove(ToolMove.Line(source[0],source[^1]),source,0,3,.3);
  Assert.False(proof.Certified);Assert.True(proof.ErrorBound>.3);
 }
 [Theory] [InlineData(.39,true)] [InlineData(.41,false)]
 public void InteriorBowIsBoundedContinuouslyDespiteZeroEndpointError(double bow,bool accepted)
 {
  var source=new[]{P(0),P(1)};var m=ToolMove.CubicBezier(source[0],P(1d/3,bow),P(2d/3,bow),source[1],0);
  Assert.Equal(accepted,VCarveNativeGeometricProof.Prove(m,source,0,1,.3).Certified);
 }
 [Fact] public void AddedLoopIsRejectedEvenWithALargeGeometricAllowance()
 {
  var source=new[]{P(0),P(1)};var m=ToolMove.CubicBezier(source[0],P(3,2),P(-2,-2),source[1],0);
  var proof=VCarveNativeGeometricProof.Prove(m,source,0,1,10);Assert.False(proof.Certified);Assert.Equal("curve-progress-not-certified",proof.Reason);
 }
 [Fact] public void ExplicitCornerAndMotionStateRemainSeparateNativePieces()
 {
  var p=new[]{P(0,0,-1),P(1,0,-1),P(1,1,-1)};var path=new TaggedToolpath(p,[new(0,1,ToolpathSpanKind.Cut),new(1,2,ToolpathSpanKind.Link)],0,"authored-corner",0,detectCurves:false);
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],2);
  var plan=VCarveNativeCurvePlanning.Nominate([path],[[new PointD[]{new(-5,-5),new(5,-5),new(5,5),new(-5,5)}]],profile,0,selectedSourceOrdinals:[new[]{0,1,2}],requiredSourceOrdinals:[new[]{0,1,2}]).Single();
  Assert.True(plan.CandidateVerified);Assert.Equal(2,plan.Primitives.Count);Assert.Equal(p[1],plan.Primitives[0].Move.End);Assert.Equal(p[1],plan.Primitives[1].Move.Start);
  Assert.Equal(ToolpathSpanKind.Cut,plan.Primitives[0].Kind);Assert.Equal(ToolpathSpanKind.Link,plan.Primitives[1].Kind);
 }
 [Fact] public void ABranchAttachmentCannotSwitchToTheNearbyOtherBranch()
 {
  var a=new[]{P(0,0,-1),P(1,0,-1),P(2,0,-1)};var b=new[]{P(1,0,-1),P(1,1,-1)};
  var paths=new[]{new TaggedToolpath(a,[],0,"trunk",0,detectCurves:false),new TaggedToolpath(b,[],0,"branch",0,detectCurves:false)};
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],2);
  var plans=VCarveNativeCurvePlanning.Nominate(paths,[[new PointD[]{new(-5,-5),new(5,-5),new(5,5),new(-5,5)}]],profile,0,selectedSourceOrdinals:[new[]{0,1,2},new[]{0,1}]);
  Assert.Equal(a[1],plans[0].Primitives[0].Move.End);Assert.Equal(a[1],plans[1].Primitives[0].Move.Start);Assert.All(plans,p=>Assert.True(p.CandidateVerified));
 }
 [Fact] public void StationaryCuspCannotBeHiddenInsideASmoothNativeCubic()
 {
  var source=new[]{P(-.125,.25),P(.125,.25)};var m=ToolMove.CubicBezier(source[0],P(.125,-1d/12),P(-.125,-1d/12),source[1],0);
  var proof=VCarveNativeGeometricProof.Prove(m,source,0,1,10);Assert.False(proof.Certified);Assert.Equal("curve-regularity-not-certified",proof.Reason);
 }
 [Fact] public void InteriorPoseIsExactAndOrderedWithoutBecomingAPrimitiveEnd()
 {
  var a=P(0);var b=P(0,1,10);var m=ToolMove.CubicBezier(a,P(.4,.5,10*.04/(3*.64)),P(.2,0,-2*10*.2/(3*.8)),b,0);
  var source=Enumerable.Range(0,101).Select(i=>VCarveNativeCurveMath.At(m,i/100d)).ToArray();
  var proof=VCarveNativeGeometricProof.Prove(m,source,0,100,.3,[new(20,.2)]);Assert.True(proof.Certified,proof.Reason);Assert.Equal(.2,proof.Parameters[20]);
  Assert.False(VCarveNativeGeometricProof.Prove(m,source,0,100,.3,[new(20,.3)]).Certified);
  Assert.False(VCarveNativeGeometricProof.Prove(m,source,0,100,.3,[new(30,.3),new(20,.2)]).Certified);
 }
 sealed record Recorded(int Region,Point3D[] Source,ToolMove Move);
 [Fact] public void FourCapturedCurvesPassGeometricErrorWithoutTheRejectedTimingAssumption()
 {
  using var stream=typeof(VCarveNativeGeometricProofTests).Assembly.GetManifestResourceStream("RW.Toolpaths.Tests.Fixtures.reviewed-geometric-correspondence.json")!;
  var cases=JsonSerializer.Deserialize<Recorded[]>(stream,new JsonSerializerOptions{IncludeFields=true})!;Assert.Equal(4,cases.Length);
  foreach(var item in cases)
  {
   var original=item.Source.ToArray();var proof=VCarveNativeGeometricProof.Prove(item.Move,item.Source,0,item.Source.Length-1,.3);
   Assert.True(proof.Certified,$"region={item.Region} reason={proof.Reason} bound={proof.ErrorBound:R}");Assert.True(proof.ErrorBound<=.3);Assert.Equal(original,item.Source);
   var fixedTiming=VCarveNativeCurveMath.Parameters(item.Source,0,item.Source.Length-1);
   Assert.True(VCarveNativeCurveMath.ContinuousBound(item.Move,item.Source,0,item.Source.Length-1,fixedTiming,.3)>.3);
  }
 }
}
