using Clipper2Lib;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Tests;
public sealed class VCarveNativeReviewedMappingTests
{
 static readonly IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> Boundaries=[new IReadOnlyList<PointD>[]{new PointD[]{new(-5,-5),new(5,-5),new(5,5),new(-5,5)}}];
 static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Linear,.1,.1,2)],2);
 static TaggedToolpath Path(Point3D[] p)=>new(p,[],0,"immutable-reviewed",0,detectCurves:false);
 [Fact]
 public void PreparedKnotsRemainProposalsAndLongStraightUsesOneNativeLine()
 {
  var p=Enumerable.Range(0,101).Select(i=>new Point3D(i/100d,0,-1)).ToArray();
  var result=VCarveNativeCurvePlanning.Nominate([Path(p)],Boundaries,Profile,0,selectedSourceOrdinals:[new[]{0,50,100}],requiredSourceOrdinals:[new[]{0,100}]).Single();
  Assert.True(result.CandidateVerified);var line=Assert.Single(result.Primitives);Assert.Equal(ToolMoveKind.Line,line.Move.Kind);
  Assert.Equal(0,line.SourceFirst);Assert.Equal(100,line.SourceLast);Assert.True(line.Move.MaxError<1e-12);
 }
 [Fact]
 public void PreparedShortcutCannotSpendAnotherErrorAllowance()
 {
  var bowed=Enumerable.Range(0,101).Select(i=>new Point3D(i/100d,.31*Math.Sin(i/100d*Math.PI),-1)).ToArray();
  var rejected=VCarveNativeCurvePlanning.Nominate([Path(bowed)],Boundaries,Profile,0,selectedSourceOrdinals:[new[]{0,100}]).Single();
  // Prepared knots nominate candidates; the complete search may discover a
  // legal curved path. It must still prove against the immutable bow rather
  // than spending its preparation error again on a straight shortcut.
  Assert.True(rejected.CandidateVerified);
  Assert.All(rejected.Primitives,primitive=>{
   Assert.InRange(primitive.Move.MaxError,0,.3);
   Assert.True(VCarveNativeGeometricProof.Prove(primitive.Move,bowed,primitive.SourceFirst,primitive.SourceLast,.3).Certified);
  });
  Assert.DoesNotContain(rejected.Primitives,primitive=>primitive.Move.Kind==ToolMoveKind.Line&&primitive.SourceFirst==0&&primitive.SourceLast==100);

 }
 [Fact]
 public void InvalidSourceOrdinalMapFailsExplicitly()
 {
  var p=new[]{new Point3D(0,0,-1),new Point3D(1,0,-1)};
  Assert.Throws<ArgumentException>(()=>VCarveNativeCurvePlanning.Nominate([Path(p)],Boundaries,Profile,0,selectedSourceOrdinals:[new[]{0,2}]));
 }
 [Fact]
 public void TinyBackstepUsesVerifiedNativeLineWithThePreparedDirection()
 {
  var p=new[]{new Point3D(0,0,-1),new Point3D(1,0,-1),new Point3D(.999,0,-1)};
  var result=VCarveNativeCurvePlanning.Nominate([Path(p)],Boundaries,Profile,0,settings:new(.3,.0005),selectedSourceOrdinals:[new[]{0,2}],requiredSourceOrdinals:[new[]{0,2}]).Single();
  Assert.All(result.Primitives,x=>Assert.True(x.LengthLowerMm>=.0005));
  Assert.True(result.NewCurveChecksPassed);Assert.True(result.CandidateVerified);
  Assert.Equal(ToolMoveKind.Line,Assert.Single(result.Primitives).Move.Kind);
 }
}
