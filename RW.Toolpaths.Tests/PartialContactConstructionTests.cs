using Clipper2Lib;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.ReviewedSource;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class PartialContactConstructionTests
{
 [Fact] public void UniformRoughingContactIsMirroredAndNeverClaimsAWholeOperation()
 {
  PointD[][][] boundaries=[[new PointD[]{new(5,5),new(15,5),new(15,15),new(5,15)}]];
  var stock=new VCarveClearedStock([new(new(0,10,-2),new(20,10,-2),20)], [2],new PointD[]{new(0,0),new(20,0),new(20,20),new(0,20)});
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,4,4)],4);var settings=new VCarvePartialContactReview.Settings(.3,.3,CapReviewExtentMm:2);
  using var bounded=new CancellationTokenSource(TimeSpan.FromSeconds(60));
  var a=VCarvePartialContactReview.Construct(boundaries,stock,profile,new(4,4){SurfaceZ=6,StartDepth=2},new(0,0,0),settings,bounded.Token);
  var b=VCarvePartialContactReview.Construct(boundaries,stock,profile,new(4,4){SurfaceZ=6,StartDepth=2},new(0,0,1),settings,bounded.Token);
  Assert.False(a.WholeOperationComplete);Assert.False(a.MachineExecutionSupported);Assert.False(b.WholeOperationComplete);
  Assert.InRange(Math.Abs(a.Root.X+b.Root.X-20),0,.001);Assert.InRange(Math.Abs(a.Root.Y-b.Root.Y),0,.001);
  Assert.All(a.Components.Concat(b.Components).Where(c=>c.Moves.Length>0),c=>{
   Assert.Equal(c.Moves[0].Start,c.Moves[^1].End);Assert.True(c.MaximumFitMm<=.3);Assert.True(c.MinimumLengthMm>=.3);
   for(int i=1;i<c.Moves.Length;i++)Assert.Equal(c.Moves[i-1].End,c.Moves[i].Start);
  });
  Assert.Contains(a.Components,c=>c.Name=="A"&&c.Moves.Length>0);Assert.Contains(b.Components,c=>c.Name=="B"&&c.Moves.Length>0);
 }
 [Fact] public void CapsuleIndexReturnsExactlyTheOldClosedBoxFilterIncludingTangency()
 {
  var random=new Random(4911);
  var caps=Enumerable.Range(0,5000).Select(i=>new RW.Toolpaths.Milling.ReviewedSource.ContactReviewProofs.AnalyticCapsuleCover.Capsule(i,
   new(random.NextDouble()*100,random.NextDouble()*100,-2),new(random.NextDouble()*100,random.NextDouble()*100,-2),random.NextDouble()*3)).ToArray();
  var index=new RW.Toolpaths.Milling.ReviewedSource.ContactReviewProofs.CapsuleSpatialIndex(caps);
  for(int i=0;i<100;i++)
  {
   double x=random.NextDouble()*110-5,y=random.NextDouble()*110-5,r=random.NextDouble()*5;
   var expected=caps.Where(c=>Math.Min(c.A.X,c.B.X)-c.Radius<=x+r&&Math.Max(c.A.X,c.B.X)+c.Radius>=x-r&&Math.Min(c.A.Y,c.B.Y)-c.Radius<=y+r&&Math.Max(c.A.Y,c.B.Y)+c.Radius>=y-r).Select(c=>c.ActualStockChord).ToArray();
   Assert.Equal(expected,index.Query(x-r,y-r,x+r,y+r).Select(c=>c.ActualStockChord));
  }
  var one=new RW.Toolpaths.Milling.ReviewedSource.ContactReviewProofs.AnalyticCapsuleCover.Capsule(999,new(1,1,-1),new(1,1,-1),1);
  Assert.Single(new RW.Toolpaths.Milling.ReviewedSource.ContactReviewProofs.CapsuleSpatialIndex([one]).Query(2,1,2,1));
 }
 [Fact] public void TwoPointDepthWallHasAnExactLinearGuideInsteadOfAnIndexFailure()
 {
  var method=typeof(VCarvePartialContactReview).GetMethod("Pchip",System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Static)!;
  var d=(double[])method.Invoke(null,new object[]{new double[]{0,8},new double[]{2,6}})!;
  Assert.Equal(new double[]{.5,.5},d);
 }
 [Fact] public void InvalidReviewPolicyCannotEnterConstruction()
 {
  var stock=new VCarveClearedStock([],[],[]);var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,1,1)],1);
  Assert.Throws<ArgumentException>(()=>VCarvePartialContactReview.Construct([],stock,profile,new(1,1),new(0,0,0),new(double.NaN,.3)));
 }
 [Fact] public void WholeCurveProofRejectsProtectedHoleBetweenSafeEndpoints()
 {
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.25,.25,4)],4);
  PointD[][] rings=[new PointD[]{new(0,0),new(10,0),new(10,10),new(0,10)},new PointD[]{new(4,4),new(4,6),new(6,6),new(6,4)}];
  var proof=new RW.Toolpaths.Milling.ReviewedSource.ContactReviewProofs.PreparedLayerProof(rings,profile,0,4);
  var a=new Point3D(2,5,-1);var b=new Point3D(8,5,-1);
  Assert.True(proof.Certify(ToolMove.CubicBezier(a,a,a,a,0)).Certified);Assert.True(proof.Certify(ToolMove.CubicBezier(b,b,b,b,0)).Certified);
  Assert.False(proof.Certify(ToolMove.CubicBezier(a,new(4,5,-1),new(6,5,-1),b,0)).Certified);
 }
}
