using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Clipper2Lib;
using Xunit;
namespace RW.Toolpaths.Tests;

public sealed class ContactRefinementGeometryTests
{
 [Fact] public void RecordedCommandGridDepthJumpHasSmallOrderedGeometricError()
 {
  var a=new Point3D(293.52170448684683,463.7305145263672,-1.632190812761344);
  var b=new Point3D(293.521704486668,463.73051452445986,-1.5457064947553276);
  var samples=new[]{new Point3D(293.5217044868021,463.73051452589035,b.Z),
   new Point3D(293.5217044867574,463.7305145254135,b.Z),
   new Point3D(293.5217044867127,463.7305145249366,b.Z)};
  Assert.True(Math.Abs(samples[0].Z-(a.Z+(b.Z-a.Z)*.25))>.03);
  Assert.InRange(VCarveMaterialSurface.ContactRefinementError(a,b,samples),0,1e-8);
 }
 [Fact] public void ARealLateralBendStillRequiresRefinement()
 {
  Assert.InRange(VCarveMaterialSurface.ContactRefinementError(new(0,0,0),new(1,0,0),
   new[]{new Point3D(.25,0,0),new Point3D(.5,.04,0),new Point3D(.75,0,0)}),.03999,.04001);
 }
 [Fact] public void DepthExtremumBeyondTheSegmentIsNotHiddenByReparameterization()
 {
  Assert.InRange(VCarveMaterialSurface.ContactRefinementError(new(0,0,0),new(0,0,-1),
   new[]{new Point3D(0,0,-.3),new Point3D(0,0,-1.2),new Point3D(0,0,-1)}),.19999,.20001);
 }
 [Fact] public void RetracingSamplesCannotMatchTheChordOutOfOrder()
 {
  Assert.InRange(VCarveMaterialSurface.ContactRefinementError(new(0,0,0),new(1,0,0),
   new[]{new Point3D(.8,0,0),new Point3D(.2,0,0),new Point3D(.9,0,0)}),.59999,.60001);
 }
 [Fact] public void CoincidentEndsDoNotDiscardAnExcursion()
 {
  Assert.Equal(.1,VCarveMaterialSurface.ContactRefinementError(new(0,0,0),new(0,0,0),new[]{new Point3D(0,0,-.1)}),12);
 }
 static PointD[] Box(double x0,double y0,double x1,double y1)=>[new(x0,y0),new(x1,y0),new(x1,y1),new(x0,y1)];
 static VCarveMaterialSurface JoinSurface()
 {
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.5,.5,20)],1);
  var stock=new VCarveClearedStock([new(new(2,0,-.49),new(2,0,-.49),100,100)],[.49],Box(-10,-10,10,10));
  return new([Box(-1,-1,1,1),Box(3,-1,5,1)],stock,profile,1,0,default,null);
 }
 [Fact] public void BoundedJoinPreservesBothEndpointsAndCertifiesEveryNativeAndPostedSweep()
 {
  var surface=JoinSurface();var a=new Point3D(0,0,-.5);var b=new Point3D(4,0,-.48);
  Assert.True(surface.IsLegalChord(a,a));Assert.True(surface.IsLegalChord(b,b));
  Assert.False(surface.IsLegalChord(a,b));
  Assert.True(VCarveMaterialSurface.TryBoundedContactJoin(a,b,.03,(p,q)=>surface.IsLegalChord(p,q),out var join,out double error));
  Assert.Equal(a,join[0]);Assert.Equal(b,join[^1]);Assert.InRange(error,.019,.021);
  for(int i=1;i<join.Length;i++)Assert.True(surface.IsLegalChord(join[i-1],join[i]));
  Assert.All(join,p=>Assert.InRange(p.Z,-.5,-.48));
 }
 [Fact] public void BoundedJoinRefusesWhenPhysicalClearanceNeedsMoreThanTheContactAllowance()
 {
  var surface=JoinSurface();var a=new Point3D(0,0,-.6);var b=new Point3D(4,0,-.48);
  Assert.True(surface.IsLegalChord(a,a));Assert.True(surface.IsLegalChord(b,b));
  Assert.False(VCarveMaterialSurface.TryBoundedContactJoin(a,b,.03,(p,q)=>surface.IsLegalChord(p,q),out var join,out _));
  Assert.Empty(join);
 }
 [Fact] public void ExhaustedContactAllowanceCannotGrantAJoin()
 {
  Assert.False(VCarveMaterialSurface.TryBoundedContactJoin(new(0,0,0),new(1,0,0),0,(_,_)=>true,out var join,out _));
  Assert.Empty(join);
 }
 [Theory] [InlineData(false)] [InlineData(true)]
 public void NearVerticalJoinPreservesTheEntireDepthExcursionInBothDirections(bool reverse)
 {
  var surface=JoinSurface();var a=new Point3D(0,0,-.8);var b=new Point3D(1e-8,0,-.48);
  if(reverse)(a,b)=(b,a);
  Assert.True(VCarveMaterialSurface.TryBoundedContactJoin(a,b,.03,(p,q)=>surface.IsLegalChord(p,q),out var join,out double error));
  Assert.Equal(a,join[0]);Assert.Equal(b,join[^1]);Assert.Equal(-.8,join.Min(p=>p.Z));Assert.Equal(-.48,join.Max(p=>p.Z));
  Assert.InRange(error,1e-8,1.000001e-8);
  Assert.True(VCarveMaterialSurface.ContactRefinementError(a,b,[join[1]])<=error);
  for(int i=1;i<join.Length;i++)Assert.True(surface.IsLegalChord(join[i-1],join[i]));
 }
 [Fact] public void SmallGeometricDeviationCannotBypassProtectedMaterialBetweenLegalEndpoints()
 {
  var surface=JoinSurface();var a=new Point3D(0,0,-.6);var b=new Point3D(4,0,-.59);
  Assert.True(surface.IsLegalChord(a,a));Assert.True(surface.IsLegalChord(b,b));
  Assert.False(VCarveMaterialSurface.TryBoundedContactJoin(a,b,.03,(p,q)=>surface.IsLegalChord(p,q),out var join,out _));
  Assert.Empty(join);
 }
}
