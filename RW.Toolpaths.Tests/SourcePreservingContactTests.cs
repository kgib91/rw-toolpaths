using RW.Toolpaths;
using RW.Toolpaths.Milling.ReviewedSource;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class SourcePreservingContactTests
{
 static VCarveContactLocus.Candidate Pose(int station,Point3D p,double gap)=>new(p,p,station,1,gap,2*Math.Max(0,gap));
 [Fact] public void EqualContactQualityDoesNotAuthorizeShortcuttingIntendedRemoval()
 {
  var start=new Point3D(0,0,-1);var end=new Point3D(3,0,-1);
  var original=Pose(0,new(1,1,-1),.1);var shortcut=Pose(0,new(1,0,-1),.1);var mandatory=Pose(1,new(2,0,-1),.2);
  VCarveContactLocus.Candidate[] catalog=[shortcut,original,mandatory];
  var source=new Dictionary<int,Point3D>{{0,original.CertifiedPose},{1,mandatory.CertifiedPose}};
  var shortest=VCarveContactLocus.ChooseCoherent(start,end,catalog);
  var preserving=VCarveContactLocus.ChooseCoherent(start,end,catalog,source);
  Assert.Equal(shortcut,shortest[0]);Assert.Equal(original,preserving[0]);
  Assert.Equal(shortest.Max(c=>c.WorstContactGap),preserving.Max(c=>c.WorstContactGap));
  Assert.Equal(new[]{0,1},preserving.Select(c=>c.SourceStation));
 }
 [Fact] public void SourceProximityCannotTradeAwayBestContact()
 {
  var original=Pose(0,new(1,0,-1),.4);var improved=Pose(0,new(1,.1,-1),.1);
  var chosen=VCarveContactLocus.ChooseCoherent(new(0,0,-1),new(2,0,-1),[original,improved],new Dictionary<int,Point3D>{{0,original.CertifiedPose}});
  Assert.Equal(improved,Assert.Single(chosen));
 }
 [Fact] public void SourcePreservationHasNoWorldCoordinateOrTraversalOrientationPreference()
 {
  var start=new Point3D(0,0,-1);var end=new Point3D(3,0,-1);
  var source=new Dictionary<int,Point3D>{{0,new(1,1,-1)},{1,new(2,-1,-1)}};
  VCarveContactLocus.Candidate[] catalog=[Pose(0,new(1,0,-1),.1),Pose(0,source[0],.1),Pose(1,source[1],.1),Pose(1,new(2,0,-1),.1)];
  var chosen=VCarveContactLocus.ChooseCoherent(start,end,catalog,source);
  Point3D Mirror(Point3D p)=>p with{X=17-p.X};
  var mirrored=VCarveContactLocus.ChooseCoherent(Mirror(start),Mirror(end),catalog.Select(c=>c with{Nomination=Mirror(c.Nomination),CertifiedPose=Mirror(c.CertifiedPose)}).ToArray(),source.ToDictionary(p=>p.Key,p=>Mirror(p.Value)));
  Assert.Equal(chosen.Select(c=>Mirror(c.CertifiedPose)),mirrored.Select(c=>c.CertifiedPose));
  var reversed=VCarveContactLocus.ChooseCoherent(end,start,catalog.Select(c=>c with{SourceStation=1-c.SourceStation}).ToArray(),source.ToDictionary(p=>1-p.Key,p=>p.Value));
  Assert.Equal(chosen.Reverse().Select(c=>c.CertifiedPose),reversed.Select(c=>c.CertifiedPose));
 }
}
