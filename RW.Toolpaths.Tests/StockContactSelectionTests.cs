using RW.Toolpaths;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.ReviewedSource;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class StockContactSelectionTests
{
 static VCarveStockContactSelection.Candidate Candidate(int pieces,double gap,VCarveStockContactRidge.Domain domain)
 {
  var moves=Enumerable.Range(0,pieces).Select(i=>ToolMove.Line(new(i,0,-1),new(i+1,0,-1))).ToArray();
  return new(domain,[],new(true,"test-ranking-only",moves,0,1,[]),gap);
 }
 [Fact] public void RemovalEnvelopeNominationCannotExcludeBetterValidGeometry()
 {
  var free=Candidate(34,.29,VCarveStockContactRidge.Domain.Connected);
  var restricted=Candidate(20,.97,VCarveStockContactRidge.Domain.SourceEnvelopeConnected);
  Assert.Same(free,VCarveStockContactSelection.Choose([restricted,free],.3));
  Assert.Same(free,VCarveStockContactSelection.Choose([free,restricted],.3));
 }
 [Fact] public void WithinConfiguredAllowanceDoesNotDemandExactZeroFinish()
 {
  var simple=Candidate(2,.25,VCarveStockContactRidge.Domain.SourceEnvelopeConnected);
  var moreComplex=Candidate(10,0,VCarveStockContactRidge.Domain.Connected);
  Assert.Same(simple,VCarveStockContactSelection.Choose([moreComplex,simple],.3));
 }
 [Fact] public void DiagnosticContactSitesAreRankingNotAnInventedFinishFailure()
 {
  var best=Candidate(3,.35,VCarveStockContactRidge.Domain.Connected);
  var worse=Candidate(2,.4,VCarveStockContactRidge.Domain.SourceEnvelopeConnected);
  Assert.Same(best,VCarveStockContactSelection.Choose([worse,best],.3));
 }
 [Fact] public void IndependentExtremaCannotBeTreatedAsActualResidualSurface()
 {
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,5,5)],5);
  var stock=new VCarveClearedStock([],[],[],[new(profile,[new(new(0,0,-1),new(2,0,-3),0)])]);
  var measured=VCarveContactSurfaceBounds.Measure(new(0,0),stock,.001);
  Assert.True(measured.Resolved);Assert.InRange(measured.Lower,.99999,1);Assert.InRange(measured.Upper,1,1.00101);
  // Combining deepest tip3with nearest radial distance0would falsely yield3.
  Assert.True(measured.Upper<1.01);
 }
 [Fact] public void PointDepthBudgetExhaustionDoesNotInventACertifiedHeight()
 {
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,5,5)],5);
  var stock=new VCarveClearedStock([],[],[],[new(profile,[new(new(0,0,-1),new(2,0,-3),0)])]);
  var measured=VCarveContactSurfaceBounds.Measure(new(0,0),stock,.00001,0);
  Assert.False(measured.Resolved);Assert.True(measured.Lower<=1);Assert.True(measured.Upper>=1);
 }
 [Fact] public void FullCutterDistanceIncludesRoundedNoseAndShoulder()
 {
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,2,2),new(CarvingCutterSectionKind.Linear,2,2,10)],6);
  var pose=new Point3D(0,0,-4);
  Assert.Equal(1,VCarveStockContactSelection.PointDistance(new(0,0,-5),pose,profile),8);
  Assert.Equal(1,VCarveStockContactSelection.PointDistance(new(3,0,0),pose,profile),8);
  Assert.Equal(0,VCarveStockContactSelection.PointDistance(new(0,0,0),pose,profile),8);
 }
}
