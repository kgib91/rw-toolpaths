using Clipper2Lib;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Geometry;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class SmoothStockContactTests
{
 [Theory] [InlineData(1)] [InlineData(.2)]
 public void NormalSmoothGenerationUsesResidualContactForCuttingAndEntry(double stepdown)
 {
  var boundary=Box(0,0,4,4);var part=Box(-20,-20,20,20);
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,1,1)],1);
  var stock=new VCarveClearedStock([new(new(2,2,-.6),new(2,2,-.6),100,100)],[.6],part);
  var result=VCarveToolpaths.Generate([new MillingRegion(boundary)],new ToolGeometry(1){VCarveProfile=profile},
      new DepthSchedule(1,stepdown){StartDepth=.6},new VCarveOptions{SmoothVCarving=true,IncludeInteriorFill=true,
        ClearedStock=stock,DetectCurves=false,CalculateSweptArea=false,PathToleranceMm=.03},BoostVoronoiProvider.CreateDefault());
  Assert.NotEmpty(result.Toolpaths);
  Assert.True(result.Toolpaths.SelectMany(p=>p.Points).Max(p=>p.Z)<=-.598,"Cutting entry must use residual-stock contact, not original stock top");
  Assert.NotNull(result.ReplacementChordGuard);
  Assert.All(result.Toolpaths.SelectMany(p=>p.Points.Zip(p.Points.Skip(1))),e=>Assert.True(result.ReplacementChordGuard!(e.First,e.Second)));
 }
 static PointD[] Box(double a,double b,double c,double d)=>[new(a,b),new(c,b),new(c,d),new(a,d)];
 static readonly PointD[] Target=Box(5,5,25,25),Part=Box(-50,-50,50,50);
 static readonly VCarveCutterProfile Cutter=new([new(CarvingCutterSectionKind.Linear,0,12.7,12.7)],12.7);
 static VCarveMaterialSurface Owner(double radius)=>new([Target],new([new(new(15,15,-6),new(15,15,-6),radius,radius)],[6],Part),Cutter,12.7,0,default,null);
 [Fact] public void ClearedCrestUsesContactHeightButHigherResidualStaysAtTop()
 {
  var source=new TaggedToolpath(new Point3D[]{new(5,10,0),new(5,20,0)},Array.Empty<ToolpathSpan>(),0,"final-carve",null,detectCurves:false);
  var lower=VCarveStockContactPaths.Project([source],Owner(100),.03,default).Single();
  Assert.All(lower.Points,p=>Assert.InRange(p.Z,-6,-5.998));
  var high=VCarveStockContactPaths.Project([source],Owner(1),.03,default).Single();
  Assert.All(high.Points,p=>Assert.Equal(0,p.Z));
  var floor=new VCarveSmoothing.FloorField([lower],Cutter,1,12.7);
  Assert.True(floor.Depth(6,15)>5.9);
 }
 [Fact] public void ActualStockReplacesOnlyArtworkExtrusionNotSelectedChannel()
 {
  var owner=Owner(100);var a=new Point3D(5,10,-5.999);var b=new Point3D(5,20,-5.999);
  var nominal=new VCarveSmoothing.CommandClearance([Target],Cutter,0,default,null);
  Assert.False(nominal.IsLegalCommandedChord(a,b));
  var actual=new VCarveSmoothing.CommandClearance([Target],Cutter,0,default,null,stockContact:owner);
  Assert.True(actual.IsLegalCommandedChord(a,b));
  var channel=new VCarveSmoothing.CommandClearance([Target],Cutter,0,default,null,
      new IReadOnlyList<PointD>[]{Box(10,10,20,20)},100000,actual);
  Assert.False(channel.IsLegalCommandedChord(a,b));
  Assert.False(actual.IsLegalCommandedChord(new(-49,15,-6),new(-49,16,-6)));
 }
 [Fact] public void AirVisitSelectionRequiresEntirePriorRemovalNotTargetOrLocalStartDepth()
 {
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
  var part=Box(-5,-5,5,5);
  var source=new TaggedToolpath(new Point3D[]{new(0,0,-1),new(2,0,-1)},Array.Empty<ToolpathSpan>(),0,"final-carve",null,detectCurves:false);
  var empty=new VCarveMaterialSurface([],new([],[],part),profile,1,0,default,null);
  var partial=new VCarveMaterialSurface([],new([new(new(0,0,-.6),new(.5,0,-.6),.5,.5)],[.6],part),profile,1,0,default,null);
  var full=new VCarveMaterialSurface([],new([new(new(0,0,-.6),new(2,0,-.6),2,2)],[.6],part),profile,1,0,default,null);
  var schedule=new DepthSchedule(1,.2){StartDepth=.6};
  var noAuthority=VCarveToolpaths.ScheduleFreshStockVisits([source],schedule,default,null,priorRemoved:(a,b)=>empty.IsLegalChord(a,b));
  var higherResidual=VCarveToolpaths.ScheduleFreshStockVisits([source],schedule,default,null,priorRemoved:(a,b)=>partial.IsLegalChord(a,b));
  var cleared=VCarveToolpaths.ScheduleFreshStockVisits([source],schedule,default,null,priorRemoved:(a,b)=>full.IsLegalChord(a,b));
  Assert.Contains(noAuthority,p=>p.Points.All(p=>p.Z>-.3));
  Assert.Contains(higherResidual,p=>p.Points.All(p=>p.Z>-.3));
  Assert.DoesNotContain(cleared,p=>p.Points.All(p=>p.Z>-.5));
  Assert.Contains(cleared,p=>p.Points.All(p=>p.Z==-1));
 }
 [Fact] public void ClearedEndpointsCannotHideAnUnclearedMiddleInterval()
 {
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
  var prior=new VCarveClearedStock([new(new(0,0,-.6),new(.8,0,-.6),.2,.2),new(new(1.2,0,-.6),new(2,0,-.6),.2,.2)],[.6],Box(-5,-5,5,5));
  var owner=new VCarveMaterialSurface([],prior,profile,1,0,default,null);
  var a=new Point3D(0,0,-.2);var b=new Point3D(2,0,-.2);
  Assert.True(owner.IsLegalChord(a,a));Assert.True(owner.IsLegalChord(b,b));Assert.False(owner.IsLegalChord(a,b));
  var source=new TaggedToolpath(new Point3D[]{a with{Z=-1},b with{Z=-1}},Array.Empty<ToolpathSpan>(),0,"final-carve",null,detectCurves:false);
  var passes=VCarveToolpaths.ScheduleFreshStockVisits([source],new(1,.2){StartDepth=.6},default,null,
      priorRemoved:(x,y)=>owner.IsLegalChord(x,y));
  Assert.Contains(passes,p=>p.Points.All(p=>p.Z>-.3));
  Assert.Contains(passes,p=>p.Points.All(p=>p.Z==-1));
 }
 [Fact] public void PriorProfileSweepsCanCertifyAirWithoutFlatColumnCredit()
 {
  var cutter=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
  var previous=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.4,.4,2)],1);
  var prior=new VCarveClearedStock([],[.6],Box(-5,-5,5,5),
      [new(previous,[new(new(0,0,-.6),new(2,0,-.6),0,0)])]);
  var owner=new VCarveMaterialSurface([],prior,cutter,1,0,default,null);
  var a=new Point3D(0,0,-.2);var b=new Point3D(2,0,-.2);
  Assert.True(owner.IsLegalChord(a,b));Assert.False(owner.IsLegalChord(a with{Z=-.8},b with{Z=-.8}));
  var source=new TaggedToolpath(new Point3D[]{a with{Z=-1},b with{Z=-1}},Array.Empty<ToolpathSpan>(),0,"final-carve",null,detectCurves:false);
  var passes=VCarveToolpaths.ScheduleFreshStockVisits([source],new(1,.2){StartDepth=.6},default,null,
      priorRemoved:(x,y)=>owner.IsLegalChord(x,y));
  Assert.DoesNotContain(passes,p=>p.Points.All(p=>p.Z>-.5));
  Assert.Contains(passes,p=>p.Points.All(p=>p.Z==-1));
 }
}
