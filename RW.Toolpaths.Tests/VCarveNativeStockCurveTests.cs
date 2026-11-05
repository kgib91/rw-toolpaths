using Clipper2Lib;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Tests;
public sealed class VCarveNativeStockCurveTests
{
 [Fact]
 public void ContinuousStockCurveRetainsPriorRemovalPartHoleAndDepthGuards()
 {
PointD[] Rect(double x0,double y0,double x1,double y1)=>[new(x0,y0),new(x1,y0),new(x1,y1),new(x0,y1)];
var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
var target=Rect(0,0,1,1); var part=Rect(-2,-2,3,3);
VCarveMaterialSurface Owner(double priorDepth,PointD[]? boundary=null)=>new([target],new([new(new(1.1,0,-priorDepth),new(1.1,1,-priorDepth),.5,.5)],[priorDepth],boundary??part),profile,1,0,default,null);
var a=new Point3D(.98,.2,-.5);var b=new Point3D(.98,.8,-.5);
var curve=ToolMove.CubicBezier(a,new(.99,.4,-.5),new(.99,.6,-.5),b,0);
var cases=new List<object>();
void Check(string name,bool actual,bool expected){cases.Add(new{name,actual,expected});Console.WriteLine($"{name}: {actual} expected {expected}");Assert.True(actual==expected,name);}
Check("solid extrusion rejects source beyond target",new VCarveNativeContainment([target],profile).MinimumSquaredSeparation(a,a)<0,true);
Check("prior deep removal permits continuous curve",Owner(.6).IsLegalNativeCurve(curve),true);
Check("shallow prior removal cannot authorize deep curve",Owner(.25).IsLegalNativeCurve(curve),false);
Check("part boundary retains physical limit",Owner(.6,Rect(0,0,1.02,1)).IsLegalNativeCurve(curve),false);
var tooDeep=ToolMove.CubicBezier(a,new(.99,.4,-2),new(.99,.6,-2),b,0);
Check("exact interior depth overshoot rejected",Owner(.6).IsLegalNativeCurve(tooDeep),false);
var outer=Rect(-2,-2,2,2);var hole=Rect(.4,.02,.6,.3);Array.Reverse(hole);
var holes=new VCarveMaterialSurface([outer,hole],new([],[],Rect(-3,-3,3,3)),profile,1,0,default,null);
var hiddenCross=ToolMove.CubicBezier(new(0,0,-.5),new(.3,1,-.5),new(.7,-1,-.5),new(1,0,-.5),0);
Check("interior hole crossing between coarse samples rejected",holes.IsLegalNativeCurve(hiddenCross),false);
var tightProfile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,1,2)],.5);
var tight=new VCarveProjectedStock([], [Rect(-.2,-.2,.2,.2)],Rect(-3,-3,3,3),tightProfile,.5,default);
Check("deepened enclosure beyond operation depth still checks cutter body",tight.ProveNativeEnvelope(new(0,0,-.7),new(.01,0,-.7),0),false);

 }
 [Fact]
 public void MixedAuthoritiesMustCoverEveryClosedChildInterval()
 {
  var m=ToolMove.CubicBezier(new(0,0,-.5),new(1d/3,0,-.5),new(2d/3,0,-.5),new(1,0,-.5),0);
  Assert.True(VCarveNativeIntervalProof.Certified(m,p=>p.End.X<=.5,p=>p.Start.X>=.5));
  Assert.False(VCarveNativeIntervalProof.Certified(m,p=>p.End.X<=.49,p=>p.Start.X>=.51));
  Assert.False(VCarveNativeIntervalProof.Certified(m,p=>p.End.X<=.5,p=>false));
  Assert.False(VCarveNativeIntervalProof.Certified(m,p=>p.End.X<=.5,p=>p.Start.X>=.5,maximumIntervals:1));
 }
 [Fact]
 public void RetainedLongSourceLineDoesNotBypassStockAuthority()
 {
  var points=new[]{new Point3D(0,0,-.5),new Point3D(1,0,-.5)};
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
  var paths=new[]{new TaggedToolpath(points,[],0,"test",0,detectCurves:false)};
  var rings=new[]{(IReadOnlyList<IReadOnlyList<PointD>>)new[]{(IReadOnlyList<PointD>)new PointD[]{new(-2,-2),new(2,-2),new(2,2),new(-2,2)}}};
  var plan=VCarveNativeCurvePlanning.Nominate(paths,rings,profile,0,stockChord:(a,b)=>false,stockCurve:m=>false).Single();
  Assert.False(plan.CandidateVerified);
  Assert.Equal(1,plan.SearchRejections["retained-line-not-clearance-certified"]);
 }
}
