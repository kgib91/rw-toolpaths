using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Tests;
public sealed class VCarveNativeLocalRepairTests
{
 sealed record Fixture(Point3D[] Points,CarvingCutterSection[] Profile,double MaximumDepth,VCarveNativeFit[] Fits);
 static Fixture Load()
 {
  using var stream=typeof(VCarveNativeLocalRepairTests).Assembly.GetManifestResourceStream("RW.Toolpaths.Tests.Fixtures.reviewed-local-resegmentation.json")!;
  return JsonSerializer.Deserialize<Fixture>(stream,new JsonSerializerOptions{IncludeFields=true})!;
 }
 static VCarveNativeContainment Guard(VCarveCutterProfile p)=>new([new PointD[]{new(-1000,-1000),new(1000,-1000),new(1000,1000),new(-1000,1000)}],p);
 [Fact]
 public void RealShortTailCanRepartitionWithoutMovingProtectedEndpointsOrCompoundingError()
 {
  var f=Load();var original=f.Points.ToArray();var profile=new VCarveCutterProfile(f.Profile,f.MaximumDepth);var guard=Guard(profile);
  var repaired=VCarveNativeShortSpanRepair.Repair(f.Points,f.Fits.ToList(),Enumerable.Range(0,f.Points.Length).ToArray(),guard,profile,new(default,5000),new(.3,.3),2048,default,new Dictionary<string,int>(),out bool exhausted,VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(f.Fits[0].Move,0)),VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(f.Fits[^1].Move,1)));
  Assert.False(exhausted);Assert.Contains(f.Fits,p=>p.LengthLower<.3);Assert.All(repaired,p=>Assert.True(p.LengthLower>=.3));
  Assert.Equal(0,repaired[0].First);Assert.Equal(f.Points.Length-1,repaired[^1].Last);
  Assert.Equal(f.Fits[0].Move.Start,repaired[0].Move.Start);Assert.Equal(f.Fits[^1].Move.End,repaired[^1].Move.End);
  Assert.Equal(original,f.Points);
  foreach(var p in repaired)
  {
   Assert.True(VCarveNativeGeometricProof.Prove(p.Move,f.Points,p.First,p.Last,.3).Certified);
   Assert.True(VCarveNativeCutterProof.Certified(p.Move,guard,profile,out _));
  }
  for(int i=1;i<repaired.Count;i++)
  {
   var a=repaired[i-1];var b=repaired[i];Assert.Equal(a.Last,b.First);Assert.Equal(a.Move.End,b.Move.Start);
   var da=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(a.Move,1));
   var db=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(b.Move,0));
   Assert.True(VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(da,db))<1e-8);
  }
 }
 [Fact]
 public void RejectedStockAuthorityCannotBeReplacedBySourceOrSampledPoses()
 {
  var f=Load();var profile=new VCarveCutterProfile(f.Profile,f.MaximumDepth);
  var guard=new VCarveNativeContainment([new PointD[]{new(-1000,-1000),new(1000,-1000),new(1000,1000),new(-1000,1000)}],profile,stockChord:(a,b)=>false,stockCurve:m=>false);
  var repaired=VCarveNativeShortSpanRepair.Repair(f.Points,f.Fits.ToList(),Enumerable.Range(0,f.Points.Length).ToArray(),guard,profile,new(default,5000),new(.3,.3),2048,default,new Dictionary<string,int>(),out _,VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(f.Fits[0].Move,0)),VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(f.Fits[^1].Move,1)));
  Assert.Equal(f.Fits,repaired);Assert.Contains(repaired,p=>p.LengthLower<.3);
 }
 [Fact]
 public void ExplicitProtectedDirectionsCannotBeInheritedFromRetainedShortEvidence()
 {
  var f=Load();var profile=new VCarveCutterProfile(f.Profile,f.MaximumDepth);
  // Deliberately differs from the incumbent. A successful repair must honor
  // this caller-owned contract; an unresolved diagnostic short is not success.
  var u=new Point3D(0,1,0);var v=new Point3D(1,0,0);
  var repaired=VCarveNativeShortSpanRepair.Repair(f.Points,f.Fits.ToList(),Enumerable.Range(0,f.Points.Length).ToArray(),Guard(profile),profile,new(default,5000),new(.3,.3),2048,default,new Dictionary<string,int>(),out _,u,v);
  foreach(var (fit,tangent,side) in new[]{(repaired[0],u,0d),(repaired[^1],v,1d)})
  {
   if(f.Fits.Contains(fit))continue; // Unchanged diagnostic evidence, never accepted output.
   var actual=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(fit.Move,side));
   Assert.True(VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(actual,tangent))<1e-8);
  }
 }
}
