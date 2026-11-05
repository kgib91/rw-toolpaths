using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Tests;
public sealed class VCarveNativeFreeJoinTests
{
 sealed record Fixture(Point3D[] Source,int Middle,ToolMove First,ToolMove Second,CarvingCutterSection[] Profile,double MaximumDepth);
 [Fact] public void ActualTailHasTwoLongSmoothCurvesWithoutMovingItsProtectedPoses()
 {
  using var stream=typeof(VCarveNativeFreeJoinTests).Assembly.GetManifestResourceStream("RW.Toolpaths.Tests.Fixtures.reviewed-free-join-tangent.json")!;
  var f=JsonSerializer.Deserialize<Fixture>(stream,new JsonSerializerOptions{IncludeFields=true})!;var original=f.Source.ToArray();
  Assert.Equal(f.Source[0],f.First.Start);Assert.Equal(f.Source[f.Middle],f.First.End);Assert.Equal(f.First.End,f.Second.Start);Assert.Equal(f.Source[^1],f.Second.End);
  Assert.True(VCarveNativeGeometricProof.Prove(f.First,f.Source,0,f.Middle,.3).Certified);Assert.True(VCarveNativeGeometricProof.Prove(f.Second,f.Source,f.Middle,f.Source.Length-1,.3).Certified);
  var first=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(f.First,1));var second=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(f.Second,0));
  Assert.True(VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(first,second))<1e-8);
  var profile=new VCarveCutterProfile(f.Profile,f.MaximumDepth);var guard=new VCarveNativeContainment([new PointD[]{new(-1000,-1000),new(1000,-1000),new(1000,1000),new(-1000,1000)}],profile);
  foreach(var m in new[]{f.First,f.Second}){Assert.True(VCarveNativeCurveMath.Length(m).Lower>=.3);Assert.True(VCarveNativeCutterProof.Certified(m,guard,profile,out _));}
  Assert.Equal(original,f.Source);
 }
}
