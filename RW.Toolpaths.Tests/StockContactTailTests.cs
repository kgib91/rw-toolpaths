using System.IO.Compression;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class StockContactTailTests
{
 [Theory][InlineData(false)][InlineData(true)]
 public void TrueContactBranchEndsDoNotInheritSampledAxialDerivatives(bool reverse)
 {
  var assembly=typeof(StockContactTailTests).Assembly;
  using var file=assembly.GetManifestResourceStream(assembly.GetManifestResourceNames().Single(n=>n.EndsWith("stock-contact-tail.json.gz")))!;
  using var compressed=new GZipStream(file,CompressionMode.Decompress);using var document=JsonDocument.Parse(compressed);
  var d=document.RootElement;var json=new JsonSerializerOptions{IncludeFields=true};T Read<T>(JsonElement e)=>JsonSerializer.Deserialize<T>(e,json)!;
  VCarveCutterProfile Profile(JsonElement e)=>new(Read<CarvingCutterSection[]>(e.GetProperty("Sections")),e.GetProperty("MaximumDepth").GetDouble());
  var profile=new VCarveCutterProfile(Read<CarvingCutterSection[]>(d.GetProperty("Profile")),d.GetProperty("MaximumDepth").GetDouble());
  var s=d.GetProperty("Stock");var stock=new VCarveClearedStock(Read<VCarveRoughingChord[]>(s.GetProperty("Chords")),Read<double[]>(s.GetProperty("FloorDepths")),Read<PointD[]>(s.GetProperty("PartBoundary")),s.GetProperty("ProfileSweeps").EnumerateArray().Select(e=>new VCarveClearedSweep(Profile(e.GetProperty("Profile")),Read<VCarveRoughingChord[]>(e.GetProperty("Chords")))).ToArray());
  var boundary=Read<PointD[][]>(d.GetProperty("Boundary"));var authority=new VCarveMaterialSurface(boundary,stock,profile,profile.MaximumDepth,d.GetProperty("SurfaceZ").GetDouble(),default,null);
  var guard=new VCarveNativeContainment(boundary,profile,stockChord:(a,b)=>authority.IsLegalChord(a,b),stockCurve:authority.IsLegalNativeCurve);
  var points=Read<Point3D[]>(d.GetProperty("Points"));var knots=Read<int[]>(d.GetProperty("Knots"));
  var input=Read<VCarveNativePrimitive[]>(d.GetProperty("InputPrimitives"));int last=points.Length-1;
  if(reverse){points=points.Reverse().ToArray();knots=knots.Reverse().Select(i=>last-i).ToArray();input=input.Reverse().Select(p=>p with{Move=VCarveNativeCurveMath.Reverse(p.Move),SourceFirst=last-p.SourceLast,SourceLast=last-p.SourceFirst}).ToArray();}
  double SourceLength(int a,int b)=>Enumerable.Range(a+1,b-a).Sum(i=>VCarveNativeCurveMath.Distance(points[i-1],points[i]));
  var original=input.Select(p=>new VCarveNativeFit(p.Move,p.SourceFirst,p.SourceLast,SourceLength(p.SourceFirst,p.SourceLast),p.LengthLowerMm,p.LengthUpperMm,0,0,p.Move.Kind==ToolMoveKind.Line)).ToList();
  var start=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[knots[1]],points[0]));var end=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[^1],points[knots[^2]]));
  List<VCarveNativeFit> Repair(VCarveNativeSourceContract contract)=>VCarveNativeShortSpanRepair.Repair(points,original,knots,guard,profile,new(default,50_000),new(.3,.3),4096,default,new Dictionary<string,int>(),out _,start,end,contract);
  var spatial=Repair(VCarveNativeSourceContract.PreparedSpatialDirections);Assert.Contains(spatial,p=>p.LengthLower<.3);
  var contact=Repair(VCarveNativeSourceContract.MedialPlanarWithStockContactDepth);
  Assert.NotEmpty(contact);Assert.Equal(0,contact[0].First);Assert.Equal(last,contact[^1].Last);
  Assert.All(contact,p=>{Assert.True(p.LengthLower>=.3);Assert.InRange(p.Move.MaxError,0,.3);Assert.True(authority.IsLegalNativeCurve(p.Move));Assert.True(VCarveNativeGeometricProof.Prove(p.Move,points,p.First,p.Last,.3).Certified);});
  for(int i=1;i<contact.Count;i++)
  {
   Assert.Equal(contact[i-1].Last,contact[i].First);Assert.Equal(contact[i-1].Move.End,contact[i].Move.Start);
   var u=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(contact[i-1].Move,1));var v=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(contact[i].Move,0));
   Assert.True(VCarveNativeCurveMath.Distance(u,v)<1e-8);
  }
  Assert.True(VCarveNativeCurveMath.Distance(VCarveNativeStockContactFit.Planar(start),VCarveNativeStockContactFit.Planar(VCarveNativeCurveMath.Derivative(contact[0].Move,0)))<1e-8);
  Assert.True(VCarveNativeCurveMath.Distance(VCarveNativeStockContactFit.Planar(end),VCarveNativeStockContactFit.Planar(VCarveNativeCurveMath.Derivative(contact[^1].Move,1)))<1e-8);
 }
}
