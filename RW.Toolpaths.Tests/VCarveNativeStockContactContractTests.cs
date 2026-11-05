using System.IO.Compression;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Tests;
public sealed class VCarveNativeStockContactContractTests
{
 sealed class Context : IDisposable
 {
  readonly JsonDocument document;
  internal readonly VCarveCutterProfile Profile;
  internal readonly IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> Boundaries;
  internal readonly TaggedToolpath[] Paths;
  internal readonly VCarveClearedStock Stock;
  internal readonly double Surface,Depth;
  readonly JsonSerializerOptions json=new(){IncludeFields=true};
  internal Context()
  {
   var assembly=typeof(Context).Assembly;using var input=assembly.GetManifestResourceStream(assembly.GetManifestResourceNames().Single(n=>n.EndsWith("stock-contact-branch.json.gz")))!;
   using var gzip=new GZipStream(input,CompressionMode.Decompress);document=JsonDocument.Parse(gzip);var root=document.RootElement;
   T Read<T>(JsonElement e)=>JsonSerializer.Deserialize<T>(e.GetRawText(),json)!;
   VCarveCutterProfile Cutter(JsonElement e)=>new(Read<CarvingCutterSection[]>(e.GetProperty("Sections")),e.GetProperty("MaximumDepth").GetDouble());
   Surface=root.GetProperty("SurfaceZ").GetDouble();Depth=root.GetProperty("MaximumDepth").GetDouble();
   Profile=new(Read<CarvingCutterSection[]>(root.GetProperty("Profile")),Depth);
   Boundaries=Read<PointD[][][]>(root.GetProperty("Boundaries"));var stock=root.GetProperty("Stock");
   Stock=new(Read<VCarveRoughingChord[]>(stock.GetProperty("Chords")),Read<double[]>(stock.GetProperty("FloorDepths")),Read<PointD[]>(stock.GetProperty("PartBoundary")),
    stock.GetProperty("ProfileSweeps").EnumerateArray().Select(e=>new VCarveClearedSweep(Cutter(e.GetProperty("Profile")),Read<VCarveRoughingChord[]>(e.GetProperty("Chords")))).ToArray());
   Paths=root.GetProperty("Paths").EnumerateArray().Select((e,i)=>new TaggedToolpath(Read<Point3D[]>(e),[new(0,e.GetArrayLength()-1,i==3?ToolpathSpanKind.Link:ToolpathSpanKind.Cut)],11,"standard-vcarve",0,i,detectCurves:false)).ToArray();
  }
  internal IReadOnlyList<VCarveNativeCurvePlan> Fit(VCarveNativeSourceContract contract,bool usePrior=true,bool degreeTwo=false)
  {
   var paths=degreeTwo?Paths.Take(2).ToArray():Paths;
   var owner=new VCarveMaterialSurface(Boundaries.SelectMany(b=>b),usePrior?Stock:new([],[],Stock.PartBoundary),Profile,Depth,Surface,default,null);
   return VCarveNativeCurvePlanning.Nominate(paths,Boundaries,Profile,Surface,
    stockChord:(a,b)=>owner.IsLegalNativeChord(a,b),stockCurve:m=>owner.IsLegalNativeCurve(m),
    selectedSourceOrdinals:paths.Select(p=>(IReadOnlyList<int>)Enumerable.Range(0,p.Points.Count).ToArray()).ToArray(),
    requiredSourceOrdinals:paths.Select(p=>(IReadOnlyList<int>)new[]{0,p.Points.Count-1}).ToArray(),sourceContract:contract);
  }
  public void Dispose()=>document.Dispose();
 }
 [Fact] public void ContactDepthDoesNotInventAnAuthoredAxialDerivativeAtATrueBranch()
 {
  using var c=new Context();var original=c.Fit(VCarveNativeSourceContract.PreparedSpatialDirections);
  Assert.False(original[0].CandidateVerified);Assert.Equal(1,original[0].BelowMinimum);
  var fitted=c.Fit(VCarveNativeSourceContract.MedialPlanarWithStockContactDepth);
  Assert.True(fitted[0].CandidateVerified);Assert.Equal(0,fitted[0].BelowMinimum);
  var curve=Assert.Single(fitted[0].Primitives);Assert.Equal(ToolMoveKind.CubicBezier,curve.Move.Kind);
  Assert.Equal(c.Paths[0].Points[0],curve.Move.Start);Assert.Equal(c.Paths[0].Points[^1],curve.Move.End);
  // The former narrow numeric range described one nomination, not an
  // operation requirement. Certify the actual contract independently for the
  // newly selected algebraic candidate instead of pinning its handle lengths.
  Assert.True(curve.LengthLowerMm>=.3);Assert.InRange(curve.Move.MaxError,0,.3);
  var raw=c.Paths[0].Points.Select(p=>p with{Z=p.Z-c.Surface}).ToArray();
  var relative=VCarveNativeCurvePlanning.TranslateZ(curve.Move,-c.Surface);
  Assert.True(VCarveNativeGeometricProof.Prove(relative,raw,0,raw.Length-1,.3).Certified);
  var authority=new VCarveMaterialSurface(c.Boundaries.SelectMany(b=>b),c.Stock,c.Profile,c.Depth,c.Surface,default,null);
  Assert.True(authority.IsLegalNativeCurve(relative));
  Assert.Equal(1,fitted[0].StockContactEnvelopeCurves);Assert.True(fitted[0].MaximumSourceAxialDirectionChangeDegrees>100);
  var reverse=Assert.Single(fitted[3].Primitives).Move;
  Assert.Equal(curve.Move.Start,reverse.End);Assert.Equal(curve.Move.End,reverse.Start);
  Assert.Equal(curve.Move.Control1,reverse.Control2);Assert.Equal(curve.Move.Control2,reverse.Control1);
 }
 [Fact] public void SmoothDegreeTwoJointsKeepTheirSpatialDirectionContract()
 {
  using var c=new Context();var fitted=c.Fit(VCarveNativeSourceContract.MedialPlanarWithStockContactDepth,degreeTwo:true);
  Assert.False(fitted[0].CandidateVerified);Assert.Equal(0,fitted[0].StockContactEnvelopeCurves);
 }
 [Fact] public void ContactContractCannotReplaceMissingPriorStockProof()
 {
  using var c=new Context();var fitted=c.Fit(VCarveNativeSourceContract.MedialPlanarWithStockContactDepth,usePrior:false);
  Assert.False(fitted[0].CandidateVerified);Assert.Equal(0,fitted[0].StockContactEnvelopeCurves);
  Assert.Throws<InvalidOperationException>(()=>VCarveNativeCurvePlanning.Nominate(c.Paths,c.Boundaries,c.Profile,c.Surface,sourceContract:VCarveNativeSourceContract.MedialPlanarWithStockContactDepth));
 }
 [Fact] public void BooleanStockCertificatesDoNotRankUnusedCandidatesByFictitiousMargin()
 {
  Point3D[] points=[new(0,0,-.5),new(.25,.1,-.5),new(.5,.15,-.5),new(.75,.1,-.5),new(1,0,-.5)];
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);int calls=0;
  var guard=new VCarveNativeContainment([new PointD[]{new(-2,-2),new(2,-2),new(2,2),new(-2,2)}],profile,
    stockChord:(_,_)=>{calls++;return true;},stockCurve:_=>true);
  var fit=VCarveNativeEdgeSearch.Find(points,0,4,VCarveNativeCurveMath.Unit(new(.25,.1,0)),VCarveNativeCurveMath.Unit(new(.25,-.1,0)),1.1,guard,profile,out var audit);
  Assert.NotNull(fit);Assert.True(audit.Qualified>1);Assert.True(calls<audit.Qualified*2,$"Unused candidates triggered {calls} stock queries for {audit.Qualified} geometric nominations");
 }
 [Fact] public void DiagnosticCandidateSnapshotCannotSubstituteReturnedGeometry()
 {
  var previous=VCarveNativeCurvePlanning.CandidateObserver.Value;
  var previousRegion=VCarveNativeCurvePlanning.RegionObserver.Value;
  try
  {
   VCarveNativeCurvePlanning.RegionObserver.Value=(index,plan)=>{Assert.Equal(0,index);var copy=(VCarveNativePrimitive[])plan.Primitives;copy[0]=copy[0] with{Move=ToolMove.Line(new(888,888,888),new(889,888,888))};};
   VCarveNativeCurvePlanning.CandidateObserver.Value=plans=>{
    var copied=(VCarveNativePrimitive[])plans[0].Primitives;
    copied[0]=copied[0] with{Move=ToolMove.Line(new(999,999,999),new(1000,999,999))};
   };
   var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
   var path=new TaggedToolpath(new Point3D[]{new(0,0,-.5),new(1,0,-.5)},Array.Empty<ToolpathSpan>(),0,"test",null,detectCurves:false);
   IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> boundary=new IReadOnlyList<PointD>[][]{new PointD[][]{[new(-2,-2),new(2,-2),new(2,2),new(-2,2)]}};
   var plans=VCarveNativeCurvePlanning.Nominate([path],boundary,profile,0);
   Assert.Equal(path.Points[0],plans[0].Primitives[0].Move.Start);
  }
  finally{VCarveNativeCurvePlanning.CandidateObserver.Value=previous;VCarveNativeCurvePlanning.RegionObserver.Value=previousRegion;}
 }
}
