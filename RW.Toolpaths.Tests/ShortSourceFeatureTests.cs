using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Xunit;
public sealed class ShortSourceFeatureTests
{
 static readonly VCarveCutterProfile Tool=new([new(CarvingCutterSectionKind.Linear,0,1,10)],2);
 static readonly IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> Boundary=[new PointD[][]{[new(-5,-5),new(5,-5),new(5,5),new(-5,5)]}];
 static VCarveNativeCurvePlan Fit(Point3D[] p,bool legal=true,int[]? required=null){
  var source=new TaggedToolpath(p,[new(0,p.Length-1,ToolpathSpanKind.Cut)],0,"test",0,0,detectCurves:false);
  return VCarveNativeCurvePlanning.Nominate([source],Boundary,Tool,0,settings:new(.3,.3),stockChord:(_,_)=>legal,stockCurve:_=>legal,
   selectedSourceOrdinals:[Enumerable.Range(0,p.Length).ToArray()],requiredSourceOrdinals:[required??[0,p.Length-1]],
   sourceContract:VCarveNativeSourceContract.MedialPlanarWithStockContactDepth).Single();
 }
 [Fact] public void WholeShortFeatureIsPreservedExactlyAndReported(){
  Point3D[] source=[new(0,0,-1),new(.08,.005,-1),new(.18,0,-1)];var plan=Fit(source);
  Assert.True(plan.CandidateVerified);Assert.Equal(2,plan.BelowMinimum);
  Assert.All(plan.Primitives,p=>{Assert.Equal(new VCarveNativeShortFeature(0,2),p.ShortSourceFeature);Assert.Equal(0,p.Move.MaxError);Assert.Equal(source[p.SourceFirst],p.Move.Start);Assert.Equal(source[p.SourceLast],p.Move.End);});
 }
 [Fact] public void ShortFeatureCannotWaiveCutterClearance(){
  var plan=Fit([new(0,0,-1),new(.18,0,-1)],false);
  Assert.False(plan.CandidateVerified);Assert.False(plan.NewCurveChecksPassed);
 }
 [Fact] public void ArbitrarySubdivisionOfLongFeatureDoesNotGetAnException(){
  var plan=Fit([new(0,0,-1),new(.1,0,-1),new(1,0,-1)],required:[0,1,2]);
  Assert.All(plan.Primitives,p=>Assert.Null(p.ShortSourceFeature));
  Assert.False(plan.CandidateVerified);
 }
 [Fact] public void WholeShortClosedVisitKeepsOneFeatureAcrossArtificialFittingSplit(){
  Point3D[] source=[new(49.257,195.714,-2),new(49.261,195.7,-2),new(49.264,195.691,-2),new(49.257,195.714,-2)];
  var plan=Fit(source);
  Assert.True(plan.CandidateVerified);Assert.Equal(3,plan.BelowMinimum);
  Assert.All(plan.Primitives,p=>{Assert.Equal(new VCarveNativeShortFeature(0,3),p.ShortSourceFeature);Assert.Equal(source[p.SourceFirst],p.Move.Start);Assert.Equal(source[p.SourceLast],p.Move.End);});
  Assert.Equal(source[0],plan.Primitives[0].Move.Start);Assert.Equal(source[0],plan.Primitives[^1].Move.End);
 }
 [Fact] public void ClosedVisitStillRequiresFreshCutterClearance(){
  var plan=Fit([new(0,0,-1),new(.02,0,-1),new(.02,.01,-1),new(0,0,-1)],false);
  Assert.False(plan.CandidateVerified);Assert.False(plan.NewCurveChecksPassed);
 }
 [Fact] public void ClosedVisitCannotCrossAnInternalTopologyPort(){
  Point3D[] p=[new(0,0,-1),new(.02,0,-1),new(.02,.01,-1),new(0,0,-1)];
  TaggedToolpath[] source=[new(p,[new(0,3,ToolpathSpanKind.Cut)],0,"loop",0,0,detectCurves:false),
   new([p[1],new(1,0,-1)],[new(0,1,ToolpathSpanKind.Cut)],0,"branch",0,1,detectCurves:false)];
  var plans=VCarveNativeCurvePlanning.Nominate(source,Boundary,Tool,0,settings:new(.3,.3),stockChord:(_,_)=>true,stockCurve:_=>true,
   sourceContract:VCarveNativeSourceContract.MedialPlanarWithStockContactDepth);
  Assert.DoesNotContain(plans[0].Primitives,p=>p.ShortSourceFeature==new VCarveNativeShortFeature(0,3));
 }
}
