using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.ReviewedSource;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class ReviewedSingleStockFrontTests
{
 static readonly PointD[][] Boundary=[[new(5,5),new(15,5),new(15,15),new(5,15)]];
 static readonly PointD[] Part=[new(0,0),new(20,0),new(20,20),new(0,20)];
 static readonly VCarveCutterProfile Cutter=new([new(CarvingCutterSectionKind.Linear,0,4,4)],4);
 static readonly ReviewedVCarveSourceSettings Settings=new(.03,.03,135*Math.PI/180);
 static ReviewedVCarveStockContact Owner(double removed)=>new([Boundary],Cutter,new(4,4){SurfaceZ=6},
  new VCarveClearedStock(removed>0?[new(new(0,10,-removed),new(20,10,-removed),20,20)]:[],removed>0?[removed]:[],Part),.03);
 [Fact] public void UniformRemainingSurfaceChangesTheOriginalCapRadius()
 {
  var input=new ReviewedVCarveRegion(0,Boundary,Boundary);var owner=Owner(2);var front=owner.DepthFront(input);
  Assert.Single(front);Assert.InRange(front[0].Min(p=>p.x),7.0039,7.0041);
  var raw=ReviewedVCarveConstructor.Construct(input,Cutter,4,6,4,Settings,contactDepthFront:front);
  var graph=owner.Project(raw).Graph;
  var cap=graph.Edges.Where(e=>e.Owner.StartsWith("cap:")).ToArray();Assert.NotEmpty(cap);
  Assert.All(cap,e=>{Assert.InRange(graph.Nodes[e.A].Z,-4,-3.99);Assert.InRange(graph.Nodes[e.B].Z,-4,-3.99);});
  // The old X=9.004 cap is replaced, not retained as an additional parallel loop.
  Assert.DoesNotContain(cap,e=>Math.Abs(graph.Nodes[e.A].X-9.004)<.001&&Math.Abs(graph.Nodes[e.B].X-9.004)<.001);
  var route=ReviewedVCarveSourceRouter.Route(graph,new(6,4,4,2));
  Assert.Equal(graph.Edges.Length,route.Accounting.CoveredEdges);
  Assert.Equal(1,route.Accounting.Components);
 }
 [Fact] public void SolidStockUsesTheSamePhysicalDepthOffset()
 {
  var front=Owner(0).DepthFront(new(0,Boundary,Boundary));Assert.Single(front);
  Assert.InRange(front[0].Min(p=>p.x),9.0039,9.0041);
 }
 [Fact] public void TranslationDoesNotChangeTheStockContactRule()
 {
  PointD Shift(PointD p)=>new(p.x+30,p.y-7);
  var b=Boundary.Select(r=>r.Select(Shift).ToArray()).ToArray();
  var owner=new ReviewedVCarveStockContact([b],Cutter,new(4,4){SurfaceZ=6},
    new([new(new(30,3,-2),new(50,3,-2),20,20)],[2],Part.Select(Shift).ToArray()),.03);
  var front=owner.DepthFront(new(0,b,b));Assert.Single(front);
  Assert.InRange(front[0].Min(p=>p.x),37.0039,37.0041);Assert.InRange(front[0].Min(p=>p.y),.0039,.0041);
 }
}
