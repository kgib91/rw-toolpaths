using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.ReviewedSource;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class ReviewedVCarveStockContactTests
{
 static PointD[] Box(double a,double b,double c,double d)=>[new(a,b),new(c,b),new(c,d),new(a,d)];
 static readonly PointD[] Target=Box(5,5,25,25),Part=Box(-50,-50,50,50);
 static readonly VCarveCutterProfile Cutter=new([new(CarvingCutterSectionKind.Linear,0,12.7,12.7)],12.7);
 static readonly DepthSchedule Job=new(12.7,2){SurfaceZ=12.7,StartDepth=6};
 static VCarveClearedStock Flat(double floor,double radius=100)=>new([new(new(15,15,-floor),new(15,15,-floor),radius,radius)],[floor],Part);
 static ReviewedVCarveConstructor.Result Graph()=>new(0,[new(5,15,0),new(10,15,-5)],[new(0,1,"same-nominal-edge")],1,0,0,0,0);
 static ReviewedVCarveStockContact Contact(VCarveClearedStock? stock)=>new(new IReadOnlyList<IReadOnlyList<PointD>>[]{new IReadOnlyList<PointD>[]{Target}},Cutter,Job,stock,.01);
 static VCarveMaterialSurface Old(VCarveClearedStock stock)=>new([Target],stock,Cutter,12.7,12.7,default,null);
 static Point3D OldContact(VCarveMaterialSurface owner,Point3D p)=>(Point3D)typeof(VCarveMaterialSurface).GetMethod("Fit",BindingFlags.Instance|BindingFlags.NonPublic)!.Invoke(owner,[p,false])!;
 [Fact] public void UniformRoughingPortsTheEstablishedKernelBeforeRouting()
 {
  var source=Graph();var stock=Flat(6);var (projected,receipt)=Contact(stock).Project(source);var legacy=Old(stock);
  for(int i=0;i<source.Nodes.Length;i++)Assert.Equal(OldContact(legacy,source.Nodes[i]),projected.Nodes[receipt.NodeMap[i]]);
  Assert.Equal(0,source.Nodes[0].Z);Assert.InRange(projected.Nodes[receipt.NodeMap[0]].Z,-6,-5.998);
  var routed=ReviewedVCarveSourceRouter.Route(projected,new(12.7,12.7,2,6));
  Assert.All(routed.Paths.SelectMany(p=>p.Points),p=>Assert.True(p.Z-12.7<=-5.998));
 }
 [Fact] public void HigherResidualAndPartPolicyCanRaiseContactWithoutGlobalClamp()
 {
  var source=Graph();var high=Contact(Flat(6,3)).Project(source);
  Assert.Equal(0,high.Graph.Nodes[high.Evidence.NodeMap[0]].Z);
  var stock=Flat(6)with{PartBoundary=Box(0,0,30,30)};var limited=Contact(stock).Project(source);
  Assert.InRange(limited.Graph.Nodes[limited.Evidence.NodeMap[0]].Z,-5,-4.99);
  Assert.Equal(OldContact(Old(stock),source.Nodes[0]),limited.Graph.Nodes[limited.Evidence.NodeMap[0]]);
 }
 [Fact] public void SlopedPriorChordsUseContinuousStockAndPreserveEdgeProvenance()
 {
  var stock=new VCarveClearedStock([new(new(5,15,-2),new(25,15,-6),5,5)],[2,6],Part);
  var source=Graph();var (projected,receipt)=Contact(stock).Project(source);var owner=Old(stock);
  foreach(var e in projected.Edges){Assert.Equal("same-nominal-edge",e.Owner);Assert.True(owner.IsLegalNativeChord(projected.Nodes[e.A],projected.Nodes[e.B]));}
  Assert.Equal(projected.Edges.Length,receipt.EdgeMap.Single().Length);
  Assert.Equal(OldContact(Old(stock),source.Nodes[0]),projected.Nodes[receipt.NodeMap[0]]);
 }
 [Fact] public void MissingRemovalEvidenceDoesNotCreditDeclaredStartingDepth()
 {
  var source=Graph();var (original,receipt)=Contact(null).Project(source);
  Assert.Same(source,original);Assert.False(receipt.HasActualStock);Assert.Equal(0,original.Nodes[0].Z);
  var empty=new VCarveClearedStock([],[],Part);var projected=Contact(empty).Project(source);
  Assert.Equal(0,projected.Graph.Nodes[projected.Evidence.NodeMap[0]].Z);
 }
 [Fact] public void AlreadyClearedFloorUsesDepthCapInsteadOfInventingSurfaceAirPath()
 {
  var source=Graph();var (projected,receipt)=Contact(Flat(12.7)).Project(source);
  Assert.All(projected.Nodes,p=>Assert.Equal(-12.7,p.Z));
  Assert.Equal(OldContact(Old(Flat(12.7)),source.Nodes[0]),projected.Nodes[receipt.NodeMap[0]]);
 }
 [Fact] public void SharedJunctionIsProjectedOnceAcrossCanonicalBranchVisits()
 {
  var source=new ReviewedVCarveConstructor.Result(0,[new(10,15,-5),new(5,15,0),new(10,20,0),new(15,15,-10)],
   [new(0,1,"one"),new(0,2,"two"),new(0,3,"three")],3,0,0,0,0);
  var (projected,receipt)=Contact(Flat(6)).Project(source);var joint=projected.Nodes[receipt.NodeMap[0]];
  Assert.Equal(source.Nodes[0].X,joint.X);Assert.Equal(source.Nodes[0].Y,joint.Y);
  var route=ReviewedVCarveSourceRouter.Route(projected,new(12.7,12.7,2,12.7));
  Assert.True(route.Accounting.ReturnEdges>0);
  Assert.All(route.Paths.SelectMany(p=>p.Points).Where(p=>p.X==joint.X&&p.Y==joint.Y),p=>Assert.Equal(12.7+joint.Z,p.Z));
 }
}
