using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class VCarveNativeSeparationIndexTests
{
 [Theory]
 [InlineData(0,1)] [InlineData(1000,1)] [InlineData(9000,.01)] [InlineData(-9000,10)]
 public void IndexedMinimumEqualsLiteralOracleAcrossHolesDepthsAndCoordinates(double origin,double scale)
 {
  PointD[] Ring(int count,double x,double y,double rx,double ry,bool reverse=false){var r=Enumerable.Range(0,count).Select(i=>new PointD(origin+scale*(x+rx*Math.Cos(2*Math.PI*i/count)),origin+scale*(y+ry*Math.Sin(2*Math.PI*i/count)))).ToArray();return reverse?r.Reverse().ToArray():r;}
  IReadOnlyList<PointD>[] boundary=[Ring(257,0,0,20,15),Ring(73,2,1,3,2,true),Ring(41,-12,-3,1.2,1.5,true)];
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,.4*scale,.4*scale),new(CarvingCutterSectionKind.Linear,.4*scale,.9*scale,scale)],1.2*scale);
  var indexed=new VCarveNativeContainment(boundary,profile);var literal=new VCarveNativeContainmentReference(boundary,profile);var random=new Random(83719);
  void Compare(Point3D a,Point3D b){double expected=literal.MinimumSquaredSeparation(a,b),actual=indexed.MinimumSquaredSeparation(a,b);Assert.True(expected==actual,$"Literal={expected:R}, indexed={actual:R}, a={a}, b={b}");}
  for(int i=0;i<1200;i++){
   var a=new Point3D(origin+(random.NextDouble()*48-24)*scale,origin+(random.NextDouble()*38-19)*scale,(random.NextDouble()*1.4-1.2)*scale);
   var b=i%4==0?a:i%4==1?a with{Z=-random.NextDouble()*1.2*scale}:new Point3D(origin+(random.NextDouble()*48-24)*scale,origin+(random.NextDouble()*38-19)*scale,(random.NextDouble()*1.4-1.2)*scale);
   Compare(a,b);
  }
  foreach(double epsilon in new[]{-1e-9,0,1e-9,.002})Compare(new(origin+(20+epsilon)*scale,origin,-.2*scale),new(origin+(19+epsilon)*scale,origin+scale,-.6*scale));
 }
 [Fact] public void DenseBoundaryQueryPrunesOnlyWhollySeparatedSubtreesAndRetainsFiniteMargin()
 {
  const int count=40000;var ring=Enumerable.Range(0,count).Select(i=>new PointD(1000*Math.Cos(i*2*Math.PI/count),1000*Math.Sin(i*2*Math.PI/count))).ToArray();
  var segments=Enumerable.Range(0,count).Select(i=>(A:ring[i],B:ring[(i+1)%count])).ToArray();var index=new MedialAxisToolpaths.SpatialSegmentIndex(segments);
  void Compare(Point3D a,Point3D b,double radius,bool requirePruning)
  {
   int calls=0;double x0=Math.Min(a.X,b.X),x1=Math.Max(a.X,b.X),y0=Math.Min(a.Y,b.Y),y1=Math.Max(a.Y,b.Y);
   double Evaluate((PointD A,PointD B) e){calls++;double left=Math.Min(e.A.x,e.B.x),right=Math.Max(e.A.x,e.B.x),bottom=Math.Min(e.A.y,e.B.y),top=Math.Max(e.A.y,e.B.y);
    if(left<=x1+radius&&right>=x0-radius&&bottom<=y1+radius&&top>=y0-radius)return -1;
    double dx=Math.Max(0,Math.Max(left-x1,x0-right)),dy=Math.Max(0,Math.Max(bottom-y1,y0-top));return dx*dx+dy*dy-radius*radius;
   }
   double expected=segments.Min(Evaluate);calls=0;double actual=index.QueryCutterSeparation(a,b,radius,Evaluate);
   Assert.Equal(expected,actual);Assert.True(double.IsFinite(actual));if(requirePruning)Assert.True(calls<count/10,$"Evaluated {calls}/{count} edges");
  }
  Compare(new(999.9,0,-.1),new(999.9,.1,-.2),.2,true);
  Compare(new(0,0,-.1),new(.1,0,-.2),.2,false);
 }
 [Fact] public void IndexedAnalyticMinimumMatchesLiteralForDenseNearWallAndRemoteCurves()
 {
  PointD[] ring=Enumerable.Range(0,12000).Select(i=>new PointD(20*Math.Cos(i*2*Math.PI/12000),15*Math.Sin(i*2*Math.PI/12000))).ToArray();
  var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.5,1)],1);
  var a=new VCarveNativeContainment([ring],profile);var b=new VCarveNativeContainmentReference([ring],profile);
  for(int i=0;i<24;i++){double t=i*2*Math.PI/24;var p=new Point3D(19.7*Math.Cos(t),14.7*Math.Sin(t),-.5);var q=p with{X=p.X+.02,Y=p.Y-.03,Z=-.8};Assert.Equal(b.MinimumSquaredSeparation(p,q),a.MinimumSquaredSeparation(p,q));}
 }
}
