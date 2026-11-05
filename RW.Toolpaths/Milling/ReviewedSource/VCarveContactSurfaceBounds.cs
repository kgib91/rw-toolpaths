using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;

// Diagnostic interval measurement, not a new surface-finish setting or stock credit.
namespace RW.Toolpaths.Milling.ReviewedSource;
internal static class VCarveContactSurfaceBounds
{
 internal sealed record Bounds(double Lower,double Upper,int Intervals,bool Resolved);
 internal static Bounds Measure(PointD q,VCarveClearedStock stock,double resolution=0.001,int maximumIntervals=500_000)
 {
  const double arithmetic=1e-8;
  double best=0;int intervals=0;
  var pending=new PriorityQueue<(VCarveRoughingChord Chord,VCarveCutterProfile Profile,double A,double B),double>();
  double Closest(Point3D a,Point3D b,double low,double high)
  {double dx=b.X-a.X,dy=b.Y-a.Y,d=dx*dx+dy*dy;return d==0?low:Math.Clamp(((q.x-a.X)*dx+(q.y-a.Y)*dy)/d,low,high);}
  double Distance(Point3D a,Point3D b,double t)=>Math.Sqrt(Math.Pow(q.x-a.X-t*(b.X-a.X),2)+Math.Pow(q.y-a.Y-t*(b.Y-a.Y),2));
  double At(VCarveRoughingChord c,VCarveCutterProfile p,double t)=>-(c.A.Z+t*(c.B.Z-c.A.Z))-p.PhysicalProfile.Bottom(Distance(c.A,c.B,t));
  void Add(VCarveRoughingChord c,VCarveCutterProfile p,double low,double high)
  {
   double near=Closest(c.A,c.B,low,high),d0=-(c.A.Z+low*(c.B.Z-c.A.Z)),d1=-(c.A.Z+high*(c.B.Z-c.A.Z));
   double upper=Math.Max(d0,d1)-p.PhysicalProfile.Bottom(Distance(c.A,c.B,near))+arithmetic;
   if(upper<=best+resolution)return;
   best=Math.Max(best,Math.Max(At(c,p,low),Math.Max(At(c,p,high),At(c,p,near)))-arithmetic);
   if(upper>best+resolution)pending.Enqueue((c,p,low,high),-upper);
  }
  foreach(var c in stock.Chords)
  {
   double radius=c.ClearedColumnRadius;if(radius<=0)continue;
   double dx=c.B.X-c.A.X,dy=c.B.Y-c.A.Y,length2=dx*dx+dy*dy;
   double at=Closest(c.A,c.B,0,1),distance=Distance(c.A,c.B,at);if(distance>radius)continue;
   double low=0,high=1;
   if(length2>0){double center=((q.x-c.A.X)*dx+(q.y-c.A.Y)*dy)/length2;double perpendicular2=Math.Pow(q.x-c.A.X-center*dx,2)+Math.Pow(q.y-c.A.Y-center*dy,2);double half=Math.Sqrt(Math.Max(0,radius*radius-perpendicular2)/length2);low=Math.Max(0,center-half);high=Math.Min(1,center+half);}
   best=Math.Max(best,-(c.A.Z+(c.B.Z-c.A.Z)*(c.B.Z<c.A.Z?high:low))-arithmetic);
  }
  foreach(var sweep in stock.ProfileSweeps??Array.Empty<VCarveClearedSweep>())foreach(var c in sweep.Chords)Add(c,sweep.Profile,0,1);
  double lastUpper=best+resolution;
  while(pending.TryDequeue(out var span,out double priority))
  {
   lastUpper=Math.Max(best,-priority);if(lastUpper<=best+resolution)break;
   if(intervals++>=maximumIntervals)return new(best,lastUpper,intervals,false);
   double middle=(span.A+span.B)/2;
   Add(span.Chord,span.Profile,span.A,middle);Add(span.Chord,span.Profile,middle,span.B);
  }
  return new(best,best+resolution,intervals,true);
 }
}
