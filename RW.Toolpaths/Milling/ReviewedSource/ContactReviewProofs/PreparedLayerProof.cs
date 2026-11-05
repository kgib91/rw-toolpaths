using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Milling.ReviewedSource.ContactReviewProofs;
internal sealed class PreparedLayerProof
{
 internal sealed record Edge(PointD A,PointD B,double L,double R,double Btm,double T,double Ex,double Ey,double Length2);
 internal sealed record Result(bool Certified,string Reason,double MinimumSquaredMargin,int RelevantEdges);
 readonly Edge[] edges;readonly PreparedRegionDistance region;readonly VCarveCutterProfile profile;
 internal double Top {get;} internal double Bottom {get;}
 static readonly double[] C3=[1,3,3,1],C6=[1,6,15,20,15,6,1];
 static double[] Multiply(double[] a,double[] b){var v=new double[7];for(int i=0;i<4;i++)for(int j=0;j<4;j++)v[i+j]+=C3[i]*C3[j]/C6[i+j]*a[i]*b[j];return v;}
 internal PreparedLayerProof(IReadOnlyList<IReadOnlyList<PointD>> rings,VCarveCutterProfile cutter,double top,double bottom)
 {region=new(rings);profile=cutter;Top=top;Bottom=bottom;edges=rings.SelectMany(r=>Enumerable.Range(0,r.Count).Select(i=>{var a=r[i];var b=r[(i+1)%r.Count];double ex=b.x-a.x,ey=b.y-a.y;return new Edge(a,b,Math.Min(a.x,b.x),Math.Max(a.x,b.x),Math.Min(a.y,b.y),Math.Max(a.y,b.y),ex,ey,ex*ex+ey*ey);})).Where(e=>e.Length2>0).ToArray();}
 internal Result Certify(ToolMove m)
 {
  const double arithmetic=1e-10;Point3D[] c=[m.Start,m.Control1,m.Control2,m.End];var h=c.Select(p=>-p.Z-Top).ToArray();double hmin=h.Min(),hmax=h.Max();
  if(hmax<=0)return new(true,"above-stock-band",double.PositiveInfinity,0);
  if(hmin<0)return new(false,"layer-zero-crossing-needs-partition",double.NegativeInfinity,0);
  if(region.SignedDistance(new(c[0].X,c[0].Y))<=0||region.SignedDistance(new(c[^1].X,c[^1].Y))<=0)return new(false,"center-not-inside-free-region",double.NegativeInfinity,0);
  // Each actual height belongs to a participating physical section. Proving
  // every participating polynomial extension is a conservative maximum across
  // section crossings, not a sample or a truncated cutter profile.
  var radiusPolynomials=new List<double[]>();double height=0;
  foreach(var s in profile.Sections)
  {
   if(hmax>=height&&hmin<=height+s.Height)
   {
    if(s.Kind==CarvingCutterSectionKind.Hemisphere){var local=h.Select(x=>x-height).ToArray();var sq=Multiply(local,local);var up=Multiply(local,[1,1,1,1]);radiusPolynomials.Add(up.Select((x,i)=>2*s.TopRadius*x-sq[i]).ToArray());}
    else{double slope=(s.TopRadius-s.BottomRadius)/s.Height;var radii=h.Select(x=>s.BottomRadius+(x-height)*slope).ToArray();radiusPolynomials.Add(Multiply(radii,radii));}
   }
   height+=s.Height;
  }
  if(radiusPolynomials.Count==0)return new(false,"unsupported-profile-height",double.NegativeInfinity,0);
  double xmin=c.Min(p=>p.X),xmax=c.Max(p=>p.X),ymin=c.Min(p=>p.Y),ymax=c.Max(p=>p.Y),rmax=profile.PhysicalRadiusAtDepth(hmax),minimum=double.PositiveInfinity;int relevant=0;
  foreach(var e in edges)
  {
   double dx=Math.Max(0,Math.Max(e.L-xmax,xmin-e.R)),dy=Math.Max(0,Math.Max(e.Btm-ymax,ymin-e.T)),box=dx*dx+dy*dy-rmax*rmax-arithmetic;
   if(box>=0){minimum=Math.Min(minimum,box);continue;}
   relevant++;var ux=c.Select(p=>p.X-e.A.x).ToArray();var uy=c.Select(p=>p.Y-e.A.y).ToArray();var proj=ux.Select((x,i)=>(x*e.Ex+uy[i]*e.Ey)/e.Length2).ToArray();double[] dist;
   if(proj.Max()<=0){var xx=Multiply(ux,ux);var yy=Multiply(uy,uy);dist=xx.Select((x,i)=>x+yy[i]).ToArray();}
   else if(proj.Min()>=1){var vx=c.Select(p=>p.X-e.B.x).ToArray();var vy=c.Select(p=>p.Y-e.B.y).ToArray();var xx=Multiply(vx,vx);var yy=Multiply(vy,vy);dist=xx.Select((x,i)=>x+yy[i]).ToArray();}
   else{var cross=ux.Select((x,i)=>e.Ex*uy[i]-e.Ey*x).ToArray();dist=Multiply(cross,cross).Select(x=>x/e.Length2).ToArray();}
   foreach(var rr in radiusPolynomials){double lower=dist.Select((x,i)=>x-rr[i]).Min()-arithmetic;minimum=Math.Min(minimum,lower);if(lower<0)return new(false,"Bernstein-enclosure-unresolved",minimum,relevant);}
  }
  return new(true,"complete-polynomial-layer",minimum,relevant);
 }
}
