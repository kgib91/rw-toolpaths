using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Milling.ReviewedSource.ContactReviewProofs;
internal static class AnalyticCapsuleCover
{
 internal sealed record Capsule(int ActualStockChord,Point3D A,Point3D B,double Radius);
 internal sealed record Cell(double L,double B,double R,double T,int Level);
 internal sealed record Result(bool Certified,int Cells,int TargetLeaves,int PriorCapsuleLeaves,int OutsideToolLeaves,int MaximumLevel,int[] ActualStockChordsUsed,object? Unresolved,double ExtraXYCorridorMm);
 static double DistanceSquared(double x,double y,Point3D a,Point3D b)
 {double ex=b.X-a.X,ey=b.Y-a.Y,l2=ex*ex+ey*ey,t=l2==0?0:Math.Clamp(((x-a.X)*ex+(y-a.Y)*ey)/l2,0,1);double dx=x-a.X-t*ex,dy=y-a.Y-t*ey;return dx*dx+dy*dy;}
 internal static Result Prove(ToolMove m,VCarveCutterProfile profile,IReadOnlyList<Capsule> caps,PreparedRegionDistance target,PreparedRegionDistance part,double top,double extra,CancellationToken token,int maximumCells=100000)
 {
  var d0=VCarveNativeCurveMath.Second(m,0);var d1=VCarveNativeCurveMath.Second(m,1);double xy=Math.Max(Math.Sqrt(d0.X*d0.X+d0.Y*d0.Y),Math.Sqrt(d1.X*d1.X+d1.Y*d1.Y))/8+extra,z=Math.Max(Math.Abs(d0.Z),Math.Abs(d1.Z))/8;
  var a=m.Start with{Z=m.Start.Z-z};var b=m.End with{Z=m.End.Z-z};double r=profile.PhysicalRadiusAtDepth(Math.Max(-a.Z,-b.Z)-top)+xy;
  double left=Math.Min(a.X,b.X)-r,right=Math.Max(a.X,b.X)+r,bottom=Math.Min(a.Y,b.Y)-r,end=Math.Max(a.Y,b.Y)+r;
  var pending=new Stack<Cell>();for(double y=Math.Floor(bottom/16)*16;y<=end;y+=16)for(double x=Math.Floor(left/16)*16;x<=right;x+=16)pending.Push(new(x,y,x+16,y+16,0));
  var used=new HashSet<int>();int visited=0,allowed=0,prior=0,outside=0,maxLevel=0;
  Result Fail(object reason)=>new(false,visited,allowed,prior,outside,maxLevel,used.Order().ToArray(),reason,extra);
  while(pending.TryPop(out var cell))
  {
   token.ThrowIfCancellationRequested();if(++visited>maximumCells)return Fail(new{Reason="explicit-cell-work-budget",cell});maxLevel=Math.Max(maxLevel,cell.Level);
   if(cell.L>right||cell.R<left||cell.B>end||cell.T<bottom)continue;
   if(!VCarveCutterBoxIntersection.Intersects(a,b,profile,top,cell.L-xy,cell.B-xy,cell.R+xy,cell.T+xy)){outside++;continue;}
   double x=(cell.L+cell.R)/2,y=(cell.B+cell.T)/2,cover=Math.Sqrt(Math.Pow(cell.R-cell.L,2)+Math.Pow(cell.T-cell.B,2))/2;
   double pd=part.SignedDistanceUncached(new(x,y)),td=target.SignedDistanceUncached(new(x,y));
   if(pd>=cover+PreparedRegionDistance.Uncertainty&&td>=cover+PreparedRegionDistance.Uncertainty){allowed++;continue;}
   bool covered=false;double best=double.NegativeInfinity;int winner=-1;
   if(pd>=cover+PreparedRegionDistance.Uncertainty)foreach(var c in caps)
   {
    double radius=c.Radius-PostedSweepInnerRegion.Reserve-1e-8;if(radius<=0)continue;
    double far=Math.Max(Math.Max(DistanceSquared(cell.L,cell.B,c.A,c.B),DistanceSquared(cell.L,cell.T,c.A,c.B)),Math.Max(DistanceSquared(cell.R,cell.B,c.A,c.B),DistanceSquared(cell.R,cell.T,c.A,c.B)));
    double margin=radius-Math.Sqrt(far);if(margin>best){best=margin;winner=c.ActualStockChord;}
    // Distance to a convex segment is convex; the maximum over a rectangle
    // is bounded by its four corners. This covers every point of the cell.
    if(margin>=0){covered=true;used.Add(c.ActualStockChord);break;}
   }
   if(covered){prior++;continue;}
   if(cell.Level>=24)return Fail(new{Reason="unresolved-closed-cell",cell,TargetDistance=td,PartDistance=pd,NearestReservedCapsuleWholeCellMargin=best,ActualStockChord=winner});
   int level=cell.Level+1;pending.Push(new(cell.L,cell.B,x,y,level));pending.Push(new(x,cell.B,cell.R,y,level));pending.Push(new(cell.L,y,x,cell.T,level));pending.Push(new(x,y,cell.R,cell.T,level));
  }
  return new(true,visited,allowed,prior,outside,maxLevel,used.Order().ToArray(),null,extra);
 }
}
