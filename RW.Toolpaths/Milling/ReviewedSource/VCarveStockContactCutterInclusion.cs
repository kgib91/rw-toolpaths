namespace RW.Toolpaths.Milling.ReviewedSource;

// Sufficient search filter in stock-relative coordinates, not execution proof.
// Each profile-section interval is monotone; endpoint bounds enclose its disks.
// A horizontal disk allowance is within the same Euclidean 3D allowance.
internal static class VCarveStockContactCutterInclusion
{
 internal static bool CoversWithin(Point3D oldPose,Point3D newPose,VCarveCutterProfile profile,double allowance)
 {
  if(newPose.Z>oldPose.Z||!double.IsFinite(allowance)||allowance<=0)return false;
  if(profile.Sections.Any(s=>(int)s.Kind!=0&&(int)s.Kind!=1))return false;
  double shift=Math.Sqrt(Math.Pow(newPose.X-oldPose.X,2)+Math.Pow(newPose.Y-oldPose.Y,2)),delta=oldPose.Z-newPose.Z,limit=Math.Max(0,-oldPose.Z);
  var breaks=new List<double>{0,limit};double height=0;
  foreach(var section in profile.Sections)
  {
   height+=section.Height;if(height>0&&height<limit)breaks.Add(height);
   double other=height-delta;if(other>0&&other<limit)breaks.Add(other);
  }
  var knots=breaks.Distinct().Order().ToArray();int remaining=4096;
  double Radius(double h)=>profile.PhysicalRadiusAtDepth(Math.Max(0,h));
  bool Prove(double a,double b,int depth)
  {
   double oldUpper=Math.Max(Radius(a),Radius(b)),newLower=Math.Min(Radius(a+delta),Radius(b+delta));
   if(newLower+allowance>=oldUpper+shift+VCarveNativeGeometricProof.ArithmeticReserve)return true;
   double mid=(a+b)/2;
   if(new[]{a,mid,b}.Any(h=>Radius(h+delta)+allowance<Radius(h)+shift))return false;
   if(depth>=14||--remaining<0)return false;
   return Prove(a,mid,depth+1)&&Prove(mid,b,depth+1);
  }
  return limit==0?Radius(delta)+allowance>=shift+VCarveNativeGeometricProof.ArithmeticReserve:Enumerable.Range(1,knots.Length-1).All(i=>Prove(knots[i-1],knots[i],0));
 }
}
