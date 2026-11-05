using Clipper2Lib;
using RW.Toolpaths.Geometry;
namespace RW.Toolpaths.Milling.ReviewedSource;

// A source nomination, never a stock/curve certificate. The existing complete
// native and removal validators alone authorize output. Search samples do not
// become motion primitives or mandatory fit joins.
internal static class VCarveStockContactRidge
{
 internal enum Domain { Connected, SourceEnvelopeConnected }
 internal sealed record Nomination(Point3D[] Centers,int Trials);
 internal static Nomination Nominate(IReadOnlyList<Point3D> nominal,
  IReadOnlyList<IReadOnlyList<PointD>> boundary,VCarveCutterProfile profile,
  VCarveNativeFitSettings settings,Func<Point3D,Point3D> contact,
  Func<ToolMove,bool> cutter,int maximumTrials,int sectionSamples,int refinementSteps,
  CancellationToken token,Action<string>? diagnostic,Domain domain)
 {
  if(nominal.Count<2||maximumTrials<=0||sectionSamples<3||refinementSteps<0)throw new ArgumentException("Invalid contact-ridge search inputs.");
  settings.Validate();var allowed=new PreparedRegionDistance(boundary);int trials=0;var result=new List<Point3D>();
  void Trial(){token.ThrowIfCancellationRequested();if(++trials>maximumTrials)throw new InvalidOperationException("CONTACT_RIDGE_POSE_BUDGET_EXHAUSTED: no old-source substitution.");}
  for(int station=0;station<nominal.Count;station++)
  {
   token.ThrowIfCancellationRequested();var p=nominal[station];var before=nominal[Math.Max(0,station-1)];var after=nominal[Math.Min(nominal.Count-1,station+1)];
   double tx=after.X-before.X,ty=after.Y-before.Y,length=Math.Sqrt(tx*tx+ty*ty);
   if(!double.IsFinite(length)||length==0)throw new InvalidOperationException("CONTACT_RIDGE_DIRECTION_UNRESOLVED");
   double nx=-ty/length,ny=tx/length;var cuts=new List<double>();
   foreach(var ring in boundary)for(int edge=0;edge<ring.Count;edge++)
   {
    var a=ring[edge];var b=ring[(edge+1)%ring.Count];double dx=b.x-a.x,dy=b.y-a.y,den=nx*dy-ny*dx;
    // Arithmetic parallelism test for nomination only, not an angular policy.
    if(Math.Abs(den)<1e-14)continue;
    double u=((a.x-p.X)*dy-(a.y-p.Y)*dx)/den,v=((a.x-p.X)*ny-(a.y-p.Y)*nx)/den;
    if(v>=0&&v<=1)cuts.Add(u);
   }
   double left=cuts.Where(u=>u<0).DefaultIfEmpty(double.NaN).Max(),right=cuts.Where(u=>u>0).DefaultIfEmpty(double.NaN).Min();
   if(!double.IsFinite(left+right)||left>=right)throw new InvalidOperationException("CONTACT_RIDGE_REGION_OWNERSHIP_UNRESOLVED");
   Trial();var original=contact(p);
   double ConnectedExtent(double end)
   {
    double low=0,high=Math.Abs(end),sign=Math.Sign(end);
    for(int step=0;step<refinementSteps;step++)
    {
     Trial();double middle=(low+high)/2;var q=new Point3D(p.X+sign*middle*nx,p.Y+sign*middle*ny,original.Z);
     // Owning XY component alone is insufficient for a finite cutter. The
     // constant-depth normal connection must earn the actual stock proof.
     if(cutter(ToolMove.Line(original,q)))low=middle;else high=middle;
    }
    return sign*low;
   }
   left=ConnectedExtent(left);right=ConnectedExtent(right);
   var evaluated=new Dictionary<double,Point3D>{{0,original}};var rejected=new HashSet<double>();
   Point3D? Evaluate(double u)
   {
    if(evaluated.TryGetValue(u,out var known))return known;if(rejected.Contains(u))return null;
    var q=new Point3D(p.X+u*nx,p.Y+u*ny,p.Z);
    if(allowed.SignedDistance(new(q.X,q.Y))<=PreparedRegionDistance.Uncertainty)return null;
    Trial();var pose=contact(q);
    // A candidate for this source station must retain its original cutter
    // envelope to the nomination allowance, rather than jump to a deeper
    // unrelated arm. This is a sufficient search filter. The full operation
    // removal certificate still uses the selected, unsplit allowance.
    if(domain==Domain.SourceEnvelopeConnected&&!VCarveStockContactCutterInclusion.CoversWithin(original,pose,profile,settings.FitErrorMm/2))
    {rejected.Add(u);return null;}
    evaluated.Add(u,pose);return pose;
   }
   double Value(double u)=>Evaluate(u) is {} pose?-pose.Z:double.NegativeInfinity;
   var grid=Enumerable.Range(0,sectionSamples).Select(i=>left+(right-left)*i/(sectionSamples-1)).Append(0).Distinct().Order().ToArray();var values=grid.Select(Value).ToArray();
   for(int i=1;i<grid.Length-1;i++)if(values[i]>=values[i-1]&&values[i]>=values[i+1])
   {
    double low=grid[i-1],high=grid[i+1],ratio=(Math.Sqrt(5)-1)/2,u=high-ratio*(high-low),v=low+ratio*(high-low),fu=Value(u),fv=Value(v);
    for(int step=0;step<refinementSteps;step++)
     if(fu>=fv){high=v;v=u;fv=fu;u=high-ratio*(high-low);fu=Value(u);}
     else{low=u;u=v;fu=fv;v=low+ratio*(high-low);fv=Value(v);}
   }
   var chosen=evaluated.OrderBy(k=>k.Value.Z).ThenBy(k=>Math.Abs(k.Key)).First();result.Add(chosen.Value);
  }
  diagnostic?.Invoke($"stage=contact-ridge domain={domain} stations={result.Count} poseTrials={trials} connectedFullCutterDomain=True oldEnvelopeIsSearchSeedOnly=True selectedFitAllowanceMm={settings.FitErrorMm:R} nativeProofStillRequired=True");
  return new(result.ToArray(),trials);
 }
}
