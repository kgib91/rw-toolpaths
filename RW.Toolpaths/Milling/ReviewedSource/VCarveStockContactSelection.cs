using Clipper2Lib;
namespace RW.Toolpaths.Milling.ReviewedSource;

// Bounded geometric search with one acceptance contract. A failed nomination
// never substitutes source motion. Contact samples rank accepted candidates;
// they are not a new finish requirement or a continuous coverage certificate.
internal static class VCarveStockContactSelection
{
 internal sealed record Candidate(VCarveStockContactRidge.Domain Domain,Point3D[] Guide,
  VCarveContactNativeConstruction.Result Native,double ContactWitnessUpperMm);
 internal static Candidate Construct(Point3D[] nominal,Point3D[] reference,
  IReadOnlyList<IReadOnlyList<PointD>> boundary,VCarveCutterProfile profile,VCarveClearedStock stock,
  VCarveNativeFitSettings settings,double pathTolerance,VCarveMaterialSurface authority,
  ReviewedVCarveContactSearchLimits limits,CancellationToken token,Action<string>? log)
 {
  var accepted=new List<Candidate>();var rejected=new List<string>();
  var targets=VCarveContactLocus.ContactSites(nominal,boundary,(x,y)=>
  {
   var bounds=VCarveContactSurfaceBounds.Measure(new(x,y),stock,pathTolerance,limits.MaximumPoseTrials);
   if(!bounds.Resolved)log?.Invoke("stage=contact-selection event=contact-measurement-unresolved conservativeBoundsRetained=True rankingOnly=True noExactSurfaceClaim=True");
   return(bounds.Lower,bounds.Upper);
  }).SelectMany(p=>new[]{p.Left,p.Right}).Where(s=>s.StockDepthLower<profile.MaximumDepth)
   .DistinctBy(s=>(s.Point.x,s.Point.y)).ToArray();
  var domains=Enum.GetValues<VCarveStockContactRidge.Domain>();
  int remainingPoseTrials=limits.MaximumPoseTrials,remainingNativeTrials=limits.MaximumNativeTrials;
  foreach(var domain in domains)
  {
   token.ThrowIfCancellationRequested();
   int poseAllocation=remainingPoseTrials/(domains.Length-(int)domain),nativeAllocation=Math.Min(limits.MaximumNativeTrialsPerNomination,remainingNativeTrials/(domains.Length-(int)domain));
   if(poseAllocation<=0||nativeAllocation<=0){rejected.Add($"{domain}:search budget exhausted");continue;}
   var nomination=VCarveStockContactRidge.Nominate(nominal,boundary,profile,settings,authority.ProjectContactPose,
    authority.IsLegalNativeCurve,poseAllocation,9,16,token,log,domain);
   remainingPoseTrials-=nomination.Trials;
   var centers=nomination.Centers;centers[0]=reference[0];centers[^1]=reference[^1];
   var guide=new List<Point3D>{centers[0]};
   foreach(var p in centers.Skip(1))if(p!=guide[^1])
    guide.AddRange(authority.ProjectCanonicalContactEdge(guide[^1],p,guide[^1],p,pathTolerance).Skip(1));
   double length=Enumerable.Range(1,guide.Count-1).Sum(i=>VCarveNativeCurveMath.Distance(guide[i-1],guide[i]));
   int maximumPieces=limits.MaximumNativePieces??Math.Max(1,Math.Min(guide.Count-1,(int)Math.Floor(length/settings.MinimumPrimitiveLengthMm)));
   int trials=nativeAllocation;double aim=settings.FitErrorMm;VCarveContactNativeConstruction.Result native;
   bool removal=false;
   do
   {
    native=VCarveContactNativeConstruction.ConstructAdaptiveContact(guide.ToArray(),settings with{FitErrorMm=aim},profile.MaximumDepth,
     authority.ProjectContactPose,authority.IsLegalNativeCurve,maximumPieces,log,
     (m,ports,available)=>VCarveContactCurveCorridor.Find(m,ports,authority.IsLegalNativeCurve,profile.MaximumDepth,available),trials);
    trials-=native.CandidateFamilies;
    if(!native.Verified)break;
    removal=VCarveNativeCurvePlanning.VerifyRemovalDeviation(reference,native.Moves,stock,profile,settings.FitErrorMm,token,log);
    if(removal)break;
    aim/=2;
   }while(trials>0&&aim>VCarveNativeGeometricProof.ArithmeticReserve);
   remainingNativeTrials-=nativeAllocation-Math.Max(0,trials);
   if(!native.Verified||!removal)
   {rejected.Add($"{domain}:native={native.Verified},removal={removal},perNominationBudget={nativeAllocation},rejections={System.Text.Json.JsonSerializer.Serialize(native.Rejections)}");log?.Invoke($"stage=contact-selection event=rejected domain={domain} native={native.Verified} removal={removal}; no source substitution");continue;}
   double quality=WitnessUpper(native.Moves,targets,profile,32);
   accepted.Add(new(domain,guide.ToArray(),native,quality));
   log?.Invoke($"stage=contact-selection event=candidate domain={domain} primitives={native.Moves.Length} fitBoundMm={native.ErrorBound:R} contactNominationWitnessUpperMm={quality:R} contactIsRankingNotFinishRequirement=True nativeAndStockAndRemovalCertified=True");
  }
  if(accepted.Count==0)throw new InvalidOperationException($"CONTACT_BRANCH_SELECTION_UNRESOLVED: {string.Join(";",rejected)}; no source substitution.");
  var selected=Choose(accepted,settings.FitErrorMm);
  log?.Invoke($"stage=contact-selection event=selected domain={selected.Domain} primitives={selected.Native.Moves.Length} contactNominationWitnessUpperMm={selected.ContactWitnessUpperMm:R} candidateCount={accepted.Count} configuredAllowanceMm={settings.FitErrorMm:R} exactZeroFinishRequired=False");
  return selected;
 }
 internal static Candidate Choose(IEnumerable<Candidate> candidates,double allowance)=>candidates
  .OrderBy(c=>Math.Max(0,c.ContactWitnessUpperMm-allowance))
  .ThenBy(c=>c.Native.Moves.Length).ThenBy(c=>c.ContactWitnessUpperMm).First();
 internal static double WitnessUpper(IReadOnlyList<ToolMove> moves,IReadOnlyList<VCarveContactLocus.Site> targets,VCarveCutterProfile profile,int subdivisions)
 {
  double worst=0;
  foreach(var s in targets)
  {
   double best=double.PositiveInfinity;var q=new Point3D(s.Point.x,s.Point.y,-s.StockDepthUpper);
   foreach(var m in moves)for(int i=0;i<=subdivisions;i++)
    best=Math.Min(best,PointDistance(q,VCarveNativeCurveMath.At(m,(double)i/subdivisions),profile));
   worst=Math.Max(worst,best+s.StockDepthUpper-s.StockDepthLower);
  }
  return worst;
 }
 internal static double PointDistance(Point3D q,Point3D p,VCarveCutterProfile profile)
 {
  double rho=Math.Sqrt(Math.Pow(q.X-p.X,2)+Math.Pow(q.Y-p.Y,2)),z=q.Z-p.Z,offset=0,best=double.PositiveInfinity;
  foreach(var s in profile.Sections)
  {
   double h=z-offset,d;
   if(s.Kind==CarvingCutterSectionKind.Hemisphere)
   {double r=s.TopRadius;d=h<=r?Math.Max(0,Math.Sqrt(rho*rho+Math.Pow(h-r,2))-r):Math.Sqrt(Math.Pow(h-r,2)+Math.Pow(Math.Max(0,rho-r),2));}
   else
   {double slope=(s.TopRadius-s.BottomRadius)/s.Height;double Value(double t)=>Math.Sqrt(Math.Pow(h-t,2)+Math.Pow(Math.Max(0,rho-s.BottomRadius-slope*t),2));d=new[]{0,s.Height,Math.Clamp(h,0,s.Height),Math.Clamp((h+slope*(rho-s.BottomRadius))/(1+slope*slope),0,s.Height)}.Min(Value);}
   best=Math.Min(best,d);offset+=s.Height;
  }
  return best;
 }
}
