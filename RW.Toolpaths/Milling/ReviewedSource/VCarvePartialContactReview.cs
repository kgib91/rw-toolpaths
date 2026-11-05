using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling.ReviewedSource.ContactReviewProofs;
namespace RW.Toolpaths.Milling.ReviewedSource;

/// <summary>A bounded, explicitly incomplete contact-review construction. A side
/// is published only after all of its native curves pass continuous geometry,
/// full-cutter clearance, depth, minimum length and connected-visit checks.
/// No result from this API is a completed operation or executable machine path.</summary>
public static class VCarvePartialContactReview
{
 public sealed record Settings(double FitErrorMm,double MinimumLengthMm,double SourceSpacingMm=.075,
  double NativeSpanMm=.65,double RadiusNominationInsetMm=.01,double CurveNominationLiftMm=.002,
  int MaximumCurveSplits=8,int MaximumBoundaryEdges=20000,int MaximumNativeCandidateCurves=1024,double CapReviewExtentMm=8);
 public sealed record Selection(int Region,int Ring,int TipVertex);
 public sealed record Component(string Name,string State,string Detail,Point3D[] Source,ToolMove[] Moves,
  double MaximumFitMm,double MinimumLengthMm,double MaximumJoinDirectionError)
 {public double SourceArcCoveredMm {get;init;} public double SourceArcTotalMm {get;init;}}
 public sealed record Result(string State,Selection Selection,Point3D Root,PointD Tip,Component[] Components,
  bool WholeOperationComplete=false,bool MachineExecutionSupported=false);
 public static Result Construct(IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> boundaries,
  VCarveClearedStock stock,VCarveCutterProfile profile,DepthSchedule depth,Selection selection,Settings settings,
  CancellationToken token=default,Action<string>? diagnostic=null)
  => Prepare(boundaries,stock,profile,depth,settings,token,diagnostic).Construct(selection);
 public sealed class Prepared
 {
  readonly Func<Selection,Action<Component>?,Result> construct;
  internal Prepared(Func<Selection,Action<Component>?,Result> construct)=>this.construct=construct;
  public Result Construct(Selection selection,Action<Component>? publish=null)=>construct(selection,publish);
 }
 public static Prepared Prepare(IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> boundaries,
  VCarveClearedStock stock,VCarveCutterProfile profile,DepthSchedule depth,Settings settings,
  CancellationToken token=default,Action<string>? diagnostic=null)
 {
  if(!double.IsFinite(settings.FitErrorMm+settings.MinimumLengthMm+settings.SourceSpacingMm+settings.NativeSpanMm+
    settings.RadiusNominationInsetMm+settings.CurveNominationLiftMm)||settings.FitErrorMm<=0||settings.MinimumLengthMm<=0||
    settings.SourceSpacingMm<=0||settings.NativeSpanMm<=0||settings.RadiusNominationInsetMm<0||settings.CurveNominationLiftMm<0)
   throw new ArgumentException("CONTACT_REVIEW_CONFIGURATION_INVALID");
  var owner=new VCarveMaterialSurface(boundaries.SelectMany(x=>x),stock,profile,depth.Depth,depth.SurfaceZ,token,diagnostic);
  var bands=owner.ContactReviewBands();var target=new PreparedRegionDistance(boundaries.SelectMany(x=>x));
  var part=new PreparedRegionDistance(new[]{stock.PartBoundary});var layers=bands.Select(b=>new PreparedLayerProof(b.Rings,profile,b.Top,b.Bottom)).ToArray();
  bool Posted(Point3D p)=>double.IsFinite(p.X+p.Y+p.Z)&&Math.Abs(p.X)<=10000&&Math.Abs(p.Y)<=10000&&Math.Abs(p.Z)<=100&&
    p.X==Math.Round(p.X*1000)/1000d&&p.Y==Math.Round(p.Y*1000)/1000d&&p.Z==Math.Round(p.Z*1000)/1000d;
  var columns=stock.Chords.Select((c,i)=>(c,i)).Where(x=>Posted(x.c.A)&&Posted(x.c.B)&&x.c.ClearedColumnRadius>0).ToArray();
  var capsuleIndices=layers.Select(l=>new CapsuleSpatialIndex(columns.Where(x=>x.c.A.Z<=-l.Bottom&&x.c.B.Z<=-l.Bottom)
   .Select(x=>new AnalyticCapsuleCover.Capsule(x.i,x.c.A,x.c.B,x.c.ClearedColumnRadius)))).ToArray();
  var poseCache=new Dictionary<Point3D,bool>();
  bool Certify(ToolMove move)
  {
   token.ThrowIfCancellationRequested();var z=VCarveNativeCurveMath.ZRange(move);if(z.Min < -depth.Depth||!double.IsFinite(z.Min+z.Max))return false;
   double radius=profile.Sections.Max(s=>Math.Max(s.BottomRadius,s.TopRadius));var controls=new[]{move.Start,move.Control1,move.Control2,move.End};
   double xmin=controls.Min(p=>p.X)-radius,xmax=controls.Max(p=>p.X)+radius,ymin=controls.Min(p=>p.Y)-radius,ymax=controls.Max(p=>p.Y)+radius;
   var capsules=capsuleIndices.Select(index=>index.Query(xmin,ymin,xmax,ymax)).ToArray();
   int visited=0;
   bool Visit(ToolMove piece,int level)
   {
    token.ThrowIfCancellationRequested();if(++visited>10000)return false;
    var poly=layers.Select(l=>l.Certify(piece)).ToArray();if(poly.All(p=>p.Certified))return true;
    if(level>=8)
    {
     bool union=true;for(int i=0;i<layers.Length;i++)if(!poly[i].Certified&&!AnalyticCapsuleCover.Prove(piece,profile,capsules[i],target,part,layers[i].Top,0,token,level>=14?100000:5000).Certified){union=false;break;}
     if(union)return true;
    }
    if(level>=14)return false;var split=VCarveNativeCurveMath.Split(piece);return Visit(split.Left,level+1)&&Visit(split.Right,level+1);
   }
   return Visit(move,0);
  }
  bool Pose(Point3D p){if(poseCache.TryGetValue(p,out bool b))return b;b=Certify(ToolMove.CubicBezier(p,p,p,p,0));poseCache[p]=b;return b;}
  // Derive a depth-limit contact handoff from the ACTUAL remaining height
  // bands. This is only a nomination; each resulting pose/curve is certified.
  Paths64? cap=null;
  foreach(var b in bands)
  {
   token.ThrowIfCancellationRequested();double radius=profile.PhysicalRadiusAtDepth(Math.Max(0,depth.Depth-b.Top))+2*VCarveMaterialSurface.PlanarReserve;
   var offset=new ClipperOffset(2,.001*PathUtils.Scale);offset.AddPaths(PathUtils.ToClipper(b.Rings),JoinType.Round,EndType.Polygon);var feasible=new Paths64();offset.Execute(-radius*PathUtils.Scale,feasible);
   cap=cap is null?feasible:Clipper.Intersect(cap,feasible,FillRule.NonZero);
  }
  return new Prepared((selection,publish)=>
  {
  if(selection.Region<0||selection.Region>=boundaries.Count||selection.Ring<0||selection.Ring>=boundaries[selection.Region].Count)
   throw new ArgumentException("CONTACT_REVIEW_SELECTION_INVALID");
  var ring=boundaries[selection.Region][selection.Ring];int count=ring.Count;
  if(selection.TipVertex<0||selection.TipVertex>=count||count<3)throw new ArgumentException("CONTACT_REVIEW_VERTEX_INVALID");
  var tip=ring[selection.TipVertex];
  var selectedRegion=new PreparedRegionDistance(boundaries[selection.Region]);
  IReadOnlyList<PointD>? selectedCap=null;int selectedCapEdge=-1;double selectedCapFraction=0;
  Point3D? root=null;double best=double.PositiveInfinity;
  foreach(var contour in PathUtils.FromClipper(cap??new()))for(int i=0;i<contour.Count;i++)
  {
   var a=contour[i];var b=contour[(i+1)%contour.Count];double dx=b.x-a.x,dy=b.y-a.y,ll=dx*dx+dy*dy,t=ll>0?Math.Clamp(((tip.x-a.x)*dx+(tip.y-a.y)*dy)/ll,0,1):0;
   double x=a.x+t*dx,y=a.y+t*dy,d=(x-tip.x)*(x-tip.x)+(y-tip.y)*(y-tip.y);
   if(d<best&&selectedRegion.SignedDistance(new(x,y))>0){best=d;root=new(x,y,-depth.Depth);selectedCap=contour;selectedCapEdge=i;selectedCapFraction=t;}
  }
  if(root is null||!Pose(root.Value))throw new InvalidOperationException("CONTACT_REVIEW_DEPTH_LIMIT_HANDOFF_UNCERTIFIED");
  PointD Normal(int e)
  {
   var a=ring[e];var b=ring[(e+1)%count];double dx=b.x-a.x,dy=b.y-a.y,L=Math.Sqrt(dx*dx+dy*dy);if(L==0)throw new InvalidOperationException("ZERO_BOUNDARY_EDGE");
   var n=new PointD(-dy/L,dx/L);if(target.SignedDistance(new((a.x+b.x)/2+n.x*.0001,(a.y+b.y)/2+n.y*.0001))<0)n=new(-n.x,-n.y);return n;
  }
  var components=new List<Component>();
  void Publish(Component component){components.Add(component);publish?.Invoke(component);}
  foreach(int direction in new[]{-1,1})
  {
   string name=direction<0?"A":"B";var source=new List<Point3D>();
   try
   {
    int edge=selection.TipVertex;double fraction=0,length=0;var parts=new List<(int Edge,double A,double B,double L)>();bool found=false;
    for(int walked=0;walked<Math.Min(count/2+1,settings.MaximumBoundaryEdges);walked++)
    {
     if(direction<0&&fraction<=1e-12){edge=(edge+count-1)%count;fraction=1;}
     var a=ring[edge];var b=ring[(edge+1)%count];double dx=b.x-a.x,dy=b.y-a.y,ll=dx*dx+dy*dy,end=direction>0?1:0;
     if(ll<=0)throw new InvalidOperationException("ZERO_BOUNDARY_EDGE");double t=((root.Value.X-a.x)*dx+(root.Value.Y-a.y)*dy)/ll;
     bool valid=direction>0?t>fraction+1e-10&&t<=end+1e-10:t>=end-1e-10&&t<fraction-1e-10;var normal=Normal(edge);
     if(valid&&(root.Value.X-a.x-t*dx)*normal.x+(root.Value.Y-a.y-t*dy)*normal.y>0){end=Math.Clamp(t,0,1);found=true;}
     double L=Math.Abs(end-fraction)*Math.Sqrt(ll);if(L>1e-12){parts.Add((edge,fraction,end,L));length+=L;}
     if(found)break;if(direction>0)edge=(edge+1)%count;fraction=0;
    }
    if(!found)throw new InvalidOperationException("CONTACT_REVIEW_BOUNDARY_HANDOFF_NOT_FOUND");
    int stations=Math.Max(33,(int)Math.Ceiling(length/settings.SourceSpacingMm)+1);if(stations>4096)throw new InvalidOperationException("CONTACT_REVIEW_STATION_BUDGET");
    var heights=new double[stations];double traversed=0;int at=0;
    for(int j=0;j<stations;j++)
    {
     token.ThrowIfCancellationRequested();double arc=length*j/(stations-1d);while(at<parts.Count-1&&arc>=traversed+parts[at].L){traversed+=parts[at++].L;}
     var p=parts[at];var a=ring[p.Edge];var b=ring[(p.Edge+1)%count];double f=p.A+(p.B-p.A)*(arc-traversed)/p.L;var q=new PointD(a.x+f*(b.x-a.x),a.y+f*(b.y-a.y));var n=Normal(p.Edge);var h=owner.ResidualSurfaceDepthBounds(q.x,q.y);heights[j]=h.Upper;Point3D selected;
     if(j==0)selected=new(q.x,q.y,owner.EntryHeight(new(q.x,q.y,depth.SurfaceZ-h.Upper))-depth.SurfaceZ);
     else if(j==stations-1)selected=root.Value;
     else
     {
      double maxr=profile.PhysicalRadiusAtDepth(Math.Max(0,depth.Depth-h.Upper)),bestRadius=-1;
      Point3D Nominate(double rr)=>new(q.x+n.x*(rr+2*VCarveMaterialSurface.PlanarReserve),q.y+n.y*(rr+2*VCarveMaterialSurface.PlanarReserve),-Math.Min(depth.Depth,h.Upper+profile.DepthAtRadius(rr)));
      for(int k=0;k<=16;k++){double rr=maxr*k/16;if(Pose(Nominate(rr)))bestRadius=rr;}
      if(bestRadius<0)throw new InvalidOperationException($"CONTACT_REVIEW_POSE_UNCERTIFIED station={j}");
      double lo=bestRadius,hi=Math.Min(maxr,bestRadius+maxr/16);for(int k=0;k<7;k++){double rr=(lo+hi)/2;if(Pose(Nominate(rr)))lo=rr;else hi=rr;}
      selected=Nominate(Math.Max(0,lo-settings.RadiusNominationInsetMm));
     }
     if(!Pose(selected))throw new InvalidOperationException($"CONTACT_REVIEW_REQUIRED_POSE_UNCERTIFIED station={j}");source.Add(selected);
    }
    var fitted=Fit(source.ToArray(),heights,settings,Certify,token);if(!fitted.Complete)throw new InvalidOperationException("CONTACT_REVIEW_UPPER_ARM_INCOMPLETE");var forward=fitted.Moves;
    // A standalone closed review visit has an explicit stop at its selected
    // surface corner and depth handoff. No machine approach is manufactured.
    var closed=forward.Reverse().Select(VCarveNativeCurveMath.Reverse).Concat(forward).ToArray();
    for(int i=1;i<closed.Length;i++)if(closed[i-1].End!=closed[i].Start)throw new InvalidOperationException("CONTACT_REVIEW_CLOSED_ROUTE_GAP");
    Publish(new(name,"CertifiedLocalNativeContactVisit","Local entry/return is closed; between-component and machine approach remain unverified.",source.ToArray(),closed,fitted.Error,fitted.Minimum,fitted.Tangent));
    diagnostic?.Invoke($"[Toolpath.ContactReview] side={name} state=certified curves={forward.Length} fitMm={fitted.Error:R} minimumMm={fitted.Minimum:R}");
   }
   catch(OperationCanceledException){throw;}
   catch(Exception error){Publish(new(name,"Failed_NotShownAsCorrected",error.Message,source.ToArray(),[],double.NaN,double.NaN,double.NaN));diagnostic?.Invoke($"[Toolpath.ContactReview] side={name} state=failed reason={error.Message}");}
  }
  // The upper contact arms do not own the lower depth-limit wall. Show a
  // bounded wall-contour review in both directions from the SAME actual-stock
  // handoff. The view extent is an explicit review bound, not a cut-completion
  // or finishing tolerance. No broad interior clearing is introduced.
  foreach(int direction in new[]{-1,1})
  {
   string name=direction<0?"Depth-wall A":"Depth-wall B";var points=new List<Point3D>{root.Value};
   try
   {
    var contour=selectedCap!;int n=contour.Count,e=selectedCapEdge;double f=selectedCapFraction,length=0;
    for(int walked=0;walked<n+1&&length<settings.CapReviewExtentMm-1e-10;walked++)
    {
     if(direction<0&&f<=1e-12){e=(e+n-1)%n;f=1;}
     var a=contour[e];var b=contour[(e+1)%n];double dx=b.x-a.x,dy=b.y-a.y,L=Math.Sqrt(dx*dx+dy*dy);if(L<=0)throw new InvalidOperationException("ZERO_DEPTH_CONTOUR_EDGE");
     double end=direction>0?1:0,take=Math.Min(Math.Abs(end-f)*L,settings.CapReviewExtentMm-length);end=f+direction*take/L;
     var next=new Point3D(a.x+end*dx,a.y+end*dy,-depth.Depth);if(VCarveNativeCurveMath.Distance(points[^1],next)>1e-10)points.Add(next);length+=take;
     if(length>=settings.CapReviewExtentMm-1e-10)break;if(direction>0)e=(e+1)%n;f=0;
    }
    var fitted=Fit(points.ToArray(),new double[points.Count],settings,Certify,token);var closed=fitted.Moves.Concat(fitted.Moves.Reverse().Select(VCarveNativeCurveMath.Reverse)).ToArray();
    Publish(new(name,fitted.Complete?"CertifiedLocalNativeContactVisit":"PartialCertifiedNativePrefix_UnfinishedContact",$"Certified {fitted.Covered:F4}/{fitted.Total:F4} mm of the requested depth-wall reference arc. Unproved tail is NOT cut or called complete. Closed exact return to the shared handoff.",points.ToArray(),closed,fitted.Error,fitted.Minimum,fitted.Tangent){SourceArcCoveredMm=fitted.Covered,SourceArcTotalMm=fitted.Total});
    diagnostic?.Invoke($"[Toolpath.ContactReview] component={name} state=certified curves={fitted.Moves.Length} fitMm={fitted.Error:R} minimumMm={fitted.Minimum:R}");
   }
   catch(OperationCanceledException){throw;}
   catch(Exception error){Publish(new(name,"Failed_NotShownAsCorrected",error.Message,points.ToArray(),[],double.NaN,double.NaN,double.NaN));diagnostic?.Invoke($"[Toolpath.ContactReview] component={name} state=failed reason={error.Message}");}
  }
  return new Result("PartialReview_NotACompletedOperation",selection,root.Value,tip,components.ToArray());
  });
 }
 static (ToolMove[] Moves,double Error,double Minimum,double Tangent,bool Complete,double Covered,double Total) Fit(Point3D[] source,double[] height,Settings settings,Func<ToolMove,bool> guard,CancellationToken token)
 {
  var u=new double[source.Length];for(int i=1;i<u.Length;i++){u[i]=u[i-1]+VCarveNativeCurveMath.Distance(source[i-1],source[i]);if(u[i]<=u[i-1])throw new InvalidOperationException("CONTACT_REVIEW_COINCIDENT_SOURCE_EVENT");}
  double total=u[^1];var x=source.Select(p=>p.X).ToArray();var y=source.Select(p=>p.Y).ToArray();var z=source.Select(p=>p.Z).ToArray();var dx=Pchip(u,x);var dy=Pchip(u,y);var dz=Pchip(u,z);
  dz[0]=-(height[1]-height[0])/(u[1]-u[0]);
  (Point3D P,Point3D D) At(double t)
  {
   int i=Array.BinarySearch(u,t);if(i<0)i=~i-1;i=Math.Clamp(i,0,u.Length-2);double h=u[i+1]-u[i],v=(t-u[i])/h;
   (double,double) Eval(double[] a,double[] d){double v2=v*v,v3=v2*v;return((2*v3-3*v2+1)*a[i]+(v3-2*v2+v)*h*d[i]+(-2*v3+3*v2)*a[i+1]+(v3-v2)*h*d[i+1],(6*v2-6*v)/h*a[i]+(3*v2-4*v+1)*d[i]+(-6*v2+6*v)/h*a[i+1]+(3*v2-2*v)*d[i+1]);}
   var a=Eval(x,dx);var b=Eval(y,dy);var c=Eval(z,dz);double lift=settings.CurveNominationLiftMm*Math.Pow(Math.Sin(Math.PI*t/total),2),derivative=settings.CurveNominationLiftMm*Math.PI/total*Math.Sin(2*Math.PI*t/total);
   return(new(a.Item1,b.Item1,c.Item1+lift),new(a.Item2,b.Item2,c.Item2+derivative));
  }
  // Shared derivatives belong to one continuous PCHIP guide. Choose native
  // joins jointly, rather than recursively splitting one refused short span
  // until the minimum becomes impossible. No source substitution is emitted.
  var positions=new SortedSet<double>{0,total};for(double at=settings.SourceSpacingMm;at<total;at+=settings.SourceSpacingMm)positions.Add(at);int initial=Math.Max(1,(int)Math.Ceiling(total/settings.NativeSpanMm));
  for(int i=1;i<initial;i++)positions.Add(total*i/initial);
  var knots=positions.ToArray();var memo=new Dictionary<(int,int),(ToolMove Move,double Error,double Length)?>();int trials=0;
  (ToolMove Move,double Error,double Length)? Edge(int first,int last)
  {
   if(memo.TryGetValue((first,last),out var old))return old;token.ThrowIfCancellationRequested();
   if(++trials>settings.MaximumNativeCandidateCurves)throw new InvalidOperationException("CONTACT_REVIEW_NATIVE_CANDIDATE_BUDGET");
   double a=knots[first],b=knots[last];var p=At(a);var q=At(b);var start=a==0?source[0]:p.P;var end=b==total?source[^1]:q.P;
   var move=ToolMove.CubicBezier(start,VCarveNativeCurveMath.Add(p.P,VCarveNativeCurveMath.Mul(p.D,(b-a)/3)),VCarveNativeCurveMath.Sub(q.P,VCarveNativeCurveMath.Mul(q.D,(b-a)/3)),end,0);
   var parameters=u.Where(t=>t>a&&t<b).Prepend(a).Append(b).ToArray();Point3D Reference(double t){int i=Array.BinarySearch(u,t);if(i>=0)return source[i];i=Math.Clamp(~i-1,0,u.Length-2);return VCarveNativeCurveMath.Lerp(source[i],source[i+1],(t-u[i])/(u[i+1]-u[i]));}
   var reference=parameters.Select(Reference).ToArray();var geom=VCarveNativeGeometricProof.ProveFreeJoin(move,reference,0,reference.Length-1,settings.FitErrorMm);var length=VCarveNativeCurvePlanning.MeasureLength(move);
   if(geom.Certified&&length.Lower>=settings.MinimumLengthMm&&guard(move))return memo[(first,last)]=(move with{MaxError=geom.ErrorBound},geom.ErrorBound,length.Lower);
   return memo[(first,last)]=null;
  }
  var dead=new HashSet<int>();var chosen=new Dictionary<int,(int Stop,ToolMove Move,double Error,double Length)>();
  int[] Stops(int a)=>Enumerable.Range(a+1,knots.Length-a-1).Where(b=>knots[b]-knots[a]>=settings.MinimumLengthMm&&knots[b]-knots[a]<=2*settings.NativeSpanMm)
   .OrderBy(b=>Math.Abs((knots[b]-knots[a])-settings.NativeSpanMm)).ToArray();
  var bestPath=new List<(int Stop,ToolMove Move,double Error,double Length)>();int farthest=0;
  var search=new Stack<(int At,int[] Stops,int Cursor)>();search.Push((0,Stops(0),0));bool solved=false;
  while(search.TryPop(out var frame))
  {
   token.ThrowIfCancellationRequested();if(frame.Cursor>=frame.Stops.Length){dead.Add(frame.At);continue;}
   int stop=frame.Stops[frame.Cursor];search.Push((frame.At,frame.Stops,frame.Cursor+1));if(dead.Contains(stop))continue;
   var edge=Edge(frame.At,stop);if(edge is not {} fit)continue;chosen[frame.At]=(stop,fit.Move,fit.Error,fit.Length);
   if(stop>farthest){var path=new List<(int Stop,ToolMove Move,double Error,double Length)>();int node=0;while(node<stop){var item=chosen[node];path.Add(item);node=item.Stop;}if(node==stop){farthest=stop;bestPath=path;}}
   if(stop==knots.Length-1){solved=true;break;}search.Push((stop,Stops(stop),0));
  }
  if(!solved&&bestPath.Count==0)throw new InvalidOperationException($"CONTACT_REVIEW_NATIVE_ROUTE_UNRESOLVED candidates={trials}; no minimum/clearance relaxation");
  var moves=new List<ToolMove>();double error=0,minimum=double.PositiveInfinity;int cursor=0;
  if(solved){while(cursor<knots.Length-1){var item=chosen[cursor];moves.Add(item.Move);error=Math.Max(error,item.Error);minimum=Math.Min(minimum,item.Length);cursor=item.Stop;}}
  else{foreach(var item in bestPath){moves.Add(item.Move);error=Math.Max(error,item.Error);minimum=Math.Min(minimum,item.Length);cursor=item.Stop;}}
  double tangent=0;for(int i=1;i<moves.Count;i++){if(moves[i-1].End!=moves[i].Start)throw new InvalidOperationException("CONTACT_REVIEW_NATIVE_POSITION_GAP");tangent=Math.Max(tangent,VCarveNativeCurveMath.Distance(VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(moves[i-1],1)),VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(moves[i],0))));}
  if(tangent>1e-8)throw new InvalidOperationException("CONTACT_REVIEW_NATIVE_SHARED_TANGENT_REFUSED");return(moves.ToArray(),error,minimum,tangent,solved,knots[cursor],total);
 }
 public static (double Lower,double Upper) NativeLength(ToolMove move){var b=VCarveNativeCurvePlanning.MeasureLength(move);return(b.Lower,b.Upper);}
 static double[] Pchip(double[] u,double[] v)
 {
  int n=u.Length;if(n<2)throw new InvalidOperationException("CONTACT_GUIDE_NEEDS_TWO_DISTINCT_POINTS");if(n==2){double m=(v[1]-v[0])/(u[1]-u[0]);return [m,m];}var h=new double[n-1];var slope=new double[n-1];for(int i=0;i<n-1;i++){h[i]=u[i+1]-u[i];slope[i]=(v[i+1]-v[i])/h[i];}var d=new double[n];
  double End(double h0,double h1,double m0,double m1){double a=((2*h0+h1)*m0-h0*m1)/(h0+h1);if(Math.Sign(a)!=Math.Sign(m0))return 0;if(Math.Sign(m0)!=Math.Sign(m1)&&Math.Abs(a)>3*Math.Abs(m0))return 3*m0;return a;}
  d[0]=End(h[0],h[1],slope[0],slope[1]);d[^1]=End(h[^1],h[^2],slope[^1],slope[^2]);for(int i=1;i<n-1;i++){if(slope[i-1]*slope[i]<=0)d[i]=0;else{double a=2*h[i]+h[i-1],b=h[i]+2*h[i-1];d[i]=(a+b)/(a/slope[i-1]+b/slope[i]);}}return d;
 }
}
