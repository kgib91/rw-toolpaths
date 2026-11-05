namespace RW.Toolpaths.Milling;

// A continuous, ordered geometric correspondence, not a feed/time law.
// Source chords and the native primitive may each pause/reparameterize. The
// explicit Bernstein correspondence covers both curves from start to finish.
internal sealed record VCarveNativeGeometryCertificate(bool Certified,double ErrorBound,string Reason,double[] Parameters);
internal readonly record struct VCarveNativeInteriorPose(int SourceOrdinal,double Parameter);
internal static class VCarveNativeGeometricProof
{
 internal const double ArithmeticReserve=1e-9;
 private const int MaximumSubdivisions=4096,MaximumDepth=14;
 internal static VCarveNativeGeometryCertificate Prove(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,double allowance,IReadOnlyList<VCarveNativeInteriorPose>? interiorPoses=null)
  =>ProveCore(move,source,first,last,allowance,interiorPoses,true);
 // Only a free fit join may move. The continuous difference enclosure includes
 // both displaced endpoints; outer topology/visit endpoints remain caller-owned.
 internal static VCarveNativeGeometryCertificate ProveFreeJoin(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,double allowance)
  =>ProveCore(move,source,first,last,allowance,null,false);
 private static VCarveNativeGeometryCertificate ProveCore(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,double allowance,IReadOnlyList<VCarveNativeInteriorPose>? interiorPoses,bool exactEndpoints)
 {
  VCarveNativeGeometryCertificate Failed(string reason)=>new(false,double.PositiveInfinity,reason,[]);
  if(last<=first||exactEndpoints&&(move.Start!=source[first]||move.End!=source[last]))return Failed("geometry-endpoint-ownership");
  if(move.Kind is not (ToolMoveKind.Line or ToolMoveKind.CubicBezier))return Failed("geometry-kind-unsupported");
  interiorPoses??=[];int ordinal=first;double parameter=0;
  foreach(var pose in interiorPoses)
  {
   if(pose.SourceOrdinal<=ordinal||pose.SourceOrdinal>=last||!double.IsFinite(pose.Parameter)||pose.Parameter<=parameter||pose.Parameter>=1)return Failed("required-interior-pose-order");
   if(VCarveNativeCurveMath.Distance(VCarveNativeCurveMath.At(move,pose.Parameter),source[pose.SourceOrdinal])>ArithmeticReserve)return Failed("required-interior-pose-not-on-curve");
   ordinal=pose.SourceOrdinal;parameter=pose.Parameter;
  }
  if(move.Kind==ToolMoveKind.CubicBezier&&!Regular(move))return Failed("curve-regularity-not-certified");
  var chord=VCarveNativeCurveMath.Sub(move.End,move.Start);
  VCarveNativeGeometryCertificate? failure=null;
  // These alternate witnesses depend on the curve/source/anchors, not the
  // projection direction. Reuse their exact results within this one proof.
  var alternativeComputed=new bool[2];
  var alternativeResults=new VCarveNativeGeometryCertificate?[2];
  VCarveNativeGeometryCertificate? Alternative(int strategy)
  {
   if(alternativeComputed[strategy])return alternativeResults[strategy];
   alternativeComputed[strategy]=true;
   var witness=strategy==0?ClosestOrderedParameters(move,source,first,last,allowance):MinimaxOrderedParameters(move,source,first,last,allowance);
   if(witness is null)return null;
   var anchors=new List<(int Ordinal,double Parameter)>{(first,0)};
   anchors.AddRange(interiorPoses.Select(p=>(p.SourceOrdinal,p.Parameter)));anchors.Add((last,1));
   for(int k=1;k<anchors.Count;k++)
   {
    var left=anchors[k-1];var right=anchors[k];witness[left.Ordinal-first]=left.Parameter;witness[right.Ordinal-first]=right.Parameter;
    for(int i=left.Ordinal+1;i<right.Ordinal;i++)witness[i-first]=Math.Clamp(witness[i-first],left.Parameter,right.Parameter);
   }
   return alternativeResults[strategy]=CertifyCorrespondence(move,source,first,last,allowance,witness);
  }
  // Alternative monotone projection WITNESSES, never different acceptance
  // tolerances or source substitutions. A curve may turn in Z while progressing
  // in XY; the full continuous correspondence still proves its geometry/order.
  foreach(var direction in new[]{chord,new Point3D(1,0,0),new Point3D(0,1,0),new Point3D(0,0,1)})
  {
   if(VCarveNativeCurveMath.Dot(chord,direction)==0)continue;
   var proof=TryProjection(move,source,first,last,allowance,direction,interiorPoses,exactEndpoints,Alternative);if(proof.Certified)return proof;
   if(failure is null||proof.ErrorBound<failure.ErrorBound)failure=proof;
  }
  return failure??Failed("curve-progress-not-certified");
 }
 private static VCarveNativeGeometryCertificate TryProjection(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,double allowance,Point3D direction,IReadOnlyList<VCarveNativeInteriorPose> interiorPoses,bool exactEndpoints,Func<int,VCarveNativeGeometryCertificate?> alternative)
 {
  VCarveNativeGeometryCertificate Failed(string reason,double bound=double.PositiveInfinity)=>new(false,bound,reason,[]);
  if(last<=first||exactEndpoints&&(move.Start!=source[first]||move.End!=source[last]))return Failed("geometry-endpoint-ownership");
  if(move.Kind is not (ToolMoveKind.Line or ToolMoveKind.CubicBezier))return Failed("geometry-kind-unsupported");
  var chord=VCarveNativeCurveMath.Sub(move.End,move.Start);double square=VCarveNativeCurveMath.Dot(chord,chord);
  if(square<=1e-24)return Failed("geometry-closed-span-needs-partition");
  double divisor=VCarveNativeCurveMath.Dot(chord,direction);
  double Project(Point3D p)=>VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(p,move.Start),direction)/divisor;
  if(move.Kind==ToolMoveKind.Line&&interiorPoses.Count==0)
  {
   double[] a=[move.Start.X,move.Start.Y,move.Start.Z],b=[move.End.X,move.End.Y,move.End.Z];var axes=Enumerable.Range(0,3).Where(k=>a[k]!=b[k]).ToArray();
   if(axes.Length==1)
   {
    int axis=axes[0];double prior=a[axis];bool exact=true;var exactParameters=new double[last-first+1];
    for(int i=first;i<=last;i++){double[] p=[source[i].X,source[i].Y,source[i].Z];if(Enumerable.Range(0,3).Any(k=>k!=axis&&p[k]!=a[k])||(b[axis]>a[axis]?p[axis]<prior||p[axis]>b[axis]:p[axis]>prior||p[axis]<b[axis])){exact=false;break;}exactParameters[i-first]=(p[axis]-a[axis])/(b[axis]-a[axis]);prior=p[axis];}
    if(exact)return new(true,0,"exact-ordered-axis-line",exactParameters);
   }
  }
  double q1=move.Kind==ToolMoveKind.Line?1d/3:Project(move.Control1),q2=move.Kind==ToolMoveKind.Line?2d/3:Project(move.Control2);
  double qa=3*(1-3*q2+3*q1),qb=6*(q2-2*q1),qc=3*q1;
  double derivativeMinimum=Math.Min(qc,3*(1-q2));
  if(qa>0){double at=-qb/(2*qa);if(at>0&&at<1)derivativeMinimum=Math.Min(derivativeMinimum,(qa*at+qb)*at+qc);}
  // A strictly progressing open projection rules out added loops/backtracking.
  // This is a sufficient regularity certificate; refusal is not necessity proof.
  if(derivativeMinimum<0)return Failed("curve-progress-not-certified");
  double Progress(double t)=>move.Kind==ToolMoveKind.Line?t:3*q1*(1-t)*(1-t)*t+3*q2*(1-t)*t*t+t*t*t;
  var parameters=new double[last-first+1];parameters[^1]=1;double priorGoal=0,priorParameter=0;
  for(int i=1;i<parameters.Length-1;i++)
  {
   double goal=Math.Max(priorGoal,Math.Clamp(Project(source[first+i]),0,1));double t;
   if(goal==priorGoal)t=priorParameter;
   else if(goal==1)t=1;
   else if(move.Kind==ToolMoveKind.Line)t=goal;
   else
   {
    double low=priorParameter,high=1;t=Math.Clamp(goal,low,high);
    // This only nominates a witness. The actual computed parameters below
    // receive a complete continuous certificate, regardless of root accuracy.
    for(int step=0;step<20;step++)
    {
     double value=Progress(t);if(value==goal)break;if(value<goal)low=t;else high=t;
     double derivative=(qa*t+qb)*t+qc;double next=derivative>0?t-(value-goal)/derivative:double.NaN;if(next==t)break;
     if(!double.IsFinite(next)||next<=low||next>=high)next=(low+high)/2;
     if(next==t)break;t=next;
    }
   }
   parameters[i]=Math.Clamp(t,priorParameter,1);priorParameter=parameters[i];priorGoal=goal;
  }
  var anchors=new List<(int Ordinal,double Parameter)>{(first,0)};anchors.AddRange(interiorPoses.Select(p=>(p.SourceOrdinal,p.Parameter)));anchors.Add((last,1));
  for(int k=1;k<anchors.Count;k++)
  {
   var left=anchors[k-1];var right=anchors[k];parameters[left.Ordinal-first]=left.Parameter;parameters[right.Ordinal-first]=right.Parameter;
   for(int i=left.Ordinal+1;i<right.Ordinal;i++)parameters[i-first]=Math.Clamp(parameters[i-first],left.Parameter,right.Parameter);
  }
  var projected=CertifyCorrespondence(move,source,first,last,allowance,parameters);
  if(projected.Certified)return projected;
  // Projection is one sufficient witness, not an extra source timing law.
  // Nominate a second monotone correspondence, then use the IDENTICAL complete
  // Bernstein difference proof. Vertex proximity alone grants no certificate.
  foreach(int strategy in new[]{0,1})
  {
   var optimized=alternative(strategy);
   if(optimized is null)continue;
   if(optimized.Certified)return optimized;
   if(optimized.ErrorBound<projected.ErrorBound)projected=optimized;
  }
  return projected;
 }
 // Greedy closest-point correspondence can consume progress needed by a later
 // small source reversal. Minimax dynamic programming nominates an ordered
 // witness jointly. Search samples are proof parameters, never output moves;
 // only the unchanged complete Bernstein enclosure authorizes a certificate.
 private static double[]? MinimaxOrderedParameters(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,double allowance)
 {
  const int divisions=256;int count=last-first+1;
  var positions=Enumerable.Range(0,divisions+1).Select(i=>VCarveNativeCurveMath.At(move,i/(double)divisions)).ToArray();
  var previous=Enumerable.Repeat(double.PositiveInfinity,divisions+1).ToArray();
  previous[0]=VCarveNativeCurveMath.Distance(move.Start,source[first]);
  var parents=new int[count,divisions+1];
  for(int ordinal=1;ordinal<count;ordinal++)
  {
   var current=new double[divisions+1];double best=double.PositiveInfinity;int predecessor=0;
   for(int j=0;j<=divisions;j++)
   {
    if(previous[j]<best){best=previous[j];predecessor=j;}
    current[j]=Math.Max(best,VCarveNativeCurveMath.Distance(positions[j],source[first+ordinal]));
    parents[ordinal,j]=predecessor;
   }
   previous=current;
  }
  if(previous[divisions]>allowance)return null;
  var result=new double[count];int cursor=divisions;
  for(int ordinal=count-1;ordinal>0;ordinal--){result[ordinal]=cursor/(double)divisions;cursor=parents[ordinal,cursor];}
  return result;
 }
 private static double[]? ClosestOrderedParameters(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,double allowance)
 {
  var parameters=new double[last-first+1];parameters[^1]=1;double prior=0;
  for(int i=1;i<parameters.Length-1;i++)
  {
   var point=source[first+i];double Value(double t){var d=VCarveNativeCurveMath.Sub(VCarveNativeCurveMath.At(move,t),point);return VCarveNativeCurveMath.Dot(d,d);}
   int best=0;double value=Value(0);
   for(int j=1;j<=64;j++){double next=Value(j/64d);if(next<value){value=next;best=j;}}
   double low=Math.Max(0,(best-1)/64d),high=Math.Min(1,(best+1)/64d),ratio=(Math.Sqrt(5)-1)/2;
   double a=high-ratio*(high-low),b=low+ratio*(high-low),fa=Value(a),fb=Value(b);
   for(int step=0;step<32;step++)
   {
    if(fa<=fb){high=b;b=a;fb=fa;a=high-ratio*(high-low);fa=Value(a);}
    else{low=a;a=b;fa=fb;b=low+ratio*(high-low);fb=Value(b);}
   }
   double time=best/64d;if(fa<value){value=fa;time=a;}if(fb<value){value=fb;time=b;}
   time=Math.Max(prior,time);if(Value(time)>allowance*allowance)return null;
   parameters[i]=time;prior=time;
  }
  return parameters;
 }
 private static VCarveNativeGeometryCertificate CertifyCorrespondence(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,double allowance,double[] parameters)
 {
  if(move.Kind==ToolMoveKind.Line)
  {
   double bound=Enumerable.Range(0,parameters.Length).Max(i=>VCarveNativeCurveMath.Distance(source[first+i],VCarveNativeCurveMath.At(move,parameters[i])))+ArithmeticReserve;
   return new(bound<=allowance,bound,bound<=allowance?"continuous-ordered-line":"continuous-order-fit-unresolved",parameters);
  }
  Point3D[] controls=[move.Start,move.Control1,move.Control2,move.End];double maximum=0;int remaining=MaximumSubdivisions;
  for(int i=0;i<parameters.Length-1;i++)
  {
   var a=source[first+i];var b=source[first+i+1];var delta=VCarveNativeCurveMath.Sub(b,a);double length2=VCarveNativeCurveMath.Dot(delta,delta);
   Point3D[] c=parameters[i]==parameters[i+1]?Enumerable.Repeat(VCarveNativeCurveMath.At(move,parameters[i]),4).ToArray():Restrict(controls,parameters[i],parameters[i+1]);
   // A polynomial source parameter with ordered controls is itself monotone.
   // It is not constrained to be linear in the native parameter or arclength.
   double l1=length2==0?1d/3:Math.Clamp(VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(c[1],a),delta)/length2,0,1);
   double l2=length2==0?2d/3:Math.Max(l1,Math.Clamp(VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(c[2],a),delta)/length2,0,1));
   double[] lambda=[0,l1,l2,1];var difference=Enumerable.Range(0,4).Select(k=>VCarveNativeCurveMath.Sub(c[k],VCarveNativeCurveMath.Add(a,VCarveNativeCurveMath.Mul(delta,lambda[k])))).ToArray();
   if(!Bound(difference,allowance,0,ref remaining,out double bound))return new(false,Math.Max(maximum,bound),remaining<0?"continuous-order-proof-budget":"continuous-order-fit-unresolved",parameters);
   maximum=Math.Max(maximum,bound);
  }
  return new(true,maximum,"continuous-ordered-cubic",parameters);
 }
 private static bool Regular(ToolMove m)
 {
  Point3D[] d=[VCarveNativeCurveMath.Mul(VCarveNativeCurveMath.Sub(m.Control1,m.Start),3),VCarveNativeCurveMath.Mul(VCarveNativeCurveMath.Sub(m.Control2,m.Control1),3),VCarveNativeCurveMath.Mul(VCarveNativeCurveMath.Sub(m.End,m.Control2),3)];
  int remaining=256;
  bool Cover(Point3D[] c,int depth)
  {
   foreach(var axis in new[]{new Point3D(1,0,0),new Point3D(0,1,0),new Point3D(0,0,1)}.Concat(c))
   {
    double minimum=c.Min(p=>VCarveNativeCurveMath.Dot(p,axis)),maximum=c.Max(p=>VCarveNativeCurveMath.Dot(p,axis));if(minimum>0||maximum<0)return true;
   }
   if(depth>=12||--remaining<0)return false;
   var a=VCarveNativeCurveMath.Lerp(c[0],c[1],.5);var b=VCarveNativeCurveMath.Lerp(c[1],c[2],.5);var middle=VCarveNativeCurveMath.Lerp(a,b,.5);
   return Cover([c[0],a,middle],depth+1)&&Cover([middle,b,c[2]],depth+1);
  }
  return Cover(d,0);
 }
 private static bool Bound(Point3D[] difference,double allowance,int depth,ref int remaining,out double bound)
 {
  bound=difference.Max(VCarveNativeCurveMath.Norm)+ArithmeticReserve;if(bound<=allowance)return true;
  if(depth>=MaximumDepth||--remaining<0)return false;
  var halves=Split(difference,.5);
  if(VCarveNativeCurveMath.Norm(halves.Left[^1])+ArithmeticReserve>allowance)return false;
  bool left=Bound(halves.Left,allowance,depth+1,ref remaining,out double a);if(!left){bound=a;return false;}
  bool right=Bound(halves.Right,allowance,depth+1,ref remaining,out double b);bound=Math.Max(a,b);return right;
 }
 private static Point3D[] Restrict(Point3D[] controls,double a,double b)
 {
  var c=controls;if(b<1)c=Split(c,b).Left;if(a>0)c=Split(c,a/b).Right;return c;
 }
 private static (Point3D[] Left,Point3D[] Right) Split(Point3D[] c,double t)
 {
  var a=VCarveNativeCurveMath.Lerp(c[0],c[1],t);var b=VCarveNativeCurveMath.Lerp(c[1],c[2],t);var d=VCarveNativeCurveMath.Lerp(c[2],c[3],t);
  var e=VCarveNativeCurveMath.Lerp(a,b,t);var f=VCarveNativeCurveMath.Lerp(b,d,t);var g=VCarveNativeCurveMath.Lerp(e,f,t);
  return ([c[0],a,e,g],[g,f,d,c[3]]);
 }
}
