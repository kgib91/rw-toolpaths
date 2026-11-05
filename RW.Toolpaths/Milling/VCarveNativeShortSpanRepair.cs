namespace RW.Toolpaths.Milling;



// One local segmentation stage inside a canonical source block. Its endpoints

// are the caller's protected topology/state boundaries. New fit boundaries may

// move; every new curve is checked directly against the immutable raw points.

internal static class VCarveNativeShortSpanRepair

{

 internal static List<VCarveNativeFit> Repair(Point3D[] points,List<VCarveNativeFit> input,IReadOnlyList<int> sourceKnots,
  VCarveNativeContainment guard,VCarveCutterProfile profile,VCarveNativeSearchBudget budget,VCarveNativeFitSettings settings,
  int maximumBlockPoints,CancellationToken token,IDictionary<string,int> failures,out bool exhausted,Point3D boundaryStart,Point3D boundaryEnd,VCarveNativeSourceContract contract=VCarveNativeSourceContract.PreparedSpatialDirections)
 {
  // Greedy repartition is direction-dependent. Search both exact orientations
  // when the first leaves a short; neither direction changes geometry or guards.
  // Both use the same operation budget and start from the same incumbent fits.
  int orientation=0;
  for(int i=0;i<points.Length&&orientation==0;i++)
  {
   var a=points[i];var b=points[points.Length-1-i];
   orientation=a.X.CompareTo(b.X);if(orientation==0)orientation=a.Y.CompareTo(b.Y);if(orientation==0)orientation=a.Z.CompareTo(b.Z);
  }
  int lastIndex=points.Length-1;
  VCarveNativeFit Reverse(VCarveNativeFit f)=>f with{Move=VCarveNativeCurveMath.Reverse(f.Move),First=lastIndex-f.Last,Last=lastIndex-f.First,TangentStart=f.TangentEnd,TangentEnd=f.TangentStart};
  List<VCarveNativeFit> Search(bool reverse,out bool spent)
  {
   if(!reverse)return RepairOneDirection(points,input,sourceKnots,guard,profile,budget,settings,maximumBlockPoints,token,failures,out spent,boundaryStart,boundaryEnd,contract);
   var found=RepairOneDirection(points.Reverse().ToArray(),input.AsEnumerable().Reverse().Select(Reverse).ToList(),sourceKnots.Reverse().Select(i=>lastIndex-i).ToArray(),
    guard,profile,budget,settings,maximumBlockPoints,token,failures,out spent,VCarveNativeCurveMath.Mul(boundaryEnd,-1),VCarveNativeCurveMath.Mul(boundaryStart,-1),contract);
   return found.AsEnumerable().Reverse().Select(Reverse).ToList();
  }
  int Shorts(List<VCarveNativeFit> fits)=>fits.Count(f=>f.LengthLower<settings.MinimumPrimitiveLengthMm);
  bool firstReverse=orientation<0;
  var first=Search(firstReverse,out exhausted);
  if(exhausted||Shorts(first)==0)return first;
  var second=Search(!firstReverse,out exhausted);
  return Shorts(second)<Shorts(first)?second:first;
 }

 private static List<VCarveNativeFit> RepairOneDirection(Point3D[] points,List<VCarveNativeFit> input,IReadOnlyList<int> sourceKnots,

  VCarveNativeContainment guard,VCarveCutterProfile profile,VCarveNativeSearchBudget budget,VCarveNativeFitSettings settings,

  int maximumBlockPoints,CancellationToken token,IDictionary<string,int> failures,out bool exhausted,Point3D boundaryStart,Point3D boundaryEnd,VCarveNativeSourceContract contract=VCarveNativeSourceContract.PreparedSpatialDirections)

 {
  var fits=input.ToList();var cumulative=new double[points.Length];for(int k=1;k<points.Length;k++)cumulative[k]=cumulative[k-1]+VCarveNativeCurveMath.Distance(points[k-1],points[k]);

  VCarveNativeFit? Find(int a,int b,Point3D u,Point3D v)

  {

   budget.CheckCandidate();var fit=VCarveNativeEdgeSearch.Find(points,a,b,u,v,cumulative[b]-cumulative[a],guard,profile,out var audit,settings);

   // A true contact branch owns planar endpoint directions. Its sampled

   // stock-contact Z slope is not an authored tangent. Interior free joins

   // keep one shared spatial direction; every candidate still earns the same

   // continuous raw geometry/depth/physical cutter certificate.

   var freedom=(a==0?VCarveNativeAxialFreedom.Start:VCarveNativeAxialFreedom.None)

       |(b==points.Length-1?VCarveNativeAxialFreedom.End:VCarveNativeAxialFreedom.None);

   if(fit is null&&contract==VCarveNativeSourceContract.MedialPlanarWithStockContactDepth&&freedom!=VCarveNativeAxialFreedom.None)

   {

    budget.CheckCandidate();

    var contact=VCarveNativeStockContactFit.Find(points[a..(b+1)],u,v,guard,profile,settings,token,out _,freedom);

    if(contact is not null)fit=contact with{First=a,Last=b};

   }

   if(fit is null){string reason="local-repair:"+audit.Outcome;failures[reason]=failures.TryGetValue(reason,out int count)?count+1:1;}return fit;

  }

  try

  {

   for(int i=0;i<fits.Count;i++)

   {

    token.ThrowIfCancellationRequested();if(fits[i].LengthLower>=settings.MinimumPrimitiveLengthMm)continue;

    IEnumerable<(int Left,int Right)> windows=new[]{(i-1,i+1),(i-1,i),(i,i+1),(i-2,i+1),(i-1,i+2)};

    // One bounded whole-block attempt for an unresolved protected-end tail.

    // Never crosses the caller's canonical topology/state boundary.

    if(i==0||i==fits.Count-1)windows=windows.Append((0,fits.Count-1));

    foreach(var (left,right) in windows.Distinct())

    {

     if(left<0||right>=fits.Count)continue;int a=fits[left].First,b=fits[right].Last;

     if(b-a+1>maximumBlockPoints||cumulative[b]-cumulative[a]<settings.MinimumPrimitiveLengthMm)continue;

     // Protected block directions belong to the prepared source contract, never

     // to a retained raw short or a previously fitted incumbent.

     var u=a==0?boundaryStart:VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(fits[left].Move,0));

     var v=b==points.Length-1?boundaryEnd:VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(fits[right].Move,1));

     var whole=Find(a,b,u,v);

     if(whole is not null)

     {fits.RemoveRange(left,right-left+1);fits.Insert(left,whole);i=Math.Max(-1,left-1);break;}

     var knots=new SortedSet<int>();double minimum=settings.MinimumPrimitiveLengthMm;

     foreach(double target in new[]{cumulative[fits[i].First]-minimum,cumulative[fits[i].First]-2*minimum,cumulative[fits[i].Last]+minimum,cumulative[fits[i].Last]+2*minimum,(cumulative[a]+cumulative[b])/2})

     {

      int near=Array.BinarySearch(cumulative,target);if(near<0)near=~near;

      for(int delta=-1;delta<=1;delta++)if(near+delta>a&&near+delta<b)knots.Add(near+delta);

     }

     bool repaired=false;

     foreach(int middle in knots)

     {

      if(cumulative[middle]-cumulative[a]<minimum||cumulative[b]-cumulative[middle]<minimum)continue;

      var reference=fits.Skip(left).Take(right-left+1).FirstOrDefault(f=>f.First<middle&&f.Last>middle);if(reference is null)continue;

      var tangents=new List<Point3D>();

      var witness=VCarveNativeGeometricProof.Prove(reference.Move,points,reference.First,reference.Last,settings.FitErrorMm);

      if(witness.Certified) tangents.Add(VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(reference.Move,witness.Parameters[middle-reference.First])));

      int before=sourceKnots.Last(n=>n<middle),after=sourceKnots.First(n=>n>middle);

      // An incumbent is one nomination, not an authored tangent contract at a

      // NEW free join. Both sides share each proposed direction and still earn

      // independent raw-geometry, depth and continuous cutter certificates.

      tangents.Add(VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[after],points[before])));

      foreach(var tangent in tangents)

      {

       var first=Find(a,middle,u,tangent);if(first is null)continue;

       var last=Find(middle,b,tangent,v);if(last is null)continue;

       fits.RemoveRange(left,right-left+1);fits.InsertRange(left,new[]{first,last});i=Math.Max(-1,left-1);repaired=true;break;

      }

      if(repaired)break;

     }

     if(repaired)break;

    }

   }

   exhausted=false;return fits;

  }

  catch(VCarveNativeBudgetException){exhausted=true;return fits;}

 }

}
