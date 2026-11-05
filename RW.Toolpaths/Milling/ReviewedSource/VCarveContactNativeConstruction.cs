using RW.Toolpaths;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Milling.ReviewedSource;

// Native C1 contact construction: shared cubic B-spline control net. Guide samples own neither internal positions nor
// derivatives. Exact conversion preserves native cubic Bezier primitives.
internal static class VCarveContactNativeConstruction
{
    internal sealed record Result(bool Verified,string Status,ToolMove[] Moves,double ErrorBound,int CandidateFamilies,Dictionary<string,int> Rejections,ToolMove? FirstStockRefusal=null)
    { public object[] Diagnostics {get;init;}=[]; }
    internal sealed record PortSeed(Point3D StartHandle,Point3D EndHandle);
    internal sealed record ConstraintReceipt(int Samples,int Iterations,double MaximumViolation,double WorstParameter=.5,QpAudit? Audit=null);
    internal sealed record QpAudit(double[][] Inverse,double[] Optimum,LinearConstraint[] Constraints,QpResult Result);
    internal sealed record LinearConstraint(double[] A,double Bound);
    internal sealed record NativeBandRow(double[] Coefficients,double MinimumZ,double Parameter);
    static double[][] BezierRestrictionRows(double[] knots,int count,double a,double b)
    {
        double[] D(double t){var basis=Basis(knots[1..^1],2,count-1,t);var row=new double[count];for(int i=0;i<count-1;i++){double f=3*basis[i]/(knots[i+4]-knots[i+1]);row[i]-=f;row[i+1]+=f;}return row;}
        var left=Basis(knots,3,count,a);var right=Basis(knots,3,count,b);var da=D(a);var db=D(b);return new[]{left,Enumerable.Range(0,count).Select(i=>left[i]+da[i]*(b-a)/3).ToArray(),Enumerable.Range(0,count).Select(i=>right[i]-db[i]*(b-a)/3).ToArray(),right};
    }
    internal sealed record QpResult(bool Converged,double[] Values,int Iterations,double MaximumViolation);
    internal static QpResult SolveContactQp(double[,] inverse,double[] optimum,IReadOnlyList<LinearConstraint> original)
    {
        int n=optimum.Length;double Dot(double[] a,double[] b)=>Enumerable.Range(0,n).Sum(i=>a[i]*b[i]);
        var rows=original.Select(q=>{var d=Enumerable.Range(0,n).Select(i=>Enumerable.Range(0,n).Sum(j=>inverse[i,j]*q.A[j])).ToArray();double scale=Math.Sqrt(Dot(q.A,d));return(A:q.A.Select(v=>v/scale).ToArray(),B:q.Bound/scale,D:d.Select(v=>v/scale).ToArray());}).ToArray();
        double violation(double[] v)=>original.Max(q=>q.Bound-Dot(q.A,v));if(violation(optimum)<=1e-8)return new(true,optimum.ToArray(),0,violation(optimum));
        // Depth planes introduce signed rows. Find a feasible point with a
        // bounded metric projection, then retain the active-set KKT optimum
        // check below. Feasibility alone never authorizes an emitted curve.
        var z=new double[n];var dual=new double[rows.Length];bool feasible=violation(z)<=1e-8;
        if(!feasible)z=optimum.ToArray();
        for(int pass=0;pass<256&&!feasible;pass++)
        {
            for(int j=0;j<rows.Length;j++)
            {
                double change=Math.Max(-dual[j],rows[j].B-Dot(rows[j].A,z));dual[j]+=change;
                for(int i=0;i<n;i++)z[i]+=change*rows[j].D[i];
            }
            if(violation(z)<=1e-8){feasible=true;break;}
        }
        if(!feasible)return new(false,z,256,violation(z));
        var active=new List<int>();bool converged=false;int iterations=0;
        // Orthogonalize active normals in the control metric. Forming their
        // Gram matrix squares the condition number; cancellation can leave a
        // false nonzero step at a fully constrained point, preventing removal
        // of a negative multiplier forever. Reorthogonalized QR computes the
        // tangent step directly, without changing feasibility/KKT tolerances.
        bool ProjectActive(out double[] step,out double[] multipliers)
        {
            step=Enumerable.Range(0,n).Select(i=>optimum[i]-z[i]).ToArray();
            int m=active.Count;multipliers=new double[m];
            if(m==0)return true;
            if(m>n)return false;
            var normals=new List<double[]>();var directions=new List<double[]>();
            var triangular=new double[m,m];var projection=new double[m];
            for(int j=0;j<m;j++)
            {
                var normal=rows[active[j]].A.ToArray();var direction=rows[active[j]].D.ToArray();
                for(int pass=0;pass<2;pass++)for(int k=0;k<j;k++)
                {
                    double coefficient=Dot(normals[k],direction);triangular[k,j]+=coefficient;
                    for(int i=0;i<n;i++){normal[i]-=coefficient*normals[k][i];direction[i]-=coefficient*directions[k][i];}
                }
                double norm=Math.Sqrt(Math.Max(0,Dot(normal,direction)));
                // Numerical rank only; this is not a geometry acceptance rule.
                if(!double.IsFinite(norm)||norm<=64*n*2.2204460492503131e-16)return false;
                triangular[j,j]=norm;normals.Add(normal.Select(v=>v/norm).ToArray());directions.Add(direction.Select(v=>v/norm).ToArray());
            }
            for(int j=0;j<m;j++)projection[j]=Dot(normals[j],step);
            for(int j=0;j<m;j++)for(int i=0;i<n;i++)step[i]-=projection[j]*directions[j][i];
            for(int j=m-1;j>=0;j--)
            {
                double value=-projection[j];for(int k=j+1;k<m;k++)value-=triangular[j,k]*multipliers[k];
                multipliers[j]=value/triangular[j,j];
            }
            // A full-rank active set has no tangent freedom. Verify its actual
            // stationarity residual before treating the computed step as zero.
            if(m==n)
            {
                var residual=new double[n];for(int i=0;i<n;i++){residual[i]=optimum[i]-z[i];for(int j=0;j<m;j++)residual[i]+=multipliers[j]*rows[active[j]].D[i];}
                if(residual.Max(Math.Abs)>1e-10)return false;
                Array.Clear(step);
            }
            return true;
        }
        for(;iterations<256;iterations++)
        {
            if(!ProjectActive(out var p,out var multipliers))break;
            if(p.Max(Math.Abs)<1e-10)
            {
                if(multipliers.Length==0||multipliers.Min()>=-1e-10){converged=true;break;}
                active.RemoveAt(Enumerable.Range(0,multipliers.Length).MinBy(i=>multipliers[i]));continue;
            }
            double alpha=1;int blocker=-1;
            for(int j=0;j<rows.Length;j++)if(!active.Contains(j))
            {
                double direction=Dot(rows[j].A,p);if(direction>=-1e-12)continue;
                double candidate=Math.Max(0,(Dot(rows[j].A,z)-rows[j].B)/(-direction));
                if(candidate<alpha){alpha=candidate;blocker=j;}
            }
            for(int i=0;i<n;i++)z[i]+=alpha*p[i];
            if(blocker>=0&&alpha<1)active.Add(blocker);
        }
        double finalViolation=violation(z);return new(converged&&finalViolation<=1e-8,z,iterations+1,finalViolation);
    }
    internal static Point3D[]? NominateContinuousControlNet(Point3D[] source,double[] interiorKnots)
    {
        var parameter=new double[source.Length];for(int i=1;i<source.Length;i++)parameter[i]=parameter[i-1]+VCarveNativeCurveMath.Distance(source[i-1],source[i]);double length=parameter[^1];if(length<=0)return null;for(int i=0;i<parameter.Length;i++)parameter[i]/=length;var knots=new[]{0d,0,0,0}.Concat(interiorKnots).Concat(new[]{1d,1,1,1}).ToArray();return SolveControl(source,parameter,knots,knots.Length-4,null);
    }
    static IEnumerable<(double T,Point3D Source,double Weight)> ContinuousGuideQuadrature(Point3D[] source,double[] parameter,double[] knots)
    {
        // Cubic basis products have degree6 and basis*linear-guide degree4.
        // Four-point Gauss integration is exact through degree7 on each union
        // of source-chord and native-knot intervals. These are algebraic
        // integration nodes, never persisted/tessellated motion primitives.
        double[] nodes=[-.8611363115940526,-.3399810435848563,.3399810435848563,.8611363115940526],weights=[.3478548451374538,.6521451548625461,.6521451548625461,.3478548451374538];
        var intervals=parameter.Concat(knots).Distinct().Order().ToArray();for(int k=1;k<intervals.Length;k++){
            double half=(intervals[k]-intervals[k-1])/2,mid=(intervals[k]+intervals[k-1])/2;if(half<=0)continue;
            for(int q=0;q<4;q++){double t=mid+half*nodes[q];int i=Array.BinarySearch(parameter,t);if(i<0)i=~i-1;i=Math.Clamp(i,0,source.Length-2);double denominator=parameter[i+1]-parameter[i];if(denominator<=0)continue;double f=(t-parameter[i])/denominator;var p=VCarveNativeCurveMath.Add(VCarveNativeCurveMath.Mul(source[i],1-f),VCarveNativeCurveMath.Mul(source[i+1],f));yield return(t,p,half*weights[q]);}
        }
    }
    static double[,]? InverseControlMetric(Point3D[] source,double[] parameter,double[] knots,int count)
    {
        int n=count-2;var h=new double[n,n*2];foreach(var row in ContinuousGuideQuadrature(source,parameter,knots)){var b=Basis(knots,3,count,row.T);for(int i=0;i<n;i++)for(int j=0;j<n;j++)h[i,j]+=b[i+1]*b[j+1]*row.Weight;}for(int i=0;i<n;i++)h[i,n+i]=1;
        for(int i=0;i<n;i++){int pivot=Enumerable.Range(i,n-i).MaxBy(j=>Math.Abs(h[j,i]));if(Math.Abs(h[pivot,i])<1e-14)return null;if(pivot!=i)for(int j=0;j<n*2;j++)(h[i,j],h[pivot,j])=(h[pivot,j],h[i,j]);double divisor=h[i,i];for(int j=0;j<n*2;j++)h[i,j]/=divisor;for(int row=0;row<n;row++)if(row!=i){double f=h[row,i];for(int j=0;j<n*2;j++)h[row,j]-=f*h[i,j];}}
        var inverse=new double[n,n];for(int i=0;i<n;i++)for(int j=0;j<n;j++)inverse[i,j]=h[i,n+j];return inverse;
    }
    static bool ConstrainPlanarProgress(Point3D[] c,double[] knots,double[] partitions,Point3D[] source,double[] parameter)
    {
        int n=c.Length-2;var metric=InverseControlMetric(source,parameter,knots,c.Length);if(metric is null)return false;
        var inverse=new double[2*n,2*n];for(int i=0;i<n;i++)for(int j=0;j<n;j++){inverse[i,j]=metric[i,j];inverse[n+i,n+j]=metric[i,j];}
        var origin=c[0];var end=VCarveNativeCurveMath.Sub(c[^1],origin);
        var optimum=c[1..^1].Select(p=>p.X-origin.X).Concat(c[1..^1].Select(p=>p.Y-origin.Y)).ToArray();
        var constraints=new List<LinearConstraint>();
        for(int span=1;span<partitions.Length;span++)
        {
            double a=partitions[span-1],b=partitions[span];var direction=VCarveNativeCurveMath.Sub(At(c,knots,3,b),At(c,knots,3,a));
            direction=VCarveNativeCurveMath.Unit(new(direction.X,direction.Y,0));
            if(direction.X==0&&direction.Y==0)continue;
            var rows=BezierRestrictionRows(knots,c.Length,a,b);
            for(int k=0;k<3;k++)
            {
                var delta=rows[k+1].Zip(rows[k],(x,y)=>x-y).ToArray();
                var row=delta[1..^1].Select(v=>v*direction.X).Concat(delta[1..^1].Select(v=>v*direction.Y)).ToArray();
                double bound=-delta[^1]*(end.X*direction.X+end.Y*direction.Y);
                if(row.All(v=>v==0)){if(bound>1e-8)return false;continue;}
                constraints.Add(new(row,bound));
            }
        }
        if(constraints.Count==0)return true;
        var result=SolveContactQp(inverse,optimum,constraints);if(!result.Converged)return false;
        for(int i=0;i<n;i++)c[i+1]=new(origin.X+result.Values[i],origin.Y+result.Values[n+i],c[i+1].Z);
        return true;
    }
    static bool ConstrainZ(Point3D[] c,double[] knots,Point3D[] source,double[] parameter,Func<Point3D,Point3D> contact,out ConstraintReceipt receipt,double maximumDepth,IReadOnlyList<NativeBandRow>? bands=null)
    {
        int n=c.Length-2;var h=InverseControlMetric(source,parameter,knots,c.Length);
        if(h is null){receipt=new(0,0,double.PositiveInfinity);return false;}
        var parameters=Enumerable.Range(1,8*c.Length-1).Select(i=>(double)i/(8*c.Length)).Concat(Enumerable.Range(1,10).SelectMany(i=>new[]{Math.Pow(.5,i),1-Math.Pow(.5,i)})).Distinct().Order().ToArray();
        var constraints=new List<(double T,double[] A,double B,double[] Direction,double Denominator)>();
        foreach(double t in parameters){var b=Basis(knots,3,c.Length,t);var p=At(c,knots,3,t);double safeZ=contact(p).Z;var a=b[1..^1];double bound=safeZ-b[0]*c[0].Z-b[^1]*c[^1].Z;var d=Enumerable.Range(0,n).Select(i=>Enumerable.Range(0,n).Sum(j=>h[i,j]*a[j])).ToArray();double den=Enumerable.Range(0,n).Sum(i=>a[i]*d[i]);if(den>1e-20)constraints.Add((t,a,bound,d,den));}
        foreach(var band in bands??[]){var b=band.Coefficients;var a=b[1..^1];double bound=band.MinimumZ-b[0]*c[0].Z-b[^1]*c[^1].Z;var d=Enumerable.Range(0,n).Select(i=>Enumerable.Range(0,n).Sum(j=>h[i,j]*a[j])).ToArray();double den=Enumerable.Range(0,n).Sum(i=>a[i]*d[i]);if(den>1e-20)constraints.Add((band.Parameter,a,bound,d,den));else if(bound>1e-8){receipt=new(constraints.Count,0,bound,band.Parameter);return false;}}
        for(int k=0;k<n;k++)
        {
            var lower=new double[n];lower[k]=1;var upper=new double[n];upper[k]=-1;
            var dl=Enumerable.Range(0,n).Select(i=>h[i,k]).ToArray();
            constraints.Add((.5,lower,-maximumDepth,dl,h[k,k]));
            constraints.Add((.5,upper,0,dl.Select(v=>-v).ToArray(),h[k,k]));
        }
        var inverse=h;var solution=SolveContactQp(inverse,c[1..^1].Select(p=>p.Z).ToArray(),constraints.Select(q=>new LinearConstraint(q.A,q.B)).ToArray());var z=solution.Values;
        receipt=new(constraints.Count,solution.Iterations,solution.MaximumViolation,constraints.MaxBy(q=>q.B-Enumerable.Range(0,n).Sum(i=>q.A[i]*z[i])).T,new(Enumerable.Range(0,n).Select(i=>Enumerable.Range(0,n).Select(j=>inverse[i,j]).ToArray()).ToArray(),c[1..^1].Select(p=>p.Z).ToArray(),constraints.Select(q=>new LinearConstraint(q.A,q.B)).ToArray(),solution));for(int i=0;i<n;i++)c[i+1]=c[i+1] with{Z=Math.Clamp(z[i],-maximumDepth,0)};return solution.Converged;
    }
    static ToolMove NativePiece(Point3D[] control,double[] knots,double a,double b)
    {
        var p=At(control,knots,3,a);var q=At(control,knots,3,b);
        return ToolMove.CubicBezier(p,VCarveNativeCurveMath.Add(p,VCarveNativeCurveMath.Mul(Derivative(control,knots,a),(b-a)/3)),
            VCarveNativeCurveMath.Sub(q,VCarveNativeCurveMath.Mul(Derivative(control,knots,b),(b-a)/3)),q,0);
    }
    static double[]? RedistributeShortJoin(Point3D[] control,double[] knots,double[] partitions,int shortSpan,double minimum)
    {
        int count=partitions.Length-1;
        var lengths=Enumerable.Range(0,count).Select(i=>VCarveNativeCurveMath.Length(NativePiece(control,knots,partitions[i],partitions[i+1]))).ToArray();
        var current=lengths[shortSpan];double needed=minimum-current.Lower+(current.Upper-current.Lower)+VCarveNativeGeometricProof.ArithmeticReserve;
        double left=shortSpan>0?lengths[shortSpan-1].Lower-minimum:double.NegativeInfinity,
            right=shortSpan+1<count?lengths[shortSpan+1].Lower-minimum:double.NegativeInfinity;
        var result=partitions.ToArray();
        if(Math.Max(left,right)>=needed)
        {
            if(left>=right)
            {
                double a=partitions[shortSpan-1],b=partitions[shortSpan],low=a,high=b;
                for(int i=0;i<40;i++){double mid=(low+high)/2;if(VCarveNativeCurveMath.Length(NativePiece(control,knots,mid,b)).Lower>=needed)low=mid;else high=mid;}
                result[shortSpan]=low;
            }
            else
            {
                double a=partitions[shortSpan+1],b=partitions[shortSpan+2],low=a,high=b;
                for(int i=0;i<40;i++){double mid=(low+high)/2;if(VCarveNativeCurveMath.Length(NativePiece(control,knots,a,mid)).Lower>=needed)high=mid;else low=mid;}
                result[shortSpan+1]=high;
            }
            return result;
        }
        // If neighbouring pieces have no spare length, remove a free fitting
        // join and refit its larger span. No source edge or executed motion is
        // deleted, and the full continuous source certificate is still required.
        int remove=left>=right?shortSpan:shortSpan+1;
        return remove>0&&remove<partitions.Length-1?partitions.Where((_,i)=>i!=remove).ToArray():null;
    }
    static ToolMove[]? CertifiedGlobalReference(Point3D[] control,double[] knots,double[] partitions,
        IReadOnlyList<Point3D> reference,int[] indices,VCarveNativeFitSettings settings,double maximumDepth,Func<ToolMove,bool> stock)
    {
        var original=Enumerable.Range(0,partitions.Length-1).Select(i=>NativePiece(control,knots,partitions[i],partitions[i+1])).ToArray();
        double error=0;
        for(int i=0;i<original.Length;i++)
        {
            if(new[]{original[i].Start,original[i].Control1,original[i].Control2,original[i].End}.Any(p=>p.Z< -maximumDepth||p.Z>0)
                ||VCarveNativeCurveMath.Length(original[i]).Lower<settings.MinimumPrimitiveLengthMm)return null;
            var proof=VCarveNativeGeometricProof.ProveFreeJoin(original[i],reference,indices[i],indices[i+1],settings.FitErrorMm);
            if(!proof.Certified)return null;error=Math.Max(error,proof.ErrorBound);original[i]=original[i] with{MaxError=proof.ErrorBound};
        }
        if(original.All(stock))return original;
        double low=0,high=Math.Max(0,settings.FitErrorMm-error);if(high<=0)return null;
        ToolMove[] Lift(double d)
        {
            var lifted=control.ToArray();for(int i=1;i<lifted.Length-1;i++)lifted[i]=lifted[i] with{Z=Math.Min(0,lifted[i].Z+d)};
            return Enumerable.Range(0,partitions.Length-1).Select(i=>NativePiece(lifted,knots,partitions[i],partitions[i+1])).ToArray();
        }
        if(!Lift(high).All(stock))return null;
        for(int iteration=0;iteration<18;iteration++){double mid=(low+high)/2;if(Lift(mid).All(stock))high=mid;else low=mid;}
        var result=Lift(high);
        for(int i=0;i<result.Length;i++)
        {
            if(new[]{result[i].Start,result[i].Control1,result[i].Control2,result[i].End}.Any(p=>p.Z< -maximumDepth||p.Z>0)
                ||VCarveNativeCurveMath.Length(result[i]).Lower<settings.MinimumPrimitiveLengthMm)return null;
            var proof=VCarveNativeGeometricProof.ProveFreeJoin(result[i],reference,indices[i],indices[i+1],settings.FitErrorMm);
            if(!proof.Certified||!stock(result[i]))return null;result[i]=result[i] with{MaxError=proof.ErrorBound};
        }
        return result;
    }
    internal static Result ConstructAdaptiveContact(Point3D[] source,VCarveNativeFitSettings settings,double maximumDepth,Func<Point3D,Point3D> contact,Func<ToolMove,bool> fullStock,int maximumPieces,Action<string>? diagnostic=null,Func<ToolMove,VCarveContactCurveCorridor.LockedPorts,double,VCarveContactCurveCorridor.Result?>? corridor=null,int maximumTrials=50_000)
    {
        settings.Validate();var rejected=new Dictionary<string,int>();void Reject(string reason)=>rejected[reason]=rejected.GetValueOrDefault(reason)+1;var parameter=new double[source.Length];for(int i=1;i<source.Length;i++)parameter[i]=parameter[i-1]+VCarveNativeCurveMath.Distance(source[i-1],source[i]);double total=parameter[^1];if(total<settings.MinimumPrimitiveLengthMm)return new(false,"source-branch-below-minimum-no-padding",[],0,0,rejected);for(int i=0;i<parameter.Length;i++)parameter[i]/=total;
        var trace=new List<object>();const double lateral=1;
        var breaks=new SortedSet<double>{0,1};ToolMove? firstRefusal=null;int trials=0;var contactMemo=new Dictionary<Point3D,Point3D>();Point3D CachedContact(Point3D p){if(!contactMemo.TryGetValue(p,out var safe))contactMemo[p]=safe=contact(p);return safe;}var bands=new List<NativeBandRow>();var attemptedPartitions=new HashSet<string>();string basisKey="";int corridorRefinements=0;
        while(breaks.Count-1<=maximumPieces&&trials<maximumTrials){
            trials++;var boundaries=breaks.ToArray();int spans=boundaries.Length-1;var knots=new[]{0d,0,0,0}.Concat(boundaries[1..^1].SelectMany(t=>new[]{t,t})).Concat(new[]{1d,1,1,1}).ToArray();int count=knots.Length-4;var c=SolveControl(source,parameter,knots,count,null);if(c is null){Reject("singular-control-system");break;}
            string key=string.Join(";",boundaries.Select(t=>t.ToString("R")));if(key!=basisKey){if(!attemptedPartitions.Add(key)){Reject("partition-search-cycle");break;}basisKey=key;bands.Clear();corridorRefinements=0;}
            if(!ConstrainPlanarProgress(c,knots,boundaries,source,parameter))
                diagnostic?.Invoke("[AdaptiveContactNative] planarProgressNominationUnresolved=True finalGeometricProofStillRequired=True");
            var beforeConstraints=c.ToArray();
            bool constrained=ConstrainZ(c,knots,source,parameter,CachedContact,out var receipt,maximumDepth,bands);diagnostic?.Invoke($"[AdaptiveContactNative] pieces={spans} controlPoints={count} constraints={receipt.Samples} iterations={receipt.Iterations} violation={receipt.MaximumViolation:R} pointConstraintsSolved={constrained} corridorRefinements={corridorRefinements}");
            var pieces=Enumerable.Range(0,spans).Select(i=>NativePiece(c,knots,boundaries[i],boundaries[i+1])).ToArray();
            var mapping=VCarveContactSourceCorrespondence.Nominate(pieces,source,parameter);
            var reference=mapping.Reference.ToList();var referenceTimes=mapping.SourceParameters;var indices=mapping.Indices;
            if(constrained)
            {
                var global=CertifiedGlobalReference(c,knots,boundaries,reference,indices,settings,maximumDepth,fullStock);
                if(global is not null&&global[0].Start==source[0]&&global[^1].End==source[^1])
                {
                    diagnostic?.Invoke("[AdaptiveContactNative] completeC1NetCertified=True fitDepthMinimumAndStockRechecked=True");
                    return new(true,"global-C1-contact-fit-and-stock-certified-coverage-separate",global,global.Max(m=>m.MaxError),trials,rejected,firstRefusal);
                }
            }
            var moves=new List<ToolMove>();var refinement=new List<(double T,double Score)>();var shapeNominations=new List<(double T,double Bend)>();bool passed=constrained,refineCorridor=false;double error=0,failedMid=.5;int shortSpan=-1;
            if(!constrained){trace.Add(new{trial=trials,pieces=spans,lateral,reason="constraint-system",receipt,beforeConstraints,afterConstraints=c.ToArray(),boundaries});Reject("stock-point-constraint-system-unresolved");refinement.Add((receipt.WorstParameter,receipt.MaximumViolation));}
            if(constrained)for(int i=0;i<spans;i++)
            {
                var candidate=NativePiece(c,knots,boundaries[i],boundaries[i+1]);
                if(VCarveNativeCurveMath.Length(candidate).Lower<settings.MinimumPrimitiveLengthMm)
                {shortSpan=i;Reject("minimum-length");passed=false;break;}
            }
            if(constrained&&passed)for(int i=0;i<spans;i++){
                double a=boundaries[i],b=boundaries[i+1];failedMid=(a+b)/2;var p=At(c,knots,3,a);var q=At(c,knots,3,b);var move=ToolMove.CubicBezier(p,VCarveNativeCurveMath.Add(p,VCarveNativeCurveMath.Mul(Derivative(c,knots,a),(b-a)/3)),VCarveNativeCurveMath.Sub(q,VCarveNativeCurveMath.Mul(Derivative(c,knots,b),(b-a)/3)),q,0);var range=VCarveNativeCurveMath.ZRange(move);
                if(new[]{move.Start,move.Control1,move.Control2,move.End}.Any(p=>p.Z< -maximumDepth||p.Z>0)){Reject("depth");passed=false;break;}if(VCarveNativeCurveMath.Length(move).Lower<settings.MinimumPrimitiveLengthMm){shortSpan=i;Reject("minimum-length");passed=false;break;}var proof=VCarveNativeGeometricProof.ProveFreeJoin(move,reference,indices[i],indices[i+1],settings.FitErrorMm);if(!proof.Certified){
                    trace.Add(new{trial=trials,pieces=spans,lateral,reason="continuous-geometry",receipt,beforeConstraints,afterConstraints=c.ToArray(),boundaries,move,geometry=proof,reference=reference.Skip(indices[i]).Take(indices[i+1]-indices[i]+1).ToArray()});
                    Reject("continuous-ordered-fit:"+proof.Reason);passed=false;
                    if(proof.Parameters.Length==indices[i+1]-indices[i]+1)for(int r=0;r<proof.Parameters.Length;r++)
                    {
                        int ordinal=indices[i]+r;double nativeParameter=a+(b-a)*proof.Parameters[r];
                        refinement.Add((nativeParameter,VCarveNativeCurveMath.Distance(reference[ordinal],VCarveNativeCurveMath.At(move,proof.Parameters[r]))));
                        if(r>0&&r+1<proof.Parameters.Length)
                        {
                            double bend=VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(reference[ordinal],reference[ordinal-1])),VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(reference[ordinal+1],reference[ordinal]))));
                            shapeNominations.Add((nativeParameter,bend));
                        }
                        if(r+1<proof.Parameters.Length)
                        {
                            var sourceMid=VCarveNativeCurveMath.Mul(VCarveNativeCurveMath.Add(reference[ordinal],reference[ordinal+1]),.5);
                            double mid=(proof.Parameters[r]+proof.Parameters[r+1])/2;
                            refinement.Add((a+(b-a)*mid,VCarveNativeCurveMath.Distance(sourceMid,VCarveNativeCurveMath.At(move,mid))));
                        }
                    }
                    break;
                }error=Math.Max(error,proof.ErrorBound);moves.Add(move with{MaxError=proof.ErrorBound});
            }
            // Source geometry and minimum length apply to the COMPLETE net
            // before stock refinement. Do not chase clearance on an early
            // piece while a later piece is already geometrically invalid.
            if(passed)for(int i=0;i<moves.Count;i++)
            {
                var move=moves[i];double a=boundaries[i],b=boundaries[i+1];failedMid=(a+b)/2;
                if(!fullStock(move)){
                    firstRefusal??=move;Reject("full-stock-curve-unresolved");passed=false;
                    if(corridor is not null&&corridorRefinements<8){var ports=(a==0?VCarveContactCurveCorridor.LockedPorts.Start:VCarveContactCurveCorridor.LockedPorts.None)|(b==1?VCarveContactCurveCorridor.LockedPorts.End:VCarveContactCurveCorridor.LockedPorts.None);var band=corridor(move,ports,Math.Max(0,settings.FitErrorMm-move.MaxError));if(band is not null){double left=a+(b-a)*band.First,right=a+(b-a)*band.Last;var rows=BezierRestrictionRows(knots,count,left,right);double[] lower=[band.SafeReference.Start.Z,band.SafeReference.Control1.Z,band.SafeReference.Control2.Z,band.SafeReference.End.Z];for(int k=0;k<4;k++)bands.Add(new(rows[k],lower[k],left+(right-left)*k/3));corridorRefinements++;refineCorridor=true;diagnostic?.Invoke($"[AdaptiveContactNative] certifiedReferenceBand={left:R}..{right:R} finalWholePrimitiveProofStillRequired=True");}}
                    for(int r=1;r<8;r++){double u=r/8d;var sample=VCarveNativeCurveMath.At(move,u);double deficit=CachedContact(sample).Z-sample.Z;refinement.Add((a+(b-a)*u,Math.Max(0,deficit)));}break;
                }
            }
            if(passed&&moves[0].Start==source[0]&&moves[^1].End==source[^1])return new(true,"adaptive-stock-constrained-native-C1-coverage-unproven",moves.ToArray(),error,trials,rejected,firstRefusal);
            if(shortSpan>=0)
            {
                var redistributed=RedistributeShortJoin(c,knots,boundaries,shortSpan,settings.MinimumPrimitiveLengthMm);
                if(redistributed is null){Reject("native-minimum-join-unresolved");break;}
                breaks=new SortedSet<double>(redistributed);
                diagnostic?.Invoke($"[AdaptiveContactNative] redistributeFreeJoinForMinimum=True shortPiece={shortSpan} primitiveArcLengthUsed=True sourceAndJunctionsUnchanged=True");
                continue;
            }
            if(refineCorridor)continue;
            if(spans>=maximumPieces)break;
            // Use the GEOMETRIC correspondence witness or a stock deficit;
            // equal source/native parameter residuals impose a false timing law.
            double spacing=settings.MinimumPrimitiveLengthMm/total;var eligible=Enumerable.Range(0,spans).Where(i=>boundaries[i+1]-boundaries[i]>2*spacing).ToArray();if(eligible.Length==0){Reject("no-unpadded-native-partition");break;}double suggested=refinement.Count==0?failedMid:refinement.OrderByDescending(r=>r.Score).First().T;
            int interval=eligible.MinBy(i=>Math.Max(boundaries[i]-suggested,Math.Max(0,suggested-boundaries[i+1])));
            if(boundaries.Any(t=>Math.Abs(t-suggested)<=spacing))
            {
                // A displaced free join is not a request for an ever smaller
                // source interval beside that join. Refine a genuine shape
                // event in the failing span instead of repeatedly clamping the
                // same witness to the minimum spacing. Bend is a nomination
                // ranking only; no angular tolerance authorizes geometry.
                interval=eligible.MinBy(i=>Math.Max(boundaries[i]-failedMid,Math.Max(0,failedMid-boundaries[i+1])));
                var features=shapeNominations.Where(f=>f.T>boundaries[interval]+spacing&&f.T<boundaries[interval+1]-spacing).OrderByDescending(f=>f.Bend).ToArray();
                suggested=features.Length>0?features[0].T:(boundaries[interval]+boundaries[interval+1])/2;
                diagnostic?.Invoke($"[AdaptiveContactNative] existingJoinWitness=True shapeEventNomination={suggested:R} arbitrarySourceEndpointNotRequired=True");
            }
            double inserted=Math.Clamp(suggested,boundaries[interval]+spacing,boundaries[interval+1]-spacing);breaks.Add(inserted);diagnostic?.Invoke($"[AdaptiveContactNative] insertFreeKnot={inserted:R} geometricOrStockWitness=True sourceTimeNotAcceptanceLaw=True");
        }
        return new(false,"adaptive-stock-constrained-native-C1-unresolved",[],0,trials,rejected,firstRefusal){Diagnostics=trace.ToArray()};
    }

    static double[] Basis(double[] knots,int degree,int count,double t)
    {
        if(t>=knots[^1]){var end=new double[count];end[^1]=1;return end;}
        var b=new double[knots.Length-1];for(int i=0;i<b.Length;i++)b[i]=knots[i]<=t&&t<knots[i+1]?1:0;
        for(int p=1;p<=degree;p++){var next=new double[b.Length-1];for(int i=0;i<next.Length;i++){double a=knots[i+p]-knots[i],z=knots[i+p+1]-knots[i+1];next[i]=(a>0?(t-knots[i])/a*b[i]:0)+(z>0?(knots[i+p+1]-t)/z*b[i+1]:0);}b=next;}
        return b.Take(count).ToArray();
    }
    static Point3D At(Point3D[] control,double[] knots,int degree,double t){var b=Basis(knots,degree,control.Length,t);var p=new Point3D(0,0,0);for(int i=0;i<control.Length;i++)p=VCarveNativeCurveMath.Add(p,VCarveNativeCurveMath.Mul(control[i],b[i]));return p;}
    static Point3D Derivative(Point3D[] c,double[] knots,double t){var d=new Point3D[c.Length-1];for(int i=0;i<d.Length;i++)d[i]=VCarveNativeCurveMath.Mul(VCarveNativeCurveMath.Sub(c[i+1],c[i]),3/(knots[i+4]-knots[i+1]));return At(d,knots[1..^1],2,t);}
    static Point3D[]? SolveControl(Point3D[] source,double[] parameter,double[] knots,int count,PortSeed? portSeed)
    {
        var fixedControls=new Dictionary<int,Point3D>{{0,source[0]},{count-1,source[^1]}};if(portSeed is not null){fixedControls[1]=portSeed.StartHandle;fixedControls[count-2]=portSeed.EndHandle;}var free=Enumerable.Range(0,count).Where(i=>!fixedControls.ContainsKey(i)).ToArray();int n=free.Length;var a=new double[n,n+3];
        foreach(var row in ContinuousGuideQuadrature(source,parameter,knots)){
            var b=Basis(knots,3,count,row.T);var rhs=row.Source;foreach(var f in fixedControls)rhs=VCarveNativeCurveMath.Sub(rhs,VCarveNativeCurveMath.Mul(f.Value,b[f.Key]));
            for(int i=0;i<n;i++){for(int j=0;j<n;j++)a[i,j]+=b[free[i]]*b[free[j]]*row.Weight;a[i,n]+=b[free[i]]*rhs.X*row.Weight;a[i,n+1]+=b[free[i]]*rhs.Y*row.Weight;a[i,n+2]+=b[free[i]]*rhs.Z*row.Weight;}
        }
        for(int i=0;i<n;i++){
            int pivot=Enumerable.Range(i,n-i).MaxBy(j=>Math.Abs(a[j,i]));if(Math.Abs(a[pivot,i])<1e-14)return null;
            if(pivot!=i)for(int j=i;j<n+3;j++)(a[i,j],a[pivot,j])=(a[pivot,j],a[i,j]);double divisor=a[i,i];for(int j=i;j<n+3;j++)a[i,j]/=divisor;
            for(int row=0;row<n;row++)if(row!=i){double factor=a[row,i];for(int j=i;j<n+3;j++)a[row,j]-=factor*a[i,j];}
        }
        var result=new Point3D[count];foreach(var f in fixedControls)result[f.Key]=f.Value;for(int i=0;i<n;i++)result[free[i]]=new(a[i,n],a[i,n+1],a[i,n+2]);return result;
    }

}
