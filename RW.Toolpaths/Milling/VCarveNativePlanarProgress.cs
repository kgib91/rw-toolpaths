namespace RW.Toolpaths.Milling;

// Contact projection owns depth, while its immutable planar source owns
// forward travel. A 3D correspondence alone can hide an added XY reversal
// behind progress in Z. Preserve every monotone/constant source projection;
// a genuine source turn does not acquire a new monotonicity requirement.
internal static class VCarveNativePlanarProgress
{
    // Fits are cached in a canonical direction and can later be consumed in
    // either source direction. Near-constant projections retain their literal
    // source origin in the final audit, so both occurrences must be admitted.
    internal sealed class AdmissionContract
    {
        private readonly Contract forward,reverse;
        internal AdmissionContract(Contract forward,Contract reverse){this.forward=forward;this.reverse=reverse;}
        internal bool TrySelectHandles(Point3D start,Point3D end,Point3D u,Point3D v,ref double alpha,ref double beta)
        {
            if(!forward.TrySelectHandles(start,end,u,v,ref alpha,ref beta))return false;
            return reverse.TrySelectHandles(end,start,VCarveNativeCurveMath.Mul(v,-1),VCarveNativeCurveMath.Mul(u,-1),ref beta,ref alpha);
        }
        internal bool Preserved(ToolMove move,out string reason)
        {
            if(!forward.Preserved(move,out reason))return false;
            if(reverse.Preserved(VCarveNativeCurveMath.Reverse(move),out reason))return true;
            reason="reversed-occurrence:"+reason;return false;
        }
    }

    private sealed class ReversedRange(IReadOnlyList<Point3D> source,int first,int last):IReadOnlyList<Point3D>
    {
        public int Count=>last-first+1;
        public Point3D this[int index]=>source[last-index];
        public IEnumerator<Point3D> GetEnumerator(){for(int i=last;i>=first;i--)yield return source[i];}
        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()=>GetEnumerator();
    }

    internal static AdmissionContract CreateAdmission(IReadOnlyList<Point3D> source,int first,int last)
        =>new(Create(source,first,last),Create(new ReversedRange(source,first,last),0,last-first));

    internal readonly record struct Projection(string Name,Point3D Direction,bool Constant,double Sign);
    internal sealed class Contract
    {
        private readonly Point3D origin;
        private readonly Projection[] projections;
        internal Contract(Point3D origin,Projection[] projections){this.origin=origin;this.projections=projections;}
        // Select handle lengths INSIDE the source-owned progress domain before
        // creating any control points. For one monotone projection, derivative
        // Bernstein coefficients are A=alpha*U, B=D-A-C, C=beta*V. The exact
        // nonnegative quadratic condition is D >= A+C-sqrt(A*C). Scaling both
        // positive handles is a feasible parameterization of that convex domain;
        // a feasible original seed is preserved exactly.
        internal bool TrySelectHandles(Point3D start,Point3D end,Point3D u,Point3D v,ref double alpha,ref double beta)
        {
            const double reserve=VCarveNativeGeometricProof.ArithmeticReserve;
            if(!double.IsFinite(alpha)||!double.IsFinite(beta)||alpha<=0||beta<=0)return false;
            double scale=1;
            foreach(var axis in projections)
            {
                double U=VCarveNativeCurveMath.Dot(u,axis.Direction),V=VCarveNativeCurveMath.Dot(v,axis.Direction);
                if(axis.Constant)
                {
                    double left=reserve-Math.Abs(VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(start,origin),axis.Direction));
                    double right=reserve-Math.Abs(VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(end,origin),axis.Direction));
                    if(left<0||right<0)return false;
                    if(U!=0)scale=Math.Min(scale,left/(alpha*Math.Abs(U)));
                    if(V!=0)scale=Math.Min(scale,right/(beta*Math.Abs(V)));
                    continue;
                }
                U*=axis.Sign;V*=axis.Sign;
                double D=axis.Sign*VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(end,start),axis.Direction);
                if(D< -reserve)return false;
                // Tiny negative direction components can arise at float/command
                // boundaries; bound their ENTIRE handle by the existing arithmetic
                // reserve rather than inventing lateral motion.
                if(U<0)scale=Math.Min(scale,reserve/(4*alpha*-U));
                if(V<0)scale=Math.Min(scale,reserve/(4*beta*-V));
                double A=alpha*Math.Max(0,U),C=beta*Math.Max(0,V);
                double required=A+C-Math.Sqrt(A)*Math.Sqrt(C);
                if(required>0)scale=Math.Min(scale,Math.Max(0,D)/required);
            }
            if(scale<1)scale=Math.Max(0,Math.BitDecrement(scale));
            alpha*=scale;beta*=scale;
            return alpha>1e-10&&beta>1e-10;
        }

        internal bool Preserved(ToolMove move,out string reason)
        {
            reason="source-planar-progress-preserved";
            if(move.Kind==ToolMoveKind.Line)return true;
            const double reserve=VCarveNativeGeometricProof.ArithmeticReserve;
            foreach(var axis in projections)
            {
                double Project(Point3D p)=>VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(p,origin),axis.Direction);
                if(axis.Constant)
                {
                    if(Math.Abs(Project(move.Start))>reserve||Math.Abs(Project(move.Control1))>reserve
                        ||Math.Abs(Project(move.Control2))>reserve||Math.Abs(Project(move.End))>reserve)
                    {reason=$"source-constant-planar-axis-{axis.Name}";return false;}
                    continue;
                }
                double a=axis.Sign*VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(move.Control1,move.Start),axis.Direction);
                double b=axis.Sign*VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(move.Control2,move.Control1),axis.Direction);
                double c=axis.Sign*VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(move.End,move.Control2),axis.Direction);
                double minimum=Math.Min(a,c),quadratic=a-2*b+c;
                if(quadratic>0)
                {
                    double t=(a-b)/quadratic;
                    if(t>0&&t<1)minimum=Math.Min(minimum,(quadratic*t+2*(b-a))*t+a);
                }
                if(minimum< -reserve)
                {reason=$"source-planar-progress-reversal-{axis.Name}:minimumDerivativeThirdMm={minimum:R}";return false;}
            }
            return true;
        }

    }

    internal static Contract Create(IReadOnlyList<Point3D> source,int first,int last)
    {
        const double reserve=VCarveNativeGeometricProof.ArithmeticReserve;
        var delta=VCarveNativeCurveMath.Sub(source[last],source[first]) with{Z=0};
        double length=VCarveNativeCurveMath.Norm(delta);
        var axes=new List<(string Name,Point3D Direction)>{("X",new(1,0,0)),("Y",new(0,1,0))};
        if(length>reserve)axes.Add(("chord",VCarveNativeCurveMath.Mul(delta,1/length)));
        var owned=new List<Projection>();
        foreach(var (name,direction) in axes)
        {
            double Project(Point3D p)=>VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(p,source[first]),direction);
            double net=Project(source[last]);
            if(Math.Abs(net)<=reserve)
            {
                bool constant=true;
                for(int i=first;i<=last;i++)if(Math.Abs(Project(source[i]))>reserve){constant=false;break;}
                if(constant)owned.Add(new(name,direction,true,0));
                continue;
            }
            double sign=Math.Sign(net);bool monotone=true;
            for(int i=first+1;i<=last;i++)
                if(sign*VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(source[i],source[i-1]),direction)< -reserve){monotone=false;break;}
            if(!monotone)continue;
            owned.Add(new(name,direction,false,sign));
        }
        return new Contract(source[first],owned.ToArray());
    }

    internal static bool Preserved(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,out string reason)
        =>Create(source,first,last).Preserved(move,out reason);
}

public static partial class VCarveNativeCurvePlanning
{
    public static bool PreservesPlanarProgress(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,out string reason)
        =>VCarveNativePlanarProgress.Preserved(move,source,first,last,out reason);
}
