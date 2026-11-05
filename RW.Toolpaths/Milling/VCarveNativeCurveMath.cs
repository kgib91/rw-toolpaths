using RW.Toolpaths;


namespace RW.Toolpaths.Milling;

// Fitting metadata surrounds the real RW.Toolpaths.ToolMove; it is not a
// replacement curve type. All stored geometry uses the library's factories.
internal sealed record VCarveNativeFit(ToolMove Move,int First,int Last,double SourceLength,
    double LengthLower,double LengthUpper,double TangentStart,double TangentEnd,
    bool OriginalSourceEvidence=false,string Canonical="",int CanonicalFirst=0,int CanonicalLast=0)
{
 public bool StockContactAxialEnvelope {get;init;}
 public bool DepthLimitContour {get;init;}
 public bool PreservedShortFeature {get;init;}
}

internal static class VCarveNativeCurveMath
{
    // Fit error is supplied per operation, never a process-wide mutable value.


    internal static ToolMove TranslateZ(ToolMove move,double delta)
    {
        Point3D P(Point3D p)=>p with{Z=p.Z+delta};
        return move.Kind switch
        {
            ToolMoveKind.Line=>ToolMove.Line(P(move.Start),P(move.End),move.MaxError),
            ToolMoveKind.CubicBezier=>ToolMove.CubicBezier(P(move.Start),P(move.Control1),P(move.Control2),P(move.End),move.MaxError),
            ToolMoveKind.ArcCw or ToolMoveKind.ArcCcw=>ToolMove.Arc(P(move.Start),P(move.End),move.ArcCenter,move.ArcRadius,move.Kind==ToolMoveKind.ArcCw,move.MaxError),
            _=>throw new NotSupportedException("Unknown native geometry")
        };
    }
    internal static Point3D Add(Point3D a,Point3D b)=>new(a.X+b.X,a.Y+b.Y,a.Z+b.Z);
    internal static Point3D Sub(Point3D a,Point3D b)=>new(a.X-b.X,a.Y-b.Y,a.Z-b.Z);
    internal static Point3D Mul(Point3D a,double b)=>new(a.X*b,a.Y*b,a.Z*b);
    internal static double Dot(Point3D a,Point3D b)=>a.X*b.X+a.Y*b.Y+a.Z*b.Z;
    internal static double Norm(Point3D a)=>Math.Sqrt(Dot(a,a));
    internal static double Distance(Point3D a,Point3D b)=>Norm(Sub(a,b));
    internal static Point3D Unit(Point3D a)=>Mul(a,1/Math.Max(1e-30,Norm(a)));
    internal static Point3D Lerp(Point3D a,Point3D b,double t)=>Add(Mul(a,1-t),Mul(b,t));
    internal static double Angle(Point3D a,Point3D b)=>Norm(a)*Norm(b)<1e-25?0:Math.Acos(Math.Clamp(Dot(a,b)/(Norm(a)*Norm(b)),-1,1))*180/Math.PI;
    internal static Point3D At(ToolMove m,double t)
    {
        if(m.Kind==ToolMoveKind.Line)return Lerp(m.Start,m.End,t);
        double s=1-t;
        return Add(Add(Mul(m.Start,s*s*s),Mul(m.Control1,3*s*s*t)),Add(Mul(m.Control2,3*s*t*t),Mul(m.End,t*t*t)));
    }
    internal static Point3D Derivative(ToolMove m,double t)
    {
        if(m.Kind==ToolMoveKind.Line)return Sub(m.End,m.Start);
        return Mul(Add(Add(Mul(Sub(m.Control1,m.Start),(1-t)*(1-t)),Mul(Sub(m.Control2,m.Control1),2*(1-t)*t)),Mul(Sub(m.End,m.Control2),t*t)),3);
    }
    internal static Point3D Second(ToolMove m,double t)
    {
        var a=Add(Sub(m.Start,Mul(m.Control1,2)),m.Control2);
        var b=Add(Sub(m.Control1,Mul(m.Control2,2)),m.End);
        return Mul(Lerp(a,b,t),6);
    }
    internal static ToolMove Reverse(ToolMove m)=>m.Kind switch
    {
        ToolMoveKind.CubicBezier=>ToolMove.CubicBezier(m.End,m.Control2,m.Control1,m.Start,m.MaxError),
        ToolMoveKind.ArcCw or ToolMoveKind.ArcCcw=>ToolMove.Arc(m.End,m.Start,m.ArcCenter,m.ArcRadius,m.Kind==ToolMoveKind.ArcCcw,m.MaxError),
        _=>ToolMove.Line(m.End,m.Start,m.MaxError)
    };
    internal static (ToolMove Left,ToolMove Right) Split(ToolMove m)
    {
        var a=Lerp(m.Start,m.Control1,.5);var b=Lerp(m.Control1,m.Control2,.5);var c=Lerp(m.Control2,m.End,.5);
        var d=Lerp(a,b,.5);var e=Lerp(b,c,.5);var f=Lerp(d,e,.5);
        return(ToolMove.CubicBezier(m.Start,a,d,f,m.MaxError),ToolMove.CubicBezier(f,e,c,m.End,m.MaxError));
    }
    internal static (double Lower,double Upper) Length(ToolMove m,double error=1e-7,int level=0)
    {
        double lower=Distance(m.Start,m.End);
        if(m.Kind==ToolMoveKind.Line)return(lower,lower);
        double upper=Distance(m.Start,m.Control1)+Distance(m.Control1,m.Control2)+Distance(m.Control2,m.End);
        if(upper-lower<=error||level>=20)return(lower,upper);
        var halves=Split(m);var a=Length(halves.Left,error/2,level+1);var b=Length(halves.Right,error/2,level+1);
        return(a.Lower+b.Lower,a.Upper+b.Upper);
    }
    internal static double LineError(Point3D p,Point3D a,Point3D b)
    {var v=Sub(b,a);double t=Math.Clamp(Dot(Sub(p,a),v)/Math.Max(1e-30,Dot(v,v)),0,1);return Distance(p,Add(a,Mul(v,t)));}
    internal static double[] Parameters(IReadOnlyList<Point3D> points,int first,int last)
    {
        var t=new double[last-first+1];for(int i=1;i<t.Length;i++)t[i]=t[i-1]+Distance(points[first+i-1],points[first+i]);
        double length=t[^1];if(length>0)for(int i=1;i<t.Length;i++)t[i]/=length;
        return t;
    }
    internal static double ContinuousBound(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,double[] ts,double limit=.3)
    {
        if(move.Kind==ToolMoveKind.Line)
            return Enumerable.Range(first,last-first+1).Max(i=>LineError(source[i],move.Start,move.End));
        double prior=Distance(At(move,0),source[first]),maximum=prior;
        for(int i=1;i<ts.Length;i++)
        {
            double current=Distance(At(move,ts[i]),source[first+i]);
            if(current>limit)return current;
            double acceleration=Math.Max(Norm(Second(move,ts[i-1])),Norm(Second(move,ts[i])));
            double Refine(double lo,double hi,Point3D a,Point3D b,double ea,double eb,int depth)
            {
                double bound=Math.Max(ea,eb)+Math.Max(Norm(Second(move,lo)),Norm(Second(move,hi)))*Math.Pow(hi-lo,2)/8;
                if(bound<=limit||depth>=12)return bound;
                double mid=(lo+hi)/2;var p=Lerp(a,b,.5);double error=Distance(At(move,mid),p);
                if(error>limit)return error;
                return Math.Max(Refine(lo,mid,a,p,ea,error,depth+1),Refine(mid,hi,p,b,error,eb,depth+1));
            }
            double enclosure=Refine(ts[i-1],ts[i],source[first+i-1],source[first+i],prior,current,0);
            maximum=Math.Max(maximum,enclosure);if(maximum>limit)return maximum;prior=current;
        }
        return maximum;
    }
    internal static IEnumerable<double> Roots(double a,double b,double c)
    {
        if(Math.Abs(a)<1e-20){if(Math.Abs(b)>1e-20)yield return -c/b;yield break;}
        double d=b*b-4*a*c;if(d<0)yield break;double q=Math.Sqrt(d);yield return(-b-q)/(2*a);yield return(-b+q)/(2*a);
    }
    internal static (double Min,double Max) ZRange(ToolMove m)
    {
        if(m.Kind==ToolMoveKind.Line)return(Math.Min(m.Start.Z,m.End.Z),Math.Max(m.Start.Z,m.End.Z));
        double a=-m.Start.Z+3*m.Control1.Z-3*m.Control2.Z+m.End.Z,b=3*m.Start.Z-6*m.Control1.Z+3*m.Control2.Z,c=-3*m.Start.Z+3*m.Control1.Z;
        var zs=new List<double>{m.Start.Z,m.End.Z};foreach(double t in Roots(3*a,2*b,c))if(t>0&&t<1)zs.Add(At(m,t).Z);
        return(zs.Min(),zs.Max());
    }

}
