namespace RW.Toolpaths.Milling;

/// <summary>Whole affine-source lower plane for the admitted sphere/grade
/// support kernel. A floating nominee chooses a dual vector only; outward
/// arithmetic proves its norm and the lower bound at BOTH original endpoints.</summary>
internal static class RoundedFloorSourceLowerBound
{
    private readonly record struct I(double Lo,double Hi)
    {
        internal static I E(double x)=>new(x,x);
        internal static I Add(I a,I b)=>new(Math.BitDecrement(a.Lo+b.Lo),Math.BitIncrement(a.Hi+b.Hi));
        internal static I Sub(I a,I b)=>new(Math.BitDecrement(a.Lo-b.Hi),Math.BitIncrement(a.Hi-b.Lo));
        internal static I Mul(I a,I b)
        {
            double p=a.Lo*b.Lo,q=a.Lo*b.Hi,r=a.Hi*b.Lo,s=a.Hi*b.Hi;
            return new(Math.BitDecrement(Math.Min(Math.Min(p,q),Math.Min(r,s))),
                Math.BitIncrement(Math.Max(Math.Max(p,q),Math.Max(r,s))));
        }
        internal static I Square(I a)=>new(a.Lo<=0&&a.Hi>=0?0:
            Math.Max(0,Math.BitDecrement(Math.Min(a.Lo*a.Lo,a.Hi*a.Hi))),
            Math.BitIncrement(Math.Max(a.Lo*a.Lo,a.Hi*a.Hi)));
        internal static I Sqrt(I a)=>new(Math.Max(0,Math.BitDecrement(Math.Sqrt(Math.Max(0,a.Lo)))),
            Math.BitIncrement(Math.Sqrt(Math.Max(0,a.Hi))));
        internal static I Divide(I a,I b)=>b.Lo>0?Mul(a,new(Math.BitDecrement(1/b.Hi),Math.BitIncrement(1/b.Lo))):new(double.NaN,double.NaN);
    }

    // In this admitted domain, |XY|<=2^10, distance<2^12, radius>=.1,
    // grade<=4 and radial<=4*R/sqrt(17). The stable spherical denominator is
    // >=R, and its square-root argument >=R^2/17. Propagating the original
    // subtraction/norm/sphere/grade roundings is below 2^-25 mm. Reserve this
    // independently of the outward enclosure below; unsupported inputs retain
    // the original subdivision proof. This is below its 1e-7 near-tip stop.
    internal static readonly double RoundoffReserve=Math.ScaleB(1,-25);

    /// <summary>For a shallow endpoint e and any t on its complete source,
    /// d(t)+G(|q-t|) >= d(e)+G(|q-e|)-max(0,m*length-depthRise).
    /// G is m-Lipschitz. Outward source arithmetic and both original kernel
    /// roundings are paid here; no source point is removed or supplies stock.</summary>
    internal static bool TryEndpointDrop(Point3D a,Point3D b,double grade,out double drop)
    {
        drop=double.PositiveInfinity;
        bool Point(Point3D p)=>double.IsFinite(p.X+p.Y+p.Z)&&Math.Abs(p.X)<=1024&&Math.Abs(p.Y)<=1024&&Math.Abs(p.Z)<=128;
        if(!Point(a)||!Point(b)||!double.IsFinite(grade)||grade<=0||grade>4)return false;
        double dx=Math.BitIncrement(Math.Abs(b.X-a.X)),dy=Math.BitIncrement(Math.Abs(b.Y-a.Y));
        double square=Math.BitIncrement(Math.BitIncrement(dx*dx)+Math.BitIncrement(dy*dy));
        double length=Math.BitIncrement(Math.Sqrt(square));
        double rise=Math.Max(0,Math.BitDecrement(Math.Abs(b.Z-a.Z)));
        double deficit=Math.Max(0,Math.BitIncrement(Math.BitIncrement(grade*length)-rise));
        // At(t) uses three rounded affine operations per coordinate. In the
        // admitted XY/Z box their total depth + grade*XY error is <2^-35 mm.
        drop=Math.BitIncrement(deficit+2*RoundoffReserve+Math.ScaleB(1,-35));
        return double.IsFinite(drop);
    }

    internal static bool TryLower(Point3D a,Point3D b,double lo,double hi,
        double x,double y,double radius,double grade,Point3D nominee,out double lower)
    {
        lower=double.NegativeInfinity;
        bool Point(Point3D p)=>double.IsFinite(p.X+p.Y+p.Z)&&Math.Abs(p.X)<=1024&&Math.Abs(p.Y)<=1024&&Math.Abs(p.Z)<=128;
        if(!Point(a)||!Point(b)||!Point(nominee)||!double.IsFinite(x+y)||Math.Abs(x)>1024||Math.Abs(y)>1024
            ||!double.IsFinite(radius)||radius<.1||radius>100||!double.IsFinite(grade)||grade<=0||grade>4
            ||lo<0||hi>1||!(lo<=hi))return false;
        double sx=x-nominee.X,sy=y-nominee.Y,r=Math.Sqrt(sx*sx+sy*sy);
        double transition=radius*grade/Math.Sqrt(1+grade*grade);
        double factor=r>0?(r<=transition?1/Math.Sqrt(Math.Max(0,radius*radius-r*r)):grade/r):0;
        double vx=sx*factor,vy=sy*factor;
        if(!double.IsFinite(vx+vy))return false;
        I norm=I.Sqrt(I.Add(I.Square(I.E(vx)),I.Square(I.E(vy))));
        for(int i=0;norm.Hi>grade&&i<3;i++)
        {
            double scale=Math.BitDecrement(Math.BitDecrement(grade/norm.Hi));
            vx*=scale;vy*=scale;
            norm=I.Sqrt(I.Add(I.Square(I.E(vx)),I.Square(I.E(vy))));
        }
        if(!(norm.Hi<=grade))return false;
        I square=I.Add(I.Square(I.E(vx)),I.Square(I.E(vy)));
        I conjugate=I.Mul(I.E(radius),I.Divide(square,I.Add(I.Sqrt(I.Add(I.E(1),square)),I.E(1))));
        I Endpoint(double t)
        {
            // The mathematical affine source equals its literal endpoint at
            // t=0 or 1. Avoid enclosing a canceling subtraction/addition there;
            // the independent original-kernel roundoff reserve still applies.
            I At(double start,double end)=>t==0?I.E(start):t==1?I.E(end):
                I.Add(I.E(start),I.Mul(I.Sub(I.E(end),I.E(start)),I.E(t)));
            I px=At(a.X,b.X),py=At(a.Y,b.Y),depth=At(-a.Z,-b.Z);
            return I.Sub(I.Add(depth,I.Add(I.Mul(I.E(vx),I.Sub(I.E(x),px)),
                I.Mul(I.E(vy),I.Sub(I.E(y),py)))),conjugate);
        }
        // Fenchel: G(q)>=v.q-R*(sqrt(1+|v|^2)-1), |v|<=grade.
        // Adding affine source depth leaves an affine function of t, whose
        // minimum on the COMPLETE interval is at an endpoint. Source poses
        // and depths never become independent extrema.
        lower=Math.BitDecrement(Math.Min(Endpoint(lo).Lo,Endpoint(hi).Lo)-RoundoffReserve);
        return double.IsFinite(lower);
    }
}
