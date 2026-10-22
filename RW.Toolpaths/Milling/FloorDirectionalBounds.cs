namespace RW.Toolpaths.Milling;

// Eight supporting planes of depth + G(distance). A plane minimum over BOTH
// endpoints also bounds every affine source pose. These bounds only prune
// candidate-tree nodes; original source kernels and query tolerances remain.
internal struct FloorDirectionalBounds
{
    private const double Diagonal=.7071067811865475; // exact represented norm < 1
    // With |XY|<=1024, |Z|<=128, R in [.1,100], 0<m<=4 and query |XY|<=1024:
    // Reuse the existing literal sphere/tail roundoff proof in
    // RoundedFloorSourceLowerBound. Plane products/sums and original affine
    // endpoint evaluation add fewer than 32 ULPs at magnitude 2^14; reserve
    // another 2^-30. This subtracts uncertainty from a lower bound; allowed
    // depth, original kernel arithmetic and query tolerance do not increase.
    internal static readonly double ArithmeticReserve=RoundedFloorSourceLowerBound.RoundoffReserve+Math.ScaleB(1,-30);
    private double px,nx,py,ny,pp,nn,pn,np;
    internal bool Eligible{get;private set;}
    internal static FloorDirectionalBounds Empty=>new(){Eligible=true,
        px=double.PositiveInfinity,nx=double.PositiveInfinity,py=double.PositiveInfinity,ny=double.PositiveInfinity,
        pp=double.PositiveInfinity,nn=double.PositiveInfinity,pn=double.PositiveInfinity,np=double.PositiveInfinity};
    internal void Add(Point3D p,double grade)
    {
        if(!double.IsFinite(p.X+p.Y+p.Z)||Math.Abs(p.X)>1024||Math.Abs(p.Y)>1024||Math.Abs(p.Z)>128)
        {Eligible=false;return;}
        double x=grade*p.X,y=grade*p.Y,d=-p.Z,h=Diagonal*(x+y),k=Diagonal*(x-y);
        px=Math.Min(px,d-x);nx=Math.Min(nx,d+x);py=Math.Min(py,d-y);ny=Math.Min(ny,d+y);
        pp=Math.Min(pp,d-h);nn=Math.Min(nn,d+h);pn=Math.Min(pn,d-k);np=Math.Min(np,d+k);
    }
    internal static FloorDirectionalBounds Merge(FloorDirectionalBounds a,FloorDirectionalBounds b)=>new(){
        Eligible=a.Eligible&&b.Eligible,px=Math.Min(a.px,b.px),nx=Math.Min(a.nx,b.nx),
        py=Math.Min(a.py,b.py),ny=Math.Min(a.ny,b.ny),pp=Math.Min(a.pp,b.pp),nn=Math.Min(a.nn,b.nn),
        pn=Math.Min(a.pn,b.pn),np=Math.Min(a.np,b.np)};
    internal readonly record struct Query(double X,double Y,double Sum,double Difference,double SupportUpper);
    internal static bool TryPrepare(double x,double y,double grade,double radius,out Query query)
    {
        query=default;
        if(!double.IsFinite(x+y+grade+radius)||Math.Abs(x)>1024||Math.Abs(y)>1024
            ||grade<=0||grade>4||radius<.1||radius>100)return false;
        double gx=grade*x,gy=grade*y;
        // Outward upper bound on the exact sphere support R*(sqrt(1+m^2)-1).
        double square=Math.BitIncrement(grade*grade),sum=Math.BitIncrement(1+square);
        double root=Math.BitIncrement(Math.Sqrt(sum));
        double support=Math.BitIncrement(radius*Math.BitIncrement(root-1));
        query=new(gx,gy,Diagonal*(gx+gy),Diagonal*(gx-gy),support);return true;
    }
    internal double Lower(Query q)=>Math.Max(Math.Max(Math.Max(px+q.X,nx-q.X),Math.Max(py+q.Y,ny-q.Y)),
        Math.Max(Math.Max(pp+q.Sum,nn-q.Sum),Math.Max(pn+q.Difference,np-q.Difference)))
        -q.SupportUpper-ArithmeticReserve;
}
