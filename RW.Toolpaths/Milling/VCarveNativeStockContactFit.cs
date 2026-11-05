namespace RW.Toolpaths.Milling;

/// <summary>Ownership of directions in immutable source geometry. Contact
/// projection authors positions/depth envelopes; it does not author an axial
/// derivative by sampling that envelope. Planar medial directions remain owned.</summary>
public enum VCarveNativeSourceContract
{
    PreparedSpatialDirections,
    MedialPlanarWithStockContactDepth
}

[Flags]
internal enum VCarveNativeAxialFreedom { None=0,Start=1,End=2,Both=Start|End }

internal static class VCarveNativeStockContactFit
{
    internal static Point3D Planar(Point3D p)=>VCarveNativeCurveMath.Unit(new(p.X,p.Y,0));
    internal static VCarveNativeFit? Find(Point3D[] p,Point3D startDirection,Point3D endDirection,
        VCarveNativeContainment guard,VCarveCutterProfile profile,VCarveNativeFitSettings settings,
        CancellationToken token,out string outcome,VCarveNativeAxialFreedom freedom=VCarveNativeAxialFreedom.Both)
    {
        outcome="stock-contact-branch-unresolved";
        if(!guard.StockAware)throw new InvalidOperationException("STOCK_CONTACT_DIRECTION_REQUIRES_ACTUAL_STOCK");
        if(p.Length<3)return null;
        var u=Planar(startDirection);var v=Planar(endDirection);
        if(VCarveNativeCurveMath.Norm(u)==0||VCarveNativeCurveMath.Norm(v)==0)return null;
        int last=p.Length-1;var cumulative=new double[p.Length];double sourceLength=0;
        for(int i=1;i<p.Length;i++)
        {cumulative[i]=cumulative[i-1]+Math.Sqrt(Math.Pow(p[i].X-p[i-1].X,2)+Math.Pow(p[i].Y-p[i-1].Y,2));sourceLength+=VCarveNativeCurveMath.Distance(p[i-1],p[i]);}
        if(cumulative[^1]<=0||sourceLength<settings.MinimumPrimitiveLengthMm)return null;
        double aa=0,ab=0,bb=0,ar=0,br=0,zr1=0,zr2=0;
        for(int i=1;i<last;i++)
        {
            double t=cumulative[i]/cumulative[^1],s=1-t,k1=3*s*s*t,k2=3*s*t*t;
            var basis=VCarveNativeCurveMath.Add(VCarveNativeCurveMath.Mul(p[0],s*s*s+k1),VCarveNativeCurveMath.Mul(p[last],k2+t*t*t));
            var rhs=VCarveNativeCurveMath.Sub(p[i],basis);
            aa+=k1*k1;ab+=k1*k2;bb+=k2*k2;
            ar+=k1*VCarveNativeCurveMath.Dot(u,rhs);br-=k2*VCarveNativeCurveMath.Dot(v,rhs);
            double z=p[i].Z-s*s*s*p[0].Z-t*t*t*p[last].Z;zr1+=k1*z;zr2+=k2*z;
        }
        double xyab=-ab*VCarveNativeCurveMath.Dot(u,v),xydet=aa*bb-xyab*xyab,zdet=aa*bb-ab*ab;
        if(xydet<=1e-20||zdet<=1e-20)return null;
        double alpha=(ar*bb-br*xyab)/xydet,beta=(br*aa-ar*xyab)/xydet;
        if(alpha<=0||beta<=0)return null;
        double z1=(zr1*bb-zr2*ab)/zdet,z2=(zr2*aa-zr1*ab)/zdet;
        double low=p.Min(q=>q.Z),high=p.Max(q=>q.Z);
        var planarProgress=VCarveNativePlanarProgress.Create(p,0,last);
        // Dimensionless candidate inventory, not extra fit/depth allowances.
        bool freeStart=(freedom&VCarveNativeAxialFreedom.Start)!=0,freeEnd=(freedom&VCarveNativeAxialFreedom.End)!=0;
        double[] lifts=[0,1d/30,.1,.2,1d/3,2d/3,1];
        foreach(double sa in new[]{.75,1d,1.25})foreach(double sb in new[]{.75,1d,1.25})
        foreach(double za in freeStart?lifts:new[]{0d})foreach(double zb in freeEnd?lifts:new[]{0d})
        {
            token.ThrowIfCancellationRequested();
            double handleA=alpha*sa,handleB=beta*sb;
            if(!planarProgress.TrySelectHandles(p[0],p[last],u,v,ref handleA,ref handleB))continue;
            var a=VCarveNativeCurveMath.Add(p[0],VCarveNativeCurveMath.Mul(u,handleA)) with{Z=freeStart?z1+za*settings.FitErrorMm:p[0].Z+handleA*startDirection.Z/Math.Sqrt(startDirection.X*startDirection.X+startDirection.Y*startDirection.Y)};
            var b=VCarveNativeCurveMath.Sub(p[last],VCarveNativeCurveMath.Mul(v,handleB)) with{Z=freeEnd?z2+zb*settings.FitErrorMm:p[last].Z-handleB*endDirection.Z/Math.Sqrt(endDirection.X*endDirection.X+endDirection.Y*endDirection.Y)};
            var move=ToolMove.CubicBezier(p[0],a,b,p[last],0);var range=VCarveNativeCurveMath.ZRange(move);
            if(range.Min<low-1e-9||range.Max>high+1e-9)continue;
            var geometry=VCarveNativeGeometricProof.Prove(move,p,0,last,settings.FitErrorMm);
            if(!geometry.Certified)continue;
            // Samples reject only. The final full-curve stock proof is mandatory.
            if(!Enumerable.Range(0,17).All(i=>{var q=VCarveNativeCurveMath.At(move,i/16d);return guard.Legal(q,q);}))continue;
            var length=VCarveNativeCurveMath.Length(move);if(length.Lower<settings.MinimumPrimitiveLengthMm)continue;
            if(!VCarveNativeCutterProof.Certified(move,guard,profile,out _,9,2048))continue;
            double Angle(Point3D x,Point3D y)=>Math.Acos(Math.Clamp(VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Unit(x),VCarveNativeCurveMath.Unit(y)),-1,1))*180/Math.PI;
            outcome="stock-contact-planar-branch-continuously-certified";
            return new(move with{MaxError=geometry.ErrorBound},0,last,sourceLength,length.Lower,length.Upper,
                Angle(startDirection,VCarveNativeCurveMath.Derivative(move,0)),Angle(endDirection,VCarveNativeCurveMath.Derivative(move,1))) {StockContactAxialEnvelope=true};
        }
        return null;
    }
}
