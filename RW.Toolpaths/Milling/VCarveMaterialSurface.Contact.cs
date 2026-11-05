namespace RW.Toolpaths.Milling;

internal sealed partial class VCarveMaterialSurface
{
    // Same top-down contact kernel as the established ordinary material fitter.
    // Input/output use stock-relative Z; total operation depth and full cutter
    // support remain owned by this material surface, not declared start depth.
    internal (double Lower,double Upper) ResidualSurfaceDepthBounds(double x,double y)
        =>(projectedStock.ClearedDepth(x,y,0,true),projectedStock.ClearedDepth(x,y,0,false));

    internal Point3D ProjectContactPose(Point3D nominal)=>Fit(nominal);

    // Contact depth includes command-grid rounding, so it can jump while XY
    // advances continuously. Compare geometry in path order, not synchronized
    // heights at an artificial shared parameter. Every retained chord still
    // needs the independent complete native AND posted cutter certificates.
    internal static double ContactRefinementError(Point3D a,Point3D b,IReadOnlyList<Point3D> samples)
    {
        double dx=b.X-a.X,dy=b.Y-a.Y,dz=b.Z-a.Z;
        double lengthSquared=dx*dx+dy*dy+dz*dz,previous=0,errorSquared=0;
        foreach(var p in samples)
        {
            double u=lengthSquared>0?((p.X-a.X)*dx+(p.Y-a.Y)*dy+(p.Z-a.Z)*dz)/lengthSquared:0;
            u=Math.Clamp(u,previous,1);
            double ex=p.X-(a.X+u*dx),ey=p.Y-(a.Y+u*dy),ez=p.Z-(a.Z+u*dz);
            errorSquared=Math.Max(errorSquared,ex*ex+ey*ey+ez*ez);previous=u;
        }
        return Math.Sqrt(errorSquared);
    }

    // A rounded command can cross a grid boundary even when its two nominal
    // source poses are arbitrarily close. Subdividing nominal XY cannot refine
    // that command. Traverse the depth change at the deeper endpoint's XY,
    // then cross at the shallower depth (or reverse that order on descent).
    // This preserves both endpoints and the complete depth excursion. The
    // right-angle join stays within the shorter leg of the original chord;
    // both complete native AND posted sweeps still require independent proof.
    internal static bool TryBoundedContactJoin(Point3D a,Point3D b,double allowance,
        Func<Point3D,Point3D,bool> prove,out Point3D[] join,out double errorBound)
    {
        join=[];errorBound=0;
        if(!double.IsFinite(allowance)||allowance<=0)return false;
        var corner=a.Z<=b.Z?a with{Z=b.Z}:b with{Z=a.Z};
        if(corner==a||corner==b)return false;
        double planar=Math.BitIncrement(Math.BitIncrement(Math.Abs(b.X-a.X))+Math.BitIncrement(Math.Abs(b.Y-a.Y)));
        double axial=Math.BitIncrement(Math.Abs(b.Z-a.Z));
        double bound=Math.Min(planar,axial);
        if(!double.IsFinite(bound)||bound>allowance||!prove(a,corner)||!prove(corner,b))return false;
        join=[a,corner,b];errorBound=bound;return true;
    }

    internal IReadOnlyList<Point3D> ProjectCanonicalContactEdge(Point3D nominalA,Point3D nominalB,
        Point3D contactA,Point3D contactB,double tolerance)
    {
        if(!double.IsFinite(tolerance)||tolerance<=0)throw new ArgumentOutOfRangeException(nameof(tolerance));
        if(!IsLegalChord(contactA,contactA)||!IsLegalChord(contactB,contactB))
            throw new InvalidOperationException("STOCK_CONTACT_ENDPOINT_UNCERTIFIED");
        var output=new List<Point3D>{contactA};
        Point3D Lerp(Point3D a,Point3D b,double t)=>new(a.X+(b.X-a.X)*t,a.Y+(b.Y-a.Y)*t,a.Z+(b.Z-a.Z)*t);
        void Append(Point3D sourceA,Point3D sourceB,Point3D a,Point3D b,int level)
        {
            token.ThrowIfCancellationRequested();
            var sourceMid=Lerp(sourceA,sourceB,.5);var mid=ProjectContactPose(sourceMid);
            var samples=new[]{ProjectContactPose(Lerp(sourceA,sourceB,.25)),mid,
                ProjectContactPose(Lerp(sourceA,sourceB,.75))};
            double deviation=ContactRefinementError(a,b,samples);
            // Contact-field samples nominate source refinement. They are not
            // clearance proof: every retained full chord independently earns
            // the existing complete native AND posted stock certificates.
            if(deviation<=tolerance&&IsLegalChord(a,b))
            {if(output[^1]!=b)output.Add(b);return;}
            if(level>=24||output.Count>=200_000||sourceMid==sourceA||sourceMid==sourceB)
            {
                if(deviation<=tolerance&&output.Count<199_998
                    &&TryBoundedContactJoin(a,b,tolerance-deviation,(p,q)=>IsLegalChord(p,q),out var join,out double joinError))
                {
                    foreach(var point in join.Skip(1))if(output[^1]!=point)output.Add(point);
                    diagnostic?.Invoke($"stage=material-contact event=bounded-contact-join level={level} geometricErrorBoundMm={deviation+joinError:R} toleranceMm={tolerance:R} a={a} b={b} nativeAndPostedSweepsCertified=True endpointsPreserved=True depthExcursionPreserved=True");
                    return;
                }
                if(diagnostic is not null)
                {
                    var ca=Command(a);var cb=Command(b);
                    bool native=Legal(a,b),posted=Legal(ca,cb);
                    diagnostic($"stage=material-contact event=refinement-refused level={level} sourceA={sourceA} sourceB={sourceB} a={a} b={b} commandA={ca} commandB={cb} nativeLegal={native} postedLegal={posted} endpointALegal={IsLegalChord(a,a)} endpointBLegal={IsLegalChord(b,b)} geometricErrorMm={deviation:R}");
                    if(!native)projectedStock.DiagnoseProof(a,b,s=>diagnostic("stage=material-contact event=native-proof-refusal "+s));
                    if(!posted)projectedStock.DiagnoseProof(ca,cb,s=>diagnostic("stage=material-contact event=posted-proof-refusal "+s));
                }
                throw new InvalidOperationException($"STOCK_CONTACT_EDGE_UNRESOLVED: level={level} geometricErrorMm={deviation:R} toleranceMm={tolerance:R} a={a} b={b}; no within-tolerance contact join with complete native and posted cutter proof could be established.");
            }
            Append(sourceA,sourceMid,a,mid,level+1);
            Append(sourceMid,sourceB,mid,b,level+1);
        }
        Append(nominalA,nominalB,contactA,contactB,0);return output;
    }
}
