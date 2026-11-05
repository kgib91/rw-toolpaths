using Clipper2Lib;
namespace RW.Toolpaths.Milling;

// Proof-only data, never passed into the physical no-gouge surface or returned
// as executed stock. Each pose translates an actual removed cutter solid by
// at most the operation's selected geometric error allowance.
internal sealed record VCarveRemovalGaugePose(Point3D Pose,VCarveCutterProfile Profile);

public static partial class VCarveNativeCurvePlanning
{
    /// <summary>Continuous directed old-removal deviation at a selected geometry
    /// allowance. This is neither exact-zero loss nor manufacturing readiness.
    /// A false result is inconclusive; the caller must refuse publication.</summary>
    public static bool VerifyRemovalDeviation(IReadOnlyList<Point3D> previousReference,
        IReadOnlyList<ToolMove> newNative,VCarveClearedStock prior,VCarveCutterProfile profile,
        double allowance,CancellationToken token=default,Action<string>? diagnostic=null)
    {
        if(previousReference.Count<2||!double.IsFinite(allowance)||allowance<=0)throw new ArgumentOutOfRangeException(nameof(allowance));
        bool Posted(Point3D p)=>double.IsFinite(p.X+p.Y+p.Z)&&p.X==Math.Round(p.X*1000)/1000d
            &&p.Y==Math.Round(p.Y*1000)/1000d&&p.Z==Math.Round(p.Z*1000)/1000d;
        Point3D ShiftPrior(Point3D p)
        {
            double z=Math.Ceiling((p.Z-allowance)*1000)/1000d;
            while(p.Z-z>allowance)z+=.001;
            return p with{Z=z};
        }
        var columns=prior.Chords.Where(c=>c.ClearedColumnRadius>0&&Posted(c.A)&&Posted(c.B))
            .Select(c=>c with{A=ShiftPrior(c.A),B=ShiftPrior(c.B)}).ToList();
        var sweeps=(prior.ProfileSweeps??[]).Select(s=>new VCarveClearedSweep(s.Profile,
            s.Chords.Where(c=>Posted(c.A)&&Posted(c.B)).Select(c=>c with{A=ShiftPrior(c.A),B=ShiftPrior(c.B)}).ToArray())).ToArray();
        double minX=prior.PartBoundary.Min(p=>p.x),maxX=prior.PartBoundary.Max(p=>p.x),
            minY=prior.PartBoundary.Min(p=>p.y),maxY=prior.PartBoundary.Max(p=>p.y);
        var air=new Point3D((minX+maxX)/2,(minY+maxY)/2,-allowance);
        columns.Add(new(air,air,Math.Sqrt(Math.Pow(maxX-minX,2)+Math.Pow(maxY-minY,2))));
        double reach=2*profile.PhysicalRadiusAtDepth(profile.MaximumDepth);
        double left=previousReference.Min(p=>p.X)-reach,right=previousReference.Max(p=>p.X)+reach,
            bottom=previousReference.Min(p=>p.Y)-reach,top=previousReference.Max(p=>p.Y)+reach;
        var offsets=new List<Point3D>{new(0,0,0)};
        for(int x=-1;x<=1;x++)for(int y=-1;y<=1;y++)for(int z=-1;z<=1;z++)
        {
            double norm=Math.Sqrt(x*x+y*y+z*z);if(norm==0)continue;
            offsets.Add(new(x/norm*allowance*(1-1e-8),y/norm*allowance*(1-1e-8),z/norm*allowance*(1-1e-8)));
        }
        if(offsets.Any(p=>VCarveNativeCurveMath.Norm(p)>allowance))throw new InvalidOperationException("REMOVAL_DEVIATION_OFFSET_INVALID");
        var poses=new List<VCarveRemovalGaugePose>();
        foreach(var move in newNative)
        {
            token.ThrowIfCancellationRequested();
            if(move.Kind is not(ToolMoveKind.Line or ToolMoveKind.CubicBezier))throw new NotSupportedException("Native removal comparison does not support this primitive kind.");
            var controls=move.Kind==ToolMoveKind.Line?new[]{move.Start,move.End}:new[]{move.Start,move.Control1,move.Control2,move.End};
            if(controls.Max(p=>p.X)<left||controls.Min(p=>p.X)>right||controls.Max(p=>p.Y)<bottom||controls.Min(p=>p.Y)>top)continue;
            // Literal unrounded poses only. The finite set undercredits the
            // actual sweep; no connecting chords receive removal credit.
            for(int i=0;i<=64;i++)foreach(var offset in offsets)
                poses.Add(new(VCarveNativeCurveMath.Add(VCarveNativeCurveMath.At(move,i/64d),offset),profile));
        }
        var gauge=new VCarveProjectedStock(columns.ToArray(),Array.Empty<IReadOnlyList<PointD>>(),prior.PartBoundary,
            profile,profile.MaximumDepth,token,sweeps,removalGauge:poses);
        bool Cell(ToolMove move)
        {
            if(move.Kind==ToolMoveKind.Line)return gauge.ProveNativeEnvelope(move.Start,move.End,0);
            var a=VCarveNativeCurveMath.Second(move,0);var b=VCarveNativeCurveMath.Second(move,1);
            double xy=Math.Max(Math.Sqrt(a.X*a.X+a.Y*a.Y),Math.Sqrt(b.X*b.X+b.Y*b.Y))/8,
                z=Math.Max(Math.Abs(a.Z),Math.Abs(b.Z))/8;
            return gauge.ProveNativeEnvelope(move.Start with{Z=move.Start.Z-z},move.End with{Z=move.End.Z-z},xy);
        }
        for(int i=1;i<previousReference.Count;i++)
        {
            token.ThrowIfCancellationRequested();var old=ToolMove.Line(previousReference[i-1],previousReference[i]);
            if(!VCarveNativeIntervalProof.Certified(old,_=>false,Cell))
            {diagnostic?.Invoke($"stage=removal-deviation certified=False allowanceMm={allowance:R} interval={i-1} targetPermissionsEmpty=True exactZeroLoss=False");return false;}
        }
        diagnostic?.Invoke($"stage=removal-deviation certified=True allowanceMm={allowance:R} intervals={previousReference.Count-1} gaugePoses={poses.Count} targetPermissionsEmpty=True noGougeAuthority=False exactZeroLoss=False");
        return true;
    }
}
