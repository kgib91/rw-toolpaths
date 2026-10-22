using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling.Artifacts;

/// <summary>One generation's original commanded medial supports provide bounded direction NOM hints.
/// The literal floor/wall/quantization delegates remain their original sequential owners; this object grants no cutting or negative authority.</summary>
public sealed class FloorAwareCorrectionPoseNominator
{
    public sealed record Limits(int MaximumTargets=128,int NearestOriginalSupports=8,
        int SitesPerDirection=16,int MaximumRequests=512,int MaximumCandidatesPerTarget=4);
    public sealed record Candidate(int RequestOrdinal,int TargetOrdinal,PointD OriginalResidualTarget,
        Point3D OriginalCommandedMedialSupport,int DirectionOrdinal,int Sample,double ActualPhysicalGap,
        double OriginalSupportPhysicalGap,double OriginalFloorCap,double OriginalBothWallCap,int OriginalSupportOrdinal);
    public sealed record Result(IReadOnlyList<FloorAwareSurfaceRampNominator.Request> Requests,
        IReadOnlyList<Candidate> Candidates,int Targets,int OriginalDirections,int ActualSites,
        int OriginalSupportPoints,string Status,string OriginalSupportIndexStatistics);
    readonly WeightedOriginalSupportIndex _originalSupports;
    readonly WeightedOriginalSupportIndex.QueryDiagnostics _queryDiagnostics=new();
    readonly Func<Point3D,Point3D> _command;
    readonly Func<Point3D,double> _floorCap,_wallCap;
    readonly Func<double,double> _quantize;
    readonly Func<double,double> _physicalRadius;
    readonly double _desired;
    readonly double _grade,_maximumDepth,_depthPerPass;
    readonly int _region;
    readonly int? _pass;

    public FloorAwareCorrectionPoseNominator(IReadOnlyList<Point3D> originalCommandedMedialSupports,
        Func<Point3D,Point3D> originalCommandPoint,Func<Point3D,double> originalCappedFloorDepth,
        Func<Point3D,double> originalBothWallAllowedDepth,Func<double,double> originalQuantizeDepth,
        Func<double,double> originalPhysicalRadius,double originalNominationGap,
        double grade,double maximumDepth,double depthPerPass,int region,int? depthPass,
        CancellationToken token)
    {
        ArgumentNullException.ThrowIfNull(originalCommandedMedialSupports);ArgumentNullException.ThrowIfNull(originalCommandPoint);
        ArgumentNullException.ThrowIfNull(originalCappedFloorDepth);ArgumentNullException.ThrowIfNull(originalBothWallAllowedDepth);ArgumentNullException.ThrowIfNull(originalQuantizeDepth);
        ArgumentNullException.ThrowIfNull(originalPhysicalRadius);
        if(!double.IsFinite(originalNominationGap)||originalNominationGap<0||originalNominationGap>100)throw new ArgumentOutOfRangeException(nameof(originalNominationGap));
        _command=originalCommandPoint;_floorCap=originalCappedFloorDepth;_wallCap=originalBothWallAllowedDepth;_quantize=originalQuantizeDepth;
        _physicalRadius=originalPhysicalRadius;_desired=originalNominationGap;
        _grade=grade;_maximumDepth=maximumDepth;_depthPerPass=depthPerPass;_region=region;_pass=depthPass;
        if(originalCommandedMedialSupports.Count>4_000_000)throw new ArgumentOutOfRangeException(nameof(originalCommandedMedialSupports));
        for(int i=0;i<originalCommandedMedialSupports.Count;i++)
        {
            if((i&1023)==0)token.ThrowIfCancellationRequested();var p=originalCommandedMedialSupports[i];
            if(!double.IsFinite(p.X)||!double.IsFinite(p.Y)||!double.IsFinite(p.Z)||Math.Abs(p.X)>10000||Math.Abs(p.Y)>10000)
                throw new ArgumentOutOfRangeException(nameof(originalCommandedMedialSupports),"An unsupported index cannot replace bounded original-direction lookup.");
        }
        token.ThrowIfCancellationRequested();_originalSupports=new WeightedOriginalSupportIndex(originalCommandedMedialSupports,originalPhysicalRadius,token,_queryDiagnostics);
    }
    public Result Nominate(IEnumerable<PointD> originalResidualTargets,CancellationToken token,
        Limits? limits=null,Action<string>? log=null)
    {
        ArgumentNullException.ThrowIfNull(originalResidualTargets);limits??=new Limits();
        if(limits.MaximumTargets is <1 or >1024||limits.NearestOriginalSupports is <1 or >8
            ||limits.SitesPerDirection is <1 or >16||limits.MaximumRequests is <1 or >1024
            ||limits.MaximumCandidatesPerTarget is <1 or >4)throw new ArgumentOutOfRangeException(nameof(limits));
        token.ThrowIfCancellationRequested();var requests=new List<FloorAwareSurfaceRampNominator.Request>();var commandKeys=new HashSet<(long,long,long)>();
        var candidates=new List<Candidate>();
        int targets=0,directions=0,sites=0;long previousBoundedQueries=_queryDiagnostics.BoundedQueries;
        foreach(var target in originalResidualTargets)
        {
            token.ThrowIfCancellationRequested();if(targets==limits.MaximumTargets||requests.Count==limits.MaximumRequests)break;targets++;
            var scored=new List<(FloorAwareSurfaceRampNominator.Request Request,WeightedOriginalSupportIndex.Hit Support,int Direction,int Sample,double Gap,double FloorCap,double WallCap)>();
            int direction=0;
            foreach(var supportHit in _originalSupports.Nearest(target.x,target.y,limits.NearestOriginalSupports,token,_queryDiagnostics))
            {
                var support=supportHit.Point;
                token.ThrowIfCancellationRequested();directions++;direction++;
                for(int sample=1;sample<=limits.SitesPerDirection;sample++)
                {
                    token.ThrowIfCancellationRequested();if(requests.Count==limits.MaximumRequests)break;
                    double fraction=sample/(double)limits.SitesPerDirection;
                    var xy=_command(new(target.x+(support.X-target.x)*fraction,target.y+(support.Y-target.y)*fraction,0));sites++;
                    var requested=new Point3D(xy.X,xy.Y,-Math.Min(_maximumDepth,_depthPerPass));
                    double floorCap=_floorCap(requested),wallCap=_wallCap(requested);
                    double cap=Math.Min(floorCap,wallCap);
                    double depth=_quantize(Math.Min(cap,Math.Min(_maximumDepth,_depthPerPass)));
                    if(!(depth>=.001)||!double.IsFinite(depth))continue;
                    var actual=_command(new(xy.X,xy.Y,-depth));
                    if(-actual.Z>cap||-actual.Z>_maximumDepth||-actual.Z>_depthPerPass)continue;
                    double deltaX=actual.X-target.x,deltaY=actual.Y-target.y;
                    double gap=Math.Sqrt(deltaX*deltaX+deltaY*deltaY)-_physicalRadius(-actual.Z);
                    if(!double.IsFinite(gap)||gap>_desired)continue;
                    double remaining=Math.Sqrt((support.X-actual.X)*(support.X-actual.X)+(support.Y-actual.Y)*(support.Y-actual.Y));
                    if(remaining < -actual.Z/_grade+2*Math.Sqrt(2)*.001+4/32768d)continue;
                    scored.Add((new(actual,new(support.X,support.Y),_grade,_maximumDepth,_depthPerPass,_region,"clearing",_pass),supportHit,direction,sample,gap,floorCap,wallCap));
                }
                if(requests.Count==limits.MaximumRequests)break;
            }
            int selected=0;
            foreach(var choice in scored.OrderBy(p=>p.Gap).ThenBy(p=>p.Direction).ThenBy(p=>p.Sample))
            {
                if(selected==limits.MaximumCandidatesPerTarget||requests.Count==limits.MaximumRequests)break;
                var actual=choice.Request.ActualEnd;var key=(BitConverter.DoubleToInt64Bits(actual.X),BitConverter.DoubleToInt64Bits(actual.Y),BitConverter.DoubleToInt64Bits(actual.Z));
                if(!commandKeys.Add(key))continue;
                candidates.Add(new(requests.Count,targets-1,target,choice.Support.Point,choice.Direction,choice.Sample,choice.Gap,
                    choice.Support.PhysicalGap,choice.FloorCap,choice.WallCap,choice.Support.OriginalOrdinal));
                requests.Add(choice.Request);selected++;
            }
        }
        string status=requests.Count>0?"bounded-original-medial-directions-nominated":"no-bounded-candidate-no-negative-credit";
        if(_queryDiagnostics.BoundedQueries!=previousBoundedQueries)status+="-UNKNOWN-original-support-query-bounded-no-negative-credit";
        log?.Invoke($"stage=floor-aware-pose event=nomination targets={targets} originalDirections={directions} actualSites={sites} requests={requests.Count} originalSupportPoints={_originalSupports.Count} "
            + "directionLookup=original-endpoint-physical-influence-bvh actualFloorAndBothWallCaps=True allIntervalScan=False candidateOnly=True "
            + _queryDiagnostics.Describe());
        return new(requests.AsReadOnly(),candidates.AsReadOnly(),targets,directions,sites,_originalSupports.Count,status,_queryDiagnostics.Describe());
    }
}
