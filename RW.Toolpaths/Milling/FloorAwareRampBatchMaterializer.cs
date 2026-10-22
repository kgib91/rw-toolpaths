using RW.Toolpaths;

namespace RW.Toolpaths.Milling.Artifacts;

/// <summary>Candidate-only batches under caller-owned ORIGINAL geometry guards.
/// Reuses the owner's immutable floor/domain/profile state; final stock and whole-target proof remain the caller's responsibility.</summary>
public static class FloorAwareRampBatchMaterializer
{
    public sealed record Owners(
        Func<Point3D,Point3D> ActualCommandPoint,
        Func<IReadOnlyList<TaggedToolpath>,List<TaggedToolpath>> Protect,
        Func<IReadOnlyList<TaggedToolpath>,List<TaggedToolpath>> ActualCommandPaths,
        Func<IReadOnlyList<TaggedToolpath>,bool> ProveActualFloor,
        Func<TaggedToolpath,bool> ProveActualWholeWalls,
        Func<IReadOnlyList<TaggedToolpath>,double> MaximumActualGrade);
    public sealed record Admission(int RequestOrdinal,int CandidateOrdinal,double Factor,double Length,
        bool EndpointPreserved,bool SurfaceEntry,bool Grade,bool Monotone,bool Schedule,bool Floor,bool WholeWalls,
        int CommandPoints,double MaximumGrade);
    public sealed record Result(IReadOnlyList<TaggedToolpath> ActualRampPaths,IReadOnlyList<int> RequestOrdinals,
        IReadOnlyList<Admission> Admissions,int ProtectCalls,int StabilizationCalls,string Reason);

    public static Result Materialize(IReadOnlyList<FloorAwareSurfaceRampNominator.Request> requests,
        Owners owners,CancellationToken token,Action<string>? log=null)
    {
        ArgumentNullException.ThrowIfNull(requests);ArgumentNullException.ThrowIfNull(owners);token.ThrowIfCancellationRequested();
        if(requests.Count>1024)throw new ArgumentOutOfRangeException(nameof(requests));
        var nominees=new FloorAwareSurfaceRampNominator.Candidate[requests.Count][];
        for(int i=0;i<requests.Count;i++)
        {
            token.ThrowIfCancellationRequested();FloorAwareSurfaceRampNominator.TryNominate(requests[i],owners.ActualCommandPoint,token,out nominees[i],out string reason);
            if(nominees[i].Length==0)log?.Invoke($"stage=floor-aware-ramp event=nomination-refusal request={i} reason={reason} negativeCredit=False");
        }
        var accepted=new TaggedToolpath?[requests.Count];var admissions=new List<Admission>();int protects=0,stabilizations=0;
        for(int wave=0;wave<4;wave++)
        {
            token.ThrowIfCancellationRequested();var requestOrdinals=new List<int>();var batch=new List<TaggedToolpath>();
            for(int i=0;i<requests.Count;i++)if(accepted[i] is null&&wave<nominees[i].Length){requestOrdinals.Add(i);batch.Add(nominees[i][wave].Path);}
            if(batch.Count==0)continue;
            log?.Invoke($"stage=floor-aware-ramp event=protect-batch wave={wave} requests={batch.Count} owner=caller-original-floor-and-wall-pair");
            var commands=owners.ActualCommandPaths(owners.Protect(batch));protects++;RequireRows(commands,batch.Count);
            for(int pass=0;pass<3;pass++)
            {
                token.ThrowIfCancellationRequested();var next=owners.ActualCommandPaths(owners.Protect(commands));protects++;stabilizations++;RequireRows(next,batch.Count);
                bool stable=SamePoints(commands,next);commands=next;if(stable)break;
            }
            token.ThrowIfCancellationRequested();bool floor=owners.ProveActualFloor(commands);
            for(int row=0;row<commands.Count;row++)
            {
                token.ThrowIfCancellationRequested();int ordinal=requestOrdinals[row];var request=requests[ordinal];var candidate=nominees[ordinal][wave];var path=commands[row];
                bool finite=path.Points.Count is >=2 and <=4096&&path.Points.All(p=>double.IsFinite(p.X)&&double.IsFinite(p.Y)&&double.IsFinite(p.Z));
                bool endpoint=finite&&path.Points[^1]==request.ActualEnd,surface=finite&&path.Points[0].Z==0;
                double maximum=owners.MaximumActualGrade(new[]{path});bool grade=maximum<=request.Grade+1e-12;
                bool monotone=finite&&Enumerable.Range(1,path.Points.Count-1).All(i=>path.Points[i].Z<=path.Points[i-1].Z);
                bool schedule=finite&&path.Points.All(p=>-p.Z>=0&&-p.Z<=request.MaximumDepth&&-p.Z<=request.DepthPerPass);
                bool walls=finite&&owners.ProveActualWholeWalls(path);
                admissions.Add(new(ordinal,wave,candidate.Factor,candidate.Length,endpoint,surface,grade,monotone,schedule,floor,walls,path.Points.Count,maximum));
                if(endpoint&&surface&&grade&&monotone&&schedule&&floor&&walls)
                    accepted[ordinal]=new(path.Points,Array.AsReadOnly(new[]{new ToolpathSpan(0,path.Points.Count-1,ToolpathSpanKind.Ramp)}),path.RegionIndex,path.Category,path.DepthPassIndex,path.VisitIdentity,detectCurves:false);
            }
        }
        var output=new List<TaggedToolpath>();var ordinals=new List<int>();for(int i=0;i<accepted.Length;i++)if(accepted[i] is not null){ordinals.Add(i);output.Add(accepted[i]!.Value);}
        return new(output.AsReadOnly(),ordinals.AsReadOnly(),admissions.AsReadOnly(),protects,stabilizations,
            output.Count==requests.Count?"all-requested-ramp-commands-admitted":"bounded-candidates-incomplete-no-negative-credit");
    }
    static void RequireRows(IReadOnlyList<TaggedToolpath> paths,int expected)
    {if(paths.Count!=expected)throw new InvalidOperationException("Original protection changed request path ordinals.");}
    static bool SamePoints(IReadOnlyList<TaggedToolpath> a,IReadOnlyList<TaggedToolpath> b)
        =>a.Count==b.Count&&Enumerable.Range(0,a.Count).All(i=>a[i].Points.SequenceEqual(b[i].Points));
}
