using Clipper2Lib;
using System.Diagnostics;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Shared source-level detours. Existing XYZ intersections alone join
/// strokes; no proximity chord, future-stock claim or residual cut is invented.</summary>
internal static partial class ToolpathSourceTraversal
{
    private readonly record struct Segment(int Owner,int End);
    private const int MaximumSegments=2_000_000,MaximumCandidates=2_000_000;

    // Already-entered generators share the same detour policy. Only an entirely
    // flat Cut suffix after a Ramp-only entry is eligible: tabs, arcs, links and
    // variable-height relief retain their original execution order and entries.
    internal static List<TaggedToolpath> EnteredDetours(IReadOnlyList<TaggedToolpath> source,
        CancellationToken token,Action<string>? log=null,Func<Point3D,Point3D,bool>? validate=null,
        MillingMotionSettings? motion=null,double surface=0)
    {
        var result=new List<TaggedToolpath>(source.Count);
        var motionCost=motion is null?null:new ToolpathMotionCost(motion,surface);
        int refused=0;double saved=0;
        for(int first=0;first<source.Count;)
        {
            token.ThrowIfCancellationRequested();
            if(!CutStart(source[first],out int cut)){result.Add(source[first++]);continue;}
            int end=first+1;var root=source[first];double z=root.Points[cut].Z;
            while(end<source.Count&&source[end].RegionIndex==root.RegionIndex
                &&source[end].Category==root.Category&&source[end].DepthPassIndex==root.DepthPassIndex
                &&CutStart(source[end],out int next)&&source[end].Points[next].Z==z)end++;
            var prepared=new List<TaggedToolpath>();
            for(int i=first;i<end;i++)
            {
                CutStart(source[i],out int start);var p=source[i];
                prepared.Add(new(p.Points.Skip(start).Select(v=>v with{Z=-1}).ToArray(),Array.Empty<ToolpathSpan>(),p.RegionIndex,
                    p.Category,p.DepthPassIndex,i,detectCurves:false));
            }
            bool Validate(Point3D a,Point3D b)=>validate is null||validate(a with{Z=z},b with{Z=z});
            var detours=Detours(prepared,token,log,validate is null?null:Validate);
            if(validate is not null&&detours.Any(p=>Enumerable.Range(1,p.Points.Count-1)
                .Any(i=>!Validate(p.Points[i-1],p.Points[i]))))
            {
                log?.Invoke("stage=source-detours event=refused reason=execution-guard sourceEntriesRetained=True");
                result.AddRange(source.Skip(first).Take(end-first));first=end;continue;
            }
            int firstCandidate=result.Count;
            foreach(var detour in detours)
            {
                var original=source[detour.VisitIdentity!.Value];CutStart(original,out int start);
                if(!detour.ContinuationCircuit){result.Add(original);continue;}
                if((detour.Points[0] with{Z=z})!=original.Points[start])
                    throw new InvalidOperationException("A source detour must retain its carrier entry.");
                var points=original.Points.Take(start).Concat(detour.Points.Select(v=>v with{Z=z})).ToArray();
                var spans=original.Spans.Where(s=>s.EndIndex<=start).ToList();
                spans.Add(new(start,points.Length-1,ToolpathSpanKind.Cut));
                result.Add(new(points,spans,original.RegionIndex,original.Category,original.DepthPassIndex,
                    original.VisitIdentity,detectCurves:false));
            }
            if(motionCost is not null)
            {
                Point3D? incoming=firstCandidate>0?result[firstCandidate-1].Points[^1]:null;
                Point3D? following=end<source.Count&&source[end].Points.Count>0?source[end].Points[0]:null;
                var original=source.Skip(first).Take(end-first).ToArray();
                var proposed=result.Skip(firstCandidate).ToArray();
                double before=motionCost.SequenceSeconds(incoming,original,following);
                double after=motionCost.SequenceSeconds(incoming,proposed,following);
                if(after>before+1e-9)
                {
                    result.RemoveRange(firstCandidate,result.Count-firstCandidate);result.AddRange(original);
                    if(++refused<=8)log?.Invoke($"stage=source-detours event=entry-cost-refused originalSec={before:R} detourSec={after:R} completeEntryCutAndExit=True");
                }
                else saved+=before-after;
            }
            first=end;
        }
        log?.Invoke($"stage=source-detours event=cost-complete machineContext={motionCost is not null} costRefusals={refused} selectedSavingSec={saved:R} completeEntryCutAndExit=True policyOwner=RW.Toolpaths");
        return result;

        static bool CutStart(TaggedToolpath p,out int start)
        {
            start=0;if(p.RequiresSourcePreservation||p.Points.Count<2)return false;
            bool cutting=false;
            foreach(var span in p.Spans)
            {
                if(span.Kind==ToolpathSpanKind.Cut){if(!cutting)start=span.StartIndex;cutting=true;}
                else if(cutting||span.Kind!=ToolpathSpanKind.Ramp)return false;
            }
            double z=p.Points[start].Z;
            return p.Points.Skip(start).All(v=>v.Z==z);
        }
    }

    internal static List<TaggedToolpath> Components(IReadOnlyList<TaggedToolpath> source,
        Func<Point3D,Point3D> firstLayer,Func<Point3D,Point3D,bool> allows,double proximity,
        CancellationToken token,Action<string>? log=null,Point3D? start=null,
        Func<Point3D,Point3D,bool>? candidateFilter=null,
        IReadOnlyList<ToolpathContinuationComponents.Edge>? nominatedPortals=null)
        =>ToolpathContinuationComponents.Assign(source,firstLayer,allows,proximity,token,log,start,candidateFilter,
            nominatedPortals:nominatedPortals)
            .OrderBy(p=>p.ContinuationComponentIndex).ThenBy(p=>p.ContinuationSourceOrdinal).ToList();

    // Offset corners are diagonally separated even when their straight sides
    // are one stepover apart. Nominate literal points on nearby source spans,
    // rather than mistaking corner-to-corner distance for the nearest corridor.
    internal static List<TaggedToolpath> RingPortals(IReadOnlyList<TaggedToolpath> source,double reach,
        Func<Point3D,Point3D> firstLayer,Func<Point3D,Point3D,bool> guard,
        CancellationToken token,Action<string>? log=null,
        Action<IReadOnlyList<ToolpathContinuationComponents.Edge>>? capturePortals=null,
        Func<VCarveSmoothing.ContinuationProofWorker?>? workerFactory=null,
        Func<Point3D,Point3D,bool>? privateWorkerPrefix=null,int? maximumWorkers=null)
    {
        if(maximumWorkers is <=0)throw new ArgumentOutOfRangeException(nameof(maximumWorkers));
        long started=Stopwatch.GetTimestamp();
        bool Eligible(int i)=>source[i].Category=="clearing"&&!source[i].RequiresSourcePreservation
            &&!source[i].ContinuationCircuit&&source[i].Points.Count>2
            &&source[i].Points[0]==source[i].Points[^1]&&source[i].Spans.All(s=>s.Kind==ToolpathSpanKind.Cut);
        var segments=new List<Segment>();var boxes=new Paths64();
        for(int owner=0;owner<source.Count;owner++)if(Eligible(owner))
        for(int end=1;end<source[owner].Points.Count;end++)
        {
            token.ThrowIfCancellationRequested();if(segments.Count>=MaximumSegments)return source.ToList();
            segments.Add(new(owner,end));boxes.Add(new(){XY(source[owner].Points[end-1]),XY(source[owner].Points[end])});
        }
        if(segments.Count==0||!(reach>0))return source.ToList();
        var index=new PolygonOps.BoundarySegmentIndex(boxes);long extent=(long)Math.Ceiling(reach*PathUtils.Scale)+2;
        var matches=new List<int>();var seen=new HashSet<int>();
        var additions=new Dictionary<(int Owner,int End),List<(double T,Point3D P)>>();
        var counts=new int[source.Count];int candidates=0,proofs=0,accepted=0,capped=0;
        var portals=capturePortals is null?null:new List<ToolpathContinuationComponents.Edge>();
        long nominationStarted=Stopwatch.GetTimestamp();
        for(int owner=0;owner<source.Count&&candidates<MaximumCandidates&&proofs<20_000;owner++)if(Eligible(owner))
        {
            var points=source[owner].Points;int stride=Math.Max(1,(points.Count-1+63)/64);
            for(int vertex=0;vertex<points.Count-1&&candidates<MaximumCandidates&&proofs<20_000;vertex+=stride)
            {
                token.ThrowIfCancellationRequested();var p=points[vertex];var q=XY(p);seen.Clear();
                if(!index.QueryBounds(q.X-extent,q.Y-extent,q.X+extent,q.Y+extent,matches,128,token))capped++;
                var nearest=new List<(double Distance,int Segment,double T,Point3D P)>();
                foreach(int ordinal in matches)
                {
                    int row=ordinal/2;if(!seen.Add(row))continue;var s=segments[row];
                    if(s.Owner==owner||counts[s.Owner]>=8||source[s.Owner].RegionIndex!=source[owner].RegionIndex)continue;
                    if(++candidates>MaximumCandidates)break;
                    var a=source[s.Owner].Points[s.End-1];var b=source[s.Owner].Points[s.End];
                    double dx=b.X-a.X,dy=b.Y-a.Y,den=dx*dx+dy*dy;if(den<=0)continue;
                    double t=((p.X-a.X)*dx+(p.Y-a.Y)*dy)/den;if(t<=0||t>=1)continue;
                    var point=new Point3D(a.X+t*dx,a.Y+t*dy,a.Z+t*(b.Z-a.Z));
                    double d=(point.X-p.X)*(point.X-p.X)+(point.Y-p.Y)*(point.Y-p.Y);
                    if(d>reach*reach||d==0)continue;
                    nearest.Add((d,row,t,point));nearest.Sort((a,b)=>a.Distance.CompareTo(b.Distance));
                    if(nearest.Count>4)nearest.RemoveAt(4);
                }
                foreach(var proposal in nearest)
                {
                    var s=segments[proposal.Segment];var key=(s.Owner,s.End);
                    if(!additions.TryGetValue(key,out var list))additions[key]=list=new();
                    if(counts[s.Owner]>=8||list.Any(v=>v.P==proposal.P))continue;
                    if(++proofs>20_000)break;
                    if(!guard(firstLayer(proposal.P),firstLayer(p))
                        ||!guard(source[s.Owner].Points[s.End-1],proposal.P)
                        ||!guard(proposal.P,source[s.Owner].Points[s.End]))continue;
                    list.Add((proposal.T,proposal.P));counts[s.Owner]++;accepted++;
                    portals?.Add(new(new(s.Owner,firstLayer(proposal.P)),new(owner,firstLayer(p))));
                }
            }
        }
        double nominationMs=Stopwatch.GetElapsedTime(nominationStarted).TotalMilliseconds;
        long auditStarted=Stopwatch.GetTimestamp();
        // Counts, additions and source are frozen after the original ordered
        // nomination loop. Final source reconstruction/proof is independent per
        // owner, and requires an explicit private full-continuation capability.
        var changed=Enumerable.Range(0,source.Count).Where(i=>counts[i]>0).ToArray();
        int workerCount=Math.Min(Math.Min(6,GeometryParallelism.WorkerLimit),Math.Min(maximumWorkers??6,changed.Length));
        VCarveSmoothing.ContinuationProofWorker[]? workers=null;
        if(workerFactory is not null&&workerCount>1)
        {
            workers=new VCarveSmoothing.ContinuationProofWorker[workerCount];
            for(int i=0;i<workers.Length;i++)
            {
                token.ThrowIfCancellationRequested();var worker=workerFactory();
                if(worker is null){workers=null;break;}workers[i]=worker;
            }
        }
        bool parallel=workers is not null;var output=source.ToArray();
        var added=new int[source.Count];var refused=new bool[source.Count];var audited=new long[source.Count];
        void Rebuild(int owner,Func<Point3D,Point3D,bool> audit)
        {
            token.ThrowIfCancellationRequested();var original=source[owner];
            var points=new List<Point3D>{original.Points[0]};
            for(int end=1;end<original.Points.Count;end++)
            {
                if((end&1023)==0)token.ThrowIfCancellationRequested();
                if(additions.TryGetValue((owner,end),out var list))foreach(var p in list.OrderBy(p=>p.T))points.Add(p.P);
                points.Add(original.Points[end]);
            }
            // Splitting several times on one chord changes command rounding.
            // Recheck the complete final subdivision, not just each nomination.
            bool legal=true;
            for(int i=1;i<points.Count;i++)
            {
                token.ThrowIfCancellationRequested();audited[owner]++;
                if(!audit(points[i-1],points[i])){legal=false;break;}
            }
            if(!legal)refused[owner]=true;
            else
            {
                added[owner]=points.Count-original.Points.Count;
                output[owner]=new(points,Array.Empty<ToolpathSpan>(),original.RegionIndex,original.Category,
                    original.DepthPassIndex,original.VisitIdentity,detectCurves:false)
                    {ConcentricSource=original.ConcentricSource};
            }
        }
        if(!parallel)
        {
            foreach(int owner in changed)Rebuild(owner,guard);
            workerCount=1;
        }
        else
        {
            var errors=new System.Runtime.ExceptionServices.ExceptionDispatchInfo?[source.Count];int next=-1,failed=0;
            Parallel.For(0,workerCount,new ParallelOptions{MaxDegreeOfParallelism=workerCount},worker=>
            {
                using var owned=GeometryParallelism.EnterWorker();
                while(!token.IsCancellationRequested&&Volatile.Read(ref failed)==0)
                {
                    int index=Interlocked.Increment(ref next);if(index>=changed.Length)break;int owner=changed[index];
                    try{Rebuild(owner,(a,b)=>(privateWorkerPrefix is null||privateWorkerPrefix(a,b))&&workers![worker].Prove(a,b));}
                    catch(Exception error)
                    {errors[owner]=System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(error);Interlocked.Exchange(ref failed,1);}
                }
            });
            // All private guards are idle before parent caches, external logs,
            // capture callbacks or returned source geometry can be published.
            token.ThrowIfCancellationRequested();foreach(var error in errors)error?.Throw();
            foreach(var worker in workers!)worker.Complete(log,"ring-portal-audit");
        }
        token.ThrowIfCancellationRequested();
        var result=output.ToList();int retained=added.Sum();var refusedOwners=Enumerable.Range(0,source.Count).Where(i=>refused[i]).ToHashSet();
        int refusedSources=refusedOwners.Count;double auditMs=Stopwatch.GetElapsedTime(auditStarted).TotalMilliseconds;
        log?.Invoke($"stage=ring-span-portals candidates={candidates} proofs={proofs} accepted={accepted} retained={retained} refusedSources={refusedSources} cappedQueries={capped} "
            +$"nominationMs={nominationMs:F3} finalAuditMs={auditMs:F3} finalAuditChords={audited.Sum()} finalAuditOwners={changed.Length} finalAuditWorkers={workerCount} "
            +$"privateQueryOwners={parallel} allWorkersJoined=True sourcePoints={source.Sum(p=>(long)p.Points.Count)} retainedPoints={result.Sum(p=>(long)p.Points.Count)} "
            +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} "
            +"maximumPortalsPerSource=8 maximumSampledVerticesPerSource=64 finalSubdivisionsRechecked=True newSourceChords=0 nominationOrderAndBudgetsUnchanged=True");
        if(portals is not null)
        {
            var surviving=portals.Where(p=>!refusedOwners.Contains(p.A.Owner)).Distinct().ToArray();
            capturePortals?.Invoke(surviving);
            log?.Invoke($"stage=ring-span-portals event=literal-nominations pairs={surviving.Length} rejectedSplitOwners={refusedOwners.Count} "
                +"coarseSamplingCannotEraseNominations=True componentProofStillRequired=True");
        }
        token.ThrowIfCancellationRequested();
        return result;
    }

    internal static List<TaggedToolpath> Detours(IReadOnlyList<TaggedToolpath> source,
        CancellationToken token,Action<string>? log=null,Func<Point3D,Point3D,bool>? allowsSplit=null,
        double maximumGrade=double.PositiveInfinity,
        Action<int,IReadOnlyList<int>>? attachedMembers=null)
    {
        token.ThrowIfCancellationRequested();
        if(source.Count<2||source.Any(p=>p.Points.Count<2||p.RequiresSourcePreservation||p.Spans.Any(s=>s.Kind!=ToolpathSpanKind.Cut)))return source.ToList();
        var segments=new List<Segment>();var boxes=new Paths64();
        for(int owner=0;owner<source.Count;owner++)
        for(int end=1;end<source[owner].Points.Count;end++)
        {
            if((segments.Count&1023)==0)token.ThrowIfCancellationRequested();
            var a=source[owner].Points[end-1];var b=source[owner].Points[end];
            if(!Finite(a)||!Finite(b)||segments.Count>=MaximumSegments)
            {log?.Invoke("stage=source-detours event=refused reason=index-domain sourcePreserved=True");return source.ToList();}
            segments.Add(new(owner,end));boxes.Add(new Path64{XY(a),XY(b)});
        }
        var index=new PolygonOps.BoundarySegmentIndex(boxes);
        var parent=Enumerable.Range(0,source.Count).ToArray();
        var closed=source.Select(p=>attachedMembers is not null ? p.Category=="clearing"?1:0
            : p.Points.Count>1&&p.Points[0]==p.Points[^1]?1:0).ToArray();
        var splits=new Dictionary<(int,int),List<(double T,Point3D P)>>();
        var matches=new List<int>();var seen=new HashSet<int>();
        long candidates=0;int contacts=0,capped=0,refused=0,crossRowPairsSkipped=0;
        for(int row=0;row<segments.Count&&candidates<MaximumCandidates;row++)
        {
            token.ThrowIfCancellationRequested();var s=segments[row];
            var a=source[s.Owner].Points[s.End-1];var b=source[s.Owner].Points[s.End];
            var qa=XY(a);var qb=XY(b);
            if(!index.QueryBounds(Math.Min(qa.X,qb.X)-2,Math.Min(qa.Y,qb.Y)-2,
                Math.Max(qa.X,qb.X)+2,Math.Max(qa.Y,qb.Y)+2,matches,256,token))capped++;
            seen.Clear();
            foreach(int ordinal in matches)
            {
                int other=ordinal/2;if(other<=row||!seen.Add(other))continue;
                var t=segments[other];
                if(s.Owner==t.Owner||source[s.Owner].RegionIndex!=source[t.Owner].RegionIndex
                    ||source[s.Owner].DepthPassIndex!=source[t.Owner].DepthPassIndex)continue;
                var firstRow=source[s.Owner].ConcentricSource;var secondRow=source[t.Owner].ConcentricSource;
                // A literal intersection does not grant permission to combine
                // two offset rows into an Euler detour and lose their sequence.
                if(firstRow is not null||secondRow is not null)
                    if(firstRow is null||secondRow is null||!ReferenceEquals(firstRow.Plan,secondRow.Plan)
                        ||firstRow.Ring!=secondRow.Ring||firstRow.Wall!=secondRow.Wall){crossRowPairsSkipped++;continue;}
                if(++candidates>MaximumCandidates)break;
                int first=Root(s.Owner),second=Root(t.Owner);
                // An open branch belongs to its first carrier. It cannot reorder
                // two closed contours by turning them into one Euler circuit.
                if(first!=second&&closed[first]+closed[second]>1)continue;
                var c=source[t.Owner].Points[t.End-1];var d=source[t.Owner].Points[t.End];
                if(!Contact(a,b,c,d,out double u,out double v,out var p,attachedMembers is not null))continue;
                if(!SplitAllowed(a,p,b)||!SplitAllowed(c,p,d)){refused++;continue;}
                Add(s,u,p);Add(t,v,p);contacts++;
                if(first!=second){int root=Math.Min(first,second),child=Math.Max(first,second);parent[child]=root;closed[root]+=closed[child];}
            }
        }
        var result=new List<TaggedToolpath>();int merged=0,openReturnsRemoved=0;
        bool IsClosed(int owner)=>source[owner].Points[0]==source[owner].Points[^1];
        foreach(var group in Enumerable.Range(0,source.Count).GroupBy(Root)
            .OrderBy(g=>g.Where(IsClosed).DefaultIfEmpty(g.Min()).Min()))
        {
            var members=group.Order().ToArray();
            if(members.Length==1){result.Add(source[members[0]]);continue;}
            var ringVisits=members.Select(i=>source[i].ConcentricSource).Where(v=>v is not null).Cast<VCarveRingOrder.Visit>().ToArray();
            if(ringVisits.Select(v=>v.Plan).Distinct().Skip(1).Any())
            {
                log?.Invoke("stage=source-detours event=ring-plan-refused reason=unrelated-ring-plans sourcePreserved=True");
                result.AddRange(members.Select(i=>source[i]));continue;
            }
            var ringSource=VCarveRingOrder.Visit.Combine(ringVisits);
            if(attachedMembers is not null&&!members.Any(i=>source[i].Category=="clearing"))
            {result.AddRange(members.Select(i=>source[i]));continue;}
            var prepared=members.Select(owner=>Split(owner)).ToArray();
            if(attachedMembers is not null)
            {
                prepared=prepared.OrderByDescending(p=>p.Category=="clearing").ToArray();
                var branches=ToolpathCircuitBuilder.Build(prepared,(_,_)=>false,.001,token,log,
                    maximumGrade:maximumGrade,preserveSourceOrder:true,literalPortalsOnly:true);
                if(branches.Count!=1||branches[0].Category!="clearing"||branches[0].VisitIdentity is not {} carrier)
                {
                    log?.Invoke($"stage=branch-detours event=refused reason=incomplete-literal-component circuits={branches.Count} sourcePreserved=True");
                    result.AddRange(members.Select(i=>source[i]));continue;
                }
                result.Add(branches[0]);attachedMembers(carrier,members.Where(i=>source[i].VisitIdentity.HasValue)
                    .Select(i=>source[i].VisitIdentity!.Value).ToArray());
                merged+=members.Length-1;continue;
            }
            bool open=closed[Root(members[0])]==0;
            if(open)prepared=prepared.OrderByDescending(p=>Enumerable.Range(1,p.Points.Count-1)
                .Sum(i=>ToolpathContinuationPlanner.Distance(p.Points[i-1],p.Points[i]))).ToArray();
            else prepared=prepared.OrderByDescending(p=>p.Points[0]==p.Points[^1]).ToArray();
            var circuits=ToolpathCircuitBuilder.Build(prepared,(_,_)=>false,.001,token,log,
                maximumGrade:maximumGrade,preserveSourceOrder:true,openCarrierReturn:open);
            foreach(var circuit in circuits)
            {
                var route=circuit;
                // Euler parity closes an open tree by replaying its carrier.
                // Drop that final return only when every required directed cut
                // is already present and the retained exit is the original one.
                var carrier=prepared[0].Points;int count=circuit.Points.Count-carrier.Count+1;
                if(open&&circuits.Count==1&&count>1
                    &&carrier.Select((p,i)=>p==circuit.Points[circuit.Points.Count-1-i]).All(v=>v))
                {
                    var edges=Enumerable.Range(1,count-1).Select(i=>(circuit.Points[i-1],circuit.Points[i])).ToHashSet();
                    if(prepared.All(p=>Enumerable.Range(1,p.Points.Count-1).All(i=>edges.Contains((p.Points[i-1],p.Points[i])))))
                    {
                        route=new(circuit.Points.Take(count).ToArray(),Array.Empty<ToolpathSpan>(),circuit.RegionIndex,
                            circuit.Category,circuit.DepthPassIndex,circuit.VisitIdentity,detectCurves:false)
                            {ContinuationCircuit=true};openReturnsRemoved++;
                    }
                }
                result.Add(route with{ConcentricSource=ringSource});
            }
            merged+=members.Length-1;
        }
        log?.Invoke($"stage=source-detours event=complete inputSources={source.Count} outputSources={result.Count} "
            +$"indexedSegments={segments.Count} candidates={candidates} contacts={contacts} merged={merged} "
            +$"refusedSplits={refused} cappedQueries={capped} budgetReached={candidates>=MaximumCandidates} openCarrierReturnsRemoved={openReturnsRemoved} "
            +$"crossRowPairsSkipped={crossRowPairsSkipped} constructedRowsPreserved=True "
            +"introducedChords=0 sourceLevelStack=True closedCarrierOrderPreserved=True");
        return result;

        int Root(int i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;}
        bool SplitAllowed(Point3D a,Point3D p,Point3D b)
        {
            if(p==a||p==b)return true;
            if(allowsSplit is not null)return allowsSplit(a,p)&&allowsSplit(p,b);
            if(a.Z!=b.Z||p.Z!=a.Z)return false;
            var ca=Posted(a);var cb=Posted(b);var cp=Posted(p);
            return (cb.X-ca.X)*(cp.Y-ca.Y)==(cb.Y-ca.Y)*(cp.X-ca.X);
        }
        void Add(Segment s,double t,Point3D p)
        {
            if(t<=0||t>=1)return;
            if(!splits.TryGetValue((s.Owner,s.End),out var list))splits[(s.Owner,s.End)]=list=new();
            if(!list.Any(e=>e.P==p))list.Add((t,p));
        }
        TaggedToolpath Split(int owner)
        {
            var original=source[owner];var points=new List<Point3D>{original.Points[0]};
            for(int end=1;end<original.Points.Count;end++)
            {
                if(splits.TryGetValue((owner,end),out var list))foreach(var split in list.OrderBy(p=>p.T))
                    if(points[^1]!=split.P)points.Add(split.P);
                if(points[^1]!=original.Points[end])points.Add(original.Points[end]);
            }
            return new(points,Array.Empty<ToolpathSpan>(),original.RegionIndex,original.Category,
                original.DepthPassIndex,original.VisitIdentity,detectCurves:false)
                {ConcentricSource=original.ConcentricSource};
        }
    }

    private static bool Contact(Point3D a,Point3D b,Point3D c,Point3D d,
        out double u,out double v,out Point3D point,bool literalEndpoints=false)
    {
        u=v=0;point=default;
        if(literalEndpoints)
        {
            if(a==c||a==d){point=a;v=a==c?0:1;return true;}
            if(b==c||b==d){point=b;u=1;v=b==c?0:1;return true;}
        }
        double ax=b.X-a.X,ay=b.Y-a.Y,bx=d.X-c.X,by=d.Y-c.Y;
        double den=ax*by-ay*bx;
        if(den==0)
        {
            // Exact shared endpoints need no rounding or intersection arithmetic.
            if(a==c||a==d){point=a;u=0;v=a==c?0:1;return true;}
            if(b==c||b==d){point=b;u=1;v=b==c?0:1;return true;}
            if(ax*(c.Y-a.Y)!=ay*(c.X-a.X))return false;
            double first=ax*ax+ay*ay,second=bx*bx+by*by;
            if(first<=0||second<=0)return false;
            foreach(var entry in new[]{(P:c,U:((c.X-a.X)*ax+(c.Y-a.Y)*ay)/first,V:0d),
                (P:d,U:((d.X-a.X)*ax+(d.Y-a.Y)*ay)/first,V:1d),
                (P:a,U:0d,V:((a.X-c.X)*bx+(a.Y-c.Y)*by)/second),
                (P:b,U:1d,V:((b.X-c.X)*bx+(b.Y-c.Y)*by)/second)})
            {
                if(entry.U<0||entry.U>1||entry.V<0||entry.V>1)continue;
                if(a.Z+entry.U*(b.Z-a.Z)!=c.Z+entry.V*(d.Z-c.Z))continue;
                point=entry.P;u=entry.U;v=entry.V;return true;
            }
            return false;
        }
        u=((c.X-a.X)*by-(c.Y-a.Y)*bx)/den;
        v=((c.X-a.X)*ay-(c.Y-a.Y)*ax)/den;
        if(u<0||u>1||v<0||v>1)return false;
        var p=new Point3D(a.X+u*ax,a.Y+u*ay,a.Z+u*(b.Z-a.Z));
        double z=c.Z+v*(d.Z-c.Z);
        if(p.Z!=z)return false;
        point=p;return true;
    }
    private static Point64 XY(Point3D p)=>new((long)Math.Round(p.X*PathUtils.Scale),(long)Math.Round(p.Y*PathUtils.Scale));
    private static bool Finite(Point3D p)=>double.IsFinite(p.X+p.Y+p.Z)&&Math.Abs(p.X)<=1_000_000&&Math.Abs(p.Y)<=1_000_000;
    private static Point3D Posted(Point3D p)=>new(VCarveCommandRounding.Round((float)p.X),VCarveCommandRounding.Round((float)p.Y),p.Z);
}
