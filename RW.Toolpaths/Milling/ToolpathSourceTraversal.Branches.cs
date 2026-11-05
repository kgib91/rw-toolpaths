using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

internal static partial class ToolpathSourceTraversal
{
    /// <summary>Attach required final-layer medial walks to their entered carriers.
    /// The caller establishes that the complete starting floor is already cleared.
    /// Geometry, stock and posted-coordinate authority remain with its adapter.</summary>
    internal static List<TaggedToolpath> FinishingBranches(IReadOnlyList<TaggedToolpath> required,
        IReadOnlyList<TaggedToolpath> entered,List<TaggedToolpath> baseline,double reach,double grade,
        Func<Point3D,Point3D,bool> validate,Func<Point3D,double> entryHeight,MillingMotionSettings motion,double surface,
        Point3D? incoming,CancellationToken token,Action<string>? log=null)
    {
        token.ThrowIfCancellationRequested();var watch=Stopwatch.StartNew();
        List<TaggedToolpath> Refuse(string reason)
        {log?.Invoke($"stage=branch-detours event=retained reason={reason} originalEntries=True");return baseline;}
        if(!(reach>0)||!(grade>0)||!double.IsFinite(reach+grade)||required.Count<2
            ||required.Select(p=>p.RegionIndex).Distinct().Count()!=1
            ||required.Any(p=>p.Points.Count<2||p.DetectCurves||p.Category is not ("clearing" or "final-carve")
                ||p.Spans.Any(s=>s.Kind!=ToolpathSpanKind.Cut)))return Refuse("source-contract");
        var carriers=required.Where(p=>p.Category=="clearing").ToArray();
        if(carriers.Length==0||carriers.Select(p=>p.DepthPassIndex).Distinct().Count()!=1
            ||carriers.Any(p=>p.Points[0]!=p.Points[^1]))return Refuse("carrier-or-pass-contract");
        if(entered.Count!=required.Count||entered.Any(p=>p.ContinuationSourceOrdinal is null)
            ||entered.Select(p=>p.ContinuationSourceOrdinal).Distinct().Count()!=required.Count)
            return Refuse("source-ownership-unavailable");
        var byOwner=entered.ToDictionary(p=>p.ContinuationSourceOrdinal!.Value);
        if(Enumerable.Range(0,required.Count).Any(i=>!byOwner.ContainsKey(i)))return Refuse("source-ownership-domain");
        var cutStarts=new Dictionary<int,int>();
        foreach(var (owner,path) in byOwner)
        {
            int cut=0;bool cutting=false;
            foreach(var span in path.Spans)
            {
                if(span.Kind==ToolpathSpanKind.Cut){if(!cutting)cut=span.StartIndex;cutting=true;}
                else if(cutting||span.Kind!=ToolpathSpanKind.Ramp)return Refuse("entered-motion-kinds");
            }
            cutStarts.Add(owner,cut);
        }
        Point3D Relative(Point3D p)=>p with{Z=p.Z-surface};
        Point3D World(Point3D p)=>p with{Z=p.Z+surface};
        bool Guard(Point3D a,Point3D b)=>validate(World(a),World(b));
        Point3D Command(Point3D p)=>new(VCarveCommandRounding.Round((float)p.X),VCarveCommandRounding.Round((float)p.Y),
            VCarveCommandRounding.Round((float)(p.Z+surface)-(float)surface));
        var sources=new TaggedToolpath[required.Count];double replayRemoved=0;
        for(int owner=0;owner<required.Count;owner++)
        {
            var source=required[owner];var points=source.Points;
            if(source.Category=="clearing")
            {
                var path=byOwner[owner];int cut=cutStarts[owner];var landing=path.Points[cut];
                int contact=Enumerable.Range(0,points.Count-1).FirstOrDefault(i=>points[i]==landing,-1);
                if(contact<0)return Refuse("carrier-landing-not-literal");
                int n=points.Count-1;
                points=Enumerable.Range(0,n+1).Select(i=>source.Points[(contact+i)%n]).ToArray();
                replayRemoved+=Length(path.Points.Skip(cut).ToArray())-Length(points);
            }
            sources[owner]=new(points.Select(Relative).ToArray(),Array.Empty<ToolpathSpan>(),source.RegionIndex,source.Category,
                0,owner,detectCurves:false);
        }
        var joined=BranchJunctions(sources,reach,grade,Guard,Command,token,log);
        var groups=new Dictionary<int,IReadOnlyList<int>>();
        var detours=Detours(joined,token,log,Guard,grade,(owner,members)=>groups.Add(owner,members));
        var circuits=detours.Where(p=>p.ContinuationCircuit&&p.Category=="clearing"&&p.VisitIdentity is {} owner
            &&groups.TryGetValue(owner,out var members)&&members.Any(i=>required[i].Category=="final-carve")).ToArray();
        if(circuits.Length==0)return Refuse("no-attached-medial-work");
        var consumed=new HashSet<int>();var replacements=new Dictionary<int,TaggedToolpath>();
        foreach(var circuit in circuits)
        {
            int owner=circuit.VisitIdentity!.Value;var original=byOwner[owner];int cut=cutStarts[owner];
            var points=original.Points.Take(cut).Concat(circuit.Points.Select(World)).ToArray();
            if(points[cut]!=original.Points[cut])return Refuse("carrier-landing-changed");
            var spans=original.Spans.Where(s=>s.EndIndex<=cut).Append(new ToolpathSpan(cut,points.Length-1,ToolpathSpanKind.Cut)).ToArray();
            replacements.Add(owner,new(points,spans,original.RegionIndex,original.Category,original.DepthPassIndex,
                original.VisitIdentity,detectCurves:false){ContinuationSourceOrdinal=owner});
            foreach(int member in groups[owner])if(!consumed.Add(member))return Refuse("source-owned-twice");
        }
        var candidate=new List<TaggedToolpath>();
        foreach(var original in entered)
        {
            int owner=original.ContinuationSourceOrdinal!.Value;
            if(replacements.TryGetValue(owner,out var replacement))candidate.Add(replacement);
            else if(!consumed.Contains(owner))candidate.Add(original);
        }
        Point3D? previous=incoming;long chords=0;
        foreach(var path in candidate)
        {
            // A source formerly sharing its predecessor's end has no independent
            // entry. If that dependency changed, require real earlier-stock entry
            // clearance rather than silently introducing an unproved plunge.
            if(previous!=path.Points[0]&&!path.Spans.Any(s=>s.Kind==ToolpathSpanKind.Ramp)
                &&entryHeight(path.Points[0])>path.Points[0].Z)return Refuse("dependent-entry-required");
            for(int i=1;i<path.Points.Count;i++)
            {
                if((++chords&1023)==0)token.ThrowIfCancellationRequested();
                if(!validate(path.Points[i-1],path.Points[i]))return Refuse("final-native-and-posted-chord");
            }
            previous=path.Points[^1];
        }
        var cost=new ToolpathMotionCost(motion,surface);
        double before=cost.SequenceSeconds(incoming,baseline,null),after=cost.SequenceSeconds(incoming,candidate,null);
        bool accept=after<before-1e-9;
        log?.Invoke($"stage=branch-detours event=complete region={required[0].RegionIndex} accepted={accept} "
            +$"sources={required.Count} carriers={carriers.Length} attachedSources={consumed.Count-replacements.Count} "
            +$"beforeVisits={baseline.Count} afterVisits={candidate.Count} beforeSec={before:R} afterSec={after:R} "
            +$"carrierEntryReplayRemovedMm={replayRemoved:R} checkedChords={chords} elapsedMs={watch.Elapsed.TotalMilliseconds:F3} "
            +"sourceOwnership=stable-ordinal completeFinalLayer=True originalMedialCutsRetained=True nativeAndF3=True");
        return accept?candidate:baseline;

        static double Length(IReadOnlyList<Point3D> points)=>Enumerable.Range(1,points.Count-1)
            .Sum(i=>ToolpathContinuationPlanner.Distance(points[i-1],points[i]));
    }

    private static TaggedToolpath[] BranchJunctions(TaggedToolpath[] original,double reach,double grade,
        Func<Point3D,Point3D,bool> guard,Func<Point3D,Point3D> command,CancellationToken token,Action<string>? log)
    {
        var watch=Stopwatch.StartNew();
        var source=original.Select(p=>
        {
            var a=p.Points[0];var b=p.Points[^1];
            // Retain both endpoints of a numerically open, posted-identical
            // source cycle. This must not introduce a new vertical entry.
            if(p.Category=="final-carve"&&a!=b&&(float)a.X==(float)b.X&&(float)a.Y==(float)b.Y
                &&ToolpathContinuationPlanner.Distance(a,b)<=1e-8&&command(a)==command(b)&&guard(b,a))
                return p with{Points=p.Points.Append(a).ToArray()};
            return p;
        }).ToArray();
        var rows=new List<Segment>();var boxes=new Paths64();
        for(int owner=0;owner<source.Length;owner++)if(source[owner].Category=="clearing")
        for(int end=1;end<source[owner].Points.Count;end++)
        {
            if((rows.Count&1023)==0)token.ThrowIfCancellationRequested();
            if(rows.Count>=MaximumSegments)return original;
            rows.Add(new(owner,end));boxes.Add(new(){XY(source[owner].Points[end-1]),XY(source[owner].Points[end])});
        }
        if(rows.Count==0)return original;
        var index=new PolygonOps.BoundarySegmentIndex(boxes);var matches=new List<int>();
        var splits=new Dictionary<(int,int),List<(double T,Point3D P)>>();var connectors=new List<TaggedToolpath>();
        long extent=(long)Math.Ceiling(reach*PathUtils.Scale)+2;int trials=0,accepted=0,capped=0;long nominees=0;
        for(int owner=0;owner<source.Length;owner++)if(source[owner].Category=="final-carve")
        {
            var candidates=new List<(double Distance,int Row,double T,Point3D A,Point3D B)>();
            foreach(var p in source[owner].Points.Distinct())
            {
                token.ThrowIfCancellationRequested();if(nominees>=MaximumCandidates)break;
                var q=XY(p);if(!index.QueryBounds(q.X-extent,q.Y-extent,q.X+extent,q.Y+extent,matches,256,token))capped++;
                foreach(int ordinal in matches.DistinctBy(v=>v/2))
                {
                    if(++nominees>MaximumCandidates)break;
                    int row=ordinal/2;var s=rows[row];var a=source[s.Owner].Points[s.End-1];var b=source[s.Owner].Points[s.End];
                    double dx=b.X-a.X,dy=b.Y-a.Y,dz=b.Z-a.Z,den=dx*dx+dy*dy+dz*dz;
                    if(den==0)continue;
                    double t=Math.Clamp(((p.X-a.X)*dx+(p.Y-a.Y)*dy+(p.Z-a.Z)*dz)/den,0,1);
                    foreach(double at in new[]{t,0d,1d}.Distinct())
                    {
                        var projected=at==0?a:at==1?b:new(a.X+at*dx,a.Y+at*dy,a.Z+at*dz);
                        double distance=ToolpathContinuationPlanner.Distance(projected,p);
                        double run=Math.Sqrt(Math.Pow(projected.X-p.X,2)+Math.Pow(projected.Y-p.Y,2));
                        var ca=command(projected);var cb=command(p);
                        double postedRun=Math.Sqrt(Math.Pow(ca.X-cb.X,2)+Math.Pow(ca.Y-cb.Y,2));
                        if(distance>reach||Math.Abs(projected.Z-p.Z)>grade*run+1e-12||Math.Abs(ca.Z-cb.Z)>grade*postedRun+1e-12)continue;
                        if(candidates.Any(c=>c.A==projected&&c.B==p))continue;
                        candidates.Add((distance,row,at,projected,p));candidates.Sort((x,y)=>x.Distance.CompareTo(y.Distance));
                        if(candidates.Count>16)candidates.RemoveAt(16);
                    }
                }
            }
            foreach(var c in candidates)
            {
                if(++trials>20_000)break;token.ThrowIfCancellationRequested();var s=rows[c.Row];
                var a=source[s.Owner].Points[s.End-1];var b=source[s.Owner].Points[s.End];
                if(!guard(a,c.A)||!guard(c.A,b)||!guard(c.A,c.B)||!guard(c.B,c.A))continue;
                if(!splits.TryGetValue((s.Owner,s.End),out var list))splits[(s.Owner,s.End)]=list=new();
                list.Add((c.T,c.A));connectors.Add(new(new[]{c.A,c.B,c.A},Array.Empty<ToolpathSpan>(),source[owner].RegionIndex,
                    "branch-link",0,null,detectCurves:false));accepted++;break;
            }
        }
        var result=new List<TaggedToolpath>();
        for(int owner=0;owner<source.Length;owner++)
        {
            var path=source[owner];var points=new List<Point3D>{path.Points[0]};
            for(int end=1;end<path.Points.Count;end++)
            {
                if((end&1023)==0)token.ThrowIfCancellationRequested();
                if(splits.TryGetValue((owner,end),out var list))foreach(var split in list.OrderBy(v=>v.T))if(points[^1]!=split.P)points.Add(split.P);
                if(points[^1]!=path.Points[end])points.Add(path.Points[end]);
            }
            result.Add(path with{Points=points});
        }
        if(result.Any(p=>Enumerable.Range(1,p.Points.Count-1).Any(i=>!guard(p.Points[i-1],p.Points[i]))))
        {log?.Invoke("stage=branch-junctions event=refused reason=complete-subdivisions sourcePreserved=True");return original;}
        log?.Invoke($"stage=branch-junctions event=complete proposed={accepted} trials={trials} nominees={nominees} cappedQueries={capped} "
            +$"elapsedMs={watch.Elapsed.TotalMilliseconds:F3} nativeAndPostedGrade=True bounded=True");
        return result.Concat(connectors).ToArray();
    }
}
