using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>The constructor's offset levels own progression. After trimming,
/// finish each piece's passes and follow its next local offset. Experimental
/// original-wall ownership completes each wall's level before advancing.
/// No global intersection or executed-chord routing graph.</summary>
internal sealed partial class VCarveRingOrder
{
    internal sealed class Plan
    {
        internal readonly IReadOnlyList<int> Parents;
        internal readonly IReadOnlyList<int> Levels,Regions;
        internal IReadOnlyList<IReadOnlyList<PointD>>? Walls{get;}
        private readonly int[] topDown,bottomUp;
        internal bool InsideOut{get;}
        internal Plan(IEnumerable<int> parents,IReadOnlyList<int>? constructionOrder=null,bool insideOut=false,
            IReadOnlyList<int>? levels=null,IReadOnlyList<int>? regions=null,IReadOnlyList<IReadOnlyList<PointD>>? walls=null)
        {
            Walls=walls?.Select(w=> (IReadOnlyList<PointD>)Array.AsReadOnly(w.ToArray())).ToArray();
            InsideOut=insideOut;Parents=Array.AsReadOnly(parents.ToArray());
            topDown=Ranks(Traverse(false));bottomUp=Ranks(Traverse(true));
            var derived=new int[Parents.Count];
            foreach(int ring in Enumerable.Range(0,Parents.Count).OrderBy(i=>topDown[i]))
                derived[ring]=Parents[ring]<0?0:derived[Parents[ring]]+1;
            Levels=Array.AsReadOnly(levels?.ToArray()??derived);
            Regions=Array.AsReadOnly(regions?.ToArray()??new int[Parents.Count]);
            if(constructionOrder is not null)
            {
                var ranks=Ranks(constructionOrder);
                if(insideOut)bottomUp=ranks;else topDown=ranks;
            }
        }
        internal int Rank(int ring,bool insideOut)=>insideOut?bottomUp[ring]:topDown[ring];
        private int[] Ranks(IReadOnlyList<int> order)
        {
            if(order.Count!=Parents.Count||order.Distinct().Count()!=Parents.Count
                ||order.Any(i=>i<0||i>=Parents.Count))throw new ArgumentException("Incomplete constructed ring order.");
            var result=new int[order.Count];for(int i=0;i<order.Count;i++)result[order[i]]=i;return result;
        }
        private int[] Traverse(bool insideOut)
        {
            var children=Enumerable.Range(0,Parents.Count).Select(_=>new List<int>()).ToArray();
            for(int i=0;i<Parents.Count;i++)if(Parents[i]>=0)children[Parents[i]].Add(i);
            var result=new List<int>();var stack=new Stack<(int Ring,bool Finish)>();var seen=new HashSet<int>();
            foreach(int root in Enumerable.Range(0,Parents.Count).Where(i=>Parents[i]<0).Reverse())stack.Push((root,false));
            while(stack.TryPop(out var at))
            {
                if(at.Finish){result.Add(at.Ring);continue;}
                if(!seen.Add(at.Ring))throw new ArgumentException("Cyclic offset construction tree.");
                if(insideOut)stack.Push((at.Ring,true));else result.Add(at.Ring);
                foreach(int child in children[at.Ring].AsEnumerable().Reverse())stack.Push((child,false));
            }
            if(result.Count!=Parents.Count)throw new ArgumentException("Incomplete offset construction tree.");
            return result.ToArray();
        }
    }
    internal sealed record Visit(Plan Plan,int Ring,IReadOnlyList<int>? Members=null,int Wall=-1)
    {
        internal IEnumerable<int> Rings=>Members??new[]{Ring};
        internal static Visit? Combine(IEnumerable<Visit?> source)
        {
            var visits=source.Where(v=>v is not null).Cast<Visit>().Distinct().ToArray();
            if(visits.Length==0)return null;
            if(visits.Any(v=>!ReferenceEquals(v.Plan,visits[0].Plan))
                ||visits.Select(v=>v.Wall).Distinct().Skip(1).Any()
                ||visits.SelectMany(v=>v.Rings).Distinct().Skip(1).Any())
                throw new InvalidOperationException("A source detour cannot combine different constructed concentric rows.");
            return visits[0];
        }
    }
    private sealed class Job
    {
        internal readonly int[] Passes;
        internal int Next;
        internal int Pending;
        internal readonly List<Job> Before=new(),After=new();
        internal bool Done=>Next==Passes.Length;
        internal LevelBatch? Batch;
        internal bool Ready=>!Done&&Pending==0&&(Batch?.Ready??true);
        internal Job(IEnumerable<int> passes)=>Passes=passes.ToArray();
    }
    private sealed class LevelBatch
    {
        internal LevelBatch? Previous;
        internal int Remaining;
        internal readonly List<Job> Members=new();
        internal bool Ready=>Previous is null||Previous.Remaining==0;
    }
    private readonly Job[] all;
    private readonly Dictionary<int,Job> jobs=new();
    private readonly CancellationToken token;
    private readonly IReadOnlyList<TaggedToolpath> paths;
    private Job? current;
    private readonly double reach;
    internal int RowCount{get;}
    internal long ParentQueries{get;}
    internal long ParentBoundsRefusals{get;}
    internal int Count{get;}
    internal int WallCount{get;}
    internal int WallLevelBatches{get;}
    internal VCarveRingOrder(IReadOnlyList<TaggedToolpath> paths,bool bottomUp,CancellationToken token,double reach=double.PositiveInfinity,
        SpatialSweep? spatialSweep=null)
    {
        this.paths=paths;this.token=token;this.reach=reach;Count=paths.Count;
        var plans=paths.Select(p=>p.ConcentricSource!.Plan).Distinct().ToArray();
        var ordered=Enumerable.Range(0,paths.Count).GroupBy(i=>
        {
            var visit=paths[i].ConcentricSource!;
            if(visit.Rings.Distinct().Skip(1).Any())
                throw new InvalidOperationException("A routed source spans multiple constructed rows; preserve rows before routing.");
            return(visit.Plan,visit.Ring);
        }).OrderBy(g=>Array.IndexOf(plans,g.Key.Plan)).ThenBy(g=>g.Key.Plan.Rank(g.Key.Ring,bottomUp));
        RowCount=ordered.Count();
        all=ordered.SelectMany(g=>g.GroupBy(i=>(paths[i].ConcentricSource!.Wall,paths[i].ContinuationSourceOrdinal.HasValue,paths[i].ContinuationSourceOrdinal??i))
            .Select(fragment=>
            {
                var job=new Job(fragment.OrderBy(i=>paths[i].DepthPassIndex??int.MaxValue));
                foreach(int owner in job.Passes)jobs.Add(owner,job);return job;
            })).ToArray();
        var walls=all.Where(j=>paths[j.Passes[0]].ConcentricSource!.Wall>=0)
            .GroupBy(j=>(paths[j.Passes[0]].ConcentricSource!.Plan,paths[j.Passes[0]].ConcentricSource!.Wall)).ToArray();
        WallCount=walls.Length;
        foreach(var wall in walls)
        {
            LevelBatch? previous=null;
            foreach(var level in wall.GroupBy(j=>wall.Key.Plan.Levels[paths[j.Passes[0]].ConcentricSource!.Ring])
                .OrderBy(g=>bottomUp?-g.Key:g.Key))
            {
                var batch=new LevelBatch{Previous=previous,Remaining=level.Count()};
                foreach(var job in level){job.Batch=batch;batch.Members.Add(job);}
                previous=batch;WallLevelBatches++;
            }
        }
        var pieces=all.Select(j=>j.Passes).ToArray();
        var offsets=new VCarveOffsetPieces(paths,pieces,bottomUp,reach,token);
        ParentQueries=offsets.Queries;ParentBoundsRefusals=offsets.BoundedQueries;
        foreach(var (first,next) in offsets.Links)
        {all[first].After.Add(all[next]);all[next].Before.Add(all[first]);all[next].Pending++;}
        InitializeSpatial(spatialSweep);
    }
    internal bool IsReady(int owner)=>jobs[owner].Ready&&jobs[owner].Passes[jobs[owner].Next]==owner;
    internal IEnumerable<(int First,int Next)> Dependencies()
    {
        foreach(var job in all){
            for(int pass=1;pass<job.Passes.Length;pass++)yield return(job.Passes[pass-1],job.Passes[pass]);
            foreach(var parent in job.Before)yield return(parent.Passes[^1],job.Passes[0]);
            if(job.Batch?.Previous is {} previous)
                foreach(var parent in all.Where(j=>ReferenceEquals(j.Batch,previous)))yield return(parent.Passes[^1],job.Passes[0]);
        }
    }
    private int[] LegacyCandidates(Point3D? cursor)
    {
        token.ThrowIfCancellationRequested();
        if(current is {Done:false})return new[]{current.Passes[current.Next]};
        if(CurrentWallReady() is {Length:>0} wallReady)return wallReady;
        // Follow the construction's next offset locally. Only when that strip
        // ends do we return to a branch or start another retained strip.
        if(current is not null)
        {
            var fromRow=paths[current.Passes[0]].ConcentricSource!;
            if(cursor is {} from)
            {
                var beside=all.Where(j=>j.Ready&&SameOffset(j,fromRow)&&Near(j,from)).ToArray();
                if(beside.Length>0)return beside.Select(j=>j.Passes[j.Next]).ToArray();
            }
            var pending=new Queue<Job>();var seen=new HashSet<Job>();pending.Enqueue(current);
            while(pending.TryDequeue(out var at))
            {
                if(!seen.Add(at))continue;
                var next=at.After.Where(j=>j.Ready).ToArray();
                if(next.Length>0)return next.Select(j=>j.Passes[j.Next]).ToArray();
                var support=at.After.Where(j=>!j.Done).SelectMany(j=>j.Before).Where(j=>j.Ready).Distinct().ToArray();
                if(support.Length>0)return support.Select(j=>j.Passes[j.Next]).ToArray();
                foreach(var parent in at.Before)pending.Enqueue(parent);
            }
        }
        var first=all.FirstOrDefault(j=>j.Ready);
        if(first is null)return Array.Empty<int>();
        var row=paths[first.Passes[0]].ConcentricSource!;
        return all.Where(j=>j.Ready&&ReferenceEquals(paths[j.Passes[0]].ConcentricSource!.Plan,row.Plan)
            &&paths[j.Passes[0]].ConcentricSource!.Ring==row.Ring).Select(j=>j.Passes[j.Next]).ToArray();
    }
    private int[] LegacyIndependentCandidates()=>CurrentWallReady() is {Length:>0} ready?ready
        :all.Where(j=>j.Ready).Select(j=>j.Passes[j.Next]).ToArray();
    private int[] CurrentWallReady()
    {
        if(current is null||paths[current.Passes[0]].ConcentricSource is not {Wall:>=0} row)return Array.Empty<int>();
        if(!current.Done)return new[]{current.Passes[current.Next]};
        return all.Where(j=>j.Ready&&paths[j.Passes[0]].ConcentricSource is {} other
            &&ReferenceEquals(row.Plan,other.Plan)&&row.Wall==other.Wall).Select(j=>j.Passes[j.Next]).ToArray();
    }
    private bool SameOffset(Job job,Visit row)
    {
        var other=paths[job.Passes[0]].ConcentricSource!;
        return ReferenceEquals(row.Plan,other.Plan)&&row.Plan.Regions[row.Ring]==other.Plan.Regions[other.Ring]
            &&row.Plan.Levels[row.Ring]==other.Plan.Levels[other.Ring];
    }
    private bool Near(Job job,Point3D from)
    {
        if(!double.IsFinite(reach))return true;
        var p=paths[job.Passes[job.Next]].Points;
        for(int i=1;i<p.Count;i++)
        {
            double x=p[i].X-p[i-1].X,y=p[i].Y-p[i-1].Y,square=x*x+y*y;
            double t=square>0?Math.Clamp(((from.X-p[i-1].X)*x+(from.Y-p[i-1].Y)*y)/square,0,1):0;
            if(Math.Pow(from.X-p[i-1].X-t*x,2)+Math.Pow(from.Y-p[i-1].Y-t*y,2)<=reach*reach)return true;
        }
        return false;
    }
    internal void Complete(int owner)
    {
        token.ThrowIfCancellationRequested();
        if(!IsReady(owner))throw new InvalidOperationException("A concentric visit left its constructed row order.");
        TrackSpatialComplete(owner);
        current=jobs[owner];current.Next++;
        if(current.Done&&current.Batch is {} batch)batch.Remaining--;
        if(current.Done)foreach(var next in current.After)next.Pending--;
        spatialPool=null;
    }
}
