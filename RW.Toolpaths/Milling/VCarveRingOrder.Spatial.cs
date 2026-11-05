namespace RW.Toolpaths.Milling;

internal sealed partial class VCarveRingOrder
{
    internal enum SpatialAxis{X,Y}
    internal sealed record SpatialSweep(SpatialAxis Axis,double WindowMm);

    // Experimental scheduling only. Null leaves normal row scheduling and its
    // final chain-tour pass unchanged; this setting grants no stock authority.
    internal static readonly AsyncLocal<SpatialSweep?> SpatialSweepOverride=new();
    private readonly record struct SpatialBounds(double MinX,double MinY,double MaxX,double MaxY)
    {
        internal double Key(SpatialAxis axis)=>axis==SpatialAxis.X?MinX:MinY;
        internal double Extent(SpatialAxis axis)=>axis==SpatialAxis.X?MaxX-MinX:MaxY-MinY;
    }
    private SpatialSweep? spatialSweep;
    private readonly Dictionary<Job,SpatialBounds> spatialBounds=new();
    private Job[] spatialJobs=[];
    private int spatialFrontierIndex;
    private int[]? spatialPool;
    private HashSet<Job> spatialAllowed=new();
    private double spatialFrontier=double.NaN,spatialFirstFrontier=double.NaN,spatialLastSelected=double.NaN;
    private int spatialFrontierAdvances,spatialFrontierReversals,spatialReversals,spatialDependencyEscapes,spatialDependencySearches,
        spatialDependencyBoundRefusals,spatialDependencyFallbacks,spatialStickySelections,spatialSpanningJobs;
    private int spatialPoolCalls,spatialPoolMin=int.MaxValue,spatialPoolMax,spatialSelected;
    private long spatialPoolTotal,spatialSupportNodes;
    private double spatialReverseDistance,spatialMaximumEscape;
    private double? spatialExternalFrontier;
    internal bool SpatialSweepEnabled=>spatialSweep is not null;
    internal double UnfinishedSpatialFrontier
    {
        get{
            while(spatialFrontierIndex<spatialJobs.Length&&spatialJobs[spatialFrontierIndex].Done)spatialFrontierIndex++;
            return spatialFrontierIndex<spatialJobs.Length?spatialBounds[spatialJobs[spatialFrontierIndex]].Key(spatialSweep!.Axis):double.PositiveInfinity;
        }
    }
    internal void SetSpatialExternalFrontier(double? frontier)
    {
        if(spatialExternalFrontier==frontier)return;
        spatialExternalFrontier=frontier;spatialPool=null;
    }
    internal bool HasStickySpatialContinuation(Point3D? cursor,double frontier)
        =>spatialSweep is not null&&current is not null&&cursor is {} from
            &&LegacyCandidates(cursor).Any(owner=>spatialBounds[jobs[owner]].Key(spatialSweep.Axis)<=frontier+spatialSweep.WindowMm
                &&Near(jobs[owner],from));
    internal double ReadySpatialAccessDistanceSquared(Point3D? cursor,double frontier)
    {
        if(spatialSweep is null)return double.PositiveInfinity;
        double best=double.PositiveInfinity;
        foreach(var job in all){
            token.ThrowIfCancellationRequested();double key=spatialBounds[job].Key(spatialSweep.Axis);
            if(!job.Ready||key>frontier+spatialSweep.WindowMm)continue;
            if(cursor is not {} from){best=Math.Min(best,Math.Pow(key-frontier,2));continue;}
            var points=paths[job.Passes[job.Next]].Points;var a=points[0];var b=points[^1];
            best=Math.Min(best,Math.Min(Math.Pow(from.X-a.X,2)+Math.Pow(from.Y-a.Y,2),Math.Pow(from.X-b.X,2)+Math.Pow(from.Y-b.Y,2)));
        }
        return best;
    }

    private void InitializeSpatial(SpatialSweep? sweep)
    {
        if(sweep is null)return;
        if(!Enum.IsDefined(sweep.Axis)||!double.IsFinite(sweep.WindowMm)||sweep.WindowMm<=0)
            throw new ArgumentOutOfRangeException(nameof(sweep),"A spatial sweep needs an X/Y axis and a finite positive window.");
        spatialSweep=sweep;
        long points=0;
        foreach(var job in all){
            double minX=double.PositiveInfinity,minY=minX,maxX=double.NegativeInfinity,maxY=maxX;
            foreach(int pass in job.Passes)foreach(var point in paths[pass].Points){
                if((++points&1023)==0)token.ThrowIfCancellationRequested();
                if(!double.IsFinite(point.X)||!double.IsFinite(point.Y))
                    throw new InvalidOperationException("A spatial sweep needs finite source XY coordinates.");
                minX=Math.Min(minX,point.X);minY=Math.Min(minY,point.Y);
                maxX=Math.Max(maxX,point.X);maxY=Math.Max(maxY,point.Y);
            }
            if(!double.IsFinite(minX+minY+maxX+maxY))
                throw new InvalidOperationException("A spatial sweep needs nonempty finite source XY bounds.");
            var bounds=new SpatialBounds(minX,minY,maxX,maxY);spatialBounds.Add(job,bounds);
            if(bounds.Extent(sweep.Axis)>sweep.WindowMm)spatialSpanningJobs++;
        }
        spatialJobs=all.OrderBy(j=>spatialBounds[j].Key(sweep.Axis)).ToArray();
    }

    internal int[] Candidates(Point3D? cursor)
    {
        if(spatialSweep is null)return LegacyCandidates(cursor);
        var pool=SpatialCandidates();
        // Keep an existing cheap local continuation sticky while it is still
        // in the sweep window. The router retains every physical/cost guard.
        if(cursor is {} from){
            var local=LegacyCandidates(cursor).Where(owner=>spatialAllowed.Contains(jobs[owner])&&Near(jobs[owner],from)).ToArray();
            if(local.Length>0){spatialStickySelections++;return local;}
        }
        return pool;
    }

    internal int[] IndependentCandidates()=>spatialSweep is null?LegacyIndependentCandidates():SpatialCandidates();

    private int[] SpatialCandidates()
    {
        token.ThrowIfCancellationRequested();
        if(spatialPool is not null)return spatialPool;
        while(spatialFrontierIndex<spatialJobs.Length&&spatialJobs[spatialFrontierIndex].Done)spatialFrontierIndex++;
        if(spatialFrontierIndex==spatialJobs.Length){spatialAllowed.Clear();return spatialPool=[];}
        var sweep=spatialSweep!;
        double key=Math.Min(spatialBounds[spatialJobs[spatialFrontierIndex]].Key(sweep.Axis),spatialExternalFrontier??double.PositiveInfinity);
        if(double.IsNaN(spatialFirstFrontier))spatialFirstFrontier=key;
        else if(key>spatialFrontier+1e-9)spatialFrontierAdvances++;
        else if(key<spatialFrontier-1e-9)spatialFrontierReversals++;
        spatialFrontier=key;double limit=key+sweep.WindowMm;
        var ready=all.Where(j=>j.Ready).ToArray();
        if(ready.Length==0)throw new InvalidOperationException("Unfinished concentric work has no dependency-ready source.");
        var allowed=ready.Where(j=>spatialBounds[j].Key(sweep.Axis)<=limit).ToArray();
        if(allowed.Length==0){
            spatialDependencySearches++;
            // An unfinished low-key source can be blocked by an offset, DPP,
            // or wall-level predecessor outside its band. Walk only real
            // incoming edges; bounded refusal never invents eligibility.
            const int seedLimit=64,nodeLimit=256,edgeLimit=1024;
            var pending=new Queue<Job>();var seen=new HashSet<Job>();var support=new HashSet<Job>();
            int seeds=0,edges=0;bool bounded=false;
            for(int i=spatialFrontierIndex;i<spatialJobs.Length;i++){
                var job=spatialJobs[i];if(spatialBounds[job].Key(sweep.Axis)>limit)break;
                if(job.Done)continue;
                if(seeds++==seedLimit){bounded=true;break;}pending.Enqueue(job);
            }
            while(pending.TryDequeue(out var job)){
                token.ThrowIfCancellationRequested();
                if(job.Done||!seen.Add(job))continue;
                if(seen.Count>nodeLimit){bounded=true;break;}
                spatialSupportNodes++;
                if(job.Ready){support.Add(job);continue;}
                IEnumerable<Job> predecessors=job.Before;
                if(job.Batch?.Previous is {Remaining:>0} batch)predecessors=predecessors.Concat(batch.Members);
                foreach(var before in predecessors){
                    if(before.Done)continue;
                    if(++edges>edgeLimit){bounded=true;break;}
                    pending.Enqueue(before);
                }
                if(bounded&&edges>edgeLimit)break;
            }
            if(bounded)spatialDependencyBoundRefusals++;
            if(support.Count==0){
                // Preserve completeness if the bounded prerequisite walk ran
                // out: use the first actually ready spatial band, never a
                // blocked/future source and never an empty fabricated result.
                spatialDependencyFallbacks++;
                double first=ready.Min(j=>spatialBounds[j].Key(sweep.Axis));
                allowed=ready.Where(j=>spatialBounds[j].Key(sweep.Axis)<=first+sweep.WindowMm).ToArray();
            }
            else {
                double first=support.Min(j=>spatialBounds[j].Key(sweep.Axis));
                allowed=support.Where(j=>spatialBounds[j].Key(sweep.Axis)<=first+sweep.WindowMm).ToArray();
            }
        }
        spatialAllowed=new(allowed);spatialPool=allowed.Select(j=>j.Passes[j.Next]).ToArray();
        spatialPoolCalls++;spatialPoolTotal+=spatialPool.Length;
        spatialPoolMin=Math.Min(spatialPoolMin,spatialPool.Length);spatialPoolMax=Math.Max(spatialPoolMax,spatialPool.Length);
        return spatialPool;
    }

    private void TrackSpatialComplete(int owner)
    {
        if(spatialSweep is null)return;
        SpatialCandidates();double key=spatialBounds[jobs[owner]].Key(spatialSweep.Axis);
        if(!double.IsNaN(spatialLastSelected)&&key<spatialLastSelected-1e-9){
            spatialReversals++;spatialReverseDistance+=spatialLastSelected-key;
        }
        spatialLastSelected=key;spatialSelected++;
        double escape=key-spatialFrontier-spatialSweep.WindowMm;
        if(escape>1e-9){spatialDependencyEscapes++;spatialMaximumEscape=Math.Max(spatialMaximumEscape,escape);}
    }

    internal string SpatialSummary()=>spatialSweep is null?"spatialSweep=None"
        :$"spatialSweep={spatialSweep.Axis} spatialWindowMm={spatialSweep.WindowMm:R} spatialScope=ring-routing-batch "
        +$"spatialSelected={spatialSelected} spatialFrontierFirst={spatialFirstFrontier:R} spatialFrontierLast={spatialFrontier:R} "
        +$"spatialFrontierAdvances={spatialFrontierAdvances} spatialFrontierReversals={spatialFrontierReversals} spatialSelectedKeyReversals={spatialReversals} spatialReverseKeyDistanceMm={spatialReverseDistance:R} "
        +$"spatialDependencyEscapes={spatialDependencyEscapes} spatialMaximumEscapeMm={spatialMaximumEscape:R} "
        +$"spatialDependencySearches={spatialDependencySearches} spatialSupportNodes={spatialSupportNodes} "
        +$"spatialDependencyBoundRefusals={spatialDependencyBoundRefusals} spatialDependencyFallbacks={spatialDependencyFallbacks} "
        +$"spatialReadyPoolCalls={spatialPoolCalls} spatialReadyPoolMin={(spatialPoolCalls>0?spatialPoolMin:0)} spatialReadyPoolMax={spatialPoolMax} spatialReadyPoolTotal={spatialPoolTotal} "
        +$"spatialStickyLocalPools={spatialStickySelections} spatialWindowSpanningJobs={spatialSpanningJobs} "
        +"spatialMaximumSupportNodes=256 spatialMaximumSupportEdges=1024 spatialMaximumFrontierSeeds=64 sourceSplitForSweep=False dependencyEligibilityUnchanged=True";
}
