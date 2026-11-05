using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Optimize only safe-plane transfers between complete continuous
/// chains. All source cuts, entries, local links and returns remain literal.</summary>
internal static class VCarveRingTour
{
    internal static List<TaggedToolpath> Improve(List<TaggedToolpath> paths,IReadOnlyList<int> owners,
        IEnumerable<(int First,int Next)> dependencies,ToolpathMotionCost cost,MillingMotionSettings motion,
        Point3D? incoming,CancellationToken token,Action<string>? log)
    {
        long started=Stopwatch.GetTimestamp();
        if(paths.Count!=owners.Count)throw new ArgumentException("Every routed visit needs its literal source owner.");
        var blocks=new List<List<int>>();
        for(int i=0;i<paths.Count;i++){
            if(i==0||paths[i-1].Points[^1]!=paths[i].Points[0])blocks.Add([]);
            blocks[^1].Add(i);
        }
        int count=blocks.Count;
        if(count<2||count>TravelOptimizer.MaximumDirectedTourVisits){
            log?.Invoke($"stage=ring-tour event=bounded chains={count} maximumChains={TravelOptimizer.MaximumDirectedTourVisits}");return paths;
        }
        var ownerSites=new Dictionary<int,(int Block,int Position)>();
        for(int block=0;block<count;block++)foreach(int index in blocks[block])ownerSites.Add(owners[index],(block,index));
        var prerequisites=new int[count];
        foreach(var (first,next) in dependencies){
            var a=ownerSites[first];var b=ownerSites[next];
            if(a.Position>=b.Position)throw new InvalidOperationException("Original routed visits violate a source dependency.");
            if(a.Block!=b.Block)prerequisites[b.Block]|=1<<a.Block;
        }
        var head=blocks.Select(b=>paths[b[0]].Points[0]).ToArray();
        var tail=blocks.Select(b=>paths[b[^1]].Points[^1]).ToArray();
        var starts=head.Select(p=>cost.GlobalTransferSeconds(incoming??p with{Z=motion.SafeZ},p)).ToArray();
        var ends=tail.Select(p=>p.Z<motion.SafeZ?cost.ChordSeconds(p,p with{Z=motion.SafeZ}):0).ToArray();
        var transfer=new double[count*count];
        for(int a=0;a<count;a++)for(int b=0;b<count;b++)transfer[a*count+b]=cost.TransferSeconds(tail[a],head[b]);
        var tour=TravelOptimizer.OrderDirected(transfer,starts,ends,prerequisites,token);
        var candidate=tour.SelectMany(b=>blocks[b].Select(i=>paths[i])).ToList();
        double before=cost.SequenceSeconds(incoming,paths,null),after=cost.SequenceSeconds(incoming,candidate,null);
        bool accepted=after<before-1e-9;
        log?.Invoke($"stage=ring-tour event=complete chains={count} order={string.Join(',',tour)} beforeSeconds={before:R} afterSeconds={after:R} accepted={accepted} "
            +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} exactForFixedChains=True sourceDependenciesPreserved=True localCutsRampsReturnsLiteral=True newCuttingChords=False");
        return accepted?candidate:paths;
    }
}
