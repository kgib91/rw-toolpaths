using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Retain the original wall identity when offset fronts merge. This
/// partitions existing directed source chords; it grants no cutting permission.</summary>
internal static class VCarveWallSources
{
    // Same-source diagnostic comparison: retain original feature ownership on
    // the already partitioned local strips. No additional source splitting.
    internal static readonly AsyncLocal<bool?> CompleteLocalWallBandsOverride=new();
    internal static List<TaggedToolpath> Assign(IReadOnlyList<TaggedToolpath> source,
        CancellationToken token,Action<string>? log,Action<int,TaggedToolpath>? partition=null)
    {
        var indices=new Dictionary<VCarveRingOrder.Plan,WallIndex>();
        var result=new List<TaggedToolpath>();long queries=0,chords=0;int splitSources=0,unclassified=0;
        for(int sourceIndex=0;sourceIndex<source.Count;sourceIndex++)
        {
            var path=source[sourceIndex];
            token.ThrowIfCancellationRequested();
            if(path.ConcentricSource is not {Plan.Walls:not null} row||path.Points.Count<2
                ||path.Spans.Any(s=>s.Kind!=ToolpathSpanKind.Cut))
            {result.Add(path);partition?.Invoke(sourceIndex,path);if(path.ConcentricSource is not null)unclassified++;continue;}
            if(!indices.TryGetValue(row.Plan,out var index))indices.Add(row.Plan,index=new(row.Plan.Walls!));
            var runs=new List<(int Wall,List<Point3D> Points)>();
            for(int end=1;end<path.Points.Count;end++)
            {
                if((end&1023)==0)token.ThrowIfCancellationRequested();
                var a=path.Points[end-1];var b=path.Points[end];
                int wall=index.Wall(new Point64((long)Math.Round((a.X+b.X)*.5*PathUtils.Scale),
                    (long)Math.Round((a.Y+b.Y)*.5*PathUtils.Scale)));queries++;chords++;
                if(runs.Count==0||runs[^1].Wall!=wall)runs.Add((wall,new(){a,b}));
                else runs[^1].Points.Add(b);
            }
            if(runs.Count>1&&path.Points[0]==path.Points[^1]&&runs[0].Wall==runs[^1].Wall)
            {
                runs[^1].Points.AddRange(runs[0].Points.Skip(1));runs.RemoveAt(0);
            }
            if(runs.Count>1)splitSources++;
            foreach(var run in runs)
            {
                var points=run.Points.ToArray();
                var part=path with{Points=points,Moves=new LineToolMoves(points),
                    Spans=new[]{new ToolpathSpan(0,points.Length-1,ToolpathSpanKind.Cut)},
                    ConcentricSource=row with{Wall=run.Wall}};
                result.Add(part);partition?.Invoke(sourceIndex,part);
            }
        }
        log?.Invoke($"stage=wall-source-order event=assigned inputSources={source.Count} outputSources={result.Count} "
            +$"splitSources={splitSources} wallPlans={indices.Count} nearestWallQueries={queries} originalDirectedChords={chords} "
            +$"unclassifiedRingSources={unclassified} sourceChordsPreserved=True newGeometry=False nearestWallIsOrderingOnly=True");
        return result;
    }
    internal static TaggedToolpath[] LocalStrips(IReadOnlyList<TaggedToolpath> source,VCarveCutterProfile profile,
        DepthSchedule depth,VCarveOptions options,CancellationToken token,Action<string>? log)
    {
        // Keep the existing cross-pass owner until the same partition can be
        // proved across capped circuit definitions. Single-layer wall pieces
        // have no hidden preceding depth job to lose when assigning local rows.
        if(depth.DepthPerPass>0&&depth.DepthPerPass<depth.Depth-1e-9
            ||source.Select(p=>p.DepthPassIndex).Distinct().Skip(1).Any()){
            log?.Invoke("stage=local-wall-offsets event=retained reason=multiple-depth-layers originalPassDependenciesPreserved=True");
            return source.ToArray();
        }
        int levels=ConcentricBandSchedule.Ramp(profile,depth.Depth,options).Levels;
        bool Slope(TaggedToolpath p)=>p.ConcentricSource is {Plan.Walls:not null} row
            &&row.Plan.Levels[row.Ring]<levels&&p.Spans.All(s=>s.Kind==ToolpathSpanKind.Cut);
        var selected=Enumerable.Range(0,source.Count).Where(i=>Slope(source[i])).ToArray();
        if(selected.Length==0)return source.ToArray();
        var pieces=Enumerable.Range(0,source.Count).Select(_=>new List<TaggedToolpath>()).ToArray();
        Assign(selected.Select(i=>source[i]).ToArray(),token,log,(i,p)=>pieces[selected[i]].Add(p));
        var plans=pieces.SelectMany(p=>p).Where(p=>p.ConcentricSource is {Wall:>0})
            .GroupBy(p=>(p.ConcentricSource!.Plan,p.ConcentricSource.Wall)).ToDictionary(g=>g.Key,g=>{
                var offsets=g.Select(p=>p.ConcentricSource!.Plan.Levels[p.ConcentricSource.Ring]).Distinct().Order().ToArray();
                var plan=new VCarveRingOrder.Plan(Enumerable.Range(0,offsets.Length).Select(i=>i-1),
                    insideOut:options.ProfileDirection==VCarveProfileDirection.BottomUp,levels:offsets,walls:g.Key.Plan.Walls);
                return(Plan:plan,Offsets:offsets);
            });
        var result=new List<TaggedToolpath>();
        for(int i=0;i<source.Count;i++){
            if(!Slope(source[i])){result.Add(source[i]);continue;}
            foreach(var p in pieces[i]){
                var row=p.ConcentricSource!;
                if(row.Wall>0&&plans.TryGetValue((row.Plan,row.Wall),out var local))
                    result.Add(p with{ConcentricSource=new(local.Plan,Array.IndexOf(local.Offsets,row.Plan.Levels[row.Ring]),
                        Wall:CompleteLocalWallBandsOverride.Value==true?row.Wall:-1),ContinuationSourceOrdinal=null});
                else result.Add(p with{ConcentricSource=row with{Wall=-1}});
            }
        }
        log?.Invoke($"stage=local-wall-offsets event=assigned inputPaths={source.Count} outputPaths={result.Count} localFamilies={plans.Count} slopeLevels={levels} "
            +$"literalDirectedChordsPreserved=True nearestOriginalWallOwnsOffsetAncestry=True wholeWallLevelBatching={CompleteLocalWallBandsOverride.Value==true} alreadyPartitionedStripsOnly=True newCutPermission=False");
        return result.ToArray();
    }

    private sealed class WallIndex
    {
        private readonly PolygonOps.BoundarySegmentIndex index;
        private readonly Dictionary<(Point64,Point64),int> owners=new();
        internal WallIndex(IReadOnlyList<IReadOnlyList<PointD>> walls)
        {
            var boundaries=PathUtils.ToClipper(walls);index=new(boundaries);
            for(int wall=0;wall<boundaries.Count;wall++)
            for(int end=0,start=boundaries[wall].Count-1;end<boundaries[wall].Count;start=end++)
            {var a=boundaries[wall][start];var b=boundaries[wall][end];owners.TryAdd((a,b),wall);owners.TryAdd((b,a),wall);}
        }
        internal int Wall(Point64 point)
        {var edge=index.NearestSegmentForCertificate(point);return owners[(edge.Start,edge.End)];}
    }
}
