using System.Diagnostics;

namespace RW.Toolpaths.Milling;

// Run AFTER exact command compaction, whole-region ordering AND host final
// surface preparation. No subsequent geometric reduction is permitted.
internal static class VCarveAirRampFeed
{
    internal static List<TaggedToolpath> Improve(IReadOnlyList<TaggedToolpath> source,VCarveCutterProfile profile,
        double surface,MillingMotionSettings motion,CancellationToken token,Action<string>? log,
        double? originalMaterialSurface=null,Point3D? incoming=null)
    {
        long begun=Stopwatch.GetTimestamp();token.ThrowIfCancellationRequested();
        if(source.Any(p=>p.DetectCurves||p.Moves is not LineToolMoves)){
            log?.Invoke("stage=air-ramp-feed refused=nonlinear-motion originalMotionRetained=True");return source.ToList();
        }
        double material=originalMaterialSurface??(double.IsFinite(motion.MaterialSurfaceZ)?motion.MaterialSurfaceZ:surface);
        double postedTop=(float)material-(float)surface;
        var cost=new ToolpathMotionCost(motion,surface);var result=new List<TaggedToolpath>(source.Count);
        CutterSweepStock? native=null,posted=null;int? region=null;Point3D? previous=incoming,cursor=null;
        int queried=0,changed=0,regions=0,demoted=0;double saved=0,lost=0;
        Point3D N(Point3D p)=>new((float)p.X,(float)p.Y,(float)p.Z-(float)material);
        Point3D Q(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),ToolpathCommandRounding.Round((float)p.Y),
            ToolpathCommandRounding.Round((float)p.Z-(float)surface)-postedTop);
        foreach(var path in source){
            token.ThrowIfCancellationRequested();
            if(region!=path.RegionIndex){
                region=path.RegionIndex;regions++;
                native=new(profile,p=>p,material,token,mergeIncrementally:true);
                posted=new(profile,p=>p,material,token,mergeIncrementally:true);
            }
            if(previous is {} prev&&prev!=path.Points[0]&&!MillingTransitionPlanner.RequiresPlanarTraverse(prev,path.Points[0])
                &&(N(prev).X!=N(path.Points[0]).X||N(prev).Y!=N(path.Points[0]).Y)){
                log?.Invoke("stage=air-ramp-feed refused=ambiguous-bridge-XY-anchor originalMotionRetained=True");return source.ToList();
            }
            if(previous is null||previous!=path.Points[0]||cursor is null)cursor=Q(path.Points[0]);
            var kinds=ToolpathSpanKinds.Expand(path);bool modified=false;
            for(int i=1;i<path.Points.Count;i++){
                token.ThrowIfCancellationRequested();var a=path.Points[i-1];var b=path.Points[i];
                var na=N(a);var nb=N(b);var qa=cursor!.Value;var qb=Q(b);
                if(ToolpathCommandRounding.IsVerticalOnly((float)b.X-(float)a.X,(float)b.Z-(float)a.Z,(float)b.Y-(float)a.Y))qb=new(qa.X,qa.Y,qb.Z);
                if(kinds[i-1] is ToolpathSpanKind.Ramp or ToolpathSpanKind.ClearedTravel){
                    bool wasFast=kinds[i-1]==ToolpathSpanKind.ClearedTravel;
                    queried++;native!.Seal();posted!.Seal();
                    bool clear=Math.Min(na.Z,nb.Z)>=MillingLocalTravel.Clearance+.001
                        &&Math.Min(qa.Z,qb.Z)>=MillingLocalTravel.Clearance;
                    clear=clear||native.ContainsTravel(na,nb,MillingLocalTravel.Clearance)
                        &&posted.ContainsTravel(qa,qb,MillingLocalTravel.Clearance);
                    if(clear&&!wasFast){
                        double gain=cost.ChordSeconds(a,b,ToolpathSpanKind.Ramp)-cost.ChordSeconds(a,b,ToolpathSpanKind.ClearedTravel);
                        if(gain>1e-9){kinds[i-1]=ToolpathSpanKind.ClearedTravel;changed++;saved+=gain;modified=true;}
                    }else if(!clear&&wasFast){
                        // An earlier donor may have changed during final surface
                        // preparation. Keep the same legal path at cutting feed;
                        // never inherit stale fast-feed authority across that step.
                        kinds[i-1]=ToolpathSpanKind.Link;demoted++;modified=true;
                        lost+=cost.ChordSeconds(a,b,ToolpathSpanKind.Link)-cost.ChordSeconds(a,b,ToolpathSpanKind.ClearedTravel);
                    }
                }
                // Commit only after querying. Neither this chord nor a future
                // ramp, cut, transfer or neighbouring region can authorize it.
                native!.RecordExecuted(na,nb);posted!.RecordExecuted(qa,qb);cursor=qb;
            }
            if(modified){
                var spans=new List<ToolpathSpan>();int first=0;
                for(int i=1;i<=kinds.Length;i++)if(i==kinds.Length||kinds[i]!=kinds[first]){
                    spans.Add(new(first,i,kinds[first]));first=i;}
                result.Add(path with{Spans=spans});
            }else result.Add(path);
            previous=path.Points[^1];
        }
        log?.Invoke($"stage=air-ramp-feed regions={regions} queries={queried} acceleratedChords={changed} savedSeconds={saved-lost:R} grossSavedSeconds={saved:R} demotedStaleFastChords={demoted} revalidationCostSeconds={lost:R} "
            +$"afterFinalCompaction=True afterSurfaceEmissionPreparation=True originalPointsAndOrderUnchanged=True cutAndContactRampPreserved=True actualEarlierPrefixOnly=True "
            +$"wholeCutterClearanceMm={MillingLocalTravel.Clearance:R} nominalAndActualF3=True elapsedMs={Stopwatch.GetElapsedTime(begun).TotalMilliseconds:F1}");
        return result;
    }
}
