using System.Diagnostics;
namespace RW.Toolpaths.Milling;

// Raise only literal completed return traces. Fresh connectors and all
// required first cuts and ramps retain their exact endpoints and motion kinds.
internal static class VCarveReturnTravel
{
    internal static List<TaggedToolpath> Improve(IReadOnlyList<TaggedToolpath> source,VCarveCutterProfile profile,
        double surface,MillingMotionSettings motion,double grade,Func<Point3D,Point3D,bool> originalGuard,
        CancellationToken token,Action<string>? log,double? originalMaterialSurface=null,Point3D? incoming=null,bool optimizeCutRetraces=false,
        bool trimCompletedExits=false)
    {
        long begun=Stopwatch.GetTimestamp();
        double materialSurface=originalMaterialSurface??surface;
        double postedTop=(float)materialSurface-(float)surface;
        var nominal=new CutterSweepStock(profile,p=>p,materialSurface,token,mergeIncrementally:true);
        var posted=new CutterSweepStock(profile,p=>p,materialSurface,token,mergeIncrementally:true);
        var literalNominal=new HashSet<(Point3D,Point3D)>();var literalPosted=new HashSet<(Point3D,Point3D)>();
        var cost=new ToolpathMotionCost(motion,surface);var result=new List<TaggedToolpath>();
        var replayMasks=new List<bool[]>();int replayEligible=0,replayChanged=0;double replayLength=0;
        int trials=0,raised=0,fast=0,eligible=0,trimmedExits=0,removedReplayChords=0;double removedReplayLength=0;
        double maximumLift=0;Point3D? previous=incoming,actual=null;
        Point3D N(Point3D p)=>new((float)p.X,(float)p.Y,(float)p.Z-(float)materialSurface);
        Point3D Q(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),ToolpathCommandRounding.Round((float)p.Y),ToolpathCommandRounding.Round((float)p.Z-(float)surface)-postedTop);
        Point3D Relative(Point3D p)=>p with{Z=p.Z-surface};
        static bool Has(HashSet<(Point3D,Point3D)> set,Point3D a,Point3D b)=>set.Contains((a,b))||set.Contains((b,a));
        static ToolpathSpan[] Spans(ToolpathSpanKind[] kinds){
            var spans=new List<ToolpathSpan>();int first=0;
            for(int i=1;i<=kinds.Length;i++)if(i==kinds.Length||kinds[i]!=kinds[first]){spans.Add(new(first,i,kinds[first]));first=i;}
            return spans.ToArray();
        }
        bool Clear(Point3D a,Point3D b,Point3D qa,Point3D qb){
            if(Math.Min(a.Z,b.Z)>=materialSurface+MillingLocalTravel.Clearance+.001
                &&Math.Min(qa.Z,qb.Z)>=MillingLocalTravel.Clearance)return true;
            return nominal.ContainsTravel(N(a),N(b),MillingLocalTravel.Clearance)
                &&posted.ContainsTravel(qa,qb,MillingLocalTravel.Clearance);
        }
        for(int pathIndex=0;pathIndex<source.Count;pathIndex++){
            var path=source[pathIndex];
            token.ThrowIfCancellationRequested();var points=path.Points.ToArray();var kinds=ToolpathSpanKinds.Expand(path);
            var replayMask=new bool[kinds.Length];replayMasks.Add(replayMask);
            if(previous is {} prev&&prev!=points[0]&&!MillingTransitionPlanner.RequiresPlanarTraverse(prev,points[0])
                &&(N(prev).X!=N(points[0]).X||N(prev).Y!=N(points[0]).Y)){
                log?.Invoke("stage=ring-return-travel refused=ambiguous-bridge-XY-anchor");return source.ToList();
            }
            if(previous is null||previous!=points[0])actual=Q(points[0]);
            int index=0;
            while(index<kinds.Length){
                bool Replay(int edge,Point3D qa){
                    if(!optimizeCutRetraces||kinds[edge]!=ToolpathSpanKind.Cut)return false;
                    var a=points[edge];var b=points[edge+1];
                    // Keep partial-axis micro moves in their original mode.
                    if(ToolpathCommandRounding.IsVerticalOnly((float)b.X-(float)a.X,(float)b.Z-(float)a.Z,(float)b.Y-(float)a.Y))return false;
                    return Has(literalNominal,N(a),N(b))&&Has(literalPosted,qa,Q(b));
                }
                bool cutReplay=Replay(index,actual??Q(points[index]));
                if(kinds[index]==ToolpathSpanKind.Link||cutReplay){
                    int end=index+1;
                    if(cutReplay){
                        var cursor=Q(points[end]);
                        while(end<kinds.Length&&Replay(end,cursor)){cursor=Q(points[end+1]);end++;}
                        for(int i=index;i<end;i++){replayMask[i]=true;replayEligible++;replayLength+=ToolpathContinuationPlanner.Distance(points[i],points[i+1]);}
                    }else while(end<kinds.Length&&kinds[end]==ToolpathSpanKind.Link)end++;
                    var original=points.Skip(index).Take(end-index+1).ToArray();int count=original.Length;
                    var pinned=new bool[count];pinned[0]=pinned[^1]=true;
                    for(int i=1;i<count;i++){
                        bool old=Has(literalNominal,N(original[i-1]),N(original[i]))&&Has(literalPosted,Q(original[i-1]),Q(original[i]));
                        if(!old)pinned[i-1]=pinned[i]=true;else eligible++;
                    }
                    nominal.Seal();posted.Seal();
                    var distance=new double[count];for(int i=1;i<count;i++)distance[i]=distance[i-1]+Math.Sqrt(Math.Pow(original[i].X-original[i-1].X,2)+Math.Pow(original[i].Y-original[i-1].Y,2));
                    var best=original;var bestKinds=Enumerable.Repeat(cutReplay?ToolpathSpanKind.Cut:ToolpathSpanKind.Link,count-1).ToArray();
                    double bestSeconds=cost.VisitSeconds(new(original,Spans(bestKinds),0,"return-study",0,detectCurves:false));
                    foreach(double height in new[]{0,.15,.3,.6,1.2,2.4,4.0}){
                        token.ThrowIfCancellationRequested();trials++;
                        var roof=original.Select((p,i)=>p.Z+(pinned[i]?0:height)).ToArray();
                        double slope=grade*.98;
                        for(int i=1;i<count;i++)roof[i]=Math.Min(roof[i],roof[i-1]+slope*(distance[i]-distance[i-1]));
                        for(int i=count-2;i>=0;i--)roof[i]=Math.Min(roof[i],roof[i+1]+slope*(distance[i+1]-distance[i]));
                        var proposal=original.Select((p,i)=>p with{Z=Math.Max(p.Z,Math.Min(motion.SafeZ-.002,roof[i]-.002))}).ToArray();
                        var proposedKinds=new ToolpathSpanKind[count-1];bool legal=true;var trialCursor=actual??Q(original[0]);
                        for(int i=1;i<count;i++){
                            if((proposal[i-1]!=original[i-1]||proposal[i]!=original[i])
                                &&!originalGuard(Relative(proposal[i-1]),Relative(proposal[i]))){legal=false;break;}
                            var a=proposal[i-1];var b=proposal[i];var qb=Q(b);
                            if(ToolpathCommandRounding.IsVerticalOnly((float)b.X-(float)a.X,(float)b.Z-(float)a.Z,(float)b.Y-(float)a.Y))qb=new(trialCursor.X,trialCursor.Y,qb.Z);
                            proposedKinds[i-1]=Clear(a,b,trialCursor,qb)?ToolpathSpanKind.ClearedTravel:ToolpathSpanKind.Link;
                            trialCursor=qb;
                        }
                        if(!legal)continue;
                        double seconds=cost.VisitSeconds(new(proposal,Spans(proposedKinds),0,"return-study",0,detectCurves:false));
                        if(seconds<bestSeconds-1e-9){best=proposal;bestKinds=proposedKinds;bestSeconds=seconds;}
                    }
                    // An already-independent reentry needs no return to this circuit's start.
                    // Only a suffix certified against the actual earlier prefix may disappear.
                    // Decide before Record: discarded trial motion must never become stock evidence.
                    if(trimCompletedExits&&cutReplay&&end==kinds.Length){
                        Point3D? following=pathIndex+1<source.Count?source[pathIndex+1].Points[0]:null;
                        bool continuous=following is {} next&&MillingTransitionPlanner.IsContinuous(points[^1],next);
                        if(!continuous){
                            double Outgoing(Point3D p)=>following is {} next?cost.TransferSeconds(p,next)
                                :p.Z<motion.SafeZ?cost.ChordSeconds(p,p with{Z=motion.SafeZ}):0;
                            var prefixSeconds=new double[count];
                            for(int i=1;i<count;i++)prefixSeconds[i]=prefixSeconds[i-1]+cost.ChordSeconds(best[i-1],best[i],bestKinds[i-1]);
                            double chosenSeconds=prefixSeconds[^1]+Outgoing(best[^1]);int keep=count;
                            int first=index==0?1:0;
                            int stride=Math.Max(1,(count-first+46)/48);
                            for(int i=first;i<count-1;i+=stride){
                                var exit=best[i];
                                if(following is {} to){
                                    bool oldTraverse=MillingTransitionPlanner.RequiresPlanarTraverse(points[^1],to);
                                    bool newTraverse=MillingTransitionPlanner.RequiresPlanarTraverse(exit,to);
                                    if(oldTraverse!=newTraverse||MillingTransitionPlanner.IsContinuous(exit,to))continue;
                                    if(!newTraverse&&(N(exit).X!=N(to).X||N(exit).Y!=N(to).Y))continue;
                                }
                                if(!originalGuard(Relative(exit),Relative(exit)))continue;
                                double seconds=prefixSeconds[i]+Outgoing(exit);
                                if(seconds<chosenSeconds-1e-9){chosenSeconds=seconds;keep=i+1;}
                            }
                            if(keep<count){
                                trimmedExits++;removedReplayChords+=count-keep;
                                for(int i=keep;i<count;i++)removedReplayLength+=ToolpathContinuationPlanner.Distance(original[i-1],original[i]);
                                end=index+keep-1;best=best.Take(keep).ToArray();bestKinds=bestKinds.Take(keep-1).ToArray();count=keep;
                                Array.Resize(ref points,end+1);Array.Resize(ref kinds,end);
                            }
                        }
                    }
                    if(!best.SequenceEqual(original))raised++;
                    if(cutReplay)for(int i=0;i<bestKinds.Length;i++)if(bestKinds[i]!=ToolpathSpanKind.Cut||best[i]!=original[i]||best[i+1]!=original[i+1])replayChanged++;
                    for(int i=0;i<count;i++){maximumLift=Math.Max(maximumLift,best[i].Z-original[i].Z);points[index+i]=best[i];}
                    for(int i=0;i<bestKinds.Length;i++){kinds[index+i]=bestKinds[i];if(bestKinds[i]==ToolpathSpanKind.ClearedTravel)fast++;}
                    // Commit only the actual chosen trajectory, never trial cuts.
                    for(int i=index;i<end;i++)Record(points[i],points[i+1]);
                    index=end;
                }else{Record(points[index],points[index+1]);index++;}
            }
            var updated=path with{Points=points,Moves=new LineToolMoves(points),Spans=Spans(kinds)};
            result.Add(updated);previous=points[^1];
        }
        double before=cost.SequenceSeconds(null,source,null),after=cost.SequenceSeconds(null,result,null);
        VerifySourceMotion(source,result,replayMasks);
        log?.Invoke($"stage=ring-return-travel beforeSeconds={before:R} afterSeconds={after:R} accepted={after<before-1e-9} trials={trials} eligibleLiteralChords={eligible} raisedSpans={raised} clearedChords={fast} maximumLift={maximumLift:R} "
            +$"elapsedMs={Stopwatch.GetElapsedTime(begun).TotalMilliseconds:F1} originalCutAndRampVerticesUnchanged={replayChanged==0&&removedReplayChords==0} requiredFirstCutAndRampVerticesUnchanged=True replayedCutChords={replayEligible} changedReplayChords={replayChanged} replayedCutLengthMm={replayLength:R} cutRetracePolicy={optimizeCutRetraces} trimmedCompletedExits={trimmedExits} removedReplayChords={removedReplayChords} removedReplayLengthMm={removedReplayLength:R} continuousHandoffsPreserved=True freshConnectorChordsPinned=True actualEarlierPrefixOnly=True wholeCutterClearanceMm={MillingLocalTravel.Clearance:R} nominalAndF3=True");
        var chosen=after<before-1e-9?result:source.ToList();
        MillingGeometryTrace.Capture("return-source",0,source);
        MillingGeometryTrace.Capture("return-result",0,chosen);
        return chosen;

        void Record(Point3D a,Point3D b){
            var na=N(a);var nb=N(b);var qa=actual??Q(a);var qb=Q(b);
            bool vertical=ToolpathCommandRounding.IsVerticalOnly((float)b.X-(float)a.X,(float)b.Z-(float)a.Z,(float)b.Y-(float)a.Y);
            if(vertical)qb=new(qa.X,qa.Y,qb.Z);
            nominal.RecordExecuted(na,nb);posted.RecordExecuted(qa,qb);
            literalNominal.Add((na,nb));literalPosted.Add((qa,qb));actual=qb;
        }
    }

    private static void VerifySourceMotion(IReadOnlyList<TaggedToolpath> before,IReadOnlyList<TaggedToolpath> after,IReadOnlyList<bool[]> replayMasks)
    {
        if(before.Count!=after.Count)throw new InvalidOperationException("Return optimization changed source path count.");
        for(int p=0;p<before.Count;p++){
            var a=before[p];var b=after[p];var kinds=ToolpathSpanKinds.Expand(a);var actualKinds=ToolpathSpanKinds.Expand(b);
            if(b.Points.Count>a.Points.Count||b.Points.Count<2)throw new InvalidOperationException("Return optimization changed source subdivision.");
            for(int i=b.Points.Count-1;i<kinds.Length;i++)if(!replayMasks[p][i])
                throw new InvalidOperationException($"Return optimization removed required source motion at {p}:{i}.");
            for(int i=0;i<actualKinds.Length;i++)if(kinds[i]!=ToolpathSpanKind.Link&&!replayMasks[p][i]
                &&(kinds[i]!=actualKinds[i]||a.Points[i]!=b.Points[i]||a.Points[i+1]!=b.Points[i+1]))
                throw new InvalidOperationException($"Return optimization changed non-Link source motion at {p}:{i}.");
        }
    }

    // Independent replay at a routing checkpoint or after final compaction.
    // Each fast chord must remain clear in the actual earlier prefix, including modal F3 XY custody.
    // This stock never contains neighbours, future paths or trial trajectories.
    internal static object VerifyFinalTravel(IReadOnlyList<TaggedToolpath> paths,VCarveCutterProfile profile,
        double surface,CancellationToken token,Action<string>? log,double? originalMaterialSurface=null,Point3D? incoming=null,
        bool finalCompactionComplete=false)
    {
        double materialSurface=originalMaterialSurface??surface;
        double postedTop=(float)materialSurface-(float)surface;
        var native=new CutterSweepStock(profile,p=>p,materialSurface,token,mergeIncrementally:true);
        var posted=new CutterSweepStock(profile,p=>p,materialSurface,token,mergeIncrementally:true);
        Point3D N(Point3D p)=>new((float)p.X,(float)p.Y,(float)p.Z-(float)materialSurface);
        Point3D Q(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),ToolpathCommandRounding.Round((float)p.Y),ToolpathCommandRounding.Round((float)p.Z-(float)surface)-postedTop);
        int fast=0,above=0,prefix=0,cuts=0;Point3D? previous=incoming,cursor=null;
        for(int p=0;p<paths.Count;p++){
            var path=paths[p];var kinds=ToolpathSpanKinds.Expand(path);
            if(previous is null||previous!=path.Points[0]||cursor is null){
                if(previous is {} prev&&!MillingTransitionPlanner.RequiresPlanarTraverse(prev,path.Points[0])
                    &&(N(prev).X!=N(path.Points[0]).X||N(prev).Y!=N(path.Points[0]).Y))
                    throw new InvalidOperationException("Travel audit found an ambiguous reentry XY anchor.");
                cursor=Q(path.Points[0]);
            }
            for(int i=1;i<path.Points.Count;i++){
                token.ThrowIfCancellationRequested();var a=path.Points[i-1];var b=path.Points[i];
                var na=N(a);var nb=N(b);var qa=cursor!.Value;var qb=Q(b);
                if(ToolpathCommandRounding.IsVerticalOnly((float)b.X-(float)a.X,(float)b.Z-(float)a.Z,(float)b.Y-(float)a.Y))qb=new(qa.X,qa.Y,qb.Z);
                if(kinds[i-1]==ToolpathSpanKind.ClearedTravel){
                    fast++;
                    if(Math.Min(na.Z,nb.Z)>=MillingLocalTravel.Clearance&&Math.Min(qa.Z,qb.Z)>=MillingLocalTravel.Clearance)above++;
                    else{
                        native.Seal();posted.Seal();
                        if(!native.ContainsTravel(na,nb,MillingLocalTravel.Clearance)||!posted.ContainsTravel(qa,qb,MillingLocalTravel.Clearance))
                            throw new InvalidOperationException($"Final positive-clearance replay refused {p}:{i-1}; native={na}->{nb}; F3={qa}->{qb}.");
                        prefix++;
                    }
                }else cuts++;
                native.RecordExecuted(na,nb);posted.RecordExecuted(qa,qb);cursor=qb;
            }
            previous=path.Points[^1];
        }
        var receipt=new{passed=true,fastChords=fast,aboveOriginalMaterialChords=above,actualEarlierPrefixChords=prefix,nonFastChords=cuts,
            positiveWholeCutterClearanceMm=MillingLocalTravel.Clearance,nominalAndActualF3=true,afterFinalCompaction=finalCompactionComplete,
            futureAndNeighborStockExcluded=true,trialStockExcluded=true};
        log?.Invoke("stage=ring-return-travel-proof "+System.Text.Json.JsonSerializer.Serialize(receipt));return receipt;
    }
}
