using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Optional same-pass demand reduction. Permanent retained donors may
/// replace contiguous target work at the existing normal finish allowance. Safe
/// reentries are charged before splitting; every ORIGINAL target is rechecked
/// against the final keepers to prevent compounded allowances. No travel credit.</summary>
internal static class VCarveCoveredRunDemand
{
    internal readonly record struct AuditChord(Point3D NativeA,Point3D NativeB,Point3D PostedA,Point3D PostedB);
    private static bool Vertical(Point3D a,Point3D b)=>ToolpathCommandRounding.IsVerticalOnly(
        (float)b.X-(float)a.X,(float)b.Z-(float)a.Z,(float)b.Y-(float)a.Y);
    // Both the final literal inventory and ORIGINAL target audit use this same
    // cursor reconstruction. A vertical command retains its current XY words;
    // the first pose belongs to this exact source visit, not a preceding path.
    internal static IEnumerable<AuditChord> ReconstructForAudit(TaggedToolpath path,double surface,double materialSurface)
    {
        double postedTop=(float)materialSurface-(float)surface;
        Point3D N(Point3D p)=>new((float)p.X,(float)p.Y,(float)p.Z-(float)materialSurface);
        Point3D Q(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),ToolpathCommandRounding.Round((float)p.Y),
            ToolpathCommandRounding.Round((float)p.Z-(float)surface)-postedTop);
        var n=N(path.Points[0]);var q=Q(path.Points[0]);
        for(int i=1;i<path.Points.Count;i++)
        {
            var a=path.Points[i-1];var b=path.Points[i];var nn=N(b);var qq=Q(b);
            if(Vertical(a,b)){nn=new(n.X,n.Y,nn.Z);qq=new(q.X,q.Y,qq.Z);}
            yield return new(n,nn,q,qq);n=nn;q=qq;
        }
    }
    internal static TaggedToolpath[] Apply(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<TaggedToolpath> otherRequired,IReadOnlyList<TaggedToolpath> originalRings,
        VCarveCutterProfile profile,DepthSchedule depth,VCarveOptions options,
        CancellationToken token,Action<string>? log,bool reuseFinalLiteralChords=true)
    {
        long begun=Stopwatch.GetTimestamp();token.ThrowIfCancellationRequested();
        long phaseStarted=begun,verified=0,nativeLiteralHits=0,postedLiteralHits=0,nativeFullProofs=0,postedFullProofs=0,bothLiteral=0;
        string phase="validation";double nominationMs=0,finalStockBuildMs=0,finalAuditMs=0;
        TaggedToolpath[] Retain(string reason){log?.Invoke($"stage=ring-covered-run-demand accepted=False reason={reason} originalSourceRetained=True noTravelPermission=True "
            +$"phase={phase} phaseElapsedMs={Stopwatch.GetElapsedTime(phaseStarted).TotalMilliseconds:F3} nominationMs={nominationMs:F3} finalStockBuildMs={finalStockBuildMs:F3} "
            +$"originalTargetIntervalsVerified={verified} finalNativeLiteralHits={nativeLiteralHits} finalPostedLiteralHits={postedLiteralHits} "
            +$"finalNativeFullProofs={nativeFullProofs} finalPostedFullProofs={postedFullProofs} literalFinalAuditEnabled={reuseFinalLiteralChords}");return source.ToArray();}
        if(source.Count==0)return source.ToArray();
        if(originalRings.Count==0)return Retain("missing-original-targets");
        if(depth.DepthPerPass>0&&depth.DepthPerPass<depth.Depth-1e-9
            ||source.Any(p=>p.DepthPassIndex is >0)||originalRings.Any(p=>p.DepthPassIndex is >0))return Retain("multiple-depth-layers");
        if(options.Motion is not {} motion)return Retain("missing-motion-settings");
        if(!(options.PathToleranceMm>0)||!double.IsFinite(options.PathToleranceMm))return Retain("invalid-finish-allowance");
        if(!double.IsFinite(options.OutputChordToleranceMm)||options.OutputChordToleranceMm<0)return Retain("invalid-output-chord-reserve");
        bool Cutting(TaggedToolpath p)=>p.Points.Count>1&&p.Spans.All(s=>s.Kind==ToolpathSpanKind.Cut);
        if(source.Concat(otherRequired).Concat(originalRings).Any(p=>!Cutting(p)))return Retain("unsupported-source-motion");
        if(source.Concat(otherRequired).Concat(originalRings).Any(p=>p.DetectCurves))return Retain("typed-curve-source");
        const long maximumChords=1_500_000;
        long targetChords=originalRings.Concat(otherRequired).Sum(p=>(long)p.Points.Count-1);
        if(targetChords>maximumChords||source.Concat(otherRequired).Sum(p=>(long)p.Points.Count-1)>maximumChords)return Retain("source-budget");
        double materialSurface=double.IsFinite(motion.MaterialSurfaceZ)?motion.MaterialSurfaceZ:depth.SurfaceZ;
        double postedTop=(float)materialSurface-(float)depth.SurfaceZ,allowance=options.PathToleranceMm/Math.Sqrt(2);
        // A later geometric simplifier may displace the donor sweep. Reserve its
        // envelope in BOTH axial/radial components; final targets still use the
        // user's single original finish allowance, never a compounded allowance.
        double nominationAllowance=allowance-options.OutputChordToleranceMm;
        if(nominationAllowance<=0)return Retain("insufficient-finish-reserve");
        Point3D N(Point3D p)=>new((float)p.X,(float)p.Y,(float)p.Z-(float)materialSurface);
        Point3D Q(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),ToolpathCommandRounding.Round((float)p.Y),
            ToolpathCommandRounding.Round((float)p.Z-(float)depth.SurfaceZ)-postedTop);
        bool Budget()=>Stopwatch.GetElapsedTime(begun).TotalSeconds<=30;
        phase="nomination";phaseStarted=Stopwatch.GetTimestamp();
        var native=new CutterSweepStock(profile,p=>p,materialSurface,token,mergeIncrementally:true,radialMarginMemoEntries:4096);
        var posted=new CutterSweepStock(profile,p=>p,materialSurface,token,mergeIncrementally:true,radialMarginMemoEntries:4096);
        var literalN=new HashSet<(Point3D,Point3D)>();var literalQ=new HashSet<(Point3D,Point3D)>();
        static bool Has(HashSet<(Point3D,Point3D)> set,Point3D a,Point3D b)=>set.Contains((a,b))||set.Contains((b,a));
        void Record(Point3D a,Point3D b){
            var na=N(a);var nb=N(b);var qa=Q(a);var qb=Q(b);
            native.RecordExecuted(na,nb);posted.RecordExecuted(qa,qb);literalN.Add((na,nb));literalQ.Add((qa,qb));
        }
        foreach(var p in otherRequired)for(int i=1;i<p.Points.Count;i++)Record(p.Points[i-1],p.Points[i]);
        native.Seal();posted.Seal();
        var cost=new ToolpathMotionCost(motion,depth.SurfaceZ);
        double Length(IReadOnlyList<Point3D> p)=>p.Zip(p.Skip(1)).Sum(e=>ToolpathContinuationPlanner.Distance(e.First,e.Second));
        var ordered=Enumerable.Range(0,source.Count).OrderBy(i=>source[i].Points.Min(p=>p.Z)).ThenByDescending(i=>Length(source[i].Points)).ToArray();
        var selected=new List<TaggedToolpath>[source.Count];
        int removedPaths=0,changedSources=0,runCandidates=0;double omittedLength=0,isolatedSaving=0;
        foreach(int owner in ordered){
            token.ThrowIfCancellationRequested();if(!Budget())return Retain("time-budget");
            var path=source[owner];var p=path.Points;var covered=new bool[p.Count-1];var keep=Enumerable.Repeat(true,p.Count-1).ToArray();
            native.Seal();posted.Seal();
            // Typed circuit definitions own their geometry and subdivision.
            // Retain them rather than copying stale definitions onto fragments.
            bool eligible=path.ContinuationCircuitDefinition is null;
            for(int i=0;i<covered.Length;i++){
                if((i&255)==0){token.ThrowIfCancellationRequested();if(!Budget())return Retain("time-budget");}
                var a=p[i];var b=p[i+1];
                covered[i]=eligible&&!Vertical(a,b)&&(Has(literalN,N(a),N(b))&&Has(literalQ,Q(a),Q(b))
                    ||native.Contains(N(a),N(b),nominationAllowance,-nominationAllowance)&&posted.Contains(Q(a),Q(b),nominationAllowance,-nominationAllowance));
                if(!covered[i])Record(a,b); // permanent donor; never omitted later
            }
            for(int first=0;first<covered.Length;){
                if(!covered[first]){first++;continue;}int end=first+1;while(end<covered.Length&&covered[end])end++;
                runCandidates++;double length=0,cutSeconds=0;
                for(int i=first;i<end;i++){length+=ToolpathContinuationPlanner.Distance(p[i],p[i+1]);cutSeconds+=cost.ChordSeconds(p[i],p[i+1],ToolpathSpanKind.Cut);}
                double replacement=first==0||end==covered.Length
                    ?length*60/motion.TravelRate+Math.Abs(p[end].Z-p[first].Z)*60/motion.PlungeRate
                    :cost.TransferSeconds(p[first],p[end]);
                if(cutSeconds>replacement+.05)for(int i=first;i<end;i++)keep[i]=false;
                first=end;
            }
            var pieces=new List<Point3D[]>();
            for(int first=0;first<keep.Length;){
                if(!keep[first]){first++;continue;}int end=first+1;while(end<keep.Length&&keep[end])end++;
                pieces.Add(p.Skip(first).Take(end-first+1).ToArray());first=end;
            }
            if(pieces.Count>1&&pieces[^1][^1]==pieces[0][0]){var joined=pieces[^1].Concat(pieces[0].Skip(1)).ToArray();pieces.RemoveAt(pieces.Count-1);pieces[0]=joined;}
            TaggedToolpath Fragment(Point3D[] points)=>path with{Points=points,Moves=new LineToolMoves(points),
                Spans=[new(0,points.Length-1,ToolpathSpanKind.Cut)],ContinuationSourceOrdinal=null};
            var beforeEntry=p[0] with{Z=motion.SafeZ};var afterExit=p[^1] with{Z=motion.SafeZ};
            double before=cost.SequenceSeconds(beforeEntry,[path],afterExit);
            var proposal=pieces.Select(Fragment).ToList();
            double after=proposal.Count==0?cost.TransferSeconds(beforeEntry,afterExit):cost.SequenceSeconds(beforeEntry,proposal,afterExit);
            if(after>=before-.05){selected[owner]=[path];Array.Fill(keep,true);}
            else{selected[owner]=proposal;changedSources++;if(proposal.Count==0)removedPaths++;
                omittedLength+=Length(p)-pieces.Sum(Length);isolatedSaving+=before-after;}
            for(int i=0;i<keep.Length;i++)if(keep[i]&&covered[i])Record(p[i],p[i+1]);
        }
        nominationMs=Stopwatch.GetElapsedTime(phaseStarted).TotalMilliseconds;
        log?.Invoke($"stage=ring-covered-run-radial-memo calls={native.RadialMarginCalls+posted.RadialMarginCalls} hits={native.RadialMarginMemoHits+posted.RadialMarginMemoHits} "
            +$"misses={native.RadialMarginMemoMisses+posted.RadialMarginMemoMisses} evictions={native.RadialMarginMemoEvictions+posted.RadialMarginMemoEvictions} "
            +$"entries={native.RadialMarginMemoEntries+posted.RadialMarginMemoEntries} maximumEntriesPerOwner=4096 exactIeeeArguments=True literalProfileMinimum=True stockEvidenceCached=False");
        if(changedSources==0)return Retain("no-cost-improvement");
        var result=selected.SelectMany(p=>p).ToArray();
        if(result.Length>source.Count+512)return Retain("fragment-growth-budget");
        // NEW proof stock from FINAL keepers only. Earlier omitted short/empty
        // targets are checked directly, not through an intermediate allowance.
        phase="final-stock-build";phaseStarted=Stopwatch.GetTimestamp();
        // Nomination donors may have disappeared. Reuse storage, never their
        // evidence: only the FINAL keepers can supply exact literal identity.
        literalN.Clear();literalQ.Clear();
        var finalN=new CutterSweepStock(profile,p=>p,materialSurface,token);var finalQ=new CutterSweepStock(profile,p=>p,materialSurface,token);
        long literalRecorded=0;
        foreach(var path in result.Concat(otherRequired))
        {
            // The original stock construction remains unchanged for every
            // unmatched full query. Exact identity uses actual modal chords.
            for(int i=1;i<path.Points.Count;i++){
                finalN.RecordExecuted(N(path.Points[i-1]),N(path.Points[i]));finalQ.RecordExecuted(Q(path.Points[i-1]),Q(path.Points[i]));}
            if(reuseFinalLiteralChords)
            foreach(var chord in ReconstructForAudit(path,depth.SurfaceZ,materialSurface))
            {
                if((literalRecorded++&255)==0)token.ThrowIfCancellationRequested();
                static bool Finite(Point3D p)=>double.IsFinite(p.X)&&double.IsFinite(p.Y)&&double.IsFinite(p.Z);
                if(Finite(chord.NativeA)&&Finite(chord.NativeB))literalN.Add((chord.NativeA,chord.NativeB));
                if(Finite(chord.PostedA)&&Finite(chord.PostedB))literalQ.Add((chord.PostedA,chord.PostedB));
            }
        }
        finalN.Seal();finalQ.Seal();finalStockBuildMs=Stopwatch.GetElapsedTime(phaseStarted).TotalMilliseconds;
        phase="final-target-audit";phaseStarted=Stopwatch.GetTimestamp();
        foreach(var path in originalRings.Concat(otherRequired)){
            foreach(var chord in ReconstructForAudit(path,depth.SurfaceZ,materialSurface)){
                if((verified++&255)==0){token.ThrowIfCancellationRequested();if(!Budget())return Retain("time-budget");}
                bool exactN=reuseFinalLiteralChords&&Has(literalN,chord.NativeA,chord.NativeB);
                if(exactN)nativeLiteralHits++;
                else{nativeFullProofs++;if(!finalN.Contains(chord.NativeA,chord.NativeB,allowance,-allowance))return Retain("original-target-coverage-refused");}
                bool exactQ=reuseFinalLiteralChords&&Has(literalQ,chord.PostedA,chord.PostedB);
                if(exactQ)postedLiteralHits++;
                else{postedFullProofs++;if(!finalQ.Contains(chord.PostedA,chord.PostedB,allowance,-allowance))return Retain("original-target-coverage-refused");}
                if(exactN&&exactQ)bothLiteral++;
            }
        }
        finalAuditMs=Stopwatch.GetElapsedTime(phaseStarted).TotalMilliseconds;
        log?.Invoke($"stage=ring-covered-run-demand accepted=True sourcePaths={source.Count} retainedFragments={result.Length} changedSources={changedSources} wholePathsRemoved={removedPaths} "
            +$"coveredRunNominations={runCandidates} omittedLengthMm={omittedLength:R} isolatedSafeEntrySavingSeconds={isolatedSaving:R} originalTargetIntervalsVerified={verified} "
            +$"finishToleranceMm={options.PathToleranceMm:R} nominalAndActualF3=True permanentRetainedDonors=True compoundedAllowance=False singleDepthPassOnly=True "
            +$"nominationAxialAndRadialAllowanceMm={nominationAllowance:R} donorOutputChordReserveMm={options.OutputChordToleranceMm:R} "
            +$"nominationMs={nominationMs:F3} finalStockBuildMs={finalStockBuildMs:F3} finalTargetAuditMs={finalAuditMs:F3} "
            +$"finalNativeLiteralHits={nativeLiteralHits} finalPostedLiteralHits={postedLiteralHits} finalBothLiteralIntervals={bothLiteral} "
            +$"finalNativeFullProofs={nativeFullProofs} finalPostedFullProofs={postedFullProofs} finalNativeLiteralChords={literalN.Count} finalPostedLiteralChords={literalQ.Count} "
            +$"literalFinalAuditEnabled={reuseFinalLiteralChords} literalAuthority=identical-final-modal-physical-sweep immutableOriginalTarget=True "
            +$"originalConcentricAncestryPreserved=True fragmentOwnersIndependent=True noTravelPermission=True elapsedMs={Stopwatch.GetElapsedTime(begun).TotalMilliseconds:F1}");
        return result;
    }
}
