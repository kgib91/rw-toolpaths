namespace RW.Toolpaths.Milling;

/// <summary>Separate finishing demand from contour-construction scaffolding.
/// Omission uses the original material plane and the existing whole-cutter
/// allowance predicate with empty stock. It grants no travel/stock credit.</summary>
internal static class VCarveRingDemand
{
    // Same-pass demand reduction is selected with smooth connection optimization.
    // Planned omission never supplies travel authority; false retains a control.
    internal static readonly AsyncLocal<bool?> CoveredRunOptimizationOverride=new();

    internal static TaggedToolpath[] WithoutCoveredRuns(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<TaggedToolpath> otherRequired,IReadOnlyList<TaggedToolpath> originalRings,
        VCarveCutterProfile profile,DepthSchedule depth,VCarveOptions options,
        CancellationToken token,Action<string>? log)
        =>CoveredRunOptimizationOverride.Value!=false
            ?VCarveCoveredRunDemand.Apply(source,otherRequired,originalRings,profile,depth,options,token,log)
            :source.ToArray();

    internal static TaggedToolpath[] Required(IReadOnlyList<TaggedToolpath> source,VCarveCutterProfile profile,
        double datum,double originalMaterialSurface,double finishTolerance,CancellationToken token,Action<string>? log)
    {
        if(!double.IsFinite(datum)||!double.IsFinite(originalMaterialSurface)||!double.IsFinite(finishTolerance)||finishTolerance<=0)
            throw new ArgumentOutOfRangeException(nameof(finishTolerance));
        double allowance=finishTolerance/Math.Sqrt(2);
        var nominal=new CutterSweepStock(profile,p=>p,originalMaterialSurface,token);
        var posted=new CutterSweepStock(profile,p=>p,originalMaterialSurface,token);
        double postedOriginalTop=(float)originalMaterialSurface-(float)datum;
        Point3D Command(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),ToolpathCommandRounding.Round((float)p.Y),
            ToolpathCommandRounding.Round((float)p.Z-(float)datum)-postedOriginalTop);
        var kept=new List<TaggedToolpath>(source.Count);int omitted=0;double omittedLength=0;
        foreach(var path in source){
            token.ThrowIfCancellationRequested();
            bool empty=path.Points.Count>1&&path.Spans.All(s=>s.Kind==ToolpathSpanKind.Cut);
            for(int i=1;empty&&i<path.Points.Count;i++){
                var a=path.Points[i-1];var b=path.Points[i];
                empty=nominal.Contains(a with{Z=a.Z-originalMaterialSurface},b with{Z=b.Z-originalMaterialSurface},allowance,-allowance)
                    &&posted.Contains(Command(a),Command(b),allowance,-allowance);
            }
            if(!empty){kept.Add(path);continue;}
            omitted++;for(int i=1;i<path.Points.Count;i++)omittedLength+=ToolpathContinuationPlanner.Distance(path.Points[i-1],path.Points[i]);
        }
        log?.Invoke($"stage=ring-demand event=complete sourcePaths={source.Count} retainedPaths={kept.Count} omittedPaths={omitted} omittedLengthMm={omittedLength:R} "
            +$"originalMaterialSurfaceZ={originalMaterialSurface:R} finishToleranceMm={finishTolerance:R} emptyNominalAndF3WholeCutterProof=True futureStockOrTravelCredit=False wholeOriginalTargetStillRequired=True");
        return kept.ToArray();
    }

    /// <summary>Remove short offset debris only when immutable, retained source
    /// sweeps already cover its entire physical removal. Donors are never
    /// candidates for omission in this pass, so two fragments cannot delete
    /// each other. Planned removal is not executed-stock/travel permission.</summary>
    internal static TaggedToolpath[] WithoutCoveredFragments(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<TaggedToolpath> otherRequired,VCarveCutterProfile profile,double datum,
        double originalMaterialSurface,double finishTolerance,double fragmentLength,
        CancellationToken token,Action<string>? log)
    {
        if(!double.IsFinite(datum)||!double.IsFinite(originalMaterialSurface)||!double.IsFinite(finishTolerance)||finishTolerance<=0)
            throw new ArgumentOutOfRangeException(nameof(finishTolerance));
        long started=System.Diagnostics.Stopwatch.GetTimestamp();
        if(source.Any(p=>p.DepthPassIndex is >0)){
            log?.Invoke("stage=ring-fragment-demand event=retained reason=required-depth-pass-predecessors originalSourcePassesPreserved=True");
            return source.ToArray();
        }
        if(!(fragmentLength>0)||!double.IsFinite(fragmentLength)||source.Count<2)return source.ToArray();
        double Length(TaggedToolpath p)=>p.Points.Zip(p.Points.Skip(1)).Sum(e=>ToolpathContinuationPlanner.Distance(e.First,e.Second));
        bool Cutting(TaggedToolpath p)=>p.Points.Count>1&&p.Spans.All(s=>s.Kind==ToolpathSpanKind.Cut);
        var lengths=source.Select(Length).ToArray();
        var candidates=Enumerable.Range(0,source.Count).Where(i=>lengths[i]<fragmentLength&&Cutting(source[i])).ToHashSet();
        if(candidates.Count==0)return source.ToArray();
        var nominal=new CutterSweepStock(profile,p=>p,originalMaterialSurface,token);
        var posted=new CutterSweepStock(profile,p=>p,originalMaterialSurface,token);
        double postedTop=(float)originalMaterialSurface-(float)datum;
        Point3D Native(Point3D p)=>new((float)p.X,(float)p.Y,(float)p.Z-(float)originalMaterialSurface);
        Point3D Command(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),ToolpathCommandRounding.Round((float)p.Y),
            ToolpathCommandRounding.Round((float)p.Z-(float)datum)-postedTop);
        long donorChords=0;
        foreach(var path in source.Where((_,i)=>!candidates.Contains(i)).Concat(otherRequired).Where(Cutting)){
            token.ThrowIfCancellationRequested();var cursor=Command(path.Points[0]);var nativeCursor=Native(path.Points[0]);
            for(int i=1;i<path.Points.Count;i++){
                var a=path.Points[i-1];var b=path.Points[i];var next=Command(b);var nativeNext=Native(b);
                if(ToolpathCommandRounding.IsVerticalOnly((float)b.X-(float)a.X,(float)b.Z-(float)a.Z,(float)b.Y-(float)a.Y))
                    {next=new(cursor.X,cursor.Y,next.Z);nativeNext=new(nativeCursor.X,nativeCursor.Y,nativeNext.Z);}
                else {nominal.RecordExecuted(nativeCursor,nativeNext);posted.RecordExecuted(cursor,next);donorChords++;}
                cursor=next;nativeCursor=nativeNext;
            }
        }
        nominal.Seal();posted.Seal();double allowance=finishTolerance/Math.Sqrt(2);int omitted=0;double omittedLength=0;
        var result=new List<TaggedToolpath>();
        for(int owner=0;owner<source.Count;owner++){
            token.ThrowIfCancellationRequested();var path=source[owner];bool covered=candidates.Contains(owner);var cursor=Command(path.Points[0]);var nativeCursor=Native(path.Points[0]);
            for(int i=1;covered&&i<path.Points.Count;i++){
                var a=path.Points[i-1];var b=path.Points[i];
                var next=Command(b);var nativeNext=Native(b);
                if(ToolpathCommandRounding.IsVerticalOnly((float)b.X-(float)a.X,(float)b.Z-(float)a.Z,(float)b.Y-(float)a.Y)){next=new(cursor.X,cursor.Y,next.Z);nativeNext=new(nativeCursor.X,nativeCursor.Y,nativeNext.Z);}
                covered=nominal.Contains(nativeCursor,nativeNext,allowance,-allowance)&&posted.Contains(cursor,next,allowance,-allowance);cursor=next;nativeCursor=nativeNext;
            }
            if(!covered){result.Add(path);continue;}
            omitted++;omittedLength+=lengths[owner];
        }
        log?.Invoke($"stage=ring-fragment-demand candidates={candidates.Count} omitted={omitted} retained={result.Count} omittedLengthMm={omittedLength:R} "
            +$"fragmentLengthBoundMm={fragmentLength:R} donorChords={donorChords} elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} originalMaterialSurfaceZ={originalMaterialSurface:R} finishToleranceMm={finishTolerance:R} "
            +"wholeCutterNativeAndF3=True immutableRetainedDonors=True cyclicOmission=False plannedRemovalOnly=True futureStockOrTravelCredit=False");
        return result.ToArray();
    }
}
