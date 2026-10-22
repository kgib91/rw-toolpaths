using System.Diagnostics;
using System.Globalization;
using T=RW.Toolpaths.Milling.ExactF3TrajectorySimplifier;

namespace RW.Toolpaths.Milling;

// One serial ApplyCore invocation, after ORIGINAL floor/artwork/channel/target
// owners exist. This is nomination; original Protect and final target proof stay.
internal sealed class VCarveActualF3TrajectoryCompaction
{
    readonly object _invocation=new();
    readonly double _surface;
    readonly Func<Point3D,Point3D> _command;
    readonly Func<TaggedToolpath,T.GuardDecision> _freshOriginalGuards;
    readonly Func<TaggedToolpath,bool> _freshOriginalNominalGrade;
    readonly CancellationToken _token;
    readonly Action<string>? _diagnostic;
    readonly Func<string>? _originalFloorDiagnostics;
    internal long Calls,Paths,SourcePoints,OutputPoints,Proposals,Guarded,LiteralPaths,PointChecks,GuardSplits,GeometrySplits,BoundaryStops;
    internal long FloorRefusals,ArtworkRefusals,ChannelRefusals,GradeRefusals,PostingRefusals;
    internal long NominalGradeChecks,NominalGradeRefusals,ActualGradeRefusals;
    internal double TotalMilliseconds,ConversionMilliseconds,FreshGuardMilliseconds;
    internal VCarveActualF3TrajectoryCompaction(double surface,Func<Point3D,Point3D> command,
        Func<TaggedToolpath,T.GuardDecision> freshOriginalGuards,Func<TaggedToolpath,bool> freshOriginalNominalGrade,
        CancellationToken token,Action<string>? diagnostic,
        Func<string>? originalFloorDiagnostics=null)
    {_surface=surface;_command=command;_freshOriginalGuards=freshOriginalGuards;_freshOriginalNominalGrade=freshOriginalNominalGrade;_token=token;_diagnostic=diagnostic;_originalFloorDiagnostics=originalFloorDiagnostics;}
    internal List<TaggedToolpath> Compact(IReadOnlyList<TaggedToolpath> paths,string stage)
    {
        _token.ThrowIfCancellationRequested();long begin=Stopwatch.GetTimestamp();Calls++;
        var result=new List<TaggedToolpath>(paths.Count);long before=0,after=0;
        for(int i=0;i<paths.Count;i++)
        {
            _token.ThrowIfCancellationRequested();var source=paths[i];Paths++;before+=source.Points.Count;
            var candidate=CompactPath(source,i);after+=candidate.Points.Count;result.Add(candidate);
        }
        TotalMilliseconds+=Stopwatch.GetElapsedTime(begin).TotalMilliseconds;SourcePoints+=before;OutputPoints+=after;
        _diagnostic?.Invoke($"stage=actual-F3-compaction event=production-nomination phase='{stage}' sourcePoints={before} candidatePoints={after} paths={paths.Count} cumulativeCalls={Calls} cumulativeProposals={Proposals} cumulativeGuarded={Guarded} floorRefusals={FloorRefusals} artworkRefusals={ArtworkRefusals} channelRefusals={ChannelRefusals} gradeRefusals={GradeRefusals} actualGradeRefusals={ActualGradeRefusals} nominalGradeChecks={NominalGradeChecks} nominalGradeRefusals={NominalGradeRefusals} returnedOriginalNominalGradeRequired=True postingRefusals={PostingRefusals} conversionMs={ConversionMilliseconds:R} freshGuardMs={FreshGuardMilliseconds:R} totalMs={TotalMilliseconds:R} invocationAndAllSpansRetained=True numericFeedInvented=False originalFloorChannelTargetUnchanged=True finalProtectRequired=True wholeTargetRequired=True stockAuditPending=True");
        if(_originalFloorDiagnostics is not null)_diagnostic?.Invoke(_originalFloorDiagnostics());
        _token.ThrowIfCancellationRequested();return result;
    }
    TaggedToolpath CompactPath(TaggedToolpath path,int pathOrdinal)
    {
        if(path.Points.Count<3||path.Points.Count>T.MaximumInputPoints||path.DetectCurves||!double.IsFinite(_surface))
        {LiteralPaths++;return path;}
        // Snapshot the COMPLETE path. Proposals never alias mutable source or
        // reconstructed nominal points; only original selected ordinals return.
        var source=path.Points.ToArray();var spans=path.Spans.ToArray();
        var ticks=new T.Tick3[source.Length];var hard=new bool[source.Length];hard[1]=true;
        var spanOrdinals=new int[source.Length-1];Array.Fill(spanOrdinals,-1);
        var kinds=new ToolpathSpanKind[spanOrdinals.Length];Array.Fill(kinds,ToolpathSpanKind.Cut);
        var keys=new T.BoundaryKey[spanOrdinals.Length];
        for(int s=0;s<spans.Length;s++)
        {
            _token.ThrowIfCancellationRequested();var span=spans[s];
            if(span.StartIndex<0||span.EndIndex<span.StartIndex||span.EndIndex>=source.Length){LiteralPaths++;return path;}
            hard[span.StartIndex]=hard[span.EndIndex]=true;
            for(int e=span.StartIndex;e<span.EndIndex;e++)
            {
                if((e&255)==0)_token.ThrowIfCancellationRequested();
                // The post allows last-span ownership; a malformed overlap is
                // conservatively literal here, with no reinterpretation.
                if(spanOrdinals[e]>=0){LiteralPaths++;return path;}
                spanOrdinals[e]=s;kinds[e]=span.Kind;
            }
        }
        long conversion=Stopwatch.GetTimestamp();
        try
        {
            for(int i=0;i<source.Length;i++)
            {
                if((i&255)==0)_token.ThrowIfCancellationRequested();
                if(!TryTicks(_command(source[i]),out ticks[i])){LiteralPaths++;return path;}
            }
        }
        finally{ConversionMilliseconds+=Stopwatch.GetElapsedTime(conversion).TotalMilliseconds;}
        for(int e=0;e<keys.Length;e++)
        {
            if((e&255)==0)_token.ThrowIfCancellationRequested();int mode=MotionMode(source[e],source[e+1],_surface);
            if(mode is <0 or >1){hard[e+1]=true;if(e+2<hard.Length)hard[e+2]=true;}
            keys[e]=new(_invocation,pathOrdinal,path.RegionIndex,path.DepthPassIndex,path.VisitIdentity,
                spanOrdinals[e],kinds[e],path.Category,mode==0?1:0,mode==0?7:4);
        }
        var nomination=T.Simplify(ticks,keys,hard,Prove,_token,diagnostic:null);
        PointChecks+=nomination.Counts.PointChecks;GuardSplits+=nomination.Counts.GuardSplits;
        GeometrySplits+=nomination.Counts.GeometrySplits;BoundaryStops+=nomination.Counts.BoundaryStops;
        if(!nomination.CompletedNomination||!nomination.Changed){LiteralPaths++;return path;}
        var ordinals=nomination.OriginalOrdinals;var selected=ordinals.Select(i=>source[i]).ToArray();
        var map=ordinals.Select((ordinal,i)=>(ordinal,i)).ToDictionary(p=>p.ordinal,p=>p.i);
        var outputSpans=spans.Select(s=>new ToolpathSpan(map[s.StartIndex],map[s.EndIndex],s.Kind)).ToArray();
        for(int i=0;i<selected.Length;i++)
        {
            if((i&255)==0)_token.ThrowIfCancellationRequested();
            if(!TryTicks(_command(selected[i]),out var command)||command!=nomination.ActualPoses[i])
                throw new InvalidOperationException("actual-F3-compaction-original-endpoint-closure");
        }
        _token.ThrowIfCancellationRequested();return new(selected,outputSpans,path.RegionIndex,path.Category,
            path.DepthPassIndex,path.VisitIdentity,detectCurves:false);
        T.GuardDecision Prove(T.Proposal proposal)
        {
            _token.ThrowIfCancellationRequested();Proposals++;
            if(!ReferenceEquals(proposal.Boundary.InvocationOwner,_invocation)||proposal.Boundary.Path!=pathOrdinal
                ||proposal.Boundary.Region!=path.RegionIndex||proposal.Boundary.Pass!=path.DepthPassIndex
                ||proposal.Boundary.Visit!=path.VisitIdentity||proposal.Boundary.Category!=path.Category
                ||!proposal.Boundary.Eligible)
                throw new InvalidOperationException("actual-F3-compaction-invocation-path-boundary-custody");
            Point3D a=Decode(proposal.A),b=Decode(proposal.B);
            bool posting=TryTicks(_command(a),out var ca)&&ca==proposal.A&&TryTicks(_command(b),out var cb)&&cb==proposal.B
                &&MotionMode(source[proposal.First],source[proposal.Last],_surface)==0;
            if(!posting){PostingRefusals++;return new(false,false,false,false,false,"actual-F3-fixed-point-or-current-post-motion");}
            long start=Stopwatch.GetTimestamp();T.GuardDecision proof;
            try
            {
                var actual=new TaggedToolpath(new[]{a,b},Array.Empty<ToolpathSpan>(),path.RegionIndex,
                    path.Category,path.DepthPassIndex,path.VisitIdentity,detectCurves:false);
                proof=_freshOriginalGuards(actual);
                _token.ThrowIfCancellationRequested();
                if(!proof.AuthoredGrade)ActualGradeRefusals++;
                // Output selects these ORIGINAL nominal endpoints, not Decode's
                // posted poses. Both spaces require the SAME original grade.
                var nominal=new TaggedToolpath(new[]{source[proposal.First],source[proposal.Last]},
                    Array.Empty<ToolpathSpan>(),path.RegionIndex,path.Category,path.DepthPassIndex,
                    path.VisitIdentity,detectCurves:false);
                NominalGradeChecks++;
                bool nominalGrade=_freshOriginalNominalGrade(nominal);
                _token.ThrowIfCancellationRequested();
                if(!nominalGrade)
                {
                    NominalGradeRefusals++;
                    if(NominalGradeRefusals<=4)
                        _diagnostic?.Invoke($"stage=actual-F3-compaction event=nominal-grade-refusal path={pathOrdinal} first={proposal.First} last={proposal.Last} actualGradePassed={proof.AuthoredGrade} nominalFrom=({nominal.Points[0].X:R},{nominal.Points[0].Y:R},{nominal.Points[0].Z:R}) nominalTo=({nominal.Points[1].X:R},{nominal.Points[1].Y:R},{nominal.Points[1].Z:R}) actualFrom=({a.X:R},{a.Y:R},{a.Z:R}) actualTo=({b.X:R},{b.Y:R},{b.Z:R}) originalSelectedOrdinalOwner=True noGradeEpsilonChanged=True failedProposalNotPublished=True");
                    _token.ThrowIfCancellationRequested();
                }
                proof=new(proof.OriginalFloor,proof.ArtworkWall,proof.OriginalChannelWall,
                    proof.AuthoredGrade&&nominalGrade,proof.ExactActualEndpoints,
                    nominalGrade?proof.Reason:"original-selected-nominal-grade");
            }
            finally{FreshGuardMilliseconds+=Stopwatch.GetElapsedTime(start).TotalMilliseconds;}
            if(!proof.OriginalFloor)FloorRefusals++;if(!proof.ArtworkWall)ArtworkRefusals++;
            if(!proof.OriginalChannelWall)ChannelRefusals++;if(!proof.AuthoredGrade)GradeRefusals++;
            if(!proof.ExactActualEndpoints)PostingRefusals++;if(proof.Passed)Guarded++;
            if(!proof.Passed&&FloorRefusals+ArtworkRefusals+ChannelRefusals+GradeRefusals+PostingRefusals<=16)
                _diagnostic?.Invoke($"stage=actual-F3-compaction event=production-guard-refusal path={pathOrdinal} first={proposal.First} last={proposal.Last} reason='{proof.Reason}' floor={proof.OriginalFloor} artwork={proof.ArtworkWall} originalChannel={proof.OriginalChannelWall} grade={proof.AuthoredGrade} actualEndpoints={proof.ExactActualEndpoints} failedProposalNotPublished=True originalSubdivisionCustodyRetained=True");
            _token.ThrowIfCancellationRequested();return proof;
        }
    }
    internal static Point3D Decode(T.Tick3 point)=>new(point.X/1000d,point.Y/1000d,point.Z/1000d);
    internal static bool TryTicks(Point3D point,out T.Tick3 ticks)
    {
        ticks=default;
        if(!TryAxis(point.X,out long x)||!TryAxis(point.Y,out long y)||!TryAxis(point.Z,out long z))return false;
        ticks=new(x,y,z);return true;
        static bool TryAxis(double value,out long ticks)
        {
            ticks=0;if(!double.IsFinite(value)||Math.Abs(value)>T.MaximumAxisTicks/1000d)return false;
            decimal exact=decimal.Parse(value.ToString("F3",CultureInfo.InvariantCulture),CultureInfo.InvariantCulture)*1000;
            if(exact!=decimal.Truncate(exact))return false;ticks=checked((long)exact);return true;
        }
    }
    internal static int MotionMode(Point3D from,Point3D to,double surface)
    {
        // Literal CURRENT Shared ToolpathMotionClassifier/IlProgramBuilder:
        // world-float classify BEFORE F3; stock-surface rapid is strict>=surface.
        float ax=(float)from.X,ay=(float)from.Y,az=(float)(surface+from.Z);
        float bx=(float)to.X,by=(float)to.Y,bz=(float)(surface+to.Z);
        float dx=bx-ax,dy=by-ay,dz=bz-az;float run=MathF.Sqrt(dx*dx+dy*dy);
        if(!float.IsFinite(ax)||!float.IsFinite(ay)||!float.IsFinite(az)||!float.IsFinite(bx)||!float.IsFinite(by)||!float.IsFinite(bz)
            ||!float.IsFinite(run)||!float.IsFinite(dz)||!float.IsFinite((float)surface))return -1;
        if(run>.001f&&MathF.Min(az,bz)>=(float)surface)return 1;
        if(MathF.Abs(dz)>.001f&&run<=.001f)return dz>0?2:bz>=(float)surface?3:4;
        return 0;
    }
}
