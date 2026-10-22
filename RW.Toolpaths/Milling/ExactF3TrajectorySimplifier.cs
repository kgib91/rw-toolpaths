using System.Collections.Immutable;

namespace RW.Toolpaths.Milling;

// DEFAULT-OFF NOMINATION. Input is a COMPLETE actual F3 XYZ linear-feed chain,
// not nominal/native points. New chords remain NOM until the caller's fresh
// original-floor, artwork-wall, original-channel-wall and grade proofs pass.
// No source used to construct those immutable owners is simplified here.
internal static class ExactF3TrajectorySimplifier
{
    internal const int RadiusTicks=2; // 0.002 mm in literal F3 axis ticks.
    internal const long MaximumAxisTicks=10_000_000; // |axis| <= 10,000 mm.
    internal const int MaximumInputPoints=262_144;
    internal const int BlockEdges=128;
    internal readonly record struct Tick3(long X,long Y,long Z);
    internal readonly record struct BoundaryKey(object InvocationOwner,int Path,int Region,int? Pass,int? Visit,
        int Span,ToolpathSpanKind SpanKind,string Category,int Motion,int CommandAxes)
    {
        // Native V-carving has no resolved UUID/feed values. One actual Apply
        // invocation owns the unchanged tool and operation posting context.
        // Shared VCarve XYZ Move uses that operation's feedRate; every original
        // span and kind remains a hard boundary, including implicit Cut gaps.
        // Rapid, vertical/partial-axis, tab, retract and curve owners stay literal.
        internal bool Eligible=>InvocationOwner is not null&&Motion==1&&CommandAxes==7
            &&SpanKind is ToolpathSpanKind.Cut or ToolpathSpanKind.Ramp or ToolpathSpanKind.Link
            &&!string.IsNullOrEmpty(Category);
    }
    internal readonly record struct Proposal(int First,int Last,Tick3 A,Tick3 B,BoundaryKey Boundary,
        UInt128 LengthSquared,UInt128 MaximumCrossSquared);
    internal readonly record struct GuardDecision(bool OriginalFloor,bool ArtworkWall,bool OriginalChannelWall,
        bool AuthoredGrade,bool ExactActualEndpoints,string Reason)
    {
        internal bool Passed=>OriginalFloor&&ArtworkWall&&OriginalChannelWall&&AuthoredGrade&&ExactActualEndpoints;
    }
    internal sealed class Metrics
    {
        internal long SourcePoints,OutputPoints,RangeChecks,PointChecks,ProposedChords,GuardedChords,
            GeometrySplits,GuardSplits,BoundaryStops,LiteralEdges,RefusedInputs;
        internal int MaximumStack,MaximumRangeEdges;
        internal string FirstRefusal="";
    }
    internal sealed record Result(bool CompletedNomination,bool Changed,string Reason,
        ImmutableArray<int> OriginalOrdinals,ImmutableArray<Tick3> ActualPoses,
        ImmutableArray<Proposal> QualifiedShortcuts,Metrics Counts)
    {
        internal bool FinishedStockCertified=>false;
        internal bool OriginalFloorSourcesChanged=>false;
        internal bool AuthoredScheduleChanged=>false;
    }
    private readonly record struct Range(int First,int Last);
    private static bool InDomain(Tick3 p)=>p.X>=-MaximumAxisTicks&&p.X<=MaximumAxisTicks
        &&p.Y>=-MaximumAxisTicks&&p.Y<=MaximumAxisTicks&&p.Z>=-MaximumAxisTicks&&p.Z<=MaximumAxisTicks;
    private static UInt128 Square(Int128 x){UInt128 u=x<0?(UInt128)(-x):(UInt128)x;return u*u;}

    // Complete symmetric trajectory correspondence: endpoint projections are
    // exact and nondecreasing in [0,D.D]. Every source segment stays inside the
    // convex radius2 tube of the finite chord. Its continuous projection covers
    // that whole chord, so every chord point ALSO has a source point <=2ticks.
    // With the coordinate bound above cross products are <=8e14, and the sum
    // of their squares is <2e30 <2^128. All arithmetic below is exact integer.
    internal static bool TryProveRange(ReadOnlySpan<Tick3> points,int first,int last,
        out UInt128 lengthSquared,out UInt128 maximumCrossSquared,out int failureOrdinal,
        Metrics? counts=null,Action? checkpoint=null)
    {
        lengthSquared=maximumCrossSquared=0;failureOrdinal=first;
        if(first<0||last<=first||last>=points.Length||last-first>BlockEdges)return false;
        Tick3 a=points[first],b=points[last];if(!InDomain(a)||!InDomain(b))return false;
        Int128 dx=b.X-a.X,dy=b.Y-a.Y,dz=b.Z-a.Z;
        lengthSquared=Square(dx)+Square(dy)+Square(dz);
        if(lengthSquared==0)return false; // Never authorize a collapsed loop.
        Int128 previous=0;UInt128 threshold=(UInt128)(RadiusTicks*RadiusTicks)*lengthSquared;
        for(int i=first+1;i<last;i++)
        {
            checkpoint?.Invoke();if(counts is not null)counts.PointChecks++;
            Tick3 p=points[i];if(!InDomain(p)){failureOrdinal=i;return false;}
            Int128 x=p.X-a.X,y=p.Y-a.Y,z=p.Z-a.Z,dot=x*dx+y*dy+z*dz;
            if(dot<previous||dot<0||(UInt128)dot>lengthSquared){failureOrdinal=i;return false;}
            previous=dot;
            UInt128 cross=Square(y*dz-z*dy)+Square(z*dx-x*dz)+Square(x*dy-y*dx);
            if(cross>maximumCrossSquared)maximumCrossSquared=cross;
            if(cross>threshold){failureOrdinal=i;return false;}
        }
        failureOrdinal=-1;return true;
    }

    internal static Result Simplify(Tick3[] source,BoundaryKey[] edgeOwners,bool[] hardAnchors,
        Func<Proposal,GuardDecision> freshOriginalGuards,CancellationToken token,
        Action? sharedCheckpoint=null,Action<string>? diagnostic=null)
    {
        ArgumentNullException.ThrowIfNull(source);ArgumentNullException.ThrowIfNull(edgeOwners);
        ArgumentNullException.ThrowIfNull(hardAnchors);ArgumentNullException.ThrowIfNull(freshOriginalGuards);
        token.ThrowIfCancellationRequested();var m=new Metrics{SourcePoints=source.Length};
        if(edgeOwners.Length!=Math.Max(0,source.Length-1)||hardAnchors.Length!=source.Length)
            throw new ArgumentException("complete-actual-F3-edge-and-anchor-ownership-required");
        if(source.Length>MaximumInputPoints)
        {
            // Refusal carries NO replacement geometry and allocates no copy
            // beyond the cap. The caller must retain its literal input chain.
            Check();m.RefusedInputs++;m.OutputPoints=source.Length;m.FirstRefusal="bounded-source-input-refusal";
            diagnostic?.Invoke($"stage=actual-F3-compaction event=literal-fallback reason='{m.FirstRefusal}' sourcePoints={source.Length} replacementPublished=False stockCredit=0");Check();
            return new(false,false,m.FirstRefusal,ImmutableArray<int>.Empty,ImmutableArray<Tick3>.Empty,ImmutableArray<Proposal>.Empty,m);
        }
        // Own one bounded snapshot; no mutable caller alias enters a proposal.
        var points=(Tick3[])source.Clone();var keys=(BoundaryKey[])edgeOwners.Clone();var hard=(bool[])hardAnchors.Clone();
        for(int i=0;i<points.Length;i++){if((i&255)==0)Check();if(!InDomain(points[i]))return Literal("exact-integer-coordinate-domain-refusal");}
        if(points.Length<3)return Literal("fewer-than-three-actual-positions");
        hard[0]=hard[^1]=true;
        var ordinals=new List<int>(points.Length){0};var admitted=new List<Proposal>();
        var stack=new Stack<Range>(BlockEdges);int start=0,logged=0;
        while(start<points.Length-1)
        {
            Check();var key=keys[start];int end=start+1;
            if(key.Eligible)
                while(end<points.Length-1&&end-start<BlockEdges&&!hard[end]&&keys[end]==key&&keys[end].Eligible)end++;
            else m.BoundaryStops++;
            if(end<points.Length-1&&(hard[end]||keys[end]!=key))m.BoundaryStops++;
            stack.Push(new(start,end));
            while(stack.Count>0)
            {
                Check();m.MaximumStack=Math.Max(m.MaximumStack,stack.Count);Range range=stack.Pop();
                int edges=range.Last-range.First;m.MaximumRangeEdges=Math.Max(m.MaximumRangeEdges,edges);
                if(edges==1){ordinals.Add(range.Last);m.LiteralEdges++;continue;}
                m.RangeChecks++;
                bool geometry=TryProveRange(points,range.First,range.Last,out var len,out var cross,out int failure,m,Check);
                if(geometry)
                {
                    var proposal=new Proposal(range.First,range.Last,points[range.First],points[range.Last],key,len,cross);
                    m.ProposedChords++;GuardDecision proof=freshOriginalGuards(proposal);Check();
                    if(proof.Passed){m.GuardedChords++;admitted.Add(proposal);ordinals.Add(range.Last);continue;}
                    m.GuardSplits++;if(m.FirstRefusal.Length==0)m.FirstRefusal="fresh-original-guards:"+proof.Reason;
                    if(logged++<16)diagnostic?.Invoke($"stage=actual-F3-compaction event=guard-refusal first={range.First} last={range.Last} reason='{proof.Reason}' floor={proof.OriginalFloor} artwork={proof.ArtworkWall} originalChannel={proof.OriginalChannelWall} grade={proof.AuthoredGrade} actualEndpoints={proof.ExactActualEndpoints} radiusTicks={RadiusTicks} lengthSquared={len} maxCrossSquared={cross}");
                    failure=range.First+edges/2;
                }
                else{m.GeometrySplits++;if(failure<=range.First||failure>=range.Last)failure=range.First+edges/2;}
                // Both children retain every original position and boundary.
                // Fresh guards run again; rejected evidence is never reused.
                stack.Push(new(failure,range.Last));stack.Push(new(range.First,failure));
            }
            start=end;
        }
        Check();m.OutputPoints=ordinals.Count;
        diagnostic?.Invoke($"stage=actual-F3-compaction event=complete sourcePoints={m.SourcePoints} outputPoints={m.OutputPoints} guardedShortcuts={m.GuardedChords} pointChecks={m.PointChecks} geometrySplits={m.GeometrySplits} guardSplits={m.GuardSplits} boundaryStops={m.BoundaryStops} maxRangeEdges={m.MaximumRangeEdges} maxStack={m.MaximumStack} radiusTicks={RadiusTicks} trajectorySymmetricBoundMm=0.002 translatedFullBodyHausdorffMm=0.002 stockHeightCertified=False sourceFloorImmutable=True authoredScheduleChanged=False");
        Check();return new(true,ordinals.Count!=points.Length,"fresh-guarded-actual-F3-nomination",
            ordinals.ToImmutableArray(),ordinals.Select(i=>points[i]).ToImmutableArray(),admitted.ToImmutableArray(),m);
        void Check(){token.ThrowIfCancellationRequested();sharedCheckpoint?.Invoke();token.ThrowIfCancellationRequested();}
        Result Literal(string reason)
        {
            Check();m.RefusedInputs++;m.OutputPoints=source.Length;m.FirstRefusal=reason;
            diagnostic?.Invoke($"stage=actual-F3-compaction event=literal-fallback reason='{reason}' sourcePoints={source.Length} stockCredit=0");Check();
            return new(false,false,reason,Enumerable.Range(0,source.Length).ToImmutableArray(),source.ToImmutableArray(),ImmutableArray<Proposal>.Empty,m);
        }
    }
}
