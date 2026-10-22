using Clipper2Lib;
using System.Diagnostics;

namespace RW.Toolpaths.Milling.Acceleration;

// Only the generation-owned RAW candidate invokes this helper. The exact
// finite chain bound is NOM geometry, never filled-set or cutter stock credit.
internal static class RawInitialClearingFrontConditioner
{
    internal const string EnvironmentName="RW_TOOLPATHS_VCARVE_INITIAL_FRONT_CONDITIONING";
    internal const int MinimumFrontPoints=4096;
    internal const int MaximumGridMove=32;
    internal static bool TryCondition(Paths64 raw,CancellationToken token,Action<string>? diagnostic,
        out Paths64? candidate,out string reason)
    {
        token.ThrowIfCancellationRequested();candidate=null;reason="front-conditioning-input-domain";
        long started=Stopwatch.GetTimestamp(),allocation=GC.GetAllocatedBytesForCurrentThread();
        int inputPoints=0,outputPoints=0,reducedRings=0,orientationFallbacks=0,tinyRings=0;
        long verified=0;int replacements=0,refused=0;bool complete=false;
        try
        {
            if(raw is null||raw.Count is<1 or>16384)return false;
            long points=0;foreach(var ring in raw){token.ThrowIfCancellationRequested();if(ring is null||ring.Count<3)return false;points+=ring.Count;}
            if(points>500000)return false;inputPoints=(int)points;
            if(inputPoints<MinimumFrontPoints){reason="front-conditioning-below-large-front-threshold";return false;}
            var result=new Paths64(raw.Count);
            foreach(var ring in raw)
            {
                token.ThrowIfCancellationRequested();
                TimeSpan remaining=TimeSpan.FromSeconds(5)-Stopwatch.GetElapsedTime(started);
                if(remaining<=TimeSpan.Zero){reason="front-conditioning-whole-front-time-budget";return false;}
                var closed=new List<Point64>(ring.Count+1);closed.AddRange(ring);closed.Add(ring[0]);
                var reduced=VCarveGpuFrontTraceReductionNominator.Reduce(closed,true,MaximumGridMove,token,remaining);
                verified+=reduced.VerifiedVertices;replacements+=reduced.ReplacementChords;refused+=reduced.RefusedReplacements;
                if(!reduced.Completed){reason="front-conditioning-whole-RAW-fallback:"+reduced.Reason;return false;}
                var next=new Path64(reduced.Points.Length-1);
                for(int i=0;i<reduced.Points.Length-1;i++){if((i&255)==0)token.ThrowIfCancellationRequested();next.Add(reduced.Points[i]);}
                // Keep every closed ring. Orientation/collapse uncertainty only
                // restores its COMPLETE literal original ring, never deletes it.
                Int128 originalArea=Area2(ring,token),newArea=Area2(next,token);
                if(next.Count<3||newArea==0||(originalArea<0)!=(newArea<0))
                {next=new Path64(ring);orientationFallbacks++;}
                if(ring.Count<=4)tinyRings++;
                if(next.Count<ring.Count)reducedRings++;
                outputPoints+=next.Count;result.Add(next);
            }
            token.ThrowIfCancellationRequested();
            if(Stopwatch.GetElapsedTime(started)>TimeSpan.FromSeconds(5))
            {reason="front-conditioning-whole-front-time-budget";return false;}
            candidate=result;complete=true;reason="exact-finite-chain-NOM-front-conditioned";return true;
        }
        catch(OverflowException){candidate=null;reason="front-conditioning-whole-RAW-arithmetic-fallback";return false;}
        finally
        {
            diagnostic?.Invoke($"stage=raw-initial-front-conditioning complete={complete} reason='{reason}' "
                +$"sourceRings={raw?.Count??0} inputPoints={inputPoints} outputPoints={(complete?outputPoints:inputPoints)} reducedRings={reducedRings} "
                +$"orientationFallbacks={orientationFallbacks} tinyRings={tinyRings} replacementChords={replacements} refusedReplacements={refused} verifiedVertices={verified} "
                +$"maximumNativeMove={MaximumGridMove} minimumFrontPoints={MinimumFrontPoints} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} allocatedBytes={GC.GetAllocatedBytesForCurrentThread()-allocation} "
                +"wholeUnconditionedRAWOnUnsupported=True sourceBoundaryUnchanged=True addedInsetPad=0 closedRingInventoryRetained=True "
                +"filledTopologyCredit=False physicalCredit=False CcoverageCredit=False stockCredit=False originalProtectRequired=True");
            token.ThrowIfCancellationRequested();
        }
    }
    private static Int128 Area2(IReadOnlyList<Point64> ring,CancellationToken token)
    {
        Int128 area=0;for(int i=0,j=ring.Count-1;i<ring.Count;j=i++)
        {if((i&255)==0)token.ThrowIfCancellationRequested();
         if(ring[i].X<=-(1L<<29)||ring[i].X>=(1L<<29)||ring[i].Y<=-(1L<<29)||ring[i].Y>=(1L<<29))throw new OverflowException();
         // At most500k points with |XY|<2^29 give |area|<2^79;
         // ordinary Int128 arithmetic is exact within this admitted domain.
         area=area+(Int128)ring[j].X*ring[i].Y-(Int128)ring[i].X*ring[j].Y;}
        return area;
    }
}
