using Clipper2Lib;
using System.Diagnostics;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling.Acceleration;

// Captured once by the eligible smooth V-carve GENERATION owner. No public
// standalone clearing API accepts this capability. Returned geometry is only
// a first-layer NOM; it never enters CertifiedInsetReuse or the medial/floor.
internal sealed class RawInitialClearingInsetNomination
{
    internal const string EnvironmentName="RW_TOOLPATHS_VCARVE_RAW_INITIAL_INSET_NOMINATION";
    internal const string Policy="full-original-source-initial-clearing-RAW-compensated-C-round-1tick-NOM-v1";
    private readonly bool _conditionInitialFront;
    private readonly bool _supplementalInitialBand;
    private readonly bool _collectorNomination;
    private RawInitialClearingInsetNomination(bool conditionInitialFront=false,bool supplementalInitialBand=false,bool collectorNomination=false){_conditionInitialFront=conditionInitialFront;_supplementalInitialBand=supplementalInitialBand;_collectorNomination=collectorNomination;}
    internal bool FrontConditioningCaptured=>_conditionInitialFront;
    internal bool SupplementalInitialBandCaptured=>_supplementalInitialBand;
    internal bool CollectorNominationCaptured=>_collectorNomination;
    internal RawInitialClearingInsetNomination OriginalCollectorFallback()=>new(_conditionInitialFront,_supplementalInitialBand,false);
    internal static RawInitialClearingInsetNomination? CaptureForSmoothGeneration(bool smooth,double validatedRampAngleDegrees,bool requested,bool conditionInitialFront=false)
    {
        if (!requested || !smooth || !double.IsFinite(validatedRampAngleDegrees)
            || validatedRampAngleDegrees <= 0 || validatedRampAngleDegrees >= 90) return null;
        var policy = VCarveGenerationPolicy.Capture();
        return new(conditionInitialFront, policy.SupplementalInitialBand, policy.InitialStockCohort);
    }

    internal static RawInitialClearingInsetNomination? CaptureForStrategy(bool smooth,double validatedRampAngleDegrees,VCarveInitialStrategy strategy)
        =>strategy.RawInitialInset&&smooth&&double.IsFinite(validatedRampAngleDegrees)&&validatedRampAngleDegrees>0&&validatedRampAngleDegrees<90
            ?new(strategy.InitialFrontConditioning,strategy.SupplementalInitialBand,strategy.InitialStockCohort):null;

    internal bool TryNominate(IReadOnlyList<IReadOnlyList<PointD>> source,double radius,CancellationToken token,Action<string>? diagnostic,
        out List<List<PointD>>? candidate,out string reason)
    {
        token.ThrowIfCancellationRequested();candidate=null;reason="raw-initial-inset-source-domain";
        long started=Stopwatch.GetTimestamp(),allocationStart=GC.GetAllocatedBytesForCurrentThread();int sourcePoints=0;
        int canonicalPoints=0,rawPoints=0,rawRings=0;double reserve=double.NaN,construction=double.NaN;string status="unsupported";
        try
        {
            if(source is null||source.Count is<1 or>16384||!double.IsFinite(radius)||radius<=0||radius>100)return false;
            double coordinate=0;var literal=new Paths64(source.Count);
            foreach(var ring in source)
            {
                token.ThrowIfCancellationRequested();if(ring is null||ring.Count<3||(long)sourcePoints+ring.Count>500000)return false;
                var row=new Path64(ring.Count);
                foreach(var p in ring)
                {
                    if((sourcePoints++&255)==0)token.ThrowIfCancellationRequested();
                    if(!double.IsFinite(p.x)||!double.IsFinite(p.y)||Math.Abs(p.x*PathUtils.Scale)>=(1L<<29)-1||Math.Abs(p.y*PathUtils.Scale)>=(1L<<29)-1)return false;
                    coordinate=Math.Max(coordinate,Math.Max(Math.Abs(p.x),Math.Abs(p.y)));
                    row.Add(new((long)Math.Round(p.x*PathUtils.Scale),(long)Math.Round(p.y*PathUtils.Scale)));
                }
                literal.Add(row);
            }
            // EXACT literal CutterInset2253 reserve arithmetic. The level is C,
            // never an invented contour/stock tolerance or changed physicalP.
            double quantum=1/PathUtils.Scale;float commandCoordinate=(float)coordinate;
            double commandError=Math.Sqrt(2)*(MathF.BitIncrement(commandCoordinate)-commandCoordinate)/2;
            double machineXYError=Math.Sqrt(2)*.0005;
            reserve=Math.Max(8*quantum,commandError+machineXYError+6*quantum);construction=radius+reserve;
            if(!float.IsFinite(commandCoordinate)||!double.IsFinite(construction))return false;
            token.ThrowIfCancellationRequested();var canonical=Clipper.Union(literal,FillRule.NonZero);token.ThrowIfCancellationRequested();
            canonicalPoints=canonical.Sum(r=>r.Count);if(canonical.Count==0||canonicalPoints>500000){reason="raw-initial-empty-or-unsupported-source-fallback";return false;}
            // Full canonical ORIGINAL vertices: no RDP, bounded-source
            // conditioning, inversion removal, component-area cutoff, or proof.
            var offsetter=new ClipperOffset(miterLimit:PolygonOps.DefaultMiterLimit,arcTolerance:1,preserveCollinear:false,reverseSolution:false);
            offsetter.AddPaths(canonical,JoinType.Round,EndType.Polygon);var raw=new Paths64();
            token.ThrowIfCancellationRequested();offsetter.Execute(-construction*PathUtils.Scale,raw);token.ThrowIfCancellationRequested();
            rawRings=raw.Count;long count=raw.Sum(r=>(long)r.Count);if(count>500000){reason="raw-initial-output-point-domain-fallback";return false;}rawPoints=(int)count;
            // Empty RAW is not an empty-stock proof. Delegate the WHOLE
            // original source to the old Contract, without deleting tiny loops.
            if(raw.Count==0||raw.Any(r=>r.Count<3)){reason="raw-initial-empty-or-degenerate-NOM-needs-original-fallback";return false;}
            int checkedPoints=0;
            foreach(var ring in raw)foreach(var p in ring)
            {if((checkedPoints++&255)==0)token.ThrowIfCancellationRequested();if(p.X<=-(1L<<29)||p.X>=(1L<<29)||p.Y<=-(1L<<29)||p.Y>=(1L<<29)){reason="raw-initial-output-coordinate-domain-fallback";return false;}}
            if(_conditionInitialFront&&RawInitialClearingFrontConditioner.TryCondition(raw,token,diagnostic,out var conditioned,out _))raw=conditioned!;
            candidate=PolygonOps.MaterializeIntegerPaths(raw);token.ThrowIfCancellationRequested();status="nominated";reason="RAW-first-inset-NOM-requires-original-smoothing-floor-artwork-channel-grade";return true;
        }
        catch(OverflowException){candidate=null;reason="raw-initial-bounded-arithmetic-fallback";return false;}
        finally
        {
            diagnostic?.Invoke($"stage=raw-initial-clearing-inset policy={Policy} status={status} reason='{reason}' radiusP={radius:R} reserve={reserve:R} constructionC={construction:R} constructionBits={BitConverter.DoubleToInt64Bits(construction)} sourceRings={source?.Count??0} sourcePoints={sourcePoints} canonicalSourcePoints={canonicalPoints} rawRings={rawRings} rawPoints={rawPoints} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} allocatedBytes={GC.GetAllocatedBytesForCurrentThread()-allocationStart} sourceReduction=False nativeArcTicks=1 originalSourceFallbackOnUnsupported=True certifiedInsetCacheCredit=False physicalCredit=False CcoverageCredit=False stockCredit=False originalMedialFloorUnchanged=True downstreamOriginalProtectRequired=True");
            token.ThrowIfCancellationRequested();
        }
    }

    internal const string BandPolicy="full-original-source-initial-clearing-RAW-exact-authored-band-round-1tick-NOM-v2";

    // Same generation-only captured capability as the qualified first-inset
    // arm. Width is the literal authored geometric cutoff, NOT P plus reserve.
    internal bool TryNominateAuthoredBand(IReadOnlyList<IReadOnlyList<PointD>> source,double authoredWidth,
        double downstreamPhysicalRadius,CancellationToken token,Action<string>? diagnostic,
        out List<List<PointD>>? candidate,out string reason)
    {
        token.ThrowIfCancellationRequested();candidate=null;reason="raw-authored-band-source-domain";
        long started=Stopwatch.GetTimestamp(),allocationStart=GC.GetAllocatedBytesForCurrentThread();
        int sourcePoints=0,canonicalPoints=0,rawPoints=0,rawRings=0;string status="unsupported";
        try
        {
            if(source is null||source.Count is<1 or>16384||!double.IsFinite(authoredWidth)||authoredWidth<=0||authoredWidth>100
                ||!double.IsFinite(downstreamPhysicalRadius)||downstreamPhysicalRadius<=0||downstreamPhysicalRadius>=authoredWidth)return false;
            var literal=new Paths64(source.Count);
            foreach(var ring in source)
            {
                token.ThrowIfCancellationRequested();if(ring is null||ring.Count<3||(long)sourcePoints+ring.Count>500000)return false;
                var row=new Path64(ring.Count);
                foreach(var p in ring)
                {
                    if((sourcePoints++&255)==0)token.ThrowIfCancellationRequested();
                    if(!double.IsFinite(p.x)||!double.IsFinite(p.y)||Math.Abs(p.x*PathUtils.Scale)>=(1L<<29)-1||Math.Abs(p.y*PathUtils.Scale)>=(1L<<29)-1)return false;
                    row.Add(new((long)Math.Round(p.x*PathUtils.Scale),(long)Math.Round(p.y*PathUtils.Scale)));
                }
                literal.Add(row);
            }
            token.ThrowIfCancellationRequested();var canonical=Clipper.Union(literal,FillRule.NonZero);token.ThrowIfCancellationRequested();
            canonicalPoints=canonical.Sum(r=>r.Count);
            if(canonical.Count==0||canonicalPoints>500000){reason="raw-authored-band-empty-or-unsupported-source-fallback";return false;}
            var offsetter=new ClipperOffset(miterLimit:PolygonOps.DefaultMiterLimit,arcTolerance:1,preserveCollinear:false,reverseSolution:false);
            offsetter.AddPaths(canonical,JoinType.Round,EndType.Polygon);var raw=new Paths64();
            token.ThrowIfCancellationRequested();offsetter.Execute(-authoredWidth*PathUtils.Scale,raw);token.ThrowIfCancellationRequested();
            rawRings=raw.Count;long count=raw.Sum(r=>(long)r.Count);
            if(count>500000){reason="raw-authored-band-output-point-domain-fallback";return false;}rawPoints=(int)count;
            // Entire old band pipeline owns unsupported cases, including its
            // original terminal proof. No RAW empty/tiny stock claim or deletion.
            if(raw.Count==0||raw.Any(r=>r.Count<3)){reason="raw-authored-band-empty-or-degenerate-NOM-whole-band-fallback";return false;}
            int checkedPoints=0;
            foreach(var ring in raw)foreach(var p in ring)
            {if((checkedPoints++&255)==0)token.ThrowIfCancellationRequested();if(p.X<=-(1L<<29)||p.X>=(1L<<29)||p.Y<=-(1L<<29)||p.Y>=(1L<<29))
                {reason="raw-authored-band-output-coordinate-domain-fallback";return false;}}
            if(_conditionInitialFront&&RawInitialClearingFrontConditioner.TryCondition(raw,token,diagnostic,out var conditioned,out _))raw=conditioned!;
            candidate=PolygonOps.MaterializeIntegerPaths(raw);token.ThrowIfCancellationRequested();
            status="nominated";reason="RAW-authored-band-NOM-requires-original-smoothing-floor-artwork-channel-grade";return true;
        }
        catch(OverflowException){candidate=null;reason="raw-authored-band-bounded-arithmetic-whole-band-fallback";return false;}
        finally
        {
            diagnostic?.Invoke($"stage=raw-initial-clearing-band policy={BandPolicy} status={status} reason='{reason}' "
                +$"authoredWidth={authoredWidth:R} widthBits={BitConverter.DoubleToInt64Bits(authoredWidth)} addedWidthReserve=0 downstreamPhysicalRadius={downstreamPhysicalRadius:R} "
                +$"sourceRings={source?.Count??0} sourcePoints={sourcePoints} canonicalSourcePoints={canonicalPoints} rawRings={rawRings} rawPoints={rawPoints} "
                +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} allocatedBytes={GC.GetAllocatedBytesForCurrentThread()-allocationStart} "
                +"sourceReduction=False nativeArcTicks=1 unsupportedWholeOriginalBandFallback=True terminalSameCoreCandidate=True "
                +"certifiedInsetCacheCredit=False physicalCredit=False filledSetCredit=False CcoverageCredit=False stockCredit=False originalMedialFloorUnchanged=True downstreamOriginalProtectRequired=True");
            token.ThrowIfCancellationRequested();
        }
    }

    internal const string ReachContourPolicy="full-original-source-supplemental-preserved-inner-reach-RAW-contour-round-1tick-NOM-v2";

    // Wnom only proposes inner stock-reach centers; saved W and physical
    // floor/artwork/channel/grade owners remain unchanged.
    internal bool TryNominateReachContour(IReadOnlyList<IReadOnlyList<PointD>> source,double authoredWidth,
        double fullPhysicalRadius,double supplementalPhysicalRadius,CancellationToken token,Action<string>? diagnostic,
        out List<List<PointD>>? candidate,out string reason)
    {
        token.ThrowIfCancellationRequested();candidate=null;reason="raw-supplemental-reach-contour-source-domain";
        long started=Stopwatch.GetTimestamp(),allocationStart=GC.GetAllocatedBytesForCurrentThread();
        double radiusDelta=fullPhysicalRadius-supplementalPhysicalRadius;
        double nominationWidth=authoredWidth+radiusDelta;
        int sourcePoints=0,canonicalPoints=0,rawPoints=0,rawRings=0;string status="unsupported";
        try
        {
            if(source is null||source.Count is<1 or>16384||!double.IsFinite(authoredWidth)||authoredWidth<=0||authoredWidth>100
                ||!double.IsFinite(fullPhysicalRadius)||fullPhysicalRadius<=0||fullPhysicalRadius>=authoredWidth
                ||!double.IsFinite(supplementalPhysicalRadius)||supplementalPhysicalRadius<=0||supplementalPhysicalRadius>fullPhysicalRadius
                ||!double.IsFinite(nominationWidth)||nominationWidth<=0||nominationWidth>100)return false;
            var literal=new Paths64(source.Count);
            foreach(var ring in source)
            {
                token.ThrowIfCancellationRequested();if(ring is null||ring.Count<3||(long)sourcePoints+ring.Count>500000)return false;
                var row=new Path64(ring.Count);
                foreach(var p in ring)
                {
                    if((sourcePoints++&255)==0)token.ThrowIfCancellationRequested();
                    if(!double.IsFinite(p.x)||!double.IsFinite(p.y)||Math.Abs(p.x*PathUtils.Scale)>=(1L<<29)-1||Math.Abs(p.y*PathUtils.Scale)>=(1L<<29)-1)return false;
                    row.Add(new((long)Math.Round(p.x*PathUtils.Scale),(long)Math.Round(p.y*PathUtils.Scale)));
                }
                literal.Add(row);
            }
            token.ThrowIfCancellationRequested();var canonical=Clipper.Union(literal,FillRule.NonZero);token.ThrowIfCancellationRequested();
            canonicalPoints=canonical.Sum(r=>r.Count);
            if(canonical.Count==0||canonicalPoints>500000){reason="raw-supplemental-reach-contour-empty-or-unsupported-source-fallback";return false;}
            var offsetter=new ClipperOffset(miterLimit:PolygonOps.DefaultMiterLimit,arcTolerance:1,preserveCollinear:false,reverseSolution:false);
            offsetter.AddPaths(canonical,JoinType.Round,EndType.Polygon);var raw=new Paths64();
            token.ThrowIfCancellationRequested();offsetter.Execute(-nominationWidth*PathUtils.Scale,raw);token.ThrowIfCancellationRequested();
            rawRings=raw.Count;long count=raw.Sum(r=>(long)r.Count);
            if(count>500000){reason="raw-supplemental-reach-contour-output-point-domain-fallback";return false;}rawPoints=(int)count;
            // Entire old band pipeline owns unsupported cases, including its
            // original terminal proof. No RAW empty/tiny stock claim or deletion.
            if(raw.Count==0||raw.Any(r=>r.Count<3)){reason="raw-supplemental-reach-contour-empty-or-degenerate-NOM-whole-band-fallback";return false;}
            int checkedPoints=0;
            foreach(var ring in raw)foreach(var p in ring)
            {if((checkedPoints++&255)==0)token.ThrowIfCancellationRequested();if(p.X<=-(1L<<29)||p.X>=(1L<<29)||p.Y<=-(1L<<29)||p.Y>=(1L<<29))
                {reason="raw-supplemental-reach-contour-output-coordinate-domain-fallback";return false;}}
            if(_conditionInitialFront&&RawInitialClearingFrontConditioner.TryCondition(raw,token,diagnostic,out var conditioned,out _))raw=conditioned!;
            candidate=PolygonOps.MaterializeIntegerPaths(raw);token.ThrowIfCancellationRequested();
            status="nominated";reason="RAW-reach-contour-NOM-requires-original-smoothing-floor-artwork-channel-grade";return true;
        }
        catch(OverflowException){candidate=null;reason="raw-supplemental-reach-contour-bounded-arithmetic-whole-band-fallback";return false;}
        finally
        {
            diagnostic?.Invoke($"stage=raw-initial-clearing-band policy={ReachContourPolicy} status={status} reason='{reason}' "
                +$"authoredWidth={authoredWidth:R} authoredWidthBits={BitConverter.DoubleToInt64Bits(authoredWidth)} nominationWidth={nominationWidth:R} nominationWidthBits={BitConverter.DoubleToInt64Bits(nominationWidth)} radiusDelta={radiusDelta:R} fullPhysicalRadius={fullPhysicalRadius:R} supplementalPhysicalRadius={supplementalPhysicalRadius:R} addedCommandReserve=0 "
                +$"sourceRings={source?.Count??0} sourcePoints={sourcePoints} canonicalSourcePoints={canonicalPoints} rawRings={rawRings} rawPoints={rawPoints} "
                +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} allocatedBytes={GC.GetAllocatedBytesForCurrentThread()-allocationStart} "
                +"sourceReduction=False nativeArcTicks=1 unsupportedWholeOriginalBandFallback=True directAllRawContourInventory=True authoredWidthUnchanged=True reachCompensationOnlyNOM=True "
                +"certifiedInsetCacheCredit=False physicalCredit=False filledSetCredit=False CcoverageCredit=False stockCredit=False originalMedialFloorUnchanged=True downstreamOriginalProtectRequired=True");
            token.ThrowIfCancellationRequested();
        }
    }
}
