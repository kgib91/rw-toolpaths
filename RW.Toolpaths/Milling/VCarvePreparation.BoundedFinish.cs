using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

public sealed partial class VCarvePreparation
{
    /// <summary>
    /// Prepares a bounded smooth finish while allowing an exact original upper
    /// sample to settle the source-plateau decision before unused depth arrays
    /// are built. Detailed consumers still receive the complete original data.
    /// This changes no source graph, floor, protected mask or cutter proof.
    /// </summary>
    public static VCarvePreparation PrepareBoundedFinish(IReadOnlyList<MillingRegion> regions,ToolGeometry tool,
        DepthSchedule depth,VCarveOptions options,IMedialAxisProvider provider,CancellationToken token=default)
    {
        var access=options.BoundaryAccess?.Resolve(regions,tool,depth,options.SmoothVCarving,
            options.SmoothRampAngleDegrees,options.StockToLeave,token);
        var resolved=access is null?options:options with{BoundaryAccess=null,AccessTarget=access};
        bool eligible=resolved.SmoothVCarving&&!resolved.IncludeInteriorFill&&resolved.EdgeFillWidth>0
            &&resolved.StockToLeave==0&&!resolved.RampOutsideEdges&&resolved.ClearedStock is null;
        return Create(access?.GetRegions(token)??regions,tool,depth,resolved,provider,token,eligible);
    }

    // No request token is retained. Cached preparation can outlive the pocket
    // worker's CTS; explicit consumers supply their current caller token and
    // property-only consumers use the safe default token.
    internal sealed class DeferredRegionDepths(Func<CancellationToken,PreparedRegion> factory,
        double sampledUpperWitness,VCarveOptions options,int regionIndex)
    {
        private readonly object gate=new();
        private Func<CancellationToken,PreparedRegion>? pending=factory;
        private PreparedRegion? completed;
        internal bool HasCompleted=>Volatile.Read(ref completed) is not null;
        internal double SampledUpperWitness=>sampledUpperWitness;
        internal PreparedRegion Complete(CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            lock(gate)
            {
                token.ThrowIfCancellationRequested();
                if(completed is {} cached)return cached;
                long started=Stopwatch.GetTimestamp();
                Log(options,$"event=deferred-depths state=materializing region={regionIndex} fullOriginalAlgorithm=True");
                var value=pending!(token);
                token.ThrowIfCancellationRequested();
                Volatile.Write(ref completed,value);pending=null;
                Log(options,$"event=deferred-depths state=complete region={regionIndex} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} "
                    +"fullOriginalAlgorithm=True publication=complete-only staleRequestTokenRetained=False");
                return value;
            }
        }
    }

    private static bool TrySampledUpperWitness(int regionIndex,IReadOnlyList<IReadOnlyList<PointD>> boundary,
        IReadOnlyList<MedialSegment> raw,VCarveOptions options,VCarveCutterProfile physical,double maximumDepth,
        double grade,CancellationToken token,out double witness)
    {
        witness=0;long started=Stopwatch.GetTimestamp();int attempted=0;
        var crests=VCarveRampSurface.Sources(boundary,options,token);
        if(crests.Count==0)return false;
        var field=new VCarveSmoothing.FloorField(crests,physical,grade,maximumDepth,token);
        var edges=boundary.SelectMany(ring=>Enumerable.Range(0,ring.Count)
            .Select(i=>(A:ring[i],B:ring[(i+1)%ring.Count]))).ToList();
        var sourceDistance=new MedialAxisToolpaths.SpatialSegmentIndex(edges);
        // These are actual samples of the original complete graph, using its
        // identical interpolation arithmetic and upper-depth formula. The cap
        // is a nomination-work bound only: an absent witness uses full sampling.
        int probes=Math.Min(128,raw.Count);
        for(int probe=0;probe<probes;probe++)
        {
            token.ThrowIfCancellationRequested();
            int ordinal=probes==1?0:(int)((long)probe*(raw.Count-1)/(probes-1));
            var segment=raw[ordinal];double length=Distance(segment.Point0,segment.Point1);
            int steps=Math.Max(1,(int)Math.Ceiling(length/MaximumSampleSpacing));
            if(steps>1_000_000)return false;
            int sample=steps/2;double fraction=(double)sample/steps;
            double x=segment.Point0.X+fraction*(segment.Point1.X-segment.Point0.X);
            double y=segment.Point0.Y+fraction*(segment.Point1.Y-segment.Point0.Y);
            double radius=sourceDistance.QueryDistance(x,y);
            double originalUpper=Math.Min(maximumDepth,physical.DepthAtRadius(radius+GeometricErrorBound));
            attempted++;
            if(originalUpper<maximumDepth)continue;
            double cap=field.Depth(x,y)+MachiningAccuracy.FloorSearch;
            double actualUpper=Math.Min(originalUpper,cap);
            if(actualUpper<maximumDepth)continue;
            witness=actualUpper;
            Log(options,$"event=sampled-upper-witness accepted=True region={regionIndex} graphOrdinal={ordinal} sample={sample}/{steps} "
                +$"x={x:R} y={y:R} upperDepth={actualUpper:R} maximumDepth={maximumDepth:R} probes={attempted} "
                +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} originalSampleArithmetic=True wholeSourceFalseOnly=True "
                +"originalPlateauMaskRequired=True fullDepthsDeferred=True");
            return true;
        }
        Log(options,$"event=sampled-upper-witness accepted=False region={regionIndex} probes={attempted} "
            +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} fallback=full-original-depth-preparation");
        return false;
    }
    private static PreparedRegion PrepareRegionDepths(int regionIndex,IReadOnlyList<IReadOnlyList<PointD>> boundary,
        IReadOnlyList<MedialSegment> raw,VCarveOptions options,VCarveCutterProfile physical,double maximumDepth,
        double medialMs,CancellationToken cancellationToken)
    {
        long depthsStarted=Stopwatch.GetTimestamp();
            var paths = new TaggedToolpath[raw.Count];
            var uppers = new TaggedToolpath[raw.Count];
            var clearancePaths = new IReadOnlyList<MedialPoint>[raw.Count];
            var chordLengths=new double[raw.Count];
            var boundaryEdges = boundary.SelectMany(ring => Enumerable.Range(0, ring.Count)
                .Select(index => (A: ring[index], B: ring[(index + 1) % ring.Count]))).ToList();
            var boundaryDistance = new MedialAxisToolpaths.SpatialSegmentIndex(boundaryEdges,enableDiagnostics:true);
            // Adjacent original graph chords share literal endpoints. Reuse
            // only identical XY queries against this immutable source index.
            const int radiusMemoLimit=65536;
            long radiusQueries=0,radiusHits=0,radiusClears=0;
            double SourceRadius(double x,double y,SourceRadiusMemo memo)
            {
                memo.Queries++;
                if(memo.Values.TryGetValue((x,y),out double cached)){memo.Hits++;return cached;}
                double value=boundaryDistance.QueryDistance(x,y);
                if(memo.Values.Count==radiusMemoLimit){memo.Values.Clear();memo.Clears++;}
                memo.Values.Add((x,y),value);return value;
            }
            void Sample(int ordinal,SourceRadiusMemo memo)
            {
                MedialSegment segment=raw[ordinal];
                cancellationToken.ThrowIfCancellationRequested();
                double length = Distance(segment.Point0, segment.Point1);
                int steps = Math.Max(1, (int)Math.Ceiling(length / MaximumSampleSpacing));
                if (steps > 1_000_000) throw new InvalidOperationException("Original V-carve graph segment is too long to certify.");
                chordLengths[ordinal]=length/steps;
                var points = new List<Point3D>(steps + 1);
                var upper = new List<Point3D>(steps + 1);
                var clearances = new List<MedialPoint>(steps + 1);
                for (int sample = 0; sample <= steps; sample++)
                {
                    if((sample&1023)==0)cancellationToken.ThrowIfCancellationRequested();
                    double fraction = (double)sample / steps;
                    double x = segment.Point0.X + fraction * (segment.Point1.X - segment.Point0.X);
                    double y = segment.Point0.Y + fraction * (segment.Point1.Y - segment.Point0.Y);
                    // Point-point bisectors are straight but their radius is nonlinear, with
                    // a possible interior neck. Endpoint radius interpolation hides that basin.
                    // Keep source clearance separate and evaluate it on the preserved artwork.
                    double radius = SourceRadius(x,y,memo);
                    clearances.Add(new MedialPoint(x, y, radius));
                    points.Add(new Point3D(x, y, -Math.Min(maximumDepth, physical.DepthAtRadius(radius))));
                    upper.Add(new Point3D(x, y, -Math.Min(maximumDepth, physical.DepthAtRadius(radius + GeometricErrorBound))));
                }
                paths[ordinal]=new TaggedToolpath(points,Array.Empty<ToolpathSpan>(),regionIndex,"final-carve",null,detectCurves:false);
                uppers[ordinal]=new TaggedToolpath(upper,Array.Empty<ToolpathSpan>(),regionIndex,"final-carve",null,detectCurves:false);
                clearancePaths[ordinal]=clearances;
            }
            void Collect(SourceRadiusMemo memo)
            {
                Interlocked.Add(ref radiusQueries,memo.Queries);Interlocked.Add(ref radiusHits,memo.Hits);
                Interlocked.Add(ref radiusClears,memo.Clears);
            }
            int sampleWorkers=options.ParallelOriginalSourceSampling&&raw.Count>=4096
                ?Math.Min(4,GeometryParallelism.WorkerLimit):1;
            if(sampleWorkers>1)
                Parallel.For(0,raw.Count,new ParallelOptions{MaxDegreeOfParallelism=sampleWorkers,CancellationToken=cancellationToken},
                    ()=>new SourceRadiusMemo(),(ordinal,_,memo)=>{Sample(ordinal,memo);return memo;},Collect);
            else
            {
                var memo=new SourceRadiusMemo();
                for(int ordinal=0;ordinal<raw.Count;ordinal++)Sample(ordinal,memo);
                Collect(memo);
            }
            double maximumChord=chordLengths.Length==0?0:chordLengths.Max();
            boundaryDistance.LogDiagnostics(message=>Log(options,$"region={regionIndex} {message}"));
            double beforeGradeMs=System.Diagnostics.Stopwatch.GetElapsedTime(depthsStarted).TotalMilliseconds;
            var (smooth, upperSmooth) = VCarveRampSurface.FitMedial(paths, uppers,
                boundary, options, physical, maximumDepth, cancellationToken);
            Log(options, $"event=prepared region={regionIndex} originalGraphEdges={raw.Count} physicalPoints={paths.Sum(path => path.Points.Count)} " +
                $"originalPeak={Peak(paths):R} smoothedPeak={Peak(smooth):R} geometryError={GeometricErrorBound:R} " +
                $"medialMs={medialMs:F1} sourceSampleMs={beforeGradeMs:F1} gradeAndPublishMs={System.Diagnostics.Stopwatch.GetElapsedTime(depthsStarted).TotalMilliseconds-beforeGradeMs:F1} " +
                $"sourceRadiusQueries={radiusQueries} sourceRadiusMemoHits={radiusHits} sourceRadiusMemoClears={radiusClears} sourceRadiusMemoLimitPerWorker={radiusMemoLimit} sourceSamplingWorkers={sampleWorkers} sourceSamplePublication=original-graph-order sourceRadiusMemoKeys=exact-XY immutableSourceIndex=True " +
                "clearanceRadiiPreserved=True routingAndCommandReservesApplied=False surfaceOwner=continuous-retained-crest medialConeAnchors=False depthFields=2");
        return new PreparedRegion(regionIndex,boundary,raw,clearancePaths,paths,smooth,upperSmooth,maximumChord);
    }

}
