namespace RW.Toolpaths.Milling;

/// <summary>Transient machine context shared by every milling generator. No
/// geometry or clearance permission is implied by a feed or safe-plane setting.</summary>
public sealed record MillingMotionSettings(double FeedRate, double PlungeRate, double TravelRate, double SafeZ,
    double MaterialSurfaceZ = double.NaN)
{
    /// <summary>Optional immutable earlier-stock authority, in final native XYZ.
    /// Must prove the whole cutter in nominal and actual posted coordinates.</summary>
    public Func<Point3D, Point3D, bool>? ClearedTravelGuard { get; init; }

    /// <summary>Optional immutable board/profile geometry for routing-cost
    /// estimates only. This setting never authorizes a commanded transfer.</summary>
    public MillingAirTransferContext? AirTransferContext { get; init; }
}

/// <summary>Complete visit cost, including retained cuts and source entry/recovery.
/// Uses the common transfer construction and nominal/posted coordinated feed.</summary>
internal sealed class ToolpathMotionCost
{
    // The verified same-region policy is selected by default. False retains
    // legacy global-height pricing for controlled comparisons.
    internal static readonly AsyncLocal<bool?> AirTransferEstimateOverride=new();
    private readonly MillingMotionSettings settings;
    private readonly float top;
    private readonly MillingFeedPolicy limits;
    private readonly record struct ChordKey(int AX,int AY,int AZ,int BX,int BY,int BZ);
    private readonly record struct ChordGeometry(double Distance,double Limit,bool Travel);
    private const int MaximumChordGeometryEntries=131072;
    private readonly int chordGeometryCacheLimit;
    private readonly Dictionary<ChordKey,ChordGeometry> chordGeometry=new();
    private readonly Queue<ChordKey> chordGeometryOrder=new();
    internal long ChordGeometryHits{get;private set;}
    internal long ChordGeometryMisses{get;private set;}
    internal long ChordGeometryEvictions{get;private set;}
    internal int ChordGeometryEntries=>chordGeometry.Count;
    internal bool LocalTransferEstimatesEnabled { get; set; }
    private long localEstimateAttempts,localEstimateAccepted,localEstimateRefused,globalTransferPrices;
    private double sampledAlternativePriceDifference;
    internal ToolpathMotionCost(MillingMotionSettings settings, double surface,int chordGeometryCacheLimit=0)
    {
        if(chordGeometryCacheLimit<0||chordGeometryCacheLimit>MaximumChordGeometryEntries)
            throw new ArgumentOutOfRangeException(nameof(chordGeometryCacheLimit));
        this.chordGeometryCacheLimit=chordGeometryCacheLimit;
        double stockTop = double.IsFinite(settings.MaterialSurfaceZ) ? settings.MaterialSurfaceZ : surface;
        if (!double.IsFinite(settings.SafeZ) || settings.SafeZ < stockTop)
            throw new ArgumentOutOfRangeException(nameof(settings), "The transfer plane must be at or above the material surface.");
        this.settings = settings; top = (float)stockTop;
        limits = new(settings.FeedRate, settings.PlungeRate, settings.TravelRate);
    }
    internal double MinimumSecondsPerMm => 60 / Math.Max(settings.FeedRate, settings.TravelRate);

    internal double CompleteVisitSeconds(Point3D from, TaggedToolpath path)
        => TransferSeconds(from, path.Points[0]) + VisitSeconds(path);

    internal double SequenceSeconds(Point3D? incoming, IReadOnlyList<TaggedToolpath> paths, Point3D? following)
    {
        double seconds = 0;
        Point3D? previous = incoming;
        bool trackLocal=LocalTransferEstimatesEnabled&&AirTransferEstimateOverride.Value!=false;
        int? previousRegion=null;bool ownRegionContext=false;
        foreach (var path in paths)
        {
            if (path.Points.Count < 2) continue;
            var from = previous ?? path.Points[0] with { Z = settings.SafeZ };
            // The incoming/following points have no region identity. Keep
            // those outer boundaries global; only internal same-region hops
            // after a real nonfast source chord can use the local estimate.
            bool local=trackLocal&&previousRegion==path.RegionIndex&&ownRegionContext;
            seconds += TransferSeconds(from,path.Points[0],local)+VisitSeconds(path);
            if(trackLocal){
                ownRegionContext=(previousRegion==path.RegionIndex&&ownRegionContext)||HasNonfastChord(path);
                previousRegion=path.RegionIndex;
            }
            previous = path.Points[^1];
        }
        if (previous is { } end)
            seconds += following is { } next ? GlobalTransferSeconds(end, next)
                : end.Z < settings.SafeZ ? ChordSeconds(end, end with { Z = settings.SafeZ }) : 0;
        return seconds;
    }

    internal static bool PreferContinuation(ToolpathMotionCost? cost, Point3D from,
        TaggedToolpath independentEntry, TaggedToolpath continuation, out double before, out double after)
    {
        before = cost?.CompleteVisitSeconds(from, independentEntry) ?? double.PositiveInfinity;
        after = cost?.CompleteVisitSeconds(from, continuation) ?? 0;
        return after <= before + 1e-9;
    }

    internal double TransferSeconds(Point3D from, Point3D to)=>TransferSeconds(from,to,LocalTransferEstimatesEnabled);

    internal double GlobalTransferSeconds(Point3D from,Point3D to)=>TransferSeconds(from,to,allowLocal:false);

    private double TransferSeconds(Point3D from,Point3D to,bool allowLocal)
    {
        if (MillingTransitionPlanner.IsContinuous(from, to)) return 0;
        if(allowLocal&&AirTransferEstimateOverride.Value!=false
            &&MillingLocalTravel.AirPlaneOptimizationOverride.Value!=false
            &&VCarvePathRouter.ConnectionOptimizationOverride.Value!=false){
            localEstimateAttempts++;
            if(settings.AirTransferContext is {} context&&context.MaterialSurface==top){
                // Routing has no actual modal cursor. This rounded nominal pose
                // is a cost nomination only; emission rechecks its real cursor.
                var estimate=new Point3D(ToolpathCommandRounding.Round((float)from.X),ToolpathCommandRounding.Round((float)from.Y),
                    ToolpathCommandRounding.Round((float)from.Z-top));
                if(context.TryBuild(from,to,estimate,(float)settings.SafeZ,settings.ClearedTravelGuard,out _) is {} construction){
                    localEstimateAccepted++;
                    double priced=construction.Moves.Sum(s=>ChordSeconds(s.Start,s.End,s.ClearedTravel));
                    sampledAlternativePriceDifference+=construction.GlobalMoves.Sum(s=>ChordSeconds(s.Start,s.End,s.ClearedTravel))-priced;
                    return priced;
                }
            }
            localEstimateRefused++;
        }
        globalTransferPrices++;
        return MillingTransitionPlanner.Build(from, to, (float)settings.SafeZ, top, settings.ClearedTravelGuard)
            .Sum(s => ChordSeconds(s.Start, s.End, s.ClearedTravel));
    }

    internal static bool HasNonfastChord(TaggedToolpath path)
    {
        var kinds=ToolpathSpanKinds.Expand(path);
        for(int i=1;i<path.Points.Count;i++)if(kinds[i-1] is ToolpathSpanKind.Cut or ToolpathSpanKind.Ramp or ToolpathSpanKind.Link){
            var a=path.Points[i-1];var b=path.Points[i];
            if((float)a.X!=(float)b.X||(float)a.Y!=(float)b.Y||(float)a.Z!=(float)b.Z)return true;
        }
        return false;
    }

    internal void Report(Action<string>? log,string owner)=>log?.Invoke($"stage=transfer-cost-estimate owner={owner} "
        +$"enabled={AirTransferEstimateOverride.Value!=false} ownRegionEnabled={LocalTransferEstimatesEnabled} knownBoardContext={settings.AirTransferContext is not null} "
        +$"localAttempts={localEstimateAttempts} localAccepted={localEstimateAccepted} localRefused={localEstimateRefused} globalPrices={globalTransferPrices} "
        +$"sampledAlternativePriceDifferenceSec={sampledAlternativePriceDifference:R} "
        +$"chordGeometryHits={ChordGeometryHits} chordGeometryMisses={ChordGeometryMisses} chordGeometryEvictions={ChordGeometryEvictions} chordGeometryEntries={ChordGeometryEntries} "
        +"memoScope=exact-float-endpoints-and-this-cost-owner modalFeedReplayed=True originalSummationOrder=True "
        +"sampledDifferenceIsNotRouteSaving=True origin=rounded-native-estimate actualModalAuthority=False initialAndCrossRegionGlobal=True "
        +"belowPlaneApproachRetained=True sourceMotionAndFeedsUnchanged=True finalEmitterProofRequired=True");

    internal double VisitSeconds(TaggedToolpath path)
    {
        var feed = new MillingFeedPolicy(settings.FeedRate, settings.PlungeRate, settings.TravelRate);
        double seconds = 0;
        var kinds = ToolpathSpanKinds.Expand(path);
        for (int i = 1; i < path.Points.Count; i++)
            seconds += Chord(path.Points[i - 1], path.Points[i], kinds[i - 1], feed);
        return seconds;
    }

    internal double ChordSeconds(Point3D a, Point3D b, bool cleared = false)
        => Chord(a, b, cleared ? ToolpathSpanKind.ClearedTravel : ToolpathSpanKind.Link, null);

    internal double ChordSeconds(Point3D a, Point3D b, ToolpathSpanKind kind)
        => Chord(a, b, kind, new MillingFeedPolicy(settings.FeedRate, settings.PlungeRate, settings.TravelRate));

    private double Chord(Point3D a, Point3D b, ToolpathSpanKind kind, MillingFeedPolicy? feed)
    {
        // One-shot pricing owners keep the original arithmetic path and do not
        // pay key construction, lookup or FIFO costs. Ring routing opts in.
        if(chordGeometryCacheLimit==0)return LiteralChord(a,b,kind,feed);
        float ax = (float)a.X, ay = (float)a.Y, az = (float)a.Z;
        float bx = (float)b.X, by = (float)b.Y, bz = (float)b.Z;
        var key=new ChordKey(BitConverter.SingleToInt32Bits(ax),BitConverter.SingleToInt32Bits(ay),BitConverter.SingleToInt32Bits(az),
            BitConverter.SingleToInt32Bits(bx),BitConverter.SingleToInt32Bits(by),BitConverter.SingleToInt32Bits(bz));
        ChordGeometry geometry;
        // Candidate visits repeatedly contain the same immutable ring body.
        // Cache only its scalar geometry, never a modal feed or a visit total:
        // the preceding chord can change the one-mm/min feed hold decision.
        if(chordGeometryCacheLimit>0&&chordGeometry.TryGetValue(key,out geometry))ChordGeometryHits++;
        else
        {
            ChordGeometryMisses++;
            geometry=PrepareChord(ax,ay,az,bx,by,bz);
            if(chordGeometryCacheLimit>0)
            {
                if(chordGeometry.Count>=chordGeometryCacheLimit)
                {chordGeometry.Remove(chordGeometryOrder.Dequeue());ChordGeometryEvictions++;}
                chordGeometry.Add(key,geometry);chordGeometryOrder.Enqueue(key);
            }
        }
        double cutting=feed?.Command(geometry.Limit,kind==ToolpathSpanKind.ClearedTravel)??Math.Floor(geometry.Limit);
        double rate=kind==ToolpathSpanKind.ClearedTravel?Math.Max(settings.FeedRate,settings.TravelRate)
            :kind!=ToolpathSpanKind.TabLift&&geometry.Travel?settings.TravelRate:cutting;
        return geometry.Distance*60/rate;
    }

    private double LiteralChord(Point3D a,Point3D b,ToolpathSpanKind kind,MillingFeedPolicy? feed)
    {
        float ax=(float)a.X,ay=(float)a.Y,az=(float)a.Z;
        float bx=(float)b.X,by=(float)b.Y,bz=(float)b.Z;
        double dx=bx-ax,dy=by-ay,dz=bz-az;
        double distance=Math.Sqrt(Math.Pow((double)bx-ax,2)+Math.Pow((double)by-ay,2)+Math.Pow((double)bz-az,2));
        bool vertical=ToolpathCommandRounding.IsVerticalOnly((float)dx,(float)dz,(float)dy);
        static double Q(float value)=>ToolpathCommandRounding.Round(value);
        double limit=Math.Min(limits.Limit(dx,dz,dy),limits.Limit(
            vertical?0:Q(bx)-Q(ax),Q(bz-top)-Q(az-top),vertical?0:Q(by)-Q(ay)));
        double cutting=feed?.Command(limit,kind==ToolpathSpanKind.ClearedTravel)??Math.Floor(limit);
        double rate=kind==ToolpathSpanKind.ClearedTravel?Math.Max(settings.FeedRate,settings.TravelRate)
            :kind!=ToolpathSpanKind.TabLift&&(vertical&&(dz>0||bz>=top)
                ||!vertical&&Math.Sqrt(dx*dx+dy*dy)>.001&&Math.Min(az,bz)>=top)
                ?settings.TravelRate:cutting;
        return distance*60/rate;
    }

    private ChordGeometry PrepareChord(float ax,float ay,float az,float bx,float by,float bz)
    {
        double dx = bx - ax, dy = by - ay, dz = bz - az;
        double distance = Math.Sqrt(Math.Pow((double)bx - ax, 2) + Math.Pow((double)by - ay, 2) + Math.Pow((double)bz - az, 2));
        bool vertical = ToolpathCommandRounding.IsVerticalOnly((float)dx, (float)dz, (float)dy);
        static double Q(float v) => ToolpathCommandRounding.Round(v);
        double limit = Math.Min(limits.Limit(dx, dz, dy), limits.Limit(
            vertical ? 0 : Q(bx) - Q(ax), Q(bz - top) - Q(az - top), vertical ? 0 : Q(by) - Q(ay)));
        bool travel = vertical && (dz > 0 || bz >= top)
                || !vertical && Math.Sqrt(dx * dx + dy * dy) > .001 && Math.Min(az, bz) >= top;
        return new(distance,limit,travel);
    }
}
