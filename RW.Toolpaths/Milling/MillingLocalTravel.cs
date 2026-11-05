using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Local transfers through actual executed cuts. Recorded cuts and
/// postedFrom use stock-relative F3 coordinates; cutting paths are never rewritten.</summary>
public sealed class MillingLocalTravel
{
    // Known-stock air fallback. False retains the safe-height control; a missing
    // boundary always retains the existing prefix / safe-height policy.
    internal static readonly AsyncLocal<bool?> AirPlaneOptimizationOverride=new();
    public const int PolicyRevision=4;
    public const double Clearance=MillingTransitionPlanner.PositiveTravelClearance;
    private readonly VCarveCutterProfile profile;
    private readonly float surface;
    private readonly CancellationToken token;
    private readonly Action<string>? diagnostic;
    private CutterSweepStock stock;
    private ClearedColumnStock? columns;
    private readonly ClearedColumnStock.FloorPrefixCache floorPrefixes=new();
    private readonly List<ClearedColumnStock.Column> cylinderCuts=new();
    private readonly bool cylindrical;
    private readonly MillingAirTransferContext? airContext;
    private readonly double fullProfileEnvelope;
    private int? region;
    private long attempts,accepted,heightOnly,refused,recorded;
    private long regionRecorded,airPlaneAttempts,airPlaneAccepted,airPlaneRefused;
    private double airPlaneSavedDistance;
    private long retiredReindexed,retiredQueries;
    private double retiredIndexMs;

    public MillingLocalTravel(VCarveCutterProfile profile,float materialSurface,
        CancellationToken cancellationToken=default,Action<string>? diagnostic=null)
        :this(profile,materialSurface,cancellationToken,diagnostic,null)
    {
    }

    public MillingLocalTravel(VCarveCutterProfile profile,float materialSurface,
        CancellationToken cancellationToken,Action<string>? diagnostic,IReadOnlyList<PointD>? stockBoundary)
    {
        ArgumentNullException.ThrowIfNull(profile);
        if(!float.IsFinite(materialSurface))throw new ArgumentOutOfRangeException(nameof(materialSurface));
        this.profile=profile;surface=materialSurface;token=cancellationToken;this.diagnostic=diagnostic;
        stock=NewStock();
        cylindrical=profile.TipRadius>0&&profile.RadiusAtDepth(profile.MaximumDepth)==profile.TipRadius;
        airContext=MillingAirTransferContext.Create(profile,surface,stockBoundary);
        fullProfileEnvelope=profile.Sections.Max(s=>Math.Max(s.BottomRadius,s.TopRadius))+Clearance;
    }

    public void BeginRegion(int regionIndex)
    {
        token.ThrowIfCancellationRequested();
        if(region==regionIndex)return;
        retiredReindexed+=stock.ReindexedChords;retiredIndexMs+=stock.StockIndexBuildMilliseconds;retiredQueries+=stock.Queries;
        region=regionIndex;stock=NewStock();columns=null;cylinderCuts.Clear();floorPrefixes.Reset();regionRecorded=0;
    }

    public void RecordExecutedCut(Point3D start,Point3D end)
    {
        token.ThrowIfCancellationRequested();
        if(region is null||!Finite(start)||!Finite(end))return;
        stock.RecordExecuted(start,end);recorded++;regionRecorded++;
        if(cylindrical&&cylinderCuts.Count<2_000_000&&Math.Min(start.Z,end.Z)<0)
        {cylinderCuts.Add(new(start,end,profile.TipRadius));columns=null;}
    }

    internal bool MatchesSurface(float materialSurface)=>materialSurface==surface;

    internal IReadOnlyList<MillingTransition>? Connect(Point3D from,Point3D to,Point3D postedFrom,float safeZ)
    {
        token.ThrowIfCancellationRequested();attempts++;
        if((attempts&127)==0)diagnostic?.Invoke($"event=progress region={region} attempted={attempts} accepted={accepted} "
            +$"recordedExecutedChords={recorded} stockQueries={retiredQueries+stock.Queries} stockIndexMs={retiredIndexMs+stock.StockIndexBuildMilliseconds:F1} "
            +$"stockReindexedChords={retiredReindexed+stock.ReindexedChords} sourceCutsPreserved=True");
        if(region is null||!float.IsFinite(safeZ)||!Finite(from)||!Finite(to)||!Finite(postedFrom)
            ||Math.Max(from.Z,to.Z)>=safeZ)return Refuse("invalid-or-safe-plane-endpoint");
        if(!MillingTransitionPlanner.RequiresPlanarTraverse(from,to))
        {
            // Subset of the old same-XY retract/approach. Preserve controlled
            // descent and the omitted-XY cursor without the redundant round trip.
            heightOnly++;accepted++;
            // Do not let a sub-continuity XY mismatch become a new full-axis
            // command. The original bridge omitted XY on this vertical move.
            return new[]{new MillingTransition(from,new(from.X,from.Y,to.Z))};
        }
        stock.Seal();
        double cap=Math.Min(safeZ-.001,surface+MillingTransitionPlanner.ApproachClearance);
        double lowest=Math.Min(Math.Max(from.Z,to.Z)+1,cap);
        if(lowest<Math.Max(from.Z,to.Z)+.01)return Refuse("no-local-lift-room");
        foreach(double candidate in new[]{lowest,(lowest+cap)/2,cap}.Distinct())
        {
            token.ThrowIfCancellationRequested();float z=(float)candidate;
            if(z>=safeZ||z<=Math.Max(from.Z,to.Z))continue;
            var nominalA=new Point3D((float)from.X,(float)from.Y,z-surface);
            var nominalB=new Point3D((float)to.X,(float)to.Y,z-surface);
            var postedA=new Point3D(postedFrom.X,postedFrom.Y,Q(z-surface));
            var postedB=new Point3D(Q((float)to.X),Q((float)to.Y),postedA.Z);
            if(!Clear(nominalA,nominalB)||!Clear(postedA,postedB))continue;
            var a=new Point3D((float)from.X,(float)from.Y,(float)from.Z);
            var b=new Point3D((float)to.X,(float)to.Y,(float)to.Z);
            var raisedA=a with{Z=z};var raisedB=b with{Z=z};accepted++;
            if(accepted<=12)diagnostic?.Invoke($"event=connect region={region} localZ={z:R} surfaceZ={surface:R} "
                +$"from=({a.X:R},{a.Y:R},{a.Z:R}) to=({b.X:R},{b.Y:R},{b.Z:R}) "
                +$"clearanceMm={Clearance:R} authority=executed-prefix nominalAndPosted=True sourceCutsPreserved=True");
            return new[]{new MillingTransition(a,raisedA),new MillingTransition(raisedA,raisedB,ClearedTravel:true),
                new MillingTransition(raisedB,b)};
        }
        return Refuse("whole-cutter-prefix-clearance-unproved");
    }

    internal IReadOnlyList<MillingTransition>? ConnectAboveStock(Point3D from,Point3D to,Point3D postedFrom,float safeZ,
        Func<Point3D,Point3D,bool>? clearedTravelGuard)
    {
        if(AirPlaneOptimizationOverride.Value==false
            ||AirPlaneOptimizationOverride.Value is null&&VCarvePathRouter.ConnectionOptimizationOverride.Value==false)return null;
        token.ThrowIfCancellationRequested();airPlaneAttempts++;
        if(region is null||regionRecorded==0)return RefuseAirPlane("no-executed-region-context");
        if(airContext is null)return RefuseAirPlane("missing-or-nonrectangular-stock-boundary");
        if(airContext.TryBuild(from,to,postedFrom,safeZ,clearedTravelGuard,out string refusal) is not {} construction)
            return RefuseAirPlane(refusal);
        airPlaneAccepted++;airPlaneSavedDistance+=2*(safeZ-construction.Plane);
        if(airPlaneAccepted<=12)diagnostic?.Invoke($"event=connect-above-stock region={region} planeZ={construction.Plane:R} originalSafeZ={safeZ:R} "
            +$"originalStockTop={surface:R} nativeAxialReserveMm={construction.NativeReserve:R} postedAxialReserveMm={construction.PostedReserve:R} "
            +$"fullProfileEnvelopeMm={fullProfileEnvelope:R} authority=known-rectangular-stock-top nominalAndActualF3=True belowPlaneApproachModesRetained=True");
        return construction.Moves;
    }

    public void Report()=>diagnostic?.Invoke($"event=summary policy={PolicyRevision} attempted={attempts} accepted={accepted+airPlaneAccepted} "
        +$"heightOnly={heightOnly} retainedSafePlane={refused-airPlaneAccepted} localPrefixRefusals={refused} recordedExecutedChords={recorded} "
        +$"floorUnionBuilds={floorPrefixes.Builds} floorUnionAddedChords={floorPrefixes.AddedChords} "
        +$"floorUnionReusedChords={floorPrefixes.ReusedChords} floorUnionResets={floorPrefixes.Resets} "
        +$"stockQueries={retiredQueries+stock.Queries} stockIndexMs={retiredIndexMs+stock.StockIndexBuildMilliseconds:F1} "
        +$"stockReindexedChords={retiredReindexed+stock.ReindexedChords} "
        +$"clearanceMm={Clearance:R} stockScope=same-region-executed-prefix futureCutCredit=False sourceCutsPreserved=True "
        +$"airPlaneAttempted={airPlaneAttempts} airPlaneAccepted={airPlaneAccepted} airPlaneRefused={airPlaneRefused} "
        +$"airPlaneSavedAboveStockDistanceMm={airPlaneSavedDistance:R} knownStockRectangle={airContext is not null}");

    private bool Clear(Point3D a,Point3D b)
    {
        if(stock.ContainsTravel(a,b,Clearance))return true;
        if(!cylindrical||cylinderCuts.Count==0)return false;
        // A flat mill can need the union of several real cleared lanes. Reuse
        // the existing conservative posted-sweep union, never sampled tip disks.
        columns??=new ClearedColumnStock(cylinderCuts,token,floorPrefixCache:floorPrefixes);
        return columns.Contains(a with{Z=Math.Min(0,a.Z)-Clearance},b with{Z=Math.Min(0,b.Z)-Clearance},
            d=>profile.PhysicalRadiusAtDepth(d)+Clearance);
    }
    private CutterSweepStock NewStock()=>new(profile,p=>p,surface,token,mergeIncrementally:true);
    private IReadOnlyList<MillingTransition>? Refuse(string reason)
    {
        if(++refused<=8)diagnostic?.Invoke($"event=retain-safe-plane region={region} reason={reason}");
        return null;
    }
    private IReadOnlyList<MillingTransition>? RefuseAirPlane(string reason)
    {
        if(++airPlaneRefused<=8)diagnostic?.Invoke($"event=retain-safe-plane region={region} airPlaneRefused={reason}");
        return null;
    }
    private static double Q(float v)=>ToolpathCommandRounding.Round(v);
    private static bool Finite(Point3D p)=>double.IsFinite(p.X)&&double.IsFinite(p.Y)&&double.IsFinite(p.Z);
}
