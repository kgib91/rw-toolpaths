using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Local transfers through actual executed cuts. Recorded cuts and
/// postedFrom use stock-relative F3 coordinates; cutting paths are never rewritten.</summary>
public sealed class MillingLocalTravel
{
    // Known-stock air fallback. False retains the safe-height control; a missing
    // boundary always retains the existing prefix / safe-height policy.
    internal static readonly AsyncLocal<bool?> AirPlaneOptimizationOverride=new();
    internal static readonly AsyncLocal<bool?> EndpointCandidateOptimizationOverride=new();
    public const int PolicyRevision=7;
    public const double Clearance=MillingTransitionPlanner.PositiveTravelClearance;
    /// <summary>Prefer the existing complete original-stock air certificate
    /// before querying mutable executed-prefix geometry. False retains the
    /// established prefix-first policy; missing stock authority still falls back.</summary>
    public bool PreferAboveStockTransfers {get;init;}
    private readonly VCarveCutterProfile profile;
    private readonly float surface;
    private readonly CancellationToken token;
    private readonly Action<string>? diagnostic;
    private CutterSweepStock stock;
    private ClearedColumnStock? columns;
    private readonly ClearedColumnStock.FloorPrefixCache floorPrefixes=new();
    private readonly List<ClearedColumnStock.Column> cylinderCuts=new();
    private readonly CylindricalFootprintBounds cylinderFootprint=new();
    private readonly bool cylindrical;
    private readonly MillingAirTransferContext? airContext;
    private readonly double fullProfileEnvelope;
    private int? region;
    private long attempts,accepted,heightOnly,refused,recorded;
    private long regionRecorded,airPlaneAttempts,airPlaneAccepted,airPlaneRefused;
    private long airFirstAttempts,airFirstAccepted,airFirstRefused;
    private double airPlaneSavedDistance;
    private long retiredReindexed,retiredQueries;
    private double retiredIndexMs;
    private long footprintChecks,footprintRefusals,footprintBypasses;
    private long endpointCandidateChecks,endpointCandidateRefusals,endpointCandidateBypasses,endpointCandidateNodes,endpointCandidateTicks;

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
        cylindrical=profile.TipRadius>0&&profile.RadiusAtDepth(profile.MaximumDepth)==profile.TipRadius;
        stock=NewStock();
        airContext=MillingAirTransferContext.Create(profile,surface,stockBoundary);
        fullProfileEnvelope=profile.Sections.Max(s=>Math.Max(s.BottomRadius,s.TopRadius))+Clearance;
    }

    public void BeginRegion(int regionIndex)
    {
        token.ThrowIfCancellationRequested();
        if(region==regionIndex)return;
        retiredReindexed+=stock.ReindexedChords;retiredIndexMs+=stock.StockIndexBuildMilliseconds;retiredQueries+=stock.Queries;
        region=regionIndex;stock=NewStock();columns=null;cylinderCuts.Clear();cylinderFootprint.Reset();floorPrefixes.Reset();regionRecorded=0;
    }

    public void RecordExecutedCut(Point3D start,Point3D end)
    {
        token.ThrowIfCancellationRequested();
        if(region is null||!Finite(start)||!Finite(end))return;
        stock.RecordExecuted(start,end);recorded++;regionRecorded++;
        if(cylindrical&&cylinderCuts.Count<2_000_000&&Math.Min(start.Z,end.Z)<0)
        {
            var column=new ClearedColumnStock.Column(start,end,profile.TipRadius);
            cylinderCuts.Add(column);cylinderFootprint.Add(column);columns=null;
        }
    }

    internal bool MatchesSurface(float materialSurface)=>materialSurface==surface;

    internal IReadOnlyList<MillingTransition>? ConnectAboveStockFirst(Point3D from,Point3D to,Point3D postedFrom,float safeZ,
        Func<Point3D,Point3D,bool>? clearedTravelGuard)
    {
        token.ThrowIfCancellationRequested();airFirstAttempts++;
        var result=ConnectAboveStock(from,to,postedFrom,safeZ,clearedTravelGuard);
        token.ThrowIfCancellationRequested();
        if(result is not null)
        {
            if(++airFirstAccepted<=8)diagnostic?.Invoke($"event=air-first-accepted region={region} "
                +"authority=known-stock-full-profile-native-and-posted prefixGeometryQueried=False belowPlaneApproachModesRetained=True");
        }
        else if(++airFirstRefused<=8)diagnostic?.Invoke($"event=air-first-refused region={region} "
            +"fallback=original-executed-prefix-then-air-then-safe-plane authorityUnchanged=True");
        return result;
    }

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

    public void Report()=>diagnostic?.Invoke($"event=summary policy={PolicyRevision} attempted={attempts+airFirstAccepted} accepted={accepted+airPlaneAccepted} "
        +$"heightOnly={heightOnly} retainedSafePlane={refused-(airPlaneAccepted-airFirstAccepted)} localPrefixRefusals={refused} recordedExecutedChords={recorded} "
        +$"floorUnionBuilds={floorPrefixes.Builds} floorUnionAddedChords={floorPrefixes.AddedChords} "
        +$"floorUnionReusedChords={floorPrefixes.ReusedChords} floorUnionResets={floorPrefixes.Resets} "
        +$"footprintBoundsChecks={footprintChecks} footprintBoundsRefusals={footprintRefusals} footprintBoundsBypasses={footprintBypasses} "
        +$"footprintBoundsReserveMm={CylindricalFootprintBounds.Reserve:R} footprintBoundsAuthority=outer-bound-of-same-region-stored-columns refusalOnly=True "
        +$"endpointCandidateChecks={endpointCandidateChecks} endpointCandidateRefusals={endpointCandidateRefusals} endpointCandidateBypasses={endpointCandidateBypasses} endpointCandidateNodes={endpointCandidateNodes} "
        +$"endpointCandidateMs={1000d*endpointCandidateTicks/System.Diagnostics.Stopwatch.Frequency:F3} endpointCandidateAuthority=outer-donor-bounds candidateDiscoveryOnly=True positiveQueriesUnchanged=True "
        +$"stockQueries={retiredQueries+stock.Queries} stockIndexMs={retiredIndexMs+stock.StockIndexBuildMilliseconds:F1} "
        +$"stockReindexedChords={retiredReindexed+stock.ReindexedChords} "
        +$"clearanceMm={Clearance:R} stockScope=same-region-executed-prefix futureCutCredit=False sourceCutsPreserved=True "
        +$"airPlaneAttempted={airPlaneAttempts} airPlaneAccepted={airPlaneAccepted} airPlaneRefused={airPlaneRefused} "
        +$"preferAboveStockTransfers={PreferAboveStockTransfers} airFirstAttempted={airFirstAttempts} airFirstAccepted={airFirstAccepted} airFirstRefused={airFirstRefused} prefixGeometryQueriesSkipped={airFirstAccepted} "
        +$"airPlaneSavedAboveStockDistanceMm={airPlaneSavedDistance:R} knownStockRectangle={airContext is not null}");

    private bool Clear(Point3D a,Point3D b)
    {
        if(!cylindrical&&EndpointCandidateOptimizationOverride.Value!=false)
        {
            long begun=System.Diagnostics.Stopwatch.GetTimestamp();
            var candidates=stock.FindTravelEndpointCandidates(a,b,Clearance);
            endpointCandidateTicks+=System.Diagnostics.Stopwatch.GetTimestamp()-begun;
            endpointCandidateChecks++;endpointCandidateNodes+=candidates.VisitedNodes;
            if(candidates.Bypassed)endpointCandidateBypasses++;
            if(!candidates.Possible)
            {
                if(++endpointCandidateRefusals<=8)diagnostic?.Invoke($"event=prefix-endpoint-without-candidate region={region} endpoint={candidates.RefusedEndpoint} "
                    +$"a={a} b={b} nodesVisited={candidates.VisitedNodes} "
                    +"decision=refuse-only stockProofQuerySkipped=True mutableHintsTouched=False futureCutCredit=False");
                return false;
            }
        }
        if(stock.ContainsTravel(a,b,Clearance))return true;
        if(!cylindrical||cylinderCuts.Count==0)return false;
        var queryA=a with{Z=Math.Min(0,a.Z)-Clearance};
        var queryB=b with{Z=Math.Min(0,b.Z)-Clearance};
        token.ThrowIfCancellationRequested();
        // Column proof can join several real lanes, but cannot reach a point
        // beyond the outer bounds of ALL its recorded cylindrical footprints.
        // Check only the constant-depth capsule used by these local transfers;
        // variable-depth recursive demands retain the complete original proof.
        if(queryA.Z==queryB.Z&&cylinderFootprint.IsUsable)
        {
            footprintChecks++;
            double requested=Math.Max(-queryA.Z,.0005);
            double requiredRadius=profile.PhysicalRadiusAtDepth(requested)+Clearance;
            if(cylinderFootprint.TryUncoveredWitness(queryA,queryB,requiredRadius,out var witness))
            {
                if(++footprintRefusals<=8)diagnostic?.Invoke($"event=column-footprint-uncovered region={region} "
                    +$"witness=({witness.X:R},{witness.Y:R},{witness.Z:R}) queryRadius={requiredRadius:R} "
                    +$"storedColumns={cylinderCuts.Count} outerBounds={cylinderFootprint.Describe()} "
                    +$"reserveMm={CylindricalFootprintBounds.Reserve:R} decision=refuse-only priorCutterProofAlreadyRefused=True columnGeometryBuilt=False");
                return false;
            }
        }
        else footprintBypasses++;
        // A flat mill can need the union of several real cleared lanes. Reuse
        // the existing conservative posted-sweep union, never sampled tip disks.
        columns??=new ClearedColumnStock(cylinderCuts,token,floorPrefixCache:floorPrefixes);
        return columns.Contains(queryA,queryB,
            d=>profile.PhysicalRadiusAtDepth(d)+Clearance);
    }

    /// <summary>Negative evidence only. Every recorded cylindrical sweep and
    /// every conservative Column union/pose proof lies inside these outward
    /// bounds. Uncertain arithmetic disables this optional test.</summary>
    internal sealed class CylindricalFootprintBounds
    {
        // One full posted-coordinate quantum plus eight Boolean-coordinate
        // quanta cover endpoint reconstruction and query-centre quantization.
        // PostedSweepInnerRegion already subtracts its independent inner reserve;
        // this additional outward reserve can only withhold a refusal.
        internal const double Reserve=.001+8/PathUtils.Scale+1e-8;
        private double minX,minY,maxX,maxY;
        private bool hasColumns,uncertain;
        internal bool IsUsable=>hasColumns&&!uncertain;
        internal void Reset(){hasColumns=false;uncertain=false;minX=minY=maxX=maxY=0;}
        internal void Add(ClearedColumnStock.Column column)
        {
            if(!Finite(column.A)||!Finite(column.B)||!double.IsFinite(column.Radius)||column.Radius<=0)
            {uncertain=true;return;}
            double left=Lower(Lower(Math.Min(column.A.X,column.B.X)-column.Radius)-Reserve);
            double bottom=Lower(Lower(Math.Min(column.A.Y,column.B.Y)-column.Radius)-Reserve);
            double right=Upper(Upper(Math.Max(column.A.X,column.B.X)+column.Radius)+Reserve);
            double top=Upper(Upper(Math.Max(column.A.Y,column.B.Y)+column.Radius)+Reserve);
            if(!double.IsFinite(left)||!double.IsFinite(bottom)||!double.IsFinite(right)||!double.IsFinite(top))
            {uncertain=true;return;}
            if(!hasColumns){minX=left;minY=bottom;maxX=right;maxY=top;hasColumns=true;}
            else{minX=Math.Min(minX,left);minY=Math.Min(minY,bottom);maxX=Math.Max(maxX,right);maxY=Math.Max(maxY,top);}
        }
        internal bool TryUncoveredWitness(Point3D a,Point3D b,double radius,out Point3D witness)
        {
            witness=default;
            if(!IsUsable||!Finite(a)||!Finite(b)||!double.IsFinite(radius)||radius<0)return false;
            // Every tested extremum is an actual point of the demanded capsule.
            // Round its comparison toward the interior, opposite to the source
            // box. Equality and all uncertain boundary cases keep the full proof.
            var left=a.X<=b.X?a:b;double x=left.X-radius;
            if(double.IsFinite(x)&&Upper(x)<minX){witness=left with{X=x};return true;}
            var right=a.X>=b.X?a:b;x=right.X+radius;
            if(double.IsFinite(x)&&Lower(x)>maxX){witness=right with{X=x};return true;}
            var bottom=a.Y<=b.Y?a:b;double y=bottom.Y-radius;
            if(double.IsFinite(y)&&Upper(y)<minY){witness=bottom with{Y=y};return true;}
            var top=a.Y>=b.Y?a:b;y=top.Y+radius;
            if(double.IsFinite(y)&&Lower(y)>maxY){witness=top with{Y=y};return true;}
            return false;
        }
        internal string Describe()=>$"({minX:R},{minY:R},{maxX:R},{maxY:R})";
        private static double Lower(double value)=>Math.BitDecrement(value);
        private static double Upper(double value)=>Math.BitIncrement(value);
    }
    private CutterSweepStock NewStock()=>new(profile,p=>p,surface,token,mergeIncrementally:true,
        endpointCandidateDiscovery:!cylindrical&&EndpointCandidateOptimizationOverride.Value!=false);
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
