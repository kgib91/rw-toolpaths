using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling.Acceleration;

namespace RW.Toolpaths.Milling;

/// <summary>Bounded finishing nominations at reachable ramp stock missed by the initial visits.
/// Real commanded poses can suppress redundant nominations. Missing witnesses grant no negative
/// stock authority; every proposed visit still passes the original continuous physical guards.</summary>
public readonly record struct VCarveRoughingChord(Point3D A, Point3D B, double TipRadius,
    double ClearedColumnRadius = 0, double StockSurfaceRadius = 0);

internal static class VCarveRampRestNominator
{
    internal readonly record struct RestContour(IReadOnlyList<PointD> Points, bool Closed);
    internal const double CleanupTolerance = .025;
    private const double WallReserve = .002;
    /// <summary>Nominate cutter-supported boundary-parallel arcs instead of
    /// independent scanline teeth. The original physical guards still own
    /// acceptance; contour sampling and tip witnesses grant no stock proof.</summary>
    internal static List<TaggedToolpath> NominateContours(IReadOnlyList<IReadOnlyList<PointD>> boundary,
        IReadOnlyList<TaggedToolpath> guarded, VCarveCutterProfile profile, double depth, double reach,
        Func<double,double,double> floorCap, Func<double,double,double> artworkDistance,
        Func<Point3D,Point3D> command, CancellationToken token, Action<string>? log,
        IReadOnlyList<VCarveRoughingChord>? roughing,
        Func<IReadOnlyList<TaggedToolpath>,List<TaggedToolpath>> protect,
        List<TaggedToolpath> protectedOutput, double grade,double stepOver=.4,double surfaceZ=0,
        double initialBand=double.PositiveInfinity,double contourTolerance=.03,
        IReadOnlyList<MedialSegment>? preparedMedial=null,
        Func<Point3D,Point3D>? floorContact=null,double maximumFloorContactRadius=0,
        ClearedColumnStock? priorStock=null,
        Func<IReadOnlyList<PointD>,bool,IEnumerable<RestContour>>? restrictContours=null,
        bool reuseFloorCoverage=true,Action<IEnumerable<PointD>>? prepareFloorQueries=null,
        bool continuousCrestSurface=false)
    {
        long started=Stopwatch.GetTimestamp();
        double pitch=profile.RoundedTip?profile.NoseLimitedPitch(2*profile.PhysicalRadiusAtDepth(depth),stepOver)
            :Math.Min(.4,Math.Max(.01,profile.PhysicalRadiusAtDepth(depth)/4));
        double authoredFinishPitch=pitch;
        double originalRadius=profile.PhysicalRadiusAtDepth(depth);
        if(double.IsFinite(initialBand)&&initialBand>originalRadius)
            pitch=ConcentricBandSchedule.FitPitch(initialBand-originalRadius,pitch);
        // Rest inherits the authored finish texture. A second, much finer tip
        // grid must not silently demand a polish finer than the main rings.
        double finishBudget=profile.RoundedTip
            ?Math.Max(ToolpathCommandRounding.CommandIncrement,profile.PhysicalProfile.Bottom(authoredFinishPitch/2))
            :CleanupTolerance;
        // Ring spacing sets the authored texture. The allowed ramp floor can
        // also rise between adjacent fronts, so nomination needs the existing
        // longitudinal source sampling scale. That is NOT a second lateral
        // stepover: sampling a contour finely must not add extra concentric rows.
        double authoredPitch=pitch;
        double samplePitch=Math.Min(pitch,DemandPitch(profile,depth));
        if(double.IsFinite(initialBand)&&initialBand>originalRadius)
            samplePitch=ConcentricBandSchedule.FitPitch(initialBand-originalRadius,samplePitch);
        // A grade-limited floor can crest BETWEEN authored rows. Keep the
        // authored family first; nominate intervening repairs only where its
        // committed cuts and prior roughing miss the authored finish budget.
        // A grade*g half-gap bounds the unseen tip-floor rise. Refinements
        // partition the authored interval, so they cannot create a competing grid.
        // A continuous crest creates parallel slope offsets. Their surface
        // spacing is bounded by XY spacing * sqrt(1+grade^2). The old linear
        // height-gap rule was needed for isolated medial cones and can demand
        // dozens of extra rows on a simple plane. Retain it for other callers.
        double subdivisions=Math.Max(1,Math.Ceiling(continuousCrestSurface
            ? Math.Sqrt(1+grade*grade) : grade*pitch/(2*finishBudget)));
        double refinementPitch=pitch/subdivisions;
        // Charge a Euclidean surface bound, not a vertical-only height error.
        // Upward and radial allowances each use e/sqrt(2), so their combined
        // displacement is <=e even on a nearly vertical conical flank.
        double axialBudget=finishBudget/Math.Sqrt(2),radialBudget=axialBudget;
        Func<double,double> finishRadius=d=>Math.Max(0,profile.PhysicalRadiusAtDepth(d)-radialBudget);
        log?.Invoke($"stage=ramp-rest-contours event=prepare pitch={pitch:R} samplePitch={samplePitch:R} refinementPitch={refinementPitch:R} refinementSubdivisions={subdivisions:R} continuousCrestSurface={continuousCrestSurface} maximumSurfaceSpacing={refinementPitch*Math.Sqrt(1+grade*grade):R} authoredFinishPitch={authoredFinishPitch:R} reach={reach:R} sourceRings={boundary.Count} sourcePoints={boundary.Sum(r=>(long)r.Count)} guardedPoints={guarded.Sum(p=>(long)p.Points.Count)}");
        var stock=new CutterSweepStock(profile,command,surfaceZ,token);
        stock.AddProtected(guarded);stock.Seal();
        var roughStock=priorStock?.CreateQuerySession()??new ClearedColumnStock((roughing??Array.Empty<VCarveRoughingChord>())
            .Select(c=>new ClearedColumnStock.Column(c.A,c.B,c.ClearedColumnRadius)),token);
        // Protect only raises native Z and subdivides original XY chords.
        // Charge their complete float/F3 shell before early whole-chord omission.
        double radialShell=2*Math.Sqrt(2)*ToolpathCommandRounding.CommandIncrement+1e-7;
        double earlyTolerance=Math.Max(0,axialBudget-2*ToolpathCommandRounding.CommandIncrement);
        Func<double,double> paddedRadius=d=>Math.Max(0,profile.PhysicalRadiusAtDepth(d)+radialShell-radialBudget);
        // These are nominations, not certified offsets. Canonicalize once and
        // reuse the raw offset owner; the original Protect below remains the
        // physical authority. Running construction/inversion proof per trial
        // would duplicate that authority and dominate generation time.
        var literal=new Paths64(boundary.Select(ring=>new Path64(ring.Select(p=>
            new Point64((long)Math.Round(p.x*PathUtils.Scale),(long)Math.Round(p.y*PathUtils.Scale))))));
        var canonical=Clipper.Union(literal,FillRule.NonZero);
        token.ThrowIfCancellationRequested();
        log?.Invoke($"stage=ramp-rest-contours event=canonical-source points={canonical.Sum(r=>(long)r.Count)} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
        // Condition the NOM operand once, before repeated absolute offsets.
        // Original boundary, floor and commanded-wall owners are unchanged.
        var nominatedSource=canonical;
        if(RawInitialClearingFrontConditioner.TryCondition(canonical,token,log,out var sourceNomination,out _))nominatedSource=sourceNomination!;
        var offset=new ClipperOffset(miterLimit:PolygonOps.DefaultMiterLimit,arcTolerance:.001*PathUtils.Scale);
        offset.AddPaths(nominatedSource,JoinType.Round,EndType.Polygon);
        const int maximumSites=2_000_000,maximumFieldQueries=maximumSites;
        double referencePitch=DemandPitch(profile,depth);
        int crestLevels=Math.Max(3,3+(int)Math.Ceiling(Math.Log2(Math.Max(1,samplePitch/referencePitch))));
        long queries=0,sampleFieldQueries=0,crestFieldQueries=0,flatSkippedFieldQueries=0;
        long unknownWallQueries=0,unknownEndpointQueries=0;
        double NominalDepth(double x,double y,string site,bool skipShallowFloor=false)
        {
            double wall=artworkDistance(x,y);
            if(!double.IsFinite(wall)||wall<0)
            {
                unknownWallQueries++;
                if(site=="open-endpoint")unknownEndpointQueries++;
                // Contact clipping can leave an open endpoint just outside the
                // physical wall domain. Unknown clearance grants no depth;
                // retain the endpoint and valid neighbouring ramp, then let
                // original Protect certify the complete resulting chords.
                if(unknownWallQueries<=4)
                    log?.Invoke($"stage=ramp-rest-contours event=unknown-wall-clearance site={site} x={x:R} y={y:R} wall={wall:R} policy=surface-cap physicalProtectionRequired=True");
                return 0;
            }
            double wallCap=profile.DepthAtRadius(Math.Max(0,wall-WallReserve));
            return skipShallowFloor&&wallCap<=finishBudget?wallCap:Math.Min(floorCap(x,y),wallCap);
        }
        int sites=0,contours=0,retained=0,needed=0,points=0,omitted=0,crests=0,clearedCrestQueries=0,bridgedGaps=0;
        var result=new List<TaggedToolpath>();
        int region=guarded.Count>0?guarded[0].RegionIndex:0;
        log?.Invoke($"stage=ramp-rest-contours event=start pitch={pitch:R} reach={reach:R} guardedPaths={guarded.Count}");
        log?.Invoke($"stage=ramp-rest-contours event=policy maximumSites={maximumSites} maximumFieldQueries={maximumFieldQueries} crestLevels={crestLevels} fieldBudgetOwner=actual-sample-and-crest-queries maximumPoints=2000000 floorDemandTolerance={(floorContact is null?finishBudget:finishBudget+ToolpathCommandRounding.CommandIncrement):R} floorWitness={(floorContact is null?"tip-site":"physical-profile-contact")} contactFrontsRequireWholeProfile=True floorWitnessCredit=nomination-only");
        // The last conical contact is a mandatory geometric level. Uniform
        // half-pitch rows on either side can miss millimetres of side stock
        // even though their tip scallops are small. Anchor the spacing there
        // and retain every physical section transition as well.
        double contact=originalRadius+WallReserve;
        var levels=new SortedSet<double>();
        var contactLevels=new HashSet<double>();
        void AddLevel(double inset)
        {
            if(double.IsFinite(initialBand)&&Math.Abs(inset-initialBand)<=8*(Math.BitIncrement(initialBand)-initialBand))inset=initialBand;
            if(Math.Abs(inset-originalRadius)<=8*(Math.BitIncrement(originalRadius)-originalRadius))inset=originalRadius;
            levels.Add(inset);
        }
        double levelCount=Math.Ceiling(Math.Max(0,reach-originalRadius)/refinementPitch);
        if(!double.IsFinite(levelCount)||levelCount>maximumSites)
            throw new InvalidOperationException("Ramp rest contour levels exceed the bounded nomination inventory.");
        for(int index=0;index<(int)levelCount;index++)
        {
            double group=Math.Floor(index/subdivisions),part=index-group*subdivisions;
            AddLevel(originalRadius+group*pitch+part*refinementPitch);
        }
        if(reach>0)levels.Add(reach);
        if(contact<=reach){levels.Add(contact);contactLevels.Add(contact);}
        if(initialBand>0&&initialBand<=reach)levels.Add(initialBand);
        double sectionDepth=0;
        foreach(var section in profile.Sections)
        {
            sectionDepth=Math.Min(depth,sectionDepth+section.Height);
            double inset=profile.PhysicalRadiusAtDepth(sectionDepth)+WallReserve;
            if(inset>0&&inset<=reach){levels.Add(inset);contactLevels.Add(inset);}
            if(sectionDepth>=depth)break;
        }
        // A terminal ridge can lie inside even a fraction of one authored
        // interval. Its existence depends on geometry, not ring spacing.
        if(reach>originalRadius)levels.Add(double.PositiveInfinity);
        int ConstructionPhase(double inset)
        {
            if(contactLevels.Contains(inset))return 0;
            if(double.IsPositiveInfinity(inset))return 2;
            double authored=(inset-originalRadius)/authoredPitch;
            return inset==reach||inset==initialBand||Math.Abs(authored-Math.Round(authored))<1e-8?1:3;
        }
        // Establish the authored contact/fill family before discovering finer
        // floor demand. Interleaving fine offsets first makes them cut material
        // the coarse family would already finish, and forces extra connections.
        // This orders nominations only: no future unprotected cut earns credit.
        int level=0,familyLevels=0;
        ClipperOffset? contactFamily=null;
        foreach(double inset in levels.OrderBy(ConstructionPhase).ThenByDescending(d=>d))
        {
            token.ThrowIfCancellationRequested();
            long levelStarted=Stopwatch.GetTimestamp();
            double samplingMs=0,stockDecisionMs=0,protectAndCommitMs=0;
            int levelContours=0;
            log?.Invoke($"stage=ramp-rest-contours event=level-start level={level} constructionPhase={ConstructionPhase(inset)} inset={inset:R} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
            bool closed=!double.IsPositiveInfinity(inset);
            bool floorDemand=!contactLevels.Contains(inset);
            bool fromContactFamily=closed&&inset>contact&&contactFamily is not null;
            List<List<PointD>> rings;
            if(!closed)
            {
                var sourceRings=PolygonOps.MaterializeIntegerPaths(nominatedSource);
                double minimumInset=originalRadius;
                rings=preparedMedial is not null
                    ?OffsetTerminalPaths.NominateRidges(new MillingRegion(boundary[0],boundary.Skip(1).ToArray()),
                        minimumInset,reach,token,preparedMedial).Select(p=>p.ToList()).ToList()
                    :PolygonOps.DifferenceToComponents(sourceRings,Array.Empty<IReadOnlyList<PointD>>())
                    .SelectMany(c=>OffsetTerminalPaths.NominateRidges(new MillingRegion(c.Outer,
                        c.Holes.Select(h=>(IReadOnlyList<PointD>)h).ToList()),
                        minimumInset,reach,token))
                    .Select(p=>p.ToList()).ToList();
            }
            else
            {
                var raw=new Paths64();
                if(fromContactFamily){contactFamily!.Execute(-(inset-contact)*PathUtils.Scale,raw);familyLevels++;}
                else offset.Execute(-inset*PathUtils.Scale,raw);
                token.ThrowIfCancellationRequested();
                if(raw.Count==0){level++;continue;}
                if(RawInitialClearingFrontConditioner.TryCondition(raw,token,log,out var conditioned,out _))raw=conditioned!;
                rings=PolygonOps.MaterializeIntegerPaths(raw);
            }
            if(rings.Count==0){level++;continue;}
            if(closed)
            {
                // Condition the NOM curves before evaluating their floor and
                // local cutter reach. Native tick-scale offset fans otherwise
                // multiply identical physical queries and retain tiny XY turns.
                // This is the shared finite-segment RDP, not boundary or stock
                // simplification; all original physical guards still follow.
                long input=rings.Sum(r=>(long)r.Count);
                rings=rings.Select(r=>{
                    var reduced=PolygonSimplify.RdpSegmentBounded(r.Append(r[0]).ToArray(),
                        Math.Min(contourTolerance,finishBudget)/3);
                    if(reduced.Count<4)return r;
                    reduced.RemoveAt(reduced.Count-1);
                    return Math.Sign(PolygonOps.SignedArea(r))==Math.Sign(PolygonOps.SignedArea(reduced))?reduced:r;
                }).ToList();
                log?.Invoke($"stage=ramp-rest-contours event=condition-front inset={inset:R} inputPoints={input} outputPoints={rings.Sum(r=>(long)r.Count)} tolerance={Math.Min(contourTolerance,finishBudget)/3:R} originalProtectionRequired=True stockCredit=False");
            }
            if(closed&&inset>=originalRadius&&!fromContactFamily)
            {
                // The outer finish is a variable-depth cutter-contact front,
                // not the maximum-depth radius translated around the artwork.
                // Reuse the same local physical reach constructor as initial
                // clearing; original Protect still certifies every new chord.
                var fronts=rings.Select(r=>new TaggedToolpath(r.Select(p=>new Point3D(p.x,p.y,-depth))
                    .Append(new(r[0].x,r[0].y,-depth)).ToArray(),Array.Empty<ToolpathSpan>(),region,
                    VCarveInitialReachSelection.QuarterOuterMarkerCategory,null,detectCurves:false)).ToArray();
                var projected=VCarveFloorAwareOuterReach.Apply(fronts,profile,depth,floorCap,artworkDistance,
                    (x,y,d)=>profile.PhysicalRadiusAtDepth(Math.Max(0,-command(new(x,y,-d)).Z)),token,log,
                    additionalWallOffset:inset-originalRadius,samplingTolerance:finishBudget);
                rings=projected.Select(p=>p.Points.Take(p.Points.Count-1).Select(q=>new PointD(q.X,q.Y)).ToList()).ToList();
                if(inset==contact)
                {
                    // Establish local wall reach ONCE. Independently moving
                    // each inner ring makes reach-solver fallbacks and changing
                    // floor sources manufacture different lateral kinks on
                    // every level. Absolute offsets of this contact front form
                    // one coherent concentric family. These remain nominations;
                    // original floor, wall and grade protection follows.
                    var familySource=Clipper.Union(new Paths64(rings.Where(r=>r.Count>=3).Select(r=>
                        new Path64(r.Select(p=>new Point64((long)Math.Round(p.x*PathUtils.Scale),
                            (long)Math.Round(p.y*PathUtils.Scale)))))),FillRule.NonZero);
                    if(familySource.Count>0)
                    {
                        contactFamily=new ClipperOffset(miterLimit:PolygonOps.DefaultMiterLimit,arcTolerance:.001*PathUtils.Scale);
                        contactFamily.AddPaths(familySource,JoinType.Round,EndType.Polygon);
                        log?.Invoke($"stage=ramp-rest-contours event=contact-family originInset={contact:R} rings={familySource.Count} points={familySource.Sum(r=>(long)r.Count)} projectionOwner=single-wall-contact originalProtectionRequired=True stockCredit=False");
                    }
                }
            }
            var pending=new List<TaggedToolpath>();
            // Committed stock is immutable throughout this level's contour
            // walk; new protected paths enter stock only after it completes.
            // Shared medial endpoints otherwise repeat complete contact-ring
            // checks. Neither a pose nor a result crosses the next stock update.
            var floorCoverage=reuseFloorCoverage?new Dictionary<Point3D,bool>():null;
            long floorCoverageHits=0,floorCoverageMisses=0,floorCoverageClears=0;
            double geometryMs=Stopwatch.GetElapsedTime(levelStarted).TotalMilliseconds;
            var contourSource=rings.SelectMany(r=>restrictContours?.Invoke(r,closed)??new[]{new RestContour(r,closed)});
            if(prepareFloorQueries is not null&&ConstructionPhase(inset)>=2)
            {
                var preparedContours=contourSource.ToArray();
                prepareFloorQueries(SampleQueries(preparedContours));contourSource=preparedContours;
            }
            foreach(var contour in contourSource)
            {
                var ring=contour.Points;bool ringClosed=contour.Closed;
                if(ring.Count<(ringClosed?3:2))continue;contours++;
                levelContours++;
                if((levelContours&4095)==0)log?.Invoke($"stage=ramp-rest-contours event=level-progress level={level} contours={levelContours} sites={sites} queries={queries} "
                    +$"geometryMs={geometryMs:F1} samplingMs={samplingMs:F1} stockDecisionMs={stockDecisionMs:F1} stockQueries={stock.Queries} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
                long sampleStarted=Stopwatch.GetTimestamp();
                var sampled=new List<Point3D>();var required=new List<bool>();bool missing=false;
                for(int i=0;i<ring.Count-(ringClosed?0:1);i++)
                {
                    var a=ring[i];var b=ring[(i+1)%ring.Count];
                    int n=Math.Max(1,(int)Math.Ceiling(Math.Sqrt(Math.Pow(b.x-a.x,2)+Math.Pow(b.y-a.y,2))/samplePitch));
                    for(int j=0;j<n;j++)
                    {
                        token.ThrowIfCancellationRequested();
                        if(++sites>maximumSites)throw new InvalidOperationException($"Ramp rest contours exceed their nomination-site inventory: sites={sites}, sampleQueries={sampleFieldQueries}, crestQueries={crestFieldQueries}, flatSkipped={flatSkippedFieldQueries}, level={level}.");
                        double x=a.x+(b.x-a.x)*j/n,y=a.y+(b.y-a.y)*j/n;
                        // A complete rougher cell at maximum depth has no rest
                        // demand. Leave its nominal depth to original Protect
                        // if the surrounding contour is needed; avoid querying
                        // the expensive floor twice over already cleared flats.
                        var full=command(new(x,y,-depth));
                        bool flatCleared=roughStock.Contains(full,full,finishRadius,axialBudget);
                        if(flatCleared)
                        {flatSkippedFieldQueries++;sampled.Add(new(x,y,-depth));continue;}
                        ChargeFieldQuery(crest:false);
                        double cap=NominalDepth(x,y,"sample",skipShallowFloor:true);
                        var point=new Point3D(x,y,-cap);sampled.Add(point);
                    }
                }
                if(!ringClosed)
                {
                    ChargeFieldQuery(crest:false);
                    var p=ring[^1];
                    sampled.Add(new(p.x,p.y,-NominalDepth(p.x,p.y,"open-endpoint",skipShallowFloor:true)));
                }
                // The allowed floor can crest between contour samples. A
                // protector can raise a chord but cannot invent its missing
                // deeper contact. Nominate those contacts before stock tests.
                var enriched=new List<Point3D>{sampled[0]};
                var maximumCovered=new List<bool>();
                double Target(double x,double y)
                {
                    ChargeFieldQuery(crest:true);
                    return NominalDepth(x,y,"crest");
                }
                for(int i=0;i<sampled.Count-(ringClosed?0:1);i++)Enrich(sampled[i],sampled[(i+1)%sampled.Count],0);
                if(ringClosed)enriched.RemoveAt(enriched.Count-1);sampled=enriched;
                void Enrich(Point3D a,Point3D b,int at)
                {
                    token.ThrowIfCancellationRequested();
                    if(at==0)
                    {
                        var fullA=command(a with{Z=-depth});var fullB=command(b with{Z=-depth});
                        if(roughStock.Contains(fullA,fullB,paddedRadius,earlyTolerance)
                            ||!floorDemand&&stock.Contains(fullA,fullB,earlyTolerance,radialShell-radialBudget))
                        {clearedCrestQueries++;enriched.Add(b);maximumCovered.Add(true);return;}
                    }
                    var mid=new Point3D((a.X+b.X)/2,(a.Y+b.Y)/2,(a.Z+b.Z)/2);
                    double cap=Target(mid.X,mid.Y);
                    if(at<crestLevels&&cap>-mid.Z+.005)
                    {crests++;mid=mid with{Z=-cap};Enrich(a,mid,at+1);Enrich(mid,b,at+1);}
                    else {enriched.Add(b);maximumCovered.Add(false);}
                    if(enriched.Count>2_000_000-points)throw new InvalidOperationException("Ramp rest crests exceed the point inventory.");
                }
                required.AddRange(Enumerable.Repeat(false,sampled.Count));
                samplingMs+=Stopwatch.GetElapsedTime(sampleStarted).TotalMilliseconds;
                long stockStarted=Stopwatch.GetTimestamp();
                for(int i=0;i<sampled.Count-(ringClosed?0:1);i++)
                {
                    // The complete maximum-depth cutter envelope was already
                    // covered; every protected shallower pose is its subset.
                    if(maximumCovered[i])continue;
                    int next=(i+1)%sampled.Count;var a=command(sampled[i]);var b=command(sampled[next]);
                    // Floor fills nominate unmet physical floor sites. Boundary
                    // contact fronts separately own full-profile wall/transition
                    // coverage. A floor-site witness grants no volume credit.
                    bool covered=false;
                    if(floorDemand)
                    {
                        covered=FloorCovered(a)&&FloorCovered(b);
                    }
                    if(!covered)covered=stock.Contains(a,b,earlyTolerance,radialShell-radialBudget)
                        ||roughStock.Contains(a,b,paddedRadius,earlyTolerance);
                    if(!covered){required[i]=required[next]=true;missing=true;needed++;}
                }
                stockDecisionMs+=Stopwatch.GetElapsedTime(stockStarted).TotalMilliseconds;
                if(!missing)continue;
                // Retain maximal demanded arcs with one contour neighbour at
                // each end. The shared source circuit builder owns connections;
                // extending every repair by a full cutter radius adds cutting
                // merely to make it connectable and duplicates that ownership.
                int count=sampled.Count;var keep=required.ToArray();
                // Follow a gap shorter than one local ring departure and
                // return before routing can turn it into tiny excursions.
                bridgedGaps+=ContourDemand.BridgeGaps(sampled,keep,ringClosed,2*samplePitch);
                if(!ringClosed)
                {
                    List<Point3D>? run=null;
                    for(int k=0;k<count;k++)
                    {
                        if(keep[k]){run??=k>0?new(){sampled[k-1]}:new();run.Add(sampled[k]);}
                        else if(run is not null){run.Add(sampled[k]);Commit(run);run=null;}
                    }
                    if(run is not null)Commit(run);
                    continue;
                }
                int anchor=Array.FindIndex(keep,k=>!k);
                if(anchor<0){sampled.Add(sampled[0]);Commit(sampled);}
                else
                {
                    List<Point3D>? run=null;
                    for(int i=1;i<=count;i++)
                    {
                        int k=(anchor+i)%count;
                        if(keep[k]){run??=new(){sampled[(k+count-1)%count]};run.Add(sampled[k]);}
                        else if(run is not null){run.Add(sampled[k]);Commit(run);run=null;}
                    }
                }
                void Commit(List<Point3D> run)
                {
                    // Reduce the proposed geometry before physical protection,
                    // using the shared 3D contour conditioner. The original
                    // floor/wall/grade guards certify every resulting chord.
                    // These are proposed repair curves, not source corners.
                    // Preserve their finite-segment error bound instead of
                    // anchoring every tiny nomination kink as a design feature.
                    run=MedialAxisToolpaths.RdpSimplify3D(run,Math.Min(contourTolerance,finishBudget),preserveSharpCorners:false);
                    points+=run.Count;
                    if(points>2_000_000)throw new InvalidOperationException("Ramp rest contours exceed two million vertices; split this operation.");
                    var path=new TaggedToolpath(run,Array.Empty<ToolpathSpan>(),region,"clearing",null,detectCurves:false);
                    pending.Add(path);
                }
                bool FloorCovered(Point3D pose)
                {
                    if(floorCoverage is not null&&floorCoverage.TryGetValue(pose,out bool cached))
                    {floorCoverageHits++;return cached;}
                    floorCoverageMisses++;
                    bool result=FloorCoveredCore(pose);
                    if(floorCoverage is not null)
                    {
                        if(floorCoverage.Count==131072){floorCoverage.Clear();floorCoverageClears++;}
                        floorCoverage.Add(pose,result);
                    }
                    return result;
                }
                bool FloorCoveredCore(Point3D pose)
                {
                    if(floorContact is null)return stock.CoversFloorTarget(pose,finishBudget,true);
                    if(stock.Contains(pose,pose,axialBudget,-radialBudget))return true;
                    double tolerance=finishBudget+ToolpathCommandRounding.CommandIncrement;
                    var contact=floorContact(pose);
                    if(!stock.CoversFloorTarget(contact,tolerance,true)||!stock.CoversFloorTarget(pose,tolerance,true))return false;
                    double radius=Math.Sqrt(Math.Pow(contact.X-pose.X,2)+Math.Pow(contact.Y-pose.Y,2));
                    if(radius>1e-9&&!stock.CoversFloorContactRing(new(pose.X,pose.Y,contact.Z),radius,tolerance))return false;
                    double maximum=maximumFloorContactRadius;
                    if(maximum>1e-9&&Math.Abs(maximum-radius)>1e-9)
                    {
                        double z=pose.Z+profile.PhysicalProfile.Bottom(maximum);
                        if(!stock.CoversFloorContactRing(new(pose.X,pose.Y,z),maximum,tolerance))return false;
                    }
                    if(maximum>1e-9)
                    {
                        double middle=maximum/2,z=pose.Z+profile.PhysicalProfile.Bottom(middle);
                        if(!stock.CoversFloorContactRing(new(pose.X,pose.Y,z),middle,tolerance))return false;
                    }
                    return true;
                }
            }
            if(pending.Count>0)
            {
                long protectStarted=Stopwatch.GetTimestamp();
                var accepted=protect(pending);
                if(accepted.Count!=pending.Count)throw new InvalidOperationException("Contour protection changed source ordinals.");
                for(int row=0;row<accepted.Count;row++)
                {
                    var path=accepted[row];bool covered=true;
                    for(int i=1;i<path.Points.Count&&covered;i++)
                    {
                        var a=command(path.Points[i-1]);var b=command(path.Points[i]);
                        covered=roughStock.Contains(a,b,finishRadius,axialBudget)
                            ||stock.Contains(a,b,axialBudget,-radialBudget);
                    }
                    if(covered){omitted++;continue;}
                    result.Add(pending[row]);protectedOutput.Add(path);retained++;
                    stock.AddProtected(new[]{path});
                }
                stock.Seal();
                protectAndCommitMs=Stopwatch.GetElapsedTime(protectStarted).TotalMilliseconds;
            }
            log?.Invoke($"stage=ramp-rest-contours event=level level={level} constructionPhase={ConstructionPhase(inset)} inset={inset:R} contours={rings.Count} retained={pending.Count} sites={sites} queries={queries} sampleQueries={sampleFieldQueries} crestQueries={crestFieldQueries} flatSkipped={flatSkippedFieldQueries} "
                +$"geometryMs={geometryMs:F1} samplingMs={samplingMs:F1} stockDecisionMs={stockDecisionMs:F1} protectAndCommitMs={protectAndCommitMs:F1} "
                +$"floorCoverageMemoHits={floorCoverageHits} floorCoverageMemoMisses={floorCoverageMisses} floorCoverageMemoClears={floorCoverageClears} floorCoverageMemoLimit=131072 "
                +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
            level++;
        }
        log?.Invoke($"stage=ramp-rest-contours event=complete contours={contours} retained={retained} bridgedContourGaps={bridgedGaps} omittedWholeStockArcs={omitted} neededSites={needed} points={points} queries={queries} unknownWallQueries={unknownWallQueries} unknownOpenEndpointQueries={unknownEndpointQueries} crestNominations={crests} clearedCrestQueries={clearedCrestQueries} pitch={pitch:R} samplePitch={samplePitch:R} authoredPitch={authoredPitch:R} authoredStepOver={stepOver:R} initialBand={initialBand:R} surfaceDisplacementBound={finishBudget:R} axialAllowance={axialBudget:R} radialAllowance={radialBudget:R} stockChords={stock.Chords} stockQueries={stock.Queries} stockAccepted={stock.Accepted} stockUnionAccepted={stock.UnionAccepted} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} lateralPitchOwner=authored-stepover longitudinalSamplingSeparate=True cutterSupportedContourArcs=True originalProtection=True newTipOnlyStockCredit=False wholeProfileCommittedStock=True omissionScope=surface-distance-not-vertical-height");
        log?.Invoke($"stage=ramp-rest-contours event=floor-targets sites={sites} queries={stock.FloorTargetQueries} covered={stock.FloorTargetsCovered} endpointSites={stock.FloorSites} endpointIndexHits={stock.FloorTargetIndexHits} chordFallbacks={stock.FloorTargetChordFallbacks} contactRings={stock.FloorRingQueries} coveredRings={stock.FloorRingCovered} ringBudgetFallbacks={stock.FloorRingBudgetFallbacks} concentricFamilyLevels={familyLevels} queryMs={stock.FloorTargetMilliseconds:F3} endpointBuildMs={stock.FloorIndexBuildMilliseconds:F3} chordBuildMs={stock.StockIndexBuildMilliseconds:F3} "
            +$"priorWholeChordTrials={stock.PriorWholeChordTrials} priorWholeChordProofs={stock.PriorWholeChordProofs} priorFloorPoseTrials={stock.PriorFloorPoseTrials} priorFloorPoseProofs={stock.PriorFloorPoseProofs} "
            +$"indexMerges={stock.MergeCount} reindexedChords={stock.ReindexedChords} maximumIndexBlocks={stock.MaximumBlocks} indexBlockLimit=8 indexPartitionVisits={stock.PartitionVisits} stockCredit=False contactFrontCredit=False");
        return result;

        IEnumerable<PointD> SampleQueries(IEnumerable<RestContour> input)
        {
            foreach(var contour in input)
            {
                var ring=contour.Points;if(ring.Count<(contour.Closed?3:2))continue;
                PointD? first=null,previous=null;
                for(int i=0;i<ring.Count-(contour.Closed?0:1);i++)
                {
                    var a=ring[i];var b=ring[(i+1)%ring.Count];
                    int count=Math.Max(1,(int)Math.Ceiling(Math.Sqrt((b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y))/samplePitch));
                    for(int j=0;j<count;j++)
                    {
                        token.ThrowIfCancellationRequested();var p=new PointD(a.x+(b.x-a.x)*j/count,a.y+(b.y-a.y)*j/count);
                        yield return p;if(previous is {} old)yield return new((old.x+p.x)/2,(old.y+p.y)/2);
                        first??=p;previous=p;
                    }
                }
                if(!contour.Closed)
                {
                    var p=ring[^1];yield return p;
                    if(previous is {} old)yield return new((old.x+p.x)/2,(old.y+p.y)/2);
                }
                else if(previous is {} last&&first is {} begin)yield return new((last.x+begin.x)/2,(last.y+begin.y)/2);
            }
        }

        void ChargeFieldQuery(bool crest)
        {
            if(crest)crestFieldQueries++;else sampleFieldQueries++;
            if(++queries>maximumFieldQueries)
                throw new InvalidOperationException($"Ramp rest field queries exceed their bounded inventory: queries={queries}, limit={maximumFieldQueries}, sites={sites}, sampleQueries={sampleFieldQueries}, crestQueries={crestFieldQueries}, flatSkipped={flatSkippedFieldQueries}, level={level}.");
        }
    }
    private static double DemandPitch(VCarveCutterProfile profile,double depth)
        =>Math.Min(.4,Math.Max(.01,profile.PhysicalRadiusAtDepth(depth)/4));

    internal static List<TaggedToolpath> NominateRasterReference(IReadOnlyList<IReadOnlyList<PointD>> boundary,
        IReadOnlyList<TaggedToolpath> guarded, VCarveCutterProfile profile, double depth, double reach,
        Func<double, double, double> floorCap, Func<double, double, double> artworkDistance,
        Func<Point3D, Point3D> command, CancellationToken token, Action<string>? log,
        IReadOnlyList<VCarveRoughingChord>? roughing = null, double grade = .5773502691896257,
        Func<IReadOnlyList<TaggedToolpath>, List<TaggedToolpath>>? protect = null,
        List<TaggedToolpath>? protectedOutput = null)
    {
        const double residual = CleanupTolerance;
        double pitch = DemandPitch(profile,depth);
        double cellRadius = pitch / Math.Sqrt(2);
        const int maximumQueries = 2_000_000, maximumPoints = 500_000;
        long started = Stopwatch.GetTimestamp();
        log?.Invoke($"stage=ramp-rest-nomination event=start guardedPaths={guarded.Count} roughingChords={roughing?.Count ?? 0} "
            + $"reach={reach:R} pitch={pitch:R} maximumQueries={maximumQueries} maximumPoints={maximumPoints}");
        var poses = new Dictionary<(long, long), Point3D>();
        foreach (var path in guarded)
        for (int i = 1; i < path.Points.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var a = command(path.Points[i - 1]); var b = command(path.Points[i]);
            AddChord(poses, a, b, pitch, token);
        }
        var index = new PreparedPointIndex(poses.Values.ToArray(), token);
        var roughIndices = (roughing ?? Array.Empty<VCarveRoughingChord>()).GroupBy(chord => chord.TipRadius)
            .Where(group => double.IsFinite(group.Key) && group.Key > 0).Select(group => {
                var snapshot = new Dictionary<(long, long), Point3D>();
                foreach (var chord in group)
                {
                    token.ThrowIfCancellationRequested();
                    AddChord(snapshot, chord.A, chord.B, pitch, token);
                }
                return (Radius: group.Key, Index: new PreparedPointIndex(snapshot.Values.ToArray(), token));
            }).ToArray();
        var result = new List<TaggedToolpath>();
        var pending = new List<TaggedToolpath>();
        int committedRows = 0;
        int queries = 0, needed = 0, points = 0, crests = 0; long witnesses = 0;
        double witnessX = 0, witnessY = 0, witnessDepth = 0;
        Func<Point3D, bool> poseWitness = p => -p.Z - profile.PhysicalProfile.Bottom(
            Math.Sqrt(Math.Pow(p.X - witnessX, 2) + Math.Pow(p.Y - witnessY, 2))) >= witnessDepth;
        double minY = boundary.SelectMany(r => r).Min(p => p.y), maxY = boundary.SelectMany(r => r).Max(p => p.y);
        double y0 = Math.Ceiling(minY / pitch) * pitch;
        int region = guarded.Count > 0 ? guarded[0].RegionIndex : 0;
        for (double y = y0; y < maxY; y += pitch)
        {
            token.ThrowIfCancellationRequested();
            var events = new List<(double X, int Delta)>();
            foreach (var ring in boundary)
            for (int i = 0; i < ring.Count; i++)
            {
                var a = ring[i]; var b = ring[(i + 1) % ring.Count];
                if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y))
                    events.Add((a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y), b.y > a.y ? 1 : -1));
            }
            events.Sort((a, b) => a.X.CompareTo(b.X));
            int winding = 0; double start = 0;
            for (int i = 0; i < events.Count; i++)
            {
                int next = winding + events[i].Delta;
                if (winding == 0 && next != 0) start = events[i].X;
                if (winding != 0 && next == 0) Scan(start, events[i].X, y);
                winding = next;
            }
            CommitRow();
        }
        log?.Invoke($"stage=ramp-rest-nomination event=complete queries={queries} missingPoseWitnesses={needed} realPoseWitnesses={witnesses} "
            + $"retainedActualPoses={index.Count} roughingPoseGroups={roughIndices.Length} paths={result.Count} points={points} crestNominations={crests} pitch={pitch:R} nominationResidual={residual:R} roughingOverlapRadius={cellRadius:R} "
            + $"committedRows={committedRows} streamedProtection={protect is not null} newTipOnlyStockCredit=False "
            + $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} "
            + "scope=sampled-reachable-tip-demand wholeStockCertificate=False missingWitnessMeansRetain=True originalContinuousFloorArtworkGradeAndStepdownRequired=True");
        return result;

        void CommitRow()
        {
            if (pending.Count == 0) return;
            if (protect is null) { pending.Clear(); return; }
            var accepted = protect(pending);
            if (accepted.Count != pending.Count)
                throw new InvalidOperationException("Incremental rest protection changed source ordinals.");
            protectedOutput?.AddRange(accepted);
            pending.Clear(); committedRows++;
        }

        void Scan(double lo, double hi, double y)
        {
            List<Point3D>? run = null;
            int lastRequired = -1, idle = 0;
            double x0 = Math.Ceiling((lo + .001) / pitch) * pitch;
            for (double x = x0; x < hi - .001; x += pitch)
            {
                token.ThrowIfCancellationRequested();
                if (++queries > maximumQueries) throw new InvalidOperationException("Ramp rest nomination exceeds its bounded grid; split this operation.");
                // A real roughing disk at the complete floor proves every
                // possible shallower tip demand here, before floor/wall work.
                // This avoids querying the finishing field over cleared flats.
                bool clearedFlat = false;
                if (run == null)
                foreach (var rough in roughIndices)
                {
                    clearedFlat = rough.Radius >= cellRadius
                        && rough.Index.AnyWithin(x, y, rough.Radius - cellRadius, -depth + residual, token);
                    if (clearedFlat) break;
                }
                if (clearedFlat) { witnesses++; continue; }
                double wall = artworkDistance(x, y);
                bool missing = false;
                double target = 0;
                if (double.IsFinite(wall) && wall >= 0 && wall <= reach)
                {
                    double cap = floorCap(x, y);
                    target = Math.Min(cap, profile.DepthAtRadius(Math.Max(0, wall - .002)));
                    double cellDemand = Math.Min(depth, cap + grade * cellRadius + .002);
                    if (target > .003)
                    {
                        missing = true;
                        foreach (var rough in roughIndices)
                        {
                            if (rough.Radius >= cellRadius && rough.Index.AnyWithin(x, y, rough.Radius - cellRadius,
                                -cellDemand + residual, token))
                            { missing = false; witnesses++; }
                            if (!missing) break;
                        }
                        witnessX = x; witnessY = y; witnessDepth = target - residual;
                        double witnessRadius = profile.PhysicalRadiusAtDepth(Math.Max(0, depth - target + residual));
                        if (missing && index.AnyWithin(x, y,
                            witnessRadius, -target + residual, token,
                            poseWitness))
                        { missing = false; witnesses++; }
                    }
                }
                var requested = new Point3D(x, y, -target);
                if (run != null && (target <= .003 || Math.Abs(run[^1].Z - requested.Z) > grade * pitch)) Flush();
                if (missing)
                {
                    needed++;
                    run ??= new List<Point3D>();
                    run.Add(requested); lastRequired = run.Count - 1; idle = 0;
                }
                else if (run != null)
                {
                    run.Add(requested);
                    if (++idle >= 3) Flush();
                }
            }
            Flush();
            void Flush()
            {
                if (run == null) return;
                if (run.Count > lastRequired + 1) run.RemoveRange(lastRequired + 1, run.Count - lastRequired - 1);
                if (run.Count == 1)
                {
                    var q = run[0]; Point3D best = q; double bestAllowed = double.NegativeInfinity;
                    foreach (var direction in new[] { (1d, 0d), (-1d, 0d), (0d, 1d), (0d, -1d) })
                    for (double length = pitch; length >= pitch / 8; length /= 2)
                    {
                        double x = q.X + length * direction.Item1, yy = q.Y + length * direction.Item2;
                        double wall = artworkDistance(x, yy);
                        if (!double.IsFinite(wall) || wall < 0) continue;
                        double cap = Math.Min(floorCap(x, yy), profile.DepthAtRadius(Math.Max(0, wall - .002)));
                        double attainable = Math.Min(-q.Z, cap + grade * length);
                        if (attainable > bestAllowed) { bestAllowed = attainable; best = new(x, yy, -Math.Min(-q.Z, cap)); }
                        if (attainable >= -q.Z - .002) break;
                    }
                    if (best == q) { run = null; return; }
                    run.Add(best);
                }
                // A crest between depth samples must be nominated deeper.
                // The floor protector can only raise a chord; it cannot invent
                // the deeper midpoint that the original slope envelope permits.
                var enriched = new List<Point3D> { run[0] };
                for (int i = 1; i < run.Count; i++) Enrich(run[i - 1], run[i], 0);
                run = enriched;
                points += run.Count;
                if (points > maximumPoints) throw new InvalidOperationException("Ramp rest nomination exceeds its bounded point inventory; split this operation.");
                var proposal = new TaggedToolpath(run.ToArray(), Array.Empty<ToolpathSpan>(), region, "clearing", null, detectCurves: false);
                result.Add(proposal); pending.Add(proposal); run = null; lastRequired = -1; idle = 0;

                void Enrich(Point3D a, Point3D b, int level)
                {
                    token.ThrowIfCancellationRequested();
                    var mid = new Point3D((a.X + b.X) / 2, (a.Y + b.Y) / 2, (a.Z + b.Z) / 2);
                    double wall = artworkDistance(mid.X, mid.Y);
                    double cap = double.IsFinite(wall) && wall >= 0
                        ? Math.Min(floorCap(mid.X, mid.Y), profile.DepthAtRadius(Math.Max(0, wall - .002))) : 0;
                    if (level < 3 && cap > -mid.Z + .005)
                    {
                        crests++; mid = mid with { Z = -cap };
                        Enrich(a, mid, level + 1); Enrich(mid, b, level + 1);
                    }
                    else enriched.Add(b);
                    if (enriched.Count > maximumPoints - points)
                        throw new InvalidOperationException("Ramp rest crest nominations exceed the bounded point inventory.");
                }
            }
        }
    }
    // Both inventories retain actual poses on affine posted chords. Keeping
    // only the deepest real pose in an XY bin can add nominations, never invent
    // removed stock. Unsupported/budget cases fail visibly rather than truncate.
    private static void AddChord(Dictionary<(long, long), Point3D> poses, Point3D a, Point3D b,
        double pitch, CancellationToken token)
    {
        double pieces = Math.Max(1, Math.Ceiling(Math.Sqrt(Math.Pow(b.X - a.X, 2) + Math.Pow(b.Y - a.Y, 2)) / pitch));
        if (!double.IsFinite(pieces) || pieces > 2_000_000)
            throw new InvalidOperationException("Ramp rest chord sampling exceeds its bounded inventory.");
        int n = (int)pieces;
        for (int j = 0; j <= n; j++)
        {
            if ((j & 1023) == 0) token.ThrowIfCancellationRequested();
            var p = new Point3D(a.X + (b.X - a.X) * j / n, a.Y + (b.Y - a.Y) * j / n, a.Z + (b.Z - a.Z) * j / n);
            if (p.Z >= 0) continue;
            var key = ((long)Math.Round(p.X * 10), (long)Math.Round(p.Y * 10));
            if (!poses.TryGetValue(key, out var old) || p.Z < old.Z) poses[key] = p;
            if (poses.Count > 2_000_000)
                throw new InvalidOperationException("Ramp rest actual-pose inventory exceeds two million sites.");
        }
    }

}
