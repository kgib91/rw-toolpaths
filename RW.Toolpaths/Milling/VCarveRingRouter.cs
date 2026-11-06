using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Offset-front routing: finish a ready ring, step to an adjacent ready
/// ring at cutting feed, or retain an independent entry. Protected source cuts
/// and the constructor's parent/depth dependencies are never removed.</summary>
internal static class VCarveRingRouter
{
    // The ready independent winner may have a cheaper, physically proved
    // connection that its former local row hid. False retains a control.
    internal static readonly AsyncLocal<bool?> IndependentEntryContinuationOverride=new();
    internal static readonly AsyncLocal<bool?> BodyCostCacheOverride=new();
    internal static readonly AsyncLocal<bool?> CostPrecheckOverride=new();
    internal static readonly AsyncLocal<bool?> IndependentEntryMemoOverride=new();
    internal static readonly AsyncLocal<bool?> SpatialMixedSourcesOverride=new();
    private readonly record struct Proposal(TaggedToolpath Actual,TaggedToolpath Entered,Point3D Target,
        double Score,double Saving,bool SurfaceStep,bool Raised);

    // Shared by every normal return family for ONE actual ready visit. Check
    // before MoveNext: advancing the lazy ancestor iterator can itself perform
    // expensive physical proofs even before it yields another departure.
    internal sealed class ReturnNominationBudget
    {
        private readonly int maximum;
        private readonly CancellationToken token;
        internal int Used {get;private set;}
        internal bool Refused {get;private set;}
        internal ReturnNominationBudget(int maximum,CancellationToken token)
        {
            if(maximum<0)throw new ArgumentOutOfRangeException(nameof(maximum));
            this.maximum=maximum;this.token=token;
        }
        internal bool CanSearch()
        {
            token.ThrowIfCancellationRequested();
            if(Used<maximum)return true;
            Refused=true;return false;
        }
        internal IEnumerable<T> Take<T>(IEnumerable<T> source)
        {
            if(!CanSearch())yield break;
            using var iterator=source.GetEnumerator();
            while(CanSearch())
            {
                if(!iterator.MoveNext())yield break;
                Used++;yield return iterator.Current;
            }
        }
    }
    internal static List<TaggedToolpath> Route(IReadOnlyList<TaggedToolpath> source,ToolGeometry tool,
        DepthSchedule depth,VCarveOptions options,Point3D? start,Func<Point3D,Point3D,bool> guard,
        Func<Point3D,double>? entryHeight,CancellationToken token,Action<string>? log,
        Func<IReadOnlyList<TaggedToolpath>,Point3D?,List<TaggedToolpath>> finishOtherSources)
    {
        long started=Stopwatch.GetTimestamp();
        if(options.MaximumContourSeamCandidates is <1 or >48)
            throw new ArgumentOutOfRangeException(nameof(options),"Contour seam candidate budget must be between 1 and 48.");
        if(options.MaximumContourReturnCandidates<0)
            throw new ArgumentOutOfRangeException(nameof(options),"Contour return candidate budget must be nonnegative.");
        var rings=source.Where(p=>p.ConcentricSource is not null).ToArray();
        var remaining=source.Where(p=>p.ConcentricSource is null).ToArray();
        var profile=VCarvePhysicalSweep.ProfileFor(tool,depth.Depth);
        double materialSurface=options.Motion is {} machine&&double.IsFinite(machine.MaterialSurfaceZ)?machine.MaterialSurfaceZ:depth.SurfaceZ;
        if(options.OptimizeContourConnections)
            {
            var originalRings=rings;
            MillingGeometryTrace.Capture("ring-required-work",source[0].RegionIndex,rings);
            rings=VCarveRingDemand.Required(rings,profile,depth.SurfaceZ,materialSurface,options.PathToleranceMm,token,log);
            bool multipleLayers=depth.DepthPerPass>0&&depth.DepthPerPass<depth.Depth-1e-9
                ||rings.Any(p=>p.DepthPassIndex is >0);
            if(multipleLayers)
                log?.Invoke("stage=ring-fragment-demand event=retained reason=multiple-depth-layers futureDonorsCannotReplaceRequiredStepdownPredecessors=True");
            else {
                double fragmentLength=profile.RoundedTip?profile.NoseLimitedPitch(2*profile.PhysicalRadiusAtDepth(depth.Depth),options.StepOver)
                    :2*profile.PhysicalRadiusAtDepth(depth.Depth)*options.StepOver;
                rings=VCarveRingDemand.WithoutCoveredFragments(rings,remaining,profile,depth.SurfaceZ,materialSurface,
                    options.PathToleranceMm,fragmentLength,token,log);
                rings=VCarveRingDemand.WithoutCoveredRuns(rings,remaining,originalRings,profile,depth,options,token,log);
                rings=VCarveWallSources.LocalStrips(rings,profile,depth,options,token,log);
            }
            MillingGeometryTrace.Capture("ring-retained-work",source[0].RegionIndex,rings);
        }
        if(options.PreserveContourWallIdentity)
            rings=VCarveWallSources.Assign(rings,token,log).ToArray();
        if(rings.Length==0)return remaining.Length==0?[]:finishOtherSources(remaining,start);
        double step=depth.DepthPerPass>0?depth.DepthPerPass:depth.Depth;
        double grade=Math.Tan(options.SmoothRampAngleDegrees*Math.PI/180);
        double maximumRadial=profile.RoundedTip?profile.NoseLimitedPitch(2*profile.PhysicalRadiusAtDepth(depth.Depth),options.StepOver)
            :2*profile.PhysicalRadiusAtDepth(depth.Depth)*options.StepOver;
        var order=new VCarveRingOrder(rings,options.ProfileDirection==VCarveProfileDirection.BottomUp,token,maximumRadial,
            options.OptimizeContourConnections?VCarveRingOrder.SpatialSweepOverride.Value:null);
        var seams=new VCarveRingSeams(rings,token);
        var ramp=options.Ramp with{Strategy=RampStrategy.Linear,
            AngleRadians=Math.Min(options.Ramp.AngleRadians>0?options.Ramp.AngleRadians:double.PositiveInfinity,
                options.SmoothRampAngleDegrees*Math.PI/180)};
        var cost=options.Motion is null?null:new ToolpathMotionCost(options.Motion,depth.SurfaceZ,chordGeometryCacheLimit:131072);
        var enteredVisits=cost is null||IndependentEntryMemoOverride.Value==false?null:new ToolpathEnteredVisitMemo(cost);
        var result=new List<TaggedToolpath>(source.Count);Point3D? cursor=start;
        bool hasOwnNonfastCostContext=false;
        var mixed=VCarveRingSpatialDispatch.Create(remaining,source,depth,options,
            options.OptimizeContourConnections?VCarveRingOrder.SpatialSweepOverride.Value:null,token,log);
        var visitOwners=new List<int>(rings.Length);
        int connected=0,entries=0,trials=0,engagementRefused=0,corridorRefused=0,costRefused=0,exitCustodyRefusals=0;
        int independentWinnerAttempts=0,independentWinnerOutsideLocal=0,independentWinnerAccepted=0,
            independentWinnerReturned=0,independentWinnerLandings=0,independentWinnerReturnDepartures=0,
            independentWinnerCostRefusals=0,independentWinnerNoProposal=0,independentWinnerBudgetRefusals=0;
        double independentWinnerSaving=0;
        int returned=0,returnCandidates=0,surfaceSteps=0,raisedSteps=0,provedParentReturns=0;long returnedChords=0;
        long returnNominations=0,returnNominationBudgetRefusals=0;
        int maximumReturnNominationsObserved=0;
        double returnedSeconds=0;TaggedToolpath? lastSource=null;
        bool hasCarrier=options.UpperContourCarrier is not null;
        var returns=new VCarveRingReturns(token,hasCarrier||options.OptimizeContourConnections,
            options.OptimizeContourConnections?(a,b)=>guard(a with{Z=a.Z-depth.SurfaceZ},b with{Z=b.Z-depth.SurfaceZ}):null);
        bool exitAnchored=true;
        double transfer=0,saved=0;
        (int Owner,TaggedToolpath Path)? closestIndependent=null;
        int nearestReentries=0;
        int landingWorkRefusals=0;
        bool continuous=options.PreferContinuousContourTraversal;
        bool optimize=options.OptimizeContourConnections;
        var independentCosts=new Dictionary<IReadOnlyList<Point3D>,(TaggedToolpath Path,double Complete,double Body)>(ReferenceEqualityComparer.Instance);
        long independentCostHits=0,independentCostMisses=0;
        long returnBudgetEntryLookups=0,returnBudgetPriceHits=0;
        long costPrecheckGuardSkips=0,additionalPureCosts=0,costPrecheckComparisons=0,
            costPrecheckCacheMisses=0,costPrecheckTransferFallbacks=0;
        bool useCostPrecheck=CostPrecheckOverride.Value!=false;
        // The complete immutable value includes spans and depth/source tags;
        // point-list identity alone cannot distinguish Cut from fast travel.
        var originalBodies=new HashSet<TaggedToolpath>(rings);
        var bodyCosts=new Dictionary<TaggedToolpath,double>();var derivedBodyCosts=new Dictionary<TaggedToolpath,double>();
        long bodyCostHits=0,bodyCostMisses=0,bodyCostUncached=0,bodyCostBoundRefusals=0,derivedBodyChords=0;
        double BodySeconds(TaggedToolpath path){
            if(cost is null)return 0;
            if(BodyCostCacheOverride.Value==false){bodyCostUncached++;return cost.VisitSeconds(path);}
            bool original=originalBodies.Contains(path);var cache=original?bodyCosts:derivedBodyCosts;
            if(cache.TryGetValue(path,out double seconds)){bodyCostHits++;return seconds;}
            bodyCostMisses++;seconds=cost.VisitSeconds(path);
            if(original||derivedBodyCosts.Count<128&&derivedBodyChords+path.Points.Count-1<=262144){
                cache.Add(path,seconds);if(!original)derivedBodyChords+=path.Points.Count-1;
            }
            else bodyCostBoundRefusals++;
            return seconds;
        }
        void ResetDepartureCosts(){
            independentCosts.Clear();derivedBodyCosts.Clear();derivedBodyChords=0;
        }
        int landingLimit=options.MaximumContourSeamCandidates;
        long landingTrials=0;int maximumLandingTrialsObserved=0;
        int[] activeCandidates=[];
        int seamAlternatives=0,independentAlternatives=0;
        Point3D Command(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),
            ToolpathCommandRounding.Round((float)p.Y),ToolpathCommandRounding.Round((float)p.Z-(float)depth.SurfaceZ));
        double skim=materialSurface+MillingLocalTravel.Clearance;
        double commandedMaterialSurface=(float)materialSurface-(float)depth.SurfaceZ;
        bool AboveMaterial(Point3D a,Point3D b)=>a.Z>=skim&&b.Z>=skim
            &&Command(a).Z>=commandedMaterialSurface+MillingLocalTravel.Clearance&&Command(b).Z>=commandedMaterialSurface+MillingLocalTravel.Clearance;
        bool SurfaceCorridor(Point3D a,Point3D b)=>hasCarrier&&AboveMaterial(a,b);
        bool ParentStep(Point3D a,Point3D b)
        {
            double engaged=2*profile.PhysicalRadiusAtDepth(Math.Max(0,depth.SurfaceZ-Math.Min(a.Z,b.Z)));
            double radial=profile.RoundedTip?profile.NoseLimitedPitch(engaged,options.StepOver):options.StepOver*engaged;
            return ToolpathEngagementLimits.Allows(a,b,radial,step,grade,Command)
                &&guard(a with{Z=a.Z-depth.SurfaceZ},b with{Z=b.Z-depth.SurfaceZ});
        }
        TaggedToolpath SurfaceTravel(TaggedToolpath path)
        {
            if(!hasCarrier)return path;
            var kinds=ToolpathSpanKinds.Expand(path);bool changed=false;
            for(int i=1;i<path.Points.Count;i++)if(kinds[i-1]==ToolpathSpanKind.Link&&AboveMaterial(path.Points[i-1],path.Points[i]))
            {kinds[i-1]=ToolpathSpanKind.ClearedTravel;changed=true;}
            if(!changed)return path;
            var spans=new List<ToolpathSpan>();int first=0;
            for(int i=1;i<=kinds.Length;i++)if(i==kinds.Length||kinds[i]!=kinds[first]){spans.Add(new(first,i,kinds[first]));first=i;}
            return path with{Spans=spans};
        }
        bool carrierReady=false;
        if(options.UpperContourCarrier is {} carrier)
        {
            if(carrier.Points.Count<3||carrier.Points[0]!=carrier.Points[^1]||carrier.RegionIndex!=rings[0].RegionIndex
                ||carrier.Spans.Any(s=>s.Kind!=ToolpathSpanKind.ClearedTravel)
                ||Enumerable.Range(1,carrier.Points.Count-1).Any(i=>!AboveMaterial(carrier.Points[i-1],carrier.Points[i])
                    ||!guard(carrier.Points[i-1] with{Z=carrier.Points[i-1].Z-depth.SurfaceZ},carrier.Points[i] with{Z=carrier.Points[i].Z-depth.SurfaceZ})))
                throw new InvalidOperationException("An upper contour carrier requires nominal/F3 whole-material clearance and the original corridor proof.");
            // Immutable original-material clearance proves this entire carrier
            // before any cut. Its return arcs need no ceremonial initial lap.
            // The first emitted connector starts at this proved carrier pose.
            if(!optimize)result.Add(carrier);
            returns.RecordCarrier(carrier);cursor=carrier.Points[^1];lastSource=carrier;carrierReady=true;
            log?.Invoke($"stage=ring-routing event=upper-carrier points={carrier.Points.Count} initialCircuitEmitted={!optimize} clearanceMm={MillingLocalTravel.Clearance:R} materialSurfaceZ={materialSurface:R} authority=above-original-material-surface nativeAndF3=True futureStockCredit=False");
        }
        bool ExitAnchored(TaggedToolpath path){
            bool anchored=true;
            for(int i=1;i<path.Points.Count;i++){
                var a=path.Points[i-1];var b=path.Points[i];
                bool vertical=ToolpathCommandRounding.IsVerticalOnly((float)b.X-(float)a.X,(float)b.Z-(float)a.Z,(float)b.Y-(float)a.Y);
                if(vertical&&(a.X!=b.X||a.Y!=b.Y))anchored=false;
                else if(!vertical)anchored=true;
            }
            return anchored;
        }
        bool mixedFailed=false;
        bool DispatchOther(bool ringsRemain){
            if(mixed is null||!mixed.Pending)return false;
            double frontier=Math.Min(ringsRemain?order.UnfinishedSpatialFrontier:double.PositiveInfinity,mixed.Frontier);
            order.SetSpatialExternalFrontier(frontier);
            var bundle=mixed.Select(frontier,cursor,token);
            if(bundle is null||ringsRemain&&exitAnchored&&order.HasStickySpatialContinuation(cursor,frontier))return false;
            double otherAccess=cursor is null?Math.Pow(bundle.Key-frontier,2):bundle.AccessDistanceSquared(cursor);
            if(ringsRemain&&order.ReadySpatialAccessDistanceSquared(cursor,frontier)<=otherAccess+1e-9)return false;
            token.ThrowIfCancellationRequested();
            var actual=finishOtherSources(bundle.Sources,cursor).Where(p=>p.Points.Count>1).ToArray();
            if(actual.Length==0){
                mixed.EmptyOutput();mixedFailed=true;
                log?.Invoke($"stage=ring-spatial-dispatch event=fallback reason=empty-entered-bundle-output sourceCount={bundle.Sources.Length} "
                    +"trialOutputsDiscarded=True retainedOriginalSourceFallback=ring-only-spatial-route");return false;
            }
            foreach(var path in actual)result.Add(path);
            hasOwnNonfastCostContext|=actual.Any(ToolpathMotionCost.HasNonfastChord);
            var last=actual[^1];cursor=last.Points[^1];lastSource=last;exitAnchored=ExitAnchored(last);
            var cut=last.Spans.LastOrDefault(s=>s.Kind==ToolpathSpanKind.Cut);
            bool anchoredCut=exitAnchored&&cut.MoveCount>0&&cut.EndIndex==last.Points.Count-1;
            returns.Clear();if(anchoredCut)returns.Record(last,null);
            ResetDepartureCosts();closestIndependent=null;activeCandidates=[];
            mixed.Complete(bundle,actual.Length,anchoredCut);return true;
        }
        List<TaggedToolpath> FallbackMixed(){
            var previous=SpatialMixedSourcesOverride.Value;
            try{SpatialMixedSourcesOverride.Value=false;
                return Route(source,tool,depth,options,start,guard,entryHeight,token,log,finishOtherSources);
            }finally{SpatialMixedSourcesOverride.Value=previous;}
        }
        TaggedToolpath Enter(TaggedToolpath path,double? connectedHeight=null)
        {
            double top=connectedHeight??(entryHeight is null?depth.SurfaceZ:Math.Max(path.Points[0].Z,entryHeight(path.Points[0])));
            var entered=RampPlanner.PlanPathEntry(path.Points,top,ramp,token,null,
                landAtSourceVertex:true,preserveClosedExit:false,preserveOpenExit:false);
            return path with{Points=entered.Points,Moves=new LineToolMoves(entered.Points),Spans=entered.Spans};
        }
        (TaggedToolpath Path,TaggedToolpath Entered,ToolpathEnteredVisitMemo.Entry? Cached) IndependentEntry(int owner,int index)
        {
            token.ThrowIfCancellationRequested();var original=rings[owner];
            if(enteredVisits is null){var path=VCarveContinuationAdapter.Rebase(original,index);return(path,Enter(path),null);}
            // Match Rebase's literal first pose without rebuilding the body.
            // Every opaque height query still runs before the memo lookup.
            int first=index;
            if(index==0||original.Spans.Any(s=>s.Kind!=ToolpathSpanKind.Cut))first=0;
            else if(original.Points[0]==original.Points[^1])first%=original.Points.Count-1;
            var point=original.Points[first];
            double top=entryHeight is null?depth.SurfaceZ:Math.Max(point.Z,entryHeight(point));
            token.ThrowIfCancellationRequested();
            if(enteredVisits.TryGet(owner,index,top,out var cached))return(cached!.Source,cached.Entered,cached);
            var rebased=VCarveContinuationAdapter.Rebase(original,index);var entered=Enter(rebased,top);
            cached=enteredVisits.Store(owner,index,top,rebased,entered);return(cached.Source,cached.Entered,cached);
        }
        double IndependentPrice(Point3D origin,(TaggedToolpath Path,TaggedToolpath Entered,ToolpathEnteredVisitMemo.Entry? Cached) entry)
            =>entry.Cached is { } cached?enteredVisits!.CompleteVisitSeconds(origin,cached)
                :cost?.CompleteVisitSeconds(origin,entry.Entered)??double.PositiveInfinity;
        double ReturnEntryBudget(int owner,Point3D origin)
        {
            var entry=IndependentEntry(owner,ToolpathVisitOrder.EntryIndex(rings[owner],origin,token));
            returnBudgetEntryLookups++;long hits=enteredVisits?.PricedHits??0;
            double complete=IndependentPrice(origin,entry);
            returnBudgetPriceHits+=(enteredVisits?.PricedHits??0)-hits;
            return complete-BodySeconds(entry.Path)+.01;
        }
        Proposal? Propose(int owner,TaggedToolpath original,TaggedToolpath path,Point3D origin,IReadOnlyList<Point3D> trace,TaggedToolpath departedFrom)
        {
            var departure=trace[^1];
            // The offset constructor selected the next row. Its finishing
            // surface/layer owns the authored grade; independent entries and
            // unrelated surfaces retain the common gentler entry ramp.
            bool surfaceStep=departedFrom.ConcentricSource is {} a&&original.ConcentricSource is {} b
                &&ReferenceEquals(a.Plan,b.Plan)&&departedFrom.DepthPassIndex==original.DepthPassIndex;
            var transitionRamp=surfaceStep?ramp with{AngleRadians=options.SmoothRampAngleDegrees*Math.PI/180}:ramp;
            bool surfaceDeparture=hasCarrier&&AboveMaterial(departure,departure);
            var firstTarget=surfaceDeparture?path.Points[0] with{Z=departure.Z}:RampPlanner.PlanTransitionEntry(departure,path.Points[0],transitionRamp);
            double run=Math.Sqrt(Math.Pow(departure.X-firstTarget.X,2)+Math.Pow(departure.Y-firstTarget.Y,2));
            double deepest=Math.Max(0,depth.SurfaceZ-Math.Min(departure.Z,firstTarget.Z));
            double maximum=profile.RoundedTip?profile.NoseLimitedPitch(2*profile.PhysicalRadiusAtDepth(deepest),options.StepOver)
                :2*profile.PhysicalRadiusAtDepth(deepest)*options.StepOver;
            if(!SurfaceCorridor(departure,firstTarget)&&run>maximum+1e-12){engagementRefused++;return null;}
            bool EntryGrade(Point3D a,Point3D b)
            {
                double permitted=b.Z<a.Z?Math.Tan(transitionRamp.AngleRadians):grade;
                bool Check(Point3D x,Point3D y)=>Math.Abs(x.Z-y.Z)<=permitted*
                    Math.Sqrt(Math.Pow(x.X-y.X,2)+Math.Pow(x.Y-y.Y,2))+1e-12;
                return Check(a,b)&&Check(Command(a),Command(b));
            }
            double roof=Math.Max(path.Points[0].Z,Math.Min(depth.SurfaceZ,departure.Z+grade*run));
            var heights=new[]{firstTarget.Z,RampPlanner.PlanTransitionEntry(departure,path.Points[0],ramp).Z,
                Math.Max(departure.Z,path.Points[0].Z),Math.Max(path.Points[0].Z,(departure.Z+roof)/2),roof}.Distinct();
            Proposal? best=null;
            (TaggedToolpath Path,double Complete,double Body) Independent()
            {
                if(independentCosts.TryGetValue(original.Points,out var cached)){independentCostHits++;return cached;}
                var entry=IndependentEntry(owner,ToolpathVisitOrder.EntryIndex(original,origin,token));
                var path=entry.Path;
                var value=(path,IndependentPrice(origin,entry),BodySeconds(path));
                independentCosts.Add(original.Points,value);independentCostMisses++;return value;
            }
            double Length(TaggedToolpath p)=>Enumerable.Range(1,p.Points.Count-1).Sum(i=>ToolpathContinuationPlanner.Distance(p.Points[i-1],p.Points[i]));
            foreach(double height in heights)
            {
                // Use the entry planner's exact source pose when its existing
                // contact test chooses an immediate Cut. A few arithmetic ULPs
                // above it are not a distinct ramp endpoint.
                var target=height<=path.Points[0].Z+1e-9?path.Points[0]:firstTarget with{Z=height};
                for(int adjustment=0;adjustment<2&&!EntryGrade(departure,target);adjustment++)
                    target=target with{Z=target.Z<departure.Z?Math.Min(departure.Z,target.Z+.001):Math.Max(path.Points[0].Z,target.Z-.001)};
                if(target.Z<path.Points[0].Z)continue;
                double engaged=2*profile.PhysicalRadiusAtDepth(Math.Max(0,depth.SurfaceZ-Math.Min(departure.Z,target.Z)));
                double radial=profile.RoundedTip?profile.NoseLimitedPitch(engaged,options.StepOver):options.StepOver*engaged;
                trials++;
                if(!EntryGrade(departure,target)||!SurfaceCorridor(departure,target)&&!ToolpathEngagementLimits.Allows(departure,target,radial,step,grade,Command))
                {engagementRefused++;continue;}
                (TaggedToolpath Path,double Complete,double Body)? precheckedIndependent=null;
                TaggedToolpath? precheckedEntered=null,precheckedCandidate=null;
                double? precheckedAfter=null;
                if(useCostPrecheck&&!continuous&&cost is not null)
                {
                    token.ThrowIfCancellationRequested();
                    // An independent cache miss can query the actual stock's
                    // entry height; retain its original guard-first ordering.
                    // A cached comparison already belongs to this departure.
                    if(independentCosts.TryGetValue(original.Points,out var knownIndependent))
                    {
                        var earlyEntered=Enter(path,target.Z);
                        var earlyCandidate=SurfaceTravel(ToolpathContinuationPlanner.Prepend(earlyEntered,trace.Append(target).ToArray()));
                        // CompleteVisitSeconds has no transfer at this literal
                        // origin. VisitSeconds is pure motion pricing and cannot
                        // call a prior-stock travel guard. Other contexts retain
                        // the original full transfer-price/guard sequence.
                        if(MillingTransitionPlanner.IsContinuous(origin,earlyCandidate.Points[0]))
                        {
                            double earlyAfter=cost.VisitSeconds(earlyCandidate);
                            costPrecheckComparisons++;
                            if(!(earlyAfter<=knownIndependent.Complete+1e-9))
                            {costRefused++;costPrecheckGuardSkips++;continue;}
                            precheckedIndependent=knownIndependent;precheckedEntered=earlyEntered;
                            precheckedCandidate=earlyCandidate;precheckedAfter=earlyAfter;
                        }
                        else costPrecheckTransferFallbacks++;
                    }
                    else costPrecheckCacheMisses++;
                }
                if(!guard(departure with{Z=departure.Z-depth.SurfaceZ},target with{Z=target.Z-depth.SurfaceZ}))
                {if(precheckedAfter.HasValue)additionalPureCosts++;corridorRefused++;continue;}
                var independent=precheckedIndependent??Independent();
                if(precheckedIndependent.HasValue)independentCostHits++;
                var entered=precheckedEntered??Enter(path,target.Z);
                var candidate=precheckedCandidate??SurfaceTravel(ToolpathContinuationPlanner.Prepend(entered,trace.Append(target).ToArray()));
                double before=independent.Complete,after=precheckedAfter??cost?.CompleteVisitSeconds(origin,candidate)??0;
                bool cheaper=after<=before+1e-9;
                if(!continuous&&!cheaper)
                {costRefused++;continue;}
                double score=cost is null?Length(candidate)-Length(independent.Path):after-independent.Body;
                if(best is not null&&score>=best.Value.Score)continue;
                best=new(candidate,entered,target,score,double.IsFinite(before)?before-after:0,surfaceStep,target.Z>firstTarget.Z+1e-9);
            }
            return best;
        }
        (int Owner,Proposal Plan)? Nearest(IReadOnlyList<Point3D> trace,Point3D origin,TaggedToolpath departedFrom,bool rememberIndependent=false,
            Func<bool>? reserveLanding=null)
        {
            int attempted=0;
            (int Owner,Proposal Plan)? best=null;
            double reach=hasCarrier&&AboveMaterial(trace[^1],trace[^1])?double.PositiveInfinity:maximumRadial;
            foreach(var landing in seams.Nearest(trace[^1],activeCandidates,reach,landingLimit+1))
            {
                // Only the current constructed row participates. Choose its
                // closest legal seam without changing the offset progression.
                if(++attempted>landingLimit){landingWorkRefusals++;break;}
                if(reserveLanding is not null&&!reserveLanding()){independentWinnerBudgetRefusals++;break;}
                landingTrials++;maximumLandingTrialsObserved=Math.Max(maximumLandingTrialsObserved,attempted);
                var path=seams.Rebase(landing,(a,b)=>guard(a with{Z=a.Z-depth.SurfaceZ},b with{Z=b.Z-depth.SurfaceZ}));
                if(path is not {} prepared)continue;
                if(rememberIndependent&&closestIndependent is null)closestIndependent=(landing.Owner,prepared);
                if(Propose(landing.Owner,rings[landing.Owner],prepared,origin,trace,departedFrom) is not {} plan)continue;
                if(!optimize)return(landing.Owner,plan);
                seamAlternatives++;
                if(best is null||plan.Score<best.Value.Plan.Score-1e-9)best=(landing.Owner,plan);
            }
            return best;
        }
        for(int visit=0;visit<rings.Length;visit++)
        {
            while(DispatchOther(ringsRemain:true)){}
            if(mixedFailed)return FallbackMixed();
            if(mixed is not null)order.SetSpatialExternalFrontier(Math.Min(order.UnfinishedSpatialFrontier,mixed.Frontier));
            if(cost is not null)cost.LocalTransferEstimatesEnabled=hasOwnNonfastCostContext;
            ResetDepartureCosts();
            if(visit%128==0)log?.Invoke($"stage=ring-routing event=progress visits={visit}/{rings.Length} optimizeConnections={optimize} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
            token.ThrowIfCancellationRequested();var candidates=order.Candidates(cursor);
            activeCandidates=candidates; // The actual departure and readiness remain fixed until Complete.
            closestIndependent=null;
            int selected=candidates[0];TaggedToolpath? selectedPath=null;double selectedCost=double.PositiveInfinity,selectedSaving=0;
            TaggedToolpath? selectedEntry=null;Point3D[]? selectedConnector=null;
            VCarveRingReturns.Departure? selectedReturn=null;
            bool selectedSurfaceStep=false,selectedRaised=false;
            if((result.Count>0||carrierReady)&&!exitAnchored)exitCustodyRefusals++;
            if((result.Count>0||carrierReady)&&exitAnchored&&cursor is {} origin&&Nearest(new[]{origin},origin,lastSource!.Value,true) is {} direct)
            {
                var proposal=direct.Plan;
                selected=direct.Owner;selectedPath=proposal.Actual;selectedEntry=proposal.Entered;
                selectedConnector=new[]{origin,proposal.Target};selectedCost=proposal.Score;selectedSaving=proposal.Saving;
                selectedSurfaceStep=proposal.SurfaceStep;selectedRaised=proposal.Raised;
            }
            // An accepted adjacent-row step already continues the constructed
            // strip. Search the completed-parent stack when that direct step
            // is unavailable, rather than re-exploring every ancestor per row.
            if(selectedPath is null&&(result.Count>0||carrierReady)&&exitAnchored&&cursor is {} actual&&cost is not null)
            {
            var returnBudget=new ReturnNominationBudget(options.MaximumContourReturnCandidates,token);
            var triedReturns=new HashSet<(VCarveRingReturns.Node,int,double)>();
            double localBudget=candidates.Max(owner=>ReturnEntryBudget(owner,actual));
            if(continuous)localBudget=double.PositiveInfinity;
            foreach(var departure in returnBudget.Take(returns.LocalDepartures(cost,localBudget)))
            {
                triedReturns.Add((departure.Anchor,departure.Portal,departure.PortalFraction));returnCandidates++;
                if(Nearest(departure.Points,actual,departure.Anchor.Source) is not {} localPlan)continue;
                var proposal=localPlan.Plan;if(proposal.Score>=selectedCost)continue;
                selected=localPlan.Owner;selectedPath=proposal.Actual;selectedEntry=proposal.Entered;selectedReturn=departure;
                selectedConnector=new[]{departure.Points[^1],proposal.Target};selectedCost=proposal.Score;selectedSaving=proposal.Saving;
                selectedSurfaceStep=proposal.SurfaceStep;selectedRaised=proposal.Raised;
            }
            foreach(int owner in candidates)
            {
                if(!returnBudget.CanSearch())break;
                var original=rings[owner];
                double budget=ReturnEntryBudget(owner,actual);
                if(continuous)budget=double.PositiveInfinity;
                foreach(var departure in returnBudget.Take(returns.Departures(original,cost,budget,maximumRadial,
                    hasCarrier||optimize?ParentStep:null)))
                {
                    if(!triedReturns.Add((departure.Anchor,departure.Portal,departure.PortalFraction)))continue;
                    returnCandidates++;
                    if(Nearest(departure.Points,actual,departure.Anchor.Source) is not {} returnedPlan)continue;
                    var proposal=returnedPlan.Plan;if(proposal.Score>=selectedCost)continue;
                    selected=returnedPlan.Owner;selectedPath=proposal.Actual;selectedEntry=proposal.Entered;selectedReturn=departure;
                    selectedConnector=new[]{departure.Points[^1],proposal.Target};selectedCost=proposal.Score;selectedSaving=proposal.Saving;
                    selectedSurfaceStep=proposal.SurfaceStep;
                    selectedRaised=proposal.Raised;
                }
            }
            returnNominations+=returnBudget.Used;
            maximumReturnNominationsObserved=Math.Max(maximumReturnNominationsObserved,returnBudget.Used);
            if(returnBudget.Refused)returnNominationBudgetRefusals++;
            }
            bool connectedThisVisit=selectedPath is not null;
            if(selectedPath is not {} routed)
            {
                // A new strip requires an independent entry. Its nearest ready
                // piece need not be the next distant outline in the CAD tree.
                closestIndependent=null;
                if(optimize&&cost is not null){
                    double best=double.PositiveInfinity;
                    foreach(int owner in order.IndependentCandidates()){
                        var original=rings[owner];
                        // Open interiors duplicate an arm. Compare the literal
                        // endpoints, including ramp recovery and transfer cost.
                        IEnumerable<int> indices=original.Points[0]!=original.Points[^1]
                            ?new[]{0,original.Points.Count-1}
                            :cursor is {} site?new[]{ToolpathVisitOrder.EntryIndex(original,site,token)}:new[]{0};
                        foreach(int index in indices){
                            token.ThrowIfCancellationRequested();independentAlternatives++;
                            var entry=IndependentEntry(owner,index);
                            var from=cursor??entry.Entered.Points[0] with{Z=options.Motion!.SafeZ};
                            double score=IndependentPrice(from,entry)-BodySeconds(original);
                            if(score<best-1e-9){best=score;closestIndependent=(owner,entry.Path);}
                        }
                    }
                }
                else if(cursor is {} independentFrom)
                foreach(var landing in seams.Nearest(independentFrom,order.IndependentCandidates(),double.PositiveInfinity,landingLimit))
                    if(seams.Rebase(landing,(a,b)=>guard(a with{Z=a.Z-depth.SurfaceZ},b with{Z=b.Z-depth.SurfaceZ})) is {} independentPath)
                    {closestIndependent=(landing.Owner,independentPath);break;}
                TaggedToolpath path;
                if(closestIndependent is {} closest)
                {
                    selected=closest.Owner;path=closest.Path;nearestReentries++;
                    // An open fragment can be entered at its nearer endpoint
                    // without replaying an extra arm. Its nearest ROW still
                    // owns the choice when the cutting step was refused.
                    if(!optimize&&rings[selected].Points[0]!=rings[selected].Points[^1]&&cursor is {} endFrom)
                        path=VCarveContinuationAdapter.Rebase(rings[selected],ToolpathVisitOrder.EntryIndex(rings[selected],endFrom,token));
                }
                else {path=rings[selected];if(cursor is {} from)
                    path=VCarveContinuationAdapter.Rebase(path,ToolpathVisitOrder.EntryIndex(path,from,token));
                }
                routed=Enter(path);
                if(IndependentEntryContinuationOverride.Value!=false&&optimize&&cost is not null
                    &&(result.Count>0||carrierReady)&&exitAnchored&&cursor is {} independentOrigin)
                {
                    // This exact target already won the ordinary ready-job entry
                    // comparison. Retain its owner and all depth prerequisites;
                    // do not rescan the global candidate set or reorder a layer.
                    if(!order.IsReady(selected))throw new InvalidOperationException("Independent continuation target is not ready.");
                    independentWinnerAttempts++;
                    if(!candidates.Contains(selected))independentWinnerOutsideLocal++;
                    double independentSeconds=cost.CompleteVisitSeconds(independentOrigin,routed),bestSeconds=independentSeconds;
                    int remainingLandings=24;
                    bool ReserveLanding(){
                        token.ThrowIfCancellationRequested();
                        if(remainingLandings==0)return false;
                        remainingLandings--;independentWinnerLandings++;return true;
                    }
                    Proposal? winner=null;VCarveRingReturns.Departure? winnerReturn=null;Point3D[]? winnerConnector=null;
                    var oldCandidates=activeCandidates;activeCandidates=new[]{selected};
                    void Consider((int Owner,Proposal Plan)? candidate,VCarveRingReturns.Departure? departure)
                    {
                        if(candidate is not {} nomination)return;
                        if(nomination.Owner!=selected)throw new InvalidOperationException("Independent continuation changed the selected owner.");
                        double seconds=cost.CompleteVisitSeconds(independentOrigin,nomination.Plan.Actual);
                        if(seconds>=bestSeconds-1e-9){independentWinnerCostRefusals++;return;}
                        bestSeconds=seconds;winner=nomination.Plan;winnerReturn=departure;
                        winnerConnector=new[]{departure?.Points[^1]??independentOrigin,nomination.Plan.Target};
                    }
                    try
                    {
                        Consider(Nearest(new[]{independentOrigin},independentOrigin,lastSource!.Value,reserveLanding:ReserveLanding),null);
                        // Match the normal cheap-direct preference. A fixed small
                        // set of real completed stack departures is considered
                        // only if that direct proposal did not beat this entry.
                        if(winner is null&&remainingLandings>0)
                        {
                            double budget=Math.Max(0,independentSeconds-BodySeconds(rings[selected])+.01);
                            var tried=new HashSet<(VCarveRingReturns.Node,int,double)>();
                            void TryDepartures(IEnumerable<VCarveRingReturns.Departure> departures)
                            {
                                foreach(var departure in departures.Take(4))
                                {
                                    token.ThrowIfCancellationRequested();if(remainingLandings==0)break;
                                    if(!tried.Add((departure.Anchor,departure.Portal,departure.PortalFraction)))continue;
                                    independentWinnerReturnDepartures++;
                                    Consider(Nearest(departure.Points,independentOrigin,departure.Anchor.Source,reserveLanding:ReserveLanding),departure);
                                }
                            }
                            TryDepartures(returns.LocalDepartures(cost,budget));
                            if(remainingLandings>0)
                                TryDepartures(returns.Departures(rings[selected],cost,budget,maximumRadial,ParentStep));
                        }
                    }
                    finally{activeCandidates=oldCandidates;}
                    if(winner is {} continuation)
                    {
                        connectedThisVisit=true;routed=continuation.Actual;selectedEntry=continuation.Entered;
                        selectedConnector=winnerConnector;selectedReturn=winnerReturn;
                        selectedSaving=independentSeconds-bestSeconds;selectedSurfaceStep=continuation.SurfaceStep;selectedRaised=continuation.Raised;
                        independentWinnerAccepted++;independentWinnerSaving+=selectedSaving;
                        if(winnerReturn is not null)independentWinnerReturned++;
                    }
                    else independentWinnerNoProposal++;
                }
                if(!connectedThisVisit)
                {
                    entries++;
                    if(cursor is {} departure)transfer+=ToolpathContinuationPlanner.Distance(departure,routed.Points[0]);
                    returns.Record(routed,null);
                }
            }
            if(connectedThisVisit)
            {
                connected++;saved+=selectedSaving;
                returns.Record(selectedEntry!.Value,selectedConnector,selectedReturn?.Anchor,selectedReturn?.Portal??0,selectedReturn?.PortalFraction??0);
                if(selectedReturn is not null){returned++;returnedChords+=selectedReturn.Points.Count-1;returnedSeconds+=selectedReturn.Seconds;provedParentReturns+=selectedReturn.ProvedParentSteps;}
                if(selectedSurfaceStep)surfaceSteps++;
                if(selectedRaised)raisedSteps++;
            }
            result.Add(routed);cursor=routed.Points[^1];order.Complete(selected);
            hasOwnNonfastCostContext|=ToolpathMotionCost.HasNonfastChord(routed);
            visitOwners.Add(selected);
            lastSource=rings[selected];
            exitAnchored=ExitAnchored(routed);
        }
        while(DispatchOther(ringsRemain:false)){}
        if(mixedFailed)return FallbackMixed();
        if(mixed is {Pending:true})throw new InvalidOperationException("A mixed spatial sweep left an intact source bundle unfinished.");
        if(cost is not null)cost.LocalTransferEstimatesEnabled=hasOwnNonfastCostContext;
        if(mixed is not null)log?.Invoke($"stage=ring-spatial-dispatch event=complete {mixed.Summary()} "
            +$"ringVisits={rings.Length} maximumCommittedDispatches={rings.Length+mixed.Bundles.Count} originalSourcesRetained=True");
        log?.Invoke($"stage=ring-routing event=complete rings={rings.Length} otherSources={remaining.Length} "
            +$"connected={connected} independentEntries={entries} trials={trials} engagementRefused={engagementRefused} "
            +$"corridorRefused={corridorRefused} costRefused={costRefused} exitCustodyRefusals={exitCustodyRefusals} transferDistance={transfer:R} selectedSavingSec={saved:R} "
            +$"returnCandidates={returnCandidates} returnedVisits={returned} returnedChords={returnedChords} "
            +$"returnedMotionSec={returnedSeconds:R} surfaceSteps={surfaceSteps} raisedSteps={raisedSteps} "
            +$"returnWorkRefusals={returns.BoundedSearches+returnNominationBudgetRefusals} "
            +$"returnNominationBudgetRefusals={returnNominationBudgetRefusals} returnNominations={returnNominations} "
            +$"maximumReturnNominationsPerVisit={options.MaximumContourReturnCandidates} maximumReturnNominationsObserved={maximumReturnNominationsObserved} "
            +"returnBudgetScope=total-normal-local-and-ancestor-families iteratorAdvanceAfterBudget=False independentWinnerBudgetUnchanged=True "
            +$"partialAxisReturnRefusals={returns.PartialAxisRefusals} postedEquivalentReturnSteps={returns.PostedEquivalentSteps} "
            +$"provedParentReturnSteps={provedParentReturns} "
            +$"parentProjectionTrials={returns.ParentProjectionTrials} parentProjectionRefusals={returns.ParentProjectionRefusals} fractionalDepartures={returns.FractionalDepartures} "
            +$"parentProjectionEdgesExamined={returns.ParentProjectionEdgesExamined} "
            +$"preferContinuousTraversal={continuous} upperCarrier={hasCarrier} returnCostGate={!continuous} "
            +$"optimizeConnections={optimize} seamAlternatives={seamAlternatives} independentAlternatives={independentAlternatives} "
            +$"direction={options.ProfileDirection} localOffsetAndDepthDependencies=True actualExitCursor=True sourceCutsPreserved=True "
            +$"constructedRows={order.RowCount} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
            +$"sourceWalls={order.WallCount} wallLevelBatches={order.WallLevelBatches} wallOrder={(order.WallCount>0?"complete-feature-offset-level":"legacy-parent-pieces")} "
            +$"offsetPieceQueries={order.ParentQueries} offsetPieceBoundsRefusals={order.ParentBoundsRefusals} "
            +$"splitLandingCandidates={seams.SplitLandings} splitLandingRefusals={seams.SplitRefusals} "
            +$"seamBoundedQueries={seams.BoundedQueries} seamCandidateChordsExamined={seams.CandidateChordsExamined} seamCandidateNominations={seams.CandidateNominations} "
            +$"seamCandidatesRetainedForSort={seams.CandidatesRetainedForSort} seamCandidatesDiscardedBeforeSort={seams.CandidatesDiscardedBeforeSort} seamOrder=literal-stable-prefix "
            +seams.IndexDiagnostics+" "
            +$"nearestRowReentries={nearestReentries} "
            +$"landingWorkRefusals={landingWorkRefusals} maximumLandingTrialsPerDeparture={landingLimit} "
            +$"seamLandingTrials={landingTrials} maximumLandingTrialsObserved={maximumLandingTrialsObserved} "
            +$"independentCostCacheHits={independentCostHits} independentCostCacheMisses={independentCostMisses} cacheScope=one-actual-departure "
            +$"returnBudgetEntryLookups={returnBudgetEntryLookups} returnBudgetPriceHits={returnBudgetPriceHits} "
            +$"costPrecheckEnabled={useCostPrecheck} costPrecheckComparisons={costPrecheckComparisons} costPrecheckGuardSkips={costPrecheckGuardSkips} "
            +$"additionalPureCosts={additionalPureCosts} costPrecheckCacheMisses={costPrecheckCacheMisses} costPrecheckTransferFallbacks={costPrecheckTransferFallbacks} "
            +"costPrecheckOrdering=cached-independent-zero-transfer-price-before-full-physical-guard acceptedCandidatesRequireFullGuard=True "
            +$"bodyCostCacheHits={bodyCostHits} bodyCostCacheMisses={bodyCostMisses} bodyCostUncached={bodyCostUncached} bodyCostCacheBoundRefusals={bodyCostBoundRefusals} "
            +"bodyCostCacheScope=route-originals-and-one-departure-derived-complete-path-values maximumDerivedBodyCacheEntries=128 maximumDerivedBodyCacheChords=262144 "
            +$"independentWinnerContinuation={IndependentEntryContinuationOverride.Value!=false} independentWinnerAttempts={independentWinnerAttempts} "
            +$"independentWinnerOutsideLocal={independentWinnerOutsideLocal} independentWinnerAccepted={independentWinnerAccepted} independentWinnerReturned={independentWinnerReturned} "
            +$"independentWinnerLandings={independentWinnerLandings} independentWinnerReturnDepartures={independentWinnerReturnDepartures} "
            +$"independentWinnerCostRefusals={independentWinnerCostRefusals} independentWinnerNoProposal={independentWinnerNoProposal} independentWinnerBudgetRefusals={independentWinnerBudgetRefusals} "
            +$"independentWinnerSavingSec={independentWinnerSaving:R} independentWinnerMaximumLandings=24 independentWinnerMaximumReturnDepartures=8 independentWinnerDepthEligibilityUnchanged=True "
            +order.SpatialSummary()+" "
            +"nextContourPolicy=constructed-offsets-per-trimmed-strip localSeamPolicy=closest-finite-segment orderOwner=RingOffsetEngine "
            +"surfaceStepAuthority=same-constructor-plan-layer-next-row "
            +"newChordAuthority=original-native-F3-floor-wall-grade executedChordGraph=False");
        if(optimize&&cost is not null&&remaining.Length==0&&!order.SpatialSweepEnabled){
            result=VCarveRingTour.Improve(result,visitOwners,order.Dependencies(),cost,options.Motion!,start,token,log);
            cursor=result.Count>0?result[^1].Points[^1]:cursor;
        }
        if(remaining.Length>0&&mixed is null)result.AddRange(finishOtherSources(remaining,cursor));
        bool completedReturns=VCarvePathRouter.CutRetraceOptimizationOverride.Value??true;
        if(optimize&&options.Motion is {} returnMotion&&(completedReturns||result.Any(p=>p.Spans.Any(s=>s.Kind==ToolpathSpanKind.Link)))){
            result=VCarveReturnTravel.Improve(result,profile,depth.SurfaceZ,returnMotion,grade,guard,token,log,materialSurface,start,
                optimizeCutRetraces:completedReturns,trimCompletedExits:completedReturns);
            if(result.Any(p=>p.Spans.Any(s=>s.Kind==ToolpathSpanKind.ClearedTravel)))
                VCarveReturnTravel.VerifyFinalTravel(result,profile,depth.SurfaceZ,token,log,materialSurface,start);
        }
        long chords=0;double maximumExcess=0;
        foreach(var path in result)
        for(int i=1;i<path.Points.Count;i++)
        {
            if((++chords&1023)==0)token.ThrowIfCancellationRequested();
            var a=path.Points[i-1];var b=path.Points[i];
            double run=Math.Sqrt(Math.Pow(a.X-b.X,2)+Math.Pow(a.Y-b.Y,2));
            double excess=Math.Abs(a.Z-b.Z)-grade*run;maximumExcess=Math.Max(maximumExcess,excess);
            if(excess>1e-8)throw new InvalidOperationException($"Smooth ring routing exceeded the requested ramp angle at region {path.RegionIndex}, chord {i-1}: {excess:R}.");
        }
        log?.Invoke($"stage=smooth-routing event=certified paths={result.Count} chords={chords} maximumExcess={maximumExcess:R} "
            +"continuityPolicy=constructor-row-sequence cuttingConnectors=original-native-F3-proof requiredFirstCutsAndRampsPreserved=True completedCutRetracesRequireActualEarlierNativeAndF3Witness=True");
        if(order.SpatialSweepEnabled){
            double nonCutDistance=0,independentGapDistance=0;long nonCutChords=0;Point3D? previous=start;
            foreach(var path in result){
                if(path.Points.Count<2)continue;
                if(previous is {} end&&!MillingTransitionPlanner.IsContinuous(end,path.Points[0]))
                    independentGapDistance+=ToolpathContinuationPlanner.Distance(end,path.Points[0]);
                var kinds=ToolpathSpanKinds.Expand(path);
                for(int i=1;i<path.Points.Count;i++)if(kinds[i-1]!=ToolpathSpanKind.Cut){
                    nonCutDistance+=ToolpathContinuationPlanner.Distance(path.Points[i-1],path.Points[i]);nonCutChords++;
                }
                previous=path.Points[^1];
            }
            log?.Invoke($"stage=ring-spatial-sweep event=complete {order.SpatialSummary()} "
                +$"routedNonCutDistanceMm={nonCutDistance:R} routedNonCutChords={nonCutChords} independentEndpointGapDistanceMm={independentGapDistance:R} "
                +"distanceScope=actual-return-optimized-source-trace finalEmitterRetractDistancesNotIncluded=True chainTourSuppressed=True");
        }
        cost?.Report(log,"smooth-ring-router");
        enteredVisits?.Report(log);
        return result;
    }
}
