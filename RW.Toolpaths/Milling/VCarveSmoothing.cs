using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling.Acceleration;
using System.Diagnostics;
using System.Globalization;

namespace RW.Toolpaths.Milling;

/// <summary>
/// A shallower, grade-constrained V-carve floor. The retained crest owns the floor;
/// physical cutter support constrains every cutting visit, including clearing.
/// Lost surface coverage is measured and restored through the common ring collector.
/// </summary>
internal static class VCarveSmoothing
{
    internal const double CoverageTolerance = MachiningAccuracy.SmoothCoverage;
    // Channel completion uses the current operation configuration.
    // Source construction, floor search and physical proofs retain independent
    // numerical/physical precision; none grants an extra finish allowance.
    // Completion is configured per operation inside ApplyCore; no hidden global finish allowance.
    private const double SweepTolerance = .00075;
    private const double FieldTolerance = MachiningAccuracy.FloorSearch;
    private const double DepthReserve = .002;
    private const int MaximumPoints = 1_000_000;
    private const int MaximumCorrectionRounds = 8;
    private const int MinimumTerminalPathsForWitnessPruning = 1024;
    private const int MaximumTerminalDominanceChords = 65_536;
    internal const double ReferenceCorrectionArcTolerance = 1 / PathUtils.Scale;

    // A per-execution diagnostic observer may synchronously snapshot the exact
    // immutable authority inputs. Production leaves it null and performs no
    // allocation, I/O or serialization here. Observers must not mutate inputs.
    internal sealed record CertificateInputs(string? OperationId, int RegionIndex, int Round,
        IReadOnlyList<IReadOnlyList<PointD>> Target, IReadOnlyList<TaggedToolpath> CommandPaths,
        VCarveCutterProfile Profile, double OutputSurfaceZ, double GapRadius,
        VCarveCoverageCertificate.Limits Limits)
    {
        internal double GoalAnalysisSurfaceZ { get; init; }
    }
    internal static readonly AsyncLocal<Action<CertificateInputs>?> CertificateObserver = new();

    // Read-only, per-execution constructor capture. The observer owns any
    // synchronous snapshot/serialization. Complete batch snapshots are emitted
    // serially before any component construction, so a diagnostic may stop here.
    // Null leaves nomination inputs, decisions and allocations unchanged.
    internal sealed record CorrectionConstructorInputs(string? OperationId, int RegionIndex,
        int Round, int Component, IReadOnlyList<IReadOnlyList<PointD>> OriginalSourceBoundary,
        IReadOnlyList<IReadOnlyList<PointD>> ErrorBoundary, double FloorCap, double InitialCutDepth,
        VCarveCutterProfile Profile, DepthSchedule Schedule, double StepOver, double ArcTolerance,
        bool OrdinaryCollector, bool DetectCurves, long PointBudget, int BatchComponentCount);
    internal static readonly AsyncLocal<Action<CorrectionConstructorInputs>?> CorrectionConstructorObserver = new();

    // Internal per-execution controls may nominate geometry or narrow a real
    // certificate budget. They cannot supply a successful proof or mutate any
    // stock authority. Production leaves this null. Capture once at Apply entry
    // so a complete original retry keeps the same observation ownership.
    internal sealed record GpuFrontCoreInputs(bool UseGpuFronts,
        IReadOnlyList<TaggedToolpath> Source, IReadOnlyList<IReadOnlyList<PointD>> Boundary,
        ToolGeometry Tool, DepthSchedule Schedule, VCarveOptions Options,
        VCarvePreparation.PreparedRegion? Preparation, int? CorrectionWorkers);
    internal sealed record GpuFrontCompletionInputs(bool SignificantEmpty, int TerminalPathCount,
        int NominatedComponents, bool OriginalCertificateRequested, CertificateInputs Authority,
        VCarveCoverageCertificate.Result Certificate);
    internal sealed record GpuFrontIntegrationControl(
        Func<IReadOnlyList<IReadOnlyList<PointD>>, CancellationToken, Action<string>?,
            VCarveGpuCorrectionFrontNominator?>? NominationOwnerFactory = null,
        bool ForceCompletionCertificateInconclusive = false,
        Action<GpuFrontCoreInputs>? CoreEntered = null,
        Action<IReadOnlyList<IReadOnlyList<PointD>>>? FixedTargetPrepared = null,
        Action<GpuFrontCompletionInputs>? CompletionObserved = null,
        int? MinimumBoxSourcePointsForControls = null);
    internal static readonly AsyncLocal<GpuFrontIntegrationControl?> GpuFrontIntegrationControls = new();

    internal static void ValidateCorrectionArcTolerance(double tolerance)
    {
        if (!double.IsFinite(tolerance) || tolerance < ReferenceCorrectionArcTolerance || tolerance > .001)
            throw new ArgumentOutOfRangeException(nameof(VCarveOptions.CorrectionArcTolerance),
                "Smooth correction arc tolerance must be finite, at least one geometry quantum and no more than 0.001 mm.");
    }

    internal static List<TaggedToolpath> Apply(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<IReadOnlyList<PointD>> boundary, ToolGeometry tool, DepthSchedule schedule,
        VCarveOptions options, CancellationToken cancellationToken, Action<string>? diagnostic,
        Action<double, string, bool>? progress = null,
        VCarvePreparation.PreparedRegion? preparation = null, int? correctionWorkers = null)
        => ApplyWithContinuation(source,boundary,tool,schedule,options,cancellationToken,diagnostic,
            progress,preparation,correctionWorkers);

    internal static List<TaggedToolpath> ApplyWithContinuation(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<IReadOnlyList<PointD>> boundary, ToolGeometry tool, DepthSchedule schedule,
        VCarveOptions options, CancellationToken cancellationToken, Action<string>? diagnostic,
        Action<double, string, bool>? progress = null,
        VCarvePreparation.PreparedRegion? preparation = null, int? correctionWorkers = null,
        Action<Func<Point3D, Point3D, bool>>? captureContinuationGuard = null,
        Action<Func<IReadOnlyList<TaggedToolpath>,List<TaggedToolpath>>>? captureClippedPathProtection=null, int sourceRegionIndex=0)
    {
        cancellationToken.ThrowIfCancellationRequested();
        ValidateCorrectionArcTolerance(options.CorrectionArcTolerance);
        options = options.WithPriorStock(cancellationToken);
        var policy = options.InitialStrategy?.GenerationPolicy ?? VCarveGenerationPolicy.Capture();
        diagnostic?.Invoke($"stage=generation-policy signature='{policy.CacheSignature(options.InitialStrategy)}' "
            + $"capturedWithRequest={options.InitialStrategy is not null} source=immutable-request deviceAvailabilityIsStockAuthority=False");
        long started = Stopwatch.GetTimestamp();
        double requested = options.CorrectionArcTolerance;
        diagnostic?.Invoke($"stage=correction-arc event=start requestedArc={requested:R} "
            + $"referenceArc={ReferenceCorrectionArcTolerance:R} firstLevelArc={ReferenceCorrectionArcTolerance:R} "
            + $"ordinaryCorrectionCandidates={options.UseOrdinaryCorrectionCandidates} ordinaryRounds=0..6 originalRecoveryRound=7 "
            + "maximumCurveError=0.001 stockCredit=0 originalChannelAndPhysicalGuards=unchanged");
        bool fallback = false;
        bool useGpuFronts = policy.GpuFronts;
        bool gpuCompletionFallback = false;
        var gpuIntegrationControl = GpuFrontIntegrationControls.Value;
        List<TaggedToolpath> result;
        try
        {
            try
            {
                result = ApplyCore(source, boundary, tool, schedule, options, cancellationToken,
                    diagnostic, progress, preparation, correctionWorkers, useGpuFronts, gpuIntegrationControl, policy, captureContinuationGuard, captureClippedPathProtection, sourceRegionIndex);
            }
            catch (InvalidOperationException error) when (useGpuFronts
                && error.Data.Contains("SmoothVCarvingGeometry")
                && error.Data["GpuVCarveFrontsUsed"] is true)
            {
                cancellationToken.ThrowIfCancellationRequested(); gpuCompletionFallback = true;
                diagnostic?.Invoke("stage=gpu-correction-front event=completion-fallback "
                    + "reason=whole-original-target-or-completion-budget-unverified "
                    + "retry=complete-original-ApplyCore originalInputsPreserved=True useGpuFronts=False "
                    + "environmentDoesNotAuthorizeRetryNominations=True safetyExceptions=propagated");
                // Complete original retry. The environment remains opt-in, but
                // explicit false prevents recursion/re-entry into nominations.
                result = ApplyCore(source, boundary, tool, schedule, options, cancellationToken,
                    diagnostic, progress, preparation, correctionWorkers, useGpuFronts: false, gpuIntegrationControl: gpuIntegrationControl,
                    policy: policy, captureContinuationGuard: captureContinuationGuard, captureClippedPathProtection:captureClippedPathProtection, sourceRegionIndex:sourceRegionIndex);
            }
        }
        catch (InvalidOperationException error) when ((requested > ReferenceCorrectionArcTolerance
            || options.UseOrdinaryCorrectionCandidates)
            && error.Data.Contains("SmoothVCarvingGeometry"))
        {
            cancellationToken.ThrowIfCancellationRequested();
            fallback = true;
            diagnostic?.Invoke($"stage=correction-arc event=fallback requestedArc={requested:R} "
                + $"usedArc={ReferenceCorrectionArcTolerance:R} "
                + $"requestedOrdinaryCandidates={options.UseOrdinaryCorrectionCandidates} usedOrdinaryCandidates=False "
                + "reason=completion-coverage-or-after-correction-point-budget "
                + "source=identical-original-inputs target=original-commanded-channel-upper safetyExceptions=propagated");
            result = ApplyCore(source, boundary, tool, schedule,
                options with { CorrectionArcTolerance = ReferenceCorrectionArcTolerance,
                    UseOrdinaryCorrectionCandidates = false }, cancellationToken,
                diagnostic, progress, preparation, correctionWorkers, useGpuFronts: false, gpuIntegrationControl: gpuIntegrationControl,
                policy: policy, captureContinuationGuard: captureContinuationGuard, captureClippedPathProtection:captureClippedPathProtection, sourceRegionIndex:sourceRegionIndex);
        }
        cancellationToken.ThrowIfCancellationRequested();
        diagnostic?.Invoke($"stage=correction-arc event=complete requestedArc={requested:R} "
            + $"usedArc={(fallback ? ReferenceCorrectionArcTolerance : requested):R} fallback={fallback} "
            + $"requestedOrdinaryCandidates={options.UseOrdinaryCorrectionCandidates} usedOrdinaryCandidates={options.UseOrdinaryCorrectionCandidates && !fallback} "
            + $"paths={result.Count} points={result.Sum(path => (long)path.Points.Count)} "
            + $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} originalChannelAndPhysicalGuards=unchanged");
        if (useGpuFronts)
            diagnostic?.Invoke($"stage=gpu-correction-front event=apply-complete originalRetry={gpuCompletionFallback} "
                + "physicalGuardsUnchanged=True wholeOriginalTargetRequiredWhenNominationsUsed=True");
        cancellationToken.ThrowIfCancellationRequested();
        return result;
    }

    // Fixed-owner guard for bounded source-construction experiments. Production
    // below deliberately closes over its later original-channel wall owner.
    internal static Func<Point3D,Point3D,bool> CreateContinuationGuard(FloorField? field,
        CommandClearance wallGuard,double surfaceZ,double grade,CancellationToken token)
    {
        var continuationFloor=field is null?null:new ActualF3CompactionFloorProof(field,grade,token);
        return(a,b)=>
        {
            var ca=CommandPoint(a,surfaceZ);var cb=CommandPoint(b,surfaceZ);
            if(Math.Abs(a.Z-b.Z)>grade*Distance(a,b)||Math.Abs(ca.Z-cb.Z)>grade*Distance(ca,cb))return false;
            if(continuationFloor is not null&&(!continuationFloor.Prove(a,b)||!continuationFloor.Prove(ca,cb)))return false;
            return wallGuard.IsLegalCommandedChord(a,b);
        };
    }

    private static List<TaggedToolpath> ApplyCore(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<IReadOnlyList<PointD>> boundary, ToolGeometry tool, DepthSchedule schedule,
        VCarveOptions options, CancellationToken cancellationToken, Action<string>? diagnostic,
        Action<double, string, bool>? progress,
        VCarvePreparation.PreparedRegion? preparation, int? correctionWorkers, bool useGpuFronts, GpuFrontIntegrationControl? gpuIntegrationControl, VCarveGenerationPolicy policy, Action<Func<Point3D, Point3D, bool>>? captureContinuationGuard, Action<Func<IReadOnlyList<TaggedToolpath>,List<TaggedToolpath>>>? captureClippedPathProtection, int sourceRegionIndex)
    {
        if(!double.IsFinite(options.PathToleranceMm)||options.PathToleranceMm<=1e-8)throw new ArgumentOutOfRangeException(nameof(options.PathToleranceMm));
        double CompletionTolerance=2*options.CompletionGapRadiusMm;
        cancellationToken.ThrowIfCancellationRequested();
        bool useTaperedLowerSweeps = policy.TaperedSweeps;
        bool useLowerSweepBudget = policy.LowerSweepBudget;
        bool useActualF3Compaction = policy.ActualF3Compaction;
        bool useBoxNeighborhoods = policy.BoxNeighborhoods;
        bool useConditionedCorrectionSource = policy.ConditionedCorrectionSource;
        bool useParallelSweepUnions = options.InitialStrategy?.ParallelSweepUnions ?? policy.ParallelSweepUnions;
        bool useArtworkFringeGoal = policy.ArtworkFringeGoal;
        bool useMaterialLayerGoal = policy.MaterialLayerGoal;
        bool useConditionedReferenceOpening = policy.ConditionedReferenceOpening;
        bool useInitialStockCohort = options.InitialStrategy?.InitialStockCohort ?? policy.InitialStockCohort;
        if (gpuIntegrationControl?.CoreEntered is { } coreEntered)
            coreEntered(new(useGpuFronts, source, boundary, tool, schedule, options, preparation, correctionWorkers));
        cancellationToken.ThrowIfCancellationRequested();
        diagnostic?.Invoke($"stage=box-correction-neighborhood event=selection enabled={useBoxNeighborhoods} "
            + "selection=RW_TOOLPATHS_VCARVE_BOX_NEIGHBORHOODS selectionFrozenForApply=True "
            + "candidateCentersOnly=True stockCredit=0 enclosureCredit=0 physicalCredit=0 "
            + "originalFloorBothWallsGradeAndWholeTargetRequired=True unsupportedUsesLiteralOffset=True");
        double grade = Math.Tan(options.SmoothRampAngleDegrees * Math.PI / 180);
        MillingGeometryTrace.Capture("smoothing-input",0,source);
        var floorSources = VCarveRampSurface.Sources(boundary, options, cancellationToken);
        int originalPathCount = source.Count;
        var profile = VCarvePhysicalSweep.ProfileFor(tool, schedule.Depth);
        VCarveMaterialSurface? stockContact = options.ClearedStock is { } remainingStock
            ? new VCarveMaterialSurface(boundary, remainingStock, profile, schedule.Depth, schedule.SurfaceZ, cancellationToken, diagnostic) : null;
        if(stockContact is not null) floorSources=VCarveStockContactPaths.Project(floorSources,stockContact,options.PathToleranceMm,cancellationToken);
        if (floorSources.Count > 0)
        {
            var ramps = VCarveRampSurface.Contours(boundary, options, profile, schedule.Depth,
                source.Count > 0 ? source[0].RegionIndex : preparation?.RegionIndex ?? sourceRegionIndex,
                cancellationToken, diagnostic);
            source = source.Concat(ramps).ToList();
        }
        if(stockContact is not null) source=VCarveStockContactPaths.Project(source,stockContact,options.PathToleranceMm,cancellationToken);
        long sourcePointCount = source.Sum(path => (long)path.Points.Count);
        long pointBudget = Math.Max(MaximumPoints, checked(8 * sourcePointCount));
        var medial = preparation?.OriginalMedialPaths.ToList()
            ?? source.Where(path => path.Category == "final-carve" && path.Points.Count >= 2).ToList();
        if (medial.Count == 0 && floorSources.Count == 0 && !useInitialStockCohort)
        {
            source = VCarveInitialReachSelection.NormalizeMarkers(source, cancellationToken);
            bool normalizeMarker = source.Any(path => path.Category == AdaptiveInitialReachContourNominator.MarkerCategory);
            diagnostic?.Invoke($"stage=smoothing event=complete reason=no-medial-profile surfaceEntriesRequired=True nominationMarkerNormalized={normalizeMarker}");
            return normalizeMarker ? AdaptiveInitialReachContourNominator.NormalizeMarkers(source, cancellationToken) : source.ToList();
        }
        if (medial.Count == 0)
            diagnostic?.Invoke($"stage=smoothing event=no-medial-initial-physical-guards originalArtworkWallAndNominalGradeRequired=True originalFloorAvailable={floorSources.Count > 0} noConstructionOrStockCredit=True");
        // New outline ramp rows must not masquerade as an original clearing
        // band and suppress the retained medial channel's coverage recovery.
        if (useInitialStockCohort && source.Take(originalPathCount).All(path => path.Category == "final-carve"))
        {
            useInitialStockCohort = false;
            diagnostic?.Invoke("stage=initial-stock-cohort event=coverage-repair reason=no-clearing-band "
                + "originalReferenceTargetRequired=True originalFloorArtworkAndGradeRetained=True");
        }
        diagnostic?.Invoke($"stage=smoothing event=start angleDegrees={options.SmoothRampAngleDegrees:R} " +
            $"maximumGrade={grade:R} sourcePaths={source.Count} medialPaths={medial.Count} " +
            $"coverageTolerance={CoverageTolerance:R} floorDepthReserve={DepthReserve:R} " +
            $"sourcePoints={sourcePointCount} pointBudget={pointBudget} budgetRule=max-one-million-or-eight-times-source " +
            "commandF3Rounding=exact-binary32-nearest-even finiteFastPathMagnitude=1000000 literalOutsideDomain=True");
        progress?.Invoke(0, "Constraining the medial depth gradient", true);
        diagnostic?.Invoke($"stage=smoothing event=analysis-reuse reused={preparation is not null} " +
            "floorOwner=continuous-retained-crest-before-routing-and-command-reserves");
        double goalSurfaceZ = VCarveMaterialLayerGoal.AnalysisSurface(useMaterialLayerGoal);
        double minimumCorrectionDepth = .001 - goalSurfaceZ;
        diagnostic?.Invoke($"stage=material-layer-goal event=selection enabled={useMaterialLayerGoal} goalSurfaceZ={goalSurfaceZ:R} "
            +$"layerDepthMm={VCarveMaterialLayerGoal.LayerDepthMm:R} selectionFrozenForApply=True "
            +"originalPhysicalCommandsAndReferenceChannelSurfaceZ=0 originalFloorArtworkWallsAndGradeRetained=True "
            +"referenceLowerAndQUseSameAnalysisPlane=True returnedPathsNotTranslated=True QViewNotReposted=True "
            +"omittedMaterialAtMostLayerDepthOnlyOutsideProjectedRawReferenceUpper=True openingAndArtworkFringeSeparatePolicies=True globalStockAccepted=False");
        // A contained lower representation can spend part of the unchanged
        // coverage budget on its mesh. This changes no commanded pose, upper
        // target or physical guard; final residual and certificate owners remain.
        double lowerSweepTolerance = useLowerSweepBudget ? CoverageTolerance / 2 : SweepTolerance;
        bool consolidateLowerSweeps = policy.GpuDiscWitnesses;
        diagnostic?.Invoke($"stage=smooth-lower-sweep event=selection hemisphereConsolidation={consolidateLowerSweeps} "
            + "selection=RW_TOOLPATHS_VCARVE_GPU experimental=True lowerOnly=True originalUpperUnchanged=True "
            + $"taperedLowerSweeps={useTaperedLowerSweeps} taperedSelection=RW_TOOLPATHS_VCARVE_TAPERED_SWEEPS "
            + $"lowerSweepBudget={useLowerSweepBudget} lowerSweepTolerance={lowerSweepTolerance:R} "
            + $"parallelLeafUnions={useParallelSweepUnions} parallelSelection=RW_TOOLPATHS_VCARVE_PARALLEL_SWEEP_UNIONS "
            + "parallelSelectionFrozenForApply=True physicalPrimitivesAndOrderedHierarchyUnchanged=True "
            + "lowerBudgetSelection=RW_TOOLPATHS_VCARVE_LOWER_SWEEP_BUDGET selectionFrozenForApply=True "
            + "guardRefusal=literal-original-pieces upperTargetAndCoverageTolerance=unchanged");
        List<List<PointD>> BuildLowerSweep(IReadOnlyList<TaggedToolpath> paths)
            => useParallelSweepUnions
                ? VCarvePhysicalSweep.BuildWithParallelLeafUnions(CommandPaths(paths, schedule.SurfaceZ), profile, goalSurfaceZ,
                    lowerSweepTolerance, cancellationToken, consolidateHemisphereChords: consolidateLowerSweeps,
                    enableTaperedRadiusRuns: useTaperedLowerSweeps)
                : useTaperedLowerSweeps
                ? VCarvePhysicalSweep.BuildWithTaperedRadiusRuns(CommandPaths(paths, schedule.SurfaceZ), profile, goalSurfaceZ,
                    lowerSweepTolerance, cancellationToken, consolidateHemisphereChords: consolidateLowerSweeps,
                    enableTaperedRadiusRuns: true)
                : VCarvePhysicalSweep.Build(CommandPaths(paths, schedule.SurfaceZ), profile, goalSurfaceZ,
                    lowerSweepTolerance, cancellationToken, consolidateHemisphereChords: consolidateLowerSweeps);
        // Anchor every point of each retained top edge. Selecting raised medial
        // spans instead makes the surface depend on branch spacing and produces
        // separate cones along an otherwise straight crest.
        FloorField? field = floorSources.Count == 0 ? null
            : new FloorField(floorSources, profile, grade, schedule.Depth, cancellationToken);
        if(field is not null)diagnostic?.Invoke($"stage=smooth-floor event=prepared sourcePaths={floorSources.Count} "
            +$"surfaceOwner=continuous-retained-crest grade={grade:R} plateauDepth={schedule.Depth:R} medialConeAnchors=False "
            +field.SupportDominanceDiagnostics+" "+field.QueryScratchDiagnostics);
        var wallGuard = new CommandClearance(boundary, profile, schedule.SurfaceZ, cancellationToken, diagnostic,
            maximumPoints: pointBudget, stockContact: stockContact);
        if (captureContinuationGuard is not null)
        {
            // Reuse the ORIGINAL floor and whole posted artwork chord authorities.
            // This adapter authorizes new shallow cutting links; executed retraces
            // need no new floor allowance or source-shell ancestry.
            var continuationFloor = field is null ? null : new ActualF3CompactionFloorProof(field, grade, cancellationToken);
            captureContinuationGuard((a, b) => {
                var ca = CommandPoint(a, schedule.SurfaceZ); var cb = CommandPoint(b, schedule.SurfaceZ);
                if (Math.Abs(a.Z-b.Z) > grade * Distance(a,b)
                    || Math.Abs(ca.Z-cb.Z) > grade * Distance(ca,cb)) return false;
                if (continuationFloor is not null && (!continuationFloor.Prove(a,b) || !continuationFloor.Prove(ca,cb))) return false;
                return wallGuard.IsLegalCommandedChord(a,b);
            });
        }

        if (source.Any(path => path.Category == VCarveInitialReachSelection.HalfCoreMarkerCategory
            || path.Category == VCarveInitialReachSelection.QuarterOuterMarkerCategory))
        {
            if (useInitialStockCohort && field is not null)
            {
                double outerNominalDepth = schedule.Depth / 4;
                source = VCarveFloorAwareOuterReach.Apply(source, profile, outerNominalDepth,
                    (x, y) => field.CappedDepth(outerNominalDepth, x, y),
                    (x, y) => wallGuard.NominationArtworkDistance(x, y),
                    (x, y, depth) => profile.PhysicalRadiusAtDepth(Math.Max(0,
                        -CommandPoint(new Point3D(x, y, -depth), schedule.SurfaceZ).Z)),
                    cancellationToken, diagnostic);
                // Different-XY tip depths do not prove 3D stock containment.
                // Retain the complete shallow band through the original guards.
                diagnostic?.Invoke("stage=initial-half-reach event=retain reason=no-whole-volume-dominance "
                    + "allShallowBandNominationsRetained=True originalFloorArtworkAndGradeRequired=True");
            }
            // A shallow profile chooses lateral sites, not the finishing depth.
            // Request the full floor there; the same Protect below owns the
            // continuous physical floor, walls and grade. Surface-origin
            // scheduling subsequently bounds each visit's fresh engagement.
            long requested = 0;
            source = source.Select(path => {
                if (!VCarveInitialReachSelection.IsSupplemental(path)) return path;
                requested += path.Points.Count;
                return path with { Points = path.Points.Select(point => point with { Z = -schedule.Depth }).ToArray() };
            }).ToList();
            diagnostic?.Invoke($"stage=supplemental-finishing-depth event=request points={requested} depth={schedule.Depth:R} "
                + "lateralNominationRetained=True originalFloorArtworkGradeAndSurfaceOriginStepdownRequired=True");
            source = VCarveInitialReachSelection.NormalizeMarkers(source, cancellationToken);
        }
        if (source.Any(path => path.Category == AdaptiveInitialReachContourNominator.MarkerCategory))
            source = AdaptiveInitialReachContourNominator.NormalizeMarkers(source, cancellationToken);
        VCarveActualF3TrajectoryCompaction? actualF3Compaction = null;
        int protectionCalls = 0;
        // Keep proof custody beside construction, instead of rediscovering the
        // same million chords in a bounded global LRU during the final audit.
        // The snapshot is owned by this immutable wall domain and is rechecked
        // against every actual command before reuse. Unknowns use literal proof.
        var protectedWalls=new Dictionary<IReadOnlyList<Point3D>,(CommandClearance Owner,Point3D[] Commands)>(ReferenceEqualityComparer.Instance);
        long certificatePoints=0,wallCertificateHits=0,wallCertificateMisses=0;
        List<TaggedToolpath> Protect(IReadOnlyList<TaggedToolpath> paths)
        {
            int call = ++protectionCalls;
            MillingGeometryTrace.Capture("protect-input",call,paths);
            string phase = "compaction";
            long started = diagnostic is null ? 0 : Stopwatch.GetTimestamp();
            long inputPoints = diagnostic is null ? 0 : paths.Sum(path => (long)path.Points.Count);
            double compactionMs = 0, floorMs = 0, wallMs = 0, gradeMs = 0;
            try
            {
                // Initial source protection has no compaction owner. Correction
                // candidates may nominate only after ORIGINAL both walls/target
                // are fixed; the original Protect always follows every proposal.
                if (actualF3Compaction is not null) paths = actualF3Compaction.Compact(paths, "before-correction-protection");
                if (started != 0) { compactionMs = Stopwatch.GetElapsedTime(started).TotalMilliseconds; started = Stopwatch.GetTimestamp(); }
                phase = "floor";
                diagnostic?.Invoke($"stage=smooth-protection-phases event=start call={call} phase=floor inputPaths={paths.Count} inputPoints={inputPoints}");
                var floor = field is null ? paths.ToList() : Constrain(paths, field, grade, cancellationToken, pointBudget,
                    options.ParallelFloorQueryPreparation,diagnostic);
                MillingGeometryTrace.Capture("protect-floor",call,floor);
                if (started != 0) { floorMs = Stopwatch.GetElapsedTime(started).TotalMilliseconds; started = Stopwatch.GetTimestamp(); }
                phase = "wall";
                var guarded = wallGuard.Apply(floor);
                MillingGeometryTrace.Capture("protect-wall",call,guarded);
                if (started != 0) { wallMs = Stopwatch.GetElapsedTime(started).TotalMilliseconds; started = Stopwatch.GetTimestamp(); }
                phase = "grade";
                var limited = guarded.SelectMany(path => LimitGraph(new[] { path }, grade, cancellationToken)).ToList();
                MillingGeometryTrace.Capture("protect-grade",call,limited);
                if (started != 0) gradeMs = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                if (floor.Count != paths.Count || guarded.Count != paths.Count || limited.Count != paths.Count)
                    throw new InvalidOperationException("Smooth V-carving protection changed path ordinals; terminal provenance is unverified.");
                if(useInitialStockCohort)
                for(int row=0;row<limited.Count;row++)
                {
                    var original=guarded[row].Points;var current=limited[row].Points;
                    if(current.Count<2||current.Count!=original.Count||certificatePoints+current.Count>2_000_000)continue;
                    var commands=new Point3D[current.Count];bool contained=true;
                    for(int i=0;i<current.Count;i++)
                    {
                        if((i&1023)==0)cancellationToken.ThrowIfCancellationRequested();
                        var before=CommandPoint(original[i],schedule.SurfaceZ);
                        var after=commands[i]=CommandPoint(current[i],schedule.SurfaceZ);
                        if(!double.IsFinite(after.X+after.Y+after.Z)||before.X!=after.X||before.Y!=after.Y||before.Z>after.Z)
                        {contained=false;break;}
                    }
                    if(contained){protectedWalls[current]=(wallGuard,commands);certificatePoints+=current.Count;}
                }
                diagnostic?.Invoke($"stage=smooth-protection-phases event=complete call={call} inputPaths={paths.Count} inputPoints={inputPoints} "
                    + $"compactedPoints={paths.Sum(path => (long)path.Points.Count)} floorPoints={floor.Sum(path => (long)path.Points.Count)} "
                    + $"wallPoints={guarded.Sum(path => (long)path.Points.Count)} gradePoints={limited.Sum(path => (long)path.Points.Count)} "
                    + $"compactionMs={compactionMs:R} floorMs={floorMs:R} wallMs={wallMs:R} gradeMs={gradeMs:R} "
                    + "scope=sequential-original-Protect phasesExcludeSummaryLog=True physicalGuardsUnchanged=True");
                return limited;
            }
            catch (Exception error)
            {
                // Preserve the original physical failure even if its diagnostic
                // consumer itself fails. No guard exception is a fallback.
                try { diagnostic?.Invoke($"stage=smooth-protection-phases event=failed call={call} phase={phase} exception={error.GetType().Name} "
                    + $"completedCompactionMs={compactionMs:R} completedFloorMs={floorMs:R} completedWallMs={wallMs:R} completedGradeMs={gradeMs:R}"); }
                catch { }
                throw;
            }
        }
        captureClippedPathProtection?.Invoke(Protect);
        wallGuard.EnableArtworkProofCapture();
        if (field is not null && !profile.RoundedTip && source.Count > originalPathCount)
        {
            // Sharp/flat profiles use the general Lipschitz chord proof. Give
            // new ramp nominations its existing 2um axial preparation reserve;
            // the finer radius-derived reserve otherwise forces thousands of
            // subdivisions on an ordinary straight constant-depth contour.
            source = source.Select((path, index) => index < originalPathCount ? path : path with {
                Points = path.Points.Select(p => p with {
                    Z = -Math.Max(0, field.Depth(p.X, p.Y) - DepthReserve)
                }).ToArray()
            }).ToList();
            diagnostic?.Invoke($"stage=outline-ramps event=prepare nonroundedAxialReserve={DepthReserve:R} "
                + "originalMedialAndFillUnchanged=True completePhysicalChordProofRequired=True");
        }
        var constrained = Protect(source);
        // Full-outline ramps are new surface work, rather than a demand to
        // restore full-depth ordinary fill. Their physically fitted poses own
        // their reference footprint; medial and authored fill retain theirs.
        if (source.Count > originalPathCount)
        {
            source = source.Take(originalPathCount).Concat(constrained.Skip(originalPathCount)).ToList();
            diagnostic?.Invoke($"stage=outline-ramps event=protected paths={source.Count - originalPathCount} "
                + $"points={constrained.Skip(originalPathCount).Sum(p => (long)p.Points.Count)} "
                + "reference=physical-ramp-surface originalMedialAndFillPreserved=True");
        }
        diagnostic?.Invoke($"stage=source-paths event=protected sourcePaths={originalPathCount} "
            + $"rampPaths={source.Count-originalPathCount} outputPaths={constrained.Count} "
            + "families=original-medial-and-concentric supplementalRestFragments=False");
        if (!options.IncludeInteriorFill)
        {
            // The slope is complete; Fill Limit adds a nearest-edge floor band.
            // Keep that selected physical channel as the link domain, so a
            // reconnect cannot bridge the untouched broad core.
            diagnostic?.Invoke($"stage=fill-limit event=complete authoredWidth={options.EdgeFillWidth:R} "
                + $"nearestEdgeLimit={options.EdgeFillWidth:R} roughingReachRequested={options.RoughingRestWidth:R} "
                + $"paths={constrained.Count} slopeAdditive=True selection=post-slope-nearest-edge-band originalMedialRetained=True");
            if (constrained.Count == 0) return constrained;
            var channel = Timed("bounded-fill-command-channel", () => VCarvePhysicalSweep.Build(
                CommandPaths(constrained, schedule.SurfaceZ), profile, 0, SweepTolerance,
                cancellationToken, upperBound: true));
            var protectedArtwork = wallGuard;
            // New links stay within the actual selected cutting channel, so a
            // short reconnect cannot turn a limited fill into an interior cut.
            wallGuard = new CommandClearance(boundary, profile, schedule.SurfaceZ,
                cancellationToken, diagnostic, channel, pointBudget, protectedArtwork);
            return CompleteInitialStockCohort(channel, null, fillLimited: true);
        }
        var changed = new List<TaggedToolpath>();
        double largestReduction = 0;
        long measuredReductionSites=0,unevaluatedReductionSites=0;
        for (int i = 0; i < source.Count; i++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            bool change = !source[i].Points.SequenceEqual(constrained[i].Points);
            foreach (Point3D point in source[i].Points)
            {
                // Once a path is known to have changed, another floor query
                // cannot change that classification; it only measured a log.
                // Keep the original literal query for EVERY still-unchanged
                // path, including single-point and unprojected rest paths.
                if(!change)
                {
                    double reduction=field is null?0:-point.Z-field.CappedDepth(-point.Z,point.X,point.Y);
                    change|=reduction>1e-8;largestReduction=Math.Max(largestReduction,reduction);measuredReductionSites++;
                }
                else if(field is not null&&field.TryEvaluatedDepth(point.X,point.Y,out double evaluated))
                {largestReduction=Math.Max(largestReduction,Math.Max(0,-point.Z-evaluated));measuredReductionSites++;}
                else unevaluatedReductionSites++;
            }
            if (change) changed.Add(source[i]);
        }
        diagnostic?.Invoke($"stage=smoothing event=constrained sourceGrade={MaximumGrade(source):R} " +
            $"outputGrade={MaximumGrade(constrained):R} maximumMeasuredDepthReduction={largestReduction:R} " +
            $"measuredReductionSites={measuredReductionSites} unevaluatedReductionSites={unevaluatedReductionSites} reductionMeasurement=already-evaluated-exact-XY " +
            $"changedPaths={changed.Count} outputPoints={constrained.Sum(path => (long)path.Points.Count)} " +
            "graphOwner=original-medial continuousSourceSegments=True fullCutterFloorProtection=True");
        if (changed.Count == 0)
        {
            diagnostic?.Invoke("stage=smoothing event=complete reason=source-already-within-grade geometryBeforeContributionFilterPreserved=True commandedWallProof=True surfaceEntriesRequired=True");
            if (options.RoughingRestWidth <= 0) return source.ToList();
            return source.ToList();
        }        if (useInitialStockCohort)

        {

            // Source is the adopted initial nomination at this point. No source

            // snapshot is taken before the original floor/artwork owners exist.

            var sourceShell = VCarveInitialSourceChordShell.Check(source, constrained,

                point => CommandPoint(point, schedule.SurfaceZ), cancellationToken, pointBudget);

            diagnostic?.Invoke($"stage=initial-source-chord-shell event=admission accepted={sourceShell.Accepted} reason='{sourceShell.Reason}' "

                + $"sourceChords={sourceShell.SourceChords} candidatePoints={sourceShell.CandidatePoints} zeroXYChords={sourceShell.ZeroXYChords} "

                + $"lastPath={sourceShell.LastPathOrdinal} lastSourceChord={sourceShell.LastSourceChordOrdinal} lastCandidatePoint={sourceShell.LastCandidatePointOrdinal} "

                + $"maximumNominalXYResidual={sourceShell.MaximumNominalXYResidual:R} maximumNominalDepthRoundoff={sourceShell.MaximumNominalDepthRoundoff:R} "

                + $"maximumActualXYDisplacement={sourceShell.MaximumActualXYDisplacement:R} maximumActualPositiveDepthDifference={sourceShell.MaximumActualPositiveDepthDifference:R} "

                + $"seconds={sourceShell.Seconds:R} originalPointBudget={pointBudget} "

                + "firstOriginalProtectCompleted=True noLatticeSnap=True actualShellMeasured=True unionCoverageCredit=False stockAcceptance=False "

                + "unsupportedFallback=literal-reference-sweep-and-original-channel physicalFloorArtworkAndGradeUnchanged=True");

            cancellationToken.ThrowIfCancellationRequested();

            if(sourceShell.Accepted)
                return CompleteInitialStockCohort(Array.Empty<IReadOnlyList<PointD>>(),sourceShell);
        }

        progress?.Invoke(.15, "Measuring the missing channel coverage", true);;

        // Audit the same float world-Z subtraction and F3 command as the post.
        // Paths stay stock-relative here; only this sweep view is quantized.
        // Preserve the already physically protected original detail as well as
        // its requested channel. Re-fitting it against only the nominal sweep's
        // polygon can erase legal shallow contact at a concave F3 boundary.
        // All adopted poses already passed the complete floor/wall/grade guards.
        var referencePaths = source.Concat(constrained.Take(originalPathCount)).ToList();
        diagnostic?.Invoke($"stage=outline-ramps event=reference protectedOriginalPaths={originalPathCount} "
            + "owner=original-request-and-physically-protected-detail artworkGuardRetained=True");
        var originalFootprint = Timed("reference-sweep", () => useParallelSweepUnions
            ? VCarvePhysicalSweep.BuildWithParallelLeafUnions(CommandPaths(referencePaths, schedule.SurfaceZ),
                VCarvePhysicalSweep.ProfileFor(tool, schedule.Depth), 0, SweepTolerance,
                cancellationToken, upperBound: true)
            : VCarvePhysicalSweep.Build(
                CommandPaths(referencePaths, schedule.SurfaceZ), tool, schedule.Depth, 0,
                SweepTolerance, cancellationToken, upperBound: true));
        var originalTarget = PolygonOps.Intersect(originalFootprint, boundary);
        if (options.AccessTarget is { } accessTarget)
            originalTarget = PolygonOps.Intersect(originalTarget, accessTarget.Target);
        // New cutting centers must preserve the entire originally commanded
        // channel, including bounded clearing bands and intentionally uncut
        // interiors. Artwork containment alone would allow widening that channel.
        // Both boundaries share one actual-command subdivision proof so that
        // inserting a new F3 point cannot invalidate an earlier domain proof.
        var artworkProofOwner = wallGuard;
        wallGuard = new CommandClearance(boundary, profile, schedule.SurfaceZ,
            cancellationToken, diagnostic, originalFootprint, pointBudget, artworkProofOwner);
        constrained = Timed("original-channel-protection", () => wallGuard.Apply(constrained)
            .SelectMany(path => LimitGraph(new[] { path }, grade, cancellationToken)).ToList());
        diagnostic?.Invoke($"stage=smooth-channel originalCommandArea={Area(originalFootprint):R} " +
            $"allowedFootprintApproximation={VCarvePhysicalSweep.EffectiveTolerance(SweepTolerance):R} " +
            "allOriginalPaths=True fullCommandChordContainment=True artworkAndOriginalChannel=True");        List<TaggedToolpath> CompleteInitialStockCohort(IEnumerable<IReadOnlyList<PointD>> channelTarget,

            VCarveInitialSourceChordShell.Result? shell, bool fillLimited = false)

        {
            // Audit the selected native and posted cutting source. A bounded
            // fill adopts only authored rows, without demanding completion past
            // its cutoff. Unbounded initial-cohort callers retain their literal
            // union/source-shell policy; neither path grants stock acceptance.
            cancellationToken.ThrowIfCancellationRequested();
            var initialCommands = Timed("initial-stock-command-view", () => CommandPaths(constrained, schedule.SurfaceZ));
            double nominalGrade = MaximumGrade(constrained), actualGrade = MaximumGrade(initialCommands);
            diagnostic?.Invoke($"stage=initial-stock-cohort event=grade nominalMaximumGrade={nominalGrade:R} " +
                $"actualCommandMaximumGrade={actualGrade:R} authoredMaximumGrade={grade:R} nominalEpsilon=1E-12 " +
                "actualGradeScope=literal-float-workspace-F3-stock-relative actualGradeMeasurement=True " +
                "newF3GradeRejectionAdded=False originalNominalGradeAndRouterPolicyRetained=True");
            try
            {
                if (nominalGrade > grade + 1e-12)
                    throw new InvalidOperationException("Initial stock cohort exceeded the original nominal grade policy.");
                if (field is not null)
                    Timed("initial-stock-floor-audit", () => {
                        if(options.ParallelFloorQueryPreparation)field.PrewarmQueries(constrained.SelectMany(p=>p.Points)
                            .Select(p=>{var c=CommandPoint(p,schedule.SurfaceZ);return new PointD(c.X,c.Y);}),pointBudget,diagnostic,"posted-floor-audit");
                        ReportCommandFloorBound(constrained, field, grade, schedule.SurfaceZ, diagnostic);
                        return 0;
                    });
                long auditedChords = Timed("initial-stock-actual-wall-audit", () => {
                    long checkedChords = 0;
                    foreach (var path in constrained)
                    {
                        cancellationToken.ThrowIfCancellationRequested();
                        bool hasProof=protectedWalls.TryGetValue(path.Points,out var proof)
                            && ReferenceEquals(proof.Owner,wallGuard)&&proof.Commands.Length==path.Points.Count;
                        var previous=path.Points.Count>0 ? CommandPoint(path.Points[0],schedule.SurfaceZ) : default;
                        if (path.Points.Count == 1 && CommandPoint(path.Points[0], schedule.SurfaceZ).Z < 0
                            && !wallGuard.IsLegalCommandedCenter(path.Points[0]))
                            throw new InvalidOperationException("Initial stock cohort has an illegal actual cutting point.");
                        for (int i = 1; i < path.Points.Count; i++)
                        {
                            cancellationToken.ThrowIfCancellationRequested();
                            if (++checkedChords > pointBudget)
                                throw new InvalidOperationException("Initial stock cohort exceeded the original command point budget.");
                            var command=CommandPoint(path.Points[i],schedule.SurfaceZ);
                            bool certified=hasProof&&previous==proof.Commands[i-1]&&command==proof.Commands[i];
                            if(certified)wallCertificateHits++;else wallCertificateMisses++;
                            if (!certified&&!wallGuard.IsLegalCommandedChord(path.Points[i - 1], path.Points[i]))
                                throw new InvalidOperationException("Initial stock cohort has an illegal actual cutter chord.");
                            previous=command;
                        }
                    }
                    return checkedChords;
                });
                diagnostic?.Invoke($"stage=protected-wall-custody event=complete reusedChords={wallCertificateHits} literalChords={wallCertificateMisses} "
                    +$"snapshotPoints={certificatePoints} sameImmutableDomain=True actualCommandsRechecked=True monotoneRaisedSameXYOnly=True budgetsUnchanged=True");
                cancellationToken.ThrowIfCancellationRequested();
                diagnostic?.Invoke($"stage=initial-stock-cohort event=complete enabled=True paths={constrained.Count} " +
                    $"points={constrained.Sum(path => (long)path.Points.Count)} auditedActualChords={auditedChords} " +
                    "selectionFrozenForApply=True originalPhysicalSurfaceZ=0 originalFloorArtworkAndGradeRetained=True " +
                    "originalCommandFloorConversionAudit=True fullActualF3WallChordAudit=True " +
                    $"channelPolicy={(shell is null ? "literal-original-union" : "ordered-original-source-chord-shell")} originalChannelUnionBuilt={shell is null} " +

                    $"sourceShellMaximumActualXYDisplacement={shell?.MaximumActualXYDisplacement ?? 0:R} sourceShellMaximumActualPositiveDepthDifference={shell?.MaximumActualPositiveDepthDifference ?? 0:R} " +

                    "openingLowerSweepWholeQAndRecoveryNotRun=True optionalActualF3CompactionNotRun=True " +
                    "coverageClaim=False wholeStockAcceptance=False candidateScope=initial-guarded-stock-cohort");
                progress?.Invoke(1, fillLimited ? "V-carve Fill Limit applied" : "Initial guarded V-carve stock cohort generated", false);
                cancellationToken.ThrowIfCancellationRequested();
                var ordered = constrained.OrderBy(path => path.Category == "final-carve" ? 1 : 0)
                    .ThenBy(path => path.DepthPassIndex ?? int.MaxValue).ToList();
                if (options.RoughingRestWidth <= 0) return ordered;
                // Required cut geometry remains intact until its connecting
                // cuts have been constructed. Only then can whole-cutter stock
                // containment omit a visit without dropping a connector's work.
                return ordered;
            }
            catch (InvalidOperationException error)
            {
                static IEnumerable<IEnumerable<double[]>> InitialRings(IEnumerable<IReadOnlyList<PointD>> rings)
                    => rings.Select(ring => ring.Select(point => new[] { point.x, point.y }));
                // Lazy bounded consumer capture, preserving the original error.
                error.Data["SmoothVCarvingGeometry"] = new {
                    basis = "float-workspace-F3-stock-relative", round = -1, spatialTolerance = CompletionTolerance,
                    fillLimited, authoredFillWidth = options.EdgeFillWidth,
                    physicalCommandSurfaceZ = 0d, goalSurfaceZ = 0d,
                    goalScope = shell is null ? "initial-stock-cohort-no-coverage-goal-or-stock-acceptance"

                        : "initial-stock-cohort-ordered-source-shell-no-union-coverage-or-stock-acceptance",

                    sourceShell = shell, originalChannelUnionBuilt = shell is null,
                    target = InitialRings(channelTarget), actual = InitialRings(Array.Empty<IReadOnlyList<PointD>>()),
                    missing = InitialRings(Array.Empty<IReadOnlyList<PointD>>()), significant = InitialRings(Array.Empty<IReadOnlyList<PointD>>()),
                    artwork = InitialRings(boundary), allowedChannel = InitialRings(channelTarget),
                    commandPaths = initialCommands.Select(path => new {
                        category = path.Category, points = path.Points.Select(point => new[] { point.X, point.Y, point.Z }) })
                };
                throw;
            }
        }

        if (useInitialStockCohort)

            return CompleteInitialStockCohort(originalTarget, null);


        // A legacy rounded-tip F3 graze can gouge a sharp corner even though the
        // nominal trace fitted it. Preserve every physically reachable part of
        // that footprint, rather than asking correction rings to repeat a gouge.
        double minimumEngagementRadius = profile.PhysicalRadiusAtDepth(minimumCorrectionDepth);
        double minimumGoalEngagementRadius = profile.PhysicalRadiusAtDepth(.001);
        // Reachability belongs to the intersection of both cutting domains.
        // Intersecting the old channel with an opening of artwork alone can
        // retain a clipped legacy gouge that no cutter can reach while also
        // staying within the original channel. Fix this admissible target once,
        // before correction; never remove residuals after a failed attempt.

        // The complete original0 upper still owns channel/gouge protection.
        // A separate physical projection defines demand below the1um layer.
        var goalReferenceTarget = originalTarget;
        if (useMaterialLayerGoal)
        {
            var projectedUpper = Timed("material-layer-reference-sweep", () => useParallelSweepUnions
                ? VCarvePhysicalSweep.BuildWithParallelLeafUnions(CommandPaths(source, schedule.SurfaceZ),
                    profile, goalSurfaceZ, SweepTolerance, cancellationToken, upperBound: true)
                : VCarvePhysicalSweep.Build(CommandPaths(source, schedule.SurfaceZ),
                    profile, goalSurfaceZ, SweepTolerance, cancellationToken, upperBound: true));
            goalReferenceTarget = PolygonOps.Intersect(originalTarget, projectedUpper);
            diagnostic?.Invoke($"stage=material-layer-goal event=projected-reference rawZeroReferenceArea={Area(originalTarget):R} "
                +$"projectedRawUpperArea={Area(projectedUpper):R} goalReferenceAreaBeforeOpeningAndArtFringe={Area(goalReferenceTarget):R} "
                +$"goalSurfaceZ={goalSurfaceZ:R} source=full-original-physical-command-paths "
                +"originalZeroChannelRetained=True upperPhysicalProfileAndGridReservesRetained=True "
                +"outsideProjectedUpperOnlyOmittedMaterialAtMost0.001mm=True openingAndArtworkFringeSeparate=True stockAcceptance=False");
        }
        bool conditionedReferenceGoalApplied = false;
        List<List<PointD>> target;
        if (useConditionedReferenceOpening)
            target = Timed("reference-opening", () => VCarveConditionedReferenceOpeningGoal.BuildMaskedGoalForPlane(
                originalTarget, goalReferenceTarget, minimumEngagementRadius, minimumGoalEngagementRadius,
                cancellationToken, diagnostic, out conditionedReferenceGoalApplied));
        else
        {
            var reachable = Timed("reference-opening", () => useMaterialLayerGoal
                ? ReachableReferenceUpperForPlane(originalTarget, minimumEngagementRadius,
                    minimumGoalEngagementRadius, cancellationToken)
                : ReachableReferenceUpper(goalReferenceTarget, minimumEngagementRadius, cancellationToken));
            target = PolygonOps.Intersect(goalReferenceTarget, reachable);
        }
        // Freeze one explicit original-artwork fringe goal before any correction.
        // Original reference/channel/floor/wall/grade owners remain complete.
        target = VCarveArtworkFringeGoal.ApplyToFixedTarget(target, boundary, useArtworkFringeGoal,
            cancellationToken, diagnostic);
        if (gpuIntegrationControl?.FixedTargetPrepared is { } fixedTargetPrepared)
        {
            cancellationToken.ThrowIfCancellationRequested(); fixedTargetPrepared(target);
            cancellationToken.ThrowIfCancellationRequested();
        }
        diagnostic?.Invoke($"stage=smooth-reference originalCommandArea={Area(originalTarget):R} " +
            $"physicallyReachableArea={Area(target):R} referenceAreaOutsideFixedGoal={Area(originalTarget) - Area(target):R} " +
            $"minimumPositiveDepth={minimumCorrectionDepth:R} minimumEngagementRadius={minimumEngagementRadius:R} goalEngagementRadius={minimumGoalEngagementRadius:R} conditionedReferenceGoal={conditionedReferenceGoalApplied} " +
            $"referenceOffsetReserve={8 / PathUtils.Scale:R} referenceArcTolerance={1 / PathUtils.Scale:R} " +
            "reference=opening-of-artwork-intersect-original-channel-upper " +
            "referenceScope=planar-minimum-engagement-opening floorAndGradeReachabilityCertified=False " +
            "reachabilityDomain=both-commanded-cutting-domains targetFixedBeforeCorrection=True originalWallGougesExcluded=True");
        if (useActualF3Compaction)
        {
            // These are the ORIGINAL immutable physical authorities. Neither
            // source/medial/floor nor reference/channel/fixed target is compacted.
            var originalCompactionArtwork = artworkProofOwner;
            var originalCompactionBothWalls = wallGuard;
            var originalCompactionFloor = field;
            var actualFloorProof = originalCompactionFloor is null ? null
                : new ActualF3CompactionFloorProof(originalCompactionFloor, grade, cancellationToken);
            actualF3Compaction = new(schedule.SurfaceZ, point => CommandPoint(point, schedule.SurfaceZ),
                path => ProveActualF3CompactionChord(path, actualFloorProof, originalCompactionArtwork,
                    originalCompactionBothWalls, grade, schedule.SurfaceZ, pointBudget, cancellationToken),
                nominal => MaximumGrade(new[] { nominal }) <= grade + 1e-12,
                cancellationToken, diagnostic, actualFloorProof is null ? null : actualFloorProof.Diagnostics);
            diagnostic?.Invoke("stage=actual-F3-compaction event=selection enabled=True "
                + "selection=RW_TOOLPATHS_VCARVE_ACTUAL_F3_COMPACTION selectionFrozenForApply=True "
                + "originalFloorArtworkChannelTargetFrozen=True beforeInitialLowerSweep=True "
                + "invocationAndSpanPostingOwner=True numericFeedAndToolUuidInvented=False returnedOriginalNominalGradeRequired=True "
                + "trajectoryBoundMm=0.002 machiningToleranceChanged=False stockAuditPending=True");
            constrained = actualF3Compaction.Compact(constrained, "before-initial-lower-sweep");
        }
        var actual = Timed("constrained-sweep", () => BuildLowerSweep(constrained));
        int added = 0;
        var correctionBoundary = new CorrectionBoundary(originalTarget);
        int minimumBoxSourcePoints = gpuIntegrationControl?.MinimumBoxSourcePointsForControls ?? 4096;
        if (minimumBoxSourcePoints is < 0 or > 4096)
            throw new ArgumentOutOfRangeException(nameof(GpuFrontIntegrationControl.MinimumBoxSourcePointsForControls));
        bool boxNeighborhoodEligible = useBoxNeighborhoods && correctionBoundary.SourcePoints >= minimumBoxSourcePoints;
        correctionBoundary.PrepareConditionedNomination(useConditionedCorrectionSource && boxNeighborhoodEligible, cancellationToken, diagnostic);
        diagnostic?.Invoke($"stage=conditioned-correction-source event=selection requested={useConditionedCorrectionSource} "
            + $"boxEligible={boxNeighborhoodEligible} active={correctionBoundary.UsesConditionedNomination} "
            + "selectionFrozenForApply=True nominationOnly=True originalPhysicalOwnersUnchanged=True");
        diagnostic?.Invoke($"stage=box-correction-neighborhood event=source-policy requested={useBoxNeighborhoods} eligible={boxNeighborhoodEligible} "
            + $"canonicalChannelSourcePoints={correctionBoundary.SourcePoints} minimumSourcePoints={minimumBoxSourcePoints} "
            + $"boundedInternalControl={gpuIntegrationControl?.MinimumBoxSourcePointsForControls is not null} "
            + "selectionFrozenForApply=True sourcePolicyFrozenOnce=True smallSourceLiteralSelector=True "
            + "observedCostDomainPolicy=True universalCrossoverClaim=False physicalGuardsUnchanged=True");
        VCarveGpuCorrectionFrontNominator? gpuFrontOwner = null;
        int gpuFrontNominatedComponents = 0;
        long boxNeighborhoodQueries = 0, boxNeighborhoodFallbackQueries = 0;
        double boxNeighborhoodMilliseconds = 0;
        if (useGpuFronts)
        {
            bool ready; string reason;
            try
            {
                if (gpuIntegrationControl?.NominationOwnerFactory is { } factory)
                {
                    gpuFrontOwner = factory(correctionBoundary.OriginalBoundary, cancellationToken, diagnostic);
                    ready = gpuFrontOwner is not null; reason = ready ? "independent-control-nomination-owner" : "control-owner-refused";
                }
                else ready = VCarveGpuCorrectionFrontNominator.TryCreate(correctionBoundary.OriginalBoundary,
                    cancellationToken, diagnostic, out gpuFrontOwner, out reason, policy);
                cancellationToken.ThrowIfCancellationRequested();
                diagnostic?.Invoke($"stage=gpu-correction-front event=selection requested=True available={ready} reason='{reason}' "
                    + "owner=one-generation-original-channel earlyRounds=0..5 originalConstructorOnRefusal=True "
                    + "candidateOnly=True fixedTargetUnchanged=True allOriginalGuardsRequired=True");
                cancellationToken.ThrowIfCancellationRequested();
            }
            catch { gpuFrontOwner?.Dispose(); throw; }
        }
        using var ownedGpuFrontOwner = gpuFrontOwner;
        var seenTerminalRamps = new HashSet<(Point3D, Point3D)>();
        var terminalPathIndices = new HashSet<int>();
        // The fixed target stays identical across correction rounds. Reuse only
        // its verified exact decomposition, never physical coverage or owners.
        var triangleDomainPreparation = new VCarveCoverageCertificate.PreparedTriangleDomain();
        bool floorAwareRamps = policy.FloorAwareRamps;
        int floorAwareNominatedPaths = 0;
        RW.Toolpaths.Milling.Artifacts.FloorAwareCorrectionPoseNominator? floorAwarePoseOwner = null;
        RW.Toolpaths.Milling.Artifacts.FloorAwareRampBatchMaterializer.Owners? floorAwareBatchOwners = null;
        if (floorAwareRamps)
        {
            // Keep directions bound to the ORIGINAL commanded medial, before protection.
            // This immutable influence BVH is built once; each residual query returns at most eight original endpoints.
            var originalSupports = CommandPaths(medial, schedule.SurfaceZ).SelectMany(path => path.Points)
                .Where(point => point.Z < 0).ToArray();
            floorAwarePoseOwner = Timed("floor-aware-original-support-index", () => new RW.Toolpaths.Milling.Artifacts.FloorAwareCorrectionPoseNominator(originalSupports,
                point => CommandPoint(point, schedule.SurfaceZ),
                point => field is null ? Math.Max(0, -point.Z) : field.CappedDepth(-point.Z, point.X, point.Y),
                wallGuard.NominateEndpointAllowedDepth,
                ceiling => QuantizeCorrectionDepth(ceiling, schedule.SurfaceZ),
                profile.PhysicalRadiusAtDepth, CoverageTolerance / 2 - 1e-8,
                grade, schedule.Depth, schedule.DepthPerPass, source[0].RegionIndex, schedule.FirstPassIndex,
                cancellationToken));
            List<TaggedToolpath> FloorAwareProtect(IReadOnlyList<TaggedToolpath> paths)
            {
                var capped = field is null ? paths.ToList() : Constrain(paths, field, grade, cancellationToken, pointBudget);
                var artwork = artworkProofOwner.Apply(capped)
                    .SelectMany(path => LimitGraph(new[] { path }, grade, cancellationToken)).ToList();
                var both = wallGuard.Apply(artwork)
                    .SelectMany(path => LimitGraph(new[] { path }, grade, cancellationToken)).ToList();
                if (both.Count != paths.Count) throw new InvalidOperationException("Floor-aware protection changed path ordinals.");
                return both;
            }
            bool ProveActualFloor(IReadOnlyList<TaggedToolpath> commands)
            {
                if (field is null) return true;
                var proved = Constrain(commands, field, grade, cancellationToken, pointBudget);
                return proved.Count == commands.Count
                    && Enumerable.Range(0, commands.Count).All(i => proved[i].Points.SequenceEqual(commands[i].Points));
            }
            bool ProveActualWalls(TaggedToolpath path)
            {
                for (int i = 0; i < path.Points.Count; i++)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    Point3D point = path.Points[i];
                    if (CommandPoint(point, schedule.SurfaceZ) != point) return false;
                    if (point.Z < 0 && !wallGuard.IsLegalCommandedCenter(point)) return false;
                    if (i > 0 && !wallGuard.IsLegalCommandedChord(path.Points[i - 1], point)) return false;
                }
                return true;
            }
            floorAwareBatchOwners = new(point => CommandPoint(point, schedule.SurfaceZ), FloorAwareProtect,
                paths => CommandPaths(paths, schedule.SurfaceZ), ProveActualFloor, ProveActualWalls, MaximumGrade);
            diagnostic?.Invoke($"stage=floor-aware-correction event=owner-prepared originalCommandedMedialSupports={originalSupports.Length} "
                + "preparedIndexOnce=True maximumTargets=128 physicalInfluenceDirections=8 sitesPerDirection=16 maximumRequests=512 maximumCandidatesPerTarget=4 queryVisitCap=65536 "
                + "originalFloorWallGradeOwnersReused=True targetUnchanged=True finalWholeTargetCertificateRequired=True stockComparisonRequired=True");
        }
        double previousArea = double.PositiveInfinity;
        VCarveCoverageCertificate.Result? latestCoverageCertificate = null;
        for (int round = 0; round <= MaximumCorrectionRounds; round++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            progress?.Invoke(.2 + .75 * round / MaximumCorrectionRounds,
                $"Checking smooth channel coverage: correction {round}", true);
            var missing = Timed($"coverage-difference-{round}", () => PolygonOps.Difference(target, actual));
            // The spatial stopping test retains all slivers wider than the
            // remaining surface budget, independently of their area.
            // Area alone would silently discard a long, narrow uncut channel.
            var significant = ResidualCentersUpper(missing, CompletionTolerance / 2, cancellationToken);
            double area = Area(missing), remaining = Area(significant);
            diagnostic?.Invoke($"stage=smooth-coverage round={round} referenceArea={Area(target):R} " +
                $"missedArea={area:R} significantArea={remaining:R} residualRings={missing.Count} " +
                $"correctionPaths={added} spatialTolerance={CompletionTolerance:R} constructionTolerance={CoverageTolerance:R} gapRadius={CompletionTolerance/2:R} bounds={Bounds(missing)} " +
                "geometryBasis=float-workspace-F3-stock-relative");
            if (floorAwarePoseOwner is not null && floorAwareBatchOwners is not null
                && significant.Count != 0 && round < MaximumCorrectionRounds)
            {
                var floorAware = Timed($"floor-aware-recovery-{round}", () =>
                {
                    var nominated = floorAwarePoseOwner.Nominate(
                        ResidualRepresentativeTargets(significant, cancellationToken, pointBudget),
                        cancellationToken, log: diagnostic);
                    return RW.Toolpaths.Milling.Artifacts.FloorAwareRampBatchMaterializer.Materialize(
                        nominated.Requests, floorAwareBatchOwners, cancellationToken, diagnostic);
                });
                if (floorAware.ActualRampPaths.Count != 0)
                {
                    long newPoints = floorAware.ActualRampPaths.Sum(path => (long)path.Points.Count);
                    long previousPoints = constrained.Sum(path => (long)path.Points.Count);
                    long cumulativePoints = checked(previousPoints + newPoints);
                    diagnostic?.Invoke($"stage=floor-aware-correction event=point-budget round={round} existingPoints={previousPoints} newPoints={newPoints} "
                        + $"cumulativePoints={cumulativePoints} pointBudget={pointBudget}");
                    if (cumulativePoints > pointBudget) throw new InvalidOperationException("Floor-aware correction exceeded the original cumulative point budget.");
                    constrained.AddRange(floorAware.ActualRampPaths);
                    added = checked(added + floorAware.ActualRampPaths.Count);
                    floorAwareNominatedPaths = checked(floorAwareNominatedPaths + floorAware.ActualRampPaths.Count);
                    var newSweep = Timed($"floor-aware-lower-sweep-{round}", () => BuildLowerSweep(floorAware.ActualRampPaths));
                    actual = Timed($"floor-aware-coverage-union-{round}", () => PolygonOps.Union(actual.Concat(newSweep)));
                    // The original collector still handles every remaining obligation in THIS round.
                    missing = Timed($"floor-aware-coverage-difference-{round}", () => PolygonOps.Difference(target, actual));
                    significant = Timed($"floor-aware-residual-centers-upper-{round}", () => ResidualCentersUpper(missing, CompletionTolerance / 2, cancellationToken));
                    area = Area(missing); remaining = Area(significant);
                }
                diagnostic?.Invoke($"stage=floor-aware-correction event=protected round={round} admittedPaths={floorAware.ActualRampPaths.Count} "
                    + $"cumulativePaths={floorAwareNominatedPaths} protectCalls={floorAware.ProtectCalls} stabilizationCalls={floorAware.StabilizationCalls} "
                    + $"remainingArea={remaining:R} originalCollectorRetained=True originalWholeTargetUnchanged=True stockDepthAcceptance=False");
            }
            bool continuousCoverageCertified = false;
            int witnessPrunedPaths = 0;
            bool denseTerminalRecovery = round >= MaximumCorrectionRounds - 1;
            bool requireGpuCompletionCertificate = gpuFrontNominatedComponents != 0 && significant.Count == 0;
            bool requireFloorAwareCompletionCertificate = floorAwareNominatedPaths != 0 && significant.Count == 0;
            bool requireBoxCompletionCertificate = (boxNeighborhoodQueries != 0 || correctionBoundary.NominationQueries != 0) && significant.Count == 0;
            bool requireConditionedReferenceCompletionCertificate = conditionedReferenceGoalApplied && significant.Count == 0;
            bool originalCertificateRequested = significant.Count != 0 && denseTerminalRecovery
                || significant.Count == 0 && terminalPathIndices.Count >= MinimumTerminalPathsForWitnessPruning;
            if (originalCertificateRequested || requireGpuCompletionCertificate || requireFloorAwareCompletionCertificate || requireBoxCompletionCertificate || requireConditionedReferenceCompletionCertificate)
            {
                // The lower polygon sweep can leave a conservative fringe even
                // when real continuous commanded cutter discs reach all stock.
                // Certify the WHOLE immutable target, never a residual selection
                // or an area cutoff. Exhaustion remains an unverified failure.
                progress?.Invoke(.98, "Checking continuous cutter coverage", true);
                long commandedIntervals = constrained.Sum(path => (long)Math.Max(0, path.Points.Count - 1));
                long targetSegments = target.Sum(ring => (long)ring.Count);
                long certificateComplexity = commandedIntervals + targetSegments;
                var certificateLimits = new VCarveCoverageCertificate.Limits
                {
                    // The saved dense replay traversed 16.7m boxes before a
                    // possible gap, rather than completing the whole domain.
                    // Scale headroom with both command and target complexity.
                    MaximumBoxes = Math.Max(500_000, Math.Min(80_000_000, checked(16 * certificateComplexity))),
                    MaximumDiscEvaluations = Math.Max(20_000_000, Math.Min(750_000_000, checked(192 * certificateComplexity))),
                    MaximumInputSegments = (int)Math.Min(int.MaxValue, Math.Max(4_000_000,
                        checked(2 * certificateComplexity))),
                    MaximumElapsed = TimeSpan.FromSeconds(Math.Clamp(certificateComplexity / 14_000.0, 30, 180)),
                    // GPU-only completion adds proof, never a new permission
                    // to remove cutting rays by surface-only witnesses.
                    RecordWitnessPathIndices = !useMaterialLayerGoal && originalCertificateRequested && terminalPathIndices.Count != 0,
                    OptionalWitnessPathIndices = !useMaterialLayerGoal && originalCertificateRequested ? terminalPathIndices : null,
                    GenerationPolicy = policy,
                    UseGpuDiscWitnesses = policy.GpuDiscWitnesses,
                    UseGpuTriangleDomain = policy.GpuTriangles,
                    UseContourBoxSeeds = policy.ContourSeeds,
                    UseGpuBoundaryNominations = policy.GpuBoundary,
                    TriangleDomainPreparation = triangleDomainPreparation,
                };
                if ((requireGpuCompletionCertificate || requireFloorAwareCompletionCertificate || requireBoxCompletionCertificate || requireConditionedReferenceCompletionCertificate)
                    && gpuIntegrationControl?.ForceCompletionCertificateInconclusive == true)
                    certificateLimits = certificateLimits with { MaximumBoxes = 1 };
                diagnostic?.Invoke($"stage=smooth-physical-certificate event=limits sourcePoints={sourcePointCount} " +
                    $"round={round} samplingTier={(significant.Count == 0 ? "lower-sweep-pass-pruning" : round == MaximumCorrectionRounds ? "after-dense-recovery" : "before-dense-recovery")} " +
                    $"gpuCompletionRequired={requireGpuCompletionCertificate} floorAwareCompletionRequired={requireFloorAwareCompletionCertificate} " +
                    $"boxCompletionRequired={requireBoxCompletionCertificate} boxNeighborhoodQueries={boxNeighborhoodQueries} " +
                    $"floorAwareNominatedPaths={floorAwareNominatedPaths} originalCertificateRequested={originalCertificateRequested} nominatedComponents={gpuFrontNominatedComponents} terminalPaths={terminalPathIndices.Count} " +
                    $"commandedIntervals={commandedIntervals} targetSegments={targetSegments} workComplexity={certificateComplexity} " +
                    $"maximumBoxes={certificateLimits.MaximumBoxes} maximumDiscEvaluations={certificateLimits.MaximumDiscEvaluations} " +
                    $"maximumInputSegments={certificateLimits.MaximumInputSegments} " +
                    $"contourBoxSeedsRequested={certificateLimits.UseContourBoxSeeds} " +
                    $"maximumElapsedSeconds={certificateLimits.MaximumElapsed.TotalSeconds:R} " +
                    "budgetRule=source-scaled-with-dense-replay-caps wholeOriginalCoverageTarget=True exhaustionMeans=inconclusive");
                var certificateCommands = CommandPaths(constrained, schedule.SurfaceZ);
                // Preserve actual original0 command poses for machining guards
                // and failure payload. Translate ONLY this Q analysis view.
                certificateCommands = VCarveMaterialLayerGoal.ForCertificate(certificateCommands, goalSurfaceZ, cancellationToken);
                if (certificateCommands.Count != constrained.Count)
                    throw new InvalidOperationException("Smooth V-carving command snapshot changed source path ordinals.");
                if (CertificateObserver.Value is { } observer)
                    observer(new CertificateInputs(options.DiagnosticContext,
                        certificateCommands.Count == 0 ? -1 : certificateCommands[0].RegionIndex, round,
                        target, certificateCommands, profile, 0, CompletionTolerance / 2, certificateLimits) { GoalAnalysisSurfaceZ = goalSurfaceZ });
                var certificate = Timed("whole-target-physical-certificate", () => VCarveCoverageCertificate.Certify(
                    target, certificateCommands, profile,
                    gapRadius: CompletionTolerance / 2, cancellationToken: cancellationToken,
                    limits: certificateLimits, diagnostic: diagnostic));
                if (requireGpuCompletionCertificate && gpuIntegrationControl?.CompletionObserved is { } completionObserved)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    completionObserved(new(significant.Count == 0, terminalPathIndices.Count, gpuFrontNominatedComponents, originalCertificateRequested,
                        new(options.DiagnosticContext, certificateCommands.Count == 0 ? -1 : certificateCommands[0].RegionIndex,
                            round, target, certificateCommands, profile, 0, CompletionTolerance / 2, certificateLimits) { GoalAnalysisSurfaceZ = goalSurfaceZ }, certificate));
                    cancellationToken.ThrowIfCancellationRequested();
                }
                latestCoverageCertificate = certificate;
                continuousCoverageCertified = certificate.Certified;
                if (requireGpuCompletionCertificate && !certificate.Certified)
                    throw CoverageFailure($"Smooth V-carving GPU correction nominations require the complete "
                        + $"unchanged target certificate before completion; reason={certificate.Reason}, round={round}, "
                        + $"nominatedComponents={gpuFrontNominatedComponents}.", round, missing, significant);
                if (requireFloorAwareCompletionCertificate && !certificate.Certified)
                    throw CoverageFailure($"Floor-aware correction requires the unchanged complete target certificate; reason={certificate.Reason}, round={round}.",
                        round, missing, significant);
                if (requireBoxCompletionCertificate && !certificate.Certified)
                    throw CoverageFailure($"Box correction nomination requires the unchanged complete target certificate; reason={certificate.Reason}, round={round}.",
                        round, missing, significant);
                if (requireConditionedReferenceCompletionCertificate && !certificate.Certified)
                    throw CoverageFailure($"Conditioned reference opening requires the complete fixed goal certificate; reason={certificate.Reason}, round={round}.",
                        round, missing, significant);
                if (!useMaterialLayerGoal && originalCertificateRequested && certificate.Certified && certificate.WitnessPathsRecorded)
                {
                    int before = constrained.Count;
                    constrained = PruneTerminalPaths(constrained, terminalPathIndices, certificate, diagnostic);
                    witnessPrunedPaths = before - constrained.Count;
                    if (witnessPrunedPaths > added)
                        throw new InvalidOperationException("Smooth V-carving witness pruning removed an untracked source path.");
                    added -= witnessPrunedPaths;
                }
            }
            if (significant.Count == 0 || continuousCoverageCertified)
            {
                diagnostic?.Invoke($"stage=smoothing event=complete correctionRounds={round} " +
                    $"correctionPaths={added} paths={constrained.Count} maximumGrade={MaximumGrade(constrained):R} " +
                    $"remainingArea={area:R} widthVerified=True " +
                    $"witnessPrunedPaths={witnessPrunedPaths} residualAreaBasis={(witnessPrunedPaths == 0 ? "current-lower-sweep" : "before-witness-pruning")} " +
                    $"coverageProof={(continuousCoverageCertified ? "whole-target-real-continuous-discs" : "lower-sweep-and-outer-residual-erosion")}");
                if (field is not null) ReportCommandFloorBound(constrained, field, grade, schedule.SurfaceZ, diagnostic);
                progress?.Invoke(1, "Smooth V-carve coverage verified", false);
                // Corrective clearing retains shallow-to-deep scheduling and
                // finishes before the medial surface pass.
                return constrained.OrderBy(path => path.Category == "final-carve" ? 1 : 0)
                    .ThenBy(path => path.DepthPassIndex ?? int.MaxValue).ToList();
            }
            if (round == MaximumCorrectionRounds)
                throw CoverageFailure($"Smooth V-carving could not restore channel coverage within {CompletionTolerance:R} mm " +
                    $"after {round} correction rounds: missed area={area:R}, significant area={remaining:R}, bounds={Bounds(missing)}.",
                    round, missing, significant);

            diagnostic?.Invoke($"stage=smooth-work event=begin work=correction-paths-{round}");
            var correctionTimer = Stopwatch.StartNew();
            var correction = new List<TaggedToolpath>();
            var terminalRequests = new List<(int First, int Last, List<List<PointD>> Error, double Radius,
                double Reach, List<TaggedToolpath> Discovery)>();
            long discoveryPoints = 0;
            long nominationRings = 0, nominationPoints = 0;
            long neighborhoodSelectionQueries = 0, neighborhoodDisjointSkips = 0, neighborhoodContactProbes = 0;
            double neighborhoodSelectionMilliseconds = 0;
            bool ordinaryCandidates = options.UseOrdinaryCorrectionCandidates && round < MaximumCorrectionRounds - 1;
            double nominationArc = options.UseOrdinaryCorrectionCandidates && round >= MaximumCorrectionRounds - 1
                ? ReferenceCorrectionArcTolerance : options.CorrectionArcTolerance;
            diagnostic?.Invoke($"stage=smooth-candidates event=begin round={round} ordinaryCollector={ordinaryCandidates} " +
                $"preserveTerminalPaths={!ordinaryCandidates} nominationArc={nominationArc:R} firstArc={ReferenceCorrectionArcTolerance:R} " +
                "candidateStockAuthority=False originalSourceRegionsAndPitch=True originalRawInsetAndDepth=True " +
                "physicalFloorAndDualCommandWalls=True fixedOriginalCoverageTarget=True " +
                $"physicalFrontCompactionSkipped={ordinaryCandidates} finishedStockEquivalence=requires-independent-audit");
            var correctionComponents = PartitionCorrectionComponents(significant, cancellationToken);
            // Query the mutable floor cache on the caller thread. All component
            // construction below then consumes immutable cap/source snapshots.
            var componentCaps = new double[correctionComponents.Count];
            double sourceCap = field is null ? source.SelectMany(path => path.Points).Max(point => -point.Z) : 0;
            int cachedFloorQueriesBefore = field?.CachedQueryCount ?? 0;
            long floorCapsStarted = Stopwatch.GetTimestamp();
            diagnostic?.Invoke($"stage=smooth-floor-caps event=begin round={round} components={correctionComponents.Count} " +
                $"outerPoints={correctionComponents.Sum(component => (long)component.Outer.Count)} floorField={field is not null} " +
                $"cachedQueriesBefore={cachedFloorQueriesBefore} owner=original-medial-floor captureHasNotStarted=True");
            for (int component = 0; component < correctionComponents.Count; component++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                componentCaps[component] = field is null ? sourceCap
                    : correctionComponents[component].Outer.Select(point => field.Depth(point.x, point.y)).Max();
            }
            diagnostic?.Invoke($"stage=smooth-floor-caps event=complete round={round} components={correctionComponents.Count} " +
                $"elapsedMs={Stopwatch.GetElapsedTime(floorCapsStarted).TotalMilliseconds:F3} " +
                $"cachedQueriesAfter={field?.CachedQueryCount ?? 0} newExactXYQueries={(field?.CachedQueryCount ?? 0) - cachedFloorQueriesBefore} " +
                "capRule=literal-maximum-over-all-outer-vertices captureHasNotStarted=True " +
                (field?.QueryScratchDiagnostics ?? "floorFieldAbsent=True") + " " + (field?.SupportConstantDiagnostics ?? "") + " " + (field?.SupportDominanceDiagnostics ?? ""));
            if (CorrectionConstructorObserver.Value is { } constructorObserver)
            {
                for (int component = 0; component < correctionComponents.Count; component++)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    var error = correctionComponents[component];
                    double cap = componentCaps[component];
                    double initialDepth = Math.Min(schedule.Depth, cap * Math.Pow(.5, round + 1));
                    initialDepth = Math.Max(minimumCorrectionDepth, initialDepth - DepthReserve);
                    if (round == MaximumCorrectionRounds - 1) initialDepth = minimumCorrectionDepth;
                    constructorObserver(new CorrectionConstructorInputs(options.DiagnosticContext,
                        source[0].RegionIndex, round, component, correctionBoundary.OriginalBoundary,
                        new[] { (IReadOnlyList<PointD>)error.Outer }.Concat(error.Holes).ToArray(),
                        cap, initialDepth, profile, schedule, options.StepOver, nominationArc,
                        ordinaryCandidates, options.DetectCurves, pointBudget, correctionComponents.Count));
                }
            }
            int requestedWorkers = ResolveCorrectionWorkers(correctionWorkers);
            // A newly published whole-source inset may replace a later local
            // computation. Preserve that serial representation choice exactly.
            // Use the authored upper depth through the real command-radius
            // adapter. Quantizing that bound downward could understate a later
            // stable correction depth at an unusual workspace surface.
            double maximumDepth = Math.Max(minimumCorrectionDepth, schedule.Depth);
            double maximumRadius = MedialAxisToolpaths.CommandedCuttingRadius(profile, 0, 0, 0,
                maximumDepth, schedule.SurfaceZ);
            double terminalRadius = MedialAxisToolpaths.CommandedCuttingRadius(profile, 0, 0, 0,
                minimumCorrectionDepth, schedule.SurfaceZ);
            double maximumAdditionalReach = profile.RoundedTip
                ? Math.Max(maximumRadius, TerminalRelevanceReach(terminalRadius)) : 0;
            bool serialCacheOwner = correctionBoundary.HasWholeSourceInsets || correctionBoundary.HasWholeNominationInsets
                || correctionComponents.Any(component => correctionBoundary.CouldCropWholeSource(
                    component.Outer, maximumRadius, maximumAdditionalReach));
            int workers = serialCacheOwner ? 1 : Math.Min(requestedWorkers, Math.Max(1, correctionComponents.Count));
            var roundDepthDiagnostics = diagnostic is null ? null : new CorrectionDepthDiagnostics();
            var correctionDepthRanges = diagnostic is null ? null : new List<(int First, int Last, int Bin)>();
            diagnostic?.Invoke($"stage=smooth-concentric event=partition round={round} " +
                $"components={correctionComponents.Count} maximumTileSpan=32 maximumTilePoints=4096 " +
                $"workers={workers} requestedWorkers={requestedWorkers} " +
                $"workerPolicy={(serialCacheOwner ? "whole-source-cache-publication" : "independent-components")} " +
                "minimumAdaptiveSpan=4 allErrorAreaRetained=True originalCoverageTargetUnchanged=True");
            CorrectionComponentOutput BuildComponent(int currentComponent)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var componentTimer = Stopwatch.StartNew();
                var component = correctionComponents[currentComponent];
                var componentCorrection = new List<TaggedToolpath>();
                var componentTerminalRequests = new List<CorrectionTerminalRequest>();
                List<string>? componentDiagnostics = diagnostic is null ? null : new();
                long componentDiscoveryPoints = 0;
                var componentDepthDiagnostics = diagnostic is null ? null : new CorrectionDepthDiagnostics();
                try
                {
                componentDiagnostics?.Add($"stage=smooth-concentric event=begin round={round} component={currentComponent} " +
                    $"errorPoints={component.Outer.Count + component.Holes.Sum(hole => hole.Count)} " +
                    $"errorBounds={Bounds(new[] { component.Outer })}");
                // Start below the local floor. If a constrained ring loses some of
                // this reach, later rounds reduce depth and approach the wall more closely.
                double cap = componentCaps[currentComponent];
                double cutDepth = Math.Min(schedule.Depth, cap * Math.Pow(.5, round + 1));
                cutDepth = Math.Max(minimumCorrectionDepth, cutDepth - DepthReserve);
                // The fixed reference includes minimum-engagement reach. The
                // last correction must actually attempt it, even when eight
                // halvings of a deep local floor still leave a larger radius.
                if (round == MaximumCorrectionRounds - 1) cutDepth = minimumCorrectionDepth;
                double radius = 0;
                double neighborhoodMilliseconds = 0, centerSelectionMilliseconds = 0;
                // Candidate-only seam precedes raw inset, error dilation and
                // filled-region construction. Refusal falls through unchanged.
                if (gpuFrontOwner is not null && round is >= 0 and <= 5 && cutDepth > minimumCorrectionDepth)
                {
                    var nominationInput = new CorrectionConstructorInputs(options.DiagnosticContext,
                        source[0].RegionIndex, round, currentComponent, correctionBoundary.OriginalBoundary,
                        new[] { (IReadOnlyList<PointD>)component.Outer }.Concat(component.Holes).ToArray(),
                        cap, cutDepth, profile, schedule, options.StepOver, nominationArc,
                        ordinaryCandidates, options.DetectCurves, pointBudget, correctionComponents.Count);
                    if (gpuFrontOwner.TryNominate(nominationInput, cancellationToken, out var gpuPaths, out string refusal))
                    {
                        Interlocked.Increment(ref gpuFrontNominatedComponents);
                        componentDiagnostics?.Add($"stage=smooth-concentric event=gpu-front-nomination round={round} "
                            + $"component={currentComponent} paths={gpuPaths.Count} "
                            + $"points={gpuPaths.Sum(path => (long)path.Points.Count)} "
                            + "beforeRawInsetAndErrorDilation=True stockCredit=0 terminalOwnership=False "
                            + "snapFloorBothWallsGradeAndWholeOriginalTargetProofRequired=True");
                        return new(gpuPaths, componentTerminalRequests, componentDiagnostics,
                            NominationRings: gpuPaths.Count,
                            NominationPoints: gpuPaths.Sum(path => (long)path.Points.Count),
                            DepthStatistics: componentDepthDiagnostics?.Final(double.NaN, gpuPaths.Count,
                                gpuPaths.Sum(path => (long)path.Points.Count), gpuPaths.Sum(path => (long)path.Points.Count), 0));
                    }
                    componentDiagnostics?.Add($"stage=smooth-concentric event=gpu-front-refusal round={round} "
                        + $"component={currentComponent} reason='{refusal}' fallback=original-constructor "
                        + "emptyNominationNeverDeclaresUnreachable=True");
                }
                var preparedNeighborhood = new VCarveBoxCorrectionNeighborhood(
                    new[] { component.Outer }.Concat(component.Holes), cancellationToken, boxNeighborhoodEligible);
                double cropMilliseconds = 0, insetMilliseconds = 0;
                var regions = new List<MillingRegion>();
                for (int depthAttempt = 0; depthAttempt < 16; depthAttempt++)
                {
                    double requestedDepth = cutDepth;
                    cutDepth = QuantizeCorrectionDepth(cutDepth, schedule.SurfaceZ);
                    componentDepthDiagnostics?.CommandedAttempt(cutDepth);
                    componentDiagnostics?.Add($"stage=smooth-concentric event=command-depth round={round} component={currentComponent} " +
                        $"attempt={depthAttempt} requestedDepth={requestedDepth:R} commandedDepth={cutDepth:R} " +
                        $"minimumEngagementFinalAttempt={round == MaximumCorrectionRounds - 1} radiusAndPitchUseCommandedDepth=True");
                    if (cutDepth <= 0) break;
                    if (useMaterialLayerGoal && cutDepth < minimumCorrectionDepth) break;
                    radius = MedialAxisToolpaths.CommandedCuttingRadius(profile, 0, 0, 0,
                        cutDepth, schedule.SurfaceZ);
                    if (radius <= 1e-7) break;
                    double terminalReach = profile.RoundedTip && cutDepth <= minimumCorrectionDepth ? radius : 0;
                    long cropStarted = Stopwatch.GetTimestamp();
                    var localBoundary = correctionBoundary.CropNomination(component.Outer, radius, cancellationToken, terminalReach);
                    cropMilliseconds += Stopwatch.GetElapsedTime(cropStarted).TotalMilliseconds;
                    bool cached = correctionBoundary.TryGetWholeNominationInset(radius, out var legal, nominationArc);
                    componentDiagnostics?.Add($"stage=smooth-concentric event=inset-begin round={round} component={currentComponent} " +
                        $"attempt={depthAttempt} radius={radius:R} sourcePoints={correctionBoundary.SourcePoints} " +
                        $"localSourcePoints={localBoundary.Sum(ring => ring.Count)} cachedWholeSource={cached} " +
                        $"nominationArcTolerance={nominationArc:R} ordinaryCollector={ordinaryCandidates} " +
                        $"additionalTerminalReach={terminalReach:R} " +
                        $"conditionedNomSource={correctionBoundary.UsesConditionedNomination} constructorRadius={radius + correctionBoundary.NominationInwardPadMm:R} " +
                        $"constructorOnlyInwardPadMm={correctionBoundary.NominationInwardPadMm:R} physicalRadiusUnchanged=True " +
                        "cropPadding=twice-radius-plus-terminal-reach-plus-coverage-and-command-reserve originalCommandWallProof=True");
                    var insetTimer = Stopwatch.StartNew();
                    if (!cached)
                    {
                        legal = CorrectionCenters(localBoundary, radius + correctionBoundary.NominationInwardPadMm, cancellationToken, nominationArc);
                        correctionBoundary.RememberWholeNominationInset(localBoundary, radius, legal, nominationArc);
                    }
                    insetMilliseconds += insetTimer.Elapsed.TotalMilliseconds;
                    componentDepthDiagnostics?.Inset(cutDepth, insetTimer.Elapsed.TotalMilliseconds);
                    componentDiagnostics?.Add($"stage=smooth-concentric event=inset-complete round={round} component={currentComponent} " +
                        $"attempt={depthAttempt} elapsedMs={insetTimer.Elapsed.TotalMilliseconds:F1}");
                    // The expanded error chooses relevant centers. It is not stock:
                    // centers and cutter sweeps may overlap already cleared material.
                    var candidateTimer = Stopwatch.StartNew();
                    var centers = preparedNeighborhood.SelectCenters(legal,
                        radius + terminalReach + CoverageTolerance, cancellationToken);
                    neighborhoodMilliseconds += candidateTimer.Elapsed.TotalMilliseconds;
                    candidateTimer.Restart();
                    regions = PolygonOps.DifferenceToComponents(centers, Array.Empty<IReadOnlyList<PointD>>())
                        .Select(region => new MillingRegion(region.Outer,
                            region.Holes.Select(hole => (IReadOnlyList<PointD>)hole).ToList())).ToList();
                    centerSelectionMilliseconds += candidateTimer.Elapsed.TotalMilliseconds;
                    if (regions.Count > 0 || cutDepth <= minimumCorrectionDepth) break;
                    cutDepth = Math.Max(minimumCorrectionDepth, cutDepth / 2);
                }
                if (regions.Count == 0)
                {
                    componentDiagnostics?.Add($"stage=smooth-concentric event=no-reachable-centers round={round} component={currentComponent} " +
                        $"depth={cutDepth:R} floorCap={cap:R} radius={radius:R} errorArea={Math.Abs(Clipper.Area(new PathD(component.Outer))):R} " +
                        $"errorBounds={Bounds(new[] { component.Outer })} referenceIsUpperBound=True");
                    return new(componentCorrection, componentTerminalRequests, componentDiagnostics,
                        NeighborhoodSelectionQueries: preparedNeighborhood.SelectionQueries,
                        NeighborhoodDisjointSkips: preparedNeighborhood.DisjointSkips,
                        NeighborhoodContactProbes: preparedNeighborhood.ContactProbes,
                        NeighborhoodSelectionMilliseconds: preparedNeighborhood.SelectionMilliseconds,
                        BoxNeighborhoodQueries: preparedNeighborhood.BoxQueries,
                        BoxNeighborhoodFallbackQueries: preparedNeighborhood.OriginalFallbackQueries,
                        BoxNeighborhoodMilliseconds: preparedNeighborhood.BoxMilliseconds,
                        DepthStatistics: componentDepthDiagnostics?.Final(cutDepth, 0, 0, 0, 0,
                            cropMilliseconds, neighborhoodMilliseconds, preparedNeighborhood.PreparationMilliseconds,
                            preparedNeighborhood.OffsetMilliseconds, preparedNeighborhood.ConversionMilliseconds, centerSelectionMilliseconds));
                }
                double diameter = 2 * profile.RadiusAtDepth(useMaterialLayerGoal ? Math.Max(0, cutDepth + goalSurfaceZ) : cutDepth);
                double pitch = options.StepOver * (profile.RoundedTip
                    ? Math.Min(profile.ClearingDiameter, diameter) : diameter);
                pitch = Math.Clamp(pitch, .0001, Math.Max(.0001, diameter * .8));
                var candidateTolerances = GeometryTolerances.Default with
                    { ArcTolerance = nominationArc, SimplifyTolerance = 0 };
                var candidateRingOptions = new RingOffsetOptions(0, pitch)
                    {
                        MinimumRingArea = 0,
                        RingLimit = 1000,
                        Tolerances = candidateTolerances,
                        // Ordinary fill does not install the physical collector's
                        // native-tick first-front policy. Keep it explicitly.
                        FirstLevelTolerances = ordinaryCandidates ? candidateTolerances with
                            { ArcTolerance = ReferenceCorrectionArcTolerance, SimplifyTolerance = 0 } : null,
                    };
                long collectorStarted = componentDepthDiagnostics is null ? 0 : Stopwatch.GetTimestamp();
                List<OffsetRing> rings;
                try
                {
                    rings = MedialAxisGeometry.BuildConcentricFill(regions, candidateRingOptions,
                        radius, preserveTerminalPaths: !ordinaryCandidates, cancellationToken: cancellationToken);
                }
                catch (InvalidOperationException error) when (!ordinaryCandidates && error.Message
                    == "A coherent inward offset front could not be certified above the geometry quantum.")
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    componentDiagnostics?.Add($"stage=smooth-concentric event=nomination-fallback round={round} "
                        + $"component={currentComponent} reason=physical-offset-front-quantum radius={radius:R} pitch={pitch:R} "
                        + "nomination=ordinary-offset-rings completeFloorWallGradeAndFixedTargetProofRequired=True");
                    // These rings nominate centers only. A constructor's failed
                    // front proof is never cutting/stock credit; the same Protect
                    // and complete fixed-target completion test still follow.
                    rings = MedialAxisGeometry.BuildConcentricFill(regions, candidateRingOptions,
                        radius, preserveTerminalPaths: false, cancellationToken: cancellationToken);
                }
                if (componentDepthDiagnostics is not null) componentDepthDiagnostics.Collector(cutDepth, Stopwatch.GetElapsedTime(collectorStarted).TotalMilliseconds);
                long componentNominationRings = rings.Count;
                long componentNominationPoints = rings.Sum(ring => (long)ring.Points.Count);
                int pass = Enumerable.Range(0, schedule.PassCount)
                    .FirstOrDefault(index => schedule.SurfaceZ - schedule.PassBottomZ(index) >= cutDepth);
                int firstRing = componentCorrection.Count;
                var materializationTimer = Stopwatch.StartNew();
                foreach (OffsetRing ring in rings)
                {
                    var points = MedialAxisToolpaths.MaterializeClearingRing(ring, -cutDepth);
                    if (points.Count >= 2)
                        componentCorrection.Add(new TaggedToolpath(points, Array.Empty<ToolpathSpan>(),
                            source[0].RegionIndex, "clearing", schedule.FirstPassIndex + pass,
                            detectCurves: options.DetectCurves));
                }
                if (profile.RoundedTip && cutDepth <= minimumCorrectionDepth && componentCorrection.Count > firstRing)
                {
                    var error = new[] { component.Outer }.Concat(component.Holes).ToList();
                    double relevance = round == MaximumCorrectionRounds - 1 ? TerminalRelevanceReach(radius) : radius;
                    var discovery = new List<TaggedToolpath>();
                    if (relevance > radius)
                    {
                        // Keep the wider ring collector as an endpoint candidate
                        // pool. It must not schedule an enlarged clearing band:
                        // only proved useful tangent rays are emitted from it.
                        var discoveryBoundary = correctionBoundary.CropNomination(component.Outer, radius,
                            cancellationToken, relevance);
                        if (!correctionBoundary.TryGetWholeNominationInset(radius, out var discoveryLegal, nominationArc))
                            discoveryLegal = CorrectionCenters(discoveryBoundary, radius + correctionBoundary.NominationInwardPadMm, cancellationToken, nominationArc);
                        var discoveryCenters = preparedNeighborhood.SelectCenters(discoveryLegal,
                            radius + relevance + CoverageTolerance, cancellationToken);
                        var discoveryRegions = PolygonOps.DifferenceToComponents(discoveryCenters,
                            Array.Empty<IReadOnlyList<PointD>>()).Select(region => new MillingRegion(region.Outer,
                                region.Holes.Select(hole => (IReadOnlyList<PointD>)hole).ToList())).ToList();
                        long discoveryCollectorStarted = componentDepthDiagnostics is null ? 0 : Stopwatch.GetTimestamp();
                        var discoveryRings = MedialAxisGeometry.BuildConcentricFill(discoveryRegions, candidateRingOptions,
                            radius, preserveTerminalPaths: !ordinaryCandidates, cancellationToken: cancellationToken);
                        if (componentDepthDiagnostics is not null) componentDepthDiagnostics.Collector(cutDepth, Stopwatch.GetElapsedTime(discoveryCollectorStarted).TotalMilliseconds);
                        componentNominationRings += discoveryRings.Count;
                        componentNominationPoints += discoveryRings.Sum(ring => (long)ring.Points.Count);
                        foreach (OffsetRing ring in discoveryRings)
                        {
                            cancellationToken.ThrowIfCancellationRequested();
                            var points = MedialAxisToolpaths.MaterializeClearingRing(ring, -cutDepth);
                            if (points.Count < 2) continue;
                            componentDiscoveryPoints += points.Count;
                            discovery.Add(new TaggedToolpath(points, Array.Empty<ToolpathSpan>(), source[0].RegionIndex,
                                "clearing", schedule.FirstPassIndex + pass, detectCurves: false));
                            // Ordered merge applies the original cumulative budget.
                            // This local cap bounds speculative worker materialization.
                            if (componentDiscoveryPoints > pointBudget) break;
                        }
                        componentDiagnostics?.Add($"stage=smooth-terminal-discovery round={round} component={currentComponent} " +
                            $"relevance={relevance:R} radius={radius:R} rings={discoveryRings.Count} paths={discovery.Count} " +
                            $"points={discovery.Sum(path => (long)path.Points.Count)} componentDiscoveryPoints={componentDiscoveryPoints} " +
                            $"localOutputPoints={componentCorrection.Skip(firstRing).Sum(path => (long)path.Points.Count)} " +
                            $"pointBudget={pointBudget} expandedRingsScheduled=False fullCandidateFootprintCrop=True " +
                            "onlyProvedTangentRaysEnterOutput=True sharedCollector=True");
                    }
                    componentTerminalRequests.Add(new(firstRing, componentCorrection.Count,
                        error, radius, relevance, discovery));
                }
                componentDiagnostics?.Add($"stage=smooth-concentric event=complete round={round} component={currentComponent} " +
                    $"depth={cutDepth:R} floorCap={cap:R} radius={radius:R} pitch={pitch:R} rings={rings.Count} " +
                    $"centerRegions={regions.Count} elapsedMs={componentTimer.Elapsed.TotalMilliseconds:F1} " +
                    $"neighborhoodMs={neighborhoodMilliseconds:F1} centerSelectionMs={centerSelectionMilliseconds:F1} " +
                    $"cropMs={cropMilliseconds:F1} insetMs={insetMilliseconds:F1} " +
                    $"neighborhoodPrepareMs={preparedNeighborhood.PreparationMilliseconds:F1} " +
                    $"neighborhoodOffsetMs={preparedNeighborhood.OffsetMilliseconds:F1} " +
                    $"neighborhoodConvertMs={preparedNeighborhood.ConversionMilliseconds:F1} " +
                    $"neighborhoodQueries={preparedNeighborhood.Queries} neighborhoodPreparations=1 " +
                    $"neighborhoodSelectionQueries={preparedNeighborhood.SelectionQueries} " +
                    $"neighborhoodDisjointSkips={preparedNeighborhood.DisjointSkips} " +
                    $"neighborhoodContactProbes={preparedNeighborhood.ContactProbes} " +
                    $"neighborhoodSelectionMs={preparedNeighborhood.SelectionMilliseconds:F1} " +
                    $"materializationMs={materializationTimer.Elapsed.TotalMilliseconds:F1} " +
                    $"nominationArcTolerance={nominationArc:R} ordinaryCollector={ordinaryCandidates} " +
                    $"nominationRings={componentNominationRings} nominationPoints={componentNominationPoints} " +
                    "sharedCollector=True originalBoundaryProof=True");
                return new(componentCorrection, componentTerminalRequests, componentDiagnostics,
                    NominationRings: componentNominationRings, NominationPoints: componentNominationPoints,
                    NeighborhoodSelectionQueries: preparedNeighborhood.SelectionQueries,
                    NeighborhoodDisjointSkips: preparedNeighborhood.DisjointSkips,
                    NeighborhoodContactProbes: preparedNeighborhood.ContactProbes,
                    NeighborhoodSelectionMilliseconds: preparedNeighborhood.SelectionMilliseconds,
                    BoxNeighborhoodQueries: preparedNeighborhood.BoxQueries,
                    BoxNeighborhoodFallbackQueries: preparedNeighborhood.OriginalFallbackQueries,
                    BoxNeighborhoodMilliseconds: preparedNeighborhood.BoxMilliseconds,
                    DepthStatistics: componentDepthDiagnostics?.Final(cutDepth, componentNominationRings, componentNominationPoints,
                        componentCorrection.Sum(path => (long)path.Points.Count), materializationTimer.Elapsed.TotalMilliseconds,
                        cropMilliseconds, neighborhoodMilliseconds, preparedNeighborhood.PreparationMilliseconds,
                        preparedNeighborhood.OffsetMilliseconds, preparedNeighborhood.ConversionMilliseconds, centerSelectionMilliseconds));
                }
                catch (Exception error)
                {
                    return new(componentCorrection, componentTerminalRequests, componentDiagnostics,
                        System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(error));
                }
            }
            int completedComponents = 0;
            void MergeComponent(int component, CorrectionComponentOutput result)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (result.Diagnostics is not null)
                    foreach (string message in result.Diagnostics) diagnostic?.Invoke(message);
                result.Failure?.Throw();
                nominationRings += result.NominationRings;
                nominationPoints += result.NominationPoints;
                neighborhoodSelectionQueries += result.NeighborhoodSelectionQueries;
                neighborhoodDisjointSkips += result.NeighborhoodDisjointSkips;
                neighborhoodContactProbes += result.NeighborhoodContactProbes;
                neighborhoodSelectionMilliseconds += result.NeighborhoodSelectionMilliseconds;
                boxNeighborhoodQueries = checked(boxNeighborhoodQueries + result.BoxNeighborhoodQueries);
                boxNeighborhoodFallbackQueries = checked(boxNeighborhoodFallbackQueries + result.BoxNeighborhoodFallbackQueries);
                boxNeighborhoodMilliseconds += result.BoxNeighborhoodMilliseconds;
                int first = correction.Count;
                if (roundDepthDiagnostics is not null && result.DepthStatistics is not null)
                {
                    roundDepthDiagnostics.Merge(result.DepthStatistics);
                    correctionDepthRanges!.Add((first, first + result.Paths.Count, result.DepthStatistics.FinalBin));
                }
                foreach (var request in result.TerminalRequests)
                {
                    foreach (var path in request.Discovery)
                    {
                        discoveryPoints += path.Points.Count;
                        if (discoveryPoints > pointBudget)
                            throw new InvalidOperationException($"Smooth V-carving terminal endpoint discovery exceeds " +
                                $"its {pointBudget} candidate-point budget at round {round}, component {component}; " +
                                $"discovery points={discoveryPoints}, coverage remains unverified.");
                    }
                    terminalRequests.Add((first + request.First, first + request.Last,
                        request.Error, request.Radius, request.Reach, request.Discovery));
                }
                correction.AddRange(result.Paths);
                completedComponents++;
                progress?.Invoke(.2 + .75 * round / MaximumCorrectionRounds,
                    $"Clearing smooth channel correction {round + 1}: area {completedComponents}", true);
            }
            // At most one component result per worker is buffered. Geometry,
            // diagnostics and terminal ordinals are merged in original order.
            for (int first = 0; first < correctionComponents.Count; first += workers)
            {
                cancellationToken.ThrowIfCancellationRequested();
                int count = Math.Min(workers, correctionComponents.Count - first);
                if (workers == 1) { MergeComponent(first, BuildComponent(first)); continue; }
                var outputs = new CorrectionComponentOutput?[count];
                Parallel.For(0, count, new ParallelOptions
                {
                    MaxDegreeOfParallelism = workers,
                    CancellationToken = cancellationToken,
                }, slot =>
                {
                    using var geometryWorker = GeometryParallelism.EnterWorker();
                    outputs[slot] = BuildComponent(first + slot);
                });
                cancellationToken.ThrowIfCancellationRequested();
                for (int slot = 0; slot < count; slot++)
                {
                    MergeComponent(first + slot, outputs[slot]!);
                }
            }
            diagnostic?.Invoke($"stage=smooth-concentric event=workers-complete round={round} workers={workers} " +
                $"components={completedComponents} elapsedMs={correctionTimer.Elapsed.TotalMilliseconds:F1} " +
                $"ordinaryCollector={ordinaryCandidates} nominationRings={nominationRings} nominationPoints={nominationPoints} " +
                $"neighborhoodSelectionQueries={neighborhoodSelectionQueries} neighborhoodDisjointSkips={neighborhoodDisjointSkips} " +
                $"neighborhoodContactProbes={neighborhoodContactProbes} neighborhoodSelectionMs={neighborhoodSelectionMilliseconds:F1} " +
                $"boxNeighborhoodEnabled={boxNeighborhoodEligible} boxNeighborhoodRequested={useBoxNeighborhoods} boxNeighborhoodQueries={boxNeighborhoodQueries} " +
                $"boxNeighborhoodFallbackQueries={boxNeighborhoodFallbackQueries} boxNeighborhoodMs={boxNeighborhoodMilliseconds:F3} " +
                "mergeOrder=original-component-order floorQueries=caller-thread rounds=serial");
            diagnostic?.Invoke($"stage=smooth-crop event=summary round={round} " + correctionBoundary.DescribeQueries());
            diagnostic?.Invoke($"stage=conditioned-correction-source event=summary round={round} " + correctionBoundary.DescribeNominationQueries());
            if (correction.Count == 0)
                throw CoverageFailure($"Smooth V-carving found no reachable correction path for missed area={area:R}, " +
                    $"bounds={Bounds(missing)} at round={round}; inspect smooth-coverage diagnostics.", round, missing, significant);
            diagnostic?.Invoke($"stage=smooth-work event=complete work=correction-paths-{round} " +
                $"elapsedMs={correctionTimer.Elapsed.TotalMilliseconds:F1} paths={correction.Count}");
            var terminalCandidateOrdinals = new List<int>();
            var limitedCorrection = Timed($"correction-protection-{round}", () =>
            {
                var centers = wallGuard.SnapCorrectionCenters(correction);
                if (terminalRequests.Count == 0) return Protect(centers);
                // Experimental only: changing optional ordinals can change later
                // surface witness selection. The .001mm optional-depth bound and
                // configured budget gate do not prove equal final removed stock.
                // Shared leaves this opt-in disabled until final-union proof exists.
                // Tighter settings also retain the original candidate snapshot.
                double depthFidelityBudget = Math.Max(options.Tolerance * .1, 1e-6);
                bool earlyAvoidanceEnabled = !useMaterialLayerGoal && options.AvoidRedundantTerminalRays
                    && double.IsFinite(depthFidelityBudget) && depthFidelityBudget >= minimumCorrectionDepth;
                // Only mandatory, fully protected cutting visits can authorize an
                // experimental omission. Disabled runs preserve the original
                // query order and one round-wide protection budget below.
                var protectedMandatory = earlyAvoidanceEnabled ? Protect(centers)
                    : new List<TaggedToolpath>();
                var terminalCoverage = new MandatoryTerminalCoverage(profile, schedule.SurfaceZ,
                    earlyAvoidanceEnabled ? (int)Math.Min(MaximumTerminalDominanceChords, pointBudget / 2) : 0);
                if (earlyAvoidanceEnabled) terminalCoverage.Add(protectedMandatory, cancellationToken);
                int terminalCount = 0, terminalTargets = 0, duplicateTerminalCandidates = 0;
                long distantRayTests = 0;
                int distantSafeRays = 0, boundedDistantTargets = 0;
                int centerIndexes = 0;
                long indexedCenterPoints = 0;
                var centerQueries = diagnostic is null ? null : new PreparedPointIndex.QueryDiagnostics();
                foreach (var request in terminalRequests)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    var legalPoints = centers.Skip(request.First).Take(request.Last - request.First)
                        .SelectMany(path => path.Points).Where(wallGuard.IsLegalCommandedCenter)
                        .Select(point =>
                        {
                            Point3D command = CommandPoint(point, schedule.SurfaceZ);
                            return new Point3D(command.X, command.Y, point.Z);
                        }).Distinct().ToArray();
                    var discoveryCenters = wallGuard.SnapCorrectionCenters(request.Discovery)
                        .SelectMany(path => path.Points).Where(wallGuard.IsLegalCommandedCenter)
                        .Select(point =>
                        {
                            Point3D command = CommandPoint(point, schedule.SurfaceZ);
                            return new Point3D(command.X, command.Y, point.Z);
                        }).Concat(legalPoints).Distinct().ToArray();
                    if (legalPoints.Length == 0 && discoveryCenters.Length == 0) continue;
                    var legalIndex = new PreparedPointIndex(legalPoints, cancellationToken, centerQueries);
                    var discoveryIndex = request.Reach > request.Radius
                        ? new PreparedPointIndex(discoveryCenters, cancellationToken, centerQueries) : null;
                    centerIndexes += discoveryIndex is null ? 1 : 2;
                    indexedCenterPoints += legalPoints.Length + (discoveryIndex?.Count ?? 0);
                    IEnumerable<PointD> terminalFrontier = denseTerminalRecovery
                        ? ResidualFrontierTargets(request.Error, CoverageTolerance / 2, cancellationToken, pointBudget)
                        : ResidualRepresentativeTargets(request.Error, cancellationToken, pointBudget);
                    foreach (PointD targetPoint in terminalFrontier)
                    {
                        terminalTargets++;
                        foreach (Point3D candidateEnd in legalIndex.Nearest(targetPoint.x, targetPoint.y, 3,
                            cancellationToken, centerQueries))
                        {
                        Point3D end = candidateEnd;
                        Point3D command = CommandPoint(end, schedule.SurfaceZ);
                        double dx = targetPoint.x - command.X, dy = targetPoint.y - command.Y;
                        double distance = Math.Sqrt(dx * dx + dy * dy);
                        if (distance <= 1e-9) continue;
                        end = new Point3D(command.X, command.Y, end.Z);
                        foreach (double fraction in new[] { 1.0, .75, .5 })
                        {
                            // The surface endpoint should approach the measured
                            // residual. Limiting a ramp to one engagement radius
                            // can leave a real gap even when a longer, shallower
                            // ramp is wholly inside both cutting domains.
                            double length = Math.Min(2 * request.Radius + CoverageTolerance, distance) * fraction;
                            Point3D start = CommandPoint(new Point3D(command.X + dx * length / distance,
                                command.Y + dy * length / distance, 0), schedule.SurfaceZ);
                            Point3D commandedEnd = CommandPoint(end, schedule.SurfaceZ);
                            if (!seenTerminalRamps.Add((start, commandedEnd)))
                            { duplicateTerminalCandidates++; continue; }
                            if (earlyAvoidanceEnabled && terminalCoverage.Covers(start, commandedEnd,
                                source[0].RegionIndex, centers[request.First].DepthPassIndex)) continue;
                            // A rounded cutter's continuous ramp can reach material
                            // between discrete F3 depth levels that constant-depth
                            // rings cannot. Shorter alternatives avoid repeatedly
                            // clipping the same wall-unsafe middle of a long ray.
                            // Every candidate receives the complete physical proof.
                            terminalCandidateOrdinals.Add(centers.Count);
                            centers.Add(new TaggedToolpath(new[] { start, end }, Array.Empty<ToolpathSpan>(),
                                source[0].RegionIndex, "clearing", centers[request.First].DepthPassIndex, detectCurves: false));
                            terminalCount++;
                            if (terminalCount * 2L > pointBudget)
                                throw new InvalidOperationException($"Smooth V-carving terminal candidates exceed the {pointBudget} point budget " +
                                    $"at round {round}; coverage remains unverified.");
                        }
                        }
                        if (request.Reach <= request.Radius) continue;
                        // A rounded ramp can follow a wall tangentially for much
                        // farther than its final engagement radius. Search the
                        // expanded shared-ring centers for actual safe chords;
                        // Euclidean nearest centers need not admit a useful ray.
                        Point3D distantStart = CommandPoint(new Point3D(targetPoint.x, targetPoint.y, 0), schedule.SurfaceZ);
                        int safeForTarget = 0;
                        foreach (Point3D candidateEnd in discoveryIndex!.Nearest(targetPoint.x, targetPoint.y,
                            cancellationToken: cancellationToken, diagnostics: centerQueries))
                        {
                            if (distantRayTests >= pointBudget) { boundedDistantTargets++; break; }
                            cancellationToken.ThrowIfCancellationRequested();
                            distantRayTests++;
                            Point3D end = new(candidateEnd.X, candidateEnd.Y, candidateEnd.Z);
                            if (!wallGuard.IsLegalCommandedChord(distantStart, end)) continue;
                            Point3D commandedEnd = CommandPoint(end, schedule.SurfaceZ);
                            if (!seenTerminalRamps.Add((distantStart, commandedEnd)))
                            { duplicateTerminalCandidates++; continue; }
                            if (earlyAvoidanceEnabled && terminalCoverage.Covers(distantStart, commandedEnd,
                                source[0].RegionIndex, centers[request.First].DepthPassIndex))
                            { if (++safeForTarget == 3) break; continue; }
                            terminalCandidateOrdinals.Add(centers.Count);
                            centers.Add(new TaggedToolpath(new[] { distantStart, end }, Array.Empty<ToolpathSpan>(),
                                source[0].RegionIndex, "clearing", centers[request.First].DepthPassIndex, detectCurves: false));
                            terminalCount++; distantSafeRays++;
                            if (++safeForTarget == 3) break;
                        }
                    }
                }
                if (centerQueries is not null)
                    diagnostic?.Invoke($"stage=smooth-center-search event=summary round={round} " +
                        $"indexes={centerIndexes} indexedPoints={indexedCenterPoints} " + centerQueries.Describe());
                diagnostic?.Invoke($"stage=smooth-terminal-avoidance round={round} " + terminalCoverage.Describe() +
                    $" earlyAvoidanceEnabled={earlyAvoidanceEnabled} depthFidelityBudget={depthFidelityBudget:R} " +
                    $"finalCommandCoordinatesOptIn={options.AvoidRedundantTerminalRays} " +
                    $"maximumOptionalDepth={minimumCorrectionDepth:R} depthBudgetRule=max-configured-tolerance-times-one-tenth-or-0.000001 " +
                    " authority=protected-mandatory-correction-chords sameRegionAndPass=True " +
                    "proof=whole-stock-height-endpoint-volume-dominance " +
                    "featureStatus=experimental finalOptionalWitnessSelection=unverified " +
                    "stockDomain=at-or-below-surface optionalPathsNeverAuthorizeOmission=True " +
                    "unprovedOrOverBudgetCandidates=retained");
                diagnostic?.Invoke($"stage=smooth-terminal-ramps round={round} candidates={terminalCount} " +
                    $"targets={terminalTargets} samplingTier={(denseTerminalRecovery ? "dense-stubborn-frontier" : "coarse-ring-representatives")} " +
                    $"frontierSpacing={(denseTerminalRecovery ? CoverageTolerance / 2 : 0):R} engagedCentersPerTarget=3 " +
                    $"repeatedCommandsSkipped={duplicateTerminalCandidates} deduplicationScope=all-correction-rounds " +
                    $"distantRayTests={distantRayTests} distantSafeRays={distantSafeRays} boundedDistantTargets={boundedDistantTargets} " +
                    $"discoveryPoints={discoveryPoints} emittedDistantRayPoints={2L * distantSafeRays} expandedRingsScheduled=False " +
                    "finalRoundRelevance=engaged-radius-squared-over-gap-radius searchExhaustionMeans=inconclusive " +
                    "source=minimum-engagement-concentric-centers nearRayReachBound=twice-cutter-radius-plus-coverage " +
                    "rayLengthFractions=1,0.75,0.5 distantRayReach=expanded-ring-candidates " +
                    "sub-F3-depths-occur-only-on-continuous-commanded-ramps=True completePhysicalProofFollows=True");
                if (!earlyAvoidanceEnabled) return Protect(centers);
                var protectedTerminals = terminalCount == 0 ? new List<TaggedToolpath>()
                    : Protect(centers.GetRange(correction.Count, terminalCount));
                long protectedChords = protectedMandatory.Sum(path => (long)Math.Max(0, path.Points.Count - 1))
                    + protectedTerminals.Sum(path => (long)Math.Max(0, path.Points.Count - 1));
                // The original floor and wall proofs shared one round-wide point
                // budget. Both only append points, so the combined protected
                // chord count also bounds each preceding proof's work.
                if (protectedChords > pointBudget)
                    throw new InvalidOperationException($"Smooth V-carving correction protection exceeds the " +
                        $"{pointBudget} point budget at round {round}; protected chords={protectedChords}.");
                protectedMandatory.AddRange(protectedTerminals);
                return protectedMandatory;
            });
            if (limitedCorrection.Count != correction.Count + terminalCandidateOrdinals.Count)
                throw new InvalidOperationException("Smooth V-carving correction protection changed terminal source ordinals.");
            if (roundDepthDiagnostics is not null)
            {
                // Point counts are attributed to ORIGINAL constructor command
                // depth, not guessed from later protected/routed geometry.
                // Original protection raises depths; no expensive point/F3 scan
                // is needed to account its aligned path outputs.
                foreach (var range in correctionDepthRanges!)
                    for (int i = range.First; i < range.Last; i++)
                    {
                        if ((i & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                        roundDepthDiagnostics.Protected(range.Bin, limitedCorrection[i].Points.Count, terminal: false);
                    }
                for (int i = correction.Count; i < limitedCorrection.Count; i++)
                {
                    if ((i & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                    roundDepthDiagnostics.Protected(CorrectionDepthDiagnostics.BinFor(minimumCorrectionDepth), limitedCorrection[i].Points.Count, terminal: true);
                }
                roundDepthDiagnostics.Report(round, diagnostic!);
                cancellationToken.ThrowIfCancellationRequested();
            }
            foreach (int ordinal in terminalCandidateOrdinals)
                terminalPathIndices.Add(constrained.Count + ordinal);
            added += limitedCorrection.Count;
            constrained.AddRange(limitedCorrection);
            var correctionSweep = Timed($"correction-sweep-{round}", () => BuildLowerSweep(limitedCorrection));
            actual = Timed($"coverage-union-{round}", () => PolygonOps.Union(actual.Concat(correctionSweep)));
            // Additional paths never remove prior coverage, making the coverage
            // error monotone even when a local depth estimate proves conservative.
            if (area > previousArea + .00001)
                throw new InvalidOperationException($"Smooth V-carving coverage grew from {previousArea:R} to {area:R} despite additive correction.");
            previousArea = area;
            long cuttingPoints = constrained.Sum(path => (long)path.Points.Count);
            diagnostic?.Invoke($"stage=smooth-budget round={round} cuttingPoints={cuttingPoints} pointBudget={pointBudget} " +
                $"sourcePoints={sourcePointCount} nextCoverageMeasurementPending=True");
            if (cuttingPoints > pointBudget)
                throw CoverageFailure($"Smooth V-carving exceeds its source-scaled {pointBudget} cutting-point budget " +
                    $"with {cuttingPoints} points from {sourcePointCount} input points; " +
                    $"before-correction missed area={area:R}, bounds={Bounds(missing)}. The new correction has not yet been remeasured.",
                    round, missing, significant);
        }
        throw new InvalidOperationException("Unreachable smooth coverage state.");

        InvalidOperationException CoverageFailure(string message, int round,
            List<List<PointD>> missing, List<List<PointD>> significant)
        {
            var error = new InvalidOperationException(message);
            if (gpuFrontNominatedComponents != 0) error.Data["GpuVCarveFrontsUsed"] = true;
            // Lazy coordinate projections make this a reviewable failure witness
            // without file I/O, large log messages, or a second geometry allocation
            // in the application. Diagnostic clients may serialize the payload.
            static IEnumerable<IEnumerable<double[]>> Rings(IEnumerable<IReadOnlyList<PointD>> rings)
                => rings.Select(ring => ring.Select(point => new[] { point.x, point.y }));
            error.Data["SmoothVCarvingGeometry"] = new
            {
                basis = "float-workspace-F3-stock-relative", round, spatialTolerance = CompletionTolerance,
                physicalCommandSurfaceZ = 0d, goalSurfaceZ, materialLayerGoalEnabled = useMaterialLayerGoal,
                materialLayerDepthMm = useMaterialLayerGoal ? VCarveMaterialLayerGoal.LayerDepthMm : 0d,
                goalScope = "projected-original-upper-then-opening-then-separate-original-artwork-fringe",
                materialOmissionBoundScope = "outside-complete-projected-raw-reference-upper-only; no whole-goal/global-stock bound",
                coverageCertificate = latestCoverageCertificate is null ? null : new
                {
                    latestCoverageCertificate.Certified, latestCoverageCertificate.Reason,
                    latestCoverageCertificate.VisitedBoxes, latestCoverageCertificate.DiscEvaluations,
                    latestCoverageCertificate.RemainingBoxes, latestCoverageCertificate.ElapsedSeconds,
                    unresolvedX = latestCoverageCertificate.UnresolvedPoint?.x,
                    unresolvedY = latestCoverageCertificate.UnresolvedPoint?.y,
                },
                target = Rings(target), actual = Rings(actual), missing = Rings(missing), significant = Rings(significant),
                artwork = Rings(boundary), allowedChannel = Rings(originalTarget),
                commandPaths = constrained.Select(path => new
                {
                    category = path.Category,
                    points = path.Points.Select(point =>
                    {
                        Point3D command = CommandPoint(point, schedule.SurfaceZ);
                        return new[] { command.X, command.Y, command.Z };
                    }),
                }),
            };
            return error;
        }

        T Timed<T>(string work, Func<T> action)
        {
            cancellationToken.ThrowIfCancellationRequested();
            diagnostic?.Invoke($"stage=smooth-work event=begin work={work}");
            var timer = Stopwatch.StartNew();
            T result = action();
            diagnostic?.Invoke($"stage=smooth-work event=complete work={work} elapsedMs={timer.Elapsed.TotalMilliseconds:F1}");
            cancellationToken.ThrowIfCancellationRequested();
            return result;
        }
    }

    /// <summary>
    /// Decomposes oversized error regions before invoking the common concentric
    /// collector. The error remains a complete union of the resulting pieces;
    /// cutter-center neighborhoods overlap by a full radius across tile seams.
    /// Dense geometry may split below 32 mm, with a 4 mm floor to avoid creating
    /// thousands of microscopic tasks around detailed artwork.
    /// </summary>
    internal static List<PolygonComponent> PartitionCorrectionComponents(
        IReadOnlyList<List<PointD>> error, CancellationToken cancellationToken = default)
    {
        var output = new List<PolygonComponent>();
        foreach (var component in PolygonOps.DifferenceToComponents(error, Array.Empty<IReadOnlyList<PointD>>()))
        {
            cancellationToken.ThrowIfCancellationRequested();
            var source = new Paths64(new[] { component.Outer }.Concat(component.Holes).Select(ring =>
                new Path64(ring.Select(point => new Point64((long)Math.Round(point.x * PathUtils.Scale),
                    (long)Math.Round(point.y * PathUtils.Scale))))));
            Rect64 bounds = Clipper.GetBounds(source);
            long span = Math.Max(bounds.right - bounds.left, bounds.bottom - bounds.top);
            if (source.Sum(ring => ring.Count) <= 4096 && span <= 64 * PathUtils.Scale)
            {
                output.Add(component);
                continue;
            }
            Split(source, bounds);
        }
        return output;

        void Split(Paths64 source, Rect64 bounds)
        {
            cancellationToken.ThrowIfCancellationRequested();
            long width = bounds.right - bounds.left, height = bounds.bottom - bounds.top;
            long span = Math.Max(width, height);
            if (span <= 4 * PathUtils.Scale
                || (span <= 32 * PathUtils.Scale && source.Sum(ring => ring.Count) <= 4096))
            {
                var paths = source.Select(ring => ring.Select(point =>
                    new PointD(point.X / PathUtils.Scale, point.Y / PathUtils.Scale)).ToList()).ToList();
                output.AddRange(PolygonOps.DifferenceToComponents(paths, Array.Empty<IReadOnlyList<PointD>>()));
                return;
            }
            var first = new Rect64(bounds);
            var second = new Rect64(bounds);
            if (width >= height) first.right = second.left = bounds.left + width / 2;
            else first.bottom = second.top = bounds.top + height / 2;
            foreach (Rect64 rectangle in new[] { first, second })
            {
                var clipped = Clipper.Union(Clipper.RectClip(rectangle, source), FillRule.NonZero);
                if (clipped.Count > 0) Split(clipped, Clipper.GetBounds(clipped));
            }
        }
    }

    private sealed record CorrectionTerminalRequest(int First, int Last, List<List<PointD>> Error,
        double Radius, double Reach, List<TaggedToolpath> Discovery);
    private sealed record CorrectionComponentOutput(List<TaggedToolpath> Paths,
        List<CorrectionTerminalRequest> TerminalRequests, List<string>? Diagnostics,
        System.Runtime.ExceptionServices.ExceptionDispatchInfo? Failure = null,
        long NominationRings = 0, long NominationPoints = 0,
        long NeighborhoodSelectionQueries = 0, long NeighborhoodDisjointSkips = 0,
        long NeighborhoodContactProbes = 0, double NeighborhoodSelectionMilliseconds = 0,
        CorrectionDepthDiagnostics? DepthStatistics = null,
        long BoxNeighborhoodQueries = 0, long BoxNeighborhoodFallbackQueries = 0, double BoxNeighborhoodMilliseconds = 0);

    private sealed class CorrectionDepthDiagnostics
    {
        private struct Bin
        {
            internal long Components, CommandedAttempts, NominationRings, NominationPoints, MaterializedPoints,
                ProtectedPaths, ProtectedPoints, TerminalPaths, TerminalPoints;
            internal double InsetMs, CollectorMs, MaterializationMs;
            internal double CropMs, NeighborhoodMs, NeighborhoodPreparationMs, NeighborhoodOffsetMs, NeighborhoodConversionMs, CenterSelectionMs;
        }
        private readonly Bin[] _bins = new Bin[6];
        internal int FinalBin { get; private set; } = 5;
        internal static int BinFor(double depth) => !double.IsFinite(depth) || depth < 0 ? 5
            : depth <= .001 ? 0 : depth <= .01 ? 1 : depth <= .1 ? 2 : depth <= 1 ? 3 : 4;
        internal void CommandedAttempt(double depth) => _bins[BinFor(depth)].CommandedAttempts++;
        internal void Inset(double depth, double milliseconds) => _bins[BinFor(depth)].InsetMs += milliseconds;
        internal void Collector(double depth, double milliseconds) => _bins[BinFor(depth)].CollectorMs += milliseconds;
        internal CorrectionDepthDiagnostics Final(double depth, long rings, long nominationPoints, long materializedPoints, double milliseconds,
            double cropMilliseconds = 0, double neighborhoodMilliseconds = 0, double neighborhoodPreparationMilliseconds = 0,
            double neighborhoodOffsetMilliseconds = 0, double neighborhoodConversionMilliseconds = 0, double centerSelectionMilliseconds = 0)
        {
            FinalBin = BinFor(depth);ref var bin = ref _bins[FinalBin];bin.Components++;
            bin.NominationRings += rings;bin.NominationPoints += nominationPoints;
            bin.MaterializedPoints += materializedPoints;bin.MaterializationMs += milliseconds;
            bin.CropMs += cropMilliseconds;bin.NeighborhoodMs += neighborhoodMilliseconds;
            bin.NeighborhoodPreparationMs += neighborhoodPreparationMilliseconds;bin.NeighborhoodOffsetMs += neighborhoodOffsetMilliseconds;
            bin.NeighborhoodConversionMs += neighborhoodConversionMilliseconds;bin.CenterSelectionMs += centerSelectionMilliseconds;return this;
        }
        internal void Merge(CorrectionDepthDiagnostics source)
        {
            for (int i = 0; i < _bins.Length; i++)
            {
                ref var a = ref _bins[i];var b = source._bins[i];
                a.Components += b.Components;a.CommandedAttempts += b.CommandedAttempts;
                a.NominationRings += b.NominationRings;a.NominationPoints += b.NominationPoints;
                a.MaterializedPoints += b.MaterializedPoints;a.InsetMs += b.InsetMs;
                a.CollectorMs += b.CollectorMs;a.MaterializationMs += b.MaterializationMs;
                a.CropMs += b.CropMs;a.NeighborhoodMs += b.NeighborhoodMs;
                a.NeighborhoodPreparationMs += b.NeighborhoodPreparationMs;a.NeighborhoodOffsetMs += b.NeighborhoodOffsetMs;
                a.NeighborhoodConversionMs += b.NeighborhoodConversionMs;a.CenterSelectionMs += b.CenterSelectionMs;
            }
        }
        internal void Protected(int index, int points, bool terminal)
        {
            ref var bin = ref _bins[index];bin.ProtectedPaths++;bin.ProtectedPoints += points;
            if (terminal) { bin.TerminalPaths++;bin.TerminalPoints += points; }
        }
        internal void Report(int round, Action<string> diagnostic)
        {
            string[] names = { "depth<=0.001", "0.001<depth<=0.01", "0.01<depth<=0.1", "0.1<depth<=1", "depth>1", "unknown" };
            for (int i = 0; i < _bins.Length; i++)
            {
                var b = _bins[i];
                diagnostic($"stage=smooth-correction-depth event=complete round={round} bin={i} range='{names[i]}' components={b.Components} commandedAttempts={b.CommandedAttempts} nominationRings={b.NominationRings} nominationPoints={b.NominationPoints} materializedPoints={b.MaterializedPoints} insetMs={b.InsetMs:R} collectorMs={b.CollectorMs:R} materializationMs={b.MaterializationMs:R} cropMs={b.CropMs:R} neighborhoodMs={b.NeighborhoodMs:R} neighborhoodPrepareMs={b.NeighborhoodPreparationMs:R} neighborhoodOffsetMs={b.NeighborhoodOffsetMs:R} neighborhoodConvertMs={b.NeighborhoodConversionMs:R} centerSelectionMs={b.CenterSelectionMs:R} protectedPaths={b.ProtectedPaths} protectedPoints={b.ProtectedPoints} terminalPaths={b.TerminalPaths} terminalPoints={b.TerminalPoints} attribution=original-commanded-constructor-depth nominationIncludesDiscovery=True materializationIncludesDiscoveryCollector=True aggregateBins=6 finalPostedDepthMeasured=False constructorPhasesAttribution=final-commanded-depth neighborhoodMsScope=regular-SelectCenters centerSelectionMsScope=regular-DifferenceToComponents preparedOffsetConversionIncludeDiscovery=True nestedInclusiveWorkerTimeSums=True exclusiveWallTimeSum=False");
            }
        }
    }

    internal static int ResolveCorrectionWorkers(int? requested = null)
    {
        int limit = Math.Max(1, Math.Min(4, Environment.ProcessorCount));
        if (requested is null && int.TryParse(Environment.GetEnvironmentVariable("RW_TOOLPATHS_CORRECTION_WORKERS"),
                NumberStyles.Integer, CultureInfo.InvariantCulture, out int configured)) requested = configured;
        return requested is > 0 ? Math.Clamp(requested.Value, 1, limit) : limit;
    }

    /// <summary>
    /// Bounds expensive cutter-center construction to the source that can affect
    /// this error component. Every desired center is within radius+tolerance of
    /// the error bounds; another radius plus numerical reserve keeps artificial
    /// crop walls beyond its complete cutter disc and the inset's command guard.
    /// The final actual-command wall proof still uses the complete original source.
    /// </summary>
    internal sealed class CorrectionBoundary
    {
        private readonly Paths64 _source;
        private readonly List<List<PointD>> _all;
        private readonly Rect64 _bounds;
        private readonly Rect64[] _ringBounds;
        private readonly double _commandReserve;
        private readonly Dictionary<(double Radius, double ArcTolerance), List<List<PointD>>> _wholeSourceInsets = new();
        private readonly object _queryLock = new();
        internal int SourcePoints { get; }
        internal IReadOnlyList<IReadOnlyList<PointD>> OriginalBoundary => _all;
        private VCarveConditionedCorrectionSource? _conditionedNomination;
        private bool _nominationPreparationAttempted;
        private string _nominationReason = "disabled";
        internal bool UsesConditionedNomination => _conditionedNomination is not null;
        internal double NominationInwardPadMm => _conditionedNomination?.InwardPadMm ?? 0;
        internal long NominationQueries => _conditionedNomination?.Queries ?? 0;
        internal bool HasWholeNominationInsets => _conditionedNomination?.HasWholeSourceInsets ?? false;
        private long _queries, _submittedPoints, _excludedPoints;
        private double _cropMilliseconds;

        internal CorrectionBoundary(IEnumerable<IReadOnlyList<PointD>> boundary)
        {
            _source = Clipper.Union(new Paths64(boundary.Select(ring => new Path64(ring.Select(point =>
                new Point64((long)Math.Round(point.x * PathUtils.Scale),
                    (long)Math.Round(point.y * PathUtils.Scale)))))), FillRule.NonZero);
            _all = Convert(_source);
            _bounds = Clipper.GetBounds(_source);
            _ringBounds = _source.Select(ring => Clipper.GetBounds(ring)).ToArray();
            SourcePoints = _source.Sum(ring => ring.Count);
            double coordinate = Math.Max(Math.Max(Math.Abs((double)_bounds.left), Math.Abs((double)_bounds.right)),
                Math.Max(Math.Abs((double)_bounds.top), Math.Abs((double)_bounds.bottom))) / PathUtils.Scale;
            float command = (float)coordinate;
            _commandReserve = .01 + Math.Sqrt(2) * (MathF.BitIncrement(command) - command) / 2;
        }

        // One immutable NOM owner publishes before component workers start.
        // Original Crop/OriginalBoundary/cache remain their literal owners.
        internal void PrepareConditionedNomination(bool enabled, CancellationToken token, Action<string>? diagnostic)
        {
            token.ThrowIfCancellationRequested();
            if (_nominationPreparationAttempted) throw new InvalidOperationException("Conditioned NOM source preparation repeated.");
            _nominationPreparationAttempted = true;
            if (!enabled) return;
            VCarveConditionedCorrectionSource.TryCreate(_source, token, diagnostic, out var owner, out _nominationReason);
            token.ThrowIfCancellationRequested(); _conditionedNomination = owner;
            if (owner is null) diagnostic?.Invoke($"stage=conditioned-correction-source event=original-fallback reason={_nominationReason} "
                + "completeOriginalCropAndCache=True partialSourcePublished=False");
        }
        internal List<List<PointD>> CropNomination(IReadOnlyList<PointD> error, double radius,
            CancellationToken cancellationToken = default, double additionalCenterReach = 0)
            => _conditionedNomination is null ? Crop(error, radius, cancellationToken, additionalCenterReach)
                : _conditionedNomination.Crop(CropBounds(error, radius, additionalCenterReach), cancellationToken);
        internal bool TryGetWholeNominationInset(double radius, out List<List<PointD>> inset,
            double arcTolerance = ReferenceCorrectionArcTolerance)
        {
            if (_conditionedNomination is null) return TryGetWholeSourceInset(radius, out inset, arcTolerance);
            return _conditionedNomination.TryGetWholeSourceInset(radius, arcTolerance, out inset);
        }
        internal void RememberWholeNominationInset(List<List<PointD>> source, double radius, List<List<PointD>> inset,
            double arcTolerance = ReferenceCorrectionArcTolerance)
        {
            if (_conditionedNomination is null) RememberWholeSourceInset(source, radius, inset, arcTolerance);
            else _conditionedNomination.RememberWholeSourceInset(source, radius, arcTolerance, inset);
        }
        internal string DescribeNominationQueries() => _conditionedNomination?.Describe()
            ?? $"conditionedNomQueries=0 active=False reason={_nominationReason} completeOriginalCropAndCache=True";

        internal List<List<PointD>> Crop(IReadOnlyList<PointD> error, double radius,
            CancellationToken cancellationToken = default, double additionalCenterReach = 0)
        {
            cancellationToken.ThrowIfCancellationRequested();
            long started = Stopwatch.GetTimestamp();
            var crop = CropBounds(error, radius, additionalCenterReach);
            if (crop.left <= _bounds.left && crop.top <= _bounds.top
                && crop.right >= _bounds.right && crop.bottom >= _bounds.bottom)
            {
                RecordQuery(SourcePoints, started);
                return _all;
            }
            // RectClip64.Execute (pinned Clipper2 1.4.0) independently rejects each
            // path with this exact bounds predicate before clipping it. Cache only
            // those bounds, retain complete intersecting rings in their original
            // order, and pass them to the unchanged clip/NonZero union. Enclosing
            // rings and signed holes remain present; no segment pruning or second
            // clipping pass can change intersection rounding or topology.
            var selected = new Paths64();
            long selectedPoints = 0;
            for (int i = 0; i < _source.Count; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (!crop.Intersects(_ringBounds[i])) continue;
                selected.Add(_source[i]);
                selectedPoints += _source[i].Count;
            }
            var clipped = Clipper.Union(Clipper.RectClip(crop, selected), FillRule.NonZero);
            cancellationToken.ThrowIfCancellationRequested();
            var result = Convert(clipped);
            RecordQuery(selectedPoints, started);
            return result;
        }

        private Rect64 CropBounds(IReadOnlyList<PointD> error, double radius, double additionalCenterReach)
        {
            double padding = 2 * radius + additionalCenterReach + CoverageTolerance + _commandReserve;
            return new Rect64(
                (long)Math.Floor((error.Min(point => point.x) - padding) * PathUtils.Scale),
                (long)Math.Floor((error.Min(point => point.y) - padding) * PathUtils.Scale),
                (long)Math.Ceiling((error.Max(point => point.x) + padding) * PathUtils.Scale),
                (long)Math.Ceiling((error.Max(point => point.y) + padding) * PathUtils.Scale));
        }

        internal bool CouldCropWholeSource(IReadOnlyList<PointD> error, double radius, double additionalCenterReach)
        {
            var crop = CropBounds(error, radius, additionalCenterReach);
            return crop.left <= _bounds.left && crop.top <= _bounds.top
                && crop.right >= _bounds.right && crop.bottom >= _bounds.bottom;
        }

        internal bool HasWholeSourceInsets { get { lock (_queryLock) return _wholeSourceInsets.Count != 0; } }

        private void RecordQuery(long submitted, long started)
        {
            double milliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            lock (_queryLock)
            {
                _queries++;
                _submittedPoints += submitted;
                _excludedPoints += SourcePoints - submitted;
                _cropMilliseconds += milliseconds;
            }
        }

        internal string DescribeQueries()
        {
            lock (_queryLock) return $"queries={_queries} sourcePoints={SourcePoints} "
                + $"submittedPoints={_submittedPoints} excludedPoints={_excludedPoints} cropMs={_cropMilliseconds:F1} "
                + "boundsPolicy=clipper-1.4-exact orderedCompleteRings=True";
        }

        internal bool TryGetWholeSourceInset(double radius, out List<List<PointD>> inset,
            double arcTolerance = ReferenceCorrectionArcTolerance)
        {
            lock (_queryLock) return _wholeSourceInsets.TryGetValue((radius, arcTolerance), out inset!);
        }

        internal void RememberWholeSourceInset(List<List<PointD>> source, double radius, List<List<PointD>> inset,
            double arcTolerance = ReferenceCorrectionArcTolerance)
        {
            if (ReferenceEquals(source, _all))
                lock (_queryLock) _wholeSourceInsets[(radius, arcTolerance)] = inset;
        }

        private static List<List<PointD>> Convert(Paths64 paths) => paths.Select(ring => ring.Select(point =>
            new PointD(point.X / PathUtils.Scale, point.Y / PathUtils.Scale)).ToList()).ToList();
    }

    /// <summary>
    /// An outer approximation to the source's radius-r morphological opening.
    /// This is a coverage reference, never a cutter-center boundary. Machining
    /// front conditioning and command-clearance repair do not belong here: an
    /// unnecessarily smaller reference could conceal reachable missed material.
    /// </summary>
    internal static List<List<PointD>> ReachableReferenceUpper(
        IEnumerable<IReadOnlyList<PointD>> boundary, double radius, CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var source = new Paths64(boundary.Select(ring => new Path64(ring.Select(point =>
            new Point64((long)Math.Round(point.x * PathUtils.Scale),
                (long)Math.Round(point.y * PathUtils.Scale))))));
        source = Clipper.Union(source, FillRule.NonZero);
        // Each round offset has at most one grid unit of arc sagitta and
        // sqrt(2)/2 units of coordinate rounding. Eight units include the
        // source conversion and subsequent Boolean rounding as well. Reducing
        // erosion and increasing dilation gives a superset of exact reach.
        const double reserve = 8;
        if (radius * PathUtils.Scale > reserve)
        {
            var inset = new ClipperOffset(arcTolerance: 1);
            inset.AddPaths(source, JoinType.Round, EndType.Polygon);
            var centers = new Paths64();
            inset.Execute(-(radius * PathUtils.Scale - reserve), centers);
            cancellationToken.ThrowIfCancellationRequested();
            var outset = new ClipperOffset(arcTolerance: 1);
            outset.AddPaths(centers, JoinType.Round, EndType.Polygon);
            var opened = new Paths64();
            outset.Execute(radius * PathUtils.Scale + reserve, opened);
            cancellationToken.ThrowIfCancellationRequested();
            source = Clipper.Intersect(source, opened, FillRule.NonZero);
        }
        return source.Select(ring => ring.Select(point =>
            new PointD(point.X / PathUtils.Scale, point.Y / PathUtils.Scale)).ToList()).ToList();
    }

    internal static List<List<PointD>> ReachableReferenceUpperForPlane(
        IEnumerable<IReadOnlyList<PointD>> boundary, double radius, double dilationRadius, CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var source = new Paths64(boundary.Select(ring => new Path64(ring.Select(point =>
            new Point64((long)Math.Round(point.x * PathUtils.Scale),
                (long)Math.Round(point.y * PathUtils.Scale))))));
        source = Clipper.Union(source, FillRule.NonZero);
        // Each round offset has at most one grid unit of arc sagitta and
        // sqrt(2)/2 units of coordinate rounding. Eight units include the
        // source conversion and subsequent Boolean rounding as well. Reducing
        // erosion and increasing dilation gives a superset of exact reach.
        const double reserve = 8;
        if (radius * PathUtils.Scale > reserve)
        {
            var inset = new ClipperOffset(arcTolerance: 1);
            inset.AddPaths(source, JoinType.Round, EndType.Polygon);
            var centers = new Paths64();
            inset.Execute(-(radius * PathUtils.Scale - reserve), centers);
            cancellationToken.ThrowIfCancellationRequested();
            var outset = new ClipperOffset(arcTolerance: 1);
            outset.AddPaths(centers, JoinType.Round, EndType.Polygon);
            var opened = new Paths64();
            outset.Execute(dilationRadius * PathUtils.Scale + reserve, opened);
            cancellationToken.ThrowIfCancellationRequested();
            source = Clipper.Intersect(source, opened, FillRule.NonZero);
        }
        return source.Select(ring => ring.Select(point =>
            new PointD(point.X / PathUtils.Scale, point.Y / PathUtils.Scale)).ToList()).ToList();
    }

    internal static List<List<PointD>> CorrectionNeighborhood(
        IEnumerable<IReadOnlyList<PointD>> error, double reach, CancellationToken cancellationToken = default)
        => new PreparedCorrectionNeighborhood(error, cancellationToken).Expand(reach, cancellationToken);

    /// <summary>One component's canonical, ordered integer offset input. Repeated
    /// radii use the same raw joined offset and reserves as independent queries.</summary>
    internal sealed class PreparedCorrectionNeighborhood
    {
        private readonly ClipperOffset _offset = new(arcTolerance: 1);
        private readonly Paths64 _source;
        private readonly Rect64 _bounds;
        private readonly Lazy<PolygonOps.BoundarySegmentIndex> _boundary;
        internal double PreparationMilliseconds { get; }
        internal double OffsetMilliseconds { get; private set; }
        internal double ConversionMilliseconds { get; private set; }
        internal int Queries { get; private set; }
        internal int SelectionQueries { get; private set; }
        internal int DisjointSkips { get; private set; }
        internal long ContactProbes { get; private set; }
        internal double SelectionMilliseconds { get; private set; }

        internal PreparedCorrectionNeighborhood(IEnumerable<IReadOnlyList<PointD>> error,
            CancellationToken cancellationToken = default)
        {
            cancellationToken.ThrowIfCancellationRequested();
            long started = Stopwatch.GetTimestamp();
            _source = Clipper.Union(new Paths64(error.Select(ring => new Path64(ring.Select(point =>
                new Point64((long)Math.Round(point.x * PathUtils.Scale),
                    (long)Math.Round(point.y * PathUtils.Scale)))))), FillRule.NonZero);
            _bounds = Clipper.GetBounds(_source);
            _boundary = new Lazy<PolygonOps.BoundarySegmentIndex>(() => new(_source));
            _offset.AddPaths(_source, JoinType.Round, EndType.Polygon);
            cancellationToken.ThrowIfCancellationRequested();
            PreparationMilliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        }

        /// <summary>
        /// Rejects a disjoint nomination before constructing its dense round offset.
        /// Possible contact always uses the original offset and intersection. This
        /// predicate grants no stock credit: actual wall/floor guards and the fixed
        /// channel coverage proof still own every emitted correction.
        /// </summary>
        internal List<List<PointD>> SelectCenters(IReadOnlyList<List<PointD>> legal,
            double reach, CancellationToken cancellationToken = default)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (!double.IsFinite(reach) || reach < 0)
                throw new ArgumentOutOfRangeException(nameof(reach));
            SelectionQueries++;
            long started = Stopwatch.GetTimestamp();
            bool possible = MayIntersect(legal, reach, cancellationToken);
            SelectionMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            if (!possible)
            {
                DisjointSkips++;
                return new List<List<PointD>>();
            }
            return PolygonOps.Intersect(legal, Expand(reach, cancellationToken));
        }

        internal bool MayIntersect(IReadOnlyList<List<PointD>> legal, double reach,
            CancellationToken cancellationToken)
        {
            if (legal.Count == 0 || _source.Count == 0) return false;
            // Expand uses reach plus four integer units. Eight units additionally
            // enclose its vertex/internal-Boolean rounding and the final Boolean's
            // rounding. The capsule query itself reserves one more integer unit.
            // Touching or uncertain contact retains the complete original path.
            double guardedReach = reach * PathUtils.Scale + 8;
            if (!double.IsFinite(guardedReach) || guardedReach > 1e12) return true;
            var integer = PathUtils.ToClipper(legal);
            Rect64 legalBounds = Clipper.GetBounds(integer);
            // Capsule arithmetic uses binary64. Outside this bounded integer
            // domain retain the original Boolean rather than claim an eight-tick
            // enclosure for unbounded coordinate-rounding error.
            static bool Bounded(Rect64 bounds) => Math.Abs((double)bounds.left) <= 1e12
                && Math.Abs((double)bounds.right) <= 1e12 && Math.Abs((double)bounds.top) <= 1e12
                && Math.Abs((double)bounds.bottom) <= 1e12;
            if (!Bounded(_bounds) || !Bounded(legalBounds)) return true;
            if ((double)legalBounds.right < _bounds.left - guardedReach
                || legalBounds.left > _bounds.right + guardedReach
                || (double)legalBounds.bottom < _bounds.top - guardedReach
                || legalBounds.top > _bounds.bottom + guardedReach) return false;
            var boundary = _boundary.Value;
            foreach (var ring in integer)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (ring.Count < 3) return true;
                if (boundary.IsInside(ring[0]) || boundary.DistanceSquared(ring[0]) <= 16)
                    return true;
                for (int end = 0, start = ring.Count - 1; end < ring.Count; start = end++)
                {
                    ContactProbes++;
                    if (boundary.ContactIntervals(ring[start], ring[end], guardedReach,
                        cancellationToken, out _).Count != 0) return true;
                }
            }
            // Boundary separation alone misses an error component wholly enclosed
            // by a legal center region. Retain that case (and touching ambiguity).
            var legalBoundary = new PolygonOps.BoundarySegmentIndex(integer);
            foreach (var ring in _source)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (ring.Count == 0) continue;
                if (legalBoundary.IsInside(ring[0]) || legalBoundary.DistanceSquared(ring[0]) <= 16)
                    return true;
            }
            return false;
        }

        internal List<List<PointD>> Expand(double reach, CancellationToken cancellationToken = default)
        {
            cancellationToken.ThrowIfCancellationRequested();
            // This dilation selects candidate centers, not a machining boundary.
            // A machining offset validator can reject a valid expanded contour merely
            // for the inward sagitta of its round joins, discarding the whole local
            // correction. Keep the complete expansion and let the original-domain
            // cutter guards and unchanged coverage measurement certify its cuts.
            long started = Stopwatch.GetTimestamp();
            var expanded = new Paths64();
            _offset.Execute(reach * PathUtils.Scale + 4, expanded);
            OffsetMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            cancellationToken.ThrowIfCancellationRequested();
            started = Stopwatch.GetTimestamp();
            var result = expanded.Select(ring => ring.Select(point =>
                new PointD(point.X / PathUtils.Scale, point.Y / PathUtils.Scale)).ToList()).ToList();
            ConversionMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            Queries++;
            return result;
        }
    }

    internal static List<List<PointD>> ResidualCentersUpper(
        IEnumerable<IReadOnlyList<PointD>> missing, double radius, CancellationToken cancellationToken = default)
        // This is a measurement, so preserve an OUTER bound of the exact erosion.
        // Four grid units cover the one-unit arc sagitta and coordinate/Boolean
        // roundings. Machining contour validation can discard valid narrow lobes
        // and must never decide whether uncut material meets the width criterion.
        => RawInset(missing, Math.Max(0, radius - 4 / PathUtils.Scale), cancellationToken);

    internal static double TerminalRelevanceReach(double engagedRadius)
        // At a rounded tip, sagitta scales as radial distance squared over
        // axial depth. Near a wall, the ramp length needed to keep a real disc
        // within a small spatial clearance similarly scales as r²/clearance.
        // This resolution-derived relevance window is a bounded constructive
        // search, never a declaration that farther cutting centers cannot help.
        => Math.Max(engagedRadius, engagedRadius * engagedRadius / (CoverageTolerance / 2));

    /// <summary>
    /// A bounded sufficient full-stock proof for optional surface-to-tip rays.
    /// A retained mandatory endpoint supplies a sufficient full-volume proof:
    /// if r'(h)>=k through depth D, ray length L<=k*D and end depth d<=D,
    /// then at every candidate pose t and stock height z its axis separation is
    /// L*(1-t)<=k*(D-d*t)<=r(D+z)-r(d*t+z). Every candidate disc is enclosed by the
    /// mandatory endpoint disc at that same stock height. Inconclusive cases
    /// retain the ordinary candidate.
    /// </summary>
    internal sealed class MandatoryTerminalCoverage
    {
        private readonly record struct Site(long X, long Y, long Z);
        private readonly record struct EndpointKey(int Region, int Pass, long X, long Y);
        private readonly Dictionary<EndpointKey, (long Z, double Reach)> _endpoints = new();
        private readonly Dictionary<long, double> _depthReaches = new();
        private readonly VCarveCutterProfile _profile;
        private readonly double _surface;

        private readonly int _maximumExamined;
        private readonly bool _monotoneProfile;
        private long _examined, _queries, _coveredEndpoint, _unrepresentable, _withoutXyzArrival;
        private bool _bounded;
        private double _preparationMilliseconds;

        internal MandatoryTerminalCoverage(VCarveCutterProfile profile, double surface,
            int maximumExaminedChords = MaximumTerminalDominanceChords)
        {
            _profile = profile; _surface = surface;
            _maximumExamined = Math.Max(0, maximumExaminedChords);
            double height = 0, previousRadius = profile.Sections[0].BottomRadius;
            bool monotone = double.IsFinite(surface) && double.IsFinite(profile.MaximumDepth);
            foreach (var section in profile.Sections)
            {
                if (height >= profile.MaximumDepth) break;
                monotone &= section.BottomRadius == previousRadius && section.TopRadius >= section.BottomRadius
                    && (section.Kind != CarvingCutterSectionKind.Hemisphere || section.Height <= section.TopRadius);
                height += section.Height;
                previousRadius = section.TopRadius;
            }
            _monotoneProfile = monotone && height >= profile.MaximumDepth;
        }

        // The caller supplies protected mandatory visits, before optional paths
        // are appended. Neither an optional ray nor a raw pre-protection contour
        // can enter this proof snapshot.
        internal void Add(IReadOnlyList<TaggedToolpath> protectedMandatory, CancellationToken cancellationToken = default)
        {
            long started = Stopwatch.GetTimestamp();
            try
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (!_monotoneProfile) return;
                foreach (var path in protectedMandatory)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    if (path.Category != "clearing" || path.DepthPassIndex is not int pass || path.Points.Count < 2) continue;
                    Point3D a = CommandPoint(path.Points[0], _surface);
                    for (int i = 1; i < path.Points.Count; i++)
                    {
                        if (_examined >= _maximumExamined) { _bounded = true; return; }
                        if ((++_examined & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                        Point3D b = CommandPoint(path.Points[i], _surface);
                        if (a != b && (path.Spans.Count == 0 || path.Spans.Any(span => span.StartIndex < i
                            && span.EndIndex >= i && span.Kind != ToolpathSpanKind.Retract))
                            && a.Z <= 0 && b.Z <= 0
                            && -Math.Min(a.Z, b.Z) <= _profile.MaximumDepth)
                        {
                            // Shared classifies raw binary32 WORLD chords before
                            // F3. A Y-only arrival never establishes the target
                            // XY cursor. Only the destination of a proven XYZ
                            // arrival can authorize a removed cutter volume;
                            // an open path's first point remains unproved here.
                            if (!HasXyzArrival(path.Points[i - 1], path.Points[i], _surface))
                                _withoutXyzArrival++;
                            else if (TrySite(b, out Site last))
                            {
                                InsertEndpoint(last, path.RegionIndex, pass);
                            }
                            else _unrepresentable++;
                        }
                        a = b;
                    }
                }
            }
            finally { _preparationMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds; }
        }

        private static bool HasXyzArrival(Point3D from, Point3D to, double surface)
        {
            float ax = (float)from.X, ay = (float)from.Y, az = (float)(surface + from.Z);
            float bx = (float)to.X, by = (float)to.Y, bz = (float)(surface + to.Z);
            if (!float.IsFinite(ax) || !float.IsFinite(ay) || !float.IsFinite(az)
                || !float.IsFinite(bx) || !float.IsFinite(by) || !float.IsFinite(bz)) return false;
            float dx = bx - ax, dy = by - ay, dz = bz - az;
            float run = MathF.Sqrt(dx * dx + dy * dy);
            return float.IsFinite(dx) && float.IsFinite(dy) && float.IsFinite(dz) && float.IsFinite(run)
                && (run > .001f || MathF.Abs(dz) <= .001f);
        }

        // start/end are the actual F3 command, not nominal workspace points.
        internal bool Covers(Point3D start, Point3D end, int region, int? pass)
        {
            _queries++;
            if (!_monotoneProfile || pass is not int passIndex || start.Z != 0 || end.Z >= 0
                || -end.Z > _profile.MaximumDepth) return false;
            if (!TrySite(start, out Site first) || !TrySite(end, out Site last))
            { _unrepresentable++; return false; }
            if (_endpoints.TryGetValue(new(region, passIndex, last.X, last.Y), out var endpoint)
                && endpoint.Z <= last.Z)
            {
                double dx = (first.X - last.X) / 1000.0, dy = (first.Y - last.Y) / 1000.0;
                double length = Math.Sqrt(dx * dx + dy * dy);
                if (length + VCarveCoverageCertificate.ArithmeticReserve + 1e-12 * length <= endpoint.Reach)
                { _coveredEndpoint++; return true; }
            }
            return false;
        }

        private void InsertEndpoint(Site site, int region, int pass)
        {
            if (site.Z >= 0) return;
            if (!_depthReaches.TryGetValue(site.Z, out double reach))
            {
                double depth = -site.Z / 1000.0, remaining = depth, slope = double.PositiveInfinity;
                foreach (var section in _profile.Sections)
                {
                    if (remaining <= 0) break;
                    double active = Math.Min(remaining, section.Height);
                    // r'(h) decreases throughout a hemisphere. Enlarging its
                    // denominator from sqrt(h*(2R-h)) to sqrt(2R*h) supplies a
                    // lower slope bound, including the infinite tip derivative.
                    double lowerSlope = section.Kind == CarvingCutterSectionKind.Hemisphere
                        ? (section.TopRadius - active) / (Math.Sqrt(active) * Math.Sqrt(2 * section.TopRadius))
                        : (section.TopRadius - section.BottomRadius) / section.Height;
                    slope = Math.Min(slope, lowerSlope);
                    remaining -= active;
                }
                reach = depth * slope;
                reach = remaining <= 0 && double.IsFinite(reach) && reach > 0
                    ? reach * (1 - 1e-12) - 2 * VCarveCoverageCertificate.ArithmeticReserve : 0;
                _depthReaches[site.Z] = reach;
            }
            if (reach <= 0) return;
            var key = new EndpointKey(region, pass, site.X, site.Y);
            if (!_endpoints.TryGetValue(key, out var previous)
                || site.Z <= previous.Z && reach >= previous.Reach) _endpoints[key] = (site.Z, reach);
        }

        private static bool TrySite(Point3D point, out Site site)
        {
            static bool Milli(double coordinate, out long integer)
            {
                integer = 0;
                // This explicit bound keeps exact subtraction and squared
                // distance finite. The divided-integer equality rejects non-F3
                // callers and numerical ambiguity without a spatial tolerance.
                if (!double.IsFinite(coordinate) || Math.Abs(coordinate) > 1_000_000_000) return false;
                integer = (long)Math.Round(coordinate * 1000);
                return integer / 1000.0 == coordinate;
            }
            if (Milli(point.X, out long x) && Milli(point.Y, out long y) && Milli(point.Z, out long z))
            { site = new(x, y, z); return true; }
            site = default;
            return false;
        }

        internal string Describe()
            => $"examinedChords={_examined} maximumExaminedChords={_maximumExamined} "
                + $"indexedEndpoints={_endpoints.Count} candidateQueries={_queries} "
                + $"omittedCandidates={_coveredEndpoint} "
                + $"endpointVolumeDominated={_coveredEndpoint} unrepresentable={_unrepresentable} "
                + $"endpointsWithoutXyzArrival={_withoutXyzArrival} authorityEndpoint=xyz-arrival-destination "
                + $"preparationMs={_preparationMilliseconds:F3} indexBounded={_bounded} monotoneProfile={_monotoneProfile}";
    }

    internal static List<TaggedToolpath> PruneTerminalPaths(IReadOnlyList<TaggedToolpath> paths,
        IReadOnlySet<int> terminalPathIndices, VCarveCoverageCertificate.Result certificate,
        Action<string>? diagnostic = null)
    {
        // A partial certificate has no removal authority. Neither a clearing
        // category nor a shallow depth establishes terminal provenance.
        if (!certificate.Certified || !certificate.WitnessPathsRecorded)
        {
            diagnostic?.Invoke($"stage=smooth-terminal-pruning event=skipped certified={certificate.Certified} " +
                $"witnessPathsRecorded={certificate.WitnessPathsRecorded} reason={certificate.Reason} removed=0");
            return paths.ToList();
        }
        if (certificate.CommandPathCount != paths.Count
            || terminalPathIndices.Any(index => index < 0 || index >= paths.Count)
            || certificate.WitnessPathIndices.Any(index => index < 0 || index >= paths.Count))
            throw new InvalidOperationException("Smooth V-carving witness pruning source path ordinals do not match the certified snapshot.");
        var witnesses = certificate.WitnessPathIndices.ToHashSet();
        var retained = paths.Where((path, index) => !terminalPathIndices.Contains(index) || witnesses.Contains(index)).ToList();
        int removed = paths.Count - retained.Count;
        diagnostic?.Invoke($"stage=smooth-terminal-pruning event=complete sourcePaths={paths.Count} " +
            $"terminalCandidates={terminalPathIndices.Count} terminalRetained={terminalPathIndices.Count - removed} " +
            $"terminalRemoved={removed} nonTerminalRetained={paths.Count - terminalPathIndices.Count} " +
            $"proofWitnessPaths={witnesses.Count} retainedPaths={retained.Count} " +
            "provenance=explicit-input-path-ordinals wholeTargetCertified=True allBoxWitnessPathsPreserved=True " +
            "residualAreaMeasurement=before-pruning");
        return retained;
    }

    internal static IEnumerable<PointD> ResidualFrontierTargets(IEnumerable<IReadOnlyList<PointD>> rings,
        double spacing, CancellationToken cancellationToken = default, long maximumSamples = MaximumPoints)
    {
        long samples = 0;
        foreach (var ring in rings)
        {
            if (ring.Count == 0) continue;
            yield return PolygonOps.Centroid(ring);
            double carry = 0;
            for (int i = 0; i < ring.Count; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                PointD a = ring[i], b = ring[(i + 1) % ring.Count];
                // Retain exact corners and extrema alongside the evenly spaced
                // frontier. F3 snapping changes both the nearest legal center
                // and the terminal ray, so a nearby sample does not subsume an
                // existing corner witness, even on a sub-spacing residual lobe.
                if (++samples > maximumSamples)
                    throw new InvalidOperationException($"Smooth V-carving residual frontier exceeds its {maximumSamples} target budget.");
                yield return a;
                double dx = b.x - a.x, dy = b.y - a.y, length = Math.Sqrt(dx * dx + dy * dy);
                if (length <= 1e-12) continue;
                double travelled = 0;
                while (carry + length - travelled >= spacing)
                {
                    travelled += spacing - carry;
                    carry = 0;
                    if (++samples > maximumSamples)
                        throw new InvalidOperationException($"Smooth V-carving residual frontier exceeds its {maximumSamples} target budget.");
                    if ((samples & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                    yield return new PointD(a.x + dx * travelled / length, a.y + dy * travelled / length);
                }
                carry += length - travelled;
            }
        }
    }

    internal static IEnumerable<PointD> ResidualRepresentativeTargets(IEnumerable<IReadOnlyList<PointD>> rings,
        CancellationToken cancellationToken = default, long maximumSamples = MaximumPoints)
    {
        long samples = 0;
        foreach (var ring in rings)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (ring.Count == 0) continue;
            PointD minX = ring[0], maxX = minX, minY = minX, maxY = minX;
            for (int i = 1; i < ring.Count; i++)
            {
                if ((i & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                PointD point = ring[i];
                if (point.x < minX.x) minX = point;
                if (point.x > maxX.x) maxX = point;
                if (point.y < minY.y) minY = point;
                if (point.y > maxY.y) maxY = point;
            }
            // This is an inexpensive construction tier, never a coverage test.
            // Concave lobes and holes that need more visits remain in the fixed
            // residual and receive every vertex plus a 4um frontier late.
            foreach (PointD point in new[] { PolygonOps.Centroid(ring), minX, maxX, minY, maxY }.Distinct())
            {
                if (++samples > maximumSamples)
                    throw new InvalidOperationException($"Smooth V-carving residual representatives exceed its {maximumSamples} target budget.");
                yield return point;
            }
        }
    }

    internal static List<List<PointD>> CorrectionCenters(
        IEnumerable<IReadOnlyList<PointD>> boundary, double radius, CancellationToken cancellationToken = default,
        double arcTolerance = ReferenceCorrectionArcTolerance)
    {
        // Candidate geometry must not lose a narrow but feasible center trace to
        // the ordinary cutter inset's blanket float/F3 reserve. Every generated
        // command will instead receive the exact dual-domain physical chord proof.
        // Arc tolerance changes nomination only; the immutable original target,
        // wall/channel/floor guards and complete coverage proof remain authority.
        ValidateCorrectionArcTolerance(arcTolerance);
        return RawInset(boundary, radius + 2 / PathUtils.Scale, cancellationToken, arcTolerance);
    }

    private static List<List<PointD>> RawInset(IEnumerable<IReadOnlyList<PointD>> rings, double radius,
        CancellationToken cancellationToken, double arcTolerance = ReferenceCorrectionArcTolerance)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var source = Clipper.Union(new Paths64(rings.Select(ring => new Path64(ring.Select(point =>
            new Point64((long)Math.Round(point.x * PathUtils.Scale),
                (long)Math.Round(point.y * PathUtils.Scale)))))), FillRule.NonZero);
        var offset = new ClipperOffset(arcTolerance: arcTolerance * PathUtils.Scale);
        offset.AddPaths(source, JoinType.Round, EndType.Polygon);
        var inset = new Paths64();
        offset.Execute(-radius * PathUtils.Scale, inset);
        cancellationToken.ThrowIfCancellationRequested();
        return inset.Select(ring => ring.Select(point =>
            new PointD(point.X / PathUtils.Scale, point.Y / PathUtils.Scale)).ToList()).ToList();
    }

    // This same original physical field is queried afresh on the complete
    // ACTUAL chord. Nominal CappedDepth preparation reserve is not reapplied;
    // no upward nominal proof transfers by merely rounding to the same F3.
    private static ExactF3TrajectorySimplifier.GuardDecision ProveActualF3CompactionChord(TaggedToolpath actual,
        ActualF3CompactionFloorProof? originalFloor, CommandClearance originalArtwork, CommandClearance originalBothWalls,
        double grade, double surface, long maximumPoints, CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();var a = actual.Points[0];var b = actual.Points[1];
        bool posting = CommandPoint(a, surface) == a && CommandPoint(b, surface) == b;
        bool floor = true;
        if (originalFloor is not null) floor = originalFloor.Prove(a, b);
        bool artwork = (a.Z >= 0 || originalArtwork.IsLegalCommandedCenter(a))
            && (b.Z >= 0 || originalArtwork.IsLegalCommandedCenter(b)) && originalArtwork.IsLegalCommandedChord(a, b);
        bool channel = (a.Z >= 0 || originalBothWalls.IsLegalCommandedCenter(a))
            && (b.Z >= 0 || originalBothWalls.IsLegalCommandedCenter(b)) && originalBothWalls.IsLegalCommandedChord(a, b);
        bool authoredGrade = MaximumGrade(new[] { actual }) <= grade + 1e-12;
        cancellationToken.ThrowIfCancellationRequested();
        return new(floor, artwork, channel, authoredGrade, posting,
            !posting ? "actual-F3-endpoints" : !floor ? "original-whole-actual-floor" : !artwork ? "original-artwork-chord"
            : !channel ? "original-channel-chord" : !authoredGrade ? "authored-grade" : "all-original-owners-pass");
    }
    private sealed class ActualF3CompactionFloorProof
    {
        private readonly FloorField originalFloor;
        private readonly double grade;
        private readonly CancellationToken token;
        internal ActualF3CompactionFloorProof(FloorField originalFloor, double grade, CancellationToken token)
        { this.originalFloor = originalFloor; this.grade = grade; this.token = token; }
        private const int MaximumDepth = 28;
        private const long MaximumPiecesPerChord = 100_000;
        private long _queries, _pieces, _endpointRefusals, _unresolvedRefusals, _arithmeticRefusals, _enclosureOperations, _originalQueryBudgetRefusals;
        private string? _firstOriginalQueryBudgetRefusal;
        private double _milliseconds;
        internal string Diagnostics() => $"stage=actual-F3-compaction event=original-actual-floor queries={_queries} pieces={_pieces} endpointRefusals={_endpointRefusals} unresolvedRefusals={_unresolvedRefusals} arithmeticRefusals={_arithmeticRefusals} originalQueryBudgetRefusals={_originalQueryBudgetRefusals} firstOriginalQueryBudgetRefusal='{_firstOriginalQueryBudgetRefusal}' enclosureOperations={_enclosureOperations} totalMs={_milliseconds:R} maxDepth={MaximumDepth} maxPiecesPerChord={MaximumPiecesPerChord} originalFieldAndCurvatureBound=True nominalCapReserveTransferred=False";
        internal bool Prove(Point3D originalA, Point3D originalB)
        {
            if (originalFloor.ProveCrestChord(originalA, originalB)) return true;
            token.ThrowIfCancellationRequested();_queries++;long started = Stopwatch.GetTimestamp(), visits = 0;
            try
            {
                if (!double.IsFinite(grade) || grade < 0 || !double.IsFinite(originalA.X + originalA.Y + originalA.Z
                    + originalB.X + originalB.Y + originalB.Z)) { _arithmeticRefusals++; return false; }
                double wholeLengthUpper = Up(Up(Math.Abs(originalB.X - originalA.X)) + Up(Math.Abs(originalB.Y - originalA.Y)));
                return Span(0, 1, 0);
                bool Span(double lo, double hi, int depth)
                {
                    token.ThrowIfCancellationRequested();_pieces++;
                    if (++visits > MaximumPiecesPerChord) { _unresolvedRefusals++; return false; }
                    var a = At(lo);var b = At(hi);
                    if (!double.IsFinite(a.FloorLower) || !double.IsFinite(a.DepthUpper) || !double.IsFinite(a.DepthLower)
                        || !double.IsFinite(b.FloorLower) || !double.IsFinite(b.DepthUpper) || !double.IsFinite(b.DepthLower))
                    { _arithmeticRefusals++; return false; }
                    double margin = Math.Min(Down(a.FloorLower - a.DepthUpper), Down(b.FloorLower - b.DepthUpper));
                    if (!double.IsFinite(margin)) { _arithmeticRefusals++; return false; }
                    if (margin < 0) { _endpointRefusals++; return false; }
                    double length = Up(wholeLengthUpper * Up(hi - lo));
                    double depthDelta = Up(Math.Max(Math.Abs(b.DepthUpper - a.DepthLower), Math.Abs(a.DepthUpper - b.DepthLower)));
                    double lipschitzCost = Up(Up(Up(grade * length) + depthDelta) * .5);
                    double curvatureCost = Up(Up(Up(originalFloor.CurvatureBound) * Up(length * length)) * .125);
                    if (!(length >= 0) || !double.IsFinite(length) || !(depthDelta >= 0) || !double.IsFinite(depthDelta)
                        || !(lipschitzCost >= 0) || !double.IsFinite(lipschitzCost)) { _arithmeticRefusals++; return false; }
                    if (margin >= lipschitzCost || (double.IsFinite(curvatureCost) && curvatureCost >= 0 && margin >= curvatureCost)) return true;
                    if (depth >= MaximumDepth) { _unresolvedRefusals++; return false; }
                    // Exact dyadic parameters on ORIGINAL a/b; these rounded
                    // nominees never become an emitted or authoritative midpoint.
                    double mid = (lo + hi) / 2;
                    return Span(lo, mid, depth + 1) && Span(mid, hi, depth + 1);
                }
                (double FloorLower, double DepthUpper, double DepthLower) At(double t)
                {
                    var x = Axis(originalA.X, originalB.X, t);var y = Axis(originalA.Y, originalB.Y, t);var z = Axis(originalA.Z, originalB.Z, t);
                    double ex = Up(Math.Max(Math.Abs(x.Center - x.Lo), Math.Abs(x.Hi - x.Center)));
                    double ey = Up(Math.Max(Math.Abs(y.Center - y.Lo), Math.Abs(y.Hi - y.Center)));
                    double error = Up(ex + ey), reserve = Up(grade * error);
                    return (Down(originalFloor.Depth(x.Center, y.Center) - reserve), Up(-z.Lo), Down(-z.Hi));
                }
            }
            catch (InvalidOperationException error) when (IsActualF3FloorProofBudgetRefusal(error))
            {
                token.ThrowIfCancellationRequested();_originalQueryBudgetRefusals++;_unresolvedRefusals++;
                _firstOriginalQueryBudgetRefusal ??= error.Message;return false;
            }
            finally { _milliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds; token.ThrowIfCancellationRequested(); }
        }
        private double Up(double value) { _enclosureOperations++; return Math.BitIncrement(value); }
        private double Down(double value) { _enclosureOperations++; return Math.BitDecrement(value); }
        private (double Lo, double Hi, double Center) Axis(double a, double b, double t)
        {
            if (t == 0 || a == b) return (a, a, a);if (t == 1) return (b, b, b);
            double difference = b - a, lo = Down(difference), hi = Up(difference);
            return (Down(a + Down(lo * t)), Up(a + Up(hi * t)), a + difference * t);
        }
    }
    internal static bool IsActualF3FloorProofBudgetRefusal(InvalidOperationException error)
        => error.Message.StartsWith("Smooth floor query exceeded its proof budget at (", StringComparison.Ordinal);

    internal static List<TaggedToolpath> Constrain(IReadOnlyList<TaggedToolpath> paths, FloorField field,
        double grade, CancellationToken cancellationToken, long maximumPoints,bool parallel=false,Action<string>? diagnostic=null)
    {
        if(parallel&&GeometryParallelism.WorkerLimit>1&&paths.Count>=4&&paths.Sum(p=>(long)p.Points.Count)>=8192)
            return field.ConstrainOwned(paths,grade,cancellationToken,maximumPoints,diagnostic);
        return ConstrainCore(paths,field,grade,cancellationToken,maximumPoints,null);
    }

    private static List<TaggedToolpath> ConstrainCore(IReadOnlyList<TaggedToolpath> paths, FloorField field,
        double grade, CancellationToken cancellationToken, long maximumPoints,Action<Point3D,Point3D>? chargePoint)
    {
        var result = new List<TaggedToolpath>(paths.Count);
        long pointCount = 0;
        foreach (TaggedToolpath path in paths)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (path.Points.Count < 2) { result.Add(path); continue; }
            Point3D Cap(Point3D point) => new(point.X, point.Y,
                -field.CappedDepth(-point.Z, point.X, point.Y));
            var points = new List<Point3D> { Cap(path.Points[0]) };
            for (int i = 1; i < path.Points.Count; i++)
            {
                Point3D rawA = path.Points[i - 1], rawB = path.Points[i];
                Subdivide(rawA, rawB, points[^1], Cap(rawB), 0);
            }
            result.Add(new TaggedToolpath(points, Array.Empty<ToolpathSpan>(), path.RegionIndex,
                path.Category, path.DepthPassIndex, path.VisitIdentity, path.DetectCurves)
                {ConcentricSource=path.ConcentricSource});

            void Subdivide(Point3D rawA, Point3D rawB, Point3D a, Point3D b, int level)
            {
                cancellationToken.ThrowIfCancellationRequested();
                double length = Distance(a, b);
                double fa = field.Depth(a.X, a.Y), fb = field.Depth(b.X, b.Y);
                double cutoff = Math.Max(fa, fb);
                double span = field.SublevelGrade(cutoff) * length;
                double t = span > 0 ? Math.Clamp((fa - fb + span) / (2 * span), 0, 1) : 0;
                double lower = Math.Max(0, Math.Max(fa - span * t, fb - span * (1 - t)));
                double commanded = -(a.Z + (b.Z - a.Z) * t);
                double curvature = field.CurvatureBound;
                double curveCost = .5 * curvature * length * length;
                double marginA = fa + a.Z, marginB = fb + b.Z;
                double curveT = curveCost > 0 && double.IsFinite(curveCost)
                    ? Math.Clamp((curveCost - (marginB - marginA)) / (2 * curveCost), 0, 1) : 0;
                double curveMargin = double.IsFinite(curveCost)
                    ? marginA + (marginB - marginA) * curveT - curveCost * curveT * (1 - curveT)
                    : double.NegativeInfinity;
                if (lower + 1e-10 >= commanded || curveMargin >= 0 || field.ProveCrestChord(a, b))
                {
                    if(chargePoint is not null)chargePoint(a,b);
                    else if (++pointCount > maximumPoints)
                        throw new InvalidOperationException($"Smooth V-carving floor proof exceeds {maximumPoints} points at " +
                            $"({a.X:R},{a.Y:R})-({b.X:R},{b.Y:R}); angle grade={grade:R}.");
                    points.Add(b);
                    return;
                }
                if (level >= 28)
                    throw new InvalidOperationException($"Smooth V-carving could not certify the complete floor chord at " +
                        $"({a.X:R},{a.Y:R})-({b.X:R},{b.Y:R}); floorLower={lower:R}, cutDepth={commanded:R}.");
                var rawMid = new Point3D((rawA.X + rawB.X) / 2, (rawA.Y + rawB.Y) / 2, (rawA.Z + rawB.Z) / 2);
                Point3D mid = Cap(rawMid);
                Subdivide(rawA, rawMid, a, mid, level + 1);
                Subdivide(rawMid, rawB, mid, b, level + 1);
            }
        }
        // Raising vertices preserves both the original physical chord proof and
        // the just-established floor proof on each complete chord.
        // Separate depth visits may legitimately revisit the same XY at different
        // depths. Shared nodes belong to the original medial floor graph, while
        // execution-grade constraints belong to each complete cutting visit.
        return result.SelectMany(path => LimitGraph(new[] { path }, grade, cancellationToken)).ToList();
    }

    /// <summary>Deepest symmetric grade envelope on shared XY nodes and their actual cutting edges.</summary>
    internal static List<TaggedToolpath> RaisedFloorSpans(IReadOnlyList<TaggedToolpath> original,
        IReadOnlyList<TaggedToolpath> limited, CancellationToken token)
    {
        var spans = new List<TaggedToolpath>();
        for (int path = 0; path < original.Count; path++)
        for (int i = 1; i < original[path].Points.Count; i++)
        {
            if ((i & 255) == 0) token.ThrowIfCancellationRequested();
            if (limited[path].Points[i - 1].Z <= original[path].Points[i - 1].Z + 1e-10
                && limited[path].Points[i].Z <= original[path].Points[i].Z + 1e-10) continue;
            spans.Add(new TaggedToolpath(new[] { limited[path].Points[i - 1], limited[path].Points[i] },
                Array.Empty<ToolpathSpan>(), original[path].RegionIndex, "final-carve", null, detectCurves: false));
        }
        return spans;
    }

    internal static List<TaggedToolpath> LimitGraph(IReadOnlyList<TaggedToolpath> source,
        double grade, CancellationToken cancellationToken = default)
        => LimitGraphCore(source, grade, cancellationToken, null, out _);

    internal static (List<TaggedToolpath> Limited, List<TaggedToolpath> Upper) LimitGraphPair(
        IReadOnlyList<TaggedToolpath> source, IReadOnlyList<TaggedToolpath> upper,
        double grade, CancellationToken token,Action<string>? diagnostic=null)
    {
        var limited = LimitGraphCore(source, grade, token, upper, out var upperLimited,diagnostic);
        return (limited, upperLimited!);
    }

    private static List<TaggedToolpath> LimitGraphCore(IReadOnlyList<TaggedToolpath> source,
        double grade, CancellationToken cancellationToken, IReadOnlyList<TaggedToolpath>? additional,
        out List<TaggedToolpath>? additionalLimited,Action<string>? diagnostic=null)
    {
        long started=Stopwatch.GetTimestamp(),seedCount=0,popCount=0,relaxations=0;
        additionalLimited = null;
        var nodes = new Dictionary<(double X, double Y), int>();
        var depths = new List<double>();
        var neighbors = new List<List<(int Node, double Cost)>>();
        var paths = new List<int[]>(source.Count);
        foreach (TaggedToolpath path in source)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var ids = new int[path.Points.Count];
            for (int i = 0; i < ids.Length; i++)
            {
                Point3D point = path.Points[i];
                if (!nodes.TryGetValue((point.X, point.Y), out int id))
                {
                    id = nodes.Count; nodes.Add((point.X, point.Y), id);
                    depths.Add(Math.Max(0, -point.Z)); neighbors.Add(new());
                }
                else depths[id] = Math.Min(depths[id], Math.Max(0, -point.Z));
                ids[i] = id;
                if (i == 0 || id == ids[i - 1]) continue;
                double cost = grade * Distance(path.Points[i - 1], point);
                neighbors[id].Add((ids[i - 1], cost));
                neighbors[ids[i - 1]].Add((id, cost));
            }
            paths.Add(ids);
        }
        var queue = new PriorityQueue<int, double>();
        Solve();
        var result = Publish(source);
        if (additional is not null)
        {
            if (additional.Count != source.Count) throw new ArgumentException("Paired grade fields require identical XY topology.", nameof(additional));
            for (int i = 0; i < depths.Count; i++) depths[i] = double.PositiveInfinity;
            for (int path = 0; path < additional.Count; path++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (additional[path].Points.Count != paths[path].Length)
                    throw new ArgumentException("Paired grade fields require identical XY topology.", nameof(additional));
                for (int i = 0; i < paths[path].Length; i++)
                {
                    var point = additional[path].Points[i]; var original = source[path].Points[i];
                    if (point.X != original.X || point.Y != original.Y)
                        throw new ArgumentException("Paired grade fields require identical XY topology.", nameof(additional));
                    int id = paths[path][i]; depths[id] = Math.Min(depths[id], Math.Max(0, -point.Z));
                }
            }
            Solve(); additionalLimited = Publish(additional);
        }
        diagnostic?.Invoke($"event=grade-graph nodes={depths.Count} directedEdges={neighbors.Sum(n=>(long)n.Count)} "
            +$"fields={(additional is null?1:2)} seeds={seedCount} pops={popCount} relaxations={relaxations} "
            +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} initialSeeds=only-literal-edge-violations originalRelaxationArithmetic=True");
        return result;

        void Solve()
        {
            queue.Clear();
            // An initial label that cannot improve ANY neighbour has no work.
            // Other labels only decrease, so it cannot become useful until its
            // own label decreases; that original relaxation enqueues it below.
            // This skips no relaxation and retains the literal rounded costs.
            for (int i = 0; i < depths.Count; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                foreach(var edge in neighbors[i])
                    if(Candidate(depths[i],edge.Cost)<depths[edge.Node])
                    {queue.Enqueue(i,depths[i]);seedCount++;break;}
            }
            while (queue.TryDequeue(out int node, out double depth))
            {
                popCount++;
                cancellationToken.ThrowIfCancellationRequested();
                if (depth > depths[node]) continue;
                foreach (var edge in neighbors[node])
                {
                    double candidate = Candidate(depth,edge.Cost);
                    // Addition at a deep floor can round upward by one ULP. On a
                    // very short chord that can exceed the grade audit despite a
                    // mathematically exact Dijkstra bound. Preserve the actual
                    // representable depth difference, without widening the angle.
                    if (candidate >= depths[edge.Node]) continue;
                    depths[edge.Node] = candidate;
                    relaxations++;
                    queue.Enqueue(edge.Node, candidate);
                }
            }
        }
        static double Candidate(double depth,double cost)
        {
            double candidate=depth+cost;
            if(candidate>depth&&candidate-depth>cost)candidate=Math.BitDecrement(candidate);
            return candidate;
        }
        List<TaggedToolpath> Publish(IReadOnlyList<TaggedToolpath> input)
            => input.Select((path, index) => new TaggedToolpath(path.Points.Select((point, i) =>
            new Point3D(point.X, point.Y, -depths[paths[index][i]])).ToList(), Array.Empty<ToolpathSpan>(),
            path.RegionIndex, path.Category, path.DepthPassIndex, path.VisitIdentity, path.DetectCurves)
                {ConcentricSource=path.ConcentricSource}).ToList();
    }

    /// <summary>Certifies physical wall clearance on the actual float/F3 cutting chords.</summary>
    internal sealed class CommandClearance
    {
        internal double NominationArtworkDistance(double x, double y)
        {
            _token.ThrowIfCancellationRequested();
            if (!double.IsFinite(x) || !double.IsFinite(y) || Math.Abs(x) > 1e6 || Math.Abs(y) > 1e6)
                return double.NaN;
            var integer = new Point64((long)Math.Round(x * PathUtils.Scale), (long)Math.Round(y * PathUtils.Scale));
            return _domains[0].Inside.IsInside(integer) ? _domains[0].Boundary.QueryDistance(x, y) : double.NaN;
        }
        private const int MaximumChordCacheEntries = 65_536;
        private const int MaximumCachedProfileSections = 64;
        private readonly List<(MedialAxisToolpaths.SpatialSegmentIndex Boundary,
            PolygonOps.BoundarySegmentIndex Inside)> _domains = new();
        private readonly VCarveCutterProfile _profile;
        private readonly double _surface;
        private readonly VCarveMaterialSurface? _stockContact;
        private readonly CancellationToken _token;
        private readonly Action<string>? _diagnostic;
        private readonly long _maximumPoints;
        private readonly Dictionary<(double, double), double> _depthCaps = new();
        private const int MaximumArtworkProofPoints = 524_288;
        private const long MaximumArtworkProofOwnedBytes = 32L * 1024 * 1024;
        private readonly IReadOnlyList<IReadOnlyList<PointD>>? _literalArtwork;
        private readonly ArtworkProofPacket? _parentArtworkProof;
        private ArtworkProofPacket? _capturedArtworkProof;
        private bool _captureArtworkProof, _reuseThisArtworkChord;
        private int _artworkProofPointLimit = MaximumArtworkProofPoints;
        private long _artworkProofByteLimit = MaximumArtworkProofOwnedBytes;
        private string? _artworkCaptureRefusal, _artworkFirstMiss;
        private long _artworkLookupTicks, _artworkReusedProofTicks;
        internal bool ArtworkIndexReused { get; }
        internal long ArtworkCertificateHits { get; private set; }
        internal long ArtworkCertificateMisses { get; private set; }
        internal long ArtworkLiteralChordQueries { get; private set; }
        internal long ChannelLiteralChordQueries { get; private set; }

        // The compact key admits only the exact binary64 values of an F3
        // command. Round-trip equality is checked; signed zero is separate.
        private readonly record struct ArtworkProofPoint(int X, int Y, int Z, int ZeroSigns, long ProofTicks)
        {
            internal static bool TryCapture(Point3D p, long ticks, out ArtworkProofPoint result)
            {
                result = default; int signs = 0;
                if (!Coordinate(p.X, 1, out int x) || !Coordinate(p.Y, 2, out int y)
                    || !Coordinate(p.Z, 4, out int z)) return false;
                result = new(x, y, z, signs, ticks); return true;
                bool Coordinate(double value, int flag, out int key)
                {
                    key = 0;
                    if (!double.IsFinite(value) || Math.Abs(value) > 1_000_000) return false;
                    double scaled = Math.Round(value * 1000);
                    if (scaled < int.MinValue || scaled > int.MaxValue) return false;
                    key = (int)scaled;
                    if (value == 0) { if (Bits(value) < 0) signs |= flag; return true; }
                    return Bits(key / 1000d) == Bits(value);
                }
            }
            internal bool SameCommand(ArtworkProofPoint p)
                => X == p.X && Y == p.Y && Z == p.Z && ZeroSigns == p.ZeroSigns;
        }
        private sealed record ArtworkProofPacket(ArtworkProofPoint[][] Paths, int Points, long OwnedBytes);

        internal void EnableArtworkProofCapture(int maximumCertificatePoints = MaximumArtworkProofPoints,
            long maximumCertificateOwnedBytes = MaximumArtworkProofOwnedBytes)
        {
            _token.ThrowIfCancellationRequested();
            if (maximumCertificatePoints < 0 || maximumCertificatePoints > MaximumArtworkProofPoints
                || maximumCertificateOwnedBytes < 0 || maximumCertificateOwnedBytes > MaximumArtworkProofOwnedBytes)
                throw new ArgumentOutOfRangeException(nameof(maximumCertificatePoints));
            _artworkProofPointLimit = maximumCertificatePoints;
            _artworkProofByteLimit = maximumCertificateOwnedBytes;
            _captureArtworkProof = _domains.Count == 1 && _literalArtwork is not null;
            if (!_captureArtworkProof) _artworkCaptureRefusal = "artwork-only-literal-owner-required";
        }
        private IReadOnlyList<IReadOnlyList<PointD>>? CopyBoundedLiteralArtwork(IReadOnlyList<IReadOnlyList<PointD>> source)
        {
            long points = 0;
            foreach (var row in source)
            {
                _token.ThrowIfCancellationRequested(); points += row.Count;
                if (points > MaximumArtworkProofPoints || source.Count > MaximumArtworkProofPoints)
                    return null;
            }
            if (16L * points + 64L * source.Count + 32 > MaximumArtworkProofOwnedBytes) return null;
            var copied = new IReadOnlyList<PointD>[source.Count];
            for (int row = 0; row < copied.Length; row++)
            {
                var values = new PointD[source[row].Count];
                for (int i = 0; i < values.Length; i++)
                { if ((i & 255) == 0) _token.ThrowIfCancellationRequested(); values[i] = source[row][i]; }
                copied[row] = Array.AsReadOnly(values);
            }
            return Array.AsReadOnly(copied);
        }
        private bool LiteralArtworkMatches(IReadOnlyList<IReadOnlyList<PointD>> source,
            IReadOnlyList<IReadOnlyList<PointD>>? owned)
        {
            if (owned is null || source.Count != owned.Count) return false;
            for (int row = 0; row < owned.Count; row++)
            {
                _token.ThrowIfCancellationRequested();
                if (source[row].Count != owned[row].Count) return false;
                for (int i = 0; i < owned[row].Count; i++)
                {
                    if ((i & 255) == 0) _token.ThrowIfCancellationRequested();
                    if (Bits(source[row][i].x) != Bits(owned[row][i].x)
                        || Bits(source[row][i].y) != Bits(owned[row][i].y)) return false;
                }
            }
            return true;
        }
        private bool TryReuseArtworkCertificate(int path, int chord, Point3D a, Point3D b)
        {
            if (_parentArtworkProof is null) return false;
            long started = Stopwatch.GetTimestamp(); _token.ThrowIfCancellationRequested();
            string? reason = null;
            if ((uint)path >= (uint)_parentArtworkProof.Paths.Length) reason = "path-ordinal";
            else
            {
                var previous = _parentArtworkProof.Paths[path];
                if (chord < 0 || chord >= previous.Length - 1) reason = "chord-ordinal";
                else if (!ArtworkProofPoint.TryCapture(a, 0, out var ca)
                    || !ArtworkProofPoint.TryCapture(b, 0, out var cb)) reason = "actual-F3-key";
                else if (!previous[chord].SameCommand(ca) || !previous[chord + 1].SameCommand(cb))
                    reason = "directed-actual-command-XYZ";
                else
                {
                    ArtworkCertificateHits++; _artworkReusedProofTicks += previous[chord + 1].ProofTicks;
                    _artworkLookupTicks += Stopwatch.GetTimestamp() - started;
                    return true;
                }
            }
            ArtworkCertificateMisses++; _artworkFirstMiss ??= reason;
            _artworkLookupTicks += Stopwatch.GetTimestamp() - started;
            return false;
        }

        // Like the existing depth-cap dictionary, this owner is used sequentially
        // by one generation. Keys are directed, already-commanded XYZ bit patterns.
        // No result is shared across profile/domain/surface owners or generations.
        private readonly Dictionary<CommandedChordKey, ChordEvidence> _chordCache = new();
        private readonly Queue<CommandedChordKey> _chordCacheOrder = new();
        private readonly int _maximumCachedChords;
        private readonly SectionSnapshot[] _profileSections;
        private readonly ProfileSnapshot _profileSnapshot;
        private readonly string? _chordCacheDisabledReason;
        internal long ChordCacheHits { get; private set; }
        internal long ChordCacheMisses { get; private set; }
        internal long ChordCacheRefusals { get; private set; }
        internal long ChordCacheEvictions { get; private set; }
        internal long LiteralChordQueries { get; private set; }
        internal long LiteralChordDomainQueries { get; private set; }
        internal int ChordCacheEntries => _chordCache.Count;

        private readonly record struct CommandedChordKey(long Ax, long Ay, long Az, long Bx, long By, long Bz)
        {
            internal static CommandedChordKey Capture(Point3D a, Point3D b)
                => new(Bits(a.X), Bits(a.Y), Bits(a.Z), Bits(b.X), Bits(b.Y), Bits(b.Z));
        }
        private readonly record struct ChordEvidence(bool Violation, double Witness);
        private readonly record struct SectionSnapshot(CarvingCutterSectionKind Kind, long Bottom, long Top, long Height)
        {
            internal static SectionSnapshot Capture(CarvingCutterSection section)
                => new(section.Kind, Bits(section.BottomRadius), Bits(section.TopRadius), Bits(section.Height));
        }
        private readonly record struct ProfileSnapshot(long MaximumDepth, long TipRadius, bool RoundedTip,
            long ClearingDiameter, long PhysicalTipRadius, long PhysicalMaximumRadius, long ContactHeight,
            bool FlatCylinder, bool StraightBall)
        {
            internal static ProfileSnapshot Capture(VCarveCutterProfile profile)
                => new(Bits(profile.MaximumDepth), Bits(profile.TipRadius), profile.RoundedTip,
                    Bits(profile.ClearingDiameter), Bits(profile.PhysicalProfile.TipRadius),
                    Bits(profile.PhysicalProfile.MaximumRadius), Bits(profile.PhysicalProfile.MaximumContactHeight),
                    profile.PhysicalProfile.IsFlatCylinder, profile.PhysicalProfile.IsStraightBall);
        }
        private static long Bits(double value) => BitConverter.DoubleToInt64Bits(value);

        internal CommandClearance(IReadOnlyList<IReadOnlyList<PointD>> boundary,
            VCarveCutterProfile profile, double surface, CancellationToken token, Action<string>? diagnostic,
            IReadOnlyList<IReadOnlyList<PointD>>? originalChannel = null, long maximumPoints = MaximumPoints,
            bool useChordCache = true, int maximumCachedChords = MaximumChordCacheEntries, VCarveMaterialSurface? stockContact = null)
            : this(boundary, profile, surface, token, diagnostic, originalChannel, maximumPoints,
                useChordCache, maximumCachedChords, null, stockContact) { }

        internal CommandClearance(IReadOnlyList<IReadOnlyList<PointD>> boundary,
            VCarveCutterProfile profile, double surface, CancellationToken token, Action<string>? diagnostic,
            IReadOnlyList<IReadOnlyList<PointD>> originalChannel, long maximumPoints,
            CommandClearance artworkProofOwner)
            : this(boundary, profile, surface, token, diagnostic, originalChannel, maximumPoints,
                true, MaximumChordCacheEntries, artworkProofOwner, artworkProofOwner._stockContact) { }

        private CommandClearance(IReadOnlyList<IReadOnlyList<PointD>> boundary,
            VCarveCutterProfile profile, double surface, CancellationToken token, Action<string>? diagnostic,
            IReadOnlyList<IReadOnlyList<PointD>>? originalChannel, long maximumPoints,
            bool useChordCache, int maximumCachedChords, CommandClearance? artworkProofOwner, VCarveMaterialSurface? stockContact)
        {
            _profile = profile; _surface = surface; _token = token; _diagnostic = diagnostic; _stockContact = stockContact;
            _maximumPoints = maximumPoints;
            if (maximumCachedChords < 0 || maximumCachedChords > MaximumChordCacheEntries)
                throw new ArgumentOutOfRangeException(nameof(maximumCachedChords));
            _maximumCachedChords = maximumCachedChords;
            _profileSnapshot = ProfileSnapshot.Capture(profile);
            bool supported = profile.Sections.Count is > 0 and <= MaximumCachedProfileSections
                && profile.PhysicalProfile.Sections.Length == profile.Sections.Count
                && double.IsFinite(surface);
            _profileSections = supported ? profile.Sections.Select(SectionSnapshot.Capture).ToArray() : Array.Empty<SectionSnapshot>();
            // The supported public contract is immutable: the sealed profile
            // copies its input array, exposes a read-only view, and its sealed
            // section records have init-only values. Capture/verify the values
            // once; no per-chord section scans or reflection/unsafe contract.
            supported = supported && ProfileSnapshotMatchesAtCreation();
            _chordCacheDisabledReason = !useChordCache || maximumCachedChords == 0 ? "disabled"
                : !supported ? "unsupported-profile-or-surface" : null;
            token.ThrowIfCancellationRequested();
            if (artworkProofOwner is not null && ReferenceEquals(profile, artworkProofOwner._profile)
                && Bits(surface) == Bits(artworkProofOwner._surface) && token == artworkProofOwner._token
                && artworkProofOwner._domains.Count == 1
                && LiteralArtworkMatches(boundary, artworkProofOwner._literalArtwork))
            {
                artworkProofOwner._token.ThrowIfCancellationRequested();
                _literalArtwork = artworkProofOwner._literalArtwork;
                _domains.Add(artworkProofOwner._domains[0]);
                _parentArtworkProof = artworkProofOwner._capturedArtworkProof;
                ArtworkIndexReused = true;
            }
            else
            {
                _literalArtwork = CopyBoundedLiteralArtwork(boundary);
                Add(_literalArtwork ?? boundary);
            }
            if (originalChannel is not null) Add(originalChannel);
            _diagnostic?.Invoke($"stage=smooth-artwork-proof-reuse event=selection indexReused={ArtworkIndexReused} "
                + $"certificateAvailable={_parentArtworkProof is not null} "
                + $"certificatePoints={_parentArtworkProof?.Points ?? 0} maximumCertificatePoints={MaximumArtworkProofPoints} "
                + $"maximumCertificateOwnedBytes={MaximumArtworkProofOwnedBytes} "
                + "policy=exact-directed-F3-XYZ-and-ordinal profileObjectAndLiteralArtworkAndSurfaceAndTokenBound=True "
                + "freshOriginalChannelProof=True decreasedDepthCertificateReuse=False initialArtworkSuccessPacketOnly=True unmatchedCommandsFresh=True");
            _diagnostic?.Invoke($"stage=smooth-command-chord-cache event=selection enabled={_chordCacheDisabledReason is null} "
                + $"reason={_chordCacheDisabledReason ?? "ready"} maximumEntries={_maximumCachedChords} "
                + "key=directed-commanded-xyz-ieee-bits owner=one-sequential-clearance-instance "
                + "profileOwnership=sealed-readonly-constructor-copy literalDomainOrder=True witnessBitsPreserved=True pointQuotasUnchanged=True");

            void Add(IReadOnlyList<IReadOnlyList<PointD>> rings)
            {
                var edges = new List<(PointD A, PointD B)>();
                foreach (var ring in rings)
                for (int i = 0; i < ring.Count; i++) edges.Add((ring[i], ring[(i + 1) % ring.Count]));
                _domains.Add((new MedialAxisToolpaths.SpatialSegmentIndex(edges),
                    new PolygonOps.BoundarySegmentIndex(Clipper.Union(PathUtils.ToClipper(rings), FillRule.NonZero))));
            }
        }

        private double AllowedDepth(Point3D command)
        {
            if (_depthCaps.TryGetValue((command.X, command.Y), out double allowed)) return allowed;
            var integer = new Point64((long)Math.Round(command.X * PathUtils.Scale),
                (long)Math.Round(command.Y * PathUtils.Scale));
            double clearance = double.PositiveInfinity;
            for(int i=0;i<_domains.Count;i++)
            {
                if(i==0&&_stockContact is not null)continue;
                var domain=_domains[i];
                if (!domain.Inside.IsInside(integer)) { clearance = 0; break; }
                clearance = Math.Min(clearance, domain.Boundary.QueryDistance(command.X, command.Y));
            }
            allowed = _profile.DepthAtRadius(Math.Max(0, clearance - 2 / PathUtils.Scale));
            if(_stockContact is not null)allowed=Math.Min(allowed,_stockContact.AllowedDepth(command.X,command.Y));
            _depthCaps[(command.X, command.Y)] = allowed;
            return allowed;
        }

        // Readonly forwarding used by floor-aware nomination. This is the same
        // existing F3 endpoint quantity; actual whole-chord/floor/grade guards remain mandatory.
        internal double NominateEndpointAllowedDepth(Point3D point)
        {
            _token.ThrowIfCancellationRequested();
            double allowed = AllowedDepth(CommandPoint(point, _surface));
            _token.ThrowIfCancellationRequested();
            return allowed;
        }

        internal bool IsLegalCommandedCenter(Point3D point)
        {
            Point3D command = CommandPoint(point, _surface);
            return command.Z < 0 && -command.Z <= AllowedDepth(command) + 1e-12;
        }

        internal bool IsLegalCommandedChord(Point3D a, Point3D b)
        {
            Point3D ca = CommandPoint(a, _surface), cb = CommandPoint(b, _surface);
            return !TryFindCommandedChordViolation(ca, cb, out _);
        }

        private bool TryFindCommandedChordViolation(Point3D ca, Point3D cb, out double witness)
        {
            _token.ThrowIfCancellationRequested();
            var key = CommandedChordKey.Capture(ca, cb);
            bool finite = double.IsFinite(ca.X) && double.IsFinite(ca.Y) && double.IsFinite(ca.Z)
                && double.IsFinite(cb.X) && double.IsFinite(cb.Y) && double.IsFinite(cb.Z);
            string? refusal = _chordCacheDisabledReason ?? (finite ? null : "nonfinite-command");
            if (refusal is null && _chordCache.TryGetValue(key, out var evidence))
            {
                ChordCacheHits++; witness = evidence.Witness;
                LogChordCache("hit", ChordCacheHits, key, evidence.Violation, witness, "identical-literal-inputs");
                _token.ThrowIfCancellationRequested();
                return evidence.Violation;
            }

            if (refusal is null) ChordCacheMisses++; else ChordCacheRefusals++;
            LiteralChordQueries++;
            witness = 0;
            bool violation = false;
            for (int domainOrdinal = 0; domainOrdinal < _domains.Count; domainOrdinal++)
            {
                var domain = _domains[domainOrdinal];
                if (domainOrdinal == 0 && _reuseThisArtworkChord) continue;
                if(domainOrdinal==0&&_stockContact is not null)
                {
                    if(!_stockContact.IsLegalChord(ca,cb)){violation=true;witness=.5;break;}
                    continue;
                }
                LiteralChordDomainQueries++;
                if (domainOrdinal == 0) ArtworkLiteralChordQueries++; else ChannelLiteralChordQueries++;
                if (domain.Boundary.TryFindDepthViolation(ca, cb, 0, 0, 1, 0, out witness, _profile))
                { violation = true; break; }
            }
            _token.ThrowIfCancellationRequested();
            LogChordCache(refusal is null ? "miss" : "refusal", refusal is null ? ChordCacheMisses : ChordCacheRefusals,
                key, violation, witness, refusal ?? "literal-original-query");
            _token.ThrowIfCancellationRequested();
            if (refusal is null)
            {
                if (_chordCache.Count == _maximumCachedChords)
                { _chordCache.Remove(_chordCacheOrder.Dequeue()); ChordCacheEvictions++; }
                _chordCache.Add(key, new(violation, witness)); _chordCacheOrder.Enqueue(key);
            }
            return violation;
        }

        private bool ProfileSnapshotMatchesAtCreation()
        {
            // Public sections are a read-only copied array of sealed init-only
            // records, while the physical array contains the same record values.
            // Confirm their copied primitive bits once at owner creation.
            // Unsupported layouts retain literal queries; supported public APIs
            // cannot mutate either view or a section after initialization.
            if (ProfileSnapshot.Capture(_profile) != _profileSnapshot
                || _profile.Sections.Count != _profileSections.Length
                || _profile.PhysicalProfile.Sections.Length != _profileSections.Length) return false;
            for (int i = 0; i < _profileSections.Length; i++)
                if (_profile.Sections[i] is not { } section
                    || SectionSnapshot.Capture(section) != _profileSections[i]
                    || _profile.PhysicalProfile.Sections[i] is not { } physical
                    || SectionSnapshot.Capture(physical) != _profileSections[i]) return false;
            return true;
        }

        private void LogChordCache(string action, long count, CommandedChordKey key, bool violation, double witness, string reason)
        {
            if (_diagnostic is null || count > 4 && (count & (count - 1)) != 0) return;
            _diagnostic($"stage=smooth-command-chord-cache event={action} reason={reason} "
                + $"hits={ChordCacheHits} misses={ChordCacheMisses} refusals={ChordCacheRefusals} "
                + $"literalQueries={LiteralChordQueries} literalDomainQueries={LiteralChordDomainQueries} "
                + $"entries={_chordCache.Count} evictions={ChordCacheEvictions} violation={violation} witnessBits={Bits(witness):x16} "
                + $"aBits={key.Ax:x16},{key.Ay:x16},{key.Az:x16} bBits={key.Bx:x16},{key.By:x16},{key.Bz:x16} "
                + "literalDomainOrder=True pointQuotasUnchanged=True");
        }

        internal List<TaggedToolpath> SnapCorrectionCenters(IReadOnlyList<TaggedToolpath> paths)
        {
            int adjusted = 0, unresolved = 0, expandedSearches = 0, computationallyBoundedSearches = 0;
            var attemptsByExtent = new SortedDictionary<int, int>();
            var hitsByExtent = new SortedDictionary<int, int>();
            double maximumMove = 0;
            var result = new List<TaggedToolpath>(paths.Count);
            foreach (var path in paths)
            {
                _token.ThrowIfCancellationRequested();
                var points = new List<Point3D>(path.Points.Count);
                foreach (Point3D point in path.Points)
                {
                    _token.ThrowIfCancellationRequested();
                    Point3D command = CommandPoint(point, _surface);
                    if (-command.Z <= AllowedDepth(command) + 1e-12) { points.Add(point); continue; }
                    // Choose an actually legal neighboring F3 center instead of
                    // letting coordinate rounding turn a minimum-depth ring into
                    // a surface-only trace. This is only an endpoint candidate:
                    // the subsequent floor, grade and complete wall-chord proofs
                    // still apply to every selected point and connecting segment.
                    Point3D selected = point;
                    bool found = false;
                    int previousExtent = 0;
                    // In a thin oblique center strip, legal F3 positions may be
                    // separated by more than a handful of grid steps. Search
                    // adaptively through the local cutter-reach neighborhood.
                    // A 16,640-candidate cap bounds pathological large cutters;
                    // exhausting it is reported and never certifies infeasibility.
                    int physicalExtent = (int)Math.Ceiling(Math.Min(64,
                        _profile.PhysicalRadiusAtDepth(-command.Z) / .001));
                    int maximumExtent = Math.Max(2, physicalExtent);
                    if (_profile.PhysicalRadiusAtDepth(-command.Z) > .064) computationallyBoundedSearches++;
                    for (int extent = 2; !found;)
                    {
                        attemptsByExtent[extent] = attemptsByExtent.TryGetValue(extent, out int attempts) ? attempts + 1 : 1;
                        if (extent > 2) expandedSearches++;
                        var candidates = new List<Point3D>();
                        for (int dx = -extent; dx <= extent; dx++)
                        for (int dy = -extent; dy <= extent; dy++)
                        {
                            if (Math.Max(Math.Abs(dx), Math.Abs(dy)) <= previousExtent) continue;
                            candidates.Add(new Point3D(command.X + dx * .001, command.Y + dy * .001, point.Z));
                        }
                        int probes = 0;
                        foreach (Point3D candidate in candidates.OrderBy(candidate => Distance(candidate, point)))
                        {
                            if ((++probes & 255) == 0) _token.ThrowIfCancellationRequested();
                            Point3D emitted = CommandPoint(candidate, _surface);
                            if (-emitted.Z > AllowedDepth(emitted) + 1e-12) continue;
                            selected = candidate; found = true;
                            hitsByExtent[extent] = hitsByExtent.TryGetValue(extent, out int hits) ? hits + 1 : 1;
                            break;
                        }
                        previousExtent = extent;
                        if (extent == maximumExtent) break;
                        extent = Math.Min(maximumExtent, extent * 2);
                    }
                    if (found) { adjusted++; maximumMove = Math.Max(maximumMove, Distance(point, selected)); }
                    else unresolved++;
                    points.Add(selected);
                }
                result.Add(new TaggedToolpath(points, Array.Empty<ToolpathSpan>(), path.RegionIndex, path.Category,
                    path.DepthPassIndex, path.VisitIdentity, detectCurves: false)
                    {ConcentricSource=path.ConcentricSource});
            }
            _diagnostic?.Invoke($"stage=smooth-command-centers adjusted={adjusted} unresolved={unresolved} " +
                $"maximumMove={maximumMove:R} expandedSearches={expandedSearches} gridStep=0.001 " +
                $"searchBound=engagement-radius-or-16640-candidate-budget computationallyBoundedSearches={computationallyBoundedSearches} domains={_domains.Count} " +
                $"searchAttempts={string.Join(",", attemptsByExtent.Select(pair => $"{pair.Key}:{pair.Value}"))} " +
                $"searchHits={string.Join(",", hitsByExtent.Select(pair => $"{pair.Key}:{pair.Value}"))} " +
                "unresolvedMeans=bounded-search-exhausted-not-infeasible " +
                "completeFloorAndWallChordProofFollows=True fixedCoverageTestStillRequired=True");
            return result;
        }

        internal List<TaggedToolpath> Apply(IReadOnlyList<TaggedToolpath> paths)
        {
            int raised = 0, splits = 0, points = 0;
            double maximumRaise = 0;
            string maximumWitness = "none";
            var result = new List<TaggedToolpath>(paths.Count);
            _capturedArtworkProof = null;
            var capturedRows = _captureArtworkProof ? new List<ArtworkProofPoint[]>(paths.Count) : null;
            int capturedPoints = 0;
            long capturedBytes = _literalArtwork?.Sum(row => 64L + 16L * row.Count) ?? 0;
            int pathOrdinal = -1;
            Point3D Cap(Point3D point)
            {
                Point3D command = CommandPoint(point, _surface);
                if (command.Z >= 0) return point;
                double allowed = AllowedDepth(command);
                if (-command.Z <= allowed + 1e-12) return point;
                double safeDepth = Math.Max(0, Math.Floor(allowed * 1000) / 1000);
                var safer = new Point3D(point.X, point.Y, -Math.Min(-point.Z, safeDepth));
                while (-CommandPoint(safer, _surface).Z > allowed + 1e-12)
                    safer = new Point3D(point.X, point.Y, Math.Min(0, safer.Z + .001));
                raised++;
                if (safer.Z - point.Z > maximumRaise)
                {
                    maximumRaise = safer.Z - point.Z;
                    var integer = new Point64((long)Math.Round(command.X * PathUtils.Scale),
                        (long)Math.Round(command.Y * PathUtils.Scale));
                    maximumWitness = $"({command.X:R},{command.Y:R},{command.Z:R})->{CommandPoint(safer, _surface).Z:R} " +
                        $"allowedDepth={allowed:R} domains=[" + string.Join(";", _domains.Select((domain, index) =>
                            $"{index}:inside={domain.Inside.IsInside(integer)},clearance={domain.Boundary.QueryDistance(command.X, command.Y):R}")) + "]";
                }
                return safer;
            }
            foreach (TaggedToolpath path in paths)
            {
                pathOrdinal++;
                _token.ThrowIfCancellationRequested();
                if (path.Points.Count < 2)
                { result.Add(path); if (capturedRows is not null) capturedRows.Add(Array.Empty<ArtworkProofPoint>()); continue; }
                var output = new List<Point3D> { Cap(path.Points[0]) };
                List<ArtworkProofPoint>? commandProof = capturedRows is null ? null : new();
                CaptureCommand(CommandPoint(output[0], _surface), 0);
                int chordOrdinal = 0;
                for (int i = 1; i < path.Points.Count; i++)
                { chordOrdinal = i - 1; Prove(output[^1], Cap(path.Points[i]), 0); }
                if (capturedRows is not null && commandProof is not null)
                {
                    capturedRows.Add(commandProof.ToArray()); capturedPoints += commandProof.Count;
                    capturedBytes += 32L + 24L * commandProof.Count;
                }
                // This pre-routing stage contains cutting traces only. Entries
                // and their indexed spans are materialized by the router later.
                result.Add(new TaggedToolpath(output, Array.Empty<ToolpathSpan>(), path.RegionIndex, path.Category,
                    path.DepthPassIndex, path.VisitIdentity, detectCurves: false)
                    {ConcentricSource=path.ConcentricSource});

                void Prove(Point3D a, Point3D b, int level)
                {
                    _token.ThrowIfCancellationRequested();
                    Point3D ca = CommandPoint(a, _surface), cb = CommandPoint(b, _surface);
                    long proofStarted = commandProof is null ? 0 : Stopwatch.GetTimestamp();
                    _reuseThisArtworkChord = TryReuseArtworkCertificate(pathOrdinal, chordOrdinal, ca, cb);
                    bool violation;
                    double witness;
                    try { violation = TryFindCommandedChordViolation(ca, cb, out witness); }
                    finally { _reuseThisArtworkChord = false; }
                    if (!violation)
                    {
                        if (++points > _maximumPoints)
                            throw new InvalidOperationException($"Smooth V-carving command wall proof exceeds {_maximumPoints} points.");
                        CaptureCommand(cb, proofStarted == 0 ? 0 : Stopwatch.GetTimestamp() - proofStarted);
                        output.Add(b); return;
                    }
                    if (level >= 28)
                        throw new InvalidOperationException($"Smooth V-carving could not prove commanded cutter-wall clearance " +
                            $"from ({ca.X:R},{ca.Y:R},{ca.Z:R}) to ({cb.X:R},{cb.Y:R},{cb.Z:R}), witness={witness:R}.");
                    double t = Math.Clamp(witness, .1, .9);
                    var mid = Cap(new Point3D(a.X + (b.X - a.X) * t,
                        a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t));
                    splits++;
                    Prove(a, mid, level + 1); Prove(mid, b, level + 1);
                }

                void CaptureCommand(Point3D command, long ticks)
                {
                    if (capturedRows is null || commandProof is null) return;
                    int nextCapacity = commandProof.Count == commandProof.Capacity
                        ? Math.Max(4, checked(commandProof.Capacity * 2)) : commandProof.Capacity;
                    long oldGrowthBacking = nextCapacity > commandProof.Capacity ? 24L * commandProof.Capacity : 0;
                    long peak = checked(capturedBytes + oldGrowthBacking + 24L * nextCapacity + 24L * (commandProof.Count + 1)
                        + 64L * paths.Count);
                    if (capturedPoints + commandProof.Count + 1 > _artworkProofPointLimit
                        || peak > _artworkProofByteLimit
                        || !ArtworkProofPoint.TryCapture(command, ticks, out var captured))
                    {
                        capturedRows = null; commandProof = null;
                        _artworkCaptureRefusal = "point-byte-or-exact-F3-key-budget";
                        return;
                    }
                    commandProof.Add(captured);
                }
            }
            _token.ThrowIfCancellationRequested();
            ArtworkProofPacket? preparedArtworkProof = capturedRows is null ? null
                : new(capturedRows.ToArray(), capturedPoints, capturedBytes);
            _diagnostic?.Invoke($"stage=smooth-artwork-proof-reuse event=summary indexReused={ArtworkIndexReused} "
                + $"hits={ArtworkCertificateHits} misses={ArtworkCertificateMisses} firstMiss={_artworkFirstMiss ?? "none"} "
                + $"artworkLiteralQueries={ArtworkLiteralChordQueries} channelLiteralQueries={ChannelLiteralChordQueries} "
                + $"lookupMs={1000d * _artworkLookupTicks / Stopwatch.Frequency:F3} "
                + $"recordedFirstPassProofMsForHits={1000d * _artworkReusedProofTicks / Stopwatch.Frequency:F3} "
                + $"capturedPoints={preparedArtworkProof?.Points ?? 0} capturedBytes={preparedArtworkProof?.OwnedBytes ?? 0} "
                + $"captureRefusal={_artworkCaptureRefusal ?? "none"} "
                + "recordedProofTimeIsNotMeasuredJobSaving=True freshChannelProof=True exactActualCommandXYZRequired=True");
            _token.ThrowIfCancellationRequested();
            _diagnostic?.Invoke($"stage=smooth-command-wall paths={paths.Count} points={points} " +
                $"raisedEndpoints={raised} refinedChords={splits} maximumDepthRaise={maximumRaise:R} " +
                $"maximumWitness={maximumWitness} domains={_domains.Count} " +
                $"chordCacheHits={ChordCacheHits} chordCacheMisses={ChordCacheMisses} chordCacheRefusals={ChordCacheRefusals} " +
                $"chordCacheEntries={ChordCacheEntries} chordCacheEvictions={ChordCacheEvictions} " +
                $"literalChordQueries={LiteralChordQueries} literalChordDomainQueries={LiteralChordDomainQueries} " +
                "completeCommandChordProof=True wallDepthTolerance=0 coordinateBasis=float-F3");
            _token.ThrowIfCancellationRequested();
            _capturedArtworkProof = preparedArtworkProof;
            return result;
        }
    }

    /// <summary>
    /// Cutter-tip ceiling for the continuous medial floor. For a floor cone
    /// d(q)+m|x-q|, cutter support gives G(r)=min_s(Bottom(s)+m|r-s|).
    /// Radially increasing profiles need only 0&lt;=s&lt;=r. G is nondecreasing and
    /// m-Lipschitz, so weighted bounding boxes certify continuous graph edges.
    /// </summary>
    internal sealed class FloorField
    {
        private readonly record struct Segment(Point3D A, Point3D B,double EndpointDrop=double.PositiveInfinity);
        private enum SourceNodeKind { Original, General, Endpoint }
        private readonly record struct EndpointSource(Point3D Point,double Drop);
        private sealed class Node
        {
            internal double MinX, MinY, MaxX, MaxY, MinDepth;
            internal double MaximumDrop;
            internal FloorDirectionalBounds Directional;
            internal SourceNodeKind Kind;
            internal int First, Count;
            internal Node? Left, Right;
        }
        private readonly record struct Work(Node? Tree, int Segment, double Lo, double Hi,bool EndpointCertificate=false);
        private readonly Segment[] _segments;
        private readonly Lazy<PolygonOps.BoundarySegmentIndex>? _crestChordIndex;
        private long _crestChordQueries, _crestChordAccepted;
        private readonly Lazy<Node> _root;
        private readonly int[] _originalSources;
        private readonly Node? _generalRoot,_endpointRoot;
        private readonly int[] _generalSources=Array.Empty<int>();
        private readonly EndpointSource[] _endpointSources=Array.Empty<EndpointSource>();
        private const double EndpointCertificateMaximumDrop=9e-8;
        private readonly VCarveCutterProfile _profile;
        private readonly CarvingCutterProfile _physical;
        private readonly double _grade, _maximumDepth, _globalSupport;
        private readonly bool _boundedCurvature;
        private readonly bool _directionalBoundsEnabled;
        private readonly CancellationToken _cancellationToken;
        private readonly Dictionary<(double, double), double> _cache = new();
        private readonly IReadOnlyDictionary<(double,double),double>? _sharedCache;
        // Nomination witnesses share the immutable exact-XY field cache. They
        // do not alter its physical depth answers or authorize stock/travel.
        private readonly Dictionary<(double,double),Point3D> _contactSources=new();
        private readonly IReadOnlyDictionary<(double,double),Point3D>? _sharedContacts;
        private int _nominationPrewarmRemaining=1_000_000;
        private long _prewarmQueries,_prewarmBatches;
        private long _prewarmBudgetRefusals;
        private readonly HashSet<(double,double)> _prewarmRefused=new();
        internal int CachedQueryCount => _cache.Count;
        internal bool TryEvaluatedDepth(double x,double y,out double depth)=>_cache.TryGetValue((x,y),out depth);
        private readonly record struct SupportSectionConstants(double FixedRadial, double FixedBottom, double Slope);
        private const int MaximumSupportConstantSections = 4096;
        private readonly SupportSectionConstants[]? _supportConstants;
        private readonly double _cachedSphereTransition, _cachedCurvatureBound;
        private readonly long _supportPreparationTicks, _supportRetainedBytes;
        private readonly string _supportPreparationReason;
        private long _supportCalls, _supportSphereBottomReuses, _supportSphereBottomFresh, _supportLinearSlopeReuses;
        private readonly SupportDominanceProof.Owner? _supportDominance;
        private readonly string _supportDominanceReason;
        private long _supportDominatedCalls, _supportDominanceDomainFallbacks;
        internal string SupportDominanceDiagnostics =>
            $"supportLaterSectionsDominated={_supportDominance is not null} supportDominanceReason={_supportDominanceReason} " +
            $"supportDominanceRetainedBytesBound={_supportDominance?.RetainedBytesBound ?? 32} supportDominanceUniformRoundoff={SupportDominanceProof.UniformRoundoffBound:R} supportDominanceProofRoundoff={_supportDominance?.RoundoffBound??0:R} " +
            $"supportDominatedCalls={_supportDominatedCalls} supportDominanceDomainFallbacks={_supportDominanceDomainFallbacks} " +
            "originalFirstSphereArithmetic=True originalAllSectionFallback=True perSupportStopwatch=False";


        internal string SupportConstantDiagnostics =>
            $"supportConstantsPrepared={_supportConstants is not null} supportConstantsReason={_supportPreparationReason} " +
            $"supportConstantsSections={_supportConstants?.Length ?? 0} supportConstantsRetainedBytesBound={_supportRetainedBytes} " +
            $"supportConstantsPreparationMs={_supportPreparationTicks * 1000d / Stopwatch.Frequency:F3} " +
            $"supportCalls={_supportCalls} supportSphereBottomReuses={_supportSphereBottomReuses} " +
            $"supportSphereBottomFresh={_supportSphereBottomFresh} supportLinearSlopeReuses={_supportLinearSlopeReuses} " +
            "immutableProfileSnapshot=True allAuthoredSectionsRetained=True originalArithmeticOrder=True supportCallStopwatch=False";

        // Traversal scratch is generation-owned; cached physical answers remain
        // in the original exact-XY dictionary. Clear preserves the original
        // empty heap insertion/dequeue sequence, including equal-priority ties.
        private const int MaximumRetainedFloorQueueCapacity = 8192;
        private PriorityQueue<Work, double>? _queryScratchQueue;
        private int _queryScratchInUse;
        private long _queryCacheHits, _queryMisses, _queryQueueCreated, _queryQueueReused;
        private long _queryQueueLiteralFallbacks, _queryQueueOversizeDrops, _queryQueueTicks;
        private long _sourcePoseTrials, _sourcePoseImprovements;
        private int _queryQueueMaximumCapacity, _queryQueueRetainedCapacity;
        private long _sourceDualBounds;
        private long _treePartitionVisits;
        private long _directionalNodeBounds,_directionalNodeRejects;
        private long _sourceEndpointBounds;
        private readonly int _preparedEndpointSources;
        private readonly int _representedEndpointSources;
        private long _endpointTreeQueries,_endpointTreeFallbacks;
        internal string QueryScratchDiagnostics =>
            $"crestChordQueries={_crestChordQueries} crestChordAccepted={_crestChordAccepted} " +
            $"floorQueryCacheHits={_queryCacheHits} floorQueryMisses={_queryMisses} " +
            $"floorQueueCreated={_queryQueueCreated} floorQueueReused={_queryQueueReused} " +
            $"floorQueueLiteralFallbacks={_queryQueueLiteralFallbacks} floorQueueOversizeDrops={_queryQueueOversizeDrops} " +
            $"floorQueueMaximumCapacity={_queryQueueMaximumCapacity} floorQueueRetainedCapacity={_queryQueueRetainedCapacity} " +
            $"floorQueueMaximumRetainedCapacity={MaximumRetainedFloorQueueCapacity} " +
            $"floorQueryTraversalMs={_queryQueueTicks * 1000d / Stopwatch.Frequency:F3} " +
            $"floorSourcePoseTrials={_sourcePoseTrials} floorSourcePoseImprovements={_sourcePoseImprovements} floorSourceDualBounds={_sourceDualBounds} " +
            $"floorEndpointSources={_preparedEndpointSources} floorEndpointBounds={_sourceEndpointBounds} " +
            $"floorEndpointRepresentedSources={_representedEndpointSources} floorEndpointCertificates={_endpointSources.Length} floorEndpointTreeQueries={_endpointTreeQueries} floorEndpointTreeFallbacks={_endpointTreeFallbacks} " +
            $"floorOriginalSourceCount={_segments.Length} floorOriginalTreePrepared={_root.IsValueCreated} " +
            $"floorTreePartitionVisits={_treePartitionVisits} " +
            $"floorDirectionalNodeBounds={_directionalNodeBounds} floorDirectionalNodeRejects={_directionalNodeRejects} " +
            $"floorPrewarmQueries={_prewarmQueries} floorPrewarmBatches={_prewarmBatches} floorPrewarmBudgetRefusals={_prewarmBudgetRefusals} floorQueryTiming=sum-owned-query-wall " +
            "floorScratchIsNotPhysicalProofCache=True originalQueryVisitLimit=True priorityOrder=certified-lower-bound";

        internal FloorField(IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile,
            double grade, double maximumDepth, CancellationToken cancellationToken = default, bool supportConstantsEnabled = true,
            bool endpointCertificatesEnabled=true,bool directionalBoundsEnabled=true)
        {
            _profile = profile; _grade = grade; _maximumDepth = maximumDepth; _cancellationToken = cancellationToken;
            _physical = new CarvingCutterProfile(profile.Sections, profile.MaximumDepth);
            _globalSupport = _physical.Support(grade);
            _boundedCurvature = profile.RoundedTip && profile.Sections.All(section =>
                section.Kind == CarvingCutterSectionKind.Hemisphere || section.TopRadius == section.BottomRadius
                || section.Height / (section.TopRadius - section.BottomRadius) >= grade);
            _segments = paths.SelectMany(path => path.Points.Zip(path.Points.Skip(1), (a, b) => new Segment(a, b))).ToArray();
            if (_segments.All(s => s.A.Z == 0 && s.B.Z == 0 && CrestDomain(s.A) && CrestDomain(s.B)))
                _crestChordIndex = new(() => new PolygonOps.BoundarySegmentIndex(new Paths64(_segments.Select(s =>
                    new Path64 { CrestPoint(s.A), CrestPoint(s.B) }))));
            if (_segments.Length == 0) throw new ArgumentException("A smooth floor requires a medial cutting segment.");
            // The endpoint/general trees own every source in the admitted
            // query domain. Keep the complete original tree available without
            // eagerly sorting/building its million unchanged source intervals.
            // Its private ordinal array is published by Lazy only after the
            // complete original build; shared source records never reorder.
            _originalSources=Enumerable.Range(0,_segments.Length).ToArray();
            _root=new Lazy<Node>(()=>BuildNode(0,_segments.Length),LazyThreadSafetyMode.ExecutionAndPublication);
            // Profile sections are an immutable read-only copied snapshot. The
            // original all-section scan remains; only query-invariant operations
            // move here. Unsupported numeric/large profiles retain literal work.
            long supportPrepared = Stopwatch.GetTimestamp();
            _supportPreparationReason = "disabled";
            _supportRetainedBytes = 96; // Added owner fields plus conservative alignment bound.
            if (supportConstantsEnabled && profile.Sections.Count <= MaximumSupportConstantSections && double.IsFinite(grade))
            {
                var constants = new SupportSectionConstants[profile.Sections.Count];
                bool finite = true;
                for (int i = 0; i < constants.Length; i++)
                {
                    var section = profile.Sections[i];
                    if (section.Kind == CarvingCutterSectionKind.Hemisphere)
                    {
                        double radius = section.TopRadius;
                        double fixedRadial = radius * _grade / Math.Sqrt(1 + _grade * _grade);
                        double fixedBottom = fixedRadial * fixedRadial
                            / (radius + Math.Sqrt(Math.Max(0, radius * radius - fixedRadial * fixedRadial)));
                        constants[i] = new(fixedRadial, fixedBottom, 0);
                        finite &= double.IsFinite(fixedRadial) && double.IsFinite(fixedBottom);
                    }
                    else
                    {
                        double slope = section.TopRadius > section.BottomRadius
                            ? section.Height / (section.TopRadius - section.BottomRadius) : double.PositiveInfinity;
                        constants[i] = new(0, 0, slope);
                        finite &= !double.IsNaN(slope);
                    }
                }
                if (finite)
                {
                    _supportConstants = constants;
                    _supportRetainedBytes = 128L + 24L * constants.Length; // Fields, array header/alignment, and three doubles per section.
                    if (profile.RoundedTip)
                    {
                        double radius = profile.Sections[0].TopRadius;
                        _cachedSphereTransition = radius * (1 - 1 / Math.Sqrt(1 + _grade * _grade));
                    }
                    _cachedCurvatureBound = _boundedCurvature
                        ? Math.Pow(1 + _grade * _grade, 1.5) / profile.Sections[0].TopRadius
                        : double.PositiveInfinity;
                    _supportPreparationReason = "immutable-section-constants";
                }
                else _supportPreparationReason = "numeric-literal-fallback";
            }
            else if (supportConstantsEnabled) _supportPreparationReason = "numeric-or-section-quota-literal-fallback";
            SupportDominanceProof.TryCreate(profile, grade, out _supportDominance, out _supportDominanceReason);
            _directionalBoundsEnabled=directionalBoundsEnabled&&_supportDominance is not null&&grade is >0 and <=4
                &&profile.Sections[0].TopRadius is >=.1 and <=100;
            if(endpointCertificatesEnabled&&_supportDominance is not null&&profile.Sections[0].TopRadius is >=.1 and <=100)
            for(int i=0;i<_segments.Length;i++)
            {
                if((i&4095)==0)cancellationToken.ThrowIfCancellationRequested();
                var s=_segments[i];
                if(RoundedFloorSourceLowerBound.TryEndpointDrop(s.A,s.B,grade,out double drop)&&drop<=FieldTolerance)
                {_segments[i]=s with{EndpointDrop=drop};_preparedEndpointSources++;}
            }
            if(_preparedEndpointSources>0)
            {
                var certificates=new Dictionary<Point3D,double>();var general=new List<int>();
                for(int i=0;i<_segments.Length;i++)
                {
                    if((i&4095)==0)cancellationToken.ThrowIfCancellationRequested();
                    var s=_segments[i];
                    if(s.EndpointDrop>EndpointCertificateMaximumDrop){general.Add(i);continue;}
                    // This is an ACTUAL original affine endpoint. One lower
                    // certificate covers every complete original source sharing
                    // it; retain the largest drop, never a best-case mixture.
                    var point=At(s,s.A.Z>=s.B.Z?0:1);_representedEndpointSources++;
                    if(!certificates.TryGetValue(point,out double old)||s.EndpointDrop>old)certificates[point]=s.EndpointDrop;
                }
                _endpointSources=certificates.Select(p=>new EndpointSource(p.Key,p.Value)).ToArray();
                _generalSources=general.ToArray();
                if(_endpointSources.Length>0)
                {
                    _endpointRoot=BuildNode(0,_endpointSources.Length,SourceNodeKind.Endpoint);
                    if(_generalSources.Length>0)_generalRoot=BuildNode(0,_generalSources.Length,SourceNodeKind.General);
                }
            }
            _supportPreparationTicks = Stopwatch.GetTimestamp() - supportPrepared;

        }

        // Only immutable geometry/support owners are shared. Each worker owns
        // its dictionaries, heap and counters. The root dictionaries remain
        // read-only until Parallel.For joins every owned query worker.
        private FloorField(FloorField source)
        {
            _segments=source._segments;_root=source._root;_generalRoot=source._generalRoot;_endpointRoot=source._endpointRoot;
            _crestChordIndex=source._crestChordIndex;
            _originalSources=source._originalSources;
            _treePartitionVisits=source._treePartitionVisits;
            _generalSources=source._generalSources;_endpointSources=source._endpointSources;
            _profile=source._profile;_physical=source._physical;_grade=source._grade;_maximumDepth=source._maximumDepth;
            _globalSupport=source._globalSupport;_boundedCurvature=source._boundedCurvature;_cancellationToken=source._cancellationToken;
            _directionalBoundsEnabled=source._directionalBoundsEnabled;
            _supportConstants=source._supportConstants;_cachedSphereTransition=source._cachedSphereTransition;
            _cachedCurvatureBound=source._cachedCurvatureBound;_supportRetainedBytes=source._supportRetainedBytes;
            _supportPreparationReason="shared-immutable-query-constants";_supportDominance=source._supportDominance;
            _supportDominanceReason=source._supportDominanceReason;_preparedEndpointSources=source._preparedEndpointSources;
            _representedEndpointSources=source._representedEndpointSources;_sharedCache=source._cache;_sharedContacts=source._contactSources;
        }

        internal void PrewarmNomination(IEnumerable<PointD> points,Action<string>? diagnostic)
        {
            if(_nominationPrewarmRemaining<=0)return;
            _nominationPrewarmRemaining-=PrewarmQueries(points,_nominationPrewarmRemaining,diagnostic,"rest-nomination");
        }

        private void MergeQueryWorker(FloorField worker,Action<string>? diagnostic,string scope)
        {
            if(worker._prewarmBudgetRefusals>0)
            {
                _prewarmBudgetRefusals+=worker._prewarmBudgetRefusals;
                foreach(var refused in worker._prewarmRefused)
                    if(_prewarmRefused.Count<4096)_prewarmRefused.Add(refused);
                diagnostic?.Invoke($"stage=smooth-floor event=preparation-refusal scope={scope} queries={worker._prewarmBudgetRefusals} "
                    +$"example={worker._prewarmRefused.FirstOrDefault()} cachedAnswer=False originalLaterQueryRequired=True originalBudgetUnchanged=True");
            }
            foreach(var entry in worker._cache)
            {
                if(_cache.TryGetValue(entry.Key,out double old)&&old!=entry.Value)
                    throw new InvalidOperationException($"Independent floor query workers disagreed at {entry.Key}: {old:R} versus {entry.Value:R}.");
                _cache.TryAdd(entry.Key,entry.Value);
            }
            foreach(var entry in worker._contactSources)_contactSources.TryAdd(entry.Key,entry.Value);
            _queryCacheHits+=worker._queryCacheHits;_queryMisses+=worker._queryMisses;
            _queryQueueCreated+=worker._queryQueueCreated;_queryQueueReused+=worker._queryQueueReused;
            _queryQueueLiteralFallbacks+=worker._queryQueueLiteralFallbacks;_queryQueueOversizeDrops+=worker._queryQueueOversizeDrops;
            _queryQueueTicks+=worker._queryQueueTicks;_queryQueueMaximumCapacity=Math.Max(_queryQueueMaximumCapacity,worker._queryQueueMaximumCapacity);
            _sourcePoseTrials+=worker._sourcePoseTrials;_sourcePoseImprovements+=worker._sourcePoseImprovements;_sourceDualBounds+=worker._sourceDualBounds;
            _sourceEndpointBounds+=worker._sourceEndpointBounds;_endpointTreeQueries+=worker._endpointTreeQueries;_endpointTreeFallbacks+=worker._endpointTreeFallbacks;
            _directionalNodeBounds+=worker._directionalNodeBounds;_directionalNodeRejects+=worker._directionalNodeRejects;
            _supportCalls+=worker._supportCalls;_supportSphereBottomReuses+=worker._supportSphereBottomReuses;_supportSphereBottomFresh+=worker._supportSphereBottomFresh;
            _supportLinearSlopeReuses+=worker._supportLinearSlopeReuses;_supportDominatedCalls+=worker._supportDominatedCalls;_supportDominanceDomainFallbacks+=worker._supportDominanceDomainFallbacks;
        }

        internal List<TaggedToolpath> ConstrainOwned(IReadOnlyList<TaggedToolpath> paths,double grade,
            CancellationToken token,long maximumPoints,Action<string>? diagnostic)
        {
            if(Volatile.Read(ref _queryScratchInUse)!=0)
                throw new InvalidOperationException("Parallel floor refinement requires its sequential query owner to be idle.");
            long started=Stopwatch.GetTimestamp(),pointCount=0;
            int count=Math.Min(4,GeometryParallelism.WorkerLimit);
            var workers=Enumerable.Range(0,count).Select(_=>new FloorField(this)).ToArray();
            var result=new TaggedToolpath[paths.Count];
            void Charge(Point3D a,Point3D b)
            {
                if(Interlocked.Increment(ref pointCount)>maximumPoints)
                    throw new InvalidOperationException($"Smooth V-carving floor proof exceeds {maximumPoints} points at "
                        +$"({a.X:R},{a.Y:R})-({b.X:R},{b.Y:R}); angle grade={grade:R}.");
            }
            try
            {
                Parallel.For(0,count,new ParallelOptions{MaxDegreeOfParallelism=count,CancellationToken=token},worker=>{
                    using var owned=GeometryParallelism.EnterWorker();
                    var ordinals=Enumerable.Range(0,paths.Count).Where(i=>i%count==worker).ToArray();
                    var refined=ConstrainCore(ordinals.Select(i=>paths[i]).ToArray(),workers[worker],grade,token,maximumPoints,Charge);
                    if(refined.Count!=ordinals.Length)
                        throw new InvalidOperationException("Parallel floor refinement changed original path ordinals.");
                    for(int i=0;i<ordinals.Length;i++)result[ordinals[i]]=refined[i];
                });
            }
            catch(AggregateException error)
            {
                var errors=error.Flatten().InnerExceptions;
                System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(errors[0]).Throw();throw;
            }
            // No query owner or published path is mutated until every worker
            // joins. One atomic counter enforces the ORIGINAL aggregate budget.
            foreach(var worker in workers)MergeQueryWorker(worker,diagnostic,"floor-refinement");
            diagnostic?.Invoke($"stage=smooth-floor event=refined-paths paths={paths.Count} chargedPoints={pointCount} workers={count} "
                +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} originalPathOrder=True "
                +"privateQueryState=True originalAggregatePointBudget=True originalSubdivision=True");
            return result.ToList();
        }

        internal int PrewarmQueries(IEnumerable<PointD> points,long maximumNewQueries,Action<string>? diagnostic=null,string scope="query-preparation")
        {
            if(GeometryParallelism.WorkerLimit<=1||maximumNewQueries<1024)return 0;
            if(Volatile.Read(ref _queryScratchInUse)!=0)throw new InvalidOperationException("Floor query preparation requires its sequential owner to be idle.");
            const int batchLimit=65536;int total=0;long started=Stopwatch.GetTimestamp();
            var batch=new HashSet<(double X,double Y)>();
            foreach(var p in points)
            {
                _cancellationToken.ThrowIfCancellationRequested();
                if(!double.IsFinite(p.x+p.y)||_cache.ContainsKey((p.x,p.y))||_prewarmRefused.Contains((p.x,p.y))||!batch.Add((p.x,p.y)))continue;
                if(batch.Count==batchLimit||total+batch.Count>=maximumNewQueries){Flush();if(total>=maximumNewQueries)break;}
            }
            Flush();
            if(total>0)diagnostic?.Invoke($"stage=smooth-floor event=prepared-queries scope={scope} queries={total} workers={Math.Min(8,GeometryParallelism.WorkerLimit)} "
                +$"batchLimit={batchLimit} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} immutableSourceShared=True privateQueryState=True originalContourOrder=True proofBudgetsUnchanged=True");
            return total;
            void Flush()
            {
                if(batch.Count<1024){batch.Clear();return;}
                var inputs=batch.ToArray();int count=Math.Min(8,GeometryParallelism.WorkerLimit);
                var workers=Enumerable.Range(0,count).Select(_=>new FloorField(this)).ToArray();
                Parallel.For(0,count,new ParallelOptions{MaxDegreeOfParallelism=count,CancellationToken=_cancellationToken},worker=>{
                    using var owned=GeometryParallelism.EnterWorker();
                    int first=(int)((long)inputs.Length*worker/count),last=(int)((long)inputs.Length*(worker+1)/count);
                    for(int i=first;i<last;i++)
                    {
                        try{workers[worker].Depth(inputs[i].X,inputs[i].Y);}
                        catch(InvalidOperationException error)when(IsActualF3FloorProofBudgetRefusal(error))
                        {
                            // Optional cache preparation grants no answer on
                            // refusal. A real later query still runs and fails
                            // visibly under its unchanged original budget.
                            workers[worker]._prewarmBudgetRefusals++;
                            if(workers[worker]._prewarmRefused.Count<4096)workers[worker]._prewarmRefused.Add(inputs[i]);
                        }
                    }
                });
                // All readers have joined. Every value came from the SAME
                // immutable source tree and unchanged literal query kernel.
                foreach(var worker in workers)MergeQueryWorker(worker,diagnostic,scope);
                total+=inputs.Length;_prewarmQueries+=inputs.Length;_prewarmBatches++;batch.Clear();
            }
        }

        internal double SupportDistance(double distance)
            =>SupportDistance(distance,out _);

        private double SupportDistance(double distance,out double contactRadius)
        {
            contactRadius=0;_supportCalls++;
            if(distance<=_profile.TipRadius){contactRadius=distance;return 0;}
            double best = _grade * distance, height = 0;
            int sectionIndex = 0;
            foreach (CarvingCutterSection section in _physical.Sections)
            {
                if (section.BottomRadius > distance) break;
                var prepared = _supportConstants;
                int index = sectionIndex++;
                if (section.Kind == CarvingCutterSectionKind.Hemisphere)
                {
                    double radius = section.TopRadius;
                    double fixedRadial = prepared is null ? radius * _grade / Math.Sqrt(1 + _grade * _grade)
                        : prepared[index].FixedRadial;
                    double radial = Math.Min(distance, fixedRadial);
                    double bottom;
                    if (prepared is not null && BitConverter.DoubleToInt64Bits(radial) == BitConverter.DoubleToInt64Bits(fixedRadial))
                    { bottom = prepared[index].FixedBottom; _supportSphereBottomReuses++; }
                    else
                    { bottom = radial * radial / (radius + Math.Sqrt(Math.Max(0, radius * radius - radial * radial))); _supportSphereBottomFresh++; }
                    double value=height+bottom+_grade*(distance-radial);
                    if(value<best)contactRadius=radial;best=Math.Min(best,value);
                    if (index == 0 && _supportDominance is not null)
                    {
                        if (double.IsFinite(distance) && distance >= 0 && distance <= SupportDominanceProof.MaximumDistance)
                        { _supportDominatedCalls++; return Math.Max(0, best); }
                        _supportDominanceDomainFallbacks++;
                    }


                }
                else
                {
                    // The flat bottom includes every radius below BottomRadius.
                    if(height==0)
                    {
                        double flat=_grade*Math.Max(0,distance-section.BottomRadius);
                        if(flat<best)contactRadius=Math.Min(distance,section.BottomRadius);best=Math.Min(best,flat);
                    }
                    double hi = Math.Min(distance, section.TopRadius);
                    double slope = prepared is null
                        ? section.TopRadius > section.BottomRadius
                            ? section.Height / (section.TopRadius - section.BottomRadius) : double.PositiveInfinity
                        : prepared[index].Slope;
                    if (prepared is not null) _supportLinearSlopeReuses++;
                    double radial = slope < _grade ? hi : section.BottomRadius;
                    double bottom = radial <= section.BottomRadius ? 0 : (radial - section.BottomRadius) * slope;
                    double value=height+bottom+_grade*(distance-radial);
                    if(value<best)contactRadius=radial;best=Math.Min(best,value);
                }
                height += section.Height;
            }
            return Math.Max(0, best);
        }

        private static Point64 CrestPoint(Point3D point) => new(
            (long)Math.Round(point.X * PathUtils.Scale), (long)Math.Round(point.Y * PathUtils.Scale));

        private static bool CrestDomain(Point3D point) => double.IsFinite(point.X + point.Y)
            && Math.Max(Math.Abs(point.X), Math.Abs(point.Y)) <= 1_000_000;

        internal bool ProveCrestChord(Point3D a, Point3D b)
        {
            if (_crestChordIndex is null) return false;
            _cancellationToken.ThrowIfCancellationRequested();
            double requested = Math.Max(-a.Z, -b.Z);
            if (!double.IsFinite(requested) || requested > _maximumDepth
                || !CrestDomain(a) || !CrestDomain(b))
                return false;
            if (requested <= 0) return true;
            _crestChordQueries++;
            // For constant zero-height crests, the entire tip floor sublevel
            // is the crest buffered by the existing inverse physical support.
            // Clearance of the whole XY chord at its deepest endpoint therefore
            // proves every interpolated depth too. Inflate for BOTH source and
            // query integer rounding; a refusal uses the original chord proof.
            double radius = SmoothRoughingPlateau.ExclusionRadius(requested, _grade, _profile)
                + _profile.TipRadius;
            if (!double.IsFinite(radius) || radius < 0) return false;
            radius += 4 / PathUtils.Scale + 1e-10 * Math.Max(1, radius);
            bool accepted = _crestChordIndex.Value.ClearanceViolation(CrestPoint(a), CrestPoint(b),
                radius * PathUtils.Scale) is null;
            if (accepted) _crestChordAccepted++;
            return accepted;
        }

        internal double CappedDepth(double requested, double x, double y)
        {
            double ceiling = Depth(x, y);
            // A fixed axial reserve erases a much wider area at a spherical tip.
            // Bound its radial effect too; the complete-chord proof then adapts
            // to this smaller local reserve instead of giving up shallow detail.
            double radial = Math.Max(0, _profile.RadiusAtDepth(ceiling) - CoverageTolerance / 16);
            double reserve = Math.Min(DepthReserve, Math.Max(1e-9, ceiling - _physical.Bottom(radial)));
            return Math.Min(requested, Math.Max(0, ceiling - reserve));
        }

        /// <summary>The actual physical floor contact of a proposed cutter
        /// pose. Tip height is not finished-stock height on a ramp: the support
        /// kernel's active original source supplies the contact direction.</summary>
        internal Point3D NominateFloorContact(Point3D pose)
        {
            double ceiling=Depth(pose.X,pose.Y);
            if(ceiling>=_maximumDepth
                ||!(_contactSources.TryGetValue((pose.X,pose.Y),out var source)
                    ||_sharedContacts is not null&&_sharedContacts.TryGetValue((pose.X,pose.Y),out source)))return pose;
            double dx=pose.X-source.X,dy=pose.Y-source.Y,distance=Math.Sqrt(dx*dx+dy*dy);
            if(distance<=1e-12)return pose;
            SupportDistance(distance,out double contact);
            return new(pose.X-contact*dx/distance,pose.Y-contact*dy/distance,
                pose.Z+_physical.Bottom(contact));
        }

        internal double MaximumFloorContactRadius
        {
            get {SupportDistance(_physical.MaximumRadius,out double radial);return radial;}
        }

        internal double SublevelGrade(double ceiling)
        {
            // Only the field below the largest commanded depth can invalidate
            // this chord. In the spherical-tip sublevel, every active support
            // cone has the much smaller exact ball gradient. Using the global
            // ramp grade here would demand nanometer steps near surface tips.
            if (!_profile.RoundedTip || ceiling <= 0) return ceiling <= 0 ? 0 : _grade;
            double radius = _profile.Sections[0].TopRadius;
            double transition = _supportConstants is null ? radius * (1 - 1 / Math.Sqrt(1 + _grade * _grade)) : _cachedSphereTransition;
            if (ceiling >= transition) return _grade;
            return Math.Min(_grade, Math.Sqrt(Math.Max(0, ceiling * (2 * radius - ceiling))) / (radius - ceiling));
        }

        // The rounded support kernel has Hessian <=K: its spherical part reaches
        // gradient m at r=R*m/sqrt(1+m²), then the linear support has no larger
        // tangential curvature. Eligibility requires later flanks at least as
        // steep as m; shallower flanks can introduce a derivative kink and retain
        // the general Lipschitz proof. Taking the minimum over kernels preserves
        // K-semiconcavity, including the continuous source-segment minimum.
        // Consequently F along a chord is >=linear(F endpoints)-K*L²*t*(1-t)/2.
        internal double CurvatureBound => _supportConstants is not null ? _cachedCurvatureBound
            : _boundedCurvature
                ? Math.Pow(1 + _grade * _grade, 1.5) / _profile.Sections[0].TopRadius
                : double.PositiveInfinity;

        internal double Depth(double x, double y)
        {
            if (_cache.TryGetValue((x, y), out double cached)
                ||_sharedCache is not null&&_sharedCache.TryGetValue((x,y),out cached)) { _queryCacheHits++; return cached; }
            _cancellationToken.ThrowIfCancellationRequested();
            long queryStarted = Stopwatch.GetTimestamp();
            _queryMisses++;
            bool ownsScratch = Interlocked.CompareExchange(ref _queryScratchInUse, 1, 0) == 0;
            PriorityQueue<Work, double> queue;
            if (ownsScratch)
            {
                if (_queryScratchQueue is null) { _queryScratchQueue = new(); _queryQueueCreated++; }
                else _queryQueueReused++;
                queue = _queryScratchQueue;
            }
            else
            {
                // Existing callers are sequential and Depth invokes no caller
                // callbacks. A future reentrant query retains the literal fresh
                // heap rather than sharing an active traversal. The exact-XY
                // cache retains its original sequential-owner contract.
                queue = new(); _queryQueueCreated++; _queryQueueLiteralFallbacks++;
            }
            double best = _maximumDepth;
            var directionalQuery=default(FloorDirectionalBounds.Query);
            bool useDirectional=_directionalBoundsEnabled&&FloorDirectionalBounds.TryPrepare(x,y,_grade,
                _profile.Sections[0].TopRadius,out directionalQuery);
            Point3D? contactSource=null;
            try
            {
                if(_endpointRoot is not null&&Math.Abs(x)<=1024&&Math.Abs(y)<=1024)
                {
                    _endpointTreeQueries++;EnqueueNode(_endpointRoot);
                    if(_generalRoot is not null)EnqueueNode(_generalRoot);
                }
                else
                {
                    if(_endpointRoot is not null)_endpointTreeFallbacks++;
                    EnqueueNode(_root.Value);
                }
                int visits = 0;
                while (queue.TryDequeue(out Work work, out double lower))
                {
                    if ((++visits & 255) == 0) _cancellationToken.ThrowIfCancellationRequested();
                    double localTolerance = best < .01 ? 1e-7 : FieldTolerance;
                    if (lower >= best - localTolerance)
                    {
                        best = Math.Min(best, lower); break;
                    }
                    if (work.Tree is { } node)
                    {
                        if (node.Left is not null) { EnqueueNode(node.Left); EnqueueNode(node.Right!); }
                        else for (int i = node.First; i < node.First + node.Count; i++)
                        {
                            if(node.Kind==SourceNodeKind.Endpoint)EnqueueEndpoint(i);
                            else EnqueueSegment(node.Kind==SourceNodeKind.General?_generalSources[i]:_originalSources[i],0,1);
                        }
                    }
                    else
                    {
                        if(work.EndpointCertificate)throw new InvalidOperationException("Endpoint source certificate exceeded the original floor-query tolerance.");
                        double mid = (work.Lo + work.Hi) / 2;
                        EnqueueSegment(work.Segment, work.Lo, mid);
                        EnqueueSegment(work.Segment, mid, work.Hi);
                    }
                    if (visits > 100_000)
                        throw new InvalidOperationException($"Smooth floor query exceeded its proof budget at ({x:R},{y:R}), best={best:R}, lower={lower:R}.");
                }
                double answer = Math.Max(0, best);
                _cache[(x, y)] = answer;
                if(contactSource is { } source)_contactSources[(x,y)]=source;
                return answer;
            }
            finally
            {
                // EnsureCapacity(0) observes the current backing capacity; it
                // does not grow or alter the remaining traversal heap.
                int capacity = queue.EnsureCapacity(0);
                _queryQueueMaximumCapacity = Math.Max(_queryQueueMaximumCapacity, capacity);
                queue.Clear();
                if (ownsScratch)
                {
                    if (capacity > MaximumRetainedFloorQueueCapacity)
                    {
                        _queryScratchQueue = null;
                        _queryQueueRetainedCapacity = 0;
                        _queryQueueOversizeDrops++;
                    }
                    else _queryQueueRetainedCapacity = capacity;
                    Volatile.Write(ref _queryScratchInUse, 0);
                }
                _queryQueueTicks += Stopwatch.GetTimestamp() - queryStarted;
            }

            void EnqueueNode(Node node)
            {
                double correlated=double.NegativeInfinity;
                if(useDirectional&&node.Directional.Eligible)
                {
                    correlated=node.Directional.Lower(directionalQuery);
                    if(node.Kind==SourceNodeKind.Endpoint)correlated=Math.BitDecrement(correlated-node.MaximumDrop);
                    _directionalNodeBounds++;
                    if(correlated>=best){_directionalNodeRejects++;return;}
                }
                double dx = Math.Max(0, Math.Max(node.MinX - x, x - node.MaxX));
                double dy = Math.Max(0, Math.Max(node.MinY - y, y - node.MaxY));
                double lower = node.MinDepth + SupportDistance(Math.Sqrt(dx * dx + dy * dy));
                if(node.Kind==SourceNodeKind.Endpoint)lower=Math.BitDecrement(lower-node.MaximumDrop);
                lower=Math.Max(lower,correlated);
                if (lower < best) queue.Enqueue(new Work(node, 0, 0, 0), lower);
            }
            void EnqueueEndpoint(int index)
            {
                var certificate=_endpointSources[index];var p=certificate.Point;
                double value=-p.Z+SupportDistance(Math.Sqrt((p.X-x)*(p.X-x)+(p.Y-y)*(p.Y-y)));
                if(value<best){best=value;contactSource=p;}
                double lower=Math.BitDecrement(value-certificate.Drop);_sourceEndpointBounds++;
                if(lower<best)queue.Enqueue(new Work(null,index,0,1,EndpointCertificate:true),lower);
            }
            void EnqueueSegment(int index, double lo, double hi)
            {
                Segment segment = _segments[index];
                Point3D a = At(segment, lo), b = At(segment, hi);
                double dx = b.X - a.X, dy = b.Y - a.Y, square = dx * dx + dy * dy;
                double t = square > 0 ? Math.Clamp(((x - a.X) * dx + (y - a.Y) * dy) / square, 0, 1) : 0;
                double qx = a.X + t * dx, qy = a.Y + t * dy;
                double nearest = Math.Sqrt((qx - x) * (qx - x) + (qy - y) * (qy - y));
                double sample=-(a.Z+t*(b.Z-a.Z))+SupportDistance(nearest);
                if(sample<best)contactSource=new(qx,qy,a.Z+t*(b.Z-a.Z));best=Math.Min(best,sample);
                double sampleA=sample=-a.Z+SupportDistance(Math.Sqrt((a.X-x)*(a.X-x)+(a.Y-y)*(a.Y-y)));
                if(sample<best)contactSource=a;best=Math.Min(best,sample);
                double sampleB=sample=-b.Z+SupportDistance(Math.Sqrt((b.X-x)*(b.X-x)+(b.Y-y)*(b.Y-y)));
                if(sample<best)contactSource=b;best=Math.Min(best,sample);
                double lower = Math.Min(-a.Z, -b.Z) + SupportDistance(nearest);
                if(lo==0&&hi==1&&double.IsFinite(segment.EndpointDrop)&&Math.Abs(x)<=1024&&Math.Abs(y)<=1024)
                {
                    _sourceEndpointBounds++;
                    double endpoint=segment.A.Z>=segment.B.Z?sampleA:sampleB;
                    lower=Math.Max(lower,Math.BitDecrement(endpoint-segment.EndpointDrop));
                    // The existing queue's unchanged tolerance can already
                    // close this COMPLETE source interval. Keep its lower
                    // certificate in that queue; no intermediate poses vanish.
                    double tolerance=best<.01?1e-7:FieldTolerance;
                    if(lower>=best-tolerance)
                    {
                        if(lower<best)queue.Enqueue(new Work(null,index,lo,hi),lower);
                        return;
                    }
                }
                if (square > 1e-24)
                {
                    // G(r)>=m*r-Support(m). Keep source depth correlated with
                    // distance along a sloped segment; separate minima become
                    // arbitrarily loose on a long ramp whose grade equals m.
                    double length = Math.Sqrt(square), k = (a.Z - b.Z) / length;
                    double along = ((x - a.X) * dx + (y - a.Y) * dy) / length;
                    double across = Math.Abs((x - a.X) * dy - (y - a.Y) * dx) / length;
                    double at = k >= _grade ? 0 : k <= -_grade ? length
                        : Math.Clamp(along - k * across / Math.Sqrt(Math.Max(1e-30, _grade * _grade - k * k)), 0, length);
                    double weighted = -a.Z + k * at + _grade * Math.Sqrt((at - along) * (at - along) + across * across);
                    lower = Math.Max(lower, weighted - _globalSupport);
                    // Nominate the real source pose attaining the conical
                    // lower support bound. The original lower bounds and
                    // priority traversal still certify the minimum; these
                    // actual poses only tighten its upper bracket.
                    if(k!=0)ObserveSourcePose(at);
                    if(k!=0 && _profile.RoundedTip && across<_profile.Sections[0].TopRadius)
                    {
                        double r=_profile.Sections[0].TopRadius;
                        double sphere=along-k*Math.Sqrt(Math.Max(0,(r*r-across*across)/(1+k*k)));
                        ObserveSourcePose(Math.Clamp(sphere,0,length));
                    }
                    void ObserveSourcePose(double distanceAlong)
                    {
                        _sourcePoseTrials++;
                        var pose=At(segment,lo+(hi-lo)*(distanceAlong/length));
                        double value=-pose.Z+SupportDistance(Math.Sqrt((pose.X-x)*(pose.X-x)+(pose.Y-y)*(pose.Y-y)));
                        if(value<best){best=value;contactSource=pose;_sourcePoseImprovements++;}
                        // Most short original source intervals already close
                        // under the original queue's tolerance. Pay for the
                        // outward dual certificate only after that literal
                        // interval actually required subdivision.
                        if(hi-lo<1 && lower<best && _supportDominance is not null && RoundedFloorSourceLowerBound.TryLower(segment.A,segment.B,lo,hi,
                            x,y,_profile.Sections[0].TopRadius,_grade,pose,out double dualLower))
                        {lower=Math.Max(lower,dualLower);_sourceDualBounds++;}
                    }
                }
                if (lower < best) queue.Enqueue(new Work(null, index, lo, hi), lower);
            }
        }

        private static Point3D At(Segment segment, double t) => new(segment.A.X + (segment.B.X - segment.A.X) * t,
            segment.A.Y + (segment.B.Y - segment.A.Y) * t, segment.A.Z + (segment.B.Z - segment.A.Z) * t);

        private Node BuildNode(int first, int count,SourceNodeKind kind=SourceNodeKind.Original)
        {
            _cancellationToken.ThrowIfCancellationRequested();
            var node = new Node { First = first, Count = count, MinX = double.PositiveInfinity,
                MinY = double.PositiveInfinity, MaxX = double.NegativeInfinity, MaxY = double.NegativeInfinity,
                MinDepth = double.PositiveInfinity,Kind=kind };
            for (int i = first; i < first + count; i++)
            {
                if(kind==SourceNodeKind.Endpoint)
                {
                    var certificate=_endpointSources[i];var p=certificate.Point;
                    node.MinX=Math.Min(node.MinX,p.X);node.MaxX=Math.Max(node.MaxX,p.X);
                    node.MinY=Math.Min(node.MinY,p.Y);node.MaxY=Math.Max(node.MaxY,p.Y);
                    node.MinDepth=Math.Min(node.MinDepth,-p.Z);node.MaximumDrop=Math.Max(node.MaximumDrop,certificate.Drop);
                    continue;
                }
                Segment segment = _segments[kind==SourceNodeKind.General?_generalSources[i]:_originalSources[i]];
                node.MinX = Math.Min(node.MinX, Math.Min(segment.A.X, segment.B.X));
                node.MinY = Math.Min(node.MinY, Math.Min(segment.A.Y, segment.B.Y));
                node.MaxX = Math.Max(node.MaxX, Math.Max(segment.A.X, segment.B.X));
                node.MaxY = Math.Max(node.MaxY, Math.Max(segment.A.Y, segment.B.Y));
                node.MinDepth = Math.Min(node.MinDepth, Math.Min(-segment.A.Z, -segment.B.Z));
            }
            if (count > 8)
            {
                bool x = node.MaxX - node.MinX >= node.MaxY - node.MinY;
                int Compare(Segment a,Segment b)=>(x?a.A.X+a.B.X:a.A.Y+a.B.Y).CompareTo(x?b.A.X+b.B.X:b.A.Y+b.B.Y);
                if(kind==SourceNodeKind.Endpoint)_treePartitionVisits+=SpatialMedianPartition.Partition(_endpointSources,first,count,
                    (a,b)=>(x?a.Point.X:a.Point.Y).CompareTo(x?b.Point.X:b.Point.Y),_cancellationToken);
                else _treePartitionVisits+=SpatialMedianPartition.Partition(kind==SourceNodeKind.General?_generalSources:_originalSources,
                    first,count,(a,b)=>Compare(_segments[a],_segments[b]),_cancellationToken);
                int half = count / 2;
                node.Left = BuildNode(first, half,kind); node.Right = BuildNode(first + half, count - half,kind);
                if(_directionalBoundsEnabled)node.Directional=FloorDirectionalBounds.Merge(node.Left.Directional,node.Right.Directional);
            }
            else if(_directionalBoundsEnabled)
            {
                var directional=FloorDirectionalBounds.Empty;
                for(int i=first;i<first+count;i++)
                {
                    if(kind==SourceNodeKind.Endpoint)directional.Add(_endpointSources[i].Point,_grade);
                    else
                    {
                        var s=_segments[kind==SourceNodeKind.General?_generalSources[i]:_originalSources[i]];
                        directional.Add(At(s,0),_grade);directional.Add(At(s,1),_grade);
                    }
                }
                node.Directional=directional;
            }
            return node;
        }
    }

    internal static double MaximumGrade(IEnumerable<TaggedToolpath> paths)
    {
        double maximum = 0;
        foreach (TaggedToolpath path in paths)
        for (int i = 1; i < path.Points.Count; i++)
        {
            double length = Distance(path.Points[i - 1], path.Points[i]);
            double delta = Math.Abs(path.Points[i].Z - path.Points[i - 1].Z);
            maximum = Math.Max(maximum, length > 1e-12 ? delta / length : delta > 1e-9 ? double.PositiveInfinity : 0);
        }
        return maximum;
    }

    private static double Distance(Point3D a, Point3D b)
        => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
    internal static List<TaggedToolpath> CommandPaths(IReadOnlyList<TaggedToolpath> paths, double surfaceZ)
    {
        return paths.Select(path =>
        {
            var points = new List<Point3D>(path.Points.Count);
            foreach (Point3D point in path.Points)
            {
                Point3D commanded = CommandPoint(point, surfaceZ);
                if (points.Count == 0 || points[^1] != commanded) points.Add(commanded);
            }
            return new TaggedToolpath(MedialAxisToolpaths.SimplifyCollinearRuns(points),
                Array.Empty<ToolpathSpan>(), path.RegionIndex, path.Category, path.DepthPassIndex,
                path.VisitIdentity, detectCurves: false);
        }).ToList();
    }

    private static Point3D CommandPoint(Point3D point, double surfaceZ)
    {
        static double F3(float value) => VCarveCommandRounding.Round(value);
        return new Point3D(F3((float)point.X), F3((float)point.Y),
            F3((float)(surfaceZ + point.Z) - (float)surfaceZ));
    }

    internal static double QuantizeCorrectionDepth(double maximumDepth, double surfaceZ)
    {
        // Concentric centers must use the radius that will actually be cut. A
        // fractional nominal depth paired with a shallower F3 command leaves a
        // systematic radial gap, especially near a spherical tip. Iterate to a
        // fixed point of the same float/workspace/F3 conversion used by emission.
        double depth = Math.Floor(maximumDepth * 1000 + 1e-9) / 1000;
        for (int attempt = 0; attempt < 64 && depth > 0; attempt++)
        {
            double commanded = -CommandPoint(new Point3D(0, 0, -depth), surfaceZ).Z;
            if (commanded == depth) return depth;
            depth = commanded < depth ? commanded
                : Math.Floor(Math.Min(depth - .001, 2 * depth - commanded) * 1000 + 1e-9) / 1000;
        }
        if (depth <= 0) return 0;
        throw new InvalidOperationException($"Smooth V-carving could not obtain a stable F3 compensation depth " +
            $"at surface {surfaceZ:R}, requested depth {maximumDepth:R}.");
    }

    private static void ReportCommandFloorBound(IReadOnlyList<TaggedToolpath> paths, FloorField field,
        double grade, double surfaceZ, Action<string>? diagnostic)
    {
        double xyError = 0, zError = 0, endpointExcess = 0;
        Point3D witness = default;
        foreach (TaggedToolpath path in paths)
        foreach (Point3D point in path.Points)
        {
            Point3D commanded = CommandPoint(point, surfaceZ);
            xyError = Math.Max(xyError, Distance(point, commanded));
            zError = Math.Max(zError, Math.Abs(point.Z - commanded.Z));
            double excess = -commanded.Z - field.Depth(commanded.X, commanded.Y);
            if (excess > endpointExcess) { endpointExcess = excess; witness = commanded; }
        }
        // Interpolated coordinate errors are bounded by their endpoint errors.
        // The physical tip field is m-Lipschitz, so this bounds the complete
        // commanded cutter-floor chord, not merely its sampled endpoints.
        double spatialFloorAllowance=0;
        double bound = zError + grade * xyError + spatialFloorAllowance;
        diagnostic?.Invoke($"stage=smooth-command-floor completeNominalChordProof=True " +
            $"maximumXYConversionError={xyError:R} maximumZConversionError={zError:R} " +
            $"completeCommandFloorExcessBound={bound:R} nominalSpatialFloorAllowance={spatialFloorAllowance:R} measuredEndpointExcess={endpointExcess:R} " +
            $"witness=({witness.X:R},{witness.Y:R},{witness.Z:R}) " +
            "basis=float-workspace-F3-stock-relative quantizationAllowanceExplicit=True " +
            field.QueryScratchDiagnostics + " " + field.SupportConstantDiagnostics + " " + field.SupportDominanceDiagnostics);
        if (endpointExcess > bound + FieldTolerance + 1e-8)
            throw new InvalidOperationException($"Smooth V-carving command floor audit exceeded its derived bound: " +
                $"endpoint={endpointExcess:R}, bound={bound:R}, witness=({witness.X:R},{witness.Y:R},{witness.Z:R}).");
    }
    private static double Area(IEnumerable<IReadOnlyList<PointD>> rings)
        => Math.Abs(rings.Sum(ring => Clipper.Area(new PathD(ring))));
    private static string Bounds(IReadOnlyList<List<PointD>> rings)
    {
        if (rings.Count == 0) return "empty";
        var points = rings.SelectMany(ring => ring).ToArray();
        return $"({points.Min(point => point.x):R},{points.Min(point => point.y):R})-" +
            $"({points.Max(point => point.x):R},{points.Max(point => point.y):R})";
    }
}
