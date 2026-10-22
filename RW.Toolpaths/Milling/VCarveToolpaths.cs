using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Configuration for medial-axis V-carving.</summary>
public sealed record VCarveOptions
{
    public MillingBoundaryAccess? BoundaryAccess { get; init; }
    internal MillingBoundaryAccess.Resolved? AccessTarget { get; init; }
    // Pocket-only preparation: a finite flat tip cannot enter a component
    // whose enclosing box cannot contain its contact disk. Never used by Detail.
    internal double PreparationContactRadius { get; init; }
    // Scheduling only: original graph, sample ordinals and scalar arithmetic
    // are unchanged. It is intentionally absent from geometry fingerprints.
    internal bool ParallelOriginalSourceSampling { get; init; }=true;
    internal bool ParallelFloorQueryPreparation { get; init; }=true;
    /// <summary>Limits cutting depth gradients and clears the resulting missed channel coverage.</summary>
    public bool SmoothVCarving { get; init; }

    /// <summary>Maximum cutting grade in degrees for smooth V-carving.</summary>
    public double SmoothRampAngleDegrees { get; init; } = 30;

    /// <summary>Retain smooth grade anchors at approved outside edges; defaults off.</summary>
    public bool RampOutsideEdges { get; init; }

    /// <summary>Entry angle used below a cleared starting floor or when legacy depth passes are skipped.</summary>
    public RampSettings Ramp { get; init; } = RampSettings.FromRatio(2);

    /// <summary>Material left at the boundary for a later finish pass.</summary>
    public double StockToLeave { get; init; }

    /// <summary>
    /// Fraction of the effective cutter diameter used for clearing passes. Rounded
    /// tips use the nose diameter, limited by the engaged diameter for shallow cuts.
    /// </summary>
    public double StepOver { get; init; } = 0.4;

    /// <summary>Medial-axis parabola discretisation tolerance in workspace units.</summary>
    public double Tolerance { get; init; } = 0.03;

    /// <summary>
    /// Round-arc nomination error for later smooth correction/discovery fill
    /// levels. Original first fronts and all physical stock proofs retain their
    /// existing precision. Inconclusive completion retries the original setting.
    /// </summary>
    public double CorrectionArcTolerance { get; init; } = 1 / PathUtils.Scale;

    /// <summary>
    /// Experimental ordinary-fill nomination for smooth correction rounds 0..6.
    /// Every candidate retains the original physical floor, wall and fixed-target
    /// completion proofs; round 7 uses the original terminal collector. Changed
    /// nominations may change finished scallops and require independent stock audit.
    /// </summary>
    public bool UseOrdinaryCorrectionCandidates { get; init; }

    /// <summary>Whether broad areas receive inward V-bit clearing paths.</summary>
    public bool IncludeInteriorFill { get; init; } = true;

    /// <summary>
    /// Inward offset from the original boundary at which additional fill centerlines
    /// stop when full clearing is disabled. Zero retains perimeter and medial detail only.
    /// The cutoff follows existing boundary normalization and stock-allowance offset
    /// tolerances rather than an exact Euclidean distance from the authored geometry.
    /// </summary>
    public double EdgeFillWidth { get; init; }

    /// <summary>Transient extra clearing reach requested by a matching rougher.
    /// Extends nominations without changing the authored band or analytical floor.</summary>
    public double RoughingRestWidth { get; init; }

    /// <summary>Actual preceding flat-tip sweeps used only to avoid redundant rest nominations.</summary>
    public IReadOnlyList<VCarveRoughingChord> RoughingRestChords { get; init; } = Array.Empty<VCarveRoughingChord>();
    /// <summary>Height-dependent actual prior material removal for ordinary finishing.
    /// StartingDepth remains an entry/pass floor; it is not a global geometry shift.</summary>
    public VCarveClearedStock? ClearedStock { get; init; }
    internal ClearedColumnStock? PriorStock { get; init; }
    internal VCarveOptions WithPriorStock(CancellationToken token) => PriorStock is not null || RoughingRestChords.Count == 0
        ? this : this with { PriorStock = new ClearedColumnStock(RoughingRestChords.Select(c =>
            new ClearedColumnStock.Column(c.A, c.B, c.ClearedColumnRadius)), token) };

    /// <summary>
    /// Selects the existing original-source dense first inset directly, avoiding
    /// a conditioned candidate and its comparison when the original is desired.
    /// Complete boundary-band construction, cleanup and command proof remain.
    /// </summary>
    public bool PreferOriginalInsetConstruction { get; init; }

    /// <summary>Fraction, work description and whether the current stage has no known total.</summary>
    public Action<double, string, bool>? Progress { get; init; }

    /// <summary>
    /// Fits optional arc/Bezier move metadata. Point-based consumers can disable this
    /// without changing cutter geometry, path routing or swept-area accounting.
    /// </summary>
    public bool DetectCurves { get; init; } = true;

    /// <summary>Builds optional planar swept-area metadata for consumers that need it.</summary>
    public bool CalculateSweptArea { get; init; } = true;

    /// <summary>
    /// Compacts directed subdivisions in native and float/F3 command space.
    /// Enable only when these are the final world coordinates: a subsequent
    /// float machine-placement transform can change command collinearity.
    /// </summary>
    public bool CompactCollinearCommands { get; init; }
    /// <summary>Transient host travel/cutting feed ratio for cleared-route cost.
    /// One retains geometric distance ordering when machine rates are unknown.</summary>
    public double ClearedTravelCostRatio {get;init;}=1;
    /// <summary>Host's actual global transfer plane and cut/rapid feed ratio.
    /// NaN means no transfer-time allowance for callers without machine context.</summary>
    public double ContinuationSafeZ {get;init;}=double.NaN;
    public double EntryTransferCostRatio {get;init;}=1;

    /// <summary>
    /// Experimental opt-in for avoiding optional terminal rays dominated by a
    /// protected mandatory visit. Final stock equivalence remains unverified:
    /// changing the candidate snapshot can change later optional witness selection.
    /// Leave disabled for finished-stock preservation. Experiments also require
    /// final world coordinates: later float placement can invalidate containment.
    /// </summary>
    public bool AvoidRedundantTerminalRays { get; init; }

    /// <summary>Optional caller identity included in generation diagnostics.</summary>
    public string? DiagnosticContext { get; init; }

    /// <summary>Optional completed original graph shared by matching roughing and Detail compilation.</summary>
    public VCarvePreparation? Preparation { get; init; }
    /// <summary>Reuse the complete analytical graph for nominal paths. Disable when
    /// sharing only the physical floor while retaining the ordinary optimized nomination.</summary>
    public bool UsePreparationGraphForNominalPaths { get; init; } = true;

    /// <summary>Transient captured initial-generation policy; no document setting or physical proof authority.</summary>
    public VCarveInitialStrategy? InitialStrategy { get; init; }

    /// <summary>Fresh-stock areas deliberately preserved by a coordinated preceding rougher.</summary>
    public VCarveRoughingAnalysis? RoughingAnalysis { get; init; }

    /// <summary>
    /// Runtime stock-transition allowance around protected bays. Selects additional clearing visits
    /// from the complete original artwork; it never changes a boundary, authored control or graph.
    /// </summary>
    public double ProtectedClearingMargin { get; init; }

}

/// <summary>
/// Produces V-carve toolpaths from pre-tessellated, world-space milling regions. The caller owns
/// document-model adaptation and output-machine coordinate conversion; this strategy owns all
/// generic geometry, depth, routing, and swept-area decisions.
/// </summary>
public static class VCarveToolpaths
{
    public const string Category = "vcarve";
    private static long _generationSequence;

    public static VCarvePreparation Prepare(IReadOnlyList<MillingRegion> regions, ToolGeometry tool,
        DepthSchedule depth, VCarveOptions options, IMedialAxisProvider provider,
        CancellationToken cancellationToken = default)
    {
        var access = options.BoundaryAccess?.Resolve(regions, tool, depth, options.SmoothVCarving,
            options.SmoothRampAngleDegrees, options.StockToLeave, cancellationToken);
        return VCarvePreparation.Create(access?.GetRegions(cancellationToken) ?? regions, tool, depth,
            access == null ? options : options with { BoundaryAccess = null, AccessTarget = access }, provider, cancellationToken);
    }

    public static ToolpathPlan Generate(
        IReadOnlyList<MillingRegion> regions,
        ToolGeometry tool,
        DepthSchedule depth,
        VCarveOptions options,
        IMedialAxisProvider provider,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(regions);
        ArgumentNullException.ThrowIfNull(tool);
        ArgumentNullException.ThrowIfNull(depth);
        ArgumentNullException.ThrowIfNull(options);
        ArgumentNullException.ThrowIfNull(provider);
        if (options.BoundaryAccess is { } boundaryAccess)
        {
            var access = boundaryAccess.Resolve(regions,tool,depth,options.SmoothVCarving,
                options.SmoothRampAngleDegrees,options.StockToLeave,cancellationToken);
            regions = access.GetRegions(cancellationToken);
            options = options with { BoundaryAccess=null,AccessTarget=access };
        }
        VCarveSmoothing.ValidateCorrectionArcTolerance(options.CorrectionArcTolerance);
        options = options.WithPriorStock(cancellationToken);
        if (options.SmoothVCarving && (!double.IsFinite(options.SmoothRampAngleDegrees)
            || options.SmoothRampAngleDegrees <= 0 || options.SmoothRampAngleDegrees >= 90))
            throw new ArgumentOutOfRangeException(nameof(options), "Smooth V-carving requires a ramp angle greater than zero and less than 90 degrees.");
        if (options.SmoothVCarving && (!double.IsFinite(options.StepOver) || options.StepOver <= 0 || options.StepOver > 1))
            throw new ArgumentOutOfRangeException(nameof(options), "Smooth V-carving requires a stepover greater than zero and no more than one.");
        if (options.SmoothVCarving)
            options = options with { DetectCurves = false };
        RW.Toolpaths.Milling.Acceleration.RawInitialClearingInsetNomination? initialInsetNomination = null;
        if (!options.IncludeInteriorFill && (!double.IsFinite(options.EdgeFillWidth) || options.EdgeFillWidth < 0))
            throw new ArgumentOutOfRangeException(nameof(options), "Edge fill width must be finite and nonnegative.");
        if (!double.IsFinite(options.RoughingRestWidth) || options.RoughingRestWidth < 0
            || options.RoughingRestWidth > 0 && !options.SmoothVCarving)
            throw new ArgumentOutOfRangeException(nameof(options), "Roughing rest reach requires smooth V-carving and a finite nonnegative width.");
        if (!double.IsFinite(options.ProtectedClearingMargin) || options.ProtectedClearingMargin < 0)
            throw new ArgumentOutOfRangeException(nameof(options), "Protected clearing margin must be finite and nonnegative.");
        if (options.Preparation is { } preparation && !preparation.Matches(regions, tool, depth, options))
            throw new InvalidOperationException("The prepared V-carve graph does not match its current physical tools, geometry, depth or ramp settings.");
        if (options.RoughingAnalysis is { } analysis && (!options.SmoothVCarving
            || !ReferenceEquals(analysis.Preparation, options.Preparation)))
            throw new InvalidOperationException("Roughing protection requires its matching completed smooth V-carve preparation.");
        long run = Interlocked.Increment(ref _generationSequence);
        using var feasibilityDiagnostics = InsetFeasibilityDiagnostics.Begin("vcarve", options.DiagnosticContext);
        long started = Stopwatch.GetTimestamp();
        string stage = "input";
        void Report(double fraction, string description, bool indeterminate = false)
        {
            cancellationToken.ThrowIfCancellationRequested();
            options.Progress?.Invoke(fraction, description, indeterminate);
        }
        void Trace(string detail)
        {
            stage = detail;
            Console.WriteLine($"[VCarve.Stage] run={run} operation='{options.DiagnosticContext}' " +
                $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} {detail}");
        }
        Trace($"stage=input provider={provider.GetType().Name} thread={Environment.CurrentManagedThreadId} " +
            $"smoothVCarving={options.SmoothVCarving} smoothRampAngleDegrees={options.SmoothRampAngleDegrees:R} " +
            $"curvePolicy={(options.SmoothVCarving ? "certified-lines" : "authored")} " +
            $"regions={regions.Count} interiorFill={options.IncludeInteriorFill} edgeFillWidth={options.EdgeFillWidth:R} detectCurves={options.DetectCurves} stockToLeave={options.StockToLeave:R} " +
            $"preferOriginalInsetConstruction={options.PreferOriginalInsetConstruction} " +
            $"correctionArcTolerance={options.CorrectionArcTolerance:R} " +
            $"useOrdinaryCorrectionCandidates={options.UseOrdinaryCorrectionCandidates} " +
            $"toolRadius={tool.Radius:R} bottomRadius={tool.BottomRadius:R} topRadius={tool.TopRadius:R} " +
            $"coneLength={tool.ConeLength:R} tipAngleRadians={tool.TipAngleRadians:R} " +
            $"depth={depth.Depth:R} depthPerPass={depth.DepthPerPass:R} surfaceZ={depth.SurfaceZ:R} " +
            $"startingDepth={depth.StartDepth:R} startingFloorZ={depth.SurfaceZ - depth.StartDepth:R} " +
            $"skipDepthPasses={depth.SkipDepthPasses} rampAngle={options.Ramp.AngleRadians:R} " +
            $"tolerance={options.Tolerance:R} stepOver={options.StepOver:R} " +
            $"physicalProfile={tool.VCarveProfile is not null} roundedTip={tool.VCarveProfile?.RoundedTip} " +
            $"noseDiameter={tool.VCarveProfile?.ClearingDiameter:R}");

        try
        {
            Report(0, "Preparing V-carve boundaries", true);
            if (regions.Count == 0
                || tool.Radius <= 1e-6
                || depth.Depth <= 0
                || (tool.VCarveProfile is null
                    && (tool.TipAngleRadians is not > 1e-6 || tool.ConeLength is not > 1e-6)))
            {
                Trace("stage=complete result=empty reason=invalid-input");
                return ToolpathPlan.Empty;
            }

            double maximumDepth = Math.Min(depth.Depth,
                tool.VCarveProfile?.MaximumDepth ?? tool.ConeLength!.Value);
            if (maximumDepth <= 1e-6)
            {
                Trace("stage=complete result=empty reason=effective-depth");
                return ToolpathPlan.Empty;
            }

            if (options.SmoothVCarving)
            {
                // Target trimming can remove every original medial branch at an
                // open platform corner. Keep its complete analytical graph when
                // the operator explicitly restores outside ramp anchors.
                if (options.RampOutsideEdges && options.AccessTarget != null && options.Preparation == null)
                    options = options with { Preparation = VCarvePreparation.Create(regions, tool, depth,
                        options, provider, cancellationToken) };
                var strategyProfile = VCarvePhysicalSweep.ProfileFor(tool, maximumDepth);
                var strategy = options.InitialStrategy ?? VCarveInitialStrategy.Capture(options, strategyProfile, maximumDepth);
                if (!strategy.Matches(options, strategyProfile, maximumDepth))
                    throw new InvalidOperationException("The captured initial V-carve strategy does not match its current request/profile.");
                options = options with { InitialStrategy = strategy };
                initialInsetNomination = RW.Toolpaths.Milling.Acceleration.RawInitialClearingInsetNomination.CaptureForStrategy(
                    options.SmoothVCarving, options.SmoothRampAngleDegrees, strategy);
                Trace($"stage=initial-strategy version={VCarveInitialStrategy.Version} mask={strategy.Mask:X2} origin={strategy.Origin} automaticEligible={strategy.AutomaticEligible} "
                    + "capturedOncePerOperation=True sourceAndRuntimeAdmissionStillRequired=True physicalAndStockCredit=False");
            }

            double depthPerPass = depth.DepthPerPass > 0
                ? Math.Min(depth.DepthPerPass, maximumDepth)
                : maximumDepth;
            var effectiveDepth = new DepthSchedule(maximumDepth, depthPerPass)
            {
                SurfaceZ = depth.SurfaceZ,
                SkipDepthPasses = depth.SkipDepthPasses,
                StartDepth = Math.Min(depth.StartDepth, maximumDepth),
            };
            Trace($"stage=boundary event=start effectiveDepth={maximumDepth:R} " +
                $"passes={effectiveDepth.PassCount} nominalPasses={effectiveDepth.NominalPassCount} " +
                $"authoredStartingDepth={depth.StartDepth:R} effectiveStartingDepth={effectiveDepth.StartDepth:R} " +
                $"skipDepthPasses={depth.SkipDepthPasses} effectiveSkipped={effectiveDepth.FirstPassIndex} " +
                $"firstBottomZ={effectiveDepth.PassBottomZ(0):R} firstEntryZ={effectiveDepth.PassTopZ(0):R} " +
                $"depthClamped={maximumDepth < depth.Depth} finalTargetRetained=True");

            var carveRegions = new List<(int RegionIndex, List<IReadOnlyList<PointD>> Rings)>(regions.Count);
            for (int regionIndex = 0; regionIndex < regions.Count; regionIndex++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var rings = BuildBoundary(regions[regionIndex], options.StockToLeave);
                Trace($"stage=boundary event=region region={regionIndex} sourcePoints=" +
                    $"{regions[regionIndex].Outer.Count + regions[regionIndex].Holes.Sum(hole => hole.Count)} " +
                    $"{DescribeBoundary(rings)}");
                if (rings.Count > 0)
                    carveRegions.Add((regionIndex, rings));
                Report(0.08 * (regionIndex + 1) / regions.Count,
                    $"Preparing boundary {regionIndex + 1}/{regions.Count}");
            }

            if (carveRegions.Count == 0)
            {
                Trace("stage=complete result=empty reason=no-boundaries-after-stock-offset");
                return ToolpathPlan.Empty;
            }

            var materialSurface = !options.SmoothVCarving && options.AccessTarget is null
                && depth.StartDepth > 0 && options.ClearedStock is { } stock
                && (stock.Chords.Count > 0 || stock.ProfileSweeps?.Count > 0)
                ? new VCarveMaterialSurface(carveRegions.SelectMany(r => r.Rings), stock,
                    VCarvePhysicalSweep.ProfileFor(tool, maximumDepth), maximumDepth,
                    depth.SurfaceZ, cancellationToken, Trace) : null;
            if (!options.SmoothVCarving && depth.StartDepth > 0 && materialSurface is null)
                Trace("stage=material-surface event=original-stock-fallback reason=no-compatible-posted-stock "
                    + "originalBranchesRetained=True declaredStartingDepthNotCreditedAsRemoval=True");

            Trace("stage=travel-order event=start");
            Report(0.08, "Ordering V-carve regions", true);
            int[] visitOrder = TravelOptimizer.Order(
                carveRegions.Select(region => (IReadOnlyList<PointD>)region.Rings[0]).ToList(),
                label: "vcarve",
                cancellationToken: cancellationToken);
            Trace($"stage=travel-order event=complete regions={visitOrder.Length}");

            var toolpaths = new List<TaggedToolpath>();
            PreparedSweepExclusion? protectedClearingSelector = null;
            var scheduledDepths = Enumerable.Range(0, effectiveDepth.PassCount)
                .Select(index => depth.SurfaceZ - effectiveDepth.PassBottomZ(index)).ToList();
            int sourcePointCount = 0;
            int compactedPointCount = 0;
            double totalWeight = carveRegions.Sum(region => (double)region.Rings.Sum(ring => ring.Count));
            double completedWeight = 0;
            List<TaggedToolpath>? outsideEdgeNominations = null;
            foreach (int visitSlot in visitOrder)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var (regionIndex, boundary) = carveRegions[visitSlot];
                var preparedRegion = options.Preparation?.Regions.FirstOrDefault(region => region.RegionIndex == regionIndex);
                bool freshStock = options.RoughingAnalysis?.ProtectedSourceRegionIndices.Contains(regionIndex) == true;
                bool entireProtected = options.RoughingAnalysis?.EntireProtectedSourceRegionIndices.Contains(regionIndex) == true;
                // A partial bay changes stock scheduling, not the ordinary
                // ocean's authored clearing layers/reference channel. Its
                // supplemental full clearing separately begins at the surface.
                var regionDepth = entireProtected ? effectiveDepth with { StartDepth = 0, SkipDepthPasses = 0 } : effectiveDepth;
                var regionOptions = entireProtected ? options with { IncludeInteriorFill = true } : options;
                Trace($"stage=coordination region={regionIndex} analysisReuse={preparedRegion is not null} " +
                    $"protectedFreshStock={freshStock} entireSource={entireProtected} " +
                    $"authoredStartingDepth={depth.StartDepth:R} ordinaryStartingDepth={regionDepth.StartDepth:R} " +
                    $"effectiveStartingDepth={(options.SmoothVCarving || freshStock ? 0 : regionDepth.StartDepth):R} " +
                    $"assignedStepdown={regionDepth.DepthPerPass:R} sourceBoundaryAndGraphPreserved=True " +
                    $"ownSurfaceOriginScheduling={options.SmoothVCarving} upstreamMetadataRequired=False");
                double regionWeight = boundary.Sum(ring => ring.Count);
                void RegionProgress(double fraction, string description, bool indeterminate)
                {
                    Report(0.1 + 0.85 * (completedWeight + regionWeight * Math.Clamp(fraction, 0, 1)) / totalWeight,
                        $"Region {regionIndex + 1}/{regions.Count}: {description}", indeterminate);
                }
                Trace($"stage=region event=start region={regionIndex}");
                double nominationWidth = options.EdgeFillWidth;
                Trace($"stage=roughing-rest-reach region={regionIndex} authoredFillWidth={options.EdgeFillWidth:R} "
                    + $"requestedRestWidth={options.RoughingRestWidth:R} nominationWidth={nominationWidth:R} "
                    + "sourceBoundaryAndFloorUnchanged=True priorRemovalCredit=False originalGuardsRequired=True");
                List<TaggedToolpath> generated = MedialAxisToolpaths.GenerateVCarveTagged(
                    options.PreferOriginalInsetConstruction,
                    provider,
                    boundary,
                    startDepth: 0.0,
                    endDepth: maximumDepth,
                    radianTipAngle: tool.TipAngleRadians ?? 0,
                    depthPerPass: regionDepth.DepthPerPass,
                    stepOver: options.StepOver,
                    tolerance: options.Tolerance,
                    regionIndex: regionIndex,
                    bottomRadiusOverride: tool.BottomRadius,
                    topRadiusOverride: tool.TopRadius,
                    coneLengthOverride: tool.ConeLength,
                    cancellationToken: cancellationToken,
                    includeInteriorFill: regionOptions.IncludeInteriorFill,
                    diagnostic: detail => Trace($"region={regionIndex} {detail}"),
                    detectCurves: options.DetectCurves,
                    cutterProfile: tool.VCarveProfile,
                    edgeFillWidth: !regionOptions.IncludeInteriorFill && nominationWidth > 0
                        ? Math.Max(0, nominationWidth - AppliedStockAllowance(options.StockToLeave))
                        : 0,
                    progress: (fraction, description, indeterminate) => RegionProgress(
                        options.SmoothVCarving ? .5 * fraction : fraction, description, indeterminate),
                    outputSurfaceZ: depth.SurfaceZ,
                    skipDepthPasses: regionDepth.SkipDepthPasses,
                    startingDepth: regionDepth.StartDepth,
                    preparedMedialGraph: options.UsePreparationGraphForNominalPaths ? preparedRegion?.OriginalMedialGraph : null,
                    initialInsetNomination: initialInsetNomination,
                    supplementalInitialBand: options.RoughingRestWidth <= 0);

                HashSet<TaggedToolpath>? shoulderNominations = null;
                if (materialSurface is not null)
                {
                    var physical = VCarvePhysicalSweep.ProfileFor(tool, maximumDepth);
                    // The original-stock perimeter misses the band exposed by a
                    // lower adjacent shoulder. Nominate that outer contact front;
                    // the actual stock field still proves every full-tool chord.
                    // Keep the original front for portions beside untouched walls.
                    double originalRadius = physical.RadiusAtDepth(maximumDepth);
                    {
                        // Boundary samples nominate contact rays, including
                        // channels narrower than the full-depth cutter diameter.
                        // Actual XY and Z are solved by the full physical guard.
                        var shoulderBoundary = BuildShoulderBoundary(regions[regionIndex],options.StockToLeave);
                        var shoulder = new List<TaggedToolpath>();
                        for (int pass = 0; pass < regionDepth.PassCount; pass++)
                        {
                            double cutDepth = depth.SurfaceZ - regionDepth.PassBottomZ(pass);
                            foreach (var ring in shoulderBoundary)
                            {
                                var points = ring.Select(p => new Point3D(p.x, p.y, -cutDepth)).ToList();
                                if (points.Count < 3) continue;
                                if (points[^1] != points[0]) points.Add(points[0]);
                                var nomination = new TaggedToolpath(points, Array.Empty<ToolpathSpan>(),
                                    regionIndex, "clearing", pass, detectCurves: false);
                                shoulder.Add(nomination);
                                (shoulderNominations ??= new()).Add(nomination);
                            }
                        }
                        int detailStart = generated.FindIndex(path => path.Category == "final-carve");
                        generated.InsertRange(detailStart >= 0 ? detailStart : generated.Count, shoulder);
                        Trace($"stage=material-surface event=shoulder-front region={regionIndex} paths={shoulder.Count} "
                            + $"scheduledStartingDepth={regionDepth.StartDepth:R} originalRadius={originalRadius:R} "
                            + $"nominationPoints={shoulderBoundary.Sum(r=>r.Count)} contourSimplification={MachiningAccuracy.SurfaceDeviation/2:R} "
                            + "floorSource=actual-posted-sweeps fullPhysicalStockGuardRequired=True originalWallFrontRetained=True");
                    }
                }

                if (options.AccessTarget is { } accessTarget)
                {
                    var physical = VCarvePhysicalSweep.ProfileFor(tool, maximumDepth);
                    // New full-depth demand exists only along interfaces opened by permission.
                    // Existing optional filling elsewhere keeps its authored policy.
                    outsideEdgeNominations ??= accessTarget.NominateOpenEdges(physical,regionDepth,options.StepOver,
                        options.StockToLeave,cancellationToken);
                    var componentIndex = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(boundary));
                    var openFill = accessTarget.Trim(outsideEdgeNominations,_=>0,cancellationToken,clipIndex:componentIndex);
                    generated.AddRange(openFill.Select(path=>path with {RegionIndex=regionIndex}));
                    generated=accessTarget.Trim(generated,physical.PhysicalRadiusAtDepth,cancellationToken);
                }

                if (freshStock)
                {
                    if (!entireProtected && !options.IncludeInteriorFill)
                    {
                        if (protectedClearingSelector is null)
                        {
                            long prepareStarted = Stopwatch.GetTimestamp();
                            cancellationToken.ThrowIfCancellationRequested();
                            Trace($"stage=protected-clearing-selection event=prepare-start "
                                + $"protectionRings={options.RoughingAnalysis!.ProtectedBoundary.Count} "
                                + $"protectionPoints={options.RoughingAnalysis.ProtectedBoundary.Sum(ring => (long)ring.Count)} "
                                + "selectorOwner=generation originalUnbufferedProtection=True");
                            protectedClearingSelector = new PreparedSweepExclusion(options.RoughingAnalysis.ProtectedBoundary);
                            cancellationToken.ThrowIfCancellationRequested();
                            Trace($"stage=protected-clearing-selection event=prepare-complete "
                                + $"elapsedMs={Stopwatch.GetElapsedTime(prepareStarted).TotalMilliseconds:F1} "
                                + "selectorOwner=generation allSourcesAndDepthsReuse=True globalBuffer=False");
                        }
                        else Trace("stage=protected-clearing-selection event=prepare-reuse selectorOwner=generation");
                        var fullClearing = MedialAxisToolpaths.GenerateClearingPassesTagged(boundary,
                            0, maximumDepth, tool.TipAngleRadians ?? 0, regionDepth.DepthPerPass,
                            options.StepOver, regionIndex, tool.BottomRadius, tool.TopRadius, tool.ConeLength,
                            includeInteriorFill: true, detectCurves: false,
                            diagnostic: detail => Trace($"region={regionIndex} protected-clearing {detail}"),
                            cutterProfile: tool.VCarveProfile, cancellationToken: cancellationToken,
                            outputSurfaceZ: depth.SurfaceZ);
                        generated.AddRange(SelectProtectedClearing(fullClearing, protectedClearingSelector,
                            tool, maximumDepth, options.ProtectedClearingMargin, cancellationToken,
                            detail => Trace($"region={regionIndex} {detail}")));
                    }
                    generated = generated.OrderBy(path => path.Category == "final-carve" ? 1 : 0)
                        .ThenBy(path => path.DepthPassIndex ?? int.MaxValue).ToList();
                }

                Func<Point3D, Point3D, bool>? continuationGuard = null;
                Func<IReadOnlyList<TaggedToolpath>,List<TaggedToolpath>>? clippedPathProtection=null;
                if (materialSurface is not null)
                {
                    generated = materialSurface.Apply(generated, shoulderNominations);
                    continuationGuard = (a,b) => materialSurface.IsLegalChord(
                        new(a.X,a.Y,a.Z-depth.SurfaceZ),new(b.X,b.Y,b.Z-depth.SurfaceZ));
                }
                if (options.SmoothVCarving)
                    generated = VCarveSmoothing.ApplyWithContinuation(generated, boundary, tool, regionDepth,
                        options, cancellationToken, Trace,
                        (fraction, description, indeterminate) => RegionProgress(
                            .5 + .45 * fraction, description, indeterminate), preparedRegion,
                        captureContinuationGuard: guard => continuationGuard = guard,
                        captureClippedPathProtection: protect=>clippedPathProtection=protect);

                if(options.AccessTarget is { } centreAccess)
                {
                    // Smoothing may nominate new chords after the first target trim. Clip
                    // those nominations before depth scheduling/ramp construction, and
                    // keep every later circuit/router link on the platform as well.
                    if(options.SmoothVCarving)
                    {
                        var inserted=new List<int>();
                        generated=centreAccess.Trim(generated,VCarvePhysicalSweep.ProfileFor(tool,maximumDepth).PhysicalRadiusAtDepth,
                            cancellationToken,retainContactingChords:clippedPathProtection is not null,interpolatedFragments:inserted);
                        if(inserted.Count>0&&clippedPathProtection is not null)
                        {
                            var before=inserted.Select(i=>generated[i]).ToArray();
                            var repaired=clippedPathProtection(before);
                            if(repaired.Count!=inserted.Count)
                                throw new InvalidOperationException("Outside-edge posting protection changed clipped path ordinals.");
                            for(int i=0;i<inserted.Count;i++)generated[inserted[i]]=repaired[i];
                            Trace($"stage=outside-trim-posting event=protected fragments={inserted.Count} "
                                +$"beforePoints={before.Sum(p=>(long)p.Points.Count)} afterPoints={repaired.Sum(p=>(long)p.Points.Count)} "
                                +"owner=original-floor-wall-grade originalCutRetained=True newPlatformVerticesFreshlyProved=True");
                        }
                    }
                    if(continuationGuard is { } physicalContinuation)
                        continuationGuard=(a,b)=>centreAccess.CentreChord(a,b) && physicalContinuation(a,b);
                }

                if (options.SmoothVCarving || freshStock)
                {
                    if (continuationGuard is not null)
                    {
                        // Protection adds proof subdivisions, not new design
                        // features. Bound their physical sweep displacement by
                        // the established 3D capsule tolerance before circuit
                        // duplication can exhaust the fixed vertex inventory.
                        // Every replacement passes the SAME native/F3 floor,
                        // artwork/channel and grade proof used by connectors.
                        long compactStarted=Stopwatch.GetTimestamp();
                        long beforePoints=generated.Sum(p=>(long)p.Points.Count),proposals=0,refusals=0;
                        bool AcceptSurfaceChord(Point3D a,Point3D b)
                        {
                            proposals++;
                            bool accepted=continuationGuard(a,b);
                            if(!accepted)refusals++;
                            return accepted;
                        }
                        generated=generated.Select(path=>SurfaceToolpathSimplifier.Simplify(path,
                            MachiningAccuracy.PathChord,cancellationToken,AcceptSurfaceChord)).ToList();
                        Trace($"stage=surface-simplification event=protected-before-circuits paths={generated.Count} "
                            +$"sourcePoints={beforePoints} retainedPoints={generated.Sum(p=>(long)p.Points.Count)} "
                            +$"chordBoundMm={MachiningAccuracy.PathChord:R} proposals={proposals} refusedChords={refusals} "
                            +$"elapsedMs={Stopwatch.GetElapsedTime(compactStarted).TotalMilliseconds:F1} "
                            +"sameNativeAndPostedFloorWallGradeRequired=True circuitVertexBudgetUnchanged=True motionKindsPreserved=True");
                        double firstStep=regionDepth.DepthPerPass>0 ? regionDepth.DepthPerPass : regionDepth.Depth;
                        var profile=VCarvePhysicalSweep.ProfileFor(tool,regionDepth.Depth);
                        // Required cuts come from the protected surface. The
                        // circuit builder alone owns connecting motion; a prior
                        // endpoint stitcher must not add crossing finishing cuts.
                        if(options.RoughingRestWidth>0)
                        {
                            generated=VCarveVisitStockPruner.Filter(generated,profile,options.RoughingRestChords,
                                p=>new(VCarveCommandRounding.Round((float)p.X),VCarveCommandRounding.Round((float)p.Y),
                                    VCarveCommandRounding.Round((float)(depth.SurfaceZ+p.Z)-(float)depth.SurfaceZ)),
                                0,cancellationToken,Trace,options.PriorStock,depth.SurfaceZ);
                        }
                        double portalDistance=Math.Max(.1,2*options.StepOver*Math.Max(profile.ClearingDiameter,
                            2*profile.PhysicalRadiusAtDepth(Math.Min(regionDepth.Depth,firstStep))));
                        MillingGeometryTrace.Capture("stock-selected",0,generated);
                        // A terminal chord certificate covers its shallower
                        // same-XY caps: this profile is radially nondecreasing,
                        // and native/posted endpoint rises only decrease grade.
                        // Re-proving each cap repeats the same containment work.
                        generated=ToolpathCircuitBuilder.Build(generated,continuationGuard,portalDistance,cancellationToken,Trace,
                            toolpaths.Count==0 ? null : toolpaths[^1].Points[^1],
                            Math.Tan(options.SmoothRampAngleDegrees*Math.PI/180));
                        MillingGeometryTrace.Capture("source-circuits",0,generated);
                    }
                    Trace($"stage=surface-origin-scheduling region={regionIndex} owner=current-vcarve "
                        + $"smooth={options.SmoothVCarving} assignedStepdown={regionDepth.DepthPerPass:R} "
                        + "upstreamStockMetadataRequired=False originalFillSettingsPreserved=True");
                    if(!generated.All(p=>p.ContinuationCircuitDefinition is not null))
                        generated = ScheduleFreshStockVisits(generated, regionDepth, cancellationToken, Trace);
                }

                Trace($"stage=compaction event=start region={regionIndex} paths={generated.Count} " +
                    $"clearingPaths={generated.Count(path => path.Category == "clearing")} " +
                    $"finalPaths={generated.Count(path => path.Category == "final-carve")}");
                RegionProgress(1, "Compacting V-carve paths", true);

                var compactedPaths = new List<TaggedToolpath>(generated.Count);
                int noPenetrationVisits = 0;
                foreach (TaggedToolpath path in generated)
                {
                    if (path.Points.Count < 2)
                        continue;

                    sourcePointCount += path.Points.Count;
                    List<Point3D> compacted = CompactAndClamp(path.Points, depth.SurfaceZ, maximumDepth,
                        preserveCertifiedChords: options.SmoothVCarving || options.AccessTarget != null || materialSurface != null,
                        preserveAllVertices: options.AccessTarget != null || materialSurface != null);
                    compactedPointCount += compacted.Count;
                    if (compacted.Count < 2)
                        continue;

                    // The current machine post emits stock-relative Z with F3.
                    // A visit whose complete commanded trace remains at Z0 cannot
                    // cut, so reject it before routing adds approach/link/retract
                    // motion. Any real descending, flat, or spring cut survives.
                    if (!HasCommandedPenetration(compacted, depth.SurfaceZ))
                    {
                        if (++noPenetrationVisits <= 8)
                            Trace($"stage=compaction event=reject reason=zero-commanded-penetration " +
                                $"region={regionIndex} category={path.Category} depthPass={path.DepthPassIndex} " +
                                $"points={compacted.Count} zDecimals=3 stockRelative=True " +
                                $"start=({compacted[0].X:R},{compacted[0].Y:R},{compacted[0].Z:R})");
                        continue;
                    }

                    compactedPaths.Add(new TaggedToolpath(
                        compacted,
                        path.Spans,
                        path.RegionIndex,
                        path.Category,
                        path.DepthPassIndex,
                        path.VisitIdentity,
                        detectCurves: options.DetectCurves) {ContinuationComponentIndex=path.ContinuationComponentIndex,
                        ContinuationPlan=path.ContinuationPlan,ContinuationSourceOrdinal=path.ContinuationSourceOrdinal,
                        ContinuationCircuit=path.ContinuationCircuit,ContinuationCircuitDefinition=path.ContinuationCircuitDefinition});
                }
                Trace($"stage=compaction event=complete region={regionIndex} retainedVisits={compactedPaths.Count} " +
                    $"zeroCommandedPenetrationVisits={noPenetrationVisits} zDecimals=3 stockRelative=True");
                RegionProgress(1, "Ordering and linking V-carve paths", true);
                var routed = VCarvePathRouter.Route(compactedPaths, boundary, tool, regionDepth,
                    regionOptions, !regionOptions.IncludeInteriorFill && options.EdgeFillWidth > 0
                        ? Math.Max(0, options.EdgeFillWidth - AppliedStockAllowance(options.StockToLeave)) : 0,
                    toolpaths.Count == 0 ? null : toolpaths[^1].Points[^1], cancellationToken, Trace,
                    continuationGuard, materialSurface is null ? null : materialSurface.EntryHeight);
                if(options.SmoothVCarving)MillingGeometryTrace.Capture("routed-depth-passes",0,routed);
                long routedPoints = 0, retainedPoints = 0;
                foreach (var path in routed)
                {
                    routedPoints += path.Points.Count;
                    var retained = options.SmoothVCarving && options.CompactCollinearCommands
                        ? ToolpathCollinearCompactor.Compact(path, depth.SurfaceZ, cancellationToken) : path;
                    retainedPoints += retained.Points.Count;
                    toolpaths.Add(retained with { VisitIdentity = toolpaths.Count });
                }
                Trace($"stage=command-collinear-compaction region={regionIndex} sourcePoints={routedPoints} " +
                    $"retainedPoints={retainedPoints} removedPoints={routedPoints - retainedPoints} " +
                    $"enabled={options.SmoothVCarving && options.CompactCollinearCommands} postedPlanarAnchorRequired=True " +
                    "nativeDirectedChord=True exactF3DirectedChord=True spanBoundariesAndMotionModePreserved=True stockAtEveryHeightPreserved=True");
                Trace($"stage=region event=complete region={regionIndex} accumulatedPaths={toolpaths.Count} " +
                    $"sourcePoints={sourcePointCount} compactedPoints={compactedPointCount}");
                completedWeight += regionWeight;
            }
            toolpaths=VCarvePathRouter.OrderRegions(toolpaths,cancellationToken,Trace);
            options.AccessTarget?.Verify(toolpaths,tool,depth,options.StockToLeave,cancellationToken);
            materialSurface?.Verify(toolpaths);

            cancellationToken.ThrowIfCancellationRequested();
            List<List<PointD>> sweptArea = new();
            if (options.CalculateSweptArea)
            {
                Report(0.95, "Calculating swept area", true);
                Trace($"stage=swept-area event=start paths={toolpaths.Count} points={compactedPointCount} radius={tool.Radius:R}");
                var centrelines = toolpaths
                    .Select(path => (IReadOnlyList<PointD>)path.Points
                        .Select(point => new PointD(point.X, point.Y))
                        .ToList())
                    .ToList();
                sweptArea = PolygonOps.BufferOpenCenterlines(centrelines, tool.Radius,
                    cancellationToken: cancellationToken);
            }
            cancellationToken.ThrowIfCancellationRequested();
            Trace($"stage=swept-area event={(options.CalculateSweptArea ? "complete" : "skipped")} polygons={sweptArea.Count} " +
                $"calculated={options.CalculateSweptArea}");

            var diagnostics = new ToolpathDiagnostics
            {
                RingCount = carveRegions.Sum(region => region.Rings.Count),
                IslandCount = carveRegions.Count,
                PathCount = toolpaths.Count,
                DepthPassCount = scheduledDepths.Count,
                CutLength = toolpaths.Sum(path => PathLength(path.Points)),
            };
            Console.WriteLine(
                $"[VCarveToolpaths] run={run} operation='{options.DiagnosticContext}' regions={carveRegions.Count} paths={toolpaths.Count} " +
                $"sourcePoints={sourcePointCount} compactedPoints={compactedPointCount} " +
                $"depth={maximumDepth:F4} passes={effectiveDepth.PassCount} " +
                $"startingDepth={depth.StartDepth:R} effectiveStartingDepth={effectiveDepth.StartDepth:R} " +
                $"skipDepthPasses={depth.SkipDepthPasses} effectiveSkipped={effectiveDepth.FirstPassIndex} " +
                $"sweptPolygons={sweptArea.Count}");
            Trace($"stage=complete result=success {DescribeOutput(toolpaths)} cutLength={diagnostics.CutLength:R}");
            Report(1, "V-carve geometry complete");

            var emissionWalls=materialSurface is null ? new VCarveSmoothing.CommandClearance(
                carveRegions.SelectMany(r=>r.Rings).ToArray(),
                VCarvePhysicalSweep.ProfileFor(tool,maximumDepth),depth.SurfaceZ,cancellationToken,Trace) : null;
            return new ToolpathPlan(toolpaths, sweptArea, diagnostics)
            {
                ReplacementChordGuard=(a,b)=>materialSurface is not null
                    ? materialSurface.IsLegalChord(new(a.X,a.Y,a.Z-depth.SurfaceZ),new(b.X,b.Y,b.Z-depth.SurfaceZ))
                    : emissionWalls!.IsLegalCommandedChord(new(a.X,a.Y,a.Z-depth.SurfaceZ),new(b.X,b.Y,b.Z-depth.SurfaceZ))
            };
        }
        catch (OperationCanceledException)
        {
            try { Console.WriteLine($"[VCarve.Cancelled] run={run} operation='{options.DiagnosticContext}' " +
                $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} lastStage='{stage}'"); }
            catch { } // A diagnostic sink must not replace cancellation.
            throw;
        }
        catch (Exception exception)
        {
            try { Console.Error.WriteLine($"[VCarve.Failed] run={run} operation='{options.DiagnosticContext}' " +
                $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} lastStage='{stage}' {exception}"); }
            catch { } // Keep the original physical/generation failure authoritative.
            throw;
        }
    }

    internal static List<TaggedToolpath> ScheduleFreshStockVisits(IReadOnlyList<TaggedToolpath> source,
        DepthSchedule schedule, CancellationToken token, Action<string>? diagnostic)
    {
        // Every post-smoothing visit participates, including newly discovered correction rings
        // and terminal rays. The same XY with a shallower tip removes a subset of the physical
        // volume of its deeper visit; therefore the later visit's fresh engagement is bounded
        // axially by this increment across its full footprint, not just at its center.
        var layers = FreshStockLayers(source, schedule, token);
        double assignedStep = schedule.DepthPerPass > 0 ? schedule.DepthPerPass : schedule.Depth;
        double commandStep = Math.Floor(assignedStep * 1000 + 1e-9) / 1000;
        double CommandDepth(double nominal) => -VCarveCommandRounding.Round((float)(schedule.SurfaceZ - nominal) - (float)schedule.SurfaceZ);
        var result = new List<TaggedToolpath>();
        int precursors = 0;
        foreach (TaggedToolpath sourcePath in source)
        {
            var path=sourcePath;
            token.ThrowIfCancellationRequested();
            double deepest = path.Points.Select(point => CommandDepth(-point.Z)).DefaultIfEmpty(0).Max();
            for (int pass = 0; pass < layers.Count && layers[pass].Command < deepest - 1e-10; pass++)
            {
                var capped = path.ContinuationCircuitDefinition?.AtDepth(layers[pass].Nominal,
                    pass==0 ? -1 : layers[pass-1].Nominal,token)
                    ?? path.Points.Select(point => point with { Z = Math.Max(point.Z, -layers[pass].Nominal) }).ToArray();
                result.Add(new TaggedToolpath(capped,
                    Array.Empty<ToolpathSpan>(), path.RegionIndex, "clearing", pass, detectCurves: false)
                    {ContinuationComponentIndex=path.ContinuationComponentIndex,ContinuationPlan=path.ContinuationPlan,
                        ContinuationSourceOrdinal=path.ContinuationSourceOrdinal,ContinuationCircuit=path.ContinuationCircuit});
                precursors++;
            }
            int targetPass = layers.Count(layer => layer.Command < deepest - 1e-10);
            if(path.ContinuationCircuitDefinition is { } definition)
            {
                var points=definition.AtDepth(double.PositiveInfinity,targetPass==0 ? -1 : layers[targetPass-1].Nominal,token);
                path=path with {Points=points,Moves=new LineToolMoves(points)};
            }
            result.Add(path.Category == "final-carve" ? path : path with { DepthPassIndex = targetPass });
        }
        diagnostic?.Invoke($"stage=coordination event=fresh-stock-passes sourceVisits={source.Count} " +
            $"precursorVisits={precursors} outputVisits={result.Count} surfaceOrigin=True " +
            $"assignedStepdown={assignedStep:R} commandedStep={commandStep:R} " +
            "allCorrectionAndTerminalVisits=True fullPhysicalFootprintSameXY=True originalWallsAndGradePreserved=True");
        // A shallow detail stroke can be completed in the first physical layer;
        // executing it there establishes its real connecting trace before deeper
        // branches, without duplicating it or changing its cut/category metadata.
        bool EarlyDetail(TaggedToolpath p)=>p.ContinuationComponentIndex.HasValue && p.Category=="final-carve"
            && p.Points.All(point=>CommandDepth(-point.Z)<=assignedStep+1e-10);
        return result.OrderBy(path=>path.ContinuationComponentIndex ?? 0)
            .ThenBy(path=>path.Category=="final-carve" && !EarlyDetail(path) ? 1 : 0)
            .ThenBy(path=>EarlyDetail(path) ? 0 : path.DepthPassIndex ?? int.MaxValue)
            .ThenBy(path=>path.ContinuationPlan is not null && path.ContinuationSourceOrdinal is { } owner
                ? path.ContinuationPlan.VisitOrder[owner] : 0).ToList();
    }

    internal static List<(double Nominal, double Command)> FreshStockLayers(IReadOnlyList<TaggedToolpath> source,
        DepthSchedule schedule, CancellationToken token, double zOffset=0)
    {
        double assignedStep = schedule.DepthPerPass > 0 ? schedule.DepthPerPass : schedule.Depth;
        double commandStep = Math.Floor(assignedStep * 1000 + 1e-9) / 1000;
        if (commandStep <= 0)
            throw new InvalidOperationException("Smooth V-carving requires a stepdown representable by the machine's 0.001 depth commands.");
        double CommandDepth(double nominal) => -VCarveCommandRounding.Round((float)(schedule.SurfaceZ - nominal) - (float)schedule.SurfaceZ);
        double deepestCommand = source.SelectMany(path => path.Points).Select(point => CommandDepth(zOffset-point.Z)).DefaultIfEmpty(0).Max();
        var layers = new List<(double Nominal, double Command)>();
        double prior = 0;
        while (prior + assignedStep < deepestCommand - 1e-10)
        {
            token.ThrowIfCancellationRequested();
            double nominal = prior + commandStep;
            double command = CommandDepth(nominal);
            while (command > prior + assignedStep + 1e-10)
            { nominal -= .001; command = CommandDepth(nominal); }
            if (command <= prior + 1e-10)
                throw new InvalidOperationException("The physical stock Z and output precision cannot represent the assigned smooth V-carving stepdown.");
            layers.Add((nominal, command)); prior = command;
            if (layers.Count > 1_000_000) throw new InvalidOperationException("Smooth V-carving surface-origin depth schedule exceeded one million layers.");
        }
        return layers;
    }

    internal static List<TaggedToolpath> SelectProtectedClearing(IReadOnlyList<TaggedToolpath> source,
        PreparedSweepExclusion selector, ToolGeometry tool, double maximumDepth,
        double stockTransitionMargin, CancellationToken cancellationToken, Action<string>? diagnostic = null)
    {
        var profile = VCarvePhysicalSweep.ProfileFor(tool, maximumDepth);
        var output = new List<TaggedToolpath>();
        var depths = new Dictionary<double, (int Visits, int Retained, long Chords, double Milliseconds)>();
        long started = Stopwatch.GetTimestamp(), queriedChords = 0;
        long sourceVertices = 0, outputVertices = 0;
        int wholeVisits = 0, partialVisits = 0, discardedVisits = 0;
        long wholeChords = 0, partialChords = 0, discardedChords = 0;
        long midpointInside = 0, midpointOutside = 0, localQueries = 0, candidateEdges = 0, boundaryClassified = 0;
        // Two grid units cover the former round-offset/intersection quantization.
        // ContactIntervals also reserves four coordinate units and one arithmetic unit.
        const double selectionReserve = 2 / PathUtils.Scale;
        diagnostic?.Invoke($"stage=protected-clearing-selection event=start sourceVisits={source.Count} "
            + $"selectionReserve={selectionReserve:R} coordinateReserve={4 / PathUtils.Scale:R} "
            + $"arithmeticReserve={1 / PathUtils.Scale:R} "
            + "selectorOwner=generation originalBoundaryCapsules=True globalBuffer=False originalChordIntervals=True");
        for (int visit = 0; visit < source.Count; visit++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            TaggedToolpath path = source[visit];
            sourceVertices += path.Points.Count;
            if (path.Points.Count < 2) { discardedVisits++; continue; }
            long visitStarted = Stopwatch.GetTimestamp(), visitQueries = 0;
            double cutDepth = -path.Points[0].Z;
            double reach = profile.RadiusAtDepth(cutDepth) + stockTransitionMargin
                + VCarvePreparation.GeometricErrorBound + selectionReserve;
            if (!depths.TryGetValue(cutDepth, out var metrics))
                diagnostic?.Invoke($"stage=protected-clearing-selection event=depth-start depth={cutDepth:R} "
                    + $"reach={reach:R} selectorReuse=True globalBuffer=False cancellation=every-256-chords-and-candidate-edges");
            bool entireVisit = true;
            int firstOutput = output.Count;
            List<Point3D>? stroke = null;
            void Flush()
            {
                if (stroke is not null && stroke.Count >= 2)
                    output.Add(new TaggedToolpath(stroke.ToArray(), Array.Empty<ToolpathSpan>(),
                        path.RegionIndex, path.Category, path.DepthPassIndex, path.VisitIdentity, detectCurves: false));
                stroke = null;
            }
            for (int chord = 1; chord < path.Points.Count; chord++)
            {
                if ((queriedChords++ & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                visitQueries++;
                Point3D a = path.Points[chord - 1], b = path.Points[chord];
                var intervals = selector.ContactIntervals(new PointD(a.X, a.Y), new PointD(b.X, b.Y),
                    reach, cancellationToken, out var queryStatistics);
                if (queryStatistics.MidpointClassified)
                { if (intervals.Count == 0) midpointOutside++; else midpointInside++; }
                else localQueries++;
                candidateEdges += queryStatistics.CandidateEdges;
                if (queryStatistics.BoundaryClassified) boundaryClassified++;
                bool entireChord = intervals.Count == 1 && intervals[0].Start == 0 && intervals[0].End == 1;
                if (entireChord) wholeChords++;
                else
                {
                    entireVisit = false;
                    if (intervals.Count == 0) discardedChords++; else partialChords++;
                }
                Point3D At(double at) => at == 0 ? a : at == 1 ? b
                    : new Point3D(a.X + (b.X - a.X) * at, a.Y + (b.Y - a.Y) * at, a.Z + (b.Z - a.Z) * at);
                if (intervals.Count == 0) { Flush(); continue; }
                foreach (var interval in intervals)
                {
                    if (interval.Start > 0) Flush();
                    stroke ??= new List<Point3D> { At(interval.Start) };
                    Point3D end = At(interval.End);
                    if (stroke.Count == 1 || stroke[^1] != end) stroke.Add(end);
                    if (interval.End < 1) Flush();
                }
            }
            if (entireVisit)
            {
                // All original points, moves, tags and visit identity survive verbatim.
                output.Add(path); wholeVisits++;
            }
            else
            {
                Flush();
                if (output.Count == firstOutput) discardedVisits++; else partialVisits++;
            }
            bool retained = output.Count > firstOutput;
            metrics = (metrics.Visits + 1, metrics.Retained + (retained ? 1 : 0), metrics.Chords + visitQueries,
                metrics.Milliseconds + Stopwatch.GetElapsedTime(visitStarted).TotalMilliseconds);
            depths[cutDepth] = metrics;
            if ((visit + 1) % 64 == 0 || visit + 1 == source.Count)
                diagnostic?.Invoke($"stage=protected-clearing-selection event=progress completedVisits={visit + 1} "
                    + $"sourceVisits={source.Count} queriedChords={queriedChords} wholeChords={wholeChords} "
                    + $"partialChords={partialChords} discardedChords={discardedChords} outputVisits={output.Count} "
                    + $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
        }
        cancellationToken.ThrowIfCancellationRequested();
        foreach (var pair in depths)
            diagnostic?.Invoke($"stage=protected-clearing-selection event=depth-complete depth={pair.Key:R} "
                + $"sourceVisits={pair.Value.Visits} retainedSourceVisits={pair.Value.Retained} "
                + $"queriedChords={pair.Value.Chords} elapsedMs={pair.Value.Milliseconds:F1} selectorReuse=True");
        outputVertices = output.Sum(path => (long)path.Points.Count);
        diagnostic?.Invoke($"stage=protected-clearing-selection event=complete elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
            + $"sourceVisits={source.Count} wholeVisits={wholeVisits} partialVisits={partialVisits} discardedVisits={discardedVisits} "
            + $"outputVisits={output.Count} sourceVertices={sourceVertices} outputVertices={outputVertices} "
            + $"queriedChords={queriedChords} wholeChords={wholeChords} partialChords={partialChords} discardedChords={discardedChords} "
            + $"midpointInside={midpointInside} midpointOutside={midpointOutside} boundaryClassified={boundaryClassified} "
            + $"localQueries={localQueries} candidateEdges={candidateEdges} "
            + $"distinctDepths={depths.Count} globalBuffer=False newArtworkWalls=False farOceanVisits=False");
        return output;
    }

    private static string DescribeBoundary(IReadOnlyList<IReadOnlyList<PointD>> rings)
    {
        if (rings.Count == 0)
            return "rings=0 points=0";
        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        long count = 0;
        double signedArea = 0;
        foreach (IReadOnlyList<PointD> ring in rings)
        {
            signedArea += Clipper.Area(new PathD(ring));
            foreach (PointD point in ring)
            {
                minX = Math.Min(minX, point.x); minY = Math.Min(minY, point.y);
                maxX = Math.Max(maxX, point.x); maxY = Math.Max(maxY, point.y);
                count++;
            }
        }
        return $"rings={rings.Count} points={count} bounds=({minX:R},{minY:R})-({maxX:R},{maxY:R}) " +
            $"size=({maxX - minX:R},{maxY - minY:R}) signedArea={signedArea:R}";
    }

    private static string DescribeOutput(IReadOnlyList<TaggedToolpath> paths)
    {
        double minZ = double.PositiveInfinity, maxZ = double.NegativeInfinity;
        long nonFinitePoints = 0;
        foreach (TaggedToolpath path in paths)
        foreach (Point3D point in path.Points)
        {
            if (!double.IsFinite(point.X) || !double.IsFinite(point.Y) || !double.IsFinite(point.Z))
                nonFinitePoints++;
            minZ = Math.Min(minZ, point.Z); maxZ = Math.Max(maxZ, point.Z);
        }
        return $"paths={paths.Count} zRange=({minZ:R},{maxZ:R}) nonFinitePoints={nonFinitePoints}";
    }

    internal static double AppliedStockAllowance(double stockToLeave) => stockToLeave <= 0.0001 ? 0 : stockToLeave;

    internal static List<IReadOnlyList<PointD>> BuildShoulderBoundary(MillingRegion region,double stockToLeave)
        // These contours nominate contact rays, not permission to cut. Keep
        // nomination error within half the existing surface budget. Projection
        // and inward direction use the ORIGINAL filled target; each fitted
        // native/F3 cutter sweep still earns its own complete physical proof.
        => PolygonOps.OffsetBoundary(region.Outer,region.Holes,
            -(stockToLeave+VCarveMaterialSurface.PlanarReserve),
            arcTolerance:1/PathUtils.Scale,simplifyTolerance:MachiningAccuracy.SurfaceDeviation/2)
            .Select(r=>(IReadOnlyList<PointD>)r).ToList();

    internal static List<IReadOnlyList<PointD>> BuildBoundary(MillingRegion region, double stockToLeave)
    {
        List<PointD> outer = NormalizeRing(region.Outer, counterClockwise: true);
        if (outer.Count < 3)
            return new List<IReadOnlyList<PointD>>();

        var holes = new List<IReadOnlyList<PointD>>(region.Holes.Count);
        foreach (IReadOnlyList<PointD> hole in region.Holes)
        {
            List<PointD> normalizedHole = NormalizeRing(hole, counterClockwise: false);
            if (normalizedHole.Count >= 3)
                holes.Add(normalizedHole);
        }

        if (AppliedStockAllowance(stockToLeave) == 0)
        {
            var original = new List<IReadOnlyList<PointD>>(1 + holes.Count) { outer };
            original.AddRange(holes);
            return original;
        }

        return PolygonOps.OffsetBoundary(
                outer,
                holes,
                delta: -stockToLeave,
                arcTolerance: 0.25,
                simplifyTolerance: 0.25)
            .Where(ring => ring.Count >= 3)
            .Select(ring => (IReadOnlyList<PointD>)ring)
            .ToList();
    }

    private static List<PointD> NormalizeRing(IReadOnlyList<PointD> source, bool counterClockwise)
    {
        int count = source.Count;
        if (count > 1 && SamePoint(source[0], source[^1]))
            count--;

        var ring = new List<PointD>(count);
        for (int index = 0; index < count; index++)
            ring.Add(source[index]);

        if (ring.Count >= 3 && (Clipper.Area(new PathD(ring)) > 0) != counterClockwise)
            ring.Reverse();

        return ring;
    }

    private static List<Point3D> CompactAndClamp(
        IReadOnlyList<Point3D> source,
        double surfaceZ,
        double maximumDepth,
        bool preserveCertifiedChords = false, bool preserveAllVertices = false)
    {
        var compacted = new List<Point3D>(source.Count);
        foreach (Point3D point in source)
        {
            var candidate = new Point3D(
                point.X,
                point.Y,
                surfaceZ + Math.Clamp(point.Z, -maximumDepth, 0.0));
            // Smooth wall certification is on float/F3 chords. A nominally
            // collinear point may round off that line and own necessary physical
            // clearance; coalescing it would invalidate the commanded proof.
            while (!preserveCertifiedChords && compacted.Count >= 2
                && IsRedundant(compacted[^2], compacted[^1], candidate))
            {
                compacted[^1] = candidate;
            }

            if (preserveAllVertices || compacted.Count == 0 || !SamePoint(compacted[^1], candidate))
                compacted.Add(candidate);
        }

        return compacted;
    }

    internal static bool HasCommandedPenetration(IReadOnlyList<Point3D> points, double surfaceZ)
        => points.Any(point => Math.Round((float)surfaceZ - (float)point.Z, 3,
            MidpointRounding.AwayFromZero) > 0);

    private static bool IsRedundant(Point3D start, Point3D middle, Point3D end)
    {
        double dx = end.X - start.X;
        double dy = end.Y - start.Y;
        double dz = end.Z - start.Z;
        double lengthSquared = dx * dx + dy * dy + dz * dz;
        if (lengthSquared <= 1e-24)
            return false;

        double mx = middle.X - start.X;
        double my = middle.Y - start.Y;
        double mz = middle.Z - start.Z;
        double projection = (mx * dx + my * dy + mz * dz) / lengthSquared;
        if (projection <= 1e-12 || projection >= 1.0 - 1e-12)
            return false;

        double closestX = start.X + projection * dx;
        double closestY = start.Y + projection * dy;
        double closestZ = start.Z + projection * dz;
        double errorX = middle.X - closestX;
        double errorY = middle.Y - closestY;
        double errorZ = middle.Z - closestZ;
        double tolerance = 1e-8 * Math.Max(1.0, Math.Sqrt(lengthSquared));
        return errorX * errorX + errorY * errorY + errorZ * errorZ <= tolerance * tolerance;
    }

    private static double PathLength(IReadOnlyList<Point3D> points)
    {
        double length = 0;
        for (int index = 1; index < points.Count; index++)
        {
            double dx = points[index].X - points[index - 1].X;
            double dy = points[index].Y - points[index - 1].Y;
            double dz = points[index].Z - points[index - 1].Z;
            length += Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        return length;
    }

    private static bool SamePoint(PointD first, PointD second)
        => Math.Abs(first.x - second.x) <= 1e-12
            && Math.Abs(first.y - second.y) <= 1e-12;

    private static bool SamePoint(Point3D first, Point3D second)
        => Math.Abs(first.X - second.X) <= 1e-12
            && Math.Abs(first.Y - second.Y) <= 1e-12
            && Math.Abs(first.Z - second.Z) <= 1e-12;
}
