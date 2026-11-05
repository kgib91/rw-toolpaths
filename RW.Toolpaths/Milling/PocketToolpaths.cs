using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Configuration for concentric-offset pocket clearing.</summary>
public sealed record PocketOptions
{
    /// <summary>Common machine rates, transfer plane and optional earlier-stock proof.
    /// Null retains geometric routing when machine context is unavailable.</summary>
    public MillingMotionSettings? Motion { get; init; }
    public MillingBoundaryAccess? BoundaryAccess { get; init; }
    internal IReadOnlyList<IReadOnlyList<PointD>>? KnownEmptyBoundary { get; init; }
    internal double KnownEmptyFloor { get; init; } = double.PositiveInfinity;
    /// <summary>
    /// Restricts roughing to a flat-floor plateau in this pocket's continuous
    /// grade envelope. Later operations are not inputs.
    /// </summary>
    public bool SmoothRoughing { get; init; }

    /// <summary>Maximum grade in degrees used by this pocket's own omission analysis.</summary>
    public double SmoothRampAngleDegrees { get; init; } = 30;
    public bool RampOutsideEdges { get; init; }

    /// <summary>Radial engagement as a fraction of tool diameter; 0.4 removes 40% per pass.</summary>
    public double StepOver { get; init; } = 0.4;

    /// <summary>Signed radial wall offset after cutter compensation. Positive leaves material;
    /// negative overcuts, and values below minus the tool radius start outside the source boundary.</summary>
    public double StockToLeave { get; init; }

    /// <summary>Work from the centre outward instead of from the wall inward.</summary>
    public bool InsideOut { get; init; }

    /// <summary>Cap on concentric rings per region.</summary>
    public int MaxRings { get; init; } = RingOffsetEngine.AbsoluteRingLimit;

    public MillingDirection Direction { get; init; } = MillingDirection.Climb;

    public RampSettings Ramp { get; init; } = RampSettings.FromRatio(2.0);

    public GeometryTolerances Tolerances { get; init; } = GeometryTolerances.Default;

    /// <summary>
    /// Cut nested rings as one continuous spiral rather than lifting between them.
    /// Turn off when something downstream needs each ring as a discrete loop.
    /// </summary>
    public bool SpiralChaining { get; init; } = true;

    /// <summary>Optional boundary that rings are trimmed to, normally the stock extents.</summary>
    public IReadOnlyList<PointD>? ClipBoundary { get; init; }

    /// <summary>
    /// Vets every ring-to-ring link before it is cut. Left null, pocketing derives one from the
    /// region it is clearing; supply an implementation to widen it with knowledge of what earlier
    /// operations already removed.
    /// </summary>
    public ILinkClearance? LinkClearance { get; init; }
    /// <summary>Fit optional curve metadata. Linear-only consumers can avoid this work.</summary>
    public bool DetectCurves { get; init; } = true;
    /// <summary>Produce the optional 2D swept-area summary. Physical stock simulation
    /// can consume the emitted paths directly without building this polygon union.</summary>
    public bool CalculateSweptArea { get; init; } = true;

    /// <summary>
    /// Source-space material excluded from this pocket's physical cutter sweep.
    /// They are removed before cutter compensation, independently of wall allowance.
    /// Null preserves ordinary pocket generation.
    /// </summary>
    public IReadOnlyList<IReadOnlyList<PointD>>? ProtectedBoundary { get; init; }

    /// <summary>Pocket-specific posted-depth replay uses immutable original
    /// command vertices. Other callers of the shared emitter retain defaults.</summary>
    internal bool PreserveCommandVertices { get; init; }

    /// <summary>Dense raster construction uses local, independently safe entries
    /// instead of rebuilding a large executed-stock continuation network.</summary>
    internal bool IndependentRasterEntries { get; init; }

    /// <summary>Use finite world-chord certificates for this pocket's complete
    /// original medial graph. Indirect strategies retain their own emitter contract.</summary>
    internal bool BoundOriginalSourceMedial { get; init; } = true;

    /// <summary>Only adaptive OffsetPocket selection can defer mutual raw-front
    /// footprint comparison to the mandatory final whole-shell audit.</summary>
    internal bool AdaptiveCompactionProposals { get; init; } = true;

    /// <summary>Raw adaptive trials may nominate with filled dilation; actual
    /// selected commands retain the original complete physical shell proof.</summary>
    internal bool TrialFilledSweepProposals { get; init; } = true;

    /// <summary>Bounded parent-local open rescue may retain the requested closed
    /// inset after the original complete physical shell proof succeeds.</summary>
    internal bool ParentLocalRescueProposals { get; init; } = true;

    /// <summary>Request the bounded complete reachable-stock proof before the
    /// eager original graph; unresolved proofs retain the full graph.</summary>
    internal bool ExactFirstCoverageProposals { get; init; } = true;

    /// <summary>Use the certified dense original-boundary arc construction for Pocket alone.</summary>
    internal bool DenseFirstArcConstruction { get; init; } = true;

    /// <summary>Keep fixed shallow and deep construction aids, with original walls and whole-shell proofs.</summary>
    internal bool TwoOriginConstruction { get; init; } = true;
    /// <summary>Only the initial dense original Pocket source can use independently
    /// certified filled-hole dilation. Every later trial retains its fixed source.</summary>
    internal bool SeparateOriginalFirstConstruction { get; init; } = true;
}

/// <summary>Counters describing what a strategy produced, for logging and regression checks.</summary>
public sealed record ToolpathDiagnostics
{
    /// <summary>Actual discontinuous XYZ visits after final shared routing,
    /// including the operation's first entry. Independent of source ring count.</summary>
    public int ExecutionEntryCount { get; init; }
    public int ExecutionTransferCount { get; init; }
    public int RingCount { get; init; }
    /// <summary>Open terminal wavefront paths retained by the shared fill geometry.</summary>
    public int StripCount { get; init; }
    public int IslandCount { get; init; }
    public int PathCount { get; init; }
    public int DepthPassCount { get; init; }

    /// <summary>Total distance travelled while cutting.</summary>
    public double CutLength { get; init; }

    /// <summary>Straight-line distance between the end of one path and the start of the next.</summary>
    public double LinkLength { get; init; }

    /// <summary>Ring-to-ring links refused by the clearance test and turned into a lift.</summary>
    public int LinkLifts { get; init; }

    /// <summary>Potential ring-entry points checked by the link-clearance policy.</summary>
    public int LinkCandidatesTested { get; init; }

    /// <summary>Stay-down links recovered by selecting a safe point other than the nearest candidate.</summary>
    public int AlternativeLinks { get; init; }

    public int PlungeEntries { get; init; }
    public int LinearRampEntries { get; init; }
    public int HelicalEntries { get; init; }

    /// <summary>Closed profile loops taken through multiple scheduled depths without retracting.</summary>
    public int DepthFirstSpirals { get; init; }

    /// <summary>Total scheduled depth passes represented by those continuous loops.</summary>
    public int PassesCombined { get; init; }

    /// <summary>Between-pass retracts removed by continuous depth-first loops.</summary>
    public int AvoidedRetracts { get; init; }
}

/// <summary>Everything one strategy produced for one operation.</summary>
public sealed class ToolpathPlan
{
    internal ToolpathPlan(
        List<TaggedToolpath> toolpaths,
        List<List<PointD>> sweptArea,
        ToolpathDiagnostics diagnostics,
        HoldingTabReport? tabReport = null)
    {
        Toolpaths = toolpaths;
        SweptArea = sweptArea;
        int entries=0;Point3D? previous=null;
        double cutting=0,ramping=0,linking=0,travel=0;
        foreach(var path in toolpaths)
        {
            if(path.Points.Count<2)continue;
            if(previous!=path.Points[0])entries++;
            previous=path.Points[^1];
            var spans=path.Spans.Count>0 ? path.Spans : new[]{new ToolpathSpan(0,path.Points.Count-1,ToolpathSpanKind.Cut)};
            foreach(var span in spans)
            for(int i=span.StartIndex+1;i<=span.EndIndex;i++)
            {
                double length=ToolpathContinuationPlanner.Distance(path.Points[i-1],path.Points[i]);
                switch(span.Kind)
                {
                    case ToolpathSpanKind.Cut:cutting+=length;break;
                    case ToolpathSpanKind.Ramp:ramping+=length;break;
                    case ToolpathSpanKind.Link:linking+=length;break;
                    case ToolpathSpanKind.ClearedTravel:travel+=length;break;
                }
            }
        }
        Diagnostics = diagnostics with{ExecutionEntryCount=entries,ExecutionTransferCount=Math.Max(0,entries-1)};
        if(toolpaths.Count>0)Console.WriteLine($"[ToolpathPlan.Execution] paths={toolpaths.Count} entries={entries} transfers={Math.Max(0,entries-1)} "
            +$"cutLength={cutting:R} rampLength={ramping:R} linkLength={linking:R} clearedTravelLength={travel:R} coordinateSpace=native finalRouting=True");
        TabReport = tabReport;
    }

    /// <summary>Continuous paths in cut order. Callers insert their own rapids between them.</summary>
    public IReadOnlyList<TaggedToolpath> Toolpaths { get; }

    /// <summary>Optional whole physical native/posted clearance authority for
    /// a downstream replacement chord, in the plan's workspace coordinates.</summary>
    public Func<Point3D,Point3D,bool>? ReplacementChordGuard { get; internal init; }
    /// <summary>Native-only clearance against the same immutable physical stock
    /// and target used by construction; excludes controller quantization.</summary>
    public Func<Point3D,Point3D,bool>? NativeReplacementChordGuard { get; internal init; }
    public Func<ToolMove,bool>? NativeReplacementCurveGuard { get; internal init; }


    /// <summary>Optional positive full-cutter clearance authority against immutable
    /// earlier posted stock. Applies to literal FINAL approach chords in workspace
    /// coordinates; legality alone never authorizes faster travel.</summary>
    public Func<Point3D,Point3D,bool>? ClearedTravelGuard { get; internal init; }

    /// <summary>Area the cutter sweeps, for stock tracking by later operations.</summary>
    public IReadOnlyList<List<PointD>> SweptArea { get; }

    public ToolpathDiagnostics Diagnostics { get; }

    /// <summary>Holding-tab outcome; <c>null</c> when the operation does not use tabs.</summary>
    public HoldingTabReport? TabReport { get; }

    internal static ToolpathPlan Empty { get; } =
        new(new List<TaggedToolpath>(), new List<List<PointD>>(), new ToolpathDiagnostics());
}

/// <summary>
/// Clears the interior of closed regions with concentric offset rings.
///
/// <para>
/// Depth passes are nested inside islands rather than the other way round: an island is taken all
/// the way to final depth before the cutter moves on. Sweeping every island at one depth before
/// stepping down would mean re-entering each pocket once per pass, and chips would be pushed back
/// under the cutter on every re-entry.
/// </para>
/// </summary>
public static class PocketToolpaths
{
    /// <summary>Category tag applied to pocket passes.</summary>
    public const string Category = "pocket";

    /// <summary>
    /// Limits this pocket's physical clearance graph and preserves all material
    /// above its flat-floor plateau. No later operation or cutter is an input.
    /// The exclusion is applied before complete physical cutter compensation.
    /// </summary>
    public static VCarveRoughingAnalysis AnalyzeSmoothRoughing(
        IReadOnlyList<MillingRegion> regions, ToolGeometry tool, DepthSchedule depth,
        PocketOptions options, IMedialAxisProvider provider,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(regions);
        ArgumentNullException.ThrowIfNull(tool);
        ArgumentNullException.ThrowIfNull(depth);
        ArgumentNullException.ThrowIfNull(options);
        ArgumentNullException.ThrowIfNull(provider);
        if (!options.SmoothRoughing)
            throw new ArgumentException("Own-pocket omission analysis requires Smooth roughing enabled.", nameof(options));
        if (tool.VCarveProfile is null && tool.ConeLength is null)
            throw new ArgumentException("Own-pocket omission analysis requires the roughing cutter's physical axial profile.", nameof(tool));
        if (!double.IsFinite(tool.Radius) || tool.Radius <= 0)
            throw new ArgumentOutOfRangeException(nameof(tool));
        if (!double.IsFinite(depth.Depth) || depth.Depth <= 0)
            throw new ArgumentOutOfRangeException(nameof(depth));
        long started = System.Diagnostics.Stopwatch.GetTimestamp();
        Console.WriteLine($"[Pocket.SmoothRoughing] event=analysis-start owner=current-pocket "
            + $"regions={regions.Count} physicalToolRadius={tool.Radius:R} ownFloor={depth.Depth:R} "
            + $"ownRampAngleDegrees={options.SmoothRampAngleDegrees:R} ownEdgeOffset={options.StockToLeave:R} "
            + $"futureOperationsRead=False retainedRoughingStrategyUnchanged=True");
        var preparation = VCarveToolpaths.Prepare(regions, tool, depth,
            new VCarveOptions
            {
                SmoothVCarving = true, SmoothRampAngleDegrees = options.SmoothRampAngleDegrees,
                RampOutsideEdges = options.RampOutsideEdges,
                BoundaryAccess = options.BoundaryAccess,
                Ramp = options.Ramp, StockToLeave = 0,
                StepOver = options.StepOver, IncludeInteriorFill = true,
                DetectCurves = false, CalculateSweptArea = false,
                DiagnosticContext = "own-pocket-smooth-roughing",
                PreparationContactRadius = tool.VCarveProfile?.TipRadius ?? 0,
            }, provider, cancellationToken);
        var analysis = preparation.AnalyzePocketPlateau(depth.Depth, options.SmoothRampAngleDegrees,
            options.StockToLeave, cancellationToken, retainBasinDiagnostics: true);
        cancellationToken.ThrowIfCancellationRequested();
        Console.WriteLine($"[Pocket.SmoothRoughing] event=analysis-complete owner=current-pocket "
            + $"elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
            + $"features={analysis.Basins.Count} omittedFeatures={analysis.Basins.Count(feature => feature.OmitRoughing)} "
            + $"protectedSources={analysis.ProtectedSourceRegionIndices.Count} protectedArea={analysis.ProtectedArea:R} "
            + $"preparation={preparation.Fingerprint} originalGeometry={preparation.OriginalGeometryFingerprint} "
            + $"futureOperationsRead=False analysisComplete=True policy=complete-footprint-flat-plateau");
        return analysis;
    }

    public static ToolpathPlan Generate(
        IReadOnlyList<MillingRegion> regions,
        ToolGeometry tool,
        DepthSchedule depth,
        PocketOptions options,
        CancellationToken cancellationToken = default,
        Action<double, string>? progress = null)
    {
        long t0 = PerfLog.Start();
        // Capture before protection/prepared geometry can consult shared evidence.
        // Later environment changes cannot change this operation's proof mode.
        options = CaptureExactFirstCoverageMode(CaptureParentLocalRescueMode(CaptureTrialFilledSweepMode(CaptureConcentricConstructionMode(CaptureAdaptiveCompactionMode(options)))));
        using var feasibilityDiagnostics = InsetFeasibilityDiagnostics.Begin("pocket");

        if (!double.IsFinite(options.StockToLeave))
            throw new ArgumentOutOfRangeException(nameof(options), "Pocket edge offset must be finite.");

        if (regions.Count == 0 || tool.Radius <= 1e-6 || depth.Depth <= 0)
            return ToolpathPlan.Empty;

        // Dense ramp exclusions do not need concentric terminal reconstruction.
        // A flat cutter clears the same plateau with raster strips plus the
        // compensated perimeter; all existing footprint / access proofs remain.
        long protectedPoints=options.ProtectedBoundary?.Sum(p=>(long)p.Count)??0;
        bool uniformFlat=PocketContinuationRouter.TryFlatCylinderRadius(tool,
            PocketContinuationRouter.RequiredPostedCylinderHeight(depth),out _);
        if(options.SmoothRoughing && protectedPoints>=20_000)
            Console.WriteLine($"[Pocket.SurfaceStrategy.Selection] protectedPoints={protectedPoints} uniformFlat={uniformFlat} "
                +$"assignedRadius={tool.Radius:R} physicalTipRadius={tool.VCarveProfile?.TipRadius:R} radiusIdentity=binary32-assignment-and-uniform-physical-section maxRings={options.MaxRings}");
        if(options.SmoothRoughing && options.MaxRings>1 && options.StockToLeave>=0
            && uniformFlat && protectedPoints>=20_000)
        {
            double width=regions.Max(r=>r.Outer.Max(p=>p.x))-regions.Min(r=>r.Outer.Min(p=>p.x));
            double height=regions.Max(r=>r.Outer.Max(p=>p.y))-regions.Min(r=>r.Outer.Min(p=>p.y));
            double angle=height>width?90:0;
            Console.WriteLine($"[Pocket.SurfaceStrategy] policy={MachiningAccuracy.Policy} strategy=raster-with-compensated-perimeter "
                +$"reason=dense-protected-flat-plateau protectedPoints={protectedPoints} angle={angle:R} "
                +$"originalConcentricPathsRequired=False authoredFloorStepdownAndAllowanceRetained=True fullPhysicalProtectionRequired=True");
            progress?.Invoke(0,"Clearing the protected roughing plateau");
            var raster=ScanlinePocketToolpaths.Generate(regions,tool,depth,options with {
                IndependentRasterEntries=true,
                Tolerances=options.Tolerances with {ArcTolerance=Math.Min(.001,options.Tolerances.ArcTolerance),
                    SimplifyTolerance=Math.Min(.001,options.Tolerances.SimplifyTolerance)} },angle,cancellationToken);
            progress?.Invoke(1,"Pocket roughing complete");
            return raster;
        }

        var access = options.BoundaryAccess?.Resolve(regions, tool, depth, options.SmoothRoughing,
            options.SmoothRampAngleDegrees, options.StockToLeave, cancellationToken);
        if (access != null)
        {
            regions = access.PocketRegions(tool.Radius, options.StockToLeave, cancellationToken);
            options=options with {KnownEmptyBoundary=access.CertifiedEmptyBoundary(PocketContinuationRouter.KnownAirReserve,cancellationToken),
                KnownEmptyFloor=depth.SurfaceZ-depth.Depth, ClipBoundary=options.ClipBoundary ?? access.PartBoundary};
        }

        int originalRegionCount = regions.Count;
        int protectedContourCount = options.ProtectedBoundary?.Count ?? 0;
        long subtractStart = PerfLog.Start();
        regions = PocketProtection.Subtract(regions, options, cancellationToken, physicalConcentric: true);
        PerfLog.Stop("PocketToolpaths.SubtractProtection", subtractStart,
            $"originalRegions={originalRegionCount} retainedRegions={regions.Count} " +
            $"protectedContours={protectedContourCount}");
        if (regions.Count == 0)
            return ToolpathPlan.Empty;

        double stepOver = tool.Radius * 2.0 * options.StepOver;
        if (stepOver <= 1e-6)
            stepOver = tool.Radius;

        Console.WriteLine($"[PocketToolpaths.EdgeOffset] mode=offset toolRadius={tool.Radius:R} "
            + $"edgeOffset={options.StockToLeave:R} firstInset={tool.Radius + options.StockToLeave:R} "
            + $"stepOver={stepOver:R} regions={regions.Count} "
            + $"adaptiveCompactionProposals={options.AdaptiveCompactionProposals} proofModeCaptured=True "
            + $"denseFirstArcConstruction={options.DenseFirstArcConstruction} twoOriginConstruction={options.TwoOriginConstruction} "
            + $"separateOriginalFirstConstruction={options.SeparateOriginalFirstConstruction} firstHoleModeCaptured=True "
            + $"trialFilledSweepProposals={options.TrialFilledSweepProposals} trialFilledModeCaptured=True "
            + $"parentLocalRescueProposals={options.ParentLocalRescueProposals} parentLocalRescueModeCaptured=True "
            + $"exactFirstCoverageProposals={options.ExactFirstCoverageProposals} exactFirstCoverageModeCaptured=True");

        long collectRingsStart = PerfLog.Start();
        var rings = MedialAxisGeometry.BuildConcentricFill(
            regions,
            new RingOffsetOptions(tool.Radius + options.StockToLeave, stepOver)
            {
                RingLimit = options.MaxRings,
                Tolerances = options.Tolerances,
                MinimumRingArea = 0,
                RetainEmittedSweepHistory = options.ProtectedBoundary is { Count: > 0 } || options.KnownEmptyBoundary is {Count:>0},
                KnownEmptyBoundary=options.KnownEmptyBoundary,
                BoundOriginalSourceMedial = options.BoundOriginalSourceMedial,
                PreserveSourceNativeGrid = options.ProtectedBoundary is { Count: > 0 },
                SelectedCleanupProposals = options.AdaptiveCompactionProposals,
                TrialFilledSweepProposals = options.TrialFilledSweepProposals,
                TrialFilledSweepModeCaptured = true,
                ParentLocalRescueProposals = options.ParentLocalRescueProposals,
                ParentLocalRescueModeCaptured = true,
                ExactFirstCoverageProposals = options.ExactFirstCoverageProposals,
                ExactFirstCoverageModeCaptured = true,
                DenseFirstArcConstruction = options.DenseFirstArcConstruction,
                TwoOriginConstruction = options.TwoOriginConstruction,
                SeparateOriginalFirstConstruction = options.SeparateOriginalFirstConstruction,
            },
            tool.Radius, preserveTerminalPaths: true, options.ClipBoundary,
            cancellationToken, (fraction, stage) => progress?.Invoke(0.65 * fraction, stage));

        if (options.ClipBoundary is { Count: >= 3 })
            rings = ClipRings(rings, options.ClipBoundary, cancellationToken);

        if (access != null) rings = access.TrimRings(rings, tool.Radius, cancellationToken);

        PerfLog.Stop("PocketToolpaths.CollectRings", collectRingsStart,
            $"regions={regions.Count} rings={rings.Count} clipPoints={options.ClipBoundary?.Count ?? 0} " +
            "scope=concentric-fill-and-clipping child=RingOffsetEngine.BuildRings");

        if (rings.Count == 0)
            return ToolpathPlan.Empty;

        progress?.Invoke(0.65, "Ordering pocket rings");
        RingOffsetEngine.BuildNesting(rings);
        var traversal = RingOffsetEngine.BuildTraversal(rings, options.InsideOut);

        // The level-zero center domain remains a routing restriction, but its
        // boundary-growth allowance is not physical wall/stock-leave authority.
        // Every offset link must also meet the original-source chord contract.
        var callerClearance = options.LinkClearance;
        if (!rings.Any(path => !path.IsClosed)) callerClearance ??= BuildTravelClearance(rings);
        var sourceClearance = new FillSourceClearance(regions,
            Math.Max(0, tool.Radius + options.StockToLeave), callerClearance, options.ClipBoundary);
        var guarded = options with
        {
            LinkClearance = sourceClearance,
            PreserveCommandVertices = true,
        };
        Console.WriteLine("[Pocket.CommandVertices] event=enabled entryContact=original-vertex "
            + "rebase=original-vertex excursionPause=original-vertex cutStepdownLateralAllowance=0");

        var plan = Emit(rings, traversal, depth, guarded, tool, Category, cancellationToken,
            progress: (fraction, stage) => progress?.Invoke(0.7 + 0.3 * fraction, stage));
        sourceClearance.Report();
        long verifyProtectionStart = PerfLog.Start();
        PocketProtection.Verify(plan.Toolpaths, tool.Radius, depth.SurfaceZ,
            options.ProtectedBoundary, cancellationToken);
        PerfLog.Stop("PocketToolpaths.VerifyProtection", verifyProtectionStart,
            $"nativePaths={plan.Toolpaths.Count} radius={tool.Radius:R} surfaceZ={depth.SurfaceZ:R} " +
            $"protectedContours={protectedContourCount} result={(protectedContourCount > 0 ? "verified" : "skipped")}");
        progress?.Invoke(1, "Pocket complete");
        access?.Verify(plan.Toolpaths,tool,depth,options.StockToLeave,cancellationToken);

        PerfLog.Stop("PocketToolpaths.Generate", t0,
            $"regions={regions.Count} rings={rings.Count} paths={plan.Toolpaths.Count} " +
            $"passes={depth.PassCount} cut={plan.Diagnostics.CutLength:F1} " +
            $"link={plan.Diagnostics.LinkLength:F1} lifts={plan.Diagnostics.LinkLifts}");

        return plan;
    }

    internal static PocketOptions CaptureAdaptiveCompactionMode(PocketOptions options)
        => options with { AdaptiveCompactionProposals = options.AdaptiveCompactionProposals
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_ADAPTIVE_COMPACTION_REFERENCE") != "1" };

    internal static PocketOptions CaptureTrialFilledSweepMode(PocketOptions options)
        => options with { TrialFilledSweepProposals = options.TrialFilledSweepProposals
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_TRIAL_FILLED_SWEEP_REFERENCE") != "1"
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1" };

    internal static PocketOptions CaptureParentLocalRescueMode(PocketOptions options)
        => options with { ParentLocalRescueProposals = options.ParentLocalRescueProposals
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_PARENT_LOCAL_RESCUE_REFERENCE") != "1"
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1" };

    internal static PocketOptions CaptureExactFirstCoverageMode(PocketOptions options)
        => options with { ExactFirstCoverageProposals = options.ExactFirstCoverageProposals
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_EXACT_FIRST_COVERAGE_REFERENCE") != "1"
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1"
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_LOCAL_MEDIAL_REFERENCE") != "1" };

    internal static PocketOptions CaptureConcentricConstructionMode(PocketOptions options)
    {
        bool reference = Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") == "1";
        return options with {
            DenseFirstArcConstruction = options.DenseFirstArcConstruction && !reference
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_FIRST_ARC_REFERENCE") != "1",
            TwoOriginConstruction = options.TwoOriginConstruction && !reference
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_REFERENCE") != "1",
            SeparateOriginalFirstConstruction = options.SeparateOriginalFirstConstruction && !reference
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_FIRST_HOLE_EROSION_REFERENCE") != "1",
        };
    }

    /// <summary>
    /// The cutter centre may go anywhere the outermost ring can be cut. Level-zero rings are
    /// already the region eroded by cutter radius plus stock to leave, including clipping, so
    /// re-offsetting the source here would repeat the most expensive geometry operation.
    /// </summary>
    private static ILinkClearance BuildTravelClearance(
        IReadOnlyList<OffsetRing> rings)
    {
        long t0 = PerfLog.Start();
        var travel = rings
            .Where(ring => ring.Level == 0 && ring.IsClosed)
            .Select(ring =>
            {
                var points = new List<PointD>(ring.Points);
                if (ring.IsHole)
                    points.Reverse();
                return points;
            })
            .ToList();

        // Union resolves overlap between source regions while preserving Clipper's hole winding.
        if (rings.Select(ring => ring.RegionIndex).Distinct().Take(2).Count() > 1)
            travel = PolygonOps.Union(travel);

        PerfLog.Stop("PocketToolpaths.BuildTravelClearance", t0,
            $"contours={travel.Count} holes={travel.Count(path => PolygonOps.SignedArea(path) < 0)}");
        return LinkClearance.WithinRegion(travel);
    }

    /// <summary>
    /// Walks islands in travel-optimised order, taking each one to full depth before moving on.
    /// Shared by pocketing and profiling; profiling supplies its own pre-built rings.
    /// </summary>
    internal static ToolpathPlan Emit(
        IReadOnlyList<OffsetRing> rings,
        RingTraversal traversal,
        DepthSchedule depth,
        PocketOptions options,
        ToolGeometry tool,
        string category,
        CancellationToken cancellationToken,
        Func<int, List<Point3D>, List<ToolpathSpan>, (List<Point3D> Points, List<ToolpathSpan> Spans)>? applyTabs = null,
        Action<double, string>? progress = null)
    {
        long t0 = PerfLog.Start();
        var toolpaths = new List<TaggedToolpath>();

        double cutLength = 0;
        int emittedPointCount = 0;
        int plunges = 0, linearRamps = 0, helicals = 0;

        var islandOrder = OrderIslands(rings, traversal, cancellationToken);
        Point3D? previousEnd = null;
        double linkLength = 0;
        int linkLifts = 0;
        int linkCandidatesTested = 0;
        int alternativeLinks = 0;
        int radialDepthRamps = 0;
        int planarRingTransitions = 0;
        int straightTransitionFallbacks = 0;
        var routedRingIndices = new HashSet<int>();

        var regionRoutes = islandOrder
            .GroupBy(islandIndex =>
            {
                var (start, _) = traversal.Islands[islandIndex];
                return rings[traversal.Order[start]].RegionIndex;
            })
            .Select(group => group.ToList())
            .ToList();

        int completedRings = 0;
        double totalRings = Math.Max(1d, (double)rings.Count * depth.PassCount);
        foreach (var regionIslands in regionRoutes)
        {
            cancellationToken.ThrowIfCancellationRequested();

            var chains = new List<RingChain>();
            foreach (int islandIndex in regionIslands)
            {
                var (start, count) = traversal.Islands[islandIndex];
                var islandRings = new List<int>(count);
                for (int i = 0; i < count; i++)
                    islandRings.Add(traversal.Order[start + i]);

                chains.AddRange(options.SpiralChaining
                    ? RingOffsetEngine.BuildChains(rings, islandRings)
                    : islandRings.Select(r => new RingChain(new List<int> { r })));
            }

            // A hole's shallow rim can be separated from the outer fronts by
            // virgin stock. Finish the material fronts, including their corner
            // branches, before entering a hole at its deepest adjacent front.
            // Only the visit order changes; each contour keeps its commanded
            // cutting direction and all of its native edges.
            var holeChains = chains.Where(chain => chain.RingIndices.All(index =>
                rings[index].IsClosed && rings[index].IsHole)).ToList();
            if (category == Category && holeChains.Count > 0)
            {
                chains = chains.Except(holeChains).Concat(holeChains.Select(chain =>
                    new RingChain(chain.RingIndices.OrderByDescending(index => rings[index].Level).ToList()))).ToList();
                Console.WriteLine($"[PocketToolpaths.HoleFrontOrder] region={rings[chains[0].RingIndices[0]].RegionIndex} "
                    + $"holeChains={holeChains.Count} order=material-fronts-then-deep-hole-fronts "
                    + $"insideOut={options.InsideOut} temporalProof=final-order");
            }

            foreach (var chain in chains)
            {
                foreach (int ringIndex in chain.RingIndices)
                    routedRingIndices.Add(ringIndex);
            }
            Point3D? regionEnd = previousEnd;

            for (int pass = 0; pass < depth.PassCount; pass++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                double topZ = depth.PassTopZ(pass);
                double bottomZ = depth.PassBottomZ(pass);
                // The authored inward/outward order applies at every depth. An
                // open terminal trace is not a closed seam to reverse on odd passes.
                // Ring dependencies live inside each chain. Independent chains
                // at this same floor can be visited from the current exit,
                // retaining the material-front-before-hole-front barrier.
                var passChains = new List<RingChain>(chains.Count);
                Point3D? chainCursor=regionEnd;
                if(chains.Count<8)passChains.AddRange(chains);
                else foreach(var batch in chains.GroupBy(chain=>(
                    Hole:chain.RingIndices.All(index=>rings[index].IsClosed&&rings[index].IsHole),
                    FirstLevel:rings[chain.RingIndices[0]].Level,LastLevel:rings[chain.RingIndices[^1]].Level)))
                {
                    var candidates=batch.ToArray();
                    var visits=candidates.Select(chain=>
                    {
                        var ring=rings[chain.RingIndices[0]];
                        var points=ring.Points.Select(p=>new Point3D(p.x,p.y,bottomZ)).ToList();
                        if(ring.IsClosed&&points[0]!=points[^1])points.Add(points[0]);
                        return new TaggedToolpath(points,Array.Empty<ToolpathSpan>(),0,category,pass,detectCurves:false);
                    }).ToArray();
                    foreach(int next in ToolpathVisitOrder.Order(visits,Enumerable.Repeat(true,visits.Length).ToArray(),chainCursor,cancellationToken))
                    {
                        passChains.Add(candidates[next]);
                        var visit=visits[next];int seam=chainCursor is {} from ? ToolpathVisitOrder.EntryIndex(visit,from,cancellationToken) : 0;
                        chainCursor=visit.Points[0]==visit.Points[^1] ? visit.Points[seam]
                            : seam==visit.Points.Count-1 ? visit.Points[0] : visit.Points[^1];
                    }
                }

                for (int chainIndex = 0; chainIndex < passChains.Count; chainIndex++)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    RingChain passChain = passChains[chainIndex];
                    progress?.Invoke(completedRings / totalRings, $"Routing pocket rings {completedRings:N0}/{totalRings:N0}");
                    completedRings += passChain.RingIndices.Count;

                    foreach (var built in BuildChainPaths(
                                 rings, passChain, topZ, bottomZ, regionEnd,
                                 requireEntry: pass == 0 && chainIndex == 0 && previousEnd is not null,
                                 options, tool, cancellationToken,
                                 ref linkLifts, ref linkCandidatesTested, ref alternativeLinks,
                                 ref radialDepthRamps, ref planarRingTransitions,
                                 ref straightTransitionFallbacks))
                    {
                        if (built.Points.Count < 2)
                            continue;

                        if (built.HasEntry)
                        {
                            switch (built.Strategy)
                            {
                                case RampStrategy.Helical: helicals++; break;
                                case RampStrategy.Linear: linearRamps++; break;
                                default: plunges++; break;
                            }
                        }

                        var points = built.Points;
                        var spans = built.Spans;
                        if (applyTabs is not null)
                            (points, spans) = applyTabs(passChain.RingIndices[0], points, spans);

                        if (points.Count < 2)
                            continue;

                        if (previousEnd is { } previous)
                        {
                            double dx = points[0].X - previous.X;
                            double dy = points[0].Y - previous.Y;
                            linkLength += Math.Sqrt(dx * dx + dy * dy);
                        }
                        previousEnd = points[^1];
                        regionEnd = points[^1];

                        cutLength += CutLength(points, spans);
                        emittedPointCount += points.Count;

                        toolpaths.Add(new TaggedToolpath(
                            points,
                            spans,
                            rings[passChain.RingIndices[0]].RegionIndex,
                            category,
                            pass,
                            visitIdentity: toolpaths.Count, detectCurves: options.DetectCurves));
                    }
                }
            }
        }

        int routeExcursions = 0;
        double routeAirSaved = 0;
        if (toolpaths.Count > 1)
        {
            (toolpaths, routeExcursions, routeAirSaved) = InterleaveNearbyRoutes(
                toolpaths,
                depth.PassCount, options.PreserveCommandVertices);
            linkLength = BoundaryLinkLength(toolpaths);
            emittedPointCount = toolpaths.Sum(path => path.Points.Count);
        }

        // Interleaving changes which stock has actually been cut before a
        // continuation. Certify floor links in the final execution order.
        if (category == Category)
        {
            MillingGeometryTrace.Capture("pocket-constructed", 0, toolpaths);
            var ordered = PocketContinuationRouter.Route(toolpaths, depth, options, tool.Radius, cancellationToken,
                tool: tool);
            toolpaths = ordered.Paths;
            MillingGeometryTrace.Capture("pocket-routed", 0, toolpaths);
            linkLifts += ordered.Lifts;
            plunges += ordered.Plunges;
            linearRamps += ordered.LinearRamps;
            helicals += ordered.Helicals;
            alternativeLinks += ordered.Rebased;
            linkCandidatesTested += ordered.Candidates;
            cutLength = toolpaths.Sum(path => CutLength(path.Points, path.Spans));
            linkLength = BoundaryLinkLength(toolpaths);
            emittedPointCount = toolpaths.Sum(path => path.Points.Count);
        }

        // Route construction and the temporal stock router above already own
        // these transitions. Rediscovering them in an emitted-chord A* graph
        // duplicates physical proofs and obscures source connectivity.
        cutLength = toolpaths.Sum(path => CutLength(path.Points,path.Spans));
        linkLength = BoundaryLinkLength(toolpaths);
        emittedPointCount = toolpaths.Sum(path => path.Points.Count);

        long sweptAreaStart = PerfLog.Start();
        var sweptCenterlines = rings.Where(ring => ring.IsClosed)
            .Select(ring => (IReadOnlyList<PointD>)ring.Points)
            .ToList();
        var linkKeys = new HashSet<(long StartX, long StartY, long EndX, long EndY)>();
        int linkCenterlineCount = 0;
        foreach (var toolpath in options.CalculateSweptArea ? toolpaths : Enumerable.Empty<TaggedToolpath>())
        {
            foreach (var span in toolpath.Spans.Where(span => span.Kind == ToolpathSpanKind.Link))
            {
                for (int pointIndex = span.StartIndex + 1;
                     pointIndex <= span.EndIndex && pointIndex < toolpath.Points.Count;
                     pointIndex++)
                {
                    Point3D start = toolpath.Points[pointIndex - 1];
                    Point3D end = toolpath.Points[pointIndex];
                    long startX = (long)Math.Round(start.X * PathUtils.Scale);
                    long startY = (long)Math.Round(start.Y * PathUtils.Scale);
                    long endX = (long)Math.Round(end.X * PathUtils.Scale);
                    long endY = (long)Math.Round(end.Y * PathUtils.Scale);
                    var key = (startX, startY, endX, endY);
                    var reverseKey = (endX, endY, startX, startY);
                    if (linkKeys.Contains(key) || linkKeys.Contains(reverseKey))
                        continue;

                    linkKeys.Add(key);
                    sweptCenterlines.Add(new List<PointD>
                    {
                        new(start.X, start.Y),
                        new(end.X, end.Y),
                    });
                    linkCenterlineCount++;
                }
            }
        }

        int bufferedPointCount = sweptCenterlines.Sum(centerline => centerline.Count);
        var sweptArea = options.CalculateSweptArea && toolpaths.Count > 0
            ? PolygonOps.Union(PolygonOps.BufferCenterlines(sweptCenterlines, tool.Radius, options.Tolerances.ArcTolerance)
                .Concat(PolygonOps.BufferOpenCenterlines(rings.Where(ring => !ring.IsClosed).Select(ring => ring.Points),
                    tool.Radius, options.Tolerances.ArcTolerance, cancellationToken)))
            : new List<List<PointD>>();
        PerfLog.Stop("PocketToolpaths.SweptArea", sweptAreaStart,
            $"rings={rings.Count} links={linkCenterlineCount} emittedPoints={emittedPointCount} " +
            $"bufferedPoints={bufferedPointCount} polygons={sweptArea.Count} calculated={options.CalculateSweptArea}");

        var diagnostics = new ToolpathDiagnostics
        {
            RingCount = rings.Count(ring => ring.IsClosed),
            StripCount = rings.Count(ring => !ring.IsClosed),
            IslandCount = traversal.Islands.Count,
            PathCount = toolpaths.Count,
            DepthPassCount = depth.PassCount,
            CutLength = cutLength,
            LinkLength = linkLength,
            LinkLifts = linkLifts,
            LinkCandidatesTested = linkCandidatesTested,
            AlternativeLinks = alternativeLinks,
            PlungeEntries = plunges,
            LinearRampEntries = linearRamps,
            HelicalEntries = helicals,
        };

        string levelTrace = string.Join(",", rings
            .GroupBy(ring => ring.Level)
            .OrderBy(group => group.Key)
            .Take(32)
            .Select(group => $"{group.Key}:{group.Count()}"));
        int missingRings = rings.Count - routedRingIndices.Count;
        PerfLog.Stop("PocketToolpaths.Emit", t0,
            $"category={category} paths={toolpaths.Count} links={linkLength:F3} lifts={linkLifts} " +
            $"routedRings={routedRingIndices.Count}/{rings.Count} missingRings={missingRings} " +
            $"levels=[{levelTrace}] " +
            $"routeExcursions={routeExcursions} routeAirSaved={routeAirSaved:F3} " +
            $"linkCandidates={linkCandidatesTested} alternatives={alternativeLinks} " +
            $"radialDepthRamps={radialDepthRamps} " +
            $"planarRingTransitions={planarRingTransitions} " +
            $"straightTransitionFallbacks={straightTransitionFallbacks}");

        return new ToolpathPlan(toolpaths, sweptArea, diagnostics);
    }

    private sealed class RouteBlock
    {
        internal RouteBlock(
            int routeId,
            IReadOnlyList<TaggedToolpath> paths,
            double cutLength)
        {
            RouteId = routeId;
            Paths = paths;
            CutLength = cutLength;
        }

        internal int RouteId { get; }
        internal IReadOnlyList<TaggedToolpath> Paths { get; }
        internal double CutLength { get; }
        internal List<RouteAttachment> Attachments { get; } = new();
        internal Dictionary<int, PreparedCarrierSegments> CarrierIndexes { get; } = new();
    }

    private sealed record RouteAttachment(
        RouteBlock Child,
        int HostPathIndex,
        RoutePause Pause);

    private readonly record struct RoutePause(
        int SegmentEndIndex,
        double SegmentPosition,
        double ExcursionLength);

    private static (List<TaggedToolpath> Paths, int Excursions, double AirSaved)
        InterleaveNearbyRoutes(
            IReadOnlyList<TaggedToolpath> source,
            int scheduledPasses, bool commandVerticesOnly = false)
    {
        List<RouteBlock> routes = BuildRouteBlocks(source);
        var attachedRoutes = new HashSet<int>();
        var carrierDiagnostics = new PreparedCarrierSegments.Diagnostics();
        bool useCarrierIndex = Environment.GetEnvironmentVariable("RW_TOOLPATHS_ROUTING_REFERENCE") != "1";
        foreach (RouteBlock guest in routes.OrderBy(route => route.CutLength))
        {
            if (!IsSpiralRoute(guest, scheduledPasses))
            {
                continue;
            }

            Point3D guestStart = guest.Paths[0].Points[0];
            Point3D guestEnd = guest.Paths[^1].Points[^1];
            (RouteBlock Host, int PathIndex, RoutePause Pause)? best = null;
            foreach (RouteBlock host in routes)
            {
                if (host.RouteId == guest.RouteId
                    || !IsCarrierRoute(host, guest))
                {
                    continue;
                }

                for (int pathIndex = 0; pathIndex < host.Paths.Count; pathIndex++)
                {
                    PreparedCarrierSegments? carrier = null;
                    if (useCarrierIndex && !host.CarrierIndexes.TryGetValue(pathIndex, out carrier))
                    {
                        carrier = new PreparedCarrierSegments(host.Paths[pathIndex], carrierDiagnostics);
                        host.CarrierIndexes.Add(pathIndex, carrier);
                    }
                    if (!TryFindRoutePause(
                            host.Paths[pathIndex],
                            carrier,
                            guestStart,
                            guestEnd,
                            best?.Pause.ExcursionLength ?? double.MaxValue,
                            carrierDiagnostics,
                            out RoutePause pause, commandVerticesOnly)
                        || (best is not null
                            && pause.ExcursionLength >= best.Value.Pause.ExcursionLength))
                    {
                        continue;
                    }

                    best = (host, pathIndex, pause);
                }
            }

            if (best is null)
                continue;

            best.Value.Host.Attachments.Add(new RouteAttachment(
                guest,
                best.Value.PathIndex,
                best.Value.Pause));
            attachedRoutes.Add(guest.RouteId);
        }

        var output = new List<TaggedToolpath>();
        foreach (RouteBlock root in routes.Where(route => !attachedRoutes.Contains(route.RouteId)))
            EmitRouteBlock(root, output);

        double airSaved = BoundaryLinkLength(source) - BoundaryLinkLength(output);
        Console.WriteLine($"[PocketContinuation.Carrier] routes={routes.Count} excursions={attachedRoutes.Count} "
            + $"mode={(useCarrierIndex ? "indexed" : "reference")} commandVerticesOnly={commandVerticesOnly} "
            + carrierDiagnostics.Describe());
        return (output, attachedRoutes.Count, airSaved);
    }

    private static List<RouteBlock> BuildRouteBlocks(IReadOnlyList<TaggedToolpath> source)
    {
        var routes = new List<RouteBlock>();
        var paths = new List<TaggedToolpath>();
        foreach (TaggedToolpath path in source)
        {
            if (paths.Count > 0
                && !PointsCoincide(paths[^1].Points[^1], path.Points[0]))
            {
                AddRoute();
            }
            paths.Add(path);
        }
        AddRoute();
        return routes;

        void AddRoute()
        {
            if (paths.Count == 0)
                return;
            var routePaths = paths.ToList();
            routes.Add(new RouteBlock(
                routes.Count,
                routePaths,
                routePaths.Sum(path => CutLength(path.Points, path.Spans))));
            paths.Clear();
        }
    }

    private static bool IsSpiralRoute(RouteBlock route, int scheduledPasses)
    {
        if (route.Paths.Count == 0
            || !IsRampedEntry(route.Paths[0])
            || route.Paths.Any(path => path.Spans.Any(span => span.Kind == ToolpathSpanKind.Link)))
        {
            return false;
        }

        var passes = route.Paths
            .Select(path => path.DepthPassIndex)
            .Where(pass => pass.HasValue)
            .Select(pass => pass!.Value)
            .Distinct()
            .Order()
            .ToArray();
        return passes.SequenceEqual(Enumerable.Range(0, scheduledPasses));
    }

    private static bool IsCarrierRoute(RouteBlock host, RouteBlock guest)
        => host.CutLength > guest.CutLength + 1e-6
            && (host.CutLength >= guest.CutLength * 2
                || host.Paths.Any(path => path.Spans.Any(
                    span => span.Kind == ToolpathSpanKind.Link)));

    private static void EmitRouteBlock(RouteBlock route, List<TaggedToolpath> output)
    {
        for (int pathIndex = 0; pathIndex < route.Paths.Count; pathIndex++)
        {
            TaggedToolpath path = route.Paths[pathIndex];
            List<RouteAttachment> attachments = route.Attachments
                .Where(attachment => attachment.HostPathIndex == pathIndex)
                .ToList();
            if (attachments.Count == 0)
            {
                output.Add(path);
                continue;
            }

            TaggedToolpath withPauses = InsertRoutePauses(
                path,
                attachments,
                out List<(RouteAttachment Attachment, int PointIndex)> pauses);
            int cursor = 0;
            foreach (var pause in pauses.OrderBy(entry => entry.PointIndex))
            {
                if (pause.PointIndex > cursor)
                    output.Add(SliceRoute(withPauses, cursor, pause.PointIndex));
                EmitRouteBlock(pause.Attachment.Child, output);
                cursor = pause.PointIndex;
            }
            if (cursor < withPauses.Points.Count - 1)
                output.Add(SliceRoute(withPauses, cursor, withPauses.Points.Count - 1));
        }
    }

    private static TaggedToolpath InsertRoutePauses(
        TaggedToolpath source,
        IReadOnlyList<RouteAttachment> attachments,
        out List<(RouteAttachment Attachment, int PointIndex)> pauses)
    {
        var points = new List<Point3D>(source.Points.Count + attachments.Count);
        var moveKinds = new List<ToolpathSpanKind>(source.Points.Count + attachments.Count);
        pauses = new List<(RouteAttachment Attachment, int PointIndex)>(attachments.Count);

        points.Add(source.Points[0]);
        var bySegment = attachments
            .GroupBy(attachment => attachment.Pause.SegmentEndIndex)
            .ToDictionary(
                group => group.Key,
                group => group.OrderBy(attachment => attachment.Pause.SegmentPosition).ToList());

        for (int segmentEnd = 1; segmentEnd < source.Points.Count; segmentEnd++)
        {
            Point3D start = source.Points[segmentEnd - 1];
            Point3D end = source.Points[segmentEnd];
            ToolpathSpanKind kind = MoveKindAt(source.Spans, segmentEnd);
            if (bySegment.TryGetValue(segmentEnd, out var segmentPauses))
            {
                foreach (RouteAttachment attachment in segmentPauses)
                {
                    double position = attachment.Pause.SegmentPosition;
                    if (position <= 1e-7)
                    {
                        pauses.Add((attachment, points.Count - 1));
                        continue;
                    }
                    if (position >= 1 - 1e-7)
                        continue;

                    points.Add(Interpolate(start, end, position));
                    moveKinds.Add(kind);
                    pauses.Add((attachment, points.Count - 1));
                }
            }

            points.Add(end);
            moveKinds.Add(kind);
            if (bySegment.TryGetValue(segmentEnd, out segmentPauses))
            {
                foreach (RouteAttachment attachment in segmentPauses)
                {
                    if (attachment.Pause.SegmentPosition >= 1 - 1e-7)
                        pauses.Add((attachment, points.Count - 1));
                }
            }
        }

        return new TaggedToolpath(
            points,
            BuildSpansFromMoves(moveKinds),
            source.RegionIndex,
            source.Category,
            source.DepthPassIndex,
            source.VisitIdentity, detectCurves: source.DetectCurves);
    }

    private static ToolpathSpanKind MoveKindAt(
        IReadOnlyList<ToolpathSpan> spans,
        int moveEndIndex)
    {
        foreach (ToolpathSpan span in spans)
        {
            if (moveEndIndex > span.StartIndex && moveEndIndex <= span.EndIndex)
                return span.Kind;
        }
        return ToolpathSpanKind.Cut;
    }

    private static List<ToolpathSpan> BuildSpansFromMoves(
        IReadOnlyList<ToolpathSpanKind> moveKinds)
    {
        var spans = new List<ToolpathSpan>();
        if (moveKinds.Count == 0)
            return spans;

        int start = 0;
        ToolpathSpanKind kind = moveKinds[0];
        for (int move = 1; move < moveKinds.Count; move++)
        {
            if (moveKinds[move] == kind)
                continue;

            spans.Add(new ToolpathSpan(start, move, kind));
            start = move;
            kind = moveKinds[move];
        }
        spans.Add(new ToolpathSpan(start, moveKinds.Count, kind));
        return spans;
    }

    private static bool IsRampedEntry(TaggedToolpath path)
        => path.Points.Count >= 2
            && path.Spans.Any(span => span.Kind == ToolpathSpanKind.Ramp)
            && path.Spans.Any(span => span.Kind == ToolpathSpanKind.Cut);

    private static bool TryFindRoutePause(
        TaggedToolpath host,
        PreparedCarrierSegments? carrier,
        Point3D guestStart,
        Point3D guestEnd,
        double upperBound,
        PreparedCarrierSegments.Diagnostics diagnostics,
        out RoutePause pause, bool commandVerticesOnly = false)
    {
        if (carrier is null || !carrier.SupportsBounds(guestStart, guestEnd))
        {
            if (carrier is not null) diagnostics.FallbackQueries++;
            // Preserve the exact former comparison for malformed coordinates:
            // a NaN excursion can replace an earlier finite candidate and make
            // the path fail. Spatial pruning must not silently ignore it.
            pause = default;
            double legacyBest = double.MaxValue;
            foreach (var span in host.Spans)
            {
                if (span.Kind != ToolpathSpanKind.Cut) continue;
                for (int segmentEnd = Math.Max(1, span.StartIndex + 1);
                    segmentEnd <= Math.Min(host.Points.Count - 1, span.EndIndex); segmentEnd++)
                {
                    Point3D start = host.Points[segmentEnd - 1], end = host.Points[segmentEnd];
                    double position;
                    if (commandVerticesOnly)
                    {
                        if (!TryVertexExcursion(start, end, guestStart, guestEnd, segmentEnd,
                                host.Points.Count, out position)) continue;
                    }
                    else position = MinimizeExcursion(start, end, guestStart, guestEnd);
                    if (position <= 1e-7 && segmentEnd == 1
                        || position >= 1 - 1e-7 && segmentEnd == host.Points.Count - 1) continue;
                    Point3D candidate = Interpolate(start, end, position);
                    double length = PlanarDistance(candidate, guestStart) + PlanarDistance(guestEnd, candidate);
                    if (length >= legacyBest) continue;
                    legacyBest = length;
                    pause = new RoutePause(segmentEnd, position, length);
                }
            }
            return legacyBest < double.MaxValue;
        }
        RoutePause selected = default;
        double best = upperBound;
        int bestOrdinal = int.MaxValue;
        carrier.VisitCandidates(guestStart, guestEnd, () => best, Evaluate, diagnostics);
        pause = selected;
        return bestOrdinal != int.MaxValue;

        void Evaluate(PreparedCarrierSegments.Segment segment)
        {
            Point3D start = segment.Start, end = segment.End;
            double position;
            if (commandVerticesOnly)
            {
                if (!TryVertexExcursion(start, end, guestStart, guestEnd, segment.EndIndex,
                        host.Points.Count, out position)) return;
            }
            else position = MinimizeExcursion(start, end, guestStart, guestEnd);
            if (position <= 1e-7 && segment.EndIndex == 1
                || position >= 1 - 1e-7 && segment.EndIndex == host.Points.Count - 1)
                return;
            Point3D candidate = Interpolate(start, end, position);
            double length = PlanarDistance(candidate, guestStart) + PlanarDistance(guestEnd, candidate);
            if (length < best || length == best && segment.Ordinal < bestOrdinal)
            {
                best = length;
                bestOrdinal = segment.Ordinal;
                selected = new RoutePause(segment.EndIndex, position, length);
            }
        }
    }

    private static bool TryVertexExcursion(Point3D start, Point3D end, Point3D guestStart,
        Point3D guestEnd, int segmentEndIndex, int pointCount, out double position)
    {
        bool allowStart = segmentEndIndex != 1, allowEnd = segmentEndIndex != pointCount - 1;
        position = allowStart && (!allowEnd || PlanarDistance(start, guestStart) + PlanarDistance(guestEnd, start)
            <= PlanarDistance(end, guestStart) + PlanarDistance(guestEnd, end)) ? 0 : 1;
        return allowStart || allowEnd;
    }

    private static double MinimizeExcursion(
        Point3D segmentStart,
        Point3D segmentEnd,
        Point3D guestStart,
        Point3D guestEnd)
    {
        double low = 0;
        double high = 1;
        for (int iteration = 0; iteration < 32; iteration++)
        {
            double first = (low * 2 + high) / 3;
            double second = (low + high * 2) / 3;
            double firstCost = ExcursionAt(first);
            double secondCost = ExcursionAt(second);
            if (firstCost <= secondCost)
                high = second;
            else
                low = first;
        }
        return (low + high) * 0.5;

        double ExcursionAt(double position)
        {
            Point3D point = Interpolate(segmentStart, segmentEnd, position);
            return PlanarDistance(point, guestStart) + PlanarDistance(guestEnd, point);
        }
    }

    private static TaggedToolpath SliceRoute(TaggedToolpath source, int start, int end)
    {
        var points = source.Points.Skip(start).Take(end - start + 1).ToList();
        var spans = new List<ToolpathSpan>();
        foreach (ToolpathSpan span in source.Spans)
        {
            int spanStart = Math.Max(start, span.StartIndex);
            int spanEnd = Math.Min(end, span.EndIndex);
            if (spanEnd <= spanStart)
                continue;
            spans.Add(new ToolpathSpan(
                spanStart - start,
                spanEnd - start,
                span.Kind));
        }
        return new TaggedToolpath(
            points,
            spans,
            source.RegionIndex,
            source.Category,
            source.DepthPassIndex,
            source.VisitIdentity, detectCurves: source.DetectCurves);
    }

    private static double BoundaryLinkLength(IReadOnlyList<TaggedToolpath> paths)
    {
        double total = 0;
        for (int index = 1; index < paths.Count; index++)
            total += PlanarDistance(paths[index - 1].Points[^1], paths[index].Points[0]);
        return total;
    }

    private static bool PointsCoincide(Point3D first, Point3D second)
        => Math.Abs(first.X - second.X) <= 1e-6
            && Math.Abs(first.Y - second.Y) <= 1e-6
            && Math.Abs(first.Z - second.Z) <= 1e-6;

    private static Point3D Interpolate(Point3D start, Point3D end, double position)
        => new(
            start.X + (end.X - start.X) * position,
            start.Y + (end.Y - start.Y) * position,
            start.Z + (end.Z - start.Z) * position);

    private static double PlanarDistance(Point3D first, Point3D second)
    {
        double dx = second.X - first.X;
        double dy = second.Y - first.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>Sequences islands so the cutter crosses as little empty space as possible.</summary>
    private static int[] OrderIslands(
        IReadOnlyList<OffsetRing> rings,
        RingTraversal traversal,
        CancellationToken cancellationToken)
    {
        if (traversal.Islands.Count <= 1)
            return traversal.Islands.Count == 0 ? Array.Empty<int>() : new[] { 0 };

        long t0 = PerfLog.Start();
        var islandsByComponent = new Dictionary<int, List<int>>();
        var componentIndices = new List<int>();
        for (int islandIndex = 0; islandIndex < traversal.Islands.Count; islandIndex++)
        {
            int componentIndex = rings[traversal.Order[traversal.Islands[islandIndex].StartIndex]].RegionIndex;
            if (!islandsByComponent.TryGetValue(componentIndex, out var componentIslands))
            {
                componentIslands = new List<int>();
                islandsByComponent.Add(componentIndex, componentIslands);
                componentIndices.Add(componentIndex);
            }

            componentIslands.Add(islandIndex);
        }

        // The outer-to-outer hop between decals may be shorter than the hop to a cutout in the
        // current decal, but leaving a decal half-cut costs a long rapid to return. Optimise the
        // decal groups first, then optimise their local contours without allowing re-entry.
        var componentBoundaries = new List<IReadOnlyList<PointD>>(componentIndices.Count);
        foreach (int componentIndex in componentIndices)
        {
            var boundary = new List<PointD>();
            foreach (int islandIndex in islandsByComponent[componentIndex])
                boundary.AddRange(rings[traversal.Order[traversal.Islands[islandIndex].StartIndex]].Points);
            componentBoundaries.Add(boundary);
        }

        var optimizedComponents = TravelOptimizer.Order(
            componentBoundaries, start: null, label: "pocket-components", cancellationToken);
        var order = new List<int>(traversal.Islands.Count);
        var componentOrder = new List<int>(optimizedComponents.Length);
        foreach (int componentPosition in optimizedComponents)
        {
            cancellationToken.ThrowIfCancellationRequested();

            int componentIndex = componentIndices[componentPosition];
            componentOrder.Add(componentIndex);
            var componentIslands = islandsByComponent[componentIndex];
            if (componentIslands.Count == 1)
            {
                order.Add(componentIslands[0]);
                continue;
            }

            var islandBoundaries = new List<IReadOnlyList<PointD>>(componentIslands.Count);
            foreach (int islandIndex in componentIslands)
                islandBoundaries.Add(rings[traversal.Order[traversal.Islands[islandIndex].StartIndex]].Points);

            foreach (int islandPosition in TravelOptimizer.Order(
                         islandBoundaries, start: null, $"pocket-component-{componentIndex}", cancellationToken))
            {
                order.Add(componentIslands[islandPosition]);
            }
        }

        string componentTrace = string.Join(",", componentOrder.Take(16));
        if (componentOrder.Count > 16)
            componentTrace += ",...";

        PerfLog.Stop("PocketToolpaths.OrderIslands", t0,
            $"islands={traversal.Islands.Count} components={componentOrder.Count} reentries=0 " +
            $"componentOrder=[{componentTrace}]");

        return order.ToArray();
    }

    private readonly record struct ChainPath(
        List<Point3D> Points,
        List<ToolpathSpan> Spans,
        RampStrategy Strategy,
        bool HasEntry);

    private readonly record struct LinkCandidate(
        PointD Point,
        int SegmentIndex,
        double SegmentPosition,
        double DistanceSquared);

    /// <summary>
    /// Walks a chain: ramp into the first ring, then step through the rest at depth, rotating
    /// each so it starts where the previous one finished.
    ///
    /// <para>
    /// A step is only taken when the clearance test can prove the straight move removes nothing
    /// the operation has to leave standing. Where it cannot, the chain is cut in two and the
    /// second half ramps in afresh, which costs a lift but never drags the cutter through an
    /// island. Ordering and nesting are heuristics, so this is what keeps a bad guess from
    /// becoming a scrapped part.
    /// </para>
    /// </summary>
    private static List<ChainPath> BuildChainPaths(
        IReadOnlyList<OffsetRing> rings,
        RingChain chain,
        double topZ,
        double bottomZ,
        Point3D? entryTarget,
        bool requireEntry,
        PocketOptions options,
        ToolGeometry tool,
        CancellationToken cancellationToken,
        ref int linkLifts,
        ref int linkCandidatesTested,
        ref int alternativeLinks,
        ref int radialDepthRamps,
        ref int planarRingTransitions,
        ref int straightTransitionFallbacks)
    {
        var clearance = options.LinkClearance ?? LinkClearance.Unrestricted;
        var paths = new List<ChainPath>(1);

        bool firstClosed = rings[chain.RingIndices[0]].IsClosed;
        var firstRing = RingOffsetEngine.Orient(rings[chain.RingIndices[0]], options.Direction);
        bool hasEntry = true;
        bool linkAtDepth = false;
        bool continuesFromPreviousPass = false;
        bool rampFromPreviousPass = false;
        double ringEntryZ = topZ;
        if (entryTarget is { } target)
        {
            if (requireEntry)
            {
                linkLifts++;
                firstRing = RebaseFillPath(firstRing, firstClosed, target);
            }
            else
            {
                bool continuesPreviousPass = Math.Abs(target.Z - topZ) <= 1e-6;
                double entryZ = continuesPreviousPass ? topZ : bottomZ;
                if (TryFindSafeFillEntry(
                        firstRing,
                        firstClosed,
                        target,
                        entryZ,
                        clearance,
                        tool.Radius,
                        out var entryRing,
                        out int candidatesTested, options.PreserveCommandVertices))
                {
                    firstRing = entryRing;
                    linkAtDepth = !continuesPreviousPass;
                    hasEntry = !linkAtDepth;
                    linkCandidatesTested += candidatesTested;
                    if (candidatesTested > 1)
                        alternativeLinks++;

                    if (continuesPreviousPass)
                    {
                        continuesFromPreviousPass = true;
                        double dx = firstRing[0].x - target.X;
                        double dy = firstRing[0].y - target.Y;
                        double connectorLength = Math.Sqrt(dx * dx + dy * dy);
                        ringEntryZ = RampPlanner.PlanTransitionEntry(target,
                            new(firstRing[0].x,firstRing[0].y,bottomZ),options.Ramp).Z;
                        rampFromPreviousPass = topZ - ringEntryZ > 1e-9;
                        if (rampFromPreviousPass && connectorLength > 1e-9)
                            radialDepthRamps++;
                    }
                }
                else
                {
                    linkCandidatesTested += candidatesTested;
                    linkLifts++;
                    firstRing = RebaseFillPath(firstRing, firstClosed, target);
                }
            }
        }
        var entry = PlanFillEntry(firstRing, firstClosed,
            linkAtDepth ? bottomZ : ringEntryZ, bottomZ, options.Ramp, options.PreserveCommandVertices, cancellationToken);

        var points = entry.Points;
        var spans = entry.Spans;
        var strategy = rampFromPreviousPass ? RampStrategy.Linear : entry.Strategy;
        if (entryTarget is { } transitionStart
            && (linkAtDepth || continuesFromPreviousPass))
        {
            PrependTransition(
                points,
                spans,
                transitionStart,
                rampFromPreviousPass ? ToolpathSpanKind.Ramp : ToolpathSpanKind.Link,
                clearance,
                tool.Radius,
                ref planarRingTransitions,
                ref straightTransitionFallbacks);
        }

        for (int i = 1; i < chain.RingIndices.Count; i++)
        {
            bool closed = rings[chain.RingIndices[i]].IsClosed;
            var ring = RingOffsetEngine.Orient(rings[chain.RingIndices[i]], options.Direction);

            if (points.Count > 0)
            {
                var last = points[^1];
                if (TryFindSafeFillEntry(
                        ring,
                        closed,
                        last,
                        bottomZ,
                        clearance,
                        tool.Radius,
                        out var rebased,
                        out int candidatesTested, options.PreserveCommandVertices))
                {
                    linkCandidatesTested += candidatesTested;
                    if (candidatesTested > 1)
                        alternativeLinks++;
                    AppendRingAtDepth(
                        points,
                        spans,
                        rebased,
                        bottomZ,
                        clearance,
                        tool.Radius,
                        ref planarRingTransitions,
                        ref straightTransitionFallbacks, isClosed: closed);
                    continue;
                }

                linkCandidatesTested += candidatesTested;
                rebased = RebaseFillPath(ring, closed, last);

                linkLifts++;
                paths.Add(new ChainPath(points, spans, strategy, hasEntry));

                var reentry = PlanFillEntry(rebased, closed, topZ, bottomZ, options.Ramp,
                    options.PreserveCommandVertices, cancellationToken);
                points = reentry.Points;
                spans = reentry.Spans;
                strategy = reentry.Strategy;
                hasEntry = true;
                continue;
            }

            AppendRingAtDepth(
                points,
                spans,
                ring,
                bottomZ,
                clearance,
                tool.Radius,
                ref planarRingTransitions,
                ref straightTransitionFallbacks, isClosed: closed);
        }

        paths.Add(new ChainPath(points, spans, strategy, hasEntry));
        return paths;
    }

    private static RampedPath PlanFillEntry(IReadOnlyList<PointD> path, bool closed,
        double topZ, double bottomZ, RampSettings settings, bool commandVerticesOnly = false,
        CancellationToken cancellationToken = default)
    {
        if (closed) return commandVerticesOnly
            ? RampPlanner.PlanClosedRingAtCommandVertices(path, topZ, bottomZ, settings,
                cancellationToken,
                diagnostic: message => Console.WriteLine($"[PocketToolpaths.CommandVertexEntry] {message}"))
            : RampPlanner.PlanClosedRing(path, topZ, bottomZ, settings);
        var trace = path.Select(p => new Point3D(p.x, p.y, bottomZ)).ToList();
        if (topZ <= bottomZ + 1e-9)
            return new RampedPath(trace, new List<ToolpathSpan> { new(0, trace.Count - 1, ToolpathSpanKind.Cut) }, RampStrategy.Plunge, 0);
        if (settings.Strategy == RampStrategy.Plunge || settings.AngleRadians <= 0)
        {
            trace.Insert(0, new Point3D(trace[0].X, trace[0].Y, topZ));
            return new RampedPath(trace, new List<ToolpathSpan>
            {
                new(0, 1, ToolpathSpanKind.Ramp), new(1, trace.Count - 1, ToolpathSpanKind.Cut)
            }, RampStrategy.Plunge, 0);
        }
        return RampPlanner.PlanPathEntry(trace, topZ, settings,
            cancellationToken,
            diagnostic: message => Console.WriteLine($"[PocketToolpaths.TerminalEntry] {message}"),
            landAtSourceVertex: commandVerticesOnly);
    }

    private static List<PointD> RebaseFillPath(List<PointD> path, bool closed, Point3D from)
    {
        if (closed) return PathUtils.RebaseNear(path, new PointD(from.X, from.Y));
        var result = new List<PointD>(path);
        if (PlanarDistance(from, new Point3D(path[^1].x, path[^1].y, from.Z))
            < PlanarDistance(from, new Point3D(path[0].x, path[0].y, from.Z))) result.Reverse();
        return result;
    }

    private static bool TryFindSafeFillEntry(IReadOnlyList<PointD> path, bool closed, Point3D from,
        double bottomZ, ILinkClearance clearance, double radius, out List<PointD> entry, out int tested,
        bool commandVerticesOnly = false)
    {
        if (closed) return TryFindSafeRebase(path, from, bottomZ, clearance, radius, out entry, out tested, commandVerticesOnly,
            localEntriesOnly:true);
        entry = RebaseFillPath(path.ToList(), false, from);
        tested = 1;
        if (clearance.IsTravelSafe(from, new Point3D(entry[0].x, entry[0].y, bottomZ), radius)) return true;
        tested++;
        entry.Reverse();
        return clearance.IsTravelSafe(from, new Point3D(entry[0].x, entry[0].y, bottomZ), radius);
    }

    private static void PrependTransition(
        List<Point3D> points,
        List<ToolpathSpan> spans,
        Point3D from,
        ToolpathSpanKind kind,
        ILinkClearance clearance,
        double toolRadius,
        ref int planarRingTransitions,
        ref int straightTransitionFallbacks)
    {
        if (points.Count == 0)
            return;

        Point3D first = points[0];
        if (Math.Abs(first.X - from.X) <= 1e-9
            && Math.Abs(first.Y - from.Y) <= 1e-9
            && Math.Abs(first.Z - from.Z) <= 1e-9)
        {
            points[0] = from;
            return;
        }

        Point3D? destinationNext = points.Count > 1 ? points[1] : null;
        bool smoothed = TryBuildPlanarTransition(
            from,
            sourcePrevious: null,
            first,
            destinationNext,
            clearance,
            toolRadius,
            out var transition);
        if (smoothed)
            planarRingTransitions++;
        else
            straightTransitionFallbacks++;

        int insertedCount = smoothed ? transition.Count - 1 : 1;
        if (smoothed)
            points.InsertRange(0, transition.Take(transition.Count - 1));
        else
            points.Insert(0, from);
        for (int spanIndex = 0; spanIndex < spans.Count; spanIndex++)
        {
            ToolpathSpan span = spans[spanIndex];
            spans[spanIndex] = new ToolpathSpan(
                span.StartIndex + insertedCount,
                span.EndIndex + insertedCount,
                span.Kind);
        }
        spans.Insert(0, new ToolpathSpan(0, insertedCount, kind));
    }

    private static bool TryBuildPlanarTransition(
        Point3D from,
        Point3D? sourcePrevious,
        Point3D to,
        Point3D? destinationNext,
        ILinkClearance clearance,
        double toolRadius,
        out List<Point3D> transition)
    {
        transition = new List<Point3D> { from, to };
        if (PlanarDistance(from, to) <= 1e-6)
            return false;

        return clearance.IsTravelSafe(from, to, toolRadius);
    }

    internal static bool TryFindSafeRebase(
        IReadOnlyList<PointD> ring,
        Point3D from,
        double bottomZ,
        ILinkClearance clearance,
        double toolRadius,
        out List<PointD> rebased,
        out int candidatesTested, bool commandVerticesOnly = false, bool localEntriesOnly = false)
    {
        var loop = RampPlanner.Normalize(ring);
        rebased = loop;
        candidatesTested = 0;
        if (loop.Count < 2)
            return false;

        var candidates = new List<LinkCandidate>(loop.Count * 2);
        var seen = new HashSet<(long X, long Y)>();
        for (int segmentIndex = 0; segmentIndex < loop.Count; segmentIndex++)
        {
            PointD start = loop[segmentIndex];
            PointD end = loop[(segmentIndex + 1) % loop.Count];
            AddCandidate(start, segmentIndex, 0);
            if (commandVerticesOnly) continue;

            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double lengthSquared = dx * dx + dy * dy;
            if (lengthSquared <= 1e-18)
                continue;

            double position = Math.Clamp(
                ((from.X - start.x) * dx + (from.Y - start.y) * dy) / lengthSquared,
                0,
                1);
            AddCandidate(
                new PointD(start.x + dx * position, start.y + dy * position),
                segmentIndex,
                position);
        }

        candidates.Sort((first, second) =>
        {
            int comparison = first.DistanceSquared.CompareTo(second.DistanceSquared);
            if (comparison != 0)
                return comparison;
            comparison = first.SegmentIndex.CompareTo(second.SegmentIndex);
            return comparison != 0
                ? comparison
                : first.SegmentPosition.CompareTo(second.SegmentPosition);
        });

        // Do not orbit a whole contour looking for a distant legal bridge when
        // its nearby entry is blocked. The normal re-entry retains every cut.
        // This also bounds expensive physical proofs to a local neighbourhood.
        double localLimit = !localEntriesOnly ? double.PositiveInfinity : candidates.Count == 0 ? 0
            : Math.Sqrt(candidates[0].DistanceSquared) + 8 * toolRadius;
        foreach (LinkCandidate candidate in candidates)
        {
            if (candidate.DistanceSquared > localLimit * localLimit + 1e-12) break;
            candidatesTested++;
            if (!clearance.IsTravelSafe(
                    from,
                    new Point3D(candidate.Point.x, candidate.Point.y, bottomZ),
                    toolRadius))
            {
                continue;
            }

            rebased = RebaseAtCandidate(loop, candidate);
            return true;
        }

        return false;

        void AddCandidate(PointD point, int segmentIndex, double segmentPosition)
        {
            var key = (
                (long)Math.Round(point.x * PathUtils.Scale),
                (long)Math.Round(point.y * PathUtils.Scale));
            if (!seen.Add(key))
                return;

            double pointDx = point.x - from.X;
            double pointDy = point.y - from.Y;
            candidates.Add(new LinkCandidate(
                point,
                segmentIndex,
                segmentPosition,
                pointDx * pointDx + pointDy * pointDy));
        }
    }

    private static List<PointD> RebaseAtCandidate(
        IReadOnlyList<PointD> ring,
        LinkCandidate candidate)
    {
        int startIndex;
        bool insertPoint;
        if (candidate.SegmentPosition <= 1e-12)
        {
            startIndex = candidate.SegmentIndex;
            insertPoint = false;
        }
        else if (candidate.SegmentPosition >= 1 - 1e-12)
        {
            startIndex = (candidate.SegmentIndex + 1) % ring.Count;
            insertPoint = false;
        }
        else
        {
            startIndex = (candidate.SegmentIndex + 1) % ring.Count;
            insertPoint = true;
        }

        var rebased = new List<PointD>(ring.Count + (insertPoint ? 1 : 0));
        if (insertPoint)
            rebased.Add(candidate.Point);
        for (int offset = 0; offset < ring.Count; offset++)
            rebased.Add(ring[(startIndex + offset) % ring.Count]);
        return rebased;
    }

    private static void AppendRingAtDepth(
        List<Point3D> points,
        List<ToolpathSpan> spans,
        IReadOnlyList<PointD> ring,
        double bottomZ,
        ILinkClearance clearance,
        double toolRadius,
        ref int planarRingTransitions,
        ref int straightTransitionFallbacks,
        bool isClosed = true)
    {
        if (ring.Count == 0)
            return;

        int linkStart = points.Count - 1;
        var ringStart = new Point3D(ring[0].x, ring[0].y, bottomZ);
        int cutStart = 0;
        if (linkStart >= 0)
        {
            Point3D? sourcePrevious = points.Count > 1 ? points[^2] : null;
            Point3D? destinationNext = ring.Count > 1
                ? new Point3D(ring[1].x, ring[1].y, bottomZ)
                : null;
            bool smoothed = TryBuildPlanarTransition(
                points[^1],
                sourcePrevious,
                ringStart,
                destinationNext,
                clearance,
                toolRadius,
                out var transition);
            if (smoothed)
            {
                planarRingTransitions++;
                points.AddRange(transition.Skip(1));
            }
            else
            {
                straightTransitionFallbacks++;
                points.Add(ringStart);
            }

            cutStart = points.Count - 1;
            spans.Add(new ToolpathSpan(linkStart, cutStart, ToolpathSpanKind.Link));
        }
        else
        {
            points.Add(ringStart);
        }

        for (int pointIndex = 1; pointIndex < ring.Count; pointIndex++)
        {
            PointD point = ring[pointIndex];
            points.Add(new Point3D(point.x, point.y, bottomZ));
        }
        if (isClosed) points.Add(new Point3D(ring[0].x, ring[0].y, bottomZ));

        spans.Add(new ToolpathSpan(cutStart, points.Count - 1, ToolpathSpanKind.Cut));
    }

    /// <summary>Trims rings to a boundary, preserving each ring's level and region.</summary>
    private static List<OffsetRing> ClipRings(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<PointD> boundary, CancellationToken cancellationToken = default)
    {
        var clipped = new List<OffsetRing>(rings.Count);
        foreach (var ring in rings)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (!ring.IsClosed)
            {
                var open = CheckedOpenClip.Require(new[] { ring.Points }, new[] { boundary }, ClipType.Intersection,
                    cancellationToken, "pocket-final-open-commands");
                foreach (var fragment in open)
                    if (fragment.Count >= 2) clipped.Add(RingOffsetEngine.CreateRing(fragment,
                        ring.Level, ring.RegionIndex, isHole: false, isClosed: false));
                continue;
            }
            foreach (var fragment in PolygonOps.Clip(new[] { ring.Points }, boundary))
            {
                if (fragment.Count >= 3)
                    clipped.Add(RingOffsetEngine.CreateRing(
                        fragment, ring.Level, ring.RegionIndex, ring.IsHole));
            }
        }
        return clipped;
    }

    internal sealed class FillSourceClearance : ILinkClearance
    {
        private readonly List<(PolygonOps.BoundarySegmentIndex Index, double MinX, double MinY, double MaxX, double MaxY)> _sources;
        private readonly double _radius;
        private readonly ILinkClearance? _caller;
        private readonly ILinkClearance? _clip;
        private long _queries, _sourceRefusals, _callerRefusals, _clipRefusals;
        private readonly double _requestedRadius;
        private readonly object _sourceMemoLock=new();
        private readonly Dictionary<(double,double,double,double),bool> _sourceMemo=new();
        private readonly Queue<(double,double,double,double)> _sourceMemoOrder=new();
        internal long SourceMemoHits;
        private long _sourceMemoMisses,_sourceMemoEvictions;

        internal FillSourceClearance(IReadOnlyList<MillingRegion> regions, double radius,
            ILinkClearance? caller, IReadOnlyList<PointD>? clip)
        {
            long started = PerfLog.Start();
            _sources = regions.Where(region => region.Outer.Count >= 3).Select(region =>
                (new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(
                    PolygonOps.Difference(new[] { region.Outer }, region.Holes))),
                    region.Outer.Min(p => p.x) - .001, region.Outer.Min(p => p.y) - .001,
                    region.Outer.Max(p => p.x) + .001, region.Outer.Max(p => p.y) + .001)).ToList();
            // ContainsCapsule adds four native ticks. Subtract eight here so
            // the net commanded chord contract is radius minus four ticks,
            // exactly the existing original first-front wall proof. The .001
            // source approximation allowance never authorizes a link to shave
            // stock-to-leave. F3 displacement has its separate posted-wall band.
            _radius = Math.Max(0, radius * PathUtils.Scale - 8);
            _requestedRadius = radius;
            _caller = caller;
            _clip = clip is { Count: >= 3 } ? LinkClearance.WithinRegion(new[] { clip }) : null;
            PerfLog.Stop("PocketToolpaths.SourceTravelClearance", started,
                $"sourceRegions={_sources.Count} requestedRadius={radius:R} "
                + $"nativeChordRadius={(_radius + 4) / PathUtils.Scale:R} "
                + "sourceApproximationCredited=False owner=original-source-with-protected-holes");
        }

        public bool IsTravelSafe(Point3D from, Point3D to, double toolRadius)
        {
            Interlocked.Increment(ref _queries);
            if (!(_caller?.IsTravelSafe(from, to, toolRadius) ?? true))
            { Interlocked.Increment(ref _callerRefusals); return false; }
            if (!(_clip?.IsTravelSafe(from, to, 0) ?? true))
            { Interlocked.Increment(ref _clipRefusals); return false; }
            // Only the immutable source capsule is independent of depth.
            // The caller's potentially changing stock/XYZ restriction and clip
            // have ALREADY run, even for a repeated source-safe XY chord.
            var key=(from.X,from.Y,to.X,to.Y);
            lock(_sourceMemoLock)
                if(_sourceMemo.TryGetValue(key,out bool cached))
                {Interlocked.Increment(ref SourceMemoHits);return cached;}
            Interlocked.Increment(ref _sourceMemoMisses);
            var a = new Point64((long)Math.Round(from.X * PathUtils.Scale), (long)Math.Round(from.Y * PathUtils.Scale));
            var b = new Point64((long)Math.Round(to.X * PathUtils.Scale), (long)Math.Round(to.Y * PathUtils.Scale));
            bool safe=false;
            foreach(var source in _sources)
            if(from.X >= source.MinX && from.X <= source.MaxX
                && from.Y >= source.MinY && from.Y <= source.MaxY
                && to.X >= source.MinX && to.X <= source.MaxX && to.Y >= source.MinY && to.Y <= source.MaxY
                && source.Index.ContainsCapsule(a, b, _radius)){safe=true;break;}
            lock(_sourceMemoLock)
            if(!_sourceMemo.ContainsKey(key))
            {
                if(_sourceMemo.Count==65536){_sourceMemo.Remove(_sourceMemoOrder.Dequeue());_sourceMemoEvictions++;}
                _sourceMemo.Add(key,safe);_sourceMemoOrder.Enqueue(key);
            }
            if (!safe && Interlocked.Increment(ref _sourceRefusals) <= 4)
                Console.WriteLine($"[Pocket.SourceTravel] event=refuse from=({from.X:R},{from.Y:R},{from.Z:R}) "
                    + $"to=({to.X:R},{to.Y:R},{to.Z:R}) requestedRadius={_requestedRadius:R} "
                    + $"nativeChordRadius={(_radius + 4) / PathUtils.Scale:R} "
                    + $"sourceApproximationCredited=False proof=complete-original-source-capsule");
            return safe;
        }

        internal void Report() => Console.WriteLine($"[Pocket.SourceTravel] event=complete queries={_queries} "
            + $"sourceRefusals={_sourceRefusals} callerRefusals={_callerRefusals} clipRefusals={_clipRefusals} "
            + $"requestedRadius={_requestedRadius:R} nativeChordRadius={(_radius + 4) / PathUtils.Scale:R} "
            + $"sourceMemoHits={SourceMemoHits} sourceMemoMisses={_sourceMemoMisses} sourceMemoEvictions={_sourceMemoEvictions} sourceMemoLimit=65536 "
            + $"scope=all-offset-links sourceApproximationCredited=False");
    }

    private static double CutLength(IReadOnlyList<Point3D> points, IReadOnlyList<ToolpathSpan> spans)
    {
        double total = 0;
        foreach (var span in spans)
        {
            if (span.Kind is not (ToolpathSpanKind.Cut or ToolpathSpanKind.Link))
                continue;

            for (int i = span.StartIndex + 1; i <= span.EndIndex && i < points.Count; i++)
            {
                double dx = points[i].X - points[i - 1].X;
                double dy = points[i].Y - points[i - 1].Y;
                total += Math.Sqrt(dx * dx + dy * dy);
            }
        }
        return total;
    }
}
