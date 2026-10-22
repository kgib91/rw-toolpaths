using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>One closed offset contour or open terminal trace, with its place in the offset tree.</summary>
public sealed class OffsetRing
{
    /// <summary>Takes ownership of <paramref name="points"/> and normalizes it to CCW winding.</summary>
    internal OffsetRing(List<PointD> points, int level, int regionIndex, bool? isHole = null, bool isClosed = true, double? offsetDistance = null)
    {
        IsClosed = isClosed;
        OffsetDistance = offsetDistance;
        double area = isClosed ? PolygonOps.SignedArea(points) : 0;
        IsHole = isHole ?? area < 0;
        if (area < 0)
        {
            points.Reverse();
            area = -area;
        }

        Points = points;
        Level = level;
        RegionIndex = regionIndex;
        Area = area;
        Perimeter = isClosed ? PolygonOps.Perimeter(points) : Enumerable.Range(1, Math.Max(0, points.Count - 1))
            .Sum(i => Math.Sqrt(Math.Pow(points[i].x - points[i - 1].x, 2) + Math.Pow(points[i].y - points[i - 1].y, 2)));
        Centroid = isClosed ? PolygonOps.Centroid(points)
            : points.Count == 0 ? new PointD() : new PointD(points.Average(p => p.x), points.Average(p => p.y));
    }

    public List<PointD> Points { get; }

    /// <summary>False for terminal slivers; no consumer may add their endpoint-to-start chord.</summary>
    public bool IsClosed { get; }

    /// <summary>Inward offset steps from the source boundary; 0 is the outermost ring.</summary>
    public int Level { get; }

    /// <summary>Source inset of this accepted front; null for pre-built contours.</summary>
    public double? OffsetDistance { get; }

    /// <summary>Index of the source <see cref="MillingRegion"/> this ring came from.</summary>
    public int RegionIndex { get; }

    /// <summary>
    /// Whether this contour bounded a hole before its cutting direction was normalized.
    /// </summary>
    public bool IsHole { get; }

    public double Perimeter { get; }

    public double Area { get; }

    public PointD Centroid { get; }

    /// <summary>Index of the enclosing ring one level out, or -1 when this ring is a root.</summary>
    public int Parent { get; internal set; } = -1;

    public List<int> Children { get; } = new();
}

/// <summary>Ring visit order plus the index ranges that make up each independent island.</summary>
public sealed class RingTraversal
{
    internal RingTraversal(IReadOnlyList<int> order, IReadOnlyList<(int StartIndex, int Count)> islands)
    {
        Order = order;
        Islands = islands;
    }

    public IReadOnlyList<int> Order { get; }

    /// <summary>Slices of <see cref="Order"/>, one per root ring, i.e. one per island.</summary>
    public IReadOnlyList<(int StartIndex, int Count)> Islands { get; }
}

/// <summary>
/// A run of rings that nest one inside the next with no branching, so the cutter can spiral
/// through them continuously instead of retracting and re-entering for each ring.
/// </summary>
public sealed class RingChain
{
    internal RingChain(List<int> ringIndices) => RingIndices = ringIndices;

    public List<int> RingIndices { get; }
}

/// <summary>A current connected offset component with no surviving next closed child.</summary>
internal sealed record CollapsedOffsetComponent(
    MillingRegion Source,
    MillingRegion CurrentComponent,
    int RegionIndex,
    int ParentLevel,
    double CurrentOffsetDistance,
    double CandidateOffsetDistance,
    double SweepRadius);

/// <summary>Inputs to <see cref="RingOffsetEngine.BuildRings"/>.</summary>
/// <param name="FirstOffset">Offset of the first ring from the boundary, normally tool radius + stock to leave.</param>
/// <param name="StepOver">Radial distance between successive rings.</param>
public sealed record RingOffsetOptions(double FirstOffset, double StepOver)
{
    /// <summary>Hard cap on rings per region.</summary>
    public int RingLimit { get; init; } = 10_000;

    /// <summary>Offset direction; negative shrinks inward (pocketing), positive grows outward.</summary>
    public double OffsetSign { get; init; } = -1.0;

    public GeometryTolerances Tolerances { get; init; } = GeometryTolerances.Default;

    /// <summary>Optional fidelity for the outermost legal cutter-center contours.</summary>
    public GeometryTolerances? FirstLevelTolerances { get; init; }

    /// <summary>
    /// Optional enclosed-area cutoff. Null retains the existing stepover-derived
    /// cutoff; zero keeps every nondegenerate closed ring, including thin insets.
    /// Enclosed area does not describe the cutter footprint swept by that ring.
    /// </summary>
    public double? MinimumRingArea { get; init; }

    /// <summary>Additional inset after the first contour. A final partial step reaches this bound.</summary>
    public double MaximumInset { get; init; } = double.PositiveInfinity;

    // A certified demand-contact work bound. Unlike MaximumInset, this does
    // not create a finite band/front or alter any preceding pitch or proof.
    internal double MaximumRelevantDistance { get; init; } = double.PositiveInfinity;
    internal IReadOnlyList<IReadOnlyList<PointD>>? KnownEmptyBoundary { get; init; }

    /// <summary>
    /// Optional canonical filled front at the finite inset bound. Intermediate
    /// trials still offset the prepared source; the authored last front enters
    /// the same spacing and swept-coverage proof as every preceding front.
    /// </summary>
    internal Func<MillingRegion, IReadOnlyList<IReadOnlyList<PointD>>>? MaximumInsetFront { get; init; }

    /// <summary>Enables physical coverage certification for inward fill fronts.</summary>
    internal double? PhysicalSweepRadius { get; init; }

    // Protected stock can split one source into many neighboring fronts. Their
    // already emitted sweeps remain evidence after a completed child disappears.
    internal bool RetainEmittedSweepHistory { get; init; }

    /// <summary>Unchanged shell and coverage operands retain their first exact
    /// residual result when terminal discovery adds no commanded strokes.</summary>
    internal bool ReuseUnchangedShellGap { get; init; } = true;

    /// <summary>Only the final-audited complete ordered front may reuse its
    /// closed sweep when it enters actual emitted-history ownership.</summary>
    internal bool ReuseAuditedClosedSweep { get; init; } = true;

    /// <summary>Actual downstream centerline clip, for coverage-based first-front pruning.</summary>
    internal IReadOnlyList<PointD>? PhysicalClipBoundary { get; init; }

    /// <summary>Bounded source-site reduction for stable offset construction; original walls remain authoritative.</summary>
    internal double SourceConstructionTolerance { get; init; }

    /// <summary>Raw trials retain source validation; the chosen command front is re-audited after cleanup.</summary>
    internal bool DeferTrialCompaction { get; init; } = true;

    /// <summary>OffsetPocket alone may propose selected cleanup without a mutual
    /// raw-footprint proof. SelectCoherentFront must certify the final whole shell
    /// before any commands or sweeps enter emitted history.</summary>
    internal bool SelectedCleanupProposals { get; init; }

    /// <summary>Only deferred adaptive Pocket trials may use a filled dilation
    /// proposal. The selected command front still requires the original complete
    /// clipped sweep and whole-shell audit before it can enter emitted history.</summary>
    internal bool TrialFilledSweepProposals { get; init; }
    internal bool TrialFilledSweepModeCaptured { get; init; }

    /// <summary>Pocket may retain the requested closed inset and nominate bounded
    /// original-source-backed open arcs for a local shell failure. Only the
    /// original complete closed/open/history shell audit authorizes the result.</summary>
    internal bool ParentLocalRescueProposals { get; init; }
    internal bool ParentLocalRescueModeCaptured { get; init; }
    internal PolygonOps.BoundarySegmentIndex? ParentLocalRescueSourceBoundary { get; init; }

    /// <summary>Pocket-only complete reachable-stock certificate may avoid the
    /// eager original graph. Generic fills retain their existing graph authority.</summary>
    internal bool ExactFirstCoverageProposals { get; init; }
    internal bool ExactFirstCoverageModeCaptured { get; init; }

    /// <summary>Operation-local immutable collapsed components may reuse their native terminal result.</summary>
    internal bool CacheCollapsedTerminals { get; init; } = true;
    // OffsetPocket alone opts in. Shared V-carve/Carving fills retain their
    // existing complete angular source graph and cache contract.
    internal bool BoundOriginalSourceMedial { get; init; }
    internal bool PreserveSourceNativeGrid { get; init; }

    // Null retains diagnostic opt-in for generic shared fill callers. Pocket
    // passes its already captured policy; other strategies have no new default.
    internal bool? DenseFirstArcConstruction { get; init; }
    internal bool? TwoOriginConstruction { get; init; }
    internal bool SeparateOriginalFirstConstruction { get; init; }

    /// <summary>Later dense construction may start from the already certified first filled front.</summary>
    internal bool SeedAcceptedFirstInset { get; init; }
    internal bool UseOriginalFirstAuthority { get; init; } = true;

    /// <summary>Native terminal traces for components with no next closed child.</summary>
    internal Func<CollapsedOffsetComponent, CancellationToken, IReadOnlyList<IReadOnlyList<PointD>>>? TerminalPaths { get; init; }

    /// <summary>Reports each attempted zero-based level and its additional inset.</summary>
    public Action<int, double>? LevelProgress { get; init; }
}

/// <summary>
/// Builds the concentric ring set that both pocketing and profiling are made of, works out how
/// the rings nest, and decides what order to cut them in.
///
/// <para>
/// Rings are offset from the <em>original</em> boundary at increasing distance rather than by
/// re-offsetting the previous ring. Re-offsetting compounds Clipper's rounding at every step, so
/// ring spacing slowly drifts away from the requested stepover and cutter engagement drifts with
/// it. Offsetting from the source keeps every ring exactly where it was asked to be.
/// </para>
/// </summary>
public static class RingOffsetEngine
{
    // Dense original boundary strokes compensate two independent cap errors.
    // This opt-in limit leaves room for actual F3 posting inside the unchanged
    // finished-stock reserve; ordinary polygon joins retain one native tick.
    internal const double DenseFirstArcExperimentLimit = .00025;
    // Parent-local trajectories are a dense-source work proposal. Preserve the
    // established coherent closed-front search for ordinary and thin pockets.
    // Qualification uses the immutable ORIGINAL finite sites, not the shrinking
    // current front: a qualified source remains eligible at its later levels.
    internal const int ParentLocalRescueSourceSegmentThreshold = 200_000;
    /// <summary>Absolute ceiling on rings per region, independent of caller configuration.</summary>
    public const int AbsoluteRingLimit = 10_000;

    // --- Ring construction ----------------------------------------------------

    /// <summary>
    /// Generates concentric rings for every region. Stops a region early when its rings stop
    /// shrinking, which is the signal that Clipper has collapsed the geometry rather than
    /// produced a genuine inner ring.
    /// </summary>
    public static List<OffsetRing> BuildRings(
        IReadOnlyList<MillingRegion> regions,
        RingOffsetOptions options,
        CancellationToken cancellationToken = default,
        Action<double, string>? progress = null)
    {
        long t0 = PerfLog.Start();
        options = CaptureExactFirstCoverageMode(CaptureParentLocalRescueMode(CaptureTrialFilledSweepMode(options)));
        var rings = new List<OffsetRing>();

        double stepOver = options.StepOver;
        if (double.IsNaN(options.MaximumInset) || options.MaximumInset < 0)
            throw new ArgumentOutOfRangeException(nameof(options), "Maximum inset must be nonnegative.");
        if (options.PhysicalSweepRadius is { } physicalRadius && (!double.IsFinite(physicalRadius) || physicalRadius <= 0))
            throw new ArgumentOutOfRangeException(nameof(options), "Physical sweep radius must be finite and positive.");
        bool adaptive = options.OffsetSign < 0 && options.PhysicalSweepRadius is { } adaptiveRadius
            && stepOver > adaptiveRadius + 1e-9;
        int ringLimit = Math.Clamp(options.RingLimit, 1, AbsoluteRingLimit);
        if (stepOver <= 1e-9)
            ringLimit = 1;

        double minArea = stepOver > 1e-9 ? stepOver * stepOver * 0.01 : 0;
        double minimumRingArea = options.MinimumRingArea ?? minArea;
        if (!double.IsFinite(minimumRingArea) || minimumRingArea < 0)
            throw new ArgumentOutOfRangeException(nameof(options), "Minimum ring area must be finite and nonnegative.");

        bool denseFirstArcConstruction = options.DenseFirstArcConstruction
            ?? (Environment.GetEnvironmentVariable("RW_TOOLPATHS_FIRST_ARC_EXPERIMENT") == "1"
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_FIRST_ARC_REFERENCE") != "1"
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1");
        bool twoOriginConstruction = options.TwoOriginConstruction
            ?? (Environment.GetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_EXPERIMENT") == "1"
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_REFERENCE") != "1"
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1");
        for (int regionIndex = 0; regionIndex < regions.Count; regionIndex++)
        {
            long regionStart = PerfLog.Start();
            cancellationToken.ThrowIfCancellationRequested();

            var region = regions[regionIndex];
            progress?.Invoke((double)regionIndex / regions.Count, $"Preparing pocket region {regionIndex + 1}/{regions.Count}");
            // An inward ring cannot survive past half the smaller bounding-box span.
            // This is an upper work bound; regions with holes can finish earlier.
            double maxInset = region.Outer.Count == 0 ? 0 : Math.Min(region.Outer.Max(p => p.x) - region.Outer.Min(p => p.x),
                region.Outer.Max(p => p.y) - region.Outer.Min(p => p.y)) / 2;
            double estimatedLevels = options.OffsetSign < 0 && stepOver > 1e-9
                ? Math.Clamp(Math.Ceiling((maxInset - options.FirstOffset) / stepOver) + 1, 1, ringLimit) : ringLimit;
            var firstTolerances = options.FirstLevelTolerances;
            var preparedOffset = PolygonOps.PrepareBoundaryOffset(
                region.Outer,
                region.Holes,
                options.Tolerances.ArcTolerance,
                options.Tolerances.SimplifyTolerance,
                sourceConstructionTolerance: options.SourceConstructionTolerance,
                sourceConstructionSweepRadius: options.PhysicalSweepRadius,
                cancellationToken: cancellationToken,
                sourceConstructionClipBoundary: options.PhysicalClipBoundary,
                sourceConstructionValidateFirstFootprint: firstTolerances is null || firstTolerances == options.Tolerances,
                twoOriginConstruction: twoOriginConstruction,
                separateOriginalFirstConstruction: options.SeparateOriginalFirstConstruction);
            var regionCoherenceOptions = AdmitParentLocalRescueSource(options,
                preparedOffset.OriginalBoundary.SegmentCount, regionIndex) with {
                    ParentLocalRescueSourceBoundary = preparedOffset.OriginalBoundary,
                };
            // Only the dense ORIGINAL boundary-stroke construction explicitly
            // compensates both arc errors before offsetting. Ordinary round
            // polygon joins retain the native-tick first bound: a coarser
            // inscribed join can otherwise shave an original concave corner.
            bool approvedDenseFirstArc = options.OffsetSign < 0 && options.PhysicalSweepRadius is > 0
                && preparedOffset.SourcePointCount > 8192
                && denseFirstArcConstruction;
            if (approvedDenseFirstArc)
            {
                firstTolerances = options.Tolerances with {
                    ArcTolerance = Math.Min(options.Tolerances.ArcTolerance, DenseFirstArcExperimentLimit), SimplifyTolerance = 0 };
                Console.WriteLine($"[RingOffsetEngine.FirstArcExperiment] region={regionIndex} "
                    + $"sourcePoints={preparedOffset.SourcePointCount} firstArc={firstTolerances.ArcTolerance:R} "
                    + "construction=compensated-original-boundary-stroke originalWalls=True "
                    + "terminalSource=complete-original coverageReserve=unchanged ordinaryJoins=native-tick");
            }
            var firstOffset = firstTolerances is null || firstTolerances == options.Tolerances
                ? preparedOffset
                : preparedOffset.WithTolerances(firstTolerances.ArcTolerance,
                    firstTolerances.SimplifyTolerance);
            int regionRingStart = rings.Count;
            int generatedLevels = 0;
            string stopReason = "limit";
            double previousLevelArea = double.MaxValue;
            int areaGrowthLevels = 0;
            double maximumAreaGrowth = 0;
            int retainedSlivers = 0, droppedSmall = 0, droppedDegenerate = 0;
            List<List<PointD>>? previousPolygons = null;
            var emittedSweepHistory = options.KnownEmptyBoundary is {Count:>0} empty
                ? PolygonOps.Intersect(empty,region.AsRingGroup()) : new List<List<PointD>>();
            List<List<PointD>>? previousClosedSweep = null;
            double currentDistance = options.FirstOffset;
            List<List<PointD>>? maximumInsetFront = null;
            PolygonOps.PreparedBoundaryOffset? establishedOffset = null;
            List<List<PointD>>? establishedFirstRequestedFront = null;
            bool useEstablishedConstruction = false;
            int seededConstructionFallbacks = 0;
            long firstCommandCompactTicks = 0, historyClosedSweepTicks = 0;
            int historyClosedSweepReuses = 0, historyClosedSweepRebuilds = 0;
            double establishedFirstRequestedDistance = options.FirstOffset + Math.Min(stepOver, options.MaximumInset);
            List<List<PointD>> EstablishedOffset(double distance, bool compactFront)
            {
                if (IsAuthoredMaximumFront(distance)) return OffsetFromSource(distance);
                if (establishedOffset is null)
                {
                    establishedOffset = preparedOffset.WithTolerances(options.Tolerances.ArcTolerance,
                        options.Tolerances.SimplifyTolerance,
                        firstTolerances is null || firstTolerances == options.Tolerances);
                    // Preserve the former fixed-cache origin: the first requested
                    // later inset, rather than the point where a fallback occurs.
                    establishedFirstRequestedFront = establishedOffset.Execute(
                        options.OffsetSign * establishedFirstRequestedDistance, cancellationToken);
                }
                return compactFront && distance == establishedFirstRequestedDistance
                    ? establishedFirstRequestedFront!
                    : establishedOffset.Execute(options.OffsetSign * distance, cancellationToken, compactFront);
            }
            List<List<PointD>> EstablishedOffsetFromSource(double distance)
            {
                if (!useEstablishedConstruction) seededConstructionFallbacks++;
                useEstablishedConstruction = true;
                return EstablishedOffset(distance, compactFront: true);
            }
            List<List<PointD>> ExecutePrepared(double distance, bool compactFront)
            {
                if (useEstablishedConstruction) return EstablishedOffset(distance, compactFront);
                try { return preparedOffset.Execute(options.OffsetSign * distance, cancellationToken, compactFront); }
                catch (InvalidOperationException error) when (preparedOffset.AcceptedInsetSeeded)
                {
                    Console.WriteLine($"[RingOffsetEngine.AcceptedInsetFallback] region={regionIndex} inset={distance:R} "
                        + $"reason=original-wall-proof error='{error.Message}' fallback=established-fixed-construction");
                    useEstablishedConstruction = true;
                    seededConstructionFallbacks++;
                    return EstablishedOffset(distance, compactFront);
                }
            }
            List<List<PointD>> OffsetFromSource(double distance)
            {
                if (options.MaximumInsetFront is not null && double.IsFinite(options.MaximumInset)
                    && distance == options.FirstOffset + options.MaximumInset)
                {
                    if (maximumInsetFront is null)
                    {
                        maximumInsetFront = options.MaximumInsetFront(region).Select(path => path.ToList()).ToList();
                        Console.WriteLine($"[RingOffsetEngine.BoundedFront] region={regionIndex} inset={distance:R} "
                            + $"contours={maximumInsetFront.Count} origin=canonical-authored-front sharedCoverageProof=True");
                    }
                    return maximumInsetFront;
                }
                return ExecutePrepared(distance, compactFront: true);
            }
            bool IsAuthoredMaximumFront(double distance) => options.MaximumInsetFront is not null
                && double.IsFinite(options.MaximumInset)
                && distance == options.FirstOffset + options.MaximumInset;
            List<List<PointD>> OffsetTrialFromSource(double distance)
                => IsAuthoredMaximumFront(distance) ? OffsetFromSource(distance)
                    : ExecutePrepared(distance, compactFront: false);
            List<List<PointD>> CompactSelected(double distance, List<List<PointD>> candidate)
                => IsAuthoredMaximumFront(distance) ? candidate
                    : preparedOffset.CompactCandidate(candidate, options.OffsetSign * distance, cancellationToken,
                        proposeForShellAudit: options.SelectedCleanupProposals);

            for (int level = 0; level < ringLimit; level++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if(level>0 && options.OffsetSign<0 && currentDistance>options.MaximumRelevantDistance)
                {
                    stopReason="target-contact-exhausted";
                    Console.WriteLine($"[RingOffsetEngine.TargetContact] region={regionIndex} level={level} "
                        + $"lastInset={currentDistance:R} bound={options.MaximumRelevantDistance:R} "
                        + "decision=stop-remote-clearance-levels physicalWall=False");
                    break;
                }

                double additionalInset = adaptive && level > 0
                    ? Math.Min(currentDistance - options.FirstOffset + stepOver, options.MaximumInset)
                    : Math.Min(level * stepOver, options.MaximumInset);
                // Distributing a finite band into equal pitches can leave the
                // last multiplication one ULP below its authored bound. Snap
                // arithmetic roundoff only, so that front uses the canonical
                // cutoff rather than adding a nearly coincident extra contour.
                if (double.IsFinite(options.MaximumInset)
                    && options.MaximumInset - additionalInset <= 8 * (Math.BitIncrement(options.MaximumInset) - options.MaximumInset))
                    additionalInset = options.MaximumInset;
                double distance = options.FirstOffset + additionalInset;
                if (!adaptive || level == 0)
                {
                    options.LevelProgress?.Invoke(level, additionalInset);
                    cancellationToken.ThrowIfCancellationRequested();
                }
                bool deferTrialCompaction = options.DeferTrialCompaction && adaptive && level > 0 && previousPolygons is { Count: > 0 }
                    && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1"
                    && Environment.GetEnvironmentVariable("RW_TOOLPATHS_TRIAL_COMPACTION_REFERENCE") != "1";
                bool originalFirstAuthority = level == 0 && options.PhysicalSweepRadius.HasValue
                    && options.UseOriginalFirstAuthority
                    && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1"
                    && Environment.GetEnvironmentVariable("RW_TOOLPATHS_ORIGINAL_FIRST_REFERENCE") != "1";
                var polygons = level == 0 ? originalFirstAuthority
                    ? firstOffset.ExecuteOriginalFirstAuthority(options.OffsetSign * distance, cancellationToken)
                    : firstOffset.Execute(options.OffsetSign * distance, cancellationToken)
                    : deferTrialCompaction ? OffsetTrialFromSource(distance) : OffsetFromSource(distance);
                if (options.PhysicalSweepRadius is { } firstSweepRadius && level == 0)
                {
                    // Only emitted first-front contours may supply later physical
                    // coverage. A caller's area cutoff cannot create ghost sweeps.
                    foreach (var polygon in polygons)
                    {
                        double area = polygon.Count < 3 ? 0 : Math.Abs(PolygonOps.SignedArea(polygon));
                        if (area == 0) droppedDegenerate++;
                        else if (area < minimumRingArea) droppedSmall++;
                    }
                    polygons = EmittedContours(polygons, minimumRingArea);
                    polygons = RetainUncoveredFirstComponents(polygons, firstSweepRadius,
                        cancellationToken, options.PhysicalClipBoundary);
                    if (Environment.GetEnvironmentVariable("RW_TOOLPATHS_FIRST_COMMAND_EXPERIMENT") == "1"
                        && Environment.GetEnvironmentVariable("RW_TOOLPATHS_FIRST_COMMAND_REFERENCE") != "1"
                        && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1")
                    {
                        long firstCompactStart = Stopwatch.GetTimestamp();
                        polygons = firstOffset.CompactFirstCommands(polygons, options.OffsetSign * distance,
                            cancellationToken);
                        firstCommandCompactTicks += Stopwatch.GetTimestamp() - firstCompactStart;
                    }
                    if (adaptive && (options.SeedAcceptedFirstInset
                            || Environment.GetEnvironmentVariable("RW_TOOLPATHS_ACCEPTED_INSET_EXPERIMENT") == "1"
                            || twoOriginConstruction)
                        && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1"
                        && Environment.GetEnvironmentVariable("RW_TOOLPATHS_ACCEPTED_INSET_REFERENCE") != "1")
                        preparedOffset.TrySeedAcceptedFirstInset(polygons, options.OffsetSign * distance, cancellationToken);
                }
                IReadOnlyList<IReadOnlyList<PointD>> terminalPaths = Array.Empty<IReadOnlyList<PointD>>();
                PreparedClosedSweep? auditedClosedSweep = null;
                if (!adaptive && options.OffsetSign < 0 && options.PhysicalSweepRadius is { } lowPitchRadius
                    && previousPolygons is { Count: > 0 } lowPitchCurrent && level > 0)
                    polygons = options.RetainEmittedSweepHistory
                        ? RemoveAlreadySweptChildren(polygons, emittedSweepHistory, cancellationToken)
                        : RemoveCompletedLowPitchChildren(lowPitchCurrent, polygons, lowPitchRadius,
                            cancellationToken, options.PhysicalClipBoundary);
                if (adaptive && previousPolygons is { Count: > 0 } current && level > 0)
                {
                    // Every trial is still evaluated against the one prepared
                    // original source. Only accepted coherent fronts advance level.
                    var selected = SelectCoherentFront(region, regionIndex, level - 1,
                        current, currentDistance, distance, polygons,
                        regionCoherenceOptions,
                        OffsetFromSource,
                        minimumRingArea, cancellationToken,
                        options.RetainEmittedSweepHistory ? emittedSweepHistory : null,
                        previousClosedSweep,
                        deferTrialCompaction ? OffsetTrialFromSource : null,
                        deferTrialCompaction ? CompactSelected : null,
                        preparedOffset.AcceptedInsetSeeded ? EstablishedOffsetFromSource : null);
                    distance = selected.Distance;
                    additionalInset = distance - options.FirstOffset;
                    polygons = selected.Polygons;
                    terminalPaths = selected.TerminalPaths;
                    auditedClosedSweep = selected.PreparedClosedSweep;
                    options.LevelProgress?.Invoke(level, additionalInset);
                    cancellationToken.ThrowIfCancellationRequested();
                }
                double regionProgress = adaptive && maxInset > options.FirstOffset
                    ? Math.Min(.99, additionalInset / Math.Max(stepOver, Math.Min(maxInset - options.FirstOffset, options.MaximumInset)))
                    : Math.Min(.99, level / estimatedLevels);
                progress?.Invoke((regionIndex + regionProgress) / regions.Count,
                    $"Pocket region {regionIndex + 1}/{regions.Count}, offset {level + 1}, inset {distance:R}");
                foreach (var trace in terminalPaths)
                    rings.Add(new OffsetRing(trace.ToList(), level, regionIndex, isHole: false,
                        isClosed: false, offsetDistance: distance));

                if (options.RetainEmittedSweepHistory && options.PhysicalSweepRadius is { } historyRadius)
                {
                    // Credit accepted paths that the caller will actually emit;
                    // source contours discarded by the area/first-front predicate
                    // and rejected trial fronts never enter this stock owner.
                    var closed = EmittedContours(polygons, minimumRingArea)
                        .Where(path => WhollyEmitted(path, true)).ToList();
                    var open = terminalPaths.Where(path => WhollyEmitted(path, false)).ToList();
                    long historySweepStart = Stopwatch.GetTimestamp();
                    List<List<PointD>>? cachedClosedSweep = null;
                    bool closedSweepReused = auditedClosedSweep is not null
                        && auditedClosedSweep.TryReuse(closed, historyRadius, .001, explicitChords: true,
                            options.PhysicalClipBoundary, cancellationToken, out cachedClosedSweep);
                    var closedSweep = closedSweepReused ? cachedClosedSweep!
                        : BufferClosedSweeps(closed, historyRadius, .001, cancellationToken,
                            explicitChords: true, options.PhysicalClipBoundary);
                    historyClosedSweepTicks += Stopwatch.GetTimestamp() - historySweepStart;
                    if (closedSweepReused) historyClosedSweepReuses++; else historyClosedSweepRebuilds++;
                    if (closedSweepReused
                        && Environment.GetEnvironmentVariable("RW_TOOLPATHS_VALIDATE_COHERENCE_GAP") == "1")
                    {
                        var reconstructed = BufferClosedSweeps(closed, historyRadius, .001, cancellationToken,
                            explicitChords: true, options.PhysicalClipBoundary);
                        if (!PreparedClosedSweep.SameIntegerSweep(closedSweep, reconstructed, cancellationToken))
                        {
                            Console.Error.WriteLine($"[RingOffsetEngine.ClosedSweepMismatch] region={regionIndex} level={level} "
                                + "kind=final-audited-complete-ordered-front exactIntegerSweep=False decision=abort");
                            throw new InvalidOperationException("Final audited closed sweep disagreed with original history construction.");
                        }
                    }
                    // Reuse only the exact complete front. Partially clipped or
                    // area-discarded contours still need the original current
                    // sweep construction during the next proof.
                    previousClosedSweep = closed.Count == polygons.Count ? closedSweep : null;
                    emittedSweepHistory = PolygonOps.Union(emittedSweepHistory
                        .Concat(closedSweep)
                        .Concat(PolygonOps.BufferOpenCenterlines(open, historyRadius, .001, cancellationToken)));
                    Console.WriteLine($"[RingOffsetEngine.EmittedHistory] region={regionIndex} level={level} "
                        + $"closed={closed.Count} open={open.Count} contours={emittedSweepHistory.Count} "
                        + $"closedSweepReused={closedSweepReused} "
                        + $"area={Math.Abs(emittedSweepHistory.Sum(PolygonOps.SignedArea)):R} "
                        + $"owner=accepted-emitted-physical-sweeps explicitEmptySpace={options.KnownEmptyBoundary is {Count:>0}} "
                        + "rejectedTrialsCredited=False");
                }

                if (polygons.Count == 0)
                {
                    stopReason = terminalPaths.Count > 0 ? "terminal-collapse" : "consumed";
                    break;
                }

                double levelArea = Math.Abs(polygons.Sum(PolygonOps.SignedArea));

                // Filled material must shrink monotonically as we move inward. Hole contours
                // expand during an inward offset, so summing each contour's absolute area can
                // grow even while the machinable region is shrinking normally.
                double areaTolerance = previousLevelArea == double.MaxValue
                    ? 0
                    : Math.Max(minArea, previousLevelArea * 1e-9);
                if (levelArea > previousLevelArea + areaTolerance)
                {
                    areaGrowthLevels++;
                    maximumAreaGrowth = Math.Max(
                        maximumAreaGrowth,
                        levelArea - previousLevelArea);
                }
                previousLevelArea = levelArea;
                generatedLevels++;

                foreach (var polygon in polygons)
                {
                    if (polygon.Count < 3)
                    {
                        droppedDegenerate++;
                        continue;
                    }
                    double area = Math.Abs(PolygonOps.SignedArea(polygon));
                    if (area == 0)
                    {
                        droppedDegenerate++;
                        continue;
                    }
                    if (area < minimumRingArea)
                    {
                        droppedSmall++;
                        continue;
                    }
                    if (area < minArea) retainedSlivers++;

                    // The constructor normalizes cutting winding. Keep the
                    // signed filled fronts intact for the next physical audit.
                    rings.Add(new OffsetRing(new List<PointD>(polygon), level, regionIndex,
                        offsetDistance: distance));
                }
                previousPolygons = polygons;
                currentDistance = distance;
                if (additionalInset >= options.MaximumInset)
                {
                    stopReason = "inset-budget";
                    break;
                }
            }

            bool WhollyEmitted(IReadOnlyList<PointD> path, bool closed)
            {
                if (options.PhysicalClipBoundary is not { Count: >= 3 } clip) return true;
                if (path.Count < 2) return false;
                // A clipped contour does not authorize its uncommanded outside
                // strokes. Conservatively withhold history for the whole trace.
                IReadOnlyList<PointD> trace = closed ? path.Concat(new[] { path[0] }).ToArray() : path;
                var outside = CheckedOpenClip.Require(new[] { trace }, new[] { clip }, ClipType.Difference,
                    cancellationToken, "accepted-emitted-history");
                return outside.Count == 0;
            }

            PerfLog.Stop("RingOffsetEngine.RegionOffsets", regionStart,
                $"region={regionIndex} sourcePaths={preparedOffset.SourcePathCount} " +
                $"rings={rings.Count - regionRingStart} levels={generatedLevels} " +
                $"stop={stopReason} netArea={previousLevelArea:F3} " +
                $"areaGrowthLevels={areaGrowthLevels} maxAreaGrowth={maximumAreaGrowth:F3} " +
                $"minimumRingArea={minimumRingArea:R} retainedSlivers={retainedSlivers} " +
                $"droppedSmall={droppedSmall} droppedDegenerate={droppedDegenerate} " +
                $"firstArc={firstTolerances?.ArcTolerance ?? options.Tolerances.ArcTolerance:R} " +
                $"firstSimplify={firstTolerances?.SimplifyTolerance ?? options.Tolerances.SimplifyTolerance:R} " +
                $"firstOffsetMs={firstOffset.OffsetExecutionMilliseconds:F2} " +
                $"firstCommandCompactMs={firstCommandCompactTicks * 1000d / Stopwatch.Frequency:F2} " +
                $"historyClosedSweepMs={historyClosedSweepTicks * 1000d / Stopwatch.Frequency:F2} " +
                $"historyClosedSweepReuses={historyClosedSweepReuses} historyClosedSweepRebuilds={historyClosedSweepRebuilds} " +
                $"firstValidateMs={firstOffset.ValidationMilliseconds:F2} firstFallbackMs={firstOffset.FallbackMilliseconds:F2} " +
                $"firstMaterializeMs={firstOffset.MaterializationMilliseconds:F2} " +
                $"originalFirstAuthorities={firstOffset.OriginalFirstAuthorityCount} " +
                $"originalFirstAuthorityMs={firstOffset.OriginalFirstAuthorityMilliseconds:F2} " +
                $"sourcePreparationReused={firstOffset.SourcePreparationReused} " +
                $"constructionPreparationReused={firstOffset.ConstructionPreparationReused} " +
                $"boundaryPartitions={preparedOffset.BoundaryPartitionCount} " +
                $"boundaryPartitionSortFallbacks={preparedOffset.BoundaryPartitionSortFallbacks} " +
                $"legacyWitnessIndexBuilt={preparedOffset.LegacyWitnessIndexBuilt} " +
                $"sourcePoints={preparedOffset.SourcePointCount} offsets={preparedOffset.OffsetCount} " +
                $"fallbacks={preparedOffset.FallbackCount} " +
                $"prepareMs={preparedOffset.PreparationMilliseconds:F2} " +
                $"executeMs={preparedOffset.OffsetExecutionMilliseconds:F2} " +
                $"validateMs={preparedOffset.ValidationMilliseconds:F2} " +
                $"fallbackMs={preparedOffset.FallbackMilliseconds:F2} " +
                $"materializeMs={preparedOffset.MaterializationMilliseconds:F2} " +
                $"compactions={preparedOffset.CompactionCount} deferredCompactions={preparedOffset.DeferredCompactionCount} " +
                $"compactionProposals={preparedOffset.CompactionProposalCount} " +
                $"compactMs={preparedOffset.CompactionMilliseconds:F2} " +
                $"acceptedInsetSeeded={preparedOffset.AcceptedInsetSeeded} seedMs={preparedOffset.AcceptedInsetSeedMilliseconds:F2} " +
                $"denseInsetReuses={preparedOffset.DenseInsetReuseCount} denseSourceConstructions={preparedOffset.DenseSourceConstructionCount} " +
                $"seededConstructionFallbacks={seededConstructionFallbacks} establishedOffsetCount={establishedOffset?.OffsetCount ?? 0}");
        }

        PerfLog.Stop("RingOffsetEngine.BuildRings", t0,
            $"regions={regions.Count} rings={rings.Count} step={stepOver:F4} limit={ringLimit}");

        progress?.Invoke(1, "Pocket offsets complete");
        return rings;
    }

    private sealed record CoherentFront(double Distance, List<List<PointD>> Polygons,
        IReadOnlyList<IReadOnlyList<PointD>> TerminalPaths, int CollapsedComponents,
        int CompletedComponents, double ShellGapArea, List<List<PointD>> GeneratedCandidate,
        List<List<PointD>> ClosedSweep)
    {
        internal PreparedClosedSweep? PreparedClosedSweep { get; set; }
        internal int UnchangedGapReuses { get; set; }
        internal int TerminalGapRechecks { get; set; }
        internal List<List<PointD>>? Shell { get; init; }
        internal List<List<PointD>>? Covered { get; init; }
        internal List<List<PointD>>? ErodedCore { get; init; }
    }

    internal static List<List<PointD>> BufferClosedSweeps(IEnumerable<IReadOnlyList<PointD>> paths,
        double radius, double arc, CancellationToken token, bool explicitChords,
        IReadOnlyList<PointD>? clipBoundary = null)
    {
        if (!explicitChords) return PolygonOps.BufferCenterlines(paths, radius, arc);
        // A joined closed offset can retain inverted inner folds when the
        // physical cutter is wider than a small contour. Explicit closing
        // chords use the established positive round-capsule union instead.
        var closed = paths.Where(path => path.Count >= 2).Select(path =>
            (IReadOnlyList<PointD>)path.Concat(new[] { path[0] }).ToArray()).ToList();
        if (clipBoundary is { Count: >= 3 })
        {
            closed = CheckedOpenClip.Require(closed, new[] { clipBoundary }, ClipType.Intersection,
                token, "closed-sweep-command-fragments")
                .Select(path => (IReadOnlyList<PointD>)path).ToList();
        }
        return PolygonOps.BufferOpenCenterlines(closed, radius, arc, token);
    }

    internal static RingOffsetOptions CaptureTrialFilledSweepMode(RingOffsetOptions options)
        => options.TrialFilledSweepModeCaptured ? options : options with {
            TrialFilledSweepProposals = options.TrialFilledSweepProposals
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_TRIAL_FILLED_SWEEP_REFERENCE") != "1"
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1",
            TrialFilledSweepModeCaptured = true,
        };

    internal static RingOffsetOptions CaptureParentLocalRescueMode(RingOffsetOptions options)
        => options.ParentLocalRescueModeCaptured ? options : options with {
            ParentLocalRescueProposals = options.ParentLocalRescueProposals
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_PARENT_LOCAL_RESCUE_REFERENCE") != "1"
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1",
            ParentLocalRescueModeCaptured = true,
        };

    internal static RingOffsetOptions AdmitParentLocalRescueSource(RingOffsetOptions options,
        int originalSourceSegments, int regionIndex)
    {
        options = CaptureParentLocalRescueMode(options);
        bool densityEligible = originalSourceSegments >= ParentLocalRescueSourceSegmentThreshold;
        bool enabled = options.ParentLocalRescueProposals && densityEligible;
        Console.WriteLine($"[RingOffsetEngine.ParentLocalRescueEligibility] region={regionIndex} "
            + $"sourceSegments={originalSourceSegments} threshold={ParentLocalRescueSourceSegmentThreshold} "
            + $"densityEligible={densityEligible} capturedFlag={options.ParentLocalRescueProposals} "
            + $"policyCaptured={options.ParentLocalRescueModeCaptured} "
            + $"capturedReferenceOrDisabled={!options.ParentLocalRescueProposals} "
            + $"mode={(enabled ? "qualified-proposal" : "original-search")} "
            + "qualification=complete-original-finite-source currentFrontSizeIndependent=True");
        return options with { ParentLocalRescueProposals = enabled };
    }

    internal static RingOffsetOptions CaptureExactFirstCoverageMode(RingOffsetOptions options)
        => options.ExactFirstCoverageModeCaptured ? options : options with {
            ExactFirstCoverageProposals = options.ExactFirstCoverageProposals
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_EXACT_FIRST_COVERAGE_REFERENCE") != "1"
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1"
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_LOCAL_MEDIAL_REFERENCE") != "1",
            ExactFirstCoverageModeCaptured = true,
        };

    /// <summary>
    /// Builds only a search proposal, never executed removal evidence. The shell
    /// excludes the retained child's signed filled interior, so its dilation can
    /// avoid constructing irrelevant inward folds during raw trials. Native arc
    /// phase and quantization can still differ from a boundary stroke: the final
    /// selected front must rebuild its original stroke and pass the whole Gap.
    /// </summary>
    internal static bool TryBuildTrialFilledSweep(IReadOnlyList<List<PointD>> paths,
        double radius, double arc, bool isClosed, IReadOnlyList<PointD>? clip,
        CancellationToken token, out List<List<PointD>> sweep, out string reason)
        => TryBuildTrialFilledSweep(paths, radius, arc, isClosed, clip, token,
            out sweep, out reason, out _, out _);

    private static bool TryBuildTrialFilledSweep(IReadOnlyList<List<PointD>> paths,
        double radius, double arc, bool isClosed, IReadOnlyList<PointD>? clip,
        CancellationToken token, out List<List<PointD>> sweep, out string reason,
        out long guardTicks, out long constructionTicks)
    {
        token.ThrowIfCancellationRequested();
        long guardStart = Stopwatch.GetTimestamp();
        guardTicks = constructionTicks = 0;
        sweep = new List<List<PointD>>();
        reason = "not-eligible";
        const double maximumIntegerCoordinate = 1_000_000_000_000d;
        if (!isClosed || !double.IsFinite(radius) || radius <= 0 || !double.IsFinite(arc) || arc <= 0
            || radius * PathUtils.Scale > maximumIntegerCoordinate)
        {
            reason = "open-or-radius";
            guardTicks = Stopwatch.GetTimestamp() - guardStart;
            return false;
        }
        bool Valid(IReadOnlyList<PointD> path)
        {
            if (path.Count < 3) return false;
            for (int i = 0; i < path.Count; i++)
            {
                if ((i & 255) == 0) token.ThrowIfCancellationRequested();
                var point = path[i];
                if (!double.IsFinite(point.x) || !double.IsFinite(point.y)
                    || Math.Abs(point.x * PathUtils.Scale) > maximumIntegerCoordinate
                    || Math.Abs(point.y * PathUtils.Scale) > maximumIntegerCoordinate) return false;
            }
            return true;
        }
        long inputVertexCount = 0;
        bool validPaths = true;
        foreach (var path in paths)
        {
            inputVertexCount += path.Count;
            if (inputVertexCount > int.MaxValue || !Valid(path)) { validPaths = false; break; }
        }
        if (!validPaths || (clip is not null && !Valid(clip)))
        {
            reason = "nonfinite-or-integer-bound";
            guardTicks = Stopwatch.GetTimestamp() - guardStart;
            return false;
        }
        var input = PathUtils.ToClipper(paths);
        var canonical = Clipper.Union(input, FillRule.NonZero);
        token.ThrowIfCancellationRequested();
        long canonicalVertexCount = 0;
        foreach (var path in canonical)
        {
            token.ThrowIfCancellationRequested();
            canonicalVertexCount += path.Count;
            if (canonicalVertexCount > int.MaxValue)
            {
                reason = "canonical-integer-bound";
                guardTicks = Stopwatch.GetTimestamp() - guardStart;
                return false;
            }
        }
        Int128 TwiceArea(Path64 path)
        {
            Int128 area = 0;
            for (int i = 0, j = path.Count - 1; i < path.Count; j = i++)
            {
                if ((i & 255) == 0) token.ThrowIfCancellationRequested();
                area += (Int128)path[j].X * path[i].Y - (Int128)path[i].X * path[j].Y;
            }
            return area;
        }
        Int128 Total(Paths64 values)
        {
            Int128 area = 0;
            foreach (var path in values) area += TwiceArea(path);
            return area;
        }
        // Audit normally supplies canonical whole components, including every
        // hole. Reject noncanonical callers rather than guessing a fill domain.
        // Integer coordinates and Int32 vertex counts keep exact area in Int128.
        if (input.Any(path => TwiceArea(path) == 0) || input.Count != canonical.Count
            || input.Count(Clipper.IsPositive) != canonical.Count(Clipper.IsPositive)
            || Total(input) != Total(canonical))
        {
            reason = "noncanonical-signed-domain";
            guardTicks = Stopwatch.GetTimestamp() - guardStart;
            return false;
        }
        if (clip is { Count: >= 3 })
        {
            // A filled child cannot nominate coverage from any closing chord
            // that actual command clipping omits. Include the implicit last edge.
            var traces = paths.Select(path => path.Concat(new[] { path[0] }));
            var engine = new Clipper64();
            engine.AddOpenSubject(PathUtils.ToClipper(traces));
            engine.AddClip(PathUtils.ToClipper(new[] { clip }));
            var outside = new Paths64();
            if (!engine.Execute(ClipType.Difference, FillRule.NonZero, new Paths64(), outside) || outside.Count != 0)
            {
                reason = "partial-command-clip";
                guardTicks = Stopwatch.GetTimestamp() - guardStart;
                return false;
            }
        }
        token.ThrowIfCancellationRequested();
        guardTicks = Stopwatch.GetTimestamp() - guardStart;
        long constructionStart = Stopwatch.GetTimestamp();
        sweep = PolygonOps.Buffer(paths, radius, arc);
        token.ThrowIfCancellationRequested();
        constructionTicks = Stopwatch.GetTimestamp() - constructionStart;
        reason = "canonical-closed-trial-proposal";
        return true;
    }

    private static List<List<PointD>> RemoveAlreadySweptChildren(List<List<PointD>> generated,
        IReadOnlyList<List<PointD>> sweep, CancellationToken token)
    {
        const double proofArc = .001;
        const double coverageError = .003 + 4 / PathUtils.Scale;
        var retained = new List<List<PointD>>();
        int completed = 0;
        foreach (var component in PolygonOps.DifferenceToComponents(generated, Array.Empty<IReadOnlyList<PointD>>()))
        {
            token.ThrowIfCancellationRequested();
            var boundaries = new[] { (IReadOnlyList<PointD>)component.Outer }.Concat(component.Holes).ToList();
            var gap = PolygonOps.Offset(PolygonOps.Difference(boundaries, sweep), -coverageError, proofArc / 4, 0);
            if (Math.Abs(gap.Sum(PolygonOps.SignedArea)) == 0) { completed++; continue; }
            retained.Add(component.Outer); retained.AddRange(component.Holes);
        }
        Console.WriteLine($"[RingOffsetEngine.CompletedHistory] completedChildren={completed} "
            + $"retainedContours={retained.Count} coverageError={coverageError:R} "
            + "proof=accepted-emitted-cutter-sweeps holesPreserved=True");
        return retained;
    }

    private static List<List<PointD>> EmittedContours(IEnumerable<List<PointD>> generated, double minimumRingArea)
        => generated.Where(path => path.Count >= 3
            && Math.Abs(PolygonOps.SignedArea(path)) >= minimumRingArea
            && PolygonOps.SignedArea(path) != 0).ToList();

    internal static List<List<PointD>> RetainUncoveredFirstComponents(
        List<List<PointD>> polygons, double radius, CancellationToken token,
        IReadOnlyList<PointD>? clipBoundary = null)
        => FilledFrontCoverage.RetainUncoveredComponents(polygons, radius, token, clipBoundary);

    private static List<List<PointD>> RemoveCompletedLowPitchChildren(
        IReadOnlyList<List<PointD>> current, List<List<PointD>> generated, double radius,
        CancellationToken token, IReadOnlyList<PointD>? clipBoundary)
    {
        token.ThrowIfCancellationRequested();
        // With no proposed child there is no discard decision or new stock
        // credit to authorize. The already emitted parent remains unchanged.
        // Avoid rebuilding its complete footprint only to return an empty front.
        if (generated.Count == 0
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_EMPTY_CHILD_PROOF_REFERENCE") != "1")
        {
            Console.WriteLine($"[RingOffsetEngine.EmptyChildProof] decision=skip reason=no-generated-child " +
                $"currentContours={current.Count} generatedContours=0 " +
                "physicalCoverageComputed=False stockAuthorityComputed=False parentFrontUnchanged=True");
            return generated;
        }
        // Low pitches need no adaptive shell search, but a filled component whose
        // entire interior has already been swept has no remaining inward work.
        // A nearly coincident next boundary alone cannot prove completion.
        if (clipBoundary is { Count: >= 3 }
            && PolygonOps.Difference(current, new[] { clipBoundary }).Count > 0) return generated;
        const double proofArc = .001;
        const double coverageError = .003 + 4 / PathUtils.Scale;
        var swept = PolygonOps.BufferCenterlines(current, radius, proofArc);
        var completed = new List<List<PointD>>();
        int completedComponents = 0;
        foreach (var component in PolygonOps.DifferenceToComponents(current, Array.Empty<IReadOnlyList<PointD>>()))
        {
            token.ThrowIfCancellationRequested();
            var boundaries = new[] { (IReadOnlyList<PointD>)component.Outer }.Concat(component.Holes).ToList();
            var gap = PolygonOps.Offset(PolygonOps.Difference(boundaries, swept), -coverageError, proofArc / 4, 0);
            if (Math.Abs(gap.Sum(PolygonOps.SignedArea)) != 0) continue;
            completedComponents++;
            completed.Add(component.Outer);
            completed.AddRange(component.Holes);
        }
        if (completedComponents == 0) return generated;
        var retained = new List<List<PointD>>();
        int removed = 0;
        foreach (var component in PolygonOps.DifferenceToComponents(generated, Array.Empty<IReadOnlyList<PointD>>()))
        {
            token.ThrowIfCancellationRequested();
            var boundaries = new[] { (IReadOnlyList<PointD>)component.Outer }.Concat(component.Holes).ToList();
            // Remove the whole child, including its holes, only when it belongs
            // wholly to completed parents. Other children retain their exact front.
            if (PolygonOps.Difference(boundaries, completed).Count == 0) { removed++; continue; }
            retained.Add(component.Outer);
            retained.AddRange(component.Holes);
        }
        Console.WriteLine($"[RingOffsetEngine.CompletedLowPitch] completedParents={completedComponents} "
            + $"removedChildren={removed} retainedContours={retained.Count} "
            + $"coverageError={coverageError:R} proof=complete-filled-interior holesPreserved=True");
        return retained;
    }

    private static CoherentFront SelectCoherentFront(
        MillingRegion source, int regionIndex, int parentLevel,
        IReadOnlyList<List<PointD>> current, double currentDistance, double requestedDistance,
        List<List<PointD>> requestedPolygons, RingOffsetOptions options,
        Func<double, List<List<PointD>>> offsetFromSource, double minimumRingArea,
        CancellationToken token, IReadOnlyList<List<PointD>>? emittedSweepHistory = null,
        IReadOnlyList<List<PointD>>? preparedCurrentSweep = null,
        Func<double, List<List<PointD>>>? rawOffsetFromSource = null,
        Func<double, List<List<PointD>>, List<List<PointD>>>? compactSelected = null,
        Func<double, List<List<PointD>>>? establishedOffsetFromSource = null)
    {
        options = CaptureParentLocalRescueMode(CaptureTrialFilledSweepMode(options));
        const double proofArc = .001;
        const double coverageError = .003 + 4 / PathUtils.Scale;
        const double minimumAdvance = 1 / PathUtils.Scale;
        double radius = options.PhysicalSweepRadius!.Value;
        double requestedAdvance = requestedDistance - currentDistance;
        long coherenceStart = PerfLog.Start();
        var stageTicks = new long[13];
        var stageCalls = new int[13];
        void CompleteStage(int stage, long start)
        {
            stageTicks[stage] += Stopwatch.GetTimestamp() - start;
            stageCalls[stage]++;
        }
        double Milliseconds(int stage) => stageTicks[stage] * 1000d / Stopwatch.Frequency;
        List<List<PointD>> TimedOffset(double distance)
        {
            long start = Stopwatch.GetTimestamp();
            var result = (rawOffsetFromSource ?? offsetFromSource)(distance);
            CompleteStage(9, start);
            return result;
        }
        long prepareStart = Stopwatch.GetTimestamp();
        var currentComponents = PolygonOps.DifferenceToComponents(current, Array.Empty<IReadOnlyList<PointD>>());
        var currentSweep = preparedCurrentSweep?.ToList() ?? BufferClosedSweeps(current, radius, proofArc, token,
            options.RetainEmittedSweepHistory, options.PhysicalClipBoundary);
        if (emittedSweepHistory is { Count: > 0 })
            currentSweep = PolygonOps.Union(currentSweep.Concat(emittedSweepHistory));
        bool preparedGapEnabled = Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1"
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_GAP_REFERENCE") != "1";
        bool validatePreparedGap = Environment.GetEnvironmentVariable("RW_TOOLPATHS_VALIDATE_COHERENCE_GAP") == "1";
        var currentGap = preparedGapEnabled
            ? new PreparedResidualCoverage(PathUtils.ToClipper(currentSweep), token) : null;
        CompleteStage(0, prepareStart);
        PreparedFilledContainment? currentContainment = null;
        int currentGapQueries = 0, containmentProofs = 0, containmentFallbacks = 0;
        int freshDiscLogs = 0, gapValidationQueries = 0;
        int unchangedGapReuses = 0, unchangedGapValidationQueries = 0, terminalGapRechecks = 0;
        bool unchangedGapReuseEnabled = options.ReuseUnchangedShellGap
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1"
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_UNCHANGED_GAP_REFERENCE") != "1";
        bool preparedContainmentEnabled = Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1";
        int trials = 0, certifiedTrials = 0, rejectedTrials = 0;
        int finalCompactionAudits = 0, finalCompactionRejected = 0;
        bool trialFilledMode = options.TrialFilledSweepProposals && compactSelected is not null;
        int trialFilledAttempts = 0, trialFilledUsed = 0, trialFilledFallbacks = 0, trialFilledFinalOriginal = 0;
        long trialFilledInputContours = 0, trialFilledInputPoints = 0;
        long trialFilledGuardTicks = 0, trialFilledConstructionTicks = 0;
        long coverageSubmittedVertices = 0, coverageResidualVertices = 0;
        bool parentLocalMode = options.ParentLocalRescueProposals && compactSelected is not null;
        int parentLocalAttempts = 0, parentLocalAccepted = 0, parentLocalRefusals = 0;
        ParentLocalRescue.Result? parentLocalResult = null;
        bool selectedAlreadyAudited = false;
        CoherentFront? safe = null;
        double unsafeAdvance = requestedAdvance;
        double nextAdvance = requestedAdvance;
        double lastGap = 0;

        while (safe is null)
        {
            token.ThrowIfCancellationRequested();
            double candidateDistance = currentDistance + nextAdvance;
            var polygons = trials == 0 ? requestedPolygons : TimedOffset(candidateDistance);
            var candidate = Audit(candidateDistance, polygons);
            trials++;
            if (candidate.ShellGapArea == 0)
            {
                certifiedTrials++;
                safe = candidate;
                break;
            }
            rejectedTrials++;
            lastGap = candidate.ShellGapArea;
            if (trials == 1 && parentLocalMode)
            {
                // Preserve all original raw-trial callbacks before considering a
                // rescue. The requested selected commands are compacted once and
                // audited with the ORIGINAL complete clipped physical sweep.
                long compactStart = Stopwatch.GetTimestamp();
                var compacted = compactSelected!(requestedDistance, candidate.GeneratedCandidate);
                CompleteStage(10, compactStart);
                var requestedFinal = Audit(requestedDistance, compacted, requireOriginalSweep: true);
                finalCompactionAudits++;
                if (requestedFinal.ShellGapArea == 0)
                {
                    safe = requestedFinal;
                    selectedAlreadyAudited = true;
                    break;
                }
                parentLocalAttempts++;
                long rescueStart = Stopwatch.GetTimestamp();
                long coverageBefore = stageTicks[5] + stageTicks[6];
                var originalBoundary = options.ParentLocalRescueSourceBoundary;
                if (originalBoundary is null)
                {
                    var originalPaths = new List<IReadOnlyList<PointD>> { source.Outer };
                    originalPaths.AddRange(source.Holes);
                    originalBoundary = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(originalPaths));
                }
                parentLocalResult = ParentLocalRescue.Try(new ParentLocalRescue.Input(originalBoundary,
                    currentComponents, requestedFinal.Polygons, requestedFinal.Shell!, requestedFinal.Covered!,
                    requestedFinal.ErodedCore!, options.PhysicalClipBoundary, currentDistance, requestedDistance, radius),
                    token, (shell, covered) => Gap(shell, covered, forceReference: true));
                CompleteStage(12, rescueStart);
                // The complete original subtract/erosion are already exclusive
                // stage owners; local proposal work excludes those child timers.
                stageTicks[12] -= stageTicks[5] + stageTicks[6] - coverageBefore;
                if (parentLocalResult.Accepted)
                {
                    token.ThrowIfCancellationRequested();
                    parentLocalAccepted++;
                    safe = requestedFinal with {
                        TerminalPaths = requestedFinal.TerminalPaths.Concat(parentLocalResult.Traces).ToList(),
                        ShellGapArea = 0,
                        Shell = null, Covered = null, ErodedCore = null,
                    };
                    selectedAlreadyAudited = true;
                    Console.WriteLine($"[RingOffsetEngine.ParentLocalRescue] event=accept region={regionIndex} level={parentLevel + 1} "
                        + $"requestedInset={requestedDistance:R} owners={parentLocalResult.Owners} parentPoints={parentLocalResult.ParentPoints} "
                        + $"localOffsets={parentLocalResult.Attempts} tracePoints={parentLocalResult.TracePoints} "
                        + $"gapBefore={parentLocalResult.GapBefore:R} gapAfter={parentLocalResult.GapAfter:R} "
                        + "originalRequestedSweep=True finalCommandFrontProof=True fullSignedShellProof=True "
                        + "requestedClosedDistanceUnchanged=True rejectedProposalCredited=False");
                    break;
                }
                parentLocalRefusals++;
                Console.WriteLine($"[RingOffsetEngine.ParentLocalRescue] event=fallback region={regionIndex} level={parentLevel + 1} "
                    + $"requestedInset={requestedDistance:R} reason={parentLocalResult.Reason} localOffsets={parentLocalResult.Attempts} "
                    + $"gapBefore={parentLocalResult.GapBefore:R} gapAfter={parentLocalResult.GapAfter:R} "
                    + "fallback=original-halving-and-eight-refinements rejectedProposalCredited=False");
                // No paths, distance, prefix or prepared closed sweep changed.
                // Continue precisely the original search from its first refusal.
            }
            unsafeAdvance = nextAdvance;
            nextAdvance /= 2;
            if (nextAdvance < minimumAdvance || currentDistance + nextAdvance == currentDistance)
            {
                if (establishedOffsetFromSource is not null)
                {
                    Console.WriteLine($"[RingOffsetEngine.AcceptedInsetFallback] region={regionIndex} level={parentLevel + 1} "
                        + $"shellGapArea={lastGap:R} reason=seeded-construction-advance-unproved "
                        + "fallback=established-compacted-trial-search");
                    ReportStages(accepted: false);
                    return SelectCoherentFront(source, regionIndex, parentLevel, current, currentDistance,
                        requestedDistance, establishedOffsetFromSource(requestedDistance), options with {
                            SelectedCleanupProposals = false, TrialFilledSweepProposals = false, ParentLocalRescueProposals = false },
                        establishedOffsetFromSource, minimumRingArea, token, emittedSweepHistory, preparedCurrentSweep);
                }
                Console.WriteLine($"[RingOffsetEngine.Coherence] event=reject region={regionIndex} level={parentLevel + 1} " +
                    $"currentInset={currentDistance:R} requestedAdvance={requestedAdvance:R} actualAdvance={nextAdvance:R} " +
                    $"refinements={trials} shellGapArea={lastGap:R} coverageError={coverageError:R} reason=numerical-advance-floor");
                throw new InvalidOperationException("A coherent inward offset front could not be certified above the geometry quantum.");
            }
        }

        // Once a safe lower bound is found, recover the largest certified
        // advance in the bracket instead of replacing the authored pitch by half.
        if (safe.Distance < requestedDistance)
        for (int refinement = 0; refinement < 8; refinement++)
        {
            token.ThrowIfCancellationRequested();
            double safeAdvance = safe.Distance - currentDistance;
            if (unsafeAdvance - safeAdvance <= 4 * proofArc) break;
            double advance = (safeAdvance + unsafeAdvance) / 2;
            double candidateDistance = currentDistance + advance;
            var candidate = Audit(candidateDistance, TimedOffset(candidateDistance));
            trials++;
            if (candidate.ShellGapArea == 0) { certifiedTrials++; safe = candidate; }
            else { rejectedTrials++; unsafeAdvance = advance; lastGap = candidate.ShellGapArea; }
        }
        if (compactSelected is not null && !selectedAlreadyAudited)
        {
            long compactStart = Stopwatch.GetTimestamp();
            var compacted = compactSelected(safe.Distance, safe.GeneratedCandidate);
            CompleteStage(10, compactStart);
            var final = Audit(safe.Distance, compacted, requireOriginalSweep: true);
            finalCompactionAudits++;
            if (final.ShellGapArea != 0)
            {
                finalCompactionRejected++;
                Console.WriteLine($"[RingOffsetEngine.CoherenceCompaction] event=fallback region={regionIndex} level={parentLevel + 1} "
                    + $"selectedInset={safe.Distance:R} compactShellGapArea={final.ShellGapArea:R} "
                    + $"selectedCleanupProposal={options.SelectedCleanupProposals} "
                    + "reason=final-command-front-unproved fallback=established-compacted-trial-search fullMutualProofRequired=True rejectedProposalCredited=False");
                ReportStages(accepted: false);
                // Repeat the established search against precisely the original
                // source construction and compact every attempted front. A raw
                // trial never authorizes an unproved final commanded front.
                var fallback = establishedOffsetFromSource ?? offsetFromSource;
                return SelectCoherentFront(source, regionIndex, parentLevel, current, currentDistance,
                    requestedDistance, fallback(requestedDistance), options with {
                        SelectedCleanupProposals = false, TrialFilledSweepProposals = false, ParentLocalRescueProposals = false }, fallback,
                    minimumRingArea, token, emittedSweepHistory, preparedCurrentSweep);
            }
            safe = final;
            Console.WriteLine($"[RingOffsetEngine.CoherenceCompaction] event=accept region={regionIndex} level={parentLevel + 1} "
                + $"selectedInset={safe.Distance:R} rawSearchTrials={trials} compactedCandidates=1 "
                + $"selectedCleanupProposal={options.SelectedCleanupProposals} "
                + "finalCommandFrontProof=True fullSignedShellProof=True rejectedRawFrontsCredited=False");
        }
        Console.WriteLine($"[RingOffsetEngine.Coherence] event=accept region={regionIndex} level={parentLevel + 1} " +
            $"acceptedInset={safe.Distance:R} requestedAdvance={requestedAdvance:R} actualAdvance={safe.Distance - currentDistance:R} " +
            $"refinements={trials - 1} collapsedComponents={safe.CollapsedComponents} terminalCount={safe.TerminalPaths.Count} " +
            $"completedComponents={safe.CompletedComponents} " +
            $"shellGapArea={safe.ShellGapArea:R} rejectedShellGapArea={lastGap:R} coverageError={coverageError:R}");
        if (options.RetainEmittedSweepHistory && options.ReuseAuditedClosedSweep
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1"
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_CLOSED_SWEEP_REFERENCE") != "1")
        {
            long acceptedSweepStart = Stopwatch.GetTimestamp();
            safe.PreparedClosedSweep = new PreparedClosedSweep(safe.Polygons, safe.ClosedSweep,
                radius, proofArc, explicitChords: true, options.PhysicalClipBoundary, token);
            CompleteStage(11, acceptedSweepStart);
        }
        ReportStages(accepted: true);
        safe.UnchangedGapReuses = unchangedGapReuses;
        safe.TerminalGapRechecks = terminalGapRechecks;
        return safe;

        void ReportStages(bool accepted) => PerfLog.Stop("RingOffsetEngine.CoherenceStages", coherenceStart,
            $"region={regionIndex} level={parentLevel + 1} trials={trials} certifiedTrials={certifiedTrials} rejectedTrials={rejectedTrials} acceptedFronts={(accepted ? 1 : 0)} "
            + $"currentPrepareMs={Milliseconds(0):F2} completedChildMs={Milliseconds(1):F2} "
            + $"shellMs={Milliseconds(2):F2} nextSweepMs={Milliseconds(3):F2} "
            + $"sweepUnionMs={Milliseconds(4):F2} coverageSubtractMs={Milliseconds(5):F2} "
            + $"coverageErodeMs={Milliseconds(6):F2} collapseFindMs={Milliseconds(7):F2} "
            + $"terminalMs={Milliseconds(8):F2} coverageQueries={stageCalls[5]} "
            + $"trialOffsetMs={Milliseconds(9):F2} extraOffsetTrials={stageCalls[9]} "
            + $"coverageSubmittedVertices={coverageSubmittedVertices} coverageResidualVertices={coverageResidualVertices} "
            + $"containmentProofs={containmentProofs} containmentFallbacks={containmentFallbacks} "
            + $"containmentEnabled={preparedContainmentEnabled} currentSweepPreparedReused={preparedCurrentSweep is not null} "
            + $"trialCompactionDeferred={compactSelected is not null} selectedCompactMs={Milliseconds(10):F2} "
            + $"selectedCleanupProposal={options.SelectedCleanupProposals && compactSelected is not null} "
            + $"trialFilledMode={trialFilledMode} trialFilledAttempts={trialFilledAttempts} trialFilledUsed={trialFilledUsed} "
            + $"trialFilledFallbacks={trialFilledFallbacks} trialFilledFinalOriginal={trialFilledFinalOriginal} "
            + $"trialFilledInputContours={trialFilledInputContours} trialFilledInputPoints={trialFilledInputPoints} "
            + $"trialFilledGuardMs={trialFilledGuardTicks * 1000d / Stopwatch.Frequency:F2} "
            + $"trialFilledConstructionMs={trialFilledConstructionTicks * 1000d / Stopwatch.Frequency:F2} trialFilledTimingParent=nextSweepMs "
            + $"selectedCompactions={stageCalls[10]} finalCompactionAudits={finalCompactionAudits} finalCompactionRejected={finalCompactionRejected} "
            + $"acceptedSweepPrepareMs={Milliseconds(11):F2} acceptedSweepSnapshots={stageCalls[11]} "
            + $"parentLocalMode={parentLocalMode} parentLocalAttempts={parentLocalAttempts} parentLocalAccepted={parentLocalAccepted} "
            + $"parentLocalRefusals={parentLocalRefusals} parentLocalProposalMs={Milliseconds(12):F2} "
            + $"parentLocalOwners={parentLocalResult?.Owners ?? 0} parentLocalParentPoints={parentLocalResult?.ParentPoints ?? 0} "
            + $"parentLocalOffsetCalls={parentLocalResult?.Attempts ?? 0} parentLocalOffsetPoints={parentLocalResult?.OffsetPoints ?? 0} "
            + $"parentLocalTracePoints={parentLocalResult?.TracePoints ?? 0} parentLocalSourceChords={parentLocalResult?.SourceChords ?? 0} "
            + $"parentLocalSupportRefusals={parentLocalResult?.SupportRefusals ?? 0} "
            + $"parentLocalOffsetMs={parentLocalResult?.OffsetMs ?? 0:F2} parentLocalProofMs={parentLocalResult?.ProofMs ?? 0:F2} "
            + "parentLocalChildTimingParent=parentLocalProposalMs-and-coverageSubtractMs-and-coverageErodeMs "
            + $"preparedGapEnabled={preparedGapEnabled} gapValidationQueries={gapValidationQueries} "
            + $"unchangedGapReuseEnabled={unchangedGapReuseEnabled} unchangedGapReuses={unchangedGapReuses} "
            + $"unchangedGapValidationQueries={unchangedGapValidationQueries} terminalGapRechecks={terminalGapRechecks} "
            + $"freshDiscQueries={currentGap?.WitnessQueries ?? 0} freshDiscProofs={currentGap?.FreshDiscProofs ?? 0} "
            + $"freshDiscProbes={currentGap?.WitnessProbes ?? 0} freshDiscMs={(currentGap?.WitnessTicks ?? 0) * 1000d / Stopwatch.Frequency:F2} "
            + $"freshDiscRadius={currentGap?.LargestWitnessRadius ?? 0:R} freshDiscTimingParent=completedChildMs "
            + $"preparedDifferenceQueries={currentGap?.DifferenceQueries ?? 0} "
            + $"preparedDifferenceConsideredVertices={currentGap?.ConsideredVertices ?? 0} preparedDifferenceSelectedVertices={currentGap?.SelectedVertices ?? 0} "
            + $"preparedClipInputBuilds={currentGap?.InputBuilds ?? 0} preparedClipInputReuses={currentGap?.InputReuses ?? 0} "
            + $"preparedBoundsMs={(currentGap?.BoundsTicks ?? 0) * 1000d / Stopwatch.Frequency:F2} "
            + $"preparedClipInputMs={(currentGap?.InputTicks ?? 0) * 1000d / Stopwatch.Frequency:F2} preparedDifferenceTimingParent=coverageSubtractMs "
            + $"currentPoints={current.Sum(path => path.Count)}");

        CoherentFront Audit(double candidateDistance, List<List<PointD>> generated, bool requireOriginalSweep = false)
        {
            token.ThrowIfCancellationRequested();
            // The proof must use contours that will actually be emitted, including
            // any caller-authored area cutoff. Terminal callbacks never stand in
            // for a connected component that still has a closed next child.
            var emitted = EmittedContours(generated, minimumRingArea);
            var polygons = new List<List<PointD>>();
            int completed = 0;
            long completedStart = Stopwatch.GetTimestamp();
            long completedCoverageBefore = stageTicks[5] + stageTicks[6];
            foreach (var component in PolygonOps.DifferenceToComponents(emitted, Array.Empty<IReadOnlyList<PointD>>()))
            {
                token.ThrowIfCancellationRequested();
                var boundaries = new List<IReadOnlyList<PointD>> { component.Outer };
                boundaries.AddRange(component.Holes);
                // A complete disc of new material proves that this child is still
                // needed. Sampling discovers the center; signed whole-disc tests
                // against both immutable domains provide the proof. No witness
                // authorizes completion, and ambiguity retains the original Gap.
                var integerBoundaries = currentGap is null ? null : PathUtils.ToClipper(boundaries);
                Point64 witness = default;
                bool fresh = currentGap is not null && currentGap.TryFreshDisc(integerBoundaries!,
                    coverageError, proofArc, token, out witness);
                if (fresh)
                {
                    if (validatePreparedGap)
                    {
                        gapValidationQueries++;
                        if (Gap(boundaries, currentSweep, forceReference: true) == 0)
                        {
                            Console.Error.WriteLine($"[RingOffsetEngine.PreparedGapMismatch] region={regionIndex} level={parentLevel + 1} "
                                + $"kind=fresh-disc witness=({witness.X},{witness.Y}) radius={currentGap!.LastWitnessRadius:R} "
                                + "originalGap=0 decision=abort");
                            throw new InvalidOperationException("Fresh-disc child retention disagreed with original coverage morphology.");
                        }
                    }
                    if (freshDiscLogs++ < 4)
                        Console.WriteLine($"[RingOffsetEngine.FreshDisc] region={regionIndex} level={parentLevel + 1} "
                            + $"witness=({witness.X / PathUtils.Scale:R},{witness.Y / PathUtils.Scale:R}) "
                            + $"radius={currentGap!.LastWitnessRadius:R} originalCoverage=True fullSignedDisc=True decision=retain-child");
                    polygons.Add(component.Outer);
                    polygons.AddRange(component.Holes);
                    continue;
                }
                // Terminate a completed filled component as a unit, including its
                // holes. Tiny contours remain when their physical cut is needed;
                // this predicate measures swept coverage rather than path size.
                bool contained = false;
                if (preparedContainmentEnabled && ++currentGapQueries > 1)
                {
                    currentContainment ??= new PreparedFilledContainment(PathUtils.ToClipper(currentSweep));
                    contained = currentContainment.ContainsCanonical(PathUtils.ToClipper(boundaries), token) is true;
                    if (contained) containmentProofs++; else containmentFallbacks++;
                }
                if (contained || Gap(boundaries, currentSweep) == 0) { completed++; continue; }
                polygons.Add(component.Outer);
                polygons.AddRange(component.Holes);
            }
            // Exclusive child work: Gap records its subtract/erosion stages below.
            CompleteStage(1, completedStart);
            stageTicks[1] -= stageTicks[5] + stageTicks[6] - completedCoverageBefore;
            long shellStart = Stopwatch.GetTimestamp();
            var shell = PolygonOps.Difference(current, polygons);
            CompleteStage(2, shellStart);
            long sweepStart = Stopwatch.GetTimestamp();
            List<List<PointD>> nextSweep;
            if (requireOriginalSweep) trialFilledFinalOriginal++;
            if (trialFilledMode && !requireOriginalSweep)
            {
                trialFilledAttempts++;
                trialFilledInputContours += polygons.Count;
                trialFilledInputPoints += polygons.Sum(path => (long)path.Count);
                bool proposed = TryBuildTrialFilledSweep(polygons, radius, proofArc, isClosed: true,
                    options.PhysicalClipBoundary, token, out nextSweep, out string reason,
                    out long guardTicks, out long constructionTicks);
                trialFilledGuardTicks += guardTicks;
                trialFilledConstructionTicks += constructionTicks;
                if (proposed) trialFilledUsed++;
                else
                {
                    trialFilledFallbacks++;
                    if (trialFilledFallbacks <= 4)
                        Console.WriteLine($"[RingOffsetEngine.TrialFilledSweep] region={regionIndex} level={parentLevel + 1} "
                            + $"reason={reason} decision=original-sweep-fallback stockAuthorized=False");
                    nextSweep = BufferClosedSweeps(polygons, radius, proofArc, token,
                        options.RetainEmittedSweepHistory, options.PhysicalClipBoundary);
                }
            }
            else nextSweep = BufferClosedSweeps(polygons, radius, proofArc, token,
                options.RetainEmittedSweepHistory, options.PhysicalClipBoundary);
            CompleteStage(3, sweepStart);
            long unionStart = Stopwatch.GetTimestamp();
            var covered = PolygonOps.Union(currentSweep.Concat(nextSweep));
            CompleteStage(4, unionStart);
            List<List<PointD>>? erodedCore = null;
            Action<List<List<PointD>>>? captureCore = requireOriginalSweep && parentLocalMode
                ? result => erodedCore = result : null;
            double gap = Gap(shell, covered, captureCore: captureCore);
            if (gap == 0)
                return new CoherentFront(candidateDistance, polygons, Array.Empty<IReadOnlyList<PointD>>(), 0, completed, 0, generated, nextSweep);
            var terminals = new List<IReadOnlyList<PointD>>();
            int collapsed = 0;
            foreach (var component in currentComponents)
            {
                token.ThrowIfCancellationRequested();
                var boundaries = new List<IReadOnlyList<PointD>> { component.Outer };
                boundaries.AddRange(component.Holes);
                long findStart = Stopwatch.GetTimestamp();
                var child = polygons.Count == 0 ? new List<List<PointD>>()
                    : PolygonOps.Intersect(boundaries, polygons);
                CompleteStage(7, findStart);
                if (child.Count > 0 && Math.Abs(child.Sum(PolygonOps.SignedArea)) > 0) continue;
                collapsed++;
                if (options.TerminalPaths is null) continue;
                var context = new CollapsedOffsetComponent(source,
                    new MillingRegion(component.Outer, component.Holes.Select(hole => (IReadOnlyList<PointD>)hole).ToList()),
                    regionIndex, parentLevel, currentDistance, candidateDistance, radius);
                long terminalStart = Stopwatch.GetTimestamp();
                var paths = options.TerminalPaths(context, token);
                CompleteStage(8, terminalStart);
                token.ThrowIfCancellationRequested();
                foreach (var path in paths)
                {
                    if (path.Count < 2) continue;
                    if (path.Any(point => !double.IsFinite(point.x) || !double.IsFinite(point.y)))
                        throw new InvalidOperationException("Collapsed offset terminal contains nonfinite coordinates.");
                    terminals.Add(path);
                }
            }
            if (terminals.Count > 0)
            {
                long terminalSweepStart = Stopwatch.GetTimestamp();
                covered = PolygonOps.Union(covered.Concat(PolygonOps.BufferOpenCenterlines(terminals, radius, proofArc, token)));
                CompleteStage(4, terminalSweepStart);
                terminalGapRechecks++;
                gap = Gap(shell, covered, captureCore: captureCore);
            }
            else if (unchangedGapReuseEnabled)
            {
                // Terminal discovery still ran for every collapsed component.
                // No accepted trace means both original operands are identical,
                // so the existing subtract/erode result remains exact authority.
                unchangedGapReuses++;
                if (validatePreparedGap)
                {
                    unchangedGapValidationQueries++;
                    double original = Gap(shell, covered, forceReference: true);
                    if (gap != original)
                    {
                        Console.Error.WriteLine($"[RingOffsetEngine.PreparedGapMismatch] region={regionIndex} level={parentLevel + 1} "
                            + $"kind=unchanged-shell-and-coverage retainedGap={gap:R} originalGap={original:R} decision=abort");
                        throw new InvalidOperationException("Identical shell and coverage operands changed their exact residual result.");
                    }
                }
            }
            else gap = Gap(shell, covered, captureCore: captureCore);
            return new CoherentFront(candidateDistance, polygons, terminals, collapsed, completed, gap, generated, nextSweep) {
                Shell = captureCore is null ? null : shell,
                Covered = captureCore is null ? null : covered,
                ErodedCore = erodedCore,
            };
        }

        double Gap(IEnumerable<IReadOnlyList<PointD>> shell, IEnumerable<IReadOnlyList<PointD>> covered,
            bool forceReference = false, Action<List<List<PointD>>>? captureCore = null)
        {
            token.ThrowIfCancellationRequested();
            coverageSubmittedVertices += shell.Sum(path => path.Count) + covered.Sum(path => path.Count);
            long subtractStart = Stopwatch.GetTimestamp();
            bool usePrepared = !forceReference && currentGap is not null && ReferenceEquals(covered, currentSweep);
            var uncovered = usePrepared
                ? currentGap!.Difference(PathUtils.ToClipper(shell), token)
                : PolygonOps.Difference(shell, covered);
            coverageResidualVertices += uncovered.Sum(path => path.Count);
            CompleteStage(5, subtractStart);
            long erodeStart = Stopwatch.GetTimestamp();
            var interior = PolygonOps.Offset(uncovered, -coverageError, proofArc / 4, 0);
            CompleteStage(6, erodeStart);
            token.ThrowIfCancellationRequested();
            double area = Math.Abs(interior.Sum(PolygonOps.SignedArea));
            captureCore?.Invoke(interior);
            if (usePrepared && validatePreparedGap)
            {
                gapValidationQueries++;
                double reference = Gap(shell, covered, forceReference: true);
                if ((area == 0) != (reference == 0))
                {
                    Console.Error.WriteLine($"[RingOffsetEngine.PreparedGapMismatch] region={regionIndex} level={parentLevel + 1} "
                        + $"kind=exact-signed-contour-difference preparedGap={area:R} originalGap={reference:R} decision=abort");
                    throw new InvalidOperationException("Prepared exact coverage difference disagreed with original coverage morphology.");
                }
            }
            return area;
        }
    }

    /// <summary>
    /// Wraps a pre-built polygon as a ring without offsetting, for callers that generate their
    /// rings elsewhere (profile passes and spring passes).
    /// </summary>
    public static OffsetRing CreateRing(
        IReadOnlyList<PointD> polygon,
        int level,
        int regionIndex,
        bool? isHole = null,
        bool isClosed = true,
        double? offsetDistance = null)
        => new(new List<PointD>(polygon), level, regionIndex, isHole, isClosed, offsetDistance);

    // --- Nesting --------------------------------------------------------------

    /// <summary>
    /// Links each ring to the enclosing ring exactly one level out in the same region, choosing
    /// the closest candidate by boundary proximity. Rings at the same level in the same region
    /// never nest inside each other, so proximity resolves the parent unambiguously and far more
    /// cheaply than a full containment test.
    /// </summary>
    public static void BuildNesting(IReadOnlyList<OffsetRing> rings)
    {
        long t0 = PerfLog.Start();

        foreach (var ring in rings)
        {
            ring.Parent = -1;
            ring.Children.Clear();
        }

        var ringsByRegionAndLevel = new Dictionary<(int RegionIndex, int Level), List<int>>();
        for (int ringIndex = 0; ringIndex < rings.Count; ringIndex++)
        {
            var ring = rings[ringIndex];
            var key = (ring.RegionIndex, ring.Level);
            if (!ringsByRegionAndLevel.TryGetValue(key, out var levelRings))
            {
                levelRings = new List<int>();
                ringsByRegionAndLevel.Add(key, levelRings);
            }
            levelRings.Add(ringIndex);
        }

        long candidateChecks = 0;

        for (int i = 0; i < rings.Count; i++)
        {
            var ring = rings[i];
            if (ring.Level == 0)
                continue;

            if (!ringsByRegionAndLevel.TryGetValue(
                    (ring.RegionIndex, ring.Level - 1),
                    out var parentCandidates))
            {
                continue;
            }

            int bestParent = -1;
            double bestDistanceSq = double.MaxValue;

            foreach (int j in parentCandidates)
            {
                var candidate = rings[j];
                if (!candidate.IsClosed) continue;
                candidateChecks++;
                double distanceSq = ClosestApproachSq(ring.Points, candidate.Points);
                if (distanceSq < bestDistanceSq)
                {
                    bestDistanceSq = distanceSq;
                    bestParent = j;
                }
            }

            ring.Parent = bestParent;
            if (bestParent >= 0)
                rings[bestParent].Children.Add(i);
        }

        PerfLog.Stop("RingOffsetEngine.BuildNesting", t0,
            $"rings={rings.Count} candidateChecks={candidateChecks}");
    }

    /// <summary>Sampled closest approach between two rings; exact distance is not required to rank parents.</summary>
    private static double ClosestApproachSq(IReadOnlyList<PointD> a, IReadOnlyList<PointD> b)
    {
        int stepA = Math.Max(1, a.Count / 8);
        int stepB = Math.Max(1, b.Count / 8);
        double best = double.MaxValue;

        for (int i = 0; i < a.Count; i += stepA)
        {
            for (int j = 0; j < b.Count; j += stepB)
            {
                double dx = a[i].x - b[j].x;
                double dy = a[i].y - b[j].y;
                double d = dx * dx + dy * dy;
                if (d < best)
                    best = d;
            }
        }

        return best;
    }

    // --- Traversal ------------------------------------------------------------

    /// <summary>
    /// Orders rings depth-first so every disconnected pocket is finished before the next one
    /// starts, keeping the cutter local instead of hopping between islands.
    ///
    /// <para>
    /// With <paramref name="insideOut"/> the post-order emits children before their parent, so
    /// enclosed material is removed while it is still supported. Cutting the enclosing ring first
    /// would leave the island floating and free to move under the cutter.
    /// </para>
    /// </summary>
    public static RingTraversal BuildTraversal(IReadOnlyList<OffsetRing> rings, bool insideOut)
    {
        long t0 = PerfLog.Start();

        var order = new List<int>(rings.Count);
        var islands = new List<(int StartIndex, int Count)>();

        var roots = new List<int>();
        for (int i = 0; i < rings.Count; i++)
        {
            if (rings[i].Parent == -1)
                roots.Add(i);
        }

        if (roots.Count > 1)
        {
            // Inside-out starts from the tightest island; outside-in from the largest.
            roots.Sort((a, b) => insideOut
                ? rings[a].Area.CompareTo(rings[b].Area)
                : rings[b].Area.CompareTo(rings[a].Area));
        }

        double lastX = double.NaN, lastY = double.NaN;

        void SortChildrenByProximity(List<int> children)
        {
            if (children.Count <= 1)
                return;

            if (double.IsNaN(lastX))
            {
                children.Sort((a, b) => rings[b].Area.CompareTo(rings[a].Area));
                return;
            }

            double curX = lastX, curY = lastY;
            for (int i = 0; i < children.Count - 1; i++)
            {
                int best = i;
                double bestDistance = double.MaxValue;
                for (int j = i; j < children.Count; j++)
                {
                    var c = rings[children[j]].Centroid;
                    double d = (c.x - curX) * (c.x - curX) + (c.y - curY) * (c.y - curY);
                    if (d < bestDistance)
                    {
                        bestDistance = d;
                        best = j;
                    }
                }
                if (best != i)
                    (children[i], children[best]) = (children[best], children[i]);

                curX = rings[children[i]].Centroid.x;
                curY = rings[children[i]].Centroid.y;
            }
        }

        void Visit(int index)
        {
            if (insideOut)
            {
                SortChildrenByProximity(rings[index].Children);
                foreach (int child in rings[index].Children)
                    Visit(child);
                order.Add(index);
            }
            else
            {
                order.Add(index);
                SortChildrenByProximity(rings[index].Children);
                foreach (int child in rings[index].Children)
                    Visit(child);
            }

            lastX = rings[index].Centroid.x;
            lastY = rings[index].Centroid.y;
        }

        var pending = new List<int>(roots);
        while (pending.Count > 0)
        {
            int pick = 0;
            if (!double.IsNaN(lastX) && pending.Count > 1)
            {
                double bestDistance = double.MaxValue;
                for (int i = 0; i < pending.Count; i++)
                {
                    var c = rings[pending[i]].Centroid;
                    double d = (c.x - lastX) * (c.x - lastX) + (c.y - lastY) * (c.y - lastY);
                    if (d < bestDistance)
                    {
                        bestDistance = d;
                        pick = i;
                    }
                }
            }

            int root = pending[pick];
            pending.RemoveAt(pick);

            int start = order.Count;
            Visit(root);
            islands.Add((start, order.Count - start));
        }

        PerfLog.Stop("RingOffsetEngine.BuildTraversal", t0,
            $"rings={rings.Count} islands={islands.Count} insideOut={insideOut}");

        return new RingTraversal(order, islands);
    }

    // --- Spiral chaining ------------------------------------------------------

    /// <summary>
    /// Groups consecutive rings in the traversal into chains that can be cut without lifting.
    /// A chain continues while each ring leads to exactly one nested neighbour; branching or a
    /// dead end closes it. Fewer chains means fewer plunges, retracts and entry marks.
    /// </summary>
    public static List<RingChain> BuildChains(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<int> order)
    {
        var chains = new List<RingChain>();
        if (order.Count == 0)
            return chains;

        var current = new List<int> { order[0] };

        for (int i = 1; i < order.Count; i++)
        {
            int previous = order[i - 1];
            int next = order[i];

            // Continuous only when the two rings are directly nested and the outer one has no
            // other branch competing for the cutter.
            bool nested =
                (rings[next].Parent == previous && rings[previous].Children.Count == 1) ||
                (rings[previous].Parent == next && rings[next].Children.Count == 1);

            if (nested)
            {
                current.Add(next);
            }
            else
            {
                chains.Add(new RingChain(current));
                current = new List<int> { next };
            }
        }

        chains.Add(new RingChain(current));
        return chains;
    }

    /// <summary>
    /// Flattens a chain into one continuous polyline, rotating each ring so it begins near where
    /// the previous ring ended. Without that the link between rings would be a long chord across
    /// the pocket instead of a short step-over move.
    /// </summary>
    public static List<PointD> MaterializeChain(
        IReadOnlyList<OffsetRing> rings,
        RingChain chain,
        MillingDirection direction)
    {
        var points = new List<PointD>();

        for (int i = 0; i < chain.RingIndices.Count; i++)
        {
            var geometry = rings[chain.RingIndices[i]];
            var ring = Orient(geometry, direction);

            if (points.Count > 0)
            {
                if (geometry.IsClosed) ring = PathUtils.RebaseNear(ring, points[^1]);
                else if (DistanceSquared(ring[^1], points[^1]) < DistanceSquared(ring[0], points[^1])) ring.Reverse();
            }

            points.AddRange(ring);

            // Close each ring back onto its own start before stepping to the next one.
            if (geometry.IsClosed && ring.Count > 0)
                points.Add(ring[0]);
        }

        return points;
    }

    /// <summary>Applies milling direction to a ring, returning a copy.</summary>
    public static List<PointD> Orient(IReadOnlyList<PointD> ring, MillingDirection direction)
    {
        if (direction == MillingDirection.Default)
            return new List<PointD>(ring);

        bool wantCcw = direction == MillingDirection.Climb;
        bool isCcw = PolygonOps.SignedArea(ring) > 0;
        if (isCcw == wantCcw)
            return new List<PointD>(ring);

        var reversed = new List<PointD>(ring);
        reversed.Reverse();
        return reversed;
    }

    /// <summary>Preserves open topology; winding applies only to closed contours.</summary>
    public static List<PointD> Orient(OffsetRing path, MillingDirection direction)
    {
        if (path.IsClosed) return Orient(path.Points, direction);
        var points = new List<PointD>(path.Points);
        if (direction == MillingDirection.Conventional) points.Reverse();
        return points;
    }

    private static double DistanceSquared(PointD a, PointD b)
        => (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}
