using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Milling.Acceleration;

namespace RW.Toolpaths.Milling;

/// <summary>
/// One-sided certificate of the entire fixed, canonical NonZero target. Each
/// accepted box has a real target-exterior or penetrating cutter-disc witness.
/// The signed distance to one fixed disc is 1-Lipschitz, including its interior;
/// retaining that negative margin avoids subdividing already-cut stock to 4um.
/// Failure or exhaustion is inconclusive, never permission to discard a residual.
/// This proves coverage only; cutter containment is a separate caller contract.
/// </summary>
internal static partial class VCarveCoverageCertificate
{
    internal const double ArithmeticReserve = 1e-9;
    internal const double ConvexBoxArithmeticReserve = 1e-8;
    internal const double InteriorClassificationReserve = 3e-5;
    private const double MinimumInteriorRayDelta = 1e-100;

    internal sealed record Limits
    {
        internal long MaximumBoxes { get; init; } = 500_000;
        internal long MaximumDiscEvaluations { get; init; } = 20_000_000;
        internal int MaximumInputSegments { get; init; } = 4_000_000;
        internal TimeSpan MaximumElapsed { get; init; } = TimeSpan.FromSeconds(30);
        internal double MinimumBoxRadius { get; init; } = 1e-10;
        internal bool RecordWitnessPathIndices { get; init; }
        internal IReadOnlySet<int>? OptionalWitnessPathIndices { get; init; }
        internal bool UseAnalyticPoseNomination { get; init; }
        internal bool UseAffineRadiusPruning { get; init; }
        internal bool UseInheritedInteriorClassification { get; init; }
        internal bool UseSphereDiscPruning { get; init; }
        internal bool UsePreparedSphereDiscPruning { get; init; }
        internal bool UseCachedTreeBounds { get; init; }
        internal bool UseGpuDiscWitnesses { get; init; }
        internal VCarveGenerationPolicy? GenerationPolicy { get; init; }
        internal bool UseGpuTriangleDomain { get; init; }
        internal bool UseGpuBoundaryNominations { get; init; }
        internal bool UseDelaunayTriangleDomain { get; init; } = true;
        internal PreparedTriangleDomain? TriangleDomainPreparation { get; init; }
        internal int MinimumGpuIntervals { get; init; } = 8192;
        internal int GpuBatchSize { get; init; } = 4096;
        internal bool UseCachedWindingRows { get; init; }
        internal bool UseContourBoxSeeds { get; init; }
        internal bool UseGpuConvexBoxWitnesses { get; init; } = true;
    }

    internal sealed record Result(bool Certified, string Reason, long VisitedBoxes,
        long DiscEvaluations, int BoundarySegments, int CommandIntervals,
        int RemainingBoxes, PointD? UnresolvedPoint, double ElapsedSeconds,
        int CommandPathCount, bool WitnessPathsRecorded, IReadOnlyList<int> WitnessPathIndices)
    {
        internal long AnalyticPoseCalls { get; init; }
        internal long AnalyticPoseHits { get; init; }
        internal long AnalyticPoseFallbacks { get; init; }
        internal long AnalyticPoseEvaluations { get; init; }
        internal long AnalyticPoseDuplicates { get; init; }
        internal long AnalyticPoseInvalid { get; init; }
        internal long AffineRadiusCalls { get; init; }
        internal long AffineRadiusRejections { get; init; }
        internal long AffineRadiusFallbacks { get; init; }
        internal long AffineRadiusGuardRefusals { get; init; }
        internal bool InteriorClassificationEligible { get; init; }
        internal long SkippedBoundaryQueries { get; init; }
        internal long StrictInteriorSeeds { get; init; }
        internal long InteriorEligibilityGuardRefusals { get; init; }
        internal long SphereDiscCalls { get; init; }
        internal long SphereDiscRejections { get; init; }
        internal long SphereDiscFallbacks { get; init; }
        internal long SphereDiscGuardRefusals { get; init; }
        internal long SpherePreparedIntervals { get; init; }
        internal long SpherePreparedSlots { get; init; }
        internal long SpherePreparationGuardRefusals { get; init; }
        internal long CachedTreeBoundReuses { get; init; }
        internal bool GpuAttempted { get; init; }
        internal bool GpuAvailable { get; init; }
        internal string GpuReason { get; init; } = "disabled";
        internal long GpuQueries { get; init; }
        internal long GpuWitnesses { get; init; }
        internal long GpuOptionalQueries { get; init; }
        internal long GpuOptionalWitnesses { get; init; }
        internal long GpuBoxQueries { get; init; }
        internal long GpuBoxWitnesses { get; init; }
        internal long GpuFallbackQueries { get; init; }
        internal long GpuSubdivisionMisses { get; init; }
        internal long GpuBatches { get; init; }
        internal double GpuKernelMilliseconds { get; init; }
        internal double GpuTransferMilliseconds { get; init; }
        internal long WindingRowHits { get; init; }
        internal long WindingRowMisses { get; init; }
        internal long WindingRowAmbiguousEvaluations { get; init; }
        internal long WindingRowRefusals { get; init; }
        internal bool ContourBoxSeedsRequested { get; init; }
        internal bool ContourBoxSeedsUsed { get; init; }
        internal int ContourSeedBoxes { get; init; }
        internal long TargetBoxExclusions { get; init; }
        internal long TargetExteriorCapExclusions { get; init; }
        internal long SkippedBoundaryDistanceQueries { get; init; }
        internal string CoverageDomain { get; init; } = "boxes";
        internal bool TriangleDomainAttempted { get; init; }
        internal bool TriangleDomainUsed { get; init; }
        internal string TriangleDomainReason { get; init; } = "disabled";
        internal int TriangleSourceTriangles { get; init; }
        internal int TriangleCorrectionTriangles { get; init; }
        internal double TrianglePreparationMilliseconds { get; init; }
        internal bool TrianglePreparationReused { get; init; }
        internal long VisitedTriangles { get; init; }
        internal long GpuTriangleQueries { get; init; }
        internal long GpuTriangleWitnesses { get; init; }
        internal long GpuOptionalTriangleQueries { get; init; }
        internal long GpuOptionalTriangleWitnesses { get; init; }
        internal long TriangleBoundaryWitnesses { get; init; }
        internal long TriangleSubdivisions { get; init; }
        internal long TriangleFallbackQueries { get; init; }
        internal long TriangleBigIntegerFallbacks { get; init; }
        internal long TriangleBoundaryNominationVisits { get; init; }
        internal long TriangleBoundaryNominationCandidates { get; init; }
        internal long TriangleBoundaryNominationRefusals { get; init; }
        internal bool TriangleBoundaryGpuAvailable { get; init; }
        internal string TriangleBoundaryGpuReason { get; init; } = "disabled";
        internal long TriangleBoundaryGpuQueries { get; init; }
        internal long TriangleBoundaryGpuWitnesses { get; init; }
        internal long TriangleBoundaryGpuMisses { get; init; }
        internal long TriangleBoundaryGpuRefusals { get; init; }
    }

    /// <param name="target">Canonical signed outer/hole rings of the immutable
    /// target; boundaries must be real target/exterior interfaces, not overlapping
    /// un-unioned source contours.</param>
    /// <param name="commandPaths">Actual commanded coordinates, with the supplied
    /// surface Z. This helper performs no rounding and invents no endpoint closure.</param>
    internal static Result Certify(IReadOnlyList<IReadOnlyList<PointD>> target,
        IReadOnlyList<TaggedToolpath> commandPaths, VCarveCutterProfile profile,
        double outputSurfaceZ = 0, double gapRadius = .004,
        CancellationToken cancellationToken = default, Limits? limits = null,
        Action<string>? diagnostic = null)
    {
        ArgumentNullException.ThrowIfNull(target);
        ArgumentNullException.ThrowIfNull(commandPaths);
        ArgumentNullException.ThrowIfNull(profile);
        if (!double.IsFinite(outputSurfaceZ) || !double.IsFinite(gapRadius) || gapRadius <= ArithmeticReserve)
            throw new ArgumentOutOfRangeException(nameof(gapRadius));
        limits ??= new Limits();
        if (limits.MaximumBoxes < 0 || limits.MaximumDiscEvaluations < 0 || limits.MaximumInputSegments < 0
            || limits.MaximumElapsed <= TimeSpan.Zero || !double.IsFinite(limits.MinimumBoxRadius)
            || limits.MinimumBoxRadius <= 0 || limits.MinimumGpuIntervals < 1
            || limits.GpuBatchSize < 1 || limits.GpuBatchSize > 4096)
            throw new ArgumentOutOfRangeException(nameof(limits));

        var work = new Budget(limits, cancellationToken) { Diagnostic = diagnostic };
        var pending = new Stack<PendingBox>();
        PointD? unresolved = null;
        long boxes = 0;
        int boundaries = 0, intervals = 0;
        int mandatoryIntervals = 0, optionalIntervals = 0;
        bool[]? witnessedPaths = null;
        bool interiorClassificationEligible = false, rayEdgesOrdinary = true;
        long preparationStarted = Stopwatch.GetTimestamp(), queryStarted = 0;
        double preparationMilliseconds = 0;
        VCarveGpuDiscWitnessContext? gpuContext = null;
        VCarveGpuDiscWitnessContext? optionalGpuContext = null;
        bool gpuAttempted = false, gpuAvailable = false;
        string gpuReason = limits.UseGpuDiscWitnesses ? "below-interval-threshold" : "disabled";
        long gpuQueries = 0, gpuWitnesses = 0, gpuFallbackQueries = 0, gpuSubdivisionMisses = 0;
        long gpuOptionalQueries = 0, gpuOptionalWitnesses = 0;
        long gpuBoxQueries = 0, gpuBoxWitnesses = 0;
        long targetBoxExclusions = 0, targetExteriorCapExclusions = 0, skippedBoundaryDistanceQueries = 0;
        bool targetBoxExclusionEligible = false;
        bool contourBoxSeedsUsed = false;
        int contourSeedBoxes = 0;
        int gpuOutstanding = 0;
        TriangleRun? triangleRun = null;
        Tree? exteriorForGpu = null, mandatoryForGpu = null, optionalForGpu = null;
        diagnostic?.Invoke($"stage=smooth-physical-certificate event=start gapRadius={gapRadius:R} " +
            $"maximumBoxes={limits.MaximumBoxes} maximumDiscEvaluations={limits.MaximumDiscEvaluations} " +
            $"maximumElapsedSeconds={limits.MaximumElapsed.TotalSeconds:R} " +
            $"analyticPoseNomination={limits.UseAnalyticPoseNomination} affineRadiusPruning={limits.UseAffineRadiusPruning} " +
            $"inheritedInteriorClassification={limits.UseInheritedInteriorClassification} " +
            $"sphereDiscPruning={limits.UseSphereDiscPruning} " +
            $"preparedSphereDiscPruning={limits.UsePreparedSphereDiscPruning} cachedTreeBounds={limits.UseCachedTreeBounds} " +
            $"gpuDiscWitnesses={limits.UseGpuDiscWitnesses} gpuBatchSize={limits.GpuBatchSize} " +
            $"contourBoxSeedsRequested={limits.UseContourBoxSeeds} " +
            $"domain={(limits.UseContourBoxSeeds ? "entire-fixed-target-ring-box-cover" : "entire-fixed-target-bounding-box")}");
        try
        {
            work.CheckNow();
            if (commandPaths.Count > limits.MaximumInputSegments) return Finish(false, "path-budget");
            if (limits.OptionalWitnessPathIndices?.Any(index => index < 0 || index >= commandPaths.Count) == true)
                throw new ArgumentOutOfRangeException(nameof(limits), "Optional witness path ordinals must match the command snapshot.");
            if (limits.RecordWitnessPathIndices) witnessedPaths = new bool[commandPaths.Count];
            var boundary = new List<Segment>();
            var contourSeedBounds = limits.UseContourBoxSeeds ? new List<Box>() : null;
            foreach (var ring in target)
            {
                // Polygon Boolean output has at least three sites. Ignoring a
                // malformed caller target could produce a false vacuous success.
                if (ring.Count < 3) throw new ArgumentException("Target rings require at least three points.", nameof(target));
                double seedX0 = double.PositiveInfinity, seedY0 = double.PositiveInfinity;
                double seedX1 = double.NegativeInfinity, seedY1 = double.NegativeInfinity;
                for (int i = 0; i < ring.Count; i++)
                {
                    work.Input();
                    PointD a = ring[i], b = ring[(i + 1) % ring.Count];
                    if (!double.IsFinite(a.x) || !double.IsFinite(a.y)
                        || !double.IsFinite(b.x) || !double.IsFinite(b.y))
                        throw new ArgumentException("Target coordinates must be finite.", nameof(target));
                    if (a.y != b.y
                        && Math.Abs(b.y - a.y) < MinimumInteriorRayDelta) rayEdgesOrdinary = false;
                    boundary.Add(new Segment(new(a.x, a.y, 0), new(b.x, b.y, 0), 0, -1));
                    if (contourSeedBounds is not null)
                    {
                        seedX0 = Math.Min(seedX0, Math.Min(a.x, b.x));
                        seedY0 = Math.Min(seedY0, Math.Min(a.y, b.y));
                        seedX1 = Math.Max(seedX1, Math.Max(a.x, b.x));
                        seedY1 = Math.Max(seedY1, Math.Max(a.y, b.y));
                    }
                }
                if (contourSeedBounds is not null)
                    contourSeedBounds.Add(new(seedX0, seedY0, seedX1, seedY1));
            }
            boundaries = boundary.Count;
            if (boundaries == 0) return Finish(true, "empty-target");
            var exterior = new Tree(boundary.ToArray(), work);
            Box domain = exterior.Bounds;
            if (domain.X1 <= domain.X0 || domain.Y1 <= domain.Y0)
                throw new ArgumentException("Target must have a nondegenerate bounding box.", nameof(target));
            targetBoxExclusionEligible = rayEdgesOrdinary
                && Ordinary(domain.X0) && Ordinary(domain.Y0) && Ordinary(domain.X1) && Ordinary(domain.Y1);
            if (limits.UseInheritedInteriorClassification)
            {
                interiorClassificationEligible = rayEdgesOrdinary
                    && Ordinary(domain.X0) && Ordinary(domain.Y0) && Ordinary(domain.X1) && Ordinary(domain.Y1);
                if (!interiorClassificationEligible) work.InteriorEligibilityGuardRefusals++;
            }

            var pieces = new List<Segment>();
            var sectionDepths = new List<double> { 0 };
            foreach (var section in profile.Sections) sectionDepths.Add(sectionDepths[^1] + section.Height);
            for (int pathIndex = 0; pathIndex < commandPaths.Count; pathIndex++)
            {
                var path = commandPaths[pathIndex];
                for (int i = 1; i < path.Points.Count; i++)
                {
                    work.Input();
                    // Retracts carry no cutting authority. Unannotated failure
                    // payloads retain the actual consecutive commanded chords.
                    if (path.Spans.Count != 0 && !path.Spans.Any(span => span.StartIndex < i
                        && span.EndIndex >= i && span.Kind != ToolpathSpanKind.Retract)) continue;
                    Point3D rawA = path.Points[i - 1], rawB = path.Points[i];
                    if (!Finite(rawA) || !Finite(rawB))
                        throw new ArgumentException("Commanded coordinates must be finite.", nameof(commandPaths));
                    var a = new Point3D(rawA.X, rawA.Y, outputSurfaceZ - rawA.Z);
                    var b = new Point3D(rawB.X, rawB.Y, outputSurfaceZ - rawB.Z);
                    if (Math.Max(a.Z, b.Z) <= 0) continue;
                    var breaks = new List<double> { 0, 1 };
                    if (a.Z != b.Z)
                        foreach (double depth in sectionDepths)
                        {
                            double t = (depth - a.Z) / (b.Z - a.Z);
                            if (t > 0 && t < 1) breaks.Add(t);
                        }
                    breaks.Sort();
                    for (int part = 1; part < breaks.Count; part++)
                    {
                        Point3D first = At(a, b, breaks[part - 1]), last = At(a, b, breaks[part]);
                        if (Math.Max(first.Z, last.Z) <= 0) continue;
                        work.Piece(pieces.Count);
                        pieces.Add(new Segment(first, last,
                            profile.PhysicalRadiusAtDepth(Math.Max(first.Z, last.Z)), pathIndex));
                    }
                }
            }
            intervals = pieces.Count;
            var mandatoryPieces = limits.OptionalWitnessPathIndices is null ? pieces.ToArray()
                : pieces.Where(piece => !limits.OptionalWitnessPathIndices.Contains(piece.SourcePathIndex)).ToArray();
            var optionalPieces = limits.OptionalWitnessPathIndices is null ? Array.Empty<Segment>()
                : pieces.Where(piece => limits.OptionalWitnessPathIndices.Contains(piece.SourcePathIndex)).ToArray();
            mandatoryIntervals = mandatoryPieces.Length; optionalIntervals = optionalPieces.Length;
            var mandatoryDiscs = mandatoryIntervals == 0 ? null : new Tree(mandatoryPieces, work);
            var optionalDiscs = optionalIntervals == 0 ? null : new Tree(optionalPieces, work);
            exteriorForGpu = exterior;
            mandatoryForGpu = mandatoryDiscs;
            optionalForGpu = optionalDiscs;
            if (limits.UseGpuDiscWitnesses && mandatoryIntervals >= limits.MinimumGpuIntervals)
            {
                gpuAttempted = true;
                gpuAvailable = mandatoryDiscs!.TryCreateGpuContext(profile, cancellationToken, diagnostic,
                    out gpuContext, out gpuReason, limits.GenerationPolicy);
                if (gpuAvailable) gpuReason = "ready";
                diagnostic?.Invoke($"stage=smooth-physical-certificate event=gpu-selection available={gpuAvailable} " +
                    $"reason={gpuReason} mandatoryIntervals={mandatoryIntervals} batchSize={limits.GpuBatchSize}");
                if (gpuAvailable && optionalDiscs is not null)
                {
                    bool optionalAvailable = optionalDiscs.TryCreateGpuContext(profile, cancellationToken, diagnostic,
                        out optionalGpuContext, out string optionalReason, limits.GenerationPolicy);
                    diagnostic?.Invoke($"stage=smooth-physical-certificate event=gpu-optional-selection available={optionalAvailable} " +
                        $"reason={optionalReason} optionalIntervals={optionalIntervals} preference=mandatory-first");
                }
                work.CheckNow();
            }
            queryStarted = Stopwatch.GetTimestamp();
            preparationMilliseconds = Stopwatch.GetElapsedTime(preparationStarted, queryStarted).TotalMilliseconds;
            if (limits.UseGpuTriangleDomain && gpuAvailable)
            {
                triangleRun = new TriangleRun(limits, work, exterior, mandatoryDiscs!, optionalDiscs,
                    gpuContext!, optionalGpuContext, profile, gapRadius, witnessedPaths, diagnostic);
                bool? triangleResult = triangleRun.TryCertify(target);
                if (triangleResult.HasValue) return Finish(triangleResult.Value, triangleRun.Reason);
            }
            if (limits.UseContourBoxSeeds && targetBoxExclusionEligible)
            {
                // A closed ring has zero winding at every point outside its
                // endpoint AABB. Hence the union of ALL ring AABBs covers every
                // point of this same fixed NonZero target, including nested,
                // negative, overlapping and self-retraced rings. No ring, hole,
                // boundary predicate or target area is discarded. Each seed is
                // a conservative superset; the original full-target exterior
                // tree still owns every accepted box. Overlaps may cost proof
                // work but cannot grant coverage. One shared Budget owns every
                // seed, subsequent split, disc query and elapsed-time check.
                if (contourSeedBounds!.Count > limits.MaximumBoxes)
                    return Finish(false, "contour-seed-box-budget");
                for (int ringIndex = contourSeedBounds.Count - 1; ringIndex >= 0; ringIndex--)
                {
                    work.CheckNow();
                    // Bounds were frozen from the SAME literal endpoint pairs
                    // as the original exterior-tree input, before callbacks or
                    // physical preparation. No later caller-target reread owns
                    // the cover or can omit an earlier captured boundary.
                    Box captured = contourSeedBounds[ringIndex];
                    double x0 = captured.X0, y0 = captured.Y0;
                    double x1 = captured.X1, y1 = captured.Y1;
                    // Outward ulps include endpoint boundaries and make even a
                    // degenerate ring seed nondegenerate. Ordinary global bounds
                    // exclude infinity/overflow; all vertices remain enclosed.
                    var seed = new Box(Math.BitDecrement(x0), Math.BitDecrement(y0),
                        Math.BitIncrement(x1), Math.BitIncrement(y1));
                    pending.Push(new(seed, false));
                    contourSeedBoxes++;
                }
                contourBoxSeedsUsed = true;
                diagnostic?.Invoke($"stage=smooth-physical-certificate event=contour-seeds seeds={contourSeedBoxes} " +
                    "allTargetRingsRetained=True fillRule=original-NonZero exteriorOwner=whole-fixed-target " +
                    "sharedOriginalQuotas=True disjointnessRequired=False witnessAuthority=unchanged");
                work.CheckNow();
            }
            else
            {
                pending.Push(new(domain, false));
                if (limits.UseContourBoxSeeds)
                    diagnostic?.Invoke("stage=smooth-physical-certificate event=contour-seeds-fallback reason=original-bounded-domain-eligibility originalGlobalBox=True");
            }
            if (gpuAvailable) return CertifyGpuBatches();
            long reported = Stopwatch.GetTimestamp();
            while (pending.TryPop(out PendingBox task))
            {
                Box box = task.Bounds;
                work.CheckNow();
                if (boxes >= limits.MaximumBoxes) { pending.Push(task); return Finish(false, "box-budget"); }
                boxes++;
                if ((boxes & 1023) == 0 && Stopwatch.GetElapsedTime(reported).TotalSeconds >= 2)
                {
                    reported = Stopwatch.GetTimestamp();
                    diagnostic?.Invoke($"stage=smooth-physical-certificate event=progress boxes={boxes} pending={pending.Count} " +
                        $"discEvaluations={work.DiscEvaluations} elapsedSeconds={work.Elapsed:R}");
                }
                var q = new PointD(box.X0 + (box.X1 - box.X0) / 2, box.Y0 + (box.Y1 - box.Y0) / 2);
                double radius = Math.Sqrt(Square((box.X1 - box.X0) / 2) + Square((box.Y1 - box.Y0) / 2));
                double boundaryDistance = 0;
                bool inside = true;
                if (task.StrictInterior) work.SkippedBoundaryQueries += 2;
                else
                {
                    boundaryDistance = exterior.BoundaryDistance(q);
                    inside = exterior.IsInside(q);
                    // Outside-center boxes cannot cross the canonical target
                    // boundary if this conservative exclusion bound is positive.
                    if (!inside && boundaryDistance - ArithmeticReserve > radius) continue;
                    // Canonical target signed distance is positive in the target
                    // and negative outside. Its 1-Lipschitz bound also certifies a
                    // mostly-exterior box whose small target cap is within the
                    // permitted width, without subdividing its empty interior.
                    double exteriorUpper = inside ? boundaryDistance : -boundaryDistance;
                    if (exteriorUpper + radius + ArithmeticReserve <= gapRadius) continue;
                }
                double desired = gapRadius - radius - ArithmeticReserve;
                int pathIndex = -1;
                // Prefer explicit mandatory owners under the identical physical
                // inequality. Optional terminal owners remain a complete fallback;
                // this changes witness selection, never the coverage criterion.
                if (mandatoryDiscs is not null && mandatoryDiscs.HasDiscWitness(q, desired, profile, out pathIndex)
                    || optionalDiscs is not null && optionalDiscs.HasDiscWitness(q, desired, profile, out pathIndex))
                {
                    if (witnessedPaths is not null) witnessedPaths[pathIndex] = true;
                    continue;
                }
                if (radius <= limits.MinimumBoxRadius)
                {
                    unresolved = q;
                    return Finish(false, "unresolved-box");
                }
                // Ordinary coordinates bound nearest-projection, BVH-distance,
                // center and radius roundoff far below 1e-5mm. Reserve that much
                // separately for the parent's distance, its circumball, and any
                // descendant's literal distance: the latter remains >gap even
                // if it rounds downward. Nonhorizontal dy>=1e-100 also prevents
                // winding cross-product underflow here. For a ray-straddling
                // edge, |cross|>=clearance*|dy|, while its roundoff is O(u*S*|dy|),
                // S<=1e4; this strict clearance dominates it. Thus every child is
                // inside under the original winding test and neither exterior
                // branch can accept. An inherited flag supplies no disc witness.
                bool strictChildren = task.StrictInterior || interiorClassificationEligible && inside
                    && Math.BitDecrement(boundaryDistance - radius - InteriorClassificationReserve) > gapRadius;
                if (box.X1 - box.X0 >= box.Y1 - box.Y0)
                {
                    if (q.x == box.X0 || q.x == box.X1) { unresolved = q; return Finish(false, "coordinate-resolution"); }
                    if (strictChildren && !task.StrictInterior) work.StrictInteriorSeeds++;
                    pending.Push(new(new(box.X0, box.Y0, q.x, box.Y1), strictChildren));
                    pending.Push(new(new(q.x, box.Y0, box.X1, box.Y1), strictChildren));
                }
                else
                {
                    if (q.y == box.Y0 || q.y == box.Y1) { unresolved = q; return Finish(false, "coordinate-resolution"); }
                    if (strictChildren && !task.StrictInterior) work.StrictInteriorSeeds++;
                    pending.Push(new(new(box.X0, box.Y0, box.X1, q.y), strictChildren));
                    pending.Push(new(new(box.X0, q.y, box.X1, box.Y1), strictChildren));
                }
            }
            return Finish(true, "whole-target-certified");
        }
        catch (Exhausted error) { return Finish(false, error.Message); }
        finally { optionalGpuContext?.Dispose(); gpuContext?.Dispose(); }

        Result CertifyGpuBatches()
        {
            // The device only nominates real commanded poses. It supplies no
            // target classification or negative coverage authority. A missing
            // nomination may subdivide a box; it can never accept or erase one.
            var tasks = new PendingGpuBox[limits.GpuBatchSize];
            var queries = new GpuDiscQuery[limits.GpuBatchSize];
            var nominations = new GpuDiscNomination[limits.GpuBatchSize];
            var discQueries = new GpuDiscQuery[limits.GpuBatchSize];
            var discNominations = new GpuDiscNomination[limits.GpuBatchSize];
            var discOrdinals = new int[limits.GpuBatchSize];
            var boxQueries = new GpuBoxQuery[limits.GpuBatchSize];
            var boxNominations = new GpuBoxNomination[limits.GpuBatchSize];
            var boxAccepted = new bool[limits.GpuBatchSize];
            var boxOwners = new int[limits.GpuBatchSize];
            var optionalQueries = new GpuDiscQuery[limits.GpuBatchSize];
            var optionalNominations = new GpuDiscNomination[limits.GpuBatchSize];
            var optionalOrdinals = new int[limits.GpuBatchSize];
            var optionalResults = new GpuDiscNomination[limits.GpuBatchSize];
            // Keep one quota callback for this certificate rather than creating
            // an instance-method delegate for every device box nomination.
            Action discCheckpoint = work.Disc;
            bool deviceHealthy = true;
            bool optionalDeviceHealthy = optionalGpuContext is not null;
            bool boxDeviceHealthy = limits.UseGpuConvexBoxWitnesses && gapRadius > ConvexBoxArithmeticReserve
                && gpuContext!.SupportsConvexBoxWitnesses;
            long reported = Stopwatch.GetTimestamp();
            while (pending.Count != 0)
            {
                int count = 0;
                while (count < tasks.Length && pending.TryPop(out PendingBox task))
                {
                    work.CheckNow();
                    if (boxes >= limits.MaximumBoxes)
                    { pending.Push(task); return Finish(false, "box-budget"); }
                    boxes++;
                    Box box = task.Bounds;
                    var q = new PointD(box.X0 + (box.X1 - box.X0) / 2, box.Y0 + (box.Y1 - box.Y0) / 2);
                    double radius = Math.Sqrt(Square((box.X1 - box.X0) / 2) + Square((box.Y1 - box.Y0) / 2));
                    double boundaryDistance = 0;
                    bool inside = true;
                    if (task.StrictInterior) work.SkippedBoundaryQueries += 2;
                    else
                    {
                        inside = exteriorForGpu!.IsInside(q);
                        // The original signed exterior upper bound is -d for
                        // an outside center, with literal distance d>=0. Thus
                        // radius+reserve<=gap already implies its cap test for
                        // every possible nearest boundary. This pays the same
                        // machining gap; it does not classify the box exterior.
                        // The ordinary domain ensures the original distance is
                        // finite, retaining reference arithmetic elsewhere.
                        if (!inside && targetBoxExclusionEligible && radius + ArithmeticReserve <= gapRadius)
                        { targetExteriorCapExclusions++; skippedBoundaryDistanceQueries++; continue; }
                        // In a connected box with no original target interface,
                        // winding is constant. An outside center plus a strict
                        // padded AABB exclusion therefore proves the whole box
                        // exterior. Ambiguous contacts retain the original test.
                        if (!inside && targetBoxExclusionEligible && exteriorForGpu.ExcludesBoundary(box))
                        { targetBoxExclusions++; skippedBoundaryDistanceQueries++; continue; }
                        if (inside && radius > gapRadius && !interiorClassificationEligible)
                        {
                            // Original exteriorUpper is nonnegative here, so its
                            // sum with radius and reserve cannot be <=gap. No
                            // boundary distance can accept this box; retain the
                            // complete physical witness/subdivision authority.
                            skippedBoundaryDistanceQueries++;
                        }
                        else
                        {
                            boundaryDistance = exteriorForGpu.BoundaryDistance(q);
                            if (!inside && boundaryDistance - ArithmeticReserve > radius) continue;
                            double exteriorUpper = inside ? boundaryDistance : -boundaryDistance;
                            if (exteriorUpper + radius + ArithmeticReserve <= gapRadius) continue;
                        }
                    }
                    tasks[count] = new(task, q, radius, boundaryDistance, inside);
                    queries[count] = new(q.x, q.y, gapRadius - radius - ArithmeticReserve);
                    boxQueries[count] = new(box.X0, box.Y0, box.X1, box.Y1, gapRadius - ConvexBoxArithmeticReserve);
                    count++;
                    gpuOutstanding++;
                }
                if (count == 0) continue;
                Array.Clear(boxAccepted, 0, count);
                Array.Clear(nominations, 0, count);
                if (boxDeviceHealthy)
                {
                    if (gpuContext!.TryNominateBoxes(boxQueries, boxNominations, count, cancellationToken, out string boxReason))
                    {
                        gpuBoxQueries += count;
                        for (int i = 0; i < count; i++)
                        {
                            boxAccepted[i] = gpuContext.TryValidateBoxNomination(boxQueries[i], boxNominations[i],
                                out boxOwners[i], discCheckpoint);
                            if (boxAccepted[i]) gpuBoxWitnesses++;
                        }
                    }
                    else
                    {
                        boxDeviceHealthy = false;
                        diagnostic?.Invoke($"stage=smooth-physical-certificate event=gpu-box-fallback reason={boxReason}");
                    }
                    work.CheckNow();
                }
                int discCount = 0;
                for (int i = 0; i < count; i++)
                {
                    if (boxAccepted[i]) continue;
                    discQueries[discCount] = queries[i];
                    discOrdinals[discCount++] = i;
                }
                string batchReason = "";
                bool nominated = deviceHealthy && (discCount == 0 || gpuContext!.TryNominate(discQueries, discNominations, discCount,
                    cancellationToken, out batchReason));
                work.CheckNow();
                gpuQueries += nominated ? discCount : 0;
                if (nominated)
                    for (int i = 0; i < discCount; i++) nominations[discOrdinals[i]] = discNominations[i];
                if (!nominated && deviceHealthy)
                {
                    deviceHealthy = false;
                    gpuReason = batchReason;
                    diagnostic?.Invoke($"stage=smooth-physical-certificate event=gpu-fallback reason={gpuReason} " +
                        $"queries={gpuQueries} witnesses={gpuWitnesses}");
                }
                Array.Clear(optionalResults, 0, count);
                if (nominated && optionalDeviceHealthy)
                {
                    int optionalCount = 0;
                    for (int i = 0; i < count; i++)
                    {
                        if (boxAccepted[i] || nominations[i].HasWitness) continue;
                        optionalQueries[optionalCount] = queries[i];
                        optionalOrdinals[optionalCount++] = i;
                    }
                    if (optionalCount != 0)
                    {
                        if (optionalGpuContext!.TryNominate(optionalQueries, optionalNominations, optionalCount,
                            cancellationToken, out string optionalReason))
                        {
                            gpuOptionalQueries += optionalCount;
                            for (int i = 0; i < optionalCount; i++)
                                optionalResults[optionalOrdinals[i]] = optionalNominations[i];
                        }
                        else
                        {
                            optionalDeviceHealthy = false;
                            diagnostic?.Invoke($"stage=smooth-physical-certificate event=gpu-optional-fallback reason={optionalReason}");
                        }
                        work.CheckNow();
                    }
                }
                for (int i = 0; i < count; i++)
                {
                    work.CheckNow();
                    PendingGpuBox item = tasks[i];
                    double desired = queries[i].Desired;
                    int owner = boxOwners[i];
                    bool accepted = boxAccepted[i];
                    if (!accepted && nominated && mandatoryForGpu!.TryValidateGpuNomination(item.Center, desired,
                        nominations[i], profile, out owner))
                    { accepted = true; gpuWitnesses++; }
                    else if (!accepted && optionalDeviceHealthy && optionalForGpu!.TryValidateGpuNomination(item.Center, desired,
                        optionalResults[i], profile, out owner))
                    {
                        accepted = true;
                        gpuOptionalWitnesses++;
                    }
                    else if (!accepted && deviceHealthy && optionalForGpu is not null && !optionalDeviceHealthy)
                    {
                        gpuFallbackQueries++;
                        accepted = optionalForGpu.HasDiscWitness(item.Center, desired, profile, out owner);
                    }
                    if (!accepted && (!deviceHealthy || item.Radius <= gapRadius / 4))
                    {
                        // Small unresolved boxes and every runtime refusal retain
                        // the complete original physical search, including optional
                        // owners, work limits and cancellation checkpoints.
                        gpuFallbackQueries++;
                        accepted = mandatoryForGpu!.HasDiscWitness(item.Center, desired, profile, out owner)
                            || optionalForGpu is not null && optionalForGpu.HasDiscWitness(item.Center, desired, profile, out owner);
                    }
                    else if (!accepted) gpuSubdivisionMisses++;
                    if (accepted)
                    {
                        if (witnessedPaths is not null) witnessedPaths[owner] = true;
                        gpuOutstanding--;
                        continue;
                    }
                    if (item.Radius <= limits.MinimumBoxRadius)
                    { unresolved = item.Center; return Finish(false, "unresolved-box"); }
                    Box box = item.Task.Bounds;
                    PointD q = item.Center;
                    bool strictChildren = item.Task.StrictInterior || interiorClassificationEligible && item.Inside
                        && Math.BitDecrement(item.BoundaryDistance - item.Radius - InteriorClassificationReserve) > gapRadius;
                    if (box.X1 - box.X0 >= box.Y1 - box.Y0)
                    {
                        if (q.x == box.X0 || q.x == box.X1)
                        { unresolved = q; return Finish(false, "coordinate-resolution"); }
                        if (strictChildren && !item.Task.StrictInterior) work.StrictInteriorSeeds++;
                        pending.Push(new(new(box.X0, box.Y0, q.x, box.Y1), strictChildren));
                        pending.Push(new(new(q.x, box.Y0, box.X1, box.Y1), strictChildren));
                    }
                    else
                    {
                        if (q.y == box.Y0 || q.y == box.Y1)
                        { unresolved = q; return Finish(false, "coordinate-resolution"); }
                        if (strictChildren && !item.Task.StrictInterior) work.StrictInteriorSeeds++;
                        pending.Push(new(new(box.X0, box.Y0, box.X1, q.y), strictChildren));
                        pending.Push(new(new(box.X0, q.y, box.X1, box.Y1), strictChildren));
                    }
                    gpuOutstanding--;
                }
                if (Stopwatch.GetElapsedTime(reported).TotalSeconds >= 2)
                {
                    reported = Stopwatch.GetTimestamp();
                    diagnostic?.Invoke($"stage=smooth-physical-certificate event=progress boxes={boxes} pending={pending.Count} " +
                        $"discEvaluations={work.DiscEvaluations} gpuQueries={gpuQueries} gpuWitnesses={gpuWitnesses} " +
                        $"gpuOptionalQueries={gpuOptionalQueries} gpuOptionalWitnesses={gpuOptionalWitnesses} " +
                        $"gpuBoxQueries={gpuBoxQueries} gpuBoxWitnesses={gpuBoxWitnesses} " +
                        $"targetBoxExclusions={targetBoxExclusions} skippedBoundaryDistanceQueries={skippedBoundaryDistanceQueries} " +
                        $"targetExteriorCapExclusions={targetExteriorCapExclusions} " +
                        $"gpuFallbackQueries={gpuFallbackQueries} gpuSubdivisionMisses={gpuSubdivisionMisses} " +
                        $"gpuKernelMs={gpuContext!.KernelMilliseconds:F3} gpuTransferMs={gpuContext.TransferMilliseconds:F3} " +
                        $"elapsedSeconds={work.Elapsed:R}");
                }
            }
            return Finish(true, "whole-target-certified");
        }

        Result Finish(bool certified, string reason)
        {
            exteriorForGpu?.WindingRows?.ReportDiagnostics();
            long completed = Stopwatch.GetTimestamp();
            double queryMilliseconds = queryStarted == 0 ? 0 : Stopwatch.GetElapsedTime(queryStarted, completed).TotalMilliseconds;
            if (queryStarted == 0)
                preparationMilliseconds = Stopwatch.GetElapsedTime(preparationStarted, completed).TotalMilliseconds;
            var result = new Result(certified, reason, triangleRun?.Used == true ? triangleRun.Visited : boxes, work.DiscEvaluations,
                boundaries, intervals, triangleRun?.Used == true ? triangleRun.Remaining : pending.Count + gpuOutstanding,
                triangleRun?.Used == true ? triangleRun.Unresolved : unresolved, work.Elapsed, commandPaths.Count,
                certified && witnessedPaths is not null,
                certified && witnessedPaths is not null
                    ? Enumerable.Range(0, witnessedPaths.Length).Where(index => witnessedPaths[index]).ToArray()
                    : Array.Empty<int>())
            {
                AnalyticPoseCalls = work.AnalyticPoseCalls,
                AnalyticPoseHits = work.AnalyticPoseHits,
                AnalyticPoseFallbacks = work.AnalyticPoseFallbacks,
                AnalyticPoseEvaluations = work.AnalyticPoseEvaluations,
                AnalyticPoseDuplicates = work.AnalyticPoseDuplicates,
                AnalyticPoseInvalid = work.AnalyticPoseInvalid,
                AffineRadiusCalls = work.AffineRadiusCalls,
                AffineRadiusRejections = work.AffineRadiusRejections,
                AffineRadiusFallbacks = work.AffineRadiusFallbacks,
                AffineRadiusGuardRefusals = work.AffineRadiusGuardRefusals,
                InteriorClassificationEligible = interiorClassificationEligible,
                SkippedBoundaryQueries = work.SkippedBoundaryQueries,
                StrictInteriorSeeds = work.StrictInteriorSeeds,
                InteriorEligibilityGuardRefusals = work.InteriorEligibilityGuardRefusals,
                SphereDiscCalls = work.SphereDiscCalls,
                SphereDiscRejections = work.SphereDiscRejections,
                SphereDiscFallbacks = work.SphereDiscFallbacks,
                SphereDiscGuardRefusals = work.SphereDiscGuardRefusals,
                SpherePreparedIntervals = work.SpherePreparedIntervals,
                SpherePreparedSlots = work.SpherePreparedSlots,
                SpherePreparationGuardRefusals = work.SpherePreparationGuardRefusals,
                CachedTreeBoundReuses = work.CachedTreeBoundReuses,
                GpuAttempted = gpuAttempted, GpuAvailable = gpuAvailable, GpuReason = triangleRun?.GpuFailureReason ?? gpuReason,
                GpuQueries = gpuQueries + (triangleRun?.DiscQueries ?? 0),
                GpuWitnesses = gpuWitnesses + (triangleRun?.DiscWitnesses ?? 0),
                GpuFallbackQueries = gpuFallbackQueries + (triangleRun?.FallbackQueries ?? 0),
                GpuOptionalQueries = gpuOptionalQueries + (triangleRun?.OptionalDiscQueries ?? 0),
                GpuOptionalWitnesses = gpuOptionalWitnesses + (triangleRun?.OptionalDiscWitnesses ?? 0),
                GpuBoxQueries = gpuBoxQueries, GpuBoxWitnesses = gpuBoxWitnesses,
                ContourBoxSeedsRequested = limits.UseContourBoxSeeds, ContourBoxSeedsUsed = contourBoxSeedsUsed,
                ContourSeedBoxes = contourSeedBoxes,
                TargetBoxExclusions = targetBoxExclusions, SkippedBoundaryDistanceQueries = skippedBoundaryDistanceQueries,
                TargetExteriorCapExclusions = targetExteriorCapExclusions,
                GpuSubdivisionMisses = gpuSubdivisionMisses, GpuBatches = (gpuContext?.Batches ?? 0) + (optionalGpuContext?.Batches ?? 0),
                GpuKernelMilliseconds = (gpuContext?.KernelMilliseconds ?? 0) + (optionalGpuContext?.KernelMilliseconds ?? 0),
                GpuTransferMilliseconds = (gpuContext?.TransferMilliseconds ?? 0) + (optionalGpuContext?.TransferMilliseconds ?? 0),
                WindingRowHits = exteriorForGpu?.WindingRows?.Hits ?? 0,
                WindingRowMisses = exteriorForGpu?.WindingRows?.Misses ?? 0,
                WindingRowAmbiguousEvaluations = exteriorForGpu?.WindingRows?.AmbiguousEvaluations ?? 0,
                WindingRowRefusals = exteriorForGpu?.WindingRows?.RefusedRows ?? 0,
                CoverageDomain = triangleRun?.Used == true ? "exact-signed-triangles"
                    : contourBoxSeedsUsed ? "contour-bounds-boxes" : "boxes",
                TriangleDomainAttempted = triangleRun is not null, TriangleDomainUsed = triangleRun?.Used ?? false,
                TriangleDomainReason = triangleRun?.Reason ?? (limits.UseGpuTriangleDomain ? "gpu-unavailable" : "disabled"),
                TriangleSourceTriangles = triangleRun?.SourceTriangles ?? 0,
                TriangleCorrectionTriangles = triangleRun?.CorrectionTriangles ?? 0,
                TrianglePreparationMilliseconds = triangleRun?.PreparationMilliseconds ?? 0,
                TrianglePreparationReused = triangleRun?.PreparationReused ?? false,
                VisitedTriangles = triangleRun?.Visited ?? 0,
                GpuTriangleQueries = triangleRun?.Queries ?? 0, GpuTriangleWitnesses = triangleRun?.Witnesses ?? 0,
                GpuOptionalTriangleQueries = triangleRun?.OptionalQueries ?? 0,
                GpuOptionalTriangleWitnesses = triangleRun?.OptionalWitnesses ?? 0,
                TriangleBoundaryWitnesses = triangleRun?.BoundaryWitnesses ?? 0,
                TriangleSubdivisions = triangleRun?.Subdivisions ?? 0,
                TriangleFallbackQueries = triangleRun?.FallbackQueries ?? 0,
                TriangleBigIntegerFallbacks = triangleRun?.BigIntegerFallbacks ?? 0,
                TriangleBoundaryNominationVisits = exteriorForGpu?.BoundaryTriangleNominationVisits ?? 0,
                TriangleBoundaryNominationCandidates = exteriorForGpu?.BoundaryTriangleNominationCandidates ?? 0,
                TriangleBoundaryNominationRefusals = exteriorForGpu?.BoundaryTriangleNominationRefusals ?? 0,
                TriangleBoundaryGpuAvailable = triangleRun?.BoundaryGpuAvailable ?? false,
                TriangleBoundaryGpuReason = triangleRun?.BoundaryGpuReason ?? "disabled",
                TriangleBoundaryGpuQueries = triangleRun?.BoundaryGpuQueries ?? 0,
                TriangleBoundaryGpuWitnesses = triangleRun?.BoundaryGpuWitnesses ?? 0,
                TriangleBoundaryGpuMisses = triangleRun?.BoundaryGpuMisses ?? 0,
                TriangleBoundaryGpuRefusals = triangleRun?.BoundaryGpuRefusals ?? 0
            };
            diagnostic?.Invoke($"stage=smooth-physical-certificate event=complete certified={certified} reason={reason} " +
                $"boxes={result.VisitedBoxes} pending={result.RemainingBoxes} boundarySegments={boundaries} commandIntervals={intervals} " +
                $"unresolvedX={result.UnresolvedPoint?.x.ToString("R", System.Globalization.CultureInfo.InvariantCulture) ?? "none"} unresolvedY={result.UnresolvedPoint?.y.ToString("R", System.Globalization.CultureInfo.InvariantCulture) ?? "none"} " +
                $"mandatoryIntervals={mandatoryIntervals} optionalIntervals={optionalIntervals} " +
                $"discEvaluations={work.DiscEvaluations} elapsedSeconds={work.Elapsed:R} arithmeticReserve={ArithmeticReserve:R} " +
                $"prepareMs={preparationMilliseconds:F3} queryMs={queryMilliseconds:F3} " +
                $"contourBoxSeedsRequested={result.ContourBoxSeedsRequested} contourBoxSeedsUsed={result.ContourBoxSeedsUsed} contourSeedBoxes={result.ContourSeedBoxes} " +
                $"analyticPoseCalls={work.AnalyticPoseCalls} analyticPoseHits={work.AnalyticPoseHits} " +
                $"analyticPoseFallbacks={work.AnalyticPoseFallbacks} analyticPoseEvaluations={work.AnalyticPoseEvaluations} " +
                $"analyticPoseDuplicates={work.AnalyticPoseDuplicates} analyticPoseInvalid={work.AnalyticPoseInvalid} " +
                $"affineRadiusCalls={work.AffineRadiusCalls} affineRadiusRejections={work.AffineRadiusRejections} " +
                $"affineRadiusFallbacks={work.AffineRadiusFallbacks} affineRadiusGuardRefusals={work.AffineRadiusGuardRefusals} " +
                $"interiorClassificationEligible={interiorClassificationEligible} skippedBoundaryQueries={work.SkippedBoundaryQueries} " +
                $"strictInteriorSeeds={work.StrictInteriorSeeds} interiorEligibilityGuardRefusals={work.InteriorEligibilityGuardRefusals} " +
                $"sphereDiscCalls={work.SphereDiscCalls} sphereDiscRejections={work.SphereDiscRejections} " +
                $"sphereDiscFallbacks={work.SphereDiscFallbacks} sphereDiscGuardRefusals={work.SphereDiscGuardRefusals} " +
                $"spherePreparedIntervals={work.SpherePreparedIntervals} spherePreparedSlots={work.SpherePreparedSlots} " +
                $"spherePreparationGuardRefusals={work.SpherePreparationGuardRefusals} cachedTreeBoundReuses={work.CachedTreeBoundReuses} " +
                $"gpuAttempted={gpuAttempted} gpuAvailable={gpuAvailable} gpuReason={result.GpuReason} gpuQueries={result.GpuQueries} " +
                $"gpuWitnesses={result.GpuWitnesses} gpuFallbackQueries={result.GpuFallbackQueries} gpuSubdivisionMisses={gpuSubdivisionMisses} " +
                $"gpuOptionalQueries={gpuOptionalQueries} gpuOptionalWitnesses={gpuOptionalWitnesses} " +
                $"gpuBoxQueries={gpuBoxQueries} gpuBoxWitnesses={gpuBoxWitnesses} " +
                $"targetBoxExclusions={targetBoxExclusions} skippedBoundaryDistanceQueries={skippedBoundaryDistanceQueries} " +
                $"targetExteriorCapExclusions={targetExteriorCapExclusions} " +
                $"gpuBatches={result.GpuBatches} gpuKernelMs={result.GpuKernelMilliseconds:F3} gpuTransferMs={result.GpuTransferMilliseconds:F3} " +
                $"windingRowHits={result.WindingRowHits} windingRowMisses={result.WindingRowMisses} " +
                $"windingRowAmbiguous={result.WindingRowAmbiguousEvaluations} windingRowRefusals={result.WindingRowRefusals} " +
                $"witnessPathsRecorded={result.WitnessPathsRecorded} witnessPaths={result.WitnessPathIndices.Count}");
            diagnostic?.Invoke($"stage=smooth-physical-certificate event=domain-complete domain={result.CoverageDomain} " +
                $"triangleAttempted={result.TriangleDomainAttempted} triangleUsed={result.TriangleDomainUsed} triangleReason={result.TriangleDomainReason} " +
                $"sourceTriangles={result.TriangleSourceTriangles} correctionTriangles={result.TriangleCorrectionTriangles} " +
                $"trianglePrepareMs={result.TrianglePreparationMilliseconds:F3} trianglePreparationReused={result.TrianglePreparationReused} " +
                $"triangleDelaunay={limits.UseDelaunayTriangleDomain} visitedTriangles={result.VisitedTriangles} " +
                $"trianglePending={result.RemainingBoxes} triangleQueries={result.GpuTriangleQueries} triangleWitnesses={result.GpuTriangleWitnesses} " +
                $"optionalTriangleQueries={result.GpuOptionalTriangleQueries} optionalTriangleWitnesses={result.GpuOptionalTriangleWitnesses} " +
                $"boundaryTriangleWitnesses={result.TriangleBoundaryWitnesses} triangleSubdivisions={result.TriangleSubdivisions} " +
                $"triangleFallbackQueries={result.TriangleFallbackQueries} triangleBigIntegerFallbacks={result.TriangleBigIntegerFallbacks} " +
                $"boundaryTriangleNominationVisits={result.TriangleBoundaryNominationVisits} " +
                $"boundaryTriangleNominationCandidates={result.TriangleBoundaryNominationCandidates} " +
                $"boundaryTriangleNominationRefusals={result.TriangleBoundaryNominationRefusals} " +
                $"boundaryGpuAvailable={result.TriangleBoundaryGpuAvailable} boundaryGpuReason={result.TriangleBoundaryGpuReason} " +
                $"boundaryGpuQueries={result.TriangleBoundaryGpuQueries} boundaryGpuWitnesses={result.TriangleBoundaryGpuWitnesses} " +
                $"boundaryGpuMisses={result.TriangleBoundaryGpuMisses} boundaryGpuRefusals={result.TriangleBoundaryGpuRefusals}");
            return result;
        }
    }

    private static bool Finite(Point3D p) => double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z);
    private static bool Ordinary(double value) => double.IsFinite(value) && Math.Abs(value) <= 10_000;
    private static double Square(double x) => x * x;
    private static Point3D At(Point3D a, Point3D b, double t)
        => new(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);
    private readonly record struct Box(double X0, double Y0, double X1, double Y1);
    private readonly record struct PendingBox(Box Bounds, bool StrictInterior);
    private readonly record struct PendingGpuBox(PendingBox Task, PointD Center, double Radius,
        double BoundaryDistance, bool Inside);
    private readonly record struct Segment(Point3D A, Point3D B, double MaximumRadius, int SourcePathIndex);
    private sealed class Exhausted : Exception { internal Exhausted(string reason) : base(reason) { } }

    private sealed class Budget
    {
        private readonly Limits _limits;
        private readonly CancellationToken _cancellation;
        private readonly long _started = Stopwatch.GetTimestamp();
        private long _operations, _input;
        internal long DiscEvaluations { get; private set; }
        internal bool UseAnalyticPoseNomination => _limits.UseAnalyticPoseNomination;
        internal bool UseAffineRadiusPruning => _limits.UseAffineRadiusPruning;
        internal long AnalyticPoseCalls, AnalyticPoseHits, AnalyticPoseFallbacks, AnalyticPoseEvaluations;
        internal long AnalyticPoseDuplicates, AnalyticPoseInvalid;
        internal long AffineRadiusCalls, AffineRadiusRejections, AffineRadiusFallbacks, AffineRadiusGuardRefusals;
        internal long SkippedBoundaryQueries, StrictInteriorSeeds, InteriorEligibilityGuardRefusals;
        internal bool UseSphereDiscPruning => _limits.UseSphereDiscPruning;
        internal bool UsePreparedSphereDiscPruning => _limits.UsePreparedSphereDiscPruning;
        internal bool UseCachedTreeBounds => _limits.UseCachedTreeBounds || _limits.UseGpuDiscWitnesses;
        internal bool UseCachedWindingRows => _limits.UseCachedWindingRows || _limits.UseGpuDiscWitnesses;
        internal CancellationToken Cancellation => _cancellation;
        internal Action<string>? Diagnostic { get; init; }
        internal long SphereDiscCalls, SphereDiscRejections, SphereDiscFallbacks, SphereDiscGuardRefusals;
        internal long SpherePreparedIntervals, SpherePreparedSlots, SpherePreparationGuardRefusals, CachedTreeBoundReuses;
        internal double Elapsed => Stopwatch.GetElapsedTime(_started).TotalSeconds;
        internal Budget(Limits limits, CancellationToken cancellation) { _limits = limits; _cancellation = cancellation; }
        internal void Check() { if ((++_operations & 255) == 0) CheckNow(); }
        internal void CheckNow()
        {
            _cancellation.ThrowIfCancellationRequested();
            if (Stopwatch.GetElapsedTime(_started) >= _limits.MaximumElapsed) throw new Exhausted("time-budget");
        }
        internal void Input()
        {
            Check();
            if (++_input > _limits.MaximumInputSegments) throw new Exhausted("input-budget");
        }
        internal void Piece(int count)
        {
            Check();
            if (count >= _limits.MaximumInputSegments) throw new Exhausted("interval-budget");
        }
        internal void Disc()
        {
            Check();
            if (DiscEvaluations >= _limits.MaximumDiscEvaluations) throw new Exhausted("disc-budget");
            DiscEvaluations++;
        }
    }

    /// <summary>Median-partition BVH shared by exact boundary witnesses, winding
    /// queries and one-sided commanded-disc searches. No polygon tessellation.</summary>
    private sealed partial class Tree
    {
        private readonly struct Node
        {
            internal readonly Box Bounds;
            internal readonly double MaximumRadius;
            internal readonly int First, Count, Left, Right;
            internal Node(Box bounds, double radius, int first, int count, int left, int right)
            { Bounds = bounds; MaximumRadius = radius; First = first; Count = count; Left = left; Right = right; }
        }
        private readonly Segment[] _segments;
        private readonly List<Node> _nodes = new();
        private readonly Budget _work;
        internal PreparedTargetWindingRows? WindingRows { get; private set; }
        private VCarveCutterProfile? _preparedSphereOwner;
        private VCarvePreparedSphereDiscBounds.PreparedProfile _preparedSphereProfile;
        private int[] _preparedSphereOrdinals = Array.Empty<int>();
        private VCarvePreparedSphereDiscBounds.PreparedInterval[] _preparedSphereIntervals
            = Array.Empty<VCarvePreparedSphereDiscBounds.PreparedInterval>();
        internal Box Bounds => _nodes[0].Bounds;
        internal Tree(Segment[] segments, Budget work) { _segments = segments; _work = work; Build(0, segments.Length); }

        internal bool TryCreateGpuContext(VCarveCutterProfile profile, CancellationToken cancellationToken,
            Action<string>? diagnostic, out VCarveGpuDiscWitnessContext? context, out string reason,
            VCarveGenerationPolicy? policy = null)
        {
            var segments = new GpuDiscSegment[_segments.Length];
            for (int i = 0; i < segments.Length; i++)
            {
                if ((i & 1023) == 0) _work.CheckNow();
                Segment s = _segments[i];
                segments[i] = new(s.A.X, s.A.Y, s.A.Z, s.B.X, s.B.Y, s.B.Z, s.MaximumRadius, s.SourcePathIndex);
            }
            var nodes = new GpuDiscNode[_nodes.Count];
            for (int i = 0; i < nodes.Length; i++)
            {
                if ((i & 1023) == 0) _work.CheckNow();
                Node n = _nodes[i];
                nodes[i] = new(n.Bounds.X0, n.Bounds.Y0, n.Bounds.X1, n.Bounds.Y1,
                    n.MaximumRadius, n.First, n.Count, n.Left, n.Right);
            }
            return VCarveGpuDiscWitnessContext.TryCreate(segments, nodes, profile, cancellationToken,
                diagnostic, out context, out reason, policy);
        }

        internal bool TryValidateGpuNomination(PointD q, double desired, GpuDiscNomination nomination,
            VCarveCutterProfile profile, out int witnessPathIndex)
        {
            witnessPathIndex = -1;
            if (!nomination.HasWitness || (uint)nomination.SegmentOrdinal >= (uint)_segments.Length
                || !double.IsFinite(nomination.Parameter) || nomination.Parameter < 0 || nomination.Parameter > 1)
                return false;
            Segment s = _segments[nomination.SegmentOrdinal];
            _work.Disc();
            Point3D pose = At(s.A, s.B, nomination.Parameter);
            if (pose.Z <= 0) return false;
            double value = Math.Sqrt(Square(pose.X - q.x) + Square(pose.Y - q.y))
                - profile.PhysicalRadiusAtDepth(pose.Z);
            if (!(value <= desired)) return false;
            witnessPathIndex = s.SourcePathIndex;
            return true;
        }

        private int Build(int first, int count)
        {
            _work.CheckNow();
            double x0 = double.PositiveInfinity, y0 = x0, x1 = double.NegativeInfinity, y1 = x1, radius = 0;
            for (int i = first; i < first + count; i++)
            {
                _work.Check();
                Segment s = _segments[i];
                x0 = Math.Min(x0, Math.Min(s.A.X, s.B.X)); y0 = Math.Min(y0, Math.Min(s.A.Y, s.B.Y));
                x1 = Math.Max(x1, Math.Max(s.A.X, s.B.X)); y1 = Math.Max(y1, Math.Max(s.A.Y, s.B.Y));
                radius = Math.Max(radius, s.MaximumRadius);
            }
            var bounds = new Box(x0, y0, x1, y1);
            int index = _nodes.Count;
            _nodes.Add(default);
            int left = -1, right = -1;
            if (count > 8)
            {
                int half = count / 2;
                Partition(first, first + count - 1, first + half, x1 - x0 >= y1 - y0);
                left = Build(first, half); right = Build(first + half, count - half);
            }
            _nodes[index] = new(bounds, radius, first, count, left, right);
            return index;
        }

        private void Partition(int lo, int hi, int middle, bool xAxis)
        {
            double Key(Segment s) => xAxis ? s.A.X / 2 + s.B.X / 2 : s.A.Y / 2 + s.B.Y / 2;
            while (lo < hi)
            {
                double pivot = Key(_segments[lo + (hi - lo) / 2]);
                int i = lo, j = hi;
                while (i <= j)
                {
                    _work.Check();
                    while (Key(_segments[i]) < pivot) { i++; _work.Check(); }
                    while (Key(_segments[j]) > pivot) { j--; _work.Check(); }
                    if (i <= j) { (_segments[i], _segments[j]) = (_segments[j], _segments[i]); i++; j--; }
                }
                if (middle <= j) hi = j;
                else if (middle >= i) lo = i;
                else break;
            }
        }

        private static double BoxDistance(PointD q, Box b)
            => Math.Sqrt(Square(Math.Max(0, Math.Max(b.X0 - q.x, q.x - b.X1)))
                + Square(Math.Max(0, Math.Max(b.Y0 - q.y, q.y - b.Y1))));
        private static double Project(PointD q, Segment s)
        {
            double dx = s.B.X - s.A.X, dy = s.B.Y - s.A.Y, square = dx * dx + dy * dy;
            return square > 0 ? Math.Clamp(((q.x - s.A.X) * dx + (q.y - s.A.Y) * dy) / square, 0, 1) : 0;
        }
        private static double Distance(PointD q, Segment s)
        {
            Point3D p = At(s.A, s.B, Project(q, s));
            return Math.Sqrt(Square(p.X - q.x) + Square(p.Y - q.y));
        }

        private void PrepareSphereIntervals(VCarveCutterProfile profile)
        {
            // Build after the immutable BVH has completed its segment partition.
            // The separate ordinal map preserves Segment's size and constructor.
            int slots = 0;
            for (int i = 0; i < _segments.Length; i++)
            {
                if ((i & 1023) == 0) _work.CheckNow();
                if (_segments[i].A.Z != _segments[i].B.Z) slots++;
            }
            if (!VCarvePreparedSphereDiscBounds.TryPrepareProfile(profile, out var preparedProfile))
            {
                _preparedSphereOrdinals = Array.Empty<int>();
                _preparedSphereIntervals = Array.Empty<VCarvePreparedSphereDiscBounds.PreparedInterval>();
                _preparedSphereOwner = profile;
                _work.SpherePreparationGuardRefusals += slots;
                return;
            }
            var ordinals = slots == 0 ? Array.Empty<int>() : new int[_segments.Length];
            var intervals = new VCarvePreparedSphereDiscBounds.PreparedInterval[slots];
            int preparedCount = 0;
            for (int i = 0; i < _segments.Length; i++)
            {
                if ((i & 1023) == 0) _work.CheckNow();
                Segment s = _segments[i];
                if (s.A.Z == s.B.Z || !VCarvePreparedSphereDiscBounds.TryPrepareInterval(s.A, s.B,
                    in preparedProfile, out var interval)) continue;
                intervals[preparedCount] = interval;
                ordinals[i] = ++preparedCount;
            }
            _preparedSphereProfile = preparedProfile;
            _preparedSphereOrdinals = ordinals;
            _preparedSphereIntervals = intervals;
            _preparedSphereOwner = profile;
            _work.SpherePreparedIntervals += preparedCount;
            _work.SpherePreparedSlots += slots;
            _work.SpherePreparationGuardRefusals += slots - preparedCount;
        }

        internal double BoundaryDistance(PointD q)
        {
            double best = double.PositiveInfinity;
            Visit(0, 0, false);
            return best;
            void Visit(int index, double cachedDistance, bool hasCachedDistance)
            {
                _work.Check();
                Node node = _nodes[index];
                double distance = hasCachedDistance ? cachedDistance : BoxDistance(q, node.Bounds);
                if (hasCachedDistance) _work.CachedTreeBoundReuses++;
                if (distance - ArithmeticReserve > best) return;
                if (node.Left < 0)
                {
                    for (int i = node.First; i < node.First + node.Count; i++) best = Math.Min(best, Distance(q, _segments[i]));
                    return;
                }
                double leftDistance = BoxDistance(q, _nodes[node.Left].Bounds);
                double rightDistance = BoxDistance(q, _nodes[node.Right].Bounds);
                bool reuse = _work.UseCachedTreeBounds;
                if (leftDistance <= rightDistance)
                { Visit(node.Left, leftDistance, reuse); Visit(node.Right, rightDistance, reuse); }
                else { Visit(node.Right, rightDistance, reuse); Visit(node.Left, leftDistance, reuse); }
            }
        }

        internal bool ExcludesBoundary(Box box)
        {
            const double reserve = 1e-8;
            var padded = new Box(Math.BitDecrement(box.X0 - reserve), Math.BitDecrement(box.Y0 - reserve),
                Math.BitIncrement(box.X1 + reserve), Math.BitIncrement(box.Y1 + reserve));
            return !Touches(0);

            bool Disjoint(Box bounds) => bounds.X1 < padded.X0 || bounds.X0 > padded.X1
                || bounds.Y1 < padded.Y0 || bounds.Y0 > padded.Y1;
            bool Touches(int index)
            {
                _work.Check();
                Node node = _nodes[index];
                if (Disjoint(node.Bounds)) return false;
                if (node.Left >= 0) return Touches(node.Left) || Touches(node.Right);
                for (int i = node.First; i < node.First + node.Count; i++)
                {
                    Segment s = _segments[i];
                    if (!Disjoint(new(Math.Min(s.A.X, s.B.X), Math.Min(s.A.Y, s.B.Y),
                        Math.Max(s.A.X, s.B.X), Math.Max(s.A.Y, s.B.Y)))) return true;
                }
                return false;
            }
        }

        internal bool IsInside(PointD q)
        {
            if (_work.UseCachedWindingRows)
            {
                if (WindingRows is null)
                {
                    var segments = new TargetWindingSegment[_segments.Length];
                    for (int i = 0; i < segments.Length; i++)
                    {
                        if ((i & 1023) == 0) _work.CheckNow();
                        Segment s = _segments[i];
                        segments[i] = new(s.A.X, s.A.Y, s.B.X, s.B.Y);
                    }
                    WindingRows = new(segments, _work.Cancellation, _work.Diagnostic, workCheckpoint: _work.Check);
                }
                if (WindingRows.TryIsInside(q, out bool cachedInside)) return cachedInside;
            }
            int winding = 0;
            Visit(0);
            return winding != 0;
            void Visit(int index)
            {
                _work.Check();
                Node node = _nodes[index];
                if (q.y < node.Bounds.Y0 || q.y >= node.Bounds.Y1 || q.x > node.Bounds.X1) return;
                if (node.Left >= 0) { Visit(node.Left); Visit(node.Right); return; }
                for (int i = node.First; i < node.First + node.Count; i++)
                {
                    Segment s = _segments[i];
                    double cross = (s.B.X - s.A.X) * (q.y - s.A.Y) - (q.x - s.A.X) * (s.B.Y - s.A.Y);
                    if (s.A.Y <= q.y && s.B.Y > q.y && cross > 0) winding++;
                    else if (s.A.Y > q.y && s.B.Y <= q.y && cross < 0) winding--;
                }
            }
        }

        internal bool HasDiscWitness(PointD q, double desired, VCarveCutterProfile profile, out int witnessPathIndex)
        {
            if (_work.UsePreparedSphereDiscPruning && !ReferenceEquals(_preparedSphereOwner, profile))
                PrepareSphereIntervals(profile);
            int selectedPath = -1;
            bool accepted = Visit(0, 0, false);
            witnessPathIndex = selectedPath;
            return accepted;
            double Lower(Node node) => BoxDistance(q, node.Bounds) - node.MaximumRadius - ArithmeticReserve;
            bool Visit(int index, double cachedLower, bool hasCachedLower)
            {
                _work.Check();
                Node node = _nodes[index];
                double lower = hasCachedLower ? cachedLower : Lower(node);
                if (hasCachedLower) _work.CachedTreeBoundReuses++;
                if (lower > desired) return false;
                if (node.Left >= 0)
                {
                    int first = node.Left, last = node.Right;
                    double firstLower = Lower(_nodes[first]), lastLower = Lower(_nodes[last]);
                    if (firstLower > lastLower) { (first, last) = (last, first); (firstLower, lastLower) = (lastLower, firstLower); }
                    bool reuse = _work.UseCachedTreeBounds;
                    return Visit(first, firstLower, reuse) || Visit(last, lastLower, reuse);
                }
                for (int i = node.First; i < node.First + node.Count; i++)
                {
                    Segment s = _segments[i];
                    if (Distance(q, s) - s.MaximumRadius - ArithmeticReserve > desired) continue;
                    if (Witness(s, i)) return true;
                }
                return false;
            }
            bool Witness(Segment s, int ordinal)
            {
                bool Accept(double value)
                {
                    if (!(value <= desired)) return false;
                    selectedPath = s.SourcePathIndex;
                    return true;
                }
                double Value(double t)
                {
                    _work.Disc();
                    Point3D p = At(s.A, s.B, t);
                    // A flat-bottom cutter at the stock surface has positive
                    // radius but is wholly unengaged. It cannot certify a cut.
                    if (p.Z <= 0) return double.PositiveInfinity;
                    return Math.Sqrt(Square(p.X - q.x) + Square(p.Y - q.y)) - profile.PhysicalRadiusAtDepth(p.Z);
                }
                double best = Math.Min(Value(0), Value(1));
                double projected = Project(q, s);
                best = Math.Min(best, Value(projected));
                if (Accept(best)) return true;
                if (s.A.Z == s.B.Z) return false;
                if (_work.UsePreparedSphereDiscPruning)
                {
                    _work.SphereDiscCalls++;
                    int preparedOrdinal = _preparedSphereOrdinals.Length == 0 ? -1 : _preparedSphereOrdinals[ordinal] - 1;
                    bool rejected = false;
                    bool bounded = preparedOrdinal >= 0 && VCarvePreparedSphereDiscBounds.TryReject(q, s.A,
                        in _preparedSphereIntervals[preparedOrdinal], _preparedSphereProfile.SphereSquaredUpper, desired, out rejected);
                    if (bounded && rejected) { _work.SphereDiscRejections++; return false; }
                    if (!bounded) _work.SphereDiscGuardRefusals++;
                    _work.SphereDiscFallbacks++;
                }
                else if (_work.UseSphereDiscPruning)
                {
                    _work.SphereDiscCalls++;
                    bool bounded = VCarveSphereDiscBounds.TryBounds(q, s.A, s.B, profile, desired,
                        out double lower, out double upper);
                    if (bounded && lower > upper) { _work.SphereDiscRejections++; return false; }
                    if (!bounded) _work.SphereDiscGuardRefusals++;
                    _work.SphereDiscFallbacks++;
                }
                if (_work.UseAffineRadiusPruning)
                {
                    _work.AffineRadiusCalls++;
                    double lower = double.NegativeInfinity;
                    bool bounded = double.IsFinite(desired) && VCarveAffineRadiusBounds.TryLowerBound(q, s.A, s.B, profile, out lower);
                    if (bounded && lower > desired) { _work.AffineRadiusRejections++; return false; }
                    if (!bounded) _work.AffineRadiusGuardRefusals++;
                    _work.AffineRadiusFallbacks++;
                }
                if (_work.UseAnalyticPoseNomination)
                {
                    _work.AnalyticPoseCalls++;
                    // The contact kernel uses tip Z; this tree stores positive
                    // penetration. Its square reserve only nominates a pose.
                    // Acceptance still uses the exact, unreserved disc below.
                    double pose = double.NaN;
                    if (double.IsFinite(q.x) && double.IsFinite(q.y) && double.IsFinite(desired)
                        && Finite(s.A) && Finite(s.B))
                        CarvingCutterContactBounds.MinimumTipAndBottom(profile.PhysicalProfile,
                            q, new PointD(s.A.X, s.A.Y), -s.A.Z,
                            new PointD(s.B.X, s.B.Y), -s.B.Z, Math.Max(0, desired), out pose);
                    if (!double.IsFinite(pose) || pose < 0 || pose > 1)
                        _work.AnalyticPoseInvalid++;
                    else if (pose == 0 || pose == 1 || pose == projected)
                        _work.AnalyticPoseDuplicates++;
                    else
                    {
                        // Value charges the same disc quota and clock/cancellation
                        // checkpoints as every original search sample.
                        double value = Value(pose);
                        _work.AnalyticPoseEvaluations++;
                        if (Accept(value)) { _work.AnalyticPoseHits++; return true; }
                    }
                    _work.AnalyticPoseFallbacks++;
                    // Do not seed best or the search interval from the nominee.
                    // A failed nomination follows the literal original search.
                }
                // Search samples are real poses, hence valid upper witnesses.
                // Neither convexity nor optimizer convergence is assumed by the
                // box proof. Missing a better pose merely makes it inconclusive.
                double lo = 0, hi = 1;
                const double ratio = .6180339887498949;
                double t0 = hi - (hi - lo) * ratio, t1 = lo + (hi - lo) * ratio;
                double v0 = Value(t0), v1 = Value(t1);
                for (int step = 0; step < 40; step++)
                {
                    best = Math.Min(best, Math.Min(v0, v1));
                    if (Accept(best)) return true;
                    if (v0 <= v1) { hi = t1; t1 = t0; v1 = v0; t0 = hi - (hi - lo) * ratio; v0 = Value(t0); }
                    else { lo = t0; t0 = t1; v0 = v1; t1 = lo + (hi - lo) * ratio; v1 = Value(t1); }
                }
                return Accept(Math.Min(best, Math.Min(v0, v1)));
            }
        }
    }
}
