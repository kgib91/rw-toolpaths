using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Source subtraction and final full-footprint proof for coordinated roughing.</summary>
internal static class PocketProtection
{
    // Shared emits float coordinates and F3 commands. This radial reserve covers
    // both planar coordinate roundings without changing authored edge allowance.
    private const double CommandReserve = .001;

    internal sealed record ProtectionDomain(IReadOnlyList<List<PointD>> Paths,
        double RequiredGuard, double AppliedOffset, double EffectiveArcTolerance,
        double MaskArcTolerance, int ProofChords, int ExpansionAttempts);

    /// <summary>The exact signed exclusion used by physical concentric fills.</summary>
    internal static ProtectionDomain PrepareConcentricMask(PocketOptions options,
        CancellationToken cancellationToken, bool? denseFirstArcExperiment = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        if (!double.IsFinite(options.StockToLeave))
            throw new ArgumentOutOfRangeException(nameof(options), "Pocket stock allowance must be finite.");
        var protection = options.ProtectedBoundary ?? Array.Empty<IReadOnlyList<PointD>>();
        // The physical first front is built from ALL original signed boundaries
        // at native-tick arc fidelity. Later fronts' coarser construction still
        // proves complete original-wall chord clearance at its requested inset;
        // its .001 arc budget is not consumed as a wall penetration allowance.
        // Reserve the literal first construction here rather than excluding an
        // extra .001 at every protected wall. Dense-first experimental commands
        // reserve their literal opt-in arc bound separately.
        const double quantum = 1 / PathUtils.Scale;
        double allLevelArc = Math.Min(options.Tolerances.ArcTolerance, .001);
        if (!double.IsFinite(allLevelArc) || allLevelArc < 0)
            throw new ArgumentOutOfRangeException(nameof(options), "Pocket arc tolerance must be finite and nonnegative.");
        bool denseFirstArc = denseFirstArcExperiment
            ?? (Environment.GetEnvironmentVariable("RW_TOOLPATHS_FIRST_ARC_EXPERIMENT") == "1"
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1");
        double actualArc = denseFirstArc
            ? Math.Min(allLevelArc, RingOffsetEngine.DenseFirstArcExperimentLimit)
            : Math.Min(allLevelArc, quantum);
        double safetyReserve = CommandReserve + actualArc + 4 * quantum;
        double positiveLeave = Math.Max(0, options.StockToLeave);
        double negativeLeaveCancellation = Math.Max(0, -options.StockToLeave);
        bool reserveReference = Environment.GetEnvironmentVariable("RW_TOOLPATHS_PROTECTION_RESERVE_REFERENCE") == "1";
        // The unchanged first inset and every original-source chord proof also
        // reserve positive stock allowance at these protected walls. Spend that
        // allowance once: guard + leave still covers the complete posting and
        // literal first-construction reserve. Negative allowance is cancelled
        // at protection, including when the requested centre inset is clamped
        // to zero. The raw signed-domain/capsule proof below remains mandatory.
        double positiveLeaveCredit = reserveReference ? 0 : Math.Min(safetyReserve, positiveLeave);
        double guard = (reserveReference ? safetyReserve : Math.Max(0, safetyReserve - positiveLeave))
            + negativeLeaveCancellation;
        var raw = Clipper.Union(PathUtils.ToClipper(protection), FillRule.NonZero);
        if (raw.Count == 0)
            return new ProtectionDomain(Array.Empty<List<PointD>>(), guard, 0, actualArc, quantum, 0, 0);
        // No simplification of this signed domain is permitted. The initial
        // offset compensates inscribed round joins and integer arithmetic, but
        // those estimates alone never authorize the mask. Certify the ENTIRE
        // original filled domain and every original boundary's full guard-radius
        // capsule. Their union is precisely the closed domain's outer dilation,
        // including hole edges, islands, narrow necks and stationary vertices.
        // ContainsCapsule adds its own four-tick query/boundary reserve.
        double applied = guard + quantum + 8 * quantum;
        int proofChords = 0;
        double maximumOffset = Math.Max(0, -options.StockToLeave) + .003 + 4 * quantum;
        for (int attempt = 0; attempt < 5 && applied <= maximumOffset; attempt++, applied += 8 * quantum)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var offsetter = new ClipperOffset(2, quantum * PathUtils.Scale);
            offsetter.AddPaths(raw, JoinType.Round, EndType.Polygon);
            var expanded = new Paths64();
            offsetter.Execute(applied * PathUtils.Scale, expanded);
            expanded = Clipper.Union(expanded, FillRule.NonZero);
            bool containsRaw = Clipper.Difference(raw, expanded, FillRule.NonZero).Count == 0;
            var index = new PolygonOps.BoundarySegmentIndex(expanded);
            bool complete = containsRaw;
            int audited = 0;
            foreach (var contour in raw)
            {
                for (int end = 0, start = contour.Count - 1; end < contour.Count; start = end++)
                {
                    if ((audited++ & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                    if (index.ContainsCapsule(contour[start], contour[end], guard * PathUtils.Scale)) continue;
                    complete = false;
                    break;
                }
                if (!complete) break;
            }
            proofChords += audited;
            Console.WriteLine($"[Pocket.Protection] stage=outer-mask-proof attempt={attempt + 1} "
                + $"rawContours={raw.Count} rawVertices={raw.Sum(path => (long)path.Count)} "
                + $"expandedContours={expanded.Count} containsRaw={containsRaw} completeCapsules={complete} "
                + $"proofChords={audited} requestedArc={options.Tolerances.ArcTolerance:R} "
                + $"requestedSimplify={options.Tolerances.SimplifyTolerance:R} effectiveArc={actualArc:R} "
                + $"allLevelConstructionArc={allLevelArc:R} denseFirstArcExperiment={denseFirstArc} "
                + $"effectiveSimplify=0 commandReserve={CommandReserve:R} quantizationGuard={4 * quantum:R} "
                + $"safetyReserve={safetyReserve:R} positiveLeaveCredit={positiveLeaveCredit:R} "
                + $"negativeLeaveCancellation={negativeLeaveCancellation:R} reserveReference={reserveReference} "
                + $"requiredGuard={guard:R} appliedOffset={applied:R} maskArc={quantum:R} maskSimplification=0 "
                + "guardFidelity=original-first-command laterArcWallCredit=0 rawAuthority=complete-signed-domain-and-boundary-capsules");
            if (complete)
                return new ProtectionDomain(PathUtils.FromClipper(expanded), guard, applied, actualArc,
                    quantum, proofChords, attempt + 1);
        }
        throw new InvalidOperationException("Concentric roughing protection could not certify the complete original signed mask expansion.");
    }

    internal static IReadOnlyList<MillingRegion> Subtract(IReadOnlyList<MillingRegion> regions,
        PocketOptions options, CancellationToken cancellationToken, bool physicalConcentric = false)
    {
        if (options.ProtectedBoundary is not { Count: > 0 } protection) return regions;
        cancellationToken.ThrowIfCancellationRequested();
        if (physicalConcentric)
        {
            // Capture the diagnostic policy before any physical mask geometry.
            // Generic scanline and Carving keep their existing subtraction path.
            bool componentReference = Environment.GetEnvironmentVariable("RW_TOOLPATHS_PROTECTION_COMPONENT_REFERENCE") == "1";
            var domain = PrepareConcentricMask(options, cancellationToken, options.DenseFirstArcConstruction);
            var result = componentReference
                ? SubtractDomain(regions, domain.Paths, cancellationToken)
                : SubtractPreparedDomain(regions, domain.Paths, cancellationToken);
            Console.WriteLine($"[Pocket.Protection] stage=source-subtraction originalRegions={regions.Count} "
                + $"retainedRegions={result.Count} protectedContours={protection.Count} "
                + $"protectedArea={Math.Abs(Clipper.Area(new PathsD(protection.Select(ring => new PathD(ring))))):R} "
                + $"expandedProtectedArea={Math.Abs(Clipper.Area(new PathsD(domain.Paths.Select(ring => new PathD(ring))))):R} "
                + $"geometryReserve={domain.EffectiveArcTolerance:R} commandReserve={CommandReserve:R} "
                + $"requiredGuard={domain.RequiredGuard:R} appliedOffset={domain.AppliedOffset:R} "
                + $"maskArcError={domain.MaskArcTolerance:R} maskSimplification=0 "
                + $"proofChords={domain.ProofChords} expansionAttempts={domain.ExpansionAttempts} "
                + $"componentReference={componentReference} "
                + $"authoredEdgeOffset={options.StockToLeave:R} rawSignedContainment=True completeBoundaryCapsules=True");
            return result;
        }
        // A negative outer-wall allowance is still honored at the original artwork.
        // Cancel it at protection only, so it cannot erode material handed to Detail.
        double geometryReserve = Math.Max(0, options.Tolerances.ArcTolerance)
            + Math.Max(0, options.Tolerances.SimplifyTolerance);
        const double maskSimplification = .001;
        const double maskArcError = .001;
        double reserve = CommandReserve + geometryReserve + Math.Max(0, -options.StockToLeave)
            + maskSimplification + maskArcError;
        // This mask is an exclusion, not an emitted cutter-centre contour. Its
        // outward expansion only needs to contain the original mask. Bound both
        // simplification and round-join chord error explicitly, then prove all
        // emitted physical footprints against the unsimplified original mask.
        var simplified = Clipper.SimplifyPaths(PathUtils.ToClipper(protection),
            maskSimplification * PathUtils.Scale);
        var offsetter = new ClipperOffset(2, maskArcError * PathUtils.Scale);
        offsetter.AddPaths(simplified, JoinType.Round, EndType.Polygon);
        var expandedPaths = new Paths64();
        offsetter.Execute(reserve * PathUtils.Scale, expandedPaths);
        var expanded = PathUtils.FromClipper(expandedPaths);
        var retained = SubtractDomain(regions, expanded, cancellationToken);
        Console.WriteLine($"[Pocket.Protection] stage=source-subtraction originalRegions={regions.Count} "
            + $"retainedRegions={retained.Count} protectedContours={protection.Count} "
            + $"protectedArea={Math.Abs(Clipper.Area(new PathsD(protection.Select(ring => new PathD(ring))))):R} "
            + $"commandReserve={CommandReserve:R} geometryReserve={geometryReserve:R} "
            + $"maskSimplification={maskSimplification:R} maskArcError={maskArcError:R} "
            + $"authoredEdgeOffset={options.StockToLeave:R}");
        return retained;
    }

    private static IReadOnlyList<MillingRegion> SubtractDomain(IReadOnlyList<MillingRegion> regions,
        IEnumerable<IReadOnlyList<PointD>> expanded, CancellationToken cancellationToken)
    {
        var retained = new List<MillingRegion>();
        foreach (var region in regions)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var source = PolygonOps.Difference(new[] { region.Outer }, region.Holes);
            retained.AddRange(PolygonOps.DifferenceToComponents(source, expanded)
                .Select(component => new MillingRegion(component.Outer,
                    component.Holes.Select(hole => (IReadOnlyList<PointD>)hole).ToArray())));
        }
        return retained;
    }

    private static IReadOnlyList<MillingRegion> SubtractPreparedDomain(IReadOnlyList<MillingRegion> regions,
        IReadOnlyList<List<PointD>> expanded, CancellationToken cancellationToken)
    {
        long started = Stopwatch.GetTimestamp();
        long allocated = GC.GetAllocatedBytesForCurrentThread();
        long preparationTicks = 0, canonicalTicks = 0, subjectConversionTicks = 0, mappingTicks = 0;
        long sourceVertices = 0;
        int sourceContours = 0, retainedCount = 0;
        bool completed = false;
        PreparedResidualCoverage? prepared = null;
        try
        {
            cancellationToken.ThrowIfCancellationRequested();
            long stage = Stopwatch.GetTimestamp();
            prepared = PolygonOps.PrepareComponentDifference(expanded, cancellationToken);
            preparationTicks = Stopwatch.GetTimestamp() - stage;
            var retained = new List<MillingRegion>();
            foreach (var region in regions)
            {
                cancellationToken.ThrowIfCancellationRequested();
                stage = Stopwatch.GetTimestamp();
                var source = PolygonOps.Difference(new[] { region.Outer }, region.Holes);
                canonicalTicks += Stopwatch.GetTimestamp() - stage;
                sourceContours += source.Count;
                sourceVertices += source.Sum(path => (long)path.Count);
                cancellationToken.ThrowIfCancellationRequested();
                stage = Stopwatch.GetTimestamp();
                var subjects = PolygonOps.ComponentDifferencePaths(source, cancellationToken);
                subjectConversionTicks += Stopwatch.GetTimestamp() - stage;
                var components = prepared.DifferenceComponents(subjects, cancellationToken);
                stage = Stopwatch.GetTimestamp();
                foreach (var component in components)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    retained.Add(new MillingRegion(component.Outer,
                        component.Holes.Select(hole => (IReadOnlyList<PointD>)hole).ToArray()));
                }
                mappingTicks += Stopwatch.GetTimestamp() - stage;
            }
            cancellationToken.ThrowIfCancellationRequested();
            retainedCount = retained.Count;
            completed = true;
            return retained;
        }
        finally
        {
            static double Milliseconds(long ticks) => ticks * 1000d / Stopwatch.Frequency;
            Console.WriteLine($"[Pocket.Protection] stage=prepared-components completed={completed} "
                + $"originalRegions={regions.Count} retainedRegions={retainedCount} "
                + $"sourceContours={sourceContours} sourceVertices={sourceVertices} "
                + $"maskContours={prepared?.OriginalContours ?? 0} maskVertices={prepared?.OriginalVertices ?? 0} "
                + $"queries={prepared?.DifferenceQueries ?? 0} consideredVertices={prepared?.ConsideredVertices ?? 0} "
                + $"selectedVertices={prepared?.SelectedVertices ?? 0} selectedContours={prepared?.SelectedContours ?? 0} "
                + $"fullSelectionQueries={prepared?.FullSelectionQueries ?? 0} emptySelectionQueries={prepared?.EmptySelectionQueries ?? 0} "
                + $"inputBuilds={prepared?.InputBuilds ?? 0} "
                + $"inputReuses={prepared?.InputReuses ?? 0} maskPrepareMs={Milliseconds(preparationTicks):F3} "
                + $"canonicalMs={Milliseconds(canonicalTicks):F3} subjectConversionMs={Milliseconds(subjectConversionTicks):F3} "
                + $"boundsMs={Milliseconds(prepared?.BoundsTicks ?? 0):F3} "
                + $"inputBuildMs={Milliseconds(prepared?.InputTicks ?? 0):F3} "
                + $"enginePrepareMs={Milliseconds(prepared?.ComponentPreparationTicks ?? 0):F3} "
                + $"executeMs={Milliseconds(prepared?.ComponentExecuteTicks ?? 0):F3} "
                + $"materializeMs={Milliseconds(prepared?.ComponentMaterializeTicks ?? 0):F3} "
                + $"mappingMs={Milliseconds(mappingTicks):F3} elapsedMs={Milliseconds(Stopwatch.GetTimestamp() - started):F3} "
                + $"allocatedBytes={GC.GetAllocatedBytesForCurrentThread() - allocated} "
                + "selection=complete-signed-contour-bounds guardTicks=1 inputOwnership=immutable-copy "
                + "subjectOrder=canonical-original clipOrder=original componentOrder=literal-polytree "
                + "timingNesting=bounds-and-inputBuild-within-enginePrepare densePolicy=full-original-clips "
                + "failurePolicy=throw-discard-partial");
        }
    }

    internal static void Verify(IReadOnlyList<TaggedToolpath> paths, double radius, double surfaceZ,
        IReadOnlyList<IReadOnlyList<PointD>>? protection, CancellationToken cancellationToken)
    {
        if (protection is not { Count: > 0 }) return;
        var proof = new PreparedSweepExclusion(protection);
        int chords = 0;
        for (int pathIndex = 0; pathIndex < paths.Count; pathIndex++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var path = paths[pathIndex];
            for (int index = 1; index < path.Points.Count; index++)
            {
                var a = path.Points[index - 1];
                var b = path.Points[index];
                if (Math.Min(a.Z, b.Z) >= surfaceZ) continue;
                chords++;
                if (proof.IsClear(new PointD(a.X, a.Y), new PointD(b.X, b.Y), radius)) continue;
                string evidence = $"path={pathIndex} chord={index - 1} radius={radius:R} "
                    + $"from=({a.X:R},{a.Y:R},{a.Z:R}) to=({b.X:R},{b.Y:R},{b.Z:R})";
                Console.WriteLine($"[Pocket.Protection] stage=emitted-footprint result=rejected {evidence}");
                throw new InvalidOperationException("Roughing cutter footprint intersects protected smooth V-carving material: " + evidence);
            }
        }
        Console.WriteLine($"[Pocket.Protection] stage=emitted-footprint result=verified chords={chords} radius={radius:R}");
    }
}
