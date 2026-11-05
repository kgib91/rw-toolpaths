using Clipper2Lib;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling.Certification;

namespace RW.Toolpaths;

public static partial class MedialAxisGeometry
{
    /// <summary>
    /// Common area-fill geometry for flat pockets and depth-limited V-carve clearing.
    /// The concentric collector retains closed contours; terminal wavefronts enter
    /// the same offset tree as open traces before any routing or depth scheduling.
    /// </summary>
    public static List<OffsetRing> BuildConcentricFill(
        IReadOnlyList<MillingRegion> regions,
        RingOffsetOptions options,
        double sweepRadius,
        bool preserveTerminalPaths,
        IReadOnlyList<PointD>? clipBoundary = null,
        CancellationToken cancellationToken = default,
        Action<double, string>? progress = null)
    {
        ArgumentNullException.ThrowIfNull(regions);
        ArgumentNullException.ThrowIfNull(options);
        if (!double.IsFinite(sweepRadius) || sweepRadius < 0)
            throw new ArgumentOutOfRangeException(nameof(sweepRadius));
        if (!double.IsFinite(options.FirstOffset) || !double.IsFinite(options.StepOver))
            throw new ArgumentOutOfRangeException(nameof(options), "Fill offsets must be finite.");
        cancellationToken.ThrowIfCancellationRequested();
        options = options with { RingLimit = Math.Clamp(options.RingLimit, 1, RingOffsetEngine.AbsoluteRingLimit) };
        // Every physical fill keeps the original-source chord proof through
        // materialization. Certified source-site construction and guarded native
        // front cleanup replace ordinary output RDP;
        // at pitches above the physical radius, the same precise fronts also
        // certify surviving contours and terminal collapses between levels.
        if (preserveTerminalPaths && options.OffsetSign < 0 && sweepRadius > 0)
        {
            var requested = options.Tolerances;
            var accurate = requested with
            {
                ArcTolerance = Math.Min(requested.ArcTolerance, .001),
                SimplifyTolerance = 0,
            };
            var first = accurate with { ArcTolerance = Math.Min(accurate.ArcTolerance, 1 / PathUtils.Scale) };
            options = options with { Tolerances = accurate, FirstLevelTolerances = first };
            Console.WriteLine($"[MedialAxisGeometry.FillFidelity] requestedArc={requested.ArcTolerance:R} "
                + $"requestedSimplify={requested.SimplifyTolerance:R} arc={accurate.ArcTolerance:R} "
                + $"simplify={accurate.SimplifyTolerance:R} firstArc={first.ArcTolerance:R} "
                + $"firstSourceCoverage=original scope=all-levels");
        }
        // A finite authored band is safe to certify in the same collector: its
        // candidate distance is capped before testing for a vanished child.
        // A surviving child at that cap never requests a terminal interior path.
        bool retainCollapses = preserveTerminalPaths && options.OffsetSign < 0 && sweepRadius > 0;
        var sourceGraphs = new Dictionary<int, PreparedTerminalMedial>();
        var collapsedComponents = new Dictionary<(int Region, int Level, double CurrentOffset, double Radius, IReadOnlyList<PointD> Outer),
            IReadOnlyList<IReadOnlyList<PointD>>>();
        int terminalCacheHits = 0;
        PreparedTerminalMedial SourceGraph(int regionIndex)
        {
            if (!sourceGraphs.TryGetValue(regionIndex, out var graph))
                sourceGraphs.Add(regionIndex, graph = new PreparedTerminalMedial(
                    regions[regionIndex], cancellationToken, lazy: true,
                    boundOriginalSource: options.BoundOriginalSourceMedial,
                    preserveNativeGrid: options.PreserveSourceNativeGrid));
            return graph;
        }
        IReadOnlyList<IReadOnlyList<PointD>> CollapsedPaths(CollapsedOffsetComponent collapse, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            // BuildCollapsedComponent reads only this fixed original source,
            // current component, physical radius and operation-owned first
            // inset/clip. Current components and all their hole contours are
            // constructed once per collector front and are immutable throughout
            // its trial loop. Outer reference identity therefore identifies the
            // complete component, including its holes. CandidateOffsetDistance
            // is diagnostic text only: it never affects native terminal geometry.
            var key = (collapse.RegionIndex, collapse.ParentLevel, collapse.CurrentOffsetDistance,
                collapse.SweepRadius, collapse.CurrentComponent.Outer);
            bool reuse = options.CacheCollapsedTerminals
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1"
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_TERMINAL_CACHE_REFERENCE") != "1";
            if (reuse && collapsedComponents.TryGetValue(key, out var paths))
            {
                terminalCacheHits++;
                Console.WriteLine($"[Offset.Terminal.Cache] region={collapse.RegionIndex} level={collapse.ParentLevel} "
                    + $"currentInset={collapse.CurrentOffsetDistance:R} candidateInset={collapse.CandidateOffsetDistance:R} "
                    + $"paths={paths.Count} completeComponentIdentity=True candidateInsetEffect=diagnostic-only");
                return paths;
            }
            var computed = OffsetTerminalPaths.BuildCollapsedComponent(collapse, () => SourceGraph(collapse.RegionIndex),
                options.FirstOffset, clipBoundary, token);
            if (reuse) collapsedComponents.Add(key, computed);
            return computed;
        }
        if (retainCollapses)
            options = options with
            {
                PhysicalSweepRadius = sweepRadius,
                PhysicalClipBoundary = clipBoundary,
                TerminalPaths = CollapsedPaths,
                SourceConstructionTolerance = .001,
            };

        var paths = RingOffsetEngine.BuildRings(regions, options, cancellationToken, progress);
        if (retainCollapses && options.FirstOffset > 4 / PathUtils.Scale)
        {
            for (int regionIndex = 0; regionIndex < regions.Count; regionIndex++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var region = regions[regionIndex];
                if (region.Outer.Count < 3) continue;
                var material = PolygonOps.Difference(new[] { region.Outer }, region.Holes);
                var boundary = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(material));
                if (!boundary.MayContainDisc(options.FirstOffset * PathUtils.Scale, .001 * PathUtils.Scale)) continue;
                if (ExactFirstCoverageGate.Enabled(options))
                {
                    var prepared = SourceGraph(regionIndex);
                    if (!prepared.FullGraphMaterialized)
                    {
                        bool covered = ExactFirstCoverageGate.TryCertify(prepared.CanonicalSource,
                            prepared.Boundary, paths, regionIndex, options.FirstOffset,
                            sweepRadius, clipBoundary, cancellationToken, out var gate);
                        var proof = gate.SourceCertificate;
                        Console.WriteLine($"[Offset.Terminal.ExactFirstGate] region={regionIndex} certified={covered} "
                            + $"reason={gate.Reason} sourcePoints={gate.SourcePoints} sidePoints={gate.SidePoints} "
                            + $"centerPoints={gate.CenterPoints} footprintPoints={gate.FootprintPoints} "
                            + $"closedRings={gate.ClosedRings} openRings={gate.OpenRings} "
                            + $"prunedComponents={gate.PrunedComponents} retainedComponents={gate.RetainedComponents} "
                            + $"componentNodes={gate.ComponentNodes} physicalChords={gate.PhysicalChords} "
                            + $"enclosedFootprintVertices={gate.EnclosedFootprintVertices} "
                            + $"unresolvedHulls={gate.UnresolvedHulls} retainedHulls={gate.RetainedHulls} hullCuts={gate.HullCuts} hullRefusals={gate.HullRefusals} "
                            + $"finalResidualEdges={proof?.FinalResidualEdges ?? -1} ownerRegistry={proof?.OriginalOwnerRegistryEnforced ?? false} "
                            + $"nominationQueries={gate.NominationQueries} memoHits={gate.NominationMemoHits} gapPaths={gate.GapPaths} "
                            + $"strictSourceRadiusNumerator={gate.StrictSourceRadius} strictSourceRadiusFractionBits={gate.StrictSourceRadiusFractionBits} actualLegalRadius={options.FirstOffset * PathUtils.Scale - 4:R} "
                            + $"coverageReserve={ExactFirstCoverageGate.CoverageReserve:R} "
                            + $"triangles={proof?.MainTriangles ?? 0} corrections={proof?.CorrectionTriangles ?? 0} "
                            + $"triangleTasks={proof?.TriangleTasks ?? 0} pieces={proof?.CertifiedPieces ?? 0} "
                            + $"bigIntegerFallbacks={proof?.BigIntegerFallbacks ?? 0} "
                            + $"candidateMs={gate.CandidateMilliseconds:F3} sourceProofMs={gate.SourceCertificateMilliseconds:F3} "
                            + $"hullComplementMs={gate.HullComplementMilliseconds:F3} physicalMs={gate.PhysicalMilliseconds:F3} acceptedSweepMs={gate.AcceptedSweepMilliseconds:F3} "
                            + $"gapMs={gate.GapMilliseconds:F3} totalMs={gate.TotalMilliseconds:F3} "
                            + $"sourceAuthority=complete-original-signed-finite-sites cutCredit=accepted-emitted-only fallback=full-original");
                        if (covered) continue;
                    }
                    else Console.WriteLine($"[Offset.Terminal.ExactFirstGate] region={regionIndex} "
                        + $"certified=False reason=full-original-already-materialized");
                }
                // A line-valued first front has no filled polygon to give to the
                // collector. Retain that exact tangent locus, including a stem
                // attached to a room whose first closed front survives.
                var exact = OffsetTerminalPaths.BuildExactFirstCollapse(regions[regionIndex],
                    SourceGraph(regionIndex).Segments, options.FirstOffset, sweepRadius, clipBoundary, cancellationToken);
                foreach (var trace in exact)
                    paths.Add(RingOffsetEngine.CreateRing(trace.ToList(), 0, regionIndex,
                        isHole: false, isClosed: false));
            }
        }
        Console.WriteLine($"[MedialAxisGeometry.Fill] regions={regions.Count} closed={paths.Count(p => p.IsClosed)} "
            + $"open={paths.Count(p => !p.IsClosed)} firstInset={options.FirstOffset:R} "
            + $"pitch={options.StepOver:R} sweepRadius={sweepRadius:R} maxInset={options.MaximumInset:R} "
            + $"terminalPolicy={preserveTerminalPaths} medialSources={sourceGraphs.Count} "
            + $"collapsedComponents={collapsedComponents.Count} terminalCacheHits={terminalCacheHits}");
        return paths;
    }
}
