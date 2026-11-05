using Clipper2Lib;
using System.Diagnostics;

namespace RW.Toolpaths.Geometry;

/// <summary>Retains independent filled fronts whose complete cutter footprint contributes material.</summary>
internal static class FilledFrontCoverage
{
    private const double ProofArc = .001;
    private const double CoverageError = .003 + 4 / PathUtils.Scale;
    private static readonly FilledFrontEvidenceCache.Cache<RetainedEvidence> RetainedCache = new(16L << 20, 64);
    private static readonly FilledFrontEvidenceCache.Cache<Footprint> FootprintCache = new(32L << 20, 128);

    internal readonly record struct Statistics(int Components, int Retained, int FreshWitnesses,
        int BooleanFallbacks, int OrderedUnionCalls, int WitnessProbes, int IndexBuilds,
        int FootprintQueries, int ContainmentQueries, int ContainmentProofs,
        int ContainmentAmbiguous, int ContainmentFirstQueryBooleans, bool EvidenceReused = false);

    internal static (FilledFrontEvidenceCache.State Results, FilledFrontEvidenceCache.State Footprints)
        InspectEvidenceCache() => (RetainedCache.Inspect(), FootprintCache.Inspect());

    internal static void ClearEvidenceCache()
    { RetainedCache.Clear(); FootprintCache.Clear(); }

    private sealed class RetainedEvidence
    {
        private readonly PointD[][] _paths;
        internal Statistics ProofStatistics { get; }
        internal long Bytes => 64L + _paths.Sum(path => 32L + 16L * path.Length);
        internal RetainedEvidence(IReadOnlyList<List<PointD>> paths, Statistics statistics)
        { _paths = paths.Select(path => path.ToArray()).ToArray(); ProofStatistics = statistics; }
        internal List<List<PointD>> Copy() => _paths.Select(path => path.ToList()).ToList();
    }

    internal static List<List<PointD>> RetainUncoveredComponents(
        IReadOnlyList<List<PointD>> polygons, double radius, CancellationToken token,
        IReadOnlyList<PointD>? clipBoundary = null, bool useFreshWitness = true,
        Action<Statistics>? observe = null, bool usePreparedContainment = true)
    {
        token.ThrowIfCancellationRequested();
        long started = Stopwatch.GetTimestamp();
        bool cacheEnabled = FilledFrontEvidenceCache.Enabled;
        if (cacheEnabled && clipBoundary is not null) clipBoundary = clipBoundary.ToArray();
        var components = PolygonOps.DifferenceToComponents(polygons, Array.Empty<IReadOnlyList<PointD>>());
        var canonical = new List<List<PointD>>();
        foreach (var component in components)
        {
            canonical.Add(component.Outer);
            canonical.AddRange(component.Holes);
        }
        if (canonical.Count != polygons.Count)
            Console.WriteLine($"[FilledFrontCoverage.Topology] inputContours={polygons.Count} "
                + $"canonicalContours={canonical.Count} components={components.Count} "
                + $"proof=identical-filled-union holesPreserved=True sizeCutoff=False");
        if (components.Count < 2) return canonical;
        // Credit only cutter strokes the downstream clip will actually emit.
        if (clipBoundary is { Count: >= 3 }
            && PolygonOps.Difference(polygons, new[] { clipBoundary }).Count > 0)
        {
            Console.WriteLine($"[FilledFrontCoverage] components={components.Count} "
                + $"retained=all reason=partially-clipped-front");
            return canonical;
        }
        const double proofArc = ProofArc;
        const double coverageError = CoverageError;
        var effectKey = cacheEnabled ? FilledFrontEvidenceCache.Key.Capture(canonical,
            radius, proofArc, coverageError, (useFreshWitness ? 1 : 0) | (usePreparedContainment ? 2 : 0),
            clipBoundary, token) : null;
        var reused = cacheEnabled ? RetainedCache.Find(effectKey, token) : null;
        if (reused is not null)
        {
            token.ThrowIfCancellationRequested();
            var copy = reused.Copy();
            token.ThrowIfCancellationRequested();
            var proof = reused.ProofStatistics;
            observe?.Invoke(new Statistics(proof.Components, proof.Retained, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, true));
            token.ThrowIfCancellationRequested();
            Console.WriteLine($"[FilledFrontCoverage.Evidence] hit=True effectHash={effectKey!.Hash:x16} "
                + $"canonicalPoints={effectKey.Points} components={proof.Components} retained={proof.Retained} "
                + $"priorBooleanFallbacks={proof.BooleanFallbacks} priorOrderedUnions={proof.OrderedUnionCalls} "
                + $"newFootprints=0 newOrderedUnions=0 elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F2} "
                + $"key=exact-ordered-signed-coordinate-bits-radius-clip-modes-proof resultOwnership=fresh-copy stockAuthority=unchanged");
            return copy;
        }
        double canonicalMs = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        long footprintTicks = 0, orderedUnionTicks = 0, morphologyTicks = 0, witnessTicks = 0;
        int footprintHits = 0, footprintMisses = 0;
        var retained = new HashSet<int>();
        var swept = new List<List<PointD>>();
        var sweptInteger = new Paths64();
        PreparedFilledContainment? containment = null;
        int queriesForSwept = 0;
        var accepted = new List<Footprint>();
        var forest = new FootprintForest();
        bool hasSweptMaterial = false;
        int materialized = 0, unions = 0, witnesses = 0, fallbacks = 0, probes = 0,
            builds = 0, queries = 0, containmentQueries = 0, containmentProofs = 0,
            containmentAmbiguous = 0, containmentFirstQueryBooleans = 0;
        // Witnesses are intentionally stricter than the existing Boolean stopping
        // test. Reserve the arc error and integer operations of every possible
        // accepted prefix; an inconclusive witness never authorizes a discard.
        double witnessRadius = coverageError + proofArc + (8d * components.Count + 8) / PathUtils.Scale;
        void MaterializeAccepted()
        {
            while (materialized < accepted.Count)
            {
                token.ThrowIfCancellationRequested();
                // Preserve the original insertion sequence and one Union per
                // accepted footprint, including the first. Integer Boolean
                // rounding makes reordered or spatially pruned unions unsafe.
                var ordered = new Paths64(sweptInteger);
                ordered.AddRange(accepted[materialized++].IntegerPaths);
                long unionStarted = Stopwatch.GetTimestamp();
                sweptInteger = PolygonOps.UnionNormalizedInteger(ordered);
                orderedUnionTicks += Stopwatch.GetTimestamp() - unionStarted;
                containment = null;
                queriesForSwept = 0;
                unions++;
            }
            // Materialize only when the original tolerant Boolean stopping test
            // is actually required; ordered unions retain integer coordinates.
            swept = PolygonOps.MaterializeIntegerPaths(sweptInteger);
        }
        // A discarded component never authorizes another discard.
        int visitOrdinal = 0;
        foreach (var item in components.Select((component, index) => (component, index))
            .OrderByDescending(item => Math.Abs(PolygonOps.SignedArea(item.component.Outer))))
        {
            token.ThrowIfCancellationRequested();
            int visit = visitOrdinal++;
            var boundaries = new List<IReadOnlyList<PointD>> { item.component.Outer };
            boundaries.AddRange(item.component.Holes);
            long footprintStarted = Stopwatch.GetTimestamp();
            var footprintKey = cacheEnabled ? FilledFrontEvidenceCache.Key.Capture(boundaries,
                radius, proofArc, 0, 0, null, token) : null;
            var indexed = cacheEnabled ? FootprintCache.Find(footprintKey, token) : null;
            if (indexed is null)
            {
                indexed = new Footprint(PolygonOps.BufferCenterlinesExact(boundaries, radius, proofArc, token));
                token.ThrowIfCancellationRequested();
                if (cacheEnabled) FootprintCache.Publish(footprintKey, indexed, indexed.EstimatedBytes, token);
                footprintMisses++;
            }
            else footprintHits++;
            var footprint = indexed.Paths;
            footprintTicks += Stopwatch.GetTimestamp() - footprintStarted;
            token.ThrowIfCancellationRequested();
            if (hasSweptMaterial)
            {
                long witnessStarted = Stopwatch.GetTimestamp();
                bool hasWitness = useFreshWitness && HasFreshWitness(boundaries, indexed, forest,
                    witnessRadius, token, ref probes, ref builds, ref queries);
                witnessTicks += Stopwatch.GetTimestamp() - witnessStarted;
                if (hasWitness) witnesses++;
                else
                {
                    // Optional diagnostics copy inputs BEFORE the exact original
                    // prefix is materialized. No scope means no capture allocation,
                    // enumeration, serialization, I/O or decision change.
                    FilledFrontCoverageFallbackCapture.Pending? capture = null;
                    if (FilledFrontCoverageFallbackCapture.Current is { } captureScope)
                    {
                        capture = captureScope.TryBegin(item.index, visit, components.Count, materialized,
                            new(BitConverter.DoubleToInt64Bits(PathUtils.Scale),
                                BitConverter.DoubleToInt64Bits(radius), BitConverter.DoubleToInt64Bits(proofArc),
                                BitConverter.DoubleToInt64Bits(coverageError), BitConverter.DoubleToInt64Bits(witnessRadius),
                                useFreshWitness, usePreparedContainment, cacheEnabled),
                            boundaries, clipBoundary, new(indexed.IntegerPaths, indexed.Paths),
                            accepted.Select(value => new FilledFrontCoverageFallbackCapture.FootprintInput(
                                value.IntegerPaths, value.Paths)), token, accepted.Count);
                    }
                    MaterializeAccepted();
                    if (swept.Count > 0)
                    {
                        // Preparation pays for repeated containment predicates
                        // against one accepted prefix. A one-use changing
                        // prefix retains its original Boolean immediately.
                        if (usePreparedContainment && ++queriesForSwept > 1)
                        {
                            containment ??= new PreparedFilledContainment(sweptInteger);
                            containmentQueries++;
                            bool? contains = containment.ContainsCanonical(indexed.IntegerPaths, token);
                            if (contains is true)
                            {
                                containmentProofs++;
                                capture?.Complete(sweptInteger, null, null, materialized,
                                    "prepared-containment-discarded", false, token);
                                continue;
                            }
                            if (contains is null) containmentAmbiguous++;
                        }
                        else if (usePreparedContainment) containmentFirstQueryBooleans++;
                        fallbacks++;
                        long morphologyStarted = Stopwatch.GetTimestamp();
                        var fresh = PolygonOps.Difference(footprint, swept);
                        var interior = PolygonOps.Offset(fresh, -coverageError, proofArc / 4, 0);
                        morphologyTicks += Stopwatch.GetTimestamp() - morphologyStarted;
                        if (Math.Abs(interior.Sum(PolygonOps.SignedArea)) == 0)
                        {
                            capture?.Complete(sweptInteger, fresh, interior, materialized,
                                "original-boolean-discarded", false, token);
                            continue;
                        }
                        capture?.Complete(sweptInteger, fresh, interior, materialized,
                            "original-boolean-retained", true, token);
                    }
                    else capture?.Complete(sweptInteger, null, null, materialized,
                        "empty-accepted-prefix-retained", true, token);
                }
            }
            retained.Add(item.index);
            accepted.Add(indexed);
            forest.Add(indexed);
            hasSweptMaterial |= footprint.Count > 0;
            if (!useFreshWitness) MaterializeAccepted();
        }
        var result = new List<List<PointD>>();
        for (int index = 0; index < components.Count; index++)
        {
            if (!retained.Contains(index)) continue;
            result.Add(components[index].Outer);
            result.AddRange(components[index].Holes);
        }
        Console.WriteLine($"[FilledFrontCoverage] components={components.Count} "
            + $"retained={retained.Count} redundant={components.Count - retained.Count} "
            + $"proof=complete-cutter-footprint coverageError={coverageError:R} "
            + $"freshDiscWitnesses={witnesses} booleanFallbacks={fallbacks} orderedUnionCalls={unions} "
            + $"witnessProbes={probes} indexBuilds={builds} footprintQueries={queries} "
            + $"containmentQueries={containmentQueries} containmentProofs={containmentProofs} "
            + $"containmentAmbiguous={containmentAmbiguous} "
            + $"containmentFirstQueryBooleans={containmentFirstQueryBooleans} "
            + $"containmentPrepareAfterQuery=1 enclosedHoles=True "
            + $"witnessRadius={witnessRadius:R} originalBooleanOrder=True");
        var statistics = new Statistics(components.Count, retained.Count, witnesses, fallbacks,
            unions, probes, builds, queries, containmentQueries, containmentProofs,
            containmentAmbiguous, containmentFirstQueryBooleans);
        observe?.Invoke(statistics);
        token.ThrowIfCancellationRequested();
        if (cacheEnabled)
        {
            var evidence = new RetainedEvidence(result, statistics);
            RetainedCache.Publish(effectKey, evidence, evidence.Bytes, token);
        }
        Console.WriteLine($"[FilledFrontCoverage.Evidence] hit=False effectHash={effectKey?.Hash:x16} "
            + $"canonicalPoints={canonical.Sum(path => path.Count)} components={components.Count} retained={retained.Count} "
            + $"footprintHits={footprintHits} footprintMisses={footprintMisses} canonicalMs={canonicalMs:F2} "
            + $"footprintMs={Milliseconds(footprintTicks):F2} witnessMs={Milliseconds(witnessTicks):F2} "
            + $"orderedUnionMs={Milliseconds(orderedUnionTicks):F2} morphologyMs={Milliseconds(morphologyTicks):F2} "
            + $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F2} cacheEnabled={cacheEnabled} "
            + $"resultCacheBytes={RetainedCache.Inspect().Bytes} footprintCacheBytes={FootprintCache.Inspect().Bytes} "
            + $"originalBooleanOrder=True stockAuthority=unchanged");
        return result;
    }

    private static double Milliseconds(long ticks) => ticks * 1000d / Stopwatch.Frequency;

    private static bool HasFreshWitness(IReadOnlyList<IReadOnlyList<PointD>> boundaries,
        Footprint candidate, FootprintForest accepted, double radius, CancellationToken token,
        ref int probes, ref int builds, ref int queries)
    {
        // These points only search for a positive proof. Missing a fresh patch
        // falls back to the complete footprint; sampling never proves coverage.
        foreach (var ring in boundaries)
        {
            int count = Math.Min(32, ring.Count);
            for (int sample = 0; sample < count; sample++)
            {
                token.ThrowIfCancellationRequested();
                var value = ring[(int)((long)sample * ring.Count / count)];
                if (!double.IsFinite(value.x) || !double.IsFinite(value.y)) return false;
                var point = new Point64((long)Math.Round(value.x * PathUtils.Scale),
                    (long)Math.Round(value.y * PathUtils.Scale));
                probes++;
                double required = radius * PathUtils.Scale;
                if (!candidate.ContainsDisc(point, required, ref builds)) continue;
                if (accepted.IsDiscClear(point, required, token, ref builds, ref queries)) return true;
            }
        }
        return false;
    }

    private sealed class Footprint
    {
        internal IReadOnlyList<List<PointD>> Paths { get; }
        internal long MinX { get; } = long.MaxValue;
        internal long MinY { get; } = long.MaxValue;
        internal long MaxX { get; } = long.MinValue;
        internal long MaxY { get; } = long.MinValue;
        private readonly Paths64 _paths;
        internal Paths64 IntegerPaths => _paths;
        private readonly Lazy<PolygonOps.BoundarySegmentIndex> _index;
        private int _indexBuildObserved;
        // Includes a conservative reservation for the lazily built segment BVH.
        internal long EstimatedBytes => 256L + _paths.Sum(path => 64L + 512L * path.Count);

        internal Footprint(IReadOnlyList<List<PointD>> paths)
        {
            Paths = paths;
            _paths = PathUtils.ToClipper(paths);
            _index = new Lazy<PolygonOps.BoundarySegmentIndex>(() => new(_paths),
                LazyThreadSafetyMode.ExecutionAndPublication);
            foreach (var ring in _paths)
            foreach (var point in ring)
            {
                MinX = Math.Min(MinX, point.X); MinY = Math.Min(MinY, point.Y);
                MaxX = Math.Max(MaxX, point.X); MaxY = Math.Max(MaxY, point.Y);
            }
        }

        private PolygonOps.BoundarySegmentIndex Index(ref int builds)
        {
            if (Interlocked.CompareExchange(ref _indexBuildObserved, 1, 0) == 0) builds++;
            return _index.Value;
        }

        internal bool ContainsDisc(Point64 point, double radius, ref int builds)
        {
            var index = Index(ref builds);
            return index.DistanceSquared(point) > radius * radius && index.IsInside(point);
        }

        internal bool IsDiscClear(Point64 point, double radius, ref int builds)
        {
            var index = Index(ref builds);
            return index.DistanceSquared(point) > radius * radius && !index.IsInside(point);
        }
    }

    // A binomial forest gives logarithmic append without rebuilding a global
    // polygon arrangement. Complete bounds prune unrelated physical footprints;
    // each leaf keeps its original NonZero boundaries, including all holes.
    private sealed class FootprintForest
    {
        private sealed class Node
        {
            internal readonly long MinX, MinY, MaxX, MaxY;
            internal readonly Footprint? Leaf;
            internal readonly Node? First, Second;
            internal Node(Footprint leaf)
            {
                Leaf = leaf;
                MinX = leaf.MinX; MinY = leaf.MinY; MaxX = leaf.MaxX; MaxY = leaf.MaxY;
            }
            internal Node(Node first, Node second)
            {
                First = first; Second = second;
                MinX = Math.Min(first.MinX, second.MinX); MinY = Math.Min(first.MinY, second.MinY);
                MaxX = Math.Max(first.MaxX, second.MaxX); MaxY = Math.Max(first.MaxY, second.MaxY);
            }
        }
        private readonly List<Node?> _roots = new();

        internal void Add(Footprint footprint)
        {
            var node = new Node(footprint);
            int level = 0;
            while (level < _roots.Count && _roots[level] is { } prior)
            {
                node = new Node(prior, node);
                _roots[level++] = null;
            }
            if (level == _roots.Count) _roots.Add(node);
            else _roots[level] = node;
        }

        internal bool IsDiscClear(Point64 point, double radius, CancellationToken token,
            ref int builds, ref int queries)
        {
            foreach (var root in _roots)
                if (root != null && !Visit(root, point, radius, token, ref builds, ref queries)) return false;
            return true;
        }

        private static bool Visit(Node node, Point64 point, double radius, CancellationToken token,
            ref int builds, ref int queries)
        {
            token.ThrowIfCancellationRequested();
            double dx = point.X < node.MinX ? (double)node.MinX - point.X
                : point.X > node.MaxX ? (double)point.X - node.MaxX : 0;
            double dy = point.Y < node.MinY ? (double)node.MinY - point.Y
                : point.Y > node.MaxY ? (double)point.Y - node.MaxY : 0;
            if (dx * dx + dy * dy > radius * radius) return true;
            if (node.Leaf != null)
            {
                queries++;
                return node.Leaf.IsDiscClear(point, radius, ref builds);
            }
            return Visit(node.First!, point, radius, token, ref builds, ref queries)
                && Visit(node.Second!, point, radius, token, ref builds, ref queries);
        }
    }
}
