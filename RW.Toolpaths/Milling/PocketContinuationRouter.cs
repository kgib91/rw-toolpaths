using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Authorizes floor continuations in final execution order against actual cleared stock.</summary>
internal static class PocketContinuationRouter
{
    private const double ArcTolerance = .001;
    // Include the bounded dense sweep's intermediate union as well as source,
    // final union and command-grid rounding. This only withholds stock credit.
    private const double ConstructionError = .003 + 6 / PathUtils.Scale;
    internal const double KnownAirReserve = ConstructionError + .002 + 8 / PathUtils.Scale;

    internal sealed record Result(List<TaggedToolpath> Paths, int Lifts, int Plunges,
        int LinearRamps, int Helicals, int Rebased, int Candidates);

    internal static Result Route(IReadOnlyList<TaggedToolpath> source, DepthSchedule depth,
        PocketOptions options, double radius, CancellationToken token, bool useIndexedCoverage = true,
        ToolGeometry? tool = null)
    {
        long routeStarted = Stopwatch.GetTimestamp();
        long allocationStarted = GC.GetAllocatedBytesForCurrentThread();
        useIndexedCoverage &= Environment.GetEnvironmentVariable("RW_TOOLPATHS_ROUTING_REFERENCE") != "1";
        bool validateIndexedCoverage = Environment.GetEnvironmentVariable("RW_TOOLPATHS_VALIDATE_ROUTING_INDEX") == "1";
        bool limitCropWork = Environment.GetEnvironmentVariable("RW_TOOLPATHS_COVERAGE_CROP_BUDGET_REFERENCE") != "1";
        var stock = new ClearedStock(radius, token, knownEmpty:options.KnownEmptyBoundary,knownEmptyFloor:options.KnownEmptyFloor);
        bool flatCylinder = TryFlatCylinderRadius(tool, RequiredPostedCylinderHeight(depth), out double physicalRadius);
        var postedStepdown = options.PreserveCommandVertices
            ? new PocketPostedStepdownStock(depth, flatCylinder ? physicalRadius : radius, token, flatCylinder) : null;
        var coverageDiagnostics = new CoverageDiagnostics();
        var followingCache = useIndexedCoverage ? new FollowingSweepCache() : null;
        var output = new List<TaggedToolpath>();
        var spatial = options.LinkClearance ?? LinkClearance.Unrestricted;
        int lifts = 0, plunges = 0, linear = 0, helicals = 0, rebased = 0, candidates = 0;
        int nextVisit = source.Where(path => path.VisitIdentity.HasValue)
            .Select(path => path.VisitIdentity!.Value).DefaultIfEmpty(-1).Max() + 1;
        int checkedLinks = 0;
        double capturedProofPrepareMilliseconds = 0, routeRepairMilliseconds = 0, executedRecordMilliseconds = 0;
        double MeasuredClearanceMilliseconds() => stock.FloorProjectionMilliseconds + stock.FloorSweepMilliseconds
            + stock.FloorUnionMilliseconds + coverageDiagnostics.ExclusiveMilliseconds
            + (postedStepdown?.ProofMilliseconds ?? 0);
        void RecordExecuted(TaggedToolpath path, ToolpathSpan span)
        {
            long started = Stopwatch.GetTimestamp();
            var stroke = path.Points.Skip(span.StartIndex).Take(span.PointCount).ToList();
            stock.Record(stroke);
            postedStepdown?.Record(stroke);
            executedRecordMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        }
        foreach (var original in source)
        {
            token.ThrowIfCancellationRequested();
            var path = original;
            int repairs = 0;
            bool finished = false;
            while (!finished)
            {
                token.ThrowIfCancellationRequested();
                finished = true;
                for (int spanIndex = 0; spanIndex < path.Spans.Count; spanIndex++)
                {
                    var span = path.Spans[spanIndex];
                    if (span.MoveCount <= 0) continue;
                    if (span.Kind != ToolpathSpanKind.Link || !IsFlat(path, span))
                    {
                        if (span.Kind is ToolpathSpanKind.Cut or ToolpathSpanKind.Ramp or ToolpathSpanKind.Link)
                            RecordExecuted(path, span);
                        continue;
                    }
                    int linkRunEnd = spanIndex;
                    while (linkRunEnd + 1 < path.Spans.Count
                        && path.Spans[linkRunEnd + 1].Kind == ToolpathSpanKind.Link
                        && path.Spans[linkRunEnd + 1].StartIndex == span.EndIndex
                        && IsFlat(path, path.Spans[linkRunEnd + 1]))
                    {
                        linkRunEnd++;
                        span = new ToolpathSpan(span.StartIndex, path.Spans[linkRunEnd].EndIndex, ToolpathSpanKind.Link);
                    }
                    double floor = path.Points[span.StartIndex].Z;
                    int cutIndex = NextCut(path, spanIndex);
                    long prepareStarted = Stopwatch.GetTimestamp();
                    double priorPrepareClearance = MeasuredClearanceMilliseconds();
                    var following = cutIndex < 0 ? Array.Empty<IReadOnlyList<PointD>>()
                        : BelowFloor(path.Points.Skip(path.Spans[cutIndex].StartIndex)
                            .Take(path.Spans[cutIndex].PointCount).ToList(), floor);
                    var proof = new TemporalClearance(spatial, stock, floor, following, radius, token,
                        coverageDiagnostics, useIndexedCoverage, followingCache, validateIndexedCoverage, limitCropWork,
                        postedStepdown?.Capture());
                    capturedProofPrepareMilliseconds += Math.Max(0,
                        Stopwatch.GetElapsedTime(prepareStarted).TotalMilliseconds
                        - (MeasuredClearanceMilliseconds() - priorPrepareClearance));
                    bool safe = true;
                    for (int index = span.StartIndex + 1; index <= span.EndIndex; index++)
                    {
                        if (!proof.IsTravelSafe(path.Points[index - 1], path.Points[index], radius))
                        { safe = false; break; }
                    }
                    checkedLinks++;
                    candidates += proof.Queries;
                    if (safe)
                    {
                        RecordExecuted(path, span);
                        spanIndex = linkRunEnd;
                        continue;
                    }
                    long repairStarted = Stopwatch.GetTimestamp();
                    double priorRepairClearance = MeasuredClearanceMilliseconds();
                    if (++repairs > original.Spans.Count * 4 + 16)
                        throw new InvalidOperationException("Ordered pocket continuation repair did not make progress.");
                    var prefix = Slice(path, 0, span.StartIndex);
                    if (prefix.Spans.Any(part => part.MoveCount > 0)) output.Add(prefix);
                    Point3D from = path.Points[span.StartIndex];
                    if (cutIndex < 0)
                    {
                        lifts++;
                        Console.WriteLine($"[PocketContinuation.Temporal] event=terminal-lift region={path.RegionIndex} "
                            + $"pass={path.DepthPassIndex} span={spanIndex} floor={floor:R} constructionError={ConstructionError:R}");
                        path = Slice(path, span.EndIndex, span.EndIndex);
                        routeRepairMilliseconds += Math.Max(0,
                            Stopwatch.GetElapsedTime(repairStarted).TotalMilliseconds
                            - (MeasuredClearanceMilliseconds() - priorRepairClearance));
                        break;
                    }
                    var tail = Slice(path, path.Spans[cutIndex].StartIndex, path.Points.Count - 1);
                    if (TryRebase(tail, from, proof, radius, token, out var continued, out int tested,
                            options.PreserveCommandVertices))
                    {
                        candidates += tested;
                        rebased++;
                        path = continued;
                        Console.WriteLine($"[PocketContinuation.Temporal] event=rebase region={path.RegionIndex} "
                            + $"pass={path.DepthPassIndex} floor={floor:R} candidates={tested} "
                            + $"constructionError={ConstructionError:R} futureScope=first-cut");
                    }
                    else if (TryNativeRetrace(tail, from, proof, stock, radius, token,
                                 out continued, out int nativeQueries, out double retraceLength,
                                 options.PreserveCommandVertices))
                    {
                        candidates += tested + nativeQueries;
                        rebased++;
                        path = continued;
                        Console.WriteLine($"[PocketContinuation.Temporal] event=native-retrace region={path.RegionIndex} "
                            + $"pass={path.DepthPassIndex} floor={floor:R} walkLength={retraceLength:R} "
                            + $"refusedDirectLength={Math.Sqrt(DistanceSquared(from, tail.Points[0])):R} "
                            + $"candidates={nativeQueries} constructionError={ConstructionError:R} "
                            + $"futureScope=first-cut graphScope=earlier-native-floor-chords");
                    }
                    else
                    {
                        candidates += tested + nativeQueries;
                        double top = path.DepthPassIndex is { } pass ? depth.PassTopZ(pass) : depth.SurfaceZ;
                        RampStrategy strategy;
                        path = options.PreserveCommandVertices
                            ? AddEntryAtCommandVertices(tail, top, options.Ramp, nextVisit++, token, out strategy)
                            : AddEntry(tail, top, options.Ramp, nextVisit++, token, out strategy);
                        lifts++;
                        switch (strategy)
                        {
                            case RampStrategy.Helical: helicals++; break;
                            case RampStrategy.Linear: linear++; break;
                            default: plunges++; break;
                        }
                        Console.WriteLine($"[PocketContinuation.Temporal] event=reentry region={path.RegionIndex} "
                            + $"pass={path.DepthPassIndex} floor={floor:R} top={top:R} candidates={tested} "
                            + $"constructionError={ConstructionError:R} futureScope=first-cut");
                    }
                    routeRepairMilliseconds += Math.Max(0,
                        Stopwatch.GetElapsedTime(repairStarted).TotalMilliseconds
                        - (MeasuredClearanceMilliseconds() - priorRepairClearance));
                    finished = false;
                    break;
                }
            }
            if (path.Spans.Any(span => span.MoveCount > 0)) output.Add(path);
        }
        Console.WriteLine($"[PocketContinuation.Temporal] event=complete inputPaths={source.Count} paths={output.Count} "
            + $"checkedLinks={checkedLinks} rebased={rebased} lifts={lifts} candidates={candidates} "
            + $"recordedStrokes={stock.RecordedStrokes} floors={stock.FloorCount} floorQueries={stock.FloorQueries} "
            + $"replayedStrokes={stock.ReplayedStrokes} deferredFloorStrokes={stock.DeferredFloorStrokes} "
            + $"floorUnionBatches={stock.FloorUnionBatches} "
            + $"floorUnionCalls={stock.FloorUnionCalls} preparedBatches={stock.PreparedBatches} "
            + $"floorProjectionCacheHits={stock.FloorProjectionCacheHits} floorProjectionCacheMisses={stock.FloorProjectionCacheMisses} "
            + $"nativeChordIndexEnabled={stock.NativeChordIndexEnabled} nativeChordQueries={stock.NativeChordQueries} "
            + $"nativeChordCachedScanQueries={stock.NativeChordCachedScanQueries} nativeChordIndexedQueries={stock.NativeChordIndexedQueries} nativeChordIndexDeferredQueries={stock.NativeChordIndexDeferredQueries} "
            + $"nativeChordExamined={stock.NativeChordExamined} nativeChordNominated={stock.NativeChordNominated} nativeChordSelected={stock.NativeChordSelected} "
            + $"nativeChordProjectionCacheHits={stock.NativeChordProjectionCacheHits} nativeChordProjectionCacheMisses={stock.NativeChordProjectionCacheMisses} "
            + $"nativeChordIndexBuilds={stock.NativeChordIndexBuilds} nativeChordIndexNodeVisits={stock.NativeChordIndexNodeVisits} "
            + $"nativeChordQueryMs={stock.NativeChordQueryMilliseconds:F1} nativeChordIndexBuildMs={stock.NativeChordIndexBuildMilliseconds:F1} nativeChordTimingParent=routeRepairMs "
            + $"floorProjectionMs={stock.FloorProjectionMilliseconds:F1} floorSweepMs={stock.FloorSweepMilliseconds:F1} "
            + $"floorUnionMs={stock.FloorUnionMilliseconds:F1} "
            + $"capturedProofPrepareMs={capturedProofPrepareMilliseconds:F1} routeRepairMs={routeRepairMilliseconds:F1} "
            + $"executedRecordMs={executedRecordMilliseconds:F1} "
            + $"priorStepdownQueries={postedStepdown?.Queries ?? 0} "
            + $"priorStepdownSurfaceProofs={postedStepdown?.SurfaceProofs ?? 0} "
            + $"priorStepdownExactProofs={postedStepdown?.ExactProofs ?? 0} "
            + $"priorStepdownPhysicalProofs={postedStepdown?.PhysicalProofs ?? 0} "
            + $"priorStepdownRefusals={postedStepdown?.Refusals ?? 0} "
            + $"priorStepdownCandidateChords={postedStepdown?.CandidateChords ?? 0} "
            + $"priorStepdownIndexBuilds={postedStepdown?.InnerProofBuilds ?? 0} "
            + $"priorStepdownMs={postedStepdown?.ProofMilliseconds ?? 0:F1} "
            + $"postedRecordMs={postedStepdown?.RecordMilliseconds ?? 0:F1} "
            + $"priorStepdownPhysicalCylinder={flatCylinder} priorStepdownPhysicalRadius={physicalRadius:R} "
            + $"priorStepdownRequiredCylinderHeight={RequiredPostedCylinderHeight(depth):R} "
            + $"priorStepdownExactRectangleQueries={postedStepdown?.ExactRectangleQueries ?? 0} "
            + $"priorStepdownExactRectangleProofs={postedStepdown?.ExactRectangleProofs ?? 0} "
            + $"priorStepdownExactRectangleTriangles={postedStepdown?.ExactRectangleTriangles ?? 0} "
            + $"priorStepdownExactRectangleDistanceChecks={postedStepdown?.ExactRectangleDistanceChecks ?? 0} "
            + $"priorStepdownExactRectangleRefinements={postedStepdown?.ExactRectangleRefinements ?? 0} "
            + $"priorStepdownExactRectangleBudgetRefusals={postedStepdown?.ExactRectangleBudgetRefusals ?? 0} "
            + $"priorStepdownExactRectangleMs={postedStepdown?.ExactRectangleMilliseconds ?? 0:F1} "
            + "priorStepdownExactRectangleTimingParent=priorStepdownMs "
            + $"priorStepdownExactRectangleNegativeCandidates={postedStepdown?.ExactRectangleNegativeCandidates ?? 0} "
            + $"priorStepdownExactRectangleNegativeProbes={postedStepdown?.ExactRectangleNegativeProbes ?? 0} "
            + $"priorStepdownExactRectangleNegativeWitnesses={postedStepdown?.ExactRectangleNegativeWitnesses ?? 0} "
            + $"priorStepdownExactRectangleNegativeDistanceChecks={postedStepdown?.ExactRectangleNegativeDistanceChecks ?? 0} "
            + $"priorStepdownExactRectangleNegativeBudgetRefusals={postedStepdown?.ExactRectangleNegativeBudgetRefusals ?? 0} "
            + $"priorStepdownExactRectangleNegativePreparedVertices={postedStepdown?.ExactRectangleNegativePreparedVertices ?? 0} "
            + $"priorStepdownExactRectangleNegativeAxisProofs={postedStepdown?.ExactRectangleNegativeAxisProofs ?? 0} "
            + $"priorStepdownExactRectangleNegativeFullDistanceChecks={postedStepdown?.ExactRectangleNegativeFullDistanceChecks ?? 0} "
            + $"priorStepdownExactRectangleNegativePreparedEnabled={postedStepdown?.PreparedNegativeArithmeticEnabled ?? false} "
            + $"priorStepdownExactRectangleNegativeMs={postedStepdown?.ExactRectangleNegativeMilliseconds ?? 0:F1} "
            + "priorStepdownExactRectangleNegativeTimingParent=priorStepdownExactRectangleMs "
            + "priorStepdownExactRectangleNegativeAuthority=selected-capsule-subset-only "
            + $"priorStepdownMemoHits={postedStepdown?.ProofMemoHits ?? 0} "
            + $"priorStepdownMemoBypasses={postedStepdown?.ProofMemoBypasses ?? 0} "
            + $"priorStepdownMemoEvictions={postedStepdown?.ProofMemoEvictions ?? 0} "
            + $"priorStepdownMemoPeakEntries={postedStepdown?.ProofMemoPeakEntries ?? 0} "
            + "priorStepdownMemoLimitPerCapturedPrefix=128 priorStepdownMemoPolicy=bounded-LRU "
            + $"priorStepdownCachedInnerQueries={postedStepdown?.CachedInnerQueries ?? 0} "
            + $"priorStepdownCachedInnerProofs={postedStepdown?.CachedInnerProofs ?? 0} "
            + $"priorStepdownCachedInnerMisses={postedStepdown?.CachedInnerMisses ?? 0} "
            + $"priorStepdownInnerCapsuleQueries={postedStepdown?.InnerCapsuleQueries ?? 0} "
            + $"priorStepdownInnerPrepareMs={postedStepdown?.InnerPrepareMilliseconds ?? 0:F1} "
            + $"priorStepdownInnerCapsuleMs={postedStepdown?.InnerCapsuleMilliseconds ?? 0:F1} "
            + "priorStepdownProofChildrenTimingParent=priorStepdownMs "
            + $"priorStepdownInnerFullChords={postedStepdown?.InnerFullInputChords ?? 0} "
            + $"priorStepdownInnerUniqueChords={postedStepdown?.InnerUniqueFullChords ?? 0} "
            + $"priorStepdownInnerDuplicateChords={postedStepdown?.InnerDuplicateChords ?? 0} "
            + $"priorStepdownInnerJoinedPaths={postedStepdown?.InnerJoinedPaths ?? 0} "
            + $"priorStepdownInnerJoinedPoints={postedStepdown?.InnerJoinedPoints ?? 0} "
            + $"priorStepdownInnerClosedCycles={postedStepdown?.InnerClosedCycles ?? 0} "
            + $"priorStepdownInnerBranchVertices={postedStepdown?.InnerBranchVertices ?? 0} "
            + $"priorStepdownInnerPartialChords={postedStepdown?.InnerPartialChords ?? 0} "
            + $"priorStepdownHeightOnlyCursorCarries={postedStepdown?.HeightOnlyCursorCarries ?? 0} "
            + $"priorStepdownPlanarBridgeCursorAnchors={postedStepdown?.PlanarBridgeCursorAnchors ?? 0} "
            + "priorStepdownLateralAllowance=0 "
            + coverageDiagnostics.Describe()
            + $" totalMs={Stopwatch.GetElapsedTime(routeStarted).TotalMilliseconds:F1} "
            + $"allocatedBytes={GC.GetAllocatedBytesForCurrentThread() - allocationStarted} "
            + $"cropWorkBudgetEnabled={limitCropWork} "
            + $"constructionError={ConstructionError:R} depthPolicy=at-or-below-floor futureScope=first-cut");
        return new Result(output, lifts, plunges, linear, helicals, rebased, candidates);
    }

    internal static double RequiredPostedCylinderHeight(DepthSchedule depth)
    {
        // Qualification covers BOTH declared stock depth and the deepest actual
        // binary32/F3 tip pose. F3 may round a nominal final floor deeper.
        float bottom = (float)(depth.SurfaceZ - depth.Depth) - (float)depth.SurfaceZ;
        double posted = -Math.Round((double)bottom * 1000, MidpointRounding.ToEven) / 1000;
        return Math.Max(depth.Depth, posted);
    }

    internal static bool TryFlatCylinderRadius(ToolGeometry? tool, double stockDepth, out double radius)
    {
        radius = 0;
        if (tool is null) return false;
        if (tool.VCarveProfile is { } profile)
        {
            double physical = profile.Sections[0].BottomRadius;
            if (profile.RoundedTip || profile.MaximumDepth < stockDepth || physical <= 0
                || !profile.Sections.All(section => section.Kind == CarvingCutterSectionKind.Linear
                    && section.BottomRadius == physical && section.TopRadius == physical)) return false;
            radius = physical;
            return true;
        }
        bool flat = tool.TipAngleRadians is null && tool.BottomRadius is null && tool.TopRadius is null
            || tool.BottomRadius == tool.Radius && tool.TopRadius == tool.Radius;
        if (flat) radius = tool.Radius;
        return flat;
    }

    private static bool TryNativeRetrace(TaggedToolpath tail, Point3D from, TemporalClearance proof,
        ClearedStock stock, double radius, CancellationToken token, out TaggedToolpath continued,
        out int queries, out double length, bool commandVerticesOnly = false)
    {
        continued = tail;
        queries = 0;
        length = 0;
        if (tail.Spans.Count == 0 || tail.Spans[0].Kind != ToolpathSpanKind.Cut) return false;
        double budget = Math.Min(100 * radius, Math.Max(12 * radius,
            2 * Math.Sqrt(DistanceSquared(from, tail.Points[0])) + 4 * radius));
        long graphStarted = Stopwatch.GetTimestamp();
        bool usePointIndex = Environment.GetEnvironmentVariable("RW_TOOLPATHS_NATIVE_GRAPH_SPLIT_REFERENCE") != "1";
        var graph = new NativeFloorGraph(stock.NativeFloorChords(from.Z, from, budget, 8192), from, token,
            commandVerticesOnly, usePointIndex);
        Console.WriteLine($"[PocketContinuation.NativeGraph] nodes={graph.Points.Count} chords={graph.Edges.Count} "
            + $"pointIndexRequested={usePointIndex} pointIndexUsed={graph.SplitPointIndexUsed} "
            + $"predictedStripCandidates={graph.SplitEstimatedStripCandidates} indexBypasses={graph.SplitIndexBypasses} "
            + $"splitCandidates={graph.SplitCandidateVisits} splitNodeBounds={graph.SplitIndexNodeBounds} "
            + $"splitPointTests={graph.SplitIndexPointTests} splitQueries={graph.SplitIndexQueries} splitFallbacks={graph.SplitIndexFallbacks} "
            + $"totalMs={Stopwatch.GetElapsedTime(graphStarted).TotalMilliseconds:F3} timingParent=routeRepairMs "
            + $"floor={from.Z:R} budget={budget:R} chordLimit=8192");
        if (graph.Start < 0)
        {
            Console.WriteLine($"[PocketContinuation.Native] event=refused reason=start-not-in-bounded-earlier-chords "
                + $"from=({from.X:R},{from.Y:R},{from.Z:R}) nodes={graph.Points.Count} chords={graph.Edges.Count} "
                + $"chordLimit=8192 budget={budget:R}");
            return false;
        }
        int initialQueries = proof.Queries;
        graph.FindDistances(budget, proof, radius, token);
        var cut = tail.Spans[0];
        var targets = tail.Points.Take(cut.PointCount).ToList();
        bool closed = cut.MoveCount >= 2 && SamePosition(targets[0], targets[^1]);
        if (closed) targets.RemoveAt(targets.Count - 1);
        else targets = new List<Point3D> { targets[0], targets[^1] };
        int stride = Math.Max(1, (int)Math.Ceiling(targets.Count / 128d));
        var proposals = new List<(int Node, Point3D Waypoint, Point3D Target, double Cost)>();
        int postedCollinearProjections = 0, refusedPostedProjections = 0;
        for (int index = 0; index < targets.Count; index += stride)
        {
            token.ThrowIfCancellationRequested();
            Point3D target = targets[index];
            var best = new List<(int Node, Point3D Waypoint, Point3D Target, double Cost)>();
            var consideredVertices = commandVerticesOnly ? new HashSet<int>() : null;
            foreach (var edge in graph.Edges)
            {
                Point3D a = graph.Points[edge.A], b = graph.Points[edge.B];
                double denominator = DistanceSquared(a, b);
                double t = denominator == 0 ? 0 : Math.Clamp(
                    ((target.X - a.X) * (b.X - a.X) + (target.Y - a.Y) * (b.Y - a.Y)) / denominator, 0, 1);
                Point3D waypoint = Interpolate(a, b, t);
                if (commandVerticesOnly)
                {
                    // Both executed endpoints remain eligible. Choosing only
                    // the nearer endpoint could discard a reachable route when
                    // that endpoint's graph walk exceeds the search budget.
                    ConsiderVertex(edge.A);
                    ConsiderVertex(edge.B);
                    // A straight posted chord can also be divided at an exact
                    // F3-collinear point without adding cutter stock. This is
                    // required for continuous annular transfers onto long
                    // horizontal/vertical executed fronts. Proximity to the
                    // native chord alone is insufficient for an oblique seam.
                    if (!PostedCollinear(a, waypoint, b))
                    { refusedPostedProjections++; continue; }
                    postedCollinearProjections++;
                }
                double fromA = graph.Distances[edge.A] + Math.Sqrt(DistanceSquared(a, waypoint));
                double fromB = graph.Distances[edge.B] + Math.Sqrt(DistanceSquared(b, waypoint));
                int node = fromA <= fromB ? edge.A : edge.B;
                double cost = Math.Min(fromA, fromB) + Math.Sqrt(DistanceSquared(waypoint, target));
                if (cost > budget || cost <= 1e-9) continue;
                Consider(node, waypoint, cost);
            }
            proposals.AddRange(best);

            void ConsiderVertex(int node)
            {
                if (!consideredVertices!.Add(node)) return;
                Point3D waypoint = graph.Points[node];
                double cost = graph.Distances[node] + Math.Sqrt(DistanceSquared(waypoint, target));
                if (!double.IsFinite(cost) || cost > budget || cost <= 1e-9) return;
                Consider(node, waypoint, cost);
            }

            void Consider(int node, Point3D waypoint, double cost)
            {
                best.Add((node, waypoint, target, cost));
                best.Sort((left, right) =>
                {
                    int compare = DistanceSquared(left.Waypoint, target).CompareTo(DistanceSquared(right.Waypoint, target));
                    return compare != 0 ? compare : left.Cost.CompareTo(right.Cost);
                });
                if (best.Count > 4) best.RemoveAt(4);
            }
        }
        proposals.Sort((left, right) => left.Cost.CompareTo(right.Cost));
        int rebases = 0;
        foreach (var proposal in proposals.Take(512))
        {
            token.ThrowIfCancellationRequested();
            if (!proof.IsTravelSafe(graph.Points[proposal.Node], proposal.Waypoint, radius)
                || !proof.IsTravelSafe(proposal.Waypoint, proposal.Target, radius)) continue;
            var walk = graph.Walk(proposal.Node, from);
            if (!SamePosition(walk[^1], proposal.Waypoint)) walk.Add(proposal.Waypoint);
            bool valid = true;
            for (int index = 1; index < walk.Count; index++)
                if (!proof.IsTravelSafe(walk[index - 1], walk[index], radius)) { valid = false; break; }
            if (!valid) continue;
            if (++rebases > 64) break;
            if (!TryRebase(tail, proposal.Waypoint, proof, radius, token,
                    out var rebased, out _, commandVerticesOnly)) continue;
            int offset = walk.Count - 1;
            var points = walk.Take(offset).Concat(rebased.Points).ToList();
            var spans = Enumerable.Range(0, offset).Select(index =>
                new ToolpathSpan(index, index + 1, ToolpathSpanKind.Link)).ToList();
            spans.AddRange(rebased.Spans.Select(span =>
                new ToolpathSpan(span.StartIndex + offset, span.EndIndex + offset, span.Kind)));
            continued = Copy(tail, points, spans);
            length = Enumerable.Range(1, walk.Count - 1).Sum(index =>
                Math.Sqrt(DistanceSquared(walk[index - 1], walk[index])));
            queries = proof.Queries - initialQueries;
            if (commandVerticesOnly) Console.WriteLine($"[PocketContinuation.Native] event=posted-route "
                + $"originalEndpointCandidates=True postedCollinearProjections={postedCollinearProjections} "
                + $"refusedPostedProjections={refusedPostedProjections} addedCutLateralAllowance=0");
            return true;
        }
        queries = proof.Queries - initialQueries;
        Console.WriteLine($"[PocketContinuation.Native] event=refused reason=bounded-search-no-certified-projection "
            + $"from=({from.X:R},{from.Y:R},{from.Z:R}) target=({tail.Points[0].X:R},{tail.Points[0].Y:R}) "
            + $"nodes={graph.Points.Count} reachable={graph.Distances.Count(double.IsFinite)} chords={graph.Edges.Count} "
            + $"proposals={proposals.Count} queries={queries} budget={budget:R} "
            + $"distanceBudgetRefusals={graph.BudgetRefusals} commandVerticesOnly={commandVerticesOnly} "
            + $"postedCollinearProjections={postedCollinearProjections} refusedPostedProjections={refusedPostedProjections} "
            + $"chordLimit=8192 targetLimit=128 projectionLimit=512 rebaseLimit=64");
        return false;
    }

    private sealed class NativeFloorGraph
    {
        internal readonly record struct Edge(int A, int B, double Length);
        internal List<Point3D> Points { get; } = new();
        internal List<Edge> Edges { get; } = new();
        private readonly List<List<(int Node, int Edge)>> _neighbours = new();
        private readonly Dictionary<(long X, long Y), int> _nodes = new();
        private int[] _parents = Array.Empty<int>();
        internal double[] Distances { get; private set; } = Array.Empty<double>();
        internal int BudgetRefusals { get; private set; }
        internal int Start { get; }
        internal long SplitCandidateVisits { get; private set; }
        internal long SplitIndexNodeBounds => _splitIndexDiagnostics.RangeNodeBounds;
        internal long SplitIndexPointTests => _splitIndexDiagnostics.RangePointTests;
        internal long SplitIndexQueries => _splitIndexDiagnostics.RangeQueries;
        internal long SplitIndexFallbacks { get; private set; }
        internal bool SplitPointIndexUsed { get; }
        internal long SplitEstimatedStripCandidates { get; }
        internal long SplitIndexBypasses { get; }
        private readonly PreparedPointIndex.QueryDiagnostics _splitIndexDiagnostics = new();
        internal NativeFloorGraph(IEnumerable<(Point3D A, Point3D B)> chords, Point3D from, CancellationToken token,
            bool commandVerticesOnly = false, bool usePointIndex = true)
        {
            var seen = new HashSet<(int A, int B)>();
            foreach (var (a, b) in chords)
            {
                token.ThrowIfCancellationRequested();
                int start = Node(a), end = Node(b);
                if (start == end || !seen.Add((Math.Min(start, end), Math.Max(start, end)))) continue;
                int edge = Edges.Count;
                Edges.Add(new Edge(start, end, Math.Sqrt(DistanceSquared(a, b))));
                _neighbours[start].Add((end, edge));
                _neighbours[end].Add((start, edge));
            }
            // A prior rebase can end in the interior of an older native chord.
            // Split that chord at the already-emitted endpoint; otherwise the
            // graph invents a long backward walk to its old vertex before it
            // can use the very same cleared edge in the forward direction.
            var unsplit = Edges.ToArray();
            Edges.Clear();
            seen.Clear();
            foreach (var neighbours in _neighbours) neighbours.Clear();
            var ordered = Enumerable.Range(0, Points.Count).OrderBy(index => Points[index].X).ToArray();
            const double ticks = 4 / PathUtils.Scale;
            (int Low, int High)[]? stripRanges = null;
            PreparedPointIndex? pointIndex = null;
            int[]? ranks = null;
            if (usePointIndex && Points.All(point => double.IsFinite(point.X) && double.IsFinite(point.Y)))
            {
                stripRanges = new (int Low, int High)[unsplit.Length];
                long estimated = 0;
                for (int edgeIndex = 0; edgeIndex < unsplit.Length; edgeIndex++)
                {
                    token.ThrowIfCancellationRequested();
                    var edge = unsplit[edgeIndex];
                    double minX = Math.Min(Points[edge.A].X, Points[edge.B].X) - ticks;
                    double maxX = Math.Max(Points[edge.A].X, Points[edge.B].X) + ticks;
                    int low = LowerX(minX), high = UpperX(maxX, low);
                    stripRanges[edgeIndex] = (low, high);
                    estimated += high - low;
                }
                SplitEstimatedStripCandidates = estimated;
                int levels = 0;
                for (int remaining = Math.Max(2, Points.Count) - 1; remaining > 0; remaining >>= 1) levels++;
                // A sparse X strip is cheaper than building and traversing a
                // second index. This admission changes candidate discovery only.
                long crossover = 4L * ((long)Points.Count + unsplit.Length) * levels;
                if (estimated > crossover)
                {
                    pointIndex = new PreparedPointIndex(Points, token);
                    ranks = new int[ordered.Length];
                    for (int rank = 0; rank < ordered.Length; rank++) ranks[ordered[rank]] = rank;
                    SplitPointIndexUsed = true;
                }
                else SplitIndexBypasses = 1;
            }
            for (int edgeIndex = 0; edgeIndex < unsplit.Length; edgeIndex++)
            {
                token.ThrowIfCancellationRequested();
                var edge = unsplit[edgeIndex];
                Point3D a = Points[edge.A], b = Points[edge.B];
                double dx = b.X - a.X, dy = b.Y - a.Y, squared = dx * dx + dy * dy;
                double minX = Math.Min(a.X, b.X) - ticks, maxX = Math.Max(a.X, b.X) + ticks;
                double minY = Math.Min(a.Y, b.Y) - ticks, maxY = Math.Max(a.Y, b.Y) + ticks;
                var splits = new List<(int Node, double T)> { (edge.A, 0), (edge.B, 1) };
                if (pointIndex is { SupportsBounds: true } && double.IsFinite(minX) && double.IsFinite(maxX)
                    && double.IsFinite(minY) && double.IsFinite(maxY))
                {
                    var candidates = pointIndex.IntersectingOrdinals(minX, minY, maxX, maxY, token, _splitIndexDiagnostics);
                    // Restore the original stable X ordering before the unchanged
                    // split predicate and equal-parameter List.Sort tie handling.
                    candidates.Sort((left, right) => ranks![left].CompareTo(ranks[right]));
                    for (int cursor = 0; cursor < candidates.Count; cursor++)
                    {
                        if ((cursor & 255) == 0) token.ThrowIfCancellationRequested();
                        Consider(candidates[cursor]);
                    }
                }
                else
                {
                    if (usePointIndex && stripRanges is null) SplitIndexFallbacks++;
                    if (stripRanges is { } ranges)
                    {
                        var range = ranges[edgeIndex];
                        for (int cursor = range.Low; cursor < range.High; cursor++)
                        {
                            if ((cursor & 255) == 0) token.ThrowIfCancellationRequested();
                            Consider(ordered[cursor]);
                        }
                    }
                    else
                    {
                        int low = LowerX(minX);
                        for (int cursor = low; cursor < ordered.Length && Points[ordered[cursor]].X <= maxX; cursor++)
                        {
                            if ((cursor & 255) == 0) token.ThrowIfCancellationRequested();
                            Consider(ordered[cursor]);
                        }
                    }
                }
                void Consider(int node)
                {
                    SplitCandidateVisits++;
                    Point3D point = Points[node];
                    if (node == edge.A || node == edge.B || point.Y < minY || point.Y > maxY) return;
                    double t = ((point.X - a.X) * dx + (point.Y - a.Y) * dy) / squared;
                    if (t <= 0 || t >= 1) return;
                    double ex = point.X - a.X - t * dx, ey = point.Y - a.Y - t * dy;
                    if (ex * ex + ey * ey <= ticks * ticks
                        && (!commandVerticesOnly || PostedCollinear(a, point, b))) splits.Add((node, t));
                }
                splits.Sort((left, right) => left.T.CompareTo(right.T));
                for (int cursor = 1; cursor < splits.Count; cursor++)
                {
                    int start = splits[cursor - 1].Node, end = splits[cursor].Node;
                    if (start == end || !seen.Add((Math.Min(start, end), Math.Max(start, end)))) continue;
                    int index = Edges.Count;
                    Edges.Add(new Edge(start, end, Math.Sqrt(DistanceSquared(Points[start], Points[end]))));
                    _neighbours[start].Add((end, index));
                    _neighbours[end].Add((start, index));
                }
            }
            Start = _nodes.GetValueOrDefault(Key(from), -1);
            int LowerX(double minX)
            {
                int low = 0, high = ordered.Length;
                while (low < high)
                {
                    int middle = (low + high) / 2;
                    if (Points[ordered[middle]].X < minX) low = middle + 1; else high = middle;
                }
                return low;
            }
            int UpperX(double maxX, int low)
            {
                int high = ordered.Length;
                while (low < high)
                {
                    int middle = (low + high) / 2;
                    if (Points[ordered[middle]].X <= maxX) low = middle + 1; else high = middle;
                }
                return low;
            }
            int Node(Point3D point)
            {
                var key = Key(point);
                if (_nodes.TryGetValue(key, out int index)) return index;
                index = Points.Count;
                _nodes.Add(key, index);
                Points.Add(point);
                _neighbours.Add(new());
                return index;
            }
        }
        private static (long X, long Y) Key(Point3D point) =>
            ((long)Math.Round(point.X * PathUtils.Scale), (long)Math.Round(point.Y * PathUtils.Scale));
        internal void FindDistances(double budget, ILinkClearance proof, double radius, CancellationToken token)
        {
            Distances = Enumerable.Repeat(double.PositiveInfinity, Points.Count).ToArray();
            _parents = Enumerable.Repeat(-1, Points.Count).ToArray();
            var permitted = new bool?[Edges.Count];
            var queue = new PriorityQueue<int, double>();
            Distances[Start] = 0;
            queue.Enqueue(Start, 0);
            while (queue.TryDequeue(out int node, out double cost))
            {
                token.ThrowIfCancellationRequested();
                if (cost != Distances[node] || cost > budget) continue;
                foreach (var neighbour in _neighbours[node])
                {
                    Edge edge = Edges[neighbour.Edge];
                    double next = cost + edge.Length;
                    if (next > budget) { BudgetRefusals++; continue; }
                    if (next >= Distances[neighbour.Node]) continue;
                    permitted[neighbour.Edge] ??= proof.IsTravelSafe(Points[edge.A], Points[edge.B], radius);
                    if (permitted[neighbour.Edge] != true) continue;
                    Distances[neighbour.Node] = next;
                    _parents[neighbour.Node] = node;
                    queue.Enqueue(neighbour.Node, next);
                }
            }
        }
        internal List<Point3D> Walk(int destination, Point3D from)
        {
            var points = new List<Point3D>();
            for (int node = destination; node >= 0; node = _parents[node]) points.Add(Points[node]);
            points.Reverse();
            points[0] = from;
            return points;
        }
    }

    private static bool TryRebase(TaggedToolpath tail, Point3D from, ILinkClearance proof,
        double radius, CancellationToken token, out TaggedToolpath continued, out int tested,
        bool commandVerticesOnly = false)
    {
        continued = tail;
        tested = 0;
        if (tail.Spans.Count == 0 || tail.Spans[0].Kind != ToolpathSpanKind.Cut) return false;
        var cut = tail.Spans[0];
        double floor = tail.Points[cut.StartIndex].Z;
        if (Math.Abs(from.Z - floor) > 1e-9) return false;
        List<Point3D> points;
        List<ToolpathSpan> spans;
        bool closed = cut.MoveCount >= 2 && SamePosition(tail.Points[cut.StartIndex], tail.Points[cut.EndIndex]);
        if (closed)
        {
            var ring = tail.Points.Take(cut.PointCount).Select(point => new PointD(point.X, point.Y)).ToList();
            if (!PocketToolpaths.TryFindSafeRebase(ring, from, floor, proof, radius,
                    out var loop, out tested, commandVerticesOnly,localEntriesOnly:true)) return false;
            if (!RebuildRingTail(tail, cut, loop, floor, token, out points, out spans)) return false;
        }
        else
        {
            bool reverse = DistanceSquared(from, tail.Points[cut.EndIndex]) < DistanceSquared(from, tail.Points[0]);
            for (int attempt = 0; attempt < 2; attempt++, reverse = !reverse)
            {
                token.ThrowIfCancellationRequested();
                tested++;
                Point3D destination = reverse ? tail.Points[cut.EndIndex] : tail.Points[0];
                if (!proof.IsTravelSafe(from, destination, radius)) continue;
                points = tail.Points.Take(cut.PointCount).ToList();
                if (reverse) points.Reverse();
                spans = new List<ToolpathSpan> { new(0, points.Count - 1, ToolpathSpanKind.Cut) };
                // If later cuts depend on the original endpoint, retrace native
                // already-cut edges back there before preserving their directed tail.
                if (reverse)
                {
                    int start = points.Count - 1;
                    points.AddRange(tail.Points.Skip(1).Take(cut.EndIndex));
                    for (int index = start; index < points.Count - 1; index++)
                        spans.Add(new ToolpathSpan(index, index + 1, ToolpathSpanKind.Link));
                }
                AppendTail(tail, cut.EndIndex, points, spans);
                PrependLink(from, points, spans);
                continued = Copy(tail, points, spans);
                return true;
            }
            return false;
        }
        PrependLink(from, points, spans);
        continued = Copy(tail, points, spans);
        return true;
    }

    private static TaggedToolpath AddEntry(TaggedToolpath tail, double top, RampSettings settings,
        int visit, CancellationToken token, out RampStrategy strategy)
        => BuildEntry(tail, top, settings, visit, token, out strategy, commandVerticesOnly: false);

    private static TaggedToolpath AddEntryAtCommandVertices(TaggedToolpath tail, double top, RampSettings settings,
        int visit, CancellationToken token, out RampStrategy strategy)
        => BuildEntry(tail, top, settings, visit, token, out strategy, commandVerticesOnly: true);

    private static TaggedToolpath BuildEntry(TaggedToolpath tail, double top, RampSettings settings,
        int visit, CancellationToken token, out RampStrategy strategy, bool commandVerticesOnly)
    {
        var cut = tail.Spans[0];
        var native = tail.Points.Take(cut.PointCount).ToList();
        List<Point3D> points;
        List<ToolpathSpan> spans;
        if (settings.Strategy == RampStrategy.Plunge || settings.AngleRadians <= 0)
        {
            points = new List<Point3D> { new(native[0].X, native[0].Y, top) };
            points.AddRange(native);
            spans = new List<ToolpathSpan> { new(0, 1, ToolpathSpanKind.Ramp), new(1, points.Count - 1, ToolpathSpanKind.Cut) };
            strategy = RampStrategy.Plunge;
        }
        else
        {
            var entry = RampPlanner.PlanPathEntry(native, top, settings, token,
                message => Console.WriteLine($"[PocketContinuation.Entry] {message}"),
                landAtSourceVertex: commandVerticesOnly, preserveClosedExit: !commandVerticesOnly);
            points = entry.Points;
            spans = entry.Spans;
            strategy = entry.Strategy;
        }
        if (cut.EndIndex < tail.Points.Count - 1 && !SamePosition(points[^1], native[^1]))
        {
            // The new entry's one complete closed Cut finishes at its original
            // contact vertex. A dependent directed tail still starts at the old
            // seam. Return over immutable, now-executed source chords as Link,
            // preserving that seam without duplicating any part of the Cut.
            if (!SamePosition(native[0], native[^1]))
                throw new InvalidOperationException("An open pocket entry changed its dependent exit seam.");
            AppendNativeSeamReturn(points, spans, native.Take(native.Count - 1).ToList(),
                native[^1], tail.RegionIndex, tail.DepthPassIndex, token);
        }
        AppendTail(tail, cut.EndIndex, points, spans);
        return new TaggedToolpath(points, spans, tail.RegionIndex, tail.Category,
            tail.DepthPassIndex, visit, detectCurves: tail.DetectCurves);
    }

    private static bool RebuildRingTail(TaggedToolpath path, ToolpathSpan firstCut,
        IReadOnlyList<PointD> ring, double floor, CancellationToken token,
        out List<Point3D> points, out List<ToolpathSpan> spans)
    {
        points = ring.Select(point => new Point3D(point.x, point.y, floor)).ToList();
        spans = new List<ToolpathSpan>();
        if (ring.Count < 2) return false;
        points.Add(points[0]);
        spans.Add(new ToolpathSpan(0, points.Count - 1, ToolpathSpanKind.Cut));
        Point3D originalSeam = path.Points[firstCut.EndIndex];
        if (!points.Take(ring.Count).Any(point => SamePosition(point, originalSeam))) return false;
        AppendNativeSeamReturn(points, spans, points.Take(ring.Count).ToList(),
            originalSeam, path.RegionIndex, path.DepthPassIndex, token);
        AppendTail(path, firstCut.EndIndex, points, spans);
        return true;
    }

    private static void AppendNativeSeamReturn(List<Point3D> points, List<ToolpathSpan> spans,
        IReadOnlyList<Point3D> cycle, Point3D originalSeam, int region, int? pass, CancellationToken token)
    {
        int seamIndex = -1;
        int startIndex = -1;
        for (int index = 0; index < cycle.Count; index++)
        {
            token.ThrowIfCancellationRequested();
            if (SamePosition(cycle[index], originalSeam)) seamIndex = index;
            if (SamePosition(cycle[index], points[^1])) startIndex = index;
        }
        if (seamIndex < 0 || startIndex < 0)
            throw new InvalidOperationException("A pocket native seam return lost an original command vertex.");
        int retraceStart = points.Count - 1;
        double forward = 0, backward = 0;
        for (int index = startIndex; index != seamIndex;)
        {
            token.ThrowIfCancellationRequested();
            int next = (index + 1) % cycle.Count;
            forward += Math.Sqrt(DistanceSquared(cycle[index], cycle[next]));
            index = next;
        }
        for (int index = startIndex; index != seamIndex;)
        {
            token.ThrowIfCancellationRequested();
            int next = (index + cycle.Count - 1) % cycle.Count;
            backward += Math.Sqrt(DistanceSquared(cycle[index], cycle[next]));
            index = next;
        }
        int direction = forward <= backward ? 1 : -1;
        for (int index = startIndex; index != seamIndex;)
        {
            token.ThrowIfCancellationRequested();
            index = (index + cycle.Count + direction) % cycle.Count;
            points.Add(index == seamIndex ? originalSeam : cycle[index]);
        }
        if (seamIndex == startIndex) points[^1] = originalSeam;
        else Console.WriteLine($"[PocketContinuation.Seam] event=native-return region={region} "
            + $"pass={pass} forwardLength={forward:R} backwardLength={backward:R} "
            + $"selectedLength={Math.Min(forward, backward):R} cutDirectionPreserved=True originalEndpointPreserved=True");
        for (int index = retraceStart; index < points.Count - 1; index++)
            spans.Add(new ToolpathSpan(index, index + 1, ToolpathSpanKind.Link));
    }

    private static void AppendTail(TaggedToolpath source, int seam, List<Point3D> points, List<ToolpathSpan> spans)
    {
        int destinationSeam = points.Count - 1;
        points.AddRange(source.Points.Skip(seam + 1));
        foreach (var span in source.Spans)
            if (span.EndIndex > seam)
                spans.Add(new ToolpathSpan(destinationSeam + Math.Max(seam, span.StartIndex) - seam,
                    destinationSeam + span.EndIndex - seam, span.Kind));
    }

    private static void PrependLink(Point3D from, List<Point3D> points, List<ToolpathSpan> spans)
    {
        points.Insert(0, from);
        for (int index = 0; index < spans.Count; index++)
        {
            var span = spans[index];
            spans[index] = new ToolpathSpan(span.StartIndex + 1, span.EndIndex + 1, span.Kind);
        }
        spans.Insert(0, new ToolpathSpan(0, 1, ToolpathSpanKind.Link));
    }

    private static TaggedToolpath Slice(TaggedToolpath source, int first, int last)
    {
        var points = source.Points.Skip(first).Take(last - first + 1).ToList();
        var spans = source.Spans.Where(span => span.EndIndex > first && span.StartIndex < last)
            .Select(span => new ToolpathSpan(Math.Max(first, span.StartIndex) - first,
                Math.Min(last, span.EndIndex) - first, span.Kind)).ToList();
        return Copy(source, points, spans);
    }

    private static TaggedToolpath Copy(TaggedToolpath source, List<Point3D> points, List<ToolpathSpan> spans)
        => new(points, spans, source.RegionIndex, source.Category, source.DepthPassIndex,
            source.VisitIdentity, detectCurves: source.DetectCurves);
    private static int NextCut(TaggedToolpath path, int span)
    {
        for (int index = span + 1; index < path.Spans.Count; index++)
            if (path.Spans[index].Kind == ToolpathSpanKind.Cut && path.Spans[index].MoveCount > 0) return index;
        return -1;
    }
    private static bool IsFlat(TaggedToolpath path, ToolpathSpan span)
        => path.Points.Skip(span.StartIndex).Take(span.PointCount).All(point => Math.Abs(point.Z - path.Points[span.StartIndex].Z) <= 1e-9);
    private static double DistanceSquared(Point3D a, Point3D b) => Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2);
    private static bool SamePosition(Point3D a, Point3D b) => DistanceSquared(a, b) <= 1e-24;

    // An emitted native endpoint can join an earlier chord only when it is
    // exactly on that same float/F3 posted segment. Native proximity alone can
    // invent a new cutter-volume sliver when the graph splits the older edge.
    internal static bool PostedCollinear(Point3D start, Point3D middle, Point3D end)
    {
        static bool Grid(Point3D p, out long x, out long y)
        {
            float a = (float)p.X, b = (float)p.Y;
            if (!float.IsFinite(a) || !float.IsFinite(b) || Math.Abs(a) > 1e9 || Math.Abs(b) > 1e9)
            { x = y = 0; return false; }
            x = (long)Math.Round((double)a * 1000, MidpointRounding.ToEven);
            y = (long)Math.Round((double)b * 1000, MidpointRounding.ToEven);
            return true;
        }
        if (!Grid(start, out long ax, out long ay) || !Grid(middle, out long bx, out long by)
            || !Grid(end, out long cx, out long cy)) return false;
        Int128 dx = cx - ax, dy = cy - ay, mx = bx - ax, my = by - ay;
        if (dx == 0 && dy == 0) return mx == 0 && my == 0;
        if (mx * dy != my * dx) return false;
        Int128 dot = mx * dx + my * dy;
        return dot >= 0 && dot <= dx * dx + dy * dy;
    }

    internal static IReadOnlyList<IReadOnlyList<PointD>> BelowFloor(IReadOnlyList<Point3D> stroke, double floor)
    {
        var pieces = new List<IReadOnlyList<PointD>>();
        List<PointD>? piece = null;
        for (int index = 1; index < stroke.Count; index++)
        {
            Point3D a = stroke[index - 1], b = stroke[index];
            if (a.Z > floor + 1e-9 && b.Z > floor + 1e-9) { piece = null; continue; }
            if (a.Z > floor + 1e-9) a = Interpolate(a, b, (a.Z - floor) / (a.Z - b.Z));
            else if (b.Z > floor + 1e-9) b = Interpolate(a, b, (floor - a.Z) / (b.Z - a.Z));
            var start = new PointD(a.X, a.Y);
            var end = new PointD(b.X, b.Y);
            if (piece is null || piece[^1] != start)
            {
                piece = new List<PointD> { start, end };
                pieces.Add(piece);
            }
            else piece.Add(end);
        }
        return pieces;
    }
    private static Point3D Interpolate(Point3D a, Point3D b, double t)
        => new(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);

    internal sealed class ClearedStock
    {
        internal sealed class Floor
        {
            internal List<List<PointD>> Swept = new();
            internal List<IReadOnlyList<PointD>> Pending = new();
            internal int ConsumedStrokes;
            internal int MaterializedBatches;
            internal readonly List<CoverageBatch> Batches = new();
            internal readonly CoverageForest Forest = new();
            internal readonly List<IReadOnlyList<IReadOnlyList<PointD>>?> Projections = new();
            internal readonly NativeChordIndex Chords = new();
            internal int IndexedStrokes;
            internal bool HasNonFiniteProjection;
            internal int CachedProjectionStrokes;
            internal long ProjectedChordCount, NativeScanWork;
            internal bool NativeIndexStarted;
        }
        private readonly double _radius;
        private readonly CancellationToken _token;
        private readonly bool _useNativeChordIndex;
        private readonly CoverageBatch? _knownEmpty;
        private readonly double _knownEmptyFloor;
        private readonly List<IReadOnlyList<Point3D>> _strokes = new();
        private readonly Dictionary<double, Floor> _floors = new();
        internal int RecordedStrokes => _strokes.Count;
        internal int FloorCount => _floors.Count;
        internal int FloorQueries { get; private set; }
        internal long ReplayedStrokes { get; private set; }
        internal int FloorUnionBatches { get; private set; }
        internal int FloorUnionCalls { get; private set; }
        internal int PreparedBatches { get; private set; }
        internal double FloorProjectionMilliseconds { get; private set; }
        internal double FloorSweepMilliseconds { get; private set; }
        internal double FloorUnionMilliseconds { get; private set; }
        internal long FloorProjectionCacheHits { get; private set; }
        internal long FloorProjectionCacheMisses { get; private set; }
        internal long NativeChordQueries { get; private set; }
        internal long NativeChordExamined { get; private set; }
        internal long NativeChordNominated { get; private set; }
        internal long NativeChordSelected { get; private set; }
        internal long NativeChordProjectionCacheHits { get; private set; }
        internal long NativeChordProjectionCacheMisses { get; private set; }
        internal long NativeChordIndexBuilds { get; private set; }
        internal long NativeChordIndexNodeVisits { get; private set; }
        internal long NativeChordCachedScanQueries { get; private set; }
        internal long NativeChordIndexedQueries { get; private set; }
        internal long NativeChordIndexDeferredQueries { get; private set; }
        internal double NativeChordQueryMilliseconds { get; private set; }
        internal double NativeChordIndexBuildMilliseconds { get; private set; }
        internal bool NativeChordIndexEnabled => _useNativeChordIndex;
        internal long DeferredFloorStrokes => _floors.Values.Sum(floor => (long)_strokes.Count - floor.ConsumedStrokes);
        internal ClearedStock(double radius, CancellationToken token, bool useNativeChordIndex = true,
            IReadOnlyList<IReadOnlyList<PointD>>? knownEmpty=null,double knownEmptyFloor=double.PositiveInfinity)
        {
            _radius = radius; _token = token;
            _knownEmptyFloor=knownEmptyFloor;
            if(knownEmpty is {Count:>0}) _knownEmpty=new CoverageBatch(knownEmpty.Select(p=>p.ToList()).ToList(),0);
            _useNativeChordIndex = useNativeChordIndex
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_NATIVE_CHORD_REFERENCE") != "1";
        }
        internal void Record(IReadOnlyList<Point3D> stroke)
        {
            _token.ThrowIfCancellationRequested();
            _strokes.Add(stroke);
        }
        internal IReadOnlyList<List<PointD>> AtFloor(double z)
            => PrepareAtFloor(z).Materialize();

        // Preparing the executed prefix constructs each new physical sweep once.
        // The original insertion-ordered global unions remain lazy diagnostics /
        // fallback authority; local queries never credit unexecuted strokes.
        internal FloorSnapshot PrepareAtFloor(double z)
        {
            _token.ThrowIfCancellationRequested();
            FloorQueries++;
            if (!_floors.TryGetValue(z, out var floor))
            {
                floor = new Floor();
                if(_knownEmpty!=null && z>=_knownEmptyFloor)
                {
                    floor.Batches.Add(_knownEmpty);floor.Forest.Add(_knownEmpty);PreparedBatches++;
                    Console.WriteLine($"[Pocket.KnownEmpty] floor={z:R} minimumFloor={_knownEmptyFloor:R} "
                        + $"authority=explicit-outside-edge-permission syntheticRecordedStrokes=0 actualRecordedStrokes={_strokes.Count} futureCutCredit=False");
                }
                _floors.Add(z, floor);
            }
            long projectionStarted = Stopwatch.GetTimestamp();
            while (floor.ConsumedStrokes < _strokes.Count)
            {
                _token.ThrowIfCancellationRequested();
                floor.Pending.AddRange(_useNativeChordIndex
                    ? Projected(floor, floor.ConsumedStrokes, z, nativeQuery: false)
                    : BelowFloor(_strokes[floor.ConsumedStrokes], z));
                floor.ConsumedStrokes++;
                ReplayedStrokes++;
            }
            FloorProjectionMilliseconds += Stopwatch.GetElapsedTime(projectionStarted).TotalMilliseconds;
            if (floor.Pending.Count > 0)
            {
                long sweepStarted = Stopwatch.GetTimestamp();
                var added = PolygonOps.BufferOpenCenterlines(floor.Pending, _radius, ArcTolerance, _token);
                var batch = new CoverageBatch(added, floor.Batches.Count);
                floor.Batches.Add(batch); floor.Forest.Add(batch); PreparedBatches++;
                FloorSweepMilliseconds += Stopwatch.GetElapsedTime(sweepStarted).TotalMilliseconds;
                floor.Pending.Clear();
                FloorUnionBatches++;
            }
            _token.ThrowIfCancellationRequested();
            return new FloorSnapshot(this, floor);
        }

        internal sealed class FloorSnapshot
        {
            private readonly ClearedStock _owner;
            private readonly Floor _floor;
            private readonly IReadOnlyList<List<PointD>> _startingSwept;
            private readonly int _startingMaterialized;
            internal int PrefixBatches { get; }
            internal FloorSnapshot(ClearedStock owner, Floor floor)
            {
                _owner = owner; _floor = floor; PrefixBatches = floor.Batches.Count;
                _startingSwept = floor.Swept; _startingMaterialized = floor.MaterializedBatches;
            }

            internal IReadOnlyList<CoverageBatch> Intersecting(CoverageBounds bounds, CoverageDiagnostics diagnostics)
                => _floor.Forest.Intersecting(bounds, PrefixBatches, _owner._token, diagnostics);

            internal IReadOnlyList<List<PointD>> Materialize()
            {
                _owner._token.ThrowIfCancellationRequested();
                long started = Stopwatch.GetTimestamp();
                if (_floor.MaterializedBatches > PrefixBatches)
                {
                    // Other snapshot consumers may have materialized a later
                    // prefix. Reconstruct this captured prefix from its immutable
                    // starting union; later stock must never enter its authority.
                    var snapshot = _startingSwept;
                    for (int i = _startingMaterialized; i < PrefixBatches; i++)
                    {
                        _owner._token.ThrowIfCancellationRequested();
                        snapshot = PolygonOps.Union(snapshot.Concat(_floor.Batches[i].Paths));
                        _owner.FloorUnionCalls++;
                    }
                    _owner.FloorUnionMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                    return snapshot;
                }
                while (_floor.MaterializedBatches < PrefixBatches)
                {
                    _owner._token.ThrowIfCancellationRequested();
                    var added = _floor.Batches[_floor.MaterializedBatches++];
                    _floor.Swept = PolygonOps.Union(_floor.Swept.Concat(added.Paths));
                    _owner.FloorUnionCalls++;
                }
                _owner.FloorUnionMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                return _floor.Swept;
            }
        }
        private IReadOnlyList<IReadOnlyList<PointD>> Projected(Floor floor, int stroke, double z,
            bool nativeQuery)
        {
            _token.ThrowIfCancellationRequested();
            while (floor.Projections.Count <= stroke) floor.Projections.Add(null);
            if (floor.Projections[stroke] is { } cached)
            {
                FloorProjectionCacheHits++;
                if (nativeQuery) NativeChordProjectionCacheHits++;
                return cached;
            }
            // RecordExecuted supplies detached, operation-owned strokes. Keep
            // projected pieces immutable and independent of sweep consumption;
            // inspecting retrace candidates never changes a Boolean flush.
            var pieces = BelowFloor(_strokes[stroke], z).Select(piece => (IReadOnlyList<PointD>)piece.ToArray()).ToArray();
            _token.ThrowIfCancellationRequested();
            if (pieces.Any(piece => piece.Any(point => !double.IsFinite(point.x) || !double.IsFinite(point.y))))
                floor.HasNonFiniteProjection = true;
            floor.Projections[stroke] = pieces;
            floor.CachedProjectionStrokes++;
            floor.ProjectedChordCount += pieces.Sum(piece => (long)Math.Max(0, piece.Count - 1));
            FloorProjectionCacheMisses++;
            if (nativeQuery) NativeChordProjectionCacheMisses++;
            return pieces;
        }

        internal readonly record struct NativeChord(PointD A, PointD B, int Stroke, int Piece, int Chord)
        {
            internal CoverageBounds Bounds => double.IsFinite(A.x) && double.IsFinite(A.y)
                && double.IsFinite(B.x) && double.IsFinite(B.y)
                ? new(Math.Min(A.x, B.x), Math.Min(A.y, B.y), Math.Max(A.x, B.x), Math.Max(A.y, B.y))
                : new(double.NegativeInfinity, double.NegativeInfinity, double.PositiveInfinity, double.PositiveInfinity);
        }

        // Fixed-size segment BVHs avoid scanning old strokes. Complete stroke
        // roots append to the same binomial hierarchy pattern used by coverage.
        // Keep newest strokes first and ascending chunks inside a long stroke;
        // only one bounded chunk needs ordering before an early query cap.
        internal sealed class NativeChordIndex
        {
            private const int BatchSize = 512;
            private const int LeafSize = 12;
            private sealed record Node(CoverageBounds Bounds, NativeChord[]? Chords = null,
                int Start = 0, int Count = 0, Node? First = null, Node? Second = null);
            private sealed record StrokeNode(CoverageBounds Bounds, Node[]? Batches = null,
                StrokeNode? First = null, StrokeNode? Second = null);
            private readonly List<StrokeNode?> _roots = new();

            internal void AddStroke(IReadOnlyList<IReadOnlyList<PointD>> pieces, int stroke,
                ClearedStock owner)
            {
                int capacity = 0;
                foreach (var piece in pieces)
                    capacity = Math.Min(BatchSize, capacity + Math.Min(BatchSize, Math.Max(0, piece.Count - 1)));
                if (capacity == 0) return;
                long started = Stopwatch.GetTimestamp();
                var pending = new List<NativeChord>(capacity);
                var batches = new List<Node>();
                for (int piece = 0; piece < pieces.Count; piece++)
                for (int chord = 1; chord < pieces[piece].Count; chord++)
                {
                    owner._token.ThrowIfCancellationRequested();
                    pending.Add(new(pieces[piece][chord - 1], pieces[piece][chord], stroke, piece, chord));
                    if (pending.Count == BatchSize) Flush();
                }
                if (pending.Count > 0) Flush();
                if (batches.Count == 0) return;
                var bounds = batches[0].Bounds;
                for (int index = 1; index < batches.Count; index++)
                    bounds = CoverageBounds.Combine(bounds, batches[index].Bounds);
                var node = new StrokeNode(bounds, batches.ToArray());
                int level = 0;
                while (level < _roots.Count && _roots[level] is { } prior)
                {
                    node = new(CoverageBounds.Combine(prior.Bounds, node.Bounds), First: prior, Second: node);
                    _roots[level++] = null;
                }
                if (level == _roots.Count) _roots.Add(node); else _roots[level] = node;
                owner.NativeChordIndexBuildMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;

                void Flush()
                {
                    var chords = pending.ToArray();
                    batches.Add(Build(chords, 0, chords.Length, owner._token));
                    pending.Clear();
                    owner.NativeChordIndexBuilds++;
                }
            }

            private static Node Build(NativeChord[] chords, int start, int count, CancellationToken token)
            {
                token.ThrowIfCancellationRequested();
                var bounds = chords[start].Bounds;
                for (int index = start + 1; index < start + count; index++)
                    bounds = CoverageBounds.Combine(bounds, chords[index].Bounds);
                if (count <= LeafSize) return new(bounds, chords, start, count);
                bool splitX = bounds.MaxX - bounds.MinX >= bounds.MaxY - bounds.MinY;
                Array.Sort(chords, start, count, Comparer<NativeChord>.Create((first, second) =>
                {
                    var a = first.Bounds; var b = second.Bounds;
                    int compare = (splitX ? a.MinX + a.MaxX : a.MinY + a.MaxY)
                        .CompareTo(splitX ? b.MinX + b.MaxX : b.MinY + b.MaxY);
                    return compare != 0 ? compare : CompareOrder(first, second);
                }));
                int firstCount = count / 2;
                return new(bounds, First: Build(chords, start, firstCount, token),
                    Second: Build(chords, start + firstCount, count - firstCount, token));
            }

            internal IEnumerable<NativeChord> Intersecting(CoverageBounds bounds, ClearedStock owner)
            {
                // A low-level root is the newest chronological block. Descend
                // newer children first, but keep chunks within ONE stroke in
                // ascending order. Long strokes may span several spatial BVHs.
                foreach (var root in _roots)
                    if (root is not null)
                        foreach (var chord in VisitStroke(root)) yield return chord;

                IEnumerable<NativeChord> VisitStroke(StrokeNode node)
                {
                    owner._token.ThrowIfCancellationRequested();
                    owner.NativeChordIndexNodeVisits++;
                    if (!node.Bounds.Intersects(bounds)) yield break;
                    if (node.Batches is { } batches)
                    {
                        foreach (var batch in batches)
                        {
                            var result = new List<NativeChord>();
                            Visit(batch, result);
                            result.Sort(CompareOrder);
                            foreach (var chord in result) yield return chord;
                        }
                    }
                    else
                    {
                        foreach (var chord in VisitStroke(node.Second!)) yield return chord;
                        foreach (var chord in VisitStroke(node.First!)) yield return chord;
                    }
                }

                void Consider(NativeChord chord, List<NativeChord> result)
                {
                    owner._token.ThrowIfCancellationRequested();
                    owner.NativeChordExamined++;
                    if (!chord.Bounds.Intersects(bounds)) return;
                    owner.NativeChordNominated++;
                    result.Add(chord);
                }
                void Visit(Node node, List<NativeChord> result)
                {
                    owner._token.ThrowIfCancellationRequested();
                    owner.NativeChordIndexNodeVisits++;
                    if (!node.Bounds.Intersects(bounds)) return;
                    if (node.Chords is { } chords)
                    {
                        for (int index = node.Start; index < node.Start + node.Count; index++) Consider(chords[index], result);
                    }
                    else { Visit(node.First!, result); Visit(node.Second!, result); }
                }
            }

            private static int CompareOrder(NativeChord first, NativeChord second)
            {
                int compare = second.Stroke.CompareTo(first.Stroke);
                if (compare != 0) return compare;
                compare = first.Piece.CompareTo(second.Piece);
                return compare != 0 ? compare : first.Chord.CompareTo(second.Chord);
            }
        }

        internal IEnumerable<(Point3D A, Point3D B)> NativeFloorChords(double z, Point3D from,
            double budget, int limit)
        {
            _token.ThrowIfCancellationRequested();
            NativeChordQueries++;
            var bounds = default(CoverageBounds);
            Floor? queryFloor = null;
            if (_useNativeChordIndex && !_floors.TryGetValue(z, out queryFloor))
                _floors.Add(z, queryFloor = new Floor());
            bool usableBounds = _useNativeChordIndex && double.IsFinite(z) && TryNativeQueryBounds(from, budget, out bounds);
            // Pay for an index only after observed repeated scans exceed four
            // complete cached floor inputs. A cap may inspect only a tiny newest
            // subset; it must not cause untouched historical projections to be
            // populated just to build an index. Once started, append normally.
            bool deferred = _useNativeChordIndex && usableBounds && !queryFloor!.HasNonFiniteProjection
                && !queryFloor.NativeIndexStarted && (queryFloor.ProjectedChordCount == 0
                    || queryFloor.CachedProjectionStrokes != _strokes.Count
                    || queryFloor.NativeScanWork <= 4 * queryFloor.ProjectedChordCount);
            if (!_useNativeChordIndex || !usableBounds || queryFloor!.HasNonFiniteProjection || deferred)
            {
                if (_useNativeChordIndex) NativeChordCachedScanQueries++;
                if (deferred) NativeChordIndexDeferredQueries++;
                using var reference = NativeFloorChordSequence(z, from, budget, limit, _useNativeChordIndex).GetEnumerator();
                while (true)
                {
                    long started = Stopwatch.GetTimestamp();
                    bool next;
                    try { next = reference.MoveNext(); }
                    finally { NativeChordQueryMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds; }
                    if (!next) yield break;
                    yield return reference.Current;
                }
            }
            long queryStarted = Stopwatch.GetTimestamp();
            List<(Point3D A, Point3D B)> selected;
            try
            {
                var floor = queryFloor!;
                int prefix = _strokes.Count;
                while (!floor.HasNonFiniteProjection && floor.IndexedStrokes < prefix)
                {
                    int stroke = floor.IndexedStrokes;
                    var pieces = Projected(floor, stroke, z, nativeQuery: true);
                    if (floor.HasNonFiniteProjection) break;
                    floor.Chords.AddStroke(pieces, stroke, this);
                    floor.IndexedStrokes++;
                }
                if (floor.HasNonFiniteProjection)
                {
                    NativeChordCachedScanQueries++;
                    selected = NativeFloorChordSequence(z, from, budget, limit, reuseProjections: true).ToList();
                }
                else
                {
                    floor.NativeIndexStarted = true;
                    NativeChordIndexedQueries++;
                    selected = new();
                    foreach (var chord in floor.Chords.Intersecting(bounds, this))
                    {
                        _token.ThrowIfCancellationRequested();
                        var a = new Point3D(chord.A.x, chord.A.y, z);
                        var b = new Point3D(chord.B.x, chord.B.y, z);
                        if (DistanceSquared(from, a) > budget * budget && DistanceSquared(from, b) > budget * budget) continue;
                        if (selected.Count >= limit) break;
                        selected.Add((a, b));
                        NativeChordSelected++;
                    }
                }
            }
            finally { NativeChordQueryMilliseconds += Stopwatch.GetElapsedTime(queryStarted).TotalMilliseconds; }
            foreach (var chord in selected)
            {
                _token.ThrowIfCancellationRequested();
                yield return chord;
            }
        }

        private static bool TryNativeQueryBounds(Point3D from, double budget, out CoverageBounds bounds)
        {
            bounds = default;
            double squaredBudget = budget * budget;
            if (!double.IsFinite(from.X) || !double.IsFinite(from.Y) || !double.IsFinite(squaredBudget)
                || squaredBudget == 0) return false;
            // These are candidate bounds only. Round outward at both the squared
            // threshold and coordinate arithmetic; the original endpoint test
            // still makes every decision, including exact radius tangencies.
            double radius = Math.BitIncrement(Math.Sqrt(Math.BitIncrement(squaredBudget)));
            bounds = new(Math.BitDecrement(from.X - radius), Math.BitDecrement(from.Y - radius),
                Math.BitIncrement(from.X + radius), Math.BitIncrement(from.Y + radius));
            return double.IsFinite(bounds.MinX) && double.IsFinite(bounds.MinY)
                && double.IsFinite(bounds.MaxX) && double.IsFinite(bounds.MaxY);
        }

        private IEnumerable<(Point3D A, Point3D B)> NativeFloorChordSequence(double z, Point3D from,
            double budget, int limit, bool reuseProjections)
        {
            int count = 0;
            Floor? floor = null;
            if (reuseProjections && !_floors.TryGetValue(z, out floor)) _floors.Add(z, floor = new Floor());
            for (int stroke = _strokes.Count - 1; stroke >= 0; stroke--)
            {
                IReadOnlyList<IReadOnlyList<PointD>> pieces;
                if (reuseProjections) pieces = Projected(floor!, stroke, z, nativeQuery: true);
                else { NativeChordProjectionCacheMisses++; pieces = BelowFloor(_strokes[stroke], z); }
                foreach (var piece in pieces)
                for (int index = 1; index < piece.Count; index++)
                {
                    _token.ThrowIfCancellationRequested();
                    NativeChordExamined++;
                    if (reuseProjections) floor!.NativeScanWork++;
                    var a = new Point3D(piece[index - 1].x, piece[index - 1].y, z);
                    var b = new Point3D(piece[index].x, piece[index].y, z);
                    if (DistanceSquared(from, a) > budget * budget && DistanceSquared(from, b) > budget * budget) continue;
                    NativeChordNominated++;
                    if (++count > limit) yield break;
                    NativeChordSelected++;
                    yield return (a, b);
                }
            }
        }
    }

    internal readonly record struct CoverageBounds(double MinX, double MinY, double MaxX, double MaxY)
    {
        internal bool Intersects(CoverageBounds other) => MaxX >= other.MinX && MinX <= other.MaxX
            && MaxY >= other.MinY && MinY <= other.MaxY;
        internal bool Contains(CoverageBounds other) => MinX <= other.MinX && MinY <= other.MinY
            && MaxX >= other.MaxX && MaxY >= other.MaxY;
        internal static CoverageBounds Around(Point3D a, Point3D b, double radius)
            => new(Math.Min(a.X, b.X) - radius, Math.Min(a.Y, b.Y) - radius,
                Math.Max(a.X, b.X) + radius, Math.Max(a.Y, b.Y) + radius);
        internal static CoverageBounds Combine(CoverageBounds a, CoverageBounds b)
            => new(Math.Min(a.MinX, b.MinX), Math.Min(a.MinY, b.MinY),
                Math.Max(a.MaxX, b.MaxX), Math.Max(a.MaxY, b.MaxY));
    }

    internal sealed class CoverageDiagnostics
    {
        internal long LocalProofs, WholeFallbacks, UndilatedProofs, NegativeWitnesses, NegativeQueries, BatchNodeVisits, BatchCandidates, BatchIndexBuilds,
            LocalIndexBuilds, LocalInputVertices, LocalOutputVertices,
            FollowingCacheHits, FollowingCacheMisses, FollowingCacheEvictions, FollowingCacheKeyVertices,
            FollowingCacheEstimatedBytes, FollowingCachePeakEstimatedBytes, FollowingCacheBypasses,
            ValidatedLocalProofs, LocalProofMismatches, RawBatchProofs, CroppedDomainProofs,
            HistoricalReserveExhausted, CropGuardRefusals, CapturedUnionBuilds,
            ContourBoundsBuilds, ContourBoundsQueries, ContourBoundsCandidates,
            ContourBoundsInputVertices, ContourBoundsConsideredVertices, ContourBoundsSubmittedVertices,
            CropWorkBudgetFallbacks, CropWorkBudgetPriorBuilds, CropWorkBudgetPriorSubmittedVertices,
            CropWorkBudgetRefusedVertices, CoverageTileSnaps, CoverageTileReuses,
            CoverageTileSmallDomainSkips, CoverageTileRepeatedQualifications, CoverageTileRangeRefusals,
            CoverageTileSubmittedVertices, LocalTileCacheHits,LocalTileCacheMisses,LocalTileCacheEvictions,LocalTileCachePeakEstimatedBytes;
        internal double FollowingSweepMilliseconds, LocalConstructionMilliseconds, NegativeWitnessMilliseconds,
            WholeConstructionMilliseconds, SpatialCheckMilliseconds, CapsuleCheckMilliseconds,
            FollowingCacheLookupMilliseconds, ValidationConstructionMilliseconds, ValidationCapsuleMilliseconds,
            CapturedUnionMilliseconds, CropMilliseconds, LocalBufferMilliseconds, LocalIndexMilliseconds,
            WholeBufferMilliseconds, WholeIndexMilliseconds, ContourBoundsBuildMilliseconds, ContourBoundsQueryMilliseconds,
            CoverageTileBoundsMilliseconds, CoverageTileTightArea, CoverageTileCropArea;
        // Detailed buffer/index/bounds counters are children of these owners.
        // Cache lookup already subtracts following sweep construction. Route
        // preparation and repair may subtract this sum to expose exclusive work.
        internal double ExclusiveMilliseconds => FollowingSweepMilliseconds + LocalConstructionMilliseconds
            + WholeConstructionMilliseconds + SpatialCheckMilliseconds + CapsuleCheckMilliseconds
            + FollowingCacheLookupMilliseconds + ValidationConstructionMilliseconds + ValidationCapsuleMilliseconds
            + CapturedUnionMilliseconds + NegativeWitnessMilliseconds;
        internal string Describe() => $"localCoverageProofs={LocalProofs} wholeCoverageFallbacks={WholeFallbacks} "
            + $"undilatedCapsuleProofs={UndilatedProofs} "
            + $"uncoveredWitnesses={NegativeWitnesses} uncoveredWitnessQueries={NegativeQueries} uncoveredWitnessMs={NegativeWitnessMilliseconds:F1} "
            + $"coverageBatchNodeVisits={BatchNodeVisits} coverageBatchCandidates={BatchCandidates} "
            + $"coverageBatchIndexBuilds={BatchIndexBuilds} localCoverageIndexBuilds={LocalIndexBuilds} "
            + $"localCoverageInputVertices={LocalInputVertices} localCoverageOutputVertices={LocalOutputVertices} "
            + $"rawBatchProofs={RawBatchProofs} croppedDomainProofs={CroppedDomainProofs} "
            + $"historicalReserveExhausted={HistoricalReserveExhausted} cropGuardRefusals={CropGuardRefusals} "
            + $"capturedUnionBuilds={CapturedUnionBuilds} capturedUnionMs={CapturedUnionMilliseconds:F1} "
            + $"contourBoundsBuilds={ContourBoundsBuilds} contourBoundsQueries={ContourBoundsQueries} contourBoundsCandidates={ContourBoundsCandidates} "
            + $"contourBoundsInputVertices={ContourBoundsInputVertices} contourBoundsConsideredVertices={ContourBoundsConsideredVertices} "
            + $"contourBoundsSubmittedVertices={ContourBoundsSubmittedVertices} "
            + $"cropWorkBudgetFallbacks={CropWorkBudgetFallbacks} cropWorkBudgetPriorBuilds={CropWorkBudgetPriorBuilds} "
            + $"cropWorkBudgetPriorSubmittedVertices={CropWorkBudgetPriorSubmittedVertices} "
            + $"cropWorkBudgetRefusedVertices={CropWorkBudgetRefusedVertices} "
            + $"coverageTileSnaps={CoverageTileSnaps} coverageTileReuses={CoverageTileReuses} "
            + $"coverageTileSmallDomainSkips={CoverageTileSmallDomainSkips} coverageTileRepeatedQualifications={CoverageTileRepeatedQualifications} "
            + $"coverageTileRangeRefusals={CoverageTileRangeRefusals} coverageTileSubmittedVertices={CoverageTileSubmittedVertices} "
            + $"coverageTileTightArea={CoverageTileTightArea:R} coverageTileCropArea={CoverageTileCropArea:R} "
            + $"coverageTileBoundsMs={CoverageTileBoundsMilliseconds:F1} coverageTileBoundsTimingParent=localCoverageMs "
            + $"localTileCacheHits={LocalTileCacheHits} localTileCacheMisses={LocalTileCacheMisses} localTileCacheEvictions={LocalTileCacheEvictions} "
            + $"localTileCachePeakEstimatedBytes={LocalTileCachePeakEstimatedBytes} localTileCacheLimit=8 localTileCacheByteLimit=8388608 "
            + $"contourBoundsBuildMs={ContourBoundsBuildMilliseconds:F1} contourBoundsQueryMs={ContourBoundsQueryMilliseconds:F1} "
            + $"coverageCropMs={CropMilliseconds:F1} localBufferMs={LocalBufferMilliseconds:F1} localIndexMs={LocalIndexMilliseconds:F1} "
            + $"followingSweepMs={FollowingSweepMilliseconds:F1} localCoverageMs={LocalConstructionMilliseconds:F1} "
            + $"followingCacheHits={FollowingCacheHits} followingCacheMisses={FollowingCacheMisses} "
            + $"followingCacheEvictions={FollowingCacheEvictions} followingCacheBypasses={FollowingCacheBypasses} "
            + $"followingCacheKeyVertices={FollowingCacheKeyVertices} followingCacheLookupMs={FollowingCacheLookupMilliseconds:F1} "
            + $"followingCacheEstimatedBytes={FollowingCacheEstimatedBytes} followingCachePeakEstimatedBytes={FollowingCachePeakEstimatedBytes} "
            + $"wholeCoverageMs={WholeConstructionMilliseconds:F1} spatialCheckMs={SpatialCheckMilliseconds:F1} "
            + $"wholeBufferMs={WholeBufferMilliseconds:F1} wholeIndexMs={WholeIndexMilliseconds:F1} "
            + $"validatedLocalProofs={ValidatedLocalProofs} localProofMismatches={LocalProofMismatches} "
            + $"validationConstructionMs={ValidationConstructionMilliseconds:F1} validationCapsuleMs={ValidationCapsuleMilliseconds:F1} "
            + $"capsuleCheckMs={CapsuleCheckMilliseconds:F1} localFailurePolicy=whole-domain-fallback "
            + $"coverageFallbackReason=local-subset-unresolved localAuthority=captured-floor-following-joint-crop "
            + $"cropOffsetReserve={TemporalClearance.CropOffsetReserve:R} ";
    }

    internal sealed class CoverageBatch
    {
        internal IReadOnlyList<List<PointD>> Paths { get; }
        internal CoverageBounds Bounds { get; }
        internal int Ordinal { get; }
        private PolygonOps.BoundarySegmentIndex? _index;
        internal CoverageBatch(IReadOnlyList<List<PointD>> paths, int ordinal)
        {
            Paths = paths;
            Ordinal = ordinal;
            double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
            double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
            foreach (var ring in paths)
            foreach (var point in ring)
            {
                minX = Math.Min(minX, point.x); minY = Math.Min(minY, point.y);
                maxX = Math.Max(maxX, point.x); maxY = Math.Max(maxY, point.y);
            }
            Bounds = new(minX, minY, maxX, maxY);
        }
        internal PolygonOps.BoundarySegmentIndex Index(CoverageDiagnostics diagnostics)
        {
            if (_index is not null) return _index;
            diagnostics.BatchIndexBuilds++;
            return _index = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(Paths));
        }
    }

    // Complete immutable signed contours from one captured floor and its first
    // following cut. Bounds select WHOLE contours, never edges or independently
    // filled boundaries. A contour whose complete integer bbox misses R has zero
    // winding everywhere in R. An outer or hole enclosing R therefore remains,
    // even when none of its boundary edges intersects R.
    internal sealed class PreparedCoverageContours
    {
        private readonly record struct Contour(PointD[] Points, long MinX, long MinY, long MaxX, long MaxY);
        private readonly Contour[] _contours;
        internal bool SupportsCrop { get; }
        internal long VertexCount { get; }

        internal PreparedCoverageContours(IEnumerable<IReadOnlyList<PointD>> paths,
            CancellationToken token, CoverageDiagnostics diagnostics)
        {
            token.ThrowIfCancellationRequested();
            long started = Stopwatch.GetTimestamp();
            var contours = new List<Contour>();
            bool supportsCrop = true;
            long vertexCount = 0;
            foreach (var path in paths)
            {
                token.ThrowIfCancellationRequested();
                if (path.Count < 3) continue;
                var points = new PointD[path.Count];
                long minX = long.MaxValue, minY = long.MaxValue, maxX = long.MinValue, maxY = long.MinValue;
                for (int index = 0; index < path.Count; index++)
                {
                    if ((index & 255) == 0) token.ThrowIfCancellationRequested();
                    PointD point = path[index];
                    points[index] = point;
                    double x = Math.Round(point.x * PathUtils.Scale), y = Math.Round(point.y * PathUtils.Scale);
                    if (!double.IsFinite(x) || !double.IsFinite(y) || Math.Abs(x) > 1e12 || Math.Abs(y) > 1e12)
                    { supportsCrop = false; continue; }
                    minX = Math.Min(minX, (long)x); minY = Math.Min(minY, (long)y);
                    maxX = Math.Max(maxX, (long)x); maxY = Math.Max(maxY, (long)y);
                }
                contours.Add(new Contour(points, minX, minY, maxX, maxY));
                vertexCount += points.Length;
            }
            token.ThrowIfCancellationRequested();
            _contours = contours.ToArray(); SupportsCrop = supportsCrop; VertexCount = vertexCount;
            double milliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            diagnostics.ContourBoundsBuilds++;
            diagnostics.ContourBoundsInputVertices += vertexCount;
            diagnostics.ContourBoundsBuildMilliseconds += milliseconds;
            diagnostics.LocalConstructionMilliseconds += milliseconds;
        }

        internal IReadOnlyList<IReadOnlyList<PointD>> Intersecting(long minX, long minY, long maxX, long maxY,
            CancellationToken token, CoverageDiagnostics diagnostics)
        {
            token.ThrowIfCancellationRequested();
            long started = Stopwatch.GetTimestamp();
            var selected = new List<IReadOnlyList<PointD>>();
            long vertices = 0;
            foreach (var contour in _contours)
            {
                token.ThrowIfCancellationRequested();
                if (contour.MaxX < minX || contour.MinX > maxX || contour.MaxY < minY || contour.MinY > maxY) continue;
                selected.Add(contour.Points); vertices += contour.Points.Length;
            }
            double milliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            diagnostics.ContourBoundsQueries++;
            diagnostics.ContourBoundsCandidates += selected.Count;
            diagnostics.ContourBoundsConsideredVertices += VertexCount;
            diagnostics.ContourBoundsSubmittedVertices += vertices;
            diagnostics.ContourBoundsQueryMilliseconds += milliseconds;
            diagnostics.LocalConstructionMilliseconds += milliseconds;
            return selected;
        }
    }

    /// <summary>
    /// Operation-local construction reuse, with no removal authority of its own.
    /// A lookup must match every ordered quantized source vertex and polyline
    /// boundary before it reuses the original open-stroke construction result.
    /// </summary>
    internal sealed class FollowingSweepCache
    {
        internal sealed class SweepKey : IEquatable<SweepKey>
        {
            private readonly Point64[][] _lines;
            private readonly long _radius, _arcTolerance;
            private readonly int _hash;
            internal long Vertices { get; }
            internal long EstimatedBytes => Vertices * 16 + _lines.Length * 32L + 64;

            internal SweepKey(IReadOnlyList<IReadOnlyList<PointD>> lines, double radius, double arcTolerance,
                CancellationToken token, out bool finite)
            {
                _radius = BitConverter.DoubleToInt64Bits(radius);
                _arcTolerance = BitConverter.DoubleToInt64Bits(arcTolerance);
                _lines = new Point64[lines.Count][];
                finite = double.IsFinite(radius) && double.IsFinite(arcTolerance);
                var hash = new System.HashCode(); hash.Add(_radius); hash.Add(_arcTolerance); hash.Add(lines.Count);
                long vertices = 0;
                for (int line = 0; line < lines.Count; line++)
                {
                    token.ThrowIfCancellationRequested();
                    var source = lines[line];
                    var points = new Point64[source.Count];
                    hash.Add(points.Length);
                    for (int i = 0; i < points.Length; i++)
                    {
                        if ((i & 1023) == 0) token.ThrowIfCancellationRequested();
                        finite &= double.IsFinite(source[i].x) && double.IsFinite(source[i].y);
                        points[i] = new((long)Math.Round(source[i].x * PathUtils.Scale),
                            (long)Math.Round(source[i].y * PathUtils.Scale));
                        hash.Add(points[i].X); hash.Add(points[i].Y);
                    }
                    _lines[line] = points; vertices += points.Length;
                }
                Vertices = vertices; _hash = hash.ToHashCode();
            }

            public bool Equals(SweepKey? other)
            {
                if (other is null || _radius != other._radius || _arcTolerance != other._arcTolerance
                    || _lines.Length != other._lines.Length) return false;
                for (int line = 0; line < _lines.Length; line++)
                {
                    var left = _lines[line]; var right = other._lines[line];
                    if (left.Length != right.Length) return false;
                    for (int i = 0; i < left.Length; i++)
                        if (left[i] != right[i]) return false;
                }
                return true;
            }
            public override bool Equals(object? other) => other is SweepKey key && Equals(key);
            public override int GetHashCode() => _hash;
            internal IReadOnlyList<IReadOnlyList<PointD>> Materialize()
                => _lines.Select(line => (IReadOnlyList<PointD>)line.Select(point =>
                    new PointD(point.X / PathUtils.Scale, point.Y / PathUtils.Scale)).ToArray()).ToArray();
        }

        private const long MaximumEstimatedBytes = 64 * 1024 * 1024;
        private const int MaximumEntries = 128;
        private readonly Dictionary<SweepKey, CoverageBatch> _entries;
        private long _estimatedBytes;
        internal FollowingSweepCache(IEqualityComparer<SweepKey>? comparer = null)
            => _entries = new Dictionary<SweepKey, CoverageBatch>(comparer);

        internal CoverageBatch Get(IEnumerable<IReadOnlyList<PointD>> following, double radius,
            double arcTolerance, CancellationToken token, CoverageDiagnostics diagnostics)
        {
            token.ThrowIfCancellationRequested();
            long started = Stopwatch.GetTimestamp();
            var lines = following.ToList();
            var key = new SweepKey(lines, radius, arcTolerance, token, out bool finite);
            token.ThrowIfCancellationRequested();
            diagnostics.FollowingCacheKeyVertices += key.Vertices;
            if (finite && _entries.TryGetValue(key, out var cached))
            {
                token.ThrowIfCancellationRequested();
                diagnostics.FollowingCacheHits++;
                diagnostics.FollowingCacheLookupMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                return cached;
            }
            diagnostics.FollowingCacheMisses++;
            // Reconstruct from the immutable integer snapshot. Scale is a power
            // of two, so the offsetter receives exactly the original integers,
            // including singleton/skipped lines and the 8192-point chunk switch.
            var source = finite ? key.Materialize() : lines;
            long sweepStarted = Stopwatch.GetTimestamp();
            var next = PolygonOps.BufferOpenCenterlines(source, radius, arcTolerance, token);
            double sweepMilliseconds = Stopwatch.GetElapsedTime(sweepStarted).TotalMilliseconds;
            diagnostics.FollowingSweepMilliseconds += sweepMilliseconds;
            token.ThrowIfCancellationRequested();
            var batch = new CoverageBatch(next, int.MaxValue);
            long bytes = key.EstimatedBytes + next.Sum(path => (long)path.Count * 16 + 32);
            if (finite && bytes <= MaximumEstimatedBytes)
            {
                if (_entries.Count >= MaximumEntries || _estimatedBytes + bytes > MaximumEstimatedBytes)
                {
                    diagnostics.FollowingCacheEvictions += _entries.Count;
                    _entries.Clear(); _estimatedBytes = 0;
                }
                _entries.Add(key, batch); _estimatedBytes += bytes;
                diagnostics.FollowingCacheEstimatedBytes = _estimatedBytes;
                diagnostics.FollowingCachePeakEstimatedBytes = Math.Max(diagnostics.FollowingCachePeakEstimatedBytes, _estimatedBytes);
            }
            else diagnostics.FollowingCacheBypasses++;
            diagnostics.FollowingCacheLookupMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds - sweepMilliseconds;
            return batch;
        }
    }

    // Appending immutable sweep batches does not rebuild a polygon arrangement.
    // A binomial forest gives complete-bound pruning and logarithmic append work.
    internal sealed class CoverageForest
    {
        private sealed class Node
        {
            internal readonly CoverageBounds Bounds;
            internal readonly int FirstOrdinal;
            internal readonly CoverageBatch? Leaf;
            internal readonly Node? First, Second;
            internal Node(CoverageBatch leaf)
            { Leaf = leaf; Bounds = leaf.Bounds; FirstOrdinal = leaf.Ordinal; }
            internal Node(Node first, Node second)
            {
                First = first; Second = second; FirstOrdinal = first.FirstOrdinal;
                Bounds = CoverageBounds.Combine(first.Bounds, second.Bounds);
            }
        }
        private readonly List<Node?> _roots = new();
        internal void Add(CoverageBatch batch)
        {
            var node = new Node(batch);
            int level = 0;
            while (level < _roots.Count && _roots[level] is { } prior)
            { node = new Node(prior, node); _roots[level++] = null; }
            if (level == _roots.Count) _roots.Add(node); else _roots[level] = node;
        }
        internal IReadOnlyList<CoverageBatch> Intersecting(CoverageBounds bounds, int prefix,
            CancellationToken token, CoverageDiagnostics diagnostics)
        {
            var result = new List<CoverageBatch>();
            foreach (var root in _roots) if (root is not null) Visit(root);
            result.Sort((a, b) => a.Ordinal.CompareTo(b.Ordinal));
            return result;
            void Visit(Node node)
            {
                token.ThrowIfCancellationRequested();
                diagnostics.BatchNodeVisits++;
                if (node.FirstOrdinal >= prefix || !node.Bounds.Intersects(bounds)) return;
                if (node.Leaf is { } batch)
                { result.Add(batch); diagnostics.BatchCandidates++; return; }
                Visit(node.First!); Visit(node.Second!);
            }
        }
    }

    internal sealed class TemporalClearance : ILinkClearance
    {
        // Both proofs start with the SAME original ordered floor union S and
        // immutable first-following sweep F. Canonical S/F have winding 0 or 1;
        // a joint NonZero intersection therefore computes (S union F) intersect R.
        // The reference's full joint union is deferred to fallback/validation.
        // No error from unrelated historical unions is accumulated: both proofs
        // consume the exact same captured S, never raw historical approximations.
        // Round offset chords are inscribed with sagitta <= ArcTolerance / 4.
        // Reserve that entire reference sagitta plus eighteen integer quanta:
        // one crop-intersection rounding, local/reference offset vertex rounding,
        // their internal and explicit unions, and query conversion. Each XY
        // rounding moves a vertex by < one quantum; eighteen covers these stages
        // without depending on the number of paths or prior batches. Shrinking
        // the local dilation by this reserve makes its complete capsule proof a
        // conservative subset of the unchanged reference morphology. Rectangle
        // intersections and artificial edges remain beyond a further four full
        // construction allowances from the capsule, outside offset influence.
        // The two extra quanta reserve reference full-union and local joint-
        // intersection rounding when commuting crop before that full union.
        internal const double CropOffsetReserve = ArcTolerance / 4 + 18 / PathUtils.Scale;
        private const double CropGuard = 4 * ConstructionError + CropOffsetReserve;
        internal static readonly long CoverageTileGuardTicks = (long)Math.Ceiling(CropGuard * PathUtils.Scale);
        internal readonly record struct CoverageTileWindow(long MinX, long MinY, long MaxX, long MaxY,
            long TileTicks, CoverageBounds Eligible);
        private readonly ILinkClearance _spatial;
        private readonly ClearedStock.FloorSnapshot _floor;
        private readonly CoverageBatch _following;
        private readonly CoverageDiagnostics _diagnostics;
        private readonly bool _useIndexedCoverage;
        private readonly bool _validateIndexedCoverage;
        private readonly bool _limitCropWork;
        private readonly bool _useCoverageTiles;
        private readonly bool _useLocalTileCache;
        private sealed record LocalTile(CoverageBounds Bounds,PolygonOps.BoundarySegmentIndex Index,
            IReadOnlyList<List<PointD>> Source,long EstimatedBytes);
        private readonly List<LocalTile> _localTiles=new(8);
        private long _localTileBytes;
        private readonly PocketPostedStepdownStock.Proof? _priorStepdown;
        private readonly double _floorZ;
        private PolygonOps.BoundarySegmentIndex? _wholeAllowed, _localAllowed, _validationAllowed;
        private PolygonOps.BoundarySegmentIndex? _undilatedAllowed;
        private IReadOnlyList<List<PointD>>? _capturedAllowed;
        private PreparedCoverageContours? _cropContours;
        private CoverageBounds _localBounds;
        private IReadOnlyList<List<PointD>>? _localSource;
        private PreparedRegionDistance? _localSourceDistance;
        private long _cropSubmittedVertices;
        private int _cropBuilds;
        private int _tightCropBuilds;
        private bool _localTiled;
        private readonly CancellationToken _token;
        internal int Queries { get; private set; }
        internal TemporalClearance(ILinkClearance spatial, ClearedStock stock, double floor,
            IEnumerable<IReadOnlyList<PointD>> following, double radius, CancellationToken token,
            CoverageDiagnostics? diagnostics = null, bool useIndexedCoverage = true,
            FollowingSweepCache? followingCache = null, bool validateIndexedCoverage = false,
            bool limitCropWork = true, PocketPostedStepdownStock.Proof? priorStepdown = null,
            bool useCoverageTiles = true,bool useLocalTileCache=true)
        {
            _spatial = spatial;
            _token = token;
            _diagnostics = diagnostics ?? new CoverageDiagnostics();
            _useIndexedCoverage = useIndexedCoverage;
            _validateIndexedCoverage = validateIndexedCoverage;
            _limitCropWork = limitCropWork;
            _useCoverageTiles = useCoverageTiles
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_CROP_TILE_REFERENCE") != "1";
            _useLocalTileCache=useLocalTileCache;
            _priorStepdown = priorStepdown;
            _floorZ = floor;
            _floor = stock.PrepareAtFloor(floor);
            if (followingCache is not null)
                _following = followingCache.Get(following, radius, ArcTolerance, token, _diagnostics);
            else
            {
                long started = Stopwatch.GetTimestamp();
                var next = PolygonOps.BufferOpenCenterlines(following, radius, ArcTolerance, token);
                _following = new CoverageBatch(next, int.MaxValue);
                _diagnostics.FollowingSweepMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            }
            if (!useIndexedCoverage) PrepareWhole();
            token.ThrowIfCancellationRequested();
        }
        public bool IsTravelSafe(Point3D from, Point3D to, double radius)
        {
            _token.ThrowIfCancellationRequested();
            Queries++;
            long spatialStarted = Stopwatch.GetTimestamp();
            bool spatiallySafe = _spatial.IsTravelSafe(from, to, radius);
            _diagnostics.SpatialCheckMilliseconds += Stopwatch.GetElapsedTime(spatialStarted).TotalMilliseconds;
            if (!spatiallySafe) return false;
            // LINK clearance's established .003122 dilation is not permission
            // to cut a deeper sliver at an earlier floor. Admit that stepdown
            // independently against the actual prior prefix, with ZERO lateral
            // stock credit and no first-following Cut in this predicate.
            if (_priorStepdown is not null && !_priorStepdown.IsSafe(from, to)) return false;
            var start = new Point64((long)Math.Round(from.X * PathUtils.Scale), (long)Math.Round(from.Y * PathUtils.Scale));
            var end = new Point64((long)Math.Round(to.X * PathUtils.Scale), (long)Math.Round(to.Y * PathUtils.Scale));
            if (_wholeAllowed is not null) return Contains(_wholeAllowed, start, end, radius);
            if(_undilatedAllowed is not null&&TryUndilated(from,to,start,end,radius))return true;
            if (_useIndexedCoverage && TryLocal(from, to, start, end, radius))
            {
                _diagnostics.LocalProofs++;
                if (_validateIndexedCoverage) ValidateLocal(from, to, start, end, radius);
                return true;
            }
            if (_useIndexedCoverage && HasUncoveredWitness(from,to,start,end,radius))
            {
                if(_validateIndexedCoverage)
                {
                    _validationAllowed ??= BuildWhole(validation:true);
                    if(Contains(_validationAllowed,start,end,radius))
                        throw new InvalidOperationException("Uncovered-stock witness disagrees with the complete reference morphology.");
                }
                return false;
            }
            // If C(r-e) is inside the complete captured fill S, then
            // C(r)=C(r-e)+disk(e) is inside S+disk(e). Use only the same
            // inscribed-dilation/error reserve as the guarded local proof.
            // This is a sufficient positive proof; unresolved capsules still
            // build the unchanged full reference morphology below.
            double inscribedDilation=ConstructionError-CropOffsetReserve;
            if(_useIndexedCoverage&&radius>=inscribedDilation)
            {
                _undilatedAllowed??=new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(CapturedAllowed()));
                if(TryUndilated(from,to,start,end,radius))return true;
            }
            _diagnostics.WholeFallbacks++;
            PrepareWhole();
            return Contains(_wholeAllowed!, start, end, radius);
        }

        private bool TryUndilated(Point3D from,Point3D to,Point64 start,Point64 end,double radius)
        {
            double inscribedDilation=ConstructionError-CropOffsetReserve;
            if(radius<inscribedDilation||!Contains(_undilatedAllowed!,start,end,radius-inscribedDilation))return false;
            _diagnostics.UndilatedProofs++;
            if(_validateIndexedCoverage)ValidateLocal(from,to,start,end,radius);
            return true;
        }

        private bool HasUncoveredWitness(Point3D from,Point3D to,Point64 start,Point64 end,double radius)
        {
            // This is negative evidence only. The crop retains the complete signed
            // S/F fill inside its guarded rectangle, including enclosing holes.
            // A capsule point farther outside that fill than the complete upper
            // dilation/rounding budget cannot belong to the reference morphology.
            // All uncertain queries still require the original whole-domain proof.
            if(_localSource==null || !double.IsFinite(radius) || radius<=0
                || !_localBounds.Contains(CoverageBounds.Around(from,to,radius+ConstructionError+4/PathUtils.Scale))) return false;
            long started=Stopwatch.GetTimestamp();
            try
            {
                _localSourceDistance ??= new PreparedRegionDistance(_localSource);
                double ax=start.X/PathUtils.Scale,ay=start.Y/PathUtils.Scale;
                double dx=(end.X-start.X)/PathUtils.Scale,dy=(end.Y-start.Y)/PathUtils.Scale;
                double length=Math.Sqrt(dx*dx+dy*dy),nx=length>0?-dy/length:1,ny=length>0?dx/length:0;
                double upper=ConstructionError+CropOffsetReserve+PreparedRegionDistance.Uncertainty+2/PathUtils.Scale;
                foreach(double t in new[]{.5,0,1})
                foreach(var offset in new[]{new PointD(0,0),new PointD(nx*radius,ny*radius),new PointD(-nx*radius,-ny*radius),
                    new PointD(radius,0),new PointD(-radius,0),new PointD(0,radius),new PointD(0,-radius)})
                {
                    _token.ThrowIfCancellationRequested();
                    var witness=new PointD(ax+t*dx+offset.x,ay+t*dy+offset.y);
                    _diagnostics.NegativeQueries++;
                    double distance=_localSourceDistance.SignedDistance(witness);
                    if(distance>=-upper) continue;
                    if(++_diagnostics.NegativeWitnesses<=8)
                        Console.WriteLine($"[PocketContinuation.UncoveredWitness] floor={_floorZ:R} from={from} to={to} "
                            + $"witness=({witness.x:R},{witness.y:R}) signedDistance={distance:R} referenceUpperDilation={upper:R} "
                            + $"radius={radius:R} prefixBatches={_floor.PrefixBatches} authority=complete-guarded-source-crop decision=refuse-only");
                    return true;
                }
                return false;
            }
            finally {_diagnostics.NegativeWitnessMilliseconds+=Stopwatch.GetElapsedTime(started).TotalMilliseconds;}
        }

        private bool TryLocal(Point3D from, Point3D to, Point64 start, Point64 end, double radius)
        {
            var required = CoverageBounds.Around(from, to, radius + ConstructionError + 4 / PathUtils.Scale);
            if (_localAllowed is not null && _localBounds.Contains(required))
            {
                if (_localTiled) _diagnostics.CoverageTileReuses++;
                bool safe = Contains(_localAllowed, start, end, radius);
                if (safe) _diagnostics.CroppedDomainProofs++;
                return safe;
            }
            if(_useLocalTileCache)
            {
                // Every entry belongs to this ONE captured floor prefix and
                // first following cut. Only the full current capsule test can
                // grant clearance. Spatial/Z restrictions already ran above.
                for(int i=_localTiles.Count-1;i>=0;i--)
                {
                    var tile=_localTiles[i];if(!tile.Bounds.Contains(required))continue;
                    _localAllowed=tile.Index;_localBounds=tile.Bounds;_localSource=tile.Source;
                    _localSourceDistance=null;_localTiled=true;
                    _localTiles.RemoveAt(i);_localTiles.Add(tile);
                    _diagnostics.LocalTileCacheHits++;_diagnostics.CoverageTileReuses++;
                    bool safe=Contains(_localAllowed,start,end,radius);
                    if(safe)_diagnostics.CroppedDomainProofs++;
                    return safe;
                }
                _diagnostics.LocalTileCacheMisses++;
            }
            long started = Stopwatch.GetTimestamp();
            double priorCapsuleMilliseconds = _diagnostics.CapsuleCheckMilliseconds;
            if (ConstructionError <= (8d * _floor.PrefixBatches + 16) / PathUtils.Scale)
                _diagnostics.HistoricalReserveExhausted++;
            var batches = _floor.Intersecting(required, _diagnostics).ToList();
            if (_following.Bounds.Intersects(required)) batches.Add(_following);
            // A complete single-batch capsule is a strict subset proof. Its
            // immutable prepared boundary needs no tolerance dilation or union.
            foreach (var batch in batches)
                if (Contains(batch.Index(_diagnostics), start, end,
                    radius + (8d * _floor.PrefixBatches + 16) / PathUtils.Scale))
                {
                    _diagnostics.LocalConstructionMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds
                        - (_diagnostics.CapsuleCheckMilliseconds - priorCapsuleMilliseconds);
                    _diagnostics.RawBatchProofs++;
                    return true;
                }
            _diagnostics.LocalConstructionMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds
                - (_diagnostics.CapsuleCheckMilliseconds - priorCapsuleMilliseconds);
            // Bound rectangle conversion and all intersection arithmetic to the
            // library's precise integer range. Unresolved bounds keep the exact
            // whole-domain fallback rather than creating a cropped authority.
            double minX = Math.Floor((required.MinX - CropGuard) * PathUtils.Scale);
            double minY = Math.Floor((required.MinY - CropGuard) * PathUtils.Scale);
            double maxX = Math.Ceiling((required.MaxX + CropGuard) * PathUtils.Scale);
            double maxY = Math.Ceiling((required.MaxY + CropGuard) * PathUtils.Scale);
            if (!double.IsFinite(minX) || !double.IsFinite(minY)
                || !double.IsFinite(maxX) || !double.IsFinite(maxY)
                || Math.Max(Math.Max(Math.Abs(minX), Math.Abs(minY)), Math.Max(Math.Abs(maxX), Math.Abs(maxY))) > 1e12)
            { _diagnostics.CropGuardRefusals++; return false; }
            var contours = CropContours();
            if (!contours.SupportsCrop)
            { _diagnostics.CropGuardRefusals++; return false; }
            // Reuse remains inside this ONE immutable floor/following authority.
            // Enlarge only the original padded crop, then inset its integer
            // eligibility window by the full unchanged artificial-edge guard.
            // A power-of-two grid at least four cutter radii bounds small moves
            // without changing the signed filled domain or its offset reserve.
            // Keep a simple small proof tight; repeated work qualifies its fourth
            // crop before the unchanged four-domain construction crossover.
            long tileStarted = Stopwatch.GetTimestamp();
            bool tiled = false;
            var eligible = required;
            bool repeated = _tightCropBuilds >= 3;
            if (_useCoverageTiles && (contours.VertexCount > 8192 || repeated))
            {
                if (TryExpandCoverageTile((long)minX, (long)minY, (long)maxX, (long)maxY, radius, out var tile))
                {
                    double tightArea = (maxX - minX) * (maxY - minY) / (PathUtils.Scale * PathUtils.Scale);
                    minX = tile.MinX; minY = tile.MinY; maxX = tile.MaxX; maxY = tile.MaxY;
                    eligible = tile.Eligible; tiled = true;
                    _diagnostics.CoverageTileSnaps++;
                    if (repeated && contours.VertexCount <= 8192) _diagnostics.CoverageTileRepeatedQualifications++;
                    _diagnostics.CoverageTileTightArea += tightArea;
                    _diagnostics.CoverageTileCropArea += (maxX - minX) * (maxY - minY) / (PathUtils.Scale * PathUtils.Scale);
                }
                else _diagnostics.CoverageTileRangeRefusals++;
            }
            else if (_useCoverageTiles) _diagnostics.CoverageTileSmallDomainSkips++;
            double tileMilliseconds = Stopwatch.GetElapsedTime(tileStarted).TotalMilliseconds;
            _diagnostics.CoverageTileBoundsMilliseconds += tileMilliseconds;
            _diagnostics.LocalConstructionMilliseconds += tileMilliseconds;
            _token.ThrowIfCancellationRequested();
            var selected = contours.Intersecting((long)minX, (long)minY, (long)maxX, (long)maxY,
                _token, _diagnostics);
            long submittedVertices = selected.Sum(path => (long)path.Count);
            // A repaired link can query hundreds of different graph edges and
            // original rebase vertices against this ONE immutable authority.
            // Repeated exact intersections may then cost more than preparing
            // the original whole proof once. Bound submitted Boolean work to
            // four complete-domain inputs after four crops; the fallback uses
            // the unchanged captured prefix, first Cut, and full morphology.
            // This is a construction crossover, never negative stock evidence.
            if (_limitCropWork && _cropBuilds >= 4
                && _cropSubmittedVertices + submittedVertices > 4 * contours.VertexCount)
            {
                _diagnostics.CropWorkBudgetFallbacks++;
                _diagnostics.CropWorkBudgetPriorBuilds += _cropBuilds;
                _diagnostics.CropWorkBudgetPriorSubmittedVertices += _cropSubmittedVertices;
                _diagnostics.ContourBoundsSubmittedVertices -= submittedVertices;
                _diagnostics.CropWorkBudgetRefusedVertices += submittedVertices;
                return false;
            }
            long cropStarted = Stopwatch.GetTimestamp();
            // Intersect the complete signed filled domain as one group. The
            // ring-oriented ClipToRect API intentionally clips each contour
            // independently, which would turn a negative hole into an outer.
            IReadOnlyList<PointD>[] rectangle = new IReadOnlyList<PointD>[] { new PointD[]
            {
                new(minX / PathUtils.Scale, minY / PathUtils.Scale),
                new(maxX / PathUtils.Scale, minY / PathUtils.Scale),
                new(maxX / PathUtils.Scale, maxY / PathUtils.Scale),
                new(minX / PathUtils.Scale, maxY / PathUtils.Scale),
            } };
            var cropped = PolygonOps.Intersect(selected, rectangle);
            double cropMilliseconds = Stopwatch.GetElapsedTime(cropStarted).TotalMilliseconds;
            _diagnostics.CropMilliseconds += cropMilliseconds;
            _diagnostics.LocalInputVertices += cropped.Sum(path => (long)path.Count);
            _token.ThrowIfCancellationRequested();
            long bufferStarted = Stopwatch.GetTimestamp();
            var certified = PolygonOps.Union(PolygonOps.Buffer(cropped,
                ConstructionError - CropOffsetReserve, ArcTolerance / 4));
            double bufferMilliseconds = Stopwatch.GetElapsedTime(bufferStarted).TotalMilliseconds;
            _diagnostics.LocalBufferMilliseconds += bufferMilliseconds;
            _diagnostics.LocalOutputVertices += certified.Sum(path => (long)path.Count);
            _token.ThrowIfCancellationRequested();
            long indexStarted = Stopwatch.GetTimestamp();
            _localAllowed = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(certified));
            double indexMilliseconds = Stopwatch.GetElapsedTime(indexStarted).TotalMilliseconds;
            _diagnostics.LocalIndexMilliseconds += indexMilliseconds;
            _localBounds = eligible;
            _localSource=cropped;_localSourceDistance=null;
            _localTiled = tiled;
            if(_useLocalTileCache&&tiled)
            {
                // Account conservatively for contours, the prepared boundary
                // tree and dictionary/array overhead. Oversized proofs retain
                // the established single current tile only.
                long estimated=256+cropped.Sum(p=>(long)p.Count)*32+certified.Sum(p=>(long)p.Count)*192;
                const long byteLimit=8L*1024*1024;
                if(estimated<=byteLimit)
                {
                    while(_localTiles.Count>=8||_localTileBytes+estimated>byteLimit)
                    {
                        _localTileBytes-=_localTiles[0].EstimatedBytes;_localTiles.RemoveAt(0);
                        _diagnostics.LocalTileCacheEvictions++;
                    }
                    _localTiles.Add(new(eligible,_localAllowed,cropped,estimated));_localTileBytes+=estimated;
                    _diagnostics.LocalTileCachePeakEstimatedBytes=Math.Max(_diagnostics.LocalTileCachePeakEstimatedBytes,_localTileBytes);
                }
            }
            _cropBuilds++;
            if (!tiled) _tightCropBuilds++;
            _cropSubmittedVertices += submittedVertices;
            if (tiled) _diagnostics.CoverageTileSubmittedVertices += submittedVertices;
            _diagnostics.LocalIndexBuilds++;
            _diagnostics.LocalConstructionMilliseconds += cropMilliseconds + bufferMilliseconds + indexMilliseconds;
            bool contained = Contains(_localAllowed, start, end, radius);
            if (contained) _diagnostics.CroppedDomainProofs++;
            return contained;
        }

        internal static bool TryExpandCoverageTile(long minX, long minY, long maxX, long maxY,
            double radius, out CoverageTileWindow window)
        {
            window = default;
            const long preciseLimit = 1_000_000_000_000;
            if (!double.IsFinite(radius) || radius <= 0 || minX > maxX || minY > maxY
                || minX < -preciseLimit || minY < -preciseLimit || maxX > preciseLimit || maxY > preciseLimit) return false;
            double requested = Math.Ceiling(4 * radius * PathUtils.Scale);
            if (!double.IsFinite(requested) || requested > preciseLimit) return false;
            long tile = 1;
            while (tile < requested) tile <<= 1;
            long lowX = Down(minX), lowY = Down(minY), highX = Up(maxX), highY = Up(maxY);
            if (lowX < -preciseLimit || lowY < -preciseLimit || highX > preciseLimit || highY > preciseLimit) return false;
            var eligible = new CoverageBounds((lowX + CoverageTileGuardTicks) / PathUtils.Scale,
                (lowY + CoverageTileGuardTicks) / PathUtils.Scale,
                (highX - CoverageTileGuardTicks) / PathUtils.Scale,
                (highY - CoverageTileGuardTicks) / PathUtils.Scale);
            window = new(lowX, lowY, highX, highY, tile, eligible);
            return true;
            long Down(long value)
            {
                long quotient = Math.DivRem(value, tile, out long remainder);
                return (remainder < 0 ? quotient - 1 : quotient) * tile;
            }
            long Up(long value)
            {
                long quotient = Math.DivRem(value, tile, out long remainder);
                return (remainder > 0 ? quotient + 1 : quotient) * tile;
            }
        }

        private PreparedCoverageContours CropContours()
        {
            if (_cropContours is not null) return _cropContours;
            // Materialization is intentionally outside all local construction
            // stopwatches: its original ordered unions have their own counters.
            var cleared = _floor.Materialize();
            return _cropContours = new PreparedCoverageContours(cleared.Concat(_following.Paths), _token, _diagnostics);
        }

        private IReadOnlyList<List<PointD>> CapturedAllowed()
        {
            if (_capturedAllowed is not null) return _capturedAllowed;
            var cleared = _floor.Materialize();
            long started = Stopwatch.GetTimestamp();
            var allowed = PolygonOps.Union(cleared.Concat(_following.Paths));
            _diagnostics.CapturedUnionMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            _diagnostics.CapturedUnionBuilds++;
            _token.ThrowIfCancellationRequested();
            return _capturedAllowed = allowed;
        }

        private void PrepareWhole()
        {
            // Keep the unchanged whole-domain morphology as the diagnostic and
            // uncertain-case fallback. Snapshot materialization credits only
            // the prefix captured before this link and its first following cut.
            _wholeAllowed = BuildWhole(validation: false);
        }

        private PolygonOps.BoundarySegmentIndex BuildWhole(bool validation)
        {
            // Validation deliberately rebuilds the original union independently.
            // Production builds the identical original joint union only when
            // the local proof is unresolved; successful crops never need it.
            IReadOnlyList<List<PointD>> allowed;
            double validationUnionMilliseconds = 0;
            if (validation)
            {
                var cleared = _floor.Materialize();
                long unionStarted = Stopwatch.GetTimestamp();
                allowed = PolygonOps.Union(cleared.Concat(_following.Paths));
                validationUnionMilliseconds = Stopwatch.GetElapsedTime(unionStarted).TotalMilliseconds;
            }
            else allowed = CapturedAllowed();
            long bufferStarted = Stopwatch.GetTimestamp();
            var certified = PolygonOps.Union(PolygonOps.Buffer(allowed, ConstructionError, ArcTolerance / 4));
            double bufferMilliseconds = Stopwatch.GetElapsedTime(bufferStarted).TotalMilliseconds;
            long indexStarted = Stopwatch.GetTimestamp();
            var index = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(certified));
            double indexMilliseconds = Stopwatch.GetElapsedTime(indexStarted).TotalMilliseconds;
            if (validation) _diagnostics.ValidationConstructionMilliseconds += validationUnionMilliseconds + bufferMilliseconds + indexMilliseconds;
            else
            {
                _diagnostics.WholeBufferMilliseconds += bufferMilliseconds;
                _diagnostics.WholeIndexMilliseconds += indexMilliseconds;
                _diagnostics.WholeConstructionMilliseconds += bufferMilliseconds + indexMilliseconds;
            }
            _token.ThrowIfCancellationRequested();
            return index;
        }

        private void ValidateLocal(Point3D from, Point3D to, Point64 start, Point64 end, double radius)
        {
            _validationAllowed ??= BuildWhole(validation: true);
            long started = Stopwatch.GetTimestamp();
            bool safe = _validationAllowed.ContainsCapsule(start, end, radius * PathUtils.Scale);
            _diagnostics.ValidationCapsuleMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            if (safe) { _diagnostics.ValidatedLocalProofs++; return; }
            _diagnostics.LocalProofMismatches++;
            Console.WriteLine($"[PocketContinuation.Coverage] event=local-proof-mismatch floor={_floorZ:R} "
                + $"prefixBatches={_floor.PrefixBatches} radius={radius:R} "
                + $"from=({from.X:R},{from.Y:R},{from.Z:R}) to=({to.X:R},{to.Y:R},{to.Z:R}) "
                + "reference=original-whole-domain action=refuse-generation");
            throw new InvalidOperationException("An indexed pocket continuation did not satisfy its original whole-domain proof.");
        }

        private bool Contains(PolygonOps.BoundarySegmentIndex index, Point64 from, Point64 to, double radius)
        {
            long started = Stopwatch.GetTimestamp();
            bool contains = index.ContainsCapsule(from, to, radius * PathUtils.Scale);
            _diagnostics.CapsuleCheckMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            return contains;
        }
    }
}
