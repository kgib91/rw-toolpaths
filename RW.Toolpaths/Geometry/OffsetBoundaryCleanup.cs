using Clipper2Lib;
using RW.Toolpaths.Milling;
using System.Diagnostics;

namespace RW.Toolpaths.Geometry;

/// <summary>Compacts native inward fronts only after command-chord and physical proofs.</summary>
internal static class OffsetBoundaryCleanup
{
    /// <summary>Reduces only the commanded first front, after original construction.
    /// Every replacement retains the complete original chain's finite chord tube,
    /// original wall clearance, signed topology and actual clipped-open coverage.</summary>
    internal static List<List<PointD>> CompactFirstCommands(List<List<PointD>> front,
        PolygonOps.BoundarySegmentIndex original, Paths64 originalMaterial,
        double signedOffset, double radius, CancellationToken token,
        IReadOnlyList<PointD>? clipBoundary = null)
    {
        token.ThrowIfCancellationRequested();
        if (signedOffset >= 0 || !double.IsFinite(radius) || radius <= 0
            || front.Sum(path => path.Count) < 128) return front;
        long started = PerfLog.Start();
        const double epsilon = .001 - 4 / PathUtils.Scale;
        const double reserve = .003 + 4 / PathUtils.Scale;
        double required = Math.Max(0, -signedOffset * PathUtils.Scale - 4);
        int queries = 0, rejectedChords = 0;
        var reduced = new List<List<PointD>>(front.Count);
        foreach (var contour in front)
        {
            token.ThrowIfCancellationRequested();
            if (contour.Count < 4) { reduced.Add(contour); continue; }
            int opposite = Enumerable.Range(1, contour.Count - 1).MaxBy(index =>
                Math.Pow(contour[index].x - contour[0].x, 2)
                + Math.Pow(contour[index].y - contour[0].y, 2));
            var chain = contour.Concat(new[] { contour[0] }).ToArray();
            var keep = new bool[chain.Length];
            keep[0] = keep[opposite] = keep[^1] = true;
            var ranges = new Stack<(int Start, int End)>();
            ranges.Push((0, opposite)); ranges.Push((opposite, chain.Length - 1));
            while (ranges.Count > 0)
            {
                token.ThrowIfCancellationRequested();
                var (first, last) = ranges.Pop();
                if (last - first < 2) continue;
                PointD a = chain[first], b = chain[last];
                double dx = b.x - a.x, dy = b.y - a.y, lengthSquared = dx * dx + dy * dy;
                double maximum = 0;
                int split = first + (last - first) / 2;
                for (int index = first + 1; index < last; index++)
                {
                    if ((index & 255) == 0) token.ThrowIfCancellationRequested();
                    double px = chain[index].x - a.x, py = chain[index].y - a.y;
                    double t = lengthSquared <= 1e-24 ? 0
                        : Math.Clamp((px * dx + py * dy) / lengthSquared, 0, 1);
                    double ex = px - t * dx, ey = py - t * dy;
                    double squared = ex * ex + ey * ey;
                    if (squared > maximum) { maximum = squared; split = index; }
                }
                bool bounded = maximum <= epsilon * epsilon;
                if (bounded)
                {
                    queries++;
                    Point64 from = ToPoint(a), to = ToPoint(b);
                    bounded = original.IsInside(from) && original.IsInside(to)
                        && original.ClearanceViolation(from, to, required) is null;
                    if (!bounded) rejectedChords++;
                }
                if (bounded) continue;
                keep[split] = true;
                ranges.Push((first, split)); ranges.Push((split, last));
            }
            var candidate = Enumerable.Range(0, contour.Count).Where(index => keep[index])
                .Select(index => contour[index]).ToList();
            reduced.Add(candidate.Count >= 3
                && PolygonOps.SignedArea(candidate) * PolygonOps.SignedArea(contour) > 0
                ? candidate : contour);
        }
        int sourcePoints = front.Sum(path => path.Count), candidatePoints = reduced.Sum(path => path.Count);
        if (sourcePoints == candidatePoints)
        {
            PerfLog.Stop("Offset.FirstCommandCompaction", started,
                $"sourcePoints={sourcePoints} candidatePoints={candidatePoints} chordQueries={queries} "
                + $"unsafeChords={rejectedChords} accepted=False reason=no-reduction");
            return front;
        }
        var outside = Clipper.Difference(PathUtils.ToClipper(reduced), originalMaterial, FillRule.NonZero);
        bool inside = outside.Count == 0;
        double outsideArea = Math.Abs(Clipper.Area(outside)) / (PathUtils.Scale * PathUtils.Scale);
        token.ThrowIfCancellationRequested();
        bool topology = inside && SameTopology(front, reduced, token);
        double lostArea = double.NaN, gainedArea = double.NaN;
        if (topology)
        {
            var before = RingOffsetEngine.BufferClosedSweeps(front, radius, .001, token, true, clipBoundary);
            var after = RingOffsetEngine.BufferClosedSweeps(reduced, radius, .001, token, true, clipBoundary);
            lostArea = Gap(before, after); gainedArea = Gap(after, before);
        }
        bool accepted = topology && lostArea == 0 && gainedArea == 0;
        PerfLog.Stop("Offset.FirstCommandCompaction", started,
            $"sourcePoints={sourcePoints} candidatePoints={candidatePoints} chordQueries={queries} "
            + $"unsafeChords={rejectedChords} epsilon={epsilon:R} originalClearance={required / PathUtils.Scale:R} "
            + $"filledOriginalContainment={inside} outsideOriginalArea={outsideArea:R} sameTopology={topology} lostFootprintArea={lostArea:R} "
            + $"gainedFootprintArea={gainedArea:R} coverageError={reserve:R} accepted={accepted} "
            + "sweep=actual-explicit-clipped-open originalConstruction=unchanged");
        return accepted ? reduced : front;

        double Gap(IReadOnlyList<List<PointD>> a, IReadOnlyList<List<PointD>> b)
        {
            token.ThrowIfCancellationRequested();
            var residual = PolygonOps.Difference(a, b);
            var interior = PolygonOps.Offset(residual, -reserve, .001 / 4, 0);
            token.ThrowIfCancellationRequested();
            return Math.Abs(interior.Sum(PolygonOps.SignedArea));
        }
    }

    internal static List<List<PointD>> Compact(IReadOnlyList<List<PointD>> front,
        PolygonOps.BoundarySegmentIndex original, double signedOffset, double? sweepRadius,
        double tolerance, CancellationToken token = default, IReadOnlyList<PointD>? clipBoundary = null,
        bool proposeForShellAudit = false)
    {
        token.ThrowIfCancellationRequested();
        if (signedOffset >= 0 || sweepRadius is not { } radius || radius <= 0 || tolerance <= 0)
            return front.Select(path => path.ToList()).ToList();
        if (!front.Any(HasBacktrack)) return front.Select(path => path.ToList()).ToList();
        long cleanupStarted = Stopwatch.GetTimestamp();
        // The same physical component rule defines both sides' real front
        // topology. A redundant source component cannot reject unrelated safe
        // compaction merely because the candidate also proves it complete.
        var reference = FilledFrontCoverage.RetainUncoveredComponents(front, radius, token, clipBoundary);
        double referenceMs = Stopwatch.GetElapsedTime(cleanupStarted).TotalMilliseconds;
        long commandStarted = Stopwatch.GetTimestamp();
        double epsilon = Math.Min(.003, tolerance);
        double required = Math.Max(0, -signedOffset * PathUtils.Scale - 4);
        int unsafeChords = 0, queries = 0, localGeometryRejected = 0, localRemoved = 0;
        int nativeFolds = 0, localFolds = 0, rdpFolds = 0, repairedRdpFolds = 0;
        int midpointRemoved = 0, midpointRejected = 0;
        var reduced = new List<List<PointD>>(reference.Count);
        foreach (var originalContour in reference)
        {
            token.ThrowIfCancellationRequested();
            var contour = originalContour;
            if (contour.Count < 4 || !HasBacktrack(contour)) { reduced.Add(contour.ToList()); continue; }
            nativeFolds += CountBacktracks(contour);
            var nativeCycle = contour.Concat(new[] { contour[0] }).ToList();
            var nativeNodes = Enumerable.Range(0, contour.Count).ToList();
            CompactCoveredBacktracks(nativeCycle, nativeNodes, original, required, epsilon,
                token, ref queries, ref unsafeChords, ref localGeometryRejected);
            localRemoved += contour.Count - nativeNodes.Count;
            var locallyCleaned = nativeNodes.Select(index => nativeCycle[index]).ToList();
            localFolds += CountBacktracks(locallyCleaned);
            if (locallyCleaned.Count >= 3 && PolygonOps.SignedArea(locallyCleaned) * PolygonOps.SignedArea(contour) > 0)
            {
                if (!HasBacktrack(locallyCleaned)) { reduced.Add(locallyCleaned); continue; }
                contour = locallyCleaned;
            }
            else nativeNodes = Enumerable.Range(0, contour.Count).ToList();
            // Anchor two forward portions of the cyclic contour. A native burr
            // at the input seam remains inside one tested chain, rather than
            // becoming two permanently retained neighboring seam endpoints.
            var anchors = Enumerable.Range(0, contour.Count).Where(index =>
            {
                PointD a = contour[(index + contour.Count - 1) % contour.Count], b = contour[index], c = contour[(index + 1) % contour.Count];
                return (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y) >= 0;
            }).ToArray();
            if (anchors.Length < 2) { reduced.Add(contour.ToList()); continue; }
            int first = anchors[0];
            int opposite = anchors.MaxBy(index =>
                Math.Pow(contour[index].x - contour[first].x, 2) + Math.Pow(contour[index].y - contour[first].y, 2));
            var chain = Enumerable.Range(0, contour.Count + 1)
                .Select(index => contour[(first + index) % contour.Count]).ToList();
            int split = (opposite - first + contour.Count) % contour.Count;
            var keep = new bool[chain.Count];
            keep[0] = keep[^1] = true;
            keep[split] = true;
            var ranges = new Stack<(int Start, int End)>();
            ranges.Push((0, split));
            ranges.Push((split, chain.Count - 1));
            while (ranges.Count > 0)
            {
                token.ThrowIfCancellationRequested();
                var (start, end) = ranges.Pop();
                if (end - start < 2) continue;
                PointD a = chain[start], b = chain[end];
                double dx = b.x - a.x, dy = b.y - a.y, squared = dx * dx + dy * dy;
                double maximum = 0;
                int farthest = start + (end - start) / 2;
                for (int index = start + 1; index < end; index++)
                {
                    if ((index & 255) == 0) token.ThrowIfCancellationRequested();
                    double px = chain[index].x - a.x, py = chain[index].y - a.y;
                    double position = squared <= 1e-24 ? 0 : Math.Clamp((px * dx + py * dy) / squared, 0, 1);
                    double ex = px - position * dx, ey = py - position * dy;
                    double distance = ex * ex + ey * ey;
                    if (distance > maximum) { maximum = distance; farthest = index; }
                }
                bool safe = maximum <= epsilon * epsilon;
                if (safe)
                {
                    queries++;
                    Point64 from = ToPoint(a), to = ToPoint(b);
                    safe = original.IsInside(from) && original.IsInside(to)
                        && original.ClearanceViolation(from, to, required) is null;
                    if (!safe) unsafeChords++;
                }
                if (safe) continue;
                keep[farthest] = true;
                ranges.Push((start, farthest));
                ranges.Push((farthest, end));
            }
            var nodes = Enumerable.Range(0, contour.Count).Where(index => keep[index]).ToList();
            rdpFolds += CountBacktracks(nodes.Select(index => chain[index]).ToList());
            CompactReversalWindows(chain, nodes, original, required, epsilon, token, ref queries, ref unsafeChords);
            CompactCoveredBacktracks(chain, nodes, original, required, epsilon, token, ref queries, ref unsafeChords, ref localGeometryRejected);
            var originalNodes = nodes.Select(index => nativeNodes[(first + index) % contour.Count]).ToList();
            var path = CompactBacktrackMidpoints(nativeCycle, originalNodes, original, required, epsilon,
                token, ref queries, ref unsafeChords, ref midpointRemoved, ref midpointRejected);
            repairedRdpFolds += CountBacktracks(path);
            if (HasBacktrack(path))
                Console.WriteLine($"[OffsetBoundaryCleanup.Stages] inset={signedOffset:R} nativePoints={originalContour.Count} "
                    + $"nativeFolds={CountBacktracks(originalContour)} localPoints={locallyCleaned.Count} "
                    + $"localFolds={CountBacktracks(locallyCleaned)} repairedRdpPoints={path.Count} repairedRdpFolds={CountBacktracks(path)}");
            if (path.Count < 3 || PolygonOps.SignedArea(path) * PolygonOps.SignedArea(contour) <= 0)
                path = contour.ToList();
            reduced.Add(path);
        }
        int removed = front.Sum(path => path.Count) - reduced.Sum(path => path.Count);
        if (removed == 0) return reduced;
        double commandMs = Stopwatch.GetElapsedTime(commandStarted).TotalMilliseconds;
        long retainStarted = Stopwatch.GetTimestamp();
        reduced = FilledFrontCoverage.RetainUncoveredComponents(reduced, radius, token, clipBoundary);
        double retainedMs = Stopwatch.GetElapsedTime(retainStarted).TotalMilliseconds;
        long topologyStarted = Stopwatch.GetTimestamp();
        bool topology = SameTopology(reference, reduced, token);
        double topologyMs = Stopwatch.GetElapsedTime(topologyStarted).TotalMilliseconds;
        const double proofArc = .001;
        const double coverageError = .003 + 4 / PathUtils.Scale;
        double lostArea = 0, gainedArea = 0;
        double nativeSweepMs = 0, compactSweepMs = 0, lostProofMs = 0, gainedProofMs = 0;
        if (topology && !proposeForShellAudit)
        {
            long stageStarted = Stopwatch.GetTimestamp();
            var nativeSweep = PolygonOps.BufferCenterlinesExact(front, radius, proofArc, token);
            nativeSweepMs = Stopwatch.GetElapsedTime(stageStarted).TotalMilliseconds;
            token.ThrowIfCancellationRequested();
            stageStarted = Stopwatch.GetTimestamp();
            var compactSweep = PolygonOps.BufferCenterlinesExact(reduced, radius, proofArc, token);
            compactSweepMs = Stopwatch.GetElapsedTime(stageStarted).TotalMilliseconds;
            token.ThrowIfCancellationRequested();
            stageStarted = Stopwatch.GetTimestamp();
            lostArea = DifferenceInteriorArea(nativeSweep, compactSweep, token);
            lostProofMs = Stopwatch.GetElapsedTime(stageStarted).TotalMilliseconds;
            stageStarted = Stopwatch.GetTimestamp();
            gainedArea = DifferenceInteriorArea(compactSweep, nativeSweep, token);
            gainedProofMs = Stopwatch.GetElapsedTime(stageStarted).TotalMilliseconds;
        }
        // A proposal retains all original chord, component and topology tests.
        // It has NO stock authority: only SelectCoherentFront's subsequent full
        // clipped closed/open/history shell audit may publish it. The ordinary
        // and first-front paths continue to require both mutual footprints.
        bool accepted = topology && (proposeForShellAudit || lostArea == 0 && gainedArea == 0);
        Console.WriteLine($"[OffsetBoundaryCleanup] inset={signedOffset:R} sourcePoints={front.Sum(path => path.Count)} "
            + $"candidatePoints={reduced.Sum(path => path.Count)} removed={removed} chordQueries={queries} "
            + $"unsafeChords={unsafeChords} epsilon={epsilon:R} originalClearance={required / PathUtils.Scale:R} "
            + $"localRemoved={localRemoved} localGeometryRejected={localGeometryRejected} remainingFoldContours={reduced.Count(HasBacktrack)} "
            + $"nativeFolds={nativeFolds} localFolds={localFolds} rdpFolds={rdpFolds} repairedRdpFolds={repairedRdpFolds} "
            + $"midpointRemoved={midpointRemoved} midpointRejected={midpointRejected} "
            + $"sameTopology={topology} footprintEvaluated={topology && !proposeForShellAudit} "
            + $"proposalOnly={proposeForShellAudit} stockAuthorized=False "
            + $"lostFootprintArea={lostArea:R} gainedFootprintArea={gainedArea:R} "
            + $"coverageError={coverageError:R} accepted={accepted} "
            + $"referenceCoverageMs={referenceMs:F2} commandCleanupMs={commandMs:F2} retainedCoverageMs={retainedMs:F2} "
            + $"topologyMs={topologyMs:F2} nativeSweepMs={nativeSweepMs:F2} compactSweepMs={compactSweepMs:F2} "
            + $"lostProofMs={lostProofMs:F2} gainedProofMs={gainedProofMs:F2} "
            + $"totalMs={Stopwatch.GetElapsedTime(cleanupStarted).TotalMilliseconds:F2} "
            + $"originalChordAndTopologyProofs=unchanged mutualFootprintDeferred={proposeForShellAudit}");
        return accepted ? reduced : front.Select(path => path.ToList()).ToList();

        static double DifferenceInteriorArea(IEnumerable<IReadOnlyList<PointD>> first,
            IEnumerable<IReadOnlyList<PointD>> second, CancellationToken cancellation)
        {
            cancellation.ThrowIfCancellationRequested();
            var difference = PolygonOps.Difference(first, second);
            var interior = PolygonOps.Offset(difference, -coverageError, proofArc / 4, 0);
            cancellation.ThrowIfCancellationRequested();
            return Math.Abs(interior.Sum(PolygonOps.SignedArea));
        }
    }

    private static bool HasBacktrack(IReadOnlyList<PointD> contour)
        => CountBacktracks(contour) != 0;

    private static int CountBacktracks(IReadOnlyList<PointD> contour)
    {
        if (contour.Count < 4) return 0;
        int count = 0;
        for (int index = 0; index < contour.Count; index++)
        {
            PointD a = contour[index], b = contour[(index + 1) % contour.Count],
                c = contour[(index + 2) % contour.Count], d = contour[(index + 3) % contour.Count];
            if ((b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y) < 0
                && (c.x - b.x) * (d.x - c.x) + (c.y - b.y) * (d.y - c.y) < 0) count++;
        }
        return count;
    }

    private static void CompactReversalWindows(IReadOnlyList<PointD> native, List<int> nodes,
        PolygonOps.BoundarySegmentIndex original, double required, double epsilon,
        CancellationToken token, ref int queries, ref int unsafeChords)
    {
        int count = native.Count - 1;
        for (int index = 0; nodes.Count >= 5 && index < nodes.Count; index++)
        {
            token.ThrowIfCancellationRequested();
            int aIndex = nodes[index], bPosition = (index + 1) % nodes.Count,
                cPosition = (index + 2) % nodes.Count, dIndex = nodes[(index + 3) % nodes.Count];
            PointD a = native[aIndex], b = native[nodes[bPosition]], c = native[nodes[cPosition]], d = native[dIndex];
            double ux = b.x - a.x, uy = b.y - a.y, vx = c.x - b.x, vy = c.y - b.y,
                wx = d.x - c.x, wy = d.y - c.y;
            if (ux * vx + uy * vy >= 0 || vx * wx + vy * wy >= 0) continue;
            double dx = d.x - a.x, dy = d.y - a.y, squared = dx * dx + dy * dy;
            bool bounded = true;
            // The authority is the ORIGINAL native interval, including points
            // removed by RDP. Local passes cannot accumulate positional error.
            for (int pointIndex = (aIndex + 1) % count; pointIndex != dIndex; pointIndex = (pointIndex + 1) % count)
            {
                double px = native[pointIndex].x - a.x, py = native[pointIndex].y - a.y;
                double position = squared <= 1e-24 ? 0 : Math.Clamp((px * dx + py * dy) / squared, 0, 1);
                double ex = px - position * dx, ey = py - position * dy;
                if (ex * ex + ey * ey > epsilon * epsilon) { bounded = false; break; }
            }
            if (!bounded) continue;
            queries++;
            Point64 from = ToPoint(a), to = ToPoint(d);
            if (!original.IsInside(from) || !original.IsInside(to)
                || original.ClearanceViolation(from, to, required) is not null)
            { unsafeChords++; continue; }
            nodes.RemoveAt(Math.Max(bPosition, cPosition));
            nodes.RemoveAt(Math.Min(bPosition, cPosition));
            index = -1;
        }
    }

    private static void CompactCoveredBacktracks(IReadOnlyList<PointD> native, List<int> nodes,
        PolygonOps.BoundarySegmentIndex original, double required, double epsilon,
        CancellationToken token, ref int queries, ref int unsafeChords, ref int geometryRejected)
    {
        int count = native.Count - 1;
        for (int index = 0; nodes.Count >= 4 && index < nodes.Count; index++)
        {
            token.ThrowIfCancellationRequested();
            int aIndex = nodes[index], bPosition = (index + 1) % nodes.Count, bIndex = nodes[bPosition],
                cPosition = (index + 2) % nodes.Count, dIndex = nodes[(index + 3) % nodes.Count];
            PointD a = native[aIndex], b = native[bIndex], c = native[nodes[cPosition]], d = native[dIndex];
            if ((b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y) >= 0
                || (c.x - b.x) * (d.x - c.x) + (c.y - b.y) * (d.y - c.y) >= 0) continue;
            bool removed = false;
            for (int extension = 0; extension < 2 && !removed; extension++)
            {
                if (extension == 1 && nodes.Count < 5) break;
                int endIndex = extension == 0 ? dIndex : nodes[(index + 4) % nodes.Count];
                PointD end = native[endIndex];
                var oldEdges = new List<(PointD A, PointD B)>();
                for (int point = aIndex; point != endIndex; point = (point + 1) % count)
                    oldEdges.Add((native[point], native[(point + 1) % count]));
                for (int variant = 0; variant < 2 && !removed; variant++)
                {
                    var newEdges = variant == 0
                        ? new List<(PointD A, PointD B)> { (a, b), (b, d) }
                        : new List<(PointD A, PointD B)> { (a, c), (c, d) };
                    if (extension != 0) newEdges.Add((d, end));
                    if (oldEdges.Any(edge => !SegmentCovered(edge.A, edge.B, newEdges, epsilon))
                        || newEdges.Any(edge => !SegmentCovered(edge.A, edge.B, oldEdges, epsilon)))
                    { geometryRejected++; continue; }
                    queries++;
                    Point64 from = ToPoint(variant == 0 ? b : a), to = ToPoint(variant == 0 ? d : c);
                    if (!original.IsInside(from) || !original.IsInside(to)
                        || original.ClearanceViolation(from, to, required) is not null)
                    { unsafeChords++; continue; }
                    nodes.RemoveAt(variant == 0 ? cPosition : bPosition);
                    removed = true;
                    index = -1;
                }
            }
        }
    }

    // A narrow fold has two forward flanks separated by a backward edge. A
    // midpoint between the peak and its projection onto the outgoing flank can
    // retain the curve while removing the return. Each interval is proved against the complete
    // ORIGINAL contour, not the previously simplified chain. A new midpoint
    // cannot participate in another edit, so these local proofs do not compound.
    private static List<PointD> CompactBacktrackMidpoints(IReadOnlyList<PointD> native, List<int> kept,
        PolygonOps.BoundarySegmentIndex original, double required, double epsilon,
        CancellationToken token, ref int queries, ref int unsafeChords, ref int removed, ref int rejected)
    {
        int count = native.Count - 1;
        var nodes = kept.Select(index => (Point: native[index], NativeIndex: index)).ToList();
        for (int index = 0; nodes.Count >= 4 && index < nodes.Count; index++)
        {
            token.ThrowIfCancellationRequested();
            int bPosition = (index + 1) % nodes.Count, cPosition = (index + 2) % nodes.Count;
            var aNode = nodes[index]; var bNode = nodes[bPosition];
            var cNode = nodes[cPosition]; var dNode = nodes[(index + 3) % nodes.Count];
            if (aNode.NativeIndex < 0 || bNode.NativeIndex < 0 || cNode.NativeIndex < 0 || dNode.NativeIndex < 0) continue;
            PointD a = aNode.Point, b = bNode.Point, c = cNode.Point, d = dNode.Point;
            double ux = b.x - a.x, uy = b.y - a.y, vx = c.x - b.x, vy = c.y - b.y,
                wx = d.x - c.x, wy = d.y - c.y;
            if (ux * vx + uy * vy >= 0 || vx * wx + vy * wy >= 0) continue;
            double outgoingSquared = wx * wx + wy * wy;
            if (outgoingSquared <= 1e-24) continue;
            double position = Math.Clamp(((b.x - c.x) * wx + (b.y - c.y) * wy) / outgoingSquared, 0, 1);
            double x = (b.x + c.x + position * wx) / 2, y = (b.y + c.y + position * wy) / 2;
            if (!double.IsFinite(x) || !double.IsFinite(y)
                || Math.Abs(x * PathUtils.Scale) >= long.MaxValue || Math.Abs(y * PathUtils.Scale) >= long.MaxValue)
            { rejected++; continue; }
            var midpoint64 = ToPoint(new PointD(x, y));
            var midpoint = new PointD(midpoint64.X / PathUtils.Scale, midpoint64.Y / PathUtils.Scale);
            double firstX = midpoint.x - a.x, firstY = midpoint.y - a.y;
            double lastX = d.x - midpoint.x, lastY = d.y - midpoint.y;
            if (firstX * lastX + firstY * lastY <= 0) continue;
            var oldEdges = new List<(PointD A, PointD B)>();
            for (int point = aNode.NativeIndex; point != dNode.NativeIndex; point = (point + 1) % count)
            {
                if ((point & 255) == 0) token.ThrowIfCancellationRequested();
                oldEdges.Add((native[point], native[(point + 1) % count]));
            }
            var newEdges = new[] { (A: a, B: midpoint), (A: midpoint, B: d) };
            if (oldEdges.Any(edge => !SegmentCovered(edge.A, edge.B, newEdges, epsilon))
                || newEdges.Any(edge => !SegmentCovered(edge.A, edge.B, oldEdges, epsilon)))
            {
                rejected++;
                continue;
            }
            bool safe = true;
            foreach (var edge in newEdges)
            {
                queries++;
                Point64 from = ToPoint(edge.A), to = ToPoint(edge.B);
                if (original.IsInside(from) && original.IsInside(to)
                    && original.ClearanceViolation(from, to, required) is null) continue;
                unsafeChords++; safe = false; break;
            }
            if (!safe) continue;
            nodes[bPosition] = (midpoint, -1);
            nodes.RemoveAt(cPosition);
            removed++;
            index = -1;
        }
        return nodes.Select(node => node.Point).ToList();
    }

    // Intersect the complete query segment with each closed epsilon capsule.
    // A capsule is its oriented rectangle plus both endpoint discs; unioning
    // parameter intervals proves coverage between vertices as well as at them.
    private static bool SegmentCovered(PointD start, PointD end,
        IEnumerable<(PointD A, PointD B)> covering, double epsilon)
    {
        double dx = end.x - start.x, dy = end.y - start.y, querySquared = dx * dx + dy * dy;
        var intervals = new List<(double Start, double End)>();
        foreach (var edge in covering)
        {
            double vx = edge.B.x - edge.A.x, vy = edge.B.y - edge.A.y, lengthSquared = vx * vx + vy * vy;
            if (querySquared <= 1e-24)
            {
                double position = lengthSquared <= 1e-24 ? 0
                    : Math.Clamp(((start.x - edge.A.x) * vx + (start.y - edge.A.y) * vy) / lengthSquared, 0, 1);
                double ex = start.x - edge.A.x - position * vx, ey = start.y - edge.A.y - position * vy;
                if (ex * ex + ey * ey <= epsilon * epsilon) return true;
                continue;
            }
            AddDisc(edge.A);
            AddDisc(edge.B);
            if (lengthSquared <= 1e-24) continue;
            double min = 0, max = 1;
            double px = start.x - edge.A.x, py = start.y - edge.A.y;
            double longitudinal = px * vx + py * vy, longitudinalDelta = dx * vx + dy * vy;
            double perpendicular = px * vy - py * vx, perpendicularDelta = dx * vy - dy * vx;
            double width = epsilon * Math.Sqrt(lengthSquared);
            if (ClipLinear(longitudinal, longitudinalDelta, 0, lengthSquared, ref min, ref max)
                && ClipLinear(perpendicular, perpendicularDelta, -width, width, ref min, ref max))
                intervals.Add((min, max));
        }
        intervals.Sort((first, second) => first.Start.CompareTo(second.Start));
        double covered = 0;
        foreach (var interval in intervals)
        {
            if (interval.Start > covered + 1e-12) return false;
            covered = Math.Max(covered, interval.End);
            if (covered >= 1 - 1e-12) return true;
        }
        return false;

        void AddDisc(PointD center)
        {
            double px = start.x - center.x, py = start.y - center.y;
            double linear = px * dx + py * dy;
            double constant = px * px + py * py - epsilon * epsilon;
            double discriminant = linear * linear - querySquared * constant;
            if (discriminant < 0) return;
            double root = Math.Sqrt(discriminant);
            double first = Math.Max(0, (-linear - root) / querySquared);
            double last = Math.Min(1, (-linear + root) / querySquared);
            if (first <= last) intervals.Add((first, last));
        }
        static bool ClipLinear(double origin, double delta, double minimum, double maximum, ref double first, ref double last)
        {
            if (Math.Abs(delta) <= 1e-24) return origin >= minimum && origin <= maximum;
            double a = (minimum - origin) / delta, b = (maximum - origin) / delta;
            first = Math.Max(first, Math.Min(a, b));
            last = Math.Min(last, Math.Max(a, b));
            return first <= last;
        }
    }

    private static bool SameTopology(IReadOnlyList<List<PointD>> native,
        IReadOnlyList<List<PointD>> reduced, CancellationToken token)
    {
        var before = PolygonOps.DifferenceToComponents(native, Array.Empty<IReadOnlyList<PointD>>());
        var after = PolygonOps.DifferenceToComponents(reduced, Array.Empty<IReadOnlyList<PointD>>());
        if (before.Count != after.Count)
        {
            Console.WriteLine($"[OffsetBoundaryCleanup.Topology] reason=component-count before={before.Count} after={after.Count} "
                + $"beforeHoles={before.Sum(component => component.Holes.Count)} afterHoles={after.Sum(component => component.Holes.Count)}");
            return false;
        }
        var matched = new HashSet<int>();
        foreach (var component in before)
        {
            token.ThrowIfCancellationRequested();
            var overlaps = Enumerable.Range(0, after.Count).Where(index =>
                BoundsOverlap(component.Outer, after[index].Outer)
                && Math.Abs(PolygonOps.Intersect(
                    new[] { (IReadOnlyList<PointD>)component.Outer }.Concat(component.Holes),
                    new[] { (IReadOnlyList<PointD>)after[index].Outer }.Concat(after[index].Holes))
                    .Sum(PolygonOps.SignedArea)) > 0).ToArray();
            if (overlaps.Length != 1 || !matched.Add(overlaps[0]))
            {
                Console.WriteLine($"[OffsetBoundaryCleanup.Topology] reason=filled-component-identity overlaps={overlaps.Length} "
                    + $"area={PolygonOps.SignedArea(component.Outer):R} bounds=({component.Outer.Min(point => point.x):R},"
                    + $"{component.Outer.Min(point => point.y):R})-({component.Outer.Max(point => point.x):R},{component.Outer.Max(point => point.y):R})");
                return false;
            }
            var other = after[overlaps[0]];
            if (component.Holes.Count != other.Holes.Count) return false;
            var holes = new HashSet<int>();
            foreach (var hole in component.Holes)
            {
                token.ThrowIfCancellationRequested();
                var holeOverlaps = Enumerable.Range(0, other.Holes.Count)
                    .Where(index => Overlaps(hole, other.Holes[index])).ToArray();
                if (holeOverlaps.Length != 1 || !holes.Add(holeOverlaps[0])) return false;
            }
        }
        return true;
    }

    private static bool Overlaps(IReadOnlyList<PointD> first, IReadOnlyList<PointD> second)
    {
        if (!BoundsOverlap(first, second)) return false;
        return Math.Abs(PolygonOps.Intersect(new[] { first }, new[] { second }).Sum(PolygonOps.SignedArea)) > 0;
    }

    private static bool BoundsOverlap(IReadOnlyList<PointD> first, IReadOnlyList<PointD> second)
    {
        return !(first.Max(point => point.x) < second.Min(point => point.x)
            || second.Max(point => point.x) < first.Min(point => point.x)
            || first.Max(point => point.y) < second.Min(point => point.y)
            || second.Max(point => point.y) < first.Min(point => point.y));
    }

    private static Point64 ToPoint(PointD point) => new(
        (long)Math.Round(point.x * PathUtils.Scale), (long)Math.Round(point.y * PathUtils.Scale));
}
