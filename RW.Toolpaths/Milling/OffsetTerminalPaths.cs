using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>
/// Retains the terminal branches of an offset wavefront as open cutter paths.
/// A connected offset component invokes this only after its next front has
/// collapsed. One cached source graph supplies native terminal ridges and
/// native connectors; surviving front spacing belongs to the offset collector.
/// </summary>
internal static class OffsetTerminalPaths
{
    /// <summary>Untrusted terminal nominations for a variable-depth fill. Unlike
    /// BuildCollapsedComponent this grants no swept-coverage certificate: the
    /// caller must apply its original three-dimensional cutter guards. Reuse the
    /// source diagram and native graph joins, without an adaptive two-dimensional
    /// coverage audit that cannot certify this variable-depth operation.</summary>
    internal static IReadOnlyList<IReadOnlyList<PointD>> NominateRidges(
        MillingRegion source, double minimumInset, double maximumInset,
        CancellationToken token,IReadOnlyList<MedialSegment>? preparedMedial=null)
    {
        var medial = preparedMedial ?? BuildSourceMedial(source, token, boundOriginalSource: true);
        var chords = new List<(PointD A, PointD B)>();
        foreach (var edge in medial)
        {
            token.ThrowIfCancellationRequested();
            double r0 = edge.Point0.Radius, dr = edge.Point1.Radius - r0;
            double lo = 0, hi = 1;
            if (Math.Abs(dr) < 1e-12)
            { if (r0 < minimumInset || r0 > maximumInset) continue; }
            else
            {
                double a = (minimumInset-r0)/dr, b = (maximumInset-r0)/dr;
                lo = Math.Max(lo, Math.Min(a,b)); hi = Math.Min(hi, Math.Max(a,b));
                if (lo >= hi) continue;
            }
            PointD At(double t) => new(edge.Point0.X+(edge.Point1.X-edge.Point0.X)*t,
                edge.Point0.Y+(edge.Point1.Y-edge.Point0.Y)*t);
            chords.Add((At(lo), At(hi)));
        }
        var paths = Join(chords).Cast<IReadOnlyList<PointD>>().ToList();
        Console.WriteLine($"[Offset.Terminal.Nominations] sourceMedial={medial.Count} preparedMedialReused={preparedMedial is not null} retainedChords={chords.Count} paths={paths.Count} minimumInset={minimumInset:R} maximumInset={maximumInset:R} stockCredit=False originalProtectionRequired=True");
        return paths;
    }

    private const double GridReserve = 4 / PathUtils.Scale;
    // Comparing independently sampled round sweeps can accumulate both chord
    // sagittae; reserve their sum when proving a branch contributes material.
    private static double SweepComparisonError(double arc) => 2 * arc + GridReserve;

    internal static IReadOnlyList<MedialSegment> BuildSourceMedial(MillingRegion source, CancellationToken token,
        bool boundOriginalSource = false, bool preserveNativeGrid = false)
    {
        token.ThrowIfCancellationRequested();
        if (source.Outer.Count < 3) return Array.Empty<MedialSegment>();
        double span = Math.Max(source.Outer.Max(point => point.x) - source.Outer.Min(point => point.x),
            source.Outer.Max(point => point.y) - source.Outer.Min(point => point.y));
        if (boundOriginalSource)
            return MedialAxisGeometry.BuildBoundedSource(source.Outer, source.Holes,
                .01, Math.Max(1, span * 2), Math.PI, token, preserveNativeGrid: preserveNativeGrid);
        return MedialAxisGeometry.Build(source.Outer, source.Holes, BoostVoronoiProvider.CreateDefault(),
            .01, Math.Max(1, span * 2), Math.PI, token, preserveNativeGrid: preserveNativeGrid);
    }

    /// <summary>
    /// Reduces one offset component whose next front has no surviving child.
    /// The current front supplies its geometry; source ridges supply only native
    /// terminal paths and connectors. Live offset components never call this.
    /// </summary>
    internal static IReadOnlyList<IReadOnlyList<PointD>> BuildCollapsedComponent(
        CollapsedOffsetComponent context,
        Func<PreparedTerminalMedial> sourceMedial,
        double firstInset,
        IReadOnlyList<PointD>? clipBoundary,
        CancellationToken token)
        => BuildCollapsedComponentCore(context, sourceMedial, firstInset, clipBoundary, token, false);

    private static IReadOnlyList<IReadOnlyList<PointD>> BuildCollapsedComponentCore(
        CollapsedOffsetComponent context, Func<PreparedTerminalMedial> sourceMedial,
        double firstInset, IReadOnlyList<PointD>? clipBoundary, CancellationToken token, bool forceFull)
    {
        const double arc = .001;
        long started = PerfLog.Start();
        long coreStarted = PerfLog.Start();
        var component = context.CurrentComponent;
        var material = PolygonOps.Difference(new[] { component.Outer }, component.Holes);
        var core = PolygonOps.Offset(material, -context.SweepRadius, arc, 0);
        if (clipBoundary is { Count: >= 3 })
            core = PolygonOps.Intersect(core, new[] { clipBoundary });
        PerfLog.Stop("Offset.Terminal.Core", coreStarted, $"contours={material.Count} coreContours={core.Count}");
        if (core.Count == 0)
        {
            Console.WriteLine($"[Offset.Terminal.Component] region={context.RegionIndex} parentLevel={context.ParentLevel} "
                + $"offset={context.CurrentOffsetDistance:R} candidateOffset={context.CandidateOffsetDistance:R} "
                + $"result=current-front-already-covers-component source=collapsed-component");
            PerfLog.Stop("Offset.Terminal.Component", started, "core=empty medialRequested=False");
            return Array.Empty<IReadOnlyList<PointD>>();
        }
        var prepared = sourceMedial();
        var localMedial = forceFull ? prepared.FullIntersecting(PathUtils.ToClipper(material), token)
            : prepared.Intersecting(PathUtils.ToClipper(material), token);
        bool localSites = prepared.LastQueryWasLocal;
        long clipStarted = PerfLog.Start();
        var native = TraceNative(localMedial, material, prepared.Boundary,
            Math.Max(0, firstInset), clipBoundary, token, out int medialCount, out int rejected);
        PerfLog.Stop("Offset.Terminal.Clip", clipStarted, $"segments={medialCount} fragments={native.Count}");
        long selectStarted = PerfLog.Start();
        var componentBoundary = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(material));
        var fullChords = native.SelectMany(path => Enumerable.Range(1, path.Count - 1)
            .Select(index => (path[index - 1], path[index]))).ToList();
        var (_, adjacent) = Graph(fullChords);
        double frontTolerance = (arc + GridReserve) * PathUtils.Scale;
        bool IsFrontLeaf(PointD point) => adjacent[ToPoint(point)].Count == 1
            && componentBoundary.DistanceSquared(ToPoint(point)) <= frontTolerance * frontTolerance;
        bool IsCornerTrajectory(List<PointD> path)
            => (IsFrontLeaf(path[0]) && adjacent[ToPoint(path[^1])].Count >= 3)
                || (IsFrontLeaf(path[^1]) && adjacent[ToPoint(path[0])].Count >= 3);
        int cornerTrajectories = native.Count(IsCornerTrajectory);
        // Select whole native fragments. Clipping the commanded path to its
        // already-cut / uncut radius boundary would fracture one sliver into
        // independent microstrokes. Core membership is only a need witness.
        var candidates = native.Where(path => !IsCornerTrajectory(path))
            .Where(path => ClipOpen(new[] { path }, core, ClipType.Intersection)
            .Any(piece => piece.Count >= 2)).ToList();
        var currentFronts = new[] { component.Outer }.Concat(component.Holes).ToList();
        List<List<PointD>> covered;
        if (clipBoundary is { Count: >= 3 })
        {
            var closedAsOpen = currentFronts.Select(path =>
                (IReadOnlyList<PointD>)path.Concat(new[] { path[0] }).ToList());
            covered = PolygonOps.BufferOpenCenterlines(
                ClipOpen(closedAsOpen, new[] { clipBoundary }, ClipType.Intersection),
                context.SweepRadius, arc, token);
        }
        else covered = PolygonOps.BufferCenterlines(currentFronts, context.SweepRadius, arc);
        var retained = PruneAgainstSweep(candidates, covered, context.SweepRadius, arc, token, out int redundant);
        var retainedChords = retained.SelectMany(path => Enumerable.Range(1, path.Count - 1)
            .Select(index => (path[index - 1], path[index]))).ToList();
        var connected = RestoreNativeConnectors(fullChords, retainedChords, token,
            out int restored, out int nativeComponents);
        var paths = JoinTrails(connected, token).Select(trail => (IReadOnlyList<PointD>)trail.Points).ToList();
        if (localSites)
        {
            // The local diagram may alter chord partition and native traversal
            // outside the query. Accept only actual original-wall-proved paths
            // that close the entire current component under the SAME collector
            // coverage reserve. Unexplained material forces the established full
            // original-site graph, rather than advancing a partial local branch.
            var localSweep = PolygonOps.BufferOpenCenterlines(paths, context.SweepRadius, arc, token);
            var missing = PolygonOps.Difference(material, PolygonOps.Union(covered.Concat(localSweep)));
            var interior = PolygonOps.Offset(missing, -(.003 + GridReserve), arc / 4, 0);
            bool certified = interior.Count == 0;
            Console.WriteLine($"[Offset.Terminal.LocalCoverage] region={context.RegionIndex} "
                + $"level={context.ParentLevel} paths={paths.Count} unresolvedContours={interior.Count} "
                + $"reserve={.003 + GridReserve:R} certified={certified} originalWalls=unchanged "
                + $"completeCurrentMaterial=True fallback=full-original-sites");
            if (!certified) return BuildCollapsedComponentCore(context, sourceMedial, firstInset, clipBoundary, token, true);
        }
        Console.WriteLine($"[Offset.Terminal.Component] region={context.RegionIndex} parentLevel={context.ParentLevel} "
            + $"offset={context.CurrentOffsetDistance:R} candidateOffset={context.CandidateOffsetDistance:R} "
            + $"source=collapsed-component originalMedial={prepared.MaterializedSegmentCount} localOriginalSites={localSites} "
            + $"candidateMedial={medialCount} nativeFragments={native.Count} "
            + $"cornerTrajectories={cornerTrajectories} "
            + $"neededFragments={candidates.Count} retainedFragments={retained.Count} redundant={redundant} "
            + $"restoredNativeEdges={restored} nativeComponents={nativeComponents} paths={paths.Count} "
            + $"rejectedChords={rejected} sweepComparisonError={SweepComparisonError(arc):R} "
            + $"endpointExtrapolation=0 inventedBridges=0");
        PerfLog.Stop("Offset.Terminal.Select", selectStarted, $"candidates={candidates.Count} retained={retained.Count} paths={paths.Count}");
        PerfLog.Stop("Offset.Terminal.Component", started, $"core=nonempty medialRequested=True paths={paths.Count}");
        return paths;
    }

    internal static IReadOnlyList<IReadOnlyList<PointD>> BuildExactFirstCollapse(
        MillingRegion source,
        IReadOnlyList<MedialSegment> sourceMedial,
        double firstInset,
        double sweepRadius,
        IReadOnlyList<PointD>? clipBoundary,
        CancellationToken token)
    {
        const double arc = .001;
        double radius = Math.Max(0, firstInset);
        var material = PolygonOps.Difference(new[] { source.Outer }, source.Holes);
        var boundary = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(material));
        // Only whole tangent branches qualify. A radius-changing corner edge
        // that touches this radius at one node is not an exact collapsed front.
        var tangent = sourceMedial.Where(segment => segment.Point0.Radius >= radius - GridReserve
            && segment.Point1.Radius >= radius - GridReserve
            && segment.Point0.Radius <= radius + arc
            && segment.Point1.Radius <= radius + arc).ToList();
        // The ORIGINAL native endpoint radii define this tangent front. Another
        // polygon erosion cannot certify a line-valued locus: cap construction
        // reserve can consume a positive legal sliver before tracing it. Retain
        // whole eligible native branches and prove their actual source capsules.
        var native = TraceNative(tangent, material, boundary, radius, clipBoundary, token,
            out _, out int rejected);
        var paths = JoinTrails(native.SelectMany(path => Enumerable.Range(1, path.Count - 1)
            .Select(index => (path[index - 1], path[index]))), token)
            .Select(trail => (IReadOnlyList<PointD>)trail.Points).ToList();
        Console.WriteLine($"[Offset.Terminal.ExactFirst] firstInset={firstInset:R} sweepRadius={sweepRadius:R} "
            + $"originalMedial={sourceMedial.Count} tangentChords={tangent.Count} nativeFragments={native.Count} paths={paths.Count} "
            + $"rejectedChords={rejected} endpointExtrapolation=0 inventedBridges=0");
        return paths;
    }

    /// <summary>Connects retained pieces exclusively through original edges inside their component.</summary>
    internal static List<(PointD A, PointD B)> RestoreNativeConnectors(
        IEnumerable<(PointD A, PointD B)> fullNative,
        IEnumerable<(PointD A, PointD B)> retainedNative,
        CancellationToken token,
        out int restored,
        out int nativeComponents)
    {
        var (edges, adjacent) = Graph(fullNative);
        var retainedKeys = Graph(retainedNative).Edges.ToHashSet();
        var selected = edges.Select(edge => retainedKeys.Contains(edge)
            || retainedKeys.Contains((edge.B, edge.A))).ToArray();
        var accounted = new HashSet<Point64>();
        var lengths = edges.Select(edge => Distance(World(edge.A), World(edge.B))).ToArray();
        int added = 0, components = 0;
        for (int first = 0; first < edges.Count; first++)
        {
            token.ThrowIfCancellationRequested();
            if (!selected[first] || accounted.Contains(edges[first].A)) continue;
            var connected = new HashSet<Point64> { edges[first].A };
            GrowSelected(connected);
            components++;
            while (FindConnector(connected, out Point64 target,
                       out Dictionary<Point64, (Point64 Previous, int Edge)> parents))
            {
                Point64 cursor = target;
                while (!connected.Contains(cursor))
                {
                    var parent = parents[cursor];
                    if (!selected[parent.Edge]) { selected[parent.Edge] = true; added++; }
                    cursor = parent.Previous;
                }
                GrowSelected(connected);
            }
            accounted.UnionWith(connected);
        }
        restored = added;
        nativeComponents = components;
        return Enumerable.Range(0, edges.Count).Where(index => selected[index])
            .Select(index => (World(edges[index].A), World(edges[index].B))).ToList();

        void GrowSelected(HashSet<Point64> connected)
        {
            var pending = new Queue<Point64>(connected);
            while (pending.TryDequeue(out Point64 point))
            {
                token.ThrowIfCancellationRequested();
                foreach (int index in adjacent[point])
                {
                    if (!selected[index]) continue;
                    Point64 next = edges[index].A == point ? edges[index].B : edges[index].A;
                    if (connected.Add(next)) pending.Enqueue(next);
                }
            }
        }

        bool FindConnector(HashSet<Point64> connected, out Point64 target,
            out Dictionary<Point64, (Point64 Previous, int Edge)> parents)
        {
            var distances = connected.ToDictionary(point => point, _ => 0.0);
            parents = new Dictionary<Point64, (Point64, int)>();
            var queue = new PriorityQueue<Point64, double>();
            foreach (Point64 point in connected) queue.Enqueue(point, 0);
            while (queue.TryDequeue(out Point64 point, out double distance))
            {
                token.ThrowIfCancellationRequested();
                if (distance > distances[point]) continue;
                if (!connected.Contains(point) && adjacent[point].Any(index => selected[index]))
                {
                    target = point;
                    return true;
                }
                foreach (int index in adjacent[point])
                {
                    Point64 next = edges[index].A == point ? edges[index].B : edges[index].A;
                    double candidate = distance + lengths[index];
                    if (distances.TryGetValue(next, out double previous) && candidate >= previous) continue;
                    distances[next] = candidate;
                    parents[next] = (point, index);
                    queue.Enqueue(next, candidate);
                }
            }
            target = default;
            return false;
        }
    }

    private static List<List<PointD>> PruneAgainstSweep(IReadOnlyList<List<PointD>> traces,
        List<List<PointD>> covered, double radius, double arc, CancellationToken token, out int redundant)
    {
        // The proof selects native branches; uncovered stock never supplies
        // geometry or a new connecting chord.
        var retained = new List<List<PointD>>();
        foreach (var trace in traces.OrderByDescending(Length))
        {
            token.ThrowIfCancellationRequested();
            var contribution = PolygonOps.BufferOpenCenterlines(new[] { trace }, radius, arc, token);
            var newMaterial = PolygonOps.Difference(contribution, covered);
            var interior = PolygonOps.Offset(newMaterial, -SweepComparisonError(arc), arc / 4, 0);
            if (interior.Count == 0) continue;
            retained.Add(trace);
            covered = PolygonOps.Union(covered.Concat(contribution));
        }
        redundant = traces.Count - retained.Count;
        return retained;
    }

    private static List<List<PointD>> TraceNative(
        IReadOnlyList<MedialSegment> medial,
        IReadOnlyList<List<PointD>> component,
        PolygonOps.BoundarySegmentIndex source,
        double requiredRadius,
        IReadOnlyList<PointD>? clipBoundary,
        CancellationToken token,
        out int medialCount,
        out int rejected)
    {
        medialCount = medial.Count;
        var candidates = medial.Select(segment => (IReadOnlyList<PointD>)new List<PointD>
        {
            new(segment.Point0.X, segment.Point0.Y), new(segment.Point1.X, segment.Point1.Y),
        }).ToList();
        candidates = ClipOpen(candidates, component, ClipType.Intersection);
        if (clipBoundary is { Count: >= 3 })
            candidates = ClipOpen(candidates, new[] { clipBoundary }, ClipType.Intersection);
        var safe = new List<(PointD A, PointD B)>();
        int refused = 0;
        foreach (var candidate in candidates)
        for (int index = 1; index < candidate.Count; index++)
        {
            token.ThrowIfCancellationRequested();
            Retain(candidate[index - 1], candidate[index], 0);
        }
        var joined = Join(safe);
        rejected = refused;
        return joined;

        bool Allows(PointD a, PointD b) => source.ContainsCapsule(ToPoint(a), ToPoint(b),
            // ContainsCapsule reserves four ticks internally. Retain only the
            // established NET four-tick original-wall quantization reserve;
            // curve approximation is not permission to reduce stock leave.
            Math.Max(0, requiredRadius * PathUtils.Scale - 8));

        void Retain(PointD a, PointD b, int subdivision)
        {
            if (Same(a, b)) return;
            if (Allows(a, b)) { safe.Add((a, b)); return; }
            // A sampled parabola can shave an original wall between its endpoints.
            // Retain independently proved portions, without bridging rejected gaps.
            if (subdivision >= 12 || Distance(a, b) <= .001) { refused++; return; }
            PointD midpoint = new((a.x + b.x) / 2, (a.y + b.y) / 2);
            Retain(a, midpoint, subdivision + 1);
            Retain(midpoint, b, subdivision + 1);
        }

    }

    private static List<IReadOnlyList<PointD>> ClipOpen(
        IEnumerable<IReadOnlyList<PointD>> paths,
        IEnumerable<IReadOnlyList<PointD>> clips,
        ClipType type)
    {
        var subject = PathUtils.ToClipper(paths);
        var clip = PathUtils.ToClipper(clips);
        if (subject.Count == 0 || clip.Count == 0)
            return type == ClipType.Difference
                ? PathUtils.FromClipper(subject).Cast<IReadOnlyList<PointD>>().ToList()
                : new List<IReadOnlyList<PointD>>();
        var engine = new Clipper64();
        engine.AddOpenSubject(subject);
        engine.AddClip(clip);
        var open = new Paths64();
        engine.Execute(type, FillRule.NonZero, new Paths64(), open);
        return PathUtils.FromClipper(open).Cast<IReadOnlyList<PointD>>().ToList();
    }

    private static List<List<PointD>> Join(IEnumerable<(PointD A, PointD B)> source)
    {
        var (edges, adjacent) = Graph(source);
        var result = new List<List<PointD>>();
        var visited = new bool[edges.Count];
        foreach (var node in adjacent.Where(node => node.Value.Count != 2).Concat(adjacent.Where(node => node.Value.Count == 2)))
        foreach (int first in node.Value)
        {
            if (visited[first]) continue;
            var path = new List<PointD> { World(node.Key) };
            Point64 current = node.Key;
            int edge = first;
            while (!visited[edge])
            {
                visited[edge] = true;
                current = edges[edge].A == current ? edges[edge].B : edges[edge].A;
                path.Add(World(current));
                if (adjacent[current].Count != 2) break;
                int next = adjacent[current].FirstOrDefault(index => !visited[index], -1);
                if (next < 0) break;
                edge = next;
            }
            result.Add(path);
        }
        return result;
    }

    internal sealed record NativeTrail(List<PointD> Points);

    internal static List<NativeTrail> JoinTrails(IEnumerable<(PointD A, PointD B)> source, CancellationToken token)
    {
        var (edges, adjacent) = Graph(source);
        var lengths = edges.Select(edge => Distance(World(edge.A), World(edge.B))).ToArray();
        var used = new bool[edges.Count];
        var result = new List<NativeTrail>();
        for (int first = 0; first < edges.Count; first++)
        {
            token.ThrowIfCancellationRequested();
            if (used[first]) continue;
            Point64 a = Farthest(edges[first].A, out _, out _);
            Point64 b = Farthest(a, out var parents, out double spineLength);
            var spine = new List<Point64> { b };
            Point64 cursor = b;
            while (cursor != a)
            {
                var parent = parents[cursor];
                used[parent.Edge] = true;
                cursor = parent.Previous;
                spine.Add(cursor);
            }
            spine.Reverse();
            var points = new List<PointD> { World(a) };
            int originalEdges = 0, retraces = 0;
            foreach (Point64 node in spine)
            {
                if (!Same(points[^1], World(node))) points.Add(World(node));
                WalkBranches(node);
            }
            result.Add(new NativeTrail(points));
            Console.WriteLine($"[Offset.Terminal.NativeTrail] points={points.Count} spineLength={spineLength:R} "
                + $"branchEdges={originalEdges} branchRetraces={retraces} "
                + $"startDegree={adjacent[a].Count} endDegree={adjacent[b].Count} bridges=0 endpointExtrapolation=0");

            void WalkBranches(Point64 start)
            {
                var stack = new List<(Point64 Node, int Next)> { (start, 0) };
                while (stack.Count > 0)
                {
                    token.ThrowIfCancellationRequested();
                    var frame = stack[^1];
                    List<int> incident = adjacent[frame.Node];
                    int index = frame.Next;
                    while (index < incident.Count && used[incident[index]]) index++;
                    if (index == incident.Count)
                    {
                        stack.RemoveAt(stack.Count - 1);
                        if (stack.Count > 0) { points.Add(World(stack[^1].Node)); retraces++; }
                        continue;
                    }
                    int edge = incident[index];
                    stack[^1] = (frame.Node, index + 1);
                    used[edge] = true;
                    Point64 next = edges[edge].A == frame.Node ? edges[edge].B : edges[edge].A;
                    points.Add(World(next));
                    originalEdges++;
                    stack.Add((next, 0));
                }
            }
        }
        return result;

        Point64 Farthest(Point64 start, out Dictionary<Point64, (Point64 Previous, int Edge)> parents, out double farthestDistance)
        {
            var distances = new Dictionary<Point64, double> { [start] = 0 };
            parents = new Dictionary<Point64, (Point64, int)>();
            var queue = new PriorityQueue<Point64, double>();
            queue.Enqueue(start, 0);
            while (queue.TryDequeue(out Point64 node, out double distance))
            {
                token.ThrowIfCancellationRequested();
                if (distance > distances[node]) continue;
                foreach (int edge in adjacent[node])
                {
                    Point64 next = edges[edge].A == node ? edges[edge].B : edges[edge].A;
                    double candidate = distance + lengths[edge];
                    if (distances.TryGetValue(next, out double existing) && candidate >= existing) continue;
                    distances[next] = candidate;
                    parents[next] = (node, edge);
                    queue.Enqueue(next, candidate);
                }
            }
            var leaves = distances.Keys.Where(node => adjacent[node].Count == 1).ToList();
            var eligible = leaves.Count >= 2 ? leaves : distances.Keys.ToList();
            Point64 farthest = eligible.OrderByDescending(node => distances[node]).First();
            farthestDistance = distances[farthest];
            return farthest;
        }
    }

    private static (List<(Point64 A, Point64 B)> Edges, Dictionary<Point64, List<int>> Adjacent)
        Graph(IEnumerable<(PointD A, PointD B)> source)
    {
        var edges = new List<(Point64 A, Point64 B)>();
        var unique = new HashSet<(Point64 A, Point64 B)>();
        var adjacent = new Dictionary<Point64, List<int>>();
        foreach (var (a, b) in source)
        {
            Point64 qa = ToPoint(a), qb = ToPoint(b);
            if (qa == qb || unique.Contains((qb, qa)) || !unique.Add((qa, qb))) continue;
            int index = edges.Count;
            edges.Add((qa, qb));
            foreach (var point in new[] { qa, qb })
            {
                if (!adjacent.TryGetValue(point, out var incident)) adjacent.Add(point, incident = new List<int>());
                incident.Add(index);
            }
        }
        return (edges, adjacent);
    }

    private static Point64 ToPoint(PointD point) => new((long)Math.Round(point.x * PathUtils.Scale),
        (long)Math.Round(point.y * PathUtils.Scale));
    private static PointD World(Point64 point) => new(point.X / PathUtils.Scale, point.Y / PathUtils.Scale);
    private static bool Same(PointD a, PointD b) => ToPoint(a) == ToPoint(b);
    private static double Distance(PointD a, PointD b) => Math.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    private static double Length(List<PointD> points) => Enumerable.Range(1, points.Count - 1)
        .Sum(index => Distance(points[index - 1], points[index]));
}
