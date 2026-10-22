using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

// Parent offsets and local windows only nominate commands. Complete original
// finite walls and the caller's unchanged whole-shell morphology decide them.
// This owner publishes no paths until every proof completes.
internal static class ParentLocalRescue
{
    internal const double Arc = .001;
    internal const double Error = .003 + 4 / PathUtils.Scale;
    internal sealed record Input(
        PolygonOps.BoundarySegmentIndex Source,
        IReadOnlyList<PolygonComponent> Parents, List<List<PointD>> Requested,
        List<List<PointD>> Shell, List<List<PointD>> Covered, List<List<PointD>> Core,
        IReadOnlyList<PointD>? Clip,
        double CurrentDistance, double RequestedDistance, double Radius);
    internal sealed record Limits(int Owners = 4, int ParentPoints = 8192,
        int CorePoints = 8192, int TracePoints = 8192, int Attempts = 4,
        int ParentComponents = 256, int ParentInputPoints = 131072,
        int OwnerChecks = 4096, int InputPoints = 1048576, int InputContours = 16384);
    internal sealed record Result(bool Accepted, string Reason,
        List<List<PointD>> Traces, double RequestedDistance,
        double GapBefore, double GapAfter, int Owners, int ParentPoints,
        int Attempts, long OffsetPoints, int TracePoints, long SourceChords,
        int SupportRefusals, double OffsetMs, double ProofMs, double TotalMs);

    internal static Result Try(Input input, CancellationToken token,
        Func<IEnumerable<IReadOnlyList<PointD>>, IEnumerable<IReadOnlyList<PointD>>, double> audit,
        Limits? limits = null, CheckedOpenClip.Execute? executeClip = null)
    {
        token.ThrowIfCancellationRequested();
        limits ??= new();
        long started = Stopwatch.GetTimestamp();
        double before = double.NaN, after = double.NaN, offsetMs = 0, proofMs = 0;
        int ownersCount = 0, parentPoints = 0, attempts = 0, tracePoints = 0, supportRefusals = 0;
        long offsetPoints = 0, sourceChords = 0;
        Result Refuse(string reason) => new(false, reason, new(), input.RequestedDistance,
            before, after, ownersCount, parentPoints, attempts, offsetPoints,
            tracePoints, sourceChords, supportRefusals, offsetMs, proofMs,
            Stopwatch.GetElapsedTime(started).TotalMilliseconds);
        if (limits.Attempts <= 0 || limits.Owners <= 0 || limits.ParentPoints <= 0
            || limits.CorePoints <= 0 || limits.TracePoints <= 0 || limits.ParentComponents <= 0
            || limits.ParentInputPoints <= 0 || limits.OwnerChecks <= 0 || limits.InputPoints <= 0
            || limits.InputContours <= 0) return Refuse("budget");
        // Guard complete input and ownership work BEFORE materializing parent
        // wrappers or scanning finite coordinates. Selected-owner budgets alone
        // cannot bound a domain with many irrelevant parents or core pieces.
        if (input.Parents.Count > limits.ParentComponents) return Refuse("parent-input-budget");
        long parentContours = input.Parents.Sum(parent => 1L + parent.Holes.Count);
        long contours = input.Requested.Count + (long)input.Shell.Count + input.Covered.Count + input.Core.Count
            + parentContours + (input.Clip is null ? 0 : 1);
        if (contours > limits.InputContours) return Refuse("input-budget");
        long allParentPoints = input.Parents.Sum(parent => parent.Outer.Count + parent.Holes.Sum(p => (long)p.Count));
        if (allParentPoints > limits.ParentInputPoints) return Refuse("parent-input-budget");
        if (input.Core.Sum(p => (long)p.Count) > limits.CorePoints) return Refuse("core-budget");
        long allPoints = allParentPoints + input.Requested.Sum(p => (long)p.Count)
            + input.Shell.Sum(p => (long)p.Count) + input.Covered.Sum(p => (long)p.Count)
            + input.Core.Sum(p => (long)p.Count) + (input.Clip?.Count ?? 0);
        if (allPoints > limits.InputPoints) return Refuse("input-budget");
        token.ThrowIfCancellationRequested();
        if (!Finite(input) || input.Radius <= 0 || input.CurrentDistance < 0
            || input.RequestedDistance <= input.CurrentDistance) return Refuse("invalid-input");
        var source = input.Source;
        // Every requested command remains source-backed before any rescue can
        // be used. Full shell and footprint operations retain all signed holes.
        if (!Walls(input.Requested.Select(Close).ToList(), source,
            input.RequestedDistance, token, out long requestedChords)) return Refuse("requested-original-wall");
        sourceChords += requestedChords;
        var covered = input.Covered;
        var shell = input.Shell;
        var core = input.Core;
        before = Area(core); after = before;
        if (before == 0) return Refuse("already-covered");
        if (!TryComponents(core, token, out var cores)) return Refuse("core-boolean-unresolved");
        if (cores.Count * (long)input.Parents.Count > limits.OwnerChecks) return Refuse("ownership-work-budget");
        var parents = input.Parents.Select(parent => new List<List<PointD>> { parent.Outer }.Concat(parent.Holes).ToList()).ToList();
        var ownerIds = new SortedSet<int>();
        foreach (var piece in cores)
        {
            token.ThrowIfCancellationRequested();
            int owner = -1;
            for (int i = 0; i < parents.Count; i++)
            {
                token.ThrowIfCancellationRequested();
                if (!TryBoolean(piece, parents[i], ClipType.Difference, token, out var outside))
                    return Refuse("owner-boolean-unresolved");
                if (outside.Count != 0) continue;
                if (owner >= 0) return Refuse("ambiguous-owner");
                owner = i;
            }
            if (owner < 0) return Refuse("missing-owner");
            ownerIds.Add(owner);
        }
        ownersCount = ownerIds.Count;
        long parentWork = ownerIds.Sum(i => parents[i].Sum(p => (long)p.Count));
        parentPoints = (int)Math.Min(int.MaxValue, parentWork);
        if (ownersCount == 0 || ownersCount > limits.Owners || parentWork > limits.ParentPoints)
            return Refuse("parent-budget");
        var windows = cores.Select(piece => Window(piece, input.Radius)).ToList();
        double advance = input.RequestedDistance - input.CurrentDistance;
        // The FIRST nominee is deterministically half the requested advance.
        // Further bounded nominees approach the current front, independently
        // of all captured safe distances, graphs, source cuts, or callbacks.
        for (int attempt = 0; attempt < limits.Attempts; attempt++)
        {
            token.ThrowIfCancellationRequested(); attempts++;
            double localAdvance = advance / Math.Pow(2, attempt + 1);
            double lowerDistance = input.CurrentDistance + localAdvance;
            if (localAdvance < 1 / PathUtils.Scale || lowerDistance == input.CurrentDistance)
                return Refuse("quantum");
            long offsetStarted = Stopwatch.GetTimestamp();
            var lower = ownerIds.SelectMany(i => PolygonOps.Offset(parents[i], -localAdvance, Arc, 0)).ToList();
            token.ThrowIfCancellationRequested();
            offsetMs += Stopwatch.GetElapsedTime(offsetStarted).TotalMilliseconds;
            offsetPoints += lower.Sum(p => (long)p.Count);
            if (lower.Sum(p => (long)p.Count) > limits.TracePoints * 4L) return Refuse("lower-budget");
            var originalOpen = lower.Where(p => p.Count >= 3).Select(Close).ToList();
            if (!TryClipOpen(originalOpen, windows, token, out var traces, executeClip)) return Refuse("clip-unresolved");
            if (input.Clip is not null
                && !TryClipOpen(traces, new[] { input.Clip }, token, out traces, executeClip)) return Refuse("clip-unresolved");
            traces = Deduplicate(traces);
            long traceWork = traces.Sum(p => (long)p.Count);
            tracePoints = (int)Math.Min(int.MaxValue, traceWork);
            if (traceWork > limits.TracePoints) return Refuse("trace-budget");
            if (traces.Count == 0) continue;
            var edges = originalOpen.SelectMany(p => Enumerable.Range(1, p.Count - 1)
                .Select(i => (A: p[i - 1], B: p[i]))).ToArray();
            bool supported = true;
            foreach (var path in traces)
            for (int i = 1; i < path.Count; i++)
            {
                token.ThrowIfCancellationRequested();
                if (edges.Any(edge => Supported(path[i - 1], path[i], edge.A, edge.B))) continue;
                supportRefusals++; supported = false;
            }
            if (!supported) return Refuse("artificial-clip-edge");
            if (!Walls(traces, source, lowerDistance, token, out long wallChords))
            { sourceChords += wallChords; continue; }
            sourceChords += wallChords;
            long proofStarted = Stopwatch.GetTimestamp();
            var actualRescue = PolygonOps.BufferOpenCenterlines(traces, input.Radius, Arc, token);
            if (!TryBoolean(covered.Concat(actualRescue), Array.Empty<IReadOnlyList<PointD>>(), ClipType.Union,
                token, out var finalCovered)) return Refuse("coverage-union-unresolved");
            after = audit(shell, finalCovered);
            proofMs += Stopwatch.GetElapsedTime(proofStarted).TotalMilliseconds;
            if (after != 0) continue;
            token.ThrowIfCancellationRequested();
            return new(true, "original-whole-shell-proven", traces, input.RequestedDistance,
                before, after, ownersCount, parentPoints, attempts, offsetPoints,
                tracePoints, sourceChords, supportRefusals, offsetMs, proofMs,
                Stopwatch.GetElapsedTime(started).TotalMilliseconds);
        }
        return Refuse("bounded-proof-refusal");
    }
    internal static double Area(IEnumerable<IReadOnlyList<PointD>> p) => Math.Abs(p.Sum(PolygonOps.SignedArea));
    internal static List<PointD> Close(List<PointD> p) => p.Concat(new[] { p[0] }).ToList();
    internal static bool TryClipOpen(IEnumerable<IReadOnlyList<PointD>> paths,
        IEnumerable<IReadOnlyList<PointD>> clips, CancellationToken token,
        out List<List<PointD>> output, CheckedOpenClip.Execute? execute = null)
        => CheckedOpenClip.Try(paths, clips, ClipType.Intersection, token, out output, execute);

    private static bool TryBoolean(IEnumerable<IReadOnlyList<PointD>> subjects,
        IEnumerable<IReadOnlyList<PointD>> clips, ClipType operation, CancellationToken token,
        out List<List<PointD>> output)
    {
        token.ThrowIfCancellationRequested();
        var engine = new Clipper64(); engine.AddSubject(PathUtils.ToClipper(subjects));
        engine.AddClip(PathUtils.ToClipper(clips)); var result = new Paths64();
        bool completed = engine.Execute(operation, FillRule.NonZero, result);
        token.ThrowIfCancellationRequested();
        output = completed ? PolygonOps.NormalizeOrder(PathUtils.FromClipper(result)) : new List<List<PointD>>();
        return completed;
    }

    private static bool TryComponents(List<List<PointD>> paths, CancellationToken token,
        out List<List<List<PointD>>> components)
    {
        token.ThrowIfCancellationRequested();
        components = new List<List<List<PointD>>>();
        var engine = new Clipper64(); engine.AddSubject(PathUtils.ToClipper(paths));
        var tree = new PolyTree64();
        bool completed = engine.Execute(ClipType.Difference, FillRule.NonZero, tree);
        token.ThrowIfCancellationRequested();
        if (!completed) return false;
        var collected = components;
        List<PointD> Convert(Path64? path) => path is null ? new List<PointD>()
            : path.Select(p => new PointD(p.X / PathUtils.Scale, p.Y / PathUtils.Scale)).ToList();
        void Collect(PolyPath64 outer)
        {
            token.ThrowIfCancellationRequested();
            var boundary = Convert(outer.Polygon);
            if (boundary.Count >= 3)
            {
                var component = new List<List<PointD>> { boundary };
                foreach (PolyPath64 hole in outer)
                {
                    var points = Convert(hole.Polygon);
                    if (points.Count >= 3) component.Add(points);
                }
                collected.Add(component);
            }
            foreach (PolyPath64 hole in outer)
                foreach (PolyPath64 island in hole) Collect(island);
        }
        foreach (PolyPath64 outer in tree) Collect(outer);
        return true;
    }
    internal static bool Walls(List<List<PointD>> paths, PolygonOps.BoundarySegmentIndex source,
        double distance, CancellationToken token, out long chords)
    {
        chords = 0; double required = Math.Max(0, distance * PathUtils.Scale - 4);
        foreach (var p in PathUtils.ToClipper(paths))
        for (int i = 1; i < p.Count; i++)
        {
            token.ThrowIfCancellationRequested(); chords++;
            if (!source.IsInside(p[i - 1]) || !source.IsInside(p[i])
                || source.ClearanceViolation(p[i - 1], p[i], required) is not null) return false;
        }
        return true;
    }
    private static List<PointD> Window(List<List<PointD>> piece, double radius)
    {
        var all = piece.SelectMany(p => p).ToArray(); double pad = radius + Error + Arc + 8 / PathUtils.Scale;
        return new() { new(all.Min(p => p.x) - pad, all.Min(p => p.y) - pad),
            new(all.Max(p => p.x) + pad, all.Min(p => p.y) - pad),
            new(all.Max(p => p.x) + pad, all.Max(p => p.y) + pad),
            new(all.Min(p => p.x) - pad, all.Max(p => p.y) + pad) };
    }
    private static bool Supported(PointD a, PointD b, PointD from, PointD to)
        => Distance(a, from, to) <= 2 / PathUtils.Scale && Distance(b, from, to) <= 2 / PathUtils.Scale;
    private static double Distance(PointD p, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y;
        double t = dx * dx + dy * dy == 0 ? 0
            : Math.Clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy), 0, 1);
        return Math.Sqrt(Math.Pow(p.x - a.x - t * dx, 2) + Math.Pow(p.y - a.y - t * dy, 2));
    }
    private static List<List<PointD>> Deduplicate(List<List<PointD>> input)
    {
        var seen = new HashSet<string>(); var result = new List<List<PointD>>();
        foreach (var path in input)
        {
            var points = PathUtils.ToClipper(new[] { path })[0];
            string forward = string.Join(";", points.Select(p => $"{p.X},{p.Y}"));
            string reverse = string.Join(";", points.AsEnumerable().Reverse().Select(p => $"{p.X},{p.Y}"));
            string key = string.CompareOrdinal(forward, reverse) <= 0 ? forward : reverse;
            if (seen.Add(key)) result.Add(path);
        }
        return result;
    }
    private const double MaximumIntegerCoordinate = 1_000_000_000_000d;
    private static bool Finite(Input input)
    {
        double pad = input.Radius + Error + Arc + 8 / PathUtils.Scale;
        if (!double.IsFinite(input.Radius) || input.Radius * PathUtils.Scale > MaximumIntegerCoordinate
            || !double.IsFinite(input.CurrentDistance) || !double.IsFinite(input.RequestedDistance)
            || input.RequestedDistance * PathUtils.Scale > MaximumIntegerCoordinate
            || !double.IsFinite(pad)) return false;
        var paths = input.Requested.Concat(input.Shell).Concat(input.Covered).Concat(input.Core)
            .Concat(input.Parents.SelectMany(parent => new[] { parent.Outer }.Concat(parent.Holes)))
            .Concat(input.Clip is null ? Array.Empty<IReadOnlyList<PointD>>() : new[] { input.Clip });
        return paths.All(path => path.Count >= 2 && path.All(point => double.IsFinite(point.x)
            && double.IsFinite(point.y)
            && (Math.Abs(point.x) + pad) * PathUtils.Scale <= MaximumIntegerCoordinate
            && (Math.Abs(point.y) + pad) * PathUtils.Scale <= MaximumIntegerCoordinate));
    }
}
