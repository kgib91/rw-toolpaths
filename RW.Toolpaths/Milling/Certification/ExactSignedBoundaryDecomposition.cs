using Clipper2Lib;
using System.Diagnostics;

namespace RW.Toolpaths.Milling.Certification;

/// <summary>An immutable oriented triangle in the original integer lattice.</summary>
internal readonly record struct SignedTriangle(Point64 A, Point64 B, Point64 C);

/// <summary>
/// An untrusted triangulator chooses a decomposition; exact directed edge-chain
/// equality establishes its winding identity. All signed correction triangles
/// remain in the result, including negative and exterior slivers. No triangle
/// gains machining or exterior authority from its orientation or provenance.
/// </summary>
internal static class ExactSignedBoundaryDecomposition
{
    internal const int MaximumInputPoints = 500_000;
    internal const int MaximumTriangles = 1_000_000;
    internal const long CoordinateLimit = 1L << 30;

    internal sealed class Diagnostics
    {
        internal bool Created, UntrustedIntersectionRepairSkipped;
        internal string Reason = "not-started";
        internal int InputPoints, MainTriangles, CorrectionTriangles, ResidualEdges, FinalResidualEdges;
        internal double TriangulationMilliseconds, ChainMilliseconds, TotalMilliseconds;
    }

    private readonly record struct EdgeKey(Point64 A, Point64 B);
    private readonly record struct LineKey(long Dx, long Dy, Int128 Cross);

    internal static bool TryCreate(Paths64 canonicalBoundary, CancellationToken token, Action? checkpoint,
        out SignedTriangle[] triangles, out Diagnostics diagnostics, bool useDelaunay = false,
        bool repairIntersections = false, Paths64? triangulationAidBoundary = null)
    {
        ArgumentNullException.ThrowIfNull(canonicalBoundary);
        token.ThrowIfCancellationRequested(); checkpoint?.Invoke();
        return TryCreateAfterInitialCheckpoint(canonicalBoundary, token, checkpoint, out triangles,
            out diagnostics, useDelaunay, repairIntersections, triangulationAidBoundary);
    }

    // The established capsule caller has already paid its entry checkpoint and
    // input guard. Keep that checkpoint placement while sharing every actual
    // triangulation and chain operation with new consumers.
    internal static bool TryCreateAfterInitialCheckpoint(Paths64 canonicalBoundary, CancellationToken token,
        Action? checkpoint, out SignedTriangle[] triangles, out Diagnostics diagnostics,
        bool useDelaunay = false, bool repairIntersections = false, Paths64? triangulationAidBoundary = null)
    {
        triangles = Array.Empty<SignedTriangle>();
        var result = new Diagnostics
        {
            UntrustedIntersectionRepairSkipped = !repairIntersections,
        };
        diagnostics = result;
        var total = Stopwatch.StartNew();
        var timer = new Stopwatch();
        int phase = 0;
        try
        {
            if (!BoundedBoundary(canonicalBoundary, token, out int points))
            { result.InputPoints = points; result.Reason = "input-or-integer-bound"; return false; }
            result.InputPoints = points;
            if (triangulationAidBoundary is not null
                && !BoundedBoundary(triangulationAidBoundary, token, out _))
            { result.Reason = "triangulation-aid-input-or-integer-bound"; return false; }
            timer.Start(); phase = 1;
            result.Reason = "triangulation-in-progress";
            var aid = new Delaunay(useDelaunay, token, 20_000_000, checkpoint, repairIntersections);
            var status = aid.Execute(triangulationAidBoundary ?? canonicalBoundary, out var proposed);
            result.TriangulationMilliseconds = timer.Elapsed.TotalMilliseconds;
            if (status != TriangulateResult.success)
            { result.Reason = $"untrusted-triangulator-{status}"; return false; }
            if (!IsBoundedProposedTriangulation(proposed))
            { result.Reason = "untrusted-triangle-shape-count-or-integer-bound"; return false; }
            checkpoint?.Invoke();
            var completed = proposed.Select(path => new SignedTriangle(path[0], path[1], path[2])).ToList();
            result.MainTriangles = completed.Count;
            timer.Restart(); phase = 2;
            result.Reason = "signed-chain-in-progress";
            var residual = Residual(canonicalBoundary, completed, token, checkpoint);
            result.ResidualEdges = residual.Count;
            bool corrected = AppendSignedCorrections(residual, completed, token, checkpoint,
                out int corrections, out bool triangleBudgetExceeded);
            result.CorrectionTriangles = corrections;
            if (!corrected)
            {
                result.Reason = triangleBudgetExceeded ? "signed-triangle-count-budget" : "residual-chain-budget-or-balance";
                return false;
            }
            result.FinalResidualEdges = Residual(canonicalBoundary, completed, token, checkpoint).Count;
            if (result.FinalResidualEdges != 0)
            { result.Reason = "exact-boundary-chain-mismatch"; return false; }
            result.ChainMilliseconds = timer.Elapsed.TotalMilliseconds;
            token.ThrowIfCancellationRequested();
            // The caller receives a fresh ordered snapshot. No mutable list or
            // triangulator storage is shared with another consumer.
            triangles = completed.ToArray();
            result.Created = true;
            result.Reason = "exact-signed-boundary-chain";
            return true;
        }
        catch (TriangulationBudgetException)
        { result.Reason = "untrusted-triangulator-work-budget"; return false; }
        catch (InsufficientExecutionStackException)
        { result.Reason = "untrusted-triangulator-stack-budget"; return false; }
        catch (OverflowException)
        { result.Reason = "bounded-chain-arithmetic-overflow"; return false; }
        catch (InvalidOperationException)
        { result.Reason = "untrusted-decomposition-or-chain-invariant"; return false; }
        finally
        {
            if (phase == 1) result.TriangulationMilliseconds = timer.Elapsed.TotalMilliseconds;
            else if (phase == 2) result.ChainMilliseconds = timer.Elapsed.TotalMilliseconds;
            result.TotalMilliseconds = total.Elapsed.TotalMilliseconds;
        }
    }

    private static bool BoundedBoundary(Paths64 boundary, CancellationToken token, out int points)
    {
        long count = boundary.Sum(path => (long)path.Count);
        points = (int)Math.Min(int.MaxValue, count);
        if (points == 0 || points > MaximumInputPoints) return false;
        int visited = 0;
        foreach (var path in boundary)
        foreach (var point in path)
        {
            if ((++visited & 1023) == 0) token.ThrowIfCancellationRequested();
            if (!Bounded(point)) return false;
        }
        return true;
    }

    internal static bool IsBoundedProposedTriangulation(Paths64 proposed)
        => proposed.Count <= MaximumTriangles && proposed.All(path => path.Count == 3 && path.All(Bounded));

    private static bool Bounded(Point64 point)
        => point.X > -CoordinateLimit && point.X < CoordinateLimit
            && point.Y > -CoordinateLimit && point.Y < CoordinateLimit;

    internal static List<(Point64 Start, Point64 End)> Residual(Paths64 boundary,
        IReadOnlyList<SignedTriangle> triangles, CancellationToken token, Action? checkpoint)
    {
        var edges = new Dictionary<EdgeKey, int>();
        int work = 0;
        void Check()
        {
            if ((++work & 1023) == 0) { token.ThrowIfCancellationRequested(); checkpoint?.Invoke(); }
        }
        void Edge(Point64 a, Point64 b, int factor)
        {
            Check();
            if (a == b) return;
            bool forward = a.X < b.X || (a.X == b.X && a.Y < b.Y);
            var key = forward ? new EdgeKey(a, b) : new EdgeKey(b, a);
            int value = edges.GetValueOrDefault(key) + (forward ? factor : -factor);
            if (value == 0) edges.Remove(key); else edges[key] = value;
        }
        foreach (var path in boundary)
            for (int i = 0; i < path.Count; i++) Edge(path[i], path[(i + 1) % path.Count], 1);
        foreach (var triangle in triangles)
        { Edge(triangle.A, triangle.B, -1); Edge(triangle.B, triangle.C, -1); Edge(triangle.C, triangle.A, -1); }
        var lines = new Dictionary<LineKey, SortedDictionary<Int128, int>>();
        foreach (var entry in edges)
        {
            Check();
            Point64 a = entry.Key.A, b = entry.Key.B;
            long dx = b.X - a.X, dy = b.Y - a.Y;
            long divisor = Gcd(Math.Abs(dx), Math.Abs(dy)); dx /= divisor; dy /= divisor;
            var line = new LineKey(dx, dy, (Int128)dx * a.Y - (Int128)dy * a.X);
            if (!lines.TryGetValue(line, out var events)) lines.Add(line, events = new());
            Int128 first = (Int128)dx * a.X + (Int128)dy * a.Y;
            Int128 last = (Int128)dx * b.X + (Int128)dy * b.Y;
            events[first] = events.GetValueOrDefault(first) + entry.Value;
            events[last] = events.GetValueOrDefault(last) - entry.Value;
        }
        var output = new List<(Point64, Point64)>();
        foreach (var line in lines)
        {
            token.ThrowIfCancellationRequested(); checkpoint?.Invoke();
            Int128 previous = 0; int count = 0;
            foreach (var entry in line.Value)
            {
                Check();
                if (count != 0)
                {
                    if (Math.Abs(count) > 16 || output.Count > MaximumInputPoints) throw new TriangulationBudgetException();
                    Point64 a = OnLine(line.Key, previous), b = OnLine(line.Key, entry.Key);
                    for (int repetition = 0; repetition < Math.Abs(count); repetition++)
                        output.Add(count > 0 ? (a, b) : (b, a));
                }
                count += entry.Value; previous = entry.Key;
            }
            if (count != 0) throw new InvalidOperationException("A finite exact chain cannot have an unbalanced line interval.");
        }
        return output;
    }

    private static Point64 OnLine(LineKey line, Int128 parameter)
    {
        Int128 length = (Int128)line.Dx * line.Dx + (Int128)line.Dy * line.Dy;
        Int128 x = line.Dx * parameter - line.Dy * line.Cross;
        Int128 y = line.Dy * parameter + line.Dx * line.Cross;
        if (x % length != 0 || y % length != 0) throw new InvalidOperationException("Original integer chain interval lost its lattice endpoint.");
        return new((long)(x / length), (long)(y / length));
    }

    internal static bool AppendSignedCorrections(IReadOnlyList<(Point64 Start, Point64 End)> residual,
        List<SignedTriangle> triangles, CancellationToken token, Action? checkpoint,
        out int corrections, out bool triangleBudgetExceeded, int maximumTriangles = MaximumTriangles)
    {
        if (maximumTriangles < 0 || maximumTriangles > MaximumTriangles) throw new ArgumentOutOfRangeException(nameof(maximumTriangles));
        corrections = 0; triangleBudgetExceeded = triangles.Count > maximumTriangles;
        if (triangleBudgetExceeded || residual.Count > 50_000) return false;
        var outgoing = new Dictionary<Point64, List<Point64>>();
        var balance = new Dictionary<Point64, int>();
        int work = 0;
        void Check()
        {
            if ((++work & 1023) == 0) { token.ThrowIfCancellationRequested(); checkpoint?.Invoke(); }
        }
        foreach (var edge in residual)
        {
            Check();
            if (!outgoing.TryGetValue(edge.Start, out var next)) outgoing.Add(edge.Start, next = new());
            next.Add(edge.End);
            balance[edge.Start] = balance.GetValueOrDefault(edge.Start) + 1;
            balance[edge.End] = balance.GetValueOrDefault(edge.End) - 1;
        }
        if (balance.Values.Any(value => value != 0)) return false;
        while (outgoing.Count > 0)
        {
            token.ThrowIfCancellationRequested(); checkpoint?.Invoke();
            Point64 start = outgoing.First().Key, current = start;
            var loop = new List<Point64>();
            do
            {
                Check();
                loop.Add(current);
                if (!outgoing.TryGetValue(current, out var next) || next.Count == 0) return false;
                Point64 end = next[^1]; next.RemoveAt(next.Count - 1);
                if (next.Count == 0) outgoing.Remove(current);
                current = end;
                if (loop.Count > residual.Count) return false;
            } while (current != start);
            for (int i = 1; i + 1 < loop.Count; i++)
            {
                Check();
                if (InternalClipper.CrossProductSign(loop[0], loop[i], loop[i + 1]) == 0) continue;
                // Main and signed correction triangles share ONE final count
                // bound. Refusal publishes no partial decomposition.
                if (triangles.Count >= maximumTriangles) { triangleBudgetExceeded = true; return false; }
                triangles.Add(new(loop[0], loop[i], loop[i + 1])); corrections++;
            }
        }
        return true;
    }

    private static long Gcd(long first, long second)
    {
        while (second != 0) (first, second) = (second, first % second);
        return first;
    }
}
