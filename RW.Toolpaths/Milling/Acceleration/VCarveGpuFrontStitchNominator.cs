using Clipper2Lib;
using System.Collections.Immutable;
using System.Diagnostics;

namespace RW.Toolpaths.Milling.Acceleration;

/// <summary>Source-only graph nomination. Preserves the literal input edge
/// multiset; it does not prove source fill, holes, wall clearance or stock.</summary>
internal static class VCarveGpuFrontStitchNominator
{
    internal const int MaximumFragments = 1_000_000;
    private const long CoordinateDomain = 1L << 29;
    // The source/pass/layer identity comes from the original schedule. Bits are
    // retained literally: even near-equal depth or offset levels cannot stitch.
    internal readonly record struct Owner(int SourceRegion, int SourceComponent,
        int OriginalPassIndex, long LayerZBits, int FrontLevel, long OffsetBits);
    internal readonly record struct Fragment(Owner Owner, Point64 A, Point64 B);
    internal sealed record Trace(Owner Owner, ImmutableArray<Point64> Points,
        ImmutableArray<int> InputOrdinals, bool IsClosed)
    {
        internal bool HoleOwnershipKnown => false;
        internal bool FilledTopologyCertified => false;
    }
    internal sealed record Result(bool Completed, string Reason,
        ImmutableArray<Trace> Traces, int InputCount, int OutputEdges,
        int BranchVertices, int OpenEnds, double ElapsedMilliseconds)
    {
        internal bool CuttingAuthority => false;
        internal bool SourceReachabilityCertified => false;
        internal bool FinishedStockAcceptance => false;
    }
    private readonly record struct Vertex(long X, long Y)
    {
        internal Vertex(Point64 p) : this(p.X, p.Y) { }
        internal Point64 Point => new(X, Y);
    }
    private sealed class Group
    {
        internal readonly Owner Owner;
        internal readonly List<int> Ordinals = new();
        internal readonly Dictionary<Vertex, List<int>> Incidence = new();
        internal Group(Owner owner) => Owner = owner;
        internal void Add(Vertex vertex, int ordinal)
        {
            if (!Incidence.TryGetValue(vertex, out var edges))
                Incidence.Add(vertex, edges = new());
            edges.Add(ordinal);
        }
    }

    internal static Result Stitch(IReadOnlyList<Fragment> input,
        CancellationToken token, TimeSpan maximumElapsed,
        Action<string>? diagnostic = null)
    {
        token.ThrowIfCancellationRequested();
        long started = Stopwatch.GetTimestamp();
        int count = input?.Count ?? 0, outputEdges = 0, branches = 0, ends = 0;
        var traces = new List<Trace>();
        Result Finish(bool complete, string reason)
        {
            token.ThrowIfCancellationRequested();
            var result = new Result(complete, reason,
                complete ? traces.ToImmutableArray() : ImmutableArray<Trace>.Empty,
                count, complete ? outputEdges : 0, branches, ends,
                Stopwatch.GetElapsedTime(started).TotalMilliseconds);
            // Observers are diagnostic only. Caller cancellation is checked even
            // if a callback swallows/throws an unrelated diagnostic exception.
            try
            {
                diagnostic?.Invoke($"stage=initial-front-stitch-probe complete={complete} "
                    + $"reason='{reason}' fragments={count} traces={result.Traces.Length} "
                    + $"outputEdges={result.OutputEdges} branchVertices={branches} openEnds={ends} "
                    + $"elapsedMs={result.ElapsedMilliseconds:F3} cuttingAuthority=False "
                    + "filledTopologyCertified=False finishedStockAcceptance=False bridges=0");
            }
            catch (Exception) { }
            token.ThrowIfCancellationRequested();
            return result;
        }
        if (input is null || count > MaximumFragments || maximumElapsed <= TimeSpan.Zero
            || maximumElapsed > TimeSpan.FromSeconds(30))
            return Finish(false, "stitch-input-or-time-budget");

        var edges = new Fragment[count];
        var groupsByOwner = new Dictionary<Owner, Group>();
        var groups = new List<Group>();
        for (int i = 0; i < count; i++)
        {
            if ((i & 255) == 0)
            {
                token.ThrowIfCancellationRequested();
                if (Stopwatch.GetElapsedTime(started) > maximumElapsed)
                    return Finish(false, "stitch-time-budget");
            }
            var edge = input[i];
            if (edge.A.X <= -CoordinateDomain || edge.A.X >= CoordinateDomain
                || edge.A.Y <= -CoordinateDomain || edge.A.Y >= CoordinateDomain
                || edge.B.X <= -CoordinateDomain || edge.B.X >= CoordinateDomain
                || edge.B.Y <= -CoordinateDomain || edge.B.Y >= CoordinateDomain
                || edge.A.X == edge.B.X && edge.A.Y == edge.B.Y
                || !double.IsFinite(BitConverter.Int64BitsToDouble(edge.Owner.LayerZBits))
                || !double.IsFinite(BitConverter.Int64BitsToDouble(edge.Owner.OffsetBits))
                || edge.Owner.FrontLevel < 0 || edge.Owner.OriginalPassIndex < 0)
                return Finish(false, "stitch-coordinate-or-owner-domain");
            edges[i] = edge;
            if (!groupsByOwner.TryGetValue(edge.Owner, out var group))
            {
                groupsByOwner.Add(edge.Owner, group = new(edge.Owner));
                groups.Add(group);
            }
            group.Ordinals.Add(i);
            group.Add(new(edge.A), i); group.Add(new(edge.B), i);
        }
        var used = new bool[count];
        int graphWork = 0;
        bool WithinBudget()
        {
            if ((++graphWork & 255) != 0) return true;
            token.ThrowIfCancellationRequested();
            return Stopwatch.GetElapsedTime(started) <= maximumElapsed;
        }
        bool Walk(Group group, int first, Vertex start)
        {
            var points = ImmutableArray.CreateBuilder<Point64>();
            var ordinals = ImmutableArray.CreateBuilder<int>();
            points.Add(start.Point);
            int next = first; var cursor = start; bool closed = false;
            while (true)
            {
                if ((outputEdges & 255) == 0)
                {
                    token.ThrowIfCancellationRequested();
                    if (Stopwatch.GetElapsedTime(started) > maximumElapsed) return false;
                }
                if (used[next]) return false;
                var edge = edges[next]; var a = new Vertex(edge.A); var b = new Vertex(edge.B);
                if (cursor != a && cursor != b) return false;
                var destination = cursor == a ? b : a;
                // Every appended chord is EXACTLY this input finite edge. A
                // cycle closes only by appending an actual edge back to start.
                used[next] = true; outputEdges++; ordinals.Add(next);
                points.Add(destination.Point); cursor = destination;
                if (cursor == start) { closed = true; break; }
                var incident = group.Incidence[cursor];
                if (incident.Count != 2) break;
                int continuation = incident[0] == next ? incident[1] : incident[0];
                if (used[continuation]) return false;
                next = continuation;
            }
            traces.Add(new(group.Owner, points.ToImmutable(), ordinals.ToImmutable(), closed));
            return true;
        }
        foreach (var group in groups)
        {
            foreach (var incident in group.Incidence.Values)
            {
                if (!WithinBudget()) return Finish(false, "stitch-time-budget");
                if (incident.Count > 2) branches++;
                if (incident.Count == 1) ends++;
            }
            // Start boundary/ambiguous arms first, then remaining degree-two
            // cycles. Input ordinal gives stable deterministic ordering.
            foreach (int ordinal in group.Ordinals)
            {
                if (!WithinBudget()) return Finish(false, "stitch-time-budget");
                if (used[ordinal]) continue;
                var edge = edges[ordinal]; var a = new Vertex(edge.A); var b = new Vertex(edge.B);
                bool aStop = group.Incidence[a].Count != 2;
                bool bStop = group.Incidence[b].Count != 2;
                if ((aStop || bStop) && !Walk(group, ordinal, aStop ? a : b))
                    return Finish(false, "stitch-time-or-incidence-refusal");
            }
            foreach (int ordinal in group.Ordinals)
            {
                if (!WithinBudget()) return Finish(false, "stitch-time-budget");
                if (!used[ordinal] && !Walk(group, ordinal, new(edges[ordinal].A)))
                    return Finish(false, "stitch-time-or-incidence-refusal");
            }
        }
        if (outputEdges != count) return Finish(false, "stitch-edge-multiset-refusal");
        if (Stopwatch.GetElapsedTime(started) > maximumElapsed) return Finish(false, "stitch-time-budget");
        return Finish(true, "literal-input-edge-multiset-retained");
    }
}
