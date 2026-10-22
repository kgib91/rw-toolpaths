using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling.Certification;
using System.Collections.Immutable;
using System.Diagnostics;

namespace RW.Toolpaths.Milling.Acceleration;

// Reduces UNTRUSTED nomination geometry only. Source and reduced paths still
// require the same actual-command floor/wall/grade/target acceptance. The strict
// exact finite-capsule check owns only a geometric chain-displacement bound.
internal static class VCarveGpuFrontTraceReductionNominator
{
    private const int MaximumChunkChords = 512;
    private const int MaximumPoints = 1_000_001;
    private const long CoordinateLimit = 1L << 29;
    internal sealed record Result(bool Completed, string Reason, ImmutableArray<Point64> Points,
        int InputPoints, int OutputChords, int ReplacementChords, int RefusedReplacements,
        long VerifiedVertices, double ElapsedMilliseconds)
    {
        internal bool CuttingAuthority => false;
        internal bool StockDisplacementCertified => false;
    }

    internal static Result Reduce(IReadOnlyList<Point64> input, bool isClosed, int maximumGridMove,
        CancellationToken token, TimeSpan maximumElapsed)
    {
        token.ThrowIfCancellationRequested(); long started = Stopwatch.GetTimestamp();
        int count = input?.Count ?? 0, replacements = 0, refused = 0;
        long verified = 0; var output = new List<Point64>();
        Result Finish(bool complete, string reason)
        {
            token.ThrowIfCancellationRequested();
            var snapshot = complete ? output.ToImmutableArray() : ImmutableArray<Point64>.Empty;
            token.ThrowIfCancellationRequested(); double elapsed = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            if (complete && elapsed > maximumElapsed.TotalMilliseconds) { complete = false; reason = "reduction-time-budget"; }
            return new(complete, reason, complete ? snapshot : ImmutableArray<Point64>.Empty,
                count, complete ? output.Count - 1 : 0, replacements, refused, verified, elapsed);
        }
        if (input is null || count is < 2 or > MaximumPoints || maximumGridMove is < 1 or > 32
            || maximumElapsed <= TimeSpan.Zero || maximumElapsed > TimeSpan.FromSeconds(5))
            return Finish(false, "reduction-input-or-budget");
        bool WithinBudget()
        {
            token.ThrowIfCancellationRequested();
            return Stopwatch.GetElapsedTime(started) <= maximumElapsed;
        }
        for (int i = 0; i < count; i++)
        {
            if ((i & 255) == 0 && !WithinBudget()) return Finish(false, "reduction-time-budget");
            var p = input[i];
            if (p.X <= -CoordinateLimit || p.X >= CoordinateLimit || p.Y <= -CoordinateLimit || p.Y >= CoordinateLimit)
                return Finish(false, "reduction-coordinate-domain");
        }
        if (isClosed && (input[0] != input[count - 1] || count < 4))
            return Finish(false, "reduction-closed-owner-mismatch");
        output.Add(input[0]);
        for (int start = 0; start < count - 1; start += MaximumChunkChords)
        {
            if (!WithinBudget()) return Finish(false, "reduction-time-budget");
            int end = Math.Min(count - 1, start + MaximumChunkChords);
            var chunk = new PointD[end - start + 1];
            for (int i = start; i <= end; i++) chunk[i - start] = new(input[i].X / PathUtils.Scale, input[i].Y / PathUtils.Scale);
            // Reuse the owned bounded-to-FINITE-segment RDP nominator. A chunk
            // bounds its uncancellable quadratic work; its floating decisions
            // are not accepted until the following exact source-vertex check.
            var reduced = PolygonSimplify.RdpSegmentBounded(chunk, maximumGridMove / PathUtils.Scale);
            if (!WithinBudget()) return Finish(false, "reduction-time-budget");
            int previous = start;
            for (int r = 1; r < reduced.Count; r++)
            {
                if (!WithinBudget()) return Finish(false, "reduction-time-budget");
                var nomination = reduced[r]; int next = previous + 1;
                while (next <= end && (input[next].X / PathUtils.Scale != nomination.x || input[next].Y / PathUtils.Scale != nomination.y)) next++;
                if (next > end) return Finish(false, "reduction-non-source-vertex-nomination");
                bool supported = true;
                for (int i = previous; i <= next; i += 3)
                {
                    if ((verified & 255) < 3 && !WithinBudget()) return Finish(false, "reduction-time-budget");
                    var a = input[i]; var b = input[Math.Min(i + 1, next)]; var c = input[Math.Min(i + 2, next)];
                    supported &= SignedBoundaryCapsuleCertificate.ContainsDyadicTriangle(input[previous], input[next],
                        a.X, a.Y, b.X, b.Y, c.X, c.Y, 0, maximumGridMove, radiusFractionBits: 0);
                    verified += Math.Min(3, next - i + 1);
                    if (!supported) break;
                }
                if (supported)
                {
                    output.Add(input[next]); if (next - previous > 1) replacements++;
                }
                else
                {
                    refused++; for (int i = previous + 1; i <= next; i++) output.Add(input[i]);
                }
                previous = next;
            }
            if (previous != end) return Finish(false, "reduction-incomplete-chunk");
        }
        if (output.Count < 2 || output[0] != input[0] || output[output.Count - 1] != input[count - 1])
            return Finish(false, "reduction-endpoint-mismatch");
        if (isClosed && (output.Count < 4 || output.Take(output.Count - 1).Distinct().Take(3).Count() < 3))
        {
            // Never turn a closed source trace into an empty/two-vertex loop.
            output = input.ToList(); replacements = 0; refused++;
        }
        return Finish(true, "exact-bounded-nomination-chain-reduction");
    }
}
