using System.Numerics;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.Acceleration;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class VCarveGpuBoundaryFp32SampleTests(ITestOutputHelper output)
{
    private static GpuBoundaryWorkLimits Limits => GpuBoundaryWorkLimits.Default;

    [Fact]
    public void NormalFactoryRequiresItsSeparateExplicitOptIn()
    {
        const string flag = "RW_TOOLPATHS_VCARVE_GPU_BOUNDARY_FP32";
        string? previous = Environment.GetEnvironmentVariable(flag);
        try
        {
            Environment.SetEnvironmentVariable(flag, null);
            GpuBoundarySegment[] segments = [new(0, 0, 100, 0)];
            if (!VCarveGpuDiscWitnessContext.BoundarySampleContext.TryCreate(segments, Build(segments), default,
                output.WriteLine, out var context, out string reason)) { Portable(reason); return; }
            using (context)
            {
                Assert.False(context!.RequestedFp32Nomination);
                GpuBoundaryQuery[] queries = [new(50, 20)]; var values = new GpuBoundarySampleNomination[1];
                Assert.True(context.TryNominate(queries, values, 1, default, out reason), reason);
                Assert.Equal(400, values[0].SquaredDistance); Assert.Equal(0, context.Fp32Queries);
                Assert.Equal("fp64-nearest-int64-winding", context.NumericMode);
            }
            output.WriteLine("[VCarve.BoundaryFp32.Test] actualCuda=True defaultMode=fp64 fp32OptInRequired=True cuttingAuthority=False");
        }
        finally { Environment.SetEnvironmentVariable(flag, previous); }
    }

    [Fact]
    public void RelativeCoordinatesPreserveFarTranslationsAndExactHoleWinding()
    {
        long x = 1L << 28, y = -(1L << 28);
        var segments = Ring(x, y, x + 100_000, y + 100_000)
            .Concat(Ring(x + 20_000, y + 20_000, x + 80_000, y + 80_000).Select(s => new GpuBoundarySegment(s.Bx, s.By, s.Ax, s.Ay)))
            .Concat(Ring(x + 40_000, y + 40_000, x + 60_000, y + 60_000))
            .Concat(new[] { new GpuBoundarySegment(x + 3, y + 7, x + 3, y + 7) }).ToArray();
        if (!Create(segments, true, out var context)) return;
        using (context)
        {
            GpuBoundaryQuery[] queries = [new(x + 10_000, y + 10_000), new(x + 30_000, y + 30_000),
                new(x + 50_000, y + 50_000), new(x + 100_010, y + 100_020), new(x + 3, y + 7), new(x, y)];
            var values = new GpuBoundarySampleNomination[queries.Length];
            Assert.True(context!.TryNominate(queries, values, values.Length, default, out string reason), reason);
            for (int i = 0; i < queries.Length; i++) Compare(segments, queries[i], values[i], 16);
            Assert.Equal(1, values[0].Winding); Assert.Equal(0, values[1].Winding); Assert.Equal(1, values[2].Winding);
            Assert.Equal(queries.Length, context.Fp32Queries); Assert.Equal(0, context.Fp64FallbackQueries);
            output.WriteLine($"[VCarve.BoundaryFp32.Test] actualCuda=True actualFp32=True exactInt64HoleWinding=True farAbsoluteTranslation={x} fp32Queries={context.Fp32Queries} cuttingAuthority=False");
        }
    }

    [Fact]
    public void DenseLongOffsetEdgesExposeOptimisticFloatNomineesRejectedByOriginalCpuProfilePredicate()
    {
        long x = 400L * 32768, y = -200L * 32768, dx = (1L << 24) - 8, dy = (1L << 24) - 123;
        var segments = new List<GpuBoundarySegment> { new(x, y, x + dx, y + dy) };
        for (int i = 0; i < 256; i++)
            segments.Add(new(x + dx - 1024 + i, y + dy - 512, x + dx - 1023 + i, y + dy - 510));
        if (!Create(segments.ToArray(), true, out var context)) return;
        using (context)
        {
            var queries = Enumerable.Range(1, 384).Select(i => new GpuBoundaryQuery(x + dx * i / 389, y + dy * i / 389 + i % 3)).ToArray();
            var values = new GpuBoundarySampleNomination[queries.Length];
            Assert.True(context!.TryNominate(queries, values, values.Length, default, out string reason), reason);
            bool optimisticRejected = false; double maximumError = 0;
            var snapshot = segments.ToArray();
            for (int i = 0; i < queries.Length; i++)
            {
                maximumError = Math.Max(maximumError, Compare(snapshot, queries[i], values[i], 16));
                var n = values[i]; var s = snapshot[n.SegmentOrdinal];
                // Original double world inputs and the nominated parameter are
                // evaluated again. A float distance cannot accept this pose.
                double qx = queries[i].X / PathUtils.Scale, qy = queries[i].Y / PathUtils.Scale;
                double ax = s.Ax / PathUtils.Scale, ay = s.Ay / PathUtils.Scale;
                double px = ax + (s.Bx / PathUtils.Scale - ax) * n.Parameter;
                double py = ay + (s.By / PathUtils.Scale - ay) * n.Parameter;
                double literalDistance = Math.Sqrt((qx - px) * (qx - px) + (qy - py) * (qy - py));
                double deviceDistance = Math.Sqrt(n.SquaredDistance) / PathUtils.Scale;
                if (literalDistance - deviceDistance <= 2e-8) continue;
                double proposedRadius = (literalDistance + deviceDistance) / 2;
                var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, proposedRadius, proposedRadius, .5)], .5);
                Assert.True(deviceDistance <= profile.PhysicalRadiusAtDepth(.25));
                Assert.False(literalDistance <= profile.PhysicalRadiusAtDepth(.25));
                optimisticRejected = true;
            }
            Assert.True(optimisticRejected, "Expected an actual optimistic FP32 distance rejected by the literal original CPU physical-radius predicate.");
            Assert.Equal(queries.Length, context.Fp32Queries);
            output.WriteLine($"[VCarve.BoundaryFp32.Test] actualCuda=True actualFp32=True denseLongOffsets=True rationalOracle=True maximumDistanceErrorTicks={maximumError:R} actualOptimisticFp32Nominee=True originalCpuPhysicalPredicateRejected=True cuttingAuthority=False");
        }
    }

    [Fact]
    public void UnsupportedRelativeQueryBatchUsesOriginalFp64AfterAnEarlierFp32Batch()
    {
        GpuBoundarySegment[] segments = [new(0, 0, 100, 0)];
        if (!Create(segments, true, out var context)) return;
        using (context)
        {
            var queries = Enumerable.Repeat(new GpuBoundaryQuery(50, 20), 4097).ToArray();
            queries[^1] = new(1L << 25, 20); var values = new GpuBoundarySampleNomination[queries.Length];
            Assert.False(context!.QuerySupportsFp32(queries[^1]));
            Assert.False(context.QuerySupportsFp32(new(long.MaxValue, long.MinValue)));
            Assert.True(context.TryNominate(queries, values, values.Length, default, out string reason), reason);
            Assert.Equal(4096, context.Fp32Queries); Assert.Equal(1, context.Fp64FallbackQueries); Assert.Equal(2, context.Batches);
            Compare(segments, queries[0], values[0], 0); Compare(segments, queries[^1], values[^1], 1e-6);
            output.WriteLine("[VCarve.BoundaryFp32.Test] actualCuda=True fp32Queries=4096 fp64FallbackQueries=1 doubleAbi=True unsupportedQueryUsesOriginalFp64=True cuttingAuthority=False");
        }
    }

    [Fact]
    public void UnsupportedSourceExtentKeepsOriginalFp64AndBoundedWorkRefusalsClearAllFields()
    {
        GpuBoundarySegment[] wide = [new(0, 0, 1L << 26, 0)];
        if (!Create(wide, true, out var context)) return;
        using (context)
        {
            Assert.False(context!.SupportsFp32SourceDomain);
            GpuBoundaryQuery[] queries = [new(1000, 20)]; var values = new GpuBoundarySampleNomination[1];
            Assert.True(context.TryNominate(queries, values, 1, default, out string reason), reason);
            Compare(wide, queries[0], values[0], 1e-6);
            Assert.Equal(0, context.Fp32Queries); Assert.Equal(1, context.Fp64FallbackQueries);
        }
        GpuBoundarySegment[] close = [new(0, 0, 100, 0), new(0, 10, 100, 10)];
        if (!Create(close, true, out var bounded, new(1, 1))) return;
        using (bounded)
        {
            var values = new GpuBoundarySampleNomination[1];
            Assert.True(bounded!.TryNominate([new(50, 20)], values, 1, default, out _));
            Assert.False(values[0].HasNomination); Assert.Equal(-1, values[0].SegmentOrdinal); Assert.Equal(0, values[0].Winding);
            Assert.True(double.IsNaN(values[0].Parameter)); Assert.True(double.IsPositiveInfinity(values[0].SquaredDistance));
            Assert.Equal(1, bounded.SegmentBudgetRefusals);
        }
        output.WriteLine("[VCarve.BoundaryFp32.Test] actualCuda=True unsupportedSourceUsesOriginalFp64=True boundedRefusalClearsAllFields=True cuttingAuthority=False");
    }

    [Fact]
    public void CallerCancellationAfterCompletedFp32BatchesClearsEveryNominee()
    {
        GpuBoundarySegment[] segments = [new(0, 0, 100, 0)]; using var cancellation = new CancellationTokenSource();
        void Diagnostic(string message) { output.WriteLine(message); if (message.Contains("event=sampled")) cancellation.Cancel(); }
        if (!VCarveGpuDiscWitnessContext.BoundarySampleContext.TryCreate(segments, Build(segments), Limits, true, cancellation.Token,
            Diagnostic, out var context, out string reason)) { Portable(reason); return; }
        using (context)
        {
            var queries = Enumerable.Repeat(new GpuBoundaryQuery(50, 20), 4097).ToArray(); var values = new GpuBoundarySampleNomination[queries.Length];
            Assert.Throws<OperationCanceledException>(() => context!.TryNominate(queries, values, values.Length, cancellation.Token, out _));
            Assert.All(values, value => { Assert.False(value.HasNomination); Assert.Equal(-1, value.SegmentOrdinal); Assert.Equal(0, value.Winding); });
            Assert.Equal(4097, context!.Fp32Queries);
        }
        output.WriteLine("[VCarve.BoundaryFp32.Test] actualCuda=True fp32Queries=4097 cancellationAfterBatchesClearsAll=True cuttingAuthority=False");
    }

    private bool Create(GpuBoundarySegment[] segments, bool mode, out VCarveGpuDiscWitnessContext.BoundarySampleContext? context, GpuBoundaryWorkLimits? limits = null)
    {
        bool ready = VCarveGpuDiscWitnessContext.BoundarySampleContext.TryCreate(segments, Build(segments), limits ?? Limits, mode,
            default, output.WriteLine, out context, out string reason); if (!ready) Portable(reason); return ready;
    }
    private void Portable(string reason) => output.WriteLine($"[VCarve.BoundaryFp32.Test] actualCuda=False portableRefusal='{reason}' cuttingAuthority=False");
    private static double Compare(GpuBoundarySegment[] segments, GpuBoundaryQuery query, GpuBoundarySampleNomination value, double maximumErrorTicks)
    {
        Assert.True(value.HasNomination); Assert.InRange(value.Parameter, 0, 1); Assert.InRange(value.SegmentOrdinal, 0, segments.Length - 1);
        BigInteger bestN = 0, bestD = 1; int winding = 0; bool any = false;
        foreach (var s in segments)
        {
            BigInteger dx = (BigInteger)s.Bx - s.Ax, dy = (BigInteger)s.By - s.Ay, qx = (BigInteger)query.X - s.Ax, qy = (BigInteger)query.Y - s.Ay;
            BigInteger length = dx * dx + dy * dy, dot = qx * dx + qy * dy, cross = dx * qy - dy * qx;
            if (s.Ay <= query.Y && s.By > query.Y && cross.Sign > 0) winding++;
            else if (s.Ay > query.Y && s.By <= query.Y && cross.Sign < 0) winding--;
            BigInteger numerator, denominator;
            if (length.IsZero || dot.Sign <= 0) { numerator = qx * qx + qy * qy; denominator = 1; }
            else if (dot >= length) { qx -= dx; qy -= dy; numerator = qx * qx + qy * qy; denominator = 1; }
            else { numerator = cross * cross; denominator = length; }
            if (!any || numerator * bestD < bestN * denominator) { any = true; bestN = numerator; bestD = denominator; }
        }
        Assert.Equal(winding, value.Winding);
        double error = Math.Abs(Math.Sqrt(value.SquaredDistance) - Math.Sqrt((double)bestN / (double)bestD));
        Assert.True(error <= maximumErrorTicks, $"Nomination distance calibration exceeded {maximumErrorTicks:R} ticks: {error:R}. This calibration is no stock allowance.");
        return error;
    }
    private static IEnumerable<GpuBoundarySegment> Ring(long x0, long y0, long x1, long y1)
        => new[] { new GpuBoundarySegment(x0, y0, x1, y0), new(x1, y0, x1, y1), new(x1, y1, x0, y1), new(x0, y1, x0, y0) };
    private static GpuBoundaryNode[] Build(GpuBoundarySegment[] segments)
    {
        var nodes = new List<GpuBoundaryNode>();
        int Add(int first, int count)
        {
            int ordinal = nodes.Count; nodes.Add(default);
            long x0 = segments.Skip(first).Take(count).Min(s => Math.Min(s.Ax, s.Bx)), y0 = segments.Skip(first).Take(count).Min(s => Math.Min(s.Ay, s.By));
            long x1 = segments.Skip(first).Take(count).Max(s => Math.Max(s.Ax, s.Bx)), y1 = segments.Skip(first).Take(count).Max(s => Math.Max(s.Ay, s.By));
            if (count <= 8) nodes[ordinal] = new(x0, y0, x1, y1, first, count, -1, -1);
            else { int half = count / 2, left = Add(first, half), right = Add(first + half, count - half); nodes[ordinal] = new(x0, y0, x1, y1, 0, 0, left, right); }
            return ordinal;
        }
        Add(0, segments.Length); return nodes.ToArray();
    }
}
