using Clipper2Lib;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.Acceleration;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class VCarveGpuFp32NominationTests(ITestOutputHelper output)
{
    [Theory]
    [InlineData(0)]
    [InlineData(400)]
    [InlineData(9998)]
    public void RelativeCoordinatesPreserveCpuAuthorityNearTheOriginalReserve(double origin)
    {
        using var enabled = new Fp32Scope();
        GpuDiscSegment[] segments = [new(origin, origin, .5, origin + 1, origin, .5, 1, 19)];
        GpuDiscNode[] nodes = [new(origin, origin, origin + 1, origin, 1, 0, 1, -1, -1)];
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)], 1);
        if (!Create(segments, nodes, profile, out var context)) return;
        using (context)
        {
            Assert.True(context!.Fp32Requested); Assert.True(context.Fp32Enabled, context.Fp32RefusalReason);
            double radius = profile.PhysicalRadiusAtDepth(.5), desired = .004;
            GpuDiscQuery[] queries = [new(origin + .5, origin + .1, desired), new(origin + .5, origin + 2, desired),
                new(origin + .5, origin + radius + desired - 2e-8, desired),
                new(origin + .5, origin + radius + desired + 2e-8, desired)];
            var results = new GpuDiscNomination[queries.Length];
            Assert.True(context.TryNominate(queries, results, queries.Length, default, out string reason), reason);
            Assert.True(results[0].HasWitness); Assert.True(context.TryValidateNomination(queries[0], results[0], out int owner));
            Assert.Equal(19, owner); Assert.False(context.TryValidateNomination(queries[1], results[1], out _));
            // Float nominees can straddle the reserve. They never gain authority:
            // the original unrounded CPU predicate rejects an outside pose.
            Assert.True(context.TryValidateNomination(queries[2], new(0, .5, 1), out _));
            Assert.False(context.TryValidateNomination(queries[3], new(0, .5, 1), out _));
            Assert.False(context.TryValidateNomination(queries[3], results[3], out _));
            GpuBoxQuery[] boxes = [new(origin, origin - .8, origin + 1, origin + .8, desired - 1e-8)];
            var nominations = new GpuBoxNomination[1];
            Assert.True(context.TryNominateBoxes(boxes, nominations, 1, default, out reason), reason);
            Assert.True(nominations[0].HasWitness); Assert.True(context.TryValidateBoxNomination(boxes[0], nominations[0], out owner));
            Assert.Equal(19, owner); Assert.Equal(queries.Length + 1, context.Fp32Queries);
        }
    }

    [Fact]
    public void UnsupportedGeometryAndQueriesRetainTheFp64NominationBackend()
    {
        using var enabled = new Fp32Scope();
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 1, 1, 1)], 1);
        if (!Create([new(0, 0, .1, 1, 0, .1, 1, 0)], [new(0, 0, 1, 0, 1, 0, 1, -1, -1)], profile, out var ordinary)) return;
        using (ordinary)
        {
            Assert.True(ordinary!.Fp32Enabled, ordinary.Fp32RefusalReason);
            GpuDiscQuery[] queries = [new(.5, 0, 101)];
            var results = new GpuDiscNomination[1];
            Assert.True(ordinary.TryNominate(queries, results, 1, default, out string reason), reason);
            Assert.True(ordinary.TryValidateNomination(queries[0], results[0], out _));
            Assert.Equal(0, ordinary.Fp32Queries); Assert.Equal(1, ordinary.Fp32RefusedQueries);
            Assert.Equal(1, ordinary.Fp64FallbackBatches);
        }
        if (!Create([new(10001, 0, .1, 10002, 0, .1, 1, 0)],
            [new(10001, 0, 10002, 0, 1, 0, 1, -1, -1)], profile, out var large)) return;
        using (large)
        {
            Assert.False(large!.Fp32Enabled); Assert.Equal("fp32-geometry-domain", large.Fp32RefusalReason);
            GpuDiscQuery[] queries = [new(10001.5, 0, 0)];
            var results = new GpuDiscNomination[1];
            Assert.True(large.TryNominate(queries, results, 1, default, out string reason), reason);
            Assert.True(large.TryValidateNomination(queries[0], results[0], out _));
            Assert.Equal(1, large.Fp64FallbackBatches);
        }
    }

    [Fact]
    public void DeliberatelyOptimisticFloatDiscNomineeIsRejectedByLiteralCpuPredicate()
    {
        using var enabled = new Fp32Scope();
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, .15, .15, 1)], 1);
        if (!Create([new(0, 0, .1, 1, 0, .1, .15, 7)], [new(0, 0, 1, 0, .15, 0, 1, -1, -1)], profile, out var context)) return;
        using (context)
        {
            Assert.True(context!.Fp32Enabled, context.Fp32RefusalReason);
            // Both values round to the same float; the original radius stays
            // smaller than the original query distance in the CPU snapshot.
            double distance = .1500000025;
            Assert.Equal((float).15, (float)distance); Assert.True(distance > .15);
            GpuDiscQuery[] queries = [new(.5, distance, 0)];
            var nominations = new GpuDiscNomination[1];
            Assert.True(context.TryNominate(queries, nominations, 1, default, out string reason), reason);
            Assert.True(nominations[0].HasWitness, "The control must exercise an actually optimistic device nominee.");
            Assert.False(context.TryValidateNomination(queries[0], nominations[0], out _));
            output.WriteLine($"optimisticNominee=True cpuAccepted=False t={nominations[0].Parameter:R} " +
                $"signedDistance={distance - profile.PhysicalRadiusAtDepth(.1):R} numericMode=fp32-relative-xy-nomination");
        }
    }

    [Theory]
    [InlineData(false, true)]
    [InlineData(true, false)]
    public void RetainedHoleAndOmittedHoleKeepOriginalWholeTargetOutcomes(bool fillHole, bool expected)
    {
        var outer = Rectangle(-.3, -.3, .3, .3);
        IReadOnlyList<IReadOnlyList<PointD>> target = fillHole ? [outer] : [outer, Rectangle(-.1, -.1, .1, .1).Reverse().ToArray()];
        TaggedToolpath[] paths = [new([new(-.2, -.2, -.1), new(.2, -.2, -.1), new(.2, .2, -.1),
            new(-.2, .2, -.1), new(-.2, -.2, -.1)], Array.Empty<ToolpathSpan>(), 0, "clearing", 0, detectCurves: false)];
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, .15, .15, 1)], 1);
        var limits = new VCarveCoverageCertificate.Limits { MaximumElapsed = TimeSpan.FromSeconds(30), MinimumGpuIntervals = 1 };
        var reference = VCarveCoverageCertificate.Certify(target, paths, profile, limits: limits);
        using var enabled = new Fp32Scope();
        var actual = VCarveCoverageCertificate.Certify(target, paths, profile,
            limits: limits with { UseGpuDiscWitnesses = true }, diagnostic: output.WriteLine);
        Assert.Equal(expected, reference.Certified); Assert.Equal(reference.Certified, actual.Certified);
        if (actual.GpuAvailable) Assert.True(actual.GpuBatches > 0);
    }

    private bool Create(GpuDiscSegment[] segments, GpuDiscNode[] nodes, VCarveCutterProfile profile,
        out VCarveGpuDiscWitnessContext? context)
    {
        bool success = VCarveGpuDiscWitnessContext.TryCreate(segments, nodes, profile, default, output.WriteLine, out context, out string reason);
        if (!success) { output.WriteLine("explicit CUDA fallback: " + reason); Assert.False(string.IsNullOrWhiteSpace(reason)); }
        return success;
    }
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1) => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
    private sealed class Fp32Scope : IDisposable
    {
        private readonly string? _previous = Environment.GetEnvironmentVariable("RW_TOOLPATHS_VCARVE_GPU_FP32");
        internal Fp32Scope() => Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_GPU_FP32", "1");
        public void Dispose() => Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_GPU_FP32", _previous);
    }
}
