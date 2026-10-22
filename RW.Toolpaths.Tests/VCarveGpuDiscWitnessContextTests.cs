using System.Reflection;
using System.Runtime.InteropServices;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.Acceleration;

namespace RW.Toolpaths.Tests;

public sealed class VCarveGpuDiscWitnessContextTests
{
    [Fact]
    public void NativeSnapshotsHaveExplicitCompatibleLayouts()
    {
        Assert.Equal(64, Marshal.SizeOf<GpuDiscSegment>());
        Assert.Equal(56, Marshal.SizeOf<GpuDiscNode>());
        Assert.Equal(24, Marshal.SizeOf<GpuDiscQuery>());
        Assert.Equal(16, Marshal.SizeOf<GpuDiscNomination>());
        Assert.Equal(40, Marshal.SizeOf<GpuBoxQuery>());
        Assert.Equal(40, Marshal.SizeOf<GpuBoxNomination>());
        Assert.NotNull(typeof(VCarveGpuDiscWitnessContext).Assembly.GetManifestResourceStream(
            "RW.Toolpaths.Milling.Acceleration.disc_witness.cu"));
    }

    [Fact]
    public void InvalidGeometryRefusesBeforeCallingNativeDriver()
    {
        var segment = Segment();
        Assert.False(VCarveGpuDiscWitnessContext.TryCreate([segment], [new(0, 0, 1, 0, 1, 0, 2, -1, -1)],
            Ball(), default, null, out var context, out string reason));
        Assert.Null(context); Assert.Equal("invalid-bvh-indices", reason);
        Assert.False(VCarveGpuDiscWitnessContext.TryCreate([new(double.NaN, 0, .5, 1, 0, .5, 1, 0)],
            [Node()], Ball(), default, null, out context, out reason));
        Assert.Null(context); Assert.Equal("invalid-segment", reason);
    }

    [Fact]
    public void CancellationBeforeSetupDoesNotEnterDriver()
    {
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveGpuDiscWitnessContext.TryCreate(
            [Segment()], [Node()], Ball(), cancellation.Token, null, out _, out _));
    }

    [Fact]
    public void LiteralValidatorRejectsForgedNominationAndUnengagedTip()
    {
        // Construct only the immutable validator snapshot, avoiding a hardware
        // prerequisite for authority and malformed-native-result regression.
        var constructor = typeof(VCarveGpuDiscWitnessContext).GetConstructors(BindingFlags.Instance | BindingFlags.NonPublic).Single();
        using var context = (VCarveGpuDiscWitnessContext)constructor.Invoke(
            new object?[] { new[] { Segment(), new GpuDiscSegment(0, 0, 0, 1, 0, 0, 1, 1) }, new[] { Node() }, Ball(), null });
        var q = new GpuDiscQuery(.5, .1, 0);
        Assert.True(context.TryValidateNomination(q, new(0, .5, 1), out int owner)); Assert.Equal(0, owner);
        Assert.False(context.TryValidateNomination(q, new(-1, .5, 1), out _));
        Assert.False(context.TryValidateNomination(q, new(2, .5, 1), out _));
        Assert.False(context.TryValidateNomination(q, new(0, double.NaN, 1), out _));
        Assert.False(context.TryValidateNomination(q, new(0, 1.01, 1), out _));
        Assert.False(context.TryValidateNomination(q, new(0, .5, 0), out _));
        Assert.False(context.TryValidateNomination(q, new(1, .5, 1), out _));
        Assert.False(context.TryValidateNomination(new(.5, 2, 0), new(0, .5, 1), out _));
        Assert.False(context.TryValidateNomination(new(double.NaN, .1, 0), new(0, .5, 1), out _));
    }

    [Fact]
    public void RealCudaBatchesSupplyOnlyCpuAcceptedPhysicalPosesOrExplicitFallback()
    {
        var logs = new List<string>();
        bool created = VCarveGpuDiscWitnessContext.TryCreate([Segment()], [Node()], Ball(), default, logs.Add,
            out var context, out string reason);
        if (!created)
        {
            Assert.False(string.IsNullOrWhiteSpace(reason)); Assert.Null(context);
            // Portable machines retain the owned CPU certificate; the replay
            // harness separately requires actual GPU execution on this host.
            return;
        }
        using (context)
        {
            var queries = new GpuDiscQuery[VCarveGpuDiscWitnessContext.MaximumBatchSize + 1];
            for (int i = 0; i < queries.Length; i++) queries[i] = new(.5, i % 2 == 0 ? .1 : 2, 0);
            var results = new GpuDiscNomination[queries.Length];
            Assert.True(context!.TryNominate(queries, results, queries.Length, default, out reason), reason);
            Assert.Equal(2, context.Batches); Assert.Equal(queries.Length, context.QueryCount);
            Assert.Equal(context.QueryCount, context.PositiveNominations + context.NominationMisses + context.TraversalRefusals);
            Assert.Contains(logs, line => line.Contains("event=ready") && line.Contains("authority=cpu-physical-disc"));
            for (int i = 0; i < queries.Length; i++)
            {
                if ((i & 1) == 0)
                {
                    Assert.True(results[i].HasWitness);
                    Assert.True(context.TryValidateNomination(queries[i], results[i], out int owner)); Assert.Equal(0, owner);
                }
                else Assert.False(results[i].HasWitness);
            }
            GpuBoxQuery[] boxes = [new(0, -.8, 1, .8, .004 - GpuConvexBoxWitnessValidator.GeometryReserve), new(0, 2, 1, 3, .004)];
            var boxResults = new GpuBoxNomination[boxes.Length];
            Assert.True(context.TryNominateBoxes(boxes, boxResults, boxes.Length, default, out reason), reason);
            Assert.True(boxResults[0].HasWitness); Assert.True(context.TryValidateBoxNomination(boxes[0], boxResults[0], out int boxOwner));
            Assert.Equal(0, boxOwner); Assert.False(boxResults[1].HasWitness); Assert.Equal(boxes.Length, context.BoxQueryCount);
        }
        Assert.Contains(logs, line => line.Contains("event=disposed"));
    }

    private static VCarveCutterProfile Ball() => new([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)], 1);
    private static GpuDiscSegment Segment() => new(0, 0, .5, 1, 0, .5, 1, 0);
    private static GpuDiscNode Node() => new(0, 0, 1, 0, 1, 0, 1, -1, -1);
}
