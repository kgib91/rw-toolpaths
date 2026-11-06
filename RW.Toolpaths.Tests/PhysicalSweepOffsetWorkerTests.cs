using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class PhysicalSweepOffsetWorkerTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DenseMixedRoundOffsetLeavesRetainTheLiteralSerialFootprint(bool upper)
    {
        const double surface = 12.7;
        var profile = new VCarveCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, .8, .8),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, .8, 3, 6) }, 4);
        var paths = new List<TaggedToolpath>();
        for (int row = 0; row < 28; row++)
        {
            var points = Enumerable.Range(0, 192).Select(i => new Point3D(
                i * .013, row * .045 + Math.Sin(i * .05) * .04,
                surface - (row % 2 == 0 ? .3 : .65))).ToArray();
            paths.Add(Path(points));
            // Interleave nonconstant pieces with flat-offset groups, retaining
            // the exact original hierarchy and signed component custody.
            paths.Add(Path(new[] { new Point3D(0, row * .045, surface - .1),
                new Point3D(.5, row * .045 + .02, surface - .5) }));
        }
        paths.Add(Path(new[] { new Point3D(0, 0, surface - .5), new(3, 0, surface - .5),
            new(3, 3, surface - .5), new(0, 3, surface - .5), new(0, 0, surface - .5) }));
        paths.Add(Path(new[] { new Point3D(.01, .01, surface - .5), new(.01, .01, surface - .5) }));
        var expected = VCarvePhysicalSweepOriginalOracle.BuildWithTaperedRadiusRuns(paths,
            profile, surface, .004, upperBound: upper);
        var actual = VCarvePhysicalSweep.Build(paths, profile, surface, .004, upperBound: upper);
        Assert.Equal(expected.Count, actual.Count);
        for (int p = 0; p < expected.Count; p++)
        {
            Assert.Equal(expected[p].Count, actual[p].Count);
            for (int i = 0; i < expected[p].Count; i++)
            {
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected[p][i].x), BitConverter.DoubleToInt64Bits(actual[p][i].x));
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected[p][i].y), BitConverter.DoubleToInt64Bits(actual[p][i].y));
            }
        }
    }

    [Fact]
    public void MixedOffsetAndUnionRequestsShareTheSameBoundedOwnerAndOrderedSlots()
    {
        int active = 0, peak = 0;
        Paths64 Work(Paths64 input, bool offset)
        {
            int current = Interlocked.Increment(ref active), old;
            do { old = Volatile.Read(ref peak); if (old >= current) break; }
            while (Interlocked.CompareExchange(ref peak, current, old) != old);
            try
            {
                Assert.Equal(1, RW.Toolpaths.Geometry.GeometryParallelism.WorkerLimit);
                if (!offset) return Clipper.Union(input, FillRule.NonZero);
                var owner = new ClipperOffset(2, 1);
                owner.AddPaths(input, JoinType.Round, EndType.Round);
                var result = new Paths64(); owner.Execute(4, result); return result;
            }
            finally { Interlocked.Decrement(ref active); }
        }
        using var queue = new VCarveOrderedUnionQueue(default, 2, input => Work(input, false));
        var slots = new List<VCarveOrderedUnionQueue.Slot>();
        var expected = new List<Paths64>();
        for (int i = 0; i < 20; i++)
        {
            var input = new Paths64 { new() { new(i * 10, 0), new(i * 10 + 3, 4), new(i * 10 + 6, 0) } };
            bool offset = i % 2 == 0;
            var owner = new ClipperOffset(2, 1);
            owner.AddPaths(input, JoinType.Round, EndType.Round);
            var rounded = new Paths64(); owner.Execute(4, rounded);
            expected.Add(offset ? rounded : Clipper.Union(input, FillRule.NonZero));
            slots.Add(offset ? queue.Submit(input, paths => Work(paths, true)) : queue.Submit(input));
            input.Clear();
        }
        queue.Complete();
        Assert.Equal(0, active);
        Assert.InRange(peak, 1, 2);
        Assert.InRange(queue.PeakInFlight, 1, 2);
        Assert.True(queue.OwnedWorkersJoined);
        for (int i = 0; i < slots.Count; i++) Exact(expected[i], queue.Read(slots[i]));
    }

    [Fact]
    public void OversizedOffsetUsesItsOwnLiteralRecipe()
    {
        using var queue = new VCarveOrderedUnionQueue(default, 2, _ => throw new InvalidOperationException("wrong recipe"), maximumBytes: 512);
        var path = new Path64 { new(0, 0), new(10, 0) }; path.Capacity = 100;
        var input = new Paths64 { path };
        var expected = new Paths64 { new() { new(1, 2), new(3, 4), new(5, 6) } };
        var slot = queue.Submit(input, received => { Assert.Same(input, received); return expected; });
        queue.Complete();
        Assert.Same(expected, queue.Read(slot));
        Assert.Equal(1, queue.LiteralFallbacks);
        Assert.True(queue.OwnedWorkersJoined);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OffsetFailureOrCancellationJoinsBeforePublication(bool cancel)
    {
        using var cancellation = new CancellationTokenSource();
        var queue = new VCarveOrderedUnionQueue(cancellation.Token, 2);
        try
        {
            VCarveOrderedUnionQueue.Slot? slot = null;
            void Run()
            {
                slot = queue.Submit(new Paths64 { new() { new(0, 0), new(10, 0) } }, input => {
                    if (cancel) { cancellation.Cancel(); return input; }
                    throw new FormatException("round-offset failure");
                });
                queue.Complete();
            }
            if (cancel) Assert.ThrowsAny<OperationCanceledException>(Run);
            else Assert.Equal("round-offset failure", Assert.Throws<FormatException>(Run).Message);
            if (slot is not null) Assert.Throws<InvalidOperationException>(() => queue.Read(slot));
        }
        finally { queue.Dispose(); }
        Assert.True(queue.OwnedWorkersJoined);
    }

    private static TaggedToolpath Path(IReadOnlyList<Point3D> points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "offset-worker-control", null, detectCurves: false);
    private static void Exact(Paths64 expected, Paths64 actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int p = 0; p < expected.Count; p++) Assert.Equal(expected[p].ToArray(), actual[p].ToArray());
    }
}
