using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class ParallelSweepUnionControls
{
    [Fact]
    public void HardwareBudgetKeepsLiteralOperandsOrderedAndNestedWorkSerial()
    {
        int workers = GeometryParallelism.HardwareWorkerLimit;
        Assert.InRange(workers, 1, Math.Min(12, Environment.ProcessorCount));
        Assert.InRange(GeometryParallelism.StockFitWorkerLimit,1,Math.Min(16,Environment.ProcessorCount));
        Assert.True(GeometryParallelism.StockFitWorkerLimit<=Math.Max(1,GC.GetGCMemoryInfo().TotalAvailableMemoryBytes/(2L<<30)));
        using var queue = new VCarveOrderedUnionQueue(default, workers);
        var inputs = Enumerable.Range(0, 2 * workers).Select(i => new Paths64 {
            Rectangle(i * 100, 0, i * 100 + 40, 40),
            Reverse(Rectangle(i * 100 + 10, 10, i * 100 + 30, 30)) }).ToArray();
        var slots = inputs.Select(queue.Submit).ToArray();
        queue.Complete();
        for (int i = 0; i < slots.Length; i++)
            Exact(Clipper.Union(inputs[i], FillRule.NonZero), queue.Read(slots[i]));
        Assert.InRange(queue.PeakRetainedInputBytes, 1, VCarveOrderedUnionQueue.MaximumRetainedInputBytes);
        Assert.True(queue.OwnedWorkersJoined);
        using var scope = GeometryParallelism.EnterWorker();
        Assert.Equal(1, GeometryParallelism.WorkerLimit);
        Assert.Equal(1, GeometryParallelism.StockFitWorkerLimit);
        Assert.Throws<ArgumentOutOfRangeException>(() => new VCarveOrderedUnionQueue(default, 2));
    }

    [Fact]
    public void LargerRoughingLeavesRetainTheirSignedFillAndByteBudget()
    {
        var input=new Paths64(Enumerable.Range(0,128).Select(i=>Rectangle(i,0,i+2,10)));
        using var original=new VCarveOrderedUnionQueue(default,2);
        Assert.Throws<ArgumentException>(()=>original.Submit(input));
        using var queue=new VCarveOrderedUnionQueue(default,2,maximumPathsPerLeaf:128);
        var slot=queue.Submit(input);queue.Complete();
        Exact(Clipper.Union(input,FillRule.NonZero),queue.Read(slot));
        Assert.InRange(queue.PeakRetainedInputBytes,1,VCarveOrderedUnionQueue.MaximumRetainedInputBytes);
        Assert.True(queue.OwnedWorkersJoined);
    }

    [Fact]
    public void StreamingWindowsReadOnlyDrainedResultsAndStillJoinBeforeFinalPublication()
    {
        using var blocked=new ManualResetEventSlim();
        using var started=new ManualResetEventSlim();
        using var queue=new VCarveOrderedUnionQueue(default,2,input=>{
            started.Set();Assert.True(blocked.Wait(TimeSpan.FromSeconds(10)));
            return Clipper.Union(input,FillRule.NonZero);
        });
        var input=new Paths64{Rectangle(0,0,40,40)};
        var slot=queue.Submit(input);
        Assert.True(started.Wait(TimeSpan.FromSeconds(10)));
        try{Assert.Throws<InvalidOperationException>(()=>queue.ReadDrained(slot));}
        finally{blocked.Set();}
        queue.Drain();Exact(Clipper.Union(input,FillRule.NonZero),queue.ReadDrained(slot));
        Assert.Throws<InvalidOperationException>(()=>queue.Read(slot));
        queue.Complete();Assert.True(queue.OwnedWorkersJoined);
        Exact(queue.ReadDrained(slot),queue.Read(slot));
    }

    [Fact]
    public void OrderedGroupsRetainHoleOwnershipDespiteReverseCompletion()
    {
        using var released = new ManualResetEventSlim();
        int others = 0;
        var groups = Enumerable.Range(0, 4).Select(i => new Paths64 {
            Rectangle(i * 100, 0, i * 100 + 40, 40),
            Reverse(Rectangle(i * 100 + 10, 10, i * 100 + 30, 30)) }).ToArray();
        Paths64 Union(Paths64 input)
        {
            if (input[0][0].X == 0)
            {
                if (!released.Wait(TimeSpan.FromSeconds(10))) throw new TimeoutException("reverse completion control");
                Assert.Equal(3, Volatile.Read(ref others));
            }
            else if (Interlocked.Increment(ref others) == 3) released.Set();
            return Clipper.Union(input, FillRule.NonZero);
        }
        using var queue = new VCarveOrderedUnionQueue(CancellationToken.None, 4, Union);
        var slots = groups.Select(queue.Submit).ToArray();
        queue.Complete();
        for (int i = 0; i < groups.Length; i++)
            Exact(Clipper.Union(groups[i], FillRule.NonZero), queue.Read(slots[i]));
        Assert.Equal(4, queue.Submitted);
        Assert.InRange(queue.PeakInFlight, 1, 4);
        Assert.True(queue.OwnedWorkersJoined);
        var expected = Clipper.Union(new Paths64(groups.SelectMany(g => g)), FillRule.NonZero);
        var actual = Clipper.Union(new Paths64(slots.SelectMany(s => queue.Read(s))), FillRule.NonZero);
        Assert.Empty(Clipper.Difference(expected, actual, FillRule.NonZero));
        Assert.Empty(Clipper.Difference(actual, expected, FillRule.NonZero));
        Assert.Equal(4, actual.Count(path => Clipper.Area(path) < 0));
    }

    [Fact]
    public void DetachedOuterListAndOversizedInputRetainLiteralFallback()
    {
        using var queue = new VCarveOrderedUnionQueue(CancellationToken.None, 2, maximumBytes: 1024);
        var source = new Paths64 { Rectangle(0, 0, 20, 20) };
        var expected = Clipper.Union(source, FillRule.NonZero);
        var submitted = queue.Submit(source);
        source.Clear();
        var largePath = Rectangle(50, 0, 80, 30);
        largePath.Capacity = 256;
        var oversized = new Paths64 { largePath, Reverse(Rectangle(55, 5, 75, 25)) };
        var fallback = queue.Submit(oversized);
        queue.Complete();
        Exact(expected, queue.Read(submitted));
        Exact(Clipper.Union(oversized, FillRule.NonZero), queue.Read(fallback));
        Assert.Equal(1, queue.Submitted);
        Assert.Equal(1, queue.LiteralFallbacks);
        Assert.InRange(queue.PeakRetainedInputBytes, 1, 1024);
    }

    [Fact]
    public void CancelledWorkerPublishesNothingAndDisposalJoins()
    {
        using var cancellation = new CancellationTokenSource();
        var queue = new VCarveOrderedUnionQueue(cancellation.Token, 2, input => {
            var result = Clipper.Union(input, FillRule.NonZero);
            cancellation.Cancel(); return result;
        });
        try
        {
            var slot = queue.Submit(new Paths64 { Rectangle(0, 0, 40, 40) });
            Assert.ThrowsAny<OperationCanceledException>(() => queue.Complete());
            Assert.Throws<InvalidOperationException>(() => queue.Read(slot));
            Assert.ThrowsAny<OperationCanceledException>(()=>queue.ReadDrained(slot));
        }
        finally { queue.Dispose(); }
        Assert.True(queue.OwnedWorkersJoined);
        queue.Dispose();
    }

    [Fact]
    public void WorkerFailurePreservesExceptionAndDrainsBeforeReturn()
    {
        var queue = new VCarveOrderedUnionQueue(CancellationToken.None, 2,
            input => throw new FormatException("retained union failure"));
        try
        {
            var error = Assert.Throws<FormatException>(() => {
                queue.Submit(new Paths64 { Rectangle(0, 0, 40, 40) });
                queue.Complete();
            });
            Assert.Equal("retained union failure", error.Message);
        }
        finally { queue.Dispose(); }
        Assert.True(queue.OwnedWorkersJoined);
    }

    [Fact]
    public void ProducerOffsetsShareExistingWorkerPermitOwner()
    {
        int active = 0, peak = 0;
        void Begin()
        {
            int current = Interlocked.Increment(ref active), old;
            do { old = Volatile.Read(ref peak); if (old >= current) break; }
            while (Interlocked.CompareExchange(ref peak, current, old) != old);
        }
        using var queue = new VCarveOrderedUnionQueue(CancellationToken.None, 2, input => {
            Begin();
            try { Thread.Sleep(1); return Clipper.Union(input, FillRule.NonZero); }
            finally { Interlocked.Decrement(ref active); }
        });
        var slots = new List<VCarveOrderedUnionQueue.Slot>();
        for (int i = 0; i < 16; i++)
        {
            slots.Add(queue.Submit(new Paths64 { Rectangle(i * 50, 0, i * 50 + 40, 40) }));
            using (queue.EnterProducerGeometry())
            {
                Begin();
                try { Clipper.Union(new Paths64 { Rectangle(0, 0, 40, 40) }, FillRule.NonZero); }
                finally { Interlocked.Decrement(ref active); }
            }
        }
        queue.Complete();
        Assert.Equal(0, active);
        Assert.InRange(peak, 1, 2);
        Assert.InRange(queue.PeakInFlight, 1, 2);
        Assert.All(slots, slot => Assert.NotEmpty(queue.Read(slot)));
    }

    [Fact]
    public void CompletePhysicalSweepsMatchLiteralUpperLowerAndNestedOwner()
    {
        var profile = new VCarveCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.5875, 3, 20) }, 3);
        var paths = new List<TaggedToolpath>();
        for (int path = 0; path < 40; path++)
        {
            var points = Enumerable.Range(0, 160).Select(i => new Point3D(
                path * 2.5 + i * .008, Math.Sin(i * .08) * .12,
                -.08 - (i % 11) * .006)).ToList();
            paths.Add(new TaggedToolpath(points, Array.Empty<ToolpathSpan>(), 0,
                "parallel-union-control", null, detectCurves: false));
        }
        paths.Add(new TaggedToolpath(new List<Point3D> {
            new(0, 4, -.2), new(4, 4, -.2), new(4, 8, -.2), new(0, 8, -.2), new(0, 4, -.2)
        }, Array.Empty<ToolpathSpan>(), 0, "closed-original-roundcaps", null, detectCurves: false));
        foreach (bool upper in new[] { false, true })
        {
            var expected = VCarvePhysicalSweepOriginalOracle.BuildWithTaperedRadiusRuns(paths,
                profile, 0, .004, upperBound: upper, consolidateHemisphereChords: true,
                enableTaperedRadiusRuns: true);
            var actual = VCarvePhysicalSweep.BuildWithParallelLeafUnions(paths,
                profile, 0, .004, upperBound: upper, consolidateHemisphereChords: true,
                enableTaperedRadiusRuns: true);
            ExactWorld(expected, actual);
            // Normal production construction must adopt the same execution-only
            // acceleration for both upper and lower dense sweeps.
            var automatic = VCarvePhysicalSweep.BuildWithTaperedRadiusRuns(paths,
                profile, 0, .004, upperBound: upper, consolidateHemisphereChords: true,
                enableTaperedRadiusRuns: true);
            ExactWorld(expected, automatic);
            using var worker = GeometryParallelism.EnterWorker();
            var nested = VCarvePhysicalSweep.BuildWithParallelLeafUnions(paths,
                profile, 0, .004, upperBound: upper, consolidateHemisphereChords: true,
                enableTaperedRadiusRuns: true);
            ExactWorld(expected, nested);
        }
    }

    private static Path64 Rectangle(long x0, long y0, long x1, long y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
    private static Path64 Reverse(Path64 path) { path.Reverse(); return path; }
    private static void Exact(Paths64 expected, Paths64 actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int p = 0; p < expected.Count; p++)
        {
            Assert.Equal(expected[p].Count, actual[p].Count);
            for (int i = 0; i < expected[p].Count; i++)
            { Assert.Equal(expected[p][i].X, actual[p][i].X); Assert.Equal(expected[p][i].Y, actual[p][i].Y); }
        }
    }
    private static void ExactWorld(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
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
}
