using Clipper2Lib;
using System.Numerics;
using System.Reflection;
using System.Runtime.InteropServices;
using RW.Toolpaths.Milling.Acceleration;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class VCarveGpuBoundarySampleTests(ITestOutputHelper output)
{
    private static GpuBoundaryWorkLimits Limits => GpuBoundaryWorkLimits.Default;

    [Fact]
    public void BoundaryAbiRetainsOriginalGridAndFiniteEdgeOrdinal()
    {
        Assert.Equal(32, Marshal.SizeOf<GpuBoundarySegment>());
        Assert.Equal(48, Marshal.SizeOf<GpuBoundaryNode>());
        Assert.Equal(16, Marshal.SizeOf<GpuBoundaryQuery>());
        Assert.Equal(32, Marshal.SizeOf<GpuBoundarySampleNomination>());
        Assert.Equal(0, Marshal.OffsetOf<GpuBoundarySegment>(nameof(GpuBoundarySegment.Ax)).ToInt32());
        Assert.Equal(24, Marshal.OffsetOf<GpuBoundarySegment>(nameof(GpuBoundarySegment.By)).ToInt32());
        Assert.Equal(32, Marshal.OffsetOf<GpuBoundaryNode>(nameof(GpuBoundaryNode.First)).ToInt32());
        Assert.Equal(44, Marshal.OffsetOf<GpuBoundaryNode>(nameof(GpuBoundaryNode.Right)).ToInt32());
        Assert.Equal(16, Marshal.OffsetOf<GpuBoundarySampleNomination>(nameof(GpuBoundarySampleNomination.SegmentOrdinal)).ToInt32());
        Assert.Equal(28, Marshal.OffsetOf<GpuBoundarySampleNomination>(nameof(GpuBoundarySampleNomination.RefusalReason)).ToInt32());
        var edge = new GpuBoundarySegment(new Point64(17, -29), new Point64(-31, 43));
        Assert.Equal(17, edge.Ax); Assert.Equal(-29, edge.Ay); Assert.Equal(-31, edge.Bx); Assert.Equal(43, edge.By);
        Assert.False(GpuBoundarySampleNomination.Refused(GpuBoundaryRefusal.NodeBudget).HasNomination);
        output.WriteLine("[VCarve.BoundarySample.Test] ABI=32/48/16/32 originalPoint64=True nominationOnly=True authority=none");
    }

    [Fact]
    public void EmptyOutOfDomainOrInvalidWorkInputsRefuseBeforeCuda()
    {
        var edge = new GpuBoundarySegment(0, 0, 10, 0);
        GpuBoundarySegment[][] invalidSegments = [[], [new(long.MinValue, 0, 0, 0)],
            [new(1L << 29, 0, 0, 0)], [new(0, 0, 0, -(1L << 29))]];
        foreach (var invalid in invalidSegments)
        {
            Assert.False(VCarveGpuDiscWitnessContext.BoundarySampleContext.TryCreate(invalid,
                [new(0, 0, 10, 0, 0, 1, -1, -1)], default, output.WriteLine, out var context, out string reason));
            Assert.Null(context); Assert.False(string.IsNullOrWhiteSpace(reason));
        }
        foreach (var limits in new[] { new GpuBoundaryWorkLimits(0, 1), new(4097, 1), new(1, 0), new(1, 65_537) })
            Assert.False(VCarveGpuDiscWitnessContext.BoundarySampleContext.ValidateSnapshot([edge], Build([edge]), limits, default, out _));
        Assert.False(VCarveGpuDiscWitnessContext.BoundarySampleContext.ValidateSnapshot([edge], [], Limits, default, out _));
    }

    [Fact]
    public void EveryNodeLeafAndEdgeMustBelongToOneEnclosingAcyclicRoot()
    {
        GpuBoundarySegment[] edges = [new(0, 0, 10, 0), new(20, 0, 30, 0)];
        GpuBoundaryNode[][] invalid =
        [
            [new(10, 0, 0, 0, 0, 2, -1, -1)],
            [new(0, 0, 30, 0, -1, 2, -1, -1)],
            [new(0, 0, 30, 0, 0, 3, -1, -1)],
            [new(0, 0, 30, 0, 0, 0, -1, -1)],
            [new(0, 0, 29, 0, 0, 2, -1, -1)],
            [new(0, 0, 30, 0, 0, 1, -1, -1)],
            [new(0, 0, 30, 0, 0, 2, -1, -1), new(0, 0, 10, 0, 0, 1, -1, -1)],
            [new(0, 0, 30, 0, 0, 0, 1, 1), new(0, 0, 30, 0, 0, 2, -1, -1)],
            [new(0, 0, 30, 0, 0, 0, 1, -1), new(0, 0, 30, 0, 0, 2, -1, -1)],
            [new(0, 0, 30, 0, 0, 0, 1, 2), new(0, 0, 10, 0, 0, 1, -1, -1), new(0, 0, 10, 0, 0, 1, -1, -1)],
            [new(0, 0, 20, 0, 0, 0, 1, 2), new(0, 0, 10, 0, 0, 1, -1, -1), new(20, 0, 30, 0, 1, 1, -1, -1)],
            [new(0, 0, 30, 0, 0, 0, 1, 2), new(0, 0, 30, 0, 0, 0, 0, 2), new(0, 0, 30, 0, 0, 2, -1, -1)],
            [new(-(1L << 29), 0, 30, 0, 0, 2, -1, -1)],
        ];
        foreach (var nodes in invalid)
        {
            Assert.False(VCarveGpuDiscWitnessContext.BoundarySampleContext.ValidateSnapshot(edges, nodes, Limits, default, out string reason));
            Assert.False(string.IsNullOrWhiteSpace(reason));
        }
        Assert.True(VCarveGpuDiscWitnessContext.BoundarySampleContext.ValidateSnapshot(edges, Build(edges, 1), Limits, default, out string validReason), validReason);
        output.WriteLine("[VCarve.BoundarySample.Test] duplicateEdgesOrNodes=False incompleteCoverage=False cycles=False excludedBounds=False acceptedValidatedTree=True");
    }

    [Fact]
    public void DepthBeyondNativeStackIsRefusedDuringSnapshotValidation()
    {
        const int count = 66;
        var edges = Enumerable.Range(0, count).Select(i => new GpuBoundarySegment(i, 0, i, 1)).ToArray();
        var nodes = new List<GpuBoundaryNode>();
        int Chain(int first)
        {
            int ordinal = nodes.Count; nodes.Add(default);
            if (first == count - 1) nodes[ordinal] = new(first, 0, first, 1, first, 1, -1, -1);
            else
            {
                int left = nodes.Count; nodes.Add(new(first, 0, first, 1, first, 1, -1, -1));
                int right = Chain(first + 1); nodes[ordinal] = new(first, 0, count - 1, 1, 0, 0, left, right);
            }
            return ordinal;
        }
        Chain(0);
        Assert.False(VCarveGpuDiscWitnessContext.BoundarySampleContext.ValidateSnapshot(edges, nodes.ToArray(), Limits, default, out string reason));
        Assert.Equal("boundary-bvh-depth", reason);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ActualCudaNestedRingsUseNonZeroRatherThanEvenOddAndRespectHoles(bool innerSameDirection)
    {
        var edges = Ring(-100, -100, 100, 100).Concat(Ring(-40, -40, 40, 40, !innerSameDirection))
            .Concat(Ring(-10, -10, 10, 10)).ToArray();
        if (!Create(edges, Build(edges, 2), Limits, out var context)) return;
        using (context)
        {
            GpuBoundaryQuery[] queries = [new(0, 0), new(20, 0), new(60, 0), new(101, 0),
                new(-100, -100), new(100, 100), new(40, 0), new(-40, 0), new(0, -40), new(0, 40)];
            var nominations = Sample(context!, edges, queries);
            Assert.Equal(innerSameDirection ? 3 : 1, nominations[0].Winding);
            Assert.Equal(innerSameDirection ? 2 : 0, nominations[1].Winding);
            Assert.Equal(1, nominations[2].Winding); Assert.Equal(0, nominations[3].Winding);
            output.WriteLine($"[VCarve.BoundarySample.Test] actualCuda=True nestedRings=True sameDirection={innerSameDirection} exactNonZero=True boundaryPolicyAuthority=False");
        }
    }

    [Fact]
    public void ActualCudaFiniteEndpointsDegeneratesAndOtherBranchesRemainVisible()
    {
        GpuBoundarySegment[] edges = [new(0, 0, 10, 0), new(30, 0, 30, 0),
            new(100, -50, 100, 50), new(-100, 20, -90, 20)];
        if (!Create(edges, Build(edges, 1), Limits, out var context)) return;
        using (context)
        {
            GpuBoundaryQuery[] queries = [new(20, 0), new(31, 4), new(-5, 3), new(5, 4), new(10, 0), new(99, 0)];
            var n = Sample(context!, edges, queries);
            Assert.Equal(0, n[0].SegmentOrdinal); Assert.Equal(1, n[0].Parameter); Assert.Equal(100, n[0].SquaredDistance);
            Assert.Equal(1, n[1].SegmentOrdinal); Assert.Equal(0, n[1].Parameter); Assert.Equal(17, n[1].SquaredDistance);
            Assert.Equal(0, n[2].Parameter); Assert.Equal(34, n[2].SquaredDistance);
            Assert.Equal(.5, n[3].Parameter); Assert.Equal(16, n[3].SquaredDistance);
            Assert.Equal(0, n[4].SquaredDistance); Assert.Equal(2, n[5].SegmentOrdinal);
            // The far vertical edge is needed for winding even where a much
            // nearer horizontal edge already fixes the distance nomination.
            Assert.Equal(1, n[3].Winding);
            output.WriteLine("[VCarve.BoundarySample.Test] actualCuda=True finiteProjection=True degenerateFiniteEdge=True windingFarBranch=True");
        }
    }

    [Fact]
    public void ActualCudaUses4096BatchesAndImmutableOriginalSnapshots()
    {
        var edges = Ring(-100, -100, 100, 100).ToArray(); var originals = (GpuBoundarySegment[])edges.Clone();
        var nodes = Build(edges, 1);
        if (!Create(edges, nodes, Limits, out var context)) return;
        using (context)
        {
            edges[0] = new(0, 0, 0, 0); nodes[0] = default;
            Assert.True(context!.TryGetOriginalSegment(0, out var original));
            Assert.Equal(originals[0].Ax, original.Ax); Assert.Equal(originals[0].Bx, original.Bx);
            var queries = Enumerable.Range(0, VCarveGpuDiscWitnessContext.MaximumBatchSize + 1)
                .Select(i => new GpuBoundaryQuery(i % 241 - 120, i % 199 - 99)).ToArray();
            Sample(context, originals, queries);
            Assert.Equal(2, context.Batches); Assert.Equal(queries.Length, context.QueryCount);
            Assert.Equal(context.QueryCount, context.CompletedNominations);
            Assert.True(context.UploadedBytes > 0);
            output.WriteLine($"[VCarve.BoundarySample.Test] actualCuda=True immutableSnapshot=True batches={context.Batches} queries={context.QueryCount} kernelMs={context.KernelMilliseconds:F3}");
        }
    }

    [Fact]
    public void ActualCudaIntegerWindingKeepsCrossesExactAtAdmittedDomainExtremes()
    {
        long limit = VCarveGpuDiscWitnessContext.BoundarySampleContext.ExclusiveCoordinateLimit - 1;
        var edges = Ring(-limit, -limit, limit, limit).ToArray();
        if (!Create(edges, Build(edges, 1), Limits, out var context)) return;
        using (context)
        {
            Sample(context!, edges, [new(0, 0), new(limit, 0), new(-limit, 1), new(1, -limit), new(-1, limit)]);
            output.WriteLine("[VCarve.BoundarySample.Test] actualCuda=True int64Winding=True independentBigIntegerReference=True crossBoundLessThan2Pow61=True");
        }
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ActualCudaBudgetRefusalClearsPartialNearestAndWinding(bool nodeBudget)
    {
        var edges = Ring(-100, -100, 100, 100).ToArray();
        var limits = nodeBudget ? new GpuBoundaryWorkLimits(1, 100) : new(100, 1);
        var nodes = nodeBudget ? Build(edges, 1) : Build(edges, 4);
        if (!Create(edges, nodes, limits, out var context)) return;
        using (context)
        {
            GpuBoundaryQuery[] queries = [new(0, 0)]; var nominations = new GpuBoundarySampleNomination[1];
            Assert.True(context!.TryNominate(queries, nominations, 1, default, out string reason), reason);
            var n = nominations[0]; Assert.False(n.HasNomination); Assert.Equal(2, n.Status);
            Assert.Equal(nodeBudget ? (int)GpuBoundaryRefusal.NodeBudget : (int)GpuBoundaryRefusal.SegmentBudget, n.RefusalReason);
            Assert.Equal(-1, n.SegmentOrdinal); Assert.Equal(0, n.Winding);
            Assert.True(double.IsNaN(n.Parameter)); Assert.True(double.IsPositiveInfinity(n.SquaredDistance));
            Assert.Equal(1, nodeBudget ? context.NodeBudgetRefusals : context.SegmentBudgetRefusals);
            output.WriteLine($"[VCarve.BoundarySample.Test] actualCuda=True budgetRefused=True kind={(nodeBudget ? "nodes" : "segments")} allNomineeFieldsCleared=True");
        }
    }

    [Fact]
    public void ActualCudaQueryDomainRefusalAndCancellationClearEarlierBatches()
    {
        var edges = Ring(-100, -100, 100, 100).ToArray();
        if (!Create(edges, Build(edges, 1), Limits, out var context)) return;
        using (context)
        {
            var queries = Enumerable.Repeat(new GpuBoundaryQuery(0, 0), VCarveGpuDiscWitnessContext.MaximumBatchSize + 1).ToArray();
            queries[^1] = new(1L << 29, 0); var nominations = new GpuBoundarySampleNomination[queries.Length];
            Assert.False(context!.TryNominate(queries, nominations, queries.Length, default, out string reason));
            Assert.Equal("boundary-query-domain", reason);
            Assert.All(nominations, n => { Assert.False(n.HasNomination); Assert.Equal(-1, n.SegmentOrdinal); Assert.Equal(0, n.Winding); });
            // Unsupported queries do not destroy a healthy immutable context.
            Sample(context, edges, [new(0, 0)]);
            using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
            Assert.Throws<OperationCanceledException>(() => context.TryNominate([new(0, 0)], nominations, 1, cancelled.Token, out _));
            Assert.False(nominations[0].HasNomination);
            output.WriteLine("[VCarve.BoundarySample.Test] actualCuda=True invalidLateBatchClearedAll=True contextRemainsUsable=True cancelledOutputNotPublished=True");
        }
    }

    [Fact]
    public void DisposedAndFailedContextsCannotReuseNominationBuffers()
    {
        var edges = Ring(-100, -100, 100, 100).ToArray();
        // No CUDA setup is needed to falsify lifecycle output handling.
        var constructor = typeof(VCarveGpuDiscWitnessContext.BoundarySampleContext)
            .GetConstructors(BindingFlags.Instance | BindingFlags.NonPublic).Single();
        var context = (VCarveGpuDiscWitnessContext.BoundarySampleContext)constructor.Invoke(new object?[] { edges, Build(edges), Limits, null });
        context.Dispose(); context.Dispose();
        var nominations = new[] { new GpuBoundarySampleNomination(0, .5, 0, 1, 1, GpuBoundaryRefusal.None) };
        Assert.Throws<ArgumentOutOfRangeException>(() => context.TryNominate([new(0, 0)], nominations, -1, default, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => context.TryNominate([new(0, 0)], nominations, 2, default, out _));
        Assert.Throws<ArgumentNullException>(() => context.TryNominate(null!, nominations, 1, default, out _));
        Assert.False(context.TryNominate([new(0, 0)], nominations, 1, default, out string reason));
        Assert.Equal("context-disposed", reason); Assert.False(nominations[0].HasNomination);
        Assert.False(context.TryGetOriginalSegment(0, out _));
        var failed = (VCarveGpuDiscWitnessContext.BoundarySampleContext)constructor.Invoke(new object?[] { edges, Build(edges), Limits, null });
        typeof(VCarveGpuDiscWitnessContext.BoundarySampleContext).GetField("_boundaryFailed", BindingFlags.Instance | BindingFlags.NonPublic)!.SetValue(failed, true);
        Assert.False(failed.TryNominate([new(0, 0)], nominations, 1, default, out _));
        Assert.False(nominations[0].HasNomination); failed.Dispose();
        using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveGpuDiscWitnessContext.BoundarySampleContext.TryCreate(edges, Build(edges),
            cancelled.Token, null, out _, out _));
    }

    [Fact]
    public void DiagnosticObserverFailureDoesNotGrantAuthorityOrDiscardCompletedSamples()
    {
        var edges = Ring(-100, -100, 100, 100).ToArray();
        bool created = VCarveGpuDiscWitnessContext.BoundarySampleContext.TryCreate(edges, Build(edges), default,
            _ => throw new InvalidOperationException("diagnostic-only"), out var context, out string reason);
        if (!created) { Assert.Null(context); Assert.False(string.IsNullOrWhiteSpace(reason)); output.WriteLine($"[VCarve.BoundarySample.Test] actualCuda=False portableRefusal='{reason}'"); return; }
        using (context)
        {
            Sample(context!, edges, [new(0, 0)]);
            Assert.True(context!.DiagnosticObserverFailures >= 2);
            output.WriteLine("[VCarve.BoundarySample.Test] actualCuda=True observerFailed=True samplesCompleted=True authority=none");
        }
    }

    [Fact]
    public void ObserverCancellingAtFinalDiagnosticClearsResultsAndPropagatesCallerCancellation()
    {
        var edges = Ring(-100, -100, 100, 100).ToArray();
        using var cancellation = new CancellationTokenSource();
        bool created = VCarveGpuDiscWitnessContext.BoundarySampleContext.TryCreate(edges, Build(edges), cancellation.Token,
            message => { if (message.Contains("event=sampled", StringComparison.Ordinal)) cancellation.Cancel(); },
            out var context, out string reason);
        if (!created) { Assert.Null(context); Assert.False(string.IsNullOrWhiteSpace(reason)); output.WriteLine($"[VCarve.BoundarySample.Test] actualCuda=False portableRefusal='{reason}'"); return; }
        using (context)
        {
            var nominations = new GpuBoundarySampleNomination[1];
            Assert.Throws<OperationCanceledException>(() => context!.TryNominate([new(0, 0)], nominations, 1, cancellation.Token, out _));
            Assert.False(nominations[0].HasNomination); Assert.Equal((int)GpuBoundaryRefusal.Cancelled, nominations[0].RefusalReason);
            output.WriteLine("[VCarve.BoundarySample.Test] actualCuda=True finalObserverCancelled=True callerCancellationPropagated=True allOutputsRefused=True");
        }
    }

    private bool Create(GpuBoundarySegment[] edges, GpuBoundaryNode[] nodes, GpuBoundaryWorkLimits limits,
        out VCarveGpuDiscWitnessContext.BoundarySampleContext? context)
    {
        bool created = VCarveGpuDiscWitnessContext.BoundarySampleContext.TryCreate(edges, nodes, limits, default,
            output.WriteLine, out context, out string reason);
        if (!created)
        { Assert.Null(context); Assert.False(string.IsNullOrWhiteSpace(reason)); output.WriteLine($"[VCarve.BoundarySample.Test] actualCuda=False portableRefusal='{reason}'"); }
        return created;
    }

    private static GpuBoundarySampleNomination[] Sample(VCarveGpuDiscWitnessContext.BoundarySampleContext context,
        GpuBoundarySegment[] edges, GpuBoundaryQuery[] queries)
    {
        var nominations = new GpuBoundarySampleNomination[queries.Length];
        Assert.True(context.TryNominate(queries, nominations, queries.Length, default, out string reason), reason);
        for (int i = 0; i < queries.Length; i++)
        {
            var n = nominations[i]; Assert.True(n.HasNomination);
            Assert.Equal(ExactWinding(edges, queries[i]), n.Winding);
            double minimum = edges.Min(s => FiniteSquare(queries[i], s, out _));
            Assert.InRange(Math.Abs(n.SquaredDistance - minimum), 0, Math.Max(1e-9, minimum * 3e-13));
            double nominatedSquare = FiniteSquare(queries[i], edges[n.SegmentOrdinal], out double parameter);
            Assert.InRange(Math.Abs(n.Parameter - parameter), 0, 1e-13);
            Assert.InRange(Math.Abs(n.SquaredDistance - nominatedSquare), 0, Math.Max(1e-9, nominatedSquare * 3e-13));
        }
        return nominations;
    }

    private static int ExactWinding(GpuBoundarySegment[] edges, GpuBoundaryQuery q)
    {
        int winding = 0;
        foreach (var s in edges)
        {
            BigInteger cross = ((BigInteger)s.Bx - s.Ax) * ((BigInteger)q.Y - s.Ay)
                - ((BigInteger)s.By - s.Ay) * ((BigInteger)q.X - s.Ax);
            if (s.Ay <= q.Y && s.By > q.Y && cross.Sign > 0) winding++;
            else if (s.Ay > q.Y && s.By <= q.Y && cross.Sign < 0) winding--;
        }
        return winding;
    }

    private static double FiniteSquare(GpuBoundaryQuery q, GpuBoundarySegment s, out double parameter)
    {
        double dx = s.Bx - s.Ax, dy = s.By - s.Ay, qx = q.X - s.Ax, qy = q.Y - s.Ay;
        double square = dx * dx + dy * dy;
        parameter = square > 0 ? Math.Clamp((qx * dx + qy * dy) / square, 0, 1) : 0;
        double ex = qx - parameter * dx, ey = qy - parameter * dy; return ex * ex + ey * ey;
    }

    private static IEnumerable<GpuBoundarySegment> Ring(long x0, long y0, long x1, long y1, bool clockwise = false)
    {
        Point64[] vertices = [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
        if (clockwise) Array.Reverse(vertices);
        for (int i = 0; i < vertices.Length; i++) yield return new(vertices[i], vertices[(i + 1) % vertices.Length]);
    }

    private static GpuBoundaryNode[] Build(GpuBoundarySegment[] edges, int leafCapacity = 4)
    {
        var nodes = new List<GpuBoundaryNode>();
        int Add(int first, int count)
        {
            int ordinal = nodes.Count; nodes.Add(default);
            long x0 = long.MaxValue, y0 = long.MaxValue, x1 = long.MinValue, y1 = long.MinValue;
            for (int i = first; i < first + count; i++)
            {
                x0 = Math.Min(x0, Math.Min(edges[i].Ax, edges[i].Bx)); y0 = Math.Min(y0, Math.Min(edges[i].Ay, edges[i].By));
                x1 = Math.Max(x1, Math.Max(edges[i].Ax, edges[i].Bx)); y1 = Math.Max(y1, Math.Max(edges[i].Ay, edges[i].By));
            }
            if (count <= leafCapacity) nodes[ordinal] = new(x0, y0, x1, y1, first, count, -1, -1);
            else { int half = count / 2, left = Add(first, half), right = Add(first + half, count - half); nodes[ordinal] = new(x0, y0, x1, y1, 0, 0, left, right); }
            return ordinal;
        }
        Add(0, edges.Length); return nodes.ToArray();
    }
}
