using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Collections.Immutable;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class FilledFrontCoverageFallbackCaptureTests(ITestOutputHelper output)
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObserverPreservesExactSelectedGeometryAndCapturesLiteralOrderedPrefix(bool preparedContainment)
    {
        using var evidence = new ReferenceEvidence();
        var input = Fixture();
        var expected = FilledFrontCoverage.RetainUncoveredComponents(input, 2, default,
            usePreparedContainment: preparedContainment);
        var captures = new List<FilledFrontCoverageFallbackCapture.Snapshot>();
        using (var scope = FilledFrontCoverageFallbackCapture.BeginScope(captures.Add))
        {
            var actual = FilledFrontCoverage.RetainUncoveredComponents(input, 2, default,
                usePreparedContainment: preparedContainment);
            EqualBits(expected, actual);
            Assert.Equal(1, scope.Completed);
        }
        var captured = Assert.Single(captures);
        Assert.Equal(0, captured.MaterializedBefore);
        Assert.Equal(3, captured.MaterializedAfter);
        Assert.Equal(3, captured.AcceptedOriginalOrder.Length);
        Assert.False(captured.ProofParameters.EvidenceCacheEnabled);
        Assert.Equal(preparedContainment, captured.ProofParameters.PreparedContainment);
        Assert.True(captured.ClipWasNull); Assert.Empty(captured.Clip);
        var rebuilt = new Paths64();
        foreach (var footprint in captured.AcceptedOriginalOrder)
        {
            var ordered = new Paths64(rebuilt);
            ordered.AddRange(Integer(footprint.IntegerPaths));
            rebuilt = PolygonOps.UnionNormalizedInteger(ordered);
        }
        EqualInteger(rebuilt, captured.ExactOrderedPrefix);
        var fresh = PolygonOps.Difference(World(captured.Candidate.Paths),
            PolygonOps.MaterializeIntegerPaths(rebuilt));
        double error = BitConverter.Int64BitsToDouble(captured.ProofParameters.CoverageErrorBits);
        double arc = BitConverter.Int64BitsToDouble(captured.ProofParameters.ProofArcBits);
        var interior = PolygonOps.Offset(fresh, -error, arc / 4, 0);
        EqualBits(fresh, World(captured.Fresh)); EqualBits(interior, World(captured.ErodedInterior));
        Assert.Equal(Math.Abs(interior.Sum(PolygonOps.SignedArea)) != 0, captured.Retained);
        Assert.Equal(captured.Retained ? "original-boolean-retained" : "original-boolean-discarded", captured.Outcome);
        output.WriteLine($"[FrontCapture.Test] prefixLeaves={captured.AcceptedOriginalOrder.Length} "
            + $"combinedPoints={captured.CombinedPoints} retained={captured.Retained} exactPrefix=True exactMorphology=True");
    }

    [Fact]
    public void OneInvocationScopeCapsAllFallbackCopiesAtFourAcrossCalls()
    {
        using var evidence = new ReferenceEvidence();
        var input = Fixture(); var expected = FilledFrontCoverage.RetainUncoveredComponents(input, 2, default);
        var captures = new List<FilledFrontCoverageFallbackCapture.Snapshot>();
        using var scope = FilledFrontCoverageFallbackCapture.BeginScope(captures.Add);
        for (int call = 0; call < 7; call++)
            EqualBits(expected, FilledFrontCoverage.RetainUncoveredComponents(input, 2, default));
        Assert.Equal(4, scope.Reservations); Assert.Equal(4, scope.Completed); Assert.Equal(4, captures.Count);
        Assert.Equal(new[] { 1, 2, 3, 4 }, captures.Select(snapshot => snapshot.CaptureOrdinal));
    }

    [Fact]
    public void MinimumAcceptedFilterPreservesSmallDecisionsWithoutConsumingReservations()
    {
        using var evidence = new ReferenceEvidence();
        var small = new List<List<PointD>> { Rectangle(0, 0, 10, 10), Rectangle(10.01, 4, 10.011, 6) };
        var expectedSmall = FilledFrontCoverage.RetainUncoveredComponents(small, 2, default);
        var expectedLarge = FilledFrontCoverage.RetainUncoveredComponents(Fixture(), 2, default);
        var captures = new List<FilledFrontCoverageFallbackCapture.Snapshot>();
        using var scope = FilledFrontCoverageFallbackCapture.BeginScope(captures.Add, minimumAcceptedFootprints: 3);
        for (int call = 0; call < 7; call++)
            EqualBits(expectedSmall, FilledFrontCoverage.RetainUncoveredComponents(small, 2, default));
        Assert.Equal(0, scope.Reservations); Assert.Equal(0, scope.Completed); Assert.Empty(captures);
        EqualBits(expectedLarge, FilledFrontCoverage.RetainUncoveredComponents(Fixture(), 2, default));
        Assert.Equal(1, scope.Reservations); Assert.Equal(1, scope.Completed);
        Assert.Equal(3, Assert.Single(captures).MinimumAcceptedFootprints);
        Assert.Throws<ArgumentOutOfRangeException>(() => FilledFrontCoverageFallbackCapture.BeginScope(_ => { }, -1));
        Assert.Throws<ArgumentOutOfRangeException>(() => FilledFrontCoverageFallbackCapture.BeginScope(_ => { }, 16_385));
        Assert.Same(scope, FilledFrontCoverageFallbackCapture.Current);
        using (var maximum = FilledFrontCoverageFallbackCapture.BeginScope(_ => { }, 16_384))
            Assert.Equal(16_384, maximum.MinimumAcceptedFootprints);
        Assert.Same(scope, FilledFrontCoverageFallbackCapture.Current);
    }

    [Fact]
    public void InputAndResultPointBudgetsRefuseWithoutObserverOrSelectionAuthority()
    {
        int callbacks = 0;
        using var scope = FilledFrontCoverageFallbackCapture.BeginScope(_ => callbacks++);
        var world = new List<List<PointD>> { Enumerable.Repeat(new PointD(0, 0), 500_001).ToList() };
        var integer = new Paths64 { new Path64(Enumerable.Repeat(new Point64(0, 0), 500_001)) };
        Assert.Null(scope.TryBegin(0, 0, 1, 0, default, Array.Empty<IReadOnlyList<PointD>>(), null,
            new(integer, world), Array.Empty<FilledFrontCoverageFallbackCapture.FootprintInput>(), default));
        var small = Input(Rectangle(0, 0, 1, 1));
        var pending = scope.TryBegin(0, 0, 1, 0, default, Array.Empty<IReadOnlyList<PointD>>(), null,
            small, Array.Empty<FilledFrontCoverageFallbackCapture.FootprintInput>(), default);
        Assert.NotNull(pending);
        integer[0].AddRange(Enumerable.Repeat(new Point64(0, 0), 500_000));
        pending!.Complete(integer, null, null, 1, "diagnostic-test", true, default);
        Assert.Equal(0, callbacks); Assert.Equal(0, scope.Completed); Assert.Equal(2, scope.Reservations);
    }

    [Fact]
    public void CaptureSnapshotsRemainDetachedAfterAllInputAndOutputOwnersMutate()
    {
        FilledFrontCoverageFallbackCapture.Snapshot? snapshot = null;
        using var scope = FilledFrontCoverageFallbackCapture.BeginScope(value => snapshot = value);
        var candidate = Input(Rectangle(1, 1, 2, 2)); var accepted = Input(Rectangle(0, 0, 3, 3));
        var canonical = new List<IReadOnlyList<PointD>> { Rectangle(1, 1, 2, 2) };
        var clip = Rectangle(-10, -10, 10, 10);
        var pending = scope.TryBegin(7, 8, 9, 0, default, canonical, clip, candidate, new[] { accepted }, default);
        Assert.NotNull(pending);
        candidate.IntegerPaths.Clear(); candidate.Paths[0].Clear();
        accepted.IntegerPaths.Clear(); accepted.Paths[0].Clear();
        canonical.Clear(); clip.Clear();
        var prefix = PathUtils.ToClipper(new[] { Rectangle(0, 0, 3, 3) });
        var fresh = new List<List<PointD>> { Rectangle(1, 1, 2, 2) };
        var interior = new List<List<PointD>> { Rectangle(1.1, 1.1, 1.9, 1.9) };
        pending!.Complete(prefix, fresh, interior, 1, "diagnostic-test", false, default);
        prefix.Clear(); fresh[0].Clear(); interior.Clear();
        Assert.NotNull(snapshot);
        Assert.Equal(7, snapshot!.ComponentOrdinal); Assert.Equal(8, snapshot.VisitOrdinal);
        Assert.Equal(4, snapshot.Candidate.IntegerPaths[0].Length); Assert.Equal(4, snapshot.Candidate.Paths[0].Length);
        Assert.Equal(4, snapshot.AcceptedOriginalOrder[0].IntegerPaths[0].Length);
        Assert.Equal(4, snapshot.CanonicalComponent[0].Length); Assert.Equal(4, snapshot.Clip.Length);
        Assert.Equal(4, snapshot.ExactOrderedPrefix[0].Length); Assert.Equal(4, snapshot.Fresh[0].Length);
        Assert.Equal(4, snapshot.ErodedInterior[0].Length);
        var valueCopy = snapshot.Candidate.IntegerPaths[0][0]; valueCopy.X = -123;
        Assert.NotEqual(valueCopy, snapshot.Candidate.IntegerPaths[0][0]);
    }

    [Fact]
    public void EmptyContourRowsCannotEvadeTheInputOrResultMetadataBudget()
    {
        int callbacks = 0;
        using var scope = FilledFrontCoverageFallbackCapture.BeginScope(_ => callbacks++);
        var emptyRows = new Paths64(Enumerable.Range(0, FilledFrontCoverageFallbackCapture.MaximumCombinedPathRows + 1)
            .Select(_ => new Path64()));
        Assert.Null(scope.TryBegin(0, 0, 1, 0, default, Array.Empty<IReadOnlyList<PointD>>(), null,
            new(emptyRows, Array.Empty<List<PointD>>()),
            Array.Empty<FilledFrontCoverageFallbackCapture.FootprintInput>(), default));
        var pending = scope.TryBegin(0, 0, 1, 0, default, Array.Empty<IReadOnlyList<PointD>>(), null,
            Input(Rectangle(0, 0, 1, 1)), Array.Empty<FilledFrontCoverageFallbackCapture.FootprintInput>(), default);
        Assert.NotNull(pending);
        pending!.Complete(emptyRows, null, null, 1, "diagnostic-test", true, default);
        Assert.Equal(0, callbacks); Assert.Equal(0, scope.Completed); Assert.Equal(2, scope.Reservations);
    }

    [Fact]
    public async Task NestedInvocationScopesRestoreAndAsyncFlowSharesTheBoundedOwner()
    {
        Assert.Null(FilledFrontCoverageFallbackCapture.Current);
        using (var first = FilledFrontCoverageFallbackCapture.BeginScope(_ => { }))
        {
            Assert.Same(first, FilledFrontCoverageFallbackCapture.Current);
            await Task.Run(() => Assert.Same(first, FilledFrontCoverageFallbackCapture.Current));
            using (var second = FilledFrontCoverageFallbackCapture.BeginScope(_ => { }))
                Assert.Same(second, FilledFrontCoverageFallbackCapture.Current);
            Assert.Same(first, FilledFrontCoverageFallbackCapture.Current);
        }
        Assert.Null(FilledFrontCoverageFallbackCapture.Current);
    }

    [Fact]
    public void OutOfOrderScopeDisposalRestoresTheNearestStillLiveAncestor()
    {
        using var first = FilledFrontCoverageFallbackCapture.BeginScope(_ => { });
        using var second = FilledFrontCoverageFallbackCapture.BeginScope(_ => { });
        using var third = FilledFrontCoverageFallbackCapture.BeginScope(_ => { });
        second.Dispose();
        Assert.Same(third, FilledFrontCoverageFallbackCapture.Current);
        third.Dispose();
        Assert.Same(first, FilledFrontCoverageFallbackCapture.Current);
        first.Dispose();
        Assert.Null(FilledFrontCoverageFallbackCapture.Current);
    }

    [Fact]
    public void ObserverFailuresIncludingCancellationKeepLiteralSelectionAndCallerCancellation()
    {
        using var evidence = new ReferenceEvidence();
        var input = Fixture(); var expected = FilledFrontCoverage.RetainUncoveredComponents(input, 2, default);
        foreach (Exception failure in new Exception[] { new InvalidOperationException("diagnostic-only"), new OperationCanceledException() })
        {
            using var scope = FilledFrontCoverageFallbackCapture.BeginScope(_ => throw failure);
            EqualBits(expected, FilledFrontCoverage.RetainUncoveredComponents(input, 2, default));
            Assert.Equal(1, scope.Reservations); Assert.Equal(0, scope.Completed);
        }
        using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        using (var scope = FilledFrontCoverageFallbackCapture.BeginScope(_ => throw new InvalidOperationException()))
        {
            Assert.Throws<OperationCanceledException>(() =>
                FilledFrontCoverage.RetainUncoveredComponents(input, 2, cancelled.Token));
            Assert.Equal(0, scope.Reservations); Assert.Equal(0, scope.Completed);
        }
        Assert.Null(FilledFrontCoverageFallbackCapture.Current);
    }

    [Fact]
    public void ObserverCancellingCallerTokenCannotTurnAnInterruptedProofIntoASelectedResult()
    {
        using var evidence = new ReferenceEvidence();
        using var cancelled = new CancellationTokenSource();
        using var scope = FilledFrontCoverageFallbackCapture.BeginScope(_ => cancelled.Cancel());
        Assert.Throws<OperationCanceledException>(() =>
            FilledFrontCoverage.RetainUncoveredComponents(Fixture(), 2, cancelled.Token));
        Assert.Equal(1, scope.Completed);
        Assert.True(cancelled.IsCancellationRequested);
    }

    [Fact]
    public void AbsentScopeReadAllocatesNoDiagnosticWork()
    {
        Assert.Null(FilledFrontCoverageFallbackCapture.Current); // Warm the getter.
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 1024; i++)
            if (FilledFrontCoverageFallbackCapture.Current is not null)
                throw new InvalidOperationException("A diagnostic scope leaked into the absent path.");
        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }

    [Fact]
    public void ObserverCannotClaimACacheHitAsANewFallbackCapture()
    {
        string? previous = Environment.GetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE");
        try
        {
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE", null);
            FilledFrontCoverage.ClearEvidenceCache(); PolygonOps.ClearCenterlineEvidenceCache();
            var input = Fixture(); var expected = FilledFrontCoverage.RetainUncoveredComponents(input, 2, default);
            using var scope = FilledFrontCoverageFallbackCapture.BeginScope(_ => throw new InvalidOperationException());
            EqualBits(expected, FilledFrontCoverage.RetainUncoveredComponents(input, 2, default));
            Assert.Equal(0, scope.Reservations); Assert.Equal(0, scope.Completed);
        }
        finally
        {
            FilledFrontCoverage.ClearEvidenceCache(); PolygonOps.ClearCenterlineEvidenceCache();
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE", previous);
        }
    }

    private static List<List<PointD>> Fixture() => new()
    { Rectangle(0, 0, 10, 10), Rectangle(40, 0, 44, 4), Rectangle(60, 0, 63, 3), Rectangle(10.01, 4, 10.011, 6) };
    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
    private static FilledFrontCoverageFallbackCapture.FootprintInput Input(List<PointD> path)
    { var paths = new List<List<PointD>> { path }; return new(PathUtils.ToClipper(paths), paths); }
    private static Paths64 Integer(ImmutableArray<ImmutableArray<Point64>> paths)
        => new(paths.Select(path => new Path64(path)));
    private static List<List<PointD>> World(ImmutableArray<ImmutableArray<PointD>> paths)
        => paths.Select(path => path.ToList()).ToList();
    private static void EqualInteger(Paths64 expected, ImmutableArray<ImmutableArray<Point64>> actual)
    {
        Assert.Equal(expected.Count, actual.Length);
        for (int i = 0; i < expected.Count; i++) Assert.Equal(expected[i].ToArray(), actual[i].ToArray());
    }
    private static void EqualBits(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
            Assert.Equal(expected[i].Select(point => (BitConverter.DoubleToInt64Bits(point.x), BitConverter.DoubleToInt64Bits(point.y))),
                actual[i].Select(point => (BitConverter.DoubleToInt64Bits(point.x), BitConverter.DoubleToInt64Bits(point.y))));
    }
    private sealed class ReferenceEvidence : IDisposable
    {
        private readonly string? _previous = Environment.GetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE");
        internal ReferenceEvidence() => Environment.SetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE", "1");
        public void Dispose() => Environment.SetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE", _previous);
    }
}
