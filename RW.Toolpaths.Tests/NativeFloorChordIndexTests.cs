using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;
using Stock = RW.Toolpaths.Milling.PocketContinuationRouter.ClearedStock;

namespace RW.Toolpaths.Tests;

public sealed class NativeFloorChordIndexTests
{
    private readonly ITestOutputHelper _output;
    public NativeFloorChordIndexTests(ITestOutputHelper output) => _output = output;

    [Fact]
    public void FloorSpecificRampPiecesKeepEveryOriginalCoordinateAndOrder()
    {
        var (reference, actual) = Stocks();
        RecordBoth(reference, actual, new Point3D[]
        {
            new(-4, 0, 0), new(-2, 0, -2), new(0, 0, 0), new(2, 0, -2), new(4, 0, 0),
        });
        RecordBoth(reference, actual, new Point3D[]
        {
            new(-3, 1, -3), new(-1, 1, 1), new(1, 1, -3), new(3, 1, 1),
        });
        foreach (double floor in new[] { 0d, -1d, -2d, -1d + 5e-10, -1d - 5e-10 })
        {
            WarmToCrossover(actual, floor, new(0, 0, floor), 100);
            AssertChords(reference.NativeFloorChords(floor, new(0, 0, floor), 100, 8192),
                actual.NativeFloorChords(floor, new(0, 0, floor), 100, 8192));
            long misses = actual.FloorProjectionCacheMisses;
            AssertChords(reference.NativeFloorChords(floor, new(1, 1, floor), 100, 8192),
                actual.NativeFloorChords(floor, new(1, 1, floor), 100, 8192));
            Assert.Equal(misses, actual.FloorProjectionCacheMisses);
        }
    }

    [Fact]
    public void CandidateQueriesDoNotConsumeSweepsOrChangeCapturedFlushes()
    {
        var (reference, actual) = Stocks();
        RecordBoth(reference, actual, new Point3D[] { new(0, 0, -2), new(2, 0, -2), new(2, 2, -2) });
        AssertChords(reference.NativeFloorChords(-1, new(0, 0, -1), 100, 8192),
            actual.NativeFloorChords(-1, new(0, 0, -1), 100, 8192));
        Assert.Equal(0, actual.PreparedBatches);
        Assert.Equal(0, actual.ReplayedStrokes);
        Assert.Equal(1L, actual.DeferredFloorStrokes);
        var oldReference = reference.PrepareAtFloor(-1);
        var oldActual = actual.PrepareAtFloor(-1);
        Assert.Equal(1, oldActual.PrefixBatches);
        Assert.Equal(reference.PreparedBatches, actual.PreparedBatches);

        RecordBoth(reference, actual, new Point3D[] { new(5, 0, -2), new(7, 0, -2) });
        AssertChords(reference.NativeFloorChords(-1, new(0, 0, -1), 100, 8192),
            actual.NativeFloorChords(-1, new(0, 0, -1), 100, 8192));
        Assert.Equal(1, actual.PreparedBatches);
        Assert.Equal(1L, actual.ReplayedStrokes);
        Assert.Equal(1L, actual.DeferredFloorStrokes);
        var newReference = reference.PrepareAtFloor(-1);
        var newActual = actual.PrepareAtFloor(-1);
        Assert.Equal(2, newActual.PrefixBatches);
        AssertPaths(newReference.Materialize(), newActual.Materialize());
        AssertPaths(oldReference.Materialize(), oldActual.Materialize());
        Assert.Equal(reference.FloorUnionCalls, actual.FloorUnionCalls);
        Assert.Equal(reference.ReplayedStrokes, actual.ReplayedStrokes);
    }

    [Fact]
    public void PreparingTheFloorReusesCandidateProjectionWithoutCombiningFlushes()
    {
        var (reference, actual) = Stocks();
        RecordBoth(reference, actual, new Point3D[] { new(0, 0, 0), new(2, 0, -2), new(4, 0, -2) });
        reference.PrepareAtFloor(-1);
        actual.PrepareAtFloor(-1);
        Assert.Equal(1L, actual.FloorProjectionCacheMisses);
        AssertChords(reference.NativeFloorChords(-1, new(0, 0, -1), 100, 8192),
            actual.NativeFloorChords(-1, new(0, 0, -1), 100, 8192));
        Assert.Equal(1L, actual.FloorProjectionCacheMisses);
        Assert.Equal(1L, actual.NativeChordProjectionCacheHits);
        Assert.Equal(0L, actual.NativeChordProjectionCacheMisses);
        Assert.Equal(1, actual.PreparedBatches);
        AssertPaths(reference.AtFloor(-1), actual.AtFloor(-1));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EnumerationCapturesExecutedStrokesOnFirstMoveNext(bool indexed)
    {
        var stock = new Stock(1, CancellationToken.None, useNativeChordIndex: indexed);
        stock.Record(new Point3D[] { new(0, 0, -2), new(1, 0, -2), new(2, 0, -2) });
        if (indexed)
        {
            WarmToCrossover(stock, -1, new(0, 0, -1), 100);
            _ = stock.NativeFloorChords(-1, new(0, 0, -1), 100, 8192).Count();
        }
        var pending = stock.NativeFloorChords(-1, new(0, 0, -1), 100, 8192);
        stock.Record(new Point3D[] { new(3, 0, -2), new(4, 0, -2) });
        var expected = stock.NativeFloorChords(-1, new(0, 0, -1), 100, 8192).ToArray();
        using var enumerator = pending.GetEnumerator();
        Assert.True(enumerator.MoveNext());
        var actual = new List<(Point3D A, Point3D B)> { enumerator.Current };
        stock.Record(new Point3D[] { new(5, 0, -2), new(6, 0, -2) });
        while (enumerator.MoveNext()) actual.Add(enumerator.Current);
        AssertChords(expected, actual);
        Assert.Equal(expected.Length + 1, stock.NativeFloorChords(-1, new(0, 0, -1), 100, 8192).Count());
    }

    [Fact]
    public void LimitIsAppliedAfterExactEndpointFilteringInOriginalStrokeOrder()
    {
        var (reference, actual) = Stocks();
        for (int stroke = 0; stroke < 8500; stroke++)
            RecordBoth(reference, actual, new Point3D[] { new(stroke * .0001, 0, -2), new(stroke * .0001, .1, -2) });
        for (int stroke = 0; stroke < 1024; stroke++)
            RecordBoth(reference, actual, new Point3D[] { new(1000 + stroke, 0, -2), new(1000 + stroke, 1, -2) });
        WarmToCrossover(actual, -1, new(0, 0, -1), 2);
        var expected = reference.NativeFloorChords(-1, new(0, 0, -1), 2, 8192).ToArray();
        var indexed = actual.NativeFloorChords(-1, new(0, 0, -1), 2, 8192).ToArray();
        Assert.Equal(8192, indexed.Length);
        Assert.Equal(8499 * .0001, indexed[0].A.X);
        AssertChords(expected, indexed);
    }

    [Fact]
    public void ALongStrokeKeepsAscendingChordOrderAcrossSpatialChunks()
    {
        var (reference, actual) = Stocks();
        RecordBoth(reference, actual, Enumerable.Range(0, 1301).Select(point => new Point3D(point, 0, -2)).ToArray());
        RecordBoth(reference, actual, new Point3D[] { new(-4, 0, -2), new(-3, 0, -2) });
        WarmToCrossover(actual, -1, new(0, 0, -1), 10000);
        var expected = reference.NativeFloorChords(-1, new(0, 0, -1), 10000, 8192).ToArray();
        var indexed = actual.NativeFloorChords(-1, new(0, 0, -1), 10000, 8192).ToArray();
        Assert.Equal(1301, indexed.Length);
        Assert.Equal(-4, indexed[0].A.X);
        Assert.Equal(0, indexed[1].A.X);
        AssertChords(expected, indexed);
    }

    [Fact]
    public void EarlyCapDoesNotNominateOlderStrokesOrLaterLongStrokeChunks()
    {
        var (reference, actual) = Stocks();
        for (int stroke = 0; stroke < 3; stroke++)
            RecordBoth(reference, actual, Enumerable.Range(0, 1301)
                .Select(point => new Point3D(stroke * 2000 + point, 0, -2)).ToArray());
        WarmToCrossover(actual, -1, new(0, 0, -1), 10000);
        long nominated = actual.NativeChordNominated, examined = actual.NativeChordExamined;
        var expected = reference.NativeFloorChords(-1, new(0, 0, -1), 10000, 33).ToArray();
        var indexed = actual.NativeFloorChords(-1, new(0, 0, -1), 10000, 33).ToArray();
        Assert.Equal(33, indexed.Length);
        AssertChords(expected, indexed);
        Assert.InRange(actual.NativeChordNominated - nominated, 34, 512);
        Assert.InRange(actual.NativeChordExamined - examined, 34, 512);
        Assert.Equal(1L, actual.NativeChordIndexedQueries);
    }

    [Fact]
    public void RepeatedEarlyCapsDoNotPopulateOrIndexUntouchedHistoricalStrokes()
    {
        var stock = new Stock(1, CancellationToken.None);
        for (int stroke = 0; stroke < 4096; stroke++)
            stock.Record(Enumerable.Range(0, 9).Select(point => new Point3D(stroke * 100d + point, 0, -2)).ToArray());
        for (int query = 0; query < 32; query++)
            Assert.Equal(33, stock.NativeFloorChords(-1, new(0, 0, -1), 1e9, 33).Count());

        Assert.Equal(5L, stock.FloorProjectionCacheMisses);
        Assert.Equal(32L, stock.NativeChordCachedScanQueries);
        Assert.Equal(32L, stock.NativeChordIndexDeferredQueries);
        Assert.Equal(0L, stock.NativeChordIndexedQueries);
        Assert.Equal(0L, stock.NativeChordIndexBuilds);
        Assert.Equal(0L, stock.ReplayedStrokes);
        Assert.Equal(0, stock.PreparedBatches);
    }

    [Theory]
    [InlineData(256)]
    [InlineData(2048)]
    [InlineData(16384)]
    public void RepeatedLocalQueriesExamineAFractionOfIrrelevantHistoricalChords(int strokes)
    {
        var (reference, actual) = Stocks();
        for (int stroke = 0; stroke < strokes; stroke++)
            RecordBoth(reference, actual, Enumerable.Range(0, 9)
                .Select(point => new Point3D(stroke * 100d + point, 0, -3)).ToArray());
        var first = new Point3D(strokes / 2 * 100d + 4, 0, -2);
        for (int query = 0; query < 5; query++)
            AssertChords(reference.NativeFloorChords(-2, first, 3, 8192),
                actual.NativeFloorChords(-2, first, 3, 8192));
        Assert.Equal(5L, actual.NativeChordCachedScanQueries);
        Assert.Equal(5L, actual.NativeChordIndexDeferredQueries);
        Assert.Equal(0L, actual.NativeChordIndexedQueries);
        long misses = actual.FloorProjectionCacheMisses;
        long builds = actual.NativeChordIndexBuilds;
        long examined = actual.NativeChordExamined;
        long referenceExamined = reference.NativeChordExamined;
        var second = new Point3D((strokes - 1) * 100d + 4, 0, -2);
        AssertChords(reference.NativeFloorChords(-2, second, 3, 8192),
            actual.NativeFloorChords(-2, second, 3, 8192));

        long localWork = actual.NativeChordExamined - examined;
        long originalWork = reference.NativeChordExamined - referenceExamined;
        _output.WriteLine($"strokes={strokes} originalExamined={originalWork} indexedExamined={localWork} "
            + $"indexBuilds={actual.NativeChordIndexBuilds} projectionMisses={misses} nodeVisits={actual.NativeChordIndexNodeVisits}");
        Assert.Equal((long)strokes, misses);
        Assert.Equal(misses, actual.FloorProjectionCacheMisses);
        Assert.Equal(0L, builds);
        Assert.True(actual.NativeChordIndexBuilds > 0);
        Assert.Equal(1L, actual.NativeChordIndexedQueries);
        Assert.Equal((long)strokes * 8, originalWork);
        Assert.InRange(localWork, 1, originalWork / 8);
        builds = actual.NativeChordIndexBuilds;
        AssertChords(reference.NativeFloorChords(-2, first, 3, 8192),
            actual.NativeFloorChords(-2, first, 3, 8192));
        Assert.Equal(builds, actual.NativeChordIndexBuilds);
        Assert.Equal(misses, actual.FloorProjectionCacheMisses);
        Assert.Equal(0, actual.PreparedBatches);
        Assert.Equal(0L, actual.ReplayedStrokes);
    }

    [Fact]
    public void CrossingSegmentWithBothEndpointsOutsideTheBudgetStillRefuses()
    {
        var (reference, actual) = Stocks();
        RecordBoth(reference, actual, new Point3D[] { new(-10, 0, -2), new(10, 0, -2) });
        WarmToCrossover(actual, -1, new(0, 0, -1), 1);
        Assert.Empty(actual.NativeFloorChords(-1, new(0, 0, -1), 1, 8192));
        AssertChords(reference.NativeFloorChords(-1, new(0, 0, -1), 1, 8192),
            actual.NativeFloorChords(-1, new(0, 0, -1), 1, 8192));
    }

    [Fact]
    public void OutwardCandidateBoundsKeepExactAndAdjacentRadiusTangencies()
    {
        var (reference, actual) = Stocks();
        foreach (double origin in new[] { 0d, 1e16, -1e16 })
        foreach (double offset in new[] { Math.BitDecrement(4d), 4d, Math.BitIncrement(4d) })
            RecordBoth(reference, actual, new Point3D[] { new(origin + offset, 0, -2), new(origin + 100, 0, -2) });
        WarmToCrossover(actual, -1, new(0, 0, -1), 4);
        foreach (double origin in new[] { 0d, 1e16, -1e16 })
            AssertChords(reference.NativeFloorChords(-1, new(origin, 0, -1), 4, 8192),
                actual.NativeFloorChords(-1, new(origin, 0, -1), 4, 8192));
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.MaxValue)]
    [InlineData(-4d)]
    [InlineData(0d)]
    [InlineData(1e-300)]
    public void ExceptionalAndNegativeBudgetsKeepTheLiteralEndpointPredicate(double budget)
    {
        var (reference, actual) = Stocks();
        RecordBoth(reference, actual, new Point3D[] { new(0, 0, -2), new(1e-200, 0, -2), new(4, 0, -2), new(100, 0, -2) });
        AssertChords(reference.NativeFloorChords(-1, new(0, 0, -1), budget, 8192),
            actual.NativeFloorChords(-1, new(0, 0, -1), budget, 8192));
    }

    [Theory]
    [InlineData(1e-161)]
    [InlineData(-1e-161)]
    public void PositiveSubnormalSquaredBudgetsKeepRoundedDistanceDecisions(double budget)
    {
        var (reference, actual) = Stocks();
        RecordBoth(reference, actual, new Point3D[]
        {
            new(0, 0, -2), new(5e-162, 0, -2), new(1e-161, 0, -2), new(2e-161, 0, -2),
        });
        Assert.True(budget * budget > 0);
        WarmToCrossover(actual, -1, new(0, 0, -1), budget);
        AssertChords(reference.NativeFloorChords(-1, new(0, 0, -1), budget, 8192),
            actual.NativeFloorChords(-1, new(0, 0, -1), budget, 8192));
    }

    [Fact]
    public void CoordinateAdditionCancellationCannotPruneAnAcceptedEndpoint()
    {
        var (reference, actual) = Stocks();
        const double origin = 1e16;
        RecordBoth(reference, actual, new Point3D[]
        {
            new(origin + 1, 0, -2), new(origin + 2, 0, -2), new(origin - 1, 0, -2), new(origin - 2, 0, -2),
        });
        WarmToCrossover(actual, -1, new(origin, 0, -1), 1);
        AssertChords(reference.NativeFloorChords(-1, new(origin, 0, -1), 1, 8192),
            actual.NativeFloorChords(-1, new(origin, 0, -1), 1, 8192));
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void NonFiniteFloorKeepsOriginalProjectionComparisons(double floor)
    {
        var (reference, actual) = Stocks();
        RecordBoth(reference, actual, new Point3D[] { new(0, 0, 0), new(1, 0, -2), new(2, 0, 0) });
        AssertChords(reference.NativeFloorChords(floor, new(0, 0, floor), 100, 8192),
            actual.NativeFloorChords(floor, new(0, 0, floor), 100, 8192));
        Assert.Equal(0L, actual.NativeChordIndexBuilds);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void NonFiniteQueryOriginKeepsOriginalDistanceComparisons(double x)
    {
        var (reference, actual) = Stocks();
        RecordBoth(reference, actual, new Point3D[] { new(0, 0, -2), new(1, 0, -2), new(2, 0, -2) });
        AssertChords(reference.NativeFloorChords(-1, new(x, 0, -1), 100, 8192),
            actual.NativeFloorChords(-1, new(x, 0, -1), 100, 8192));
        Assert.Equal(0L, actual.NativeChordIndexBuilds);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void NonFiniteProjectedCoordinateFallsBackWithoutPruning(double x)
    {
        var (reference, actual) = Stocks();
        RecordBoth(reference, actual, new Point3D[] { new(x, 0, -2), new(0, 0, -2), new(1, 0, -2) });
        AssertChords(reference.NativeFloorChords(-1, new(0, 0, -1), 100, 8192),
            actual.NativeFloorChords(-1, new(0, 0, -1), 100, 8192));
        Assert.Equal(0L, actual.NativeChordIndexBuilds);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CancellationAfterTheFirstYieldStillPropagates(bool indexed)
    {
        using var cancellation = new CancellationTokenSource();
        var stock = new Stock(1, cancellation.Token, useNativeChordIndex: indexed);
        stock.Record(new Point3D[] { new(0, 0, -2), new(1, 0, -2), new(2, 0, -2) });
        if (indexed)
        {
            WarmToCrossover(stock, -1, new(0, 0, -1), 100);
            _ = stock.NativeFloorChords(-1, new(0, 0, -1), 100, 8192).Count();
        }
        using var enumerator = stock.NativeFloorChords(-1, new(0, 0, -1), 100, 8192).GetEnumerator();
        Assert.True(enumerator.MoveNext());
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => { enumerator.MoveNext(); });
    }

    private static (Stock Reference, Stock Actual) Stocks()
        => (new(1, CancellationToken.None, useNativeChordIndex: false), new(1, CancellationToken.None));

    private static void RecordBoth(Stock reference, Stock actual, IReadOnlyList<Point3D> stroke)
    {
        reference.Record(stroke);
        actual.Record(stroke);
    }

    private static void WarmToCrossover(Stock stock, double floor, Point3D from, double budget)
    {
        // Each uncapped enumeration accounts for the actual full historical
        // scan. The strict four-domain crossover becomes eligible afterward.
        for (int query = 0; query < 5; query++)
            _ = stock.NativeFloorChords(floor, from, budget, int.MaxValue).Count();
    }

    private static void AssertChords(IEnumerable<(Point3D A, Point3D B)> reference,
        IEnumerable<(Point3D A, Point3D B)> actual)
    {
        static long[] Encode(IEnumerable<(Point3D A, Point3D B)> chords)
            => chords.SelectMany(chord => new[] { chord.A.X, chord.A.Y, chord.A.Z, chord.B.X, chord.B.Y, chord.B.Z })
                .Select(BitConverter.DoubleToInt64Bits).ToArray();
        Assert.Equal(Encode(reference), Encode(actual));
    }

    private static void AssertPaths(IReadOnlyList<List<PointD>> reference, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(reference.Count, actual.Count);
        for (int path = 0; path < reference.Count; path++)
        {
            Assert.Equal(reference[path].Count, actual[path].Count);
            for (int point = 0; point < reference[path].Count; point++)
            {
                Assert.Equal(BitConverter.DoubleToInt64Bits(reference[path][point].x), BitConverter.DoubleToInt64Bits(actual[path][point].x));
                Assert.Equal(BitConverter.DoubleToInt64Bits(reference[path][point].y), BitConverter.DoubleToInt64Bits(actual[path][point].y));
            }
        }
    }
}
