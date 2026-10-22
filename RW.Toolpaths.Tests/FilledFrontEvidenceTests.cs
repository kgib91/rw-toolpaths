using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Filled front exact evidence controls", DisableParallelization = true)]
public sealed class FilledFrontEvidenceCollection { }

[Collection("Filled front exact evidence controls")]
public sealed class FilledFrontEvidenceTests
{
    [Fact]
    public void ExactCanonicalRetainedFrontReusesProofAndReturnsIndependentCoordinates()
    {
        using var scope = new EvidenceScope();
        var input = NestedFronts();
        var expected = Reference(input, .7);
        var first = FilledFrontCoverage.RetainUncoveredComponents(input, .7, CancellationToken.None);
        Equal(expected, first);
        FilledFrontCoverage.Statistics statistics = default;
        var second = FilledFrontCoverage.RetainUncoveredComponents(first, .7, CancellationToken.None,
            observe: value => statistics = value);
        Equal(expected, second);
        Assert.True(statistics.EvidenceReused);
        Assert.Equal(0, statistics.OrderedUnionCalls);
        Assert.Equal(0, statistics.IndexBuilds);
        Assert.Equal(1, FilledFrontCoverage.InspectEvidenceCache().Results.Hits);
        second[0][0] = new PointD(12345, 12345);
        Equal(expected, FilledFrontCoverage.RetainUncoveredComponents(input, .7, CancellationToken.None));
    }

    [Fact]
    public void RadiusClipHoleWindingAndCoordinateMutationsRequireTheirOwnProof()
    {
        using var scope = new EvidenceScope();
        var input = NestedFronts();
        var clip = Rectangle(-10, -10, 70, 50);
        Compare(input, .7, clip);
        long publications = FilledFrontCoverage.InspectEvidenceCache().Results.Publications;
        Compare(input, .9, clip);
        Assert.True(FilledFrontCoverage.InspectEvidenceCache().Results.Publications > publications);
        publications = FilledFrontCoverage.InspectEvidenceCache().Results.Publications;
        clip[0] = new PointD(-9, -10);
        Compare(input, .9, clip);
        Assert.True(FilledFrontCoverage.InspectEvidenceCache().Results.Publications > publications);
        publications = FilledFrontCoverage.InspectEvidenceCache().Results.Publications;
        input[1].Reverse(); // Filled hole becomes positive; the native signed authority changes.
        Compare(input, .9, clip);
        Assert.True(FilledFrontCoverage.InspectEvidenceCache().Results.Publications > publications);
        publications = FilledFrontCoverage.InspectEvidenceCache().Results.Publications;
        input[^1][0] = new PointD(input[^1][0].x + 1 / PathUtils.Scale, input[^1][0].y);
        Compare(input, .9, clip);
        Assert.True(FilledFrontCoverage.InspectEvidenceCache().Results.Publications > publications);
    }

    [Fact]
    public void UnchangedComponentsReuseOnlyTheirOwnCompleteFootprints()
    {
        using var scope = new EvidenceScope();
        var input = new List<List<PointD>> { Rectangle(0, 0, 4, 4), Rectangle(10, 0, 14, 4), Rectangle(20, 0, 24, 4) };
        Compare(input, .7, null);
        long hits = FilledFrontCoverage.InspectEvidenceCache().Footprints.Hits;
        input[^1][1] = new PointD(24.5, 0);
        Compare(input, .7, null);
        Assert.True(FilledFrontCoverage.InspectEvidenceCache().Footprints.Hits >= hits + 2);
    }

    [Fact]
    public void WitnessAndContainmentModesHaveSeparateCompleteResults()
    {
        using var scope = new EvidenceScope();
        var input = new List<List<PointD>> { Rectangle(0, 0, 10, 10), Rectangle(10.01, 4, 10.011, 6) };
        foreach (bool witnesses in new[] { true, false })
        foreach (bool containment in new[] { true, false })
        {
            var expected = Reference(input, 2, null, witnesses, containment);
            Equal(expected, FilledFrontCoverage.RetainUncoveredComponents(input, 2, CancellationToken.None,
                useFreshWitness: witnesses, usePreparedContainment: containment));
        }
        Assert.Equal(4, FilledFrontCoverage.InspectEvidenceCache().Results.Publications);
    }

    [Fact]
    public void WarmCancellationAndCanceledFinalObservationDoNotPublishRetainedProofs()
    {
        using var scope = new EvidenceScope();
        var input = NestedFronts();
        FilledFrontCoverage.RetainUncoveredComponents(input, .7, CancellationToken.None);
        var state = FilledFrontCoverage.InspectEvidenceCache().Results;
        using var canceled = new CancellationTokenSource(); canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() =>
            FilledFrontCoverage.RetainUncoveredComponents(input, .7, canceled.Token));
        Assert.Equal(state, FilledFrontCoverage.InspectEvidenceCache().Results);
        FilledFrontCoverage.ClearEvidenceCache();
        using var during = new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(() => FilledFrontCoverage.RetainUncoveredComponents(input,
            .7, during.Token, observe: _ => during.Cancel()));
        Assert.Equal(0, FilledFrontCoverage.InspectEvidenceCache().Results.Publications);
        // Independently completed component buffers can survive cancellation of
        // the later whole-front predicate; no unfinished retained proof survives.
    }

    [Fact]
    public async Task ConcurrentReadersPreserveExactReferenceOrderingAndOwnedResults()
    {
        using var scope = new EvidenceScope();
        var input = NestedFronts();
        var expected = Reference(input, .7);
        var results = await Task.WhenAll(Enumerable.Range(0, 8).Select(_ => Task.Run(() =>
            FilledFrontCoverage.RetainUncoveredComponents(input, .7, CancellationToken.None))));
        foreach (var actual in results) Equal(expected, actual);
        results[0][0][0] = new PointD(-12345, -12345);
        foreach (var actual in results.Skip(1)) Equal(expected, actual);
        Assert.InRange(FilledFrontCoverage.InspectEvidenceCache().Results.Bytes, 1, 16L << 20);
    }

    [Fact]
    public void JoinedBufferEvidenceRetainsExactOrderArcRadiusAndMutationOwnership()
    {
        using var scope = new EvidenceScope();
        var input = NestedFronts();
        var expected = PolygonOps.BufferCenterlines(input, .7, .001);
        var first = PolygonOps.BufferCenterlinesExact(input, .7, .001, CancellationToken.None);
        Equal(expected, first);
        var second = PolygonOps.BufferCenterlinesExact(input, .7, .001, CancellationToken.None);
        Equal(expected, second);
        Assert.Equal(1, PolygonOps.InspectCenterlineEvidenceCache().Hits);
        first[0][0] = new PointD(12345, 12345);
        Equal(expected, PolygonOps.BufferCenterlinesExact(input, .7, .001, CancellationToken.None));
        foreach (var settings in new[] { (Radius: .9, Arc: .001), (Radius: .7, Arc: .0005) })
            Equal(PolygonOps.BufferCenterlines(input, settings.Radius, settings.Arc),
                PolygonOps.BufferCenterlinesExact(input, settings.Radius, settings.Arc, CancellationToken.None));
        input.Reverse();
        Equal(PolygonOps.BufferCenterlines(input, .7, .001),
            PolygonOps.BufferCenterlinesExact(input, .7, .001, CancellationToken.None));
        Assert.Equal(4, PolygonOps.InspectCenterlineEvidenceCache().Publications);
        using var canceled = new CancellationTokenSource(); canceled.Cancel();
        var state = PolygonOps.InspectCenterlineEvidenceCache();
        Assert.Throws<OperationCanceledException>(() => PolygonOps.BufferCenterlinesExact(input, .7, .001, canceled.Token));
        Assert.Equal(state, PolygonOps.InspectCenterlineEvidenceCache());
    }

    [Fact]
    public void ExactEffectKeysIncludeEveryOrderedBitAndCacheBudgetsEvictWithoutChangingValues()
    {
        using var scope = new EvidenceScope();
        var ring = Rectangle(0, 0, 1, 1);
        var key = Key(ring, 1, null);
        Assert.True(key.Same(Key(ring.Select(point => new PointD(point.x, point.y)).ToList(), 1, null)));
        ring[0] = new PointD(double.Epsilon, 0);
        Assert.False(key.Same(Key(ring, 1, null))); // Exact bits, even below the integer grid.
        Assert.False(key.Same(Key(Rectangle(0, 0, 1, 1), Math.BitIncrement(1d), null)));
        Assert.False(key.Same(Key(Rectangle(0, 0, 1, 1), 1, Array.Empty<PointD>())));
        var cache = new FilledFrontEvidenceCache.Cache<string>(1000, 2);
        var keys = Enumerable.Range(0, 3).Select(index => Key(Rectangle(index, 0, index + 1, 1), 1, null)).ToArray();
        for (int index = 0; index < 3; index++) cache.Publish(keys[index], index.ToString(), 100, CancellationToken.None);
        Assert.Null(cache.Find(keys[0], CancellationToken.None));
        Assert.Equal("2", cache.Find(keys[2], CancellationToken.None));
        Assert.Equal(1, cache.Inspect().Evictions);
        cache.Publish(key, "oversized", 1000, CancellationToken.None);
        Assert.Equal(1, cache.Inspect().Oversized);
        Assert.InRange(cache.Inspect().Bytes, 0, 1000);
        using var canceled = new CancellationTokenSource(); canceled.Cancel();
        var before = cache.Inspect();
        Assert.Throws<OperationCanceledException>(() => cache.Publish(key, "canceled", 100, canceled.Token));
        Assert.Equal(before, cache.Inspect());
    }

    private static FilledFrontEvidenceCache.Key Key(IReadOnlyList<PointD> path, double radius,
        IReadOnlyList<PointD>? clip) => FilledFrontEvidenceCache.Key.Capture(new[] { path }, radius,
            .001, .003 + 4 / PathUtils.Scale, 3, clip, CancellationToken.None)!;

    private static void Compare(List<List<PointD>> input, double radius, IReadOnlyList<PointD>? clip)
        => Equal(Reference(input, radius, clip), FilledFrontCoverage.RetainUncoveredComponents(input,
            radius, CancellationToken.None, clip));

    private static List<List<PointD>> Reference(List<List<PointD>> input, double radius,
        IReadOnlyList<PointD>? clip = null, bool witnesses = true, bool containment = true)
    {
        string? previous = Environment.GetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE");
        try
        {
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE", "1");
            return FilledFrontCoverage.RetainUncoveredComponents(input, radius, CancellationToken.None,
                clip, witnesses, usePreparedContainment: containment);
        }
        finally { Environment.SetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE", previous); }
    }

    private static List<List<PointD>> NestedFronts()
    {
        var hole = Rectangle(3, 3, 17, 17); hole.Reverse();
        return new() { Rectangle(0, 0, 20, 20), hole, Rectangle(7, 7, 13, 13),
            Rectangle(28, 0, 32, 4), Rectangle(38, 0, 42, 4) };
    }

    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };

    private static void Equal(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int ring = 0; ring < expected.Count; ring++)
        {
            Assert.Equal(expected[ring].Count, actual[ring].Count);
            for (int point = 0; point < expected[ring].Count; point++)
            {
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected[ring][point].x), BitConverter.DoubleToInt64Bits(actual[ring][point].x));
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected[ring][point].y), BitConverter.DoubleToInt64Bits(actual[ring][point].y));
            }
        }
    }

    private sealed class EvidenceScope : IDisposable
    {
        private readonly string? _previous = Environment.GetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE");
        internal EvidenceScope()
        {
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE", null);
            FilledFrontCoverage.ClearEvidenceCache(); PolygonOps.ClearCenterlineEvidenceCache();
        }
        public void Dispose()
        {
            FilledFrontCoverage.ClearEvidenceCache(); PolygonOps.ClearCenterlineEvidenceCache();
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE", _previous);
        }
    }
}
