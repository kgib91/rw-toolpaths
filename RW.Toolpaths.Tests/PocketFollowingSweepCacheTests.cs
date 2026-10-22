using System.Collections;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketFollowingSweepCacheTests
{
    [Fact]
    public void EqualQuantizedOrderedGeometryReusesTheExactOriginalSweep()
    {
        var cache = new PocketContinuationRouter.FollowingSweepCache();
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        IReadOnlyList<PointD>[] original = [new PointD[] { new(0, 0), new(10, 0), new(10, 2) }];
        double drift = .1 / PathUtils.Scale;
        IReadOnlyList<PointD>[] clone = [original[0].Select(p => new PointD(p.x + drift, p.y - drift)).ToArray()];
        var first = cache.Get(original, .5, .001, CancellationToken.None, diagnostics);
        var second = cache.Get(clone, .5, .001, CancellationToken.None, diagnostics);
        Assert.Same(first, second);
        EqualPolygons(PolygonOps.BufferOpenCenterlines(original, .5, .001), second.Paths);
        Assert.Equal(1, diagnostics.FollowingCacheMisses); Assert.Equal(1, diagnostics.FollowingCacheHits);
    }

    [Fact]
    public void HashCollisionsAndDifferentPolylineBoundariesCannotInventAConnectingCut()
    {
        var cache = new PocketContinuationRouter.FollowingSweepCache(new CollidingComparer());
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        IReadOnlyList<PointD>[] split = [new PointD[] { new(0, 0), new(2, 0) }, new PointD[] { new(10, 0), new(12, 0) }];
        IReadOnlyList<PointD>[] joined = [split.SelectMany(p => p).ToArray()];
        var splitResult = cache.Get(split, .5, .001, CancellationToken.None, diagnostics);
        var joinedResult = cache.Get(joined, .5, .001, CancellationToken.None, diagnostics);
        Assert.False(new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(splitResult.Paths))
            .IsInside(new Point64(6 * (long)PathUtils.Scale, 0)));
        Assert.True(new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(joinedResult.Paths))
            .IsInside(new Point64(6 * (long)PathUtils.Scale, 0)));
        Assert.Same(splitResult, cache.Get(split, .5, .001, CancellationToken.None, diagnostics));
        Assert.Same(joinedResult, cache.Get(joined, .5, .001, CancellationToken.None, diagnostics));
        EqualPolygons(PolygonOps.BufferOpenCenterlines(split, .5, .001), splitResult.Paths);
        EqualPolygons(PolygonOps.BufferOpenCenterlines(joined, .5, .001), joinedResult.Paths);
        Assert.Equal(2, diagnostics.FollowingCacheMisses); Assert.Equal(2, diagnostics.FollowingCacheHits);
    }

    [Fact]
    public void RadiusArcPolicyAndInputMutationRetainDistinctImmutableEntries()
    {
        var cache = new PocketContinuationRouter.FollowingSweepCache();
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var source = new List<PointD> { new(-.5 / PathUtils.Scale, .5 / PathUtils.Scale), new(10, 0), new(10, 4) };
        var saved = source.ToArray();
        var first = cache.Get(new[] { source }, .5, .001, CancellationToken.None, diagnostics);
        cache.Get(new[] { source }, .6, .001, CancellationToken.None, diagnostics);
        cache.Get(new[] { source }, .5, .002, CancellationToken.None, diagnostics);
        source[1] = new(10, 10);
        var changed = cache.Get(new[] { source }, .5, .001, CancellationToken.None, diagnostics);
        Assert.NotSame(first, changed);
        Assert.Same(first, cache.Get(new[] { saved }, .5, .001, CancellationToken.None, diagnostics));
        EqualPolygons(PolygonOps.BufferOpenCenterlines(new[] { saved }, .5, .001), first.Paths);
        EqualPolygons(PolygonOps.BufferOpenCenterlines(new[] { source }, .5, .001), changed.Paths);
        Assert.Equal(4, diagnostics.FollowingCacheMisses); Assert.Equal(1, diagnostics.FollowingCacheHits);
    }

    [Fact]
    public void ASkippedSingletonStillPartitionsThe8192PointConstructionSwitch()
    {
        var cache = new PocketContinuationRouter.FollowingSweepCache();
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var trace = Enumerable.Range(0, 8192).Select(i => new PointD(i * .001, 0)).ToArray();
        IReadOnlyList<PointD>[] before = [trace];
        IReadOnlyList<PointD>[] after = [trace, new PointD[] { new(-10, -10) }];
        var direct = cache.Get(before, .25, .001, CancellationToken.None, diagnostics);
        var chunked = cache.Get(after, .25, .001, CancellationToken.None, diagnostics);
        EqualPolygons(PolygonOps.BufferOpenCenterlines(before, .25, .001), direct.Paths);
        EqualPolygons(PolygonOps.BufferOpenCenterlines(after, .25, .001), chunked.Paths);
        Assert.Same(direct, cache.Get(before, .25, .001, CancellationToken.None, diagnostics));
        Assert.Same(chunked, cache.Get(after, .25, .001, CancellationToken.None, diagnostics));
        Assert.Equal(2, diagnostics.FollowingCacheMisses); Assert.Equal(2, diagnostics.FollowingCacheHits);
    }

    [Fact]
    public void CancellationNeverInstallsAPartialEntryAndAlsoAppliesToHits()
    {
        var cache = new PocketContinuationRouter.FollowingSweepCache();
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var source = new PointD[] { new(0, 0), new(10, 0) };
        using var duringKey = new CancellationTokenSource();
        var canceling = new CancelingLine(source, duringKey.Cancel);
        Assert.Throws<OperationCanceledException>(() => cache.Get(new[] { canceling }, .5, .001, duringKey.Token, diagnostics));
        Assert.Equal(0, diagnostics.FollowingCacheMisses);
        cache.Get(new[] { source }, .5, .001, CancellationToken.None, diagnostics);
        using var beforeHit = new CancellationTokenSource(); beforeHit.Cancel();
        Assert.Throws<OperationCanceledException>(() => cache.Get(new[] { source }, .5, .001, beforeHit.Token, diagnostics));
        Assert.Equal(0, diagnostics.FollowingCacheHits);
        cache.Get(new[] { source }, .5, .001, CancellationToken.None, diagnostics);
        Assert.Equal(1, diagnostics.FollowingCacheMisses); Assert.Equal(1, diagnostics.FollowingCacheHits);
    }

    [Fact]
    public void ConstructionReuseCannotPromoteItsGeometryIntoExecutedStock()
    {
        var cache = new PocketContinuationRouter.FollowingSweepCache();
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        cache.Get(new IReadOnlyList<PointD>[] { new PointD[] { new(0, 0), new(20, 0) } },
            .5, .001, CancellationToken.None, diagnostics);
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        var proof = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -2, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None,
            diagnostics, followingCache: cache);
        Assert.False(proof.IsTravelSafe(new(5, 0, -2), new(15, 0, -2), .5));
        Assert.Equal(0, stock.RecordedStrokes);
    }

    [Fact]
    public void EveryLocalApprovalCanBeIndependentlyRecheckedAgainstTheLegacyWholeDomain()
    {
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        stock.Record([new(0, 0, -1), new(20, 0, -1)]);
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var proof = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None,
            diagnostics, validateIndexedCoverage: true);
        Assert.True(proof.IsTravelSafe(new(5, 0, -1), new(15, 0, -1), .5));
        Assert.True(proof.IsTravelSafe(new(6, 0, -1), new(14, 0, -1), .5));
        Assert.Equal(2, diagnostics.LocalProofs); Assert.Equal(2, diagnostics.ValidatedLocalProofs);
        Assert.Equal(0, diagnostics.LocalProofMismatches);
    }

    private sealed class CollidingComparer : IEqualityComparer<PocketContinuationRouter.FollowingSweepCache.SweepKey>
    {
        public bool Equals(PocketContinuationRouter.FollowingSweepCache.SweepKey? a,
            PocketContinuationRouter.FollowingSweepCache.SweepKey? b) => a?.Equals(b) ?? b is null;
        public int GetHashCode(PocketContinuationRouter.FollowingSweepCache.SweepKey key) => 0;
    }
    private sealed class CancelingLine(PointD[] points, Action cancel) : IReadOnlyList<PointD>
    {
        public int Count => points.Length;
        public PointD this[int index] { get { cancel(); return points[index]; } }
        public IEnumerator<PointD> GetEnumerator() => ((IEnumerable<PointD>)points).GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }
    private static void EqualPolygons(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++) Assert.Equal(expected[i].ToArray(), actual[i].ToArray());
    }
}
