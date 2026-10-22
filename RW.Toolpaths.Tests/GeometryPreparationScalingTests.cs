using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class GeometryPreparationScalingTests
{
    [Fact]
    public void FirstAndLaterToleranceContextsShareSnapshotsAndRetainIndependentExactOutput()
    {
        var outer = Circle(0, 0, 10, 256);
        var holes = new List<IReadOnlyList<PointD>> { Circle(3, 0, 1, 64) };
        var expectedFirst = PolygonOps.PrepareBoundaryOffset(outer, holes, .001, 0,
            sourceConstructionTolerance: .003).Execute(-.7);
        var expectedLater = PolygonOps.PrepareBoundaryOffset(outer, holes, .01, .005,
            sourceConstructionTolerance: .003).Execute(-1.4);
        var prepared = PolygonOps.PrepareBoundaryOffset(outer, holes, .01, .005,
            sourceConstructionTolerance: .003);
        var first = prepared.WithTolerances(.001, 0);
        Assert.True(first.SourcePreparationReused);
        Assert.True(first.ConstructionPreparationReused);
        Assert.False(prepared.SourcePreparationReused);
        var cacheField = typeof(PolygonOps.PreparedBoundaryOffset).GetField("_feasibilityCache",
            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic)!;
        Assert.NotSame(cacheField.GetValue(prepared), cacheField.GetValue(first));
        outer.Clear(); holes.Clear();
        EqualPaths(expectedFirst, first.Execute(-.7));
        var later = prepared.Execute(-1.4);
        EqualPaths(expectedLater, later);
        later.Clear();
        EqualPaths(expectedLater, prepared.Execute(-1.4));
    }

    [Fact]
    public void ReusingPreparedSourceStillObservesCancellation()
    {
        using var cancel = new CancellationTokenSource();
        var prepared = PolygonOps.PrepareBoundaryOffset(Rect(0, 0, 10, 10), null,
            cancellationToken: cancel.Token);
        cancel.Cancel();
        Assert.Throws<OperationCanceledException>(() => prepared.WithTolerances(.001, 0));
        Assert.Throws<OperationCanceledException>(() => prepared.Execute(-1));
    }

    [Fact]
    public void CachedNormalizationKeysPreserveLegacySortWithEqualAreasAndCentroids()
    {
        var random = new Random(982451653);
        var source = Enumerable.Range(0, 100).Select(index =>
        {
            var circle = Circle(index % 3, index % 5, index % 4 + .5, 64);
            int first = random.Next(circle.Count);
            return circle.Skip(first).Concat(circle.Take(first)).ToList();
        }).ToList();
        source.Add(new List<PointD>());
        var expected = source.Select(path => path.ToList()).ToList();
        foreach (var path in expected)
        {
            if (path.Count == 0) continue;
            int first = Enumerable.Range(0, path.Count).OrderBy(i => path[i].x).ThenBy(i => path[i].y).First();
            var rotated = path.Skip(first).Concat(path.Take(first)).ToList();
            path.Clear(); path.AddRange(rotated);
        }
        expected.Sort((first, second) =>
        {
            int comparison = Math.Abs(PolygonOps.SignedArea(second)).CompareTo(Math.Abs(PolygonOps.SignedArea(first)));
            if (comparison != 0) return comparison;
            var a = PolygonOps.Centroid(first); var b = PolygonOps.Centroid(second);
            comparison = a.x.CompareTo(b.x);
            return comparison != 0 ? comparison : a.y.CompareTo(b.y);
        });
        EqualPaths(expected, PolygonOps.NormalizeOrder(source));
    }

    [Fact]
    public void IntegerUnionReusePreservesEveryOrderedBooleanInputAndOutput()
    {
        var legacy = new List<List<PointD>>();
        var retained = new Paths64();
        for (int i = 0; i < 64; i++)
        {
            var outer = Circle((i * 7 % 13) * .67 - 5, (i * 11 % 17) * .71 - 6,
                .83 + i % 3 * .19, 40);
            var hole = Circle((i * 7 % 13) * .67 - 5, (i * 11 % 17) * .71 - 6, .3, 16);
            hole.Reverse();
            var next = new[] { (IReadOnlyList<PointD>)outer, hole };
            legacy = PolygonOps.Union(legacy.Concat(next));
            var subjects = new Paths64(retained);
            subjects.AddRange(PathUtils.ToClipper(next));
            retained = PolygonOps.UnionNormalizedInteger(subjects);
            EqualPaths(legacy, PolygonOps.MaterializeIntegerPaths(retained));
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void IntegerUnionReusePreservesLegacyRoundTripForReachableExtremeIntersections(
        bool mirrorX, bool swapAxes)
    {
        const long origin = (1L << 53) + 16;
        List<PointD> Path(params (long X, long Y)[] coordinates)
            => coordinates.Select(coordinate =>
            {
                long x = mirrorX ? -coordinate.X : coordinate.X;
                long y = coordinate.Y;
                if (swapAxes) (x, y) = (y, x);
                var point = new PointD(x / PathUtils.Scale, y / PathUtils.Scale);
                // Every authored input is exactly reachable from the existing
                // double-coordinate callsites; the odd intersection is created
                // by Clipper's integer origin + rounded relative delta.
                Assert.Equal(x, (long)Math.Round(point.x * PathUtils.Scale));
                Assert.Equal(y, (long)Math.Round(point.y * PathUtils.Scale));
                return point;
            }).ToList();
        var inputs = new[]
        {
            Path((origin, 0), (origin + 6, 8), (origin, 8)),
            Path((origin - 4, 4), (origin + 10, 4), (origin + 10, 12), (origin - 4, 12)),
            Path((origin - 4, 3), (origin + 10, 3), (origin + 10, 12), (origin - 4, 12)),
        };
        var legacy = new List<List<PointD>>();
        var retained = new Paths64();
        foreach (var input in inputs)
        {
            legacy = PolygonOps.Union(legacy.Append(input));
            var subjects = new Paths64(retained);
            subjects.AddRange(PathUtils.ToClipper(new[] { input }));
            retained = PolygonOps.UnionNormalizedInteger(subjects);
            EqualPaths(legacy, PolygonOps.MaterializeIntegerPaths(retained));
            Assert.Equal(PathUtils.ToClipper(legacy).SelectMany(path => path).Select(point => (point.X, point.Y)),
                retained.SelectMany(path => path).Select(point => (point.X, point.Y)));
        }
        if (!mirrorX && !swapAxes)
            Assert.Contains(retained.SelectMany(path => path), point => point.X == origin + 4 && point.Y == 3);
    }

    [Fact]
    public void MedianBoundaryBuildPreservesFrozenNearestAndClearanceWitnessSequence()
    {
        // Captured from the current package/runtime snapshot before these edits,
        // including all start/end coordinates in the original first-hit order.
        var outer = Circle(0, 0, 50, 2048);
        var holes = Enumerable.Range(0, 8).Select(i => (IReadOnlyList<PointD>)Circle(
            Math.Cos(i * Math.PI / 4) * 25, Math.Sin(i * Math.PI / 4) * 25, 3, 128));
        var source = Clipper.Union(PathUtils.ToClipper(new[] { (IReadOnlyList<PointD>)outer }.Concat(holes)), FillRule.NonZero);
        var index = new PolygonOps.BoundarySegmentIndex(source);
        Assert.True(index.PartitionCount > 0);
        Assert.False(index.LegacyWitnessIndexBuilt);
        Assert.True(index.DistanceSquared(new Point64(0, 0)) > 0);
        Assert.False(index.LegacyWitnessIndexBuilt);
        ulong hash = 14695981039346656037;
        void Add(long value) { hash ^= unchecked((ulong)value); hash *= 1099511628211; }
        var random = new Random(982451653);
        for (int i = 0; i < 2048; i++)
        {
            var point = new Point64((long)((random.NextDouble() - .5) * 120 * PathUtils.Scale),
                (long)((random.NextDouble() - .5) * 120 * PathUtils.Scale));
            var nearest = index.NearestSegment(point);
            Add(nearest.Start.X); Add(nearest.Start.Y); Add(nearest.End.X); Add(nearest.End.Y);
            var end = new Point64(point.X + (long)(random.NextDouble() * PathUtils.Scale),
                point.Y + (long)(random.NextDouble() * PathUtils.Scale));
            var violation = index.ClearanceViolation(point, end, PathUtils.Scale / 2);
            Add(violation.HasValue ? 1 : 0);
            if (violation is { } edge) { Add(edge.Start.X); Add(edge.Start.Y); Add(edge.End.X); Add(edge.End.Y); }
        }
        Assert.True(index.LegacyWitnessIndexBuilt);
        Assert.Equal("7299FD19DCFE849E", hash.ToString("X16"));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(4)]
    public void EqualDistanceSourceEdgesKeepFrozenLegacyTieWitness(int rotation)
    {
        var ring = Rect(-1, -1, 1, 1);
        ring = ring.Skip(rotation).Concat(ring.Take(rotation)).ToList();
        var index = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(new[] { ring }));
        // Four edges fit in a leaf, so the closing edge wins the exact tie.
        var expected = (PathUtils.ToClipper(new[] { ring })[0][^1], PathUtils.ToClipper(new[] { ring })[0][0]);
        Assert.Equal(expected, index.NearestSegment(new Point64(0, 0)));
    }

    [Fact]
    public void ConcurrentFirstWitnessQueriesPublishOneCompleteLegacyHierarchy()
    {
        var source = PathUtils.ToClipper(new[] { Circle(0, 0, 10, 1024) });
        var sequential = new PolygonOps.BoundarySegmentIndex(source);
        var expected = sequential.NearestSegment(new Point64(0, 0));
        var concurrent = new PolygonOps.BoundarySegmentIndex(source);
        Parallel.For(0, 64, _ => Assert.Equal(expected, concurrent.NearestSegment(new Point64(0, 0))));
        Assert.True(concurrent.LegacyWitnessIndexBuilt);
    }

    private static List<PointD> Circle(double x, double y, double radius, int count)
        => Enumerable.Range(0, count).Select(index =>
        {
            double angle = index * Math.PI * 2 / count;
            return new PointD(x + radius * Math.Cos(angle), y + radius * Math.Sin(angle));
        }).ToList();

    private static List<PointD> Rect(double x0, double y0, double x1, double y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };

    private static void EqualPaths(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
            Assert.Equal(expected[i].Select(point => (point.x, point.y)), actual[i].Select(point => (point.x, point.y)));
    }
}
