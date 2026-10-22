using System.Reflection;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeFloorGraphPointIndexTests
{
    [Fact]
    public void InclusiveRectangleReturnsOriginalOrdinalsFromAnImmutableSnapshot()
    {
        var points = Enumerable.Range(0, 4096).Select(i => new Point3D(i % 64, i / 64, i)).ToArray();
        var snapshot = points.ToArray();
        var index = new PreparedPointIndex(points);
        points[0] = new(10000, 10000, -1);
        var diagnostics = new PreparedPointIndex.QueryDiagnostics();
        foreach (var box in new[] { (0d, 0d, 0d, 0d), (15d, 27d, 18d, 29d),
            (-1d, -1d, 64d, 64d), (30d, 30d, 29d, 29d),
            (Math.BitDecrement(4d), 7d, Math.BitIncrement(4d), 7d) })
        {
            var expected = Enumerable.Range(0, snapshot.Length).Where(i =>
                snapshot[i].X >= box.Item1 && snapshot[i].Y >= box.Item2
                && snapshot[i].X <= box.Item3 && snapshot[i].Y <= box.Item4).ToArray();
            Assert.Equal(expected, index.IntersectingOrdinals(box.Item1, box.Item2, box.Item3, box.Item4,
                diagnostics: diagnostics).OrderBy(i => i).ToArray());
        }
        Assert.True(index.SupportsBounds);
        Assert.Equal(5, diagnostics.RangeQueries);
        Assert.True(diagnostics.RangeNodeBounds > 0);
    }

    [Fact]
    public void RectangleNonfiniteFallbackPreservesTheLiteralInclusivePredicate()
    {
        Point3D[] points = [new(0, 0, 1), new(double.NaN, 2, 2),
            new(double.PositiveInfinity, 3, 3), new(double.NegativeInfinity, 4, 4)];
        var index = new PreparedPointIndex(points);
        var diagnostics = new PreparedPointIndex.QueryDiagnostics();
        Assert.False(index.SupportsBounds);
        Assert.Equal(new[] { 0, 2, 3 }, index.IntersectingOrdinals(double.NegativeInfinity, 0,
            double.PositiveInfinity, 4, diagnostics: diagnostics).OrderBy(i => i).ToArray());
        Assert.Empty(index.IntersectingOrdinals(double.NaN, 0, 1, 4));
        Assert.Equal(1, diagnostics.RangeFallbackQueries);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GraphPointsEdgesAndStartKeepOriginalOrderAndExactLengths(bool commandVerticesOnly)
    {
        (Point3D A, Point3D B)[] chords = [
            (new(0, 0, -2), new(10, 0, -2)),
            (new(3, 0, -2), new(3, 4, -2)),
            (new(7, 0, -2), new(7, -4, -2)),
            (new(0, 0, -2), new(10, 10, -2)),
            (new(5, 5, -2), new(5, 9, -2)),
            (new(10, 0, -2), new(0, 0, -2)),
            (new(3, 0, -8), new(3, 4, -8)),
            (new(3, 4 / PathUtils.Scale, -2), new(6, 4 / PathUtils.Scale, -2))];
        AssertEquivalent(Build(chords, new(3, 0, -2), commandVerticesOnly, false),
            Build(chords, new(3, 0, -2), commandVerticesOnly, true));
    }

    [Theory]
    [InlineData(128, false)]
    [InlineData(128, true)]
    [InlineData(512, false)]
    [InlineData(1024, false)]
    public void SharedXDistributedYPrunesTheFormerAllPointStripScan(int rows, bool commandVerticesOnly)
    {
        var chords = Enumerable.Range(0, rows).SelectMany(row => new (Point3D, Point3D)[] {
            (new(0, row * 10, -1), new(0, row * 10 + 2, -1)),
            (new(0, row * 10 + 1, -1), new(1, row * 10 + 1, -1)) }).ToArray();
        var reference = Build(chords, new(0, 1, -1), commandVerticesOnly, false);
        var actual = Build(chords, new(0, 1, -1), commandVerticesOnly, true);
        AssertEquivalent(reference, actual);
        Assert.True(actual.CandidateVisits < reference.CandidateVisits / 16,
            $"rows={rows} referenceCandidates={reference.CandidateVisits} indexedCandidates={actual.CandidateVisits}");
        Assert.True(actual.PointTests < reference.CandidateVisits / 8,
            $"rows={rows} originalCandidates={reference.CandidateVisits} indexPointTests={actual.PointTests}");
        Assert.True(actual.NodeBounds > 0);
        Assert.True(actual.PointIndexUsed);
        Assert.Equal(reference.CandidateVisits, actual.EstimatedStripCandidates);
        Assert.Equal(0, actual.IndexBypasses);
        Assert.Equal(chords.Length, actual.IndexQueries);
        Assert.Equal(0, actual.IndexFallbacks);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SparseXStripsReuseOriginalScanInsteadOfBuildingPointIndex(bool commandVerticesOnly)
    {
        var chords = Enumerable.Range(0, 2048).Select(row =>
            (new Point3D(row * 10, row % 11, -1), new Point3D(row * 10 + 1, row % 11, -1))).ToArray();
        var reference = Build(chords, new(0, 0, -1), commandVerticesOnly, false);
        var actual = Build(chords, new(0, 0, -1), commandVerticesOnly, true);
        AssertEquivalent(reference, actual);
        Assert.False(actual.PointIndexUsed);
        Assert.Equal(1, actual.IndexBypasses);
        Assert.Equal(reference.CandidateVisits, actual.EstimatedStripCandidates);
        Assert.Equal(reference.CandidateVisits, actual.CandidateVisits);
        Assert.Equal(0, actual.IndexQueries);
        Assert.Equal(0, actual.NodeBounds);
        Assert.Equal(0, actual.PointTests);
        Assert.Equal(0, actual.IndexFallbacks);
    }

    [Fact]
    public void NonfiniteGraphCoordinatesUseTheOriginalScan()
    {
        (Point3D A, Point3D B)[] chords = [
            (new(0, 0, 0), new(1, 1, 0)),
            (new(double.NaN, 0, 0), new(2, 2, 0)),
            (new(double.PositiveInfinity, 3, 0), new(3, 3, 0))];
        var reference = Build(chords, new(0, 0, 0), false, false);
        var actual = Build(chords, new(0, 0, 0), false, true);
        AssertEquivalent(reference, actual);
        Assert.Equal(0, actual.IndexQueries);
        Assert.True(actual.IndexFallbacks > 0);
    }

    [Fact]
    public void CancellationPreventsRectangleAndGraphConstruction()
    {
        var index = new PreparedPointIndex(new[] { new Point3D(0, 0, 0) });
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => index.IntersectingOrdinals(0, 0, 1, 1, cancelled.Token));
        var exception = Assert.Throws<TargetInvocationException>(() => Build(
            new[] { (new Point3D(0, 0, 0), new Point3D(1, 1, 0)) }, new(0, 0, 0),
            false, true, cancelled.Token));
        Assert.IsType<OperationCanceledException>(exception.InnerException);
    }

    private sealed record Snapshot(Point3D[] Points, (int A, int B, double Length)[] Edges,
        int Start, long CandidateVisits, long NodeBounds, long PointTests, long IndexQueries, long IndexFallbacks,
        bool PointIndexUsed, long EstimatedStripCandidates, long IndexBypasses);

    private static Snapshot Build(IEnumerable<(Point3D A, Point3D B)> chords, Point3D from,
        bool commandVerticesOnly, bool indexed, CancellationToken token = default)
    {
        const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
        var type = typeof(PocketContinuationRouter).GetNestedType("NativeFloorGraph", BindingFlags.NonPublic)!;
        var graph = type.GetConstructors(flags).Single().Invoke(new object[] { chords, from, token, commandVerticesOnly, indexed });
        T Read<T>(string name) => (T)type.GetProperty(name, flags)!.GetValue(graph)!;
        var points = Read<List<Point3D>>("Points").ToArray();
        var edges = Read<System.Collections.IEnumerable>("Edges").Cast<object>().Select(edge =>
        {
            var edgeType = edge.GetType();
            return ((int)edgeType.GetProperty("A", flags)!.GetValue(edge)!,
                (int)edgeType.GetProperty("B", flags)!.GetValue(edge)!,
                (double)edgeType.GetProperty("Length", flags)!.GetValue(edge)!);
        }).ToArray();
        return new(points, edges, Read<int>("Start"), Read<long>("SplitCandidateVisits"),
            Read<long>("SplitIndexNodeBounds"), Read<long>("SplitIndexPointTests"),
            Read<long>("SplitIndexQueries"), Read<long>("SplitIndexFallbacks"), Read<bool>("SplitPointIndexUsed"),
            Read<long>("SplitEstimatedStripCandidates"), Read<long>("SplitIndexBypasses"));
    }

    private static void AssertEquivalent(Snapshot reference, Snapshot actual)
    {
        Assert.Equal(reference.Points, actual.Points);
        Assert.Equal(reference.Edges, actual.Edges);
        Assert.Equal(reference.Start, actual.Start);
    }
}
