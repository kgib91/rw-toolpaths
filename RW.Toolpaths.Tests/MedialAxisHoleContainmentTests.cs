using Clipper2Lib;
using Classifier = RW.Toolpaths.MedialAxisEdgeClassifier;

namespace RW.Toolpaths.Tests;

public class MedialAxisHoleContainmentTests
{
    [Fact]
    public void CompleteHoleBoundsIndexMatchesOriginalRayPredicateAtEveryBoundaryTie()
    {
        PointD[] outer = [new(-1000, -1000), new(1000, -1000), new(1000, 1000), new(-1000, 1000)];
        IReadOnlyList<PointD>[] holes = Enumerable.Range(0, 512).Select(index =>
        {
            double x = -900 + 55 * (index % 32), y = -900 + 100 * (index / 32);
            PointD[] ring = [new(x, y), new(x + 30, y), new(x + 20, y + 40), new(x, y + 40), new(x, y)];
            if ((index & 1) == 0) Array.Reverse(ring);
            return (IReadOnlyList<PointD>)ring;
        }).ToArray();
        var prepared = Prepare(outer, holes, memoLimit: 4);
        foreach (var hole in holes)
        foreach (var point in hole)
        {
            Check(new(point.x, point.y));
            Check(new(Math.BitDecrement(point.x), point.y));
            Check(new(Math.BitIncrement(point.x), point.y));
            Check(new(point.x, Math.BitDecrement(point.y)));
            Check(new(point.x, Math.BitIncrement(point.y)));
        }
        var random = new Random(123417);
        for (int index = 0; index < 4096; index++)
            Check(new(-1100 + 2200 * random.NextDouble(), -1100 + 2200 * random.NextDouble()));
        Assert.True(prepared.HolePredicateTests < prepared.Queries * holes.Length / 32,
            $"Sparse-hole bounds must exclude distant predicates: queries={prepared.Queries}, "
            + $"predicates={prepared.HolePredicateTests}, holes={holes.Length}.");
        Assert.True(prepared.HoleNodeVisits > 0);

        void Check(Classifier.EdgePoint point)
        {
            bool outerContains = ReferenceContains(point, outer);
            bool holeContains = outerContains && holes.Any(hole => ReferenceContains(point, hole));
            Assert.Equal(outerContains && !holeContains, prepared.Contains(point, out bool actualHole));
            Assert.Equal(holeContains, actualHole);
        }
    }

    [Fact]
    public void IndexedClassificationPreservesAllColorsAndOriginalOrderedTraversal()
    {
        PointD[] outer = [new(-10, -10), new(200, -10), new(200, 200), new(-10, 200)];
        IReadOnlyList<PointD>[] holes = Enumerable.Range(0, 64)
            .Select(i => (IReadOnlyList<PointD>)Square(5 + i % 8 * 20, 5 + i / 8 * 20, 10)).ToArray();
        var queries = new List<Classifier.EdgePoint>();
        foreach (var hole in holes) foreach (var vertex in hole) queries.Add(new(vertex.x, vertex.y));
        queries.Add(new(-100, -100)); queries.Add(new(180, 180)); queries.Add(new(8, 8));
        var cell = new Classifier.CellSite(true, new(-9999, -9999), null);
        var input = new List<Classifier.EdgeData>();
        // Repeated references model immutable native vertices shared by halfedges.
        for (int repeat = 0; repeat < 8; repeat++)
        foreach (var point in queries)
        {
            int index = input.Count;
            input.Add(new(index, index + 1, true, false, false, -1, -1, point, point, cell, cell));
            input.Add(new(index + 1, index, true, false, false, -1, -1, point, point, cell, cell));
        }
        int last = input.Count;
        input.Add(new(last, -1, true, false, true, -1, -1, null, null, cell, cell));
        input.Add(new(last + 1, -1, false, true, false, -1, -1, null, null, cell, cell));
        var indexed = input.Select(edge => edge with { }).ToArray();
        var reference = input.Select(edge => edge with { }).ToArray();
        var logs = new List<string>();
        Classifier.ClassifyEdges(indexed, outer, holes, diagnostic: logs.Add);
        Classifier.ClassifyEdgesReference(reference, outer, holes);
        Assert.Equal(reference.Select(edge => edge.Color), indexed.Select(edge => edge.Color));
        Assert.Equal(Classifier.SelectInnerPrimaryTraversal(reference),
            Classifier.SelectInnerPrimaryTraversal(indexed));
        Assert.Contains(logs, line => line.Contains("event=containment") && line.Contains("memoHits="));
    }

    [Fact]
    public void BoundedMemoUsesOnlyExactImmutablePointIdentityAndRecomputesAfterClearing()
    {
        PointD[] outer = Square(0, 0, 100);
        IReadOnlyList<PointD>[] holes = [Square(20, 20, 20)];
        var prepared = Prepare(outer, holes, memoLimit: 2);
        var material = new Classifier.EdgePoint(10, 10);
        var excluded = new Classifier.EdgePoint(30, 30);
        var outside = new Classifier.EdgePoint(110, 110);
        Assert.True(prepared.Contains(material, out _));
        Assert.True(prepared.Contains(material, out _));
        Assert.Equal(1, prepared.MemoHits);
        Assert.False(prepared.Contains(excluded, out bool isHole)); Assert.True(isHole);
        Assert.False(prepared.Contains(outside, out isHole)); Assert.False(isHole);
        Assert.Equal(1, prepared.MemoClears);
        Assert.True(prepared.Contains(material, out _));
        Assert.Equal(1, prepared.MemoHits);
        // Equal values in a separate object are deliberately recomputed. A close
        // but distinct point never becomes evidence for its neighbor either.
        Assert.True(prepared.Contains(new(10, 10), out _));
        Assert.True(prepared.Contains(new(Math.BitIncrement(10d), 10), out _));
        Assert.Equal(1, prepared.MemoHits);
        Assert.InRange(prepared.MemoCount, 0, prepared.MemoLimit);
    }

    [Fact]
    public void CancellationIsCheckedBeforeWarmPointEvidenceAndDuringPreparation()
    {
        using var cancellation = new CancellationTokenSource();
        PointD[] outer = Square(0, 0, 100);
        IReadOnlyList<PointD>[] holes = Enumerable.Range(0, 128)
            .Select(i => (IReadOnlyList<PointD>)Square(i, i, 1)).ToArray();
        var prepared = Prepare(outer, holes, cancellation.Token);
        var point = new Classifier.EdgePoint(90, 10);
        Assert.True(prepared.Contains(point, out _));
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => prepared.Contains(point, out _));
        Assert.Throws<OperationCanceledException>(() => Prepare(outer, holes, cancellation.Token));
        Assert.True(Prepare(outer, holes).Contains(point, out _));
    }

    [Fact]
    public void EmptyAndOverlappingHoleBoxesRetainCompleteSignedMembership()
    {
        PointD[] outer = Square(-20, -20, 100);
        IReadOnlyList<PointD>[] holes = [[], [new(5, 5)], [new(0, 0), new(10, 0), new(20, 0)],
            Square(0, 0, 30), Square(10, 10, 30), Square(20, 20, 30), [], [], [], []];
        var indexed = Prepare(outer, holes);
        var original = Prepare(outer, holes, useHoleIndex: false, memoLimit: 0);
        for (int x = -25; x <= 85; x += 5)
        for (int y = -25; y <= 85; y += 5)
        {
            var point = new Classifier.EdgePoint(x, y);
            Assert.Equal(original.Contains(point, out bool expectedHole), indexed.Contains(point, out bool actualHole));
            Assert.Equal(expectedHole, actualHole);
        }
    }

    [Fact]
    public void InclusiveBoundsPreserveTinyHolesAtLargeNativeCoordinates()
    {
        const double origin = 2_000_000_000;
        PointD[] outer = Square(origin - 100, origin - 100, 200);
        var tiny = Square(origin, origin, 1);
        IReadOnlyList<PointD>[] holes = Enumerable.Range(0, 16)
            .Select(i => (IReadOnlyList<PointD>)Square(origin - 90 + 5 * i, origin - 90, 1))
            .Append(tiny).ToArray();
        var indexed = Prepare(outer, holes);
        foreach (double x in new[] { Math.BitDecrement(origin), origin, Math.BitIncrement(origin),
            origin + .5, Math.BitDecrement(origin + 1), origin + 1, Math.BitIncrement(origin + 1) })
        foreach (double y in new[] { Math.BitDecrement(origin), origin, Math.BitIncrement(origin),
            origin + .5, origin + 1, Math.BitIncrement(origin + 1) })
        {
            var point = new Classifier.EdgePoint(x, y);
            bool expectedHole = holes.Any(hole => ReferenceContains(point, hole));
            Assert.Equal(!expectedHole, indexed.Contains(point, out bool actualHole));
            Assert.Equal(expectedHole, actualHole);
        }
    }

    private static Classifier.PreparedRegionContainment Prepare(IReadOnlyList<PointD> outer,
        IReadOnlyList<IReadOnlyList<PointD>> holes, CancellationToken token = default,
        bool useHoleIndex = true, int memoLimit = Classifier.PreparedRegionContainment.DefaultMemoLimit)
        => new(new Classifier.PreparedRingContainment(outer, token),
            holes.Select(hole => new Classifier.PreparedRingContainment(hole, token)).ToArray(),
            token, useHoleIndex, memoLimit);

    private static PointD[] Square(double x, double y, double size)
        => [new(x, y), new(x + size, y), new(x + size, y + size), new(x, y + size)];

    private static bool ReferenceContains(Classifier.EdgePoint point, IReadOnlyList<PointD> ring)
    {
        bool inside = false;
        for (int index = 0, previous = ring.Count - 1; index < ring.Count; previous = index++)
        {
            var current = ring[index]; var prior = ring[previous];
            if ((current.y > point.Y) != (prior.y > point.Y)
                && (prior.x - current.x) * (point.Y - current.y) / (prior.y - current.y) + current.x > point.X)
                inside = !inside;
        }
        return inside;
    }
}
