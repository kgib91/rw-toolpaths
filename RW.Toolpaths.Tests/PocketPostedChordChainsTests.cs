using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketPostedChordChainsTests
{
    [Fact]
    public void ExactEdgesBranchesAndExplicitCycleClosuresPreserveTheCompleteSignedStrokeDomain()
    {
        var source = new List<PocketPostedChordChains.Segment>();
        void Cycle(long cx, long cy, long radius)
        {
            var points = Enumerable.Range(0, 64).Select(index => new PocketPostedChordChains.Point(
                cx + (long)Math.Round(radius * Math.Cos(Math.Tau * index / 64)),
                cy + (long)Math.Round(radius * Math.Sin(Math.Tau * index / 64)))).ToArray();
            for (int index = 0; index < points.Length; index++) source.Add(new(points[index], points[(index + 1) % points.Length]));
        }
        Cycle(10000, 10000, 10000); Cycle(40000, 10000, 5000);
        source.AddRange([new(new(20000, 10000), new(25000, 10000)),
            new(new(25000, 10000), new(25000, 15000)), new(new(25000, 10000), new(25000, 5000))]);
        source.AddRange(source.Take(13).Select(edge => new PocketPostedChordChains.Segment(edge.B, edge.A)).ToArray());
        var joined = PocketPostedChordChains.Build(source, CancellationToken.None);
        Assert.Equal(13, joined.DuplicateEdges);
        Assert.Equal(2, joined.BranchVertices);
        Assert.Equal(2, joined.ClosedCycles);
        var expected = source.Select(Canonical).ToHashSet();
        var actual = joined.Paths.SelectMany(path => path.Zip(path.Skip(1), (a, b) => Canonical(new(a, b)))).ToHashSet();
        Assert.True(expected.SetEquals(actual), "Joining added or lost an actual F3 cutter chord.");
        Assert.True(joined.Paths.Sum(path => path.Count - 1) == joined.UniqueEdges);
        Assert.True(joined.Paths.Count < source.Count / 10);

        // Each cycle remains a ROUND-ended open stroke with its original closing
        // edge. Both canonical per-chord and joined domains retain their holes.
        var reference = Sweep(source.Select(edge => new[] { edge.A, edge.B }));
        var swept = Sweep(joined.Paths);
        Assert.Equal(reference.Count(path => Clipper.Area(path) < 0), swept.Count(path => Clipper.Area(path) < 0));
        Assert.Equal(reference.Count(path => Clipper.Area(path) > 0), swept.Count(path => Clipper.Area(path) > 0));
        Assert.Equal(2, swept.Count(path => Clipper.Area(path) < 0));
        var oldIndex = new PolygonOps.BoundarySegmentIndex(reference);
        var newIndex = new PolygonOps.BoundarySegmentIndex(swept);
        foreach (var point in new[] { new PointD(10, 10), new PointD(40, 10), new PointD(28, 10) })
        { Assert.False(oldIndex.IsInside(Grid(point))); Assert.False(newIndex.IsInside(Grid(point))); }
        foreach (var point in new[] { new PointD(20.4, 10), new PointD(25, 11), new PointD(45.4, 10) })
        { Assert.True(oldIndex.IsInside(Grid(point))); Assert.True(newIndex.IsInside(Grid(point))); }
    }

    [Fact]
    public void DenseOverlappingEarlierChainsUseTheSamePhysicalAuthorityWithFewerRoundCaps()
    {
        var joined = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5), 1, CancellationToken.None,
            permitExactRectangle: false);
        var reference = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5), 1, CancellationToken.None,
            permitExactRectangle: false, joinFullChords: false);
        foreach (double y in new[] { -.2, .2 })
        {
            var chain = Enumerable.Range(0, 65).Select(index => new Point3D(index / 8d, y, -3.5)).ToArray();
            // Repeat the complete executed chain in reverse; its swept stock
            // footprint is identical and must not acquire duplicate round caps.
            foreach (var stroke in new[] { chain, chain.Reverse().ToArray() })
            { joined.Record(stroke); reference.Record(stroke); }
        }
        Point3D a = new(2, 0, -7), b = new(6, 0, -7);
        Assert.True(reference.Capture().IsSafe(a, b));
        Assert.True(joined.Capture().IsSafe(a, b));
        Assert.Equal(256, joined.InnerFullInputChords);
        Assert.Equal(128, joined.InnerUniqueFullChords);
        Assert.Equal(128, joined.InnerDuplicateChords);
        Assert.Equal(2, joined.InnerJoinedPaths);
        Assert.Equal(130, joined.InnerJoinedPoints);
        Assert.Equal(256, reference.InnerJoinedPaths);
    }

    [Fact]
    public void PartialThresholdRampNeverManufacturesAChainAcrossTheMissingStockGap()
    {
        foreach (bool join in new[] { false, true })
        {
            var stock = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5), .1, CancellationToken.None,
                permitExactRectangle: false, joinFullChords: join);
            stock.Record([new(0, 0, -3.5), new(4.9, 0, -3.5)]);
            stock.Record([new(0, 0, 0), new(10, 0, -7)]);
            Assert.False(stock.Capture().IsSafe(new(4.85, 0, -7), new(5.05, 0, -7)));
            Assert.Equal(1, stock.InnerPartialChords);
            Assert.Equal(1, stock.InnerFullInputChords);
            // (4.95,.099) lies in the new capsule but outside both old eligible
            // strokes ending4.9 and beginning5; no chord can fill that gap.
            Assert.True(Math.Sqrt(.05 * .05 + .099 * .099) > .1);
        }
    }

    [Fact]
    public void CancellationPreventsPublishingAnyPartiallyConstructedChainSet()
    {
        using var canceled = new CancellationTokenSource();
        canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => PocketPostedChordChains.Build(
            [new(new(0, 0), new(1000, 0))], canceled.Token));
    }

    private static PocketPostedChordChains.Segment Canonical(PocketPostedChordChains.Segment edge)
        => edge.A.X < edge.B.X || edge.A.X == edge.B.X && edge.A.Y <= edge.B.Y ? edge : new(edge.B, edge.A);

    private static Point64 Grid(PointD point) => new((long)Math.Round(point.x * PathUtils.Scale),
        (long)Math.Round(point.y * PathUtils.Scale));

    private static Paths64 Sweep(IEnumerable<IEnumerable<PocketPostedChordChains.Point>> chains)
    {
        var offsetter = new ClipperOffset(2, .001 * PathUtils.Scale);
        offsetter.AddPaths(new Paths64(chains.Select(chain => new Path64(chain.Select(point =>
            Grid(new PointD(point.X / 1000d, point.Y / 1000d)))))), JoinType.Round, EndType.Round);
        var result = new Paths64();
        offsetter.Execute((1.25 - 16 / PathUtils.Scale) * PathUtils.Scale, result);
        return Clipper.Union(result, FillRule.NonZero);
    }
}
