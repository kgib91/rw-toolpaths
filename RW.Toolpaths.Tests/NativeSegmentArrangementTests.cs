using Clipper2Lib;
using Xunit;
using Point = RW.Toolpaths.NativeSegmentArrangement.Point;
using Segment = RW.Toolpaths.NativeSegmentArrangement.Segment;

namespace RW.Toolpaths.Tests;

public sealed class NativeSegmentArrangementTests
{
    [Fact]
    public void OneTickBacktrackRetainsEveryOriginalSiteAndItsOwner()
    {
        Segment[] source = { new(new(0, 0), new(10, 10)), new(new(10, 10), new(9, 9)),
            new(new(9, 9), new(15, 0)) };
        var saved = source.ToArray();
        var result = NativeSegmentArrangement.Prepare(source);
        Assert.True(result.Changed);
        Assert.Equal(1, result.SplitOriginalSites);
        Assert.Equal(1, result.DuplicateAtoms);
        Assert.Equal(3, result.AtomicSites);
        var spur = Assert.Single(result.Atoms!, a => SameUndirected(a.Segment, new(new(9, 9), new(10, 10))));
        Assert.Equal(0, spur.OriginalOrdinal);
        Assert.Contains(1, spur.AdditionalOriginalOrdinals);
        Assert.Equal(saved, source);
        AssertSameFiniteUnion(source, result);
    }

    [Fact]
    public void SharedEndpointAndOneTickGapAreDistinct()
    {
        Segment[] ordinary = { new(new(0, 0), new(10, 0)), new(new(10, 0), new(12, 4)),
            new(new(5, 1), new(5, 9)) };
        Assert.False(NativeSegmentArrangement.Prepare(ordinary).Changed);
        Segment[] touch = { ordinary[0], new(new(5, 0), new(6, 1)) };
        var result = NativeSegmentArrangement.Prepare(touch);
        Assert.True(result.Changed);
        Assert.Equal(3, result.AtomicSites);
        AssertSameFiniteUnion(touch, result);
        // The nearby endpoint remains exactly one native tick away. It is not
        // treated as an authored contact or moved onto the first segment.
        Assert.Equal(1, DistanceSquared(new(5, 1), ordinary[0]));
    }

    [Fact]
    public void ReversedPartialOverlapHasBothOwnersAndNoLostTail()
    {
        Segment[] source = { new(new(0, 0), new(10, 0)), new(new(7, 0), new(2, 0)) };
        var result = NativeSegmentArrangement.Prepare(source);
        Assert.Equal(3, result.AtomicSites);
        Assert.Equal(1, result.DuplicateAtoms);
        var common = Assert.Single(result.Atoms!, a => SameUndirected(a.Segment, new(new(2, 0), new(7, 0))));
        Assert.Contains(1, common.AdditionalOriginalOrdinals);
        AssertSameFiniteUnion(source, result);
    }

    [Fact]
    public void IntegerCrossingIsSplitWithoutMovingEitherSegment()
    {
        Segment[] source = { new(new(0, 0), new(10, 10)), new(new(0, 10), new(10, 0)) };
        var result = NativeSegmentArrangement.Prepare(source);
        Assert.Equal(4, result.AtomicSites);
        Assert.Equal(2, result.SplitOriginalSites);
        Assert.All(result.Atoms!, atom => Assert.True(atom.Segment.A == new Point(5, 5) || atom.Segment.B == new Point(5, 5)));
        AssertSameFiniteUnion(source, result);
    }

    [Fact]
    public void NonintegerCrossingFailsWithTheOriginalWitness()
    {
        Segment[] source = { new(new(0, 0), new(3, 3)), new(new(0, 2), new(3, 0)) };
        var error = Assert.Throws<NativeSegmentArrangement.UnsupportedContactException>(() => NativeSegmentArrangement.Prepare(source));
        Assert.Contains("noninteger-proper-crossing", error.Message);
        Assert.Contains("0/1", error.Message);
        Assert.Equal(0, error.FirstOriginal);
        Assert.Equal(1, error.SecondOriginal);
        Assert.Equal("noninteger-proper-crossing", error.Reason);
    }

    [Fact]
    public void FullInt32RangeCrossAndProjectionDoNotOverflow()
    {
        int low = -int.MaxValue, high = int.MaxValue;
        Segment[] source = { new(new(low, low), new(high, high)), new(new(low, high), new(high, low)) };
        var result = NativeSegmentArrangement.Prepare(source);
        Assert.Equal(4, result.AtomicSites);
        Assert.All(result.Atoms!, atom => Assert.True(atom.Segment.A == new Point(0, 0) || atom.Segment.B == new Point(0, 0)));
    }

    [Fact]
    public void CancellationAndWorkBudgetCannotPublishPartialSites()
    {
        Segment[] source = { new(new(0, 0), new(10, 0)), new(new(5, 0), new(5, 10)),
            new(new(8, 0), new(8, 10)) };
        using var cancel = new CancellationTokenSource(); cancel.Cancel();
        Assert.Throws<OperationCanceledException>(() => NativeSegmentArrangement.Prepare(source, cancel.Token));
        Assert.Contains("pair-work-budget", Assert.Throws<InvalidOperationException>(() =>
            NativeSegmentArrangement.Prepare(source, candidatePairLimit: 1)).Message);
        Assert.True(NativeSegmentArrangement.Prepare(source).Changed);
    }

    [Fact]
    public void ProviderRetainsFiniteMedialGeometryForBacktrackingHole()
    {
        PointD[] outer = { new(0, 0), new(1000, 0), new(1000, 1000), new(0, 1000) };
        IReadOnlyList<PointD>[] holes = { new PointD[] { new(100, 100), new(200, 200), new(199, 199),
            new(300, 100), new(300, 400), new(100, 400) } };
        var result = BoostVoronoiProvider.CreateDefault().ConstructMedialAxis(outer, holes, .01, 10000);
        Assert.NotEmpty(result);
        Assert.All(result, segment => {
            Assert.True(double.IsFinite(segment.Point0.X) && double.IsFinite(segment.Point0.Y) && double.IsFinite(segment.Point0.Radius));
            Assert.True(double.IsFinite(segment.Point1.X) && double.IsFinite(segment.Point1.Y) && double.IsFinite(segment.Point1.Radius));
        });
    }

    private static bool SameUndirected(Segment a, Segment b) => a == b || a.A == b.B && a.B == b.A;
    private static void AssertSameFiniteUnion(Segment[] source, NativeSegmentArrangement.Result result)
    {
        var atoms = result.Atoms!;
        foreach (Segment segment in source)
        {
            // Independent distance-to-union oracle: preserve endpoints, interior
            // probes, and near-contact clearances, including the reversed spur.
            for (int t = 0; t <= 4; t++)
                for (int dx = -1; dx <= 1; dx++)
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        double x = segment.A.X + ((double)segment.B.X - segment.A.X) * t / 4 + dx;
                        double y = segment.A.Y + ((double)segment.B.Y - segment.A.Y) * t / 4 + dy;
                        double before = source.Min(s => DistanceSquared(x, y, s));
                        double after = atoms.Min(a => DistanceSquared(x, y, a.Segment));
                        Assert.Equal(before, after, 11);
                    }
        }
        for (int ordinal = 0; ordinal < source.Length; ordinal++)
            Assert.Contains(atoms, atom => atom.OriginalOrdinal == ordinal || atom.AdditionalOriginalOrdinals.Contains(ordinal));
    }
    private static double DistanceSquared(Point p, Segment s) => DistanceSquared(p.X, p.Y, s);
    private static double DistanceSquared(double x, double y, Segment s)
    {
        double dx = (double)s.B.X - s.A.X, dy = (double)s.B.Y - s.A.Y;
        double t = Math.Clamp(((x - s.A.X) * dx + (y - s.A.Y) * dy) / (dx * dx + dy * dy), 0, 1);
        double ex = x - s.A.X - t * dx, ey = y - s.A.Y - t * dy;
        return ex * ex + ey * ey;
    }
}
