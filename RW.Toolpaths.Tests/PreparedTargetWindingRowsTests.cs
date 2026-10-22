using Clipper2Lib;
using RW.Toolpaths.Milling.Acceleration;

namespace RW.Toolpaths.Tests;

public sealed class PreparedTargetWindingRowsTests
{
    [Fact]
    public void CachedDyadicRowsMatchLiteralNonZeroWithNestedHolesAndSharedVertices()
    {
        var segments = Ring(0, 0, 4, 4).Concat(Ring(1, 1, 3, 3).Reverse().Select(Reverse))
            .Concat(Ring(1.5, 1.5, 2.5, 2.5)).Concat(Ring(4, 0, 5, 1)).ToArray();
        var rows = new PreparedTargetWindingRows(segments, default);
        double[] ys = [0, Math.BitIncrement(0), .25, 1, Math.BitDecrement(1), 1.5, 2, 2.5, 3, 4, 5];
        double[] xs = [-1, 0, Math.BitIncrement(0), .5, 1, 1.5, 2, 2.5, 3, 4, Math.BitDecrement(4), 5, 6];
        foreach (double y in ys)
            foreach (double x in xs)
            {
                var q = new PointD(x, y);
                Assert.True(rows.TryIsInside(q, out bool inside)); Assert.Equal(Literal(segments, q), inside);
            }
        Assert.Equal(ys.Length, rows.Misses); Assert.Equal(ys.Length * (xs.Length - 1), rows.Hits);
        Assert.True(rows.AmbiguousEvaluations > 0);
    }

    [Fact]
    public void ShuffledCrossingsAndQueriesNearRoundingIntervalsMatchOriginalExpression()
    {
        var random = new Random(774918);
        var segments = Enumerable.Range(0, 257).Select(i => new TargetWindingSegment(
            random.NextDouble() * 20 - 10, random.NextDouble() * 20 - 10,
            random.NextDouble() * 20 - 10, random.NextDouble() * 20 - 10)).ToArray();
        var rows = new PreparedTargetWindingRows(segments, default);
        for (int row = 0; row < 40; row++)
        {
            double y = row / 4d - 5;
            for (int i = 0; i < 80; i++) Compare(new(random.NextDouble() * 24 - 12, y));
            foreach (TargetWindingSegment segment in segments.Take(8))
            {
                if (!(Math.Min(segment.Ay, segment.By) <= y && y < Math.Max(segment.Ay, segment.By))) continue;
                double crossing = segment.Ax + (segment.Bx - segment.Ax) * (y - segment.Ay) / (segment.By - segment.Ay);
                Compare(new(crossing, y)); Compare(new(Math.BitIncrement(crossing), y)); Compare(new(Math.BitDecrement(crossing), y));
                Compare(new(crossing - 2e-8, y)); Compare(new(crossing + 2e-8, y));
            }
        }
        Assert.True(rows.Hits > 3000); Assert.True(rows.AmbiguousEvaluations > 0);
        void Compare(PointD q) { Assert.True(rows.TryIsInside(q, out bool inside)); Assert.Equal(Literal(segments, q), inside); }
    }

    [Fact]
    public void TinyButSupportedDeltasAndMaximumCoordinatesKeepLiteralSigns()
    {
        TargetWindingSegment[] segments = [new(-10000, -1e-98, 10000, 1e-98), new(10000, 1e-98, -10000, -1e-98),
            new(9999, -2e-100, 10000, 2e-100), new(0, 0, 1e-99, 1e-99)];
        var rows = new PreparedTargetWindingRows(segments, default);
        foreach (double y in new[] { -1e-98, -1e-100, 0, 1e-100, 1e-98 })
            foreach (double x in new[] { -10000, -1e-8, 0, 1e-99, 1e-8, 9999.5, 10000 })
            {
                var q = new PointD(x, y); Assert.True(rows.TryIsInside(q, out bool inside)); Assert.Equal(Literal(segments, q), inside);
            }
    }

    [Fact]
    public void UnsupportedRowsRefuseWithoutSupplyingWindingAuthority()
    {
        var tiny = new PreparedTargetWindingRows([new(0, 0, 1, 1e-110)], default);
        Assert.False(tiny.TryIsInside(new(.5, 5e-111), out _));
        Assert.True(tiny.TryIsInside(new(.5, 1), out bool outside)); Assert.False(outside);
        var rows = new PreparedTargetWindingRows(Ring(0, 0, 1, 1), default);
        Assert.False(rows.TryIsInside(new(double.NaN, .5), out _));
        Assert.False(rows.TryIsInside(new(.5, double.PositiveInfinity), out _));
        Assert.False(rows.TryIsInside(new(10001, .5), out _));
        var large = new PreparedTargetWindingRows([new(10001, 0, 10002, 1)], default);
        Assert.False(large.TryIsInside(new(.5, .5), out _));
    }

    [Fact]
    public void CacheEvictionAndMemoryRefusalRetainLiteralOutcomes()
    {
        var segments = Ring(0, 0, 4, 4);
        var rows = new PreparedTargetWindingRows(segments, default, maximumRows: 2, maximumBytes: 256);
        foreach (double y in new[] { .5, 1, 1.5, .5, 2, 3, 1 })
        {
            var q = new PointD(2, y); Assert.True(rows.TryIsInside(q, out bool inside)); Assert.Equal(Literal(segments, q), inside);
            Assert.InRange(rows.Rows, 0, 2); Assert.InRange(rows.CacheBytes, 0, 256);
        }
        Assert.True(rows.Evictions > 0);
        var tooSmall = new PreparedTargetWindingRows(segments, default, maximumBytes: 64);
        Assert.False(tooSmall.TryIsInside(new(2, 2), out _)); Assert.Equal(0, tooSmall.Rows);
    }

    [Fact]
    public void CancellationAndDeadlineCheckpointRemainObservable()
    {
        using var cancellation = new CancellationTokenSource();
        int checkpoints = 0;
        var rows = new PreparedTargetWindingRows(Ring(0, 0, 1, 1), cancellation.Token,
            workCheckpoint: () => checkpoints++);
        Assert.True(rows.TryIsInside(new(.5, .5), out _)); Assert.True(checkpoints > 0);
        cancellation.Cancel(); Assert.Throws<OperationCanceledException>(() => rows.TryIsInside(new(.5, .5), out _));
        Assert.Throws<OperationCanceledException>(() => new PreparedTargetWindingRows(Ring(0, 0, 1, 1), cancellation.Token));
    }

    [Fact]
    public void DefaultCacheRetainsThousandsOfDyadicRowsWithinItsByteLimit()
    {
        var rows = new PreparedTargetWindingRows(Ring(0, 0, 4, 4), default);
        for (int i = 0; i < 5000; i++)
        {
            Assert.True(rows.TryIsInside(new(2, i / 8192d), out bool inside)); Assert.True(inside);
            Assert.InRange(rows.CacheBytes, 0, 64L * 1024 * 1024);
        }
        Assert.Equal(5000, rows.Rows); Assert.Equal(0, rows.Evictions);
        for (int i = 0; i < 100; i++) Assert.True(rows.TryIsInside(new(2, i / 8192d), out _));
        Assert.Equal(100, rows.Hits); Assert.Equal(5000, rows.Misses);
    }

    private static bool Literal(TargetWindingSegment[] segments, PointD q)
    {
        int winding = 0;
        foreach (TargetWindingSegment s in segments)
        {
            double cross = (s.Bx - s.Ax) * (q.y - s.Ay) - (q.x - s.Ax) * (s.By - s.Ay);
            if (s.Ay <= q.y && s.By > q.y && cross > 0) winding++;
            else if (s.Ay > q.y && s.By <= q.y && cross < 0) winding--;
        }
        return winding != 0;
    }
    private static TargetWindingSegment[] Ring(double x0, double y0, double x1, double y1) =>
        [new(x0, y0, x1, y0), new(x1, y0, x1, y1), new(x1, y1, x0, y1), new(x0, y1, x0, y0)];
    private static TargetWindingSegment Reverse(TargetWindingSegment s) => new(s.Bx, s.By, s.Ax, s.Ay);
}
