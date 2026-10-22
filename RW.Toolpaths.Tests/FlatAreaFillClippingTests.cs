using Clipper2Lib;

namespace RW.Toolpaths.Tests;

public class FlatAreaFillClippingTests
{
    [Fact]
    public void IndexedClipsMatchOriginalPerStrokeIncludingNestedHolesAndOverlaps()
    {
        var regions = new List<List<PointD>>
        {
            Box(-20, -20, 20, 20), Box(-10, -10, 10, 10, reverse: true),
            Box(-3, -3, 3, 3), Box(25, -4, 35, 4), Box(30, -2, 40, 6),
            Box(200, 200, 220, 220)
        };
        var strokes = new List<List<PointD>>
        {
            new() { new(-30, 0), new(50, 0) }, // nested filled/empty regions
            new() { new(0, -12), new(0, -15) }, // wholly inside outer
            new() { new(-6, 0), new(-5, 1) },  // wholly inside hole
            new() { new(20, -20), new(20, 20) }, // touches boundary
            new() { new(80, 0), new(81, 0), new(81, 0), new(82, 0) },
            new() { new(50, 0), new(-30, 0) } // reverse direction
        };
        var random = new Random(914723);
        for (int i = 0; i < 200; i++)
        {
            double x = random.NextDouble() * 100 - 35;
            double y = random.NextDouble() * 50 - 25;
            strokes.Add(new() { new(x, y), new(x + random.NextDouble() * 8, y + .3) });
        }
        // Repeated clip sets exercise shared prepared data after prior executions.
        strokes.AddRange(strokes.Take(20).Select(s => s.ToList()).ToList());
        var expected = Original(strokes, regions, -6.35);
        var actual = MedialAxisToolpaths.GenerateFlatAreaFill(strokes, regions, -6.35).ToList();
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void DisjointRingsAreSkippedWithoutChangingQuantizationOrDirection()
    {
        var regions = Enumerable.Range(0, 100)
            .Select(i => Box(i * 20, 0, i * 20 + 10, 10)).ToList();
        var strokes = Enumerable.Range(0, 100)
            .Select(i => new List<PointD> { new(i * 20 - .1234567, 5.1234567), new(i * 20 + 11, 5.1234567) })
            .ToList();
        var logs = new List<string>();
        var progress = new List<double>();
        var actual = MedialAxisToolpaths.GenerateFlatAreaFill(strokes, regions, -1,
            diagnostic: logs.Add, progress: (value, _, _) => progress.Add(value)).ToList();
        Assert.Equal(Original(strokes, regions, -1), actual);
        Assert.Contains(logs, log => log.Contains("event=complete") && log.Contains("selectedVertices=400"));
        Assert.Equal(1, progress[^1]);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CancellationIsObservedAfterProgressCallback(bool emptyRegion)
    {
        using var cancel = new CancellationTokenSource();
        var strokes = new List<List<PointD>> { new() { new(-2, 0), new(2, 0) } };
        var regions = emptyRegion ? new List<List<PointD>>() : new() { Box(-1, -1, 1, 1) };
        Assert.Throws<OperationCanceledException>(() => MedialAxisToolpaths.GenerateFlatAreaFill(
            strokes, regions, -1, cancel.Token, progress: (_, _, _) => cancel.Cancel()).ToList());
    }

    private static List<PointD> Box(double left, double top, double right, double bottom, bool reverse = false)
    {
        var result = new List<PointD> { new(left, top), new(right, top), new(right, bottom), new(left, bottom) };
        if (reverse) result.Reverse();
        return result;
    }

    private static List<MedialSegment> Original(List<List<PointD>> strokes, List<List<PointD>> regions, double depth)
    {
        var clips = PathUtils.ToClipper(regions.Select(r => r.Select(p => p)));
        var result = new List<MedialSegment>();
        foreach (var stroke in strokes)
        {
            var clipper = new Clipper64();
            clipper.AddOpenSubject(PathUtils.ToClipper(new[] { stroke })[0]);
            foreach (var clip in clips) clipper.AddClip(clip);
            var open = new Paths64();
            clipper.Execute(ClipType.Difference, FillRule.NonZero, new Paths64(), open);
            foreach (var path in PathUtils.FromClipper(open))
                for (int i = 1; i < path.Count; i++)
                    result.Add(new(new(path[i - 1].x, path[i - 1].y, depth), new(path[i].x, path[i].y, depth)));
        }
        return result;
    }
}
