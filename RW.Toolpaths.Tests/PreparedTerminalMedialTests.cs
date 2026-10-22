using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PreparedTerminalMedialTests
{
    [Fact]
    public void AnEmptyTerminalCoreNeverRequestsTheOriginalDiagram()
    {
        var region = new MillingRegion(Rectangle(0, 0, 4, 40));
        var context = new CollapsedOffsetComponent(region, region, 0, 0, 3, 9, 3);
        var paths = OffsetTerminalPaths.BuildCollapsedComponent(context,
            () => throw new InvalidOperationException("A fully swept component requested the medial graph."),
            3, null, CancellationToken.None);
        Assert.Empty(paths);
    }

    [Fact]
    public void IndexedCandidatesMatchACompleteIntegerBoundsScanAndNativeOrdering()
    {
        var segments = Enumerable.Range(0, 12345).Select(index =>
        {
            double x = (index * 37 % 131) - 50.00001;
            double y = (index * 17 % 97) - 20.00001;
            return new MedialSegment(new(x, y, 1), new(x + (index % 9) - 4, y + (index % 5) - 2, 1));
        }).ToArray();
        var region = new MillingRegion(Rectangle(-100, -100, 100, 100));
        var prepared = new PreparedTerminalMedial(region, CancellationToken.None, segments);
        foreach (var rectangle in new[] { Rectangle(-1, -1, 3, 7), Rectangle(-50, -20, -49, -19),
            Rectangle(-100, -100, 100, 100), Rectangle(120, 120, 130, 130) })
        {
            var clip = PathUtils.ToClipper(new[] { rectangle });
            var bounds = Clipper.GetBounds(clip);
            var expected = segments.Where(segment =>
            {
                long ax = (long)Math.Round(segment.Point0.X * PathUtils.Scale);
                long ay = (long)Math.Round(segment.Point0.Y * PathUtils.Scale);
                long bx = (long)Math.Round(segment.Point1.X * PathUtils.Scale);
                long by = (long)Math.Round(segment.Point1.Y * PathUtils.Scale);
                return Math.Min(ax, bx) <= bounds.right && Math.Max(ax, bx) >= bounds.left
                    && Math.Min(ay, by) <= bounds.bottom && Math.Max(ay, by) >= bounds.top;
            });
            Assert.Equal(expected, prepared.Intersecting(clip, CancellationToken.None));
        }
    }

    [Fact]
    public void CandidateInsetHasNoGeometricEffectOnAFixedCollapsedComponent()
    {
        var source = new MillingRegion(Rectangle(0, 0, 16, 60));
        var current = new MillingRegion(Rectangle(3, 3, 13, 57));
        var prepared = new PreparedTerminalMedial(source, CancellationToken.None,
            new[] { new MedialSegment(new(8, 8, 8), new(8, 52, 8)) });
        var first = new CollapsedOffsetComponent(source, current, 0, 0, 3, 9, 3);
        var a = OffsetTerminalPaths.BuildCollapsedComponent(first, () => prepared, 3, null, CancellationToken.None);
        var b = OffsetTerminalPaths.BuildCollapsedComponent(first with { CandidateOffsetDistance = 17 },
            () => prepared, 3, null, CancellationToken.None);
        Assert.Single(a);
        Assert.Single(b);
        Assert.Equal(a.SelectMany(path => path), b.SelectMany(path => path));
    }

    private static IReadOnlyList<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => new PointD[] { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
}
