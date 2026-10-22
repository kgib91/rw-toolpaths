using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class OptionalSweptAreaTests
{
    [Theory]
    [InlineData(ProfileSide.Inside, false)]
    [InlineData(ProfileSide.OnLine, true)]
    public void ProfileSweptAreaIsOptionalWithoutChangingMotionOrDiagnostics(ProfileSide side, bool clipped)
    {
        var regions = new[] { new MillingRegion(Rectangle(-2, 2, 12, 8)) };
        var tool = ToolGeometry.Flat(1);
        var depth = new DepthSchedule(2, 1);
        var options = new ProfileOptions
        {
            Side = side,
            ClipBoundary = clipped ? Rectangle(0, 0, 10, 10) : null
        };
        Assert.True(options.CalculateSweptArea);

        var withArea = ProfileToolpaths.Generate(regions, tool, depth, options);
        var withoutArea = ProfileToolpaths.Generate(regions, tool, depth,
            options with { CalculateSweptArea = false });

        AssertSameMotionAndDiagnostics(withArea, withoutArea);
        if (clipped)
            Assert.All(withoutArea.Toolpaths.SelectMany(path => path.Points), point => Assert.InRange(point.X, 0, 10));
        Console.WriteLine($"[OptionalSweptArea.Profile] side={side} clipped={clipped} "
            + $"paths={withArea.Toolpaths.Count} defaultPolygons={withArea.SweptArea.Count} omittedPolygons={withoutArea.SweptArea.Count}");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PocketSweptAreaIsOptionalWithoutChangingMotionOrDiagnostics(bool scanline)
    {
        var regions = new[] { new MillingRegion(Rectangle(0, 0, 12, 8)) };
        var tool = ToolGeometry.Flat(1);
        var depth = new DepthSchedule(2, 1);
        var options = new PocketOptions { StepOver = .4 };
        Assert.True(options.CalculateSweptArea);
        ToolpathPlan Generate(PocketOptions settings) => scanline
            ? ScanlinePocketToolpaths.Generate(regions, tool, depth, settings, angleDegrees: 27)
            : PocketToolpaths.Generate(regions, tool, depth, settings);

        var withArea = Generate(options);
        var withoutArea = Generate(options with { CalculateSweptArea = false });

        AssertSameMotionAndDiagnostics(withArea, withoutArea);
        if (scanline)
        {
            Assert.Contains(withoutArea.Toolpaths, path => path.Category == ProfileToolpaths.Category);
            Assert.Contains(withoutArea.Toolpaths, path => path.Category == PocketToolpaths.Category);
        }
        Console.WriteLine($"[OptionalSweptArea.Pocket] scanline={scanline} "
            + $"paths={withArea.Toolpaths.Count} defaultPolygons={withArea.SweptArea.Count} omittedPolygons={withoutArea.SweptArea.Count}");
    }

    private static void AssertSameMotionAndDiagnostics(ToolpathPlan withArea, ToolpathPlan withoutArea)
    {
        Assert.NotEmpty(withArea.Toolpaths);
        Assert.NotEmpty(withArea.SweptArea);
        Assert.Empty(withoutArea.SweptArea);
        Assert.Equal(withArea.Diagnostics, withoutArea.Diagnostics);
        Assert.Equal(withArea.Toolpaths.Count, withoutArea.Toolpaths.Count);
        for (int index = 0; index < withArea.Toolpaths.Count; index++)
        {
            var expected = withArea.Toolpaths[index];
            var actual = withoutArea.Toolpaths[index];
            Assert.Equal(expected.Points.ToArray(), actual.Points.ToArray());
            Assert.Equal(expected.Spans.ToArray(), actual.Spans.ToArray());
            Assert.Equal(expected.Moves.ToArray(), actual.Moves.ToArray());
            Assert.Equal((expected.RegionIndex, expected.Category, expected.DepthPassIndex, expected.VisitIdentity),
                (actual.RegionIndex, actual.Category, actual.DepthPassIndex, actual.VisitIdentity));
        }
    }

    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
}
