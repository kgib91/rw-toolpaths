using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class SignedEdgeOffsetTests
{
    private static List<PointD> Rect(double min, double max) =>
        [new(min, min), new(max, min), new(max, max), new(min, max)];

    [Theory]
    [InlineData(ProfileSide.Outside, 1, 16, 84)]
    [InlineData(ProfileSide.Outside, -1, 18, 82)]
    [InlineData(ProfileSide.Outside, -3, 20, 80)]
    [InlineData(ProfileSide.Outside, -4, 21, 79)]
    [InlineData(ProfileSide.Inside, 1, 24, 76)]
    [InlineData(ProfileSide.Inside, -1, 22, 78)]
    [InlineData(ProfileSide.Inside, -3, 20, 80)]
    [InlineData(ProfileSide.Inside, -4, 19, 81)]
    public void ProfileOffsetsCenterlineAfterCutterCompensation(ProfileSide side, double offset, double min, double max)
    {
        var plan = ProfileToolpaths.Generate([new MillingRegion(Rect(20, 80))], ToolGeometry.Flat(3),
            new DepthSchedule(2, 2), new ProfileOptions { Side = side, StockToLeave = offset, Ramp = RampSettings.Plunge });
        var points = plan.Toolpaths.SelectMany(p => p.Points).ToArray();
        Assert.NotEmpty(points);
        Assert.Equal(min, points.Min(p => p.X), 3);
        Assert.Equal(max, points.Max(p => p.X), 3);
        Assert.Equal(min, points.Min(p => p.Y), 3);
        Assert.Equal(max, points.Max(p => p.Y), 3);
    }

    [Theory]
    [InlineData(false, 1, 4, 56)]
    [InlineData(false, -1, 2, 58)]
    [InlineData(false, -3, 0, 60)]
    [InlineData(false, -4, -1, 61)]
    [InlineData(true, 1, 4, 56)]
    [InlineData(true, -1, 2, 58)]
    [InlineData(true, -3, 0, 60)]
    [InlineData(true, -4, -1, 61)]
    public void PocketStartsAtSignedOffsetForBothStrategies(bool scanline, double offset, double min, double max)
    {
        var regions = new[] { new MillingRegion(Rect(0, 60)) };
        var tool = ToolGeometry.Flat(3);
        var depth = new DepthSchedule(2, 2);
        var options = new PocketOptions { StockToLeave = offset, Ramp = RampSettings.Plunge };
        var plan = scanline ? ScanlinePocketToolpaths.Generate(regions, tool, depth, options)
            : PocketToolpaths.Generate(regions, tool, depth, options);
        var points = plan.Toolpaths.SelectMany(p => p.Points).ToArray();
        Assert.NotEmpty(points);
        Assert.Equal(min, points.Min(p => p.X), 3);
        Assert.Equal(max, points.Max(p => p.X), 3);
        Assert.Equal(min, points.Min(p => p.Y), 3);
        Assert.Equal(max, points.Max(p => p.Y), 3);
    }

    [Theory]
    [InlineData(ProfileSide.Outside, -4, 19, 41)]
    [InlineData(ProfileSide.Inside, -4, 21, 39)]
    public void CrossingSourceBoundaryPreservesHolePolarity(ProfileSide side, double offset, double min, double max)
    {
        var prepared = ProfileToolpaths.Prepare([new MillingRegion(Rect(0, 60), [Rect(20, 40)])],
            ToolGeometry.Flat(3), new ProfileOptions { Side = side, StockToLeave = offset });
        Assert.Equal(2, prepared.Rings.Count);
        var hole = Assert.Single(prepared.Rings, ring => ring.IsHole);
        Assert.Equal(min, hole.Points.Min(p => p.x), 3);
        Assert.Equal(max, hole.Points.Max(p => p.x), 3);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void OnLineRejectsNonzeroOffset(double offset)
    {
        Assert.Throws<ArgumentException>(() => ProfileToolpaths.Prepare([new MillingRegion(Rect(0, 60))],
            ToolGeometry.Flat(3), new ProfileOptions { Side = ProfileSide.OnLine, StockToLeave = offset }));
    }

    [Fact]
    public void InsideStockEdgeBypassUsesSignedInset()
    {
        var prepared = ProfileToolpaths.Prepare([new MillingRegion(Rect(0, 60))], ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.Inside, StockToLeave = -1, StockOutline = Rect(0, 60) });
        Assert.True(prepared.EdgeBypassCount > 0);
        var points = prepared.Rings.SelectMany(r => r.Points).ToArray();
        Assert.Equal(-3, points.Min(p => p.x), 3);
        Assert.Equal(63, points.Max(p => p.x), 3);
    }
}
