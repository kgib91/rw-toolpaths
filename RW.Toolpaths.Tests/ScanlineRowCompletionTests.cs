using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class ScanlineRowCompletionTests
{
    [Theory]
    [InlineData(5)]
    [InlineData(3)]
    public void IndependentRowsCompleteTheirOwnStepdownsBeforeDeparting(int passes)
    {
        var region = new MillingRegion(new List<PointD> { new(0, 0), new(80, 0), new(80, 40), new(0, 40) });
        var plan = ScanlinePocketToolpaths.Generate([region], ToolGeometry.Flat(1),
            new DepthSchedule(2 * passes, 2), new PocketOptions {
                IndependentRasterEntries = true, StepOver = .6, CalculateSweptArea = false,
                DetectCurves = false, Ramp = RampSettings.FromRatio(2) });
        var rows = plan.Toolpaths.Where(p => p.Points.Count >= 2
            && p.Points.All(v => v.Y == p.Points[0].Y)).ToArray();
        Assert.True(rows.Length > 2 * passes);
        for (int first = 0; first < rows.Length; first += passes)
        {
            var block = rows.Skip(first).Take(passes).ToArray();
            Assert.Equal(Enumerable.Range(0, passes), block.Select(p => p.DepthPassIndex!.Value));
            Assert.All(block, p => Assert.Equal(block[0].Points[0].Y, p.Points[0].Y));
            for (int pass = 1; pass < passes; pass++)
                Assert.Equal(block[pass - 1].Points[^1], block[pass].Points[0]);
        }
    }
}
