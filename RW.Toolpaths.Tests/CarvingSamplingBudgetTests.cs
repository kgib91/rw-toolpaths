using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class CarvingSamplingBudgetTests
{
    [Theory]
    [InlineData(98, 2008)]
    [InlineData(99.99, 2048)]
    public void AcceptedTwoKImageGeneratesAtHighSamplingPercent(double percent, int expectedRows)
    {
        const int pixels = 2048;
        var source = new HeightmapSurface(new HeightmapGrid(pixels, pixels, new float[pixels * pixels]),
            new(8, 8, 0, 0), 10, 5);
        var stages = new List<string>();
        var plan = CarvingFinishingToolpaths.Generate(source, new ToolGeometry(1), false,
            new() { SamplingPercent = percent, StepOver = .5 },
            progress: (_, stage) => stages.Add(stage));

        Assert.Contains($"Cutter clearance samples {expectedRows:N0}/{expectedRows:N0}", stages);
        Assert.NotEmpty(plan.Toolpaths);
        Assert.InRange(plan.Toolpaths.SelectMany(path => path.Points).Min(point => point.Z), 5, 5.02);
        Assert.All(plan.Toolpaths.SelectMany(path => path.Points), point => Assert.True(double.IsFinite(point.Z)));
    }

    [Fact]
    public void ExcessiveLegacySpacingRequestStillFailsBeforeAllocatingOrSampling()
    {
        var source = new HeightmapSurface(new HeightmapGrid(2, 2, new float[4]), new(8, 8, 0, 0), 10, 5);
        var stages = new List<string>();
        var error = Assert.Throws<InvalidOperationException>(() => new CarvingClearanceField(source, .5, false, null,
            new() { SamplingSpacing = 8d / 2048 }, default, (_, stage) => stages.Add(stage)));
        Assert.Contains("Sampling Spacing", error.Message);
        Assert.Empty(stages);
    }
}
