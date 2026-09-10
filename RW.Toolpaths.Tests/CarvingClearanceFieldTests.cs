using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class CarvingClearanceFieldTests
{
    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, 12.5)]
    [InlineData(true, 12.5)]
    [InlineData(false, 50)]
    public void CoarseFieldBoundsOriginalPeaksMasksAndContinuousAngledChords(bool invert, double samplingPercent)
    {
        var random = new Random(61907);
        var pixels = Enumerable.Range(0, 17 * 13).Select(_ => (float)random.NextDouble()).ToArray();
        var mask = Enumerable.Repeat(true, pixels.Length).ToArray(); mask[6 * 17 + 8] = false;
        var source = new HeightmapSurface(new HeightmapGrid(17, 13, pixels, mask), new(8, 6, 0, 0), 10, 5, invert);
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 0.6, 0.6),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0.6, 3, 20) }, 5);
        var field = new CarvingClearanceField(source, cutter.MaximumRadius, true, cutter,
            new() { SamplingSpacing = 0.75, SamplingPercent = samplingPercent, Tolerance = 0.02, OutputRoundingMargin = 0.001 }, default, null);
        for (int i = 0; i < 200; i++)
        {
            var a = new PointD(random.NextDouble() * 8, random.NextDouble() * 6);
            double height = field.Sample(a);
            for (int r = 0; r <= 8; r++) for (int direction = 0; direction < 24; direction++)
            {
                double radius = r * 1.2 / 8, angle = direction * Math.PI / 12;
                double bottom = radius <= 0.6 ? 0.6 - Math.Sqrt(Math.Max(0, 0.36 - radius * radius))
                    : 0.6 + 20 * (radius - 0.6) / 2.4;
                Assert.True(height + bottom + 1e-8 >= source.Sample(a.x + radius * Math.Cos(angle), a.y + radius * Math.Sin(angle)));
            }
            var b = new PointD(random.NextDouble() * 8, random.NextDouble() * 6);
            double end = field.Sample(b), lift = field.ChordLift(a, height, b, end, 20), sampled = 0;
            for (int t = 0; t <= 1000; t++)
            {
                double u = t / 1000d;
                sampled = Math.Max(sampled, field.Sample(new(a.x * (1 - u) + b.x * u, a.y * (1 - u) + b.y * u))
                    - height * (1 - u) - end * u);
            }
            Assert.InRange(lift - sampled, -1e-8, 0.03);
        }
    }
    [Theory]
    [InlineData(128, 192, 12.5, 16, 24, 12, 4)]
    [InlineData(128, 192, 12.5, 16, 24, 4, 12)]
    [InlineData(17, 13, 12.5, 3, 2, 8, 6)]
    [InlineData(2, 3, 0.1, 2, 2, 8, 6)]
    public void PercentScalesBothSourceDimensionsIndependentlyOfPartAspect(int width, int height,
        double percent, int fieldWidth, int fieldHeight, double partWidth, double partHeight)
    {
        var pixels = Enumerable.Repeat(0.5f, width * height).ToArray();
        var source = new HeightmapSurface(new HeightmapGrid(width, height, pixels),
            new(partWidth, partHeight, 0, 0), 10, 5);
        var field = new CarvingClearanceField(source, 0.1, false, null,
            new() { SamplingPercent = percent, SamplingSpacing = 100 }, default, null);
        Assert.Equal(fieldWidth, field.Width);
        Assert.Equal(fieldHeight, field.Height);
        Assert.True(field.Sample(new(partWidth / 2, partHeight / 2)) >= source.Sample(partWidth / 2, partHeight / 2));
        Assert.True(double.IsFinite(field.Sample(new(partWidth, partHeight))));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(100.1)]
    [InlineData(double.NaN)]
    public void InvalidSamplingPercentIsRejected(double percent)
    {
        var source = new HeightmapSurface(new HeightmapGrid(2, 2, new float[] { 0, 0, 0, 0 }), new(4, 4, 0, 0), 10, 5);
        var error = Assert.Throws<ArgumentException>(() => CarvingFinishingToolpaths.Generate(source,
            new ToolGeometry(0.5), false, new() { SamplingPercent = percent }));
        Assert.Contains("percent", error.Message);
    }

    [Theory]
    [InlineData(2, 2)]
    [InlineData(3, 3)]
    [InlineData(3, 24)]
    [InlineData(24, 3)]
    [InlineData(17, 17)]
    [InlineData(32, 32)]
    public void MinimumPercentFieldRetainsFlatReliefDepth(int width, int height)
    {
        var source = new HeightmapSurface(new HeightmapGrid(width, height, new float[width * height]),
            new(4, 4, 0, 0), 10, 5);
        var plan = CarvingFinishingToolpaths.Generate(source, new ToolGeometry(0.5), false,
            new() { SamplingPercent = 12.5 });
        Assert.NotEmpty(plan.Toolpaths);
        double minimum = plan.Toolpaths.SelectMany(path => path.Points).Min(point => point.Z);
        Assert.InRange(minimum, source.Minimum - 1e-8, source.Minimum + 0.02);
    }

    [Fact]
    public void FullPercentUsesExactSourcePathAndOverridesLegacySpacing()
    {
        var source = new HeightmapSurface(new HeightmapGrid(2, 2, new float[] { 0, 0, 0, 0 }), new(4, 4, 0, 0), 10, 5);
        var stages = new List<string>();
        var plan = CarvingFinishingToolpaths.Generate(source, new ToolGeometry(0.5), false,
            new() { SamplingPercent = 100, SamplingSpacing = double.Epsilon },
            progress: (_, stage) => stages.Add(stage));
        Assert.NotEmpty(plan.Toolpaths);
        Assert.DoesNotContain(stages, stage => stage.Contains("clearance field", StringComparison.OrdinalIgnoreCase));
    }
}
