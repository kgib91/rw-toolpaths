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
    public void GuideChordLiftBoundsInterpolatedFieldAtEverySamplingLevel(bool invert, double samplingPercent)
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
        Assert.DoesNotContain(stages, stage => stage.Contains("clearance samples", StringComparison.OrdinalIgnoreCase));
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, 12.5)]
    [InlineData(true, 12.5)]
    [InlineData(false, 90)]
    public void GuidedChordsProtectOriginalPixelsMasksAndTaperedSides(bool invert, double samplingPercent)
    {
        const int width = 17, height = 13;
        var random = new Random(61907);
        var pixels = Enumerable.Range(0, width * height).Select(_ => (float)random.NextDouble()).ToArray();
        var mask = Enumerable.Repeat(true, pixels.Length).ToArray();
        mask[6 * width + 8] = false;
        var source = new HeightmapSurface(new HeightmapGrid(width, height, pixels, mask), new(8, 6, 0, 0), 10, 5, invert);
        var cutter = new CarvingCutterProfile([
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 0.6, 0.6),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0.6, 3, 20),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 3, 3, 70)], 5);
        var plan = CarvingFinishingToolpaths.Generate(source, new ToolGeometry(0.6), true,
            new() { SamplingPercent = samplingPercent, SamplingSpacing = 0.75, StepOver = 0.6,
                ScanAngle = 37, Tolerance = 0.04, OutputRoundingMargin = 0.001 }, cutter: cutter);
        Assert.NotEmpty(plan.Toolpaths);

        // Decode original pixels and invert the physical ball/cone independently of
        // HeightmapSurface's clearance bounds, its hierarchy, and the guide field.
        double Target(double x, double y)
        {
            if (x < 0 || y < 0 || x > 8 || y > 6) return 10;
            double gx = x * 2, gy = y * 2;
            int ix = Math.Min((int)gx, width - 2), iy = Math.Min((int)gy, height - 2);
            int i = iy * width + ix;
            if (!mask[i] || !mask[i + 1] || !mask[i + width] || !mask[i + width + 1]) return 10;
            double u = gx - ix, v = gy - iy;
            double value = pixels[i] * (1 - u) * (1 - v) + pixels[i + 1] * u * (1 - v)
                + pixels[i + width] * (1 - u) * v + pixels[i + width + 1] * u * v;
            return 10 - 5 * (invert ? value : 1 - value);
        }
        double[] radii = [0, 0.2, 0.4, 0.59, 0.6, 0.75, 1, 1.2, 3];
        double maximumPenetration = double.NegativeInfinity;
        foreach (var path in plan.Toolpaths)
        for (int i = 1; i < path.Points.Count; i++)
        {
            var a = path.Points[i - 1]; var b = path.Points[i];
            int divisions = Math.Max(2, (int)Math.Ceiling(Math.Sqrt(Math.Pow(b.X - a.X, 2) + Math.Pow(b.Y - a.Y, 2)) / 0.08));
            for (int sample = 0; sample <= divisions; sample++)
            {
                double t = (double)sample / divisions;
                double x = a.X + (b.X - a.X) * t, y = a.Y + (b.Y - a.Y) * t, z = a.Z + (b.Z - a.Z) * t;
                foreach (double radius in radii)
                {
                    double bottom = radius <= 0.6 ? 0.6 - Math.Sqrt(Math.Max(0, 0.36 - radius * radius))
                        : 0.6 + (radius - 0.6) * 20 / 2.4;
                    for (int direction = 0; direction < (radius == 0 ? 1 : 24); direction++)
                    {
                        double angle = direction * Math.PI / 12;
                        maximumPenetration = Math.Max(maximumPenetration,
                            Target(x + radius * Math.Cos(angle), y + radius * Math.Sin(angle)) - z - bottom);
                    }
                }
            }
        }
        Assert.True(maximumPenetration <= 1e-7, $"Sampled physical cutter penetration: {maximumPenetration:R} mm");
    }

    [Theory]
    [InlineData(false, 2, 2, 0.1)]
    [InlineData(true, 2, 2, 0.1)]
    [InlineData(false, 3, 24, 12.5)]
    [InlineData(true, 3, 24, 12.5)]
    [InlineData(false, 17, 17, 90)]
    [InlineData(true, 17, 17, 90)]
    public void GuidedFlatAndPlainBallRetainDepthOnTinyImages(bool ball, int width, int height, double percent)
    {
        var source = new HeightmapSurface(new HeightmapGrid(width, height, new float[width * height]), new(4, 4, 0, 0), 10, 5);
        var plan = CarvingFinishingToolpaths.Generate(source, new ToolGeometry(0.5), ball,
            new() { SamplingPercent = percent, ScanAngle = 45 });
        Assert.NotEmpty(plan.Toolpaths);
        Assert.InRange(plan.Toolpaths.SelectMany(path => path.Points).Min(point => point.Z), 5, 5.02);
    }

    [Fact]
    public void SampledGuideReachesFirstContactBesideRaisedWall()
    {
        var pixels = new float[401 * 161];
        for (int y = 0; y < 161; y++) for (int x = 218; x < 401; x++) pixels[y * 401 + x] = 1;
        var source = new HeightmapSurface(new HeightmapGrid(401, 161, pixels), new(20, 8, 0, 0), 10, 10);
        var cutter = new CarvingCutterProfile([
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 0.6, 0.6),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0.6, 3, 20)], 10);
        var plan = CarvingFinishingToolpaths.Generate(source, new ToolGeometry(0.6), true,
            new() { SamplingPercent = 90, StepOver = 0.21, Tolerance = 0.017 }, cutter: cutter);
        int checkedChords = 0;
        foreach (var path in plan.Toolpaths)
        for (int i = 1; i < path.Points.Count; i++)
        {
            var a = path.Points[i - 1]; var b = path.Points[i];
            if (a.Y < 2 || a.Y > 6 || Math.Min(a.X, b.X) > 10 || Math.Max(a.X, b.X) < 10) continue;
            double t = (10 - a.X) / (b.X - a.X);
            Assert.InRange(a.Z + (b.Z - a.Z) * t, 6.9, 7);
            checkedChords++;
        }
        Assert.True(checkedChords > 0);
    }

    [Theory]
    [InlineData(12.5)]
    [InlineData(90)]
    public void PlainBallGuideUsesSphericalContactOnPlanarSlope(double percent)
    {
        var pixels = new float[65 * 65];
        for (int y = 0; y < 65; y++) for (int x = 0; x < 65; x++) pixels[y * 65 + x] = x / 64f;
        var source = new HeightmapSurface(new HeightmapGrid(65, 65, pixels), new(8, 8, 0, 0), 10, 4);
        var plan = CarvingFinishingToolpaths.Generate(source, new ToolGeometry(0.5), true,
            new() { SamplingPercent = percent, ScanAngle = 37, Tolerance = 0.02 });
        // Support of z=6+0.5x by a radius0.5 ball. Treating the ball as a flat
        // disk would instead add0.25mm and fail this geometric contact check.
        double sphericalLift = 0.5 * (Math.Sqrt(1.25) - 1);
        int checkedChords = 0;
        foreach (var path in plan.Toolpaths)
        for (int i = 1; i < path.Points.Count; i++)
        {
            var a = path.Points[i - 1]; var b = path.Points[i];
            if (Math.Min(a.X, b.X) > 4 || Math.Max(a.X, b.X) < 4 || Math.Abs(b.X - a.X) < 1e-12) continue;
            double t = (4 - a.X) / (b.X - a.X), y = a.Y + (b.Y - a.Y) * t;
            if (y < 2 || y > 6) continue;
            Assert.InRange(a.Z + (b.Z - a.Z) * t, 8 + sphericalLift, 8 + sphericalLift + 0.02);
            checkedChords++;
        }
        Assert.True(checkedChords > 0);
    }

    [Fact]
    public void GuideThenSourceProofProgressIsMonotonicAndCancelable()
    {
        var source = new HeightmapSurface(new HeightmapGrid(17, 17, new float[17 * 17]), new(4, 4, 0, 0), 10, 5);
        var events = new List<(double Fraction, string Stage)>();
        var options = new CarvingOptions { SamplingPercent = 90 };
        var plan = CarvingFinishingToolpaths.Generate(source, new ToolGeometry(0.5), true, options,
            progress: (fraction, stage) => events.Add((fraction, stage)));
        Assert.NotEmpty(plan.Toolpaths);
        Assert.Contains(events, e => e.Stage.Contains("clearance samples") && Math.Abs(e.Fraction - 0.15) < 1e-9);
        Assert.Contains(events, e => e.Stage.Contains("scanlines") && e.Fraction > 0.15);
        Assert.Equal(1, events[^1].Fraction);
        for (int i = 1; i < events.Count; i++) Assert.True(events[i].Fraction >= events[i - 1].Fraction);
        using var cancellation = new CancellationTokenSource();
        Assert.ThrowsAny<OperationCanceledException>(() => CarvingFinishingToolpaths.Generate(source,
            new ToolGeometry(0.5), true, options, cancellation.Token,
            (fraction, _) => { if (fraction >= 0.03) cancellation.Cancel(); }));
    }
}

