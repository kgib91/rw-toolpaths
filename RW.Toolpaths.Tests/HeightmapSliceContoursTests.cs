using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class HeightmapSliceContoursTests
{
    [Theory]
    [InlineData(0, false, false)]
    [InlineData(37, true, false)]
    [InlineData(90, false, true)]
    public void TracedSlicesBoundBilinearSaddlesAndPreserveMaskedVoids(double angle, bool mirror, bool invert)
    {
        var random = new Random(83491);
        var values = Enumerable.Range(0, 17 * 13).Select(_ => (float)random.NextDouble()).ToArray();
        var mask = Enumerable.Repeat(true, values.Length).ToArray();
        mask[6 * 17 + 8] = false;
        var placement = new HeightmapPlacement(24, 12, 3, 7, angle, mirror);
        var surface = new HeightmapSurface(new HeightmapGrid(17, 13, values, mask), placement, 10, 5, invert);
        Assert.True(HeightmapSliceContours.TryBuild(surface, 7.5, 0.02, default, out var rings, out _));
        Assert.NotEmpty(rings);
        var scaled = Clipper.ScalePaths64(rings, 1_000_000);
        for (int iy = 1; iy < 80; iy++)
        for (int ix = 1; ix < 120; ix++)
        {
            double x = ix * 24d / 120, y = iy * 12d / 80;
            var world = placement.ToWorld(x, y);
            var point = new Point64((long)Math.Round(world.x * 1_000_000), (long)Math.Round(world.y * 1_000_000));
            int winding = 0;
            foreach (var ring in scaled)
                if (Clipper.PointInPolygon(point, ring) != PointInPolygonResult.IsOutside)
                    winding += Clipper.IsPositive(ring) ? 1 : -1;
            double target = surface.Sample(x, y);
            if (winding != 0) Assert.True(target <= 7.50002, $"Slice crosses target at ({x},{y}): {target}");
            if (target < 7.48) Assert.NotEqual(0, winding);
        }
    }

    [Fact]
    public void ConstantAndThresholdAlignedSamplesCloseWithoutFragmentFallback()
    {
        foreach (var values in new[] { new float[] { 0, 0, 0, 0 }, new float[] { 0, 1, 1, 0 }, new float[] { 0.5f, 1, 0, 0.5f } })
        {
            var surface = new HeightmapSurface(new HeightmapGrid(2, 2, values), new(10, 10, 0, 0), 10, 5);
            Assert.True(HeightmapSliceContours.TryBuild(surface, 7.5, 0.02, default, out var rings, out _));
            Assert.NotEmpty(rings);
        }
    }
}
