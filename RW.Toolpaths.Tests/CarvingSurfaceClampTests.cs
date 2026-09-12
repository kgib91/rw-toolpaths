using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class CarvingSurfaceClampTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EntirelyWhiteOrMaskedImageStillProducesNoMachining(bool masked)
    {
        var source = new HeightmapSurface(new HeightmapGrid(2, 2,
            masked ? new float[4] : Enumerable.Repeat(1f, 4).ToArray(),
            masked ? new bool[4] : null), new(6, 6, 0, 0), 10, 4);
        var plan = CarvingFinishingToolpaths.Generate(source, new(.6), true,
            new() { SamplingPercent = 90 });
        Assert.Empty(plan.Toolpaths);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TopClippingPreservesTheProvenSlopeUntilItsActualIntersection(bool reverse)
    {
        Point3D[] source = [new(0, 2, 6), new(4, 6, 12), new(8, 10, 12), new(12, 14, 6)];
        if (reverse) Array.Reverse(source);
        var clipped = CarvingFinishingToolpaths.ClampRasterToSurface(source, 10, out int clamped, out int intersections);
        Assert.Equal(2, clamped);
        Assert.Equal(2, intersections);
        Assert.Equal(6, clipped.Length);
        // Independent evaluation of the original piecewise-linear envelope. Merely
        // clamping the two high vertices would undercut each approaching slope.
        for (int i = 1; i < clipped.Length; i++)
        for (int step = 0; step <= 20; step++)
        {
            double t = step / 20d;
            double x = clipped[i - 1].X * (1 - t) + clipped[i].X * t;
            double y = clipped[i - 1].Y * (1 - t) + clipped[i].Y * t;
            double z = clipped[i - 1].Z * (1 - t) + clipped[i].Z * t;
            double original = x < 4 ? 6 + 1.5 * x : x <= 8 ? 12 : 12 - 1.5 * (x - 8);
            Assert.Equal(x + 2, y, 10);
            Assert.Equal(Math.Min(10, original), z, 10);
        }
    }
}
