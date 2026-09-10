using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class CarvingCutterProfileTests
{
    [Theory]
    [InlineData(20, 6.35)]
    [InlineData(1, 6.35)]
    [InlineData(20, 0.3)]
    public void ClippedSupportsAndPatchPlanesBoundIndependentPhysicalSamples(double taper, double depth)
    {
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 0.6, 0.6),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0.6, 3, taper),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 3, 3, 70) }, depth);
        var random = new Random(27492);
        double Bottom(double r) => r <= 0.6 ? 0.6 - Math.Sqrt(Math.Max(0, 0.36 - r * r))
            : r <= 3 ? 0.6 + taper * (r - 0.6) / 2.4 : double.PositiveInfinity;
        for (int iteration = 0; iteration < 180; iteration++)
        {
            double ax = 100, ay = 300, bx = ax + random.NextDouble() * 0.2, by = ay + random.NextDouble() * 0.2;
            double x0 = ax + (random.NextDouble() - 0.5) * 3, y0 = ay + (random.NextDouble() - 0.5) * 3;
            double x1 = x0 + 0.01 + random.NextDouble(), y1 = y0 + 0.01 + random.NextDouble();
            double gx = iteration == 0 ? 0 : 50 * (random.NextDouble() - 0.5), gy = iteration == 0 ? 0 : 50 * (random.NextDouble() - 0.5);
            double clipped = cutter.ClippedPlaneSupport(gx, gy, x0, y0, x1, y1, ax, ay, bx, by, out _, out _);
            double h00 = random.NextDouble() * 10, h10 = random.NextDouble() * 10;
            double h01 = random.NextDouble() * 10, h11 = random.NextDouble() * 10;
            double bound = cutter.PatchUpperBound(x0, y0, x1, y1, h00, h10, h01, h11,
                ax, ay, 0, bx, by, 0, 0, queryRectangle: true);
            double az = random.NextDouble(), bz = random.NextDouble();
            double chord = cutter.PatchUpperBound(x0, y0, x1, y1, h00, h10, h01, h11,
                ax, ay, az, bx, by, bz, 0.0011);
            for (int iy = 0; iy <= 24; iy++)
            for (int ix = 0; ix <= 24; ix++)
            {
                double tx = ix / 24d, ty = iy / 24d;
                double x = x0 + (x1 - x0) * tx, y = y0 + (y1 - y0) * ty;
                double dx = x - Math.Clamp(x, ax, bx), dy = y - Math.Clamp(y, ay, by);
                double height = (h00 * (1 - tx) + h10 * tx) * (1 - ty) + (h01 * (1 - tx) + h11 * tx) * ty;
                double bottom = Bottom(Math.Sqrt(dx * dx + dy * dy));
                if (bottom <= depth)
                {
                    Assert.True(clipped + 1e-7 >= gx * x + gy * y - bottom, $"Clipped support undercut iteration={iteration}");
                    Assert.True(bound + 1e-7 >= height - bottom, $"Patch support undercut iteration={iteration}");
                }
                for (int t = 0; t <= 4; t++)
                {
                    double u = t / 4d, cx = ax * (1 - u) + bx * u, cy = ay * (1 - u) + by * u;
                    dx = Math.Max(0, Math.Abs(x - cx) - 0.0011); dy = Math.Max(0, Math.Abs(y - cy) - 0.0011);
                    bottom = Bottom(Math.Sqrt(dx * dx + dy * dy));
                    if (bottom <= depth)
                        Assert.True(chord + 1e-7 >= height - bottom - (az * (1 - u) + bz * u), $"Chord support undercut iteration={iteration}");
                }
            }
        }
    }
}
