using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class CarvingPatchSectionTests
{
    [Theory]
    [InlineData("narrowing", 0.9, 0)]
    [InlineData("narrowing", 0.9, 0.03)]
    [InlineData("wide-shaft", 0.8, 0)]
    [InlineData("wide-shaft", 0.8, 0.03)]
    [InlineData("partial-ball", 0.12, 0)]
    [InlineData("partial-ball", 0.12, 0.03)]
    public void SectionBoundsProtectIndependentPhysicalSamplesOnNegativeChordsAndRectangles(
        string shape, double contactHeight, double margin)
    {
        var sections = shape switch
        {
            "narrowing" => new[] {
                new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 0.4, 0.4),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.1, 0.2, 2) },
            "wide-shaft" => new[] {
                new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 0.35, 0.35),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0.35, 0.35, 0.15),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.6, 1.6, 5) },
            _ => new[] { new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 0.6, 0.6) }
        };
        var cutter = new CarvingCutterProfile(sections, contactHeight);

        // These are physical cross-sections, derived independently of Bottom,
        // Support, the new dual bounds, and segment/rectangle distance helpers.
        double PhysicalBottom(double radius)
        {
            if (shape == "partial-ball")
                return radius <= 0.6 ? 0.6 - Math.Sqrt(Math.Max(0, 0.36 - radius * radius)) : double.PositiveInfinity;
            double tip = shape == "narrowing" ? 0.4 : 0.35;
            if (radius <= tip) return tip - Math.Sqrt(Math.Max(0, tip * tip - radius * radius));
            if (shape == "narrowing") return radius <= 1.1 ? 0.4 : double.PositiveInfinity;
            return radius <= 1.6 ? 0.5 : double.PositiveInfinity;
        }

        // Both XY coordinates decrease while Z increases. The patch has a saddle,
        // so checking only a source-gradient plane would not reproduce its heights.
        const double ax = 100.2, ay = 300.1, az = 0.2;
        const double bx = 99.6, by = 299.7, bz = 0.7;
        double offset = shape == "wide-shaft" ? 1.15 : shape == "narrowing" ? 0.8 : 0.22;
        double x0 = (ax + bx) / 2 + offset - 0.15, x1 = x0 + 0.3;
        double y0 = (ay + by) / 2 - 0.12, y1 = y0 + 0.24;
        const double h00 = 1.4, h10 = 0.9, h01 = 1.1, h11 = 1.8;
        foreach (bool rectangle in new[] { false, true })
        {
            double bound = cutter.PatchUpperBound(x0, y0, x1, y1, h00, h10, h01, h11,
                ax, ay, az, bx, by, bz, margin, queryRectangle: rectangle);
            double reversed = cutter.PatchUpperBound(x0, y0, x1, y1, h00, h10, h01, h11,
                bx, by, bz, ax, ay, az, margin, queryRectangle: rectangle);
            Assert.True(double.IsFinite(bound), $"No finite bound for contacting {shape}, rectangle={rectangle}");
            Assert.InRange(Math.Abs(bound - reversed), 0, 1e-7);

            int contacts = 0;
            double maximumResidual = double.NegativeInfinity;
            for (int iy = 0; iy <= 12; iy++)
            for (int ix = 0; ix <= 12; ix++)
            {
                double u = ix / 12d, v = iy / 12d;
                double x = x0 + (x1 - x0) * u, y = y0 + (y1 - y0) * v;
                double height = h00 * (1 - u) * (1 - v) + h10 * u * (1 - v)
                    + h01 * (1 - u) * v + h11 * u * v;
                void Contact(double cx, double cy, double z)
                {
                    // Independent nearest point in the square rounding footprint.
                    double dx = Math.Max(0, Math.Abs(x - cx) - margin);
                    double dy = Math.Max(0, Math.Abs(y - cy) - margin);
                    double bottom = PhysicalBottom(Math.Sqrt(dx * dx + dy * dy));
                    if (bottom > contactHeight) return;
                    contacts++;
                    maximumResidual = Math.Max(maximumResidual, height - z - bottom);
                }
                if (rectangle)
                {
                    for (int cy = 0; cy <= 8; cy++)
                    for (int cx = 0; cx <= 8; cx++)
                        Contact(bx + (ax - bx) * cx / 8, by + (ay - by) * cy / 8, Math.Min(az, bz));
                }
                else
                {
                    for (int sample = 0; sample <= 24; sample++)
                    {
                        double t = sample / 24d;
                        Contact(ax + (bx - ax) * t, ay + (by - ay) * t, az + (bz - az) * t);
                    }
                }
            }
            Assert.True(contacts > 0, $"The {shape} case must exercise actual physical contact.");
            Assert.True(maximumResidual > 0, $"The {shape} case must contain material that needs clearance.");
            Assert.True(bound + 1e-7 >= maximumResidual,
                $"{shape}, rectangle={rectangle}, margin={margin:R}: bound={bound:R}, physical residual={maximumResidual:R}");
        }
    }

    [Fact]
    public void SquareMarginCornerContactSurvivesSegmentDistanceClipping()
    {
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0.1, 0.1, 1) }, 0.5);
        const double x0 = 0.17, y0 = 0.17, x1 = 0.19, y1 = 0.19;
        double unrounded = cutter.PatchUpperBound(x0, y0, x1, y1, 0.8, 0.9, 1.0, 0.7,
            0, 0, 0.2, -0.2, -0.2, 0.4, 0);
        double rounded = cutter.PatchUpperBound(x0, y0, x1, y1, 0.8, 0.9, 1.0, 0.7,
            0, 0, 0.2, -0.2, -0.2, 0.4, 0.1);

        // Nominally the nearest point is >0.24mm from the radius0.1 cutter.
        // Only the (+0.1,+0.1) square corner reaches the patch: distance<0.1.
        Assert.True(Math.Sqrt(x0 * x0 + y0 * y0) > 0.1 + 0.1);
        Assert.True(Math.Sqrt(Math.Pow(x0 - 0.1, 2) + Math.Pow(y0 - 0.1, 2)) < 0.1);
        Assert.True(double.IsNegativeInfinity(unrounded));
        Assert.True(double.IsFinite(rounded));
        Assert.True(rounded + 1e-7 >= 0.8 - 0.2);
    }

    [Fact]
    public void PartialHeightClippingIncludesBallContactButExcludesUnreachableWideShaft()
    {
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 0.6, 0.6),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.6, 1.6, 5) }, 0.12);
        double contacting = cutter.PatchUpperBound(0.34, -0.01, 0.35, 0.01, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0);
        double unreachable = cutter.PatchUpperBound(0.37, -0.01, 0.38, 0.01, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0);
        double physicalBottom = 0.6 - Math.Sqrt(0.36 - 0.34 * 0.34);
        Assert.InRange(physicalBottom, 0, 0.12);
        Assert.True(double.IsFinite(contacting));
        Assert.True(contacting + 1e-7 >= 1 - physicalBottom);
        Assert.True(double.IsNegativeInfinity(unreachable));
    }
}
