using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class CarvingExteriorChordTests
{
    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(3, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    [InlineData(3, true)]
    public void SlopedTaperChordCertifiesExteriorClearanceWithoutMicroscopicSubdivision(int edge, bool reverse)
    {
        const double surfaceZ = 10, margin = 0.0011, tolerance = 0.008, clearance = 0.003;
        var source = new HeightmapSurface(new HeightmapGrid(2, 2, new float[4]), new(10, 10, 0, 0), surfaceZ, 6.35);
        var cutter = new CarvingCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 0.6, 0.6),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0.6, 3, 20),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 3, 3, 70) }, 6.35);
        PointD Point(double distance, double along) => edge switch
        {
            0 => new(distance + margin, along),
            1 => new(10 - distance - margin, along),
            2 => new(along, distance + margin),
            _ => new(along, 10 - distance - margin)
        };
        // Independent physical taper inverse: these two distances both contact the
        // conical wall. Tip Z changes by 3.33 mm while exterior clearance stays fixed.
        double Bottom(double radius) => radius <= 0.6
            ? 0.6 - Math.Sqrt(Math.Max(0, 0.36 - radius * radius))
            : 0.6 + 20 * (radius - 0.6) / 2.4;
        var a = Point(0.75, 4); var b = Point(1.15, 4.8);
        double za = surfaceZ - Bottom(0.75) + clearance;
        double zb = surfaceZ - Bottom(1.15) + clearance;
        if (reverse) { (a, b) = (b, a); (za, zb) = (zb, za); }
        double lift = source.FlatCutterChordLift(a, za, b, zb, cutter.MaximumRadius,
            tolerance, default, margin, cutter);
        Assert.InRange(lift, 0, tolerance);
        for (int i = 0; i <= 100; i++)
        {
            double t = i / 100d, x = a.x + (b.x - a.x) * t, y = a.y + (b.y - a.y) * t;
            double nearest = Math.Min(Math.Min(x, y), Math.Min(10 - x, 10 - y)) - margin;
            double physicalContact = za + (zb - za) * t + lift + Bottom(nearest);
            Assert.InRange(physicalContact - surfaceZ, clearance - 1e-8, clearance + tolerance + 1e-8);
        }
        // The same physical chord lowered into the exterior must still be rejected.
        double penetrating = source.FlatCutterChordLift(a, za - 0.03, b, zb - 0.03,
            cutter.MaximumRadius, tolerance, default, margin, cutter);
        Assert.True(double.IsPositiveInfinity(penetrating));
    }
}
