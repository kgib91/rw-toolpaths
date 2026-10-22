using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ConcentricMachiningQualityTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OrdinarySquareCornersDoNotProduceSeparateMedialStrokes(bool insideOut)
    {
        var region = new MillingRegion(new List<PointD>
        {
            new(0, 0), new(60, 0), new(60, 90), new(0, 90),
        });
        var fill = MedialAxisGeometry.BuildConcentricFill(new[] { region },
            new RingOffsetOptions(3, 6) { MinimumRingArea = 0 }, 3, true);
        var open = fill.Where(path => !path.IsClosed).ToArray();
        Assert.InRange(open.Length, 0, 1);
        Assert.All(open.SelectMany(path => path.Points), point => Assert.InRange(point.x, 29.998, 30.002));

        var plan = PocketToolpaths.Generate(new[] { region }, ToolGeometry.Flat(3),
            new DepthSchedule(2, 1), new PocketOptions
            {
                StepOver = 1, InsideOut = insideOut, SpiralChaining = true,
                Ramp = RampSettings.FromRatio(4), DetectCurves = false, CalculateSweptArea = false,
            });
        Assert.Equal(2, plan.Diagnostics.LinearRampEntries + plan.Diagnostics.PlungeEntries + plan.Diagnostics.HelicalEntries);
        foreach (var layer in plan.Toolpaths.GroupBy(path => path.DepthPassIndex))
        {
            var paths = layer.ToArray();
            for (int index = 1; index < paths.Length; index++)
                Assert.Equal(paths[index - 1].Points[^1], paths[index].Points[0]);
        }
    }

    [Theory]
    [InlineData(128)]
    [InlineData(1024)]
    public void ARoundEndedCurvedTerminalSliverRemainsOneOrderedOpenArc(int subdivisions)
    {
        var outline = new List<PointD>();
        for (int index = 0; index <= subdivisions; index++)
        {
            double angle = Math.PI / 2 * index / subdivisions;
            outline.Add(new PointD(46 * Math.Cos(angle), 46 * Math.Sin(angle)));
        }
        // Round ends make this a constant-width curved slot. Flat radial ends
        // have actual corner pockets and need a coherent extra contour instead.
        for (int index = 1; index < subdivisions; index++)
        {
            double angle = Math.PI / 2 + Math.PI * index / subdivisions;
            outline.Add(new PointD(8 * Math.Cos(angle), 38 + 8 * Math.Sin(angle)));
        }
        for (int index = subdivisions; index >= 0; index--)
        {
            double angle = Math.PI / 2 * index / subdivisions;
            outline.Add(new PointD(30 * Math.Cos(angle), 30 * Math.Sin(angle)));
        }
        for (int index = 1; index < subdivisions; index++)
        {
            double angle = Math.PI + Math.PI * index / subdivisions;
            outline.Add(new PointD(38 + 8 * Math.Cos(angle), 8 * Math.Sin(angle)));
        }
        var fill = MedialAxisGeometry.BuildConcentricFill(new[] { new MillingRegion(outline) },
            new RingOffsetOptions(3, 6) { MinimumRingArea = 0 }, 3, true);
        foreach (var boundary in fill.Where(path => path.IsClosed))
            Console.WriteLine($"[Quality.CurvedSlot] subdivisions={subdivisions} hole={boundary.IsHole} "
                + $"points={boundary.Points.Count} perimeter={boundary.Perimeter:R} "
                + $"bounds=({boundary.Points.Min(p => p.x):R},{boundary.Points.Min(p => p.y):R})-"
                + $"({boundary.Points.Max(p => p.x):R},{boundary.Points.Max(p => p.y):R})");
        Assert.Single(fill.Where(path => path.IsClosed));
        var terminal = Assert.Single(fill.Where(path => !path.IsClosed));
        Assert.NotEqual(terminal.Points[0], terminal.Points[^1]);
        Assert.All(terminal.Points, point => Assert.InRange(Math.Sqrt(point.x * point.x + point.y * point.y), 37.98, 38.02));
        var angles = terminal.Points.Select(point => Math.Atan2(point.y, point.x)).ToArray();
        double sign = Math.Sign(angles[^1] - angles[0]);
        Assert.NotEqual(0, sign);
        for (int index = 1; index < angles.Length; index++)
            Assert.True((angles[index] - angles[index - 1]) * sign >= -1e-6,
                $"Terminal arc reverses into a branch at point {index}, source subdivisions={subdivisions}.");

        var plan = PocketToolpaths.Generate(new[] { new MillingRegion(outline) }, ToolGeometry.Flat(3),
            new DepthSchedule(2, 1), new PocketOptions
            {
                StepOver = 1, SpiralChaining = true,
                Ramp = RampSettings.FromRatio(4), DetectCurves = false, CalculateSweptArea = false,
            });
        int entries = plan.Diagnostics.LinearRampEntries + plan.Diagnostics.PlungeEntries + plan.Diagnostics.HelicalEntries;
        Console.WriteLine($"[Quality.CurvedSlot.Pocket] subdivisions={subdivisions} depths=2 "
            + $"entries={entries} visits={plan.Toolpaths.Count} independentFrontBoundaries={fill.Count(path => path.IsClosed)}");
        Assert.Equal(2, entries);
        foreach (var layer in plan.Toolpaths.GroupBy(path => path.DepthPassIndex))
        {
            var paths = layer.ToArray();
            for (int index = 1; index < paths.Length; index++)
                Assert.Equal(paths[index - 1].Points[^1], paths[index].Points[0]);
        }
    }
}
