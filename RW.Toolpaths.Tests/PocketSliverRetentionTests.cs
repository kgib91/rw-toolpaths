using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketSliverRetentionTests
{
    [Theory]
    [InlineData(false, .002)]
    [InlineData(true, .002)]
    [InlineData(false, .005)]
    [InlineData(true, .005)]
    public void PositiveThinInsetRetainsBothEdgesAndTerminalCornerCoverage(bool insideOut, double width)
    {
        const double radius = 3;
        double pocketWidth = 18 + width;
        var region = new MillingRegion(new List<PointD>
        {
            new(0, 0), new(pocketWidth, 0), new(pocketWidth, 60), new(0, 60),
        });
        var plan = PocketToolpaths.Generate(new[] { region }, ToolGeometry.Flat(radius),
            new DepthSchedule(1, 1), new PocketOptions
            {
                StepOver = 1, InsideOut = insideOut, SpiralChaining = true,
                Ramp = RampSettings.FromRatio(4), DetectCurves = false, CalculateSweptArea = false,
                Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = 0 },
            });

        Assert.True(plan.Diagnostics.RingCount >= 2);
        Assert.InRange(plan.Diagnostics.StripCount, 0, 1);
        Assert.Equal(1, plan.Diagnostics.PlungeEntries + plan.Diagnostics.LinearRampEntries + plan.Diagnostics.HelicalEntries);
        for (int path = 1; path < plan.Toolpaths.Count; path++)
            Assert.Equal(plan.Toolpaths[path - 1].Points[^1], plan.Toolpaths[path].Points[0]);

        // A single center trace after the first ring misses these two thin edge
        // bands. Coherent intermediate fronts must cover them without adding
        // independent corner strokes and entries.
        var witnesses = new[]
        {
            new PointD(6 + width / 4, 30), new PointD(12 + width * 3 / 4, 30),
            // Full diameter pitch at square corners requires a coherent spacing
            // adjustment; coverage alone must not justify a separate medial stub.
            new PointD(6.5, 6.5), new PointD(pocketWidth - 6.5, 6.5),
            new PointD(6.5, 53.5), new PointD(pocketWidth - 6.5, 53.5),
        };
        foreach (var witness in witnesses)
        {
            double closest = double.PositiveInfinity;
            foreach (var visit in plan.Toolpaths)
            foreach (var span in visit.Spans.Where(span => span.Kind is ToolpathSpanKind.Cut or ToolpathSpanKind.Link))
            for (int i = span.StartIndex + 1; i <= span.EndIndex; i++)
            {
                var a = visit.Points[i - 1];
                var b = visit.Points[i];
                if (a.Z != -1 || b.Z != -1) continue;
                double dx = b.X - a.X, dy = b.Y - a.Y;
                double lengthSquared = dx * dx + dy * dy;
                double t = lengthSquared == 0 ? 0 : Math.Clamp(((witness.x - a.X) * dx + (witness.y - a.Y) * dy) / lengthSquared, 0, 1);
                double ex = witness.x - (a.X + t * dx), ey = witness.y - (a.Y + t * dy);
                closest = Math.Min(closest, Math.Sqrt(ex * ex + ey * ey));
            }
            Assert.True(closest <= radius + 1e-9,
                $"Uncut reachable material: insideOut={insideOut}, width={width:R}, witness={witness}, cutterDistance={closest:R}.");
        }
    }
}
