using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketFullStepFidelityTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DefaultSimplificationDoesNotLeaveReachableWallCrescentsAtFullStepover(bool insideOut)
    {
        const int vertices = 256;
        const double pocketRadius = 12, toolRadius = 3;
        var outline = Enumerable.Range(0, vertices).Select(index =>
        {
            double angle = index * Math.PI * 2 / vertices;
            return new PointD(pocketRadius * Math.Cos(angle), pocketRadius * Math.Sin(angle));
        }).ToList();
        var plan = PocketToolpaths.Generate(new[] { new MillingRegion(outline) },
            ToolGeometry.Flat(toolRadius), new DepthSchedule(1, 1), new PocketOptions
            {
                StepOver = 1, InsideOut = insideOut, DetectCurves = false, CalculateSweptArea = false,
            });
        Assert.Equal(2, plan.Diagnostics.RingCount);

        // These points are strictly inside the original polygon and reachable by
        // a source-clear cutter disc. Coarse RDP wall chords leave crescents here
        // despite a completed inward core, so checking only that core is insufficient.
        for (int index = 0; index < 96; index++)
        {
            double angle = (index + .37) * Math.PI * 2 / 96;
            var witness = new PointD(11.98 * Math.Cos(angle), 11.98 * Math.Sin(angle));
            double closest = double.PositiveInfinity;
            foreach (var path in plan.Toolpaths)
            foreach (var span in path.Spans.Where(span => span.Kind is ToolpathSpanKind.Cut or ToolpathSpanKind.Link))
            for (int point = span.StartIndex + 1; point <= span.EndIndex; point++)
            {
                var a = path.Points[point - 1];
                var b = path.Points[point];
                if (a.Z != -1 || b.Z != -1) continue;
                double dx = b.X - a.X, dy = b.Y - a.Y;
                double squaredLength = dx * dx + dy * dy;
                double t = squaredLength == 0 ? 0 : Math.Clamp(
                    ((witness.x - a.X) * dx + (witness.y - a.Y) * dy) / squaredLength, 0, 1);
                double ex = witness.x - a.X - t * dx, ey = witness.y - a.Y - t * dy;
                closest = Math.Min(closest, Math.Sqrt(ex * ex + ey * ey));
            }
            Assert.True(closest <= toolRadius,
                $"Reachable wall material remains: insideOut={insideOut}, angle={angle:R}, cutterDistance={closest:R}.");
        }
    }
}
