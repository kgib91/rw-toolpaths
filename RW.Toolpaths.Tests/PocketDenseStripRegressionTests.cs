using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

/// <summary>Dense imported wall tessellation must not turn one residual strip into hundreds of visits.</summary>
public sealed class PocketDenseStripRegressionTests
{
    private const double Radius = 3;
    private const double Width = 16;
    private const double Length = 120;
    private readonly ITestOutputHelper _output;

    public PocketDenseStripRegressionTests(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData(false, 256)]
    [InlineData(true, 256)]
    [InlineData(false, 2048)]
    [InlineData(true, 2048)]
    public void QuantizedWallVerticesDoNotMultiplyStripVisits(bool insideOut, int wallSegments)
    {
        var outline = DenseSlot(wallSegments);
        var region = new MillingRegion(outline);
        var depth = new DepthSchedule(2, 1);
        var options = new PocketOptions
        {
            InsideOut = insideOut, StepOver = 1, SpiralChaining = false,
            Ramp = RampSettings.Plunge, DetectCurves = false, CalculateSweptArea = false,
        };
        var baseline = PocketToolpaths.Generate([region], ToolGeometry.Flat(Radius), depth,
            options with { StepOver = .99 });
        var plan = PocketToolpaths.Generate([region], ToolGeometry.Flat(Radius), depth, options);
        _output.WriteLine($"insideOut={insideOut}, sourceVertices={outline.Count}, "
            + $"rings={plan.Diagnostics.RingCount}, strips={plan.Diagnostics.StripCount}, paths={plan.Toolpaths.Count}.");

        Assert.Equal(1, baseline.Diagnostics.RingCount);
        Assert.Equal(1, baseline.Diagnostics.StripCount);
        Assert.Equal(1, plan.Diagnostics.StripCount);
        Assert.Equal(depth.PassCount, plan.Diagnostics.PlungeEntries + plan.Diagnostics.LinearRampEntries + plan.Diagnostics.HelicalEntries);
        foreach (var layer in plan.Toolpaths.GroupBy(path => path.DepthPassIndex))
            Assert.Equal(2, layer.Select(path => path.VisitIdentity).Distinct().Count());

        // Partial overlap can also lose its terminal wavefront before the next
        // requested inset. Both nearby stepovers must retain that open middle.
        var baselineMoves = Moves(baseline).ToArray();
        var moves = Moves(plan).ToArray();
        Assert.True(baselineMoves.Min(move => DistanceSquared(new PointD(8, 60), move.A, move.B)) <= Radius * Radius);

        for (int pass = 0; pass < depth.PassCount; pass++)
        {
            var layer = moves.Where(move => move.Pass == pass).ToArray();
            Assert.NotEmpty(layer);
            for (double y = 6.1; y <= Length - 6.1; y += 1)
            foreach (double x in new[] { 6.1, 8.0, 9.9 })
            {
                var witness = new PointD(x, y);
                double distance = Math.Sqrt(layer.Min(move => DistanceSquared(witness, move.A, move.B)));
                Assert.True(distance <= Radius + .002,
                    $"Uncut strip at ({x:R},{y:R}), pass={pass}, insideOut={insideOut}, "
                    + $"vertices={outline.Count}, nearestCut={distance:R}.");
            }
        }

        var terminalMoves = moves.Where(move => Math.Abs((move.A.x + move.B.x) / 2 - Width / 2) < .01
            && Math.Abs(move.A.x - move.B.x) < .01).ToArray();
        Assert.NotEmpty(terminalMoves);
        foreach (var move in terminalMoves)
        for (int wall = 0; wall < outline.Count; wall++)
        {
            double clearance = Math.Sqrt(SegmentDistanceSquared(move.A, move.B,
                outline[wall], outline[(wall + 1) % outline.Count]));
            Assert.True(clearance >= Radius - .0012,
                $"Retained terminal chord lacks source clearance: {move.A}->{move.B}, "
                + $"wall={wall}, clearance={clearance:R}, insideOut={insideOut}.");
        }
    }

    private static List<PointD> DenseSlot(int wallSegments)
    {
        // Every coordinate lands on the same integer grid as an imported contour.
        // Alternating submicron wall facets are below machining accuracy but create
        // genuine segment-site Voronoi branches if the entire source is skeletonized.
        double Noise(int index) => index % 4 switch
        {
            0 => 0, 1 => 16 / PathUtils.Scale,
            2 => -12 / PathUtils.Scale, _ => 8 / PathUtils.Scale,
        };
        var points = new List<PointD> { new(0, 0), new(Width, 0) };
        for (int i = 1; i < wallSegments; i++)
            points.Add(new PointD(Width + Noise(i), Length * i / wallSegments));
        points.Add(new PointD(Width, Length));
        points.Add(new PointD(0, Length));
        for (int i = wallSegments - 1; i > 0; i--)
            points.Add(new PointD(Noise(i + 1), Length * i / wallSegments));
        return points;
    }

    private static IEnumerable<(PointD A, PointD B, int Pass)> Moves(ToolpathPlan plan)
    {
        foreach (var path in plan.Toolpaths)
        foreach (var span in path.Spans.Where(span => span.Kind is ToolpathSpanKind.Cut or ToolpathSpanKind.Link))
        for (int i = span.StartIndex + 1; i <= span.EndIndex; i++)
        {
            Point3D a = path.Points[i - 1], b = path.Points[i];
            if (Math.Abs(a.Z - b.Z) < 1e-9)
                yield return (new PointD(a.X, a.Y), new PointD(b.X, b.Y), path.DepthPassIndex!.Value);
        }
    }

    private static (long, long, long, long) Key(PointD a, PointD b)
    {
        var qa = ((long)Math.Round(a.x * PathUtils.Scale), (long)Math.Round(a.y * PathUtils.Scale));
        var qb = ((long)Math.Round(b.x * PathUtils.Scale), (long)Math.Round(b.y * PathUtils.Scale));
        return qa.CompareTo(qb) <= 0 ? (qa.Item1, qa.Item2, qb.Item1, qb.Item2)
            : (qb.Item1, qb.Item2, qa.Item1, qa.Item2);
    }

    private static double SegmentDistanceSquared(PointD a, PointD b, PointD c, PointD d)
    {
        double abx = b.x - a.x, aby = b.y - a.y, cdx = d.x - c.x, cdy = d.y - c.y;
        double denominator = abx * cdy - aby * cdx;
        if (Math.Abs(denominator) > 1e-20)
        {
            double acx = c.x - a.x, acy = c.y - a.y;
            double t = (acx * cdy - acy * cdx) / denominator;
            double u = (acx * aby - acy * abx) / denominator;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return 0;
        }
        return Math.Min(Math.Min(DistanceSquared(a, c, d), DistanceSquared(b, c, d)),
            Math.Min(DistanceSquared(c, a, b), DistanceSquared(d, a, b)));
    }

    private static double DistanceSquared(PointD point, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y;
        double square = dx * dx + dy * dy;
        double t = square == 0 ? 0
            : Math.Clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / square, 0, 1);
        double x = point.x - a.x - t * dx, y = point.y - a.y - t * dy;
        return x * x + y * y;
    }
}
