using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class DeferredFrontCompactionTests
{
    [Fact]
    public void SelectedCleanupReproducesEstablishedPhysicalFrontWithoutCleaningOtherTrials()
    {
        var source = Enumerable.Range(0, 1024).Select(index =>
        {
            double angle = index * Math.Tau / 1024;
            return new PointD(27 * Math.Cos(angle), 27 * Math.Sin(angle) + (Math.Sin(angle) >= 0 ? 60 : 0));
        }).ToList();
        var deferred = PolygonOps.PrepareBoundaryOffset(source, null, .001, 0,
            sourceConstructionSweepRadius: 3);
        var reference = PolygonOps.PrepareBoundaryOffset(source, null, .001, 0,
            sourceConstructionSweepRadius: 3);
        List<List<PointD>> chosen = null!, expected = null!;
        foreach (double inset in new[] { 3d, 9d, 12.25 })
        {
            chosen = deferred.Execute(-inset, compactFront: false);
            expected = reference.Execute(-inset);
        }
        var final = deferred.CompactCandidate(chosen, -12.25, CancellationToken.None);
        Assert.Equal(3, deferred.DeferredCompactionCount);
        Assert.Equal(1, deferred.CompactionCount);
        Assert.Equal(3, reference.CompactionCount);
        Assert.Equal(Canonical(expected), Canonical(final));
    }

    [Theory]
    [InlineData(15.999)]
    [InlineData(16)]
    [InlineData(16.001)]
    public void NearTerminalSlotRetainsExactCommandsAndIndependentWallAndCoverageWitnesses(double width)
    {
        var region = new MillingRegion(Rectangle(0, 0, width, 60));
        var options = new RingOffsetOptions(3, 6) { MinimumRingArea = 0, RingLimit = 20 };
        var reference = MedialAxisGeometry.BuildConcentricFill(new[] { region },
            options with { DeferTrialCompaction = false, CacheCollapsedTerminals = false, SeedAcceptedFirstInset = false }, 3, true);
        var deferred = MedialAxisGeometry.BuildConcentricFill(new[] { region }, options, 3, true);
        Assert.Equal(RingCommands(reference), RingCommands(deferred));
        Assert.Contains(deferred, ring => !ring.IsClosed);
        foreach (var ring in deferred)
        foreach (var point in ring.Points)
            Assert.True(Math.Min(Math.Min(point.x, width - point.x), Math.Min(point.y, 60 - point.y)) >= 3 - 4 / PathUtils.Scale);
        for (double x = 3.25; x < width - 3; x += .5)
        for (double y = 3.25; y < 57; y += .5)
            Assert.True(deferred.Min(ring => Distance(new(x, y), ring)) <= 3 + .003 + 4 / PathUtils.Scale,
                $"Near-terminal reachable stock was missed at ({x:R},{y:R}).");
    }

    [Fact]
    public void UnprovedCompactedCandidateFallsBackToTheEstablishedTrialSearch()
    {
        var source = new MillingRegion(Rectangle(0, 0, 60, 90));
        var options = new RingOffsetOptions(3, 6) { PhysicalSweepRadius = 3, MinimumRingArea = 0 };
        Func<double, List<List<PointD>>> offset = inset => new() { Rectangle(inset, inset, 60 - inset, 90 - inset) };
        var method = typeof(RingOffsetEngine).GetMethod("SelectCoherentFront", BindingFlags.Static | BindingFlags.NonPublic)!;
        object Search(Func<double, List<List<PointD>>>? raw,
            Func<double, List<List<PointD>>, List<List<PointD>>>? compact)
            => method.Invoke(null, new object?[] { source, 0, 0, offset(3), 3d, 9d, offset(9),
                options, offset, 0d, CancellationToken.None, null, null, raw, compact, null })!;
        var expected = Search(null, null);
        int compactions = 0;
        var recovered = Search(offset, (_, _) =>
        {
            compactions++;
            // This final command front leaves the required shell uncovered.
            return new() { Rectangle(29, 44, 31, 46) };
        });
        Assert.Equal(1, compactions);
        var type = recovered.GetType();
        Assert.Equal(type.GetProperty("Distance")!.GetValue(expected), type.GetProperty("Distance")!.GetValue(recovered));
        Assert.Equal(Canonical((List<List<PointD>>)type.GetProperty("Polygons")!.GetValue(expected)!),
            Canonical((List<List<PointD>>)type.GetProperty("Polygons")!.GetValue(recovered)!));
    }

    private static string[] RingCommands(IEnumerable<OffsetRing> rings)
        => rings.Select(ring => $"{ring.Level}:{ring.IsClosed}:{ring.IsHole}:{ring.OffsetDistance:R}:"
            + string.Join(";", ring.Points.Select(point => $"{point.x:R},{point.y:R}"))).ToArray();

    private static string[] Canonical(IEnumerable<List<PointD>> paths)
        => paths.Select(path => string.Join(";", path.Select(point => $"{point.x:R},{point.y:R}"))).ToArray();

    private static double Distance(PointD point, OffsetRing ring)
    {
        double best = double.PositiveInfinity;
        for (int index = 0; index < ring.Points.Count - (ring.IsClosed ? 0 : 1); index++)
        {
            var a = ring.Points[index]; var b = ring.Points[(index + 1) % ring.Points.Count];
            double dx = b.x - a.x, dy = b.y - a.y, squared = dx * dx + dy * dy;
            double t = squared == 0 ? 0 : Math.Clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / squared, 0, 1);
            best = Math.Min(best, Math.Sqrt(Math.Pow(point.x - a.x - t * dx, 2) + Math.Pow(point.y - a.y - t * dy, 2)));
        }
        return best;
    }

    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
}
