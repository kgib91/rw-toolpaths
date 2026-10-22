using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ConcentricFillTopologyTests
{
    [Fact]
    public void PocketAndVCarveConsumersPreserveTheSameOpenTerminalTrace()
    {
        var rings = new List<OffsetRing>
        {
            RingOffsetEngine.CreateRing(new PointD[] { new(0, 0), new(8, 0), new(8, 8), new(0, 8) }, 0, 0),
            RingOffsetEngine.CreateRing(new PointD[] { new(2, 1), new(2, 3), new(5, 3) }, 1, 0, isClosed: false),
        };
        RingOffsetEngine.BuildNesting(rings);
        var pocket = PocketToolpaths.Emit(rings, RingOffsetEngine.BuildTraversal(rings, insideOut: true),
            new DepthSchedule(1, 1), new PocketOptions { CalculateSweptArea = false },
            ToolGeometry.Flat(.5), PocketToolpaths.Category, CancellationToken.None);
        var clearing = MedialAxisToolpaths.MaterializeClearingRing(rings[1], -1);

        Assert.Equal(3, clearing.Count);
        Assert.NotEqual(clearing[0], clearing[^1]);
        Assert.Equal(new Point3D(2, 1, -1), clearing[0]);
        Assert.Equal(new Point3D(5, 3, -1), clearing[^1]);
        Assert.Contains(pocket.Toolpaths, path => path.Spans.Any(span => span.Kind == ToolpathSpanKind.Ramp));
        foreach (var path in pocket.Toolpaths)
        foreach (var span in path.Spans.Where(span => span.Kind == ToolpathSpanKind.Cut))
        for (int index = span.StartIndex + 1; index <= span.EndIndex; index++)
        {
            var from = path.Points[index - 1];
            var to = path.Points[index];
            Assert.True(Math.Abs(from.X - to.X) < 1e-9 || Math.Abs(from.Y - to.Y) < 1e-9,
                $"Invented closing diagonal: ({from.X:R},{from.Y:R}) to ({to.X:R},{to.Y:R}).");
        }
    }

    [Fact]
    public void SharedCollectorPartialInsetRemainsExactThroughVCarveCompatibility()
    {
        var rectangle = new List<PointD> { new(0, 0), new(30, 0), new(30, 20), new(0, 20) };
        var layers = OffsetFill.OffsetArray(new[] { rectangle }, -.6, maxInset: 1);
        Assert.Equal(2, layers.Count);
        Assert.InRange(Math.Abs(layers[0][0].Min(point => point.x) - .6), 0, 1 / PathUtils.Scale);
        Assert.Equal(1, layers[1][0].Min(point => point.x), 6);
        Assert.All(layers.SelectMany(layer => layer), ring => Assert.Equal(ring[0], ring[^1]));
    }
    [Fact]
    public void DefaultFillTolerancePreservesCurvedOffsetPositionsAtEveryLevel()
    {
        const int vertexCount = 512;
        const double sourceRadius = 20, firstInset = 1, pitch = 1.6;
        var outline = Enumerable.Range(0, vertexCount).Select(index =>
        {
            double angle = index * Math.PI * 2 / vertexCount;
            return new PointD(sourceRadius * Math.Cos(angle), sourceRadius * Math.Sin(angle));
        }).ToList();
        var rings = MedialAxisGeometry.BuildConcentricFill(new[] { new MillingRegion(outline) },
            new RingOffsetOptions(firstInset, pitch)
            {
                RingLimit = 4,
                MaximumInset = 3 * pitch,
            }, sweepRadius: 1, preserveTerminalPaths: true);

        // This intentionally bounded fill has no terminal path to hide missing
        // contour fidelity. Each source-based offset must remain on the requested
        // radial locus, independent of the previous level's simplification.
        Assert.Equal(4, rings.Count);
        Assert.All(rings, ring => Assert.True(ring.IsClosed));
        foreach (var ring in rings)
        {
            double targetRadius = sourceRadius - firstInset - ring.Level * pitch;
            for (int index = 0; index < 128; index++)
            {
                double angle = (index + .37) * Math.PI * 2 / 128;
                var target = new PointD(targetRadius * Math.Cos(angle), targetRadius * Math.Sin(angle));
                double distance = DistanceToClosedRing(target, ring.Points);
                Assert.True(distance <= .002,
                    $"Curved offset lost source fidelity: level={ring.Level}, radius={targetRadius:R}, " +
                    $"angle={angle:R}, deviation={distance:R}, points={ring.Points.Count}.");
            }
        }
    }

    [Fact]
    public void VCarveClearingCompactionPreservesNativeTrailBranchRetraces()
    {
        // A terminal graph trail cuts a side branch and returns to its junction
        // before continuing along the spine. The reversal is required cutting
        // motion even though all three branch points are collinear.
        var terminal = RingOffsetEngine.CreateRing(new PointD[]
        {
            new(0, 0), new(3, 0), new(0, 0), new(0, 4),
        }, level: 1, regionIndex: 0, isClosed: false);
        var source = new[] { terminal };
        RingOffsetEngine.BuildNesting(source);
        var pocket = PocketToolpaths.Emit(source, RingOffsetEngine.BuildTraversal(source, insideOut: true),
            new DepthSchedule(1, 1), new PocketOptions { CalculateSweptArea = false },
            ToolGeometry.Flat(.5), PocketToolpaths.Category, CancellationToken.None);
        var materialized = MedialAxisToolpaths.MaterializeClearingRing(terminal, -1);
        var compacted = MedialAxisToolpaths.SimplifyCollinearRuns(materialized);

        Assert.Contains(new Point3D(3, 0, -1), pocket.Toolpaths.SelectMany(path => path.Points));
        Assert.Contains(new Point3D(3, 0, -1), compacted);
        Assert.Equal(materialized[0], compacted[0]);
        Assert.Equal(materialized[^1], compacted[^1]);
    }

    [Fact]
    public void FiniteBandAdaptsCornerSpacingAndStopsAtTheAuthoredFront()
    {
        var source = new List<PointD> { new(0, 0), new(60, 0), new(60, 90), new(0, 90) };
        var rings = MedialAxisGeometry.BuildConcentricFill(new[] { new MillingRegion(source) },
            new RingOffsetOptions(3, 6) { MaximumInset = 12, MinimumRingArea = 0 },
            sweepRadius: 3, preserveTerminalPaths: true);

        // Three nominal square fronts leave diagonal corner stock at full pitch.
        // The finite band must insert coherent fronts without entering its core.
        Assert.True(rings.Count > 3);
        Assert.All(rings, ring => Assert.True(ring.IsClosed));
        Assert.Equal(15, rings.Max(ring => ring.OffsetDistance));
        Assert.All(rings.SelectMany(ring => ring.Points), point =>
            Assert.InRange(Math.Min(Math.Min(point.x, 60 - point.x), Math.Min(point.y, 90 - point.y)), 2.999, 15.001));
        for (double x = 3.25; x < 57; x += .5)
        for (double y = 3.25; y < 87; y += .5)
        {
            if (Math.Min(Math.Min(x, 60 - x), Math.Min(y, 90 - y)) > 15) continue;
            var witness = new PointD(x, y);
            Assert.True(rings.Min(ring => DistanceToClosedRing(witness, ring.Points)) <= 3.003,
                $"Finite band left reachable stock at ({x:R},{y:R}).");
        }
        Assert.True(rings.Min(ring => DistanceToClosedRing(new(30, 45), ring.Points)) > 10);
    }

    [Fact]
    public void FiniteBandRetainsCollapsedComponentWhileASeparateCoreSurvives()
    {
        var narrow = new MillingRegion(new List<PointD> { new(0, 0), new(16, 0), new(16, 60), new(0, 60) });
        var broad = new MillingRegion(new List<PointD> { new(30, 0), new(90, 0), new(90, 90), new(30, 90) });
        var rings = MedialAxisGeometry.BuildConcentricFill(new[] { narrow, broad },
            new RingOffsetOptions(3, 6) { MaximumInset = 6, MinimumRingArea = 0 },
            sweepRadius: 3, preserveTerminalPaths: true);
        Assert.Contains(rings, ring => ring.RegionIndex == 0 && !ring.IsClosed);
        Assert.DoesNotContain(rings, ring => ring.RegionIndex == 1 && !ring.IsClosed);
        Assert.All(rings, ring => Assert.InRange(ring.OffsetDistance ?? 3, 3, 9));
        Assert.All(rings.Where(ring => ring.RegionIndex == 0).SelectMany(ring => ring.Points), point =>
            Assert.InRange(Math.Min(Math.Min(point.x, 16 - point.x), Math.Min(point.y, 60 - point.y)), 2.999, 9.001));
        var centerTrace = rings.Where(ring => ring.RegionIndex == 0 && !ring.IsClosed).ToList();
        for (double y = 8; y <= 52; y += .5)
            Assert.True(centerTrace.Min(ring => DistanceToOpenRing(new(8, y), ring.Points)) <= 3.001,
                $"Collapsed finite-band sliver lost its native spine at y={y:R}.");
        Assert.True(rings.Where(ring => ring.RegionIndex == 1)
            .Min(ring => DistanceToClosedRing(new(60, 45), ring.Points)) > 15);
    }

    private static double DistanceToOpenRing(PointD target, IReadOnlyList<PointD> points)
        => Enumerable.Range(1, points.Count - 1).Min(index =>
            DistanceToSegment(target, points[index - 1], points[index]));

    private static double DistanceToClosedRing(PointD target, IReadOnlyList<PointD> points)
    {
        double minimum = double.PositiveInfinity;
        for (int index = 0; index < points.Count; index++)
        {
            var from = points[index];
            var to = points[(index + 1) % points.Count];
            minimum = Math.Min(minimum, DistanceToSegment(target, from, to));
        }
        return minimum;
    }

    private static double DistanceToSegment(PointD target, PointD from, PointD to)
    {
        double dx = to.x - from.x, dy = to.y - from.y;
        double lengthSquared = dx * dx + dy * dy;
        double t = lengthSquared == 0 ? 0 : Math.Clamp(
            ((target.x - from.x) * dx + (target.y - from.y) * dy) / lengthSquared, 0, 1);
        return Math.Sqrt(Math.Pow(target.x - from.x - t * dx, 2) + Math.Pow(target.y - from.y - t * dy, 2));
    }

}
