using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class ProfileToolFitTests
{
    [Theory]
    [InlineData(-1, 22)]
    [InlineData(-3, 20)]
    [InlineData(-4, 19)]
    public void InsideProfile_OpeningFitIgnoresSignedOffsetButEligibleContourKeepsIt(double offset, double expectedMinX)
    {
        var prepared = ProfileToolpaths.Prepare(
            [new MillingRegion(Rect(0, 0, 4, 4)), new MillingRegion(Rect(20, 0, 50, 30))],
            ToolGeometry.Flat(3), new ProfileOptions { Side = ProfileSide.Inside, StockToLeave = offset });

        var ring = Assert.Single(prepared.Rings);
        Assert.Equal(1, ring.RegionIndex);
        Assert.Equal(expectedMinX, ring.Points.Min(point => point.x), 4);
        Assert.Equal(2, prepared.SourceMaterial.Count);
    }

    [Theory]
    [InlineData(-1, 22)]
    [InlineData(-3, 20)]
    [InlineData(-4, 19)]
    public void OutsideProfile_OpeningFitOmitsUndersizedHoleButKeepsOuterAndEligibleHole(double offset, double expectedMinX)
    {
        var prepared = ProfileToolpaths.Prepare(
            [new MillingRegion(Rect(-10, -10, 80, 40), [Rect(0, 0, 4, 4), Rect(20, 0, 50, 30)])],
            ToolGeometry.Flat(3), new ProfileOptions { Side = ProfileSide.Outside, StockToLeave = offset });

        Assert.Equal(2, prepared.Rings.Count);
        Assert.Single(prepared.Rings, ring => !ring.IsHole);
        var hole = Assert.Single(prepared.Rings, ring => ring.IsHole);
        Assert.Equal(expectedMinX, hole.Points.Min(point => point.x), 4);
        Assert.Equal(3, prepared.SourceMaterial.Count);
    }

    [Fact]
    public void InsideProfile_OpeningFitRejectsLongDiagonalSlitDespiteLargeBounds()
    {
        var diagonal = Rect(-50, -2.995, 50, 2.995)
            .Select(point => new PointD((point.x - point.y) / Math.Sqrt(2), (point.x + point.y) / Math.Sqrt(2)))
            .ToList();
        var prepared = ProfileToolpaths.Prepare([new MillingRegion(diagonal)], ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.Inside, StockToLeave = -4 });
        Assert.Empty(prepared.Rings);
    }

    [Fact]
    public void InsideProfile_OpeningFitIncludesProtectedHoleAndFindsOffCenterRoom()
    {
        var thinWall = new MillingRegion(Rect(0, 0, 100, 100), [Rect(2, 2, 98, 98)]);
        var room = Assert.Single(PolygonOps.Union([Rect(120, 0, 220, 2), Rect(208, -6, 220, 14)]));
        var prepared = ProfileToolpaths.Prepare([thinWall, new MillingRegion(room)], ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.Inside, StockToLeave = -4 });
        Assert.NotEmpty(prepared.Rings);
        Assert.All(prepared.Rings, ring => Assert.Equal(1, ring.RegionIndex));
    }

    [Fact]
    public void OutsideProfile_OpeningFitAccountsForNestedRetainedIslandWithoutSkippingItsOuterProfile()
    {
        var prepared = ProfileToolpaths.Prepare(
            [new MillingRegion(Rect(0, 0, 100, 100), [Rect(20, 20, 80, 80)]),
             new MillingRegion(Rect(22, 22, 78, 78))],
            ToolGeometry.Flat(3), new ProfileOptions { Side = ProfileSide.Outside, StockToLeave = -4 });
        Assert.Equal(2, prepared.Rings.Count);
        Assert.DoesNotContain(prepared.Rings, ring => ring.IsHole);
        Assert.Contains(prepared.Rings, ring => ring.RegionIndex == 1);
    }

    [Fact]
    public void OpeningFitAcceptsExactTangentFitAndLeavesOnLineUnfiltered()
    {
        var inside = ProfileToolpaths.Prepare([new MillingRegion(Rect(0, 0, 6, 6))], ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.Inside, StockToLeave = -1 });
        Assert.Single(inside.Rings);
        var onLine = ProfileToolpaths.Prepare([new MillingRegion(Rect(0, 0, 1, 1))], ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.OnLine });
        Assert.Single(onLine.Rings);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(2)]
    public void InsideProfile_CollapsedNeckRetractsInsteadOfLinkingAcrossProtectedMaterial(int springPasses)
    {
        // The 5.9mm neck cannot admit a 6mm cutter. Its offset leaves two valid
        // rooms only about 1.2mm apart, so a radius-only link test crosses the neck.
        List<PointD> source =
        [
            new(0, 0), new(10, 0), new(10, 2.05), new(10.1, 2.05),
            new(10.1, 0), new(20.1, 0), new(20.1, 10), new(10.1, 10),
            new(10.1, 7.95), new(10, 7.95), new(10, 10), new(0, 10),
        ];
        var plan = ProfileToolpaths.Generate(
            [new MillingRegion(source)], ToolGeometry.Flat(3), new DepthSchedule(2, 2),
            new ProfileOptions
            {
                Side = ProfileSide.Inside,
                SpringPasses = springPasses,
                Ramp = RampSettings.Plunge,
                Tolerances = new GeometryTolerances { ArcTolerance = 0.001, SimplifyTolerance = 0 },
            });

        Assert.Equal(2, plan.Diagnostics.RingCount);
        // Spring-pass retraces within a previously cut room are legal. No Link
        // may bridge the two rooms through their cutter-inaccessible neck.
        foreach(var path in plan.Toolpaths)
        foreach(var span in path.Spans.Where(s=>s.Kind==ToolpathSpanKind.Link))
        for(int i=span.StartIndex+1;i<=span.EndIndex;i++)
            Assert.Equal(path.Points[i-1].X<10,path.Points[i].X<10);
        Assert.True(plan.Diagnostics.ExecutionEntryCount>=2);
        Assert.True(plan.Diagnostics.LinkLifts >= 2 * (1 + springPasses));
        foreach (var path in plan.Toolpaths)
        foreach (var point in path.Points)
            Assert.True(point.X < 10 || point.X > 10.1,
                $"Unexpected cutter-center motion through the collapsed neck at ({point.X:R},{point.Y:R}).");
    }

    private static List<PointD> Rect(double minX, double minY, double maxX, double maxY) =>
    [
        new PointD(minX, minY),
        new PointD(maxX, minY),
        new PointD(maxX, maxY),
        new PointD(minX, maxY),
    ];

    private static List<PointD> Circle(double centerX, double centerY, double radius, int segments = 64)
    {
        var points = new List<PointD>(segments);
        for (int index = 0; index < segments; index++)
        {
            double angle = Math.PI * 2.0 * index / segments;
            points.Add(new PointD(
                centerX + radius * Math.Cos(angle),
                centerY + radius * Math.Sin(angle)));
        }
        return points;
    }

    [Fact]
    public void InsideProfile_OversizedTool_DoesNotGenerateCollapsedCircle()
    {
        var plan = ProfileToolpaths.Generate(
            [new MillingRegion(Circle(0, 0, 2))],
            ToolGeometry.Flat(3),
            new DepthSchedule(2, 2),
            new ProfileOptions { Side = ProfileSide.Inside });

        Assert.Empty(plan.Toolpaths);
        Assert.Equal(0, plan.Diagnostics.RingCount);
    }

    [Fact]
    public void InsideProfile_ToolThatFits_KeepsCircle()
    {
        var plan = ProfileToolpaths.Generate(
            [new MillingRegion(Circle(0, 0, 4))],
            ToolGeometry.Flat(3),
            new DepthSchedule(2, 2),
            new ProfileOptions { Side = ProfileSide.Inside });

        Assert.NotEmpty(plan.Toolpaths);
        Assert.Equal(1, plan.Diagnostics.RingCount);
    }

    [Fact]
    public void OutsideProfile_OversizedTool_DropsUndersizedHoleOnly()
    {
        var plan = ProfileToolpaths.Generate(
            [new MillingRegion(Rect(-50, -50, 50, 50), [Circle(0, 0, 2)])],
            ToolGeometry.Flat(3),
            new DepthSchedule(2, 2),
            new ProfileOptions { Side = ProfileSide.Outside });

        Assert.NotEmpty(plan.Toolpaths);
        Assert.Equal(1, plan.Diagnostics.RingCount);
        Assert.All(
            plan.Toolpaths.SelectMany(path => path.Points),
            point => Assert.True(
                Math.Sqrt(point.X * point.X + point.Y * point.Y) > 40,
                $"Unexpected undersized-hole path at ({point.X:F3}, {point.Y:F3})"));
    }

    [Fact]
    public void Pocket_OversizedTool_DoesNotGenerateCollapsedCircle()
    {
        var plan = PocketToolpaths.Generate(
            [new MillingRegion(Circle(0, 0, 2))],
            ToolGeometry.Flat(3),
            new DepthSchedule(2, 2),
            new PocketOptions());

        Assert.Empty(plan.Toolpaths);
        Assert.Equal(0, plan.Diagnostics.RingCount);
    }

    [Fact]
    public void InsideProfile_LargeDecalWithUndersizedCornerNubs_KeepsMainContourOnly()
    {
        var pieces = new List<IReadOnlyList<PointD>>
        {
            Rect(0, 0, 400, 200),
            Circle(-2, -2, 3),
            Circle(402, -2, 3),
            Circle(402, 202, 3),
            Circle(-2, 202, 3),
        };
        var decal = Assert.Single(PolygonOps.Union(pieces));

        var plan = ProfileToolpaths.Generate(
            [new MillingRegion(decal)],
            ToolGeometry.Flat(3.175),
            new DepthSchedule(2, 2),
            new ProfileOptions { Side = ProfileSide.Inside });

        Assert.NotEmpty(plan.Toolpaths);
        Assert.Equal(1, plan.Diagnostics.RingCount);
        Assert.All(
            plan.Toolpaths.SelectMany(path => path.Points),
            point =>
            {
                Assert.InRange(point.X, 3.17, 396.83);
                Assert.InRange(point.Y, 3.17, 196.83);
            });
    }

    [Fact]
    public void OutsideProfile_OmitEdgeLoops_SlicesCrossingMotionsToInteriorFragments()
    {
        var clipBoundary = Rect(20, -10, 100, 90);
        var plan = ProfileToolpaths.Generate(
            [new MillingRegion(Rect(0, 0, 120, 80))],
            ToolGeometry.Flat(3),
            new DepthSchedule(2, 2),
            new ProfileOptions
            {
                Side = ProfileSide.Outside,
                ClipBoundary = clipBoundary,
                OmitEdgeLoops = true,
                Ramp = RampSettings.Plunge,
            });

        Assert.NotEmpty(plan.Toolpaths);
        Assert.All(plan.Toolpaths, path =>
        {
            for (int pointIndex = 1; pointIndex < path.Points.Count; pointIndex++)
            {
                Point3D start = path.Points[pointIndex - 1];
                Point3D end = path.Points[pointIndex];
                double midpointX = (start.X + end.X) * 0.5;
                double midpointY = (start.Y + end.Y) * 0.5;
                Assert.InRange(midpointX, 20 + 1e-9, 100 - 1e-9);
                Assert.InRange(midpointY, -10 + 1e-9, 90 - 1e-9);
            }
        });

        Assert.DoesNotContain(
            plan.Toolpaths.SelectMany(path => path.Points.Zip(path.Points.Skip(1))),
            segment => Math.Abs(segment.First.X - 20) < 1e-9
                && Math.Abs(segment.Second.X - 20) < 1e-9);
        Assert.DoesNotContain(
            plan.Toolpaths.SelectMany(path => path.Points.Zip(path.Points.Skip(1))),
            segment => Math.Abs(segment.First.X - 100) < 1e-9
                && Math.Abs(segment.Second.X - 100) < 1e-9);
    }
}
