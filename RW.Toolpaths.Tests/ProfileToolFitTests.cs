using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class ProfileToolFitTests
{
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
}