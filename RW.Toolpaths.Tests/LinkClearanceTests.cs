using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

/// <summary>
/// The cutter is allowed to stay down between rings because lifting costs time, but only where
/// the straight move provably removes nothing the operation has to leave standing. These cover
/// the guard itself and the invariant it exists to protect: no emitted move crosses an island.
/// </summary>
public class LinkClearanceTests
{
    private static List<PointD> Rect(double minX, double minY, double maxX, double maxY) => new()
    {
        new PointD(minX, minY),
        new PointD(maxX, minY),
        new PointD(maxX, maxY),
        new PointD(minX, maxY),
    };

    private static List<PointD> Reversed(List<PointD> ring)
    {
        var copy = new List<PointD>(ring);
        copy.Reverse();
        return copy;
    }

    /// <summary>Square travel area with a square island punched out of the middle.</summary>
    private static ILinkClearance RegionWithIsland()
        => LinkClearance.WithinRegion(new List<IReadOnlyList<PointD>>
        {
            Rect(0, 0, 100, 100),
            Reversed(Rect(40, 40, 60, 60)),
        });

    [Fact]
    public void RejectsALinkThatCrossesAnIsland()
    {
        var clearance = RegionWithIsland();

        Assert.False(clearance.IsTravelSafe(
            new Point3D(10, 50, -2), new Point3D(90, 50, -2), toolRadius: 3));
    }

    [Fact]
    public void RejectsALinkThatClipsTheCornerOfAnIsland()
    {
        var clearance = RegionWithIsland();

        Assert.False(clearance.IsTravelSafe(
            new Point3D(30, 55, -2), new Point3D(55, 30, -2), toolRadius: 3));
    }

    [Fact]
    public void AllowsALinkThatStaysClearOfTheIsland()
    {
        var clearance = RegionWithIsland();

        Assert.True(clearance.IsTravelSafe(
            new Point3D(10, 20, -2), new Point3D(90, 20, -2), toolRadius: 3));
    }

    [Fact]
    public void RejectsALinkThatLeavesTheRegionEntirely()
    {
        var clearance = RegionWithIsland();

        Assert.False(clearance.IsTravelSafe(
            new Point3D(10, 10, -2), new Point3D(150, 10, -2), toolRadius: 3));
    }

    [Fact]
    public void AllowsALinkThatRunsAlongTheRegionBoundary()
    {
        var clearance = RegionWithIsland();

        // Ring level zero is cut from the travel region's own boundary, so its links sit on it.
        Assert.True(clearance.IsTravelSafe(
            new Point3D(0, 0, -2), new Point3D(0, 100, -2), toolRadius: 3));
    }

    [Fact]
    public void AlwaysLiftOnlyAllowsRingsThatAlreadyMeet()
    {
        Assert.True(LinkClearance.AlwaysLift.IsTravelSafe(
            new Point3D(10, 10, -2), new Point3D(10, 10, -2), toolRadius: 3));
        Assert.False(LinkClearance.AlwaysLift.IsTravelSafe(
            new Point3D(10, 10, -2), new Point3D(10, 11, -2), toolRadius: 3));
    }

    [Fact]
    public void PocketLinkSearch_UsesSafeAlternativeWhenNearestPointIsBlocked()
    {
        var clearance = RegionWithIsland();
        var ring = Rect(70, 0, 90, 100);
        var from = new Point3D(30, 50, -2);

        bool found = PocketToolpaths.TryFindSafeRebase(
            ring,
            from,
            bottomZ: -2,
            clearance,
            toolRadius: 3,
            out var rebased,
            out int candidatesTested);

        Assert.True(found);
        Assert.True(candidatesTested > 1, "the nearest point should be rejected by the island");
        Assert.True(clearance.IsTravelSafe(
            from,
            new Point3D(rebased[0].x, rebased[0].y, -2),
            toolRadius: 3));
    }

    [Fact]
    public void PocketNeverDragsTheCutterThroughAnIsland()
    {
        var island = Rect(40, 40, 60, 60);
        var plan = PocketToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 100), new[] { (IReadOnlyList<PointD>)island }) },
            ToolGeometry.Flat(3),
            new DepthSchedule(3, 3),
            new PocketOptions());

        Assert.NotEmpty(plan.Toolpaths);

        // Anything the cutter may touch has to stay a radius clear of the island it must leave.
        var keepOut = new Path64(island.Select(p => new Point64(
            (long)Math.Round((p.x + (p.x < 50 ? 3 : -3)) * PathUtils.Scale),
            (long)Math.Round((p.y + (p.y < 50 ? 3 : -3)) * PathUtils.Scale))));

        foreach (var toolpath in plan.Toolpaths)
        {
            var points = toolpath.Points;
            for (int i = 1; i < points.Count; i++)
            {
                var moved = new Path64
                {
                    new((long)Math.Round(points[i - 1].X * PathUtils.Scale), (long)Math.Round(points[i - 1].Y * PathUtils.Scale)),
                    new((long)Math.Round(points[i].X * PathUtils.Scale), (long)Math.Round(points[i].Y * PathUtils.Scale)),
                };

                var clipper = new Clipper64();
                clipper.AddOpenSubject(moved);
                clipper.AddClip(new Paths64 { keepOut });

                var closed = new Paths64();
                var inside = new Paths64();
                clipper.Execute(ClipType.Intersection, FillRule.NonZero, closed, inside);

                Assert.True(
                    inside.Count == 0,
                    $"move ({points[i - 1].X:F2},{points[i - 1].Y:F2}) -> ({points[i].X:F2},{points[i].Y:F2}) enters the island");
            }
        }
    }
}
