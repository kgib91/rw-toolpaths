using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

/// <summary>
/// A ring handed to the cutter must contain every vertex the offset engine produced. Losing one
/// on a coarse ring — an offset triangle is three vertices — replaces a corner with a chord and
/// leaves a wedge of material standing.
/// </summary>
public class RingRebaseTests
{
    private static List<PointD> Square(double size) => new()
    {
        new PointD(0, 0),
        new PointD(size, 0),
        new PointD(size, size),
        new PointD(0, size),
    };

    private static List<PointD> Triangle(double size, double cx = 0, double cy = 0) => new()
    {
        new PointD(cx, cy),
        new PointD(cx + size, cy),
        new PointD(cx + size * 0.5, cy + size * 0.866),
    };

    [Fact]
    public void RebaseNear_OnAnOpenRing_KeepsEveryVertex()
    {
        var ring = Square(10);

        var rebased = PathUtils.RebaseNear(ring, new PointD(10, 10));

        Assert.Equal(ring.Count, rebased.Count);
        Assert.Equal(new PointD(10, 10), rebased[0]);
        foreach (var vertex in ring)
            Assert.Contains(rebased, p => p.x == vertex.x && p.y == vertex.y);
    }

    [Fact]
    public void RebaseNear_OnAClosedRing_KeepsEveryVertexAndStaysClosed()
    {
        var ring = Square(10);
        ring.Add(ring[0]);

        var rebased = PathUtils.RebaseNear(ring, new PointD(10, 10));

        Assert.Equal(ring.Count, rebased.Count);
        Assert.Equal(rebased[0], rebased[^1]);
        foreach (var vertex in ring)
            Assert.Contains(rebased, p => p.x == vertex.x && p.y == vertex.y);
    }

    [Fact]
    public void RebaseNear_PreservesTheCycleOrder()
    {
        var ring = Square(10);

        var rebased = PathUtils.RebaseNear(ring, new PointD(10, 10));

        Assert.Equal(
            new[] { new PointD(10, 10), new PointD(0, 10), new PointD(0, 0), new PointD(10, 0) },
            rebased);
    }

    public static IEnumerable<object[]> Regions()
    {
        yield return new object[] { "solid-triangle", new MillingRegion(Triangle(80)) };
        yield return new object[]
        {
            "triangle-outline",
            new MillingRegion(Triangle(80), new[] { (IReadOnlyList<PointD>)Triangle(40, 20, 11.55) }),
        };
        yield return new object[]
        {
            "triangle-with-island",
            new MillingRegion(Triangle(80), new[] { (IReadOnlyList<PointD>)Triangle(12, 34, 19) }),
        };
    }

    [Theory]
    [MemberData(nameof(Regions))]
    public void Pocket_CutsEveryRingVertex_InBothDirections(string name, MillingRegion region)
    {
        var tool = ToolGeometry.Flat(1.5);
        var depth = new DepthSchedule(3, 3);
        var options = new PocketOptions { StepOver = 0.4, MaxRings = 32 };

        var rings = RingOffsetEngine.BuildRings(
            new[] { region },
            new RingOffsetOptions(tool.Radius, tool.Radius * 2.0 * options.StepOver)
            {
                RingLimit = options.MaxRings,
            });

        foreach (bool insideOut in new[] { false, true })
        {
            var plan = PocketToolpaths.Generate(
                new[] { region }, tool, depth, options with { InsideOut = insideOut });

            var cut = plan.Toolpaths.SelectMany(t => t.Points).ToList();

            for (int r = 0; r < rings.Count; r++)
            {
                foreach (var vertex in rings[r].Points)
                {
                    Assert.True(
                        cut.Any(p => Math.Abs(p.X - vertex.x) < 1e-9 && Math.Abs(p.Y - vertex.y) < 1e-9),
                        $"{name} insideOut={insideOut}: ring {r} vertex ({vertex.x:F4},{vertex.y:F4}) never cut");
                }
            }
        }
    }
}
