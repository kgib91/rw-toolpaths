using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class OpenOffsetTopologyTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OpenTerminalChildSurvivesTraversalWithoutAnImplicitClosingChord(bool insideOut)
    {
        var wall = RingOffsetEngine.CreateRing(new PointD[]
        {
            new(0, 0), new(10, 0), new(10, 60), new(0, 60)
        }, 0, 0);
        var terminal = RingOffsetEngine.CreateRing(new PointD[]
        {
            new(5, 4), new(5, 30), new(6, 56)
        }, 1, 0, isClosed: false);
        var geometry = new[] { wall, terminal };
        RingOffsetEngine.BuildNesting(geometry);
        Assert.Equal(0, terminal.Parent);
        var traversal = RingOffsetEngine.BuildTraversal(geometry, insideOut);
        var chain = Assert.Single(RingOffsetEngine.BuildChains(geometry, traversal.Order));
        var points = RingOffsetEngine.MaterializeChain(geometry, chain, MillingDirection.Climb);
        var forbiddenA = terminal.Points[0];
        var forbiddenB = terminal.Points[^1];
        Assert.DoesNotContain(Enumerable.Range(1, points.Count - 1), i =>
            (points[i - 1] == forbiddenA && points[i] == forbiddenB)
            || (points[i - 1] == forbiddenB && points[i] == forbiddenA));
        Assert.All(terminal.Points, p => Assert.Contains(p, points));
    }

    [Fact]
    public void InsetBudgetIncludesOneFinalPartialStep()
    {
        var region = new MillingRegion(new PointD[]
        {
            new(0, 0), new(40, 0), new(40, 40), new(0, 40)
        });
        var levels = new List<double>();
        var rings = RingOffsetEngine.BuildRings(new[] { region }, new RingOffsetOptions(0, 3)
        {
            MaximumInset = 4, LevelProgress = (_, inset) => levels.Add(inset),
            Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = 0 }
        });
        Assert.Equal(new[] { 0d, 3, 4 }, levels);
        Assert.Equal(3, rings.Count);
        Assert.InRange(rings.Single(r => r.Level == 2).Points.Min(p => p.x), 3.999, 4.001);
    }
}
