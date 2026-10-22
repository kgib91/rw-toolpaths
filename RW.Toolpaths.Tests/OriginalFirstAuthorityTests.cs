using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class OriginalFirstAuthorityTests
{
    [Fact]
    public void DenseAnnulusUsesExactlyEstablishedOriginalFirstCommandsAndPreservesHole()
    {
        var outer = Circle(0, 0, 30, 8192);
        var hole = Circle(0, 0, 8, 2048);
        var selected = Prepared(outer, new[] { hole }, .001);
        var reference = Prepared(outer, new[] { hole }, 0);
        var actual = selected.ExecuteOriginalFirstAuthority(-3);
        var expected = reference.Execute(-3);
        Assert.Equal(Commands(expected), Commands(actual));
        Assert.Equal(1, selected.OriginalFirstAuthorityCount);
        var filled = PolygonOps.DifferenceToComponents(actual, Array.Empty<IReadOnlyList<PointD>>());
        var component = Assert.Single(filled);
        Assert.Single(component.Holes);
        Assert.Empty(PolygonOps.Intersect(actual, new[] { hole }));
        Assert.Equal(Commands(actual), Commands(selected.ExecuteOriginalFirstAuthority(-3)));
        Assert.Equal(1, selected.OriginalFirstAuthorityCount);
    }

    [Theory]
    [InlineData(5.999)]
    [InlineData(6.001)]
    public void NearNeckFirstCommandsMatchOriginalConstructionOnBothSidesOfCollapse(double neckWidth)
    {
        var roomA = Circle(-14, 0, 12, 2048);
        var roomB = Circle(14, 0, 12, 2048);
        var neck = new List<PointD> { new(-14, -neckWidth / 2), new(14, -neckWidth / 2),
            new(14, neckWidth / 2), new(-14, neckWidth / 2) };
        var source = Assert.Single(PolygonOps.Union(new[] { roomA, roomB, neck }));
        var selected = Prepared(source, null, .001);
        var reference = Prepared(source, null, 0);
        var actual = selected.ExecuteOriginalFirstAuthority(-3);
        Assert.Equal(Commands(reference.Execute(-3)), Commands(actual));
        Assert.Equal(1, selected.OriginalFirstAuthorityCount);
        Assert.NotEmpty(actual);
    }

    [Fact]
    public void ExactSourceWithoutConditioningRetainsOrdinaryOffsetPath()
    {
        var rectangle = new List<PointD> { new(0, 0), new(60, 0), new(60, 90), new(0, 90) };
        var selected = Prepared(rectangle, null, 0);
        var reference = Prepared(rectangle, null, 0);
        Assert.Equal(Commands(reference.Execute(-3)), Commands(selected.ExecuteOriginalFirstAuthority(-3)));
        Assert.Equal(0, selected.OriginalFirstAuthorityCount);
    }

    private static PolygonOps.PreparedBoundaryOffset Prepared(IReadOnlyList<PointD> outer,
        IEnumerable<IReadOnlyList<PointD>>? holes, double constructionTolerance)
        => PolygonOps.PrepareBoundaryOffset(outer, holes, 1 / PathUtils.Scale, 0,
            sourceConstructionTolerance: constructionTolerance, sourceConstructionSweepRadius: 3);

    private static List<PointD> Circle(double x, double y, double radius, int count)
        => Enumerable.Range(0, count).Select(index => {
            double angle = 2 * Math.PI * index / count;
            return new PointD(x + radius * Math.Cos(angle), y + radius * Math.Sin(angle));
        }).ToList();

    private static string[] Commands(IEnumerable<IReadOnlyList<PointD>> paths)
        => paths.Select(path => string.Join(";", path.Select(point => $"{point.x:R},{point.y:R}"))).ToArray();
}
