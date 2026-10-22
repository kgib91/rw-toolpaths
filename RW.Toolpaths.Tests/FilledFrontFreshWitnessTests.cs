using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class FilledFrontFreshWitnessTests
{
    [Fact]
    public void DisjointFrontsKeepExactOutputWithoutAnyAccumulatedUnion()
    {
        var polygons = Enumerable.Range(0, 64).Select(index =>
            Rectangle(index % 8 * 8, index / 8 * 8, 2, 2)).ToList();
        FilledFrontCoverage.Statistics stats = default;
        var actual = FilledFrontCoverage.RetainUncoveredComponents(polygons, .7,
            CancellationToken.None, observe: value => stats = value);
        var reference = FilledFrontCoverage.RetainUncoveredComponents(polygons, .7,
            CancellationToken.None, useFreshWitness: false);

        EqualPaths(reference, actual);
        Assert.Equal(64, stats.Retained);
        Assert.Equal(63, stats.FreshWitnesses);
        Assert.Equal(0, stats.BooleanFallbacks);
        Assert.Equal(0, stats.OrderedUnionCalls);
        Assert.Equal(0, stats.FootprintQueries);
    }

    [Theory]
    [InlineData(.1)]
    [InlineData(.7)]
    [InlineData(2)]
    [InlineData(5)]
    public void NearbyAndNestedFrontsPreserveOriginalOrderedBooleanDecisions(double radius)
    {
        var random = new Random(982451653);
        for (int fixture = 0; fixture < 24; fixture++)
        {
            var polygons = new List<List<PointD>>();
            for (int component = 0; component < 16; component++)
            {
                double x = component % 4 * 3.5 + random.NextDouble() * .2;
                double y = component / 4 * 3.5 + random.NextDouble() * .2;
                polygons.Add(Rectangle(x, y, .15 + random.NextDouble() * 2.8,
                    .15 + random.NextDouble() * 2.8));
            }
            EqualPaths(FilledFrontCoverage.RetainUncoveredComponents(polygons, radius,
                CancellationToken.None, useFreshWitness: false),
                FilledFrontCoverage.RetainUncoveredComponents(polygons, radius,
                    CancellationToken.None));
        }
    }

    [Fact]
    public void CompleteLateralFootprintRetainsAFrontWhoseCenterlineIsCovered()
    {
        // The first cutter sweep covers every point of the second centerline,
        // but leaves fresh material around its far lateral edge. Centerline-only
        // coverage would incorrectly discard this component.
        var polygons = new List<List<PointD>>
        {
            Rectangle(0, 0, 10, 10), Rectangle(10.2, 4, .2, 2),
        };
        var actual = FilledFrontCoverage.RetainUncoveredComponents(polygons, 1,
            CancellationToken.None);
        EqualPaths(FilledFrontCoverage.RetainUncoveredComponents(polygons, 1,
            CancellationToken.None, useFreshWitness: false), actual);
        Assert.Equal(2, actual.Count);
    }

    [Fact]
    public void HolesAndNestedIslandsStayInTheSameNonZeroDomain()
    {
        var hole = Rectangle(2, 2, 16, 16);
        hole.Reverse();
        var polygons = new List<List<PointD>>
        {
            Rectangle(0, 0, 20, 20), hole, Rectangle(8, 8, 4, 4),
            Rectangle(25, 0, 2, 2),
        };
        EqualPaths(FilledFrontCoverage.RetainUncoveredComponents(polygons, .4,
            CancellationToken.None, useFreshWitness: false),
            FilledFrontCoverage.RetainUncoveredComponents(polygons, .4,
                CancellationToken.None));
    }

    [Fact]
    public void InconclusiveFreshDiscUsesOriginalBooleanFallback()
    {
        var polygons = new List<List<PointD>>
        {
            Rectangle(0, 0, 10, 10), Rectangle(10.01, 4, .001, 2),
        };
        FilledFrontCoverage.Statistics stats = default;
        var actual = FilledFrontCoverage.RetainUncoveredComponents(polygons, 2,
            CancellationToken.None, observe: value => stats = value);
        EqualPaths(FilledFrontCoverage.RetainUncoveredComponents(polygons, 2,
            CancellationToken.None, useFreshWitness: false), actual);
        Assert.True(stats.BooleanFallbacks > 0);
        Assert.True(stats.OrderedUnionCalls > 0);
    }

    [Fact]
    public void ClippedAwayStrokesNeverAuthorizeAComponentDiscard()
    {
        var polygons = new List<List<PointD>>
        {
            Rectangle(-3, -3, 8, 8), Rectangle(6, 0, 1, 1),
        };
        var clip = Rectangle(0, -10, 10, 20);
        EqualPaths(FilledFrontCoverage.RetainUncoveredComponents(polygons, 2,
            CancellationToken.None, clip, useFreshWitness: false),
            FilledFrontCoverage.RetainUncoveredComponents(polygons, 2,
                CancellationToken.None, clip));
    }

    [Fact]
    public void MirroredFrontsAndZeroRadiusMatchTheReference()
    {
        var polygons = new List<List<PointD>>
        {
            Rectangle(0, 0, 4, 4), Rectangle(6, 0, 2, 2), Rectangle(9, 0, 1, 1),
        };
        foreach (var polygon in polygons)
        {
            for (int index = 0; index < polygon.Count; index++)
                polygon[index] = new PointD(-polygon[index].x, polygon[index].y);
            polygon.Reverse();
        }
        foreach (double radius in new[] { 0, .0001, .5, 3d })
            EqualPaths(FilledFrontCoverage.RetainUncoveredComponents(polygons, radius,
                CancellationToken.None, useFreshWitness: false),
                FilledFrontCoverage.RetainUncoveredComponents(polygons, radius,
                    CancellationToken.None));
    }

    [Fact]
    public void CancellationDoesNotPublishPartialCoverage()
    {
        using var cancel = new CancellationTokenSource();
        cancel.Cancel();
        bool observed = false;
        Assert.Throws<OperationCanceledException>(() =>
            FilledFrontCoverage.RetainUncoveredComponents(new[] { Rectangle(0, 0, 2, 2) },
                1, cancel.Token, observe: _ => observed = true));
        Assert.False(observed);
    }

    private static List<PointD> Rectangle(double x, double y, double width, double height) =>
        new() { new(x, y), new(x + width, y), new(x + width, y + height), new(x, y + height) };

    private static void EqualPaths(IReadOnlyList<List<PointD>> expected,
        IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int ring = 0; ring < expected.Count; ring++)
        {
            Assert.Equal(expected[ring].Count, actual[ring].Count);
            for (int point = 0; point < expected[ring].Count; point++)
            {
                Assert.Equal(expected[ring][point].x, actual[ring][point].x);
                Assert.Equal(expected[ring][point].y, actual[ring][point].y);
            }
        }
    }
}
