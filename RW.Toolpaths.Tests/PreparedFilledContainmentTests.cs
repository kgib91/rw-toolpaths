using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class PreparedFilledContainmentTests
{
    [Fact]
    public void EnclosedHoleRejectsFilledCandidateWhoseEveryVertexIsInside()
    {
        var hole = Rect(4, 4, 6, 6); hole.Reverse();
        var region = Canonical(Rect(0, 0, 10, 10), hole);
        var candidate = Canonical(Rect(2, 2, 8, 8));
        var prepared = new PreparedFilledContainment(region);
        Assert.False(prepared.ContainsCanonical(candidate, CancellationToken.None));
        Assert.Equal(1, prepared.EnclosedHoleQueries);
        Assert.NotEmpty(Clipper.Difference(candidate, region, FillRule.NonZero));
    }

    [Fact]
    public void CandidateHoleAndNestedFilledIslandRetainNonZeroContainment()
    {
        var hole = Rect(3, 3, 7, 7); hole.Reverse();
        var candidateHole = Rect(2, 2, 8, 8); candidateHole.Reverse();
        var region = Canonical(Rect(0, 0, 10, 10), hole, Rect(4, 4, 6, 6));
        var candidate = Canonical(Rect(1, 1, 9, 9), candidateHole, Rect(4.5, 4.5, 5.5, 5.5));
        Assert.True(new PreparedFilledContainment(region).ContainsCanonical(candidate, CancellationToken.None));
        Assert.Empty(Clipper.Difference(candidate, region, FillRule.NonZero));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(2)]
    [InlineData(-2)]
    public void TouchingAndQuantizedNearBoundariesUseExistingBooleanPolicy(double quanta)
    {
        var region = Canonical(Rect(0, 0, 10, 10));
        var candidate = Canonical(Rect(quanta / PathUtils.Scale, 1, 9, 9));
        Assert.Null(new PreparedFilledContainment(region).ContainsCanonical(candidate, CancellationToken.None));
    }

    [Fact]
    public void StrictProofsAgreeWithFrozenBooleanPredicateAcrossMirroredHolesAndDisjointComponents()
    {
        var random = new Random(179426549);
        for (int i = 0; i < 256; i++)
        {
            var outer = Rect(-10, -10, 10, 10);
            var hole = Rect(-2, -2, 2, 2); hole.Reverse();
            var region = Canonical(outer, hole, Rect(20, 0, 30, 10));
            double x = random.NextDouble() * 44 - 12, y = random.NextDouble() * 24 - 12;
            var candidate = Canonical(Rect(x, y, x + random.NextDouble() * 5 + .1, y + random.NextDouble() * 5 + .1));
            if (i % 2 == 0)
            {
                foreach (var path in region.Concat(candidate))
                {
                    for (int j = 0; j < path.Count; j++) path[j] = new Point64(-path[j].X, path[j].Y);
                    path.Reverse();
                }
            }
            var result = new PreparedFilledContainment(region).ContainsCanonical(candidate, CancellationToken.None);
            if (result.HasValue)
                Assert.Equal(Clipper.Difference(candidate, region, FillRule.NonZero).Count == 0, result.Value);
        }
    }

    [Fact]
    public void EmptyRegionsMutationAndCancellationKeepSnapshotContract()
    {
        var region = Canonical(Rect(0, 0, 10, 10));
        var prepared = new PreparedFilledContainment(region);
        region[0].Clear(); region.Clear();
        Assert.True(prepared.ContainsCanonical(Canonical(Rect(1, 1, 9, 9)), CancellationToken.None));
        Assert.True(prepared.ContainsCanonical(new Paths64(), CancellationToken.None));
        Assert.False(new PreparedFilledContainment(new Paths64()).ContainsCanonical(Canonical(Rect(1, 1, 9, 9)), CancellationToken.None));
        using var cancel = new CancellationTokenSource(); cancel.Cancel();
        Assert.Throws<OperationCanceledException>(() => prepared.ContainsCanonical(new Paths64(), cancel.Token));
    }

    [Fact]
    public void FrontContainmentShortcutMatchesOriginalTolerantBooleanDecisions()
    {
        var paths = new List<List<PointD>> { Rect(0, 0, 10, 10), Rect(2, 2, 8, 8),
            Rect(2.001, 2.001, 7.999, 7.999), Rect(10.01, 4, 10.011, 6), Rect(20, 0, 22, 2) };
        foreach (double radius in new[] { .1, .7, 2d, 5d })
        {
            var expected = FilledFrontCoverage.RetainUncoveredComponents(paths, radius,
                CancellationToken.None, useFreshWitness: false, usePreparedContainment: false);
            var actual = FilledFrontCoverage.RetainUncoveredComponents(paths, radius,
                CancellationToken.None);
            Assert.Equal(expected.Count, actual.Count);
            for (int i = 0; i < actual.Count; i++) Assert.Equal(expected[i], actual[i]);
        }
    }

    private static Paths64 Canonical(params IReadOnlyList<PointD>[] paths)
        => Clipper.Union(PathUtils.ToClipper(paths), FillRule.NonZero);

    private static List<PointD> Rect(double x0, double y0, double x1, double y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
}
