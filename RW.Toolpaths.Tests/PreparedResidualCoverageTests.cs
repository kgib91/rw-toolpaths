using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class PreparedResidualCoverageTests
{
    private const double Error = .003 + 4 / PathUtils.Scale;
    private const double Arc = .001;

    [Fact]
    public void EnclosingOuterHoleAndIslandRemainWholeEvenWhenTheirEdgesMissQuery()
    {
        var hole = Rect(-8, -8, 8, 8); hole.Reverse();
        var coverage = Canonical(Rect(-10, -10, 10, 10), hole, Rect(-1, -1, 1, 1), Rect(100, 100, 110, 110));
        var prepared = new PreparedResidualCoverage(coverage, CancellationToken.None);
        foreach (var subject in new[] { Canonical(Rect(-3, -3, 3, 3)), Canonical(Rect(2, 2, 4, 4)),
                     Canonical(Rect(-.5, -.5, .5, .5)) })
            EqualDifference(coverage, subject, prepared.Difference(subject, CancellationToken.None));
        Assert.True(prepared.SelectedVertices < prepared.ConsideredVertices);
        Assert.True(prepared.InputReuses > 0);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(17)]
    [InlineData(45)]
    [InlineData(89)]
    public void ExactNativeResidualAndErosionMatchAtTangentAndHalfQuantumBoundaries(double degrees)
    {
        double angle = degrees * Math.PI / 180;
        foreach (double shift in new[] { -.5 / PathUtils.Scale, .5 / PathUtils.Scale })
        {
            var outer = Rotate(Rect(-5, -5, 5, 5), angle, shift);
            var hole = Rotate(Rect(-2, -2, 2, 2), angle, shift); hole.Reverse();
            var coverage = Canonical(outer, hole, Rect(20, 20, 22, 22));
            var prepared = new PreparedResidualCoverage(coverage, CancellationToken.None);
            foreach (double x in new[] { -5d, -2d, -2 + Error, -2 - Error, 0, 2d, 5d, 20d })
            {
                var subject = Canonical(Rotate(Rect(x, -.1, x + .2, .1), angle, shift));
                var actual = prepared.Difference(subject, CancellationToken.None);
                var expected = Reference(coverage, subject);
                EqualPaths(expected, actual);
                Assert.Equal(ErodedArea(expected) == 0, ErodedArea(actual) == 0);
            }
        }
    }

    [Fact]
    public void RepeatedDistantQueriesReuseOriginalVerticesAndImmutableSnapshot()
    {
        var coverage = Canonical(Enumerable.Range(0, 96).Select(i => (IReadOnlyList<PointD>)Rect(i * 100, 0, i * 100 + 10, 10)).ToArray());
        var frozen = new Paths64(coverage.Select(path => new Path64(path)));
        var prepared = new PreparedResidualCoverage(coverage, CancellationToken.None);
        coverage[0].Clear(); coverage.Clear();
        for (int i = 0; i < 8; i++)
        {
            var subject = Canonical(Rect(-1 + i / 100d, 1, 5, 9));
            EqualDifference(frozen, subject, prepared.Difference(subject, CancellationToken.None));
        }
        Assert.Equal(1, prepared.InputBuilds);
        Assert.Equal(7, prepared.InputReuses);
        Assert.True(prepared.SelectedVertices * 90 < prepared.ConsideredVertices);
    }

    [Fact]
    public void CompleteDiscFindsVirginHoleAlthoughEveryCandidateVertexIsAlreadyCovered()
    {
        var hole = Rect(-1, -1, 1, 1); hole.Reverse();
        var coverage = Canonical(Rect(-10, -10, 10, 10), hole);
        var candidate = Canonical(Rect(-2, -2, 2, 2));
        var coveredBoundary = new PolygonOps.BoundarySegmentIndex(coverage);
        Assert.All(candidate[0], point => Assert.True(coveredBoundary.IsInside(point)));
        var prepared = new PreparedResidualCoverage(coverage, CancellationToken.None);
        Assert.True(prepared.TryFreshDisc(candidate, Error, Arc, CancellationToken.None, out var witness));
        Assert.False(coveredBoundary.IsInside(witness));
        Assert.True(coveredBoundary.DistanceSquared(witness)
            > Math.Pow(prepared.LastWitnessRadius * PathUtils.Scale + 4, 2));
        Assert.True(new PolygonOps.BoundarySegmentIndex(candidate).ContainsCapsule(witness, witness,
            prepared.LastWitnessRadius * PathUtils.Scale));
        Assert.True(ErodedArea(Reference(coverage, candidate)) > 0);
    }

    [Fact]
    public void CandidateHoleThinSliverAndTouchingBoundaryCannotInventFreshDisc()
    {
        var hole = Rect(-2, -2, 2, 2); hole.Reverse();
        var coverage = Canonical(Rect(-5, -5, 5, 5), hole);
        var prepared = new PreparedResidualCoverage(coverage, CancellationToken.None);
        foreach (var candidate in new[] { coverage, Canonical(Rect(5, 0, 5.001, 1)),
                     Canonical(Rect(-1, 0, 1, .001)), Canonical(Rect(3, 3, 5, 5)) })
        {
            Assert.False(prepared.TryFreshDisc(candidate, Error, Arc, CancellationToken.None, out _));
            EqualDifference(coverage, candidate, prepared.Difference(candidate, CancellationToken.None));
            Assert.Equal(0, ErodedArea(Reference(coverage, candidate)));
        }
    }

    [Fact]
    public void PartiallyClippedPhysicalCoverageOnlyCreditsItsActualFilledDomain()
    {
        var coverage = Canonical(Rect(0, -5, 5, 5));
        var candidate = Canonical(Rect(-3, -2, 3, 2));
        var prepared = new PreparedResidualCoverage(coverage, CancellationToken.None);
        Assert.True(prepared.TryFreshDisc(candidate, Error, Arc, CancellationToken.None, out var witness));
        Assert.True(witness.X < 0);
        Assert.True(ErodedArea(Reference(coverage, candidate)) > 0);
    }

    [Fact]
    public void FreshDiscDecisionsAlwaysHaveIndependentOriginalMorphologyWitnesses()
    {
        var random = new Random(32452843);
        var hole = Rect(-2, -2, 2, 2); hole.Reverse();
        var coverage = Canonical(Rect(-8, -8, 8, 8), hole, Rect(15, -4, 20, 4));
        var prepared = new PreparedResidualCoverage(coverage, CancellationToken.None);
        int positives = 0;
        for (int i = 0; i < 192; i++)
        {
            double x = random.NextDouble() * 32 - 10, y = random.NextDouble() * 20 - 10;
            var candidate = Canonical(Rect(x, y, x + random.NextDouble() * 3 + .002,
                y + random.NextDouble() * 3 + .002));
            if (!prepared.TryFreshDisc(candidate, Error, Arc, CancellationToken.None, out _)) continue;
            positives++;
            Assert.True(ErodedArea(Reference(coverage, candidate)) > 0);
        }
        Assert.True(positives > 32);
    }

    [Fact]
    public void CancellationCannotPublishPartialInputAndLargeCoordinatesUseOriginalMorphology()
    {
        var coverage = Canonical(Rect(0, 0, 10, 10));
        var candidate = Canonical(Rect(-1, -1, 1, 1));
        var prepared = new PreparedResidualCoverage(coverage, CancellationToken.None);
        using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => new PreparedResidualCoverage(coverage, cancelled.Token));
        Assert.Throws<OperationCanceledException>(() => prepared.Difference(candidate, cancelled.Token));
        Assert.Throws<OperationCanceledException>(() => prepared.TryFreshDisc(candidate, Error, Arc, cancelled.Token, out _));
        Assert.Equal(0, prepared.InputBuilds);
        EqualDifference(coverage, candidate, prepared.Difference(candidate, CancellationToken.None));
        var large = Canonical(Rect(40_000_000, 0, 40_000_010, 10));
        var largeCandidate = Canonical(Rect(39_999_999, 1, 40_000_002, 2));
        var guarded = new PreparedResidualCoverage(large, CancellationToken.None);
        Assert.False(guarded.TryFreshDisc(largeCandidate, Error, Arc, CancellationToken.None, out _));
        EqualDifference(large, largeCandidate, guarded.Difference(largeCandidate, CancellationToken.None));
    }

    private static void EqualDifference(Paths64 coverage, Paths64 subject, List<List<PointD>> actual)
        => EqualPaths(Reference(coverage, subject), actual);

    private static List<List<PointD>> Reference(Paths64 coverage, Paths64 subject)
        => PolygonOps.Difference(PathUtils.FromClipper(subject), PathUtils.FromClipper(coverage));

    private static void EqualPaths(List<List<PointD>> expected, List<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++) Assert.Equal(expected[i].ToArray(), actual[i].ToArray());
    }

    private static double ErodedArea(List<List<PointD>> residual)
        => Math.Abs(PolygonOps.Offset(residual, -Error, Arc / 4, 0).Sum(PolygonOps.SignedArea));

    private static Paths64 Canonical(params IReadOnlyList<PointD>[] paths)
        => Clipper.Union(PathUtils.ToClipper(paths), FillRule.NonZero);

    private static List<PointD> Rect(double x0, double y0, double x1, double y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };

    private static List<PointD> Rotate(List<PointD> path, double angle, double shift)
        => path.Select(point => new PointD(point.x * Math.Cos(angle) - point.y * Math.Sin(angle) + shift,
            point.x * Math.Sin(angle) + point.y * Math.Cos(angle) - shift)).ToList();
}
