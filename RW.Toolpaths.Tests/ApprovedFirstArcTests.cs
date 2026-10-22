using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Approved first arc controls", DisableParallelization = true)]
public sealed class ApprovedFirstArcCollection { }

[Collection("Approved first arc controls")]
public sealed class ApprovedFirstArcTests
{
    [Theory]
    [InlineData(6, 1)]
    [InlineData(5.999, 0)]
    public void OriginalTangentAuthorityRetainsExactWidthAndRejectsUndersizedOpening(double width, int count)
    {
        var region = new MillingRegion(new List<PointD> { new(0, 0), new(width, 0), new(width, 60), new(0, 60) });
        var fill = Fill(region, true);
        Assert.Equal(count, fill.Count);
        Assert.All(fill, path => Assert.False(path.IsClosed));
        ProveOriginalChords(region, fill);
        if (count != 0)
        {
            var trace = Assert.Single(fill);
            Assert.True(Math.Abs(trace.Points[0].y - trace.Points[^1].y) >= 53.99);
        }
    }

    [Fact]
    public void ExactWidthStemRemainsBesideTheWiderRoom()
    {
        var region = new MillingRegion(new List<PointD> {
            new(0, 0), new(6, 0), new(6, 44), new(16, 44), new(16, 70),
            new(-10, 70), new(-10, 44), new(0, 44) });
        var fill = Fill(region, true);
        Assert.Contains(fill, path => !path.IsClosed && path.Level == 0
            && path.Points.Min(point => point.y) <= 3.01 && path.Points.Max(point => point.y) >= 39);
        Assert.Contains(fill, path => path.IsClosed);
        ProveOriginalChords(region, fill);
    }

    [Fact]
    public void DenseSignedAnnulusPreservesOriginalWallsAndTheExistingCoverageContract()
    {
        var region = new MillingRegion(Circle(30, 9216), new[] { (IReadOnlyList<PointD>)Circle(8, 2048) });
        var reference = Fill(region, false);
        var candidate = Fill(region, true);
        ProveOriginalChords(region, candidate);
        List<List<PointD>> Sweep(List<OffsetRing> fill) => PolygonOps.Union(
            PolygonOps.BufferCenterlines(fill.Where(path => path.IsClosed).Select(path => (IReadOnlyList<PointD>)path.Points), 3, .001)
                .Concat(PolygonOps.BufferOpenCenterlines(fill.Where(path => !path.IsClosed).Select(path => (IReadOnlyList<PointD>)path.Points), 3, .001)));
        var lost = PolygonOps.Difference(Sweep(reference), Sweep(candidate));
        Assert.Empty(PolygonOps.Offset(lost, -(.003 + 4 / PathUtils.Scale), .00025, 0));
        // The central authored hole is never removed by changing arc construction.
        Assert.Empty(PolygonOps.Intersect(Sweep(candidate), new[] { Circle(7.99, 1024) }));
    }

    private static List<OffsetRing> Fill(MillingRegion region, bool experiment)
    {
        string? previous = Environment.GetEnvironmentVariable("RW_TOOLPATHS_FIRST_ARC_EXPERIMENT");
        string? reference = Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE");
        try
        {
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_FIRST_ARC_EXPERIMENT", experiment ? "1" : null);
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE", null);
            return MedialAxisGeometry.BuildConcentricFill(new[] { region },
                new RingOffsetOptions(3, 5.4) { MinimumRingArea = 0,
                    Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = 0 } }, 3, true);
        }
        finally
        {
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_FIRST_ARC_EXPERIMENT", previous);
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE", reference);
        }
    }

    private static void ProveOriginalChords(MillingRegion region, IEnumerable<OffsetRing> fill)
    {
        var original = PathUtils.ToClipper(PolygonOps.Difference(new[] { region.Outer }, region.Holes));
        var boundary = new PolygonOps.BoundarySegmentIndex(original);
        foreach (var path in fill)
        {
            var points = PathUtils.ToClipper(new[] { (IReadOnlyList<PointD>)path.Points })[0];
            int edges = path.IsClosed ? points.Count : points.Count - 1;
            for (int i = 0; i < edges; i++)
            {
                Assert.True(boundary.IsInside(points[i]));
                var violation = boundary.ClearanceViolation(points[i], points[(i + 1) % points.Count], 3 * PathUtils.Scale - 4);
                Assert.True(violation is null, $"Original wall violation: closed={path.IsClosed} level={path.Level} "
                    + $"a={points[i]} b={points[(i + 1) % points.Count]} wall={violation}.");
            }
        }
    }

    private static List<PointD> Circle(double radius, int count) => Enumerable.Range(0, count)
        .Select(i => new PointD(radius * Math.Cos(Math.Tau * i / count), radius * Math.Sin(Math.Tau * i / count))).ToList();
}
