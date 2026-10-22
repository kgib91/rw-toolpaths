using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarvePreparedCorrectionTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(301.41172)]
    [InlineData(-100000)]
    public void PreparedNeighborhoodKeepsExactOrderedCoordinatesAcrossRepeatedRadii(double origin)
    {
        IReadOnlyList<IReadOnlyList<PointD>> rings = new[]
        {
            new PointD[] { new(origin, 0), new(origin + 5, .001), new(origin + 5, 5),
                new(origin + 2, 5), new(origin + 2, 2), new(origin, 2) },
            new PointD[] { new(origin + 3, 1), new(origin + 3, 1.6), new(origin + 4, 1.6), new(origin + 4, 1) },
        };
        var prepared = new VCarveSmoothing.PreparedCorrectionNeighborhood(rings);
        double[] radii = [1.2, .6, .3, .06433826408401311, .01, 0, .3, 1.2];
        foreach (double radius in radii)
            Equal(ReferenceNeighborhood(rings, radius), prepared.Expand(radius));
        Assert.Equal(radii.Length, prepared.Queries);
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => prepared.Expand(.3, cancelled.Token));
        Assert.Equal(radii.Length, prepared.Queries);
    }

    [Fact]
    public void PreparedNeighborhoodSnapshotsInputsAndReturnsIndependentResults()
    {
        var ring = new List<PointD> { new(0, 0), new(4, 0), new(4, 4), new(0, 4) };
        var prepared = new VCarveSmoothing.PreparedCorrectionNeighborhood(new[] { ring });
        var expected = prepared.Expand(.1);
        var first = prepared.Expand(.1);
        ring[0] = new(100, 100);
        first[0][0] = new(100, 100);
        Equal(expected, prepared.Expand(.1));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(100000)]
    public void DisjointCenterBroadPhaseKeepsCompleteOriginalIntersectionIncludingNestingAndHoles(double origin)
    {
        IReadOnlyList<IReadOnlyList<PointD>> error =
        [
            new PointD[] { new(origin, 0), new(origin + 6, 0), new(origin + 6, 5),
                new(origin + 4, 5), new(origin + 4, 2), new(origin + 2, 2),
                new(origin + 2, 5), new(origin, 5) },
            new PointD[] { new(origin + .5, .5), new(origin + .5, 1.5),
                new(origin + 1.5, 1.5), new(origin + 1.5, .5) },
        ];
        var prepared = new VCarveSmoothing.PreparedCorrectionNeighborhood(error);
        int attempts = 0;
        foreach (double reach in new[] { 0d, .01, .1, .5, 1, 3 })
        foreach (double x in new[] { -10d, -1, 0, .7, 2.1, 2.9, 4, 5.9, 8 })
        foreach (double width in new[] { .01, .8, 12 })
        {
            // Long crossing chords, enclosing rectangles and centers wholly inside
            // an error hole challenge both the boundary and filled-nesting tests.
            var legal = new List<List<PointD>>
            {
                new() { new(origin + x, .8), new(origin + x + width, .8),
                    new(origin + x + width, 1.2), new(origin + x, 1.2) },
            };
            Equal(PolygonOps.Intersect(legal, ReferenceNeighborhood(error, reach)),
                prepared.SelectCenters(legal, reach));
            attempts++;
        }
        Assert.Equal(attempts, prepared.SelectionQueries);
        Assert.True(prepared.DisjointSkips > 0);
        Assert.True(prepared.Queries > 0);
        Assert.True(prepared.Queries < attempts);
        Console.WriteLine($"[VCarve.CorrectionContactBroadPhase.Test] origin={origin:R} "
            + $"selectionQueries={prepared.SelectionQueries} exactOffsets={prepared.Queries} "
            + $"disjointSkips={prepared.DisjointSkips} contactProbes={prepared.ContactProbes} orderedOutputEqual=True");
    }

    [Fact]
    public void DisjointCenterBroadPhaseDoesNotTurnAnInconclusiveNominationIntoStockAuthority()
    {
        IReadOnlyList<IReadOnlyList<PointD>> error =
            [new PointD[] { new(0, 0), new(1, 0), new(1, 1), new(0, 1) }];
        var prepared = new VCarveSmoothing.PreparedCorrectionNeighborhood(error);
        var enclosing = new List<List<PointD>>
            { new() { new(-10, -10), new(10, -10), new(10, 10), new(-10, 10) } };
        Equal(PolygonOps.Intersect(enclosing, ReferenceNeighborhood(error, .1)),
            prepared.SelectCenters(enclosing, .1));
        var enclosed = new List<List<PointD>>
            { new() { new(.2, .2), new(.8, .2), new(.8, .8), new(.2, .8) } };
        Equal(PolygonOps.Intersect(enclosed, ReferenceNeighborhood(error, 0)),
            prepared.SelectCenters(enclosed, 0));
        Assert.Equal(0, prepared.DisjointSkips);
        var empty = new List<List<PointD>>();
        Assert.Empty(prepared.SelectCenters(empty, .1));
        Assert.Equal(1, prepared.DisjointSkips);
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => prepared.SelectCenters(empty, .1, cancelled.Token));
        Assert.Equal(3, prepared.SelectionQueries);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(100000)]
    public void PreparedCropMatchesFullSourceWithEnclosingRingsHolesAndTangencies(double origin)
    {
        var rings = new List<IReadOnlyList<PointD>>
        {
            new PointD[] { new(origin, 0), new(origin + 20, 0), new(origin + 20, 20), new(origin, 20) },
            new PointD[] { new(origin + 8, 3), new(origin + 8, 17), new(origin + 12, 17), new(origin + 12, 3) },
            new PointD[] { new(origin + 30, 0), new(origin + 40, 0), new(origin + 40, 20), new(origin + 30, 20) },
        };
        var cropper = new VCarveSmoothing.CorrectionBoundary(rings);
        foreach (double x in new[] { -3.0, 0, 6, 8, 10, 12, 18, 22, 30, 50 })
        foreach (double radius in new[] { .001, .5, 1.0, 25 })
        {
            PointD[] error = [new(origin + x, 5), new(origin + x + .01, 5),
                new(origin + x + .01, 5.01), new(origin + x, 5.01)];
            Equal(ReferenceCrop(rings, error, radius, radius),
                cropper.Crop(error, radius, additionalCenterReach: radius));
        }
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => cropper.Crop(rings[0], 1, cancelled.Token));
    }

    [Fact]
    public void PreparedCropRejectsOnlyDisjointCompleteRingsAndRetainsIntegerOrdering()
    {
        var rings = Enumerable.Range(0, 128).Select(island => (IReadOnlyList<PointD>)Enumerable.Range(0, 64)
            .Select(vertex => new PointD(island * 30 + 5 * Math.Cos(vertex * Math.PI / 32),
                5 * Math.Sin(vertex * Math.PI / 32))).ToArray()).ToArray();
        var cropper = new VCarveSmoothing.CorrectionBoundary(rings);
        for (int island = 0; island < 128; island += 7)
        {
            PointD[] error = [new(island * 30, 0), new(island * 30 + .01, 0),
                new(island * 30 + .01, .01), new(island * 30, .01)];
            Equal(ReferenceCrop(rings, error, .2, 0), cropper.Crop(error, .2));
        }
        string statistics = cropper.DescribeQueries();
        Assert.Contains("orderedCompleteRings=True", statistics);
        Assert.DoesNotContain("excludedPoints=0 ", statistics);
        Console.WriteLine("[VCarve.PreparedCrop.Test] " + statistics);
    }

    private static List<List<PointD>> ReferenceNeighborhood(IEnumerable<IReadOnlyList<PointD>> rings, double reach)
    {
        var canonical = Clipper.Union(Integer(rings), FillRule.NonZero);
        var offset = new ClipperOffset(arcTolerance: 1);
        offset.AddPaths(canonical, JoinType.Round, EndType.Polygon);
        var expanded = new Paths64();
        offset.Execute(reach * PathUtils.Scale + 4, expanded);
        return Convert(expanded);
    }

    private static List<List<PointD>> ReferenceCrop(IEnumerable<IReadOnlyList<PointD>> rings,
        IReadOnlyList<PointD> error, double radius, double reach)
    {
        var canonical = Clipper.Union(Integer(rings), FillRule.NonZero);
        var bounds = Clipper.GetBounds(canonical);
        double coordinate = Math.Max(Math.Max(Math.Abs((double)bounds.left), Math.Abs((double)bounds.right)),
            Math.Max(Math.Abs((double)bounds.top), Math.Abs((double)bounds.bottom))) / PathUtils.Scale;
        float command = (float)coordinate;
        double reserve = .01 + Math.Sqrt(2) * (MathF.BitIncrement(command) - command) / 2;
        double padding = 2 * radius + reach + VCarveSmoothing.CoverageTolerance + reserve;
        var crop = new Rect64(
            (long)Math.Floor((error.Min(point => point.x) - padding) * PathUtils.Scale),
            (long)Math.Floor((error.Min(point => point.y) - padding) * PathUtils.Scale),
            (long)Math.Ceiling((error.Max(point => point.x) + padding) * PathUtils.Scale),
            (long)Math.Ceiling((error.Max(point => point.y) + padding) * PathUtils.Scale));
        if (crop.left <= bounds.left && crop.top <= bounds.top && crop.right >= bounds.right && crop.bottom >= bounds.bottom)
            return Convert(canonical);
        return Convert(Clipper.Union(Clipper.RectClip(crop, canonical), FillRule.NonZero));
    }

    private static Paths64 Integer(IEnumerable<IReadOnlyList<PointD>> rings)
        => new(rings.Select(ring => new Path64(ring.Select(point => new Point64(
            (long)Math.Round(point.x * PathUtils.Scale), (long)Math.Round(point.y * PathUtils.Scale))))));

    private static List<List<PointD>> Convert(Paths64 paths)
        => paths.Select(ring => ring.Select(point => new PointD(point.X / PathUtils.Scale, point.Y / PathUtils.Scale)).ToList()).ToList();

    private static void Equal(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++) Assert.Equal(expected[i], actual[i]);
    }
}
