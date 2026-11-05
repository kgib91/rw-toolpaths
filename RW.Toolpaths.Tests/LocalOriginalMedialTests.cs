using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Local original medial diagnostic controls", DisableParallelization = true)]
public sealed class LocalOriginalMedialDiagnosticCollection { }

[Collection("Local original medial diagnostic controls")]
public sealed class LocalOriginalMedialTests
{
    [Fact]
    public void ProtectedLocalAndFullSourcesShareTheSameGlobalGridConditioning()
    {
        double tick = 1 / PathUtils.Scale;
        var raw = new MillingRegion(Rectangle(0, 0, 400 + tick, 400 + tick),
            Enumerable.Range(0, 25).Select(index => Circle(20 + (index % 5) * 80 + .5 * tick,
                20 + (index / 5) * 80 + .5 * tick, 5, 64)).ToArray());
        var grid = PathUtils.FromClipper(PathUtils.ToClipper(new[] { raw.Outer }.Concat(raw.Holes)));
        var expected = new PreparedTerminalMedial(new MillingRegion(grid[0], grid.Skip(1).ToArray()),
            default, lazy: true, preserveNativeGrid: true);
        var actual = new PreparedTerminalMedial(raw, default, lazy: true, preserveNativeGrid: true);
        var window = PathUtils.ToClipper(new[] { Rectangle(8, 8, 38, 38) });
        var reference = expected.Intersecting(window, default);
        var result = actual.Intersecting(window, default);
        Assert.True(expected.LastQueryWasLocal);
        Assert.True(actual.LastQueryWasLocal);
        Assert.NotEmpty(reference);
        Assert.Equal(reference, result);
    }

    [Fact]
    public void OmittedFiniteSitesCannotWinAnywhereInTheCertifiedQueryWindow()
    {
        var source = Sites();
        var query = new RectD(-400, -300, 400, 300);
        var local = Assert.IsType<BoostVoronoiProvider.LocalDiagram>(BoostVoronoiProvider.TryConstructLocalMedialAxis(
            source, query, .05, CancellationToken.None));
        Assert.True(local.OriginalSiteOrdinals.Count < source.Sites.Count / 4);
        Assert.All(source.Sites.Where(site => site.Outer), site => Assert.Contains(site.Ordinal, local.OriginalSiteOrdinals));
        var retained = local.OriginalSiteOrdinals.Select(index => source.Sites[index]).ToArray();
        for (int yi = 0; yi <= 24; yi++)
        for (int xi = 0; xi <= 32; xi++)
        {
            var point = new PointD(query.left + xi * (query.right - query.left) / 32,
                query.top + yi * (query.bottom - query.top) / 24);
            double original = source.Sites.Min(site => DistanceSquared(point, site.A, site.B));
            double selected = retained.Min(site => DistanceSquared(point, site.A, site.B));
            Assert.Equal(original, selected);
        }
        Assert.Equal(local.OriginalSiteOrdinals.Order(), local.OriginalSiteOrdinals);
    }

    [Fact]
    public void LocalQueriesOutsideTheOriginalOuterNeverOmitInfiniteEdges()
    {
        Assert.Null(BoostVoronoiProvider.TryConstructLocalMedialAxis(Sites(),
            new RectD(9900, 9900, 10100, 10100), .05, CancellationToken.None));
    }

    [Fact]
    public void QuadraticControlHullCertifiesFiniteChordsThroughAProjectionReversal()
    {
        // The tangent initially runs backwards along the long endpoint chord.
        // An infinite-line deviation alone does not bound endpoint distance.
        var edge = Parabola(-2, 10);
        var output = new List<MedialSegment>();
        const double error = .001;
        Assert.True(BoundedLocalMedialEmitter.TryEmit(edge, new RectD(-20, -10, 20, 30),
            error, 1e-10, output, CancellationToken.None));
        Assert.NotEmpty(output);
        Assert.Equal(-2, output[0].Point0.X);
        Assert.Equal(10, output[^1].Point1.X);
        for (int i = 0; i <= 4000; i++)
        {
            double x = -2 + 12d * i / 4000;
            var point = new PointD(x, (x * x + 4) / 4);
            double distance = Math.Sqrt(output.Min(segment => DistanceSquared(point,
                new(segment.Point0.X, segment.Point0.Y), new(segment.Point1.X, segment.Point1.Y))));
            Assert.InRange(distance, 0, error);
        }
    }

    [Fact]
    public void ACurveWhoseEndpointsMissTheWindowRetainsItsCrossingControlHull()
    {
        var output = new List<MedialSegment>();
        Assert.True(BoundedLocalMedialEmitter.TryEmit(Parabola(-10, 10),
            new RectD(-.5, .5, .5, 1.5), .001, 1e-10, output, CancellationToken.None));
        Assert.Contains(output, segment => Math.Min(segment.Point0.X, segment.Point1.X) <= 0
            && Math.Max(segment.Point0.X, segment.Point1.X) >= 0);
    }

    [Fact]
    public void IndeterminateCurvesLeaveNoPartialGeometry()
    {
        var edge = Parabola(-2, 10) with
        { Cell = new(true, new(0, 0), null) };
        var output = new List<MedialSegment>();
        Assert.False(BoundedLocalMedialEmitter.TryEmit(edge, new RectD(-20, -10, 20, 30),
            .001, 1e-10, output, CancellationToken.None));
        Assert.Empty(output);
    }

    [Fact]
    public void IllConditionedIntegerFocusCannotSpendAnUnboundedArithmeticError()
    {
        var a = new MedialAxisEdgeClassifier.EdgePoint(0, 0);
        var b = new MedialAxisEdgeClassifier.EdgePoint(1000000000, 999999999);
        var focus = new MedialAxisEdgeClassifier.EdgePoint(999999999, 999999998);
        // Exact determinant is -1 despite billion-unit coefficient products.
        var edge = new MedialAxisEdgeClassifier.EdgeData(0, 1, true, false, false,
            -1, -1, new(0, 1), new(100, 101), new(true, focus, null), new(false, null, new(a, b)))
            { IsCurved = true };
        var output = new List<MedialSegment>();
        Assert.False(BoundedLocalMedialEmitter.TryEmit(edge, new RectD(-10, -10, 110, 110),
            .001, 1e-10, output, CancellationToken.None));
        Assert.Empty(output);
    }

    [Fact]
    public void TerminalWallProofDoesNotSpendTheCurveApproximationAllowance()
    {
        var source = new MillingRegion(Rectangle(0, 0, 16, 60));
        var boundary = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(new[] { source.Outer }));
        var medial = new[] { new MedialSegment(new(2.9995, 8, 3), new(2.9995, 52, 3)) };
        object?[] arguments = { medial, new List<List<PointD>> { source.Outer.ToList() }, boundary,
            3d, null, CancellationToken.None, 0, 0 };
        var trace = typeof(OffsetTerminalPaths).GetMethod("TraceNative",
            System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic)!;
        var paths = Assert.IsType<List<List<PointD>>>(trace.Invoke(null, arguments));
        Assert.Empty(paths);
        Assert.True((int)arguments[7]! > 0);
    }

    [Fact]
    public void CompleteSourceConditioningAndSignedHolesRemainAuthoritative()
    {
        var region = new MillingRegion(Rectangle(0, 0, 400, 400),
            Enumerable.Range(0, 25).Select(index => Circle(20 + (index % 5) * 80,
                20 + (index / 5) * 80, 5, 64)).ToArray());
        var prepared = new PreparedTerminalMedial(region, CancellationToken.None, lazy: true);
        var query = Rectangle(8, 8, 38, 38);
        var local = prepared.Intersecting(PathUtils.ToClipper(new[] { query }), CancellationToken.None);
        Assert.True(prepared.LastQueryWasLocal);
        Assert.False(prepared.FullGraphMaterialized);
        Assert.NotEmpty(local);
        Assert.False(prepared.Boundary.IsInside(new Point64(20 * PathUtils.Scale, 20 * PathUtils.Scale)));
        Assert.True(prepared.Boundary.IsInside(new Point64(10 * PathUtils.Scale, 10 * PathUtils.Scale)));
        var again = prepared.Intersecting(PathUtils.ToClipper(new[] { query }), CancellationToken.None);
        Assert.Same(local, again);
    }

    [Theory]
    [InlineData("RW_TOOLPATHS_LOCAL_MEDIAL_REFERENCE")]
    [InlineData("RW_TOOLPATHS_COHERENCE_REFERENCE")]
    public void ExistingReferenceModesRestoreTheCompleteOriginalSiteGraph(string flag)
    {
        string? previous = Environment.GetEnvironmentVariable(flag);
        try
        {
            Environment.SetEnvironmentVariable(flag, "1");
            var region = new MillingRegion(Rectangle(0, 0, 200, 200),
                Enumerable.Range(0, 17).Select(index => Circle(10 + (index % 5) * 38,
                    10 + (index / 5) * 45, 3, 64)).ToArray());
            var prepared = new PreparedTerminalMedial(region, CancellationToken.None, lazy: true);
            prepared.Intersecting(PathUtils.ToClipper(new[] { Rectangle(1, 1, 20, 20) }), CancellationToken.None);
            Assert.False(prepared.LastQueryWasLocal);
            Assert.True(prepared.FullGraphMaterialized);
        }
        finally { Environment.SetEnvironmentVariable(flag, previous); }
    }

    private static BoostVoronoiProvider.LocalSiteSet Sites()
        => new(Rectangle(-10000, -10000, 10000, 10000),
            Enumerable.Range(0, 40).Select(index => Circle(index == 0 ? -100 : 2000 + (index % 8) * 700,
                index == 0 ? 0 : 2000 + (index / 8) * 700, 50, 32)).ToArray(), CancellationToken.None);

    private static MedialAxisEdgeClassifier.EdgeData Parabola(double x0, double x1)
        => new(0, 1, true, false, false, -1, -1,
            new(x0, (x0 * x0 + 4) / 4), new(x1, (x1 * x1 + 4) / 4),
            new(true, new(0, 2), null), new(false, null, new(new(-20, 0), new(20, 0))))
            { IsCurved = true, IsLinear = false };

    private static IReadOnlyList<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => new PointD[] { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
    private static IReadOnlyList<PointD> Circle(double x, double y, double radius, int points)
        => Enumerable.Range(0, points).Select(i => new PointD(x + radius * Math.Cos(2 * Math.PI * i / points),
            y + radius * Math.Sin(2 * Math.PI * i / points))).Reverse().ToArray();
    private static double DistanceSquared(PointD point, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y, squared = dx * dx + dy * dy;
        double t = squared == 0 ? 0 : Math.Clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / squared, 0, 1);
        double ex = point.x - a.x - t * dx, ey = point.y - a.y - t * dy;
        return ex * ex + ey * ey;
    }
}
