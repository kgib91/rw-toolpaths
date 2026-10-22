using Clipper2Lib;
using RW.Toolpaths;

namespace RW.Toolpaths.Tests;

public sealed class MedialAxisGeometryTests
{
    [Theory]
    [InlineData(100)]
    [InlineData(-100)]
    public void HalfGridSitesUseTheSignedMaskGridBeforeTranslation(double origin)
    {
        var provider = new CapturingProvider();
        double tick = 1 / PathUtils.Scale;
        PointD[] source = [new(origin + .5 * tick, 200 + .5 * tick), new(origin + 3.5 * tick, 200 + .5 * tick),
            new(origin + 3.5 * tick, 200 + 4.5 * tick), new(origin + .5 * tick, 200 + 4.5 * tick)];
        _ = MedialAxisGeometry.Build(source, [], provider, .01, 0, preserveNativeGrid: true);
        var expected = PathUtils.ToClipper(new[] { (IReadOnlyList<PointD>)source }).Single();
        for (int i = 0; i < source.Length; i++)
        {
            Assert.Equal(expected[i].X - expected[0].X, provider.Boundary[i].x - provider.Boundary[0].x);
            Assert.Equal(expected[i].Y - expected[0].Y, provider.Boundary[i].y - provider.Boundary[0].y);
        }
    }

    [Fact]
    public void GridAlignedSourceKeepsEveryFiniteSiteUnderConditioning()
    {
        var provider = new CapturingProvider();
        double tick = 1 / PathUtils.Scale;
        PointD[] source = [new(100, 200), new(100 + 3 * tick, 200 + tick),
            new(100 + 7 * tick, 200 + 9 * tick), new(100, 200 + 9 * tick)];
        _ = MedialAxisGeometry.Build(source, [], provider, .01, 0, preserveNativeGrid: true);
        Assert.All(provider.Boundary, p => { Assert.Equal(Math.Round(p.x), p.x); Assert.Equal(Math.Round(p.y), p.y); });
        for (int i = 1; i < source.Length; i++)
        {
            Assert.Equal((source[i].x - source[0].x) * PathUtils.Scale, provider.Boundary[i].x - provider.Boundary[0].x);
            Assert.Equal((source[i].y - source[0].y) * PathUtils.Scale, provider.Boundary[i].y - provider.Boundary[0].y);
        }
    }

    [Fact]
    public void ProviderReceivesCenteredCoordinatesAndReturnsPhysicalRadii()
    {
        var provider = new CapturingProvider();
        var diagnostics = new List<string>();
        PointD[] outer = Rectangle(1e12 - 4, -2e12 - 2, 1e12 + 4, -2e12 + 2);
        PointD[] hole = Rectangle(1e12 - 1, -2e12 - .5, 1e12 + 1, -2e12 + .5);

        MedialSegment actual = Assert.Single(MedialAxisGeometry.Build(outer, [hole], provider,
            .01, 1.25, Math.PI + 1e-6, diagnostic: diagnostics.Add));

        Assert.Equal(-4 * PathUtils.Scale, provider.Boundary.Min(point => point.x));
        Assert.Equal(4 * PathUtils.Scale, provider.Boundary.Max(point => point.x));
        Assert.Equal(-2 * PathUtils.Scale, provider.Boundary.Min(point => point.y));
        Assert.Equal(2 * PathUtils.Scale, provider.Boundary.Max(point => point.y));
        Assert.Equal(PathUtils.Scale, provider.Holes.Single().Max(point => point.x));
        Assert.Equal(.01, provider.Tolerance);
        Assert.Equal(1.25 * PathUtils.Scale, provider.MaximumRadius);
        Assert.Equal(Math.PI + 1e-6, provider.FilteringAngle);
        Assert.True(provider.UseBigIntegers);
        Assert.Equal(new MedialPoint(1e12, -2e12, 2), actual.Point0);
        Assert.Equal(new MedialPoint(1e12 + 2, -2e12, 2), actual.Point1);
        Assert.Contains(diagnostics, message => message.Contains("event=complete segments=1 outsideBounds=0"));
    }

    [Fact]
    public void VeryLargeShapeUsesBoundedProviderSitesWithoutCappingPhysicalRadius()
    {
        var provider = new CapturingProvider();
        PointD[] outer = Rectangle(1e12, -1e12, 1e12 + 1e9, -1e12 + 2e9);
        MedialSegment actual = Assert.Single(MedialAxisGeometry.Build(outer, [], provider, .01, 3e9));
        Assert.All(provider.Boundary, point =>
        {
            Assert.InRange(Math.Abs(point.x), 0, 5e7);
            Assert.InRange(Math.Abs(point.y), 0, 5e7);
        });
        Assert.Equal(1.5e8, provider.MaximumRadius);
        Assert.Equal(2.5e8, actual.Point0.Radius);
        Assert.Equal(1e12 + .5e9, actual.Point0.X);
        Assert.Equal(-1e12 + 1e9, actual.Point0.Y);
    }

    [Theory]
    [InlineData(1, 1e9, -1e9, 0)]
    [InlineData(1, -3e8, 4e8, 1)]
    [InlineData(4, 1e9, 1e9, 3)]
    public void BoostGeometryIsCovariantUnderTranslationQuarterTurnsAndScale(
        double scale, double offsetX, double offsetY, int quarterTurns)
    {
        PointD[] outer = Rectangle(-8, -5, 8, 5);
        PointD[] hole = Rectangle(-2, -1, 2, 1).Reverse().ToArray();
        var provider = BoostVoronoiProvider.CreateDefault();
        var reference = MedialAxisGeometry.Build(outer, [hole], provider, .01, -1, Math.PI + 1e-6);
        Assert.NotEmpty(reference);
        var actual = MedialAxisGeometry.Build(outer.Select(Transform).ToArray(),
            [hole.Select(Transform).ToArray()], provider, .01, -1, Math.PI + 1e-6);
        var expected = reference.Select(segment => new MedialSegment(
            TransformMedial(segment.Point0), TransformMedial(segment.Point1))).ToArray();
        Assert.Equal(expected.Length, actual.Count);
        Assert.All(expected, segment => Assert.Contains(actual, candidate => Matches(segment, candidate)));
        Assert.All(actual, segment => Assert.Contains(expected, candidate => Matches(segment, candidate)));

        PointD Transform(PointD point)
        {
            var (x, y) = quarterTurns switch
            {
                1 => (-point.y, point.x),
                2 => (-point.x, -point.y),
                3 => (point.y, -point.x),
                _ => (point.x, point.y),
            };
            return new PointD(x * scale + offsetX, y * scale + offsetY);
        }
        MedialPoint TransformMedial(MedialPoint point)
        {
            PointD xy = Transform(new PointD(point.X, point.Y));
            return new MedialPoint(xy.x, xy.y, point.Radius * scale);
        }
    }

    [Fact]
    public void CancellationBeforeDispatchDoesNotCallProvider()
    {
        var provider = new CapturingProvider();
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => MedialAxisGeometry.Build(
            Rectangle(0, 0, 8, 4), [], provider, .01, 1, cancellationToken: cancellation.Token));
        Assert.Empty(provider.Boundary);
    }

    [Fact]
    public void UnlimitedRadiusUsesLocalDiagonalAndPreservesProviderTolerance()
    {
        var provider = new CapturingProvider();
        _ = MedialAxisGeometry.Build(Rectangle(100, 200, 103, 204), [], provider, .03, -1);
        Assert.Equal(5 * PathUtils.Scale, provider.MaximumRadius);
        Assert.Equal(.03, provider.Tolerance);
    }

    private static bool Matches(MedialSegment first, MedialSegment second)
        => Close(first.Point0, second.Point0) && Close(first.Point1, second.Point1)
            || Close(first.Point0, second.Point1) && Close(first.Point1, second.Point0);

    private static bool Close(MedialPoint first, MedialPoint second)
        => Math.Abs(first.X - second.X) <= 2e-5 && Math.Abs(first.Y - second.Y) <= 2e-5
            && Math.Abs(first.Radius - second.Radius) <= 2e-5;

    private static PointD[] Rectangle(double left, double bottom, double right, double top)
        => [new(left, bottom), new(right, bottom), new(right, top), new(left, top)];

    private sealed class CapturingProvider : IMedialAxisProvider
    {
        internal IReadOnlyList<PointD> Boundary { get; private set; } = [];
        internal IReadOnlyList<IReadOnlyList<PointD>> Holes { get; private set; } = [];
        internal double Tolerance { get; private set; }
        internal double MaximumRadius { get; private set; }
        internal double FilteringAngle { get; private set; }
        internal bool UseBigIntegers { get; private set; }

        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle = 3 * Math.PI / 4, bool useBigIntegers = true)
        {
            Boundary = boundary; Holes = holes; Tolerance = tolerance;
            MaximumRadius = maxRadius; FilteringAngle = filteringAngle; UseBigIntegers = useBigIntegers;
            double width = boundary.Max(point => point.x) - boundary.Min(point => point.x);
            return [new(new(0, 0, width / 4), new(width / 4, 0, width / 4))];
        }
    }
}
