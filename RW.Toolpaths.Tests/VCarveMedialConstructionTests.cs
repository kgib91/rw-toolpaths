using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveMedialConstructionTests
{
    private static JsonDocument Fixture() => JsonDocument.Parse(typeof(VCarveMedialConstructionTests).Assembly
        .GetManifestResourceStream("RW.Toolpaths.Tests.Fixtures.ordinary-medial-baseline.json")!);
    private static List<IReadOnlyList<PointD>> Rings(JsonElement rings) => rings.EnumerateArray().Select(r =>
        (IReadOnlyList<PointD>)r.EnumerateArray().Select(p => new PointD(p.GetProperty("x").GetDouble() * 25.4,
            p.GetProperty("y").GetDouble() * 25.4)).ToArray()).ToList();

    [Fact]
    public void OriginalTwelveCubicsProduceTheRecordedTessellationAndLayer()
    {
        using var fixture = Fixture();
        var cubics = fixture.RootElement.GetProperty("authored").EnumerateArray().Select(c => c.EnumerateArray()
            .Select(p => new PointD(p[0].GetDouble(), p[1].GetDouble())).ToArray()).ToArray();
        var ring = new List<PointD> { cubics[0][0] };
        foreach (var c in cubics) VCarveMedialConstruction.AppendCubic(ring, c[0], c[1], c[2], c[3]);
        var source = fixture.RootElement.GetProperty("cases")[0];
        var expected = Rings(source.GetProperty("sourceInches"))[0];
        Assert.Equal(55, ring.Count); Assert.Equal(expected.Count, ring.Count);
        for (int i = 0; i < ring.Count; i++) Assert.InRange(double.Hypot(ring[i].x - expected[i].x, ring[i].y - expected[i].y), 0, 2e-12);
        _ = VCarveMedialConstruction.Prepare([ring], observe: (stage, actual) =>
        {
            if (stage == "layer") EqualRings(Rings(source.GetProperty("layerInches")), actual);
        });
    }

    [Theory]
    [InlineData("void", 102)]
    [InlineData("rectangle", 10)]
    [InlineData("circle", 48)]
    [InlineData("annulus", 288)]
    public void CompleteConstructionMatchesRecordedCoordinatesRadiiAndDirectedOrder(string name, int count)
    {
        using var fixture = Fixture();
        var item = fixture.RootElement.GetProperty("cases").EnumerateArray().Single(c => c.GetProperty("name").GetString() == name);
        // Hole polarity is part of layer construction, independent of ellipse tessellation.
        var source = Rings(item.GetProperty("sourceInches"));
        if (name == "annulus") source[1] = source[1].Reverse().ToArray();
        var expectedBoundary = Rings(item.GetProperty("preparedInches"));
        var actualBoundary = VCarveMedialConstruction.Prepare(source);
        EqualRings(expectedBoundary, actualBoundary);
        double radius = item.GetProperty("call").GetProperty("maxRadius").GetDouble() / VCarveMedialConstruction.SiteScale;
        var actual = VCarveMedialConstruction.Construct(BoostVoronoiProvider.CreateDefault(), source, radius);
        Assert.Equal(count, actual.Count);
        var expected = item.GetProperty("medial");
        for (int i = 0; i < count; i++)
        {
            Point(actual[i].Point0, expected[i].GetProperty("point0"));
            Point(actual[i].Point1, expected[i].GetProperty("point1"));
        }
        static void Point(MedialPoint actual, JsonElement expected)
        {
            Assert.InRange(Math.Abs(actual.X - expected.GetProperty("x").GetDouble() / 4096 * 25.4), 0, 1e-8);
            Assert.InRange(Math.Abs(actual.Y - expected.GetProperty("y").GetDouble() / 4096 * 25.4), 0, 1e-8);
            Assert.InRange(Math.Abs(actual.Radius - expected.GetProperty("radius").GetDouble() / 4096 * 25.4), 0, 1e-8);
        }
    }

    [Fact]
    public void ConditioningCannotEraseASmallSourceComponent()
    {
        PointD[] source = [new(0, 0), new(.01, 0), new(.01, .01), new(0, .01)];
        var diagnostics = new List<string>();
        var prepared = VCarveMedialConstruction.Prepare([source], diagnostic: diagnostics.Add);
        Assert.Equal(source, Assert.Single(prepared));
        Assert.Contains(diagnostics, s => s.Contains("reason=topology-change"));
        Assert.NotEmpty(VCarveMedialConstruction.Construct(BoostVoronoiProvider.CreateDefault(), [source], 1));
    }

    [Fact]
    public void AllTwentySevenPlacedRegionsMatchCompleteHandlerSitesAndDirectedRadiusSamples()
    {
        using var compressed = typeof(VCarveMedialConstructionTests).Assembly.GetManifestResourceStream(
            "RW.Toolpaths.Tests.Fixtures.ordinary-void-medial.json.gz")!;
        using var stream = new System.IO.Compression.GZipStream(compressed, System.IO.Compression.CompressionMode.Decompress);
        using var fixture = JsonDocument.Parse(stream);
        var regions = fixture.RootElement.GetProperty("regions"); Assert.Equal(27, regions.GetArrayLength());
        foreach (var region in regions.EnumerateArray())
        {
            var source = region.GetProperty("layerMm").EnumerateArray().Select(r => (IReadOnlyList<PointD>)r.EnumerateArray()
                .Select(p => new PointD(p[0].GetDouble(), p[1].GetDouble())).ToArray()).ToList();
            var prepared = VCarveMedialConstruction.Prepare(source, sourceIsPreparedLayer: true);
            var actualSites = prepared.SelectMany(r => r).Select(p => ((long)Math.Floor(p.x / 25.4 * 4096 + .5),
                (long)Math.Floor(p.y / 25.4 * 4096 + .5))).ToHashSet();
            var call = region.GetProperty("input");
            var expectedSites = call.GetProperty("boundary").EnumerateArray().Concat(call.GetProperty("holes")
                .EnumerateArray().SelectMany(r => r.EnumerateArray())).Select(p => (p.GetProperty("x").GetInt64(), p.GetProperty("y").GetInt64())).ToHashSet();
            Assert.True(expectedSites.SetEquals(actualSites), $"Prepared sites differ in region {region.GetProperty("region")}");
            double radius = call.GetProperty("maxRadius").GetDouble() / VCarveMedialConstruction.SiteScale;
            var actual = VCarveMedialConstruction.Construct(BoostVoronoiProvider.CreateDefault(), source, radius, sourceIsPreparedLayer: true);
            var expected = region.GetProperty("medial"); Assert.Equal(expected.GetArrayLength(), actual.Count);
            for (int i = 0; i < actual.Count; i++)
            {
                Check(actual[i].Point0, expected[i].GetProperty("point0"));
                Check(actual[i].Point1, expected[i].GetProperty("point1"));
            }
        }
        static void Check(MedialPoint actual, JsonElement expected)
        {
            Assert.InRange(Math.Abs(actual.X - expected.GetProperty("x").GetDouble() / 4096 * 25.4), 0, 1e-8);
            Assert.InRange(Math.Abs(actual.Y - expected.GetProperty("y").GetDouble() / 4096 * 25.4), 0, 1e-8);
            Assert.InRange(Math.Abs(actual.Radius - expected.GetProperty("radius").GetDouble() / 4096 * 25.4), 0, 1e-8);
        }
    }

    [Fact]
    public void CancellationAndNonfiniteGeometryFailBeforeConstruction()
    {
        using var canceled = new CancellationTokenSource(); canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveMedialConstruction.Prepare([], canceled.Token));
        PointD[] invalid = [new(0, 0), new(double.NaN, 1), new(1, 0)];
        Assert.Throws<ArgumentException>(() => VCarveMedialConstruction.Prepare([invalid]));
    }

    private static void EqualRings(IReadOnlyList<IReadOnlyList<PointD>> expected, IReadOnlyList<IReadOnlyList<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
        {
            var ring = expected[i].ToList();
            if (ring.Count > 1 && ring[0] == ring[^1]) ring.RemoveAt(ring.Count - 1);
            Assert.Equal(ring.Count, actual[i].Count);
            for (int j = 0; j < ring.Count; j++)
            { Assert.Equal(ring[j].x, actual[i][j].x, 10); Assert.Equal(ring[j].y, actual[i][j].y, 10); }
        }
    }
}
