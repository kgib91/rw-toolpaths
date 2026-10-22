using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public class CertifiedInsetReuseTests
{
    [Theory]
    [InlineData(false, 0)]
    [InlineData(false, 1.25)]
    [InlineData(true, 0)]
    public void CombinedGenerationMatchesIndependentCertifiedCalls(bool interior, double edgeWidth)
    {
        var source = Source();
        var provider = BoostVoronoiProvider.CreateDefault();
        var expectedClearing = MedialAxisToolpaths.GenerateClearingPassesTagged(source,
            0, 2, Math.PI / 3, 1, stepOver: .55, regionIndex: 7,
            includeInteriorFill: interior, detectCurves: false, edgeFillWidth: edgeWidth,
            outputSurfaceZ: 12.7004);
        var expectedFinal = MedialAxisToolpaths.Generate(provider, source,
            0, 2, Math.PI / 3, 1, includeInteriorFill: interior, outputSurfaceZ: 12.7004);
        Assert.NotEmpty(expectedClearing);
        Assert.NotNull(expectedFinal);
        Assert.NotEmpty(expectedFinal!);

        var logs = new List<string>();
        var actual = MedialAxisToolpaths.GenerateVCarveTagged(provider, source,
            0, 2, Math.PI / 3, 1, stepOver: .55, regionIndex: 7,
            includeInteriorFill: interior, diagnostic: logs.Add, detectCurves: false,
            edgeFillWidth: edgeWidth, outputSurfaceZ: 12.7004);

        Assert.Equal(expectedClearing.Count + expectedFinal!.Count, actual.Count);
        for (int i = 0; i < expectedClearing.Count; i++)
        {
            Assert.Equal(expectedClearing[i].Points, actual[i].Points);
            Assert.Equal(expectedClearing[i].Moves, actual[i].Moves);
            Assert.Equal(expectedClearing[i].DepthPassIndex, actual[i].DepthPassIndex);
            Assert.Equal(expectedClearing[i].Category, actual[i].Category);
            Assert.Equal(7, actual[i].RegionIndex);
        }
        for (int i = 0; i < expectedFinal.Count; i++)
            Assert.Equal(expectedFinal[i], actual[expectedClearing.Count + i].Points);
        var completion=Assert.Single(logs,line=>line.Contains("stage=certified-inset-reuse"));
        // Perimeter-only final carving does not request a bottom fill inset.
        Assert.Equal($"prepared=2 reused={(interior ? 1 : 0)}",System.Text.RegularExpressions.Regex.Match(completion,@"prepared=\d+ reused=\d+").Value);
        Assert.Contains("capacity=1",completion);
    }

    [Fact]
    public void PreparedGraphWithoutInteriorFillSkipsUnusedBottomInset()
    {
        var source = Source();
        var provider = BoostVoronoiProvider.CreateDefault();
        var graph = MedialAxisToolpaths.GenerateRawMedialAxisSegments(provider, source,
            Math.PI / 3, 1, maxRadiusOverride: 2 * Math.Tan(Math.PI / 6));
        Assert.NotEmpty(graph);
        var expected = MedialAxisToolpaths.Generate(provider, source, 0, 2, Math.PI / 3, 1,
            includeInteriorFill: false, preparedMedialGraph: graph);
        Assert.NotNull(expected);
        Assert.NotEmpty(expected!);
        var logs = new List<string>();
        var actual = MedialAxisToolpaths.GenerateVCarveTagged(provider, source, 0, 2, Math.PI / 3, 1,
            includeInteriorFill: false, detectCurves: false, diagnostic: logs.Add,
            preparedMedialGraph: graph).Where(path => path.Category == "final-carve").ToList();
        Assert.Equal(expected!.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++) Assert.Equal(expected[i], actual[i].Points);
        var completion=Assert.Single(logs,line=>line.Contains("stage=certified-inset-reuse"));
        Assert.Equal("prepared=2 reused=0",System.Text.RegularExpressions.Regex.Match(completion,@"prepared=\d+ reused=\d+").Value);
    }

    [Fact]
    public void ResultAndSourceMutationCannotAlterRetainedCertificate()
    {
        var original = Source();
        var expected = PolygonOps.CutterInset(original, .7);
        Assert.NotEmpty(expected);
        var mutableSource = original.Select(ring => ring.ToList()).ToList();
        using var reuse = new MedialAxisToolpaths.CertifiedInsetReuse();
        var first = reuse.Contract(mutableSource, .7);
        first[0].Clear();
        first.Clear();
        mutableSource[0][0] = new PointD(-200, -200);

        var second = reuse.Contract(original, .7);
        EqualRings(expected, second);
        second[0].Clear();
        EqualRings(expected, reuse.Contract(original, .7));
        Assert.Equal(1, reuse.PreparedCount);
        Assert.Equal(2, reuse.ReuseCount);
    }

    [Theory]
    [InlineData("radius")]
    [InlineData("sub-grid-coordinate")]
    [InlineData("hole")]
    [InlineData("ring-order")]
    [InlineData("winding")]
    public void ChangedInputCannotReuseAnotherCertificate(string change)
    {
        var original = Source();
        using var reuse = new MedialAxisToolpaths.CertifiedInsetReuse();
        _ = reuse.Contract(original, .7);
        var changed = original.Select(ring => ring.ToList()).ToList();
        double radius = .7;
        switch (change)
        {
            case "radius": radius = Math.BitIncrement(radius); break;
            case "sub-grid-coordinate": changed[0][0] = new PointD(.000001, 0); break;
            case "hole": changed.RemoveAt(1); break;
            case "ring-order": changed.Reverse(); break;
            case "winding": changed[1].Reverse(); break;
        }
        EqualRings(PolygonOps.CutterInset(changed, radius), reuse.Contract(changed, radius));
        Assert.Equal(2, reuse.PreparedCount);
        Assert.Equal(0, reuse.ReuseCount);
        // Single-entry retention also prevents reuse of an earlier certificate
        // after a different input has replaced it.
        EqualRings(PolygonOps.CutterInset(original, .7), reuse.Contract(original, .7));
        Assert.Equal(3, reuse.PreparedCount);
    }

    [Fact]
    public void CancellationIsObservedForAnOtherwiseExactReuse()
    {
        using var reuse = new MedialAxisToolpaths.CertifiedInsetReuse();
        var source = Source();
        _ = reuse.Contract(source, .7);
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => reuse.Contract(source, .7, cancellation.Token));
        Assert.Equal(0, reuse.ReuseCount);
    }

    private static List<IReadOnlyList<PointD>> Source() => new()
    {
        new List<PointD> { new(0, 0), new(20, 0), new(20, 14), new(0, 14) },
        new List<PointD> { new(8, 4), new(8, 10), new(12, 10), new(12, 4) }
    };

    private static void EqualRings(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
            Assert.Equal(expected[i].Select(point => (point.x, point.y)),
                actual[i].Select(point => (point.x, point.y)));
    }
}
