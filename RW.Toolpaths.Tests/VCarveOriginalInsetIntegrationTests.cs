using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class VCarveOriginalInsetIntegrationTests
{
    private const double Surface = 12.7004;

    [Theory]
    [InlineData(false, false, 0d)]
    [InlineData(false, false, 1.25)]
    [InlineData(true, false, 1.25)]
    [InlineData(true, true, 0d)]
    public void CapturedConstructionModeKeepsEquivalentCommandsAndInvocationDiagnostics(
        bool rounded, bool interiorFill, double edgeWidth)
    {
        var source = Source();
        var profile = Profile(rounded);
        var provider = BoostVoronoiProvider.CreateDefault();
        var referenceLogs = new List<string>();
        var disabledLogs = new List<string>();
        var preferredLogs = new List<string>();
        var reference = MedialAxisToolpaths.GenerateVCarveTagged(provider, source,
            0, 2, Math.PI / 3, 1, stepOver: .55, regionIndex: 7,
            includeInteriorFill: interiorFill, detectCurves: false, diagnostic: referenceLogs.Add,
            cutterProfile: profile, edgeFillWidth: edgeWidth, outputSurfaceZ: Surface);
        var disabled = MedialAxisToolpaths.GenerateVCarveTagged(false, provider, source,
            0, 2, Math.PI / 3, 1, stepOver: .55, regionIndex: 7,
            includeInteriorFill: interiorFill, detectCurves: false, diagnostic: disabledLogs.Add,
            cutterProfile: profile, edgeFillWidth: edgeWidth, outputSurfaceZ: Surface);
        var preferred = MedialAxisToolpaths.GenerateVCarveTagged(true, provider, source,
            0, 2, Math.PI / 3, 1, stepOver: .55, regionIndex: 7,
            includeInteriorFill: interiorFill, detectCurves: false, diagnostic: preferredLogs.Add,
            cutterProfile: profile, edgeFillWidth: edgeWidth, outputSurfaceZ: Surface);

        Assert.NotEmpty(reference);
        Assert.Contains(reference, path => path.Category == "clearing");
        Assert.Contains(reference, path => path.Category == "final-carve");
        EqualPaths(reference, disabled);
        EqualPaths(reference, preferred);
        AssertMode(referenceLogs, false);
        AssertMode(disabledLogs, false);
        AssertMode(preferredLogs, true);
        ProveClearingCommandChords(source, preferred, profile);
    }

    [Fact]
    public void PreferredInvocationCacheRetainsDetachedExactSourceAndResultSnapshots()
    {
        var original = Source();
        var mutableSource = original.Select(path => path.ToList()).ToList();
        var expected = PolygonOps.CutterInset(original, .7, preferOriginalFirstConstruction: true,
            useOriginalHoleErosion: false);
        var logs = new List<string>();
        using (var reuse = new MedialAxisToolpaths.CertifiedInsetReuse(logs.Add,
            preferOriginalFirstConstruction: true))
        {
            var first = reuse.Contract(mutableSource, .7);
            Assert.NotEmpty(first);
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
        AssertMode(logs, true);
    }

    [Fact]
    public void SeparateModeOwnersCannotReuseEachOthersConstructionPolicy()
    {
        var source = Source();
        var legacyExpected = PolygonOps.CutterInset(source, .7);
        var preferredExpected = PolygonOps.CutterInset(source, .7, preferOriginalFirstConstruction: true,
            useOriginalHoleErosion: false);
        var legacyLogs = new List<string>();
        var preferredLogs = new List<string>();
        using (var legacy = new MedialAxisToolpaths.CertifiedInsetReuse(legacyLogs.Add))
        using (var preferred = new MedialAxisToolpaths.CertifiedInsetReuse(preferredLogs.Add,
            preferOriginalFirstConstruction: true))
        {
            EqualRings(legacyExpected, legacy.Contract(source, .7));
            EqualRings(preferredExpected, preferred.Contract(source, .7));
            EqualRings(preferredExpected, preferred.Contract(source, .7));
            EqualRings(legacyExpected, legacy.Contract(source, .7));
            Assert.Equal(1, legacy.PreparedCount);
            Assert.Equal(1, preferred.PreparedCount);
            Assert.Equal(1, legacy.ReuseCount);
            Assert.Equal(1, preferred.ReuseCount);
        }
        AssertMode(legacyLogs, false);
        AssertMode(preferredLogs, true);
    }

    [Theory]
    [InlineData("radius")]
    [InlineData("sub-grid-coordinate")]
    [InlineData("hole")]
    [InlineData("winding")]
    public void PreferredCacheRecomputesWhenItsExactOriginalInputsChange(string change)
    {
        var source = Source();
        var changed = source.Select(path => path.ToList()).ToList();
        double radius = .7;
        switch (change)
        {
            case "radius": radius = Math.BitIncrement(radius); break;
            case "sub-grid-coordinate": changed[0][0] = new PointD(.000001, 0); break;
            case "hole": changed.RemoveAt(1); break;
            case "winding": changed[1].Reverse(); break;
        }
        using var reuse = new MedialAxisToolpaths.CertifiedInsetReuse(preferOriginalFirstConstruction: true);
        _ = reuse.Contract(source, .7);
        EqualRings(PolygonOps.CutterInset(changed, radius, preferOriginalFirstConstruction: true,
                useOriginalHoleErosion: false),
            reuse.Contract(changed, radius));
        Assert.Equal(2, reuse.PreparedCount);
        Assert.Equal(0, reuse.ReuseCount);
    }

    [Fact]
    public void PreferredExactReuseStillObservesCancellation()
    {
        using var reuse = new MedialAxisToolpaths.CertifiedInsetReuse(preferOriginalFirstConstruction: true);
        var source = Source();
        _ = reuse.Contract(source, .7);
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => reuse.Contract(source, .7, cancellation.Token));
        Assert.Equal(1, reuse.PreparedCount);
        Assert.Equal(0, reuse.ReuseCount);
    }

    private static void AssertMode(List<string> diagnostics, bool preferred)
        => Assert.Single(diagnostics, line => line.Contains("stage=certified-inset-reuse event=complete")
            && line.Contains($"preferOriginalFirstConstruction={preferred}"));

    private static void ProveClearingCommandChords(IReadOnlyList<IReadOnlyList<PointD>> source,
        IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile)
    {
        var index = new PolygonOps.BoundarySegmentIndex(Clipper.Union(PathUtils.ToClipper(source), FillRule.NonZero));
        foreach (var path in paths.Where(path => path.Category == "clearing"))
        {
            Assert.NotEmpty(path.Points);
            double radius = MedialAxisToolpaths.CommandedCuttingRadius(profile, 0, 0, 0,
                -path.Points[0].Z, Surface);
            double required = (radius + Math.Sqrt(2) * .0005) * PathUtils.Scale + 2;
            for (int end = 1; end < path.Points.Count; end++)
            {
                Assert.Equal(path.Points[0].Z, path.Points[end].Z);
                static Point64 Command(Point3D point) => new((long)Math.Round((float)point.X * PathUtils.Scale),
                    (long)Math.Round((float)point.Y * PathUtils.Scale));
                var a = Command(path.Points[end - 1]); var b = Command(path.Points[end]);
                Assert.True(index.IsInside(a));
                Assert.True(index.IsInside(b));
                Assert.Null(index.ClearanceViolation(a, b, required));
            }
        }
    }

    private static void EqualPaths(IReadOnlyList<TaggedToolpath> expected, IReadOnlyList<TaggedToolpath> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int path = 0; path < expected.Count; path++)
        {
            Assert.Equal(expected[path].Points.ToArray(), actual[path].Points.ToArray());
            Assert.Equal(expected[path].Moves.ToArray(), actual[path].Moves.ToArray());
            Assert.Equal(expected[path].Spans.ToArray(), actual[path].Spans.ToArray());
            Assert.Equal(expected[path].RegionIndex, actual[path].RegionIndex);
            Assert.Equal(expected[path].Category, actual[path].Category);
            Assert.Equal(expected[path].DepthPassIndex, actual[path].DepthPassIndex);
            Assert.Equal(expected[path].VisitIdentity, actual[path].VisitIdentity);
        }
    }

    private static void EqualRings(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int path = 0; path < expected.Count; path++)
            Assert.Equal(expected[path].Select(point => (point.x, point.y)).ToArray(),
                actual[path].Select(point => (point.x, point.y)).ToArray());
    }

    private static List<IReadOnlyList<PointD>> Source()
        => new()
        {
            new PointD[] { new(0, 0), new(20, 0), new(20, 14), new(0, 14) },
            new PointD[] { new(8, 4), new(8, 10), new(12, 10), new(12, 4) },
        };

    private static VCarveCutterProfile Profile(bool rounded)
        => rounded ? new([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
            new(CarvingCutterSectionKind.Linear, 1, 3, 8)], 3)
            : new([new(CarvingCutterSectionKind.Linear, 0, 6 * Math.Tan(Math.PI / 6), 6)], 3);
}
