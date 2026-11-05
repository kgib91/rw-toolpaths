using System.IO.Compression;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class EmojiPocketRoutingTests
{
    [Theory]
    [InlineData(5000, false)]
    [InlineData(50, true)]
    public void ProvedCrossPocketFeedCompetesWithTheCompleteEntryWithoutLosingCuts(double travelRate, bool keepLongLink)
    {
        using var stream = typeof(EmojiPocketRoutingTests).Assembly.GetManifestResourceStream(
            "RW.Toolpaths.Tests.Fixtures.emoji-pocket-constructed.json.gz")!;
        using var unzip = new GZipStream(stream, CompressionMode.Decompress);
        using var document = JsonDocument.Parse(unzip);
        var root = document.RootElement;
        double radius = root.GetProperty("radius").GetDouble();
        var source = root.GetProperty("paths").EnumerateArray().Select(p => new TaggedToolpath(
            p.GetProperty("Points").EnumerateArray().Select(Point).ToArray(),
            p.GetProperty("Spans").EnumerateArray().Select(s => new ToolpathSpan(
                s[0].GetInt32(), s[1].GetInt32(), (ToolpathSpanKind)s[2].GetInt32())).ToArray(),
            p.GetProperty("RegionIndex").GetInt32(), p.GetProperty("Category").GetString(),
            p.GetProperty("DepthPassIndex").GetInt32(), detectCurves: false)).ToArray();
        var regions = root.GetProperty("regions").EnumerateArray().Select(r => new MillingRegion(
            r.GetProperty("Outer").EnumerateArray().Select(Xy).ToList(),
            r.GetProperty("Holes").EnumerateArray().Select(h => h.EnumerateArray().Select(Xy).ToList()).ToList())).ToArray();
        var depth = new DepthSchedule(2, 2) { SurfaceZ = 10 };
        var tool = new ToolGeometry(radius) {
            VCarveProfile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 3.175, 3.175, 70)], 70)
        };
        var options = new PocketOptions {
            Ramp = RampSettings.FromRatio(2), PreserveCommandVertices = true,
            LinkClearance = new PocketToolpaths.FillSourceClearance(regions, radius, null, null),
            DetectCurves = false, CalculateSweptArea = false
        };
        var reference = PocketContinuationRouter.Route(source, depth, options, radius, default, tool: tool).Paths;
        var motion = new MillingMotionSettings(1000, 500, travelRate, 15, 10);
        var selected = PocketContinuationRouter.Route(source, depth, options with { Motion = motion }, radius, default, tool: tool).Paths;
        var oldLinks = Edges(reference, ToolpathSpanKind.Link).ToArray();
        var newLinks = Edges(selected, ToolpathSpanKind.Link).ToArray();
        Assert.Contains(oldLinks, e => Length(e) > 26);
        Assert.Equal(keepLongLink, newLinks.Any(e => Length(e) > 26));
        // The screenshot's eye/side-wedge excursion and return boundaries must
        // be actual floor links, not disconnected visits with implicit lifts.
        Point3D[] highlightedDepartures = [new(46.99932861328125,39.03271484375,8),
            new(43.340789794921875,41.070465087890625,8),
            new(23.633331298828125,32.93670654296875,8),
            new(19.838043212890625,34.13397216796875,8)];
        foreach (var departure in highlightedDepartures)
        {
            Assert.Contains(newLinks, e => e.A == departure && e.B.Z == departure.Z);
            Assert.DoesNotContain(selected.Zip(selected.Skip(1)), pair =>
                pair.First.Points[^1] == departure && pair.Second.Points[0] != departure);
        }

        // Compare the real directed native cutting chords, independently of
        // seam choice, entry samples and optional Link retraces.
        var required = Edges(source, ToolpathSpanKind.Cut).ToHashSet();
        var retained = Edges(selected, ToolpathSpanKind.Cut).ToHashSet();
        Assert.True(required.IsSubsetOf(retained), $"Lost {required.Except(retained).Count()} original directed cuts.");
        // Entering a contour may recut the native XY chords under its original
        // ramp. Those extra floor chords must still come from that fixed source.
        var allowed = source.SelectMany(p => Enumerable.Range(1, p.Points.Count - 1)
            .Select(i => (p.Points[i - 1] with { Z = 8 }, p.Points[i] with { Z = 8 }))).ToHashSet();
        allowed.UnionWith(allowed.Select(e => (e.Item2, e.Item1)).ToArray());
        Assert.All(retained, e => Assert.Contains(e, allowed));
        var cost = new ToolpathMotionCost(motion, 10);
        double before = cost.SequenceSeconds(null, reference, null), after = cost.SequenceSeconds(null, selected, null);
        Assert.True(after <= before + 1e-8, $"Complete route increased from {before:R}s to {after:R}s.");
        if (!keepLongLink) Assert.True(before - after > .5, $"Expected measured saving, got {before - after:R}s.");
    }

    private static Point3D Point(JsonElement p) => new(p[0].GetDouble(), p[1].GetDouble(), p[2].GetDouble());
    private static PointD Xy(JsonElement p) => new(p[0].GetDouble(), p[1].GetDouble());
    private static double Length((Point3D A, Point3D B) e) => Math.Sqrt(
        Math.Pow(e.B.X - e.A.X, 2) + Math.Pow(e.B.Y - e.A.Y, 2) + Math.Pow(e.B.Z - e.A.Z, 2));
    private static IEnumerable<(Point3D A, Point3D B)> Edges(IEnumerable<TaggedToolpath> paths, ToolpathSpanKind kind)
        => paths.SelectMany(p => p.Spans.Where(s => s.Kind == kind).SelectMany(s =>
            Enumerable.Range(s.StartIndex + 1, s.MoveCount).Select(i => (p.Points[i - 1], p.Points[i]))));
}
