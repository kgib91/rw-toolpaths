using RW.Toolpaths.Milling;
using System.IO.Compression;
using System.Text.Json;

namespace RW.Toolpaths.Tests;

public sealed class MillingPlanningPolicyTests
{
    private static TaggedToolpath Cut(params Point3D[] points) => new(points, [], 0, "clearing", 0, detectCurves: false);
    private static readonly MillingMotionSettings Motion = new(1000, 500, 5000, 5);

    [Theory]
    [InlineData("pocket")]
    [InlineData("scanline-pocket")]
    [InlineData("profile")]
    [InlineData("carving-rough")]
    [InlineData("carving-finish")]
    [InlineData("vcarve")]
    public void ALongExecutedDetourLosesToTheCompleteIndependentEntry(string owner)
    {
        Point3D a = new(0, 0, -1), b = new(0, 250, -1), c = new(2, 250, -1), d = new(2, 0, -1);
        var first = Cut(a, b, c, d); var next = Cut(a, new(3, 0, -1)) with { DepthPassIndex = 1 };
        var log = new List<string>();
        var result = ToolpathContinuationPlanner.Reconnect([first, next], owner, 1, default,
            diagnostic: log.Add, motion: Motion);
        Assert.Equal(a, result[1].Points[0]);
        Assert.Equal(next.Points, result[1].Points);
        Assert.Contains(log, s => s.Contains("machineContext=True") && s.Contains("completeBridgeEntryAndCut=True"));
    }

    [Theory]
    [InlineData("pocket")]
    [InlineData("scanline-pocket")]
    [InlineData("profile")]
    [InlineData("carving-rough")]
    [InlineData("carving-finish")]
    [InlineData("vcarve")]
    public void AShortExecutedReturnBeatsLiftingWithTheSamePolicy(string owner)
    {
        Point3D a = new(0, 0, -1), b = new(1, 0, -1), c = new(0, 1, -1);
        var result = ToolpathContinuationPlanner.Reconnect([Cut(a, b), Cut(a, c) with { DepthPassIndex = 1 }], owner, 1, default,
            motion: Motion);
        Assert.Equal(new[] { b, a, c }, result[1].Points);
        Assert.Contains(result[1].Spans, s => s.Kind == ToolpathSpanKind.Link);
        Assert.Equal(b, result[0].Points[^1]);
    }

    [Fact]
    public void FeedProjectionAndOutputResolutionHaveOneLibraryOwner()
    {
        var feed = new MillingFeedPolicy(1000, 500);
        Assert.Equal(500, feed.Limit(0, -6.35, 0));
        Assert.Equal(1000, feed.Limit(3, -1, 0));
        Assert.Equal(Math.Asin(.5), feed.MaximumDescentAngle);
        Assert.Equal(925, feed.Command(925.5));
        Assert.Equal(925, feed.Command(926.5));
        Assert.Equal(924, feed.Command(924.5));
        Assert.Equal(1, feed.ResolutionHeldCommands);
        Assert.Equal(5000, feed.Command(1000, clearedTravel: true));
        Assert.Equal(500, feed.Command(500));
    }

    [Fact]
    public void ATransferUsesTheCommonStockTopReserveAndAWholePrefixProof()
    {
        var proofs = new List<(Point3D, Point3D)>();
        var moves = MillingTransitionPlanner.Build(new(0, 0, -3), new(1, 0, -2), 5, 0,
            (a, b) => { proofs.Add((a, b)); return b.Z >= -1; });
        Assert.Equal(new Point3D(0, 0, 5), moves[0].End);
        Assert.Equal(new Point3D(1, 0, 5), moves[1].End);
        var fast = Assert.Single(moves.Where(m => m.ClearedTravel));
        Assert.Equal((double)MillingTransitionPlanner.ApproachClearance, fast.Start.Z);
        Assert.True(fast.End.Z >= -1);
        Assert.Contains((fast.Start, fast.End), proofs);
        Assert.Equal(fast.End, moves[^1].Start);
        Assert.Equal(new Point3D(1, 0, -2), moves[^1].End);
    }

    [Fact]
    public void EntryPricingIncludesTheSameProvedFastApproachAsEmission()
    {
        int queries = 0;
        var settings = Motion with { ClearedTravelGuard = (_, b) => { queries++; return b.Z >= -.9; } };
        var ordinary = new ToolpathMotionCost(Motion, 0).TransferSeconds(new(2, 0, -1), new(0, 0, -1));
        var proved = new ToolpathMotionCost(settings, 0).TransferSeconds(new(2, 0, -1), new(0, 0, -1));
        Assert.InRange(queries, 1, 15);
        Assert.InRange(proved, .17, .19);
        Assert.True(proved < ordinary - .1);
    }

    [Fact]
    public void NoGeometricPermissionIsCreatedByFavourableMachineCosts()
    {
        var first = Cut(new(0, 0, -1), new(1, 0, -1));
        var next = Cut(new(1.01, 0, -1), new(2, 0, -1));
        var result = ToolpathContinuationPlanner.Reconnect([first, next], "all-milling", 1, default,
            motion: Motion);
        Assert.Equal(next.Points, result[1].Points);
        Assert.DoesNotContain(result[1].Spans, s => s.Kind == ToolpathSpanKind.Link);
    }

    [Fact]
    public void AnUnavailableSurfaceRampDoesNotRejectASeparatelyProvedContinuation()
    {
        var source = Cut(new(.1, 0, -1), new(.1, 0, -1));
        var cylinder = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, .5, .5, 5)], 5);
        Point3D from = new(0, 0, -1);
        var result = VCarvePathRouter.Route([source], [], new ToolGeometry(.5) { VCarveProfile = cylinder },
            new DepthSchedule(1, 1), new VCarveOptions { SmoothVCarving = true, Motion = Motion, StepOver = 1,
                DetectCurves = false }, 0, from, continuationCutGuard: (_, _) => true);
        Assert.Equal(from, result[0].Points[0]);
        Assert.Contains(result[0].Spans, s => s.Kind == ToolpathSpanKind.Link);
    }

    [Fact]
    public void FullCostIncludesTheRemainingCutAndItsControlledBelowStockDescent()
    {
        var cost = new ToolpathMotionCost(Motion, 0);
        var source = Cut(new(0, 0, -1), new(10, 0, -1));
        Assert.Equal(.6, cost.VisitSeconds(source), 9);
        double expected = 6 * 60d / 5000 + 2 * 60d / 5000
            + (5 - (double).1f) * 60 / 5000 + (1 + (double).1f) * 60 / 500;
        Assert.Equal(expected, cost.TransferSeconds(new(2, 0, -1), source.Points[0]), 6);
        Assert.Equal(expected + .6, cost.CompleteVisitSeconds(new(2, 0, -1), source), 6);
    }

    [Fact]
    public void PricingCannotInventALiftAtAnAlreadyContinuousEmittedBoundary()
    {
        Point3D a = new(100, 200, -1), b = new(100.00001, 200, -1);
        Assert.NotEqual(a, b);
        Assert.True(MillingTransitionPlanner.IsContinuous(a, b));
        Assert.Equal(0, new ToolpathMotionCost(Motion, 0).TransferSeconds(a, b));
        // Emission tolerance grants no stock identity or geometric shortcut.
        var planner = new ToolpathContinuationPlanner("continuity-cost", 1, default);
        Assert.Null(planner.Connect(a, [b], (_, _) => false));
    }

    [Fact]
    public void SparseSourceTagsHaveTheSameDefaultCutsAndTabMotionAsEmission()
    {
        var path = new TaggedToolpath(new Point3D[] {
            new(0, 0, -1), new(1, 0, -1), new(1, 0, 1), new(2, 0, 1), new(2, 0, -1) },
            new ToolpathSpan[] { new(1, 2, ToolpathSpanKind.TabLift) }, 0, "profile", 0, detectCurves: false);
        Assert.Equal(new[] { ToolpathSpanKind.Cut, ToolpathSpanKind.TabLift, ToolpathSpanKind.Cut, ToolpathSpanKind.Cut },
            ToolpathSpanKinds.Expand(path));
        Assert.Equal(.06 + .12 + .012 + .24, new ToolpathMotionCost(Motion, 0).VisitSeconds(path), 9);
    }

    [Fact]
    public void AnInnerDepthSliceCannotRedefineTheMachinesStockTop()
    {
        var settings = Motion with { MaterialSurfaceZ = 0 };
        var outer = new ToolpathMotionCost(settings, 0);
        var slice = new ToolpathMotionCost(settings, -3);
        Point3D from = new(0, 0, -4), to = new(1, 0, -4);
        Assert.Equal(outer.TransferSeconds(from, to), slice.TransferSeconds(from, to));
        Assert.Equal(outer.ChordSeconds(from, to), slice.ChordSeconds(from, to));
    }

    [Theory]
    [InlineData(3, 1)]
    [InlineData(1000, 2)]
    public void SourceDetoursAccountForTheirBranchReturnAndTheEntryTheyReplace(double branchLength, int visits)
    {
        var carrier = Cut(new(0, 0, -1), new(10, 0, -1), new(10, 10, -1), new(0, 10, -1), new(0, 0, -1));
        var branch = new TaggedToolpath(new Point3D[] { new(5, 0, 0), new(5, 0, -1), new(5, branchLength, -1) },
            new ToolpathSpan[] { new(0, 1, ToolpathSpanKind.Ramp), new(1, 2, ToolpathSpanKind.Cut) },
            0, "clearing", 0, detectCurves: false);
        var result = ToolpathSourceTraversal.EnteredDetours([carrier, branch], default, motion: Motion);
        Assert.Equal(visits, result.Count);
        Assert.Contains(result.SelectMany(p => p.Points), p => p == new Point3D(5, branchLength, -1));
    }

    [Fact]
    public void TheRecordedBackground1496LinkLosesWithoutChangingItsRequiredCutChords()
    {
        var assembly = typeof(MillingPlanningPolicyTests).Assembly;
        using var compressed = assembly.GetManifestResourceStream(assembly.GetManifestResourceNames()
            .Single(n => n.EndsWith("recorded-long-link.json.gz")))!;
        using var stream = new GZipStream(compressed, CompressionMode.Decompress);
        using var json = JsonDocument.Parse(stream);
        var root = json.RootElement;
        static Point3D Point(JsonElement p) => new(p[0].GetDouble(), p[1].GetDouble(), p[2].GetDouble());
        var source = root.GetProperty("source").EnumerateArray().Select(Point).ToArray();
        var points = root.GetProperty("linked").EnumerateArray().Select(Point).ToArray();
        var spans = root.GetProperty("spans").EnumerateArray().Select(s =>
            new ToolpathSpan(s[0].GetInt32(), s[1].GetInt32(), (ToolpathSpanKind)s[2].GetInt32())).ToArray();
        var old = new TaggedToolpath(points, spans, 0, "clearing", 0, detectCurves: false);
        var entry = RampPlanner.PlanPathEntry(source, 12.7, RampSettings.FromRatio(2), default, null,
            landAtSourceVertex: true, preserveClosedExit: false, preserveOpenExit: false);
        var independent = new TaggedToolpath(entry.Points, entry.Spans, 0, "clearing", 0, detectCurves: false);
        var cost = new ToolpathMotionCost(new(1000, 500, 5000, 17.7, 12.7), 12.7);
        Assert.False(ToolpathMotionCost.PreferContinuation(cost, Point(root.GetProperty("from")), independent, old,
            out double independentSeconds, out double linkedSeconds));
        Assert.InRange(linkedSeconds, 28.20, 28.30);
        Assert.InRange(independentSeconds, 0, 4);
        var cut = entry.Spans.Single(s => s.Kind == ToolpathSpanKind.Cut);
        for (int i = 1; i < source.Length; i++)
            Assert.Contains(Enumerable.Range(cut.StartIndex + 1, cut.MoveCount), j =>
                entry.Points[j - 1] == source[i - 1] && entry.Points[j] == source[i]
                || entry.Points[j - 1] == source[i] && entry.Points[j] == source[i - 1]);
    }
}
