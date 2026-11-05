using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Adaptive cleanup proposal diagnostics", DisableParallelization = true)]
public sealed class AdaptiveCleanupProposalDiagnosticCollection;

[Collection("Adaptive cleanup proposal diagnostics")]
public sealed class AdaptiveCleanupProposalTests
{
    [Fact]
    public void PocketModeIsCapturedOnceAndOtherFillCallersKeepFullProofByDefault()
    {
        const string variable = "RW_TOOLPATHS_ADAPTIVE_COMPACTION_REFERENCE";
        string? previous = Environment.GetEnvironmentVariable(variable);
        try
        {
            Environment.SetEnvironmentVariable(variable, null);
            Assert.False(new RingOffsetOptions(3, 6).SelectedCleanupProposals);
            var captured = PocketToolpaths.CaptureAdaptiveCompactionMode(new PocketOptions { BoundOriginalSourceMedial = false });
            Assert.True(captured.AdaptiveCompactionProposals);
            Environment.SetEnvironmentVariable(variable, "1");
            Assert.True(captured.AdaptiveCompactionProposals);
            Assert.False(PocketToolpaths.CaptureAdaptiveCompactionMode(new PocketOptions()).AdaptiveCompactionProposals);
            Environment.SetEnvironmentVariable(variable, null);
            Assert.False(PocketToolpaths.CaptureAdaptiveCompactionMode(new PocketOptions {
                AdaptiveCompactionProposals = false }).AdaptiveCompactionProposals);
        }
        finally { Environment.SetEnvironmentVariable(variable, previous); }
    }

    [Fact]
    public void ActualPocketKeepsItsCapturedProofModeWhenProgressChangesReferenceEnvironment()
    {
        const string variable = "RW_TOOLPATHS_ADAPTIVE_COMPACTION_REFERENCE";
        string? previous = Environment.GetEnvironmentVariable(variable);
        try
        {
            Environment.SetEnvironmentVariable(variable, null);
            var (_, log) = Capture(() => PocketToolpaths.Generate(new[] { Source() }, ToolGeometry.Flat(3),
                new DepthSchedule(1, 1), new PocketOptions { StepOver = .55, MaxRings = 2,
                    CalculateSweptArea = false, DetectCurves = false },
                progress: (_, _) => Environment.SetEnvironmentVariable(variable, "1")));
            Assert.Contains("adaptiveCompactionProposals=True proofModeCaptured=True", log);
            Assert.Contains("selectedCleanupProposal=True", log);
            Assert.Contains("finalCommandFrontProof=True fullSignedShellProof=True", log);
        }
        finally { Environment.SetEnvironmentVariable(variable, previous); }
    }

    [Fact]
    public void ProposalRetainsOriginalHoleWallsTopologyAndExactEstablishedCommands()
    {
        var sourceHole = Circle(3); sourceHole.Reverse();
        var outer = BurrRectangle(-7, -7, 25, 25);
        var hole = Circle(6); hole.Reverse();
        var front = new List<List<PointD>> { outer, hole };
        var original = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(new[] { Rectangle(-10, -10, 30, 30), sourceHole }));
        (object fullValue, string fullLog) = Capture(() => OffsetBoundaryCleanup.Compact(front, original, -3, 3, .001));
        (object proposedValue, string proposedLog) = Capture(() => OffsetBoundaryCleanup.Compact(front, original, -3, 3, .001,
            proposeForShellAudit: true));
        var full = (List<List<PointD>>)fullValue;
        var proposed = (List<List<PointD>>)proposedValue;
        Assert.Equal(Commands(full), Commands(proposed));
        Assert.True(proposed.Sum(path => path.Count) < front.Sum(path => path.Count));
        var components = PolygonOps.DifferenceToComponents(proposed, Array.Empty<IReadOnlyList<PointD>>());
        Assert.Single(components);
        Assert.Single(components[0].Holes);
        Assert.Equal(hole.Select(ToPoint).OrderBy(p => p.X).ThenBy(p => p.Y),
            components[0].Holes[0].Select(ToPoint).OrderBy(p => p.X).ThenBy(p => p.Y));
        foreach (var path in proposed)
        for (int i = 0; i < path.Count; i++)
            Assert.Null(original.ClearanceViolation(ToPoint(path[i]), ToPoint(path[(i + 1) % path.Count]), 3 * PathUtils.Scale - 4));
        Assert.Contains("footprintEvaluated=True proposalOnly=False", fullLog);
        Assert.Contains("footprintEvaluated=False proposalOnly=True stockAuthorized=False", proposedLog);
    }

    [Fact]
    public void FirstAndOrdinaryPreparedOffsetsNeverCreateAnAdaptiveProposal()
    {
        var prepared = PolygonOps.PrepareBoundaryOffset(Source().Outer, null, .001, 0, sourceConstructionSweepRadius: 3);
        var first = prepared.Execute(-3, CancellationToken.None);
        Assert.NotEmpty(first);
        Assert.Equal(0, prepared.CompactionProposalCount);
        var ordinary = prepared.CompactCandidate(new List<List<PointD>> { BurrRectangle(9, 9, 51, 81) }, -9, CancellationToken.None);
        Assert.NotEmpty(ordinary);
        Assert.Equal(0, prepared.CompactionProposalCount);
        var (_, log) = Capture(() => RingOffsetEngine.BuildRings(new[] { Source() }, Options() with {
            SelectedCleanupProposals = true, RingLimit = 1 }));
        Assert.Contains("compactionProposals=0", log);
        Assert.DoesNotContain("proposalOnly=True", log);
    }

    [Fact]
    public void PositiveProposalRequiresFinalWholeShellProofAndCapturesOnlyActualClippedCommands()
    {
        int proposals = 0;
        var clip = Rectangle(0, 0, 60, 90);
        (object accepted, string log) = Capture(() => Search(Options() with { PhysicalClipBoundary = clip },
            (_, raw) => { proposals++; return Clone(raw); }));
        Assert.Equal(1, proposals);
        Assert.Contains("finalCommandFrontProof=True fullSignedShellProof=True", log);
        Assert.Contains("selectedCleanupProposal=True", log);
        Assert.Equal(0d, (double)accepted.GetType().GetProperty("ShellGapArea")!.GetValue(accepted)!);
        var commands = Polygons(accepted);
        var owner = Owner(accepted);
        Assert.True(owner.TryReuse(commands, 3, .001, true, clip, CancellationToken.None, out var actual));
        var rebuilt = RingOffsetEngine.BufferClosedSweeps(commands, 3, .001, CancellationToken.None, true, clip);
        Assert.True(PreparedClosedSweep.SameIntegerSweep(actual!, rebuilt, CancellationToken.None));
    }

    [Fact]
    public void MissingProposalChildFailsActualFinalGapAndFallbackKeepsFullMutualProofAndNoRejectedHistory()
    {
        var prepared = PolygonOps.PrepareBoundaryOffset(Source().Outer, null, .001, 0, sourceConstructionSweepRadius: 3);
        List<List<PointD>> Full(double distance) => prepared.CompactCandidate(Raw(distance), -distance, CancellationToken.None);
        var bad = new List<List<PointD>> { Rectangle(29, 44, 31, 46) };
        (object recovered, string log) = Capture(() => Search(Options(), (_, _) => Clone(bad), Full));
        Assert.Contains("reason=final-command-front-unproved", log);
        Assert.Contains("fullMutualProofRequired=True rejectedProposalCredited=False", log);
        Assert.Contains("proposalOnly=False", log);
        Assert.Equal(0, prepared.CompactionProposalCount);
        (object reference, _) = Capture(() => Search(Options() with { SelectedCleanupProposals = false }, null, Full));
        Assert.Equal(Commands(Polygons(reference)), Commands(Polygons(recovered)));
        Assert.Equal(reference.GetType().GetProperty("Distance")!.GetValue(reference),
            recovered.GetType().GetProperty("Distance")!.GetValue(recovered));
        Assert.False(Owner(recovered).TryReuse(bad, 3, .001, true, null, CancellationToken.None, out _));
        Assert.True(Owner(recovered).TryReuse(Polygons(recovered), 3, .001, true, null, CancellationToken.None, out _));
    }

    [Fact]
    public void CancellationDuringFinalAuditCannotPublishAProposedSweep()
    {
        using var cancelled = new CancellationTokenSource();
        var (_, log) = Capture(() => {
            var error = Assert.Throws<TargetInvocationException>(() => Search(Options(), (_, raw) => {
                cancelled.Cancel(); return Clone(raw);
            }, token: cancelled.Token));
            Assert.IsType<OperationCanceledException>(error.InnerException);
            return new object();
        });
        Assert.DoesNotContain("finalCommandFrontProof=True", log);
        Assert.DoesNotContain("owner=accepted-emitted-physical-sweeps", log);
    }

    [Fact]
    public void CancelledCleanupDoesNotCompleteOrCountAProposal()
    {
        var prepared = PolygonOps.PrepareBoundaryOffset(Source().Outer, null, .001, 0, sourceConstructionSweepRadius: 3);
        using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => prepared.CompactCandidate(Raw(9), -9, cancelled.Token, proposeForShellAudit: true));
        Assert.Equal(0, prepared.CompactionProposalCount);
        Assert.Equal(0, prepared.CompactionCount);
    }

    private static RingOffsetOptions Options() => new(3, 6) {
        PhysicalSweepRadius = 3, MinimumRingArea = 0, RetainEmittedSweepHistory = true,
        SelectedCleanupProposals = true };
    private static MillingRegion Source() => new(Rectangle(0, 0, 60, 90));
    private static List<List<PointD>> Raw(double distance) => new() { BurrRectangle(distance, distance, 60 - distance, 90 - distance) };
    private static object Search(RingOffsetOptions options,
        Func<double, List<List<PointD>>, List<List<PointD>>>? compact,
        Func<double, List<List<PointD>>>? established = null, CancellationToken token = default)
    {
        Func<double, List<List<PointD>>> full = established ?? (Func<double, List<List<PointD>>>)Raw;
        var method = typeof(RingOffsetEngine).GetMethod("SelectCoherentFront", BindingFlags.Static | BindingFlags.NonPublic)!;
        return method.Invoke(null, new object?[] { Source(), 0, 0, Raw(3), 3d, 9d, compact is null ? full(9) : Raw(9), options,
            full, 0d, token, null, null, compact is null ? null : (Func<double, List<List<PointD>>>)Raw, compact, established })!;
    }
    private static List<List<PointD>> Polygons(object value) => (List<List<PointD>>)value.GetType().GetProperty("Polygons")!.GetValue(value)!;
    private static PreparedClosedSweep Owner(object value) => (PreparedClosedSweep)value.GetType()
        .GetProperty("PreparedClosedSweep", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(value)!;
    private static (object Result, string Log) Capture(Func<object> action)
    {
        var previousOut = Console.Out; var previousError = Console.Error; using var log = new StringWriter();
        using var logSelection = RW.Diagnostics.DiagnosticLog.PushNamespaces("*");
        Console.SetOut(log); Console.SetError(log);
        try { return (action(), log.ToString()); }
        finally { Console.SetOut(previousOut); Console.SetError(previousError); }
    }
    private static List<List<PointD>> Clone(List<List<PointD>> paths) => paths.Select(path => path.ToList()).ToList();
    private static string[] Commands(List<List<PointD>> paths) => paths.Select(path => string.Join(";", path.Select(p => $"{p.x:R},{p.y:R}"))).ToArray();
    private static Point64 ToPoint(PointD p) => new((long)Math.Round(p.x * PathUtils.Scale), (long)Math.Round(p.y * PathUtils.Scale));
    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1) => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
    private static List<PointD> BurrRectangle(double x0, double y0, double x1, double y1)
    {
        double middle = (x0 + x1) / 2;
        return new() { new(x0, y0), new(x1, y0), new(x1, y1), new(middle, y1),
            new(middle + .02, y1 + .00005), new(middle - .02, y1 + .0001), new(x0, y1) };
    }
    private static List<PointD> Circle(double radius) => Enumerable.Range(0, 512).Select(i => new PointD(
        radius * Math.Cos(i * Math.Tau / 512), radius * Math.Sin(i * Math.Tau / 512))).ToList();
}
