using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[Collection("Adaptive cleanup proposal diagnostics")]
public sealed class TrialFilledSweepProposalTests
{
    private const string Reference = "RW_TOOLPATHS_TRIAL_FILLED_SWEEP_REFERENCE";

    [Fact]
    public void PocketCapturesModeBeforeGeometryAndGenericRingKeepsOriginalDefault()
    {
        using var environment = new EnvironmentScope();
        Assert.True(new PocketOptions().TrialFilledSweepProposals);
        Assert.False(new RingOffsetOptions(3, 6).TrialFilledSweepProposals);
        var pocket = PocketToolpaths.CaptureTrialFilledSweepMode(new PocketOptions { BoundOriginalSourceMedial = false });
        var ring = RingOffsetEngine.CaptureTrialFilledSweepMode(Options());
        Assert.True(pocket.TrialFilledSweepProposals);
        Assert.True(ring.TrialFilledSweepProposals);
        Environment.SetEnvironmentVariable(Reference, "1");
        Assert.True(pocket.TrialFilledSweepProposals);
        Assert.True(RingOffsetEngine.CaptureTrialFilledSweepMode(ring).TrialFilledSweepProposals);
        Assert.False(PocketToolpaths.CaptureTrialFilledSweepMode(new PocketOptions()).TrialFilledSweepProposals);
        Assert.False(RingOffsetEngine.CaptureTrialFilledSweepMode(Options()).TrialFilledSweepProposals);
    }

    [Theory]
    [InlineData("RW_TOOLPATHS_TRIAL_FILLED_SWEEP_REFERENCE")]
    [InlineData("RW_TOOLPATHS_COHERENCE_REFERENCE")]
    public void EachReferenceRestoresOriginalTrialsWithoutChangingOtherPocketPolicies(string variable)
    {
        using var environment = new EnvironmentScope();
        Environment.SetEnvironmentVariable(variable, "1");
        var captured = PocketToolpaths.CaptureTrialFilledSweepMode(new PocketOptions());
        Assert.False(captured.TrialFilledSweepProposals);
        Assert.True(captured.AdaptiveCompactionProposals);
        Assert.True(captured.BoundOriginalSourceMedial);
        var (_, log) = Capture(() => Search(Options(), (_, raw) => Clone(raw)));
        Assert.Contains("trialFilledMode=False", log);
        Assert.Contains("trialFilledUsed=0", log);
    }

    [Fact]
    public void ActualPocketRetainsCapturedTrialModeAfterProgressChangesReference()
    {
        using var environment = new EnvironmentScope();
        var (value, log) = Capture(() => PocketToolpaths.Generate(new[] { Source() }, ToolGeometry.Flat(3),
            new DepthSchedule(1, 1), new PocketOptions { StepOver = .55, MaxRings = 2,
                DetectCurves = false, CalculateSweptArea = false },
            progress: (_, _) => Environment.SetEnvironmentVariable(Reference, "1")));
        Assert.NotEmpty(((ToolpathPlan)value).Toolpaths);
        Assert.Contains("trialFilledSweepProposals=True trialFilledModeCaptured=True", log);
        Assert.Contains("trialFilledMode=True", log);
        Assert.Contains("trialFilledFinalOriginal=1", log);
        Assert.Contains("finalCommandFrontProof=True fullSignedShellProof=True", log);
    }

    [Fact]
    public void CompleteSignedProposalPreservesAllHolesOfMultipleOuterComponents()
    {
        var hole1 = Rectangle(8, 8, 18, 18); hole1.Reverse();
        var hole2 = Rectangle(23, 23, 33, 33); hole2.Reverse();
        var hole3 = Rectangle(58, 8, 68, 18); hole3.Reverse();
        var paths = new List<List<PointD>> { Rectangle(0, 0, 40, 40), hole1, hole2,
            Rectangle(50, 0, 90, 40), hole3 };
        Assert.True(Try(paths, true, null, default, out var filled, out _));
        foreach (var point in new[] { new PointD(13, 13), new PointD(28, 28), new PointD(63, 13) })
            Assert.Empty(PolygonOps.Intersect(Disc(point), filled));
        var components = PolygonOps.DifferenceToComponents(filled, Array.Empty<IReadOnlyList<PointD>>());
        Assert.Equal(2, components.Count);
        Assert.Equal(3, components.Sum(component => component.Holes.Count));
    }

    [Fact]
    public void NoncanonicalSignedInputCannotNominateFilledCoverage()
    {
        Assert.False(Try(new() { Rectangle(0, 0, 40, 40), Rectangle(10, 10, 20, 20) },
            true, null, default, out _, out var reason));
        Assert.Equal("noncanonical-signed-domain", reason);
        var inverted = Rectangle(0, 0, 40, 40); inverted.Reverse();
        Assert.False(Try(new() { inverted }, true, null, default, out _, out _));
        var crossing = new List<PointD> { new(0, 0), new(20, 20), new(0, 20), new(20, 0) };
        Assert.False(Try(new() { crossing }, true, null, default, out _, out _));
    }

    [Fact]
    public void PartialCommandClipFallsBackToActualFiniteClosedFragments()
    {
        var closed = new List<List<PointD>> { Rectangle(-10, -2, 10, 2) };
        var clip = Rectangle(-3, -3, 3, 3);
        Assert.False(Try(closed, true, clip, default, out _, out var reason));
        Assert.Equal("partial-command-clip", reason);
        var actual = RingOffsetEngine.BufferClosedSweeps(closed, 3, .001, default, true, clip);
        Assert.Empty(PolygonOps.Intersect(Disc(new(-10, 0)), actual));
    }

    [Fact]
    public void CompleteClosedClipQualifiesAndOpenTerminalsNeverGainAnImplicitClosingChord()
    {
        Assert.True(Try(Raw(9), true, Rectangle(0, 0, 60, 90), default, out _, out _));
        var open = new List<List<PointD>> { new() { new(0, 0), new(20, 0), new(20, 20) } };
        Assert.False(Try(open, false, null, default, out _, out _));
        var actual = PolygonOps.BufferOpenCenterlines(open, 3, .001);
        Assert.Empty(PolygonOps.Intersect(Disc(new(10, 10)), actual));
    }

    [Fact]
    public void InvalidCoordinatesCannotPublishAProposal()
    {
        var nonfinite = Raw(9); nonfinite[0][0] = new(double.NaN, 0);
        Assert.False(Try(nonfinite, true, null, default, out var absent, out _));
        Assert.Empty(absent);
        var excessive = Raw(9); excessive[0][0] = new(1e12, 0);
        Assert.False(Try(excessive, true, null, default, out _, out _));
        Assert.False(Try(Raw(9), true, new() { new(double.PositiveInfinity, 0), new(1, 0), new(1, 1) },
            default, out _, out _));
        Assert.False(RingOffsetEngine.TryBuildTrialFilledSweep(Raw(9), double.PositiveInfinity, .001,
            true, null, default, out _, out _));
    }

    [Fact]
    public void PositiveSelectedFrontMustPassOriginalWholeGapAndOwnOnlyOriginalClippedSweep()
    {
        using var environment = new EnvironmentScope();
        var clip = Rectangle(0, 0, 60, 90);
        var (accepted, log) = Capture(() => Search(Options() with { PhysicalClipBoundary = clip }, (_, raw) => Clone(raw)));
        Assert.Contains("trialFilledMode=True", log);
        Assert.Contains("trialFilledUsed=1", log);
        Assert.Contains("trialFilledFinalOriginal=1", log);
        Assert.Contains("finalCommandFrontProof=True fullSignedShellProof=True", log);
        Assert.Equal(0d, (double)accepted.GetType().GetProperty("ShellGapArea")!.GetValue(accepted)!);
        var commands = Polygons(accepted);
        Assert.True(Owner(accepted).TryReuse(commands, 3, .001, true, clip, default, out var sweep));
        var rebuilt = RingOffsetEngine.BufferClosedSweeps(commands, 3, .001, default, true, clip);
        Assert.True(PreparedClosedSweep.SameIntegerSweep(sweep!, rebuilt, default));
    }

    [Fact]
    public void MissingSelectedChildRunsOriginalSearchWithBothProposalModesOffAndFullMutualCleanup()
    {
        using var environment = new EnvironmentScope();
        var prepared = PolygonOps.PrepareBoundaryOffset(Source().Outer, null, .001, 0, sourceConstructionSweepRadius: 3);
        List<List<PointD>> Full(double distance) => prepared.CompactCandidate(BurrRaw(distance), -distance, default);
        var bad = new List<List<PointD>> { Rectangle(29, 44, 31, 46) };
        var (recovered, log) = Capture(() => Search(Options(), (_, _) => Clone(bad), Full));
        Assert.Contains("reason=final-command-front-unproved", log);
        Assert.Contains("fullMutualProofRequired=True rejectedProposalCredited=False", log);
        Assert.Contains("proposalOnly=False", log);
        Assert.Contains("trialFilledMode=False", log);
        Assert.Equal(0, prepared.CompactionProposalCount);
        var (expected, _) = Capture(() => Search(Options() with {
            TrialFilledSweepProposals = false, SelectedCleanupProposals = false }, null, Full));
        Assert.Equal(Commands(Polygons(expected)), Commands(Polygons(recovered)));
        Assert.Equal(expected.GetType().GetProperty("Distance")!.GetValue(expected),
            recovered.GetType().GetProperty("Distance")!.GetValue(recovered));
        Assert.False(Owner(recovered).TryReuse(bad, 3, .001, true, null, default, out _));
        Assert.True(Owner(recovered).TryReuse(Polygons(recovered), 3, .001, true, null, default, out _));
    }

    [Fact]
    public void NoFinalCompactorCannotEnableFilledTrialsEvenWhenExplicitlyRequested()
    {
        using var environment = new EnvironmentScope();
        var (_, log) = Capture(() => Search(Options(), null));
        Assert.Contains("trialFilledMode=False", log);
        Assert.Contains("trialFilledUsed=0", log);
    }

    [Fact]
    public void FirstFrontAndOrdinarySharedFillKeepOriginalSweepConstruction()
    {
        using var environment = new EnvironmentScope();
        var (_, firstLog) = Capture(() => RingOffsetEngine.BuildRings(new[] { Source() }, Options() with { RingLimit = 1 }));
        Assert.DoesNotContain("trialFilledMode=True", firstLog);
        var (_, ordinaryLog) = Capture(() => RingOffsetEngine.BuildRings(new[] { Source() },
            Options() with { StepOver = 2, RingLimit = 2 }));
        Assert.DoesNotContain("trialFilledMode=True", ordinaryLog);
    }

    [Fact]
    public void CancelledPrimitiveCannotReturnFilledCoverage()
    {
        using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => Try(Raw(9), true, null, cancelled.Token, out _, out _));
    }

    [Fact]
    public void CancellationDuringFinalAuditCannotPublishProposedStockHistory()
    {
        using var environment = new EnvironmentScope();
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

    private static bool Try(List<List<PointD>> paths, bool closed, List<PointD>? clip, CancellationToken token,
        out List<List<PointD>> sweep, out string reason)
        => RingOffsetEngine.TryBuildTrialFilledSweep(paths, 3, .001, closed, clip, token, out sweep, out reason);
    private static List<List<PointD>> Disc(PointD point) => PolygonOps.BufferOpenCenterlines(
        new[] { (IReadOnlyList<PointD>)new[] { point, new PointD(point.x + .01, point.y) } }, .05, .00025);
    private static RingOffsetOptions Options() => new(3, 6) { PhysicalSweepRadius = 3, MinimumRingArea = 0,
        RetainEmittedSweepHistory = true, SelectedCleanupProposals = true, TrialFilledSweepProposals = true };
    private static MillingRegion Source() => new(Rectangle(0, 0, 60, 90));
    private static List<List<PointD>> Raw(double distance) => new() { Rectangle(distance, distance, 60 - distance, 90 - distance) };
    private static List<List<PointD>> BurrRaw(double distance)
    {
        double high = 90 - distance;
        return new() { new() { new(distance, distance), new(60 - distance, distance), new(60 - distance, high),
            new(30, high), new(30 + .02, high + .00005), new(30 - .02, high + .0001), new(distance, high) } };
    }
    private static object Search(RingOffsetOptions options, Func<double, List<List<PointD>>, List<List<PointD>>>? compact,
        Func<double, List<List<PointD>>>? established = null, CancellationToken token = default)
    {
        Func<double, List<List<PointD>>> full = established ?? (Func<double, List<List<PointD>>>)Raw;
        var method = typeof(RingOffsetEngine).GetMethod("SelectCoherentFront", BindingFlags.Static | BindingFlags.NonPublic)!;
        return method.Invoke(null, new object?[] { Source(), 0, 0, Raw(3), 3d, 9d,
            compact is null ? full(9) : Raw(9), options, full, 0d, token, null, null,
            compact is null ? null : (Func<double, List<List<PointD>>>)Raw, compact, established })!;
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
    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1) => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
    private sealed class EnvironmentScope : IDisposable
    {
        private readonly string? _reference = Environment.GetEnvironmentVariable(Reference);
        private readonly string? _coherence = Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE");
        internal EnvironmentScope()
        {
            Environment.SetEnvironmentVariable(Reference, null);
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE", null);
        }
        public void Dispose()
        {
            Environment.SetEnvironmentVariable(Reference, _reference);
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE", _coherence);
        }
    }
}
