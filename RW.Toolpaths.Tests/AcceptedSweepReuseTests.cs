using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Accepted sweep diagnostic controls", DisableParallelization = true)]
public sealed class AcceptedSweepDiagnosticCollection;

[Collection("Accepted sweep diagnostic controls")]
public sealed class AcceptedSweepReuseTests
{
    [Fact]
    public void ExactOrderedClonedCommandsReuseOriginalClippedSweepIncludingHole()
    {
        var hole = Rectangle(8, 8, 12, 12); hole.Reverse();
        var paths = new List<List<PointD>> { Rectangle(1, 1, 19, 19), hole };
        var clip = Rectangle(0, 0, 20, 20);
        var original = Sweep(paths, clip);
        var owner = new PreparedClosedSweep(paths, original, 2, .001, true, clip, CancellationToken.None);
        Assert.True(owner.TryReuse(Clone(paths), 2, .001, true, clip.ToList(), CancellationToken.None, out var reused));
        Assert.Same(original, reused);
        Assert.True(PreparedClosedSweep.SameIntegerSweep(reused!, Sweep(Clone(paths), clip), CancellationToken.None));
        Assert.Empty(PolygonOps.Difference(reused!, original));
        Assert.Empty(PolygonOps.Difference(original, reused!));
    }

    [Fact]
    public void PartialEmissionOrChangedCommandOrderNeverCreditsWholeAuditedSweep()
    {
        var paths = new List<List<PointD>> { Rectangle(0, 0, 20, 20), Rectangle(40, 0, 60, 20) };
        var owner = new PreparedClosedSweep(paths, Sweep(paths, null), 2, .001, true, null, CancellationToken.None);
        Assert.False(owner.TryReuse(new List<List<PointD>> { paths[0] }, 2, .001, true, null, CancellationToken.None, out _));
        Assert.False(owner.TryReuse(paths.AsEnumerable().Reverse().ToList(), 2, .001, true, null, CancellationToken.None, out _));
        var reversed = Clone(paths); reversed[0].Reverse();
        Assert.False(owner.TryReuse(reversed, 2, .001, true, null, CancellationToken.None, out _));
        var rotated = Clone(paths); rotated[0].Add(rotated[0][0]); rotated[0].RemoveAt(0);
        Assert.False(owner.TryReuse(rotated, 2, .001, true, null, CancellationToken.None, out _));
    }

    [Fact]
    public void SnapshotRejectsLaterMutableCoordinatesClipAndConstructionIdentityChanges()
    {
        var paths = new List<List<PointD>> { Rectangle(0, 0, 20, 20) };
        var clip = Rectangle(-1, -1, 21, 21);
        var captured = Clone(paths); var capturedClip = clip.ToList();
        var owner = new PreparedClosedSweep(paths, Sweep(paths, clip), 2, .001, true, clip, CancellationToken.None);
        paths[0][0] = new(.00001, 0);
        clip[0] = new(-1.01, -1);
        Assert.True(owner.TryReuse(captured, 2, .001, true, capturedClip, CancellationToken.None, out _));
        Assert.False(owner.TryReuse(paths, 2, .001, true, capturedClip, CancellationToken.None, out _));
        Assert.False(owner.TryReuse(captured, 2, .001, true, clip, CancellationToken.None, out _));
        Assert.False(owner.TryReuse(captured, Math.BitIncrement(2d), .001, true, capturedClip, CancellationToken.None, out _));
        Assert.False(owner.TryReuse(captured, 2, Math.BitIncrement(.001), true, capturedClip, CancellationToken.None, out _));
        Assert.False(owner.TryReuse(captured, 2, .001, false, capturedClip, CancellationToken.None, out _));
        Assert.False(owner.TryReuse(captured, 2, .001, true, null, CancellationToken.None, out _));
    }

    [Theory]
    [InlineData(8191)]
    [InlineData(8192)]
    public void ClosingChordChunkBoundaryRemainsPartOfExactCommandIdentity(int count)
    {
        var path = Enumerable.Range(0, count).Select(i => new PointD(10 + 9 * Math.Cos(Math.Tau * i / count),
            10 + 9 * Math.Sin(Math.Tau * i / count))).ToList();
        var paths = new List<List<PointD>> { path };
        var original = Sweep(paths, null);
        var owner = new PreparedClosedSweep(paths, original, 2, .001, true, null, CancellationToken.None);
        Assert.True(owner.TryReuse(Clone(paths), 2, .001, true, null, CancellationToken.None, out var reused));
        Assert.True(PreparedClosedSweep.SameIntegerSweep(reused!, Sweep(Clone(paths), null), CancellationToken.None));
        var changed = Clone(paths); changed[0].Add(changed[0][0]);
        Assert.False(owner.TryReuse(changed, 2, .001, true, null, CancellationToken.None, out _));
    }

    [Fact]
    public void CancelledCaptureAndLookupPropagateWithoutPublishingAuthority()
    {
        var paths = new List<List<PointD>> { Rectangle(0, 0, 20, 20) };
        using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => new PreparedClosedSweep(paths, new(), 2, .001, true, null, cancelled.Token));
        var owner = new PreparedClosedSweep(paths, new(), 2, .001, true, null, CancellationToken.None);
        Assert.Throws<OperationCanceledException>(() => owner.TryReuse(paths, 2, .001, true, null, cancelled.Token, out _));
    }

    [Fact]
    public void EmptyTerminalDiscoveryRetainsIdenticalGapAndTheEstablishedAcceptedFront()
    {
        int callbacks = 0;
        var options = Options() with { TerminalPaths = (_, _) => { callbacks++; return Array.Empty<IReadOnlyList<PointD>>(); } };
        (object actual, string log) = Capture(() => Search(options));
        int actualCallbacks = callbacks; callbacks = 0;
        (object expected, _) = Capture(() => Search(options with { ReuseUnchangedShellGap = false }));
        Assert.True(actualCallbacks > 0);
        Assert.Equal(callbacks, actualCallbacks);
        AssertSameFront(expected, actual);
        Assert.True(Counter(actual, "UnchangedGapReuses") > 0);
        Assert.Equal(0, Counter(actual, "TerminalGapRechecks"));
        var stored = Owner(actual);
        var commands = Polygons(actual);
        Assert.True(stored.TryReuse(commands, 3, .001, true, null, CancellationToken.None, out var reused));
        Assert.True(PreparedClosedSweep.SameIntegerSweep(reused!, RingOffsetEngine.BufferClosedSweeps(commands,
            3, .001, CancellationToken.None, true), CancellationToken.None));
    }

    [Fact]
    public void ActualTerminalStrokesRequireNewGapAndAreExcludedFromClosedSweepOwner()
    {
        int callbacks = 0;
        var strokes = Enumerable.Range(0, 14).Select(i => (IReadOnlyList<PointD>)new PointD[] {
            new(3, 3 + 4 * i), new(37, 3 + 4 * i) }).ToArray();
        var options = Options() with { TerminalPaths = (_, _) => { callbacks++; return strokes; } };
        (object actual, string log) = Capture(() => Search(options));
        Assert.Equal(1, callbacks);
        Assert.Equal(1, Counter(actual, "TerminalGapRechecks"));
        Assert.Equal(0, Counter(actual, "UnchangedGapReuses"));
        Assert.Empty(Polygons(actual));
        Assert.Equal(strokes.Length, ((IReadOnlyList<IReadOnlyList<PointD>>)actual.GetType().GetProperty("TerminalPaths")!.GetValue(actual)!).Count);
        Assert.True(Owner(actual).TryReuse(Polygons(actual), 3, .001, true, null, CancellationToken.None, out var closed));
        Assert.Empty(closed!);
    }

    [Fact]
    public void RejectedFinalCleanupCannotPublishItsClosedSweepIntoHistory()
    {
        var options = Options();
        var rejected = new List<List<PointD>> { Rectangle(19, 29, 21, 31) };
        (object actual, string log) = Capture(() => Search(options, (_, _) => Clone(rejected)));
        (object expected, _) = Capture(() => Search(options));
        Assert.Contains("reason=final-command-front-unproved", log);
        AssertSameFront(expected, actual);
        var owner = Owner(actual);
        Assert.False(owner.TryReuse(rejected, 3, .001, true, null, CancellationToken.None, out _));
        Assert.True(owner.TryReuse(Polygons(actual), 3, .001, true, null, CancellationToken.None, out var sweep));
        Assert.True(PreparedClosedSweep.SameIntegerSweep(sweep!, RingOffsetEngine.BufferClosedSweeps(Polygons(actual),
            3, .001, CancellationToken.None, true), CancellationToken.None));
    }

    private static RingOffsetOptions Options() => new(3, 6) {
        PhysicalSweepRadius = 3, MinimumRingArea = 0, RetainEmittedSweepHistory = true };

    private static object Search(RingOffsetOptions options,
        Func<double, List<List<PointD>>, List<List<PointD>>>? compact = null)
    {
        var source = new MillingRegion(Rectangle(0, 0, 40, 60));
        List<List<PointD>> Offset(double inset) => inset >= 20 ? new()
            : new() { Rectangle(inset, inset, 40 - inset, 60 - inset) };
        var method = typeof(RingOffsetEngine).GetMethod("SelectCoherentFront", BindingFlags.Static | BindingFlags.NonPublic)!;
        return method.Invoke(null, new object?[] { source, 0, 0, Offset(3), 3d, 20d, Offset(20), options,
            (Func<double, List<List<PointD>>>)Offset, 0d, CancellationToken.None, null, null,
            compact is null ? null : (Func<double, List<List<PointD>>>)Offset, compact, null })!;
    }

    private static (object Result, string Log) Capture(Func<object> action)
    {
        var old = Console.Out; var oldError = Console.Error; using var log = new StringWriter();
        Console.SetOut(log); Console.SetError(log);
        try { return (action(), log.ToString()); }
        finally { Console.SetOut(old); Console.SetError(oldError); }
    }
    private static int Counter(object value, string field) => (int)value.GetType()
        .GetProperty(field, BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(value)!;
    private static List<List<PointD>> Polygons(object value)
        => (List<List<PointD>>)value.GetType().GetProperty("Polygons")!.GetValue(value)!;
    private static PreparedClosedSweep Owner(object value)
        => (PreparedClosedSweep)value.GetType().GetProperty("PreparedClosedSweep", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(value)!;
    private static void AssertSameFront(object expected, object actual)
    {
        Assert.Equal(expected.GetType().GetProperty("Distance")!.GetValue(expected), actual.GetType().GetProperty("Distance")!.GetValue(actual));
        Assert.Equal(Commands(Polygons(expected)), Commands(Polygons(actual)));
    }
    private static string[] Commands(IEnumerable<IReadOnlyList<PointD>> paths)
        => paths.Select(path => string.Join(";", path.Select(p => $"{p.x:R},{p.y:R}"))).ToArray();
    private static List<List<PointD>> Sweep(List<List<PointD>> paths, IReadOnlyList<PointD>? clip)
        => RingOffsetEngine.BufferClosedSweeps(paths, 2, .001, CancellationToken.None, true, clip);
    private static List<List<PointD>> Clone(List<List<PointD>> paths) => paths.Select(path => path.ToList()).ToList();
    private static List<PointD> Rectangle(double left, double bottom, double right, double top)
        => new() { new(left, bottom), new(right, bottom), new(right, top), new(left, top) };
}
