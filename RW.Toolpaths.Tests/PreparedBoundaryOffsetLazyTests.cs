using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using System.Reflection;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class PreparedBoundaryOffsetLazyTests(ITestOutputHelper output)
{
    [Fact]
    public void ExactZeroAndOriginalInfeasibilityDoNotConstructUnusedConditioning()
    {
        var outer = Circle(0, 0, 10, 256);
        var holes = new List<IReadOnlyList<PointD>> { Circle(3, 0, 1, 64) };
        var expected = LegacyZeroFront(outer, holes);
        var prepared = Prepare(outer, holes, .003);
        var first = prepared.WithTolerances(.00025, 0);
        Assert.True(first.ConstructionPreparationReused);
        outer.Clear(); holes.Clear(); // Deferred work must use the captured original source.
        EqualPaths(expected, prepared.Execute(0));
        EqualPaths(expected, first.Execute(0));
        Assert.Empty(first.Execute(-100));
        Assert.False(prepared.ConstructionPrepared); Assert.False(first.ConstructionPrepared);
        Assert.Equal(0, prepared.ConstructionBuildCount); Assert.Equal(0, first.ConstructionBuildCount);
        Assert.Equal(0, prepared.OrdinaryOffsetterBuildCount); Assert.Equal(0, first.OrdinaryOffsetterBuildCount);
        Assert.Equal(1, prepared.ConstructionSkippedExecutions); Assert.Equal(2, first.ConstructionSkippedExecutions);
        Report("zero-and-infeasible", first);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DeferredInitializationMatchesEagerTimingForEveryOrderedTrialWithHoles(bool physicalCleanup)
    {
        var outer = Circle(0, 0, 10, 256);
        var holes = new List<IReadOnlyList<PointD>> { Circle(3, 0, 1, 64) };
        var reference = Prepare(outer, holes, .003, physicalCleanup ? 1 : null);
        var actual = Prepare(outer, holes, .003, physicalCleanup ? 1 : null);
        ForceEagerPreparation(reference);
        Assert.True(reference.ConstructionPrepared); Assert.False(actual.ConstructionPrepared);
        outer.Clear(); holes.Clear();
        foreach (double delta in new[] { 0, -.7, -1.4, .25, -.7 })
            EqualPaths(reference.Execute(delta), actual.Execute(delta));
        Assert.Equal(1, actual.ConstructionBuildCount); Assert.True(actual.ConstructionPrepared);
        Assert.Equal(1, actual.OrdinaryOffsetterBuildCount);
        Report("eager-order-control", actual);
    }

    [Fact]
    public void OrdinaryInwardFrontRetainsTheLiteralLegacyRdpUnionXorAndOffset()
    {
        var outer = Circle(0, 0, 10, 256);
        const double tolerance = .003, delta = -.7, arc = .001;
        Paths64 original = OriginalSource(outer, null);
        var originalRegion = Clipper.Union(original, FillRule.NonZero);
        var reduced = new Paths64(original.Select(path =>
        {
            var points = PolygonSimplify.RdpSegmentBounded(PolygonOps.MaterializeIntegerPaths(new() { path })[0], tolerance);
            var candidate = points.Count >= 3 ? PathUtils.ToClipper(new[] { points })[0] : path;
            return Clipper.Area(candidate) * Clipper.Area(path) > 0 ? candidate : path;
        }));
        var reducedRegion = Clipper.Union(reduced, FillRule.NonZero);
        bool changed = reduced.Sum(path => path.Count) < original.Sum(path => path.Count)
            && Clipper.Xor(originalRegion, reducedRegion, FillRule.NonZero).Any(path => Clipper.Area(path) != 0);
        var offsetter = LegacyOffsetter(arc);
        offsetter.AddPaths(reduced, JoinType.Round, EndType.Polygon);
        var expected = new Paths64();
        offsetter.Execute((delta - (changed ? tolerance : 0)) * PathUtils.Scale, expected);
        var actual = Prepare(outer, null, tolerance).Execute(delta);
        EqualPaths(PolygonOps.NormalizeOrder(PolygonOps.MaterializeIntegerPaths(expected)), actual);
        var walls = new PolygonOps.BoundarySegmentIndex(originalRegion);
        foreach (var contour in PathUtils.ToClipper(actual))
        for (int i = 0; i < contour.Count; i++)
            Assert.Null(walls.ClearanceViolation(contour[i], contour[(i + 1) % contour.Count], -delta * PathUtils.Scale - 4));
    }

    [Fact]
    public void OutwardOrdinaryFrontBuildsOnlyItsRequiredOriginalSourceOffsetter()
    {
        var outer = Circle(0, 0, 10, 512);
        var hole = Circle(3, 0, 1, 64);
        var holes = new[] { (IReadOnlyList<PointD>)hole };
        var original = OriginalSource(outer, holes);
        var offsetter = LegacyOffsetter(.001);
        offsetter.AddPaths(original, JoinType.Round, EndType.Polygon);
        var expected = new Paths64(); offsetter.Execute(.25 * PathUtils.Scale, expected);
        var actual = Prepare(outer, holes, .003);
        EqualPaths(PolygonOps.NormalizeOrder(PolygonOps.MaterializeIntegerPaths(expected)), actual.Execute(.25));
        Assert.Equal(1, actual.ConstructionBuildCount);
        Assert.Equal(0, actual.OrdinaryOffsetterBuildCount); Assert.Equal(1, actual.OutwardOffsetterBuildCount);
        Report("outward-original-only", actual);
    }

    [Fact]
    public void DenseOutwardBandsNeedNeitherConditioningNorAnOrdinaryOffsetter()
    {
        var outer = DenseRectangle(0, 0, 60, 90);
        var original = OriginalSource(outer, null);
        int originalPoints = original.Sum(path => path.Count);
        int reducedPoints = original.Sum(path => PolygonSimplify.RdpSegmentBounded(
            PolygonOps.MaterializeIntegerPaths(new() { path })[0], .003).Count);
        Assert.True(originalPoints > 4096);
        Assert.True(reducedPoints <= 2048);
        // The legacy outward classifier uses ORIGINAL source count, even when
        // the independently conditioned count would choose ordinary offsets.
        // Build the expected boundary-band operand directly, without consulting
        // a PreparedBoundaryOffset's classifier or deferred construction.
        var expected = PolygonOps.NormalizeOrder(PolygonOps.MaterializeIntegerPaths(
            PolygonOps.BuildOffsetCandidateForCertificate(
                Clipper.Union(original, FillRule.NonZero), .25, .001)));
        var reference = Prepare(outer, null, .003);
        var actual = Prepare(outer, null, .003);
        ForceEagerPreparation(reference);
        var result = actual.Execute(.25, compactFront: false);
        EqualPaths(expected, result);
        EqualPaths(reference.Execute(.25, compactFront: false), result);
        Assert.False(actual.ConstructionPrepared); Assert.Equal(0, actual.ConstructionBuildCount);
        Assert.Equal(0, actual.OrdinaryOffsetterBuildCount); Assert.Equal(0, actual.OutwardOffsetterBuildCount);
        Assert.Equal(1, actual.DenseSourceConstructionCount);
        output.WriteLine($"[Offset.Lazy.Test] classifier=original-outward-count originalPoints={originalPoints} "
            + $"independentReducedPoints={reducedPoints} literalBoundaryBands=True");
        Report("dense-outward-no-unused-work", actual);
    }

    [Fact]
    public void DenseInwardAndBacktrackingCachesRetainEagerReferenceOutput()
    {
        var outer = DenseRectangle(0, 0, 60, 90);
        var reference = Prepare(outer, null, 0);
        var actual = Prepare(outer, null, 0);
        ForceEagerPreparation(reference);
        foreach (double delta in new[] { -1d, -3d, -2d })
            EqualPaths(reference.Execute(delta, compactFront: false), actual.Execute(delta, compactFront: false));
        Assert.Equal(0, actual.OrdinaryOffsetterBuildCount);
        Assert.Equal(reference.DenseSourceConstructionCount, actual.DenseSourceConstructionCount);
        Assert.Equal(reference.DenseInsetReuseCount, actual.DenseInsetReuseCount);
        Assert.Equal(2, actual.DenseInsetReuseCount);
        Report("dense-fixed-cache", actual);
    }

    [Fact]
    public void AcceptedFirstInsetStillPublishesTheSameConstructionCache()
    {
        var outer = DenseRectangle(0, 0, 60, 90);
        var reference = Prepare(outer, null, 0, 3);
        var actual = Prepare(outer, null, 0, 3);
        ForceEagerPreparation(reference);
        var accepted = new List<List<PointD>> { Rectangle(3, 3, 57, 87) };
        Assert.True(reference.TrySeedAcceptedFirstInset(accepted, -3, default));
        Assert.True(actual.TrySeedAcceptedFirstInset(accepted, -3, default));
        foreach (double delta in new[] { -9d, -6d, -8d })
            EqualPaths(reference.Execute(delta, compactFront: false), actual.Execute(delta, compactFront: false));
        Assert.True(actual.AcceptedInsetSeeded); Assert.Equal(3, actual.DenseInsetReuseCount);
        Assert.Equal(0, actual.DenseSourceConstructionCount); Assert.Equal(0, actual.OrdinaryOffsetterBuildCount);
        Report("accepted-first-cache", actual);
    }

    [Fact]
    public void OriginalFirstSelectionKeepsItsExactGuardAndPublishedCommands()
    {
        var outer = Circle(0, 0, 10, 512);
        var reference = Prepare(outer, null, .003, 1);
        var actual = Prepare(outer, null, .003, 1);
        ForceEagerPreparation(reference);
        var expected = reference.ExecuteOriginalFirstAuthority(-1);
        EqualPaths(expected, actual.ExecuteOriginalFirstAuthority(-1));
        EqualPaths(expected, actual.ExecuteOriginalFirstAuthority(-1));
        Assert.Equal(reference.OriginalFirstAuthorityCount, actual.OriginalFirstAuthorityCount);
        Assert.Equal(1, actual.OriginalFirstAuthorityCount);
        Assert.Equal(1, actual.ConstructionBuildCount); Assert.Equal(0, actual.OrdinaryOffsetterBuildCount);
        Report("original-first-guard", actual);
    }

    [Fact]
    public void ToleranceContextsShareOneConditionedSnapshotAndSeparateOffsetters()
    {
        var outer = Circle(0, 0, 10, 256);
        var prepared = Prepare(outer, null, .003);
        var first = prepared.WithTolerances(.00025, 0);
        Assert.True(first.SourcePreparationReused); Assert.True(first.ConstructionPreparationReused);
        Assert.False(first.ConstructionPrepared); Assert.False(prepared.ConstructionPrepared);
        first.Execute(-.7);
        Assert.True(prepared.ConstructionPrepared); Assert.Equal(1, prepared.ConstructionBuildCount);
        prepared.Execute(-1.4);
        Assert.Equal(1, first.ConstructionBuildCount); Assert.Equal(1, prepared.ConstructionBuildCount);
        Assert.Equal(1, first.OrdinaryOffsetterBuildCount); Assert.Equal(1, prepared.OrdinaryOffsetterBuildCount);
    }

    [Fact]
    public void CancelledRequestLeavesConstructionDeferredAndCanBeRetried()
    {
        var actual = Prepare(Circle(0, 0, 10, 256), null, .003);
        using var cancel = new CancellationTokenSource(); cancel.Cancel();
        Assert.Throws<OperationCanceledException>(() => actual.Execute(-1, cancel.Token));
        Assert.False(actual.ConstructionPrepared); Assert.Equal(0, actual.ConstructionBuildCount);
        Assert.Equal(0, actual.OrdinaryOffsetterBuildCount);
        Assert.NotEmpty(actual.Execute(-1));
        Assert.True(actual.ConstructionPrepared); Assert.Equal(1, actual.ConstructionBuildCount);
    }

    private static PolygonOps.PreparedBoundaryOffset Prepare(IReadOnlyList<PointD> outer,
        IEnumerable<IReadOnlyList<PointD>>? holes, double tolerance, double? sweep = null)
        => PolygonOps.PrepareBoundaryOffset(outer, holes, .001, 0,
            sourceConstructionTolerance: tolerance, sourceConstructionSweepRadius: sweep, twoOriginConstruction: false);

    // Force the SAME original algorithms at the former eager evaluation point.
    // Sequence tests then exercise zero/infeasible fronts, caller mutation,
    // tolerance sharing, original-first publication and fixed-cache backtracking
    // under different initialization timing, comparing every ordered coordinate.
    private static void ForceEagerPreparation(PolygonOps.PreparedBoundaryOffset prepared)
        => typeof(PolygonOps.PreparedBoundaryOffset).GetMethod("GetOrdinaryOffsetter",
            BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(prepared, new object[] { CancellationToken.None });

    private void Report(string scenario, PolygonOps.PreparedBoundaryOffset prepared)
        => output.WriteLine($"[Offset.Lazy.Test] scenario={scenario} constructionPrepared={prepared.ConstructionPrepared} "
            + $"constructionBuilds={prepared.ConstructionBuildCount} constructionSkips={prepared.ConstructionSkippedExecutions} "
            + $"ordinaryOffsetterBuilds={prepared.OrdinaryOffsetterBuildCount} outwardOffsetterBuilds={prepared.OutwardOffsetterBuildCount} "
            + $"ordinaryOffsetterSkips={prepared.OrdinaryOffsetterSkippedExecutions} "
            + $"conditioningMs={prepared.ConstructionPreparationMilliseconds:F3} exactOrderedReference=True");

    private static List<List<PointD>> LegacyZeroFront(IReadOnlyList<PointD> outer, IEnumerable<IReadOnlyList<PointD>>? holes)
        => PolygonOps.NormalizeOrder(PolygonOps.MaterializeIntegerPaths(Clipper.Union(OriginalSource(outer, holes), FillRule.NonZero)));

    private static Paths64 OriginalSource(IReadOnlyList<PointD> outer, IEnumerable<IReadOnlyList<PointD>>? holes)
    {
        var source = PathUtils.ToClipper(new[] { outer });
        if (!Clipper.IsPositive(source[0])) source[0].Reverse();
        foreach (var hole in holes ?? Array.Empty<IReadOnlyList<PointD>>())
        {
            var path = PathUtils.ToClipper(new[] { hole })[0];
            if (Clipper.IsPositive(path)) path.Reverse();
            source.Add(path);
        }
        return source;
    }

    private static ClipperOffset LegacyOffsetter(double arc)
        => new(miterLimit: PolygonOps.DefaultMiterLimit, arcTolerance: arc * PathUtils.Scale,
            preserveCollinear: false, reverseSolution: false);

    private static List<PointD> Circle(double x, double y, double radius, int count)
        => Enumerable.Range(0, count).Select(index =>
        {
            double angle = index * 2 * Math.PI / count;
            return new PointD(x + radius * Math.Cos(angle), y + radius * Math.Sin(angle));
        }).ToList();

    private static List<PointD> DenseRectangle(double x0, double y0, double x1, double y1)
    {
        const int count = 2048;
        var points = new List<PointD>(4 * count);
        for (int i = 0; i < count; i++) points.Add(new(x0 + (x1 - x0) * i / count, y0));
        for (int i = 0; i < count; i++) points.Add(new(x1, y0 + (y1 - y0) * i / count));
        for (int i = 0; i < count; i++) points.Add(new(x1 - (x1 - x0) * i / count, y1));
        for (int i = 0; i < count; i++) points.Add(new(x0, y1 - (y1 - y0) * i / count));
        return points;
    }

    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };

    private static void EqualPaths(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
            Assert.Equal(expected[i].Select(point => (point.x, point.y)), actual[i].Select(point => (point.x, point.y)));
    }
}
