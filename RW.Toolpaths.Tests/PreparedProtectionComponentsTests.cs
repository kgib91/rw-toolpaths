using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[Collection("Full source medial reference controls")]
public sealed class PreparedProtectionComponentsTests
{
    [Fact]
    public void EnclosingSignedHoleAndNestedIslandRemainCompleteWithoutEdgeContact()
    {
        IReadOnlyList<PointD>[] mask = [Box(-10, -10, 30, 30), Negative(Box(1, 1, 19, 19)),
            Box(8, 8, 12, 12), Box(100, 100, 110, 110)];
        var prepared = PolygonOps.PrepareComponentDifference(mask, CancellationToken.None);
        var subject = new[] { Box(0, 0, 20, 20) };
        var actual = Query(prepared, subject);
        EqualComponents(PolygonOps.DifferenceToComponents(subject, mask), actual);
        var component = Assert.Single(actual);
        Assert.Single(component.Holes);
        Assert.True(PolygonOps.Contains(Flatten(actual), 2, 2));
        Assert.False(PolygonOps.Contains(Flatten(actual), 10, 10));
        Assert.False(PolygonOps.Contains(Flatten(actual), .5, .5));

        // Neither enclosing signed contour has an edge in this smaller subject.
        // Its positive/negative winding must cancel across the complete subject.
        var insideHole = new[] { Box(2, 2, 6, 6) };
        EqualComponents(PolygonOps.DifferenceToComponents(insideHole, mask), Query(prepared, insideHole));
        Assert.Empty(Assert.Single(Query(prepared, insideHole)).Holes);
        Assert.True(prepared.SelectedVertices < prepared.ConsideredVertices);
    }

    [Fact]
    public void EnclosingPositiveMaskIsSelectedEvenThoughItsEdgesMissSubject()
    {
        IReadOnlyList<PointD>[] mask = [Box(-10, -10, 30, 30), Box(100, 0, 110, 10)];
        var prepared = PolygonOps.PrepareComponentDifference(mask, CancellationToken.None);
        Assert.Empty(Query(prepared, [Box(0, 0, 20, 20)]));
        Assert.Equal(4, prepared.SelectedVertices);
        Assert.Equal(8, prepared.ConsideredVertices);
    }

    [Theory]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    public void CompleteContourContactGuardIncludesOneTickAndPrunesTwoTickGap(int gapTicks, bool selected)
    {
        double gap = gapTicks / PathUtils.Scale;
        IReadOnlyList<PointD>[] subject = [Box(0, 0, 10, 10)];
        IReadOnlyList<PointD>[] mask = [Box(10 + gap, 0, 20, 10)];
        var prepared = PolygonOps.PrepareComponentDifference(mask, CancellationToken.None);
        EqualComponents(PolygonOps.DifferenceToComponents(subject, mask), Query(prepared, subject));
        Assert.Equal(selected ? 4 : 0, prepared.SelectedVertices);
        Assert.Equal(selected ? 1 : 0, prepared.InputBuilds);
    }

    [Fact]
    public void EqualYMinimaTouchingContoursAndNegativeCoordinatesKeepLiteralTreeOrder()
    {
        IReadOnlyList<PointD>[] mask = [Box(-15, -3, -12, 4), Box(100, -3, 110, 4),
            Box(3, -3, 6, 4), Box(18, -3, 20, 4), Box(-5, -3, -2, 4)];
        IReadOnlyList<PointD>[] subject = [Box(-20, -10, 20, 10), Negative(Box(-10, -5, -8, -1))];
        var prepared = PolygonOps.PrepareComponentDifference(mask, CancellationToken.None);
        var actual = Query(prepared, subject);
        EqualComponents(PolygonOps.DifferenceToComponents(subject, mask), actual);
        Assert.Single(actual);
        Assert.True(actual[0].Holes.Count >= 4);
        Assert.True(prepared.SelectedVertices < prepared.ConsideredVertices);
    }

    [Fact]
    public void EmptyOrDisjointSelectionStillExecutesOriginalSubjectTreeCollection()
    {
        IReadOnlyList<PointD>[] subjects = [Box(0, 0, 20, 20), Negative(Box(2, 2, 18, 18)),
            Box(8, 8, 12, 12), Box(30, 0, 40, 10)];
        foreach (var clips in new IReadOnlyList<PointD>[][] { [], [Box(100, 100, 110, 110)] })
        {
            var prepared = PolygonOps.PrepareComponentDifference(clips, CancellationToken.None);
            var actual = Query(prepared, subjects);
            EqualComponents(PolygonOps.DifferenceToComponents(subjects, clips), actual);
            Assert.Equal(3, actual.Count);
            Assert.Equal(1, actual.Sum(component => component.Holes.Count));
            Assert.Equal(0, prepared.SelectedVertices);
            Assert.Equal(1, prepared.DifferenceQueries);
        }
    }

    [Fact]
    public void ConversionKeepsClosingDuplicatesMinimumVerticesAndTiesToEven()
    {
        double quantum = 1 / PathUtils.Scale;
        IReadOnlyList<PointD> ring = [new(-1.5 * quantum, -.5 * quantum), new(1.5 * quantum, -.5 * quantum),
            new(1.5 * quantum, 1.5 * quantum), new(-1.5 * quantum, -.5 * quantum)];
        var integer = PolygonOps.ComponentDifferencePaths([ring, [new(7, 7), new(8, 8)]], CancellationToken.None);
        var actual = Assert.Single(integer);
        Assert.Equal(4, actual.Count);
        Assert.Equal(new long[] { -2, 0, 2, 0, 2, 2, -2, 0 }, actual.SelectMany(p => new[] { p.X, p.Y }));
        Assert.Equal(actual[0], actual[^1]);
        var prepared = PolygonOps.PrepareComponentDifference([ring, [new(7, 7), new(8, 8)]], CancellationToken.None);
        Assert.Equal(1, prepared.OriginalContours);
        Assert.Equal(4, prepared.OriginalVertices);
        EqualComponents(PolygonOps.DifferenceToComponents([Box(-1, -1, 1, 1)], [ring]),
            Query(prepared, [Box(-1, -1, 1, 1)]));
    }

    [Fact]
    public void RepeatedQueriesOwnMaskAndOutputsAndReuseOnlyCompleteSelectedContours()
    {
        var mask = Enumerable.Range(0, 32).Select(i => Box(i * 100, 0, i * 100 + 10, 10).ToList()).ToList();
        var frozen = mask.Select(path => (IReadOnlyList<PointD>)path.ToArray()).ToArray();
        var prepared = PolygonOps.PrepareComponentDifference(mask, CancellationToken.None);
        mask[0].Clear(); mask.Clear();
        for (int i = 0; i < 8; i++)
        {
            IReadOnlyList<PointD>[] subject = [Box(-1 + i / 100d, -1, 5, 11)];
            var actual = Query(prepared, subject);
            EqualComponents(PolygonOps.DifferenceToComponents(subject, frozen), actual);
            foreach (var component in actual) { component.Outer.Clear(); component.Holes.Clear(); }
        }
        Assert.Equal(1, prepared.InputBuilds);
        Assert.Equal(7, prepared.InputReuses);
        Assert.Equal(32, prepared.SelectedVertices);
        Assert.Equal(1024, prepared.ConsideredVertices);
    }

    [Fact]
    public void CancellationPublishesNoComponentsOrPreparedInputsAndLaterQueryRemainsExact()
    {
        IReadOnlyList<PointD>[] mask = [Box(4, 4, 16, 16)];
        IReadOnlyList<PointD>[] subject = [Box(0, 0, 20, 20)];
        var prepared = PolygonOps.PrepareComponentDifference(mask, CancellationToken.None);
        using var canceled = new CancellationTokenSource(); canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => PolygonOps.PrepareComponentDifference(mask, canceled.Token));
        Assert.Throws<OperationCanceledException>(() => prepared.DifferenceComponents(
            PolygonOps.ComponentDifferencePaths(subject, CancellationToken.None), canceled.Token));
        Assert.Equal(0, prepared.InputBuilds);
        Assert.Equal(0, prepared.DifferenceQueries);
        EqualComponents(PolygonOps.DifferenceToComponents(subject, mask), Query(prepared, subject));
        var tree = new PolyTree64(); tree.AddChild(new Path64 { new(0, 0), new(10, 0), new(10, 10), new(0, 10) });
        Assert.Throws<OperationCanceledException>(() => PolygonOps.MaterializeComponents(tree, canceled.Token));
        Assert.Equal(1, tree.Count);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FailedComponentExecutionClearsEmptyOrPartialTreeAndNeverPublishesIt(bool partial)
    {
        var tree = new PolyTree64();
        if (partial)
            tree.AddChild(new Path64 { new(0, 0), new(20, 0), new(20, 20), new(0, 20) })
                .AddChild(new Path64 { new(4, 4), new(4, 16), new(16, 16), new(16, 4) });
        var failure = Assert.Throws<InvalidOperationException>(() =>
            PreparedResidualCoverage.RequireSuccessfulComponentDifference(false, tree));
        Assert.Contains("Prepared protection component difference", failure.Message);
        Assert.Contains("partial components were discarded", failure.Message);
        Assert.Equal(0, tree.Count);
    }

    [Fact]
    public void SuccessfulExecutionGuardPreservesCompleteNestedTreeAndLiteralCollectorOrder()
    {
        var tree = new PolyTree64();
        tree.AddChild(new Path64 { new(0, 0), new(20, 0), new(20, 20), new(0, 20) })
            .AddChild(new Path64 { new(2, 2), new(2, 18), new(18, 18), new(18, 2) })
            .AddChild(new Path64 { new(8, 8), new(12, 8), new(12, 12), new(8, 12) });
        var before = PolygonOps.MaterializeComponents(tree);
        var first = tree.Child(0);
        PreparedResidualCoverage.RequireSuccessfulComponentDifference(true, tree);
        Assert.Same(first, tree.Child(0));
        var actual = PolygonOps.MaterializeComponents(tree);
        EqualComponents(before, actual);
        Assert.Equal(2, actual.Count);
        Assert.Equal(new PointD(0, 0), actual[0].Outer[0]);
        Assert.Equal(8 / PathUtils.Scale, actual[1].Outer[0].x);
        Assert.Equal(8 / PathUtils.Scale, actual[1].Outer[0].y);
    }

    [Fact]
    public void PhysicalReferenceKeepsAllRegionCoordinatesAndGenericPathIgnoresDiagnosticPolicy()
    {
        var regions = new[] { new MillingRegion(Box(-20, -20, 20, 20), [Box(-15, -15, -12, -12)]),
            new MillingRegion(Box(30, 0, 40, 10)) };
        var options = new PocketOptions { StockToLeave = .25,
            ProtectedBoundary = [Box(-10, -10, 10, 10), Negative(Box(-6, -6, 6, 6)), Box(-2, -2, 2, 2),
                Box(100, 100, 110, 110)] };
        const string flag = "RW_TOOLPATHS_PROTECTION_COMPONENT_REFERENCE";
        string? previous = Environment.GetEnvironmentVariable(flag);
        try
        {
            Environment.SetEnvironmentVariable(flag, "1");
            var physicalReference = PocketProtection.Subtract(regions, options, CancellationToken.None, physicalConcentric: true);
            var genericReference = PocketProtection.Subtract(regions, options, CancellationToken.None, physicalConcentric: false);
            Environment.SetEnvironmentVariable(flag, null);
            EqualRegions(physicalReference, PocketProtection.Subtract(regions, options, CancellationToken.None, physicalConcentric: true));
            EqualRegions(genericReference, PocketProtection.Subtract(regions, options, CancellationToken.None, physicalConcentric: false));
        }
        finally { Environment.SetEnvironmentVariable(flag, previous); }
    }

    private static List<PolygonComponent> Query(PreparedResidualCoverage prepared, IEnumerable<IReadOnlyList<PointD>> subject)
        => prepared.DifferenceComponents(PolygonOps.ComponentDifferencePaths(subject, CancellationToken.None), CancellationToken.None);

    private static void EqualComponents(IReadOnlyList<PolygonComponent> expected, IReadOnlyList<PolygonComponent> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
        {
            EqualPath(expected[i].Outer, actual[i].Outer);
            Assert.Equal(expected[i].Holes.Count, actual[i].Holes.Count);
            for (int j = 0; j < expected[i].Holes.Count; j++) EqualPath(expected[i].Holes[j], actual[i].Holes[j]);
        }
    }

    private static void EqualRegions(IReadOnlyList<MillingRegion> expected, IReadOnlyList<MillingRegion> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
        {
            EqualPath(expected[i].Outer, actual[i].Outer);
            Assert.Equal(expected[i].Holes.Count, actual[i].Holes.Count);
            for (int j = 0; j < expected[i].Holes.Count; j++) EqualPath(expected[i].Holes[j], actual[i].Holes[j]);
        }
    }

    private static void EqualPath(IReadOnlyList<PointD> expected, IReadOnlyList<PointD> actual)
        => Assert.Equal(expected.SelectMany(p => new[] { BitConverter.DoubleToInt64Bits(p.x), BitConverter.DoubleToInt64Bits(p.y) }),
            actual.SelectMany(p => new[] { BitConverter.DoubleToInt64Bits(p.x), BitConverter.DoubleToInt64Bits(p.y) }));

    private static IReadOnlyList<IReadOnlyList<PointD>> Flatten(IEnumerable<PolygonComponent> components)
        => components.SelectMany(component => new[] { (IReadOnlyList<PointD>)component.Outer }
            .Concat(component.Holes)).ToArray();

    private static IReadOnlyList<PointD> Negative(IReadOnlyList<PointD> path) => path.Reverse().ToArray();
    private static IReadOnlyList<PointD> Box(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
}
