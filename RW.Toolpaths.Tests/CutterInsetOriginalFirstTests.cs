using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class CutterInsetOriginalFirstTests
{
    [Fact]
    public void OmittedAndExplicitFalseKeepTheSameOrdinaryConstruction()
    {
        var source = new[] { Rectangle(0, 0, 20, 20), Circle(10, 10, 2, 128).Reverse().ToArray() };
        Assert.Equal(Geometry(PolygonOps.CutterInset(source, 1)),
            Geometry(PolygonOps.CutterInset(source, 1, preferOriginalFirstConstruction: false)));
    }

    [Fact]
    public void DenseHoleOptInUsesExistingOriginalAuthorityAndProvesEveryCommandedChord()
    {
        var source = DenseSource(concaveOuter: false);
        const double radius = 1;
        var expected = ExistingOriginalFirst(source, radius, out var prepared);
        Assert.Equal(1, prepared.OriginalFirstAuthorityCount);
        Assert.Equal(1, prepared.OriginalHoleErosionCount);
        var actual = PolygonOps.CutterInset(Rings(source), radius, preferOriginalFirstConstruction: true);
        Assert.Equal(Geometry(expected), Geometry(actual));
        var component = Assert.Single(PolygonOps.DifferenceToComponents(actual, Array.Empty<IReadOnlyList<PointD>>()));
        Assert.Equal(32, component.Holes.Count);
        ProveCommands(source, actual, radius);
    }

    [Fact]
    public void ConcaveOuterRefusesHoleProposalAndKeepsCompleteOriginalFallback()
    {
        var source = DenseSource(concaveOuter: true);
        const double radius = 1;
        var expected = ExistingOriginalFirst(source, radius, out var prepared);
        Assert.Equal(1, prepared.OriginalFirstAuthorityCount);
        Assert.Equal(0, prepared.OriginalHoleErosionCount);
        var actual = PolygonOps.CutterInset(Rings(source), radius, preferOriginalFirstConstruction: true);
        Assert.Equal(Geometry(expected), Geometry(actual));
        ProveCommands(source, actual, radius);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CallerCandidateCannotBypassOriginalContainmentProof(bool preferOriginalFirst)
    {
        var source = new[] { Rectangle(0, 0, 20, 20) };
        var outside = new[] { Rectangle(-2, -2, 22, 22) };
        Assert.Throws<InvalidOperationException>(() => PolygonOps.CutterInset(source, 1,
            candidateRings: outside, preferOriginalFirstConstruction: preferOriginalFirst));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CancelledCallCannotPublishOrdinaryOrPreferredFront(bool preferOriginalFirst)
    {
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => PolygonOps.CutterInset(
            new[] { Rectangle(0, 0, 20, 20) }, 1, cancellationToken: cancellation.Token,
            preferOriginalFirstConstruction: preferOriginalFirst));
    }

    private static List<List<PointD>> ExistingOriginalFirst(MillingRegion source, double radius,
        out PolygonOps.PreparedBoundaryOffset prepared)
    {
        var rings = Rings(source).ToArray();
        var canonical = Clipper.Union(PathUtils.ToClipper(rings), FillRule.NonZero);
        var components = PolygonOps.DifferenceToComponents(PathUtils.FromClipper(canonical),
            Array.Empty<IReadOnlyList<PointD>>());
        var component = Assert.Single(components);
        // The fixture is bounded by64mm. Use the existing reserve contract to
        // compare the new selection with the already-tested prepared owner.
        const double quantum = 1 / PathUtils.Scale;
        double commandError = Math.Sqrt(2) * (MathF.BitIncrement(64f) - 64f) / 2;
        double reserve = Math.Max(8 * quantum, commandError + Math.Sqrt(2) * .0005 + 6 * quantum);
        prepared = PolygonOps.PrepareBoundaryOffset(component.Outer, component.Holes,
            arcTolerance: quantum, simplifyTolerance: 0, sourceConstructionTolerance: .001,
            sourceConstructionSweepRadius: radius, separateOriginalFirstConstruction: true);
        var front = prepared.ExecuteOriginalFirstAuthority(-(radius + reserve));
        var candidate = PathUtils.FromClipper(Clipper.Union(PathUtils.ToClipper(front), FillRule.NonZero));
        return PolygonOps.CutterInset(rings, radius, candidateRings: candidate);
    }

    private static void ProveCommands(MillingRegion source, IEnumerable<IReadOnlyList<PointD>> paths, double radius)
    {
        var material = Clipper.Union(PathUtils.ToClipper(Rings(source)), FillRule.NonZero);
        var boundary = new PolygonOps.BoundarySegmentIndex(material);
        double required = (radius + Math.Sqrt(2) * .0005) * PathUtils.Scale + 2;
        foreach (var path in paths)
        for (int end = 0, start = path.Count - 1; end < path.Count; start = end++)
        {
            Point64 Command(PointD point) => new((long)Math.Round((float)point.x * PathUtils.Scale),
                (long)Math.Round((float)point.y * PathUtils.Scale));
            var a = Command(path[start]); var b = Command(path[end]);
            Assert.True(boundary.IsInside(a));
            Assert.Null(boundary.ClearanceViolation(a, b, required));
        }
    }

    private static MillingRegion DenseSource(bool concaveOuter)
    {
        IReadOnlyList<PointD> outer = concaveOuter
            ? new PointD[] { new(0, 0), new(64, 0), new(64, 64), new(48, 64),
                new(48, 60), new(44, 60), new(44, 64), new(0, 64) }
            : Rectangle(0, 0, 64, 64);
        return new(outer, Enumerable.Range(0, 32).Select(i => (IReadOnlyList<PointD>)
            Circle(8 + 8 * (i % 6), 8 + 8 * (i / 6), 2, 512).Reverse().ToArray()).ToArray());
    }

    private static IEnumerable<IReadOnlyList<PointD>> Rings(MillingRegion source)
        => new[] { source.Outer }.Concat(source.Holes);
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => new PointD[] { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
    private static PointD[] Circle(double x, double y, double radius, int count)
        => Enumerable.Range(0, count).Select(i => new PointD(x + radius * Math.Cos(2 * Math.PI * i / count),
            y + radius * Math.Sin(2 * Math.PI * i / count))).ToArray();
    private static string[] Geometry(IEnumerable<IReadOnlyList<PointD>> paths)
        => paths.Select(path => string.Join(";", path.Select(point => $"{point.x:R},{point.y:R}"))).ToArray();
}
