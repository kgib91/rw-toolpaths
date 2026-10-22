using System.Runtime.InteropServices;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Classifier = RW.Toolpaths.MedialAxisEdgeClassifier;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Full source medial reference controls", DisableParallelization = true)]
public sealed class FullSourceMedialReferenceCollection { }

[Collection("Full source medial reference controls")]
public sealed class SourceMedialFiniteChordTests
{
    [Fact]
    public void FiniteCapsulesRetainAProjectionReversalAndExactNativeEndpoints()
    {
        var edge = Parabola(-2, 10);
        var output = new List<MedialSegment>();
        Assert.True(Emit(edge, new(0, 0, 1), output, out _, out _, out double bound));
        Assert.True(output.Count > 1);
        Assert.InRange(bound, 0, .001);
        Assert.InRange(Sample(output, t => new(-2 + 12 * t, (Math.Pow(-2 + 12 * t, 2) + 4) / 4)), 0, .001);
        RadiusAndContinuity(edge, output);
    }

    [Fact]
    public void ReversingAnOriginalEdgeExactlyReversesItsEndpointAndRadiusStream()
    {
        var edge = Parabola(-2, 10);
        var output = new List<MedialSegment>(); var reversed = new List<MedialSegment>();
        Assert.True(Emit(edge, new(0, 0, 1), output, out _, out _, out _));
        Assert.True(Emit(edge with { Vertex0 = edge.Vertex1, Vertex1 = edge.Vertex0 },
            new(0, 0, 1), reversed, out _, out _, out _));
        Assert.Equal(output.Count, reversed.Count);
        for (int i = 0; i < output.Count; i++)
        {
            Assert.Equal(output[i].Point0, reversed[^(i + 1)].Point1);
            Assert.Equal(output[i].Point1, reversed[^(i + 1)].Point0);
        }
    }

    [Fact]
    public void InfiniteLineFlatnessCannotAcceptAChordWithAReversedFiniteProjection()
    {
        var edge = SkewParabola(out var point);
        const double scale = 32768;
        var infinite = new List<MedialSegment>();
        MedialAxisSegmentEmitter.TryCreateOutputSegments(edge, infinite, false, false, .001 * scale,
            MedialAxisSegmentEmitter.ParabolaDiscretizationMethod.Error2D, -1, true);
        Assert.Single(infinite);
        Assert.True(Distance(point(100d / 101), infinite[0]) > .001 * scale * 50);
        var finite = new List<MedialSegment>();
        Assert.True(Emit(edge, new(0, 0, scale), finite, out _, out _, out double bound));
        Assert.True(finite.Count > 1);
        Assert.InRange(bound, 0, .001);
        Assert.InRange(Sample(finite, point) / scale, 0, .001);
        RadiusAndContinuity(edge, finite);
    }

    [Fact]
    public void ADisplacedNativeEndpointCannotUseAnInfiniteLineToHideItsFitError()
    {
        var edge = Parabola(-2, 10);
        edge = edge with { Vertex0 = new(edge.Vertex0!.X, edge.Vertex0.Y + .1) };
        var output = new List<MedialSegment>();
        Assert.False(Emit(edge, new(0, 0, 1), output, out string reason, out _, out _));
        Assert.Equal("native-endpoint-fit", reason);
        Assert.Empty(output);
    }

    [Fact]
    public void DegenerateFocusAndDirectrixNeverPublishPartOfAnOriginalEdge()
    {
        var output = new List<MedialSegment>();
        Assert.False(Emit(Parabola(-2, 10) with { Cell = new(true, new(0, 0), null) },
            new(0, 0, 1), output, out string reason, out _, out _));
        Assert.Equal("degenerate-focus-directrix", reason);
        Assert.Empty(output);
    }

    [Fact]
    public void LargeIntegerCancellationAndIncompatibleNativeEndpointsRemainUncertified()
    {
        var edge = new Classifier.EdgeData(0, 1, true, false, false, -1, -1, new(0, 1), new(100, 101),
            new(true, new(999999999, 999999998), null),
            new(false, null, new(new(0, 0), new(1000000000, 999999999)))) { IsCurved = true };
        var output = new List<MedialSegment>();
        Assert.False(Emit(edge, new(0, 0, 1), output, out string reason, out _, out _));
        Assert.Equal("native-endpoint-fit", reason);
        Assert.Empty(output);
    }

    [Fact]
    public void BudgetFailureAfterAStagedLeafPublishesNothingAndAllowsTheCompleteReferenceEdge()
    {
        var edge = Parabola(-2, 10);
        var sentinel = new MedialSegment(new(-5, -5, 1), new(-4, -4, 1));
        var output = new List<MedialSegment> { sentinel };
        Assert.False(BoundedLocalMedialEmitter.TryEmitFull(edge, new(0, 0, 1), .01, output,
            CancellationToken.None, out string reason, out int visited, out double stagedBound, maximumPieces: 8));
        Assert.Equal("piece-budget", reason);
        Assert.Equal(9, visited);
        Assert.True(stagedBound > 0);
        Assert.Equal(sentinel, Assert.Single(output));
        var reference = new List<MedialSegment>();
        Legacy(edge, reference); Legacy(edge, output);
        Assert.Equal(reference, output.Skip(1));
    }

    [Fact]
    public void CancelledFullCurveCannotPublishAnyGeometry()
    {
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        var output = new List<MedialSegment>();
        Assert.Throws<OperationCanceledException>(() => BoundedLocalMedialEmitter.TryEmitFull(
            Parabola(-2, 10), new(0, 0, 1), .001, output, cancellation.Token, out _, out _, out _));
        Assert.Empty(output);
    }

    [Fact]
    public void LinearOriginalVerticesAndFocusRadiiRemainExactlyUnchanged()
    {
        var edge = Parabola(-2, 10) with { IsCurved = false, IsLinear = true,
            Cell = new(true, new(-3, 2), null), TwinCell = new(true, new(3, 2), null),
            Vertex0 = new(0, -10), Vertex1 = new(0, 20) };
        var reference = new List<MedialSegment>(); var output = new List<MedialSegment>();
        Legacy(edge, reference);
        Assert.True(Emit(edge, new(0, 0, 1), output, out _, out _, out _));
        Assert.Equal(reference, output);
        Assert.All(output, s => { Assert.True(s.Point0.Radius > 1); Assert.True(s.Point1.Radius > 1); });
    }

    [Fact]
    public void NonPowerOfTwoScaleAndLargeWorldTranslationAreIncludedInTheFiniteCertificate()
    {
        var edge = Parabola(-2, 10);
        var placement = new BoundedLocalMedialEmitter.SourceCoordinates(1e10 + .125, -1e10 + .25, 30789.125);
        var output = new List<MedialSegment>();
        Assert.True(Emit(edge, placement, output, out _, out _, out double bound));
        Assert.InRange(bound, 0, .001);
        var world = output.Select(s => new MedialSegment(Convert(s.Point0), Convert(s.Point1))).ToList();
        double worst = Sample(world, t => {
            double x = -2 + 12 * t; return new(x / placement.Scale + placement.OriginX,
                (x * x + 4) / 4 / placement.Scale + placement.OriginY); });
        Assert.InRange(worst, 0, .001);
        MedialPoint Convert(MedialPoint p) => new(p.X / placement.Scale + placement.OriginX,
            p.Y / placement.Scale + placement.OriginY, p.Radius / placement.Scale);
    }

    [Fact]
    public void UnrepresentableWorldPlacementRefusesInsteadOfClaimingAFalse001Bound()
    {
        var output = new List<MedialSegment>();
        Assert.False(Emit(Parabola(-2, 10), new(1e15, -1e15, 32768), output, out string reason, out _, out _));
        Assert.Equal("native-endpoint-fit", reason);
        Assert.Empty(output);
    }

    [Fact]
    public void SourceReferenceFlagResolvesBeforeLazyGraphPublicationAndRestoresAngularBytes()
    {
        const string flag = "RW_TOOLPATHS_SOURCE_MEDIAL_REFERENCE";
        string? previous = Environment.GetEnvironmentVariable(flag);
        try
        {
            Environment.SetEnvironmentVariable(flag, null);
            var source = new MillingRegion(Rectangle(0, 0, 30, 40), new[] { Rectangle(8, 12, 15, 19) });
            var legacy = new PreparedTerminalMedial(source, CancellationToken.None);
            Environment.SetEnvironmentVariable(flag, "1");
            var reference = new PreparedTerminalMedial(source, CancellationToken.None, lazy: true, boundOriginalSource: true);
            Assert.False(reference.FullGraphMaterialized);
            Assert.True(SegmentBytes(legacy.Segments).SequenceEqual(SegmentBytes(reference.Segments)));
            Assert.True(reference.FullGraphMaterialized);
            Assert.False(reference.Boundary.IsInside(new Point64(10 * PathUtils.Scale, 15 * PathUtils.Scale)));
        }
        finally { Environment.SetEnvironmentVariable(flag, previous); }
    }

    [Fact]
    public void DefaultSharedFillEligibilityAndPublicProviderRemainAngularAfterBoundedSourceUse()
    {
        Assert.True(new PocketOptions().BoundOriginalSourceMedial);
        Assert.False(new RingOffsetOptions(1, 1).BoundOriginalSourceMedial);
        var outer = Rectangle(0, 0, 30, 40); var holes = new[] { Rectangle(8, 12, 15, 19) };
        var provider = BoostVoronoiProvider.CreateDefault();
        var before = MedialAxisGeometry.Build(outer, holes, provider, .01, 80);
        var bound = MedialAxisGeometry.BuildBoundedSource(outer, holes, .01, 80, 3 * Math.PI / 4, CancellationToken.None);
        Assert.NotEmpty(bound);
        var after = MedialAxisGeometry.Build(outer, holes, provider, .01, 80);
        Assert.True(SegmentBytes(before).SequenceEqual(SegmentBytes(after)));
    }

    [Fact]
    public void IndirectCarvingPocketSlicesPreserveEveryAngularCommandByte()
    {
        const string flag = "RW_TOOLPATHS_SOURCE_MEDIAL_REFERENCE";
        string? previous = Environment.GetEnvironmentVariable(flag);
        try
        {
            const int size = 33;
            var heights = Enumerable.Range(0, size * size).Select(i =>
            {
                double dx = i % size - 16, dy = i / size - 16;
                return (float)(.1 + .7 * Math.Exp(-(dx * dx + dy * dy) / 30));
            }).ToArray();
            var surface = new HeightmapSurface(new HeightmapGrid(size, size, heights),
                new HeightmapPlacement(50, 40, 0, 0), 10, 6);
            var tool = new ToolGeometry(2);
            var options = new CarvingOptions { StepOver = .4, SurfaceAllowance = .2, CalculateSweptArea = false };
            Environment.SetEnvironmentVariable(flag, null);
            var actual = CarvingRoughingToolpaths.Generate(surface, tool, 2, options);
            Environment.SetEnvironmentVariable(flag, "1");
            var angular = CarvingRoughingToolpaths.Generate(surface, tool, 2, options);
            Assert.NotEmpty(actual.Toolpaths);
            Assert.Equal(angular.Diagnostics, actual.Diagnostics);
            Assert.Equal(angular.Toolpaths.Count, actual.Toolpaths.Count);
            for (int i = 0; i < actual.Toolpaths.Count; i++)
            {
                var a = actual.Toolpaths[i]; var b = angular.Toolpaths[i];
                Assert.True(MemoryMarshal.AsBytes(a.Points.ToArray().AsSpan()).SequenceEqual(
                    MemoryMarshal.AsBytes(b.Points.ToArray().AsSpan())));
                Assert.Equal(b.Spans, a.Spans);
                Assert.Equal(b.DepthPassIndex, a.DepthPassIndex);
                Assert.Equal(b.VisitIdentity, a.VisitIdentity);
                Assert.Equal(b.RegionIndex, a.RegionIndex);
                Assert.Equal(b.Category, a.Category);
            }
        }
        finally { Environment.SetEnvironmentVariable(flag, previous); }
    }

    private static bool Emit(Classifier.EdgeData edge, BoundedLocalMedialEmitter.SourceCoordinates placement,
        List<MedialSegment> output, out string reason, out int visited, out double bound)
        => BoundedLocalMedialEmitter.TryEmitFull(edge, placement, .001, output, CancellationToken.None,
            out reason, out visited, out bound);
    private static void Legacy(Classifier.EdgeData edge, List<MedialSegment> output)
        => MedialAxisSegmentEmitter.TryCreateOutputSegments(edge, output, false, false, .01,
            MedialAxisSegmentEmitter.ParabolaDiscretizationMethod.CentralAngle, -1, false);
    private static byte[] SegmentBytes(IReadOnlyList<MedialSegment> values)
        => MemoryMarshal.AsBytes(values.ToArray().AsSpan()).ToArray();
    private static Classifier.EdgeData Parabola(double a, double b)
        => new(0, 1, true, false, false, -1, -1, new(a, (a * a + 4) / 4), new(b, (b * b + 4) / 4),
            new(true, new(0, 2), null), new(false, null, new(new(-20, 0), new(20, 0)))) { IsCurved = true };
    private static Classifier.EdgeData SkewParabola(out Func<double, Classifier.EdgePoint> point)
    {
        const double dx = 10000, dy = 99;
        double length = Math.Sqrt(dx * dx + dy * dy), projection = (dx * 101 + dy) / length, normal = 1 / length;
        Classifier.EdgePoint Evaluate(double t)
        {
            double u = -100 + 101 * t, projected = projection + u, height = (u * u + normal * normal) / (2 * normal);
            return new((dx * projected - dy * height) / length, (dy * projected + dx * height) / length);
        }
        point = Evaluate;
        return new(0, 1, true, false, false, -1, -1, Evaluate(0), Evaluate(1),
            new(true, new(101, 1), null), new(false, null, new(new(0, 0), new(10000, 99)))) { IsCurved = true };
    }
    private static double Sample(List<MedialSegment> output, Func<double, Classifier.EdgePoint> point)
    {
        double worst = 0;
        for (int i = 0; i <= 4000; i++) worst = Math.Max(worst, output.Min(s => Distance(point(i / 4000d), s)));
        return worst;
    }
    private static double Distance(Classifier.EdgePoint p, MedialSegment s)
        => Math.Sqrt(BoundedLocalMedialEmitter.FiniteChordDistanceSquared(p,
            new(s.Point0.X, s.Point0.Y), new(s.Point1.X, s.Point1.Y)));
    private static void RadiusAndContinuity(Classifier.EdgeData edge, List<MedialSegment> output)
    {
        var focus = edge.Cell.ContainsPoint ? edge.Cell.Point! : edge.TwinCell.Point!;
        Assert.Equal(edge.Vertex0!.X, output[0].Point0.X); Assert.Equal(edge.Vertex0.Y, output[0].Point0.Y);
        Assert.Equal(edge.Vertex1!.X, output[^1].Point1.X); Assert.Equal(edge.Vertex1.Y, output[^1].Point1.Y);
        for (int i = 0; i < output.Count; i++)
        {
            foreach (var p in new[] { output[i].Point0, output[i].Point1 })
            {
                double dx = p.X - focus.X, dy = p.Y - focus.Y;
                Assert.Equal(Math.Sqrt(dx * dx + dy * dy), p.Radius);
            }
            if (i > 0) Assert.Equal(output[i - 1].Point1, output[i].Point0);
        }
    }
    private static IReadOnlyList<PointD> Rectangle(double a, double b, double c, double d)
        => new PointD[] { new(a, b), new(c, b), new(c, d), new(a, d) };
}
