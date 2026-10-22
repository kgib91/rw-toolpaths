using System.Runtime.InteropServices;
using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class OriginalHoleErosionTests
{
    [Fact]
    public void CompleteDenseOriginalFirstPreservesEveryHoleAndFullFloatWall()
    {
        var source = Source(); var prepared = Prepare(source, true);
        var first = prepared.ExecuteOriginalFirstAuthority(-1);
        Assert.Equal(1, prepared.OriginalHoleErosionCount);
        var component = Assert.Single(PolygonOps.DifferenceToComponents(first, Array.Empty<IReadOnlyList<PointD>>()));
        Assert.Equal(32, component.Holes.Count);
        var original = PathUtils.ToClipper(new[] { source.Outer }.Concat(source.Holes));
        var boundary = new PolygonOps.BoundarySegmentIndex(Clipper.Union(original, FillRule.NonZero));
        foreach (var path in first)
        for (int i = 0; i < path.Count; i++)
        {
            Point64 Command(PointD p) => new((long)Math.Round((float)p.x * PathUtils.Scale),
                (long)Math.Round((float)p.y * PathUtils.Scale));
            var a = Command(path[i]); var b = Command(path[(i + 1) % path.Count]);
            Assert.True(boundary.IsInside(a));
            Assert.Null(boundary.ClearanceViolation(a, b, PathUtils.Scale - 4));
        }
        Assert.Equal(Geometry(first), Geometry(prepared.ExecuteOriginalFirstAuthority(-1)));
        Assert.Equal(1, prepared.OriginalHoleErosionCount);
    }

    [Fact]
    public void GenericAndLaterConstructionNeverUsesInitialHoleProposal()
    {
        var source = Source(); var ordinary = Prepare(source, false); var scoped = Prepare(source, true);
        Assert.Equal(Geometry(ordinary.Execute(-1)), Geometry(scoped.Execute(-1)));
        Assert.Equal(0, scoped.OriginalHoleErosionCount);
        var first = Prepare(source, true); first.ExecuteOriginalFirstAuthority(-1);
        Assert.Equal(1, first.OriginalHoleErosionCount);
        first.Execute(-2);
        Assert.Equal(1, first.OriginalHoleErosionCount);
        Assert.False(new RingOffsetOptions(1, 1).SeparateOriginalFirstConstruction);
    }

    [Fact]
    public void CapturedReferenceAndExplicitFalseCannotBeChangedByLaterEnvironment()
    {
        const string name = "RW_TOOLPATHS_FIRST_HOLE_EROSION_REFERENCE";
        string? previous = Environment.GetEnvironmentVariable(name);
        try
        {
            Environment.SetEnvironmentVariable(name, null);
            var enabled = PocketToolpaths.CaptureConcentricConstructionMode(new PocketOptions());
            Assert.True(enabled.SeparateOriginalFirstConstruction);
            Environment.SetEnvironmentVariable(name, "1");
            Assert.True(enabled.SeparateOriginalFirstConstruction);
            Assert.False(PocketToolpaths.CaptureConcentricConstructionMode(new PocketOptions()).SeparateOriginalFirstConstruction);
            Assert.False(PocketToolpaths.CaptureConcentricConstructionMode(new PocketOptions {
                SeparateOriginalFirstConstruction = false }).SeparateOriginalFirstConstruction);
        }
        finally { Environment.SetEnvironmentVariable(name, previous); }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void ZeroAndOutwardOffsetsRetainCompleteReferenceBytes(double delta)
    {
        var source = Source(); var proposed = Prepare(source, true); var original = Prepare(source, false);
        Assert.Equal(Geometry(original.ExecuteOriginalFirstAuthority(delta)),
            Geometry(proposed.ExecuteOriginalFirstAuthority(delta)));
        Assert.Equal(0, proposed.OriginalHoleErosionCount);
    }

    [Fact]
    public void TouchingHoleKnotsRefuseProposalAndRetainCompleteOriginalConstruction()
    {
        var source = Source(); var holes = source.Holes.ToArray();
        // Tangent holes meet at an exact native knot; canonical contact policy
        // must remain the established complete construction's decision.
        holes[1] = Circle(12, 8, 2, 512);
        var touched = new MillingRegion(source.Outer, holes);
        var expected = Prepare(touched, false).ExecuteOriginalFirstAuthority(-1);
        var proposed = Prepare(touched, true);
        Assert.Equal(Geometry(expected), Geometry(proposed.ExecuteOriginalFirstAuthority(-1)));
        Assert.Equal(0, proposed.OriginalHoleErosionCount);
    }

    [Fact]
    public void ConcaveOuterUsesCompleteOriginalFallback()
    {
        var source = Source();
        var outer = new[] { new PointD(0, 0), new(64, 0), new(64, 64), new(48, 64),
            new(48, 60), new(44, 60), new(44, 64), new(0, 64) };
        var concave = new MillingRegion(outer, source.Holes);
        var proposed = Prepare(concave, true);
        Assert.Equal(Geometry(Prepare(concave, false).ExecuteOriginalFirstAuthority(-1)),
            Geometry(proposed.ExecuteOriginalFirstAuthority(-1)));
        Assert.Equal(0, proposed.OriginalHoleErosionCount);
    }

    [Fact]
    public void NestedPositiveIslandCannotBeReplacedByOneOuterImmediateHoleDecomposition()
    {
        var source = Source(); var holes = source.Holes.ToArray();
        // Both input masks are negative. Their nested NonZero winding creates a
        // filled inner island, so the one-outer/immediate-hole proposal refuses.
        holes[0] = Circle(8, 8, 3, 512); holes[1] = Circle(8, 8, 1, 512);
        var nested = new MillingRegion(source.Outer, holes); var proposed = Prepare(nested, true);
        Assert.Equal(Geometry(Prepare(nested, false).ExecuteOriginalFirstAuthority(-1)),
            Geometry(proposed.ExecuteOriginalFirstAuthority(-1)));
        Assert.Equal(0, proposed.OriginalHoleErosionCount);
    }

    [Fact]
    public void NativeCoordinateRangeRefusesOnlyTheProposal()
    {
        var source = Source();
        IReadOnlyList<PointD> Translate(IReadOnlyList<PointD> ring)
            => ring.Select(p => new PointD(p.x + 34000, p.y - 34000)).ToArray();
        var translated = new MillingRegion(Translate(source.Outer), source.Holes.Select(Translate).ToArray());
        var proposed = Prepare(translated, true);
        Assert.Equal(Geometry(Prepare(translated, false).ExecuteOriginalFirstAuthority(-1)),
            Geometry(proposed.ExecuteOriginalFirstAuthority(-1)));
        Assert.Equal(0, proposed.OriginalHoleErosionCount);
    }

    [Fact]
    public void AdjacentOneTickCollinearReversalRetainsEveryOriginalFiniteWall()
    {
        var source = Source(); var holes = source.Holes.ToArray();
        double q = 1 / PathUtils.Scale;
        holes[0] = new[] { new PointD(6, 6), new(10, 6), new(10 - q, 6), new(10, 10), new(6, 10) };
        var reversed = new MillingRegion(source.Outer, holes); var proposed = Prepare(reversed, true);
        Assert.NotEmpty(proposed.ExecuteOriginalFirstAuthority(-1));
        Assert.Equal(1, proposed.OriginalHoleErosionCount);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactThreeOwnerWitnessAcceptsOnlyItsOrdinaryAdjacentEndpoint(bool reverse)
    {
        var knot = new NativeSegmentArrangement.Point(2431885, 5345167);
        var segments = Witness();
        if (reverse) segments = segments.Reverse().Select(s => new NativeSegmentArrangement.Segment(s.B, s.A)).ToArray();
        Assert.Equal(2, Junction(knot, segments, new[] { (0, 261), (0, 262), (0, 263) }));
    }

    [Fact]
    public void ThreeOwnerCrossContourOrNonadjacentContactsCannotUseTheWitnessException()
    {
        var knot = new NativeSegmentArrangement.Point(2431885, 5345167); var segments = Witness();
        Assert.Equal(0, Junction(knot, segments, new[] { (0, 261), (0, 262), (1, 263) }));
        Assert.Equal(0, Junction(knot, segments, new[] { (0, 261), (0, 262), (0, 264) }));
        // The ordinary third owner must be its actual authored endpoint, and
        // must not introduce a second collinear overlap.
        var displaced = segments.ToArray();
        displaced[2] = new(new(knot.X + 1, knot.Y), displaced[2].B);
        Assert.Equal(0, Junction(knot, displaced, new[] { (0, 261), (0, 262), (0, 263) }));
        var collinear = segments.ToArray();
        collinear[2] = new(knot, new(knot.X + 2, knot.Y - 2));
        Assert.Equal(0, Junction(knot, collinear, new[] { (0, 261), (0, 262), (0, 263) }));
    }

    [Fact]
    public void WarmFirstCancellationCannotReturnCachedAcceptance()
    {
        var prepared = Prepare(Source(), true); prepared.ExecuteOriginalFirstAuthority(-1);
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => prepared.ExecuteOriginalFirstAuthority(-1, cancellation.Token));
    }

    [Fact]
    public void IndirectCarvingPocketSlicesRetainAllCommandsUnderFirstHoleReference()
    {
        const string flag = "RW_TOOLPATHS_FIRST_HOLE_EROSION_REFERENCE";
        string? previous = Environment.GetEnvironmentVariable(flag);
        try
        {
            const int size = 33;
            var heights = Enumerable.Range(0, size * size).Select(i => {
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
            var reference = CarvingRoughingToolpaths.Generate(surface, tool, 2, options);
            Assert.NotEmpty(actual.Toolpaths);
            Assert.Equal(reference.Diagnostics, actual.Diagnostics);
            Assert.Equal(reference.Toolpaths.Count, actual.Toolpaths.Count);
            for (int i = 0; i < actual.Toolpaths.Count; i++)
            {
                var a = actual.Toolpaths[i]; var b = reference.Toolpaths[i];
                Assert.True(MemoryMarshal.AsBytes(a.Points.ToArray().AsSpan()).SequenceEqual(
                    MemoryMarshal.AsBytes(b.Points.ToArray().AsSpan())));
                Assert.Equal(b.Spans, a.Spans); Assert.Equal(b.DepthPassIndex, a.DepthPassIndex);
                Assert.Equal(b.VisitIdentity, a.VisitIdentity); Assert.Equal(b.RegionIndex, a.RegionIndex);
                Assert.Equal(b.Category, a.Category);
            }
        }
        finally { Environment.SetEnvironmentVariable(flag, previous); }
    }

    private static PolygonOps.PreparedBoundaryOffset Prepare(MillingRegion source, bool enabled)
        => PolygonOps.PrepareBoundaryOffset(source.Outer, source.Holes, .00025, 0,
            sourceConstructionTolerance: .001, sourceConstructionSweepRadius: 1,
            separateOriginalFirstConstruction: enabled);
    private static NativeSegmentArrangement.Segment[] Witness()
        => new[] {
            new NativeSegmentArrangement.Segment(new(2428914, 5348138), new(2431886, 5345166)),
            new NativeSegmentArrangement.Segment(new(2431886, 5345166), new(2431885, 5345167)),
            new NativeSegmentArrangement.Segment(new(2431885, 5345167), new(2432947, 5344366)),
        };
    private static int Junction(NativeSegmentArrangement.Point knot, NativeSegmentArrangement.Segment[] segments,
        (int Ring, int Edge)[] owners)
        => (int)typeof(PolygonOps).GetNestedType("OriginalHoleErosionSource", BindingFlags.NonPublic)!
            .GetMethod("AdjacentOverlapJunction", BindingFlags.Static | BindingFlags.NonPublic)!
            .Invoke(null, new object[] { knot, new[] { 0, 1, 2 }, segments, owners, new[] { 300, 300 } })!;
    private static MillingRegion Source()
        => new(new[] { new PointD(0, 0), new(64, 0), new(64, 64), new(0, 64) },
            Enumerable.Range(0, 32).Select(i => (IReadOnlyList<PointD>)Circle(8 + 8 * (i % 6), 8 + 8 * (i / 6), 2, 512)).ToArray());
    private static PointD[] Circle(double x, double y, double radius, int points)
        => Enumerable.Range(0, points).Select(i => new PointD(x + radius * Math.Cos(2 * Math.PI * i / points),
            y + radius * Math.Sin(2 * Math.PI * i / points))).ToArray();
    private static long[][][] Geometry(IEnumerable<IReadOnlyList<PointD>> paths)
        => PathUtils.ToClipper(paths).Select(p => p.Select(q => new[] { q.X, q.Y }).ToArray()).ToArray();
}
