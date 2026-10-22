using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class SkippedDepthPassTests
{
    private static readonly ToolGeometry Cone = new(4)
    {
        BottomRadius = 0, TopRadius = 4, ConeLength = 8, TipAngleRadians = 2 * Math.Atan(.5),
    };

    [Fact]
    public void ScheduleSkipsNominalLayersAndEntersFirstRetainedLayerFromRealSurface()
    {
        var depth = new DepthSchedule(9, 2) { SurfaceZ = 11, SkipDepthPasses = 2 };

        Assert.Equal(5, depth.NominalPassCount);
        Assert.Equal(2, depth.FirstPassIndex);
        Assert.Equal(3, depth.PassCount);
        Assert.Equal(new[] { 5d, 3, 2 }, Enumerable.Range(0, depth.PassCount).Select(depth.PassBottomZ));
        Assert.Equal(new[] { 11d, 5, 3 }, Enumerable.Range(0, depth.PassCount).Select(depth.PassTopZ));
    }

    [Fact]
    public void ShortenedLocalDepthRetainsFinalTargetAndNegativeCountFails()
    {
        var local = new DepthSchedule(1.5, 1) { SurfaceZ = 10, SkipDepthPasses = 8 };

        Assert.Equal(1, local.FirstPassIndex);
        Assert.Equal(1, local.PassCount);
        Assert.Equal(10, local.PassTopZ(0));
        Assert.Equal(8.5, local.PassBottomZ(0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new DepthSchedule(4, 1) { SkipDepthPasses = -1 });
    }

    [Fact]
    public void VCarveSkipTwoStartsAtThirdLayerAndPreservesRemainingPlanarCutGeometry()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [Rectangle(0, 0, 20, 16)];
        List<TaggedToolpath> Clearing(int skip) => MedialAxisToolpaths.GenerateClearingPassesTagged(
            boundary, 0, 3.5, Cone.TipAngleRadians!.Value, 1,
            bottomRadiusOverride: Cone.BottomRadius, topRadiusOverride: Cone.TopRadius,
            coneLengthOverride: Cone.ConeLength, includeInteriorFill: false,
            detectCurves: false, edgeFillWidth: 2.1, skipDepthPasses: skip);
        var ordinary = Clearing(0);
        var skipped = Clearing(2);

        Assert.Equal(new[] { 2, 3 }, skipped.Select(path => path.DepthPassIndex!.Value).Distinct());
        Assert.Equal(new[] { -3.5, -3d }, skipped.SelectMany(path => path.Points).Select(point => point.Z).Distinct().Order());
        Assert.Equal(Geometry(ordinary.Where(path => path.DepthPassIndex >= 2)), Geometry(skipped));
    }

    [Fact]
    public void PublicVCarveStrategyRampsFromSurfaceStraightToRetainedLayerAndFinishesPartialBottom()
    {
        var plan = VCarveToolpaths.Generate([new MillingRegion(Rectangle(0, 0, 20, 16))], Cone,
            new DepthSchedule(3.5, 1) { SurfaceZ = 10, SkipDepthPasses = 2 },
            new VCarveOptions
            {
                IncludeInteriorFill = false, CalculateSweptArea = false, DetectCurves = false,
                Ramp = RampSettings.FromRatio(4),
            }, new EmptyProvider());

        Assert.Equal(2, plan.Diagnostics.DepthPassCount);
        Assert.Equal(new[] { 2, 3 }, plan.Toolpaths.Select(path => path.DepthPassIndex!.Value).Distinct());
        Assert.Equal(10, plan.Toolpaths[0].Points[0].Z);
        Assert.Equal(6.5, plan.Toolpaths.SelectMany(path => path.Points).Min(point => point.Z));
        Assert.All(plan.Toolpaths, path =>
        {
            var ramp = Assert.Single(path.Spans.Where(span => span.Kind == ToolpathSpanKind.Ramp));
            Assert.Equal(10, path.Points[ramp.StartIndex].Z);
            double target = path.DepthPassIndex == 2 ? 7 : 6.5;
            Assert.Equal(target, path.Points[ramp.EndIndex].Z, 10);
            AssertRequestedRamp(path.Points, ramp, .25);
            var cut = Assert.Single(path.Spans.Where(span => span.Kind == ToolpathSpanKind.Cut));
            Assert.All(path.Points.Skip(cut.StartIndex), point => Assert.Equal(target, point.Z));
        });
    }

    [Fact]
    public void PhysicallyTruncatedVCarveDepthRetainsItsFinalLayerEvenWhenAllNominalLayersWereSkipped()
    {
        var shortTool = Cone with { ConeLength = 2, TopRadius = 1 };
        var plan = VCarveToolpaths.Generate([new MillingRegion(Rectangle(0, 0, 10, 10))], shortTool,
            new DepthSchedule(7, 1) { SkipDepthPasses = 6 },
            new VCarveOptions { IncludeInteriorFill = false, CalculateSweptArea = false, DetectCurves = false },
            new EmptyProvider());

        Assert.Equal(1, plan.Diagnostics.DepthPassCount);
        Assert.NotEmpty(plan.Toolpaths);
        Assert.All(plan.Toolpaths, path => Assert.Equal(1, path.DepthPassIndex));
        Assert.Equal(-2, plan.Toolpaths.SelectMany(path => path.Points).Min(point => point.Z));
        Assert.Equal(0, plan.Toolpaths[0].Points[0].Z);
    }

    [Fact]
    public void DeepStartingVariableFinalDetailRampsToLocalEnvelopeAndKeepsEveryOriginalCutChord()
    {
        Point3D[] source = [new(2, 2, 7), new(3, 2, 9.5), new(6, 2, 7.5)];
        var detail = new TaggedToolpath(source, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
        var routed = VCarvePathRouter.Route([detail], [Rectangle(0, 0, 20, 16)], Cone,
            new DepthSchedule(4, 1) { SurfaceZ = 10, SkipDepthPasses = 2 },
            new VCarveOptions { IncludeInteriorFill = false, Ramp = RampSettings.FromRatio(4), DetectCurves = false },
            0, null);
        var entered = Assert.Single(routed);
        var ramp = Assert.Single(entered.Spans.Where(span => span.Kind == ToolpathSpanKind.Ramp));

        Assert.Equal(10, entered.Points[0].Z);
        Assert.Equal(source, entered.Points.TakeLast(source.Length));
        AssertRequestedRamp(entered.Points, ramp, .25);
        Assert.All(entered.Points.Take(ramp.EndIndex + 1), point =>
        {
            double target = point.X <= 3
                ? 7 + (point.X - 2) * 2.5
                : 9.5 - (point.X - 3) * 2 / 3;
            Assert.True(point.Z >= target - 1e-9, $"Ramp undercut local detail at {point}: target Z={target:R}");
        });
        Assert.True(entered.Points[ramp.EndIndex].Z > 7,
            "Entry should contact reachable shallow detail rather than force it to the nominal tier.");
        Assert.Equal(7, entered.Points.Min(point => point.Z));
        Assert.Null(entered.DepthPassIndex);
        Assert.Equal("final-carve", entered.Category);
    }

    [Fact]
    public void ShallowFinalDetailSurvivesSkipCountBeyondItsLocalDepth()
    {
        Point3D[] source = [new(2, 2, -.25), new(3, 2, -.5)];
        var entered = RampPlanner.PlanPathEntry(source, 0, RampSettings.FromRatio(2));

        Assert.Equal(source, entered.Points.TakeLast(source.Length));
        Assert.InRange(entered.Points.Min(point => point.Z), -.5, -.5);
        AssertRequestedRamp(entered.Points, entered.Spans[0], .5);
    }

    [Fact]
    public void TinyEntryLoopHonorsRatioBeyondDefaultLapCap()
    {
        Point3D[] source = [new(0, 0, -3), new(.01, 0, -3), new(.01, .01, -3), new(0, .01, -3), new(0, 0, -3)];
        var entered = RampPlanner.PlanPathEntry(source, 0, RampSettings.FromRatio(4));

        Assert.True(entered.Laps > RampSettings.FromRatio(4).MaxLaps);
        RampEntryStockTests.AssertFullDirectedSourceChords(source, entered);
        var ramp = entered.Spans[0];
        double run = entered.Points.Take(ramp.EndIndex + 1).Zip(entered.Points.Skip(1), (a, b) =>
            Math.Sqrt(Math.Pow(b.X - a.X, 2) + Math.Pow(b.Y - a.Y, 2))).Sum();
        Assert.True(run >= 12 - 1e-8, $"Required physical descent was shortened: {run:R} mm.");
        AssertRequestedRamp(entered.Points, entered.Spans[0], .25);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PocketAndProfileFirstRetainedPassRampFromSurface(bool profile)
    {
        var regions = new[] { new MillingRegion(Rectangle(0, 0, 30, 20)) };
        var depth = new DepthSchedule(3.5, 1) { SurfaceZ = 10, SkipDepthPasses = 2 };
        var ramp = RampSettings.FromRatio(4);
        var plan = profile
            ? ProfileToolpaths.Generate(regions, ToolGeometry.Flat(1), depth, new ProfileOptions { Ramp = ramp })
            : PocketToolpaths.Generate(regions, ToolGeometry.Flat(1), depth, new PocketOptions { Ramp = ramp, DetectCurves = false });

        Assert.Equal(2, plan.Diagnostics.DepthPassCount);
        Assert.Equal(10, plan.Toolpaths[0].Points[0].Z);
        var entry = plan.Toolpaths[0].Spans.First(span => span.Kind == ToolpathSpanKind.Ramp);
        Assert.Equal(7, plan.Toolpaths[0].Points[entry.EndIndex].Z, 9);
        AssertRequestedRamp(plan.Toolpaths[0].Points, entry, .25);
        Assert.Equal(6.5, plan.Toolpaths.SelectMany(path => path.Points).Min(point => point.Z));
    }

    [Fact]
    public void ZeroSkipPreservesExistingVCarveGeometryAndMetadata()
    {
        ToolpathPlan Generate(DepthSchedule depth) => VCarveToolpaths.Generate(
            [new MillingRegion(Rectangle(0, 0, 20, 16))], Cone, depth,
            new VCarveOptions { IncludeInteriorFill = false, CalculateSweptArea = false, DetectCurves = false },
            new EmptyProvider());
        var ordinary = Generate(new DepthSchedule(3.5, 1));
        var explicitZero = Generate(new DepthSchedule(3.5, 1) { SkipDepthPasses = 0 });

        Assert.Equal(Geometry(ordinary.Toolpaths), Geometry(explicitZero.Toolpaths));
        Assert.All(explicitZero.Toolpaths, path => Assert.DoesNotContain(path.Spans, span => span.Kind == ToolpathSpanKind.Ramp));
    }

    [Fact]
    public void ImpossibleHorizontalEntryAndCancellationFailExplicitly()
    {
        Assert.Throws<InvalidOperationException>(() => RampPlanner.PlanPathEntry(
            [new(0, 0, -1), new(0, 0, -2)], 0, RampSettings.FromRatio(2)));
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => RampPlanner.PlanPathEntry(
            [new(0, 0, -1), new(1, 0, -1)], 0, RampSettings.FromRatio(2), cancelled.Token));
    }

    private static void AssertRequestedRamp(IReadOnlyList<Point3D> points, ToolpathSpan ramp, double slope)
    {
        for (int i = ramp.StartIndex + 1; i <= ramp.EndIndex; i++)
        {
            Point3D a = points[i - 1], b = points[i];
            double distance = Math.Sqrt(Math.Pow(b.X - a.X, 2) + Math.Pow(b.Y - a.Y, 2));
            Assert.True(distance > 1e-12, $"Ramp contains a vertical plunge: {a} -> {b}");
            Assert.InRange(a.Z - b.Z, -1e-9, slope * distance + 1e-8);
        }
    }

    private static PointD[] Rectangle(double left, double bottom, double right, double top)
        => [new(left, bottom), new(right, bottom), new(right, top), new(left, top)];

    private static string[] Geometry(IEnumerable<TaggedToolpath> paths) => paths.Select(path =>
        $"{path.RegionIndex}:{path.Category}:{path.DepthPassIndex}:" +
        string.Join(";", path.Points.Select(point => $"{point.X:R},{point.Y:R},{point.Z:R}"))).Order().ToArray();

    private sealed class EmptyProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle = 3 * Math.PI / 4, bool useBigIntegers = true) => [];
    }
}
