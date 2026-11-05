using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketSmoothRoughingTests
{
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => new[] { new PointD(x0, y0), new PointD(x1, y0), new PointD(x1, y1), new PointD(x0, y1) };

    private static ToolGeometry FlatPhysical(double radius = 1.25, double length = 12)
        => new(radius)
        {
            VCarveProfile = new VCarveCutterProfile(new[]
            { new CarvingCutterSection(CarvingCutterSectionKind.Linear, radius, radius, length) }, length),
        };

    private static ToolGeometry ConePhysical(double radius = 6)
        => new(radius)
        { BottomRadius = 0, TopRadius = radius, ConeLength = radius, TipAngleRadians = Math.PI / 2 };

    private static PocketOptions SmoothOptions(double grade = 15)
        => new() { SmoothRoughing = true, SmoothRampAngleDegrees = grade,
            StepOver = .4, DetectCurves = false, CalculateSweptArea = false };

    private static VCarveRoughingAnalysis Analyze(IReadOnlyList<MillingRegion> regions,
        ToolGeometry? tool = null, DepthSchedule? depth = null, PocketOptions? options = null)
        => PocketToolpaths.AnalyzeSmoothRoughing(regions, tool ?? FlatPhysical(),
            depth ?? new DepthSchedule(3, .5), options ?? SmoothOptions(), BoostVoronoiProvider.CreateDefault());

    [Fact]
    public void IsolatedFlatToolBasinUsesItsOwnFloorWithFiniteSmoothedUncertainty()
    {
        var source = new MillingRegion(Rectangle(0, 0, 8, 8));
        var rough = Analyze(new[] { source });
        Assert.All(rough.Basins, feature =>
        {
            Assert.Equal(3, feature.OriginalPeakDepth);
            Assert.True(double.IsFinite(feature.BoundedMaximumDepth));
            Assert.True(double.IsFinite(feature.Uncertainty));
            Assert.True(feature.BoundedMaximumDepth < feature.RoughingFloor);
            Assert.True(feature.OmitRoughing, feature.Reason);
        });
        Assert.Contains(0, rough.EntireProtectedSourceRegionIndices);
        Assert.Equal(64, rough.ProtectedArea, 5);
        Assert.Empty(PolygonOps.Difference(source.AsRingGroup(), rough.ProtectedBoundary));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FlatToolNeckedBayKeepsOceanAndSharpAttachmentWhileProtectingBay(bool mirrored)
    {
        PointD[] boundary = { new(0, 0), new(30, 0), new(30, 14), new(38, 14),
            new(38, 10), new(46, 10), new(46, 20), new(38, 20), new(38, 16),
            new(30, 16), new(30, 30), new(0, 30) };
        if (mirrored) boundary = boundary.Select(point => new PointD(-point.x, point.y)).Reverse().ToArray();
        var source = new MillingRegion(boundary);
        var rough = Analyze(new[] { source });
        Assert.Contains(rough.Basins, feature => feature.OmitRoughing && Math.Abs(feature.Peak.x) > 38);
        Assert.Contains(rough.Basins, feature => !feature.OmitRoughing && Math.Abs(feature.Peak.x) < 30);
        Assert.All(rough.Basins, feature => Assert.True(double.IsFinite(feature.Uncertainty)));
        Assert.True(PolygonOps.Contains(rough.ProtectedBoundary, mirrored ? -42 : 42, 15));
        Assert.False(PolygonOps.Contains(rough.ProtectedBoundary, mirrored ? -15 : 15, 15));
        Assert.True(PolygonOps.Contains(rough.ProtectedBoundary, mirrored ? -34 : 34, 15));
        Assert.Empty(rough.EntireProtectedSourceRegionIndices);
        Assert.Empty(PolygonOps.Difference(rough.ProtectedBoundary, source.AsRingGroup()));
    }

    [Fact]
    public void OwnRampAndOwnFloorCanChangeOmissionWithoutAnyFinishingInput()
    {
        var regions = new[] { new MillingRegion(Rectangle(0, 0, 8, 8)) };
        var shallowGrade = Analyze(regions);
        var steepGrade = Analyze(regions, options: SmoothOptions(75));
        var shallowFloor = Analyze(regions, depth: new DepthSchedule(.25, .25));
        Assert.NotEmpty(shallowGrade.ProtectedBoundary);
        Assert.NotEmpty(steepGrade.ProtectedBoundary);
        Assert.NotEmpty(shallowFloor.ProtectedBoundary);
        Assert.True(steepGrade.ProtectedArea < shallowGrade.ProtectedArea);
        Assert.True(shallowFloor.ProtectedArea < shallowGrade.ProtectedArea);
        Assert.NotEqual(shallowGrade.Preparation.Fingerprint, steepGrade.Preparation.Fingerprint);
        Assert.Equal(shallowGrade.Preparation.OriginalGeometryFingerprint, steepGrade.Preparation.OriginalGeometryFingerprint);
        Assert.Equal(shallowGrade.Preparation.OriginalGeometryFingerprint, shallowFloor.Preparation.OriginalGeometryFingerprint);
    }

    [Theory]
    [InlineData(.5)]
    [InlineData(.1374)]
    public void OrdinarySmoothVCarveUsesSurfaceOriginPassesWithoutAnyUpstreamMetadata(double stepdown)
    {
        var regions = new[] { new MillingRegion(Rectangle(0, 0, 4, 4)) };
        var tool = ConePhysical(3);
        var depth = new DepthSchedule(2, stepdown) { StartDepth = 2, SkipDepthPasses = 2 };
        var options = new VCarveOptions { SmoothVCarving = true, SmoothRampAngleDegrees = 15,
            IncludeInteriorFill = false, DetectCurves = false, CalculateSweptArea = false, StepOver = .8 };
        var plan = VCarveToolpaths.Generate(regions, tool, depth, options, BoostVoronoiProvider.CreateDefault());
        Assert.NotEmpty(plan.Toolpaths);
        Assert.Contains(plan.Toolpaths, path => path.Category == "clearing" && path.DepthPassIndex == 0);
        var firstLayerCuts=plan.Toolpaths.Where(path => path.Category == "clearing" && path.DepthPassIndex == 0)
            .SelectMany(path => path.Spans.Where(s=>s.Kind is ToolpathSpanKind.Cut or ToolpathSpanKind.Ramp)
                .SelectMany(s=>path.Points.Skip(s.StartIndex).Take(s.PointCount))).ToArray();
        Assert.NotEmpty(firstLayerCuts);
        // Proved links/retraces may visit deeper stock already cut by an
        // earlier completed ring; fresh first-layer cutting retains its cap.
        Assert.All(firstLayerCuts,
            point => Assert.True(-point.Z <= stepdown + .0005,$"Fresh first-layer cut exceeds {stepdown:R}: {point}"));
        Assert.Equal(2, depth.StartDepth);
        Assert.Equal(2, depth.SkipDepthPasses);
        Assert.False(options.IncludeInteriorFill);
        Assert.Null(options.Preparation);
        Assert.Null(options.RoughingAnalysis);
    }

    [Fact]
    public void ShortOwnPhysicalToolAndThresholdUncertaintyRetainOrdinaryRoughing()
    {
        var regions = new[] { new MillingRegion(Rectangle(0, 0, 8, 8)) };
        var shortTool = Analyze(regions, FlatPhysical(1.25, 1));
        Assert.Empty(shortTool.ProtectedBoundary);
        Assert.All(shortTool.Basins, feature =>
        {
            Assert.False(feature.OmitRoughing);
            Assert.Contains("tool-too-short", feature.Reason);
            Assert.True(double.IsFinite(feature.Uncertainty));
        });
        var normal = Analyze(regions);
        double bound = normal.Basins.Max(feature => feature.BoundedMaximumDepth);
        Assert.All(normal.Preparation.AnalyzeRoughing(bound).Basins, feature => Assert.False(feature.OmitRoughing));
        Assert.All(normal.Preparation.AnalyzeRoughing(Math.BitDecrement(bound)).Basins, feature => Assert.False(feature.OmitRoughing));
    }

    [Fact]
    public void DisabledUnknownPhysicalProfileAndCanceledAnalysisCannotPublishProtection()
    {
        var regions = new[] { new MillingRegion(Rectangle(0, 0, 8, 8)) };
        Assert.Throws<ArgumentException>(() => Analyze(regions, options: new PocketOptions()));
        Assert.Throws<ArgumentException>(() => Analyze(regions, tool: ToolGeometry.Flat(1.25)));
        Assert.Throws<InvalidOperationException>(() => PocketToolpaths.AnalyzeSmoothRoughing(regions,
            FlatPhysical(), new DepthSchedule(3, .5), SmoothOptions(), new NativeMedialAxisProvider()));
        using var canceled = new CancellationTokenSource();
        canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => PocketToolpaths.AnalyzeSmoothRoughing(regions,
            FlatPhysical(), new DepthSchedule(3, .5), SmoothOptions(), BoostVoronoiProvider.CreateDefault(), canceled.Token));
    }

    [Fact]
    public void FloorReachingUnchangedOceanKeepsOrdinaryOffsetAndScanlinePaths()
    {
        var regions = new[] { new MillingRegion(Rectangle(0, 0, 12, 12)) };
        var physical = FlatPhysical(.5);
        var depth = new DepthSchedule(.5, .5);
        var options = SmoothOptions(75) with { SpiralChaining = false, Ramp = RampSettings.FromRatio(0) };
        var rough = Analyze(regions, physical, depth, options);
        Assert.Empty(rough.ProtectedBoundary);
        Assert.False(PolygonOps.Contains(rough.ProtectedBoundary, 6, 6));
        var ordinary = options with { SmoothRoughing = false };
        AssertSame(PocketToolpaths.Generate(regions, physical, depth, ordinary),
            PocketToolpaths.Generate(regions, physical, depth, options with { ProtectedBoundary = rough.ProtectedBoundary }));
        AssertSame(ScanlinePocketToolpaths.Generate(regions, physical, depth, ordinary),
            ScanlinePocketToolpaths.Generate(regions, physical, depth, options with { ProtectedBoundary = rough.ProtectedBoundary }));
        static void AssertSame(ToolpathPlan a, ToolpathPlan b)
        {
            Assert.NotEmpty(a.Toolpaths);
            Assert.NotEmpty(b.Toolpaths);
            Assert.Equal(a.Toolpaths.Count, b.Toolpaths.Count);
            for (int i = 0; i < a.Toolpaths.Count; i++)
            {
                Assert.Equal(a.Toolpaths[i].Points.ToArray(), b.Toolpaths[i].Points.ToArray());
                Assert.Equal(a.Toolpaths[i].Spans.ToArray(), b.Toolpaths[i].Spans.ToArray());
            }
        }
    }

}
