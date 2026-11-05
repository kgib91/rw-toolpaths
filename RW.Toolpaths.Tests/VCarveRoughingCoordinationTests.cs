using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveRoughingCoordinationTests
{
    [Fact]
    public void ParallelOriginalSourceSamplesKeepEverySerialRadiusAndBothDepthFields()
    {
        var ring=Enumerable.Range(0,8192).Select(i=>{
            double angle=i*Math.Tau/8192,r=20+3*Math.Sin(5*angle);
            return new PointD(r*Math.Cos(angle),r*Math.Sin(angle));
        }).ToArray();
        var regions=new[]{new MillingRegion(ring)};
        var serial=VCarveToolpaths.Prepare(regions,Tool(),new(6,1),Options with{ParallelOriginalSourceSampling=false},
            BoostVoronoiProvider.CreateDefault());
        var parallel=VCarveToolpaths.Prepare(regions,Tool(),new(6,1),Options with{ParallelOriginalSourceSampling=true},
            BoostVoronoiProvider.CreateDefault());
        Assert.Equal(serial.Fingerprint,parallel.Fingerprint);
        Assert.Equal(serial.Regions.Count,parallel.Regions.Count);
        for(int r=0;r<serial.Regions.Count;r++)
        {
            var a=serial.Regions[r];var b=parallel.Regions[r];
            Assert.True(a.OriginalMedialGraph.Count>=4096);
            Assert.Equal(a.MaximumChordLength,b.MaximumChordLength);
            Assert.Equal(a.OriginalMedialPaths.Count,b.OriginalMedialPaths.Count);
            for(int i=0;i<a.OriginalMedialPaths.Count;i++)
            {
                Assert.Equal(a.OriginalMedialPaths[i].Points,b.OriginalMedialPaths[i].Points);
                Assert.Equal(a.SourceClearancePaths[i],b.SourceClearancePaths[i]);
                Assert.Equal(a.SmoothedMedialPaths[i].Points,b.SmoothedMedialPaths[i].Points);
                Assert.Equal(a.UpperMedialPaths[i].Points,b.UpperMedialPaths[i].Points);
            }
        }
    }

    private static ToolGeometry Tool(double length = 12) => new(12)
    { BottomRadius = 0, TopRadius = length, ConeLength = length, TipAngleRadians = Math.PI / 2 };
    private static VCarveOptions Options => new()
    { SmoothVCarving = true, SmoothRampAngleDegrees = 15, DetectCurves = false, CalculateSweptArea = false,
        IncludeInteriorFill = false, StepOver = .4, Tolerance = .03 };
    private static PointD[] Rectangle(double left, double bottom, double right, double top)
        => new[] { new PointD(left, bottom), new PointD(right, bottom), new PointD(right, top), new PointD(left, top) };
    private static VCarvePreparation Prepare(IReadOnlyList<MillingRegion> regions, DepthSchedule? depth = null,
        ToolGeometry? tool = null) => VCarveToolpaths.Prepare(regions, tool ?? Tool(), depth ?? new DepthSchedule(6, 1),
            Options, BoostVoronoiProvider.CreateDefault());

    [Fact]
    public void IsolatedPuddleProtectsEntireArtworkIncludingItsAcuteCorners()
    {
        var source = new MillingRegion(Rectangle(0, 0, 8, 8));
        var result = Prepare(new[] { source }).AnalyzeRoughing(3);
        Assert.All(result.Basins, basin => Assert.True(basin.OmitRoughing, basin.Reason));
        Assert.All(result.Basins, basin => Assert.True(basin.EntireSourceRegion));
        Assert.Equal(64, result.ProtectedArea, 5);
        Assert.Empty(PolygonOps.Difference(source.AsRingGroup(), result.ProtectedBoundary));
    }

    [Fact]
    public void OceanCornerSpokesRemainInOneFloorReachingBasin()
    {
        var result = Prepare(new[] { new MillingRegion(Rectangle(0, 0, 40, 40)) }).AnalyzeRoughing(3);
        Assert.Single(result.Basins);
        Assert.False(result.Basins[0].OmitRoughing);
        Assert.Empty(result.ProtectedBoundary);
        Assert.True(result.Basins[0].SmoothedPeakDepth >= 3);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NeckedBayProtectsOriginalDiscsWithOceanOverlapPrecedence(bool mirrored)
    {
        PointD[] artwork = new[] { new PointD(0, 0), new PointD(30, 0), new PointD(30, 14),
            new PointD(38, 14), new PointD(38, 10), new PointD(46, 10), new PointD(46, 20),
            new PointD(38, 20), new PointD(38, 16), new PointD(30, 16), new PointD(30, 30), new PointD(0, 30) };
        if (mirrored) artwork = artwork.Select(point => new PointD(-point.x, point.y)).Reverse().ToArray();
        var source = new MillingRegion(artwork);
        var result = Prepare(new[] { source }).AnalyzeRoughing(3);
        Assert.Contains(result.Basins, basin => basin.OmitRoughing && !basin.EntireSourceRegion);
        Assert.Contains(result.Basins, basin => !basin.OmitRoughing);
        Assert.True(PolygonOps.Contains(result.ProtectedBoundary, mirrored ? -42 : 42, 15));
        Assert.False(PolygonOps.Contains(result.ProtectedBoundary, mirrored ? -15 : 15, 15));
        Assert.Empty(PolygonOps.Difference(result.ProtectedBoundary, source.AsRingGroup()));
    }

    [Fact]
    public void HolesAndNestedSourceIslandsRemainUncutInProtectionTopology()
    {
        var outer = new MillingRegion(Rectangle(0, 0, 12, 12), new[] { (IReadOnlyList<PointD>)Rectangle(4, 4, 8, 8).Reverse().ToArray() });
        var island = new MillingRegion(Rectangle(5, 5, 7, 7));
        var result = Prepare(new[] { outer, island }).AnalyzeRoughing(3);
        Assert.Contains(0, result.EntireProtectedSourceRegionIndices);
        Assert.Contains(1, result.EntireProtectedSourceRegionIndices);
        Assert.False(PolygonOps.Contains(result.ProtectedBoundary, 4.5, 4.5));
        Assert.True(PolygonOps.Contains(result.ProtectedBoundary, 6, 6));
        Assert.Equal(132, result.ProtectedArea, 5);
    }

    [Theory]
    [InlineData(8, false)]
    [InlineData(8, true)]
    [InlineData(30, false)]
    [InlineData(30, true)]
    [InlineData(80, false)]
    [InlineData(80, true)]
    public void FloorReachingNeckAttachmentCannotContaminateShallowTerminalBay(double length, bool mirrored)
    {
        double bay = 30 + length;
        PointD[] artwork = { new(0, 0), new(30, 0), new(30, 13), new(bay, 13), new(bay, 10),
            new(bay + 8, 10), new(bay + 8, 20), new(bay, 20), new(bay, 17), new(30, 17), new(30, 30), new(0, 30) };
        if (mirrored) artwork = artwork.Select(point => new PointD(-point.x, point.y)).Reverse().ToArray();
        var physical = new ToolGeometry(3)
        { VCarveProfile = new VCarveCutterProfile(new[]
            { new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 1.6, 1.6),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.6, 3, 20) }, 21.6) };
        var preparation = Prepare(new[] { new MillingRegion(artwork) }, tool: physical);
        var analysis = preparation.AnalyzeRoughing(3);
        var terminal = analysis.Basins.Single(feature => !feature.IsAttachment && Math.Abs(feature.Peak.x) > bay);
        Assert.True(terminal.OmitRoughing, terminal.Reason);
        Assert.True(terminal.BoundedMaximumDepth < 3);
        Assert.Contains(analysis.Basins, feature => feature.IsAttachment && !feature.OmitRoughing);
        Assert.True(PolygonOps.Contains(analysis.ProtectedBoundary, mirrored ? -(bay + 4) : bay + 4, 15));
        double neckX = 30 + length / 2;
        Assert.False(PolygonOps.Contains(analysis.ProtectedBoundary, mirrored ? -neckX : neckX, 15));
        Assert.Contains(analysis.Basins, feature => !feature.IsAttachment && Math.Abs(feature.Peak.x) < 30 && !feature.OmitRoughing);
    }

    [Fact]
    public void EntireSourceProtectionIncludesTheNeckWhenAllBasinsAndAttachmentsStayBelowFloor()
    {
        var source = new MillingRegion(new PointD[] { new(0, 0), new(8, 0), new(8, 3), new(12, 3),
            new(12, 0), new(20, 0), new(20, 8), new(12, 8), new(12, 5), new(8, 5), new(8, 8), new(0, 8) });
        var analysis = Prepare(new[] { source }).AnalyzeRoughing(3);
        Assert.Contains(analysis.Basins, feature => feature.IsAttachment);
        Assert.All(analysis.Basins, feature => Assert.True(feature.OmitRoughing, feature.Reason));
        Assert.Contains(0, analysis.EntireProtectedSourceRegionIndices);
        Assert.Empty(PolygonOps.Difference(source.AsRingGroup(), analysis.ProtectedBoundary));
        Assert.Equal(136, analysis.ProtectedArea, 5);
    }

    [Fact]
    public void EqualityAndUncertaintyRetainRoughingAtTheNumericalThreshold()
    {
        var prepared = Prepare(new[] { new MillingRegion(Rectangle(0, 0, 8, 8)) });
        var initial = prepared.AnalyzeRoughing(3);
        double bound = initial.Basins.Max(basin => basin.BoundedMaximumDepth);
        Assert.All(prepared.AnalyzeRoughing(bound).Basins, basin => Assert.False(basin.OmitRoughing));
        Assert.All(prepared.AnalyzeRoughing(Math.BitDecrement(bound)).Basins, basin => Assert.False(basin.OmitRoughing));
        Assert.All(prepared.AnalyzeRoughing(Math.BitIncrement(bound)).Basins, basin => Assert.True(basin.OmitRoughing));
        Assert.All(prepared.AnalyzeRoughing(initial.Basins.Max(basin => basin.SmoothedPeakDepth) + .001).Basins,
            basin => Assert.False(basin.OmitRoughing));
    }

    [Theory]
    [InlineData(true, "authored-depth-below-roughing-floor")]
    [InlineData(false, "physical-tool-too-short-for-roughing-floor")]
    public void IncompatibleAuthoredDepthOrToolLengthCannotCauseRoughingOmission(bool authoredDepth, string reason)
    {
        var result = Prepare(new[] { new MillingRegion(Rectangle(0, 0, 8, 8)) },
            authoredDepth ? new DepthSchedule(2, 1) : new DepthSchedule(6, 1),
            authoredDepth ? Tool() : Tool(2)).AnalyzeRoughing(3);
        Assert.All(result.Basins, basin => { Assert.False(basin.OmitRoughing); Assert.Equal(reason, basin.Reason); });
        Assert.Empty(result.ProtectedBoundary);
    }

    [Fact]
    public void MonotonicallyWideningOceanAttachmentDoesNotInventAnIndependentBay()
    {
        var source = new MillingRegion(new[] { new PointD(0, 0), new PointD(30, 0), new PointD(30, 10),
            new PointD(40, 14), new PointD(40, 16), new PointD(30, 20), new PointD(30, 30), new PointD(0, 30) });
        var result = Prepare(new[] { source }).AnalyzeRoughing(3);
        Assert.Single(result.Basins);
        Assert.Empty(result.ProtectedBoundary);
    }

    [Fact]
    public void PreparedAnalysisIsReusedWithoutProviderCallsAndPreservesAuthoredSettings()
    {
        var region = new MillingRegion(Rectangle(0, 0, 6, 6));
        var depth = new DepthSchedule(3, .5) { StartDepth = 3, SkipDepthPasses = 2 };
        var tool = Tool(); var options = Options;
        var preparation = VCarveToolpaths.Prepare(new[] { region }, tool, depth, options, BoostVoronoiProvider.CreateDefault());
        var analysis = preparation.AnalyzeRoughing(3);
        var plan = VCarveToolpaths.Generate(new[] { region }, tool, depth,
            options with { Preparation = preparation, RoughingAnalysis = analysis }, new RejectProvider());
        Assert.NotEmpty(plan.Toolpaths);
        Assert.Contains(plan.Toolpaths, path => path.Category == "clearing" && path.DepthPassIndex == 0);
        Assert.True(plan.Toolpaths.Where(path => path.Category == "clearing" && path.DepthPassIndex == 0)
            .SelectMany(path => path.Spans.Count == 0 ? path.Points : path.Spans.Where(span => span.Kind == ToolpathSpanKind.Cut)
                .SelectMany(span => path.Points.Skip(span.StartIndex).Take(span.PointCount))).All(point => -point.Z <= .501));
        Assert.Equal(3, depth.StartDepth); Assert.Equal(2, depth.SkipDepthPasses); Assert.False(options.IncludeInteriorFill);
        Assert.Throws<InvalidOperationException>(() => VCarveToolpaths.Generate(new[] { region }, tool,
            depth with { DepthPerPass = .75 }, options with { Preparation = preparation, RoughingAnalysis = analysis }, new RejectProvider()));
    }

    [Fact]
    public void DisabledSmoothingAndCanceledPreparationCannotSupplyCoordination()
    {
        var region = new MillingRegion(Rectangle(0, 0, 8, 8));
        Assert.Throws<ArgumentException>(() => VCarveToolpaths.Prepare(new[] { region }, Tool(), new DepthSchedule(6, 1),
            Options with { SmoothVCarving = false }, new RejectProvider()));
        Assert.Throws<InvalidOperationException>(() => VCarveToolpaths.Prepare(new[] { region }, Tool(), new DepthSchedule(6, 1),
            Options, NativeMedialAxisProvider.CreateDefault()));
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveToolpaths.Prepare(new[] { region }, Tool(), new DepthSchedule(6, 1),
            Options, new RejectProvider(), cancellation.Token));
    }

    [Fact]
    public void EveryLateCorrectionAndTerminalVisitGetsCommandableSurfaceStepdowns()
    {
        const double stepdown = .1374;
        var correction = new TaggedToolpath(new[] { new Point3D(0, 0, -.83), new Point3D(10, 0, -.83) },
            Array.Empty<ToolpathSpan>(), 0, "clearing", 99, detectCurves: false);
        var terminal = new TaggedToolpath(new[] { new Point3D(3, 0, -.7), new Point3D(7, 0, -.9) },
            Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
        var schedule = new DepthSchedule(1, stepdown) { SurfaceZ = 12.7 };
        var result = VCarveToolpaths.ScheduleFreshStockVisits(new[] { correction, terminal }, schedule, default, null);
        double Command(double depth) => -double.Parse(((float)(schedule.SurfaceZ - depth) - (float)schedule.SurfaceZ)
            .ToString("F3", System.Globalization.CultureInfo.InvariantCulture), System.Globalization.CultureInfo.InvariantCulture);
        foreach (var original in new[] { correction, terminal })
        {
            var visits = result.Where(path => path.Points[0].X == original.Points[0].X).ToArray();
            for (int point = 0; point < original.Points.Count; point++)
            {
                double previous = 0;
                foreach (var visit in visits)
                {
                    Assert.Equal(original.Points[point].X, visit.Points[point].X);
                    Assert.Equal(original.Points[point].Y, visit.Points[point].Y);
                    double commanded = Command(-visit.Points[point].Z);
                    Assert.True(commanded - previous <= stepdown + 1e-10,
                        $"Fresh command increment {commanded - previous:R} exceeds assigned stepdown {stepdown:R}.");
                    previous = commanded;
                }
                Assert.Equal(Command(-original.Points[point].Z), previous);
            }
        }
        Assert.Contains(result, path => path.Category == "final-carve" && path.Points.SequenceEqual(terminal.Points));
        Assert.Throws<InvalidOperationException>(() => VCarveToolpaths.ScheduleFreshStockVisits(new[] { correction },
            new DepthSchedule(1, .0009), default, null));
    }

    [Fact]
    public void UnchangedShallowRoundedTipFinishIsBoundedByPhysicalRemovedVolume()
    {
        var rounded = new ToolGeometry(3)
        { VCarveProfile = new VCarveCutterProfile(new[]
            { new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 1.6, 1.6),
                new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.6, 3, 20) }, 21.6) };
        var region = new MillingRegion(Rectangle(0, 0, .1, .1));
        var preparation = Prepare(new[] { region }, tool: rounded);
        var result = preparation.AnalyzeRoughing(3);
        Assert.All(result.Basins, basin => Assert.True(basin.OmitRoughing));
        Assert.Equal(0, preparation.MaximumPreservedFinishDepth(0, .05, .05));
        // Sub-command-resolution removal cannot authorize cleanup by consuming
        // the consumer's .001mm comparison allowance.
        preparation.BindCompletedFinish(new[] { (new Point3D(.05, .05, -.0008), new Point3D(.05, .05, -.0008)) });
        Assert.Equal(0, preparation.MaximumPreservedFinishDepth(0, .05, .05));
        Assert.Equal(0, preparation.MaximumPreservedFinishDepth(0, 0, 0));
        Assert.True(preparation.PreservedFinishDepthLipschitz >= 1);
    }

    private sealed class RejectProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle, bool useBigIntegers)
            => throw new InvalidOperationException("Completed preparation must be reused instead of calling the native provider again.");
    }
}
