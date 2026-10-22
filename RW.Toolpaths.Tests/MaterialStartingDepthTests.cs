using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class MaterialStartingDepthTests
{
    private static readonly ToolGeometry Cone = new(4)
    {
        BottomRadius = 0, TopRadius = 4, ConeLength = 8, TipAngleRadians = 2 * Math.Atan(.5),
    };

    [Fact]
    public void DecimalStartingFloorAnchorsRemainingPassesWithoutMovingPhysicalStockSurface()
    {
        var depth = new DepthSchedule(4.7, .8) { SurfaceZ = 11, StartDepth = 2.3 };

        Assert.Equal(11, depth.SurfaceZ);
        Assert.Equal(3, depth.PassCount);
        Assert.Equal(8.7, depth.PassTopZ(0), 10);
        Assert.Equal(7.9, depth.PassBottomZ(0), 10);
        Assert.Equal(7.1, depth.PassBottomZ(1), 10);
        Assert.Equal(6.3, depth.PassBottomZ(2), 10);
        Assert.Equal(depth.PassBottomZ(0), depth.PassTopZ(1));
    }

    [Fact]
    public void StartingAtFinalDepthKeepsOneFloorFinishingPass()
    {
        var depth = new DepthSchedule(2.5, 1) { SurfaceZ = 10, StartDepth = 2.5 };

        Assert.Equal(1, depth.NominalPassCount);
        Assert.Equal(1, depth.PassCount);
        Assert.Equal(7.5, depth.PassTopZ(0));
        Assert.Equal(7.5, depth.PassBottomZ(0));
    }

    [Fact]
    public void LegacySkipCanFilterFloorRelativeScheduleWithoutChangingEntryFloor()
    {
        var depth = new DepthSchedule(4.5, 1) { SurfaceZ = 10, StartDepth = 1.25, SkipDepthPasses = 1 };

        Assert.Equal(4, depth.NominalPassCount);
        Assert.Equal(3, depth.PassCount);
        Assert.Equal(8.75, depth.PassTopZ(0));
        Assert.Equal(new[] { 6.75, 5.75, 5.5 }, Enumerable.Range(0, depth.PassCount).Select(depth.PassBottomZ));
    }

    [Fact]
    public void InvalidMaterialStartingFloorsFailExplicitly()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new DepthSchedule(4, 1) { StartDepth = -.1 });
        Assert.Throws<ArgumentOutOfRangeException>(() => new DepthSchedule(4, 1) { StartDepth = 4.1 });
        Assert.Throws<ArgumentOutOfRangeException>(() => new DepthSchedule(4, 1) { StartDepth = double.NaN });
    }

    [Fact]
    public void VCarveClearingBelowFloorStillUsesAbsoluteStockDepthForCutterRadius()
    {
        var paths = MedialAxisToolpaths.GenerateClearingPassesTagged(
            [Rectangle(0, 0, 20, 16)], 0, 3.75, Cone.TipAngleRadians!.Value, 1,
            bottomRadiusOverride: Cone.BottomRadius, topRadiusOverride: Cone.TopRadius,
            coneLengthOverride: Cone.ConeLength, includeInteriorFill: false,
            detectCurves: false, startingDepth: 1.25);

        Assert.Equal(new[] { -3.75, -3.25, -2.25 }, paths.SelectMany(path => path.Points).Select(point => point.Z).Distinct().Order());
        foreach (var layer in paths.GroupBy(path => path.DepthPassIndex))
        {
            double absoluteDepth = -layer.First().Points[0].Z;
            double expectedRadius = absoluteDepth * .5;
            // Existing cutter inset reserves micron-scale clearance for F3 XY
            // commands. The radius must still use the full original-stock depth.
            Assert.InRange(layer.SelectMany(path => path.Points).Min(point => point.X), expectedRadius, expectedRadius + .0011);
        }
    }

    [Fact]
    public void PublicVCarveRampsFromKnownFloorToNextPassWithOriginalSurfaceCoordinates()
    {
        var plan = VCarveToolpaths.Generate([new MillingRegion(Rectangle(0, 0, 20, 16))], Cone,
            new DepthSchedule(3.75, 1) { SurfaceZ = 12.7, StartDepth = 1.25 },
            new VCarveOptions
            {
                IncludeInteriorFill = false, CalculateSweptArea = false, DetectCurves = false,
                Ramp = RampSettings.FromRatio(4),
            }, new EmptyProvider());

        Assert.Equal(3, plan.Diagnostics.DepthPassCount);
        Assert.Equal(11.45, plan.Toolpaths[0].Points[0].Z, 10);
        Assert.Equal(8.95, plan.Toolpaths.SelectMany(path => path.Points).Min(point => point.Z), 10);
        Assert.All(plan.Toolpaths, path =>
        {
            var entry = Assert.Single(path.Spans.Where(span => span.Kind == ToolpathSpanKind.Ramp));
            Assert.Equal(11.45, path.Points[entry.StartIndex].Z, 10);
            double actualDepth = path.DepthPassIndex switch { 0 => 2.25, 1 => 3.25, _ => 3.75 };
            Assert.Equal(12.7 - actualDepth, path.Points[entry.EndIndex].Z, 10);
            AssertRampAngle(path.Points, entry, .25);
            double expectedRadius = actualDepth * .5;
            Assert.InRange(path.Points.Min(point => point.X), expectedRadius, expectedRadius + .0011);
        });
    }

    [Fact]
    public void EqualFloorVCarveRetainsFinishingGeometryWithoutUnnecessaryDescent()
    {
        var plan = VCarveToolpaths.Generate([new MillingRegion(Rectangle(0, 0, 20, 16))], Cone,
            new DepthSchedule(2.5, 1) { SurfaceZ = 10, StartDepth = 2.5 },
            new VCarveOptions { IncludeInteriorFill = false, CalculateSweptArea = false, DetectCurves = false },
            new EmptyProvider());

        Assert.Equal(1, plan.Diagnostics.DepthPassCount);
        Assert.NotEmpty(plan.Toolpaths);
        Assert.All(plan.Toolpaths, path =>
        {
            Assert.All(path.Points, point => Assert.Equal(7.5, point.Z));
            Assert.DoesNotContain(path.Spans, span => span.Kind == ToolpathSpanKind.Ramp);
        });
    }

    [Fact]
    public void FinalDetailWithNoClearingRampsFromFloorAndKeepsShallowCornerAboveIt()
    {
        Point3D[] source = [new(2, 2, 7.5), new(3, 2, 10.75), new(6, 2, 8)];
        var detail = new TaggedToolpath(source, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
        var entered = Assert.Single(VCarvePathRouter.Route([detail], [Rectangle(0, 0, 20, 16)], Cone,
            new DepthSchedule(4, 1) { SurfaceZ = 11, StartDepth = 2.5 },
            new VCarveOptions { IncludeInteriorFill = false, Ramp = RampSettings.FromRatio(4), DetectCurves = false },
            0, null));
        var ramp = Assert.Single(entered.Spans.Where(span => span.Kind == ToolpathSpanKind.Ramp));

        Assert.Equal(8.5, entered.Points[0].Z);
        Assert.Equal(source, entered.Points.TakeLast(source.Length));
        Assert.Contains(source[1], entered.Points);
        Assert.Equal(7.5, entered.Points.Min(point => point.Z));
        AssertRampAngle(entered.Points, ramp, .25);
        Assert.All(entered.Points.Take(ramp.EndIndex + 1), point =>
        {
            double target = point.X <= 3 ? 7.5 + (point.X - 2) * 3.25
                : 10.75 - (point.X - 3) * 2.75 / 3;
            Assert.True(point.Z >= target - 1e-9, $"Floor entry undercuts shallow target at {point}");
        });
    }

    [Fact]
    public void DetailBeginningAboveClearedFloorKeepsItsOriginalStartHeight()
    {
        Point3D[] source = [new(2, 2, 10.75), new(3, 2, 7.5)];
        var detail = new TaggedToolpath(source, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
        var entered = Assert.Single(VCarvePathRouter.Route([detail], [Rectangle(0, 0, 20, 16)], Cone,
            new DepthSchedule(4, 1) { SurfaceZ = 11, StartDepth = 2.5 },
            new VCarveOptions { IncludeInteriorFill = false, DetectCurves = false }, 0, null));

        Assert.Equal(source, entered.Points);
        Assert.Equal(10.75, entered.Points[0].Z);
        Assert.DoesNotContain(entered.Spans, span => span.Kind == ToolpathSpanKind.Ramp);
    }

    [Fact]
    public void PhysicalDepthTruncationRetainsFinalTargetAndClampsEntryAboveIt()
    {
        var shortTool = Cone with { ConeLength = 2, TopRadius = 1 };
        var plan = VCarveToolpaths.Generate([new MillingRegion(Rectangle(0, 0, 10, 10))], shortTool,
            new DepthSchedule(4, 1) { SurfaceZ = 10, StartDepth = 4 },
            new VCarveOptions { IncludeInteriorFill = false, CalculateSweptArea = false, DetectCurves = false },
            new EmptyProvider());

        Assert.Equal(1, plan.Diagnostics.DepthPassCount);
        Assert.NotEmpty(plan.Toolpaths);
        Assert.All(plan.Toolpaths.SelectMany(path => path.Points), point => Assert.Equal(8, point.Z));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PocketAndProfileStartAtKnownFloorAndRampToFirstDeeperLayer(bool profile)
    {
        var regions = new[] { new MillingRegion(Rectangle(0, 0, 30, 20)) };
        var depth = new DepthSchedule(4, 1) { SurfaceZ = 11, StartDepth = 1.25 };
        var settings = RampSettings.FromRatio(4);
        var plan = profile
            ? ProfileToolpaths.Generate(regions, ToolGeometry.Flat(1), depth, new ProfileOptions { Ramp = settings })
            : PocketToolpaths.Generate(regions, ToolGeometry.Flat(1), depth, new PocketOptions { Ramp = settings, DetectCurves = false });
        var entry = plan.Toolpaths[0].Spans.First(span => span.Kind == ToolpathSpanKind.Ramp);

        Assert.Equal(3, plan.Diagnostics.DepthPassCount);
        Assert.Equal(9.75, plan.Toolpaths[0].Points[entry.StartIndex].Z);
        Assert.Equal(8.75, plan.Toolpaths[0].Points[entry.EndIndex].Z, 9);
        AssertRampAngle(plan.Toolpaths[0].Points, entry, .25);
        Assert.Equal(7, plan.Toolpaths.SelectMany(path => path.Points).Min(point => point.Z));
    }

    private static void AssertRampAngle(IReadOnlyList<Point3D> points, ToolpathSpan ramp, double slope)
    {
        for (int i = ramp.StartIndex + 1; i <= ramp.EndIndex; i++)
        {
            Point3D a = points[i - 1], b = points[i];
            double distance = Math.Sqrt(Math.Pow(b.X - a.X, 2) + Math.Pow(b.Y - a.Y, 2));
            Assert.True(distance > 1e-12, $"Entry descends vertically below cleared floor: {a} -> {b}");
            Assert.InRange(a.Z - b.Z, -1e-9, slope * distance + 1e-8);
        }
    }

    private static PointD[] Rectangle(double left, double bottom, double right, double top)
        => [new(left, bottom), new(right, bottom), new(right, top), new(left, top)];

    private sealed class EmptyProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle = 3 * Math.PI / 4, bool useBigIntegers = true) => [];
    }
}
