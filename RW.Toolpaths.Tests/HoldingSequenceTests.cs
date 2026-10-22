using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class HoldingSequenceTests
{
    private static List<PointD> Rect(double x, double y, double w, double h) =>
        [new(x, y), new(x + w, y), new(x + w, y + h), new(x, y + h)];

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void EveryDiameterProtectsCommonFootprintsIncludingWastePlugsAndRetainedIslands(bool nestedIsland)
    {
        var stock = Rect(0, 0, 120, 100);
        var regions = new List<MillingRegion> { new(Rect(10, 10, 100, 80), [Rect(30, 30, 60, 40)]) };
        if (nestedIsland) regions.Add(new(Rect(46, 43, 28, 14)));
        var radii = new[] { 3d, 1.5, .5 };
        var members = radii.Select(r => new HoldingProfileMember(ProfileToolpaths.Prepare(regions,
            ToolGeometry.Flat(r), new ProfileOptions { Side = ProfileSide.Outside, ClipBoundary = stock }), r)).ToArray();
        var shared = HoldingTabPlanner.PlanSequence(members, stock, 4, 5, 2);
        Assert.Equal(3, shared.Count); Assert.NotEmpty(shared[0].Report.Footprints);
        var common = shared[0].Report.Footprints.SelectMany(f => f.Corners).Select(p => (p.x, p.y)).ToArray();
        for (int i = 0; i < shared.Count; i++)
        {
            Assert.Equal(common, shared[i].Report.Footprints.SelectMany(f => f.Corners).Select(p => (p.x, p.y)));
            Assert.True(shared[i].Report.UnresolvedCount == 0, $"stage={i}: {shared[i].Report.SafetyDiagnostic} {string.Join("; ", shared[i].Report.PieceCensus.TakeLast(2))}");
            Assert.Contains(shared[i].Report.PieceCensus, c => c.Contains("sequence connectivity:"));
            var paths = ProfileToolpaths.Generate(regions, ToolGeometry.Flat(radii[i]), new DepthSchedule(10.2, 2) { SurfaceZ = 10 },
                new ProfileOptions { Side = ProfileSide.Outside, ClipBoundary = stock,
                    Tabs = new() { Count = 4, Length = 5, Thickness = 2.2, ConstrainFloating = true }, SharedHoldingPlan = shared[i] });
            var protectedInterior = PolygonOps.Offset(shared[i].Report.Footprints.Select(f => f.Corners.ToList()).ToList(), -.05);
            var deepCuts = new List<List<PointD>>();
            foreach (var path in paths.Toolpaths)
            foreach (var span in path.Spans)
            for (int point = span.StartIndex; point < span.EndIndex; point++)
            {
                var a = path.Points[point]; var b = path.Points[point + 1];
                if (Math.Min(a.Z, b.Z) >= 2 - .001) continue;
                deepCuts.Add([new(a.X, a.Y), new(b.X, b.Y)]);
            }
            var cut = PolygonOps.BufferCenterlines(deepCuts, radii[i]);
            Assert.True(PolygonOps.Intersect(cut, protectedInterior).Sum(p => Math.Abs(Clipper.Area(new PathD(p)))) < .01,
                $"stage={i}: physical cutter intersects a common retained footprint");
        }
    }

    [Fact]
    public void SmallToolRecoversNarrowVoidThatLargeToolCannotEnter()
    {
        var regions = new[] { new MillingRegion(Rect(5, 5, 30, 30), [Rect(15, 12, 2, 14)]) };
        var large = ProfileToolpaths.Prepare(regions, ToolGeometry.Flat(2), new ProfileOptions { Side = ProfileSide.Outside });
        var small = ProfileToolpaths.Prepare(regions, ToolGeometry.Flat(.4), new ProfileOptions { Side = ProfileSide.Outside });
        Assert.Single(large.Rings); Assert.Equal(2, small.Rings.Count);
        var stillInaccessible = ProfileToolpaths.Prepare(regions, ToolGeometry.Flat(1.1), new ProfileOptions { Side = ProfileSide.Outside });
        Assert.Single(stillInaccessible.Rings);
    }

    [Fact]
    public void DetailChannelUsesDetailResolutionWithRoughingClearance()
    {
        var stock = Rect(0, 0, 80, 80);
        var regions = new[] { new MillingRegion(Rect(10, 10, 60, 60), [Rect(38, 20, 2.8, 20)]) };
        var members = new[] { 3d, .4 }.Select(radius => new HoldingProfileMember(
            ProfileToolpaths.Prepare(regions, ToolGeometry.Flat(radius),
                new ProfileOptions { Side = ProfileSide.Outside }), radius)).ToArray();

        var plans = HoldingTabPlanner.PlanSequence(members, stock, 1, .75, 2);

        Assert.Single(members[0].Profile.Rings);
        Assert.Equal(2, members[1].Profile.Rings.Count);
        Assert.All(plans, plan => Assert.True(plan.Report.UnresolvedCount == 0, plan.Report.SafetyDiagnostic));
        Assert.Contains(plans[1].Report.Footprints, footprint =>
            footprint.Corners.Any(point => point.x >= 38 && point.x <= 40.8 && point.y >= 20 && point.y <= 40));
    }

    [Fact]
    public void SmallRetainedIslandUsesSameDebrisThresholdDuringPlanningAndValidation()
    {
        var stock = Rect(0, 0, 100, 100);
        var regions = new[]
        {
            new MillingRegion(Rect(10, 10, 80, 80), [Rect(30, 30, 40, 40)]),
            new MillingRegion(Rect(49, 49, 1.5, 1.5))
        };
        var members = new[] { 3d, .4 }.Select(radius => new HoldingProfileMember(
            ProfileToolpaths.Prepare(regions, ToolGeometry.Flat(radius),
                new ProfileOptions { Side = ProfileSide.Outside }), radius)).ToArray();

        var plans = HoldingTabPlanner.PlanSequence(members, stock, 1, .75, 2);

        Assert.All(plans, plan => Assert.True(plan.Report.UnresolvedCount == 0, plan.Report.SafetyDiagnostic));
        Assert.Contains(plans[0].Report.Footprints, footprint =>
            PolygonOps.Intersect([footprint.Corners.ToList()], [Rect(49, 49, 1.5, 1.5)]).Count > 0);
    }

    [Fact]
    public void NeighboringChannelsKeepCommonBridgesAndImpossibleQuotasRemainDiagnostic()
    {
        var stock = Rect(0, 0, 100, 100);
        var regions = new[] { new MillingRegion(Rect(10, 10, 80, 80), [Rect(24, 25, 22, 50), Rect(49, 25, 22, 50)]) };
        var members = new[] { 3d, .5 }.Select(r => new HoldingProfileMember(ProfileToolpaths.Prepare(regions,
            ToolGeometry.Flat(r), new ProfileOptions { Side = ProfileSide.Outside }), r)).ToArray();
        var shared = HoldingTabPlanner.PlanSequence(members, stock, 4, 4, 2);
        Assert.All(shared, p => Assert.True(p.Report.UnresolvedCount == 0, p.Report.SafetyDiagnostic));
        var impossible = HoldingTabPlanner.PlanSequence(members, stock, 1000, 4, 2);
        Assert.All(impossible, p => Assert.NotNull(p.Report.SafetyDiagnostic));
    }

    [Fact]
    public void ScanlinePocketKeepsHolesAndDepthPassLimits()
    {
        var region = new MillingRegion(Rect(0, 0, 30, 20), [Rect(12, 6, 6, 8)]);
        var plan = ScanlinePocketToolpaths.Generate([region], ToolGeometry.Flat(1), new DepthSchedule(5, 2), new PocketOptions { StepOver = .4 }, 31);
        Assert.NotEmpty(plan.Toolpaths);
        Assert.Equal(3, plan.Diagnostics.DepthPassCount);
        // The perimeter carrier can spiral through all depths before raster
        // rows. Its terminal-depth tag is not a global execution-depth order.
        Assert.Equal(new int?[]{0,1,2},plan.Toolpaths.Select(p=>p.DepthPassIndex).Distinct().Order());
        Assert.All(plan.Toolpaths.SelectMany(p => p.Points), p => Assert.InRange(p.Z, -5, 0));
        foreach (var path in plan.Toolpaths)
        foreach (var span in path.Spans)
        for (int point = span.StartIndex; point < span.EndIndex; point++)
        {
            var a = path.Points[point]; var b = path.Points[point + 1];
            Assert.InRange(Math.Abs(a.Z - b.Z), 0, 2.00001);
            var midpoint = new PointD((a.X + b.X) / 2, (a.Y + b.Y) / 2);
            Assert.False(midpoint.x > 12 && midpoint.x < 18 && midpoint.y > 6 && midpoint.y < 14);
        }
    }
}
