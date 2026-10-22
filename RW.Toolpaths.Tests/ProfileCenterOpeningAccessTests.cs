using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ProfileCenterOpeningAccessTests
{
    [Fact]
    public void DefaultCenterRetainsMicroscopicComponentsAndLongThinSlits()
    {
        var regions = Regions();
        var prepared = ProfileToolpaths.Prepare(regions, ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.OnLine, MinimumOpeningRadius = .05 });
        Assert.Equal(3, prepared.Rings.Count);
        Assert.Equal(new[] { 0, 1, 2 }, prepared.Rings.Select(r => r.RegionIndex).Order().ToArray());
    }

    [Fact]
    public void CenterAccessSkipsOnlyWholeInaccessibleOriginalComponentsAndKeepsTheirIdentitySpace()
    {
        var prepared = ProfileToolpaths.Prepare(Regions(), ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.OnLine, SkipRegionsWithoutToolAccess = true,
                MinimumOpeningRadius = .05 });
        var ring = Assert.Single(prepared.Rings);
        Assert.Equal(2, ring.RegionIndex);
        Assert.InRange(ring.Points.Max(p => p.x) - ring.Points.Min(p => p.x), .1499, .1501);
        Assert.Equal(3, prepared.SourceComponents.Count);
        Assert.Equal(3, prepared.SourceMaterial.Count);
        Assert.Equal(new[] { 0, 1, 2 }, prepared.SourceComponents.Select(c => c.RegionIndex).ToArray());
    }

    [Fact]
    public void APointedMinimumFootprintRetainsAnIndependentlyFeasibleTinyComponent()
    {
        var smallFootprint = ProfileToolpaths.Prepare(Regions(), ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.OnLine, SkipRegionsWithoutToolAccess = true,
                MinimumOpeningRadius = .002 });
        var largeFootprint = ProfileToolpaths.Prepare(Regions(), ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.OnLine, SkipRegionsWithoutToolAccess = true,
                MinimumOpeningRadius = .05 });
        var tiny = Assert.Single(smallFootprint.Rings, r => r.RegionIndex == 0);
        Assert.InRange(tiny.Points.Max(p => p.x) - tiny.Points.Min(p => p.x), .0059, .0061);
        Assert.DoesNotContain(largeFootprint.Rings, r => r.RegionIndex == 0);
        Assert.Equal(3, smallFootprint.SourceComponents.Count);
    }

    [Fact]
    public void AFeasibleRoomRetainsItsThinNeckAndSharpAuthoredPoint()
    {
        List<PointD> source = [new(0, 0), new(2, 0), new(2, .99), new(5, 1),
            new(2, 1.01), new(2, 2), new(0, 2)];
        var prepared = ProfileToolpaths.Prepare([new MillingRegion(source)], ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.OnLine, SkipRegionsWithoutToolAccess = true,
                MinimumOpeningRadius = .05 });
        var ring = Assert.Single(prepared.Rings);
        Assert.Contains(ring.Points, p => p.x == 5 && p.y == 1);
        Assert.Contains(ring.Points, p => p.x == 2 && Math.Abs(p.y - .99) < 1.0 / 32768);
        Assert.Contains(ring.Points, p => p.x == 2 && Math.Abs(p.y - 1.01) < 1.0 / 32768);
    }

    [Fact]
    public void FullFilledComponentIncludesItsProtectedHolesInTheAccessProof()
    {
        var prepared = ProfileToolpaths.Prepare([new MillingRegion(Rect(0, 0, 3, 3),
            [Rect(.05, .05, 2.95, 2.95)])], ToolGeometry.Flat(3),
            new ProfileOptions { Side = ProfileSide.OnLine, SkipRegionsWithoutToolAccess = true,
                MinimumOpeningRadius = .05 });
        Assert.Empty(prepared.Rings);
        Assert.Equal(2, prepared.SourceMaterial.Count);
        Assert.Equal(2, Assert.Single(prepared.SourceComponents).Boundaries.Count);
    }

    [Fact]
    public void NoPositiveCommandCandidateSkipsCenterVisitsOnlyWhenEnabled()
    {
        MillingRegion[] regions = [new(Rect(0, 0, 10, 10))];
        var enabled = ProfileToolpaths.Generate(regions, ToolGeometry.Flat(1), new DepthSchedule(2, 2),
            new ProfileOptions { Side = ProfileSide.OnLine, Ramp = RampSettings.Plunge,
                SkipRegionsWithoutToolAccess = true, MinimumOpeningRadius = null });
        var ordinary = ProfileToolpaths.Generate(regions, ToolGeometry.Flat(1), new DepthSchedule(2, 2),
            new ProfileOptions { Side = ProfileSide.OnLine, Ramp = RampSettings.Plunge });
        Assert.Empty(enabled.Toolpaths);
        Assert.Empty(enabled.SweptArea);
        Assert.NotEmpty(ordinary.Toolpaths);
    }

    [Theory]
    [InlineData(ProfileSide.Inside)]
    [InlineData(ProfileSide.Outside)]
    public void AccessFlagDoesNotChangeInsideOrOutsidePolicies(ProfileSide side)
    {
        MillingRegion[] regions = [new(Rect(0, 0, 10, 10))];
        var ordinary = ProfileToolpaths.Prepare(regions, ToolGeometry.Flat(1), new ProfileOptions { Side = side });
        var flagged = ProfileToolpaths.Prepare(regions, ToolGeometry.Flat(1),
            new ProfileOptions { Side = side, SkipRegionsWithoutToolAccess = true, MinimumOpeningRadius = null });
        Assert.Equal(ordinary.Rings.Count, flagged.Rings.Count);
        Assert.Equal(ordinary.Rings.SelectMany(r => r.Points).ToArray(), flagged.Rings.SelectMany(r => r.Points).ToArray());
    }

    [Theory]
    [InlineData(-.01)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void EnabledCenterRejectsAnInvalidPhysicalRadius(double radius)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => ProfileToolpaths.Prepare(
            [new MillingRegion(Rect(0, 0, 10, 10))], ToolGeometry.Flat(1),
            new ProfileOptions { Side = ProfileSide.OnLine, SkipRegionsWithoutToolAccess = true,
                MinimumOpeningRadius = radius }));
    }

    private static MillingRegion[] Regions() => [new(Rect(0, 0, .006, .01)),
        new(Rect(10, 0, 10.03, 10)), new(Rect(20, 0, 20.15, 4))];

    private static List<PointD> Rect(double x0, double y0, double x1, double y1) =>
        [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
}
