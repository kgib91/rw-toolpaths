using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveSupplementalFinishingTests
{
    [Fact]
    public void ShallowLateralSitesFinishToThePermittedFloorWithSurfaceOriginStepdowns()
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
            new(CarvingCutterSectionKind.Linear, 1, 2, 10)], 10);
        var tool = new ToolGeometry(2) { VCarveProfile = profile };
        var options = new VCarveOptions { SmoothVCarving = true, IncludeInteriorFill = false, EdgeFillWidth = 4 };
        options = options with { InitialStrategy = VCarveInitialStrategy.Capture(options, profile, 3) };
        var shallow = new TaggedToolpath([new(5, 5, -.75), new(15, 5, -.75), new(15, 15, -.75),
            new(5, 15, -.75), new(5, 5, -.75)], [], 0,
            VCarveInitialReachSelection.HalfCoreMarkerCategory, null, detectCurves: false);
        var medial = new TaggedToolpath([new(6, 10, -3), new(14, 10, -3)], [], 0, "final-carve", null, detectCurves: false);
        IReadOnlyList<PointD>[] boundary = [[new(0, 0), new(20, 0), new(20, 20), new(0, 20)]];
        var schedule = new DepthSchedule(3, 1);
        var finish = VCarveSmoothing.Apply([shallow, medial], boundary, tool, schedule, options, default, Console.WriteLine);
        // Witness on the lateral contour must be cut to the flat floor,
        // rather than its construction depth, after the real physical owners.
        var visits = VCarveToolpaths.ScheduleFreshStockVisits(finish, schedule, default, Console.WriteLine)
            .Where(path => path.Points.Any(p => p.X == 5 && p.Y == 5)).ToArray();
        Assert.Contains(visits, path => path.Points.Any(p => p.X == 5 && p.Y == 5 && p.Z == -3));
        foreach (double depth in new[] { 1d, 2d, 3d })
            Assert.Contains(visits, path => path.Points.Any(p => p.X == 5 && p.Y == 5 && p.Z == -depth));
        Assert.All(shallow.Points, point => Assert.Equal(-.75, point.Z));
        Assert.All(visits, path => Assert.Equal("clearing", path.Category));
        Console.WriteLine("[VCarve.SupplementalFinishingTest] fullDepthWitness=3 constructionDepth=0.75 stepdown=1 originalNominationImmutable=True");
    }
}
