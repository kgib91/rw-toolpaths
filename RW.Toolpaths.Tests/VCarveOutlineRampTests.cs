using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveOutlineRampTests
{
    private static readonly VCarveCutterProfile Profile = new([
        new(CarvingCutterSectionKind.Hemisphere, 0, .5, .5),
        new(CarvingCutterSectionKind.Linear, .5, 1.5, 10)], 10.5);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OpenPartAndSelectedInnerShoulderDoNotNominateRampRings(bool rampOutside)
    {
        PointD[] Box(double x, double y, double w, double h) =>
            [new(x,y),new(x+w,y),new(x+w,y+h),new(x,y+h)];
        var outside = Box(0,0,80,40);
        var selectedVoid = Box(10,15,10,10);
        var retainedIsland = Box(50,15,10,10);
        var targets = new[] { new MillingRegion(outside,[selectedVoid,retainedIsland]) };
        var tool = new ToolGeometry(1.5) { VCarveProfile = Profile };
        var access = new MillingBoundaryAccess([new MillingRegion(selectedVoid)],outside)
            .Resolve(targets,tool,new(2,1),true,30,0,default);
        var boundary = access.GetRegions(default).SelectMany(r => r.AsRingGroup()).ToArray();
        var logs = new List<string>();
        var options = new VCarveOptions { SmoothVCarving=true, RampOutsideEdges=rampOutside,
            IncludeInteriorFill=false, EdgeFillWidth=Profile.RampFillWidth(2,30),
            StepOver=.2, AccessTarget=access };
        var paths = VCarveRampSurface.Contours(boundary,options,Profile,2,0,default,logs.Add);
        Assert.NotEmpty(paths);
        var points = paths.SelectMany(p => p.Points.Concat(p.Points.Zip(p.Points.Skip(1),
            (a,b) => new Point3D((a.X+b.X)/2,(a.Y+b.Y)/2,(a.Z+b.Z)/2)))).ToArray();
        Assert.Contains(points,p => p.X > 46 && p.X < 50 && p.Y > 16 && p.Y < 24);
        if (rampOutside)
        {
            Assert.Contains(points,p => p.X < 1 && p.Y > 10 && p.Y < 30);
            Assert.Contains(points,p => p.X > 20 && p.X < 21 && p.Y > 16 && p.Y < 24);
        }
        else
        {
            Assert.DoesNotContain(points,p => p.X < 40 || p.Y < 5 || p.Y > 35);
            Assert.Contains(logs,s => s.Contains("contourSource=retained-physical-walls"));
        }
        Console.WriteLine($"[VCarve.OutsideShoulder.Test] rampOutside={rampOutside} paths={paths.Count} "
            + $"minX={points.Min(p=>p.X):R} maxX={points.Max(p=>p.X):R} selectedVoidIsDemand=False");
    }

    [Fact]
    public void SmoothGenerationFinishesRetainedIslandWithoutFillingOpenShoulders()
    {
        PointD[] Box(double x, double y, double w, double h) =>
            [new(x,y),new(x+w,y),new(x+w,y+h),new(x,y+h)];
        var outside=Box(0,0,80,40);
        var selectedVoid=Box(10,15,10,10);
        var plan=VCarveToolpaths.Generate([new MillingRegion(outside,[selectedVoid,Box(50,15,10,10)])],
            new ToolGeometry(1.5) { VCarveProfile=Profile },new(2,1),
            new VCarveOptions { SmoothVCarving=true, IncludeInteriorFill=false,
                EdgeFillWidth=Profile.RampFillWidth(2,30), StepOver=.2,
                BoundaryAccess=new([new MillingRegion(selectedVoid)],outside),
                DiagnosticContext="retained-island-open-shoulders", DetectCurves=false, CalculateSweptArea=false },
            BoostVoronoiProvider.CreateDefault());
        var clearing=plan.Toolpaths.Where(p=>p.Category=="clearing").ToArray();
        Assert.NotEmpty(clearing);
        Assert.DoesNotContain(clearing.SelectMany(p=>p.Points),p=>p.X<40 || p.Y<5 || p.Y>35);
        var commands=VCarveSmoothing.CommandPaths(plan.Toolpaths,0);
        Assert.InRange(RemovedDepth(commands,new(49,20)),Math.Tan(Math.PI/6)-.025,Math.Tan(Math.PI/6)+.005);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SufficientFillFinishesTheWholeSlopeAlongLongEdgesAndIslands(bool island)
    {
        PointD[] outer = [new(0, 0), new(80, 0), new(80, 40), new(0, 40)];
        PointD[] hole = [new(20, 15), new(20, 25), new(60, 25), new(60, 15)];
        IReadOnlyList<IReadOnlyList<PointD>> holes = island ? [hole] : [];
        var options = new VCarveOptions { SmoothVCarving = true, IncludeInteriorFill = false,
            EdgeFillWidth = Profile.RampFillWidth(2, 30), StepOver = .2, SmoothRampAngleDegrees = 30,
            DetectCurves = false, CalculateSweptArea = false, DiagnosticContext = $"full-outline-island-{island}" };
        var plan = VCarveToolpaths.Generate([new MillingRegion(outer, holes)],
            new ToolGeometry(1.5) { VCarveProfile = Profile }, new DepthSchedule(2, 1),
            options, BoostVoronoiProvider.CreateDefault());
        Assert.NotEmpty(plan.Toolpaths);
        var commands = VCarveSmoothing.CommandPaths(plan.Toolpaths, 0);
        double grade = Math.Tan(Math.PI / 6);
        foreach (double inset in new[] { .2, .5, 1d, 2d, 3d })
        {
            var sites = new List<PointD> { new(40, inset), new(40, 40 - inset),
                new(inset, 20), new(80 - inset, 20) };
            if (island) sites.AddRange([new(40, 15 - inset), new(40, 25 + inset),
                new(20 - inset, 20), new(60 + inset, 20)]);
            foreach (var site in sites)
            {
                double expected = Math.Min(2, grade * inset);
                double actual = RemovedDepth(commands, site);
                Assert.True(actual >= expected - .025 && actual <= expected + .005,
                    $"Incomplete or gouged outline slope at ({site.x},{site.y}): expected={expected:R}, actual={actual:R}.");
            }
        }
        // Ordinary fill remains off: the wide central floor has no new clearing.
        if (!island) Assert.Equal(0, RemovedDepth(commands, new(25, 20)));
        Assert.Contains(plan.Toolpaths, p => p.Category == "final-carve");
        Console.WriteLine($"[VCarve.OutlineRamp.Test] island={island} paths={plan.Toolpaths.Count} "
            + $"points={plan.Toolpaths.Sum(p => (long)p.Points.Count)} fillLimit={options.EdgeFillWidth:R} edgeMidpointSurfaceBudget=0.025");
    }

    [Theory]
    [InlineData(0)]
    [InlineData(7)]
    public void AClosedOutlineGetsRampContoursWithoutAnyMedialBranches(int regionIndex)
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[new(0, 0), new(40, 0), new(40, 20), new(0, 20)]];
        var logs = new List<string>();
        var paths = VCarveSmoothing.ApplyWithContinuation([], boundary, new ToolGeometry(1.5) { VCarveProfile = Profile },
            new DepthSchedule(2, 1), new VCarveOptions { SmoothVCarving = true,
                IncludeInteriorFill = false, EdgeFillWidth = 0, StepOver = .2 }, default, logs.Add,
                sourceRegionIndex: regionIndex);
        Assert.NotEmpty(paths);
        Assert.All(paths, p => Assert.Equal(regionIndex, p.RegionIndex));
        Assert.Contains(logs, s => s.Contains("stage=outline-ramps event=protected"));
        Assert.True(RemovedDepth(VCarveSmoothing.CommandPaths(paths, 0), new(20, 2)) > 1.12);
        Assert.All(paths, p => Assert.Equal(p.Points[0], p.Points[^1]));
    }

    [Fact]
    public void AShallowValidAngleStopsAtGeometryRatherThanAnUnreachableRingBudget()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[new(0, 0), new(40, 0), new(40, 20), new(0, 20)]];
        var paths = VCarveRampSurface.Contours(boundary, new VCarveOptions {
            SmoothVCarving = true, SmoothRampAngleDegrees = .01, StepOver = .2 }, Profile, 2, 0, default, null);
        Assert.NotEmpty(paths);
        Assert.InRange(paths.Count, 1, 101);
    }

    [Fact]
    public void CurvedInnerAndOuterOutlinesReceiveTheFullSlope()
    {
        PointD[] Circle(double radius, bool reverse = false)
        {
            var points = Enumerable.Range(0, 128).Select(i => new PointD(12 + radius * Math.Cos(i * Math.PI / 64),
                12 + radius * Math.Sin(i * Math.PI / 64))).ToArray();
            return reverse ? points.Reverse().ToArray() : points;
        }
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [Circle(10), Circle(4, true)];
        var paths = VCarveSmoothing.Apply([], boundary, new ToolGeometry(1.5) { VCarveProfile = Profile },
            new DepthSchedule(2, 1), new VCarveOptions { SmoothVCarving = true,
                IncludeInteriorFill = false, EdgeFillWidth = Profile.RampFillWidth(2, 30), StepOver = .2 }, default, null);
        var commands = VCarveSmoothing.CommandPaths(paths, 0);
        foreach (double inset in new[] { .5, 1d, 2d })
        foreach (double radius in new[] { 10 - inset, 4 + inset })
        for (int i = 0; i < 8; i++)
        {
            double angle = i * Math.PI / 4;
            double actual = RemovedDepth(commands, new(12 + radius * Math.Cos(angle), 12 + radius * Math.Sin(angle)));
            double expected = Math.Tan(Math.PI / 6) * inset;
            Assert.InRange(actual, expected - .025, expected + .005);
        }
    }

    [Fact]
    public void CrestChordProofPreservesStraightRampsAndRefusesAnIslandCrossing()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [
            [new(0, 0), new(40, 0), new(40, 20), new(0, 20)],
            [new(15, 4), new(15, 6), new(25, 6), new(25, 4)]];
        var crests = VCarveRampSurface.Sources(boundary, new VCarveOptions(), default);
        var field = new VCarveSmoothing.FloorField(crests, Profile, Math.Tan(Math.PI / 6), 2);
        Assert.True(field.ProveCrestChord(new(1, 1, -.45), new(39, 1, -.45)));
        Assert.False(field.ProveCrestChord(new(5, 5, -.5), new(35, 5, -.5)));
        Assert.False(field.ProveCrestChord(new(1, 1, -.45), new(39, 1, -2.1)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.01)]
    [InlineData(.25)]
    [InlineData(1)]
    [InlineData(2)]
    public void FillLimitCannotTruncateTheSlope(double width)
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[new(0, 0), new(80, 0), new(80, 40), new(0, 40)]];
        var logs = new List<string>();
        var paths = VCarveRampSurface.Contours(boundary, new VCarveOptions {
            SmoothVCarving = true, IncludeInteriorFill = false, EdgeFillWidth = width, StepOver = .2 },
            Profile, 2, 0, default, logs.Add);
        Assert.True(width < Profile.RampFillWidth(2, 30));
        var slope = VCarveRampSurface.Contours(boundary, new VCarveOptions {
            SmoothVCarving = true, IncludeInteriorFill = false, EdgeFillWidth = 0, StepOver = .2 },
            Profile, 2, 0, default, null);
        Assert.NotEmpty(slope);
        Assert.All(slope, ramp => Assert.Contains(paths, p => p.Points.SequenceEqual(ramp.Points)));
        Assert.Contains(paths.SelectMany(p => p.Points), p => p.X > 3.5 && p.X < 3.7);
        Assert.Contains(logs, s => s.Contains("rampTruncated=False") && s.Contains("fillLimitAppliesTo=post-slope-nearest-edge-distance"));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(8)]
    public void ZeroFillKeepsTheFullSlopeAndMedialSourceWithoutInteriorCleanup(double roughingReach)
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[new(0, 0), new(80, 0), new(80, 40), new(0, 40)]];
        var source = new TaggedToolpath([new(20, 20, -2), new(60, 20, -2)], [], 0, "final-carve", null, detectCurves: false);
        var logs = new List<string>();
        var paths = VCarveSmoothing.Apply([source], boundary, new ToolGeometry(1.5) { VCarveProfile = Profile },
            new DepthSchedule(2, 1), new VCarveOptions { SmoothVCarving = true,
                IncludeInteriorFill = false, EdgeFillWidth = 0, RoughingRestWidth = roughingReach }, default, logs.Add);
        var medial = Assert.Single(paths, p => p.Category == "final-carve");
        Assert.Equal(source.Points.Select(p => (p.X, p.Y)), medial.Points.Select(p => (p.X, p.Y)));
        Assert.NotEmpty(paths.Where(p => p.Category == "clearing"));
        Assert.All(paths.Where(p => p.Category == "clearing").SelectMany(p => p.Points),
            p => Assert.InRange(Math.Min(Math.Min(p.X, 80-p.X), Math.Min(p.Y, 40-p.Y)), 0, Profile.RampFillWidth(2,30)+.003));
        Assert.Contains(logs, s => s.Contains("stage=fill-limit event=complete") && s.Contains("slopeAdditive=True"));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(8)]
    public void SmallDistanceLimitFinishesTheSlopeAndCleanupBandButLeavesTheWideCore(double roughingReach)
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[new(0, 0), new(80, 0), new(80, 40), new(0, 40)]];
        var paths = VCarveSmoothing.Apply([], boundary, new ToolGeometry(1.5) { VCarveProfile = Profile },
            new DepthSchedule(2, 1), new VCarveOptions { SmoothVCarving = true,
                IncludeInteriorFill = false, EdgeFillWidth = 1, RoughingRestWidth = roughingReach, StepOver = .2 }, default, null);
        Assert.NotEmpty(paths);
        foreach (var point in paths.SelectMany(p => p.Points))
            Assert.InRange(Math.Min(Math.Min(point.X, 80 - point.X), Math.Min(point.Y, 40 - point.Y)), 0, Profile.RampFillWidth(2,30)+1.003);
        Assert.True(RemovedDepth(VCarveSmoothing.CommandPaths(paths,0),new(40,3))>1.7);
        Assert.Equal(0,RemovedDepth(VCarveSmoothing.CommandPaths(paths,0),new(40,20)));
    }

    [Fact]
    public void CapturedRoughingRestPolicyCannotExtendTheAuthoredRingBand()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[new(0, 0), new(80, 0), new(80, 40), new(0, 40)]];
        var source = new TaggedToolpath([
            new(.65, .65, -2), new(79.35, .65, -2), new(79.35, 39.35, -2),
            new(.65, 39.35, -2), new(.65, .65, -2)], [], 0, "clearing", null, detectCurves: false);
        var options = new VCarveOptions { SmoothVCarving = true, IncludeInteriorFill = false,
            EdgeFillWidth = 1, RoughingRestWidth = 8, StepOver = .2 };
        var captured = VCarveInitialStrategy.Capture(options, Profile, 2);
        Assert.True(captured.InitialStockCohort);
        var logs = new List<string>();
        var paths = VCarveSmoothing.Apply([source], boundary, new ToolGeometry(1.5) { VCarveProfile = Profile },
            new DepthSchedule(2, 1), options with { InitialStrategy = captured }, default, logs.Add);
        foreach (var point in paths.SelectMany(p => p.Points))
            Assert.InRange(Math.Min(Math.Min(point.X, 80 - point.X), Math.Min(point.Y, 40 - point.Y)), 0, Profile.RampFillWidth(2,30)+options.EdgeFillWidth+.003);
        Assert.DoesNotContain(logs, s => s.Contains("stage=ramp-rest-contours"));
        Assert.Equal(1+VCarveRampSurface.Contours(boundary,options,Profile,2,0,default,null).Count,paths.Count);
        Assert.All(paths,p=>Assert.Equal(p.Points[0],p.Points[^1]));
    }

    // Every value is a real cutter pose on an actual posted chord. Taking the
    // nearest XY pose gives a lower removal witness, independent of the planner's
    // floor field, coverage unions and contour bookkeeping.
    private static double RemovedDepth(IReadOnlyList<TaggedToolpath> paths, PointD site)
    {
        double depth = 0;
        foreach (var path in paths)
        for (int i = 1; i < path.Points.Count; i++)
        {
            var a = path.Points[i - 1]; var b = path.Points[i];
            double dx = b.X - a.X, dy = b.Y - a.Y, length2 = dx * dx + dy * dy;
            double t = length2 > 0 ? Math.Clamp(((site.x - a.X) * dx + (site.y - a.Y) * dy) / length2, 0, 1) : 0;
            double radius = Math.Sqrt(Math.Pow(a.X + t * dx - site.x, 2) + Math.Pow(a.Y + t * dy - site.y, 2));
            if (radius <= Profile.PhysicalRadiusAtDepth(Math.Max(0, -a.Z - t * (b.Z - a.Z))))
                depth = Math.Max(depth, -a.Z - t * (b.Z - a.Z) - Profile.PhysicalProfile.Bottom(radius));
        }
        return depth;
    }
}
