using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarvePostSlopeFillTests
{
    private static readonly VCarveCutterProfile Profile = new([
        new(CarvingCutterSectionKind.Hemisphere, 0, .5, .5),
        new(CarvingCutterSectionKind.Linear, .5, 1.5, 10)], 10.5);
    private const double Depth = 2;
    private static readonly double FloorInset = Profile.RampFillWidth(Depth,30);

    [Fact]
    public void TriangularResidualGetsNestedRingsWithinTheNearestEdgeDistance()
    {
        // An equilateral residual with a 2.4mm inscribed diameter, AFTER
        // the slope. The original triangle is far wider than either limit.
        double radius = FloorInset + 1.2;
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[
            new(-Math.Sqrt(3)*radius,-radius), new(Math.Sqrt(3)*radius,-radius), new(0,2*radius)]];
        var narrow = Contours(boundary, 1.3);
        var refused = Contours(boundary, .7);
        var floor = FloorPaths(narrow,boundary).ToArray();
        Assert.True(floor.Length >= 3);
        Assert.All(floor, p => Assert.Equal(p.Points[0],p.Points[^1]));
        Assert.True(Nearest(narrow,new(0,0)) < .2, "The little central triangle must be finished.");
        Assert.True(Nearest(refused,new(0,0)) > .4, "The narrower distance band must leave the inner core alone.");
        Assert.All(FloorPaths(refused,boundary),p=>Assert.Equal(p.Points[0],p.Points[^1]));
        Capture("triangle",boundary,narrow);
    }

    [Fact]
    public void OneBandClearsANarrowLobeAndLeavesTheWideConnectedCoreUnfilled()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[new(0,0),new(80,0),new(80,60),
            new(20,60),new(20,11),new(0,11)]];
        var logs = new List<string>();
        var paths = Contours(boundary, 2.5, logs.Add);
        var floor = FloorPaths(paths,boundary).ToArray();
        Assert.NotEmpty(floor);
        Assert.True(Nearest(floor,new(9,5.5)) < .2, "The narrow connected lobe needs cleanup.");
        Assert.True(Nearest(floor,new(50,30)) > 20, "The broad core lies beyond the cleanup band.");
        Assert.All(floor,p=>Assert.Equal(p.Points[0],p.Points[^1]));
        Assert.Contains(logs,s=>s.Contains("stage=post-slope-fill") && s.Contains("diameterConversion=False")
            && s.Contains("openingMask=False"));
        // Every nominated floor point remains inside the authored band.
        foreach(var path in floor)
        for(int i=1;i<path.Points.Count;i++)
        {
            var v=path.Points[i];
            double distance=boundary.Min(ring=>Enumerable.Range(0,ring.Count)
                .Min(edge=>SegmentDistance(new(v.X,v.Y),ring[edge],ring[(edge+1)%ring.Count])));
            Assert.InRange(distance-FloorInset,0,2.501);
        }
        Capture("narrow-lobe-wide-bay",boundary,paths);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.7)]
    public void DistanceStartsAtTheCompletedSlopeFrontAndDoesNotDeductAllowanceTwice(double allowance)
    {
        // The supplied boundary already includes allowance. Its remaining gap
        // after the final slope front is 2mm, independent of the original width.
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[new(0,0),new(50,0),
            new(50,2*FloorInset+2),new(0,2*FloorInset+2)]];
        var paths = VCarveRampSurface.Contours(boundary, Options(.6) with {StockToLeave=allowance},
            Profile,Depth,0,default,null);
        Assert.True(Nearest(FloorPaths(paths,boundary),new(25,FloorInset+1)) > .35);
        var accepted = VCarveRampSurface.Contours(boundary, Options(1.1) with {StockToLeave=allowance},
            Profile,Depth,0,default,null);
        Assert.True(Nearest(FloorPaths(accepted,boundary),new(25,FloorInset+1)) < .2);
    }

    [Fact]
    public void CutterSupportCollarDoesNotExcludeTheRemainingPocketShownBetweenSlopeRows()
    {
        double lip=Depth/Math.Tan(Math.PI/6);
        IReadOnlyList<IReadOnlyList<PointD>> boundary=[[new(0,0),new(30,0),
            new(30,2*lip+2),new(0,2*lip+2)]];
        double gap=2*lip+2-2*FloorInset;
        Assert.InRange(gap,1.7,1.8);
        var paths=Contours(boundary,.95);
        Assert.True(Nearest(FloorPaths(paths,boundary),new(15,lip+1))<.2,
            "The support collar is already completed slope, not part of the remaining gap.");
    }

    [Fact]
    public void TestDesignsRougherSizedFloorPatchGetsCleanupBetweenTheCompletedSlopeFronts()
    {
        const double depth=6.35, rougherDiameter=6.35, angle=50;
        var profile=new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),
            new(CarvingCutterSectionKind.Linear,1.5875,3,20)],21.5875);
        double reach=profile.RampFillWidth(depth,angle);
        double height=2*depth/Math.Tan(angle*Math.PI/180)+rougherDiameter;
        double remainingGap=height-2*reach;
        Assert.InRange(remainingGap,4.86,4.88);
        Assert.True(remainingGap<reach && rougherDiameter>reach);
        IReadOnlyList<IReadOnlyList<PointD>> boundary=[[new(0,0),new(50,0),new(50,height),new(0,height)]];
        var logs=new List<string>();
        var paths=VCarveRampSurface.Contours(boundary,Options(reach) with {
            SmoothRampAngleDegrees=angle,StepOver=.45},profile,depth,0,default,logs.Add);
        var cleanup=paths.Where(p=>p.Points.All(v=>Math.Min(v.Y,height-v.Y)>reach+.001)).ToArray();
        Assert.NotEmpty(cleanup);
        // A fixed lattice can finish up to one pitch short of the collapse;
        // it must add interior rings, without squeezing in another spacing.
        double nearest=Nearest(cleanup,new(25,height/2));
        Assert.True(nearest<.86,
            $"Rougher-sized patch needs interior rows within one pitch of its centre: nearest={nearest:R}.");
        Assert.Contains(logs,s=>s.Contains("residualBoundary=complete-last-slope-front"));
        Capture("rougher-circle-gap",boundary,paths,profile,depth,angle);
    }

    [Fact]
    public void FullInteriorFillIncludesTheWideBayAndZeroLimitKeepsOnlyTheSlope()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[new(0,0),new(35,0),new(35,20),new(0,20)]];
        var full = VCarveRampSurface.Contours(boundary, Options(0) with {IncludeInteriorFill=true},Profile,Depth,0,default,null);
        Assert.True(Nearest(full,new(17.5,10)) < .2);
        Assert.Empty(FloorPaths(Contours(boundary,0),boundary));
    }

    [Fact]
    public void FillLimitIsNotHalvedAndIncreasingItPreservesEarlierRows()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[new(0,0),new(50,0),
            new(50,2*FloorInset+2),new(0,2*FloorInset+2)]];
        var smaller=Contours(boundary,.6);
        var paths=Contours(boundary,1.1);
        Assert.True(Nearest(FloorPaths(paths,boundary),new(25,FloorInset+1))<.2);
        // Literal source rows and closure/order remain unchanged as the band grows.
        foreach(var path in smaller)
            Assert.Contains(paths,other=>path.Points.SequenceEqual(other.Points));
    }

    [Fact]
    public void AnIslandAndOuterSlopeMeasureTheRemainingAnnularGapTogether()
    {
        PointD[] Circle(double radius,bool reverse=false)
        {
            var points=Enumerable.Range(0,256).Select(i=>new PointD(radius*Math.Cos(i*Math.PI/128),
                radius*Math.Sin(i*Math.PI/128))).ToArray();
            return reverse?points.Reverse().ToArray():points;
        }
        // A 1.5mm remaining gap after BOTH completed slope fronts.
        double inner=20-2*FloorInset-1.5, middle=(20+inner)/2;
        IReadOnlyList<IReadOnlyList<PointD>> boundary=[Circle(20),Circle(inner,true)];
        Assert.True(Nearest(FloorPaths(Contours(boundary,.85),boundary),new(middle,0))<.2);
        Assert.True(Nearest(FloorPaths(Contours(boundary,.4),boundary),new(middle,0))>.3);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.2)]
    public void PointedAndFlatNosesKeepTheSamePostSlopeDistanceContract(double tipRadius)
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,tipRadius,1.5,10)],10);
        IReadOnlyList<IReadOnlyList<PointD>> boundary=[[new(0,0),new(30,0),
            new(30,2*profile.RampFillWidth(Depth,30)+2),new(0,2*profile.RampFillWidth(Depth,30)+2)]];
        var paths=VCarveSmoothing.Apply([],boundary,new ToolGeometry(1.5){VCarveProfile=profile},
            new(Depth,1),Options(1.1),default,null);
        var commands=VCarveSmoothing.CommandPaths(paths,0);
        Assert.True(Nearest(commands,new(15,profile.RampFillWidth(Depth,30)+1))<.08);
        Assert.InRange(commands.SelectMany(p=>p.Points).Max(p=>-p.Z),1.99,2.001);
        Assert.All(commands.SelectMany(p=>p.Points),p=>Assert.InRange(-p.Z,0,Depth+.001));
    }

    [Fact]
    public void CleanupBandStillPassesNativeAndPostedFloorWallAndGradeGuards()
    {
        double radius = FloorInset+1.2;
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[
            new(-Math.Sqrt(3)*radius,-radius),new(Math.Sqrt(3)*radius,-radius),new(0,2*radius)]];
        var paths = VCarveSmoothing.Apply([],boundary,new ToolGeometry(1.5){VCarveProfile=Profile},
            new(Depth,1){SurfaceZ=12.7},Options(1.3),default,null);
        var commands = VCarveSmoothing.CommandPaths(paths,12.7);
        var edges = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(boundary));
        foreach(var path in commands)
        foreach(var p in path.Points)
        {
            double distance=Math.Sqrt(edges.DistanceSquared(new((long)Math.Round(p.X*PathUtils.Scale),(long)Math.Round(p.Y*PathUtils.Scale))))/PathUtils.Scale;
            Assert.InRange(-p.Z,0,Math.Min(Depth,distance*Math.Tan(Math.PI/6))+.003);
            Assert.True(distance+.003>=Profile.PhysicalRadiusAtDepth(-p.Z));
        }
        Assert.True(Nearest(commands,new(0,0))<.2);
        Assert.InRange(commands.SelectMany(p=>p.Points).Max(p=>-p.Z),1.99,2.001);
    }

    [Fact]
    public void ExactSleevePocketIncludesThePreviouslyMissingInnerRows()
    {
        using var resource=typeof(VCarvePostSlopeFillTests).Assembly.GetManifestResourceStream(
            "RW.Toolpaths.Tests.Fixtures.vcarve-sleeve-pocket.json.gz")!;
        using var gzip=new System.IO.Compression.GZipStream(resource,System.IO.Compression.CompressionMode.Decompress);
        using var document=System.Text.Json.JsonDocument.Parse(gzip);
        var root=document.RootElement;
        var boundary=root.GetProperty("boundary").EnumerateArray().Select(ring=>(IReadOnlyList<PointD>)ring.EnumerateArray()
            .Select(p=>new PointD(p[0].GetDouble(),p[1].GetDouble())).ToArray()).ToArray();
        var profile=new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),
            new(CarvingCutterSectionKind.Linear,1.5875,3,20)],21.5875);
        double limit=root.GetProperty("fillLimit").GetDouble();
        var witness=new PointD(root.GetProperty("witness")[0].GetDouble(),root.GetProperty("witness")[1].GetDouble());
        var logs=new List<string>();
        var paths=VCarveRampSurface.Contours(boundary,Options(limit) with {
            SmoothRampAngleDegrees=50,StepOver=.45},profile,6.35,0,default,logs.Add);
        // The former disc-opening selector leaves this exact witness over 3mm
        // from cutting. It needs native concentric rows within the fixed pitch.
        Assert.True(Nearest(paths,witness)<.86,$"Missing sleeve cleanup near {witness}: nearest={Nearest(paths,witness):R}");
        Assert.True(Nearest(paths,new(149.5,150))<.86,"Both residual lobes need inner rings.");
        Assert.All(paths,p=>Assert.Equal(p.Points[0],p.Points[^1]));
        Assert.Contains(logs,s=>s.Contains("nearestEdgeLimit=6.068546") && s.Contains("openingMask=False"));
        Capture("exact-sleeve-pocket",boundary,paths,profile,6.35,50);
    }

    private static VCarveOptions Options(double width) => new(){SmoothVCarving=true,IncludeInteriorFill=false,
        EdgeFillWidth=width,StepOver=.2,SmoothRampAngleDegrees=30,DetectCurves=false,CalculateSweptArea=false};
    private static List<TaggedToolpath> Contours(IReadOnlyList<IReadOnlyList<PointD>> boundary,double width,Action<string>? log=null)
        => VCarveRampSurface.Contours(boundary,Options(width),Profile,Depth,0,default,log);
    private static IEnumerable<TaggedToolpath> FloorPaths(IEnumerable<TaggedToolpath> paths,
        IReadOnlyList<IReadOnlyList<PointD>> boundary)
        => paths.Where(p=>p.Points.All(v=>boundary.Min(ring=>Enumerable.Range(0,ring.Count)
            .Min(i=>SegmentDistance(new(v.X,v.Y),ring[i],ring[(i+1)%ring.Count])))
                >Profile.RampFillWidth(Depth,30)+.001));
    private static double SegmentDistance(PointD p,PointD a,PointD b)
    {
        double dx=b.x-a.x,dy=b.y-a.y;
        double t=Math.Clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy),0,1);
        return Math.Sqrt(Math.Pow(p.x-a.x-t*dx,2)+Math.Pow(p.y-a.y-t*dy,2));
    }
    private static double Nearest(IEnumerable<TaggedToolpath> paths,PointD site)
        => paths.SelectMany(p=>p.Points.Zip(p.Points.Skip(1),(a,b)=>SegmentDistance(site,new(a.X,a.Y),new(b.X,b.Y))))
            .DefaultIfEmpty(double.PositiveInfinity).Min();
    private static void Capture(string name,IReadOnlyList<IReadOnlyList<PointD>> boundary,IReadOnlyList<TaggedToolpath> paths,
        VCarveCutterProfile? profile=null,double depth=Depth,double angle=30)
    {
        string? directory=Environment.GetEnvironmentVariable("RW_POST_SLOPE_FILL_CAPTURE");
        if(directory is null)return;
        Directory.CreateDirectory(directory);
        profile??=Profile;
        File.WriteAllText(Path.Combine(directory,name+".json"),System.Text.Json.JsonSerializer.Serialize(new{
            boundary=boundary.Select(r=>r.Select(p=>new[]{p.x,p.y})),floorInset=profile.RampFillWidth(depth,angle),rampReach=profile.RampFillWidth(depth,angle),
            paths=paths.Select(p=>p.Points.Select(v=>new[]{v.X,v.Y,v.Z}))}));
    }
}
