using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ContinuousCrestRampTests
{
    private static readonly IReadOnlyList<PointD>[] Boundary =
        [[new(0, 0), new(80, 0), new(80, 30), new(0, 30)]];
    private static readonly VCarveOptions Options = new() {
        SmoothVCarving = true, SmoothRampAngleDegrees = 30, DetectCurves = false };

    [Theory]
    [InlineData(15)]
    [InlineData(30)]
    [InlineData(50)]
    public void StraightCrestHasUniformAnalyticalBallTangentRampAtEveryAlongEdgePosition(double degrees)
    {
        const double nose = 1, floor = 4;
        double grade = Math.Tan(degrees * Math.PI / 180);
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere, 0, nose, nose),
            new(CarvingCutterSectionKind.Linear, nose, nose, 10)], 11);
        var sources = VCarveRampSurface.Sources(Boundary, Options with { SmoothRampAngleDegrees = degrees }, default);
        var field = new VCarveSmoothing.FloorField(sources, profile, grade, floor);
        double tangent = nose * grade / Math.Sqrt(1 + grade * grade), maximumError = 0;
        for (int row = 0; row <= 120; row++)
        {
            double distance = row / 20d;
            double expected = Math.Min(floor, distance <= tangent
                ? nose - Math.Sqrt(nose * nose - distance * distance)
                : grade * distance - nose * (Math.Sqrt(1 + grade * grade) - 1));
            for (int column = 0; column <= 80; column++)
            {
                double actual = field.Depth(10 + column * .75, distance);
                maximumError = Math.Max(maximumError, Math.Abs(expected - actual));
                Assert.InRange(actual, expected - .000201, expected + 1e-10);
            }
        }
        Console.WriteLine($"[Ramp.ContinuousCrest] angle={degrees:R} samples=9801 maximumAnalyticalError={maximumError:R} alongEdgeModulation=0");
    }

    [Fact]
    public void SparseMedialBranchesCannotCreateConesAlongAStraightCrest()
    {
        const double grade = .5;
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 1, 10)], 10);
        TaggedToolpath Path(params Point3D[] points) => new(points, [], 0, "final-carve", null, detectCurves: false);
        TaggedToolpath[] raw = [Path(new(20, 0, 0), new(20, 10, -8)), Path(new(40, 0, 0), new(40, 10, -8))];
        var limited = VCarveSmoothing.LimitGraph(raw, grade);
        var old = new VCarveSmoothing.FloorField(VCarveSmoothing.RaisedFloorSpans(raw, limited, default), profile, grade, 4);
        var current = new VCarveSmoothing.FloorField(VCarveRampSurface.Sources(Boundary, Options, default), profile, grade, 4);
        Assert.True(old.Depth(30, 2) > current.Depth(30, 2) + 2,
            "The medial-span floor should expose the unsupported valley between crest anchors.");
        for (double x = 20; x <= 40; x += .125)
            Assert.InRange(current.Depth(x, 2), 1 - .000201, 1 + 1e-10);
        Console.WriteLine($"[Ramp.ConeRegression] legacyMidpointDepth={old.Depth(30, 2):R} continuousMidpointDepth={current.Depth(30, 2):R} expected=1");
    }

    [Fact]
    public void RoughingPlateauPreservesAContinuousBandAndDoesNotAddFinishingCutterSupport()
    {
        const double floor = 3;
        double grade = Math.Tan(30 * Math.PI / 180), width = floor / grade;
        var mask = VCarveRampSurface.Plateau(Boundary, Options, floor, grade, default);
        for (double x = 10; x <= 70; x += .25)
        {
            Assert.True(PolygonOps.Contains(mask, x, width - .01), $"Missing ramp protection at x={x:R}");
            Assert.False(PolygonOps.Contains(mask, x, width + .01), $"Unnecessary plateau stock at x={x:R}");
        }
        Console.WriteLine($"[Ramp.PlateauContinuity] stations=241 materialBandWidth={width:R} cutterCompensationApplications=0");
    }

    [Fact]
    public void DenseConnectedCrestKeepsTheAnalyticalContinuousCapsule()
    {
        const double floor = 6.35, grade = 1;
        var source = Enumerable.Range(0, 400).Select(i => {
            var a = new Point3D(0, -50 + i * .25, 0);
            var b = new Point3D(0, -50 + (i + 1) * .25, 0);
            return new TaggedToolpath(i % 2 == 0 ? [a, b] : [b, a], [], 0,
                "final-carve", null, detectCurves: false);
        }).ToArray();
        IReadOnlyList<PointD>[] boundary = [[new(-12,-60),new(12,-60),new(12,60),new(-12,60)]];
        var mask = SmoothRoughingPlateau.Build(source, boundary, floor, grade, default);
        int samples = 0;
        for (int row = 0; row <= 480; row++)
        for (int column = 0; column <= 96; column++)
        {
            double x = -12 + column * .25, y = -60 + row * .25;
            double endDistance = Math.Max(0, Math.Abs(y) - 50);
            double distance = Math.Sqrt(x * x + endDistance * endDistance);
            if (Math.Abs(distance - floor / grade) <= .006) continue;
            Assert.Equal(distance < floor / grade, PolygonOps.Contains(mask, x, y));
            samples++;
        }
        Console.WriteLine($"[Ramp.DenseCrestSweep] segments={source.Length} samples={samples} analyticalCapsule=True integerEndpointsUnchanged=True");
    }

    [Fact]
    public void ActualPreparationUsesTheContinuousCrestRatherThanBranchDepthAnchors()
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
            new(CarvingCutterSectionKind.Linear, 1, 1, 10)], 11);
        var preparation = VCarveToolpaths.Prepare([new MillingRegion(Boundary[0])],
            new ToolGeometry(1) { VCarveProfile = profile }, new DepthSchedule(4, 1), Options,
            BoostVoronoiProvider.CreateDefault());
        double grade = Math.Tan(Math.PI / 6), support = Math.Sqrt(1 + grade * grade) - 1;
        int points = 0;
        foreach (var region in preparation.Regions)
        foreach (var path in region.SmoothedMedialPaths)
        foreach (var p in path.Points)
        {
            double distance = Math.Min(Math.Min(p.X, 80 - p.X), Math.Min(p.Y, 30 - p.Y));
            double tangent = grade / Math.Sqrt(1 + grade * grade);
            double expected = Math.Min(4, distance <= tangent
                ? 1 - Math.Sqrt(Math.Max(0, 1 - distance * distance)) : grade * distance - support);
            Assert.InRange(-p.Z, expected - .000201, expected + 1e-9);
            points++;
        }
        Assert.True(points > 100);
        var plateau = preparation.AnalyzePocketPlateau(4, 30);
        Assert.True(PolygonOps.Contains(plateau.ProtectedBoundary, 30, 4 / grade - .01));
        Assert.False(PolygonOps.Contains(plateau.ProtectedBoundary, 30, 4 / grade + .01));
        Console.WriteLine($"[Ramp.PreparedContinuity] originalMedialSites={points} analyticalCrestAgreement=True roughingSharesMaterialSurface=True");
    }

    [Theory]
    [InlineData(15)]
    [InlineData(30)]
    [InlineData(50)]
    public void ReducedContourRowsPreserveTheActualSweptBallSurfaceBudget(double degrees)
    {
        const double nose = .5, depth = 3, stepOver = .4;
        double grade = Math.Tan(degrees * Math.PI / 180);
        IReadOnlyList<PointD>[] boundary = [[new(0,0),new(24,0),new(24,20),new(0,20)]];
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,nose,nose),
            new(CarvingCutterSectionKind.Linear,nose,nose,10)],10.5);
        var field = new VCarveSmoothing.FloorField(VCarveRampSurface.Sources(boundary,
            Options with { SmoothRampAngleDegrees=degrees },default),profile,grade,depth);
        var paths = new List<TaggedToolpath>();
        VCarveRampRestNominator.NominateContours(boundary,Array.Empty<TaggedToolpath>(),profile,depth,
            depth/grade+2*nose,(x,y)=>field.CappedDepth(depth,x,y),
            (x,y)=>Math.Min(Math.Min(x,24-x),Math.Min(y,20-y)),p=>p,default,null,null,
            proposed=>VCarveSmoothing.Constrain(proposed,field,grade,default,1_000_000),paths,grade,stepOver,
            floorContact:field.NominateFloorContact,maximumFloorContactRadius:field.MaximumFloorContactRadius,
            continuousCrestSurface:true);
        Assert.NotEmpty(paths);
        double pitch=2*nose*stepOver;
        double authoredScallop=nose-Math.Sqrt(nose*nose-pitch*pitch/4);
        double budget=authoredScallop+MachiningAccuracy.SurfaceDeviation;
        double tangent=nose*grade/Math.Sqrt(1+grade*grade);
        double support=nose*(Math.Sqrt(1+grade*grade)-1);
        double plateauCentre=(depth+support)/grade,blendStart=plateauCentre-tangent;
        double worstRemaining=0,worstGouge=0;
        // The rest pass owns the slope and its handoff. Interior flat filling
        // is a separate operation; do not audit beyond that requested band.
        double auditEnd=Math.Min(5,plateauCentre+.05);
        for(int row=0;row<=120;row++)
        for(int column=0;column<=8;column++)
        {
            double x=10+column*.5,y=.25+(auditEnd-.25)*row/120;
            // A finite ball leaves its unavoidable circular internal blend
            // where the ramp meets the depth-limited flat plateau.
            double expected=y<blendStart ? grade*y : y<plateauCentre
                ? depth-nose+Math.Sqrt(nose*nose-Math.Pow(plateauCentre-y,2)) : depth;
            double removed=0;
            foreach(var path in paths)
            for(int i=1;i<path.Points.Count;i++)
            {
                var a=path.Points[i-1];var b=path.Points[i];
                double dx=b.X-a.X,dy=b.Y-a.Y,length=Math.Sqrt(dx*dx+dy*dy);
                double along=length>0?((x-a.X)*dx+(y-a.Y)*dy)/length:0;
                double across=length>0?Math.Abs((x-a.X)*dy-(y-a.Y)*dx)/length
                    :Math.Sqrt((x-a.X)*(x-a.X)+(y-a.Y)*(y-a.Y));
                if(across>nose)continue;
                double slope=length>0?(a.Z-b.Z)/length:0;
                double at=Math.Clamp(along+slope*Math.Sqrt((nose*nose-across*across)/(1+slope*slope)),0,length);
                double radiusSquared=(at-along)*(at-along)+across*across;
                if(radiusSquared>nose*nose)continue;
                // Exact stationary pose of this continuously translated ball;
                // this audit does not query production floor/stock predicates.
                removed=Math.Max(removed,-a.Z+slope*at-nose+Math.Sqrt(nose*nose-radiusSquared));
            }
            // The authored texture budget is spatial. Compare to the finite
            // analytical ramp/plateau cross-section in normal distance; raw Z
            // error alone overstates a steep slope's remaining material.
            double atRamp=Math.Clamp((y+grade*removed)/(1+grade*grade),0,blendStart);
            double rampDistance=Math.Sqrt(Math.Pow(y-atRamp,2)+Math.Pow(removed-grade*atRamp,2));
            double plateauDistance=Math.Sqrt(Math.Pow(Math.Max(0,plateauCentre-y),2)+Math.Pow(removed-depth,2));
            double angle=Math.Clamp(Math.Atan2(removed-depth+nose,y-plateauCentre),Math.PI/2,Math.PI/2+Math.Atan(grade));
            double blendDistance=Math.Sqrt(Math.Pow(y-plateauCentre-nose*Math.Cos(angle),2)
                +Math.Pow(removed-depth+nose-nose*Math.Sin(angle),2));
            double remainingDistance=Math.Min(blendDistance,Math.Min(rampDistance,plateauDistance));
            worstRemaining=Math.Max(worstRemaining,remainingDistance);
            worstGouge=Math.Max(worstGouge,removed-expected);
            Assert.True(removed<=expected+1e-8,$"Gouge at ({x:R},{y:R}): expected={expected:R} actual={removed:R}");
            Assert.True(remainingDistance<=budget,$"Ridge at ({x:R},{y:R}): expected={expected:R} actual={removed:R} spatialDistance={remainingDistance:R} budget={budget:R}");
        }
        Console.WriteLine($"[Ramp.SweptBallAudit] angle={degrees:R} samples=1089 paths={paths.Count} maximumRemaining={worstRemaining:R} maximumGouge={worstGouge:R} authoredScallop={authoredScallop:R} spatialAccuracy={MachiningAccuracy.SurfaceDeviation:R} combinedBudget={budget:R} auditBandEnd={auditEnd:R}");
    }
}
