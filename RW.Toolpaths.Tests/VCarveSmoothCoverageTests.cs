using System.Globalization;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveSmoothCoverageTests
{
    [Theory]
    [InlineData(33, 232.67421930680894, 126.86696286034066)]
    [InlineData(36, 266.1263227282848, 117.0173736122756)]
    public void SavedConcaveResidualFrontiersRestoreRealContinuousCutterCoverage(int region, double x, double y)
    {
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 21.5875);
        var tool = new ToolGeometry(3) { VCarveProfile = profile, BottomRadius = 1.5875,
            TopRadius = 3, ConeLength = 20, TipAngleRadians = 2 * Math.Atan((3 - 1.5875) / 20) };
        double surface = (double)12.7f;
        var witnesses = new List<string>();
        var previousTrace = MillingGeometryTrace.Observer.Value;
        MillingGeometryTrace.Observer.Value = (stage, batch, paths) => {
            if (stage is "protect-grade" or "stock-selected" or "source-circuits" or "routed-depth-passes")
                foreach (var group in paths.GroupBy(p => p.Category))
                    witnesses.Add($"stage={stage} batch={batch} category={group.Key} discDistance={DiscDistance(stage == "routed-depth-passes" ? group.ToArray() : VCarveSmoothing.CommandPaths(group.ToArray(), surface), stage == "routed-depth-passes" ? surface : 0):R}");
        };
        // Diameter allowance belongs to this operation; radius is half of it.
        // The pre-integration implementation used .016 and also produced a
        // .004521331 mm Region 36 witness, so .004 was not its contract.
        var operationOptions = new VCarveOptions { SmoothVCarving = true, IncludeInteriorFill = true,
            PathToleranceMm = .016, StepOver = .4, DetectCurves = false, CalculateSweptArea = false,
            DiagnosticContext = $"saved-residual-frontier-{region}" };
        ToolpathPlan result;
        try { result = VCarveToolpaths.Generate([new MillingRegion(VCarveSavedResidualFrontierFixture.Boundary(region))],
            tool, new DepthSchedule(6.35, 2) { SurfaceZ = surface, StartDepth = 6.35 },
            // This fixture exercises the unbounded native completion contract.
            // Finite operation Fill Limits intentionally permit unfinished stock.
            operationOptions, BoostVoronoiProvider.CreateDefault()); }
        finally { MillingGeometryTrace.Observer.Value = previousTrace; }
        double distance = DiscDistance(result.Toolpaths, surface);
        Assert.True(distance + 1e-9 <= operationOptions.CompletionGapRadiusMm,
            $"Saved region {region} leaves the independently proved gap witness ({x:R},{y:R}) at distance {distance:R} mm.\n"
            + string.Join('\n', witnesses));
        Console.WriteLine($"[VCarve.Smooth.SavedFrontierTest] region={region} commandDiscDistanceUpper={distance:R} " +
            $"radiusThreshold={operationOptions.CompletionGapRadiusMm:R} proof=real-continuous-command-disc source=complete-saved-artwork");

        double DiscDistance(IReadOnlyList<TaggedToolpath> paths, double surfaceZ)
        {
            double distance = double.PositiveInfinity;
            foreach (var path in CommandPaths(paths, surfaceZ))
            for (int i = 1; i < path.Points.Count; i++)
            {
                Point3D a = path.Points[i - 1], b = path.Points[i];
                if (Math.Max(-a.Z, -b.Z) <= 0) continue;
                double Value(double t)
                {
                    double dx = a.X + (b.X - a.X) * t - x, dy = a.Y + (b.Y - a.Y) * t - y;
                    double penetration = -a.Z - (b.Z - a.Z) * t;
                    if (penetration <= 0) return double.PositiveInfinity;
                    return Math.Sqrt(dx * dx + dy * dy) - profile.PhysicalRadiusAtDepth(penetration);
                }
                // Every sample is a real disc on the commanded chord. Optimization
                // only improves this upper distance bound; it cannot invent coverage.
                double lo = 0, hi = 1;
                for (int search = 0; search < 40; search++)
                {
                    double left = hi - (hi - lo) * .6180339887498949;
                    double right = lo + (hi - lo) * .6180339887498949;
                    if (Value(left) <= Value(right)) hi = right; else lo = left;
                }
                distance = Math.Min(distance, Math.Min(Math.Min(Value(0), Value(1)), Value((lo + hi) / 2)));
            }
            return distance;
        }
    }

    [Fact]
    public void SavedRegion36TighterOperationAllowanceFailsClosed()
    {
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 21.5875);
        var tool = new ToolGeometry(3) { VCarveProfile = profile, BottomRadius = 1.5875,
            TopRadius = 3, ConeLength = 20, TipAngleRadians = 2 * Math.Atan((3 - 1.5875) / 20) };
        var error = Assert.Throws<InvalidOperationException>(() => VCarveToolpaths.Generate(
            [new MillingRegion(VCarveSavedResidualFrontierFixture.Boundary(36))], tool,
            new DepthSchedule(6.35, 2) { SurfaceZ = (double)12.7f, StartDepth = 6.35 },
            new VCarveOptions { SmoothVCarving = true, IncludeInteriorFill = true,
                PathToleranceMm = .008, StepOver = .4, DetectCurves = false, CalculateSweptArea = false },
            BoostVoronoiProvider.CreateDefault()));
        Assert.Contains("could not restore channel coverage within 0.008 mm", error.Message);
    }

    [Theory]
    [InlineData(16, 5)]
    [InlineData(80, 12)]
    public void RoutedRoundedBallTaperRetainsUniformRampBandsAndPostedWallClearance(double length, double width)
    {
        const double surface = 12.7, depth = 3;
        PointD[] outline = [new(0, 0), new(length, 0), new(length, width), new(0, 1.5)];
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], depth);
        var tool = new ToolGeometry(3) { VCarveProfile = profile };
        var schedule = new DepthSchedule(depth, 1.5) { SurfaceZ = surface };
        var options = new VCarveOptions
        {
            IncludeInteriorFill = false,
            EdgeFillWidth = profile.RampFillWidth(depth, 15),
            SmoothRampAngleDegrees = 15,
            DetectCurves = false,
            CalculateSweptArea = false,
            DiagnosticContext = $"independent-width-{length:R}-{width:R}",
        };
        var provider = BoostVoronoiProvider.CreateDefault();
        var baseline = VCarveToolpaths.Generate([new MillingRegion(outline)], tool, schedule, options, provider);
        var smooth = VCarveToolpaths.Generate([new MillingRegion(outline)], tool, schedule,
            options with { SmoothVCarving = true }, provider);
        Assert.NotEmpty(smooth.Toolpaths);
        int surfaceEntries=smooth.Toolpaths.Where((path,index)=>index==0
            ||smooth.Toolpaths[index-1].Points[^1]!=path.Points[0]).Count();
        int rampContours=VCarveRampSurface.Contours([outline],options with {SmoothVCarving=true},
            profile,depth,0,default,null).Count;
        Assert.InRange(surfaceEntries,1,baseline.Toolpaths.Count+rampContours*schedule.PassCount);
        Assert.All(smooth.Toolpaths,path=>Assert.Null(path.ContinuationCircuitDefinition));
        Assert.Contains(smooth.Toolpaths,path=>path.Spans.Any(span=>span.Kind==ToolpathSpanKind.Ramp));
        if (length == 80)
            Assert.True(smooth.Toolpaths.Count <= 300,
                $"Wide unfilled taper schedules {smooth.Toolpaths.Count} visits; only source contours and medial branches are required.");
        for (int i = 0; i < smooth.Toolpaths.Count; i++)
            Assert.True(smooth.Toolpaths[i].Points[0].Z == surface
                || i > 0 && smooth.Toolpaths[i - 1].Points[^1] == smooth.Toolpaths[i].Points[0],
                $"Smooth visit {i} neither enters at surface nor continues the preceding cut.");

        // This independent audit runs after compaction and routing and includes
        // full entry paths. Both reference and result use the real float/F3
        // command convention, so uncommandable nominal tip grazes are excluded.
        const double sweepTolerance = .00075;
        var commands = CommandPaths(smooth.Toolpaths, surface);
        AssertCommandWallClearance(commands, outline, profile);
        // Judge the ramp surface at the authored nose stepover. A finite fill
        // does not demand restoration of an ordinary full-depth cut channel.
        double grade = Math.Tan(Math.PI / 12), nose = profile.Sections[0].TopRadius;
        double halfPitch = profile.ClearingDiameter * options.StepOver / 2;
        double scallop = (nose - Math.Sqrt(nose * nose - halfPitch * halfPitch)) * Math.Sqrt(1 + grade * grade);
        // A uniform offset lattice can end before the medial core of a taper.
        // Beyond its last local row, require that row's physical ball envelope;
        // do not demand an extra irregular contour or invented medial stroke.
        var rows=VCarveRampSurface.Contours([outline],options with {SmoothVCarving=true},profile,depth,0,default,null);
        double lastRow=rows.SelectMany(p=>p.Points.Zip(p.Points.Skip(1)))
            .Where(s=>Math.Abs(s.First.Y-s.Second.Y)<1e-9
                && Math.Min(s.First.X,s.Second.X)<=length/2 && Math.Max(s.First.X,s.Second.X)>=length/2)
            .Select(s=>s.First.Y).Max();
        double worstShortfall = 0;
        for (double y = .2; y < (width + 1.5) / 4; y += .2)
        {
            double x = length / 2;
            double clearance = double.PositiveInfinity;
            for (int edge = 0; edge < outline.Length; edge++)
            {
                var a = outline[edge]; var b = outline[(edge + 1) % outline.Length];
                double dx = b.x - a.x, dy = b.y - a.y;
                clearance = Math.Min(clearance, (dx * (y - a.y) - dy * (x - a.x)) / Math.Sqrt(dx * dx + dy * dy));
            }
            double expected = Math.Min(depth, grade * clearance), removed = 0;
            foreach (var path in commands)
            for (int i = 1; i < path.Points.Count; i++)
            {
                var a = path.Points[i - 1]; var b = path.Points[i];
                double dx = b.X - a.X, dy = b.Y - a.Y, square = dx * dx + dy * dy;
                double t = square > 0 ? Math.Clamp(((x - a.X) * dx + (y - a.Y) * dy) / square, 0, 1) : 0;
                double radius = Math.Sqrt(Math.Pow(a.X + t * dx - x, 2) + Math.Pow(a.Y + t * dy - y, 2));
                double tipDepth = -a.Z - t * (b.Z - a.Z);
                if (radius <= profile.PhysicalRadiusAtDepth(Math.Max(0, tipDepth)))
                    removed = Math.Max(removed, tipDepth - profile.PhysicalProfile.Bottom(radius));
            }
            worstShortfall = Math.Max(worstShortfall, expected - removed);
            double requiredLower=expected-scallop-options.PathToleranceMm;
            if(y>lastRow)
            {
                double tangent=nose*grade/Math.Sqrt(1+grade*grade);
                double lastTip=lastRow<=tangent
                    ? nose-Math.Sqrt(Math.Max(0,nose*nose-lastRow*lastRow))
                    : grade*lastRow-nose*(Math.Sqrt(1+grade*grade)-1);
                double r=y-lastRow;
                requiredLower=Math.Max(0,Math.Min(depth,lastTip)-profile.PhysicalProfile.Bottom(r)
                    -options.PathToleranceMm);
            }
            if (removed < requiredLower)
            {
                foreach (var path in commands)
                for (int i=1;i<path.Points.Count;i++)
                {
                    var a=path.Points[i-1];var b=path.Points[i];
                    double dx=b.X-a.X,dy=b.Y-a.Y,den=dx*dx+dy*dy;
                    double t=den>0?Math.Clamp(((x-a.X)*dx+(y-a.Y)*dy)/den,0,1):0;
                    double r=Math.Sqrt(Math.Pow(a.X+t*dx-x,2)+Math.Pow(a.Y+t*dy-y,2));
                    if(r<1.5 && a.Z<0 && b.Z<0)
                        Console.WriteLine($"[VCarve.RingSpacing.Coverage] site=({x:R},{y:R}) category={path.Category} "
                            +$"a={a} b={b} radius={r:R} tipDepth={-a.Z-t*(b.Z-a.Z):R}");
                }
            }
            Assert.True(removed >= requiredLower && removed<=expected+.005,
                $"Ramp coverage at ({x:R},{y:R}): expected={expected:R} removed={removed:R} lower={requiredLower:R} "
                +$"lastConcentricRow={lastRow:R} scallop={scallop:R}");
        }
        // Full-outline slopes intentionally add work beyond the ordinary medial
        // channel, even with no fill. The original artwork still bounds the
        // complete upper cutter sweep. Wide untouched floors are checked by
        // VCarveOutlineRampTests independently of this tapered channel fixture.
        const double footprintTolerance = .009;
        var actualUpper = VCarvePhysicalSweep.Build(commands, profile, 0, sweepTolerance, upperBound: true);
        var allowedFootprint = RawMorphology([outline], footprintTolerance);
        var outsideArtwork = PolygonOps.Difference(actualUpper, allowedFootprint);
        double extraArea = Math.Abs(outsideArtwork.Sum(polygon => Clipper.Area(new PathD(polygon))));
        Assert.True(outsideArtwork.Count == 0,
            $"Full-outline ramp crosses artwork: length={length:R}, width={width:R}, " +
            $"extraRings={outsideArtwork.Count}, extraArea={extraArea:R}, footprintTolerance={footprintTolerance:R}.");
        Console.WriteLine($"[VCarve.Smooth.CoverageTest] length={length:R} width={width:R} baselinePaths={baseline.Toolpaths.Count} " +
            $"smoothPaths={smooth.Toolpaths.Count} maximumSurfaceShortfall={worstShortfall:R} authoredScallop={scallop:R} " +
            $"extraArea={extraArea:R} footprintTolerance={footprintTolerance:R} " +
            "coordinates=float-F3 stage=after-routing continuousPhysicalWallAudit=True fullOutlineRamps=True");
    }

    private static List<List<PointD>> RawMorphology(IEnumerable<IReadOnlyList<PointD>> source, double delta)
    {
        const double scale = PathUtils.Scale;
        var integer = new Paths64(source.Select(ring => new Path64(ring.Select(point =>
            new Point64((long)Math.Round(point.x * scale), (long)Math.Round(point.y * scale))))));
        var offset = new ClipperOffset(2, .25);
        offset.AddPaths(Clipper.Union(integer, FillRule.NonZero), JoinType.Round, EndType.Polygon);
        var result = new Paths64();
        offset.Execute(delta * scale, result);
        return Clipper.Union(result, FillRule.NonZero)
            .Select(ring => ring.Select(point => new PointD(point.X / scale, point.Y / scale)).ToList()).ToList();
    }

    private static void AssertCommandWallClearance(IReadOnlyList<TaggedToolpath> paths,
        IReadOnlyList<PointD> outline, VCarveCutterProfile profile)
    {
        double sphere = profile.Sections[0].TopRadius, worst = double.PositiveInfinity;
        long chords = 0;
        for (int pathIndex = 0; pathIndex < paths.Count; pathIndex++)
        for (int i = 1; i < paths[pathIndex].Points.Count; i++)
        {
            var path = paths[pathIndex];
            var a = path.Points[i - 1]; var b = path.Points[i];
            double da = -a.Z, db = -b.Z, delta = db - da;
            if (Math.Max(da, db) <= 0) continue;
            chords++;
            for (int edge = 0; edge < outline.Count; edge++)
            {
                var p = outline[edge]; var q = outline[(edge + 1) % outline.Count];
                double vx = q.x - p.x, vy = q.y - p.y, length = Math.Sqrt(vx * vx + vy * vy);
                double ca = (vx * (a.Y - p.y) - vy * (a.X - p.x)) / length;
                double cb = (vx * (b.Y - p.y) - vy * (b.X - p.x)) / length;
                var samples = new List<double> { 0, 1 };
                if (delta != 0)
                {
                    samples.Add((0 - da) / delta);
                    samples.Add((sphere - da) / delta);
                    double slope = (cb - ca) / delta;
                    if (slope >= 0)
                    {
                        // Exact stationary point of linear wall clearance minus
                        // the concave spherical radius, including off-midpoint
                        // extrema. Cone sections need only interval endpoints.
                        double root = Math.Sqrt(1 + slope * slope);
                        double depth = sphere / (root * (root + slope));
                        samples.Add((depth - da) / delta);
                    }
                }
                foreach (double t in samples)
                {
                    if (t < 0 || t > 1) continue;
                    double depth = da + delta * t;
                    if (depth < 0) continue;
                    double clearance = ca + (cb - ca) * t;
                    double radius = profile.PhysicalRadiusAtDepth(depth);
                    worst = Math.Min(worst, clearance - radius);
                    if (clearance < radius - 1e-6)
                    {
                        string spans = string.Join(",", path.Spans.Where(span => span.StartIndex < i && span.EndIndex >= i)
                            .Select(span => span.Kind.ToString()).DefaultIfEmpty("Cut"));
                        Assert.True(false,
                            $"Commanded sphere/taper crosses wall: chord={chords}, path={pathIndex}, category={path.Category}, " +
                            $"localChord={i - 1}, span={spans}, edge={edge}, t={t:R}, depth={depth:R}, clearance={clearance:R}, radius={radius:R}, " +
                            $"from=({a.X:R},{a.Y:R},{a.Z:R}), to=({b.X:R},{b.Y:R},{b.Z:R}).");
                    }
                }
            }
        }
        Assert.True(chords > 0);
        Console.WriteLine($"[VCarve.Smooth.ContinuousWallTest] chords={chords} minimumClearanceReserve={worst:R} " +
            "sphereStationaryPoint=True coneEndpoints=True coordinateBasis=float-F3");
    }

    private static List<TaggedToolpath> CommandPaths(IReadOnlyList<TaggedToolpath> source, double surface)
    {
        static double Command(float value)
            => double.Parse(value.ToString("F3", CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);
        return source.Select(path => new TaggedToolpath(path.Points.Select(point => new Point3D(
            Command((float)point.X), Command((float)point.Y), Command((float)point.Z - (float)surface))).ToArray(),
            path.Spans, path.RegionIndex, path.Category, path.DepthPassIndex, detectCurves: false)).ToList();
    }
}
