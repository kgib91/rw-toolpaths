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
        var result = VCarveToolpaths.Generate([new MillingRegion(VCarveSavedResidualFrontierFixture.Boundary(region))],
            tool, new DepthSchedule(6.35, 2) { SurfaceZ = surface, StartDepth = 6.35 },
            new VCarveOptions { SmoothVCarving = true, IncludeInteriorFill = false, EdgeFillWidth = 3.175,
                StepOver = .4, DetectCurves = false, CalculateSweptArea = false,
                DiagnosticContext = $"saved-residual-frontier-{region}" }, BoostVoronoiProvider.CreateDefault());
        double distance = double.PositiveInfinity;
        foreach (var path in CommandPaths(result.Toolpaths, surface))
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
        Assert.True(distance + 1e-9 <= .004,
            $"Saved region {region} leaves the independently proved gap witness ({x:R},{y:R}) at distance {distance:R} mm.");
        Console.WriteLine($"[VCarve.Smooth.SavedFrontierTest] region={region} commandDiscDistanceUpper={distance:R} " +
            "radiusThreshold=0.004 proof=real-continuous-command-disc source=complete-saved-artwork");
    }

    [Theory]
    [InlineData(16, 5)]
    [InlineData(80, 12)]
    public void RoutedRoundedBallTaperRestoresCommandedWidthWithoutWideningTheOriginalChannel(double length, double width)
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
            EdgeFillWidth = 0,
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
        Assert.InRange(surfaceEntries,1,baseline.Toolpaths.Count);
        Assert.Contains(smooth.Toolpaths,path=>path.Spans.Any(span=>span.Kind==ToolpathSpanKind.Ramp));
        if (length == 80)
            Assert.True(smooth.Toolpaths.Count <= 300,
                $"Wide unfilled taper schedules {smooth.Toolpaths.Count} independent visits; early terminal recovery should remain sparse.");
        for (int i = 0; i < smooth.Toolpaths.Count; i++)
            Assert.True(smooth.Toolpaths[i].Points[0].Z == surface
                || i > 0 && smooth.Toolpaths[i - 1].Points[^1] == smooth.Toolpaths[i].Points[0],
                $"Smooth visit {i} neither enters at surface nor continues the preceding cut.");

        // This independent audit runs after compaction and routing and includes
        // full entry paths. Both reference and result use the real float/F3
        // command convention, so uncommandable nominal tip grazes are excluded.
        const double sweepTolerance = .00075;
        var originalUpper = VCarvePhysicalSweep.Build(
            CommandPaths(baseline.Toolpaths, surface), profile, 0, sweepTolerance, upperBound: true);
        var reference = PolygonOps.Intersect(originalUpper, [outline]);
        // At F3 the first positive command is .001 mm. Its finite ball footprint
        // cannot enter a mathematically sharp source corner. An independent
        // opening defines reachable source material without borrowing the
        // smoother's already-trimmed reference paths or its clearance helper.
        double firstRadius = profile.PhysicalRadiusAtDepth(.001);
        var centers = RawMorphology(reference, -firstRadius);
        var reachable = RawMorphology(centers, firstRadius);
        reference = PolygonOps.Intersect(reference, reachable);
        var commands = CommandPaths(smooth.Toolpaths, surface);
        AssertCommandWallClearance(commands, outline, profile);
        var actual = VCarvePhysicalSweep.Build(commands, profile, 0, sweepTolerance);
        var residual = PolygonOps.Difference(reference, actual);
        // Pure geometric erosion must not invoke machining-contour validation:
        // that validator can discard a valid residual contour because a round
        // join's polygon chord lies slightly inside the requested offset.
        var significant = RawMorphology(residual, -VCarveSmoothing.CoverageTolerance / 2);
        double area = Math.Abs(residual.Sum(polygon => Clipper.Area(new PathD(polygon))));
        Assert.True(significant.Count == 0,
            $"Routed commanded coverage leaves significant pockets: count={significant.Count}, totalResidualArea={area:R}.");
        // Restoring lost width must not clear the originally uncut interior of
        // a depth-capped, unfilled V-carve. The long taper exposes this even when
        // every resulting command remains strictly inside the artwork walls.
        // Compare an upper bound of the actual sweep against the ordinary upper
        // footprint expanded by the 8 micron spatial contract plus 1 micron for
        // the two independently tessellated physical sweeps and Boolean grid.
        const double footprintTolerance = .009;
        var actualUpper = VCarvePhysicalSweep.Build(commands, profile, 0, sweepTolerance, upperBound: true);
        var allowedFootprint = RawMorphology(originalUpper, footprintTolerance);
        var outsideOriginal = PolygonOps.Difference(actualUpper, allowedFootprint);
        double extraArea = Math.Abs(outsideOriginal.Sum(polygon => Clipper.Area(new PathD(polygon))));
        Assert.True(outsideOriginal.Count == 0,
            $"Smooth correction widened the original unfilled channel: length={length:R}, width={width:R}, " +
            $"extraRings={outsideOriginal.Count}, extraArea={extraArea:R}, footprintTolerance={footprintTolerance:R}.");
        Console.WriteLine($"[VCarve.Smooth.CoverageTest] length={length:R} width={width:R} baselinePaths={baseline.Toolpaths.Count} " +
            $"smoothPaths={smooth.Toolpaths.Count} residualArea={area:R} " +
            $"spatialTolerance={VCarveSmoothing.CoverageTolerance:R} firstEngagedRadius={firstRadius:R} " +
            $"extraArea={extraArea:R} footprintTolerance={footprintTolerance:R} " +
            "coordinates=float-F3 stage=after-routing continuousPhysicalWallAudit=True originalChannelBounded=True");
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
