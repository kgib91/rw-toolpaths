using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class SmoothRoughingPlateauTests
{
    [Fact]
    public void PairedGradeFieldsPreserveTheIndependentResultsAndVisitMetadata()
    {
        TaggedToolpath Path(params Point3D[] points) => new(points, [], 2, "final-carve", 1, 17, detectCurves: false);
        TaggedToolpath[] source = [Path(new(0, 0, 0), new(1, 0, -3), new(2, 0, -1)),
            Path(new(1, 0, -3), new(1, 2, -.1)), Path(new(2, 0, -2), new(3, 0, -4))];
        var upper = source.Select(path => path with { Points = path.Points.Select(p => p with { Z = p.Z - .013 }).ToArray() }).ToArray();
        var pair = VCarveSmoothing.LimitGraphPair(source, upper, .7, default);
        var expected = new[] { VCarveSmoothing.LimitGraph(source, .7), VCarveSmoothing.LimitGraph(upper, .7) };
        foreach (var (actual, reference) in new[] { (pair.Limited, expected[0]), (pair.Upper, expected[1]) })
        for (int i = 0; i < actual.Count; i++)
        {
            Assert.Equal(reference[i].Points, actual[i].Points);
            Assert.Equal(reference[i].VisitIdentity, actual[i].VisitIdentity);
            Assert.Equal(reference[i].DepthPassIndex, actual[i].DepthPassIndex);
        }
    }

    [Fact]
    public void DeepShortChordsKeepTheAuthoredGradeAfterFloatingPointAddition()
    {
        const double grade = .5773502691896257;
        for (int i = 0; i < 100; i++)
        {
            var path = new TaggedToolpath(new Point3D[] { new(300, 200, -6 - i * .001),
                new(300 + 1e-5 + i * 1e-9, 200, -6.35) }, [], 0, "final-carve", null, detectCurves: false);
            var limited = VCarveSmoothing.LimitGraph([path], grade);
            double actual = VCarveSmoothing.MaximumGrade(limited);
            Assert.True(actual <= grade + 1e-12, $"case={i} authoredGrade={grade:R} actualGrade={actual:R}");
        }
        Console.WriteLine($"[Plateau.GradeArithmetic] shortDeepChords=100 maximumGrade={grade:R} widenedAngle=False");
    }

    [Theory]
    [InlineData(.000001)]
    [InlineData(.01)]
    [InlineData(.1)]
    [InlineData(.3)]
    [InlineData(2)]
    public void RoundedFinishPlateauUsesTheSamePhysicalSupportAsFinishing(double delta)
    {
        const double floor = 3, grade = .5773502691896257, nose = 1.5875;
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, nose, nose),
            new(CarvingCutterSectionKind.Linear, nose, nose, 10)], 10);
        var original = new TaggedToolpath([new(0, 0, -floor), new(1, 0, -floor)], [], 0, "final-carve", null, detectCurves: false);
        var smooth = original with { Points = [new(0, 0, -floor + delta), new(1, 0, -floor + delta)] };
        IReadOnlyList<PointD>[] boundary = [[new(-20, -20), new(20, -20), new(20, 20), new(-20, 20)]];
        var mask = SmoothRoughingPlateau.Build([original], boundary, floor, grade, default, [smooth], profile);
        var field = new VCarveSmoothing.FloorField([smooth], profile, grade, floor);
        // Independent sphere/cone tangency, including the shallow spherical cap.
        double transition = nose * (1 - 1 / Math.Sqrt(1 + grade * grade));
        double radius = delta < transition ? Math.Sqrt(delta * (2 * nose - delta))
            : (delta + nose * (Math.Sqrt(1 + grade * grade) - 1)) / grade;
        Assert.True(PolygonOps.Contains(mask, .5, Math.Max(0, radius - .005)));
        Assert.False(PolygonOps.Contains(mask, .5, radius + .01));
        Assert.InRange(field.Depth(.5, radius), floor - .00001, floor);
        if (radius > .01) Assert.True(field.Depth(.5, radius - .01) < floor - .000001);
    }

    [Fact]
    public void SharedSourcePlateauBoundMatchesDetailedBasinsWithoutRepeatingTheWatershed()
    {
        var preparation = PocketToolpaths.AnalyzeSmoothRoughing([new(Box(30)), new(Box(4))],
            Flat(.2), new DepthSchedule(3, .5), Options, BoostVoronoiProvider.CreateDefault()).Preparation;
        var fast = preparation.AnalyzePocketPlateau(3, 15);
        var detailed = preparation.AnalyzePocketPlateau(3, 15, retainBasinDiagnostics: true);
        Assert.Equal(detailed.EntireProtectedSourceRegionIndices.Order(), fast.EntireProtectedSourceRegionIndices.Order());
        Assert.Equal(detailed.ProtectedBoundary.SelectMany(r => r), fast.ProtectedBoundary.SelectMany(r => r));
        Assert.Same(preparation, fast.Preparation);
        var positive = preparation.AnalyzePocketPlateau(3, 15, .3);
        var negative = preparation.AnalyzePocketPlateau(3, 15, -.3);
        Assert.Equal(fast.ProtectedArea, positive.ProtectedArea);
        Assert.True(negative.ProtectedArea < fast.ProtectedArea);
        Assert.Same(preparation, positive.Preparation);
    }



    [Fact]
    public void ZeroAllowanceSkipsOnlyCompleteComponentsThatCannotContainTheFlatTip()
    {
        var provider = new CountingProvider();
        MillingRegion[] source = [new(Box(30)), new(Box(1).Select(p => new PointD(p.x + 100, p.y)).ToArray())];
        var result = PocketToolpaths.AnalyzeSmoothRoughing(source, Flat(3), new DepthSchedule(3, 1),
            Options, provider);
        Assert.Equal(1, provider.Calls);
        Assert.Equal(2, result.Preparation.OriginalRegions.Count);
        Assert.Contains(1, result.EntireProtectedSourceRegionIndices);
        Assert.True(PolygonOps.Contains(result.ProtectedBoundary, 100.5, .5));
    }

    private sealed class CountingProvider : IMedialAxisProvider
    {
        public int Calls;
        private readonly IMedialAxisProvider inner = BoostVoronoiProvider.CreateDefault();
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle = 3 * Math.PI / 4, bool useBigIntegers = true)
        {
            Calls++;
            Assert.True(boundary.Max(p => p.x) - boundary.Min(p => p.x) > 6 * PathUtils.Scale,
                "An unreachable microscopic component entered the native graph provider.");
            return inner.ConstructMedialAxis(boundary, holes, tolerance, maxRadius, filteringAngle, useBigIntegers);
        }
    }

    private static PointD[] Box(double size) => [new(0, 0), new(size, 0), new(size, size), new(0, size)];
    private static ToolGeometry Flat(double radius) => new(radius) { VCarveProfile = new(
        [new(CarvingCutterSectionKind.Linear, radius, radius, 10)], 10) };
    private static PocketOptions Options => new() { SmoothRoughing = true, SmoothRampAngleDegrees = 15,
        DetectCurves = false, CalculateSweptArea = false, SpiralChaining = false, Ramp = RampSettings.FromRatio(0) };

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousCrestWithoutRoomForAFlatPlateauEmitsNoHole(bool scanline)
    {
        MillingRegion[] source = [new(Box(4))];
        var tool = Flat(.2); var depth = new DepthSchedule(.65, .3);
        var analysis = PocketToolpaths.AnalyzeSmoothRoughing(source, tool, depth, Options, BoostVoronoiProvider.CreateDefault());
        double attainableTipDepth = (2 - tool.Radius) * Math.Tan(15 * Math.PI / 180);
        Assert.All(analysis.Basins, basin => {
            Assert.Equal(attainableTipDepth, basin.SmoothedPeakDepth, 8);
            Assert.True(basin.OmitRoughing);
        });
        var options = Options with { ProtectedBoundary = analysis.ProtectedBoundary };
        var plan = scanline ? ScanlinePocketToolpaths.Generate(source, tool, depth, options)
            : PocketToolpaths.Generate(source, tool, depth, options);
        Assert.Empty(plan.Toolpaths);
        Console.WriteLine($"[Plateau.Regression] scanline={scanline} floor={depth.Depth:R} attainableTipDepth={attainableTipDepth:R} fullCutterFits=False paths={plan.Toolpaths.Count}");
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, -.1)]
    [InlineData(true, -.1)]
    [InlineData(false, .3)]
    [InlineData(true, .3)]
    public void EveryCuttingChordHonorsTheSignedPlateauAllowance(bool scanline, double leave)
    {
        MillingRegion[] source = [new(Box(15))];
        var tool = Flat(.2); var depth = new DepthSchedule(1.5, .5);
        var options = Options with { StockToLeave = leave };
        var analysis = PocketToolpaths.AnalyzeSmoothRoughing(source, tool, depth, options, BoostVoronoiProvider.CreateDefault());
        var anchors = analysis.Preparation.Regions.SelectMany(r => r.OriginalMedialPaths)
            .SelectMany(p => p.Points).Where(p => p.Z == 0).Distinct().ToArray();
        Assert.NotEmpty(anchors);
        options = options with { ProtectedBoundary = analysis.ProtectedBoundary };
        var plan = scanline ? ScanlinePocketToolpaths.Generate(source, tool, depth, options)
            : PocketToolpaths.Generate(source, tool, depth, options);
        Assert.NotEmpty(plan.Toolpaths);
        double required = tool.Radius + depth.Depth / Math.Tan(15 * Math.PI / 180) + leave;
        double minimum = double.PositiveInfinity;
        int chords = 0;
        foreach (var path in plan.Toolpaths)
        for (int i = 1; i < path.Points.Count; i++)
        {
            var a = path.Points[i - 1]; var b = path.Points[i];
            if (Math.Min(a.Z, b.Z) >= 0) continue;
            chords++;
            foreach (var q in anchors)
            {
                // Independent analytic distance to the complete chord, not
                // endpoint membership in the new polygon construction.
                double dx = b.X - a.X, dy = b.Y - a.Y, length2 = dx * dx + dy * dy;
                double t = length2 == 0 ? 0 : Math.Clamp(((q.X - a.X) * dx + (q.Y - a.Y) * dy) / length2, 0, 1);
                double distance = Math.Sqrt(Math.Pow(a.X + t * dx - q.X, 2) + Math.Pow(a.Y + t * dy - q.Y, 2));
                minimum = Math.Min(minimum, distance);
                Assert.True(distance >= required - 1e-6,
                    $"scanline={scanline} leave={leave:R} chord={i} anchor=({q.X:R},{q.Y:R}) distance={distance:R} required={required:R}");
            }
        }
        Assert.True(chords > 0);
        Console.WriteLine($"[Plateau.Chords] scanline={scanline} leave={leave:R} chords={chords} minimum={minimum:R} required={required:R}");
    }

    [Fact]
    public void ContinuousRampPlateauMatchesIndependentAnalyticConeMinimaOnASharedGraph()
    {
        double[] depths = [0, 4, .2, 3, 4, 3, .4, 4, .1];
        var source = new List<TaggedToolpath>();
        Point3D Node(int x, int y) => new(x, y, -depths[3 * y + x]);
        void Edge(Point3D a, Point3D b) => source.Add(new([a, b], [], 0, "final-carve", null, detectCurves: false));
        for (int y = 0; y < 3; y++) for (int x = 0; x < 3; x++)
        {
            if (x < 2) Edge(Node(x, y), Node(x + 1, y));
            if (y < 2) Edge(Node(x, y), Node(x, y + 1));
        }
        const double grade = .7, floor = 2.5;
        var smooth = VCarveSmoothing.LimitGraph(source, grade);
        var floorSources = smooth.Where((path, i) => path.Points[0].Z > source[i].Points[0].Z + 1e-10
            || path.Points[1].Z > source[i].Points[1].Z + 1e-10).ToArray();
        Assert.NotEmpty(floorSources);
        IReadOnlyList<PointD>[] boundary = [[new(-4, -4), new(6, -4), new(6, 6), new(-4, 6)]];
        var mask = SmoothRoughingPlateau.Build(source, boundary, floor, grade, default, smooth);
        int protectedPoints = 0, flatPoints = 0;
        for (double x = -3.9; x < 6; x += .13)
        for (double y = -3.9; y < 6; y += .13)
        {
            double minimum = double.PositiveInfinity;
            foreach (var path in floorSources)
            {
                var a = path.Points[0]; var b = path.Points[1];
                double dx = b.X - a.X, dy = b.Y - a.Y, length = Math.Sqrt(dx * dx + dy * dy);
                double along = ((x - a.X) * dx + (y - a.Y) * dy) / length;
                double across = Math.Abs((x - a.X) * dy - (y - a.Y) * dx) / length;
                double slope = (a.Z - b.Z) / length;
                double at = slope >= grade ? 0 : slope <= -grade ? length
                    : Math.Clamp(along - slope * across / Math.Sqrt(grade * grade - slope * slope), 0, length);
                minimum = Math.Min(minimum, -a.Z + slope * at + grade * Math.Sqrt(Math.Pow(at - along, 2) + across * across));
            }
            if (minimum < floor - .001)
            { protectedPoints++; Assert.True(PolygonOps.Contains(mask, x, y), $"Missing analytic ramp at ({x:R},{y:R}), depth={minimum:R}"); }
            else if (minimum > floor + .02)
            { flatPoints++; Assert.False(PolygonOps.Contains(mask, x, y), $"Erased proven plateau at ({x:R},{y:R}), depth={minimum:R}"); }
        }
        Assert.True(protectedPoints > 500 && flatPoints > 500);
    }

    [Fact]
    public void UnchangedShallowWallDetailDoesNotEraseAnUnrelatedReachableFloor()
    {
        TaggedToolpath Path(params Point3D[] points) => new(points, [], 0, "final-carve", null, detectCurves: false);
        TaggedToolpath[] original = [Path(new(0, 0, -.1), new(0, 1, -.2)), Path(new(20, 0, -3), new(20, 1, -3))];
        TaggedToolpath[] smooth = [original[0], Path(new(20, 0, -1), new(20, 1, -1.5))];
        IReadOnlyList<PointD>[] boundary = [[new(-5, -10), new(30, -10), new(30, 10), new(-5, 10)]];
        var mask = SmoothRoughingPlateau.Build(original, boundary, 3, .5, default, smooth);
        // The former all-shallow-anchor rule excluded a ~5.8mm disk here,
        // despite smoothing having made no depth change on the nearby wall.
        Assert.False(PolygonOps.Contains(mask, 2, 0));
        Assert.True(PolygonOps.Contains(mask, 20, 0));
        Assert.True(PolygonOps.Contains(mask, 20, 2));
        Assert.Empty(SmoothRoughingPlateau.Build(original, boundary, 3, .5, default, original));
    }

    [Fact]
    public void EmptyOriginalGraphCannotAuthorizeAFlatFloorCut()
    {
        var result = PocketToolpaths.AnalyzeSmoothRoughing([new(Box(10))], Flat(.2),
            new DepthSchedule(1, .5), Options, new EmptyProvider());
        Assert.Equal(100, result.ProtectedArea, 6);
        Assert.Contains(0, result.EntireProtectedSourceRegionIndices);
        Assert.All(result.Basins, basin => Assert.Contains("empty-original-graph", basin.Reason));
    }

    private sealed class EmptyProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle = 3 * Math.PI / 4, bool useBigIntegers = true) => [];
    }

    [Fact]
    public void BoundedLeafUnionsPreserveEveryOriginalFlatSpanCapsule()
    {
        var paths = Enumerable.Range(0, 200).Select(i => new TaggedToolpath(
            new Point3D[] { new(i * .1, 0, 0), new(i * .1, .1, 0) }, [], 0, "final-carve", null, detectCurves: false)).ToArray();
        IReadOnlyList<PointD>[] boundary = [[new(-5, -5), new(25, -5), new(25, 5), new(-5, 5)]];
        var mask = SmoothRoughingPlateau.Build(paths, boundary, 1, 1, default);
        foreach (var path in paths)
        foreach (var center in path.Points)
        for (int angle = 0; angle < 64; angle++)
        {
            double x = center.X + .999 * Math.Cos(angle * Math.PI / 32);
            double y = center.Y + .999 * Math.Sin(angle * Math.PI / 32);
            Assert.True(PolygonOps.Contains(mask, x, y), $"Lost original flat capsule at ({x:R},{y:R}).");
        }
        Assert.False(PolygonOps.Contains(mask, 24, 0));
    }

    [Fact]
    public void OppositeIncomingRampsPreserveTheLowerInterpolatedBridgeFloor()
    {
        TaggedToolpath Path(params Point3D[] points) => new(points, [], 0, "final-carve", null, detectCurves: false);
        TaggedToolpath[] original = [Path(new(0, 0, -3), new(10, 0, -3)),
            Path(new(0, 1, 0), new(0, 0, -3)), Path(new(10, 1, 0), new(10, 0, -3))];
        var smooth = VCarveSmoothing.LimitGraph(original, 1);
        IReadOnlyList<PointD>[] boundary = [[new(-5, -5), new(15, -5), new(15, 5), new(-5, 5)]];
        var originalOnly = SmoothRoughingPlateau.Build(original, boundary, 3, 1, default);
        var complete = SmoothRoughingPlateau.Build(original, boundary, 3, 1, default, smooth);
        Assert.False(PolygonOps.Contains(originalOnly, 5, 0));
        Assert.True(PolygonOps.Contains(complete, 5, 0));
        Assert.True(PolygonOps.Contains(complete, 5, 1.99));
        Assert.False(PolygonOps.Contains(complete, 5, 3));
    }

    [Fact]
    public void VariableDepthSpanExcludesInteriorDiscsAndPreservesSignedSourceHoles()
    {
        var line = new TaggedToolpath([new(0, 0, 0), new(10, 0, -2)], [], 0, "final-carve", null, detectCurves: false);
        IReadOnlyList<PointD>[] boundary = [[new(-10, -10), new(20, -10), new(20, 10), new(-10, 10)],
            [new(4, -1), new(4, 1), new(6, 1), new(6, -1)]];
        var mask = SmoothRoughingPlateau.Build([line], boundary, 3, .5, default);
        for (int i = 0; i <= 100; i++)
        {
            double t = i / 100d, x = 10 * t, radius = (3 - 2 * t) / .5;
            for (int a = 0; a < 32; a++)
            {
                double px = x + radius * .99 * Math.Cos(a * Math.PI / 16);
                double py = radius * .99 * Math.Sin(a * Math.PI / 16);
                if (!PolygonOps.Contains(boundary, px, py)) continue;
                Assert.True(PolygonOps.Contains(mask, px, py), $"Missing continuous floor exclusion at ({px:R},{py:R}), t={t:R}.");
            }
        }
        Assert.False(PolygonOps.Contains(mask, 5, 0));
        Assert.False(PolygonOps.Contains(mask, 19, 0));
    }
}
