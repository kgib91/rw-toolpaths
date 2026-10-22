using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarvePhysicalSweepTests
{
    private const double Tolerance = .002;

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConsolidatedSphereChordUsesOnlyPhysicalWitnessesAndKeepsOriginalUpperTarget(bool reverse)
    {
        var profile = Profile(true);
        var a = new Point3D(.1234567, -.3212345, -.001);
        var b = new Point3D(14.234567, 3.123456, -.9);
        if (reverse) (a, b) = (b, a);
        var path = Path(a, b);
        var literal = VCarvePhysicalSweep.Build([path], profile, 0, .00075,
            consolidateHemisphereChords: false);
        var candidate = VCarvePhysicalSweep.Build([path], profile, 0, .00075,
            consolidateHemisphereChords: true);
        var originalUpper = VCarvePhysicalSweep.Build([path], profile, 0, .00075,
            upperBound: true, consolidateHemisphereChords: false);
        var candidateUpper = VCarvePhysicalSweep.Build([path], profile, 0, .00075,
            upperBound: true, consolidateHemisphereChords: true);
        Assert.Equal(originalUpper.Count, candidateUpper.Count);
        for (int i = 0; i < originalUpper.Count; i++) Assert.Equal(originalUpper[i], candidateUpper[i]);
        Assert.True(Area(candidate) >= Area(literal) - .0001);
        foreach (var polygon in candidate)
        for (int i = 0; i < polygon.Count; i++)
        for (int part = 0; part <= 3; part++)
        {
            var first = polygon[i]; var last = polygon[(i + 1) % polygon.Count];
            var q = new PointD(first.x + (last.x - first.x) * part / 3,
                first.y + (last.y - first.y) * part / 3);
            Assert.True(PhysicalContains(q, a, b, profile), $"Consolidated lower sweep overstates cut at {q.x:R},{q.y:R}.");
        }
        Assert.Empty(Clipper.Difference(PathUtils.ToClipper(candidate), PathUtils.ToClipper(originalUpper), FillRule.NonZero));
        Console.WriteLine($"[VCarve.ConsolidatedSphereSweep.Test] reverse={reverse} "
            + $"referenceLowerArea={Area(literal):R} candidateLowerArea={Area(candidate):R} originalUpperUnchanged=True");
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void BoundsEncloseActualVariableDepthSweepsAcrossNoseAndTaper(bool rounded, bool reverse)
    {
        var profile = Profile(rounded);
        var a = new Point3D(.1234567, -.3212345, 0);
        var b = new Point3D(12.234561, 1.723498, -3.7);
        if (reverse) (a, b) = (b, a);
        var path = Path(a, b);
        var lower = VCarvePhysicalSweep.Build([path], profile, 0, Tolerance);
        var upper = VCarvePhysicalSweep.Build([path], profile, 0, Tolerance, upperBound: true);
        var upperInteger = PathUtils.ToClipper(upper);
        Assert.NotEmpty(lower);
        Assert.True(Area(upper) > Area(lower));
        foreach (var polygon in lower)
        for (int i = 0; i < polygon.Count; i++)
        for (int part = 0; part <= 3; part++)
        {
            var first = polygon[i]; var last = polygon[(i + 1) % polygon.Count];
            var q = new PointD(first.x + (last.x - first.x) * part / 3,
                first.y + (last.y - first.y) * part / 3);
            Assert.True(PhysicalContains(q, a, b, profile), $"Lower footprint overstates cut at {q.x:R},{q.y:R}.");
        }
        for (int sample = 0; sample <= 160; sample++)
        {
            double t = sample / 160d;
            double x = a.X + (b.X - a.X) * t, y = a.Y + (b.Y - a.Y) * t;
            double depth = -(a.Z + (b.Z - a.Z) * t);
            if (depth <= 0) continue;
            double radius = profile.PhysicalRadiusAtDepth(depth);
            for (int angle = 0; angle < 96; angle++)
            {
                double theta = angle * 2 * Math.PI / 96;
                Assert.True(Contains(upperInteger, new PointD(x + radius * Math.Cos(theta), y + radius * Math.Sin(theta))),
                    $"Upper footprint misses physical boundary: t={t:R}, angle={theta:R}, radius={radius:R}.");
            }
        }
        // This also detects accidentally replacing adaptive hulls with the full
        // nominal tool-radius buffer: a 14 mm trace has a small uncertainty band.
        Assert.InRange(Area(upper) - Area(lower), 0, .2);
    }

    [Fact]
    public void PointedConeStartingOffGridRetainsWholeTaperInsteadOfOnlyEndDisc()
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 2, 5)], 5);
        var a = new Point3D(.1234567, .1234567, 0);
        var b = new Point3D(20.1234567, .1234567, -2.5);
        var footprint = VCarvePhysicalSweep.Build([Path(a, b)], profile, 0, Tolerance);
        Assert.True(Contains(footprint, new(5.1234567, .3)));
        Assert.True(Contains(footprint, new(10.1234567, .55)));
        foreach (var point in footprint.SelectMany(polygon => polygon))
            Assert.True(PhysicalContains(point, a, b, profile), $"Pointed lower sweep overcuts at {point.x:R},{point.y:R}.");
    }

    [Fact]
    public void ShallowBallSweepUsesEngagedRadiusAndSuppliedSurfaceCoordinates()
    {
        var profile = Profile(true);
        var tool = new ToolGeometry(20) { VCarveProfile = profile };
        var shifted = Path(new Point3D(0, 0, 7.96), new Point3D(10, 0, 7.96));
        var lower = VCarvePhysicalSweep.Build([shifted], tool, 4, 8, Tolerance);
        var zero = VCarvePhysicalSweep.Build([Path(new(0, 0, -.04), new(10, 0, -.04))], profile, 0, Tolerance);
        Assert.Equal(Area(zero), Area(lower), 8);
        Assert.All(lower.SelectMany(p => p), p => Assert.InRange(Math.Abs(p.y), 0, .281));
        double radius = Math.Sqrt(.04 * (2 - .04));
        double expectedArea = 20 * radius + Math.PI * radius * radius;
        Assert.InRange(expectedArea - Area(lower), 0, .05);
    }

    [Fact]
    public void ClosedTracePreservesUncutHoleAndSeparateTracesDoNotGainConnectingChord()
    {
        var profile = Profile(false);
        var ring = Path(new(0, 0, -.4), new(10, 0, -.4), new(10, 10, -.4),
            new(0, 10, -.4), new(0, 0, -.4));
        var separate = Path(new(20, 0, -.4), new(20, 10, -.4));
        var footprint = VCarvePhysicalSweep.Build([ring, separate], profile, 0, Tolerance);
        Assert.False(Contains(footprint, new(5, 5)));
        Assert.False(Contains(footprint, new(15, 5)));
        Assert.True(Contains(footprint, new(5, 0)));
        Assert.True(Contains(footprint, new(20, 5)));
        Assert.Contains(footprint, polygon => Clipper.Area(new PathD(polygon)) < 0);
    }

    [Fact]
    public void SurfaceGrazesAndRetractsDoNotClaimCoverageButDescendingRampDoes()
    {
        var profile = Profile(false);
        var grazing = Path(new(0, 0, 0), new(10, 0, 0));
        var retract = new TaggedToolpath([new(20, 0, -.5), new(30, 0, -.5)],
            [new ToolpathSpan(0, 1, ToolpathSpanKind.Retract)], 0, "final-carve", null, detectCurves: false);
        Assert.Empty(VCarvePhysicalSweep.Build([grazing, retract], profile, 0, Tolerance));
        var ramp = new TaggedToolpath([new(40, 0, 1), new(50, 0, -1)],
            [new ToolpathSpan(0, 1, ToolpathSpanKind.Ramp)], 0, "final-carve", null, detectCurves: false);
        var footprint = VCarvePhysicalSweep.Build([ramp], profile, 0, Tolerance);
        Assert.False(Contains(footprint, new(42, 0)));
        Assert.True(Contains(footprint, new(49, 0)));
    }

    [Fact]
    public void PhysicalSweepDoesNotClampAlreadyCommandedDepthToNominalOperationCap()
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 4, 4)], 1);
        var footprint = VCarvePhysicalSweep.Build([Path(new(0, 0, -2), new(5, 0, -2))], profile, 0, Tolerance);
        Assert.True(Contains(footprint, new(2, 1.9)));
        Assert.False(Contains(footprint, new(2, 2.01)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void ConstantDepthRunBoundsMatchIndependentCapsuleUnionAndOriginalHullProof(int shape)
    {
        PointD[] centers = shape switch
        {
            0 => [new(.123456, .234567), new(5, 0), new(5, 4), new(1, 4), new(1, .3), new(4, .3)],
            1 => [new(0, 0), new(10, 0), new(10, 10), new(0, 10), new(0, 0)],
            2 => [new(0, 0), new(.2, 0), new(.2, .2), new(0, .2), new(0, 0)],
            _ => [new(0, 0), new(4, 4), new(0, 4), new(4, 0), new(0, 0)]
        };
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 3, 3)], 3);
        const double radius = 1;
        var path = Path(centers.Select(p => new Point3D(p.x, p.y, -radius)).ToArray());
        var lower = VCarvePhysicalSweep.Build([path], profile, 0, Tolerance);
        var upper = VCarvePhysicalSweep.Build([path], profile, 0, Tolerance, upperBound: true);
        var oldLower = VCarvePhysicalSweep.Build([path], profile, 0, Tolerance, accelerateConstantDepthRuns: false);
        var oldUpper = VCarvePhysicalSweep.Build([path], profile, 0, Tolerance, upperBound: true, accelerateConstantDepthRuns: false);
        Assert.Empty(Clipper.Difference(PathUtils.ToClipper(lower), PathUtils.ToClipper(oldUpper), FillRule.NonZero));
        Assert.Empty(Clipper.Difference(PathUtils.ToClipper(oldLower), PathUtils.ToClipper(upper), FillRule.NonZero));
        foreach (var polygon in lower)
        for (int i = 0; i < polygon.Count; i++)
        for (int part = 0; part <= 2; part++)
        {
            var a = polygon[i]; var b = polygon[(i + 1) % polygon.Count];
            var q = new PointD(a.x + (b.x - a.x) * part / 2, a.y + (b.y - a.y) * part / 2);
            double nearest = double.PositiveInfinity;
            for (int edge = 1; edge < centers.Length; edge++)
                nearest = Math.Min(nearest, PointSegmentDistance(q, centers[edge - 1], centers[edge]));
            Assert.True(nearest <= radius + 1e-8, $"Constant-run lower bound overstates cut: shape={shape}, distance={nearest:R}.");
        }
        var integerUpper = PathUtils.ToClipper(upper);
        for (int edge = 1; edge < centers.Length; edge++)
        for (int sample = 0; sample <= 12; sample++)
        for (int angle = 0; angle < 64; angle++)
        {
            var a = centers[edge - 1]; var b = centers[edge];
            double t = sample / 12d, theta = 2 * Math.PI * angle / 64;
            var q = new PointD(a.x + (b.x - a.x) * t + radius * Math.Cos(theta),
                a.y + (b.y - a.y) * t + radius * Math.Sin(theta));
            Assert.True(Contains(integerUpper, q), $"Constant-run upper bound misses physical capsule: shape={shape}, edge={edge}, t={t:R}.");
        }
        if (shape == 1) Assert.False(Contains(upper, new(5, 5)));
        if (shape == 2) Assert.True(Contains(lower, new(.1, .1)));
    }

    [Fact]
    public void ConstantDepthRunsStopAtRetractsInsteadOfInventingAConnectingSweep()
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 3, 3)], 3);
        Point3D[] points = [new(0, 0, -1), new(1, 0, -1), new(2, 0, -1), new(3, 0, -1),
            new(20, 0, -1), new(21, 0, -1), new(22, 0, -1), new(23, 0, -1)];
        var path = new TaggedToolpath(points,
            [new(0, 3, ToolpathSpanKind.Cut), new(3, 4, ToolpathSpanKind.Retract), new(4, 7, ToolpathSpanKind.Cut)],
            0, "clearing", 0, detectCurves: false);
        var sweep = VCarvePhysicalSweep.Build([path], profile, 0, Tolerance, upperBound: true);
        Assert.True(Contains(sweep, new(1, 0)));
        Assert.True(Contains(sweep, new(22, 0)));
        Assert.False(Contains(sweep, new(12, 0)));
    }

    [Fact]
    public void LongCollinearConstantRunKeepsEndCapsWithoutPerChordDiscExpansion()
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 3, 3)], 3);
        var path = Path(Enumerable.Range(0, 10001).Select(i => new Point3D(.123456 + i * .001, .234567, -1)).ToArray());
        var lower = VCarvePhysicalSweep.Build([path], profile, 0, Tolerance);
        var upper = VCarvePhysicalSweep.Build([path], profile, 0, Tolerance, upperBound: true);
        double exactArea = 20 + Math.PI;
        Assert.InRange(Area(lower), exactArea - .05, exactArea);
        Assert.InRange(Area(upper), exactArea, exactArea + .05);
        Assert.Single(lower);
        Assert.True(Contains(lower, new(-.8, .234567)));
        Assert.True(Contains(lower, new(11, .234567)));
    }

    [Fact]
    public void TwoUnionHierarchyLevelsRetainPhysicalBoundsAtNearlyTangentIntersections()
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 3, 3)], 3);
        const double radius = 1, length = .003;
        // More than 64*32 canonical groups exercises both permitted hierarchy
        // levels. Alternating lobes meet within one grid quantum; the offsets
        // deliberately change around non-lattice centers and batch boundaries.
        var centers = Enumerable.Range(0, 2050).Select(i => new PointD(
            .1234567 + (i % 2) * (2 * radius + length - .000025) + .000011 * Math.Sin(i * 1.7),
            .2345678 + .000037 * Math.Cos(i * 2.3))).ToArray();
        var paths = centers.Select(center => Path(Enumerable.Range(0, 4)
            .Select(i => new Point3D(center.x + i * length / 3, center.y, -radius)).ToArray())).ToArray();
        var lower = VCarvePhysicalSweep.Build(paths, profile, 0, .00075);
        var upper = PathUtils.ToClipper(VCarvePhysicalSweep.Build(paths, profile, 0, .00075, upperBound: true));
        Assert.NotEmpty(lower);
        foreach (var polygon in lower)
        for (int i = 0; i < polygon.Count; i++)
        for (int part = 0; part <= 2; part++)
        {
            var a = polygon[i]; var b = polygon[(i + 1) % polygon.Count];
            var q = new PointD(a.x + (b.x - a.x) * part / 2, a.y + (b.y - a.y) * part / 2);
            double nearestSquared = double.PositiveInfinity;
            foreach (var center in centers)
            {
                double dx = q.x - Math.Clamp(q.x, center.x, center.x + length), dy = q.y - center.y;
                nearestSquared = Math.Min(nearestSquared, dx * dx + dy * dy);
            }
            Assert.True(nearestSquared <= radius * radius + 1e-10,
                $"Hierarchical lower union overstates physical capsule at {q.x:R},{q.y:R}: distanceSquared={nearestSquared:R}.");
        }
        foreach (var center in centers)
        for (int endpoint = 0; endpoint <= 1; endpoint++)
        for (int angle = 0; angle < 32; angle++)
        {
            double theta = 2 * Math.PI * angle / 32;
            var q = new PointD(center.x + endpoint * length + radius * Math.Cos(theta), center.y + radius * Math.Sin(theta));
            Assert.True(Contains(upper, q),
                $"Hierarchical upper union misses physical capsule at {q.x:R},{q.y:R}.");
        }
    }

    [Fact]
    public void BooleanUpperEnclosureRemovesTheSavedMicroscopicFalseHoleWithoutFillingARealHole()
    {
        // Exact three-vertex hole from the saved 691-path source sweep. Its area
        // is only 5 integer-grid squares, but it falsely reduced clearance at an
        // original 1.92385 mm cutter disc from 1.924 mm to 0.759 mm.
        PointD[] falseHole = [new(278.0807800292969, 341.7298278808594),
            new(278.0809326171875, 341.7295227050781), new(278.0809631347656, 341.7294006347656)];
        PointD[] outer = [new(275, 338), new(282, 338), new(282, 346), new(275, 346)];
        PointD[] realHole = [new(281, 344), new(281, 344.5), new(281.5, 344.5), new(281.5, 344)];
        var raw = PathUtils.ToClipper([outer.ToList(), falseHole.ToList(), realHole.ToList()]);
        var upper = VCarvePhysicalSweep.BoundBooleanResult(raw, upperBound: true);
        var witness = new PointD(278.840, 341.731);
        const double radius = 1.9238515625;
        double clearance = double.PositiveInfinity;
        foreach (var ring in PathUtils.FromClipper(upper))
        for (int i = 0; i < ring.Count; i++)
            clearance = Math.Min(clearance, PointSegmentDistance(witness, ring[i], ring[(i + 1) % ring.Count]));
        Assert.True(clearance >= radius, $"Boolean enclosure retained a false internal boundary at clearance {clearance:R}.");
        for (int angle = 0; angle < 128; angle++)
        {
            double theta = 2 * Math.PI * angle / 128;
            Assert.True(Contains(upper, new(witness.x + radius * Math.Cos(theta), witness.y + radius * Math.Sin(theta))));
        }
        Assert.False(Contains(upper, new(281.25, 344.25)));
        Assert.Equal(2, upper.Count);
        Assert.All(PathUtils.FromClipper(upper).SelectMany(ring => ring), point =>
        {
            Assert.InRange(point.x, 275 - .00025, 282 + .00025);
            Assert.InRange(point.y, 338 - .00025, 346 + .00025);
        });
    }

    [Fact]
    public void BooleanLowerEnclosureDiscardsCollapsedGridIslandsAndPreservesProtectedHoles()
    {
        double quantum = 1 / PathUtils.Scale;
        PointD[] outer = [new(0, 0), new(2, 0), new(2, 2), new(0, 2)];
        PointD[] hole = [new(.5, .5), new(.5, 1.5), new(1.5, 1.5), new(1.5, .5)];
        PointD[] numericalIsland = [new(2 + quantum, 1), new(2 + 3 * quantum, 1), new(2 + quantum, 1 + 2 * quantum)];
        var raw = PathUtils.ToClipper([outer.ToList(), hole.ToList(), numericalIsland.ToList()]);
        var lower = VCarvePhysicalSweep.BoundBooleanResult(raw, upperBound: false);
        Assert.True(Contains(lower, new(.25, .25)));
        Assert.False(Contains(lower, new(1, 1)));
        Assert.Equal(2, lower.Count);
        Assert.All(PathUtils.FromClipper(lower).SelectMany(ring => ring), point =>
        {
            Assert.InRange(point.x, 0, 2);
            Assert.InRange(point.y, 0, 2);
        });
        Assert.Empty(Clipper.Difference(lower, PathUtils.ToClipper([outer.ToList(), hole.ToList()]), FillRule.NonZero));
    }

    private static double PointSegmentDistance(PointD point, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y, square = dx * dx + dy * dy;
        double t = square > 0 ? Math.Clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / square, 0, 1) : 0;
        return Math.Sqrt(Math.Pow(point.x - a.x - t * dx, 2) + Math.Pow(point.y - a.y - t * dy, 2));
    }

    private static VCarveCutterProfile Profile(bool rounded) => rounded
        ? new([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
            new(CarvingCutterSectionKind.Linear, 1, 2, 4)], 5)
        : new([new(CarvingCutterSectionKind.Linear, .2, 2, 5)], 5);

    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);

    private static double Area(List<List<PointD>> polygons)
        => polygons.Sum(p => Clipper.Area(new PathD(p)));

    private static bool Contains(List<List<PointD>> polygons, PointD q)
        => Contains(PathUtils.ToClipper(polygons), q);

    private static bool Contains(Paths64 polygons, PointD q)
    {
        var point = new Point64((long)Math.Round(q.x * PathUtils.Scale), (long)Math.Round(q.y * PathUtils.Scale));
        int winding = 0;
        foreach (var polygon in polygons)
        {
            var location = Clipper.PointInPolygon(point, polygon);
            if (location == PointInPolygonResult.IsOn) return true;
            if (location == PointInPolygonResult.IsInside) winding += Math.Sign(Clipper.Area(polygon));
        }
        return winding != 0;
    }

    // An independent membership oracle: radius squared minus XY distance squared
    // is quadratic in travel parameter inside each physical profile section.
    private static bool PhysicalContains(PointD point, Point3D a, Point3D b, VCarveCutterProfile profile)
    {
        var split = new List<double> { 0, 1 };
        double da = -a.Z, db = -b.Z, sectionHeight = 0;
        foreach (var section in profile.Sections)
        {
            sectionHeight += section.Height;
            double t = (sectionHeight - da) / (db - da);
            if (t > 0 && t < 1) split.Add(t);
        }
        split.Sort();
        double At(double t)
        {
            double dx = a.X + (b.X - a.X) * t - point.x;
            double dy = a.Y + (b.Y - a.Y) * t - point.y;
            double radius = profile.PhysicalRadiusAtDepth(da + (db - da) * t);
            return radius * radius - dx * dx - dy * dy;
        }
        for (int i = 1; i < split.Count; i++)
        {
            double first = At(split[i - 1]), last = At(split[i]), middle = At((split[i - 1] + split[i]) / 2);
            double quadratic = 2 * (first + last - 2 * middle), linear = last - first - quadratic;
            double maximum = Math.Max(first, last);
            if (quadratic < 0)
            {
                double t = Math.Clamp(-linear / (2 * quadratic), 0, 1);
                maximum = Math.Max(maximum, first + linear * t + quadratic * t * t);
            }
            if (maximum >= -1e-9) return true;
        }
        return false;
    }
}
