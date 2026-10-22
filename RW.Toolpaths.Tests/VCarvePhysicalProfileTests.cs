using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarvePhysicalProfileTests
{
    private static VCarveCutterProfile BallTaper(double maximumDepth = 5) => new(
        [new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
         new(CarvingCutterSectionKind.Linear, 1, 3, 8)], maximumDepth);

    private static IReadOnlyList<IReadOnlyList<PointD>> Box(double size = 20) =>
        [new PointD[] { new(0, 0), new(size, 0), new(size, size), new(0, size) }];

    [Theory]
    [InlineData(0.2)]
    [InlineData(1.0)]
    [InlineData(1.5)]
    public void AnalyticalDepthUsesSphericalNoseThenTaperFromActualTip(double clearance)
    {
        var z = MedialAxisToolpaths.CreateAnalyticalZFunction(Box(), 2, 7, 0, 2,
            cutterProfile: BallTaper());
        double depth = clearance < 1 ? clearance * clearance / (1 + Math.Sqrt(1 - clearance * clearance))
            : 1 + (clearance - 1) / 0.25;
        Assert.Equal(-2 - depth, z(clearance, 10), 12);
    }

    [Theory]
    [InlineData(0.1)]
    [InlineData(2.0)]
    public void ClearingUsesNosePitchBoundedByEngagementAndPreservesPhysicalClearance(double depth)
    {
        var logs = new List<string>();
        var paths = MedialAxisToolpaths.GenerateClearingPassesTagged(Box(), 0, depth, 0, depth,
            stepOver: 0.4, detectCurves: false, diagnostic: logs.Add, cutterProfile: BallTaper());
        double radius = depth < 1 ? Math.Sqrt(depth * (2 - depth)) : 1 + 0.25 * (depth - 1);
        double pitch = 0.4 * Math.Min(2, 2 * radius);
        Assert.NotEmpty(paths);
        Assert.All(paths.SelectMany(path => path.Points), point =>
        {
            Assert.Equal(-depth, point.Z);
            double clearance = Math.Min(Math.Min(point.X, 20 - point.X), Math.Min(point.Y, 20 - point.Y));
            Assert.True(clearance >= radius - 0.0001, $"depth={depth:R}, radius={radius:R}, clearance={clearance:R}");
        });
        var xs = paths.SelectMany(path => path.Points).Select(point => point.X).ToArray();
        double firstInset = xs.Min();
        // The perimeter reserves integer/float geometry and F3 machine XY error.
        Assert.InRange(firstInset - radius, Math.Sqrt(2) * .0005, .0015);
        Assert.Contains(xs, x => Math.Abs(x - firstInset - pitch) < 0.0001);
        Assert.Contains(logs, line => line.Contains("stage=clearing-fill event=start")
            && line.Contains($"pitch={pitch:R}") && line.Contains("noseDiameter=2") && line.Contains("roundedTip=True"));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactGuardCoversOffCenterNoseAndTaperTransitions(bool reverse)
    {
        // The protected point is closest at one quarter of the chord, across both
        // the spherical section and taper. Endpoints are analytically correct.
        var boundary = new[] { (A: new PointD(1, 0.5), B: new PointD(1, 0.5)) };
        var profile = BallTaper();
        var path = new List<Point3D> { new(0, 0, 0), new(4, 0, 0) };
        if (reverse) path.Reverse();
        double Depth(double x) => ExpectedDepth(Math.Sqrt((x - 1) * (x - 1) + 0.25), 5);
        Assert.True((Depth(0) * 0.75 + Depth(4) * 0.25) - Depth(1) > 1);
        var refined = MedialAxisToolpaths.EnsureZFidelityFromBoundary(path,
            new MedialAxisToolpaths.SpatialSegmentIndex(boundary), 0, 0, 1, -5, 0.003, profile);
        Assert.InRange(refined.Count, 3, 300);
        foreach (var point in refined)
            Assert.Equal(-Depth(point.X), point.Z, 11);
        for (int i = 1; i < refined.Count; i++)
        for (int sample = 0; sample <= 1024; sample++)
        {
            double t = sample / 1024d;
            var a = refined[i - 1]; var b = refined[i];
            double overcut = -(a.Z + (b.Z - a.Z) * t) - Depth(a.X + (b.X - a.X) * t);
            Assert.True(overcut <= 0.003000001, $"overcut={overcut:R}, segment={i}, t={t:R}");
        }
    }

    [Fact]
    public void PhysicalSingleConePreservesLegacyClearingAndMedialGeometry()
    {
        const double bottom = 0.05, top = 3.175, height = 10;
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, bottom, top, height)], height);
        var provider = BoostVoronoiProvider.CreateDefault();
        List<TaggedToolpath> Generate(VCarveCutterProfile? physical) => MedialAxisToolpaths.GenerateVCarveTagged(
            provider, Box(), 0, 3, 2 * Math.Atan((top - bottom) / height), 2,
            bottomRadiusOverride: bottom, topRadiusOverride: top, coneLengthOverride: height,
            detectCurves: false, cutterProfile: physical);
        var legacy = Generate(null); var physical = Generate(profile);
        Assert.Equal(legacy.Count, physical.Count);
        for (int i = 0; i < legacy.Count; i++)
        {
            Assert.Equal(legacy[i].Category, physical[i].Category);
            Assert.Equal(legacy[i].DepthPassIndex, physical[i].DepthPassIndex);
            Assert.Equal(legacy[i].Points.Count, physical[i].Points.Count);
            for (int j = 0; j < legacy[i].Points.Count; j++)
            {
                Assert.Equal(legacy[i].Points[j].X, physical[i].Points[j].X, 10);
                Assert.Equal(legacy[i].Points[j].Y, physical[i].Points[j].Y, 10);
                Assert.Equal(legacy[i].Points[j].Z, physical[i].Points[j].Z, 10);
            }
        }
    }

    [Fact]
    public void RoundedToolAtItsDepthCapStillSeparatesBroadFlatCoreFromMedialDetail()
    {
        var messages = new List<string>();
        var paths = MedialAxisToolpaths.GenerateVCarveTagged(BoostVoronoiProvider.CreateDefault(),
            Box(), 0, 2, 0, 2, detectCurves: false, diagnostic: messages.Add, cutterProfile: BallTaper(2));
        Assert.Contains(paths, path => path.Category == "clearing");
        Assert.Contains(paths, path => path.Category == "final-carve" && path.Points.Any(point => point.Z > -1));
        Assert.Contains(messages, line => line.StartsWith("stage=trim-and-flat-fill event=complete")
            && !line.Contains("flatStrokes=0 "));
        Assert.All(paths.SelectMany(path => path.Points), point => Assert.InRange(point.Z, -2, 0));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NonlinearProfileFloorClippingRetainsFullCornerDetailWithoutFill(bool reverse)
    {
        var profile = new VCarveCutterProfile(
            [new(CarvingCutterSectionKind.Hemisphere, 0, 0.79, 0.79),
             new(CarvingCutterSectionKind.Linear, 0.79, 3, 20)], 20.79);
        const double endDepth = 1, radius0 = 0.2, radius1 = 1;
        double threshold = 0.79 + (endDepth - 0.79) * (3 - 0.79) / 20;
        var a = new MedialPoint(0, 0, -MedialAxisToolpaths.RadiusToClippingDepth(radius0,
            endDepth, 0, 1, profile));
        var b = new MedialPoint(10, 0, -MedialAxisToolpaths.RadiusToClippingDepth(radius1,
            endDepth, 0, 1, profile));
        if (reverse) (a, b) = (b, a);
        var flat = new List<(PointD A, PointD B)>();
        var trimmed = MedialAxisToolpaths.TrimSegment(new MedialSegment(a, b), -endDepth, flat);
        Assert.NotNull(trimmed);
        var crossing = reverse ? trimmed.Value.Point0 : trimmed.Value.Point1;
        Assert.Equal(10 * (threshold - radius0) / (radius1 - radius0), crossing.X, 12);
        Assert.Single(flat);

        var final = MedialAxisToolpaths.Generate(BoostVoronoiProvider.CreateDefault(), Box(10),
            0, 0.2, 0, 0.2, includeInteriorFill: false, cutterProfile: profile);
        Assert.NotNull(final);
        Assert.Contains(final.SelectMany(path => path), point => -point.Z > 0.19999);
        Assert.Contains(final.SelectMany(path => path), point => -point.Z < 0.01);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObservedSphericalEquatorCuspRefinesBeyondConeBudgetWithoutRelaxingTolerance(bool reverse)
    {
        // Actual failing subchord from the artwork. Its 0.148-micron XY span
        // straddles the sphere/taper join and carries a 12.89-micron depth change.
        var edge = (A: new PointD(164.20669555664062, 424.0229187011719),
            B: new PointD(178.15567016601562, 409.3109130859375));
        var witnessA = new Point3D(165.30959136117525, 424.00788708444645, 0);
        var witnessB = new Point3D(165.30944300862774, 424.0078890432305, 0);
        var profile = new VCarveCutterProfile(
            [new(CarvingCutterSectionKind.Hemisphere, 0, 0.79, 0.79),
             new(CarvingCutterSectionKind.Linear, 0.79, 3, 20)], 20.79);
        double Expected(double x, double y)
        {
            double ex = edge.B.x - edge.A.x, ey = edge.B.y - edge.A.y;
            double t = Math.Clamp(((x - edge.A.x) * ex + (y - edge.A.y) * ey) / (ex * ex + ey * ey), 0, 1);
            double dx = x - edge.A.x - t * ex, dy = y - edge.A.y - t * ey;
            double radius = Math.Sqrt(dx * dx + dy * dy);
            return Math.Min(6.35, radius < 0.79
                ? radius * radius / (0.79 + Math.Sqrt(0.79 * 0.79 - radius * radius))
                : 0.79 + (radius - 0.79) * 20 / (3 - 0.79));
        }
        var diagnostics = new List<string>();
        foreach (bool expand in new[] { false, true })
        {
            double dx = witnessA.X - witnessB.X, dy = witnessA.Y - witnessB.Y;
            var a = expand ? new Point3D(witnessA.X + 10000 * dx, witnessA.Y + 10000 * dy, 0) : witnessA;
            var b = expand ? new Point3D(witnessA.X - 6000 * dx, witnessA.Y - 6000 * dy, 0) : witnessB;
            if (reverse) (a, b) = (b, a);
            var result = MedialAxisToolpaths.EnsureZFidelityFromBoundary([a, b],
                new MedialAxisToolpaths.SpatialSegmentIndex([edge]), 0, 0, 1, -6.35, 0.003,
                profile, diagnostics.Add);
            Assert.InRange(result.Count, 3, 500);
            for (int i = 1; i < result.Count; i++)
            for (int sample = 0; sample <= 1024; sample++)
            {
                double t = sample / 1024d;
                var p = result[i - 1]; var q = result[i];
                double x = p.X + (q.X - p.X) * t, y = p.Y + (q.Y - p.Y) * t;
                double overcut = -(p.Z + (q.Z - p.Z) * t) - Expected(x, y);
                Assert.True(overcut <= 0.003000001, $"overcut={overcut:R}, segment={i}, t={t:R}, expanded={expand}");
            }
        }
        Assert.Contains(diagnostics, line => line.Contains("event=extended-refinement"));
    }

    [Fact]
    public void ClearingChecksCancellationBeforeOffsetWork()
    {
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => MedialAxisToolpaths.GenerateClearingPassesTagged(
            Box(), 0, 2, 0, 1, cutterProfile: BallTaper(), cancellationToken: cancellation.Token));
    }

    private static double ExpectedDepth(double clearance, double cap) => Math.Min(cap,
        clearance < 1 ? clearance * clearance / (1 + Math.Sqrt(1 - clearance * clearance))
            : 1 + (clearance - 1) / 0.25);
}
