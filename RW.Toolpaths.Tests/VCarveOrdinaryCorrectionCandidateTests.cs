using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveOrdinaryCorrectionCandidateTests
{
    [Fact]
    public void DefaultFalseRetainsOrderedPhysicalCollectorOutput()
    {
        Assert.False(new VCarveOptions().UseOrdinaryCorrectionCandidates);
        var expected = Run(Options() with { UseOrdinaryCorrectionCandidates = false }, null);
        var actual = Run(Options(), null);
        EqualPaths(expected, actual);
    }

    [Fact]
    public void OrdinaryNominationStillCompletesOriginalCoverageWallsFloorAndGrade()
    {
        var logs = new List<string>();
        var result = Run(Options() with { UseOrdinaryCorrectionCandidates = true }, logs.Add);
        Assert.Contains(logs, line => line.Contains("stage=smooth-candidates event=begin round=0")
            && line.Contains("ordinaryCollector=True") && line.Contains("preserveTerminalPaths=False")
            && line.Contains("candidateStockAuthority=False"));
        Assert.Contains(logs, line => line.Contains("stage=smooth-concentric event=workers-complete")
            && line.Contains("ordinaryCollector=True") && line.Contains("nominationRings=")
            && line.Contains("nominationPoints="));
        Assert.Contains(logs, line => line.Contains("stage=smoothing event=complete") && line.Contains("widthVerified=True"));
        Assert.Contains(logs, line => line.Contains("stage=correction-arc event=complete") && line.Contains("fallback=False"));
        var source = Source(); var boundary = Boundary(); var tool = Tool();
        var commands = VCarveSmoothing.CommandPaths(result, 0);
        var originalUpper = VCarvePhysicalSweep.Build(VCarveSmoothing.CommandPaths(source, 0),
            tool, 3, 0, .00075, upperBound: true);
        var originalTarget = PolygonOps.Intersect(originalUpper, boundary);
        var profile = VCarvePhysicalSweep.ProfileFor(tool, 3);
        var target = PolygonOps.Intersect(originalTarget,
            VCarveSmoothing.ReachableReferenceUpper(originalTarget, profile.PhysicalRadiusAtDepth(.001)));
        var lower = VCarvePhysicalSweep.Build(commands, tool, 3, 0, .00075);
        Assert.Empty(VCarveSmoothing.ResidualCentersUpper(PolygonOps.Difference(target, lower), .004));
        double grade = Math.Tan(Math.PI / 12);
        Assert.True(VCarveSmoothing.MaximumGrade(result) <= grade + 1e-8);
        var limited = VCarveSmoothing.LimitGraph(source, grade);
        var originalFloor = new VCarveSmoothing.FloorField(limited, profile, grade, 3);
        double xyConversion = 0, zConversion = 0;
        foreach (var path in result)
        foreach (var a in path.Points)
        {
            static double F3(float value) => double.Parse(value.ToString("F3", System.Globalization.CultureInfo.InvariantCulture),
                System.Globalization.CultureInfo.InvariantCulture);
            var b = new Point3D(F3((float)a.X), F3((float)a.Y), F3((float)a.Z));
            xyConversion = Math.Max(xyConversion, Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2)));
            zConversion = Math.Max(zConversion, Math.Abs(a.Z - b.Z));
        }
        double derivedFloorBound = zConversion + grade * xyConversion + .0002 + 1e-8;
        foreach (var path in commands)
        for (int end = 1; end < path.Points.Count; end++)
        for (int sample = 0; sample <= 8; sample++)
        {
            double t = sample / 8d;
            var a = path.Points[end - 1]; var b = path.Points[end];
            double x = a.X + (b.X - a.X) * t, y = a.Y + (b.Y - a.Y) * t;
            double depth = -(a.Z + (b.Z - a.Z) * t);
            double clearance = Math.Min(Math.Min(x, 10 - x), Math.Min(y, 4 - y));
            Assert.True(profile.PhysicalRadiusAtDepth(Math.Max(0, depth)) <= clearance + 1e-9);
            // Reuse only the existing audit's derived command conversion bound;
            // candidate mode supplies no additional floor or depth allowance.
            Assert.True(depth <= originalFloor.Depth(x, y) + derivedFloorBound);
        }
    }

    [Fact]
    public void ClassifiedFailureRetriesOriginalCollectorAndReferenceArcTogether()
    {
        var reference = Run(Options(), null);
        var logs = new List<string>(); bool injected = false;
        var actual = Run(Options() with { UseOrdinaryCorrectionCandidates = true, CorrectionArcTolerance = .00025 }, line =>
        {
            logs.Add(line);
            if (injected || !line.Contains("stage=smooth-concentric event=partition")) return;
            injected = true;
            var error = new InvalidOperationException("inconclusive candidate completion");
            error.Data["SmoothVCarvingGeometry"] = new { Round = 0 };
            throw error;
        });
        Assert.True(injected);
        Assert.Contains(logs, line => line.Contains("stage=correction-arc event=fallback")
            && line.Contains("usedOrdinaryCandidates=False")
            && line.Contains($"usedArc={VCarveSmoothing.ReferenceCorrectionArcTolerance:R}"));
        EqualPaths(reference, actual);
    }

    [Fact]
    public void UnclassifiedFailureAndCancellationNeverRetryOriginalCollector()
    {
        var logs = new List<string>();
        var failure = new InvalidOperationException("unverified terminal ordinals");
        var actual = Assert.Throws<InvalidOperationException>(() => Run(
            Options() with { UseOrdinaryCorrectionCandidates = true }, line =>
            {
                logs.Add(line);
                if (line.Contains("stage=smooth-concentric event=partition")) throw failure;
            }));
        Assert.Same(failure, actual);
        using var cancellation = new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(() => Run(Options() with { UseOrdinaryCorrectionCandidates = true }, line =>
        {
            logs.Add(line);
            if (line.Contains("stage=smooth-concentric event=partition")) cancellation.Cancel();
        }, cancellation.Token));
        Assert.DoesNotContain(logs, line => line.Contains("stage=correction-arc event=fallback"));
    }

    [Fact]
    public void RecoveryCannotReuseCoarserCachedInsetAtSamePhysicalRadius()
    {
        var boundary = new VCarveSmoothing.CorrectionBoundary(Boundary());
        IReadOnlyList<PointD> error = [new(1, 1), new(2, 1), new(2, 2), new(1, 2)];
        var whole = boundary.Crop(error, 10);
        var reference = VCarveSmoothing.CorrectionCenters(whole, .25);
        var coarse = VCarveSmoothing.CorrectionCenters(whole, .25, arcTolerance: .00025);
        boundary.RememberWholeSourceInset(whole, .25, coarse, .00025);
        Assert.False(boundary.TryGetWholeSourceInset(.25, out _));
        boundary.RememberWholeSourceInset(whole, .25, reference);
        Assert.True(boundary.TryGetWholeSourceInset(.25, out var retainedReference));
        Assert.Same(reference, retainedReference);
        Assert.True(boundary.TryGetWholeSourceInset(.25, out var retainedCoarse, .00025));
        Assert.Same(coarse, retainedCoarse);
    }

    private static VCarveOptions Options() => new() { SmoothVCarving = true, SmoothRampAngleDegrees = 15,
        IncludeInteriorFill = false, DetectCurves = false };
    private static IReadOnlyList<TaggedToolpath> Source() => [new TaggedToolpath(
        [new(1, 2, -.2), new(4, 2, -3), new(8, 2, -3)], Array.Empty<ToolpathSpan>(),
        0, "final-carve", null, visitIdentity: 17, detectCurves: false)];
    private static IReadOnlyList<IReadOnlyList<PointD>> Boundary() =>
        [new PointD[] { new(0, 0), new(10, 0), new(10, 4), new(0, 4) }];
    private static ToolGeometry Tool() => new(5) { BottomRadius = 0, TopRadius = 5, ConeLength = 10,
        TipAngleRadians = 2 * Math.Atan(.5) };
    private static List<TaggedToolpath> Run(VCarveOptions options, Action<string>? diagnostics,
        CancellationToken token = default) => VCarveSmoothing.Apply(Source(), Boundary(), Tool(),
            new DepthSchedule(3, 1), options, token, diagnostics, correctionWorkers: 1);
    private static void EqualPaths(IReadOnlyList<TaggedToolpath> expected, IReadOnlyList<TaggedToolpath> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
        {
            Assert.Equal(expected[i].Points, actual[i].Points);
            Assert.Equal(expected[i].Moves, actual[i].Moves);
            Assert.Equal(expected[i].Spans, actual[i].Spans);
            Assert.Equal(expected[i].Category, actual[i].Category);
            Assert.Equal(expected[i].RegionIndex, actual[i].RegionIndex);
            Assert.Equal(expected[i].DepthPassIndex, actual[i].DepthPassIndex);
            Assert.Equal(expected[i].VisitIdentity, actual[i].VisitIdentity);
        }
    }
}
