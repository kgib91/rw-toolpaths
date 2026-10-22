using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveCorrectionArcToleranceTests
{
    [Fact]
    public void DefaultAndExplicitReferenceKeepOrderedCorrectionGeometry()
    {
        Assert.Equal(1 / PathUtils.Scale, new VCarveOptions().CorrectionArcTolerance);
        var fixture = Fixture();
        var defaultLogs = new List<string>();
        var referenceLogs = new List<string>();
        var defaults = Run(fixture, Options(), defaultLogs.Add);
        var reference = Run(fixture, Options() with
            { CorrectionArcTolerance = VCarveSmoothing.ReferenceCorrectionArcTolerance }, referenceLogs.Add);
        EqualPaths(reference, defaults);
        Assert.Contains(defaults, path => path.Category == "clearing");
        Assert.Contains(defaultLogs, line => line.Contains("stage=correction-arc event=complete")
            && line.Contains("fallback=False"));
        Assert.Equal(referenceLogs.Where(line => line.StartsWith("stage=smooth-reference ")),
            defaultLogs.Where(line => line.StartsWith("stage=smooth-reference ")));
    }

    [Theory]
    [InlineData(.00025)]
    [InlineData(.001)]
    public void CoarserNominationCompletesOriginalTargetAndKeepsCommandedWallsAndGrade(double tolerance)
    {
        var fixture = Fixture();
        var originalPoints = fixture.Source.SelectMany(path => path.Points).ToArray();
        var logs = new List<string>();
        var result = Run(fixture, Options() with { CorrectionArcTolerance = tolerance }, logs.Add);
        Assert.Contains(result, path => path.Category == "clearing");
        Assert.Contains(logs, line => line.Contains("stage=smoothing event=complete")
            && line.Contains("widthVerified=True"));
        Assert.Contains(logs, line => line.Contains("stage=smooth-concentric event=inset-begin")
            && line.Contains($"nominationArcTolerance={tolerance:R}"));
        Assert.Contains(logs, line => line.Contains("stage=correction-arc event=complete")
            && line.Contains("fallback=False"));
        Assert.Equal(originalPoints, fixture.Source.SelectMany(path => path.Points));
        Assert.True(VCarveSmoothing.MaximumGrade(result) <= Math.Tan(Math.PI / 12) + 1e-8);

        // Reconstruct the target from original commanded input, independently
        // of the candidate's completion diagnostic or its generated contours.
        var commands = VCarveSmoothing.CommandPaths(result, 0);
        var originalUpper = VCarvePhysicalSweep.Build(VCarveSmoothing.CommandPaths(fixture.Source, 0),
            fixture.Tool, 3, 0, .00075, upperBound: true);
        var originalTarget = PolygonOps.Intersect(originalUpper, fixture.Boundary);
        var profile = VCarvePhysicalSweep.ProfileFor(fixture.Tool, 3);
        var target = PolygonOps.Intersect(originalTarget,
            VCarveSmoothing.ReachableReferenceUpper(originalTarget, profile.PhysicalRadiusAtDepth(.001)));
        var lower = VCarvePhysicalSweep.Build(commands, fixture.Tool, 3, 0, .00075);
        var missing = PolygonOps.Difference(target, lower);
        Assert.Empty(VCarveSmoothing.ResidualCentersUpper(missing, VCarveSmoothing.CoverageTolerance / 2));

        // The fixture is a rectangle, so physical cone-disc containment has an
        // independent closed-form check across every actual F3 command chord.
        foreach (var path in commands)
        for (int end = 1; end < path.Points.Count; end++)
        for (int sample = 0; sample <= 8; sample++)
        {
            double t = sample / 8d;
            var a = path.Points[end - 1]; var b = path.Points[end];
            double x = a.X + (b.X - a.X) * t, y = a.Y + (b.Y - a.Y) * t;
            double depth = -(a.Z + (b.Z - a.Z) * t);
            double radius = profile.PhysicalRadiusAtDepth(Math.Max(0, depth));
            double clearance = Math.Min(Math.Min(x, 10 - x), Math.Min(y, 4 - y));
            Assert.True(radius <= clearance + 1e-9,
                $"arc={tolerance:R} center=({x:R},{y:R}) depth={depth:R} radius={radius:R} clearance={clearance:R}");
        }
    }

    [Fact]
    public void CoarserRawInsetReducesNominationVerticesAndRetainsRelevantCenters()
    {
        IReadOnlyList<IReadOnlyList<PointD>> source = [new PointD[]
            { new(0, 0), new(10, 0), new(10, 10), new(6, 10), new(6, 4), new(4, 4), new(4, 10), new(0, 10) }];
        var reference = VCarveSmoothing.CorrectionCenters(source, .75);
        var coarse = VCarveSmoothing.CorrectionCenters(source, .75, arcTolerance: .00025);
        Assert.NotEmpty(reference);
        Assert.NotEmpty(coarse);
        Assert.True(coarse.Sum(ring => ring.Count) < reference.Sum(ring => ring.Count));
        Assert.True(PolygonOps.Contains(coarse, 2, 2));
        Assert.False(PolygonOps.Contains(coarse, 5, 8));
    }

    [Fact]
    public void ClassifiedCompletionFailureRetriesExactReferenceInputAndGeometry()
    {
        var fixture = Fixture();
        var reference = Run(fixture, Options(), null);
        var logs = new List<string>();
        bool failed = false;
        var result = Run(fixture, Options() with { CorrectionArcTolerance = .00025 }, line =>
        {
            logs.Add(line);
            if (failed || !line.Contains("stage=smooth-concentric event=partition")) return;
            failed = true;
            var failure = new InvalidOperationException("inconclusive completion witness");
            failure.Data["SmoothVCarvingGeometry"] = new { Round = 0 };
            throw failure;
        });
        Assert.True(failed);
        Assert.Single(logs, line => line.Contains("stage=correction-arc event=fallback"));
        EqualPaths(reference, result);
    }

    [Fact]
    public void UnclassifiedInvariantFailurePropagatesWithoutReferenceRetry()
    {
        var failure = new InvalidOperationException("unverified source ordinals");
        var logs = new List<string>();
        var actual = Assert.Throws<InvalidOperationException>(() => Run(Fixture(),
            Options() with { CorrectionArcTolerance = .00025 }, line =>
            {
                logs.Add(line);
                if (line.Contains("stage=smooth-concentric event=partition")) throw failure;
            }));
        Assert.Same(failure, actual);
        Assert.DoesNotContain(logs, line => line.Contains("stage=correction-arc event=fallback"));
    }

    [Fact]
    public void CancellationDuringCoarseCorrectionPropagatesWithoutReferenceRetry()
    {
        using var cancellation = new CancellationTokenSource();
        var logs = new List<string>();
        Assert.Throws<OperationCanceledException>(() => Run(Fixture(),
            Options() with { CorrectionArcTolerance = .00025 }, line =>
            {
                logs.Add(line);
                if (line.Contains("stage=smooth-concentric event=partition")) cancellation.Cancel();
            }, cancellation.Token));
        Assert.Contains(logs, line => line.Contains("stage=smooth-concentric event=partition"));
        Assert.DoesNotContain(logs, line => line.Contains("stage=correction-arc event=fallback"));
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    [InlineData(-.001)]
    [InlineData(0)]
    [InlineData(.00001)]
    [InlineData(.00101)]
    public void InvalidToleranceIsRejectedByPublicEntryAndEmptySmoothing(double tolerance)
    {
        var options = Options() with { CorrectionArcTolerance = tolerance };
        var publicError = Assert.Throws<ArgumentOutOfRangeException>(() => VCarveToolpaths.Generate(
            [], new ToolGeometry(1), new DepthSchedule(1, 1), options, new EmptyProvider()));
        var internalError = Assert.Throws<ArgumentOutOfRangeException>(() => VCarveSmoothing.Apply(
            [], [], new ToolGeometry(1), new DepthSchedule(1, 1), options, CancellationToken.None, null));
        Assert.Equal(nameof(VCarveOptions.CorrectionArcTolerance), publicError.ParamName);
        Assert.Equal(publicError.ParamName, internalError.ParamName);
    }

    private static VCarveOptions Options() => new()
    {
        SmoothVCarving = true, SmoothRampAngleDegrees = 15,
        IncludeInteriorFill = false, DetectCurves = false,
    };

    private sealed record Input(IReadOnlyList<TaggedToolpath> Source,
        IReadOnlyList<IReadOnlyList<PointD>> Boundary, ToolGeometry Tool);

    private static Input Fixture() => new(
        [new TaggedToolpath([new(1, 2, -.2), new(4, 2, -3), new(8, 2, -3)],
            Array.Empty<ToolpathSpan>(), 0, "final-carve", null, visitIdentity: 17, detectCurves: false)],
        [new PointD[] { new(0, 0), new(10, 0), new(10, 4), new(0, 4) }],
        new ToolGeometry(5) { BottomRadius = 0, TopRadius = 5, ConeLength = 10,
            TipAngleRadians = 2 * Math.Atan(.5) });

    private static List<TaggedToolpath> Run(Input input, VCarveOptions options,
        Action<string>? diagnostic, CancellationToken token = default)
        => VCarveSmoothing.Apply(input.Source, input.Boundary, input.Tool,
            new DepthSchedule(3, 1), options, token, diagnostic, correctionWorkers: 1);

    private static void EqualPaths(IReadOnlyList<TaggedToolpath> expected, IReadOnlyList<TaggedToolpath> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int path = 0; path < expected.Count; path++)
        {
            Assert.Equal(expected[path].Points, actual[path].Points);
            Assert.Equal(expected[path].Moves, actual[path].Moves);
            Assert.Equal(expected[path].Spans, actual[path].Spans);
            Assert.Equal(expected[path].RegionIndex, actual[path].RegionIndex);
            Assert.Equal(expected[path].Category, actual[path].Category);
            Assert.Equal(expected[path].DepthPassIndex, actual[path].DepthPassIndex);
            Assert.Equal(expected[path].VisitIdentity, actual[path].VisitIdentity);
        }
    }

    private sealed class EmptyProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle = 3 * Math.PI / 4, bool useBigIntegers = true) => [];
    }
}
