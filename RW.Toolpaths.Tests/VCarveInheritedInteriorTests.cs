using Clipper2Lib;
using RW.Toolpaths.Milling;
using Cert = RW.Toolpaths.Milling.VCarveCoverageCertificate;

namespace RW.Toolpaths.Tests;

public sealed class VCarveInheritedInteriorTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WholeTargetAndHoleRetainEveryBoxDiscAndWitnessOwner(bool hole)
    {
        IReadOnlyList<IReadOnlyList<PointD>> target = hole
            ? [Rectangle(-2, -2, 2, 2), Rectangle(-.6, -.6, .6, .6).Reverse().ToArray()]
            : [Rectangle(-2, -2, 2, 2)];
        var limits = new Cert.Limits { RecordWitnessPathIndices = true };
        var reference = Cert.Certify(target, ClearingChords(), Cylinder(.12), limits: limits);
        var logs = new List<string>();
        var actual = Cert.Certify(target, ClearingChords(), Cylinder(.12),
            limits: limits with { UseInheritedInteriorClassification = true }, diagnostic: logs.Add);
        AssertSame(reference, actual); Assert.True(actual.Certified, actual.ToString());
        Assert.True(actual.InteriorClassificationEligible);
        Assert.True(actual.StrictInteriorSeeds > 0, actual.ToString());
        Assert.True(actual.SkippedBoundaryQueries > actual.VisitedBoxes, actual.ToString());
        Assert.Equal(0, actual.SkippedBoundaryQueries % 2);
        Assert.Equal(0, reference.SkippedBoundaryQueries); Assert.Equal(0, reference.StrictInteriorSeeds);
        string completion = Assert.Single(logs.Where(line => line.Contains("event=complete")));
        Assert.Contains($"skippedBoundaryQueries={actual.SkippedBoundaryQueries} ", completion);
        Assert.Contains($"strictInteriorSeeds={actual.StrictInteriorSeeds} ", completion);
    }

    [Fact]
    public void NearCornersAndSeparatedOutsideComponentsRetainExteriorDecisions()
    {
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(-2, -2, 2, 2), Rectangle(5, 5, 5.002, 5.002)];
        var limits = new Cert.Limits { RecordWitnessPathIndices = true };
        var reference = Cert.Certify(target, ClearingChords(), Cylinder(.12), limits: limits);
        var actual = Cert.Certify(target, ClearingChords(), Cylinder(.12),
            limits: limits with { UseInheritedInteriorClassification = true });
        AssertSame(reference, actual); Assert.True(actual.Certified, actual.ToString());
    }

    [Fact]
    public void DirectDiscAcceptanceDoesNotCreateAnInteriorSeed()
    {
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(-.01, -.01, .01, .01)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, -.1), new(0, 0, -.1))];
        var reference = Cert.Certify(target, paths, Cylinder(.1));
        var actual = Cert.Certify(target, paths, Cylinder(.1), limits: new() { UseInheritedInteriorClassification = true });
        AssertSame(reference, actual); Assert.True(actual.Certified);
        Assert.Equal(0, actual.StrictInteriorSeeds); Assert.Equal(0, actual.SkippedBoundaryQueries);
    }

    [Theory]
    [InlineData("large-coordinate")]
    [InlineData("tiny-ray-delta")]
    public void UnsupportedDomainFallsBackToEveryOriginalBoundaryQuery(string scenario)
    {
        double translate = scenario == "large-coordinate" ? 10_001 : 0;
        IReadOnlyList<IReadOnlyList<PointD>> target = scenario == "tiny-ray-delta"
            ? [Rectangle(-2, -2, 2, 2), new PointD[] { new(0, 0), new(.2, 1e-110), new(.2, .2), new(0, .2) }.Reverse().ToArray()]
            : [Rectangle(translate - 2, -2, translate + 2, 2)];
        var reference = Cert.Certify(target, ClearingChords(translate), Cylinder(.12));
        var actual = Cert.Certify(target, ClearingChords(translate), Cylinder(.12),
            limits: new() { UseInheritedInteriorClassification = true });
        AssertSame(reference, actual); Assert.True(actual.Certified, actual.ToString());
        Assert.False(actual.InteriorClassificationEligible); Assert.Equal(1, actual.InteriorEligibilityGuardRefusals);
        Assert.Equal(0, actual.StrictInteriorSeeds); Assert.Equal(0, actual.SkippedBoundaryQueries);
    }

    [Fact]
    public void InconclusiveUncutTargetPreservesTheUnresolvedBoxAndWorkQuota()
    {
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(-2, -2, 2, 2)];
        var limits = new Cert.Limits { MaximumBoxes = 128, RecordWitnessPathIndices = true };
        var reference = Cert.Certify(target, [], Cylinder(.12), limits: limits);
        var actual = Cert.Certify(target, [], Cylinder(.12),
            limits: limits with { UseInheritedInteriorClassification = true });
        AssertSame(reference, actual); Assert.False(actual.Certified);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoxDiscClockAndCancellationRemainAuthoritative(bool enabled)
    {
        Assert.False(new Cert.Limits().UseInheritedInteriorClassification);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(-2, -2, 2, 2)];
        var limits = new Cert.Limits { UseInheritedInteriorClassification = enabled };
        var boxes = Cert.Certify(target, ClearingChords(), Cylinder(.12), limits: limits with { MaximumBoxes = 0 });
        Assert.False(boxes.Certified); Assert.Equal("box-budget", boxes.Reason); Assert.Equal(1, boxes.RemainingBoxes);
        var discs = Cert.Certify(target, ClearingChords(), Cylinder(.12), limits: limits with { MaximumDiscEvaluations = 0 });
        Assert.False(discs.Certified); Assert.Equal("disc-budget", discs.Reason); Assert.Equal(0, discs.DiscEvaluations);
        var timed = Cert.Certify(target, ClearingChords(), Cylinder(.12), limits: limits with { MaximumElapsed = TimeSpan.FromTicks(1) });
        Assert.False(timed.Certified); Assert.Equal("time-budget", timed.Reason);
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => Cert.Certify(target, ClearingChords(), Cylinder(.12),
            cancellationToken: cancellation.Token, limits: limits));
    }

    private static void AssertSame(Cert.Result reference, Cert.Result actual)
    {
        Assert.Equal(reference.Certified, actual.Certified); Assert.Equal(reference.Reason, actual.Reason);
        Assert.Equal(reference.VisitedBoxes, actual.VisitedBoxes); Assert.Equal(reference.DiscEvaluations, actual.DiscEvaluations);
        Assert.Equal(reference.CommandIntervals, actual.CommandIntervals); Assert.Equal(reference.RemainingBoxes, actual.RemainingBoxes);
        Assert.Equal(reference.UnresolvedPoint, actual.UnresolvedPoint); Assert.Equal(reference.WitnessPathsRecorded, actual.WitnessPathsRecorded);
        Assert.Equal(reference.WitnessPathIndices, actual.WitnessPathIndices);
    }
    private static IReadOnlyList<TaggedToolpath> ClearingChords(double translate = 0)
        => Enumerable.Range(0, 23).Select(i => Path(new(translate - 2, -2 + i * .18, -.1),
            new(translate + 2, -2 + i * .18, -.1))).ToArray();
    private static VCarveCutterProfile Cylinder(double radius)
        => new([new(CarvingCutterSectionKind.Linear, radius, radius, 1)], 1);
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
}
