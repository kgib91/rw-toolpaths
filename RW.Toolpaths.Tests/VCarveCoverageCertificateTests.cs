using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveCoverageCertificateTests
{
    [Theory]
    [InlineData("sphere", 3, 51, 1)]
    [InlineData("sphere-reversed", 3, 51, 1)]
    [InlineData("multiple-components", 303, 168, 24)]
    [InlineData("profile-transition", 1, 3, 2)]
    [InlineData("mandatory-owner", 1, 3, 2)]
    [InlineData("disc-budget", 1, 2, 2)]
    public void FrozenCertificateTraversalAndWitnessesRemainExact(string scenario,
        long expectedBoxes, long expectedDiscs, int expectedIntervals)
    {
        IReadOnlyList<IReadOnlyList<PointD>> target;
        IReadOnlyList<TaggedToolpath> paths;
        VCarveCutterProfile profile;
        var limits = new VCarveCoverageCertificate.Limits { RecordWitnessPathIndices = true };
        if (scenario.StartsWith("sphere"))
        {
            profile = new([new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
                new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 21.5875);
            target = [Rectangle(.49, .033, .51, .036)];
            Point3D a = new(0, 0, 0), b = new(1, 0, -.001);
            paths = [scenario == "sphere-reversed" ? Path(b, a) : Path(a, b)];
        }
        else if (scenario == "multiple-components")
        {
            profile = Cylinder(.5);
            target = Enumerable.Range(0, 24).Select(i =>
                (IReadOnlyList<PointD>)Rectangle(3 * i - .2, -.2, 3 * i + .2, .2)).Reverse().ToArray();
            paths = Enumerable.Range(0, 24).Select(i =>
                Path(new(3 * i, 0, -.1), new(3 * i, 0, -.1))).Reverse().ToArray();
        }
        else if (scenario == "profile-transition")
        {
            profile = new([new(CarvingCutterSectionKind.Linear, .1, .6, .5),
                new(CarvingCutterSectionKind.Linear, .6, 1.6, 1)], 1.5);
            target = [Rectangle(.35, -.15, .65, .15)];
            paths = [Path(new(-1, 0, .1), new(1, 0, -1.5))];
        }
        else
        {
            profile = Cylinder(1);
            target = [Rectangle(-.2, -.2, .2, .2)];
            paths = [Path(new(0, 0, -.1), new(0, 0, -.1)), Path(new(0, 0, -.1), new(0, 0, -.1))];
            limits = limits with { OptionalWitnessPathIndices = new HashSet<int> { 0 } };
            if (scenario == "disc-budget") limits = limits with { MaximumDiscEvaluations = 2 };
        }
        var result = VCarveCoverageCertificate.Certify(target, paths, profile, limits: limits);
        // These snapshots were captured from the frozen pre-optimization native
        // assembly. Preserve the traversal, physical poses, budget accounting and
        // mandatory-owner preference, not merely the final success flag.
        bool certified = scenario != "disc-budget";
        Assert.Equal(certified, result.Certified);
        Assert.Equal(certified ? "whole-target-certified" : "disc-budget", result.Reason);
        Assert.Equal(expectedBoxes, result.VisitedBoxes);
        Assert.Equal(expectedDiscs, result.DiscEvaluations);
        Assert.Equal(expectedIntervals, result.CommandIntervals);
        Assert.Equal(certified, result.WitnessPathsRecorded);
        int[] expectedWitnesses = scenario == "multiple-components" ? Enumerable.Range(0, 24).ToArray()
            : scenario == "mandatory-owner" ? [1] : certified ? [0] : [];
        Assert.Equal(expectedWitnesses, result.WitnessPathIndices);
    }

    [Fact]
    public void CompleteDiscInteriorCertifiesWithoutSubdividingToTolerance()
    {
        var result = Certify([Rectangle(-.5, -.5, .5, .5)], [Path(new(0, 0, -.1), new(0, 0, -.1))], Cylinder(1));
        Assert.True(result.Certified, result.ToString());
        Assert.Equal("whole-target-certified", result.Reason);
        Assert.Equal(1, result.VisitedBoxes);
        Assert.InRange(result.DiscEvaluations, 1, 3);
    }

    [Fact]
    public void CompletionDiagnosticsRetainWorkCountsAndStageTiming()
    {
        var messages = new List<string>();
        var result = VCarveCoverageCertificate.Certify([Rectangle(-.5, -.5, .5, .5)],
            [Path(new(0, 0, -.1), new(0, 0, -.1))], Cylinder(1), diagnostic: messages.Add);
        Assert.True(result.Certified, result.ToString());
        Assert.Equal(3, result.DiscEvaluations);
        string completion = Assert.Single(messages.Where(message => message.Contains("event=complete")));
        Assert.Contains($"boxes={result.VisitedBoxes} ", completion);
        Assert.Contains($"discEvaluations={result.DiscEvaluations} ", completion);
        Assert.Contains($"boundarySegments={result.BoundarySegments} ", completion);
        Assert.Contains($"commandIntervals={result.CommandIntervals} ", completion);
        Assert.Contains("prepareMs=", completion);
        Assert.Contains("queryMs=", completion);
    }

    [Theory]
    [InlineData(-1e-12, false)]
    [InlineData(1e-12, true)]
    public void DiscRetainsArithmeticReserveAtAcceptanceBoundary(double margin, bool expected)
    {
        double radius = Math.Sqrt(.2 * .2 + .2 * .2) - .004
            + VCarveCoverageCertificate.ArithmeticReserve + margin;
        var result = VCarveCoverageCertificate.Certify([Rectangle(-.2, -.2, .2, .2)],
            [Path(new(0, 0, -.1), new(0, 0, -.1))], Cylinder(radius),
            limits: new() { MaximumBoxes = 1, RecordWitnessPathIndices = true });
        Assert.Equal(expected, result.Certified);
        Assert.Equal(expected ? "whole-target-certified" : "box-budget", result.Reason);
        Assert.Equal(1, result.VisitedBoxes);
        Assert.Equal(3, result.DiscEvaluations);
        Assert.Equal(expected ? new[] { 0 } : Array.Empty<int>(), result.WitnessPathIndices);
    }

    [Fact]
    public void RealMissedChannelCannotReceiveCertificate()
    {
        var result = Certify([Rectangle(-1, -1, 1, 1)], [Path(new(-1, 0, -.1), new(1, 0, -.1))], Cylinder(.2));
        Assert.False(result.Certified, result.ToString());
        Assert.NotEqual("empty-target", result.Reason);
    }

    [Theory]
    [InlineData(.02)]
    [InlineData(100)]
    public void LongThinUncutChannelRejectsIndependentlyOfItsArea(double length)
    {
        // Width 8.2um exceeds the fixed 8um permitted gap diameter. In the
        // first case its entire area is only .000164 square millimeters.
        var result = Certify([Rectangle(0, 0, length, .0082)], [], Cylinder(1));
        Assert.False(result.Certified, result.ToString());
        Assert.Equal(0, result.CommandIntervals);
    }

    [Fact]
    public void SubToleranceUncutTargetIsCertifiedByActualExteriorDistance()
    {
        var result = Certify([Rectangle(0, 0, .002, .002)], [], Cylinder(1));
        Assert.True(result.Certified, result.ToString());
        Assert.Equal(0, result.DiscEvaluations);
    }

    [Fact]
    public void MostlyOutsideBoxCertifiesThinTargetCapsWithoutCuttingDiscs()
    {
        // The single root box straddles two tiny target caps in opposite
        // corners. Its center lies far outside both components. Signed
        // exterior distance plus circumradius bounds every cap, while a
        // zero outside distance would force irrelevant empty subdivision.
        var result = Certify([Rectangle(-1.001, -1.001, -1, -1),
            Rectangle(1, 1, 1.001, 1.001)], [], Cylinder(1));
        Assert.True(result.Certified, result.ToString());
        Assert.Equal(1, result.VisitedBoxes);
        Assert.Equal(0, result.DiscEvaluations);
    }

    [Fact]
    public void MostlyOutsideBoxCannotHideInteriorGapWiderThanEightMicrons()
    {
        // The same exterior-centered topology now has a 10um-wide uncut
        // component. Its interior contains a real >4um-radius gap.
        var result = Certify([Rectangle(-1.01, -1.01, -1, -1),
            Rectangle(1, 1, 1.01, 1.01)], [], Cylinder(1));
        Assert.False(result.Certified, result.ToString());
        Assert.True(result.VisitedBoxes > 1);
        Assert.Equal(0, result.DiscEvaluations);
    }

    [Fact]
    public void HoleIsExcludedButFillingSameHoleIntoTargetRejects()
    {
        PointD[] outer = Rectangle(-1, -1, 1, 1);
        PointD[] hole = Rectangle(-.5, -.5, .5, .5).Reverse().ToArray();
        var frame = Path(new(-.75, -.75, -.1), new(.75, -.75, -.1), new(.75, .75, -.1),
            new(-.75, .75, -.1), new(-.75, -.75, -.1));
        var withHole = Certify([outer, hole], [frame], Cylinder(.4));
        var filled = Certify([outer], [frame], Cylinder(.4));
        Assert.True(withHole.Certified, withHole.ToString());
        Assert.False(filled.Certified, filled.ToString());
        Assert.Equal(8, withHole.BoundarySegments);
    }

    [Fact]
    public void EntireTargetIsCheckedIncludingComponentAbsentFromAnyResidualSelection()
    {
        var cut = Path(new(0, 0, -.1), new(0, 0, -.1));
        PointD[] covered = Rectangle(-.2, -.2, .2, .2), missed = Rectangle(5, 5, 5.02, 5.02);
        Assert.True(Certify([covered], [cut], Cylinder(1)).Certified);
        var result = Certify([covered, missed], [cut], Cylinder(1));
        Assert.False(result.Certified, result.ToString());
        Assert.Equal(8, result.BoundarySegments);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousShallowSphericalRampCertifiesBeyondConservativePolygon(bool reverse)
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 21.5875);
        Point3D a = new(0, 0, 0), b = new(1, 0, -.001);
        var ramp = reverse ? Path(b, a) : Path(a, b);
        PointD[] target = Rectangle(.49, .033, .51, .036);
        var conservative = VCarvePhysicalSweep.Build([ramp], profile, 0, .25);
        var polygonGap = Clipper.Difference(PathUtils.ToClipper([target.ToList()]),
            PathUtils.ToClipper(conservative), FillRule.NonZero);
        Assert.NotEmpty(polygonGap);
        // The disc at the mid-ramp pose reaches about 0.03984 mm radius;
        // neither endpoint disc comes anywhere near this target rectangle.
        Assert.True(Math.Sqrt(.5 * .5 + .033 * .033) > profile.PhysicalRadiusAtDepth(.001));
        var result = Certify([target], [ramp], profile);
        Assert.True(result.Certified, result.ToString());
        Assert.True(result.DiscEvaluations > 3);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.1)]
    public void WhollyUnengagedFlatBottomChordDoesNotCountAsCutting(double z)
    {
        var result = Certify([Rectangle(-.2, -.2, .2, .2)], [Path(new(-1, 0, z), new(1, 0, z))], Cylinder(10));
        Assert.False(result.Certified, result.ToString());
        Assert.Equal(0, result.CommandIntervals);
        Assert.Equal(0, result.DiscEvaluations);
    }

    [Fact]
    public void SurfaceCrossingClipsAirPosesAndKeepsPenetratingPart()
    {
        var result = Certify([Rectangle(.4, -.1, .6, .1)],
            [Path(new(-1, 0, .2), new(1, 0, -.2))], Cylinder(.5));
        Assert.True(result.Certified, result.ToString());
        Assert.Equal(1, result.CommandIntervals);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MultipleBvhBranchesRetainAllTargetComponentsAndCutWitnesses(bool omitLastCut)
    {
        var target = Enumerable.Range(0, 24).Select(i =>
            (IReadOnlyList<PointD>)Rectangle(3 * i - .2, -.2, 3 * i + .2, .2)).Reverse().ToArray();
        var paths = Enumerable.Range(0, omitLastCut ? 23 : 24).Select(i =>
            Path(new(3 * i, 0, -.1), new(3 * i, 0, -.1))).Reverse().ToArray();
        var result = Certify(target, paths, Cylinder(.5));
        Assert.Equal(!omitLastCut, result.Certified);
        Assert.Equal(96, result.BoundarySegments);
        Assert.Equal(paths.Length, result.CommandIntervals);
    }

    [Fact]
    public void TaggedRetractHasNoCuttingAuthorityEvenBelowSurface()
    {
        var retract = new TaggedToolpath(new[] { new Point3D(-1, 0, -.1), new Point3D(1, 0, -.1) },
            new[] { new ToolpathSpan(0, 1, ToolpathSpanKind.Retract) }, 0, "final-carve", null, detectCurves: false);
        var result = Certify([Rectangle(-.2, -.2, .2, .2)], [retract], Cylinder(1));
        Assert.False(result.Certified);
        Assert.Equal(0, result.CommandIntervals);
    }

    [Fact]
    public void CallerSuppliedSurfaceZDefinesActualPenetration()
    {
        var paths = new[] { Path(new(-1, 0, 9.9), new(1, 0, 9.9)) };
        IReadOnlyList<PointD>[] target = [Rectangle(-.2, -.2, .2, .2)];
        Assert.True(VCarveCoverageCertificate.Certify(target, paths, Cylinder(.5), outputSurfaceZ: 10).Certified);
        Assert.False(VCarveCoverageCertificate.Certify(target, paths, Cylinder(.5), outputSurfaceZ: 0).Certified);
    }

    [Fact]
    public void ExhaustedBoxOrDiscBudgetIsInconclusive()
    {
        var target = new IReadOnlyList<PointD>[] { Rectangle(-.2, -.2, .2, .2) };
        var paths = new[] { Path(new(0, 0, -.1), new(0, 0, -.1)) };
        var boxes = VCarveCoverageCertificate.Certify(target, paths, Cylinder(1),
            limits: new() { MaximumBoxes = 0 });
        var discs = VCarveCoverageCertificate.Certify(target, paths, Cylinder(1),
            limits: new() { MaximumDiscEvaluations = 0 });
        Assert.False(boxes.Certified);
        Assert.Equal("box-budget", boxes.Reason);
        Assert.False(discs.Certified);
        Assert.Equal("disc-budget", discs.Reason);
    }

    [Fact]
    public void PreparationBudgetIsInconclusiveAndCancellationPropagates()
    {
        IReadOnlyList<PointD>[] target = [Rectangle(-1, -1, 1, 1)];
        var bounded = VCarveCoverageCertificate.Certify(target, [], Cylinder(1),
            limits: new() { MaximumInputSegments = 3 });
        Assert.False(bounded.Certified);
        Assert.Equal("input-budget", bounded.Reason);
        var timed = VCarveCoverageCertificate.Certify(target, [], Cylinder(1),
            limits: new() { MaximumElapsed = TimeSpan.FromTicks(1) });
        Assert.False(timed.Certified);
        Assert.Equal("time-budget", timed.Reason);
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveCoverageCertificate.Certify(target, [], Cylinder(1),
            cancellationToken: cancellation.Token));
    }

    [Fact]
    public void EmptyTargetIsVacuousButMalformedTargetCannotSilentlyVanish()
    {
        Assert.True(Certify([], [], Cylinder(1)).Certified);
        Assert.Throws<ArgumentException>(() => Certify([[new(0, 0), new(1, 1)]], [], Cylinder(1)));
    }

    private static VCarveCoverageCertificate.Result Certify(IReadOnlyList<IReadOnlyList<PointD>> target,
        IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile)
        => VCarveCoverageCertificate.Certify(target, paths, profile);
    private static VCarveCutterProfile Cylinder(double radius)
        => new([new(CarvingCutterSectionKind.Linear, radius, radius, 1)], 1);
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
}
