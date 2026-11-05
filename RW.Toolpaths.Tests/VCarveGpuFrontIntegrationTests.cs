using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.Acceleration;
using System.Collections.Immutable;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

// Staged integration controls. These run the real ApplyCore, protection and
// Certify. The independent producer supplies only untrusted nomination edges.
// No control can replace a real certificate with a successful answer.
[Collection("Pocket construction policy controls")]
public sealed class VCarveGpuFrontIntegrationTests
{
    private readonly ITestOutputHelper _output;
    public VCarveGpuFrontIntegrationTests(ITestOutputHelper output) => _output = output;
    private sealed class Lifetime : IDisposable
    {
        internal int Disposals;
        public void Dispose() => Disposals++;
    }
    private sealed record Fixture(IReadOnlyList<TaggedToolpath> Source,
        IReadOnlyList<IReadOnlyList<PointD>> Boundary, ToolGeometry Tool,
        DepthSchedule Schedule, VCarveOptions Options);
    private sealed class Capture
    {
        internal readonly List<string> Logs = new();
        internal readonly List<VCarveSmoothing.GpuFrontCoreInputs> Cores = new();
        internal readonly List<VCarveSmoothing.GpuFrontCompletionInputs> Completions = new();
        internal readonly List<long[][]> FixedTargets = new();
        internal readonly Lifetime Lifetime = new();
        internal int Factories, ProducerCalls;
        internal VCarveGpuCorrectionFrontNominator? Owner;
    }

    [Fact]
    public void DisabledRouteDoesNotCreateNominationOwnerAndPreservesOriginalResult()
    {
        var fixture = Ramp(); var baseline = Run(fixture, false, null, null);
        var capture = new Capture();
        var control = new VCarveSmoothing.GpuFrontIntegrationControl(
            NominationOwnerFactory: (_, _, _) => throw new InvalidOperationException("Off route created GPU owner."),
            CoreEntered: capture.Cores.Add);
        var actual = Run(fixture, false, control, capture.Logs.Add);
        EqualGeometryBits(baseline, actual);
        Assert.Single(capture.Cores); Assert.False(capture.Cores[0].UseGpuFronts);
        Assert.DoesNotContain(capture.Logs, message => message.Contains("event=gpu-front-nomination"));
        Assert.Contains(capture.Logs, message => message.Contains("taperedLowerSweeps=False"));
    }

    [Fact]
    public void UnsupportedBoxSelectorsRetainWholeLiteralOrderedSelection()
    {
        foreach (double origin in new[] { 0d, 40_000_000d })
        {
            var error = new IReadOnlyList<PointD>[]
            {
                Rectangle(origin, origin, origin + 8, origin + 8),
                Rectangle(origin + 2, origin + 2, origin + 4, origin + 4).Reverse().ToArray(),
                new PointD[] { new(origin + 12, origin), new(origin + 18, origin), new(origin + 18, origin + 2),
                    new(origin + 14, origin + 2), new(origin + 14, origin + 6), new(origin + 12, origin + 6) },
                Rectangle(origin + 5, origin + 1, origin + 10, origin + 7)
            };
            var contact = new List<List<PointD>>
            {
                Rectangle(origin - 1, origin - 1, origin + 20, origin + 10).ToList(),
                Rectangle(origin + 1, origin + 1, origin + 3, origin + 3).Reverse().ToList()
            };
            var disjoint = new List<List<PointD>> { Rectangle(origin + 200, origin + 200, origin + 210, origin + 210).ToList() };
            foreach (var legal in new[] { contact, disjoint })
            foreach (double reach in origin == 0 ? new[] { 32d, 40d } : new[] { .02, 40d })
            {
                var original = new VCarveSmoothing.PreparedCorrectionNeighborhood(error);
                var candidate = new VCarveBoxCorrectionNeighborhood(error, useBoxSelector: true);
                var expected = original.SelectCenters(legal, reach);
                var actual = candidate.SelectCenters(legal, reach);
                Assert.Equal(TargetBits(expected), TargetBits(actual));
                Assert.Equal(0, candidate.BoxQueries); Assert.Equal(1, candidate.OriginalFallbackQueries);
                if (ReferenceEquals(legal, disjoint)) Assert.Empty(actual);
                else Assert.NotEmpty(actual);
            }
        }
        _output.WriteLine("unsupportedWholeSelectorOrderedBits=True oversizedReachAndCoordinateOwners=True holesReflexOverlapAndDisjoint=True originalMayIntersectRetained=True");
    }

    [Fact]
    public void SmallChannelBoxRequestUsesLiteralSelectorWithoutExtraCertificateAndKeepsPhysicalGuards()
    {
        var fixture=RoundedRamp();var oldTargets=new List<long[][]>();
        var baseline=Run(fixture,false,new VCarveSmoothing.GpuFrontIntegrationControl(
            FixedTargetPrepared: target=>oldTargets.Add(TargetBits(target))),null);
        var capture=new Capture();var certificates=new List<VCarveSmoothing.CertificateInputs>();
        var previous=VCarveSmoothing.CertificateObserver.Value;List<TaggedToolpath> actual;
        try
        {
            VCarveSmoothing.CertificateObserver.Value=certificates.Add;
            var control=new VCarveSmoothing.GpuFrontIntegrationControl(
                ForceCompletionCertificateInconclusive:true,
                FixedTargetPrepared:target=>capture.FixedTargets.Add(TargetBits(target)));
            actual=Run(fixture,false,control,capture.Logs.Add,boxNeighborhoods:true);
        }
        finally{VCarveSmoothing.CertificateObserver.Value=previous;}
        EqualGeometryBits(baseline,actual);Assert.Single(oldTargets);Assert.Single(capture.FixedTargets);
        Assert.Equal(oldTargets[0],capture.FixedTargets[0]);Assert.Empty(certificates);
        Assert.Contains(capture.Logs,line=>line.Contains("event=source-policy requested=True eligible=False")
            &&line.Contains("minimumSourcePoints=4096")&&line.Contains("boundedInternalControl=False"));
        Assert.DoesNotContain(capture.Logs,line=>line.Contains("boxCompletionRequired=True")
            ||System.Text.RegularExpressions.Regex.IsMatch(line,@"boxNeighborhoodQueries=[1-9]\d*"));
        VerifyOriginalPhysicalGuards(fixture,actual);
    }

    [Fact]
    public void DisabledBoxNeighborhoodRetainsOriginalGeometryWithoutAdditionalCertificate()
    {
        var fixture = RoundedRamp(); var baseline = Run(fixture, false, null, null);
        var capture = new Capture();
        var actual = Run(fixture, false, new VCarveSmoothing.GpuFrontIntegrationControl(
            MinimumBoxSourcePointsForControls: 0,
            CoreEntered: capture.Cores.Add), capture.Logs.Add, boxNeighborhoods: false);
        EqualGeometryBits(baseline, actual);
        Assert.Single(capture.Cores);
        Assert.Contains(capture.Logs, line => line.Contains("stage=box-correction-neighborhood event=selection enabled=False"));
        Assert.DoesNotContain(capture.Logs, line => line.Contains("boxCompletionRequired=True"));
        Assert.DoesNotContain(capture.Logs, line => System.Text.RegularExpressions.Regex.IsMatch(line, @"boxNeighborhoodQueries=[1-9]\d*"));
    }

    [Fact]
    public void BoxNeighborhoodUsesCapturedPolicyAndUnchangedTargetWithOriginalPhysicalGuards()
    {
        var fixture = RoundedRamp(); var oldTargets = new List<long[][]>();
        Run(fixture, false, new VCarveSmoothing.GpuFrontIntegrationControl(
            FixedTargetPrepared: target => oldTargets.Add(TargetBits(target))), null);
        var capture = new Capture(); var certificates = new List<VCarveSmoothing.CertificateInputs>();
        var previous = VCarveSmoothing.CertificateObserver.Value;
        List<TaggedToolpath> actual;
        try
        {
            VCarveSmoothing.CertificateObserver.Value = certificates.Add;
            var control = new VCarveSmoothing.GpuFrontIntegrationControl(
            MinimumBoxSourcePointsForControls: 0,
                NominationOwnerFactory: (_, _, _) => throw new InvalidOperationException("Box NOM created GPU front owner."),
                CoreEntered: inputs => { capture.Cores.Add(inputs); Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_BOX_NEIGHBORHOODS", "0"); },
                FixedTargetPrepared: target => capture.FixedTargets.Add(TargetBits(target)));
            actual = Run(fixture, false, control, capture.Logs.Add, boxNeighborhoods: true);
        }
        finally { VCarveSmoothing.CertificateObserver.Value = previous; }
        Assert.Single(capture.Cores); Assert.Single(oldTargets); Assert.Single(capture.FixedTargets);
        var core = capture.Cores[0];
        Assert.Same(fixture.Source, core.Source); Assert.Same(fixture.Boundary, core.Boundary);
        Assert.Same(fixture.Tool, core.Tool); Assert.Same(fixture.Schedule, core.Schedule); Assert.Same(fixture.Options, core.Options);
        Assert.Equal(oldTargets[0], capture.FixedTargets[0]); Assert.NotEmpty(certificates);
        Assert.All(certificates, item => Assert.Equal(capture.FixedTargets[0], TargetBits(item.Target)));
        Assert.Contains(capture.Logs, line => line.Contains("stage=box-correction-neighborhood event=selection enabled=True")
            && line.Contains("selectionFrozenForApply=True") && line.Contains("physicalCredit=0"));
        Assert.Contains(capture.Logs, line => line.Contains("boxCompletionRequired=True"));
        Assert.Contains(capture.Logs, line => line.Contains("stage=smooth-physical-certificate") && line.Contains("certified=True"));
        VerifyOriginalPhysicalGuards(fixture, actual);
    }

    [Fact]
    public void BoxNeighborhoodCannotPublishAfterRealFixedTargetCertificateBudgetRefusal()
    {
        var fixture = RoundedRamp(); var capture = new Capture();
        var certificates = new List<VCarveSmoothing.CertificateInputs>();
        var previous = VCarveSmoothing.CertificateObserver.Value;
        try
        {
            VCarveSmoothing.CertificateObserver.Value = certificates.Add;
            var control = new VCarveSmoothing.GpuFrontIntegrationControl(
            MinimumBoxSourcePointsForControls: 0,
                ForceCompletionCertificateInconclusive: true, CoreEntered: capture.Cores.Add,
                FixedTargetPrepared: target => capture.FixedTargets.Add(TargetBits(target)));
            var failure = Assert.Throws<InvalidOperationException>(() => Run(fixture, false, control,
                capture.Logs.Add, boxNeighborhoods: true));
            Assert.Contains("Box correction nomination requires the unchanged complete target certificate", failure.Message);
            Assert.True(failure.Data.Contains("SmoothVCarvingGeometry"));
        }
        finally { VCarveSmoothing.CertificateObserver.Value = previous; }
        Assert.Single(capture.Cores); Assert.Single(capture.FixedTargets); Assert.NotEmpty(certificates);
        Assert.Contains(certificates, item => item.Limits.MaximumBoxes == 1);
        Assert.All(certificates, item => Assert.Equal(capture.FixedTargets[0], TargetBits(item.Target)));
        Assert.Contains(capture.Logs, line => line.Contains("boxCompletionRequired=True") && line.Contains("maximumBoxes=1"));
        Assert.Contains(capture.Logs, line => line.Contains("certified=False") && line.Contains("reason=box-budget"));
        Assert.DoesNotContain(capture.Logs, line => line.Contains("stage=smoothing event=complete")
            || line.Contains("stage=correction-arc event=complete") || line.Contains("event=completion-fallback"));
    }

    [Fact]
    public void TaperedLowerProfileQuotaKeepsLiteralGeometryAndCapturedPolicyWithOriginalGuards()
    {
        var rounded = RoundedRamp();
        var shapes = rounded.Tool.VCarveProfile!.Sections.Concat(Enumerable.Repeat(
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, .5, .5, .001), 63)).ToArray();
        var profile = new VCarveCutterProfile(shapes, rounded.Tool.VCarveProfile.MaximumDepth);
        var fixture = rounded with { Tool = rounded.Tool with { VCarveProfile = profile } };
        Assert.Equal(65, profile.Sections.Count);
        Assert.False(new TaperedRadiusRun(profile, 0, CancellationToken.None).Eligible);
        var oldTargets = new List<long[][]>();
        var baseline = Run(fixture, false, new VCarveSmoothing.GpuFrontIntegrationControl(
            FixedTargetPrepared: target => oldTargets.Add(TargetBits(target))), null);
        var capture = new Capture();
        var control = new VCarveSmoothing.GpuFrontIntegrationControl(
            NominationOwnerFactory: (_, _, _) => throw new InvalidOperationException("Tapered lower policy created GPU nomination owner."),
            CoreEntered: inputs => { capture.Cores.Add(inputs); Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_TAPERED_SWEEPS", "0"); },
            FixedTargetPrepared: target => capture.FixedTargets.Add(TargetBits(target)));
        var actual = Run(fixture, false, control, capture.Logs.Add, tapered: true);
        EqualGeometryBits(baseline, actual);
        Assert.Single(capture.Cores); Assert.False(capture.Cores[0].UseGpuFronts);
        Assert.Single(oldTargets); Assert.Single(capture.FixedTargets); Assert.Equal(oldTargets[0], capture.FixedTargets[0]);
        Assert.Contains(capture.Logs, message => message.Contains("taperedLowerSweeps=True") && message.Contains("selectionFrozenForApply=True"));
        VerifyOriginalPhysicalGuards(fixture, actual);
        _output.WriteLine($"taperedRequested=True realProfileQuotaRefused=True literalGeometryBits=True fixedTargetBits=True capturedPolicySurvivesEnvironmentMutation=True fullFloorWallGradeGuards=True paths={actual.Count}");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FloorAwareProtectedRecoveryRequiresRealWholeTargetCertificateBeforePublication(bool contourSeeds)
    {
        var fixture = Ramp(); var capture = new Capture();
        var certificates = new List<VCarveSmoothing.CertificateInputs>();
        var previousObserver = VCarveSmoothing.CertificateObserver.Value;
        var control = new VCarveSmoothing.GpuFrontIntegrationControl(
            ForceCompletionCertificateInconclusive: true, CoreEntered: capture.Cores.Add,
            FixedTargetPrepared: target => capture.FixedTargets.Add(TargetBits(target)));
        try
        {
            VCarveSmoothing.CertificateObserver.Value = certificates.Add;
            var failure = Assert.Throws<InvalidOperationException>(() => Run(fixture, false, control,
                capture.Logs.Add, floorAware: true, contourSeeds: contourSeeds));
            Assert.Contains("Floor-aware correction requires the unchanged complete target certificate", failure.Message);
        }
        finally { VCarveSmoothing.CertificateObserver.Value = previousObserver; }
        Assert.Single(capture.Cores); Assert.False(capture.Cores[0].UseGpuFronts);
        Assert.Single(capture.FixedTargets); Assert.NotEmpty(certificates);
        Assert.Contains(certificates, item => item.Limits.MaximumBoxes == 1);
        foreach (var item in certificates) Assert.Equal(capture.FixedTargets[0], TargetBits(item.Target));
        Assert.All(certificates, item => Assert.Equal(contourSeeds, item.Limits.UseContourBoxSeeds));
        Assert.Contains(capture.Logs, message => message.Contains("stage=floor-aware-correction event=protected")
            && System.Text.RegularExpressions.Regex.IsMatch(message, @"cumulativePaths=[1-9]\d*"));
        Assert.Contains(capture.Logs, message => message.Contains("floorAwareCompletionRequired=True")
            && message.Contains("maximumBoxes=1"));
        Assert.Contains(capture.Logs, message => message.Contains("certified=False")
            && (message.Contains("reason=box-budget") || message.Contains("reason=contour-seed-box-budget")));
        Assert.DoesNotContain(capture.Logs, message => message.Contains("stage=smoothing event=complete")
            || message.Contains("stage=correction-arc event=complete"));
        Assert.DoesNotContain(capture.Logs, message => message.Contains("event=completion-fallback"));
        var budgetIndex = capture.Logs.FindIndex(message => message.Contains("stage=floor-aware-correction event=point-budget"));
        var sweepIndex = capture.Logs.FindIndex(message => message.Contains("event=begin work=floor-aware-lower-sweep-"));
        Assert.True(budgetIndex >= 0 && sweepIndex > budgetIndex, "Actual protected sum must be checked before appended recovery is swept.");
        var counts = System.Text.RegularExpressions.Regex.Match(capture.Logs[budgetIndex],
            @"existingPoints=(\d+) newPoints=(\d+) cumulativePoints=(\d+) pointBudget=(\d+)");
        Assert.True(counts.Success);
        long previous = long.Parse(counts.Groups[1].Value), added = long.Parse(counts.Groups[2].Value);
        long cumulative = long.Parse(counts.Groups[3].Value), budget = long.Parse(counts.Groups[4].Value);
        Assert.True(added > 0); Assert.Equal(checked(previous + added), cumulative); Assert.True(cumulative <= budget);
    }

    [Fact]
    public void LowerSweepBudgetKeepsFixedTargetAndPhysicalGuardsAfterEnvironmentMutation()
    {
        var fixture = RoundedRamp(); var oldTargets = new List<long[][]>();
        Run(fixture, false, new VCarveSmoothing.GpuFrontIntegrationControl(
            FixedTargetPrepared: target => oldTargets.Add(TargetBits(target))), null);
        var capture = new Capture();
        var control = new VCarveSmoothing.GpuFrontIntegrationControl(
            CoreEntered: inputs => { capture.Cores.Add(inputs); Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_LOWER_SWEEP_BUDGET", "0"); },
            FixedTargetPrepared: target => capture.FixedTargets.Add(TargetBits(target)));
        var actual = Run(fixture, false, control, capture.Logs.Add, tapered: true, lowerSweepBudget: true);
        Assert.Single(capture.Cores); Assert.Single(oldTargets); Assert.Single(capture.FixedTargets);
        Assert.Equal(oldTargets[0], capture.FixedTargets[0]);
        Assert.Contains(capture.Logs, line => line.Contains("lowerSweepBudget=True lowerSweepTolerance=0.004")
            && line.Contains("selectionFrozenForApply=True"));
        VerifyOriginalPhysicalGuards(fixture, actual);
    }

    [Fact]
    public void LowerSweepBudgetCannotPublishWhenFreshWholeTargetQuotaRefuses()
    {
        var capture = new Capture(); var certificates = new List<VCarveSmoothing.CertificateInputs>();
        var previous = VCarveSmoothing.CertificateObserver.Value;
        try
        {
            VCarveSmoothing.CertificateObserver.Value = certificates.Add;
            var control = new VCarveSmoothing.GpuFrontIntegrationControl(ForceCompletionCertificateInconclusive: true,
                CoreEntered: capture.Cores.Add, FixedTargetPrepared: target => capture.FixedTargets.Add(TargetBits(target)));
            Assert.Throws<InvalidOperationException>(() => Run(Ramp(), false, control, capture.Logs.Add,
                floorAware: true, contourSeeds: true, tapered: true, lowerSweepBudget: true));
        }
        finally { VCarveSmoothing.CertificateObserver.Value = previous; }
        Assert.Single(capture.Cores); Assert.Single(capture.FixedTargets); Assert.NotEmpty(certificates);
        Assert.Contains(certificates, item => item.Limits.MaximumBoxes == 1);
        Assert.All(certificates, item => Assert.Equal(capture.FixedTargets[0], TargetBits(item.Target)));
        Assert.DoesNotContain(capture.Logs, line => line.Contains("stage=smoothing event=complete")
            || line.Contains("stage=correction-arc event=complete"));
    }

    [Fact]
    public void ContourSeedGpuCompletionStillRequiresSameFullTargetQuotaAndOriginalOnceOnlyFallback()
    {
        var fixture = Ramp(); var baseline = Run(fixture, false, null, null);
        var capture = new Capture();
        var actual = Run(fixture, true, Control(capture, forceInconclusive: true), capture.Logs.Add,
            contourSeeds: true);
        var complete = Assert.Single(capture.Completions);
        Assert.True(complete.SignificantEmpty); Assert.True(complete.NominatedComponents > 0);
        Assert.True(complete.Authority.Limits.UseContourBoxSeeds);
        Assert.Equal(1, complete.Authority.Limits.MaximumBoxes);
        Assert.True(complete.Certificate.ContourBoxSeedsRequested);
        Assert.True(complete.Certificate.ContourBoxSeedsUsed);
        Assert.Equal(complete.Authority.Target.Count, complete.Certificate.ContourSeedBoxes);
        Assert.Equal("contour-bounds-boxes", complete.Certificate.CoverageDomain);
        Assert.False(complete.Certificate.Certified); Assert.Equal("box-budget", complete.Certificate.Reason);
        Assert.Equal(capture.FixedTargets[0], TargetBits(complete.Authority.Target));
        Assert.Equal(capture.FixedTargets[0], capture.FixedTargets[1]);
        Assert.False(complete.OriginalCertificateRequested);
        Assert.False(complete.Authority.Limits.RecordWitnessPathIndices);
        Assert.Null(complete.Authority.Limits.OptionalWitnessPathIndices);
        Assert.False(complete.Certificate.WitnessPathsRecorded); Assert.Empty(complete.Certificate.WitnessPathIndices);
        Assert.Equal(new[] { true, false }, capture.Cores.Select(core => core.UseGpuFronts));
        Assert.Single(capture.Logs.Where(message => message.Contains("event=completion-fallback")));
        Assert.Contains(capture.Logs, message => message.Contains("event=contour-seeds")
            && message.Contains("allTargetRingsRetained=True") && message.Contains("sharedOriginalQuotas=True"));
        Assert.DoesNotContain(capture.Logs, message => message.Contains("stage=smooth-terminal-pruning event=complete"));
        EqualGeometryBits(baseline, actual);
        Assert.Equal(1, capture.Factories); Assert.Equal(1, capture.Lifetime.Disposals);
    }

    [Fact]
    public void SignificantZeroWithNoTerminalPathsRequiresActualWholeTargetCertificate()
    {
        var fixture = Ramp(); var capture = new Capture();
        var actual = Run(fixture, true, Control(capture), capture.Logs.Add);
        Assert.True(capture.Owner!.CompletedComponents > 0);
        var complete = Assert.Single(capture.Completions);
        Assert.True(complete.SignificantEmpty); Assert.Equal(0, complete.TerminalPathCount);
        Assert.True(complete.NominatedComponents > 0); Assert.True(complete.Certificate.Certified);
        Assert.Equal(capture.FixedTargets[0], TargetBits(complete.Authority.Target));
        Assert.False(complete.OriginalCertificateRequested);
        Assert.False(complete.Authority.Limits.RecordWitnessPathIndices);
        Assert.Null(complete.Authority.Limits.OptionalWitnessPathIndices);
        Assert.Single(capture.Cores); Assert.True(capture.Cores[0].UseGpuFronts);
        Assert.Equal(1, capture.Factories); Assert.Equal(1, capture.Lifetime.Disposals);
        Assert.DoesNotContain(capture.Logs, message => message.Contains("event=completion-fallback"));
        VerifyOriginalPhysicalGuards(fixture, actual);
    }

    [Fact]
    public void TinyActualCertificateBudgetRetriesCompleteOriginalCoreExactlyOnceWithGpuExplicitlyFalse()
    {
        var fixture = Ramp(); var baseline = Run(fixture, false, null, null);
        var capture = new Capture();
        var actual = Run(fixture, true, Control(capture, forceInconclusive: true), capture.Logs.Add);
        var complete = Assert.Single(capture.Completions);
        Assert.True(complete.SignificantEmpty); Assert.Equal(0, complete.TerminalPathCount);
        Assert.True(complete.NominatedComponents > 0); Assert.False(complete.Certificate.Certified);
        Assert.Equal("box-budget", complete.Certificate.Reason); Assert.Equal(1, complete.Authority.Limits.MaximumBoxes);
        Assert.Equal(new[] { true, false }, capture.Cores.Select(core => core.UseGpuFronts));
        foreach (var core in capture.Cores)
        {
            Assert.Same(fixture.Source, core.Source); Assert.Same(fixture.Boundary, core.Boundary);
            Assert.Same(fixture.Tool, core.Tool); Assert.Same(fixture.Schedule, core.Schedule);
            Assert.Same(fixture.Options, core.Options); Assert.Null(core.Preparation); Assert.Equal(4, core.CorrectionWorkers);
        }
        Assert.Equal(capture.FixedTargets[0], capture.FixedTargets[1]);
        Assert.Single(capture.Logs.Where(message => message.Contains("event=completion-fallback")));
        Assert.Equal(1, capture.Factories); Assert.Equal(1, capture.Lifetime.Disposals);
        EqualGeometryBits(baseline, actual);
    }

    [Fact]
    public void HoledDisconnectedOriginalTargetIsRetainedBitForBitAtCompletion()
    {
        var fixture = Ramp(holed: true); var capture = new Capture();
        var actual = Run(fixture, true, Control(capture), capture.Logs.Add);
        Assert.True(capture.Owner!.CompletedComponents > 0); Assert.NotEmpty(capture.Completions);
        foreach (var complete in capture.Completions)
        {
            Assert.Equal(capture.FixedTargets[0], TargetBits(complete.Authority.Target));
            Assert.False(PolygonOps.Contains(complete.Authority.Target, 5, 5));
            Assert.Contains(complete.Authority.Target.SelectMany(ring => ring), point => point.x > 12);
            Assert.True(complete.Certificate.Certified);
            // CPU terminal rays also use category=clearing. Category cannot
            // identify nomination ownership; the separate terminal0 control
            // verifies accepted GPU paths do not acquire optional ownership.
        }
        Assert.Equal(1, capture.Lifetime.Disposals); VerifyOriginalPhysicalGuards(fixture, actual);
    }

    [Fact]
    public void IndependentBadCentersStillReceiveRealFloorBothWallsAndGradeProtection()
    {
        var fixture = Ramp(); var capture = new Capture();
        var actual = Run(fixture, true, Control(capture, adversarial: true), capture.Logs.Add);
        Assert.True(capture.Owner!.CompletedComponents > 0);
        Assert.Contains(capture.Logs, message => message.Contains("stage=smooth-command-centers") && message.Contains("domains=2"));
        Assert.Contains(capture.Logs, message => message.Contains("stage=smooth-command-wall") && message.Contains("domains=2"));
        Assert.Contains(capture.Logs, message => message.Contains("stage=smooth-command-floor"));
        VerifyOriginalPhysicalGuards(fixture, actual);
    }

    [Fact]
    public void GpuOnlyCompletionWithSmallOriginalTerminalCohortNeverPrunesSurfaceWitnessRays()
    {
        var fixture = RoundedRamp(); var capture = new Capture();
        var actual = Run(fixture, true, Control(capture, adversarial: true), capture.Logs.Add);
        var complete = Assert.Single(capture.Completions.Where(item => item.SignificantEmpty
            && item.TerminalPathCount is > 0 and < 1024));
        Assert.True(complete.NominatedComponents > 0); Assert.False(complete.OriginalCertificateRequested);
        Assert.True(complete.Certificate.Certified); Assert.False(complete.Authority.Limits.RecordWitnessPathIndices);
        Assert.Null(complete.Authority.Limits.OptionalWitnessPathIndices);
        Assert.False(complete.Certificate.WitnessPathsRecorded);
        Assert.DoesNotContain(capture.Logs, message => message.Contains("stage=smooth-terminal-pruning event=complete"));
        // No pruning occurred between the actual certificate snapshot and the
        // successful return. Compare the entire commanded chord snapshot,
        // including every original terminal ray, independent of output order.
        static string Key(TaggedToolpath path) => string.Join(",", path.Points.SelectMany(point => new[]
            { BitConverter.DoubleToInt64Bits(point.X), BitConverter.DoubleToInt64Bits(point.Y), BitConverter.DoubleToInt64Bits(point.Z) }));
        var returned = VCarveSmoothing.CommandPaths(actual, fixture.Schedule.SurfaceZ).Select(Key).OrderBy(key => key).ToArray();
        Assert.Equal(complete.Authority.CommandPaths.Select(Key).OrderBy(key => key), returned);
        Assert.Equal(1, capture.Lifetime.Disposals);
    }

    [Fact]
    public void ExceptionFromActualProtectionDoesNotBecomeGeometryFallback()
    {
        var fixture = Ramp(); var capture = new Capture();
        var expected = new InvalidOperationException("Deliberate failure at actual two-domain command protection.");
        bool nominated = false;
        var actual = Assert.Throws<InvalidOperationException>(() => Run(fixture, true, Control(capture), message =>
        {
            capture.Logs.Add(message);
            nominated |= message.Contains("event=gpu-front-nomination");
            if (nominated && message.Contains("stage=smooth-command-wall")) throw expected;
        }));
        Assert.Same(expected, actual); Assert.True(nominated);
        Assert.Single(capture.Cores); Assert.Equal(1, capture.Lifetime.Disposals);
        Assert.DoesNotContain(capture.Logs, message => message.Contains("event=completion-fallback"));
    }

    [Fact]
    public void UnavailableOwnerRunsOriginalConstructorAndProducesOriginalResult()
    {
        var fixture = Ramp(); var baseline = Run(fixture, false, null, null); var capture = new Capture();
        var control = new VCarveSmoothing.GpuFrontIntegrationControl(
            NominationOwnerFactory: (_, token, _) => { token.ThrowIfCancellationRequested(); capture.Factories++; return null; },
            CoreEntered: capture.Cores.Add);
        var actual = Run(fixture, true, control, capture.Logs.Add);
        EqualGeometryBits(baseline, actual); Assert.Equal(1, capture.Factories); Assert.Single(capture.Cores);
        Assert.Contains(capture.Logs, message => message.Contains("event=inset-begin"));
        Assert.DoesNotContain(capture.Logs, message => message.Contains("event=gpu-front-nomination"));
    }

    [Theory]
    [InlineData("empty")]
    [InlineData("default")]
    [InlineData("late-failure")]
    [InlineData("runtime")]
    public void RefusedNominationExecutesOriginalConstructorInsteadOfDeclaringUnreachable(string mode)
    {
        var fixture = Ramp(); var baseline = Run(fixture, false, null, null); var capture = new Capture();
        var actual = Run(fixture, true, Control(capture, producerMode: mode), capture.Logs.Add);
        Assert.True(capture.ProducerCalls > 0); Assert.Equal(0, capture.Owner!.CompletedComponents);
        Assert.True(capture.Owner.RefusedComponents > 0); Assert.Equal(1, capture.Lifetime.Disposals);
        Assert.Contains(capture.Logs, message => message.Contains("event=gpu-front-refusal"));
        Assert.Contains(capture.Logs, message => message.Contains("event=inset-begin"));
        Assert.Empty(capture.Completions); EqualGeometryBits(baseline, actual);
    }

    [Fact]
    public void ThrowingSelectionDiagnosticDisposesNewOwnerWithoutAnOriginalRetry()
    {
        var fixture = Ramp(); var capture = new Capture();
        var expected = new InvalidOperationException("Selection observer failed.");
        var actual = Assert.Throws<InvalidOperationException>(() => Run(fixture, true, Control(capture), message =>
        {
            if (message.Contains("event=selection requested=True")) throw expected;
        }));
        Assert.Same(expected, actual); Assert.Single(capture.Cores);
        Assert.Equal(1, capture.Factories); Assert.Equal(1, capture.Lifetime.Disposals);
    }

    [Theory]
    [InlineData("selection")]
    [InlineData("producer")]
    [InlineData("certificate")]
    [InlineData("fallback")]
    [InlineData("apply-complete")]
    public void CallerCancellationPreventsPublicationAndDisposesOwnerOnce(string stage)
    {
        var fixture = Ramp(); var capture = new Capture(); using var cancellation = new CancellationTokenSource();
        var control = Control(capture, forceInconclusive: stage == "fallback", producerCanceled: stage == "producer" ? cancellation : null);
        if (stage == "certificate") control = control with { CompletionObserved = complete =>
            { capture.Completions.Add(complete); cancellation.Cancel(); } };
        Assert.ThrowsAny<OperationCanceledException>(() => Run(fixture, true, control, message =>
        {
            capture.Logs.Add(message);
            if (stage == "selection" && message.Contains("event=selection requested=True")
                || stage == "fallback" && message.Contains("event=completion-fallback")
                || stage == "apply-complete" && message.Contains("event=apply-complete")) cancellation.Cancel();
        }, cancellation.Token));
        Assert.Equal(1, capture.Factories); Assert.Equal(1, capture.Lifetime.Disposals);
        if (stage == "fallback") Assert.Single(capture.Cores);
    }

    private static VCarveSmoothing.GpuFrontIntegrationControl Control(Capture capture,
        bool forceInconclusive = false, bool adversarial = false, CancellationTokenSource? producerCanceled = null, string? producerMode = null)
        => new(NominationOwnerFactory: (channel, token, diagnostic) =>
        {
            capture.Factories++;
            Assert.True(VCarveGpuDistanceFrontNominator.BoundarySnapshot.TryPrepare(channel, token, out var snapshot, out string reason), reason);
            bool UnusedSampler(GpuBoundaryQuery[] _, GpuBoundarySampleNomination[] __, int ___,
                CancellationToken ct, out string detail) { ct.ThrowIfCancellationRequested(); detail = "control-producer-uses-literal-offsets"; return false; }
            var copied = channel.Select(ring => (IReadOnlyList<PointD>)ring.ToArray()).ToArray();
            capture.Owner = VCarveGpuCorrectionFrontNominator.CreateForNominationControls(snapshot!, UnusedSampler,
                capture.Lifetime, (_, settings, _, ct, _) =>
                {
                    capture.ProducerCalls++; producerCanceled?.Cancel(); ct.ThrowIfCancellationRequested();
                    if (producerMode == "runtime") throw new InvalidOperationException("Independent recoverable nomination failure.");
                    if (producerMode is "empty" or "default" or "late-failure")
                        return new(producerMode != "late-failure", "deliberate-" + producerMode,
                            producerMode == "default" ? default : ImmutableArray<VCarveGpuDistanceFrontNominator.Fragment>.Empty,
                            0, 0, 0, 0, 0, 0, 0) { CanonicalFieldIncidenceMatched = true };
                    return LiteralFronts(copied, settings, adversarial, ct);
                }, diagnostic);
            return capture.Owner;
        }, ForceCompletionCertificateInconclusive: forceInconclusive,
        CoreEntered: capture.Cores.Add, FixedTargetPrepared: target => capture.FixedTargets.Add(TargetBits(target)),
        CompletionObserved: capture.Completions.Add);

    private static VCarveGpuDistanceFrontNominator.Result LiteralFronts(IReadOnlyList<IReadOnlyList<PointD>> channel,
        VCarveGpuDistanceFrontNominator.Settings settings, bool adversarial, CancellationToken token)
    {
        var fragments = ImmutableArray.CreateBuilder<VCarveGpuDistanceFrontNominator.Fragment>();
        if (!adversarial)
        {
            for (int index = 0; index < settings.FrontCount; index++)
            {
                token.ThrowIfCancellationRequested(); double offset = settings.FirstOffset + index * settings.Pitch;
                var rings = PolygonOps.Offset(channel, -offset, arcTolerance: 1 / PathUtils.Scale, simplifyTolerance: 0);
                if (rings.Count == 0) break;
                foreach (var ring in PathUtils.ToClipper(rings))
                for (int edge = 0; edge < ring.Count; edge++)
                {
                    var a = ring[edge]; var b = ring[(edge + 1) % ring.Count];
                    if (Clip(a, b, settings.Bounds, out var ca, out var cb) && ca != cb)
                        fragments.Add(new(ca, cb, index, offset));
                }
                Assert.True(fragments.Count <= settings.MaximumFragments, "Control exceeded nomination fragment admission.");
            }
        }
        if (adversarial || fragments.Count == 0)
        {
            // Intentionally close to the expanded error window wall. Declared
            // offset/incidence is only a nomination, never cutting authority.
            var a = new Point64(settings.Bounds.left, settings.Bounds.top);
            var b = new Point64(Math.Min(settings.Bounds.right, settings.Bounds.left + 3277), settings.Bounds.top);
            fragments.Add(new(a, b, 0, settings.FirstOffset));
        }
        return new(true, "independent-literal-front-nominations", fragments.ToImmutable(), 0, 0, 0, 0, 0, 0, 0)
            { CanonicalFieldIncidenceMatched = true };
        // Independent finite-segment clipping for test nominations. Dropping
        // a long edge whose endpoints lie outside the local error window would
        // fail to nominate its actual crossing and miss the intended branch.
        static bool Clip(Point64 a, Point64 b, Rect64 r, out Point64 ca, out Point64 cb)
        {
            double dx = b.X - a.X, dy = b.Y - a.Y, first = 0, last = 1;
            bool Half(double p, double q)
            {
                if (p == 0) return q >= 0;
                double t = q / p;
                if (p < 0) { if (t > last) return false; first = Math.Max(first, t); }
                else { if (t < first) return false; last = Math.Min(last, t); }
                return true;
            }
            ca = cb = default;
            if (!Half(-dx, a.X - r.left) || !Half(dx, r.right - a.X)
                || !Half(-dy, a.Y - r.top) || !Half(dy, r.bottom - a.Y)) return false;
            Point64 At(double t) => new(Math.Clamp((long)Math.Round(a.X + t * dx), r.left, r.right),
                Math.Clamp((long)Math.Round(a.Y + t * dy), r.top, r.bottom));
            ca = At(first); cb = At(last); return true;
        }
    }

    [Fact]
    public void CapturedNominationPolicySurvivesOriginalRetryAndEnvironmentMutation()
    {
        // Full interior work reaches correction/recovery. The default bounded
        // no-fill fixture completes before those stages and cannot test a retry.
        var fixture = Ramp();
        fixture = fixture with { Options = fixture.Options with { IncludeInteriorFill = true } };
        var baseline = Run(fixture, false, null, null, contourSeeds: true);
        var capture = new Capture();
        var control = Control(capture, forceInconclusive: true) with {
            CoreEntered = input => {
                capture.Cores.Add(input);
                Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_GPU_FRONTS", "0");
                Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_CONTOUR_SEEDS", "0");
            }
        };
        var actual = Run(fixture, true, control, capture.Logs.Add, contourSeeds: true);
        Assert.Equal(new[] { true, false }, capture.Cores.Select(core => core.UseGpuFronts));
        var complete = Assert.Single(capture.Completions);
        Assert.False(complete.Certificate.Certified);
        var policy = Assert.IsType<VCarveGenerationPolicy>(complete.Authority.Limits.GenerationPolicy);
        Assert.True(policy.GpuFronts);
        Assert.True(policy.ContourSeeds);
        Assert.True(complete.Authority.Limits.UseContourBoxSeeds);
        Assert.Single(capture.Logs.Where(message => message.Contains("event=completion-fallback")));
        EqualGeometryBits(baseline, actual);
        Assert.Equal(1, capture.Factories);
        Assert.Equal(1, capture.Lifetime.Disposals);
        _output.WriteLine("capturedPolicySurvivesOriginalRetry=True environmentMutationIgnored=True originalGeometryBitsPreserved=True");
    }

    private List<TaggedToolpath> Run(Fixture fixture, bool gpu,
        VCarveSmoothing.GpuFrontIntegrationControl? control, Action<string>? diagnostic, CancellationToken token = default,
        bool floorAware = false, bool contourSeeds = false, bool tapered = false, bool lowerSweepBudget = false, bool boxNeighborhoods = false)
    {
        var isolatedFlags = new[] { "RW_TOOLPATHS_VCARVE_GPU_FRONTS", "RW_TOOLPATHS_VCARVE_GPU",
            "RW_TOOLPATHS_VCARVE_GPU_TRIANGLES", "RW_TOOLPATHS_VCARVE_GPU_BOUNDARY",
            "RW_TOOLPATHS_VCARVE_GPU_FP32", "RW_TOOLPATHS_VCARVE_GPU_BOUNDARY_FP32",
            "RW_TOOLPATHS_VCARVE_FLOOR_AWARE_RAMPS", "RW_TOOLPATHS_VCARVE_CONTOUR_SEEDS",
            "RW_TOOLPATHS_GPU_GOLDEN_REFERENCE",
            "RW_TOOLPATHS_VCARVE_TAPERED_SWEEPS", "RW_TOOLPATHS_VCARVE_LOWER_SWEEP_BUDGET",
            "RW_TOOLPATHS_VCARVE_ACTUAL_F3_COMPACTION", "RW_TOOLPATHS_VCARVE_BOX_NEIGHBORHOODS" };
        var previousEnvironment = isolatedFlags.ToDictionary(flag => flag, Environment.GetEnvironmentVariable);
        var previousControl = VCarveSmoothing.GpuFrontIntegrationControls.Value;
        var runLog = new List<string>();
        try
        {
            foreach (string flag in isolatedFlags) Environment.SetEnvironmentVariable(flag, "0");
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_GPU_FRONTS", gpu ? "1" : "0");
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_FLOOR_AWARE_RAMPS", floorAware ? "1" : "0");
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_CONTOUR_SEEDS", contourSeeds ? "1" : "0");
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_TAPERED_SWEEPS", tapered ? "1" : "0");
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_LOWER_SWEEP_BUDGET", lowerSweepBudget ? "1" : "0");
            Environment.SetEnvironmentVariable("RW_TOOLPATHS_VCARVE_BOX_NEIGHBORHOODS", boxNeighborhoods ? "1" : "0");
            VCarveSmoothing.GpuFrontIntegrationControls.Value = control;
            return VCarveSmoothing.Apply(fixture.Source, fixture.Boundary, fixture.Tool, fixture.Schedule,
                fixture.Options, token, message => { runLog.Add(message); diagnostic?.Invoke(message); }, correctionWorkers: 4);
        }
        finally
        {
            VCarveSmoothing.GpuFrontIntegrationControls.Value = previousControl;
            foreach (var previous in previousEnvironment) Environment.SetEnvironmentVariable(previous.Key, previous.Value);
            _output.WriteLine(string.Join(Environment.NewLine, runLog));
        }
    }

    private static Fixture Ramp(bool holed = false)
    {
        var source = new List<TaggedToolpath>();
        var boundary = new List<IReadOnlyList<PointD>> { Rectangle(0, 0, 10, holed ? 10 : 4) };
        if (!holed) source.Add(Path(new(1, 2, -.2), new(4, 2, -3), new(8, 2, -3)));
        else
        {
            source.Add(Path(new(2, 2, -.2), new(8, 2, -3), new(8, 8, -3), new(2, 8, -3), new(2, 2, -.2)));
            source.Add(Path(new(15, 2, -.2), new(18, 2, -3), new(22, 2, -3)));
            boundary.Add(Rectangle(14, 0, 24, 4));
            boundary.Add(Rectangle(4, 4, 6, 6).Reverse().ToArray());
        }
        var tool = new ToolGeometry(5) { BottomRadius = 0, TopRadius = 5, ConeLength = 10,
            TipAngleRadians = 2 * Math.Atan(.5) };
        return new(source, boundary, tool, new DepthSchedule(3, 1), new VCarveOptions
        {
            SmoothVCarving = true, SmoothRampAngleDegrees = 15, IncludeInteriorFill = false,
            DetectCurves = false, StepOver = .4,
            CorrectionArcTolerance = VCarveSmoothing.ReferenceCorrectionArcTolerance,
            UseOrdinaryCorrectionCandidates = false
        });
        static TaggedToolpath Path(params Point3D[] points)
            => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
    }

    private static Fixture RoundedRamp()
    {
        var ramp = Ramp();
        var sections = new[] { new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, .05, .05),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, .05, .5, 1) };
        var profile = new VCarveCutterProfile(sections, 1.05);
        var tool = new ToolGeometry(.5) { VCarveProfile = profile, BottomRadius = .05, TopRadius = .5,
            ConeLength = 1, TipAngleRadians = 2 * Math.Atan(.45) };
        var source = ramp.Source.Select(path => new TaggedToolpath(path.Points.Select(point =>
            new Point3D(point.X * .1, point.Y * .1, point.Z * .1)).ToArray(), Array.Empty<ToolpathSpan>(),
            path.RegionIndex, path.Category, path.DepthPassIndex, detectCurves: false)).ToArray();
        var boundary = ramp.Boundary.Select(ring => (IReadOnlyList<PointD>)ring.Select(point =>
            new PointD(point.x * .1, point.y * .1)).ToArray()).ToArray();
        return new(source, boundary, tool, new DepthSchedule(.3, .1), ramp.Options);
    }

    private static void VerifyOriginalPhysicalGuards(Fixture fixture, IReadOnlyList<TaggedToolpath> actual)
    {
        double grade = Math.Tan(fixture.Options.SmoothRampAngleDegrees * Math.PI / 180);
        Assert.True(VCarveSmoothing.MaximumGrade(actual) <= grade + 1e-10);
        var profile = VCarvePhysicalSweep.ProfileFor(fixture.Tool, fixture.Schedule.Depth);
        var originalChannel = VCarvePhysicalSweep.Build(VCarveSmoothing.CommandPaths(fixture.Source, fixture.Schedule.SurfaceZ),
            fixture.Tool, fixture.Schedule.Depth, 0, .00075, CancellationToken.None, upperBound: true);
        var limited = VCarveSmoothing.LimitGraph(fixture.Source, grade, CancellationToken.None);
        var anchors = new List<TaggedToolpath>();
        for (int path = 0; path < fixture.Source.Count; path++)
        for (int i = 1; i < fixture.Source[path].Points.Count; i++)
            if (limited[path].Points[i - 1].Z > fixture.Source[path].Points[i - 1].Z + 1e-10
                || limited[path].Points[i].Z > fixture.Source[path].Points[i].Z + 1e-10)
                anchors.Add(new(new[] { limited[path].Points[i - 1], limited[path].Points[i] },
                    Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false));
        var field = new VCarveSmoothing.FloorField(anchors, profile, grade, fixture.Schedule.Depth, CancellationToken.None);
        foreach (var path in actual)
        {
            foreach (var p in path.Points)
                Assert.True(-p.Z <= field.CappedDepth(-p.Z, p.X, p.Y) + 1e-8, "Nomination bypassed original full-tool floor.");
            var commanded = VCarveSmoothing.CommandPaths(new[] { path }, fixture.Schedule.SurfaceZ)[0];
            for (int chord = 1; chord < commanded.Points.Count; chord++)
            {
                var a = commanded.Points[chord - 1]; var b = commanded.Points[chord];
                for (int sample = 0; sample <= 32; sample++)
                {
                    double t = sample / 32.0, x = a.X + (b.X - a.X) * t, y = a.Y + (b.Y - a.Y) * t;
                    double depth = -(a.Z + (b.Z - a.Z) * t); if (depth <= 0) continue;
                    double radius = profile.PhysicalRadiusAtDepth(depth);
                    Assert.True(SignedClearance(fixture.Boundary, x, y) + 1e-8 >= radius, "Nomination bypassed artwork wall.");
                    Assert.True(SignedClearance(originalChannel, x, y) + 1e-8 >= radius, "Nomination widened original commanded channel.");
                }
            }
        }
    }
    private static double SignedClearance(IReadOnlyList<IReadOnlyList<PointD>> rings, double x, double y)
    {
        double closest = double.PositiveInfinity; int winding = 0;
        foreach (var ring in rings)
        for (int i = 0; i < ring.Count; i++)
        {
            var a = ring[i]; var b = ring[(i + 1) % ring.Count];
            double dx = b.x - a.x, dy = b.y - a.y, square = dx * dx + dy * dy;
            double t = square > 0 ? Math.Clamp(((x - a.x) * dx + (y - a.y) * dy) / square, 0, 1) : 0;
            double ex = x - a.x - t * dx, ey = y - a.y - t * dy;
            closest = Math.Min(closest, Math.Sqrt(ex * ex + ey * ey));
            double cross = dx * (y - a.y) - dy * (x - a.x);
            if (a.y <= y && b.y > y && cross > 0) winding++;
            else if (a.y > y && b.y <= y && cross < 0) winding--;
        }
        return winding == 0 ? -closest : closest;
    }
    private static long[][] TargetBits(IReadOnlyList<IReadOnlyList<PointD>> target)
        => target.Select(ring => ring.SelectMany(point => new[] { BitConverter.DoubleToInt64Bits(point.x),
            BitConverter.DoubleToInt64Bits(point.y) }).ToArray()).ToArray();
    private static void EqualGeometryBits(IReadOnlyList<TaggedToolpath> expected, IReadOnlyList<TaggedToolpath> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int path = 0; path < expected.Count; path++)
        {
            Assert.Equal(expected[path].Category, actual[path].Category);
            Assert.Equal(expected[path].RegionIndex, actual[path].RegionIndex);
            Assert.Equal(expected[path].DepthPassIndex, actual[path].DepthPassIndex);
            Assert.Equal(expected[path].VisitIdentity, actual[path].VisitIdentity);
            Assert.Equal(expected[path].Spans, actual[path].Spans); Assert.Equal(expected[path].Moves, actual[path].Moves);
            Assert.Equal(expected[path].Points.SelectMany(p => new[] { BitConverter.DoubleToInt64Bits(p.X),
                BitConverter.DoubleToInt64Bits(p.Y), BitConverter.DoubleToInt64Bits(p.Z) }),
                actual[path].Points.SelectMany(p => new[] { BitConverter.DoubleToInt64Bits(p.X),
                BitConverter.DoubleToInt64Bits(p.Y), BitConverter.DoubleToInt64Bits(p.Z) }));
        }
    }
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => new[] { new PointD(x0, y0), new PointD(x1, y0), new PointD(x1, y1), new PointD(x0, y1) };
}
