using Clipper2Lib;
using System.Reflection;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class VCarveGpuCoverageCertificateTests(ITestOutputHelper output)
{
    [Theory]
    [InlineData("disc-interior", true)]
    [InlineData("sphere-ramp", true)]
    [InlineData("sphere-reversed", true)]
    [InlineData("profile-transition", true)]
    [InlineData("surface-crossing", true)]
    [InlineData("retained-hole", true)]
    [InlineData("filled-hole", false)]
    [InlineData("missing-component", false)]
    [InlineData("multiple-components", true)]
    [InlineData("last-component-missed", false)]
    [InlineData("uncut-channel", false)]
    [InlineData("air-only", false)]
    [InlineData("retract-only", false)]
    [InlineData("translated-surface", true)]
    [InlineData("mandatory-owner", true)]
    public void GpuNominationsPreserveWholeTargetPhysicalOutcomes(string scenario, bool expectedCertified)
    {
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(-.2, -.2, .2, .2)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, -.1), new(0, 0, -.1))];
        VCarveCutterProfile profile = Cylinder(1);
        double surface = 0;
        var limits = Limits();
        switch (scenario)
        {
            case "sphere-ramp":
            case "sphere-reversed":
                profile = BallTaper();
                target = [Rectangle(.49, .033, .51, .036)];
                Point3D a = new(0, 0, 0), b = new(1, 0, -.001);
                paths = [scenario == "sphere-reversed" ? Path(b, a) : Path(a, b)];
                break;
            case "profile-transition":
                profile = new([new(CarvingCutterSectionKind.Linear, .1, .6, .5),
                    new(CarvingCutterSectionKind.Linear, .6, 1.6, 1)], 1.5);
                target = [Rectangle(.35, -.15, .65, .15)];
                paths = [Path(new(-1, 0, .1), new(1, 0, -1.5))];
                break;
            case "surface-crossing":
                profile = Cylinder(.5);
                target = [Rectangle(.4, -.1, .6, .1)];
                paths = [Path(new(-1, 0, .2), new(1, 0, -.2))];
                break;
            case "retained-hole":
            case "filled-hole":
                var outer = Rectangle(-.3, -.3, .3, .3);
                var hole = Rectangle(-.1, -.1, .1, .1).Reverse().ToArray();
                target = scenario == "retained-hole" ? [outer, hole] : [outer];
                profile = Cylinder(.15);
                paths = [Path(new(-.2, -.2, -.1), new(.2, -.2, -.1), new(.2, .2, -.1),
                    new(-.2, .2, -.1), new(-.2, -.2, -.1))];
                break;
            case "missing-component":
                target = [Rectangle(-.2, -.2, .2, .2), Rectangle(5, 5, 5.02, 5.02)];
                break;
            case "multiple-components":
            case "last-component-missed":
                profile = Cylinder(.5);
                target = Enumerable.Range(0, 24).Select(index =>
                    (IReadOnlyList<PointD>)Rectangle(3 * index - .2, -.2, 3 * index + .2, .2)).Reverse().ToArray();
                paths = Enumerable.Range(0, scenario == "last-component-missed" ? 23 : 24)
                    .Select(index => Path(new(3 * index, 0, -.1), new(3 * index, 0, -.1))).Reverse().ToArray();
                break;
            case "uncut-channel":
                target = [Rectangle(-1, -1, 1, 1)];
                profile = Cylinder(.2);
                paths = [Path(new(-1, 0, -.1), new(1, 0, -.1))];
                break;
            case "air-only":
                paths = [Path(new(-1, 0, 0), new(1, 0, 0))];
                break;
            case "retract-only":
                paths = [new TaggedToolpath([new(-1, 0, -.1), new(1, 0, -.1)],
                    [new(0, 1, ToolpathSpanKind.Retract)], 0, "final-carve", null, detectCurves: false)];
                break;
            case "translated-surface":
                paths = [Path(new(-1, 0, 9.9), new(1, 0, 9.9))];
                surface = 10;
                break;
            case "mandatory-owner":
                paths = [Path(new(0, 0, -.1), new(0, 0, -.1)), Path(new(0, 0, -.1), new(0, 0, -.1))];
                limits = limits with { OptionalWitnessPathIndices = new HashSet<int> { 0 } };
                break;
        }
        var cpu = Certify(target, paths, profile, surface, limits with { UseGpuDiscWitnesses = false });
        var gpu = Certify(target, paths, profile, surface, limits with { UseGpuDiscWitnesses = true });
        Assert.Equal(expectedCertified, cpu.Certified);
        Assert.Equal(cpu.Certified, gpu.Certified);
        Assert.Equal(cpu.BoundarySegments, gpu.BoundarySegments);
        Assert.Equal(cpu.CommandIntervals, gpu.CommandIntervals);
        Assert.Equal(cpu.CommandPathCount, gpu.CommandPathCount);
        if (expectedCertified)
        {
            Assert.Equal("whole-target-certified", gpu.Reason);
            Assert.True(gpu.WitnessPathsRecorded);
            Assert.All(gpu.WitnessPathIndices, owner => Assert.InRange(owner, 0, paths.Count - 1));
        }
        if (scenario == "mandatory-owner") Assert.Equal(new[] { 1 }, gpu.WitnessPathIndices);
        if (scenario is "air-only" or "retract-only") Assert.Equal(0, gpu.CommandIntervals);
        if (!gpu.GpuAvailable) Assert.Equal(0, gpu.GpuWitnesses);
    }

    [Theory]
    [InlineData("box-budget")]
    [InlineData("disc-budget")]
    [InlineData("input-budget")]
    [InlineData("time-budget")]
    public void GpuWorkCannotConvertQuotaExhaustionIntoCertificate(string reason)
    {
        var limits = Limits() with { UseGpuDiscWitnesses = true };
        limits = reason switch
        {
            "box-budget" => limits with { MaximumBoxes = 0 },
            "disc-budget" => limits with { MaximumDiscEvaluations = 0 },
            "input-budget" => limits with { MaximumInputSegments = 3 },
            "time-budget" => limits with { MaximumElapsed = TimeSpan.FromTicks(1) },
            _ => throw new ArgumentOutOfRangeException(nameof(reason))
        };
        var result = Certify([Rectangle(-.2, -.2, .2, .2)],
            [Path(new(0, 0, -.1), new(0, 0, -.1))], Cylinder(1), 0, limits);
        Assert.False(result.Certified);
        Assert.Equal(reason, result.Reason);
        Assert.False(result.WitnessPathsRecorded);
        Assert.Empty(result.WitnessPathIndices);
    }

    [Theory]
    [InlineData(-1e-12, false)]
    [InlineData(1e-12, true)]
    public void GpuPositiveRetainsCpuArithmeticReserve(double margin, bool expectedCertified)
    {
        double radius = Math.Sqrt(.2 * .2 + .2 * .2) - .004
            + VCarveCoverageCertificate.ArithmeticReserve + margin;
        var result = Certify([Rectangle(-.2, -.2, .2, .2)],
            [Path(new(0, 0, -.1), new(0, 0, -.1))], Cylinder(radius), 0,
            Limits() with { UseGpuDiscWitnesses = true, MaximumBoxes = 1 });
        Assert.Equal(expectedCertified, result.Certified);
        Assert.Equal(expectedCertified ? "whole-target-certified" : "box-budget", result.Reason);
        Assert.Equal(1, result.VisitedBoxes);
    }

    [Fact]
    public void AlreadyCancelledGpuRequestPropagatesCancellation()
    {
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveCoverageCertificate.Certify(
            [Rectangle(-.2, -.2, .2, .2)], [Path(new(0, 0, -.1), new(0, 0, -.1))], Cylinder(1),
            cancellationToken: cancellation.Token, limits: Limits() with { UseGpuDiscWitnesses = true },
            diagnostic: output.WriteLine));
    }

    [Fact]
    public void GpuRequestRetainsMalformedInputRejection()
    {
        var limits = Limits() with { UseGpuDiscWitnesses = true };
        Assert.Throws<ArgumentException>(() => Certify([[new(0, 0), new(1, 1)]], [], Cylinder(1), 0, limits));
        Assert.Throws<ArgumentException>(() => Certify([Rectangle(-.2, -.2, .2, .2)],
            [Path(new(double.NaN, 0, -.1), new(0, 0, -.1))], Cylinder(1), 0, limits));
        Assert.Throws<ArgumentOutOfRangeException>(() => Certify([Rectangle(-.2, -.2, .2, .2)],
            [Path(new(0, 0, -.1), new(0, 0, -.1))], Cylinder(1), 0,
            limits with { OptionalWitnessPathIndices = new HashSet<int> { 1 } }));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(7)]
    [InlineData(64)]
    public void BatchSizeDoesNotChangePhysicalCoverageOrOwnerEligibility(int batchSize)
    {
        var target = Enumerable.Range(0, 24).Select(index =>
            (IReadOnlyList<PointD>)Rectangle(3 * index - .2, -.2, 3 * index + .2, .2)).ToArray();
        var paths = Enumerable.Range(0, 24).Select(index =>
            Path(new(3 * index, 0, -.1), new(3 * index, 0, -.1))).ToArray();
        var result = Certify(target, paths, Cylinder(.5), 0,
            Limits() with { UseGpuDiscWitnesses = true, GpuBatchSize = batchSize });
        Assert.True(result.Certified, result.ToString());
        Assert.Equal(Enumerable.Range(0, 24).ToArray(), result.WitnessPathIndices);
        if (result.GpuAvailable) Assert.True(result.GpuBatches > 0);
    }

    [Fact]
    public void BelowThresholdRetainsReferenceSearchWithoutGpuInitialization()
    {
        var result = Certify([Rectangle(-.2, -.2, .2, .2)],
            [Path(new(0, 0, -.1), new(0, 0, -.1))], Cylinder(1), 0,
            Limits() with { UseGpuDiscWitnesses = true, MinimumGpuIntervals = 2 });
        Assert.True(result.Certified);
        Assert.False(result.GpuAttempted);
        Assert.False(result.GpuAvailable);
        Assert.Equal("below-interval-threshold", result.GpuReason);
        Assert.Equal(0, result.GpuQueries);
        Assert.Equal(0, result.GpuBatches);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(-1, 1)]
    [InlineData(1, 0)]
    [InlineData(1, -1)]
    [InlineData(1, 4097)]
    public void MalformedGpuLimitsCannotEnterDeviceWork(int minimumIntervals, int batchSize)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => Certify([Rectangle(-.2, -.2, .2, .2)],
            [Path(new(0, 0, -.1), new(0, 0, -.1))], Cylinder(1), 0,
            Limits() with { UseGpuDiscWitnesses = true, MinimumGpuIntervals = minimumIntervals, GpuBatchSize = batchSize }));
    }

    [Theory]
    [InlineData("outside", false, true)]
    [InlineData("inside", true, true)]
    [InlineData("retained-hole", false, true)]
    [InlineData("hole-crossing", false, false)]
    [InlineData("vertex-contact", false, false)]
    [InlineData("horizontal-contact", false, false)]
    [InlineData("near-horizontal-contact", false, false)]
    [InlineData("separated-horizontal", false, true)]
    [InlineData("diagonal-crossing", false, false)]
    [InlineData("diagonal-aabb-refusal", false, false)]
    [InlineData("between-components", false, true)]
    [InlineData("encloses-components", false, false)]
    public void TargetBoxExclusionRetainsOriginalBoundaryContacts(string scenario, bool expectedInside, bool expectedExclusion)
    {
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(0, 0, 2, 2)];
        (double x0, double y0, double x1, double y1) bounds = (3, 3, 4, 4);
        switch (scenario)
        {
            case "inside": bounds = (.7, .7, 1.3, 1.3); break;
            case "retained-hole":
            case "hole-crossing":
                target = [Rectangle(0, 0, 2, 2), Rectangle(.5, .5, 1.5, 1.5).Reverse().ToArray()];
                bounds = scenario == "retained-hole" ? (.7, .7, 1.3, 1.3) : (.4, .8, .8, 1.2);
                break;
            case "vertex-contact": bounds = (2, 2, 2.2, 2.2); break;
            case "horizontal-contact": bounds = (.8, 2, 1.2, 2.3); break;
            case "near-horizontal-contact": bounds = (.8, 2 + 5e-9, 1.2, 2.3); break;
            case "separated-horizontal": bounds = (.8, 2 + 2e-8, 1.2, 2.3); break;
            case "diagonal-crossing":
            case "diagonal-aabb-refusal":
                // No segment endpoint lies in either query. The first box is
                // crossed by the thin stripe; the second is actually disjoint
                // but overlaps a boundary AABB and must conservatively refuse.
                target = [[new(-2, -2), new(2, 2), new(2, 2.1), new(-2, -1.9)]];
                bounds = scenario == "diagonal-crossing" ? (-.2, .09, .2, .19) : (1.1, -1.2, 1.2, -1.1);
                break;
            case "between-components":
            case "encloses-components":
                target = [Rectangle(-2, -2, -1, -1), Rectangle(1, 1, 2, 2)];
                bounds = scenario == "between-components" ? (-.5, -.5, .5, .5) : (-3, -3, 3, 3);
                break;
        }
        var (inside, excludes) = InspectTargetBox(target, bounds);
        output.WriteLine($"scenario={scenario} originalWindingInside={inside} boundaryExcluded={excludes} bounds={bounds}");
        Assert.Equal(expectedInside, inside);
        Assert.Equal(expectedExclusion, excludes);
    }

    [Theory]
    [InlineData("tiny-ray-delta")]
    [InlineData("coordinate-domain")]
    public void UnsupportedTargetArithmeticRetainsReferenceAndRefusesBoxExclusion(string scenario)
    {
        IReadOnlyList<IReadOnlyList<PointD>> target;
        IReadOnlyList<TaggedToolpath> paths;
        if (scenario == "tiny-ray-delta")
        {
            target = [[new(0, 0), new(2, 1e-120), new(2, 2), new(0, 2)], Rectangle(5, 5, 5.02, 5.02)];
            paths = [Path(new(1, 1, -.1), new(1, 1, -.1))];
        }
        else
        {
            target = [Rectangle(10_001, 0, 10_001.2, .2), Rectangle(10_006, 5, 10_006.02, 5.02)];
            paths = [Path(new(10_001.1, .1, -.1), new(10_001.1, .1, -.1))];
        }
        var cpu = Certify(target, paths, Cylinder(.5), 0, Limits() with { UseGpuDiscWitnesses = false });
        var gpu = Certify(target, paths, Cylinder(.5), 0, Limits() with { UseGpuDiscWitnesses = true });
        Assert.False(cpu.Certified);
        Assert.False(gpu.Certified);
        Assert.Equal(0, gpu.TargetBoxExclusions);
        Assert.Equal(0, gpu.TargetExteriorCapExclusions);
        Assert.Empty(gpu.WitnessPathIndices);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ExteriorGapCapsPreserveThinDiagonalCoverageAndRealMissingEnd(bool cutCompleteChannel)
    {
        IReadOnlyList<IReadOnlyList<PointD>> target =
            [[new(-1, -1), new(1, 1), new(1, 1.02), new(-1, -.98)]];
        IReadOnlyList<TaggedToolpath> paths =
            [Path(new(-1, -1, -.1), new(cutCompleteChannel ? 1 : 0, cutCompleteChannel ? 1 : 0, -.1))];
        var limits = Limits() with { GpuBatchSize = 16 };
        var cpu = Certify(target, paths, Cylinder(.02), 0, limits with { UseGpuDiscWitnesses = false });
        var gpu = Certify(target, paths, Cylinder(.02), 0, limits with { UseGpuDiscWitnesses = true });
        Assert.Equal(cutCompleteChannel, cpu.Certified);
        Assert.Equal(cpu.Certified, gpu.Certified);
        if (!cutCompleteChannel) Assert.Empty(gpu.WitnessPathIndices);
    }

    [Theory]
    [InlineData(-1e-12, true)]
    [InlineData(1e-12, false)]
    public void ExteriorGapCapRetainsTheOriginalReserveAtBoundaryCenter(double margin, bool expectedCertified)
    {
        double span = (.004 - VCarveCoverageCertificate.ArithmeticReserve + margin) * Math.Sqrt(2);
        IReadOnlyList<IReadOnlyList<PointD>> target = [[new(0, 0), new(0, span), new(span, 0)]];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(10, 10, -.1), new(10, 10, -.1))];
        var limits = Limits() with { MaximumBoxes = 1 };
        var cpu = Certify(target, paths, Cylinder(.0001), 0, limits with { UseGpuDiscWitnesses = false });
        var gpu = Certify(target, paths, Cylinder(.0001), 0, limits with { UseGpuDiscWitnesses = true });
        Assert.Equal(expectedCertified, cpu.Certified);
        Assert.Equal(cpu.Certified, gpu.Certified);
        Assert.Equal(1, gpu.VisitedBoxes);
        if (gpu.GpuAvailable && expectedCertified) Assert.Equal(1, gpu.TargetExteriorCapExclusions);
        if (!expectedCertified) Assert.Empty(gpu.WitnessPathIndices);
    }

    private static (bool inside, bool excludes) InspectTargetBox(IReadOnlyList<IReadOnlyList<PointD>> target,
        (double x0, double y0, double x1, double y1) bounds)
    {
        // Exercise the actual private BVH through geometry cases, without
        // duplicating its AABB traversal or exposing a production test hook.
        const BindingFlags members = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
        Type certificate = typeof(VCarveCoverageCertificate);
        Type segmentType = certificate.GetNestedType("Segment", BindingFlags.NonPublic)!;
        Type budgetType = certificate.GetNestedType("Budget", BindingFlags.NonPublic)!;
        Type treeType = certificate.GetNestedType("Tree", BindingFlags.NonPublic)!;
        Type boxType = certificate.GetNestedType("Box", BindingFlags.NonPublic)!;
        Array segments = Array.CreateInstance(segmentType, target.Sum(ring => ring.Count));
        int ordinal = 0;
        foreach (var ring in target)
        for (int index = 0; index < ring.Count; index++)
        {
            PointD a = ring[index], b = ring[(index + 1) % ring.Count];
            object segment = Activator.CreateInstance(segmentType, members, null,
                [new Point3D(a.x, a.y, 0), new Point3D(b.x, b.y, 0), 0d, -1], null)!;
            segments.SetValue(segment, ordinal++);
        }
        object budget = Activator.CreateInstance(budgetType, members, null,
            [new VCarveCoverageCertificate.Limits(), CancellationToken.None], null)!;
        object tree = Activator.CreateInstance(treeType, members, null, [segments, budget], null)!;
        object box = Activator.CreateInstance(boxType, members, null,
            [bounds.x0, bounds.y0, bounds.x1, bounds.y1], null)!;
        var center = new PointD(bounds.x0 + (bounds.x1 - bounds.x0) / 2, bounds.y0 + (bounds.y1 - bounds.y0) / 2);
        bool inside = (bool)treeType.GetMethod("IsInside", members)!.Invoke(tree, [center])!;
        bool excludes = (bool)treeType.GetMethod("ExcludesBoundary", members)!.Invoke(tree, [box])!;
        return (inside, excludes);
    }

    private VCarveCoverageCertificate.Result Certify(IReadOnlyList<IReadOnlyList<PointD>> target,
        IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile, double surface,
        VCarveCoverageCertificate.Limits limits)
    {
        var result = VCarveCoverageCertificate.Certify(target, paths, profile, surface,
            limits: limits, diagnostic: output.WriteLine);
        output.WriteLine($"result certified={result.Certified} reason={result.Reason} gpuAttempted={result.GpuAttempted} "
            + $"gpuAvailable={result.GpuAvailable} gpuReason={result.GpuReason} gpuQueries={result.GpuQueries} "
            + $"gpuWitnesses={result.GpuWitnesses} gpuFallbackQueries={result.GpuFallbackQueries} "
            + $"gpuSubdivisionMisses={result.GpuSubdivisionMisses} gpuBatches={result.GpuBatches}");
        return result;
    }

    private static VCarveCoverageCertificate.Limits Limits() => new()
    {
        MinimumGpuIntervals = 1, GpuBatchSize = 64, RecordWitnessPathIndices = true,
        MaximumBoxes = 500_000, MaximumDiscEvaluations = 20_000_000, MaximumElapsed = TimeSpan.FromSeconds(30)
    };
    private static VCarveCutterProfile Cylinder(double radius)
        => new([new(CarvingCutterSectionKind.Linear, radius, radius, 1)], 1);
    private static VCarveCutterProfile BallTaper()
        => new([new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 21.5875);
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
}
