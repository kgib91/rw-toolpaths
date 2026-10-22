using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class VCarveTriangleCoverageCertificateTests(ITestOutputHelper output)
{
    [Theory]
    [InlineData("triangle", true)]
    [InlineData("acute-triangle", true)]
    [InlineData("profile-transition", true)]
    [InlineData("retained-hole", true)]
    [InlineData("nested-island", true)]
    [InlineData("filled-hole", false)]
    [InlineData("covered-components", true)]
    [InlineData("missed-component", false)]
    [InlineData("vertex-islands-one-owner", false)]
    public void ExactSignedTrianglesPreserveWholeTargetCoverageAndRealMissingStock(string scenario, bool expected)
    {
        IReadOnlyList<IReadOnlyList<PointD>> target = [[new(-.25, -.25), new(.25, -.25), new(0, .25)]];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, -.125), new(0, 0, -.125))];
        var profile = Cylinder(.5);
        switch (scenario)
        {
            case "acute-triangle":
                target = [[new(-.75, -.0625), new(.75, -.0625), new(.5, .0625)]];
                profile = Cylinder(.125);
                paths = [Path(new(-1, 0, -.125), new(1, 0, -.125))];
                break;
            case "profile-transition":
                target = [Rectangle(-.125, -.125, .125, .125)];
                profile = new([new(CarvingCutterSectionKind.Linear, 0, .25, .5),
                    new(CarvingCutterSectionKind.Linear, .25, 1.25, .5)], 1);
                paths = [Path(new(-1, 0, -.125), new(1, 0, -1))];
                break;
            case "retained-hole":
            case "nested-island":
            case "filled-hole":
                var outer = Rectangle(-.375, -.375, .375, .375);
                var hole = Rectangle(-.125, -.125, .125, .125).Reverse().ToArray();
                target = scenario == "filled-hole" ? [outer] : [outer, hole];
                profile = Cylinder(.1875);
                var perimeter = Path(new(-.25, -.25, -.125), new(.25, -.25, -.125),
                    new(.25, .25, -.125), new(-.25, .25, -.125), new(-.25, -.25, -.125));
                paths = [perimeter];
                if (scenario == "nested-island")
                {
                    target = [outer, hole, Rectangle(-.03125, -.03125, .03125, .03125)];
                    paths = [perimeter, Path(new(0, 0, -.125), new(0, 0, -.125))];
                }
                break;
            case "covered-components":
            case "missed-component":
                target = Enumerable.Range(0, 8).Select(index =>
                    (IReadOnlyList<PointD>)Rectangle(3 * index - .25, -.25, 3 * index + .25, .25)).ToArray();
                paths = Enumerable.Range(0, scenario == "covered-components" ? 8 : 7)
                    .Select(index => Path(new(3 * index, 0, -.125), new(3 * index, 0, -.125))).ToArray();
                break;
            case "vertex-islands-one-owner":
                // Three cutting intervals have the same path owner, but their
                // separate discs leave the triangle's interior uncut. Owner
                // equality cannot combine them into one convex sweep witness.
                target = [[new(0, 0), new(1, 0), new(0, 1)]];
                profile = Cylinder(.0625);
                paths = [new TaggedToolpath([new(0, 0, -.125), new(0, 0, -.125),
                    new(1, 0, -.125), new(1, 0, -.125), new(0, 1, -.125), new(0, 1, -.125)],
                    [new(0, 1, ToolpathSpanKind.Cut), new(1, 2, ToolpathSpanKind.Retract),
                     new(2, 3, ToolpathSpanKind.Cut), new(3, 4, ToolpathSpanKind.Retract),
                     new(4, 5, ToolpathSpanKind.Cut)], 0, "final-carve", null, detectCurves: false)];
                break;
        }
        var reference = Certify(target, paths, profile, Limits() with { UseGpuDiscWitnesses = false, UseGpuTriangleDomain = false });
        var triangles = Certify(target, paths, profile, Limits());
        Assert.Equal(expected, reference.Certified);
        Assert.Equal(reference.Certified, triangles.Certified);
        Assert.Equal(reference.BoundarySegments, triangles.BoundarySegments);
        Assert.Equal(reference.CommandIntervals, triangles.CommandIntervals);
        Assert.Equal(paths.Count, triangles.CommandPathCount);
        if (triangles.GpuAvailable) AssertTriangleDomain(triangles);
        if (expected)
        {
            Assert.True(triangles.WitnessPathsRecorded);
            Assert.NotEmpty(triangles.WitnessPathIndices);
            Assert.All(triangles.WitnessPathIndices, owner => Assert.InRange(owner, 0, paths.Count - 1));
        }
        else
        {
            Assert.False(triangles.WitnessPathsRecorded);
            Assert.Empty(triangles.WitnessPathIndices);
        }
    }

    [Theory]
    [InlineData("non-grid", "target-not-exact-grid")]
    [InlineData("large-coordinate", "target-coordinate-domain")]
    [InlineData("steep-profile", "unsupported-convex-profile")]
    public void RefusedTrianglePreparationPreservesTheOriginalBoxCertificate(string scenario, string expectedReason)
    {
        double x = scenario == "non-grid" ? .123456789123 : scenario == "large-coordinate" ? 10_001 : 0;
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(x - .125, -.125, x + .125, .125)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(x, 0, -.125), new(x, 0, -.125))];
        var profile = Cylinder(1);
        if (scenario == "steep-profile")
        {
            // This finite original profile exceeds the ordinary slope bound;
            // it must retain literal CPU coverage rather than triangle authority.
            profile = new([new(CarvingCutterSectionKind.Linear, 0, .5, .001)], .001);
            paths = [Path(new(0, 0, -.001), new(0, 0, -.001))];
        }
        var reference = Certify(target, paths, profile, Limits() with { UseGpuDiscWitnesses = false, UseGpuTriangleDomain = false });
        var refused = Certify(target, paths, profile, Limits());
        Assert.True(reference.Certified, reference.ToString());
        Assert.Equal(reference.Certified, refused.Certified);
        Assert.False(refused.TriangleDomainUsed);
        Assert.Equal("boxes", refused.CoverageDomain);
        if (refused.GpuAvailable)
        {
            Assert.True(refused.TriangleDomainAttempted);
            Assert.Equal(expectedReason, refused.TriangleDomainReason);
        }
    }

    [Theory]
    [InlineData("box-budget")]
    [InlineData("disc-budget")]
    [InlineData("input-budget")]
    [InlineData("time-budget")]
    public void TriangleTasksCannotAuthorizeCoverageAfterQuotaExhaustion(string reason)
    {
        var limits = reason switch
        {
            "box-budget" => Limits() with { MaximumBoxes = 0 },
            "disc-budget" => Limits() with { MaximumDiscEvaluations = 0 },
            "input-budget" => Limits() with { MaximumInputSegments = 3 },
            "time-budget" => Limits() with { MaximumElapsed = TimeSpan.FromTicks(1) },
            _ => throw new ArgumentOutOfRangeException(nameof(reason))
        };
        var result = Certify([Rectangle(-.25, -.25, .25, .25)],
            [Path(new(0, 0, -.125), new(0, 0, -.125))], Cylinder(1), limits);
        Assert.False(result.Certified);
        Assert.Equal(reason, result.Reason);
        Assert.False(result.WitnessPathsRecorded);
        Assert.Empty(result.WitnessPathIndices);
    }

    [Fact]
    public void CancellationAtRequestAndAfterFirstDiagnosticPropagates()
    {
        using var before = new CancellationTokenSource(); before.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveCoverageCertificate.Certify(
            [Rectangle(-.25, -.25, .25, .25)], [Path(new(0, 0, -.125), new(0, 0, -.125))], Cylinder(1),
            cancellationToken: before.Token, limits: Limits(), diagnostic: output.WriteLine));
        using var during = new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(() => VCarveCoverageCertificate.Certify(
            [Rectangle(-.25, -.25, .25, .25)], [Path(new(0, 0, -.125), new(0, 0, -.125))], Cylinder(1),
            cancellationToken: during.Token, limits: Limits(), diagnostic: line =>
            { output.WriteLine(line); during.Cancel(); }));
    }

    [Fact]
    public void MalformedInputsRetainTheirOriginalRejection()
    {
        Assert.Throws<ArgumentException>(() => Certify([[new(0, 0), new(1, 1)]], [], Cylinder(1), Limits()));
        Assert.Throws<ArgumentException>(() => Certify([Rectangle(-.25, -.25, .25, .25)],
            [Path(new(double.NaN, 0, -.125), new(0, 0, -.125))], Cylinder(1), Limits()));
        Assert.Throws<ArgumentOutOfRangeException>(() => Certify([Rectangle(-.25, -.25, .25, .25)],
            [Path(new(0, 0, -.125), new(0, 0, -.125))], Cylinder(1),
            Limits() with { OptionalWitnessPathIndices = new HashSet<int> { 1 } }));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(7)]
    [InlineData(64)]
    public void TriangleBatchesPreserveEveryComponentOwner(int batchSize)
    {
        var target = Enumerable.Range(0, 8).Select(index =>
            (IReadOnlyList<PointD>)Rectangle(3 * index - .25, -.25, 3 * index + .25, .25)).ToArray();
        var paths = Enumerable.Range(0, 8).Select(index => Path(new(3 * index, 0, -.125), new(3 * index, 0, -.125))).ToArray();
        var result = Certify(target, paths, Cylinder(.5), Limits() with { GpuBatchSize = batchSize });
        Assert.True(result.Certified, result.ToString());
        Assert.Equal(Enumerable.Range(0, 8).ToArray(), result.WitnessPathIndices);
        if (result.GpuAvailable)
        {
            AssertTriangleDomain(result);
            Assert.True(result.GpuTriangleQueries > 0);
            Assert.True(result.GpuTriangleWitnesses > 0);
        }
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MandatoryFirstPreferenceAndOptionalFallbackPreserveOriginalPathOrdinals(bool mandatoryCovers)
    {
        var target = new IReadOnlyList<PointD>[] { Rectangle(-.25, -.25, .25, .25) };
        var paths = new[] { Path(new(0, 0, -.125), new(0, 0, -.125)),
            Path(new(mandatoryCovers ? 0 : 5, 0, -.125), new(mandatoryCovers ? 0 : 5, 0, -.125)) };
        var result = Certify(target, paths, Cylinder(1), Limits() with { OptionalWitnessPathIndices = new HashSet<int> { 0 } });
        Assert.True(result.Certified, result.ToString());
        Assert.Equal(new[] { mandatoryCovers ? 1 : 0 }, result.WitnessPathIndices);
        if (result.GpuAvailable) AssertTriangleDomain(result);
    }

    [Fact]
    public void MandatoryOriginalDiscAtProfileJoinWinsBeforeOptionalConvexTriangle()
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
            new(CarvingCutterSectionKind.Linear, 1, 1, 1)], 2);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(-.125, -.125, .125, .125)];
        // Owner 1 is exactly at the profile join: a literal radius-one disc
        // covers both target triangles, while the strict convex-span guard
        // refuses the join. Optional owner 0 is deeper in an eligible section.
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, -1.5), new(0, 0, -1.5)),
            Path(new(0, 0, -1), new(0, 0, -1))];
        var result = Certify(target, paths, profile, Limits() with { OptionalWitnessPathIndices = new HashSet<int> { 0 } });
        Assert.True(result.Certified, result.ToString());
        Assert.Equal(new[] { 1 }, result.WitnessPathIndices);
        if (result.GpuAvailable)
        {
            AssertTriangleDomain(result);
            Assert.True(result.GpuQueries > 0);
            Assert.Equal(0, result.GpuOptionalTriangleWitnesses);
        }
    }

    [Fact]
    public void SignedZeroRefusesBitExactTriangleAdmissionAndRetainsOriginalCoverage()
    {
        double negativeZero = BitConverter.Int64BitsToDouble(long.MinValue);
        IReadOnlyList<IReadOnlyList<PointD>> target =
            [[new(negativeZero, -.125), new(.25, -.125), new(.25, .125), new(negativeZero, .125)]];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(.125, 0, -.125), new(.125, 0, -.125))];
        var reference = Certify(target, paths, Cylinder(1), Limits() with { UseGpuDiscWitnesses = false, UseGpuTriangleDomain = false });
        var result = Certify(target, paths, Cylinder(1), Limits());
        Assert.True(reference.Certified);
        Assert.Equal(reference.Certified, result.Certified);
        Assert.False(result.TriangleDomainUsed);
        Assert.Equal("boxes", result.CoverageDomain);
        if (result.GpuAvailable) Assert.Equal("target-not-exact-grid", result.TriangleDomainReason);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DensifiedThinTargetNeedsOnlyOriginalBoundaryFringeWitnesses(bool useDelaunay)
    {
        const int segments = 16;
        const double width = 64 / PathUtils.Scale;
        var ring = new List<PointD>(2 * segments + 2);
        for (int i = 0; i <= segments; i++) ring.Add(new(i / (double)segments, 0));
        for (int i = segments; i >= 0; i--) ring.Add(new(i / (double)segments, width));
        IReadOnlyList<IReadOnlyList<PointD>> target = [ring];
        // The actual cutting interval creates a device context but is far from
        // every target point. Coverage can only come from the original boundary
        // fringe. Collinear removal in triangulation does not enlarge an
        // original boundary segment into a long-edge acceptance witness.
        IReadOnlyList<TaggedToolpath> paths = [Path(new(5, 5, -.125), new(5.25, 5, -.125))];
        var limits = Limits() with { UseDelaunayTriangleDomain = useDelaunay };
        var reference = Certify(target, paths, Cylinder(.125),
            limits with { UseGpuDiscWitnesses = false, UseGpuTriangleDomain = false });
        var result = Certify(target, paths, Cylinder(.125), limits);
        Assert.True(reference.Certified, reference.ToString());
        Assert.True(result.Certified, result.ToString());
        Assert.Empty(reference.WitnessPathIndices);
        Assert.Empty(result.WitnessPathIndices);
        Assert.Equal(2 * segments + 2, result.BoundarySegments);
        Assert.Equal(1, result.CommandIntervals);
        if (result.GpuAvailable)
        {
            AssertTriangleDomain(result);
            Assert.True(result.TriangleSubdivisions > 0);
            Assert.True(result.TriangleBoundaryWitnesses > 0);
            Assert.True(result.TriangleBoundaryNominationCandidates > 0);
            Assert.Equal(0, result.GpuTriangleWitnesses);
            Assert.Equal(0, result.GpuWitnesses);
        }
    }

    [Fact]
    public void ReusedPreparedDomainCannotReusePhysicalCoverageFromEarlierCommands()
    {
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(-.125, -.125, .125, .125)];
        var limits = Limits() with { TriangleDomainPreparation = new VCarveCoverageCertificate.PreparedTriangleDomain() };
        var covered = Certify(target, [Path(new(0, 0, -.125), new(0, 0, -.125))], Cylinder(.5), limits);
        var missed = Certify(target, [Path(new(5, 5, -.125), new(5, 5, -.125))], Cylinder(.5), limits);
        Assert.True(covered.Certified, covered.ToString());
        Assert.Equal(new[] { 0 }, covered.WitnessPathIndices);
        Assert.False(covered.TrianglePreparationReused);
        Assert.False(missed.Certified, missed.ToString());
        Assert.False(missed.WitnessPathsRecorded);
        Assert.Empty(missed.WitnessPathIndices);
        if (covered.GpuAvailable)
        {
            AssertTriangleDomain(covered);
            AssertTriangleDomain(missed);
            Assert.True(missed.TrianglePreparationReused);
            Assert.True(missed.GpuTriangleQueries > 0);
            Assert.Equal(0, missed.GpuTriangleWitnesses);
        }
    }

    [Theory]
    [InlineData(false, true)]
    [InlineData(true, true)]
    [InlineData(false, false)]
    [InlineData(true, false)]
    public void DeviceBoundaryNominationsRequireExactFiniteCapsulesAndCannotDiscardUncutInterior(bool delaunay, bool thin)
    {
        const int segments = 16;
        double width = thin ? 64 / PathUtils.Scale : .125;
        var ring = new List<PointD>();
        for (int i = 0; i <= segments; i++) ring.Add(new(i / (double)segments, 0));
        for (int i = segments; i >= 0; i--) ring.Add(new(i / (double)segments, width));
        IReadOnlyList<IReadOnlyList<PointD>> target = [ring];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(5, 5, -.125), new(5.25, 5, -.125))];
        var limits = Limits() with { UseDelaunayTriangleDomain = delaunay };
        var reference = Certify(target, paths, Cylinder(.125), limits);
        var result = Certify(target, paths, Cylinder(.125), limits with { UseGpuBoundaryNominations = true });
        Assert.Equal(thin, reference.Certified);
        Assert.Equal(reference.Certified, result.Certified);
        Assert.Empty(result.WitnessPathIndices);
        Assert.Equal(reference.BoundarySegments, result.BoundarySegments);
        Assert.Equal(0, result.GpuTriangleWitnesses);
        if (result.GpuAvailable)
        {
            AssertTriangleDomain(result);
            Assert.True(result.TriangleBoundaryGpuAvailable, result.TriangleBoundaryGpuReason);
            Assert.True(result.TriangleBoundaryGpuQueries > 0);
            Assert.True(result.TriangleBoundaryGpuMisses > 0);
            if (thin) Assert.True(result.TriangleBoundaryGpuWitnesses > 0);
            else
            {
                Assert.False(result.WitnessPathsRecorded);
                Assert.True(result.RemainingBoxes > 0);
            }
            output.WriteLine($"[BoundaryTriangleControl] actualCudaExecuted=True thin={thin} "
                + $"queries={result.TriangleBoundaryGpuQueries} witnesses={result.TriangleBoundaryGpuWitnesses} "
                + $"misses={result.TriangleBoundaryGpuMisses} positiveAuthority=original-exact-finite-capsule");
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundarySamplerCancellationPropagatesAndPublishesNoCoverageResult(bool throwFromObserver)
    {
        using var cancellation = new CancellationTokenSource();
        bool ready = false, disposed = false;
        Action<string> diagnostic = message =>
        {
            output.WriteLine(message);
            if (message.Contains("stage=boundary-gpu-sample event=disposed", StringComparison.Ordinal)) disposed = true;
            if (message.Contains("boundary-device-selection available=True", StringComparison.Ordinal))
            {
                ready = true; cancellation.Cancel();
                if (throwFromObserver) throw new OperationCanceledException(cancellation.Token);
            }
        };
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(0, 0, 1, .001953125)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(5, 5, -.125), new(5.25, 5, -.125))];
        VCarveCoverageCertificate.Result? result = null;
        try
        {
            result = VCarveCoverageCertificate.Certify(target, paths, Cylinder(.125),
                cancellationToken: cancellation.Token, diagnostic: diagnostic,
                limits: Limits() with { UseGpuBoundaryNominations = true });
        }
        catch (OperationCanceledException exception)
        {
            Assert.True(ready);
            Assert.Equal(cancellation.Token, exception.CancellationToken);
            Assert.Null(result);
            Assert.True(disposed);
            output.WriteLine("[BoundaryTriangleControl] actualCudaReady=True cancellationPropagated=True noCoveragePublished=True");
            return;
        }
        Assert.False(ready);
        Assert.NotNull(result);
        output.WriteLine("[BoundaryTriangleControl] portableCudaRefusal=True cancellationDeviceControlUnexecuted=True");
    }

    [Fact]
    public void DisabledDeviceThresholdUsesTheOriginalBoxReference()
    {
        var result = Certify([Rectangle(-.25, -.25, .25, .25)],
            [Path(new(0, 0, -.125), new(0, 0, -.125))], Cylinder(1), Limits() with { MinimumGpuIntervals = 2 });
        Assert.True(result.Certified);
        Assert.False(result.GpuAttempted);
        Assert.False(result.TriangleDomainUsed);
        Assert.Equal("boxes", result.CoverageDomain);
        Assert.Equal("below-interval-threshold", result.GpuReason);
        Assert.Equal(0, result.GpuTriangleQueries);
    }

    private VCarveCoverageCertificate.Result Certify(IReadOnlyList<IReadOnlyList<PointD>> target,
        IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile, VCarveCoverageCertificate.Limits limits)
    {
        var result = VCarveCoverageCertificate.Certify(target, paths, profile, limits: limits, diagnostic: output.WriteLine);
        var triangleCounters = typeof(VCarveCoverageCertificate.Result)
            .GetProperties(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
            .Where(property => property.Name.Contains("Triangle", StringComparison.Ordinal))
            .Select(property => $"{property.Name}={property.GetValue(result)}");
        output.WriteLine($"certified={result.Certified} reason={result.Reason} domain={result.CoverageDomain} "
            + $"gpuAvailable={result.GpuAvailable} owners={string.Join(',', result.WitnessPathIndices)} "
            + string.Join(' ', triangleCounters));
        return result;
    }

    private static void AssertTriangleDomain(VCarveCoverageCertificate.Result result)
    {
        Assert.True(result.TriangleDomainAttempted);
        Assert.True(result.TriangleDomainUsed, result.TriangleDomainReason);
        Assert.Equal("exact-signed-triangles", result.CoverageDomain);
        Assert.True(result.TriangleSourceTriangles > 0);
        Assert.True(result.VisitedTriangles > 0);
    }

    private static VCarveCoverageCertificate.Limits Limits() => new()
    {
        UseGpuDiscWitnesses = true, UseGpuTriangleDomain = true, MinimumGpuIntervals = 1,
        GpuBatchSize = 16, RecordWitnessPathIndices = true, MaximumBoxes = 500_000,
        MaximumDiscEvaluations = 20_000_000, MaximumElapsed = TimeSpan.FromSeconds(30)
    };
    private static VCarveCutterProfile Cylinder(double radius)
        => new([new(CarvingCutterSectionKind.Linear, radius, radius, 1)], 1);
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
}
