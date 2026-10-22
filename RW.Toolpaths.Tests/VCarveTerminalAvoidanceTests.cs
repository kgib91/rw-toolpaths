using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveTerminalAvoidanceTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void MandatoryEndpointEnclosesEverySampledStockHeight(bool rounded, bool reverse)
    {
        var profile = Profile(rounded);
        var authority = Path(new(0, 0, -.005), new(2, 0, -.005), new(0, 0, -.005));
        if (reverse) authority = Path(authority.Points.Reverse().ToArray());
        var index = new VCarveSmoothing.MandatoryTerminalCoverage(profile, 12.7);
        index.Add([authority]);
        Point3D start = new(2, .003, 0), end = new(2, 0, -.001);
        Assert.True(index.Covers(start, end, 7, 3));

        // Independently query the physical bottom at constant-depth mandatory
        // centers. Probe the whole candidate cross-section at multiple stock
        // heights, not only its projected surface footprint or tip endpoints.
        var physical = new CarvingCutterProfile(profile.Sections, profile.MaximumDepth);
        int probes = 0;
        for (int pose = 1; pose <= 32; pose++)
        {
            double t = pose / 32.0, x = start.X + (end.X - start.X) * t,
                y = start.Y + (end.Y - start.Y) * t, depth = .001 * t;
            for (int height = 0; height <= 16; height++)
            {
                double z = -depth + depth * height / 16;
                double radius = profile.PhysicalRadiusAtDepth(depth + z);
                for (int angle = 0; angle < 24; angle++)
                {
                    double theta = angle * 2 * Math.PI / 24;
                    double px = x + radius * Math.Cos(theta), py = y + radius * Math.Sin(theta);
                    double nearestX = Math.Clamp(px, 0, 2);
                    double distance = Math.Sqrt((px - nearestX) * (px - nearestX) + py * py);
                    Assert.True(-.005 + physical.Bottom(distance) <= z + 1e-12,
                        $"Stock-depth witness lost at ({px:R},{py:R},{z:R}).");
                    probes++;
                }
            }
        }
        Assert.Equal(13_056, probes);
    }

    [Fact]
    public void RawYOnlyArrivalCannotAuthorizeAnUnvisitedF3Endpoint()
    {
        var profile = Profile(true);
        Point3D sourceStart = new(10000.00049, 0, -.0001), sourceEnd = new(10000.00244, 0, -.0012);
        float dx = (float)sourceEnd.X - (float)sourceStart.X, dz = (float)sourceEnd.Z - (float)sourceStart.Z;
        Assert.True(MathF.Abs(dz) > .001f && MathF.Sqrt(dx * dx) <= .001f);
        static double F3(double value) => double.Parse(((float)value).ToString("F3",
            System.Globalization.CultureInfo.InvariantCulture), System.Globalization.CultureInfo.InvariantCulture);
        Point3D desired = new(F3(sourceEnd.X), 0, F3(sourceEnd.Z));
        Point3D actual = new(F3(sourceStart.X), 0, desired.Z); // Y-only leaves the previously posted XY.
        Assert.NotEqual(actual.X, desired.X);
        var physical = new CarvingCutterProfile(profile.Sections, profile.MaximumDepth);
        Assert.True(actual.Z + physical.Bottom(Math.Abs(desired.X - actual.X)) > desired.Z + 1e-9,
            "A native endpoint the machine never visits must not prove its required stock depth.");

        var index = new VCarveSmoothing.MandatoryTerminalCoverage(profile, 0);
        index.Add([Path(sourceStart, sourceEnd)]);
        Assert.False(index.Covers(new(desired.X + .01, 0, 0), desired, 7, 3));
        Assert.Contains("endpointsWithoutXyzArrival=1", index.Describe());
        var constantDepth = new VCarveSmoothing.MandatoryTerminalCoverage(profile, 0);
        constantDepth.Add([Path(sourceStart with { Z = -.001 }, sourceEnd with { Z = -.001 })]);
        Assert.True(constantDepth.Covers(new(desired.X + .01, 0, 0), desired, 7, 3));
    }

    [Fact]
    public void FirstOpenEndpointNeedsItsOwnProvenXyzArrival()
    {
        var profile = Profile(true);
        Point3D first = new(0, 0, -.005), second = new(1, 0, -.001);
        var open = new VCarveSmoothing.MandatoryTerminalCoverage(profile, 0);
        open.Add([Path(first, second)]);
        Assert.False(open.Covers(new(.01, 0, 0), new(0, 0, -.001), 7, 3));
        var closed = new VCarveSmoothing.MandatoryTerminalCoverage(profile, 0);
        closed.Add([Path(first, second, first)]);
        Assert.True(closed.Covers(new(.01, 0, 0), new(0, 0, -.001), 7, 3));
    }

    [Theory]
    [InlineData(0, true)]
    [InlineData(10, false)]
    public void EndpointAuthorityUsesRawBinary32WorkspaceClassifierBeforeF3(double surface, bool xyzArrival)
    {
        Point3D sourceStart = new(0, 0, 0), sourceEnd = new(.001, 0, -.001);
        float run = (float)sourceEnd.X - (float)sourceStart.X;
        float dz = (float)(surface + sourceEnd.Z) - (float)(surface + sourceStart.Z);
        Assert.Equal(xyzArrival, !(MathF.Abs(dz) > .001f && MathF.Abs(run) <= .001f));
        var index = new VCarveSmoothing.MandatoryTerminalCoverage(Profile(true), surface);
        index.Add([Path(sourceStart, sourceEnd)]);
        Assert.Equal(xyzArrival, index.Covers(new(.011, 0, 0), new(.001, 0, -.001), 7, 3));
    }

    [Fact]
    public void NonfiniteRawCommandArrivalNeverCreatesEndpointAuthority()
    {
        var index = new VCarveSmoothing.MandatoryTerminalCoverage(Profile(true), 0);
        // Both nominal endpoints are finite, but their binary32 casts overflow.
        index.Add([Path(new(1e40, 0, -.001), new(2e40, 0, -.002))]);
        Assert.False(index.Covers(new(.01, 0, 0), new(0, 0, -.001), 7, 3));
        Assert.Contains("indexedEndpoints=0", index.Describe());
        Assert.Contains("endpointsWithoutXyzArrival=1", index.Describe());
    }

    [Fact]
    public void SurfaceCertificateCannotHideAnUncutRequiredDepth()
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 1, 1, 1)], 1);
        var mandatory = Path(new(0, 0, -.001), new(2, 0, -.001));
        var index = new VCarveSmoothing.MandatoryTerminalCoverage(profile, 0);
        index.Add([mandatory]);
        Point3D start = new(.5, 0, 0), end = new(2, 0, -.002);
        Assert.False(index.Covers(start, end, 7, 3));
        IReadOnlyList<PointD>[] target = [new PointD[] { new(1.8, -.1), new(2.1, -.1), new(2.1, .1), new(1.8, .1) }];
        Assert.True(VCarveCoverageCertificate.Certify(target, [mandatory], profile).Certified);
        var physical = new CarvingCutterProfile(profile.Sections, 1);
        Assert.True(mandatory.Points[^1].Z + physical.Bottom(0) > -.0015);
        Assert.True(end.Z + physical.Bottom(0) < -.0015);
    }

    [Fact]
    public void MandatoryEndpointEnclosesShortNoncollinearRayAtEveryStockHeight()
    {
        var profile = Profile(true);
        var index = new VCarveSmoothing.MandatoryTerminalCoverage(profile, 12.7);
        index.Add([Path(new(.99, 0, -.001), new(1, 0, -.001))]);
        Point3D start = new(1, .02, 0), end = new(1, 0, -.001);
        Assert.True(index.Covers(start, end, 7, 3));
        Assert.Contains("endpointVolumeDominated=1", index.Describe());
        var physical = new CarvingCutterProfile(profile.Sections, profile.MaximumDepth);
        for (int pose = 1; pose <= 64; pose++)
        for (int height = 0; height <= 32; height++)
        for (int angle = 0; angle < 24; angle++)
        {
            double t = pose / 64.0, depth = .001 * t, z = -depth + depth * height / 32;
            double radius = profile.PhysicalRadiusAtDepth(depth + z), theta = angle * 2 * Math.PI / 24;
            double dx = radius * Math.Cos(theta), dy = .02 * (1 - t) + radius * Math.Sin(theta);
            Assert.True(-.001 + physical.Bottom(Math.Sqrt(dx * dx + dy * dy)) <= z + 1e-12);
        }
        Assert.False(index.Covers(new(1, .06, 0), end, 7, 3));
        // A mid-ramp surface disc of the longer ray reaches beyond this
        // endpoint's physical cut; a projected center check would miss it.
        double outside = .03 + profile.PhysicalRadiusAtDepth(.0005);
        Assert.True(-.001 + physical.Bottom(outside) > 0);
        Assert.False(index.Covers(start, new(1, 0, -.002), 7, 3));
        Assert.False(index.Covers(start, new(1.001, 0, -.001), 7, 3));
    }

    [Fact]
    public void CylinderEquatorAndCompoundZeroSlopeCannotAuthorizeEndpointVolumeSkip()
    {
        VCarveCutterProfile[] profiles = [
            new([new(CarvingCutterSectionKind.Linear, 1, 1, 2)], 2),
            new([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)], 1),
            new([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
                new(CarvingCutterSectionKind.Linear, 1, 1, 1)], 2),
            new([new(CarvingCutterSectionKind.Linear, .1, .2, .001),
                new(CarvingCutterSectionKind.Linear, .2, .2, .999)], 1)];
        double[] depths = [.001, 1, 1.001, .002];
        for (int i = 0; i < profiles.Length; i++)
        {
            var index = new VCarveSmoothing.MandatoryTerminalCoverage(profiles[i], 0);
            index.Add([Path(new(.99, 0, -depths[i]), new(1, 0, -depths[i]))]);
            Assert.False(index.Covers(new(1, .001, 0), new(1, 0, -.001), 7, 3));
        }
        var collapsed = new VCarveSmoothing.MandatoryTerminalCoverage(Profile(true), 0);
        collapsed.Add([Path(new(1, 0, -.001), new(1, 0, -.001))]);
        Assert.False(collapsed.Covers(new(1, .01, 0), new(1, 0, -.001), 7, 3));
    }

    [Fact]
    public void MissingGapBeyondMandatoryChordStillProducesTerminal()
    {
        var profile = Profile(true);
        var mandatory = Path(new(0, 0, -.005), new(1, 0, -.005));
        var index = new VCarveSmoothing.MandatoryTerminalCoverage(profile, 0);
        index.Add([mandatory]);
        Assert.False(index.Covers(new(-1, 0, 0), new(1, 0, -.001), 7, 3));
        var physical = new CarvingCutterProfile(profile.Sections, profile.MaximumDepth);
        Assert.True(-.005 + physical.Bottom(.5) > -.0001);
        // This actual ramp pose penetrates the stock at the missing XY point.
        Assert.True(-.00025 + physical.Bottom(0) < -.0001);
    }

    [Fact]
    public void OneF3StepOffLineAndDifferentPassOrRegionHaveNoRemovalAuthority()
    {
        var index = new VCarveSmoothing.MandatoryTerminalCoverage(Profile(true), 0);
        index.Add([Path(new(0, 0, -.005), new(1, 0, -.005))]);
        Assert.False(index.Covers(new(.5, .001, 0), new(1, 0, -.001), 7, 3));
        Assert.False(index.Covers(new(.5, 0, 0), new(1, 0, -.001), 7, 4));
        Assert.False(index.Covers(new(.5, 0, 0), new(1, 0, -.001), 8, 3));
        Assert.False(index.Covers(new(.5, 0, 0), new(1, 0, -.001), 7, null));
        Assert.False(index.Covers(new(.50001, 0, 0), new(1, 0, -.001), 7, 3));
    }

    [Fact]
    public void WallClippedMandatoryRayCannotAuthorizeItsOriginalLongRay()
    {
        var index = new VCarveSmoothing.MandatoryTerminalCoverage(Profile(true), 0);
        // Wall/floor protection kept only the final half of this ray engaged.
        index.Add([Path(new(-1, 0, 0), new(-.5, 0, 0), new(0, 0, -.001))]);
        Assert.False(index.Covers(new(-1, 0, 0), new(0, 0, -.001), 7, 3));
        Assert.True(index.Covers(new(-.01, 0, 0), new(0, 0, -.001), 7, 3));
    }

    [Fact]
    public void OnlyExplicitMandatoryCuttingPassesWithinMonotoneContactHeightEnterIndex()
    {
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Linear, .1, .2, .002),
            new(CarvingCutterSectionKind.Linear, .2, .1, .002)], .001);
        var index = new VCarveSmoothing.MandatoryTerminalCoverage(profile, 0);
        index.Add([Path(new(0, 0, -.002), new(1, 0, -.002))]);
        Assert.False(index.Covers(new(.5, 0, 0), new(1, 0, -.001), 7, 3));
        var finalCarve = new TaggedToolpath([new(0, 0, -.001), new(1, 0, -.001)],
            Array.Empty<ToolpathSpan>(), 7, "final-carve", 3, detectCurves: false);
        var retract = new TaggedToolpath(finalCarve.Points,
            [new ToolpathSpan(0, 1, ToolpathSpanKind.Retract)], 7, "clearing", 3, detectCurves: false);
        index.Add([finalCarve, retract]);
        Assert.False(index.Covers(new(.5, 0, 0), new(1, 0, -.001), 7, 3));
    }

    [Fact]
    public void BoundedIndexAndFiniteAmbiguityFallBackWithoutWeakeningProof()
    {
        var index = new VCarveSmoothing.MandatoryTerminalCoverage(Profile(true), 0, maximumExaminedChords: 1);
        index.Add([Path(new(0, 0, -.005), new(1, 0, -.005), new(2, 0, -.005))]);
        Assert.True(index.Covers(new(1, .01, 0), new(1, 0, -.001), 7, 3));
        Assert.False(index.Covers(new(2, .01, 0), new(2, 0, -.001), 7, 3));
        Assert.False(index.Covers(new(double.NaN, 0, 0), new(1, 0, -.001), 7, 3));
        Assert.False(index.Covers(new(double.PositiveInfinity, 0, 0), new(1, 0, -.001), 7, 3));
        Assert.False(index.Covers(new(1e12, 0, 0), new(1, 0, -.001), 7, 3));
        Assert.Contains("indexBounded=True", index.Describe());
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => index.Add([], cancellation.Token));
    }

    [Theory]
    [InlineData(true, .003)]
    [InlineData(false, .03)]
    public void UnprovedCommandCoordinatesOrTighterDepthBudgetRetainOriginalCandidates(bool finalCoordinates, double tolerance)
    {
        var profile = Profile(true);
        PointD[] outline = [new(0, 0), new(16, 0), new(16, 5), new(0, 1.5)];
        var source = MedialAxisToolpaths.GenerateVCarveTagged(BoostVoronoiProvider.CreateDefault(),
            [outline], 0, 3, 0, 1.5, includeInteriorFill: false, detectCurves: false, cutterProfile: profile);
        var diagnostics = new List<string>();
        var result = VCarveSmoothing.Apply(source, [outline], new ToolGeometry(3) { VCarveProfile = profile },
            new DepthSchedule(3, 1.5), new VCarveOptions { SmoothVCarving = true,
                SmoothRampAngleDegrees = 30, IncludeInteriorFill = false, DetectCurves = false,
                Tolerance = tolerance, AvoidRedundantTerminalRays = finalCoordinates },
            CancellationToken.None, diagnostics.Add, correctionWorkers: 1);
        Assert.NotEmpty(result);
        var avoidance = diagnostics.Where(line => line.Contains("stage=smooth-terminal-avoidance")).ToArray();
        Assert.NotEmpty(avoidance);
        Assert.All(avoidance, line =>
        {
            Assert.Contains("earlyAvoidanceEnabled=False", line);
            Assert.Contains("examinedChords=0 ", line);
            Assert.Contains("omittedCandidates=0 ", line);
        });
        Assert.Contains(diagnostics, line => line.Contains("stage=smooth-terminal-ramps")
            && !line.Contains("candidates=0 "));
        // Disabled experiments retain one original round-wide floor/wall proof,
        // after all optional candidates exist. Splitting the proof would reset
        // its work budget and could change cache query order before generation.
        for (int start = 0; start < diagnostics.Count; start++)
        {
            const string prefix = "stage=smooth-work event=begin work=correction-protection-";
            if (!diagnostics[start].StartsWith(prefix, StringComparison.Ordinal)) continue;
            string round = diagnostics[start][prefix.Length..];
            int end = diagnostics.FindIndex(start + 1, line => line.StartsWith(
                $"stage=smooth-work event=complete work=correction-protection-{round} ", StringComparison.Ordinal));
            Assert.True(end > start);
            var work = diagnostics.GetRange(start + 1, end - start - 1);
            string wall = Assert.Single(work.Where(line => line.Contains("stage=smooth-command-wall")));
            int terminals = work.FindIndex(line => line.Contains("stage=smooth-terminal-ramps"));
            if (terminals >= 0) Assert.True(work.IndexOf(wall) > terminals);
        }
        Assert.False(new VCarveOptions().AvoidRedundantTerminalRays);
    }

    private static VCarveCutterProfile Profile(bool rounded) => rounded
        ? new([new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 3)
        : new([new(CarvingCutterSectionKind.Linear, .01, 3.01, 3)], 3);

    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 7, "clearing", 3, detectCurves: false);
}
