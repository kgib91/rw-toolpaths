using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.Certification;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class VCarveEndpointVolumeDominanceTests(ITestOutputHelper output)
{
    [Theory]
    [InlineData(0, 0)]
    [InlineData(400, 10000)]
    [InlineData(-9999, -9000)]
    public void SlopedPostedChordProvesAndSamplesCompleteStockVolume(double origin, double surface)
    {
        var profile = BallTaper();
        var a = Posted(origin + .02, origin + .01, surface - .002);
        var b = Posted(origin + .03, origin + .01, surface - .001);
        var mandatory = Posted(origin, origin, surface - .2);
        Assert.True(VCarveEndpointVolumeDominance.TryProve(a, b, mandatory, profile, surface,
            out var proof, diagnostic: output.WriteLine), proof.Describe());
        Assert.Equal("strict-radial-margin", proof.Mode);
        Assert.Equal(2, proof.RadiusEvaluations);
        Assert.True(proof.UnreservedMargin > VCarveEndpointVolumeDominance.ArithmeticReserve);
        SampleFullStockVolume(a, b, mandatory, profile, surface);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExactSameAxisCylinderIdentityUsesNoRadialAllowance(bool reverse)
    {
        var profile = Cylinder(.25, 1);
        var a = new Point3D(400, -300, .25);
        var b = new Point3D(400, -300, -.5);
        var mandatory = new Point3D(400, -300, -1);
        if (reverse) (a, b) = (b, a);
        Assert.True(VCarveEndpointVolumeDominance.TryProve(a, b, mandatory, profile, 0,
            out var proof, diagnostic: output.WriteLine), proof.Describe());
        Assert.Equal("exact-same-axis", proof.Mode); Assert.Equal(0, proof.RadiusEvaluations);
        Assert.Equal(0, proof.MaximumAxisDistance);
        Assert.True(double.IsNaN(proof.RequiredRadius));
        SampleFullStockVolume(a, b, mandatory, profile, 0);
        // A displaced cylinder cannot borrow a reserve as a machining tolerance.
        Assert.False(VCarveEndpointVolumeDominance.TryProve(a with { X = a.X + .001 }, b,
            mandatory, profile, 0, out var displaced, diagnostic: output.WriteLine));
        Assert.Equal("insufficient-strict-volume-margin", displaced.Reason);
    }

    [Fact]
    public void SameAxisPointedRayAtTheSameDepthIsExactWithoutTheUniformRadialBound()
    {
        var profile = Cone(); var a = new Point3D(0, 0, 0); var b = new Point3D(0, 0, -.2);
        Assert.True(VCarveEndpointVolumeDominance.TryProve(a, b, b, profile, 0,
            out var proof, diagnostic: output.WriteLine));
        Assert.Equal("exact-same-axis", proof.Mode);
        Assert.Equal(0, proof.MinimumMandatoryAxialHeight); Assert.Equal(0, proof.RadiusEvaluations);
        SampleFullStockVolume(a, b, b, profile, 0);
    }

    [Theory]
    [InlineData(.001, .151, .02)]
    [InlineData(.149, .3, .001)]
    [InlineData(.15, .31, .002)]
    public void ShallowAndDeepSectionJoinsRemainFullVolumeQueries(double sourceDepth, double mandatoryDepth, double offset)
    {
        var profile = BallTaper();
        var a = new Point3D(offset, 0, 0); var b = new Point3D(offset, 0, -sourceDepth);
        var mandatory = new Point3D(0, 0, -mandatoryDepth);
        Assert.True(VCarveEndpointVolumeDominance.TryProve(a, b, mandatory, profile, 0,
            out var proof, diagnostic: output.WriteLine), proof.Describe());
        Assert.Equal("strict-radial-margin", proof.Mode);
        SampleFullStockVolume(a, b, mandatory, profile, 0);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(.00000002, true)]
    [InlineData(-.00000002, false)]
    public void GeneralBoundNeedsStrictPhysicalMarginWithoutAnAddedTolerance(double unreservedMargin, bool expected)
    {
        double gradient = 1 + unreservedMargin / .1;
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 2 * gradient, 2)], 2);
        var a = new Point3D(0, 0, 0); var b = new Point3D(.1, 0, -.1); var mandatory = new Point3D(0, 0, -.3);
        Assert.Equal(expected, VCarveEndpointVolumeDominance.TryProve(a, b, mandatory, profile, 0,
            out var proof, diagnostic: output.WriteLine));
        Assert.Equal(2, proof.RadiusEvaluations);
        if (expected) SampleFullStockVolume(a, b, mandatory, profile, 0);
        else Assert.Equal("insufficient-strict-volume-margin", proof.Reason);
    }

    [Fact]
    public void APositiveSurfaceDiscComparisonCannotHideAnUncutSubsurfaceSlice()
    {
        var profile = BallTaper();
        var a = new Point3D(.009, 0, 0); var b = new Point3D(.009, 0, -.2); var mandatory = new Point3D(0, 0, -.21);
        Assert.True(.009 + profile.PhysicalRadiusAtDepth(.2) < profile.PhysicalRadiusAtDepth(.21));
        const double z = -.055;
        double sourceHeight = z - b.Z, mandatoryHeight = z - mandatory.Z;
        var interiorSourcePoint = new Point3D(b.X + profile.PhysicalRadiusAtDepth(sourceHeight) - 1e-6, 0, z);
        Assert.True(Distance(interiorSourcePoint, b) < profile.PhysicalRadiusAtDepth(sourceHeight));
        Assert.True(Distance(interiorSourcePoint, mandatory) > profile.PhysicalRadiusAtDepth(mandatoryHeight));
        Assert.False(VCarveEndpointVolumeDominance.TryProve(a, b, mandatory, profile, 0,
            out var proof, diagnostic: output.WriteLine));
        Assert.Equal("insufficient-strict-volume-margin", proof.Reason);
        output.WriteLine($"[VCarve.EndpointVolume.Test] surfaceDiscCovered=True subsurfaceMiss=True "
            + $"stockZ={z:R} sourceHeight={sourceHeight:R} targetHeight={mandatoryHeight:R} addedTolerance=0");
    }

    [Fact]
    public void OffCapDiscontinuityNonmonotonicityAndMicroscopicJoinNeverGainAuthority()
    {
        VCarveCutterProfile[] unsupported =
        [
            new([new(CarvingCutterSectionKind.Linear, .25, .25, 1),
                new(CarvingCutterSectionKind.Linear, .3, .5, 1)], .5),
            new([new(CarvingCutterSectionKind.Linear, .25, .25, 1),
                new(CarvingCutterSectionKind.Linear, .25, .2, 1)], .5),
            new([new(CarvingCutterSectionKind.Linear, .25, .25, 1),
                new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)], .5),
            new([new(CarvingCutterSectionKind.Linear, .25, .25, 1),
                new(CarvingCutterSectionKind.Linear, .25, .25, 1e-20)], 1),
            new([new(CarvingCutterSectionKind.Linear, 0, 2, .01)], .01),
            new([new(CarvingCutterSectionKind.Hemisphere, 0, 1e-8, 1e-8)], 1e-8),
        ];
        foreach (var profile in unsupported)
        {
            var prepared = VCarveEndpointVolumeDominance.Prepare(profile);
            Assert.False(prepared.Supported);
            Assert.False(VCarveEndpointVolumeDominance.TryProve(new(0, 0, 0), new(0, 0, -.001),
                new(0, 0, -.1), prepared, 0, out var proof, diagnostic: output.WriteLine));
            Assert.Equal(prepared.Reason, proof.Reason); Assert.Equal(0, proof.RadiusEvaluations);
        }
    }

    [Fact]
    public void NominalCapNeverClampsActualRepresentedPhysicalStock()
    {
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 2, 2)], .1);
        var a = new Point3D(.02, 0, 0); var b = new Point3D(.02, 0, -.001); var mandatory = new Point3D(0, 0, -.2);
        Assert.True(VCarveEndpointVolumeDominance.TryProve(a, b, mandatory, profile, 0,
            out var proof, diagnostic: output.WriteLine));
        Assert.True(proof.MandatoryDepth > profile.MaximumDepth);
        SampleFullStockVolume(a, b, mandatory, profile, 0);
    }

    [Fact]
    public void AmbiguousRepresentedTopAndNearTipDerivativeDomainsRefuseConservatively()
    {
        var ball = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, .15, .15)], .15);
        Assert.False(VCarveEndpointVolumeDominance.TryProve(new(0, 0, 0), new(0, 0, -.001), new(0, 0, -.15), ball, 0,
            out var boundary, diagnostic: output.WriteLine));
        Assert.Equal("outside-or-ambiguous-represented-height", boundary.Reason);
        var largeBall = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 100, 100)], 100);
        Assert.False(VCarveEndpointVolumeDominance.TryProve(new(.001, 0, 0), new(.001, 0, 0), new(0, 0, -1), largeBall, .0002,
            out var nearTip, diagnostic: output.WriteLine));
        Assert.Equal("near-tip-radius-evaluation-domain", nearTip.Reason);
        // The same-axis proof has no derivative singularity or radius evaluation.
        Assert.True(VCarveEndpointVolumeDominance.TryProve(new(0, 0, 0), new(0, 0, 0), new(0, 0, -1), largeBall, .0002,
            out var identity, diagnostic: output.WriteLine));
        Assert.Equal("exact-same-axis", identity.Mode); Assert.Equal(0, identity.RadiusEvaluations);
    }

    [Fact]
    public void MalformedUnpostedUnengagedShallowerAndUnrepresentedCommandsRefuse()
    {
        var profile = Cone(); var a = new Point3D(.02, 0, 0); var b = new Point3D(.02, 0, -.001); var m = new Point3D(0, 0, -.2);
        (Point3D A, Point3D B, Point3D M, double Surface)[] invalid =
        [
            (a with { X = double.NaN }, b, m, 0), (a, b with { Z = double.PositiveInfinity }, m, 0),
            (a, b, m with { X = 10000.001 }, 0), (a with { X = .0200001 }, b, m, 0),
            (a, b, m, double.NaN), (a, b, m, 10000.001),
            (a with { Z = 1 }, b with { Z = 1 }, m, 0), (a with { Z = 0 }, b with { Z = 0 }, m, 0),
            (a, b, m with { Z = 0 }, 0), (a, b with { Z = -.3 }, m, 0),
            (a, b, m with { Z = -2.001 }, 0),
        ];
        foreach (var query in invalid)
        {
            Assert.False(VCarveEndpointVolumeDominance.TryProve(query.A, query.B, query.M, profile, query.Surface,
                out var proof, diagnostic: output.WriteLine));
            Assert.False(proof.Proven); Assert.Equal(0, proof.RadiusEvaluations);
        }
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveEndpointVolumeDominance.TryProve(a, b, m, profile, 0,
            out _, cancellation.Token));
    }

    private static VCarveCutterProfile Cone() => new([new(CarvingCutterSectionKind.Linear, 0, 2, 2)], 2);
    private static VCarveCutterProfile Cylinder(double radius, double height)
        => new([new(CarvingCutterSectionKind.Linear, radius, radius, height)], height);
    private static VCarveCutterProfile BallTaper()
        => new([new(CarvingCutterSectionKind.Hemisphere, 0, .15, .15), new(CarvingCutterSectionKind.Linear, .15, 1.15, 1)], 1);
    private static Point3D Posted(double x, double y, double z)
        => new(Math.Round(x * 1000) / 1000, Math.Round(y * 1000) / 1000, Math.Round(z * 1000) / 1000);
    private static double Distance(Point3D a, Point3D b)
    { double x = a.X - b.X, y = a.Y - b.Y; return Math.Sqrt(x * x + y * y); }

    private void SampleFullStockVolume(Point3D a, Point3D b, Point3D mandatory, VCarveCutterProfile profile, double surface)
    {
        int samples = 0;
        for (int step = 0; step <= 32; step++)
        {
            double t = step / 32d;
            var pose = new Point3D(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);
            if (pose.Z > surface) continue;
            for (int level = 0; level <= 16; level++)
            {
                double z = pose.Z + (surface - pose.Z) * level / 16;
                double h = z - pose.Z, targetH = z - mandatory.Z;
                double sourceRadius = profile.PhysicalRadiusAtDepth(h), targetRadius = profile.PhysicalRadiusAtDepth(targetH);
                foreach (double fraction in new[] { 0d, .5, .9999 })
                for (int direction = 0; direction < 16; direction++)
                {
                    double theta = 2 * Math.PI * direction / 16;
                    var point = new Point3D(pose.X + sourceRadius * fraction * Math.Cos(theta),
                        pose.Y + sourceRadius * fraction * Math.Sin(theta), z);
                    Assert.True(targetH >= 0 && Distance(point, mandatory) <= targetRadius,
                        $"Complete stock slice misses t={t:R},z={z:R},sourceHeight={h:R},targetHeight={targetH:R},point=({point.X:R},{point.Y:R}).");
                    samples++;
                }
            }
        }
        output.WriteLine($"[VCarve.EndpointVolume.Test] independentContinuousPoseStockHeightRadialSamples={samples} complete3D=True addedTolerance=0");
    }
}
