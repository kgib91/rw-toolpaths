using Clipper2Lib;
using RW.Toolpaths;

namespace RW.Toolpaths.Tests;

public class MedialAxisCallContractTests
{
    [Fact]
    public void GenerateRawMedialAxisSegments_PassesExactKernelContractArguments()
    {
        var provider = new CapturingProvider();

        var region = new List<IReadOnlyList<PointD>>
        {
            new List<PointD>
            {
                new(0, 0), new(2, 0), new(2, 1), new(0, 1), new(0, 0)
            }
        };

        const double radianTipAngle = Math.PI / 3.0; // 60 deg
        const double depthPerPass = 0.5;
        const double tolerance = 0.03;

        // We only care about provider call contract, not output geometry.
        _ = MedialAxisToolpaths.GenerateRawMedialAxisSegments(
            provider,
            region,
            radianTipAngle,
            depthPerPass,
            tolerance,
            maxRadiusOverride: null);

        Assert.True(provider.WasCalled, "Expected provider to be invoked.");

        Assert.Equal(tolerance, provider.LastTolerance, 12);
        Assert.Equal(3.0 * Math.PI / 4.0, provider.LastFilteringAngle, 12);
        Assert.True(provider.LastUseBigIntegers);

        // JS g_/y_ behavior: maxRadius = tan(radianTipAngle / 2) * depthPerPass,
        // then multiplied by scale before calling construct_medial_axis.
        double expectedRadius = Math.Tan(radianTipAngle / 2.0) * depthPerPass * 4096.0;
        Assert.Equal(expectedRadius, provider.LastMaxRadius, 8);
    }

    private sealed class CapturingProvider : IMedialAxisProvider
    {
        public bool WasCalled { get; private set; }
        public double LastTolerance { get; private set; }
        public double LastMaxRadius { get; private set; }
        public double LastFilteringAngle { get; private set; }
        public bool LastUseBigIntegers { get; private set; }

        public IReadOnlyList<MedialSegment> ConstructMedialAxis(
            IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes,
            double tolerance,
            double maxRadius,
            double filteringAngle = 3 * Math.PI / 4,
            bool useBigIntegers = true)
        {
            WasCalled = true;
            LastTolerance = tolerance;
            LastMaxRadius = maxRadius;
            LastFilteringAngle = filteringAngle;
            LastUseBigIntegers = useBigIntegers;

            return Array.Empty<MedialSegment>();
        }
    }

    [Fact]
    public void Generate_DefaultRdpTolerance_PreservesProviderBends()
    {
        var provider = new ShallowBendProvider();
        var region = new List<IReadOnlyList<PointD>>
        {
            new List<PointD>
            {
                new(0, 0), new(1, 0), new(1, 1), new(0, 1), new(0, 0)
            }
        };

        var result = MedialAxisToolpaths.Generate(
            provider,
            region,
            startDepth: 0.0,
            endDepth: 1.0,
            radianTipAngle: Math.PI / 2.0,
            depthPerPass: 1.0,
            tolerance: 0.03);

        Assert.NotNull(result);
        var path = Assert.Single(result!);
        Assert.Equal(3, path.Count);
        Assert.Equal(0.10, path[1].X, 6);
        Assert.Equal(0.001, path[1].Y, 6);
    }

    private sealed class ShallowBendProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(
            IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes,
            double tolerance,
            double maxRadius,
            double filteringAngle = 3 * Math.PI / 4,
            bool useBigIntegers = true)
        {
            const double scale = 4096.0;
            const double radius = 0.1 * scale;

            return new[]
            {
                new MedialSegment(
                    new MedialPoint(0.00 * scale, 0.000 * scale, radius),
                    new MedialPoint(0.10 * scale, 0.001 * scale, radius)),
                new MedialSegment(
                    new MedialPoint(0.10 * scale, 0.001 * scale, radius),
                    new MedialPoint(0.20 * scale, 0.000 * scale, radius)),
            };
        }
    }
}

// --- RdpSimplify3D -----------------------------------------------------------
public class RdpSimplify3DTests
{
    private static Point3D P(double x, double y, double z = 0) => new(x, y, z);

    [Fact]
    public void EmptyAndTinyPaths_ReturnedUnchanged()
    {
        Assert.Empty(MedialAxisToolpaths.RdpSimplify3D(new List<Point3D>(), 0.01));
        var one = new List<Point3D> { P(0, 0) };
        Assert.Equal(one, MedialAxisToolpaths.RdpSimplify3D(one, 0.01));
        var two = new List<Point3D> { P(0, 0), P(1, 0) };
        Assert.Equal(two, MedialAxisToolpaths.RdpSimplify3D(two, 0.01));
    }

    [Fact]
    public void CollinearPoints_ReducedToEndpoints()
    {
        // Five perfectly collinear points -> keep only start and end
        var path = new List<Point3D> { P(0,0), P(1,0), P(2,0), P(3,0), P(4,0) };
        var result = MedialAxisToolpaths.RdpSimplify3D(path, 0.001);
        Assert.Equal(2, result.Count);
        Assert.Equal(P(0, 0), result[0]);
        Assert.Equal(P(4, 0), result[1]);
    }

    [Fact]
    public void LargeDeviationPoint_IsPreserved()
    {
        // Middle point is 1.0 unit off the chord — well above any tolerance
        var path = new List<Point3D> { P(0,0), P(1,1), P(2,0) };
        var result = MedialAxisToolpaths.RdpSimplify3D(path, 0.001);
        Assert.Equal(3, result.Count);
    }

    [Fact]
    public void DenseArc_ReducedSignificantly()
    {
        // Sample a quarter-circle arc with 100 points
        var path = new List<Point3D>();
        for (int i = 0; i <= 100; i++)
        {
            double t = i / 100.0 * Math.PI / 2.0;
            path.Add(P(Math.Cos(t), Math.Sin(t)));
        }
        var result = MedialAxisToolpaths.RdpSimplify3D(path, 0.001);
        // 0.001 tolerance should keep far fewer than 100 points while still
        // covering the arc endpoints
        Assert.True(result.Count < 50,
            $"Expected < 50 points for tol=0.001, got {result.Count}");
        Assert.Equal(path[0], result[0]);
        Assert.Equal(path[^1], result[^1]);
    }

    [Fact]
    public void LooserTolerance_FewerPoints()
    {
        var path = new List<Point3D>();
        for (int i = 0; i <= 200; i++)
        {
            double t = i / 200.0 * Math.PI * 2;
            path.Add(new Point3D(Math.Cos(t), Math.Sin(t), 0));
        }
        var tight  = MedialAxisToolpaths.RdpSimplify3D(path, 0.0001);
        var loose  = MedialAxisToolpaths.RdpSimplify3D(path, 0.01);
        Assert.True(loose.Count < tight.Count,
            $"Looser tolerance ({loose.Count}) should yield fewer points than tight ({tight.Count})");
    }

    [Fact]
    public void ZeroTolerance_PreservesAllPoints()
    {
        var path = new List<Point3D> { P(0,0), P(1,0.001), P(2,0), P(3,-0.001), P(4,0) };
        var result = MedialAxisToolpaths.RdpSimplify3D(path, 0.0);
        Assert.Equal(path.Count, result.Count);
    }

    [Fact]
    public void SharpCorner_IsPreservedWithLargeTolerance()
    {
        // Regression: global RDP can drop short valid corner spokes.
        // This path has a clear 90-degree corner that must be preserved.
        var path = new List<Point3D>
        {
            P(0.00, 0.00),
            P(0.50, 0.00),
            P(1.00, 0.00), // corner anchor
            P(1.00, 0.50),
            P(1.00, 1.00)
        };

        var result = MedialAxisToolpaths.RdpSimplify3D(path, 0.25);

        Assert.Contains(P(1.00, 0.00), result);
        Assert.Equal(path[0], result[0]);
        Assert.Equal(path[^1], result[^1]);
    }
}

public class ClearingPassStepOverTests
{
    [Fact]
    public void GenerateClearingPasses_LargerBottomRadius_WidensIntermediateLayerSpacing()
    {
        var boundary = new List<IReadOnlyList<PointD>>
        {
            new List<PointD>
            {
                new(0, 0),
                new(4, 0),
                new(4, 4),
                new(0, 4),
                new(0, 0)
            }
        };

        const double startDepth = 0.0;
        const double endDepth = 0.3;
        const double depthPerPass = 0.1;
        const double stepOver = 0.5;

        // angle only affects derived defaults; overrides below define the profile.
        const double tipAngle = Math.PI / 3.0;
        const double topRadius = 0.3;
        const double coneLength = 0.3;

        var smallBottom = MedialAxisToolpaths.GenerateClearingPasses(
            boundary,
            startDepth,
            endDepth,
            tipAngle,
            depthPerPass,
            stepOver,
            bottomRadiusOverride: 0.0,
            topRadiusOverride: topRadius,
            coneLengthOverride: coneLength);

        var largerBottom = MedialAxisToolpaths.GenerateClearingPasses(
            boundary,
            startDepth,
            endDepth,
            tipAngle,
            depthPerPass,
            stepOver,
            bottomRadiusOverride: 0.15,
            topRadiusOverride: topRadius,
            coneLengthOverride: coneLength);

        int smallAtFirstLayer = smallBottom.Count(p => p.Count > 0 && Math.Abs(p[0].Z + 0.1) < 1e-9);
        int largeAtFirstLayer = largerBottom.Count(p => p.Count > 0 && Math.Abs(p[0].Z + 0.1) < 1e-9);

        Assert.True(smallAtFirstLayer > 0, "Expected at least one path at z=-0.1 for baseline profile.");
        Assert.True(largeAtFirstLayer > 0, "Expected at least one path at z=-0.1 for larger-bottom profile.");
        Assert.True(
            largeAtFirstLayer < smallAtFirstLayer,
            $"Expected fewer z=-0.1 paths with larger bottom radius; baseline={smallAtFirstLayer}, largerBottom={largeAtFirstLayer}");
    }
}
