using Clipper2Lib;

namespace RW.Toolpaths.Tests;

public sealed class VCarveDepthFidelityTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ObservedOffCenterFiniteTipTransitionStaysWithinDepthTolerance(bool reverse)
    {
        var edge = (A: new PointD(266.2732849121094, 284.521484375),
            B: new PointD(266.26318359375, 284.53125));
        var start = new Point3D(266.2288818359375, 284.4925537109375, 0);
        var end = new Point3D(266.2484130859375, 284.49212646484375, 0);
        var boundary = new[] { edge };
        if (reverse) (start, end) = (end, start);
        var original = Snap([start, end], boundary, 0.05, 0.3125, 6.35);
        Assert.InRange(MaximumSampledOvercut(original, boundary, 0.05, 0.3125, 6.35), 0.0045, 0.0048);
        Assert.True(Math.Abs((original[0].Z + original[1].Z) / 2
            - ExpectedZ((start.X + end.X) / 2, (start.Y + end.Y) / 2, boundary, 0.05, 0.3125, 6.35)) < 0.003);

        var refined = Refine(original, boundary, 0.05, 0.3125, 6.35, 0.003);

        Assert.True(refined.Count > 2);
        Assert.True(refined.Count < 20);
        Assert.InRange(MaximumSampledOvercut(refined, boundary, 0.05, 0.3125, 6.35), 0, 0.003000001);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OffMidpointPointFeatureCannotHideBetweenDepthClampedEndpoints(bool reverse)
    {
        // The point site is closest at one quarter of the chord. Both endpoints and
        // the midpoint reach the depth cap, so midpoint-only refinement accepts it.
        (PointD A, PointD B)[] boundary = [(new(1, 0.1), new(1, 0.1))];
        var path = Snap([new(0, 0, 0), new(4, 0, 0)], boundary, 0.05, 0.3125, 1);
        if (reverse) path.Reverse();
        Assert.Equal(-1, path[0].Z);
        Assert.Equal(-1, path[1].Z);
        Assert.True(MaximumSampledOvercut(path, boundary, 0.05, 0.3125, 1) > 0.8);

        var refined = Refine(path, boundary, 0.05, 0.3125, 1, 0.003);

        Assert.InRange(MaximumSampledOvercut(refined, boundary, 0.05, 0.3125, 1), 0, 0.003000001);
    }

    [Fact]
    public void UnequalEndpointDepthsDoNotMakeLipschitzShortcutTooPermissive()
    {
        const double tolerance = 0.003, slope = 0.3125;
        double length = slope * tolerance * 2.8;
        // Distance to this vertical edge falls then rises. The endpoint-depth
        // difference is one tolerance, but the off-center minimum needs refinement.
        double center = (length - slope * tolerance) / 2;
        (PointD A, PointD B)[] boundary = [(new(center, -1), new(center, 1))];
        var path = Snap([new(0, 0, 0), new(length, 0, 0)], boundary, 0, slope, 1);
        var index = new MedialAxisToolpaths.SpatialSegmentIndex(boundary);

        Assert.True(index.TryFindDepthViolation(path[0], path[1], 0, 0, slope, tolerance, out _));
        var refined = Refine(path, boundary, 0, slope, 1, tolerance);
        Assert.InRange(MaximumSampledOvercut(refined, boundary, 0, slope, 1), 0, tolerance + 1e-9);
    }

    [Fact]
    public void StraightReachableConeSlopeRetainsItsTwoEndpoints()
    {
        (PointD A, PointD B)[] boundary = [(new(-10, 0), new(10, 0))];
        var path = Snap([new(0, 0.5, 0), new(1, 1, 0)], boundary, 0.05, 0.3125, 6.35);
        var refined = Refine(path, boundary, 0.05, 0.3125, 6.35, 0.003);
        Assert.Equal(2, refined.Count);
        for (int i = 0; i < 2; i++)
        {
            Assert.Equal(path[i].X, refined[i].X);
            Assert.Equal(path[i].Y, refined[i].Y);
            Assert.Equal(path[i].Z, refined[i].Z, precision: 12);
        }
    }

    private static List<Point3D> Refine(List<Point3D> path, IReadOnlyList<(PointD A, PointD B)> boundary,
        double bottom, double slope, double depth, double tolerance)
        => MedialAxisToolpaths.EnsureZFidelityFromBoundary(path,
            new MedialAxisToolpaths.SpatialSegmentIndex(boundary), 0, bottom, slope, -depth, tolerance);

    private static List<Point3D> Snap(List<Point3D> path, IReadOnlyList<(PointD A, PointD B)> boundary,
        double bottom, double slope, double depth)
        => path.Select(p => new Point3D(p.X, p.Y, ExpectedZ(p.X, p.Y, boundary, bottom, slope, depth))).ToList();

    private static double MaximumSampledOvercut(List<Point3D> path,
        IReadOnlyList<(PointD A, PointD B)> boundary, double bottom, double slope, double depth)
    {
        double maximum = 0;
        for (int i = 1; i < path.Count; i++)
        for (int sample = 0; sample <= 4096; sample++)
        {
            double t = sample / 4096d;
            var a = path[i - 1]; var b = path[i];
            double expected = ExpectedZ(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t,
                boundary, bottom, slope, depth);
            maximum = Math.Max(maximum, expected - (a.Z + (b.Z - a.Z) * t));
        }
        return maximum;
    }

    private static double ExpectedZ(double x, double y, IReadOnlyList<(PointD A, PointD B)> boundary,
        double bottom, double slope, double depth)
    {
        double distance = double.PositiveInfinity;
        foreach (var (a, b) in boundary)
        {
            double dx = b.x - a.x, dy = b.y - a.y;
            double squared = dx * dx + dy * dy;
            double t = squared == 0 ? 0 : Math.Clamp(((x - a.x) * dx + (y - a.y) * dy) / squared, 0, 1);
            double ex = x - a.x - t * dx, ey = y - a.y - t * dy;
            distance = Math.Min(distance, Math.Sqrt(ex * ex + ey * ey));
        }
        return -Math.Clamp((distance - bottom) / slope, 0, depth);
    }
}
