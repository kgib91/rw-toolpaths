using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class RasterMicrotextureFilterTests
{
    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(37, false)]
    [InlineData(37, true)]
    [InlineData(90, false)]
    [InlineData(133, true)]
    public void ShallowWaveTrainIsBridgedWithoutLoweringOrChangingDirection(double angle, bool reverse)
    {
        // Flat crests must not consume the merge span before the next valley.
        var original = Enumerable.Range(0, 121).Select(i =>
            Point(i * 0.05, i % 12 is <= 2 or >= 10 ? 3 : 2.97, angle)).ToArray();
        if (reverse) Array.Reverse(original);
        var result = RasterMicrotextureFilter.Apply(original, 1.2, 0.05);
        Assert.True(result.RemovedPoints > 50);
        Assert.Equal(0, result.ResultReversals);
        Assert.True(result.OriginalReversals > 10);
        Assert.Equal(original[0], result.Points[0]);
        Assert.Equal(original[^1], result.Points[^1]);
        CheckEnvelope(original, result, 1.2, 0.05);
        var opposite = RasterMicrotextureFilter.Apply(original.Reverse().ToArray(), 1.2, 0.05);
        Assert.Equal(result.Points, opposite.Points.Reverse());
    }

    [Theory]
    [InlineData(0)]
    [InlineData(0.8)]
    [InlineData(-0.7)]
    public void BroadValleyAndPlanarSlopesKeepTheirShape(double slope)
    {
        var valley = Enumerable.Range(0, 161).Select(i =>
        {
            double x = (i - 80) * 0.05;
            return Point(x, 5 + Math.Abs(x) * 0.4 + slope * x, 37);
        }).ToArray();
        var result = RasterMicrotextureFilter.Apply(valley, 1.2, 0.05);
        Assert.Equal(valley, result.Points);
        var plane = Enumerable.Range(0, 5000).Select(i => Point(i * 0.001, 5 + i * 0.001 * slope, 37)).ToArray();
        Assert.Equal(plane, RasterMicrotextureFilter.Apply(plane, 1.2, 0.05).Points);
    }

    [Fact]
    public void DeepNarrowValleyRemainsWhenItsAmplitudeExceedsAllowance()
    {
        Point3D[] original = [new(0, 0, 5), new(0.3, 0, 4), new(0.6, 0, 5)];
        var result = RasterMicrotextureFilter.Apply(original, 1.2, 0.05);
        Assert.Equal(original, result.Points);
        Assert.Equal(0, result.MaximumLift);
    }

    [Fact]
    public void NarrowProtectedPeakIsNeverCutAway()
    {
        Point3D[] original = [new(0, 0, 3), new(0.1, 0, 2.98), new(0.2, 0, 3),
            new(0.3, 0, 5), new(0.4, 0, 3), new(0.5, 0, 2.98), new(0.6, 0, 3)];
        var result = RasterMicrotextureFilter.Apply(original, 1.2, 0.05);
        Assert.Contains(original[3], result.Points);
        Assert.Equal(5, result.Points.Max(point => point.Z));
        CheckEnvelope(original, result, 1.2, 0.05);
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 37)]
    public void LongShelvesAndTinyEndpointTiltDoNotAnchorAShallowNarrowValley(bool reverse, double angle)
    {
        var original = Enumerable.Range(0, 2001).Select(i =>
        {
            double x = i * 0.01;
            double z = x is >= 9.7 and <= 10.3 ? 7 - 0.03 * Math.Sin(Math.PI * (x - 9.7) / 0.6) : 7;
            if (i == 2000) z += 0.0005;
            return Point(x, z, angle);
        }).ToArray();
        if (reverse) Array.Reverse(original);
        var result = RasterMicrotextureFilter.Apply(original, 1.2, 0.05);
        Assert.True(result.RemovedPoints > 50);
        Assert.All(result.Points, point => Assert.True(point.Z >= 7 - 1e-8));
        CheckEnvelope(original, result, 1.2, 0.05);
        Assert.Equal(original[0], result.Points[0]);
        Assert.Equal(original[^1], result.Points[^1]);
    }

    [Fact]
    public void SuccessiveMergesCannotAccumulateTheirOwnAllowance()
    {
        var random = new Random(240917);
        var original = Enumerable.Range(0, 12001).Select(i =>
        {
            double x = i * 0.003;
            return Point(x, 8 + 0.15 * x + 0.35 * Math.Sin(x)
                + 0.025 * Math.Cos(x * 32) + 0.015 * random.NextDouble(), 71);
        }).ToArray();
        var result = RasterMicrotextureFilter.Apply(original, 1.2, 0.05);
        Assert.True(result.RemovedPoints > 1000);
        CheckEnvelope(original, result, 1.2, 0.05);
        Assert.InRange(result.CandidateCount, 0, original.Length * 3);
        Assert.InRange(result.ProofNodesVisited, 0, result.CandidateCount * 256L);
    }

    [Fact]
    public void MillionPointDenseRowHasBoundedProofWork()
    {
        var original = Enumerable.Range(0, 1_000_001).Select(i =>
            new Point3D(i * 0.0001, 0, (i & 1) == 0 ? 3 : 2.97)).ToArray();
        var result = RasterMicrotextureFilter.Apply(original, 1.2, 0.05);
        Assert.Equal(500_000, result.RemovedPoints);
        Assert.Equal(0, result.ResultReversals);
        Assert.InRange(result.CandidateCount, 0, original.Length * 3);
        Assert.InRange(result.ProofNodesVisited, 0, result.CandidateCount * 256L);
        CheckEnvelope(original, result, 1.2, 0.05);
    }

    [Fact]
    public void CancelledFilteringDoesNotReturnPartialPath()
    {
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => RasterMicrotextureFilter.Apply(
            new Point3D[] { new(0, 0, 1), new(0.5, 0, 0.99), new(1, 0, 1) }, 1.2, 0.05, cancellation.Token));
    }

    private static Point3D Point(double distance, double height, double angle)
    {
        double radians = angle * Math.PI / 180;
        return new Point3D(12 + distance * Math.Cos(radians), -7 + distance * Math.Sin(radians), height);
    }

    private static void CheckEnvelope(IReadOnlyList<Point3D> original, RasterMicrotextureResult result,
        double maximumSpan, double maximumLift)
    {
        // Independent linear interpolation at every original vertex proves the
        // complete piecewise-linear envelope, including vertices removed long ago.
        int segment = 0, previousRetained = 0;
        double observed = 0;
        for (int i = 0; i < original.Count; i++)
        {
            if (segment + 1 < result.Points.Length - 1 && original[i] == result.Points[segment + 1])
            {
                if (i - previousRetained > 1) Assert.InRange(Distance(result.Points[segment], result.Points[segment + 1]), 0, maximumSpan + 1e-8);
                previousRetained = i; segment++;
            }
            var a = result.Points[segment]; var b = result.Points[segment + 1];
            double t = Distance(a, original[i]) / Distance(a, b);
            double lift = a.Z + (b.Z - a.Z) * t - original[i].Z;
            Assert.InRange(lift, -1e-8, maximumLift + 1e-8);
            observed = Math.Max(observed, lift);
        }
        if (original.Count - 1 - previousRetained > 1)
            Assert.InRange(Distance(result.Points[^2], result.Points[^1]), 0, maximumSpan + 1e-8);
        Assert.InRange(Math.Abs(observed - result.MaximumLift), 0, 1e-8);
    }

    private static double Distance(Point3D a, Point3D b) => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
}
