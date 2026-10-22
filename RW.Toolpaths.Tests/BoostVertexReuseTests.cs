using Clipper2Lib;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Boost vertex allocation controls", DisableParallelization = true)]
public sealed class BoostVertexAllocationCollection;

[Collection("Boost vertex allocation controls")]
public sealed class BoostVertexReuseTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void ReusedNativeVertexObjectsPreserveEveryOrderedOutputBit(int fixture)
    {
        PointD[] Rectangle(double left, double bottom, double right, double top)
            => [new(left, bottom), new(right, bottom), new(right, top), new(left, top)];
        PointD[] outer = Rectangle(-12, -10, 13, 14);
        IReadOnlyList<IReadOnlyList<PointD>> holes = [];
        switch (fixture)
        {
            case 1:
                holes = [Rectangle(-4, -3, 5, 6).Reverse().ToArray()];
                break;
            case 2:
                holes = [Rectangle(-7, -6, -2, 4).Reverse().ToArray(),
                    Rectangle(1, -3, 8, 7).Reverse().ToArray()];
                break;
            case 3:
                outer = [new(0, 0), new(8, 0), new(8, 3.999), new(16, 3.999),
                    new(16, 0), new(24, 0), new(24, 8), new(16, 8),
                    new(16, 4.001), new(8, 4.001), new(8, 8), new(0, 8)];
                break;
            case 4:
                outer = Enumerable.Range(0, 512).Select(i => {
                    double angle = 2 * Math.PI * i / 512;
                    double radius = 10 + .01 * Math.Sin(13 * angle);
                    return new PointD(radius * Math.Cos(angle), radius * Math.Sin(angle));
                }).ToArray();
                holes = [Rectangle(-2, -1, 3, 2).Reverse().ToArray()];
                break;
        }
        const string setting = "RW_TOOLPATHS_VERTEX_ALLOCATION_REFERENCE";
        string? previous = Environment.GetEnvironmentVariable(setting);
        try
        {
            var provider = BoostVoronoiProvider.CreateDefault();
            Environment.SetEnvironmentVariable(setting, "1");
            var reference = MedialAxisGeometry.Build(outer, holes, provider, .01, -1, Math.PI + 1e-6);
            Environment.SetEnvironmentVariable(setting, null);
            var actual = MedialAxisGeometry.Build(outer, holes, provider, .01, -1, Math.PI + 1e-6);
            Assert.NotEmpty(reference);
            Assert.Equal(reference.Count, actual.Count);
            for (int i = 0; i < reference.Count; i++)
            {
                AssertPoint(reference[i].Point0, actual[i].Point0);
                AssertPoint(reference[i].Point1, actual[i].Point1);
            }
        }
        finally { Environment.SetEnvironmentVariable(setting, previous); }
    }

    private static void AssertPoint(MedialPoint expected, MedialPoint actual)
    {
        Assert.Equal(BitConverter.DoubleToInt64Bits(expected.X), BitConverter.DoubleToInt64Bits(actual.X));
        Assert.Equal(BitConverter.DoubleToInt64Bits(expected.Y), BitConverter.DoubleToInt64Bits(actual.Y));
        Assert.Equal(BitConverter.DoubleToInt64Bits(expected.Radius), BitConverter.DoubleToInt64Bits(actual.Radius));
    }
}
