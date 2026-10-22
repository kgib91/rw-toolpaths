using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class FirstFrontCoverageTests
{
    [Theory]
    [InlineData(.0005, 1)]
    [InlineData(.04, 2)]
    public void FirstComponentRemovalRequiresItsWholeCutterFootprintToBeCovered(double separation, int expected)
    {
        var main = Rectangle(0, 0, 20, 20);
        var small = new List<PointD>
        {
            new(20 + separation, 9.99), new(20 + separation + .0001, 10), new(20 + separation, 10.01),
        };
        // Both tiny filled triangles lie in the main contour's R=3 sweep.
        // Only the near-coincident triangle's COMPLETE footprint is redundant;
        // the other extends the reached wall by .04 mm and must survive.
        var result = RingOffsetEngine.RetainUncoveredFirstComponents(new() { small, main }, 3, CancellationToken.None);
        Assert.Equal(expected, result.Count);
        Assert.Contains(result, path => path.Count == 4 && path.Max(point => point.x) == 20);
        if (expected == 2)
            Assert.Contains(result, path => path.Max(point => point.x) > 20 + separation);
    }

    [Fact]
    public void ADownstreamClipCannotAuthorizeRemovalUsingAnOmittedFront()
    {
        var main = Rectangle(0, 0, 20, 20);
        var small = Rectangle(20.0004, 9.99, 20.0005, 10.01);
        var clip = Rectangle(20.0003, 0, 21, 20);
        var result = RingOffsetEngine.RetainUncoveredFirstComponents(new() { main, small }, 3,
            CancellationToken.None, clip);
        Assert.Equal(2, result.Count);
        Assert.Contains(result, path => path.Min(point => point.x) > 20);
    }

    [Fact]
    public void SeparateThinFirstFrontsRemainWhenEitherCutsNewMaterial()
    {
        var first = Rectangle(0, 0, .01, 10);
        var second = Rectangle(1, 0, 1.01, 10);
        var result = RingOffsetEngine.RetainUncoveredFirstComponents(new() { first, second }, 3, CancellationToken.None);
        Assert.Equal(2, result.Count);
    }

    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
}
