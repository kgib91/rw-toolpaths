using Clipper2Lib;
using RW.Toolpaths;
using Xunit;

namespace RW.Toolpaths.Tests;

public class PathUtilsCanonicalizeTests
{
    [Fact]
    public void CanonicalizeRings_ReturnsExplicitlyClosedRing()
    {
        var ring = new List<PointD>
        {
            new(0, 0),
            new(1, 0),
            new(1, 1),
            new(0, 1),
        };

        var canonical = PathUtils.CanonicalizeRings(new[] { ring });

        Assert.NotEmpty(canonical);
        Assert.True(canonical[0].Count >= 4);
        Assert.True(SamePoint(canonical[0][0], canonical[0][^1]));
    }

    private static bool SamePoint(PointD a, PointD b)
        => Math.Abs(a.x - b.x) <= 1e-12 && Math.Abs(a.y - b.y) <= 1e-12;
}
