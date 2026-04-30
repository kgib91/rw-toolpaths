using Clipper2Lib;
using RW.Toolpaths;

namespace RW.Toolpaths.Tests;

public class RampUtilsTests
{
    [Fact]
    public void LoopIn_EmitsRampFromEntryToDepth()
    {
        var ring = new List<PointD>
        {
            new(0, 0),
            new(1, 0),
            new(1, 1),
            new(0, 1),
            new(0, 0)
        };

        const double depth = -0.20;
        const double entryZ = 0.0;
        const double rampAngle = Math.PI / 18.0; // 10 deg

        var ramp = RampUtils.LoopIn(ring, depth, entryZ, rampAngle);

        Assert.NotEmpty(ramp);
        Assert.True(ramp.Count >= 2, $"Expected at least 2 ramp points, got {ramp.Count}");

        // Ramp starts near entry height and ends at target depth.
        Assert.True(Math.Abs(ramp[0].Z - entryZ) < 1e-9, $"Expected first point at entryZ={entryZ}, got {ramp[0].Z}");
        Assert.True(Math.Abs(ramp[^1].Z - depth) < 1e-9, $"Expected last point at depth={depth}, got {ramp[^1].Z}");

        // Z must be monotone non-increasing from entry to depth.
        for (int i = 1; i < ramp.Count; i++)
            Assert.True(ramp[i].Z <= ramp[i - 1].Z + 1e-12, $"Ramp Z increased at index {i}: {ramp[i - 1].Z} -> {ramp[i].Z}");
    }
}
