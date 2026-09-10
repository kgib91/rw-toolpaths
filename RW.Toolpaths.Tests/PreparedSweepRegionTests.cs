using Clipper2Lib;
using RW.Toolpaths.Geometry;
using Xunit;

namespace RW.Toolpaths.Tests;

public class PreparedSweepRegionTests
{
    [Fact]
    public void CapsuleProofPreservesHolesAndNarrowClearances()
    {
        var outer = new PathD { new(0, 0), new(20, 0), new(20, 20), new(0, 20) };
        var hole = new PathD { new(8, 8), new(12, 8), new(12, 12), new(8, 12) };
        var region = Clipper.Difference(new PathsD { outer }, new PathsD { hole }, FillRule.NonZero, 6);
        var prepared = new PreparedSweepRegion(region);
        Assert.True(prepared.Contains(new(2, 2), new(18, 2), 1));
        Assert.False(prepared.Contains(new(2, 10), new(18, 10), 1));
        Assert.False(prepared.Contains(new(10, 10), new(10, 10), 0.1));
        Assert.False(prepared.Contains(new(1, 1), new(1, 19), 1.001));
        var random = new Random(184);
        int proven = 0;
        for (int i = 0; i < 500; i++)
        {
            var a = new PointD(random.NextDouble() * 24 - 2, random.NextDouble() * 24 - 2);
            var b = new PointD(random.NextDouble() * 24 - 2, random.NextDouble() * 24 - 2);
            double radius = 0.1 + random.NextDouble();
            if (!prepared.Contains(a, b, radius)) continue;
            proven++;
            // Independent polygon boolean oracle for the accepted physical capsule.
            var stroke = new ClipperOffset(2, 0.00001);
            var centerline = new Path64 { new((long)Math.Round(a.x * 1e6), (long)Math.Round(a.y * 1e6)),
                new((long)Math.Round(b.x * 1e6), (long)Math.Round(b.y * 1e6)) };
            stroke.AddPath(centerline, JoinType.Round, EndType.Round);
            var required = new Paths64(); stroke.Execute(radius * 1e6, required);
            var boundary = new Paths64(region.Select(path => new Path64(path.Select(p =>
                new Point64((long)Math.Round(p.x * 1e6), (long)Math.Round(p.y * 1e6))))));
            Assert.Empty(Clipper.Difference(required, boundary, FillRule.NonZero));
        }
        Assert.True(proven > 20);
    }
}
