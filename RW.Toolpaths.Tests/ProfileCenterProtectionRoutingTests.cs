using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ProfileCenterProtectionRoutingTests
{
    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1) => new()
    {
        new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)
    };

    [Fact]
    public void CenterClippingRetainsOriginalOpenFragmentsWithoutInventingStockBoundaryCuts()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rectangle(-2, 2, 12, 8)) },
            ToolGeometry.Flat(1), new DepthSchedule(2, 2),
            new ProfileOptions
            {
                Side = ProfileSide.OnLine, Ramp = RampSettings.Plunge,
                ClipBoundary = Rectangle(0, 0, 10, 10)
            });

        Assert.NotEmpty(plan.Toolpaths);
        var cutMoves = plan.Toolpaths.SelectMany(path => path.Spans
            .Where(span => span.Kind == ToolpathSpanKind.Cut)
            .SelectMany(span => Enumerable.Range(span.StartIndex, span.MoveCount)
                .Select(index => (Start: path.Points[index], End: path.Points[index + 1])))).ToArray();
        Assert.NotEmpty(cutMoves);
        Assert.All(cutMoves, move =>
        {
            Assert.InRange(move.Start.X, 0, 10);
            Assert.InRange(move.End.X, 0, 10);
            Assert.Equal(move.Start.Y, move.End.Y, 8);
            Assert.True(Math.Abs(move.Start.Y - 2) < 1e-8 || Math.Abs(move.Start.Y - 8) < 1e-8,
                $"Clipping invented boundary motion: {move.Start} -> {move.End}");
        });
        Assert.All(plan.Toolpaths, path => Assert.Equal(0, path.DepthPassIndex));
        Assert.Equal(20, plan.Diagnostics.CutLength, 6);
    }

    [Fact]
    public void CenterClippingRetainsCoincidentBoundaryOnlyWithinActualStock()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rectangle(-2, 0, 12, 8)) },
            ToolGeometry.Flat(1), new DepthSchedule(1, 1),
            new ProfileOptions
            {
                Side = ProfileSide.OnLine, Ramp = RampSettings.Plunge,
                ClipBoundary = Rectangle(0, 0, 10, 10)
            });
        Assert.Contains(plan.Toolpaths.SelectMany(path => path.Points), point => Math.Abs(point.Y) < 1e-8);
        Assert.All(plan.Toolpaths.SelectMany(path => path.Points), point => Assert.InRange(point.X, 0, 10));
        Assert.Equal(20, plan.Diagnostics.CutLength, 6);
    }

    [Fact]
    public void ProtectedCenterPolicySeparatesNearbyContoursBeforeIncomingStockProof()
    {
        var regions = new[]
        {
            new MillingRegion(Rectangle(0, 0, 100, 100)),
            new MillingRegion(Rectangle(20, -11, 30, -1))
        };
        var plan = ProfileToolpaths.Generate(regions, ToolGeometry.Flat(3), new DepthSchedule(2, 2),
            new ProfileOptions
            {
                Side = ProfileSide.OnLine, Ramp = RampSettings.Plunge,
                LinkClearance = LinkClearance.AlwaysLift
            });
        Assert.Equal(440, plan.Diagnostics.CutLength, 6);
        Assert.DoesNotContain(plan.Toolpaths.SelectMany(path => path.Spans), span => span.Kind == ToolpathSpanKind.Link);
        Assert.Equal(2, plan.Toolpaths.Count);
    }
}
