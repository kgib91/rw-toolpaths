using System.Reflection;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketCommandGraphTests
{
    [Fact]
    public void NativeCollinearGraphEndpointCannotInventAnOffChordPostedSubdivision()
    {
        Point3D a = new(.0001, .0001, -7), b = new(.0101, .0171, -7);
        Point3D middle = new(a.X + (b.X - a.X) * .35, a.Y + (b.Y - a.Y) * .35, -7);
        Assert.True(Math.Abs((middle.X - a.X) * (b.Y - a.Y) - (middle.Y - a.Y) * (b.X - a.X)) < 1e-18);
        Assert.False(PocketContinuationRouter.PostedCollinear(a, middle, b));
        Assert.True(PocketContinuationRouter.PostedCollinear(new(.0001, .0001, -7), new(.0051, .0051, -7), new(.0101, .0101, -7)));
        Assert.False(PocketContinuationRouter.PostedCollinear(new(.0001, .0001, -7), new(.0201, .0201, -7), new(.0101, .0101, -7)));
        Assert.False(PocketContinuationRouter.PostedCollinear(new(.0001, .0001, -7), new(.0009, .0001, -7), new(.0002, .0002, -7)));
        Assert.False(PocketContinuationRouter.PostedCollinear(a, new(double.NaN, 0, -7), b));
    }

    [Fact]
    public void ExcursionPauseUsesAnExistingPostedCommandInsteadOfAnInteriorMinimizer()
    {
        Point3D[] points = [new(-30.0001, .0001, -7), new(-10.0001, .0001, -7),
            new(10.0001, .0001, -7), new(30.0001, .0001, -7)];
        var host = new TaggedToolpath(points,
            new ToolpathSpan[] { new(0, 3, ToolpathSpanKind.Cut) }, 0, "pocket", 1);
        var method = typeof(PocketToolpaths).GetMethod("TryFindRoutePause", BindingFlags.NonPublic | BindingFlags.Static)!;
        object Query(bool vertices)
        {
            object?[] arguments = [host, null, new Point3D(-2, 2, -7), new Point3D(2, 2, -7),
                double.MaxValue, new RW.Toolpaths.Geometry.PreparedCarrierSegments.Diagnostics(), null, vertices];
            Assert.True((bool)method.Invoke(null, arguments)!);
            return arguments[6]!;
        }
        var legacy = Query(false); var selected = Query(true);
        double Position(object pause) => (double)pause.GetType().GetProperty("SegmentPosition")!.GetValue(pause)!;
        Assert.InRange(Position(legacy), .1, .9);
        Assert.Contains(Position(selected), new[] { 0d, 1d });
        int segment = (int)selected.GetType().GetProperty("SegmentEndIndex")!.GetValue(selected)!;
        int endpoint = Position(selected) == 0 ? segment - 1 : segment;
        Assert.InRange(endpoint, 1, points.Length - 2);
    }
}
