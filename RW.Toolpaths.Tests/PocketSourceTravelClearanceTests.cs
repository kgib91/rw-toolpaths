using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketSourceTravelClearanceTests
{
    private const double Requested = 3.175 + 1.587499976158142;

    [Fact]
    public void SourceMemoReusesXYAcrossPassesButAlwaysRechecksTheCallersCurrentStockAndDepth()
    {
        var caller=new ChangingCaller();
        var source=new PocketToolpaths.FillSourceClearance([new(Rectangle(0,0,80,80))],Requested,caller,null);
        Assert.True(source.IsTravelSafe(new(10,10,-3),new(30,10,-3),3.175));
        Assert.True(source.IsTravelSafe(new(10,10,-7),new(30,10,-7),3.175));
        Assert.True(source.SourceMemoHits>0);
        caller.Allowed=false;
        Assert.False(source.IsTravelSafe(new(10,10,-7),new(30,10,-7),3.175));
        caller.Allowed=true;
        Assert.False(source.IsTravelSafe(new(10,10,-20),new(30,10,-20),3.175));
        Assert.Equal(4,caller.Queries);
        var hole=new PocketToolpaths.FillSourceClearance([new(Rectangle(0,0,80,80),[Rectangle(18,8,22,12)])],Requested,null,null);
        Assert.False(hole.IsTravelSafe(new(10,10,-3),new(30,10,-3),3.175));
    }

    private sealed class ChangingCaller:ILinkClearance
    {
        internal bool Allowed=true;
        internal int Queries;
        public bool IsTravelSafe(Point3D from,Point3D to,double radius)
        {Queries++;return Allowed&&from.Z>=-10&&to.Z>=-10;}
    }

    [Fact]
    public void AuditedBackgroundContinuationCannotSpendSourceApproximationOnStockLeave()
    {
        // Candidate08 raw203311's actual stored float endpoints and the original
        // source's five adjacent wall vertices. Closing the protected shape far
        // to the right leaves the exact closest wall witness unchanged.
        var wall = new List<PointD> { new(205.87510483, 424.65358166), new(205.87039311, 424.74003709),
            new(205.86702623, 424.82370548), new(205.86500529, 424.90457547),
            new(205.8643314, 424.98263569), new(230, 426), new(230, 424), new(205.88, 424.5) };
        var outer = Rectangle(0, 0, 345, 500);
        var source = new MillingRegion(outer, new[] { wall });
        var from = new Point3D(201.11167907714844, 424.5472717285156, -3.5);
        var to = new Point3D(200.83209228515625, 433.3264465332031, -3.5);
        var index = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(PolygonOps.Difference(new[] { outer }, new[] { wall })));
        Assert.True(index.ContainsCapsule(Native(from), Native(to), (Requested - .001) * PathUtils.Scale - 4));
        var guarded = new PocketToolpaths.FillSourceClearance(new[] { source }, Requested, null, null);
        Assert.False(guarded.IsTravelSafe(from, to, 3.175));
        var postedA = Posted(from); var postedB = Posted(to);
        double actual = ChordDistance(postedA, postedB, wall);
        Assert.Equal(4.761430398413065, actual, 8);
        Assert.True(actual < Requested - Math.Sqrt(2) * .0005 - 4 / PathUtils.Scale);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(17)]
    [InlineData(45)]
    [InlineData(89)]
    public void RotatedHalfGridWallsKeepOnlyNativeFirstCommandReserveAndPostedBand(double angleDegrees)
    {
        double angle = angleDegrees * Math.PI / 180;
        PointD Rotate(double x, double y) => new(201.000015258789 + x * Math.Cos(angle) - y * Math.Sin(angle),
            424.000015258789 + x * Math.Sin(angle) + y * Math.Cos(angle));
        var source = new List<PointD> { Rotate(0, 0), Rotate(40, 0), Rotate(40, 60), Rotate(0, 60) };
        var clearance = new PocketToolpaths.FillSourceClearance(new[] { new MillingRegion(source) }, Requested, null, null);
        Point3D At(double x, double y) { var p = Rotate(x, y); return new(p.x, p.y, -7); }
        var allowedA = At(Requested + .0005, 10); var allowedB = At(Requested + .0005, 40);
        Assert.True(clearance.IsTravelSafe(allowedA, allowedB, 3.175));
        double posted = ChordDistance(Posted(allowedA), Posted(allowedB), source);
        Assert.True(posted >= Requested - Math.Sqrt(2) * .0005 - 4 / PathUtils.Scale);
        Assert.False(clearance.IsTravelSafe(At(Requested - .0005, 10), At(Requested - .0005, 40), 3.175));
    }

    [Fact]
    public void CompleteSourceCapsuleIncludesEnclosedProtectedHoleAndCallerRestriction()
    {
        var region = new MillingRegion(Rectangle(0, 0, 80, 80), new[] { Rectangle(35, 30, 45, 50) });
        var source = new PocketToolpaths.FillSourceClearance(new[] { region }, Requested, null, null);
        var a = new Point3D(10, 40, -3.5); var b = new Point3D(70, 40, -3.5);
        Assert.False(source.IsTravelSafe(a, b, 3.175));
        Assert.True(source.IsTravelSafe(new(10, 10, -3.5), new(70, 10, -3.5), 3.175));
        var restricted = new PocketToolpaths.FillSourceClearance(new[] { region }, Requested, LinkClearance.AlwaysLift, null);
        Assert.False(restricted.IsTravelSafe(new(10, 10, -3.5), new(70, 10, -3.5), 3.175));
        var clipped = new PocketToolpaths.FillSourceClearance(new[] { region }, Requested, null, Rectangle(0, 0, 20, 80));
        Assert.False(clipped.IsTravelSafe(new(10, 10, -3.5), new(70, 10, -3.5), 3.175));
    }

    private static Point64 Native(Point3D p) => new((long)Math.Round(p.X * PathUtils.Scale), (long)Math.Round(p.Y * PathUtils.Scale));
    private static PointD Posted(Point3D p) => new(Math.Round((double)(float)p.X, 3, MidpointRounding.ToEven),
        Math.Round((double)(float)p.Y, 3, MidpointRounding.ToEven));
    private static double ChordDistance(PointD a, PointD b, IReadOnlyList<PointD> closed)
    {
        double minimum = double.PositiveInfinity;
        for (int i = 0; i < closed.Count; i++)
        {
            var c = closed[i]; var d = closed[(i + 1) % closed.Count];
            minimum = Math.Min(minimum, Math.Min(Math.Min(PointDistance(a, c, d), PointDistance(b, c, d)),
                Math.Min(PointDistance(c, a, b), PointDistance(d, a, b))));
        }
        return minimum;
    }
    private static double PointDistance(PointD p, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y, denominator = dx * dx + dy * dy;
        double t = denominator == 0 ? 0 : Math.Clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / denominator, 0, 1);
        return Math.Sqrt(Math.Pow(p.x - a.x - t * dx, 2) + Math.Pow(p.y - a.y - t * dy, 2));
    }
    private static List<PointD> Rectangle(double left, double bottom, double right, double top)
        => new() { new(left, bottom), new(right, bottom), new(right, top), new(left, top) };
}
