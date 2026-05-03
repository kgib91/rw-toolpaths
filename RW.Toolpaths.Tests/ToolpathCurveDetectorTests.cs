using Clipper2Lib;
using RW.Toolpaths;

namespace RW.Toolpaths.Tests;

public class ToolpathCurveDetectorTests
{
    [Fact]
    public void DetectMoves_CircularRun_EmitsArcWithinTolerance()
    {
        var points = new List<Point3D>();
        const double radius = 2.0;
        for (int i = 0; i <= 160; i++)
        {
            double angle = i * Math.PI / 320.0;
            points.Add(new Point3D(radius * Math.Cos(angle), radius * Math.Sin(angle), -0.125));
        }

        var moves = ToolpathCurveDetector.DetectMoves(points).ToList();

        Assert.InRange(moves.Count, 1, 2);
        Assert.All(moves, move =>
        {
            Assert.Equal(ToolMoveKind.ArcCcw, move.Kind);
            Assert.Equal(0.0, move.ArcCenter.x, 10);
            Assert.Equal(0.0, move.ArcCenter.y, 10);
            Assert.Equal(radius, move.ArcRadius, 10);
            Assert.True(move.MaxError > 0.0);
            Assert.True(move.MaxError <= ToolpathCurveDetector.DefaultTolerance);
        });
    }

    [Fact]
    public void DetectMoves_SparseCircularRun_DoesNotEmitArcPastSpatialTolerance()
    {
        var points = new List<Point3D>();
        const double radius = 10.0;
        for (int i = 0; i <= 3; i++)
        {
            double angle = i * 0.1;
            points.Add(new Point3D(radius * Math.Cos(angle), radius * Math.Sin(angle), -0.125));
        }

        var moves = ToolpathCurveDetector.DetectMoves(points).ToList();

        Assert.DoesNotContain(moves, move => move.Kind is ToolMoveKind.ArcCw or ToolMoveKind.ArcCcw);
    }

    [Fact]
    public void DetectMoves_ImplicitClosedCircularRing_UsesCompactArcMoves()
    {
        var points = new List<Point3D>();
        const double radius = 1.5;
        for (int i = 0; i < 256; i++)
        {
            double angle = i * Math.PI * 2.0 / 256.0;
            points.Add(new Point3D(radius * Math.Cos(angle), radius * Math.Sin(angle), -0.2));
        }

        var moves = ToolpathCurveDetector.DetectMoves(points, closePath: true).ToList();

        Assert.True(moves.Count <= 4);
        Assert.All(moves, move => Assert.True(move.Kind is ToolMoveKind.ArcCw or ToolMoveKind.ArcCcw));
        Assert.All(moves, move => Assert.True(move.MaxError <= ToolpathCurveDetector.DefaultTolerance));
    }

    [Fact]
    public void DetectMoves_ParabolicMedialRun_EmitsBezierWithinTolerance()
    {
        var points = new List<Point3D>();
        for (int i = 0; i <= 80; i++)
        {
            double x = i / 80.0;
            double y = 0.35 * x * x;
            double z = -0.05 - 0.08 * x + 0.025 * x * x;
            points.Add(new Point3D(x, y, z));
        }

        var moves = ToolpathCurveDetector.DetectMoves(points).ToList();

        Assert.Contains(moves, move => move.Kind == ToolMoveKind.CubicBezier);
        Assert.All(moves, move => Assert.True(move.MaxError <= ToolpathCurveDetector.DefaultTolerance));
    }

    [Fact]
    public void DetectMoves_SharpTurn_DoesNotFitSingleBezierAcrossCorner()
    {
        var points = new List<Point3D>
        {
            new(0.00, 0.00, -0.1),
            new(0.25, 0.00, -0.1),
            new(0.50, 0.00, -0.1),
            new(0.50, 0.25, -0.1),
            new(0.50, 0.50, -0.1),
        };

        var moves = ToolpathCurveDetector.DetectMoves(points).ToList();

        Assert.DoesNotContain(moves, move => move.Kind == ToolMoveKind.CubicBezier);
        Assert.True(moves.Count >= 2);
    }

    [Fact]
    public void DetectMoves_SharpTurn_DoesNotFitArcAcrossCorner()
    {
        var points = new List<Point3D>
        {
            new(0.00, 0.00, -0.1),
            new(0.20, 0.00, -0.1),
            new(0.40, 0.00, -0.1),
            new(0.40, 0.20, -0.1),
            new(0.40, 0.40, -0.1),
        };

        var moves = ToolpathCurveDetector.DetectMoves(points).ToList();

        Assert.DoesNotContain(moves, move => move.Kind is ToolMoveKind.ArcCw or ToolMoveKind.ArcCcw);
        Assert.True(moves.Count >= 2);
    }

    [Fact]
    public void TaggedToolpath_DerivesMovesWithoutChangingPoints()
    {
        var region = new List<IReadOnlyList<PointD>>
        {
            new List<PointD>
            {
                new(0, 0),
                new(1, 0),
                new(1, 1),
                new(0, 1),
                new(0, 0),
            }
        };

        var tagged = MedialAxisToolpaths.GenerateClearingPassesTagged(
            boundary: region,
            startDepth: 0.0,
            endDepth: 0.2,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.1,
            stepOver: 0.4);

        Assert.NotEmpty(tagged);
        Assert.All(tagged, path =>
        {
            Assert.NotEmpty(path.Points);
            if (path.Points.Count >= 2)
                Assert.NotEmpty(path.Moves);
            Assert.All(path.Moves, move => Assert.True(move.MaxError <= ToolpathCurveDetector.DefaultTolerance));
        });
    }
}