using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

/// <summary>
/// Entry moves have to descend at the requested angle, never overshoot the pass depth, and
/// always leave the ring fully machined at final depth.
/// </summary>
public class RampPlannerTests
{
    private static List<PointD> Square(double size) => new()
    {
        new PointD(0, 0),
        new PointD(size, 0),
        new PointD(size, size),
        new PointD(0, size),
    };

    // --- Settings -------------------------------------------------------------

    [Fact]
    public void FromRatio_ConvertsHorizontalPerVerticalToAngle()
    {
        // A 2:1 ratio travels 2mm across per 1mm down.
        var settings = RampSettings.FromRatio(2.0);

        Assert.Equal(Math.Atan(0.5), settings.AngleRadians, 9);
        Assert.Equal(20.0, settings.HorizontalRunFor(10.0), 6);
    }

    [Fact]
    public void FromRatio_NonPositiveRatioBecomesPlunge()
    {
        Assert.Equal(RampStrategy.Plunge, RampSettings.FromRatio(0).Strategy);
        Assert.Equal(RampStrategy.Plunge, RampSettings.FromRatio(-3).Strategy);
    }

    // --- Strategy selection ---------------------------------------------------

    [Fact]
    public void Auto_LongRingUsesLinearRamp()
    {
        // 400mm perimeter absorbs a 20mm ramp inside a single lap.
        var result = RampPlanner.PlanClosedRing(Square(100), 0, -10, RampSettings.FromRatio(2.0));

        Assert.Equal(RampStrategy.Linear, result.Strategy);
    }

    [Fact]
    public void Auto_ShortRingUsesHelicalRamp()
    {
        // A 4mm square has a 16mm perimeter and cannot absorb a 20mm ramp in one lap.
        var result = RampPlanner.PlanClosedRing(Square(4), 0, -10, RampSettings.FromRatio(2.0));

        Assert.Equal(RampStrategy.Helical, result.Strategy);
        Assert.True(result.Laps > 1);
    }

    [Fact]
    public void Auto_RingExactlyAsLongAsTheRampStaysLinear()
    {
        // 20mm perimeter, 20mm run: one lap is exactly enough, so no spiral is needed.
        var result = RampPlanner.PlanClosedRing(Square(5), 0, -10, RampSettings.FromRatio(2.0));

        Assert.Equal(RampStrategy.Linear, result.Strategy);
    }

    [Fact]
    public void Auto_ZeroAngleFallsBackToPlunge()
    {
        var result = RampPlanner.PlanClosedRing(Square(100), 0, -10, RampSettings.Plunge);

        Assert.Equal(RampStrategy.Plunge, result.Strategy);
    }

    [Fact]
    public void Helical_LapCountHoldsTheRequestedAngle()
    {
        // 40mm perimeter, 10mm drop, 2:1 ratio needs 20mm of travel: one lap is enough.
        var oneLap = RampPlanner.PlanClosedRing(Square(10), 0, -10, RampSettings.FromRatio(2.0));
        Assert.True(oneLap.Laps <= 1 || oneLap.Strategy == RampStrategy.Linear);

        // A 4mm square has a 16mm perimeter; a 20mm run needs two laps.
        var twoLaps = RampPlanner.PlanClosedRing(Square(4), 0, -10, RampSettings.FromRatio(2.0));
        Assert.Equal(RampStrategy.Helical, twoLaps.Strategy);
        Assert.Equal(2, twoLaps.Laps);
    }

    [Fact]
    public void Helical_LapCountIsCapped()
    {
        var result = RampPlanner.PlanClosedRing(
            Square(1),
            0,
            -100,
            RampSettings.FromRatio(8.0) with { MaxLaps = 3 });

        Assert.Equal(3, result.Laps);
    }

    // --- Depth invariants -----------------------------------------------------

    [Theory]
    [InlineData(100.0)]
    [InlineData(20.0)]
    [InlineData(5.0)]
    public void Ramp_DescendsMonotonicallyAndNeverOvershoots(double size)
    {
        var result = RampPlanner.PlanClosedRing(Square(size), 0, -10, RampSettings.FromRatio(2.0));

        double previous = double.MaxValue;
        var ramp = result.Spans.Single(s => s.Kind == ToolpathSpanKind.Ramp);
        for (int i = ramp.StartIndex; i <= ramp.EndIndex; i++)
        {
            double z = result.Points[i].Z;
            Assert.True(z <= previous + 1e-9, $"ramp rose at index {i}");
            Assert.True(z >= -10 - 1e-9, $"ramp overshot the pass depth at index {i}");
            previous = z;
        }
    }

    [Theory]
    [InlineData(100.0)]
    [InlineData(20.0)]
    [InlineData(5.0)]
    public void Ramp_StartsAtEntryHeightAndReachesFullDepth(double size)
    {
        var result = RampPlanner.PlanClosedRing(Square(size), 0, -10, RampSettings.FromRatio(2.0));

        Assert.Equal(0, result.Points[0].Z, 9);
        Assert.Equal(-10, result.Points[^1].Z, 9);
    }

    [Theory]
    [InlineData(100.0)]
    [InlineData(20.0)]
    [InlineData(5.0)]
    public void CuttingPass_CoversTheWholeRingAtFullDepth(double size)
    {
        // A ramp leaves an uncut wedge behind it, so the pass is only complete when the
        // full perimeter has been travelled at the pass depth.
        var result = RampPlanner.PlanClosedRing(Square(size), 0, -10, RampSettings.FromRatio(2.0));

        double perimeter = size * 4;
        double cutLength = 0;

        foreach (var span in result.Spans.Where(s => s.Kind == ToolpathSpanKind.Cut))
        {
            for (int i = span.StartIndex + 1; i <= span.EndIndex; i++)
            {
                var a = result.Points[i - 1];
                var b = result.Points[i];
                Assert.Equal(-10, b.Z, 6);
                cutLength += Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
            }
        }

        Assert.True(cutLength >= perimeter - 1e-6,
            $"cut only {cutLength:F3}mm of a {perimeter:F3}mm ring");
    }

    [Fact]
    public void LinearRamp_DoesNotCutTheRampedArcTwice()
    {
        var result = RampPlanner.PlanClosedRing(Square(100), 0, -10, RampSettings.FromRatio(2.0));

        double cutLength = 0;
        foreach (var span in result.Spans.Where(s => s.Kind == ToolpathSpanKind.Cut))
        {
            for (int i = span.StartIndex + 1; i <= span.EndIndex; i++)
            {
                var a = result.Points[i - 1];
                var b = result.Points[i];
                cutLength += Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
            }
        }

        // Exactly one lap: the remainder after the ramp plus the ramped arc itself.
        Assert.Equal(400, cutLength, 3);
    }

    [Fact]
    public void LinearRamp_RunLengthMatchesTheRequestedAngle()
    {
        var result = RampPlanner.PlanClosedRing(Square(100), 0, -10, RampSettings.FromRatio(2.0));

        var ramp = result.Spans.Single(s => s.Kind == ToolpathSpanKind.Ramp);
        double rampLength = 0;
        for (int i = ramp.StartIndex + 1; i <= ramp.EndIndex; i++)
        {
            var a = result.Points[i - 1];
            var b = result.Points[i];
            rampLength += Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
        }

        Assert.Equal(20.0, rampLength, 3);
    }

    // --- Guards ---------------------------------------------------------------

    [Fact]
    public void ZeroDrop_EmitsOnlyACuttingLap()
    {
        var ring = Square(100);
        var result = RampPlanner.PlanClosedRing(ring, -10, -10, RampSettings.FromRatio(2.0));

        Assert.DoesNotContain(result.Spans, s => s.Kind == ToolpathSpanKind.Ramp);
        Assert.All(result.Points, p => Assert.Equal(-10, p.Z, 9));
        Assert.Equal(ring[0].x, result.Points[0].X, 9);
        Assert.Equal(ring[0].y, result.Points[0].Y, 9);
        Assert.Equal(result.Points[0], result.Points[^1]);
    }

    [Fact]
    public void DegenerateRing_TerminatesWithoutOutput()
    {
        var degenerate = new List<PointD> { new(5, 5) };

        var result = RampPlanner.PlanClosedRing(degenerate, 0, -10, RampSettings.FromRatio(2.0));

        Assert.Empty(result.Points);
    }

    [Fact]
    public void ZeroLengthRing_DoesNotSpin()
    {
        var collapsed = new List<PointD> { new(5, 5), new(5, 5), new(5, 5) };

        var result = RampPlanner.PlanClosedRing(collapsed, 0, -10, RampSettings.FromRatio(2.0));

        Assert.Equal(RampStrategy.Plunge, result.Strategy);
    }

    [Fact]
    public void ExplicitlyClosedRing_IsNotWalkedTwice()
    {
        var closed = Square(100);
        closed.Add(closed[0]);

        var result = RampPlanner.PlanClosedRing(closed, 0, -10, RampSettings.FromRatio(2.0));

        var ramp = result.Spans.Single(s => s.Kind == ToolpathSpanKind.Ramp);
        double rampLength = 0;
        for (int i = ramp.StartIndex + 1; i <= ramp.EndIndex; i++)
        {
            var a = result.Points[i - 1];
            var b = result.Points[i];
            rampLength += Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
        }

        Assert.Equal(20.0, rampLength, 3);
    }

    // --- Seam selection -------------------------------------------------------

    [Fact]
    public void FindNearestVertex_SmallRingScansExhaustively()
    {
        var ring = Square(100);

        Assert.Equal(2, RampPlanner.FindNearestVertex(ring, new PointD(99, 99)));
        Assert.Equal(0, RampPlanner.FindNearestVertex(ring, new PointD(-5, -5)));
    }

    [Fact]
    public void FindNearestVertex_LargeRingRefinesTheCoarseGuess()
    {
        var circle = new List<PointD>();
        for (int i = 0; i < 512; i++)
        {
            double a = 2 * Math.PI * i / 512;
            circle.Add(new PointD(50 * Math.Cos(a), 50 * Math.Sin(a)));
        }

        int index = RampPlanner.FindNearestVertex(circle, new PointD(50, 0));

        Assert.Equal(0, index);
    }

    [Fact]
    public void RotateTo_MovesTheSeamWithoutChangingTheRing()
    {
        var ring = Square(100);

        var rotated = RampPlanner.RotateTo(ring, 2);

        Assert.Equal(4, rotated.Count);
        Assert.Equal(ring[2], rotated[0]);
        Assert.Equal(ring[3], rotated[1]);
        Assert.Equal(ring[0], rotated[2]);
    }
}
