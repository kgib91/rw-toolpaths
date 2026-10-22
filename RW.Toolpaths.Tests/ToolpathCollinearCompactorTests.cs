using System.Globalization;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ToolpathCollinearCompactorTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SubdivisionRemovalPreservesContinuousRemovedDepthForConeAndBall(bool ball)
    {
        const double surface = 12.699999809265137;
        var points = Enumerable.Range(0, 101)
            .Select(i => new Point3D(i * .02, .137, surface - .4 - i * .002)).ToArray();
        var original = Path(points);
        var compacted = ToolpathCollinearCompactor.Compact(original, surface);
        Assert.True(compacted.Points.Count < original.Points.Count / 2);
        double maximumDifference = 0;
        int contacts = 0;
        for (int x = -5; x <= 45; x++)
        for (int y = -16; y <= 16; y++)
        {
            double qx = x * .05, qy = .137 + y * .05;
            double before = RemovedDepth(original.Points, surface, qx, qy, ball);
            double after = RemovedDepth(compacted.Points, surface, qx, qy, ball);
            maximumDifference = Math.Max(maximumDifference, Math.Abs(before - after));
            Assert.Equal(before, after, 10);
            if (before > 0) contacts++;
        }
        Assert.True(contacts > 0);
        Console.WriteLine($"[Toolpath.StockDepthOracle] profile={(ball ? "ball" : "cone")} " +
            $"beforePoints={original.Points.Count} afterPoints={compacted.Points.Count} " +
            $"continuousContacts={contacts} maximumRemovedDepthDifference={maximumDifference:R}");
    }

    [Fact]
    public void DepthOracleRejectsShallowCutWithSameSurfaceCenterline()
    {
        var deep = new[] { new Point3D(0, 0, -1), new Point3D(2, 0, -1) };
        var shallow = new[] { new Point3D(0, 0, -.25), new Point3D(2, 0, -.25) };
        Assert.Equal(1, RemovedDepth(deep, 0, 1, 0, false));
        Assert.Equal(.25, RemovedDepth(shallow, 0, 1, 0, false));
    }

    [Fact]
    public void NativeLineThatRoundsIntoCornerKeepsItsStockDefiningMiddle()
    {
        var original = Path([new(.0001, .0001, -1), new(.0001, .0001, -1), new(.0007, .0004, -1), new(.0013, .0007, -1)]);
        var compacted = ToolpathCollinearCompactor.Compact(original, 0);
        Assert.Same(original.Points, compacted.Points);
        // Removing the native-collinear point changes the commanded stock depth.
        Assert.True(RemovedDepth(original.Points, 0, .001, 0, false)
            > RemovedDepth([original.Points[0], original.Points[^1]], 0, .001, 0, false) + .001);
    }

    [Fact]
    public void NativeBendBacktrackAndClosedLoopKeepTheirRemovalGeometry()
    {
        foreach (var points in new Point3D[][]
        {
            [new(0, 0, -1), new(1, .0000001, -1), new(2, 0, -1)],
            [new(0, 0, -1), new(2, 0, -1), new(1, 0, -1)],
            [new(0, 0, -1), new(1, 0, -1), new(0, 0, -1)],
        })
        {
            var original = Path(new[] { points[0] }.Concat(points).ToArray());
            Assert.Same(original.Points, ToolpathCollinearCompactor.Compact(original, 0).Points);
        }
    }

    [Fact]
    public void AggregationCannotChangeVerticalAxisOnlyOrAirFeedClassification()
    {
        foreach (var points in new Point3D[][]
        {
            // Each small chord is XYZ; the aggregate would become Y-only.
            [new(0, 0, -1), new(0, 0, -1.0008), new(0, 0, -1.0016)],
            // Rapid approach above stock then plunge into it must remain separate.
            [new(0, 0, .002), new(0, 0, .001), new(0, 0, -.001)],
            // Two Y-only steep chords would become an XYZ chord after merging.
            [new(0, 0, -.1), new(.00075, 0, -.103), new(.0015, 0, -.106)],
        })
        {
            var original = Path(new[] { points[0] }.Concat(points).ToArray());
            Assert.Same(original.Points, ToolpathCollinearCompactor.Compact(original, 0).Points);
        }
    }

    [Fact]
    public void FeedAndCuttingAuthorityBoundariesAndVisitIdentitySurvive()
    {
        var points = Enumerable.Range(0, 7).Select(i => new Point3D(i, 0, -Math.Min(i, 3) * .1)).ToArray();
        var original = new TaggedToolpath(points,
            [new(0, 3, ToolpathSpanKind.Ramp), new(3, 6, ToolpathSpanKind.Cut)],
            4, "clearing", 2, 91, detectCurves: false);
        var compacted = ToolpathCollinearCompactor.Compact(original, 0);
        Assert.Equal(new[] { points[0], points[1], points[3], points[6] }, compacted.Points);
        Assert.Equal(new ToolpathSpan[] { new(0, 2, ToolpathSpanKind.Ramp), new(2, 3, ToolpathSpanKind.Cut) }, compacted.Spans);
        Assert.Equal(original.RegionIndex, compacted.RegionIndex);
        Assert.Equal(original.DepthPassIndex, compacted.DepthPassIndex);
        Assert.Equal(original.VisitIdentity, compacted.VisitIdentity);
    }

    [Fact]
    public void CurveConsumersUncertainCoordinatesAndCancellationKeepTheirContracts()
    {
        var curve = new TaggedToolpath([new(0, 0, -1), new(1, 0, -1), new(2, 0, -1)],
            Array.Empty<ToolpathSpan>(), 0, "clearing", 0, detectCurves: true);
        Assert.Same(curve.Points, ToolpathCollinearCompactor.Compact(curve, 0).Points);
        foreach (double coordinate in new[] { double.NaN, double.PositiveInfinity, 1e12 })
        {
            var uncertain = Path([new(coordinate, 0, -1), new(coordinate, 0, -1), new(coordinate, 1, -1), new(coordinate, 2, -1)]);
            Assert.Same(uncertain.Points, ToolpathCollinearCompactor.Compact(uncertain, 0).Points);
        }
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => ToolpathCollinearCompactor.Compact(curve, 0, cancellation.Token));
    }

    [Fact]
    public void FirstExplicitPlanarEndpointAnchorsAnUnknownIncomingCursor()
    {
        var original = Path([new(0, 0, -1), new(1, 0, -1), new(2, 0, -1), new(3, 0, -1)]);
        var compacted = ToolpathCollinearCompactor.Compact(original, 0);
        Assert.Equal(new[] { original.Points[0], original.Points[1], original.Points[3] }, compacted.Points);
        var incoming = new Point3D(0, 1, -1);
        var before = PostedTrace(original.Points, incoming);
        var after = PostedTrace(compacted.Points, incoming);
        var unsafeTrace = PostedTrace([original.Points[0], original.Points[3]], incoming);
        Assert.Equal(1, RemovedDepth(before, 0, 1, 0, false));
        Assert.True(RemovedDepth(unsafeTrace, 0, 1, 0, false) < .9);
        Assert.Equal(RemovedDepth(before, 0, 1, 0, false), RemovedDepth(after, 0, 1, 0, false));
    }

    [Fact]
    public void PlanarEndpointAfterVerticalOnlyDriftMustBeRetained()
    {
        var original = Path([new(0, 0, -.98), new(.0008, 0, -1),
            new(1.0008, 0, -1.5), new(2.0008, 0, -2), new(3.0008, 0, -2.5)]);
        var compacted = ToolpathCollinearCompactor.Compact(original, 0);
        Assert.Equal(new[] { original.Points[0], original.Points[1], original.Points[2], original.Points[4] }, compacted.Points);
        var before = PostedTrace(original.Points, original.Points[0]);
        var after = PostedTrace(compacted.Points, original.Points[0]);
        var unsafeTrace = PostedTrace([original.Points[0], original.Points[1], original.Points[3], original.Points[4]], original.Points[0]);
        Assert.True(RemovedDepth(unsafeTrace, 0, 1.001, 0, false) > RemovedDepth(before, 0, 1.001, 0, false) + .0001);
        for (int sample = 0; sample <= 60; sample++)
            Assert.Equal(RemovedDepth(before, 0, sample * .05, 0, false), RemovedDepth(after, 0, sample * .05, 0, false), 10);
    }

    private static Point3D[] PostedTrace(IReadOnlyList<Point3D> points, Point3D initial)
    {
        // Independent posted cursor: a vertical-only command supplies no planar
        // coordinates, even when the nominal endpoint rounded to a different XY.
        var trace = new List<Point3D> { new(F3((float)initial.X), F3((float)initial.Y), F3((float)initial.Z)) };
        for (int i = 1; i < points.Count; i++)
        {
            var a = points[i - 1]; var b = points[i];
            float dx = (float)b.X - (float)a.X, dy = (float)b.Y - (float)a.Y;
            float dz = (float)b.Z - (float)a.Z;
            bool verticalOnly = MathF.Abs(dz) > .001f && MathF.Sqrt(dx * dx + dy * dy) <= .001f;
            trace.Add(verticalOnly ? new(trace[^1].X, trace[^1].Y, F3((float)b.Z))
                : new(F3((float)b.X), F3((float)b.Y), F3((float)b.Z)));
        }
        return trace.ToArray();
    }

    [Fact]
    public void LineMetadataDoesNotAllocateOneLargeRecordPerPoint()
    {
        var points = Enumerable.Range(0, 100_001).Select(i => new Point3D(i, 0, -1)).ToArray();
        _ = Path([new(0, 0, 0), new(1, 0, 0)]); // JIT/constructor warmup.
        long before = GC.GetAllocatedBytesForCurrentThread();
        var path = Path(points);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.True(allocated < 1024, $"Line-only metadata allocated {allocated} bytes for {points.Length} owned points.");
        Assert.Equal(100_000, path.Moves.Count);
        Assert.Equal(ToolMove.Line(points[17], points[18]), path.Moves[17]);
        Assert.Equal(ToolMove.Line(points[^2], points[^1]), path.Moves.Last());
        Assert.Throws<ArgumentOutOfRangeException>(() => path.Moves[-1]);
        Assert.Throws<ArgumentOutOfRangeException>(() => path.Moves[path.Moves.Count]);
    }

    private static TaggedToolpath Path(IReadOnlyList<Point3D> points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "clearing", 0, detectCurves: false);

    private static double F3(float value)
        => double.Parse(value.ToString("F3", CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);

    // Independent continuous cutter/stock-height oracle, rather than a 2D
    // centerline or footprint comparison. A cutter's lower envelope determines
    // the deepest removed stock at each query. Cone and sphere extrema are
    // analytic over the complete chord, including its variable tip height.
    private static double RemovedDepth(IReadOnlyList<Point3D> points, double surface,
        double qx, double qy, bool ball)
    {
        double removed = 0;
        for (int i = 1; i < points.Count; i++)
        {
            var a = points[i - 1]; var b = points[i];
            double ax = F3((float)a.X), ay = F3((float)a.Y), az = F3((float)a.Z - (float)surface);
            double bx = F3((float)b.X), by = F3((float)b.Y), bz = F3((float)b.Z - (float)surface);
            double dx = bx - ax, dy = by - ay, length = Math.Sqrt(dx * dx + dy * dy);
            double ux = length > 0 ? dx / length : 1, uy = length > 0 ? dy / length : 0;
            double along = (qx - ax) * ux + (qy - ay) * uy;
            double perpendicular = Math.Abs((qx - ax) * uy - (qy - ay) * ux);
            double slope = length > 0 ? (bz - az) / length : 0;
            double position;
            if (ball)
            {
                const double radius = 1;
                double square = radius * radius - perpendicular * perpendicular;
                if (square < 0) continue;
                double reach = Math.Sqrt(square);
                double first = Math.Max(0, along - reach), last = Math.Min(length, along + reach);
                if (first > last) continue;
                position = Math.Clamp(along - slope * Math.Sqrt(square / (1 + slope * slope)), first, last);
                double radialSquare = perpendicular * perpendicular + (position - along) * (position - along);
                double tip = length > 0 ? az + slope * position : Math.Min(az, bz);
                removed = Math.Max(removed, -tip - radius + Math.Sqrt(Math.Max(0, radius * radius - radialSquare)));
            }
            else
            {
                const double heightPerRadius = 2;
                position = Math.Abs(slope) < heightPerRadius
                    ? Math.Clamp(along - slope * perpendicular / Math.Sqrt(heightPerRadius * heightPerRadius - slope * slope), 0, length)
                    : slope > 0 ? 0 : length;
                double radial = Math.Sqrt(perpendicular * perpendicular + (position - along) * (position - along));
                double tip = length > 0 ? az + slope * position : Math.Min(az, bz);
                removed = Math.Max(removed, -tip - heightPerRadius * radial);
            }
        }
        return removed;
    }
}
