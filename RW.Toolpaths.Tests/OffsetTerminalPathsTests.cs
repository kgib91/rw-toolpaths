using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class OffsetTerminalPathsTests
{
    private static readonly GeometryTolerances Accuracy = new() { ArcTolerance = .001, SimplifyTolerance = 0 };

    [Theory]
    [InlineData(0, 1, false)]
    [InlineData(33, 1, true)]
    [InlineData(71, .5, false)]
    [InlineData(-122, 2, true)]
    public void TerminalWavefrontSlotIsOneOpenBranchUnderRigidTransformsAndWinding(
        double degrees, double scale, bool reverse)
    {
        double angle = degrees * Math.PI / 180;
        PointD Transform(PointD p) => new(137 + scale * (p.x * Math.Cos(angle) - p.y * Math.Sin(angle)),
            -93 + scale * (p.x * Math.Sin(angle) + p.y * Math.Cos(angle)));
        PointD Original(PointD p) => new(((p.x - 137) * Math.Cos(angle) + (p.y + 93) * Math.Sin(angle)) / scale,
            (-(p.x - 137) * Math.Sin(angle) + (p.y + 93) * Math.Cos(angle)) / scale);
        var outer = Rectangle(0, 0, 16, 60).Select(Transform).ToList();
        if (reverse) outer.Reverse();
        var region = new MillingRegion(outer);
        var closed = Rings(region, 3 * scale, 6 * scale);
        var terminal = Assert.Single(Build(region, closed, 3 * scale, 6 * scale, 3 * scale));
        Assert.Equal(1, terminal.Level);
        Assert.NotEqual(terminal.Points[0], terminal.Points[^1]);
        Assert.All(terminal.Points, point => Assert.InRange(Original(point).x, 8 - .004 / scale, 8 + .004 / scale));
        // The native source ridge runs from y=8 to y=52. Extrapolating tiny
        // graph ends by a cutter radius manufactured ladder rungs elsewhere.
        Assert.InRange(Math.Abs(Original(terminal.Points[0]).y - Original(terminal.Points[^1]).y), 43.99, 44.01);
        AssertClearance(region, terminal.Points, 3 * scale);
    }

    [Theory]
    [InlineData(6, 1)]
    [InlineData(5.9, 0)]
    public void ExactFirstCollapseRetainsWholeTangentAxisAndRejectsUndersizedOpening(double width, int paths)
    {
        var region = new MillingRegion(Rectangle(0, 0, width, 60));
        var closed = Rings(region, 3, 6);
        Assert.Empty(closed);
        var terminal = Build(region, closed, 3, 6, 3);
        Assert.Equal(paths, terminal.Count);
        if (paths == 0) return;
        var axis = Assert.Single(terminal);
        Assert.Equal(0, axis.Level);
        Assert.All(axis.Points, point => Assert.InRange(point.x, 2.999, 3.001));
        Assert.True(Math.Abs(axis.Points[0].y - axis.Points[^1].y) >= 53.99);
        AssertClearance(region, axis.Points, 3);
    }

    [Fact]
    public void TerminalChildBelongsToTheLastCollapsedFrontAndHonorsTheAuthoredRingCap()
    {
        var region = new MillingRegion(Rectangle(0, 0, 16, 80));
        var fill = Fill(region, 3, 6, 3);
        var closed = fill.Where(path => path.IsClosed).ToArray();
        var terminal = Assert.Single(fill.Where(path => !path.IsClosed));
        Assert.Equal(closed.Max(path => path.Level) + 1, terminal.Level);
        Assert.True(terminal.OffsetDistance > closed.Max(path => path.OffsetDistance));
        Assert.All(terminal.Points, point => Assert.InRange(point.x, 7.999, 8.001));
        Assert.NotEqual(terminal.Points[0], terminal.Points[^1]);
        AssertClearance(region, terminal.Points, 3);
        Assert.Empty(Fill(region, 3, 6, 3, ringLimit: 1).Where(path => !path.IsClosed));
    }

    [Fact]
    public void ExactTangentStemSurvivesBesideAWiderRoomWhoseFirstOffsetIsClosed()
    {
        var region = new MillingRegion(new List<PointD>
        {
            new(0, 0), new(6, 0), new(6, 44), new(16, 44), new(16, 70),
            new(-10, 70), new(-10, 44), new(0, 44),
        });
        var closed = Rings(region, 3, 6);
        Assert.NotEmpty(closed);
        Assert.All(closed, ring => Assert.True(ring.Points.Min(point => point.y) >= 43.99,
            "A positive first inset cannot represent the exact-width stem."));
        var terminal = Build(region, closed, 3, 6, 3);
        Assert.Contains(terminal, path => path.Level == 0 && path.Points.Min(point => point.y) <= 3.01
            && path.Points.Max(point => point.y) >= 39);
        var moves = closed.SelectMany(ring => Edges(ring.Points, true))
            .Concat(terminal.SelectMany(path => Edges(path.Points, false))).ToArray();
        foreach (PointD witness in Grid(.5, 5.5, 6, 38))
        {
            double distance = Math.Sqrt(moves.Min(move => DistanceSquared(witness, move.A, move.B)));
            Assert.True(distance <= 3.002,
                $"Mixed terminal tangent stem lost material: witness={witness}, distance={distance:R}.");
        }
        Assert.All(terminal, path => AssertClearance(region, path.Points, 3));
    }

    [Theory]
    [InlineData(.51)]
    [InlineData(.8)]
    [InlineData(.99)]
    public void CollapsedWavefrontKeepsTheReachableCenterAtPartialStepoversAboveHalfDiameter(double stepOver)
    {
        const double radius = 3;
        double pitch = 2 * radius * stepOver;
        // The first centre region's half-width lies strictly between R and
        // pitch: the next requested inset is empty, yet the wall pass misses
        // a positive-width centre band even though stepover is below 100%.
        double width = 3 * radius + pitch;
        var region = new MillingRegion(Rectangle(0, 0, width, 60));
        var closed = Rings(region, radius, pitch);
        Assert.Single(closed);
        var terminal = Assert.Single(Build(region, closed, radius, pitch, radius));
        Assert.Equal(1, terminal.Level);
        AssertClearance(region, terminal.Points, radius);
        var moves = closed.SelectMany(ring => Edges(ring.Points, true))
            .Concat(Edges(terminal.Points, false)).ToArray();
        for (double y = 6.01; y <= 53.99; y += .5)
        foreach (double x in new[] { 6.005, width / 2, width - 6.005 })
        {
            var witness = new PointD(x, y);
            double distance = Math.Sqrt(moves.Min(move => DistanceSquared(witness, move.A, move.B)));
            Assert.True(distance <= radius + .002,
                $"Partial-step terminal core remains: stepover={stepOver:R}, width={width:R}, "
                + $"witness={witness}, cutterDistance={distance:R}.");
        }
    }

    [Fact]
    public void ForkedNativeGraphCutsEveryOriginalChordInOneOpenWalkWithoutBridgeChords()
    {
        var source = new[]
        {
            (new PointD(0, 0), new PointD(10, 0)), (new PointD(10, 0), new PointD(20, 0)),
            (new PointD(10, 0), new PointD(10, 3)), (new PointD(10, 3), new PointD(12, 5)),
        };
        var trail = Assert.Single(OffsetTerminalPaths.JoinTrails(source, CancellationToken.None));
        Assert.NotEqual(trail.Points[0], trail.Points[^1]);
        AssertOnlyOriginalEdges(source, trail.Points);
        // An open walk on this tree must repeat the two short branch chords;
        // no diagonal can substitute for returning through their native node.
        Assert.Equal(6, trail.Points.Count - 1);
    }

    [Fact]
    public void NativeCycleWithOneStemUsesAnOpenWalkInsteadOfReturningToTheOnlyLeaf()
    {
        var source = new[]
        {
            (new PointD(0, 0), new PointD(10, 0)), (new PointD(10, 0), new PointD(10, 10)),
            (new PointD(10, 10), new PointD(0, 10)), (new PointD(0, 10), new PointD(0, 0)),
            (new PointD(0, 0), new PointD(-5, 0)),
        };
        var trail = Assert.Single(OffsetTerminalPaths.JoinTrails(source, CancellationToken.None));
        Assert.NotEqual(trail.Points[0], trail.Points[^1]);
        AssertOnlyOriginalEdges(source, trail.Points);
    }

    [Fact]
    public void NeededPiecesReconnectThroughNativeChordsAndLeaveDeadSpursOut()
    {
        var source = new[]
        {
            (new PointD(0, 0), new PointD(5, 0)), (new PointD(5, 0), new PointD(10, 0)),
            (new PointD(10, 0), new PointD(15, 0)), (new PointD(15, 0), new PointD(20, 0)),
            (new PointD(5, 0), new PointD(5, 4)),
        };
        var connected = OffsetTerminalPaths.RestoreNativeConnectors(source,
            new[] { source[0], source[3] }, CancellationToken.None, out int restored, out int components);
        Assert.Equal(2, restored);
        Assert.Equal(1, components);
        var trail = Assert.Single(OffsetTerminalPaths.JoinTrails(connected, CancellationToken.None));
        AssertOnlyOriginalEdges(source.Take(4), trail.Points);
        Assert.Equal(4, trail.Points.Count - 1);
    }

    [Fact]
    public void SeparateNativeComponentsNeverAcquireASyntheticConnector()
    {
        var source = new[]
        {
            (new PointD(0, 0), new PointD(10, 0)), (new PointD(20, 0), new PointD(30, 0)),
        };
        var connected = OffsetTerminalPaths.RestoreNativeConnectors(source, source,
            CancellationToken.None, out int restored, out int components);
        Assert.Equal(0, restored);
        Assert.Equal(2, components);
        var trails = OffsetTerminalPaths.JoinTrails(connected, CancellationToken.None);
        Assert.Equal(2, trails.Count);
        Assert.All(trails, trail => AssertOnlyOriginalEdges(
            source.Where(edge => edge.Item1 == trail.Points[0] || edge.Item2 == trail.Points[0]), trail.Points));
    }

    [Fact]
    public void TinyRealCenterBandWiderThanTheCompoundSweepErrorStillGetsItsTerminalCut()
    {
        const double radius = 3, gapWidth = .02;
        double comparisonError = 2 * Accuracy.ArcTolerance + 4 / PathUtils.Scale;
        Assert.True(gapWidth > 4 * comparisonError);
        var region = new MillingRegion(Rectangle(0, 0, 4 * radius + gapWidth, 60));
        var closed = Rings(region, radius, 2 * radius);
        Assert.Single(closed);
        var terminal = Assert.Single(Build(region, closed, radius, 2 * radius, radius));
        var ringMoves = closed.SelectMany(ring => Edges(ring.Points, true)).ToArray();
        var terminalMoves = Edges(terminal.Points, false).ToArray();
        for (double y = 8; y <= 52; y += .5)
        {
            var witness = new PointD(2 * radius + gapWidth / 2, y);
            double originalDistance = Math.Sqrt(ringMoves.Min(move => DistanceSquared(witness, move.A, move.B)));
            Assert.True(originalDistance > radius + 4 * comparisonError,
                $"Tiny-band witness was already cut: distance={originalDistance:R}.");
            double actualDistance = Math.Sqrt(terminalMoves.Min(move => DistanceSquared(witness, move.A, move.B)));
            Assert.True(actualDistance <= radius,
                $"A numerical error budget removed a physical terminal band: {witness}, distance={actualDistance:R}.");
        }
        AssertClearance(region, terminal.Points, radius);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void WavefrontBranchesCoverBendsRoomsAndProtectedIslandSides(int fixture)
    {
        var region = fixture switch
        {
            0 => new MillingRegion(new List<PointD>
            {
                new(0, 0), new(16, 0), new(16, 44), new(56, 44), new(56, 60), new(0, 60),
            }),
            1 => new MillingRegion(new List<PointD>
            {
                new(0, 0), new(16, 0), new(16, 44), new(32, 44), new(32, 76),
                new(-16, 76), new(-16, 44), new(0, 44),
            }),
            _ => new MillingRegion(Rectangle(0, 0, 32, 60), new[] { Rectangle(13, 20, 19, 40) }),
        };
        var closed = Rings(region, 3, 6);
        var terminal = Build(region, closed, 3, 6, 3);
        Assert.NotEmpty(closed);
        Assert.All(terminal, path => AssertClearance(region, path.Points, 3));
        var moves = closed.SelectMany(ring => Edges(ring.Points, true))
            .Concat(terminal.SelectMany(path => Edges(path.Points, false))).ToArray();
        IEnumerable<PointD> witnesses = fixture switch
        {
            0 => Grid(6.5, 9.5, 8, 44).Concat(Grid(16, 48, 50.5, 53.5)),
            1 => Grid(6.5, 9.5, 8, 36).Concat(Grid(-7, 23, 58.5, 61.5)),
            _ => Grid(6.25, 6.75, 25, 35).Concat(Grid(25.25, 25.75, 25, 35)),
        };
        foreach (PointD witness in witnesses)
        {
            double distance = Math.Sqrt(moves.Min(move => DistanceSquared(witness, move.A, move.B)));
            Assert.True(distance <= 3.002,
                $"Uncut terminal wavefront: fixture={fixture}, witness={witness}, distance={distance:R}.");
        }
    }

    private static List<OffsetRing> Rings(MillingRegion region, double first, double pitch, int ringLimit = 100)
        => Fill(region, first, pitch, first, ringLimit).Where(path => path.IsClosed).ToList();

    private static List<OffsetRing> Build(MillingRegion region,
        IReadOnlyList<OffsetRing> closed, double first, double pitch, double radius)
        => Fill(region, first, pitch, radius).Where(path => !path.IsClosed).ToList();

    private static List<OffsetRing> Fill(MillingRegion region, double first, double pitch, double radius, int ringLimit = 100)
        => MedialAxisGeometry.BuildConcentricFill(new[] { region }, new RingOffsetOptions(first, pitch)
        {
            RingLimit = ringLimit, Tolerances = Accuracy, MinimumRingArea = 0,
        }, radius, preserveTerminalPaths: true);

    private static void AssertClearance(MillingRegion region, IReadOnlyList<PointD> path, double radius)
    {
        var walls = new[] { region.Outer }.Concat(region.Holes).SelectMany(wall => Edges(wall, true)).ToArray();
        foreach (var chord in Edges(path, false))
        foreach (var wall in walls)
        {
            double actual = Math.Sqrt(SegmentDistanceSquared(chord.A, chord.B, wall.A, wall.B));
            Assert.True(actual >= radius - .0012,
                $"Terminal chord lacks original-wall clearance: {chord.A}->{chord.B}, clearance={actual:R}, required={radius:R}.");
        }
    }

    private static IEnumerable<(PointD A, PointD B)> Edges(IReadOnlyList<PointD> path, bool closed)
    {
        for (int i = 1; i < path.Count; i++) yield return (path[i - 1], path[i]);
        if (closed && path.Count > 1) yield return (path[^1], path[0]);
    }

    private static IEnumerable<PointD> Grid(double minX, double maxX, double minY, double maxY)
    {
        for (double x = minX; x <= maxX + 1e-9; x += .5)
        for (double y = minY; y <= maxY + 1e-9; y += .5) yield return new PointD(x, y);
    }

    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };

    private static double DistanceSquared(PointD p, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y, length = dx * dx + dy * dy;
        double t = length <= 1e-18 ? 0 : Math.Clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length, 0, 1);
        double ex = p.x - a.x - t * dx, ey = p.y - a.y - t * dy;
        return ex * ex + ey * ey;
    }

    private static double SegmentDistanceSquared(PointD a, PointD b, PointD c, PointD d)
    {
        double abx = b.x - a.x, aby = b.y - a.y, cdx = d.x - c.x, cdy = d.y - c.y;
        double denominator = abx * cdy - aby * cdx;
        if (Math.Abs(denominator) > 1e-18)
        {
            double t = ((c.x - a.x) * cdy - (c.y - a.y) * cdx) / denominator;
            double u = ((c.x - a.x) * aby - (c.y - a.y) * abx) / denominator;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return 0;
        }
        return Math.Min(Math.Min(DistanceSquared(a, c, d), DistanceSquared(b, c, d)),
            Math.Min(DistanceSquared(c, a, b), DistanceSquared(d, a, b)));
    }

    private static void AssertOnlyOriginalEdges(IEnumerable<(PointD A, PointD B)> source, IReadOnlyList<PointD> path)
    {
        var originals = source.ToArray();
        var emitted = Edges(path, false).ToArray();
        static bool SameEdge((PointD A, PointD B) first, (PointD A, PointD B) second)
            => (first.A == second.A && first.B == second.B) || (first.A == second.B && first.B == second.A);
        foreach (var chord in originals)
            Assert.Contains(emitted, move => SameEdge(move, chord));
        foreach (var move in emitted)
            Assert.Contains(originals, chord => SameEdge(move, chord));
    }
}
