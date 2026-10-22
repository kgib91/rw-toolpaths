using Clipper2Lib;
using RW.Toolpaths.Milling;
using System.Globalization;
using System.Reflection;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class PocketCommandVertexTests
{
    private readonly ITestOutputHelper _output;
    public PocketCommandVertexTests(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClosedEntriesReplayCompleteOrderedPostedChordsAtEveryDepth(bool repeatedClosure)
    {
        PointD[] cycle = DenseObliqueCycle();
        PointD[] ring = repeatedClosure ? cycle.Concat([cycle[0]]).ToArray() : cycle;
        var original = ring.Select(Xy).ToArray();
        var settings = RampSettings.FromRatio(4);
        var expectedPlanarChords = ClosedPoints(cycle, 0).Select(PostedPoint).ToArray();
        double top = 0;
        foreach (double floor in new[] { -.731, -1.462, -2.193 })
        {
            var messages = new List<string>();
            var entered = RampPlanner.PlanClosedRingAtCommandVertices(ring, top, floor, settings,
                diagnostic: messages.Add);
            AssertRampGradeAndFloor(entered, settings, floor);
            AssertOriginalClosedWalk(cycle, floor, entered);
            var cut = CutSpan(entered);
            Assert.Equal(entered.Points[cut.StartIndex], entered.Points[^1]);
            Assert.Contains(cycle, vertex => Xy(vertex) == Xy(entered.Points[^1]));
            Assert.Equal(original, ring.Select(Xy));
            Assert.Contains(messages, message => message.Contains("sourceVertexContact=True"));

            // Replay the whole entry chronologically before selecting the cut.
            // Resetting a cursor at the cut would conceal omitted-axis mistakes.
            Point3D[] postedCut = PostedCut(entered);
            Point3D[] postedSource = ClosedPoints(cycle, floor).Select(PostedPoint).ToArray();
            AssertOrderedClosedChords(postedSource, postedCut);
            Assert.Equal(Edges(expectedPlanarChords), Edges(postedSource.Select(p => p with { Z = 0 }).ToArray()));
            _output.WriteLine($"closed={repeatedClosure} top={top:R} floor={floor:R} "
                + $"sourceVertices={cycle.Length} rampEnd={CutSpan(entered).StartIndex} "
                + $"cutChords={postedCut.Length - 1} finalExit={postedCut[^1]}");
            top = floor;
        }
    }

    [Fact]
    public void LegacyInteriorContactSplitsPostedChordsAndAddsPhysicalStockOutsideTheOriginalChord()
    {
        PointD[] cycle = ObliqueCorners();
        // This contact lies well inside the first oblique edge and rounds to
        // its outward side by over half a micron; adjacent edges cannot reach
        // the independent added-volume witness below.
        var settings = RampSettings.FromRatio(2.076);
        var legacy = RampPlanner.PlanClosedRing(cycle, 0, -1, settings);
        var entered = RampPlanner.PlanClosedRingAtCommandVertices(cycle, 0, -1, settings);
        Point3D[] source = ClosedPoints(cycle, -1).Select(PostedPoint).ToArray();
        Point3D[] oldCut = PostedCut(legacy), newCut = PostedCut(entered);
        var originalChord = (A: source[0], B: source[1]);
        Assert.DoesNotContain(Edges(oldCut), chord => chord == originalChord);
        Assert.Contains(originalChord, Edges(newCut));
        Point3D nativeContact = legacy.Points[CutSpan(legacy).StartIndex];
        Assert.DoesNotContain(cycle, vertex => Xy(vertex) == Xy(nativeContact));
        AssertOriginalClosedWalk(cycle, -1, entered);

        double dx = source[1].X - source[0].X, dy = source[1].Y - source[0].Y;
        double length = Math.Sqrt(dx * dx + dy * dy);
        Point3D postedContact = oldCut[0];
        double offset = Math.Sqrt(PointSegmentDistanceSquared(postedContact, source[0], source[1]));
        Assert.True(offset > .0005, $"Negative control is inconclusive: posted contact drift={offset:R}.");
        double side = Math.Sign(dx * (postedContact.Y - source[0].Y)
            - dy * (postedContact.X - source[0].X));
        Assert.NotEqual(0, side);
        const double radius = .4;
        foreach (bool ball in new[] { false, true })
        {
            double witnessZ = ball ? -.8 : -.9;
            double vertical = ball ? witnessZ - (-1 + radius) : 0;
            double reach = Math.Sqrt(radius * radius - vertical * vertical) - .0003;
            var witness = new Point3D(postedContact.X - side * dy / length * reach,
                postedContact.Y + side * dx / length * reach, witnessZ);
            Assert.True(ContainsFloorStock(oldCut, witness, radius, ball),
                $"Legacy split misses its added-volume witness {witness}; ball={ball}.");
            Assert.False(ContainsFloorStock(newCut, witness, radius, ball),
                $"Whole original chord removes the legacy added-volume witness {witness}; ball={ball}.");
            _output.WriteLine($"legacyContact={nativeContact} postedContact={postedContact} "
                + $"postedDrift={offset:R} addedStock={witness} ball={ball}");
        }

        // The public generic entry keeps its existing interior-contact default.
        var publicEntry = RampPlanner.PlanPathEntry(ClosedPoints(cycle, -1), 0, settings);
        Assert.DoesNotContain(cycle, vertex => Xy(vertex) == Xy(publicEntry.Points[CutSpan(publicEntry).StartIndex]));
    }

    [Fact]
    public void OpenEntryLandsAtReachableShallowerOriginalVertexAndReplaysTheWholeForwardTrace()
    {
        Point3D[] source = [new(.0005, .0015, -1.1), new(1.3015, .5015, -.8),
            new(2.2135, 1.7075, -.4), new(3.0315, 1.8995, -.9)];
        Point3D[] original = source.ToArray();
        var settings = RampSettings.FromRatio(4);
        var messages = new List<string>();
        var entered = RampPlanner.PlanPathEntry(source, 0, settings,
            diagnostic: messages.Add, landAtSourceVertex: true);
        var ramp = entered.Spans.Single(span => span.Kind == ToolpathSpanKind.Ramp);
        Assert.Equal(source[2], entered.Points[ramp.EndIndex]);
        Assert.True(entered.Points[ramp.EndIndex].Z > source.Min(p => p.Z));
        for (int index = 1; index <= ramp.EndIndex; index++)
        {
            Point3D a = entered.Points[index - 1], b = entered.Points[index];
            int sourceIndex = Array.FindIndex(source, p => Xy(p) == Xy(a));
            Assert.InRange(sourceIndex, 0, source.Length - 2);
            Assert.Equal(Xy(source[sourceIndex + 1]), Xy(b));
            Assert.True(a.Z >= source[sourceIndex].Z - 1e-10);
            Assert.True(b.Z >= source[sourceIndex + 1].Z - 1e-10);
            Assert.InRange(a.Z - b.Z, -1e-10, PlanarDistance(a, b) * Math.Tan(settings.AngleRadians) + 1e-10);
        }
        Point3D[] cut = entered.Points.Skip(CutSpan(entered).StartIndex).ToArray();
        Assert.True(ContainsForwardTrace(cut, source), "The complete directed authored XYZ trace is absent.");
        Assert.True(ContainsForwardTrace(PostedCut(entered), source.Select(PostedPoint).ToArray()),
            "Chronological F3 replay loses a complete directed authored cutting chord.");
        Assert.DoesNotContain(Edges(cut), chord => chord.A == source[^1] && chord.B == source[0]);
        Assert.Equal(source[^1], entered.Points[^1]);
        Assert.Equal(original, source);
        Assert.Contains(messages, message => message.Contains("sourceVertexContact=True")
            && message.Contains("originalCutTracePreserved=True"));
        _output.WriteLine($"openContact={entered.Points[ramp.EndIndex]} finalExit={entered.Points[^1]} "
            + $"rampChords={ramp.MoveCount} cutChords={cut.Length - 1}");
    }

    [Fact]
    public void AHelicalLapCapCannotSteepenPocketEntryAboveTheRequestedGrade()
    {
        PointD[] cycle = ObliqueCorners().Select(p => new PointD(p.x / 10, p.y / 10)).ToArray();
        var settings = RampSettings.FromRatio(4) with { Strategy = RampStrategy.Helical, MaxLaps = 1 };
        var entered = RampPlanner.PlanClosedRingAtCommandVertices(cycle, 0, -1, settings);
        Assert.True(entered.Laps > settings.MaxLaps, $"Insufficient descent laps: actual={entered.Laps}.");
        AssertRampGradeAndFloor(entered, settings, -1);
        AssertOriginalClosedWalk(cycle, -1, entered);
        AssertOrderedClosedChords(ClosedPoints(cycle, -1).Select(PostedPoint).ToArray(), PostedCut(entered));
        var legacy = RampPlanner.PlanClosedRing(cycle, 0, -1, settings);
        var ramp = legacy.Spans.Single(span => span.Kind == ToolpathSpanKind.Ramp);
        Assert.Contains(Edges(legacy.Points.Take(ramp.PointCount).ToArray()), chord =>
            chord.A.Z - chord.B.Z > PlanarDistance(chord.A, chord.B) * Math.Tan(settings.AngleRadians) + 1e-10);
        _output.WriteLine($"requestedSlope={Math.Tan(settings.AngleRadians):R} "
            + $"legacyCap={settings.MaxLaps} commandVertexLaps={entered.Laps}");
    }

    [Fact]
    public void NearestSafeRebaseUsesAnOriginalVertexAndLeavesTheDefaultProjectionPolicyAvailable()
    {
        PointD[] source = ObliqueCorners();
        var original = source.Select(Xy).ToArray();
        var from = new Point3D(1.9, -.5, -1);
        var clearance = new RecordingClearance(_ => true);
        Assert.True(PocketToolpaths.TryFindSafeRebase(source, from, -1, clearance, .4,
            out var rebased, out int tested, commandVerticesOnly: true));
        Assert.Equal(1, tested);
        Assert.Equal(tested, clearance.Destinations.Count);
        AssertOriginalRebase(source, rebased);
        Assert.All(clearance.Destinations, to => Assert.Contains(source, p => Xy(p) == Xy(to)));
        Assert.Equal(original, source.Select(Xy));

        Assert.True(PocketToolpaths.TryFindSafeRebase(source, from, -1, LinkClearance.Unrestricted, .4,
            out var legacyProjection, out _));
        Assert.DoesNotContain(source, p => Xy(p) == Xy(legacyProjection[0]));
        Assert.Equal(source.Length + 1, legacyProjection.Count);
        _output.WriteLine($"from={from} originalVertex={Xy(rebased[0])} "
            + $"legacyProjection={Xy(legacyProjection[0])} tested={tested}");
    }

    [Fact]
    public void ARefusedNearestVertexRetriesOnlyOtherOriginalVertices()
    {
        PointD[] source = DenseObliqueCycle();
        var target = source[source.Length / 2];
        var clearance = new RecordingClearance(to => Xy(to) == Xy(target));
        Assert.True(PocketToolpaths.TryFindSafeRebase(source, new(.2, -.5, -2), -2, clearance, .4,
            out var rebased, out int tested, commandVerticesOnly: true));
        Assert.True(tested > 1, $"Alternative-vertex fixture tested only {tested} candidate(s).");
        Assert.Equal(tested, clearance.Destinations.Count);
        Assert.Equal(Xy(target), Xy(rebased[0]));
        AssertOriginalRebase(source, rebased);
        Assert.All(clearance.Destinations, to =>
        {
            Assert.Equal(-2, to.Z);
            Assert.Contains(source, vertex => Xy(vertex) == Xy(to));
        });
        _output.WriteLine($"alternativeVertex={Xy(rebased[0])} candidatesTested={tested} sourceVertices={source.Length}");
    }

    [Fact]
    public void ACallerRejectingEveryOriginalVertexCannotAuthorizeASyntheticRebase()
    {
        PointD[] source = ObliqueCorners();
        var clearance = new RecordingClearance(_ => false);
        Assert.False(PocketToolpaths.TryFindSafeRebase(source, new(1.9, -.5, -1), -1, clearance, .4,
            out var unchanged, out int tested, commandVerticesOnly: true));
        Assert.Equal(source.Length, tested);
        Assert.Equal(source.Length, clearance.Destinations.Select(Xy).Distinct().Count());
        Assert.Equal(source.Select(Xy), unchanged.Select(Xy));
        Assert.All(clearance.Destinations, to => Assert.Contains(source, p => Xy(p) == Xy(to)));
        _output.WriteLine($"callerRefusal=True candidatesTested={tested} originalVertices={source.Length}");
    }

    [Fact]
    public void CancelledCommandVertexEntryProducesNoCompletedPathOrSourceMutation()
    {
        PointD[] source = DenseObliqueCycle();
        var original = source.Select(Xy).ToArray();
        var messages = new List<string>();
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => RampPlanner.PlanClosedRingAtCommandVertices(
            source, 0, -1, RampSettings.FromRatio(4), cancellation.Token, messages.Add));
        Assert.Empty(messages);
        Assert.Equal(original, source.Select(Xy));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RepairedEntryKeepsExactlyOneFloorCycleAndReturnsToADependentSeamOnlyAsExecutedLinks(bool dependentTail)
    {
        PointD[] cycle = ObliqueCorners();
        var native = ClosedPoints(cycle, -1).ToList();
        var spans = new List<ToolpathSpan> { new(0, native.Count - 1, ToolpathSpanKind.Cut) };
        Point3D outgoingStart = native[^1], outgoingEnd = new(-3, 3, -1), laterEnd = new(-4, 3, -1);
        if (dependentTail)
        {
            int seam = native.Count - 1;
            native.Add(outgoingEnd); native.Add(laterEnd);
            spans.Add(new(seam, seam + 1, ToolpathSpanKind.Link));
            spans.Add(new(seam + 1, seam + 2, ToolpathSpanKind.Cut));
        }
        var source = new TaggedToolpath(native, spans, 0, "pocket", 0);
        var method = typeof(PocketContinuationRouter).GetMethod("AddEntryAtCommandVertices", BindingFlags.NonPublic | BindingFlags.Static)!;
        object?[] arguments = [source, 0d, RampSettings.FromRatio(2.076), 11, CancellationToken.None, null];
        var entered = (TaggedToolpath)method.Invoke(null, arguments)!;
        ToolpathSpan cut = entered.Spans.First(span => span.Kind == ToolpathSpanKind.Cut);
        var floorCycle = entered.Points.Skip(cut.StartIndex).Take(cut.PointCount).ToArray();
        AssertOrderedClosedChords(ClosedPoints(cycle, -1), floorCycle);
        Assert.NotEqual(outgoingStart, floorCycle[0]);
        if (!dependentTail)
        {
            Assert.Equal(floorCycle[0], entered.Points[^1]);
            Assert.DoesNotContain(entered.Spans, span => span.Kind == ToolpathSpanKind.Link);
        }
        else
        {
            var lastCut = entered.Spans.Last(span => span.Kind == ToolpathSpanKind.Cut);
            Assert.Equal(outgoingEnd, entered.Points[lastCut.StartIndex]);
            Assert.Equal(laterEnd, entered.Points[lastCut.EndIndex]);
            var links = entered.Spans.Where(span => span.Kind == ToolpathSpanKind.Link)
                .SelectMany(span => Edges(entered.Points.Skip(span.StartIndex).Take(span.PointCount).ToArray())).ToArray();
            Assert.Equal((outgoingStart, outgoingEnd), links[^1]);
            var originalEdges = Edges(ClosedPoints(cycle, -1)).ToArray();
            Assert.All(links.Take(links.Length - 1), chord => Assert.True(originalEdges.Contains(chord)
                || originalEdges.Contains((chord.B, chord.A)), $"Seam return invented a command chord: {chord}."));
        }
        Assert.Equal(native, source.Points);
        _output.WriteLine($"dependentTail={dependentTail} contact={floorCycle[0]} cutMoves={cut.MoveCount} "
            + $"sourceMoves={cycle.Length} finalExit={entered.Points[^1]}");
    }

    private static PointD[] ObliqueCorners()
        => [new(.0005, .0015), new(4.0135, 1.2175), new(3.3815, 4.0395), new(-.8125, 2.7055)];

    private static PointD[] DenseObliqueCycle()
    {
        PointD[] corners = ObliqueCorners();
        var vertices = new List<PointD>();
        const int subdivisions = 37;
        for (int edge = 0; edge < corners.Length; edge++)
        for (int step = 0; step < subdivisions; step++)
        {
            PointD a = corners[edge], b = corners[(edge + 1) % corners.Length];
            double t = step / (double)subdivisions;
            vertices.Add(new PointD(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
        }
        return vertices.ToArray();
    }

    private static Point3D[] ClosedPoints(IReadOnlyList<PointD> cycle, double floor)
        => cycle.Select(p => new Point3D(p.x, p.y, floor))
            .Append(new Point3D(cycle[0].x, cycle[0].y, floor)).ToArray();

    private static ToolpathSpan CutSpan(RampedPath entered)
        => Assert.Single(entered.Spans.Where(span => span.Kind == ToolpathSpanKind.Cut));

    private static void AssertRampGradeAndFloor(RampedPath entered, RampSettings settings, double floor)
    {
        var ramp = Assert.Single(entered.Spans.Where(span => span.Kind == ToolpathSpanKind.Ramp));
        for (int index = ramp.StartIndex + 1; index <= ramp.EndIndex; index++)
        {
            Point3D a = entered.Points[index - 1], b = entered.Points[index];
            double run = PlanarDistance(a, b), drop = a.Z - b.Z;
            Assert.InRange(drop, -1e-10, run * Math.Tan(settings.AngleRadians) + 1e-10);
            Assert.True(b.Z >= floor - 1e-10, $"Ramp chord {index} goes below floor: {a} -> {b}; floor={floor:R}.");
        }
        Assert.Equal(floor, entered.Points[ramp.EndIndex].Z);
    }

    private static void AssertOriginalClosedWalk(IReadOnlyList<PointD> source, double floor, RampedPath entered)
    {
        Assert.All(entered.Points, point => Assert.Contains(source, vertex => Xy(vertex) == Xy(point)));
        var cut = CutSpan(entered);
        Point3D[] points = entered.Points.Skip(cut.StartIndex).Take(cut.PointCount).ToArray();
        Assert.All(points, point => Assert.Equal(floor, point.Z));
        AssertOrderedClosedChords(ClosedPoints(source, floor), points);
    }

    private static void AssertOrderedClosedChords(IReadOnlyList<Point3D> source, IReadOnlyList<Point3D> cut)
    {
        int count = source.Count - 1;
        int phase = Enumerable.Range(0, count).FirstOrDefault(index => source[index] == cut[0], -1);
        Assert.True(phase >= 0, $"Cut starts at a non-source vertex: {cut[0]}.");
        Assert.Equal(count, cut.Count - 1);
        Assert.Equal(cut[0], cut[^1]);
        for (int index = 1; index < cut.Count; index++)
        {
            var expected = (A: source[(phase + index - 1) % count], B: source[(phase + index) % count]);
            var actual = (A: cut[index - 1], B: cut[index]);
            Assert.True(expected == actual, $"Directed chord {index} changed: expected={expected}, actual={actual}.");
        }
    }

    private static void AssertOriginalRebase(IReadOnlyList<PointD> source, IReadOnlyList<PointD> rebased)
    {
        Assert.Equal(source.Count, rebased.Count);
        int phase = Enumerable.Range(0, source.Count).First(index => Xy(source[index]) == Xy(rebased[0]));
        for (int index = 0; index < source.Count; index++)
            Assert.Equal(Xy(source[(phase + index) % source.Count]), Xy(rebased[index]));
        AssertOrderedClosedChords(ClosedPoints(source, -1).Select(PostedPoint).ToArray(),
            ClosedPoints(rebased, -1).Select(PostedPoint).ToArray());
    }

    private static bool ContainsForwardTrace(IReadOnlyList<Point3D> actual, IReadOnlyList<Point3D> source)
        => Enumerable.Range(0, Math.Max(0, actual.Count - source.Count + 1))
            .Any(start => Enumerable.Range(0, source.Count).All(index => actual[start + index] == source[index]));

    private static Point3D[] PostedCut(RampedPath entered)
    {
        var posted = new List<Point3D> { PostedPoint(entered.Points[0]) };
        for (int index = 1; index < entered.Points.Count; index++)
        {
            Point3D a = entered.Points[index - 1], b = entered.Points[index];
            float dx = (float)b.X - (float)a.X, dy = (float)b.Y - (float)a.Y;
            bool verticalOnly = MathF.Abs((float)b.Z - (float)a.Z) > .001f
                && MathF.Sqrt(dx * dx + dy * dy) <= .001f;
            posted.Add(verticalOnly ? posted[^1] with { Z = PostedCoordinate(b.Z) } : PostedPoint(b));
        }
        var cut = CutSpan(entered);
        return posted.Skip(cut.StartIndex).Take(cut.PointCount).ToArray();
    }

    private static Point3D PostedPoint(Point3D point)
        => new(PostedCoordinate(point.X), PostedCoordinate(point.Y), PostedCoordinate(point.Z));

    private static double PostedCoordinate(double value)
        => double.Parse(((float)value).ToString("F3", CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);

    private static (double X, double Y) Xy(PointD point) => (point.x, point.y);
    private static (double X, double Y) Xy(Point3D point) => (point.X, point.Y);

    private static IEnumerable<(Point3D A, Point3D B)> Edges(IReadOnlyList<Point3D> points)
    {
        for (int index = 1; index < points.Count; index++) yield return (points[index - 1], points[index]);
    }

    private static double PlanarDistance(Point3D a, Point3D b)
        => Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));

    private static double PointSegmentDistanceSquared(Point3D point, Point3D a, Point3D b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
        double lengthSquared = dx * dx + dy * dy + dz * dz;
        double t = lengthSquared == 0 ? 0 : Math.Clamp(((point.X - a.X) * dx
            + (point.Y - a.Y) * dy + (point.Z - a.Z) * dz) / lengthSquared, 0, 1);
        double qx = point.X - a.X - t * dx, qy = point.Y - a.Y - t * dy, qz = point.Z - a.Z - t * dz;
        return qx * qx + qy * qy + qz * qz;
    }

    private static bool ContainsFloorStock(IReadOnlyList<Point3D> points, Point3D witness, double radius, bool ball)
        => Edges(points).Any(chord => ball
            ? PointSegmentDistanceSquared(witness, chord.A with { Z = chord.A.Z + radius },
                chord.B with { Z = chord.B.Z + radius }) < radius * radius
            : witness.Z >= chord.A.Z && PointSegmentDistanceSquared(witness with { Z = chord.A.Z },
                chord.A, chord.B) < radius * radius);

    private sealed class RecordingClearance(Func<Point3D, bool> accept) : ILinkClearance
    {
        public List<Point3D> Destinations { get; } = new();
        public bool IsTravelSafe(Point3D from, Point3D to, double radius)
        {
            Destinations.Add(to);
            return accept(to);
        }
    }
}
