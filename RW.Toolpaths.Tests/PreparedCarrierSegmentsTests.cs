using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using System.Reflection;

namespace RW.Toolpaths.Tests;

public sealed class PreparedCarrierSegmentsTests
{
    [Fact]
    public void ATrailingNonfiniteChordRetainsTheExactFormerRouteFailure()
    {
        var path = Path(new (Point3D A, Point3D B)[]
        { (new(-10, 0, -1), new(10, 0, -1)), (new(0, 10, -1), new(double.NaN, 10, -1)) });
        var index = new PreparedCarrierSegments(path);
        var diagnostics = new PreparedCarrierSegments.Diagnostics();
        var method = typeof(PocketToolpaths).GetMethod("TryFindRoutePause", BindingFlags.NonPublic | BindingFlags.Static)!;
        object?[] arguments = [path, index, new Point3D(-2, 1, 0), new Point3D(2, 1, 0),
            double.MaxValue, diagnostics, null, false];
        Assert.False((bool)method.Invoke(null, arguments)!);
        Assert.Equal(1, diagnostics.FallbackQueries);
        arguments[1] = null;
        Assert.False((bool)method.Invoke(null, arguments)!);
    }

    [Fact]
    public void IndexedExcursionPreservesTheExistingMinimizerAndOriginalTies()
    {
        var random = new Random(55267);
        var chords = new List<(Point3D A, Point3D B)>
        {
            (new(-10, 0, -1), new(10, 0, -1)),
            (new(-10, 0, -1), new(10, 0, -1)), // Identical minimizers retain ordinal zero.
            (new(0, -1000, -1), new(0, 1000, -1)), // A long chord cannot be pruned by endpoints alone.
        };
        for (int i = 0; i < 513; i++)
        {
            double x = random.NextDouble() * 80 - 40, y = random.NextDouble() * 80 - 40;
            chords.Add((new(x, y, -1), new(x + random.NextDouble() * 20 - 10, y + random.NextDouble() * 20 - 10, -1)));
        }
        var index = new PreparedCarrierSegments(Path(chords));
        var queries = new List<(Point3D A, Point3D B)> { (new(-2, 1, 0), new(2, 1, 0)) };
        for (int i = 0; i < 96; i++)
            queries.Add((new(random.NextDouble() * 100 - 50, random.NextDouble() * 100 - 50, 0),
                new(random.NextDouble() * 100 - 50, random.NextDouble() * 100 - 50, 0)));
        foreach (var (a, b) in queries)
        {
            double expected = double.MaxValue; int expectedOrdinal = -1;
            for (int i = 0; i < chords.Count; i++)
            {
                double cost = LegacyMinimizer(chords[i].A, chords[i].B, a, b);
                if (cost < expected) { expected = cost; expectedOrdinal = i; }
            }
            double actual = double.MaxValue; int actualOrdinal = int.MaxValue;
            index.VisitCandidates(a, b, () => actual, segment =>
            {
                double cost = LegacyMinimizer(segment.Start, segment.End, a, b);
                if (cost < actual || cost == actual && segment.Ordinal < actualOrdinal)
                { actual = cost; actualOrdinal = segment.Ordinal; }
            });
            Assert.Equal(expected, actual);
            Assert.Equal(expectedOrdinal, actualOrdinal);
        }
    }

    [Fact]
    public void CompleteChordBoundsAvoidAlmostAllUnrelatedMinimizations()
    {
        var chords = new List<(Point3D A, Point3D B)> { (new(-10, 0, -1), new(10, 0, -1)) };
        for (int i = 0; i < 8192; i++)
            chords.Add((new(i * 5 + 100, 50, -1), new(i * 5 + 103, 50, -1)));
        var diagnostics = new PreparedCarrierSegments.Diagnostics();
        var index = new PreparedCarrierSegments(Path(chords), diagnostics);
        double best = double.MaxValue;
        index.VisitCandidates(new(-2, 1, 0), new(2, 1, 0), () => best,
            segment => best = Math.Min(best, LegacyMinimizer(segment.Start, segment.End, new(-2, 1, 0), new(2, 1, 0))),
            diagnostics);
        Assert.InRange(best, 4.472, 4.473);
        Assert.True(diagnostics.CandidateVisits < chords.Count / 100,
            $"Unexpected carrier amplification: {diagnostics.Describe()}.");
        Assert.True(diagnostics.PrunedNodes > 0);
    }

    private static TaggedToolpath Path(IReadOnlyList<(Point3D A, Point3D B)> chords)
    {
        var points = new List<Point3D>(); var spans = new List<ToolpathSpan>();
        foreach (var (a, b) in chords)
        { int first = points.Count; points.Add(a); points.Add(b); spans.Add(new(first, first + 1, ToolpathSpanKind.Cut)); }
        return new TaggedToolpath(points, spans, 0, "pocket", 0, detectCurves: false);
    }

    // The diagnostic reference deliberately keeps the existing 32-iteration
    // evaluator. This verifies only pruning and tie ordering, not a replacement
    // optimizer or a weaker geometric definition of excursion length.
    private static double LegacyMinimizer(Point3D start, Point3D end, Point3D a, Point3D b)
    {
        double low = 0, high = 1;
        for (int iteration = 0; iteration < 32; iteration++)
        {
            double first = (low * 2 + high) / 3, second = (low + high * 2) / 3;
            if (Cost(first) <= Cost(second)) high = second; else low = first;
        }
        return Cost((low + high) * .5);
        double Cost(double position)
        {
            double x = start.X + (end.X - start.X) * position;
            double y = start.Y + (end.Y - start.Y) * position;
            return Math.Sqrt(Math.Pow(x - a.X, 2) + Math.Pow(y - a.Y, 2))
                + Math.Sqrt(Math.Pow(x - b.X, 2) + Math.Pow(y - b.Y, 2));
        }
    }
}
