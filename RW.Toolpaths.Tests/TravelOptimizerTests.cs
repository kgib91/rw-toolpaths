using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

/// <summary>
/// Island sequencing decides how much of a job is spent cutting air, so the tour must respect
/// the cutter's starting position and beat a naive left-to-right walk.
/// </summary>
public class TravelOptimizerTests
{
    private static List<PointD> Square(double originX, double originY, double size = 10) => new()
    {
        new PointD(originX, originY),
        new PointD(originX + size, originY),
        new PointD(originX + size, originY + size),
        new PointD(originX, originY + size),
    };

    private static double TourLength(
        IReadOnlyList<IReadOnlyList<PointD>> islands,
        IReadOnlyList<int> order,
        PointD? start)
    {
        double total = 0;
        PointD? previous = start;

        foreach (int index in order)
        {
            if (previous is { } from)
                total += ClosestApproach(islands[index], from);
            previous = islands[index][0];
        }
        return total;
    }

    private static double ClosestApproach(IReadOnlyList<PointD> island, PointD point)
    {
        double best = double.MaxValue;
        foreach (var vertex in island)
        {
            double dx = vertex.x - point.x;
            double dy = vertex.y - point.y;
            best = Math.Min(best, Math.Sqrt(dx * dx + dy * dy));
        }
        return best;
    }

    [Fact]
    public void SingleIsland_IsReturnedUnchanged()
    {
        var islands = new List<IReadOnlyList<PointD>> { Square(0, 0) };

        Assert.Equal(new[] { 0 }, TravelOptimizer.Order(islands));
    }

    [Fact]
    public void EmptyInput_ReturnsEmptyOrder()
    {
        Assert.Empty(TravelOptimizer.Order(new List<IReadOnlyList<PointD>>()));
    }

    [Fact]
    public void EveryIslandIsVisitedExactlyOnce()
    {
        var islands = new List<IReadOnlyList<PointD>>();
        for (int i = 0; i < 12; i++)
            islands.Add(Square(i * 37 % 200, i * 53 % 200));

        var order = TravelOptimizer.Order(islands);

        Assert.Equal(islands.Count, order.Length);
        Assert.Equal(islands.Count, order.Distinct().Count());
    }

    [Fact]
    public void StartPositionDecidesWhichEndOfTheRowIsCutFirst()
    {
        var islands = new List<IReadOnlyList<PointD>>
        {
            Square(0, 0),
            Square(50, 0),
            Square(100, 0),
        };

        Assert.Equal(0, TravelOptimizer.Order(islands, new PointD(-20, 0))[0]);
        Assert.Equal(2, TravelOptimizer.Order(islands, new PointD(140, 0))[0]);
    }

    [Fact]
    public void IslandsInARowAreCutInSequence()
    {
        var islands = new List<IReadOnlyList<PointD>>
        {
            Square(100, 0),
            Square(0, 0),
            Square(200, 0),
            Square(50, 0),
            Square(150, 0),
        };

        var order = TravelOptimizer.Order(islands, new PointD(-20, 0));

        Assert.Equal(new[] { 1, 3, 0, 4, 2 }, order);
    }

    [Fact]
    public void LocalSearchBeatsTheInputOrdering()
    {
        // Interleaved columns: walking them in index order zig-zags across the table.
        var islands = new List<IReadOnlyList<PointD>>();
        for (int i = 0; i < 10; i++)
            islands.Add(Square(i % 2 == 0 ? i * 20 : 400 - i * 20, 0));

        var start = new PointD(-50, 0);
        var order = TravelOptimizer.Order(islands, start);

        double optimized = TourLength(islands, order, start);
        double asGiven = TourLength(islands, Enumerable.Range(0, islands.Count).ToArray(), start);

        Assert.True(optimized < asGiven,
            $"optimizer did not improve on the input order: {optimized:F1} vs {asGiven:F1}");
    }

    [Fact]
    public void ClusteredIslandsAreFinishedBeforeMovingToTheNextCluster()
    {
        var islands = new List<IReadOnlyList<PointD>>
        {
            Square(0, 0),
            Square(1000, 0),
            Square(20, 0),
            Square(1020, 0),
            Square(40, 0),
        };

        var order = TravelOptimizer.Order(islands, new PointD(-50, 0));

        // Indices 0/2/4 are the near cluster; they must all precede the far one.
        var nearPositions = new[] { 0, 2, 4 }.Select(i => Array.IndexOf(order, i));
        var farPositions = new[] { 1, 3 }.Select(i => Array.IndexOf(order, i));

        Assert.True(nearPositions.Max() < farPositions.Min(),
            "the tour left the near cluster and came back");
    }

    [Fact]
    public void OrderIsStableForRepeatedRuns()
    {
        var islands = new List<IReadOnlyList<PointD>>();
        for (int i = 0; i < 15; i++)
            islands.Add(Square(i * 31 % 300, i * 17 % 300));

        var first = TravelOptimizer.Order(islands, new PointD(0, 0));
        var second = TravelOptimizer.Order(islands, new PointD(0, 0));

        Assert.Equal(first, second);
    }
}
