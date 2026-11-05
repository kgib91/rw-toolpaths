using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ClearedColumnStockTests
{
    [Fact]
    public void GrowingExecutedPrefixReusesOnlyEarlierFloorSolidsAndMatchesColdProofs()
    {
        var cache = new ClearedColumnStock.FloorPrefixCache();
        var rows = new List<ClearedColumnStock.Column>();
        const int count = 24;
        for (int i = 0; i < count; i++)
        {
            rows.Add(new(new(i * 1.5, -10, -2), new(i * 1.5, 10, -2), 2));
            var incremental = new ClearedColumnStock(rows, default, floorPrefixCache: cache);
            var cold = new ClearedColumnStock(rows, default);
            foreach (double x in new[] { -3d, 0, i * 1.5 - .75, i * 1.5, i * 1.5 + 3 })
            {
                var a = new Point3D(x, -4, -2);
                var b = new Point3D(x, 4, -2);
                Assert.Equal(cold.Contains(a, b, _ => 1), incremental.Contains(a, b, _ => 1));
            }
            // Future lanes cannot enter the cached proof.
            Assert.False(incremental.Contains(new(i * 1.5 + 3, 0, -2), new(i * 1.5 + 3, 1, -2), _ => 1));
        }
        Assert.Equal(count, cache.AddedChords);
        Assert.Equal(count * (count - 1) / 2, cache.ReusedChords);
        Assert.Equal(count, cache.Builds);
        Console.WriteLine($"[ClearedPrefix.Test] snapshots={count} offsetChords={cache.AddedChords} reusedChords={cache.ReusedChords} coldProofsEqual=True futureCredit=False");
    }

    [Fact]
    public void ChangedFloorRadiusOrSourceCannotReuseAnIncompatibleClearedUnion()
    {
        var cache = new ClearedColumnStock.FloorPrefixCache();
        ClearedColumnStock Make(params ClearedColumnStock.Column[] rows) => new(rows, default, floorPrefixCache: cache);
        Assert.True(Make(new ClearedColumnStock.Column(new(0, -10, -2), new(0, 10, -2), 2))
            .Contains(new(0, -4, -2), new(0, 4, -2), _ => 1));
        // A deeper cut elsewhere does not deepen the old cleared stroke.
        var deeper = Make(new(new(0, -10, -2), new(0, 10, -2), 2),
            new(new(20, -10, -3), new(20, 10, -3), 2));
        Assert.False(deeper.Contains(new(0, -4, -3), new(0, 4, -3), _ => 1));
        Assert.True(deeper.Contains(new(20, -4, -3), new(20, 4, -3), _ => 1));
        Assert.False(Make(new ClearedColumnStock.Column(new(20, -10, -3), new(20, 10, -3), .5))
            .Contains(new(20, -4, -3), new(20, 4, -3), _ => 1));
        Assert.False(Make(new ClearedColumnStock.Column(new(40, -10, -3), new(40, 10, -3), 2))
            .Contains(new(20, -4, -3), new(20, 4, -3), _ => 1));
        cache.Reset();
        Assert.False(Make(new ClearedColumnStock.Column(new(50, -10, -3), new(50, 10, -3), 2))
            .Contains(new(40, -4, -3), new(40, 4, -3), _ => 1));
    }

    [Fact]
    public void AppendingActualBridgeCutsClosesAGapOnlyAfterTheyExecute()
    {
        var cache = new ClearedColumnStock.FloorPrefixCache();
        var rows = new List<ClearedColumnStock.Column> {
            new(new(0, 0, -2), new(1, 0, -2), 1),
            new(new(9, 0, -2), new(10, 0, -2), 1) };
        var before = new ClearedColumnStock(rows, default, floorPrefixCache: cache);
        Assert.False(before.Contains(new(0, 0, -2), new(10, 0, -2), _ => .5));
        rows.Add(new(new(1, 0, -2), new(9, 0, -2), 1));
        var after = new ClearedColumnStock(rows, default, floorPrefixCache: cache);
        Assert.True(after.Contains(new(0, 0, -2), new(10, 0, -2), _ => .5));
        // An old snapshot remains immutable after the cache grows.
        Assert.False(before.Contains(new(0, 0, -2), new(10, 0, -2), _ => .5));
    }

    [Fact]
    public void PrefixGeometryReuseDoesNotShareQueryBudgetsAndCancellationRejectsNewProofs()
    {
        var cache = new ClearedColumnStock.FloorPrefixCache();
        ClearedColumnStock.Column[] rows = [new(new(0, -10, -2), new(0, 10, -2), 2)];
        var first = new ClearedColumnStock(rows, default, maximumQueries:1, floorPrefixCache:cache);
        Assert.True(first.Contains(new(0, 0, -2), new(0, 1, -2), _ => 1));
        Assert.False(first.Contains(new(0, 2, -2), new(0, 3, -2), _ => 1));
        Assert.True(first.QueryBudgetReached);
        using var cancellation = new CancellationTokenSource();
        var next = new ClearedColumnStock(rows, cancellation.Token, maximumQueries:1, floorPrefixCache:cache);
        Assert.True(next.Contains(new(0, 2, -2), new(0, 3, -2), _ => 1));
        Assert.False(next.QueryBudgetReached);
        Assert.Equal(1, cache.Builds);
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => next.Contains(new(0, 4, -2), new(0, 5, -2), _ => 1));
    }

    [Fact]
    public void ProofSessionsReuseOneStockUnionWithoutConsumingAnotherStagesBudget()
    {
        var snapshot = new ClearedColumnStock([
            new(new(-1.5, -20, -2), new(-1.5, 20, -2), 2),
            new(new(1.5, -20, -2), new(1.5, 20, -2), 2)], default);
        var first = snapshot.CreateQuerySession(1);
        Assert.True(first.Contains(new(0, 0, -2), new(0, 0, -2), _ => 1));
        Assert.False(first.Contains(new(0, 1, -2), new(0, 1, -2), _ => 1));
        Assert.True(first.QueryBudgetReached);
        var next = snapshot.CreateQuerySession(1);
        Assert.True(next.Contains(new(0, 1, -2), new(0, 1, -2), _ => 1));
        Assert.False(next.QueryBudgetReached);
        Assert.Equal(1, snapshot.FloorUnionBuilds);
        Assert.Equal(first.FloorUnionBuilds, next.FloorUnionBuilds);
    }

    [Fact]
    public void EqualPostedFloorIsClearedButAnyDeeperCutRemainsRequired()
    {
        var stock = new ClearedColumnStock([new(new(0, 0, -2), new(10, 0, -2), 3)], default);
        Assert.True(stock.Contains(new(2, 0, -2), new(8, 0, -2), _ => 1));
        double deeper = -Math.BitIncrement(2);
        Assert.False(stock.Contains(new(2, 0, deeper), new(8, 0, deeper), _ => 1));
    }

    [Fact]
    public void AdjacentRealRoughingLanesJointlyClearTheFinishingCutter()
    {
        var stock = new ClearedColumnStock([
            new(new(-1.5, -20, -2), new(-1.5, 20, -2), 2),
            new(new(1.5, -20, -2), new(1.5, 20, -2), 2)], default);
        // Neither lane alone contains the cutter: 1.5 + 1 > 2.
        Assert.True(stock.Contains(new(0, -5, -2), new(0, 5, -2), _ => 1));
        Assert.True(stock.FloorUnionAccepted > 0);
        Assert.Equal(1, stock.FloorUnionBuilds);
        Assert.True(stock.Contains(new(0, -3, -1), new(0, 3, -1), _ => 1));
        Assert.Equal(1, stock.FloorUnionBuilds);
    }

    [Fact]
    public void ARealGapAndAShallowerFloorCannotAcquireStockCredit()
    {
        var stock = new ClearedColumnStock([
            new(new(-2, -20, -2), new(-2, 20, -2), 1),
            new(new(2, -20, -2), new(2, 20, -2), 1)], default);
        Assert.False(stock.Contains(new(0, -1, -1), new(0, 1, -1), _ => .5));
        Assert.False(stock.Contains(new(2, -1, -2.001), new(2, 1, -2.001), _ => .5));
    }

    [Fact]
    public void JoiningPostedChainsCannotBridgeAnUncutGap()
    {
        var stock = new ClearedColumnStock([
            new(new(0, 0, -2), new(1, 0, -2), 1),
            new(new(9, 0, -2), new(10, 0, -2), 1)], default);
        Assert.False(stock.Contains(new(0, 0, -2), new(10, 0, -2), _ => .5));
    }
}
