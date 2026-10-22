using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ClearedColumnStockTests
{
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
