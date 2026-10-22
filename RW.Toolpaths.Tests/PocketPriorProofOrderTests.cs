using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketPriorProofOrderTests
{
    [Fact]
    public void CompatibleCapturedInnerProofSkipsAnOtherwiseUnresolvedRectangleSubset()
    {
        var stock = StripStock();
        var reference = StripStock(cachedInnerFirst: false);
        var proof = stock.Capture();
        var oldOrder = reference.Capture();
        // This first query has no earlier endpoint poses, hence prepares the
        // same complete INNER stock in both orders without exact subdivision.
        Assert.True(proof.IsSafe(new(2.1, 0, -2), new(7.9, 0, -2)));
        Assert.True(oldOrder.IsSafe(new(2.1, 0, -2), new(7.9, 0, -2)));
        Assert.Equal(1, stock.InnerProofBuilds);
        Assert.Equal(0, stock.ExactRectangleQueries);
        // Its endpoints are actual earlier poses. The bounded exact subset
        // selects the first256 genuine but insufficient nearby old chords;
        // the complete prepared INNER strip already proves the whole capsule.
        Assert.True(proof.IsSafe(new(2, 0, -2), new(8, 0, -2)));
        Assert.True(oldOrder.IsSafe(new(2, 0, -2), new(8, 0, -2)));
        Assert.Equal(1, stock.CachedInnerQueries);
        Assert.Equal(1, stock.CachedInnerProofs);
        Assert.Equal(0, stock.CachedInnerMisses);
        Assert.Equal(0, stock.ExactRectangleQueries);
        // The exact negative applies only to its selected first256 capsules;
        // the omitted eligible strips still authorize the unchanged INNER proof.
        Assert.Equal(1, reference.ExactRectangleNegativeWitnesses);
        Assert.Equal(0, reference.ExactRectangleBudgetRefusals);
        Assert.Equal(0, stock.ExactRectangleNegativeWitnesses);
        Assert.Equal(1, reference.ExactRectangleQueries);
        Assert.Equal(1, stock.InnerProofBuilds);
        Assert.Equal(2, stock.InnerCapsuleQueries);
        Assert.True(stock.InnerPrepareMilliseconds >= 0);
        Assert.True(stock.InnerCapsuleMilliseconds >= 0);
        Assert.True(stock.InnerPrepareMilliseconds + stock.InnerCapsuleMilliseconds
            + stock.ExactRectangleMilliseconds <= stock.ProofMilliseconds + .0001);
    }

    [Fact]
    public void CachedInnerMissStillRunsExactTangentialEndpointAndRectangleAuthority()
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(2, 1), 3, CancellationToken.None);
        stock.Record([new(8, 8, -1), new(8, 52, -1)]);
        stock.Record([new(3, 3, -1), new(13, 3, -1), new(13, 57, -1), new(3, 57, -1), new(3, 3, -1)]);
        var proof = stock.Capture();
        // Regardless of this deliberately non-endpoint probe's answer, its
        // center window contains the following tangential query's endpoints.
        _ = proof.IsSafe(new(4, 5, -2), new(7, 7, -2));
        Assert.Equal(1, stock.InnerProofBuilds);
        Assert.Equal(0, stock.ExactRectangleQueries);
        Assert.True(proof.IsSafe(new(8, 8, -2), new(3, 3, -2)));
        Assert.Equal(1, stock.CachedInnerQueries);
        Assert.Equal(1, stock.CachedInnerMisses);
        Assert.Equal(0, stock.CachedInnerProofs);
        Assert.Equal(1, stock.ExactRectangleProofs);
        // A cached negative was checked exactly once, then exact authority
        // resolved the actual boundary equality without lateral allowance.
        Assert.Equal(2, stock.InnerCapsuleQueries);
        Assert.Equal(1, stock.InnerProofBuilds);
    }

    [Fact]
    public void PreparedOlderNegativeRemainsCapturedAcrossLaterBatchPublicationAndLruEviction()
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(2, 1), 1, CancellationToken.None);
        for (int index = 0; index < 2044; index++)
            stock.Record([new(1000 + 20 * index, 0, -1), new(1010 + 20 * index, 0, -1)]);
        stock.Record([new(2, 0, -1), new(2, 0, -1)]);
        stock.Record([new(8, 0, -1), new(8, 0, -1)]);
        stock.Record([new(0, 1.8, -1), new(10, 1.8, -1)]);
        var older = stock.Capture();
        Assert.False(older.IsSafe(new(2.1, 0, -2), new(7.9, 0, -2)));
        Assert.Equal(1, stock.InnerProofBuilds);
        // Future chords publish an immutable batch across the captured ordinal
        // boundary. The old index and all exact witnesses must remain old.
        stock.Record([new(0, -.2, -1), new(10, -.2, -1)]);
        stock.Record([new(0, .2, -1), new(10, .2, -1)]);
        Assert.False(older.IsSafe(new(2, 0, -2), new(8, 0, -2)));
        Assert.Equal(1, stock.CachedInnerMisses);
        for (int index = 0; index < 129; index++)
            Assert.True(older.IsSafe(new(index, 0, -1), new(index + 1, 0, -1)));
        long priorQueries = stock.ExactRectangleQueries;
        Assert.False(older.IsSafe(new(2, 0, -2), new(8, 0, -2)));
        Assert.Equal(priorQueries + 1, stock.ExactRectangleQueries);
        Assert.Equal(2, stock.CachedInnerMisses);
        Assert.True(stock.ProofMemoEvictions >= 3);
        Assert.True(stock.Capture().IsSafe(new(2, 0, -2), new(8, 0, -2)));
    }

    private static PocketPostedStepdownStock StripStock(bool cachedInnerFirst = true)
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(2, 1), 1, CancellationToken.None,
            permitCachedInnerFirst: cachedInnerFirst);
        for (int index = 0; index < 256; index++)
            stock.Record([new(2, 1.8, -1), new(8, 1.8, -1)]);
        stock.Record([new(0, -.2, -1), new(10, -.2, -1)]);
        stock.Record([new(0, .2, -1), new(10, .2, -1)]);
        stock.Record([new(2, 0, -1), new(2, 0, -1)]);
        stock.Record([new(8, 0, -1), new(8, 0, -1)]);
        return stock;
    }
}
