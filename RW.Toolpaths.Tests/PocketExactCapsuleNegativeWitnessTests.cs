using System.Collections;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketExactCapsuleNegativeWitnessTests
{
    private static readonly PocketExactCapsuleCoverage.Segment Query = new(0, 0, 10000, 0);

    [Fact]
    public void ExactMiddleGapStopsSubdivisionOnlyAfterEverySelectedCapsuleIsChecked()
    {
        var earlier = Enumerable.Range(0, 256).Select(index =>
            new PocketExactCapsuleCoverage.Segment(0, index % 2 == 0 ? 2000 : -2000,
                10000, index % 2 == 0 ? 2000 : -2000)).ToArray();
        var current = PocketExactCapsuleCoverage.Prove(Query, earlier, 1, CancellationToken.None);
        var reference = PocketExactCapsuleCoverage.Prove(Query, earlier, 1, CancellationToken.None,
            maximumNegativeProbes: 0);
        Assert.False(current.Proven);
        Assert.False(reference.Proven);
        Assert.Equal(1, current.NegativeCandidates);
        Assert.Equal(1, current.NegativeProbes);
        Assert.Equal(1, current.NegativeWitnesses);
        Assert.Equal(256, current.NegativeDistanceChecks);
        Assert.Equal(1, current.Triangles);
        Assert.Equal(0, current.Refinements);
        Assert.False(current.BudgetExhausted);
        Assert.True(reference.BudgetExhausted);
        Assert.True(reference.Refinements > current.Refinements);
        Assert.True(current.NegativeMilliseconds >= 0);
    }

    [Fact]
    public void SideWitnessDetectsMaterialMissingWhenTheWholeQueryCenterlineIsCovered()
    {
        PocketExactCapsuleCoverage.Segment[] earlier = [new(0, 500, 10000, 500)];
        var proof = PocketExactCapsuleCoverage.Prove(Query, earlier, 1, CancellationToken.None);
        Assert.False(proof.Proven);
        Assert.Equal(3, proof.NegativeCandidates);
        Assert.Equal(1, proof.NegativeProbes);
        Assert.Equal(1, proof.NegativeDistanceChecks);
        Assert.Equal(1, proof.NegativeWitnesses);
        Assert.Equal(0, proof.Refinements);
    }

    [Fact]
    public void AdaptiveDyadicWitnessFindsAGapDespiteCoveredCenterSidesAndAllFourCorners()
    {
        PocketExactCapsuleCoverage.Segment[] earlier = [
            new(0, 0, 0, 0), new(10000, 0, 10000, 0), new(4000, 0, 6000, 0),
        ];
        var proof = PocketExactCapsuleCoverage.Prove(Query, earlier, 1, CancellationToken.None);
        Assert.False(proof.Proven);
        Assert.Equal(1, proof.NegativeWitnesses);
        Assert.True(proof.Refinements > 0);
        Assert.True(proof.NegativeCandidates > 3);
        Assert.InRange(proof.NegativeCandidates, 1, 32);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompleteUnionOfFiniteEndcapsAndRotatedStrokesNeverProducesAnOutsideWitness(bool rotated)
    {
        (long X, long Y) Point(long x, long y) => rotated
            ? (40000 + 3 * x / 5 - 4 * y / 5, 50000 + 4 * x / 5 + 3 * y / 5) : (x, y);
        PocketExactCapsuleCoverage.Segment Segment(long ax, long ay, long bx, long by)
        {
            var a = Point(ax, ay); var b = Point(bx, by);
            return new(a.X, a.Y, b.X, b.Y);
        }
        var query = Segment(8000, 8000, 3000, 3000);
        PocketExactCapsuleCoverage.Segment[] earlier = [
            Segment(8000, 8000, 8000, 52000), Segment(3000, 3000, 13000, 3000),
            Segment(13000, 3000, 13000, 57000), Segment(13000, 57000, 3000, 57000),
            Segment(3000, 57000, 3000, 3000),
        ];
        var current = PocketExactCapsuleCoverage.Prove(query, earlier, 3, CancellationToken.None);
        var reference = PocketExactCapsuleCoverage.Prove(query, earlier, 3, CancellationToken.None,
            maximumNegativeProbes: 0);
        Assert.True(current.Proven);
        Assert.Equal(reference.Proven, current.Proven);
        Assert.Equal(reference.Triangles, current.Triangles);
        Assert.Equal(reference.DistanceChecks, current.DistanceChecks);
        Assert.Equal(0, current.NegativeWitnesses);
    }

    [Fact]
    public void UnionWithExactTangencyThroughoutTheMiddleNeverReceivesAnOutsideWitness()
    {
        // Every point of the query rectangle lies in one of these capsules;
        // they meet with exact equality on the whole query centerline.
        PocketExactCapsuleCoverage.Segment[] earlier = [new(0, 1000, 10000, 1000), new(0, -1000, 10000, -1000)];
        var proof = PocketExactCapsuleCoverage.Prove(Query, earlier, 1, CancellationToken.None,
            maximumTriangles: 2);
        Assert.Equal(0, proof.NegativeWitnesses);
        Assert.Equal(0, proof.NegativeProbes);
        Assert.True(proof.BudgetExhausted);
    }

    [Fact]
    public void OmittedEligibleCapsuleInvalidatesOnlyTheSubsetCertificate()
    {
        PocketExactCapsuleCoverage.Segment[] subset = [new(0, 1800, 10000, 1800)];
        var unresolved = PocketExactCapsuleCoverage.Prove(Query, subset, 1, CancellationToken.None);
        var complete = PocketExactCapsuleCoverage.Prove(Query, [subset[0], Query], 1, CancellationToken.None);
        Assert.False(unresolved.Proven);
        Assert.Equal(1, unresolved.NegativeWitnesses);
        Assert.True(complete.Proven);
        Assert.Equal(0, complete.NegativeWitnesses);
    }

    [Fact]
    public void IncompleteNegativeDistanceBudgetKeepsOriginalSubdivisionAndPublishesNoOutsideWitness()
    {
        PocketExactCapsuleCoverage.Segment[] earlier = [new(0, 2000, 10000, 2000), new(0, -2000, 10000, -2000)];
        var proof = PocketExactCapsuleCoverage.Prove(Query, earlier, 1, CancellationToken.None,
            maximumTriangles: 2, maximumNegativeDistanceChecks: 1);
        Assert.False(proof.Proven);
        Assert.True(proof.BudgetExhausted);
        Assert.Equal(1, proof.NegativeBudgetRefusals);
        Assert.Equal(0, proof.NegativeDistanceChecks);
        Assert.Equal(0, proof.NegativeWitnesses);
        Assert.True(proof.Refinements > 0);
    }

    [Theory]
    [InlineData(0, 2048)]
    [InlineData(8, 0)]
    public void DisabledNegativeBudgetRetainsReferenceProofBehavior(int probes, int distanceChecks)
    {
        PocketExactCapsuleCoverage.Segment[] earlier = [new(0, 2000, 10000, 2000)];
        var proof = PocketExactCapsuleCoverage.Prove(Query, earlier, 1, CancellationToken.None,
            maximumTriangles: 2, maximumNegativeProbes: probes, maximumNegativeDistanceChecks: distanceChecks);
        Assert.False(proof.Proven);
        Assert.True(proof.BudgetExhausted);
        Assert.Equal(0, proof.NegativeCandidates);
        Assert.Equal(0, proof.NegativeWitnesses);
    }

    [Fact]
    public void CancellationDuringTheExactFullSubsetScanPublishesNoCertificate()
    {
        using var canceled = new CancellationTokenSource();
        var earlier = new CancelDuringExactEnumeration(canceled);
        Assert.Throws<OperationCanceledException>(() =>
            PocketExactCapsuleCoverage.Prove(Query, earlier, 1, canceled.Token));
        Assert.Equal(3, earlier.Enumerations);
    }

    private sealed class CancelDuringExactEnumeration(CancellationTokenSource canceled)
        : IReadOnlyList<PocketExactCapsuleCoverage.Segment>
    {
        internal int Enumerations { get; private set; }
        public int Count => 256;
        public PocketExactCapsuleCoverage.Segment this[int index] => new(0, 2000, 10000, 2000);
        public IEnumerator<PocketExactCapsuleCoverage.Segment> GetEnumerator()
        {
            int enumeration = ++Enumerations;
            for (int index = 0; index < Count; index++)
            {
                if (enumeration == 3 && index == 40) canceled.Cancel();
                yield return this[index];
            }
        }
        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }
}
