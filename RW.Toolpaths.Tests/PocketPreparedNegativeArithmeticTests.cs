using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketPreparedNegativeArithmeticTests
{
    private static readonly PocketExactCapsuleCoverage.Segment Horizontal = new(0, 0, 10000, 0);

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void EverySeparatingAxisExcludesTheCompleteSelectedSetWithTheOriginalLimits(int axis)
    {
        (long x, long y) = axis switch { 0 => (7000, 0), 1 => (3000, 0), 2 => (5000, 2000), _ => (5000, -2000) };
        var earlier = Enumerable.Repeat(new PocketExactCapsuleCoverage.Segment(x, y, x, y), 256).ToArray();
        var actual = Compare(Horizontal, earlier, 1);
        Assert.False(actual.Proven);
        Assert.Equal(1, actual.NegativeWitnesses);
        Assert.Equal(1, actual.NegativePreparedVertices);
        Assert.Equal(256, actual.NegativeAxisProofs);
        Assert.Equal(0, actual.NegativeFullDistanceChecks);
        Assert.Equal(256, actual.NegativeDistanceChecks);
        Assert.Equal(0, actual.Refinements);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void OverlappingAxisBoundsRetainTheExactFinitePointEndcapOrInteriorPredicate(int kind)
    {
        var earlier = kind switch
        {
            0 => new PocketExactCapsuleCoverage.Segment(4100, 900, 4100, 900),
            1 => new PocketExactCapsuleCoverage.Segment(4100, 900, 4200, 900),
            _ => new PocketExactCapsuleCoverage.Segment(0, 0, 10000, 10000),
        };
        var actual = Compare(Horizontal, [earlier], 1);
        Assert.False(actual.Proven);
        Assert.Equal(1, actual.NegativeWitnesses);
        Assert.Equal(1, actual.NegativePreparedVertices);
        Assert.Equal(0, actual.NegativeAxisProofs);
        Assert.Equal(1, actual.NegativeFullDistanceChecks);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(1.0000000000000002)]
    [InlineData(3.175)]
    [InlineData(.0009765625)]
    public void QuadraticSideWitnessUsesTheLiteralBinary64RadiusWithoutNormalOrRadiusRounding(double radius)
    {
        var query = new PocketExactCapsuleCoverage.Segment(0, 0, 10001, 12003);
        long shift = (long)Math.Ceiling(radius * 1000 / 2);
        var old = new PocketExactCapsuleCoverage.Segment(0, shift, 10001, 12003 + shift);
        var actual = Compare(query, [old], radius);
        Assert.False(actual.Proven);
        Assert.Equal(3, actual.NegativeCandidates);
        Assert.Equal(1, actual.NegativeWitnesses);
        Assert.Equal(0, actual.NegativeAxisProofs);
        Assert.Equal(1, actual.NegativeFullDistanceChecks);
    }

    [Theory]
    [InlineData(.001)]
    [InlineData(.0009765625)]
    [InlineData(3.175)]
    public void LargeCoordinateApproximationCannotTurnExactTangentialEqualityIntoAnOutsideCertificate(double radius)
    {
        const long origin = 1_000_000_000_000;
        var query = new PocketExactCapsuleCoverage.Segment(origin, -origin, origin + 10001, -origin + 12003);
        var actual = Compare(query, [query], radius, maximumTriangles: 2);
        // Approximate nomination can be unresolved near this magnitude. The
        // rectangle is exactly inside this identical physical capsule, so an
        // exact OUTSIDE witness is forbidden even when the bounded proof stops.
        Assert.Equal(0, actual.NegativeWitnesses);
        Assert.Equal(actual.NegativeDistanceChecks,
            actual.NegativeAxisProofs + actual.NegativeFullDistanceChecks);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ASelectedNegativeKeepsTheInnerFallbackAndCannotCreditACompleteFutureStroke(bool prepared)
    {
        var stock = MissingMiddleStock(prepared);
        var captured = stock.Capture();
        Assert.False(captured.IsSafe(new(0, 0, -2), new(10, 0, -2)));
        Assert.Equal(1, stock.ExactRectangleNegativeWitnesses);
        Assert.Equal(1, stock.InnerProofBuilds);
        Assert.Equal(stock.ExactRectangleNegativeDistanceChecks,
            stock.ExactRectangleNegativeAxisProofs + stock.ExactRectangleNegativeFullDistanceChecks);
        stock.Record([new(0, 0, -1), new(10, 0, -1)]);
        Assert.False(captured.IsSafe(new(0, 0, -2), new(10, 0, -2)));
        Assert.Equal(1, stock.ProofMemoHits);
        Assert.True(stock.Capture().IsSafe(new(0, 0, -2), new(10, 0, -2)));
    }

    internal static PocketPostedStepdownStock MissingMiddleStock(bool prepared = true)
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(2, 1), 1, CancellationToken.None,
            permitPreparedNegativeArithmetic: prepared);
        stock.Record([new(0, 0, -1), new(0, 0, -1)]);
        stock.Record([new(10, 0, -1), new(10, 0, -1)]);
        stock.Record([new(0, -2, -1), new(10, -2, -1)]);
        stock.Record([new(0, 2, -1), new(10, 2, -1)]);
        return stock;
    }

    private static PocketExactCapsuleCoverage.Result Compare(PocketExactCapsuleCoverage.Segment query,
        IReadOnlyList<PocketExactCapsuleCoverage.Segment> earlier, double radius, int maximumTriangles = 512)
    {
        var reference = PocketExactCapsuleCoverage.Prove(query, earlier, radius, CancellationToken.None,
            maximumTriangles: maximumTriangles, usePreparedNegativeArithmetic: false);
        var actual = PocketExactCapsuleCoverage.Prove(query, earlier, radius, CancellationToken.None,
            maximumTriangles: maximumTriangles);
        Assert.Equal((reference.Proven, reference.Triangles, reference.DistanceChecks, reference.Refinements,
                reference.BudgetExhausted, reference.NegativeCandidates, reference.NegativeProbes,
                reference.NegativeWitnesses, reference.NegativeDistanceChecks, reference.NegativeBudgetRefusals),
            (actual.Proven, actual.Triangles, actual.DistanceChecks, actual.Refinements,
                actual.BudgetExhausted, actual.NegativeCandidates, actual.NegativeProbes,
                actual.NegativeWitnesses, actual.NegativeDistanceChecks, actual.NegativeBudgetRefusals));
        Assert.Equal(0, reference.NegativePreparedVertices);
        Assert.Equal(0, reference.NegativeAxisProofs);
        Assert.Equal(reference.NegativeDistanceChecks, reference.NegativeFullDistanceChecks);
        Assert.Equal(actual.NegativeProbes, actual.NegativePreparedVertices);
        Assert.Equal(actual.NegativeDistanceChecks, actual.NegativeAxisProofs + actual.NegativeFullDistanceChecks);
        return actual;
    }
}

[CollectionDefinition("Prepared negative arithmetic controls", DisableParallelization = true)]
public sealed class PocketPreparedNegativeArithmeticCollection { }

[Collection("Prepared negative arithmetic controls")]
public sealed class PocketPreparedNegativeEnvironmentTests
{
    [Fact]
    public void ReferenceEnvironmentIsCapturedPerStockInstanceAndItsCountersRemainIndependent()
    {
        const string name = "RW_TOOLPATHS_PRIOR_NEGATIVE_ARITHMETIC_REFERENCE";
        string? previous = Environment.GetEnvironmentVariable(name);
        try
        {
            Environment.SetEnvironmentVariable(name, "1");
            var reference = PocketPreparedNegativeArithmeticTests.MissingMiddleStock();
            Environment.SetEnvironmentVariable(name, null);
            var prepared = PocketPreparedNegativeArithmeticTests.MissingMiddleStock();
            Assert.False(reference.PreparedNegativeArithmeticEnabled);
            Assert.True(prepared.PreparedNegativeArithmeticEnabled);
            Assert.False(reference.Capture().IsSafe(new(0, 0, -2), new(10, 0, -2)));
            Assert.False(prepared.Capture().IsSafe(new(0, 0, -2), new(10, 0, -2)));
            Assert.Equal(0, reference.ExactRectangleNegativePreparedVertices);
            Assert.Equal(0, reference.ExactRectangleNegativeAxisProofs);
            Assert.Equal(reference.ExactRectangleNegativeDistanceChecks, reference.ExactRectangleNegativeFullDistanceChecks);
            Assert.True(prepared.ExactRectangleNegativeAxisProofs > 0);
            Assert.Equal(1, prepared.ExactRectangleNegativePreparedVertices);
            Assert.Equal(0, prepared.ExactRectangleNegativeFullDistanceChecks);
            Assert.Equal(reference.ExactRectangleNegativeWitnesses, prepared.ExactRectangleNegativeWitnesses);
        }
        finally { Environment.SetEnvironmentVariable(name, previous); }
    }
}
