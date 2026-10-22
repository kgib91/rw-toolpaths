using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketExactCapsuleCoverageTests
{
    [Theory]
    [InlineData(3.175, 10000, 0)]
    [InlineData(3.175, 9000, 12000)]
    [InlineData(3.175, 10001, 12003)]
    [InlineData(.0009765625, 101, 37)]
    public void ExactPhysicalRadiusAndQuadraticNormalRetainTangentialRectangleEquality(double radius, long x, long y)
    {
        var segment = new PocketExactCapsuleCoverage.Segment(123, -457, 123 + x, -457 + y);
        var proof = PocketExactCapsuleCoverage.Prove(segment, [segment], radius, CancellationToken.None);
        Assert.True(proof.Proven);
        Assert.Equal(2, proof.Triangles);
        Assert.Equal(6, proof.DistanceChecks);
        Assert.Equal(0, proof.Refinements);
        Assert.Equal(0, proof.NegativeWitnesses);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PreviouslyCutEndpointDiscsAndFiniteCapsuleTrianglesKeepTheRingAndStripContinuous(bool oblique)
    {
        Point3D Point(double x, double y, double depth)
            => oblique ? new(40 + .6 * x - .8 * y, 50 + .8 * x + .6 * y, depth) : new(x, y, depth);
        var stock = new PocketPostedStepdownStock(new DepthSchedule(2, 1), 3, CancellationToken.None);
        var reference = new PocketPostedStepdownStock(new DepthSchedule(2, 1), 3, CancellationToken.None,
            permitExactRectangle: false);
        Point3D[][] strokes = [
            [Point(8, 8, -1), Point(8, 52, -1)],
            [Point(3, 3, -1), Point(13, 3, -1), Point(13, 57, -1), Point(3, 57, -1), Point(3, 3, -1)],
        ];
        foreach (var stroke in strokes) { stock.Record(stroke); reference.Record(stroke); }
        var from = Point(8, 8, -2); var to = Point(3, 3, -2);
        Assert.False(reference.Capture().IsSafe(from, to));
        Assert.True(stock.Capture().IsSafe(from, to));
        Assert.Equal(1, stock.ExactRectangleProofs);
        Assert.Equal(0, stock.InnerProofBuilds);
        Assert.Equal(0, stock.Refusals);
        Assert.True(stock.ExactRectangleRefinements > 0);
        Assert.Equal(0, stock.ExactRectangleNegativeWitnesses);
    }

    [Fact]
    public void EarlierEndpointDiscsNeverAuthorizeMissingMaterialBetweenThem()
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(2, 1), 1, CancellationToken.None);
        stock.Record([new(0, 0, -1), new(0, 0, -1)]);
        stock.Record([new(10, 0, -1), new(10, 0, -1)]);
        stock.Record([new(0, -2, -1), new(10, -2, -1)]);
        stock.Record([new(0, 2, -1), new(10, 2, -1)]);
        // (5,0) is in the target sweep and farther thanR from every old stroke.
        Assert.False(stock.Capture().IsSafe(new(0, 0, -2), new(10, 0, -2)));
        Assert.Equal(1, stock.ExactRectangleQueries);
        Assert.Equal(0, stock.ExactRectangleProofs);
        Assert.True(stock.ExactRectangleDistanceChecks <= 4096);
        Assert.Equal(1, stock.ExactRectangleNegativeWitnesses);
    }

    [Fact]
    public void FutureCapsulesAcrossImmutablePublicationCannotCompleteAnOldRectangleCertificate()
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(2, 1), 3, CancellationToken.None);
        for (int index = 0; index < 2046; index++)
            stock.Record([new(1000 + 20 * index, 0, -1), new(1010 + 20 * index, 0, -1)]);
        stock.Record([new(8, 8, -1), new(8, 52, -1)]);
        stock.Record([new(3, 3, -1), new(3, 3, -1)]);
        var captured = stock.Capture();
        stock.Record([new(3, 3, -1), new(13, 3, -1), new(13, 57, -1), new(3, 57, -1), new(3, 3, -1)]);
        Assert.False(captured.IsSafe(new(8, 8, -2), new(3, 3, -2)));
        Assert.True(stock.Capture().IsSafe(new(8, 8, -2), new(3, 3, -2)));
        Assert.Equal(1, stock.ExactRectangleProofs);
    }

    [Fact]
    public void UnresolvedBoundedTriangulationAndCancellationPublishNoCertificate()
    {
        var query = new PocketExactCapsuleCoverage.Segment(0, 0, 10000, 0);
        var misses = new PocketExactCapsuleCoverage.Segment(0, 2000, 10000, 2000);
        var proof = PocketExactCapsuleCoverage.Prove(query, [misses], 1, CancellationToken.None,
            maximumTriangles: 2, maximumDistanceChecks: 3, maximumNegativeProbes: 0);
        Assert.False(proof.Proven);
        Assert.True(proof.BudgetExhausted);
        using var canceled = new CancellationTokenSource();
        canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => PocketExactCapsuleCoverage.Prove(query, [query], 1, canceled.Token));
    }

    [Fact]
    public void CylinderHeightMustCoverTheActualDeeperPostedF3Floor()
    {
        var depth = new DepthSchedule(3.0006, 1) { SurfaceZ = 12.7 };
        double required = PocketContinuationRouter.RequiredPostedCylinderHeight(depth);
        Assert.Equal(3.001, required);
        var tool = new ToolGeometry(3) { VCarveProfile = new VCarveCutterProfile(
            [new(CarvingCutterSectionKind.Linear, 3, 3, 3.0008)], 3.0008) };
        Assert.True(PocketContinuationRouter.TryFlatCylinderRadius(tool, depth.Depth, out _));
        Assert.False(PocketContinuationRouter.TryFlatCylinderRadius(tool, required, out _));
    }

    [Fact]
    public void CompletedQueryMemoUsesFullPostedDepthAndNeverCreditsLaterStockToAnOldPrefix()
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5), 1, CancellationToken.None);
        stock.Record([new(0, 0, -3.5), new(10, 0, -3.5)]);
        var proof = stock.Capture();
        Assert.True(proof.IsSafe(new(2, 0, -7), new(8, 0, -7)));
        Assert.True(proof.IsSafe(new(2, 0, -7), new(8, 0, -7)));
        Assert.Equal(1, stock.ProofMemoHits);
        Assert.False(proof.IsSafe(new(2, 0, -7.001), new(8, 0, -7.001)));
        Assert.False(proof.IsSafe(new(2, .01, -7), new(8, .01, -7)));
        stock.Record([new(2, .01, -3.5), new(8, .01, -3.5)]);
        Assert.False(proof.IsSafe(new(2, .01, -7), new(8, .01, -7)));
        Assert.Equal(2, stock.ProofMemoHits);
        Assert.True(stock.Capture().IsSafe(new(2, .01, -7), new(8, .01, -7)));
    }

    [Fact]
    public void QueryMemoIsBoundedAndCancellationStillRunsBeforeCachedApproval()
    {
        using var canceled = new CancellationTokenSource();
        var stock = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5), 1, canceled.Token);
        var proof = stock.Capture();
        for (int index = 0; index < 129; index++)
            Assert.True(proof.IsSafe(new(index, 0, -3.5), new(index + 1, 0, -3.5)));
        Assert.Equal(128, stock.ProofMemoPeakEntries);
        Assert.Equal(1, stock.ProofMemoEvictions);
        Assert.Equal(0, stock.ProofMemoBypasses);
        Assert.True(proof.IsSafe(new(128, 0, -3.5), new(129, 0, -3.5)));
        Assert.Equal(1, stock.ProofMemoHits);
        // The oldest query was evicted, while the recently used answer stays.
        Assert.True(proof.IsSafe(new(0, 0, -3.5), new(1, 0, -3.5)));
        Assert.Equal(2, stock.ProofMemoEvictions);
        Assert.True(proof.IsSafe(new(128, 0, -3.5), new(129, 0, -3.5)));
        Assert.Equal(2, stock.ProofMemoHits);
        canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => proof.IsSafe(new(128, 0, -3.5), new(129, 0, -3.5)));
    }
}
