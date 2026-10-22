using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketIndexedCoverageTests
{
    [Fact]
    public void LocalCoverageKeepsLegacyDecisionsAcrossFloorsHolesAndExecutedPrefixes()
    {
        var indexed = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        var legacy = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var random = new Random(96143);
        // Deliberately disconnected components, thin annular interiors, ramps
        // crossing several floors, and repeated lower-depth strokes. Every
        // query checks a complete cutter capsule rather than sampled vertices.
        List<Point3D[]> strokes =
        [
            [new(0, 0, -3), new(20, 0, -3), new(20, 20, -3), new(0, 20, -3), new(0, 0, -3)],
            [new(100, 100, -3), new(120, 100, -3), new(120, 120, -3), new(100, 120, -3), new(100, 100, -3)],
            [new(0, 5, 0), new(20, 5, -3)],
            [new(0, 5.5, -3), new(20, 5.5, -3)],
        ];
        for (int i = 0; i < 16; i++)
        {
            double x = random.NextDouble() * 20, y = random.NextDouble() * 20;
            strokes.Add([new(x, y, i % 3 == 0 ? 0 : -3), new(x + 1 + random.NextDouble() * 6, y + random.NextDouble(), -3)]);
        }
        int approved = 0, refused = 0;
        foreach (var stroke in strokes)
        {
            indexed.Record(stroke); legacy.Record(stroke);
            foreach (double floor in new[] { -1d, -2d, -3d })
            {
                IReadOnlyList<PointD>[] following = [new PointD[] { new(22, 5), new(22, 9) }];
                var local = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
                    indexed, floor, following, .5, CancellationToken.None, diagnostics);
                var control = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
                    legacy, floor, following, .5, CancellationToken.None, useIndexedCoverage: false);
                var queries = new List<(Point3D A, Point3D B)>
                {
                    (new(5, 0, floor), new(15, 0, floor)),
                    (new(5, 5.25, floor), new(15, 5.25, floor)),
                    (new(5, 10, floor), new(15, 10, floor)),
                    (new(15, 5, floor), new(22, 5, floor)),
                    (new(105, 100, floor), new(115, 100, floor)),
                };
                for (int i = 0; i < 6; i++)
                {
                    double x = random.NextDouble() * 25, y = random.NextDouble() * 22;
                    queries.Add((new(x, y, floor), new(x + random.NextDouble() * 4, y + random.NextDouble() * 4, floor)));
                }
                foreach (var (a, b) in queries)
                {
                    bool expected = control.IsTravelSafe(a, b, .5);
                    Assert.Equal(expected, local.IsTravelSafe(a, b, .5));
                    if (expected) approved++; else refused++;
                }
            }
        }
        Assert.True(approved > 100); Assert.True(refused > 100);
        Assert.True(diagnostics.LocalProofs > 0);
        Assert.True(diagnostics.WholeFallbacks > 0);
    }

    [Fact]
    public void CapturedCoverageNeverCreditsLaterExecutionOrAFutureFloor()
    {
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        stock.Record([new(0, 0, -1), new(5, 0, -1)]);
        var before = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None);
        stock.Record([new(5, 0, -1), new(25, 0, -1)]);
        // Materialize the later prefix first. An older snapshot must still
        // refuse the virgin segment even when it needs the global fallback.
        stock.AtFloor(-1);
        Assert.False(before.IsTravelSafe(new(10, 0, -1), new(20, 0, -1), .5));
        var after = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None);
        Assert.True(after.IsTravelSafe(new(10, 0, -1), new(20, 0, -1), .5));
        var deeper = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -2, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None);
        Assert.False(deeper.IsTravelSafe(new(1, 0, -2), new(4, 0, -2), .5));
    }

    [Fact]
    public void AShortLocalCapsulePrunesDistantSweepsWithoutMaterializingTheirUnion()
    {
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        for (int i = 0; i < 48; i++)
        {
            double x = i * 100;
            stock.Record([new(x, 0, -1), new(x + 20, 0, -1), new(x + 20, .2, -1), new(x, .2, -1)]);
            stock.PrepareAtFloor(-1); // Preserve distinct immutable physical batches.
        }
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var proof = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None, diagnostics);
        Assert.True(proof.IsTravelSafe(new(5, .1, -1), new(15, .1, -1), .5));
        Assert.Equal(1, diagnostics.BatchCandidates);
        Assert.Equal(1, diagnostics.LocalProofs);
        Assert.Equal(0, diagnostics.WholeFallbacks);
        Assert.Equal(0, stock.FloorUnionCalls);
    }

    [Fact]
    public void ExactCapturedDomainCropKeepsItsToleranceAfterUnrelatedPrefixGrowth()
    {
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        for (int i = 0; i < 32; i++)
        {
            double x = i * 100;
            stock.Record([new(x, 0, -1), new(x + 20, 0, -1)]);
            stock.PrepareAtFloor(-1);
        }
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var proof = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None, diagnostics,
            validateIndexedCoverage: true);
        // An exact same-radius retrace has no interior spare clearance. Historical
        // batches must not consume the local offset's construction allowance.
        Assert.True(proof.IsTravelSafe(new(5, 0, -1), new(15, 0, -1), .5));
        Assert.Equal(1, diagnostics.HistoricalReserveExhausted);
        Assert.Equal(1, diagnostics.CroppedDomainProofs);
        Assert.Equal(1, diagnostics.ValidatedLocalProofs);
        Assert.Equal(0, diagnostics.WholeFallbacks);
        Assert.Equal(0, diagnostics.CapturedUnionBuilds);
        Assert.Equal(1, diagnostics.ContourBoundsBuilds);
        Assert.True(diagnostics.ContourBoundsSubmittedVertices < diagnostics.ContourBoundsConsideredVertices / 8);
        Assert.True(diagnostics.LocalInputVertices < 20);
    }

    [Fact]
    public void JointSignedDomainCropKeepsAnExecutedHoleAndAnAuthorizedIslandDistinct()
    {
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        stock.Record([new(0, 0, -1), new(20, 0, -1), new(20, 20, -1), new(0, 20, -1), new(0, 0, -1)]);
        IReadOnlyList<PointD>[] following = [new PointD[] { new(10, 10), new(12, 10) }];
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var outsideIsland = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, following, .5, CancellationToken.None, diagnostics, validateIndexedCoverage: true);
        // The first following cut is an authorized positive island inside the
        // executed annulus's negative hole, not permission for the entire hole.
        Assert.False(outsideIsland.IsTravelSafe(new(5, 10, -1), new(15, 10, -1), .5));
        Assert.Equal(0, diagnostics.CroppedDomainProofs);
        Assert.Equal(0, diagnostics.LocalProofMismatches);
        var onIsland = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, following, .5, CancellationToken.None, diagnostics, validateIndexedCoverage: true);
        Assert.True(onIsland.IsTravelSafe(new(10.2, 10, -1), new(11.8, 10, -1), .5));
        Assert.Equal(1, diagnostics.CroppedDomainProofs);
        Assert.Equal(1, diagnostics.ValidatedLocalProofs);
        Assert.Equal(0, diagnostics.LocalProofMismatches);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(17)]
    [InlineData(45)]
    [InlineData(89)]
    public void CroppedDiagonalIntersectionsRetainWholeCapsuleDecisionsNearGridAndToleranceEdges(double degrees)
    {
        double angle = degrees * Math.PI / 180, c = Math.Cos(angle), s = Math.Sin(angle);
        double q = 1 / PathUtils.Scale;
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        Point3D Point(double t, double normal = 0)
            => new(.5 * q + 20 * c * t - s * normal, -.5 * q + 20 * s * t + c * normal, -1);
        stock.Record([Point(0), Point(1)]);
        stock.PrepareAtFloor(-1);
        // Include a signed annular footprint and separated sweeps. The crop may
        // intersect both the outer contour and its hole, never filling the hole.
        stock.Record([new(40, 40, -1), new(60, 40, -1), new(60, 60, -1), new(40, 60, -1), new(40, 40, -1)]);
        stock.PrepareAtFloor(-1);
        for (int i = 0; i < 14; i++)
        {
            stock.Record([new(100 + i * 30, 0, -1), new(120 + i * 30, 0, -1)]);
            stock.PrepareAtFloor(-1);
        }
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        int approved = 0, refused = 0;
        foreach (double normal in new[] { -0.0035, -0.0027, -0.001, -q / 2, 0, q / 2, 0.001, 0.0027, 0.0035 })
        {
            var candidate = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
                stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None, diagnostics,
                validateIndexedCoverage: true);
            var reference = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
                stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None, useIndexedCoverage: false);
            bool safe = candidate.IsTravelSafe(Point(.2, normal), Point(.8, normal), .5);
            Assert.Equal(reference.IsTravelSafe(Point(.2, normal), Point(.8, normal), .5), safe);
            if (safe)
            {
                approved++;
                // Independent analytic capsule-in-capsule bound: every point of
                // this parallel query capsule is within radius + construction
                // allowance of the actual source chord, including its round caps.
                Assert.True(Math.Abs(normal) <= .003 + 4 * q);
            }
            else refused++;
        }
        var hole = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None, diagnostics,
            validateIndexedCoverage: true);
        Assert.False(hole.IsTravelSafe(new(45, 50, -1), new(55, 50, -1), .5));
        Assert.True(approved >= 5); Assert.True(refused >= 2);
        Assert.True(diagnostics.CroppedDomainProofs >= 5);
        Assert.Equal(0, diagnostics.LocalProofMismatches);
    }
}
