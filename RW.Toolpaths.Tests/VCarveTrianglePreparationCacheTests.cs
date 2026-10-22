using Clipper2Lib;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.Certification;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class VCarveTrianglePreparationCacheTests(ITestOutputHelper output)
{
    [Fact]
    public void EqualFreshBoundaryReusesOnlyTheCompleteExactDecomposition()
    {
        var owner = new VCarveCoverageCertificate.PreparedTriangleDomain();
        Assert.True(Create(owner, Grid(), false, out var first, out var firstProof, out bool firstReused));
        Assert.False(firstReused);
        Assert.True(firstProof.Created);
        Assert.Equal(0, firstProof.FinalResidualEdges);
        int checkpoints = 0;
        Assert.True(owner.TryCreateOrReuse(Grid(), false, default, () => checkpoints++,
            out var second, out var secondProof, out bool secondReused), secondProof.Reason);
        Assert.True(secondReused);
        Assert.True(checkpoints > 0);
        Assert.Same(first, second);
        Assert.NotSame(firstProof, secondProof);
        Assert.Equal(firstProof.MainTriangles, secondProof.MainTriangles);
        Assert.Equal(firstProof.CorrectionTriangles, secondProof.CorrectionTriangles);
        Assert.Equal(0, secondProof.FinalResidualEdges);
        Assert.Equal(first.ToArray(), second.ToArray());
        output.WriteLine($"reused={secondReused} triangles={second.Count} comparisonCheckpoints={checkpoints} completeExactChain={secondProof.Created}");
    }

    [Fact]
    public void CallerMutationsCannotChangeSavedGridTrianglesOrDiagnostics()
    {
        var source = Grid();
        var owner = new VCarveCoverageCertificate.PreparedTriangleDomain();
        Assert.True(Create(owner, source, false, out var triangles, out var proof, out _));
        SignedTriangle firstTriangle = triangles[0];
        string originalReason = proof.Reason;
        source[0][0] = new(-100, -100);
        source[1].Clear();
        source.Add(Rectangle(8192, 0, 9216, 1024));
        proof.Created = false; proof.FinalResidualEdges = 123; proof.Reason = "caller-mutated";
        Assert.False(triangles is SignedTriangle[]);
        Assert.Throws<NotSupportedException>(() => ((IList<SignedTriangle>)triangles)[0] = default);
        Assert.True(Create(owner, Grid(), false, out var reusedTriangles, out var reusedProof, out bool reused));
        Assert.True(reused);
        Assert.Equal(firstTriangle, reusedTriangles[0]);
        Assert.True(reusedProof.Created);
        Assert.Equal(0, reusedProof.FinalResidualEdges);
        Assert.Equal(originalReason, reusedProof.Reason);
        output.WriteLine("callerInputMutationIsolated=True returnedTrianglesReadOnly=True returnedDiagnosticsIsolated=True");
    }

    [Theory]
    [InlineData("coordinate")]
    [InlineData("ring-order")]
    [InlineData("ring-count")]
    [InlineData("point-order")]
    [InlineData("quality")]
    public void EveryOriginalBoundarySequenceAndQualityFlagParticipatesInReuse(string change)
    {
        var owner = new VCarveCoverageCertificate.PreparedTriangleDomain();
        Assert.True(Create(owner, Grid(), false, out _, out _, out _));
        Paths64 changed = Grid();
        bool useDelaunay = false;
        switch (change)
        {
            case "coordinate": changed[0][1] = new(1280, 0); break;
            case "ring-order": (changed[0], changed[1]) = (changed[1], changed[0]); break;
            case "ring-count": changed.Add(Rectangle(8192, 0, 9216, 1024)); break;
            case "point-order":
                Point64 start = changed[0][0]; changed[0].RemoveAt(0); changed[0].Add(start);
                break;
            case "quality": useDelaunay = true; break;
        }
        Assert.True(Create(owner, changed, useDelaunay, out var rebuilt, out var proof, out bool reused));
        Assert.False(reused);
        Assert.True(proof.Created);
        Assert.Equal(0, proof.FinalResidualEdges);
        Assert.True(Create(owner, changed, useDelaunay, out var next, out _, out bool nextReused));
        Assert.True(nextReused);
        Assert.Same(rebuilt, next);
        output.WriteLine($"change={change} firstRequestReused={reused} identicalNextRequestReused={nextReused}");
    }

    [Fact]
    public void CancellationBeforeCreationCannotPublishAPreparedDomain()
    {
        var owner = new VCarveCoverageCertificate.PreparedTriangleDomain();
        using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => owner.TryCreateOrReuse(Grid(), false,
            cancelled.Token, null, out _, out _, out _));
        Assert.True(Create(owner, Grid(), false, out _, out _, out bool reused));
        Assert.False(reused);
    }

    [Fact]
    public void CancellationAndQuotaCheckpointsRemainLiveOnCacheReuse()
    {
        var owner = new VCarveCoverageCertificate.PreparedTriangleDomain();
        Assert.True(Create(owner, Grid(), false, out _, out _, out _));
        using var cancellation = new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(() => owner.TryCreateOrReuse(Grid(), false,
            cancellation.Token, cancellation.Cancel, out _, out _, out _));
        Assert.Throws<InvalidOperationException>(() => owner.TryCreateOrReuse(Grid(), false,
            default, () => throw new InvalidOperationException("caller-work-quota"), out _, out _, out _));
        Assert.True(Create(owner, Grid(), false, out _, out _, out bool reused));
        Assert.True(reused);
        output.WriteLine("reuseCancellationPropagated=True reuseQuotaPropagated=True priorSuccessfulEntryPreserved=True");
    }

    [Fact]
    public void RejectedInputsAreNeverCachedAndCanBeRepaired()
    {
        var owner = new VCarveCoverageCertificate.PreparedTriangleDomain();
        var invalid = new Paths64 { new() { new(0, 0), new(1024, 0) } };
        for (int attempt = 0; attempt < 2; attempt++)
        {
            Assert.False(Create(owner, invalid, false, out var triangles, out var proof, out bool reused));
            Assert.False(reused);
            Assert.False(proof.Created);
            Assert.Empty(triangles);
        }
        invalid[0] = Rectangle(0, 0, 1024, 1024);
        Assert.True(Create(owner, invalid, false, out _, out _, out bool firstReused));
        Assert.False(firstReused);
        Assert.True(Create(owner, invalid, false, out _, out _, out bool nextReused));
        Assert.True(nextReused);
    }

    [Fact]
    public void FailedRequestDoesNotPoisonThePreviousSuccessfulExactEntry()
    {
        var owner = new VCarveCoverageCertificate.PreparedTriangleDomain();
        Assert.True(Create(owner, Grid(), false, out var original, out _, out _));
        var invalid = new Paths64 { new() { new(0, 0), new(1024, 0) } };
        Assert.False(Create(owner, invalid, true, out _, out _, out bool failedReused));
        Assert.False(failedReused);
        Assert.True(Create(owner, Grid(), false, out var next, out _, out bool reused));
        Assert.True(reused);
        Assert.Same(original, next);
    }

    [Fact]
    public void ASeparateGenerationOwnerMustBuildItsOwnExactSnapshot()
    {
        var first = new VCarveCoverageCertificate.PreparedTriangleDomain();
        var second = new VCarveCoverageCertificate.PreparedTriangleDomain();
        Assert.True(Create(first, Grid(), false, out var firstTriangles, out _, out _));
        Assert.True(Create(second, Grid(), false, out var secondTriangles, out _, out bool reused));
        Assert.False(reused);
        Assert.NotSame(firstTriangles, secondTriangles);
        Assert.Equal(firstTriangles.ToArray(), secondTriangles.ToArray());
    }

    [Fact]
    public async Task ConcurrentIdenticalRequestsPublishOneSuccessfulImmutableSnapshot()
    {
        var owner = new VCarveCoverageCertificate.PreparedTriangleDomain();
        var tasks = Enumerable.Range(0, 4).Select(_ => Task.Run(() =>
        {
            bool created = owner.TryCreateOrReuse(Grid(), false, default, null,
                out var triangles, out var proof, out bool reused);
            return (created, triangles, proof, reused);
        })).ToArray();
        var results = await Task.WhenAll(tasks);
        Assert.All(results, result =>
        {
            Assert.True(result.created, result.proof.Reason);
            Assert.Equal(0, result.proof.FinalResidualEdges);
            Assert.Same(results[0].triangles, result.triangles);
        });
        Assert.Single(results.Where(result => !result.reused));
    }

    private bool Create(VCarveCoverageCertificate.PreparedTriangleDomain owner, Paths64 grid, bool quality,
        out IReadOnlyList<SignedTriangle> triangles, out ExactSignedBoundaryDecomposition.Diagnostics proof, out bool reused)
    {
        bool created = owner.TryCreateOrReuse(grid, quality, default, null, out triangles, out proof, out reused);
        output.WriteLine($"created={created} reused={reused} quality={quality} reason={proof.Reason} "
            + $"sourcePoints={proof.InputPoints} mainTriangles={proof.MainTriangles} corrections={proof.CorrectionTriangles} finalResidual={proof.FinalResidualEdges}");
        return created;
    }
    private static Paths64 Grid() => new() { Rectangle(0, 0, 1024, 1024), Rectangle(4096, 0, 5120, 1024) };
    private static Path64 Rectangle(long x0, long y0, long x1, long y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
}
