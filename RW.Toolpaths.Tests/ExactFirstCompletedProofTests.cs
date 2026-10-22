using Gate = RW.Toolpaths.Milling.Certification.ExactFirstCoverageGate;
using SourceProof = RW.Toolpaths.Milling.Certification.SignedBoundaryCapsuleCertificate;

namespace RW.Toolpaths.Tests;

[Collection("Local original medial diagnostic controls")]
public sealed class ExactFirstCompletedProofTests
{
    [Theory]
    [InlineData(9000d, false, false)]
    [InlineData(10000d, false, false)]
    [InlineData(10300d, false, true)]
    [InlineData(9000d, true, false)]
    [InlineData(10000d, true, false)]
    [InlineData(10300d, true, true)]
    public void CompletedOriginalProofIsAcceptedWithElapsedDiagnostic(
        double elapsedMilliseconds, bool retainedEnclosures, bool exceededDeadline)
    {
        var result = CompleteEvidence(retainedEnclosures);
        var source = result.SourceCertificate;

        Assert.True(Gate.CompleteComputedProof(result, CancellationToken.None, elapsedMilliseconds));

        Assert.True(result.Certified);
        Assert.Equal("complete-original-reachable-stock-covered", result.Reason);
        Assert.Equal(elapsedMilliseconds, result.TotalMilliseconds);
        Assert.Equal(exceededDeadline, result.DeadlineExceededAfterCompletedProof);
        Assert.True(result.GapComputed);
        Assert.Equal(0, result.GapPaths);
        Assert.True(result.PhysicalCertified);
        Assert.Same(source, result.SourceCertificate);
    }

    public static IEnumerable<object[]> IncompleteEvidenceCases()
    {
        string[] missing =
        {
            "gap-not-computed", "positive-gap", "invalid-negative-gap", "physical-not-certified",
            "source-missing", "source-default", "source-chain-unclosed", "source-chain-invalid-negative",
            "source-owners-not-enforced", "source-neither-completion-flag", "source-work-budget-partial",
        };
        foreach (string defect in missing)
        foreach (double elapsed in new[] { 9000d, 10000d, 10300d })
            yield return new object[] { defect, elapsed };
    }

    [Theory]
    [MemberData(nameof(IncompleteEvidenceCases))]
    public void MissingOrPartialEvidenceNeverGainsCreditFromCompletionTime(string defect, double elapsedMilliseconds)
    {
        var result = CompleteEvidence();
        switch (defect)
        {
            case "gap-not-computed": result.GapComputed = false; break;
            case "positive-gap": result.GapPaths = 1; break;
            case "invalid-negative-gap": result.GapPaths = -1; break;
            case "physical-not-certified": result.PhysicalCertified = false; break;
            case "source-missing": result.SourceCertificate = null; break;
            case "source-default": result.SourceCertificate = new SourceProof.Diagnostics(); break;
            case "source-chain-unclosed": result.SourceCertificate!.FinalResidualEdges = 1; break;
            case "source-chain-invalid-negative": result.SourceCertificate!.FinalResidualEdges = -1; break;
            case "source-owners-not-enforced": result.SourceCertificate!.OriginalOwnerRegistryEnforced = false; break;
            case "source-neither-completion-flag":
                result.SourceCertificate!.Certified = false;
                result.SourceCertificate.SupportOutsideEnclosuresCertified = false;
                break;
            case "source-work-budget-partial":
                result.SourceCertificate!.Certified = false;
                result.SourceCertificate.SupportOutsideEnclosuresCertified = false;
                result.SourceCertificate.Reason = "capsule-work-budget";
                result.SourceCertificate.UnresolvedPieces = 1;
                break;
            default: throw new ArgumentOutOfRangeException(nameof(defect), defect, "Unknown evidence defect.");
        }

        Assert.False(Gate.CompleteComputedProof(result, CancellationToken.None, elapsedMilliseconds));
        Assert.False(result.Certified);
        Assert.NotEqual("complete-original-reachable-stock-covered", result.Reason);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    [InlineData(-1d)]
    public void InvalidElapsedTimeCannotPublishCompletedProof(double elapsedMilliseconds)
    {
        var result = CompleteEvidence();
        Assert.False(Gate.CompleteComputedProof(result, CancellationToken.None, elapsedMilliseconds));
        Assert.False(result.Certified);
        Assert.NotEqual("complete-original-reachable-stock-covered", result.Reason);
    }

    [Theory]
    [InlineData(9000d)]
    [InlineData(10300d)]
    public void CancellationStillPropagatesWhenEveryProofHasCompleted(double elapsedMilliseconds)
    {
        var result = CompleteEvidence();
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();

        Assert.Throws<OperationCanceledException>(() =>
            Gate.CompleteComputedProof(result, cancellation.Token, elapsedMilliseconds));
        Assert.False(result.Certified);
    }

    private static Gate.Diagnostics CompleteEvidence(bool retainedEnclosures = false)
        => new()
        {
            GapComputed = true,
            GapPaths = 0,
            PhysicalCertified = true,
            SourcePoints = 200_000,
            PhysicalChords = 8,
            SourceCertificate = new SourceProof.Diagnostics
            {
                InputPoints = 12,
                MainTriangles = 4,
                TriangleTasks = 8,
                CertifiedPieces = retainedEnclosures ? 3 : 4,
                FinalResidualEdges = 0,
                OriginalOwnerRegistryEnforced = true,
                OriginalSiteRegistryEdges = 4,
                Certified = !retainedEnclosures,
                SupportOutsideEnclosuresCertified = retainedEnclosures,
                UnresolvedPieces = retainedEnclosures ? 1 : 0,
                Reason = retainedEnclosures
                    ? "complete-signed-support-in-original-capsules-or-recorded-enclosures"
                    : "exact-signed-chain-and-complete-finite-capsule-support",
            },
        };
}
