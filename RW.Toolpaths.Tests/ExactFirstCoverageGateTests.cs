using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling.Certification;
using System.Numerics;
using System.Reflection;

namespace RW.Toolpaths.Tests;

[Collection("Local original medial diagnostic controls")]
public sealed class ExactFirstCoverageGateTests
{
    [Fact]
    public void CompleteSignedSourceBandFitsFiniteOriginalCapsules()
    {
        var source = new Paths64 { Rectangle(0, 0, 1000, 1000) };
        var centers = new Paths64 { Rectangle(100, 100, 900, 900) };
        var index = new PolygonOps.BoundarySegmentIndex(source);
        Assert.True(ExactFirstCoverageGate.CertifyCenterEnclosure(source, centers, index, 100,
            CancellationToken.None, out var proof), proof.Reason);
        Assert.True(proof.Certified);
        Assert.Equal(0, proof.ResidualEdges);
    }

    [Fact]
    public void ExactOriginalEdgeHintAvoidsAnUnrelatedNearestChoice()
    {
        var source = new Paths64 { Rectangle(0, 0, 1000, 1000) };
        var support = new Paths64 { new Path64 { new(0, 0), new(1000, 0), new(500, 9) } };
        int choices = 0;
        (Point64, Point64) PoorChoice(Point64 point)
        { choices++; return (new(0, 1000), new(1000, 1000)); }
        Assert.True(SignedBoundaryCapsuleCertificate.TryCertify(support, PoorChoice, 10,
            CancellationToken.None, out var proof, originalBoundary: source), proof.Reason);
        Assert.Equal(0, choices);
        Assert.Equal(1, proof.OriginalEdgeHintCertificates);
        Assert.Equal(1, proof.MaximumPendingTasks);
        Assert.Equal(4, proof.OriginalHintEdges);
    }

    [Fact]
    public void OriginalEdgeHintCannotExtendItsFiniteEndpointCapsule()
    {
        var source = new Paths64 { Rectangle(0, -100, 10, 0) };
        var support = new Paths64 { new Path64 { new(0, 0), new(10, 0), new(20, 1) } };
        Assert.False(SignedBoundaryCapsuleCertificate.TryCertify(support,
            _ => (new Point64(0, 0), new Point64(10, 0)), 1,
            CancellationToken.None, out var proof, 2000, originalBoundary: source));
        Assert.True(proof.OriginalEdgeHintTrials > 0);
        Assert.Equal(0, proof.OriginalEdgeHintCertificates);
        Assert.True(proof.InheritedSiteTrials > 0);
        Assert.InRange(proof.MaximumPendingTasks, 1, 49);
    }

    [Fact]
    public void ExactSourceHintsAndNearestChoicesCertifyTheSameCompleteAnnularBand()
    {
        var source = new Paths64 { Rectangle(0, 0, 1000, 1000) };
        var support = new Paths64(source) { Rectangle(100, 100, 900, 900, true) };
        var index = new PolygonOps.BoundarySegmentIndex(source);
        Assert.True(SignedBoundaryCapsuleCertificate.TryCertify(support, index.NearestSegmentForCertificate,
            100, CancellationToken.None, out var reference, originalBoundary: source, useOriginalEdgeHints: false), reference.Reason);
        Assert.True(SignedBoundaryCapsuleCertificate.TryCertify(support, index.NearestSegmentForCertificate,
            100, CancellationToken.None, out var hinted, originalBoundary: source), hinted.Reason);
        Assert.Equal(reference.MainTriangles, hinted.MainTriangles);
        Assert.Equal(reference.CorrectionTriangles, hinted.CorrectionTriangles);
        Assert.Equal(0, hinted.ResidualEdges);
        Assert.True(hinted.OriginalEdgeHintTrials > 0);
        Assert.True(hinted.OriginalSiteChoices <= reference.OriginalSiteChoices);
        Assert.InRange(hinted.MaximumPendingTasks, 1, 49);
    }

    [Fact]
    public void CanonicalSourceHintsStillRejectADeletedExactWidthStem()
    {
        var source = new Paths64 { Rectangle(0, 0, 200, 1000) };
        var index = new PolygonOps.BoundarySegmentIndex(source);
        Assert.False(SignedBoundaryCapsuleCertificate.TryCertify(source, index.NearestSegmentForCertificate,
            99, CancellationToken.None, out var proof, 2000, originalBoundary: source));
        Assert.False(proof.Certified);
        Assert.True(proof.InheritedSiteTrials > 0);
        Assert.InRange(proof.MaximumPendingTasks, 1, 49);
    }

    [Fact]
    public void OmittingUntrustedIntersectionRepairPreservesTheExactAnnularCertificate()
    {
        var source = new Paths64 { Rectangle(0, 0, 1000, 1000) };
        var support = new Paths64(source) { Rectangle(100, 100, 900, 900, true) };
        var index = new PolygonOps.BoundarySegmentIndex(source);
        Assert.True(SignedBoundaryCapsuleCertificate.TryCertify(support, index.NearestSegmentForCertificate,
            100, CancellationToken.None, out var reference, originalBoundary: source, repairIntersections: true), reference.Reason);
        Assert.True(SignedBoundaryCapsuleCertificate.TryCertify(support, index.NearestSegmentForCertificate,
            100, CancellationToken.None, out var uncheckedAid, originalBoundary: source), uncheckedAid.Reason);
        Assert.False(reference.UntrustedIntersectionRepairSkipped);
        Assert.True(uncheckedAid.UntrustedIntersectionRepairSkipped);
        Assert.Equal(reference.MainTriangles, uncheckedAid.MainTriangles);
        Assert.Equal(reference.CorrectionTriangles, uncheckedAid.CorrectionTriangles);
        Assert.Equal(reference.ResidualEdges, uncheckedAid.ResidualEdges);
        Assert.Equal(reference.CertifiedPieces, uncheckedAid.CertifiedPieces);
        Assert.Equal(reference.RefinedPieces, uncheckedAid.RefinedPieces);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CrossingUntrustedBoundaryCannotBypassCompleteOriginalCapsuleSupport(bool repair)
    {
        var crossing = new Paths64 { new Path64 { new(0, 0), new(100, 100), new(0, 100), new(100, 0) } };
        Assert.False(SignedBoundaryCapsuleCertificate.TryCertify(crossing,
            _ => (new Point64(0, 0), new Point64(100, 0)), 1,
            CancellationToken.None, out var proof, 2000, repairIntersections: repair, originalBoundary: crossing));
        Assert.False(proof.Certified);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SkippingUntrustedRepairCannotAuthorizeAFilledProtectedHole(bool repair)
    {
        var source = new Paths64 { Rectangle(0, 0, 1000, 1000), Rectangle(300, 300, 700, 700, true) };
        var support = new Paths64(source) { Rectangle(100, 100, 900, 900, true) };
        var index = new PolygonOps.BoundarySegmentIndex(source);
        Assert.False(SignedBoundaryCapsuleCertificate.TryCertify(support, index.NearestSegmentForCertificate,
            100, CancellationToken.None, out var proof, 2000, originalBoundary: source, repairIntersections: repair));
        Assert.False(proof.Certified);
    }

    [Fact]
    public void ExactWidthLineValuedStemCannotBeDeletedByAnEmptyCenterProposal()
    {
        var source = new Paths64 { Rectangle(0, 0, 200, 1000) };
        Assert.False(ExactFirstCoverageGate.CertifyCenterEnclosure(source, new Paths64(),
            new PolygonOps.BoundarySegmentIndex(source), 99, CancellationToken.None, out var proof));
        Assert.False(proof.Certified);
    }

    [Fact]
    public void DeletedDisconnectedLegalCenterComponentFailsCompleteSourceSupport()
    {
        var source = new Paths64 { Rectangle(0, 0, 1000, 1000), Rectangle(3000, 0, 4000, 1000) };
        var complement = new Paths64(source) { Rectangle(100, 100, 900, 900, reverse: true) };
        var index = new PolygonOps.BoundarySegmentIndex(source);
        Assert.False(SignedBoundaryCapsuleCertificate.TryCertify(complement,
            index.NearestSegmentForCertificate, 100, CancellationToken.None, out var proof, 2000, originalBoundary: source));
        Assert.False(proof.Certified);
    }

    [Fact]
    public void ASourceHoleFilledByTheProposedCenterCannotPassItsNegativeSupport()
    {
        var source = new Paths64 { Rectangle(0, 0, 1000, 1000), Rectangle(300, 300, 700, 700, true) };
        var complement = new Paths64(source) { Rectangle(100, 100, 900, 900, true) };
        var index = new PolygonOps.BoundarySegmentIndex(source);
        Assert.False(SignedBoundaryCapsuleCertificate.TryCertify(complement,
            index.NearestSegmentForCertificate, 100, CancellationToken.None, out var proof, 2000, originalBoundary: source));
        Assert.False(proof.Certified);
    }

    [Fact]
    public void BoundaryCapsulesIncludeTheFilledCenterAndItsEntireCutterFootprint()
    {
        var centers = new Paths64 { Rectangle(0, 0, 1000, 1000) };
        var footprint = new Paths64 { Rectangle(-110, -110, 1110, 1110) };
        Assert.True(ExactFirstCoverageGate.CertifyPhysicalEnclosure(centers, footprint,
            100 / PathUtils.Scale, CancellationToken.None, out int chords, out int enclosed));
        Assert.Equal(4, chords);
        Assert.Equal(0, enclosed);
    }

    [Fact]
    public void AWholeUncrossedFootprintHoleInsideTheCenterFailsEnclosure()
    {
        var centers = new Paths64 { Rectangle(0, 0, 1000, 1000) };
        var footprint = new Paths64 { Rectangle(-110, -110, 1110, 1110), Rectangle(400, 400, 600, 600, true) };
        Assert.False(ExactFirstCoverageGate.CertifyPhysicalEnclosure(centers, footprint,
            100 / PathUtils.Scale, CancellationToken.None, out int chords, out int enclosed));
        Assert.Equal(4, chords); // All boundary capsules passed before the hole test.
        Assert.Equal(4, enclosed);
    }

    [Fact]
    public void AnOmittedCenterComponentFailsPhysicalEnclosure()
    {
        var centers = new Paths64 { Rectangle(0, 0, 1000, 1000), Rectangle(3000, 0, 4000, 1000) };
        var footprint = new Paths64 { Rectangle(-110, -110, 1110, 1110) };
        Assert.False(ExactFirstCoverageGate.CertifyPhysicalEnclosure(centers, footprint,
            100 / PathUtils.Scale, CancellationToken.None, out _, out _));
    }

    [Fact]
    public void ACenterBoundaryCutterDiscCrossingTheOuterCandidateFails()
    {
        var centers = new Paths64 { Rectangle(0, 0, 1000, 1000) };
        var footprint = new Paths64 { Rectangle(-99, -110, 1110, 1110) };
        Assert.False(ExactFirstCoverageGate.CertifyPhysicalEnclosure(centers, footprint,
            100 / PathUtils.Scale, CancellationToken.None, out _, out _));
    }

    [Fact]
    public void CapsuleAuthorityUsesFiniteEndpointsInsteadOfAnInfiniteSupportingLine()
    {
        var support = new Paths64 { Rectangle(1000, -1, 1002, 1) };
        Assert.False(SignedBoundaryCapsuleCertificate.TryCertify(support,
            _ => (new Point64(-10, 0), new Point64(10, 0)), 5,
            CancellationToken.None, out var proof, 1000, originalBoundary: new Paths64 { new Path64 { new(-10,0),new(10,0) } }));
        Assert.False(proof.Certified);
    }

    [Fact]
    public void ArbitraryTriangulationCannotExceedTheExactCapsuleWorkBudget()
    {
        var support = new Paths64 { Rectangle(0, 0, 1000, 1000) };
        Assert.False(SignedBoundaryCapsuleCertificate.TryCertify(support,
            _ => (new Point64(0, 0), new Point64(1000, 0)), 10,
            CancellationToken.None, out var proof, maximumTriangleTasks: 0, originalBoundary: support));
        Assert.Equal("capsule-work-budget", proof.Reason);
    }

    [Fact]
    public void CancellationPropagatesRatherThanBecomingAQualityFallback()
    {
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => SignedBoundaryCapsuleCertificate.TryCertify(
            new Paths64 { Rectangle(0, 0, 1000, 1000) }, _ => default,
            100, cancellation.Token, out _));
    }

    [Fact]
    public void CoordinateBoundRejectsInsteadOfWrappingExactChainArithmetic()
    {
        var support = new Paths64 { Rectangle(1L << 30, 0, (1L << 30) + 100, 100) };
        Assert.False(SignedBoundaryCapsuleCertificate.TryCertify(support, _ => default,
            100, CancellationToken.None, out var proof));
        Assert.Equal("input-or-integer-bound", proof.Reason);
    }

    [Fact]
    public void UntrustedOutputMustKeepEveryVertexAndTriangleInsideTheIntegerProofRange()
    {
        Assert.True(SignedBoundaryCapsuleCertificate.IsBoundedProposedTriangulation(new Paths64 {
            new Path64 { new(-(1L << 30) + 1, 0), new(0, (1L << 30) - 1), new(1, 0) } }));
        Assert.False(SignedBoundaryCapsuleCertificate.IsBoundedProposedTriangulation(new Paths64 {
            new Path64 { new(0, 0), new(1L << 30, 1), new(1, 0) } }));
        Assert.False(SignedBoundaryCapsuleCertificate.IsBoundedProposedTriangulation(new Paths64 {
            new Path64 { new(0, 0), new(1, -(1L << 30)), new(1, 0) } }));
        Assert.False(SignedBoundaryCapsuleCertificate.IsBoundedProposedTriangulation(new Paths64 {
            Rectangle(0, 0, 100, 100) }));
    }

    [Fact]
    public void InternalDeadlineCheckpointInterruptsWorkWithoutSwallowingItsBudgetException()
    {
        int calls = 0;
        var path = new Path64(Enumerable.Range(0, 1024).Select(i => new Point64(
            (long)Math.Round(10000 * Math.Cos(i * 2 * Math.PI / 1024)),
            (long)Math.Round(10000 * Math.Sin(i * 2 * Math.PI / 1024)))));
        Assert.Throws<DeadlineProbeException>(() => SignedBoundaryCapsuleCertificate.TryCertify(
            new Paths64 { path }, _ => (new Point64(-20000, 0), new Point64(20000, 0)),
            20000, CancellationToken.None, out _, workCheckpoint: () => {
                if (++calls == 2) throw new DeadlineProbeException(); }, originalBoundary: new Paths64 { path }));
        Assert.Equal(2, calls);
    }

    [Fact]
    public void RangeProvenMultiplyAndCheckedLargePathAgreeWithBigInteger()
    {
        var values = new Int128[] { long.MinValue, long.MaxValue, -1, 0, 1, (Int128)1 << 80, -((Int128)1 << 90) };
        foreach (Int128 a in values)
        foreach (Int128 b in values)
        {
            BigInteger expected = (BigInteger)a * (BigInteger)b;
            if (expected < (BigInteger)Int128.MinValue || expected > (BigInteger)Int128.MaxValue)
                Assert.Throws<OverflowException>(() => CheckedInt128.Multiply(a, b));
            else Assert.Equal((Int128)expected, CheckedInt128.Multiply(a, b));
        }
        Assert.Throws<OverflowException>(() => CheckedInt128.Multiply(Int128.MinValue, -1));
    }

    [Theory]
    [InlineData(99, true)]
    [InlineData(100, true)]
    [InlineData(101, false)]
    public void LargeDyadicSquaredCrossFallsBackToExactBigInteger(long height, bool expected)
    {
        const int shift = 48;
        Type pointType = typeof(SignedBoundaryCapsuleCertificate).GetNestedType("DyadicPoint", BindingFlags.NonPublic)!;
        object point = Activator.CreateInstance(pointType, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            null, new object[] { (Int128)0, (Int128)height << shift }, null)!;
        var site = (new Point64(-(1L << 29), 0), new Point64(1L << 29, 0));
        var proof = new SignedBoundaryCapsuleCertificate.Diagnostics();
        MethodInfo capsule = typeof(SignedBoundaryCapsuleCertificate).GetMethod("Capsule", BindingFlags.Static | BindingFlags.NonPublic)!;
        bool actual = (bool)capsule.Invoke(null, new object[] { point, site, 100L, shift, proof })!;
        Assert.Equal(expected, actual);
        Assert.True(proof.BigIntegerFallbacks > 0);
    }

    [Theory]
    [InlineData(156057.59765625, 10227128571L)]
    [InlineData(104, 6553595L)]
    public void DyadicSourceRadiusStaysStrictlyBelowTraceEligibilityAndArithmeticReserve(double nativeFirstInset, long expected)
    {
        long numerator = ExactFirstCoverageGate.StrictSourceRadiusForTraceNative(nativeFirstInset / PathUtils.Scale);
        Assert.Equal(expected, numerator);
        double radius = numerator / (double)(1L << ExactFirstCoverageGate.SourceRadiusFractionBits);
        double lowerThreshold = nativeFirstInset - 4 - ExactFirstCoverageGate.TraceDistanceArithmeticReserveTicks;
        Assert.True(radius < lowerThreshold);
        Assert.InRange(lowerThreshold - radius, double.Epsilon, 2d / 65536);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(25)]
    public void UnsupportedRadiusDenominatorCannotWrapExactCapsuleArithmetic(int fractionBits)
    {
        Assert.False(SignedBoundaryCapsuleCertificate.TryCertify(
            new Paths64 { Rectangle(0, 0, 100, 100) }, _ => default, 100,
            CancellationToken.None, out var proof, radiusFractionBits: fractionBits));
        Assert.Equal("input-or-integer-bound", proof.Reason);
    }

    [Theory]
    [InlineData(-1, true)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    public void FractionalRadiusLargeDyadicBoundaryMatchesAnIndependentBigIntegerCapsule(int quantumOffset, bool expected)
    {
        const int shift = 48, fractionBits = 16;
        const long numerator = 123 * 65536 + 32768;
        Int128 scale = (Int128)1 << shift;
        var site = (new Point64(-(1L << 29), -(1L << 29)), new Point64(1L << 29, -(1L << 29)));
        Int128 x = 0, y = (Int128)site.Item1.Y * scale + ((Int128)(numerator + quantumOffset) << (shift - fractionBits));
        var proof = new SignedBoundaryCapsuleCertificate.Diagnostics();
        bool actual = InvokeDyadicCapsule(x, y, site, numerator, shift, fractionBits, proof);
        Assert.Equal(ReferenceDyadicCapsule(x, y, site, numerator, shift, fractionBits), actual);
        Assert.Equal(expected, actual);
        Assert.True(proof.BigIntegerFallbacks > 0);
    }

    [Theory]
    [InlineData(81919, false)]
    [InlineData(81920, true)]
    public void FractionalRadiusKeepsFiniteEndpointAndZeroLengthCapsulesExact(long numerator, bool expected)
    {
        const int shift = 48, fractionBits = 16;
        Int128 scale = (Int128)1 << shift;
        var endpointSite = (new Point64(-5, 0), new Point64(5, 0));
        Assert.Equal(expected, InvokeDyadicCapsule(6 * scale, 3 * scale / 4, endpointSite,
            numerator, shift, fractionBits, new()));
        var pointSite = (new Point64(0, 0), new Point64(0, 0));
        Assert.Equal(expected, InvokeDyadicCapsule(3 * scale / 4, scale, pointSite,
            numerator, shift, fractionBits, new()));
    }

    [Fact]
    public void NonnegativeRangeFastProductAndCheckedBoundaryCasesAgreeWithBigInteger()
    {
        var multiply = typeof(SignedBoundaryCapsuleCertificate).GetMethod("MultiplyNonnegative", BindingFlags.NonPublic | BindingFlags.Static)!;
        var values = new Int128[] { 0, 1, ((Int128)1 << 63) - 1, (Int128)1 << 63,
            (Int128)1 << 64, (Int128)1 << 100, Int128.MaxValue, -1, Int128.MinValue };
        foreach (Int128 a in values)
        foreach (Int128 b in values)
        {
            BigInteger expected = (BigInteger)a * (BigInteger)b;
            if (expected < (BigInteger)Int128.MinValue || expected > (BigInteger)Int128.MaxValue)
            {
                var failure = Assert.Throws<TargetInvocationException>(() => multiply.Invoke(null, new object[] { a, b }));
                Assert.IsType<OverflowException>(failure.InnerException);
            }
            else Assert.Equal((Int128)expected, (Int128)multiply.Invoke(null, new object[] { a, b })!);
        }
    }

    [Fact]
    public void FractionalSourceRadiusStillRejectsADeletedExactWidthStem()
    {
        var source = new Paths64 { Rectangle(0, 0, 200, 1000) };
        var index = new PolygonOps.BoundarySegmentIndex(source);
        Assert.False(ExactFirstCoverageGate.CertifyCenterEnclosure(source, new Paths64(), index,
            100 * 65536L - 1, CancellationToken.None, out var proof, radiusFractionBits: 16));
        Assert.False(proof.Certified);
    }

    private static bool InvokeDyadicCapsule(Int128 x, Int128 y, (Point64, Point64) site,
        long numerator, int shift, int fractionBits, SignedBoundaryCapsuleCertificate.Diagnostics proof)
    {
        Type pointType = typeof(SignedBoundaryCapsuleCertificate).GetNestedType("DyadicPoint", BindingFlags.NonPublic)!;
        object point = Activator.CreateInstance(pointType, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            null, new object[] { x, y }, null)!;
        var method = typeof(SignedBoundaryCapsuleCertificate).GetMethod("CapsuleWithDyadicRadius", BindingFlags.Static | BindingFlags.NonPublic)!;
        return (bool)method.Invoke(null, new object[] { point, site, numerator, shift, proof, fractionBits })!;
    }

    private static bool ReferenceDyadicCapsule(Int128 x, Int128 y, (Point64, Point64) site,
        long numerator, int shift, int fractionBits)
    {
        BigInteger scale = BigInteger.One << shift, denominator = BigInteger.One << fractionBits;
        BigInteger dx = (BigInteger)site.Item2.X - site.Item1.X, dy = (BigInteger)site.Item2.Y - site.Item1.Y;
        BigInteger wx = (BigInteger)x - site.Item1.X * scale, wy = (BigInteger)y - site.Item1.Y * scale;
        BigInteger length = dx * dx + dy * dy, projection = wx * dx + wy * dy;
        BigInteger bound = (BigInteger)numerator * numerator * scale * scale;
        if (projection <= 0 || length.IsZero) return (wx * wx + wy * wy) * denominator * denominator <= bound;
        if (projection >= length * scale)
        {
            wx = (BigInteger)x - site.Item2.X * scale; wy = (BigInteger)y - site.Item2.Y * scale;
            return (wx * wx + wy * wy) * denominator * denominator <= bound;
        }
        BigInteger cross = dx * wy - dy * wx;
        return cross * cross * denominator * denominator <= bound * length;
    }

    [Fact]
    public void GateAuthorityUsesCapturedPocketOptionAndGenericDefaultIsOff()
    {
        var generic = new RW.Toolpaths.Milling.RingOffsetOptions(1,1);
        Assert.False(ExactFirstCoverageGate.Enabled(generic));
        Assert.False(ExactFirstCoverageGate.Enabled(generic with { ExactFirstCoverageProposals = true }));
        var captured = generic with { ExactFirstCoverageProposals = true, ExactFirstCoverageModeCaptured = true };
        Assert.True(ExactFirstCoverageGate.Enabled(captured));
        Assert.False(ExactFirstCoverageGate.Enabled(captured with { ExactFirstCoverageProposals = false }));
        Assert.Equal(.003 + 4 / PathUtils.Scale, ExactFirstCoverageGate.CoverageReserve);
    }

    private static Path64 Rectangle(long x0, long y0, long x1, long y1, bool reverse = false)
    {
        var path = new Path64 { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
        if (reverse) path.Reverse(); return path;
    }
    private sealed class DeadlineProbeException : Exception { }
}
