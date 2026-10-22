using Clipper2Lib;
using RW.Toolpaths.Milling.Certification;
using System.Numerics;

namespace RW.Toolpaths.Tests;

public sealed class ExactSignedBoundaryDecompositionTests
{
    [Theory]
    [InlineData("hole")]
    [InlineData("island")]
    [InlineData("exterior-notch")]
    [InlineData("rounded-sliver")]
    public void UntrustedAidCannotChangeTheOriginalSignedWinding(string scenario)
    {
        var outer = Rectangle(0, 0, 20, 20);
        Paths64 source = scenario switch
        {
            "hole" => new() { outer, Rectangle(6, 6, 14, 14, reverse: true) },
            "island" => new() { outer, Rectangle(25, 3, 30, 9) },
            "exterior-notch" => new() { new Path64 { new(0, 0), new(20, 0), new(20, 6), new(6, 6), new(6, 20), new(0, 20) } },
            _ => new() { new Path64 { new(0, 0), new(11, 0), new(10, 10), new(0, 11) } },
        };
        Paths64 aid = scenario == "rounded-sliver" ? new() { Rectangle(0, 0, 11, 11) } : new() { outer };
        var originalPoints = source.SelectMany(path => path).ToArray();
        Assert.True(ExactSignedBoundaryDecomposition.TryCreate(source, default, null,
            out var triangles, out var proof, triangulationAidBoundary: aid), proof.Reason);
        Assert.True(proof.Created); Assert.True(proof.UntrustedIntersectionRepairSkipped);
        Assert.True(proof.CorrectionTriangles > 0);
        Assert.Equal(proof.MainTriangles + proof.CorrectionTriangles, triangles.Length);
        Assert.Equal(0, proof.FinalResidualEdges);
        Assert.Equal(originalPoints, source.SelectMany(path => path).ToArray());
        if (scenario == "island") Assert.Contains(triangles.Skip(proof.MainTriangles), triangle => Cross(triangle.A, triangle.B, triangle.C).Sign > 0);
        else Assert.Contains(triangles.Skip(proof.MainTriangles), triangle => Cross(triangle.A, triangle.B, triangle.C).Sign < 0);
        AssertOriginalWinding(source, triangles, -2, 32, -2, 22);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CollinearSourceSplitsRemainAnExactChainWithoutInventingCorrections(bool useDelaunay)
    {
        var source = new Paths64 { new Path64 { new(0, 0), new(6, 0), new(12, 0), new(12, 12), new(0, 12), new(0, 6) } };
        Assert.True(ExactSignedBoundaryDecomposition.TryCreate(source, default, null,
            out var triangles, out var proof, useDelaunay: useDelaunay), proof.Reason);
        Assert.Equal(0, proof.ResidualEdges); Assert.Equal(0, proof.CorrectionTriangles);
        AssertOriginalWinding(source, triangles, -1, 13, -1, 13);
    }

    [Fact]
    public void EachConsumerOwnsAFreshOrderedTriangleSnapshot()
    {
        var source = new Paths64 { Rectangle(-3, -2, 11, 9) };
        Assert.True(ExactSignedBoundaryDecomposition.TryCreate(source, default, null, out var first, out var firstProof), firstProof.Reason);
        Assert.True(ExactSignedBoundaryDecomposition.TryCreate(source, default, null, out var second, out var secondProof), secondProof.Reason);
        Assert.NotSame(first, second); Assert.Equal(first, second);
        var original = second[0]; first[0] = default;
        Assert.Equal(original, second[0]);
    }

    [Fact]
    public void SignedCorrectionsConsumeTheSameTotalTriangleBudgetAsMainTriangles()
    {
        var source = new Paths64 { Rectangle(0, 0, 4, 4) };
        var main = new SignedTriangle(new(0, 0), new(4, 0), new(4, 4));
        var residual = ExactSignedBoundaryDecomposition.Residual(source, new[] { main }, default, null);
        Assert.NotEmpty(residual);
        var denied = new List<SignedTriangle> { main };
        Assert.False(ExactSignedBoundaryDecomposition.AppendSignedCorrections(residual, denied, default,
            null, out int deniedCorrections, out bool exhausted, maximumTriangles: 1));
        Assert.True(exhausted); Assert.Equal(0, deniedCorrections); Assert.Single(denied);
        var permitted = new List<SignedTriangle> { main };
        Assert.True(ExactSignedBoundaryDecomposition.AppendSignedCorrections(residual, permitted, default,
            null, out int corrections, out exhausted, maximumTriangles: 2));
        Assert.False(exhausted); Assert.Equal(1, corrections); Assert.Equal(2, permitted.Count);
        Assert.Empty(ExactSignedBoundaryDecomposition.Residual(source, permitted, default, null));
        AssertOriginalWinding(source, permitted.ToArray(), -1, 5, -1, 5);
    }

    [Fact]
    public void OpenResidualChainCannotCreateADecomposition()
    {
        var triangles = new List<SignedTriangle>();
        Assert.False(ExactSignedBoundaryDecomposition.AppendSignedCorrections(
            new[] { (new Point64(0, 0), new Point64(8, 0)), (new Point64(8, 0), new Point64(8, 8)) },
            triangles, default, null, out int corrections, out bool exhausted));
        Assert.Equal(0, corrections); Assert.False(exhausted); Assert.Empty(triangles);
    }

    [Fact]
    public void InputAndAidDomainRefusalsPublishNoPartialTriangles()
    {
        Assert.False(ExactSignedBoundaryDecomposition.TryCreate(new Paths64(), default, null, out var triangles, out var proof));
        Assert.Empty(triangles); Assert.False(proof.Created); Assert.Equal("input-or-integer-bound", proof.Reason);
        var bad = new Paths64 { Rectangle(1L << 30, 0, (1L << 30) + 4, 4) };
        Assert.False(ExactSignedBoundaryDecomposition.TryCreate(bad, default, null, out triangles, out proof));
        Assert.Empty(triangles); Assert.Equal("input-or-integer-bound", proof.Reason);
        Assert.False(ExactSignedBoundaryDecomposition.TryCreate(new() { Rectangle(0, 0, 4, 4) }, default,
            null, out triangles, out proof, triangulationAidBoundary: bad));
        Assert.Empty(triangles); Assert.Equal("triangulation-aid-input-or-integer-bound", proof.Reason);
    }

    [Fact]
    public void CancellationAndDeadlineRemainInterruptionsInsteadOfDecompositionRefusals()
    {
        var source = new Paths64 { Rectangle(0, 0, 8, 8) };
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => ExactSignedBoundaryDecomposition.TryCreate(
            source, cancellation.Token, null, out _, out _));
        int checkpoints = 0;
        Assert.Throws<DeadlineProbeException>(() => ExactSignedBoundaryDecomposition.TryCreate(source, default,
            () => { if (++checkpoints == 2) throw new DeadlineProbeException(); }, out _, out _));
        Assert.Equal(2, checkpoints);
    }

    [Theory]
    [InlineData(0, 0)]
    [InlineData(16, 16)]
    [InlineData(32, 24)]
    [InlineData(48, 24)]
    public void ReusableDyadicTriangleCapsuleAgreesWithIndependentFiniteSegmentOracle(int shift, int fractionBits)
    {
        var a = new Point64(-10, 0); var b = new Point64(10, 0);
        Point64[][] candidates =
        [
            [new(-5, 1), new(0, 2), new(5, 1)],
            [new(10, 0), new(11, 1), new(12, 0)],
            [new(10, 0), new(11, 1), new(13, 0)],
            [new(100, -1), new(101, 0), new(100, 1)],
        ];
        foreach (var triangle in candidates)
        {
            Int128 scale = (Int128)1 << shift;
            long radius = 2L << fractionBits;
            bool expected = triangle.All(point => FiniteCapsuleOracle((BigInteger)point.X * (BigInteger)scale,
                (BigInteger)point.Y * (BigInteger)scale, a, b, radius, shift, fractionBits));
            bool actual = SignedBoundaryCapsuleCertificate.ContainsDyadicTriangle(a, b,
                triangle[0].X * scale, triangle[0].Y * scale, triangle[1].X * scale, triangle[1].Y * scale,
                triangle[2].X * scale, triangle[2].Y * scale, shift, radius, fractionBits);
            Assert.Equal(expected, actual);
        }
    }

    [Fact]
    public void ReusableCapsuleKeepsBigIntegerAuthorityAtTheLargestDyadicScale()
    {
        var a = new Point64(-(1L << 30) + 1, 17); var b = new Point64((1L << 30) - 1, 23);
        const int shift = 48, fractionBits = 24;
        Int128 scale = (Int128)1 << shift;
        var vertices = new Point64[] { new(0, 20), new(1, 20), new(0, 21) };
        long radius = 2L << fractionBits;
        Assert.All(vertices, point => Assert.True(FiniteCapsuleOracle((BigInteger)point.X * (BigInteger)scale,
            (BigInteger)point.Y * (BigInteger)scale, a, b, radius, shift, fractionBits)));
        Assert.True(SignedBoundaryCapsuleCertificate.ContainsDyadicTriangle(a, b,
            0, 20 * scale, scale, 20 * scale, 0, 21 * scale, shift, radius, out int fallbacks, fractionBits));
        Assert.Equal(3, fallbacks);
    }

    [Theory]
    [InlineData(-1, 24, 1)]
    [InlineData(49, 24, 1)]
    [InlineData(0, -1, 1)]
    [InlineData(0, 25, 1)]
    [InlineData(0, 24, 0)]
    public void ReusableCapsuleRefusesUnsupportedDomains(int shift, int fractionBits, long radius)
        => Assert.False(SignedBoundaryCapsuleCertificate.ContainsDyadicTriangle(new(0, 0), new(4, 0),
            0, 0, 1, 0, 0, 1, shift, radius, fractionBits));

    private static void AssertOriginalWinding(Paths64 source, SignedTriangle[] triangles, int x0, int x1, int y0, int y1)
    {
        int tested = 0;
        for (int x = x0; x <= x1; x++)
        for (int y = y0; y <= y1; y++)
        {
            long qx = 2L * x + 1, qy = 2L * y + 1;
            if (source.Any(path => OnBoundary(path, qx, qy))
                || triangles.Any(triangle => OnBoundary(new Path64 { triangle.A, triangle.B, triangle.C }, qx, qy))) continue;
            int expected = source.Sum(path => Winding(path, qx, qy));
            int actual = triangles.Sum(triangle => Winding(new Path64 { triangle.A, triangle.B, triangle.C }, qx, qy));
            Assert.Equal(expected, actual); tested++;
        }
        Assert.True(tested > 20);
    }

    private static int Winding(Path64 path, long qx, long qy)
    {
        int winding = 0;
        for (int i = 0; i < path.Count; i++)
        {
            var a = path[i]; var b = path[(i + 1) % path.Count];
            BigInteger cross = ProbeCross(a, b, qx, qy);
            if (2 * a.Y <= qy && 2 * b.Y > qy && cross.Sign > 0) winding++;
            else if (2 * a.Y > qy && 2 * b.Y <= qy && cross.Sign < 0) winding--;
        }
        return winding;
    }

    private static bool OnBoundary(Path64 path, long qx, long qy)
    {
        for (int i = 0; i < path.Count; i++)
        {
            var a = path[i]; var b = path[(i + 1) % path.Count];
            if (ProbeCross(a, b, qx, qy).IsZero && qx >= 2 * Math.Min(a.X, b.X) && qx <= 2 * Math.Max(a.X, b.X)
                && qy >= 2 * Math.Min(a.Y, b.Y) && qy <= 2 * Math.Max(a.Y, b.Y)) return true;
        }
        return false;
    }

    private static BigInteger ProbeCross(Point64 a, Point64 b, long qx, long qy)
        => ((BigInteger)b.X - a.X) * (qy - 2 * a.Y) - ((BigInteger)b.Y - a.Y) * (qx - 2 * a.X);

    private static BigInteger Cross(Point64 a, Point64 b, Point64 c)
        => ((BigInteger)b.X - a.X) * ((BigInteger)c.Y - a.Y) - ((BigInteger)b.Y - a.Y) * ((BigInteger)c.X - a.X);

    private static bool FiniteCapsuleOracle(BigInteger x, BigInteger y, Point64 a, Point64 b,
        long numerator, int shift, int fractionBits)
    {
        BigInteger scale = BigInteger.One << shift, denominatorSquared = BigInteger.One << (2 * fractionBits);
        BigInteger dx = (BigInteger)b.X - a.X, dy = (BigInteger)b.Y - a.Y;
        BigInteger length = dx * dx + dy * dy, wx = x - a.X * scale, wy = y - a.Y * scale;
        BigInteger projection = wx * dx + wy * dy, radius = numerator * scale;
        if (length.IsZero || projection <= 0) return (wx * wx + wy * wy) * denominatorSquared <= radius * radius;
        if (projection >= length * scale)
        {
            BigInteger vx = x - b.X * scale, vy = y - b.Y * scale;
            return (vx * vx + vy * vy) * denominatorSquared <= radius * radius;
        }
        BigInteger cross = dx * wy - dy * wx;
        return cross * cross * denominatorSquared <= radius * radius * length;
    }

    private static Path64 Rectangle(long x0, long y0, long x1, long y1, bool reverse = false)
    {
        var path = new Path64 { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
        if (reverse) path.Reverse();
        return path;
    }

    private sealed class DeadlineProbeException : Exception { }
}
