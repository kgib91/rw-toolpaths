using System.IO.Compression;
using System.Numerics;
using System.Text.Json;
using Clipper2Lib;
using Point = RW.Toolpaths.NativeSegmentArrangement.Point;
using Segment = RW.Toolpaths.NativeSegmentArrangement.Segment;

namespace RW.Toolpaths.Tests;

public sealed class RationalNativeSegmentArrangementTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void RecordedContactsPreserveEveryOriginalSubchainExactly(int sign)
    {
        Point P(int x, int y) => new(sign * x, sign * y);
        Segment[] source = {
            new(P(4023800, -3149786), P(4027573, -3154784)),
            new(P(4025953, -3152638), P(4025952, -3152637)),
            new(P(4025952, -3152637), P(4025952, -3152628)),
        };
        var result = NativeSegmentArrangement.PrepareWide(source);
        Assert.Equal(1925, result.Scale);
        Assert.Equal(7, result.Atoms.Count);
        Assert.Equal(3, result.SplitOriginalSites);
        for (int owner = 0; owner < source.Length; owner++)
        {
            var s = source[owner];
            var atoms = result.Atoms.Where(a => a.OriginalOrdinal == owner || a.AdditionalOriginalOrdinals.Contains(owner)).ToArray();
            var cursor = new NativeSegmentArrangement.WidePoint(s.A.X * result.Scale, s.A.Y * result.Scale);
            BigInteger sum = 0;
            foreach (var atom in atoms)
            {
                Assert.Equal(cursor, atom.A);
                Assert.Equal((BigInteger)(s.B.X - s.A.X) * ((BigInteger)atom.B.Y - s.A.Y * result.Scale),
                    (BigInteger)(s.B.Y - s.A.Y) * ((BigInteger)atom.B.X - s.A.X * result.Scale));
                sum += (BigInteger)(atom.B.X - atom.A.X) * (s.B.X - s.A.X)
                    + (BigInteger)(atom.B.Y - atom.A.Y) * (s.B.Y - s.A.Y);
                cursor = atom.B;
            }
            Assert.Equal(new(s.B.X * result.Scale, s.B.Y * result.Scale), cursor);
            Assert.Equal(((BigInteger)(s.B.X - s.A.X) * (s.B.X - s.A.X)
                + (BigInteger)(s.B.Y - s.A.Y) * (s.B.Y - s.A.Y)) * result.Scale, sum);
        }
    }

    [Fact]
    public void ReversedDuplicateSubchainsRetainAllOwners()
    {
        Segment[] source = { new(new(0, 0), new(3, 3)), new(new(0, 2), new(3, 0)), new(new(3, 3), new(0, 0)) };
        var result = NativeSegmentArrangement.PrepareWide(source);
        Assert.Equal(5, result.Scale);
        Assert.Equal(4, result.Atoms.Count);
        Assert.Equal(2, result.DuplicateAtoms);
        Assert.All(result.Atoms.Where(a => a.OriginalOrdinal == 0), a => Assert.Contains(2, a.AdditionalOriginalOrdinals));
    }

    [Fact]
    public void CancellationCannotPublishAPartialWideArrangement()
    {
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => NativeSegmentArrangement.PrepareWide(
            new[] { new Segment(new(0, 0), new(3, 3)), new Segment(new(0, 2), new(3, 0)) }, cancellation.Token));
    }

    [Fact]
    public void ExcessiveExactDenominatorsFailBeforeNativeSubmission()
    {
        Segment[] source = { new(new(0, 0), new(int.MaxValue, int.MaxValue - 1)),
            new(new(0, int.MaxValue), new(int.MaxValue - 2, 0)) };
        var error = Assert.Throws<InvalidOperationException>(() => NativeSegmentArrangement.PrepareWide(source));
        Assert.Contains("rational-", error.Message);
        Assert.Contains("range", error.Message);
    }

    [Fact]
    public void CompleteCapturedSixtyDegreeBoundaryBuildsFiniteSourceMedial()
    {
        using var resource = GetType().Assembly.GetManifestResourceStream("RW.Toolpaths.Tests.Fixtures.ramp60-medial.json.gz")!;
        using var gzip = new GZipStream(resource, CompressionMode.Decompress);
        using var document = JsonDocument.Parse(gzip);
        var data = document.RootElement;
        PointD[] Ring(JsonElement r) => r.EnumerateArray().Select(p => new PointD(p[0].GetDouble(), p[1].GetDouble())).ToArray();
        var boundary = Ring(data.GetProperty("boundary"));
        var holes = data.GetProperty("holes").EnumerateArray().Select(r => (IReadOnlyList<PointD>)Ring(r)).ToArray();
        Assert.Equal(36467, boundary.Length + holes.Sum(r => r.Count));
        var result = BoostVoronoiProvider.CreateDefault().ConstructBoundedSourceMedialAxis(boundary, holes,
            data.GetProperty("tolerance").GetDouble(), data.GetProperty("maxRadius").GetDouble(),
            data.GetProperty("filteringAngle").GetDouble(), default, null, null, new(0, 0, PathUtils.Scale));
        Assert.NotEmpty(result);
        foreach (var segment in result)
            foreach (var p in new[] { segment.Point0, segment.Point1 })
            {
                Assert.True(double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Radius));
                Assert.True(p.Radius >= 0);
            }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(1925)]
    [InlineData(10000000000)]
    public void WideNativeConstructionPreservesAnIndependentIntegerDiagram(long scale)
    {
        int[] ax = { -500, 500, 500, -500, -100, -100, 100, 100 };
        int[] ay = { -300, -300, 300, 300, -50, 50, 50, -50 };
        int[] bx = { 500, 500, -500, -500, -100, 100, 100, -100 };
        int[] by = { -300, 300, 300, -300, 50, 50, -50, -50 };
        var reference = BoostVoronoiInterop.bv_construct(ax, ay, bx, by, ax.Length);
        var wide = BoostVoronoiInterop.bv_construct_wide(ax.Select(v => v * scale).ToArray(), ay.Select(v => v * scale).ToArray(),
            bx.Select(v => v * scale).ToArray(), by.Select(v => v * scale).ToArray(), ax.Length);
        Assert.NotEqual(IntPtr.Zero, reference);
        Assert.NotEqual(IntPtr.Zero, wide);
        try
        {
            Assert.Equal(BoostVoronoiInterop.bv_edge_count(reference), BoostVoronoiInterop.bv_edge_count(wide));
            Assert.Equal(BoostVoronoiInterop.bv_cell_count(reference), BoostVoronoiInterop.bv_cell_count(wide));
            int count = BoostVoronoiInterop.bv_vertex_count(reference);
            Assert.Equal(count, BoostVoronoiInterop.bv_vertex_count(wide));
            var original = new BoostVoronoiInterop.BvVertex[count];
            var actual = new BoostVoronoiInterop.BvVertex[count];
            BoostVoronoiInterop.bv_get_vertices(reference, original, count);
            BoostVoronoiInterop.bv_get_vertices(wide, actual, count);
            foreach (var vertex in original)
                Assert.Contains(actual, candidate => Math.Abs(candidate.X / scale - vertex.X) < 1e-6
                    && Math.Abs(candidate.Y / scale - vertex.Y) < 1e-6);
        }
        finally
        {
            BoostVoronoiInterop.bv_destroy(reference);
            BoostVoronoiInterop.bv_destroy(wide);
        }
    }
}
