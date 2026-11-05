using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths;

public sealed partial class BoostVoronoiProvider
{
    /// <summary>Original integer sites, under the COMPLETE source's unchanged conditioning.
    /// Selected segments are submitted individually; missing ring edges never become bridges.</summary>
    internal sealed class LocalSiteSet
    {
        internal readonly record struct Site(PointD A, PointD B, int Ordinal, bool Outer);
        internal IReadOnlyList<Site> Sites { get; }
        internal IReadOnlyList<PointD> Outer { get; }

        internal LocalSiteSet(IReadOnlyList<PointD> outer, IReadOnlyList<IReadOnlyList<PointD>> holes,
            CancellationToken token)
        {
            var sites = new List<Site>(outer.Count + holes.Sum(hole => hole.Count));
            Outer = RoundAndNormalizeRing(outer);
            Add(Outer, true);
            foreach (var hole in holes) Add(RoundAndNormalizeRing(hole), false);
            Sites = sites;
            void Add(IReadOnlyList<PointD> ring, bool isOuter)
            {
                if (ring.Count < 3) return;
                for (int i = 0; i < ring.Count; i++)
                {
                    if ((i & 4095) == 0) token.ThrowIfCancellationRequested();
                    sites.Add(new(ring[i], ring[(i + 1) % ring.Count], sites.Count, isOuter));
                }
            }
        }
    }

    internal sealed record LocalDiagram(IReadOnlyList<MedialSegment> Segments,
        IReadOnlyList<int> OriginalSiteOrdinals, int SourceSiteCount, double InfluenceUpperBound,
        double NumericalReserve, int EmittedEdges, int SkippedEdges);

    /// <summary>Nearest-site identity is unchanged throughout the padded query rectangle.
    /// Distance to one retained finite original segment bounds the nearest distance from above;
    /// a site's AABB distance bounds its influence from below. Equality always retains the site.</summary>
    internal static LocalDiagram? TryConstructLocalMedialAxis(LocalSiteSet source, RectD query,
        double maximumChordError, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        if (source.Sites.Count < 3 || maximumChordError <= 0) return null;
        long started = PerfLog.Start();
        double magnitude = Math.Max(1, source.Sites.Max(site => Math.Max(
            Math.Max(Math.Abs(site.A.x), Math.Abs(site.A.y)), Math.Max(Math.Abs(site.B.x), Math.Abs(site.B.y)))));
        double reserve = Math.Max(1e-7, magnitude * 1e-12);
        // A bounded chord meeting the real query can originate up to its error
        // outside it. Charge that tube before certifying omitted sites.
        double padding = maximumChordError + 8 + reserve;
        var influenceWindow = new RectD(Math.BitDecrement(query.left - padding), Math.BitDecrement(query.top - padding),
            Math.BitIncrement(query.right + padding), Math.BitIncrement(query.bottom + padding));
        var corners = new[] { new PointD(influenceWindow.left, influenceWindow.top),
            new PointD(influenceWindow.right, influenceWindow.top),
            new PointD(influenceWindow.right, influenceWindow.bottom),
            new PointD(influenceWindow.left, influenceWindow.bottom) };
        // Retaining the entire original outer ring encloses all material
        // queries. Infinite primary edges therefore cannot enter this window.
        var outerIndex = new PolygonOps.BoundarySegmentIndex(new Paths64
            { new Path64(source.Outer.Select(Quantize)) });
        if (corners.Any(corner => !outerIndex.IsInside(Quantize(corner)))
            || Enumerable.Range(0, 4).Any(i => !outerIndex.ContainsCapsule(
                Quantize(corners[i]), Quantize(corners[(i + 1) % 4]), 4)))
        {
            Console.WriteLine("[Offset.Terminal.LocalSites] result=fallback reason=query-not-strictly-inside-original-outer");
            return null;
        }
        PointD center = new((influenceWindow.left + influenceWindow.right) / 2,
            (influenceWindow.top + influenceWindow.bottom) / 2);
        int witness = 0;
        double nearestSquared = double.PositiveInfinity;
        for (int i = 0; i < source.Sites.Count; i++)
        {
            if ((i & 4095) == 0) token.ThrowIfCancellationRequested();
            var site = source.Sites[i];
            double distance = DistanceSquared(center, site.A, site.B);
            if (distance < nearestSquared) { nearestSquared = distance; witness = i; }
        }
        var witnessSite = source.Sites[witness];
        // Distance to a convex finite segment is convex. Its maximum on a
        // rectangle is bounded by the four corner values.
        double upper = Math.BitIncrement(corners.Max(corner => DistanceUpper(corner, witnessSite.A, witnessSite.B)) + reserve);
        var retained = new List<LocalSiteSet.Site>();
        for (int i = 0; i < source.Sites.Count; i++)
        {
            if ((i & 4095) == 0) token.ThrowIfCancellationRequested();
            var site = source.Sites[i];
            double dx = Math.Max(0, Math.Max(
                (OutwardInterval.Exact(Math.Min(site.A.x, site.B.x)) - OutwardInterval.Exact(influenceWindow.right)).Low,
                (OutwardInterval.Exact(influenceWindow.left) - OutwardInterval.Exact(Math.Max(site.A.x, site.B.x))).Low));
            double dy = Math.Max(0, Math.Max(
                (OutwardInterval.Exact(Math.Min(site.A.y, site.B.y)) - OutwardInterval.Exact(influenceWindow.bottom)).Low,
                (OutwardInterval.Exact(influenceWindow.top) - OutwardInterval.Exact(Math.Max(site.A.y, site.B.y))).Low));
            var lowerSquared = OutwardInterval.Exact(dx).Square() + OutwardInterval.Exact(dy).Square();
            double lower = Math.Max(0, Math.BitDecrement(Math.Sqrt(Math.Max(0, lowerSquared.Low))));
            if (site.Outer || lower <= upper + reserve) retained.Add(site);
        }
        if (retained.Count == source.Sites.Count) return null;
        var sites = retained.Select(site => new SegmentSiteData(site.A.x, site.A.y, site.B.x, site.B.y,
            new(site.A.x, site.A.y), new(site.B.x, site.B.y))).ToArray();
        int[] ax = retained.Select(site => (int)site.A.x).ToArray(), ay = retained.Select(site => (int)site.A.y).ToArray();
        int[] bx = retained.Select(site => (int)site.B.x).ToArray(), by = retained.Select(site => (int)site.B.y).ToArray();
        IntPtr handle = BoostVoronoiInterop.bv_construct(ax, ay, bx, by, retained.Count);
        if (handle == IntPtr.Zero) return null;
        try
        {
            token.ThrowIfCancellationRequested();
            int edgeCount = BoostVoronoiInterop.bv_edge_count(handle), vertexCount = BoostVoronoiInterop.bv_vertex_count(handle),
                cellCount = BoostVoronoiInterop.bv_cell_count(handle);
            var edges = new BoostVoronoiInterop.BvEdge[edgeCount];
            var vertices = new BoostVoronoiInterop.BvVertex[vertexCount];
            var cells = new BoostVoronoiInterop.BvCell[cellCount];
            BoostVoronoiInterop.bv_get_edges(handle, edges, edgeCount);
            BoostVoronoiInterop.bv_get_vertices(handle, vertices, vertexCount);
            BoostVoronoiInterop.bv_get_cells(handle, cells, cellCount);
            var cellSites = cells.Select(cell => MakeCellSite(cell, sites)).ToArray();
            var points = vertices.Select(vertex => new MedialAxisEdgeClassifier.EdgePoint(vertex.X, vertex.Y)).ToArray();
            var output = new List<MedialSegment>();
            int emitted = 0;
            // Stable ORIGINAL site owners order the local edges. A subset
            // diagram has different native indices; they are not advertised as
            // full-diagram command equivalence.
            var finite = Enumerable.Range(0, edgeCount).Where(i => edges[i].IsPrimary != 0
                && edges[i].IsInfinite == 0 && i < edges[i].TwinIndex)
                .OrderBy(i => Math.Min(retained[cells[edges[i].CellIndex].SourceIndex].Ordinal,
                    retained[cells[edges[i].TwinCellIndex].SourceIndex].Ordinal))
                .ThenBy(i => Math.Max(retained[cells[edges[i].CellIndex].SourceIndex].Ordinal,
                    retained[cells[edges[i].TwinCellIndex].SourceIndex].Ordinal)).ThenBy(i => i);
            foreach (int i in finite)
            {
                token.ThrowIfCancellationRequested();
                var e = edges[i];
                if (e.Vertex0Index < 0 || e.Vertex1Index < 0) return null;
                var edge = new MedialAxisEdgeClassifier.EdgeData(i, e.TwinIndex, true, false, false,
                    e.PrevIndex, e.NextIndex, points[e.Vertex0Index], points[e.Vertex1Index],
                    cellSites[e.CellIndex], cellSites[e.TwinCellIndex])
                { IsLinear = e.IsLinear != 0, IsCurved = e.IsCurved != 0 };
                if (!BoundedLocalMedialEmitter.TryEmit(edge, influenceWindow, maximumChordError, reserve, output, token))
                {
                    Console.WriteLine($"[Offset.Terminal.LocalSites] result=fallback reason=unresolved-finite-curve edge={i}");
                    return null;
                }
                emitted++;
            }
            PerfLog.Stop("Offset.Terminal.LocalDiagram", started,
                $"sourceSites={source.Sites.Count} retainedSites={retained.Count} edges={edgeCount} emittedEdges={emitted} "
                + $"segments={output.Count} upper={upper:R} reserve={reserve:R} chordError={maximumChordError:R} "
                + "conditioning=complete-original-source originalOwners=True fakeBridges=0 fullWinding=caller-authority");
            return new(output, retained.Select(site => site.Ordinal).ToArray(), source.Sites.Count,
                upper, reserve, emitted, 0);
        }
        finally { BoostVoronoiInterop.bv_destroy(handle); }

        static Point64 Quantize(PointD point) => new((long)Math.Round(point.x), (long)Math.Round(point.y));
    }

    private static double DistanceSquared(PointD point, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y, length = dx * dx + dy * dy;
        double t = length == 0 ? 0 : Math.Clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / length, 0, 1);
        double ex = point.x - a.x - t * dx, ey = point.y - a.y - t * dy;
        return ex * ex + ey * ey;
    }

    private static double DistanceUpper(PointD point, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y, length = dx * dx + dy * dy;
        double t = length == 0 ? 0 : Math.Clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / length, 0, 1);
        var fraction = OutwardInterval.Exact(t); var complement = OutwardInterval.Exact(1) - fraction;
        var x = OutwardInterval.Exact(a.x) * complement + OutwardInterval.Exact(b.x) * fraction;
        var y = OutwardInterval.Exact(a.y) * complement + OutwardInterval.Exact(b.y) * fraction;
        return ((OutwardInterval.Exact(point.x) - x).Square()
            + (OutwardInterval.Exact(point.y) - y).Square()).SqrtUpper();
    }
}
