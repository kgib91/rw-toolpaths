using Clipper2Lib;
using System.Globalization;

namespace RW.Toolpaths;

/// <summary>
/// <see cref="IMedialAxisProvider"/> backed by native Boost.Polygon.Voronoi
/// via P/Invoke to <c>boostvoronoi.dll</c> / <c>libboostvoronoi.so</c>.
///
///   integer segment-site Voronoi  ->  ClassifyEdges  ->
///   SelectInnerPrimaryTraversal   ->  TryCreateOutputSegments
/// </summary>
public sealed class BoostVoronoiProvider : IMedialAxisProvider
{
    public static BoostVoronoiProvider CreateDefault() => new();

    public IReadOnlyList<MedialSegment> ConstructMedialAxis(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance,
        double maxRadius,
        double filteringAngle = 3 * Math.PI / 4,
        bool useBigIntegers = true)
    {
        long t0 = PerfLog.Start();
        if (boundary is null) throw new ArgumentNullException(nameof(boundary));
        if (holes is null)    throw new ArgumentNullException(nameof(holes));
        if (boundary.Count < 3)
        {
            PerfLog.Stop("BoostVoronoiProvider.ConstructMedialAxis", t0, "boundary<3");
            return Array.Empty<MedialSegment>();
        }

        // Caller controls coordinate scale domain (e.g. e.up in ridge-pass()).
        var scaledBoundary = RoundAndNormalizeRing(boundary);
        var scaledHoles = new List<List<PointD>>(holes.Count);
        for (int i = 0; i < holes.Count; i++)
        {
            var ring = RoundAndNormalizeRing(holes[i]);
            if (ring.Count >= 3)
                scaledHoles.Add(ring);
        }

        // Build one SegmentSiteData per edge of each ring.
        // Low/High follow Boost's canonical y-then-x ordering so that
        // source_category maps back to the correct endpoint.
        var segSites = BuildSegmentSites(scaledBoundary, scaledHoles);
        int segCount = segSites.Count;
        if (segCount < 3)
        {
            PerfLog.Stop("BoostVoronoiProvider.ConstructMedialAxis", t0, "segments<3");
            return Array.Empty<MedialSegment>();
        }

        // Unpack to parallel int[] arrays for the C call.
        int[] ax = new int[segCount], ay = new int[segCount];
        int[] bx = new int[segCount], by = new int[segCount];
        for (int i = 0; i < segCount; i++)
        {
            var s = segSites[i];
            ax[i] = (int)s.Ax; ay[i] = (int)s.Ay;
            bx[i] = (int)s.Bx; by[i] = (int)s.By;
        }

        IntPtr handle = BoostVoronoiInterop.bv_construct(ax, ay, bx, by, segCount);
        if (handle == IntPtr.Zero)
        {
            PerfLog.Stop("BoostVoronoiProvider.ConstructMedialAxis", t0, "construct-handle-null");
            return Array.Empty<MedialSegment>();
        }

        try
        {
            var output = RunPipeline(handle, segSites, scaledBoundary, scaledHoles,
                                     tolerance, maxRadius, filteringAngle);
            PerfLog.Stop(
                "BoostVoronoiProvider.ConstructMedialAxis",
                t0,
                $"boundaryPts={boundary.Count} holes={holes.Count} segSites={segCount} out={output.Count}");
            return output;
        }
        finally
        {
            BoostVoronoiInterop.bv_destroy(handle);
        }
    }

    // -- Pipeline --------------------------------------------------------------

    private static IReadOnlyList<MedialSegment> RunPipeline(
        IntPtr handle,
        IReadOnlyList<SegmentSiteData> segSites,
        IReadOnlyList<PointD> scaledBoundary,
        IReadOnlyList<IReadOnlyList<PointD>> scaledHoles,
        double tolerance,
        double maxRadius,
        double filteringAngle)
    {
        long t0 = PerfLog.Start();
        int edgeCount   = BoostVoronoiInterop.bv_edge_count(handle);
        int vertexCount = BoostVoronoiInterop.bv_vertex_count(handle);
        int cellCount   = BoostVoronoiInterop.bv_cell_count(handle);

        var bvEdges    = new BoostVoronoiInterop.BvEdge[edgeCount];
        var bvVertices = new BoostVoronoiInterop.BvVertex[vertexCount];
        var bvCells    = new BoostVoronoiInterop.BvCell[cellCount];

        BoostVoronoiInterop.bv_get_edges(handle, bvEdges, edgeCount);
        BoostVoronoiInterop.bv_get_vertices(handle, bvVertices, vertexCount);
        BoostVoronoiInterop.bv_get_cells(handle, bvCells, cellCount);

        // Build CellSite lookup from Boost source_index + source_category.
        var cellSites = new MedialAxisEdgeClassifier.CellSite[cellCount];
        for (int i = 0; i < cellCount; i++)
            cellSites[i] = MakeCellSite(bvCells[i], segSites);

        // Build EdgeData[]; IsLinear/IsCurved are set post-construction.
        var edgeData = new MedialAxisEdgeClassifier.EdgeData[edgeCount];
        for (int i = 0; i < edgeCount; i++)
        {
            ref var e = ref bvEdges[i];
            bool isPrimary = e.IsPrimary != 0;
            edgeData[i] = new MedialAxisEdgeClassifier.EdgeData(
                Index:       i,
                TwinIndex:   e.TwinIndex,
                IsPrimary:   isPrimary,
                IsSecondary: !isPrimary,
                IsInfinite:  e.IsInfinite != 0,
                PrevIndex:   e.PrevIndex,
                NextIndex:   e.NextIndex,
                Vertex0: e.Vertex0Index >= 0
                    ? new MedialAxisEdgeClassifier.EdgePoint(
                          bvVertices[e.Vertex0Index].X,
                          bvVertices[e.Vertex0Index].Y)
                    : null,
                Vertex1: e.Vertex1Index >= 0
                    ? new MedialAxisEdgeClassifier.EdgePoint(
                          bvVertices[e.Vertex1Index].X,
                          bvVertices[e.Vertex1Index].Y)
                    : null,
                Cell:     e.CellIndex     >= 0 ? cellSites[e.CellIndex]     : cellSites[0],
                TwinCell: e.TwinCellIndex >= 0 ? cellSites[e.TwinCellIndex] : cellSites[0]
            )
            {
                IsLinear = e.IsLinear != 0,
                IsCurved = e.IsCurved != 0,
            };
        }

        bool debugEnabled = IsMedialDebugEnabled();

        MedialAxisEdgeClassifier.ClassifyEdges(edgeData, scaledBoundary, scaledHoles);

        List<int>? walkedUnfiltered = null;
        List<int> walked;
        int innerPrimaryCount = 0;

        if (debugEnabled)
        {
            innerPrimaryCount = edgeData.Count(e => e.Color == MedialAxisEdgeClassifier.Colors.InnerPrimary);
            walkedUnfiltered = MedialAxisEdgeClassifier.SelectInnerPrimaryTraversal(edgeData, Math.PI + 1e-6);
            walked = MedialAxisEdgeClassifier.SelectInnerPrimaryTraversal(edgeData, filteringAngle);
        }
        else
        {
            walked = MedialAxisEdgeClassifier.SelectInnerPrimaryTraversal(edgeData, filteringAngle);
        }

        if (debugEnabled)
        {
            var walkedSet = new HashSet<int>(walked);
            var angleFilteredIndices = walkedUnfiltered!
                .Where(idx => !walkedSet.Contains(idx))
                .ToList();
            var angleFilteredEdges = angleFilteredIndices
                .Select(idx => edgeData[idx])
                .ToList();

            int filteredLinear = angleFilteredEdges.Count(e => e.IsLinear);
            int filteredCurved = angleFilteredEdges.Count(e => e.IsCurved);
            int filteredPointPoint = angleFilteredEdges.Count(e => e.Cell.ContainsPoint && e.TwinCell.ContainsPoint);
            int filteredPointSegment = angleFilteredEdges.Count(e => e.Cell.ContainsPoint ^ e.TwinCell.ContainsPoint);
            int filteredSegmentSegment = angleFilteredEdges.Count(e => !e.Cell.ContainsPoint && !e.TwinCell.ContainsPoint);

            var filteredAngles = angleFilteredIndices
                .Select(idx => TryGetCellsAngleForDebug(edgeData[idx]))
                .Where(a => a.HasValue)
                .Select(a => a!.Value)
                .ToList();

            string angleStats = filteredAngles.Count > 0
                ? string.Create(
                    CultureInfo.InvariantCulture,
                    $"min={filteredAngles.Min():F6} avg={filteredAngles.Average():F6} max={filteredAngles.Max():F6}")
                : "min=na avg=na max=na";

            Console.Error.WriteLine(
                string.Create(
                    CultureInfo.InvariantCulture,
                    $"[medial-axis][boost] edges total={edgeCount} primary={edgeData.Count(e => e.IsPrimary)} secondary={edgeData.Count(e => e.IsSecondary)} finite={edgeData.Count(e => !e.IsInfinite)} innerPrimary={innerPrimaryCount} walkedUnfiltered={walkedUnfiltered!.Count} walkedFiltered={walked.Count} angleFiltered={Math.Max(0, walkedUnfiltered.Count - walked.Count)} filteringAngle={filteringAngle:F6}"));
            Console.Error.WriteLine(
                string.Create(
                    CultureInfo.InvariantCulture,
                    $"[medial-axis][boost] angleFilteredBreakdown linear={filteredLinear} curved={filteredCurved} pointPoint={filteredPointPoint} pointSegment={filteredPointSegment} segmentSegment={filteredSegmentSegment}"));
            Console.Error.WriteLine($"[medial-axis][boost] angleFilteredStats {angleStats}");
        }

        // maxRadius is already in the same coordinate scale domain as the inputs.
        double emitMaxRadius = maxRadius > 0 ? maxRadius : -1.0;
        // CentralAngle tolerance is in radians â€” scale-invariant, pass as-is.

        var rawOutput = new List<MedialSegment>(walked.Count);
        int skippedNullVertices = 0;
        int emitRejected = 0;
        foreach (int idx in walked)
        {
            var edge = edgeData[idx];
            if (edge.Vertex0 is null || edge.Vertex1 is null)
            {
                skippedNullVertices++;
                continue;
            }

            bool emitted = MedialAxisSegmentEmitter.TryCreateOutputSegments(
                edge, rawOutput,
                noParabola: false,
                showSites:  false,
                tolerance:  tolerance,
                method:     MedialAxisSegmentEmitter.ParabolaDiscretizationMethod.CentralAngle,
                maxRadius:  emitMaxRadius);

            if (!emitted)
            {
                emitRejected++;
            }
        }

        if (debugEnabled)
        {
            var (nodeCount, degree1Count, branchCount) = ComputeDegreeStats(rawOutput);
            Console.Error.WriteLine(
                string.Create(
                    CultureInfo.InvariantCulture,
                    $"[medial-axis][boost] emit walked={walked.Count} skippedNullVertices={skippedNullVertices} emitRejected={emitRejected} outSegments={rawOutput.Count} graphNodes={nodeCount} leafNodes={degree1Count} branchNodes={branchCount}"));
        }

        PerfLog.Stop(
            "BoostVoronoiProvider.RunPipeline",
            t0,
            $"edges={edgeCount} vertices={vertexCount} cells={cellCount} out={rawOutput.Count}");

        return rawOutput;
    }

    private static bool IsMedialDebugEnabled() =>
        string.Equals(
            Environment.GetEnvironmentVariable("RW_TOOLPATHS_MEDIAL_DEBUG"),
            "1",
            StringComparison.Ordinal);

    private static (int NodeCount, int Degree1Count, int BranchCount) ComputeDegreeStats(
        IReadOnlyList<MedialSegment> segments)
    {
        var degrees = new Dictionary<(long X, long Y), int>();

        static (long X, long Y) Key(MedialPoint point)
        {
            const double scale = 1_000_000.0;
            return (
                (long)Math.Round(point.X * scale, MidpointRounding.AwayFromZero),
                (long)Math.Round(point.Y * scale, MidpointRounding.AwayFromZero));
        }

        foreach (var segment in segments)
        {
            var k0 = Key(segment.Point0);
            var k1 = Key(segment.Point1);

            degrees[k0] = degrees.TryGetValue(k0, out int d0) ? d0 + 1 : 1;
            degrees[k1] = degrees.TryGetValue(k1, out int d1) ? d1 + 1 : 1;
        }

        int degree1 = 0;
        int branch = 0;
        foreach (var kv in degrees)
        {
            if (kv.Value == 1)
            {
                degree1++;
            }
            else if (kv.Value >= 3)
            {
                branch++;
            }
        }

        return (degrees.Count, degree1, branch);
    }

    private static double? TryGetCellsAngleForDebug(MedialAxisEdgeClassifier.EdgeData edge)
    {
        if (edge.Cell.ContainsPoint || edge.TwinCell.ContainsPoint)
            return null;

        var left = edge.Cell.Segment;
        var right = edge.TwinCell.Segment;
        if (left is null || right is null)
            return null;

        if (EqualPoints(left.Low, right.Low))
            return Angle(left.High, left.Low, right.High);
        if (EqualPoints(left.High, right.Low))
            return Angle(left.Low, left.High, right.High);
        if (EqualPoints(left.Low, right.High))
            return Angle(left.High, left.Low, right.Low);
        if (EqualPoints(left.High, right.High))
            return Angle(left.Low, left.High, right.Low);

        return null;
    }

    private static bool EqualPoints(MedialAxisEdgeClassifier.EdgePoint a, MedialAxisEdgeClassifier.EdgePoint b) =>
        Math.Abs(a.X - b.X) < 1e-5 && Math.Abs(a.Y - b.Y) < 1e-5;

    private static double Angle(
        MedialAxisEdgeClassifier.EdgePoint a,
        MedialAxisEdgeClassifier.EdgePoint pivot,
        MedialAxisEdgeClassifier.EdgePoint b)
    {
        double ax = a.X - pivot.X;
        double ay = a.Y - pivot.Y;
        double bx = b.X - pivot.X;
        double by = b.Y - pivot.Y;

        double dot = ax * bx + ay * by;
        double denom = Math.Sqrt((ax * ax + ay * ay) * (bx * bx + by * by));
        if (denom <= 0)
            return 0;

        return Math.Acos(Math.Clamp(dot / denom, -1.0, 1.0));
    }

    // -- CellSite reconstruction -----------------------------------------------

    private static MedialAxisEdgeClassifier.CellSite MakeCellSite(
        BoostVoronoiInterop.BvCell cell,
        IReadOnlyList<SegmentSiteData> segs)
    {
        if (cell.ContainsPoint != 0)
        {
            var seg = segs[cell.SourceIndex];
            bool isStartPointCategory =
                cell.SourceCategory == BoostVoronoiInterop.SourceCategory.SegmentStartPoint;
            var pt = isStartPointCategory ? seg.Low : seg.High;
            return new MedialAxisEdgeClassifier.CellSite(true, pt, null);
        }
        else
        {
            var seg = segs[cell.SourceIndex];
            return new MedialAxisEdgeClassifier.CellSite(
                false, null,
                new MedialAxisEdgeClassifier.SegmentSite(seg.Low, seg.High));
        }
    }

    // -- Segment site building -------------------------------------------------

    /// <summary>
    /// Carries the raw A->B coordinates passed to Boost plus the Low/High
    /// endpoints in insertion order
    /// used to
    /// reconstruct <see cref="MedialAxisEdgeClassifier.CellSite"/> values.
    /// </summary>
    private readonly record struct SegmentSiteData(
        double Ax, double Ay, double Bx, double By,
        MedialAxisEdgeClassifier.EdgePoint Low,
        MedialAxisEdgeClassifier.EdgePoint High);

    private static List<SegmentSiteData> BuildSegmentSites(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes)
    {
        int estimated = boundary.Count;
        for (int i = 0; i < holes.Count; i++)
            estimated += holes[i].Count;

        var result = new List<SegmentSiteData>(estimated);
        AddRing(result, boundary);
        for (int i = 0; i < holes.Count; i++)
            AddRing(result, holes[i]);
        return result;

        static void AddRing(List<SegmentSiteData> list, IReadOnlyList<PointD> ring)
        {
            if (ring.Count < 2) return;
            for (int i = 0; i < ring.Count; i++)
            {
                var a = ring[i];
                var b = ring[(i + 1) % ring.Count];
                var low = new MedialAxisEdgeClassifier.EdgePoint(a.x, a.y);
                var high = new MedialAxisEdgeClassifier.EdgePoint(b.x, b.y);
                list.Add(new SegmentSiteData(a.x, a.y, b.x, b.y, low, high));
            }
        }
    }

    // -- Helpers ---------------------------------------------------------------

    private static List<PointD> RoundAndNormalizeRing(IReadOnlyList<PointD> ring)
    {
        var result = new List<PointD>(ring.Count);
        for (int i = 0; i < ring.Count; i++)
        {
            var p = ring[i];
            result.Add(new PointD(Math.Round(p.x), Math.Round(p.y)));
        }

        if (result.Count > 1
            && result[0].x == result[^1].x
            && result[0].y == result[^1].y)
            result.RemoveAt(result.Count - 1);

        return result;
    }
}


