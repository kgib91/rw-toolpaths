using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>
/// A single accepted holding-tab footprint: an oriented rectangle spanning a cut
/// channel, embedded a little into the material on both sides.
/// Coordinates are shape space while planning and world space once the caller
/// has applied the operation transform.
/// </summary>
public sealed class HoldingTabFootprint
{
    /// <summary>Footprint corners in winding order (0-1-2-3).</summary>
    public PointD[] Corners { get; set; } = new PointD[4];

    /// <summary>Top surface height of the retained tab (vertical axis, never transformed in XZ).</summary>
    public double HeightY { get; set; }
}

/// <summary>
/// Falsifiable result of holding-tab planning for one operation:
/// how much material was found, how much of it floats, and how much of the
/// floating material was actually bridged.
/// </summary>
public sealed class HoldingTabReport
{
    public int PieceCount { get; set; }
    public int AnchoredCount { get; set; }
    public int FloatingCount { get; set; }
    public int TabbedCount { get; set; }
    public int UnresolvedCount { get; set; }

    public bool HasUnresolvedFloatingPieces => UnresolvedCount > 0;

    /// <summary>Accepted tab footprints, for rendering and verification.</summary>
    public List<HoldingTabFootprint> Footprints { get; } = new();

    /// <summary>Outlines of floating pieces that could not be bridged.</summary>
    public List<List<PointD>> UnresolvedOutlines { get; } = new();

    /// <summary>One line per remaining-material piece, for falsifying the classification.</summary>
    public List<string> PieceCensus { get; } = new();

    /// <summary>One concrete cause per unresolved piece.</summary>
    public List<string> UnresolvedReasons { get; } = new();

    public string Summary =>
        $"pieces={PieceCount} anchored={AnchoredCount} floating={FloatingCount} " +
        $"tabbed={TabbedCount} unresolved={UnresolvedCount}";

    public string? SafetyDiagnostic => !HasUnresolvedFloatingPieces
        ? null
        : $"{Summary}. {string.Join(" ", UnresolvedReasons)}";
}

/// <summary>
/// Holding-tab plan for one operation: the arc intervals each cutter ring must
/// lift over, plus the report describing why.
/// </summary>
public sealed class HoldingTabPlan
{
    /// <summary>Ring index → arc intervals (shape-space arc length from ring vertex 0).</summary>
    public Dictionary<int, List<(double Start, double End)>> RingZones { get; } = new();

    public HoldingTabReport Report { get; } = new();
}

/// <summary>
/// Plans holding tabs in <b>material space</b> rather than per cutter ring.
/// <para>
/// A cutter ring is not a piece: one band of remaining material is bounded by
/// several rings, and one ring bounds parts of several pieces. Planning per ring
/// therefore lets a neighbouring ring's full-depth pass sweep straight through a
/// strip another ring tried to preserve.
/// </para>
/// <para>The model is:</para>
/// <list type="number">
/// <item>Cut region C = union over rings of (ring ⊕ disk of tool radius) — Minkowski sum.</item>
/// <item>Remaining material R = stock S − C, split into connected components (PolyTree).</item>
/// <item>Components are classified anchored / floating by material connectivity.</item>
/// <item>Candidate tab footprints bridge a floating piece to a different piece across a channel.</item>
/// <item>Greedy union-find acceptance gives every floating piece <c>tabCount</c> bridges to the anchored root.</item>
/// <item>Each accepted footprint is projected onto <b>every</b> ring that crosses it.</item>
/// </list>
/// A tab is useful iff it merges two distinct components of R.
/// </summary>
public static class HoldingTabPlanner
{
    private sealed class Piece
    {
        public Piece(List<PointD> outer, List<List<PointD>> holes)
        {
            Outer = outer;
            Holes = holes;
        }

        public List<PointD> Outer;
        public List<List<PointD>> Holes;
        public List<List<PointD>> WebCore { get; set; } = new();
        public PolygonSetIndex? WebCoreIndex { get; set; }
        public double Area;
        public double SourceOverlapArea;
        public double CentroidX;
        public double CentroidY;
        public double MinX, MaxX, MinY, MaxY;
        public bool IsAnchored;
        public bool IsPartMaterial;
        public bool IsFloating;
        public bool IsIgnoredThinSliver;
    }

    private readonly struct Candidate
    {
        public Candidate(
            int pieceA, int pieceB,
            double centerX, double centerY,
            double channelCenterX, double channelCenterY,
            double axisX, double axisY,
            double halfSpan, double channelHalfSpan, double halfWidth, double cutterClearance,
            int contourIndex, double contourPosition, double score)
        {
            PieceA = pieceA;
            PieceB = pieceB;
            CenterX = centerX;
            CenterY = centerY;
            ChannelCenterX = channelCenterX;
            ChannelCenterY = channelCenterY;
            AxisX = axisX;
            AxisY = axisY;
            HalfSpan = halfSpan;
            ChannelHalfSpan = channelHalfSpan;
            HalfWidth = halfWidth;
            CutterClearance = cutterClearance;
            ContourIndex = contourIndex;
            ContourPosition = contourPosition;
            Score = score;
        }

        public int PieceA { get; }
        public int PieceB { get; }
        public double CenterX { get; }
        public double CenterY { get; }
        public double ChannelCenterX { get; }
        public double ChannelCenterY { get; }
        public double AxisX { get; }
        public double AxisY { get; }
        public double HalfSpan { get; }
        public double ChannelHalfSpan { get; }
        public double HalfWidth { get; }
        public double CutterClearance { get; }
        public int ContourIndex { get; }
        public double ContourPosition { get; }
        public double Score { get; }
    }

    private readonly record struct CandidateBuildDiagnostics(
        int ChannelHits,
        int OwnerAttachmentFails,
        int ReceiverAttachmentFails);

    private readonly record struct PolygonBounds(
        double MinX,
        double MinY,
        double MaxX,
        double MaxY);

    private readonly record struct IndexedPolygon(
        List<PointD> Points,
        PolygonBounds Bounds,
        int Winding);

    private sealed class PolygonSetIndex
    {
        private readonly IndexedPolygon[] _polygons;

        internal PolygonSetIndex(IEnumerable<List<PointD>> polygons)
        {
            _polygons = polygons
                .Where(polygon => polygon.Count >= 3)
                .Select(polygon => new IndexedPolygon(
                    polygon,
                    GetBounds(polygon),
                    PolygonArea(polygon) >= 0 ? 1 : -1))
                .ToArray();
        }

        internal bool Contains(double x, double y)
        {
            int winding = 0;
            foreach (IndexedPolygon polygon in _polygons)
            {
                PolygonBounds bounds = polygon.Bounds;
                if (x < bounds.MinX || x > bounds.MaxX
                    || y < bounds.MinY || y > bounds.MaxY)
                {
                    continue;
                }
                if (PointInPolygon(polygon.Points, x, y))
                    winding += polygon.Winding;
            }
            return winding != 0;
        }
    }

    private sealed class PieceBoundsIndex
    {
        private readonly List<Piece> _pieces;
        private readonly List<int>?[] _cells;
        private readonly int _columns;
        private readonly int _rows;
        private readonly double _minX;
        private readonly double _minY;
        private readonly double _cellWidth;
        private readonly double _cellHeight;

        internal PieceBoundsIndex(List<Piece> pieces)
        {
            _pieces = pieces;
            if (pieces.Count == 0)
            {
                _columns = 1;
                _rows = 1;
                _cells = new List<int>?[1];
                _cellWidth = 1;
                _cellHeight = 1;
                return;
            }

            _minX = pieces.Min(piece => piece.MinX);
            _minY = pieces.Min(piece => piece.MinY);
            double maxX = pieces.Max(piece => piece.MaxX);
            double maxY = pieces.Max(piece => piece.MaxY);
            double width = Math.Max(maxX - _minX, 1e-9);
            double height = Math.Max(maxY - _minY, 1e-9);
            int targetCellCount = Math.Max(1, pieces.Count * 4);
            _columns = Math.Clamp(
                (int)Math.Ceiling(Math.Sqrt(targetCellCount * width / height)),
                1,
                256);
            _rows = Math.Clamp(
                (int)Math.Ceiling((double)targetCellCount / _columns),
                1,
                256);
            _cellWidth = width / _columns;
            _cellHeight = height / _rows;
            _cells = new List<int>?[_columns * _rows];

            for (int pieceIndex = 0; pieceIndex < pieces.Count; pieceIndex++)
            {
                Piece piece = pieces[pieceIndex];
                int minColumn = Column(piece.MinX);
                int maxColumn = Column(piece.MaxX);
                int minRow = Row(piece.MinY);
                int maxRow = Row(piece.MaxY);
                for (int row = minRow; row <= maxRow; row++)
                {
                    for (int column = minColumn; column <= maxColumn; column++)
                    {
                        int cellIndex = row * _columns + column;
                        (_cells[cellIndex] ??= new List<int>()).Add(pieceIndex);
                    }
                }
            }
        }

        internal int Find(double x, double y)
        {
            List<int>? candidates = _cells[Row(y) * _columns + Column(x)];
            if (candidates is null)
                return -1;

            foreach (int pieceIndex in candidates)
            {
                Piece piece = _pieces[pieceIndex];
                if (x < piece.MinX || x > piece.MaxX || y < piece.MinY || y > piece.MaxY)
                    continue;
                if (PointInPiece(piece, x, y))
                    return pieceIndex;
            }
            return -1;
        }

        private int Column(double x)
            => Math.Clamp((int)((x - _minX) / _cellWidth), 0, _columns - 1);

        private int Row(double y)
            => Math.Clamp((int)((y - _minY) / _cellHeight), 0, _rows - 1);
    }

    private readonly record struct BoundaryArcSegment(
        double StartX,
        double StartY,
        double TangentX,
        double TangentY,
        double StartArc,
        double EndArc);

    private sealed class BoundaryArcIndex
    {
        private readonly List<PointD> _boundary;
        private readonly BoundaryArcSegment[] _segments;

        internal BoundaryArcIndex(List<PointD> boundary, double perimeter)
        {
            _boundary = boundary;
            Perimeter = perimeter;
            var segments = new List<BoundaryArcSegment>(boundary.Count);
            double walked = 0;
            for (int index = 0; index < boundary.Count; index++)
            {
                int next = (index + 1) % boundary.Count;
                double startX = boundary[index].x;
                double startY = boundary[index].y;
                double deltaX = boundary[next].x - startX;
                double deltaY = boundary[next].y - startY;
                double length = Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
                if (length < 1e-6)
                    continue;

                segments.Add(new BoundaryArcSegment(
                    startX,
                    startY,
                    deltaX / length,
                    deltaY / length,
                    walked,
                    walked + length));
                walked += length;
            }
            _segments = segments.ToArray();
        }

        internal double Perimeter { get; }

        internal void GetPointAndTangent(
            double arc,
            out double pointX,
            out double pointY,
            out double tangentX,
            out double tangentY)
        {
            if (Perimeter > 0)
            {
                arc %= Perimeter;
                if (arc < 0)
                    arc += Perimeter;
            }

            int low = 0;
            int high = _segments.Length;
            while (low < high)
            {
                int middle = low + (high - low) / 2;
                if (_segments[middle].EndArc < arc)
                    low = middle + 1;
                else
                    high = middle;
            }

            if (low < _segments.Length)
            {
                BoundaryArcSegment segment = _segments[low];
                double distance = arc - segment.StartArc;
                tangentX = segment.TangentX;
                tangentY = segment.TangentY;
                pointX = segment.StartX + tangentX * distance;
                pointY = segment.StartY + tangentY * distance;
                return;
            }

            pointX = _boundary[0].x;
            pointY = _boundary[0].y;
            tangentX = 1;
            tangentY = 0;
        }
    }

    /// <summary>
    /// Plans holding tabs for one operation.
    /// </summary>
    /// <param name="rings">Cutter centreline rings in shape space.</param>
    /// <param name="ringPerimeters">Closed perimeter of each ring, matching <paramref name="rings"/>.</param>
    /// <param name="stockOutline">Stock frame in shape space; null falls back to an inflated bounding box.</param>
    /// <param name="sourceMaterial">Filled source regions in shape space (outers CCW, holes CW).</param>
    /// <param name="toolRadius">Cutter radius in shape space.</param>
    /// <param name="tabCount">Bridges required per floating piece.</param>
    /// <param name="tabLength">Tab length along the cut path, in shape space.</param>
    /// <param name="tabHeightY">Top surface height of the retained tab.</param>
    /// <param name="ignoredSliverThickness">Maximum ignored sliver thickness, in shape space.</param>
    public static HoldingTabPlan Plan(
        IReadOnlyList<List<PointD>> rings,
        IReadOnlyList<double> ringPerimeters,
        List<PointD>? stockOutline,
        List<List<PointD>> sourceMaterial,
        double toolRadius,
        int tabCount,
        double tabLength,
        double tabHeightY,
        double ignoredSliverThickness)
    {
        long totalStart = PerfLog.Start();
        var plan = new HoldingTabPlan();
        if (rings.Count == 0 || tabCount < 1 || tabLength <= 0 || toolRadius <= 0)
        {
            plan.Report.UnresolvedReasons.Add(
                $"planner skipped: rings={rings.Count}, tabCount={tabCount}, " +
                $"tabLength={tabLength:F3}, toolRadius={toolRadius:F3}");
            return plan;
        }

        // ── Phase 1: material model ────────────────────────────────────────────
    long materialModelStart = PerfLog.Start();
        var ringPolys = new List<List<PointD>>(rings.Count);
        foreach (var ring in rings)
            if (ring.Count >= 3) ringPolys.Add(ring);

        var cutRegion = PolygonOps.BufferCenterlines(ringPolys, toolRadius);
        if (cutRegion.Count == 0)
        {
            plan.Report.UnresolvedReasons.Add("planner skipped: cut region is empty after buffering rings.");
            return plan;
        }

        var stock = BuildStockOutline(stockOutline, cutRegion, sourceMaterial, toolRadius, tabLength);
        var components = PolygonOps.DifferenceToComponents(
            new List<List<PointD>> { stock }, cutRegion);

        double debrisArea = toolRadius * toolRadius * 0.5;
        var pieces = new List<Piece>(components.Count);
        foreach (var component in components)
        {
            double area = Math.Abs(PolygonArea(component.Outer));
            foreach (var hole in component.Holes)
                area -= Math.Abs(PolygonArea(hole));
            if (area < debrisArea)
                continue;

            var piece = new Piece(component.Outer, component.Holes) { Area = area };
            ComputeBounds(piece.Outer, out piece.MinX, out piece.MaxX, out piece.MinY, out piece.MaxY);
            ComputeCentroid(piece.Outer, out piece.CentroidX, out piece.CentroidY);
            pieces.Add(piece);
        }
        PerfLog.Stop("HoldingTabPlanner.MaterialModel", materialModelStart,
            $"rings={rings.Count} cutPolygons={cutRegion.Count} components={components.Count} " +
            $"pieces={pieces.Count}");

        plan.Report.PieceCount = pieces.Count;
        if (pieces.Count == 0)
        {
            plan.Report.UnresolvedReasons.Add("planner skipped: no remaining material components.");
            return plan;
        }

        // A floating remnant only needs enough local thickness to carry the retained
        // bridge. Requiring a large fraction of cutter diameter rejects the narrow
        // strips that Constrain Floating exists to keep from entering the cutter.
        // Keep a real morphological thickness proof, but use a modest machinable floor.
        double minWeb = Math.Max(0.5, toolRadius * 0.2);
        double webCoreInset = minWeb * 0.5;
        double frameEpsilon = Math.Max(toolRadius * 0.25, 0.02);
        int anchorPieceIndex = -1;
        int boundaryTouchingPieces = 0;
        double anchorArea = double.MinValue;
        for (int i = 0; i < pieces.Count; i++)
        {
            if (!TouchesOutline(pieces[i].Outer, stock, frameEpsilon))
                continue;

            boundaryTouchingPieces++;
            if (pieces[i].Area <= anchorArea)
                continue;

            anchorPieceIndex = i;
            anchorArea = pieces[i].Area;
        }

        if (anchorPieceIndex < 0)
        {
            for (int i = 0; i < pieces.Count; i++)
            {
                if (pieces[i].Area <= anchorArea)
                    continue;

                anchorPieceIndex = i;
                anchorArea = pieces[i].Area;
            }
        }

        plan.Report.PieceCensus.Add(
            $"anchor: policy=largest-boundary-carrier, piece={anchorPieceIndex}, " +
            $"area={anchorArea:F2}, boundaryTouching={boundaryTouchingPieces}");

        var floatingPieces = new List<int>();
        long classificationStart = PerfLog.Start();
        int sourceFilledPieces = 0;
        int thinFloatingPieces = 0;
        int ignoredThinSlivers = 0;
        var sourceOverlapMillisecondsByPiece = new double[pieces.Count];
        var webCoreMillisecondsByPiece = new double[pieces.Count];
        var sliverMillisecondsByPiece = new double[pieces.Count];
        var boundedSourceMaterial = sourceMaterial
            .Select(polygon => (Polygon: polygon, Bounds: GetBounds(polygon)))
            .ToList();
        int classificationParallelism = Math.Max(1, Environment.ProcessorCount - 1);
        Parallel.For(
            0,
            pieces.Count,
            new ParallelOptions { MaxDegreeOfParallelism = classificationParallelism },
            i =>
        {
            Piece piece = pieces[i];
            piece.IsAnchored = i == anchorPieceIndex;
            long sourceOverlapStart = Stopwatch.GetTimestamp();
            piece.SourceOverlapArea = sourceMaterial.Count == 0
                ? piece.Area
                : ComputeSourceOverlapArea(piece, boundedSourceMaterial);
            sourceOverlapMillisecondsByPiece[i] = ElapsedMilliseconds(sourceOverlapStart);
            piece.IsPartMaterial = piece.SourceOverlapArea > 0.01;
            // CNC safety is topological, not semantic: every disconnected stock
            // component can move into the cutter. Source fill only describes design
            // ownership; it must not exempt nested islands inside holes from tabs.
            piece.IsFloating = !piece.IsAnchored;

            var material = new List<List<PointD>>(1 + piece.Holes.Count)
            {
                piece.Outer
            };
            material.AddRange(piece.Holes);
            long webCoreStart = Stopwatch.GetTimestamp();
            piece.WebCore = PolygonOps.Offset(material, -webCoreInset);
            piece.WebCoreIndex = new PolygonSetIndex(piece.WebCore);
            webCoreMillisecondsByPiece[i] = ElapsedMilliseconds(webCoreStart);

            long sliverStart = Stopwatch.GetTimestamp();
            if (piece.IsFloating
                && ignoredSliverThickness > 0
                && PolygonPerimeter(piece.Outer) >= ignoredSliverThickness * 4
                && PolygonOps.Offset(
                    material, -ignoredSliverThickness * 0.5).Count == 0)
            {
                piece.IsIgnoredThinSliver = true;
                piece.IsFloating = false;
            }
            sliverMillisecondsByPiece[i] = ElapsedMilliseconds(sliverStart);
        });

        for (int i = 0; i < pieces.Count; i++)
        {
            Piece piece = pieces[i];
            if (piece.IsPartMaterial)
                sourceFilledPieces++;
            if (piece.IsIgnoredThinSliver)
                ignoredThinSlivers++;
            if (piece.IsAnchored)
                plan.Report.AnchoredCount++;
            if (piece.IsFloating)
            {
                floatingPieces.Add(i);
                if (piece.WebCore.Count == 0)
                    thinFloatingPieces++;
            }
        }

        double sourceOverlapMilliseconds = sourceOverlapMillisecondsByPiece.Sum();
        double webCoreMilliseconds = webCoreMillisecondsByPiece.Sum();
        double sliverMilliseconds = sliverMillisecondsByPiece.Sum();

        plan.Report.FloatingCount = floatingPieces.Count;
        plan.Report.PieceCensus.Add(
            $"classification: sourceFilled={sourceFilledPieces}, nestedIslands={floatingPieces.Count - sourceFilledPieces}, "
            + $"ignoredThinSlivers={ignoredThinSlivers}, ignoredBelow={ignoredSliverThickness:F3}, "
            + $"noMinWebCore={thinFloatingPieces}, minWeb={minWeb:F3}");
        PerfLog.Stop("HoldingTabPlanner.Classification", classificationStart,
            $"pieces={pieces.Count} floating={floatingPieces.Count} sourcePolygons={sourceMaterial.Count} " +
            $"parallelism={classificationParallelism} " +
            $"overlapMs={sourceOverlapMilliseconds:F2} webCoreMs={webCoreMilliseconds:F2} " +
            $"sliverMs={sliverMilliseconds:F2}");
        if (floatingPieces.Count == 0)
            return plan;

        // ── Phase 2: candidates + greedy union-find solve ─────────────────────
        long candidateBuildStart = PerfLog.Start();
        var candidatesByPiece = new Dictionary<int, List<Candidate>>(floatingPieces.Count);
        var candidateDiagnostics = new Dictionary<int, CandidateBuildDiagnostics>(floatingPieces.Count);
        var pieceBoundsIndex = new PieceBoundsIndex(pieces);
        var candidateBuildResults = new (
            List<Candidate> Candidates,
            CandidateBuildDiagnostics Diagnostics)[floatingPieces.Count];
        int candidateParallelism = Math.Max(1, Environment.ProcessorCount - 1);
        Parallel.For(
            0,
            floatingPieces.Count,
            new ParallelOptions { MaxDegreeOfParallelism = candidateParallelism },
            position =>
        {
            int pieceIndex = floatingPieces[position];
            List<Candidate> candidates = BuildCandidates(
            pieceIndex, pieces, pieceBoundsIndex, toolRadius, tabLength, minWeb,
                out CandidateBuildDiagnostics diagnostics);

            candidateBuildResults[position] = (candidates, diagnostics);
        });

        for (int position = 0; position < floatingPieces.Count; position++)
        {
            int pieceIndex = floatingPieces[position];
            candidatesByPiece[pieceIndex] = candidateBuildResults[position].Candidates;
            candidateDiagnostics[pieceIndex] = candidateBuildResults[position].Diagnostics;
        }
        int candidateCount = candidatesByPiece.Values.Sum(candidates => candidates.Count);
        PerfLog.Stop("HoldingTabPlanner.BuildCandidates", candidateBuildStart,
            $"pieces={floatingPieces.Count} candidates={candidateCount} " +
            $"parallelism={candidateParallelism}");

        plan.Report.PieceCensus.Add(
            $"length: configuredTabLength={tabLength:F3}, shortenedCandidates=0 (disabled)");

        var parent = new int[pieces.Count + 1];
        for (int i = 0; i < parent.Length; i++) parent[i] = i;
        int anchorRoot = pieces.Count;
        for (int i = 0; i < pieces.Count; i++)
            if (pieces[i].IsAnchored) Union(parent, i, anchorRoot);

        var accepted = new List<Candidate>();
        var bridgeCounts = new int[pieces.Count];
        var reportedPieces = new HashSet<int>();
        long candidateSolveStart = PerfLog.Start();
        int webChecks = 0;

        foreach (int pieceIndex in floatingPieces
                     .OrderBy(index => candidatesByPiece[index].Count))
        {
            var candidates = candidatesByPiece[pieceIndex];
            if (candidates.Count == 0)
            {
                string reason = pieces[pieceIndex].WebCore.Count == 0
                    ? $"piece cannot contain the minimum web (minWeb={minWeb:F3})."
                    : $"no full-length channel candidate found (piece area={pieces[pieceIndex].Area:F2}, "
                        + $"tabLength={tabLength:F3}, "
                        + $"channelHits={candidateDiagnostics[pieceIndex].ChannelHits}, "
                        + $"ownerAttachmentFails={candidateDiagnostics[pieceIndex].OwnerAttachmentFails}, "
                        + $"receiverAttachmentFails={candidateDiagnostics[pieceIndex].ReceiverAttachmentFails}).";
                RecordUnresolved(plan, pieces, pieceIndex, reportedPieces, reason);
                continue;
            }

            int overlapRejects = 0;
            int footprintWidthRejects = 0;
            int attachmentCoreRejects = 0;
            var available = new List<Candidate>(candidates.Count);
            foreach (var candidate in candidates.OrderByDescending(item => item.Score))
            {
                if (OverlapsAccepted(candidate, accepted))
                {
                    overlapRejects++;
                    continue;
                }

                webChecks++;
                var webFailure = GetWebFailure(candidate, pieces, minWeb);
                if (webFailure != WebFailure.None)
                {
                    if (webFailure == WebFailure.FootprintTooNarrow)
                        footprintWidthRejects++;
                    else
                        attachmentCoreRejects++;
                    continue;
                }

                available.Add(candidate);
            }

            List<Candidate> batch = FindBestCandidateBatch(available, tabCount);
            if (batch.Count != tabCount)
            {
                RecordUnresolved(plan, pieces, pieceIndex, reportedPieces,
                    $"exact quota unavailable: capacity={batch.Count}/{tabCount}, " +
                    $"overlap={overlapRejects}, footprintTooNarrow={footprintWidthRejects}, " +
                    $"attachmentCore={attachmentCoreRejects}, minWeb={minWeb:F3}, " +
                    $"tabLength={tabLength:F3}, candidates={candidates.Count}.");
            }
            else
            {
                plan.Report.PieceCensus.Add(
                    $"spacing: piece#{pieceIndex}, contour={batch[0].ContourIndex}, " +
                    $"normalizedGaps=[{DescribeNormalizedGaps(batch)}]");
            }

            foreach (var candidate in batch)
            {
                accepted.Add(candidate);
                bridgeCounts[pieceIndex]++;
                if (candidate.PieceB >= 0 && candidate.PieceB != pieceIndex)
                    Union(parent, pieceIndex, candidate.PieceB);
            }
        }
        PerfLog.Stop("HoldingTabPlanner.SolveCandidates", candidateSolveStart,
            $"candidates={candidateCount} webChecks={webChecks} accepted={accepted.Count}");

        int partialQuotaPieces = 0;
        foreach (int pieceIndex in floatingPieces)
        {
            bool satisfied = bridgeCounts[pieceIndex] == tabCount;
            bool anchored = Find(parent, pieceIndex) == Find(parent, anchorRoot);
            if (satisfied && anchored)
            {
                plan.Report.TabbedCount++;
                continue;
            }

            if (bridgeCounts[pieceIndex] > 0)
                partialQuotaPieces++;

            if (satisfied && !anchored)
            {
                RecordUnresolved(plan, pieces, pieceIndex, reportedPieces,
                    $"has {bridgeCounts[pieceIndex]} bridges but none of them reach anchored material.");
            }
            plan.Report.UnresolvedCount++;
        }

        plan.Report.PieceCensus.Add(
            $"quota: requiredPerIsland={tabCount}, complete={plan.Report.TabbedCount}, "
            + $"partial={partialQuotaPieces}, zero={plan.Report.UnresolvedCount - partialQuotaPieces}");

        // ── Phase 3: projection onto every ring that crosses a footprint ──────
    long projectionStart = PerfLog.Start();
        foreach (var candidate in accepted)
        {
            var footprint = new HoldingTabFootprint
            {
                Corners = QuadCorners(candidate),
                HeightY = tabHeightY
            };
            plan.Report.Footprints.Add(footprint);

            for (int ringIndex = 0; ringIndex < rings.Count; ringIndex++)
            {
                var ring = rings[ringIndex];
                if (ring.Count < 3) continue;

                double perimeter = ringIndex < ringPerimeters.Count ? ringPerimeters[ringIndex] : 0;
                if (perimeter < 0.001) continue;

                ProjectFootprintOntoRing(candidate, ring, perimeter, toolRadius, ringIndex, plan.RingZones);
            }
        }

        foreach (var zones in plan.RingZones.Values)
            MergeZones(zones);

        string zonesPerRing = string.Join(", ", plan.RingZones
            .OrderBy(entry => entry.Key)
            .Select(entry => $"{entry.Key}:{entry.Value.Count}"));
        plan.Report.PieceCensus.Add(
            $"projection: physicalTabLength={tabLength:F3}, cutterClearanceEachSide={toolRadius:F3}, " +
            $"zonesPerRing=[{zonesPerRing}]");
        PerfLog.Stop("HoldingTabPlanner.Projection", projectionStart,
            $"accepted={accepted.Count} rings={rings.Count} zonedRings={plan.RingZones.Count}");
        PerfLog.Stop("HoldingTabPlanner.Plan", totalStart, plan.Report.Summary);

        return plan;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Phase 1 helpers — material model
    // ═══════════════════════════════════════════════════════════════════════════

    /// <summary>
    /// Builds the stock frame. A supplied outline is a hard physical boundary: growing it
    /// would invent material outside the stock and allow a tab to terminate in air.
    /// </summary>
    internal static List<PointD> BuildStockOutline(
        List<PointD>? provided,
        List<List<PointD>> cutRegion,
        List<List<PointD>> sourceMaterial,
        double toolRadius,
        double tabLength)
    {
        if (provided != null && provided.Count >= 3)
        {
            var quad = new List<PointD>(provided);
            if (PolygonArea(quad) < 0) quad.Reverse();
            return quad;
        }

        double minX = double.MaxValue, maxX = double.MinValue;
        double minY = double.MaxValue, maxY = double.MinValue;

        void Extend(List<List<PointD>> polys)
        {
            foreach (var poly in polys)
                foreach (var point in poly)
                {
                    double x = point.x, y = point.y;
                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                }
        }

        Extend(cutRegion);
        Extend(sourceMaterial);

        if (minX > maxX)
            return new List<PointD>();

        double margin = toolRadius * 4.0 + tabLength;
        minX -= margin; minY -= margin; maxX += margin; maxY += margin;
        return new List<PointD>
        {
            new PointD(minX, minY), new PointD(maxX, minY),
            new PointD(maxX, maxY), new PointD(minX, maxY)
        };
    }

    private static bool TouchesOutline(
        List<PointD> polygon, List<PointD> outline, double epsilon)
    {
        if (outline.Count < 3) return false;
        double epsilonSquared = epsilon * epsilon;

        foreach (var point in polygon)
        {
            double px = point.x, py = point.y;
            for (int i = 0, j = outline.Count - 1; i < outline.Count; j = i++)
            {
                double distanceSquared = PointSegmentDistanceSquared(
                    px, py, outline[j].x, outline[j].y, outline[i].x, outline[i].y);
                if (distanceSquared <= epsilonSquared)
                    return true;
            }
        }

        return false;
    }

    private static double ComputeSourceOverlapArea(
        Piece piece,
        IReadOnlyList<(List<PointD> Polygon, PolygonBounds Bounds)> sourceMaterial)
    {
        var relevantSource = sourceMaterial
            .Where(entry => BoundsOverlap(piece, entry.Bounds))
            .Select(entry => (IReadOnlyList<PointD>)entry.Polygon)
            .ToList();
        if (relevantSource.Count == 0)
            return 0;

        var subject = new List<List<PointD>>(1 + piece.Holes.Count) { piece.Outer };
        subject.AddRange(piece.Holes);

        var overlap = PolygonOps.Intersect(subject, relevantSource);
        double overlapArea = 0;
        foreach (var poly in overlap)
            overlapArea += PolygonArea(poly);

        return Math.Abs(overlapArea);
    }

    private static PolygonBounds GetBounds(IReadOnlyList<PointD> polygon)
    {
        double minX = double.MaxValue;
        double minY = double.MaxValue;
        double maxX = double.MinValue;
        double maxY = double.MinValue;
        foreach (PointD point in polygon)
        {
            minX = Math.Min(minX, point.x);
            minY = Math.Min(minY, point.y);
            maxX = Math.Max(maxX, point.x);
            maxY = Math.Max(maxY, point.y);
        }
        return new PolygonBounds(minX, minY, maxX, maxY);
    }

    private static bool BoundsOverlap(Piece piece, PolygonBounds bounds)
        => piece.MinX <= bounds.MaxX
            && piece.MaxX >= bounds.MinX
            && piece.MinY <= bounds.MaxY
            && piece.MaxY >= bounds.MinY;

    private static double ElapsedMilliseconds(long startTimestamp)
        => (Stopwatch.GetTimestamp() - startTimestamp) * 1000.0 / Stopwatch.Frequency;

    // ═══════════════════════════════════════════════════════════════════════════
    // Phase 2 helpers — candidates and acceptance
    // ═══════════════════════════════════════════════════════════════════════════

    private static List<Candidate> BuildCandidates(
        int pieceIndex,
        List<Piece> pieces,
        PieceBoundsIndex pieceBoundsIndex,
        double toolRadius,
        double tabLength,
        double minWeb,
        out CandidateBuildDiagnostics diagnostics)
    {
        var candidates = new List<Candidate>();
        diagnostics = default;
        Piece piece = pieces[pieceIndex];
        if (piece.WebCore.Count == 0)
            return candidates;

        int channelHits = 0;
        int ownerAttachmentFails = 0;
        int receiverAttachmentFails = 0;

        double sampleSpacing = Math.Max(Math.Min(toolRadius * 0.35, tabLength * 0.1), 0.2);
        double marchStep = Math.Max(toolRadius * 0.35, 0.1);
        double minChannel = toolRadius;
        double maxChannel = toolRadius * 6;
        double maxAttachmentDepth = Math.Max(toolRadius * 6, tabLength * 2);
        double halfWidth = tabLength * 0.5;

        var boundaries = new List<List<PointD>>(1 + piece.Holes.Count) { piece.Outer };
        boundaries.AddRange(piece.Holes);

        var sharpCorners = new List<PointD>();
        foreach (var boundary in boundaries)
            CollectSharpCorners(boundary, sharpCorners);

        for (int boundaryIndex = 0; boundaryIndex < boundaries.Count; boundaryIndex++)
        {
            var boundary = boundaries[boundaryIndex];
            if (boundary.Count < 3) continue;

            double boundaryPerimeter = PolygonPerimeter(boundary);
            if (boundaryPerimeter < tabLength)
                continue;
            var boundaryArcIndex = new BoundaryArcIndex(boundary, boundaryPerimeter);

            double walkedArc = 0;
            double nextSampleArc = 0;
            for (int i = 0; i < boundary.Count; i++)
            {
                int next = (i + 1) % boundary.Count;
                double startX = boundary[i].x, startY = boundary[i].y;
                double dx = boundary[next].x - boundary[i].x;
                double dy = boundary[next].y - boundary[i].y;
                double length = Math.Sqrt(dx * dx + dy * dy);
                if (length < 1e-5) continue;

                double tangentX = dx / length, tangentY = dy / length;
                for (; nextSampleArc < walkedArc + length; nextSampleArc += sampleSpacing)
                {
                    double distance = nextSampleArc - walkedArc;
                    double sampleX = startX + tangentX * distance;
                    double sampleY = startY + tangentY * distance;

                    // Outward normal: probe both sides and keep the one leaving the piece.
                    double normalX = tangentY, normalY = -tangentX;
                    if (PointInPiece(piece, sampleX + normalX * marchStep, sampleY + normalY * marchStep))
                    {
                        normalX = -normalX;
                        normalY = -normalY;
                    }
                    if (PointInPiece(piece, sampleX + normalX * marchStep, sampleY + normalY * marchStep))
                        continue;

                    int otherPiece = -1;
                    double span = 0;
                    for (double march = marchStep; march <= maxChannel; march += marchStep)
                    {
                        double probeX = sampleX + normalX * march;
                        double probeY = sampleY + normalY * march;
                        int hit = pieceBoundsIndex.Find(probeX, probeY);
                        if (hit < 0) continue;
                        if (hit == pieceIndex) break;

                        if (pieces[hit].WebCore.Count == 0)
                            break;

                        otherPiece = hit;
                        span = march;
                        break;
                    }

                    if (otherPiece < 0 || span < minChannel)
                        continue;

                    channelHits++;

                    double channelHalfSpan = span * 0.5;
                    double tangentNormalX = -normalY;
                    double tangentNormalY = normalX;
                    if (!TryFindContourAttachmentDepth(
                            piece, boundaryArcIndex, nextSampleArc,
                            pieces[pieceIndex].WebCoreIndex!,
                            halfWidth, minWeb, marchStep, maxAttachmentDepth,
                            out double firstAttachmentDepth))
                    {
                        ownerAttachmentFails++;
                        continue;
                    }

                    if (!TryFindAttachmentDepth(
                            pieces[otherPiece].WebCoreIndex!,
                            sampleX + normalX * span, sampleY + normalY * span,
                            normalX, normalY,
                            tangentNormalX, tangentNormalY,
                            halfWidth, minWeb, marchStep, maxAttachmentDepth,
                            out double secondAttachmentDepth))
                    {
                        receiverAttachmentFails++;
                        continue;
                    }

                    double fullSpan = firstAttachmentDepth + span + secondAttachmentDepth;
                    double halfSpan = fullSpan * 0.5;
                    double centerDistance = (span + secondAttachmentDepth - firstAttachmentDepth) * 0.5;
                    double centerX = sampleX + normalX * centerDistance;
                    double centerY = sampleY + normalY * centerDistance;
                    double channelCenterX = sampleX + normalX * channelHalfSpan;
                    double channelCenterY = sampleY + normalY * channelHalfSpan;

                    double cornerDistance = NearestSharpCornerDistance(sharpCorners, sampleX, sampleY);
                    double cornerScore = Math.Min(cornerDistance / Math.Max(tabLength * 2, 1e-3), 1.0);
                    double spanScore = 1 - Math.Min(span / maxChannel, 1.0);
                    double score = cornerScore * 0.7 + spanScore * 0.3;
                    double contourPosition = nextSampleArc / boundaryPerimeter;

                    candidates.Add(new Candidate(
                        pieceIndex, otherPiece, centerX, centerY,
                        channelCenterX, channelCenterY,
                        normalX, normalY, halfSpan, channelHalfSpan, halfWidth, toolRadius,
                        boundaryIndex, contourPosition, score));
                }

                walkedArc += length;
            }
        }

        diagnostics = new CandidateBuildDiagnostics(
            channelHits, ownerAttachmentFails, receiverAttachmentFails);
        return candidates;
    }

    private static bool TryFindContourAttachmentDepth(
        Piece piece,
        BoundaryArcIndex boundary,
        double centerArc,
        PolygonSetIndex webCore,
        double halfLength, double minWeb,
        double step, double maxDepth,
        out double depth)
    {
        depth = 0;
        double sideProbe = Math.Max(minWeb * 0.25, 0.02);

        for (double candidateDepth = minWeb; candidateDepth <= maxDepth; candidateDepth += step)
        {
            bool fits = true;
            for (int sample = 0; sample < 5; sample++)
            {
                double alongArc = halfLength * (-1 + sample * 0.5);
                boundary.GetPointAndTangent(
                    centerArc + alongArc,
                    out double pointX, out double pointY,
                    out double tangentX, out double tangentY);

                double inwardX = -tangentY;
                double inwardY = tangentX;
                if (!PointInPiece(
                        piece,
                        pointX + inwardX * sideProbe,
                        pointY + inwardY * sideProbe))
                {
                    inwardX = -inwardX;
                    inwardY = -inwardY;
                }

                if (!webCore.Contains(
                        pointX + inwardX * candidateDepth,
                        pointY + inwardY * candidateDepth))
                {
                    fits = false;
                    break;
                }
            }

            if (fits)
            {
                depth = candidateDepth;
                return true;
            }
        }

        return false;
    }

    private static double PolygonPerimeter(List<PointD> polygon)
    {
        double perimeter = 0;
        for (int index = 0; index < polygon.Count; index++)
        {
            int next = (index + 1) % polygon.Count;
            double deltaX = polygon[next].x - polygon[index].x;
            double deltaY = polygon[next].y - polygon[index].y;
            perimeter += Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
        }
        return perimeter;
    }

    private static bool TryFindAttachmentDepth(
        PolygonSetIndex webCore,
        double edgeX, double edgeY,
        double inwardX, double inwardY,
        double tangentX, double tangentY,
        double halfWidth, double minWeb,
        double step, double maxDepth,
        out double depth)
    {
        depth = 0;
        for (double candidateDepth = minWeb; candidateDepth <= maxDepth; candidateDepth += step)
        {
            double centerX = edgeX + inwardX * candidateDepth;
            double centerY = edgeY + inwardY * candidateDepth;
            if (EdgeFits(centerX, centerY, tangentX, tangentY, halfWidth, webCore))
            {
                depth = candidateDepth;
                return true;
            }
        }

        return false;
    }

    private static bool EdgeFits(
        double centerX, double centerY,
        double tangentX, double tangentY,
        double halfWidth,
        PolygonSetIndex webCore)
    {
        for (int sample = 0; sample < 5; sample++)
        {
            double along = halfWidth * (-1 + sample * 0.5);
                if (!webCore.Contains(
                    centerX + tangentX * along,
                    centerY + tangentY * along))
            {
                return false;
            }
        }

        return true;
    }

    private static List<Candidate> FindBestCandidateBatch(
        List<Candidate> candidates, int requiredCount)
    {
        var best = new List<Candidate>(requiredCount);
        if (requiredCount < 1 || candidates.Count == 0)
            return best;

        foreach (var contourCandidates in candidates
                     .GroupBy(candidate => candidate.ContourIndex)
                     .Select(group => group.OrderBy(candidate => candidate.ContourPosition).ToList()))
        {
            for (int seedIndex = 0; seedIndex < contourCandidates.Count; seedIndex++)
            {
                Candidate seed = contourCandidates[seedIndex];
                for (int direction = -1; direction <= 1; direction += 2)
                {
                    var current = new List<Candidate>(requiredCount) { seed };
                    for (int step = 1; step < requiredCount; step++)
                    {
                        double target = NormalizeContourPosition(
                            seed.ContourPosition + direction * (double)step / requiredCount);
                        Candidate? next = FindNearestCompatibleCandidate(
                            contourCandidates, target, current);
                        if (!next.HasValue)
                            break;
                        current.Add(next.Value);
                    }

                    if (IsBetterSpacedBatch(current, best))
                        best = current;
                }
            }
        }

        return best;
    }

    private static Candidate? FindNearestCompatibleCandidate(
        List<Candidate> sortedCandidates,
        double target,
        List<Candidate> selected)
    {
        int low = 0;
        int high = sortedCandidates.Count;
        while (low < high)
        {
            int middle = low + (high - low) / 2;
            if (sortedCandidates[middle].ContourPosition < target)
                low = middle + 1;
            else
                high = middle;
        }

        int leftRaw = low - 1;
        int rightRaw = low;
        var visited = new HashSet<int>();
        while (visited.Count < sortedCandidates.Count)
        {
            GetWrappedCandidate(
                sortedCandidates, leftRaw,
                out int leftIndex, out double leftPosition);
            GetWrappedCandidate(
                sortedCandidates, rightRaw,
                out int rightIndex, out double rightPosition);

            double leftDistance = target - leftPosition;
            double rightDistance = rightPosition - target;
            bool takeLeft = leftDistance < rightDistance
                || (Math.Abs(leftDistance - rightDistance) <= 1e-6
                    && sortedCandidates[leftIndex].Score >= sortedCandidates[rightIndex].Score);

            int candidateIndex;
            if (takeLeft)
            {
                candidateIndex = leftIndex;
                leftRaw--;
            }
            else
            {
                candidateIndex = rightIndex;
                rightRaw++;
            }

            if (!visited.Add(candidateIndex))
                continue;

            Candidate candidate = sortedCandidates[candidateIndex];
            if (!OverlapsAccepted(candidate, selected))
                return candidate;
        }

        return null;
    }

    private static void GetWrappedCandidate(
        List<Candidate> candidates,
        int rawIndex,
        out int index,
        out double unwrappedPosition)
    {
        int count = candidates.Count;
        int cycle = rawIndex / count;
        index = rawIndex % count;
        if (index < 0)
        {
            index += count;
            cycle--;
        }

        unwrappedPosition = candidates[index].ContourPosition + cycle;
    }

    private static bool IsBetterSpacedBatch(
        List<Candidate> candidateBatch,
        List<Candidate> currentBest)
    {
        if (candidateBatch.Count != currentBest.Count)
            return candidateBatch.Count > currentBest.Count;
        if (candidateBatch.Count == 0)
            return false;

        ComputeSpacingQuality(
            candidateBatch,
            out double candidateMinimumGap,
            out double candidateGapVariance,
            out double candidateScore);
        ComputeSpacingQuality(
            currentBest,
            out double bestMinimumGap,
            out double bestGapVariance,
            out double bestScore);

        if (Math.Abs(candidateMinimumGap - bestMinimumGap) > 1e-5)
            return candidateMinimumGap > bestMinimumGap;
        if (Math.Abs(candidateGapVariance - bestGapVariance) > 1e-6)
            return candidateGapVariance < bestGapVariance;
        return candidateScore > bestScore;
    }

    private static void ComputeSpacingQuality(
        List<Candidate> candidates,
        out double minimumGap,
        out double gapVariance,
        out double averageScore)
    {
        if (candidates.Count < 2)
        {
            minimumGap = candidates.Count;
            gapVariance = 0;
            averageScore = candidates.Count == 0 ? 0 : candidates[0].Score;
            return;
        }

        double[] positions = candidates
            .Select(candidate => candidate.ContourPosition)
            .OrderBy(position => position)
            .ToArray();
        double idealGap = 1.0 / positions.Length;
        minimumGap = 1;
        gapVariance = 0;
        for (int index = 0; index < positions.Length; index++)
        {
            double next = index + 1 < positions.Length
                ? positions[index + 1]
                : positions[0] + 1;
            double gap = next - positions[index];
            minimumGap = Math.Min(minimumGap, gap);
            double error = gap - idealGap;
            gapVariance += error * error;
        }

        gapVariance /= positions.Length;
        averageScore = candidates.Average(candidate => candidate.Score);
    }

    private static string DescribeNormalizedGaps(List<Candidate> candidates)
    {
        double[] positions = candidates
            .Select(candidate => candidate.ContourPosition)
            .OrderBy(position => position)
            .ToArray();
        var gaps = new string[positions.Length];
        for (int index = 0; index < positions.Length; index++)
        {
            double next = index + 1 < positions.Length
                ? positions[index + 1]
                : positions[0] + 1;
            gaps[index] = (next - positions[index]).ToString("F3");
        }

        return string.Join(",", gaps);
    }

    private static double NormalizeContourPosition(double position)
        => position - Math.Floor(position);

    private static bool OverlapsAccepted(Candidate candidate, List<Candidate> accepted)
    {
        foreach (var other in accepted)
            if (RectanglesOverlap(candidate, other))
                return true;
        return false;
    }

    private enum WebFailure
    {
        None,
        FootprintTooNarrow,
        AttachmentCore
    }

    private static WebFailure GetWebFailure(
        Candidate candidate, List<Piece> pieces, double minWeb)
    {
        // The footprint itself must survive a morphological opening at the requested
        // minimum-web thickness. Candidate footprints are exact oriented rectangles, so
        // this is equivalent to eroding and dilating the quad but avoids two Clipper offsets.
        double openingRadius = minWeb * 0.5;
        if (candidate.HalfSpan + 1e-9 < openingRadius
            || candidate.HalfWidth + 1e-9 < openingRadius)
        {
            return WebFailure.FootprintTooNarrow;
        }

        // The owner side was already proven over the complete curved TabLength span
        // while constructing the candidate. The receiving side remains a straight
        // footprint edge and must fit completely in its eroded material core.
        return AttachmentEdgeFits(candidate, pieces[candidate.PieceB].WebCoreIndex!, 1)
            ? WebFailure.None
            : WebFailure.AttachmentCore;
    }

    private static bool AttachmentEdgeFits(
        Candidate candidate,
        PolygonSetIndex webCore,
        double axisDirection)
    {
        double tangentX = -candidate.AxisY;
        double tangentY = candidate.AxisX;
        double edgeCenterX = candidate.CenterX
            + candidate.AxisX * candidate.HalfSpan * axisDirection;
        double edgeCenterY = candidate.CenterY
            + candidate.AxisY * candidate.HalfSpan * axisDirection;

        for (int sample = 0; sample < 5; sample++)
        {
            double across = candidate.HalfWidth * (-1 + sample * 0.5);
            double sampleX = edgeCenterX + tangentX * across;
            double sampleY = edgeCenterY + tangentY * across;
            if (!webCore.Contains(sampleX, sampleY))
                return false;
        }

        return true;
    }

    private static void RecordUnresolved(
        HoldingTabPlan plan, List<Piece> pieces, int pieceIndex, HashSet<int> reportedPieces, string reason)
    {
        if (!reportedPieces.Add(pieceIndex))
            return;

        Piece piece = pieces[pieceIndex];
        plan.Report.UnresolvedReasons.Add(
            $"piece#{pieceIndex} area={piece.Area:F2} at ({piece.CentroidX:F2},{piece.CentroidY:F2}): {reason}");

        var outline = new List<PointD>(piece.Outer.Count);
        foreach (var point in piece.Outer)
            outline.Add(new PointD(point.x, point.y));
        plan.Report.UnresolvedOutlines.Add(outline);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Phase 3 helpers — projection onto rings
    // ═══════════════════════════════════════════════════════════════════════════

    private static void ProjectFootprintOntoRing(
        Candidate candidate,
        List<PointD> ring,
        double perimeter, double cutterRadius,
        int ringIndex,
        Dictionary<int, List<(double Start, double End)>> ringZones)
    {
        double arc = 0;

        for (int i = 0; i < ring.Count; i++)
        {
            int next = (i + 1) % ring.Count;
            double startX = ring[i].x, startY = ring[i].y;
            double endX = ring[next].x, endY = ring[next].y;
            double dx = endX - startX, dy = endY - startY;
            double length = Math.Sqrt(dx * dx + dy * dy);
            if (length < 1e-6) continue;

            if (ClipSegmentToRectangle(candidate, cutterRadius, startX, startY, endX, endY,
                    out double enter, out double exit))
            {
                double zoneStart = arc + enter * length;
                double zoneEnd = arc + exit * length;
                if (zoneEnd - zoneStart > 1e-4)
                {
                    if (!ringZones.TryGetValue(ringIndex, out var zones))
                    {
                        zones = new List<(double Start, double End)>();
                        ringZones[ringIndex] = zones;
                    }
                    zones.Add((zoneStart, zoneEnd));
                }
            }

            arc += length;
            if (arc > perimeter + 1e-3) break;
        }
    }

    /// <summary>
    /// Liang–Barsky clip of a cutter centreline against the candidate footprint
    /// expanded by the cutter radius. The physical tab remains the unexpanded
    /// footprint; the expanded interval keeps the cutter envelope out of it.
    /// </summary>
    private static bool ClipSegmentToRectangle(
        Candidate candidate, double cutterRadius,
        double startX, double startY, double endX, double endY,
        out double enter, out double exit)
    {
        enter = 0;
        exit = 1;

        double crossX = -candidate.AxisY;
        double crossY = candidate.AxisX;

        double startU = (startX - candidate.CenterX) * candidate.AxisX + (startY - candidate.CenterY) * candidate.AxisY;
        double startV = (startX - candidate.CenterX) * crossX + (startY - candidate.CenterY) * crossY;
        double endU = (endX - candidate.CenterX) * candidate.AxisX + (endY - candidate.CenterY) * candidate.AxisY;
        double endV = (endX - candidate.CenterX) * crossX + (endY - candidate.CenterY) * crossY;

        return ClipAgainstSlab(startU, endU - startU, candidate.HalfSpan + cutterRadius, ref enter, ref exit)
            && ClipAgainstSlab(startV, endV - startV, candidate.HalfWidth + cutterRadius, ref enter, ref exit)
            && exit > enter;
    }

    private static bool ClipAgainstSlab(
        double start, double delta, double halfExtent, ref double enter, ref double exit)
    {
        if (Math.Abs(delta) < 1e-9)
            return Math.Abs(start) <= halfExtent;

        double low = (-halfExtent - start) / delta;
        double high = (halfExtent - start) / delta;
        if (low > high) (low, high) = (high, low);

        if (low > enter) enter = low;
        if (high < exit) exit = high;
        return exit >= enter;
    }

    private static void MergeZones(List<(double Start, double End)> zones)
    {
        if (zones.Count < 2) return;

        zones.Sort((left, right) => left.Start.CompareTo(right.Start));
        var merged = new List<(double Start, double End)>(zones.Count);
        var current = zones[0];
        for (int i = 1; i < zones.Count; i++)
        {
            if (zones[i].Start <= current.End + 1e-4)
            {
                if (zones[i].End > current.End)
                    current = (current.Start, zones[i].End);
                continue;
            }
            merged.Add(current);
            current = zones[i];
        }
        merged.Add(current);

        zones.Clear();
        zones.AddRange(merged);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometry primitives
    // ═══════════════════════════════════════════════════════════════════════════

    private static PointD[] QuadCorners(Candidate candidate)
        => QuadCorners(candidate, candidate.HalfSpan);

    private static PointD[] QuadCorners(Candidate candidate, double halfSpan)
    {
        double crossX = -candidate.AxisY;
        double crossY = candidate.AxisX;
        double ax = candidate.AxisX * halfSpan;
        double ay = candidate.AxisY * halfSpan;
        double bx = crossX * candidate.HalfWidth;
        double by = crossY * candidate.HalfWidth;

        return new PointD[]
        {
            new PointD(candidate.CenterX - ax - bx, candidate.CenterY - ay - by),
            new PointD(candidate.CenterX + ax - bx, candidate.CenterY + ay - by),
            new PointD(candidate.CenterX + ax + bx, candidate.CenterY + ay + by),
            new PointD(candidate.CenterX - ax + bx, candidate.CenterY - ay + by)
        };
    }

    private static bool RectanglesOverlap(Candidate first, Candidate second)
    {
        // Embedded ends may share material inside a small island while crossing
        // distinct portions of the surrounding cutter channel.
        return !SeparatedOnCandidateAxis(first, second, first.AxisX, first.AxisY)
            && !SeparatedOnCandidateAxis(first, second, -first.AxisY, first.AxisX)
            && !SeparatedOnCandidateAxis(first, second, second.AxisX, second.AxisY)
            && !SeparatedOnCandidateAxis(first, second, -second.AxisY, second.AxisX);
    }

    private static bool SeparatedOnCandidateAxis(
        Candidate first,
        Candidate second,
        double axisX,
        double axisY)
    {
        double centerDistance = Math.Abs(
            (second.ChannelCenterX - first.ChannelCenterX) * axisX
            + (second.ChannelCenterY - first.ChannelCenterY) * axisY);
        double firstRadius = ProjectedCandidateRadius(first, axisX, axisY);
        double secondRadius = ProjectedCandidateRadius(second, axisX, axisY);
        return centerDistance > firstRadius + secondRadius + 1e-5;
    }

    private static double ProjectedCandidateRadius(
        Candidate candidate,
        double axisX,
        double axisY)
    {
        double protectedHalfSpan = candidate.ChannelHalfSpan + candidate.CutterClearance;
        double protectedHalfWidth = candidate.HalfWidth + candidate.CutterClearance;
        double along = Math.Abs(candidate.AxisX * axisX + candidate.AxisY * axisY);
        double across = Math.Abs(-candidate.AxisY * axisX + candidate.AxisX * axisY);
        return protectedHalfSpan * along + protectedHalfWidth * across;
    }

    private static bool PointInPiece(Piece piece, double x, double y)
    {
        if (!PointInPolygon(piece.Outer, x, y))
            return false;
        foreach (var hole in piece.Holes)
            if (PointInPolygon(hole, x, y))
                return false;
        return true;
    }

    private static bool PointInPolygon(List<PointD> polygon, double x, double y)
    {
        bool inside = false;
        for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
        {
            var current = polygon[i];
            var previous = polygon[j];
            if ((current.y > y) != (previous.y > y)
                && x < (previous.x - current.x) * (y - current.y) / (previous.y - current.y) + current.x)
            {
                inside = !inside;
            }
        }
        return inside;
    }

    private static void CollectSharpCorners(
        List<PointD> polygon, List<PointD> corners)
    {
        const double sharpCornerCosine = 0.81915204;  // 35°
        if (polygon.Count < 3) return;

        for (int i = 0; i < polygon.Count; i++)
        {
            int previous = (i + polygon.Count - 1) % polygon.Count;
            int next = (i + 1) % polygon.Count;
            double incomingX = polygon[i].x - polygon[previous].x;
            double incomingY = polygon[i].y - polygon[previous].y;
            double outgoingX = polygon[next].x - polygon[i].x;
            double outgoingY = polygon[next].y - polygon[i].y;
            double incomingLength = Math.Sqrt(incomingX * incomingX + incomingY * incomingY);
            double outgoingLength = Math.Sqrt(outgoingX * outgoingX + outgoingY * outgoingY);
            if (incomingLength < 1e-5 || outgoingLength < 1e-5) continue;

            double dot = (incomingX * outgoingX + incomingY * outgoingY) / (incomingLength * outgoingLength);
            if (dot < sharpCornerCosine)
                corners.Add(new PointD(polygon[i].x, polygon[i].y));
        }
    }

    private static double NearestSharpCornerDistance(
        List<PointD> corners, double x, double y)
    {
        if (corners.Count == 0) return double.MaxValue;

        double best = double.MaxValue;
        foreach (var corner in corners)
        {
            double dx = corner.x - x, dy = corner.y - y;
            double distanceSquared = dx * dx + dy * dy;
            if (distanceSquared < best) best = distanceSquared;
        }
        return Math.Sqrt(best);
    }

    private static double PointSegmentDistanceSquared(
        double px, double py, double ax, double ay, double bx, double by)
    {
        double dx = bx - ax, dy = by - ay;
        double lengthSquared = dx * dx + dy * dy;
        double t = lengthSquared < 1e-12 ? 0 : ((px - ax) * dx + (py - ay) * dy) / lengthSquared;
        t = Math.Clamp(t, 0, 1);
        double cx = ax + dx * t - px;
        double cy = ay + dy * t - py;
        return cx * cx + cy * cy;
    }

    private static double PolygonArea(List<PointD> polygon)
    {
        double area = 0;
        for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
            area += (polygon[j].x * polygon[i].y) - (polygon[i].x * polygon[j].y);
        return area * 0.5;
    }

    private static void ComputeBounds(
        List<PointD> polygon,
        out double minX, out double maxX, out double minY, out double maxY)
    {
        minX = double.MaxValue; maxX = double.MinValue;
        minY = double.MaxValue; maxY = double.MinValue;
        foreach (var point in polygon)
        {
            double x = point.x, y = point.y;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
        }
    }

    private static void ComputeCentroid(
        List<PointD> polygon, out double centroidX, out double centroidY)
    {
        double sumX = 0, sumY = 0;
        foreach (var point in polygon)
        {
            sumX += point.x;
            sumY += point.y;
        }
        centroidX = sumX / Math.Max(1, polygon.Count);
        centroidY = sumY / Math.Max(1, polygon.Count);
    }

    private static int Find(int[] parent, int node)
    {
        while (parent[node] != node)
        {
            parent[node] = parent[parent[node]];
            node = parent[node];
        }
        return node;
    }

    private static void Union(int[] parent, int first, int second)
    {
        int rootA = Find(parent, first);
        int rootB = Find(parent, second);
        if (rootA != rootB)
            parent[rootB] = rootA;
    }
}
