using Clipper2Lib;

namespace RW.Toolpaths;

internal static class MedialAxisEdgeClassifier
{
    internal static class Colors
    {
        public const int Unclassified = 11;
        public const int Secondary = 1;
        public const int OuterPrimary = 2;
        public const int InnerPrimary = 3;
        public const int HolePrimary = 4;
    }

    private const double Epsilon = 1e-5;

    internal sealed record class EdgePoint(double X, double Y);

    internal sealed record class SegmentSite(EdgePoint Low, EdgePoint High);

    internal sealed record class CellSite(bool ContainsPoint, EdgePoint? Point, SegmentSite? Segment);

    internal sealed record class EdgeData(
        int Index,
        int TwinIndex,
        bool IsPrimary,
        bool IsSecondary,
        bool IsInfinite,
        int PrevIndex,
        int NextIndex,
        EdgePoint? Vertex0,
        EdgePoint? Vertex1,
        CellSite Cell,
        CellSite TwinCell)
    {
        public int Color { get; set; }
        public bool IsLinear { get; set; }
        public bool IsCurved { get; set; }
    }

    internal static void ClassifyEdges(
        IReadOnlyList<EdgeData> edges,
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes)
    {
        var candidates = new List<EdgeData>();

        foreach (var edge in edges)
        {
            if (edge.IsPrimary)
            {
                if (edge.IsInfinite)
                {
                    edge.Color = Colors.OuterPrimary;
                }
                else
                {
                    edge.Color = Colors.Unclassified;
                    candidates.Add(edge);
                }
            }
            else
            {
                edge.Color = Colors.Secondary;
            }
        }

        var stitchedPolygon = BuildPolygonWithSentinels(boundary, holes);

        foreach (var edge in candidates)
        {
            if (edge.Color == Colors.InnerPrimary)
            {
                continue;
            }

            var nonBorderPoint = GetNonBorderPoint(edge);
            if (PointInPoly(nonBorderPoint, stitchedPolygon))
            {
                edge.Color = Colors.InnerPrimary;

                if (edge.TwinIndex >= 0 && edge.TwinIndex < edges.Count)
                {
                    edges[edge.TwinIndex].Color = Colors.InnerPrimary;
                }
            }
        }
    }

    internal static List<int> SelectInnerPrimaryTraversal(
        IReadOnlyList<EdgeData> edges,
        double filteringAngle)
    {
        var innerPrimary = edges
            .Where(edge => edge.Color == Colors.InnerPrimary)
            .ToList();

        var remaining = new HashSet<int>(innerPrimary.Select(edge => edge.Index));
        var walked = new List<int>();

        while (remaining.Count > 0)
        {
            var start = edges[remaining.First()];

            foreach (var candidate in innerPrimary)
            {
                if (!remaining.Contains(candidate.Index))
                {
                    continue;
                }

                var previous = GetPreviousThroughSecondary(edges, candidate);
                if (previous is null || !remaining.Contains(previous.Index))
                {
                    start = candidate;
                    break;
                }
            }

            var path = new List<int>();
            EdgeData? current = start;
            while (current is not null && remaining.Contains(current.Index))
            {
                walked.Add(current.Index);
                path.Add(current.Index);
                remaining.Remove(current.Index);
                current = GetNextThroughSecondary(edges, current);
            }

            foreach (int edgeIndex in path)
            {
                int twinIndex = edges[edgeIndex].TwinIndex;
                if (twinIndex >= 0)
                {
                    remaining.Remove(twinIndex);
                }
            }
        }

        return walked
            .Where(index =>
            {
                double? angle = GetCellsAngle(edges[index]);
                return angle is null || angle <= filteringAngle;
            })
            .ToList();
    }

    private static List<EdgePoint> BuildPolygonWithSentinels(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes)
    {
        var sentinel = new EdgePoint(0, 0);
        var stitched = new List<EdgePoint> { sentinel };

        AppendRing(stitched, boundary, sentinel);
        foreach (var hole in holes)
        {
            AppendRing(stitched, hole, sentinel);
        }

        return stitched;
    }

    private static void AppendRing(List<EdgePoint> stitched, IReadOnlyList<PointD> ring, EdgePoint sentinel)
    {
        if (ring.Count == 0)
        {
            return;
        }

        foreach (var point in ring)
        {
            stitched.Add(new EdgePoint(point.x, point.y));
        }

        if (!EqualPoints(
                new EdgePoint(ring[0].x, ring[0].y),
                new EdgePoint(ring[^1].x, ring[^1].y)))
        {
            stitched.Add(new EdgePoint(ring[0].x, ring[0].y));
        }

        stitched.Add(sentinel);
    }

    private static EdgePoint GetNonBorderPoint(EdgeData edge)
    {
        var vertex0 = edge.Vertex0 ?? throw new InvalidOperationException("Expected finite edge vertex0");
        var vertex1 = edge.Vertex1 ?? throw new InvalidOperationException("Expected finite edge vertex1");

        if (edge.Cell.ContainsPoint)
        {
            var point = edge.Cell.Point ?? throw new InvalidOperationException("Expected point site");
            if (!EqualPoints(vertex0, point))
            {
                return vertex0;
            }
        }
        else
        {
            var segment = edge.Cell.Segment ?? throw new InvalidOperationException("Expected segment site");
            if (!EqualPoints(vertex0, segment.Low) && !EqualPoints(vertex0, segment.High))
            {
                return vertex0;
            }
        }

        return vertex1;
    }

    private static EdgeData? GetPreviousThroughSecondary(IReadOnlyList<EdgeData> edges, EdgeData edge)
    {
        var current = GetEdge(edges, edge.PrevIndex);
        while (current is not null && current.IsSecondary && current.Index != edge.Index)
        {
            var twin = GetEdge(edges, current.TwinIndex);
            current = twin is null ? null : GetEdge(edges, twin.PrevIndex);
        }

        return current;
    }

    private static EdgeData? GetNextThroughSecondary(IReadOnlyList<EdgeData> edges, EdgeData edge)
    {
        var current = GetEdge(edges, edge.NextIndex);
        while (current is not null && current.IsSecondary && current.Index != edge.Index)
        {
            var twin = GetEdge(edges, current.TwinIndex);
            current = twin is null ? null : GetEdge(edges, twin.NextIndex);
        }

        return current;
    }

    private static double? GetCellsAngle(EdgeData edge)
    {
        if (edge.Cell.ContainsPoint || edge.TwinCell.ContainsPoint)
        {
            return null;
        }

        var left = edge.Cell.Segment ?? throw new InvalidOperationException("Expected segment site on cell");
        var right = edge.TwinCell.Segment ?? throw new InvalidOperationException("Expected segment site on twin cell");

        if (EqualPoints(left.Low, right.Low))
        {
            return Angle(left.High, left.Low, right.High);
        }

        if (EqualPoints(left.High, right.Low))
        {
            return Angle(left.Low, left.High, right.High);
        }

        if (EqualPoints(left.Low, right.High))
        {
            return Angle(left.High, left.Low, right.Low);
        }

        if (EqualPoints(left.High, right.High))
        {
            return Angle(left.Low, left.High, right.Low);
        }

        return null;
    }

    private static double Angle(EdgePoint a, EdgePoint pivot, EdgePoint b)
    {
        double ax = a.X - pivot.X;
        double ay = a.Y - pivot.Y;
        double bx = b.X - pivot.X;
        double by = b.Y - pivot.Y;

        double dot = ax * bx + ay * by;
        double denom = Math.Sqrt((ax * ax + ay * ay) * (bx * bx + by * by));
        if (denom <= 0)
        {
            return 0;
        }

        return Math.Acos(Math.Clamp(dot / denom, -1.0, 1.0));
    }

    private static bool PointInPoly(EdgePoint point, IReadOnlyList<EdgePoint> polygon)
    {
        bool inside = false;
        int current = 0;
        int previous = polygon.Count - 1;

        while (current < polygon.Count)
        {
            var a = polygon[current];
            var b = polygon[previous];

            bool crosses = (a.Y - point.Y > Epsilon) != (b.Y - point.Y > Epsilon);
            if (crosses)
            {
                double x = (b.X - a.X) * (point.Y - a.Y) / (b.Y - a.Y) + a.X - point.X;
                if (x > Epsilon)
                {
                    inside = !inside;
                }
            }

            previous = current++;
        }

        return inside;
    }

    internal static bool EqualPoints(EdgePoint a, EdgePoint b) =>
        Math.Abs(a.X - b.X) < Epsilon && Math.Abs(a.Y - b.Y) < Epsilon;

    private static EdgeData? GetEdge(IReadOnlyList<EdgeData> edges, int index) =>
        index >= 0 && index < edges.Count ? edges[index] : null;
}
