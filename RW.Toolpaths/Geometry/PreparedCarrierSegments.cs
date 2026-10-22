namespace RW.Toolpaths.Geometry;

/// <summary>
/// Immutable carrier cut chords. A sum of the guest endpoints' distances to a
/// complete node box bounds every possible excursion through its chords.
/// The caller retains its existing minimizer and original-ordinal tie policy.
/// </summary>
internal sealed class PreparedCarrierSegments
{
    internal readonly record struct Segment(Point3D Start, Point3D End, int EndIndex, int Ordinal);
    private readonly record struct Node(double MinX, double MinY, double MaxX, double MaxY,
        int First, int Count, int Left, int Right);
    private readonly Segment[] _segments;
    private readonly Node[] _nodes;
    private readonly bool _finite;
    internal int Count => _segments.Length;
    internal bool SupportsBounds(Point3D a, Point3D b)
        => _finite && double.IsFinite(a.X) && double.IsFinite(a.Y)
            && double.IsFinite(b.X) && double.IsFinite(b.Y);

    internal sealed class Diagnostics
    {
        internal long IndexBuilds, IndexedSegments, NodeVisits, CandidateVisits, PrunedNodes, FallbackQueries;
        internal double BuildMilliseconds, QueryMilliseconds;
        internal string Describe() => $"carrierIndexBuilds={IndexBuilds} carrierSegments={IndexedSegments} "
            + $"carrierNodeVisits={NodeVisits} carrierCandidateVisits={CandidateVisits} "
            + $"carrierPrunedNodes={PrunedNodes} carrierFallbackQueries={FallbackQueries} "
            + $"carrierBuildMs={BuildMilliseconds:F1} carrierQueryMs={QueryMilliseconds:F1} "
            + "excursionMinimizerIterations=32 tiePolicy=original-segment-order";
    }

    internal PreparedCarrierSegments(TaggedToolpath path, Diagnostics? diagnostics = null)
    {
        long started = System.Diagnostics.Stopwatch.GetTimestamp();
        var segments = new List<Segment>();
        _finite = true;
        foreach (var span in path.Spans)
        {
            if (span.Kind != ToolpathSpanKind.Cut) continue;
            for (int end = Math.Max(1, span.StartIndex + 1); end <= Math.Min(path.Points.Count - 1, span.EndIndex); end++)
            {
                var a = path.Points[end - 1]; var b = path.Points[end];
                _finite &= double.IsFinite(a.X) && double.IsFinite(a.Y)
                    && double.IsFinite(b.X) && double.IsFinite(b.Y);
                segments.Add(new(a, b, end, segments.Count));
            }
        }
        _segments = segments.ToArray();
        var nodes = new List<Node>();
        if (_finite && _segments.Length > 0) Build(0, _segments.Length);
        _nodes = nodes.ToArray();
        if (diagnostics is not null)
        {
            diagnostics.IndexBuilds++; diagnostics.IndexedSegments += Count;
            diagnostics.BuildMilliseconds += System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        }

        int Build(int first, int count)
        {
            double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
            double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
            for (int i = first; i < first + count; i++)
            {
                var segment = _segments[i];
                minX = Math.Min(minX, Math.Min(segment.Start.X, segment.End.X));
                minY = Math.Min(minY, Math.Min(segment.Start.Y, segment.End.Y));
                maxX = Math.Max(maxX, Math.Max(segment.Start.X, segment.End.X));
                maxY = Math.Max(maxY, Math.Max(segment.Start.Y, segment.End.Y));
            }
            int ordinal = nodes.Count; nodes.Add(default);
            int left = -1, right = -1;
            if (count > 12)
            {
                bool x = maxX - minX >= maxY - minY;
                Array.Sort(_segments, first, count, Comparer<Segment>.Create((a, b) =>
                {
                    double ac = x ? a.Start.X / 2 + a.End.X / 2 : a.Start.Y / 2 + a.End.Y / 2;
                    double bc = x ? b.Start.X / 2 + b.End.X / 2 : b.Start.Y / 2 + b.End.Y / 2;
                    int compare = ac.CompareTo(bc);
                    return compare != 0 ? compare : a.Ordinal.CompareTo(b.Ordinal);
                }));
                int half = count / 2;
                left = Build(first, half); right = Build(first + half, count - half);
            }
            nodes[ordinal] = new(minX, minY, maxX, maxY, first, count, left, right);
            return ordinal;
        }
    }

    internal void VisitCandidates(Point3D guestStart, Point3D guestEnd,
        Func<double> best, Action<Segment> evaluate, Diagnostics? diagnostics = null)
    {
        long started = System.Diagnostics.Stopwatch.GetTimestamp();
        if (!SupportsBounds(guestStart, guestEnd))
        {
            if (diagnostics is not null) diagnostics.FallbackQueries++;
            foreach (var segment in _segments.OrderBy(segment => segment.Ordinal))
            { if (diagnostics is not null) diagnostics.CandidateVisits++; evaluate(segment); }
        }
        else if (_nodes.Length > 0)
        {
            var queue = new PriorityQueue<int, double>();
            queue.Enqueue(0, LowerBound(_nodes[0]));
            while (queue.TryDequeue(out int index, out double lower))
            {
                if (diagnostics is not null) diagnostics.NodeVisits++;
                if (lower > best())
                { if (diagnostics is not null) diagnostics.PrunedNodes += queue.Count + 1; break; }
                var node = _nodes[index];
                if (node.Left >= 0)
                { queue.Enqueue(node.Left, LowerBound(_nodes[node.Left])); queue.Enqueue(node.Right, LowerBound(_nodes[node.Right])); }
                else
                    for (int i = node.First; i < node.First + node.Count; i++)
                    { if (diagnostics is not null) diagnostics.CandidateVisits++; evaluate(_segments[i]); }
            }
        }
        if (diagnostics is not null)
            diagnostics.QueryMilliseconds += System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds;

        double LowerBound(Node node)
        {
            double distance = Distance(guestStart, node) + Distance(guestEnd, node);
            // Directed slack protects pruning near floating-point ties. A loose
            // box changes visits only; every retained candidate uses the exact
            // former evaluator and comparison, including coincident chords.
            return Math.Max(0, distance - 64 * double.Epsilon
                - 64 * Math.Abs(Math.BitIncrement(distance) - distance));
        }
    }

    private static double Distance(Point3D point, Node node)
    {
        double x = point.X < node.MinX ? node.MinX - point.X : point.X > node.MaxX ? point.X - node.MaxX : 0;
        double y = point.Y < node.MinY ? node.MinY - point.Y : point.Y > node.MaxY ? point.Y - node.MaxY : 0;
        return Math.Sqrt(x * x + y * y);
    }
}
