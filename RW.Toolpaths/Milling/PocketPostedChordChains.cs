namespace RW.Toolpaths.Milling;

/// <summary>Joins only existing exact integer edges at the caller's coordinate
/// scale; branches end chains and cycles retain their closing edge. No snapping,
/// interpolation or scale conversion occurs here.</summary>
internal static class PocketPostedChordChains
{
    internal readonly record struct Point(long X, long Y);
    internal readonly record struct Segment(Point A, Point B);
    internal sealed record Result(List<List<Point>> Paths, int UniqueEdges, int DuplicateEdges,
        int BranchVertices, int ClosedCycles);

    internal static Result Build(IReadOnlyList<Segment> source, CancellationToken token)
    {
        var edges = new List<Segment>();
        var unique = new HashSet<Segment>();
        var adjacency = new Dictionary<Point, List<int>>();
        int duplicates = 0;
        foreach (var original in source)
        {
            token.ThrowIfCancellationRequested();
            if (original.A == original.B) continue;
            bool ordered = original.A.X < original.B.X || original.A.X == original.B.X && original.A.Y <= original.B.Y;
            var edge = ordered ? original : new Segment(original.B, original.A);
            if (!unique.Add(edge)) { duplicates++; continue; }
            int index = edges.Count;
            edges.Add(edge);
            Add(edge.A, index); Add(edge.B, index);
        }
        var used = new bool[edges.Count];
        var result = new List<List<Point>>();
        int branches = adjacency.Count(entry => entry.Value.Count > 2);
        for (int index = 0; index < edges.Count; index++)
        {
            if (used[index]) continue;
            var edge = edges[index];
            if (adjacency[edge.A].Count != 2) Walk(edge.A, index);
            else if (adjacency[edge.B].Count != 2) Walk(edge.B, index);
        }
        for (int index = 0; index < edges.Count; index++)
            if (!used[index]) Walk(edges[index].A, index);
        return new(result, edges.Count, duplicates, branches,
            result.Count(path => path.Count > 2 && path[0] == path[^1]));

        void Add(Point point, int index)
        {
            if (!adjacency.TryGetValue(point, out var incident)) adjacency.Add(point, incident = new());
            incident.Add(index);
        }
        void Walk(Point start, int index)
        {
            var path = new List<Point> { start };
            Point current = start;
            while (!used[index])
            {
                token.ThrowIfCancellationRequested();
                used[index] = true;
                var edge = edges[index];
                current = edge.A == current ? edge.B : edge.A;
                path.Add(current);
                var incident = adjacency[current];
                if (incident.Count != 2) break;
                index = incident[0] == index ? incident[1] : incident[0];
            }
            result.Add(path);
        }
    }
}
