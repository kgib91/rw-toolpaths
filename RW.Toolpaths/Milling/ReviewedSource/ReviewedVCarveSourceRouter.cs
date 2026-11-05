using RW.Toolpaths;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Milling.ReviewedSource;

// Open undirected route inspection: retain every required source edge, pair
// remaining odd vertices with shortest paths on those same edges, then perform
// an Euler walk. No invented cutting connectors or geometry repairs.
public static class ReviewedVCarveSourceRouter
{
    public sealed record Receipt(int Region, int Components, int RequiredEdges, int CoveredEdges,
        int ReturnEdges, double RequiredLength, double ReturnLength);
    public static (List<TaggedToolpath> Paths, Receipt Accounting) Route(ReviewedVCarveConstructor.Result source, ReviewedVCarveSchedule job)
    {
        var nodes = source.Nodes; var original = source.Edges;
        var adjacent = Enumerable.Range(0, nodes.Length).Select(_ => new List<int>()).ToArray();
        for (int i = 0; i < original.Length; i++) { adjacent[original[i].A].Add(i); adjacent[original[i].B].Add(i); }
        double Length(int edge) { var e = original[edge]; var a = nodes[e.A]; var b = nodes[e.B]; return Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2) + Math.Pow(a.Z - b.Z, 2)); }
        (double[] Distance, int[] Previous) Shortest(int root)
        {
            var distance = Enumerable.Repeat(double.PositiveInfinity, nodes.Length).ToArray(); var previous = Enumerable.Repeat(-1, nodes.Length).ToArray();
            var queue = new PriorityQueue<int, double>(); distance[root] = 0; queue.Enqueue(root, 0);
            while (queue.TryDequeue(out int node, out double known))
            {
                if (known != distance[node]) continue;
                foreach (int i in adjacent[node])
                {
                    var e = original[i]; int next = e.A == node ? e.B : e.A; double d = known + Length(i);
                    if (d >= distance[next]) continue;
                    distance[next] = d; previous[next] = i; queue.Enqueue(next, d);
                }
            }
            return (distance, previous);
        }
        var remaining = Enumerable.Range(0, nodes.Length).Where(i => adjacent[i].Count > 0).ToHashSet();
        var paths = new List<TaggedToolpath>(); var covered = new HashSet<int>(); int components = 0, returns = 0; double returnLength = 0;
        var schedule = new DepthSchedule(job.Depth, job.Stepdown) { SurfaceZ = job.Surface, StartDepth = job.StartingDepth };
        while (remaining.Count > 0)
        {
            int root = remaining.Min(); var component = new List<int>(); var todo = new Stack<int>(); todo.Push(root); remaining.Remove(root);
            while (todo.TryPop(out int n))
            {
                component.Add(n);
                foreach (int i in adjacent[n]) { var e = original[i]; int next = e.A == n ? e.B : e.A; if (remaining.Remove(next)) todo.Push(next); }
            }
            components++; var odds = component.Where(n => adjacent[n].Count % 2 == 1).ToHashSet();
            int start = component.OrderByDescending(n => nodes[n].Z).ThenBy(n => n).First();
            if (odds.Count > 0)
            {
                var first = Shortest(odds.Min()); start = odds.MaxBy(n => first.Distance[n]);
                var next = Shortest(start); int end = odds.Where(n => n != start).MaxBy(n => next.Distance[n]);
                odds.Remove(start); odds.Remove(end);
            }
            var instances = new List<(int Original, int A, int B)>();
            var componentSet = component.ToHashSet();
            for (int i = 0; i < original.Length; i++) if (componentSet.Contains(original[i].A)) instances.Add((i, original[i].A, original[i].B));
            while (odds.Count > 0)
            {
                int from = odds.Min(); odds.Remove(from); var shortest = Shortest(from); int to = odds.MinBy(n => shortest.Distance[n]); odds.Remove(to);
                while (to != from)
                {
                    int i = shortest.Previous[to]; if (i < 0) throw new InvalidOperationException("Disconnected route-pair witness");
                    var e = original[i]; instances.Add((i, e.A, e.B)); returns++; returnLength += Length(i); to = e.A == to ? e.B : e.A;
                }
            }
            var local = component.ToDictionary(n => n, _ => new List<int>());
            for (int i = 0; i < instances.Count; i++) { local[instances[i].A].Add(i); local[instances[i].B].Add(i); }
            var used = new bool[instances.Count]; var cursor = component.ToDictionary(n => n, _ => 0);
            var stack = new Stack<(int Node, int Incoming)>(); stack.Push((start, -1)); var reverse = new List<int>();
            while (stack.TryPeek(out var top))
            {
                var list = local[top.Node]; while (cursor[top.Node] < list.Count && used[list[cursor[top.Node]]]) cursor[top.Node]++;
                if (cursor[top.Node] == list.Count) { stack.Pop(); if (top.Incoming >= 0) reverse.Add(top.Incoming); continue; }
                int i = list[cursor[top.Node]++]; used[i] = true; var e = instances[i]; stack.Push((e.A == top.Node ? e.B : e.A, i));
            }
            reverse.Reverse(); if (reverse.Count != instances.Count) throw new InvalidOperationException("Incomplete Euler route");
            for (int pass = 0; pass < schedule.PassCount; pass++)
            {
                double cap = job.Surface - schedule.PassBottomZ(pass); int current = start; var points = new List<Point3D>(); var spans = new List<ToolpathSpan>();
                Point3D World(int n) { var p = nodes[n]; return p with { Z = job.Surface + Math.Max(p.Z, -cap) }; }
                points.Add(World(current)); var thisPass = new HashSet<int>();
                foreach (int i in reverse)
                {
                    var e = instances[i]; if (e.A != current && e.B != current) throw new InvalidOperationException("Euler route lost its cursor");
                    current = e.A == current ? e.B : e.A; points.Add(World(current));
                    var kind = thisPass.Add(e.Original) ? ToolpathSpanKind.Cut : ToolpathSpanKind.Link;
                    spans.Add(new(points.Count - 2, points.Count - 1, kind)); covered.Add(e.Original);
                }
                paths.Add(new(points, spans, source.Region, "standard-vcarve", pass, components - 1, detectCurves: false));
            }
        }
        if (covered.Count != original.Length) throw new InvalidOperationException($"Missing required work region={source.Region} covered={covered.Count}/{original.Length}");
        return (paths, new(source.Region, components, original.Length, covered.Count, returns, Enumerable.Range(0, original.Length).Sum(Length), returnLength));
    }
}
