using Clipper2Lib;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace RW.Toolpaths.Milling;

/// <summary>Contours of the continuous upper triangulation of a bilinear height field.
/// Interior triangle edges are never sent through polygon union.</summary>
internal static class HeightmapSliceContours
{
    private struct Vertex
    {
        internal PointD Point;
        internal int FirstEdge;
        internal int Incoming;
        internal int Outgoing;
        internal Vertex(PointD point) { Point = point; FirstEdge = -1; Incoming = 0; Outgoing = 0; }
    }

    internal static bool TryBuild(HeightmapSurface surface, double limit, double tolerance,
        CancellationToken token, out PathsD contours, out int patches, Action<int, int>? progress = null)
    {
        long started = Stopwatch.GetTimestamp();
        const double keyScale = 100_000_000;
        var lookup = new Dictionary<(long X, long Y), int>();
        var vertices = new List<Vertex>();
        var connections = new List<(int To, int Next)>();
        int patchCount = 0, edges = 0, welded = 0;
        int Index(double x, double y)
        {
            var key = (X: checked((long)Math.Round(x * keyScale)), Y: checked((long)Math.Round(y * keyScale)));
            if (lookup.TryGetValue(key, out int index)) return index;
            // Shared coarse/fine edges can differ by the last floating-point bit.
            // The weld is at most 15 nanometres, below Clipper's micrometre grid.
            for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++)
                if (lookup.TryGetValue((key.X + dx, key.Y + dy), out index))
                { welded++; lookup[key] = index; return index; }
            index = vertices.Count;
            lookup.Add(key, index); vertices.Add(new Vertex(new PointD(x, y)));
            return index;
        }
        void Edge(PointD a, PointD b)
        {
            int from = Index(a.x, a.y), to = Index(b.x, b.y);
            if (from == to) return;
            ref var source = ref CollectionsMarshal.AsSpan(vertices)[from];
            connections.Add((to, source.FirstEdge));
            source.FirstEdge = connections.Count - 1; source.Outgoing++;
            CollectionsMarshal.AsSpan(vertices)[to].Incoming++;
            if (++edges > 4_000_000)
                throw new InvalidOperationException("Carving slice exceeds four million contour edges; increase tolerance.");
        }
        PointD Crossing((double X, double Y, double H) p, (double X, double Y, double H) q)
        {
            double t = (limit - p.H) / (q.H - p.H);
            return new PointD(p.X + t * (q.X - p.X), p.Y + t * (q.Y - p.Y));
        }
        void Triangle((double X, double Y, double H) a, (double X, double Y, double H) b,
            (double X, double Y, double H) c)
        {
            PointD entering = default, leaving = default;
            int crossings = 0;
            void Side((double X, double Y, double H) p, (double X, double Y, double H) q)
            {
                bool pin = p.H <= limit, qin = q.H <= limit;
                if (pin == qin) return;
                var point = Crossing(p, q);
                if (pin) leaving = point; else entering = point;
                crossings++;
            }
            Side(a, b); Side(b, c); Side(c, a);
            if (crossings == 2) Edge(leaving, entering);
        }
        void Patch(double x0, double y0, double x1, double y1, double a, double b, double c, double d)
        {
            if ((++patchCount & 255) == 0) token.ThrowIfCancellationRequested();
            if (patchCount > 32_000_000)
                throw new InvalidOperationException("Carving slice exceeds its subdivision budget; increase tolerance.");
            if (Math.Max(Math.Max(a, b), Math.Max(c, d)) <= limit
                || Math.Min(Math.Min(a, b), Math.Min(c, d)) > limit) return;
            double saddle = a - b - c + d;
            if (Math.Abs(saddle) / 4 <= tolerance / 2)
            {
                if (saddle >= 0)
                {
                    Triangle((x0, y0, a), (x1, y0, b), (x1, y1, d));
                    Triangle((x0, y0, a), (x1, y1, d), (x0, y1, c));
                }
                else
                {
                    Triangle((x0, y0, a), (x1, y0, b), (x0, y1, c));
                    Triangle((x1, y0, b), (x1, y1, d), (x0, y1, c));
                }
                return;
            }
            double mx = (x0 + x1) / 2, my = (y0 + y1) / 2;
            double ab = (a + b) / 2, ac = (a + c) / 2, bd = (b + d) / 2, cd = (c + d) / 2, center = (a + b + c + d) / 4;
            Patch(x0, y0, mx, my, a, ab, ac, center); Patch(mx, y0, x1, my, ab, b, center, bd);
            Patch(x0, my, mx, y1, ac, center, c, cd); Patch(mx, my, x1, y1, center, bd, cd, d);
        }
        void Perimeter((double X, double Y, double H) a, (double X, double Y, double H) b)
        {
            bool ain = a.H <= limit, bin = b.H <= limit;
            if (!ain && !bin) return;
            Edge(ain ? new PointD(a.X, a.Y) : Crossing(a, b), bin ? new PointD(b.X, b.Y) : Crossing(a, b));
        }
        int width = surface.Grid.Width - 1, height = surface.Grid.Height - 1;
        bool Included(int x, int y) => x >= 0 && y >= 0 && x < width && y < height && surface.CellIncluded(x, y);
        for (int y = 0; y < height; y++)
        {
            token.ThrowIfCancellationRequested();
            if ((y & 7) == 0) progress?.Invoke(y, height);
            for (int x = 0; x < width; x++)
            {
                if (!Included(x, y)) continue;
                double a = surface.VertexHeight(x, y), b = surface.VertexHeight(x + 1, y);
                double c = surface.VertexHeight(x, y + 1), d = surface.VertexHeight(x + 1, y + 1);
                double x0 = x * surface.PixelX, x1 = (x + 1) * surface.PixelX;
                double y0 = y * surface.PixelY, y1 = (y + 1) * surface.PixelY;
                Patch(x0, y0, x1, y1, a, b, c, d);
                if (!Included(x, y - 1)) Perimeter((x0, y0, a), (x1, y0, b));
                if (!Included(x + 1, y)) Perimeter((x1, y0, b), (x1, y1, d));
                if (!Included(x, y + 1)) Perimeter((x1, y1, d), (x0, y1, c));
                if (!Included(x - 1, y)) Perimeter((x0, y1, c), (x0, y0, a));
            }
        }
        progress?.Invoke(height, height);
        patches = patchCount;
        contours = new PathsD();
        int unbalanced = vertices.Count(v => v.Incoming != v.Outgoing);
        if (unbalanced != 0)
        {
            Console.WriteLine($"[Carving.ContourTrace] fallback=unbalanced vertices={vertices.Count} edges={edges} unbalanced={unbalanced}");
            return false;
        }
        for (int start = 0; start < vertices.Count; start++)
        {
            if ((start & 1023) == 0) token.ThrowIfCancellationRequested();
            while (vertices[start].FirstEdge >= 0)
            {
                var ring = new PathD();
                int current = start;
                do
                {
                    ref var vertex = ref CollectionsMarshal.AsSpan(vertices)[current];
                    ring.Add(surface.Placement.ToWorld(vertex.Point.x, vertex.Point.y));
                    int first = vertex.FirstEdge;
                    if (first < 0) throw new InvalidOperationException("Carving contour traversal encountered an open boundary.");
                    var connection = connections[first]; vertex.FirstEdge = connection.Next;
                    current = connection.To;
                    if ((ring.Count & 1023) == 0) token.ThrowIfCancellationRequested();
                } while (current != start);
                if (ring.Count >= 3) contours.Add(ring);
            }
        }
        contours = Clipper.Union(contours, new PathsD(), FillRule.NonZero, 6);
        Console.WriteLine($"[Carving.ContourTrace] vertices={vertices.Count} edges={edges} welded={welded} contours={contours.Count} points={contours.Sum(p => p.Count)} patches={patches} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
        return true;
    }
}
