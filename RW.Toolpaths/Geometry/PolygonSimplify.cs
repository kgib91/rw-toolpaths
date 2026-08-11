using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

/// <summary>
/// Vertex-reduction helpers applied to Clipper2 output before it becomes toolpath.
/// Round joins emit dense near-collinear fans; feeding those straight to a controller
/// wastes look-ahead buffer and slows the machine down.
/// </summary>
public static class PolygonSimplify
{
    /// <summary>Default simplification tolerance in workspace units (mm).</summary>
    public const double DefaultTolerance = 0.25;

    /// <summary>
    /// Ramer-Douglas-Peucker simplification for a closed polygon. Corners are preserved
    /// because they are always the farthest point from the chord spanning them.
    /// </summary>
    /// <param name="points">Closed polygon vertices.</param>
    /// <param name="epsilon">Maximum allowed deviation in workspace units.</param>
    public static List<PointD> Rdp(IReadOnlyList<PointD> points, double epsilon)
    {
        int n = points.Count;
        if (n < 4 || epsilon <= 0)
            return new List<PointD>(points);

        var keep = new bool[n];
        keep[0] = true;
        keep[n - 1] = true;

        var stack = new Stack<(int Start, int End)>();
        stack.Push((0, n - 1));

        while (stack.Count > 0)
        {
            var (s, e) = stack.Pop();
            if (e - s < 2)
                continue;

            double maxDist = 0;
            int maxIdx = s;

            double ax = points[s].x, ay = points[s].y;
            double dx = points[e].x - ax, dy = points[e].y - ay;
            double lenSq = dx * dx + dy * dy;

            for (int i = s + 1; i < e; i++)
            {
                double px = points[i].x - ax, py = points[i].y - ay;
                double dist = lenSq < 1e-18
                    ? Math.Sqrt(px * px + py * py)
                    : Math.Abs(px * dy - py * dx) / Math.Sqrt(lenSq);

                if (dist > maxDist)
                {
                    maxDist = dist;
                    maxIdx = i;
                }
            }

            if (maxDist > epsilon)
            {
                keep[maxIdx] = true;
                stack.Push((s, maxIdx));
                stack.Push((maxIdx, e));
            }
        }

        var result = new List<PointD>(n);
        for (int i = 0; i < n; i++)
        {
            if (keep[i])
                result.Add(points[i]);
        }
        return result;
    }

    /// <summary>
    /// Applies <see cref="Rdp"/> to every polygon, dropping any that degenerate below
    /// three vertices. Polygons of six or fewer vertices are passed through untouched
    /// so that rectangles and triangles never lose a corner to rounding.
    /// </summary>
    public static List<List<PointD>> RdpAll(
        IEnumerable<IReadOnlyList<PointD>> polygons,
        double epsilon)
    {
        var output = new List<List<PointD>>();
        foreach (var polygon in polygons)
        {
            if (polygon.Count < 3)
                continue;

            var simplified = polygon.Count > 6
                ? Rdp(polygon, epsilon)
                : new List<PointD>(polygon);

            if (simplified.Count >= 3)
                output.Add(simplified);
        }
        return output;
    }
}
