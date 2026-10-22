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
        => RdpCore(points, epsilon, false);

    // A machining boundary needs distance to the finite replacement segment, including
    // vertices that double back past an endpoint of that segment.
    internal static List<PointD> RdpSegmentBounded(IReadOnlyList<PointD> points, double epsilon)
        => RdpCore(points, epsilon, true);

    private static List<PointD> RdpCore(IReadOnlyList<PointD> points, double epsilon, bool boundToSegment)
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
            double length = Math.Sqrt(lenSq);

            for (int i = s + 1; i < e; i++)
            {
                double px = points[i].x - ax, py = points[i].y - ay;
                double dist;
                if (boundToSegment)
                {
                    double t = lenSq < 1e-18 ? 0 : Math.Clamp((px * dx + py * dy) / lenSq, 0, 1);
                    double ex = px - t * dx, ey = py - t * dy;
                    dist = Math.Sqrt(ex * ex + ey * ey);
                }
                else dist = lenSq < 1e-18 ? Math.Sqrt(px * px + py * py) : Math.Abs(px * dy - py * dx) / length;

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
    /// Applies <see cref="Rdp"/> to every nondegenerate polygon. A valid source
    /// contour survives unchanged when simplification collapses it. Polygons of
    /// six or fewer vertices are passed through untouched so that rectangles and
    /// triangles never lose a corner to rounding.
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

            double sourceArea = PolygonOps.SignedArea(polygon);
            if (!double.IsFinite(sourceArea) || sourceArea == 0)
                continue;

            var simplified = polygon.Count > 6
                ? Rdp(polygon, epsilon)
                : new List<PointD>(polygon);

            double simplifiedArea = simplified.Count >= 3 ? PolygonOps.SignedArea(simplified) : 0;
            if (simplified.Count < 3 || !double.IsFinite(simplifiedArea) || simplifiedArea == 0)
            {
                Console.WriteLine($"[PolygonSimplify.RdpAll] result=retain-source reason=degenerate-simplification "
                    + $"sourcePoints={polygon.Count} simplifiedPoints={simplified.Count} "
                    + $"sourceArea={sourceArea:R} simplifiedArea={simplifiedArea:R} epsilon={epsilon:R}");
                simplified = new List<PointD>(polygon);
            }
            output.Add(simplified);
        }
        return output;
    }
}
