using Clipper2Lib;
using MIConvexHull;

namespace RW.Toolpaths;

/// <summary>
/// Pure .NET medial-axis provider.
///
/// This implementation approximates the interior medial axis by building a
/// Delaunay triangulation over sampled boundary points, then connecting
/// circumcenters of adjacent interior triangles.
/// </summary>
public sealed class NativeMedialAxisProvider : IMedialAxisProvider
{
    private const double InputScale = 4096.0;

    public static NativeMedialAxisProvider CreateDefault() => new();

    public IReadOnlyList<MedialSegment> ConstructMedialAxis(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance,
        double maxRadius,
        double filteringAngle = 3 * Math.PI / 4,
        bool useBigIntegers = true)
    {
        if (boundary is null) throw new ArgumentNullException(nameof(boundary));
        if (holes is null) throw new ArgumentNullException(nameof(holes));
        if (boundary.Count < 3) return Array.Empty<MedialSegment>();

        // JS parity note: construct_medial_axis first rounds all inputs.
        var roundedBoundary = RoundAndNormalizeRing(boundary, InputScale);
        var roundedHoles = holes
            .Select(h => RoundAndNormalizeRing(h, InputScale))
            .Where(r => r.Count >= 3)
            .ToList();

        var boundarySegments = BuildSegments(roundedBoundary, roundedHoles);

        // Keep sampling dense enough that point-based Delaunay better tracks
        // the JS segment-site Voronoi behavior in narrow interior channels.
        double sampleStep = Math.Max(4.0, tolerance * InputScale);
        var vertices = SampleBoundaryVertices(roundedBoundary, roundedHoles, sampleStep);
        if (vertices.Count < 3)
            return Array.Empty<MedialSegment>();

        var triangulation = DelaunayTriangulation<SampleVertex, TriCell>.Create(vertices, 1e-10);
        var cells = triangulation.Cells.ToList();
        var indexByCell = new Dictionary<TriCell, int>(ReferenceEqualityComparer<TriCell>.Instance);
        for (int i = 0; i < cells.Count; i++)
            indexByCell[cells[i]] = i;

        var centers = new (double X, double Y)?[cells.Count];
        for (int i = 0; i < cells.Count; i++)
        {
            var c = Circumcenter(cells[i]);
            if (c is null)
                continue;

            if (IsInsideRegion(c.Value.X, c.Value.Y, roundedBoundary, roundedHoles))
                centers[i] = c;
        }

        var candidates = new List<MedialSegment>();

        for (int i = 0; i < cells.Count; i++)
        {
            var c0 = centers[i];
            if (c0 is null)
                continue;

            foreach (var neighbor in cells[i].Adjacency)
            {
                if (neighbor is null || !indexByCell.TryGetValue(neighbor, out int j) || i >= j)
                    continue;

                var c1 = centers[j];
                if (c1 is null)
                    continue;

                double dx = c1.Value.X - c0.Value.X;
                double dy = c1.Value.Y - c0.Value.Y;
                if (dx * dx + dy * dy <= 1e-12)
                    continue;

                // Keep candidate generation permissive: JS classification marks
                // inner-primary edges first, then prunes later. Early midpoint/
                // angle culling in this approximate provider drops valid inner
                // branches in narrow channels.

                double r0 = DistanceToNearestSegment(c0.Value.X, c0.Value.Y, boundarySegments);
                double r1 = DistanceToNearestSegment(c1.Value.X, c1.Value.Y, boundarySegments);

                // Mirror the JS create_output_segments behavior where maxRadius
                // acts as a guard for long point-point spans.
                if (maxRadius > 0 && r0 > maxRadius * InputScale && r1 > maxRadius * InputScale)
                    continue;

                candidates.Add(new MedialSegment(
                    new MedialPoint(c0.Value.X, c0.Value.Y, r0),
                    new MedialPoint(c1.Value.X, c1.Value.Y, r1)));
            }
        }

        return PostProcessSegments(candidates, roundedBoundary, roundedHoles)
            .Select(seg => new MedialSegment(
                new MedialPoint(seg.Point0.X / InputScale, seg.Point0.Y / InputScale, seg.Point0.Radius / InputScale),
                new MedialPoint(seg.Point1.X / InputScale, seg.Point1.Y / InputScale, seg.Point1.Radius / InputScale)))
            .ToList();
    }

    private static List<PointD> RoundAndNormalizeRing(IReadOnlyList<PointD> ring, double scale)
    {
        var result = new List<PointD>(ring.Count);
        for (int i = 0; i < ring.Count; i++)
        {
            var p = ring[i];
            result.Add(new PointD(Math.Round(p.x * scale), Math.Round(p.y * scale)));
        }

        if (result.Count > 1 && result[0].x == result[^1].x && result[0].y == result[^1].y)
            result.RemoveAt(result.Count - 1);
        return result;
    }

    private static List<PointD> NormalizeRing(IReadOnlyList<PointD> ring)
    {
        var result = ring.ToList();
        if (result.Count > 1 && result[0].x == result[^1].x && result[0].y == result[^1].y)
            result.RemoveAt(result.Count - 1);
        return result;
    }

    private static List<SampleVertex> SampleBoundaryVertices(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        double step)
    {
        var vertices = new List<SampleVertex>();
        int nextId = 0;

        AddRing(boundary);
        foreach (var hole in holes)
            AddRing(hole);

        return vertices;

        void AddRing(IReadOnlyList<PointD> ring)
        {
            if (ring.Count < 2)
                return;

            for (int i = 0; i < ring.Count; i++)
            {
                var a = ring[i];
                var b = ring[(i + 1) % ring.Count];
                AddVertex(a.x, a.y);

                double dx = b.x - a.x;
                double dy = b.y - a.y;
                double len = Math.Sqrt(dx * dx + dy * dy);
                if (len <= step)
                    continue;

                // Use ceiling so every segment longer than step gets at least
                // one interior sample; floor under-sampled and dropped branches.
                int parts = (int)Math.Ceiling(len / step);
                for (int k = 1; k < parts; k++)
                {
                    double t = (double)k / parts;
                    AddVertex(a.x + dx * t, a.y + dy * t);
                }
            }
        }

        void AddVertex(double x, double y)
        {
            vertices.Add(new SampleVertex(nextId++, x, y));
        }
    }

    private static List<(PointD A, PointD B)> BuildSegments(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes)
    {
        var segments = new List<(PointD A, PointD B)>();
        AddRing(boundary);
        foreach (var hole in holes)
            AddRing(hole);
        return segments;

        void AddRing(IReadOnlyList<PointD> ring)
        {
            if (ring.Count < 2)
                return;

            for (int i = 0; i < ring.Count; i++)
            {
                var a = ring[i];
                var b = ring[(i + 1) % ring.Count];
                segments.Add((a, b));
            }
        }
    }

    private static bool PassesCellAngleFilter(
        (double X, double Y) p0,
        (double X, double Y) p1,
        IReadOnlyList<(PointD A, PointD B)> segments,
        double filteringAngle)
    {
        // Approximate JS get_cells_angle by using the two nearest segment sites.
        var midX = (p0.X + p1.X) * 0.5;
        var midY = (p0.Y + p1.Y) * 0.5;

        var nearest = segments
            .Select(s => (seg: s, dist: DistancePointToSegment(midX, midY, s.A.x, s.A.y, s.B.x, s.B.y)))
            .OrderBy(x => x.dist)
            .Take(2)
            .ToArray();

        if (nearest.Length < 2)
            return true;

        var s1 = nearest[0].seg;
        var s2 = nearest[1].seg;

        if (!TryGetSharedEndpoint(s1, s2, out var pivot))
            return true;

        // JS applies this angle test to Voronoi segment-cells that truly share
        // a boundary vertex. Our nearest-segment approximation can pair unrelated
        // edges in the interior, so only enforce angle pruning near that corner.
        double midPivotDistance = Math.Sqrt((midX - pivot.x) * (midX - pivot.x) + (midY - pivot.y) * (midY - pivot.y));
        double d0 = nearest[0].dist;
        double d1 = nearest[1].dist;
        if (d0 <= 1e-9 || d1 <= 1e-9)
            return true;
        if (d0 > d1 * 2.0 || d1 > d0 * 2.0)
            return true;
        if (midPivotDistance > Math.Max(d0, d1) * 1.5)
            return true;

        var a = !PointsEqual(s1.A, pivot) ? s1.A : s1.B;
        var b = !PointsEqual(s2.A, pivot) ? s2.A : s2.B;

        double v1x = a.x - pivot.x;
        double v1y = a.y - pivot.y;
        double v2x = b.x - pivot.x;
        double v2y = b.y - pivot.y;

        double n1 = Math.Sqrt(v1x * v1x + v1y * v1y);
        double n2 = Math.Sqrt(v2x * v2x + v2y * v2y);
        if (n1 <= 1e-9 || n2 <= 1e-9)
            return true;

        double dot = (v1x * v2x + v1y * v2y) / (n1 * n2);
        dot = Math.Clamp(dot, -1.0, 1.0);
        double angle = Math.Acos(dot);

        return angle <= filteringAngle;
    }

    private static bool TryGetSharedEndpoint(
        (PointD A, PointD B) s1,
        (PointD A, PointD B) s2,
        out PointD shared)
    {
        if (PointsEqual(s1.A, s2.A)) { shared = s1.A; return true; }
        if (PointsEqual(s1.A, s2.B)) { shared = s1.A; return true; }
        if (PointsEqual(s1.B, s2.A)) { shared = s1.B; return true; }
        if (PointsEqual(s1.B, s2.B)) { shared = s1.B; return true; }

        shared = default;
        return false;
    }

    private static bool PointsEqual(PointD a, PointD b)
        => Math.Abs(a.x - b.x) <= 1e-9 && Math.Abs(a.y - b.y) <= 1e-9;

    private static IEnumerable<MedialSegment> PostProcessSegments(
        IReadOnlyList<MedialSegment> segments,
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes)
    {
        // JS construct_medial_axis rejects invalid primary edges and NaNs.
        foreach (var seg in segments)
        {
            if (double.IsNaN(seg.Point0.X) || double.IsNaN(seg.Point0.Y) ||
                double.IsNaN(seg.Point1.X) || double.IsNaN(seg.Point1.Y))
                continue;

            if (Math.Abs(seg.Point0.X - seg.Point1.X) <= 1e-12 &&
                Math.Abs(seg.Point0.Y - seg.Point1.Y) <= 1e-12)
                continue;

            var midX = (seg.Point0.X + seg.Point1.X) * 0.5;
            var midY = (seg.Point0.Y + seg.Point1.Y) * 0.5;
            if (!IsInsideRegion(midX, midY, boundary, holes))
                continue;

            yield return seg;
        }
    }

    private static (double X, double Y)? Circumcenter(TriCell cell)
    {
        var p0 = cell.Vertices[0];
        var p1 = cell.Vertices[1];
        var p2 = cell.Vertices[2];

        double d = 2.0 * (p0.X * (p1.Y - p2.Y) + p1.X * (p2.Y - p0.Y) + p2.X * (p0.Y - p1.Y));
        if (Math.Abs(d) < 1e-12)
            return null;

        double p0sq = p0.X * p0.X + p0.Y * p0.Y;
        double p1sq = p1.X * p1.X + p1.Y * p1.Y;
        double p2sq = p2.X * p2.X + p2.Y * p2.Y;

        double ux = (p0sq * (p1.Y - p2.Y) + p1sq * (p2.Y - p0.Y) + p2sq * (p0.Y - p1.Y)) / d;
        double uy = (p0sq * (p2.X - p1.X) + p1sq * (p0.X - p2.X) + p2sq * (p1.X - p0.X)) / d;

        return (ux, uy);
    }

    private static bool IsInsideRegion(
        double x,
        double y,
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes)
    {
        if (!PointInRing(x, y, boundary))
            return false;

        foreach (var hole in holes)
        {
            if (PointInRing(x, y, hole))
                return false;
        }

        return true;
    }

    private static bool PointInRing(double x, double y, IReadOnlyList<PointD> ring)
    {
        bool inside = false;
        for (int i = 0, j = ring.Count - 1; i < ring.Count; j = i++)
        {
            var pi = ring[i];
            var pj = ring[j];

            bool intersects = ((pi.y > y) != (pj.y > y)) &&
                              (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y + double.Epsilon) + pi.x);
            if (intersects)
                inside = !inside;
        }

        return inside;
    }

    private static double DistanceToNearestSegment(double x, double y, IReadOnlyList<(PointD A, PointD B)> segments)
    {
        double best = double.PositiveInfinity;
        foreach (var (a, b) in segments)
        {
            double d = DistancePointToSegment(x, y, a.x, a.y, b.x, b.y);
            if (d < best)
                best = d;
        }

        return best;
    }

    private static double DistancePointToSegment(double px, double py, double ax, double ay, double bx, double by)
    {
        double abx = bx - ax;
        double aby = by - ay;
        double apx = px - ax;
        double apy = py - ay;
        double ab2 = abx * abx + aby * aby;
        if (ab2 <= 1e-18)
            return Math.Sqrt(apx * apx + apy * apy);

        double t = Math.Clamp((apx * abx + apy * aby) / ab2, 0.0, 1.0);
        double cx = ax + abx * t;
        double cy = ay + aby * t;
        double dx = px - cx;
        double dy = py - cy;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    private sealed class SampleVertex : IVertex
    {
        public SampleVertex(int id, double x, double y)
        {
            Id = id;
            X = x;
            Y = y;
            Position = new[] { x, y };
        }

        public int Id { get; }
        public double X { get; }
        public double Y { get; }
        public double[] Position { get; }
    }

    private sealed class TriCell : TriangulationCell<SampleVertex, TriCell>
    {
    }

    private sealed class ReferenceEqualityComparer<T> : IEqualityComparer<T>
        where T : class
    {
        public static readonly ReferenceEqualityComparer<T> Instance = new();
        public bool Equals(T? x, T? y) => ReferenceEquals(x, y);
        public int GetHashCode(T obj) => System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(obj);
    }
}

