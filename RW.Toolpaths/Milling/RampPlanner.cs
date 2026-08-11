using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>How the cutter gets from clearance height down to the pass depth.</summary>
public enum RampStrategy
{
    /// <summary>Pick per ring based on how much of the ramp the ring's perimeter can absorb.</summary>
    Auto = 0,

    /// <summary>Straight down. Only safe with centre-cutting tools in soft material.</summary>
    Plunge = 1,

    /// <summary>Descend along the ring until depth is reached, then finish the ring flat.</summary>
    Linear = 2,

    /// <summary>Spiral around the ring over as many laps as the ramp angle requires.</summary>
    Helical = 3,
}

/// <summary>Entry-move configuration.</summary>
/// <param name="AngleRadians">
/// Descent angle from horizontal. Small angles are gentle and slow; zero means plunge.
/// </param>
public sealed record RampSettings(double AngleRadians)
{
    public RampStrategy Strategy { get; init; } = RampStrategy.Auto;

    /// <summary>
    /// Cap on helical laps. Bounds the entry move when a ring is very small relative to the
    /// requested ramp angle, where the exact angle would otherwise cost many revolutions.
    /// </summary>
    public int MaxLaps { get; init; } = 8;

    /// <summary>Straight-down entry.</summary>
    public static readonly RampSettings Plunge = new(0) { Strategy = RampStrategy.Plunge };

    /// <summary>
    /// Builds settings from a horizontal-travel-per-unit-of-descent ratio, the form CAM
    /// operators usually configure (a ratio of 2 travels 2mm across for every 1mm down).
    /// </summary>
    public static RampSettings FromRatio(double horizontalPerVertical)
    {
        if (horizontalPerVertical <= 0)
            return Plunge;
        return new RampSettings(Math.Atan(1.0 / horizontalPerVertical));
    }

    /// <summary>Horizontal distance needed to descend <paramref name="drop"/>.</summary>
    public double HorizontalRunFor(double drop)
    {
        if (AngleRadians <= 0 || drop <= 0)
            return 0;

        double tan = Math.Tan(AngleRadians);
        return tan <= 1e-9 ? double.PositiveInfinity : drop / tan;
    }
}

/// <summary>A ring's entry move plus the cutting pass that follows it.</summary>
public sealed class RampedPath
{
    internal RampedPath(List<Point3D> points, List<ToolpathSpan> spans, RampStrategy strategy, int laps)
    {
        Points = points;
        Spans = spans;
        Strategy = strategy;
        Laps = laps;
    }

    public List<Point3D> Points { get; }

    public List<ToolpathSpan> Spans { get; }

    /// <summary>Strategy actually used, which may differ from the request when Auto was set.</summary>
    public RampStrategy Strategy { get; }

    /// <summary>Helical laps spent descending; 0 for other strategies.</summary>
    public int Laps { get; }
}

/// <summary>
/// Plans how the cutter enters the material for one closed ring at one depth pass.
///
/// <para>
/// Every strategy guarantees the ring is fully cut at the pass depth: a ramp leaves an
/// uncut wedge behind it, so the ramped arc is always machined again at final depth before
/// the pass is considered complete.
/// </para>
/// </summary>
public static class RampPlanner
{
    /// <summary>
    /// Produces the entry move and the full-depth cutting pass for one closed ring.
    /// </summary>
    /// <param name="ring">Closed ring; a repeated closing vertex is tolerated.</param>
    /// <param name="entryZ">Z the cutter starts from, normally the previous pass depth.</param>
    /// <param name="cutZ">Z at the bottom of this pass. Must be below <paramref name="entryZ"/>.</param>
    /// <param name="settings">Entry configuration.</param>
    public static RampedPath PlanClosedRing(
        IReadOnlyList<PointD> ring,
        double entryZ,
        double cutZ,
        RampSettings settings)
    {
        var loop = Normalize(ring);
        var points = new List<Point3D>();
        var spans = new List<ToolpathSpan>();

        if (loop.Count < 2)
            return new RampedPath(points, spans, RampStrategy.Plunge, 0);

        double drop = entryZ - cutZ;
        double perimeter = ClosedPerimeter(loop);

        if (drop <= 1e-9 || perimeter <= 1e-9)
        {
            points.Add(new Point3D(loop[0].x, loop[0].y, cutZ));
            AppendArc(points, loop, cutZ, 0, perimeter);
            spans.Add(new ToolpathSpan(0, points.Count - 1, ToolpathSpanKind.Cut));
            return new RampedPath(points, spans, RampStrategy.Plunge, 0);
        }

        double run = settings.HorizontalRunFor(drop);
        var strategy = Resolve(settings, run, perimeter);

        return strategy switch
        {
            RampStrategy.Plunge => PlanPlunge(loop, entryZ, cutZ, perimeter),
            RampStrategy.Helical => PlanHelical(loop, entryZ, cutZ, perimeter, run, settings.MaxLaps),
            _ => PlanLinear(loop, entryZ, cutZ, perimeter, run),
        };
    }

    private static RampStrategy Resolve(RampSettings settings, double run, double perimeter)
    {
        if (settings.Strategy != RampStrategy.Auto)
            return settings.Strategy;

        if (settings.AngleRadians <= 0 || double.IsInfinity(run) || run <= 1e-9)
            return RampStrategy.Plunge;

        // A ring shorter than the ramp needs more than one lap to hold the requested angle.
        return run > perimeter ? RampStrategy.Helical : RampStrategy.Linear;
    }

    // --- Strategies -----------------------------------------------------------

    private static RampedPath PlanPlunge(
        List<PointD> loop, double entryZ, double cutZ, double perimeter)
    {
        var points = new List<Point3D>
        {
            new(loop[0].x, loop[0].y, entryZ),
            new(loop[0].x, loop[0].y, cutZ),
        };
        var spans = new List<ToolpathSpan> { new(0, 1, ToolpathSpanKind.Ramp) };

        int cutStart = points.Count - 1;
        AppendArc(points, loop, cutZ, 0, perimeter);
        spans.Add(new ToolpathSpan(cutStart, points.Count - 1, ToolpathSpanKind.Cut));

        return new RampedPath(points, spans, RampStrategy.Plunge, 0);
    }

    /// <summary>
    /// Descends along the ring, then cuts the remainder flat and returns over the ramped arc
    /// so the whole ring ends up at full depth.
    /// </summary>
    private static RampedPath PlanLinear(
        List<PointD> loop, double entryZ, double cutZ, double perimeter, double run)
    {
        double rampLength = Math.Min(run, perimeter);
        double drop = entryZ - cutZ;

        var points = new List<Point3D> { new(loop[0].x, loop[0].y, entryZ) };

        double travelled = 0;
        int index = 0;
        int guard = MaxSteps(loop.Count);

        while (travelled < rampLength - 1e-12 && guard-- > 0)
        {
            var from = loop[index % loop.Count];
            var to = loop[(index + 1) % loop.Count];
            double edge = Distance(from, to);
            index++;

            if (edge <= 1e-12)
                continue;

            if (travelled + edge >= rampLength)
            {
                // Land exactly on depth partway along this edge.
                double t = (rampLength - travelled) / edge;
                points.Add(new Point3D(
                    from.x + (to.x - from.x) * t,
                    from.y + (to.y - from.y) * t,
                    cutZ));
                travelled = rampLength;
                break;
            }

            travelled += edge;
            points.Add(new Point3D(to.x, to.y, entryZ - drop * (travelled / rampLength)));
        }

        var spans = new List<ToolpathSpan> { new(0, points.Count - 1, ToolpathSpanKind.Ramp) };

        // Finish the ring at depth from where the ramp ended, then sweep back over the ramped
        // arc, which is still holding the wedge of material the descent could not reach.
        int cutStart = points.Count - 1;
        AppendArc(points, loop, cutZ, travelled, perimeter);
        AppendArc(points, loop, cutZ, 0, travelled);
        spans.Add(new ToolpathSpan(cutStart, points.Count - 1, ToolpathSpanKind.Cut));

        return new RampedPath(points, spans, RampStrategy.Linear, 0);
    }

    /// <summary>
    /// Spirals down over as many laps as the ramp angle needs, then cuts one clean lap at depth.
    /// </summary>
    private static RampedPath PlanHelical(
        List<PointD> loop, double entryZ, double cutZ, double perimeter, double run, int maxLaps)
    {
        int laps = (int)Math.Ceiling(run / perimeter);
        laps = Math.Clamp(laps, 1, Math.Max(1, maxLaps));

        double total = perimeter * laps;
        double drop = entryZ - cutZ;

        var points = new List<Point3D> { new(loop[0].x, loop[0].y, entryZ) };

        double travelled = 0;
        for (int lap = 0; lap < laps; lap++)
        {
            for (int i = 0; i < loop.Count; i++)
            {
                var from = loop[i];
                var to = loop[(i + 1) % loop.Count];
                double edge = Distance(from, to);
                if (edge <= 1e-12)
                    continue;

                travelled += edge;
                double t = Math.Min(travelled / total, 1.0);
                points.Add(new Point3D(to.x, to.y, entryZ - drop * t));
            }
        }

        var spans = new List<ToolpathSpan> { new(0, points.Count - 1, ToolpathSpanKind.Ramp) };

        int cutStart = points.Count - 1;
        AppendArc(points, loop, cutZ, 0, perimeter);
        spans.Add(new ToolpathSpan(cutStart, points.Count - 1, ToolpathSpanKind.Cut));

        return new RampedPath(points, spans, RampStrategy.Helical, laps);
    }

    // --- Ring helpers ---------------------------------------------------------

    /// <summary>
    /// Index of the ring vertex nearest a point. Large rings are sampled coarsely and then
    /// refined locally rather than scanned exhaustively.
    /// </summary>
    public static int FindNearestVertex(IReadOnlyList<PointD> ring, PointD target)
    {
        int count = ring.Count;
        if (count <= 1)
            return 0;

        if (count <= 64)
        {
            int best = 0;
            double bestDistance = double.MaxValue;
            for (int i = 0; i < count; i++)
            {
                double d = DistanceSq(ring[i], target);
                if (d < bestDistance)
                {
                    bestDistance = d;
                    best = i;
                }
            }
            return best;
        }

        int step = Math.Max(2, count / 24);
        int coarseBest = 0;
        double coarseDistance = double.MaxValue;

        for (int i = 0; i < count; i += step)
        {
            double d = DistanceSq(ring[i], target);
            if (d < coarseDistance)
            {
                coarseDistance = d;
                coarseBest = i;
            }
        }

        double tailDistance = DistanceSq(ring[count - 1], target);
        if (tailDistance < coarseDistance)
        {
            coarseDistance = tailDistance;
            coarseBest = count - 1;
        }

        int radius = Math.Min(count - 1, step * 2);
        int refined = coarseBest;
        double refinedDistance = coarseDistance;

        for (int offset = -radius; offset <= radius; offset++)
        {
            int index = ((coarseBest + offset) % count + count) % count;
            double d = DistanceSq(ring[index], target);
            if (d < refinedDistance)
            {
                refinedDistance = d;
                refined = index;
            }
        }

        return refined;
    }

    /// <summary>Rotates a closed ring so it begins at <paramref name="startIndex"/>.</summary>
    public static List<PointD> RotateTo(IReadOnlyList<PointD> ring, int startIndex)
    {
        var loop = Normalize(ring);
        if (loop.Count == 0)
            return loop;

        int start = ((startIndex % loop.Count) + loop.Count) % loop.Count;
        if (start == 0)
            return loop;

        var rotated = new List<PointD>(loop.Count);
        for (int i = 0; i < loop.Count; i++)
            rotated.Add(loop[(start + i) % loop.Count]);
        return rotated;
    }

    /// <summary>Strips a duplicated closing vertex so edges can be walked cyclically.</summary>
    internal static List<PointD> Normalize(IReadOnlyList<PointD> ring)
    {
        var loop = new List<PointD>(ring);
        while (loop.Count > 1 &&
               Math.Abs(loop[0].x - loop[^1].x) < 1e-12 &&
               Math.Abs(loop[0].y - loop[^1].y) < 1e-12)
        {
            loop.RemoveAt(loop.Count - 1);
        }
        return loop;
    }

    internal static double ClosedPerimeter(IReadOnlyList<PointD> loop)
    {
        double total = 0;
        for (int i = 0; i < loop.Count; i++)
            total += Distance(loop[i], loop[(i + 1) % loop.Count]);
        return total;
    }

    /// <summary>
    /// Emits the portion of the ring between two arc-length positions at a fixed Z, inserting
    /// interpolated points where the range starts or ends partway along an edge.
    /// </summary>
    private static void AppendArc(
        List<Point3D> points,
        IReadOnlyList<PointD> loop,
        double z,
        double fromArc,
        double toArc)
    {
        if (toArc - fromArc <= 1e-12)
            return;

        double arc = 0;
        int guard = MaxSteps(loop.Count);
        int i = 0;

        while (arc < toArc - 1e-12 && guard-- > 0)
        {
            var from = loop[i % loop.Count];
            var to = loop[(i + 1) % loop.Count];
            i++;

            double edge = Distance(from, to);
            if (edge <= 1e-12)
                continue;

            double edgeStart = arc;
            double edgeEnd = arc + edge;
            arc = edgeEnd;

            if (edgeEnd <= fromArc + 1e-12)
                continue;

            // Enter partway along this edge.
            if (edgeStart < fromArc)
            {
                double t = (fromArc - edgeStart) / edge;
                AddDistinct(points, new Point3D(
                    from.x + (to.x - from.x) * t,
                    from.y + (to.y - from.y) * t,
                    z));
            }

            // Leave partway along this edge.
            if (edgeEnd > toArc)
            {
                double t = (toArc - edgeStart) / edge;
                AddDistinct(points, new Point3D(
                    from.x + (to.x - from.x) * t,
                    from.y + (to.y - from.y) * t,
                    z));
                return;
            }

            AddDistinct(points, new Point3D(to.x, to.y, z));
        }
    }

    private static void AddDistinct(List<Point3D> points, Point3D point)
    {
        if (points.Count > 0)
        {
            var last = points[^1];
            if (Math.Abs(last.X - point.X) < 1e-12 &&
                Math.Abs(last.Y - point.Y) < 1e-12 &&
                Math.Abs(last.Z - point.Z) < 1e-12)
                return;
        }
        points.Add(point);
    }

    /// <summary>Bounds cyclic walks so a degenerate ring cannot spin forever.</summary>
    private static int MaxSteps(int vertexCount) => Math.Max(16, vertexCount * 8);

    private static double Distance(PointD a, PointD b)
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    private static double DistanceSq(PointD a, PointD b)
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return dx * dx + dy * dy;
    }
}
