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
    internal RampedPath(List<Point3D> points, List<ToolpathSpan> spans, RampStrategy strategy, int laps,
        int redundantPointsRemoved = 0, double redundantCutTravelRemoved = 0, bool partialAxisFallback = false)
    {
        Points = points;
        Spans = spans;
        Strategy = strategy;
        Laps = laps;
        RedundantPointsRemoved = redundantPointsRemoved;
        RedundantCutTravelRemoved = redundantCutTravelRemoved;
        PartialAxisFallback = partialAxisFallback;
    }

    public List<Point3D> Points { get; }

    public List<ToolpathSpan> Spans { get; }

    /// <summary>Strategy actually used, which may differ from the request when Auto was set.</summary>
    public RampStrategy Strategy { get; }

    /// <summary>Helical laps spent descending; 0 for other strategies.</summary>
    public int Laps { get; }

    internal int RedundantPointsRemoved { get; }

    internal double RedundantCutTravelRemoved { get; }

    internal bool PartialAxisFallback { get; }
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
    /// Enters an existing cutting polyline from above without descending faster than
    /// the configured ramp angle or below any original cutting chord. The ramp lands
    /// at its first reachable local target contact, which may be shallower than the
    /// operation's nominal depth. Every original directed XYZ cutting chord is
    /// subsequently executed whole at its authored depth. Public closed entries
    /// retain their legacy closing seam: downstream consumers may append an
    /// already-authorized at-depth tail or apply a later placement transform.
    /// Removing a repeated suffix without those consumers' endpoint and posted
    /// motion contracts can change the outgoing chord and its removed stock.
    /// </summary>
    public static RampedPath PlanPathEntry(
        IReadOnlyList<Point3D> path,
        double entryZ,
        RampSettings settings,
        CancellationToken cancellationToken = default,
        Action<string>? diagnostic = null,
        bool landAtSourceVertex = false)
        => PlanPathEntry(path, entryZ, settings, cancellationToken, diagnostic,
            landAtSourceVertex, preserveClosedExit: true);

    // Retain the legacy public CLR signature and exit. Internal Pocket and
    // V-carve routing opt into reduction only for final command coordinates and
    // visits whose outgoing motion has an endpoint-aware stock proof.
    internal static RampedPath PlanPathEntry(
        IReadOnlyList<Point3D> path,
        double entryZ,
        RampSettings settings,
        CancellationToken cancellationToken,
        Action<string>? diagnostic,
        bool landAtSourceVertex,
        bool preserveClosedExit)
    {
        ArgumentNullException.ThrowIfNull(path);
        ArgumentNullException.ThrowIfNull(settings);
        cancellationToken.ThrowIfCancellationRequested();
        if (path.Count < 2)
            return new RampedPath(new List<Point3D>(), new List<ToolpathSpan>(), RampStrategy.Linear, 0);
        if (entryZ <= path[0].Z + 1e-9)
        {
            var unchanged = path.ToList();
            return new RampedPath(unchanged,
                new List<ToolpathSpan> { new(0, unchanged.Count - 1, ToolpathSpanKind.Cut) }, RampStrategy.Linear, 0);
        }

        double slope = Math.Tan(settings.AngleRadians);
        if (settings.Strategy == RampStrategy.Plunge || !double.IsFinite(slope) || slope <= 1e-9)
            throw new InvalidOperationException("Skipping depth passes requires a positive ramp angle.");

        bool closed = path[0] == path[^1];
        bool partialAxisFallback = false;
        double length = 0;
        int distinctSourceEdges = 0;
        for (int i = 1; i < path.Count; i++)
        {
            if ((i & 1023) == 0) cancellationToken.ThrowIfCancellationRequested();
            length += PlanarDistance(path[i - 1], path[i]);
            if (path[i - 1] != path[i]) distinctSourceEdges++;
            if (closed && !partialAxisFallback)
                partialAxisFallback = RequiresLegacyClosedTail(path[i - 1], path[i]);
        }
        if (length <= 1e-9)
        {
            diagnostic?.Invoke("stage=ramp-entry event=reject reason=no-horizontal-entry-path");
            throw new InvalidOperationException("The cutting path has no horizontal length for a ramp entry.");
        }

        var points = new List<Point3D> { new(path[0].X, path[0].Y, entryZ) };
        int fromIndex = 0, direction = 1, traversals = 0, contactSegment = -1;
        double travelled = 0;
        // Match the existing carving path budget without steepening a tiny ring's
        // entry when the configured angle requires more than MaxLaps revolutions.
        const int MaximumEntryPoints = 2_000_000;
        while (contactSegment < 0)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (points.Count >= MaximumEntryPoints)
            {
                diagnostic?.Invoke($"stage=ramp-entry event=reject reason=point-budget " +
                    $"points={points.Count} traversals={traversals} requestedSlope={slope:R}");
                throw new InvalidOperationException("Ramp entry exceeds two million points; lengthen the entry path or increase the ramp angle.");
            }

            int toIndex = fromIndex + direction;
            Point3D from = path[fromIndex], to = path[toIndex];
            double distance = PlanarDistance(from, to);
            double currentZ = points[^1].Z;
            double nextZ = currentZ - distance * slope;
            double gapStart = currentZ - from.Z, gapEnd = nextZ - to.Z;
            if (gapEnd <= 1e-9)
            {
                // A physically certified command chord must retain its source XY
                // endpoints: rounding a synthetic contact can cross a nearby wall.
                // Landing at the next vertex stays above the original chord and
                // descends no faster than the requested ramp plane.
                double t = landAtSourceVertex ? 1
                    : gapStart <= 1e-9 ? 0 : Math.Clamp(gapStart / (gapStart - gapEnd), 0, 1);
                Point3D contact = landAtSourceVertex ? to : new Point3D(from.X + (to.X - from.X) * t,
                    from.Y + (to.Y - from.Y) * t, from.Z + (to.Z - from.Z) * t);
                if (closed && !partialAxisFallback)
                    partialAxisFallback = RequiresLegacyClosedTail(points[^1], contact);
                points.Add(contact);
                travelled += distance * t;
                contactSegment = Math.Min(fromIndex, toIndex);
                fromIndex = toIndex;
                break;
            }
            if (distance > 1e-12)
            {
                var rampPoint = new Point3D(to.X, to.Y, nextZ);
                if (closed && !partialAxisFallback)
                    partialAxisFallback = RequiresLegacyClosedTail(points[^1], rampPoint);
                points.Add(rampPoint);
            }
            travelled += distance;
            fromIndex = toIndex;
            if (fromIndex == path.Count - 1)
            {
                traversals++;
                if (closed) fromIndex = 0;
                else direction = -1;
            }
            else if (fromIndex == 0 && direction < 0)
            {
                traversals++;
                direction = 1;
            }
        }

        int cutStart = points.Count - 1;
        int redundantPointsRemoved = 0;
        double redundantCutTravelRemoved = 0;
        if (closed)
        {
            // Retain the existing contact-to-seam suffix, including an interior
            // contact's partial chord. Then execute only the missing source prefix.
            // Each original directed XYZ chord is still executed whole once: an
            // interior contact's chord is recut whole in the prefix, rather than
            // split at a new endpoint that could change its rounded depth profile.
            // The omitted tail consists of exact source chords already cut in
            // the retained suffix. Partial-axis command classification can leave
            // different planar cursors on different laps, so preserve those tails.
            for (int i = contactSegment + 1; i < path.Count; i++)
            {
                if ((i & 1023) == 0) cancellationToken.ThrowIfCancellationRequested();
                if (!partialAxisFallback)
                    partialAxisFallback = RequiresLegacyClosedTail(points[^1], path[i]);
                AppendDistinct(points, path[i]);
            }
            // Callers retaining a dependent outgoing seam keep the old exit.
            // A vertex contact with an endpoint-aware outgoing link instead
            // executes one complete rotated cycle and exits at that contact.
            int prefixEnd = preserveClosedExit || partialAxisFallback ? path.Count - 1
                : points[cutStart] == path[contactSegment] ? contactSegment : contactSegment + 1;
            int suffixCount = points.Count;
            for (int i = 0; i <= prefixEnd; i++)
            {
                if ((i & 1023) == 0) cancellationToken.ThrowIfCancellationRequested();
                AppendDistinct(points, path[i]);
            }
            redundantPointsRemoved = distinctSourceEdges - (points.Count - suffixCount);
            for (int i = prefixEnd + 1; i < path.Count; i++)
            {
                if ((i & 1023) == 0) cancellationToken.ThrowIfCancellationRequested();
                Point3D a = path[i - 1], b = path[i];
                double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
                redundantCutTravelRemoved += Math.Sqrt(dx * dx + dy * dy + dz * dz);
            }
        }
        else
        {
            // An open path has no permissible closing chord. Return along its
            // exact prefix before recutting the complete original directed trace.
            for (int i = contactSegment; i >= 0; i--)
                AppendDistinct(points, path[i]);
            foreach (Point3D point in path)
                AppendDistinct(points, point);
        }
        var spans = new List<ToolpathSpan>
        {
            new(0, cutStart, ToolpathSpanKind.Ramp),
            new(cutStart, points.Count - 1, ToolpathSpanKind.Cut),
        };
        diagnostic?.Invoke($"stage=ramp-entry event=complete sourcePoints={path.Count} " +
            $"entryZ={entryZ:R} sourceStartZ={path[0].Z:R} contactZ={points[cutStart].Z:R} " +
            $"requestedSlope={slope:R} horizontalRun={travelled:R} traversals={traversals} " +
            $"points={points.Count} redundantPointsRemoved={redundantPointsRemoved} " +
            $"closedTailReductionAllowed={!(preserveClosedExit || partialAxisFallback)} partialAxisFallback={partialAxisFallback} " +
            $"cutCycleClosed={closed && points[cutStart] == points[^1]} cutMoves={points.Count - cutStart - 1} " +
            $"redundantCutTravelRemoved={redundantCutTravelRemoved:R} " +
            $"finalExit=({points[^1].X:R},{points[^1].Y:R},{points[^1].Z:R}) " +
            $"localTargetContact=True sourceVertexContact={landAtSourceVertex} originalCutTracePreserved=True");
        return new RampedPath(points, spans,
            closed && traversals > 0 ? RampStrategy.Helical : RampStrategy.Linear,
            closed ? traversals : 0, redundantPointsRemoved, redundantCutTravelRemoved, partialAxisFallback);
    }

    private static bool RequiresLegacyClosedTail(Point3D a, Point3D b)
    {
        // Use the same command-shape owner as Shared. Subdivision never turns
        // a sloped chord into an axis-only command. Even sub-float native drift
        // is harmless here when its two actual F3 planar endpoints coincide.
        float ax = (float)a.X, ay = (float)a.Y, az = (float)a.Z;
        float bx = (float)b.X, by = (float)b.Y, bz = (float)b.Z;
        if (!float.IsFinite(ax) || !float.IsFinite(ay) || !float.IsFinite(az)
            || !float.IsFinite(bx) || !float.IsFinite(by) || !float.IsFinite(bz)) return true;
        float dz = bz - az;
        if (!float.IsFinite(dz)) return true;
        float dx = bx - ax, dy = by - ay;
        if (!float.IsFinite(dx) || !float.IsFinite(dy)) return true;
        return ToolpathCommandRounding.IsVerticalOnly(dx,dz,dy)
            && (ToolpathCommandRounding.Round(ax)!=ToolpathCommandRounding.Round(bx)
                || ToolpathCommandRounding.Round(ay)!=ToolpathCommandRounding.Round(by));
    }

    private static double PlanarDistance(Point3D a, Point3D b)
        => Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));

    private static void AppendDistinct(List<Point3D> points, Point3D point)
    {
        if (points[^1] != point)
            points.Add(point);
    }

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

    /// <summary>Pocket-only entry policy. Replaying the same immutable command
    /// chords at every scheduled depth is stronger than replaying an XY curve:
    /// a rounded interior contact can split a chord differently on later passes.
    /// Public entry defaults and other machining strategies remain unchanged.</summary>
    internal static RampedPath PlanClosedRingAtCommandVertices(
        IReadOnlyList<PointD> ring, double entryZ, double cutZ, RampSettings settings,
        CancellationToken token = default, Action<string>? diagnostic = null)
    {
        token.ThrowIfCancellationRequested();
        var loop = Normalize(ring);
        double perimeter = ClosedPerimeter(loop);
        double drop = entryZ - cutZ;
        double run = settings.HorizontalRunFor(drop);
        var strategy = Resolve(settings, run, perimeter);
        if (loop.Count < 2 || drop <= 1e-9 || perimeter <= 1e-9
            || strategy == RampStrategy.Plunge
            || strategy == RampStrategy.Helical && Math.Ceiling(run / perimeter) <= Math.Max(1, settings.MaxLaps))
        {
            var wholeVertexEntry = PlanClosedRing(loop, entryZ, cutZ, settings);
            token.ThrowIfCancellationRequested();
            return wholeVertexEntry;
        }
        // Descent follows every whole original edge; the final edge clamps its
        // height drop to the remaining depth, producing a shallower grade. Cut
        // exactly one complete directed cycle from that original contact vertex.
        // Pocket's outgoing link is checked from the actual resulting endpoint;
        // retaining the former closing seam would recut an unnecessary suffix
        // and make the complete cycle appear open to chronological routing.
        var source = loop.Select(p => new Point3D(p.x, p.y, cutZ)).ToList();
        source.Add(source[0]);
        return PlanPathEntry(source, entryZ, settings, token, diagnostic,
            landAtSourceVertex: true, preserveClosedExit: false);
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
