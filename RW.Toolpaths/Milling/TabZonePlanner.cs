using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>An arc-length interval on a ring that the cutter must lift over.</summary>
public readonly record struct TabZone(double Start, double End)
{
    public double Length => End - Start;
}

/// <summary>
/// Places holding tabs around a ring and lifts the toolpath over them.
///
/// <para>
/// Tabs are flat-topped with vertical walls rather than ramped. A ramped tab would be shortened
/// at the top by the ramp length, so the retained material would be narrower than requested; a
/// flat top keeps the full configured length at full thickness.
/// </para>
/// </summary>
public static class TabZonePlanner
{
    /// <summary>Cosine below which a vertex counts as a corner (about 32 degrees of turn).</summary>
    private const double CornerTurnCosine = 0.85;

    /// <summary>Zones covering more of the ring than this leave no room to cut, so they are dropped.</summary>
    private const double MaxPerimeterCoverage = 0.9;

    /// <summary>Upper bound on arc positions probed per ring, so a long contour stays cheap.</summary>
    private const int MaxSeatSamples = 360;

    /// <summary>Rotations of the evenly spaced pattern compared before the best one is kept.</summary>
    private const int PhaseTrials = 60;

    /// <summary>Fractions of the tab half-length that must all sit on bridging material.</summary>
    private static readonly double[] SupportSamples = { -0.8, 0.0, 0.8 };

    /// <summary>
    /// Spaces <paramref name="tabCount"/> tabs as evenly as the ring allows, skipping seats that
    /// are not fit to carry a tab.
    ///
    /// <para>
    /// A seat is rejected at a corner, because a tab straddling one is only supported on a single
    /// side and snaps off while the cutter changes direction climbing it. It is also rejected when
    /// <paramref name="bridgesMaterial"/> reports no stock on both sides of the kerf: a tab only
    /// holds if it joins two bodies, and one that reaches over the edge of the stock is machined
    /// into air.
    /// </para>
    /// <para>
    /// Even spacing is the objective, not a constraint. Every rotation of the ideal pattern is
    /// scored by how many tabs it seats, then by the tightest gap it leaves, so a blocked seat
    /// costs a nudge rather than the whole ring's retention.
    /// </para>
    /// </summary>
    public static bool TryBuildEvenlySpaced(
        IReadOnlyList<PointD> ring,
        double perimeter,
        int tabCount,
        double zoneLength,
        double toolRadius,
        out TabZone[] zones,
        Func<PointD, PointD, bool>? bridgesMaterial = null)
    {
        zones = Array.Empty<TabZone>();

        if (ring.Count < 2 || perimeter < 0.001 || tabCount < 1 || zoneLength <= 0)
            return false;

        var loop = RampPlanner.Normalize(ring);
        int vertexCount = loop.Count;
        if (vertexCount < 3)
            return false;

        var vertexArcs = new double[vertexCount];
        var cornerArcs = new List<double>();
        double arc = 0;
        for (int vertex = 0; vertex < vertexCount; vertex++)
        {
            vertexArcs[vertex] = arc;

            int previous = (vertex + vertexCount - 1) % vertexCount;
            int next = (vertex + 1) % vertexCount;

            double inX = loop[vertex].x - loop[previous].x;
            double inY = loop[vertex].y - loop[previous].y;
            double outX = loop[next].x - loop[vertex].x;
            double outY = loop[next].y - loop[vertex].y;
            double inLength = Math.Sqrt(inX * inX + inY * inY);
            double outLength = Math.Sqrt(outX * outX + outY * outY);

            if (inLength > 1e-5 && outLength > 1e-5)
            {
                double turn = (inX * outX + inY * outY) / (inLength * outLength);
                if (turn < CornerTurnCosine)
                    cornerArcs.Add(arc);
            }

            arc += outLength;
        }

        double loopLength = arc;
        if (loopLength < 1e-9)
            return false;

        double halfZone = zoneLength * 0.5;
        double cornerClearance = halfZone + toolRadius;
        double supportHalf = Math.Max(0, halfZone - toolRadius);

        double CircularDistance(double first, double second)
        {
            double distance = Math.Abs(first - second);
            return Math.Min(distance, perimeter - distance);
        }

        bool IsClearOfCorners(double center)
        {
            foreach (double corner in cornerArcs)
            {
                if (CircularDistance(center, corner) < cornerClearance - 1e-4)
                    return false;
            }
            return true;
        }

        bool BridgesOnBothSides(double center)
        {
            if (bridgesMaterial is null)
                return true;

            foreach (double fraction in SupportSamples)
            {
                var (point, normal) = SampleRing(
                    loop, vertexArcs, loopLength, center + supportHalf * fraction);
                if (Math.Abs(normal.x) < 1e-12 && Math.Abs(normal.y) < 1e-12)
                    return false;
                if (!bridgesMaterial(point, normal))
                    return false;
            }
            return true;
        }

        int sampleCount = (int)Math.Clamp(
            Math.Ceiling(perimeter / Math.Max(zoneLength * 0.2, 1e-4)), 16, MaxSeatSamples);
        double sampleStep = perimeter / sampleCount;

        var seatable = new bool[sampleCount];
        int seatableCount = 0;
        for (int i = 0; i < sampleCount; i++)
        {
            double center = i * sampleStep;
            if (!IsClearOfCorners(center) || !BridgesOnBothSides(center))
                continue;

            seatable[i] = true;
            seatableCount++;
        }

        if (seatableCount == 0)
            return false;

        // Never let a tab wander into the slot its neighbour is aiming for.
        int searchRadius = (int)Math.Ceiling(sampleCount / (2.0 * tabCount));

        List<double>? best = null;
        double bestMinimumGap = -1;
        double bestDrift = double.MaxValue;

        for (int trial = 0; trial < PhaseTrials; trial++)
        {
            double phase = perimeter * trial / PhaseTrials;
            var placed = new List<double>(tabCount);
            double drift = 0;

            for (int tabIndex = 0; tabIndex < tabCount; tabIndex++)
            {
                double ideal = phase + perimeter * tabIndex / tabCount;
                int idealIndex = (int)Math.Round(ideal / sampleStep);
                double? seat = null;

                for (int step = 0; step <= searchRadius && seat is null; step++)
                {
                    for (int side = 0; side < 2; side++)
                    {
                        if (step == 0 && side == 1)
                            continue;

                        int offset = side == 0 ? step : -step;
                        int index = ((idealIndex + offset) % sampleCount + sampleCount) % sampleCount;
                        if (!seatable[index])
                            continue;

                        double center = index * sampleStep;
                        if (placed.Any(other => CircularDistance(other, center) < zoneLength - 1e-4))
                            continue;

                        seat = center;
                        break;
                    }
                }

                if (seat is null)
                    continue;

                drift += CircularDistance(seat.Value, ideal);
                placed.Add(seat.Value);
            }

            if (placed.Count == 0)
                continue;

            double minimumGap = MinimumGap(placed, perimeter);
            int bestCount = best?.Count ?? 0;
            bool better = placed.Count > bestCount
                || (placed.Count == bestCount
                    && (minimumGap > bestMinimumGap + 1e-6
                        || (minimumGap > bestMinimumGap - 1e-6 && drift < bestDrift)));

            if (!better)
                continue;

            best = placed;
            bestMinimumGap = minimumGap;
            bestDrift = drift;
        }

        if (best is null)
            return false;

        zones = best
            .OrderBy(center => center)
            .Select(center => new TabZone(center - halfZone, center + halfZone))
            .ToArray();
        return true;
    }

    /// <summary>Point and unit left-hand normal at an arc position along a closed loop.</summary>
    private static (PointD Point, PointD Normal) SampleRing(
        IReadOnlyList<PointD> loop,
        double[] vertexArcs,
        double loopLength,
        double position)
    {
        double target = position % loopLength;
        if (target < 0)
            target += loopLength;

        int index = Array.BinarySearch(vertexArcs, target);
        if (index < 0)
            index = ~index - 1;
        index = Math.Clamp(index, 0, loop.Count - 1);

        var from = loop[index];
        var to = loop[(index + 1) % loop.Count];
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double length = Math.Sqrt(dx * dx + dy * dy);
        if (length < 1e-12)
            return (from, new PointD(0, 0));

        double t = Math.Clamp((target - vertexArcs[index]) / length, 0, 1);
        return (
            new PointD(from.x + dx * t, from.y + dy * t),
            new PointD(-dy / length, dx / length));
    }

    /// <summary>Tightest arc gap between neighbouring tab centres, wrapping the seam.</summary>
    private static double MinimumGap(List<double> centers, double perimeter)
    {
        if (centers.Count < 2)
            return perimeter;

        var sorted = centers.OrderBy(center => center).ToList();
        double minimum = perimeter - (sorted[^1] - sorted[0]);
        for (int i = 1; i < sorted.Count; i++)
            minimum = Math.Min(minimum, sorted[i] - sorted[i - 1]);
        return minimum;
    }

    /// <summary>
    /// Rejects a zone set that leaves the ring with almost nothing left to cut, which means
    /// placement failed rather than that the part needs that much retention.
    /// </summary>
    public static TabZone[]? Normalize(IReadOnlyList<TabZone> zones, double perimeter)
    {
        if (zones.Count == 0 || perimeter < 0.001)
            return null;

        double coverage = zones.Sum(zone => zone.Length);
        if (coverage >= perimeter * MaxPerimeterCoverage)
            return null;

        return zones.ToArray();
    }

    /// <summary>
    /// Validates zones projected from accepted physical tab footprints. Unlike heuristic tab
    /// placement, full-ring coverage is authoritative: dropping it would machine through the
    /// footprint that produced it.
    /// </summary>
    internal static TabZone[]? NormalizeProjected(
        IReadOnlyList<TabZone> zones,
        double perimeter)
    {
        if (zones.Count == 0 || perimeter < 0.001)
            return null;

        var projected = zones
            .Where(zone => zone.Length > 1e-6)
            .ToArray();
        return projected.Length == 0 ? null : projected;
    }

    /// <summary>Z the cutter must hold at arc position <paramref name="arc"/>.</summary>
    public static double ZAt(double arc, IReadOnlyList<TabZone> zones, double perimeter, double cutZ, double tabZ)
    {
        if (tabZ <= cutZ + 1e-5 || perimeter <= 1e-9)
            return cutZ;

        double wrapped = arc % perimeter;
        if (wrapped < 0)
            wrapped += perimeter;

        foreach (var zone in zones)
        {
            if (zone.Length <= 1e-6)
                continue;

            // Zones can straddle the seam, so test the neighbouring periods too.
            for (int offset = -1; offset <= 1; offset++)
            {
                double local = wrapped - (zone.Start + offset * perimeter);
                if (local >= 0 && local <= zone.Length)
                    return tabZ;
            }
        }

        return cutZ;
    }

    /// <summary>
    /// Lifts an existing closed-ring path over its tab zones, splitting moves at every zone
    /// boundary and inserting the vertical walls.
    ///
    /// <para>
    /// Where the path is already above the tab (during a ramp) the higher of the two heights
    /// wins, so an entry move is never dragged down into the material it is climbing out of.
    /// </para>
    /// </summary>
    public static (List<Point3D> Points, List<ToolpathSpan> Spans) ApplyToClosedPath(
        IReadOnlyList<Point3D> points,
        IReadOnlyList<ToolpathSpan> spans,
        IReadOnlyList<TabZone> zones,
        double perimeter,
        double cutZ,
        double tabZ)
    {
        if (points.Count < 2 || zones.Count == 0 || perimeter <= 1e-9 || tabZ <= cutZ + 1e-5)
            return (points.ToList(), spans.ToList());

        var boundaries = new List<double>(zones.Count * 2);
        foreach (var zone in zones)
        {
            boundaries.Add(zone.Start);
            boundaries.Add(zone.End);
        }

        var outputPoints = new List<Point3D>(points.Count + zones.Count * 4);
        var kinds = new List<ToolpathSpanKind>(outputPoints.Capacity);

        var kindByIndex = BuildKindLookup(points.Count, spans);

        double arc = 0;
        Emit(outputPoints, kinds, points[0], arc, kindByIndex[0]);

        for (int i = 1; i < points.Count; i++)
        {
            var from = points[i - 1];
            var to = points[i];
            double dx = to.X - from.X;
            double dy = to.Y - from.Y;
            double length = Math.Sqrt(dx * dx + dy * dy);
            var kind = kindByIndex[i];

            if (length <= 1e-12)
            {
                Emit(outputPoints, kinds, to, arc, kind);
                continue;
            }

            // Split this move wherever it crosses into or out of a tab.
            var crossings = new List<double>();
            double segmentEnd = arc + length;
            foreach (double boundary in boundaries)
            {
                double absolute = boundary
                    + Math.Floor((arc - boundary) / perimeter) * perimeter;
                if (absolute <= arc + 1e-9)
                    absolute += perimeter;

                for (; absolute < segmentEnd - 1e-9; absolute += perimeter)
                {
                    double t = (absolute - arc) / length;
                    crossings.Add(t);
                }
            }
            crossings.Sort();

            double previousT = 0;
            foreach (double t in crossings)
            {
                if (t - previousT <= 1e-12)
                    continue;

                var split = new Point3D(from.X + dx * t, from.Y + dy * t, Lerp(from.Z, to.Z, t));
                double splitArc = arc + length * t;

                // Reach the boundary at the height of the run just travelled, then step
                // vertically to the height the next run needs.
                Emit(outputPoints, kinds, split, splitArc - 1e-7, kind);
                Emit(outputPoints, kinds, split, splitArc + 1e-7, kind);
                previousT = t;
            }

            arc += length;
            Emit(outputPoints, kinds, to, arc, kind);
        }

        return (outputPoints, BuildSpans(kinds));

        void Emit(List<Point3D> target, List<ToolpathSpanKind> targetKinds, Point3D point, double at, ToolpathSpanKind kind)
        {
            double lifted = ZAt(at, zones, perimeter, cutZ, tabZ);
            double z = Math.Max(point.Z, lifted);
            bool onTab = z > cutZ + 1e-5;

            var resolved = onTab && kind == ToolpathSpanKind.Cut ? ToolpathSpanKind.TabLift : kind;
            var emitted = new Point3D(point.X, point.Y, z);

            if (target.Count > 0)
            {
                var last = target[^1];
                if (Math.Abs(last.X - emitted.X) < 1e-12 &&
                    Math.Abs(last.Y - emitted.Y) < 1e-12 &&
                    Math.Abs(last.Z - emitted.Z) < 1e-12)
                    return;
            }

            target.Add(emitted);
            targetKinds.Add(resolved);
        }
    }

    private static double Lerp(double a, double b, double t) => a + (b - a) * t;

    private static ToolpathSpanKind[] BuildKindLookup(int pointCount, IReadOnlyList<ToolpathSpan> spans)
    {
        var kinds = new ToolpathSpanKind[pointCount];
        for (int i = 0; i < pointCount; i++)
            kinds[i] = ToolpathSpanKind.Cut;

        foreach (var span in spans)
        {
            for (int i = Math.Max(0, span.StartIndex); i <= Math.Min(pointCount - 1, span.EndIndex); i++)
                kinds[i] = span.Kind;
        }
        return kinds;
    }

    private static List<ToolpathSpan> BuildSpans(IReadOnlyList<ToolpathSpanKind> kinds)
    {
        var spans = new List<ToolpathSpan>();
        if (kinds.Count == 0)
            return spans;

        int start = 0;
        for (int i = 1; i < kinds.Count; i++)
        {
            if (kinds[i] == kinds[start])
                continue;

            spans.Add(new ToolpathSpan(start, i, kinds[start]));
            start = i;
        }

        spans.Add(new ToolpathSpan(start, kinds.Count - 1, kinds[start]));
        return spans;
    }
}
