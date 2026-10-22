using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>
/// Consolidates already proved strip traces using short, independently proved bridges.
/// Every original trace chord is retained; a closed trace retains its closing chord.
/// </summary>
internal static class PocketStripRouter
{
    internal static List<List<PointD>> Route(
        IReadOnlyList<List<PointD>> traces,
        double pitch,
        Func<PointD, PointD, bool> allows,
        CancellationToken cancellationToken,
        out int bridges)
        => Route(traces, pitch, allows, cancellationToken, out bridges, out _);

    internal static List<List<PointD>> Route(
        IReadOnlyList<List<PointD>> traces,
        double pitch,
        Func<PointD, PointD, bool> allows,
        CancellationToken cancellationToken,
        out int bridges,
        out int rejected)
    {
        ArgumentNullException.ThrowIfNull(traces);
        ArgumentNullException.ThrowIfNull(allows);
        if (!double.IsFinite(pitch) || pitch <= 0)
            throw new ArgumentOutOfRangeException(nameof(pitch));

        cancellationToken.ThrowIfCancellationRequested();
        bridges = 0;
        rejected = 0;
        var routes = new List<List<PointD>>();
        if (traces.Count == 0)
            return routes;

        // Endpoints, rather than all vertices, bound the neighbourhood searches.
        // Buckets are one maximum bridge length wide, including negative coordinates.
        var grid = new Dictionary<(long X, long Y), List<Endpoint>>();
        var lengths = new double[traces.Count];
        var used = new bool[traces.Count];
        for (int index = 0; index < traces.Count; index++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            List<PointD> trace = traces[index];
            if (trace.Count == 0)
            {
                used[index] = true;
                continue;
            }
            AddEndpoint(new Endpoint(index, true, trace[0]));
            AddEndpoint(new Endpoint(index, false, trace[^1]));
            for (int point = 1; point < trace.Count; point++)
                lengths[index] += Math.Sqrt(DistanceSquared(trace[point - 1], trace[point]));
        }

        var seedOrder = Enumerable.Range(0, traces.Count)
            .OrderByDescending(index => lengths[index])
            .ThenBy(index => index)
            .ToArray();
        var proofCache = new Dictionary<(double AX, double AY, double BX, double BY), bool>();
        double pitchSquared = pitch * pitch;
        int refused = 0;

        foreach (int seed in seedOrder)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (used[seed])
                continue;

            used[seed] = true;
            // Store whole oriented traces as chunks so extending the front is constant
            // time and cannot rewrite, simplify, or accidentally drop source chords.
            var chunks = new LinkedList<OrientedTrace>();
            chunks.AddLast(new OrientedTrace(seed, false));
            PointD head = traces[seed][0], tail = traces[seed][^1];

            while (true)
            {
                cancellationToken.ThrowIfCancellationRequested();
                Candidate? atHead = FindNearest(head, true);
                Candidate? atTail = FindNearest(tail, false);
                Candidate? next = Pick(atHead, atTail);
                if (next is null)
                    break;

                Candidate selected = next.Value;
                int traceIndex = selected.Endpoint.TraceIndex;
                used[traceIndex] = true;
                bool reverse = selected.AtHead ? selected.Endpoint.Start : !selected.Endpoint.Start;
                var chunk = new OrientedTrace(traceIndex, reverse);
                PointD farEnd = selected.Endpoint.Start ? traces[traceIndex][^1] : traces[traceIndex][0];
                if (selected.AtHead)
                {
                    chunks.AddFirst(chunk);
                    head = farEnd;
                }
                else
                {
                    chunks.AddLast(chunk);
                    tail = farEnd;
                }
                if (selected.DistanceSquared > 0)
                    bridges++;
            }

            var route = new List<PointD>();
            foreach (OrientedTrace chunk in chunks)
            {
                cancellationToken.ThrowIfCancellationRequested();
                List<PointD> trace = traces[chunk.TraceIndex];
                int first = chunk.Reverse ? trace.Count - 1 : 0;
                int step = chunk.Reverse ? -1 : 1;
                for (int index = first; index >= 0 && index < trace.Count; index += step)
                {
                    // Removing only the repeated shared endpoint retains both adjacent
                    // chords. Distinct endpoints explicitly emit the proved bridge.
                    if (index == first && route.Count > 0 && SamePoint(route[^1], trace[index]))
                        continue;
                    route.Add(trace[index]);
                }
            }
            routes.Add(route);
        }

        rejected = refused;
        return routes;

        void AddEndpoint(Endpoint endpoint)
        {
            (long X, long Y) cell = Cell(endpoint.Point);
            if (!grid.TryGetValue(cell, out List<Endpoint>? endpoints))
                grid.Add(cell, endpoints = new List<Endpoint>());
            endpoints.Add(endpoint);
        }

        (long X, long Y) Cell(PointD point)
            => ((long)Math.Floor(point.x / pitch), (long)Math.Floor(point.y / pitch));

        Candidate? FindNearest(PointD frontier, bool atHead)
        {
            (long X, long Y) cell = Cell(frontier);
            var candidates = new List<Candidate>();
            for (long dx = -1; dx <= 1; dx++)
            for (long dy = -1; dy <= 1; dy++)
            {
                if (!grid.TryGetValue((cell.X + dx, cell.Y + dy), out List<Endpoint>? endpoints))
                    continue;
                foreach (Endpoint endpoint in endpoints)
                {
                    if (used[endpoint.TraceIndex])
                        continue;
                    double distanceSquared = DistanceSquared(frontier, endpoint.Point);
                    if (distanceSquared <= pitchSquared)
                        candidates.Add(new Candidate(endpoint, atHead, distanceSquared));
                }
            }

            candidates.Sort((a, b) =>
            {
                int distance = a.DistanceSquared.CompareTo(b.DistanceSquared);
                if (distance != 0)
                    return distance;
                int length = lengths[b.Endpoint.TraceIndex].CompareTo(lengths[a.Endpoint.TraceIndex]);
                if (length != 0)
                    return length;
                int trace = a.Endpoint.TraceIndex.CompareTo(b.Endpoint.TraceIndex);
                return trace != 0 ? trace : b.Endpoint.Start.CompareTo(a.Endpoint.Start);
            });

            foreach (Candidate candidate in candidates)
            {
                cancellationToken.ThrowIfCancellationRequested();
                // The predicate receives the direction of the emitted connecting move.
                PointD a = atHead ? candidate.Endpoint.Point : frontier;
                PointD b = atHead ? frontier : candidate.Endpoint.Point;
                var key = (a.x, a.y, b.x, b.y);
                if (!proofCache.TryGetValue(key, out bool safe))
                {
                    safe = allows(a, b);
                    proofCache.Add(key, safe);
                    if (!safe)
                        refused++;
                }
                if (safe)
                    return candidate;
            }
            return null;
        }
    }

    private static Candidate? Pick(Candidate? atHead, Candidate? atTail)
    {
        if (atHead is null)
            return atTail;
        if (atTail is null)
            return atHead;
        // Prefer extending the tail on a tie, preserving the seed's natural ordering.
        return atHead.Value.DistanceSquared < atTail.Value.DistanceSquared ? atHead : atTail;
    }

    private static bool SamePoint(PointD a, PointD b) => a.x == b.x && a.y == b.y;

    private static double DistanceSquared(PointD a, PointD b)
        => (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);

    private readonly record struct Endpoint(int TraceIndex, bool Start, PointD Point);
    private readonly record struct OrientedTrace(int TraceIndex, bool Reverse);
    private readonly record struct Candidate(Endpoint Endpoint, bool AtHead, double DistanceSquared);
}
