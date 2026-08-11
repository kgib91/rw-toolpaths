using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>
/// Orders disconnected toolpath islands so the cutter spends as little time as possible on rapid
/// travel between them.
/// </summary>
/// <remarks>
/// Islands are compared by closest approach between sampled boundary points, which is the distance
/// the cutter actually has to cross rather than the distance between centroids. The tour is seeded
/// with nearest-neighbour and then improved with 2-opt and Or-opt passes, the standard local-search
/// pair that removes the long orphan hops a purely greedy walk leaves behind. The tour is open: the
/// cutter never has to return to where it started.
/// </remarks>
public static class TravelOptimizer
{
    private const int MaxSamplesPerIsland = 16;
    private const int MinSamplesPerIsland = 2;
    private const int SampleBudget = 4096;

    /// <summary>Above this island count the cost matrix is too large; fall back to greedy.</summary>
    private const int MaxIslandsForMatrix = 1500;

    private const int MaxOrOptSegmentLength = 3;
    private const double ImprovementEpsilon = 1e-4;

    /// <summary>
    /// Computes the visit order for a set of islands.
    /// </summary>
    /// <param name="islands">Boundary points of each island, in island index order.</param>
    /// <param name="start">Current cutter position, or <c>null</c> when entry is unconstrained.</param>
    /// <param name="label">Diagnostic label written to the performance log.</param>
    public static int[] Order(
        IReadOnlyList<IReadOnlyList<PointD>> islands,
        PointD? start = null,
        string label = "islands",
        CancellationToken cancellationToken = default)
    {
        long t0 = PerfLog.Start();

        int count = islands.Count;
        var order = new int[count];
        for (int i = 0; i < count; i++)
            order[i] = i;

        if (count <= 1)
            return order;

        int samplesPerIsland = Math.Clamp(SampleBudget / count, MinSamplesPerIsland, MaxSamplesPerIsland);
        var samples = BuildSamples(islands, samplesPerIsland);

        if (count > MaxIslandsForMatrix)
        {
            GreedyTour(order, samples, start, cancellationToken);
            PerfLog.Stop("TravelOptimizer.Order", t0,
                $"{label} islands={count} mode=greedy-only length={TourLength(order, samples, start):F2}");
            return order;
        }

        double[] cost = BuildCostMatrix(samples, cancellationToken);
        double[] startCost = BuildStartCosts(samples, start);

        NearestNeighbourTour(order, cost, startCost);
        double seeded = TourLength(order, cost, startCost);

        int rounds = ImproveTour(order, cost, startCost, cancellationToken);
        double optimized = TourLength(order, cost, startCost);

        PerfLog.Stop("TravelOptimizer.Order", t0,
            $"{label} islands={count} samples={samplesPerIsland} rounds={rounds} " +
            $"nearestNeighbour={seeded:F2} optimized={optimized:F2} saved={seeded - optimized:F2}");

        return order;
    }

    /// <summary>Evenly samples each island's boundary, packed as flat x/y pairs.</summary>
    private static double[][] BuildSamples(
        IReadOnlyList<IReadOnlyList<PointD>> islands,
        int samplesPerIsland)
    {
        var samples = new double[islands.Count][];
        for (int i = 0; i < islands.Count; i++)
        {
            var points = islands[i];
            if (points.Count == 0)
            {
                samples[i] = Array.Empty<double>();
                continue;
            }

            int take = Math.Min(samplesPerIsland, points.Count);
            var packed = new double[take * 2];
            for (int s = 0; s < take; s++)
            {
                int index = (int)((long)s * points.Count / take);
                packed[s * 2] = points[index].x;
                packed[s * 2 + 1] = points[index].y;
            }
            samples[i] = packed;
        }
        return samples;
    }

    private static double ClosestApproach(double[] a, double[] b)
    {
        if (a.Length == 0 || b.Length == 0)
            return 0;

        double best = double.MaxValue;
        for (int i = 0; i < a.Length; i += 2)
        {
            for (int j = 0; j < b.Length; j += 2)
            {
                double dx = a[i] - b[j];
                double dy = a[i + 1] - b[j + 1];
                double d = dx * dx + dy * dy;
                if (d < best)
                    best = d;
            }
        }
        return Math.Sqrt(best);
    }

    private static double ClosestApproach(double[] a, PointD point)
    {
        if (a.Length == 0)
            return 0;

        double best = double.MaxValue;
        for (int i = 0; i < a.Length; i += 2)
        {
            double dx = a[i] - point.x;
            double dy = a[i + 1] - point.y;
            double d = dx * dx + dy * dy;
            if (d < best)
                best = d;
        }
        return Math.Sqrt(best);
    }

    private static double[] BuildCostMatrix(double[][] samples, CancellationToken cancellationToken)
    {
        int count = samples.Length;
        var cost = new double[count * count];
        for (int i = 0; i < count; i++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            for (int j = i + 1; j < count; j++)
            {
                double distance = ClosestApproach(samples[i], samples[j]);
                cost[i * count + j] = distance;
                cost[j * count + i] = distance;
            }
        }
        return cost;
    }

    private static double[] BuildStartCosts(double[][] samples, PointD? start)
    {
        var startCost = new double[samples.Length];
        if (start is null)
            return startCost;

        for (int i = 0; i < samples.Length; i++)
            startCost[i] = ClosestApproach(samples[i], start.Value);
        return startCost;
    }

    private static void NearestNeighbourTour(int[] order, double[] cost, double[] startCost)
    {
        int count = order.Length;
        var visited = new bool[count];
        int current = -1;

        for (int position = 0; position < count; position++)
        {
            int best = -1;
            double bestCost = double.MaxValue;
            for (int candidate = 0; candidate < count; candidate++)
            {
                if (visited[candidate])
                    continue;
                double candidateCost = current < 0
                    ? startCost[candidate]
                    : cost[current * count + candidate];
                if (candidateCost < bestCost)
                {
                    bestCost = candidateCost;
                    best = candidate;
                }
            }

            visited[best] = true;
            order[position] = best;
            current = best;
        }
    }

    private static void GreedyTour(
        int[] order,
        double[][] samples,
        PointD? start,
        CancellationToken cancellationToken)
    {
        int count = order.Length;
        var visited = new bool[count];
        int current = -1;

        for (int position = 0; position < count; position++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            int best = -1;
            double bestCost = double.MaxValue;
            for (int candidate = 0; candidate < count; candidate++)
            {
                if (visited[candidate])
                    continue;
                double candidateCost = current < 0
                    ? (start is null ? 0 : ClosestApproach(samples[candidate], start.Value))
                    : ClosestApproach(samples[current], samples[candidate]);
                if (candidateCost < bestCost)
                {
                    bestCost = candidateCost;
                    best = candidate;
                }
            }

            visited[best] = true;
            order[position] = best;
            current = best;
        }
    }

    private static int ImproveTour(
        int[] order,
        double[] cost,
        double[] startCost,
        CancellationToken cancellationToken)
    {
        int count = order.Length;
        int maxRounds = count <= 200 ? 60 : count <= 800 ? 20 : 6;

        int round = 0;
        while (round < maxRounds)
        {
            cancellationToken.ThrowIfCancellationRequested();
            bool improved = TwoOptPass(order, cost, startCost);
            improved |= OrOptPass(order, cost, startCost);
            round++;
            if (!improved)
                break;
        }
        return round;
    }

    /// <summary>Reverses tour sections to remove crossing edges.</summary>
    private static bool TwoOptPass(int[] order, double[] cost, double[] startCost)
    {
        int count = order.Length;
        bool improved = false;

        for (int i = 0; i < count - 1; i++)
        {
            for (int j = i + 1; j < count; j++)
            {
                int before = i == 0 ? -1 : order[i - 1];
                int head = order[i];
                int tail = order[j];
                int after = j + 1 < count ? order[j + 1] : -1;

                double currentCost =
                    (before < 0 ? startCost[head] : cost[before * count + head])
                    + (after < 0 ? 0 : cost[tail * count + after]);
                double reversedCost =
                    (before < 0 ? startCost[tail] : cost[before * count + tail])
                    + (after < 0 ? 0 : cost[head * count + after]);

                if (reversedCost < currentCost - ImprovementEpsilon)
                {
                    Array.Reverse(order, i, j - i + 1);
                    improved = true;
                }
            }
        }
        return improved;
    }

    /// <summary>Relocates short runs of islands to cheaper positions in the tour.</summary>
    private static bool OrOptPass(int[] order, double[] cost, double[] startCost)
    {
        int count = order.Length;
        bool improved = false;
        var segment = new int[MaxOrOptSegmentLength];
        var reduced = new int[count];

        for (int length = 1; length <= MaxOrOptSegmentLength && length < count; length++)
        {
            for (int i = 0; i + length <= count; i++)
            {
                Array.Copy(order, i, segment, 0, length);

                int reducedCount = 0;
                for (int k = 0; k < count; k++)
                {
                    if (k < i || k >= i + length)
                        reduced[reducedCount++] = order[k];
                }

                int bestPosition = i;
                double bestCost = InsertionCost(reduced, reducedCount, i, segment, length, cost, startCost, count);

                for (int position = 0; position <= reducedCount; position++)
                {
                    if (position == i)
                        continue;
                    double candidateCost = InsertionCost(
                        reduced, reducedCount, position, segment, length, cost, startCost, count);
                    if (candidateCost < bestCost - ImprovementEpsilon)
                    {
                        bestCost = candidateCost;
                        bestPosition = position;
                    }
                }

                if (bestPosition == i)
                    continue;

                Array.Copy(reduced, 0, order, 0, bestPosition);
                Array.Copy(segment, 0, order, bestPosition, length);
                Array.Copy(reduced, bestPosition, order, bestPosition + length, reducedCount - bestPosition);
                improved = true;
            }
        }
        return improved;
    }

    private static double InsertionCost(
        int[] reduced,
        int reducedCount,
        int position,
        int[] segment,
        int segmentLength,
        double[] cost,
        double[] startCost,
        int count)
    {
        int before = position == 0 ? -1 : reduced[position - 1];
        int after = position < reducedCount ? reduced[position] : -1;
        int head = segment[0];
        int tail = segment[segmentLength - 1];

        double withSegment =
            (before < 0 ? startCost[head] : cost[before * count + head])
            + (after < 0 ? 0 : cost[tail * count + after]);
        double withoutSegment = after < 0
            ? 0
            : (before < 0 ? startCost[after] : cost[before * count + after]);

        return withSegment - withoutSegment;
    }

    private static double TourLength(int[] order, double[] cost, double[] startCost)
    {
        int count = order.Length;
        if (count == 0)
            return 0;

        double total = startCost[order[0]];
        for (int i = 1; i < count; i++)
            total += cost[order[i - 1] * count + order[i]];
        return total;
    }

    private static double TourLength(int[] order, double[][] samples, PointD? start)
    {
        if (order.Length == 0)
            return 0;

        double total = start is null ? 0 : ClosestApproach(samples[order[0]], start.Value);
        for (int i = 1; i < order.Length; i++)
            total += ClosestApproach(samples[order[i - 1]], samples[order[i]]);
        return total;
    }
}
