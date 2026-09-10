namespace RW.Toolpaths.Milling;

internal readonly record struct RasterMicrotextureResult(Point3D[] Points, int RemovedPoints,
    double MaximumLift, int ProofBudgetSkips, int OriginalReversals, int ResultReversals,
    long ProofNodesVisited = 0, int CandidateCount = 0);

/// <summary>Bridges shallow valleys within a cutter-sized span, without ever lowering
/// the proven raster. The deviation limit is measured against the original polyline,
/// not the successively simplified one. It is independent of chord accuracy.</summary>
internal static class RasterMicrotextureFilter
{
    // An inconclusive bounded proof retains the vertex. This bounds worst-case work
    // on arbitrarily dense or adversarial rows without relaxing the height limit.
    private const int MaximumProofNodes = 256;
    private const double Epsilon = 1e-10;

    internal static RasterMicrotextureResult Apply(IReadOnlyList<Point3D> points,
        double maximumSpan, double maximumLift, CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        if (!double.IsFinite(maximumSpan) || maximumSpan < 0
            || !double.IsFinite(maximumLift) || maximumLift < 0)
            throw new ArgumentOutOfRangeException(nameof(maximumSpan));
        int n = points.Count;
        int before = CountReversals(points);
        if (n < 3 || maximumSpan == 0 || maximumLift == 0)
            return new(points.ToArray(), 0, 0, 0, before, before);

        // Canonical orientation makes the same row and its reversal choose the same
        // bridges. Emission below restores the caller's direction and exact endpoints.
        bool reverse = points[0].X > points[n - 1].X
            || (points[0].X == points[n - 1].X && points[0].Y > points[n - 1].Y);
        var original = new Point3D[n];
        for (int i = 0; i < n; i++) original[i] = points[reverse ? n - 1 - i : i];
        double dx = original[^1].X - original[0].X, dy = original[^1].Y - original[0].Y;
        double length = Math.Sqrt(dx * dx + dy * dy);
        if (!double.IsFinite(length) || length <= Epsilon)
            return new(points.ToArray(), 0, 0, 0, before, before);
        dx /= length; dy /= length;
        var distance = new double[n];
        var previous = new int[n]; var next = new int[n];
        for (int i = 0; i < n; i++)
        {
            if ((i & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            double x = original[i].X - original[0].X, y = original[i].Y - original[0].Y;
            distance[i] = x * dx + y * dy;
            // This helper is only for straight, strictly monotone XY raster runs.
            // Leave unexpected paths intact rather than inventing a projection.
            if (!double.IsFinite(distance[i]) || !double.IsFinite(original[i].Z)
                || Math.Abs(x * dy - y * dx) > 1e-8 * Math.Max(1, length)
                || (i > 0 && distance[i] <= distance[i - 1] + Epsilon))
                return new(points.ToArray(), 0, 0, 0, before, before);
            previous[i] = i - 1; next[i] = i + 1;
        }

        // Each node bounds its original vertices below the chord through its two
        // endpoints. Unlike an absolute minimum-Z box, this bound remains tight on
        // steep planar slopes. Bottom-up construction and storage are both O(n).
        var deficit = new double[checked(n * 4)];
        double Line(int a, int b, int at) => original[a].Z
            + (original[b].Z - original[a].Z) * ((distance[at] - distance[a]) / (distance[b] - distance[a]));
        void Build(int node, int a, int b)
        {
            if ((node & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            if (a == b) return;
            int mid = a + (b - a) / 2;
            Build(node * 2, a, mid); Build(node * 2 + 1, mid + 1, b);
            double left = Math.Max(0, Line(a, b, mid) - original[mid].Z) + deficit[node * 2];
            double right = Math.Max(Line(a, b, mid + 1) - original[mid + 1].Z, 0) + deficit[node * 2 + 1];
            deficit[node] = Math.Max(left, right);
        }
        Build(1, 0, n - 1);

        // A broad valley is a feature, even when its very bottom is represented by
        // closely spaced samples. Anchor troughs between crests farther apart than
        // the cutter span only when their depth exceeds the smoothing allowance.
        // Adaptive clearance paths can omit intermediate crests, and a tiny endpoint
        // height difference can tilt a long flat shelf after detrending. Neither
        // should turn a shallow micro-valley into a protected broad feature.
        var anchored = new bool[n];
        int trend = 0, lastSlopeEnd = 0, lastCrestEnd = 0, troughStart = -1, troughEnd = -1;
        void PreserveWideTrough(int nextCrestStart)
        {
            if (troughStart < 0 || distance[nextCrestStart] - distance[lastCrestEnd] <= maximumSpan + Epsilon) return;
            double depth = 0;
            for (int i = troughStart; i <= troughEnd; i++)
                depth = Math.Max(depth, Line(lastCrestEnd, nextCrestStart, i) - original[i].Z);
            if (depth <= maximumLift + Epsilon) return;
            for (int i = troughStart; i <= troughEnd; i++) anchored[i] = true;
        }
        double previousResidual = 0;
        for (int i = 1; i < n; i++)
        {
            if ((i & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            double residual = original[i].Z - Line(0, n - 1, i);
            double delta = residual - previousResidual; previousResidual = residual;
            if (Math.Abs(delta) <= Epsilon) continue;
            int current = delta > 0 ? 1 : -1;
            if (trend > 0 && current < 0)
            {
                PreserveWideTrough(lastSlopeEnd);
                lastCrestEnd = i - 1; troughStart = troughEnd = -1;
            }
            else if (trend < 0 && current > 0) { troughStart = lastSlopeEnd; troughEnd = i - 1; }
            trend = current; lastSlopeEnd = i;
        }
        PreserveWideTrough(n - 1);

        var queue = new Queue<int>(n);
        var queued = new bool[n]; var removed = new bool[n];
        void Enqueue(int index)
        {
            if (index <= 0 || index >= n - 1 || removed[index] || queued[index] || anchored[index]) return;
            queue.Enqueue(index); queued[index] = true;
        }
        for (int i = 1; i < n - 1; i++) Enqueue(i);
        int removedCount = 0, proofSkips = 0, candidates = 0;
        long proofNodes = 0;
        while (queue.TryDequeue(out int index))
        {
            if ((++candidates & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            queued[index] = false;
            if (removed[index]) continue;
            int a = previous[index], b = next[index];
            if (distance[b] - distance[a] > maximumSpan + Epsilon) continue;
            double lift = Line(a, b, index) - original[index].Z;
            // Retain collinear crests for the normal chord simplifier. Prematurely
            // merging them would consume the allowed span before adjacent valleys.
            if (lift <= Epsilon || lift > maximumLift) continue;
            int visited = 0; bool exhausted = false;
            bool Prove(int node, int low, int high)
            {
                if (high <= a || low >= b) return true;
                if (++visited > MaximumProofNodes) { exhausted = true; return false; }
                if (low > a && high < b)
                {
                    double bound = Math.Max(Line(a, b, low) - original[low].Z,
                        Line(a, b, high) - original[high].Z) + deficit[node];
                    if (bound <= maximumLift) return true;
                    if (low == high) return false;
                }
                int mid = low + (high - low) / 2;
                return Prove(node * 2, low, mid) && Prove(node * 2 + 1, mid + 1, high);
            }
            bool proven = Prove(1, 0, n - 1);
            proofNodes += Math.Min(visited, MaximumProofNodes);
            if (!proven)
            {
                if (exhausted) proofSkips++;
                continue;
            }
            // The new chord is above the old pair because their shared vertex lies
            // below it. Induction preserves the entire original clearance envelope.
            removed[index] = true; next[a] = b; previous[b] = a; removedCount++;
            Enqueue(a); Enqueue(b);
        }

        var output = new Point3D[n - removedCount];
        int count = 0, left = 0; double actualLift = 0;
        for (int i = 0; i < n; i++)
        {
            if ((i & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            if (!removed[i]) { output[count++] = original[i]; left = i; }
            else actualLift = Math.Max(actualLift, Line(left, next[left], i) - original[i].Z);
        }
        if (reverse) Array.Reverse(output);
        return new(output, removedCount, actualLift, proofSkips, before, CountReversals(output), proofNodes, candidates);
    }

    private static int CountReversals(IReadOnlyList<Point3D> points)
    {
        int direction = 0, count = 0;
        for (int i = 1; i < points.Count; i++)
        {
            double delta = points[i].Z - points[i - 1].Z;
            if (Math.Abs(delta) <= Epsilon || !double.IsFinite(delta)) continue;
            int current = delta > 0 ? 1 : -1;
            if (direction != 0 && direction != current) count++;
            direction = current;
        }
        return count;
    }
}
