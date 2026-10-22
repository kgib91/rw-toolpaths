using Clipper2Lib;
using System.Collections.Immutable;
using System.Diagnostics;

namespace RW.Toolpaths.Milling.Acceleration;

// A conforming SAMPLED field is still only a front nomination. Its finite
// triangles and shared roots provide no source-fill, wall, floor or stock proof.
internal static class VCarveGpuCanonicalFrontMeshNominator
{
    internal readonly record struct Leaf(long X0, long Y0, long X1, long Y1);
    internal delegate bool ScalarBatch((long X, long Y)[] sites, double[] scalar,
        CancellationToken token, out string reason);
    internal sealed record Result(bool Completed, string Reason,
        ImmutableArray<VCarveGpuDistanceFrontNominator.Fragment> Fragments,
        long AtomicBoundaryEntries, long FieldSites, long RootRecords, long DuplicateChords,
        long CropEdgeOpenEnds, long InteriorOpenEnds, long BranchVertices, double ElapsedMilliseconds)
    {
        internal bool CuttingAuthority => false;
        internal bool FilledSourceTopologyCertified => false;
        internal bool FinishedStockAcceptance => false;
    }
    private readonly record struct Vertex(long X, long Y) : IComparable<Vertex>
    {
        internal Point64 Point => new(X, Y);
        public int CompareTo(Vertex other) { int x = X.CompareTo(other.X); return x != 0 ? x : Y.CompareTo(other.Y); }
    }
    private readonly record struct RootKey(Vertex A, Vertex B, long OffsetBits);
    private readonly record struct ChordKey(Vertex A, Vertex B, int Front, long OffsetBits);
    private readonly record struct IncidenceKey(Vertex Vertex, int Front, long OffsetBits);

    internal static Result Build(VCarveGpuDistanceFrontNominator.BoundarySnapshot immutableOwner,
        IReadOnlyList<Leaf> leaves, VCarveGpuDistanceFrontNominator.Settings settings,
        ScalarBatch sampleActualField, CancellationToken token, TimeSpan maximumElapsed)
    {
        ArgumentNullException.ThrowIfNull(immutableOwner); ArgumentNullException.ThrowIfNull(sampleActualField);
        token.ThrowIfCancellationRequested(); long started = Stopwatch.GetTimestamp();
        long entries = 0, sitesCount = 0, duplicates = 0, cropEnds = 0, interiorEnds = 0, branches = 0;
        var roots = new Dictionary<RootKey, Vertex>(); var fragments = new List<VCarveGpuDistanceFrontNominator.Fragment>();
        Result Finish(bool completed, string reason)
        {
            token.ThrowIfCancellationRequested();
            if (completed && Stopwatch.GetElapsedTime(started) > maximumElapsed)
            { completed = false; reason = "canonical-mesh-time-budget"; }
            var result = new Result(completed, reason,
                completed ? fragments.ToImmutableArray() : ImmutableArray<VCarveGpuDistanceFrontNominator.Fragment>.Empty,
                entries, sitesCount, roots.Count, duplicates, cropEnds, interiorEnds, branches,
                Stopwatch.GetElapsedTime(started).TotalMilliseconds);
            token.ThrowIfCancellationRequested();
            if (completed && Stopwatch.GetElapsedTime(started) > maximumElapsed)
                return result with { Completed = false, Reason = "canonical-mesh-time-budget", Fragments = ImmutableArray<VCarveGpuDistanceFrontNominator.Fragment>.Empty,
                    ElapsedMilliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds };
            return result;
        }
        bool WithinBudget()
        { token.ThrowIfCancellationRequested(); return Stopwatch.GetElapsedTime(started) <= maximumElapsed; }
        if (leaves is null || leaves.Count > Math.Min(settings.MaximumCells, 200_000) || !settings.GloballyAnchoredTiles
            || settings.MaximumSites is < 5 or > 1_000_000 || settings.MaximumCells < 1 || settings.MaximumCells > 4L * settings.MaximumSites
            || settings.MaximumFragments is < 1 or > 1_000_000 || settings.FrontCount is < 1 or > 1000
            || !double.IsFinite(settings.FirstOffset) || settings.FirstOffset <= 0 || !double.IsFinite(settings.Pitch) || settings.Pitch <= 0
            || !double.IsFinite(settings.FirstOffset + (settings.FrontCount - 1) * settings.Pitch)
            || settings.Bounds.left <= -(1L << 29) || settings.Bounds.top <= -(1L << 29)
            || settings.Bounds.right >= 1L << 29 || settings.Bounds.bottom >= 1L << 29
            || settings.Bounds.left >= settings.Bounds.right || settings.Bounds.top >= settings.Bounds.bottom
            || maximumElapsed <= TimeSpan.Zero || maximumElapsed > TimeSpan.FromSeconds(30))
            return Finish(false, "canonical-mesh-input-or-time-budget");
        var uniqueLeaves = new HashSet<Leaf>();
        var horizontal = new Dictionary<long, SortedSet<long>>(); var vertical = new Dictionary<long, SortedSet<long>>();
        bool Add(Dictionary<long, SortedSet<long>> groups, long line, long coordinate)
        {
            if (!groups.TryGetValue(line, out var points)) groups.Add(line, points = new());
            if (points.Add(coordinate)) entries++;
            return entries <= 2L * settings.MaximumSites;
        }
        foreach (var cell in leaves)
        {
            if (!WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
            const long domain = 1L << 29;
            long width = cell.X1 - cell.X0;
            if (cell.X0 <= -domain || cell.Y0 <= -domain || cell.X1 >= domain || cell.Y1 >= domain
                || width < 2 || width > 1L << 20 || cell.Y1 - cell.Y0 != width || (width & (width - 1)) != 0
                || cell.X0 % width != 0 || cell.Y0 % width != 0 || !uniqueLeaves.Add(cell))
                return Finish(false, "canonical-leaf-domain");
            long mx = cell.X0 + (cell.X1 - cell.X0) / 2, my = cell.Y0 + (cell.Y1 - cell.Y0) / 2;
            foreach (long x in new[] { cell.X0, mx, cell.X1 })
                if (!Add(horizontal, cell.Y0, x) || !Add(horizontal, cell.Y1, x)) return Finish(false, "canonical-boundary-entry-budget");
            foreach (long y in new[] { cell.Y0, my, cell.Y1 })
                if (!Add(vertical, cell.X0, y) || !Add(vertical, cell.X1, y)) return Finish(false, "canonical-boundary-entry-budget");
        }
        foreach (var cell in leaves)
        {
            if (!WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
            for (long span = (cell.X1 - cell.X0) * 2; span <= 1L << 20; span *= 2)
            {
                long ax = cell.X0 / span * span, ay = cell.Y0 / span * span;
                if (cell.X0 < ax) ax -= span; if (cell.Y0 < ay) ay -= span;
                if (uniqueLeaves.Contains(new(ax, ay, ax + span, ay + span))) return Finish(false, "canonical-overlapping-leaves");
            }
        }
        var fieldSites = new HashSet<Vertex>();
        int fieldWork = 0;
        bool AddSite(Vertex point)
        {
            if ((++fieldWork & 255) == 0) token.ThrowIfCancellationRequested();
            fieldSites.Add(point); return fieldSites.Count <= settings.MaximumSites;
        }
        foreach (var row in horizontal) foreach (long x in row.Value)
        {
            if ((fieldWork & 255) == 0 && !WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
            if (!AddSite(new(x, row.Key))) return Finish(false, "canonical-field-site-budget");
        }
        foreach (var column in vertical) foreach (long y in column.Value)
        {
            if ((fieldWork & 255) == 0 && !WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
            if (!AddSite(new(column.Key, y))) return Finish(false, "canonical-field-site-budget");
        }
        foreach (var cell in leaves)
        {
            if ((fieldWork & 255) == 0 && !WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
            if (!AddSite(new(cell.X0 + (cell.X1 - cell.X0) / 2, cell.Y0 + (cell.Y1 - cell.Y0) / 2))) return Finish(false, "canonical-field-site-budget");
        }
        sitesCount = fieldSites.Count;
        if (sitesCount > settings.MaximumSites) return Finish(false, "canonical-field-site-budget");
        var orderedSites = fieldSites.OrderBy(p => p.X).ThenBy(p => p.Y).ToArray();
        if (!WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
        var values = new double[orderedSites.Length];
        bool sampled = sampleActualField(orderedSites.Select(p => (p.X, p.Y)).ToArray(), values, token, out string refusal);
        token.ThrowIfCancellationRequested();
        if (!sampled)
            return Finish(false, "canonical-sampler-" + refusal);
        if (!WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
        var field = new Dictionary<Vertex, double>(orderedSites.Length);
        for (int i = 0; i < orderedSites.Length; i++)
        {
            if ((i & 255) == 0 && !WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
            // Every admitted native-grid query/source pair is separated by at
            // most 2^30 * sqrt(2) ticks. This looser bound also rejects forged
            // finite values whose subtraction would overflow root arithmetic.
            if (!double.IsFinite(values[i]) || Math.Abs(values[i]) > 4L * (1L << 29) / PathUtils.Scale)
                return Finish(false, "canonical-nonfinite-or-outside-field-domain");
            field.Add(orderedSites[i], values[i] == 0 ? 0 : values[i]);
        }
        var chords = new HashSet<ChordKey>();
        Vertex Root(Vertex a, Vertex b, double offset)
        {
            // Direction, fitting cell and traversal order are absent from this
            // key. Both neighbors use the same literal atomic edge and offset.
            if (b.CompareTo(a) < 0) (a, b) = (b, a);
            if (field[a] == offset) return a; if (field[b] == offset) return b;
            var key = new RootKey(a, b, BitConverter.DoubleToInt64Bits(offset));
            if (roots.TryGetValue(key, out var previous)) return previous;
            double t = (offset - field[a]) / (field[b] - field[a]);
            var point = new Vertex((long)Math.Round(a.X + t * (b.X - a.X)), (long)Math.Round(a.Y + t * (b.Y - a.Y)));
            roots.Add(key, point); return point;
        }
        bool Emit(Vertex a, Vertex b, int level, double offset)
        {
            var pa = a.Point; var pb = b.Point;
            if (!VCarveGpuDistanceFrontNominator.ClipRequestedWindow(ref pa, ref pb, settings.Bounds) || pa == pb) return true;
            a = new(pa.X, pa.Y); b = new(pb.X, pb.Y); if (b.CompareTo(a) < 0) (a, b) = (b, a);
            if (!chords.Add(new(a, b, level, BitConverter.DoubleToInt64Bits(offset)))) { duplicates++; return true; }
            if (fragments.Count >= settings.MaximumFragments) return false;
            fragments.Add(new(a.Point, b.Point, level, offset)); return true;
        }
        foreach (var cell in leaves)
        {
            if (!WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
            var boundary = new List<Vertex>();
            foreach (long x in horizontal[cell.Y0].GetViewBetween(cell.X0, cell.X1)) boundary.Add(new(x, cell.Y0));
            foreach (long y in vertical[cell.X1].GetViewBetween(cell.Y0, cell.Y1)) if (y != cell.Y0) boundary.Add(new(cell.X1, y));
            foreach (long x in horizontal[cell.Y1].GetViewBetween(cell.X0, cell.X1).Reverse()) if (x != cell.X1) boundary.Add(new(x, cell.Y1));
            foreach (long y in vertical[cell.X0].GetViewBetween(cell.Y0, cell.Y1).Reverse()) if (y != cell.Y1 && y != cell.Y0) boundary.Add(new(cell.X0, y));
            var center = new Vertex(cell.X0 + (cell.X1 - cell.X0) / 2, cell.Y0 + (cell.Y1 - cell.Y0) / 2);
            for (int i = 0; i < boundary.Count; i++)
            {
                if (!WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
                Vertex[] triangle = { center, boundary[i], boundary[(i + 1) % boundary.Count] };
                double low = triangle.Min(p => field[p]), high = triangle.Max(p => field[p]);
                double firstValue = Math.Ceiling((low - settings.FirstOffset) / settings.Pitch);
                double lastValue = Math.Floor((high - settings.FirstOffset) / settings.Pitch);
                if (firstValue >= settings.FrontCount || lastValue < 0) continue;
                int first = (int)Math.Max(0, firstValue), last = (int)Math.Min(settings.FrontCount - 1, lastValue);
                for (int level = first; level <= last; level++)
                {
                    double offset = settings.FirstOffset + level * settings.Pitch;
                    if (triangle.All(p => field[p] == offset)) return Finish(false, "canonical-flat-triangle-ambiguity");
                    var crossings = new List<Vertex>(3); bool flat = false;
                    for (int edge = 0; edge < 3; edge++)
                    {
                        var a = triangle[edge]; var b = triangle[(edge + 1) % 3]; double va = field[a] - offset, vb = field[b] - offset;
                        if (va == 0 && vb == 0)
                        { if (!Emit(a, b, level, offset)) return Finish(false, "canonical-fragment-budget"); flat = true; }
                        else if (va == 0 || vb == 0 || va < 0 && vb > 0 || va > 0 && vb < 0)
                        {
                            var point = Root(a, b, offset); if (!crossings.Contains(point)) crossings.Add(point);
                            if (roots.Count > settings.MaximumSites) return Finish(false, "canonical-root-record-budget");
                        }
                    }
                    if (!flat && crossings.Count == 2 && !Emit(crossings[0], crossings[1], level, offset))
                        return Finish(false, "canonical-fragment-budget");
                    if (!flat && crossings.Count > 2) return Finish(false, "canonical-triangle-crossing-ambiguity");
                }
            }
        }
        var incidence = new Dictionary<IncidenceKey, int>();
        foreach (var fragment in fragments)
        {
            if (!WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
            foreach (var p in new[] { fragment.A, fragment.B })
            {
                var key = new IncidenceKey(new(p.X, p.Y), fragment.FrontIndex, BitConverter.DoubleToInt64Bits(fragment.Offset));
                incidence[key] = incidence.GetValueOrDefault(key) + 1;
            }
        }
        foreach (var item in incidence)
        {
            if (!WithinBudget()) return Finish(false, "canonical-mesh-time-budget");
            if (item.Value > 2) branches++;
            if (item.Value != 1) continue;
            var p = item.Key.Vertex;
            if (p.X == settings.Bounds.left || p.X == settings.Bounds.right || p.Y == settings.Bounds.top || p.Y == settings.Bounds.bottom) cropEnds++;
            else interiorEnds++;
        }
        return Finish(interiorEnds == 0 && branches == 0,
            interiorEnds != 0 ? "canonical-interior-open-end-refusal" : branches != 0 ? "canonical-branch-refusal" : "conforming-sampled-front-nomination");
    }
}
