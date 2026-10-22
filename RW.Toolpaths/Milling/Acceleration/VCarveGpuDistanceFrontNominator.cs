using Clipper2Lib;
using System.Collections.Immutable;
using System.Diagnostics;

namespace RW.Toolpaths.Milling.Acceleration;

/// <summary>Experimental candidate construction only. Sampled distance/winding
/// may omit a nominee but never establish empty stock, wall clearance, floor
/// clearance, grade, finished-stock quality, or final target coverage.</summary>
internal static class VCarveGpuDistanceFrontNominator
{
    internal sealed class BoundarySnapshot
    {
        private readonly GpuBoundarySegment[] _segments;
        private readonly GpuBoundaryNode[] _nodes;
        private readonly int[] _originalOrdinals;
        internal ImmutableArray<ImmutableArray<PointD>> OriginalBoundary { get; }
        internal double MaximumGridConversionMove { get; }
        internal double PreparationMilliseconds { get; }
        internal int SegmentCount => _segments.Length;
        internal int NodeCount => _nodes.Length;

        private BoundarySnapshot(GpuBoundarySegment[] segments, GpuBoundaryNode[] nodes, int[] originalOrdinals,
            ImmutableArray<ImmutableArray<PointD>> original, double move, double milliseconds)
        { _segments = segments; _nodes = nodes; _originalOrdinals = originalOrdinals; OriginalBoundary = original;
          MaximumGridConversionMove = move; PreparationMilliseconds = milliseconds; }

        internal GpuBoundarySegment Segment(int ordinal) => _segments[ordinal];
        internal int OriginalOrdinal(int ordinal) => _originalOrdinals[ordinal];
        internal bool TryCreateSampler(CancellationToken token, Action<string>? diagnostic,
            out VCarveGpuDiscWitnessContext.BoundarySampleContext? context, out string reason)
            => VCarveGpuDiscWitnessContext.BoundarySampleContext.TryCreate(_segments, _nodes, token, diagnostic, out context, out reason);

        internal static bool TryPrepare(IReadOnlyList<IReadOnlyList<PointD>> boundary, CancellationToken token,
            out BoundarySnapshot? snapshot, out string reason)
        {
            snapshot = null; reason = ""; token.ThrowIfCancellationRequested(); long started = Stopwatch.GetTimestamp();
            if (boundary is null || boundary.Count is < 1 or > 16_384) { reason = "front-source-row-budget"; return false; }
            long count = boundary.Sum(ring => (long)ring.Count);
            if (count is < 1 or > VCarveGpuDiscWitnessContext.BoundarySampleContext.MaximumSegments)
            { reason = "front-source-point-budget"; return false; }
            var originals = ImmutableArray.CreateBuilder<ImmutableArray<PointD>>(boundary.Count);
            var items = new List<OrderedEdge>((int)count); double maximumMove = 0; int originalOrdinal = 0;
            foreach (var ring in boundary)
            {
                token.ThrowIfCancellationRequested();
                var copied = ImmutableArray.CreateBuilder<PointD>(ring.Count); var grid = new Point64[ring.Count];
                for (int i = 0; i < ring.Count; i++)
                {
                    if ((i & 255) == 0) token.ThrowIfCancellationRequested(); var p = ring[i];
                    if (!double.IsFinite(p.x) || !double.IsFinite(p.y) || Math.Abs(p.x * PathUtils.Scale) >= (1L << 29) - 1
                        || Math.Abs(p.y * PathUtils.Scale) >= (1L << 29) - 1)
                    { reason = "front-source-domain"; return false; }
                    grid[i] = new((long)Math.Round(p.x * PathUtils.Scale), (long)Math.Round(p.y * PathUtils.Scale));
                    double dx = grid[i].X / PathUtils.Scale - p.x, dy = grid[i].Y / PathUtils.Scale - p.y;
                    maximumMove = Math.Max(maximumMove, Math.Sqrt(dx * dx + dy * dy)); copied.Add(p);
                }
                originals.Add(copied.MoveToImmutable());
                for (int i = 0; i < grid.Length; i++)
                {
                    var edge = new GpuBoundarySegment(grid[i], grid[(i + 1) % grid.Length]);
                    ulong key = Morton((edge.Ax + edge.Bx) / 2, (edge.Ay + edge.By) / 2);
                    items.Add(new(key, originalOrdinal++, edge));
                }
            }
            token.ThrowIfCancellationRequested();
            // One stable spatial ordering followed by a linear balanced build;
            // original finite-edge provenance survives in a separate map.
            items.Sort((a, b) => { int order = a.Key.CompareTo(b.Key); return order != 0 ? order : a.Ordinal.CompareTo(b.Ordinal); });
            token.ThrowIfCancellationRequested();
            var segments = items.Select(item => item.Edge).ToArray(); var ordinals = items.Select(item => item.Ordinal).ToArray();
            var nodes = new List<GpuBoundaryNode>(Math.Max(1, segments.Length / 4));
            int Add(int first, int length)
            {
                token.ThrowIfCancellationRequested(); int ordinal = nodes.Count; nodes.Add(default);
                if (length <= 8)
                {
                    long x0 = long.MaxValue, y0 = long.MaxValue, x1 = long.MinValue, y1 = long.MinValue;
                    for (int i = first; i < first + length; i++)
                    { var s = segments[i]; x0 = Math.Min(x0, Math.Min(s.Ax, s.Bx)); y0 = Math.Min(y0, Math.Min(s.Ay, s.By));
                      x1 = Math.Max(x1, Math.Max(s.Ax, s.Bx)); y1 = Math.Max(y1, Math.Max(s.Ay, s.By)); }
                    nodes[ordinal] = new(x0, y0, x1, y1, first, length, -1, -1);
                }
                else
                {
                    int half = length / 2, left = Add(first, half), right = Add(first + half, length - half);
                    var a = nodes[left]; var b = nodes[right];
                    nodes[ordinal] = new(Math.Min(a.X0, b.X0), Math.Min(a.Y0, b.Y0), Math.Max(a.X1, b.X1), Math.Max(a.Y1, b.Y1), 0, 0, left, right);
                }
                return ordinal;
            }
            Add(0, segments.Length); var nodeArray = nodes.ToArray();
            if (!VCarveGpuDiscWitnessContext.BoundarySampleContext.ValidateSnapshot(segments, nodeArray,
                GpuBoundaryWorkLimits.Default, token, out reason)) return false;
            snapshot = new(segments, nodeArray, ordinals, originals.MoveToImmutable(), maximumMove,
                Stopwatch.GetElapsedTime(started).TotalMilliseconds); return true;
        }

        private readonly record struct OrderedEdge(ulong Key, int Ordinal, GpuBoundarySegment Edge);
        private static ulong Morton(long x, long y)
        {
            static ulong Spread(ulong value)
            {
                value &= 0x1fffff; value = (value | value << 16) & 0x0000ffff0000ffff;
                value = (value | value << 8) & 0x00ff00ff00ff00ff;
                value = (value | value << 4) & 0x0f0f0f0f0f0f0f0f;
                value = (value | value << 2) & 0x3333333333333333;
                return (value | value << 1) & 0x5555555555555555;
            }
            return Spread((ulong)(x + (1L << 29)) >> 9) | Spread((ulong)(y + (1L << 29)) >> 9) << 1;
        }
    }

    internal readonly record struct Settings(Rect64 Bounds, double FirstOffset, double Pitch, int FrontCount,
        double MaximumLeafSpan, int MaximumSites, int MaximumCells, int MaximumFragments, TimeSpan MaximumElapsed,
        bool AnalyticStraightCells = true, bool LocalDistanceFit = true, bool GloballyAnchoredTiles = true,
        bool CanonicalActualRoots = false);
    internal readonly record struct CorrectionSetup(Settings NominationSettings, double CutDepth, double CommandRadius,
        int RegionIndex, int DepthPassIndex);

    internal static bool TryCorrectionSetup(VCarveSmoothing.CorrectionConstructorInputs input, Rect64 bounds,
        int frontCount, double nominationLeafSpan, int maximumSites, TimeSpan maximumElapsed,
        out CorrectionSetup setup, out string reason)
    {
        setup = default; reason = "";
        if (maximumSites is < 5 or > 1_000_000 || input.PointBudget < 2)
        { reason = "unsupported-authored-output-or-probe-site-budget"; return false; }
        double depth = VCarveSmoothing.QuantizeCorrectionDepth(input.InitialCutDepth, input.Schedule.SurfaceZ);
        if (!(depth > 0) || !double.IsFinite(input.StepOver) || input.StepOver <= 0)
        { reason = "unengaged-or-invalid-authored-stepover"; return false; }
        double radius = MedialAxisToolpaths.CommandedCuttingRadius(input.Profile, 0, 0, 0, depth, input.Schedule.SurfaceZ);
        double diameter = 2 * input.Profile.RadiusAtDepth(depth);
        double pitch = input.StepOver * (input.Profile.RoundedTip ? Math.Min(input.Profile.ClearingDiameter, diameter) : diameter);
        pitch = Math.Clamp(pitch, .0001, Math.Max(.0001, diameter * .8));
        if (!(radius > 1e-7) || !double.IsFinite(radius)) { reason = "unengaged-command-radius"; return false; }
        int pass = Enumerable.Range(0, input.Schedule.PassCount)
            .FirstOrDefault(index => input.Schedule.SurfaceZ - input.Schedule.PassBottomZ(index) >= depth);
        int maximumFragments = (int)Math.Min(1_000_000, Math.Max(1, input.PointBudget / 2));
        var settings = new Settings(bounds, radius + 2 / PathUtils.Scale, pitch, frontCount, nominationLeafSpan,
            maximumSites, checked(4 * maximumSites), maximumFragments, maximumElapsed);
        setup = new(settings, depth, radius, input.RegionIndex, input.Schedule.FirstPassIndex + pass); return true;
    }
    internal readonly record struct Fragment(Point64 A, Point64 B, int FrontIndex, double Offset);
    internal sealed record Result(bool CompletedNomination, string Reason, ImmutableArray<Fragment> Fragments,
        long SampleSites, long CellTasks, long SplitCells, long AnalyticStraightCells, long MarchingCells,
        double ElapsedMilliseconds, double MaximumGridConversionMove)
    {
        internal bool CuttingAuthority => false;
        internal bool FixedTargetCertified => false;
        internal bool FinishedStockAudited => false;
        internal long LocalDistanceFitCells { get; init; }
        internal long CachedGeometryPatches { get; init; }
        internal long CanonicalRootRecords { get; init; }
        internal long CanonicalCropEdgeOpenEnds { get; init; }
        internal long CanonicalInteriorOpenEnds { get; init; }
        internal long CanonicalBranchVertices { get; init; }
        internal long CanonicalFieldSites { get; init; }
        internal bool CanonicalFieldIncidenceMatched { get; init; }
        internal bool ActualCudaExecuted { get; init; }
        internal double BoundaryPreparationMilliseconds { get; init; }
        internal double SamplerPreparationMilliseconds { get; init; }
        internal double SamplerKernelMilliseconds { get; init; }
        internal double SamplerTransferMilliseconds { get; init; }
        internal double ColdSamplerAndNominationMilliseconds { get; init; }
    }
    internal delegate bool SampleBatch(GpuBoundaryQuery[] sites, GpuBoundarySampleNomination[] nominations,
        int count, CancellationToken token, out string reason);
    private readonly record struct Cell(long X0, long Y0, long X1, long Y1);
    private readonly record struct Sample(double SignedDistance, GpuBoundarySampleNomination Nomination);

    internal static Result Run(BoundarySnapshot snapshot, Settings settings, CancellationToken token,
        Action<string>? diagnostic = null)
    {
        token.ThrowIfCancellationRequested(); long coldStarted = Stopwatch.GetTimestamp();
        if (!snapshot.TryCreateSampler(token, diagnostic, out var context, out string reason))
            return new(false, reason, ImmutableArray<Fragment>.Empty, 0, 0, 0, 0, 0, 0, snapshot.MaximumGridConversionMove);
        using (context)
        {
            var result = Run(snapshot, settings, context!.TryNominate, token, diagnostic);
            return result with { ActualCudaExecuted = context.QueryCount > 0,
                BoundaryPreparationMilliseconds = snapshot.PreparationMilliseconds,
                SamplerPreparationMilliseconds = context.PreparationMilliseconds,
                SamplerKernelMilliseconds = context.KernelMilliseconds,
                SamplerTransferMilliseconds = context.TransferMilliseconds,
                ColdSamplerAndNominationMilliseconds = Stopwatch.GetElapsedTime(coldStarted).TotalMilliseconds };
        }
    }

    // The delegate overload allows an independent exhaustive CPU nomination
    // oracle in artifact controls. It never enters a production cutting path.
    internal static Result Run(BoundarySnapshot snapshot, Settings settings, SampleBatch sampleBatch,
        CancellationToken token, Action<string>? diagnostic = null, VCarveGpuFrontGeometryCache? geometryCache = null)
    {
        token.ThrowIfCancellationRequested(); long started = Stopwatch.GetTimestamp();
        long cells = 0, splits = 0, straight = 0, marching = 0, fitted = 0, reusedPatches = 0;
        var samples = new Dictionary<(long X, long Y), Sample>(); var fragments = new List<Fragment>();
        var leaves = new List<VCarveGpuCanonicalFrontMeshNominator.Leaf>(); VCarveGpuCanonicalFrontMeshNominator.Result? mesh = null;
        Result Finish(bool completed, string reason)
        {
            token.ThrowIfCancellationRequested();
            var result = new Result(completed, reason, completed ? fragments.ToImmutableArray() : ImmutableArray<Fragment>.Empty,
                samples.Count, cells, splits, straight, marching, Stopwatch.GetElapsedTime(started).TotalMilliseconds,
                snapshot.MaximumGridConversionMove) { LocalDistanceFitCells = fitted, CachedGeometryPatches = reusedPatches,
                    CanonicalRootRecords = mesh?.RootRecords ?? 0, CanonicalCropEdgeOpenEnds = mesh?.CropEdgeOpenEnds ?? 0,
                    CanonicalInteriorOpenEnds = mesh?.InteriorOpenEnds ?? 0, CanonicalBranchVertices = mesh?.BranchVertices ?? 0,
                    CanonicalFieldSites = mesh?.FieldSites ?? 0, CanonicalFieldIncidenceMatched = mesh?.Completed ?? false };
            diagnostic?.Invoke($"stage=gpu-distance-front-probe event=complete nominationCompleted={completed} reason='{reason}' "
                + $"sites={samples.Count} cells={cells} splits={splits} analyticStraightCells={straight} marchingCells={marching} localDistanceFitCells={fitted} cachedGeometryPatches={reusedPatches} "
                + $"fragments={result.Fragments.Length} maximumSites={settings.MaximumSites} maximumCells={settings.MaximumCells} "
                + $"maximumFragments={settings.MaximumFragments} frontCount={settings.FrontCount} firstOffset={settings.FirstOffset:R} "
                + $"pitch={settings.Pitch:R} maximumLeafSpan={settings.MaximumLeafSpan:R} elapsedMs={result.ElapsedMilliseconds:F3} "
                + $"canonicalActualRoots={settings.CanonicalActualRoots} canonicalRootRecords={result.CanonicalRootRecords} canonicalFieldSites={result.CanonicalFieldSites} "
                + $"canonicalCropEdgeOpenEnds={result.CanonicalCropEdgeOpenEnds} canonicalInteriorOpenEnds={result.CanonicalInteriorOpenEnds} canonicalBranches={result.CanonicalBranchVertices} "
                + "cuttingAuthority=False fixedTargetCertified=False finishedStockAudited=False negativeWindingAuthority=False");
            token.ThrowIfCancellationRequested(); return result;
        }
        var bounds = settings.Bounds; const long domain = 1L << 29;
        if (bounds.left <= -domain || bounds.top <= -domain || bounds.right >= domain || bounds.bottom >= domain
            || bounds.left >= bounds.right || bounds.top >= bounds.bottom || !double.IsFinite(settings.FirstOffset)
            || settings.FirstOffset <= 0 || !double.IsFinite(settings.Pitch) || settings.Pitch <= 0
            || settings.FrontCount is < 1 or > 1000 || !double.IsFinite(settings.MaximumLeafSpan)
            || !double.IsFinite(settings.FirstOffset + (settings.FrontCount - 1) * settings.Pitch)
            || settings.FirstOffset + (settings.FrontCount - 1) * settings.Pitch > 4 * domain / PathUtils.Scale
            || settings.MaximumLeafSpan < 2 / PathUtils.Scale || settings.MaximumLeafSpan > .25
            || settings.MaximumSites is < 5 or > 1_000_000 || settings.MaximumCells < 1
            || settings.MaximumCells > 4L * settings.MaximumSites || settings.MaximumFragments is < 1 or > 1_000_000
            || settings.MaximumElapsed <= TimeSpan.Zero || settings.MaximumElapsed > TimeSpan.FromSeconds(30))
            return Finish(false, "unsupported-probe-settings");
        if (geometryCache is not null)
        {
            if (!ReferenceEquals(geometryCache.Owner, snapshot) || !geometryCache.Available)
                return Finish(false, "immutable-geometry-owner-mismatch-or-unavailable");
            sampleBatch = geometryCache.TryNominate;
        }
        var pending = new Queue<Cell>();
        if (settings.GloballyAnchoredTiles)
        {
            const long span = 1L << 20; // 32 mm, power-of-two native-grid lattice.
            static long FloorTile(long value) { long q = Math.DivRem(value, span, out long remainder); return remainder < 0 ? q - 1 : q; }
            long tileX = FloorTile(bounds.left), tileY = FloorTile(bounds.top);
            for (long y = tileY * span; y < bounds.bottom; y += span)
            for (long x = tileX * span; x < bounds.right; x += span)
            {
                if (x <= -domain || y <= -domain || x + span >= domain || y + span >= domain)
                    return Finish(false, "anchored-tile-domain");
                if (pending.Count >= settings.MaximumCells) return Finish(false, "nomination-tile-budget");
                pending.Enqueue(new(x, y, x + span, y + span));
            }
        }
        else pending.Enqueue(new(bounds.left, bounds.top, bounds.right, bounds.bottom));
        long leafSpan = Math.Max(2, (long)Math.Floor(Math.Min(settings.MaximumLeafSpan, settings.Pitch / 8) * PathUtils.Scale));
        while (pending.Count != 0)
        {
            token.ThrowIfCancellationRequested();
            if (Stopwatch.GetElapsedTime(started) > settings.MaximumElapsed) return Finish(false, "nomination-time-budget");
            var batch = new List<(Cell Cell, VCarveGpuFrontGeometryCache.Patch? Cached)>(1024); var needed = new HashSet<(long X, long Y)>();
            for (int i = 0; i < 1024 && pending.Count > 0; i++)
            {
                if (++cells > settings.MaximumCells) return Finish(false, "nomination-cell-budget");
                var cell = pending.Dequeue();
                if (cell.X1 < bounds.left || cell.X0 > bounds.right || cell.Y1 < bounds.top || cell.Y0 > bounds.bottom) continue;
                VCarveGpuFrontGeometryCache.Patch? patch = null;
                if (geometryCache is not null && geometryCache.TryPatch(Key(cell, settings), out var found))
                { patch = found; reusedPatches++; }
                batch.Add((cell, patch));
                if (patch is null) foreach (var site in Sites(cell)) if (!samples.ContainsKey(site)) needed.Add(site);
            }
            if (samples.Count + needed.Count > settings.MaximumSites) return Finish(false, "nomination-site-budget");
            if (needed.Count != 0)
            {
                var keys = needed.OrderBy(site => site.X).ThenBy(site => site.Y).ToArray();
                var queries = keys.Select(site => new GpuBoundaryQuery(site.X, site.Y)).ToArray();
                var nominations = new GpuBoundarySampleNomination[queries.Length];
                if (!sampleBatch(queries, nominations, queries.Length, token, out string refusal)) return Finish(false, "sampler-" + refusal);
                token.ThrowIfCancellationRequested();
                for (int i = 0; i < keys.Length; i++)
                {
                    var n = nominations[i];
                    if (!n.HasNomination || n.SegmentOrdinal < 0 || n.SegmentOrdinal >= snapshot.SegmentCount
                        || !double.IsFinite(n.SquaredDistance) || n.SquaredDistance < 0 || !double.IsFinite(n.Parameter)
                        || n.Parameter < 0 || n.Parameter > 1) return Finish(false, "sampler-refused-or-invalid-nomination");
                    double distance = Math.Sqrt(n.SquaredDistance) / PathUtils.Scale;
                    samples.Add(keys[i], new(n.Winding != 0 ? distance : -distance, n));
                }
            }
            foreach (var entry in batch)
            {
                token.ThrowIfCancellationRequested(); var cell = entry.Cell; var sites = Sites(cell);
                var patch = entry.Cached ?? BuildPatch(snapshot, cell, sites, sites.Select(site => samples[site]).ToArray(), settings);
                if (entry.Cached is null) geometryCache?.StorePatch(Key(cell, settings), patch);
                double dx = (cell.X1 - cell.X0) / PathUtils.Scale, dy = (cell.Y1 - cell.Y0) / PathUtils.Scale;
                double diagonal = Math.Sqrt(dx * dx + dy * dy);
                // Sample bounds control nominations only. Unsampled geometry,
                // finite-edge switches, thin holes and negative winding are
                // never an empty-stock or coverage certificate.
                double low = patch.MinimumSigned - diagonal, high = patch.MaximumSigned + diagonal;
                if (!LevelRange(low, high, settings, out int first, out int last))
                {
                    if (settings.CanonicalActualRoots) leaves.Add(new(cell.X0, cell.Y0, cell.X1, cell.Y1));
                    continue;
                }
                if (patch.NominationMode != 0)
                {
                    if (patch.NominationMode == 1) straight++; else fitted++;
                    if (settings.CanonicalActualRoots)
                    { leaves.Add(new(cell.X0, cell.Y0, cell.X1, cell.Y1)); continue; }
                    var scalar = sites.Take(4).Select(site => patch.Origin
                        + patch.Gx * ((site.X - cell.X0) / PathUtils.Scale - dx / 2)
                        + patch.Gy * ((site.Y - cell.Y0) / PathUtils.Scale - dy / 2)).ToArray();
                    if (LevelRange(scalar.Min(), scalar.Max(), settings, out first, out last))
                        for (int level = first; level <= last; level++)
                            if (!Emit(cell, sites, scalar, level, settings.FirstOffset + level * settings.Pitch, fragments, settings.MaximumFragments, bounds))
                                return Finish(false, "nomination-fragment-budget");
                    continue;
                }
                if (cell.X1 - cell.X0 > leafSpan || cell.Y1 - cell.Y0 > leafSpan)
                {
                    bool splitX = cell.X1 - cell.X0 > leafSpan, splitY = cell.Y1 - cell.Y0 > leafSpan;
                    int children = splitX && splitY ? 4 : 2;
                    if (pending.Count + children > settings.MaximumCells - cells) return Finish(false, "nomination-pending-cell-budget");
                    long midX = sites[4].X, midY = sites[4].Y;
                    if (splitX && (midX <= cell.X0 || midX >= cell.X1) || splitY && (midY <= cell.Y0 || midY >= cell.Y1))
                        return Finish(false, "nomination-native-cell-resolution");
                    splits++;
                    if (splitX && splitY)
                    {
                        pending.Enqueue(new(cell.X0, cell.Y0, midX, midY)); pending.Enqueue(new(midX, cell.Y0, cell.X1, midY));
                        pending.Enqueue(new(midX, midY, cell.X1, cell.Y1)); pending.Enqueue(new(cell.X0, midY, midX, cell.Y1));
                    }
                    else if (splitX)
                    { pending.Enqueue(new(cell.X0, cell.Y0, midX, cell.Y1)); pending.Enqueue(new(midX, cell.Y0, cell.X1, cell.Y1)); }
                    else
                    { pending.Enqueue(new(cell.X0, cell.Y0, cell.X1, midY)); pending.Enqueue(new(cell.X0, midY, cell.X1, cell.Y1)); }
                }
                else
                {
                    if (settings.CanonicalActualRoots)
                    { leaves.Add(new(cell.X0, cell.Y0, cell.X1, cell.Y1)); marching++; continue; }
                    marching++; var scalar = new[] { patch.V0, patch.V1, patch.V2, patch.V3 };
                    if (LevelRange(scalar.Min(), scalar.Max(), settings, out first, out last))
                        for (int level = first; level <= last; level++)
                            if (!Emit(cell, sites, scalar, level, settings.FirstOffset + level * settings.Pitch, fragments, settings.MaximumFragments, bounds))
                                return Finish(false, "nomination-fragment-budget");
                }
            }
        }
        if (settings.CanonicalActualRoots)
        {
            bool ActualScalars((long X, long Y)[] sites, double[] scalar, CancellationToken ct, out string reason)
            {
                reason = ""; ct.ThrowIfCancellationRequested();
                var missing = sites.Where(site => !samples.ContainsKey(site)).Distinct().ToArray();
                if (samples.Count + missing.Length > settings.MaximumSites) { reason = "site-budget"; return false; }
                if (missing.Length != 0)
                {
                    var queries = missing.Select(site => new GpuBoundaryQuery(site.X, site.Y)).ToArray();
                    var nominations = new GpuBoundarySampleNomination[queries.Length];
                    if (!sampleBatch(queries, nominations, queries.Length, ct, out reason)) return false;
                    ct.ThrowIfCancellationRequested();
                    for (int i = 0; i < missing.Length; i++)
                    {
                        if ((i & 255) == 0) ct.ThrowIfCancellationRequested(); var n = nominations[i];
                        if (!n.HasNomination || (uint)n.SegmentOrdinal >= (uint)snapshot.SegmentCount
                            || !double.IsFinite(n.Parameter) || n.Parameter < 0 || n.Parameter > 1
                            || !double.IsFinite(n.SquaredDistance) || n.SquaredDistance < 0
                            || n.Winding < -snapshot.SegmentCount || n.Winding > snapshot.SegmentCount)
                        { reason = "refused-or-invalid-nomination"; return false; }
                        double distance = Math.Sqrt(n.SquaredDistance) / PathUtils.Scale;
                        samples.Add(missing[i], new(n.Winding != 0 ? distance : -distance, n));
                    }
                }
                for (int i = 0; i < sites.Length; i++) scalar[i] = samples[sites[i]].SignedDistance;
                return true;
            }
            mesh = VCarveGpuCanonicalFrontMeshNominator.Build(snapshot, leaves, settings, ActualScalars, token,
                settings.MaximumElapsed - Stopwatch.GetElapsedTime(started));
            if (!mesh.Completed) return Finish(false, mesh.Reason);
            fragments.AddRange(mesh.Fragments);
        }
        return Finish(true, "completed-requested-nomination-window");
    }

    private static VCarveGpuFrontGeometryCache.PatchKey Key(Cell cell, Settings settings)
        => new(cell.X0, cell.Y0, cell.X1, cell.Y1, BitConverter.DoubleToInt64Bits(settings.MaximumLeafSpan),
            (settings.AnalyticStraightCells ? 1 : 0) | (settings.LocalDistanceFit ? 2 : 0));

    // Radius, front count, pitch and component bounds are absent from this
    // patch: it is a sampled immutable-boundary field, with no stock authority.
    private static VCarveGpuFrontGeometryCache.Patch BuildPatch(BoundarySnapshot snapshot, Cell cell,
        (long X, long Y)[] sites, Sample[] samples, Settings settings)
    {
        double minimum = samples.Min(sample => sample.SignedDistance), maximum = samples.Max(sample => sample.SignedDistance);
        double origin = 0, gx = 0, gy = 0, error = 0; int mode = 0;
        double middleX = cell.X0 + (cell.X1 - cell.X0) / 2.0, middleY = cell.Y0 + (cell.Y1 - cell.Y0) / 2.0;
        int ordinal = samples[0].Nomination.SegmentOrdinal;
        if (settings.AnalyticStraightCells && samples.All(sample => sample.Nomination.Winding != 0
            && sample.Nomination.SegmentOrdinal == ordinal && sample.Nomination.Parameter > 0 && sample.Nomination.Parameter < 1))
        {
            var edge = snapshot.Segment(ordinal); double ex = edge.Bx - edge.Ax, ey = edge.By - edge.Ay;
            double length = Math.Sqrt(ex * ex + ey * ey); int side = 0; bool consistent = length > 0;
            foreach (var site in sites)
            {
                int sign = Math.Sign(ex * (site.Y - edge.Ay) - ey * (site.X - edge.Ax));
                if (sign == 0 || side != 0 && sign != side) { consistent = false; break; }
                side = sign;
            }
            if (consistent)
            {
                origin = side * (ex * (middleY - edge.Ay) - ey * (middleX - edge.Ax)) / length / PathUtils.Scale;
                gx = -side * ey / length; gy = side * ex / length; mode = 1;
            }
        }
        if (mode == 0 && settings.LocalDistanceFit && samples.All(sample => sample.Nomination.Winding != 0))
        {
            double dx = (cell.X1 - cell.X0) / PathUtils.Scale, dy = (cell.Y1 - cell.Y0) / PathUtils.Scale;
            origin = (samples[0].SignedDistance + samples[1].SignedDistance + samples[2].SignedDistance + samples[3].SignedDistance) / 4;
            gx = ((samples[1].SignedDistance - samples[0].SignedDistance) + (samples[2].SignedDistance - samples[3].SignedDistance)) / (2 * dx);
            gy = ((samples[3].SignedDistance - samples[0].SignedDistance) + (samples[2].SignedDistance - samples[1].SignedDistance)) / (2 * dy);
            for (int i = 0; i < sites.Length; i++)
            {
                double predicted = origin + gx * ((sites[i].X - middleX) / PathUtils.Scale) + gy * ((sites[i].Y - middleY) / PathUtils.Scale);
                error = Math.Max(error, Math.Abs(predicted - samples[i].SignedDistance));
            }
            if (error <= settings.MaximumLeafSpan) mode = 2;
        }
        return new(minimum, maximum, samples[0].SignedDistance, samples[1].SignedDistance,
            samples[2].SignedDistance, samples[3].SignedDistance, origin, gx, gy, mode, error);
    }

    private static (long X, long Y)[] Sites(Cell c)
    {
        long x = c.X0 + (c.X1 - c.X0) / 2, y = c.Y0 + (c.Y1 - c.Y0) / 2;
        return new[] { (c.X0, c.Y0), (c.X1, c.Y0), (c.X1, c.Y1), (c.X0, c.Y1), (x, y),
            (x, c.Y0), (c.X1, y), (x, c.Y1), (c.X0, y) };
    }
    private static bool LevelRange(double low, double high, Settings settings, out int first, out int last)
    {
        double a = Math.Ceiling((low - settings.FirstOffset) / settings.Pitch), b = Math.Floor((high - settings.FirstOffset) / settings.Pitch);
        first = 0; last = -1;
        if (a >= settings.FrontCount || b < 0 || !double.IsFinite(a) || !double.IsFinite(b)) return false;
        first = (int)Math.Max(0, a); last = (int)Math.Min(settings.FrontCount - 1, b); return first <= last;
    }

    private static bool Emit(Cell cell, (long X, long Y)[] sites, double[] scalar, int level, double offset,
        List<Fragment> fragments, int maximumFragments, Rect64 requestedBounds)
    {
        var intersections = new List<Point64>(4);
        for (int i = 0; i < 4; i++)
        {
            int j = (i + 1) % 4; double a = scalar[i] - offset, b = scalar[j] - offset;
            if (a == 0 && b == 0) continue;
            if (a == 0 || b == 0 || a < 0 && b > 0 || a > 0 && b < 0)
            {
                double t = a == 0 ? 0 : b == 0 ? 1 : a / (a - b);
                var point = new Point64((long)Math.Round(sites[i].X + t * (sites[j].X - sites[i].X)),
                    (long)Math.Round(sites[i].Y + t * (sites[j].Y - sites[i].Y)));
                if (!intersections.Contains(point)) intersections.Add(point);
            }
        }
        // Ambiguous four-crossing cells retain both candidate pairs. Neither
        // pairing can authorize a chord; original physical protection follows.
        for (int i = 1; i < intersections.Count; i += 2)
        {
            var a = intersections[i - 1]; var b = intersections[i];
            if (!ClipRequestedWindow(ref a, ref b, requestedBounds) || a == b) continue;
            if (fragments.Count >= maximumFragments) return false;
            fragments.Add(new(a, b, level, offset));
        }
        return true;
    }

    internal static bool ClipRequestedWindow(ref Point64 a, ref Point64 b, Rect64 bounds)
    {
        double x = a.X, y = a.Y, dx = b.X - a.X, dy = b.Y - a.Y, lower = 0, upper = 1;
        bool Restrict(double p, double q)
        {
            if (p == 0) return q >= 0;
            double t = q / p;
            if (p < 0) { if (t > upper) return false; lower = Math.Max(lower, t); }
            else { if (t < lower) return false; upper = Math.Min(upper, t); }
            return lower <= upper;
        }
        if (!Restrict(-dx, x - bounds.left) || !Restrict(dx, bounds.right - x)
            || !Restrict(-dy, y - bounds.top) || !Restrict(dy, bounds.bottom - y)) return false;
        a = new(Math.Clamp((long)Math.Round(x + lower * dx), bounds.left, bounds.right),
            Math.Clamp((long)Math.Round(y + lower * dy), bounds.top, bounds.bottom));
        b = new(Math.Clamp((long)Math.Round(x + upper * dx), bounds.left, bounds.right),
            Math.Clamp((long)Math.Round(y + upper * dy), bounds.top, bounds.bottom));
        return true;
    }
}
