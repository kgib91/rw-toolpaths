using System.Diagnostics;

namespace RW.Toolpaths.Milling.Acceleration;

// This owner stores geometry NOMINATIONS only. No path/stock/floor/wall/grade,
// target coverage, physical cutter predicate or finished-stock decision enters it.
internal sealed class VCarveGpuFrontGeometryCache : IDisposable
{
    internal readonly record struct PatchKey(long X0, long Y0, long X1, long Y1, long PrecisionBits, int Modes);
    internal readonly record struct Patch(double MinimumSigned, double MaximumSigned,
        double V0, double V1, double V2, double V3, double Origin, double Gx, double Gy,
        int NominationMode, double ObservedFitError);
    private readonly object _cacheGate = new();
    private readonly VCarveGpuDistanceFrontNominator.SampleBatch _backend;
    private readonly Dictionary<(long X, long Y), GpuBoundarySampleNomination> _sites;
    private readonly Queue<(long X, long Y)> _siteOrder = new();
    private readonly Dictionary<PatchKey, Patch> _patches;
    private readonly Queue<PatchKey> _patchOrder = new();
    private readonly int _maximumSites, _maximumPatches;
    private bool _disposed, _failed;
    internal VCarveGpuDistanceFrontNominator.BoundarySnapshot Owner { get; }
    internal long SiteHits { get; private set; }
    internal long SiteMisses { get; private set; }
    internal long PatchHits { get; private set; }
    internal long PatchMisses { get; private set; }
    internal long SiteEvictions { get; private set; }
    internal long PatchEvictions { get; private set; }
    internal double BackendMilliseconds { get; private set; }
    internal long ReservedBytes { get; }
    internal int SiteCount { get { lock (_cacheGate) return _sites.Count; } }
    internal int PatchCount { get { lock (_cacheGate) return _patches.Count; } }
    internal bool Available { get { lock (_cacheGate) return !_disposed && !_failed; } }
    internal string? FailureReason { get; private set; }

    internal VCarveGpuFrontGeometryCache(VCarveGpuDistanceFrontNominator.BoundarySnapshot owner,
        VCarveGpuDistanceFrontNominator.SampleBatch backend, int maximumSites = 262_144,
        int maximumPatches = 32_768, long maximumBytes = 64L << 20)
    {
        ArgumentNullException.ThrowIfNull(owner); ArgumentNullException.ThrowIfNull(backend);
        if (maximumSites is < 1 or > 1_000_000 || maximumPatches is < 1 or > 200_000
            || maximumBytes <= 131_072 || maximumBytes > 64L << 20) throw new ArgumentOutOfRangeException(nameof(maximumBytes));
        // Conservative retained storage envelopes cover dictionary entries,
        // spare capacity, FIFO arrays and fixed owners, not only payload bytes.
        long proposed = checked(131_072L + 160L * maximumSites + 384L * maximumPatches);
        if (proposed > maximumBytes) throw new ArgumentOutOfRangeException(nameof(maximumBytes), "Geometry cache envelope exceeds its explicit byte cap.");
        Owner = owner; _backend = backend; _maximumSites = maximumSites; _maximumPatches = maximumPatches; ReservedBytes = proposed;
        _sites = new(Math.Min(maximumSites, 4096)); _patches = new(Math.Min(maximumPatches, 4096));
    }

    internal bool TryNominate(GpuBoundaryQuery[] queries, GpuBoundarySampleNomination[] nominations,
        int count, CancellationToken token, out string reason)
    {
        ArgumentNullException.ThrowIfNull(queries); ArgumentNullException.ThrowIfNull(nominations);
        if (count < 0 || count > queries.Length || count > nominations.Length || count > 1_000_000)
            throw new ArgumentOutOfRangeException(nameof(count));
        reason = ""; Array.Fill(nominations, GpuBoundarySampleNomination.Refused(GpuBoundaryRefusal.InputDomain), 0, count);
        token.ThrowIfCancellationRequested();
        lock (_cacheGate)
        {
            if (_disposed || _failed) { reason = FailureReason ?? "geometry-cache-disposed"; return false; }
            try
            {
                var missing = new List<GpuBoundaryQuery>(); var ordinals = new List<int>();
                for (int i = 0; i < count; i++)
                {
                    if ((i & 255) == 0) token.ThrowIfCancellationRequested(); var q = queries[i];
                    if (_sites.TryGetValue((q.X, q.Y), out var cached)) { nominations[i] = cached; SiteHits++; }
                    else { missing.Add(q); ordinals.Add(i); SiteMisses++; }
                }
                if (missing.Count != 0)
                {
                    var needed = missing.ToArray(); var returned = new GpuBoundarySampleNomination[needed.Length]; long started = Stopwatch.GetTimestamp();
                    bool completed = _backend(needed, returned, needed.Length, token, out reason);
                    BackendMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                    if (!completed)
                    {
                        _failed = true; FailureReason = reason; Clear();
                        Array.Fill(nominations, GpuBoundarySampleNomination.Refused(GpuBoundaryRefusal.Runtime), 0, count); return false;
                    }
                    token.ThrowIfCancellationRequested();
                    for (int i = 0; i < needed.Length; i++)
                    {
                        if ((i & 255) == 0) token.ThrowIfCancellationRequested();
                        var result = returned[i]; nominations[ordinals[i]] = result;
                        // Work refusal/negative/partial results are never cached.
                        if (result.HasNomination && (uint)result.SegmentOrdinal < (uint)Owner.SegmentCount
                            && double.IsFinite(result.Parameter) && result.Parameter >= 0 && result.Parameter <= 1
                            && double.IsFinite(result.SquaredDistance) && result.SquaredDistance >= 0
                            && result.Winding >= -Owner.SegmentCount && result.Winding <= Owner.SegmentCount)
                        {
                            var key = (needed[i].X, needed[i].Y);
                            if (_sites.ContainsKey(key)) continue;
                            while (_sites.Count >= _maximumSites) { _sites.Remove(_siteOrder.Dequeue()); SiteEvictions++; }
                            _sites.Add(key, result); _siteOrder.Enqueue(key);
                        }
                    }
                }
                token.ThrowIfCancellationRequested(); return true;
            }
            catch (OperationCanceledException)
            { Array.Fill(nominations, GpuBoundarySampleNomination.Refused(GpuBoundaryRefusal.Cancelled), 0, count); throw; }
            catch
            {
                _failed = true; FailureReason = "geometry-cache-backend-exception"; Clear();
                Array.Fill(nominations, GpuBoundarySampleNomination.Refused(GpuBoundaryRefusal.Runtime), 0, count); throw;
            }
        }
    }

    internal bool TryPatch(PatchKey key, out Patch patch)
    {
        lock (_cacheGate)
        {
            patch = default;
            if (_disposed || _failed) return false;
            bool found = _patches.TryGetValue(key, out patch);
            if (found) PatchHits++; else PatchMisses++; return found;
        }
    }

    internal void StorePatch(PatchKey key, Patch patch)
    {
        lock (_cacheGate)
        {
            if (_disposed || _failed || _patches.ContainsKey(key)) return;
            while (_patches.Count >= _maximumPatches) { _patches.Remove(_patchOrder.Dequeue()); PatchEvictions++; }
            _patches.Add(key, patch); _patchOrder.Enqueue(key);
        }
    }

    private void Clear() { _sites.Clear(); _siteOrder.Clear(); _patches.Clear(); _patchOrder.Clear(); }
    public void Dispose() { lock (_cacheGate) { if (_disposed) return; _disposed = true; Clear(); } }
}
