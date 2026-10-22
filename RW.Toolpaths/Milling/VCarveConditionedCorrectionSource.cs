using System.Diagnostics;
using System.Security.Cryptography;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling.Acceleration;

namespace RW.Toolpaths.Milling;

// Immutable, invocation-owned NOM geometry. No inside, clearance, floor,
// physical radius, authored schedule, fixed target or stock authority changes.
internal sealed class VCarveConditionedCorrectionSource
{
    internal const string SelectionEnvironment = "RW_TOOLPATHS_VCARVE_CONDITIONED_CORRECTION_SOURCE";
    internal const int MaximumGridMove = 32;
    internal const int MinimumSourcePoints = 4096;
    private const int MaximumSourcePoints = 4_000_000;
    private readonly Paths64 _source;
    private readonly List<List<PointD>> _all;
    private readonly Rect64 _originalBounds;
    private readonly Rect64[] _ringBounds;
    private readonly object _queryLock = new();
    private readonly Dictionary<(double Radius, double ArcTolerance), List<List<PointD>>> _wholeSourceInsets = new();
    private long _queries, _submittedPoints, _excludedPoints;
    private double _cropMilliseconds;
    internal int OriginalPoints { get; }
    internal int NominationPoints { get; }
    internal double PreparationMilliseconds { get; private set; }
    internal string OriginalHash { get; }
    internal string NominationHash { get; }
    internal double InwardPadMm => MaximumGridMove / PathUtils.Scale;
    internal long Queries { get { lock (_queryLock) return _queries; } }
    internal bool HasWholeSourceInsets { get { lock (_queryLock) return _wholeSourceInsets.Count != 0; } }
    private VCarveConditionedCorrectionSource(Paths64 source, Paths64 original, double elapsed,
        string originalHash, string nominationHash)
    {
        _source = source; _all = Convert(source); _originalBounds = Clipper.GetBounds(original);
        _ringBounds = source.Select(r => Clipper.GetBounds(r)).ToArray();
        OriginalPoints = original.Sum(r => r.Count); NominationPoints = source.Sum(r => r.Count);
        PreparationMilliseconds = elapsed; OriginalHash = originalHash; NominationHash = nominationHash;
    }
    internal static bool TryCreate(Paths64 original, CancellationToken token,
        Action<string>? diagnostic, out VCarveConditionedCorrectionSource? result, out string reason)
    {
        result = null; reason = "unsupported-source"; token.ThrowIfCancellationRequested();
        long started = Stopwatch.GetTimestamp(); long originalPoints = 0;
        foreach (var ring in original)
        {
            token.ThrowIfCancellationRequested(); originalPoints += ring.Count;
            if (ring.Count < 3 || ring.Count > 1_000_000 || originalPoints > MaximumSourcePoints) return false;
        }
        if (originalPoints < MinimumSourcePoints) { reason = "small-source-original"; return false; }
        var reduced = new Paths64(original.Count); long verified = 0; int refused = 0, retainedRings = 0;
        foreach (var ring in original)
        {
            token.ThrowIfCancellationRequested();
            double remaining = 5 - Stopwatch.GetElapsedTime(started).TotalSeconds;
            if (remaining <= 0) { reason = "source-conditioning-time-budget"; return false; }
            var closed = ring.ToList(); closed.Add(closed[0]);
            var r = VCarveGpuFrontTraceReductionNominator.Reduce(closed, true, MaximumGridMove,
                token, TimeSpan.FromSeconds(remaining));
            // A refusal never publishes a partially conditioned source. The
            // caller retains its complete original Crop/cache owner.
            if (!r.Completed) { reason = r.Reason; return false; }
            var candidate = new Path64(r.Points.Take(r.Points.Length - 1));
            double oldArea = Clipper.Area(ring), newArea = Clipper.Area(candidate);
            bool sameOrientation = double.IsFinite(newArea) && newArea != 0 && oldArea * newArea > 0;
            reduced.Add(sameOrientation ? candidate : new Path64(ring));
            if (!sameOrientation) retainedRings++;
            refused += r.RefusedReplacements; verified += r.VerifiedVertices;
        }
        token.ThrowIfCancellationRequested();
        // Distinct reduced filled NOM arrangement. The exact finite-chain
        // bound does not certify its winding, topology or physical inclusion.
        var source = Clipper.Union(reduced, FillRule.NonZero);
        token.ThrowIfCancellationRequested();
        long points = source.Sum(r => (long)r.Count);
        if (source.Count == 0 || points > MaximumSourcePoints)
        { reason = "conditioned-union-empty-or-point-budget"; return false; }
        string originalHash = Hash(original, token), nominationHash = Hash(source, token);
        double elapsed = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        if (elapsed > 5000) { reason = "source-conditioning-time-budget"; return false; }
        var completed = new VCarveConditionedCorrectionSource(source, original, elapsed, originalHash, nominationHash);
        token.ThrowIfCancellationRequested(); elapsed = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        if (elapsed > 5000) { reason = "source-conditioning-time-budget"; return false; }
        completed.PreparationMilliseconds = elapsed; result = completed;
        reason = "complete-bounded-NOM-source";
        diagnostic?.Invoke($"stage=conditioned-correction-source event=prepared originalPoints={originalPoints} "
            + $"nominationPoints={points} nativeChainBudget={MaximumGridMove} inwardPadMm={result.InwardPadMm:R} "
            + $"verifiedVertices={verified} refusedReplacements={refused} retainedRings={retainedRings} "
            + $"elapsedMs={elapsed:F3} originalHash={originalHash} nominationHash={nominationHash} "
            + "sourceOwner=one-ApplyCore completeOriginalFallback=True chainBoundIsNotFilledTopology=True "
            + "physicalRadiusScheduleFloorArtworkChannelTargetUnchanged=True stockAuthority=False");
        return true;
    }
    internal List<List<PointD>> Crop(Rect64 originalCrop, CancellationToken token)
    {
        token.ThrowIfCancellationRequested(); long started = Stopwatch.GetTimestamp();
        if (originalCrop.left <= _originalBounds.left && originalCrop.top <= _originalBounds.top
            && originalCrop.right >= _originalBounds.right && originalCrop.bottom >= _originalBounds.bottom)
        { Record(NominationPoints, started); return _all; }
        var selected = new Paths64(); long submitted = 0;
        for (int i = 0; i < _source.Count; i++)
        {
            token.ThrowIfCancellationRequested(); if (!originalCrop.Intersects(_ringBounds[i])) continue;
            selected.Add(_source[i]); submitted += _source[i].Count;
        }
        var clipped = Clipper.Union(Clipper.RectClip(originalCrop, selected), FillRule.NonZero);
        token.ThrowIfCancellationRequested(); var result = Convert(clipped); Record(submitted, started); return result;
    }
    private void Record(long submitted, long started)
    {
        double elapsed = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        lock (_queryLock) { _queries++; _submittedPoints += submitted; _excludedPoints += NominationPoints - submitted; _cropMilliseconds += elapsed; }
    }
    internal bool TryGetWholeSourceInset(double radius, double arc, out List<List<PointD>> inset)
    { lock (_queryLock) return _wholeSourceInsets.TryGetValue((radius, arc), out inset!); }
    internal void RememberWholeSourceInset(List<List<PointD>> source, double radius, double arc, List<List<PointD>> inset)
    { if (ReferenceEquals(source, _all)) lock (_queryLock) _wholeSourceInsets[(radius, arc)] = inset; }
    internal string Describe()
    {
        lock (_queryLock) return $"conditionedNomQueries={_queries} originalSourcePoints={OriginalPoints} "
            + $"nominationSourcePoints={NominationPoints} nominationSubmittedPoints={_submittedPoints} "
            + $"nominationExcludedPoints={_excludedPoints} nominationCropMs={_cropMilliseconds:F3} "
            + $"preparationMs={PreparationMilliseconds:F3} nativeChainBudget={MaximumGridMove} inwardPadMm={InwardPadMm:R} "
            + "nominationOnly=True originalCropBoundsAndWholeSourceIdentityRetained=True originalPhysicalOwnersUnchanged=True";
    }
    private static List<List<PointD>> Convert(Paths64 rows) => rows.Select(r => r.Select(p =>
        new PointD(p.X / PathUtils.Scale, p.Y / PathUtils.Scale)).ToList()).ToList();
    private static string Hash(Paths64 rows, CancellationToken token)
    {
        using var hash = IncrementalHash.CreateHash(HashAlgorithmName.SHA256); long work = 0;
        void Add(long value) { Span<byte> bytes = stackalloc byte[8]; System.Buffers.Binary.BinaryPrimitives.WriteInt64LittleEndian(bytes, value); hash.AppendData(bytes); }
        Add(rows.Count); foreach (var row in rows) { Add(row.Count); foreach (var p in row)
            { if ((work++ & 255) == 0) token.ThrowIfCancellationRequested(); Add(p.X); Add(p.Y); } }
        token.ThrowIfCancellationRequested(); return System.Convert.ToHexString(hash.GetHashAndReset()).ToLowerInvariant();
    }
}
