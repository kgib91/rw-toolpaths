using Clipper2Lib;
using System.Diagnostics;

namespace RW.Toolpaths.Milling.Acceleration;

// Generation-owned candidate construction. This type never accepts a cut,
// changes a target, declares a component empty, or supplies physical coverage.
// Its caller must apply the original snapping/floor/dual-wall/grade guards and
// the entire unchanged target certificate. Refusal means the old constructor.
internal sealed class VCarveGpuCorrectionFrontNominator : IDisposable
{
    private const int MaximumSitesPerComponent = 200_000;
    private const int MaximumEarlyRound = 5;
    private const long CoordinateLimit = 1L << 29;
    internal delegate VCarveGpuDistanceFrontNominator.Result Producer(
        VCarveGpuDistanceFrontNominator.BoundarySnapshot snapshot, VCarveGpuDistanceFrontNominator.Settings settings,
        VCarveGpuDistanceFrontNominator.SampleBatch sampler, CancellationToken token, VCarveGpuFrontGeometryCache geometry);
    private readonly object _gate = new();
    private readonly VCarveGpuDistanceFrontNominator.BoundarySnapshot _snapshot;
    private readonly VCarveGpuDistanceFrontNominator.SampleBatch _sample;
    private readonly IDisposable _samplerLifetime;
    private readonly Func<string> _samplerStatistics;
    private readonly Producer _producer;
    private readonly Func<long, double> _elapsedMilliseconds;
    private readonly Func<long> _timestamp;
    private readonly VCarveGpuFrontGeometryCache _geometry;
    private readonly Action<string>? _diagnostic;
    private bool _disposed;
    internal long CompletedComponents { get; private set; }
    internal long RefusedComponents { get; private set; }
    internal long MaterializedPoints { get; private set; }
    internal double NominationMilliseconds { get; private set; }
    internal double QueueMilliseconds { get; private set; }
    internal bool CuttingAuthority => false;
    internal bool FixedTargetCertified => false;
    internal bool FinishedStockAudited => false;
    internal bool TerminalOwnership => false;
    internal string? LastReason { get; private set; }
    internal long LiteralEdgesQualified { get; private set; }
    internal long NominatedChordsAfterReduction { get; private set; }
    internal long ReductionReplacementChords { get; private set; }
    internal long ReductionRefusedReplacements { get; private set; }
    internal long ReductionRefusals { get; private set; }
    internal double ReductionMilliseconds { get; private set; }

    private VCarveGpuCorrectionFrontNominator(VCarveGpuDistanceFrontNominator.BoundarySnapshot snapshot,
        VCarveGpuDistanceFrontNominator.SampleBatch sample, IDisposable samplerLifetime,
        Func<string> samplerStatistics, Action<string>? diagnostic, Producer? producer = null,
        Func<long, double>? elapsedMilliseconds = null, Func<long>? timestamp = null)
    {
        _snapshot = snapshot; _sample = sample; _samplerLifetime = samplerLifetime;
        _samplerStatistics = samplerStatistics; _diagnostic = diagnostic;
        _geometry = new(snapshot, sample);
        _producer = producer ?? ((owner, settings, sampler, token, geometry)
            => VCarveGpuDistanceFrontNominator.Run(owner, settings, sampler, token, geometryCache: geometry));
        _elapsedMilliseconds = elapsedMilliseconds ?? (started => Stopwatch.GetElapsedTime(started).TotalMilliseconds);
        _timestamp = timestamp ?? Stopwatch.GetTimestamp;
    }

    // Internal artifact controls use the same cache/materialization/refusal path
    // with an independent sampling oracle and a bounded deterministic producer.
    // These injected nominations acquire no additional geometry or cut authority.
    internal static VCarveGpuCorrectionFrontNominator CreateForNominationControls(
        VCarveGpuDistanceFrontNominator.BoundarySnapshot snapshot, VCarveGpuDistanceFrontNominator.SampleBatch sample,
        IDisposable samplerLifetime, Producer producer, Action<string>? diagnostic = null,
        Func<long, double>? elapsedMilliseconds = null, Func<long>? timestamp = null)
    {
        try { return new(snapshot, sample, samplerLifetime, () => "sampler=independent-control", diagnostic, producer, elapsedMilliseconds, timestamp); }
        catch { samplerLifetime.Dispose(); throw; }
    }

    internal static bool TryCreate(IReadOnlyList<IReadOnlyList<PointD>> originalChannel,
        CancellationToken token, Action<string>? diagnostic,
        out VCarveGpuCorrectionFrontNominator? context, out string reason, VCarveGenerationPolicy? policy = null)
    {
        context = null; token.ThrowIfCancellationRequested();
        if (originalChannel is null || originalChannel.Count is < 1 or > 16_384)
        { reason = "source-row-budget-or-domain"; return false; }
        for (int i = 0; i < originalChannel.Count; i++)
        {
            if ((i & 255) == 0) token.ThrowIfCancellationRequested();
            if (originalChannel[i] is null || originalChannel[i].Count < 3)
            { reason = "source-row-budget-or-domain"; return false; }
        }
        if (!VCarveGpuDistanceFrontNominator.BoundarySnapshot.TryPrepare(originalChannel, token,
            out var snapshot, out reason)) return false;
        if (!snapshot!.TryCreateSampler(token, diagnostic, out var sampler, out reason, policy)) return false;
        VCarveGpuCorrectionFrontNominator? owner = null;
        try
        {
            var actualSampler = sampler!;
            owner = new VCarveGpuCorrectionFrontNominator(snapshot, actualSampler.TryNominate, actualSampler,
                () => $"deviceQueries={actualSampler.QueryCount} kernelMs={actualSampler.KernelMilliseconds:F3} transferMs={actualSampler.TransferMilliseconds:F3}", diagnostic);
            token.ThrowIfCancellationRequested(); context = owner; return true;
        }
        catch { if (owner is not null) owner.Dispose(); else sampler!.Dispose(); throw; }
    }

    // Only early correction rounds use this nominator. The existing original
    // minimum-engagement terminal/discovery owner remains required in rounds6/7.
    // This is not the separate initial mandatory-perimeter/front replacement.
    internal bool TryNominate(VCarveSmoothing.CorrectionConstructorInputs? input,
        CancellationToken token, out List<TaggedToolpath> paths, out string reason)
    {
        paths = new(); reason = ""; token.ThrowIfCancellationRequested();
        long queuedAt = _timestamp();
        lock (_gate)
        {
            token.ThrowIfCancellationRequested();
            double queueMilliseconds = _elapsedMilliseconds(queuedAt);
            long started = _timestamp();
            QueueMilliseconds += queueMilliseconds;
            bool Finish(bool complete, string detail, List<TaggedToolpath>? result = null)
            {
                token.ThrowIfCancellationRequested();
                double elapsed = _elapsedMilliseconds(started);
                bool timely = elapsed <= 5_000;
                if (complete && !timely) { complete = false; detail = "component-adapter-time-budget"; }
                Log($"stage=gpu-correction-front event={(complete ? "nomination-before-publication-check" : "refusal")} "
                    + $"round={input?.Round ?? -1} component={input?.Component ?? -1} reason='{detail}' elapsedMs={elapsed:F3} queueMs={queueMilliseconds:F3} workMs={elapsed:F3} "
                    + $"paths={(complete ? result!.Count : 0)} points={(complete ? result!.Sum(path => (long)path.Points.Count) : 0)} "
                    + "candidateOnly=True cuttingAuthority=False terminalOwnership=False fixedTargetUnchanged=True "
                    + "requiresSnapFloorDualWallsGradeAndWholeTargetProof=True literalStitchBridges=0 replacementChordsRequireNewGuards=True");
                token.ThrowIfCancellationRequested();
                elapsed = _elapsedMilliseconds(started);
                if (complete && elapsed > 5_000) { complete = false; detail = "component-adapter-time-budget"; }
                NominationMilliseconds += elapsed; LastReason = detail;
                if (complete) { CompletedComponents++; MaterializedPoints += result!.Sum(path => (long)path.Points.Count); }
                else RefusedComponents++;
                return complete;
            }
            try
            {
            if (input is null || _disposed || !_geometry.Available || input.Round is < 0 or > MaximumEarlyRound
                || input.ErrorBoundary is null || input.ErrorBoundary.Count == 0
                || input.ErrorBoundary.Any(ring => ring is null || ring.Count < 3)
                || input.Profile is null || input.Schedule is null || input.PointBudget < 2
                || !double.IsFinite(input.InitialCutDepth) || !double.IsFinite(input.FloorCap)
                || input.FloorCap < 0 || input.InitialCutDepth <= 0)
            { reason = "unsupported-owner-round-or-error-boundary"; Finish(false, reason); return false; }
            if (input.DetectCurves)
            { reason = "unsupported-fitted-curve-posting-context"; Finish(false, reason); return false; }
            double stableDepth = VCarveSmoothing.QuantizeCorrectionDepth(input.InitialCutDepth, input.Schedule.SurfaceZ);
            if (!(stableDepth > .001) || !double.IsFinite(stableDepth))
            { reason = "minimum-engagement-requires-original-terminal-owner"; Finish(false, reason); return false; }
            double radius = MedialAxisToolpaths.CommandedCuttingRadius(input.Profile, 0, 0, 0,
                stableDepth, input.Schedule.SurfaceZ);
            double reach = radius + VCarveSmoothing.CoverageTolerance;
            if (!(radius > 1e-7) || !double.IsFinite(reach))
            { reason = "unsupported-commanded-radius"; Finish(false, reason); return false; }
            double x0 = double.PositiveInfinity, y0 = x0, x1 = double.NegativeInfinity, y1 = x1;
            long errorPoints = 0;
            foreach (var ring in input.ErrorBoundary)
            foreach (var point in ring)
            {
                if ((++errorPoints & 255) == 0) token.ThrowIfCancellationRequested();
                if (errorPoints > input.PointBudget || !double.IsFinite(point.x) || !double.IsFinite(point.y))
                { reason = "error-point-budget-or-domain"; Finish(false, reason); return false; }
                x0 = Math.Min(x0, point.x); y0 = Math.Min(y0, point.y);
                x1 = Math.Max(x1, point.x); y1 = Math.Max(y1, point.y);
            }
            x0 = Math.Floor((x0 - reach) * PathUtils.Scale); y0 = Math.Floor((y0 - reach) * PathUtils.Scale);
            x1 = Math.Ceiling((x1 + reach) * PathUtils.Scale); y1 = Math.Ceiling((y1 + reach) * PathUtils.Scale);
            if (!double.IsFinite(x0) || !double.IsFinite(y0) || !double.IsFinite(x1) || !double.IsFinite(y1)
                || x0 <= -CoordinateLimit || y0 <= -CoordinateLimit || x1 >= CoordinateLimit || y1 >= CoordinateLimit)
            { reason = "nomination-window-domain"; Finish(false, reason); return false; }
            var bounds = new Rect64((long)x0, (long)y0, (long)x1, (long)y1);
            if (!VCarveGpuDistanceFrontNominator.TryCorrectionSetup(input, bounds, 1000, .001,
                MaximumSitesPerComponent, TimeSpan.FromSeconds(3), out var setup, out reason))
            { Finish(false, reason); return false; }
            var nominated = _producer(_snapshot,
                setup.NominationSettings with { CanonicalActualRoots = true }, _sample, token, _geometry);
            token.ThrowIfCancellationRequested();
            if (nominated is null)
            { reason = "producer-uninitialized-result"; Finish(false, reason); return false; }
            if (!nominated.CompletedNomination || !nominated.CanonicalFieldIncidenceMatched
                || nominated.CanonicalInteriorOpenEnds != 0 || nominated.CanonicalBranchVertices != 0)
            { reason = "producer-" + nominated.Reason; Finish(false, reason); return false; }
            if (nominated.Fragments.IsDefaultOrEmpty)
            { reason = "empty-nomination-retains-original-reachability-constructor"; Finish(false, reason); return false; }
            if (nominated.Fragments.Length > setup.NominationSettings.MaximumFragments)
            { reason = "producer-fragment-budget"; Finish(false, reason); return false; }
            var fragments = new VCarveGpuFrontStitchNominator.Fragment[nominated.Fragments.Length];
            for (int i = 0; i < fragments.Length; i++)
            {
                if ((i & 255) == 0) token.ThrowIfCancellationRequested();
                var fragment = nominated.Fragments[i];
                if (fragment.FrontIndex < 0 || fragment.FrontIndex >= setup.NominationSettings.FrontCount
                    || BitConverter.DoubleToInt64Bits(fragment.Offset) != BitConverter.DoubleToInt64Bits(
                        setup.NominationSettings.FirstOffset + fragment.FrontIndex * setup.NominationSettings.Pitch))
                { reason = "fragment-authored-level-or-offset-mismatch"; Finish(false, reason); return false; }
                if (!InWindow(fragment.A, bounds) || !InWindow(fragment.B, bounds))
                { reason = "fragment-outside-requested-window"; Finish(false, reason); return false; }
                fragments[i] = new(new(input.RegionIndex, input.Component, setup.DepthPassIndex,
                    BitConverter.DoubleToInt64Bits(-setup.CutDepth), fragment.FrontIndex,
                    BitConverter.DoubleToInt64Bits(fragment.Offset)), fragment.A, fragment.B);
            }
            var stitched = VCarveGpuFrontStitchNominator.Stitch(fragments, token, TimeSpan.FromSeconds(1));
            if (!stitched.Completed || stitched.OutputEdges != fragments.Length || stitched.BranchVertices != 0)
            { reason = "stitch-" + stitched.Reason; Finish(false, reason); return false; }
            long literalEdges = stitched.Traces.Sum(trace => (long)trace.Points.Length - 1);
            if (literalEdges != fragments.Length)
            { reason = "literal-stitch-input-edge-multiset-mismatch"; Finish(false, reason); return false; }
            LiteralEdgesQualified += literalEdges;
            var materialized = new List<TaggedToolpath>(stitched.Traces.Length); long points = 0, chords = 0;
            long replacements = 0, refusedReplacements = 0, reductionRefusals = 0;
            double reductionMilliseconds = 0;
            foreach (var trace in stitched.Traces)
            {
                token.ThrowIfCancellationRequested();
                // Literal incidence/edge multiset was qualified above. Optional
                // reduction replaces only nomination chords; every resulting
                // new actual F3 chord still receives the original guards.
                var reduction = VCarveGpuFrontTraceReductionNominator.Reduce(trace.Points,
                    trace.IsClosed, 32, token, TimeSpan.FromSeconds(1));
                reductionMilliseconds += reduction.ElapsedMilliseconds;
                replacements += reduction.ReplacementChords;
                refusedReplacements += reduction.RefusedReplacements;
                var chosen = trace.Points;
                if (reduction.Completed) chosen = reduction.Points;
                else reductionRefusals++; // Keep the complete original trace.
                var coordinates = new Point3D[chosen.Length];
                for (int i = 0; i < coordinates.Length; i++)
                {
                    if ((i & 255) == 0) token.ThrowIfCancellationRequested();
                    var point = chosen[i]; coordinates[i] = new(point.X / PathUtils.Scale, point.Y / PathUtils.Scale, -setup.CutDepth);
                }
                points += coordinates.Length; chords += coordinates.Length - 1;
                if (coordinates.Length < 2 || points > input.PointBudget)
                { reason = "materialization-point-budget"; Finish(false, reason); return false; }
                materialized.Add(new(coordinates, Array.Empty<ToolpathSpan>(), input.RegionIndex,
                    "clearing", setup.DepthPassIndex, detectCurves: input.DetectCurves));
            }
            if (materialized.Sum(path => (long)Math.Max(0, path.Points.Count - 1)) != chords)
            { reason = "materialization-reduced-chord-count-mismatch"; Finish(false, reason); return false; }
            NominatedChordsAfterReduction += chords;
            ReductionReplacementChords += replacements;
            ReductionRefusedReplacements += refusedReplacements;
            ReductionRefusals += reductionRefusals; ReductionMilliseconds += reductionMilliseconds;
            Log($"stage=gpu-correction-front-reduction round={input.Round} component={input.Component} "
                + $"literalInputEdges={literalEdges} nominatedOutputChords={chords} replacementChords={replacements} "
                + $"refusedReplacements={refusedReplacements} reductionRefusals={reductionRefusals} reductionMs={reductionMilliseconds:F3} "
                + "literalEdgeMultisetQualifiedBeforeReduction=True literalStitchBridges=0 "
                + "nominationBoundNativeTicks=32 nominationBoundMillimetres=0.0009765625 "
                + "newFiniteChordsRequireSnapFloorDualWallsGradeAndWholeTargetProof=True stockCredit=0");
            token.ThrowIfCancellationRequested();
            reason = "canonical-front-nomination-only";
            if (!Finish(true, reason, materialized))
            { reason = "component-adapter-time-budget"; return false; }
            paths = materialized; return true;
            }
            catch (OperationCanceledException) { throw; }
            catch (Exception error) when (error is ArgumentException or InvalidOperationException or OverflowException
                or DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
            {
                token.ThrowIfCancellationRequested(); reason = "recoverable-nomination-" + error.GetType().Name;
                Finish(false, reason); return false;
            }
        }
    }

    private void Log(string message)
    {
        try { _diagnostic?.Invoke(message); }
        catch (Exception) { }
    }

    private static bool InWindow(Point64 point, Rect64 bounds)
        => point.X >= bounds.left && point.X <= bounds.right && point.Y >= bounds.top && point.Y <= bounds.bottom;

    public void Dispose()
    {
        lock (_gate)
        {
            if (_disposed) return; _disposed = true;
            try
            {
                Log($"stage=gpu-correction-front event=dispose completedComponents={CompletedComponents} "
                    + $"refusedComponents={RefusedComponents} materializedPoints={MaterializedPoints} "
                    + $"nominationMs={NominationMilliseconds:F3} queueMs={QueueMilliseconds:F3} {_samplerStatistics()} "
                    + $"siteHits={_geometry.SiteHits} patchHits={_geometry.PatchHits} "
                    + $"literalEdgesQualified={LiteralEdgesQualified} nominatedChordsAfterReduction={NominatedChordsAfterReduction} "
                    + $"replacementChords={ReductionReplacementChords} refusedReplacements={ReductionRefusedReplacements} "
                    + $"reductionRefusals={ReductionRefusals} reductionMs={ReductionMilliseconds:F3} stockCredit=0");
            }
            finally { _geometry.Dispose(); _samplerLifetime.Dispose(); }
        }
    }
}
