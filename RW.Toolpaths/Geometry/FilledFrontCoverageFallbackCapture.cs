using Clipper2Lib;
using System.Collections.Immutable;

namespace RW.Toolpaths.Geometry;

/// <summary>Bounded, opt-in snapshots of the unchanged filled-front stopping
/// predicate. Every exposed coordinate is a detached value copy. Captures and
/// observer failures have no cutting or component-selection authority.</summary>
internal static class FilledFrontCoverageFallbackCapture
{
    internal const int MaximumCaptures = 4;
    internal const long MaximumCombinedPoints = 1_000_000;
    internal const int MaximumCombinedPathRows = 16_384;
    private const int MaximumAcceptedFootprints = 16_384;
    // Even the AsyncLocal owner is allocated only when a diagnostic installs a
    // scope. An absent observer requires only a nullable field/value read.
    private static AsyncLocal<Scope?>? _invocation;
    internal static Scope? Current => Volatile.Read(ref _invocation)?.Value is { IsDisposed: false } scope ? scope : null;

    internal readonly record struct Parameters(long ScaleBits, long RadiusBits, long ProofArcBits,
        long CoverageErrorBits, long WitnessRadiusBits, bool FreshWitness, bool PreparedContainment,
        bool EvidenceCacheEnabled);

    // This transient input is used only while making an observer-owned copy.
    internal readonly record struct FootprintInput(Paths64 IntegerPaths, IReadOnlyList<List<PointD>> Paths);
    internal sealed record FootprintSnapshot(ImmutableArray<ImmutableArray<Point64>> IntegerPaths,
        ImmutableArray<ImmutableArray<PointD>> Paths);
    internal sealed record Snapshot(int CaptureOrdinal, int ComponentOrdinal, int VisitOrdinal,
        int ComponentCount, int MaterializedBefore, int MaterializedAfter, Parameters ProofParameters,
        ImmutableArray<ImmutableArray<PointD>> CanonicalComponent, bool ClipWasNull,
        ImmutableArray<PointD> Clip, FootprintSnapshot Candidate,
        ImmutableArray<FootprintSnapshot> AcceptedOriginalOrder,
        ImmutableArray<ImmutableArray<Point64>> ExactOrderedPrefix,
        ImmutableArray<ImmutableArray<PointD>> Fresh,
        ImmutableArray<ImmutableArray<PointD>> ErodedInterior,
        string Outcome, bool Retained, long CombinedPoints, long CombinedPathRows,
        int MinimumAcceptedFootprints);

    internal static Scope BeginScope(Action<Snapshot> observer, int minimumAcceptedFootprints = 0)
    {
        ArgumentNullException.ThrowIfNull(observer);
        if (minimumAcceptedFootprints is < 0 or > MaximumAcceptedFootprints)
            throw new ArgumentOutOfRangeException(nameof(minimumAcceptedFootprints));
        var invocation = Volatile.Read(ref _invocation);
        if (invocation is null)
        {
            var candidate = new AsyncLocal<Scope?>();
            invocation = Interlocked.CompareExchange(ref _invocation, candidate, null) ?? candidate;
        }
        var scope = new Scope(observer, invocation.Value, minimumAcceptedFootprints);
        invocation.Value = scope;
        return scope;
    }

    internal sealed class Scope : IDisposable
    {
        private readonly Action<Snapshot> _observer;
        private readonly Scope? _previous;
        private readonly object _observerGate = new();
        private int _reservations, _completed, _limitLogged, _disposed, _filtered;
        internal bool IsDisposed => Volatile.Read(ref _disposed) != 0;
        internal int Reservations => Volatile.Read(ref _reservations);
        internal int Completed => Volatile.Read(ref _completed);
        internal int MinimumAcceptedFootprints { get; }

        internal Scope(Action<Snapshot> observer, Scope? previous, int minimumAcceptedFootprints)
        { _observer = observer; _previous = previous; MinimumAcceptedFootprints = minimumAcceptedFootprints; }

        internal Pending? TryBegin(int component, int visit, int count, int materialized,
            Parameters parameters, IReadOnlyList<IReadOnlyList<PointD>> canonicalComponent,
            IReadOnlyList<PointD>? clip, FootprintInput candidate,
            IEnumerable<FootprintInput> acceptedOriginalOrder, CancellationToken token,
            int acceptedFootprintCount = -1)
        {
            if (IsDisposed) return null;
            if (MinimumAcceptedFootprints > 0)
            {
                try
                {
                    // The production caller supplies the captured list's Count;
                    // optional diagnostic callers enumerate only the filter bound.
                    int countForFilter = acceptedFootprintCount;
                    if (countForFilter < 0 && !acceptedOriginalOrder.TryGetNonEnumeratedCount(out countForFilter))
                        countForFilter = acceptedOriginalOrder.Take(MinimumAcceptedFootprints).Count();
                    if (countForFilter < MinimumAcceptedFootprints)
                    {
                        int filtered = Interlocked.Increment(ref _filtered);
                        if (filtered <= 4 || (filtered & (filtered - 1)) == 0)
                            Console.WriteLine($"[FilledFrontCoverage.FallbackCapture] event=filtered accepted={countForFilter} "
                                + $"minimumAccepted={MinimumAcceptedFootprints} filtered={filtered} reservationConsumed=False selectionAuthority=False");
                        return null;
                    }
                }
                catch (Exception exception)
                {
                    Log("refused", 0, "filter-" + exception.GetType().Name);
                    return null;
                }
            }
            int ordinal;
            do
            {
                ordinal = Volatile.Read(ref _reservations);
                if (ordinal >= MaximumCaptures)
                {
                    if (Interlocked.Exchange(ref _limitLogged, 1) == 0)
                        Log("refused", ordinal, "capture-count-budget");
                    return null;
                }
            } while (Interlocked.CompareExchange(ref _reservations, ordinal + 1, ordinal) != ordinal);
            ordinal++;
            try
            {
                token.ThrowIfCancellationRequested();
                // Empty diagnostic leaves cannot evade the coordinate budget
                // and produce an unbounded record array.
                var accepted = acceptedOriginalOrder.Take(MaximumAcceptedFootprints + 1).ToArray();
                if (accepted.Length > MaximumAcceptedFootprints)
                { Log("refused", ordinal, "accepted-footprint-budget"); return null; }
                long rows = Rows(candidate) + canonicalComponent.Count + (clip is null ? 0 : 1);
                if (rows > MaximumCombinedPathRows) { Log("refused", ordinal, "input-path-row-budget"); return null; }
                long points = Count(candidate) + canonicalComponent.Sum(path => (long)path.Count) + (clip?.Count ?? 0);
                foreach (var footprint in accepted)
                {
                    rows += Rows(footprint);
                    if (rows > MaximumCombinedPathRows) break;
                    points += Count(footprint);
                    if (points > MaximumCombinedPoints) break;
                }
                if (points > MaximumCombinedPoints) { Log("refused", ordinal, "input-point-budget"); return null; }
                if (rows > MaximumCombinedPathRows) { Log("refused", ordinal, "input-path-row-budget"); return null; }
                var copiedAccepted = ImmutableArray.CreateBuilder<FootprintSnapshot>(accepted.Length);
                foreach (var footprint in accepted) copiedAccepted.Add(Copy(footprint, token));
                var result = new Pending(this, ordinal, component, visit, count, materialized, parameters,
                    CopyWorld(canonicalComponent, token), clip is null, CopyClip(clip, token),
                    Copy(candidate, token), copiedAccepted.MoveToImmutable(), points, rows);
                token.ThrowIfCancellationRequested();
                return result;
            }
            catch (Exception exception)
            {
                Log("refused", ordinal, exception is OperationCanceledException ? "capture-cancelled" : exception.GetType().Name);
                return null;
            }
        }

        internal void Publish(Snapshot snapshot)
        {
            if (IsDisposed) { Log("refused", snapshot.CaptureOrdinal, "scope-disposed"); return; }
            try
            {
                lock (_observerGate)
                {
                    if (IsDisposed) { Log("refused", snapshot.CaptureOrdinal, "scope-disposed"); return; }
                    _observer(snapshot);
                    Interlocked.Increment(ref _completed);
                }
                Console.WriteLine($"[FilledFrontCoverage.FallbackCapture] event=captured ordinal={snapshot.CaptureOrdinal} "
                    + $"component={snapshot.ComponentOrdinal} accepted={snapshot.AcceptedOriginalOrder.Length} "
                    + $"materializedBefore={snapshot.MaterializedBefore} materializedAfter={snapshot.MaterializedAfter} "
                    + $"points={snapshot.CombinedPoints} pathRows={snapshot.CombinedPathRows} minimumAccepted={MinimumAcceptedFootprints} "
                    + $"outcome={snapshot.Outcome} retained={snapshot.Retained} "
                    + $"selectionAuthority=False prefixOrder=unchanged snapshots=detached-readonly");
            }
            catch (Exception exception)
            {
                Log("observer-failed", snapshot.CaptureOrdinal, exception.GetType().Name);
            }
        }

        public void Dispose()
        {
            if (Interlocked.Exchange(ref _disposed, 1) != 0) return;
            // Restore the prior invocation only when this scope is current.
            // Disposing an out-of-order scope never overwrites a newer owner.
            if (ReferenceEquals(_invocation!.Value, this))
            {
                var previous = _previous;
                while (previous is { IsDisposed: true }) previous = previous._previous;
                _invocation.Value = previous;
            }
        }
    }

    internal sealed class Pending
    {
        private readonly Scope _scope;
        private readonly int _ordinal, _component, _visit, _count, _materialized;
        private readonly Parameters _parameters;
        private readonly ImmutableArray<ImmutableArray<PointD>> _canonical;
        private readonly bool _clipWasNull;
        private readonly ImmutableArray<PointD> _clip;
        private readonly FootprintSnapshot _candidate;
        private readonly ImmutableArray<FootprintSnapshot> _accepted;
        private readonly long _inputPoints;
        private readonly long _inputRows;
        private bool _finished;

        internal Pending(Scope scope, int ordinal, int component, int visit, int count, int materialized,
            Parameters parameters, ImmutableArray<ImmutableArray<PointD>> canonical, bool clipWasNull,
            ImmutableArray<PointD> clip, FootprintSnapshot candidate,
            ImmutableArray<FootprintSnapshot> accepted, long inputPoints, long inputRows)
        {
            _scope = scope; _ordinal = ordinal; _component = component; _visit = visit; _count = count;
            _materialized = materialized; _parameters = parameters; _canonical = canonical;
            _clipWasNull = clipWasNull; _clip = clip; _candidate = candidate; _accepted = accepted;
            _inputPoints = inputPoints; _inputRows = inputRows;
        }

        internal void Complete(Paths64 exactOrderedPrefix, IReadOnlyList<List<PointD>>? fresh,
            IReadOnlyList<List<PointD>>? erodedInterior, int materializedAfter, string outcome,
            bool retained, CancellationToken token)
        {
            if (_finished) return;
            _finished = true;
            try
            {
                token.ThrowIfCancellationRequested();
                long rows = _inputRows + exactOrderedPrefix.Count + (fresh?.Count ?? 0) + (erodedInterior?.Count ?? 0);
                if (rows > MaximumCombinedPathRows) { Log("refused", _ordinal, "result-path-row-budget"); return; }
                long points = _inputPoints + exactOrderedPrefix.Sum(path => (long)path.Count)
                    + (fresh?.Sum(path => (long)path.Count) ?? 0)
                    + (erodedInterior?.Sum(path => (long)path.Count) ?? 0);
                if (points > MaximumCombinedPoints) { Log("refused", _ordinal, "result-point-budget"); return; }
                var snapshot = new Snapshot(_ordinal, _component, _visit, _count, _materialized, materializedAfter,
                    _parameters, _canonical, _clipWasNull, _clip, _candidate, _accepted,
                    CopyInteger(exactOrderedPrefix, token),
                    fresh is null ? ImmutableArray<ImmutableArray<PointD>>.Empty : CopyWorld(fresh, token),
                    erodedInterior is null ? ImmutableArray<ImmutableArray<PointD>>.Empty : CopyWorld(erodedInterior, token),
                    outcome, retained, points, rows, _scope.MinimumAcceptedFootprints);
                token.ThrowIfCancellationRequested();
                _scope.Publish(snapshot);
            }
            catch (Exception exception)
            {
                Log("refused", _ordinal, exception is OperationCanceledException ? "capture-cancelled" : exception.GetType().Name);
            }
        }
    }

    private static long Count(FootprintInput input)
        => input.IntegerPaths.Sum(path => (long)path.Count) + input.Paths.Sum(path => (long)path.Count);
    private static long Rows(FootprintInput input) => (long)input.IntegerPaths.Count + input.Paths.Count;
    private static FootprintSnapshot Copy(FootprintInput input, CancellationToken token)
        => new(CopyInteger(input.IntegerPaths, token), CopyWorld(input.Paths, token));
    private static ImmutableArray<ImmutableArray<Point64>> CopyInteger(Paths64 paths, CancellationToken token)
    {
        var result = ImmutableArray.CreateBuilder<ImmutableArray<Point64>>(paths.Count);
        foreach (var path in paths)
        {
            var row = ImmutableArray.CreateBuilder<Point64>(path.Count);
            for (int i = 0; i < path.Count; i++)
            { if ((i & 255) == 0) token.ThrowIfCancellationRequested(); row.Add(path[i]); }
            result.Add(row.MoveToImmutable());
        }
        return result.MoveToImmutable();
    }
    private static ImmutableArray<ImmutableArray<PointD>> CopyWorld(
        IEnumerable<IReadOnlyList<PointD>> paths, CancellationToken token)
    {
        var result = ImmutableArray.CreateBuilder<ImmutableArray<PointD>>();
        foreach (var path in paths)
        {
            var row = ImmutableArray.CreateBuilder<PointD>(path.Count);
            for (int i = 0; i < path.Count; i++)
            { if ((i & 255) == 0) token.ThrowIfCancellationRequested(); row.Add(path[i]); }
            result.Add(row.MoveToImmutable());
        }
        return result.ToImmutable();
    }
    private static ImmutableArray<PointD> CopyClip(IReadOnlyList<PointD>? clip, CancellationToken token)
    {
        if (clip is null) return ImmutableArray<PointD>.Empty;
        var row = ImmutableArray.CreateBuilder<PointD>(clip.Count);
        for (int i = 0; i < clip.Count; i++)
        { if ((i & 255) == 0) token.ThrowIfCancellationRequested(); row.Add(clip[i]); }
        return row.MoveToImmutable();
    }
    private static void Log(string action, int ordinal, string reason)
        => Console.WriteLine($"[FilledFrontCoverage.FallbackCapture] event={action} ordinal={ordinal} reason={reason} "
            + $"maximumCaptures={MaximumCaptures} maximumPoints={MaximumCombinedPoints} maximumPathRows={MaximumCombinedPathRows} selectionAuthority=False");
}
