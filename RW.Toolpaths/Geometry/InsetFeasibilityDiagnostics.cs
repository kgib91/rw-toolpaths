using System.Diagnostics;

namespace RW.Toolpaths.Geometry;

/// <summary>Operation-local aggregates with bounded reproducible witnesses. The
/// opt-in trace uses the same geometric queries and never changes their result.</summary>
internal static class InsetFeasibilityDiagnostics
{
    private static readonly AsyncLocal<Scope?> Current = new();
    private static readonly Scope Unscoped = new("unscoped", null, ownsCurrent: false);

    internal static IDisposable Begin(string consumer, string? operation = null)
    {
        if (Current.Value is not null) return EmptyScope.Instance;
        var scope = new Scope(consumer, operation, ownsCurrent: true);
        Current.Value = scope;
        return scope;
    }

    internal static void Cache(bool possible)
        => (Current.Value ?? Unscoped).Cache(possible);

    internal static void Solve(double radius, bool possible, int probes, long started)
        => (Current.Value ?? Unscoped).Solve(radius, possible, probes, Stopwatch.GetTimestamp() - started);

    private sealed class EmptyScope : IDisposable
    {
        internal static readonly EmptyScope Instance = new();
        public void Dispose() { }
    }

    private sealed class Scope : IDisposable
    {
        private readonly string _consumer;
        private readonly string? _operation;
        private readonly bool _ownsCurrent;
        private readonly bool _verbose = Environment.GetEnvironmentVariable("RW_TOOLPATHS_TRACE_INSET_FEASIBILITY") == "1";
        private long _solves, _possible, _impossible, _probes, _ticks, _cachePossible, _cacheImpossible;
        private int _possibleWitnesses, _impossibleWitnesses, _disposed;

        internal Scope(string consumer, string? operation, bool ownsCurrent)
            => (_consumer, _operation, _ownsCurrent) = (consumer, operation, ownsCurrent);

        internal void Cache(bool possible)
        {
            if (possible) Interlocked.Increment(ref _cachePossible);
            else Interlocked.Increment(ref _cacheImpossible);
        }

        internal void Solve(double radius, bool possible, int probes, long ticks)
        {
            long solves = Interlocked.Increment(ref _solves);
            if (possible) Interlocked.Increment(ref _possible);
            else Interlocked.Increment(ref _impossible);
            Interlocked.Add(ref _probes, probes);
            Interlocked.Add(ref _ticks, ticks);
            int witness = possible ? Interlocked.Increment(ref _possibleWitnesses)
                : Interlocked.Increment(ref _impossibleWitnesses);
            if (_verbose || witness <= 4)
                Console.WriteLine($"[PolygonOps.InsetFeasibility] consumer={_consumer} operation='{_operation}' "
                    + $"radius={radius / PathUtils.Scale:F4} possible={possible} probes={probes} "
                    + $"elapsedMs={ticks * 1000.0 / Stopwatch.Frequency:F2} trace={_verbose} witness={witness}");
            // Unscoped library consumers have no operation exit. Their summaries
            // are logarithmically bounded instead of growing with each query.
            if (!_ownsCurrent && solves >= 1024 && (solves & (solves - 1)) == 0) Summary();
        }

        private void Summary()
            => Console.WriteLine($"[PolygonOps.InsetFeasibility.Summary] consumer={_consumer} operation='{_operation}' "
                + $"solves={Interlocked.Read(ref _solves)} possible={Interlocked.Read(ref _possible)} "
                + $"impossible={Interlocked.Read(ref _impossible)} probes={Interlocked.Read(ref _probes)} "
                + $"queryMs={Interlocked.Read(ref _ticks) * 1000.0 / Stopwatch.Frequency:F2} "
                + $"cachePossible={Interlocked.Read(ref _cachePossible)} cacheImpossible={Interlocked.Read(ref _cacheImpossible)} "
                + $"trace={_verbose} witnessLimitPerResult=4 timings=sum-query-wall");

        public void Dispose()
        {
            if (Interlocked.Exchange(ref _disposed, 1) != 0) return;
            if (_ownsCurrent) Current.Value = null;
            Summary();
        }
    }
}
