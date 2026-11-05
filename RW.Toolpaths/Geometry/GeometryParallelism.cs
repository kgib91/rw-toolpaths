namespace RW.Toolpaths.Geometry;

/// <summary>Keep independent component workers from multiplying the existing
/// native offset/union worker budget. Scope flows into nested Parallel calls.</summary>
internal static class GeometryParallelism
{
    private static readonly AsyncLocal<int> Limit = new();
    // Dense exact geometry can use additional cores on larger workstations.
    // Keep the established four-worker ceiling on smaller machines and reserve
    // at least half the logical processors once the budget grows beyond four.
    internal static int HardwareWorkerLimit => Math.Min(Environment.ProcessorCount,
        Math.Clamp(Environment.ProcessorCount / 2, 4, 12));
    internal static int WorkerLimit => Limit.Value > 0 ? Limit.Value : HardwareWorkerLimit;
    // Stock-fit workers retain large private proof memos. Admit at most one
    // worker per two GiB of the runtime's memory limit, leaving a logical-core
    // quarter for the desktop once the budget grows beyond four workers.
    internal static int StockFitWorkerLimit => Limit.Value > 0 ? Limit.Value : Math.Min(
        Math.Min(Environment.ProcessorCount,Math.Clamp(Environment.ProcessorCount*3/4,4,16)),
        (int)Math.Clamp(GC.GetGCMemoryInfo().TotalAvailableMemoryBytes/(2L<<30),1,16));
    internal static IDisposable EnterWorker()
    {
        int previous = Limit.Value;
        Limit.Value = 1;
        return new WorkerScope(previous);
    }

    private sealed class WorkerScope : IDisposable
    {
        private readonly int _previous;
        private bool _disposed;
        internal WorkerScope(int previous) => _previous = previous;
        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            Limit.Value = _previous;
        }
    }
}
