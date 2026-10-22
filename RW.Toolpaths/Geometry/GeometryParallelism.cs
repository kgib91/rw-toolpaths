namespace RW.Toolpaths.Geometry;

/// <summary>Keep independent component workers from multiplying the existing
/// native offset/union worker budget. Scope flows into nested Parallel calls.</summary>
internal static class GeometryParallelism
{
    private static readonly AsyncLocal<int> Limit = new();
    internal static int WorkerLimit => Limit.Value > 0 ? Limit.Value : Math.Min(4, Environment.ProcessorCount);
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
