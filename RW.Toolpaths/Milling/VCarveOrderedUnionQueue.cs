using System.Collections.Concurrent;
using System.Diagnostics;
using System.Runtime.ExceptionServices;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

// Retains original leaf batches and their publication order. A streaming
// consumer can read a fully drained window; final geometry publishes only
// after every worker joins. Failed or cancelled work publishes no geometry.
internal sealed class VCarveOrderedUnionQueue : IDisposable
{
    internal const long MaximumRetainedInputBytes = 32L * 1024 * 1024;
    internal sealed class Slot
    {
        internal Paths64? Source;
        internal Paths64? Result;
        internal ExceptionDispatchInfo? Failure;
        internal long RetainedBytes;
        internal static Slot Completed(Paths64 paths) => new() { Result = paths };
    }

    private readonly BlockingCollection<Slot> _queue;
    private readonly SemaphoreSlim _permits;
    private readonly ManualResetEventSlim _empty = new(true);
    private readonly Task[] _workers;
    private readonly Func<Paths64, Paths64> _union;
    private readonly CancellationToken _token;
    private readonly object _sync = new();
    private readonly long _maximumBytes;
    private readonly int _maximumPathsPerLeaf;
    private ExceptionDispatchInfo? _failure;
    private long _retainedBytes, _computeTicks, _waitTicks, _contours;
    private int _inFlight, _peakInFlight;
    private bool _complete, _disposed;
    internal int WorkerCount => _workers.Length;
    internal long Submitted { get; private set; }
    internal long LiteralFallbacks { get; private set; }
    internal long PeakRetainedInputBytes { get; private set; }
    internal int PeakInFlight => _peakInFlight;
    internal long CompletedContours => Interlocked.Read(ref _contours);
    internal bool OwnedWorkersJoined => _workers.All(worker => worker.IsCompleted);
    internal double ComputeMilliseconds => Interlocked.Read(ref _computeTicks) * 1000.0 / Stopwatch.Frequency;
    internal double WaitMilliseconds => Interlocked.Read(ref _waitTicks) * 1000.0 / Stopwatch.Frequency;

    internal VCarveOrderedUnionQueue(CancellationToken token, int workers,
        Func<Paths64, Paths64>? union = null, long maximumBytes = MaximumRetainedInputBytes, int maximumPathsPerLeaf=32)
    {
        if (workers < 1 || workers > GeometryParallelism.WorkerLimit)
            throw new ArgumentOutOfRangeException(nameof(workers));
        if (maximumBytes < 1 || maximumBytes > MaximumRetainedInputBytes)
            throw new ArgumentOutOfRangeException(nameof(maximumBytes));
        if(maximumPathsPerLeaf<1||maximumPathsPerLeaf>128)throw new ArgumentOutOfRangeException(nameof(maximumPathsPerLeaf));
        token.ThrowIfCancellationRequested();
        _maximumPathsPerLeaf=maximumPathsPerLeaf;
        _token = token; _maximumBytes = maximumBytes;
        _union = union ?? (paths => Clipper.Union(paths, FillRule.NonZero));
        _queue = new(workers);
        _permits = new SemaphoreSlim(workers, workers);
        _workers = new Task[workers];
        int created = 0;
        try
        {
            for (; created < workers; created++)
                _workers[created] = Task.Factory.StartNew(Worker, CancellationToken.None,
                    TaskCreationOptions.LongRunning, TaskScheduler.Default);
        }
        catch
        {
            _queue.CompleteAdding();
            Task.WaitAll(_workers.Take(created).ToArray());
            _queue.Dispose(); _permits.Dispose(); _empty.Dispose();
            throw;
        }
    }

    internal Slot Submit(Paths64 source)
    {
        CheckActive();
        if (source.Count > _maximumPathsPerLeaf) throw new ArgumentException($"Leaf union has at most {_maximumPathsPerLeaf} paths.", nameof(source));
        long bytes = checked(64 + 8L * source.Count);
        foreach (var path in source) bytes = checked(bytes + 64 + 16L * path.Capacity);
        if (bytes > _maximumBytes)
        {
            Drain(); LiteralFallbacks++;
            long started = Stopwatch.GetTimestamp();
            var result = _union(source);
            Interlocked.Add(ref _computeTicks, Stopwatch.GetTimestamp() - started);
            _token.ThrowIfCancellationRequested();
            return Slot.Completed(result);
        }
        while (true)
        {
            WaitPermit();
            lock (_sync)
            {
                if (bytes <= _maximumBytes - _retainedBytes)
                {
                    _retainedBytes += bytes;
                    PeakRetainedInputBytes = Math.Max(PeakRetainedInputBytes, _retainedBytes);
                    _inFlight++; _peakInFlight = Math.Max(_peakInFlight, _inFlight);
                    _empty.Reset();
                    break;
                }
            }
            _permits.Release();
            Drain();
        }
        Slot? slot = null;
        try
        {
            // Path objects are complete and never mutated after Flush. Detach
            // only the outer list before the producer clears/reuses pending.
            slot = new Slot { Source = new Paths64(source), RetainedBytes = bytes };
            _queue.Add(slot, _token); Submitted++;
            return slot;
        }
        catch
        {
            Release(bytes);
            throw;
        }
    }

    // ConstantRun's literal ClipperOffset consumes the same worker owner.
    // The producer acquires a permit within that shared budget.
    internal IDisposable EnterProducerGeometry()
    {
        CheckActive(); WaitPermit();
        try { CheckActive(); return new Permit(_permits); }
        catch { _permits.Release(); throw; }
    }

    internal void Drain()
    {
        long started = Stopwatch.GetTimestamp();
        _empty.Wait(_token);
        Interlocked.Add(ref _waitTicks, Stopwatch.GetTimestamp() - started);
        Volatile.Read(ref _failure)?.Throw(); _token.ThrowIfCancellationRequested();
    }

    internal void Complete()
    {
        CheckActive();
        _queue.CompleteAdding();
        Task.WaitAll(_workers);
        Volatile.Read(ref _failure)?.Throw(); _token.ThrowIfCancellationRequested();
        _complete = true;
    }

    internal Paths64 Read(Slot slot)
    {
        if (!_complete) throw new InvalidOperationException("Union output requires joined workers.");
        slot.Failure?.Throw();
        return slot.Result ?? throw new InvalidOperationException("Original union group was not completed.");
    }

    // A bounded streaming consumer can retire an entire drained window in
    // submission order. Complete/Dispose still joins every owned worker before
    // the consumer may publish its final geometry. No live slot is readable.
    internal Paths64 ReadDrained(Slot slot)
    {
        _token.ThrowIfCancellationRequested();Volatile.Read(ref _failure)?.Throw();
        lock(_sync)
            if(_inFlight!=0)throw new InvalidOperationException("Streaming union output requires a drained window.");
        slot.Failure?.Throw();
        return slot.Result??throw new InvalidOperationException("Original union group was not completed.");
    }

    private void WaitPermit()
    {
        long started = Stopwatch.GetTimestamp();
        _permits.Wait(_token);
        Interlocked.Add(ref _waitTicks, Stopwatch.GetTimestamp() - started);
    }
    private void CheckActive()
    {
        if (_disposed || _complete) throw new ObjectDisposedException(nameof(VCarveOrderedUnionQueue));
        Volatile.Read(ref _failure)?.Throw(); _token.ThrowIfCancellationRequested();
    }
    private void Worker()
    {
        using var geometryWorker = GeometryParallelism.EnterWorker();
        foreach (var slot in _queue.GetConsumingEnumerable())
        {
            try
            {
                _token.ThrowIfCancellationRequested();
                long started = Stopwatch.GetTimestamp();
                var result = _union(slot.Source!);
                Interlocked.Add(ref _computeTicks, Stopwatch.GetTimestamp() - started);
                _token.ThrowIfCancellationRequested();
                slot.Result = result;
                Interlocked.Add(ref _contours, result.Count);
            }
            catch (Exception error)
            {
                slot.Failure = ExceptionDispatchInfo.Capture(error);
                Interlocked.CompareExchange(ref _failure, slot.Failure, null);
            }
            finally
            {
                slot.Source = null;
                Release(slot.RetainedBytes);
            }
        }
    }
    private void Release(long bytes)
    {
        lock (_sync)
        {
            _retainedBytes -= bytes;
            if (--_inFlight == 0) _empty.Set();
        }
        _permits.Release();
    }
    private sealed class Permit : IDisposable
    {
        private SemaphoreSlim? _semaphore;
        internal Permit(SemaphoreSlim semaphore) => _semaphore = semaphore;
        public void Dispose() => Interlocked.Exchange(ref _semaphore, null)?.Release();
    }
    public void Dispose()
    {
        if (_disposed) return;
        _queue.CompleteAdding();
        Task.WaitAll(_workers); // Drain even on producer failure/cancellation.
        _disposed = true;
        _queue.Dispose(); _permits.Dispose(); _empty.Dispose();
        Console.WriteLine($"[VCarve.PhysicalUnionQueue] event=closed workers={WorkerCount} workersJoined={OwnedWorkersJoined} " +
            $"submitted={Submitted} literalFallbacks={LiteralFallbacks} remainingInFlight={_inFlight} remainingInputBytes={_retainedBytes} " +
            $"peakInFlight={PeakInFlight} peakInputBytes={PeakRetainedInputBytes} maximumInputBytes={_maximumBytes} maximumPathsPerLeaf={_maximumPathsPerLeaf} " +
            $"computeMs={ComputeMilliseconds:F3} waitMs={WaitMilliseconds:F3} complete={_complete} " +
            $"failure={Volatile.Read(ref _failure)?.SourceException.GetType().Name ?? "none"} " +
            $"computeScope=sum-worker-inclusive outputPublication=after-join-and-token-check");
    }
}
