using Clipper2Lib;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace RW.Toolpaths.Milling.Acceleration;

// Native-grid coordinates are the caller's original directed finite edges.
// No sample result, including exact integer winding, is cutting authority.
[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuBoundarySegment
{
    internal readonly long Ax, Ay, Bx, By;
    internal GpuBoundarySegment(long ax, long ay, long bx, long by)
    { Ax = ax; Ay = ay; Bx = bx; By = by; }
    internal GpuBoundarySegment(Point64 a, Point64 b) : this(a.X, a.Y, b.X, b.Y) { }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuBoundaryNode
{
    internal readonly long X0, Y0, X1, Y1;
    internal readonly int First, Count, Left, Right;
    internal GpuBoundaryNode(long x0, long y0, long x1, long y1, int first, int count, int left, int right)
    { X0 = x0; Y0 = y0; X1 = x1; Y1 = y1; First = first; Count = count; Left = left; Right = right; }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuBoundaryQuery
{
    internal readonly long X, Y;
    internal GpuBoundaryQuery(long x, long y) { X = x; Y = y; }
    internal GpuBoundaryQuery(Point64 point) : this(point.X, point.Y) { }
}

internal enum GpuBoundaryRefusal
{
    None = 0, NodeBudget = 1, SegmentBudget = 2, StackBudget = 3,
    InvalidNativeInput = 4, InputDomain = 5, Runtime = 6, Disposed = 7, Cancelled = 8
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuBoundarySampleNomination
{
    // SquaredDistance is in native grid units squared, not millimetres.
    internal readonly double Parameter, SquaredDistance;
    internal readonly int SegmentOrdinal, Winding, Status, RefusalReason;
    internal GpuBoundarySampleNomination(int ordinal, double parameter, double squaredDistance,
        int winding, int status, GpuBoundaryRefusal refusal)
    { Parameter = parameter; SquaredDistance = squaredDistance; SegmentOrdinal = ordinal;
      Winding = winding; Status = status; RefusalReason = (int)refusal; }
    internal bool HasNomination => Status == 1 && RefusalReason == 0;
    internal bool NonZeroWindingNomination => HasNomination && Winding != 0;
    internal static GpuBoundarySampleNomination Refused(GpuBoundaryRefusal reason)
        => new(-1, double.NaN, double.PositiveInfinity, 0, 2, reason);
}

internal readonly record struct GpuBoundaryWorkLimits(int MaximumNodeVisits, int MaximumSegmentVisits)
{
    internal static GpuBoundaryWorkLimits Default => new(4096, 16_384);
}

internal sealed partial class VCarveGpuDiscWitnessContext
{
    /// <summary>Optional finite-boundary sample nomination using the existing
    /// CUDA provider. Original integer inputs are snapshotted and validated;
    /// bounded-work refusals and all successful samples have no proof authority.</summary>
    internal sealed class BoundarySampleContext : IDisposable
    {
        internal const long ExclusiveCoordinateLimit = 1L << 29;
        internal const int MaximumSegments = 1_000_000;
        internal const int MaximumNodes = 2_000_000;
        internal const int MaximumQueriesPerCall = 1_000_000;
        internal const long Fp32RelativeCoordinateLimit = 1L << 24;
        private readonly object _boundaryGate = new();
        private readonly GpuBoundarySegment[] _boundarySegments;
        private readonly GpuBoundaryNode[] _boundaryNodes;
        private readonly GpuBoundaryWorkLimits _limits;
        private readonly Action<string>? _boundaryDiagnostic;
        private readonly GpuBoundaryQuery[] _boundaryQueries = new GpuBoundaryQuery[MaximumBatchSize];
        private readonly GpuBoundarySampleNomination[] _boundaryNominations = new GpuBoundarySampleNomination[MaximumBatchSize];
        private IntPtr _boundaryContext, _boundaryModule, _boundaryFunction, _boundaryFp32Function;
        private ulong _boundaryDeviceSegments, _boundaryDeviceNodes, _boundaryDeviceQueries, _boundaryDeviceNominations;
        private bool _boundaryDisposed, _boundaryFailed;
        internal string DeviceName { get; private set; } = "";
        internal string? FailureReason { get; private set; }
        internal long Batches { get; private set; }
        internal long QueryCount { get; private set; }
        internal long CompletedNominations { get; private set; }
        internal long NodeBudgetRefusals { get; private set; }
        internal long SegmentBudgetRefusals { get; private set; }
        internal long OtherTraversalRefusals { get; private set; }
        internal long UploadedBytes { get; private set; }
        internal long DiagnosticObserverFailures { get; private set; }
        internal double PreparationMilliseconds { get; private set; }
        internal double CompilationMilliseconds { get; private set; }
        internal double KernelMilliseconds { get; private set; }
        internal double TransferMilliseconds { get; private set; }
        internal bool KernelCacheHit { get; private set; }
        internal bool RequestedFp32Nomination { get; private init; }
        internal bool SupportsFp32SourceDomain { get; }
        internal long Fp32Queries { get; private set; }
        internal long Fp64FallbackQueries { get; private set; }
        internal string NumericMode => RequestedFp32Nomination && SupportsFp32SourceDomain
            ? "fp32-nearest-int64-winding-with-fp64-fallback" : "fp64-nearest-int64-winding";

        private BoundarySampleContext(GpuBoundarySegment[] segments, GpuBoundaryNode[] nodes,
            GpuBoundaryWorkLimits limits, Action<string>? diagnostic)
        {
            _boundarySegments = segments; _boundaryNodes = nodes; _limits = limits; _boundaryDiagnostic = diagnostic;
            var root = nodes[0]; SupportsFp32SourceDomain = root.X1 - root.X0 <= Fp32RelativeCoordinateLimit
                && root.Y1 - root.Y0 <= Fp32RelativeCoordinateLimit;
        }

        internal static bool TryCreate(GpuBoundarySegment[] segments, GpuBoundaryNode[] nodes,
            CancellationToken token, Action<string>? diagnostic, out BoundarySampleContext? context, out string reason)
            => TryCreate(segments, nodes, GpuBoundaryWorkLimits.Default, token, diagnostic, out context, out reason);

        internal static bool TryCreate(GpuBoundarySegment[] segments, GpuBoundaryNode[] nodes,
            GpuBoundaryWorkLimits limits, CancellationToken token, Action<string>? diagnostic,
            out BoundarySampleContext? context, out string reason)
            => TryCreate(segments, nodes, limits,
                VCarveGenerationPolicy.Capture().GpuBoundaryFp32,
                token, diagnostic, out context, out reason);

        // Explicit mode is useful for isolated controls; the normal factory's
        // separately opt-in environment flag remains false by default.
        internal static bool TryCreate(GpuBoundarySegment[] segments, GpuBoundaryNode[] nodes,
            GpuBoundaryWorkLimits limits, bool useFp32Nomination, CancellationToken token, Action<string>? diagnostic,
            out BoundarySampleContext? context, out string reason)
        {
            context = null; reason = ""; token.ThrowIfCancellationRequested();
            if (segments is null || nodes is null || segments.Length is < 1 or > MaximumSegments
                || nodes.Length is < 1 or > MaximumNodes) { reason = "boundary-input-count"; return false; }
            var segmentSnapshot = (GpuBoundarySegment[])segments.Clone();
            var nodeSnapshot = (GpuBoundaryNode[])nodes.Clone();
            if (!ValidateSnapshot(segmentSnapshot, nodeSnapshot, limits, token, out reason)) return false;
            if (!OperatingSystem.IsWindows() || !Environment.Is64BitProcess) { reason = "unsupported-platform"; return false; }
            var candidate = new BoundarySampleContext(segmentSnapshot, nodeSnapshot, limits, diagnostic)
                { RequestedFp32Nomination = useFp32Nomination };
            long started = Stopwatch.GetTimestamp(); IntPtr previous = IntPtr.Zero; bool capturedPrevious = false;
            try
            {
                Check(Driver.cuInit(0), "boundary-driver-init");
                Check(Driver.cuCtxGetCurrent(out previous), "boundary-context-read"); capturedPrevious = true;
                Check(Driver.cuDeviceGetCount(out int devices), "boundary-device-count");
                if (devices == 0) throw new GpuFailure("no-cuda-device");
                Check(Driver.cuDeviceGet(out int device, 0), "boundary-device-select");
                Check(Driver.cuDeviceComputeCapability(out int major, out int minor, device), "boundary-device-capability");
                if (major < 3) throw new GpuFailure("unsupported-double-precision-device");
                var name = new byte[256]; Check(Driver.cuDeviceGetName(name, name.Length, device), "boundary-device-name");
                candidate.DeviceName = Encoding.UTF8.GetString(name).TrimEnd('\0');
                Check(Driver.cuCtxCreate_v2(out candidate._boundaryContext, 0, device), "boundary-context-create");
                long compilationStarted = Stopwatch.GetTimestamp();
                // The existing embedded resource/cache/provider is shared.
                // No profile, disc/box/triangle, or machining API is changed.
                byte[] ptx = CompileKernel(major, minor, false, false, token, out bool cacheHit);
                candidate.CompilationMilliseconds = Stopwatch.GetElapsedTime(compilationStarted).TotalMilliseconds;
                candidate.KernelCacheHit = cacheHit;
                Check(Driver.cuModuleLoadData(out candidate._boundaryModule, ptx), "boundary-module-load");
                Check(Driver.cuModuleGetFunction(out candidate._boundaryFunction, candidate._boundaryModule,
                    "nominate_boundary_samples"), "boundary-kernel-find");
                if (candidate.RequestedFp32Nomination && candidate.SupportsFp32SourceDomain)
                    Check(Driver.cuModuleGetFunction(out candidate._boundaryFp32Function, candidate._boundaryModule,
                        "nominate_boundary_samples_fp32"), "boundary-fp32-kernel-find");
                candidate._boundaryDeviceSegments = candidate.UploadSnapshot(segmentSnapshot, token);
                candidate._boundaryDeviceNodes = candidate.UploadSnapshot(nodeSnapshot, token);
                Check(Driver.cuMemAlloc_v2(out candidate._boundaryDeviceQueries, Bytes<GpuBoundaryQuery>(MaximumBatchSize)), "boundary-queries-allocate");
                Check(Driver.cuMemAlloc_v2(out candidate._boundaryDeviceNominations, Bytes<GpuBoundarySampleNomination>(MaximumBatchSize)), "boundary-results-allocate");
                token.ThrowIfCancellationRequested();
                candidate.PreparationMilliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                candidate.Log("ready", $"segments={segments.Length} nodes={nodes.Length} uploadedBytes={candidate.UploadedBytes} "
                    + $"nodeBudget={limits.MaximumNodeVisits} segmentBudget={limits.MaximumSegmentVisits} batchCapacity={MaximumBatchSize} "
                    + $"fp32Requested={candidate.RequestedFp32Nomination} fp32SourceSupported={candidate.SupportsFp32SourceDomain} "
                    + $"fp32RelativeCoordinateLimit={Fp32RelativeCoordinateLimit} "
                    + $"preparationMs={candidate.PreparationMilliseconds:F2} compilationMs={candidate.CompilationMilliseconds:F2} kernelCacheHit={cacheHit}");
                token.ThrowIfCancellationRequested();
                context = candidate; return true;
            }
            catch (OperationCanceledException) { candidate.Dispose(); throw; }
            catch (Exception exception) when (IsRecoverable(exception))
            {
                reason = Failure(exception); candidate.FailureReason = reason; candidate._boundaryFailed = true;
                candidate.Log("refused", $"reason='{reason}'"); candidate.Dispose(); return false;
            }
            finally { if (capturedPrevious) Driver.cuCtxSetCurrent(previous); }
        }

        internal static bool ValidateSnapshot(GpuBoundarySegment[] segments, GpuBoundaryNode[] nodes,
            GpuBoundaryWorkLimits limits, CancellationToken token, out string reason)
        {
            reason = ""; token.ThrowIfCancellationRequested();
            if (segments is null || nodes is null || segments.Length is < 1 or > MaximumSegments
                || nodes.Length is < 1 or > MaximumNodes) { reason = "boundary-input-count"; return false; }
            if (limits.MaximumNodeVisits is < 1 or > 4096 || limits.MaximumSegmentVisits is < 1 or > 65_536)
            { reason = "boundary-work-limits"; return false; }
            for (int i = 0; i < segments.Length; i++)
            {
                if ((i & 255) == 0) token.ThrowIfCancellationRequested();
                var s = segments[i];
                if (!InDomain(s.Ax) || !InDomain(s.Ay) || !InDomain(s.Bx) || !InDomain(s.By))
                { reason = "boundary-segment-domain"; return false; }
            }
            var visitedNodes = new bool[nodes.Length]; var visitedSegments = new bool[segments.Length];
            var pending = new Stack<(int Ordinal, int Depth)>(); pending.Push((0, 1));
            int coveredNodes = 0, coveredSegments = 0;
            while (pending.Count != 0)
            {
                if ((coveredNodes & 255) == 0) token.ThrowIfCancellationRequested();
                var item = pending.Pop(); int ordinal = item.Ordinal;
                if (ordinal < 0 || ordinal >= nodes.Length || visitedNodes[ordinal])
                { reason = "boundary-bvh-cycle-or-shared-node"; return false; }
                if (item.Depth > 64) { reason = "boundary-bvh-depth"; return false; }
                visitedNodes[ordinal] = true; coveredNodes++; var node = nodes[ordinal];
                if (!InDomain(node.X0) || !InDomain(node.Y0) || !InDomain(node.X1) || !InDomain(node.Y1)
                    || node.X0 > node.X1 || node.Y0 > node.Y1)
                { reason = "boundary-bvh-bounds"; return false; }
                if (node.Left == -1 && node.Right == -1)
                {
                    if (node.Count <= 0 || node.First < 0 || node.First > segments.Length - node.Count)
                    { reason = "boundary-bvh-leaf-range"; return false; }
                    for (int i = node.First; i < node.First + node.Count; i++)
                    {
                        if ((i & 255) == 0) token.ThrowIfCancellationRequested();
                        var s = segments[i];
                        if (visitedSegments[i]) { reason = "boundary-bvh-duplicate-segment"; return false; }
                        if (!Encloses(node, s.Ax, s.Ay) || !Encloses(node, s.Bx, s.By))
                        { reason = "boundary-bvh-excludes-segment"; return false; }
                        visitedSegments[i] = true; coveredSegments++;
                    }
                }
                else
                {
                    if (node.Count != 0 || node.First != 0 || node.Left < 0 || node.Right < 0
                        || node.Left >= nodes.Length || node.Right >= nodes.Length || node.Left == node.Right)
                    { reason = "boundary-bvh-branch-indices"; return false; }
                    for (int childIndex = 0; childIndex < 2; childIndex++)
                    {
                        int child = childIndex == 0 ? node.Left : node.Right;
                        var n = nodes[child];
                        if (!Encloses(node, n.X0, n.Y0) || !Encloses(node, n.X1, n.Y1))
                        { reason = "boundary-bvh-excludes-child"; return false; }
                        pending.Push((child, item.Depth + 1));
                    }
                }
            }
            if (coveredNodes != nodes.Length || coveredSegments != segments.Length)
            { reason = "boundary-bvh-incomplete-coverage"; return false; }
            return true;
        }

        private static bool InDomain(long value) => value > -ExclusiveCoordinateLimit && value < ExclusiveCoordinateLimit;
        private static bool Encloses(GpuBoundaryNode n, long x, long y) => n.X0 <= x && x <= n.X1 && n.Y0 <= y && y <= n.Y1;

        internal bool QuerySupportsFp32(GpuBoundaryQuery query)
        {
            var root = _boundaryNodes[0]; const long limit = Fp32RelativeCoordinateLimit;
            return SupportsFp32SourceDomain && InDomain(query.X) && InDomain(query.Y)
                && query.X - root.X0 >= -limit && query.X - root.X0 <= limit
                && query.X - root.X1 >= -limit && query.X - root.X1 <= limit
                && query.Y - root.Y0 >= -limit && query.Y - root.Y0 <= limit
                && query.Y - root.Y1 >= -limit && query.Y - root.Y1 <= limit;
        }

        internal bool TryGetOriginalSegment(int ordinal, out GpuBoundarySegment segment)
        {
            lock (_boundaryGate)
            {
                segment = default;
                if (_boundaryDisposed || ordinal < 0 || ordinal >= _boundarySegments.Length) return false;
                segment = _boundarySegments[ordinal]; return true;
            }
        }

        internal bool TryNominate(GpuBoundaryQuery[] queries, GpuBoundarySampleNomination[] nominations,
            int count, CancellationToken token, out string reason)
        {
            ArgumentNullException.ThrowIfNull(queries); ArgumentNullException.ThrowIfNull(nominations);
            if (count < 0 || count > queries.Length || count > nominations.Length) throw new ArgumentOutOfRangeException(nameof(count));
            RefuseAll(nominations, count, token.IsCancellationRequested ? GpuBoundaryRefusal.Cancelled : GpuBoundaryRefusal.InputDomain);
            token.ThrowIfCancellationRequested(); reason = "";
            if (count > MaximumQueriesPerCall) { reason = "boundary-query-count"; return false; }
            lock (_boundaryGate)
            {
                if (_boundaryDisposed || _boundaryFailed)
                {
                    reason = _boundaryDisposed ? "context-disposed" : FailureReason ?? "context-failed";
                    RefuseAll(nominations, count, _boundaryDisposed ? GpuBoundaryRefusal.Disposed : GpuBoundaryRefusal.Runtime); return false;
                }
                IntPtr previous = IntPtr.Zero; bool capturedPrevious = false;
                try
                {
                    Check(Driver.cuCtxGetCurrent(out previous), "boundary-context-read"); capturedPrevious = true;
                    Check(Driver.cuCtxSetCurrent(_boundaryContext), "boundary-context-activate");
                    for (int first = 0; first < count; first += MaximumBatchSize)
                    {
                        token.ThrowIfCancellationRequested(); int batchCount = Math.Min(MaximumBatchSize, count - first);
                        Array.Copy(queries, first, _boundaryQueries, 0, batchCount);
                        for (int i = 0; i < batchCount; i++)
                            if (!InDomain(_boundaryQueries[i].X) || !InDomain(_boundaryQueries[i].Y))
                            { reason = "boundary-query-domain"; RefuseAll(nominations, count, GpuBoundaryRefusal.InputDomain); return false; }
                        bool useFp32 = RequestedFp32Nomination && SupportsFp32SourceDomain;
                        if (useFp32)
                            for (int i = 0; i < batchCount; i++)
                                if (!QuerySupportsFp32(_boundaryQueries[i])) { useFp32 = false; break; }
                        long started = Stopwatch.GetTimestamp(); CopyToDevice(_boundaryDeviceQueries, _boundaryQueries, batchCount);
                        TransferMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                        started = Stopwatch.GetTimestamp(); LaunchBoundary(batchCount, useFp32);
                        Check(Driver.cuCtxSynchronize(), "boundary-kernel-synchronize");
                        KernelMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                        token.ThrowIfCancellationRequested(); started = Stopwatch.GetTimestamp();
                        CopyFromDevice(_boundaryNominations, _boundaryDeviceNominations, batchCount);
                        TransferMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                        for (int i = 0; i < batchCount; i++)
                        {
                            var n = _boundaryNominations[i];
                            if (n.HasNomination)
                            {
                                if (n.SegmentOrdinal < 0 || n.SegmentOrdinal >= _boundarySegments.Length
                                    || !double.IsFinite(n.Parameter) || n.Parameter < 0 || n.Parameter > 1
                                    || !double.IsFinite(n.SquaredDistance) || n.SquaredDistance < 0
                                    || n.Winding < -_boundarySegments.Length || n.Winding > _boundarySegments.Length)
                                    throw new GpuFailure("invalid-boundary-result");
                                CompletedNominations++;
                            }
                            else if (n.Status == 2 && n.SegmentOrdinal == -1 && n.Winding == 0
                                && double.IsNaN(n.Parameter) && double.IsPositiveInfinity(n.SquaredDistance)
                                && n.RefusalReason is >= 1 and <= 4)
                            {
                                if (n.RefusalReason == 1) NodeBudgetRefusals++;
                                else if (n.RefusalReason == 2) SegmentBudgetRefusals++;
                                else OtherTraversalRefusals++;
                            }
                            else throw new GpuFailure("invalid-boundary-result-status");
                        }
                        Array.Copy(_boundaryNominations, 0, nominations, first, batchCount);
                        Batches++; QueryCount += batchCount;
                        if (useFp32) Fp32Queries += batchCount;
                        else if (RequestedFp32Nomination) Fp64FallbackQueries += batchCount;
                    }
                    token.ThrowIfCancellationRequested();
                    Log("sampled", $"batches={Batches} queries={QueryCount} completed={CompletedNominations} "
                        + $"nodeBudgetRefusals={NodeBudgetRefusals} segmentBudgetRefusals={SegmentBudgetRefusals} "
                        + $"otherTraversalRefusals={OtherTraversalRefusals} kernelMs={KernelMilliseconds:F2} transferMs={TransferMilliseconds:F2}");
                    token.ThrowIfCancellationRequested();
                    return true;
                }
                catch (OperationCanceledException) { RefuseAll(nominations, count, GpuBoundaryRefusal.Cancelled); throw; }
                catch (Exception exception) when (IsRecoverable(exception))
                {
                    reason = Failure(exception); FailureReason = reason; _boundaryFailed = true;
                    RefuseAll(nominations, count, GpuBoundaryRefusal.Runtime); Log("refused", $"reason='{reason}'"); return false;
                }
                finally { if (capturedPrevious) Driver.cuCtxSetCurrent(previous); }
            }
        }

        private static void RefuseAll(GpuBoundarySampleNomination[] nominations, int count, GpuBoundaryRefusal reason)
            => Array.Fill(nominations, GpuBoundarySampleNomination.Refused(reason), 0, count);

        private ulong UploadSnapshot<T>(T[] values, CancellationToken token) where T : struct
        {
            token.ThrowIfCancellationRequested(); Check(Driver.cuMemAlloc_v2(out ulong pointer, Bytes<T>(values.Length)), "boundary-snapshot-allocate");
            try
            {
                long started = Stopwatch.GetTimestamp(); CopyToDevice(pointer, values, values.Length);
                TransferMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                UploadedBytes += checked((long)Bytes<T>(values.Length)); token.ThrowIfCancellationRequested(); return pointer;
            }
            catch { Driver.cuMemFree_v2(pointer); throw; }
        }

        private void LaunchBoundary(int count, bool useFp32)
        {
            ulong[] values = { _boundaryDeviceSegments, _boundaryDeviceNodes, (ulong)_boundarySegments.Length,
                (ulong)_boundaryNodes.Length, _boundaryDeviceQueries, _boundaryDeviceNominations, (ulong)count,
                (ulong)_limits.MaximumNodeVisits, (ulong)_limits.MaximumSegmentVisits };
            GCHandle valuesPin = GCHandle.Alloc(values, GCHandleType.Pinned), addressesPin = default;
            try
            {
                var addresses = new IntPtr[values.Length];
                for (int i = 0; i < addresses.Length; i++) addresses[i] = IntPtr.Add(valuesPin.AddrOfPinnedObject(), i * sizeof(ulong));
                addressesPin = GCHandle.Alloc(addresses, GCHandleType.Pinned);
                Check(Driver.cuLaunchKernel(useFp32 ? _boundaryFp32Function : _boundaryFunction, (uint)((count + 127) / 128), 1, 1, 128, 1, 1,
                    0, IntPtr.Zero, addressesPin.AddrOfPinnedObject(), IntPtr.Zero), "boundary-kernel-launch");
            }
            finally { if (addressesPin.IsAllocated) addressesPin.Free(); valuesPin.Free(); }
        }

        private void Log(string eventName, string detail)
        {
            try { _boundaryDiagnostic?.Invoke($"stage=boundary-gpu-sample event={eventName} device='{DeviceName}' {detail} "
                + $"numericMode={NumericMode} fp32Requested={RequestedFp32Nomination} fp32SourceSupported={SupportsFp32SourceDomain} "
                + $"fp32Queries={Fp32Queries} fp64FallbackQueries={Fp64FallbackQueries} coordinateUnits=native-grid authority=none nominationOnly=True"); }
            catch { DiagnosticObserverFailures++; }
        }

        public void Dispose()
        {
            lock (_boundaryGate)
            {
                if (_boundaryDisposed) return; _boundaryDisposed = true;
                if (_boundaryContext != IntPtr.Zero)
                {
                    IntPtr previous = IntPtr.Zero; Driver.cuCtxGetCurrent(out previous); Driver.cuCtxSetCurrent(_boundaryContext);
                    foreach (ulong pointer in new[] { _boundaryDeviceNominations, _boundaryDeviceQueries, _boundaryDeviceNodes, _boundaryDeviceSegments })
                        if (pointer != 0) Driver.cuMemFree_v2(pointer);
                    if (_boundaryModule != IntPtr.Zero) Driver.cuModuleUnload(_boundaryModule);
                    Driver.cuCtxDestroy_v2(_boundaryContext);
                    Driver.cuCtxSetCurrent(previous == _boundaryContext ? IntPtr.Zero : previous); _boundaryContext = IntPtr.Zero;
                }
                Log("disposed", $"batches={Batches} queries={QueryCount} completed={CompletedNominations} "
                    + $"nodeBudgetRefusals={NodeBudgetRefusals} segmentBudgetRefusals={SegmentBudgetRefusals} otherTraversalRefusals={OtherTraversalRefusals} "
                    + $"kernelMs={KernelMilliseconds:F2} transferMs={TransferMilliseconds:F2} failed={_boundaryFailed} reason='{FailureReason}'");
            }
        }
    }
}
