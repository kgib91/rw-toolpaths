using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace RW.Toolpaths.Milling.Acceleration;

// These layouts are shared with the owned CUDA source. All geometry is an
// immutable snapshot in actual commanded coordinates, not rounded nomination
// geometry. Nomination status 0/2 has no negative-proof authority.
[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuDiscSegment
{
    internal readonly double Ax, Ay, Az, Bx, By, Bz, MaximumRadius;
    internal readonly int SourcePathIndex;
    internal GpuDiscSegment(double ax, double ay, double az, double bx, double by, double bz,
        double maximumRadius, int sourcePathIndex)
    { Ax = ax; Ay = ay; Az = az; Bx = bx; By = by; Bz = bz; MaximumRadius = maximumRadius; SourcePathIndex = sourcePathIndex; }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuDiscNode
{
    internal readonly double X0, Y0, X1, Y1, MaximumRadius;
    internal readonly int First, Count, Left, Right;
    internal GpuDiscNode(double x0, double y0, double x1, double y1, double maximumRadius,
        int first, int count, int left, int right)
    { X0 = x0; Y0 = y0; X1 = x1; Y1 = y1; MaximumRadius = maximumRadius; First = first; Count = count; Left = left; Right = right; }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuDiscQuery
{
    internal readonly double X, Y, Desired;
    internal GpuDiscQuery(double x, double y, double desired) { X = x; Y = y; Desired = desired; }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct GpuDiscNomination
{
    // Parameter first keeps both the native and managed result at sixteen bytes.
    internal readonly double Parameter;
    internal readonly int SegmentOrdinal, Status;
    internal GpuDiscNomination(int segmentOrdinal, double parameter, int status)
    { Parameter = parameter; SegmentOrdinal = segmentOrdinal; Status = status; }
    internal bool HasWitness => Status == 1;
}

/// <summary>Optional owned CUDA positive-witness nomination. The literal CPU
/// physical predicate remains the authority for every accepted pose. Driver,
/// compilation, memory and traversal failures are explicit fallback reasons.</summary>
internal sealed partial class VCarveGpuDiscWitnessContext : IDisposable
{
    internal const int MaximumBatchSize = 4096;
    private const int MaximumTraversalNodes = 4096;
    private const string ResourceName = "RW.Toolpaths.Milling.Acceleration.disc_witness.cu";
    private static readonly object KernelCacheGate = new();
    private static readonly Dictionary<string, byte[]> KernelCodeCache = new();
    private readonly object _gate = new();
    private readonly GpuDiscSegment[] _segments;
    private readonly VCarveCutterProfile _profile;
    private readonly GpuConvexBoxWitnessValidator.PreparedProfile _boxValidationProfile;
    private readonly Action<string>? _diagnostic;
    private readonly int _nodeCount, _sectionCount;
    private IntPtr _context, _module, _function, _boxFunction, _fp32Module, _fp32Function, _fp32BoxFunction;
    private IntPtr _triangleFunction, _fp32TriangleFunction;
    private ulong _deviceSegments, _deviceNodes, _deviceSections, _deviceQueries, _deviceNominations;
    private ulong _deviceBoxQueries, _deviceBoxNominations;
    private ulong _deviceTriangleQueries, _deviceTriangleNominations;
    private bool _disposed, _failed;
    private readonly GpuDiscQuery[] _queryBuffer = new GpuDiscQuery[MaximumBatchSize];
    private readonly GpuDiscNomination[] _nominationBuffer = new GpuDiscNomination[MaximumBatchSize];
    private readonly GpuBoxQuery[] _boxQueryBuffer = new GpuBoxQuery[MaximumBatchSize];
    private readonly GpuBoxNomination[] _boxNominationBuffer = new GpuBoxNomination[MaximumBatchSize];
    private readonly GpuTriangleQuery[] _triangleQueryBuffer = new GpuTriangleQuery[MaximumBatchSize];
    private readonly GpuTriangleNomination[] _triangleNominationBuffer = new GpuTriangleNomination[MaximumBatchSize];
    private long _lastDiagnostic = Stopwatch.GetTimestamp();
    internal string DeviceName { get; private set; } = "";
    internal long Batches { get; private set; }
    internal long QueryCount { get; private set; }
    internal long PositiveNominations { get; private set; }
    internal long NominationMisses { get; private set; }
    internal long TraversalRefusals { get; private set; }
    internal double MaximumBatchKernelMilliseconds { get; private set; }
    internal double KernelMilliseconds { get; private set; }
    internal double BoxKernelMilliseconds { get; private set; }
    internal long BoxQueryCount { get; private set; }
    internal double TriangleKernelMilliseconds { get; private set; }
    internal long TriangleQueryCount { get; private set; }
    internal double TransferMilliseconds { get; private set; }
    internal double PreparationMilliseconds { get; private set; }
    internal double CompilationMilliseconds { get; private set; }
    internal bool KernelCacheHit { get; private set; }
    internal bool UseGoldenReference { get; private set; }
    internal bool SupportsConvexBoxWitnesses => _boxValidationProfile.Supported;
    internal bool SupportsConvexTriangleWitnesses => _boxValidationProfile.Supported;
    internal bool Fp32Requested { get; private set; }
    internal bool Fp32Enabled { get; private set; }
    internal string Fp32RefusalReason { get; private set; } = "";
    internal long Fp32Queries { get; private set; }
    internal long Fp32Batches { get; private set; }
    internal long Fp32RefusedQueries { get; private set; }
    internal long Fp64FallbackBatches { get; private set; }
    internal long UploadedBytes { get; private set; }
    internal string? FailureReason { get; private set; }

    private VCarveGpuDiscWitnessContext(GpuDiscSegment[] segments, GpuDiscNode[] nodes,
        VCarveCutterProfile profile, Action<string>? diagnostic)
    {
        _segments = (GpuDiscSegment[])segments.Clone(); _profile = profile; _diagnostic = diagnostic;
        _boxValidationProfile = GpuConvexBoxWitnessValidator.Prepare(profile);
        _nodeCount = nodes.Length; _sectionCount = profile.Sections.Count;
    }

    internal static bool TryCreate(GpuDiscSegment[] segments, GpuDiscNode[] nodes,
        VCarveCutterProfile profile, CancellationToken cancellationToken, Action<string>? diagnostic,
        out VCarveGpuDiscWitnessContext? context, out string reason)
    {
        context = null; reason = "";
        cancellationToken.ThrowIfCancellationRequested();
        if (!ValidateGeometry(segments, nodes, profile, out reason)) return false;
        if (!OperatingSystem.IsWindows() || !Environment.Is64BitProcess) { reason = "unsupported-platform"; return false; }
        var candidate = new VCarveGpuDiscWitnessContext(segments, nodes, profile, diagnostic);
        long started = Stopwatch.GetTimestamp();
        IntPtr previous = IntPtr.Zero;
        bool capturedPrevious = false;
        try
        {
            Check(Driver.cuInit(0), "driver-init");
            Check(Driver.cuCtxGetCurrent(out previous), "context-read"); capturedPrevious = true;
            Check(Driver.cuDeviceGetCount(out int devices), "device-count");
            if (devices == 0) throw new GpuFailure("no-cuda-device");
            Check(Driver.cuDeviceGet(out int device, 0), "device-select");
            Check(Driver.cuDeviceComputeCapability(out int major, out int minor, device), "device-capability");
            if (major < 3) throw new GpuFailure("unsupported-double-precision-device");
            var name = new byte[256]; Check(Driver.cuDeviceGetName(name, name.Length, device), "device-name");
            candidate.DeviceName = Encoding.UTF8.GetString(name).TrimEnd('\0');
            Check(Driver.cuCtxCreate_v2(out candidate._context, 0, device), "context-create");
            long compilationStarted = Stopwatch.GetTimestamp();
            candidate.UseGoldenReference = Environment.GetEnvironmentVariable("RW_TOOLPATHS_GPU_GOLDEN_REFERENCE") == "1";
            byte[] ptx = CompileKernel(major, minor, candidate.UseGoldenReference, fp32: false, cancellationToken, out bool cacheHit);
            candidate.CompilationMilliseconds = Stopwatch.GetElapsedTime(compilationStarted).TotalMilliseconds;
            candidate.KernelCacheHit = cacheHit;
            Check(Driver.cuModuleLoadData(out candidate._module, ptx), "module-load");
            Check(Driver.cuModuleGetFunction(out candidate._function, candidate._module, "nominate_disc_witnesses"), "kernel-find");
            Check(Driver.cuModuleGetFunction(out candidate._boxFunction, candidate._module, "nominate_box_witnesses"), "box-kernel-find");
            Check(Driver.cuModuleGetFunction(out candidate._triangleFunction, candidate._module, "nominate_triangle_witnesses"), "triangle-kernel-find");
            candidate.Fp32Requested = Environment.GetEnvironmentVariable("RW_TOOLPATHS_VCARVE_GPU_FP32") == "1";
            if (candidate.Fp32Requested)
            {
                if (!Fp32DomainSupported(candidate._segments, nodes, profile, out string fp32Reason))
                    candidate.Fp32RefusalReason = fp32Reason;
                else
                {
                    try
                    {
                        compilationStarted = Stopwatch.GetTimestamp();
                        byte[] fp32Ptx = CompileKernel(major, minor, candidate.UseGoldenReference, fp32: true,
                            cancellationToken, out _);
                        candidate.CompilationMilliseconds += Stopwatch.GetElapsedTime(compilationStarted).TotalMilliseconds;
                        Check(Driver.cuModuleLoadData(out candidate._fp32Module, fp32Ptx), "fp32-module-load");
                        Check(Driver.cuModuleGetFunction(out candidate._fp32Function, candidate._fp32Module, "nominate_disc_witnesses"), "fp32-kernel-find");
                        Check(Driver.cuModuleGetFunction(out candidate._fp32BoxFunction, candidate._fp32Module, "nominate_box_witnesses"), "fp32-box-kernel-find");
                        Check(Driver.cuModuleGetFunction(out candidate._fp32TriangleFunction, candidate._fp32Module, "nominate_triangle_witnesses"), "fp32-triangle-kernel-find");
                        candidate.Fp32Enabled = true;
                    }
                    catch (Exception exception) when (IsRecoverable(exception)) { candidate.Fp32RefusalReason = Failure(exception); }
                }
            }
            var sections = new GpuDiscSection[candidate._sectionCount];
            double height = 0;
            for (int i = 0; i < sections.Length; i++)
            {
                var section = profile.Sections[i];
                sections[i] = new(section.BottomRadius, section.TopRadius, section.Height, height, (int)section.Kind);
                height += section.Height;
            }
            candidate._deviceSegments = candidate.AllocateAndUpload(candidate._segments);
            candidate._deviceNodes = candidate.AllocateAndUpload(nodes);
            candidate._deviceSections = candidate.AllocateAndUpload(sections);
            Check(Driver.cuMemAlloc_v2(out candidate._deviceQueries, Bytes<GpuDiscQuery>(MaximumBatchSize)), "queries-allocate");
            Check(Driver.cuMemAlloc_v2(out candidate._deviceNominations, Bytes<GpuDiscNomination>(MaximumBatchSize)), "nominations-allocate");
            Check(Driver.cuMemAlloc_v2(out candidate._deviceBoxQueries, Bytes<GpuBoxQuery>(MaximumBatchSize)), "box-queries-allocate");
            Check(Driver.cuMemAlloc_v2(out candidate._deviceBoxNominations, Bytes<GpuBoxNomination>(MaximumBatchSize)), "box-nominations-allocate");
            Check(Driver.cuMemAlloc_v2(out candidate._deviceTriangleQueries, Bytes<GpuTriangleQuery>(MaximumBatchSize)), "triangle-queries-allocate");
            Check(Driver.cuMemAlloc_v2(out candidate._deviceTriangleNominations, Bytes<GpuTriangleNomination>(MaximumBatchSize)), "triangle-nominations-allocate");
            cancellationToken.ThrowIfCancellationRequested();
            candidate.PreparationMilliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            candidate._diagnostic?.Invoke($"stage=smooth-gpu-witness event=ready device='{candidate.DeviceName}' " +
                $"computeCapability={major}.{minor} intervals={segments.Length} nodes={nodes.Length} sections={sections.Length} " +
                $"uploadedBytes={candidate.UploadedBytes} batchCapacity={MaximumBatchSize} nodeBudget={MaximumTraversalNodes} " +
                $"preparationMs={candidate.PreparationMilliseconds:F2} compilationMs={candidate.CompilationMilliseconds:F2} " +
                $"kernelCacheHit={candidate.KernelCacheHit} goldenReference={candidate.UseGoldenReference} " +
                $"convexBoxProfileSupported={candidate.SupportsConvexBoxWitnesses} convexTriangleProfileSupported={candidate.SupportsConvexTriangleWitnesses} fp32Requested={candidate.Fp32Requested} " +
                $"numericMode={(candidate.Fp32Enabled ? "fp32-relative-xy-nomination" : "fp64-nomination")} " +
                $"fp32Refusal='{candidate.Fp32RefusalReason}' authority=cpu-physical-disc");
            context = candidate; return true;
        }
        catch (OperationCanceledException) { candidate.Dispose(); throw; }
        catch (Exception exception) when (IsRecoverable(exception))
        {
            reason = Failure(exception); candidate.FailureReason = reason;
            candidate.Dispose();
            diagnostic?.Invoke($"stage=smooth-gpu-witness event=fallback reason='{reason}' preparationMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F2}");
            return false;
        }
        finally { if (capturedPrevious) Driver.cuCtxSetCurrent(previous); }
    }

    internal bool TryNominateBoxes(GpuBoxQuery[] queries, GpuBoxNomination[] nominations, int count,
        CancellationToken cancellationToken, out string reason)
    {
        ArgumentNullException.ThrowIfNull(queries); ArgumentNullException.ThrowIfNull(nominations);
        if (count < 0 || count > queries.Length || count > nominations.Length) throw new ArgumentOutOfRangeException(nameof(count));
        cancellationToken.ThrowIfCancellationRequested(); reason = "";
        lock (_gate)
        {
            if (_disposed || _failed) { reason = FailureReason ?? "context-disposed"; return false; }
            IntPtr previous = IntPtr.Zero; bool capturedPrevious = false;
            try
            {
                Check(Driver.cuCtxGetCurrent(out previous), "context-read"); capturedPrevious = true;
                Check(Driver.cuCtxSetCurrent(_context), "context-activate");
                for (int first = 0; first < count; first += MaximumBatchSize)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    int batchCount = Math.Min(MaximumBatchSize, count - first);
                    Array.Copy(queries, first, _boxQueryBuffer, 0, batchCount);
                    for (int i = 0; i < batchCount; i++)
                    {
                        GpuBoxQuery q = _boxQueryBuffer[i];
                        if (!double.IsFinite(q.X0) || !double.IsFinite(q.Y0) || !double.IsFinite(q.X1)
                            || !double.IsFinite(q.Y1) || !double.IsFinite(q.Desired)) throw new GpuFailure("nonfinite-box-query");
                    }
                    bool fp32 = Fp32Enabled && BoxQueriesSupportFp32(_boxQueryBuffer, batchCount);
                    if (Fp32Requested && !fp32) { Fp32RefusedQueries += batchCount; Fp64FallbackBatches++; }
                    long started = Stopwatch.GetTimestamp(); CopyToDevice(_deviceBoxQueries, _boxQueryBuffer, batchCount);
                    TransferMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                    started = Stopwatch.GetTimestamp(); Launch(batchCount, box: true, fp32: fp32); Check(Driver.cuCtxSynchronize(), "box-kernel-synchronize");
                    double elapsed = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                    KernelMilliseconds += elapsed; BoxKernelMilliseconds += elapsed;
                    MaximumBatchKernelMilliseconds = Math.Max(MaximumBatchKernelMilliseconds, elapsed);
                    cancellationToken.ThrowIfCancellationRequested();
                    started = Stopwatch.GetTimestamp(); CopyFromDevice(_boxNominationBuffer, _deviceBoxNominations, batchCount);
                    TransferMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                    Array.Copy(_boxNominationBuffer, 0, nominations, first, batchCount);
                    for (int i = 0; i < batchCount; i++)
                    {
                        int status = _boxNominationBuffer[i].Status;
                        if (status == 1) PositiveNominations++; else if (status == 0) NominationMisses++; else TraversalRefusals++;
                    }
                    Batches++; QueryCount += batchCount; BoxQueryCount += batchCount;
                    if (fp32) { Fp32Batches++; Fp32Queries += batchCount; }
                    if (Stopwatch.GetElapsedTime(_lastDiagnostic).TotalSeconds >= 2)
                    {
                        _lastDiagnostic = Stopwatch.GetTimestamp();
                        _diagnostic?.Invoke($"stage=smooth-gpu-witness event=progress kind=convex-box batches={Batches} queries={QueryCount} " +
                            $"boxQueries={BoxQueryCount} positiveNominations={PositiveNominations} misses={NominationMisses} traversalRefusals={TraversalRefusals} " +
                            $"kernelWallMs={KernelMilliseconds:F2} boxKernelWallMs={BoxKernelMilliseconds:F2} transferMs={TransferMilliseconds:F2} " +
                            $"lastBatchQueries={batchCount} lastBatchBlocks={(batchCount + 127) / 128} threadsPerBlock=128 " +
                            $"numericMode={(fp32 ? "fp32-relative-xy-nomination" : "fp64-nomination")} fp32Queries={Fp32Queries} fp32RefusedQueries={Fp32RefusedQueries} " +
                            $"lastBatchKernelMs={elapsed:F2} maximumBatchKernelMs={MaximumBatchKernelMilliseconds:F2}");
                    }
                }
                return true;
            }
            catch (OperationCanceledException) { throw; }
            catch (Exception exception) when (IsRecoverable(exception))
            {
                reason = Failure(exception); FailureReason = reason; _failed = true;
                _diagnostic?.Invoke($"stage=smooth-gpu-witness event=fallback kind=convex-box reason='{reason}' batches={Batches} queries={QueryCount}");
                return false;
            }
            finally { if (capturedPrevious) Driver.cuCtxSetCurrent(previous); }
        }
    }

    internal bool TryValidateBoxNomination(GpuBoxQuery query, GpuBoxNomination nomination,
        out int sourcePathIndex, Action? discCheckpoint = null)
        => GpuConvexBoxWitnessValidator.TryValidate(_segments, _boxValidationProfile, query, nomination, out sourcePathIndex, discCheckpoint);

    internal bool TryNominateTriangles(GpuTriangleQuery[] queries, GpuTriangleNomination[] nominations, int count,
        CancellationToken cancellationToken, out string reason)
    {
        ArgumentNullException.ThrowIfNull(queries); ArgumentNullException.ThrowIfNull(nominations);
        if (count < 0 || count > queries.Length || count > nominations.Length) throw new ArgumentOutOfRangeException(nameof(count));
        cancellationToken.ThrowIfCancellationRequested(); reason = "";
        lock (_gate)
        {
            if (_disposed || _failed) { reason = FailureReason ?? "context-disposed"; return false; }
            IntPtr previous = IntPtr.Zero; bool capturedPrevious = false;
            try
            {
                Check(Driver.cuCtxGetCurrent(out previous), "context-read"); capturedPrevious = true;
                Check(Driver.cuCtxSetCurrent(_context), "context-activate");
                for (int first = 0; first < count; first += MaximumBatchSize)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    int batchCount = Math.Min(MaximumBatchSize, count - first);
                    Array.Copy(queries, first, _triangleQueryBuffer, 0, batchCount);
                    for (int i = 0; i < batchCount; i++)
                    {
                        GpuTriangleQuery q = _triangleQueryBuffer[i];
                        if (!double.IsFinite(q.Ax) || !double.IsFinite(q.Ay) || !double.IsFinite(q.Bx)
                            || !double.IsFinite(q.By) || !double.IsFinite(q.Cx) || !double.IsFinite(q.Cy)
                            || !double.IsFinite(q.Desired)) throw new GpuFailure("nonfinite-triangle-query");
                    }
                    bool fp32 = Fp32Enabled && TriangleQueriesSupportFp32(_triangleQueryBuffer, batchCount);
                    if (Fp32Requested && !fp32) { Fp32RefusedQueries += batchCount; Fp64FallbackBatches++; }
                    long started = Stopwatch.GetTimestamp(); CopyToDevice(_deviceTriangleQueries, _triangleQueryBuffer, batchCount);
                    TransferMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                    started = Stopwatch.GetTimestamp(); Launch(batchCount, fp32: fp32, triangle: true);
                    Check(Driver.cuCtxSynchronize(), "triangle-kernel-synchronize");
                    double elapsed = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                    KernelMilliseconds += elapsed; TriangleKernelMilliseconds += elapsed;
                    MaximumBatchKernelMilliseconds = Math.Max(MaximumBatchKernelMilliseconds, elapsed);
                    cancellationToken.ThrowIfCancellationRequested();
                    started = Stopwatch.GetTimestamp(); CopyFromDevice(_triangleNominationBuffer, _deviceTriangleNominations, batchCount);
                    TransferMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                    Array.Copy(_triangleNominationBuffer, 0, nominations, first, batchCount);
                    for (int i = 0; i < batchCount; i++)
                    {
                        int status = _triangleNominationBuffer[i].Status;
                        if (status == 1) PositiveNominations++; else if (status == 0) NominationMisses++; else TraversalRefusals++;
                    }
                    Batches++; QueryCount += batchCount; TriangleQueryCount += batchCount;
                    if (fp32) { Fp32Batches++; Fp32Queries += batchCount; }
                    if (Stopwatch.GetElapsedTime(_lastDiagnostic).TotalSeconds >= 2)
                    {
                        _lastDiagnostic = Stopwatch.GetTimestamp();
                        _diagnostic?.Invoke($"stage=smooth-gpu-witness event=progress kind=convex-triangle batches={Batches} queries={QueryCount} " +
                            $"triangleQueries={TriangleQueryCount} positiveNominations={PositiveNominations} misses={NominationMisses} traversalRefusals={TraversalRefusals} " +
                            $"kernelWallMs={KernelMilliseconds:F2} triangleKernelWallMs={TriangleKernelMilliseconds:F2} transferMs={TransferMilliseconds:F2} " +
                            $"lastBatchQueries={batchCount} lastBatchBlocks={(batchCount + 127) / 128} threadsPerBlock=128 " +
                            $"numericMode={(fp32 ? "fp32-relative-xy-nomination" : "fp64-nomination")} fp32Queries={Fp32Queries} fp32RefusedQueries={Fp32RefusedQueries} " +
                            $"lastBatchKernelMs={elapsed:F2} maximumBatchKernelMs={MaximumBatchKernelMilliseconds:F2}");
                    }
                }
                return true;
            }
            catch (OperationCanceledException) { throw; }
            catch (Exception exception) when (IsRecoverable(exception))
            {
                reason = Failure(exception); FailureReason = reason; _failed = true;
                _diagnostic?.Invoke($"stage=smooth-gpu-witness event=fallback kind=convex-triangle reason='{reason}' batches={Batches} queries={QueryCount}");
                return false;
            }
            finally { if (capturedPrevious) Driver.cuCtxSetCurrent(previous); }
        }
    }

    internal bool TryValidateTriangleNomination(GpuTriangleQuery query, GpuTriangleNomination nomination,
        out int sourcePathIndex, Action? discCheckpoint = null)
        => GpuConvexTriangleWitnessValidator.TryValidate(_segments, _boxValidationProfile, query, nomination,
            out sourcePathIndex, discCheckpoint);

    internal bool TryNominate(GpuDiscQuery[] queries, GpuDiscNomination[] nominations, int count,
        CancellationToken cancellationToken, out string reason)
    {
        ArgumentNullException.ThrowIfNull(queries); ArgumentNullException.ThrowIfNull(nominations);
        if (count < 0 || count > queries.Length || count > nominations.Length) throw new ArgumentOutOfRangeException(nameof(count));
        cancellationToken.ThrowIfCancellationRequested(); reason = "";
        lock (_gate)
        {
            if (_disposed || _failed) { reason = FailureReason ?? "context-disposed"; return false; }
            IntPtr previous = IntPtr.Zero; bool capturedPrevious = false;
            try
            {
                Check(Driver.cuCtxGetCurrent(out previous), "context-read"); capturedPrevious = true;
                Check(Driver.cuCtxSetCurrent(_context), "context-activate");
                for (int first = 0; first < count; first += MaximumBatchSize)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    int batchCount = Math.Min(MaximumBatchSize, count - first);
                    Array.Copy(queries, first, _queryBuffer, 0, batchCount);
                    for (int i = 0; i < batchCount; i++)
                    {
                        GpuDiscQuery q = _queryBuffer[i];
                        if (!double.IsFinite(q.X) || !double.IsFinite(q.Y) || !double.IsFinite(q.Desired))
                            throw new GpuFailure("nonfinite-query");
                    }
                    bool fp32 = Fp32Enabled && DiscQueriesSupportFp32(_queryBuffer, batchCount);
                    if (Fp32Requested && !fp32) { Fp32RefusedQueries += batchCount; Fp64FallbackBatches++; }
                    long transferStarted = Stopwatch.GetTimestamp();
                    CopyToDevice(_deviceQueries, _queryBuffer, batchCount);
                    TransferMilliseconds += Stopwatch.GetElapsedTime(transferStarted).TotalMilliseconds;
                    long kernelStarted = Stopwatch.GetTimestamp();
                    Launch(batchCount, fp32: fp32);
                    Check(Driver.cuCtxSynchronize(), "kernel-synchronize");
                    double batchKernelMilliseconds = Stopwatch.GetElapsedTime(kernelStarted).TotalMilliseconds;
                    KernelMilliseconds += batchKernelMilliseconds;
                    MaximumBatchKernelMilliseconds = Math.Max(MaximumBatchKernelMilliseconds, batchKernelMilliseconds);
                    cancellationToken.ThrowIfCancellationRequested();
                    transferStarted = Stopwatch.GetTimestamp();
                    CopyFromDevice(_nominationBuffer, _deviceNominations, batchCount);
                    TransferMilliseconds += Stopwatch.GetElapsedTime(transferStarted).TotalMilliseconds;
                    Array.Copy(_nominationBuffer, 0, nominations, first, batchCount);
                    for (int i = 0; i < batchCount; i++)
                    {
                        int status = _nominationBuffer[i].Status;
                        if (status == 1) PositiveNominations++;
                        else if (status == 0) NominationMisses++;
                        else TraversalRefusals++;
                    }
                    Batches++; QueryCount += batchCount;
                    if (fp32) { Fp32Batches++; Fp32Queries += batchCount; }
                    if (Stopwatch.GetElapsedTime(_lastDiagnostic).TotalSeconds >= 2)
                    {
                        _lastDiagnostic = Stopwatch.GetTimestamp();
                        _diagnostic?.Invoke($"stage=smooth-gpu-witness event=progress batches={Batches} queries={QueryCount} " +
                            $"positiveNominations={PositiveNominations} misses={NominationMisses} traversalRefusals={TraversalRefusals} " +
                            $"kernelWallMs={KernelMilliseconds:F2} transferMs={TransferMilliseconds:F2} " +
                            $"lastBatchQueries={batchCount} lastBatchBlocks={(batchCount + 127) / 128} threadsPerBlock=128 " +
                            $"numericMode={(fp32 ? "fp32-relative-xy-nomination" : "fp64-nomination")} fp32Queries={Fp32Queries} fp32RefusedQueries={Fp32RefusedQueries} " +
                            $"lastBatchKernelMs={batchKernelMilliseconds:F2} maximumBatchKernelMs={MaximumBatchKernelMilliseconds:F2}");
                    }
                }
                return true;
            }
            catch (OperationCanceledException) { throw; }
            catch (Exception exception) when (IsRecoverable(exception))
            {
                reason = Failure(exception); FailureReason = reason; _failed = true;
                _diagnostic?.Invoke($"stage=smooth-gpu-witness event=fallback reason='{reason}' batches={Batches} queries={QueryCount}");
                return false;
            }
            finally { if (capturedPrevious) Driver.cuCtxSetCurrent(previous); }
        }
    }

    // A CUDA success flag alone carries no authority. This predicate is exactly
    // the CPU certificate's literal disc inequality with an actually engaged tip.
    internal bool TryValidateNomination(GpuDiscQuery query, GpuDiscNomination nomination, out int sourcePathIndex)
    {
        sourcePathIndex = -1;
        if (!nomination.HasWitness || nomination.SegmentOrdinal < 0 || nomination.SegmentOrdinal >= _segments.Length
            || !double.IsFinite(nomination.Parameter) || nomination.Parameter < 0 || nomination.Parameter > 1
            || !double.IsFinite(query.X) || !double.IsFinite(query.Y) || !double.IsFinite(query.Desired)) return false;
        GpuDiscSegment s = _segments[nomination.SegmentOrdinal]; double t = nomination.Parameter;
        double z = s.Az + (s.Bz - s.Az) * t;
        if (!(z > 0)) return false;
        double dx = s.Ax + (s.Bx - s.Ax) * t - query.X;
        double dy = s.Ay + (s.By - s.Ay) * t - query.Y;
        if (!(Math.Sqrt(dx * dx + dy * dy) - _profile.PhysicalRadiusAtDepth(z) <= query.Desired)) return false;
        sourcePathIndex = s.SourcePathIndex; return true;
    }

    private static bool ValidateGeometry(GpuDiscSegment[] segments, GpuDiscNode[] nodes,
        VCarveCutterProfile profile, out string reason)
    {
        reason = "";
        if (segments is null || nodes is null || profile is null || segments.Length == 0 || nodes.Length == 0
            || profile.Sections.Count == 0 || profile.Sections.Count > 32) { reason = "unsupported-input"; return false; }
        // Validate all native index ranges before upload; cycles are bounded by
        // the kernel visit budget. Broad-phase misses cannot certify stock.
        foreach (GpuDiscNode node in nodes)
        {
            if (node.First < 0 || node.Count < 0 || node.First > segments.Length - node.Count
                || node.Left < -1 || node.Right < -1 || node.Left >= nodes.Length || node.Right >= nodes.Length
                || (node.Left >= 0 && node.Right < 0)) { reason = "invalid-bvh-indices"; return false; }
        }
        foreach (GpuDiscSegment s in segments)
            if (!double.IsFinite(s.Ax) || !double.IsFinite(s.Ay) || !double.IsFinite(s.Az)
                || !double.IsFinite(s.Bx) || !double.IsFinite(s.By) || !double.IsFinite(s.Bz)
                || !double.IsFinite(s.MaximumRadius) || s.MaximumRadius < 0 || s.SourcePathIndex < 0)
            { reason = "invalid-segment"; return false; }
        return true;
    }

    private static bool Fp32DomainSupported(GpuDiscSegment[] segments, GpuDiscNode[] nodes,
        VCarveCutterProfile profile, out string reason)
    {
        reason = "";
        static bool Within(double value, double limit) => double.IsFinite(value) && Math.Abs(value) <= limit;
        foreach (GpuDiscSegment s in segments)
            if (!Within(s.Ax, 10000) || !Within(s.Ay, 10000) || !Within(s.Bx, 10000) || !Within(s.By, 10000)
                || !Within(s.Az, 100) || !Within(s.Bz, 100) || !Within(s.MaximumRadius, 100))
            { reason = "fp32-geometry-domain"; return false; }
        foreach (GpuDiscNode n in nodes)
            if (!Within(n.X0, 10000) || !Within(n.Y0, 10000) || !Within(n.X1, 10000) || !Within(n.Y1, 10000)
                || !Within(n.MaximumRadius, 100)) { reason = "fp32-bvh-domain"; return false; }
        double height = 0;
        foreach (CarvingCutterSection section in profile.Sections)
        {
            height += section.Height;
            if (!(section.Height >= 1e-6) || !Within(height, 100) || !Within(section.BottomRadius, 100)
                || !Within(section.TopRadius, 100) || !(Math.Max(section.BottomRadius, section.TopRadius) >= 1e-6))
            { reason = "fp32-profile-domain"; return false; }
        }
        return true;
    }

    private static bool DiscQueriesSupportFp32(GpuDiscQuery[] queries, int count)
    {
        for (int i = 0; i < count; i++)
            if (Math.Abs(queries[i].X) > 10000 || Math.Abs(queries[i].Y) > 10000 || Math.Abs(queries[i].Desired) > 100) return false;
        return true;
    }
    private static bool BoxQueriesSupportFp32(GpuBoxQuery[] queries, int count)
    {
        for (int i = 0; i < count; i++)
            if (Math.Abs(queries[i].X0) > 10000 || Math.Abs(queries[i].Y0) > 10000
                || Math.Abs(queries[i].X1) > 10000 || Math.Abs(queries[i].Y1) > 10000 || Math.Abs(queries[i].Desired) > 100) return false;
        return true;
    }

    private static bool TriangleQueriesSupportFp32(GpuTriangleQuery[] queries, int count)
    {
        for (int i = 0; i < count; i++)
            if (Math.Abs(queries[i].Ax) > 10000 || Math.Abs(queries[i].Ay) > 10000
                || Math.Abs(queries[i].Bx) > 10000 || Math.Abs(queries[i].By) > 10000
                || Math.Abs(queries[i].Cx) > 10000 || Math.Abs(queries[i].Cy) > 10000
                || Math.Abs(queries[i].Desired) > 100) return false;
        return true;
    }

    private void Launch(int count, bool box = false, bool fp32 = false, bool triangle = false)
    {
        // CUDA takes addresses of argument values, including device pointers.
        // Eight-byte slots also hold the four-byte little-endian integer args.
        ulong[] values = { _deviceSegments, _deviceNodes, (ulong)_nodeCount, _deviceSections,
            (ulong)_sectionCount, triangle ? _deviceTriangleQueries : box ? _deviceBoxQueries : _deviceQueries,
            triangle ? _deviceTriangleNominations : box ? _deviceBoxNominations : _deviceNominations, (ulong)count, MaximumTraversalNodes };
        GCHandle valuesPin = GCHandle.Alloc(values, GCHandleType.Pinned);
        GCHandle addressesPin = default;
        try
        {
            var addresses = new IntPtr[values.Length];
            for (int i = 0; i < addresses.Length; i++) addresses[i] = IntPtr.Add(valuesPin.AddrOfPinnedObject(), i * sizeof(ulong));
            addressesPin = GCHandle.Alloc(addresses, GCHandleType.Pinned);
            IntPtr function = fp32 ? (triangle ? _fp32TriangleFunction : box ? _fp32BoxFunction : _fp32Function)
                : (triangle ? _triangleFunction : box ? _boxFunction : _function);
            Check(Driver.cuLaunchKernel(function, (uint)((count + 127) / 128), 1, 1,
                128, 1, 1, 0, IntPtr.Zero, addressesPin.AddrOfPinnedObject(), IntPtr.Zero), "kernel-launch");
        }
        finally { if (addressesPin.IsAllocated) addressesPin.Free(); valuesPin.Free(); }
    }

    private ulong AllocateAndUpload<T>(T[] values) where T : struct
    {
        Check(Driver.cuMemAlloc_v2(out ulong pointer, Bytes<T>(values.Length)), "snapshot-allocate");
        try
        {
            long started = Stopwatch.GetTimestamp(); CopyToDevice(pointer, values, values.Length);
            TransferMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            UploadedBytes += checked((long)Bytes<T>(values.Length)); return pointer;
        }
        catch { Driver.cuMemFree_v2(pointer); throw; }
    }
    private static nuint Bytes<T>(int count) where T : struct => checked((nuint)(Marshal.SizeOf<T>() * (long)count));
    private static void CopyToDevice<T>(ulong pointer, T[] values, int count) where T : struct
    {
        GCHandle pin = GCHandle.Alloc(values, GCHandleType.Pinned);
        try { Check(Driver.cuMemcpyHtoD_v2(pointer, pin.AddrOfPinnedObject(), Bytes<T>(count)), "upload"); }
        finally { pin.Free(); }
    }
    private static void CopyFromDevice<T>(T[] values, ulong pointer, int count) where T : struct
    {
        GCHandle pin = GCHandle.Alloc(values, GCHandleType.Pinned);
        try { Check(Driver.cuMemcpyDtoH_v2(pin.AddrOfPinnedObject(), pointer, Bytes<T>(count)), "download"); }
        finally { pin.Free(); }
    }

    public void Dispose()
    {
        lock (_gate)
        {
            if (_disposed) return; _disposed = true;
            if (_context == IntPtr.Zero) return;
            IntPtr previous = IntPtr.Zero; Driver.cuCtxGetCurrent(out previous); Driver.cuCtxSetCurrent(_context);
            foreach (ulong pointer in new[] { _deviceTriangleNominations, _deviceTriangleQueries, _deviceBoxNominations, _deviceBoxQueries,
                _deviceNominations, _deviceQueries, _deviceSections, _deviceNodes, _deviceSegments })
                if (pointer != 0) Driver.cuMemFree_v2(pointer);
            if (_module != IntPtr.Zero) Driver.cuModuleUnload(_module);
            if (_fp32Module != IntPtr.Zero) Driver.cuModuleUnload(_fp32Module);
            Driver.cuCtxDestroy_v2(_context);
            Driver.cuCtxSetCurrent(previous == _context ? IntPtr.Zero : previous); _context = IntPtr.Zero;
            _diagnostic?.Invoke($"stage=smooth-gpu-witness event=disposed device='{DeviceName}' batches={Batches} queries={QueryCount} " +
                $"uploadedBytes={UploadedBytes} transferMs={TransferMilliseconds:F2} kernelWallMs={KernelMilliseconds:F2} " +
                $"preparationMs={PreparationMilliseconds:F2} compilationMs={CompilationMilliseconds:F2} " +
                $"kernelCacheHit={KernelCacheHit} goldenReference={UseGoldenReference} positiveNominations={PositiveNominations} " +
                $"misses={NominationMisses} traversalRefusals={TraversalRefusals} maximumBatchKernelMs={MaximumBatchKernelMilliseconds:F2} " +
                $"boxQueries={BoxQueryCount} boxKernelWallMs={BoxKernelMilliseconds:F2} " +
                $"triangleQueries={TriangleQueryCount} triangleKernelWallMs={TriangleKernelMilliseconds:F2} " +
                $"fp32Requested={Fp32Requested} fp32Enabled={Fp32Enabled} fp32Batches={Fp32Batches} fp32Queries={Fp32Queries} " +
                $"fp32RefusedQueries={Fp32RefusedQueries} fp64FallbackBatches={Fp64FallbackBatches} fp32Refusal='{Fp32RefusalReason}' " +
                $"failed={_failed} reason='{FailureReason}'");
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    private readonly struct GpuDiscSection
    {
        internal readonly double BottomRadius, TopRadius, Height, StartHeight;
        internal readonly int Kind;
        internal GpuDiscSection(double bottomRadius, double topRadius, double height, double startHeight, int kind)
        { BottomRadius = bottomRadius; TopRadius = topRadius; Height = height; StartHeight = startHeight; Kind = kind; }
    }

    private static byte[] CompileKernel(int major, int minor, bool goldenReference, bool fp32, CancellationToken token, out bool cacheHit)
    {
        lock (KernelCacheGate)
        {
            token.ThrowIfCancellationRequested();
            string key = $"compute_{major}{minor}:golden={goldenReference}:fp32={fp32}";
            if (KernelCodeCache.TryGetValue(key, out byte[]? existing)) { cacheHit = true; return existing; }
            byte[] result = CompileKernelUncached(major, minor, goldenReference, fp32, token);
            KernelCodeCache.Add(key, result); cacheHit = false; return result;
        }
    }

    private static byte[] CompileKernelUncached(int major, int minor, bool goldenReference, bool fp32, CancellationToken token)
    {
        using Stream source = typeof(VCarveGpuDiscWitnessContext).Assembly.GetManifestResourceStream(ResourceName)
            ?? throw new GpuFailure("kernel-resource-missing");
        using var reader = new StreamReader(source); string code = reader.ReadToEnd();
        IntPtr library = LoadNvrtc(); IntPtr program = IntPtr.Zero;
        try
        {
            T Bind<T>(string name) where T : Delegate => Marshal.GetDelegateForFunctionPointer<T>(NativeLibrary.GetExport(library, name));
            var create = Bind<NvrtcCreateProgram>("nvrtcCreateProgram"); var compile = Bind<NvrtcCompileProgram>("nvrtcCompileProgram");
            var size = Bind<NvrtcGetSize>("nvrtcGetPTXSize"); var ptx = Bind<NvrtcGetBytes>("nvrtcGetPTX");
            Check(create(out program, code, "disc_witness.cu", 0, IntPtr.Zero, IntPtr.Zero), "nvrtc-create");
            string[] options = { "--std=c++11", $"--gpu-architecture=compute_{major}{minor}", "--fmad=false", "--prec-div=true", "--prec-sqrt=true",
                goldenReference ? "-DGPU_GOLDEN_REFERENCE=1" : "-DGPU_GOLDEN_REFERENCE=0",
                fp32 ? "-DGPU_FP32_NOMINATION=1" : "-DGPU_FP32_NOMINATION=0" };
            token.ThrowIfCancellationRequested();
            int result = compile(program, options.Length, options);
            if (result != 0)
            {
                Bind<NvrtcGetSize>("nvrtcGetProgramLogSize")(program, out nuint logSize);
                var log = new byte[checked((int)logSize)];
                Bind<NvrtcGetBytes>("nvrtcGetProgramLog")(program, log);
                throw new GpuFailure($"nvrtc-compile-{result}: {Encoding.UTF8.GetString(log).TrimEnd('\0')}");
            }
            token.ThrowIfCancellationRequested(); Check(size(program, out nuint bytes), "nvrtc-ptx-size");
            var output = new byte[checked((int)bytes)]; Check(ptx(program, output), "nvrtc-ptx"); return output;
        }
        finally
        {
            if (program != IntPtr.Zero)
                Marshal.GetDelegateForFunctionPointer<NvrtcDestroyProgram>(NativeLibrary.GetExport(library, "nvrtcDestroyProgram"))(ref program);
            NativeLibrary.Free(library);
        }
    }

    private static IntPtr LoadNvrtc()
    {
        // Prefer the installed toolkit rather than shipping an unrelated CUDA
        // runtime. CUDA-free machines preserve the managed implementation.
        string? cudaPath = Environment.GetEnvironmentVariable("CUDA_PATH");
        var folders = new List<string> { AppContext.BaseDirectory };
        if (!string.IsNullOrWhiteSpace(cudaPath)) folders.Add(Path.Combine(cudaPath, "bin"));
        string toolkitRoot = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles), "NVIDIA GPU Computing Toolkit", "CUDA");
        if (Directory.Exists(toolkitRoot))
            folders.AddRange(Directory.GetDirectories(toolkitRoot).OrderByDescending(path => path, StringComparer.OrdinalIgnoreCase).Select(path => Path.Combine(path, "bin")));
        foreach (string folder in folders.Distinct(StringComparer.OrdinalIgnoreCase))
        {
            if (!Directory.Exists(folder)) continue;
            foreach (string file in Directory.GetFiles(folder, "nvrtc64_*.dll").OrderByDescending(path => path, StringComparer.OrdinalIgnoreCase))
                if (NativeLibrary.TryLoad(file, out IntPtr library)) return library;
        }
        foreach (string name in new[] { "nvrtc64_130_0.dll", "nvrtc64_120_0.dll", "nvrtc64_112_0.dll" })
            if (NativeLibrary.TryLoad(name, out IntPtr library)) return library;
        throw new GpuFailure("nvrtc-runtime-unavailable");
    }

    private static bool IsRecoverable(Exception exception) => exception is GpuFailure or DllNotFoundException
        or EntryPointNotFoundException or BadImageFormatException or ExternalException or IOException or UnauthorizedAccessException;
    private static string Failure(Exception exception)
    {
        string value = exception is GpuFailure ? exception.Message : exception.GetType().Name + ": " + exception.Message;
        return value.Length > 1200 ? value[..1200] : value;
    }
    private static void Check(int result, string operation) { if (result != 0) throw new GpuFailure($"{operation}-error-{result}"); }
    private sealed class GpuFailure : Exception { internal GpuFailure(string reason) : base(reason) { } }

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] private delegate int NvrtcCreateProgram(out IntPtr program,
        [MarshalAs(UnmanagedType.LPStr)] string source, [MarshalAs(UnmanagedType.LPStr)] string name, int headerCount, IntPtr headers, IntPtr names);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] private delegate int NvrtcCompileProgram(IntPtr program, int count,
        [MarshalAs(UnmanagedType.LPArray, ArraySubType = UnmanagedType.LPStr, SizeParamIndex = 1)] string[] options);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] private delegate int NvrtcGetSize(IntPtr program, out nuint bytes);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] private delegate int NvrtcGetBytes(IntPtr program, [Out] byte[] bytes);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)] private delegate int NvrtcDestroyProgram(ref IntPtr program);

    private static class Driver
    {
        private const string Library = "nvcuda.dll";
        [DllImport(Library)] internal static extern int cuInit(uint flags);
        [DllImport(Library)] internal static extern int cuDeviceGetCount(out int count);
        [DllImport(Library)] internal static extern int cuDeviceGet(out int device, int ordinal);
        [DllImport(Library)] internal static extern int cuDeviceGetName([Out] byte[] name, int length, int device);
        [DllImport(Library)] internal static extern int cuDeviceComputeCapability(out int major, out int minor, int device);
        [DllImport(Library)] internal static extern int cuCtxGetCurrent(out IntPtr context);
        [DllImport(Library)] internal static extern int cuCtxCreate_v2(out IntPtr context, uint flags, int device);
        [DllImport(Library)] internal static extern int cuCtxSetCurrent(IntPtr context);
        [DllImport(Library)] internal static extern int cuCtxSynchronize();
        [DllImport(Library)] internal static extern int cuCtxDestroy_v2(IntPtr context);
        [DllImport(Library)] internal static extern int cuModuleLoadData(out IntPtr module, [In] byte[] image);
        [DllImport(Library, CharSet = CharSet.Ansi)] internal static extern int cuModuleGetFunction(out IntPtr function, IntPtr module, string name);
        [DllImport(Library)] internal static extern int cuModuleUnload(IntPtr module);
        [DllImport(Library)] internal static extern int cuMemAlloc_v2(out ulong pointer, nuint bytes);
        [DllImport(Library)] internal static extern int cuMemFree_v2(ulong pointer);
        [DllImport(Library)] internal static extern int cuMemcpyHtoD_v2(ulong destination, IntPtr source, nuint bytes);
        [DllImport(Library)] internal static extern int cuMemcpyDtoH_v2(IntPtr destination, ulong source, nuint bytes);
        [DllImport(Library)] internal static extern int cuLaunchKernel(IntPtr function, uint gridX, uint gridY, uint gridZ,
            uint blockX, uint blockY, uint blockZ, uint sharedBytes, IntPtr stream, IntPtr parameters, IntPtr extra);
    }
}
