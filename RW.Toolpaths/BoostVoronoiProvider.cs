using Clipper2Lib;
using System.Globalization;
using System.Diagnostics;

namespace RW.Toolpaths;

/// <summary>
/// <see cref="IMedialAxisProvider"/> backed by native Boost.Polygon.Voronoi
/// via P/Invoke to <c>boostvoronoi.dll</c> / <c>libboostvoronoi.so</c>.
///
///   integer segment-site Voronoi  ->  ClassifyEdges  ->
///   SelectInnerPrimaryTraversal   ->  TryCreateOutputSegments
/// </summary>
public sealed partial class BoostVoronoiProvider : IMedialAxisProvider
{
    public static BoostVoronoiProvider CreateDefault() => new();

    internal IReadOnlyList<MedialSegment> ConstructNominalMedialAxis(
        IReadOnlyList<PointD> boundary, IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance, double maxRadius, double cornerAngle, CancellationToken token,
        Action<string>? diagnostic, Action<double, string, bool>? progress)
        => ConstructCore(boundary, holes, tolerance, maxRadius, cornerAngle, true,
            token, diagnostic, progress, false, nominalRadiusSampling: true);

    public IReadOnlyList<MedialSegment> ConstructMedialAxis(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance,
        double maxRadius,
        double filteringAngle = Math.PI,
        bool useBigIntegers = true)
        => ConstructMedialAxis(boundary, holes, tolerance, maxRadius, filteringAngle,
            useBigIntegers, CancellationToken.None);

    public IReadOnlyList<MedialSegment> ConstructMedialAxis(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance,
        double maxRadius,
        double filteringAngle,
        bool useBigIntegers,
        CancellationToken cancellationToken,
        Action<string>? diagnostic = null,
        Action<double, string, bool>? progress = null)
        => ConstructCore(boundary, holes, tolerance, maxRadius, filteringAngle, useBigIntegers,
            cancellationToken, diagnostic, progress, certifiedGeometricError: false);

    internal IReadOnlyList<MedialSegment> ConstructCertifiedMedialAxis(
        IReadOnlyList<PointD> boundary, IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance, double maxRadius, double filteringAngle, CancellationToken cancellationToken,
        Action<string>? diagnostic, Action<double, string, bool>? progress)
        => ConstructCore(boundary, holes, tolerance, maxRadius, filteringAngle, true,
            cancellationToken, diagnostic, progress, certifiedGeometricError: true);

    internal IReadOnlyList<MedialSegment> ConstructBoundedSourceMedialAxis(
        IReadOnlyList<PointD> boundary, IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance, double maxRadius, double filteringAngle, CancellationToken cancellationToken,
        Action<string>? diagnostic, Action<double, string, bool>? progress,
        BoundedLocalMedialEmitter.SourceCoordinates coordinates)
        => ConstructCore(boundary, holes, tolerance, maxRadius, filteringAngle, true,
            cancellationToken, diagnostic, progress, certifiedGeometricError: false, coordinates);

    private static IReadOnlyList<MedialSegment> ConstructCore(
        IReadOnlyList<PointD> boundary, IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance, double maxRadius, double filteringAngle, bool useBigIntegers,
        CancellationToken cancellationToken, Action<string>? diagnostic,
        Action<double, string, bool>? progress, bool certifiedGeometricError,
        BoundedLocalMedialEmitter.SourceCoordinates? sourceCoordinates = null,
        bool nominalRadiusSampling = false)
    {
        cancellationToken.ThrowIfCancellationRequested();
        long t0 = PerfLog.Start();
        if (boundary is null) throw new ArgumentNullException(nameof(boundary));
        if (holes is null)    throw new ArgumentNullException(nameof(holes));
        if (boundary.Count < 3)
        {
            PerfLog.Stop("BoostVoronoiProvider.ConstructMedialAxis", t0, "boundary<3");
            return Array.Empty<MedialSegment>();
        }

        // Caller controls coordinate scale domain (e.g. e.up in ridge-pass()).
        var scaledBoundary = RoundAndNormalizeRing(boundary);
        var scaledHoles = new List<List<PointD>>(holes.Count);
        for (int i = 0; i < holes.Count; i++)
        {
            var ring = RoundAndNormalizeRing(holes[i]);
            if (ring.Count >= 3)
                scaledHoles.Add(ring);
        }

        // Build one SegmentSiteData per edge of each ring.
        // Low/High follow Boost's canonical y-then-x ordering so that
        // source_category maps back to the correct endpoint.
        var segSites = BuildSegmentSites(scaledBoundary, scaledHoles, cancellationToken, out var wideArrangement);
        int segCount = segSites.Count;
        if (segCount < 3)
        {
            PerfLog.Stop("BoostVoronoiProvider.ConstructMedialAxis", t0, "segments<3");
            return Array.Empty<MedialSegment>();
        }

        // Unpack to parallel int[] arrays for the C call.
        int[] ax = new int[segCount], ay = new int[segCount];
        int[] bx = new int[segCount], by = new int[segCount];
        for (int i = 0; i < segCount; i++)
        {
            var s = segSites[i];
            if (wideArrangement is null)
            {
                ax[i] = (int)s.Ax; ay[i] = (int)s.Ay;
                bx[i] = (int)s.Bx; by[i] = (int)s.By;
            }
        }

        cancellationToken.ThrowIfCancellationRequested();
        diagnostic?.Invoke($"stage=voronoi-construction event=start sites={segCount}");
        progress?.Invoke(0, "Building Voronoi diagram", true);
        cancellationToken.ThrowIfCancellationRequested();
        long constructAt = Stopwatch.GetTimestamp();
        IntPtr handle = wideArrangement is null
            ? BoostVoronoiInterop.bv_construct(ax, ay, bx, by, segCount)
            : BoostVoronoiInterop.bv_construct_wide(
                wideArrangement.Atoms.Select(a => a.A.X).ToArray(), wideArrangement.Atoms.Select(a => a.A.Y).ToArray(),
                wideArrangement.Atoms.Select(a => a.B.X).ToArray(), wideArrangement.Atoms.Select(a => a.B.Y).ToArray(), segCount);

        if (handle == IntPtr.Zero)
        {
            PerfLog.Stop("BoostVoronoiProvider.ConstructMedialAxis", t0, "construct-handle-null");
            throw new InvalidOperationException("Native medial construction failed; the complete source diagram is required.");
        }

        try
        {
            diagnostic?.Invoke($"stage=voronoi-construction event=complete sites={segCount} elapsedMs={Stopwatch.GetElapsedTime(constructAt).TotalMilliseconds:F2}");
            progress?.Invoke(0.15, "Reading Voronoi diagram", false);
            cancellationToken.ThrowIfCancellationRequested();
            var output = RunPipeline(handle, segSites, scaledBoundary, scaledHoles,
                                     tolerance, maxRadius, filteringAngle, cancellationToken, diagnostic, progress,
                                     certifiedGeometricError, sourceCoordinates, wideArrangement?.Scale ?? 1, nominalRadiusSampling);
            PerfLog.Stop(
                "BoostVoronoiProvider.ConstructMedialAxis",
                t0,
                $"boundaryPts={boundary.Count} holes={holes.Count} segSites={segCount} out={output.Count}");
            return output;
        }
        finally
        {
            BoostVoronoiInterop.bv_destroy(handle);
        }
    }

    // -- Pipeline --------------------------------------------------------------

    private static IReadOnlyList<MedialSegment> RunPipeline(
        IntPtr handle,
        IReadOnlyList<SegmentSiteData> segSites,
        IReadOnlyList<PointD> scaledBoundary,
        IReadOnlyList<IReadOnlyList<PointD>> scaledHoles,
        double tolerance,
        double maxRadius,
        double filteringAngle,
        CancellationToken cancellationToken,
        Action<string>? diagnostic,
        Action<double, string, bool>? progress,
        bool certifiedGeometricError,
        BoundedLocalMedialEmitter.SourceCoordinates? sourceCoordinates, long nativeScale = 1,
        bool nominalRadiusSampling = false)
    {
        cancellationToken.ThrowIfCancellationRequested();
        long t0 = PerfLog.Start();
        int edgeCount   = BoostVoronoiInterop.bv_edge_count(handle);
        int vertexCount = BoostVoronoiInterop.bv_vertex_count(handle);
        int cellCount   = BoostVoronoiInterop.bv_cell_count(handle);

        var bvEdges    = new BoostVoronoiInterop.BvEdge[edgeCount];
        var bvVertices = new BoostVoronoiInterop.BvVertex[vertexCount];
        var bvCells    = new BoostVoronoiInterop.BvCell[cellCount];

        BoostVoronoiInterop.bv_get_edges(handle, bvEdges, edgeCount);
        BoostVoronoiInterop.bv_get_vertices(handle, bvVertices, vertexCount);
        BoostVoronoiInterop.bv_get_cells(handle, bvCells, cellCount);

        if (nativeScale != 1)
            for (int i = 0; i < bvVertices.Length; i++)
            {
                if ((i & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
                bvVertices[i].X /= nativeScale;
                bvVertices[i].Y /= nativeScale;
            }

        for (int i = 0; i < bvVertices.Length; i++)
        {
            if ((i & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
            if (double.IsFinite(bvVertices[i].X) && double.IsFinite(bvVertices[i].Y)) continue;
            Console.WriteLine($"[BoostVoronoi.InvalidNativeVertex] vertex={i} x={bvVertices[i].X:R} y={bvVertices[i].Y:R} "
                + $"sites={segSites.Count} edges={edgeCount} vertices={vertexCount} originalSignedRings=True decision=fail-complete-diagram");
            foreach (var edge in bvEdges.Where(edge => edge.Vertex0Index == i || edge.Vertex1Index == i).Take(16))
                Console.WriteLine($"[BoostVoronoi.InvalidNativeVertex.Incident] vertex={i} "
                    + $"from={edge.Vertex0Index} to={edge.Vertex1Index} primary={edge.IsPrimary} infinite={edge.IsInfinite} "
                    + $"source={bvCells[edge.CellIndex].SourceIndex} twinSource={bvCells[edge.TwinCellIndex].SourceIndex}");
            string? captureDirectory = Environment.GetEnvironmentVariable("RW_TOOLPATHS_MEDIAL_FAILURE_CAPTURE");
            if (!string.IsNullOrWhiteSpace(captureDirectory))
            {
                try
                {
                    Directory.CreateDirectory(captureDirectory);
                    string capturePath = Path.Combine(captureDirectory, $"medial-failure-{Guid.NewGuid():N}.json");
                    File.WriteAllText(capturePath, System.Text.Json.JsonSerializer.Serialize(new
                    {
                        boundary = scaledBoundary.Select(p => new[] { p.x, p.y }),
                        holes = scaledHoles.Select(ring => ring.Select(p => new[] { p.x, p.y })),
                        tolerance, maxRadius, filteringAngle,
                    }));
                    Console.WriteLine($"[BoostVoronoi.FailureCapture] path={capturePath} sites={segSites.Count} complete=True");
                }
                catch (Exception error) when (error is IOException or UnauthorizedAccessException)
                {
                    Console.WriteLine($"[BoostVoronoi.FailureCapture] failed={error.Message}");
                }
            }
            throw new InvalidOperationException($"The complete original-site medial diagram contains a nonfinite native vertex {i}; no original edge or material may be omitted.");
        }

        // Build CellSite lookup from Boost source_index + source_category.
        var cellSites = new MedialAxisEdgeClassifier.CellSite[cellCount];
        for (int i = 0; i < cellCount; i++)
            cellSites[i] = MakeCellSite(bvCells[i], segSites);

        // Vertices are immutable values identified by the native diagram's
        // stable index. Reuse their objects across halfedges without changing
        // any coordinate, site owner or edge/traversal ordinal.
        bool referenceVertexAllocation = string.Equals(
            Environment.GetEnvironmentVariable("RW_TOOLPATHS_VERTEX_ALLOCATION_REFERENCE"), "1", StringComparison.Ordinal);
        MedialAxisEdgeClassifier.EdgePoint[]? vertexPoints = referenceVertexAllocation
            ? null : new MedialAxisEdgeClassifier.EdgePoint[vertexCount];
        if (vertexPoints is not null)
            for (int i = 0; i < vertexPoints.Length; i++)
            {
                if ((i & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
                vertexPoints[i] = new(bvVertices[i].X, bvVertices[i].Y);
            }
        int endpointReferences = 0;
        MedialAxisEdgeClassifier.EdgePoint? Vertex(int index)
        {
            if (index < 0) return null;
            endpointReferences++;
            return vertexPoints is not null ? vertexPoints[index]
                : new(bvVertices[index].X, bvVertices[index].Y);
        }

        // Build EdgeData[]; IsLinear/IsCurved are set post-construction.
        var edgeData = new MedialAxisEdgeClassifier.EdgeData[edgeCount];
        for (int i = 0; i < edgeCount; i++)
        {
            if ((i & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
            ref var e = ref bvEdges[i];
            bool isPrimary = e.IsPrimary != 0;
            edgeData[i] = new MedialAxisEdgeClassifier.EdgeData(
                Index:       i,
                TwinIndex:   e.TwinIndex,
                IsPrimary:   isPrimary,
                IsSecondary: !isPrimary,
                IsInfinite:  e.IsInfinite != 0,
                PrevIndex:   e.PrevIndex,
                NextIndex:   e.NextIndex,
                Vertex0: Vertex(e.Vertex0Index),
                Vertex1: Vertex(e.Vertex1Index),
                Cell:     e.CellIndex     >= 0 ? cellSites[e.CellIndex]     : cellSites[0],
                TwinCell: e.TwinCellIndex >= 0 ? cellSites[e.TwinCellIndex] : cellSites[0]
            )
            {
                IsLinear = e.IsLinear != 0,
                IsCurved = e.IsCurved != 0,
            };
        }

        Console.WriteLine($"[BoostVoronoi.VertexObjects] mode={(referenceVertexAllocation ? "per-halfedge-reference" : "immutable-native-index")} "
            + $"vertices={vertexCount} endpointReferences={endpointReferences} "
            + $"allocatedObjects={(referenceVertexAllocation ? endpointReferences : vertexCount)} "
            + $"numericValues=unchanged nativeIndices=unchanged sourceOwners=unchanged");

        bool debugEnabled = IsMedialDebugEnabled();

        diagnostic?.Invoke($"stage=voronoi-classification event=start edges={edgeCount}");
        progress?.Invoke(0.25, "Classifying medial edges", false);
        long classifyAt = Stopwatch.GetTimestamp();
        MedialAxisEdgeClassifier.ClassifyEdges(edgeData, scaledBoundary, scaledHoles, cancellationToken, diagnostic,
            progress is null ? null : fraction => progress(0.25 + 0.5 * fraction, "Classifying medial edges", false));
        PerfLog.Stop("BoostVoronoiProvider.ClassifyEdges", classifyAt, $"edges={edgeCount}");
        diagnostic?.Invoke($"stage=voronoi-classification event=timing elapsedMs={Stopwatch.GetElapsedTime(classifyAt).TotalMilliseconds:F2}");
        diagnostic?.Invoke($"stage=voronoi-traversal event=start edges={edgeCount}");
        progress?.Invoke(0.75, "Connecting medial edges", false);
        long traverseAt = Stopwatch.GetTimestamp();

        var walked = MedialAxisEdgeClassifier.SelectInnerPrimaryTraversal(edgeData, cancellationToken, diagnostic,
            progress is null ? null : fraction => progress(0.75 + 0.15 * fraction, "Connecting medial edges", false));
        PerfLog.Stop("BoostVoronoiProvider.TraverseEdges", traverseAt, $"edges={edgeCount} walked={walked.Count}");
        diagnostic?.Invoke($"stage=voronoi-traversal event=complete walked={walked.Count} "
            + $"elapsedMs={Stopwatch.GetElapsedTime(traverseAt).TotalMilliseconds:F2} anglePruning=False");

        // maxRadius is already in the same coordinate scale domain as the inputs.
        double emitMaxRadius = maxRadius > 0 ? maxRadius : -1.0;
        // CentralAngle tolerance is in radians â€” scale-invariant, pass as-is.

        var rawOutput = new List<MedialSegment>(walked.Count);
        int cornerRejected = 0;
        int skippedNullVertices = 0;
        int emitRejected = 0;
        int sourceCertifiedCurves = 0, sourceCertifiedLinear = 0, sourceFallbacks = 0;
        long sourcePieces = 0;
        double sourceMaximumWorldBound = 0;
        var sourceFallbackReasons = sourceCoordinates is null ? null : new Dictionary<string, int>();
        diagnostic?.Invoke($"stage=voronoi-emission event=start edges={walked.Count} " +
            $"discretization={(sourceCoordinates is not null ? "finite-world-source-chord" : certifiedGeometricError ? "bounded-physical-error" : "central-angle")} tolerance={tolerance:R}");
        progress?.Invoke(0.9, "Emitting medial paths", false);
        long emitAt = Stopwatch.GetTimestamp(), progressAt = Stopwatch.GetTimestamp();
        int completed = 0;
        foreach (int idx in walked)
        {
            if ((completed++ & 1023) == 0)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if ((diagnostic is not null || progress is not null) && Stopwatch.GetElapsedTime(progressAt).TotalMilliseconds >= 250)
                {
                    diagnostic?.Invoke($"stage=voronoi-emission event=progress completed={completed} total={walked.Count}");
                    progress?.Invoke(0.9 + 0.1 * completed / Math.Max(1, walked.Count), "Emitting medial paths", false);
                    progressAt = Stopwatch.GetTimestamp();
                }
            }
            var edge = edgeData[idx];
            // The requested construction policy acts on the two generating
            // segments, never on branch length or the appearance of output.
            if (filteringAngle < Math.PI && MedialAxisEdgeClassifier.SharedCornerAngle(edge) is { } angle
                && angle > filteringAngle)
            { cornerRejected++; continue; }
            if (edge.Vertex0 is null || edge.Vertex1 is null)
            {
                skippedNullVertices++;
                continue;
            }

            if (sourceCoordinates is { } coordinates)
            {
                int before = rawOutput.Count;
                bool certified = BoundedLocalMedialEmitter.TryEmitFull(edge, coordinates, .001,
                    rawOutput, cancellationToken, out string failure, out int visited, out double acceptedBound);
                sourcePieces += visited;
                if (certified)
                {
                    if (edge.IsCurved) sourceCertifiedCurves++; else sourceCertifiedLinear++;
                    sourceMaximumWorldBound = Math.Max(sourceMaximumWorldBound, acceptedBound);
                    continue;
                }
                if (rawOutput.Count != before)
                    throw new InvalidOperationException("An uncertified full-source medial edge published partial geometry.");
                sourceFallbacks++;
                sourceFallbackReasons![failure] = sourceFallbackReasons.GetValueOrDefault(failure) + 1;
                if (sourceFallbacks <= 8)
                    Console.WriteLine($"[Offset.Terminal.SourceEmissionFallback] edge={idx} reason={failure} pieces={visited} "
                        + $"first=({edge.Vertex0.X:R},{edge.Vertex0.Y:R}) last=({edge.Vertex1.X:R},{edge.Vertex1.Y:R}) "
                        + $"curved={edge.IsCurved} decision=complete-angular-edge partialPublished=False "
                        + $"originalSites=True originalOwner=True wallAuthority=unchanged");
            }

            bool emitted = MedialAxisSegmentEmitter.TryCreateOutputSegments(
                edge, rawOutput,
                noParabola: false,
                showSites:  false,
                tolerance:  tolerance,
                method:     certifiedGeometricError ? MedialAxisSegmentEmitter.ParabolaDiscretizationMethod.Error2D
                    : MedialAxisSegmentEmitter.ParabolaDiscretizationMethod.CentralAngle,
                maxRadius:  emitMaxRadius,
                rejectIncompleteDiscretization: certifiedGeometricError,
                samplePointPointRadius: nominalRadiusSampling);

            if (!emitted)
            {
                emitRejected++;
            }
        }

        if (sourceCoordinates is { } placement)
        {
            string message = $"[Offset.Terminal.SourceEmission] traversalEdges={walked.Count} segments={rawOutput.Count} "
                + $"certifiedCurved={sourceCertifiedCurves} certifiedLinear={sourceCertifiedLinear} "
                + $"fallbackEdges={sourceFallbacks} nullVertices={skippedNullVertices} rejected={emitRejected} "
                + $"visitedPieces={sourcePieces} maximumAcceptedWorldBound={sourceMaximumWorldBound:R} "
                + $"maximumWorldError=0.001 originX={placement.OriginX:R} originY={placement.OriginY:R} scale={placement.Scale:R} "
                + $"fallbackReasons={string.Join(",", sourceFallbackReasons!.OrderBy(pair => pair.Key).Select(pair => pair.Key + ":" + pair.Value))} "
                + "finiteControlCapsule=True worldConversionEnclosed=True originalSites=True originalOwners=True "
                + "originalSignedClassifier=True originalTraversal=True atomicEdges=True wallAllowanceChange=0";
            Console.WriteLine(message);
            diagnostic?.Invoke(message);
        }

        // A complete-original reference fallback is permissible only as the
        // unchanged full diagram, never by omitting a bad sample or native edge.
        // The caller's original wall and complete coverage proofs still decide
        // whether any emitted paths may be used as physical cutter motion.
        for (int i = 0; i < rawOutput.Count; i++)
        {
            if ((i & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
            var segment = rawOutput[i];
            bool Finite(MedialPoint point) => double.IsFinite(point.X) && double.IsFinite(point.Y)
                && double.IsFinite(point.Radius) && point.Radius >= 0;
            if (Finite(segment.Point0) && Finite(segment.Point1)) continue;
            Console.WriteLine($"[BoostVoronoi.InvalidEmittedSegment] ordinal={i} first={segment.Point0} second={segment.Point1} "
                + $"completeSegments={rawOutput.Count} originalSignedRings=True decision=fail-complete-output");
            throw new InvalidOperationException($"The complete original-site medial output contains a nonfinite or negative-radius segment {i}; no sample or original edge may be omitted.");
        }
        cancellationToken.ThrowIfCancellationRequested();
        progress?.Invoke(1, "Medial paths ready", false);
        PerfLog.Stop("BoostVoronoiProvider.EmitSegments", emitAt, $"edges={walked.Count} segments={rawOutput.Count}");
        diagnostic?.Invoke($"stage=voronoi-emission event=complete completed={walked.Count} total={walked.Count} segments={rawOutput.Count} elapsedMs={Stopwatch.GetElapsedTime(emitAt).TotalMilliseconds:F2}");
        if (debugEnabled)
        {
            var (nodeCount, degree1Count, branchCount) = ComputeDegreeStats(rawOutput);
            Console.Error.WriteLine(
                string.Create(
                    CultureInfo.InvariantCulture,
                    $"[medial-axis][boost] emit walked={walked.Count} skippedNullVertices={skippedNullVertices} emitRejected={emitRejected} outSegments={rawOutput.Count} graphNodes={nodeCount} leafNodes={degree1Count} branchNodes={branchCount}"));
        }

        PerfLog.Stop(
            "BoostVoronoiProvider.RunPipeline",
            t0,
            $"edges={edgeCount} vertices={vertexCount} cells={cellCount} out={rawOutput.Count}");

        diagnostic?.Invoke($"stage=voronoi-construction cornerAngleRadians={filteringAngle:R} cornerRejected={cornerRejected} sourceSegmentTest=True");
        return rawOutput;
    }

    private static bool IsMedialDebugEnabled() =>
        string.Equals(
            Environment.GetEnvironmentVariable("RW_TOOLPATHS_MEDIAL_DEBUG"),
            "1",
            StringComparison.Ordinal);

    private static (int NodeCount, int Degree1Count, int BranchCount) ComputeDegreeStats(
        IReadOnlyList<MedialSegment> segments)
    {
        var degrees = new Dictionary<(long X, long Y), int>();

        static (long X, long Y) Key(MedialPoint point)
        {
            const double scale = 1_000_000.0;
            return (
                (long)Math.Round(point.X * scale, MidpointRounding.AwayFromZero),
                (long)Math.Round(point.Y * scale, MidpointRounding.AwayFromZero));
        }

        foreach (var segment in segments)
        {
            var k0 = Key(segment.Point0);
            var k1 = Key(segment.Point1);

            degrees[k0] = degrees.TryGetValue(k0, out int d0) ? d0 + 1 : 1;
            degrees[k1] = degrees.TryGetValue(k1, out int d1) ? d1 + 1 : 1;
        }

        int degree1 = 0;
        int branch = 0;
        foreach (var kv in degrees)
        {
            if (kv.Value == 1)
            {
                degree1++;
            }
            else if (kv.Value >= 3)
            {
                branch++;
            }
        }

        return (degrees.Count, degree1, branch);
    }

    // -- CellSite reconstruction -----------------------------------------------

    private static MedialAxisEdgeClassifier.CellSite MakeCellSite(
        BoostVoronoiInterop.BvCell cell,
        IReadOnlyList<SegmentSiteData> segs)
    {
        if (cell.ContainsPoint != 0)
        {
            var seg = segs[cell.SourceIndex];
            bool isStartPointCategory =
                cell.SourceCategory == BoostVoronoiInterop.SourceCategory.SegmentStartPoint;
            var pt = isStartPointCategory ? seg.Low : seg.High;
            return new MedialAxisEdgeClassifier.CellSite(true, pt, null);
        }
        else
        {
            var seg = segs[cell.SourceIndex];
            return new MedialAxisEdgeClassifier.CellSite(
                false, null,
                new MedialAxisEdgeClassifier.SegmentSite(seg.Low, seg.High));
        }
    }

    // -- Segment site building -------------------------------------------------

    /// <summary>
    /// Carries the raw A->B coordinates passed to Boost plus the Low/High
    /// endpoints in insertion order
    /// used to
    /// reconstruct <see cref="MedialAxisEdgeClassifier.CellSite"/> values.
    /// </summary>
    private readonly record struct SegmentSiteData(
        double Ax, double Ay, double Bx, double By,
        MedialAxisEdgeClassifier.EdgePoint Low,
        MedialAxisEdgeClassifier.EdgePoint High);

    private static List<SegmentSiteData> BuildSegmentSites(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        CancellationToken cancellationToken, out NativeSegmentArrangement.WideResult? wideArrangement)
    {
        wideArrangement = null;
        int estimated = boundary.Count;
        for (int i = 0; i < holes.Count; i++)
            estimated += holes[i].Count;

        var result = new List<SegmentSiteData>(estimated);
        AddRing(result, boundary);
        for (int i = 0; i < holes.Count; i++)
            AddRing(result, holes[i]);
        cancellationToken.ThrowIfCancellationRequested();
        if (Environment.GetEnvironmentVariable("RW_TOOLPATHS_SEGMENT_ARRANGEMENT_REFERENCE") == "1")
        {
            Console.WriteLine($"[BoostVoronoi.SegmentArrangement] mode=original-reference originalSites={result.Count} originalSignedRings=True");
            return result;
        }
        long arrangementAt = Stopwatch.GetTimestamp();
        var original = result;
        NativeSegmentArrangement.Result arrangement;
        try
        {
            arrangement = NativeSegmentArrangement.Prepare(original.Select(site => new NativeSegmentArrangement.Segment(
                new(checked((int)site.Ax), checked((int)site.Ay)),
                new(checked((int)site.Bx), checked((int)site.By)))).ToArray(), cancellationToken);
        }
        catch (NativeSegmentArrangement.UnsupportedContactException unsupported)
        {
            cancellationToken.ThrowIfCancellationRequested();
            Console.WriteLine($"[BoostVoronoi.SegmentArrangement] mode=exact-rational-retry reason={unsupported.Reason} "
                + $"firstOriginal={unsupported.FirstOriginal} secondOriginal={unsupported.SecondOriginal} originalSites={original.Count}");
            wideArrangement = NativeSegmentArrangement.PrepareWide(original.Select(site => new NativeSegmentArrangement.Segment(
                new(checked((int)site.Ax), checked((int)site.Ay)),
                new(checked((int)site.Bx), checked((int)site.By)))).ToArray(), cancellationToken);
            double scale = wideArrangement.Scale;
            return wideArrangement.Atoms.Select(atom => new SegmentSiteData(
                atom.A.X / scale, atom.A.Y / scale, atom.B.X / scale, atom.B.Y / scale,
                new(atom.A.X / scale, atom.A.Y / scale), new(atom.B.X / scale, atom.B.Y / scale))).ToList();
        }
        if (arrangement.Atoms is { } atoms)
        {
            result = new List<SegmentSiteData>(atoms.Count);
            foreach (var atom in atoms)
            {
                if ((result.Count & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
                var a = atom.Segment.A; var b = atom.Segment.B;
                var owner = original[atom.OriginalOrdinal];
                if (a.X == owner.Ax && a.Y == owner.Ay && b.X == owner.Bx && b.Y == owner.By)
                    result.Add(owner);
                else
                    result.Add(new(a.X, a.Y, b.X, b.Y, new(a.X, a.Y), new(b.X, b.Y)));
            }
        }
        cancellationToken.ThrowIfCancellationRequested();
        Console.WriteLine($"[BoostVoronoi.SegmentArrangement] mode=exact-integer-finite-union changed={arrangement.Changed} "
            + $"originalSites={arrangement.OriginalSites} atomicSites={arrangement.AtomicSites} "
            + $"splitOriginalSites={arrangement.SplitOriginalSites} duplicateAtoms={arrangement.DuplicateAtoms} "
            + $"candidatePairs={arrangement.CandidatePairs} nodeVisits={arrangement.NodeVisits} "
            + $"buildMs={arrangement.BuildMilliseconds:F2} queryMs={arrangement.QueryMilliseconds:F2} "
            + $"certificateMs={arrangement.CertificateMilliseconds:F2} elapsedMs={Stopwatch.GetElapsedTime(arrangementAt).TotalMilliseconds:F2} "
            + $"completeOriginalFiniteUnion=True originalSubchainsVerified=True originalOwnerMappings=True originalSignedRings=True geometryDisplacement=0");
        return result;

        static void AddRing(List<SegmentSiteData> list, IReadOnlyList<PointD> ring)
        {
            if (ring.Count < 2) return;
            for (int i = 0; i < ring.Count; i++)
            {
                var a = ring[i];
                var b = ring[(i + 1) % ring.Count];
                var low = new MedialAxisEdgeClassifier.EdgePoint(a.x, a.y);
                var high = new MedialAxisEdgeClassifier.EdgePoint(b.x, b.y);
                list.Add(new SegmentSiteData(a.x, a.y, b.x, b.y, low, high));
            }
        }
    }

    // -- Helpers ---------------------------------------------------------------

    private static List<PointD> RoundAndNormalizeRing(IReadOnlyList<PointD> ring)
    {
        var result = new List<PointD>(ring.Count);
        for (int i = 0; i < ring.Count; i++)
        {
            var p = ring[i];
            var rounded = new PointD(Math.Round(p.x), Math.Round(p.y));
            // Boolean residual boundaries can contain adjacent vertices that
            // quantize to one site. A zero-length segment is a point site in
            // Boost and otherwise corrupts segment/point cell reconstruction.
            if (result.Count == 0 || result[^1] != rounded) result.Add(rounded);
        }

        if (result.Count > 1
            && result[0].x == result[^1].x
            && result[0].y == result[^1].y)
            result.RemoveAt(result.Count - 1);

        return result;
    }
}
