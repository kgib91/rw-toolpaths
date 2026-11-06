using System.Runtime.CompilerServices;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Geometry;

[assembly: InternalsVisibleTo("RW.Toolpaths.Tests")]

namespace RW.Toolpaths;

// --- Public types -------------------------------------------------------------

/// <summary>
/// A single medial-axis segment with depth (radius) information.
///
/// source â€” the output of <c>kernel.construct_medial_axis</c> and <c>ridge-pass</c>.
/// </summary>
public readonly record struct MedialSegment(
    MedialPoint Point0,
    MedialPoint Point1);

/// <summary>
/// One endpoint of a medial-axis segment.
/// <see cref="Radius"/> is the distance to the nearest polygon edge â€”
/// i.e. the half-width of the V-groove at this point.
/// </summary>
public readonly record struct MedialPoint(double X, double Y, double Radius);

/// <summary>
/// A toolpath polyline plus routing metadata for downstream consumers.
/// </summary>
/// <param name="Points">Polyline points in tool travel order.</param>
/// <param name="RegionIndex">Index of the island/region that produced this path.</param>
/// <param name="Category">
///   Pass category, for example <c>"clearing"</c> or <c>"final-carve"</c>.
/// </param>
/// <param name="DepthPassIndex">
///   Deepest zero-based depth layer represented by this path. A continuous profile spiral may
///   contain several scheduled layers and is tagged with the terminal layer it reaches;
///   <c>null</c> identifies non-layered paths such as a final carve.
/// </param>
public readonly record struct TaggedToolpath
{
    public TaggedToolpath(
        IReadOnlyList<Point3D> points,
        int regionIndex,
        string category,
        int? depthPassIndex,
        int? visitIdentity = null)
        : this(
            points,
            ToolpathCurveDetector.DetectMoves(points),
            regionIndex,
            category,
            depthPassIndex,
            visitIdentity)
    {
    }

    public TaggedToolpath(
        IReadOnlyList<Point3D> points,
        IReadOnlyList<ToolMove> moves,
        int regionIndex,
        string category,
        int? depthPassIndex,
        int? visitIdentity = null)
    {
        Points = points;
        Moves = moves;
        RegionIndex = regionIndex;
        Category = category;
        DepthPassIndex = depthPassIndex;
        VisitIdentity = visitIdentity;
        _spans = null;
        DetectCurves = true;
    }

    public TaggedToolpath(
        IReadOnlyList<Point3D> points,
        IReadOnlyList<ToolpathSpan> spans,
        int regionIndex,
        string category,
        int? depthPassIndex,
        int? visitIdentity = null,
        bool detectCurves = true)
    {
        Points = points;
        Moves = detectCurves ? ToolpathCurveDetector.DetectMoves(points)
            : new LineToolMoves(points);
        RegionIndex = regionIndex;
        Category = category;
        DepthPassIndex = depthPassIndex;
        VisitIdentity = visitIdentity;
        _spans = spans;
        DetectCurves = detectCurves;
    }

    private readonly IReadOnlyList<ToolpathSpan>? _spans;
    internal bool DetectCurves { get; }
    // A point-emitting owner can omit optional move fitting without granting
    // source-detour or reduction eligibility formerly blocked by DetectCurves.
    internal bool PreserveSourceTraversal { get; init; } = false;
    internal bool RequiresSourcePreservation=>DetectCurves||PreserveSourceTraversal;
    /// <summary>Transient execution component, proved before depth scheduling.
    /// Source region, cut category and visit identity retain their own meanings.</summary>
    internal int? ContinuationComponentIndex { get; init; } = null;
    internal ToolpathContinuationComponents.Plan? ContinuationPlan {get;init;}=null;
    internal int? ContinuationSourceOrdinal {get;init;}=null;
    internal bool ContinuationCircuit {get;init;}=false;
    internal ToolpathCircuitBuilder.Definition? ContinuationCircuitDefinition {get;init;}=null;
    internal VCarveRingOrder.Visit? ConcentricSource {get;init;}=null;

    public IReadOnlyList<Point3D> Points { get; init; }

    /// <summary>
    /// Curve-aware moves derived from <see cref="Points"/>.  Lines, arcs, and
    /// cubic Beziers are emitted only when their measured fit error is within
    /// <see cref="ToolpathCurveDetector.DefaultTolerance"/>.
    /// </summary>
    public IReadOnlyList<ToolMove> Moves { get; init; }

    /// <summary>
    /// Classification of each run of <see cref="Points"/> as cutting, ramping, lifted over a tab
    /// and so on. Empty when the producing strategy does not distinguish them.
    /// </summary>
    public IReadOnlyList<ToolpathSpan> Spans
    {
        get => _spans ?? Array.Empty<ToolpathSpan>();
        init => _spans = value;
    }

    public int RegionIndex { get; init; }
    public string Category { get; init; }
    public int? DepthPassIndex { get; init; }

    /// <summary>
    /// Source contour/depth visit shared by fragments created when a routed carrier pauses for
    /// an excursion. <c>null</c> means this path is an independent visit.
    /// </summary>
    public int? VisitIdentity { get; init; }
}

/// <summary>
/// Provides a straight-skeleton / medial axis for a polygon.
/// Implement this interface with any .NET Voronoi-of-segments library.
/// </summary>
public interface IMedialAxisProvider
{
    /// <summary>
    /// Computes the medial axis (straight skeleton) of a polygon.
    /// </summary>
    /// <param name="boundary">
    ///   Outer boundary vertices in order (CCW, workspace coordinates).
    /// </param>
    /// <param name="holes">
    ///   Optional hole polygons (CW, workspace coordinates).
    /// </param>
    /// <param name="tolerance">
    ///   Parabola discretisation tolerance.
    /// </param>
    /// <param name="maxRadius">
    ///   Maximum medial-axis radius (= half tool-width).  Segments whose
    ///   inscribed-circle radius exceeds this are omitted.
    /// </param>
    /// <param name="filteringAngle">
    ///   Maximum angle of two segment sites meeting at one input corner.
    ///   PI retains the complete inner graph; ordinary construction requests 3PI/4.
    /// </param>
    /// <param name="useBigIntegers">
    ///   When <c>true</c>, the Voronoi computation uses 128-bit integer
    ///   arithmetic for higher precision.
    /// </param>
    /// <returns>
    ///   All inner medial-axis segments.  Each segment carries the
    ///   inscribed-circle radius at each endpoint.
    /// </returns>
    IReadOnlyList<MedialSegment> ConstructMedialAxis(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance,
        double maxRadius,
        double filteringAngle = Math.PI,
        bool   useBigIntegers = true);

    /// <summary>Cancelable, observable overload; older providers remain compatible.</summary>
    IReadOnlyList<MedialSegment> ConstructMedialAxis(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance,
        double maxRadius,
        double filteringAngle,
        bool useBigIntegers,
        CancellationToken cancellationToken,
        Action<string>? diagnostic = null,
        Action<double, string, bool>? progress = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        diagnostic?.Invoke("stage=voronoi-construction event=start provider-progress=opaque");
        progress?.Invoke(0, "Computing medial axis...", true);
        var result = ConstructMedialAxis(boundary, holes, tolerance, maxRadius, filteringAngle, useBigIntegers);
        cancellationToken.ThrowIfCancellationRequested();
        progress?.Invoke(1, "Medial axis complete", false);
        return result;
    }
}

// --- Main algorithm -----------------------------------------------------------

/// <summary>
/// V-carve medial-axis toolpath generator.
///
/// <para>
/// This is a 1-to-1 C# port of the V-carve pipeline in
/// <list type="bullet">
///   <item><c>contour-pass</c> â€” generateSimplifiedLayer</item>
///   <item><c>ridge-pass</c> â€” generateMedialAxis</item>
///   <item><c>plan-pass</c> â€” generateMedialAxisToolpaths</item>
///   <item><c>clip-pass</c> â€” trimSegment</item>
///   <item>flat-fill pass â€” generateFlatAreaFill</item>
///   <item><c>carve-main</c> â€” the public V-carve entry point</item>
/// </list>
/// </para>
///
/// <para>
/// <b>External dependency</b>: the medial axis itself requires a
/// Inject an <see cref="IMedialAxisProvider"/> to supply this.
/// </para>
/// </summary>
public static class MedialAxisToolpaths
{
    // --- Scale constants -----------------------------------------------------
    //
    // The algorithm retries with doubled scale up to 4 times if the
    // radius-to-bounding-box ratio exceeds 0.55.

    private const double InitialScale = 4_096.0;

    /// <summary>
    /// Generates raw medial-axis segments for visualization/debugging.
    ///
    /// Retains the unconditioned diagnostic contract with XY-space radii.
    /// Ordinary nominal construction is owned by VCarveMedialConstruction;
    /// this raw diagnostic does not apply that preparation policy.
    /// </summary>
    public static IReadOnlyList<MedialSegment> GenerateRawMedialAxisSegments(
        IMedialAxisProvider provider,
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double radianTipAngle,
        double depthPerPass,
        double tolerance = 0.03,
        double? maxRadiusOverride = null,
        double? bottomRadiusOverride = null,
        double? topRadiusOverride = null,
        double? coneLengthOverride = null)
    {
        if (provider is null) throw new ArgumentNullException(nameof(provider));

        var normalizedBoundary = NormalizeBoundaryRings(boundary);
        if (normalizedBoundary.Count == 0)
            return Array.Empty<MedialSegment>();

        var profile = ResolveToolProfile(
            startDepth: 0.0,
            endDepth: Math.Max(depthPerPass, 0.0),
            radianTipAngle,
            depthPerPass,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride);

        double maxRadius = maxRadiusOverride ?? profile.PassRadius;
        double scale = InitialScale;

        List<List<PointD>>? quantizedLayer = null;
        IReadOnlyList<MedialSegment> raw = Array.Empty<MedialSegment>();

        for (int attempt = 0; attempt < 4; attempt++)
        {
            quantizedLayer = QuantizeMedialBoundary(normalizedBoundary, scale);
            if (quantizedLayer is null || quantizedLayer.Count == 0)
                return Array.Empty<MedialSegment>();

            raw = GenerateMedialAxis(
                provider,
                quantizedLayer,
                maxRadius,
                scale,
                tolerance,
                CancellationToken.None);

            // For preview mode with explicit override, skip bbox-breakout filtering
            // and return what the kernel produced.
            if (maxRadiusOverride is not null)
                return raw;

            double maxR = 0;
            foreach (var seg in raw)
                maxR = Math.Max(maxR, Math.Max(seg.Point0.Radius, seg.Point1.Radius));

            var bb = BoundingBox(quantizedLayer.SelectMany(r => r));
            double minor = Math.Min(bb.maxX - bb.minX, bb.maxY - bb.minY);
            if (maxR < minor * 0.55)
                return raw;

            scale *= 2;
        }

        return raw;
    }

    // --- Public entry point ---------------------------------------------------

    /// <summary>
    /// Generates V-carve toolpaths for one pocket region.
    ///
    /// </summary>
    /// <param name="provider">Medial-axis provider (Voronoi of segments).</param>
    /// <param name="boundary">
    ///   Closed polygon rings that define the V-carve region.
    ///   First element = outer boundary; subsequent elements = holes.
    /// </param>
    /// <param name="startDepth">
    ///   Z of the top of the region (typically 0 or a previous pass depth).
    ///   Positive = above material surface.
    /// </param>
    /// <param name="endDepth">
    ///   Maximum cut depth (positive = deeper).
    /// </param>
    /// <param name="radianTipAngle">
    ///   Full included tip angle of the V-bit in radians.
    ///   Use <c>bit.radianTipAngle()</c>.
    /// </param>
    /// <param name="depthPerPass">
    ///   Maximum depth of each V-carve roughing pass.
    /// </param>
    /// <returns>
    ///   A list of continuous toolpath polylines, or <c>null</c> if the
    ///   region is too small / degenerate for V-carving.
    /// </returns>
    // Default RDP simplification tolerance (in workspace units = inches).
    // Keep disabled by default to avoid collapsing valid corner spokes.
    private const double DefaultRdpTolerance = 0.0;
    // Preserve corners with at least this XY turn before running RDP.
    private const double DefaultRdpCornerAngleDeg = 12.0;

    public static List<List<Point3D>>? Generate(
        IMedialAxisProvider provider,
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double tolerance = 0.03,
        double rdpTolerance = DefaultRdpTolerance,
        double? bottomRadiusOverride = null,
        double? topRadiusOverride = null,
        double? coneLengthOverride = null,
        CancellationToken cancellationToken = default,
        bool includeInteriorFill = true,
        Action<string>? diagnostic = null,
        VCarveCutterProfile? cutterProfile = null,
        Action<double, string, bool>? progress = null,
        double outputSurfaceZ = 0,
        IReadOnlyList<MedialSegment>? preparedMedialGraph = null)
        => GenerateCore(provider, boundary, startDepth, endDepth, radianTipAngle, depthPerPass,
            tolerance, rdpTolerance, bottomRadiusOverride, topRadiusOverride, coneLengthOverride,
            cancellationToken, includeInteriorFill, diagnostic, cutterProfile, progress, outputSurfaceZ,
            preparedMedialGraph, null);

    private static List<List<Point3D>>? GenerateCore(
        IMedialAxisProvider provider,
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double tolerance,
        double rdpTolerance,
        double? bottomRadiusOverride,
        double? topRadiusOverride,
        double? coneLengthOverride,
        CancellationToken cancellationToken = default,
        bool includeInteriorFill = true,
        Action<string>? diagnostic = null,
        VCarveCutterProfile? cutterProfile = null,
        Action<double, string, bool>? progress = null,
        double outputSurfaceZ = 0,
        IReadOnlyList<MedialSegment>? preparedMedialGraph = null,
        CertifiedInsetReuse? insetReuse = null,
        Func<double,double,VCarveMaterialSurface.ClearingLayer?>? materialClearingInset = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        diagnostic?.Invoke("stage=medial-axis event=start");
        progress?.Invoke(0, "Computing medial detail...", true);
        double endZ   = -endDepth;
        var    result = GenerateMedialAxisToolpaths(
            provider,
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            tolerance,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride,
            cancellationToken, cutterProfile, diagnostic,
            (fraction, stage, indeterminate) => progress?.Invoke(4.0 / 7 * fraction, stage, indeterminate),
            outputSurfaceZ, preparedMedialGraph, insetReuse, includeInteriorFill, materialClearingInset);

        if (result is null)
        {
            diagnostic?.Invoke("stage=medial-axis event=complete result=empty");
            return null;
        }
        var (medialAxis, flatAreaRegion) = result.Value;
        diagnostic?.Invoke($"stage=medial-axis event=complete segments={medialAxis.Count} flatRings={flatAreaRegion.Count}");

        // Trim segments to final depth and collect flat-area strokes
        diagnostic?.Invoke("stage=trim-and-flat-fill event=start");
        progress?.Invoke(4.0 / 7, "Trimming medial detail...", true);
        var flatStrokes  = new List<(PointD A, PointD B)>();
        var trimmed      = medialAxis
            .Select(seg => TrimSegment(seg, endZ, flatStrokes))
            .Where(s => s is not null)
            .Select(s => s!.Value)
            .ToList();

        if (includeInteriorFill)
        {
            // Join adjacent flat-area segments and fill the flat bottom area.
            var joinedFlat = JoinSegments(flatStrokes);
            var flatFill = GenerateFlatAreaFill(joinedFlat, flatAreaRegion, endZ,
                cancellationToken, diagnostic,
                (fraction, stage, indeterminate) => progress?.Invoke(4.0 / 7 + 1.5 / 7 * fraction, stage, indeterminate));
            trimmed.AddRange(flatFill);
        }
        diagnostic?.Invoke($"stage=trim-and-flat-fill event=complete interiorFill={includeInteriorFill} " +
            $"flatStrokes={flatStrokes.Count} resultingSegments={trimmed.Count}");

        // Connect segments into continuous polylines.
        diagnostic?.Invoke("stage=join event=start");
        progress?.Invoke(5.5 / 7, "Connecting medial detail...", true);
        var polylines = JoinIntoPolylines(trimmed);
        if (polylines is null)
        {
            diagnostic?.Invoke("stage=join event=complete result=empty");
            return null;
        }
        diagnostic?.Invoke($"stage=join event=complete paths={polylines.Count} points={polylines.Sum(path => (long)path.Count)}");

        if (rdpTolerance > 0)
        {
            for (int i = 0; i < polylines.Count; i++)
                polylines[i] = RdpSimplify3D(polylines[i], rdpTolerance);
        }

        // Re-derive Z analytically along the (possibly simplified) XY paths so the
        // V-cone touches the original boundary at every point — not just at the
        // surviving vertices.  Recomputes Z at vertices and bisects any chord
        // whose linear-Z interpolation drifts from the analytical depth curve
        // by more than zTolerance.  See EnsureZFidelityFromBoundary.
        diagnostic?.Invoke("stage=depth-fidelity event=start");
        var zProfile = ResolveToolProfile(
            startDepth, endDepth, radianTipAngle, depthPerPass,
            bottomRadiusOverride, topRadiusOverride, coneLengthOverride);
        var boundarySegs = BuildBoundarySegments(NormalizeBoundaryRings(boundary));
        double zTolerance = Math.Max(tolerance * 0.1, 1e-6);
        var spatialIndex = new SpatialSegmentIndex(boundarySegs);
        long sourcePointCount = polylines.Sum(path => (long)path.Count);
        long lastProgress = 0;
        for (int i = 0; i < polylines.Count; i++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            long now = Environment.TickCount64;
            if (i == 0 || now - lastProgress >= 100)
            {
                progress?.Invoke(5.5 / 7 + 1.5 / 7 * i / polylines.Count,
                    $"Refining depth: path {i + 1}/{polylines.Count}", false);
                lastProgress = now;
            }
            polylines[i] = EnsureZFidelityFromBoundary(
                polylines[i],
                spatialIndex,
                startDepth,
                zProfile.BottomRadius,
                Math.Max(1e-9, zProfile.RadiusSlope),
                endZ,
                zTolerance, cutterProfile, diagnostic);
        }
        diagnostic?.Invoke($"stage=depth-fidelity event=complete paths={polylines.Count} " +
            $"sourcePoints={sourcePointCount} outputPoints={polylines.Sum(path => (long)path.Count)} " +
            $"boundarySegments={boundarySegs.Count} zTolerance={zTolerance:R}");
        spatialIndex.LogDiagnostics(diagnostic);
        progress?.Invoke(1, "Medial detail complete", false);
        return polylines;
    }

    // --- generateToolpathsForPockets (clear-main) ------------------------------------

    /// <summary>
    /// Generates OffsetFill clearing passes for a V-carve pocket at each depth
    /// layer, using the V-bit geometry to determine the reachable area at each
    /// depth.
    ///
    /// Port of <c>clear-main(n, t, e)</c> for standard V-carve (non-roughing, non-detail
    /// passType).
    ///
    /// <para>
    /// The V-bit's effective cutting width at depth <c>k</c> is
    /// <c>2 * (k - startDepth) * tan(tipAngle/2)</c>.  The reachable region at
    /// each depth is the original boundary contracted by that amount.  Each
    /// layer is filled with concentric OffsetFill rings at
    /// <c>stepOver Ã— bitDiameter</c> spacing.
    /// </para>
    /// </summary>
    /// <param name="boundary">
    ///   Closed polygon rings (outer boundary first, then holes).
    /// </param>
    /// <param name="startDepth">Top of the region (usually 0).</param>
    /// <param name="endDepth">Maximum cut depth (positive = deeper).</param>
    /// <param name="radianTipAngle">Full included V-bit angle in radians.</param>
    /// <param name="depthPerPass">Maximum depth increment per pass.</param>
    /// <param name="stepOver">
    ///   Step-over fraction of effective bit diameter. Default 0.4.
    /// </param>
    /// <param name="includeInteriorFill">
    ///   When <c>false</c>, retains each depth layer's perimeter and limits additional
    ///   ring centers to <paramref name="edgeFillWidth"/> from the supplied boundary.
    ///   The cutoff follows the existing polygon normalization and offset tolerances.
    /// </param>
    public static List<List<Point3D>> GenerateClearingPasses(
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double stepOver = 0.4,
        double? bottomRadiusOverride = null,
        double? topRadiusOverride = null,
        double? coneLengthOverride = null,
        bool includeInteriorFill = true,
        VCarveCutterProfile? cutterProfile = null,
        double edgeFillWidth = 0,
        Action<double, string, bool>? progress = null,
        double outputSurfaceZ = 0)
    {
        return GenerateClearingPassGeometries(
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride,
            includeInteriorFill, cutterProfile: cutterProfile, edgeFillWidth: edgeFillWidth, progress: progress,
            outputSurfaceZ: outputSurfaceZ)
            .Select(pass => pass.Points)
            .ToList();
    }

    /// <summary>
    /// Same as <see cref="GenerateClearingPasses"/>, but includes metadata tags
    /// for region routing and depth-layer ordering.
    /// </summary>
    public static List<TaggedToolpath> GenerateClearingPassesTagged(
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double stepOver = 0.4,
        int regionIndex = 0,
        double? bottomRadiusOverride = null,
        double? topRadiusOverride = null,
        double? coneLengthOverride = null,
        bool includeInteriorFill = true,
        bool detectCurves = true,
        Action<string>? diagnostic = null,
        VCarveCutterProfile? cutterProfile = null,
        CancellationToken cancellationToken = default,
        double edgeFillWidth = 0,
        Action<double, string, bool>? progress = null,
        double outputSurfaceZ = 0,
        int skipDepthPasses = 0,
        double startingDepth = 0,
        double excludedDiscRadius = 0,
        IReadOnlyList<VCarveRoughingChord>? roughingChords = null)
        => GenerateClearingPassesTaggedCore(boundary, startDepth, endDepth, radianTipAngle,
            depthPerPass, stepOver, regionIndex, bottomRadiusOverride, topRadiusOverride,
            coneLengthOverride, includeInteriorFill, detectCurves, diagnostic, cutterProfile,
            cancellationToken, edgeFillWidth, progress, outputSurfaceZ, skipDepthPasses, startingDepth, null,
            excludedDiscRadius: excludedDiscRadius,
            restFillStock:excludedDiscRadius>0 ? new VCarvePostedFloorSweep(roughingChords??Array.Empty<VCarveRoughingChord>()) : null);

    private static List<TaggedToolpath> GenerateClearingPassesTaggedCore(
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double stepOver,
        int regionIndex,
        double? bottomRadiusOverride,
        double? topRadiusOverride,
        double? coneLengthOverride,
        bool includeInteriorFill = true,
        bool detectCurves = true,
        Action<string>? diagnostic = null,
        VCarveCutterProfile? cutterProfile = null,
        CancellationToken cancellationToken = default,
        double edgeFillWidth = 0,
        Action<double, string, bool>? progress = null,
        double outputSurfaceZ = 0,
        int skipDepthPasses = 0,
        double startingDepth = 0,
        CertifiedInsetReuse? insetReuse = null,
        RW.Toolpaths.Milling.Acceleration.RawInitialClearingInsetNomination? initialInsetNomination = null,
        bool retainRawFirst = true, bool retainRawCore = true, bool retainRawMiddle = true,
        Action<bool>? selectedRawRolesCompleted = null,
        bool concentricClearingOnly = false,
        double excludedDiscRadius = 0,
        VCarvePostedFloorSweep? restFillStock = null,
        Func<double,double,VCarveMaterialSurface.ClearingLayer?>? materialClearingInset = null)
    {
        var geometries = GenerateClearingPassGeometries(
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride,
            includeInteriorFill,
            diagnostic, cutterProfile, cancellationToken, edgeFillWidth, progress, outputSurfaceZ, skipDepthPasses, startingDepth, insetReuse, initialInsetNomination,
            retainRawFirst, retainRawCore, retainRawMiddle, selectedRawRolesCompleted, concentricClearingOnly, excludedDiscRadius, restFillStock, materialClearingInset);

        var result = new List<TaggedToolpath>(geometries.Count);
        foreach (var (points, depthPassIndex) in geometries)
        {
            bool closed = points.Count > 1 && points[0].X == points[^1].X
                && points[0].Y == points[^1].Y && points[0].Z == points[^1].Z;
            result.Add(detectCurves
                ? new TaggedToolpath(points, ToolpathCurveDetector.DetectMoves(points, closePath: closed),
                    regionIndex, "clearing", depthPassIndex)
                : new TaggedToolpath(points, Array.Empty<ToolpathSpan>(),
                    regionIndex, "clearing", depthPassIndex, detectCurves: false));
        }

        return result;
    }

    private static List<(List<Point3D> Points, int DepthPassIndex)> GenerateClearingPassGeometries(
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double stepOver,
        double? bottomRadiusOverride,
        double? topRadiusOverride,
        double? coneLengthOverride,
        bool includeInteriorFill,
        Action<string>? diagnostic = null,
        VCarveCutterProfile? cutterProfile = null,
        CancellationToken cancellationToken = default,
        double edgeFillWidth = 0,
        Action<double, string, bool>? progress = null,
        double outputSurfaceZ = 0,
        int skipDepthPasses = 0,
        double startingDepth = 0,
        CertifiedInsetReuse? insetReuse = null,
        RW.Toolpaths.Milling.Acceleration.RawInitialClearingInsetNomination? initialInsetNomination = null,
        bool retainRawFirst = true, bool retainRawCore = true, bool retainRawMiddle = true,
        Action<bool>? selectedRawRolesCompleted = null,
        bool concentricClearingOnly = false,
        double excludedDiscRadius = 0,
        VCarvePostedFloorSweep? restFillStock = null,
        Func<double,double,VCarveMaterialSurface.ClearingLayer?>? materialClearingInset = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        ArgumentOutOfRangeException.ThrowIfNegative(skipDepthPasses);
        if (!double.IsFinite(startingDepth) || startingDepth < 0 || startingDepth > endDepth)
            throw new ArgumentOutOfRangeException(nameof(startingDepth));
        if (!double.IsFinite(edgeFillWidth) || edgeFillWidth < 0)
            throw new ArgumentOutOfRangeException(nameof(edgeFillWidth));
        if (endDepth <= startDepth) return new();

        var normalizedBoundary = NormalizeBoundaryRings(boundary);
        if (normalizedBoundary.Count == 0)
            return new();

        // Clearing passes should use the canonical boundary directly; the
        // contour simplification step is for medial-axis stability, not pocket
        // offset fidelity.
        var clearingBoundary = normalizedBoundary;
        var discExclusion = excludedDiscRadius > 0 && (includeInteriorFill||edgeFillWidth>0)
            ? new VCarveDiscSweepExclusion(clearingBoundary,excludedDiscRadius,cancellationToken,diagnostic,"oriented-layer-boundary") : null;

        var profile = ResolveToolProfile(
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride);

        double finalRadius = cutterProfile?.RadiusAtDepth(endDepth - startDepth) ?? profile.FinalRadius;
        double finalCommandRadius = CommandedCuttingRadius(cutterProfile, profile.BottomRadius,
            profile.RadiusSlope, startDepth, endDepth, outputSurfaceZ, coneLengthOverride);
        double bitDiameter = 2.0 * finalRadius;
        if (bitDiameter <= 0) return new();

        double T = stepOver * bitDiameter;
        double M = 0.05 * bitDiameter;

        // startDepth is the physical geometry origin. The independently authored
        // cleared floor controls scheduling only; it must not shrink the cutter's
        // footprint against the unchanged original stock boundary.
        double firstDepth = startingDepth > 0 ? Math.Max(startDepth, startingDepth)
            : Math.Floor(startDepth / depthPerPass) * depthPerPass;
        int nominalLayers = Math.Max(1, (int)Math.Ceiling((endDepth - firstDepth) / depthPerPass - 1e-9));
        int effectiveSkipped = Math.Min(skipDepthPasses, nominalLayers - 1);
        diagnostic?.Invoke($"stage=depth-schedule nominalPasses={nominalLayers} " +
            $"startingDepth={startingDepth:R} physicalGeometryOrigin={startDepth:R} " +
            $"skipDepthPasses={skipDepthPasses} effectiveSkipped={effectiveSkipped} " +
            $"remainingPasses={nominalLayers - effectiveSkipped} finalTargetRetained=True " +
            $"firstDepth={Math.Min(endDepth, firstDepth + (effectiveSkipped + 1d) * depthPerPass):R}");

        // layers: (z, zTop, rings, isBottomLayer, effectiveRadiusAtLayer)
        var layers = new List<(double z, double zTop, List<List<PointD>> rings, bool isBottom,
            double layerRadius, double nominalRadius, int nominalPassIndex)>();

        double N = firstDepth;
        double k = Math.Min(endDepth, firstDepth + (effectiveSkipped + 1d) * depthPerPass);
        double b = endDepth;
        int nominalPassIndex = effectiveSkipped;


        const double Eps = 1e-5;
        bool CloseTo(double a, double x) => Math.Abs(a - x) < Eps;
        int plannedLayers = nominalLayers - effectiveSkipped;
        int preparationIndex = 0;
        var materialRadii=new Dictionary<double,double>();

        HashSet<double>? rawNominationDepths = initialInsetNomination?.CollectorNominationCaptured == true ? new() : null;

        List<List<PointD>> ContractLayer(double layerDepth, double radius)
        {
            cancellationToken.ThrowIfCancellationRequested();
            progress?.Invoke(.25 * preparationIndex / plannedLayers,
                $"Preparing clearing pass {++preparationIndex}/{plannedLayers} at {layerDepth:0.###} mm", true);
            double commandedDepth = CommandedMachineDepth(layerDepth, outputSurfaceZ);
            double nominalRadius = cutterProfile?.RadiusAtDepth(Math.Max(0, layerDepth - startDepth))
                ?? profile.BottomRadius + profile.RadiusSlope * Math.Max(0, layerDepth - startDepth);
            diagnostic?.Invoke($"stage=clearing-inset event=start depth={layerDepth:R} " +
                $"commandedDepth={commandedDepth:R} radius={radius:R} nominalRadius={nominalRadius:R} " +
                $"surfaceZ={outputSurfaceZ:R} zDecimals=3 sourceRings={clearingBoundary.Count}");
            List<List<PointD>> rings;
            var contact=materialClearingInset?.Invoke(layerDepth,radius);
            if(contact is not null)
            {
                rings=contact.Rings;
                materialRadii.Add(layerDepth,contact.MinimumRadius);
            }
            else if(initialInsetNomination is not null&&initialInsetNomination.TryNominate(clearingBoundary,radius,cancellationToken,diagnostic,out var rawNomination,out string rawReason))
            {
                rings=rawNomination!;
                rawNominationDepths?.Add(layerDepth);
            }
            else
                rings=Contract(clearingBoundary,radius,cancellationToken,insetReuse);
            diagnostic?.Invoke($"stage=clearing-inset event=complete depth={layerDepth:R} " +
                $"radius={radius:R} resultRings={rings.Count} " +
                $"result={(rings.Count == 0 ? "unreachable-at-depth" : "reachable")}");
            return rings;
        }

        // Intermediate depth layers (depths strictly before endDepth)
        while (k < b && !CloseTo(k, b))
        {
            double depthFromStart = Math.Max(0.0, k - startDepth);
            double nominalRadius = cutterProfile?.RadiusAtDepth(depthFromStart)
                ?? (profile.BottomRadius + profile.RadiusSlope * depthFromStart);
            double contractAmount = CommandedCuttingRadius(cutterProfile, profile.BottomRadius,
                profile.RadiusSlope, startDepth, k, outputSurfaceZ, coneLengthOverride);
            var layer = ContractLayer(k, contractAmount);
            if (layer.Count > 0)
                layers.Add((-k, -N, layer, false, materialRadii.GetValueOrDefault(k,contractAmount), nominalRadius, nominalPassIndex));

            N = k;
            k += depthPerPass;
            nominalPassIndex++;
        }

        // Bottom layer (k â‰ˆ endDepth after loop)
        // Use the original boundary contracted to the full-depth cutter radius.
        var contracted       = ContractLayer(endDepth, finalCommandRadius);

        if (contracted.Count > 0)
        {
            // A deeper erosion is already a subset of every shallower erosion.
            // Intersecting independently rounded offsets adds seam lobes without
            // adding a cutter-clearance constraint.
            var bt = contracted;

            if (CloseTo(k, b))
            {
                // Normal case: final layer is exactly at endDepth.
                // Only the contracted core (bt) is safe to mill: the V-bit cutting
                // radius at endDepth == profile.FinalRadius, so any tool-centre path
                // closer to the boundary than that would cut outside the glyph.
                if (bt.Count > 0)
                    layers.Add((-k, -N, bt, true, materialRadii.GetValueOrDefault(endDepth,finalCommandRadius), finalRadius, nominalPassIndex));
            }
            else if (bt.Count > 0)
            {
                // Overshot endDepth slightly â€” just add bottom layer.
                layers.Add((-b, -N, bt, true, materialRadii.GetValueOrDefault(endDepth,finalCommandRadius), finalRadius, nominalPassIndex));
            }
        }

        // Generate OffsetFill toolpaths for each layer
        var result = new List<(List<Point3D> Points, int DepthPassIndex)>();
        List<List<PointD>>? edgeCore = null;
        bool rawBandNominated = false;
        if (!includeInteriorFill && edgeFillWidth > 0 && layers.Any(layer => edgeFillWidth > layer.layerRadius))
        {
            progress?.Invoke(.25, "Preparing edge clearing boundary...", true);
            // Width is an authored geometric limit, not another cutter radius.
            // Its terminal trace is validated at each layer's actual radius below.
            double proofRadius = layers.Where(layer => edgeFillWidth > layer.layerRadius)
                .Max(layer => layer.layerRadius);
            if(initialInsetNomination is not null&&initialInsetNomination.TryNominateAuthoredBand(
                clearingBoundary,edgeFillWidth,proofRadius,cancellationToken,diagnostic,out var rawBand,out _))
            {
                edgeCore=rawBand!;
                rawBandNominated=true;
            }
            else
            {
                var coreFronts = PolygonOps.DifferenceToComponents(clearingBoundary, Array.Empty<IReadOnlyList<PointD>>())
                    .SelectMany(component => PolygonOps.PrepareBoundaryOffset(component.Outer, component.Holes,
                        arcTolerance: 1 / PathUtils.Scale, simplifyTolerance: 0,
                        sourceConstructionSweepRadius: proofRadius > 0 ? proofRadius : null,
                        cancellationToken: cancellationToken)
                        .Execute(-edgeFillWidth, cancellationToken));
                // This ONE canonical authored-inset front owns both subsequent
                // centerline clipping and the collector's maximum-inset contour.
                // Fold removal retains original-width clearance and physical proof.
                edgeCore = PolygonOps.Union(coreFronts);
            }
            diagnostic?.Invoke($"stage=clearing-band event=prepared origin=original-boundary " +
                $"width={edgeFillWidth:R} coreRings={edgeCore.Count} " +
                $"physicalProofRadius={proofRadius:R} canonicalCoreOwner={(rawBandNominated?"shared-RAW-authored-band-NOM":"shared-prepared-offset")} " +
                $"rawBandNominated={rawBandNominated} authoredWidthUnchanged=True terminalSharesSameCore={rawBandNominated} " +
                $"physicalCredit={!rawBandNominated} downstreamOriginalProtectRequired={rawBandNominated}");
        }
        int depthPassIndex = 0;
        foreach (var (z, zTop, rings, isBottom, layerRadius, nominalRadius, originalPassIndex) in layers)
        {
            cancellationToken.ThrowIfCancellationRequested();
            double layerDiameter = Math.Max(0.0, 2.0 * nominalRadius);
            double normalStepOver = stepOver * layerDiameter;
            double finishStepOver = 0.05 * layerDiameter;

            // A rounded nose sets the floor scallop spacing even when its taper has
            // a much larger footprint at the stock surface. Shallow cuts use only
            // the engaged portion of that nose.
            double stepOverDist = cutterProfile?.RoundedTip == true
                ? cutterProfile.NoseLimitedPitch(layerDiameter,stepOver)
                : isBottom ? finishStepOver : normalStepOver;
            if (stepOverDist <= 0)
                stepOverDist = isBottom ? M : T;
            if (stepOverDist <= 0)
                stepOverDist = T;

            diagnostic?.Invoke($"stage=clearing-fill event=start depth={-z:R} radius={layerRadius:R} nominalRadius={nominalRadius:R} " +
                $"pitch={stepOverDist:R} stepOver={stepOver:R} noseDiameter={cutterProfile?.ClearingDiameter ?? 0:R} " +
                $"roundedTip={cutterProfile?.RoundedTip ?? false} bottomLayer={isBottom} interiorFill={includeInteriorFill} " +
                $"edgeFillWidth={edgeFillWidth:R} edgeFillOrigin=original-boundary");
            double maximumInset = includeInteriorFill ? double.PositiveInfinity
                : Math.Max(0, edgeFillWidth - layerRadius);
            int maximumIterations = includeInteriorFill || maximumInset > 0 ? 1_000 : 0;
            bool hasEdgeCore = maximumInset > 0 && edgeCore is { Count: > 0 };
            double fillPitch = stepOverDist;
            List<List<PointD>>? terminalCore = null;
            if (hasEdgeCore)
            {
                // The authored last front belongs to the common collector. It is
                // certified against preceding fronts there, so adaptive spacing
                // can add coherent contours before reaching this exact cutoff.
                // Never Boolean-clip a nearly coincident independently rounded
                // last contour against the canonical edge core.
                fillPitch=ConcentricBandSchedule.FitPitch(maximumInset,stepOverDist,maximumIterations);
                // RAW band candidate is shared literally: it has no physical or
                // filled-set credit before original Smooth Protect. The old
                // source-index/chord constructor remains the whole fallback.
                terminalCore = materialRadii.ContainsKey(-z) ? PolygonOps.Intersect(edgeCore!,rings) : rawBandNominated ? edgeCore
                    : PolygonOps.CutterInset(clearingBoundary, layerRadius, edgeCore!, cancellationToken);
            }
            diagnostic?.Invoke($"stage=clearing-band event=limit depth={-z:R} origin=original-boundary " +
                $"width={edgeFillWidth:R} perimeterInset={layerRadius:R} maximumAdditionalInset={maximumInset:R} " +
                $"ringLimit={maximumIterations} requestedRings={(double.IsFinite(maximumInset) ? Math.Ceiling(maximumInset / stepOverDist) : double.PositiveInfinity):R} " +
                $"fillPitch={fillPitch:R} terminalCoreContour={terminalCore is not null} " +
                $"terminalFrontOwner=shared-collector mandatoryPerimeter=true");
            void ReportRing(int ring, double inset)
            {
                bool measured = double.IsFinite(maximumInset) && maximumInset > 0;
                double local = measured ? Math.Min(1, inset / maximumInset) : 0;
                progress?.Invoke(.25 + .75 * (depthPassIndex + local) / layers.Count,
                    $"Clearing pass {depthPassIndex + 1}/{layers.Count}: ring {ring + 1}, inset {inset:0.###} mm",
                    !measured);
            }
            // The normal ContractLayer is certified; the eligible RAW mode is
            // NOM awaiting original Smooth Protect floor/wall/grade guards.
            // Every fill level offsets this fixed boundary, never its predecessor.
            bool collectorNomination = rawNominationDepths?.Contains(-z) == true
                && (!hasEdgeCore || rawBandNominated);
            bool rawCollectorRequested = collectorNomination;
            long rawCollectorStarted = System.Diagnostics.Stopwatch.GetTimestamp();
            int rawMiddleLevels = 0, rawMiddleRings = 0, suppressedMiddleLevels = 0;
            bool selectiveRawRolesRequested = !retainRawFirst || !retainRawCore || !retainRawMiddle;
            long rawMiddlePoints = 0;
            string rawCollectorReason = "not-requested";
            var collected = new List<OffsetRing>();
            if (collectorNomination)
            {
                // These absolute offsets nominate centers from the unchanged
                // ORIGINAL artwork, never from a previous offset or filled set.
                // Literal first/core fronts are materialized below. Every other
                // signed/tiny RAW ring remains until original Smooth Protect.
                rawCollectorReason = "complete-original-source-absolute-RAW-NOM-levels";
                if (!double.IsFinite(maximumInset) || !double.IsFinite(fillPitch)
                    || fillPitch <= 1e-9 || maximumInset < 0)
                {
                    collectorNomination = false;
                    rawCollectorReason = "unsupported-finite-NOM-inset-schedule";
                }
                else
                {
                    ReportRing(0, 0);
                    long retainedRawPoints = rings.Sum(front => (long)front.Count)
                        + (terminalCore?.Sum(front => (long)front.Count) ?? 0);
                    if (retainedRawPoints > 500_000)
                    {
                        collectorNomination = false;
                        rawCollectorReason = "aggregate-500000-RAW-collector-point-limit";
                    }
                    for (int level = 1; collectorNomination && level <= maximumIterations; level++)
                    {
                        cancellationToken.ThrowIfCancellationRequested();
                        double additionalInset = Math.Min(level * fillPitch, maximumInset);
                        if (maximumInset - additionalInset <= 8 * (Math.BitIncrement(maximumInset) - maximumInset))
                            additionalInset = maximumInset;
                        ReportRing(level, additionalInset);
                        if (additionalInset == maximumInset && terminalCore is not null) break;
                        if (!retainRawMiddle) { suppressedMiddleLevels++; continue; }
                        double absoluteNominationInset = layerRadius + additionalInset;
                        string rawReason = "missing-captured-RAW-owner";
                        if (initialInsetNomination is null || !initialInsetNomination.TryNominateAuthoredBand(
                            clearingBoundary, absoluteNominationInset, layerRadius, cancellationToken,
                            detail => diagnostic?.Invoke($"stageRole=absolute-intermediate-RAW-NOM depth={-z:R} level={level} " +
                                $"additionalInset={additionalInset:R} absoluteNominationInset={absoluteNominationInset:R} " +
                                "savedAuthoredSettingsUnchanged=True terminalCoreAuthority=False " +
                                detail.Replace("authoredWidth=", "absoluteNominationInsetOperand=")
                                    .Replace("terminalSameCoreCandidate=True", "terminalSameCoreCandidate=False")),
                            out var rawMiddle, out rawReason))
                        {
                            collectorNomination = false;
                            rawCollectorReason = "whole-original-collector-fallback:" + rawReason;
                            break;
                        }
                        long points = rawMiddle!.Sum(front => (long)front.Count);
                        if (points > 500_000 - retainedRawPoints)
                        {
                            collectorNomination = false;
                            rawCollectorReason = "aggregate-500000-RAW-collector-point-limit";
                            break;
                        }
                        retainedRawPoints += points;
                        rawMiddleLevels++; rawMiddleRings += rawMiddle.Count; rawMiddlePoints += points;
                        foreach (var front in rawMiddle)
                            collected.Add(new OffsetRing(front, level, 0, isClosed: true, offsetDistance: additionalInset));
                        if (additionalInset == maximumInset) break;
                    }
                }
            }
            if (!collectorNomination)
            {
                // Any unsupported RAW level discards the WHOLE middle proposal;
                // the literal original collector, including its terminal proofs,
                // constructs the complete level. No partial RAW credit survives.
                collected.Clear();
                var layerRegions = PolygonOps.DifferenceToComponents(rings, Array.Empty<IReadOnlyList<PointD>>())
                    .Select(component => new MillingRegion(component.Outer,
                        component.Holes.Select(hole => (IReadOnlyList<PointD>)hole).ToList())).ToList();
                collected = MedialAxisGeometry.BuildConcentricFill(
                    layerRegions,
                    new RingOffsetOptions(0, fillPitch)
                    {
                        RingLimit = Math.Max(1, maximumIterations + 1),
                        MaximumInset = maximumInset,
                        MaximumInsetFront = terminalCore is null ? null : region =>
                            PolygonOps.Intersect(terminalCore, new[] { region.Outer }.Concat(region.Holes)),
                        MinimumRingArea = 0,
                        Tolerances = GeometryTolerances.Default with
                        {
                            ArcTolerance = 1 / PathUtils.Scale,
                            SimplifyTolerance = 0,
                        },
                        LevelProgress = ReportRing,
                    },
                    layerRadius,
                    preserveTerminalPaths: !concentricClearingOnly,
                    cancellationToken: cancellationToken);
            }
            diagnostic?.Invoke($"stage=raw-absolute-clearing-collector depth={-z:R} requested={rawCollectorRequested} " +
                $"enabled={collectorNomination} reason='{rawCollectorReason}' middleLevels={rawMiddleLevels} middleRings={rawMiddleRings} " +
                $"middlePoints={rawMiddlePoints} suppressedMiddleLevels={suppressedMiddleLevels} selectiveRawRolesRequested={selectiveRawRolesRequested} " +
                $"retainRawFirst={retainRawFirst} retainRawCore={retainRawCore} retainRawMiddle={retainRawMiddle} " +
                $"maximumRetainedRawPoints=500000 elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(rawCollectorStarted).TotalMilliseconds:F3} " +
                "originalSourceAbsoluteOffsets=True preparedIntermediateOffsetsUsed=FalseWhenEnabled nativeArcTicks=1 " +
                $"frontConditioningCaptured={initialInsetNomination?.FrontConditioningCaptured == true} partialProposalPublished=False " +
                "allSignedTinyRawRingsRetained=True noTerminalSkeletonNominationCredit=True physicalCredit=False stockCredit=False originalProtectRequired=True");
            bool selectedRawRolesSupported = collectorNomination
                && (!retainRawFirst || rings.Count > 0)
                && (!retainRawCore || terminalCore is { Count: > 0 });
            selectedRawRolesCompleted?.Invoke(selectedRawRolesSupported);
            var paths = new List<List<Point3D>>(collected.Count);
            bool contourRest=discExclusion is not null && (includeInteriorFill||maximumInset>0);
            int literalFirstRings = 0, literalTerminalRings = 0;
            if (collectorNomination)
            {
                void RetainLiteralFront(IEnumerable<List<PointD>> fronts, bool terminal)
                {
                    foreach (var front in fronts)
                    {
                        cancellationToken.ThrowIfCancellationRequested();
                        var trace = front.Select(p => new Point3D(p.x, p.y, z)).ToList();
                        if (trace.Count > 1 && trace[0] != trace[^1]) trace.Add(trace[0]);
                        if (trace.Count < 2) throw new InvalidOperationException("A bounded RAW clearing front lost its literal ring.");
                        if(!contourRest||!terminal)paths.Add(trace);
                        if (terminal) literalTerminalRings++; else literalFirstRings++;
                    }
                }
                if (retainRawFirst) RetainLiteralFront(rings, false);
                if (retainRawCore && terminalCore is not null) RetainLiteralFront(terminalCore, true);
            }
            var bandCandidates = new List<List<Point3D>>();
            foreach (OffsetRing ring in collected)
            {
                if (collectorNomination && (ring.Level == 0 || (terminalCore is not null
                    && ring.IsClosed && ring.OffsetDistance == maximumInset))) continue;
                var stroke = MaterializeClearingRing(ring, z);
                if (stroke.Count < 2) continue;
                if(contourRest&&ring.Level>0)continue;
                // Intermediate fronts and open terminals retain the authored
                // centerline band guard. The canonical final contour is already
                // that guard's boundary; clipping it against an independently
                // rounded copy would fragment the contour into microstrokes.
                bool canonicalFinal = terminalCore is not null && ring.IsClosed
                    && ring.OffsetDistance == maximumInset;
                if (hasEdgeCore && ring.Level > 0 && !canonicalFinal) bandCandidates.Add(stroke);
                else paths.Add(stroke);
            }
            if (bandCandidates.Count > 0)
            {
                var clipped = ClipClearingPathsOutsideCore(bandCandidates, edgeCore!, z);
                paths.AddRange(clipped);
                diagnostic?.Invoke($"stage=clearing-band event=clipped depth={-z:R} " +
                    $"inputPaths={bandCandidates.Count} clippedPaths={clipped.Count} canonicalFinalFrontWhole=True");
            }
            if(contourRest)
            {
                // Preserve the mandatory perimeter's original visit before
                // adding independent rest-pocket cycles at shared vertices.
                paths=JoinTouchingClearingCycles(paths);
                IReadOnlyList<IReadOnlyList<PointD>> area=hasEdgeCore ? PolygonOps.Difference(rings,edgeCore!) : rings;
                var cleared=restFillStock?.AtDepth(-z,cancellationToken,diagnostic)??Array.Empty<IReadOnlyList<PointD>>();
                var rest=discExclusion!.Fill(area,cleared,fillPitch,maximumInset,cancellationToken,diagnostic);
                RingOffsetEngine.BuildNesting(rest);
                foreach(int ordinal in RingOffsetEngine.BuildTraversal(rest,false).Order)
                    paths.Add(MaterializeClearingRing(rest[ordinal],z));
            }
            diagnostic?.Invoke($"stage=clearing-concentric event=topology depth={-z:R} " +
                $"closed={collected.Count(ring => ring.IsClosed)} open={collected.Count(ring => !ring.IsClosed)} " +
                $"sharedCollector=True fixedCompensatedBoundary=True");
            diagnostic?.Invoke($"stage=raw-clearing-collector-nomination depth={-z:R} enabled={collectorNomination} " +
                $"capturedInitialStockCohort={initialInsetNomination?.CollectorNominationCaptured == true} literalFirstRings={literalFirstRings} " +
                $"literalRawCoreRings={literalTerminalRings} collectedRings={collected.Count} materializedPaths={paths.Count} " +
                $"openTerminalSkeletonNominationAbsent={collectorNomination} " +
                $"selectedRawRolesSupported={selectedRawRolesSupported} retainRawFirst={retainRawFirst} retainRawCore={retainRawCore} retainRawMiddle={retainRawMiddle} " +
                $"allSelectedRoleLiteralRawFrontsWhenEnabled=True allLiteralRawFrontsWhenEnabled={!selectiveRawRolesRequested} " +
                "tinyRingAreaCutoffAdded=False originalRingWorkCaps=True " +
                "physicalTerminalProofCredit=False originalFloorArtworkGradeRequired=True stockCredit=False unsupportedWholeOriginalCollectorFallback=True");
            int separateCycles = paths.Count;
            if(!contourRest)paths = JoinTouchingClearingCycles(paths);
            if (paths.Count != separateCycles)
                diagnostic?.Invoke($"stage=clearing-topology depth={-z:R} sourceCycles={separateCycles} " +
                    $"connectedVisits={paths.Count} preservedCoverage=True");
            foreach (var path in paths)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var simplified = SimplifyCollinearRuns(path);
                result.Add((simplified, originalPassIndex));
            }

            diagnostic?.Invoke($"stage=clearing-fill event=complete depth={-z:R} paths={paths.Count} pitch={stepOverDist:R}");
            depthPassIndex++;
            progress?.Invoke(.25 + .75 * depthPassIndex / layers.Count,
                $"Clearing pass {depthPassIndex}/{layers.Count} complete", false);
        }

        progress?.Invoke(1, "Clearing complete", false);
        return result;
    }

    /// <summary>Materializes one collected clearing trace without inventing an open closing chord.</summary>
    internal static List<Point3D> MaterializeClearingRing(OffsetRing ring, double depth)
    {
        var points = ring.Points.Select(point => new Point3D(point.x, point.y, depth)).ToList();
        if (ring.IsClosed && points.Count > 1 && points[0] != points[^1])
            points.Add(points[0]);
        return points;
    }

    // A polygon boolean may return boundary cycles pinched at a shared vertex.
    // They are one connected cutter trace, even when one cycle is microscopic.
    // Splice those cycles at their exact common point; retain every source chord
    // and winding rather than inventing a link or rejecting a small contour.
    internal static List<List<Point3D>> JoinTouchingClearingCycles(List<List<Point3D>> paths)
    {
        var parent = Enumerable.Range(0, paths.Count).ToArray();
        int Root(int i) => parent[i] == i ? i : parent[i] = Root(parent[i]);
        var owner = new Dictionary<Point3D, int>();
        for (int i = 0; i < paths.Count; i++)
        {
            var path = paths[i];
            if (path.Count < 3 || path[0] != path[^1]) continue;
            for (int j = 0; j < path.Count - 1; j++)
            {
                if (owner.TryGetValue(path[j], out int previous)) parent[Root(i)] = Root(previous);
                else owner.Add(path[j], i);
            }
        }
        var groups = Enumerable.Range(0, paths.Count).GroupBy(Root).Select(group => group.ToList()).ToList();
        if (groups.Count == paths.Count) return paths;
        var joined = new List<List<Point3D>>(groups.Count);
        foreach (var group in groups.OrderBy(group => group[0]))
        {
            var circuit = new List<Point3D>(paths[group[0]]);
            var pending = group.Skip(1).ToList();
            while (pending.Count > 0)
            {
                var positions = new Dictionary<Point3D, int>();
                for (int j = 0; j < circuit.Count - 1; j++) positions.TryAdd(circuit[j], j);
                bool inserted = false;
                for (int p = 0; p < pending.Count && !inserted; p++)
                {
                    var cycle = paths[pending[p]];
                    if (SameCommandCycle(circuit, cycle))
                    {
                        // Independently derived mandatory/terminal cycles can
                        // resolve to the same command. They add no floor sweep.
                        pending.RemoveAt(p);
                        inserted = true;
                        continue;
                    }
                    int count = cycle.Count - 1;
                    for (int j = 0; j < count; j++)
                    {
                        if (!positions.TryGetValue(cycle[j], out int at)) continue;
                        circuit.InsertRange(at + 1, Enumerable.Range(1, count - 1)
                            .Select(step => cycle[(j + step) % count]).Append(cycle[j]));
                        pending.RemoveAt(p);
                        inserted = true;
                        break;
                    }
                }
                if (!inserted) throw new InvalidOperationException("A connected clearing boundary lost its shared vertex.");
            }
            joined.Add(circuit);
        }
        return joined;
    }

    private static bool SameCommandCycle(IReadOnlyList<Point3D> first, IReadOnlyList<Point3D> second)
    {
        if (first.Count != second.Count) return false;
        static bool Same(Point3D a, Point3D b) => (float)a.X == (float)b.X
            && (float)a.Y == (float)b.Y && (float)a.Z == (float)b.Z;
        int count = first.Count - 1;
        for (int start = 0; start < count; start++)
        {
            if (!Same(first[0], second[start])) continue;
            bool forward = true, reverse = true;
            for (int i = 1; i < count && (forward || reverse); i++)
            {
                forward &= Same(first[i], second[(start + i) % count]);
                reverse &= Same(first[i], second[(start - i + count) % count]);
            }
            if (forward || reverse) return true;
        }
        return false;
    }

    private static List<List<Point3D>> ClipClearingPathsOutsideCore(
        IReadOnlyList<List<Point3D>> paths, IReadOnlyList<List<PointD>> core, double depth)
    {
        // Clip centerlines as open strokes: subtracting closed areas would invent
        // extra toolpaths along the core boundary. Every surviving piece keeps its Z.
        var clipper = new Clipper64();
        clipper.AddOpenSubject(PathUtils.ToClipper(paths.Select(path =>
            path.Select(point => new PointD(point.X, point.Y)))));
        clipper.AddClip(PathUtils.ToClipper(core));
        var closed = new Paths64();
        var open = new Paths64();
        clipper.Execute(ClipType.Difference, FillRule.NonZero, closed, open);
        return PathUtils.FromClipper(open).Where(path => path.Count >= 2)
            .Select(path => path.Select(point => new Point3D(point.x, point.y, depth)).ToList()).ToList();
    }

    // --- generateToolpathsForObject / full V-carve (object-main) ----------------------

    /// <summary>
    /// Generates the full V-carve toolpath: clearing passes at each depth layer
    /// followed by the final medial-axis V-carve pass.
    ///
    /// (cutTypes.fill = true, standard passType).
    /// </summary>
    /// <param name="provider">Medial-axis provider.</param>
    /// <param name="boundary">Closed polygon rings (outer boundary first, holes after).</param>
    /// <param name="startDepth">Top of the region (usually 0).</param>
    /// <param name="endDepth">Maximum cut depth (positive = deeper).</param>
    /// <param name="radianTipAngle">Full included V-bit tip angle in radians.</param>
    /// <param name="depthPerPass">Maximum depth per pass.</param>
    /// <param name="stepOver">Step-over fraction of effective bit diameter (default 0.4).</param>
    /// <param name="tolerance">Medial-axis parabola discretisation tolerance (default 0.03).</param>
    /// <param name="includeInteriorFill">
    ///   Whether to clear broad areas with inward V-bit rings. Set to <c>false</c>
    ///   after roughing to retain perimeter and corner-detail paths only.
    /// </param>
    /// <returns>
    ///   All toolpath polylines: clearing passes first, then the final V-carve pass.
    /// </returns>
    public static List<List<Point3D>> GenerateVCarve(
        IMedialAxisProvider provider,
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double stepOver    = 0.4,
        double tolerance   = 0.03,
        double? bottomRadiusOverride = null,
        double? topRadiusOverride = null,
        double? coneLengthOverride = null,
        bool includeInteriorFill = true,
        VCarveCutterProfile? cutterProfile = null,
        double edgeFillWidth = 0,
        Action<double, string, bool>? progress = null)
    {
        var components = GenerateVCarveComponents(
            provider,
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            tolerance,
            bottomRadiusOverride: bottomRadiusOverride,
            topRadiusOverride: topRadiusOverride,
            coneLengthOverride: coneLengthOverride,
            includeInteriorFill: includeInteriorFill, cutterProfile: cutterProfile,
            edgeFillWidth: edgeFillWidth, progress: progress);

        var result = new List<List<Point3D>>(components.ClearingPasses.Count + components.FinalPass.Count);
        result.AddRange(components.ClearingPasses);
        result.AddRange(components.FinalPass);
        return result;
    }

    /// <summary>
    /// Same as <see cref="GenerateVCarve"/>, but emits per-path metadata tags
    /// for region and pass categorization.
    /// </summary>
    public static List<TaggedToolpath> GenerateVCarveTagged(
        IMedialAxisProvider provider,
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double stepOver    = 0.4,
        double tolerance   = 0.03,
        int regionIndex = 0,
        double? bottomRadiusOverride = null,
        double? topRadiusOverride = null,
        double? coneLengthOverride = null,
        CancellationToken cancellationToken = default,
        bool includeInteriorFill = true,
        Action<string>? diagnostic = null,
        bool detectCurves = true,
        VCarveCutterProfile? cutterProfile = null,
        double edgeFillWidth = 0,
        Action<double, string, bool>? progress = null,
        double outputSurfaceZ = 0,
        int skipDepthPasses = 0,
        double startingDepth = 0,
        IReadOnlyList<MedialSegment>? preparedMedialGraph = null)
        => GenerateVCarveTagged(false, provider, boundary, startDepth, endDepth,
            radianTipAngle, depthPerPass, stepOver, tolerance, regionIndex,
            bottomRadiusOverride, topRadiusOverride, coneLengthOverride,
            cancellationToken, includeInteriorFill, diagnostic, detectCurves,
            cutterProfile, edgeFillWidth, progress, outputSurfaceZ,
            skipDepthPasses, startingDepth, preparedMedialGraph);

    internal static List<TaggedToolpath> GenerateVCarveTagged(
        bool preferOriginalFirstConstruction,
        IMedialAxisProvider provider,
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double stepOver = 0.4,
        double tolerance = 0.03,
        int regionIndex = 0,
        double? bottomRadiusOverride = null,
        double? topRadiusOverride = null,
        double? coneLengthOverride = null,
        CancellationToken cancellationToken = default,
        bool includeInteriorFill = true,
        Action<string>? diagnostic = null,
        bool detectCurves = true,
        VCarveCutterProfile? cutterProfile = null,
        double edgeFillWidth = 0,
        Action<double, string, bool>? progress = null,
        double outputSurfaceZ = 0,
        int skipDepthPasses = 0,
        double startingDepth = 0,
        IReadOnlyList<MedialSegment>? preparedMedialGraph = null,
        RW.Toolpaths.Milling.Acceleration.RawInitialClearingInsetNomination? initialInsetNomination = null,
        bool supplementalInitialBand = true,
        bool concentricClearingOnly = false,
        bool includeClearingPasses = true,
        double excludedDiscRadius = 0,
        VCarvePostedFloorSweep? restFillStock = null,
        Func<double,double,VCarveMaterialSurface.ClearingLayer?>? materialClearingInset = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        long t0 = PerfLog.Start();
        using var insetReuse = new CertifiedInsetReuse(diagnostic, preferOriginalFirstConstruction);
        var result = new List<TaggedToolpath>();

        // Clearing passes (clear-main)
        diagnostic?.Invoke($"stage=clearing event=start interiorFill={includeInteriorFill} detectCurves={detectCurves} " +
            $"mode={(includeInteriorFill ? "interior-and-perimeters" : edgeFillWidth > 0 ? "edge-band" : "perimeters-only")} " +
            $"edgeFillWidth={edgeFillWidth:R} edgeFillOrigin=original-boundary");
        var clearing = includeClearingPasses ? GenerateClearingPassesTaggedCore(
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            regionIndex,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride,
            includeInteriorFill,
            detectCurves,
            diagnostic, cutterProfile, cancellationToken, edgeFillWidth,
            (fraction, stage, indeterminate) => progress?.Invoke(.3 * fraction, stage, indeterminate),
            outputSurfaceZ, skipDepthPasses, startingDepth, insetReuse, initialInsetNomination,
            concentricClearingOnly: concentricClearingOnly, excludedDiscRadius: excludedDiscRadius,restFillStock:restFillStock,
            materialClearingInset:materialClearingInset) : new List<TaggedToolpath>();
        diagnostic?.Invoke($"stage=clearing event=complete paths={clearing.Count} points={clearing.Sum(path => (long)path.Points.Count)} "
            + $"concentricClearingOnly={concentricClearingOnly}");
        cancellationToken.ThrowIfCancellationRequested();
        result.AddRange(clearing);

        // One extra lateral NOM at half the original requested depth. The
        // original medial generation/floor, full-depth visit and authored
        // stepdown remain unchanged; original Smooth Protect owns all cuts.
        if (supplementalInitialBand && initialInsetNomination?.SupplementalInitialBandCaptured == true
            && startDepth == 0 && double.IsFinite(endDepth) && endDepth / 2 > 0)
        {
            double supplementalDepth = endDepth / 2;
            long supplementalStarted = System.Diagnostics.Stopwatch.GetTimestamp();
            diagnostic?.Invoke($"stage=supplemental-initial-clearing-band event=start originalDepth={endDepth:R} " +
                $"supplementalDepth={supplementalDepth:R} commandedDepth={CommandedMachineDepth(supplementalDepth,outputSurfaceZ):R} " +
                $"radius={cutterProfile?.RadiusAtDepth(supplementalDepth):R} authoredWidth={edgeFillWidth:R} " +
                "capturedOnce=True originalFullDepthNomFirst=True originalMedialFloorUnchanged=True physicalCredit=False stockCredit=False");
            var supplemental = new List<TaggedToolpath>();
            bool boundedLadder = !includeInteriorFill && cutterProfile is not null;
            long aggregateNominationPoints = 0;
            string nominationFallback = "none";
            if (boundedLadder)
            {
                double fullRadius = cutterProfile!.PhysicalRadiusAtDepth(CommandedMachineDepth(endDepth, outputSurfaceZ));
                foreach (int divisor in new[] { 2, 4, 8 })
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    double levelDepth = endDepth / divisor;
                    double levelRadius = cutterProfile.PhysicalRadiusAtDepth(CommandedMachineDepth(levelDepth, outputSurfaceZ));
                    double nominationWidth = edgeFillWidth + fullRadius - levelRadius;
                    if (!double.IsFinite(nominationWidth) || nominationWidth <= 0 || nominationWidth > 100)
                    { boundedLadder = false; nominationFallback = "derived-width-domain"; break; }
                    diagnostic?.Invoke($"stage=supplemental-initial-reach-level divisor={divisor} nominalDepth={levelDepth:R} " +
                        $"commandedDepth={CommandedMachineDepth(levelDepth, outputSurfaceZ):R} physicalRadius={levelRadius:R} " +
                        $"savedAuthoredWidth={edgeFillWidth:R} nominationDerivedWidth={nominationWidth:R} widthRole=untrusted-center-NOM " +
                        "wholeOriginalArtwork=True physicalCredit=False stockCredit=False");
                    bool selectedRoleCompleted = false;
                    bool retainFirst = divisor == 4, retainCore = divisor != 4, retainMiddle = false;
                    string selectedRole = retainFirst ? "outer-first" : "inner-core";
                    diagnostic?.Invoke($"stage=supplemental-initial-selected-roles divisor={divisor} nominalDepth={levelDepth:R} " +
                        $"selectedRole={selectedRole} retainOuter={retainFirst} retainCore={retainCore} retainMiddle={retainMiddle} " +
                        "originalFullBandUnchanged=True originalMedialFloorUnchanged=True physicalCredit=False stockCredit=False");
                    var level = GenerateClearingPassesTaggedCore(boundary, startDepth, levelDepth,
                        radianTipAngle, depthPerPass, stepOver, regionIndex, bottomRadiusOverride,
                        topRadiusOverride, coneLengthOverride, includeInteriorFill, detectCurves,
                        detail => diagnostic?.Invoke($"savedAuthoredWidth={edgeFillWidth:R} nominationDerivedWidth={nominationWidth:R} " +
                            "localWidthRole=untrusted-center-NOM " + detail.Replace("authoredWidth=", "nominationWidth=")),
                        cutterProfile, cancellationToken, nominationWidth, progress: null,
                        outputSurfaceZ: outputSurfaceZ, skipDepthPasses: skipDepthPasses,
                        startingDepth: levelDepth, insetReuse: null, initialInsetNomination: initialInsetNomination,
                        retainRawFirst: retainFirst, retainRawCore: retainCore, retainRawMiddle: retainMiddle,
                        selectedRawRolesCompleted: complete => selectedRoleCompleted = complete);
                    if (!selectedRoleCompleted)
                    { boundedLadder = false; nominationFallback = "selected-RAW-role-unsupported-whole-original-half-band-fallback"; break; }
                    long points = level.Sum(path => (long)path.Points.Count);
                    if (points > 500_000 - aggregateNominationPoints)
                    { boundedLadder = false; nominationFallback = "aggregate-500000-materialized-point-limit"; break; }
                    aggregateNominationPoints += points;
                    if (divisor == 2)
                        level = level.Select(path => path with { Category = VCarveInitialReachSelection.HalfCoreMarkerCategory }).ToList();
                    else if (divisor == 4)
                        level = level.Select(path => path with { Category = VCarveInitialReachSelection.QuarterOuterMarkerCategory }).ToList();
                    else if (divisor == 8)
                        level = level.Select(path => path with { Category = VCarveInitialReachSelection.EighthCoreMarkerCategory }).ToList();
                    supplemental.AddRange(level);
                }
            }
            else nominationFallback = "unsupported-edge-band-profile";
            if (!boundedLadder)
            {
                supplemental.Clear();
                supplemental.AddRange(GenerateClearingPassesTaggedCore(boundary, startDepth,
                    supplementalDepth, radianTipAngle, depthPerPass, stepOver, regionIndex,
                    bottomRadiusOverride, topRadiusOverride, coneLengthOverride, includeInteriorFill,
                    detectCurves, diagnostic, cutterProfile, cancellationToken, edgeFillWidth,
                    progress: null, outputSurfaceZ: outputSurfaceZ, skipDepthPasses: skipDepthPasses,
                    startingDepth: supplementalDepth, insetReuse: null,
                    initialInsetNomination: initialInsetNomination.OriginalCollectorFallback()));
            }
            cancellationToken.ThrowIfCancellationRequested();
            // Preserve role custody for the common finishing-depth request,
            // including whole-band fallback nominations.
            if (!boundedLadder)
                supplemental = supplemental.Select(path => path with { Category = VCarveInitialReachSelection.HalfCoreMarkerCategory }).ToList();
            result.AddRange(supplemental);
            diagnostic?.Invoke($"stage=supplemental-initial-clearing-band event=complete depth={supplementalDepth:R} " +
                $"paths={supplemental.Count} points={supplemental.Sum(path => (long)path.Points.Count)} " +
                $"elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(supplementalStarted).TotalMilliseconds:F3} " +
                $"constructor={(boundedLadder ? "whole-RAW-NOM-band-ladder-v4" : "whole-original-half-band-fallback")} " +
                $"levelDivisors=2,4,8 aggregateNominationPoints={aggregateNominationPoints} maximumAggregateNominationPoints=500000 " +
                $"nominationFallback='{nominationFallback}' adaptiveXYMovement=FalseAtConstructor halfCoreSlopeSelectionRequested=False floorAwareOuterNominationRequested=True partialLadderPublished=False " +
                "selectedRoles=half-inner-core,quarter-outer-first,eighth-inner-core supplementalMiddleRoles=none " +
                "savedAuthoredWidthUnchanged=True originalStepdownUnchanged=True originalMedialGeneratedOnce=True insetProofCacheNotUsed=True " +
                "combinedSourceOriginalProtectRequired=True physicalCredit=False stockCredit=False");
        }

        // Final V-carve pass (carve-main)
        var finalPass = GenerateCore(
            provider,
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            tolerance,
            rdpTolerance: DefaultRdpTolerance,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride,
            cancellationToken,
            includeInteriorFill,
            diagnostic, cutterProfile,
            (fraction, stage, indeterminate) => progress?.Invoke(.3 + .7 * fraction, stage, indeterminate),
            outputSurfaceZ, preparedMedialGraph, insetReuse, materialClearingInset);
        cancellationToken.ThrowIfCancellationRequested();
        if (finalPass is not null)
        {
            foreach (var path in finalPass)
                result.Add(new TaggedToolpath(path, Array.Empty<ToolpathSpan>(),
                    regionIndex, "final-carve", null, detectCurves: detectCurves));
        }

        PerfLog.Stop(
            "MedialAxisToolpaths.GenerateVCarve",
            t0,
            $"rings={boundary.Count} interiorFill={includeInteriorFill} clearing={clearing.Count} final={(finalPass?.Count ?? 0)}");

        return result;
    }

    /// <summary>
    /// Same as <see cref="GenerateVCarve"/> but returns the clearing passes and the
    /// final medial-axis pass in separate lists so the caller can render them distinctly.
    /// </summary>
    public static (List<List<Point3D>> ClearingPasses, List<List<Point3D>> FinalPass)
        GenerateVCarveComponents(
            IMedialAxisProvider provider,
            IReadOnlyList<IReadOnlyList<PointD>> boundary,
            double startDepth,
            double endDepth,
            double radianTipAngle,
            double depthPerPass,
            double stepOver    = 0.4,
            double tolerance   = 0.03,
            double rdpTolerance = DefaultRdpTolerance,
            double? bottomRadiusOverride = null,
            double? topRadiusOverride = null,
            double? coneLengthOverride = null,
            bool includeInteriorFill = true,
            VCarveCutterProfile? cutterProfile = null,
            double edgeFillWidth = 0,
            Action<double, string, bool>? progress = null)
    {
        long t0 = PerfLog.Start();
        using var insetReuse = new CertifiedInsetReuse();
        var clearing = GenerateClearingPassGeometries(
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride,
            includeInteriorFill, cutterProfile: cutterProfile, edgeFillWidth: edgeFillWidth,
            progress: (fraction, stage, indeterminate) => progress?.Invoke(.3 * fraction, stage, indeterminate),
            insetReuse: insetReuse).Select(pass => pass.Points).ToList();
        var final = GenerateCore(
            provider,
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            tolerance,
            rdpTolerance,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride,
            includeInteriorFill: includeInteriorFill, cutterProfile: cutterProfile,
            progress: (fraction, stage, indeterminate) => progress?.Invoke(.3 + .7 * fraction, stage, indeterminate),
            insetReuse: insetReuse)
            ?? new List<List<Point3D>>();

        PerfLog.Stop(
            "MedialAxisToolpaths.GenerateVCarveComponents",
            t0,
            $"rings={boundary.Count} interiorFill={includeInteriorFill} clearing={clearing.Count} final={final.Count}");

        return (clearing, final);
    }

    /// <summary>
    /// Same as <see cref="GenerateVCarveComponents"/>, but includes path tags.
    /// </summary>
    public static (List<TaggedToolpath> ClearingPasses, List<TaggedToolpath> FinalPass)
        GenerateVCarveComponentsTagged(
            IMedialAxisProvider provider,
            IReadOnlyList<IReadOnlyList<PointD>> boundary,
            double startDepth,
            double endDepth,
            double radianTipAngle,
            double depthPerPass,
            double stepOver    = 0.4,
            double tolerance   = 0.03,
            double rdpTolerance = DefaultRdpTolerance,
            int regionIndex = 0,
            double? bottomRadiusOverride = null,
            double? topRadiusOverride = null,
            double? coneLengthOverride = null,
            bool includeInteriorFill = true,
            VCarveCutterProfile? cutterProfile = null,
            double edgeFillWidth = 0,
            Action<double, string, bool>? progress = null)
    {
        long t0 = PerfLog.Start();
        using var insetReuse = new CertifiedInsetReuse();
        var clearing = GenerateClearingPassesTaggedCore(
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            regionIndex,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride,
            includeInteriorFill, cutterProfile: cutterProfile, edgeFillWidth: edgeFillWidth,
            progress: (fraction, stage, indeterminate) => progress?.Invoke(.3 * fraction, stage, indeterminate),
            insetReuse: insetReuse);
        var finalGeometry = GenerateCore(
            provider,
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            tolerance,
            rdpTolerance,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride,
            includeInteriorFill: includeInteriorFill, cutterProfile: cutterProfile,
            progress: (fraction, stage, indeterminate) => progress?.Invoke(.3 + .7 * fraction, stage, indeterminate),
            insetReuse: insetReuse)
            ?? new List<List<Point3D>>();
        var final = finalGeometry
            .Select(path => new TaggedToolpath(path, regionIndex, "final-carve", null))
            .ToList();

        PerfLog.Stop(
            "MedialAxisToolpaths.GenerateVCarveComponents",
            t0,
            $"rings={boundary.Count} interiorFill={includeInteriorFill} clearing={clearing.Count} final={final.Count}");

        return (clearing, final);
    }

    /// <summary>
    /// Generates tagged V-carve paths for multiple regions/islands in order.
    /// Region indices are assigned by enumeration order.
    /// </summary>
    public static List<TaggedToolpath> GenerateVCarveTaggedForRegions(
        IMedialAxisProvider provider,
        IEnumerable<IReadOnlyList<IReadOnlyList<PointD>>> regions,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double stepOver    = 0.4,
        double tolerance   = 0.03,
        double? bottomRadiusOverride = null,
        double? topRadiusOverride = null,
        double? coneLengthOverride = null,
        bool includeInteriorFill = true,
        VCarveCutterProfile? cutterProfile = null,
        double edgeFillWidth = 0,
        Action<double, string, bool>? progress = null)
    {
        if (provider is null) throw new ArgumentNullException(nameof(provider));
        if (regions is null) throw new ArgumentNullException(nameof(regions));

        var result = new List<TaggedToolpath>();
        int regionIndex = 0;

        var regionList = regions.ToList();
        foreach (var region in regionList)
        {
            var tagged = GenerateVCarveTagged(
                provider,
                region,
                startDepth,
                endDepth,
                radianTipAngle,
                depthPerPass,
                stepOver,
                tolerance,
                regionIndex,
                bottomRadiusOverride,
                topRadiusOverride,
                coneLengthOverride,
                includeInteriorFill: includeInteriorFill, cutterProfile: cutterProfile,
                edgeFillWidth: edgeFillWidth,
                progress: (fraction, stage, indeterminate) => progress?.Invoke(
                    (regionIndex + fraction) / regionList.Count, stage, indeterminate));

            result.AddRange(tagged);
            regionIndex++;
        }

        return result;
    }

    // --- generateMedialAxisToolpaths (plan-pass) ------------------------------------

    /// <summary>
    /// Computes the medial axis and flat-area region for the pocket.
    ///
    /// Port of <c>plan-pass(n, t, e)</c> where t = radianTipAngle, e = depthPerPass.
    /// </summary>
    private static (IReadOnlyList<MedialSegment> medialAxis,
                    List<List<PointD>> flatAreaRegion)?
        GenerateMedialAxisToolpaths(
            IMedialAxisProvider provider,
            IReadOnlyList<IReadOnlyList<PointD>> boundary,
            double startDepth,
            double endDepth,
            double radianTipAngle,
            double depthPerPass,
            double tolerance,
            double? bottomRadiusOverride,
            double? topRadiusOverride,
            double? coneLengthOverride,
            CancellationToken cancellationToken,
            VCarveCutterProfile? cutterProfile = null,
            Action<string>? diagnostic = null,
            Action<double, string, bool>? progress = null,
            double outputSurfaceZ = 0,
            IReadOnlyList<MedialSegment>? preparedMedialGraph = null,
            CertifiedInsetReuse? insetReuse = null,
            bool includeInteriorFill = true,
            Func<double,double,VCarveMaterialSurface.ClearingLayer?>? materialClearingInset = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        long t0 = PerfLog.Start();
        var normalizedBoundary = NormalizeBoundaryRings(boundary);
        if (normalizedBoundary.Count == 0)
        {
            PerfLog.Stop("MedialAxisToolpaths.GenerateMedialAxisToolpaths", t0, "empty-normalized");
            return null;
        }

        var profile = ResolveToolProfile(
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride);

        double maxRadius = cutterProfile?.RadiusAtDepth(endDepth - startDepth) ?? profile.FinalRadius;
        if (preparedMedialGraph is not null)
        {
            var converted = preparedMedialGraph.Select(segment => new MedialSegment(
                new MedialPoint(segment.Point0.X, segment.Point0.Y,
                    -startDepth - RadiusToClippingDepth(segment.Point0.Radius, endDepth - startDepth,
                        profile.BottomRadius, profile.RadiusSlope, cutterProfile)),
                new MedialPoint(segment.Point1.X, segment.Point1.Y,
                    -startDepth - RadiusToClippingDepth(segment.Point1.Radius, endDepth - startDepth,
                        profile.BottomRadius, profile.RadiusSlope, cutterProfile)))).ToArray();
            double commandRadius = CommandedCuttingRadius(cutterProfile, profile.BottomRadius,
                profile.RadiusSlope, startDepth, endDepth, outputSurfaceZ, coneLengthOverride);
            // Only GenerateCore's actual interior-fill branch consumes this
            // region. Retain the original Contract call/cache on that branch.
            var preparedFlat = includeInteriorFill
                ? materialClearingInset?.Invoke(endDepth,commandRadius)?.Rings
                    ?? Contract(normalizedBoundary, commandRadius, cancellationToken, insetReuse)
                : new List<List<PointD>>();
            diagnostic?.Invoke($"stage=flat-area-work branch=prepared-medial computed={includeInteriorFill} "
                + $"reason={(includeInteriorFill ? "interior-fill-consumer" : "no-interior-fill-consumer")} "
                + $"radius={commandRadius:R} rings={preparedFlat.Count} medialConversionUnchanged=True");
            diagnostic?.Invoke($"stage=medial-axis event=reused preparedGraphEdges={preparedMedialGraph.Count} " +
                "physicalDepthBoundary=original-artwork depthFidelityRequired=True");
            cancellationToken.ThrowIfCancellationRequested();
            return (converted, preparedFlat);
        }
        double scale    = InitialScale;
        string? lastErr  = null;

        List<List<PointD>>?       quantizedLayer = null;
        List<MedialSegment>       medialAxis      = new();

        for (int attempt = 0; attempt < 4; attempt++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            long tAttempt = PerfLog.Start();
            progress?.Invoke(0, $"Preparing medial geometry (attempt {attempt + 1}/4)...", true);
            // Quantize the original polygon layer without contour cleanup
            quantizedLayer = QuantizeMedialBoundary(normalizedBoundary, scale);
            if (quantizedLayer is null || quantizedLayer.Count == 0)
            {
                PerfLog.Stop("MedialAxisToolpaths.GenerateMedialAxisToolpaths.Attempt", tAttempt, $"attempt={attempt + 1} empty-simplified");
                PerfLog.Stop("MedialAxisToolpaths.GenerateMedialAxisToolpaths", t0, "empty-simplified");
                return null;
            }

            // Compute medial axis via Voronoi-of-segments
            var rawSegments = GenerateMedialAxis(
                provider,
                quantizedLayer,
                maxRadius,
                scale,
                tolerance,
                cancellationToken, diagnostic,
                (fraction, stage, indeterminate) => progress?.Invoke(.95 * fraction, stage, indeterminate));

            double maxR    = 0;
            medialAxis.Clear();
            foreach (var seg in rawSegments)
            {
                cancellationToken.ThrowIfCancellationRequested();
                double r = Math.Max(seg.Point0.Radius, seg.Point1.Radius);
                if (r > maxR) maxR = r;

                // Convert radius to Z depth: z = -startDepth - radius / halfTan
                double d0 = RadiusToClippingDepth(seg.Point0.Radius, endDepth - startDepth,
                    profile.BottomRadius, profile.RadiusSlope, cutterProfile);
                double d1 = RadiusToClippingDepth(seg.Point1.Radius, endDepth - startDepth,
                    profile.BottomRadius, profile.RadiusSlope, cutterProfile);
                medialAxis.Add(new MedialSegment(
                    new MedialPoint(seg.Point0.X, seg.Point0.Y,
                        -startDepth - d0),
                    new MedialPoint(seg.Point1.X, seg.Point1.Y,
                        -startDepth - d1)));
            }

            // Success condition: no radius exceeds 55 % of bounding box minor axis
            var bb = BoundingBox(quantizedLayer.SelectMany(r => r));
            double minor = Math.Min(bb.maxX - bb.minX, bb.maxY - bb.minY);
            if (maxR < minor * 0.55)
            {
                lastErr = null;
                PerfLog.Stop("MedialAxisToolpaths.GenerateMedialAxisToolpaths.Attempt", tAttempt, $"attempt={attempt + 1} scale={scale:R} raw={rawSegments.Count} accepted=true");
                break;
            }

            PerfLog.Stop("MedialAxisToolpaths.GenerateMedialAxisToolpaths.Attempt", tAttempt, $"attempt={attempt + 1} scale={scale:R} raw={rawSegments.Count} accepted=false");
            lastErr = $"medial-axis radius ({maxR:F4}) breaks out of bbox ({minor:F4})";
            scale *= 2; // double scale and retry
        }

        if (lastErr is not null)
            Console.Error.WriteLine($"medial-axis: failed after scale increases; {lastErr}");

        // Flat bottom region: contract by cutter radius at final depth.
        // Use the raw normalized boundary so this region
        // matches exactly the area the clearing passes fill at endZ; otherwise the Difference
        // clip in GenerateFlatAreaFill removes medial-axis flat strokes from sub-regions that
        // are not covered by clearing rings, producing a visible toolpath gap.
        double flatCommandRadius = CommandedCuttingRadius(cutterProfile, profile.BottomRadius,
            profile.RadiusSlope, startDepth, endDepth, outputSurfaceZ, coneLengthOverride);
        progress?.Invoke(.95, "Preparing flat-area boundary...", true);
        // Contract's original entry check also applies when its unused work is skipped.
        cancellationToken.ThrowIfCancellationRequested();
        List<List<PointD>> flatArea = includeInteriorFill
            ? materialClearingInset?.Invoke(endDepth,flatCommandRadius)?.Rings
                ?? Contract(normalizedBoundary, flatCommandRadius, cancellationToken, insetReuse)
            : new List<List<PointD>>();
        diagnostic?.Invoke($"stage=flat-area-work branch=constructed-medial computed={includeInteriorFill} "
            + $"reason={(includeInteriorFill ? "interior-fill-consumer" : "no-interior-fill-consumer")} "
            + $"radius={flatCommandRadius:R} rings={flatArea.Count} medialConversionUnchanged=True");
        diagnostic?.Invoke($"stage=flat-area-inset depth={endDepth:R} commandedDepth={CommandedMachineDepth(endDepth, outputSurfaceZ):R} nominalRadius={maxRadius:R} " +
            $"commandRadius={flatCommandRadius:R} surfaceZ={outputSurfaceZ:R} zDecimals=3");
        progress?.Invoke(1, "Medial geometry complete", false);

        PerfLog.Stop(
            "MedialAxisToolpaths.GenerateMedialAxisToolpaths",
            t0,
            $"rings={boundary.Count} segments={medialAxis.Count} flatRings={flatArea.Count}");

        return (medialAxis, flatArea);
    }

    // TrimSegment interpolates its depth coordinate to locate the XY floor crossing.
    // Before the physical Z pass, this coordinate must be affine in clearance radius:
    // interpolating the nonlinear spherical inverse (or a capped cone inverse) would
    // move that crossing and drop otherwise reachable corner detail.
    internal static double RadiusToClippingDepth(double radius, double depthSpan,
        double legacyBottomRadius, double legacySlope, VCarveCutterProfile? cutterProfile)
    {
        if (cutterProfile is null)
            return Math.Max(0, (radius - legacyBottomRadius) * (1.0 / legacySlope));
        var first = cutterProfile.Sections[0];
        if (first.Kind == CarvingCutterSectionKind.Linear
            && cutterProfile.MaximumDepth <= first.Height && first.TopRadius > first.BottomRadius)
        {
            double slope = (first.TopRadius - first.BottomRadius) / first.Height;
            return Math.Max(0, (radius - first.BottomRadius) * (1.0 / slope));
        }
        return depthSpan + radius - cutterProfile.RadiusAtDepth(depthSpan);
    }

    // The post stores workspace Z as float, subtracts the float stock surface,
    // then formats stock-relative Z with F3. Compensation follows whichever is
    // deeper: the authored depth or the actual rounded command. Pitch and target
    // Z remain nominal. Physical lookup deliberately extends beyond the nominal
    // operation cap when the represented nose/taper continues above it.
    internal static double CommandedCuttingRadius(VCarveCutterProfile? cutterProfile,
        double bottomRadius, double radiusSlope, double startDepth, double layerDepth,
        double outputSurfaceZ, double? coneLength = null)
    {
        double commandedDepth = CommandedMachineDepth(layerDepth, outputSurfaceZ);
        double penetration = Math.Max(0, Math.Max(layerDepth, commandedDepth) - startDepth);
        return cutterProfile?.PhysicalRadiusAtDepth(penetration)
            ?? bottomRadius + radiusSlope * Math.Min(penetration, coneLength ?? double.PositiveInfinity);
    }

    private static double CommandedMachineDepth(double depth, double surfaceZ)
    {
        // Match the actual post's numeric formatting, including its runtime's
        // midpoint rule. Compensation must not invent an extra quantum at ties.
        float relativeZ = (float)(surfaceZ - depth) - (float)surfaceZ;
        return -double.Parse(relativeZ.ToString("F3", System.Globalization.CultureInfo.InvariantCulture),
            System.Globalization.CultureInfo.InvariantCulture);
    }

    private static (double BottomRadius, double RadiusSlope, double PassRadius, double FinalRadius)
        ResolveToolProfile(
            double startDepth,
            double endDepth,
            double radianTipAngle,
            double depthPerPass,
            double? bottomRadiusOverride,
            double? topRadiusOverride,
            double? coneLengthOverride)
    {
        double bottomRadius = Math.Max(0.0, bottomRadiusOverride ?? 0.0);
        double depthSpan = Math.Max(1e-9, endDepth - startDepth);

        double defaultSlope = Math.Max(1e-9, Math.Tan(radianTipAngle / 2.0));
        double topRadius = topRadiusOverride ?? (bottomRadius + defaultSlope * depthSpan);
        if (topRadius < bottomRadius)
            topRadius = bottomRadius;

        double coneLength = coneLengthOverride ?? depthSpan;
        coneLength = Math.Max(1e-9, coneLength);

        double radiusSlope = (topRadius - bottomRadius) / coneLength;
        if (radiusSlope < 1e-9)
            radiusSlope = 1e-9;

        double passDepth = Math.Max(0.0, depthPerPass);
        double passRadius = bottomRadius + radiusSlope * passDepth;
        if (passRadius > topRadius)
            passRadius = topRadius;

        double finalRadius = bottomRadius + radiusSlope * depthSpan;
        if (finalRadius > topRadius)
            finalRadius = topRadius;
        return (bottomRadius, radiusSlope, passRadius, finalRadius);
    }

    private static List<List<PointD>> NormalizeBoundaryRings(
        IReadOnlyList<IReadOnlyList<PointD>> boundary)
    {
        var rings = boundary
            .Where(r => r.Count >= 3)
            .Select(r => r.ToList())
            .ToList();

        if (rings.Count == 0)
            return new List<List<PointD>>();

        return PathUtils.CanonicalizeRings(rings);
    }

    /// <summary>Quantizes the original boundary for the native integer sites.
    /// Preserves its vertices and topology without contour smoothing.</summary>
    internal static List<List<PointD>> QuantizeMedialBoundary(
        IReadOnlyList<IReadOnlyList<PointD>> boundary, double scale)
    {
        var scaled = PathUtils.ToClipper(boundary.Select(r => r.Select(p =>
            new PointD(p.x * scale / PathUtils.Scale, p.y * scale / PathUtils.Scale))));
        return PathUtils.FromClipper(scaled)
            .Select(r => r.Select(p => new PointD(p.x * PathUtils.Scale / scale,
                p.y * PathUtils.Scale / scale)).ToList())
            .Where(r => r.Count >= 3).ToList();
    }

    // --- generateMedialAxis (ridge-pass) ----------------------------------------------

    /// <summary>
    /// Computes the medial axis using the injected <paramref name="provider"/>.
    ///
    /// Port of <c>ridge-pass(n, t, scaleCtx)</c> where t = maxRadius.
    /// </summary>
    private static IReadOnlyList<MedialSegment> GenerateMedialAxis(
        IMedialAxisProvider provider,
        IReadOnlyList<List<PointD>> quantizedLayer,
        double maxRadius,
        double scale,
        double tolerance,
        CancellationToken cancellationToken,
        Action<string>? diagnostic = null,
        Action<double, string, bool>? progress = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        long t0 = PerfLog.Start();
        // Group into boundary + holes via NonIntersectingPathGroups
        var groups = NonIntersectingPathGroups(quantizedLayer).ToList();
        var result = new List<MedialSegment>();
        int groupCount = 0;
        foreach (var (outerBoundary, holes) in groups)
        {
            cancellationToken.ThrowIfCancellationRequested();
            groupCount++;
            long tGroup = PerfLog.Start();

            var segments = MedialAxisGeometry.Build(outerBoundary,
                holes.Select(hole => (IReadOnlyList<PointD>)hole).ToList(), provider,
                tolerance, maxRadius, Math.PI, cancellationToken, diagnostic,
                (fraction, stage, indeterminate) => progress?.Invoke(
                    (groupCount - 1 + fraction) / groups.Count,
                    $"Medial group {groupCount}/{groups.Count}: {stage}", indeterminate),
                coordinateScale: scale);
            diagnostic?.Invoke($"stage=medial-policy contourCleanup=False anglePruning=False scale={scale:R} "
                + "source=quantized-original-boundary insideOutsideClassificationPreserved=True");

            if (segments.Count > 0)
                result.Capacity = Math.Max(result.Capacity, result.Count + segments.Count);
            result.AddRange(segments);

            PerfLog.Stop(
                "MedialAxisToolpaths.GenerateMedialAxis.Group",
                tGroup,
                $"outerPts={outerBoundary.Count} holes={holes.Count} emitted={segments.Count}");
        }

        PerfLog.Stop(
            "MedialAxisToolpaths.GenerateMedialAxis",
            t0,
            $"groups={groupCount} output={result.Count}");

        return result;
    }

    // --- trimSegment (clip-pass) ----------------------------------------------------

    /// <summary>
    /// Clips a medial-axis segment to the cut depth, collecting the below-
    /// depth portion into <paramref name="flatStrokes"/>.
    ///
    /// Port of <c>clip-pass(n, t, e)</c> where t = endZ, e = accumulator.
    /// </summary>
    internal static MedialSegment? TrimSegment(
        MedialSegment seg,
        double endZ,
        List<(PointD A, PointD B)> flatStrokes)
    {
        // p0.z / p1.z hold the depth as negative numbers (deeper = more negative)
        double z0 = seg.Point0.Radius; // actually stored as Z after plan-pass()
        double z1 = seg.Point1.Radius;

        // Both above cut depth -> keep
        if (z0 >= endZ && z1 >= endZ) return seg;

        // Both below cut depth -> flat-area stroke
        if (z0 <= endZ && z1 <= endZ)
        {
            flatStrokes.Add((
                new PointD(seg.Point0.X, seg.Point0.Y),
                new PointD(seg.Point1.X, seg.Point1.Y)));
            return null;
        }

        // Straddles cut depth -> clip and add below portion to flatStrokes
        double frac;
        MedialPoint clipped;

        if (z0 > endZ && z1 < endZ)
        {
            frac = (endZ - z0) / (z1 - z0);
            clipped = new MedialPoint(
                seg.Point0.X + frac * (seg.Point1.X - seg.Point0.X),
                seg.Point0.Y + frac * (seg.Point1.Y - seg.Point0.Y),
                endZ);
            flatStrokes.Add((new PointD(clipped.X, clipped.Y),
                             new PointD(seg.Point1.X, seg.Point1.Y)));
            return seg with { Point1 = clipped };
        }
        else // z0 < endZ && z1 > endZ
        {
            frac = (endZ - z1) / (z0 - z1);
            clipped = new MedialPoint(
                seg.Point1.X + frac * (seg.Point0.X - seg.Point1.X),
                seg.Point1.Y + frac * (seg.Point0.Y - seg.Point1.Y),
                endZ);
            flatStrokes.Add((new PointD(seg.Point0.X, seg.Point0.Y),
                             new PointD(clipped.X, clipped.Y)));
            return seg with { Point0 = clipped };
        }
    }

    // --- joinSegments (__) ---------------------------------------------------

    /// <summary>
    /// Merges collinear/adjacent flat-area strokes into polylines.
    ///
    /// </summary>
    internal static List<List<PointD>> JoinSegments(
        IEnumerable<(PointD A, PointD B)> strokes)
    {
        var polylines = new List<List<PointD>>();
        var current   = new List<PointD>();

        foreach (var (a, b) in strokes)
        {
            if (current.Count > 0)
            {
                var prev = current[^1];
                bool connects = Math.Abs(prev.x - a.x) < 1e-5 &&
                                Math.Abs(prev.y - a.y) < 1e-5;
                if (connects)
                {
                    current.Add(b);
                    continue;
                }
                polylines.Add(current);
            }
            current = new List<PointD> { a, b };
        }
        if (current.Count > 0) polylines.Add(current);
        return polylines;
    }

    // --- generateFlatAreaFill --------------------------------------------

    /// <summary>
    /// Fills the flat-bottom region with segments at the final cut depth.
    ///
    /// Emits final-depth segments for flat-area stroke input.
    ///
    /// <code>
    ///   let i = Pt.fromPointArrays(n);
    ///   i = i.exclusionFromStrokes(t);   // re.exclusionFromStrokes(i, t)
    ///   for (let s of i.all()) ...         // emit segments at endZ
    /// </code>
    /// <c>re.exclusionFromStrokes(paths, region)</c> iterates over each open
    /// path in <c>paths</c> and applies <c>ge.remove(path, region)</c> â€”
    /// a Clipper DIFFERENCE that keeps only the portions of the open stroke
    /// that lie <b>OUTSIDE</b> the closed region.  When the flat-area polygon
    /// is empty every stroke survives unchanged; when it is non-empty the
    /// interior floor strokes are discarded (they're already covered by the
    /// regular medial-axis traversal).
    /// </summary>
    internal static IEnumerable<MedialSegment> GenerateFlatAreaFill(
        IEnumerable<List<PointD>> flatStrokes,
        IEnumerable<List<PointD>> flatAreaRegion,
        double endZ,
        CancellationToken cancellationToken = default,
        Action<string>? diagnostic = null,
        Action<double, string, bool>? progress = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var regionList = flatAreaRegion as IList<List<PointD>> ?? flatAreaRegion.ToList();
        var strokeList = flatStrokes as IList<List<PointD>> ?? flatStrokes.ToList();
        long processed = 0, emitted = 0, selectedVertices = 0, cacheHits = 0;
        var timer = System.Diagnostics.Stopwatch.StartNew();
        long lastReportMs = 0;
        void Report(bool complete = false)
        {
            if (!complete && processed != 0 && timer.ElapsedMilliseconds - lastReportMs < 250)
                return;
            lastReportMs = timer.ElapsedMilliseconds;
            diagnostic?.Invoke($"stage=flat-area-clip event={(complete ? "complete" : "progress")} " +
                $"strokes={processed}/{strokeList.Count} clipRings={regionList.Count} " +
                $"selectedVertices={selectedVertices} cacheHits={cacheHits} emitted={emitted} " +
                $"elapsedMs={timer.Elapsed.TotalMilliseconds:F1}");
            progress?.Invoke(strokeList.Count == 0 ? 1 : (double)processed / strokeList.Count,
                $"Clipping flat strokes {processed}/{strokeList.Count}", false);
        }
        Report();

        // When the flat area is empty every stroke lies "outside" it -> return all unchanged.
        if (regionList.Count == 0)
        {
            foreach (var polyline in strokeList)
            {
                cancellationToken.ThrowIfCancellationRequested();
                PointD? prev = null;
                foreach (var pt in polyline)
                {
                    if (prev is not null)
                    {
                        emitted++;
                        yield return new MedialSegment(
                            new MedialPoint(prev.Value.x, prev.Value.y, endZ),
                            new MedialPoint(pt.x, pt.y, endZ));
                    }
                    prev = pt;
                }
                processed++;
                Report();
            }
            Report(complete: true);
            yield break;
        }

        // Non-empty flat area: clip each stroke to OUTSIDE the flat-area polygon
        // (Clipper Difference: subject = open stroke, clip = closed flat area).
        var clipPaths = PathUtils.ToClipper(regionList.Select(r => r.Select(p => p)));
        var clipBounds = clipPaths.Select(Clipper.GetBounds).ToArray();
        // A ring disjoint from the stroke's integer bounding box has zero winding
        // everywhere on that stroke. Enclosing rings (including holes/islands) have
        // intersecting boxes and remain in their original order and orientation.
        // Unlike clipping polygons to a local rectangle, this introduces no new
        // rounded vertices and preserves the original integer clipping geometry.
        var reusable = new Dictionary<string, ReuseableDataContainer64>();
        long cachedVertices = 0;
        const long cacheVertexBudget = 1_000_000;

        foreach (var polyline in strokeList)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (polyline.Count < 2)
            {
                processed++;
                Report();
                continue;
            }
            var subject = PathUtils.ToClipper(new[] { polyline })[0];
            var bounds = Clipper.GetBounds(subject);
            var selected = new List<int>();
            int vertexCount = 0;
            for (int i = 0; i < clipPaths.Count; i++)
            {
                var candidate = clipBounds[i];
                if (candidate.left > bounds.right || candidate.right < bounds.left ||
                    candidate.top > bounds.bottom || candidate.bottom < bounds.top)
                    continue;
                selected.Add(i);
                vertexCount += clipPaths[i].Count;
            }
            selectedVertices += vertexCount;
            string key = string.Join(",", selected);
            if (!reusable.TryGetValue(key, out var prepared))
            {
                prepared = new ReuseableDataContainer64();
                prepared.AddPaths(new Paths64(selected.Select(i => clipPaths[i])), PathType.Clip, false);
                if (reusable.Count < 256 && cachedVertices + vertexCount <= cacheVertexBudget)
                {
                    reusable.Add(key, prepared);
                    cachedVertices += vertexCount;
                }
            }
            else cacheHits++;

            var clipper = new Clipper64();
            clipper.AddOpenSubject(subject);
            clipper.AddReuseableData(prepared);

            var openResult   = new Paths64();
            var closedResult = new Paths64();
            // Clipper64.Execute: (clipType, fillRule, solutionClosed, solutionOpen)
            clipper.Execute(ClipType.Difference, FillRule.NonZero,
                            closedResult, openResult);

            foreach (var clipped in PathUtils.FromClipper(openResult))
            {
                PointD? prev = null;
                foreach (var pt in clipped)
                {
                    if (prev is not null)
                    {
                        emitted++;
                        yield return new MedialSegment(
                            new MedialPoint(prev.Value.x, prev.Value.y, endZ),
                            new MedialPoint(pt.x, pt.y, endZ));
                    }
                    prev = pt;
                }
            }
            processed++;
            Report();
        }
        Report(complete: true);
    }

    // --- joinIntoPolylines -----------------------------------------------

    /// <summary>
    /// Connects medial-axis segments into continuous polylines by chaining
    /// segments whose endpoints match.
    ///
    ///
    /// returns <c>null</c> for the same case so callers can propagate the
    /// </summary>
    internal static List<List<Point3D>>? JoinIntoPolylines(
        IEnumerable<MedialSegment> segments)
    {
        long t0 = PerfLog.Start();
        const double eps = 1e-5;
        bool Close(double a, double b) => Math.Abs(a - b) < eps;

        var polylines   = new List<List<Point3D>>();
        var currentLine = new List<Point3D>();
        Point3D?       prevEnd  = null;

        foreach (var seg in segments)
        {
            var p0 = new Point3D(seg.Point0.X, seg.Point0.Y, seg.Point0.Radius);
            var p1 = new Point3D(seg.Point1.X, seg.Point1.Y, seg.Point1.Radius);

            bool connects = prevEnd is not null
                && Close(prevEnd.Value.X, p0.X)
                && Close(prevEnd.Value.Y, p0.Y)
                && Close(prevEnd.Value.Z, p0.Z);
            bool reverseConnects = prevEnd is not null
                && Close(prevEnd.Value.X, p1.X)
                && Close(prevEnd.Value.Y, p1.Y)
                && Close(prevEnd.Value.Z, p1.Z);

            if (connects)
            {
                currentLine.Add(p1);
                prevEnd = p1;
                continue;
            }

            if (reverseConnects)
            {
                currentLine.Add(p0);
                prevEnd = p0;
                continue;
            }

            if (!connects)
            {
                if (currentLine.Count > 0)
                    polylines.Add(currentLine);
                currentLine = new List<Point3D> { p0 };
            }

            currentLine.Add(p1);
            prevEnd = p1;
        }

        if (currentLine.Count > 0)
            polylines.Add(currentLine);

        if (polylines.Count == 0)
        {
            PerfLog.Stop("MedialAxisToolpaths.JoinIntoPolylines", t0, "empty");
            return null;
        }

        PathUtils.NearestNeighborSort(polylines);

        PerfLog.Stop(
            "MedialAxisToolpaths.JoinIntoPolylines",
            t0,
            $"polylines={polylines.Count}");

        return polylines;
    }

    // --- Geometry helpers -----------------------------------------------------

    /// <summary>
    /// Separates a flat list of rings into (boundary, holes) groups based on
    /// containment.  Each outer CCW ring is paired with its CW hole rings.
    ///
    /// </summary>
    internal static IEnumerable<(List<PointD> Boundary, List<List<PointD>> Holes)>
        NonIntersectingPathGroups(IReadOnlyList<List<PointD>> rings)
    {
        // Convert each ring once to integer coordinates and reuse for winding + containment tests.
        var boundaries = new List<(List<PointD> Ring, Path64 IntPath)>();
        var holes = new List<(List<PointD> Ring, Point64 Probe)>();

        for (int i = 0; i < rings.Count; i++)
        {
            var ring = rings[i];
            var intPath = PathUtils.ToClipper(new[] { ring })[0];
            if (Clipper2Lib.Clipper.Area(intPath) > 0)
            {
                boundaries.Add((ring, intPath));
            }
            else if (intPath.Count > 0)
            {
                holes.Add((ring, intPath[0]));
            }
        }

        for (int b = 0; b < boundaries.Count; b++)
        {
            var boundary = boundaries[b];
            var myHoles = new List<List<PointD>>();
            for (int h = 0; h < holes.Count; h++)
            {
                var hole = holes[h];
                if (PathUtils.PointInPolygon(hole.Probe, boundary.IntPath))
                    myHoles.Add(hole.Ring);
            }

            yield return (boundary.Ring, myHoles);
        }
    }

    /// <summary>Contract (inward offset) all rings by <paramref name="amount"/>.</summary>
    private static List<List<PointD>> Contract(
        IEnumerable<IEnumerable<PointD>> rings, double amount,
        CancellationToken cancellationToken = default,
        CertifiedInsetReuse? insetReuse = null)
    {
        // Clearing perimeters and flat-area exclusions share the same original-
        // source contraction and complete commanded-chord clearance contract.
        var source = rings.Select(ring => (IReadOnlyList<PointD>)ring.ToArray()).ToArray();
        return insetReuse is null
            ? PolygonOps.CutterInset(source, amount, cancellationToken: cancellationToken)
            : insetReuse.Contract(source, amount, cancellationToken);
    }

    /// <summary>
    /// One region invocation's last certified Contract result. Only the fixed
    /// CutterInset defaults used by Contract enter this cache; candidate fronts
    /// and edge-band clipping never do. Keeping one entry bounds retained data
    /// while preserving the bottom clearing inset for the final flat-area query.
    /// </summary>
    internal sealed class CertifiedInsetReuse : IDisposable
    {
        private readonly Action<string>? _diagnostic;
        private readonly bool _preferOriginalFirstConstruction;
        private PointD[][]? _source;
        private PointD[][]? _result;
        private long _radiusBits;
        private int _sourceMismatches;
        private int _radiusMismatches;
        private double _computeMilliseconds;
        internal int PreparedCount { get; private set; }
        internal int ReuseCount { get; private set; }

        internal CertifiedInsetReuse(Action<string>? diagnostic = null,
            bool preferOriginalFirstConstruction = false)
        {
            _diagnostic = diagnostic;
            _preferOriginalFirstConstruction = preferOriginalFirstConstruction;
        }

        internal List<List<PointD>> Contract(
            IReadOnlyList<IReadOnlyList<PointD>> source, double radius,
            CancellationToken cancellationToken = default)
        {
            cancellationToken.ThrowIfCancellationRequested();
            long radiusBits = BitConverter.DoubleToInt64Bits(radius);
            if (_source is not null)
            {
                if (_radiusBits != radiusBits)
                    _radiusMismatches++;
                else if (MatchesSource(source, cancellationToken))
                {
                    ReuseCount++;
                    // No caller receives the arrays held by the cache, including
                    // the first caller whose returned lists may later be edited.
                    var reused = _result!.Select(ring => ring.ToList()).ToList();
                    cancellationToken.ThrowIfCancellationRequested();
                    return reused;
                }
                else
                    _sourceMismatches++;
            }

            // Exact ordered original coordinates are stricter than a quantized
            // hash: holes, winding, order and sub-grid changes cannot collide.
            var sourceSnapshot = source.Select(ring => ring.ToArray()).ToArray();
            long started = System.Diagnostics.Stopwatch.GetTimestamp();
            var computed = PolygonOps.CutterInset(sourceSnapshot, radius,
                cancellationToken: cancellationToken,
                preferOriginalFirstConstruction: _preferOriginalFirstConstruction,
                useOriginalHoleErosion: false);
            _computeMilliseconds += System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            cancellationToken.ThrowIfCancellationRequested();
            _source = sourceSnapshot;
            _result = computed.Select(ring => ring.ToArray()).ToArray();
            _radiusBits = radiusBits;
            PreparedCount++;
            return computed;
        }

        private bool MatchesSource(IReadOnlyList<IReadOnlyList<PointD>> source,
            CancellationToken cancellationToken)
        {
            if (_source!.Length != source.Count) return false;
            for (int ringIndex = 0; ringIndex < source.Count; ringIndex++)
            {
                var saved = _source[ringIndex];
                var candidate = source[ringIndex];
                if (saved.Length != candidate.Count) return false;
                for (int i = 0; i < saved.Length; i++)
                {
                    if ((i & 1023) == 0) cancellationToken.ThrowIfCancellationRequested();
                    if (BitConverter.DoubleToInt64Bits(saved[i].x) != BitConverter.DoubleToInt64Bits(candidate[i].x)
                        || BitConverter.DoubleToInt64Bits(saved[i].y) != BitConverter.DoubleToInt64Bits(candidate[i].y))
                        return false;
                }
            }
            return true;
        }

        public void Dispose()
        {
            _diagnostic?.Invoke($"stage=certified-inset-reuse event=complete capacity=1 " +
                $"prepared={PreparedCount} reused={ReuseCount} sourceMismatches={_sourceMismatches} " +
                $"radiusMismatches={_radiusMismatches} computeMs={_computeMilliseconds:F1} " +
                $"preferOriginalFirstConstruction={_preferOriginalFirstConstruction} " +
                "identity=exact-ordered-source-and-command-radius policy=Contract-defaults");
            _source = null;
            _result = null;
        }
    }

    /// <summary>Returns the difference of two polygon sets (subject minus clip).</summary>
    private static List<List<PointD>> DifferencePolygons(
        IEnumerable<IEnumerable<PointD>> subject, IEnumerable<IEnumerable<PointD>> clip)
    {
        var pathsA = PathUtils.ToClipper(subject);
        var pathsB = PathUtils.ToClipper(clip);
        if (pathsA.Count == 0) return new();
        if (pathsB.Count == 0) return PathUtils.FromClipper(pathsA);
        var result = Clipper2Lib.Clipper.BooleanOp(
            ClipType.Difference, pathsA, pathsB, FillRule.NonZero);
        return PathUtils.FromClipper(result);
    }

    /// <summary>Returns the axis-aligned bounding box of a set of points.</summary>
    private static (double minX, double minY, double maxX, double maxY)
        BoundingBox(IEnumerable<PointD> pts)
    {
        double minX =  double.MaxValue, minY =  double.MaxValue;
        double maxX = -double.MaxValue, maxY = -double.MaxValue;
        foreach (var p in pts)
        {
            if (p.x < minX) minX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.x > maxX) maxX = p.x;
            if (p.y > maxY) maxY = p.y;
        }
        return (minX, minY, maxX, maxY);
    }

    // --- Z-fidelity (analytical V-carve depth) -------------------------------------

    /// <summary>
    /// Builds a delegate <c>(x,y) =&gt; analytical V-carve Z</c> for the given
    /// pocket boundary and V-bit profile.  Use with
    /// <see cref="ToolpathCurveDetector.SampleMove(ToolMove, double, Func{double,double,double}?)"/>
    /// to render fitted arcs/Beziers while keeping Z analytically correct
    /// (<c>Z = max(endZ, -startDepth - (dist - bottomRadius) / radiusSlope)</c>).
    /// </summary>
    public static Func<double, double, double> CreateAnalyticalZFunction(
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double? bottomRadiusOverride = null,
        double? topRadiusOverride = null,
        double? coneLengthOverride = null,
        VCarveCutterProfile? cutterProfile = null)
    {
        var profile = ResolveToolProfile(
            startDepth, endDepth, radianTipAngle, depthPerPass,
            bottomRadiusOverride, topRadiusOverride, coneLengthOverride);
        var segments = BuildBoundarySegments(NormalizeBoundaryRings(boundary));
        double startZ = -startDepth;
        double endZ   = -endDepth;
        double invSlope = 1.0 / Math.Max(1e-9, profile.RadiusSlope);
        double bottomR  = profile.BottomRadius;
        var spatialIndex = new SpatialSegmentIndex(segments);
        return (x, y) => AnalyticalZ(x, y, spatialIndex, startZ, bottomR, invSlope, endZ, cutterProfile);
    }

    /// <summary>
    /// Re-derives Z along a (possibly XY-simplified) toolpath from the analytical
    /// V-carve depth function:
    /// <code>
    ///   Z(x,y) = max(endZ, -startDepth - (dist((x,y), boundary) - bottomRadius) / radiusSlope)
    /// </code>
    /// Each retained vertex's Z is replaced with the analytical value, then every
    /// chord is recursively bisected wherever its linear-Z interpolation deviates
    /// from the analytical Z curve by more than <paramref name="zTolerance"/>.
    /// XY topology is preserved exactly; only Z waypoints are added back where
    /// the depth profile curves significantly.
    /// </summary>
    internal static List<Point3D> EnsureZFidelityFromBoundary(
        List<Point3D> path,
        SpatialSegmentIndex spatialIndex,
        double startDepth,
        double bottomRadius,
        double radiusSlope,
        double endZ,
        double zTolerance,
        VCarveCutterProfile? cutterProfile = null,
        Action<string>? diagnostic = null)
    {
        if (path.Count < 2 || zTolerance <= 0)
            return path;

        double startZ = -startDepth;
        double invSlope = 1.0 / radiusSlope;

        // Snap every retained vertex's Z to its analytical value first.
        var snapped = new List<Point3D>(path.Count);
        foreach (var p in path)
        {
            double z = AnalyticalZ(p.X, p.Y, spatialIndex, startZ, bottomRadius, invSlope, endZ, cutterProfile);
            snapped.Add(new Point3D(p.X, p.Y, z));
        }

        var refinement = cutterProfile is not null && diagnostic is not null
            ? new ZRefinementDiagnostics() : null;
        var output = new List<Point3D>(snapped.Count * 2);
        output.Add(snapped[0]);
        for (int i = 0; i < snapped.Count - 1; i++)
        {
            try
            {
                BisectChordForZ(
                    snapped[i], snapped[i + 1], spatialIndex,
                    startZ, bottomRadius, invSlope, endZ, zTolerance,
                    output, depth: 0, cutterProfile, refinement);
            }
            catch (InvalidOperationException error)
            {
                throw new InvalidOperationException($"V-carve source chord {i}/{snapped.Count - 1}: " +
                    $"({snapped[i].X:R},{snapped[i].Y:R},{snapped[i].Z:R})->" +
                    $"({snapped[i + 1].X:R},{snapped[i + 1].Y:R},{snapped[i + 1].Z:R}). {error.Message}", error);
            }
        }
        if (refinement?.MaximumDepth > MaxZFidelityBisectDepth)
            diagnostic?.Invoke($"stage=depth-fidelity event=extended-refinement " +
                $"recursionDepth={refinement.MaximumDepth} budget={MaxRoundedZFidelityBisectDepth} " +
                $"extendedBranches={refinement.ExtendedBranches} sourcePoints={path.Count} outputPoints={output.Count} " +
                $"zTolerance={zTolerance:R} noseDiameter={cutterProfile!.ClearingDiameter:R}");
        if (refinement?.CylinderWithdrawals > 0)
            diagnostic?.Invoke($"stage=depth-fidelity event=cylinder-withdrawal count={refinement.CylinderWithdrawals} "
                + $"sourcePoints={path.Count} outputPoints={output.Count} zTolerance={zTolerance:R} "
                + "physicalRadiusPreserved=True verticalTransition=True");
        return output;
    }

    private const int MaxZFidelityBisectDepth = 14;
    // At a hemisphere's equator its inverse depth has a square-root cusp. Halving
    // XY converges at half the cone's exponent, so permit twice the recursion
    // budget for rounded tools. Every accepted chord still needs the exact guard.
    private const int MaxRoundedZFidelityBisectDepth = 28;

    private sealed class ZRefinementDiagnostics
    {
        public int MaximumDepth;
        public int ExtendedBranches;
        public int CylinderWithdrawals;
    }


    /// <summary>
    /// Splits any path segment whose |dz| exceeds the given cap, inserting
    /// midpoints with analytical Z. This bounds the error of the GPU swept-
    /// volume renderer, which projects the tool tip along the segment's XZ
    /// footprint only and otherwise leaves pillar-shaped gaps on V-carve ramps
    /// where dz is large. Flat (constant-Z) chords are left untouched because
    /// XZ projection is exact for them.
    /// </summary>
    internal static List<Point3D> EnforceMaxChordLength(
        List<Point3D> path,
        IReadOnlyList<(PointD A, PointD B)> boundarySegments,
        double startDepth,
        double bottomRadius,
        double radiusSlope,
        double endZ,
        double maxChordDz)
    {
        if (path.Count < 2 || boundarySegments.Count == 0 || maxChordDz <= 0)
            return path;

        double startZ = -startDepth;
        double invSlope = 1.0 / radiusSlope;

        var spatialIndex = new SpatialSegmentIndex(boundarySegments);

        var output = new List<Point3D>(path.Count * 2);
        output.Add(path[0]);
        for (int i = 1; i < path.Count; i++)
        {
            SubdivideChord(path[i - 1], path[i], spatialIndex,
                startZ, bottomRadius, invSlope, endZ,
                maxChordDz, output, depth: 0);
        }
        return output;
    }

    private const int MaxChordSubdivideDepth = 14;

    private static void SubdivideChord(
        Point3D a, Point3D b,
        SpatialSegmentIndex spatialIndex,
        double startZ, double bottomRadius, double invSlope, double endZ,
        double maxChordDz,
        List<Point3D> output, int depth)
    {
        double dz = b.Z - a.Z;
        if (Math.Abs(dz) <= maxChordDz || depth >= MaxChordSubdivideDepth)
        {
            output.Add(b);
            return;
        }

        double mx = 0.5 * (a.X + b.X);
        double my = 0.5 * (a.Y + b.Y);
        double mz = AnalyticalZ(mx, my, spatialIndex, startZ, bottomRadius, invSlope, endZ);
        var mid = new Point3D(mx, my, mz);
        SubdivideChord(a, mid, spatialIndex, startZ, bottomRadius, invSlope, endZ,
            maxChordDz, output, depth + 1);
        SubdivideChord(mid, b, spatialIndex, startZ, bottomRadius, invSlope, endZ,
            maxChordDz, output, depth + 1);
    }

    private static void BisectChordForZ(
        Point3D a,
        Point3D b,
        SpatialSegmentIndex spatialIndex,
        double startZ,
        double bottomRadius,
        double invSlope,
        double endZ,
        double zTolerance,
        List<Point3D> output,
        int depth,
        VCarveCutterProfile? cutterProfile = null,
        ZRefinementDiagnostics? refinement = null)
    {
        int maximumDepth = cutterProfile?.RoundedTip == true
            ? MaxRoundedZFidelityBisectDepth : MaxZFidelityBisectDepth;
        if (refinement is not null)
        {
            refinement.MaximumDepth = Math.Max(refinement.MaximumDepth, depth);
            if (depth == MaxZFidelityBisectDepth + 1) refinement.ExtendedBranches++;
        }
        double dx = b.X - a.X;
        double dy = b.Y - a.Y;
        if ((dx * dx + dy * dy) < 1e-18)
        {
            output.Add(b);
            return;
        }

        double mx = 0.5 * (a.X + b.X);
        double my = 0.5 * (a.Y + b.Y);
        double zChord = 0.5 * (a.Z + b.Z);
        double zAnalytical = AnalyticalZ(mx, my, spatialIndex, startZ, bottomRadius, invSlope, endZ, cutterProfile);

        bool midpointFits = Math.Abs(zChord - zAnalytical) <= zTolerance;
        double split = 0.5;
        // Both an off-center depth minimum and the surface clamp of a finite tip can
        // escape a midpoint test. Certify the entire cutting interval against the
        // original boundary before accepting it.
        bool overcuts = (midpointFits || depth >= maximumDepth)
            && spatialIndex.TryFindDepthViolation(a, b, startZ, bottomRadius,
                1.0 / invSlope, zTolerance, out split, cutterProfile);
        if (midpointFits && !overcuts)
        {
            output.Add(b);
            return;
        }

        if (depth >= maximumDepth)
        {
            // A constant-radius section creates a real jump in the inverse
            // clearance function. Lift at the fitting endpoint, rather than
            // interpolating a tilted cylinder into the retained corner. This
            // final dyadic XY interval is already below geometric tolerance.
            if (overcuts && cutterProfile is not null)
            {
                double sectionStart = 0;
                foreach (var section in cutterProfile.Sections)
                {
                    if (section.Kind == CarvingCutterSectionKind.Linear
                        && section.BottomRadius == section.TopRadius)
                    {
                        double liftZ = startZ - sectionStart;
                        var raisedA = new Point3D(a.X, a.Y, Math.Max(a.Z, liftZ));
                        var raisedB = new Point3D(b.X, b.Y, Math.Max(b.Z, liftZ));
                        if ((raisedA.Z > a.Z || raisedB.Z > b.Z)
                            && !spatialIndex.TryFindDepthViolation(raisedA, raisedB, startZ,
                                bottomRadius, 1.0 / invSlope, zTolerance, out _, cutterProfile))
                        {
                            output.Add(raisedA); output.Add(raisedB); output.Add(b);
                            if (refinement is not null) refinement.CylinderWithdrawals++;
                            return;
                        }
                    }
                    sectionStart += section.Height;
                }
            }
            if (overcuts)
                throw new InvalidOperationException($"V-carve depth refinement could not certify boundary clearance " +
                    $"within {zTolerance:R} mm at ({a.X:R},{a.Y:R},{a.Z:R})->({b.X:R},{b.Y:R},{b.Z:R}); " +
                    $"recursionDepth={depth} budget={maximumDepth} roundedTip={cutterProfile?.RoundedTip ?? false} " +
                    $"noseDiameter={cutterProfile?.ClearingDiameter ?? 0:R} profileDepth={cutterProfile?.MaximumDepth ?? 0:R}.");
            output.Add(b);
            return;
        }

        if (overcuts)
        {
            // Avoid repeatedly splitting at an endpoint when the minimum lies at the
            // open depth-tolerance boundary. Interior extrema are inserted directly.
            if (split < 0.01 || split > 0.99) split = 0.5;
            mx = a.X + split * dx;
            my = a.Y + split * dy;
            zAnalytical = AnalyticalZ(mx, my, spatialIndex, startZ, bottomRadius, invSlope, endZ, cutterProfile);
        }

        var mid = new Point3D(mx, my, zAnalytical);
        BisectChordForZ(a, mid, spatialIndex, startZ, bottomRadius, invSlope, endZ, zTolerance, output, depth + 1, cutterProfile, refinement);
        BisectChordForZ(mid, b, spatialIndex, startZ, bottomRadius, invSlope, endZ, zTolerance, output, depth + 1, cutterProfile, refinement);
    }

    private static double AnalyticalZ(
        double x,
        double y,
        SpatialSegmentIndex spatialIndex,
        double startZ,
        double bottomRadius,
        double invSlope,
        double endZ,
        VCarveCutterProfile? cutterProfile = null)
    {
        double r = spatialIndex.QueryDistance(x, y);
        // F3 XY rounding must not push a full-radius cylinder off its fitting
        // side of a depth jump. Reserve this in the clearance domain, where
        // the straight ball's inverse has unbounded slope at its equator.
        if (cutterProfile is not null && cutterProfile.FirstCylinderHeight < startZ - endZ)
            r = Math.Max(0, r - .002);
        double depthBelowStart = cutterProfile?.DepthAtRadius(r)
            ?? (Math.Max(0.0, r - bottomRadius) * invSlope);
        double z = startZ - depthBelowStart;
        if (z < endZ) z = endZ;
        return z;
    }

    private static double DistanceToBoundary(
        double x, double y, IReadOnlyList<(PointD A, PointD B)> segments)
    {
        double bestSq = double.PositiveInfinity;
        for (int i = 0; i < segments.Count; i++)
        {
            var (a, b) = segments[i];
            double abx = b.x - a.x;
            double aby = b.y - a.y;
            double apx = x - a.x;
            double apy = y - a.y;
            double ab2 = abx * abx + aby * aby;
            double t = ab2 <= 1e-18 ? 0.0 : (apx * abx + apy * aby) / ab2;
            if (t < 0.0) t = 0.0;
            else if (t > 1.0) t = 1.0;
            double cx = a.x + abx * t;
            double cy = a.y + aby * t;
            double dx = x - cx;
            double dy = y - cy;
            double d2 = dx * dx + dy * dy;
            if (d2 < bestSq) bestSq = d2;
        }
        return bestSq < double.PositiveInfinity ? Math.Sqrt(bestSq) : 0.0;
    }

    /// <summary>
    /// Immutable double-coordinate BVH for analytical wall distance and continuous
    /// cutter clearance. Virtual cell extents retain the old grid's first-seen
    /// order without allocating cells or repeating segment references.
    /// </summary>
    internal sealed class SpatialSegmentIndex
    {
        private const int LeafCapacity = 8;
        private const double ArithmeticReserve = 16 * 2.2204460492503131e-16;
        private readonly (PointD A, PointD B)[] _segments;
        private readonly Box[] _bounds;
        private readonly CellRange[] _cells;
        private readonly int[] _order;
        private readonly Node[] _nodes;
        private readonly double _minX, _minY;
        private readonly double _cellSizeX, _cellSizeY;
        private readonly int _cols, _rows;
        private readonly QueryDiagnostics? _diagnostics;

        [ThreadStatic] private static QueryScratch? _scratch;
        private static QueryScratch Scratch => _scratch ??= new QueryScratch();

        private sealed class QueryScratch
        {
            internal readonly List<OrderedCandidate> Candidates = new();
            internal readonly List<RadiusInterval> Intervals = new();
            internal WeakReference<SpatialSegmentIndex>? NearestOwner;
            internal int NearestOrdinal;
        }

        private readonly record struct Box(double MinX, double MinY, double MaxX, double MaxY)
        {
            internal bool Intersects(Box other) => MinX <= other.MaxX && MaxX >= other.MinX
                && MinY <= other.MaxY && MaxY >= other.MinY;
        }
        private readonly record struct CellRange(int MinC, int MinR, int MaxC, int MaxR)
        {
            internal bool Intersects(CellRange other) => MinC <= other.MaxC && MaxC >= other.MinC
                && MinR <= other.MaxR && MaxR >= other.MinR;
        }
        private readonly record struct Node(Box Bounds, CellRange Cells,
            int First, int Count, int Left, int Right);
        private readonly record struct OrderedCandidate(int Ordinal, int FirstCell) : IComparable<OrderedCandidate>
        {
            public int CompareTo(OrderedCandidate other)
            {
                int result = FirstCell.CompareTo(other.FirstCell);
                return result != 0 ? result : Ordinal.CompareTo(other.Ordinal);
            }
        }

        internal sealed class QueryDiagnostics
        {
            internal long NearestQueries, NearestNodes, DistanceTests, NearestSeedTrials, NearestSeedWins;
            internal long CandidateQueries, CandidateNodes, CandidateLeafTests, Candidates;
            internal long ScratchGrowths, PartitionFallbacks;
            internal int NodeCount;
            internal double BuildMilliseconds;
        }
        internal QueryDiagnostics? Diagnostics => _diagnostics;

        public SpatialSegmentIndex(IReadOnlyList<(PointD A, PointD B)> segments, bool enableDiagnostics = false)
        {
            string? diagnosticSwitch = Environment.GetEnvironmentVariable("RW_TOOLPATHS_WALL_INDEX_DIAGNOSTICS");
            if (enableDiagnostics || diagnosticSwitch == "1") _diagnostics = new();
            long started = _diagnostics is null ? 0 : System.Diagnostics.Stopwatch.GetTimestamp();
            _segments = segments.ToArray();
            _bounds = new Box[_segments.Length];
            _cells = new CellRange[_segments.Length];
            _order = new int[_segments.Length];
            if (_segments.Length == 0)
            {
                _nodes = Array.Empty<Node>();
                return;
            }

            double minX = double.MaxValue, minY = double.MaxValue;
            double maxX = -double.MaxValue, maxY = -double.MaxValue;
            foreach (var seg in _segments)
            {
                minX = Math.Min(minX, Math.Min(seg.A.x, seg.B.x));
                maxX = Math.Max(maxX, Math.Max(seg.A.x, seg.B.x));
                minY = Math.Min(minY, Math.Min(seg.A.y, seg.B.y));
                maxY = Math.Max(maxY, Math.Max(seg.A.y, seg.B.y));
            }
            _minX = minX; _minY = minY;
            int dims = Math.Clamp((int)Math.Sqrt(_segments.Length), 8, 64);
            _cols = dims; _rows = dims;
            _cellSizeX = Math.Max(1e-9, maxX - minX) / _cols;
            _cellSizeY = Math.Max(1e-9, maxY - minY) / _rows;

            for (int i = 0; i < _segments.Length; i++)
            {
                var seg = _segments[i];
                double x0 = Math.Min(seg.A.x, seg.B.x), x1 = Math.Max(seg.A.x, seg.B.x);
                double y0 = Math.Min(seg.A.y, seg.B.y), y1 = Math.Max(seg.A.y, seg.B.y);
                // These casts intentionally match the old grid insertion arithmetic.
                // Reconstructing cell bounds in world coordinates introduces a
                // roundoff-sensitive disagreement at exact cell boundaries.
                _cells[i] = new(
                    Math.Clamp((int)((x0 - _minX) / _cellSizeX), 0, _cols - 1),
                    Math.Clamp((int)((y0 - _minY) / _cellSizeY), 0, _rows - 1),
                    Math.Clamp((int)((x1 - _minX) / _cellSizeX), 0, _cols - 1),
                    Math.Clamp((int)((y1 - _minY) / _cellSizeY), 0, _rows - 1));
                _bounds[i] = ExpandedBox(x0, y0, x1, y1);
                _order[i] = i;
            }
            _nodes = new Node[CountNodes(_segments.Length)];
            int nextNode = 0;
            Build(0, _segments.Length, ref nextNode);
            if (_diagnostics is not null)
            {
                _diagnostics.NodeCount = _nodes.Length;
                _diagnostics.BuildMilliseconds = System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            }
        }

        internal void LogDiagnostics(Action<string>? diagnostic)
        {
            if (_diagnostics is not { } counters) return;
            string message = $"stage=wall-index event=queries segments={_segments.Length} nodes={counters.NodeCount} " +
                $"buildMs={counters.BuildMilliseconds:F3} nearestQueries={counters.NearestQueries} " +
                $"nearestNodes={counters.NearestNodes} distanceTests={counters.DistanceTests} " +
                $"nearestSeedTrials={counters.NearestSeedTrials} nearestSeedWins={counters.NearestSeedWins} " +
                $"candidateQueries={counters.CandidateQueries} candidateNodes={counters.CandidateNodes} " +
                $"candidateLeafTests={counters.CandidateLeafTests} candidates={counters.Candidates} " +
                $"scratchGrowths={counters.ScratchGrowths} partitionFallbacks={counters.PartitionFallbacks}";
            if (diagnostic is not null) diagnostic(message);
            else Console.Error.WriteLine(message);
        }

        private static int CountNodes(int count)
            => count <= LeafCapacity ? 1 : 1 + CountNodes(count / 2) + CountNodes(count - count / 2);

        private int Build(int first, int count, ref int nextNode)
        {
            double x0 = double.PositiveInfinity, y0 = x0, x1 = double.NegativeInfinity, y1 = x1;
            int c0 = int.MaxValue, r0 = c0, c1 = int.MinValue, r1 = c1;
            for (int i = first; i < first + count; i++)
            {
                int ordinal = _order[i];
                Box bounds = _bounds[ordinal]; CellRange cells = _cells[ordinal];
                x0 = Math.Min(x0, bounds.MinX); y0 = Math.Min(y0, bounds.MinY);
                x1 = Math.Max(x1, bounds.MaxX); y1 = Math.Max(y1, bounds.MaxY);
                c0 = Math.Min(c0, cells.MinC); r0 = Math.Min(r0, cells.MinR);
                c1 = Math.Max(c1, cells.MaxC); r1 = Math.Max(r1, cells.MaxR);
            }
            int node = nextNode++;
            int left = -1, right = -1;
            if (count > LeafCapacity)
            {
                int half = count / 2;
                Partition(first, first + count - 1, first + half, x1 - x0 >= y1 - y0);
                left = Build(first, half, ref nextNode);
                right = Build(first + half, count - half, ref nextNode);
            }
            _nodes[node] = new(new(x0, y0, x1, y1), new(c0, r0, c1, r1), first, count, left, right);
            return node;
        }

        private void Partition(int lo, int hi, int middle, bool xAxis)
        {
            int remaining = 2 * (int)Math.Log2(hi - lo + 1) + 4;
            while (lo < hi)
            {
                // Bound adversarial quickselect work; the fallback is restricted to
                // the unresolved partition rather than sorting every tree level.
                if (remaining-- == 0)
                {
                    Array.Sort(_order, lo, hi - lo + 1,
                        Comparer<int>.Create((a, b) => CompareCentroid(a, b, xAxis)));
                    if (_diagnostics is not null) Interlocked.Increment(ref _diagnostics.PartitionFallbacks);
                    return;
                }
                int pivot = _order[lo + (hi - lo) / 2];
                int i = lo, j = hi;
                while (i <= j)
                {
                    while (CompareCentroid(_order[i], pivot, xAxis) < 0) i++;
                    while (CompareCentroid(_order[j], pivot, xAxis) > 0) j--;
                    if (i <= j) { (_order[i], _order[j]) = (_order[j], _order[i]); i++; j--; }
                }
                if (middle <= j) hi = j;
                else if (middle >= i) lo = i;
                else return;
            }
        }

        private int CompareCentroid(int first, int second, bool xAxis)
        {
            var a = _segments[first]; var b = _segments[second];
            double ac = xAxis ? a.A.x / 2 + a.B.x / 2 : a.A.y / 2 + a.B.y / 2;
            double bc = xAxis ? b.A.x / 2 + b.B.x / 2 : b.A.y / 2 + b.B.y / 2;
            int result = ac.CompareTo(bc);
            return result != 0 ? result : first.CompareTo(second);
        }

        private static Box ExpandedBox(double x0, double y0, double x1, double y1)
        {
            double reserveX = ArithmeticReserve * Math.Max(1, Math.Max(Math.Abs(x0), Math.Abs(x1)));
            double reserveY = ArithmeticReserve * Math.Max(1, Math.Max(Math.Abs(y0), Math.Abs(y1)));
            return new(Math.BitDecrement(x0 - reserveX), Math.BitDecrement(y0 - reserveY),
                Math.BitIncrement(x1 + reserveX), Math.BitIncrement(y1 + reserveY));
        }

        private static double BoxDistanceSquared(double x, double y, Box bounds)
        {
            double dx = Math.Max(0, Math.Max(bounds.MinX - x, x - bounds.MaxX));
            double dy = Math.Max(0, Math.Max(bounds.MinY - y, y - bounds.MaxY));
            // Round the broad-phase bound down; terminal distance arithmetic remains
            // unchanged, including the old degenerate-segment cutoff.
            dx = Math.Max(0, Math.BitDecrement(dx)); dy = Math.Max(0, Math.BitDecrement(dy));
            return Math.Max(0, Math.BitDecrement(dx * dx + dy * dy));
        }

        // The literal separation evaluator uses the polygon-box lower bound
        // only for edges wholly outside the expanded chord box. Never prune a
        // possibly touching edge: its unchanged stationary-point arithmetic
        // must still run. Far subtree bounds are rounded outward/downward.
        internal double QueryCutterSeparation(Point3D a,Point3D b,double radius,
            Func<(PointD A,PointD B),double> evaluate)
        {
            if(_nodes.Length==0)return double.PositiveInfinity;
            double x0=Math.Min(a.X,b.X),x1=Math.Max(a.X,b.X),y0=Math.Min(a.Y,b.Y),y1=Math.Max(a.Y,b.Y);
            var expanded=new Box(x0-radius,y0-radius,x1+radius,y1+radius);
            double radiusSquaredUpper=Math.BitIncrement(radius*radius),minimum=double.PositiveInfinity;
            double Lower(Box bounds)
            {
                double dx=Math.Max(0,Math.Max(bounds.MinX-x1,x0-bounds.MaxX));
                double dy=Math.Max(0,Math.Max(bounds.MinY-y1,y0-bounds.MaxY));
                dx=Math.Max(0,Math.BitDecrement(dx));dy=Math.Max(0,Math.BitDecrement(dy));
                double xx=Math.Max(0,Math.BitDecrement(dx*dx)),yy=Math.Max(0,Math.BitDecrement(dy*dy));
                double square=Math.Max(0,Math.BitDecrement(xx+yy));
                return Math.BitDecrement(square-radiusSquaredUpper);
            }
            void Visit(int ordinal)
            {
                var node=_nodes[ordinal];
                if(!node.Bounds.Intersects(expanded)&&Lower(node.Bounds)>=minimum)return;
                if(node.Left<0)
                {
                    for(int i=node.First;i<node.First+node.Count;i++)minimum=Math.Min(minimum,evaluate(_segments[_order[i]]));
                    return;
                }
                double left=Lower(_nodes[node.Left].Bounds),right=Lower(_nodes[node.Right].Bounds);
                if(left<=right){Visit(node.Left);Visit(node.Right);}else{Visit(node.Right);Visit(node.Left);}
            }
            Visit(0);return minimum;
        }

        public double QueryDistance(double x, double y)
        {
            if (_segments.Length == 0) return 0.0;
            double bestSq = double.PositiveInfinity;
            long visits = 0, tests = 0;
            var scratch=Scratch;int winner=-1,seed=-1;
            if(scratch.NearestOwner is { } prior && prior.TryGetTarget(out var owner)&&ReferenceEquals(owner,this))
            {
                seed=scratch.NearestOrdinal;
                double nominee=DistanceToSegmentSq(x,y,_segments[seed]);
                if(double.IsFinite(nominee)){bestSq=nominee;winner=seed;}
                if(_diagnostics is not null)Interlocked.Increment(ref _diagnostics.NearestSeedTrials);
            }
            // A balanced median tree over an int-sized array has depth below 32.
            Span<int> pending = stackalloc int[64];
            int count = 1; pending[0] = 0;
            while (count > 0)
            {
                Node node = _nodes[pending[--count]];
                if (_diagnostics is not null) visits++;
                if (BoxDistanceSquared(x, y, node.Bounds) > bestSq) continue;
                if (node.Left < 0)
                {
                    for (int i = node.First; i < node.First + node.Count; i++)
                    {
                        double d2 = DistanceToSegmentSq(x, y, _segments[_order[i]]);
                        if (_diagnostics is not null) tests++;
                        if (d2 < bestSq) {bestSq = d2;winner=_order[i];}
                    }
                }
                else
                {
                    double left = BoxDistanceSquared(x, y, _nodes[node.Left].Bounds);
                    double right = BoxDistanceSquared(x, y, _nodes[node.Right].Bounds);
                    if (left <= right) { pending[count++] = node.Right; pending[count++] = node.Left; }
                    else { pending[count++] = node.Left; pending[count++] = node.Right; }
                }
            }
            if (_diagnostics is not null)
            {
                Interlocked.Increment(ref _diagnostics.NearestQueries);
                Interlocked.Add(ref _diagnostics.NearestNodes, visits);
                Interlocked.Add(ref _diagnostics.DistanceTests, tests);
                if(winner>=0&&winner==seed)Interlocked.Increment(ref _diagnostics.NearestSeedWins);
            }
            if(winner>=0)
            {
                // Neighbouring queries often share the closest original wall.
                // Re-evaluate that literal segment at the NEW query, then run
                // the unchanged complete conservative tree. Only a candidate
                // ordinal is reused; never a distance or wall permission.
                if(scratch.NearestOwner is null)scratch.NearestOwner=new(this);
                else scratch.NearestOwner.SetTarget(this);
                scratch.NearestOrdinal=winner;
            }
            return bestSq < double.PositiveInfinity ? Math.Sqrt(bestSq) : 0.0;
        }

        // Endpoints must already have analytical Z from this boundary. This permits the
        // endpoint-distance lower bound used by the Lipschitz shortcut below.
        internal bool TryFindDepthViolation(Point3D a, Point3D b, double startZ,
            double bottomRadius, double radiusSlope, double zTolerance, out double parameter,
            VCarveCutterProfile? cutterProfile = null)
        {
            if (cutterProfile is not null)
                return TryFindProfileDepthViolation(a, b, startZ, cutterProfile, zTolerance, out parameter);
            parameter = 0.5;
            double depthA = startZ - a.Z, depthB = startZ - b.Z;
            if (_segments.Length == 0 || Math.Max(depthA, depthB) <= zTolerance) return false;
            double dx = b.X - a.X, dy = b.Y - a.Y;
            double depthDelta = depthB - depthA;
            double length = Math.Sqrt(dx * dx + dy * dy);
            double endpointRadiusDelta = radiusSlope * depthDelta;
            // Boundary distance is 1-Lipschitz. Positive-depth endpoints provide lower
            // bounds even at the maximum-depth clamp; very short chords need no index query.
            if (Math.Min(depthA, depthB) > zTolerance
                && length * length - endpointRadiusDelta * endpointRadiusDelta
                    <= 2 * length * radiusSlope * zTolerance) return false;

            double lo = 0, hi = 1;
            if (depthA <= zTolerance) lo = (zTolerance - depthA) / depthDelta;
            if (depthB <= zTolerance) hi = (zTolerance - depthA) / depthDelta;
            double radius0 = bottomRadius + radiusSlope * (depthA - zTolerance);
            double radiusDelta = radiusSlope * depthDelta;
            double radius = Math.Max(radius0 + radiusDelta * lo, radius0 + radiusDelta * hi);
            double x0 = a.X + lo * dx, x1 = a.X + hi * dx;
            double y0 = a.Y + lo * dy, y1 = a.Y + hi * dy;
            var candidates = GatherCandidates(x0, y0, x1, y1, radius);

            double minimum = -1e-12;
            double witness = 0.5;
            bool violated = false;
            Span<double> cuts = stackalloc double[4];
            foreach (var candidate in candidates)
            {
                var edge = _segments[candidate.Ordinal];
                double ex = edge.B.x - edge.A.x, ey = edge.B.y - edge.A.y;
                double edgeLengthSquared = ex * ex + ey * ey;
                double ax = a.X - edge.A.x, ay = a.Y - edge.A.y;
                double projection0 = edgeLengthSquared > 0 ? (ax * ex + ay * ey) / edgeLengthSquared : 0;
                double projectionDelta = edgeLengthSquared > 0 ? (dx * ex + dy * ey) / edgeLengthSquared : 0;
                int count = 2;
                cuts[0] = lo; cuts[1] = hi;
                if (projectionDelta != 0)
                {
                    double first = -projection0 / projectionDelta;
                    double last = (1 - projection0) / projectionDelta;
                    if (first > lo && first < hi) cuts[count++] = first;
                    if (last > lo && last < hi) cuts[count++] = last;
                }
                cuts[..count].Sort();
                for (int i = 1; i < count; i++)
                {
                    double low = cuts[i - 1], high = cuts[i];
                    double projection = projection0 + projectionDelta * (low + high) / 2;
                    double qx = ax, qy = ay, vx = dx, vy = dy;
                    if (projection >= 1) { qx -= ex; qy -= ey; }
                    else if (projection > 0)
                    {
                        qx -= projection0 * ex; qy -= projection0 * ey;
                        vx -= projectionDelta * ex; vy -= projectionDelta * ey;
                    }
                    // distance^2 - requiredRadius^2 is quadratic on this projection
                    // interval. Its minimum is at an endpoint or its stationary point.
                    double quadratic = vx * vx + vy * vy - radiusDelta * radiusDelta;
                    double linear = 2 * (qx * vx + qy * vy - radius0 * radiusDelta);
                    void Check(double t)
                    {
                        double x = qx + vx * t, y = qy + vy * t;
                        double required = radius0 + radiusDelta * t;
                        double value = x * x + y * y - required * required;
                        if (value < minimum) { minimum = value; witness = t; violated = true; }
                    }
                    Check(low); Check(high);
                    if (quadratic > 0)
                    {
                        double stationary = -linear / (2 * quadratic);
                        if (stationary > low && stationary < high) Check(stationary);
                    }
                }
            }
            parameter = witness;
            return violated;
        }

        // On each physical section, radius squared is a quadratic of axial depth.
        // This lets the same all-chord proof cover a hemisphere, its taper and a
        // cylindrical plateau without sampling past a section transition.
        private readonly record struct RadiusInterval(double Low, double High,
            double Depth0, double DepthDelta, double BottomRadius, double Slope, double BallRadius)
        {
            public double Quadratic => BallRadius > 0 ? -DepthDelta * DepthDelta
                : Slope * Slope * DepthDelta * DepthDelta;
            public double Linear => BallRadius > 0 ? 2 * (BallRadius - Depth0) * DepthDelta
                : 2 * (BottomRadius + Slope * Depth0) * Slope * DepthDelta;
            public double RadiusSquared(double t)
            {
                double h = Depth0 + DepthDelta * t;
                if (BallRadius > 0) return h * (2 * BallRadius - h);
                double radius = BottomRadius + Slope * h;
                return radius * radius;
            }
        }

        private bool TryFindProfileDepthViolation(Point3D a, Point3D b, double startZ,
            VCarveCutterProfile profile, double zTolerance, out double parameter)
        {
            parameter = 0.5;
            double depthA = startZ - a.Z, depthB = startZ - b.Z;
            double minDepth = Math.Min(depthA, depthB), maxDepth = Math.Max(depthA, depthB);
            if (_segments.Length == 0 || maxDepth <= zTolerance) return false;

            // Retain the proven short-chord shortcut and exact cone arithmetic when
            // the entire depth interval (including its tolerance reserve) is linear.
            double sectionStart = 0;
            foreach (var section in profile.Sections)
            {
                double sectionEnd = Math.Min(profile.MaximumDepth, sectionStart + section.Height);
                if (section.Kind == CarvingCutterSectionKind.Linear
                    && minDepth - zTolerance >= sectionStart && maxDepth <= sectionEnd)
                {
                    double slope = (section.TopRadius - section.BottomRadius) / section.Height;
                    return TryFindDepthViolation(a, b, startZ,
                        section.BottomRadius - slope * sectionStart, slope, zTolerance, out parameter);
                }
                sectionStart = sectionEnd;
                if (sectionStart >= profile.MaximumDepth) break;
            }

            double depthDelta = depthB - depthA;
            double lo = 0, hi = 1;
            if (depthA <= zTolerance) lo = (zTolerance - depthA) / depthDelta;
            if (depthB <= zTolerance) hi = (zTolerance - depthA) / depthDelta;
            var intervals = Scratch.Intervals;
            intervals.Clear();
            sectionStart = 0;
            foreach (var section in profile.Sections)
            {
                double sectionEnd = Math.Min(profile.MaximumDepth, sectionStart + section.Height);
                double low = lo, high = hi;
                if (depthDelta == 0)
                {
                    if (depthA - zTolerance < sectionStart || depthA - zTolerance > sectionEnd)
                    {
                        sectionStart = sectionEnd;
                        if (sectionStart >= profile.MaximumDepth) break;
                        continue;
                    }
                }
                else
                {
                    double t0 = (sectionStart + zTolerance - depthA) / depthDelta;
                    double t1 = (sectionEnd + zTolerance - depthA) / depthDelta;
                    low = Math.Max(low, Math.Min(t0, t1));
                    high = Math.Min(high, Math.Max(t0, t1));
                }
                if (high >= low)
                    intervals.Add(new RadiusInterval(low, high, depthA - zTolerance - sectionStart,
                        depthDelta, section.BottomRadius,
                        (section.TopRadius - section.BottomRadius) / section.Height,
                        section.Kind == CarvingCutterSectionKind.Hemisphere ? section.TopRadius : 0));
                sectionStart = sectionEnd;
                if (sectionStart >= profile.MaximumDepth) break;
            }

            double dx = b.X - a.X, dy = b.Y - a.Y;
            double radius = profile.RadiusAtDepth(maxDepth - zTolerance);
            double x0 = a.X + lo * dx, x1 = a.X + hi * dx;
            double y0 = a.Y + lo * dy, y1 = a.Y + hi * dy;
            var candidates = GatherCandidates(x0, y0, x1, y1, radius);

            double minimum = -1e-12, witness = 0.5;
            bool violated = false;
            Span<double> cuts = stackalloc double[4];
            foreach (var candidate in candidates)
            {
                var edge = _segments[candidate.Ordinal];
                double ex = edge.B.x - edge.A.x, ey = edge.B.y - edge.A.y;
                double edgeLengthSquared = ex * ex + ey * ey;
                double ax = a.X - edge.A.x, ay = a.Y - edge.A.y;
                double projection0 = edgeLengthSquared > 0 ? (ax * ex + ay * ey) / edgeLengthSquared : 0;
                double projectionDelta = edgeLengthSquared > 0 ? (dx * ex + dy * ey) / edgeLengthSquared : 0;
                foreach (var interval in intervals)
                {
                    int count = 2;
                    cuts[0] = interval.Low; cuts[1] = interval.High;
                    if (projectionDelta != 0)
                    {
                        double first = -projection0 / projectionDelta;
                        double last = (1 - projection0) / projectionDelta;
                        if (first > interval.Low && first < interval.High) cuts[count++] = first;
                        if (last > interval.Low && last < interval.High) cuts[count++] = last;
                    }
                    cuts[..count].Sort();
                    for (int i = 1; i < count; i++)
                    {
                        double low = cuts[i - 1], high = cuts[i];
                        double projection = projection0 + projectionDelta * (low + high) / 2;
                        double qx = ax, qy = ay, vx = dx, vy = dy;
                        if (projection >= 1) { qx -= ex; qy -= ey; }
                        else if (projection > 0)
                        {
                            qx -= projection0 * ex; qy -= projection0 * ey;
                            vx -= projectionDelta * ex; vy -= projectionDelta * ey;
                        }
                        double quadratic = vx * vx + vy * vy - interval.Quadratic;
                        double linear = 2 * (qx * vx + qy * vy) - interval.Linear;
                        void Check(double t)
                        {
                            double x = qx + vx * t, y = qy + vy * t;
                            double value = x * x + y * y - interval.RadiusSquared(t);
                            if (value < minimum) { minimum = value; witness = t; violated = true; }
                        }
                        Check(low); Check(high);
                        if (quadratic > 0)
                        {
                            double stationary = -linear / (2 * quadratic);
                            if (stationary > low && stationary < high) Check(stationary);
                        }
                    }
                }
            }
            parameter = witness;
            return violated;
        }

        private List<OrderedCandidate> GatherCandidates(double x0, double y0, double x1, double y1, double radius)
        {
            var candidates = Scratch.Candidates;
            candidates.Clear();
            if (_nodes.Length == 0) return candidates;
            int initialCapacity = candidates.Capacity;
            int c0 = Math.Clamp((int)Math.Floor((Math.Min(x0, x1) - radius - _minX) / _cellSizeX), 0, _cols - 1);
            int c1 = Math.Clamp((int)Math.Floor((Math.Max(x0, x1) + radius - _minX) / _cellSizeX), 0, _cols - 1);
            int r0 = Math.Clamp((int)Math.Floor((Math.Min(y0, y1) - radius - _minY) / _cellSizeY), 0, _rows - 1);
            int r1 = Math.Clamp((int)Math.Floor((Math.Max(y0, y1) + radius - _minY) / _cellSizeY), 0, _rows - 1);
            var cells = new CellRange(c0, r0, c1, r1);
            Box reach = ExpandedBox(Math.Min(x0, x1) - radius, Math.Min(y0, y1) - radius,
                Math.Max(x0, x1) + radius, Math.Max(y0, y1) + radius);
            long visits = 0, tests = 0;
            Span<int> pending = stackalloc int[64];
            int count = 1; pending[0] = 0;
            while (count > 0)
            {
                Node node = _nodes[pending[--count]];
                if (_diagnostics is not null) visits++;
                if (!node.Bounds.Intersects(reach) || !node.Cells.Intersects(cells)) continue;
                if (node.Left < 0)
                {
                    for (int i = node.First; i < node.First + node.Count; i++)
                    {
                        int ordinal = _order[i]; CellRange segmentCells = _cells[ordinal];
                        if (_diagnostics is not null) tests++;
                        if (!_bounds[ordinal].Intersects(reach) || !segmentCells.Intersects(cells)) continue;
                        // Grid loops visit columns, then rows, then original segment
                        // ordinals. A segment first appears at the earliest overlap
                        // of its cell rectangle with the query rectangle.
                        int firstCell = Math.Max(c0, segmentCells.MinC) * _rows + Math.Max(r0, segmentCells.MinR);
                        candidates.Add(new(ordinal, firstCell));
                    }
                }
                else { pending[count++] = node.Right; pending[count++] = node.Left; }
            }
            candidates.Sort();
            if (_diagnostics is not null)
            {
                Interlocked.Increment(ref _diagnostics.CandidateQueries);
                Interlocked.Add(ref _diagnostics.CandidateNodes, visits);
                Interlocked.Add(ref _diagnostics.CandidateLeafTests, tests);
                Interlocked.Add(ref _diagnostics.Candidates, candidates.Count);
                if (candidates.Capacity != initialCapacity) Interlocked.Increment(ref _diagnostics.ScratchGrowths);
            }
            return candidates;
        }

        // Snapshot used by exact reference-oracle diagnostics; production queries
        // consume worker-local scratch directly and do not allocate this array.
        internal int[] CandidateOrdinals(double x0, double y0, double x1, double y1, double radius)
            => GatherCandidates(x0, y0, x1, y1, radius).Select(candidate => candidate.Ordinal).ToArray();

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private static double DistanceToSegmentSq(double x, double y, (PointD A, PointD B) seg)
        {
            var (a, b) = seg;
            double abx = b.x - a.x;
            double aby = b.y - a.y;
            double apx = x - a.x;
            double apy = y - a.y;
            double ab2 = abx * abx + aby * aby;
            double t = ab2 <= 1e-18 ? 0.0 : (apx * abx + apy * aby) / ab2;
            if (t < 0.0) t = 0.0;
            else if (t > 1.0) t = 1.0;
            double cx = a.x + abx * t;
            double cy = a.y + aby * t;
            double dx = x - cx;
            double dy = y - cy;
            return dx * dx + dy * dy;
        }
    }

    private static List<(PointD A, PointD B)> BuildBoundarySegments(
        IReadOnlyList<IReadOnlyList<PointD>> rings)
    {
        var segs = new List<(PointD A, PointD B)>();
        foreach (var ring in rings)
        {
            if (ring.Count < 2) continue;
            // Rings from CanonicalizeRings are closed (first == last).  Iterate
            // pairwise and skip a degenerate closing segment if present.
            for (int i = 0; i < ring.Count - 1; i++)
            {
                var a = ring[i];
                var b = ring[i + 1];
                if (Math.Abs(a.x - b.x) < 1e-15 && Math.Abs(a.y - b.y) < 1e-15)
                    continue;
                segs.Add((a, b));
            }
        }
        return segs;
    }

    /// <summary>
    /// Ramer-Douglas-Peucker simplification on a 3-D polyline.
    /// Removes points whose perpendicular distance from the chord is less than
    /// <paramref name="tolerance"/> (workspace units).
    /// </summary>
    internal static List<Point3D> RdpSimplify3D(List<Point3D> path, double tolerance, bool preserveSharpCorners=true)
    {
        if (path.Count <= 2 || tolerance <= 0) return path;

        var keep = new bool[path.Count];

        // Anchor endpoints plus sharp XY corners so valid spokes/corner features
        // are never removed by global curve simplification.
        double minTurnRad = DefaultRdpCornerAngleDeg * Math.PI / 180.0;
        keep[0] = true;
        keep[path.Count - 1] = true;
        for (int i = 1; i < path.Count - 1; i++)
        {
            if (preserveSharpCorners && IsSharpCornerXY(path[i - 1], path[i], path[i + 1], minTurnRad))
                keep[i] = true;
        }

        // Run RDP independently between adjacent anchors.
        double tolSq = tolerance * tolerance;
        int start = 0;
        while (start < path.Count - 1)
        {
            int end = start + 1;
            while (end < path.Count && !keep[end]) end++;
            if (end >= path.Count) end = path.Count - 1;

            RdpMark(path, start, end, tolSq, keep, boundedSegments:!preserveSharpCorners);
            start = end;
        }

        var result = new List<Point3D>(path.Count);
        for (int i = 0; i < path.Count; i++)
            if (keep[i]) result.Add(path[i]);
        return result;
    }

    private static void RdpMark(
        List<Point3D> path, int start, int end, double tolSq, bool[] keep, bool boundedSegments=false)
    {
        if (end - start <= 1) return;

        var a = path[start];
        var b = path[end];
        double abx = b.X - a.X, aby = b.Y - a.Y, abz = b.Z - a.Z;
        double ab2 = abx * abx + aby * aby + abz * abz;

        double maxDist2 = 0;
        int    split    = start + 1;

        for (int i = start + 1; i < end; i++)
        {
            var p = path[i];
            double apx = p.X - a.X, apy = p.Y - a.Y, apz = p.Z - a.Z;

            double dist2;
            if (ab2 < 1e-24)
            {
                dist2 = apx * apx + apy * apy + apz * apz;
            }
            else if(boundedSegments)
            {
                double t=Math.Clamp((apx*abx+apy*aby+apz*abz)/ab2,0,1);
                double x=apx-t*abx,y=apy-t*aby,z=apz-t*abz;
                dist2=x*x+y*y+z*z;
            }
            else
            {
                // Cross product AP x AB, divided by |AB|^2
                double cx = apy * abz - apz * aby;
                double cy = apz * abx - apx * abz;
                double cz = apx * aby - apy * abx;
                dist2 = (cx * cx + cy * cy + cz * cz) / ab2;
            }

            if (dist2 > maxDist2) { maxDist2 = dist2; split = i; }
        }

        if (maxDist2 > tolSq)
        {
            keep[split] = true;
            RdpMark(path, start, split, tolSq, keep,boundedSegments);
            RdpMark(path, split, end,   tolSq, keep,boundedSegments);
        }
    }

    private static bool IsSharpCornerXY(Point3D prev, Point3D curr, Point3D next, double minTurnRad)
    {
        double v1x = curr.X - prev.X;
        double v1y = curr.Y - prev.Y;
        double v2x = next.X - curr.X;
        double v2y = next.Y - curr.Y;

        double len1 = Math.Sqrt(v1x * v1x + v1y * v1y);
        double len2 = Math.Sqrt(v2x * v2x + v2y * v2y);
        if (len1 < 1e-12 || len2 < 1e-12) return false;

        double dot = (v1x * v2x + v1y * v2y) / (len1 * len2);
        dot = Math.Clamp(dot, -1.0, 1.0);
        double turn = Math.Acos(dot);
        return turn >= minTurnRad;
    }

    /// <summary>
    /// Removes interior points that are collinear with their neighbours,
    /// collapsing many small aligned segments into single long ones.
    ///
    /// Uses a relative cross-product tolerance so it is scale-invariant.
    /// </summary>
    internal static List<Point3D> SimplifyCollinearRuns(List<Point3D> path)
    {
        if (path.Count <= 2) return path;

        var result = new List<Point3D>(path.Count) { path[0] };
        for (int i = 1; i < path.Count - 1; i++)
        {
            var prev = result[^1];
            var curr = path[i];
            var next = path[i + 1];

            if (!IsCollinear3D(prev, curr, next))
                result.Add(curr);
        }
        result.Add(path[^1]);
        return result;
    }

    // Three 3-D points are collinear when |AB × AC|² ≤ ε² |AC|².
    private static bool IsCollinear3D(Point3D a, Point3D b, Point3D c)
    {
        double abx = b.X - a.X, aby = b.Y - a.Y, abz = b.Z - a.Z;
        double acx = c.X - a.X, acy = c.Y - a.Y, acz = c.Z - a.Z;

        double cx = aby * acz - abz * acy;
        double cy = abz * acx - abx * acz;
        double cz = abx * acy - aby * acx;

        double cross2 = cx * cx + cy * cy + cz * cz;
        double ac2    = acx * acx + acy * acy + acz * acz;

        // Native terminal trails revisit branch junctions. Removing a reversal
        // would delete its entire cutting excursion, even when the points lie
        // on one line. B must project onto the segment from A through C.
        if (ac2 == 0)
            return abx == 0 && aby == 0 && abz == 0;
        double projection = abx * acx + aby * acy + abz * acz;
        if (projection < 0 || projection > ac2)
            return false;

        const double eps = 1e-9;
        return cross2 <= eps * eps * ac2;
    }

}
