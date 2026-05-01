using System.Runtime.CompilerServices;
using Clipper2Lib;

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
///   Zero-based depth-layer index for clearing passes; <c>null</c> for non-layered
///   passes (e.g. final carve).
/// </param>
public readonly record struct TaggedToolpath(
    IReadOnlyList<Point3D> Points,
    int RegionIndex,
    string Category,
    int? DepthPassIndex);

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
    ///   Maximum cell angle (in radians) above which a medial-axis edge is
    ///   filtered out.  Prunes edges at sharp interior corners.
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
        double filteringAngle = 3 * Math.PI / 4,
        bool   useBigIntegers = true);
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
    /// Uses the same preprocessing/grouping/provider contract as the production
    /// path, but keeps radius values in XY-space (no Z conversion, no trimming).
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

        List<List<PointD>>? simplifiedLayer = null;
        IReadOnlyList<MedialSegment> raw = Array.Empty<MedialSegment>();

        for (int attempt = 0; attempt < 4; attempt++)
        {
            simplifiedLayer = GenerateSimplifiedLayer(normalizedBoundary, scale);
            if (simplifiedLayer is null || simplifiedLayer.Count == 0)
                return Array.Empty<MedialSegment>();

            raw = GenerateMedialAxis(provider, simplifiedLayer, maxRadius, scale, tolerance);

            // For preview mode with explicit override, skip bbox-breakout filtering
            // and return what the kernel produced.
            if (maxRadiusOverride is not null)
                return raw;

            double maxR = 0;
            foreach (var seg in raw)
                maxR = Math.Max(maxR, Math.Max(seg.Point0.Radius, seg.Point1.Radius));

            var bb = BoundingBox(simplifiedLayer.SelectMany(r => r));
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
        double? coneLengthOverride = null)
    {
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
            coneLengthOverride);

        if (result is null) return null;
        var (medialAxis, flatAreaRegion) = result.Value;

        // Trim segments to final depth and collect flat-area strokes
        var flatStrokes  = new List<(PointD A, PointD B)>();
        var trimmed      = medialAxis
            .Select(seg => TrimSegment(seg, endZ, flatStrokes))
            .Where(s => s is not null)
            .Select(s => s!.Value)
            .ToList();

        // Join adjacent flat-area segments
        var joinedFlat = JoinSegments(flatStrokes);

        // Generate concentric fill for the flat bottom area
        var flatFill = GenerateFlatAreaFill(joinedFlat, flatAreaRegion, endZ);
        trimmed.AddRange(flatFill);

        // Connect segments into continuous polylines.
        var polylines = JoinIntoPolylines(trimmed);
        if (polylines is null) return null;

        if (rdpTolerance > 0)
        {
            for (int i = 0; i < polylines.Count; i++)
                polylines[i] = RdpSimplify3D(polylines[i], rdpTolerance);
        }
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
    public static List<List<Point3D>> GenerateClearingPasses(
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double startDepth,
        double endDepth,
        double radianTipAngle,
        double depthPerPass,
        double stepOver = 0.4,
        double? bottomRadiusOverride = null,
        double? topRadiusOverride = null,
        double? coneLengthOverride = null)
    {
        var tagged = GenerateClearingPassesTagged(
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            regionIndex: 0,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride);

        return tagged.Select(t => t.Points.ToList()).ToList();
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
        double? coneLengthOverride = null)
    {
        if (endDepth <= startDepth) return new();

        var normalizedBoundary = NormalizeBoundaryRings(boundary);
        if (normalizedBoundary.Count == 0)
            return new();

        // Keep clearing offsets in the same geometric domain family used by
        // medial-axis planning to avoid tiny red/blue seam mismatches.
        var clearingBoundary = GenerateSimplifiedLayer(normalizedBoundary, InitialScale)
            ?? normalizedBoundary;

        var profile = ResolveToolProfile(
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride);

        double bitDiameter = 2.0 * profile.FinalRadius;
        if (bitDiameter <= 0) return new();

        double T = stepOver * bitDiameter;
        double M = 0.05 * bitDiameter;
        const double B = 2.0;
        double _t = B * M;

        double firstDepth = Math.Floor(startDepth / depthPerPass) * depthPerPass;

        // layers: (z, zTop, rings, isBottomLayer, effectiveRadiusAtLayer)
        var layers = new List<(double z, double zTop, List<List<PointD>> rings, bool isBottom, double layerRadius)>();

        double N = firstDepth;
        double k = firstDepth + depthPerPass;
        double b = endDepth;


        const double Eps = 1e-5;
        bool CloseTo(double a, double x) => Math.Abs(a - x) < Eps;

        // Intermediate depth layers (depths strictly before endDepth)
        while (k < b && !CloseTo(k, b))
        {
            double depthFromStart = Math.Max(0.0, k - startDepth);
            double contractAmount = profile.BottomRadius + profile.RadiusSlope * depthFromStart;
            var layer = Contract(clearingBoundary, contractAmount);
            if (layer.Count > 0)
                layers.Add((-k, -N, layer, false, contractAmount));
            N = k;
            k += depthPerPass;
        }

        // Bottom layer (k â‰ˆ endDepth after loop)
        //         = boundary contracted by _t  (small clearance)
        //         = boundary contracted to full depth
        var bottomLayerRings = Contract(clearingBoundary, _t);
        var contracted       = Contract(clearingBoundary, profile.FinalRadius);

        if (contracted.Count > 0)
        {
            var bt = IntersectPolygons(bottomLayerRings, contracted);

            if (CloseTo(k, b))
            {
                // Normal case: final layer is exactly at endDepth.
                // Only the contracted core (bt) is safe to mill: the V-bit cutting
                // radius at endDepth == profile.FinalRadius, so any tool-centre path
                // closer to the boundary than that would cut outside the glyph.
                if (bt.Count > 0)
                    layers.Add((-k, -N, bt, true, profile.FinalRadius));
            }
            else if (bt.Count > 0)
            {
                // Overshot endDepth slightly â€” just add bottom layer.
                layers.Add((-b, -N, bt, true, profile.FinalRadius));
            }
        }

        // Generate OffsetFill toolpaths for each layer
        var result = new List<TaggedToolpath>();
        int depthPassIndex = 0;
        foreach (var (z, zTop, rings, isBottom, layerRadius) in layers)
        {
            double layerDiameter = Math.Max(0.0, 2.0 * layerRadius);
            double normalStepOver = stepOver * layerDiameter;
            double finishStepOver = 0.05 * layerDiameter;

            double stepOverDist = isBottom ? finishStepOver : normalStepOver;
            if (stepOverDist <= 0)
                stepOverDist = isBottom ? M : T;
            if (stepOverDist <= 0)
                stepOverDist = T;

            var paths = OffsetFill.Generate(rings, z, zTop, stepOverDist);
            foreach (var path in paths)
            {
                result.Add(new TaggedToolpath(SimplifyCollinearRuns(path), regionIndex, "clearing", depthPassIndex));
            }

            depthPassIndex++;
        }

        return result;
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
        double? coneLengthOverride = null)
    {
        var tagged = GenerateVCarveTagged(
            provider,
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            tolerance,
            regionIndex: 0,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride);

        return tagged.Select(t => t.Points.ToList()).ToList();
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
        double? coneLengthOverride = null)
    {
        long t0 = PerfLog.Start();
        var result = new List<TaggedToolpath>();

        // Clearing passes (clear-main)
        var clearing = GenerateClearingPassesTagged(
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            regionIndex,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride);
        result.AddRange(clearing);

        // Final V-carve pass (carve-main)
        var finalPass = Generate(
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
            coneLengthOverride);
        if (finalPass is not null)
        {
            foreach (var path in finalPass)
                result.Add(new TaggedToolpath(path, regionIndex, "final-carve", null));
        }

        PerfLog.Stop(
            "MedialAxisToolpaths.GenerateVCarve",
            t0,
            $"rings={boundary.Count} clearing={clearing.Count} final={(finalPass?.Count ?? 0)}");

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
            double? coneLengthOverride = null)
    {
        var tagged = GenerateVCarveComponentsTagged(
            provider,
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            tolerance,
            rdpTolerance,
            regionIndex: 0,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride);

        return (
            tagged.ClearingPasses.Select(t => t.Points.ToList()).ToList(),
            tagged.FinalPass.Select(t => t.Points.ToList()).ToList());
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
            double? coneLengthOverride = null)
    {
        long t0 = PerfLog.Start();
        var clearing = GenerateClearingPassesTagged(
            boundary,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            regionIndex,
            bottomRadiusOverride,
            topRadiusOverride,
            coneLengthOverride);
        var finalGeometry = Generate(
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
            coneLengthOverride)
            ?? new List<List<Point3D>>();
        var final = finalGeometry
            .Select(path => new TaggedToolpath(path, regionIndex, "final-carve", null))
            .ToList();

        PerfLog.Stop(
            "MedialAxisToolpaths.GenerateVCarveComponents",
            t0,
            $"rings={boundary.Count} clearing={clearing.Count} final={final.Count}");

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
        double? coneLengthOverride = null)
    {
        if (provider is null) throw new ArgumentNullException(nameof(provider));
        if (regions is null) throw new ArgumentNullException(nameof(regions));

        var result = new List<TaggedToolpath>();
        int regionIndex = 0;

        foreach (var region in regions)
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
                coneLengthOverride);

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
            double? coneLengthOverride)
    {
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

        double maxRadius = profile.PassRadius;
        double invSlope = 1.0 / profile.RadiusSlope;

        double scale    = InitialScale;
        string? lastErr  = null;

        List<List<PointD>>?       simplifiedLayer = null;
        List<MedialSegment>       medialAxis      = new();

        for (int attempt = 0; attempt < 4; attempt++)
        {
            long tAttempt = PerfLog.Start();
            // Simplify/scale the polygon layer
            simplifiedLayer = GenerateSimplifiedLayer(normalizedBoundary, scale);
            if (simplifiedLayer is null || simplifiedLayer.Count == 0)
            {
                PerfLog.Stop("MedialAxisToolpaths.GenerateMedialAxisToolpaths.Attempt", tAttempt, $"attempt={attempt + 1} empty-simplified");
                PerfLog.Stop("MedialAxisToolpaths.GenerateMedialAxisToolpaths", t0, "empty-simplified");
                return null;
            }

            // Compute medial axis via Voronoi-of-segments
            var rawSegments = GenerateMedialAxis(provider, simplifiedLayer, maxRadius, scale, tolerance);

            double maxR    = 0;
            medialAxis.Clear();
            foreach (var seg in rawSegments)
            {
                double r = Math.Max(seg.Point0.Radius, seg.Point1.Radius);
                if (r > maxR) maxR = r;

                // Convert radius to Z depth: z = -startDepth - radius / halfTan
                double d0 = Math.Max(0.0, (seg.Point0.Radius - profile.BottomRadius) * invSlope);
                double d1 = Math.Max(0.0, (seg.Point1.Radius - profile.BottomRadius) * invSlope);
                medialAxis.Add(new MedialSegment(
                    new MedialPoint(seg.Point0.X, seg.Point0.Y,
                        -startDepth - d0),
                    new MedialPoint(seg.Point1.X, seg.Point1.Y,
                        -startDepth - d1)));
            }

            // Success condition: no radius exceeds 55 % of bounding box minor axis
            var bb = BoundingBox(simplifiedLayer.SelectMany(r => r));
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
        var flatArea = Contract(simplifiedLayer!, profile.FinalRadius);

        PerfLog.Stop(
            "MedialAxisToolpaths.GenerateMedialAxisToolpaths",
            t0,
            $"rings={boundary.Count} segments={medialAxis.Count} flatRings={flatArea.Count}");

        return (medialAxis, flatArea);
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

    // --- generateSimplifiedLayer (contour-pass) ----------------------------------------

    /// <summary>
    /// Returns the polygon region at the first cut depth, simplified and scaled
    /// to integer coordinates for numerical stability.
    ///
    /// Port of <c>gh(contour-pass(n), scale)</c> which expands to:
    /// <code>
    ///   n.simplify(undefined, scale)
    ///    .contract(OFFSET_DISTANCE, MiterJoin)
    ///    .expand(2 * OFFSET_DISTANCE, MiterJoin)
    ///    .contract(OFFSET_DISTANCE, MiterJoin)
    ///    .simplify(undefined, scale)
    ///    .clean()
    /// </code>
    /// where <c>OFFSET_DISTANCE = 16 / 32768</c> workspace units.  The
    /// contract->expand->contract morphological pass removes sharp concavities
    /// that would destabilise the Voronoi/medial-axis computation.
    /// </summary>
    internal static List<List<PointD>>? GenerateSimplifiedLayer(
        IReadOnlyList<IReadOnlyList<PointD>> boundary,
        double scale)
    {
        long t0 = PerfLog.Start();
        // The algorithm works in an integer coordinate space where
        //   1 unit  = (1 / scale) workspace units
        //   1 unit  = (PathUtils.Scale / scale) Clipper64 integer units
        // With scale = p_ = 4096:
        //   OFFSET_DISTANCE = 16/32768 workspace  =  (16/32768)*4096 = 2 integer units
        //   Clipper64 delta = 2 * (PathUtils.Scale / scale) = 2 * (32768/4096) = 16
        // OFFSET_DISTANCE = 16/32768 workspace units.
        // Clipper64 integer coords here = workspace_coord * scale
        // -> delta = (16/32768) * scale  (e.g. scale=4096 -> delta=2; scale=32768 -> delta=16)
        long morphDelta = (long)Math.Round(16.0 / 32768.0 * scale);

        // 1. Scale up to [scale]-scaled Clipper64 coords + lighten (= simplify)
        // active scale to preserve the same geometric threshold.
        double simplifyTolerance = 64.0 * (scale / PathUtils.Scale);
        if (simplifyTolerance < 1.0) simplifyTolerance = 1.0;

        var scaled = PathUtils.ToClipper(boundary.Select(r => r.Select(p =>
            new PointD(p.x * scale / PathUtils.Scale, p.y * scale / PathUtils.Scale))));

        // Match medial-axis preprocessing simplification strength
        // without affecting other pipelines that use PathUtils.Lighten.
        var pass1 = PathUtils.Lighten(scaled, simplifyTolerance);
        if (pass1.Count == 0)
        {
            PerfLog.Stop("MedialAxisToolpaths.GenerateSimplifiedLayer", t0, $"scale={scale:R} input={boundary.Count} pass1=0");
            return null;
        }

        // 2. contract(OFFSET_DISTANCE, Miter) -> expand(2*OFFSET_DISTANCE, Miter) -> contract(OFFSET_DISTANCE, Miter)
        //    Net: zero offset, but removes sharp inward features (opening morph)
        var contracted1 = Clipper.InflatePaths(pass1,         -morphDelta, JoinType.Miter, EndType.Polygon);
        if (contracted1.Count == 0)
        {
            PerfLog.Stop("MedialAxisToolpaths.GenerateSimplifiedLayer", t0, $"scale={scale:R} input={boundary.Count} contracted1=0");
            return null;
        }
        var expanded    = Clipper.InflatePaths(contracted1,  2 * morphDelta, JoinType.Miter, EndType.Polygon);
        if (expanded.Count == 0)
        {
            PerfLog.Stop("MedialAxisToolpaths.GenerateSimplifiedLayer", t0, $"scale={scale:R} input={boundary.Count} expanded=0");
            return null;
        }
        var contracted2 = Clipper.InflatePaths(expanded,       -morphDelta, JoinType.Miter, EndType.Polygon);
        if (contracted2.Count == 0)
        {
            PerfLog.Stop("MedialAxisToolpaths.GenerateSimplifiedLayer", t0, $"scale={scale:R} input={boundary.Count} contracted2=0");
            return null;
        }

        // 3. simplify again + clean (drop rings with < 3 points)
        var pass2 = PathUtils.Lighten(contracted2, simplifyTolerance);
        if (pass2.Count == 0)
        {
            PerfLog.Stop("MedialAxisToolpaths.GenerateSimplifiedLayer", t0, $"scale={scale:R} input={boundary.Count} pass2=0");
            return null;
        }

        // 4. Scale back to workspace coordinates
        var simplified = PathUtils.FromClipper(pass2)
            .Select(r => r.Select(p => new PointD(p.x * PathUtils.Scale / scale, p.y * PathUtils.Scale / scale))
                          .ToList())
            .Where(r => r.Count >= 3)   // clean(): discard degenerate rings
            .ToList<List<PointD>>();

        PerfLog.Stop(
            "MedialAxisToolpaths.GenerateSimplifiedLayer",
            t0,
            $"scale={scale:R} input={boundary.Count} output={simplified.Count}");

        return simplified;
    }

    // --- generateMedialAxis (ridge-pass) ----------------------------------------------

    /// <summary>
    /// Computes the medial axis using the injected <paramref name="provider"/>.
    ///
    /// Port of <c>ridge-pass(n, t, scaleCtx)</c> where t = maxRadius.
    /// </summary>
    private static IReadOnlyList<MedialSegment> GenerateMedialAxis(
        IMedialAxisProvider provider,
        IReadOnlyList<List<PointD>> simplifiedLayer,
        double maxRadius,
        double scale,
        double tolerance)
    {
        long t0 = PerfLog.Start();
        // Group into boundary + holes via NonIntersectingPathGroups
        var groups = NonIntersectingPathGroups(simplifiedLayer);
        var result = new List<MedialSegment>();
        int groupCount = 0;
        double scaleInv = 1.0 / scale;

        foreach (var (outerBoundary, holes) in groups)
        {
            groupCount++;
            long tGroup = PerfLog.Start();
            var scaledBoundary = new List<PointD>(outerBoundary.Count);
            for (int i = 0; i < outerBoundary.Count; i++)
            {
                var p = outerBoundary[i];
                scaledBoundary.Add(new PointD(p.x * scale, p.y * scale));
            }

            var scaledHoles = new List<IReadOnlyList<PointD>>(holes.Count);
            for (int h = 0; h < holes.Count; h++)
            {
                var hole = holes[h];
                var scaledHole = new List<PointD>(hole.Count);
                for (int i = 0; i < hole.Count; i++)
                {
                    var p = hole[i];
                    scaledHole.Add(new PointD(p.x * scale, p.y * scale));
                }

                scaledHoles.Add(scaledHole);
            }

            // A negative maxRadius signals "no limit" (preview mode).
            // Keep the value finite and within 32-bit signed range because the
            // WASM bridge may coerce this argument into an int-like path.
            var bb = BoundingBox(outerBoundary);
            double width = Math.Max(0, bb.maxX - bb.minX);
            double height = Math.Max(0, bb.maxY - bb.minY);
            double noLimitRadius = Math.Sqrt(width * width + height * height);
            if (noLimitRadius <= 0) noLimitRadius = Math.Max(width, height);
            if (noLimitRadius <= 0) noLimitRadius = 1.0;

            double requestedRadius = maxRadius > 0 ? maxRadius : noLimitRadius;
            double providerMaxRadius = scale * requestedRadius;

            var segments = provider.ConstructMedialAxis(
                scaledBoundary,
                scaledHoles,
                tolerance:      tolerance,
                maxRadius:      providerMaxRadius,
                filteringAngle: 3.0 * Math.PI / 4.0,
                useBigIntegers: true);

            if (segments.Count > 0)
                result.Capacity = Math.Max(result.Capacity, result.Count + segments.Count);

            foreach (var seg in segments)
            {
                result.Add(new MedialSegment(
                    new MedialPoint(seg.Point0.X * scaleInv, seg.Point0.Y * scaleInv,
                                    seg.Point0.Radius * scaleInv),
                    new MedialPoint(seg.Point1.X * scaleInv, seg.Point1.Y * scaleInv,
                                    seg.Point1.Radius * scaleInv)));
            }

            if (IsMedialBoundsValidationEnabled())
            {
                // Optional debug validation, disabled by default for perf-sensitive runs.
                int oob = 0;
                for (int s = 0; s < segments.Count; s++)
                {
                    var seg = segments[s];
                    double x0 = seg.Point0.X * scaleInv;
                    double y0 = seg.Point0.Y * scaleInv;
                    double x1 = seg.Point1.X * scaleInv;
                    double y1 = seg.Point1.Y * scaleInv;
                    bool outside = x0 < bb.minX - 1e-6 || x0 > bb.maxX + 1e-6 || y0 < bb.minY - 1e-6 || y0 > bb.maxY + 1e-6
                                || x1 < bb.minX - 1e-6 || x1 > bb.maxX + 1e-6 || y1 < bb.minY - 1e-6 || y1 > bb.maxY + 1e-6;
                    if (outside) oob++;
                }

                if (oob > 0)
                    Console.Error.WriteLine($"[medial-axis] {oob}/{segments.Count} segments have endpoints outside polygon bbox ({bb.minX:F4},{bb.minY:F4})-({bb.maxX:F4},{bb.maxY:F4}) pts={outerBoundary.Count}");
            }

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
        double endZ)
    {
        var regionList = flatAreaRegion as IList<List<PointD>> ?? flatAreaRegion.ToList();

        // When the flat area is empty every stroke lies "outside" it -> return all unchanged.
        if (regionList.Count == 0)
        {
            foreach (var polyline in flatStrokes)
            {
                PointD? prev = null;
                foreach (var pt in polyline)
                {
                    if (prev is not null)
                        yield return new MedialSegment(
                            new MedialPoint(prev.Value.x, prev.Value.y, endZ),
                            new MedialPoint(pt.x, pt.y, endZ));
                    prev = pt;
                }
            }
            yield break;
        }

        // Non-empty flat area: clip each stroke to OUTSIDE the flat-area polygon
        // (Clipper Difference: subject = open stroke, clip = closed flat area).
        var clipPaths = PathUtils.ToClipper(regionList.Select(r => r.Select(p => p)));

        foreach (var polyline in flatStrokes)
        {
            if (polyline.Count < 2) continue;

            var clipper = new Clipper64();
            clipper.AddOpenSubject(PathUtils.ToClipper(new[] { polyline })[0]);
            foreach (var cp in clipPaths)
                clipper.AddClip(cp);

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
                        yield return new MedialSegment(
                            new MedialPoint(prev.Value.x, prev.Value.y, endZ),
                            new MedialPoint(pt.x, pt.y, endZ));
                    prev = pt;
                }
            }
        }
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
    private static IEnumerable<(List<PointD> Boundary, List<List<PointD>> Holes)>
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

    private static bool IsMedialBoundsValidationEnabled() =>
        string.Equals(
            Environment.GetEnvironmentVariable("RW_TOOLPATHS_MEDIAL_VALIDATE"),
            "1",
            StringComparison.Ordinal);

    /// <summary>Contract (inward offset) all rings by <paramref name="amount"/>.</summary>
    private static List<List<PointD>> Contract(
        IEnumerable<IEnumerable<PointD>> rings, double amount)
    {
        var paths  = PathUtils.ToClipper(rings);
        var result = Clipper2Lib.Clipper.InflatePaths(
            paths, -amount * PathUtils.Scale,
            JoinType.Round, EndType.Polygon);
        return PathUtils.FromClipper(result);
    }

    /// <summary>Returns the intersection of two polygon sets.</summary>
    private static List<List<PointD>> IntersectPolygons(
        IEnumerable<IEnumerable<PointD>> a, IEnumerable<IEnumerable<PointD>> b)
    {
        var pathsA = PathUtils.ToClipper(a);
        var pathsB = PathUtils.ToClipper(b);
        if (pathsA.Count == 0 || pathsB.Count == 0) return new();
        var result = Clipper2Lib.Clipper.BooleanOp(
            ClipType.Intersection, pathsA, pathsB, FillRule.NonZero);
        return PathUtils.FromClipper(result);
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

    /// <summary>
    /// Ramer-Douglas-Peucker simplification on a 3-D polyline.
    /// Removes points whose perpendicular distance from the chord is less than
    /// <paramref name="tolerance"/> (workspace units).
    /// </summary>
    internal static List<Point3D> RdpSimplify3D(List<Point3D> path, double tolerance)
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
            if (IsSharpCornerXY(path[i - 1], path[i], path[i + 1], minTurnRad))
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

            RdpMark(path, start, end, tolSq, keep);
            start = end;
        }

        var result = new List<Point3D>(path.Count);
        for (int i = 0; i < path.Count; i++)
            if (keep[i]) result.Add(path[i]);
        return result;
    }

    private static void RdpMark(
        List<Point3D> path, int start, int end, double tolSq, bool[] keep)
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
            RdpMark(path, start, split, tolSq, keep);
            RdpMark(path, split, end,   tolSq, keep);
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

        const double eps = 1e-9;
        return cross2 <= eps * eps * ac2;
    }

}


