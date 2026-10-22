using System.Globalization;
using Clipper2Lib;

namespace RW.Toolpaths;

/// <summary>
/// Shared medial geometry in workspace coordinates. Providers implement the native diagram;
/// this entry point owns its coordinate conditioning and the return to physical distances.
/// </summary>
public static partial class MedialAxisGeometry
{
    private const double MaximumProviderSpan = 100_000_000;

    /// <summary>
    /// Builds one outer boundary and its holes, returning workspace XY and clearance radii.
    /// Provider tolerance and filtering angle retain their provider-defined angular semantics.
    /// A nonpositive maximum radius requests the complete local diagram. The optional coordinate
    /// scale retains existing V-carve precision retries without duplicating provider conversion.
    /// </summary>
    public static IReadOnlyList<MedialSegment> Build(
        IReadOnlyList<PointD> boundary,
        IReadOnlyList<IReadOnlyList<PointD>> holes,
        IMedialAxisProvider provider,
        double tolerance,
        double maxRadius,
        double filteringAngle = 3 * Math.PI / 4,
        CancellationToken cancellationToken = default,
        Action<string>? diagnostic = null,
        Action<double, string, bool>? progress = null,
        double coordinateScale = PathUtils.Scale,
        double? maximumGeometricError = null, bool preserveNativeGrid = false)
        => BuildCore(boundary, holes, provider, tolerance, maxRadius, filteringAngle,
            cancellationToken, diagnostic, progress, coordinateScale, maximumGeometricError, false, preserveNativeGrid);

    internal static IReadOnlyList<MedialSegment> BuildBoundedSource(
        IReadOnlyList<PointD> boundary, IReadOnlyList<IReadOnlyList<PointD>> holes,
        double tolerance, double maxRadius, double filteringAngle,
        CancellationToken cancellationToken, double coordinateScale = PathUtils.Scale,
        Action<string>? diagnostic = null, bool preserveNativeGrid = false)
        => BuildCore(boundary, holes, BoostVoronoiProvider.CreateDefault(), tolerance, maxRadius,
            filteringAngle, cancellationToken, diagnostic, null, coordinateScale, null, true, preserveNativeGrid);

    private static IReadOnlyList<MedialSegment> BuildCore(
        IReadOnlyList<PointD> boundary, IReadOnlyList<IReadOnlyList<PointD>> holes,
        IMedialAxisProvider provider, double tolerance, double maxRadius, double filteringAngle,
        CancellationToken cancellationToken, Action<string>? diagnostic,
        Action<double, string, bool>? progress, double coordinateScale,
        double? maximumGeometricError, bool boundedSource, bool preserveNativeGrid)
    {
        ArgumentNullException.ThrowIfNull(boundary);
        ArgumentNullException.ThrowIfNull(holes);
        ArgumentNullException.ThrowIfNull(provider);
        cancellationToken.ThrowIfCancellationRequested();
        if (!double.IsFinite(tolerance) || tolerance <= 0)
            throw new ArgumentOutOfRangeException(nameof(tolerance));
        if (!double.IsFinite(maxRadius))
            throw new ArgumentOutOfRangeException(nameof(maxRadius));
        if (!double.IsFinite(filteringAngle))
            throw new ArgumentOutOfRangeException(nameof(filteringAngle));
        if (!double.IsFinite(coordinateScale) || coordinateScale <= 0)
            throw new ArgumentOutOfRangeException(nameof(coordinateScale));
        if (maximumGeometricError is { } geometricError && (!double.IsFinite(geometricError) || geometricError <= 0))
            throw new ArgumentOutOfRangeException(nameof(maximumGeometricError));
        if (boundary.Count < 3)
            return Array.Empty<MedialSegment>();

        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        Include(boundary);
        foreach (var hole in holes)
        {
            ArgumentNullException.ThrowIfNull(hole);
            Include(hole);
        }
        double width = maxX - minX, height = maxY - minY;
        double span = Math.Max(width, height);
        if (!double.IsFinite(span))
            throw new ArgumentException("Medial geometry spans exceed the finite coordinate range.", nameof(boundary));
        if (span <= 0)
            return Array.Empty<MedialSegment>();
        double originX = minX + width / 2, originY = minY + height / 2;
        double diagonal = span * Math.Sqrt(1 + Math.Pow(Math.Min(width, height) / span, 2));
        double requestedRadius = maxRadius > 0 ? maxRadius : diagonal;
        double scale = Math.Min(coordinateScale, MaximumProviderSpan / span);
        // The native segment-site bridge accepts signed 32-bit sites. Radius is
        // also bounded without capping its physical value: reduce the common scale.
        scale = Math.Min(scale, (int.MaxValue - 1d) / requestedRadius);
        if (!double.IsFinite(scale) || scale <= 0)
            throw new ArgumentException("Medial geometry cannot be represented in the provider coordinate range.", nameof(boundary));
        // An integral source translated by half a native tick is rounded again
        // with alternating ties. That can cross neighbouring edges and feed
        // invalid segment sites to Boost. Use an integral translation instead.
        if (preserveNativeGrid)
        {
            originX = Math.Round(originX * scale) / scale;
            originY = Math.Round(originY * scale) / scale;
        }
        var scaledBoundary = ScaleRing(boundary);
        var scaledHoles = holes.Select(hole => (IReadOnlyList<PointD>)ScaleRing(hole)).ToList();
        string parameters = string.Create(CultureInfo.InvariantCulture,
            $"provider={provider.GetType().Name} points={boundary.Count} holes={holes.Count} " +
            $"origin=({originX:R},{originY:R}) scale={scale:R} providerTolerance={tolerance:R} " +
            $"requestedRadius={maxRadius:R} effectiveRadius={requestedRadius:R} filteringAngle={filteringAngle:R} nativeGridTranslation={preserveNativeGrid} " +
            $"sourceQuantization={(preserveNativeGrid ? "global-signed-mask-grid" : "provider-rounding")}");
        diagnostic?.Invoke($"stage=medial-geometry event=start {parameters}");
        long started = PerfLog.Start();
        IReadOnlyList<MedialSegment> Construct(IReadOnlyList<PointD> outer,IReadOnlyList<IReadOnlyList<PointD>> inner)
            => boundedSource && provider is BoostVoronoiProvider sourceProvider
            ? sourceProvider.ConstructBoundedSourceMedialAxis(outer, inner, tolerance,
                requestedRadius * scale, filteringAngle, cancellationToken, diagnostic, progress,
                new(originX, originY, scale))
            : maximumGeometricError is { } maximumError && provider is BoostVoronoiProvider exact
            ? exact.ConstructCertifiedMedialAxis(outer, inner, maximumError * scale,
                requestedRadius * scale, filteringAngle, cancellationToken, diagnostic, progress)
            : provider.ConstructMedialAxis(outer, inner, tolerance,
                requestedRadius * scale, filteringAngle, true, cancellationToken, diagnostic, progress);
        var native=Construct(scaledBoundary,scaledHoles);
        cancellationToken.ThrowIfCancellationRequested();
        var result = new List<MedialSegment>(native.Count);
        int outsideBounds = 0;
        double boundsTolerance = 2 / scale;
        foreach (var segment in native)
        {
            if ((result.Count & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            var a = World(segment.Point0);
            var b = World(segment.Point1);
            if (Outside(a) || Outside(b)) outsideBounds++;
            result.Add(new MedialSegment(a, b));
        }
        diagnostic?.Invoke($"stage=medial-geometry event=complete segments={result.Count} outsideBounds={outsideBounds} {parameters}");
        PerfLog.Stop("MedialAxisGeometry.Build", started, $"segments={result.Count} outsideBounds={outsideBounds} {parameters}");
        if (outsideBounds > 0 && string.Equals(Environment.GetEnvironmentVariable("RW_TOOLPATHS_MEDIAL_VALIDATE"), "1", StringComparison.Ordinal))
            Console.Error.WriteLine($"[Medial.Geometry.Bounds] outside={outsideBounds}/{result.Count} {parameters}");
        return result;

        void Include(IReadOnlyList<PointD> ring)
        {
            for (int i = 0; i < ring.Count; i++)
            {
                if ((i & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                PointD p = ring[i];
                if (!double.IsFinite(p.x) || !double.IsFinite(p.y))
                    throw new ArgumentException("Medial boundaries require finite coordinates.", nameof(boundary));
                minX = Math.Min(minX, p.x); maxX = Math.Max(maxX, p.x);
                minY = Math.Min(minY, p.y); maxY = Math.Max(maxY, p.y);
            }
        }
        List<PointD> ScaleRing(IReadOnlyList<PointD> ring)
        {
            var scaled = new List<PointD>(ring.Count);
            for (int i = 0; i < ring.Count; i++)
            {
                if ((i & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                PointD p = ring[i];
                // Quantize once in the signed physical mask's global grid.
                // Rounding half-grid sites after an odd integer translation
                // changes ties-to-even ownership and can cross adjacent edges.
                scaled.Add(ConditionSourcePoint(p, originX, originY, scale, preserveNativeGrid));
            }
            return scaled;
        }
        MedialPoint World(MedialPoint p)
        {
            if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Radius) || p.Radius < 0)
                throw new InvalidOperationException($"Medial provider returned invalid endpoint ({p.X:R},{p.Y:R},{p.Radius:R}). {parameters}");
            return new MedialPoint(p.X / scale + originX, p.Y / scale + originY, p.Radius / scale);
        }
        bool Outside(MedialPoint p) => p.X < minX - boundsTolerance || p.X > maxX + boundsTolerance
            || p.Y < minY - boundsTolerance || p.Y > maxY + boundsTolerance;
    }

    internal static PointD ConditionSourcePoint(PointD p, double originX, double originY,
        double scale, bool preserveNativeGrid) => preserveNativeGrid ? new PointD(
            Math.Round(p.x * scale) - Math.Round(originX * scale),
            Math.Round(p.y * scale) - Math.Round(originY * scale))
            : new PointD((p.x - originX) * scale, (p.y - originY) * scale);
}
