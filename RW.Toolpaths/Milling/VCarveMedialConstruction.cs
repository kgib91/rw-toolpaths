using Clipper2Lib;
using Legacy = ClipperLib;

namespace RW.Toolpaths.Milling;

/// <summary>Ordinary V-carve construction in an explicitly defined physical grid.
/// The prepared polygon nominates the medial graph; it is not cutter-fit authority.</summary>
public static class VCarveMedialConstruction
{
    public const string Policy = "ordinary-medial-inch-grid-v1";
    public const double CurveToleranceMm = .001 * 25.4;
    public const double SiteScale = 4096 / 25.4;
    public const double CornerAngle = 3 * Math.PI / 4;
    private const double PolygonScale = 32768;
    private const double Inch = 25.4;

    /// <summary>Prepares one signed layer before selected-layer composition.</summary>
    public static List<List<PointD>> PrepareLayer(IReadOnlyList<IReadOnlyList<PointD>> source,
        FillRule fillRule = FillRule.NonZero, CancellationToken token = default)
    {
        var paths = new Paths64(source.Select(r => new Path64(r.Select(p => new Point64(
            Round(p.x / Inch * PolygonScale), Round(p.y / Inch * PolygonScale))))));
        return World(UnionNominal(Lighten(paths, token), fillRule));
    }

    /// <summary>Sequential legacy boolean between prepared layers. Adjacent
    /// coordinate cleanup precedes each boolean, independently of Lighten.</summary>
    public static List<List<PointD>> CombineLayer(IReadOnlyList<IReadOnlyList<PointD>> subject,
        IReadOnlyList<IReadOnlyList<PointD>> clip, bool subtract, CancellationToken token = default)
    {
        List<List<Legacy.IntPoint>> Grid(IReadOnlyList<IReadOnlyList<PointD>> rings)
        {
            var paths = new List<List<Legacy.IntPoint>>();
            foreach (var ring in rings)
            {
                token.ThrowIfCancellationRequested();
                var keep = new List<Legacy.IntPoint>();
                double square = Math.Pow(.001 * PolygonScale, 2);
                double Distance(Legacy.IntPoint a, Legacy.IntPoint b) => Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2);
                foreach (var p in ring)
                {
                    var q = new Legacy.IntPoint(Round(p.x / Inch * PolygonScale), Round(p.y / Inch * PolygonScale));
                    if (keep.Count == 0 || Distance(keep[^1], q) > square) keep.Add(q);
                }
                if (keep.Count > 1 && Distance(keep[0], keep[^1]) <= square) keep.RemoveAt(keep.Count - 1);
                if (keep.Count >= 3) paths.Add(keep);
            }
            return paths;
        }
        var first = Grid(subject); var second = Grid(clip);
        if (first.Count == 0 && second.Count == 0) return new();
        var engine = new Legacy.Clipper();
        engine.AddPaths(first, Legacy.PolyType.ptSubject, true);
        engine.AddPaths(second, Legacy.PolyType.ptClip, true);
        var solution = new List<List<Legacy.IntPoint>>();
        if (!engine.Execute(subtract ? Legacy.ClipType.ctDifference : Legacy.ClipType.ctUnion,
            solution, Legacy.PolyFillType.pftNonZero, Legacy.PolyFillType.pftNonZero))
            throw new InvalidOperationException("Nominal layer composition failed; incomplete regions are not accepted.");
        token.ThrowIfCancellationRequested();
        return solution.Select(r => r.Select(p => new PointD(p.X / PolygonScale * Inch, p.Y / PolygonScale * Inch)).ToList()).ToList();
    }

    /// <summary>Adds cubic endpoints after subdivision in world coordinates.</summary>
    public static void AppendCubic(List<PointD> output, PointD a, PointD b, PointD c, PointD d,
        CancellationToken token = default)
    {
        Split(new(a.x / Inch, a.y / Inch), new(b.x / Inch, b.y / Inch),
            new(c.x / Inch, c.y / Inch), new(d.x / Inch, d.y / Inch), 0);
        void Split(PointD p, PointD q, PointD r, PointD s, int depth)
        {
            token.ThrowIfCancellationRequested();
            double flatness = .75 * Math.Sqrt(Math.Max(DistanceSquared(q, p, s), DistanceSquared(r, p, s)));
            if (flatness < .001) { output.Add(new(s.x * Inch, s.y * Inch)); return; }
            if (depth >= 32) throw new InvalidOperationException("V-carve cubic subdivision did not converge.");
            PointD pq = Mid(p, q), qr = Mid(q, r), rs = Mid(r, s), pqr = Mid(pq, qr), qrs = Mid(qr, rs), center = Mid(pqr, qrs);
            Split(p, pq, pqr, center, depth + 1); Split(center, qrs, rs, s, depth + 1);
        }
    }

    public static List<List<PointD>> Prepare(IReadOnlyList<IReadOnlyList<PointD>> source,
        CancellationToken token = default, Action<string>? diagnostic = null,
        Action<string, IReadOnlyList<IReadOnlyList<PointD>>>? observe = null,
        bool sourceIsPreparedLayer = false)
        => PrepareCore(source, token, diagnostic, observe, out _, sourceIsPreparedLayer);

    private static List<List<PointD>> PrepareCore(IReadOnlyList<IReadOnlyList<PointD>> source,
        CancellationToken token, Action<string>? diagnostic,
        Action<string, IReadOnlyList<IReadOnlyList<PointD>>>? observe, out bool originalRequired,
        bool sourceIsPreparedLayer = false)
    {
        token.ThrowIfCancellationRequested();
        diagnostic?.Invoke($"stage=ordinary-medial-preparation event=start policy={Policy} sourceRings={source.Count} sourceIsPreparedLayer={sourceIsPreparedLayer} polygonKernel=native-clipper2-1.5.4");
        foreach (var ring in source)
            foreach (var point in ring)
                if (!double.IsFinite(point.x) || !double.IsFinite(point.y))
                    throw new ArgumentException("V-carve boundaries require finite coordinates.", nameof(source));
        // Legacy polygon preparation is defined in ticks per inch, independently
        // of the workspace's millimetre clipping and signed stock grids.
        var rings = source.Select(r => new Path64(r.Select(p => new Point64(
            Round(p.x / Inch * PolygonScale), Round(p.y / Inch * PolygonScale))))).ToList();
        var layer = sourceIsPreparedLayer ? new Paths64(rings) : Union(Lighten(new Paths64(rings), token), PolygonScale);
        Observe("layer", layer);
        var prepared = Union(layer, 4096);
        int offsetIndex = 0;
        Observe("coarse-union", prepared);
        prepared = Offset(prepared, -16);
        Observe("contract", prepared);
        prepared = Offset(prepared, 32);
        Observe("expand", prepared);
        prepared = Offset(prepared, -16);
        Observe("recontract", prepared);
        prepared = Union(prepared, 4096);
        var cleaned = new Paths64();
        foreach (var ring in prepared)
        {
            token.ThrowIfCancellationRequested();
            var keep = new Path64();
            foreach (var p in ring)
                if (keep.Count == 0 || Squared(p, keep[^1]) > Math.Pow(.001 * PolygonScale, 2)) keep.Add(p);
            if (keep.Count > 1 && Squared(keep[0], keep[^1]) <= Math.Pow(.001 * PolygonScale, 2)) keep.RemoveAt(keep.Count - 1);
            if (keep.Count >= 3) cleaned.Add(keep);
        }
        Observe("prepared", cleaned);
        var result = World(cleaned);
        // Conditioning must never erase a component, hole or narrow connection.
        // Retain the complete source on a topology change; no branch length test.
        originalRequired = Signature(new Paths64(rings)) != Signature(cleaned)
            || result.Count == 0 && source.Any(r => r.Count >= 3);
        if (originalRequired)
        {
            diagnostic?.Invoke($"stage=ordinary-medial-preparation event=retain-source reason=topology-change policy={Policy} sourceRings={source.Count} candidateRings={result.Count}");
            result = source.Select(r => r.ToList()).ToList();
        }
        diagnostic?.Invoke($"stage=ordinary-medial-preparation policy={Policy} inputVertices={source.Sum(r => r.Count)} layerVertices={layer.Sum(r => r.Count)} preparedVertices={result.Sum(r => r.Count)} "
            + $"curveFlatnessMm={CurveToleranceMm:R} polygonTicksPerInch={PolygonScale} medialTicksPerInch=4096 cornerAngleDegrees=135 "
            + "polygonKernel=native-clipper2-1.5.4 layerKernel=clipper-6.4.2 authority=nominal-only");
        return result;

        string Signature(Paths64 paths)
        {
            var clipper = new Clipper64(); clipper.AddSubject(paths);
            var tree = new PolyTree64(); clipper.Execute(ClipType.Union, FillRule.NonZero, tree);
            string Walk(PolyPath64 node) => "(" + string.Concat(Enumerable.Range(0, node.Count)
                .Select(i => Walk((PolyPath64)node[i])).OrderBy(s => s, StringComparer.Ordinal)) + ")";
            return Walk(tree);
        }

        Paths64 Union(Paths64 input, double scale)
        {
            token.ThrowIfCancellationRequested();
            var scaled = new Paths64(input.Select(r => new Path64(r.Select(p => new Point64(
                Round(p.X / PolygonScale * scale), Round(p.Y / PolygonScale * scale))))));
            var union = UnionNominal(scaled, FillRule.NonZero);
            return new Paths64(union.Select(r => new Path64(r.Select(p => new Point64(
                Round(p.X / scale * PolygonScale), Round(p.Y / scale * PolygonScale))))));
        }
        Paths64 Offset(Paths64 input, double distance)
        {
            token.ThrowIfCancellationRequested();
            var lightened = Lighten(input, token);
            Observe("offset-input-" + offsetIndex++, lightened);
            var offset = NativeMedialPolygonKernel.Execute(lightened, true, FillRule.NonZero, distance, token);
            Observe("offset-raw-" + (offsetIndex - 1), offset);
            return Lighten(offset, token);
        }
        void Observe(string stage, Paths64 input) => observe?.Invoke(stage, World(input));
    }

    public static IReadOnlyList<MedialSegment> Construct(IMedialAxisProvider provider,
        IReadOnlyList<IReadOnlyList<PointD>> source, double maximumRadius,
        CancellationToken token = default, Action<string>? diagnostic = null,
        double tolerance = .03, Action<double, string, bool>? progress = null,
        bool sourceIsPreparedLayer = false)
    {
        var prepared = PrepareCore(source, token, diagnostic, null, out bool originalRequired, sourceIsPreparedLayer);
        var output = new List<MedialSegment>();
        foreach (var (outer, holes) in MedialAxisToolpaths.NonIntersectingPathGroups(prepared))
        {
            token.ThrowIfCancellationRequested();
            if (originalRequired)
            {
                output.AddRange(MedialAxisGeometry.Build(outer, holes.Select(h => (IReadOnlyList<PointD>)h).ToList(),
                    provider, tolerance, maximumRadius, Math.PI, token, diagnostic));
                continue;
            }
            var scaledOuter = Scale(outer);
            var scaledHoles = holes.Select(h => (IReadOnlyList<PointD>)Scale(h)).ToList();
            var segments = provider is BoostVoronoiProvider boost
                ? boost.ConstructNominalMedialAxis(scaledOuter, scaledHoles, tolerance, maximumRadius * SiteScale,
                    CornerAngle, token, diagnostic, progress)
                : provider.ConstructMedialAxis(scaledOuter, scaledHoles,
                    tolerance, maximumRadius * SiteScale, CornerAngle, true, token, diagnostic, progress);
            output.AddRange(segments.Select(s => new MedialSegment(WorldPoint(s.Point0), WorldPoint(s.Point1))));
        }
        diagnostic?.Invoke($"stage=ordinary-medial-construction policy={Policy} segments={output.Count} siteScalePerMm={SiteScale:R} preparedRings={prepared.Count} absoluteGrid=True");
        return output;
        List<PointD> Scale(IReadOnlyList<PointD> ring) => ring.Select(p => new PointD(Round(p.x / Inch * 4096), Round(p.y / Inch * 4096))).ToList();
        MedialPoint WorldPoint(MedialPoint p) => new(p.X / 4096 * Inch, p.Y / 4096 * Inch, p.Radius / 4096 * Inch);
    }

    private static Paths64 UnionNominal(Paths64 input, FillRule fillRule) => NativeMedialPolygonKernel.Execute(input, false, fillRule, 0);
    private static List<List<PointD>> World(Paths64 input) => input.Select(r => r.Select(p =>
        new PointD(p.X / PolygonScale * Inch, p.Y / PolygonScale * Inch)).ToList()).ToList();
    private static long Round(double value) => checked((long)Math.Floor(value + .5));
    private static PointD Mid(PointD a, PointD b) => new(a.x * .5 + b.x * .5, a.y * .5 + b.y * .5);
    private static double Squared(Point64 a, Point64 b) => Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2);
    private static double DistanceSquared(PointD p, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y;
        double t = dx == 0 && dy == 0 ? 0 : Math.Clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy), 0, 1);
        return Math.Pow(p.x - a.x - t * dx, 2) + Math.Pow(p.y - a.y - t * dy, 2);
    }
    private static Paths64 Lighten(Paths64 input, CancellationToken token)
    {
        var result = new Paths64();
        foreach (var ring in input)
        {
            var current = new Path64(ring);
            if (current.Count > 1 && current[0] == current[^1]) current.RemoveAt(current.Count - 1);
            bool changed;
            do
            {
                token.ThrowIfCancellationRequested();
                changed = false;
                var next = new Path64();
                for (int i = 0; i < current.Count; i++)
                {
                    next.Add(current[i]);
                    if (i + 1 >= current.Count) continue;
                    var a = current[i]; var p = current[i + 1]; var b = current[(i + 2) % current.Count];
                    if (DistanceSquared(new(p.X, p.Y), new(a.X, a.Y), new(b.X, b.Y)) <= 64 * 64)
                    { i++; changed = true; }
                }
                current = next;
            } while (changed && current.Count > 2);
            if (current.Count >= 3) result.Add(current);
        }
        return result;
    }
}
