using System.Drawing;
using System.Drawing.Drawing2D;
using Clipper2Lib;
using RW.Toolpaths;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public class SGlyphGuiTopologyTests
{
    private const double PxToInch = 1.0 / 96.0;
    private readonly ITestOutputHelper _output;

    public SGlyphGuiTopologyTests(ITestOutputHelper output)
    {
        _output = output;
    }

    [Fact]
    public void AbelS_DumpsMedialAxisDiagnostics()
    {
        var contours = ExtractContours("S", "Abel", 180f)
            .Select(c => c.Select(p => new PointD(p.X * PxToInch, p.Y * PxToInch)).ToList())
            .Where(r => r.Count >= 3)
            .ToList();

        Assert.NotEmpty(contours);

        var openContours = contours
            .Select(r => r.Count > 1 && SamePoint(r[0], r[^1]) ? r.Take(r.Count - 1).ToList() : r)
            .ToList();

        var roots = OffsetFill.BuildPathTree(contours);
        Assert.Single(roots);
        Assert.Empty(roots[0].Children);

        var contour = contours[0];
        var bbox = BoundingBox(contour);
        var region = BuildMedialAxisRegions(contours).Single();
        var boundary = region[0].ToList();
        var polygon = PathUtils.ToClipper(new[] { boundary })[0];
        var normalized = Clipper.BooleanOp(ClipType.Union, PathUtils.ToClipper(contours), new Paths64(), FillRule.NonZero);
        var simplified = MedialAxisToolpaths.GenerateSimplifiedLayer(region, 4096.0);
        Assert.NotNull(simplified);
        Assert.NotEmpty(simplified!);
        var simplifiedPaths = PathUtils.ToClipper(simplified!);
        var simplifiedBbox = BoundingBox(simplified!.SelectMany(r => r).ToList());

        var provider = BoostVoronoiProvider.CreateDefault();
        var raw = MedialAxisToolpaths.GenerateRawMedialAxisSegments(
            provider,
            region,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.05,
            tolerance: 0.03);

        int outsidePolygonEndpoints = 0;
        int outsideSimplifiedEndpoints = 0;
        int outsideBoundsEndpoints = 0;
        int outsidePolygonMidpoints = 0;
        int outsideSimplifiedMidpoints = 0;
        double maxSegmentLength = 0;
        MedialSegment maxRawSegment = default;
        foreach (var seg in raw)
        {
            var segLen = SegmentLength(seg);
            if (segLen > maxSegmentLength)
            {
                maxSegmentLength = segLen;
                maxRawSegment = seg;
            }
            CountEndpoint(seg.Point0);
            CountEndpoint(seg.Point1);
            CountMidpoint(seg);
        }

        var (_, vcarve) = MedialAxisToolpaths.GenerateVCarveComponents(
            provider,
            region,
            startDepth: 0.0,
            endDepth: 0.25,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.05,
            stepOver: 0.4,
            tolerance: 0.03);

        var openRegion = BuildMedialAxisRegions(openContours).Single();
        var openRaw = MedialAxisToolpaths.GenerateRawMedialAxisSegments(
            provider,
            openRegion,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.05,
            tolerance: 0.03);
        var (_, openVcarve) = MedialAxisToolpaths.GenerateVCarveComponents(
            provider,
            openRegion,
            startDepth: 0.0,
            endDepth: 0.25,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.05,
            stepOver: 0.4,
            tolerance: 0.03);

        double maxPolylineStep = 0;
        Point3D maxStepStart = default;
        Point3D maxStepEnd = default;
        foreach (var path in vcarve)
        {
            for (int i = 1; i < path.Count; i++)
            {
                var dx = path[i].X - path[i - 1].X;
                var dy = path[i].Y - path[i - 1].Y;
                var step = Math.Sqrt(dx * dx + dy * dy);
                if (step > maxPolylineStep)
                {
                    maxPolylineStep = step;
                    maxStepStart = path[i - 1];
                    maxStepEnd = path[i];
                }
            }
        }

        _output.WriteLine($"contours={contours.Count} rootChildren={roots[0].Children.Count} regionRings={region.Count}");
        _output.WriteLine($"bbox=({bbox.minX:F4},{bbox.minY:F4})-({bbox.maxX:F4},{bbox.maxY:F4}) pts={contour.Count}");
        _output.WriteLine($"openContourPts={openContours[0].Count}");
        _output.WriteLine($"normalizedRings={normalized.Count}");
        _output.WriteLine($"simplifiedRings={simplified!.Count}");
        _output.WriteLine($"simplifiedBbox=({simplifiedBbox.minX:F4},{simplifiedBbox.minY:F4})-({simplifiedBbox.maxX:F4},{simplifiedBbox.maxY:F4})");
        _output.WriteLine($"rawSegments={raw.Count} outsidePolygonEndpoints={outsidePolygonEndpoints} outsideSimplifiedEndpoints={outsideSimplifiedEndpoints} outsidePolygonMidpoints={outsidePolygonMidpoints} outsideSimplifiedMidpoints={outsideSimplifiedMidpoints} outsideBoundsEndpoints={outsideBoundsEndpoints} maxRawSegmentLen={maxSegmentLength:F6}");
        _output.WriteLine($"vcarvePaths={vcarve.Count} maxPolylineStep={maxPolylineStep:F6}");
        _output.WriteLine($"openRawSegments={openRaw.Count} openVcarvePaths={openVcarve.Count}");
        _output.WriteLine($"maxRawSegment=({maxRawSegment.Point0.X:F4},{maxRawSegment.Point0.Y:F4})->({maxRawSegment.Point1.X:F4},{maxRawSegment.Point1.Y:F4})");
        _output.WriteLine($"maxVcarveStep=({maxStepStart.X:F4},{maxStepStart.Y:F4},{maxStepStart.Z:F4})->({maxStepEnd.X:F4},{maxStepEnd.Y:F4},{maxStepEnd.Z:F4})");

        Assert.NotEmpty(raw);

        void CountEndpoint(MedialPoint pt)
        {
            var scaled = new Point64((long)Math.Round(pt.X * PathUtils.Scale), (long)Math.Round(pt.Y * PathUtils.Scale));
            if (!PathUtils.PointInPolygon(scaled, polygon))
                outsidePolygonEndpoints++;
            if (!IsInsideAny(scaled, simplifiedPaths))
                outsideSimplifiedEndpoints++;
            if (pt.X < bbox.minX - 1e-6 || pt.X > bbox.maxX + 1e-6 || pt.Y < bbox.minY - 1e-6 || pt.Y > bbox.maxY + 1e-6)
                outsideBoundsEndpoints++;
        }

        void CountMidpoint(MedialSegment seg)
        {
            var mx = (seg.Point0.X + seg.Point1.X) * 0.5;
            var my = (seg.Point0.Y + seg.Point1.Y) * 0.5;
            var scaled = new Point64((long)Math.Round(mx * PathUtils.Scale), (long)Math.Round(my * PathUtils.Scale));
            if (!PathUtils.PointInPolygon(scaled, polygon))
                outsidePolygonMidpoints++;
            if (!IsInsideAny(scaled, simplifiedPaths))
                outsideSimplifiedMidpoints++;
        }

        static bool IsInsideAny(Point64 pt, Paths64 paths)
        {
            foreach (var path in paths)
            {
                if (PathUtils.PointInPolygon(pt, path))
                    return true;
            }

            return false;
        }
    }

    private static double SegmentLength(MedialSegment seg)
    {
        var dx = seg.Point1.X - seg.Point0.X;
        var dy = seg.Point1.Y - seg.Point0.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    private static (double minX, double minY, double maxX, double maxY) BoundingBox(IReadOnlyList<PointD> pts)
    {
        double minX = double.MaxValue, minY = double.MaxValue;
        double maxX = double.MinValue, maxY = double.MinValue;
        foreach (var p in pts)
        {
            minX = Math.Min(minX, p.x);
            minY = Math.Min(minY, p.y);
            maxX = Math.Max(maxX, p.x);
            maxY = Math.Max(maxY, p.y);
        }

        return (minX, minY, maxX, maxY);
    }

    private static bool SamePoint(PointD a, PointD b) =>
        Math.Abs(a.x - b.x) <= 1e-9 && Math.Abs(a.y - b.y) <= 1e-9;

    private static List<IReadOnlyList<IReadOnlyList<PointD>>> BuildMedialAxisRegions(List<List<PointD>> rings)
    {
        var roots = OffsetFill.BuildPathTree(rings);
        var regions = new List<IReadOnlyList<IReadOnlyList<PointD>>>();

        static List<PointD> EnsureWinding(List<PointD> ring, bool ccw)
        {
            var area = Clipper.Area(PathUtils.ToClipper(new[] { ring })[0]);
            bool isCcw = area > 0;
            if (isCcw == ccw) return ring;

            var copy = new List<PointD>(ring);
            copy.Reverse();
            return copy;
        }

        static void Walk(PathTreeNode node, List<IReadOnlyList<IReadOnlyList<PointD>>> output)
        {
            var region = new List<IReadOnlyList<PointD>>
            {
                EnsureWinding(node.Points, ccw: true)
            };

            foreach (var hole in node.Children)
                region.Add(EnsureWinding(hole.Points, ccw: false));

            output.Add(region);

            foreach (var hole in node.Children)
            foreach (var nestedOuter in hole.Children)
                Walk(nestedOuter, output);
        }

        foreach (var root in roots)
            Walk(root, regions);

        return regions;
    }

    private static List<List<PointF>> ExtractContours(string text, string fontFamily, float emSize, float flatness = 0.5f)
    {
        using var path = new GraphicsPath();
        path.AddString(
            text,
            new FontFamily(fontFamily),
            (int)FontStyle.Regular,
            emSize,
            new PointF(0f, 0f),
            StringFormat.GenericDefault);

        path.Flatten(new Matrix(), flatness);

        var points = path.PathPoints;
        var types = path.PathTypes;
        var contours = new List<List<PointF>>();
        List<PointF>? current = null;

        for (int i = 0; i < points.Length; i++)
        {
            var kind = (PathPointType)(types[i] & (byte)PathPointType.PathTypeMask);
            var close = (types[i] & (byte)PathPointType.CloseSubpath) != 0;

            if (kind == PathPointType.Start)
            {
                if (current is { Count: > 1 })
                {
                    CloseIfNeeded(current);
                    contours.Add(current);
                }

                current = new List<PointF> { points[i] };
            }
            else
            {
                current ??= new List<PointF>();
                current.Add(points[i]);
            }

            if (close && current is { Count: > 1 })
            {
                CloseIfNeeded(current);
                contours.Add(current);
                current = null;
            }
        }

        if (current is { Count: > 1 })
        {
            CloseIfNeeded(current);
            contours.Add(current);
        }

        return contours;
    }

    private static void CloseIfNeeded(List<PointF> contour)
    {
        var first = contour[0];
        var last = contour[^1];
        if (Math.Abs(first.X - last.X) > 1e-5f || Math.Abs(first.Y - last.Y) > 1e-5f)
            contour.Add(first);
    }
}
