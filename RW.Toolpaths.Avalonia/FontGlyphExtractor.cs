using System.Drawing;
using System.Drawing.Drawing2D;
using RW.Toolpaths;

namespace RW.Toolpaths.Avalonia;

internal static class FontGlyphExtractor
{
    [System.Runtime.Versioning.SupportedOSPlatform("windows")]
    public static List<List<PointF>> ExtractContours(string text, string fontFamily, float emSize, float flatness = 0.5f)
    {
        long t0 = PerfLog.Start();
        if (string.IsNullOrWhiteSpace(text))
        {
            PerfLog.Stop("FontGlyphExtractor.ExtractContours", t0, "empty-text");
            return new List<List<PointF>>();
        }

        if (!OperatingSystem.IsWindows())
        {
            var fallback = BuildFallbackContours(text, emSize);
            PerfLog.Stop("FontGlyphExtractor.ExtractContours", t0, $"fallback=true contours={fallback.Count}");
            return fallback;
        }

        try
        {
            using var path = new GraphicsPath();
            path.AddString(
                text,
                new FontFamily(fontFamily),
                (int)FontStyle.Regular,
                emSize,
                new PointF(0f, 0f),
                StringFormat.GenericTypographic);

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

            if (contours.Count == 0)
            {
                var fallback = BuildFallbackContours(text, emSize);
                PerfLog.Stop("FontGlyphExtractor.ExtractContours", t0, $"fallback=true contours={fallback.Count}");
                return fallback;
            }

            PerfLog.Stop("FontGlyphExtractor.ExtractContours", t0, $"fallback=false contours={contours.Count}");

            return contours;
        }
        catch
        {
            var fallback = BuildFallbackContours(text, emSize);
            PerfLog.Stop("FontGlyphExtractor.ExtractContours", t0, $"fallback=exception contours={fallback.Count}");
            return fallback;
        }
    }

    private static List<List<PointF>> BuildFallbackContours(string text, float emSize)
    {
        long t0 = PerfLog.Start();
        // Minimal cross-platform fallback: one closed rectangular contour per
        // character cell so the modern UI remains functional without GDI+.
        var contours = new List<List<PointF>>();
        float w = Math.Max(8f, emSize * 0.55f);
        float h = Math.Max(8f, emSize);
        float gap = Math.Max(2f, emSize * 0.08f);

        float x = 0f;
        foreach (char ch in text)
        {
            if (char.IsWhiteSpace(ch))
            {
                x += w * 0.5f;
                continue;
            }

            var ring = new List<PointF>
            {
                new(x, 0f),
                new(x + w, 0f),
                new(x + w, h),
                new(x, h),
                new(x, 0f)
            };
            contours.Add(ring);
            x += w + gap;
        }

        PerfLog.Stop("FontGlyphExtractor.BuildFallbackContours", t0, $"chars={text.Length} contours={contours.Count}");
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

