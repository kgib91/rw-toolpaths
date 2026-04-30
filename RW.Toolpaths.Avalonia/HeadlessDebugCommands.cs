using System.Globalization;
using System.Text.Json;
using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths;

namespace RW.Toolpaths.Avalonia;

internal static class HeadlessDebugCommands
{
    private const double PxToInch = 1.0 / 96.0;

    public static bool TryRun(string[] args)
    {
        if (args.Length == 0) return false;

        if (HasFlag(args, "--run-medial-payload"))
        {
            RunMedialPayload(args);
            return true;
        }

        if (HasFlag(args, "--run-vcarve-payload"))
        {
            RunVCarvePayload(args);
            return true;
        }

        if (HasFlag(args, "--dump-glyph-payload"))
        {
            if (OperatingSystem.IsWindows())
            {
                DumpGlyphPayload(args);
            }
            return true;
        }

        if (HasFlag(args, "--analyze-medial-payload"))
        {
            AnalyzeMedialPayload(args);
            return true;
        }

        if (HasFlag(args, "--benchmark-medial-payload"))
        {
            BenchmarkMedialPayload(args);
            return true;
        }

        if (HasFlag(args, "--help") || HasFlag(args, "-h"))
        {
            PrintUsage();
            return true;
        }

        return false;
    }

    private static void RunMedialPayload(string[] args)
    {
        string payloadPath = GetRequiredValue(args, "--run-medial-payload");

        var options = new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true
        };

        var payloadText = File.ReadAllText(payloadPath);
        var payload = JsonSerializer.Deserialize<MedialPayload>(payloadText, options)
            ?? throw new InvalidOperationException("Failed to deserialize medial payload");

        if (payload.Boundary is null || payload.Boundary.Count < 3)
            throw new InvalidOperationException("payload.boundary must contain at least 3 points");

        var holes = payload.Holes ?? new List<List<XY>>();

        double tolerance = payload.Tolerance ?? 0.03;
        double filteringAngle = payload.FilteringAngle ?? (3 * Math.PI / 4);
        bool useBigIntegers = payload.UseBigIntegers ?? true;

        double maxRadius;
        if (payload.MaxRadius.HasValue)
        {
            maxRadius = payload.MaxRadius.Value;
        }
        else if (payload.DepthPerPass.HasValue && payload.RadianTipAngle.HasValue)
        {
            maxRadius = Math.Tan(payload.RadianTipAngle.Value / 2.0) * payload.DepthPerPass.Value;
        }
        else
        {
            throw new InvalidOperationException(
                "payload.maxRadius is required unless depthPerPass and radianTipAngle are provided");
        }

        var provider = BoostVoronoiProvider.CreateDefault();

        var boundary = payload.Boundary
            .Select(p => new PointD(p.X, p.Y))
            .ToList();

        var holeRings = holes
            .Select(r => (IReadOnlyList<PointD>)r.Select(p => new PointD(p.X, p.Y)).ToList())
            .ToList();

        var result = provider.ConstructMedialAxis(
            boundary,
            holeRings,
            tolerance,
            maxRadius,
            filteringAngle,
            useBigIntegers);

        var output = result.Select(s => new
        {
            point0 = new { x = s.Point0.X, y = s.Point0.Y, radius = s.Point0.Radius },
            point1 = new { x = s.Point1.X, y = s.Point1.Y, radius = s.Point1.Radius }
        }).ToList();

        Console.WriteLine(JsonSerializer.Serialize(output));
    }

    private static void AnalyzeMedialPayload(string[] args)
    {
        string payloadPath = GetRequiredValue(args, "--analyze-medial-payload");

        var options = new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true
        };

        var payloadText = File.ReadAllText(payloadPath);
        var payload = JsonSerializer.Deserialize<MedialPayload>(payloadText, options)
            ?? throw new InvalidOperationException("Failed to deserialize medial payload");

        if (payload.Boundary is null || payload.Boundary.Count < 3)
            throw new InvalidOperationException("payload.boundary must contain at least 3 points");

        var holes = payload.Holes ?? new List<List<XY>>();

        double tolerance = payload.Tolerance ?? 0.03;
        double filteringAngle = payload.FilteringAngle ?? (3 * Math.PI / 4);
        bool useBigIntegers = payload.UseBigIntegers ?? true;

        double depthPerPass = payload.DepthPerPass ?? ParseDouble(GetValue(args, "--depth-per-pass") ?? "0.5");
        double radianTipAngle = payload.RadianTipAngle ?? ParseDouble(GetValue(args, "--radian-tip-angle") ?? (Math.PI / 3.0).ToString(CultureInfo.InvariantCulture));
        double maxRadius = payload.MaxRadius ?? (Math.Tan(radianTipAngle / 2.0) * depthPerPass);

        double startDepth = ParseDouble(GetValue(args, "--start-depth") ?? "0");
        double endDepth = ParseDouble(GetValue(args, "--end-depth") ?? "0.25");
        double stepOver = ParseDouble(GetValue(args, "--step-over") ?? "0.4");
        double? bottomRadius = payload.BottomRadius;
        double? topRadius = payload.TopRadius;

        var provider = BoostVoronoiProvider.CreateDefault();

        var region = new List<IReadOnlyList<PointD>>
        {
            payload.Boundary.Select(p => new PointD(p.X, p.Y)).ToList()
        };
        region.AddRange(holes.Select(r => (IReadOnlyList<PointD>)r.Select(p => new PointD(p.X, p.Y)).ToList()));

        var raw = MedialAxisToolpaths.GenerateRawMedialAxisSegments(
            provider,
            region,
            radianTipAngle,
            depthPerPass,
            tolerance,
            maxRadiusOverride: maxRadius);

        var rawRadii = raw
            .SelectMany(s => new[] { s.Point0.Radius, s.Point1.Radius })
            .ToList();

        double rawMinR = rawRadii.Count == 0 ? 0 : rawRadii.Min();
        double rawMaxR = rawRadii.Count == 0 ? 0 : rawRadii.Max();
        double rawAvgR = rawRadii.Count == 0 ? 0 : rawRadii.Average();

        int rawEndpointCount = rawRadii.Count;
        int shallow05Pct = rawRadii.Count(r => r <= maxRadius * 0.05);
        int shallow10Pct = rawRadii.Count(r => r <= maxRadius * 0.10);

        var (clearing, vcarve) = MedialAxisToolpaths.GenerateVCarveComponents(
            provider,
            region,
            startDepth,
            endDepth,
            radianTipAngle,
            depthPerPass,
            stepOver,
            tolerance,
            bottomRadiusOverride: bottomRadius,
            topRadiusOverride: topRadius);

        int vPaths = vcarve.Count;
        int vSegments = vcarve.Sum(p => Math.Max(0, p.Count - 1));
        double minVZ = vcarve.SelectMany(p => p).DefaultIfEmpty(new Point3D(0, 0, 0)).Min(p => p.Z);

        int nearSurfaceSegments = 0;
        int lowAlphaSegments = 0;

        foreach (var path in vcarve)
        {
            for (int i = 1; i < path.Count; i++)
            {
                double segZ = (path[i - 1].Z + path[i].Z) * 0.5;
                if (segZ > -0.01)
                    nearSurfaceSegments++;

                // Mirror MainForm VCarveColorForDepth alpha mapping.
                double denom = minVZ >= 0 ? -0.25 : minVZ;
                double t = Math.Clamp(segZ / denom, 0.0, 1.0);
                int alpha = (int)(40 + 190 * t);
                if (alpha < 80)
                    lowAlphaSegments++;
            }
        }

        var output = new
        {
            payload = payloadPath,
            settings = new
            {
                tolerance,
                filteringAngle,
                maxRadius,
                useBigIntegers,
                depthPerPass,
                radianTipAngle,
                startDepth,
                endDepth,
                stepOver
            },
            rawKernel = new
            {
                segments = raw.Count,
                endpointCount = rawEndpointCount,
                radiusMin = rawMinR,
                radiusMax = rawMaxR,
                radiusAvg = rawAvgR,
                endpointRadiusLe5PctMaxRadius = shallow05Pct,
                endpointRadiusLe10PctMaxRadius = shallow10Pct
            },
            postProcessing = new
            {
                clearingPaths = clearing.Count,
                vcarvePaths = vPaths,
                vcarveSegments = vSegments,
                minVZ,
                nearSurfaceSegments,
                lowAlphaSegments
            }
        };

        Console.WriteLine(JsonSerializer.Serialize(output, new JsonSerializerOptions
        {
            WriteIndented = true
        }));
    }

    private static void RunVCarvePayload(string[] args)
    {
        string payloadPath = GetRequiredValue(args, "--run-vcarve-payload");

        var options = new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true
        };

        var payloads = LoadVCarvePayloads(payloadPath, options);
        if (payloads.Count == 0)
            throw new InvalidOperationException("No payloads found in input JSON.");

        var provider = BoostVoronoiProvider.CreateDefault();

        double argStartDepth = ParseDouble(GetValue(args, "--start-depth") ?? "0");
        double argEndDepth = ParseDouble(GetValue(args, "--end-depth") ?? "0.25");
        double argStepOver = ParseDouble(GetValue(args, "--step-over") ?? "0.4");
        double argTolerance = ParseDouble(GetValue(args, "--tolerance") ?? "0.03");

        var taggedPaths = new List<TaggedToolpath>();
        var perRegion = new List<object>();

        for (int i = 0; i < payloads.Count; i++)
        {
            var payload = payloads[i];

            if (payload.Boundary is null || payload.Boundary.Count < 3)
                throw new InvalidOperationException($"payload[{i}].boundary must contain at least 3 points");

            var holes = payload.Holes ?? new List<List<XY>>();

            double depthPerPass = payload.DepthPerPass
                ?? ParseDouble(GetValue(args, "--depth-per-pass") ?? "0.5");
            double radianTipAngle = payload.RadianTipAngle
                ?? ParseDouble(GetValue(args, "--radian-tip-angle")
                    ?? (Math.PI / 3.0).ToString(CultureInfo.InvariantCulture));

            double startDepth = payload.StartDepth ?? argStartDepth;
            double endDepth = payload.EndDepth ?? argEndDepth;
            double stepOver = payload.StepOver ?? argStepOver;
            double tolerance = payload.Tolerance ?? argTolerance;

            if (endDepth <= startDepth)
                throw new InvalidOperationException($"payload[{i}] requires endDepth > startDepth");

            var region = new List<IReadOnlyList<PointD>>
            {
                payload.Boundary.Select(p => new PointD(p.X, p.Y)).ToList()
            };
            region.AddRange(holes.Select(r => (IReadOnlyList<PointD>)r.Select(p => new PointD(p.X, p.Y)).ToList()));

            int regionIndex = payload.RegionIndex ?? i;
            var (clearing, vcarve) = MedialAxisToolpaths.GenerateVCarveComponentsTagged(
                provider,
                region,
                startDepth,
                endDepth,
                radianTipAngle,
                depthPerPass,
                stepOver,
                tolerance,
                regionIndex: regionIndex,
                bottomRadiusOverride: payload.BottomRadius,
                topRadiusOverride: payload.TopRadius,
                coneLengthOverride: payload.ConeLength);

            taggedPaths.AddRange(clearing);
            taggedPaths.AddRange(vcarve);

            perRegion.Add(new
            {
                regionIndex,
                clearingPaths = clearing.Count,
                finalPaths = vcarve.Count,
                totalPaths = clearing.Count + vcarve.Count,
                depthPasses = clearing
                    .Where(t => t.DepthPassIndex.HasValue)
                    .Select(t => t.DepthPassIndex!.Value)
                    .Distinct()
                    .OrderBy(v => v)
                    .ToList()
            });
        }

        var output = new
        {
            payload = payloadPath,
            payloadCount = payloads.Count,
            toolpathCount = taggedPaths.Count,
            regions = perRegion,
            toolpaths = taggedPaths.Select((t, index) => new
            {
                index,
                regionIndex = t.RegionIndex,
                category = t.Category,
                depthPassIndex = t.DepthPassIndex,
                points = t.Points.Select(p => new { x = p.X, y = p.Y, z = p.Z }).ToList()
            }).ToList()
        };

        Console.WriteLine(JsonSerializer.Serialize(output, new JsonSerializerOptions
        {
            WriteIndented = true
        }));
    }

    private static List<MedialPayload> LoadVCarvePayloads(string payloadPath, JsonSerializerOptions options)
    {
        var payloadText = File.ReadAllText(payloadPath);
        using var json = JsonDocument.Parse(payloadText);

        if (json.RootElement.ValueKind == JsonValueKind.Object
            && json.RootElement.TryGetProperty("payloads", out var payloadsElement)
            && payloadsElement.ValueKind == JsonValueKind.Array)
        {
            var payloads = new List<MedialPayload>();
            foreach (var item in payloadsElement.EnumerateArray())
            {
                var payload = item.Deserialize<MedialPayload>(options);
                if (payload is not null)
                    payloads.Add(payload);
            }

            return payloads;
        }

        var single = JsonSerializer.Deserialize<MedialPayload>(payloadText, options)
            ?? throw new InvalidOperationException("Failed to deserialize V-carve payload.");
        return new List<MedialPayload> { single };
    }

    private static void BenchmarkMedialPayload(string[] args)
    {
        string payloadPath = GetRequiredValue(args, "--benchmark-medial-payload");

        var options = new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true
        };

        var payloadText = File.ReadAllText(payloadPath);
        var payload = JsonSerializer.Deserialize<MedialPayload>(payloadText, options)
            ?? throw new InvalidOperationException("Failed to deserialize medial payload");

        if (payload.Boundary is null || payload.Boundary.Count < 3)
            throw new InvalidOperationException("payload.boundary must contain at least 3 points");

        var holes = payload.Holes ?? new List<List<XY>>();
        double tolerance = payload.Tolerance ?? 0.03;
        double depthPerPass = payload.DepthPerPass ?? ParseDouble(GetValue(args, "--depth-per-pass") ?? "0.05");
        double radianTipAngle = payload.RadianTipAngle ?? ParseDouble(GetValue(args, "--radian-tip-angle") ?? (Math.PI / 3.0).ToString(CultureInfo.InvariantCulture));
        double startDepth = ParseDouble(GetValue(args, "--start-depth") ?? "0");
        double endDepth = ParseDouble(GetValue(args, "--end-depth") ?? "0.25");
        double stepOver = ParseDouble(GetValue(args, "--step-over") ?? "0.4");
        double? bottomRadius = payload.BottomRadius;
        double? topRadius = payload.TopRadius;

        int iterations = ParseInt(GetValue(args, "--iterations") ?? "20");
        int warmup = ParseInt(GetValue(args, "--warmup") ?? "2");
        if (iterations <= 0)
            throw new InvalidOperationException("--iterations must be > 0");
        if (warmup < 0)
            throw new InvalidOperationException("--warmup must be >= 0");

        var provider = BoostVoronoiProvider.CreateDefault();
        var region = new List<IReadOnlyList<PointD>>(1 + holes.Count)
        {
            payload.Boundary.Select(p => new PointD(p.X, p.Y)).ToList()
        };

        for (int i = 0; i < holes.Count; i++)
            region.Add(holes[i].Select(p => new PointD(p.X, p.Y)).ToList());

        // Warmup runs for JIT/native cold start effects.
        for (int i = 0; i < warmup; i++)
        {
            _ = MedialAxisToolpaths.GenerateVCarveComponents(
                provider,
                region,
                startDepth,
                endDepth,
                radianTipAngle,
                depthPerPass,
                stepOver,
                tolerance);
        }

        var times = new List<double>(iterations);
        int clearingPaths = 0;
        int vcarvePaths = 0;
        int vcarveSegments = 0;

        for (int i = 0; i < iterations; i++)
        {
            var sw = Stopwatch.StartNew();
            var (clearing, vcarve) = MedialAxisToolpaths.GenerateVCarveComponents(
                provider,
                region,
                startDepth,
                endDepth,
                radianTipAngle,
                depthPerPass,
                stepOver,
                tolerance,
                bottomRadiusOverride: bottomRadius,
                topRadiusOverride: topRadius);
            sw.Stop();

            times.Add(sw.Elapsed.TotalMilliseconds);

            if (i == iterations - 1)
            {
                clearingPaths = clearing.Count;
                vcarvePaths = vcarve.Count;
                vcarveSegments = vcarve.Sum(p => Math.Max(0, p.Count - 1));
            }
        }

        var sorted = times.OrderBy(t => t).ToList();
        double avg = times.Average();
        double min = sorted[0];
        double max = sorted[^1];
        double p50 = Percentile(sorted, 0.50);
        double p90 = Percentile(sorted, 0.90);
        double p95 = Percentile(sorted, 0.95);

        var output = new
        {
            payload = payloadPath,
            iterations,
            warmup,
            timingsMs = new
            {
                min,
                p50,
                p90,
                p95,
                avg,
                max
            },
            output = new
            {
                clearingPaths,
                vcarvePaths,
                vcarveSegments
            }
        };

        Console.WriteLine(JsonSerializer.Serialize(output, new JsonSerializerOptions
        {
            WriteIndented = true
        }));

        static double Percentile(IReadOnlyList<double> sortedValues, double p)
        {
            if (sortedValues.Count == 0)
                return 0;

            double idx = Math.Clamp(p, 0.0, 1.0) * (sortedValues.Count - 1);
            int lo = (int)Math.Floor(idx);
            int hi = (int)Math.Ceiling(idx);
            if (lo == hi)
                return sortedValues[lo];

            double w = idx - lo;
            return sortedValues[lo] * (1.0 - w) + sortedValues[hi] * w;
        }
    }

    [System.Runtime.Versioning.SupportedOSPlatform("windows")]
    private static void DumpGlyphPayload(string[] args)
    {
        string text = GetRequiredValue(args, "--text");
        string font = GetValue(args, "--font") ?? "Arial";
        float fontSize = ParseFloat(GetValue(args, "--font-size") ?? "90");

        double tolerance = ParseDouble(GetValue(args, "--tolerance") ?? "0.03");
        double depthPerPass = ParseDouble(GetValue(args, "--depth-per-pass") ?? "0.5");
        double vBitAngleDeg = ParseDouble(GetValue(args, "--vbit-angle-deg") ?? "60");

        var rawContours = FontGlyphExtractor.ExtractContours(text, font, fontSize);

        var contoursInches = rawContours
            .Select(c => c.Select(p => new XY(p.X * PxToInch, p.Y * PxToInch)).ToList())
            .ToList();

        var rings = contoursInches
            .Select(r => r.Select(p => new PointD(p.X, p.Y)).ToList())
            .Where(r => r.Count >= 3)
            .ToList();

        var regions = BuildMedialAxisRegions(rings);

        double radianTipAngle = vBitAngleDeg * Math.PI / 180.0;
        double maxRadius = Math.Tan(radianTipAngle / 2.0) * depthPerPass;

        var payloads = regions.Select((region, index) => new
        {
            regionIndex = index,
            boundary = region[0].Select(p => new { x = p.x, y = p.y }).ToList(),
            holes = region.Skip(1)
                .Select(r => r.Select(p => new { x = p.x, y = p.y }).ToList())
                .ToList(),
            tolerance,
            filteringAngle = 3 * Math.PI / 4,
            maxRadius,
            useBigIntegers = true,
            parabolaMethod = "CENTRAL_ANGLE",
            depthPerPass,
            radianTipAngle
        }).ToList();

        var output = new
        {
            text,
            font,
            fontSize,
            contourCount = contoursInches.Count,
            regionCount = payloads.Count,
            contours = contoursInches,
            payloads
        };

        Console.WriteLine(JsonSerializer.Serialize(output));
    }

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

    private static bool HasFlag(string[] args, string flag) =>
        args.Any(a => string.Equals(a, flag, StringComparison.OrdinalIgnoreCase));

    private static string? GetValue(string[] args, string key)
    {
        for (int i = 0; i < args.Length - 1; i++)
        {
            if (string.Equals(args[i], key, StringComparison.OrdinalIgnoreCase))
                return args[i + 1];
        }

        return null;
    }

    private static string GetRequiredValue(string[] args, string key) =>
        GetValue(args, key)
        ?? throw new InvalidOperationException($"Missing required argument: {key}");

    private static double ParseDouble(string value)
    {
        if (!double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var parsed))
            throw new InvalidOperationException($"Invalid number: {value}");
        return parsed;
    }

    private static float ParseFloat(string value)
    {
        if (!float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var parsed))
            throw new InvalidOperationException($"Invalid number: {value}");
        return parsed;
    }

    private static int ParseInt(string value)
    {
        if (!int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out var parsed))
            throw new InvalidOperationException($"Invalid integer: {value}");
        return parsed;
    }

    private static void PrintUsage()
    {
        Console.WriteLine("Headless debug commands:");
        Console.WriteLine("  --run-medial-payload <jsonPath>");
        Console.WriteLine("    Runs BoostVoronoiProvider with literal payload and prints segments JSON.");
        Console.WriteLine();
        Console.WriteLine("  --run-vcarve-payload <jsonPath> [--start-depth <n>] [--end-depth <n>] [--depth-per-pass <n>] [--radian-tip-angle <n>] [--step-over <n>] [--tolerance <n>]");
        Console.WriteLine("    Runs tagged V-carve generation and prints toolpaths JSON with region/category/depthPassIndex metadata.");
        Console.WriteLine("    Accepts either a single payload object or an object with payloads:[...].");
        Console.WriteLine();
        Console.WriteLine("  --dump-glyph-payload --text <value> [--font <name>] [--font-size <n>]");
        Console.WriteLine("      [--tolerance <n>] [--depth-per-pass <n>] [--vbit-angle-deg <n>]");
        Console.WriteLine("    Dumps contours and region payload(s) exactly as GUI preprocessing produces.");
        Console.WriteLine();
        Console.WriteLine("  --analyze-medial-payload <jsonPath> [--start-depth <n>] [--end-depth <n>] [--step-over <n>]");
        Console.WriteLine("    Compares raw kernel output vs post-processing/visualization metrics for one payload.");
        Console.WriteLine();
        Console.WriteLine("  --benchmark-medial-payload <jsonPath> [--iterations <n>] [--warmup <n>] [--start-depth <n>] [--end-depth <n>] [--step-over <n>]");
        Console.WriteLine("    Runs repeated in-process V-carve generation and prints min/p50/p90/p95/avg/max timings.");
    }

    private sealed class MedialPayload
    {
        public int? RegionIndex { get; set; }
        public List<XY> Boundary { get; set; } = new();
        public List<List<XY>>? Holes { get; set; }
        public double? Tolerance { get; set; }
        public double? MaxRadius { get; set; }
        public double? FilteringAngle { get; set; }
        public bool? UseBigIntegers { get; set; }

        public double? StartDepth { get; set; }
        public double? EndDepth { get; set; }
        public double? StepOver { get; set; }
        public double? DepthPerPass { get; set; }
        public double? RadianTipAngle { get; set; }
        public double? BottomRadius { get; set; }
        public double? TopRadius { get; set; }
        public double? ConeLength { get; set; }
    }

    private sealed record XY(double X, double Y);
}

