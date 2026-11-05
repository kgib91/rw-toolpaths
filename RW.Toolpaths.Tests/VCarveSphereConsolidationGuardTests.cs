using System.Globalization;
using System.Text.RegularExpressions;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class VCarveSphereConsolidationGuardTests
{
    [Theory]
    [InlineData("planar", "consolidationPlanarRefusals")]
    [InlineData("surface-z", "consolidationWorldZRefusals")]
    [InlineData("endpoint-z", "consolidationWorldZRefusals")]
    [InlineData("depth", "consolidationDepthRefusals")]
    [InlineData("radius", "consolidationRadiusRefusals")]
    [InlineData("tip", "consolidationTipReserveRefusals")]
    [InlineData("profile", "consolidationProfileRefusals")]
    [InlineData("join", "consolidationJoinRefusals")]
    public void RefusedDomainUsesLiteralOrderedOriginalPieces(string domain, string counter)
    {
        var profile = Sphere(1);
        double surface = 0;
        var a = new Point3D(.123456, .234567, -.001);
        var b = new Point3D(2.123456, .234567, -.002);
        switch (domain)
        {
            case "planar": a = a with { X = a.X + 10001 }; b = b with { X = b.X + 10001 }; break;
            case "surface-z": surface = 101; a = a with { Z = surface - .001 }; b = b with { Z = surface - .002 }; break;
            case "endpoint-z": surface = 100; a = a with { Z = 101 }; b = b with { Z = surface - .002 }; break;
            case "depth": surface = 50; a = a with { Z = surface - .001 }; b = b with { Z = surface - 100.001 }; break;
            case "radius": profile = Sphere(100.0001); break;
            case "tip": a = a with { Z = 0 }; break;
            case "profile": profile = new([new(CarvingCutterSectionKind.Linear, 0, 2, 2)], 2); break;
            case "join": b = b with { Z = -1 }; break;
        }
        var path = Path(a, b);
        var expected = Run([path], profile, surface, false, false, out _);
        var actual = Run([path], profile, surface, true, false, out string log);
        Equal(expected, actual);
        Assert.True(Counter(log, counter) > 0, log);
        Assert.Equal(0, Counter(log, "consolidatedChords"));
        Assert.Contains("consolidationRefusal=literal-original-pieces", log);
        Console.WriteLine($"[VCarve.SphereConsolidationGuard.Test] domain={domain} "
            + $"refusals={Counter(log, counter)} orderedOriginalEqual=True");
    }

    [Fact]
    public void RoundedSectionStartAliasingCannotGiveConsolidationAnUnsupportedSphereRadius()
    {
        const double firstSphereHeight = 1, subUlpHeight = 1e-20;
        Assert.Equal(firstSphereHeight, firstSphereHeight + subUlpHeight);
        var profile = new VCarveCutterProfile(
        [
            new(CarvingCutterSectionKind.Hemisphere, 0, 1, firstSphereHeight),
            new(CarvingCutterSectionKind.Linear, 1, 2, subUlpHeight),
            new(CarvingCutterSectionKind.Linear, 2, 2, 1),
        ], 2);
        // The existing literal profile includes the third section at the rounded
        // join. Its endpoint radius cannot enter a first-sphere convexity proof.
        Assert.Equal(2, profile.PhysicalRadiusAtDepth(firstSphereHeight));
        var path = Path(new(.123456, .234567, -.001), new(2.123456, .234567, -firstSphereHeight));
        var reference = Run([path], profile, 0, false, false, out _);
        var candidate = Run([path], profile, 0, true, false, out string log);
        Equal(reference, candidate);
        Assert.True(Counter(log, "consolidationJoinRefusals") > 0, log);
        Assert.Equal(0, Counter(log, "consolidatedChords"));
        Console.WriteLine("[VCarve.SphereConsolidationGuard.Test] sectionHeight=1e-20 "
            + "literalJoinRadius=2 firstSphereRadius=1 joinRefused=True orderedOriginalEqual=True");
    }

    [Fact]
    public void TranslatedOrdinaryChordAtLargestSphereRadiusRetainsPhysicalSubset()
    {
        const double surface = 75, sphereRadius = 100;
        var profile = Sphere(sphereRadius);
        var a = new Point3D(9999, -.234567, surface - .001);
        var b = new Point3D(9999.5, .123456, surface - .002);
        var lower = Run([Path(a, b)], profile, surface, true, false, out string log);
        Assert.NotEmpty(lower);
        Assert.True(Counter(log, "consolidatedChords") > 0, log);
        Assert.Equal(0, Counter(log, "consolidationWorldZRefusals"));
        foreach (var polygon in lower)
        for (int i = 0; i < polygon.Count; i++)
        for (int sample = 0; sample <= 4; sample++)
        {
            var first = polygon[i]; var last = polygon[(i + 1) % polygon.Count];
            var q = new PointD(first.x + (last.x - first.x) * sample / 4,
                first.y + (last.y - first.y) * sample / 4);
            Assert.True(SphereCenterDistance(q, a, b, surface, sphereRadius) <= sphereRadius,
                $"Consolidated translated lower sweep claims unreachable stock at {q.x:R},{q.y:R}.");
        }
        Console.WriteLine("[VCarve.SphereConsolidationGuard.Test] worldSurface=75 sphereRadius=100 "
            + "maximumCommandX=9999.5 continuousPhysicalSubset=True");
    }

    [Fact]
    public void OptionalConsolidationKeepsUpperReferenceAndConstantDepthRunOrdered()
    {
        var profile = Sphere(1);
        var variable = Path(new(.123456, .234567, -.001), new(2.123456, .234567, -.5));
        var upperReference = Run([variable], profile, 0, false, true, out _);
        var upperCandidate = Run([variable], profile, 0, true, true, out string upperLog);
        Equal(upperReference, upperCandidate);
        Assert.Equal(0, Counter(upperLog, "consolidationRequests"));
        var flat = Path(new(0, 0, -.1), new(1, 0, -.1), new(1, 1, -.1), new(0, 1, -.1), new(0, 0, -.1));
        var flatReference = Run([flat], profile, 0, false, false, out _);
        var flatCandidate = Run([flat], profile, 0, true, false, out string flatLog);
        Equal(flatReference, flatCandidate);
        Assert.Equal(0, Counter(flatLog, "consolidationRequests"));
        Assert.True(Counter(flatLog, "flatRuns") > 0);
    }

    private static List<List<PointD>> Run(IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile,
        double surface, bool enabled, bool upper, out string diagnostics)
    {
        var original = Console.Out;
        using var output = new StringWriter(CultureInfo.InvariantCulture);
        try
        {
            using var logSelection = RW.Diagnostics.DiagnosticLog.PushNamespaces("*");
            Console.SetOut(output);
            return VCarvePhysicalSweep.Build(paths, profile, surface, .00075,
                upperBound: upper, consolidateHemisphereChords: enabled);
        }
        finally { Console.SetOut(original); diagnostics = output.ToString(); }
    }

    private static long Counter(string diagnostics, string name)
    {
        var match = Regex.Matches(diagnostics, @"\b" + Regex.Escape(name) + @"=(\d+)").Cast<Match>().Last();
        return long.Parse(match.Groups[1].Value, CultureInfo.InvariantCulture);
    }

    private static double SphereCenterDistance(PointD q, Point3D a, Point3D b, double surface, double radius)
    {
        double ax = a.X - q.x, ay = a.Y - q.y, az = radius - (surface - a.Z);
        double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
        double square = dx * dx + dy * dy + dz * dz;
        double t = square > 0 ? Math.Clamp(-(ax * dx + ay * dy + az * dz) / square, 0, 1) : 0;
        return Math.Sqrt(Math.Pow(ax + dx * t, 2) + Math.Pow(ay + dy * t, 2) + Math.Pow(az + dz * t, 2));
    }

    private static void Equal(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++) Assert.Equal(expected[i], actual[i]);
    }

    private static VCarveCutterProfile Sphere(double radius)
        => new([new(CarvingCutterSectionKind.Hemisphere, 0, radius, radius)], radius);
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
}
