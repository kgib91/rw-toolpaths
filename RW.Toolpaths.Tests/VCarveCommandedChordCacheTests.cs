using System.Reflection;
using System.Runtime.ExceptionServices;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class VCarveCommandedChordCacheTests(ITestOutputHelper output)
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CachedLiteralResultPreservesTheOriginalWitnessBits(bool violation)
    {
        var boundary = Boundary(1);
        var profile = Cylinder(violation ? 2 : .125);
        var cache = Create(boundary, profile);
        Point3D a = new(-.375, .125, -.125), b = new(.5, .25, -.25);
        var literal = Literal([boundary], profile, a, b);
        Assert.Equal(violation, literal.Violation);
        Equal(literal, Query(cache, a, b));
        Equal(literal, Query(cache, a, b));
        Assert.Equal(1, Counter(cache, "ChordCacheMisses"));
        Assert.Equal(1, Counter(cache, "ChordCacheHits"));
        Assert.Equal(1, Counter(cache, "LiteralChordQueries"));
        output.WriteLine($"literalViolation={literal.Violation} literalWitnessBits={Bits(literal.Witness):x16} identicalCacheWitness=True");
    }

    [Fact]
    public void BothDomainsKeepTheirOriginalShortCircuitOrderAndWitness()
    {
        var artwork = Boundary(2);
        var channel = Boundary(.125);
        var profile = Cylinder(.25);
        var cache = Create(artwork, profile, channel: channel);
        Point3D a = new(-.5, .0625, -.125), b = new(.5, .0625, -.25);
        var literal = Literal([artwork, channel], profile, a, b);
        Assert.True(literal.Violation);
        Equal(literal, Query(cache, a, b));
        Assert.Equal(2, Counter(cache, "LiteralChordDomainQueries"));
        Equal(literal, Query(cache, a, b));
        Assert.Equal(2, Counter(cache, "LiteralChordDomainQueries"));
        var reverseDomains = Create(channel, profile, channel: artwork);
        Equal(Literal([channel, artwork], profile, a, b), Query(reverseDomains, a, b));
        Assert.Equal(1, Counter(reverseDomains, "LiteralChordDomainQueries"));
    }

    [Theory]
    [InlineData("ax")]
    [InlineData("ay")]
    [InlineData("az")]
    [InlineData("bx")]
    [InlineData("by")]
    [InlineData("bz")]
    [InlineData("direction")]
    public void EveryDirectedCommandedCoordinateIsPartOfTheKey(string change)
    {
        var boundary = Boundary(2);
        var profile = Cylinder(.125);
        var cache = Create(boundary, profile);
        Point3D a = new(-.25, .125, -.125), b = new(.375, .25, -.25);
        Query(cache, a, b);
        var changedA = a; var changedB = b;
        switch (change)
        {
            case "ax": changedA = a with { X = Math.BitIncrement(a.X) }; break;
            case "ay": changedA = a with { Y = Math.BitIncrement(a.Y) }; break;
            case "az": changedA = a with { Z = Math.BitIncrement(a.Z) }; break;
            case "bx": changedB = b with { X = Math.BitIncrement(b.X) }; break;
            case "by": changedB = b with { Y = Math.BitIncrement(b.Y) }; break;
            case "bz": changedB = b with { Z = Math.BitIncrement(b.Z) }; break;
            case "direction": changedA = b; changedB = a; break;
        }
        Equal(Literal([boundary], profile, changedA, changedB), Query(cache, changedA, changedB));
        Assert.Equal(2, Counter(cache, "ChordCacheMisses"));
        Assert.Equal(0, Counter(cache, "ChordCacheHits"));
        Query(cache, changedA, changedB);
        Assert.Equal(1, Counter(cache, "ChordCacheHits"));
    }

    [Fact]
    public void SignedZeroKeepsItsOwnCommandedCoordinateKey()
    {
        double negativeZero = BitConverter.Int64BitsToDouble(long.MinValue);
        var boundary = Boundary(2);
        var profile = Cylinder(.125);
        var cache = Create(boundary, profile);
        Point3D a = new(0, .25, -.125), b = new(.5, .25, -.125);
        var negativeA = a with { X = negativeZero };
        Assert.NotEqual(Bits(a.X), Bits(negativeA.X));
        Equal(Literal([boundary], profile, a, b), Query(cache, a, b));
        Equal(Literal([boundary], profile, negativeA, b), Query(cache, negativeA, b));
        Assert.Equal(2, Counter(cache, "ChordCacheMisses"));
        Assert.Equal(2, Counter(cache, "ChordCacheEntries"));
        Query(cache, a, b); Query(cache, negativeA, b);
        Assert.Equal(2, Counter(cache, "ChordCacheHits"));
    }

    [Fact]
    public void ProfileDomainAndSurfaceOwnersNeverShareEvidence()
    {
        Point3D a = new(-.25, 0, -.125), b = new(.25, 0, -.125);
        var wide = Boundary(2); var narrow = Boundary(.0625);
        var smallProfile = Cylinder(.125); var largeProfile = Cylinder(3);
        var first = Create(wide, smallProfile);
        Assert.False(Query(first, a, b).Violation);
        Query(first, a, b);
        foreach (var owner in new[] { Create(narrow, smallProfile), Create(wide, largeProfile), Create(wide, smallProfile, surface: 12.7) })
        {
            Query(owner, a, b);
            Assert.Equal(0, Counter(owner, "ChordCacheHits"));
            Assert.Equal(1, Counter(owner, "LiteralChordQueries"));
        }
        Assert.True(Query(Create(narrow, smallProfile), a, b).Violation);
        Assert.True(Query(Create(wide, largeProfile), a, b).Violation);
    }

    [Fact]
    public void PublicProfileAndInputMutationsCannotAlterTheOwnerSnapshot()
    {
        CarvingCutterSection[] sections = [new(CarvingCutterSectionKind.Linear, .125, .125, 1)];
        var profile = new VCarveCutterProfile(sections, 1);
        var ring = new[] { new PointD(-2, -2), new(2, -2), new(2, 2), new(-2, 2) };
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [ring];
        var cache = Create(boundary, profile);
        Point3D a = new(-.25, 0, -.125), b = new(.25, 0, -.125);
        var expected = Query(cache, a, b);
        sections[0] = sections[0] with { BottomRadius = 3, TopRadius = 3 };
        ring[0] = new(100, 100);
        var exposed = Assert.IsAssignableFrom<IList<CarvingCutterSection>>(profile.Sections);
        Assert.True(exposed.IsReadOnly);
        Assert.Throws<NotSupportedException>(() => exposed[0] = sections[0]);
        Assert.Equal(.125, profile.Sections[0].TopRadius);
        Equal(expected, Query(cache, a, b));
        Assert.Equal(1, Counter(cache, "LiteralChordQueries"));
    }

    [Fact]
    public void CancellationBeforeAHitNeverReturnsCachedAuthority()
    {
        using var cancellation = new CancellationTokenSource();
        var cache = Create(Boundary(2), Cylinder(.125), token: cancellation.Token);
        Point3D a = new(-.25, 0, -.125), b = new(.25, 0, -.125);
        Query(cache, a, b);
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => Query(cache, a, b));
        Assert.Equal(0, Counter(cache, "ChordCacheHits"));
    }

    [Theory]
    [InlineData("miss")]
    [InlineData("hit")]
    public void DiagnosticCancellationDoesNotPublishOrReturnAResult(string action)
    {
        using var cancellation = new CancellationTokenSource();
        var cache = Create(Boundary(2), Cylinder(.125), token: cancellation.Token,
            diagnostic: line => { output.WriteLine(line); if (line.Contains($"event={action} ", StringComparison.Ordinal)) cancellation.Cancel(); });
        Point3D a = new(-.25, 0, -.125), b = new(.25, 0, -.125);
        if (action == "hit") Query(cache, a, b);
        Assert.Throws<OperationCanceledException>(() => Query(cache, a, b));
        Assert.Equal(action == "hit" ? 1 : 0, Counter(cache, "ChordCacheEntries"));
    }

    [Fact]
    public void IsLegalAndApplyReuseTheSameLiteralPredicateAndStillChargePointsOnHits()
    {
        var cache = Create(Boundary(2), Cylinder(.125), maximumPoints: 1);
        Point3D a = new(-.25, 0, -.125), b = new(.25, 0, -.125);
        Assert.True(Invoke<bool>(cache, "IsLegalCommandedChord", a, b));
        Assert.True(Invoke<bool>(cache, "IsLegalCommandedChord", b, a));
        var failure = Assert.Throws<InvalidOperationException>(() =>
            Invoke<List<TaggedToolpath>>(cache, "Apply", (object)new[] { Path(a, b, a) }));
        Assert.Contains("command wall proof exceeds 1 points", failure.Message);
        Assert.Equal(2, Counter(cache, "ChordCacheHits"));
        Assert.Equal(2, Counter(cache, "LiteralChordQueries"));
    }

    [Fact]
    public void BoundedEvictionNeverTruncatesALiteralProof()
    {
        var boundary = Boundary(2); var profile = Cylinder(.125);
        var cache = Create(boundary, profile, maximumCachedChords: 2);
        Point3D a = new(-.25, 0, -.125);
        Point3D[] ends = [new(.125, 0, -.125), new(.25, 0, -.125), new(.375, 0, -.125)];
        foreach (var b in ends) Equal(Literal([boundary], profile, a, b), Query(cache, a, b));
        Assert.Equal(2, Counter(cache, "ChordCacheEntries"));
        Assert.Equal(1, Counter(cache, "ChordCacheEvictions"));
        Equal(Literal([boundary], profile, a, ends[0]), Query(cache, a, ends[0]));
        Assert.Equal(4, Counter(cache, "LiteralChordQueries"));
        Assert.Equal(2, Counter(cache, "ChordCacheEntries"));
        Assert.Equal(2, Counter(cache, "ChordCacheEvictions"));
    }

    [Fact]
    public void NonfiniteCommandAndUnsupportedProfileRetainLiteralQueries()
    {
        var boundary = Boundary(2); var profile = Cylinder(.125);
        var cache = Create(boundary, profile);
        Point3D a = new(-.25, 0, double.NaN), b = new(.25, 0, -.125);
        var literal = Assert.Throws<ArgumentOutOfRangeException>(() => Literal([boundary], profile, a, b));
        for (int i = 0; i < 2; i++)
        {
            var refused = Assert.Throws<ArgumentOutOfRangeException>(() => Query(cache, a, b));
            Assert.Equal(literal.ParamName, refused.ParamName);
        }
        Assert.Equal(2, Counter(cache, "ChordCacheRefusals"));
        Assert.Equal(0, Counter(cache, "ChordCacheEntries"));
        var largeProfile = new VCarveCutterProfile(Enumerable.Range(0, 65)
            .Select(_ => new CarvingCutterSection(CarvingCutterSectionKind.Linear, .125, .125, 1)), 1);
        var unsupported = Create(boundary, largeProfile);
        a = a with { Z = -.125 };
        Equal(Literal([boundary], largeProfile, a, b), Query(unsupported, a, b));
        Query(unsupported, a, b);
        Assert.Equal(2, Counter(unsupported, "ChordCacheRefusals"));
        Assert.Equal(2, Counter(unsupported, "LiteralChordQueries"));
        Assert.Equal(0, Counter(unsupported, "ChordCacheEntries"));
    }

    [Fact]
    public void CachedAndUncachedProtectionReturnIdenticalCommandPoints()
    {
        var boundary = Boundary(2); var profile = Cylinder(.25);
        var cached = Create(boundary, profile); var original = Create(boundary, profile, useCache: false);
        var paths = new[] { Path(new(-.5, .25, -.125), new(.5, .25, -.25), new(-.5, .25, -.125)) };
        var expected = Invoke<List<TaggedToolpath>>(original, "Apply", (object)paths);
        Invoke<List<TaggedToolpath>>(cached, "Apply", (object)paths);
        var actual = Invoke<List<TaggedToolpath>>(cached, "Apply", (object)paths);
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
            Assert.Equal(expected[i].Points.SelectMany(BitsOfPoint), actual[i].Points.SelectMany(BitsOfPoint));
        Assert.True(Counter(cached, "ChordCacheHits") >= 2);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RefinedChordsAcrossAHoleKeepTheLiteralSubdivisionPoints(bool rounded)
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [Boundary(2)[0],
            Boundary(.25)[0].Reverse().ToArray()];
        var profile = rounded
            ? new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, .5, .5)], .5)
            : Cylinder(.125);
        var cached = Create(boundary, profile);
        var literal = Create(boundary, profile, useCache: false);
        var paths = new[] { Path(new(-1, 0, -.125), new(1, 0, -.125)) };
        Assert.False(Invoke<bool>(cached, "IsLegalCommandedChord", paths[0].Points[0], paths[0].Points[1]));
        var expected = Invoke<List<TaggedToolpath>>(literal, "Apply", (object)paths);
        var actual = Invoke<List<TaggedToolpath>>(cached, "Apply", (object)paths);
        Assert.Single(expected); Assert.Single(actual);
        Assert.True(expected[0].Points.Count > 2);
        Assert.Equal(expected[0].Points.SelectMany(BitsOfPoint), actual[0].Points.SelectMany(BitsOfPoint));
        Assert.True(Counter(cached, "ChordCacheHits") > 0);
        var repeated = Invoke<List<TaggedToolpath>>(cached, "Apply", (object)paths);
        Assert.Equal(expected[0].Points.SelectMany(BitsOfPoint), repeated[0].Points.SelectMany(BitsOfPoint));
        output.WriteLine($"rounded={rounded} refinedPoints={actual[0].Points.Count} sameLiteralSubdivisionBits=True");
    }

    private object Create(IReadOnlyList<IReadOnlyList<PointD>> boundary, VCarveCutterProfile profile,
        IReadOnlyList<IReadOnlyList<PointD>>? channel = null, double surface = 0,
        CancellationToken token = default, long maximumPoints = 1_000_000, bool useCache = true,
        int maximumCachedChords = 65_536, Action<string>? diagnostic = null)
        => ClearanceType.GetConstructors(BindingFlags.Instance | BindingFlags.NonPublic)
            .Single(ctor => ctor.GetParameters().Length == 9)
            .Invoke([boundary, profile, surface, token, diagnostic ?? output.WriteLine, channel,
                maximumPoints, useCache, maximumCachedChords]);

    private static Type ClearanceType => typeof(VCarveSmoothing).GetNestedType("CommandClearance", BindingFlags.NonPublic)!;
    private static long Counter(object cache, string name)
        => Convert.ToInt64(ClearanceType.GetProperty(name, BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(cache));
    private static (bool Violation, double Witness) Query(object cache, Point3D a, Point3D b)
    {
        object?[] arguments = [a, b, 0d];
        bool violation = Invoke<bool>(cache, "TryFindCommandedChordViolation", arguments);
        return (violation, (double)arguments[2]!);
    }
    private static T Invoke<T>(object cache, string method, params object?[] arguments)
    {
        try { return (T)ClearanceType.GetMethod(method, BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(cache, arguments)!; }
        catch (TargetInvocationException failure) when (failure.InnerException is { } inner)
        { ExceptionDispatchInfo.Capture(inner).Throw(); throw; }
    }
    private static (bool Violation, double Witness) Literal(IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> domains,
        VCarveCutterProfile profile, Point3D a, Point3D b)
    {
        double witness = 0;
        foreach (var domain in domains)
        {
            var edges = new List<(PointD A, PointD B)>();
            foreach (var ring in domain)
            for (int i = 0; i < ring.Count; i++) edges.Add((ring[i], ring[(i + 1) % ring.Count]));
            var index = new MedialAxisToolpaths.SpatialSegmentIndex(edges);
            if (index.TryFindDepthViolation(a, b, 0, 0, 1, 0, out witness, profile)) return (true, witness);
        }
        return (false, witness);
    }
    private static void Equal((bool Violation, double Witness) expected, (bool Violation, double Witness) actual)
    { Assert.Equal(expected.Violation, actual.Violation); Assert.Equal(Bits(expected.Witness), Bits(actual.Witness)); }
    private static long Bits(double value) => BitConverter.DoubleToInt64Bits(value);
    private static IEnumerable<long> BitsOfPoint(Point3D point) => [Bits(point.X), Bits(point.Y), Bits(point.Z)];
    private static VCarveCutterProfile Cylinder(double radius)
        => new([new(CarvingCutterSectionKind.Linear, radius, radius, 1)], 1);
    private static IReadOnlyList<IReadOnlyList<PointD>> Boundary(double half)
        => [[new(-half, -half), new(half, -half), new(half, half), new(-half, half)]];
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "clearing", null, detectCurves: false);
}
