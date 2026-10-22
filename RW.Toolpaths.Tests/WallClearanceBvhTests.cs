using System.Diagnostics;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class WallClearanceBvhTests(ITestOutputHelper output)
{
    [Fact]
    public void NearestDistanceMatchesUnchangedTerminalArithmeticAcrossDenseAndDegenerateBounds()
    {
        var random = new Random(51067);
        foreach (var segments in Fixtures())
        {
            var optimized = new MedialAxisToolpaths.SpatialSegmentIndex(segments);
            var legacy = new LegacyWallClearanceGrid(segments);
            for (int i = 0; i < 1000; i++)
            {
                double x = random.NextDouble() * 150 - 75, y = random.NextDouble() * 150 - 75;
                double expected = Distance(x, y, segments);
                Assert.Equal(expected, optimized.QueryDistance(x, y));
                Assert.Equal(legacy.QueryDistance(x, y), optimized.QueryDistance(x, y));
            }
            foreach (var edge in segments)
            {
                Assert.Equal(Distance(edge.A.x, edge.A.y, segments), optimized.QueryDistance(edge.A.x, edge.A.y));
                Assert.Equal(Distance(edge.B.x, edge.B.y, segments), optimized.QueryDistance(edge.B.x, edge.B.y));
            }
        }
    }

    [Fact]
    public void ConservativelyPrunedCandidatesRetainLegacyFirstSeenOrderAndEveryReachableSegment()
    {
        var random = new Random(883);
        foreach (var segments in Fixtures())
        {
            var optimized = new MedialAxisToolpaths.SpatialSegmentIndex(segments);
            var legacy = new LegacyWallClearanceGrid(segments);
            for (int i = 0; i < 400; i++)
            {
                double x0 = random.NextDouble() * 130 - 65, y0 = random.NextDouble() * 130 - 65;
                double x1 = x0 + random.NextDouble() * 10 - 5, y1 = y0 + random.NextDouble() * 10 - 5;
                double radius = i % 7 == 0 ? 0 : random.NextDouble() * 3;
                AssertCandidates(optimized, legacy, segments, x0, y0, x1, y1, radius);
            }
            // Exact virtual cell boundaries, touching boxes and out-of-bounds clamps.
            for (int i = 0; i <= 64; i++)
            {
                double x = -50 + 100 * i / 64d;
                AssertCandidates(optimized, legacy, segments, x, -50, x, 50, 0);
                AssertCandidates(optimized, legacy, segments, Math.BitDecrement(x), -50, Math.BitIncrement(x), 50, 0);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContinuousConeAndRoundedProfileWitnessesExactlyMatchFrozenGrid(bool rounded)
    {
        var profile = rounded ? RoundedProfile() : null;
        var random = new Random(931);
        foreach (var segments in Fixtures())
        {
            var optimized = new MedialAxisToolpaths.SpatialSegmentIndex(segments);
            var legacy = new LegacyWallClearanceGrid(segments);
            for (int i = 0; i < 1500; i++)
            {
                double ax = random.NextDouble() * 110 - 55, ay = random.NextDouble() * 110 - 55;
                double bx = ax + random.NextDouble() * 20 - 10, by = ay + random.NextDouble() * 20 - 10;
                double ad = Depth(legacy.QueryDistance(ax, ay), profile), bd = Depth(legacy.QueryDistance(bx, by), profile);
                AssertWitness(optimized, legacy, new(ax, ay, -ad), new(bx, by, -bd), profile);
                AssertWitness(optimized, legacy, new(bx, by, -bd), new(ax, ay, -ad), profile);
            }
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EqualViolationsKeepLegacyOffCenterSplitAndIdenticalEmittedToolpaths(bool rounded)
    {
        // Symmetric point sites have identical violation minima at different
        // parameters. The legacy cell traversal picks its first-seen site.
        (PointD A, PointD B)[] segments = Enumerable.Range(0, 48).Select(i =>
            (new PointD(100 + i, 100 + i), new PointD(100 + i, 100 + i))).ToArray();
        segments[0] = (new(3, .1), new(3, .1));
        segments[30] = (new(1, .1), new(1, .1));
        var legacy = new LegacyWallClearanceGrid(segments);
        var optimized = new MedialAxisToolpaths.SpatialSegmentIndex(segments);
        var profile = rounded ? RoundedProfile() : null;
        var path = new List<Point3D> { new(0, 0, -1), new(4, 0, -1) };
        AssertWitness(optimized, legacy, path[0], path[1], profile);
        var expected = LegacyWallClearanceGrid.EnsureZFidelityFromBoundary(path, legacy,
            0, .05, .3125, -1, .003, profile);
        var actual = MedialAxisToolpaths.EnsureZFidelityFromBoundary(path, optimized,
            0, .05, .3125, -1, .003, profile);
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void PreviousNearestWallIsOnlyANewlyEvaluatedCandidateAndCannotCrossOwners()
    {
        var segments=DenseCircle(1024);
        var index=new MedialAxisToolpaths.SpatialSegmentIndex(segments,enableDiagnostics:true);
        var other=new MedialAxisToolpaths.SpatialSegmentIndex([(new(1000,1000),new(1001,1001))]);
        for(int i=0;i<400;i++)
        {
            double x=i%4==0?-500:49.95,y=i%4==0?300:i*.0001;
            Assert.Equal(Distance(x,y,segments),index.QueryDistance(x,y));
            Assert.Equal(Distance(x+.0001,y,segments),index.QueryDistance(x+.0001,y));
            other.QueryDistance(x,y);
            Assert.Equal(Distance(-x,-y,segments),index.QueryDistance(-x,-y));
        }
        Assert.True(index.Diagnostics!.NearestSeedTrials>0);
        Assert.True(index.Diagnostics.NearestSeedWins>0);
    }

    [Fact]
    public void ImmutableIndexAndWorkerScratchSupportConcurrentCrossIndexQueries()
    {
        var segments = DenseCircle(8192);
        var otherSegments = new[] { (new PointD(0, 0), new PointD(0, 0)) };
        var optimized = new MedialAxisToolpaths.SpatialSegmentIndex(segments, enableDiagnostics: true);
        var other = new MedialAxisToolpaths.SpatialSegmentIndex(otherSegments);
        var legacy = new LegacyWallClearanceGrid(segments);
        var profile = RoundedProfile();
        Parallel.For(0, 5000, i =>
        {
            double angle = i * .031;
            double ax = 49 * Math.Cos(angle), ay = 49 * Math.Sin(angle);
            double bx = ax + .25 * Math.Sin(angle), by = ay - .25 * Math.Cos(angle);
            Assert.Equal(legacy.QueryDistance(ax, ay), optimized.QueryDistance(ax, ay));
            AssertWitness(optimized, legacy, new(ax, ay, -1), new(bx, by, -1), profile);
            Assert.Equal(Math.Sqrt(ax * ax + ay * ay), other.QueryDistance(ax, ay));
            AssertWitness(optimized, legacy, new(bx, by, -1), new(ax, ay, -1), null);
        });
        Assert.Equal(5000, optimized.Diagnostics!.NearestQueries);
        Assert.True(optimized.Diagnostics.CandidateQueries > 0);
        // Original list mutation cannot invalidate either tree bounds or witnesses.
        double before = optimized.QueryDistance(49, 0);
        segments[0] = (new(1000, 1000), new(1001, 1001));
        Assert.Equal(before, optimized.QueryDistance(49, 0));
    }

    [Fact]
    public void DenseBoundaryQueriesHaveObservableSublinearDiscoveryAndReusableScratch()
    {
        var segments = DenseCircle(32768);
        var optimized = new MedialAxisToolpaths.SpatialSegmentIndex(segments, enableDiagnostics: true);
        optimized.QueryDistance(49.9, 0);
        optimized.CandidateOrdinals(49.9, 0, 49.9, .001, .01);
        long distanceTests = optimized.Diagnostics!.DistanceTests;
        long candidateTests = optimized.Diagnostics.CandidateLeafTests;
        Assert.InRange(distanceTests, 1, 128);
        Assert.InRange(candidateTests, 0, 128);
        long growths = optimized.Diagnostics.ScratchGrowths;
        optimized.CandidateOrdinals(49.9, 0, 49.9, .001, .01);
        Assert.Equal(growths, optimized.Diagnostics.ScratchGrowths);
        Assert.Null(new MedialAxisToolpaths.SpatialSegmentIndex(segments).Diagnostics);
        optimized.LogDiagnostics(output.WriteLine);
    }

    [Fact]
    public void TranslationScaleAndTouchingRoundoffKeepExactTerminalResultsAndWitnesses()
    {
        var random = new Random(6651);
        foreach (double scale in new[] { 1e-8, .001, 1, 10000 })
        foreach (double translation in new[] { 0, 200, 1e8 })
        {
            var segments = DenseCircle(256).Select(edge =>
                (new PointD(translation + scale * edge.A.x, translation + scale * edge.A.y),
                 new PointD(translation + scale * edge.B.x, translation + scale * edge.B.y))).ToArray();
            var optimized = new MedialAxisToolpaths.SpatialSegmentIndex(segments);
            var legacy = new LegacyWallClearanceGrid(segments);
            for (int i = 0; i < 500; i++)
            {
                double ax = translation + scale * (random.NextDouble() * 110 - 55);
                double ay = translation + scale * (random.NextDouble() * 110 - 55);
                double bx = translation + scale * (random.NextDouble() * 110 - 55);
                double by = translation + scale * (random.NextDouble() * 110 - 55);
                Assert.Equal(Distance(ax, ay, segments), optimized.QueryDistance(ax, ay));
                AssertWitness(optimized, legacy, new(ax, ay, -Depth(legacy.QueryDistance(ax, ay), null)),
                    new(bx, by, -Depth(legacy.QueryDistance(bx, by), null)), null);
            }
        }
    }

    [Fact]
    public void FocusedFiveAlternatingBenchmarks()
    {
        if (Environment.GetEnvironmentVariable("RW_WALL_INDEX_BENCHMARK") != "1") return;
        var results = new List<object>();
        var constructionResults = new List<object>();
        using var process = Process.GetCurrentProcess();
        foreach (int count in new[] { 4096, 32768, 174000 })
        {
            var segments = DenseCircle(count);
            var legacy = new LegacyWallClearanceGrid(segments);
            var optimized = new MedialAxisToolpaths.SpatialSegmentIndex(segments);
            const int queries = 10000;
            var inputs = Enumerable.Range(0, queries).Select(i =>
            {
                double angle = i * .01637;
                return (X: 49.8 * Math.Cos(angle), Y: 49.8 * Math.Sin(angle));
            }).ToArray();
            double Run(bool baseline)
            {
                double sum = 0;
                foreach (var p in inputs)
                {
                    sum += baseline ? legacy.QueryDistance(p.X, p.Y) : optimized.QueryDistance(p.X, p.Y);
                    var a = new Point3D(p.X, p.Y, -1);
                    var b = new Point3D(p.X + .08, p.Y + .02, -1);
                    bool violated = baseline
                        ? legacy.TryFindDepthViolation(a, b, 0, .05, .3125, .003, out double witness)
                        : optimized.TryFindDepthViolation(a, b, 0, .05, .3125, .003, out witness);
                    sum += witness + (violated ? 1 : 0);
                }
                return sum;
            }
            Run(true); Run(false);
            for (int repetition = 0; repetition < 5; repetition++)
            {
                double? baselineResult = null;
                foreach (bool baseline in repetition % 2 == 0 ? new[] { true, false } : new[] { false, true })
                {
                    GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
                    long cpu = process.TotalProcessorTime.Ticks;
                    long allocation = GC.GetAllocatedBytesForCurrentThread();
                    long started = Stopwatch.GetTimestamp();
                    double sum = Run(baseline);
                    double milliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                    long bytes = GC.GetAllocatedBytesForCurrentThread() - allocation;
                    double cpuMs = TimeSpan.FromTicks(process.TotalProcessorTime.Ticks - cpu).TotalMilliseconds;
                    process.Refresh();
                    if (baselineResult.HasValue) Assert.Equal(baselineResult.Value, sum);
                    else baselineResult = sum;
                    var row = new { segmentCount = count, queryCount = queries, repetition, baseline,
                        milliseconds, cpuMs, allocatedBytes = bytes, checksum = sum,
                        peakWorkingSetBytes = process.PeakWorkingSet64,
                        peakWorkingSetScope = "cumulative benchmark process", privateBytes = process.PrivateMemorySize64 };
                    results.Add(row);
                    output.WriteLine(JsonSerializer.Serialize(row));
                }
            }
            for (int repetition = 0; repetition < 5; repetition++)
            foreach (bool baseline in repetition % 2 == 0 ? new[] { true, false } : new[] { false, true })
            {
                GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
                long cpu = process.TotalProcessorTime.Ticks;
                long allocation = GC.GetAllocatedBytesForCurrentThread();
                long started = Stopwatch.GetTimestamp();
                object built = baseline ? new LegacyWallClearanceGrid(segments)
                    : new MedialAxisToolpaths.SpatialSegmentIndex(segments);
                double milliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
                long bytes = GC.GetAllocatedBytesForCurrentThread() - allocation;
                double cpuMs = TimeSpan.FromTicks(process.TotalProcessorTime.Ticks - cpu).TotalMilliseconds;
                GC.KeepAlive(built);
                process.Refresh();
                var row = new { segmentCount = count, repetition, baseline, milliseconds, cpuMs,
                    allocatedBytes = bytes, peakWorkingSetBytes = process.PeakWorkingSet64,
                    peakWorkingSetScope = "cumulative benchmark process", privateBytes = process.PrivateMemorySize64 };
                constructionResults.Add(row);
                output.WriteLine(JsonSerializer.Serialize(row));
            }
        }
        string? destination = Environment.GetEnvironmentVariable("RW_WALL_INDEX_BENCHMARK_FILE");
        if (!string.IsNullOrEmpty(destination))
        {
            File.WriteAllText(destination, JsonSerializer.Serialize(results, new JsonSerializerOptions { WriteIndented = true }));
            File.WriteAllText(Path.Combine(Path.GetDirectoryName(destination)!, "wall-index-build-benchmarks.json"),
                JsonSerializer.Serialize(constructionResults, new JsonSerializerOptions { WriteIndented = true }));
        }
    }

    private static void AssertWitness(MedialAxisToolpaths.SpatialSegmentIndex optimized,
        LegacyWallClearanceGrid legacy, Point3D a, Point3D b, VCarveCutterProfile? profile)
    {
        bool expected = legacy.TryFindDepthViolation(a, b, 0, .05, .3125, .003, out double ep, profile);
        bool actual = optimized.TryFindDepthViolation(a, b, 0, .05, .3125, .003, out double ap, profile);
        Assert.True(expected == actual && ep == ap,
            $"witness differs rounded={profile?.RoundedTip} expected={expected}/{ep:R} actual={actual}/{ap:R} " +
            $"a=({a.X:R},{a.Y:R},{a.Z:R}) b=({b.X:R},{b.Y:R},{b.Z:R})");
    }

    private static void AssertCandidates(MedialAxisToolpaths.SpatialSegmentIndex optimized,
        LegacyWallClearanceGrid legacy, IReadOnlyList<(PointD A, PointD B)> segments,
        double x0, double y0, double x1, double y1, double radius)
    {
        var actual = optimized.CandidateOrdinals(x0, y0, x1, y1, radius);
        var expected = legacy.CandidateOrdinals(x0, y0, x1, y1, radius);
        var retained = actual.ToHashSet();
        Assert.Equal(expected.Where(retained.Contains), actual);
        foreach (int ordinal in expected)
        {
            var seg = segments[ordinal];
            if (Math.Min(seg.A.x, seg.B.x) <= Math.Max(x0, x1) + radius
                && Math.Max(seg.A.x, seg.B.x) >= Math.Min(x0, x1) - radius
                && Math.Min(seg.A.y, seg.B.y) <= Math.Max(y0, y1) + radius
                && Math.Max(seg.A.y, seg.B.y) >= Math.Min(y0, y1) - radius)
                Assert.Contains(ordinal, actual);
        }
    }

    private static double Depth(double distance, VCarveCutterProfile? profile)
        => Math.Min(3, profile?.DepthAtRadius(distance) ?? Math.Max(0, distance - .05) / .3125);
    private static VCarveCutterProfile RoundedProfile() => new([
        new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
        new(CarvingCutterSectionKind.Linear, 1.5875, 3, 10),
        new(CarvingCutterSectionKind.Linear, 3, 3, 1)], 12.5875);

    private static IEnumerable<(PointD A, PointD B)[]> Fixtures()
    {
        yield return [];
        yield return [(new(0, 0), new(0, 0))];
        yield return Enumerable.Range(0, 128).Select(i => (new PointD(0, i - 64), new PointD(0, i - 63))).ToArray();
        yield return Enumerable.Range(0, 128).Select(i => (new PointD(i - 64, 0), new PointD(i - 63, 0))).ToArray();
        yield return DenseCircle(2048);
        var random = new Random(5147);
        yield return Enumerable.Range(0, 1024).Select(i =>
        {
            var a = new PointD(random.NextDouble() * 100 - 50, random.NextDouble() * 100 - 50);
            var b = i % 11 == 0 ? a : new PointD(a.x + random.NextDouble() * 100 - 50, a.y + random.NextDouble() * 100 - 50);
            return (a, b);
        }).ToArray();
    }

    private static (PointD A, PointD B)[] DenseCircle(int count)
        => Enumerable.Range(0, count).Select(i =>
            (new PointD(50 * Math.Cos(2 * Math.PI * i / count), 50 * Math.Sin(2 * Math.PI * i / count)),
             new PointD(50 * Math.Cos(2 * Math.PI * (i + 1) / count), 50 * Math.Sin(2 * Math.PI * (i + 1) / count)))).ToArray();

    private static double Distance(double x, double y, IReadOnlyList<(PointD A, PointD B)> segments)
    {
        double bestSq = double.PositiveInfinity;
        foreach (var (a, b) in segments)
        {
            double abx = b.x - a.x, aby = b.y - a.y, apx = x - a.x, apy = y - a.y;
            double ab2 = abx * abx + aby * aby;
            double t = ab2 <= 1e-18 ? 0.0 : (apx * abx + apy * aby) / ab2;
            if (t < 0) t = 0;
            else if (t > 1) t = 1;
            double dx = x - (a.x + abx * t), dy = y - (a.y + aby * t);
            double d2 = dx * dx + dy * dy;
            if (d2 < bestSq) bestSq = d2;
        }
        return bestSq < double.PositiveInfinity ? Math.Sqrt(bestSq) : 0;
    }
}
