using System.Diagnostics;
using System.Text.Json;
using RW.Toolpaths.Geometry;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class PreparedPointIndexTests(ITestOutputHelper output)
{
    [Fact]
    public void DepthDependentPhysicalRadiusBoundsMatchLiteralPoseWitnesses()
    {
        var random=new Random(7204);
        var points=Enumerable.Range(0,2048).Select(_=>new Point3D(random.NextDouble()*20,
            random.NextDouble()*20,-random.NextDouble()*6)).ToArray();
        var index=new PreparedPointIndex(points);
        static double Radius(double h)=>Math.Sqrt(Math.Max(0,h));
        for(int i=0;i<512;i++)
        {
            double x=random.NextDouble()*20,y=random.NextDouble()*20,z=-random.NextDouble()*6;
            bool Actual(Point3D p)=>p.Z<=z&&(p.X-x)*(p.X-x)+(p.Y-y)*(p.Y-y)<=Radius(z-p.Z)*Radius(z-p.Z);
            bool expected=points.Any(Actual);
            Assert.Equal(expected,index.AnyWithin(x,y,Radius(z-points.Min(p=>p.Z)),z,
                predicate:Actual,radiusAtDepth:Radius));
        }
    }

    [Fact]
    public void PhysicalPoseDiskQueriesMatchIndependentDepthAndRadiusWitnesses()
    {
        var random = new Random(147);
        var points = Enumerable.Range(0, 2048).Select(_ => new Point3D(random.NextDouble() * 20,
            random.NextDouble() * 20, -random.NextDouble() * 6)).ToArray();
        var index = new RW.Toolpaths.Geometry.PreparedPointIndex(points);
        for (int i = 0; i < 256; i++)
        {
            double x = random.NextDouble() * 20, y = random.NextDouble() * 20;
            double radius = random.NextDouble() * 3, z = -random.NextDouble() * 6;
            bool Filter(Point3D p) => p.X + p.Y >= 15;
            bool expected = points.Any(p => p.Z <= z && (p.X - x) * (p.X - x) + (p.Y - y) * (p.Y - y) <= radius * radius && Filter(p));
            Assert.Equal(expected, index.AnyWithin(x, y, radius, z, predicate: Filter));
        }
        Console.WriteLine("[PreparedPointIndex.PoseWitness] queries=256 originalPoints=2048 literalDepthDiskPredicateMatched=True");
    }

    [Theory]
    [InlineData(0)]
    [InlineData(301.41172)]
    [InlineData(-100000)]
    public void IncrementalOrderAndNearestThreeMatchOriginalStableSort(double origin)
    {
        var random = new Random(187321);
        var points = Enumerable.Range(0, 4096).Select(i => new Point3D(
            origin + random.Next(-100, 101) / 8.0, random.Next(-100, 101) / 8.0, i % 11)).ToArray();
        var index = new PreparedPointIndex(points);
        foreach (var target in Enumerable.Range(0, 48).Select(_ =>
            new Point3D(origin + 30 * random.NextDouble() - 15, 30 * random.NextDouble() - 15, 0))
            .Concat(points.Where((_, ordinal) => ordinal % 257 == 0)))
        {
            var expected = Reference(points, target.X, target.Y).ToArray();
            Assert.Equal(expected, index.Nearest(target.X, target.Y));
            Assert.Equal(expected.Take(3), index.Nearest(target.X, target.Y, 3));
        }
    }

    [Fact]
    public void TiesCoincidentPointsZVariantsAndFloatingLimitsKeepOriginalOrdinals()
    {
        Point3D[][] cases = [[], [new(0, 0, 3)],
            Enumerable.Range(0, 2048).Select(i => new Point3D((i % 4) switch { 0 => 1, 1 => -1, _ => 0 },
                (i % 4) switch { 2 => 1, 3 => -1, _ => 0 }, i)).ToArray(),
            Enumerable.Range(0, 2048).Select(i => new Point3D(4, 7, i)).ToArray(),
            [new(double.Epsilon, 0, 1), new(-double.Epsilon, 0, 2), new(0, double.Epsilon, 3),
                new(0, -double.Epsilon, 4), new(0, 0, 5)],
            [new(1e308, 1e308, 1), new(-1e308, -1e308, 2), new(1e200, -1e200, 3), new(0, 0, 4)],
            [new(double.NaN, 2, 1), new(double.PositiveInfinity, 2, 2), new(0, 0, 3),
                new(double.NegativeInfinity, 1, 4)]];
        foreach (var points in cases)
        foreach (var target in new[] { (0.0, 0.0), (4.0, 7.0), (1e308, -1e308),
            (double.NaN, 0.0), (double.PositiveInfinity, 0.0) })
        {
            var expected = Reference(points, target.Item1, target.Item2).ToArray();
            var index = new PreparedPointIndex(points);
            Assert.Equal(expected, index.Nearest(target.Item1, target.Item2));
            Assert.Equal(expected.Take(3), index.Nearest(target.Item1, target.Item2, 3));
        }
    }

    [Fact]
    public void PartialQueriesDoNotScanEveryPointAndSnapshotCanBeShared()
    {
        var points = Enumerable.Range(0, 65536).Select(i => new Point3D(i % 256, i / 256, i)).ToArray();
        var expected = Reference(points, 20.2, 17.4).Take(3).ToArray();
        var index = new PreparedPointIndex(points);
        points[0] = new(20.2, 17.4, -1);
        var diagnostics = new PreparedPointIndex.QueryDiagnostics();
        Assert.Equal(expected, index.Nearest(20.2, 17.4, 3, diagnostics: diagnostics));
        Assert.Equal(3, diagnostics.ReturnedPoints);
        Assert.True(diagnostics.PointDistances < 256, diagnostics.Describe());
        output.WriteLine("[PreparedPointIndex.Pruning] " + diagnostics.Describe());
        Parallel.For(0, 32, _ => Assert.Equal(expected, index.Nearest(20.2, 17.4, 3)));
    }

    [Fact]
    public void IncrementalValidRaySearchPreservesBudgetsAndDeduplication()
    {
        var points = Enumerable.Range(0, 2048).Select(i => new Point3D(i % 32, i / 32, i)).ToArray();
        var index = new PreparedPointIndex(points);
        (double X, double Y)[] targets = [(4, 4), (4, 4), (8.3, 1.7), (14, 21), (4, 4)];
        foreach (int budget in new[] { 0, 1, 3, 17, 4096 })
        {
            var expected = Search((x, y) => Reference(points, x, y), budget);
            var actual = Search((x, y) => index.Nearest(x, y), budget);
            Assert.Equal(expected.Candidates, actual.Candidates);
            Assert.Equal(expected.Attempts, actual.Attempts);
            Assert.Equal(expected.BoundedTargets, actual.BoundedTargets);
        }

        (List<((double X, double Y) Target, Point3D End)> Candidates, long Attempts, int BoundedTargets)
            Search(Func<double, double, IEnumerable<Point3D>> nearest, int budget)
        {
            var candidates = new List<((double, double), Point3D)>();
            var seen = new HashSet<((double, double), Point3D)>();
            long attempts = 0;
            int boundedTargets = 0;
            foreach (var target in targets)
            {
                int valid = 0;
                foreach (Point3D point in nearest(target.X, target.Y))
                {
                    if (attempts >= budget) { boundedTargets++; break; }
                    attempts++;
                    if ((int)point.Z % 13 != 0 || !seen.Add((target, point))) continue;
                    candidates.Add((target, point));
                    if (++valid == 3) break;
                }
            }
            return (candidates, attempts, boundedTargets);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConstructionKeepsOrderedCircularAndCoincidentInputWorkBounded(bool coincident)
    {
        var points = Enumerable.Range(0, 65536).Select(i =>
        {
            double angle = 2 * Math.PI * i / 65536;
            return new Point3D(coincident ? 4 : 100 * Math.Cos(angle),
                coincident ? 7 : 100 * Math.Sin(angle), i);
        }).ToArray();
        var diagnostics = new PreparedPointIndex.QueryDiagnostics();
        var index = new PreparedPointIndex(points, diagnostics: diagnostics);
        Assert.True(diagnostics.BuildPartitionVisits < 128L * points.Length, diagnostics.Describe());
        if (!coincident) Assert.True(diagnostics.RobustPartitions > 0, diagnostics.Describe());
        output.WriteLine("[PreparedPointIndex.Construction] coincident=" + coincident + " " + diagnostics.Describe());
        foreach (double x in new[] { -101.2, -20.3, 0, 4, 78.1, 100.2 })
            Assert.Equal(Reference(points, x, 7).Take(3), index.Nearest(x, 7, 3));
    }

    [Fact]
    public void CancellationStopsConstructionAndEnumerationWithoutMutatingSnapshot()
    {
        var points = Enumerable.Range(0, 4096).Select(i => new Point3D(i, i % 17, i)).ToArray();
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => new PreparedPointIndex(points, cancelled.Token));
        var index = new PreparedPointIndex(points);
        Assert.Throws<OperationCanceledException>(() => index.Nearest(0, 0, cancellationToken: cancelled.Token).ToArray());
        using var during = new CancellationTokenSource();
        using var enumerator = index.Nearest(0, 0, cancellationToken: during.Token).GetEnumerator();
        Assert.True(enumerator.MoveNext());
        during.Cancel();
        Assert.Throws<OperationCanceledException>(() => { while (enumerator.MoveNext()) { } });
        Assert.Equal(Reference(points, 0, 0).Take(3), index.Nearest(0, 0, 3));
    }

    [Fact]
    public void FocusedBenchmarkUsesFiveAlternatingRepetitionsWithoutDiagnostics()
    {
        string? destination = Environment.GetEnvironmentVariable("RW_POINT_INDEX_BENCHMARK_OUTPUT");
        if (string.IsNullOrWhiteSpace(destination)) return;
        var rows = new List<object>();
        foreach (int count in new[] { 1024, 8192, 65536 })
        {
            var points = Enumerable.Range(0, count).Select(i =>
            {
                double angle = 2 * Math.PI * i / count;
                return new Point3D(100 * Math.Cos(angle), 100 * Math.Sin(angle), i % 37);
            }).ToArray();
            var targets = Enumerable.Range(0, 192).Select(i =>
            {
                double angle = 2 * Math.PI * (i + .371) / 192;
                return (X: 100.2 * Math.Cos(angle), Y: 100.2 * Math.Sin(angle));
            }).ToArray();
            Run(false); Run(true);
            for (int repetition = 0; repetition < 5; repetition++)
                foreach (bool indexed in repetition % 2 == 0 ? new[] { false, true } : new[] { true, false })
                {
                    GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
                    using var process = Process.GetCurrentProcess();
                    TimeSpan cpu = process.TotalProcessorTime;
                    long bytes = GC.GetAllocatedBytesForCurrentThread();
                    var timer = Stopwatch.StartNew();
                    double checksum = Run(indexed);
                    timer.Stop();
                    rows.Add(new { points = count, queries = targets.Length, repetition, indexed,
                        elapsedMilliseconds = timer.Elapsed.TotalMilliseconds,
                        cpuMilliseconds = (process.TotalProcessorTime - cpu).TotalMilliseconds,
                        allocatedBytes = GC.GetAllocatedBytesForCurrentThread() - bytes,
                        peakProcessMemoryBytes = process.PeakWorkingSet64, checksum });
                    output.WriteLine($"[PreparedPointIndex.Benchmark] points={count} queries={targets.Length} " +
                        $"repetition={repetition} indexed={indexed} elapsedMs={timer.Elapsed.TotalMilliseconds:F3} checksum={checksum:R}");
                }

            double Run(bool indexed)
            {
                var index = indexed ? new PreparedPointIndex(points) : null;
                double checksum = 0;
                foreach (var target in targets)
                {
                    IEnumerable<Point3D> near = indexed ? index!.Nearest(target.X, target.Y, 3)
                        : Reference(points, target.X, target.Y).ToArray().Take(3);
                    foreach (Point3D point in near) checksum += point.X + point.Y + point.Z;
                    // Simulate a three-valid-ray search that rejects nearer rays.
                    int accepted = 0;
                    foreach (Point3D point in indexed ? index!.Nearest(target.X, target.Y) : Reference(points, target.X, target.Y))
                    {
                        if ((int)point.Z % 7 != 0) continue;
                        checksum += point.X + point.Y + point.Z;
                        if (++accepted == 3) break;
                    }
                }
                return checksum;
            }
            Assert.Equal(Run(false), Run(true));
        }
        File.WriteAllText(destination, JsonSerializer.Serialize(new { diagnosticsEnabled = false,
            includesIndexConstruction = true, peakMemoryScope = "cumulative-test-process", rows },
            new JsonSerializerOptions { WriteIndented = true }));
    }

    private static IOrderedEnumerable<Point3D> Reference(IReadOnlyList<Point3D> points, double x, double y)
        => points.OrderBy(point => (point.X - x) * (point.X - x) + (point.Y - y) * (point.Y - y));
}
