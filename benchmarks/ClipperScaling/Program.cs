using System.Collections;
using System.Diagnostics;
using System.Reflection;
using System.Runtime.Loader;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;

if (args.Length < 3) throw new ArgumentException("ClipperScaling <output.json> <baseline.dll> <candidate.dll> [other.dll ...] [--fixtures=directory]");
string output = Path.GetFullPath(args[0]);
var libraries = args.Skip(1).Where(a => !a.StartsWith("--")).Select(p => new Library(p)).ToArray();
string? fixtureDirectory = args.FirstOrDefault(a => a.StartsWith("--fixtures="))?.Split('=', 2)[1];
bool diagnosticOnly = args.Contains("--diagnostic-only");
bool fixturesOnly = args.Contains("--fixtures-only");
bool queueOnly = args.Contains("--queue-only");
bool traceScanlines = Environment.GetEnvironmentVariable("RW_CLIPPER_TRACE_SCANLINES") == "1";
if (diagnosticOnly && !traceScanlines) throw new ArgumentException("--diagnostic-only requires RW_CLIPPER_TRACE_SCANLINES=1; empty traces are not sequence evidence.");
long comparedScanlines = 0, sequenceComparisons = 0;
var capturedOperations = new Dictionary<string, (string Operation, string Fill, bool Preserve, bool Reverse)>();
if (!diagnosticOnly && libraries.Any(l => l.Assembly.GetType("Clipper2Lib.ClipperDiagnostics") != null))
    throw new ArgumentException("Timing runs require counters compiled out; use --diagnostic-only for counter/sequence checks.");
var checks = new List<object>();
var measurements = new List<object>();
var baseline = libraries[0];
foreach (var library in libraries.Skip(1))
{
    Require(baseline.Assembly.FullName == library.Assembly.FullName, "assembly identity " + library.Path);
    Require(baseline.Api() == library.Api(), "public API " + library.Path);
}
var rectangle = new long[][] { new[] { 0L, 0L }, new[] { 1000L, 0L }, new[] { 1000L, 1000L }, new[] { 0L, 1000L } };
var hole = new long[][] { new[] { 200L, 200L }, new[] { 200L, 800L }, new[] { 800L, 800L }, new[] { 800L, 200L } };
var touching = rectangle.Select(p => new[] { p[0] + 1000, p[1] }).ToArray();
var crossing = new long[][] { new[] { -50L, 500L }, new[] { 1100L, 500L }, new[] { 500L, -50L } };
var open = new long[][] { new[] { -100L, 500L }, new[] { 500L, 500L }, new[] { 1100L, 500L } };
var cases = new List<(string Name, long[][][] Subject, long[][][] Clip, long[][][] Open)> {
    ("holes-touch-open", new[] { rectangle, hole, touching }, new[] { crossing }, new[] { open }),
    ("empty", Array.Empty<long[][]>(), Array.Empty<long[][]>(), Array.Empty<long[][]>()),
    ("duplicates-horizontal", new[] { rectangle.Concat(new[] { rectangle[3], rectangle[0] }).ToArray(), rectangle }, new[] { touching }, new[] { open }),
    ("extreme-long", new[] { rectangle.Select(p => new[] { p[0] + (1L << 53), p[1] - (1L << 53) }).ToArray() }, Array.Empty<long[][]>(), Array.Empty<long[][]>())
};
var random = new Random(17041);
for (int i = 0; i < 30; i++)
{
    long[][] Ring() => Enumerable.Range(0, 20).Select(_ => new[] { (long)random.Next(-2000, 2001), (long)random.Next(-2000, 2001) }).ToArray();
    cases.Add(("self-intersections-" + i, new[] { Ring(), Ring() }, new[] { Ring() }, new[] { Ring().Take(6).ToArray() }));
}
if (fixtureDirectory is not null)
    foreach (string path in Directory.EnumerateFiles(fixtureDirectory, "*.json").OrderBy(p => p, StringComparer.Ordinal))
    {
        using var doc = JsonDocument.Parse(File.ReadAllText(path));
        if (!doc.RootElement.TryGetProperty("subject", out var subject)) continue;
        capturedOperations[Path.GetFileNameWithoutExtension(path)] = (doc.RootElement.GetProperty("operation").GetString()!, doc.RootElement.GetProperty("fill").GetString()!, doc.RootElement.GetProperty("preserveCollinear").GetBoolean(), doc.RootElement.GetProperty("reverse").GetBoolean());
        cases.Add((Path.GetFileNameWithoutExtension(path), subject.Deserialize<long[][][]>()!,
            doc.RootElement.GetProperty("clip").Deserialize<long[][][]>()!,
            doc.RootElement.GetProperty("open").Deserialize<long[][][]>()!));
    }
if (fixturesOnly) cases.RemoveRange(0, 34);
foreach (var fixture in cases)
    foreach (string fill in new[] { "EvenOdd", "NonZero", "Positive", "Negative" })
        foreach (string operation in new[] { "Union", "Difference", "Intersection", "Xor" })
            foreach (bool reverse in new[] { false, true })
            foreach (bool preserve in new[] { false, true })
            {
                var prepared = libraries.Select(l => l.Prepare(fixture.Subject, fixture.Clip, fixture.Open, operation, fill, reverse, preserve)).ToArray();
                string expected = prepared[0].RunHash();
                for (int i = 1; i < prepared.Length; i++)
                {
                    Require(prepared[i].RunHash() == expected && prepared[i].RunHash() == expected,
                        $"ordered output/repeated execution {fixture.Name} {fill} {operation} reverse={reverse} {libraries[i].Path}");
                    if (diagnosticOnly)
                    {
                        long[] sequence = prepared[0].Scanlines();
                        Require(prepared[i].Scanlines().SequenceEqual(sequence), "executed scanlines " + fixture.Name);
                        comparedScanlines += sequence.Length; sequenceComparisons++;
                    }
                }
            }
checks.Add(new { kind = "ordered-polygons-open-paths-repeat", cases = cases.Count, executions = cases.Count * 64 * (1 + 2 * (libraries.Length - 1)) });
foreach (var fixture in cases.Take(4))
    foreach (string join in new[] { "Miter", "Round", "Square" })
        foreach (double delta in new[] { -12.5, 0, 25.5 })
        {
            string expected = baseline.Offset(fixture.Subject, join, delta);
            foreach (var l in libraries.Skip(1)) Require(l.Offset(fixture.Subject, join, delta) == expected, "offset " + fixture.Name);
        }
checks.Add(new { kind = "offsets", cases = 36 });
foreach (int size in new[] { 0, 1, 2, 9, 256, 4096 })
{
    var values = Enumerable.Range(0, size).Select(_ => random.NextInt64(-1000, 1001)).Concat(new[] { long.MinValue, long.MaxValue, long.MaxValue }).ToArray();
    foreach (var l in libraries) Require(l.Queue(values, randomSeed: size).SequenceEqual(baseline.Queue(values, randomSeed: size)), "scanline sequence size=" + size);
}
checks.Add(new { kind = "descending-distinct-interleaved-events", sizes = new[] { 0, 1, 2, 9, 256, 4096 } });
// Change one dimension at a time: disjoint vertical placements grow distinct Y
// without intersections; equal-Y placements grow vertices without distinct Y.
if (!diagnosticOnly && !fixturesOnly && !queueOnly) foreach (int count in new[] { 250, 1000, 4000, 16000 })
    foreach (bool distinctY in new[] { false, true })
        Measure($"disjoint-{count}-distinctY-{distinctY}", Enumerable.Range(0, count).Select(i => new long[][] {
            new[] { i * 20L, distinctY ? i * 4L : 0L }, new[] { i * 20L + 10, distinctY ? i * 4L + 1 : 1L },
            new[] { i * 20L + 10, distinctY ? i * 4L + 3 : 3L }, new[] { i * 20L, distinctY ? i * 4L + 2 : 2L }
        }).ToArray(), Array.Empty<long[][]>(), Array.Empty<long[][]>());
if (!diagnosticOnly && !fixturesOnly && !queueOnly) foreach (int count in new[] { 64, 128, 256 })
    Measure("crossing-" + count, Enumerable.Range(0, count).Select(i => new long[][] {
        new[] { i * 20L, 0L }, new[] { count * 20L - i * 20L + 9, 4000L },
        new[] { count * 20L - i * 20L, 4000L }, new[] { i * 20L + 9, 0L }
    }).ToArray(), Array.Empty<long[][]>(), Array.Empty<long[][]>());
if (!diagnosticOnly && !queueOnly) foreach (var fixture in cases.Skip(fixturesOnly ? 0 : 34))
{
    var operation = capturedOperations[fixture.Name];
    Measure(fixture.Name, fixture.Subject, fixture.Clip, fixture.Open, operation.Operation, operation.Fill, operation.Preserve, operation.Reverse);
}
if (!diagnosticOnly && !fixturesOnly) foreach (int count in new[] { 1000, 8000, 64000, 174000 })
{
    var queueRuns = libraries.Select(l => l.PrepareQueue()).ToArray();
    foreach (var run in queueRuns) run(count);
    for (int repetition = 0; repetition < 5; repetition++)
        foreach (int i in (repetition % 2 == 0 ? Enumerable.Range(0, libraries.Length) : Enumerable.Range(0, libraries.Length).Reverse()))
        {
            using var process = Process.GetCurrentProcess();
            double cpuBefore = process.TotalProcessorTime.TotalMilliseconds;
            long allocatedBefore = GC.GetTotalAllocatedBytes(true);
            long started = Stopwatch.GetTimestamp();
            long checksum = queueRuns[i](count);
            double ms = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            long allocated = GC.GetTotalAllocatedBytes(true) - allocatedBefore;
            double cpu = process.TotalProcessorTime.TotalMilliseconds - cpuBefore;
            process.Refresh();
            measurements.Add(new { fixture = "queue-descending-" + count, build = i, repetition, milliseconds = ms, cpuMilliseconds = cpu, allocatedBytes = allocated, peakWorkingSetBytes = process.PeakWorkingSet64, checksum });
        }
}
Directory.CreateDirectory(Path.GetDirectoryName(output)!);
File.WriteAllText(output, JsonSerializer.Serialize(new { status = "passed", libraries = libraries.Select(l => new {
    l.Path, identity = l.Assembly.FullName, sha256 = Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(l.Path))) }), checks, measurements, traceScanlines, sequenceComparisons, comparedScanlines,
    timing = "Five alternating repetitions after one untimed warmup. Counters must be compiled out. Reflection input/output conversion excluded; Execute includes output construction. CPU/allocation are process-wide deltas; peak working set is process lifetime and cannot be attributed to one call." }, new JsonSerializerOptions { WriteIndented = true }));
Console.WriteLine($"Passed {cases.Count} fixtures across {libraries.Length} builds; results: {output}");
void Require(bool pass, string message) { if (!pass) throw new InvalidOperationException("FAIL " + message); }
void Measure(string name, long[][][] subject, long[][][] clip, long[][][] opened, string operation = "Union", string fill = "NonZero", bool preserve = true, bool reverse = false)
{
    var prepared = libraries.Select(l => l.Prepare(subject, clip, opened, operation, fill, reverse, preserve)).ToArray();
    string expected = prepared[0].RunHash();
    foreach (var p in prepared) Require(p.RunHash() == expected, "benchmark output " + name);
    for (int repetition = 0; repetition < 5; repetition++)
        foreach (int i in (repetition % 2 == 0 ? Enumerable.Range(0, libraries.Length) : Enumerable.Range(0, libraries.Length).Reverse()))
        {
            GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
            using var process = Process.GetCurrentProcess();
            double cpuBefore = process.TotalProcessorTime.TotalMilliseconds;
            long allocationBefore = GC.GetTotalAllocatedBytes(true);
            long started = Stopwatch.GetTimestamp();
            prepared[i].Execute();
            double milliseconds = Stopwatch.GetElapsedTime(started).TotalMilliseconds;
            long allocated = GC.GetTotalAllocatedBytes(true) - allocationBefore;
            double cpu = process.TotalProcessorTime.TotalMilliseconds - cpuBefore;
            process.Refresh();
            Require(prepared[i].Hash() == expected, "timed output " + name);
            measurements.Add(new { fixture = name, operation, fill, preserveCollinear = preserve, reverse, build = i, repetition, vertices = subject.Sum(p => p.Length), milliseconds, cpuMilliseconds = cpu, allocatedBytes = allocated, peakWorkingSetBytes = process.PeakWorkingSet64, outputSha256 = expected });
        }
}

sealed class Library
{
    private delegate bool PopEvent(out long value);
    public string Path { get; }
    public Assembly Assembly { get; }
    readonly Type point, path, paths, engine, baseType;
    public Library(string file)
    {
        Path = System.IO.Path.GetFullPath(file);
        Assembly = new AssemblyLoadContext("clipper-" + Guid.NewGuid(), isCollectible: true).LoadFromAssemblyPath(Path);
        Type T(string name) => Assembly.GetType("Clipper2Lib." + name, true)!;
        point = T("Point64"); path = T("Path64"); paths = T("Paths64"); engine = T("Clipper64"); baseType = T("ClipperBase");
    }
    public string Api() => string.Join('\n', Assembly.GetExportedTypes().OrderBy(t => t.FullName).SelectMany(t => new[] { t.FullName! }.Concat(t.GetMembers(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly).Select(m => m.ToString()!).OrderBy(s => s))));
    public object Paths(long[][][] data)
    {
        var result = (IList)Activator.CreateInstance(paths)!;
        foreach (var ring in data) { var p = (IList)Activator.CreateInstance(path)!; foreach (var v in ring) p.Add(Activator.CreateInstance(point, v[0], v[1])!); result.Add(p); }
        return result;
    }
    object EnumValue(string type, string value) => Enum.Parse(Assembly.GetType("Clipper2Lib." + type, true)!, value);
    public Prepared Prepare(long[][][] subject, long[][][] clip, long[][][] open, string operation, string fill, bool reverse, bool preserve = true)
    {
        var e = Activator.CreateInstance(engine)!;
        engine.GetProperty("ReverseSolution")!.SetValue(e, reverse);
        engine.GetProperty("PreserveCollinear")!.SetValue(e, preserve);
        engine.GetMethod("AddSubject", new[] { paths })!.Invoke(e, new[] { Paths(subject) });
        engine.GetMethod("AddClip", new[] { paths })!.Invoke(e, new[] { Paths(clip) });
        engine.GetMethod("AddOpenSubject", new[] { paths })!.Invoke(e, new[] { Paths(open) });
        return new Prepared(this, e, engine.GetMethods().Single(m => m.Name == "Execute" && m.GetParameters().Length == 4 && m.GetParameters()[2].ParameterType == paths),
            new[] { EnumValue("ClipType", operation), EnumValue("FillRule", fill), Activator.CreateInstance(paths)!, Activator.CreateInstance(paths)! });
    }
    public string Offset(long[][][] source, string join, double delta)
    {
        var t = Assembly.GetType("Clipper2Lib.ClipperOffset", true)!;
        var e = Activator.CreateInstance(t, 2.0, .25, false, false)!;
        t.GetMethod("AddPaths")!.Invoke(e, new[] { Paths(source), EnumValue("JoinType", join), EnumValue("EndType", "Polygon") });
        var result = Activator.CreateInstance(paths)!;
        t.GetMethod("Execute", new[] { typeof(double), paths })!.Invoke(e, new[] { (object)delta, result });
        return Hash(result);
    }
    public long[] Queue(long[] values, int randomSeed)
    {
        object e = Activator.CreateInstance(engine)!;
        var reset = baseType.GetMethod("Reset", BindingFlags.NonPublic | BindingFlags.Instance)!;
        var insert = baseType.GetMethod("InsertScanline", BindingFlags.NonPublic | BindingFlags.Instance)!;
        var pop = baseType.GetMethod("PopScanline", BindingFlags.NonPublic | BindingFlags.Instance)!;
        reset.Invoke(e, null);
        var result = new List<long>();
        var random = new Random(randomSeed);
        var pending = new SortedSet<long>();
        for (int i = 0; i < values.Length; i++)
        {
            // Events added between pops cannot lie above the consumed scanline.
            long value = result.Count == 0 ? values[i] : Math.Min(result[^1], values[i]);
            insert.Invoke(e, new object[] { value }); pending.Add(value);
            if (random.Next(5) == 0) Pop();
        }
        while (pending.Count > 0) Pop();
        var empty = new object[] { 0L };
        if ((bool)pop.Invoke(e, empty)!) throw new InvalidOperationException("queue failed empty");
        reset.Invoke(e, null); insert.Invoke(e, new object[] { 19L });
        if (!(bool)pop.Invoke(e, empty)! || (long)empty[0] != 19) throw new InvalidOperationException("queue failed reuse");
        return result.ToArray();
        void Pop() { var a = new object[] { 0L }; if (!(bool)pop.Invoke(e, a)! || (long)a[0] != pending.Max) throw new InvalidOperationException("queue failed expected event"); result.Add((long)a[0]); pending.Remove(pending.Max); }
    }
    public Func<int, long> PrepareQueue()
    {
        object e = Activator.CreateInstance(engine)!;
        var clear = (Action)baseType.GetMethod("Clear")!.CreateDelegate(typeof(Action), e);
        var insert = (Action<long>)baseType.GetMethod("InsertScanline", BindingFlags.NonPublic | BindingFlags.Instance)!.CreateDelegate(typeof(Action<long>), e);
        var pop = (PopEvent)baseType.GetMethod("PopScanline", BindingFlags.NonPublic | BindingFlags.Instance)!.CreateDelegate(typeof(PopEvent), e);
        return count => { clear(); for (int i = count; i > 0; i--) insert(i); long sum = 0; while (pop(out long y)) sum += y; if (sum != (long)count * (count + 1) / 2) throw new InvalidOperationException("queue checksum"); return sum; };
    }
    public string Hash(params object[] values)
    {
        using var hash = IncrementalHash.CreateHash(HashAlgorithmName.SHA256);
        foreach (IEnumerable value in values)
        {
            int count = 0;
            foreach (IEnumerable ring in value)
            {
                int points = 0;
                foreach (object p in ring) { hash.AppendData(BitConverter.GetBytes((long)point.GetField("X")!.GetValue(p)!)); hash.AppendData(BitConverter.GetBytes((long)point.GetField("Y")!.GetValue(p)!)); points++; }
                hash.AppendData(BitConverter.GetBytes(points)); count++;
            }
            hash.AppendData(BitConverter.GetBytes(count));
        }
        return Convert.ToHexString(hash.GetHashAndReset());
    }
}
sealed class Prepared(Library library, object engine, MethodInfo execute, object[] arguments)
{
    public void Execute() { if (!(bool)execute.Invoke(engine, arguments)!) throw new InvalidOperationException("Clipper Execute returned false"); }
    public string Hash() => library.Hash(arguments[2], arguments[3]);
    public string RunHash() { Execute(); return Hash(); }
    public long[] Scanlines()
    {
        object diagnostics = library.Assembly.GetType("Clipper2Lib.ClipperBase")!.GetField("_diagnostics", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(engine)!;
        long[] sequence = ((IEnumerable)diagnostics.GetType().GetField("Scanlines", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(diagnostics)!).Cast<long>().ToArray();
        long consumed = (long)diagnostics.GetType().GetField("ConsumedScanlines", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(diagnostics)!;
        if (sequence.Length != consumed) throw new InvalidOperationException("Retained trace is incomplete.");
        return sequence;
    }
}
