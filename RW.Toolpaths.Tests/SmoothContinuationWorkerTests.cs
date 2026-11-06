using System.Collections;
using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class SmoothContinuationWorkerTests
{
    private const double Grade = .5;
    private static readonly VCarveCutterProfile Cutter = new(
        [new(CarvingCutterSectionKind.Linear, .1, .1, 4)], 4);

    [Theory]
    [InlineData(0)]
    [InlineData(12.7)]
    [InlineData(-12.7)]
    public async Task PrivateWorkersMatchTheOriginalGuardAndPublishOnlyAfterJoin(double surface)
    {
        var source = Path([new(-5, 0, -1), new(5, 0, -1)]);
        var parent = new VCarveSmoothing.FloorField([source], Cutter, Grade, 4);
        var wall = Walls(surface);
        var serialFloor = new VCarveSmoothing.FloorField([source], Cutter, Grade, 4);
        var serialWall = Walls(surface);
        var serial = VCarveSmoothing.CreateContinuationGuard(serialFloor, serialWall, surface, Grade, default);
        // Read-only warm parents exercise both cache tiers without giving any
        // worker authority to mutate the original dictionaries during queries.
        parent.Depth(0, 0);
        Assert.True(wall.IsLegalCommandedChord(new(-2, 0, -.5), new(2, 0, -.5)));
        int floorEntries = Entries(parent, "_cache"), wallEntries = Entries(wall, "_chordCache");
        long parentQueries = wall.LiteralChordQueries;
        var queries = Enumerable.Range(0, 48).Select(i => (
            A: new Point3D(-4 + i % 8 * .4, (i % 3 - 1) * .15, i % 4 == 0 ? -1.6 : -.4),
            B: new Point3D(1 + i % 7 * .35, (i % 5 - 2) * .1, i % 4 == 0 ? -1.6 : -.45))).ToArray();
        var expected = queries.Select(q => serial(q.A, q.B)).ToArray();
        Assert.Contains(true, expected);
        Assert.Contains(false, expected);
        var workers = Enumerable.Range(0, 4).Select(_ =>
            VCarveSmoothing.CreateContinuationQueryWorker(parent, wall, surface, Grade, default)!).ToArray();
        Assert.All(workers, worker => Assert.NotNull(worker));
        var actual = new bool[queries.Length];
        await Task.WhenAll(workers.Select((worker, index) => Task.Run(() =>
        {
            for (int i = index; i < queries.Length; i += workers.Length)
                actual[i] = worker.Prove(queries[i].A, queries[i].B);
        })));
        Assert.Equal(expected, actual);
        Assert.Equal(floorEntries, Entries(parent, "_cache"));
        Assert.Equal(wallEntries, Entries(wall, "_chordCache"));
        Assert.Equal(parentQueries, wall.LiteralChordQueries);
        var diagnostics = new List<string>();
        foreach (var worker in workers) worker.Complete(diagnostics.Add);
        Assert.Equal(4, diagnostics.Count(s => s.Contains("event=proof-worker")));
        Assert.True(Entries(parent, "_cache") > floorEntries);
        Assert.True(wall.LiteralChordQueries > parentQueries);
        int messages = diagnostics.Count;
        foreach (var worker in workers) worker.Complete(diagnostics.Add);
        Assert.Equal(messages, diagnostics.Count);
        var merged = VCarveSmoothing.CreateContinuationGuard(parent, wall, surface, Grade, default);
        Assert.Equal(expected, queries.Select(q => merged(q.A, q.B)).ToArray());
    }

    [Fact]
    public void WorkerRetainsIndependentGradeFloorArtworkAndSelectedChannelRefusals()
    {
        var floor = new VCarveSmoothing.FloorField(
            [Path([new(-5, 0, -1), new(5, 0, -1)])], Cutter, Grade, 4);
        var wall = Walls(0, [Box(-3, -2, 3, 2)]);
        var worker = VCarveSmoothing.CreateContinuationQueryWorker(floor, wall, 0, Grade, default)!;
        Assert.True(worker.Prove(new(-2, 0, -.5), new(2, 0, -.5)));
        Assert.False(worker.Prove(new(0, 0, -.2), new(.1, 0, -.7))); // grade
        Assert.False(worker.Prove(new(-2, 0, -1.5), new(2, 0, -1.5))); // original floor
        Assert.False(worker.Prove(new(-4, 0, -.5), new(4, 0, -.5))); // selected channel
        var artworkOnly = VCarveSmoothing.CreateContinuationQueryWorker(null, Walls(0), 0, Grade, default)!;
        Assert.True(artworkOnly.Prove(new(-4, 0, -.5), new(4, 0, -.5)));
        Assert.False(artworkOnly.Prove(new(9.95, -1, -.5), new(9.95, 1, -.5))); // physical tip over artwork
        var diagnostics = new List<string>();
        worker.Complete(diagnostics.Add);
        Assert.Contains(diagnostics, s => s.Contains("gradeRefusals=1") && s.Contains("floorRefusals=1")
            && s.Contains("wallRefusals=1") && s.Contains("wallDomains=2"));
        artworkOnly.Complete();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(12.7)]
    public void WorkerCannotAcceptANominalChordWhoseActualF3DepthCrossesTheOriginalFloor(double surface)
    {
        var floor = new VCarveSmoothing.FloorField(
            [Path([new(-2, 0, -.9997), new(2, 0, -.9997)])], Cutter, Grade, 4);
        var wall = Walls(surface);
        var a = new Point3D(-1, 0, -.9996);
        var b = new Point3D(1, 0, -.9996);
        Assert.True(-a.Z < floor.Depth(0, 0));
        Assert.True(VCarveSmoothing.CreateContinuationGuard(null, wall, surface, Grade, default)(a, b));
        Assert.False(VCarveSmoothing.CreateContinuationGuard(floor, wall, surface, Grade, default)(a, b));
        var worker = VCarveSmoothing.CreateContinuationQueryWorker(floor, wall, surface, Grade, default)!;
        Assert.False(worker.Prove(a, b));
        worker.Complete();
    }

    [Fact]
    public void OpaqueActualStockHasNoInferredPrivateWorkerCapability()
    {
        var owner = new VCarveMaterialSurface([Box(-10, -10, 10, 10)],
            new VCarveClearedStock([], [], Box(-20, -20, 20, 20)), Cutter, 4, 0, default, null);
        var wall = new VCarveSmoothing.CommandClearance([Box(-10, -10, 10, 10)],
            Cutter, 0, default, null, stockContact: owner);
        Assert.Null(VCarveSmoothing.CreateContinuationQueryWorker(null, wall, 0, Grade, default));
        Assert.NotNull(VCarveSmoothing.CreateContinuationGuard(null, wall, 0, Grade, default));
    }

    [Fact]
    public void CancellationNeverPublishesPrivateAnswers()
    {
        using var cancellation = new CancellationTokenSource();
        var source = Path([new(-5, 0, -1), new(5, 0, -1)]);
        var floor = new VCarveSmoothing.FloorField([source], Cutter, Grade, 4, cancellation.Token);
        var wall = new VCarveSmoothing.CommandClearance([Box(-10, -10, 10, 10)],
            Cutter, 0, cancellation.Token, null);
        var worker = VCarveSmoothing.CreateContinuationQueryWorker(floor, wall, 0, Grade, cancellation.Token)!;
        Assert.True(worker.Prove(new(-2, 0, -.5), new(2, 0, -.5)));
        int floorEntries = Entries(floor, "_cache"), wallEntries = Entries(wall, "_chordCache");
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => worker.Prove(new(-2, 0, -.5), new(2, 0, -.5)));
        Assert.Throws<OperationCanceledException>(() => worker.Complete());
        Assert.Throws<OperationCanceledException>(() =>
            VCarveSmoothing.CreateContinuationQueryWorker(floor, wall, 0, Grade, cancellation.Token));
        Assert.Equal(floorEntries, Entries(floor, "_cache"));
        Assert.Equal(wallEntries, Entries(wall, "_chordCache"));
    }

    [Theory]
    [InlineData(4)]
    [InlineData(6)]
    public void ParallelSimplificationMatchesSerialPathsMotionKindsMetadataAndOriginalOrder(int maximumWorkers)
    {
        var paths = SimplificationPaths();
        var floorSource = Path([new(-5, 0, -1), new(5, 0, -1)]);
        var serialFloor = new VCarveSmoothing.FloorField([floorSource], Cutter, Grade, 4);
        var serialWall = Walls(12.7, [Box(-8, -8, 8, 8)]);
        var serial = VCarveSmoothing.CreateContinuationGuard(serialFloor, serialWall, 12.7, Grade, default);
        // The prefix is an additional exact authority used in BOTH branches.
        // Refusing long chords on one row exercises retained source points.
        bool Prefix(Point3D a, Point3D b) => a.Y < 1 || Math.Abs(b.X - a.X) <= .8;
        var expected = VCarveToolpaths.SimplifySurfacePaths(paths, .002, default,
            (a, b) => Prefix(a, b) && serial(a, b), null, maximumWorkers: 1);
        var floor = new VCarveSmoothing.FloorField([floorSource], Cutter, Grade, 4);
        var wall = Walls(12.7, [Box(-8, -8, 8, 8)]);
        int factories = 0, serialCalls = 0, active = 0, maximumActive = 0;
        var diagnostics = new List<string>();
        var actual = VCarveToolpaths.SimplifySurfacePaths(paths, .002, default,
            (a, b) => { Interlocked.Increment(ref serialCalls); return serial(a, b); },
            () => { factories++; return VCarveSmoothing.CreateContinuationQueryWorker(floor, wall, 12.7, Grade, default); },
            diagnostics.Add, maximumWorkers, (a, b) =>
            {
                int current = Interlocked.Increment(ref active);
                int previous;
                do { previous = Volatile.Read(ref maximumActive); }
                while (current > previous && Interlocked.CompareExchange(ref maximumActive, current, previous) != previous);
                try { return Prefix(a, b); }
                finally { Interlocked.Decrement(ref active); }
            });
        Assert.Equal(maximumWorkers, factories);
        Assert.Equal(0, serialCalls);
        Assert.InRange(maximumActive, 1, maximumWorkers);
        Assert.Equal(0, active);
        EqualPaths(expected, actual);
        Assert.Same(paths[0].Points, actual[0].Points); // original final-carve remains literal
        Assert.True(actual.Skip(1).Sum(p => p.Points.Count) < paths.Skip(1).Sum(p => p.Points.Count));
        Assert.Contains(diagnostics, s => s.Contains("event=protected-sources") && s.Contains($"workers={maximumWorkers}")
            && s.Contains("allWorkersJoined=True"));
        Assert.Equal(maximumWorkers, diagnostics.Count(s => s.Contains("event=proof-worker")));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OpaqueOrUnavailableFactoryUsesTheLiteralSequentialCallback(bool unavailable)
    {
        var paths = SimplificationPaths();
        int active = 0, maximumActive = 0, calls = 0, factories = 0;
        bool Accept(Point3D a, Point3D b)
        {
            int current = ++active; maximumActive = Math.Max(maximumActive, current); calls++;
            try { return Math.Abs(b.X - a.X) <= 1; }
            finally { active--; }
        }
        var expected = paths.Select(path => path.Category == "final-carve" ? path
            : SurfaceToolpathSimplifier.Simplify(path, .002, default, (a, b) => Math.Abs(b.X - a.X) <= 1)).ToList();
        var diagnostics = new List<string>();
        var actual = VCarveToolpaths.SimplifySurfacePaths(paths, .002, default, Accept,
            unavailable ? () => { factories++; return null; } : null, diagnostics.Add, maximumWorkers: 6,
            privateWorkerPrefix: (_, _) => throw new InvalidOperationException("No private capability was supplied."));
        Assert.True(calls > 0);
        Assert.Equal(1, maximumActive);
        Assert.Equal(0, active);
        Assert.Equal(unavailable ? 1 : 0, factories);
        EqualPaths(expected, actual);
        Assert.Contains(diagnostics, s => s.Contains("event=protected-sources") && s.Contains("workers=1"));
        Assert.DoesNotContain(diagnostics, s => s.Contains("event=proof-worker"));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CancellationOrCallbackFailureJoinsWorkersBeforeReturningAndNeverMergesAPrefix(bool cancel)
    {
        using var cancellation = new CancellationTokenSource();
        var paths = SimplificationPaths();
        var wall = new VCarveSmoothing.CommandClearance([Box(-10, -10, 10, 10)],
            Cutter, 0, cancellation.Token, null);
        var diagnostics = new List<string>();
        int active = 0, calls = 0;
        List<TaggedToolpath> Run() => VCarveToolpaths.SimplifySurfacePaths(paths, .002, cancellation.Token,
            (_, _) => throw new InvalidOperationException("Serial callback must not run."),
            () => VCarveSmoothing.CreateContinuationQueryWorker(null, wall, 0, Grade, cancellation.Token),
            diagnostics.Add, maximumWorkers: 4, privateWorkerPrefix: (_, _) =>
            {
                Interlocked.Increment(ref active);
                try
                {
                    if (Interlocked.Increment(ref calls) == 1)
                    {
                        if (cancel) cancellation.Cancel();
                        else throw new InvalidOperationException("Injected private prefix failure");
                    }
                    return true;
                }
                finally { Interlocked.Decrement(ref active); }
            });
        if (cancel) Assert.ThrowsAny<OperationCanceledException>(() => Run());
        else Assert.Equal("Injected private prefix failure", Assert.Throws<InvalidOperationException>(() => Run()).Message);
        Assert.True(calls > 0);
        Assert.Equal(0, Volatile.Read(ref active));
        Assert.Equal(0, Entries(wall, "_chordCache"));
        Assert.Equal(0, wall.LiteralChordQueries);
        Assert.DoesNotContain(diagnostics, s => s.Contains("event=proof-worker") || s.Contains("event=protected-sources"));
        Assert.All(paths, p => Assert.Equal(65, p.Points.Count));
    }

    private static List<TaggedToolpath> SimplificationPaths() => Enumerable.Range(0, 8).Select(ordinal =>
    {
        var points = Enumerable.Range(0, 65).Select(i =>
            new Point3D(-4 + i / 8d, -3.5 + ordinal, -.5 + (i % 2 == 0 ? 0 : .0002))).ToArray();
        return new TaggedToolpath(points,
            [new(0, 24, ToolpathSpanKind.Cut), new(24, 40, ToolpathSpanKind.Link),
                new(40, 48, ToolpathSpanKind.Cut), new(48, 64, ToolpathSpanKind.ClearedTravel)],
            ordinal, ordinal == 0 ? "final-carve" : "smooth-ramp", ordinal % 2, 100 + ordinal, detectCurves: false)
        { ContinuationComponentIndex = ordinal, ContinuationSourceOrdinal = ordinal + 20 };
    }).ToList();

    private static void EqualPaths(IReadOnlyList<TaggedToolpath> expected, IReadOnlyList<TaggedToolpath> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
        {
            Assert.Equal(expected[i].Points, actual[i].Points);
            Assert.Equal(expected[i].Spans, actual[i].Spans);
            Assert.Equal(expected[i].RegionIndex, actual[i].RegionIndex);
            Assert.Equal(expected[i].Category, actual[i].Category);
            Assert.Equal(expected[i].DepthPassIndex, actual[i].DepthPassIndex);
            Assert.Equal(expected[i].VisitIdentity, actual[i].VisitIdentity);
            Assert.Equal(expected[i].ContinuationComponentIndex, actual[i].ContinuationComponentIndex);
            Assert.Equal(expected[i].ContinuationSourceOrdinal, actual[i].ContinuationSourceOrdinal);
            Assert.Equal(expected[i].DetectCurves, actual[i].DetectCurves);
        }
    }

    private static VCarveSmoothing.CommandClearance Walls(double surface,
        IReadOnlyList<IReadOnlyList<PointD>>? channel = null) =>
        new([Box(-10, -10, 10, 10)], Cutter, surface, default, null, originalChannel: channel);

    private static TaggedToolpath Path(IReadOnlyList<Point3D> points, int ordinal = 0, string category = "smooth-ramp") =>
        new(points, Array.Empty<ToolpathSpan>(), ordinal, category, ordinal % 2, 100 + ordinal, detectCurves: false)
        { ContinuationComponentIndex = ordinal, ContinuationSourceOrdinal = ordinal + 20 };

    private static PointD[] Box(double minX, double minY, double maxX, double maxY) =>
        [new(minX, minY), new(maxX, minY), new(maxX, maxY), new(minX, maxY)];

    private static int Entries(object owner, string name) =>
        ((IDictionary)owner.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(owner)!).Count;
}
