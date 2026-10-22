using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveStayDownLinkerTests
{
    private static TaggedToolpath Path(Point3D a, Point3D b, int region = 0, int? pass = null,
        string category = "clearing") => new([a, b], [], region, category, pass, detectCurves: false);

    [Fact]
    public void AdjacentRowsBecomeOneSerpentineVisitAndRetainEveryOriginalCut()
    {
        var source = Enumerable.Range(0, 5).Select(i => Path(new(0, i * .4, -1 - i * .1),
            new(10, i * .4, -1 - i * .1))).ToArray();
        int checks = 0;
        var result = VCarveStayDownLinker.Join(source, .8, (a, b) => {
            checks++;
            Assert.True(Math.Abs(a.Z - b.Z) / Math.Abs(a.Y - b.Y) < .3);
            return true;
        }, default);
        var joined = Assert.Single(result);
        Assert.Equal(4, checks);
        Assert.Equal(10, joined.Points.Count);
        AssertOriginalChords(source, result);
    }

    [Fact]
    public void ARefusedConnectorKeepsBothOriginalCutsAndADistantPairNeedsNoProof()
    {
        var source = new[] { Path(new(0, 0, -1), new(2, 0, -1)),
            Path(new(0, .4, -1), new(2, .4, -1)), Path(new(0, 10, -1), new(2, 10, -1)) };
        int calls = 0;
        var result = VCarveStayDownLinker.Join(source, 1, (_, _) => { calls++; return false; }, default);
        Assert.Equal(3, result.Count);
        Assert.Equal(1, calls);
        AssertOriginalChords(source, result);
    }

    [Fact]
    public void RegionCategoryAndExistingDepthPassRemainSeparate()
    {
        var source = new[] { Path(new(0, 0, -1), new(2, 0, -1)),
            Path(new(0, .1, -1), new(2, .1, -1), region: 1),
            Path(new(0, .2, -1), new(2, .2, -1), region: 1, pass: 0),
            Path(new(0, .3, -1), new(2, .3, -1), region: 1, pass: 0, category: "final-carve") };
        var result = VCarveStayDownLinker.Join(source, 1, (_, _) => throw new Exception("No cross-group connection allowed"), default);
        Assert.Equal(4, result.Count);
        AssertOriginalChords(source, result);
    }

    [Fact]
    public void EveryNewConnectorReceivesSurfaceOriginPrecursorPasses()
    {
        var source = new[] { Path(new(0, 0, -5), new(10, 0, -5)),
            Path(new(0, .4, -5.1), new(10, .4, -5.1)) };
        var joined = Assert.Single(VCarveStayDownLinker.Join(source, 1, (_, _) => true, default));
        var schedule = VCarveToolpaths.ScheduleFreshStockVisits([joined], new DepthSchedule(6, 2), default, null);
        Assert.Equal(3, schedule.Count);
        for (int pass = 0; pass < schedule.Count; pass++)
        {
            var visit = schedule[pass];
            Assert.Equal(joined.Points.Select(p => (p.X, p.Y)), visit.Points.Select(p => (p.X, p.Y)));
            for (int i = 0; i < visit.Points.Count; i++)
            {
                double previousDepth = pass == 0 ? 0 : -schedule[pass - 1].Points[i].Z;
                Assert.InRange(-visit.Points[i].Z - previousDepth, 0, 2.000000001);
            }
        }
    }

    [Fact]
    public void ExistingMotionSpansRemainIntact()
    {
        var ramp = Path(new(0, 0, 0), new(2, 0, -1)) with { Spans = new[] { new ToolpathSpan(0, 1, ToolpathSpanKind.Ramp) } };
        var cut = Path(new(0, .4, -1), new(2, .4, -1));
        var result = VCarveStayDownLinker.Join([ramp, cut], 1, (_, _) => throw new Exception("Existing motion span"), default);
        Assert.Equal(new[] { ramp, cut }, result);
    }

    [Fact]
    public void CancellationPropagatesBeforeAnyRouting()
    {
        using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveStayDownLinker.Join([], 1, (_, _) => true, cancelled.Token));
    }

    private static void AssertOriginalChords(IEnumerable<TaggedToolpath> source, IEnumerable<TaggedToolpath> result)
    {
        var chords = result.SelectMany(path => path.Points.Zip(path.Points.Skip(1))).ToArray();
        foreach (var path in source)
        foreach (var (a, b) in path.Points.Zip(path.Points.Skip(1)))
            Assert.Contains(chords, chord => chord.First == a && chord.Second == b || chord.First == b && chord.Second == a);
    }
}
