using Clipper2Lib;
using RW.Toolpaths.Milling;
using System.Globalization;

namespace RW.Toolpaths.Tests;

public sealed class RampEntryStockTests
{
    [Fact]
    public void LegacyPublicEntrySignatureAndOptionalDefaultsRemainAvailable()
    {
        var method = typeof(RampPlanner).GetMethod(nameof(RampPlanner.PlanPathEntry),
            [typeof(IReadOnlyList<Point3D>), typeof(double), typeof(RampSettings),
                typeof(CancellationToken), typeof(Action<string>), typeof(bool)]);
        Assert.NotNull(method);
        var parameters = method.GetParameters();
        Assert.True(parameters[3].IsOptional);
        Assert.True(parameters[4].IsOptional);
        Assert.True(parameters[5].IsOptional);
        Assert.Null(parameters[4].DefaultValue);
        Assert.Equal(false, parameters[5].DefaultValue);
        Assert.Single(typeof(RampPlanner).GetMethods().Where(m => m.Name == nameof(RampPlanner.PlanPathEntry)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PublicClosedEntryKeepsLegacySeamForUnknownDownstreamConsumers(bool vertexContact)
    {
        Point3D[] ring = [new(0, 0, -1), new(10, 0, -1), new(10, 10, -1), new(0, 10, -1), new(0, 0, -1)];
        var messages = new List<string>();
        var entered = RampPlanner.PlanPathEntry(ring, 0, RampSettings.FromRatio(4),
            diagnostic: messages.Add, landAtSourceVertex: vertexContact);
        var ramp = entered.Spans.Single(span => span.Kind == ToolpathSpanKind.Ramp);
        var expected = entered.Points.Take(ramp.EndIndex)
            .Concat(LegacyCut(ring, entered.Points[ramp.EndIndex])).ToArray();
        Assert.Equal(expected, entered.Points);
        Assert.Equal(ring[0], entered.Points[^1]);
        Assert.Equal(0, entered.RedundantPointsRemoved);
        Assert.Equal(0, entered.RedundantCutTravelRemoved);
        Assert.Contains(messages, message => message.Contains("closedTailReductionAllowed=False")
            && message.Contains("finalExit=(0,0,-1)"));

        // The fixture would change its exit under the explicit V-carve policy.
        // This keeps the public seam assertion falsifiable rather than vacuous.
        var optimized = PlanOptimizedEntry(ring, 0, RampSettings.FromRatio(4), landAtSourceVertex: vertexContact);
        Assert.Equal(ring[1], optimized.Points[^1]);
        Assert.True(optimized.RedundantPointsRemoved > 0);
    }

    [Fact]
    public void PocketReentryKeepsPostedOutgoingTailChordAndItsRemovedStock()
    {
        Point3D[] ring = [new(0, 0, -1), new(10, 0, -1), new(10, 10, -1), new(0, 10, -1), new(0, 0, -1)];
        Point3D destination = new(5, 5, -1);
        var tail = new TaggedToolpath(ring.Concat([destination, new Point3D(6, 5, -1)]).ToList(),
            [new(0, 4, ToolpathSpanKind.Cut), new(4, 5, ToolpathSpanKind.Link), new(5, 6, ToolpathSpanKind.Cut)],
            3, "pocket", 0, 17, detectCurves: false);
        // Exercise the real entry-plus-tail splice without changing its visibility
        // or asking the outer clearance router to authorize an artificial fixture.
        var addEntry = typeof(PocketContinuationRouter).GetMethod("AddEntry",
            System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic);
        Assert.NotNull(addEntry);
        object?[] arguments = [tail, 0d, RampSettings.FromRatio(4), 23, CancellationToken.None, null];
        var entered = Assert.IsType<TaggedToolpath>(addEntry.Invoke(null, arguments));
        Assert.Equal(RampStrategy.Linear, arguments[^1]);
        Assert.Equal(23, entered.VisitIdentity);
        Assert.Equal(tail.RegionIndex, entered.RegionIndex);
        Assert.Equal(tail.DepthPassIndex, entered.DepthPassIndex);
        var link = entered.Spans.Single(span => span.Kind == ToolpathSpanKind.Link);
        Assert.Equal(new[] { ring[0], destination }, entered.Points.Skip(link.StartIndex).Take(link.PointCount));

        var ramp = entered.Spans.Single(span => span.Kind == ToolpathSpanKind.Ramp);
        var legacy = entered.Points.Take(ramp.EndIndex)
            .Concat(LegacyCut(ring, entered.Points[ramp.EndIndex])).Concat(tail.Points.Skip(5)).ToArray();
        var posted = PostedPoints(entered.Points);
        var oldPosted = PostedPoints(legacy);
        Assert.Equal(oldPosted, posted);
        Assert.Equal(new[] { ring[0], destination }, posted.Skip(link.StartIndex).Take(link.PointCount));

        var unsafeEntry = PlanOptimizedEntry(ring, 0, RampSettings.FromRatio(4));
        var unsafePosted = PostedPoints(unsafeEntry.Points.Concat(tail.Points.Skip(5)).ToArray());
        Assert.Equal(ring[1], unsafePosted[unsafeEntry.Points.Count - 1]);
        // These positive-volume witnesses distinguish the two full-depth transfer
        // chords even though both entries recut every ring edge at the same depth.
        Point3D lostStock = new(2, 2, -.9), addedStock = new(8, 2, -.9);
        foreach (bool ball in new[] { false, true })
        {
            Assert.True(ContainsStock(oldPosted, lostStock, ball), $"Legacy tail misses stock witness {lostStock}; ball={ball}.");
            Assert.True(ContainsStock(posted, lostStock, ball), $"Public pocket entry loses stock witness {lostStock}; ball={ball}.");
            Assert.False(ContainsStock(unsafePosted, lostStock, ball));
            Assert.True(ContainsStock(unsafePosted, addedStock, ball), $"Unsafe splice lacks negative-control witness {addedStock}; ball={ball}.");
            Assert.False(ContainsStock(posted, addedStock, ball));
        }
    }

    [Theory]
    [InlineData(false, -.5)]
    [InlineData(true, -.5)]
    [InlineData(false, -.2)]
    [InlineData(true, -.2)]
    [InlineData(false, 2.5)]
    [InlineData(true, 2.5)]
    [InlineData(false, 10)]
    [InlineData(true, 10)]
    public void ClosedEntryPreservesEveryDirectedFullDepthChordAndEntireCutterSweep(bool vertexContact, double entryZ)
    {
        Point3D[] source = [new(.0031, .0027, -1), new(4.0031, .0027, -1.2),
            new(4.0031, 3.0027, -.7), new(.0031, 3.0027, -1.3), new(.0031, .0027, -1)];
        var messages = new List<string>();
        var entered = PlanOptimizedEntry(source, entryZ, RampSettings.FromRatio(4),
            diagnostic: messages.Add, landAtSourceVertex: vertexContact);
        AssertFullDirectedSourceChords(source, entered);
        var cut = CutPoints(entered);
        var legacy = LegacyCut(source, cut[0]);
        Assert.Equal(legacy.Take(cut.Length), cut);
        Assert.Equal(legacy.Length - cut.Length, entered.RedundantPointsRemoved);
        Assert.Equal(ChordTravel(legacy) - ChordTravel(cut), entered.RedundantCutTravelRemoved, 10);
        Assert.Contains(source, point => point == cut[^1]);
        Assert.Single(messages);
        Assert.Contains("redundantPointsRemoved=", messages[0]);
        Assert.Contains("finalExit=", messages[0]);
        AssertSweptStockEqual(legacy, cut);
        var ramp = entered.Spans[0];
        for (int i = 1; i <= ramp.EndIndex; i++)
        {
            Point3D a = entered.Points[i - 1], b = entered.Points[i];
            double run = Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
            Assert.InRange(a.Z - b.Z, -1e-10, run * .25 + 1e-10);
        }
    }

    [Fact]
    public void ConsecutiveDuplicateSourceVerticesDoNotInflateRemovedPointCount()
    {
        Point3D[] source = [new(0, 0, -1), new(10, 0, -1), new(10, 0, -1),
            new(10, 10, -1), new(0, 10, -1), new(0, 0, -1)];
        var entered = PlanOptimizedEntry(source, 0, RampSettings.FromRatio(4), landAtSourceVertex: true);
        AssertFullDirectedSourceChords(source, entered);
        Assert.Equal(3, entered.RedundantPointsRemoved);
        Assert.Equal(30, entered.RedundantCutTravelRemoved);
    }

    [Fact]
    public void StockOracleDistinguishesSameXyTraceAtDifferentDepths()
    {
        Point3D[] deep = [new(0, 0, -1.2), new(4, 0, -.8)];
        Point3D[] shallow = deep.Select(p => p with { Z = p.Z + .4 }).ToArray();
        Assert.Equal(deep.Select(p => (p.X, p.Y)), shallow.Select(p => (p.X, p.Y)));
        Assert.True(ContainsStock(deep, new(1, 0, -1), ball: false));
        Assert.False(ContainsStock(shallow, new(1, 0, -1), ball: false));
        Assert.True(ContainsStock(deep, new(1, 0, -1), ball: true));
        Assert.False(ContainsStock(shallow, new(1, 0, -1), ball: true));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TinySlopedChordsKeepTheirPostedCutWithoutAWholeCircuitFallback(bool vertexContact)
    {
        Point3D[] source = [new(0, 0, -.003), new(.0008, 0, -.0014),
            new(.004, 0, -.003), new(.004, .004, -.003), new(0, .004, -.003), new(0, 0, -.003)];
        var entered = PlanOptimizedEntry(source, -.0012, RampSettings.FromRatio(4), landAtSourceVertex: vertexContact);
        Assert.False(entered.PartialAxisFallback);
        AssertFullDirectedSourceChords(source, entered);
        var cut = CutPoints(entered);
        AssertSweptStockEqual(PostedPoints(LegacyCut(source,cut[0])),PostedPoints(cut));
        Assert.Contains(PostedPoints(cut),p=>p.X==.001&&p.Z==-.001);
    }

    [Fact]
    public void TinyRampContactAndInteriorRemainderKeepAllCommandedAxes()
    {
        Point3D[] flat = [new(0, 0, -.003), new(.0008, 0, -.003),
            new(.01, 0, -.003), new(.01, .01, -.003), new(0, .01, -.003), new(0, 0, -.003)];
        var ramp = PlanOptimizedEntry(flat, -.001, RampSettings.FromRatio(.5));
        Assert.False(ramp.PartialAxisFallback);
        AssertFullDirectedSourceChords(flat,ramp);
        Point3D[] variable = [new(0, 0, -.01), new(.002, 0, -.001),
            new(.012, 0, -.001), new(.012, .012, -.01), new(0, .012, -.01), new(0, 0, -.01)];
        var entered = PlanOptimizedEntry(variable, -.004, RampSettings.FromRatio(4));
        Assert.False(entered.PartialAxisFallback);
        AssertFullDirectedSourceChords(variable,entered);
        var cut=CutPoints(entered);
        AssertSweptStockEqual(PostedPoints(LegacyCut(variable,cut[0])),PostedPoints(cut));
    }

    [Fact]
    public void SmoothRoutingOmitsRepeatedTailWithoutCommandCompaction()
    {
        Point3D[] ring = [new(0, 0, -1), new(10, 0, -1), new(10, 10, -1), new(0, 10, -1), new(0, 0, -1)];
        var source = new TaggedToolpath(ring, [], 0, "final-carve", null, 17, detectCurves: false);
        var messages = new List<string>();
        var routed = VCarvePathRouter.Route([source], [], ToolGeometry.Flat(1), new DepthSchedule(2, 1),
            new VCarveOptions { SmoothVCarving = true, DetectCurves = false }, 0, null, diagnostic: messages.Add);
        var path = Assert.Single(routed);
        var ramp = path.Spans.Single(s => s.Kind == ToolpathSpanKind.Ramp);
        var cut=path.Points.Skip(ramp.EndIndex).ToArray();
        AssertSweptStockEqual(PostedPoints(LegacyCut(ring,cut[0])),PostedPoints(cut));
        Assert.Equal(ring[1], path.Points[^1]);
        Assert.Contains(messages, m => m.Contains("stage=path-routing")
            && m.Contains("entryExitChanges=1"));
    }

    [Fact]
    public void CoordinatesOutsideFinitePostedWorkspaceRetainLegacyTail()
    {
        const double origin = 1e40, edge = 1e35;
        Point3D[] source = [new(origin, origin, -1), new(origin + edge, origin, -1),
            new(origin + edge, origin + edge, -1), new(origin, origin + edge, -1), new(origin, origin, -1)];
        var entered = PlanOptimizedEntry(source, 0, RampSettings.FromRatio(4), landAtSourceVertex: true);
        Assert.True(entered.PartialAxisFallback);
        Assert.Equal(0, entered.RedundantPointsRemoved);
        Assert.Equal(source[0], entered.Points[^1]);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GroupBoundaryUsesCompletedRingExitAndActualFollowingEntryDecision(bool startsAtLegacyExit)
    {
        Point3D[] ring = [new(0, 0, -1), new(10, 0, -1.2), new(10, 10, -1),
            new(0, 10, -1), new(0, 0, -1)];
        Point3D nextStart = startsAtLegacyExit ? ring[0] : ring[1];
        var sources = new[]
        {
            new TaggedToolpath(ring, [], 0, "final-carve", null, 17, detectCurves: false),
            new TaggedToolpath([nextStart, nextStart with { X = nextStart.X + 5 }], [],
                1, "final-carve", null, 23, detectCurves: false),
        };
        var messages = new List<string>();
        var routed = VCarvePathRouter.Route(sources,
            [new PointD[] { new(-20, -20), new(30, -20), new(30, 30), new(-20, 30) }],
            ToolGeometry.Flat(1), new DepthSchedule(2, 1),
            new VCarveOptions { SmoothVCarving = true, DetectCurves = false, SmoothRampAngleDegrees = 15,
                CompactCollinearCommands = true },
            0, null, diagnostic: messages.Add);
        Assert.Equal(2, routed.Count);
        Assert.Equal(ring[1], routed[0].Points[^1]);
        Assert.Equal(17, routed[0].VisitIdentity);
        Assert.Equal(23, routed[1].VisitIdentity);
        Assert.Equal(1, routed[1].RegionIndex);
        Assert.Equal("final-carve", routed[1].Category);
        Assert.Null(routed[1].DepthPassIndex);
        Assert.Equal(startsAtLegacyExit ? 0 : nextStart.Z, routed[1].Points[0].Z);
        Assert.Equal(startsAtLegacyExit, routed[1].Spans.Any(s => s.Kind == ToolpathSpanKind.Ramp));
        Assert.DoesNotContain(routed, path => path.Spans.Any(s => s.Kind == ToolpathSpanKind.Link));
        Assert.Contains(messages, m => m.Contains("stage=path-routing region=0")
            && m.Contains("entryExitChanges=1") && m.Contains("finalExit=(10,0,-1.2)")
            && m.Contains("groupBoundaryExitGuards=0"));
    }

    [Fact]
    public void ActualExitCoincidenceAvoidsUnnecessarySurfaceEntry()
    {
        Point3D[] ring = [new(0, 0, -1), new(10, 0, -1), new(10, 10, -1), new(0, 10, -1), new(0, 0, -1)];
        var sources = new[]
        {
            new TaggedToolpath(ring, [], 0, "final-carve", null, 17, detectCurves: false),
            new TaggedToolpath([ring[1], new(15, 0, -1)], [], 0, "final-carve", null, 23, detectCurves: false),
        };
        var messages = new List<string>();
        var legacy = RouteSmooth(sources, compact: false);
        var optimized = RouteSmooth(sources, compact: true, messages.Add);
        Assert.Equal(new int?[] { 17, 23 }, optimized.Select(path => path.VisitIdentity));
        Assert.Equal(ring[1], legacy[0].Points[^1]);
        Assert.Equal(ring[1], optimized[0].Points[^1]);
        Assert.Equal(optimized[0].Points,legacy[0].Points);
        Assert.Equal(optimized[0].Points[^1],optimized[1].Points[0]);
        Assert.DoesNotContain(optimized[1].Spans, span => span.Kind == ToolpathSpanKind.Ramp);
        AssertPostedRampsAndCutUnionEqual(legacy, optimized);
        Assert.Contains(messages, message => message.Contains("stage=path-routing")
            && message.Contains("entryExitChanges=1"));
    }

    [Fact]
    public void FormerSeamRequiresEntryWhenCompletedRingEndsElsewhere()
    {
        Point3D[] ring = [new(0, 0, -1), new(10, 0, -1), new(10, 10, -1), new(0, 10, -1), new(0, 0, -1)];
        var sources = new[]
        {
            new TaggedToolpath(ring, [], 0, "final-carve", null, 17, detectCurves: false),
            new TaggedToolpath([ring[0], new(5, 5, -1)], [], 0, "final-carve", null, 23, detectCurves: false),
        };
        var messages = new List<string>();
        var legacy = RouteSmooth(sources, compact: false);
        var optimized = RouteSmooth(sources, compact: true, messages.Add);
        Assert.Equal(new int?[] { 17, 23 }, optimized.Select(path => path.VisitIdentity));
        Assert.Equal(ring[1], optimized[0].Points[^1]);
        Assert.Equal(legacy[0].Points, optimized[0].Points);
        Assert.Equal(0,optimized[1].Points[0].Z);
        Assert.Contains(optimized[1].Spans, span => span.Kind == ToolpathSpanKind.Ramp);
        AssertPostedRampsAndCutUnionEqual(legacy, optimized);
        Assert.Contains(messages, message => message.Contains("stage=path-routing")
            && message.Contains("legacyContinuationExitGuards=0")
            && message.Contains("entryExitChanges=2"));
    }

    [Fact]
    public void NextGroupOrderAndOrientationUseActualCompletedRingExit()
    {
        Point3D[] ring = [new(0, 0, -1), new(10, 0, -1), new(10, 10, -1), new(0, 10, -1), new(0, 0, -1)];
        var sources = new[]
        {
            new TaggedToolpath(ring, [], 0, "final-carve", null, 17, detectCurves: false),
            new TaggedToolpath([new Point3D(0, 0, -.5)], [], 0, "final-carve", null, 20, detectCurves: false),
            new TaggedToolpath([new(1, 0, -1), new(2, 0, -1)], [], 1, "final-carve", null, 23, detectCurves: false),
            new TaggedToolpath([new(9, 0, -1), new(8, 0, -1)], [], 1, "final-carve", null, 29, detectCurves: false),
        };
        var messages = new List<string>();
        var legacy = RouteSmooth(sources, compact: false);
        var optimized = RouteSmooth(sources, compact: true, messages.Add);
        Assert.Equal(new int?[] { 17, 20, 29, 23 }, optimized.Select(path => path.VisitIdentity));
        Assert.Equal(legacy.Select(path => path.VisitIdentity), optimized.Select(path => path.VisitIdentity));
        Assert.Equal(ring[1], optimized[0].Points[^1]);
        Assert.Empty(optimized[1].Points);
        AssertPostedRampsAndCutUnionEqual(legacy, optimized);
        Assert.Equal(new Point3D(2, 0, 0), optimized[3].Points[0]);
        Assert.Contains(messages, message => message.Contains("stage=path-routing region=0")
            && message.Contains("groupBoundaryExitGuards=0"));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LinkableClearingRetainsLegacyExitLinksAndFullPlanStockUnion(bool oldSeamCanLink)
    {
        Point3D[] ring = [new(0, 0, -1), new(10, 0, -1), new(10, 10, -1), new(0, 10, -1), new(0, 0, -1)];
        Point3D[] next = [new(oldSeamCanLink ? .2 : 10.2, 0, -1), new(20, 0, -1)];
        var sources = new[]
        {
            new TaggedToolpath(ring, [], 0, "clearing", 0, 17, detectCurves: false),
            new TaggedToolpath(next, [], 0, "clearing", 0, 23, detectCurves: false),
        };
        var messages = new List<string>();
        var routed = VCarvePathRouter.Route(sources,
            [new PointD[] { new(-5, -5), new(25, -5), new(25, 15), new(-5, 15) }],
            new ToolGeometry(4) { BottomRadius = 0, TopRadius = 4, ConeLength = 8, TipAngleRadians = 2 * Math.Atan(.5) },
            new DepthSchedule(2, 1) { StartDepth = .5 },
            new VCarveOptions { IncludeInteriorFill = true, Ramp = RampSettings.FromRatio(4), DetectCurves = false },
            0, null, diagnostic: messages.Add);
        Assert.Equal(oldSeamCanLink ? 3 : 2, routed.Count);
        Assert.Equal(ring[0], routed[0].Points[^1]);
        var links = routed.Where(path => path.Spans.Any(s => s.Kind == ToolpathSpanKind.Link)).ToArray();
        if (oldSeamCanLink)
        {
            Assert.Equal(new[] { ring[0], next[0] }, Assert.Single(links).Points);
            Assert.Equal(next, routed[^1].Points);
            Assert.DoesNotContain(routed[^1].Spans, span => span.Kind == ToolpathSpanKind.Ramp);
        }
        else
        {
            Assert.Empty(links);
            Assert.Contains(routed[^1].Spans, span => span.Kind == ToolpathSpanKind.Ramp);
        }
        var expected = new List<IReadOnlyList<Point3D>>();
        for (int i = 0; i < routed.Count; i++)
        {
            var path = routed[i];
            if (path.Spans.Any(s => s.Kind == ToolpathSpanKind.Link))
            {
                expected.Add(new[] { ring[0], next[0] });
                continue;
            }
            var ramp = path.Spans.SingleOrDefault(s => s.Kind == ToolpathSpanKind.Ramp);
            if (ramp.Kind != ToolpathSpanKind.Ramp) { expected.Add(next); continue; }
            Point3D contact = path.Points[ramp.EndIndex];
            var legacyCut = i == 0 ? LegacyCut(ring, contact) : new[] { contact, next[0], next[1] };
            expected.Add(path.Points.Take(ramp.EndIndex).Concat(legacyCut).ToArray());
        }
        AssertSweepChordsEqual(expected.SelectMany(Chords).ToArray(), routed.SelectMany(p => Chords(p.Points)).ToArray(), 88, 48);
        Assert.Contains(messages, m => m.Contains("closedTailReductionAllowed=False") && m.Contains("redundantPointsRemoved=0"));
        Assert.Contains(messages, m => m.Contains("stage=path-routing") && m.Contains("entryExitChanges=0")
            && m.Contains("redundantPointsRemoved=0") && m.Contains("redundantCutTravelRemoved=0"));
    }

    [Fact]
    public void EmptyEntryDoesNotInventAnActualExit()
    {
        var single = new TaggedToolpath([new Point3D(0, 0, -1)], [], 0, "final-carve", null, detectCurves: false);
        var following = new TaggedToolpath([new Point3D(0, 0, -1), new(5, 0, -1)], [],
            1, "final-carve", null, 23, detectCurves: false);
        var routed = VCarvePathRouter.Route([single, following], [], ToolGeometry.Flat(1), new DepthSchedule(2, 1),
            new VCarveOptions { SmoothVCarving = true, DetectCurves = false }, 0, new Point3D(20, 0, 0));
        Assert.Equal(2, routed.Count);
        Assert.Empty(routed[0].Points);
        // The original incoming cursor chooses the far endpoint of the following
        // trace and still adds its surface entry. The empty source cannot become
        // a phantom at-depth continuation or a new ordering/orientation origin.
        Assert.Equal(new Point3D(5, 0, 0), routed[1].Points[0]);
        Assert.Contains(routed[1].Spans, span => span.Kind == ToolpathSpanKind.Ramp);
        Assert.Equal(23, routed[1].VisitIdentity);
    }

    private static RampedPath PlanOptimizedEntry(IReadOnlyList<Point3D> path, double entryZ,
        RampSettings settings, Action<string>? diagnostic = null, bool landAtSourceVertex = false)
        => RampPlanner.PlanPathEntry(path, entryZ, settings, CancellationToken.None,
            diagnostic, landAtSourceVertex, preserveClosedExit: false);

    private static List<TaggedToolpath> RouteSmooth(IReadOnlyList<TaggedToolpath> source, bool compact,
        Action<string>? diagnostic = null, Point3D? start = null)
        => VCarvePathRouter.Route(source, [], ToolGeometry.Flat(1), new DepthSchedule(2, 1),
            new VCarveOptions { SmoothVCarving = true, DetectCurves = false,
                SmoothRampAngleDegrees = 15, CompactCollinearCommands = compact },
            0, start ?? new Point3D(0, 0, 0), diagnostic: diagnostic);

    private static void AssertPostedRampsAndCutUnionEqual(IReadOnlyList<TaggedToolpath> legacy,
        IReadOnlyList<TaggedToolpath> optimized)
    {
        Assert.Equal(legacy.Count, optimized.Count);
        var oldCuts = new HashSet<(Point3D A, Point3D B)>();
        var newCuts = new HashSet<(Point3D A, Point3D B)>();
        var oldPoses = new HashSet<Point3D>();
        var newPoses = new HashSet<Point3D>();
        for (int visit = 0; visit < legacy.Count; visit++)
        {
            Assert.Equal(legacy[visit].VisitIdentity, optimized[visit].VisitIdentity);
            var oldRamps = legacy[visit].Spans.Where(span => span.Kind == ToolpathSpanKind.Ramp).ToArray();
            var newRamps = optimized[visit].Spans.Where(span => span.Kind == ToolpathSpanKind.Ramp).ToArray();
            Assert.Equal(oldRamps.Length, newRamps.Length);
            for (int ramp = 0; ramp < oldRamps.Length; ramp++)
            {
                var oldPoints = legacy[visit].Points.Skip(oldRamps[ramp].StartIndex).Take(oldRamps[ramp].PointCount).ToArray();
                var newPoints = optimized[visit].Points.Skip(newRamps[ramp].StartIndex).Take(newRamps[ramp].PointCount).ToArray();
                Assert.Equal(oldPoints, newPoints);
                Assert.Equal(PostedPoints(oldPoints), PostedPoints(newPoints));
            }
            CollectCuts(legacy[visit], oldCuts, oldPoses);
            CollectCuts(optimized[visit], newCuts, newPoses);
        }
        // Exact posted chord and stationary-pose union equality proves the cut
        // sweep for any cutter shape. Ramp equality is checked independently;
        // no tolerance or surface-only footprint stands in for either contract.
        Assert.True(oldCuts.SetEquals(newCuts),
            $"Posted directed cut union changed; missing={string.Join(";", oldCuts.Except(newCuts).Take(4))}; added={string.Join(";", newCuts.Except(oldCuts).Take(4))}.");
        Assert.True(oldPoses.SetEquals(newPoses), "Posted cutter-pose union changed.");

        static void CollectCuts(TaggedToolpath path, HashSet<(Point3D A, Point3D B)> chords,
            HashSet<Point3D> poses)
        {
            if (path.Points.Count == 0) return;
            var cuts = path.Spans.Count == 0
                ? new[] { new ToolpathSpan(0, path.Points.Count - 1, ToolpathSpanKind.Cut) }
                : path.Spans.Where(span => span.Kind == ToolpathSpanKind.Cut);
            foreach (var span in cuts)
            {
                var posted = PostedPoints(path.Points.Skip(span.StartIndex).Take(span.PointCount).ToArray());
                foreach (var point in posted) poses.Add(point);
                foreach (var chord in Chords(posted).Where(chord => chord.A != chord.B)) chords.Add(chord);
            }
        }
    }

    internal static void AssertFullDirectedSourceChords(IReadOnlyList<Point3D> source, RampedPath entered)
    {
        var sourceChords = Chords(source).Where(c => c.A != c.B).ToArray();
        var cutChords = Chords(CutPoints(entered)).Where(c => c.A != c.B).ToArray();
        // Exact unbroken XYZ endpoint equality proves the original variable-depth
        // chord survives quantization as well as any cutter profile swept along it.
        foreach (var chord in sourceChords.Distinct())
            Assert.Equal(sourceChords.Count(c => c == chord), cutChords.Count(c => c == chord));
        var extra = cutChords.Where(c => !sourceChords.Contains(c)).ToArray();
        if (extra.Length == 0) return;
        var partial = Assert.Single(extra);
        Assert.Equal(cutChords[0], partial);
        Assert.Contains(sourceChords, chord => partial.B == chord.B
            && OnDirectedChord(partial.A, chord.A, chord.B));
    }

    private static Point3D[] CutPoints(RampedPath entered)
    {
        var span = Assert.Single(entered.Spans.Where(s => s.Kind == ToolpathSpanKind.Cut));
        return entered.Points.Skip(span.StartIndex).Take(span.EndIndex - span.StartIndex + 1).ToArray();
    }

    private static IEnumerable<(Point3D A, Point3D B)> Chords(IReadOnlyList<Point3D> points)
    {
        for (int i = 1; i < points.Count; i++) yield return (points[i - 1], points[i]);
    }

    private static bool IsPartialAxis(Point3D a, Point3D b)
    {
        float dx = (float)b.X - (float)a.X, dy = (float)b.Y - (float)a.Y;
        return (float)b.Z != (float)a.Z && dx == 0 && dy == 0;
    }

    private static double PostedCoordinate(double native)
        => double.Parse(((float)native).ToString("F3", CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);

    private static Point3D[] PostedPoints(IReadOnlyList<Point3D> native)
    {
        // Independent actual-axis cursor: vertical-only commands change depth
        // while preserving both previous planar axes, even across F3 grid cells.
        var points = new List<Point3D> { new(PostedCoordinate(native[0].X),
            PostedCoordinate(native[0].Y), PostedCoordinate(native[0].Z)) };
        for (int i = 1; i < native.Count; i++)
        {
            Point3D previous = points[^1], next = native[i];
            points.Add(IsPartialAxis(native[i - 1], next)
                ? previous with { Z = PostedCoordinate(next.Z) }
                : new(PostedCoordinate(next.X), PostedCoordinate(next.Y), PostedCoordinate(next.Z)));
        }
        return points.ToArray();
    }

    private static bool OnDirectedChord(Point3D p, Point3D a, Point3D b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
        double t = ((p.X - a.X) * dx + (p.Y - a.Y) * dy + (p.Z - a.Z) * dz) / (dx * dx + dy * dy + dz * dz);
        return t >= -1e-12 && t <= 1 + 1e-12
            && Math.Abs(p.X - a.X - t * dx) < 1e-12
            && Math.Abs(p.Y - a.Y - t * dy) < 1e-12
            && Math.Abs(p.Z - a.Z - t * dz) < 1e-12;
    }

    private static Point3D[] LegacyCut(IReadOnlyList<Point3D> source, Point3D contact)
    {
        int vertex = -1;
        for (int i = 1; i < source.Count; i++) if (source[i] == contact) vertex = i;
        int suffixStart = vertex >= 0 ? vertex + 1 : Enumerable.Range(1, source.Count - 1)
            .Where(i => OnDirectedChord(contact, source[i - 1], source[i]))
            // At a rounded almost-vertex contact, tolerance may also admit the
            // next chord's start. The arrived-at chord has the larger parameter.
            .OrderByDescending(i => ChordParameter(contact, source[i - 1], source[i])).First();
        var cut = new List<Point3D> { contact };
        foreach (var p in source.Skip(suffixStart).Concat(source))
            if (cut[^1] != p) cut.Add(p);
        return cut.ToArray();
    }

    private static double ChordTravel(IReadOnlyList<Point3D> points) => Chords(points).Sum(c =>
        Math.Sqrt(Math.Pow(c.B.X - c.A.X, 2) + Math.Pow(c.B.Y - c.A.Y, 2) + Math.Pow(c.B.Z - c.A.Z, 2)));

    private static double ChordParameter(Point3D p, Point3D a, Point3D b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
        return ((p.X - a.X) * dx + (p.Y - a.Y) * dy + (p.Z - a.Z) * dz) / (dx * dx + dy * dy + dz * dz);
    }

    private static void AssertSweptStockEqual(IReadOnlyList<Point3D> legacy, IReadOnlyList<Point3D> cut)
        => AssertSweepChordsEqual(Chords(legacy).ToArray(), Chords(cut).ToArray());

    private static void AssertSweepChordsEqual(IReadOnlyList<(Point3D A, Point3D B)> legacy,
        IReadOnlyList<(Point3D A, Point3D B)> cut, int maxX = 18, int maxY = 14)
    {
        // Independent analytic cutter-volume membership, across actual stock
        // heights, rather than a surface-footprint or emitted-byte comparison.
        foreach (bool ball in new[] { false, true })
        for (int z = 0; z <= 12; z++)
        for (int y = -2; y <= maxY; y++)
        for (int x = -2; x <= maxX; x++)
        {
            var voxel = new Point3D(x / 4d, y / 4d, -z / 8d);
            Assert.Equal(ContainsStock(legacy, voxel, ball), ContainsStock(cut, voxel, ball));
        }
    }

    private static bool ContainsStock(IReadOnlyList<Point3D> points, Point3D q, bool ball)
        => ContainsStock(Chords(points), q, ball);

    private static bool ContainsStock(IEnumerable<(Point3D A, Point3D B)> chords, Point3D q, bool ball)
    {
        foreach (var (a, b) in chords)
        {
            double dx = b.X - a.X, dy = b.Y - a.Y, dz = b.Z - a.Z;
            if (ball)
            {
                const double radius = .4;
                double vx = q.X - a.X, vy = q.Y - a.Y, vz = q.Z - (a.Z + radius);
                double norm = dx * dx + dy * dy + dz * dz;
                double t = norm == 0 ? 0 : Math.Clamp((vx * dx + vy * dy + vz * dz) / norm, 0, 1);
                if (Math.Pow(vx - t * dx, 2) + Math.Pow(vy - t * dy, 2)
                    + Math.Pow(vz - t * dz, 2) <= radius * radius) return true;
            }
            else
            {
                // Finite pointed cone: radius = .5 * height above tip, 0 <= height <= 3.
                double lo = 0, hi = 1, height = q.Z - a.Z;
                if (dz == 0) { if (height < 0 || height > 3) continue; }
                else
                {
                    double t0 = height / dz, t3 = (height - 3) / dz;
                    lo = Math.Max(lo, Math.Min(t0, t3)); hi = Math.Min(hi, Math.Max(t0, t3));
                    if (lo > hi) continue;
                }
                double vx = q.X - a.X, vy = q.Y - a.Y;
                double aa = dx * dx + dy * dy - .25 * dz * dz;
                double bb = -2 * (vx * dx + vy * dy) + .5 * height * dz;
                double cc = vx * vx + vy * vy - .25 * height * height;
                double Value(double t) => aa * t * t + bb * t + cc;
                double minimum = Math.Min(Value(lo), Value(hi));
                if (aa > 0) minimum = Math.Min(minimum, Value(Math.Clamp(-bb / (2 * aa), lo, hi)));
                if (minimum <= 0) return true;
            }
        }
        return false;
    }
}
