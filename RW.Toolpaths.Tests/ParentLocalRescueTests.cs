using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Parent local rescue controls", DisableParallelization = true)]
public sealed class ParentLocalRescueCollection;

[Collection("Parent local rescue controls")]
public sealed class ParentLocalRescueTests
{
    [Fact]
    public void CompleteOriginalShellProofRetainsRequestedClosedDistanceAndInputs()
    {
        var input = Fixture();
        string before = Commands(input.Requested.Concat(input.Shell).Concat(input.Covered));
        var result = Try(input);
        Assert.True(result.Accepted, result.Reason);
        Assert.Equal(16, result.RequestedDistance);
        Assert.Equal(1, result.Owners);
        Assert.Equal(1, result.Attempts);
        Assert.True(result.GapBefore > 0);
        Assert.Equal(0, result.GapAfter);
        Assert.NotEmpty(result.Traces);
        Assert.Equal(before, Commands(input.Requested.Concat(input.Shell).Concat(input.Covered)));
        var actual = PolygonOps.Union(input.Covered.Concat(
            PolygonOps.BufferOpenCenterlines(result.Traces, input.Radius, .001)));
        Assert.Equal(0, Gap(input.Shell, actual));
    }

    [Theory]
    [InlineData(false, true)]
    [InlineData(true, false)]
    public void CompleteSignedHolesAndIndependentOwnersRemainInWholeProof(bool two, bool hole)
    {
        var input = Fixture(two, hole);
        var result = Try(input);
        Assert.True(result.Accepted, result.Reason);
        Assert.Equal(two ? 2 : 1, result.Owners);
        Assert.Equal(0, result.GapAfter);
        if (hole)
            Assert.All(result.Traces.SelectMany(p => p), p => Assert.True(p.x <= 40 || p.x >= 60 || p.y <= 40 || p.y >= 60));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NegativeCoordinatesAndObliqueFiniteEdgesUseSameAuthority(bool rotate)
    {
        PointD Map(PointD p) => rotate ? new(-p.y, p.x) : new(p.x - 400, p.y - 550);
        var input = Fixture(map: Map);
        var result = Try(input);
        Assert.True(result.Accepted, result.Reason);
        Assert.Equal(0, result.GapAfter);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TinySignedSourceHoleOrThinSourceFeatureCannotReceiveRescueCredit(bool thin)
    {
        var initial = Fixture();
        var accepted = Try(initial);
        Assert.True(accepted.Accepted, accepted.Reason);
        var path = accepted.Traces[0];
        var point = path[path.Count / 2];
        double half = 4 / PathUtils.Scale;
        var hole = Rectangle(point.x - half, point.y - (thin ? 1 : half),
            point.x + half, point.y + (thin ? 1 : half), hole: true);
        var source = new List<List<PointD>> { Rectangle(0, 0, 100, 100), hole };
        var result = Try(initial with { Source = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(source)) });
        Assert.False(result.Accepted);
        Assert.Empty(result.Traces);
    }

    [Theory]
    [InlineData("owners")]
    [InlineData("parents")]
    [InlineData("core")]
    [InlineData("trace")]
    [InlineData("attempts")]
    public void EveryBoundedWorkRefusalPublishesNoPaths(string budget)
    {
        var input = Fixture(two: true);
        var limits = budget switch {
            "owners" => new ParentLocalRescue.Limits(Owners: 1),
            "parents" => new ParentLocalRescue.Limits(ParentPoints: 1),
            "core" => new ParentLocalRescue.Limits(CorePoints: 1),
            "trace" => new ParentLocalRescue.Limits(TracePoints: 1),
            _ => new ParentLocalRescue.Limits(Attempts: 0),
        };
        var result = Try(input, limits);
        Assert.False(result.Accepted);
        Assert.Empty(result.Traces);
    }

    [Theory]
    [InlineData("parent-count")]
    [InlineData("parent-input")]
    [InlineData("owner-checks")]
    [InlineData("input-points")]
    [InlineData("input-contours")]
    public void CompleteInputAndAllOwnerWorkAreBoundedBeforeProposalAllocation(string budget)
    {
        var input = Fixture(two: true);
        var limits = budget switch {
            "parent-count" => new ParentLocalRescue.Limits(ParentComponents: 1),
            "parent-input" => new ParentLocalRescue.Limits(ParentInputPoints: 1),
            "owner-checks" => new ParentLocalRescue.Limits(OwnerChecks: 1),
            "input-points" => new ParentLocalRescue.Limits(InputPoints: 1),
            _ => new ParentLocalRescue.Limits(InputContours: 1),
        };
        int audits = 0;
        var result = ParentLocalRescue.Try(input, CancellationToken.None, (_, _) => { audits++; return 0; }, limits);
        Assert.False(result.Accepted);
        Assert.Empty(result.Traces);
        Assert.Equal(0, result.Attempts);
        Assert.Equal(0, audits);
    }

    [Fact]
    public void RefusedOpenBooleanDiscardsEvenApparentlyUsablePartialPaths()
    {
        var input = Fixture();
        var genuine = Try(input);
        Assert.True(genuine.Accepted, genuine.Reason);
        int calls = 0, audits = 0;
        bool Refused(Clipper64 _, Paths64 closed, Paths64 open)
        {
            calls++;
            open.AddRange(PathUtils.ToClipper(genuine.Traces));
            return false;
        }
        Assert.False(ParentLocalRescue.TryClipOpen(genuine.Traces, new[] { Rectangle(0, 0, 100, 100) },
            CancellationToken.None, out var partial, Refused));
        Assert.Empty(partial);
        var result = ParentLocalRescue.Try(input, CancellationToken.None, (_, _) => { audits++; return 0; }, executeClip: Refused);
        Assert.Equal("clip-unresolved", result.Reason);
        Assert.False(result.Accepted);
        Assert.Empty(result.Traces);
        Assert.Equal(2, calls);
        Assert.Equal(0, audits);
    }

    [Theory]
    [InlineData(ClipType.Intersection, "pocket-final-open-commands")]
    [InlineData(ClipType.Difference, "accepted-emitted-history")]
    public void FinalCommandAndHistoryClipBoundariesAbortRatherThanPublishPartialPaths(ClipType operation, string owner)
    {
        var paths = new List<List<PointD>> { new() { new(10, 10), new(40, 30) } };
        bool Partial(Clipper64 _, Paths64 closed, Paths64 open)
        {
            open.AddRange(PathUtils.ToClipper(paths)); return false;
        }
        var error = Assert.Throws<InvalidOperationException>(() => CheckedOpenClip.Require(paths,
            new[] { Rectangle(0, 0, 100, 100) }, operation, CancellationToken.None, owner, Partial));
        Assert.Contains(owner, error.Message);
        Assert.Contains("partial paths cannot be emitted or credited", error.Message);
        var complete = CheckedOpenClip.Require(paths, new[] { Rectangle(0, 0, 100, 100) }, operation,
            CancellationToken.None, owner);
        if (operation == ClipType.Intersection)
        {
            Assert.Single(complete);
            Assert.Equal(paths[0].Select(p => (p.x, p.y)).OrderBy(p => p.x),
                complete[0].Select(p => (p.x, p.y)).OrderBy(p => p.x));
        }
        else Assert.Empty(complete);
    }

    [Fact]
    public void MissingOrAmbiguousFullCoreOwnershipIsRefusal()
    {
        var input = Fixture();
        var remote = new PolygonComponent { Outer = Rectangle(1000, 1000, 1010, 1010) };
        var missing = Try(input with { Parents = new[] { remote } });
        Assert.Equal("missing-owner", missing.Reason);
        Assert.Empty(missing.Traces);
        var duplicated = Try(input with { Parents = new[] { input.Parents[0], input.Parents[0] } });
        Assert.Equal("ambiguous-owner", duplicated.Reason);
        Assert.Empty(duplicated.Traces);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(100_000_000)]
    public void NonfiniteAndNativeRangeInputsFallBackWithoutGeometry(double radius)
    {
        var result = Try(Fixture() with { Radius = radius });
        Assert.Equal("invalid-input", result.Reason);
        Assert.Empty(result.Traces);
    }

    [Fact]
    public void CompleteOriginalWallRejectsExteriorRequestedCommands()
    {
        var input = Fixture();
        var result = Try(input with { Requested = new() { Rectangle(1000, 1000, 1010, 1010) } });
        Assert.Equal("requested-original-wall", result.Reason);
        Assert.Empty(result.Traces);
    }

    [Fact]
    public void PositiveWholeGapRefusesEvenSourceSupportedOpenTrajectories()
    {
        var input = Fixture();
        int audits = 0;
        var result = ParentLocalRescue.Try(input, CancellationToken.None, (_, _) => { audits++; return 1; });
        Assert.True(audits > 0);
        Assert.Equal("bounded-proof-refusal", result.Reason);
        Assert.True(result.GapAfter > 0);
        Assert.Empty(result.Traces);
    }

    [Fact]
    public void PhysicalClipProducesOnlyActualOpenSegmentsAndWholeProofRefusesMissingStock()
    {
        var input = Fixture();
        var result = Try(input with { Clip = Rectangle(0, 0, 20, 100) });
        Assert.False(result.Accepted);
        Assert.Empty(result.Traces);
        Assert.True(result.GapAfter > 0);
    }

    [Fact]
    public void CancellationBeforeOrDuringWholeProofCannotPublishPaths()
    {
        var input = Fixture();
        using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => ParentLocalRescue.Try(input, cancelled.Token, Gap));
        using var during = new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(() => ParentLocalRescue.Try(input, during.Token, (_, _) => {
            during.Cancel(); return 0;
        }));
    }

    [Fact]
    public void CapturedPolicyIsPocketOnlyAndStableAfterEnvironmentChanges()
    {
        using var environment = new EnvironmentScope();
        Assert.False(new RingOffsetOptions(3, 6).ParentLocalRescueProposals);
        var pocket = PocketToolpaths.CaptureParentLocalRescueMode(new PocketOptions());
        var ring = RingOffsetEngine.CaptureParentLocalRescueMode(new RingOffsetOptions(3, 6) {
            ParentLocalRescueProposals = true });
        Environment.SetEnvironmentVariable(Reference, "1");
        Assert.True(pocket.ParentLocalRescueProposals);
        Assert.True(RingOffsetEngine.CaptureParentLocalRescueMode(ring).ParentLocalRescueProposals);
        Assert.False(PocketToolpaths.CaptureParentLocalRescueMode(new PocketOptions()).ParentLocalRescueProposals);
        Assert.False(RingOffsetEngine.CaptureParentLocalRescueMode(new RingOffsetOptions(3, 6) {
            ParentLocalRescueProposals = true }).ParentLocalRescueProposals);
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(4, false)]
    [InlineData(8192, false)]
    [InlineData(199999, false)]
    [InlineData(200000, true)]
    [InlineData(291380, true)]
    public void DenseWorkAdmissionUsesOriginalSourceSitesAndKeepsBothLaterLevels(int originalSites, bool admitted)
    {
        using var environment = new EnvironmentScope();
        var policy = RingOffsetEngine.CaptureParentLocalRescueMode(Options());
        var first = RingOffsetEngine.AdmitParentLocalRescueSource(policy, originalSites, 114);
        // BG114's later accepted parent shrinks from 25,875 to 2,106 points.
        // Neither size enters the admission: only the same original site count.
        var later = RingOffsetEngine.AdmitParentLocalRescueSource(policy, originalSites, 114);
        Assert.Equal(admitted, first.ParentLocalRescueProposals);
        Assert.Equal(admitted, later.ParentLocalRescueProposals);
        Assert.True(policy.ParentLocalRescueProposals);
        Assert.True(first.ParentLocalRescueModeCaptured);
        Assert.False(RingOffsetEngine.AdmitParentLocalRescueSource(new RingOffsetOptions(3, 6),
            originalSites, 114).ParentLocalRescueProposals);
    }

    [Theory]
    [InlineData(Reference)]
    [InlineData("RW_TOOLPATHS_COHERENCE_REFERENCE")]
    public void OriginalSourceAdmissionHonorsCapturedReferencePolicy(string reference)
    {
        using var environment = new EnvironmentScope();
        var captured = RingOffsetEngine.CaptureParentLocalRescueMode(Options());
        Environment.SetEnvironmentVariable(reference, "1");
        Assert.True(RingOffsetEngine.AdmitParentLocalRescueSource(captured, 291380, 114).ParentLocalRescueProposals);
        var (uncaptured, log) = Capture(() => RingOffsetEngine.AdmitParentLocalRescueSource(Options(), 291380, 114));
        Assert.False(((RingOffsetOptions)uncaptured).ParentLocalRescueProposals);
        Assert.Contains("densityEligible=True capturedFlag=False policyCaptured=True capturedReferenceOrDisabled=True mode=original-search", log);
    }

    [Theory]
    [InlineData(false, 18)]
    [InlineData(true, 18)]
    [InlineData(false, 18.002)]
    [InlineData(true, 18.002)]
    [InlineData(false, 60)]
    [InlineData(true, 60)]
    public void DefaultPocketSimpleSourcesKeepExactOriginalSearchCommands(bool insideOut, double width)
    {
        using var environment = new EnvironmentScope();
        var region = new MillingRegion(Rectangle(0, 0, width, 90));
        var options = new PocketOptions { StepOver = 1, InsideOut = insideOut, SpiralChaining = true,
            Ramp = RampSettings.FromRatio(4), DetectCurves = false, CalculateSweptArea = false,
            Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = 0 } };
        var (candidateObject, candidateLog) = Capture(() => PocketToolpaths.Generate(new[] { region },
            ToolGeometry.Flat(3), new DepthSchedule(2, 1), options));
        Environment.SetEnvironmentVariable(Reference, "1");
        var (referenceObject, _) = Capture(() => PocketToolpaths.Generate(new[] { region },
            ToolGeometry.Flat(3), new DepthSchedule(2, 1), options));
        var candidate = (ToolpathPlan)candidateObject;
        var referencePlan = (ToolpathPlan)referenceObject;
        string PostedInputs(ToolpathPlan plan) => string.Join("|", plan.Toolpaths.Select(path =>
            string.Join(";", path.Points.Select(p => $"{p.X:R},{p.Y:R},{p.Z:R}")) + ":"
            + string.Join(";", path.Spans.Select(span => $"{span.Kind},{span.StartIndex},{span.EndIndex}"))));
        Assert.Equal(PostedInputs(referencePlan), PostedInputs(candidate));
        Assert.Equal(referencePlan.Diagnostics.RingCount, candidate.Diagnostics.RingCount);
        Assert.Equal(referencePlan.Diagnostics.StripCount, candidate.Diagnostics.StripCount);
        Assert.Equal(referencePlan.Diagnostics.LinearRampEntries, candidate.Diagnostics.LinearRampEntries);
        Assert.Contains("sourceSegments=4 threshold=200000 densityEligible=False capturedFlag=True", candidateLog);
        Assert.Contains("mode=original-search qualification=complete-original-finite-source", candidateLog);
        Assert.DoesNotContain("[RingOffsetEngine.ParentLocalRescue] event=accept", candidateLog);
    }

    [Theory]
    [InlineData("RW_TOOLPATHS_EXACT_FIRST_COVERAGE_REFERENCE")]
    [InlineData("RW_TOOLPATHS_COHERENCE_REFERENCE")]
    [InlineData("RW_TOOLPATHS_LOCAL_MEDIAL_REFERENCE")]
    public void CompleteReachabilityPolicyIsCapturedBeforeGeometryAndOtherFillsKeepFalse(string variable)
    {
        string? previous = Environment.GetEnvironmentVariable(variable);
        try
        {
            Environment.SetEnvironmentVariable(variable, null);
            Assert.False(new RingOffsetOptions(3, 6).ExactFirstCoverageProposals);
            var pocket = PocketToolpaths.CaptureExactFirstCoverageMode(new PocketOptions());
            var ring = RingOffsetEngine.CaptureExactFirstCoverageMode(new RingOffsetOptions(3, 6) {
                ExactFirstCoverageProposals = true });
            Assert.True(pocket.ExactFirstCoverageProposals);
            Assert.True(ring.ExactFirstCoverageProposals);
            Environment.SetEnvironmentVariable(variable, "1");
            Assert.True(pocket.ExactFirstCoverageProposals);
            Assert.True(RingOffsetEngine.CaptureExactFirstCoverageMode(ring).ExactFirstCoverageProposals);
            Assert.False(PocketToolpaths.CaptureExactFirstCoverageMode(new PocketOptions()).ExactFirstCoverageProposals);
            Assert.False(RingOffsetEngine.CaptureExactFirstCoverageMode(new RingOffsetOptions(3, 6) {
                ExactFirstCoverageProposals = true }).ExactFirstCoverageProposals);
        }
        finally { Environment.SetEnvironmentVariable(variable, previous); }
    }

    [Fact]
    public void RequestedDistanceRescueIsPublishedOnlyWithOriginalClosedSweepOwnership()
    {
        using var environment = new EnvironmentScope();
        var (accepted, log) = Capture(() => Search(Options(), (_, raw) => Clone(raw)));
        Assert.Equal(16d, accepted.GetType().GetProperty("Distance")!.GetValue(accepted));
        Assert.Equal(0d, accepted.GetType().GetProperty("ShellGapArea")!.GetValue(accepted));
        var terminals = (IReadOnlyList<IReadOnlyList<PointD>>)accepted.GetType().GetProperty("TerminalPaths")!.GetValue(accepted)!;
        Assert.NotEmpty(terminals);
        Assert.Contains("[RingOffsetEngine.ParentLocalRescue] event=accept", log);
        Assert.Contains("originalRequestedSweep=True finalCommandFrontProof=True fullSignedShellProof=True", log);
        var closed = (List<List<PointD>>)accepted.GetType().GetProperty("Polygons")!.GetValue(accepted)!;
        var owner = (PreparedClosedSweep)accepted.GetType().GetProperty("PreparedClosedSweep", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(accepted)!;
        Assert.True(owner.TryReuse(closed, 3, .001, true, null, CancellationToken.None, out var actual));
        Assert.True(PreparedClosedSweep.SameIntegerSweep(actual!, RingOffsetEngine.BufferClosedSweeps(closed, 3, .001,
            CancellationToken.None, true), CancellationToken.None));
        Assert.False(owner.TryReuse(terminals.Select(p => p.ToList()).ToList(), 3, .001, true, null,
            CancellationToken.None, out _));
    }

    [Fact]
    public void RefusedRequestedCleanupReturnsToOriginalSearchAndNeverRecursesIntoRescue()
    {
        using var environment = new EnvironmentScope();
        var missing = new List<List<PointD>> { Rectangle(49, 49, 51, 51) };
        var (result, log) = Capture(() => Search(Options(), (_, _) => Clone(missing)));
        Assert.Contains("[RingOffsetEngine.ParentLocalRescue] event=fallback", log);
        Assert.Contains("fallback=original-halving-and-eight-refinements", log);
        Assert.Contains("reason=final-command-front-unproved", log);
        Assert.Equal(1, Count(log, "[RingOffsetEngine.ParentLocalRescue] event=fallback"));
        var (reference, _) = Capture(() => Search(Options() with { ParentLocalRescueProposals = false }, null));
        Assert.Equal(reference.GetType().GetProperty("Distance")!.GetValue(reference), result.GetType().GetProperty("Distance")!.GetValue(result));
        Assert.Equal(Commands((List<List<PointD>>)reference.GetType().GetProperty("Polygons")!.GetValue(reference)!),
            Commands((List<List<PointD>>)result.GetType().GetProperty("Polygons")!.GetValue(result)!));
    }

    [Fact]
    public void ExistingCollapsedCallbacksRunBeforeRescueAndRetainTheirCurrentOwnerIdentity()
    {
        using var environment = new EnvironmentScope();
        var owners = new List<IReadOnlyList<PointD>>();
        var current = Raw(10); current.Add(Rectangle(120, 10, 122, 12));
        var source = new MillingRegion(Rectangle(0, 0, 200, 100));
        var options = Options() with { TerminalPaths = (context, _) => {
            owners.Add(context.CurrentComponent.Outer); return Array.Empty<IReadOnlyList<PointD>>();
        } };
        var (accepted, _) = Capture(() => Search(options, (_, raw) => Clone(raw), source, current));
        Assert.True(owners.Count >= 2);
        Assert.All(owners, owner => Assert.Same(owners[0], owner));
        Assert.Equal(0d, accepted.GetType().GetProperty("ShellGapArea")!.GetValue(accepted));
    }

    private const string Reference = "RW_TOOLPATHS_PARENT_LOCAL_RESCUE_REFERENCE";
    private static ParentLocalRescue.Result Try(ParentLocalRescue.Input input, ParentLocalRescue.Limits? limits = null)
        => ParentLocalRescue.Try(input, CancellationToken.None, Gap, limits);
    private static double Gap(IEnumerable<IReadOnlyList<PointD>> shell, IEnumerable<IReadOnlyList<PointD>> covered)
        => ParentLocalRescue.Area(PolygonOps.Offset(PolygonOps.Difference(shell, covered), -ParentLocalRescue.Error, .00025, 0));
    private static ParentLocalRescue.Input Fixture(bool two = false, bool hole = false, Func<PointD, PointD>? map = null)
    {
        var source = new List<List<PointD>> { Rectangle(0, 0, 100, 100) };
        var current = Raw(10); var next = Raw(16);
        if (two) { source.Add(Rectangle(120, 0, 220, 100)); current.Add(Rectangle(130, 10, 210, 90)); next.Add(Rectangle(136, 16, 204, 84)); }
        if (hole) { source.Add(Rectangle(40, 40, 60, 60, true)); current.Add(Rectangle(30, 30, 70, 70, true)); next.Add(Rectangle(24, 24, 76, 76, true)); }
        if (map is not null) { source = source.Select(p => p.Select(map).ToList()).ToList(); current = current.Select(p => p.Select(map).ToList()).ToList(); next = next.Select(p => p.Select(map).ToList()).ToList(); }
        var covered = PolygonOps.Union(RingOffsetEngine.BufferClosedSweeps(current, 3, .001, CancellationToken.None, true)
            .Concat(RingOffsetEngine.BufferClosedSweeps(next, 3, .001, CancellationToken.None, true)));
        var shell = PolygonOps.Difference(current, next);
        var core = PolygonOps.Offset(PolygonOps.Difference(shell, covered), -ParentLocalRescue.Error, .00025, 0);
        return new(new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(source)),
            PolygonOps.DifferenceToComponents(current, Array.Empty<IReadOnlyList<PointD>>()), next, shell, covered, core, null, 10, 16, 3);
    }
    private static RingOffsetOptions Options() => new(10, 6) { PhysicalSweepRadius = 3, MinimumRingArea = 0,
        RetainEmittedSweepHistory = true, SelectedCleanupProposals = true, ParentLocalRescueProposals = true };
    private static object Search(RingOffsetOptions options, Func<double, List<List<PointD>>, List<List<PointD>>>? compact,
        MillingRegion? source = null, List<List<PointD>>? current = null)
    {
        var method = typeof(RingOffsetEngine).GetMethod("SelectCoherentFront", BindingFlags.Static | BindingFlags.NonPublic)!;
        return method.Invoke(null, new object?[] { source ?? new MillingRegion(Rectangle(0, 0, 100, 100)), 0, 0,
            current ?? Raw(10), 10d, 16d, Raw(16), options, (Func<double, List<List<PointD>>>)Raw, 0d,
            CancellationToken.None, null, null, compact is null ? null : (Func<double, List<List<PointD>>>)Raw, compact, null })!;
    }
    private static List<List<PointD>> Raw(double distance) => new() { Rectangle(distance, distance, 100 - distance, 100 - distance) };
    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1, bool hole = false)
    { var path = new List<PointD> { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) }; if (hole) path.Reverse(); return path; }
    private static List<List<PointD>> Clone(List<List<PointD>> paths) => paths.Select(p => p.ToList()).ToList();
    private static string Commands(IEnumerable<IReadOnlyList<PointD>> paths) => string.Join("|", paths.Select(p => string.Join(";", p.Select(v => $"{v.x:R},{v.y:R}"))));
    private static int Count(string value, string fragment) => (value.Length - value.Replace(fragment, "").Length) / fragment.Length;
    private static (object Result, string Log) Capture(Func<object> action)
    {
        using var logSelection = RW.Diagnostics.DiagnosticLog.PushNamespaces("*");
        var previous = Console.Out; using var log = new StringWriter(); Console.SetOut(log);
        try { return (action(), log.ToString()); } finally { Console.SetOut(previous); }
    }
    private sealed class EnvironmentScope : IDisposable
    {
        private readonly string? _reference = Environment.GetEnvironmentVariable(Reference);
        private readonly string? _coherence = Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE");
        internal EnvironmentScope() { Environment.SetEnvironmentVariable(Reference, null); Environment.SetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE", null); }
        public void Dispose() { Environment.SetEnvironmentVariable(Reference, _reference); Environment.SetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE", _coherence); }
    }
}
