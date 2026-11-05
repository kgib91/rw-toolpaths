using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Pocket construction policy controls", DisableParallelization = true)]
public sealed class PocketConstructionPolicyCollection;

[Collection("Pocket construction policy controls")]
public sealed class PocketConstructionPolicyTests
{
    private static readonly string[] Variables = {
        "RW_TOOLPATHS_FIRST_ARC_EXPERIMENT", "RW_TOOLPATHS_TWO_ORIGIN_EXPERIMENT",
        "RW_TOOLPATHS_FIRST_ARC_REFERENCE", "RW_TOOLPATHS_TWO_ORIGIN_REFERENCE",
        "RW_TOOLPATHS_COHERENCE_REFERENCE", "RW_TOOLPATHS_ACCEPTED_INSET_REFERENCE",
    };

    [Fact]
    public void PocketDefaultsCaptureBothModesAndSharedFillRetainsDiagnosticOptIn()
    {
        using var environment = new EnvironmentScope();
        var captured = PocketToolpaths.CaptureConcentricConstructionMode(new PocketOptions());
        Assert.True(captured.DenseFirstArcConstruction);
        Assert.True(captured.TwoOriginConstruction);
        Assert.Null(new RingOffsetOptions(3, 4).DenseFirstArcConstruction);
        Assert.Null(new RingOffsetOptions(3, 4).TwoOriginConstruction);
        Environment.SetEnvironmentVariable("RW_TOOLPATHS_FIRST_ARC_REFERENCE", "1");
        Environment.SetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_REFERENCE", "1");
        Assert.True(captured.DenseFirstArcConstruction);
        Assert.True(captured.TwoOriginConstruction);
        var reference = PocketToolpaths.CaptureConcentricConstructionMode(new PocketOptions());
        Assert.False(reference.DenseFirstArcConstruction);
        Assert.False(reference.TwoOriginConstruction);
    }

    [Theory]
    [InlineData("RW_TOOLPATHS_FIRST_ARC_REFERENCE", false, true)]
    [InlineData("RW_TOOLPATHS_TWO_ORIGIN_REFERENCE", true, false)]
    [InlineData("RW_TOOLPATHS_COHERENCE_REFERENCE", false, false)]
    public void ReferencePoliciesAreIndependentAndExplicitScopeCannotBeEnabledByDiagnosticEnvironment(
        string variable, bool first, bool origins)
    {
        using var environment = new EnvironmentScope();
        Environment.SetEnvironmentVariable(variable, "1");
        var captured = PocketToolpaths.CaptureConcentricConstructionMode(new PocketOptions());
        Assert.Equal(first, captured.DenseFirstArcConstruction);
        Assert.Equal(origins, captured.TwoOriginConstruction);
        Environment.SetEnvironmentVariable("RW_TOOLPATHS_FIRST_ARC_EXPERIMENT", "1");
        Environment.SetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_EXPERIMENT", "1");
        var scoped = PocketToolpaths.CaptureConcentricConstructionMode(new PocketOptions {
            DenseFirstArcConstruction = false, TwoOriginConstruction = false });
        Assert.False(scoped.DenseFirstArcConstruction);
        Assert.False(scoped.TwoOriginConstruction);
    }

    [Fact]
    public void ActualDensePocketUsesCapturedDefaultsAfterProgressMutatesBothReferences()
    {
        using var environment = new EnvironmentScope();
        // Dense original and construction sites are separate eligibility tests.
        // Many curved islands remain dense after the bounded source reduction.
        var holes = Enumerable.Range(0, 36).Select(i => (IReadOnlyList<PointD>)Circle(8, 256)
            .Select(p => new PointD(p.x + 30 + 28 * (i % 6), p.y + 30 + 28 * (i / 6))).ToArray()).ToArray();
        var source = new MillingRegion(new[] { new PointD(0, 0), new(200, 0), new(200, 200), new(0, 200) }, holes);
        var writer = new StringWriter(); var previous = Console.Out;
        ToolpathPlan actual;
        try
        {
            using var logSelection = RW.Diagnostics.DiagnosticLog.PushNamespaces("*");
            Console.SetOut(writer);
            actual = PocketToolpaths.Generate(new[] { source }, ToolGeometry.Flat(3),
                new DepthSchedule(1, 1), new PocketOptions { StepOver = .55, MaxRings = 2,
                    DetectCurves = false, CalculateSweptArea = false },
                progress: (_, _) => {
                    Environment.SetEnvironmentVariable("RW_TOOLPATHS_FIRST_ARC_REFERENCE", "1");
                    Environment.SetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_REFERENCE", "1");
                });
        }
        finally { Console.SetOut(previous); }
        Assert.NotEmpty(actual.Toolpaths);
        string log = writer.ToString();
        Assert.Contains("denseFirstArcConstruction=True twoOriginConstruction=True", log);
        Assert.Contains("firstArc=0.00025", log);
        Assert.Contains("twoOriginExperiment=True", log);
        Assert.Contains("event=publish-shallow", log);
        Assert.Contains("originalWalls=True", log);
        Assert.Contains("fullSignedShellProof=True", log);
    }

    [Fact]
    public void PreparedAndToleranceOwnersKeepTheirCapturedOriginPolicy()
    {
        using var environment = new EnvironmentScope();
        var outer = Circle(30, 5000);
        var prepared = PolygonOps.PrepareBoundaryOffset(outer, null, .001, 0,
            sourceConstructionSweepRadius: 3, twoOriginConstruction: true);
        Environment.SetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_REFERENCE", "1");
        Assert.True(prepared.TwoOriginExperimentEnabled);
        Assert.True(prepared.WithTolerances(.00025, 0).TwoOriginExperimentEnabled);
        var legacy = PolygonOps.PrepareBoundaryOffset(outer, null, .001, 0,
            sourceConstructionSweepRadius: 3, twoOriginConstruction: false);
        Environment.SetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_EXPERIMENT", "1");
        Assert.False(legacy.TwoOriginExperimentEnabled);
        Assert.False(legacy.WithTolerances(.00025, 0).TwoOriginExperimentEnabled);
    }

    [Fact]
    public void IndirectCarvingKeepsScopedConstructionDisabledInActualSliceGeneration()
    {
        using var environment = new EnvironmentScope();
        const int size = 17;
        var heights = Enumerable.Range(0, size * size).Select(i => {
            double dx = i % size - 8, dy = i / size - 8;
            return (float)(.1 + .7 * Math.Exp(-(dx * dx + dy * dy) / 12));
        }).ToArray();
        var surface = new HeightmapSurface(new HeightmapGrid(size, size, heights),
            new HeightmapPlacement(40, 30, 0, 0), 10, 6);
        var writer = new StringWriter(); var previous = Console.Out;
        try
        {
            using var logSelection = RW.Diagnostics.DiagnosticLog.PushNamespaces("*");
            Console.SetOut(writer);
            var actual = CarvingRoughingToolpaths.Generate(surface, new ToolGeometry(2), 2,
                new CarvingOptions { StepOver = .4, SurfaceAllowance = .2, CalculateSweptArea = false });
            Assert.NotEmpty(actual.Toolpaths);
        }
        finally { Console.SetOut(previous); }
        string log = writer.ToString();
        Assert.Contains("denseFirstArcConstruction=False twoOriginConstruction=False", log);
        Assert.DoesNotContain("[RingOffsetEngine.FirstArcExperiment]", log);
        Assert.DoesNotContain("event=publish-shallow", log);
    }

    [Theory]
    [InlineData(true, .00025)]
    [InlineData(false, 1 / PathUtils.Scale)]
    public void ProtectionReserveUsesTheSameCapturedFirstConstruction(bool first, double arc)
    {
        using var environment = new EnvironmentScope();
        var options = new PocketOptions { DenseFirstArcConstruction = first,
            ProtectedBoundary = new[] { (IReadOnlyList<PointD>)Circle(8, 32) } };
        var domain = PocketProtection.PrepareConcentricMask(options, CancellationToken.None,
            options.DenseFirstArcConstruction);
        Assert.Equal(arc, domain.EffectiveArcTolerance);
        Assert.Equal(.001 + arc + 4 / PathUtils.Scale, domain.RequiredGuard);
        Assert.True(domain.ProofChords > 0);
    }

    private static List<PointD> Circle(double radius, int count) => Enumerable.Range(0, count)
        .Select(i => new PointD(radius * Math.Cos(Math.Tau * i / count), radius * Math.Sin(Math.Tau * i / count))).ToList();

    private sealed class EnvironmentScope : IDisposable
    {
        private readonly string?[] _previous = Variables.Select(Environment.GetEnvironmentVariable).ToArray();
        internal EnvironmentScope() { foreach (string variable in Variables) Environment.SetEnvironmentVariable(variable, null); }
        public void Dispose() { for (int i = 0; i < Variables.Length; i++) Environment.SetEnvironmentVariable(Variables[i], _previous[i]); }
    }
}
