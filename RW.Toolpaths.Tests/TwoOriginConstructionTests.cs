using System.Reflection;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Two origin construction diagnostic controls", DisableParallelization = true)]
public sealed class TwoOriginConstructionCollection { }

[Collection("Two origin construction diagnostic controls")]
public sealed class TwoOriginConstructionTests
{
    [Fact]
    public void FixedDeepAndShallowOriginsServeBacktrackingWithoutReplacingSourceAuthority()
    {
        using var scope = new ExperimentScope();
        var outer = Dense(Rectangle(0, 0, 60, 90));
        var hole = Rectangle(20, 30, 40, 60);
        var prepared = Prepare(outer, new[] { hole });
        var firstContext = prepared.WithTolerances(1 / PathUtils.Scale, 0);
        var accepted = firstContext.Execute(-3, compactFront: false);
        Assert.True(prepared.TrySeedAcceptedFirstInset(accepted, -3, CancellationToken.None));
        Assert.Equal(0, prepared.TwoOriginDeepAidPoints);
        Assert.Equal(3, prepared.TwoOriginShallowDistance);
        string shallowIdentity = Aid(prepared, "_shallowInsetBase");

        var deep = prepared.Execute(-9, compactFront: false);
        ProveOriginal(deep, outer, new[] { hole }, 9);
        Assert.Equal(1, prepared.DenseSourceConstructionCount);
        Assert.Equal(9, prepared.TwoOriginDeepDistance);
        string deepIdentity = Aid(prepared, "_denseInsetBase");
        var backtracked = prepared.Execute(-6, compactFront: false);
        ProveOriginal(backtracked, outer, new[] { hole }, 6);
        Assert.Equal(1, prepared.TwoOriginShallowReuseCount);
        ProveOriginal(prepared.Execute(-10, compactFront: false), outer, new[] { hole }, 10);
        Assert.Equal(1, prepared.TwoOriginDeepReuseCount);
        Assert.Equal(1, prepared.DenseSourceConstructionCount);
        Assert.Equal(shallowIdentity, Aid(prepared, "_shallowInsetBase"));
        Assert.Equal(deepIdentity, Aid(prepared, "_denseInsetBase"));

        // The established recovery owns the same immutable geometry, with no
        // experimental seed state. It can reproduce the original search.
        var recovery = prepared.WithTolerances(.001, 0, validateFirstFootprint: false);
        Assert.Same(Field(prepared, "_geometry"), Field(recovery, "_geometry"));
        Assert.Same(Field(prepared, "_construction"), Field(recovery, "_construction"));
        Assert.Equal(0, recovery.TwoOriginShallowAidPoints);
        Assert.Equal(0, recovery.TwoOriginDeepAidPoints);
        Assert.Equal(Commands(deep), Commands(recovery.Execute(-9, compactFront: false)));
        Assert.Equal(0, recovery.TwoOriginShallowReuseCount);
    }

    [Fact]
    public void BacktrackingBeforeSeedAndAtDeepOriginUsesCompleteOriginalConstruction()
    {
        using var scope = new ExperimentScope();
        var outer = Dense(Rectangle(0, 0, 60, 90));
        var prepared = Prepare(outer, null);
        var accepted = prepared.WithTolerances(1 / PathUtils.Scale, 0).Execute(-3, compactFront: false);
        Assert.True(prepared.TrySeedAcceptedFirstInset(accepted, -3, CancellationToken.None));
        ProveOriginal(prepared.Execute(-2, compactFront: false), outer, null, 2);
        Assert.Equal(0, prepared.TwoOriginDeepAidPoints);
        Assert.Equal(0, prepared.TwoOriginShallowReuseCount);
        var deep = prepared.Execute(-9, compactFront: false);
        string identity = Aid(prepared, "_denseInsetBase");
        Assert.Equal(Commands(deep), Commands(prepared.Execute(-9, compactFront: false)));
        Assert.Equal(0, prepared.TwoOriginShallowReuseCount);
        Assert.Equal(identity, Aid(prepared, "_denseInsetBase"));
        Assert.Equal(3, prepared.DenseSourceConstructionCount);
    }

    [Fact]
    public void ShallowBacktrackRetainsReachableNeckAndDeepSplitProtectsEveryOriginalWall()
    {
        using var scope = new ExperimentScope();
        const double width = 12.02;
        var outer = Dense(new List<PointD> { new(-30,-12), new(-8,-12), new(-8,-width/2),
            new(8,-width/2), new(8,-12), new(30,-12), new(30,12), new(8,12),
            new(8,width/2), new(-8,width/2), new(-8,12), new(-30,12) });
        var prepared = Prepare(outer, null);
        var first = prepared.WithTolerances(1 / PathUtils.Scale, 0).Execute(-3, compactFront: false);
        Assert.True(prepared.TrySeedAcceptedFirstInset(first, -3, CancellationToken.None));
        var deep = prepared.Execute(-8, compactFront: false);
        Assert.Equal(2, PolygonOps.DifferenceToComponents(deep, Array.Empty<IReadOnlyList<PointD>>()).Count);
        var shallow = prepared.Execute(-5.5, compactFront: false);
        Assert.Single(PolygonOps.DifferenceToComponents(shallow, Array.Empty<IReadOnlyList<PointD>>()));
        ProveOriginal(shallow, outer, null, 5.5);
        ProveOriginal(deep, outer, null, 8);
        Assert.Equal(1, prepared.DenseSourceConstructionCount);
        Assert.Equal(1, prepared.TwoOriginShallowReuseCount);
    }

    [Fact]
    public void MissingProtectedHoleAndInsufficientClearanceCannotPublishShallowAid()
    {
        using var scope = new ExperimentScope();
        var outer = Dense(Rectangle(0, 0, 60, 90));
        var hole = Rectangle(20, 30, 40, 60);
        var prepared = Prepare(outer, new[] { hole });
        Assert.False(prepared.TrySeedAcceptedFirstInset(new List<List<PointD>> { Rectangle(3,3,57,87) }, -3, CancellationToken.None));
        Assert.False(prepared.AcceptedInsetSeeded);
        var insufficient = prepared.WithTolerances(1 / PathUtils.Scale, 0).Execute(-1, compactFront: false);
        Assert.False(prepared.TrySeedAcceptedFirstInset(insufficient, -3, CancellationToken.None));
        Assert.Equal(0, prepared.TwoOriginShallowAidPoints);
        Assert.Equal(0, prepared.TwoOriginDeepAidPoints);
    }

    [Fact]
    public void CanceledSeedAndWarmTrialPublishNoAidOrWorkAndPreserveExistingIdentity()
    {
        using var scope = new ExperimentScope();
        var outer = Dense(Rectangle(0, 0, 60, 90));
        var prepared = Prepare(outer, null);
        var accepted = prepared.WithTolerances(1 / PathUtils.Scale, 0).Execute(-3, compactFront: false);
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => prepared.TrySeedAcceptedFirstInset(accepted, -3, cancellation.Token));
        Assert.Equal(0, prepared.TwoOriginShallowAidPoints);
        Assert.True(prepared.TrySeedAcceptedFirstInset(accepted, -3, CancellationToken.None));
        prepared.Execute(-9, compactFront: false);
        int offsets = prepared.OffsetCount;
        string shallow = Aid(prepared, "_shallowInsetBase"), deep = Aid(prepared, "_denseInsetBase");
        Assert.Throws<OperationCanceledException>(() => prepared.Execute(-6, cancellation.Token, compactFront: false));
        Assert.Equal(offsets, prepared.OffsetCount);
        Assert.Equal(shallow, Aid(prepared, "_shallowInsetBase"));
        Assert.Equal(deep, Aid(prepared, "_denseInsetBase"));
    }

    [Fact]
    public void ZeroAndOutwardRequestsRetainExactSourceAfterBothOriginsArePublished()
    {
        using var scope = new ExperimentScope();
        var outer = Dense(Rectangle(0, 0, 60, 90));
        var hole = Rectangle(20, 30, 40, 60);
        var prepared = Prepare(outer, new[] { hole });
        var accepted = prepared.WithTolerances(1 / PathUtils.Scale, 0).Execute(-3, compactFront: false);
        Assert.True(prepared.TrySeedAcceptedFirstInset(accepted, -3, CancellationToken.None));
        prepared.Execute(-9, compactFront: false);
        var reference = Prepare(outer, new[] { hole });
        Assert.Equal(Commands(reference.Execute(0, compactFront: false)), Commands(prepared.Execute(0, compactFront: false)));
        Assert.Equal(Commands(reference.Execute(.25, compactFront: false)), Commands(prepared.Execute(.25, compactFront: false)));
        Assert.Equal(0, prepared.TwoOriginShallowReuseCount);
        Assert.Equal(0, prepared.TwoOriginDeepReuseCount);
    }

    [Fact]
    public void DisabledExperimentKeepsExistingSingleOriginSeedBehavior()
    {
        using var scope = new ExperimentScope(null);
        var outer = Dense(Rectangle(0, 0, 60, 90));
        var prepared = Prepare(outer, null);
        Assert.False(prepared.TwoOriginExperimentEnabled);
        var accepted = prepared.WithTolerances(1 / PathUtils.Scale, 0).Execute(-3, compactFront: false);
        Assert.True(prepared.TrySeedAcceptedFirstInset(accepted, -3, CancellationToken.None));
        ProveOriginal(prepared.Execute(-9, compactFront: false), outer, null, 9);
        ProveOriginal(prepared.Execute(-6, compactFront: false), outer, null, 6);
        Assert.Equal(0, prepared.DenseSourceConstructionCount);
        Assert.Equal(2, prepared.DenseInsetReuseCount);
        Assert.Equal(0, prepared.TwoOriginShallowAidPoints);
        Assert.Equal(0, prepared.TwoOriginShallowReuseCount);
    }

    private static PolygonOps.PreparedBoundaryOffset Prepare(List<PointD> outer, IEnumerable<IReadOnlyList<PointD>>? holes)
        => PolygonOps.PrepareBoundaryOffset(outer, holes, .001, 0, sourceConstructionSweepRadius: 3,
            sourceConstructionValidateFirstFootprint: false);
    private static object? Field(object prepared, string name) => prepared.GetType()
        .GetField(name, BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(prepared);
    private static string Aid(object prepared, string name) => JsonSerializer.Serialize(
        ((Paths64?)Field(prepared, name))?.Select(path => path.Select(point => new[] { point.X, point.Y })));
    private static string[] Commands(IEnumerable<IReadOnlyList<PointD>> paths)
        => paths.Select(path => string.Join(";", path.Select(point => $"{point.x:R},{point.y:R}"))).ToArray();
    private static void ProveOriginal(List<List<PointD>> candidate, List<PointD> outer,
        IEnumerable<IReadOnlyList<PointD>>? holes, double inset)
    {
        Assert.NotEmpty(candidate);
        var source = PolygonOps.Difference(new[] { outer }, holes ?? Array.Empty<IReadOnlyList<PointD>>());
        var native = PathUtils.ToClipper(candidate);
        Assert.Empty(Clipper.Difference(native, PathUtils.ToClipper(source), FillRule.NonZero));
        var walls = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(source));
        int chords = 0;
        foreach (var path in native)
        for (int i = 0; i < path.Count; i++)
        {
            var a = path[i]; var b = path[(i + 1) % path.Count]; chords++;
            Assert.True(walls.IsInside(a));
            Assert.Null(walls.ClearanceViolation(a, b, inset * PathUtils.Scale - 4));
        }
        Console.WriteLine($"[TwoOrigin.Control] inset={inset:R} contours={native.Count} chords={chords} "
            + "completeOriginalFilledContainment=True originalWallChords=True stockAcceptance=False");
    }
    private static List<PointD> Dense(List<PointD> corners)
        => Enumerable.Range(0, corners.Count).SelectMany(edge => Enumerable.Range(0, 1100).Select(index => {
            var a = corners[edge]; var b = corners[(edge + 1) % corners.Count]; double t = index / 1100d;
            return new PointD(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
        })).ToList();
    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => new() { new(x0,y0), new(x1,y0), new(x1,y1), new(x0,y1) };
    private sealed class ExperimentScope : IDisposable
    {
        private readonly string? _previous = Environment.GetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_EXPERIMENT");
        internal ExperimentScope(string? value = "1") => Environment.SetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_EXPERIMENT", value);
        public void Dispose() => Environment.SetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_EXPERIMENT", _previous);
    }
}
