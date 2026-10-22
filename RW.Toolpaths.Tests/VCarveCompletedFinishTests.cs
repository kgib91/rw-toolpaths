using System.Globalization;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveCompletedFinishTests
{
    private static readonly MillingRegion[] Artwork =
    { new(new PointD[] { new(0, 0), new(8, 0), new(8, 4), new(0, 4) }) };
    private static readonly ToolGeometry Cone = new(3)
    {
        VCarveProfile = new VCarveCutterProfile(new[]
        { new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0, 3, 3) }, 3),
        ConeLength = 3, TipAngleRadians = Math.PI / 2
    };
    private static readonly DepthSchedule Schedule = new(3, .5) { SurfaceZ = 4, StartDepth = 3 };
    private static readonly VCarveOptions Options = new()
    {
        SmoothVCarving = true, SmoothRampAngleDegrees = 15, IncludeInteriorFill = false,
        StepOver = .4, DetectCurves = false, CalculateSweptArea = false
    };

    [Fact]
    public void RoutedCommandedSmoothFinishRejectsPreviouslyAcceptedFlatCleanupGouge()
    {
        var provider = BoostVoronoiProvider.CreateDefault();
        var preparation = VCarveToolpaths.Prepare(Artwork, Cone, Schedule, Options, provider);
        var analysis = preparation.AnalyzeRoughing(3);
        Assert.Contains(0, analysis.EntireProtectedSourceRegionIndices);
        var plan = VCarveToolpaths.Generate(Artwork, Cone, Schedule,
            Options with { Preparation = preparation, RoughingAnalysis = analysis }, provider);
        Assert.Equal(0, preparation.MaximumPreservedFinishDepth(0, 4, 2.14));
        var chords = plan.Toolpaths.SelectMany(path => path.Points.Zip(path.Points.Skip(1),
            (a, b) => (Start: Command(a), End: Command(b)))).ToArray();
        preparation.BindCompletedFinish(chords);

        // The point is strictly inside the .15mm flat-cutter footprint centered
        // at (4,2). Every actual F3 cone chord is independently optimized here.
        double physicallyRemoved = ConeMaximum(chords, 4, 2.14);
        double certificate = preparation.MaximumPreservedFinishDepth(0, 4, 2.14);
        Assert.InRange(physicallyRemoved, 1.04, 1.07);
        Assert.True(certificate + .001 <= physicallyRemoved + 1e-10,
            $"Certificate {certificate:R} overstates actual F3 cone removal {physicallyRemoved:R}.");
        Assert.True(1.15 > certificate + .001,
            "The previously accepted 1.15mm flat cleanup must reject at this strict contact.");
        Assert.True(1.15 - physicallyRemoved > .08);
    }

    [Fact]
    public void CompletedEntryRampsUseContinuousPhysicalChordsRatherThanAnalyticalGraphPeaks()
    {
        var preparation = VCarveToolpaths.Prepare(Artwork, Cone, Schedule, Options,
            BoostVoronoiProvider.CreateDefault());
        var chords = new[]
        {
            (Start: new Point3D(0, 2, 0), End: new Point3D(8, 2, -2)),
            (Start: new Point3D(1, 1, .5), End: new Point3D(1, 1, -.3))
        };
        preparation.BindCompletedFinish(chords);
        foreach (var query in new[] { new PointD(4, 2), new PointD(3.3, 2.12), new PointD(1, 1), new PointD(1, 1.1) })
        {
            double actual = ConeMaximum(chords, query.x, query.y);
            double certificate = preparation.MaximumPreservedFinishDepth(0, query.x, query.y);
            Assert.True(certificate + .001 <= actual + 1e-10);
            Assert.True(certificate >= Math.Max(0, actual - .001000001));
        }
        // A continuous ramp midpoint removes stock even when neither endpoint
        // can witness any positive depth at that same location.
        Assert.InRange(preparation.MaximumPreservedFinishDepth(0, 4, 2), .998, 1);
        Assert.InRange(preparation.MaximumPreservedFinishDepth(0, 1, 1), .298, .3);
        Assert.Equal(1, preparation.PreservedFinishDepthLipschitz);
    }

    [Fact]
    public void FailedCanceledOrClearedCompilationCannotPublishNewRemovalProof()
    {
        var preparation = VCarveToolpaths.Prepare(Artwork, Cone, Schedule, Options,
            BoostVoronoiProvider.CreateDefault());
        Assert.False(preparation.HasCompletedFinish);
        var cut = (new Point3D(4, 2, -1), new Point3D(4, 2, -1));
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => preparation.BindCompletedFinish(new[] { cut }, cancellation.Token));
        Assert.Equal(0, preparation.MaximumPreservedFinishDepth(0, 4, 2));
        Assert.False(preparation.HasCompletedFinish);
        Assert.Throws<InvalidOperationException>(() => preparation.BindCompletedFinish(FailedBatch()));
        Assert.False(preparation.HasCompletedFinish);
        Assert.Throws<ArgumentException>(() => preparation.BindCompletedFinish(new[]
        { (new Point3D(double.NaN, 2, -1), new Point3D(4, 2, -1)) }));
        Assert.Equal(0, preparation.MaximumPreservedFinishDepth(0, 4, 2));
        preparation.BindCompletedFinish(new[] { cut });
        Assert.True(preparation.HasCompletedFinish);
        Assert.True(preparation.MaximumPreservedFinishDepth(0, 4, 2) > .99);
        preparation.ClearCompletedFinish();
        Assert.False(preparation.HasCompletedFinish);
        Assert.Equal(0, preparation.MaximumPreservedFinishDepth(0, 4, 2));

        IEnumerable<(Point3D, Point3D)> FailedBatch()
        {
            yield return cut;
            throw new InvalidOperationException("Incomplete command batch.");
        }
    }

    [Fact]
    public void RoundedCompositePhysicalWitnessIsAConservativeFiniteSlopeEnclosure()
    {
        var sections = new[]
        {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 1.6, 1.6),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.6, 3, 20)
        };
        var tool = new ToolGeometry(3) { VCarveProfile = new VCarveCutterProfile(sections, 21.6) };
        var depth = new DepthSchedule(6, .5);
        var preparation = VCarveToolpaths.Prepare(Artwork, tool, depth, Options, BoostVoronoiProvider.CreateDefault());
        preparation.BindCompletedFinish(new[] { (new Point3D(4, 2, -5), new Point3D(4, 2, -5)) });
        var physical = new CarvingCutterProfile(sections, 6);
        Assert.True(double.IsFinite(preparation.PreservedFinishDepthLipschitz));
        for (int i = 0; i <= 1000; i++)
        {
            double radius = i * .0031;
            double actual = Math.Max(0, 5 - physical.Bottom(radius));
            double certificate = preparation.MaximumPreservedFinishDepth(0, 4 + radius, 2);
            Assert.True(certificate <= actual + 1e-10);
            if (certificate > 0) Assert.True(certificate + .001 <= actual + 1e-10);
        }
    }

    [Fact]
    public void IndependentFinishScopeReusesCompleteAnalysisWithoutInheritingPublishedRemoval()
    {
        var preparation = Prepare();
        var analysis = preparation.AnalyzeRoughing(3);
        preparation.BindCompletedFinish(new[] { Stationary(1.25) });

        var scope = preparation.CreateIndependentFinishScope();
        var scopedAnalysis = analysis.WithPreparation(scope);

        Assert.NotSame(preparation, scope);
        Assert.Same(preparation.OriginalRegions, scope.OriginalRegions);
        Assert.Same(preparation.Regions, scope.Regions);
        Assert.Equal(preparation.Fingerprint, scope.Fingerprint);
        Assert.Same(scope, scopedAnalysis.Preparation);
        Assert.Same(analysis.ProtectedBoundary, scopedAnalysis.ProtectedBoundary);
        Assert.Same(analysis.ProtectedRegions, scopedAnalysis.ProtectedRegions);
        Assert.Same(analysis.Basins, scopedAnalysis.Basins);
        Assert.Same(analysis.ProtectedSourceRegionIndices, scopedAnalysis.ProtectedSourceRegionIndices);
        Assert.Same(analysis.EntireProtectedSourceRegionIndices, scopedAnalysis.EntireProtectedSourceRegionIndices);
        Assert.Equal(analysis.ProtectedArea, scopedAnalysis.ProtectedArea);
        Assert.False(scope.HasCompletedFinish);
        Assert.Equal(0, scope.MaximumPreservedFinishDepth(0, 4, 2));
        Assert.True(preparation.HasCompletedFinish);
        Assert.InRange(preparation.MaximumPreservedFinishDepth(0, 4, 2), 1.248, 1.25);
    }

    [Fact]
    public void StaleCompilationClearCannotDiscardAnotherCompilationsNewerFinish()
    {
        var cached = Prepare();
        var analysis = cached.AnalyzeRoughing(3);
        var older = cached.CreateIndependentFinishScope();
        var newer = cached.CreateIndependentFinishScope();
        var olderAnalysis = analysis.WithPreparation(older);
        var newerAnalysis = analysis.WithPreparation(newer);

        older.BindCompletedFinish(new[] { Stationary(.75) });
        newer.BindCompletedFinish(new[] { Stationary(1.5) });
        Assert.InRange(olderAnalysis.Preparation.MaximumPreservedFinishDepth(0, 4, 2), .748, .75);
        Assert.InRange(newerAnalysis.Preparation.MaximumPreservedFinishDepth(0, 4, 2), 1.498, 1.5);

        // This reproduces the stale failed compiler's Dispose after the newer bind.
        older.ClearCompletedFinish();
        Assert.False(older.HasCompletedFinish);
        Assert.Equal(0, older.MaximumPreservedFinishDepth(0, 4, 2));
        Assert.True(newer.HasCompletedFinish);
        Assert.InRange(newer.MaximumPreservedFinishDepth(0, 4, 2), 1.498, 1.5);
        Assert.False(cached.HasCompletedFinish);
        Assert.Equal(0, cached.MaximumPreservedFinishDepth(0, 4, 2));
    }

    [Fact]
    public void FailedAndCanceledScopePublicationCannotChangeOtherCompletedScope()
    {
        var cached = Prepare();
        var failing = cached.CreateIndependentFinishScope();
        var completed = cached.CreateIndependentFinishScope();
        completed.BindCompletedFinish(new[] { Stationary(1.5) });
        double preserved = completed.MaximumPreservedFinishDepth(0, 4, 2);

        Assert.Throws<InvalidOperationException>(() => failing.BindCompletedFinish(FailedBatch()));
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => failing.BindCompletedFinish(
            new[] { Stationary(2.5) }, cancellation.Token));
        failing.ClearCompletedFinish();

        Assert.False(failing.HasCompletedFinish);
        Assert.False(cached.HasCompletedFinish);
        Assert.True(completed.HasCompletedFinish);
        Assert.Equal(preserved, completed.MaximumPreservedFinishDepth(0, 4, 2));

        static IEnumerable<(Point3D Start, Point3D End)> FailedBatch()
        {
            yield return Stationary(2.5);
            throw new InvalidOperationException("Incomplete compilation-owned command batch.");
        }
    }

    [Fact]
    public void ForkOfBoundScopeStartsUnboundAndKeepsOriginalAnalyticalOwner()
    {
        var preparation = Prepare();
        var analysis = preparation.AnalyzeRoughing(3);
        var first = preparation.CreateIndependentFinishScope();
        first.BindCompletedFinish(new[] { Stationary(1.5) });
        var second = first.CreateIndependentFinishScope();
        var secondAnalysis = analysis.WithPreparation(second);

        Assert.Same(preparation.Regions, second.Regions);
        Assert.Same(preparation.OriginalRegions, second.OriginalRegions);
        Assert.Same(analysis.ProtectedBoundary, secondAnalysis.ProtectedBoundary);
        Assert.False(second.HasCompletedFinish);
        second.BindCompletedFinish(new[] { Stationary(.75) });
        first.ClearCompletedFinish();
        Assert.True(second.HasCompletedFinish);
        Assert.InRange(second.MaximumPreservedFinishDepth(0, 4, 2), .748, .75);
        Assert.False(preparation.HasCompletedFinish);
    }

    [Fact]
    public void AnalysisCannotBeReboundToUnrelatedGeometryEvenWithMatchingAuthoredFingerprint()
    {
        var preparation = Prepare();
        var analysis = preparation.AnalyzeRoughing(3);
        var separatelyAnalyzed = Prepare();
        Assert.Equal(preparation.Fingerprint, separatelyAnalyzed.Fingerprint);
        Assert.NotSame(preparation.Regions, separatelyAnalyzed.Regions);
        Assert.Throws<ArgumentException>(() => analysis.WithPreparation(separatelyAnalyzed));
        Assert.Throws<ArgumentNullException>(() => analysis.WithPreparation(null!));
    }

    private static VCarvePreparation Prepare() => VCarveToolpaths.Prepare(
        Artwork, Cone, Schedule, Options, BoostVoronoiProvider.CreateDefault());

    private static (Point3D Start, Point3D End) Stationary(double depth)
        => (new Point3D(4, 2, -depth), new Point3D(4, 2, -depth));

    private static Point3D Command(Point3D point)
    {
        static double F3(float value) => double.Parse(value.ToString("F3", CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);
        // The native plan has already translated relative tip Z to surface Z.
        return new Point3D(F3((float)point.X), F3((float)point.Y),
            F3((float)point.Z - (float)Schedule.SurfaceZ));
    }

    private static double ConeMaximum(IEnumerable<(Point3D Start, Point3D End)> chords, double x, double y)
    {
        double maximum = 0;
        foreach (var (a, b) in chords)
        {
            double Value(double t)
            {
                double px = a.X + t * (b.X - a.X), py = a.Y + t * (b.Y - a.Y);
                double depth = -a.Z - t * (b.Z - a.Z);
                return depth - Math.Sqrt((px - x) * (px - x) + (py - y) * (py - y));
            }
            // Independent numerical optimization of the concave continuous
            // physical cone contact; no production BVH/witness helper is used.
            double left = 0, right = 1;
            for (int i = 0; i < 90; i++)
            {
                double third = (right - left) / 3;
                if (Value(left + third) < Value(right - third)) left += third;
                else right -= third;
            }
            maximum = Math.Max(maximum, Math.Max(Math.Max(Value(0), Value(1)), Value((left + right) / 2)));
        }
        return maximum;
    }
}
