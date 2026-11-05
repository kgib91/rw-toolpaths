using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveParallelCorrectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ParallelCorrectionPreservesOrderedGeometryAndAlreadyCompleteChannels(bool rounded)
    {
        var fixture = Fixture(rounded);
        var serialLog = new List<string>();
        var parallelLog = new List<string>();
        var serialProgress = new List<string>();
        var parallelProgress = new List<string>();
        int caller = Environment.CurrentManagedThreadId;
        List<TaggedToolpath> Run(int workers, List<string> log, List<string> progress)
            => VCarveSmoothing.Apply(fixture.Paths, fixture.Boundary, fixture.Tool, new DepthSchedule(3, 1.5),
                new VCarveOptions { SmoothVCarving = true, SmoothRampAngleDegrees = rounded ? 30 : 15,
                    // Full fill reaches correction dispatch. Fill Limit exits
                    // with its bounded initial cohort before correction runs.
                    IncludeInteriorFill = true, DetectCurves = false }, CancellationToken.None,
                message => { Assert.Equal(caller, Environment.CurrentManagedThreadId); log.Add(message); },
                (_, message, _) => { Assert.Equal(caller, Environment.CurrentManagedThreadId); progress.Add(message); },
                correctionWorkers: workers);

        var serial = Run(1, serialLog, serialProgress);
        var parallel = Run(4, parallelLog, parallelProgress);
        if(rounded)
        {
            // Full outline ramps already complete this rounded fixture. The
            // scheduler must not invent correction or terminal visits merely
            // because multiple workers were requested.
            Assert.Contains(parallelLog,line=>line.Contains("stage=smoothing event=complete correctionRounds=0 correctionPaths=0")
                &&line.Contains("widthVerified=True"));
            Assert.DoesNotContain(parallelLog,line=>line.Contains("stage=smooth-concentric event=partition"));
        }
        else
        {
            Assert.True(parallelLog.Any(line => line.Contains("workerPolicy=independent-components")),
                string.Join(Environment.NewLine,parallelLog.Where(line=>line.Contains("stage=smooth-concentric")
                    ||line.Contains("stage=smoothing event=complete")||line.Contains("stage=smooth-residual"))));
            int expectedWorkers = Math.Min(4, Environment.ProcessorCount);
            Assert.Contains(parallelLog, line => line.Contains("event=workers-complete")
                && line.Contains($"workers={expectedWorkers} "));
        }
        Assert.Contains(parallel, path => path.Category == "clearing");
        Assert.Equal(serial.Count, parallel.Count);
        for (int i = 0; i < serial.Count; i++)
        {
            Assert.Equal(serial[i].Points, parallel[i].Points);
            Assert.Equal(serial[i].Spans, parallel[i].Spans);
            Assert.Equal(serial[i].Moves, parallel[i].Moves);
            Assert.Equal(serial[i].RegionIndex, parallel[i].RegionIndex);
            Assert.Equal(serial[i].Category, parallel[i].Category);
            Assert.Equal(serial[i].DepthPassIndex, parallel[i].DepthPassIndex);
            Assert.Equal(serial[i].VisitIdentity, parallel[i].VisitIdentity);
        }
        Assert.Equal(serialProgress, parallelProgress);
        Assert.Equal(serialLog.Where(line => line.Contains("stage=smooth-concentric event=begin")),
            parallelLog.Where(line => line.Contains("stage=smooth-concentric event=begin")));
        Assert.Contains(parallel,path=>path.Category=="final-carve");
    }

    [Fact]
    public void CancellationAtCorrectionDispatchPropagatesWithoutPublishingResult()
    {
        var fixture = Fixture(false);
        using var cancellation = new CancellationTokenSource();
        bool reachedCorrection = false;
        Assert.Throws<OperationCanceledException>(() => VCarveSmoothing.Apply(fixture.Paths, fixture.Boundary,
            fixture.Tool, new DepthSchedule(3, 1.5),
            new VCarveOptions { SmoothVCarving = true, SmoothRampAngleDegrees = 15,
                IncludeInteriorFill = true, DetectCurves = false }, cancellation.Token, message =>
            {
                if (!message.Contains("stage=smooth-concentric event=partition")) return;
                reachedCorrection = true;
                cancellation.Cancel();
            }, correctionWorkers: 4));
        Assert.True(reachedCorrection);
    }

    [Fact]
    public void WholeSourceCacheAndCropBoundsExposeSerialPublicationBarrier()
    {
        IReadOnlyList<PointD>[] source = [new PointD[] { new(0, 0), new(8, 0), new(8, 8), new(0, 8) }];
        var boundary = new VCarveSmoothing.CorrectionBoundary(source);
        IReadOnlyList<PointD> error = [new(3, 3), new(4, 3), new(4, 4), new(3, 4)];
        Assert.False(boundary.CouldCropWholeSource(error, .1, 0));
        Assert.True(boundary.CouldCropWholeSource(error, 3, 0));
        Assert.False(boundary.HasWholeSourceInsets);
        var whole = boundary.Crop(error, 3);
        boundary.RememberWholeSourceInset(whole, 3, []);
        Assert.True(boundary.HasWholeSourceInsets);
        Assert.Equal(1, VCarveSmoothing.ResolveCorrectionWorkers(1));
        Assert.InRange(VCarveSmoothing.ResolveCorrectionWorkers(100), 1, 4);
    }

    private static (IReadOnlyList<TaggedToolpath> Paths, IReadOnlyList<IReadOnlyList<PointD>> Boundary,
        ToolGeometry Tool) Fixture(bool rounded)
    {
        VCarveCutterProfile? profile = rounded ? new([
            new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 3) : null;
        ToolGeometry tool = rounded ? new(3) { VCarveProfile = profile }
            : new(5) { BottomRadius = 0, TopRadius = 5, ConeLength = 10, TipAngleRadians = 2 * Math.Atan(.5) };
        IReadOnlyList<PointD> outline = rounded
            ? [new(0, 0), new(16, 0), new(16, 5), new(0, 1.5)]
            : [new(0, 0), new(10, 0), new(10, 4), new(0, 4)];
        IReadOnlyList<TaggedToolpath> raw = rounded
            ? MedialAxisToolpaths.GenerateVCarveTagged(BoostVoronoiProvider.CreateDefault(),
                [outline], 0, 3, 0, 1.5, includeInteriorFill: false, detectCurves: false, cutterProfile: profile)
            : [new TaggedToolpath([new(1, 2, -.2), new(4, 2, -3), new(8, 2, -3)],
                Array.Empty<ToolpathSpan>(), 0, "final-carve", null, visitIdentity: 17, detectCurves: false)];
        var paths = new List<TaggedToolpath>();
        var boundary = new List<IReadOnlyList<PointD>>();
        for (int copy = 0; copy < 4; copy++)
        {
            double shift = copy * 40;
            boundary.Add(outline.Select(point => new PointD(point.x + shift, point.y)).ToArray());
            // A signed protected island outside the original channel still
            // participates in the complete-source commanded-wall proof.
            boundary.Add([new(shift + .1, .01), new(shift + .1, .04),
                new(shift + .2, .04), new(shift + .2, .01)]);
            paths.AddRange(raw.Select(path => new TaggedToolpath(
                path.Points.Select(point => new Point3D(point.X + shift, point.Y, point.Z)).ToArray(),
                path.Spans, path.RegionIndex, path.Category, path.DepthPassIndex,
                path.VisitIdentity, detectCurves: false)));
        }
        return (paths, boundary, tool);
    }
}
