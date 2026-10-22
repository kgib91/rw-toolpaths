using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveConstructorObserverTests
{
    [Fact]
    public void CompleteConstructorBatchIsCapturedSeriallyBeforeDispatchWithoutChangingOutput()
    {
        var captures = new List<VCarveSmoothing.CorrectionConstructorInputs>();
        int caller = Environment.CurrentManagedThreadId;
        var previous = VCarveSmoothing.CorrectionConstructorObserver.Value;
        List<TaggedToolpath> candidate;
        try
        {
            VCarveSmoothing.CorrectionConstructorObserver.Value = input =>
            {
                Assert.Equal(caller, Environment.CurrentManagedThreadId);
                captures.Add(input);
            };
            candidate = Run(message =>
            {
                if (!message.Contains("stage=smooth-concentric event=partition round=0")) return;
                var firstBatch = captures.Where(input => input.Round == 0).ToArray();
                Assert.NotEmpty(firstBatch);
                Assert.Equal(firstBatch[0].BatchComponentCount, firstBatch.Length);
                Assert.Equal(Enumerable.Range(0, firstBatch.Length), firstBatch.Select(input => input.Component));
            });
        }
        finally { VCarveSmoothing.CorrectionConstructorObserver.Value = previous; }
        var reference = Run(null);
        Assert.Equal(reference.Count, candidate.Count);
        for (int path = 0; path < reference.Count; path++)
        {
            Assert.Equal(reference[path].Points, candidate[path].Points);
            Assert.Equal(reference[path].Spans, candidate[path].Spans);
            Assert.Equal(reference[path].Moves, candidate[path].Moves);
            Assert.Equal(reference[path].Category, candidate[path].Category);
            Assert.Equal(reference[path].RegionIndex, candidate[path].RegionIndex);
            Assert.Equal(reference[path].DepthPassIndex, candidate[path].DepthPassIndex);
            Assert.Equal(reference[path].VisitIdentity, candidate[path].VisitIdentity);
        }
        Assert.NotEmpty(captures);
        Assert.All(captures, input =>
        {
            Assert.Same(captures[0].OriginalSourceBoundary, input.OriginalSourceBoundary);
            Assert.NotEmpty(input.ErrorBoundary);
            Assert.False(input.OrdinaryCollector);
            Assert.Equal(VCarveSmoothing.ReferenceCorrectionArcTolerance, input.ArcTolerance);
            Assert.Equal(3, input.Schedule.Depth);
            Assert.True(input.InitialCutDepth >= .001);
            Assert.True(double.IsFinite(input.FloorCap));
        });
    }

    [Fact]
    public void DiagnosticMayStopAfterAllInputsBeforeAnyConstructorWorkOrOutput()
    {
        var captures = new List<VCarveSmoothing.CorrectionConstructorInputs>();
        var logs = new List<string>();
        var stop = new OperationCanceledException("diagnostic input capture complete; no job completion");
        var previous = VCarveSmoothing.CorrectionConstructorObserver.Value;
        try
        {
            VCarveSmoothing.CorrectionConstructorObserver.Value = input =>
            {
                captures.Add(input);
                if (input.Component == input.BatchComponentCount - 1) throw stop;
            };
            var actual = Assert.Throws<OperationCanceledException>(() => Run(logs.Add));
            Assert.Same(stop, actual);
        }
        finally { VCarveSmoothing.CorrectionConstructorObserver.Value = previous; }
        Assert.NotEmpty(captures);
        Assert.Equal(captures[0].BatchComponentCount, captures.Count);
        Assert.Equal(Enumerable.Range(0, captures.Count), captures.Select(input => input.Component));
        Assert.DoesNotContain(logs, message => message.Contains("stage=smooth-concentric event=partition")
            || message.Contains("stage=smooth-concentric event=inset-begin")
            || message.Contains("stage=smoothing event=complete"));
    }

    private static List<TaggedToolpath> Run(Action<string>? diagnostic)
    {
        IReadOnlyList<TaggedToolpath> source = [new TaggedToolpath(
            [new(1, 2, -.2), new(4, 2, -3), new(8, 2, -3)], Array.Empty<ToolpathSpan>(),
            0, "final-carve", null, visitIdentity: 17, detectCurves: false)];
        IReadOnlyList<IReadOnlyList<PointD>> boundary =
            [new PointD[] { new(0, 0), new(10, 0), new(10, 4), new(0, 4) }];
        var tool = new ToolGeometry(5) { BottomRadius = 0, TopRadius = 5, ConeLength = 10,
            TipAngleRadians = 2 * Math.Atan(.5) };
        return VCarveSmoothing.Apply(source, boundary, tool, new DepthSchedule(3, 1),
            new VCarveOptions { SmoothVCarving = true, SmoothRampAngleDegrees = 15,
                IncludeInteriorFill = false, DetectCurves = false }, CancellationToken.None,
            diagnostic, correctionWorkers: 4);
    }
}
