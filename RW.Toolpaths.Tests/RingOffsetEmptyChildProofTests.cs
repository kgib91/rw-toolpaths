using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class RingOffsetEmptyChildProofTests
{
    private const string ReferenceVariable = "RW_TOOLPATHS_EMPTY_CHILD_PROOF_REFERENCE";
    private static readonly MethodInfo ChildProof = typeof(RingOffsetEngine).GetMethod(
        "RemoveCompletedLowPitchChildren", BindingFlags.Static | BindingFlags.NonPublic)!;

    [Fact]
    public void CompleteLowPitchCollectorKeepsOrderedOutputWhileSkippingItsEmptyNextFront()
    {
        var referenceLogs = new List<string>();
        var candidateLogs = new List<string>();
        var reference = Run(true, referenceLogs);
        var candidate = Run(false, candidateLogs);
        Assert.NotEmpty(reference);
        Assert.Equal(reference.Count, candidate.Count);
        for (int i = 0; i < reference.Count; i++)
        {
            Assert.Equal(reference[i].Points, candidate[i].Points);
            Assert.Equal(reference[i].IsClosed, candidate[i].IsClosed);
            Assert.Equal(reference[i].IsHole, candidate[i].IsHole);
            Assert.Equal(reference[i].Level, candidate[i].Level);
            Assert.Equal(reference[i].RegionIndex, candidate[i].RegionIndex);
            Assert.Equal(reference[i].OffsetDistance, candidate[i].OffsetDistance);
        }
        Assert.Contains(referenceLogs, line => line.Contains("[RingOffsetEngine.CompletedLowPitch]"));
        Assert.DoesNotContain(referenceLogs, line => line.Contains("[RingOffsetEngine.EmptyChildProof]"));
        Assert.Contains(candidateLogs, line => line.Contains("[RingOffsetEngine.EmptyChildProof]")
            && line.Contains("physicalCoverageComputed=False") && line.Contains("stockAuthorityComputed=False"));
    }

    [Fact]
    public void EmptyProposalReturnsItsOriginalListWithoutReadingClipOrCurrentGeometry()
    {
        WithReference(false, () =>
        {
            var generated = new List<List<PointD>>();
            var current = new List<List<PointD>> { new() { new(double.NaN, double.PositiveInfinity) } };
            IReadOnlyList<PointD> clip = [new(double.NaN, 0), new(1, 0), new(0, 1)];
            var result = (List<List<PointD>>)ChildProof.Invoke(null,
                [current, generated, 1d, CancellationToken.None, clip])!;
            Assert.Same(generated, result);
            Assert.Empty(result);
        });
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CancellationIsObservedEvenWhenNoChildExists(bool reference)
    {
        WithReference(reference, () =>
        {
            using var cancellation = new CancellationTokenSource();
            cancellation.Cancel();
            var error = Assert.Throws<TargetInvocationException>(() => ChildProof.Invoke(null,
                [new List<List<PointD>>(), new List<List<PointD>>(), 1d, cancellation.Token, null]));
            Assert.IsType<OperationCanceledException>(error.InnerException);
        });
    }

    [Fact]
    public void NonemptyChildRetainsLiteralCoverageDecision()
    {
        List<List<PointD>> current = [[new(0, 0), new(10, 0), new(10, 10), new(0, 10)]];
        List<List<PointD>> generated = [[new(1, 1), new(9, 1), new(9, 9), new(1, 9)]];
        List<List<PointD>> Invoke(bool reference)
        {
            List<List<PointD>>? result = null;
            WithReference(reference, () => result = (List<List<PointD>>)ChildProof.Invoke(null,
                [current, generated, .25d, CancellationToken.None, null])!);
            return result!;
        }
        var expected = Invoke(true); var actual = Invoke(false);
        Assert.NotEmpty(expected);
        Assert.Equal(expected.SelectMany(path => path), actual.SelectMany(path => path));
        Assert.Equal(expected.Select(path => path.Count), actual.Select(path => path.Count));
    }

    private static List<OffsetRing> Run(bool reference, List<string> messages)
    {
        List<OffsetRing>? result = null;
        WithReference(reference, () =>
        {
            var previousOutput = Console.Out;
            using var output = new StringWriter();
            try
            {
                using var logSelection = RW.Diagnostics.DiagnosticLog.PushNamespaces("*");
                Console.SetOut(output);
                result = MedialAxisGeometry.BuildConcentricFill(
                    [new MillingRegion([new(0, 0), new(2, 0), new(2, .15), new(0, .15)], [])],
                    new RingOffsetOptions(0, .5)
                    { MinimumRingArea = 0, RingLimit = 10, Tolerances = GeometryTolerances.Default with
                        { ArcTolerance = 1 / PathUtils.Scale, SimplifyTolerance = 0 } },
                    1, preserveTerminalPaths: true);
            }
            finally { Console.SetOut(previousOutput); }
            messages.AddRange(output.ToString().Split(Environment.NewLine));
        });
        return result!;
    }

    private static void WithReference(bool reference, Action action)
    {
        string? previous = Environment.GetEnvironmentVariable(ReferenceVariable);
        try
        {
            Environment.SetEnvironmentVariable(ReferenceVariable, reference ? "1" : "0");
            action();
        }
        finally { Environment.SetEnvironmentVariable(ReferenceVariable, previous); }
    }
}
