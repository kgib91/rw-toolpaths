using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class ConditionedReferenceOpeningControls
{
    [Fact]
    public void SmallAndUnsupportedInputRetainWholeLiteralOpening()
    {
        var source = new List<List<PointD>> {
            new() { new(0,0),new(4,0),new(4,4),new(0,4) },
            new() { new(1,1),new(1,3),new(3,3),new(3,1) } };
        foreach (double radius in new[] { .056338042245, .0001 })
        {
            var expected = PolygonOps.Intersect(source, VCarveSmoothing.ReachableReferenceUpper(source, radius));
            var actual = VCarveConditionedReferenceOpeningGoal.BuildMaskedGoal(source, radius,
                CancellationToken.None, null, out bool applied);
            Assert.False(applied); Exact(expected, actual);
        }
        var shifted = source.Select(r => r.Select(p => new PointD(p.x + 17000, p.y)).ToList()).ToList();
        var shiftedExpected = PolygonOps.Intersect(shifted, VCarveSmoothing.ReachableReferenceUpper(shifted, .056338042245));
        var shiftedActual = VCarveConditionedReferenceOpeningGoal.BuildMaskedGoal(shifted, .056338042245,
            CancellationToken.None, null, out bool shiftedApplied);
        Assert.False(shiftedApplied); Exact(shiftedExpected, shiftedActual);
    }

    [Fact]
    public void DenseSourceConditionsButOriginalHoleAndInputRemainOwned()
    {
        var source = Dense(); var snapshot = source.Select(r => r.ToList()).ToList();
        var goal = VCarveConditionedReferenceOpeningGoal.BuildMaskedGoal(source, .056338042245,
            CancellationToken.None, null, out bool applied);
        Assert.True(applied); Assert.NotEmpty(goal); Exact(snapshot, source);
        var originalNative = PathUtils.ToClipper(source); var goalNative = PathUtils.ToClipper(goal);
        Assert.Empty(Clipper.Difference(goalNative, originalNative, FillRule.NonZero));
        var holeInterior = new Paths64 { new() { new(24576,24576),new(40960,24576),new(40960,40960),new(24576,40960) } };
        Assert.Empty(Clipper.Intersect(goalNative, holeInterior, FillRule.NonZero));
        Assert.True(goal.Sum(r => (long)r.Count) < source.Sum(r => (long)r.Count));
        // This verifies operand/mask ownership; it supplies no physical or
        // stock certificate and makes no old-opening reproduction assertion.
    }

    [Fact]
    public void CancellationBeforeEntryOrPublicationReturnsNoGoal()
    {
        using var before = new CancellationTokenSource(); before.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(() =>
            VCarveConditionedReferenceOpeningGoal.BuildMaskedGoal(Dense(), .056338042245,
                before.Token, null, out _));
        using var publish = new CancellationTokenSource(); bool observed = false;
        Assert.ThrowsAny<OperationCanceledException>(() =>
            VCarveConditionedReferenceOpeningGoal.BuildMaskedGoal(Dense(), .056338042245,
                publish.Token, line => { observed = true; publish.Cancel(); }, out _));
        Assert.True(observed);
    }

    private static List<List<PointD>> Dense()
    {
        var outer = new List<PointD>(); const int steps = 2048;
        for (int i=0;i<steps;i++) outer.Add(new(i/(double)steps*2,0));
        for (int i=0;i<steps;i++) outer.Add(new(2,i/(double)steps*2));
        for (int i=0;i<steps;i++) outer.Add(new(2-i/(double)steps*2,2));
        for (int i=0;i<steps;i++) outer.Add(new(0,2-i/(double)steps*2));
        return new() { outer, new() { new(.5,.5),new(.5,1.5),new(1.5,1.5),new(1.5,.5) } };
    }
    private static void Exact(IReadOnlyList<List<PointD>> expected,IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count,actual.Count);
        for(int r=0;r<expected.Count;r++)
        {
            Assert.Equal(expected[r].Count,actual[r].Count);
            for(int p=0;p<expected[r].Count;p++)
            {
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected[r][p].x),BitConverter.DoubleToInt64Bits(actual[r][p].x));
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected[r][p].y),BitConverter.DoubleToInt64Bits(actual[r][p].y));
            }
        }
    }
}
