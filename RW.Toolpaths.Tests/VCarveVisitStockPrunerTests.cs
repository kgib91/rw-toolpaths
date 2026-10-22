using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveVisitStockPrunerTests
{
    [Fact]
    public void ClearedInteriorArcsAreRemovedBeforeConnectingTheRemainingWallWork()
    {
        var cut = Path(new(0, 0, -1), new(1, 0, -1), new(4, 0, -1), new(8, 0, -1),
            new(8, 8, -1), new(4, 8, -1), new(1, 8, -1), new(0, 8, -1), new(0, 0, -1));
        var rough = new VCarveRoughingChord(new(0, -5, -1), new(0, 13, -1), 3, 3);
        var arc = Assert.Single(Filter(cut, 0, rough));
        Assert.Equal(new Point3D(1, 0, -1), arc.Points[0]);
        Assert.Equal(new Point3D(1, 8, -1), arc.Points[^1]);
        Assert.Equal(cut.Points.Skip(1).Take(6), arc.Points);
        Assert.DoesNotContain(arc.Points, p => p.X == 0);
    }

    [Fact]
    public void AxisOnlyVisitsRetainTheirOriginalIncomingCursorCustody()
    {
        var cut = Path(new(0, 0, -1), new(0, 0, -2));
        var rough = new VCarveRoughingChord(new(0, 0, -3), new(1, 0, -3), 3, 3);
        Assert.Equal(cut, Assert.Single(Filter(cut, 0, rough)));
    }

    [Fact]
    public void AxisOnlyRefusalUsesTheRealFloatSurfaceAddition()
    {
        var cut = Path(new(0, 0, -2.99815), new(0, 0, -2.99915));
        var rough = new VCarveRoughingChord(new(0, 0, -4), new(1, 0, -4), 3, 3);
        var result = VCarveVisitStockPruner.Filter([cut], Profile, [rough], p => p, 0,
            default, surfaceZ: 12.7);
        Assert.Equal(cut, Assert.Single(result));
    }

    [Fact]
    public void SmallSlopedChordsCanUseARealWholeVolumeRoughingWitness()
    {
        var cut=Path(new(0,0,-1),new(.0008,0,-1.0016));
        var rough=new VCarveRoughingChord(new(0,0,-3),new(1,0,-3),3,3);
        Assert.Empty(Filter(cut,0,rough));
    }

    private static readonly VCarveCutterProfile Profile = new([
        new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
        new(CarvingCutterSectionKind.Linear, 1, 2, 5)], 6);
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, [], 0, "clearing", null, detectCurves: false);
    private static List<TaggedToolpath> Filter(TaggedToolpath path, double tolerance,
        params VCarveRoughingChord[] roughing)
        => VCarveVisitStockPruner.Filter([path], Profile, roughing, p => p, tolerance, default);

    [Fact]
    public void TinyFullVisitCanOnlyChangeTheStockByItsMaximumPostedDepth()
    {
        var path = Path(new(0, 0, -.001), new(10, 0, -.025));
        Assert.Empty(Filter(path, .025));
        Assert.Single(Filter(path, 0));
        Assert.Single(Filter(path with { Points = new[] { new Point3D(0, 0, -.001), new Point3D(10, 0, -.026) } }, .025));
    }

    [Fact]
    public void ADeepPointInsideAnOtherwiseShallowChainRetainsTheWholeVisit()
    {
        var path = Path(new(0, 0, -.001), new(1, 0, -.026), new(2, 0, -.001));
        Assert.Single(Filter(path, .025));
    }

    [Fact]
    public void EarlierFlatCutterClearsTheCompleteFinishingCutterVolume()
    {
        var cut = Path(new(0, 0, -1), new(4, 0, -1));
        var previous = new VCarveRoughingChord(new(0, 0, -2), new(4, 0, -2), 2, 2);
        Assert.Empty(Filter(cut, 0, previous));
        // Independent profile cross-sections at every sampled stock height,
        // with actual earlier flat poses covering each translated cut position.
        for (int i = 0; i <= 40; i++)
        for (int j = 0; j <= 20; j++)
        {
            double x = i / 10d, height = j / 20d;
            Assert.True(Profile.PhysicalRadiusAtDepth(height) < previous.TipRadius);
            Assert.True(-2 < -1 + height);
            Assert.InRange(x, previous.A.X, previous.B.X);
        }
    }

    [Fact]
    public void CoveredTipCannotHideAnUncutCutterFlank()
    {
        var cut = Path(new(.9, 0, -1), new(1, 0, -1));
        var prior = new VCarveRoughingChord(new(0, 0, -2), new(.1, 0, -2), 1, 1);
        Assert.Single(Filter(cut, .025, prior));
    }

    [Fact]
    public void AShallowerPreviousCutCannotSuppressADeeperDemand()
    {
        var cut = Path(new(0, 0, -1), new(1, 0, -1));
        var shallow = new VCarveRoughingChord(new(0, 0, -.95), new(1, 0, -.95), 3, 3);
        Assert.Single(Filter(cut, .025, shallow));
        var near = shallow with { A = shallow.A with { Z = -.98 }, B = shallow.B with { Z = -.98 } };
        Assert.Empty(Filter(cut, .025, near));
        Assert.Single(Filter(cut, 0, near));
    }

    [Fact]
    public void WholeChordIsCheckedIncludingTheGapBetweenClearedEnds()
    {
        var cut = Path(new(0, 0, -1), new(10, 0, -1));
        var left = new VCarveRoughingChord(new(0, 0, -2), new(.1, 0, -2), 2, 2);
        var right = new VCarveRoughingChord(new(10, 0, -2), new(10.1, 0, -2), 2, 2);
        Assert.Single(Filter(cut, .025, left, right));
    }

    [Fact]
    public void ExistingTabsAndNonfiniteCoordinatesDoNotAcquireOmissionAuthority()
    {
        var path = Path(new(0, 0, -.001), new(1, 0, -.001));
        Assert.Single(Filter(path with { Spans = new[] { new ToolpathSpan(0, 1, ToolpathSpanKind.TabLift) } }, .025));
        Assert.Single(Filter(Path(new(double.NaN, 0, -.001), new(1, 0, -.001)), .025));
    }

    [Fact]
    public void PostedDepthOwnsTheDecisionAndCancellationPropagates()
    {
        var path = Path(new(0, 0, -.024), new(1, 0, -.024));
        Assert.Single(VCarveVisitStockPruner.Filter([path], Profile, [], p => p with { Z = -.026 }, .025, default));
        using var cancelled = new CancellationTokenSource(); cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveVisitStockPruner.Filter([], Profile, [], p => p, .025, cancelled.Token));
    }

    [Fact]
    public void TipDiskWithoutAProvedStockHeightColumnCannotSuppressVolume()
    {
        var path = Path(new(0, 0, -1), new(1, 0, -1));
        var tipOnly = new VCarveRoughingChord(new(0, 0, -2), new(1, 0, -2), 3);
        Assert.Single(Filter(path, .025, tipOnly));
    }
}
