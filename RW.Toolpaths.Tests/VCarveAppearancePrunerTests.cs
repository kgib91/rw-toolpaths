using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveAppearancePrunerTests
{
    private static readonly VCarveCutterProfile Profile = new([
        new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)], 1);
    private static TaggedToolpath Visit(double depth) => new([
        new Point3D(0, 0, -depth), new Point3D(.1, 0, -depth)], [], 0, "clearing", null, detectCurves: false);

    [Fact]
    public void PhysicalInspectionScaleUsesTheClosestStatedViewingDistance()
    {
        Assert.Equal(152.4, VCarveAppearancePruner.InspectionDistanceMm);
        Assert.InRange(VCarveAppearancePruner.PixelPitch, .022, .023);
    }

    [Fact]
    public void ShallowSharpMarksAreRetainedEvenWhenTheirDepthIsBelowCleanupTolerance()
    {
        const int size = 51;
        var flat = new double[size * size];
        var smoothOffset = Enumerable.Repeat(.01, flat.Length).ToArray();
        var groove = new double[flat.Length];
        for (int y = 0; y < size; y++)
        for (int x = 0; x < size; x++)
            groove[y * size + x] = .01 * Math.Exp(-Math.Pow((x - size / 2) * VCarveAppearancePruner.PixelPitch / .03, 2));
        Assert.False(VCarveAppearancePruner.Compare(flat, smoothOffset, size, size, VCarveAppearancePruner.PixelPitch).Visible);
        var score = VCarveAppearancePruner.Compare(flat, groove, size, size, VCarveAppearancePruner.PixelPitch);
        Assert.True(score.Visible);
        Assert.True(score.MaximumShadeDifference > .035);
    }

    [Fact]
    public void EqualVerticalChangesCanHaveDifferentSurfaceDisplacements()
    {
        const int size = 25;
        var a = new double[size * size]; var b = new double[a.Length];
        for (int y = 0; y < size; y++)
        for (int x = 0; x < size; x++)
        {
            a[y * size + x] = 10 * x * VCarveAppearancePruner.PixelPitch;
            b[y * size + x] = a[y * size + x] + .06;
        }
        Assert.False(VCarveAppearancePruner.Compare(a, b, size, size, VCarveAppearancePruner.PixelPitch).Visible);
        Assert.True(VCarveAppearancePruner.Compare(new double[a.Length], Enumerable.Repeat(.06, a.Length).ToArray(),
            size, size, VCarveAppearancePruner.PixelPitch).Visible);
    }

    [Fact]
    public void DuplicateVisitsCannotAllDisappearThroughIncrementalComparison()
    {
        var cut = Visit(.2);
        var result = VCarveAppearancePruner.Filter([cut, cut], Profile, [], p => p, (_, _) => 100, default);
        Assert.Single(result);
        Assert.Equal(cut, result[0]);
    }

    [Fact]
    public void FullCutterSurfaceMakesTinyMarksRemovableButVisibleGroovesRemain()
    {
        Assert.Empty(VCarveAppearancePruner.Filter([Visit(.0001)], Profile, [], p => p, (_, _) => 100, default));
        Assert.Single(VCarveAppearancePruner.Filter([Visit(.2)], Profile, [], p => p, (_, _) => 100, default));
    }

    [Fact]
    public void ContourGuardUsesActualStockChangeRatherThanFootprintProximity()
    {
        Assert.Empty(VCarveAppearancePruner.Filter([Visit(.0001)], Profile, [], p => p, (_, _) => 0, default));
        Assert.Single(VCarveAppearancePruner.Filter([Visit(.02)], Profile, [], p => p, (_, _) => 0, default));
    }

    [Fact]
    public void ActualCommandsOwnTheScoreAndCancellationPropagates()
    {
        Assert.Single(VCarveAppearancePruner.Filter([Visit(.0001)], Profile, [], p => p with { Z = -.2 }, (_, _) => 100, default));
        using var cancel = new CancellationTokenSource(); cancel.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarveAppearancePruner.Filter([], Profile, [], p => p, (_, _) => 100, cancel.Token));
    }

    [Fact]
    public void IsolatedPlungesAndUnrepresentedTabMotionCannotDisappear()
    {
        var plunge = Visit(.5) with { Points = new[] { new Point3D(0, 0, -.1), new Point3D(0, 0, -.5) } };
        Assert.Single(VCarveAppearancePruner.Filter([plunge], Profile, [], p => p, (_, _) => 100, default));
        var tab = Visit(.0001) with { Spans = new[] { new ToolpathSpan(0, 1, ToolpathSpanKind.TabLift) } };
        Assert.Single(VCarveAppearancePruner.Filter([tab], Profile, [], p => p, (_, _) => 100, default));
    }

    [Fact]
    public void AClearanceColumnWithoutItsActualStockSurfaceCannotAuthorizeAppearanceOmissions()
    {
        var prior = new VCarveRoughingChord(new(0, 0, -1), new(1, 0, -1), 2, 1);
        Assert.Single(VCarveAppearancePruner.Filter([Visit(.0001)], Profile, [prior], p => p, (_, _) => 100, default));
        Assert.Empty(VCarveAppearancePruner.Filter([Visit(.02)], Profile,
            [prior with { StockSurfaceRadius = 2 }], p => p, (_, _) => 100, default));
        var invalid = Visit(.0001) with { Points = new[] { new Point3D(double.NaN, 0, 0), new Point3D(1, 0, 0) } };
        Assert.Single(VCarveAppearancePruner.Filter([invalid], Profile, [], p => p, (_, _) => 100, default));
    }

    [Fact]
    public void DeepNarrowVisitsAreEvaluatedAcrossTheirCompleteCutterFootprint()
    {
        var wide = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 2, 2)], 2);
        var cut = Visit(1.5);
        var logs = new List<string>();
        var result = VCarveAppearancePruner.Filter([cut, cut, cut], wide, [], p => p, (_, _) => 100, default, logs.Add);
        Assert.Single(result);
        Assert.Contains(logs, log => log.Contains("removedVisits=2 "));
        Assert.DoesNotContain(logs, log => log.Contains("reason=no-eligible-local-patches"));
    }

    [Fact]
    public void LocalFinishedFormCanAcceptMicrotextureWithoutAcceptingBroadShapeLoss()
    {
        const int size = 61;
        var flat = new double[size * size]; var fine = new double[flat.Length];
        var broad = new double[flat.Length];
        for (int y = 0; y < size; y++)
        for (int x = 0; x < size; x++)
        {
            double xx = (x - size / 2) * VCarveAppearancePruner.PixelPitch;
            double yy = (y - size / 2) * VCarveAppearancePruner.PixelPitch;
            fine[y * size + x] = .03 * Math.Exp(-(xx * xx + yy * yy) / (.03 * .03));
            broad[y * size + x] = .3 * Math.Exp(-(xx * xx + yy * yy) / (.3 * .3));
        }
        Assert.True(VCarveAppearancePruner.Compare(flat, fine, size, size, VCarveAppearancePruner.PixelPitch).Visible);
        Assert.False(VCarveAppearancePruner.Compare(flat, fine, size, size, VCarveAppearancePruner.PixelPitch, false).Visible);
        Assert.True(VCarveAppearancePruner.Compare(flat, broad, size, size, VCarveAppearancePruner.PixelPitch, false).Visible);
        Assert.True(VCarveAppearancePruner.Compare(flat, Enumerable.Repeat(.1, flat.Length).ToArray(),
            size, size, VCarveAppearancePruner.PixelPitch, false).Visible);
    }

    [Fact]
    public void ClearingAppearanceAllowanceDoesNotWeakenFinalArtworkDetail()
    {
        var cut = Visit(.005);
        Assert.Empty(VCarveAppearancePruner.Filter([cut], Profile, [], p => p, (_, _) => 100, default));
        Assert.Single(VCarveAppearancePruner.Filter([cut with { Category = "final-carve" }],
            Profile, [], p => p, (_, _) => 100, default));
    }
}
