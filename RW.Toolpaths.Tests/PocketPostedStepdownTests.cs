using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class PocketPostedStepdownTests
{
    private readonly ITestOutputHelper _output;
    public PocketPostedStepdownTests(ITestOutputHelper output) => _output = output;

    [Fact]
    public void ActualRaw193348LinkHasAStockWitnessBeyondEveryOlderNearbyPostedCapsule()
    {
        // Candidate09b geometry SHA90e21af7c33c354abde5fb83f86fb967bfb8567cd9391f0f8ff92bf4944b2e2d.
        // These are ALL earlier posted chords within3.3mm of the independent
        // witness; every omitted earlier chord is farther than3.3mm. Full audit
        // refines a definite missing area1.462224e-5mm² at3.125e-8mm arc error.
        long[][] rows = [
            [190017,31816,245402,-3500,31851,245342,-3500],
            [190018,31851,245342,-3500,31890,245276,-3500],
            [190019,31890,245276,-3500,31842,245441,-3500],
            [190199,31816,245402,-3500,31851,245342,-3500],
            [190200,31851,245342,-3500,31890,245276,-3500],
            [190201,31890,245276,-3500,31842,245441,-3500],
            [190354,29534,241986,-3500,29656,241711,-3500],
            [190355,29656,241711,-3500,29856,241244,-3500],
            [190356,29856,241244,-3500,29973,240964,-3500],
            [190357,29973,240964,-3500,30169,240485,-3500],
            [190358,30169,240485,-3500,30412,239873,-3500],
            [190538,36064,241373,-3500,36058,242080,-3500],
            [190539,36058,242080,-3500,36040,242727,-3500],
            [190540,36040,242727,-3500,36025,243013,-3500],
            [190541,36025,243013,-3500,36000,243338,-3500],
            [192983,31890,245276,5000,31890,245276,-3500],
            [192984,31890,245276,-3500,31842,245441,-3586],
            [193164,31816,245402,-7000,31851,245342,-7000],
            [193165,31851,245342,-7000,31890,245276,-7000],
            [193166,31890,245276,-7000,31842,245441,-7000],
            [193346,31816,245402,-7000,31851,245342,-7000],
            [193347,31851,245342,-7000,31890,245276,-7000],
        ];
        const double radius = 3.175;
        var witness = new PointD(32.8761589, 242.2531386);
        var stock = new PocketPostedStepdownStock(new DepthSchedule(10.5, 3.5), radius, CancellationToken.None);
        foreach (var row in rows)
        {
            Point3D a = new(row[1] / 1000d, row[2] / 1000d, row[3] / 1000d);
            Point3D b = new(row[4] / 1000d, row[5] / 1000d, row[6] / 1000d);
            stock.Record([a, b]);
            // Vertical and sloped entries in this subset have the same whole
            // planar support; using the complete chord only enlarges evidence.
            Assert.True(Distance(witness, a, b) > radius, $"Earlier raw{row[0]} covers witness.");
        }
        Point3D from = new(31.890, 245.276, -7), to = new(28.803, 243.429, -7);
        Assert.True(Distance(witness, from, to) < radius);
        Assert.False(stock.Capture().IsSafe(from, to));
        _output.WriteLine($"actualRaw=193348 witness={witness} priorChords={rows.Length} "
            + $"minimumEarlierDistance={rows.Min(row => Distance(witness,
                new(row[1] / 1000d, row[2] / 1000d, 0), new(row[4] / 1000d, row[5] / 1000d, 0))):R}");
    }

    [Fact]
    public void OmittedPlanarRampAxesCarryTheRealCursorAcrossSpansIntoTheNextShortCut()
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(14, 3.5), .1,
            CancellationToken.None, permitFlatFootprintProof: false);
        Point3D a = new(.00049, .00049, -6.995), b = new(.00124, .00049, -6.9965);
        Point3D c = new(.00124, .00124, -6.998), d = new(.00199, .00124, -6.998);
        // Slope2 over .00075mm native edges triggers Shared Y-only commands.
        // The nominal end rounds to(1,1), but the machine still stands at(0,0).
        stock.Record([a, b, c]);
        var omitted = stock.Capture();
        Assert.False(omitted.IsSafe(c, d));
        // A contiguous SECOND span emits XYZ to(2,1); its start remains(0,0),
        // not the omitted nominal vertex(1,1).
        stock.Record([c, d]);
        var recorded = stock.Capture();
        Assert.True(recorded.IsSafe(new(0, 0, -10.498), new(.002, .001, -10.498)));
        Assert.False(recorded.IsSafe(new(.001, .001, -10.498), new(.002, .001, -10.498)));
    }

    [Fact]
    public void HeightOnlyVisitResetCarriesActualPlanarCursorUntilAnActualXYZCommand()
    {
        // Independent actual-reader witness: posted-reset-review-15b.json.
        // The first steep edge is Y-only, as are the next visit's safe-plane
        // retract/approach because its nominal XY equals the old nominal end.
        var stock = new PocketPostedStepdownStock(new DepthSchedule(4, 1), .1,
            CancellationToken.None, permitFlatFootprintProof: false);
        stock.Record([new(.00049, 0, -1), new(.00124, 0, -2)]);
        stock.Record([new(.00124, 0, -1), new(.00124, 0, -2)]);
        var prior = stock.Capture();
        Assert.True(prior.IsSafe(new(0, 0, -3), new(0, 0, -3)));
        Assert.False(prior.IsSafe(new(.001, 0, -3), new(.001, 0, -3)));
        Assert.Equal(1, stock.HeightOnlyCursorCarries);
        Assert.Equal(0, stock.PlanarBridgeCursorAnchors);
        // Although both nominal endpoints F3-round toX1, dy=0 posts XYZ and
        // the actual following Cut travelsX0->1. Credit exactly that chord.
        stock.Record([new(.00124, 0, -3), new(.0014, 0, -3)]);
        Assert.True(stock.Capture().IsSafe(new(0, 0, -4), new(.001, 0, -4)));
        Assert.False(prior.IsSafe(new(.001, 0, -3), new(.001, 0, -3)));
    }

    [Theory]
    [InlineData(.000075, .000075, false)]
    [InlineData(.00011, 0, true)]
    [InlineData(0, .00011, true)]
    public void SafePlaneBridgeUsesIndividualNominalAxisThresholdsAndStillPostsShortXYZTraverses(
        double shiftX, double shiftY, bool emitsTraverse)
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(3, 1), .1,
            CancellationToken.None, permitFlatFootprintProof: false);
        stock.Record([new(.00049, .00049, -1), new(.00118, .00118, -2)]);
        double x = .00118 + shiftX, y = .00118 + shiftY;
        // The diagonal .000075/.000075 change exceeds1e-4 in norm but not
        // in either axis. Conversely .00011 emits a safe-plane XYZ bridge
        // despite a total run much smaller than the vertical-motion threshold.
        stock.Record([new(x, y, -1), new(x, y, -2)]);
        var proof = stock.Capture();
        Assert.Equal(emitsTraverse, proof.IsSafe(new(.001, .001, -3), new(.001, .001, -3)));
        Assert.Equal(emitsTraverse ? 1 : 0, stock.PlanarBridgeCursorAnchors);
        Assert.Equal(emitsTraverse ? 0 : 1, stock.HeightOnlyCursorCarries);
    }

    [Fact]
    public void PhysicalCylinderQualificationUsesItsExactRadiusAndCompleteCuttingHeight()
    {
        var full = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 3.175, 3.175, 70)], 70);
        var tool = new ToolGeometry((float)3.175) { VCarveProfile = full };
        Assert.True(PocketContinuationRouter.TryFlatCylinderRadius(tool, 12.7, out double radius));
        Assert.Equal(3.175, radius);
        Assert.NotEqual(tool.Radius, radius);
        var shortTool = tool with { VCarveProfile = new VCarveCutterProfile(
            [new(CarvingCutterSectionKind.Linear, 3.175, 3.175, 3)], 3) };
        Assert.False(PocketContinuationRouter.TryFlatCylinderRadius(shortTool, 7, out _));
        var rounded = tool with { VCarveProfile = new VCarveCutterProfile(
            [new(CarvingCutterSectionKind.Hemisphere, 0, 3.175, 3.175)], 3.175) };
        Assert.False(PocketContinuationRouter.TryFlatCylinderRadius(rounded, 3, out _));
    }

    [Fact]
    public void EngagedPocketWidthOnlyAuthorizesStockFromALiteralCylinder()
    {
        var sections = new CarvingCutterSection[] {
            new(CarvingCutterSectionKind.Linear, .75, .75, 20),
            new(CarvingCutterSectionKind.Linear, .793, 3, 10),
            new(CarvingCutterSectionKind.Linear, 3, 3, 40) };
        var tool = new ToolGeometry(.75) { PhysicalProfile = new(sections, 5) };
        Assert.True(PocketContinuationRouter.TryFlatCylinderRadius(tool, 5.001, out double radius));
        Assert.Equal(.75, radius);
        Assert.False(PocketContinuationRouter.TryFlatCylinderRadius(tool with { Radius = 3 }, 5, out _));
        Assert.False(PocketContinuationRouter.TryFlatCylinderRadius(tool, 20, out _));
        var taper = new ToolGeometry(1) { PhysicalProfile = new([
            new(CarvingCutterSectionKind.Linear, .5, 3, 10)], 2) };
        Assert.False(PocketContinuationRouter.TryFlatCylinderRadius(taper, 2, out _));
        var shortTool = tool with { PhysicalProfile = new([
            new(CarvingCutterSectionKind.Linear, .75, .75, 2)], 2) };
        Assert.False(PocketContinuationRouter.TryFlatCylinderRadius(shortTool, 5, out _));
        var narrowShank = tool with { PhysicalProfile = new([
            new(CarvingCutterSectionKind.Linear, .75, .75, 2),
            new(CarvingCutterSectionKind.Linear, .3, .3, 10)], 5) };
        Assert.False(PocketContinuationRouter.TryFlatCylinderRadius(narrowShank, 5, out _));
    }

    [Fact]
    public void PostedOriginalChordRetracesHaveZeroLateralStockCreditAcrossDepthsAndOldSnapshots()
    {
        var depth = new DepthSchedule(10.5, 3.5) { SurfaceZ = 12.7 };
        var stock = new PocketPostedStepdownStock(depth, .5, CancellationToken.None);
        Point3D A(double z, double normal = 0) => new(10.0004998, 11.0015002 + normal, depth.SurfaceZ + z);
        Point3D B(double z, double normal = 0) => new(31.8015002, 23.3134998 + normal, depth.SurfaceZ + z);
        stock.Record([A(-3.5), B(-3.5)]);
        var captured = stock.Capture();
        Assert.True(captured.IsSafe(A(-7), B(-7)));
        Assert.False(captured.IsSafe(A(-7, .001), B(-7, .001)));
        Assert.False(captured.IsSafe(A(-7.001), B(-7.001)));
        stock.Record([A(-7, .001), B(-7, .001)]);
        Assert.False(captured.IsSafe(A(-10.5, .001), B(-10.5, .001)));
        Assert.True(stock.Capture().IsSafe(A(-10.5, .001), B(-10.5, .001)));
        Assert.True(stock.ExactProofs >= 2);
        Assert.True(stock.Refusals >= 3);
        _output.WriteLine($"exact={stock.ExactProofs} refused={stock.Refusals} physical={stock.PhysicalProofs}");
    }

    [Fact]
    public void FollowingCutAndLinkConstructionReserveNeverAdmitPriorStepdownStock()
    {
        var depth = new DepthSchedule(7, 3.5);
        var earlier = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        earlier.Record([new(0, 0, -3.5), new(10, 0, -3.5)]);
        var posted = new PocketPostedStepdownStock(depth, .5, CancellationToken.None);
        posted.Record([new(0, 0, -3.5), new(10, 0, -3.5)]);
        IReadOnlyList<PointD>[] following = [new PointD[] { new(1, .001), new(9, .001) }];
        Point3D from = new(1, .001, -7), to = new(9, .001, -7);
        var clearanceOnly = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            earlier, -7, following, .5, CancellationToken.None);
        Assert.True(clearanceOnly.IsTravelSafe(from, to, .5));
        var complete = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            earlier, -7, following, .5, CancellationToken.None, priorStepdown: posted.Capture());
        Assert.False(complete.IsTravelSafe(from, to, .5));
        // An actual missing-stock witness belongs to the new capsule and lies
        // strictly beyond every older radius-.5 cutter pose. No RDP/.001 or
        // LINK .003122 allowance can turn it into executed prior-floor stock.
        PointD witness = new(5, .5005);
        Assert.True(Math.Abs(witness.y - .001) < .5);
        Assert.True(Math.Abs(witness.y) > .5);
    }

    [Fact]
    public void FlatCylinderCanUseCompleteOverlappingEarlierCapsulesAndClippedRamps()
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5), 1, CancellationToken.None);
        stock.Record([new(0, -.2, -3.5), new(10, -.2, -3.5)]);
        stock.Record([new(0, .2, -3.5), new(10, .2, -3.5)]);
        Assert.True(stock.Capture().IsSafe(new(2, 0, -7), new(8, 0, -7)));
        Assert.Equal(1, stock.PhysicalProofs);
        var ramp = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5), .5, CancellationToken.None);
        ramp.Record([new(0, 0, 0), new(10, 0, -7)]);
        Assert.True(ramp.Capture().IsSafe(new(7, 0, -7), new(8, 0, -7)));
        Assert.False(ramp.Capture().IsSafe(new(1, 0, -7), new(2, 0, -7)));
    }

    [Fact]
    public void RoundedOrConicalAuthorityRequiresActualSamePostedChordAndNeverAFlatFootprintShortcut()
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5), 1,
            CancellationToken.None, permitFlatFootprintProof: false);
        stock.Record([new(0, -.2, -3.5), new(10, -.2, -3.5)]);
        stock.Record([new(0, .2, -3.5), new(10, .2, -3.5)]);
        var proof = stock.Capture();
        Assert.False(proof.IsSafe(new(2, 0, -7), new(8, 0, -7)));
        Assert.True(proof.IsSafe(new(2, .2, -7), new(8, .2, -7)));
        Assert.Equal(0, stock.InnerProofBuilds);
    }

    [Fact]
    public void SurfaceAndExplicitStartingFloorEntitlementsRespectPostedStepdownAndCancellation()
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5) { SurfaceZ = 12.7 },
            .5, CancellationToken.None);
        Assert.True(stock.Capture().IsSafe(new(0, 0, 9.2), new(10, 0, 9.2)));
        Assert.False(stock.Capture().IsSafe(new(0, 0, 9.199), new(10, 0, 9.199)));
        var cleared = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5) { StartDepth = 2 },
            .5, CancellationToken.None);
        Assert.True(cleared.Capture().IsSafe(new(0, 0, -5.5), new(10, 0, -5.5)));
        Assert.False(cleared.Capture().IsSafe(new(0, 0, -5.501), new(10, 0, -5.501)));
        using var canceled = new CancellationTokenSource();
        var interrupted = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5), .5, canceled.Token);
        canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => interrupted.Record([new(0, 0, -3.5), new(10, 0, -3.5)]));
        Assert.Throws<OperationCanceledException>(() => interrupted.Capture().IsSafe(new(0, 0, -7), new(10, 0, -7)));
    }

    [Fact]
    public void ImmutableBatchBoundaryRetainsCapturedPrefixAfterLaterStockIsPublished()
    {
        var stock = new PocketPostedStepdownStock(new DepthSchedule(7, 3.5), .5, CancellationToken.None);
        for (int i = 0; i < 2047; i++)
            stock.Record([new(i * 20, 0, -3.5), new(i * 20 + 10, 0, -3.5)]);
        var captured = stock.Capture();
        stock.Record([new(2, 2, -3.5), new(8, 2, -3.5)]);
        Assert.True(captured.IsSafe(new(2, 0, -7), new(8, 0, -7)));
        Assert.False(captured.IsSafe(new(2, 2, -7), new(8, 2, -7)));
        Assert.True(stock.Capture().IsSafe(new(2, 2, -7), new(8, 2, -7)));
    }

    private static double Distance(PointD p, Point3D a, Point3D b)
    {
        double x = b.X - a.X, y = b.Y - a.Y, denominator = x * x + y * y;
        double t = denominator == 0 ? 0 : Math.Clamp(((p.x - a.X) * x + (p.y - a.Y) * y) / denominator, 0, 1);
        return Math.Sqrt(Math.Pow(p.x - a.X - t * x, 2) + Math.Pow(p.y - a.Y - t * y, 2));
    }
}
