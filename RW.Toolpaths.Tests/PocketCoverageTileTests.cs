using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketCoverageTileTests
{
    [Fact]
    public void AlternatingTilesReuseOnlyTheirCapturedStockAndStillCheckCurrentSpatialPermission()
    {
        var stock=LineStock();
        for(int i=0;i<200;i++)stock.Record([new(1000+20*i,0,-1),new(1010+20*i,0,-1)]);
        var diagnostics=new PocketContinuationRouter.CoverageDiagnostics();
        var spatial=new MutableSpatial();
        var proof=new PocketContinuationRouter.TemporalClearance(spatial,stock,-1,
            Array.Empty<IReadOnlyList<PointD>>(),.5,default,diagnostics,validateIndexedCoverage:true);
        var reference=new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,stock,-1,
            Array.Empty<IReadOnlyList<PointD>>(),.5,default,validateIndexedCoverage:true,useLocalTileCache:false);
        for(int pass=0;pass<4;pass++)
        foreach(double x in new[]{2d,10d,20d,30d})
        {
            var a=new Point3D(x,0,-1);var b=new Point3D(x+1,0,-1);
            Assert.True(reference.IsTravelSafe(a,b,.5));Assert.True(proof.IsTravelSafe(a,b,.5));
        }
        Assert.Equal(4,diagnostics.LocalIndexBuilds);
        Assert.True(diagnostics.LocalTileCacheHits>=12);
        Assert.InRange(diagnostics.LocalTileCachePeakEstimatedBytes,1,8L*1024*1024);
        stock.Record([new(40,0,-1),new(60,0,-1)]);stock.AtFloor(-1);
        Assert.False(proof.IsTravelSafe(new(45,0,-1),new(50,0,-1),.5));
        spatial.Allow=false;
        Assert.False(proof.IsTravelSafe(new(2,0,-1),new(3,0,-1),.5));
        Assert.Equal(0,diagnostics.LocalProofMismatches);
    }

    private sealed class MutableSpatial:ILinkClearance
    {
        internal bool Allow=true;
        public bool IsTravelSafe(Point3D from,Point3D to,double radius)=>Allow;
    }

    [Theory]
    [InlineData(.5, 65536)]
    [InlineData(3.175, 524288)]
    [InlineData(6.35, 1048576)]
    public void IntegerTilesGrowOutwardAndInsetEligibilityByTheCompleteUnchangedGuard(double radius, long ticks)
    {
        Assert.True(PocketContinuationRouter.TemporalClearance.TryExpandCoverageTile(-101, -17, 149, 29, radius, out var tile));
        Assert.Equal(ticks, tile.TileTicks);
        Assert.True(tile.MinX <= -101 && tile.MinY <= -17 && tile.MaxX >= 149 && tile.MaxY >= 29);
        Assert.Equal(0, tile.MinX % ticks); Assert.Equal(0, tile.MinY % ticks);
        Assert.Equal(0, tile.MaxX % ticks); Assert.Equal(0, tile.MaxY % ticks);
        long guard = PocketContinuationRouter.TemporalClearance.CoverageTileGuardTicks;
        Assert.Equal(444, guard); // Complete current construction + crop reserves.
        Assert.Equal((tile.MinX + guard) / PathUtils.Scale, tile.Eligible.MinX);
        Assert.Equal((tile.MinY + guard) / PathUtils.Scale, tile.Eligible.MinY);
        Assert.Equal((tile.MaxX - guard) / PathUtils.Scale, tile.Eligible.MaxX);
        Assert.Equal((tile.MaxY - guard) / PathUtils.Scale, tile.Eligible.MaxY);
    }

    [Theory]
    [InlineData(-65537, -65536, -131072, -65536)]
    [InlineData(-1, 0, -65536, 0)]
    [InlineData(0, 65537, 0, 131072)]
    public void NegativeAndExactGridEdgesUseMathematicalOutwardRounding(long min, long max, long expectedMin, long expectedMax)
    {
        Assert.True(PocketContinuationRouter.TemporalClearance.TryExpandCoverageTile(min, -7, max, 13, .5, out var tile));
        Assert.Equal(expectedMin, tile.MinX);
        Assert.Equal(expectedMax, tile.MaxX);
    }

    [Fact]
    public void UnsupportedTileArithmeticKeepsTheOriginalPreciseRangeFallbackAvailable()
    {
        Assert.False(PocketContinuationRouter.TemporalClearance.TryExpandCoverageTile(999999999999, 0, 1000000000000, 1, 3.175, out _));
        Assert.False(PocketContinuationRouter.TemporalClearance.TryExpandCoverageTile(-1000000000000, 0, -999999999999, 1, 3.175, out _));
        Assert.False(PocketContinuationRouter.TemporalClearance.TryExpandCoverageTile(0, 0, 100, 100, double.NaN, out _));
        Assert.False(PocketContinuationRouter.TemporalClearance.TryExpandCoverageTile(0, 0, 100, 100, double.PositiveInfinity, out _));
        Assert.False(PocketContinuationRouter.TemporalClearance.TryExpandCoverageTile(0, 0, 100, 100, 0, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FourthRepeatedSmallCropAvoidsTheConstructionBudgetWithoutChangingAnyWholeCapsuleDecision(bool negative)
    {
        var stock = LineStock(negative);
        var tiled = new PocketContinuationRouter.CoverageDiagnostics();
        var tight = new PocketContinuationRouter.CoverageDiagnostics();
        var candidate = Proof(stock, tiled);
        var reference = Proof(stock, tight, tiles: false);
        Point3D Point(double x) => new(negative ? -x : x, 0, -1);
        foreach (double x in new[] { 2d, 3d, 4d, 5d, 5.25 })
        {
            bool expected = reference.IsTravelSafe(Point(x), Point(x + 1), .5);
            Assert.True(expected);
            Assert.Equal(expected, candidate.IsTravelSafe(Point(x), Point(x + 1), .5));
        }
        Assert.Equal(3, tiled.CoverageTileSmallDomainSkips);
        Assert.Equal(1, tiled.CoverageTileRepeatedQualifications);
        Assert.Equal(1, tiled.CoverageTileSnaps);
        Assert.Equal(1, tiled.CoverageTileReuses);
        Assert.Equal(4, tiled.LocalIndexBuilds);
        Assert.Equal(0, tiled.CropWorkBudgetFallbacks);
        Assert.Equal(0, tiled.WholeFallbacks);
        Assert.Equal(1, tight.CropWorkBudgetFallbacks);
        Assert.Equal(0, tight.WholeFallbacks);
        Assert.True(tight.UndilatedProofs>0);
        Assert.True(tiled.CoverageTileCropArea >= tiled.CoverageTileTightArea);
        Assert.True(tiled.CoverageTileSubmittedVertices > 0);
        Assert.True(tiled.CoverageTileBoundsMilliseconds <= tiled.LocalConstructionMilliseconds + .0001);
    }

    [Fact]
    public void DenseCapturedDomainQualifiesItsFirstCropWhilePruningCompleteDistantContours()
    {
        var stock = LineStock();
        for (int index = 0; index < 200; index++)
            stock.Record([new(1000 + 20 * index, 0, -1), new(1010 + 20 * index, 0, -1)]);
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var proof = Proof(stock, diagnostics);
        Assert.True(proof.IsTravelSafe(new(2, 0, -1), new(3, 0, -1), .5));
        Assert.True(diagnostics.ContourBoundsInputVertices > 8192);
        Assert.Equal(1, diagnostics.CoverageTileSnaps);
        Assert.Equal(0, diagnostics.CoverageTileRepeatedQualifications);
        Assert.Equal(0, diagnostics.CoverageTileSmallDomainSkips);
        Assert.True(diagnostics.ContourBoundsSubmittedVertices < diagnostics.ContourBoundsConsideredVertices / 20);
        Assert.Equal(0, diagnostics.WholeFallbacks);
        Assert.Equal(1, diagnostics.ValidatedLocalProofs);
    }

    [Fact]
    public void CachedJointNonZeroTileRefusesTheExecutedHoleAndKeepsOnlyItsAuthorizedFollowingIsland()
    {
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        stock.Record([new(0, 0, -1), new(4, 0, -1), new(4, 4, -1), new(0, 4, -1), new(0, 0, -1)]);
        for (int index = 0; index < 200; index++)
            stock.Record([new(1000 + 20 * index, 0, -1), new(1010 + 20 * index, 0, -1)]);
        IReadOnlyList<PointD>[] following = [new PointD[] { new(2.25, 2.5), new(2.75, 2.5) }];
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var proof = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted, stock, -1,
            following, .5, CancellationToken.None, diagnostics, validateIndexedCoverage: true);
        Assert.True(proof.IsTravelSafe(new(.8, 0, -1), new(1.2, 0, -1), .5));
        Assert.Equal(1, diagnostics.CoverageTileSnaps);
        // The complete second capsule lies inside that tile's eligibility
        // window. Its negative hole must survive the cached joint NonZero crop,
        // before the unchanged whole proof independently refuses it too.
        Assert.False(proof.IsTravelSafe(new(1, 1, -1), new(1, 1, -1), .5));
        Assert.Equal(1, diagnostics.CoverageTileReuses);
        Assert.Equal(1, diagnostics.LocalIndexBuilds);
        Assert.Equal(0, diagnostics.WholeFallbacks);
        Assert.True(diagnostics.NegativeWitnesses>0);
        Assert.True(proof.IsTravelSafe(new(2.3, 2.5, -1), new(2.7, 2.5, -1), .5));
        Assert.False(proof.IsTravelSafe(new(1, 2.5, -1), new(3, 2.5, -1), .5));
        Assert.Equal(0, diagnostics.LocalProofMismatches);
    }

    [Fact]
    public void CachedTileNeverCreditsLaterExecutedStockOrADeeperFloor()
    {
        var stock = LineStock();
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var older = Proof(stock, diagnostics);
        foreach (double x in new[] { 2d, 3d, 4d, 5d })
            Assert.True(older.IsTravelSafe(new(x, 0, -1), new(x + 1, 0, -1), .5));
        stock.Record([new(40, 0, -1), new(60, 0, -1)]);
        stock.AtFloor(-1);
        Assert.False(older.IsTravelSafe(new(45, 0, -1), new(50, 0, -1), .5));
        Assert.True(Proof(stock, new()).IsTravelSafe(new(45, 0, -1), new(50, 0, -1), .5));
        var deeper = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted, stock, -2,
            Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None, validateIndexedCoverage: true);
        Assert.False(deeper.IsTravelSafe(new(5, 0, -2), new(6, 0, -2), .5));
        Assert.Equal(0, diagnostics.LocalProofMismatches);
    }

    [Fact]
    public void CancellationStillPrecedesACachedTileApproval()
    {
        using var canceled = new CancellationTokenSource();
        var stock = LineStock(token: canceled.Token);
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var proof = Proof(stock, diagnostics, token: canceled.Token);
        foreach (double x in new[] { 2d, 3d, 4d, 5d })
            Assert.True(proof.IsTravelSafe(new(x, 0, -1), new(x + 1, 0, -1), .5));
        Assert.Equal(1, diagnostics.CoverageTileSnaps);
        canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => proof.IsTravelSafe(new(5.25, 0, -1), new(6.25, 0, -1), .5));
    }

    private static PocketContinuationRouter.ClearedStock LineStock(bool negative = false, CancellationToken token = default)
    {
        var stock = new PocketContinuationRouter.ClearedStock(.5, token);
        stock.Record([new(0, 0, -1), new(negative ? -40 : 40, 0, -1)]);
        return stock;
    }
    private static PocketContinuationRouter.TemporalClearance Proof(PocketContinuationRouter.ClearedStock stock,
        PocketContinuationRouter.CoverageDiagnostics diagnostics, bool tiles = true, CancellationToken token = default)
        => new(LinkClearance.Unrestricted, stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, token,
            diagnostics, validateIndexedCoverage: true, useCoverageTiles: tiles);
}
