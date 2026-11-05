using System.Collections;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketCoverageCropTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IntegerRectCropPreservesTheJointSignedFillIncludingEnclosingHoles(bool mirror)
    {
        PointD[] Circle(double radius,int count)=>Enumerable.Range(0,count).Select(i=>
            new PointD(radius*Math.Cos(2*Math.PI*i/count),radius*Math.Sin(2*Math.PI*i/count))).ToArray();
        var hole=Circle(9,2048);Array.Reverse(hole);
        IReadOnlyList<PointD>[] source=[Circle(20,1024),hole,Circle(2,512),Rect(-12,.5,12,1.5)];
        if(mirror)source=source.Select(path=>(IReadOnlyList<PointD>)path.Select(p=>new PointD(-p.x,p.y)).ToArray()).ToArray();
        foreach(var window in new[]{Rect(-3,-3,3,3),Rect(-11,-2,12,2),Rect(15,-6,22,9)})
        {
            var expected=PolygonOps.Intersect(source,[window]);
            var actual=PolygonOps.ClipSignedToRect(source,Tick(window[0].x),Tick(window[0].y),
                Tick(window[2].x),Tick(window[2].y),default);
            var difference=Clipper.Xor(PathUtils.ToClipper(expected),PathUtils.ToClipper(actual),FillRule.NonZero);
            Console.WriteLine($"[SignedRectCrop.Compare] mirror={mirror} window=({window[0].x},{window[0].y})..({window[2].x},{window[2].y}) "
                +$"expectedCounts={string.Join(',',expected.Select(p=>p.Count))} actualCounts={string.Join(',',actual.Select(p=>p.Count))} "
                +$"differencePaths={difference.Count} differenceAreaTicks={difference.Sum(p=>Math.Abs(Clipper.Area(p))):R} "
                +$"differenceVertices={string.Join(';',difference.SelectMany(p=>p).Take(16).Select(p=>$"({p.X},{p.Y})"))}");
            // Both backends retain the identical signed fill. At a rounded
            // extremum Clipper can express a zero-area retrace differently;
            // bound that boundary discrepancy to ONE native tick, inside the
            // existing eighteen-tick crop reserve.
            Assert.All(difference,path=>{
                Assert.Equal(0,Clipper.Area(path));
                Assert.True(path.Max(p=>p.X)-path.Min(p=>p.X)<=1
                    ||path.Max(p=>p.Y)-path.Min(p=>p.Y)<=1);
            });
        }
    }

    [Fact]
    public void ApprovedAirIsDepthScopedAndNeverFabricatesAnExecutedStrokeOrLaterSnapshot()
    {
        var hole=Rect(-2,-2,2,2);Array.Reverse(hole);
        IReadOnlyList<PointD>[] air=[Rect(-10,-10,10,10),hole];
        var stock=new PocketContinuationRouter.ClearedStock(.5,default,knownEmpty:air,knownEmptyFloor:-1);
        var before=stock.PrepareAtFloor(-1);
        Assert.Equal(0,stock.RecordedStrokes);
        Assert.True(PolygonOps.Contains(before.Materialize(),5,0));
        Assert.False(PolygonOps.Contains(before.Materialize(),0,0));
        Assert.Empty(stock.AtFloor(-2));
        stock.Record([new(-1,0,-1),new(1,0,-1)]);
        Assert.True(PolygonOps.Contains(stock.AtFloor(-1),0,0));
        Assert.False(PolygonOps.Contains(before.Materialize(),0,0));
    }

    [Fact]
    public void ApprovedAirJoinsEarlierCutAndFirstFollowingCutWithoutBecomingACuttingTarget()
    {
        var air=PolygonOps.Offset([Rect(2,0,18,10)],-PocketContinuationRouter.KnownAirReserve,.0001,0)
            .Select(p=>(IReadOnlyList<PointD>)p).ToArray();
        var permitted=new PocketContinuationRouter.ClearedStock(1,default,knownEmpty:air,knownEmptyFloor:-1);
        var ordinary=new PocketContinuationRouter.ClearedStock(1,default);
        Point3D[] earlier=[new(1,0,-1),new(1,10,-1)];permitted.Record(earlier);ordinary.Record(earlier);
        IReadOnlyList<PointD>[] following=[[new(19,0),new(19,10)]];
        var allowed=new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,permitted,-1,following,1,default);
        var blocked=new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,ordinary,-1,following,1,default);
        Assert.True(allowed.IsTravelSafe(new(1,5,-1),new(19,5,-1),1));
        Assert.False(blocked.IsTravelSafe(new(1,5,-1),new(19,5,-1),1));
        Assert.Equal(1,permitted.RecordedStrokes);
    }
    [Fact]
    public void UncoveredCapsulePointRefusesWithoutWholeBufferAndRetainsTheReferenceDecision()
    {
        var stock=new PocketContinuationRouter.ClearedStock(.5,default);
        stock.Record([new(0,0,-1),new(10,0,-1)]);
        var diagnostics=new PocketContinuationRouter.CoverageDiagnostics();
        var local=new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,stock,-1,
            Array.Empty<IReadOnlyList<PointD>>(),.5,default,diagnostics,validateIndexedCoverage:true);
        var reference=new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,stock,-1,
            Array.Empty<IReadOnlyList<PointD>>(),.5,default,useIndexedCoverage:false);
        Point3D a=new(1,0,-1),b=new(9,2,-1);
        Assert.False(local.IsTravelSafe(a,b,.5));Assert.False(reference.IsTravelSafe(a,b,.5));
        Assert.Equal(1,diagnostics.NegativeWitnesses);Assert.Equal(0,diagnostics.WholeFallbacks);
        Assert.True(local.IsTravelSafe(a,new(9,0,-1),.5));
    }
    [Fact]
    public void EnclosingOuterAndHoleStayWhenAllTheirEdgesMissTheWindow()
    {
        var outer = Rect(0, 0, 20, 20);
        var hole = Rect(5, 5, 15, 15); Array.Reverse(hole);
        var island = Rect(9, 9, 11, 11);
        IReadOnlyList<PointD>[] source = [outer, hole, island, Rect(100, 100, 120, 120)];
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var prepared = new PocketContinuationRouter.PreparedCoverageContours(source, CancellationToken.None, diagnostics);
        var selected = prepared.Intersecting(Tick(7), Tick(7), Tick(13), Tick(13), CancellationToken.None, diagnostics);
        Assert.Equal(3, selected.Count);
        var window = Rect(7, 7, 13, 13);
        var clipped = PolygonOps.Intersect(selected, [window]);
        EqualPolygons(PolygonOps.Intersect(source, [window]), clipped);
        var boundary = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(clipped));
        Assert.False(boundary.ContainsCapsule(new(Tick(8), Tick(8)), new(Tick(8), Tick(8)), .1 * PathUtils.Scale));
        Assert.True(boundary.ContainsCapsule(new(Tick(10), Tick(10)), new(Tick(10), Tick(10)), .1 * PathUtils.Scale));
    }

    [Fact]
    public void ExpandedWindowsReuseImmutableBoundsAndPruneOnlyZeroWindingContours()
    {
        var outer = Rect(0, 0, 20, 20);
        var hole = Rect(5, 5, 15, 15); Array.Reverse(hole);
        IReadOnlyList<PointD>[] source = [outer, hole, .. Enumerable.Range(1, 64).Select(i => Rect(i * 100, 0, i * 100 + 20, 20))];
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var prepared = new PocketContinuationRouter.PreparedCoverageContours(source, CancellationToken.None, diagnostics);
        var saved = source.Select(path => (IReadOnlyList<PointD>)path.ToArray()).ToArray();
        // A caller's later mutation cannot make stored bounds disagree with the
        // signed contours used by the proof; preparation owns both snapshots.
        outer[0] = new(-500, -500); hole[0] = new(-500, -500);
        foreach (var window in new[] { Rect(7, 7, 13, 13), Rect(-1, -1, 4, 4), Rect(299, -1, 321, 21) })
        {
            var selected = prepared.Intersecting(Tick(window[0].x), Tick(window[0].y), Tick(window[2].x), Tick(window[2].y),
                CancellationToken.None, diagnostics);
            EqualPolygons(PolygonOps.Intersect(saved, [window]), PolygonOps.Intersect(selected, [window]));
        }
        Assert.Equal(1, diagnostics.ContourBoundsBuilds);
        Assert.Equal(3, diagnostics.ContourBoundsQueries);
        Assert.True(diagnostics.ContourBoundsSubmittedVertices < diagnostics.ContourBoundsConsideredVertices / 20);
    }

    [Fact]
    public void CancellationCannotPublishPartialBoundsAndAlsoAppliesToReusedBounds()
    {
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        using var during = new CancellationTokenSource();
        var canceling = new CancelingContour(Rect(0, 0, 20, 20), during.Cancel);
        Assert.Throws<OperationCanceledException>(() => new PocketContinuationRouter.PreparedCoverageContours(
            [canceling], during.Token, diagnostics));
        Assert.Equal(0, diagnostics.ContourBoundsBuilds);
        var prepared = new PocketContinuationRouter.PreparedCoverageContours([Rect(0, 0, 20, 20)], CancellationToken.None, diagnostics);
        using var canceled = new CancellationTokenSource(); canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => prepared.Intersecting(0, 0, 1, 1, canceled.Token, diagnostics));
        Assert.Equal(0, diagnostics.ContourBoundsQueries);
        Assert.Single(prepared.Intersecting(0, 0, 1, 1, CancellationToken.None, diagnostics));
    }

    [Theory]
    [InlineData(double.PositiveInfinity)]
    [InlineData(30517578.1251)]
    public void UnboundedCoordinatesDeclineTheFixedReserveProof(double outside)
    {
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var prepared = new PocketContinuationRouter.PreparedCoverageContours([Rect(0, 0, outside, 20)], CancellationToken.None, diagnostics);
        Assert.False(prepared.SupportsCrop);
    }

    [Theory]
    [InlineData(-.5)]
    [InlineData(0)]
    [InlineData(.5)]
    public void CropBeforeJointUnionKeepsReferenceDecisionsAtCrossingAndToleranceEdges(double halfTick)
    {
        double q = 1 / PathUtils.Scale, shift = halfTick * q;
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        stock.Record([new(-4, shift, -1), new(4, shift, -1)]);
        stock.PrepareAtFloor(-1);
        for (int i = 0; i < 16; i++)
        {
            stock.Record([new(100 + 30 * i, 0, -1), new(120 + 30 * i, 0, -1)]);
            stock.PrepareAtFloor(-1);
        }
        IReadOnlyList<PointD>[] following = [new PointD[] { new(-shift, -4), new(-shift, 4) }];
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        int approved = 0, refused = 0;
        foreach (double distance in new[] { .08, .12, .145, .149, .151, .16, .2 })
        {
            var candidate = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
                stock, -1, following, .5, CancellationToken.None, diagnostics, validateIndexedCoverage: true);
            var reference = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
                stock, -1, following, .5, CancellationToken.None, useIndexedCoverage: false);
            Point3D a = new(-distance, -distance, -1), b = new(distance, distance, -1);
            bool safe = candidate.IsTravelSafe(a, b, .5);
            Assert.Equal(reference.IsTravelSafe(a, b, .5), safe);
            if (safe)
            {
                approved++;
                // Independent complete-capsule bound for the two perpendicular
                // actual cylinders: the worst point has both axis distances
                // distance + radius/sqrt(2), plus the tiny source translation.
                Assert.True(distance + .5 / Math.Sqrt(2) + Math.Abs(shift) <= .5 + .003 + 4 * q);
            }
            else refused++;
        }
        Assert.True(approved >= 3); Assert.True(refused >= 2);
        Assert.True(diagnostics.CroppedDomainProofs >= 3);
        Assert.Equal(0, diagnostics.LocalProofMismatches);
        Assert.True(diagnostics.ContourBoundsSubmittedVertices < diagnostics.ContourBoundsConsideredVertices / 4);
    }

    [Fact]
    public void CachedBoundsOfAnOldSnapshotNeverCreditLaterExecutedStock()
    {
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        stock.Record([new(0, 0, -1), new(5, 0, -1)]);
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var before = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None, diagnostics,
            validateIndexedCoverage: true);
        Assert.True(before.IsTravelSafe(new(1, 0, -1), new(4, 0, -1), .5));
        Assert.Equal(1, diagnostics.ContourBoundsBuilds);
        stock.Record([new(5, 0, -1), new(25, 0, -1)]);
        stock.AtFloor(-1);
        Assert.False(before.IsTravelSafe(new(10, 0, -1), new(20, 0, -1), .5));
        Assert.Equal(1, diagnostics.ContourBoundsBuilds);
        Assert.Equal(0, diagnostics.LocalProofMismatches);
        var after = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None, validateIndexedCoverage: true);
        Assert.True(after.IsTravelSafe(new(10, 0, -1), new(20, 0, -1), .5));
    }

    [Fact]
    public void RepeatedSuccessfulCropsUseTheCapturedUndilatedCapsuleWithoutCreditingNewStock()
    {
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        stock.Record([new(0, 0, -1), new(30, 0, -1)]);
        IReadOnlyList<PointD>[] following = [new PointD[] { new(40, 0), new(50, 0) }];
        var diagnostics = new PocketContinuationRouter.CoverageDiagnostics();
        var limited = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, following, .5, CancellationToken.None, diagnostics, validateIndexedCoverage: true,useLocalTileCache:false);
        var unlimited = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, following, .5, CancellationToken.None, limitCropWork: false,useLocalTileCache:false);
        var original = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, following, .5, CancellationToken.None, useIndexedCoverage: false);
        // Alternate separate immutable captured contours. Each new window must
        // be cropped again, while the whole proof's input remains exactly fixed.
        for (int i = 0; i < 10; i++)
        {
            double x = i % 2 == 0 ? 1 + i : 40.5 + i / 2;
            Point3D a = new(x, 0, -1), b = new(x + .25, 0, -1);
            Assert.True(original.IsTravelSafe(a, b, .5));
            Assert.Equal(original.IsTravelSafe(a, b, .5), limited.IsTravelSafe(a, b, .5));
            Assert.Equal(original.IsTravelSafe(a, b, .5), unlimited.IsTravelSafe(a, b, .5));
            if (i == 3)
            {
                stock.Record([new(60, 0, -1), new(80, 0, -1)]);
                stock.AtFloor(-1); // Force the shared floor beyond this snapshot.
            }
        }
        Assert.Equal(1, diagnostics.CropWorkBudgetFallbacks);
        Assert.Equal(1, diagnostics.CapturedUnionBuilds);
        Assert.Equal(0, diagnostics.WholeFallbacks);
        Assert.True(diagnostics.UndilatedProofs>0);
        Assert.True(diagnostics.CropWorkBudgetPriorBuilds >= 4);
        Assert.Equal(0, diagnostics.LocalProofMismatches);
        Point3D futureA = new(65, 0, -1), futureB = new(75, 0, -1);
        Assert.False(limited.IsTravelSafe(futureA, futureB, .5));
        Assert.False(original.IsTravelSafe(futureA, futureB, .5));
        var newer = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -1, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None);
        Assert.True(newer.IsTravelSafe(futureA, futureB, .5));
        var deeper = new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,
            stock, -2, Array.Empty<IReadOnlyList<PointD>>(), .5, CancellationToken.None);
        Assert.False(deeper.IsTravelSafe(futureA with { Z = -2 }, futureB with { Z = -2 }, .5));
    }

    private static long Tick(double value) => (long)Math.Round(value * PathUtils.Scale);
    private static PointD[] Rect(double minX, double minY, double maxX, double maxY)
        => [new(minX, minY), new(maxX, minY), new(maxX, maxY), new(minX, maxY)];
    private static void EqualPolygons(IReadOnlyList<List<PointD>> expected, IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++) Assert.Equal(expected[i].ToArray(), actual[i].ToArray());
    }
    private sealed class CancelingContour(PointD[] points, Action cancel) : IReadOnlyList<PointD>
    {
        public int Count => points.Length;
        public PointD this[int index] { get { cancel(); return points[index]; } }
        public IEnumerator<PointD> GetEnumerator() => ((IEnumerable<PointD>)points).GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }
}
