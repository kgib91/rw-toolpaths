using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class OffsetBoundaryCleanupTests
{
    [Fact]
    public void ACurvedFoldKeepsBothForwardFlanksWithCompleteNativeSegmentProofs()
    {
        // C is almost on A-B, yet the INTERIOR of C-D is farther than the
        // 3um proof budget from A-B-D. Endpoint-only Hausdorff misses that gap.
        var native = new List<PointD>
        {
            new(0,0), new(.1,0), new(.005,.0001), new(.2,.008), new(1,1), new(0,1),
        };
        var walls = Index(new[] { Rectangle(-4,-4,5,5) });
        var clean = Assert.Single(OffsetBoundaryCleanup.Compact(new[] { native }, walls, -3, 3, .003));
        Assert.DoesNotContain(native[1], clean);
        Assert.DoesNotContain(native[2], clean);
        Assert.True(clean.Count < native.Count);
        for (int index = 0; index < clean.Count; index++)
        {
            PointD a = clean[index], b = clean[(index + 1) % clean.Count],
                c = clean[(index + 2) % clean.Count], d = clean[(index + 3) % clean.Count];
            Assert.False((b.x-a.x)*(c.x-b.x)+(b.y-a.y)*(c.y-b.y)<0
                && (c.x-b.x)*(d.x-c.x)+(c.y-b.y)*(d.y-c.y)<0);
            Assert.Null(walls.ClearanceViolation(ToPoint(a), ToPoint(b), 3 * PathUtils.Scale - 4));
        }
        var originalSweep = PolygonOps.BufferCenterlines(new[] { native }, 3, .001);
        var cleanSweep = PolygonOps.BufferCenterlines(new[] { clean }, 3, .001);
        const double error = .003 + 4 / PathUtils.Scale;
        Assert.Empty(PolygonOps.Offset(PolygonOps.Difference(originalSweep, cleanSweep), -error, .00025, 0));
        Assert.Empty(PolygonOps.Offset(PolygonOps.Difference(cleanSweep, originalSweep), -error, .00025, 0));
    }

    [Fact]
    public void RawDenseRoundedSlotCompactionPreservesClearanceAndPhysicalCoverage()
    {
        var source = RoundSlot(1024);
        var raw = PolygonOps.OffsetBoundary(source, null, -3, .001, 0);
        var native = PolygonOps.DifferenceToComponents(raw, Array.Empty<IReadOnlyList<PointD>>())
            .SelectMany(component => new[] { component.Outer }.Concat(component.Holes)).ToList();
        var original = Index(new[] { source });
        var clean = OffsetBoundaryCleanup.Compact(native, original, -3, 3, .001);
        var ring = Assert.Single(clean);
        double perimeter = Perimeter(ring);
        int reversals = Enumerable.Range(0, ring.Count).Count(index =>
        {
            PointD a = ring[(index + ring.Count - 1) % ring.Count], b = ring[index], c = ring[(index + 1) % ring.Count];
            double dx = b.x - a.x, dy = b.y - a.y, ex = c.x - b.x, ey = c.y - b.y;
            return dx * ex + dy * ey < -.98 * Math.Sqrt((dx * dx + dy * dy) * (ex * ex + ey * ey));
        });
        Console.WriteLine($"[Cleanup.RawSlot] sourcePoints={source.Count} nativePoints={native.Sum(path => path.Count)} "
            + $"nativePerimeter={native.Sum(Perimeter):R} cleanedPoints={ring.Count} cleanedPerimeter={perimeter:R} reversals={reversals}");
        // This isolates the raw helper's geometric certificates. Machining
        // monotonicity is asserted by the strict shared-collector test below.
        Assert.InRange(Math.Abs(perimeter - 48 * Math.PI), 0, .025);
        for (int index = 0; index < ring.Count; index++)
            Assert.Null(original.ClearanceViolation(ToPoint(ring[index]), ToPoint(ring[(index + 1) % ring.Count]),
                (3 - 4 / PathUtils.Scale) * PathUtils.Scale));
        var oldSweep = PolygonOps.BufferCenterlines(native, 3, .001);
        var cleanSweep = PolygonOps.BufferCenterlines(clean, 3, .001);
        Assert.Empty(PolygonOps.Offset(PolygonOps.Difference(oldSweep, cleanSweep), -(.003 + 4 / PathUtils.Scale), .00025, 0));
        Assert.Empty(PolygonOps.Offset(PolygonOps.Difference(cleanSweep, oldSweep), -(.003 + 4 / PathUtils.Scale), .00025, 0));
    }

    [Fact]
    public void SharedPhysicalDenseSlotHasNoBacktracksAndRetainsItsAnalyticPerimeter()
    {
        var source=RoundSlot(1024);
        var fill=MedialAxisGeometry.BuildConcentricFill(new[] {new RW.Toolpaths.Milling.MillingRegion(source)},
            new RW.Toolpaths.Milling.RingOffsetOptions(3,6) {MinimumRingArea=0},3,true);
        var ring=Assert.Single(fill.Where(path=>path.IsClosed)).Points;
        Assert.InRange(Math.Abs(Perimeter(ring)-48*Math.PI),0,.025);
        for(int index=0;index<ring.Count;index++)
        {
            PointD a=ring[(index+ring.Count-1)%ring.Count],b=ring[index],c=ring[(index+1)%ring.Count];
            double ux=b.x-a.x,uy=b.y-a.y,vx=c.x-b.x,vy=c.y-b.y;
            Assert.True(ux*vx+uy*vy>=-.98*Math.Sqrt((ux*ux+uy*uy)*(vx*vx+vy*vy)),
                $"Physical dense-slot front reverses at vertex{index}.");
        }
        Assert.Single(fill.Where(path=>!path.IsClosed));
    }

    [Fact]
    public void UnsafeHoleChordsRetainTheirNativeChainWhileUnrelatedBurrsAreRemoved()
    {
        var source = Rectangle(-10, -10, 30, 30);
        var sourceHole = Circle(3); sourceHole.Reverse();
        var outer = Rectangle(-7, -7, 25, 25);
        outer.InsertRange(3, new[] { new PointD(12, 25), new PointD(12.02, 25.00005), new PointD(11.98, 25.0001) });
        var hole = Circle(6); hole.Reverse();
        var front = new List<List<PointD>> { outer, hole };
        var clean = OffsetBoundaryCleanup.Compact(front, Index(new[] { source, sourceHole }), -3, 3, .001);

        Assert.Equal(2, clean.Count);
        Assert.True(clean[0].Count < outer.Count);
        Assert.Equal(hole.Select(ToPoint).OrderBy(point => point.X).ThenBy(point => point.Y),
            clean[1].Select(ToPoint).OrderBy(point => point.X).ThenBy(point => point.Y));
        Assert.True(PolygonOps.SignedArea(clean[1]) < 0);
    }

    [Fact]
    public void AShortRealBoundaryBranchAndCanonicalZeroFrontRemainPresent()
    {
        var source = Rectangle(-10, -10, 20, 20);
        var front = new List<PointD> { new(0,0),new(10,0),new(10,10),new(5.01,10),new(5.01,10.1),new(5,10.1),new(5,10),new(0,10) };
        var original = Index(new[] { source });
        var clean = Assert.Single(OffsetBoundaryCleanup.Compact(new[] { front }, original, -3, 3, .001));
        Assert.Contains(new PointD(5.01,10.1), clean);
        Assert.Contains(new PointD(5,10.1), clean);
        Assert.Equal(front, Assert.Single(OffsetBoundaryCleanup.Compact(new[] { front }, original, 0, 3, .001)));
        Assert.Equal(front, Assert.Single(OffsetBoundaryCleanup.Compact(new[] { front }, original, 3, 3, .001)));
    }

    [Fact]
    public void ABacktrackAcrossTheInputSeamIsInsideTheCyclicProof()
    {
        var native = new List<PointD>
        {
            new(5.02,10.00005),new(4.98,10.0001),new(0,10),new(0,0),new(10,0),new(10,10),new(5,10),
        };
        var clean = Assert.Single(OffsetBoundaryCleanup.Compact(new[] { native },
            Index(new[] { Rectangle(-10,-10,20,20) }), -3, 3, .001));
        Assert.DoesNotContain(native[0], clean);
        Assert.True(clean.Count < native.Count);
        Assert.InRange(Math.Abs(Perimeter(clean) - 40), 0, .002);
    }

    [Fact]
    public void ASeparateFilledIslandInsideAProtectedHoleDoesNotRejectUnrelatedCleanup()
    {
        var source = Rectangle(-10,-10,50,50);
        var sourceHole = Rectangle(10,10,30,30); sourceHole.Reverse();
        var sourceIsland = Rectangle(14,14,26,26);
        var outer = new List<PointD>
        {
            new(-7,-7),new(47,-7),new(47,47),new(20,47),new(20.02,47.00005),new(19.98,47.0001),new(-7,47),
        };
        var hole = Rectangle(7,7,33,33); hole.Reverse();
        var island = Rectangle(17,17,23,23);
        var clean = OffsetBoundaryCleanup.Compact(new[] { outer,hole,island },
            Index(new[] { source,sourceHole,sourceIsland }), -3, 3, .001);
        Assert.True(clean[0].Count < outer.Count);
        var components = PolygonOps.DifferenceToComponents(clean, Array.Empty<IReadOnlyList<PointD>>());
        Assert.Equal(2, components.Count);
        Assert.Equal(1, components.Sum(component => component.Holes.Count));
    }

    [Fact]
    public void AFrontWithoutActualBacktracksRetainsEveryAuthoredPoint()
    {
        var native=Circle(10);
        var clean=Assert.Single(OffsetBoundaryCleanup.Compact(new[] {native},
            Index(new[] {Rectangle(-20,-20,20,20)}),-3,3,.001));
        Assert.Equal(native,clean);
    }

    [Fact]
    public void APartiallyClippedCarrierCannotDiscardAnotherPositiveFront()
    {
        var carrier=new List<PointD>
        {
            new(0,0),new(10,0),new(10,10),new(5,10),new(5.02,10.00005),new(4.98,10.0001),new(0,10),
        };
        var other=Rectangle(10.001,4,10.0011,4.0001);
        var clip=Rectangle(8,-5,14,15);
        var clean=OffsetBoundaryCleanup.Compact(new[] {carrier,other},
            Index(new[] {Rectangle(-20,-20,20,20)}),-3,3,.001,clipBoundary:clip);
        Assert.Equal(2,PolygonOps.DifferenceToComponents(clean,Array.Empty<IReadOnlyList<PointD>>()).Count);
    }

    private static PolygonOps.BoundarySegmentIndex Index(IEnumerable<IReadOnlyList<PointD>> source)
        => new(PathUtils.ToClipper(PolygonOps.Union(source)));
    private static Point64 ToPoint(PointD point) => new((long)Math.Round(point.x * PathUtils.Scale), (long)Math.Round(point.y * PathUtils.Scale));
    private static List<PointD> Rectangle(double x, double y, double endX, double endY)
        => new() { new(x,y),new(endX,y),new(endX,endY),new(x,endY) };
    private static List<PointD> Circle(double radius) => Enumerable.Range(0,512).Select(index =>
        new PointD(radius * Math.Cos(index * 2 * Math.PI / 512),radius * Math.Sin(index * 2 * Math.PI / 512))).ToList();
    private static double Perimeter(IReadOnlyList<PointD> path) => Enumerable.Range(0,path.Count).Sum(index =>
    {
        PointD a=path[index],b=path[(index+1)%path.Count];
        return Math.Sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    });
    private static List<PointD> RoundSlot(int count)
    {
        var outline=new List<PointD>();
        for(int index=0;index<=count;index++) { double angle=Math.PI/2*index/count; outline.Add(new(46*Math.Cos(angle),46*Math.Sin(angle))); }
        for(int index=1;index<count;index++) { double angle=Math.PI/2+Math.PI*index/count; outline.Add(new(8*Math.Cos(angle),38+8*Math.Sin(angle))); }
        for(int index=count;index>=0;index--) { double angle=Math.PI/2*index/count; outline.Add(new(30*Math.Cos(angle),30*Math.Sin(angle))); }
        for(int index=1;index<count;index++) { double angle=Math.PI+Math.PI*index/count; outline.Add(new(38+8*Math.Cos(angle),8*Math.Sin(angle))); }
        return outline;
    }
}
