using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class CurvedOffsetConstructionTests
{
    [Theory]
    [InlineData(128)]
    [InlineData(1024)]
    public void DenseCurvedFrontHasMonotoneTravelAndProvenOriginalWallClearance(int subdivisions)
    {
        var outline = RoundSlot(subdivisions);
        var original = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(new[] { outline }));
        var baseline = PolygonOps.PrepareBoundaryOffset(outline, null, .001, 0).Execute(-3);
        var prepared = PolygonOps.PrepareBoundaryOffset(outline, null, .001, 0,
            sourceConstructionTolerance: .001, sourceConstructionSweepRadius: 3);
        var fronts = RingOffsetEngine.RetainUncoveredFirstComponents(prepared.Execute(-3), 3, CancellationToken.None);
        var front = Assert.Single(fronts);
        double reversal = 0, maxDeviation = 0;
        int rejected = 0, reversedEdges = 0;
        for (int i = 0; i < front.Count; i++)
        {
            var a = front[i]; var b = front[(i + 1) % front.Count];
            var midpoint = new PointD((a.x + b.x) / 2, (a.y + b.y) / 2);
            var circles = new[] { (X: 0d, Y: 0d, R: 43d, Sign: 1d), (X: 0d, Y: 0d, R: 33d, Sign: -1d),
                (X: 0d, Y: 38d, R: 5d, Sign: 1d), (X: 38d, Y: 0d, R: 5d, Sign: 1d) };
            var circle = circles.MinBy(c => Math.Abs(Math.Sqrt(Math.Pow(midpoint.x-c.X, 2)+Math.Pow(midpoint.y-c.Y, 2))-c.R));
            double rx = midpoint.x-circle.X, ry = midpoint.y-circle.Y, radius = Math.Sqrt(rx*rx+ry*ry);
            maxDeviation = Math.Max(maxDeviation, Math.Abs(radius-circle.R));
            double advance = ((b.x-a.x)*-ry + (b.y-a.y)*rx) / radius * circle.Sign;
            if (advance < -1 / PathUtils.Scale) { reversedEdges++; reversal -= advance; }
            Point64 Native(PointD p) => new((long)Math.Round(p.x*PathUtils.Scale), (long)Math.Round(p.y*PathUtils.Scale));
            if (original.ClearanceViolation(Native(a), Native(b), 3*PathUtils.Scale-4) is not null) rejected++;
        }
        var originalSweep = PolygonOps.BufferCenterlines(baseline, 3, .001);
        var actualSweep = PolygonOps.BufferCenterlines(fronts, 3, .001);
        double loss = Math.Abs(PolygonOps.Offset(PolygonOps.Difference(originalSweep, actualSweep),
            -.003-4/PathUtils.Scale, .00025, 0).Sum(PolygonOps.SignedArea));
        double perimeter = PolygonOps.Perimeter(front);
        Console.WriteLine($"[Offset.Construction.Quality] sourceSubdivisions={subdivisions} sourcePoints={outline.Count} "
            + $"frontPoints={front.Count} perimeter={perimeter:R} reversedEdges={reversedEdges} "
            + $"negativeAdvance={reversal:R} maxAnalyticDeviation={maxDeviation:R} "
            + $"originalCapsuleFailures={rejected} lostOriginalFootprintArea={loss:R}");
        Assert.InRange(perimeter, 48*Math.PI-.02, 48*Math.PI+.02);
        Assert.Equal(0, reversedEdges);
        Assert.InRange(maxDeviation, 0, .002);
        Assert.Equal(0, rejected);
        Assert.Equal(0, loss);

        // Zero fronts and outward authored overcuts retain the exact original
        // prepared source, independently of inward construction reduction.
        Assert.Equal(Geometry(PolygonOps.PrepareBoundaryOffset(outline, null, .001, 0).Execute(0)), Geometry(prepared.Execute(0)));
        Assert.Equal(Geometry(PolygonOps.PrepareBoundaryOffset(outline, null, .001, 0).Execute(.25)), Geometry(prepared.Execute(.25)));
    }

    [Fact]
    public void LowPitchSharedFillCanonicalizesEveryFirstFront()
    {
        var fill = MedialAxisGeometry.BuildConcentricFill(new[] { new MillingRegion(RoundSlot(1024)) },
            new RingOffsetOptions(3, 3) { MinimumRingArea = 0 }, 3, true);
        Assert.Single(fill.Where(ring => ring.Level == 0));
        Assert.All(fill, ring => Assert.True(ring.IsClosed));
        Assert.All(fill, ring => Assert.True(ring.Perimeter > 1));
    }

    [Fact]
    public void SharpOffsetCannotAmplifyAConstructionGuardIntoLostReachableStock()
    {
        var outline = new List<PointD> { new(-50, 0), new(0, 1000), new(50, 0) };
        for (int i=1;i<512;i++)
        {
            double angle=-Math.PI*i/512;
            outline.Add(new(50*Math.Cos(angle),50*Math.Sin(angle)));
        }
        var prepared=PolygonOps.PrepareBoundaryOffset(outline,null,.001,0,
            sourceConstructionTolerance:.001,sourceConstructionSweepRadius:3);
        var first=prepared.Execute(-3);
        double legalTip=1000-3/Math.Sin(Math.Atan(.05));
        // Quantized wall intersections amplify their coordinate error by the
        // same acute angle. Bound that native-grid error explicitly.
        double intersectionGridError=2/PathUtils.Scale/Math.Sin(Math.Atan(.05));
        var witness=new PointD(0,legalTip+3-.006);
        Assert.InRange(first.Max(path=>path.Max(p=>p.y)),legalTip-intersectionGridError,legalTip+intersectionGridError);
        Assert.True(first.Min(path=>Distance(witness,path,true))<=3.003,
            "An inward guard amplified at a sharp intersection removed a reachable cutter-cap crescent.");
        // Repeated construction of the first inset must retain its certified
        // original front. Later conditioned fronts are accepted only by the
        // collector's independent physical shell certificate.
        Assert.Equal(Geometry(first),Geometry(prepared.Execute(-3)));
    }

    [Fact]
    public void EquivalentCollinearReductionPreservesTheExactRequestedInset()
    {
        var outline=new List<PointD>();
        for(int i=0;i<256;i++) outline.Add(new(i*60d/256,0));
        for(int i=0;i<256;i++) outline.Add(new(60,i*90d/256));
        for(int i=0;i<256;i++) outline.Add(new(60-i*60d/256,90));
        for(int i=0;i<256;i++) outline.Add(new(0,90-i*90d/256));
        var first=PolygonOps.PrepareBoundaryOffset(outline,null,.001,0,
            sourceConstructionTolerance:.001,sourceConstructionSweepRadius:3).Execute(-3);
        Assert.Equal(3,Assert.Single(first).Min(p=>p.x));
        Assert.Equal(57,first[0].Max(p=>p.x));
    }

    [Theory]
    [InlineData(3.0005)]
    [InlineData(3.002)]
    [InlineData(3.01)]
    public void ConstructionReductionCannotConsumeAReachableThinCurvedSliver(double halfWidth)
    {
        var fill = MedialAxisGeometry.BuildConcentricFill(new[] { new MillingRegion(RoundSlot(1024, halfWidth)) },
            new RingOffsetOptions(3, 6) { MinimumRingArea = 0 }, 3, true);
        Assert.NotEmpty(fill);
        for (int i=16;i<=112;i++)
        {
            double angle=Math.PI/2*i/128;
            var witness=new PointD(38*Math.Cos(angle),38*Math.Sin(angle));
            Assert.True(fill.Min(ring => Distance(witness, ring.Points, ring.IsClosed)) <= 3.002,
                $"Reachable curved sliver lost: halfWidth={halfWidth:R}, angle={angle:R}.");
        }
        // This tube contains disks of radius three on its authored axis. Sample
        // both reachable sides and both round ends, not just its centerline.
        for (int i=0;i<=128;i++)
        for (int j=0;j<32;j++)
        {
            double angle=Math.PI/2*i/128, discAngle=2*Math.PI*j/32;
            var witness=new PointD(38*Math.Cos(angle)+2.995*Math.Cos(discAngle),
                38*Math.Sin(angle)+2.995*Math.Sin(discAngle));
            Assert.True(fill.Min(ring=>Distance(witness,ring.Points,ring.IsClosed))<=3.002,
                $"Reachable sliver footprint lost: halfWidth={halfWidth:R}, axisAngle={angle:R}, discAngle={discAngle:R}.");
        }
    }

    private static string Geometry(IEnumerable<IReadOnlyList<PointD>> rings)
        => System.Text.Json.JsonSerializer.Serialize(rings.Select(ring=>ring.Select(p=>new[]{p.x,p.y})));

    private static double Distance(PointD p, IReadOnlyList<PointD> path, bool closed)
    {
        double best=double.PositiveInfinity;
        for(int i=closed?0:1;i<path.Count;i++)
        {
            var a=path[(i+path.Count-1)%path.Count];var b=path[i];
            double dx=b.x-a.x,dy=b.y-a.y,length=dx*dx+dy*dy;
            double t=length>0?Math.Clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/length,0,1):0;
            best=Math.Min(best,Math.Sqrt(Math.Pow(p.x-a.x-t*dx,2)+Math.Pow(p.y-a.y-t*dy,2)));
        }
        return best;
    }

    private static List<PointD> RoundSlot(int count, double halfWidth=8)
    {
        var points = new List<PointD>();
        for (int i=0;i<=count;i++) { double a=Math.PI/2*i/count; points.Add(new((38+halfWidth)*Math.Cos(a),(38+halfWidth)*Math.Sin(a))); }
        for (int i=1;i<count;i++) { double a=Math.PI/2+Math.PI*i/count; points.Add(new(halfWidth*Math.Cos(a),38+halfWidth*Math.Sin(a))); }
        for (int i=count;i>=0;i--) { double a=Math.PI/2*i/count; points.Add(new((38-halfWidth)*Math.Cos(a),(38-halfWidth)*Math.Sin(a))); }
        for (int i=1;i<count;i++) { double a=Math.PI+Math.PI*i/count; points.Add(new(38+halfWidth*Math.Cos(a),halfWidth*Math.Sin(a))); }
        return points;
    }
}
