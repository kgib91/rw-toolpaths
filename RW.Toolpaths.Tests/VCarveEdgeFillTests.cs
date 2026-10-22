using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveEdgeFillTests
{
    [Fact]
    public void OffsetFillStopsAtExactPartialTerminalInset()
    {
        var paths = OffsetFill.Generate([Rectangle(0, 0, 30, 20)], -1, 0, .6, maxInset: 1);

        Assert.Equal(3, paths.Count);
        var insets = paths.Select(path => path.Min(point => point.X)).Order().ToArray();
        Assert.Equal(0, insets[0], 6);
        Assert.InRange(Math.Abs(.6 - insets[1]), 0, 1 / PathUtils.Scale);
        Assert.Equal(1, insets[2], 6);
    }

    [Fact]
    public void ZeroInsetKeepsOnlyOriginalPerimeters()
    {
        var boundary = new[] { Rectangle(0, 0, 30, 20) };
        var old = OffsetFill.Generate(boundary, -1, 0, .6, maxIterations: 0);
        var capped = OffsetFill.Generate(boundary, -1, 0, .6, maxInset: 0);
        Assert.Equal(Geometry(old), Geometry(capped));
    }

    [Fact]
    public void BoundedOffsetChecksCancellationBetweenRings()
    {
        using var cancellation = new CancellationTokenSource();
        int reported = 0;
        Assert.Throws<OperationCanceledException>(() => OffsetFill.Generate(
            [Rectangle(0, 0, 30, 20)], -1, 0, .6, maxInset: 2,
            cancellationToken: cancellation.Token,
            progress: (ring, _) => { reported = ring; if (ring == 1) cancellation.Cancel(); }));
        Assert.Equal(1, reported);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EdgeFillStaysOutsideOffsetCoreAtEveryDepth(bool rounded)
    {
        const double width = 2.35;
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [Rectangle(0, 0, 20, 16)];
        var profile = rounded ? Ball() : null;
        var perimeter = Clearing(boundary, false, 0, profile);
        var bounded = Clearing(boundary, false, width, profile);
        var full = Clearing(boundary, true, 0, profile);

        Assert.True(bounded.Count > perimeter.Count);
        Assert.True(bounded.Count < full.Count);
        Assert.Equal(new[] { -2d, -1d }, bounded.Select(path => path.Points[0].Z).Distinct().Order());
        foreach (var path in bounded)
        foreach (var point in Samples(path.Points))
        {
            double clearance = RectangleClearance(point, 0, 0, 20, 16);
            double radius = profile?.RadiusAtDepth(-point.Z) ?? -.5 * point.Z;
            Assert.True(clearance >= radius - 1e-4 && clearance <= width + 1e-4,
                $"depth={-point.Z:R} radius={radius:R} clearance={clearance:R} point={point.X:R},{point.Y:R}");
        }
    }

    [Fact]
    public void WidthBelowCompensatedPerimeterPreservesMandatoryPaths()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [Rectangle(0, 0, 20, 16)];
        var perimeter = Clearing(boundary, false, 0);
        var narrowBand = Clearing(boundary, false, .1);
        Assert.Equal(Geometry(perimeter.Select(path => path.Points)), Geometry(narrowBand.Select(path => path.Points)));
    }

    [Fact]
    public void NarrowNookFillsWhileTheBroadCoreRemainsUncleared()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [[
            new(0,0), new(30,0), new(30,8), new(34,8),
            new(34,12), new(30,12), new(30,20), new(0,20)
        ]];
        var bounded = Clearing(boundary, false, 3.2).Where(path => path.Points[0].Z == -2).ToList();

        for (double x = 31; x <= 33; x += .5)
        for (double y = 9; y <= 11; y += .5)
            Assert.True(DistanceToPaths(new(x, y), bounded) <= 1.001, $"Uncleared nook at {x},{y}");
        Assert.True(DistanceToPaths(new(15, 10), bounded) > 5);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HoleBoundaryParticipatesInBandAndRemainsProtected(bool reverse)
    {
        var outer = Rectangle(0, 0, 30, 30);
        var hole = Rectangle(10, 10, 20, 20).AsEnumerable().Reverse().ToList();
        if (reverse) { outer.Reverse(); hole.Reverse(); }
        var paths = Clearing([outer, hole], false, 2.1);
        Assert.Contains(paths.SelectMany(path => Samples(path.Points)), point => point.X is > 7 and < 10 && point.Y is > 10 and < 20);
        foreach (var path in paths)
        foreach (var point in Samples(path.Points))
        {
            Assert.False(point.X is > 10 and < 20 && point.Y is > 10 and < 20);
            double outerDistance = RectangleClearance(point, 0, 0, 30, 30);
            double holeDistance = Math.Sqrt(Math.Pow(Math.Max(0, Math.Max(10 - point.X, point.X - 20)), 2)
                + Math.Pow(Math.Max(0, Math.Max(10 - point.Y, point.Y - 20)), 2));
            double clearance = Math.Min(outerDistance, holeDistance);
            Assert.True(clearance >= -.5 * point.Z - .001 && clearance <= 2.101,
                $"depth={-point.Z:R} clearance={clearance:R} point={point.X:R},{point.Y:R}");
        }
    }

    [Fact]
    public void FullFillIgnoresBandWidth()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [Rectangle(0, 0, 12, 10)];
        Assert.Equal(Geometry(Clearing(boundary, true, 0).Select(path => path.Points)),
            Geometry(Clearing(boundary, true, 2).Select(path => path.Points)));
    }

    [Fact]
    public void TerminalHoleContourDoesNotFragmentIntoOpenMicroStrokes()
    {
        var hole = Rectangle(10, 10, 20, 20).AsEnumerable().Reverse().ToList();
        var paths = MedialAxisToolpaths.GenerateClearingPassesTagged([Rectangle(0, 0, 30, 30), hole],
            0, 2, Math.PI / 3, 1, bottomRadiusOverride: 0, topRadiusOverride: 2,
            coneLengthOverride: 4, includeInteriorFill: false, detectCurves: true, edgeFillWidth: 2.1);
        Assert.InRange(paths.Count, 1, 36);
        foreach (var path in paths)
        {
            Assert.Equal(path.Points[0], path.Points[^1]);
            Assert.NotEmpty(path.Moves);
            Assert.Equal(path.Points[^1], path.Moves[^1].End);
        }
    }

    [Theory]
    [InlineData(2.0)]
    [InlineData(1.88799999)]
    [InlineData(1.88800001)]
    public void DenseCurvedTerminalContourRemainsWholeNearPitchMultiples(double width)
    {
        static List<PointD> Circle(double radius, bool reverse)
        {
            var ring = Enumerable.Range(0, 512).Select(index =>
            {
                double angle = index * 2 * Math.PI / 512;
                return new PointD(20 + radius * Math.Cos(angle), 20 + radius * Math.Sin(angle));
            }).ToList();
            if (reverse) ring.Reverse();
            return ring;
        }
        var profile = Ball();
        var paths = Clearing([Circle(20, false), Circle(6, true)], false, width, profile);
        Assert.InRange(paths.Count, 1, 30);
        foreach (var path in paths)
        {
            Assert.Equal(path.Points[0], path.Points[^1]);
            double pathLength = path.Points.Zip(path.Points.Skip(1), (a, b) =>
                Math.Sqrt(Math.Pow(b.X-a.X, 2) + Math.Pow(b.Y-a.Y, 2))).Sum();
            Assert.True(pathLength > 1, $"width={width:R} length={pathLength:R} " +
                $"points={path.Points.Count} path={string.Join(';', path.Points.Select(point => $"{point.X:R},{point.Y:R},{point.Z:R}"))}");
            foreach (var point in Samples(path.Points))
            {
                double radius = Math.Sqrt(Math.Pow(point.X-20,2) + Math.Pow(point.Y-20,2));
                double clearance = Math.Min(20-radius, radius-6);
                Assert.InRange(clearance, profile.RadiusAtDepth(-point.Z)-.003, width+.003);
            }
        }
    }

    [Fact]
    public void WidthBarelyAboveCutterRadiusPreservesEveryMandatoryPerimeter()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [Rectangle(0, 0, 20, 16)];
        var profile = Ball();
        var mandatory = Geometry(Clearing(boundary, false, 0, profile).Select(path => path.Points));
        var bounded = Geometry(Clearing(boundary, false, profile.RadiusAtDepth(2) + 1e-8, profile)
            .Select(path => path.Points));
        Assert.All(mandatory, path => Assert.Contains(path, bounded));
    }

    [Fact]
    public void FullPitchFiniteBandInsertsCoherentFrontsWithoutCrossingItsCore()
    {
        var paths = MedialAxisToolpaths.GenerateClearingPassesTagged([Rectangle(0, 0, 60, 90)],
            0, 2, Math.PI / 3, 1, stepOver: 1,
            bottomRadiusOverride: 0, topRadiusOverride: 2, coneLengthOverride: 4,
            includeInteriorFill: false, detectCurves: false, edgeFillWidth: 4.5)
            .Where(path => path.Points[0].Z == -1).ToList();

        // Five nominal one-millimetre square fronts leave corner crescents with
        // this half-millimetre cutter. The finite band needs additional fronts.
        Assert.True(paths.Count > 5);
        Assert.All(paths, path => Assert.Equal(path.Points[0], path.Points[^1]));
        foreach (var point in paths.SelectMany(path => Samples(path.Points)))
            Assert.InRange(RectangleClearance(point, 0, 0, 60, 90), .499, 4.5001);
        for (double x = .75; x < 4.5; x += .1)
        for (double y = .75; y < 4.5; y += .1)
            Assert.True(DistanceToPaths(new(x, y), paths) <= .503,
                $"Finite V-carve band left a corner crescent at ({x:R},{y:R}).");
        Assert.True(DistanceToPaths(new(30, 45), paths) > 20);
        Assert.Contains(paths, path => Math.Abs(path.Points.Min(point => point.X) - 4.5) < 1e-5);
    }

    [Theory]
    [InlineData(512)]
    [InlineData(4096)]
    public void DenseCurvedMandatoryPerimetersKeepMonotoneTravel(int vertices)
    {
        List<PointD> Circle(double radius, bool reverse)
        {
            var ring=Enumerable.Range(0,vertices).Select(i=>new PointD(20+radius*Math.Cos(2*Math.PI*i/vertices),
                20+radius*Math.Sin(2*Math.PI*i/vertices))).ToList();
            if(reverse) ring.Reverse();
            return ring;
        }
        var profile=Ball();
        double radius=profile.RadiusAtDepth(1);
        var paths=Clearing([Circle(20,false),Circle(6,true)],false,2,profile)
            .Where(path=>path.Points[0].Z==-1).ToList();
        foreach(double expectedInset in new[]{radius,2d})
        {
        var fronts=paths.Where(path=>Math.Abs(path.Points.Average(point=>
            {
                double radial=Math.Sqrt(Math.Pow(point.X-20,2)+Math.Pow(point.Y-20,2));
                return Math.Min(20-radial,radial-6);
            })-expectedInset)<.003).ToList();
        Assert.Equal(2,fronts.Count);
        foreach(var path in fronts)
        {
            var points=path.Points;
            double direction=Math.Sign(Enumerable.Range(1,points.Count-1).Sum(i=>
                (double)points[i-1].X*points[i].Y-(double)points[i].X*points[i-1].Y)),negative=0;
            int reversed=0;
            for(int i=1;i<points.Count;i++)
            {
                var a=points[i-1];var b=points[i];
                double rx=(a.X+b.X)/2-20,ry=(a.Y+b.Y)/2-20;
                double advance=((b.X-a.X)*-ry+(b.Y-a.Y)*rx)/Math.Sqrt(rx*rx+ry*ry);
                if(advance*direction < -1/PathUtils.Scale)
                {
                    reversed++;negative-=advance*direction;
                    if(reversed<=4) Console.WriteLine($"[VCarve.MandatoryReversal] edge={i-1} "
                        + $"from=({a.X:R},{a.Y:R}) to=({b.X:R},{b.Y:R}) "
                        + $"previous=({points[(i+points.Count-3)%(points.Count-1)].X:R},{points[(i+points.Count-3)%(points.Count-1)].Y:R}) "
                        + $"next=({points[(i+1)%points.Count].X:R},{points[(i+1)%points.Count].Y:R}) advance={advance*direction:R}");
                }
            }
            Console.WriteLine($"[VCarve.CanonicalContour] vertices={vertices} inset={expectedInset:R} points={points.Count} "
                + $"reversedEdges={reversed} negativeTravel={negative:R}");
            Assert.Equal(0,reversed);
        }
        }
    }

    [Fact]
    public void BandLeavesFinalMedialDetailUnchangedAndReportsProgress()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = [Rectangle(0, 0, 12, 10)];
        var provider = new StraightSpokeProvider();
        var reports = new List<(double Fraction, string Stage, bool Indeterminate)>();
        List<TaggedToolpath> Generate(double width) => MedialAxisToolpaths.GenerateVCarveTagged(
            provider, boundary, 0, 2, Math.PI / 3, 1,
            bottomRadiusOverride: 0, topRadiusOverride: 2, coneLengthOverride: 4,
            includeInteriorFill: false, detectCurves: false, edgeFillWidth: width,
            progress: (fraction, stage, indeterminate) => reports.Add((fraction, stage, indeterminate)));
        var perimeter = Generate(0);
        var band = Generate(2);

        Assert.Equal(Geometry(perimeter.Where(path => path.Category == "final-carve").Select(path => path.Points)),
            Geometry(band.Where(path => path.Category == "final-carve").Select(path => path.Points)));
        Assert.Contains(reports, update => update.Stage.Contains("Clearing pass") && !update.Indeterminate);
        Assert.Contains(reports, update => update.Stage.Contains("Refining depth"));
        Assert.All(reports, update => Assert.InRange(update.Fraction, 0, 1));
        Assert.Equal(1, reports[^1].Fraction);
    }

    private static List<TaggedToolpath> Clearing(IReadOnlyList<IReadOnlyList<PointD>> boundary,
        bool full, double width, VCarveCutterProfile? profile = null)
        => MedialAxisToolpaths.GenerateClearingPassesTagged(boundary, 0, 2, Math.PI / 3, 1,
            bottomRadiusOverride: 0, topRadiusOverride: 2, coneLengthOverride: 4,
            includeInteriorFill: full, detectCurves: false, cutterProfile: profile, edgeFillWidth: width);

    private static VCarveCutterProfile Ball() => new([
        new(CarvingCutterSectionKind.Hemisphere, 0, .4, .4),
        new(CarvingCutterSectionKind.Linear, .4, 3, 20)
    ], 20.4);

    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0,y0),new(x1,y0),new(x1,y1),new(x0,y1)];

    private static double RectangleClearance(Point3D point, double x0, double y0, double x1, double y1)
        => Math.Min(Math.Min(point.X-x0,x1-point.X),Math.Min(point.Y-y0,y1-point.Y));

    private static IEnumerable<Point3D> Samples(IReadOnlyList<Point3D> points)
    {
        for (int i=1; i<points.Count; i++)
        for (int j=0; j<=8; j++)
        {
            double t=j/8d; var a=points[i-1]; var b=points[i];
            yield return new(a.X+(b.X-a.X)*t,a.Y+(b.Y-a.Y)*t,a.Z);
        }
    }

    private static double DistanceToPaths(PointD point, IEnumerable<TaggedToolpath> paths)
    {
        double minimum=double.PositiveInfinity;
        foreach(var path in paths)
        for(int i=1;i<path.Points.Count;i++)
        {
            var a=path.Points[i-1];var b=path.Points[i];
            double dx=b.X-a.X,dy=b.Y-a.Y,length2=dx*dx+dy*dy;
            double t=length2>0?Math.Clamp(((point.x-a.X)*dx+(point.y-a.Y)*dy)/length2,0,1):0;
            minimum=Math.Min(minimum,Math.Sqrt(Math.Pow(point.x-a.X-t*dx,2)+Math.Pow(point.y-a.Y-t*dy,2)));
        }
        return minimum;
    }

    private static string[] Geometry(IEnumerable<IReadOnlyList<Point3D>> paths)
        => paths.Select(path=>string.Join(";",path.Select(point=>$"{point.X:R},{point.Y:R},{point.Z:R}"))).ToArray();

    private sealed class StraightSpokeProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle = 3 * Math.PI / 4, bool useBigIntegers = true)
        {
            double x = (boundary.Min(point => point.x) + boundary.Max(point => point.x)) / 2;
            double y0 = boundary.Min(point => point.y), y1 = boundary.Max(point => point.y);
            return [new(new(x, y0, 0), new(x, (y0 + y1) / 2, (y1 - y0) / 2))];
        }
    }
}
