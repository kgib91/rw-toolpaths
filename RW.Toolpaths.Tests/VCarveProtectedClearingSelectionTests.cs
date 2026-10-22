using System.Collections;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveProtectedClearingSelectionTests
{
    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, 1)]
    [InlineData(true, 1)]
    [InlineData(false, 2)]
    [InlineData(true, 2)]
    public void LocalIntervalsContainFormerBufferedChordsWithSignedHolesIslandsAndMirrors(bool mirror, int order)
    {
        IReadOnlyList<PointD>[] rings = [Box(0, 0, 20, 20), Box(5, 5, 15, 15, true),
            Box(8, 8, 12, 12), Box(9, 9, 11, 11, true), Box(40, 40, 50, 50)];
        if (mirror) rings = rings.Select(r => (IReadOnlyList<PointD>)r.Select(p => new PointD(-p.x, p.y)).ToArray()).ToArray();
        if (order == 1) rings = rings.Reverse().ToArray();
        if (order == 2) rings = rings.Skip(2).Concat(rings.Take(2)).ToArray();
        var selector = new PreparedSweepExclusion(rings);
        const double radius = .75;
        var mask = PathUtils.ToClipper(PolygonOps.Buffer(rings, radius, 1 / PathUtils.Scale));
        var outerMask = PathUtils.ToClipper(PolygonOps.Buffer(rings, radius + 12 / PathUtils.Scale, 1 / PathUtils.Scale));
        int checkedOldChords = 0;
        foreach (double y in new[] { -.75, 0, 4.5, 5, 7, 8.5, 10, 15, 20, 20.75, 45 })
        {
            PointD a = new(mirror ? 55 : -5, y), b = new(mirror ? -55 : 55, y);
            var intervals = selector.ContactIntervals(a, b, radius + 2 / PathUtils.Scale);
            var clipper = new Clipper64();
            clipper.AddOpenSubject(new Paths64 { new() { Grid(a), Grid(b) } }); clipper.AddClip(mask);
            var closed = new Paths64(); var open = new Paths64();
            clipper.Execute(ClipType.Intersection, FillRule.NonZero, closed, open);
            foreach (var stroke in open)
            for (int point = 1; point < stroke.Count; point++)
            {
                double first = Parameter(a, b, new(stroke[point - 1].X / PathUtils.Scale, stroke[point - 1].Y / PathUtils.Scale));
                double last = Parameter(a, b, new(stroke[point].X / PathUtils.Scale, stroke[point].Y / PathUtils.Scale));
                Assert.Contains(intervals, interval => interval.Start <= Math.Min(first, last) + 1e-10
                    && interval.End >= Math.Max(first, last) - 1e-10);
                checkedOldChords++;
            }
            foreach (var interval in intervals)
            {
                PointD At(double at) => new(a.x + (b.x - a.x) * at, a.y + (b.y - a.y) * at);
                // Independently prove the entire selected chord stays local,
                // including hole gaps, against an outer polygon envelope.
                var locality = new Clipper64();
                locality.AddOpenSubject(new Paths64 { new() { Grid(At(interval.Start)), Grid(At(interval.End)) } });
                locality.AddClip(outerMask);
                var localClosed = new Paths64(); var remainder = new Paths64();
                locality.Execute(ClipType.Difference, FillRule.NonZero, localClosed, remainder);
                Assert.Empty(remainder);
            }
            foreach (var interval in intervals)
            foreach (double at in new[] { interval.Start, (interval.Start + interval.End) / 2, interval.End })
            {
                PointD point = new(a.x + (b.x - a.x) * at, a.y + (b.y - a.y) * at);
                Assert.True(DistanceToFilled(rings, point) <= radius + 8 / PathUtils.Scale,
                    $"Excess local selection at ({point.x:R},{point.y:R}) interval={interval}.");
            }
        }
        Assert.True(checkedOldChords > 15);
    }

    [Fact]
    public void WholeInteriorVisitPreservesPointsMovesIdentityCategoryAndDepthWithoutFarOceanExtension()
    {
        var selector = new PreparedSweepExclusion([Box(0, 0, 3, 3)]);
        var inside = Visit([new(.25, .25, -1), new(1, 1, -1), new(2, 1, -1)], 17);
        var crossing = Visit([new(-100, 1, -1), new(100, 1, -1)], 18);
        var outside = Visit([new(30, 30, -1), new(40, 40, -1)], 19);
        var logs = new List<string>();
        var selected = Select([inside, crossing, outside], selector, logs);
        Assert.Equal(2, selected.Count);
        Assert.Same(inside.Points, selected[0].Points);
        Assert.Same(inside.Moves, selected[0].Moves);
        Assert.Equal(inside.VisitIdentity, selected[0].VisitIdentity);
        Assert.Equal(inside.Category, selected[0].Category);
        Assert.Equal(inside.DepthPassIndex, selected[0].DepthPassIndex);
        Assert.Equal(crossing.VisitIdentity, selected[1].VisitIdentity);
        Assert.InRange(selected[1].Points[0].X, -1.009, -1.007);
        Assert.InRange(selected[1].Points[^1].X, 4.007, 4.009);
        Assert.All(selected[1].Points, p => Assert.Equal(-1, p.Z));
        Assert.Contains(logs, l => l.Contains("globalBuffer=False") && l.Contains("event=complete"));
        Assert.Contains(logs, l => l.Contains("midpointInside=") && l.Contains("candidateEdges="));
    }

    [Fact]
    public void GapReentryRemainsSeparateAndOriginalSelectedVerticesSurvive()
    {
        var selector = new PreparedSweepExclusion([Box(0, 0, 2, 2), Box(10, 0, 12, 2)]);
        var path = Visit([new(-5, 1, -1), new(1, 1, -1), new(6, 1, -1), new(11, 1, -1), new(18, 1, -1)], 23);
        var selected = Select([path], selector);
        Assert.Equal(2, selected.Count);
        Assert.Contains(path.Points[1], selected[0].Points);
        Assert.Contains(path.Points[3], selected[1].Points);
        Assert.DoesNotContain(path.Points[2], selected.SelectMany(p => p.Points));
        Assert.True(selected[0].Points[^1].X < 4 && selected[1].Points[0].X > 8);
        Assert.All(selected, p => Assert.Equal(23, p.VisitIdentity));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.001)]
    [InlineData(1)]
    public void StationaryRadiusAndParallelTangencyAreConservative(double radius)
    {
        var selector = new PreparedSweepExclusion([Box(0, 0, 10, 10)]);
        Assert.Equal((0d, 1d), Assert.Single(selector.ContactIntervals(new(5, 5), new(5, 5), radius)));
        Assert.Equal((0d, 1d), Assert.Single(selector.ContactIntervals(new(-radius, 5), new(-radius, 5), radius)));
        Assert.Empty(selector.ContactIntervals(new(-radius - .01, 5), new(-radius - .01, 5), radius));
        Assert.NotEmpty(selector.ContactIntervals(new(-radius, -20), new(-radius, 30), radius));
        var stationary = Visit([new(5, 5, -1), new(5, 5, -1)], 31);
        Assert.Same(stationary.Points, Assert.Single(Select([stationary], selector)).Points);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LongNearCornerTangentAndShortTranslatedChordRetainRequiredContact(bool mirror)
    {
        double Sign(double x) => mirror ? -x : x;
        var rings = new[] { (IReadOnlyList<PointD>)Box(1200, 1200, 1201, 1201)
            .Select(p => new PointD(Sign(p.x), p.y)).ToArray() };
        var selector = new PreparedSweepExclusion(rings);
        double offset = 1 / Math.Sqrt(2);
        PointD a = new(Sign(-1800), 4200 - 2 * offset), b = new(Sign(4200), -1800 - 2 * offset);
        var interval = Assert.Single(selector.ContactIntervals(a, b, 1));
        PointD tangent = new(Sign(1200 - offset), 1200 - offset);
        double center = Parameter(a, b, tangent);
        Assert.True(interval.Start < center && interval.End > center);
        Assert.Contains(selector.ContactIntervals(tangent, new(tangent.x + Sign(.001), tangent.y), 1),
            i => i.Start == 0);
        Assert.Empty(selector.ContactIntervals(new(Sign(1198), 1198), new(Sign(1198.001), 1198), 1));
    }

    [Fact]
    public void EmptyMaskNonfiniteRadiusAndCancellationRejectConsistently()
    {
        var empty = new PreparedSweepExclusion(Array.Empty<IReadOnlyList<PointD>>());
        Assert.Empty(empty.ContactIntervals(new(0, 0), new(10, 0), 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => empty.ContactIntervals(new(0, 0), new(1, 0), double.NaN));
        Assert.Throws<ArgumentException>(() => empty.ContactIntervals(new(double.NaN, 0), new(1, 0), 1));
        using var canceled = new CancellationTokenSource(); canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => empty.ContactIntervals(new(0, 0), new(1, 0), 1, canceled.Token));
        var selector = new PreparedSweepExclusion([Box(0, 0, 10, 10)]);
        using var midRun = new CancellationTokenSource();
        var points = new CancelingPoints(Enumerable.Range(0, 1000).Select(i => new Point3D(2 + i * .001, 2, -1)).ToArray(), midRun);
        var path = Visit([new(2, 2, -1), new(3, 2, -1)], 41) with { Points = points };
        Assert.Throws<OperationCanceledException>(() => VCarveToolpaths.SelectProtectedClearing([path], selector,
            Tool(), 3, 0, midRun.Token));
        Assert.InRange(points.Reads, 300, 556);
    }

    private static List<TaggedToolpath> Select(IReadOnlyList<TaggedToolpath> source, PreparedSweepExclusion selector,
        List<string>? logs = null) => VCarveToolpaths.SelectProtectedClearing(source, selector, Tool(), 3, 0,
            CancellationToken.None, logs is null ? null : logs.Add);
    private static ToolGeometry Tool() => new(3)
    {
        VCarveProfile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 3, 3)], 3),
        ConeLength = 3, TipAngleRadians = Math.PI / 2,
    };
    private static TaggedToolpath Visit(IReadOnlyList<Point3D> points, int identity) => new(points,
        Array.Empty<ToolpathSpan>(), 7, "clearing", 2, identity, detectCurves: false);
    private static Point64 Grid(PointD p) => new((long)Math.Round(p.x * PathUtils.Scale), (long)Math.Round(p.y * PathUtils.Scale));
    private static IReadOnlyList<PointD> Box(double x0, double y0, double x1, double y1, bool reverse = false)
    {
        PointD[] points = [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
        if (reverse) Array.Reverse(points);
        return points;
    }
    private static double Parameter(PointD a, PointD b, PointD p)
    {
        double dx = b.x - a.x, dy = b.y - a.y;
        return ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy);
    }
    private static double DistanceToFilled(IReadOnlyList<IReadOnlyList<PointD>> rings, PointD point)
    {
        int winding = 0; double distance = double.PositiveInfinity;
        foreach (var ring in rings)
        {
            for (int i = 0; i < ring.Count; i++)
            {
                PointD a = ring[i], b = ring[(i + 1) % ring.Count];
                double dx = b.x - a.x, dy = b.y - a.y;
                double side = dx * (point.y - a.y) - dy * (point.x - a.x);
                if (a.y <= point.y && b.y > point.y && side > 0) winding++;
                else if (a.y > point.y && b.y <= point.y && side < 0) winding--;
                double len = dx * dx + dy * dy;
                double at = len == 0 ? 0 : Math.Clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / len, 0, 1);
                distance = Math.Min(distance, Math.Sqrt(Math.Pow(point.x - a.x - dx * at, 2) + Math.Pow(point.y - a.y - dy * at, 2)));
            }
        }
        return winding == 0 ? distance : 0;
    }
    private sealed class CancelingPoints(Point3D[] source, CancellationTokenSource cancel) : IReadOnlyList<Point3D>
    {
        public int Reads { get; private set; }
        public int Count => source.Length;
        public Point3D this[int index]
        {
            get { if (++Reads == 300) cancel.Cancel(); return source[index]; }
        }
        public IEnumerator<Point3D> GetEnumerator() => ((IEnumerable<Point3D>)source).GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }
}
