using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveOrderedDiscHullTests
{
    [Fact]
    public void OneIdenticalDiscRowRetainsTheIndependentDuplicateRowHullExactly()
    {
        using var hull=new VCarveOrderedDiscHull();
        foreach(int count in new[]{16,32,128,256})
        foreach(double radius in new[]{1e-7,.001,.1,1,100})
        {
            var circle=Circle(count);var center=new Point3D(.5/PathUtils.Scale,123.4567,0);
            var raw=new List<Point64>();AddDisc(raw,circle,center,radius);AddDisc(raw,circle,center,radius);
            hull.BeginCircle(circle);hull.AddDisc(center,radius);
            Assert.Equal(OriginalHull(raw).ToArray(),hull.FinishHull().ToArray());
        }
    }

    [Theory]
    [InlineData(8)]
    [InlineData(32)]
    [InlineData(256)]
    [InlineData(1024)]
    public void RandomQuantizedEndpointDiscsMatchIndependentOriginalHullInCompleteOrder(int count)
    {
        var random = new Random(12071 + count);
        var circle = Circle(count);
        using var ordered = new VCarveOrderedDiscHull();
        for (int sample = 0; sample < 96; sample++)
        {
            var a = new Point3D((random.NextDouble() - .5) * 1000,
                (random.NextDouble() - .5) * 1000, 0);
            if (sample % 4 == 0) a = new((sample - 48 + .5) / PathUtils.Scale, -.5 / PathUtils.Scale, 0);
            double radiusA = sample % 12 == 0 ? 0 : Math.Pow(10, -7 + 9 * random.NextDouble());
            var b = sample % 11 == 0 ? a : new Point3D(a.X + random.NextDouble() * 12 - 6, a.Y + random.NextDouble() * 12 - 6, 0);
            double radiusB = sample % 11 == 0 ? radiusA : Math.Pow(10, -7 + 9 * random.NextDouble());
            var raw = new List<Point64>();
            ordered.BeginCircle(circle);
            if (sample % 3 != 0) { ordered.AddDisc(a, radiusA); AddDisc(raw, circle, a, radiusA); }
            if (sample % 3 == 2) { ordered.AddDisc(b, radiusB); AddDisc(raw, circle, b, radiusB); }
            Assert.Equal(OriginalHull(raw).ToArray(), ordered.FinishHull().ToArray());
        }
        Assert.Equal(1, ordered.CircleOrderBuilds);
    }

    [Fact]
    public void QuantizedXPairsAndLargeTieGroupsAreSortedByIntegerY()
    {
        using var ordered = new VCarveOrderedDiscHull();
        var pair = new[] { (X: 0d, Y: 1d), (X: 1e-9, Y: -1d) };
        var larger = new[] { (X: -3e-9, Y: 1d), (X: -2e-9, Y: -1d), (X: -1e-9, Y: .5d),
            (X: 0d, Y: -.5d), (X: 1e-9, Y: 0d), (X: 2e-9, Y: .25d) };
        foreach (var circle in new[] { pair, larger })
        {
            var raw = new List<Point64>();
            ordered.BeginCircle(circle); ordered.AddDisc(new(0, 0, 0), 1);
            AddDisc(raw, circle, new(0, 0, 0), 1);
            Assert.Equal(OriginalHull(raw).ToArray(), ordered.FinishHull().ToArray());
        }
        Assert.Equal(1, ordered.PairSwaps);
        Assert.Equal(6, ordered.MaximumTieGroup);
    }

    [Fact]
    public void ReversedDiscsNegativeRadiusAndHugeTranslationsKeepOriginalExtremeArithmetic()
    {
        var circle = Circle(64);
        using var ordered = new VCarveOrderedDiscHull();
        foreach (var a in new[] { new Point3D(-.5 / PathUtils.Scale, .5 / PathUtils.Scale, 0),
            new Point3D(1e12, -1e12, 0), new Point3D(1e15, -1e15, 0) })
        foreach (bool reverse in new[] { false, true })
        {
            var b = new Point3D(a.X + .01, a.Y - .02, 0);
            var raw = new List<Point64>(); ordered.BeginCircle(circle);
            if (reverse) { ordered.AddDisc(b, 2); AddDisc(raw, circle, b, 2); }
            ordered.AddDisc(a, -1); AddDisc(raw, circle, a, -1);
            if (!reverse) { ordered.AddDisc(b, 2); AddDisc(raw, circle, b, 2); }
            Assert.Equal(OriginalHull(raw).ToArray(), ordered.FinishHull().ToArray());
        }
        Assert.True(ordered.SortFallbacks >= 6);
    }

    [Fact]
    public void PointRowsDuplicatesAndReturnedHullRemainDetachedFromPooledScratch()
    {
        using var ordered = new VCarveOrderedDiscHull();
        ordered.BeginCircle(Circle(16)); ordered.AddPoint(new Point64(4, 9)); ordered.AddPoint(new Point64(4, 9));
        Assert.Equal(new[] { new Point64(4, 9) }, ordered.FinishHull().ToArray());
        ordered.BeginCircle(Circle(32)); ordered.AddDisc(new(2, -3, 0), 1);
        var retained = ordered.FinishHull(); var snapshot = retained.ToArray();
        ordered.BeginCircle(Circle(1024)); ordered.AddDisc(new(100, -100, 0), .000001);
        ordered.AddDisc(new(101, -99, 0), 4); _ = ordered.FinishHull();
        ordered.Dispose();
        Assert.Equal(snapshot, retained.ToArray());
        Assert.True(ordered.DuplicatesSkipped > 0);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CompleteLowerUpperBuildOutputsMatchForProfilesSpansAndConstantRuns(bool upper, bool accelerate)
    {
        const double surface = 7.125;
        var paths = new[]
        {
            Path([new(.5 / PathUtils.Scale, -.5 / PathUtils.Scale, surface), new(.137, -.421, surface - .0001),
                new(1.24, .619, surface - .8), new(3.47, -.729, surface - 2), new(6.25, .253, surface - 3)]),
            Path(Enumerable.Range(0, 20).Select(index => new Point3D(8 + index * .31, 3 + Math.Sin(index * .3), surface - .45)).ToArray()),
            new TaggedToolpath { Points = new Point3D[] { new(-2, 0, surface - .7), new(-1, 1, surface - .8), new(0, 2, surface - .9) },
                Moves = Array.Empty<ToolMove>(), Spans = new ToolpathSpan[] { new(0, 1, ToolpathSpanKind.Retract), new(1, 2, ToolpathSpanKind.Cut) }, Category = "clearing" },
            Path([new(20, 20, surface - .04), new(20, 20, surface - .04), new(20.00001, 20, surface - .041)]),
        };
        foreach (var profile in new[]
        {
            new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875), new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 21.5875),
            new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 2, 5)], 5),
            new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, .4, .4, 1), new(CarvingCutterSectionKind.Linear, .4, 2, 4)], 5),
        })
        {
            var original = VCarvePhysicalSweep.Build(paths, profile, surface, .00075, upperBound: upper,
                accelerateConstantDepthRuns: accelerate, orderedDiscHulls: false);
            var candidate = VCarvePhysicalSweep.Build(paths, profile, surface, .00075, upperBound: upper,
                accelerateConstantDepthRuns: accelerate, orderedDiscHulls: true);
            Assert.Equal(Encode(original), Encode(candidate));
            var tool = new ToolGeometry(3) { VCarveProfile = profile };
            var forwarded = VCarvePhysicalSweep.Build(paths, tool, 3, surface, .00075, upperBound: upper,
                accelerateConstantDepthRuns: accelerate, orderedDiscHulls: true);
            Assert.Equal(Encode(candidate), Encode(forwarded));
        }
    }

    private static TaggedToolpath Path(Point3D[] points) => new()
    { Points = points, Moves = Array.Empty<ToolMove>(), Spans = Array.Empty<ToolpathSpan>(), Category = "clearing" };
    private static (double X, double Y)[] Circle(int count) => Enumerable.Range(0, count)
        .Select(index => (Math.Cos(2 * Math.PI * index / count), Math.Sin(2 * Math.PI * index / count))).ToArray();
    private static void AddDisc(List<Point64> points, (double X, double Y)[] circle, Point3D center, double radius)
    { foreach (var unit in circle) points.Add(new Point64((long)Math.Round((center.X + radius * unit.X) * PathUtils.Scale), (long)Math.Round((center.Y + radius * unit.Y) * PathUtils.Scale))); }
    // Independent literal old general-point algorithm, not the prototype's rows.
    private static Path64 OriginalHull(List<Point64> points)
    {
        points.Sort((a, b) => a.X == b.X ? a.Y.CompareTo(b.Y) : a.X.CompareTo(b.X));
        var unique = points.Distinct().ToArray();
        if (unique.Length <= 2) return new Path64(unique);
        static Int128 Cross(Point64 a, Point64 b, Point64 c)
            => (Int128)(b.X - a.X) * (c.Y - a.Y) - (Int128)(b.Y - a.Y) * (c.X - a.X);
        var hull = new Path64();
        foreach (var point in unique)
        { while (hull.Count >= 2 && Cross(hull[^2], hull[^1], point) <= 0) hull.RemoveAt(hull.Count - 1); hull.Add(point); }
        int lower = hull.Count;
        for (int index = unique.Length - 2; index >= 0; index--)
        { var point = unique[index]; while (hull.Count > lower && Cross(hull[^2], hull[^1], point) <= 0) hull.RemoveAt(hull.Count - 1); hull.Add(point); }
        hull.RemoveAt(hull.Count - 1); return hull;
    }
    private static byte[] Encode(IReadOnlyList<List<PointD>> rings)
    {
        using var stream = new MemoryStream(); using var writer = new BinaryWriter(stream);
        writer.Write(rings.Count);
        foreach (var ring in rings) { writer.Write(ring.Count); foreach (var point in ring) { writer.Write(point.x); writer.Write(point.y); } }
        writer.Flush(); return stream.ToArray();
    }
}
