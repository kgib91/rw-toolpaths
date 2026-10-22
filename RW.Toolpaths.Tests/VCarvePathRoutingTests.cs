using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarvePathRoutingTests
{
    private static readonly ToolGeometry Cone = new(5)
    {
        BottomRadius = 0, TopRadius = 5, ConeLength = 10, TipAngleRadians = 2 * Math.Atan(.5)
    };

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, true)]
    public void PublicStrategyLinksOnlyWithinTheOffsetBandAndRetainsSweepMetadataOption(bool detectCurves, bool calculateSweptArea)
    {
        var plan = VCarveToolpaths.Generate([new MillingRegion(Rectangle(0, 0, 20, 16))], Cone,
            new DepthSchedule(4, 2) { SurfaceZ = 10 }, new VCarveOptions
            {
                IncludeInteriorFill = false, EdgeFillWidth = 1.9, StockToLeave = .5,
                StepOver = .4, DetectCurves = detectCurves, CalculateSweptArea = calculateSweptArea
            }, new EmptyProvider());
        var links = plan.Toolpaths.Where(IsLink).ToArray();

        Assert.NotEmpty(links);
        Assert.Equal(calculateSweptArea, plan.SweptArea.Count > 0);
        Assert.Equal(Enumerable.Range(0, plan.Toolpaths.Count), plan.Toolpaths.Select(path => path.VisitIdentity!.Value));
        foreach (var link in links)
        {
            Assert.Equal(0, link.DepthPassIndex);
            Assert.All(link.Points, point => Assert.Equal(8, point.Z));
            for (int i = 0; i <= 20; i++)
            {
                double t = i / 20d;
                double x = link.Points[0].X + t * (link.Points[1].X - link.Points[0].X);
                double y = link.Points[0].Y + t * (link.Points[1].Y - link.Points[0].Y);
                Assert.InRange(Math.Min(Math.Min(x, 20 - x), Math.Min(y, 16 - y)), 1.499, 1.901);
            }
        }
        Assert.True(plan.Toolpaths.Where(path => path.Category == "clearing")
            .Select(path => path.DepthPassIndex!.Value).SequenceEqual(plan.Toolpaths
                .Where(path => path.Category == "clearing").Select(path => path.DepthPassIndex!.Value).Order()));
    }

    [Fact]
    public void ReordersVisitsAndRotatesClosedSeamsWithoutChangingAnyCutChord()
    {
        var source = new[] { 0d, 100, 10, 90, 20, 80 }.Select(x => Loop(x, 0, x + 2, 2, -2, 0)).ToList();
        var routed = Route(source, [Rectangle(-5, -5, 110, 10)], new() { IncludeInteriorFill = false });

        Assert.True(Transfer(routed) < Transfer(source) * .6,
            $"Expected shorter actual transfers: before={Transfer(source)}, after={Transfer(routed)}");
        Assert.Equal(Chords(source), Chords(routed));
        Assert.All(routed, path => Assert.Equal(path.Points[0], path.Points[^1]));
    }

    [Fact]
    public void ReversesOnlyConstantDepthOpenCutsAndPreservesDepthAndCategoryOrder()
    {
        TaggedToolpath[] source =
        [
            Path([new(0, 0, -1), new(1, 0, -1)], "clearing", 0),
            Path([new(10, 0, -1), new(2, 0, -1)], "clearing", 0),
            Path([new(3, 0, 0), new(2, 0, -2)], "clearing", 1,
                [new ToolpathSpan(0, 1, ToolpathSpanKind.Ramp)]),
            Path([new(9, 0, -.5), new(2, 0, -2)], "final-carve", null)
        ];
        var routed = Route(source, [Rectangle(-5, -5, 20, 10)], new() { IncludeInteriorFill = false });

        Assert.Equal(Chords(source), Chords(routed));
        Assert.Equal(new int?[] { 0, 0, 1, null }, routed.Select(path => path.DepthPassIndex));
        Assert.Equal(2, routed[1].Points[0].X);
        Assert.Equal(source[2].Points, routed[2].Points);
        Assert.Equal(source[2].Spans, routed[2].Spans);
        Assert.Equal(source[3].Points, routed[3].Points);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ShortClearingLinksPreserveAllCutsAndStayInsidePhysicalLayerAndFillBand(bool rounded)
    {
        var tool = rounded ? new ToolGeometry(3)
        {
            VCarveProfile = new VCarveCutterProfile(
                [new(CarvingCutterSectionKind.Hemisphere, 0, .4, .4),
                 new(CarvingCutterSectionKind.Linear, .4, 3, 20)], 20.4)
        } : Cone;
        double radius = rounded ? tool.VCarveProfile!.RadiusAtDepth(2) : 1;
        double spacing = rounded ? .1 : .3;
        double width = radius + spacing;
        TaggedToolpath[] source = [Loop(radius, radius, 20 - radius, 16 - radius, -2, 0),
            Loop(width, width, 20 - width, 16 - width, -2, 0)];
        var options = new VCarveOptions { IncludeInteriorFill = false, EdgeFillWidth = width };
        var routed = VCarvePathRouter.Route(source, [Rectangle(0, 0, 20, 16)], tool,
            new DepthSchedule(4, 2), options, width, null);
        var links = routed.Where(IsLink).ToArray();

        Assert.Single(links);
        Assert.Equal(Chords(source), Chords(routed.Where(path => !IsLink(path))));
        Assert.True(Transfer(routed) < Transfer(source));
        foreach (var link in links)
        {
            var a = link.Points[0]; var b = link.Points[^1];
            double pitch = rounded ? .4 * .8 : .4 * 2 * radius;
            Assert.True(Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2)) <= pitch + 1e-6);
            for (int i = 0; i <= 20; i++)
            {
                double t = i / 20d, x = a.X + t * (b.X - a.X), y = a.Y + t * (b.Y - a.Y);
                double clearance = Math.Min(Math.Min(x, 20 - x), Math.Min(y, 16 - y));
                Assert.InRange(clearance, radius - .0011, width + .0011);
                Assert.Equal(-2, a.Z + t * (b.Z - a.Z));
            }
        }
    }

    [Fact]
    public void ShortLinkCannotCrossTheUnfilledCore()
    {
        TaggedToolpath[] source =
        [
            Path([new(4, 1.4, -2), new(5, 1.4, -2)], "clearing", 0),
            Path([new(5, 1.6, -2), new(6, 1.6, -2)], "clearing", 0)
        ];
        var routed = Route(source, [Rectangle(0, 0, 10, 3)], new() { IncludeInteriorFill = false, EdgeFillWidth = 1.4 });

        Assert.DoesNotContain(routed, IsLink);
        Assert.Equal(Chords(source), Chords(routed));
    }

    [Fact]
    public void ShortLinkCannotCutAcrossARetainedHoleCorner()
    {
        double corner = 5 - Math.Sqrt(.5);
        TaggedToolpath[] source =
        [
            Path([new(corner - .01, corner - .01, -2), new(corner, corner, -2)], "clearing", 0),
            Path([new(4, 5, -2), new(4, 5.01, -2)], "clearing", 0)
        ];
        var hole = Rectangle(5, 5, 10, 10);
        hole.Reverse();
        var routed = Route(source, [Rectangle(0, 0, 20, 20), hole], new() { IncludeInteriorFill = true });
        Assert.DoesNotContain(routed, IsLink);
        Assert.Equal(Chords(source), Chords(routed));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.5)]
    public void PerimetersOnlyOrWidthBelowCutterRadiusCannotAddClearing(double width)
    {
        TaggedToolpath[] source = [Loop(1, 1, 19, 15, -2, 0), Loop(1.2, 1.2, 18.8, 14.8, -2, 0)];
        var routed = Route(source, [Rectangle(0, 0, 20, 16)], new() { IncludeInteriorFill = false, EdgeFillWidth = width });
        Assert.DoesNotContain(routed, IsLink);
        Assert.Equal(Chords(source), Chords(routed));
    }

    [Fact]
    public void DoesNotAddLinksAcrossDepthOrRegionBoundaries()
    {
        TaggedToolpath[] source = [Loop(1, 1, 19, 15, -1, 0), Loop(1.2, 1.2, 18.8, 14.8, -2, 1),
            Loop(1.4, 1.4, 18.6, 14.6, -2, 1) with { RegionIndex = 1 }];
        var routed = Route(source, [Rectangle(0, 0, 20, 16)], new() { IncludeInteriorFill = true });
        Assert.DoesNotContain(routed, IsLink);
        Assert.Equal(Chords(source), Chords(routed));
    }

    [Fact]
    public void RoutingHonorsCancellation()
    {
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(() => VCarvePathRouter.Route(
            [Loop(1, 1, 19, 15, -2, 0)], [Rectangle(0, 0, 20, 16)], Cone,
            new DepthSchedule(4, 2), new VCarveOptions(), 0, null, cancelled.Token));
    }

    private static List<TaggedToolpath> Route(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<IReadOnlyList<PointD>> boundary, VCarveOptions options)
        => VCarvePathRouter.Route(source, boundary, Cone, new DepthSchedule(4, 2), options, options.EdgeFillWidth, null);

    private static TaggedToolpath Path(IReadOnlyList<Point3D> points, string category, int? depth,
        IReadOnlyList<ToolpathSpan>? spans = null)
        => new(points, spans ?? Array.Empty<ToolpathSpan>(), 0, category, depth, detectCurves: false);

    private static TaggedToolpath Loop(double x0, double y0, double x1, double y1, double z, int depth)
        => Path([new(x0, y0, z), new(x1, y0, z), new(x1, y1, z), new(x0, y1, z), new(x0, y0, z)], "clearing", depth);

    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];

    private static bool IsLink(TaggedToolpath path) => path.Spans.Any(span => span.Kind == ToolpathSpanKind.Link);

    private static string[] Chords(IEnumerable<TaggedToolpath> paths) => paths.SelectMany(path =>
        path.Points.Zip(path.Points.Skip(1), (a, b) =>
        {
            string first = $"{a.X:R},{a.Y:R},{a.Z:R}", second = $"{b.X:R},{b.Y:R},{b.Z:R}";
            return $"{path.RegionIndex}:{path.Category}:{path.DepthPassIndex}:" +
                (string.CompareOrdinal(first, second) < 0 ? first + ";" + second : second + ";" + first);
        })).Order().ToArray();

    private static double Transfer(IReadOnlyList<TaggedToolpath> paths)
        => Enumerable.Range(1, Math.Max(0, paths.Count - 1)).Sum(i =>
            Math.Sqrt(Math.Pow(paths[i - 1].Points[^1].X - paths[i].Points[0].X, 2) +
                Math.Pow(paths[i - 1].Points[^1].Y - paths[i].Points[0].Y, 2)));

    private sealed class EmptyProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle = 3 * Math.PI / 4, bool useBigIntegers = true)
            => Array.Empty<MedialSegment>();
    }
}
