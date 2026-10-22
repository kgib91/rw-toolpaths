using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class PocketContinuationRouterTests
{
    private readonly ITestOutputHelper _output;
    public PocketContinuationRouterTests(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PublicPocketKeepsTheRingAndStripContinuousWithOneEntryAtEachDepth(bool insideOut)
    {
        var depth = new DepthSchedule(2, 1);
        var plan = PocketToolpaths.Generate(new[] { new MillingRegion(Rectangle(0, 0, 16, 60)) },
            ToolGeometry.Flat(3), depth, new PocketOptions
            {
                StepOver = 1, InsideOut = insideOut, SpiralChaining = true,
                Ramp = RampSettings.FromRatio(4), DetectCurves = false, CalculateSweptArea = false,
                Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = 0 },
            });
        _output.WriteLine($"insideOut={insideOut}, rings={plan.Diagnostics.RingCount}, strips={plan.Diagnostics.StripCount}, paths={plan.Toolpaths.Count}.");
        Assert.Equal(1, plan.Diagnostics.RingCount);
        Assert.Equal(1, plan.Diagnostics.StripCount);

        for (int pass = 0; pass < depth.PassCount; pass++)
        {
            TaggedToolpath[] layer = plan.Toolpaths.Where(path => path.DepthPassIndex == pass).ToArray();
            // The terminal trace now belongs to its parent offset chain rather
            // than arriving as a second cleanup visit.
            Assert.Single(layer);
            for (int index = 1; index < layer.Length; index++)
                Assert.Equal(layer[index - 1].Points[^1], layer[index].Points[0]);
            var descendingRamps = layer.SelectMany(path => path.Spans
                .Where(span => span.Kind == ToolpathSpanKind.Ramp
                    && path.Points[span.StartIndex].Z > path.Points[span.EndIndex].Z + 1e-9)).ToArray();
            // The entry may cross a source corner and therefore contain several
            // consecutive ramp spans. It must remain one uninterrupted descent.
            Assert.NotEmpty(descendingRamps);
            for (int index = 1; index < descendingRamps.Length; index++)
                Assert.Equal(descendingRamps[index - 1].EndIndex, descendingRamps[index].StartIndex);
            Assert.Equal(depth.PassTopZ(pass), layer[0].Points[descendingRamps[0].StartIndex].Z, 9);
            Assert.Equal(depth.PassBottomZ(pass), layer[0].Points[descendingRamps[^1].EndIndex].Z, 9);
            var links = layer.SelectMany(path => path.Spans
                .Where(span => span.Kind == ToolpathSpanKind.Link)
                .Select(span => (Path: path, Span: span))).ToArray();
            Assert.NotEmpty(links);
            Assert.Contains(links, link => Math.Abs(link.Path.Points[link.Span.StartIndex].Z - depth.PassBottomZ(pass)) < 1e-9);
            foreach (var (path, span) in links)
            {
                double linkZ = path.Points[span.StartIndex].Z;
                Assert.True(Math.Abs(linkZ - depth.PassBottomZ(pass)) < 1e-9
                    || pass > 0 && Math.Abs(linkZ - depth.PassTopZ(pass)) < 1e-9,
                    $"Link occurs at an unproved floor: insideOut={insideOut}, pass={pass}, Z={linkZ:R}.");
                foreach (Point3D point in path.Points.Skip(span.StartIndex).Take(span.PointCount))
                {
                    Assert.Equal(linkZ, point.Z, 9);
                    Assert.InRange(point.X, 3 - .0012, 13 + .0012);
                    Assert.InRange(point.Y, 3 - .0012, 57 + .0012);
                }
            }
            var floorMoves = layer.SelectMany(path => OpenEdges(path.Points))
                .Where(edge => Math.Abs(edge.A.Z - depth.PassBottomZ(pass)) < 1e-9
                    && Math.Abs(edge.B.Z - depth.PassBottomZ(pass)) < 1e-9).ToArray();
            for (double y = 8; y <= 52; y += .5)
            for (double x = 6.5; x <= 9.5; x += .5)
            {
                var witness = new PointD(x, y);
                double distance = Math.Sqrt(floorMoves.Min(edge => PointSegmentDistanceSquared(witness, edge.A, edge.B)));
                Assert.True(distance <= 3.002,
                    $"Continuation lost middle-strip coverage: insideOut={insideOut}, pass={pass}, witness=({x:R},{y:R}), distance={distance:R}.");
            }
        }
        foreach (var visit in plan.Toolpaths.GroupBy(path => path.VisitIdentity))
            Assert.Single(visit.Select(path => (path.RegionIndex, path.DepthPassIndex)).Distinct());
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ANewSourceRegionKeepsItsEntryEvenWhenTheOperationWideTravelAreaOverlaps(bool insideOut)
    {
        // The legal centre regions overlap, so a two-dimensional operation-wide
        // clearance policy could approve a bridge. Source-region ownership still
        // requires the new region's own depth entry and separate logical visit.
        var plan = PocketToolpaths.Generate(new[]
        {
            new MillingRegion(Rectangle(0, 0, 16, 60)),
            new MillingRegion(Rectangle(8, 0, 24, 60)),
        }, ToolGeometry.Flat(3), new DepthSchedule(1, 1), new PocketOptions
        {
            StepOver = 1, InsideOut = insideOut, SpiralChaining = true,
            Ramp = RampSettings.FromRatio(4), DetectCurves = false, CalculateSweptArea = false,
            Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = 0 },
        });

        Assert.Equal(2, plan.Toolpaths.Select(path => path.RegionIndex).Distinct().Count());
        int checkedBoundaries = 0;
        for (int index = 1; index < plan.Toolpaths.Count; index++)
        {
            TaggedToolpath previous = plan.Toolpaths[index - 1], current = plan.Toolpaths[index];
            if (previous.RegionIndex == current.RegionIndex) continue;
            checkedBoundaries++;
            Assert.Equal(0, current.Points[0].Z, 9);
            Assert.Contains(current.Spans, span => span.Kind == ToolpathSpanKind.Ramp
                && current.Points[span.StartIndex].Z > current.Points[span.EndIndex].Z + 1e-9);
        }
        Assert.True(checkedBoundaries > 0);
        foreach (var visit in plan.Toolpaths.GroupBy(path => path.VisitIdentity))
            Assert.Single(visit.Select(path => (path.RegionIndex, path.DepthPassIndex)).Distinct());
    }

    [Fact]
    public void AnEarlyRoomCannotCrossTheVirginStemToAnotherRoomAtFloor()
    {
        var plan = StemmedRooms(insideOut: true);
        var farRoom = Assert.Single(plan.Toolpaths.Where(path =>
            path.Points.Min(point => point.X) > 80));
        Assert.Equal(0, farRoom.Points[0].Z, 9);
        Assert.Contains(farRoom.Spans, span => span.Kind == ToolpathSpanKind.Ramp
            && farRoom.Points[span.StartIndex].Z > farRoom.Points[span.EndIndex].Z);
        Assert.Equal(2, plan.Diagnostics.LinearRampEntries + plan.Diagnostics.HelicalEntries + plan.Diagnostics.PlungeEntries);
        AssertNoPrematureStemShortcut(plan);
    }

    [Fact]
    public void TheSameLongStemTransferStaysAtFloorAfterTheStemHasActuallyBeenCut()
    {
        var plan = StemmedRooms(insideOut: false);
        Assert.Equal(1, plan.Diagnostics.LinearRampEntries + plan.Diagnostics.HelicalEntries + plan.Diagnostics.PlungeEntries);
        Assert.Equal(0, plan.Diagnostics.LinkLifts);
        Assert.Contains(plan.Toolpaths.SelectMany(path => path.Spans
            .Where(span => span.Kind == ToolpathSpanKind.Link)
            .Select(span => (Path: path, Span: span))), link =>
            Math.Sqrt(Math.Pow(link.Path.Points[link.Span.EndIndex].X - link.Path.Points[link.Span.StartIndex].X, 2)
                + Math.Pow(link.Path.Points[link.Span.EndIndex].Y - link.Path.Points[link.Span.StartIndex].Y, 2)) > 70);
        AssertNoPrematureStemShortcut(plan);
    }

    [Theory]
    [InlineData(false, 1)]
    [InlineData(true, 4)]
    public void ProtectedAnnularFrontsKeepOnlyTheirPhysicallyRequiredEntriesAtEveryDepth(bool insideOut, int entriesPerDepth)
    {
        var hole = Rectangle(30, 30, 70, 70);
        var outline = Rectangle(0, 0, 100, 100);
        var depth = new DepthSchedule(6, 2);
        var plan = PocketToolpaths.Generate(new[] { new MillingRegion(outline, new[] { hole }) },
            ToolGeometry.Flat(3), depth, new PocketOptions
            {
                InsideOut = insideOut, Ramp = RampSettings.FromRatio(2), DetectCurves = false,
                CalculateSweptArea = false,
            });
        // The four inset15 corner fronts are disconnected before the parent
        // contour is cut. Inside-out visits each of them first and needs four
        // entries; outside-in has already cut the connecting parent front.
        Assert.Equal(entriesPerDepth * depth.PassCount,
            plan.Diagnostics.LinearRampEntries + plan.Diagnostics.HelicalEntries + plan.Diagnostics.PlungeEntries);
        if (!insideOut)
        {
            Assert.Equal(0, plan.Diagnostics.LinkLifts);
            for (int index = 1; index < plan.Toolpaths.Count; index++)
                Assert.Equal(plan.Toolpaths[index - 1].Points[^1], plan.Toolpaths[index].Points[0]);
        }
        foreach (int pass in Enumerable.Range(0, depth.PassCount))
        {
            var paths = plan.Toolpaths.Where(path => path.DepthPassIndex == pass).ToArray();
            Assert.NotEmpty(paths);
            int descents = paths.Sum(path => path.Spans.Count(span => span.Kind == ToolpathSpanKind.Ramp
                && path.Points[span.StartIndex].Z > path.Points[span.EndIndex].Z));
            Assert.Equal(entriesPerDepth, descents);
        }
        var protectedSource = new List<IReadOnlyList<PointD>> { outline, hole.AsEnumerable().Reverse().ToList() };
        var indexOfWalls = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(protectedSource));
        foreach (var path in plan.Toolpaths)
        foreach (var (a, b) in OpenEdges(path.Points))
        {
            var from = new Point64((long)Math.Round(a.X * PathUtils.Scale), (long)Math.Round(a.Y * PathUtils.Scale));
            var to = new Point64((long)Math.Round(b.X * PathUtils.Scale), (long)Math.Round(b.Y * PathUtils.Scale));
            Assert.Null(indexOfWalls.ClearanceViolation(from, to, 3 * PathUtils.Scale - 4));
        }
    }

    [Fact]
    public void ARebasedClosedCutReturnsToItsOldSeamByTheShorterNativeWalk()
    {
        var oldSeam = new Point3D(10, 1, -1);
        var native = new List<Point3D>
        {
            oldSeam, new(10, 0, -1), new(20, 0, -1), new(20, 10, -1), new(10, 10, -1), oldSeam,
        };
        var from = new Point3D(0, 0, -1);
        var paths = new[]
        {
            new TaggedToolpath(new List<Point3D> { native[1], from },
                new List<ToolpathSpan> { new(0, 1, ToolpathSpanKind.Cut) }, 0, PocketToolpaths.Category, 0, 0),
            new TaggedToolpath(new[] { from }.Concat(native).ToList(),
                new List<ToolpathSpan> { new(0, 1, ToolpathSpanKind.Link), new(1, native.Count, ToolpathSpanKind.Cut) },
                0, PocketToolpaths.Category, 0, 1),
            new TaggedToolpath(new List<Point3D> { oldSeam, native[^2] },
                new List<ToolpathSpan> { new(0, 1, ToolpathSpanKind.Cut) }, 0, PocketToolpaths.Category, 0, 2),
        };
        var routed = PocketContinuationRouter.Route(paths, new DepthSchedule(1, 1),
            new PocketOptions { LinkClearance = new HorizontalDeparture() }, 3, CancellationToken.None);
        Assert.Equal(0, routed.Lifts);
        var repaired = routed.Paths.Single(path => path.VisitIdentity == 1);
        var cut = repaired.Spans.Single(span => span.Kind == ToolpathSpanKind.Cut);
        var returns = repaired.Spans.Where(span => span.Kind == ToolpathSpanKind.Link && span.StartIndex >= cut.EndIndex).ToArray();
        double returnLength = returns.Sum(span => OpenEdges(repaired.Points.Skip(span.StartIndex).Take(span.PointCount).ToArray())
            .Sum(edge => Math.Sqrt(Math.Pow(edge.B.X - edge.A.X, 2) + Math.Pow(edge.B.Y - edge.A.Y, 2))));
        Assert.Equal(1, returnLength, 9);
        Assert.Equal(oldSeam, repaired.Points[^1]);
        Assert.Equal(repaired.Points[^1], routed.Paths[^1].Points[0]);
        var actual = OpenEdges(repaired.Points.Skip(cut.StartIndex).Take(cut.PointCount).ToArray()).ToArray();
        foreach (var edge in OpenEdges(native)) Assert.Contains(edge, actual);
    }

    private sealed class HorizontalDeparture : ILinkClearance
    {
        public bool IsTravelSafe(Point3D from, Point3D to, double radius)
            => from.X != 0 || to.Y == 0;
    }

    private ToolpathPlan StemmedRooms(bool insideOut)
    {
        var outline = new List<PointD>
        {
            new(0, 0), new(18, 0), new(18, 3), new(82, 3), new(82, 0), new(100, 0),
            new(100, 18), new(82, 18), new(82, 15), new(18, 15), new(18, 18), new(0, 18),
        };
        var plan = PocketToolpaths.Generate(new[] { new MillingRegion(outline) }, ToolGeometry.Flat(3),
            new DepthSchedule(1, 1), new PocketOptions
            {
                StepOver = 1, InsideOut = insideOut, SpiralChaining = true,
                Ramp = RampSettings.FromRatio(4), DetectCurves = false, CalculateSweptArea = false,
                Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = 0 },
            });
        _output.WriteLine($"stemmed rooms insideOut={insideOut}: paths={plan.Toolpaths.Count}, "
            + $"entries={plan.Diagnostics.LinearRampEntries + plan.Diagnostics.HelicalEntries + plan.Diagnostics.PlungeEntries}.");
        foreach (var path in plan.Toolpaths)
            _output.WriteLine($"  from={path.Points[0]} to={path.Points[^1]} spans="
                + string.Join(',', path.Spans.Select(span => $"{span.Kind}:{span.StartIndex}..{span.EndIndex}")));
        return plan;
    }

    private static void AssertNoPrematureStemShortcut(ToolpathPlan plan)
    {
        var witness = new PointD(50, 9);
        var previous = new List<(Point3D A, Point3D B)>();
        foreach (var path in plan.Toolpaths)
        for (int spanIndex = 0; spanIndex < path.Spans.Count; spanIndex++)
        {
            var span = path.Spans[spanIndex];
            var floorEdges = OpenEdges(path.Points.Skip(span.StartIndex).Take(span.PointCount).ToArray())
                .Where(edge => edge.A.Z <= -1 + 1e-9 && edge.B.Z <= -1 + 1e-9).ToArray();
            if (span.Kind == ToolpathSpanKind.Link && floorEdges.Any(edge => PointSegmentDistanceSquared(witness, edge.A, edge.B) <= 9))
            {
                var next = path.Spans.Skip(spanIndex + 1).FirstOrDefault(part => part.Kind == ToolpathSpanKind.Cut);
                var nextEdges = next.MoveCount > 0
                    ? OpenEdges(path.Points.Skip(next.StartIndex).Take(next.PointCount).ToArray()).ToArray()
                    : Array.Empty<(Point3D A, Point3D B)>();
                Assert.True(previous.Concat(nextEdges).Any(edge => PointSegmentDistanceSquared(witness, edge.A, edge.B) <= 9),
                    "A floor transfer crosses the untouched stem using a later room or parent cut as premature stock clearance.");
            }
            previous.AddRange(floorEdges);
            // A descending entry only clears a floor disc at its landing; its
            // above-floor XY motion cannot authorize a floor shortcut.
            foreach (var point in path.Points.Skip(span.StartIndex).Take(span.PointCount).Where(point => point.Z <= -1 + 1e-9))
                previous.Add((point, point));
        }
    }

    private static List<PointD> Rectangle(double minX, double minY, double maxX, double maxY)
        => new() { new(minX, minY), new(maxX, minY), new(maxX, maxY), new(minX, maxY) };

    private static IEnumerable<(Point3D A, Point3D B)> OpenEdges(IReadOnlyList<Point3D> points)
    {
        for (int index = 1; index < points.Count; index++) yield return (points[index - 1], points[index]);
    }

    private static double PointSegmentDistanceSquared(PointD point, Point3D a, Point3D b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y;
        double lengthSquared = dx * dx + dy * dy;
        double t = lengthSquared == 0 ? 0 : Math.Clamp(((point.x - a.X) * dx + (point.y - a.Y) * dy) / lengthSquared, 0, 1);
        return Math.Pow(point.x - a.X - t * dx, 2) + Math.Pow(point.y - a.Y - t * dy, 2);
    }
}
