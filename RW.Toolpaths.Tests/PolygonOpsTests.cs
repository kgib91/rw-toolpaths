using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

/// <summary>
/// Locks in the Clipper2 conventions the milling strategies depend on: uniform scaling,
/// round joins, RDP density reduction, deterministic ordering, and nesting classification.
/// </summary>
public class PolygonOpsTests
{
    private const double Tol = 1e-6;

    private static List<PointD> Rect(double minX, double minY, double maxX, double maxY) => new()
    {
        new PointD(minX, minY),
        new PointD(maxX, minY),
        new PointD(maxX, maxY),
        new PointD(minX, maxY),
    };

    private static List<PointD> Circle(double cx, double cy, double r, int segments = 64)
    {
        var points = new List<PointD>(segments);
        for (int i = 0; i < segments; i++)
        {
            double a = 2 * Math.PI * i / segments;
            points.Add(new PointD(cx + r * Math.Cos(a), cy + r * Math.Sin(a)));
        }
        return points;
    }

    [Fact]
    public void Offset_ShrinksSquareByExactDelta()
    {
        var square = Rect(0, 0, 100, 100);

        var result = PolygonOps.Offset(new[] { square }, -10);

        Assert.Single(result);
        var bounds = Bounds(result[0]);
        Assert.Equal(10, bounds.MinX, 2);
        Assert.Equal(10, bounds.MinY, 2);
        Assert.Equal(90, bounds.MaxX, 2);
        Assert.Equal(90, bounds.MaxY, 2);
    }

    [Fact]
    public void Offset_FromOriginalAtIncreasingDistance_KeepsExactRingSpacing()
    {
        // The pocket engine offsets from the source boundary rather than re-offsetting the
        // previous ring; this asserts that spacing never drifts away from the requested stepover.
        var square = Rect(0, 0, 100, 100);

        double previousInset = 0;
        for (int i = 1; i <= 8; i++)
        {
            var rings = PolygonOps.Offset(new[] { square }, -i * 5.0);
            Assert.Single(rings);

            double inset = Bounds(rings[0]).MinX;
            Assert.Equal(i * 5.0, inset, 2);
            Assert.Equal(5.0, inset - previousInset, 2);
            previousInset = inset;
        }
    }

    [Fact]
    public void Offset_CollapsedShape_ReturnsEmpty()
    {
        var square = Rect(0, 0, 10, 10);

        Assert.Empty(PolygonOps.Offset(new[] { square }, -20));
    }

    [Fact]
    public void Offset_CollapsedCircle_ReturnsEmptyInsteadOfInverting()
    {
        var circle = Circle(0, 0, 2);

        Assert.Empty(PolygonOps.Offset(new[] { circle }, -3));
    }

    [Fact]
    public void Offset_CircleLargerThanDelta_Remains()
    {
        var circle = Circle(0, 0, 4);

        var result = PolygonOps.Offset(new[] { circle }, -3);

        Assert.Single(result);
        Assert.InRange(Math.Abs(PolygonOps.SignedArea(result[0])), 2.5, 3.5);
    }

    [Fact]
    public void Offset_CollapsedNeighborDoesNotRebuildValidContour()
    {
        var valid = Circle(0, 0, 4, segments: 512);
        var collapsed = Circle(20, 0, 2, segments: 512);

        var standalone = PolygonOps.Offset(
            new[] { valid }, -3, simplifyTolerance: 0);
        var mixed = PolygonOps.Offset(
            new[] { valid, collapsed }, -3, simplifyTolerance: 0);

        Assert.Single(standalone);
        Assert.Single(mixed);
        Assert.Equal(
            Math.Abs(PolygonOps.SignedArea(standalone[0])),
            Math.Abs(PolygonOps.SignedArea(mixed[0])),
            4);
    }

    [Fact]
    public void Offset_RoundJoin_ProducesRoundedCorner()
    {
        var square = Rect(0, 0, 100, 100);

        // Growing outward with round joins puts an arc at each corner, so the extreme
        // corner vertex must sit inside the square corner a miter join would have produced.
        var result = PolygonOps.Offset(new[] { square }, 10, simplifyTolerance: 0);

        Assert.Single(result);
        foreach (var p in result[0])
        {
            double dx = Math.Max(0, Math.Max(-p.x, p.x - 100));
            double dy = Math.Max(0, Math.Max(-p.y, p.y - 100));
            Assert.True(Math.Sqrt(dx * dx + dy * dy) <= 10 + 0.5,
                $"vertex ({p.x:F3},{p.y:F3}) exceeds the round-join envelope");
        }
    }

    [Fact]
    public void Offset_NegativeCoordinates_AreNotBiased()
    {
        // A (long)(v + 0.5) cast truncates toward zero and shifts negative coordinates
        // by a whole quantum; Math.Round does not.
        var square = Rect(-100, -100, -20, -20);

        var result = PolygonOps.Offset(new[] { square }, -5);

        Assert.Single(result);
        var bounds = Bounds(result[0]);
        Assert.Equal(-95, bounds.MinX, 2);
        Assert.Equal(-25, bounds.MaxX, 2);
    }

    [Fact]
    public void OffsetBoundary_WithHole_ProducesOuterAndInnerRing()
    {
        var outer = Rect(0, 0, 100, 100);
        var hole = Rect(40, 40, 60, 60);

        var result = PolygonOps.OffsetBoundary(outer, new[] { hole }, -5);

        Assert.Equal(2, result.Count);
        // Inward offset shrinks the outer wall and grows the hole.
        var outerRing = result[0];
        var innerRing = result[1];
        Assert.Equal(5, Bounds(outerRing).MinX, 2);
        Assert.Equal(35, Bounds(innerRing).MinX, 2);
    }

    [Fact]
    public void OffsetBoundary_IgnoresIncomingWindingOrder()
    {
        var outer = Rect(0, 0, 100, 100);
        var reversed = new List<PointD>(outer);
        reversed.Reverse();

        var fromCcw = PolygonOps.OffsetBoundary(outer, null, -10);
        var fromCw = PolygonOps.OffsetBoundary(reversed, null, -10);

        Assert.Single(fromCcw);
        Assert.Single(fromCw);
        Assert.Equal(Bounds(fromCcw[0]).MinX, Bounds(fromCw[0]).MinX, 6);
        Assert.Equal(Bounds(fromCcw[0]).MaxX, Bounds(fromCw[0]).MaxX, 6);
    }

    [Fact]
    public void Rdp_CollapsesDenseArcWithoutLosingShape()
    {
        var circle = Circle(0, 0, 50, 2048);

        var simplified = PolygonSimplify.Rdp(circle, 0.25);

        Assert.True(simplified.Count < circle.Count / 4,
            $"expected substantial reduction, got {simplified.Count} of {circle.Count}");
        foreach (var p in simplified)
            Assert.Equal(50, Math.Sqrt(p.x * p.x + p.y * p.y), 3);
    }

    [Fact]
    public void Rdp_PreservesCorners()
    {
        var square = new List<PointD>();
        // Densely sample each edge; RDP must still keep the four corners.
        for (int edge = 0; edge < 4; edge++)
        {
            var (sx, sy) = edge switch
            {
                0 => (0.0, 0.0),
                1 => (100.0, 0.0),
                2 => (100.0, 100.0),
                _ => (0.0, 100.0),
            };
            var (ex, ey) = edge switch
            {
                0 => (100.0, 0.0),
                1 => (100.0, 100.0),
                2 => (0.0, 100.0),
                _ => (0.0, 0.0),
            };
            for (int i = 0; i < 25; i++)
            {
                double t = i / 25.0;
                square.Add(new PointD(sx + (ex - sx) * t, sy + (ey - sy) * t));
            }
        }

        var simplified = PolygonSimplify.Rdp(square, 0.25);

        foreach (var corner in new[] { (0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0) })
        {
            Assert.Contains(simplified, p =>
                Math.Abs(p.x - corner.Item1) < Tol && Math.Abs(p.y - corner.Item2) < Tol);
        }
    }

    [Fact]
    public void RdpAll_LeavesSmallPolygonsUntouched()
    {
        var triangle = new List<PointD> { new(0, 0), new(10, 0), new(5, 10) };

        var result = PolygonSimplify.RdpAll(new[] { triangle }, 5.0);

        Assert.Single(result);
        Assert.Equal(3, result[0].Count);
    }

    [Fact]
    public void Difference_RemovesClipRegion()
    {
        var outer = Rect(0, 0, 100, 100);
        var clip = Rect(0, 0, 100, 50);

        var result = PolygonOps.Difference(new[] { outer }, new[] { clip });

        Assert.Single(result);
        var bounds = Bounds(result[0]);
        Assert.Equal(50, bounds.MinY, 3);
        Assert.Equal(100, bounds.MaxY, 3);
    }

    [Fact]
    public void DifferenceToComponents_TreatsIslandInsideHoleAsSeparateComponent()
    {
        var outer = Rect(0, 0, 100, 100);

        // Clip is an annulus: a CCW ring with a CW void punched through it. Subtracting it
        // leaves a frame plus a solid island stranded inside the frame's hole.
        var clipRing = Rect(20, 20, 80, 80);
        var clipVoid = Rect(40, 40, 60, 60);
        clipVoid.Reverse();

        var components = PolygonOps.DifferenceToComponents(
            new[] { outer },
            new[] { clipRing, clipVoid });

        Assert.Equal(2, components.Count);

        var frame = components.Single(c => Math.Abs(PolygonOps.SignedArea(c.Outer)) > 3000);
        Assert.Single(frame.Holes);

        var island = components.Single(c => Math.Abs(PolygonOps.SignedArea(c.Outer)) < 1000);
        Assert.Equal(400, Math.Abs(PolygonOps.SignedArea(island.Outer)), 1);
    }

    [Fact]
    public void Clip_KeepsConcentricRingsSeparate()
    {
        // Batching nested rings through one NonZero intersect would merge them into a single
        // filled region and destroy the ring structure the pocket engine relies on.
        var rings = new List<List<PointD>>
        {
            Rect(0, 0, 100, 100),
            Rect(10, 10, 90, 90),
            Rect(20, 20, 80, 80),
        };

        var result = PolygonOps.Clip(rings, Rect(-50, -50, 150, 150));

        Assert.Equal(3, result.Count);
    }

    [Fact]
    public void Clip_TrimsPolygonCrossingBoundary()
    {
        var polygon = Rect(0, 0, 100, 100);

        var result = PolygonOps.Clip(new[] { polygon }, Rect(50, -10, 200, 200));

        Assert.Single(result);
        Assert.Equal(50, Bounds(result[0]).MinX, 3);
    }

    [Fact]
    public void BufferCenterlines_ClosedRing_YieldsAnnulusNotDisc()
    {
        var ring = Rect(0, 0, 100, 100);

        var result = PolygonOps.BufferCenterlines(new[] { ring }, 5);

        // An annulus is an outer contour plus an inner void; a filled disc would be one contour.
        Assert.Equal(2, result.Count);
        Assert.Contains(result, p => PolygonOps.SignedArea(p) > 0);
        Assert.Contains(result, p => PolygonOps.SignedArea(p) < 0);
    }

    [Fact]
    public void Open_RejectsRegionThinnerThanProbe()
    {
        var sliver = Rect(0, 0, 100, 1);

        Assert.Empty(PolygonOps.Open(new[] { sliver }, 2.0));
        Assert.NotEmpty(PolygonOps.Open(new[] { Rect(0, 0, 100, 100) }, 2.0));
    }

    [Fact]
    public void NormalizeOrder_IsDeterministicRegardlessOfInputOrder()
    {
        var a = Rect(0, 0, 10, 10);
        var b = Rect(50, 50, 90, 90);

        var forward = PolygonOps.NormalizeOrder(new List<List<PointD>> { a, b });
        var reverse = PolygonOps.NormalizeOrder(new List<List<PointD>>
        {
            new(b),
            new(a),
        });

        Assert.Equal(forward.Count, reverse.Count);
        for (int i = 0; i < forward.Count; i++)
        {
            Assert.Equal(forward[i].Count, reverse[i].Count);
            for (int j = 0; j < forward[i].Count; j++)
            {
                Assert.Equal(forward[i][j].x, reverse[i][j].x, 9);
                Assert.Equal(forward[i][j].y, reverse[i][j].y, 9);
            }
        }
    }

    [Fact]
    public void NormalizeOrder_SortsLargestAreaFirst()
    {
        var small = Rect(0, 0, 10, 10);
        var large = Rect(0, 0, 100, 100);

        var result = PolygonOps.NormalizeOrder(new List<List<PointD>> { small, large });

        Assert.Equal(10000, Math.Abs(PolygonOps.SignedArea(result[0])), 1);
    }

    [Fact]
    public void Perimeter_IncludesClosingEdge()
    {
        Assert.Equal(400, PolygonOps.Perimeter(Rect(0, 0, 100, 100)), 6);
    }

    [Fact]
    public void BuildTree_NestsHoleUnderOuter()
    {
        var outer = Rect(0, 0, 100, 100);
        var hole = Rect(40, 40, 60, 60);

        var roots = PolygonNesting.BuildTree(new[] { hole, outer });

        Assert.Single(roots);
        Assert.Single(roots[0].Children);
        Assert.Equal(10000, Math.Abs(PolygonOps.SignedArea(roots[0].Points)), 1);
    }

    [Fact]
    public void BuildTree_NestsIslandThreeLevelsDeep()
    {
        var outer = Rect(0, 0, 100, 100);
        var hole = Rect(20, 20, 80, 80);
        var island = Rect(40, 40, 60, 60);

        var roots = PolygonNesting.BuildTree(new[] { island, outer, hole });

        Assert.Single(roots);
        Assert.Single(roots[0].Children);
        Assert.Single(roots[0].Children[0].Children);
    }

    [Fact]
    public void GroupOuterWithHoles_StartsNewGroupForNestedIsland()
    {
        var outer = Rect(0, 0, 100, 100);
        var hole = Rect(20, 20, 80, 80);
        var island = Rect(40, 40, 60, 60);

        var groups = PolygonNesting.GroupOuterWithHoles(
            PolygonNesting.BuildTree(new[] { outer, hole, island }));

        Assert.Equal(2, groups.Count);
        Assert.Equal(2, groups[0].Count); // outer + its hole
        Assert.Single(groups[1]);         // island is solid material again
    }

    private static (double MinX, double MinY, double MaxX, double MaxY) Bounds(IReadOnlyList<PointD> polygon)
    {
        double minX = double.MaxValue, minY = double.MaxValue;
        double maxX = double.MinValue, maxY = double.MinValue;
        foreach (var p in polygon)
        {
            minX = Math.Min(minX, p.x);
            minY = Math.Min(minY, p.y);
            maxX = Math.Max(maxX, p.x);
            maxY = Math.Max(maxY, p.y);
        }
        return (minX, minY, maxX, maxY);
    }
}
