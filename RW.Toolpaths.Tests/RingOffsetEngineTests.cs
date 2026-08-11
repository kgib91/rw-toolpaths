using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

/// <summary>
/// Guards the ring-ordering guarantees the legacy generator established: exact ring spacing,
/// finishing one island before moving to the next, and cutting enclosed material while it is
/// still supported.
/// </summary>
public class RingOffsetEngineTests
{
    private static List<PointD> Rect(double minX, double minY, double maxX, double maxY) => new()
    {
        new PointD(minX, minY),
        new PointD(maxX, minY),
        new PointD(maxX, maxY),
        new PointD(minX, maxY),
    };

    private static MillingRegion Region(double minX, double minY, double maxX, double maxY)
        => new(Rect(minX, minY, maxX, maxY));

    private static List<PointD> Circle(double centerX, double centerY, double radius, int segments)
        => Enumerable.Range(0, segments)
            .Select(index =>
            {
                double angle = index * Math.PI * 2 / segments;
                return new PointD(
                    centerX + Math.Cos(angle) * radius,
                    centerY + Math.Sin(angle) * radius);
            })
            .ToList();

    private static RingOffsetOptions Options(double firstOffset, double stepOver, int ringLimit = 10_000)
        => new(firstOffset, stepOver) { RingLimit = ringLimit };

    // --- Ring construction ----------------------------------------------------

    [Fact]
    public void BuildRings_SpacingMatchesStepOverExactly()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 100, 100) },
            Options(firstOffset: 3, stepOver: 6));

        Assert.True(rings.Count >= 5);
        for (int i = 0; i < rings.Count; i++)
        {
            Assert.Equal(i, rings[i].Level);
            double inset = rings[i].Points.Min(p => p.x);
            Assert.Equal(3 + i * 6.0, inset, 2);
        }
    }

    [Fact]
    public void BuildRings_StopsWhenRegionIsConsumed()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 20, 20) },
            Options(firstOffset: 5, stepOver: 5));

        // A 20mm square with a 5mm first offset leaves a 10mm core; only one ring fits.
        Assert.Single(rings);
    }

    [Fact]
    public void BuildRings_DenseCircleStopsBeforeCollapsedOffsetInverts()
    {
        const double sourceRadius = 5.5;
        const double toolRadius = 3.175;
        var rings = RingOffsetEngine.BuildRings(
            new[] { new MillingRegion(Circle(0, 0, sourceRadius, 256)) },
            Options(firstOffset: toolRadius, stepOver: toolRadius, ringLimit: 32));

        var ring = Assert.Single(rings);
        Assert.Equal(0, ring.Level);
        Assert.All(ring.Points, point =>
        {
            double centerlineRadius = Math.Sqrt(point.x * point.x + point.y * point.y);
            double edgeClearance = sourceRadius - centerlineRadius;
            Assert.True(
                edgeClearance >= toolRadius - 0.01,
                $"centerline clearance {edgeClearance:F6} is smaller than tool radius {toolRadius:F6}");
        });
    }

    [Fact]
    public void BuildRings_RespectsRingLimit()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 500, 500) },
            Options(firstOffset: 3, stepOver: 6, ringLimit: 4));

        Assert.Equal(4, rings.Count);
    }

    [Fact]
    public void BuildRings_ZeroStepOver_ProducesSingleRing()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 100, 100) },
            Options(firstOffset: 3, stepOver: 0));

        Assert.Single(rings);
    }

    [Fact]
    public void BuildRings_ToolTooLarge_ProducesNothing()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 10, 10) },
            Options(firstOffset: 20, stepOver: 5));

        Assert.Empty(rings);
    }

    [Fact]
    public void BuildRings_NormalizesWindingToCounterClockwise()
    {
        var clockwise = Rect(0, 0, 100, 100);
        clockwise.Reverse();

        var rings = RingOffsetEngine.BuildRings(
            new[] { new MillingRegion(clockwise) },
            Options(firstOffset: 5, stepOver: 10));

        Assert.NotEmpty(rings);
        foreach (var ring in rings)
            Assert.True(PolygonOps.SignedArea(ring.Points) > 0);
    }

    [Fact]
    public void BuildRings_RegionWithHole_ProducesOuterAndInnerRingsPerLevel()
    {
        var region = new MillingRegion(Rect(0, 0, 100, 100), new[] { Rect(40, 40, 60, 60) });

        var rings = RingOffsetEngine.BuildRings(new[] { region }, Options(firstOffset: 3, stepOver: 6));

        // Level 0 must contain both the shrunken outer wall and the grown hole wall.
        Assert.Equal(2, rings.Count(r => r.Level == 0));
        Assert.Single(rings.Where(ring => ring.Level == 0 && !ring.IsHole));
        Assert.Single(rings.Where(ring => ring.Level == 0 && ring.IsHole));
        Assert.All(rings.Where(ring => ring.Level == 0),
            ring => Assert.True(PolygonOps.SignedArea(ring.Points) > 0));
    }

    [Fact]
    public void BuildRings_ManyHolesDoNotStopConcentricOffsetsAtLevelZero()
    {
        var holes = new List<IReadOnlyList<PointD>>();
        for (int row = 0; row < 5; row++)
        {
            for (int column = 0; column < 5; column++)
            {
                double minX = 20 + column * 32;
                double minY = 20 + row * 32;
                holes.Add(Rect(minX, minY, minX + 10, minY + 10));
            }
        }

        var rings = RingOffsetEngine.BuildRings(
            new[] { new MillingRegion(Rect(0, 0, 200, 200), holes) },
            Options(firstOffset: 2, stepOver: 2, ringLimit: 3));

        Assert.Equal(new[] { 0, 1, 2 }, rings
            .Select(ring => ring.Level)
            .Distinct()
            .Order()
            .ToArray());
        Assert.All(Enumerable.Range(0, 3), level =>
        {
            Assert.Single(rings.Where(ring => ring.Level == level && !ring.IsHole));
            Assert.Equal(25, rings.Count(ring => ring.Level == level && ring.IsHole));
        });
    }

    [Fact]
    public void BuildRings_ContinuesAfterNeighbouringHolesMerge()
    {
        var holes = new IReadOnlyList<PointD>[]
        {
            Rect(20, 20, 50, 60),
            Rect(52, 20, 82, 60),
            Rect(88, 20, 108, 40),
        };

        var rings = RingOffsetEngine.BuildRings(
            new[] { new MillingRegion(Rect(0, 0, 140, 80), holes) },
            Options(firstOffset: 0.5, stepOver: 1, ringLimit: 6));

        Assert.Equal(Enumerable.Range(0, 6), rings
            .Select(ring => ring.Level)
            .Distinct()
            .Order());
        Assert.Contains(
            Enumerable.Range(1, 5),
            level => rings.Count(ring => ring.Level == level)
                < rings.Count(ring => ring.Level == 0));
        Assert.All(
            Enumerable.Range(0, 6),
            level => Assert.Contains(
                rings,
                ring => ring.Level == level && !ring.IsHole));
    }

    // --- Nesting --------------------------------------------------------------

    [Fact]
    public void BuildNesting_ParentsEachRingToTheLevelOutside()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 100, 100) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);

        Assert.Equal(-1, rings[0].Parent);
        for (int i = 1; i < rings.Count; i++)
        {
            Assert.True(rings[i].Parent >= 0);
            Assert.Equal(rings[i].Level - 1, rings[rings[i].Parent].Level);
        }
    }

    [Fact]
    public void BuildNesting_KeepsSeparateRegionsIndependent()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 60, 60), Region(200, 200, 260, 260) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);

        foreach (var ring in rings)
        {
            if (ring.Parent >= 0)
                Assert.Equal(ring.RegionIndex, rings[ring.Parent].RegionIndex);
        }
    }

    // --- Traversal ------------------------------------------------------------

    [Fact]
    public void BuildTraversal_CompletesEachIslandBeforeStartingTheNext()
    {
        // The behaviour ToolpathGeneratorTopologyTests locked in: no hopping between pockets.
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 60, 60), Region(200, 0, 260, 60), Region(400, 0, 460, 60) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);

        var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut: false);

        Assert.Equal(3, traversal.Islands.Count);

        var seen = new HashSet<int>();
        int current = -1;
        foreach (int index in traversal.Order)
        {
            int region = rings[index].RegionIndex;
            if (region != current)
            {
                Assert.True(seen.Add(region), $"returned to region {region} after leaving it");
                current = region;
            }
        }
    }

    [Fact]
    public void BuildTraversal_OutsideIn_CutsParentBeforeChildren()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 100, 100) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);

        var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut: false);

        var position = Position(traversal.Order);
        foreach (var ring in rings)
        {
            if (ring.Parent >= 0)
                Assert.True(position[ring.Parent] < position[rings.IndexOf(ring)]);
        }
    }

    [Fact]
    public void BuildTraversal_InsideOut_CutsChildrenBeforeParent()
    {
        // Enclosed material must be removed while the surrounding ring still supports it.
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 100, 100) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);

        var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut: true);

        var position = Position(traversal.Order);
        foreach (var ring in rings)
        {
            if (ring.Parent >= 0)
                Assert.True(position[ring.Parent] > position[rings.IndexOf(ring)]);
        }
    }

    [Fact]
    public void BuildTraversal_VisitsEveryRingExactlyOnce()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 80, 80), Region(150, 0, 230, 80) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);

        foreach (bool insideOut in new[] { false, true })
        {
            var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut);
            Assert.Equal(rings.Count, traversal.Order.Count);
            Assert.Equal(rings.Count, traversal.Order.Distinct().Count());
            Assert.Equal(rings.Count, traversal.Islands.Sum(i => i.Count));
        }
    }

    [Fact]
    public void BuildTraversal_OutsideIn_StartsWithLargestIsland()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 40, 40), Region(200, 0, 320, 120) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);

        var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut: false);

        Assert.Equal(1, rings[traversal.Order[0]].RegionIndex);
    }

    // --- Chaining -------------------------------------------------------------

    [Fact]
    public void BuildChains_ChainsAnUnbranchedRingRunIntoOneSpiral()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 100, 100) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);
        var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut: false);

        var chains = RingOffsetEngine.BuildChains(rings, traversal.Order);

        Assert.Single(chains);
        Assert.Equal(rings.Count, chains[0].RingIndices.Count);
    }

    [Fact]
    public void BuildChains_BreaksBetweenIslands()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 60, 60), Region(200, 0, 260, 60) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);
        var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut: false);

        var chains = RingOffsetEngine.BuildChains(rings, traversal.Order);

        Assert.Equal(2, chains.Count);
        foreach (var chain in chains)
            Assert.Single(chain.RingIndices.Select(i => rings[i].RegionIndex).Distinct());
    }

    [Fact]
    public void BuildChains_CoversEveryRingOnce()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 100, 100), Region(300, 0, 360, 60) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);
        var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut: false);

        var chains = RingOffsetEngine.BuildChains(rings, traversal.Order);

        var covered = chains.SelectMany(c => c.RingIndices).ToList();
        Assert.Equal(rings.Count, covered.Count);
        Assert.Equal(rings.Count, covered.Distinct().Count());
    }

    [Fact]
    public void MaterializeChain_KeepsRingTransitionsShort()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 100, 100) },
            Options(firstOffset: 3, stepOver: 6));
        RingOffsetEngine.BuildNesting(rings);
        var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut: false);
        var chains = RingOffsetEngine.BuildChains(rings, traversal.Order);

        var points = RingOffsetEngine.MaterializeChain(rings, chains[0], MillingDirection.Climb);

        // Rebasing must keep every step within a ring's own edge length plus the stepover;
        // without it the link between rings would be a chord across the whole pocket.
        double longest = 0;
        for (int i = 1; i < points.Count; i++)
        {
            double dx = points[i].x - points[i - 1].x;
            double dy = points[i].y - points[i - 1].y;
            longest = Math.Max(longest, Math.Sqrt(dx * dx + dy * dy));
        }

        Assert.True(longest < 100, $"longest move {longest:F2}mm suggests rebasing did not apply");
    }

    [Fact]
    public void MaterializeChain_ClimbAndConventionalRunOppositeDirections()
    {
        var rings = RingOffsetEngine.BuildRings(
            new[] { Region(0, 0, 100, 100) },
            Options(firstOffset: 5, stepOver: 40));
        RingOffsetEngine.BuildNesting(rings);
        var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut: false);
        var chains = RingOffsetEngine.BuildChains(rings, traversal.Order);

        var climb = RingOffsetEngine.MaterializeChain(rings, chains[0], MillingDirection.Climb);
        var conventional = RingOffsetEngine.MaterializeChain(rings, chains[0], MillingDirection.Conventional);

        Assert.True(PolygonOps.SignedArea(climb) > 0);
        Assert.True(PolygonOps.SignedArea(conventional) < 0);
    }

    [Fact]
    public void Orient_DefaultLeavesWindingUntouched()
    {
        var clockwise = Rect(0, 0, 10, 10);
        clockwise.Reverse();

        var result = RingOffsetEngine.Orient(clockwise, MillingDirection.Default);

        Assert.True(PolygonOps.SignedArea(result) < 0);
    }

    private static Dictionary<int, int> Position(IReadOnlyList<int> order)
    {
        var position = new Dictionary<int, int>(order.Count);
        for (int i = 0; i < order.Count; i++)
            position[order[i]] = i;
        return position;
    }
}
