using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

/// <summary>
/// End-to-end checks on the two milling strategies: the cutter must stay out of the walls,
/// step down in order, finish one island before starting the next, and retain the part when
/// holding tabs are configured.
/// </summary>
public class MillingStrategyTests
{
    private static List<PointD> Rect(double minX, double minY, double maxX, double maxY) => new()
    {
        new PointD(minX, minY),
        new PointD(maxX, minY),
        new PointD(maxX, maxY),
        new PointD(minX, maxY),
    };

    private static DepthSchedule Depth(double depth, double perPass) => new(depth, perPass);

    private static IEnumerable<Point3D> AllPoints(ToolpathPlan plan)
        => plan.Toolpaths.SelectMany(t => t.Points);

    // --- Pocketing ------------------------------------------------------------

    [Fact]
    public void Pocket_ProducesPathsCoveringEveryDepthPass()
    {
        var plan = PocketToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 100)) },
            ToolGeometry.Flat(3),
            Depth(9, 3),
            new PocketOptions());

        Assert.NotEmpty(plan.Toolpaths);
        Assert.Equal(3, plan.Diagnostics.DepthPassCount);

        var passes = plan.Toolpaths.Select(t => t.DepthPassIndex!.Value).Distinct().OrderBy(v => v).ToList();
        Assert.Equal(new List<int> { 0, 1, 2 }, passes);
    }

    [Fact]
    public void Pocket_AnnularRegionStaysContinuousAcrossRingsAndDepths()
    {
        var plan = PocketToolpaths.Generate(
            new[]
            {
                new MillingRegion(
                    Rect(0, 0, 100, 100),
                    new[] { Rect(30, 30, 70, 70) }),
            },
            ToolGeometry.Flat(3),
            Depth(6, 2),
            new PocketOptions { Ramp = RampSettings.FromRatio(2) });

        Assert.True(plan.Diagnostics.RingCount >= 2);
        Assert.Equal(0, plan.Diagnostics.LinkLifts);
        Assert.Equal(new[] { 0, 1, 2 }, plan.Toolpaths
            .Select(path => path.DepthPassIndex!.Value)
            .Distinct()
            .ToArray());

        for (int pathIndex = 1; pathIndex < plan.Toolpaths.Count; pathIndex++)
        {
            Point3D previous = plan.Toolpaths[pathIndex - 1].Points[^1];
            Point3D current = plan.Toolpaths[pathIndex].Points[0];
            Assert.Equal(previous.X, current.X, 6);
            Assert.Equal(previous.Y, current.Y, 6);
            Assert.Equal(previous.Z, current.Z, 6);
        }

        Assert.All(
            plan.Toolpaths.SelectMany(path => path.Spans
                .Where(span => span.Kind == ToolpathSpanKind.Link)
                .Select(span => (path.Points, Span: span))),
            transition =>
            {
                Assert.Equal(1, transition.Span.MoveCount);
            });
    }

    [Fact]
    public void Pocket_KeepsTheCutterOffTheWalls()
    {
        var region = Rect(0, 0, 100, 100);
        var plan = PocketToolpaths.Generate(
            new[] { new MillingRegion(region) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new PocketOptions());

        // The cutter centre must never come closer to the wall than its own radius.
        foreach (var p in AllPoints(plan))
        {
            double clearance = Math.Min(
                Math.Min(p.X, 100 - p.X),
                Math.Min(p.Y, 100 - p.Y));
            Assert.True(clearance >= 3 - 0.3, $"cutter centre {clearance:F3}mm from the wall at ({p.X:F2},{p.Y:F2})");
        }
    }

    [Fact]
    public void Pocket_StockToLeaveHoldsTheCutterFurtherBack()
    {
        var plain = PocketToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 100)) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new PocketOptions());

        var withStock = PocketToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 100)) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new PocketOptions { StockToLeave = 2 });

        Assert.Equal(3, AllPoints(plain).Min(p => p.X), 1);
        Assert.Equal(5, AllPoints(withStock).Min(p => p.X), 1);
    }

    [Fact]
    public void Pocket_NeverCutsBelowTheRequestedDepth()
    {
        var plan = PocketToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 100)) },
            ToolGeometry.Flat(3),
            Depth(9, 2),
            new PocketOptions());

        foreach (var p in AllPoints(plan))
        {
            Assert.True(p.Z >= -9 - 1e-6, $"cut to {p.Z:F4}, past the -9 floor");
            Assert.True(p.Z <= 1e-6, $"path rose to {p.Z:F4}, above the surface");
        }
    }

    [Fact]
    public void Pocket_TakesEachIslandToFullDepthBeforeMovingOn()
    {
        // Re-entering every pocket once per depth pass would push chips back under the cutter.
        var plan = PocketToolpaths.Generate(
            new[]
            {
                new MillingRegion(Rect(0, 0, 60, 60)),
                new MillingRegion(Rect(200, 0, 260, 60)),
            },
            ToolGeometry.Flat(3),
            Depth(6, 2),
            new PocketOptions());

        var visited = new HashSet<int>();
        int current = -1;
        foreach (var path in plan.Toolpaths)
        {
            if (path.RegionIndex == current)
                continue;
            Assert.True(visited.Add(path.RegionIndex), "returned to a region after leaving it");
            current = path.RegionIndex;
        }

        Assert.Equal(2, visited.Count);
    }

    [Fact]
    public void Pocket_RebasesTheNextDisconnectedRegionNearThePreviousEndpoint()
    {
        var plan = PocketToolpaths.Generate(
            new[]
            {
                new MillingRegion(Rect(100, 0, 190, 90)),
                new MillingRegion(Rect(0, 0, 90, 90)),
            },
            ToolGeometry.Flat(3),
            Depth(2, 2),
            new PocketOptions());

        Assert.Equal(new[] { 0, 1 }, plan.Toolpaths
            .Select(path => path.RegionIndex)
            .Distinct()
            .ToArray());

        TaggedToolpath secondRegion = plan.Toolpaths
            .First(path => path.RegionIndex == 1);
        Assert.True(
            secondRegion.Points[0].X > 80,
            $"second region entered at far seam X={secondRegion.Points[0].X:F3}");
    }

    [Fact]
    public void Pocket_MultiPassRoutePausesCarrierForNearbyEntrySpirals()
    {
        var plan = PocketToolpaths.Generate(
            new[]
            {
                new MillingRegion(Rect(0, 0, 200, 20)),
                new MillingRegion(Rect(35, 22, 49, 36)),
                new MillingRegion(Rect(135, 22, 149, 36)),
            },
            ToolGeometry.Flat(2),
            Depth(6, 2),
            new PocketOptions { MaxRings = 1 });

        var regionSequence = plan.Toolpaths
            .Select(path => path.RegionIndex)
            .Where((region, index) => index == 0
                || region != plan.Toolpaths[index - 1].RegionIndex)
            .ToList();

        Assert.Equal(5, regionSequence.Count);
        Assert.Equal(0, regionSequence[0]);
        Assert.Equal(0, regionSequence[^1]);
        Assert.Equal(3, regionSequence.Count(region => region == 0));
        Assert.Equal(new[] { 1, 2 }, regionSequence
            .Where(region => region != 0)
            .Order());
    }

    [Fact]
    public void Profile_FinishesEachDisconnectedDecalBeforeTravellingToTheNext()
    {
        // Each decal has an outer contour and a cutout, so it produces multiple independent
        // profile roots. They must be kept together rather than interleaved by nearest travel.
        var plan = ProfileToolpaths.Generate(
            new[]
            {
                new MillingRegion(Rect(0, 0, 100, 80), new[] { Rect(10, 25, 30, 55) }),
                new MillingRegion(Rect(120, 0, 220, 80), new[] { Rect(130, 25, 150, 55) }),
            },
            ToolGeometry.Flat(3),
            Depth(4, 4),
            new ProfileOptions { Side = ProfileSide.OnLine });

        var completed = new HashSet<int>();
        int current = -1;
        foreach (var path in plan.Toolpaths)
        {
            if (path.RegionIndex == current)
                continue;

            Assert.True(
                completed.Add(path.RegionIndex),
                $"returned to disconnected decal {path.RegionIndex} after moving to decal {current}");
            current = path.RegionIndex;
        }

        Assert.Equal(2, completed.Count);
    }

    [Fact]
    public void Pocket_DepthPassesDescendInOrderWithinAnIsland()
    {
        var plan = PocketToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 100)) },
            ToolGeometry.Flat(3),
            Depth(9, 3),
            new PocketOptions());

        int previous = -1;
        foreach (var path in plan.Toolpaths)
        {
            Assert.NotNull(path.DepthPassIndex);
            Assert.True(path.DepthPassIndex >= previous, "depth passes went out of order");
            previous = path.DepthPassIndex!.Value;
        }
    }

    [Fact]
    public void Pocket_SpiralChainingCutsFewerSeparatePaths()
    {
        var region = new[] { new MillingRegion(Rect(0, 0, 100, 100)) };
        var tool = ToolGeometry.Flat(3);
        var depth = Depth(3, 3);

        var spiral = PocketToolpaths.Generate(region, tool, depth, new PocketOptions());
        var perRing = PocketToolpaths.Generate(region, tool, depth,
            new PocketOptions { SpiralChaining = false });

        Assert.True(spiral.Toolpaths.Count < perRing.Toolpaths.Count);
        Assert.True(spiral.Diagnostics.LinkLength <= perRing.Diagnostics.LinkLength + 1e-6);
        Assert.Contains(
            spiral.Toolpaths.SelectMany(path => path.Spans),
            span => span.Kind == ToolpathSpanKind.Link);
    }

    [Fact]
    public void Pocket_ReportsSweptArea()
    {
        var plan = PocketToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 100)) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new PocketOptions());

        Assert.NotEmpty(plan.SweptArea);
        double swept = plan.SweptArea.Sum(p => Math.Abs(PolygonOps.SignedArea(p)));
        Assert.True(swept > 1000, $"swept area {swept:F0} is implausibly small for a 100mm pocket");
    }

    [Fact]
    public void Pocket_RegionWithIsland_LeavesTheIslandStanding()
    {
        var plan = PocketToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 100), new[] { Rect(40, 40, 60, 60) }) },
            ToolGeometry.Flat(2),
            Depth(2, 2),
            new PocketOptions());

        // Nothing may enter the island plus the cutter radius.
        foreach (var p in AllPoints(plan))
        {
            bool insideIsland = p.X > 40 + 2 - 0.3 && p.X < 60 - 2 + 0.3
                             && p.Y > 40 + 2 - 0.3 && p.Y < 60 - 2 + 0.3;
            Assert.False(insideIsland, $"cut into the island at ({p.X:F2},{p.Y:F2})");
        }
    }

    [Fact]
    public void Pocket_ToolLargerThanRegion_ProducesNothing()
    {
        var plan = PocketToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 5, 5)) },
            ToolGeometry.Flat(10),
            Depth(3, 3),
            new PocketOptions());

        Assert.Empty(plan.Toolpaths);
    }

    [Fact]
    public void Pocket_ClipBoundaryTrimsThePath()
    {
        var plan = PocketToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 100)) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new PocketOptions { ClipBoundary = Rect(-10, -10, 50, 110) });

        Assert.NotEmpty(plan.Toolpaths);
        Assert.True(AllPoints(plan).Max(p => p.X) <= 50 + 1e-6);
    }

    // --- Profiling ------------------------------------------------------------

    [Fact]
    public void Profile_OutsideRunsOutsideTheContour()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 80, 80)) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new ProfileOptions { Side = ProfileSide.Outside });

        Assert.NotEmpty(plan.Toolpaths);
        Assert.Equal(17, AllPoints(plan).Min(p => p.X), 1);
        Assert.Equal(83, AllPoints(plan).Max(p => p.X), 1);
    }

    [Fact]
    public void Profile_InsideRunsInsideTheContour()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 80, 80)) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new ProfileOptions { Side = ProfileSide.Inside });

        Assert.Equal(23, AllPoints(plan).Min(p => p.X), 1);
        Assert.Equal(77, AllPoints(plan).Max(p => p.X), 1);
    }

    [Fact]
    public void Profile_OnLineTracesTheContourExactly()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 80, 80)) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new ProfileOptions { Side = ProfileSide.OnLine });

        Assert.Equal(20, AllPoints(plan).Min(p => p.X), 1);
        Assert.Equal(80, AllPoints(plan).Max(p => p.X), 1);
    }

    [Fact]
    public void Profile_StockToLeaveShiftsTheContourOutward()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 80, 80)) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new ProfileOptions { Side = ProfileSide.Outside, StockToLeave = 1 });

        Assert.Equal(16, AllPoints(plan).Min(p => p.X), 1);
    }

    [Fact]
    public void Profile_SpringPassesRepeatTheContour()
    {
        var single = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 80, 80)) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new ProfileOptions { Side = ProfileSide.Outside });

        var sprung = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 80, 80)) },
            ToolGeometry.Flat(3),
            Depth(3, 3),
            new ProfileOptions { Side = ProfileSide.Outside, SpringPasses = 2 });

        double singleCut = single.Diagnostics.CutLength;
        double sprungCut = sprung.Diagnostics.CutLength;

        Assert.True(sprungCut > singleCut * 2.5,
            $"two spring passes should roughly triple the cut length; {singleCut:F1} -> {sprungCut:F1}");
    }

    [Fact]
    public void Profile_RegionWithHole_CutsBothContours()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 100), new[] { Rect(40, 40, 60, 60) }) },
            ToolGeometry.Flat(2),
            Depth(2, 2),
            new ProfileOptions { Side = ProfileSide.Outside });

        // The outer wall grows outward and the hole shrinks inward.
        Assert.Equal(-2, AllPoints(plan).Min(p => p.X), 1);
        Assert.Contains(AllPoints(plan), p => p.X > 40 && p.X < 60);
        Assert.Equal(2, plan.Diagnostics.LinkLifts);
        Assert.Equal(0, plan.Toolpaths
            .SelectMany(path => path.Spans)
            .Count(span => span.Kind == ToolpathSpanKind.Link));
    }

    [Fact]
    public void Profile_RouteInspectionServicesFourNearbyLoopsFromTheLargeCycle()
    {
        var regions = new[]
        {
            new MillingRegion(Rect(0, 0, 100, 100)),
            new MillingRegion(Rect(20, -11, 30, -1)),
            new MillingRegion(Rect(101, 20, 111, 30)),
            new MillingRegion(Rect(70, 101, 80, 111)),
            new MillingRegion(Rect(-11, 60, -1, 70)),
        };
        var plan = ProfileToolpaths.Generate(
            regions,
            ToolGeometry.Flat(3),
            Depth(2, 2),
            new ProfileOptions
            {
                Side = ProfileSide.OnLine,
                Ramp = RampSettings.Plunge,
            });

        var regionSequence = plan.Toolpaths
            .Select(path => path.RegionIndex)
            .Where((region, index) => index == 0 || region != plan.Toolpaths[index - 1].RegionIndex)
            .ToList();

        Assert.Equal(9, regionSequence.Count);
        Assert.All(Enumerable.Range(0, 5), index => Assert.Equal(0, regionSequence[index * 2]));
        Assert.Equal(new[] { 1, 2, 3, 4 }, regionSequence.Where((_, index) => index % 2 == 1).Order());
        Assert.Equal(560, plan.Diagnostics.CutLength, 6);
        Assert.InRange(plan.Diagnostics.LinkLength, 8 - 1e-6, 8 + 1e-6);
        Assert.Equal(0, plan.Diagnostics.LinkLifts);
        Assert.Equal(8, plan.Toolpaths
            .SelectMany(path => path.Spans)
            .Count(span => span.Kind == ToolpathSpanKind.Link));
    }

    [Fact]
    public void Profile_NearbyLoopsLinkAtEachDepthWithoutReentryRamps()
    {
        var regions = new[]
        {
            new MillingRegion(Rect(0, 0, 100, 100)),
            new MillingRegion(Rect(20, -11, 30, -1)),
            new MillingRegion(Rect(101, 20, 111, 30)),
            new MillingRegion(Rect(70, 101, 80, 111)),
            new MillingRegion(Rect(-11, 60, -1, 70)),
        };
        var plan = ProfileToolpaths.Generate(
            regions,
            ToolGeometry.Flat(3),
            Depth(6, 2),
            new ProfileOptions
            {
                Side = ProfileSide.OnLine,
                // A 2mm step needs 50mm of ramp travel: helical on each 40mm
                // corner loop, but linear on the 400mm carrier.
                Ramp = RampSettings.FromRatio(25),
            });

        foreach (int regionIndex in Enumerable.Range(1, 4))
        {
            var paths = plan.Toolpaths
                .Where(path => path.RegionIndex == regionIndex)
                .ToList();
            Assert.Equal(new[] { 0, 1, 2 }, paths
                .Select(path => path.DepthPassIndex!.Value)
                .ToArray());
            Assert.All(paths, path =>
            {
                Assert.Equal(2, path.Spans.Count(span => span.Kind == ToolpathSpanKind.Link));
                Assert.DoesNotContain(path.Spans, span => span.Kind == ToolpathSpanKind.Ramp);
            });
        }

        Assert.Equal(new[] { 0, 1, 2 }, plan.Toolpaths
            .Where(path => path.RegionIndex == 0)
            .Select(path => path.DepthPassIndex!.Value)
            .Distinct()
            .Order());
        Assert.Equal(0, plan.Diagnostics.DepthFirstSpirals);
        Assert.Equal(0, plan.Diagnostics.PassesCombined);
        Assert.Equal(0, plan.Diagnostics.AvoidedRetracts);
        Assert.Equal(0, plan.Diagnostics.HelicalEntries);
        Assert.Equal(3, plan.Diagnostics.LinearRampEntries);
        Assert.Equal(0, plan.Diagnostics.LinkLifts);
        Assert.Equal(1680, plan.Diagnostics.CutLength, 6);
        Assert.Equal(24, plan.Diagnostics.LinkLength, 6);
    }

    [Fact]
    public void Profile_LinearLeafLoopContinuesFromEachPreviousRampEndpoint()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 10, 10)) },
            ToolGeometry.Flat(3),
            Depth(6, 2),
            new ProfileOptions
            {
                Side = ProfileSide.OnLine,
                Ramp = RampSettings.FromRatio(2),
            });

        var path = Assert.Single(plan.Toolpaths);
        Assert.Equal(2, path.DepthPassIndex);

        var ramps = path.Spans.Where(span => span.Kind == ToolpathSpanKind.Ramp).ToList();
        var cuts = path.Spans.Where(span => span.Kind == ToolpathSpanKind.Cut).ToList();
        Assert.Equal(3, ramps.Count);
        Assert.Equal(3, cuts.Count);
        for (int pass = 1; pass < ramps.Count; pass++)
            Assert.Equal(cuts[pass - 1].EndIndex, ramps[pass].StartIndex);

        Assert.Equal(0, path.Points[0].Z, 6);
        Assert.Equal(-6, path.Points[^1].Z, 6);
        Assert.Equal(1, plan.Diagnostics.DepthFirstSpirals);
        Assert.Equal(3, plan.Diagnostics.PassesCombined);
        Assert.Equal(2, plan.Diagnostics.AvoidedRetracts);
    }

    [Fact]
    public void Profile_AutoHelicalDepthFirstLoopPreservesHoldingTabs()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 180, 180)) },
            ToolGeometry.Flat(3),
            Depth(6, 2),
            new ProfileOptions
            {
                Side = ProfileSide.Outside,
                Ramp = RampSettings.FromRatio(400),
                Tabs = new TabOptions { Count = 4, Length = 8, Thickness = 1 },
            });

        var path = Assert.Single(plan.Toolpaths);
        var tabPoints = path.Spans
            .Where(span => span.Kind == ToolpathSpanKind.TabLift)
            .SelectMany(span => Enumerable.Range(span.StartIndex, span.PointCount))
            .Distinct()
            .Select(index => path.Points[index])
            .ToList();

        Assert.NotEmpty(tabPoints);
        Assert.Equal(-5, tabPoints.Max(point => point.Z), 6);
        Assert.Contains(path.Points, point => Math.Abs(point.Z - (-6)) < 1e-6);
        Assert.All(path.Points, point => Assert.True(point.Z >= -6 - 1e-6));
        Assert.Equal(1, plan.Diagnostics.DepthFirstSpirals);
        Assert.Equal(3, plan.Diagnostics.PassesCombined);
        Assert.Equal(2, plan.Diagnostics.AvoidedRetracts);
    }

    // --- Holding tabs ---------------------------------------------------------

    [Fact]
    public void Tabs_LiftTheCutterToTheTabHeight()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 180, 180)) },
            ToolGeometry.Flat(3),
            Depth(10, 10),
            new ProfileOptions
            {
                Side = ProfileSide.Outside,
                Tabs = new TabOptions { Count = 4, Length = 8, Thickness = 3 },
            });

        var liftSpans = plan.Toolpaths
            .SelectMany(t => t.Spans
                .Where(s => s.Kind == ToolpathSpanKind.TabLift)
                .Select(s => Enumerable.Range(s.StartIndex, s.PointCount).Select(i => t.Points[i]).ToList()))
            .ToList();

        Assert.NotEmpty(liftSpans);
        foreach (var lift in liftSpans)
        {
            // The plateau sits 3mm up from the -10 floor; the span also owns the vertical
            // step back down, so the deepest point is the floor itself.
            Assert.Equal(-7, lift.Max(p => p.Z), 6);
            Assert.True(lift.Count(p => Math.Abs(p.Z - (-7)) < 1e-9) >= 2, "tab had no flat top");
        }
    }

    [Fact]
    public void Tabs_LeaveTheRestOfTheContourAtFullDepth()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 180, 180)) },
            ToolGeometry.Flat(3),
            Depth(10, 10),
            new ProfileOptions
            {
                Side = ProfileSide.Outside,
                Tabs = new TabOptions { Count = 4, Length = 8, Thickness = 3 },
            });

        Assert.Contains(AllPoints(plan), p => Math.Abs(p.Z - (-10)) < 1e-6);
    }

    [Fact]
    public void Tabs_NeverCutDeeperThanTheOperation()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 180, 180)) },
            ToolGeometry.Flat(3),
            Depth(10, 5),
            new ProfileOptions
            {
                Side = ProfileSide.Outside,
                Tabs = new TabOptions { Count = 4, Length = 8, Thickness = 3 },
            });

        Assert.All(AllPoints(plan), p => Assert.True(p.Z >= -10 - 1e-6));
    }

    [Fact]
    public void Tabs_ThicknessIsClampedToTheCutDepth()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(20, 20, 180, 180)) },
            ToolGeometry.Flat(3),
            Depth(5, 5),
            new ProfileOptions
            {
                Side = ProfileSide.Outside,
                Tabs = new TabOptions { Count = 3, Length = 8, Thickness = 50 },
            });

        // A tab thicker than the cut would mean never cutting at all; it clamps to the surface.
        Assert.All(AllPoints(plan), p => Assert.True(p.Z <= 1e-6 && p.Z >= -5 - 1e-6));
    }

    [Fact]
    public void TabZones_AvoidCorners()
    {
        var square = Rect(0, 0, 100, 100);

        Assert.True(TabZonePlanner.TryBuildEvenlySpaced(
            square, 400, tabCount: 4, zoneLength: 10, toolRadius: 3, out var zones));

        double[] corners = { 0, 100, 200, 300 };
        foreach (var zone in zones)
        {
            double center = (zone.Start + zone.End) * 0.5;
            foreach (double corner in corners)
            {
                double distance = Math.Abs(center - corner);
                distance = Math.Min(distance, 400 - distance);
                Assert.True(distance >= 5 + 3 - 1e-3,
                    $"tab centred at {center:F2} sits on the corner at {corner:F0}");
            }
        }
    }

    [Fact]
    public void TabZones_RejectedWhenTheyWouldSwallowTheRing()
    {
        var zones = new[] { new TabZone(0, 95) };

        Assert.Null(TabZonePlanner.Normalize(zones, perimeter: 100));
        Assert.NotNull(TabZonePlanner.Normalize(new[] { new TabZone(0, 10) }, perimeter: 100));
    }

    [Fact]
    public void ProjectedTabZones_CanAuthoritativelyCoverAnEntireRing()
    {
        var zones = new[] { new TabZone(0, 0.008) };

        Assert.Null(TabZonePlanner.Normalize(zones, perimeter: 0.008));
        var projected = TabZonePlanner.NormalizeProjected(zones, perimeter: 0.008);

        Assert.NotNull(projected);
        Assert.Single(projected!);
    }

    [Fact]
    public void ProfileRoute_FullyProtectedIntermediateRingUsesATabHeightPortal()
    {
        var rings = new List<OffsetRing>
        {
            RingOffsetEngine.CreateRing(Rect(0, 0, 40, 40), level: 0, regionIndex: 0),
            RingOffsetEngine.CreateRing(Rect(45, 0, 45.002, 0.002), level: 0, regionIndex: 1),
            RingOffsetEngine.CreateRing(Rect(50, 0, 70, 20), level: 0, regionIndex: 2),
        };
        var root = new ProfileRouteNode(0);
        var protectedNode = new ProfileRouteNode(1);
        var leaf = new ProfileRouteNode(2);
        root.Children.Add(new ProfileRouteBranch(
            new ProfilePortal(0, new PointD(40, 0), 1, new PointD(45, 0), 5),
            protectedNode));
        protectedNode.Children.Add(new ProfileRouteBranch(
            new ProfilePortal(1, new PointD(45.002, 0), 2, new PointD(50, 0), 4.998),
            leaf));
        var route = new ProfileRouteForest(
            new[] { new ProfileRouteTree(root, new[] { 0, 1, 2 }) },
            PortalCount: 2,
            SourceProximity: 1.5);

        const double tabZ = -8;
        var plan = ProfileRouteMaterializer.Emit(
            rings,
            route,
            new DepthSchedule(10, 5),
            new ProfileOptions
            {
                Direction = MillingDirection.Climb,
                Ramp = RampSettings.FromRatio(8),
                Tabs = new TabOptions { Count = 1, Length = 10, Thickness = 2 },
            },
            ToolGeometry.Flat(1.5),
            CancellationToken.None,
            (ringIndex, points, spans) =>
            {
                if (ringIndex != 1)
                    return (points, spans);

                double cutZ = points.Min(point => point.Z);
                return TabZonePlanner.ApplyToClosedPath(
                    points,
                    spans,
                    new[] { new TabZone(0, rings[ringIndex].Perimeter) },
                    rings[ringIndex].Perimeter,
                    cutZ,
                    tabZ);
            });

        var protectedPaths = plan.Toolpaths
            .Where(path => path.RegionIndex == 1)
            .ToList();
        Assert.NotEmpty(protectedPaths);
        Assert.DoesNotContain(
            protectedPaths.SelectMany(path => path.Points),
            point => point.Z < tabZ - 1e-6);
    }

    [Fact]
    public void Tabs_AreNotSeatedWhereTheStockRunsOut()
    {
        // Stock is only 1mm wider than the part on the left and right, so the offcut there is
        // eaten by the kerf: a tab on those runs would bridge the part to air.
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 50)) },
            ToolGeometry.Flat(3),
            Depth(10, 10),
            new ProfileOptions
            {
                Side = ProfileSide.Outside,
                StockOutline = Rect(-4, -12, 104, 62),
                Tabs = new TabOptions { Count = 4, Length = 8, Thickness = 3 },
            });

        var tabPoints = TabLiftPoints(plan);

        Assert.NotEmpty(tabPoints);
        Assert.All(tabPoints, p => Assert.True(
            p.Y <= -3 + 0.05 || p.Y >= 53 - 0.05,
            $"tab seated at ({p.X:F2},{p.Y:F2}), where the stock runs out before the offcut does"));
    }

    [Fact]
    public void Tabs_UseEveryEdgeWhenTheStockHasRoomForThem()
    {
        var plan = ProfileToolpaths.Generate(
            new[] { new MillingRegion(Rect(0, 0, 100, 50)) },
            ToolGeometry.Flat(3),
            Depth(10, 10),
            new ProfileOptions
            {
                Side = ProfileSide.Outside,
                StockOutline = Rect(-20, -20, 120, 70),
                Tabs = new TabOptions { Count = 4, Length = 8, Thickness = 3 },
            });

        var tabPoints = TabLiftPoints(plan);

        Assert.NotEmpty(tabPoints);
        Assert.Contains(tabPoints, p => p.X <= -3 + 0.05 || p.X >= 103 - 0.05);
    }

    private static List<Point3D> TabLiftPoints(ToolpathPlan plan)
        => plan.Toolpaths
            .SelectMany(t => t.Spans
                .Where(s => s.Kind == ToolpathSpanKind.TabLift)
                .SelectMany(s => Enumerable.Range(s.StartIndex, s.PointCount).Select(i => t.Points[i])))
            .ToList();

    [Fact]
    public void TabZones_ZAtLiftsOnlyInsideAZone()
    {
        var zones = new[] { new TabZone(10, 20) };

        Assert.Equal(-10, TabZonePlanner.ZAt(5, zones, 100, -10, -7), 9);
        Assert.Equal(-7, TabZonePlanner.ZAt(15, zones, 100, -10, -7), 9);
        Assert.Equal(-10, TabZonePlanner.ZAt(25, zones, 100, -10, -7), 9);
        // Arc length wraps around the seam.
        Assert.Equal(-7, TabZonePlanner.ZAt(115, zones, 100, -10, -7), 9);
    }

    [Fact]
    public void TabZones_SplitEveryLapOfAHelicalRamp()
    {
        var ramped = RampPlanner.PlanClosedRing(
            Rect(0, 0, 10, 10),
            entryZ: 0,
            cutZ: -10,
            RampSettings.FromRatio(20) with
            {
                Strategy = RampStrategy.Helical,
                MaxLaps = 4,
            });

        Assert.Equal(4, ramped.Laps);

        var lifted = TabZonePlanner.ApplyToClosedPath(
            ramped.Points,
            ramped.Spans,
            new[] { new TabZone(2, 8) },
            perimeter: 40,
            cutZ: -10,
            tabZ: -7);

        int tabRuns = 0;
        for (int index = 1; index < lifted.Points.Count; index++)
        {
            Point3D from = lifted.Points[index - 1];
            Point3D to = lifted.Points[index];
            if (Math.Abs(from.Y) > 1e-9 || Math.Abs(to.Y) > 1e-9 || to.X <= from.X)
                continue;

            double midpointX = (from.X + to.X) * 0.5;
            if (midpointX <= 2 + 1e-9 || midpointX >= 8 - 1e-9)
                continue;

            tabRuns++;
            Assert.True(Math.Min(from.Z, to.Z) >= -7 - 1e-9,
                $"traversal {tabRuns} cut through the tab from Z={from.Z:F3} to Z={to.Z:F3}");
        }

        Assert.Equal(ramped.Laps + 1, tabRuns);
    }
}
