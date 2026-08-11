using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Holding-tab configuration for a profile operation.</summary>
public sealed record TabOptions
{
    /// <summary>Tabs per retained piece.</summary>
    public int Count { get; init; } = 4;

    /// <summary>Tab length along the cut path.</summary>
    public double Length { get; init; } = 5.0;

    /// <summary>Height of retained material measured up from the bottom of the cut.</summary>
    public double Thickness { get; init; } = 2.0;

    /// <summary>
    /// Plan tabs in material space so every piece that would otherwise come loose is bridged,
    /// instead of just spacing tabs evenly around each loop. Costlier, but the only mode that
    /// can prove no offcut is left free to move under the cutter.
    /// </summary>
    public bool ConstrainFloating { get; init; }

    /// <summary>Remnants thinner than this are treated as swarf rather than parts to retain.</summary>
    public double IgnoredSliverThickness { get; init; } = 0.25;
}

/// <summary>Configuration for contour/profile milling.</summary>
public sealed record ProfileOptions
{
    public ProfileSide Side { get; init; } = ProfileSide.Outside;

    /// <summary>
    /// Whether an outside profile cuts each material component's outer boundary.
    /// </summary>
    public bool IncludeOuterEnvelopes { get; init; } = true;

    /// <summary>
    /// Whether an outside profile omits boundaries around interior voids.
    /// </summary>
    public bool ExcludeInnerEnvelopes { get; init; }

    /// <summary>Material deliberately left on the wall for a later finishing pass.</summary>
    public double StockToLeave { get; init; }

    /// <summary>Extra passes at final depth that remove the deflection the first pass left.</summary>
    public int SpringPasses { get; init; }

    public MillingDirection Direction { get; init; } = MillingDirection.Climb;

    public RampSettings Ramp { get; init; } = RampSettings.FromRatio(2.0);

    public GeometryTolerances Tolerances { get; init; } = GeometryTolerances.Default;

    /// <summary>Holding tabs; <c>null</c> cuts straight through.</summary>
    public TabOptions? Tabs { get; init; }

    /// <summary>Stock frame used by material-space tab planning; falls back to a padded bounding box.</summary>
    public IReadOnlyList<PointD>? StockOutline { get; init; }

    /// <summary>Optional boundary that rings are trimmed to.</summary>
    public IReadOnlyList<PointD>? ClipBoundary { get; init; }
}

/// <summary>
/// Cuts a single contour around each region rather than clearing its interior.
///
/// <para>
/// Regions are first decomposed into connected components so tab planning can tell genuine
/// material from the voids punched through it. Offsetting a region without that step would let a
/// tab be placed on a bridge that does not exist in the source design.
/// </para>
/// </summary>
public static class ProfileToolpaths
{
    /// <summary>Category tag applied to profile passes.</summary>
    public const string Category = "profile";

    public static ToolpathPlan Generate(
        IReadOnlyList<MillingRegion> regions,
        ToolGeometry tool,
        DepthSchedule depth,
        ProfileOptions options,
        CancellationToken cancellationToken = default)
    {
        long t0 = PerfLog.Start();

        if (regions.Count == 0 || tool.Radius <= 1e-6 || depth.Depth <= 0)
            return ToolpathPlan.Empty;

        double offset = options.Side switch
        {
            ProfileSide.Outside => tool.Radius + options.StockToLeave,
            ProfileSide.Inside => -(tool.Radius + options.StockToLeave),
            _ => 0.0,
        };

        var rings = new List<OffsetRing>();
        var sourceMaterial = new List<List<PointD>>();
        var sourceComponents = new List<ProfileSourceComponent>();
        int componentIndex = 0;
        bool filterOutsideBoundaries = options.Side == ProfileSide.Outside;
        bool includeOuter = !filterOutsideBoundaries || options.IncludeOuterEnvelopes;
        bool includeInner = !filterOutsideBoundaries || !options.ExcludeInnerEnvelopes;

        foreach (var region in regions)
        {
            cancellationToken.ThrowIfCancellationRequested();

            var components = PolygonOps.DifferenceToComponents(
                new[] { region.Outer },
                region.Holes);

            foreach (var component in components)
            {
                var material = new List<IReadOnlyList<PointD>>(1 + component.Holes.Count) { component.Outer };
                material.AddRange(component.Holes);

                sourceMaterial.Add(component.Outer);
                sourceMaterial.AddRange(component.Holes);
                sourceComponents.Add(new ProfileSourceComponent(
                    componentIndex,
                    material.Select(boundary => (IReadOnlyList<PointD>)boundary).ToList()));

                var contours = Math.Abs(offset) < 1e-9
                    ? material.Select(boundary => new List<PointD>(boundary)).ToList()
                    : PolygonOps.Offset(
                        material,
                        offset,
                        options.Tolerances.ArcTolerance,
                        options.Tolerances.SimplifyTolerance);

                foreach (var contour in contours)
                {
                    if (contour.Count < 3)
                        continue;

                    rings.Add(RingOffsetEngine.CreateRing(contour, level: 0, componentIndex));
                }

                componentIndex++;
            }
        }

        // Compensation needs every material boundary: omitting an outer boundary before
        // PolygonOps.Offset reinterprets its holes as positive islands. Filter the final rings
        // instead, where IsHole preserves each compensated contour's material polarity.
        if (filterOutsideBoundaries)
            rings = rings.Where(ring => ring.IsHole ? includeInner : includeOuter).ToList();

        if (options.ClipBoundary is { Count: >= 3 })
            rings = ClipRings(rings, options.ClipBoundary);

        if (rings.Count == 0)
            return ToolpathPlan.Empty;

        HoldingTabReport? tabReport = null;
        var tabHook = BuildTabHook(rings, sourceMaterial, tool, depth, options, out tabReport);

        ProfileRouteForest route = ProfileRoutePlanner.Plan(
            rings,
            sourceComponents,
            tool.Radius,
            cancellationToken);
        ToolpathPlan plan = ProfileRouteMaterializer.Emit(
            rings,
            route,
            depth,
            options,
            tool,
            cancellationToken,
            tabHook);

        PerfLog.Stop("ProfileToolpaths.Generate", t0,
            $"regions={regions.Count} side={options.Side} outer={includeOuter} inner={includeInner} rings={rings.Count} " +
            $"paths={plan.Toolpaths.Count} tabs={(options.Tabs is null ? "off" : tabReport?.Summary ?? "on")}");

        return new ToolpathPlan(
            plan.Toolpaths.ToList(),
            plan.SweptArea.ToList(),
            plan.Diagnostics,
            tabReport);
    }

    /// <summary>
    /// Resolves tab zones per ring, then returns a hook that lifts each emitted path over them.
    /// </summary>
    private static Func<int, List<Point3D>, List<ToolpathSpan>, (List<Point3D>, List<ToolpathSpan>)>?
        BuildTabHook(
            IReadOnlyList<OffsetRing> rings,
            IReadOnlyList<List<PointD>> sourceMaterial,
            ToolGeometry tool,
            DepthSchedule depth,
            ProfileOptions options,
            out HoldingTabReport? report)
    {
        report = null;

        var tabs = options.Tabs;
        if (tabs is null || tabs.Count < 1 || tabs.Length <= 0)
            return null;

        double thickness = Math.Clamp(tabs.Thickness, 0, depth.Depth);
        double tabZ = depth.SurfaceZ - depth.Depth + thickness;
        if (thickness <= 1e-6)
            return null;

        var zonesByRing = new Dictionary<int, TabZone[]>();

        if (tabs.ConstrainFloating)
        {
            var ringPoints = rings.Select(r => r.Points).ToList();
            var perimeters = rings.Select(r => r.Perimeter).ToList();

            var plan = HoldingTabPlanner.Plan(
                ringPoints,
                perimeters,
                options.StockOutline is null ? null : new List<PointD>(options.StockOutline),
                sourceMaterial.Select(m => new List<PointD>(m)).ToList(),
                tool.Radius,
                tabs.Count,
                tabs.Length,
                tabZ,
                tabs.IgnoredSliverThickness);

            report = plan.Report;

            foreach (var (ringIndex, intervals) in plan.RingZones)
            {
                var zones = intervals.Select(i => new TabZone(i.Start, i.End)).ToList();
                var normalized = TabZonePlanner.NormalizeProjected(
                    zones, rings[ringIndex].Perimeter);
                if (normalized is null)
                    continue;

                zonesByRing[ringIndex] = normalized;
            }
        }
        else
        {
            // Keep a cutter-width of clearance either side so the tab is not undercut as the
            // tool rolls onto and off it.
            double protectedLength = tabs.Length + tool.Radius * 2;
            var bridgesMaterial = BuildBridgeTest(
                rings, sourceMaterial, tool.Radius, tabs.Length, options.StockOutline);

            int seated = 0;
            int starvedRings = 0;
            for (int i = 0; i < rings.Count; i++)
            {
                if (!TabZonePlanner.TryBuildEvenlySpaced(
                        rings[i].Points, rings[i].Perimeter, tabs.Count, protectedLength, tool.Radius,
                        out var zones, bridgesMaterial))
                {
                    starvedRings++;
                    continue;
                }

                var normalized = TabZonePlanner.Normalize(zones, rings[i].Perimeter);
                if (normalized is null)
                {
                    starvedRings++;
                    continue;
                }

                zonesByRing[i] = normalized;
                seated += normalized.Length;
            }

            PerfLog.Stop("ProfileToolpaths.TabSeats", PerfLog.Start(),
                $"rings={rings.Count} tabbedRings={zonesByRing.Count} starvedRings={starvedRings} " +
                $"seated={seated}/{rings.Count * tabs.Count} " +
                $"bridgeTest={(bridgesMaterial is null ? "unavailable" : "on")}");
        }

        if (zonesByRing.Count == 0)
            return null;

        return (ringIndex, points, spans) =>
        {
            if (!zonesByRing.TryGetValue(ringIndex, out var zones))
                return (points, spans);

            double seamArc = ArcPositionOnRing(
                rings[ringIndex].Points,
                new PointD(points[0].X, points[0].Y));
            TabZone[] rebasedZones = options.Direction == MillingDirection.Conventional
                ? zones.Select(zone => new TabZone(
                    seamArc - zone.End,
                    seamArc - zone.Start)).ToArray()
                : zones.Select(zone => new TabZone(
                    zone.Start - seamArc,
                    zone.End - seamArc)).ToArray();

            double pathCutZ = points.Min(point => point.Z);
            var lifted = TabZonePlanner.ApplyToClosedPath(
                points, spans, rebasedZones, rings[ringIndex].Perimeter, pathCutZ, tabZ);
            return (lifted.Points, lifted.Spans);
        };
    }

    private static double ArcPositionOnRing(
        IReadOnlyList<PointD> ring,
        PointD target)
    {
        var loop = RampPlanner.Normalize(ring);
        double bestDistanceSquared = double.MaxValue;
        double bestArc = 0;
        double arc = 0;

        for (int i = 0; i < loop.Count; i++)
        {
            var start = loop[i];
            var end = loop[(i + 1) % loop.Count];
            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double lengthSquared = dx * dx + dy * dy;
            double length = Math.Sqrt(lengthSquared);
            if (length <= 1e-12)
                continue;

            double position = Math.Clamp(
                ((target.x - start.x) * dx + (target.y - start.y) * dy) / lengthSquared,
                0,
                1);
            double closestX = start.x + dx * position;
            double closestY = start.y + dy * position;
            double targetDx = target.x - closestX;
            double targetDy = target.y - closestY;
            double distanceSquared = targetDx * targetDx + targetDy * targetDy;
            if (distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;
                bestArc = arc + length * position;
            }
            arc += length;
        }

        return bestArc;
    }

    /// <summary>
    /// Builds the "is this seat actually a bridge?" test for evenly spaced tabs.
    ///
    /// <para>
    /// A tab only retains a part if stock survives the cut on <b>both</b> sides of the kerf, so
    /// the test probes across the cut into the material model that is left once every ring has
    /// been swept. Spacing alone cannot tell a bridge from a seat on a ring running along the
    /// edge of the stock, where the tab is machined into air and holds nothing.
    /// </para>
    /// </summary>
    private static Func<PointD, PointD, bool>? BuildBridgeTest(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<List<PointD>> sourceMaterial,
        double toolRadius,
        double tabLength,
        IReadOnlyList<PointD>? stockOutline)
    {
        var ringPolys = new List<List<PointD>>(rings.Count);
        foreach (var ring in rings)
            if (ring.Points.Count >= 3)
                ringPolys.Add(ring.Points);

        var cutRegion = PolygonOps.BufferCenterlines(ringPolys, toolRadius);
        if (cutRegion.Count == 0)
            return null;

        var material = sourceMaterial.Select(m => new List<PointD>(m)).ToList();
        var stock = HoldingTabPlanner.BuildStockOutline(
            stockOutline is null ? null : new List<PointD>(stockOutline),
            cutRegion,
            material,
            toolRadius,
            tabLength);
        if (stock.Count < 3)
            return null;

        var remaining = PolygonOps.Difference(new List<List<PointD>> { stock }, cutRegion);
        if (remaining.Count == 0)
            return null;

        // Clear the kerf wall by a machinable web before calling the far side material: a sliver
        // narrower than this tears off with the tab still attached to it.
        double reach = toolRadius + Math.Max(0.25, toolRadius * 0.2);

        return (point, normal) =>
            PolygonOps.Contains(remaining, point.x + normal.x * reach, point.y + normal.y * reach)
            && PolygonOps.Contains(remaining, point.x - normal.x * reach, point.y - normal.y * reach);
    }

    private static List<OffsetRing> ClipRings(
        IReadOnlyList<OffsetRing> rings,
        IReadOnlyList<PointD> boundary)
    {
        var clipped = new List<OffsetRing>(rings.Count);
        foreach (var ring in rings)
        {
            foreach (var fragment in PolygonOps.Clip(new[] { ring.Points }, boundary))
            {
                if (fragment.Count >= 3)
                    clipped.Add(RingOffsetEngine.CreateRing(
                        fragment, ring.Level, ring.RegionIndex, ring.IsHole));
            }
        }
        return clipped;
    }
}
