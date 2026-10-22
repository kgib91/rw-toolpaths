using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

public sealed record HoldingProfileMember(PreparedProfile Profile, double CutterRadius);

public static partial class HoldingTabPlanner
{
    /// <summary>
    /// Plan every stock prefix before emission, then preserve the same physical rectangles
    /// on every cutter. Intermediate offcuts participate even when a later cutter removes them.
    /// </summary>
    public static IReadOnlyList<HoldingTabPlan> PlanSequence(
        IReadOnlyList<HoldingProfileMember> members, List<PointD>? stockOutline,
        int tabCount, double tabLength, double tabHeightY, double ignoredSliverThickness = .25,
        CancellationToken cancellationToken = default)
    {
        if (members.Count == 0) return Array.Empty<HoldingTabPlan>();
        if (members.Any(m => !double.IsFinite(m.CutterRadius) || m.CutterRadius <= 0))
            throw new ArgumentException("Every holding-group member requires a positive cutter radius.", nameof(members));
        var plans = new List<HoldingTabPlan>();
        var prefixCuts = new List<List<List<PointD>>>();
        var cut = new List<List<PointD>>();
        var source = new List<List<PointD>>();
        var footprints = new List<HoldingTabFootprint>();
        double maximumRadius = members.Max(m => m.CutterRadius);
        double minimumRadius = members.Min(m => m.CutterRadius);
        foreach (var member in members)
        {
            cancellationToken.ThrowIfCancellationRequested();
            cut = PolygonOps.Union(cut.Concat(PolygonOps.BufferCenterlines(
                member.Profile.Rings.Select(r => r.Points).ToList(), member.CutterRadius)).ToList());
            source = PolygonOps.Union(source.Concat(member.Profile.SourceMaterial).ToList());
            prefixCuts.Add(cut);
            var plan = Plan(member.Profile.Rings.Select(r => r.Points).ToArray(),
                member.Profile.Rings.Select(r => r.Perimeter).ToArray(), stockOutline, source,
                maximumRadius, tabCount, tabLength, tabHeightY, ignoredSliverThickness, cut, minimumRadius);
            footprints.AddRange(plan.Report.Footprints);
            plans.Add(plan);
        }
        // Canonical deduplication avoids multiplying identical bridges from identical stages.
        footprints = footprints.GroupBy(f => string.Join(";", f.Corners.Select(p => $"{p.x:R},{p.y:R}")), StringComparer.Ordinal)
            .Select(g => g.First()).ToList();
        var stock = BuildStockOutline(stockOutline, cut, source, maximumRadius, tabLength);
        for (int i = 0; i < members.Count; i++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var plan = plans[i]; var member = members[i];
            plan.RingZones.Clear(); plan.Report.Footprints.Clear(); plan.Report.Footprints.AddRange(footprints);
            foreach (var footprint in footprints)
            {
                var c = FootprintCandidate(footprint);
                for (int ring = 0; ring < member.Profile.Rings.Count; ring++)
                    ProjectFootprintOntoRing(c, member.Profile.Rings[ring].Points,
                        member.Profile.Rings[ring].Perimeter, member.CutterRadius, ring, plan.RingZones);
            }
            foreach (var zones in plan.RingZones.Values) MergeZones(zones);
            ValidatePrefixConnectivity(plan.Report, stock, prefixCuts[i], footprints,
                minimumRadius, ignoredSliverThickness, tabCount, i);
            Console.WriteLine($"[HoldingTabs.Sequence] stage={i} radius={member.CutterRadius:R} rings={member.Profile.Rings.Count} commonFootprints={footprints.Count} zoned={plan.RingZones.Count} {plan.Report.Summary}");
        }
        return plans;
    }

    private static Candidate FootprintCandidate(HoldingTabFootprint footprint)
    {
        var a = footprint.Corners[0]; var b = footprint.Corners[1]; var c = footprint.Corners[2];
        double dx = b.x - a.x, dy = b.y - a.y;
        double span = Math.Sqrt(dx * dx + dy * dy);
        double wx = c.x - b.x, wy = c.y - b.y;
        return new Candidate(0, 0, (a.x + c.x) / 2, (a.y + c.y) / 2,
            (a.x + c.x) / 2, (a.y + c.y) / 2, dx / span, dy / span,
            span / 2, span / 2, Math.Sqrt(wx * wx + wy * wy) / 2, 0, 0, 0, 0);
    }

    private static void ValidatePrefixConnectivity(HoldingTabReport report, List<PointD> stock,
        List<List<PointD>> cut, IReadOnlyList<HoldingTabFootprint> footprints,
        double minimumRadius, double sliverThickness, int tabCount, int stage)
    {
        // Prefix-local failures may be resolved by footprints planned for another member. Reassess
        // both connectivity and the configured bridge quota against the authoritative common set.
        report.UnresolvedReasons.Clear(); report.UnresolvedOutlines.Clear(); report.UnresolvedCount = 0;
        report.TabbedCount = 0;
        var protectedCut = PolygonOps.Difference(cut, footprints.Select(f => f.Corners.ToList()).ToList());
        var components = PolygonOps.DifferenceToComponents(new[] { stock }, protectedCut);
        var pieces = components.Where(c => Math.Abs(PolygonArea(c.Outer)) - c.Holes.Sum(h => Math.Abs(PolygonArea(h)))
            >= minimumRadius * minimumRadius * .5).ToList();
        int anchor = -1; double area = -1;
        for (int i = 0; i < pieces.Count; i++)
        {
            double candidateArea = Math.Abs(PolygonArea(pieces[i].Outer));
            if (TouchesOutline(pieces[i].Outer, stock, Math.Max(.02, minimumRadius * .25)) && candidateArea > area)
            { area = candidateArea; anchor = i; }
        }
        if (anchor < 0 && pieces.Count > 0)
            anchor = pieces.Select((p, i) => (Area: Math.Abs(PolygonArea(p.Outer)), Index: i)).MaxBy(p => p.Area).Index;
        int disconnected = 0;
        for (int i = 0; i < pieces.Count; i++)
        {
            if (i == anchor) continue;
            var material = new List<List<PointD>> { pieces[i].Outer }; material.AddRange(pieces[i].Holes);
            if (sliverThickness > 0 && PolygonOps.Offset(material, -sliverThickness / 2).Count == 0) continue;
            disconnected++;
            report.UnresolvedOutlines.Add(pieces[i].Outer);
            report.UnresolvedReasons.Add($"stage#{stage}: retained component#{i} remains disconnected after common tab projection ({DescribePiece(pieces[i].Outer)}).");
        }
        var unbridged = PolygonOps.DifferenceToComponents(new[] { stock }, cut);
        var bridges = PolygonOps.Union(footprints.Select(f => f.Corners.ToList()).ToList());
        var bridgeComponents = PolygonOps.DifferenceToComponents(bridges, Array.Empty<List<PointD>>());
        var pieceBounds = unbridged.Select(p => GetBounds(p.Outer)).ToArray();
        var pieceMaterials = unbridged.Select(p => new[] { p.Outer }.Concat(p.Holes).ToList()).ToArray();
        var bridgeCounts = new int[unbridged.Count];
        foreach (var bridge in bridgeComponents)
        {
            var bounds = GetBounds(bridge.Outer);
            var bridgeMaterial = new[] { bridge.Outer }.Concat(bridge.Holes).ToList();
            var contacts = Enumerable.Range(0, unbridged.Count).Where(i =>
                pieceBounds[i].MaxX >= bounds.MinX && pieceBounds[i].MinX <= bounds.MaxX
                && pieceBounds[i].MaxY >= bounds.MinY && pieceBounds[i].MinY <= bounds.MaxY
                && PolygonOps.Intersect(pieceMaterials[i], bridgeMaterial).Count > 0).ToArray();
            if (contacts.Length > 1) foreach (int contact in contacts) bridgeCounts[contact]++;
        }
        var carriers = unbridged.Where(p => TouchesOutline(p.Outer, stock, Math.Max(.02, minimumRadius * .25))).ToList();
        var originalAnchor = (carriers.Count > 0 ? carriers : unbridged).MaxBy(p => Math.Abs(PolygonArea(p.Outer)));
        int quotaShortfalls = 0;
        for (int pieceIndex = 0; pieceIndex < unbridged.Count; pieceIndex++)
        {
            var piece = unbridged[pieceIndex];
            if (ReferenceEquals(piece, originalAnchor)) continue;
            var material = new List<List<PointD>> { piece.Outer }; material.AddRange(piece.Holes);
            if (Math.Abs(PolygonArea(piece.Outer)) - piece.Holes.Sum(h => Math.Abs(PolygonArea(h))) < minimumRadius * minimumRadius * .5
                || sliverThickness > 0 && PolygonOps.Offset(material, -sliverThickness / 2).Count == 0) continue;
            int quota = bridgeCounts[pieceIndex];
            if (quota >= tabCount) { report.TabbedCount++; continue; }
            quotaShortfalls++;
            report.UnresolvedOutlines.Add(piece.Outer);
            report.UnresolvedReasons.Add($"stage#{stage}: exact common bridge quota unavailable: capacity={quota}/{tabCount} ({DescribePiece(piece.Outer)}).");
        }
        report.UnresolvedCount = disconnected + quotaShortfalls;
        report.PieceCensus.Add($"sequence connectivity: stage={stage}, components={pieces.Count}, anchor={anchor}, disconnected={disconnected}, quotaShortfalls={quotaShortfalls}");
        Console.WriteLine($"[HoldingTabs.Connectivity] stage={stage} components={pieces.Count} anchor={anchor} disconnected={disconnected} quotaShortfalls={quotaShortfalls} featureRadius={minimumRadius:R}");
    }

    private static string DescribePiece(List<PointD> outline)
    {
        var bounds = GetBounds(outline);
        return FormattableString.Invariant($"bounds=({bounds.MinX:F2},{bounds.MinY:F2})..({bounds.MaxX:F2},{bounds.MaxY:F2}) mm");
    }
}
