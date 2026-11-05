using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Optional fill selector, never a stock or cutter-clearance certificate.
/// Legal virtual centres are an erosion; their disc sweep is the opening.
/// Rest material is contoured as a filled region; source rings are not clipped.</summary>
internal sealed class VCarveDiscSweepExclusion
{
    internal IReadOnlyList<IReadOnlyList<PointD>> SweptRegion { get; }
    // Keep exactly tangent centre sets representable as polygons. This is a
    // four-grid-quantum geometric selection tolerance, not cutter compensation.
    internal const double TangencyTolerance = 4 / PathUtils.Scale;

    internal VCarveDiscSweepExclusion(IEnumerable<IReadOnlyList<PointD>> domain,
        double radius, CancellationToken token, Action<string>? diagnostic, string reference)
    {
        if (!double.IsFinite(radius) || radius <= 0) throw new ArgumentOutOfRangeException(nameof(radius));
        long started = Stopwatch.GetTimestamp();
        token.ThrowIfCancellationRequested();
        var source = domain.ToArray();
        double constructionRadius = Math.Max(0, radius - TangencyTolerance);
        SweptRegion = PolygonOps.Intersect(source, PolygonOps.Open(source, constructionRadius, .001));
        token.ThrowIfCancellationRequested();
        diagnostic?.Invoke($"stage=disc-sweep event=prepared reference={reference} radius={radius:R} "
            + $"constructionRadius={constructionRadius:R} tangencyTolerance={TangencyTolerance:R} "
            + $"sourceRings={source.Length} sweepRings={SweptRegion.Count} "
            + $"sourceArea={Math.Abs(source.Sum(PolygonOps.SignedArea)):R} sweepArea={Math.Abs(SweptRegion.Sum(PolygonOps.SignedArea)):R} "
            + $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:R} "
            + "selection=fill-minus-dilate-erode radiusNotDiameter=True fillLimitUnchanged=True "
            + "actualRougherFootprintUsed=False stockCredit=False physicalGuardsRequired=True");
    }

    internal List<OffsetRing> Fill(IReadOnlyList<IReadOnlyList<PointD>> fillArea,
        IReadOnlyList<IReadOnlyList<PointD>> actualCleared,double pitch,double maximumInset,
        CancellationToken token,Action<string>? diagnostic)
    {
        token.ThrowIfCancellationRequested();
        var excluded=PolygonOps.Intersect(SweptRegion,actualCleared);
        var rest=PolygonOps.Difference(fillArea,excluded);
        var regions=PolygonOps.DifferenceToComponents(rest,Array.Empty<IReadOnlyList<PointD>>())
            .Select(c=>new MillingRegion(c.Outer,c.Holes)).ToArray();
        double geometricReach=regions.Select(r=>Math.Min(r.Outer.Max(p=>p.x)-r.Outer.Min(p=>p.x),
            r.Outer.Max(p=>p.y)-r.Outer.Min(p=>p.y))/2).DefaultIfEmpty(0).Max();
        double reach=Math.Min(maximumInset,geometricReach);
        int intervals=(int)Math.Min(RingOffsetEngine.AbsoluteRingLimit-1,Math.Floor(reach/pitch+1e-10));
        var rings=MedialAxisGeometry.BuildConcentricFill(regions,new RingOffsetOptions(0,pitch) {
            MaximumInset=intervals*pitch,RingLimit=intervals+1,MinimumRingArea=0,
            Tolerances=GeometryTolerances.Default with {ArcTolerance=.001,SimplifyTolerance=0},
        },sweepRadius:0,preserveTerminalPaths:false,cancellationToken:token);
        if(rings.Any(r=>!r.IsClosed))throw new InvalidOperationException("Rest fill lost its closed contour topology.");
        diagnostic?.Invoke($"stage=rest-fill event=contoured inputArea={Math.Abs(fillArea.Sum(PolygonOps.SignedArea)):R} "
            +$"virtualSweepArea={Math.Abs(SweptRegion.Sum(PolygonOps.SignedArea)):R} "
            +$"verifiedExclusionArea={Math.Abs(excluded.Sum(PolygonOps.SignedArea)):R} "
            +$"remainingArea={Math.Abs(rest.Sum(PolygonOps.SignedArea)):R} regions={regions.Length} rings={rings.Count} pitch={pitch:R} "
            +"source=fill-area-minus-verified-disc-sweep ringClipping=False closedLoops=True unknownRoughingRetainsFill=True physicalGuardsRequired=True");
        if(MillingGeometryTrace.Observer.Value is not null)
            MillingGeometryTrace.Capture("rest-fill-region",0,rest.Select(r=>new TaggedToolpath(
                r.Select(p=>new Point3D(p.x,p.y,0)).ToArray(),Array.Empty<ToolpathSpan>(),0,"rest-fill-region",null,detectCurves:false)).ToArray());
        return rings;
    }

}
