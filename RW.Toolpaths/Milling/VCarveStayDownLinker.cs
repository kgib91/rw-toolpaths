using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Joins protected cuts before surface-origin stepdown scheduling. Every added
/// chord needs its own original physical guards; nearby endpoints alone grant no clearance.</summary>
internal static class VCarveStayDownLinker
{
    internal static List<TaggedToolpath> Join(IReadOnlyList<TaggedToolpath> source,
        double maximumDistance, Func<Point3D, Point3D, bool> legalChord,
        CancellationToken token, Action<string>? diagnostic = null)
    {
        token.ThrowIfCancellationRequested();
        var output = new List<TaggedToolpath>();
        long accepted = 0, rejected = 0, distant = 0, coincident = 0;
        double addedLength = 0;
        Point3D? previous = null;
        for (int first = 0; first < source.Count;)
        {
            token.ThrowIfCancellationRequested();
            var representative = source[first];
            int end = first + 1;
            while (end < source.Count && source[end].RegionIndex == representative.RegionIndex
                && source[end].Category == representative.Category
                && source[end].DepthPassIndex == representative.DepthPassIndex) end++;
            var group = source.Skip(first).Take(end - first).ToArray();
            // Existing ramps, tabs, or other motion modes keep their original visits.
            bool eligible = group.All(path => path.Points.Count >= 2
                && path.Spans.All(span => span.Kind == ToolpathSpanKind.Cut));
            if (!eligible)
            {
                output.AddRange(group);
                previous = group.LastOrDefault(path => path.Points.Count > 0).Points?.Last();
                first = end;
                continue;
            }
            var samples = group.Select(path => (IReadOnlyList<PointD>)new PointD[] {
                new(path.Points[0].X, path.Points[0].Y), new(path.Points[^1].X, path.Points[^1].Y) }).ToArray();
            int[] order = TravelOptimizer.Order(samples,
                previous is { } p ? new PointD(p.X, p.Y) : null,
                label: $"vcarve-stay-down region={representative.RegionIndex} category={representative.Category}",
                cancellationToken: token);
            List<Point3D>? chain = null;
            TaggedToolpath owner = representative;
            foreach (int index in order)
            {
                token.ThrowIfCancellationRequested();
                var path = group[index];
                var points = path.Points;
                if (previous is { } exit && Distance(exit, points[^1]) < Distance(exit, points[0]))
                    points = points.Reverse().ToArray();
                bool join = false;
                if (chain is not null)
                {
                    var a = chain[^1]; var b = points[0];
                    double length = Distance(a, b);
                    if (a == b) { join = true; coincident++; }
                    else if (length > 1e-9 && length <= maximumDistance)
                    {
                        join = legalChord(a, b);
                        if (join) { accepted++; addedLength += length; }
                        else rejected++;
                        if ((join ? accepted : rejected) <= 4)
                            diagnostic?.Invoke($"stage=smooth-stay-down event={(join ? "accept" : "retain-separate")} "
                                + $"from=({a.X:R},{a.Y:R},{a.Z:R}) to=({b.X:R},{b.Y:R},{b.Z:R}) length={length:R}");
                    }
                    else distant++;
                    if (!join) Flush();
                }
                if (chain is null) { owner = path; chain = points.ToList(); }
                else
                {
                    int skip = chain[^1] == points[0] ? 1 : 0;
                    for (int i = skip; i < points.Count; i++) chain.Add(points[i]);
                }
                previous = chain[^1];
            }
            Flush();
            first = end;

            void Flush()
            {
                if (chain is null) return;
                output.Add(new TaggedToolpath(chain.ToArray(), Array.Empty<ToolpathSpan>(),
                    owner.RegionIndex, owner.Category, owner.DepthPassIndex, owner.VisitIdentity, detectCurves: false));
                chain = null;
            }
        }
        diagnostic?.Invoke($"stage=smooth-stay-down event=complete sourceVisits={source.Count} outputVisits={output.Count} "
            + $"acceptedLinks={accepted} coincidentLinks={coincident} rejectedPhysicalLinks={rejected} distantLinks={distant} "
            + $"maximumDistance={maximumDistance:R} addedCutLength={addedLength:R} "
            + "originalCutChordsRetained=True beforeSurfaceOriginStepdown=True connectionsAreCuttingMoves=True");
        return output;
    }

    private static double Distance(Point3D a, Point3D b)
        => Math.Sqrt(Math.Pow(b.X - a.X, 2) + Math.Pow(b.Y - a.Y, 2));
}
