using System.Text.Json;
using RW.Toolpaths;

namespace RW.Toolpaths.Milling.ReviewedSource;

// Explicit artwork resolution requested by the user. This removes whole short
// terminal tasks, not short command subdivisions. The raw design graph stays
// available. One pass on that immutable graph prevents cumulative erosion.
public static class ReviewedVCarveDetailResolution
{

    public sealed record Branch(int Tip, int Root, int[] Edges, double PlanarLengthMm,
        double PoseLengthMm, bool Omitted, string Decision);
    public sealed record Receipt(int Region, int OriginalEdges, int RetainedEdges,
        int OmittedBranches, int OmittedEdges, double MaximumOmittedPoseLengthMm, Branch[] Branches);
    public static (ReviewedVCarveConstructor.Result Source, Receipt Accounting) Apply(ReviewedVCarveConstructor.Result raw,double MinimumFeatureMm)
    {
        var adjacent = Enumerable.Range(0, raw.Nodes.Length).Select(_ => new List<int>()).ToArray();
        for (int i = 0; i < raw.Edges.Length; i++)
        { adjacent[raw.Edges[i].A].Add(i); adjacent[raw.Edges[i].B].Add(i); }
        var branches = new List<Branch>();
        for (int tip = 0; tip < adjacent.Length; tip++)
        {
            if (adjacent[tip].Count != 1) continue;
            int at = tip, previous = -1; double planar = 0, pose = 0; var edges = new List<int>();
            do
            {
                int i = adjacent[at].Single(i => i != previous); var e = raw.Edges[i];
                int next = e.A == at ? e.B : e.A; var a = raw.Nodes[at]; var b = raw.Nodes[next];
                double xy2 = Math.Pow(b.X - a.X, 2) + Math.Pow(b.Y - a.Y, 2);
                planar += Math.Sqrt(xy2); pose += Math.Sqrt(xy2 + Math.Pow(b.Z - a.Z, 2));
                edges.Add(i); at = next; previous = i;
            } while (adjacent[at].Count == 2);
            bool eligible = adjacent[at].Count >= 3 && pose < MinimumFeatureMm
                && edges.All(i => raw.Edges[i].Owner.StartsWith("medial:", StringComparison.Ordinal));
            string reason = adjacent[at].Count < 3 ? "standalone-stroke-retained"
                : pose >= MinimumFeatureMm ? "feature-at-or-above-resolution-retained"
                : !eligible ? "depth-cap-or-point-task-retained" : "sub-resolution-terminal-branch";
            branches.Add(new(tip, at, edges.ToArray(), planar, pose, eligible, reason));
        }
        // A compact whole component may have only short spokes. Retain its two
        // longest arms rather than deleting the authored component itself.
        foreach (var group in branches.Where(b => b.Omitted).GroupBy(b => b.Root))
        {
            if (group.Count() != adjacent[group.Key].Count) continue;
            foreach (var b in group.OrderByDescending(b => b.PoseLengthMm).ThenBy(b => b.Tip).Take(2).ToArray())
            {
                int i = branches.IndexOf(b);
                branches[i] = b with { Omitted = false, Decision = "whole-component-core-retained" };
            }
        }
        var omitted = branches.Where(b => b.Omitted).ToArray();
        var remove = omitted.SelectMany(b => b.Edges).ToHashSet();
        var kept = raw.Edges.Where((_, i) => !remove.Contains(i)).ToArray();
        foreach (var branch in omitted)
            if (!kept.Any(e => e.A == branch.Root || e.B == branch.Root))
                throw new InvalidOperationException($"stage=detail-resolution region={raw.Region} tip={branch.Tip} root={branch.Root} reason=missing-retained-root-pose");
        var receipt = new Receipt(raw.Region, raw.Edges.Length, kept.Length, omitted.Length, remove.Count,
            omitted.Select(b => b.PoseLengthMm).DefaultIfEmpty().Max(), branches.ToArray());
        Console.WriteLine($"[DetailResolution] region={raw.Region} minimumFeatureMm={MinimumFeatureMm:R} rawEdges={raw.Edges.Length} retainedEdges={kept.Length} omittedBranches={omitted.Length} omittedEdges={remove.Count} maximumOmittedPoseLengthMm={receipt.MaximumOmittedPoseLengthMm:R} pass=single-immutable-source inventedCutEdges=0");
        foreach (var b in omitted)
            Console.WriteLine($"[DetailResolutionBranch] region={raw.Region} tip={b.Tip} root={b.Root} tipXYZ={raw.Nodes[b.Tip]} rootXYZ={raw.Nodes[b.Root]} planarMm={b.PlanarLengthMm:R} poseMm={b.PoseLengthMm:R} edges={b.Edges.Length} decision={b.Decision}");
        return (raw with { Edges = kept }, receipt);
    }

}
