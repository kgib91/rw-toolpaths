namespace RW.Toolpaths.Milling;

/// <summary>Selects remaining source arcs from already protected cutting curves.
/// Every omitted original chord has its whole posted cutter volume in REAL prior
/// stock. Literal vertices and retained chord geometry are unchanged. Selection
/// precedes source-circuit construction and depth scheduling.</summary>
internal static class VCarveVisitStockPruner
{
    internal static List<TaggedToolpath> Filter(IReadOnlyList<TaggedToolpath> source,
        VCarveCutterProfile profile, IReadOnlyList<VCarveRoughingChord> roughing,
        Func<Point3D, Point3D> command, double tolerance, CancellationToken token,
        Action<string>? diagnostic = null, ClearedColumnStock? stock = null, double surfaceZ = 0)
    {
        token.ThrowIfCancellationRequested();
        if (!double.IsFinite(tolerance) || tolerance < 0 || tolerance > .05)
            throw new ArgumentOutOfRangeException(nameof(tolerance));
        stock = stock?.CreateQuerySession();
        var output = new List<TaggedToolpath>(source.Count);
        int shallow = 0, cleared = 0, retained = 0, splitVisits = 0, partialAxisRefusals = 0;
        long inputPoints = 0, outputPoints = 0, coveredChords = 0, checkedChords = 0;
        foreach (var path in source)
        {
            token.ThrowIfCancellationRequested();
            inputPoints += path.Points.Count;
            var points = path.Points.Select(command).ToArray();
            bool eligible = points.Length >= 2 && points.All(Finite)
                && path.Spans.All(span => span.Kind == ToolpathSpanKind.Cut);
            // An omitted-XY command cannot establish the nominal planar cursor.
            // Such a visit retains its original motion and entry custody.
            if (eligible)
            for (int i = 1; i < path.Points.Count; i++)
            {
                var a = path.Points[i - 1]; var b = path.Points[i];
                if (ToolpathCommandRounding.IsVerticalOnly((float)b.X - (float)a.X,
                    (float)(surfaceZ + b.Z) - (float)(surfaceZ + a.Z), (float)b.Y - (float)a.Y))
                { eligible = false; partialAxisRefusals++; break; }
            }
            if (!eligible) { Keep(path); continue; }
            if (points.All(p => -p.Z <= tolerance)) { shallow++; continue; }
            if (roughing.Count == 0 && stock is null) { Keep(path); continue; }
            stock ??= new(roughing.Select(c => new ClearedColumnStock.Column(c.A, c.B, c.ClearedColumnRadius)), token);
            var runs = new List<List<Point3D>>();
            List<Point3D>? run = null;
            int covered = 0;
            for (int i = 1; i < points.Length; i++)
            {
                if ((i & 1023) == 0) token.ThrowIfCancellationRequested();
                checkedChords++;
                if (stock.Contains(points[i - 1], points[i], profile.PhysicalRadiusAtDepth, tolerance))
                {
                    covered++; coveredChords++;
                    if (run is not null) { runs.Add(run); run = null; }
                }
                else
                {
                    run ??= new() { path.Points[i - 1] };
                    run.Add(path.Points[i]);
                }
            }
            if (run is not null) runs.Add(run);
            if (covered == 0) { Keep(path); continue; }
            if (runs.Count == 0) { cleared++; continue; }
            splitVisits++;
            if (path.Points[0] == path.Points[^1] && runs.Count > 1 && runs[^1][^1] == runs[0][0])
            {
                var wrapped = runs[^1]; wrapped.AddRange(runs[0].Skip(1));
                runs[0] = wrapped; runs.RemoveAt(runs.Count - 1);
            }
            foreach (var arc in runs)
                Keep(path with { Points = arc, Spans = Array.Empty<ToolpathSpan>() });
        }
        diagnostic?.Invoke($"stage=visit-stock-pruning event=complete sourceVisits={source.Count} retainedVisits={retained} "
            + $"removedVisits={shallow + cleared} stockSurfaceVisits={shallow} priorRoughingVisits={cleared} splitVisits={splitVisits} "
            + $"checkedChords={checkedChords} coveredChords={coveredChords} removedPoints={inputPoints - outputPoints} "
            + $"tolerance={tolerance:R} retainedRealPoses={stock?.Poses ?? 0} queries={stock?.Queries ?? 0} "
            + $"floorUnionBuilds={stock?.FloorUnionBuilds ?? 0} floorUnionChords={stock?.FloorUnionChords ?? 0} "
            + $"floorUnionAccepted={stock?.FloorUnionAccepted ?? 0} floorUnionMs={stock?.FloorUnionMilliseconds ?? 0:F3} "
            + $"partialAxisRefusals={partialAxisRefusals} poseBudgetReached={stock?.PoseBudgetReached ?? false} "
            + $"queryBudgetReached={stock?.QueryBudgetReached ?? false} "
            + "wholeCommandedCutterVolume=True equalPostedFloorIsCleared=True existingVerticesOnly=True "
            + "missingEvidenceRetains=True beforeSourceCircuits=True beforeSurfaceOriginStepdown=True");
        return output;

        void Keep(TaggedToolpath path)
        { output.Add(path); retained++; outputPoints += path.Points.Count; }
    }

    private static bool Finite(Point3D p) => double.IsFinite(p.X + p.Y + p.Z)
        && Math.Abs(p.X) <= 10000 && Math.Abs(p.Y) <= 10000 && Math.Abs(p.Z) <= 100;
}
