using Clipper2Lib;

namespace RW.Toolpaths;

/// <summary>
/// Low-level geometry helpers that mirror the small utility functions
/// </summary>
public static class PathUtils
{
    // --- Coordinate scaling ---------------------------------------------------
    //
    // to convert floating-point workspace coordinates (inches or mm) to 64-bit
    // integers for Clipper2.  We keep the same value so numeric results match.

    /// <summary>
    /// Scale factor that converts workspace units (inches / mm) to Clipper2
    /// </summary>
    public const double Scale = 32_768.0;

    // --- Coordinate conversions -----------------------------------------------

    /// <summary>
    /// Converts a list of 2-D floating-point rings to Clipper2 integer paths.
    /// </summary>
    public static Paths64 ToClipper(IEnumerable<IEnumerable<PointD>> floatPaths)
        => new(floatPaths.Select(ring =>
               new Path64(ring.Select(p =>
                   new Point64((long)Math.Round(p.x * Scale),
                                (long)Math.Round(p.y * Scale))))));

    /// <summary>
    /// Converts Clipper2 integer paths back to floating-point rings.
    /// </summary>
    public static List<List<PointD>> FromClipper(Paths64 intPaths)
        => intPaths.Select(ring =>
               ring.Select(p => new PointD(p.X / Scale, p.Y / Scale))
                   .ToList())
           .ToList();

    /// <summary>
    /// Canonicalizes a ring set by performing a polygon union.
    /// This removes overlaps and resolves self-intersections into
    /// non-overlapping simple contours (outer rings and holes).
    /// </summary>
    public static List<List<PointD>> CanonicalizeRings(
        IEnumerable<IEnumerable<PointD>> rings,
        FillRule fillRule = FillRule.NonZero)
    {
        var filtered = rings
            .Select(r => r.ToList())
            .Where(r => r.Count >= 3)
            .ToList();

        var paths = ToClipper(filtered);

        if (paths.Count == 0)
            return new List<List<PointD>>();

        var merged = Clipper.BooleanOp(
            ClipType.Union,
            paths,
            new Paths64(),
            fillRule);

        var canonical = FromClipper(Lighten(merged));
        foreach (var ring in canonical)
        {
            if (ring.Count == 0)
                continue;

            var first = ring[0];
            var last = ring[^1];
            if (Math.Abs(first.x - last.x) > 1e-12 || Math.Abs(first.y - last.y) > 1e-12)
                ring.Add(first);
        }

        return canonical;
    }

    // --- Lighten --------------------------------------------------------------
    //
    // Removes near-collinear vertices to reduce vertex count.  Different
    // pipeline stages use different effective tolerances, so this helper keeps
    // a conservative default and allows call sites to opt into larger values.

    /// <summary>
    /// Removes near-collinear vertices.
    /// </summary>
    /// <param name="paths">Input integer-coordinate paths.</param>
    /// <param name="tolerance">
    /// Simplification tolerance in integer coordinate units.
    /// Default 1.0 preserves prior behavior for non-medial pipelines.
    /// </param>
    public static Paths64 Lighten(Paths64 paths, double tolerance = 1.0)
        => Clipper.SimplifyPaths(paths, tolerance, false);

    // --- Polygon orientation --------------------------------------------------
    //
    //   "climb"        -> counterclockwise (positive area in Y-up space)
    //   "conventional" -> clockwise        (negative area in Y-up space)

    /// <summary>
    /// Ensures a closed ring has the winding order required by
    /// <paramref name="millingDirection"/>.
    /// </summary>
    /// <param name="path">Integer-coordinate ring (Clipper2).</param>
    /// <param name="millingDirection">
    ///   <c>"climb"</c> -> CCW, <c>"conventional"</c> -> CW,
    ///   <c>null</c>/<c>"default"</c> -> unchanged.
    /// </param>
    public static Path64 OrientPath(Path64 path, string? millingDirection)
    {
        if (millingDirection is null or "default")
            return path;

        // In Clipper2 (Y-up math convention): Area > 0 â†” CCW
        bool wantCcw = millingDirection == "climb";
        bool isCcw   = Clipper.Area(path) > 0;

        if (isCcw == wantCcw)
            return path;

        // Reverse to flip winding
        var reversed = new Path64(path);
        reversed.Reverse();
        return reversed;
    }

    // --- RebaseNear -----------------------------------------------------------
    //
    //   Rotates a closed polygon so that it starts at the vertex nearest
    //   `point`.  Produces a smooth entry from the previous toolpath ring.

    /// <summary>
    /// Rotates a closed polygon ring so that it starts at the vertex nearest
    /// <paramref name="target"/>.
    /// </summary>
    public static List<PointD> RebaseNear(
        IList<PointD> closedRing,
        PointD target,
        double tolerance = 0.0)
    {
        if (closedRing.Count == 0)
            return new List<PointD>(closedRing);

        // a closed ring from offset paths.
        if (tolerance < 0)
            throw new ArgumentOutOfRangeException(nameof(tolerance),
                "RebaseNear tolerance must be >= 0.");

        // tolerance of target.
        double firstDx = closedRing[0].x - target.x;
        double firstDy = closedRing[0].y - target.y;
        double bestDist = Math.Sqrt(firstDx * firstDx + firstDy * firstDy);
        if (bestDist <= tolerance)
            return new List<PointD>(closedRing);

        int bestIdx = 0;

        for (int i = 1; i < closedRing.Count - 1; i++)
        {
            double dx = closedRing[i].x - target.x;
            double dy = closedRing[i].y - target.y;
            double d  = Math.Sqrt(dx * dx + dy * dy);
            if (d < bestDist)
            {
                bestDist = d;
                bestIdx  = i;
                if (d <= tolerance)
                    break;
            }
        }

        if (bestIdx == 0)
            return new List<PointD>(closedRing);

        //   bt = n.slice(bestIdx)
        //   mt = n.slice(1, bestIdx+1)
        //   return [...bt, ...mt]
        var result = new List<PointD>(closedRing.Count);
        result.AddRange(closedRing.Skip(bestIdx));
        result.AddRange(closedRing.Skip(1).Take(bestIdx));
        return result;
    }

    // --- Nearest-neighbour sort -----------------------------------------------
    //
    //   In-place reordering that preserves the first path, then for each
    //   position t picks the closest remaining start point for slot t+1.

    /// <summary>
    /// Re-orders toolpath segments in-place using a greedy nearest-neighbour
    /// heuristic (start of next path closest to end of previous path).
    /// </summary>
    public static void NearestNeighborSort(List<List<Point3D>> paths)
    {
        if (paths.Count <= 1) return;

        // for (t=0; t<n.length-1; t++) {
        //   e = end(paths[t]);
        //   i = paths[t+1];
        //   o = dist2(e, start(i));
        //   for (l=t+2; l<n.length; l++) ... if (h<o) swap(i, paths[l])
        // }
        for (int t = 0; t < paths.Count - 1; t++)
        {
            if (paths[t].Count == 0)
                continue;

            var end = paths[t][^1];

            int bestIndex = t + 1;
            double bestDist = StartDistSq(paths[bestIndex], end);

            for (int l = t + 2; l < paths.Count; l++)
            {
                double d = StartDistSq(paths[l], end);
                if (d < bestDist)
                {
                    bestDist = d;
                    bestIndex = l;
                }
            }

            if (bestIndex != t + 1)
            {
                (paths[t + 1], paths[bestIndex]) = (paths[bestIndex], paths[t + 1]);
            }
        }
    }

    private static double StartDistSq(List<Point3D> path, Point3D from)
    {
        if (path.Count == 0)
            return double.PositiveInfinity;

        var start = path[0];
        var dx = from.X - start.X;
        var dy = from.Y - start.Y;
        return dx * dx + dy * dy;
    }

    // --- Point-in-polygon -----------------------------------------------------
    //
    // Wrapper used by path-tree construction (mirrors Or.pointInPolygon / contains-node).

    /// <summary>
    /// Returns <c>true</c> if <paramref name="pt"/> is inside or on
    /// <paramref name="polygon"/>.
    /// </summary>
    public static bool PointInPolygon(Point64 pt, Path64 polygon)
    {
        var result = Clipper.PointInPolygon(pt, polygon);
        return result is PointInPolygonResult.IsInside or PointInPolygonResult.IsOn;
    }
}


