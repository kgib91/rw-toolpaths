using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths;

/// <summary>Compatibility entry points for the shared concentric offset collector.</summary>
public static class OffsetFill
{

    /// <summary>Collects offset centerlines using the shared ring engine, preserving partial terminal inset.</summary>
    public static List<List<Point3D>> Generate(
        IEnumerable<IEnumerable<PointD>> boundary,
        double depth,
        double zTop,
        double stepOver,
        double?  rampingAngle     = null,
        string?  millingDirection = null,
        int      maxIterations    = 1_000,
        double maxInset = double.PositiveInfinity,
        CancellationToken cancellationToken = default,
        Action<int, double>? progress = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        ValidateInset(maxInset);
        if (!double.IsFinite(stepOver) || stepOver <= 0)
            throw new ArgumentOutOfRangeException(nameof(stepOver));
        long t0 = PerfLog.Start();
        if (depth > zTop)
            throw new ArgumentException(
                "depth must be <= zTop (deeper is more negative, e.g. depth=-0.1, zTop=0)");

        var rings = Collect(boundary, -stepOver, maxIterations, maxInset, cancellationToken, progress);
        if (rings.Count == 0)
            return new List<List<Point3D>>();

        MillingDirection direction = millingDirection switch
        {
            "climb" => MillingDirection.Climb,
            "conventional" => MillingDirection.Conventional,
            _ => MillingDirection.Default,
        };
        var result = new List<List<Point3D>>();
        if ((rampingAngle is null or 0) && direction == MillingDirection.Default)
        {
            foreach (OffsetRing ring in rings.AsEnumerable().Reverse())
                result.Add(Materialize(ring, depth));
            PathUtils.NearestNeighborSort(result);
        }
        else
        {
            RingOffsetEngine.BuildNesting(rings);
            var traversal = RingOffsetEngine.BuildTraversal(rings, insideOut: true);
            foreach (RingChain chain in RingOffsetEngine.BuildChains(rings, traversal.Order))
            {
                cancellationToken.ThrowIfCancellationRequested();
                var points = RingOffsetEngine.MaterializeChain(rings, chain, direction)
                    .Select(point => new Point3D(point.x, point.y, depth)).ToList();
                if (points.Count < 2) continue;
                if (rampingAngle > 0)
                    points = RampPlanner.PlanPathEntry(points, zTop, new RampSettings(rampingAngle.Value),
                        cancellationToken, detail => Console.WriteLine($"[OffsetFill.Entry] {detail}")).Points;
                result.Add(points);
            }
        }
        PerfLog.Stop("OffsetFill.Generate", t0,
            $"rings={rings.Count} open={rings.Count(ring => !ring.IsClosed)} output={result.Count} " +
            $"direction={direction} ramp={rampingAngle ?? 0.0:R} sharedConcentricGeometry=True");
        return result;
    }

    private static List<Point3D> Materialize(OffsetRing ring, double depth)
    {
        var points = ring.Points.Select(point => new Point3D(point.x, point.y, depth)).ToList();
        if (ring.IsClosed && points.Count > 1 && points[0] != points[^1])
            points.Add(points[0]);
        return points;
    }

    private static List<OffsetRing> Collect(
        IEnumerable<IEnumerable<PointD>> paths,
        double delta,
        int maxIterations,
        double maxInset,
        CancellationToken cancellationToken,
        Action<int, double>? progress)
    {
        cancellationToken.ThrowIfCancellationRequested();
        ValidateInset(maxInset);
        if (!double.IsFinite(delta)) throw new ArgumentOutOfRangeException(nameof(delta));
        var boundary = PathUtils.CanonicalizeRings(paths);
        var regions = PolygonOps.DifferenceToComponents(boundary, Array.Empty<IReadOnlyList<PointD>>())
            .Select(component => new MillingRegion(component.Outer,
                component.Holes.Select(hole => (IReadOnlyList<PointD>)hole).ToList())).ToList();
        var options = new RingOffsetOptions(0, Math.Abs(delta))
        {
            RingLimit = (int)Math.Clamp((long)maxIterations + 1, 1, RingOffsetEngine.AbsoluteRingLimit),
            OffsetSign = delta <= 0 ? -1 : 1,
            MaximumInset = maxInset,
            MinimumRingArea = 0,
            Tolerances = GeometryTolerances.Default with { SimplifyTolerance = 0 },
            LevelProgress = progress,
        };
        return MedialAxisGeometry.BuildConcentricFill(regions, options, 0,
            preserveTerminalPaths: false,
            clipBoundary: null, cancellationToken);
    }


    /// <summary>Returns additional offset levels using the same source-based collector as milling.</summary>
    public static List<List<List<PointD>>> OffsetArray(
        IEnumerable<IEnumerable<PointD>> paths,
        double delta,
        int maxIterations = 1_000,
        double maxInset = double.PositiveInfinity,
        CancellationToken cancellationToken = default,
        Action<int, double>? progress = null)
    {
        return Collect(paths, delta, maxIterations, maxInset, cancellationToken, progress)
            .Where(ring => ring.Level > 0)
            .GroupBy(ring => ring.Level).OrderBy(group => group.Key)
            .Select(group => group.Select(ring => Materialize(ring, 0)
                .Select(point => new PointD(point.X, point.Y)).ToList()).ToList()).ToList();
    }


    /// <summary>Builds polygon containment for supplied boundaries without generating offsets.</summary>
    public static List<PathTreeNode> BuildPathTree(
        IEnumerable<IEnumerable<PointD>> paths,
        int offset = 0)
    {
        var roots = new List<PathTreeNode>();
        foreach (var ring in paths)
        {
            var node = new PathTreeNode(ring.ToList(), offset);
            InsertIntoTree(roots, node);
        }
        return roots;
    }

    /// <summary>Projects the shared offset nesting tree into the compatibility node representation.</summary>
    public static List<PathTreeNode> BuildOffsetTree(
        IEnumerable<IEnumerable<PointD>> paths,
        double delta,
        int maxIterations = 1_000,
        double maxInset = double.PositiveInfinity,
        CancellationToken cancellationToken = default,
        Action<int, double>? progress = null)
    {
        var rings = Collect(paths, delta, maxIterations, maxInset, cancellationToken, progress);
        RingOffsetEngine.BuildNesting(rings);
        var nodes = rings.Select(ring => new PathTreeNode(
            Materialize(ring, 0).Select(point => new PointD(point.X, point.Y)).ToList(),
            ring.Level, ring.IsClosed)).ToList();
        var roots = new List<PathTreeNode>();
        for (int index = 0; index < rings.Count; index++)
        {
            int parent = rings[index].Parent;
            if (parent < 0) roots.Add(nodes[index]);
            else nodes[parent].Children.Add(nodes[index]);
        }
        return roots;
    }


    private static PathTreeNode? InsertIntoTree(
        List<PathTreeNode> roots,
        PathTreeNode node)
    {
        var intNode = node.GetIntPath();

        for (int i = roots.Count - 1; i >= 0; i--)
        {
            var root    = roots[i];
            var intRoot = root.GetIntPath();

            if (IsContainedIn(root, intNode))
            {
                node.Children.Add(root);                // t.children.push(n[e])
                roots.RemoveAt(i);                      // n.splice(e, 1)
            }
            else if (IsContainedIn(node, intRoot))
            {
                return InsertUnder(root, node);
            }
        }

        roots.Add(node);
        return null;
    }
    private static PathTreeNode InsertUnder(PathTreeNode parent, PathTreeNode node)
    {
        var intNode = node.GetIntPath();

        while (true)
        {
            PathTreeNode? containingChild = null;
            foreach (var child in parent.Children)
            {
                var intChild = child.GetIntPath();
                if (IsContainedIn(node, intChild))
                {
                    containingChild = child;
                    break;
                }
            }

            if (containingChild is null)
                break;

            parent = containingChild;
        }

        for (int i = parent.Children.Count - 1; i >= 0; i--)
        {
            var child    = parent.Children[i];
            var intChild = child.GetIntPath();
            if (IsContainedIn(child, intNode))
            {
                node.Children.Add(child);
                parent.Children.RemoveAt(i);
            }
        }

        parent.Children.Add(node);
        return parent;
    }

    private static bool IsContainedIn(PathTreeNode inner, Path64 outerPath)
    {
        if (inner.Points.Count == 0 || outerPath.Count == 0) return false;
        var p = inner.GetIntPath()[0];
        return PathUtils.PointInPolygon(p, outerPath);
    }


    private static void ValidateInset(double maxInset)
    {
        if (double.IsNaN(maxInset) || maxInset < 0)
            throw new ArgumentOutOfRangeException(nameof(maxInset));
    }

}
