namespace RW.Toolpaths;

/// <summary>
/// A 3-D point used for toolpath output.
/// </summary>
public readonly record struct Point3D(double X, double Y, double Z);

/// <summary>
/// One node in the polygon nesting tree used by the offset-fill algorithm.
///
/// Mirrors the node objects produced by <c>path-tree</c> (pathTree) and <c>tree-offset</c>
/// <list type="bullet">
///   <item>The polygon ring at this offset level.</item>
///   <item>How many offset steps away from the original boundary it is.</item>
///   <item>Zero or more child nodes that are strictly contained within it.</item>
/// </list>
/// </summary>
public class PathTreeNode
{
    /// <summary>
    /// The polygon ring for this node, in floating-point workspace coordinates.
    /// </summary>
    public List<Clipper2Lib.PointD> Points { get; set; }

    /// <summary>
    /// How many inward-offset steps from the original boundary this ring is.
    /// 0 = original boundary,  1 = first offset ring,  2 = second, ...
    /// </summary>
    public int Offset { get; set; }

    /// <summary>
    /// Rings that are geometrically inside this ring.
    /// </summary>
    public List<PathTreeNode> Children { get; set; }

    private Clipper2Lib.Path64? _cachedIntPath;

    public PathTreeNode(List<Clipper2Lib.PointD> points, int offset)
    {
        Points   = points;
        Offset   = offset;
        Children = new List<PathTreeNode>();
    }

    public Clipper2Lib.Path64 GetIntPath()
    {
        _cachedIntPath ??= PathUtils.ToClipper(new[] { Points })[0];
        return _cachedIntPath;
    }
}


