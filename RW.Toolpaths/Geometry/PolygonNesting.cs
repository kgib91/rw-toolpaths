using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

/// <summary>
/// Polygon containment/nesting analysis. Determines which rings are outer boundaries and
/// which are holes or islands, so the milling strategies know what is material and what is air.
/// </summary>
public static class PolygonNesting
{
    /// <summary>
    /// Builds a nesting tree from a flat list of rings. Every ring lands at the depth
    /// implied by how many other rings contain it.
    /// </summary>
    public static List<PathTreeNode> BuildTree(
        IEnumerable<IReadOnlyList<PointD>> rings,
        int offset = 0)
    {
        var roots = new List<PathTreeNode>();
        foreach (var ring in rings)
        {
            if (ring.Count < 3)
                continue;
            Insert(roots, new PathTreeNode(new List<PointD>(ring), offset));
        }
        return roots;
    }

    /// <summary>
    /// Inserts <paramref name="node"/> at the correct nesting level.
    /// Returns the parent that accepted it, or <c>null</c> when it became a root.
    /// </summary>
    /// <remarks>
    /// The single-pass order matters: after absorbing some roots as children, the loop can
    /// still discover that the node itself belongs inside a later root, and must then descend.
    /// </remarks>
    public static PathTreeNode? Insert(List<PathTreeNode> roots, PathTreeNode node)
    {
        var intNode = node.GetIntPath();

        for (int i = roots.Count - 1; i >= 0; i--)
        {
            var root = roots[i];

            if (Contains(intNode, root))
            {
                node.Children.Add(root);
                roots.RemoveAt(i);
            }
            else if (Contains(root.GetIntPath(), node))
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

        // Descend to the deepest existing descendant that still contains the node.
        while (true)
        {
            PathTreeNode? containingChild = null;
            foreach (var child in parent.Children)
            {
                if (Contains(child.GetIntPath(), node))
                {
                    containingChild = child;
                    break;
                }
            }

            if (containingChild is null)
                break;

            parent = containingChild;
        }

        // Any sibling that now falls inside the new node becomes its child.
        for (int i = parent.Children.Count - 1; i >= 0; i--)
        {
            var child = parent.Children[i];
            if (Contains(intNode, child))
            {
                node.Children.Add(child);
                parent.Children.RemoveAt(i);
            }
        }

        parent.Children.Add(node);
        return parent;
    }

    /// <summary>
    /// Returns <c>true</c> when <paramref name="inner"/>'s representative vertex lies inside
    /// <paramref name="outerPath"/>. Rings produced by offsetting never cross, so a single
    /// point test is sufficient and far cheaper than a full containment check.
    /// </summary>
    public static bool Contains(Path64 outerPath, PathTreeNode inner)
    {
        if (inner.Points.Count == 0 || outerPath.Count == 0)
            return false;
        return PathUtils.PointInPolygon(inner.GetIntPath()[0], outerPath);
    }

    /// <summary>
    /// Splits a nesting tree into machining groups. Each group is one outer boundary plus the
    /// holes immediately inside it; islands nested within a hole start a new group because they
    /// are solid material again.
    /// </summary>
    public static List<List<List<PointD>>> GroupOuterWithHoles(IEnumerable<PathTreeNode> roots)
    {
        var groups = new List<List<List<PointD>>>();
        var queue = new Queue<PathTreeNode>(roots);

        while (queue.Count > 0)
        {
            var node = queue.Dequeue();
            var group = new List<List<PointD>> { node.Points };

            foreach (var child in node.Children)
            {
                group.Add(child.Points);
                foreach (var grandchild in child.Children)
                    queue.Enqueue(grandchild);
            }

            groups.Add(group);
        }

        return groups;
    }
}
