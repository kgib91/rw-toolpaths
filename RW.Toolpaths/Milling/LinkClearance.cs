using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>
/// Decides whether the cutter may step straight from one ring to the next at cutting depth
/// instead of lifting clear and re-entering.
/// </summary>
/// <remarks>
/// The built-in policies are two-dimensional and deliberately conservative: a link is approved
/// only when the cutter provably stays inside the area this operation is allowed to remove, so
/// none of them can approve a gouge. What they cannot know is that an earlier operation already
/// cleared the material standing in the way, which is why the cut depth is carried through — a
/// host that maintains a stock model implements this interface to win those links back.
/// </remarks>
public interface ILinkClearance
{
    /// <summary>
    /// True when a straight move at cutting depth from <paramref name="from"/> to
    /// <paramref name="to"/> removes only material this operation is allowed to remove.
    /// </summary>
    bool IsTravelSafe(Point3D from, Point3D to, double toolRadius);
}

/// <summary>Built-in <see cref="ILinkClearance"/> policies.</summary>
public static class LinkClearance
{
    /// <summary>
    /// Stray length tolerated before a link counts as having left the travel region, in workspace
    /// units. Rings and the travel region are cut from the same offset call, so a legitimate link
    /// does not stray at all and this only has to absorb integer quantisation.
    /// </summary>
    public const double DefaultTolerance = 4.0 / PathUtils.Scale;

    /// <summary>
    /// How far the travel region is grown before testing, in mm. The outermost ring is cut from
    /// the region's own boundary, so its links run along it; without this they would read as
    /// leaving. A micron is far below anything that counts as a gouge.
    /// </summary>
    private const double BoundaryEpsilon = 0.001;

    /// <summary>Approves every link; for callers that have already proven travel is safe.</summary>
    public static ILinkClearance Unrestricted { get; } = new UnrestrictedClearance();

    /// <summary>Approves only rings that already meet, so every real gap becomes a lift.</summary>
    public static ILinkClearance AlwaysLift { get; } = new AlwaysLiftClearance();

    /// <summary>
    /// Approves links that stay inside <paramref name="travelRegion"/>: the area the cutter
    /// centre may occupy without cutting outside the operation's remit. For pocketing that is the
    /// region eroded by cutter radius plus stock to leave, so the holes of
    /// <paramref name="travelRegion"/> are exactly the islands that have to survive.
    /// </summary>
    /// <param name="travelRegion">Rings in Clipper winding: outers counter-clockwise, holes clockwise.</param>
    public static ILinkClearance WithinRegion(
        IEnumerable<IReadOnlyList<PointD>> travelRegion,
        double tolerance = DefaultTolerance)
    {
        var paths = new Paths64();
        foreach (var ring in travelRegion)
        {
            if (ring.Count < 3)
                continue;

            var path = new Path64(ring.Count);
            foreach (var point in ring)
            {
                path.Add(new Point64(
                    (long)Math.Round(point.x * PathUtils.Scale),
                    (long)Math.Round(point.y * PathUtils.Scale)));
            }
            paths.Add(path);
        }

        if (paths.Count == 0)
            return AlwaysLift;

        var grown = Clipper.InflatePaths(
            paths,
            BoundaryEpsilon * PathUtils.Scale,
            JoinType.Miter,
            EndType.Polygon,
            miterLimit: 2.0);

        return new RegionClearance(
            grown.Count > 0 ? grown : paths,
            Math.Max(tolerance, DefaultTolerance));
    }

    private static double PlanarDistance(Point3D from, Point3D to)
    {
        double dx = to.X - from.X;
        double dy = to.Y - from.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    private sealed class UnrestrictedClearance : ILinkClearance
    {
        public bool IsTravelSafe(Point3D from, Point3D to, double toolRadius) => true;
    }

    private sealed class AlwaysLiftClearance : ILinkClearance
    {
        public bool IsTravelSafe(Point3D from, Point3D to, double toolRadius)
            => PlanarDistance(from, to) <= DefaultTolerance;
    }

    private sealed class RegionClearance : ILinkClearance
    {
        private readonly Paths64 _travelRegion;
        private readonly double _tolerance;

        internal RegionClearance(Paths64 travelRegion, double tolerance)
        {
            _travelRegion = travelRegion;
            _tolerance = tolerance;
        }

        public bool IsTravelSafe(Point3D from, Point3D to, double toolRadius)
        {
            if (PlanarDistance(from, to) <= _tolerance)
                return true;

            var clipper = new Clipper64();
            clipper.AddOpenSubject(new Path64(2)
            {
                new((long)Math.Round(from.X * PathUtils.Scale), (long)Math.Round(from.Y * PathUtils.Scale)),
                new((long)Math.Round(to.X * PathUtils.Scale), (long)Math.Round(to.Y * PathUtils.Scale)),
            });
            clipper.AddClip(_travelRegion);

            // Difference keeps the parts of the link that fall outside the region: any of it is a gouge.
            var enclosed = new Paths64();
            var strayed = new Paths64();
            clipper.Execute(ClipType.Difference, FillRule.NonZero, enclosed, strayed);

            return StrayLength(strayed) <= _tolerance;
        }

        private static double StrayLength(Paths64 paths)
        {
            double total = 0;
            foreach (var path in paths)
            {
                for (int i = 1; i < path.Count; i++)
                {
                    double dx = (path[i].X - path[i - 1].X) / PathUtils.Scale;
                    double dy = (path[i].Y - path[i - 1].Y) / PathUtils.Scale;
                    total += Math.Sqrt(dx * dx + dy * dy);
                }
            }
            return total;
        }
    }
}
