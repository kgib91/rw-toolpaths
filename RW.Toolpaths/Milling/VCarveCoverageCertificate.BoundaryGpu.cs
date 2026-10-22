using Clipper2Lib;
using RW.Toolpaths.Milling.Acceleration;

namespace RW.Toolpaths.Milling;

internal static partial class VCarveCoverageCertificate
{
    private sealed partial class Tree
    {
        // Export precisely the original target edges in this tree's immutable
        // order. The provider independently validates the entire BVH snapshot.
        // Integer queries and device distances only choose a candidate edge.
        internal bool TryCreateBoundaryGpu(Action<string>? diagnostic,
            out VCarveGpuDiscWitnessContext.BoundarySampleContext? context, out string reason)
        {
            context = null;
            reason = "boundary-grid-domain";
            var segments = new GpuBoundarySegment[_segments.Length];
            for (int i = 0; i < segments.Length; i++)
            {
                _work.Check();
                Segment s = _segments[i];
                if (!Exact(s.A.X, out long ax) || !Exact(s.A.Y, out long ay)
                    || !Exact(s.B.X, out long bx) || !Exact(s.B.Y, out long by)) return false;
                segments[i] = new(ax, ay, bx, by);
            }
            var nodes = new GpuBoundaryNode[_nodes.Count];
            for (int i = 0; i < nodes.Length; i++)
            {
                _work.Check();
                Node n = _nodes[i];
                bool leaf = n.Left < 0;
                nodes[i] = new((long)Math.Floor(n.Bounds.X0 * PathUtils.Scale),
                    (long)Math.Floor(n.Bounds.Y0 * PathUtils.Scale),
                    (long)Math.Ceiling(n.Bounds.X1 * PathUtils.Scale),
                    (long)Math.Ceiling(n.Bounds.Y1 * PathUtils.Scale),
                    leaf ? n.First : 0, leaf ? n.Count : 0, n.Left, n.Right);
            }
            return VCarveGpuDiscWitnessContext.BoundarySampleContext.TryCreate(segments, nodes,
                _work.Cancellation, diagnostic, out context, out reason);

            static bool Exact(double coordinate, out long integer)
            {
                integer = 0;
                if (!double.IsFinite(coordinate) || Math.Abs(coordinate) >= 16_384) return false;
                integer = (long)Math.Round(coordinate * PathUtils.Scale);
                return Math.Abs(integer) < VCarveGpuDiscWitnessContext.BoundarySampleContext.ExclusiveCoordinateLimit
                    && BitConverter.DoubleToInt64Bits(integer / PathUtils.Scale) == BitConverter.DoubleToInt64Bits(coordinate);
            }
        }

        internal bool IsOriginalBoundaryNomination(GpuBoundarySampleNomination nomination)
            => nomination.HasNomination && (uint)nomination.SegmentOrdinal < (uint)_segments.Length
                && double.IsFinite(nomination.Parameter) && nomination.Parameter >= 0 && nomination.Parameter <= 1
                && double.IsFinite(nomination.SquaredDistance) && nomination.SquaredDistance >= 0;

        internal bool TryBoundaryGpuTriangleWitness(DyadicTriangle task, GpuTriangleQuery query,
            long radius, GpuBoundarySampleNomination nomination, out int big)
        {
            big = 0;
            return IsOriginalBoundaryNomination(nomination)
                && Capsule(nomination.SegmentOrdinal, task, query, radius, out big);
        }
    }
}
