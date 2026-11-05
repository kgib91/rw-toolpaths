using Clipper2Lib;
using System.Diagnostics;

namespace RW.Toolpaths.Geometry;

/// <summary>
/// Clipper2-backed polygon algebra shared by every milling strategy.
///
/// Conventions applied uniformly here:
/// <list type="bullet">
///   <item>Integer scaling uses <see cref="PathUtils.Scale"/> so all stages agree on precision.</item>
///   <item>Offsets use round joins, matching the shape a round cutter actually sweeps.</item>
///   <item>Clipper2 parameters are set explicitly rather than inherited from library defaults.</item>
///   <item>Offset results are RDP-simplified; raw round-join output is far too dense to machine.</item>
/// </list>
/// </summary>
public static partial class PolygonOps
{
    private static long _integerUnionPrecisionFallbacks;
    private static readonly FilledFrontEvidenceCache.Cache<CenterlineBufferEvidence> CenterlineBufferCache = new(16L << 20, 64);
    private static readonly FilledFrontEvidenceCache.Cache<CenterlineBufferEvidence> FilledDilationCache = new(128L << 20, 512);

    private sealed class CenterlineBufferEvidence
    {
        private readonly PointD[][] _paths;
        internal long Bytes => 64L + _paths.Sum(path => 32L + 16L * path.Length);
        internal CenterlineBufferEvidence(IReadOnlyList<List<PointD>> paths)
            => _paths = paths.Select(path => path.ToArray()).ToArray();
        internal List<List<PointD>> Copy() => _paths.Select(path => path.ToList()).ToList();
    }

    internal static FilledFrontEvidenceCache.State InspectCenterlineEvidenceCache() => CenterlineBufferCache.Inspect();
    internal static void ClearCenterlineEvidenceCache() => CenterlineBufferCache.Clear();

    /// <summary>Reuses only an identical ordered joined-stroke construction.
    /// The public buffer and its integer union remain the reference computation.</summary>
    internal static List<List<PointD>> BufferCenterlinesExact(
        IReadOnlyList<IReadOnlyList<PointD>> centrelines, double radius,
        double arcTolerance, CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        long started = Stopwatch.GetTimestamp();
        bool enabled = FilledFrontEvidenceCache.Enabled;
        var key = enabled ? FilledFrontEvidenceCache.Key.Capture(centrelines,
            radius, arcTolerance, 0, 11, null, cancellationToken) : null;
        var reused = enabled ? CenterlineBufferCache.Find(key, cancellationToken) : null;
        if (reused is not null)
        {
            var copy = reused.Copy();
            cancellationToken.ThrowIfCancellationRequested();
            Console.WriteLine($"[PolygonOps.CenterlineEvidence] hit=True effectHash={key!.Hash:x16} "
                + $"inputPoints={key.Points} outputPoints={copy.Sum(path => path.Count)} "
                + $"elapsedMs={ElapsedMilliseconds(started):F2} construction=unchanged-joined-order resultOwnership=fresh-copy");
            return copy;
        }
        var output = BufferCenterlines(centrelines, radius, arcTolerance);
        cancellationToken.ThrowIfCancellationRequested();
        if (enabled)
        {
            var evidence = new CenterlineBufferEvidence(output);
            CenterlineBufferCache.Publish(key, evidence, evidence.Bytes, cancellationToken);
        }
        Console.WriteLine($"[PolygonOps.CenterlineEvidence] hit=False effectHash={key?.Hash:x16} "
            + $"inputPoints={centrelines.Sum(path => path.Count)} outputPoints={output.Sum(path => path.Count)} "
            + $"elapsedMs={ElapsedMilliseconds(started):F2} cacheEnabled={enabled} construction=unchanged-joined-order");
        return output;
    }
    private readonly record struct BoundarySegment(
        Point64 Start,
        Point64 End,
        long MinX,
        long MinY,
        long MaxX,
        long MaxY,
        int Ordinal = -1);

    internal sealed class BoundarySegmentIndex
    {
        private const int LeafSize = 8;

        private sealed class Node
        {
            internal long MinX { get; init; }
            internal long MinY { get; init; }
            internal long MaxX { get; init; }
            internal long MaxY { get; init; }
            internal int Start { get; init; }
            internal int Count { get; init; }
            internal Node? First { get; init; }
            internal Node? Second { get; init; }
        }

        private readonly BoundarySegment[] _segments;
        private readonly Node? _root;
        private readonly bool _legacyWitnessOrder;
        private readonly Lazy<BoundarySegmentIndex>? _legacyWitnessIndex;
        private int _partitionCount;
        private int _partitionSortFallbacks;

        internal sealed class ClearanceCache
        {
            internal double KnownClearance;
            internal double EmptyAtRadius = double.PositiveInfinity;
        }

        internal BoundarySegmentIndex(Paths64 boundaries)
        {
            var segments = new List<BoundarySegment>();
            foreach (Path64 boundary in boundaries)
            {
                for (int endIndex = 0, startIndex = boundary.Count - 1;
                     endIndex < boundary.Count;
                     startIndex = endIndex++)
                {
                    Point64 start = boundary[startIndex];
                    Point64 end = boundary[endIndex];
                    segments.Add(new BoundarySegment(
                        start,
                        end,
                        Math.Min(start.X, end.X),
                        Math.Min(start.Y, end.Y),
                        Math.Max(start.X, end.X),
                        Math.Max(start.Y, end.Y),
                        segments.Count));
                }
            }

            _segments = segments.ToArray();
            _root = _segments.Length == 0 ? null : BuildNode(0, _segments.Length);
            _legacyWitnessIndex = new Lazy<BoundarySegmentIndex>(BuildLegacyWitnessIndex,
                LazyThreadSafetyMode.ExecutionAndPublication);
        }

        private BoundarySegmentIndex(BoundarySegment[] segments)
        {
            _legacyWitnessOrder = true;
            _segments = segments;
            _root = segments.Length == 0 ? null : BuildNode(0, segments.Length);
        }

        internal int SegmentCount => _segments.Length;

        // Bounded nomination only. Original ordinals survive spatial partitioning;
        // callers still prove intersections against their original XYZ segments.
        internal bool QueryBounds(long minX,long minY,long maxX,long maxY,List<int> matches,
            int limit,CancellationToken token)
        {
            matches.Clear();int visited=0;bool complete=true;
            Find(_root);return complete;
            void Find(Node? node)
            {
                if(node is null||!complete||node.MaxX<minX||node.MinX>maxX||node.MaxY<minY||node.MinY>maxY)return;
                if((++visited&255)==0)token.ThrowIfCancellationRequested();
                if(node.Count==0){Find(node.First);Find(node.Second);return;}
                for(int i=node.Start;i<node.Start+node.Count;i++)
                {
                    var s=_segments[i];
                    if(s.MaxX<minX||s.MinX>maxX||s.MaxY<minY||s.MinY>maxY)continue;
                    if(matches.Count>=limit){complete=false;return;}
                    matches.Add(s.Ordinal);
                }
            }
        }
        internal int PartitionCount => _partitionCount;
        internal int PartitionSortFallbacks => _partitionSortFallbacks;
        internal bool LegacyWitnessIndexBuilt => _legacyWitnessIndex?.IsValueCreated == true;

        private BoundarySegmentIndex LegacyWitnessIndex()
            => _legacyWitnessIndex!.Value;

        private BoundarySegmentIndex BuildLegacyWitnessIndex()
        {
            // Geometry-only queries do not depend on traversal order. Clearance
            // repairs and nearest-edge ties do: recreate the exact former tree
            // only when a caller needs a legacy witness, from original ordinals.
            var original = new BoundarySegment[_segments.Length];
            foreach (var segment in _segments) original[segment.Ordinal] = segment;
            return new BoundarySegmentIndex(original);
        }

        // A vertex proof does not prove the connecting chord: it can cross an island
        // or shave a round corner. Return the actual source edge that violates the
        // complete commanded chord so offset repair remains falsifiable.
        internal (Point64 Start, Point64 End)? ClearanceViolation(
            Point64 start, Point64 end, double radius)
        {
            if (!_legacyWitnessOrder) return LegacyWitnessIndex().ClearanceViolation(start, end, radius);
            if (_root == null) return null;
            var query = new BoundarySegment(start, end, Math.Min(start.X, end.X), Math.Min(start.Y, end.Y),
                Math.Max(start.X, end.X), Math.Max(start.Y, end.Y));
            double requiredSquared = radius * radius;
            (Point64 Start, Point64 End)? Visit(Node node)
            {
                if (node.MaxX < query.MinX - radius || node.MinX > query.MaxX + radius
                    || node.MaxY < query.MinY - radius || node.MinY > query.MaxY + radius) return null;
                if (node.Count == 0) return Visit(node.First!) ?? Visit(node.Second!);
                for (int i = node.Start; i < node.Start + node.Count; i++)
                {
                    var edge = _segments[i];
                    if (edge.MaxX < query.MinX - radius || edge.MinX > query.MaxX + radius
                        || edge.MaxY < query.MinY - radius || edge.MinY > query.MaxY + radius) continue;
                    Int128 a = Cross(end.X - start.X, end.Y - start.Y, edge.Start.X - start.X, edge.Start.Y - start.Y);
                    Int128 b = Cross(end.X - start.X, end.Y - start.Y, edge.End.X - start.X, edge.End.Y - start.Y);
                    Int128 c = Cross(edge.End.X - edge.Start.X, edge.End.Y - edge.Start.Y, start.X - edge.Start.X, start.Y - edge.Start.Y);
                    Int128 d = Cross(edge.End.X - edge.Start.X, edge.End.Y - edge.Start.Y, end.X - edge.Start.X, end.Y - edge.Start.Y);
                    bool crosses = ((a < 0 && b > 0) || (a > 0 && b < 0))
                        && ((c < 0 && d > 0) || (c > 0 && d < 0));
                    if (crosses || DistanceToSegmentSquared(start, edge) < requiredSquared
                        || DistanceToSegmentSquared(end, edge) < requiredSquared
                        || DistanceToSegmentSquared(edge.Start, query) < requiredSquared
                        || DistanceToSegmentSquared(edge.End, query) < requiredSquared)
                        return (edge.Start, edge.End);
                }
                return null;
            }
            return Visit(_root);
        }

        internal bool IsInside(Point64 point) => _root != null && Winding(_root, point) != 0;

        // A strict filled-region proof cannot choose a touching/intersection
        // policy on Clipper's behalf. Include integer uncertainty around every
        // edge; callers fall back to their original Boolean for these cases.
        internal bool BoundaryMayMeet(Point64 start, Point64 end)
        {
            if (_root is null) return false;
            const double uncertainty = 4;
            var query = new BoundarySegment(start, end, Math.Min(start.X, end.X), Math.Min(start.Y, end.Y),
                Math.Max(start.X, end.X), Math.Max(start.Y, end.Y));
            bool Visit(Node node)
            {
                if (node.MaxX < query.MinX - uncertainty || node.MinX > query.MaxX + uncertainty
                    || node.MaxY < query.MinY - uncertainty || node.MinY > query.MaxY + uncertainty) return false;
                if (node.Count == 0) return Visit(node.First!) || Visit(node.Second!);
                for (int i = node.Start; i < node.Start + node.Count; i++)
                {
                    var edge = _segments[i];
                    if (edge.MaxX < query.MinX - uncertainty || edge.MinX > query.MaxX + uncertainty
                        || edge.MaxY < query.MinY - uncertainty || edge.MinY > query.MaxY + uncertainty) continue;
                    Int128 a = Cross(end.X - start.X, end.Y - start.Y, edge.Start.X - start.X, edge.Start.Y - start.Y);
                    Int128 b = Cross(end.X - start.X, end.Y - start.Y, edge.End.X - start.X, edge.End.Y - start.Y);
                    Int128 c = Cross(edge.End.X - edge.Start.X, edge.End.Y - edge.Start.Y, start.X - edge.Start.X, start.Y - edge.Start.Y);
                    Int128 d = Cross(edge.End.X - edge.Start.X, edge.End.Y - edge.Start.Y, end.X - edge.Start.X, end.Y - edge.Start.Y);
                    if (((a < 0 && b > 0) || (a > 0 && b < 0))
                        && ((c < 0 && d > 0) || (c > 0 && d < 0))) return true;
                    if (DistanceToSegmentSquared(start, edge) <= uncertainty * uncertainty
                        || DistanceToSegmentSquared(end, edge) <= uncertainty * uncertainty
                        || DistanceToSegmentSquared(edge.Start, query) <= uncertainty * uncertainty
                        || DistanceToSegmentSquared(edge.End, query) <= uncertainty * uncertainty) return true;
                }
                return false;
            }
            return Visit(_root);
        }

        internal bool ContainsCapsule(Point64 start, Point64 end, double radius)
        {
            if (_root == null || Winding(_root, start) == 0) return false;
            // Reserve coordinate quantization at both the query and stored boundary.
            radius += 4;
            double radius2 = radius * radius;
            var query = new BoundarySegment(start, end, Math.Min(start.X, end.X), Math.Min(start.Y, end.Y),
                Math.Max(start.X, end.X), Math.Max(start.Y, end.Y));
            bool Clear(Node node)
            {
                if (node.MaxX < query.MinX - radius || node.MinX > query.MaxX + radius
                    || node.MaxY < query.MinY - radius || node.MinY > query.MaxY + radius) return true;
                if (node.Count == 0) return Clear(node.First!) && Clear(node.Second!);
                for (int i = node.Start; i < node.Start + node.Count; i++)
                {
                    var edge = _segments[i];
                    if (edge.MaxX < query.MinX - radius || edge.MinX > query.MaxX + radius
                        || edge.MaxY < query.MinY - radius || edge.MinY > query.MaxY + radius) continue;
                    Int128 a = Cross(end.X - start.X, end.Y - start.Y, edge.Start.X - start.X, edge.Start.Y - start.Y);
                    Int128 b = Cross(end.X - start.X, end.Y - start.Y, edge.End.X - start.X, edge.End.Y - start.Y);
                    Int128 c = Cross(edge.End.X - edge.Start.X, edge.End.Y - edge.Start.Y, start.X - edge.Start.X, start.Y - edge.Start.Y);
                    Int128 d = Cross(edge.End.X - edge.Start.X, edge.End.Y - edge.Start.Y, end.X - edge.Start.X, end.Y - edge.Start.Y);
                    if (((a < 0 && b > 0) || (a > 0 && b < 0)) && ((c < 0 && d > 0) || (c > 0 && d < 0))) return false;
                    if (DistanceToSegmentSquared(start, edge) <= radius2 || DistanceToSegmentSquared(end, edge) <= radius2
                        || DistanceToSegmentSquared(edge.Start, query) <= radius2 || DistanceToSegmentSquared(edge.End, query) <= radius2)
                        return false;
                }
                return true;
            }
            return Clear(_root);
        }

        internal double DistanceSquared(Point64 point)
            => _root is null
                ? double.MaxValue
                : DistanceSquared(_root, point, double.MaxValue);

        internal string DescribeNearestSegment(Point64 point)
        {
            if (_segments.Length == 0) return "none";
            var nearest = NearestSegment(point);
            return $"({nearest.Start.X / PathUtils.Scale:R},{nearest.Start.Y / PathUtils.Scale:R})->"
                + $"({nearest.End.X / PathUtils.Scale:R},{nearest.End.Y / PathUtils.Scale:R})";
        }

        internal (Point64 Start, Point64 End) NearestSegment(Point64 point)
        {
            if (!_legacyWitnessOrder) return LegacyWitnessIndex().NearestSegment(point);
            return NearestSegmentForCertificate(point);
        }

        // An untrusted site choice for a separate exact finite-capsule proof.
        // It may use current BVH ties; existing witness ordering is unchanged.
        internal (Point64 Start, Point64 End) NearestSegmentForCertificate(Point64 point)
        {
            if (_root == null) throw new InvalidOperationException("The source boundary has no segments.");
            int nearest = -1;
            double best = double.MaxValue;
            void Visit(Node node)
            {
                if (DistanceToBoundsSquared(point, node) >= best) return;
                if (node.Count > 0)
                {
                    for (int i = node.Start; i < node.Start + node.Count; i++)
                    {
                        double distance = DistanceToSegmentSquared(point, _segments[i]);
                        if (distance < best) { best = distance; nearest = i; }
                    }
                    return;
                }
                Node first = node.First!, second = node.Second!;
                if (DistanceToBoundsSquared(point, second) < DistanceToBoundsSquared(point, first))
                    (first, second) = (second, first);
                Visit(first);
                Visit(second);
            }
            Visit(_root);
            return (_segments[nearest].Start, _segments[nearest].End);
        }

        // Fractional coordinates only nominate a literal immutable original
        // finite site. Exact capsule predicates separately authorize support.
        // The existing integer witness/nearest ordering above is unchanged.
        internal (Point64 Start, Point64 End) NearestSegmentForCertificate(double x, double y)
        {
            if (_root == null || !double.IsFinite(x) || !double.IsFinite(y))
                throw new InvalidOperationException("The source boundary/query has no finite site choice.");
            int nearest = -1;
            double best = double.MaxValue;
            double Bounds(Node node)
            {
                double dx = x < node.MinX ? node.MinX - x : x > node.MaxX ? x - node.MaxX : 0;
                double dy = y < node.MinY ? node.MinY - y : y > node.MaxY ? y - node.MaxY : 0;
                return dx * dx + dy * dy;
            }
            void Visit(Node node)
            {
                if (Bounds(node) >= best) return;
                if (node.Count > 0)
                {
                    for (int i = node.Start; i < node.Start + node.Count; i++)
                    {
                        var site = _segments[i];
                        double dx = site.End.X - site.Start.X, dy = site.End.Y - site.Start.Y;
                        double wx = x - site.Start.X, wy = y - site.Start.Y;
                        double length = dx * dx + dy * dy;
                        double t = length == 0 ? 0 : Math.Clamp((wx * dx + wy * dy) / length, 0, 1);
                        double rx = wx - t * dx, ry = wy - t * dy;
                        double distance = rx * rx + ry * ry;
                        if (distance < best) { best = distance; nearest = i; }
                    }
                    return;
                }
                Node first = node.First!, second = node.Second!;
                if (Bounds(second) < Bounds(first)) (first, second) = (second, first);
                Visit(first); Visit(second);
            }
            Visit(_root);
            if (nearest < 0) throw new InvalidOperationException("The source boundary has no finite site choice.");
            return (_segments[nearest].Start, _segments[nearest].End);
        }

        internal bool MayContainDisc(double radius, double resolution, ClearanceCache? cache = null)
        {
            // A first/later offset pair shares the immutable hierarchy, but
            // retains the exact independent feasibility-cache history it had
            // before preparation reuse. Other one-shot queries need no cache.
            cache ??= new ClearanceCache();
            if (_root == null || radius >= cache.EmptyAtRadius)
            { InsetFeasibilityDiagnostics.Cache(false); return false; }
            if (radius <= cache.KnownClearance)
            { InsetFeasibilityDiagnostics.Cache(true); return true; }
            long started = Stopwatch.GetTimestamp();
            var pending = new Stack<(double X0, double Y0, double X1, double Y1)>();
            pending.Push((_root.MinX, _root.MinY, _root.MaxX, _root.MaxY));
            int probes = 0;
            bool possible = false;
            while (pending.TryPop(out var box))
            {
                double mx = (box.X0 + box.X1) / 2, my = (box.Y0 + box.Y1) / 2;
                var center = new Point64((long)Math.Round(mx), (long)Math.Round(my));
                double distance = Math.Sqrt(DistanceSquared(center));
                if (Winding(_root, center) == 0) distance = -distance;
                cache.KnownClearance = Math.Max(cache.KnownClearance, distance);
                double hx = (box.X1 - box.X0) / 2, hy = (box.Y1 - box.Y0) / 2;
                double extent = Math.Sqrt(hx * hx + hy * hy);
                probes++;
                // Signed distance is 1-Lipschitz. Include integer rounding of the box center.
                // If this bound is below the radius, no point in the box can fit the cutter.
                if (distance + extent + 1 < radius) continue;
                if (distance >= radius || extent <= Math.Max(2, resolution) || probes >= 8192)
                { possible = true; break; }
                pending.Push((box.X0, box.Y0, mx, my)); pending.Push((mx, box.Y0, box.X1, my));
                pending.Push((box.X0, my, mx, box.Y1)); pending.Push((mx, my, box.X1, box.Y1));
            }
            if (!possible) cache.EmptyAtRadius = Math.Min(cache.EmptyAtRadius, radius);
            InsetFeasibilityDiagnostics.Solve(radius, possible, probes, started);
            return possible;
        }

        internal bool HasInteriorPointWithClearance(double radius, CancellationToken cancellationToken)
        {
            if (_root == null) return false;
            long started = Stopwatch.GetTimestamp();
            // Eligibility is a physical fit proof. Reserve only the coordinate
            // quantization used throughout this index, never machining arc/RDP tolerance.
            double required = Math.Max(0, radius - 4);
            if (_root.MaxX - _root.MinX < 2 * required || _root.MaxY - _root.MinY < 2 * required)
            {
                Console.WriteLine($"[PolygonOps.OpeningFit] radius={radius / PathUtils.Scale:R} "
                    + $"fits=False reason=bounds probes=0");
                return false;
            }
            var pending = new PriorityQueue<(long X0, long Y0, long X1, long Y1), double>();
            int probes = 0;
            double knownClearance = 0;
            bool fits = Probe((_root.MinX, _root.MinY, _root.MaxX, _root.MaxY));
            while (!fits && pending.TryDequeue(out var box, out _))
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (box.X1 - box.X0 >= box.Y1 - box.Y0)
                {
                    long middle = box.X0 + (box.X1 - box.X0) / 2;
                    fits = Probe((box.X0, box.Y0, middle, box.Y1))
                        || Probe((middle + 1, box.Y0, box.X1, box.Y1));
                }
                else
                {
                    long middle = box.Y0 + (box.Y1 - box.Y0) / 2;
                    fits = Probe((box.X0, box.Y0, box.X1, middle))
                        || Probe((box.X0, middle + 1, box.X1, box.Y1));
                }
            }
            Console.WriteLine($"[PolygonOps.OpeningFit] radius={radius / PathUtils.Scale:R} fits={fits} "
                + $"bestClearance={knownClearance / PathUtils.Scale:R} probes={probes} "
                + $"elapsedMs={ElapsedMilliseconds(started):F2}");
            return fits;

            bool Probe((long X0, long Y0, long X1, long Y1) box)
            {
                if (box.X0 > box.X1 || box.Y0 > box.Y1) return false;
                if (probes >= 500_000)
                {
                    Console.WriteLine($"[PolygonOps.OpeningFit] radius={radius / PathUtils.Scale:R} "
                        + $"result=unresolved bestClearance={knownClearance / PathUtils.Scale:R} probes={probes}");
                    throw new InvalidOperationException("Physical cutter fit could not be resolved within the opening-search budget.");
                }
                var center = new Point64(box.X0 + (box.X1 - box.X0) / 2,
                    box.Y0 + (box.Y1 - box.Y0) / 2);
                double distance = Math.Sqrt(DistanceSquared(center));
                bool inside = Winding(_root, center) != 0;
                if (!inside) distance = -distance;
                knownClearance = Math.Max(knownClearance, distance);
                probes++;
                if (inside && distance >= required) return true;
                double hx = Math.Max(center.X - box.X0, box.X1 - center.X);
                double hy = Math.Max(center.Y - box.Y0, box.Y1 - center.Y);
                double upperBound = distance + Math.Sqrt(hx * hx + hy * hy);
                // Signed boundary distance is 1-Lipschitz. A rejected box cannot
                // contain a fit witness, including narrow off-center rooms or holes.
                if (upperBound >= required && (box.X0 != box.X1 || box.Y0 != box.Y1))
                    pending.Enqueue(box, -upperBound);
                return false;
            }
        }

        // Test a line against an already prepared, canonical region. Ambiguous quantized
        // boundary cases return null so the caller can retain Clipper's exact boundary policy.
        internal bool? ContainsSegment(Point64 start, Point64 end, double tolerance)
        {
            if (_root == null) return false;
            var cuts = new List<double> { 0, 1 };
            long dx = end.X - start.X, dy = end.Y - start.Y;
            long minX = Math.Min(start.X, end.X), maxX = Math.Max(start.X, end.X);
            long minY = Math.Min(start.Y, end.Y), maxY = Math.Max(start.Y, end.Y);
            bool ambiguous = false;
            void Intersections(Node node)
            {
                if (ambiguous || node.MaxX < minX || node.MinX > maxX || node.MaxY < minY || node.MinY > maxY) return;
                if (node.Count == 0) { Intersections(node.First!); Intersections(node.Second!); return; }
                for (int i = node.Start; i < node.Start + node.Count; i++)
                {
                    var edge = _segments[i];
                    if (edge.MaxX < minX || edge.MinX > maxX || edge.MaxY < minY || edge.MinY > maxY) continue;
                    long ex = edge.End.X - edge.Start.X, ey = edge.End.Y - edge.Start.Y;
                    long ox = edge.Start.X - start.X, oy = edge.Start.Y - start.Y;
                    Int128 denominator = Cross(dx, dy, ex, ey);
                    Int128 alongEdge = Cross(ox, oy, dx, dy);
                    if (denominator == 0)
                    {
                        if (alongEdge == 0) { ambiguous = true; return; }
                        continue;
                    }
                    Int128 alongLink = Cross(ox, oy, ex, ey);
                    if (denominator < 0) { denominator = -denominator; alongEdge = -alongEdge; alongLink = -alongLink; }
                    if (alongLink >= 0 && alongLink <= denominator && alongEdge >= 0 && alongEdge <= denominator)
                        cuts.Add((double)alongLink / (double)denominator);
                }
            }
            Intersections(_root);
            if (ambiguous) return null;
            cuts.Sort();
            double length = Math.Sqrt((double)dx * dx + (double)dy * dy), stray = 0;
            for (int i = 1; i < cuts.Count; i++)
            {
                double span = cuts[i] - cuts[i - 1];
                if (span <= 0) continue;
                double t = (cuts[i] + cuts[i - 1]) / 2;
                var point = new Point64((long)Math.Round(start.X + t * dx), (long)Math.Round(start.Y + t * dy));
                if (DistanceSquared(point) <= 4) return null;
                if (Winding(_root, point) == 0) stray += span * length;
                if (stray > tolerance + 4) return false;
            }
            if (stray == 0) return true;
            return Math.Abs(stray - tolerance) <= 4 ? null : stray <= tolerance;
        }

        private static Int128 Cross(long ax, long ay, long bx, long by) => (Int128)ax * by - (Int128)ay * bx;

        private static readonly IReadOnlyList<(double Start, double End)> FullContact =
            Array.AsReadOnly(new[] { (0d, 1d) });

        // The dilation of a filled polygon is its interior plus the capsules of
        // its boundary edges. Compute only the intersection with this chord,
        // never the offset polygon's potentially enormous edge arrangement.
        internal IReadOnlyList<(double Start, double End)> ContactIntervals(
            Point64 start, Point64 end, double radius, CancellationToken token,
            out ContactIntervalQueryStatistics statistics)
        {
            statistics = default;
            token.ThrowIfCancellationRequested();
            if (_root is null) return Array.Empty<(double, double)>();
            long dx = end.X - start.X, dy = end.Y - start.Y;
            var midpoint = new Point64(start.X + (long)Math.Round(dx / 2d),
                start.Y + (long)Math.Round(dy / 2d));
            double extent = Math.Max(Math.Sqrt((double)(start.X - midpoint.X) * (start.X - midpoint.X)
                    + (double)(start.Y - midpoint.Y) * (start.Y - midpoint.Y)),
                Math.Sqrt((double)(end.X - midpoint.X) * (end.X - midpoint.X)
                    + (double)(end.Y - midpoint.Y) * (end.Y - midpoint.Y)));
            double boundaryDistance = Math.Sqrt(DistanceSquared(midpoint));
            // A boundary capsule suffices regardless of which signed region
            // contains the midpoint; avoid a winding query in this common case.
            if (boundaryDistance + extent + 1 <= radius)
            { statistics = new(true, 0, true); return FullContact; }
            bool inside = IsInside(midpoint);
            double distance = inside ? 0 : boundaryDistance;
            // Distance to the filled region is 1-Lipschitz. One additional grid
            // unit bounds arithmetic; the caller separately reserves quantization.
            if (distance + extent + 1 <= radius)
            { statistics = new(true, 0); return FullContact; }
            if (distance - extent - 1 > radius)
            { statistics = new(true, 0); return Array.Empty<(double, double)>(); }
            if (dx == 0 && dy == 0)
            { statistics = new(true, 0); return distance <= radius + 1 ? FullContact : Array.Empty<(double, double)>(); }

            double length = Math.Sqrt((double)dx * dx + (double)dy * dy);
            double guardedRadius = radius + 1;
            var contacts = new List<(double Start, double End)>();
            int work = 0;
            int candidateEdges = 0;
            void Add(double first, double last)
            {
                first = Math.Max(0, Math.BitDecrement(first));
                last = Math.Min(1, Math.BitIncrement(last));
                if (first <= last) contacts.Add((first, last));
            }
            void Disc(Point64 center)
            {
                long wx = center.X - start.X, wy = center.Y - start.Y;
                double along = ((double)((Int128)wx * dx + (Int128)wy * dy)) / length;
                double across = Math.Abs((double)Cross(wx, wy, dx, dy)) / length;
                if (across > guardedRadius) return;
                // Projection roots avoid catastrophic cancellation in a raw
                // quadratic discriminant for long, nearly tangent chords.
                double half = Math.Sqrt(Math.Max(0,
                    (guardedRadius - across) * (guardedRadius + across)));
                Add((along - half) / length, (along + half) / length);
            }
            static bool Slab(double origin, double change, double low, double high,
                ref double first, ref double last)
            {
                if (change == 0) return origin >= low && origin <= high;
                double a = (low - origin) / change, b = (high - origin) / change;
                first = Math.Max(first, Math.Min(a, b));
                last = Math.Min(last, Math.Max(a, b));
                return first <= last;
            }
            void Capsule(BoundarySegment edge)
            {
                Disc(edge.Start); Disc(edge.End);
                long ex = edge.End.X - edge.Start.X, ey = edge.End.Y - edge.Start.Y;
                if (ex == 0 && ey == 0) return;
                long wx = start.X - edge.Start.X, wy = start.Y - edge.Start.Y;
                double edgeLength = Math.Sqrt((double)ex * ex + (double)ey * ey);
                double along = (double)((Int128)wx * ex + (Int128)wy * ey) / edgeLength;
                double advance = (double)((Int128)dx * ex + (Int128)dy * ey) / edgeLength;
                double across = (double)Cross(ex, ey, wx, wy) / edgeLength;
                double crossing = (double)Cross(ex, ey, dx, dy) / edgeLength;
                double first = 0, last = 1;
                if (Slab(along, advance, 0, edgeLength, ref first, ref last)
                    && Slab(across, crossing, -guardedRadius, guardedRadius, ref first, ref last))
                    Add(first, last);
            }
            double minX = Math.Min(start.X, end.X) - guardedRadius;
            double maxX = Math.Max(start.X, end.X) + guardedRadius;
            double minY = Math.Min(start.Y, end.Y) - guardedRadius;
            double maxY = Math.Max(start.Y, end.Y) + guardedRadius;
            void Visit(Node node)
            {
                if ((work++ & 255) == 0) token.ThrowIfCancellationRequested();
                if (node.MaxX < minX || node.MinX > maxX || node.MaxY < minY || node.MinY > maxY) return;
                if (node.Count == 0) { Visit(node.First!); Visit(node.Second!); return; }
                for (int i = node.Start; i < node.Start + node.Count; i++)
                {
                    if ((work++ & 255) == 0) token.ThrowIfCancellationRequested();
                    var edge = _segments[i];
                    if (edge.MaxX < minX || edge.MinX > maxX || edge.MaxY < minY || edge.MinY > maxY) continue;
                    candidateEdges++;
                    Capsule(edge);
                }
            }
            Visit(_root);
            statistics = new(false, candidateEdges);
            contacts.Sort((a, b) => a.Start.CompareTo(b.Start));
            var merged = new List<(double Start, double End)>();
            foreach (var contact in contacts)
            {
                if ((work++ & 255) == 0) token.ThrowIfCancellationRequested();
                if (merged.Count == 0 || contact.Start > merged[^1].End) merged.Add(contact);
                else merged[^1] = (merged[^1].Start, Math.Max(merged[^1].End, contact.End));
            }
            // Outside boundary capsules, no boundary crossing exists, so each
            // remaining gap has one constant NonZero winding classification.
            var result = new List<(double Start, double End)>();
            void Gap(double first, double last)
            {
                token.ThrowIfCancellationRequested();
                if (first >= last) return;
                double at = (first + last) / 2;
                var point = new Point64(start.X + (long)Math.Round(dx * at),
                    start.Y + (long)Math.Round(dy * at));
                if (IsInside(point)) result.Add((first, last));
            }
            double position = 0;
            foreach (var contact in merged)
            {
                Gap(position, contact.Start); result.Add(contact); position = contact.End;
            }
            Gap(position, 1);
            if (result.Count < 2) return result;
            merged.Clear();
            foreach (var contact in result)
            {
                if (merged.Count == 0 || contact.Start > merged[^1].End) merged.Add(contact);
                else merged[^1] = (merged[^1].Start, Math.Max(merged[^1].End, contact.End));
            }
            token.ThrowIfCancellationRequested();
            return merged;
        }

        private int Winding(Node node, Point64 point)
        {
            if (node.MaxX < point.X || node.MinY > point.Y || node.MaxY <= point.Y) return 0;
            if (node.Count == 0) return Winding(node.First!, point) + Winding(node.Second!, point);
            int winding = 0;
            for (int i = node.Start; i < node.Start + node.Count; i++)
            {
                var edge = _segments[i];
                if (edge.MinY > point.Y || edge.MaxY <= point.Y || edge.MaxX < point.X) continue;
                Int128 side = Cross(edge.End.X - edge.Start.X, edge.End.Y - edge.Start.Y,
                    point.X - edge.Start.X, point.Y - edge.Start.Y);
                if (edge.Start.Y <= point.Y && edge.End.Y > point.Y && side > 0) winding++;
                else if (edge.Start.Y > point.Y && edge.End.Y <= point.Y && side < 0) winding--;
            }
            return winding;
        }

        private Node BuildNode(int start, int count)
        {
            long minX = long.MaxValue;
            long minY = long.MaxValue;
            long maxX = long.MinValue;
            long maxY = long.MinValue;
            for (int index = start; index < start + count; index++)
            {
                BoundarySegment segment = _segments[index];
                minX = Math.Min(minX, segment.MinX);
                minY = Math.Min(minY, segment.MinY);
                maxX = Math.Max(maxX, segment.MaxX);
                maxY = Math.Max(maxY, segment.MaxY);
            }

            if (count <= LeafSize)
            {
                return new Node
                {
                    MinX = minX,
                    MinY = minY,
                    MaxX = maxX,
                    MaxY = maxY,
                    Start = start,
                    Count = count,
                };
            }

            bool splitX = maxX - minX >= maxY - minY;
            if (_legacyWitnessOrder) Array.Sort(
                _segments,
                start,
                count,
                Comparer<BoundarySegment>.Create((first, second) =>
                {
                    double firstCenter = splitX
                        ? (double)first.MinX + first.MaxX
                        : (double)first.MinY + first.MaxY;
                    double secondCenter = splitX
                        ? (double)second.MinX + second.MaxX
                        : (double)second.MinY + second.MaxY;
                    return firstCenter.CompareTo(secondCenter);
                }));
            else PartitionAtMedian(start, count, splitX);

            int firstCount = count / 2;
            return new Node
            {
                MinX = minX,
                MinY = minY,
                MaxX = maxX,
                MaxY = maxY,
                First = BuildNode(start, firstCount),
                Second = BuildNode(start + firstCount, count - firstCount),
            };
        }

        private void PartitionAtMedian(int start, int count, bool splitX)
        {
            int left = start, right = start + count - 1, median = start + count / 2;
            int budget = 2 * (int)Math.Log2(count) + 2;
            int Compare(BoundarySegment first, BoundarySegment second)
            {
                double a = splitX ? (double)first.MinX + first.MaxX : (double)first.MinY + first.MaxY;
                double b = splitX ? (double)second.MinX + second.MaxX : (double)second.MinY + second.MaxY;
                int comparison = a.CompareTo(b);
                return comparison != 0 ? comparison : first.Ordinal.CompareTo(second.Ordinal);
            }
            while (left < right)
            {
                if (--budget < 0)
                {
                    // Introselect guard for adversarially ordered boundaries.
                    Array.Sort(_segments, left, right - left + 1,
                        Comparer<BoundarySegment>.Create(Compare));
                    _partitionSortFallbacks++;
                    break;
                }
                _partitionCount++;
                int middle = left + (right - left) / 2;
                if (Compare(_segments[middle], _segments[left]) < 0)
                    (_segments[middle], _segments[left]) = (_segments[left], _segments[middle]);
                if (Compare(_segments[right], _segments[left]) < 0)
                    (_segments[right], _segments[left]) = (_segments[left], _segments[right]);
                if (Compare(_segments[right], _segments[middle]) < 0)
                    (_segments[right], _segments[middle]) = (_segments[middle], _segments[right]);
                BoundarySegment pivot = _segments[middle];
                int first = left, last = right;
                while (first <= last)
                {
                    while (Compare(_segments[first], pivot) < 0) first++;
                    while (Compare(_segments[last], pivot) > 0) last--;
                    if (first > last) break;
                    (_segments[first], _segments[last]) = (_segments[last], _segments[first]);
                    first++; last--;
                }
                if (median <= last) right = last;
                else if (median >= first) left = first;
                else break;
            }
        }

        private double DistanceSquared(Node node, Point64 point, double best)
        {
            if (DistanceToBoundsSquared(point, node) >= best)
                return best;

            if (node.Count > 0)
            {
                for (int index = node.Start; index < node.Start + node.Count; index++)
                    best = Math.Min(best, DistanceToSegmentSquared(point, _segments[index]));
                return best;
            }

            Node first = node.First!;
            Node second = node.Second!;
            double firstDistance = DistanceToBoundsSquared(point, first);
            double secondDistance = DistanceToBoundsSquared(point, second);
            if (secondDistance < firstDistance)
                (first, second) = (second, first);

            best = DistanceSquared(first, point, best);
            return DistanceSquared(second, point, best);
        }

        private static double DistanceToBoundsSquared(Point64 point, Node node)
        {
            double dx = point.X < node.MinX
                ? node.MinX - point.X
                : point.X > node.MaxX ? point.X - node.MaxX : 0;
            double dy = point.Y < node.MinY
                ? node.MinY - point.Y
                : point.Y > node.MaxY ? point.Y - node.MaxY : 0;
            return dx * dx + dy * dy;
        }

        private static double DistanceToSegmentSquared(
            Point64 point,
            BoundarySegment segment)
        {
            double dx = (double)segment.End.X - segment.Start.X;
            double dy = (double)segment.End.Y - segment.Start.Y;
            double lengthSquared = dx * dx + dy * dy;
            double position = lengthSquared <= 0
                ? 0
                : Math.Clamp(
                    (((double)point.X - segment.Start.X) * dx
                        + ((double)point.Y - segment.Start.Y) * dy) / lengthSquared,
                    0,
                    1);
            double nearestX = segment.Start.X + dx * position;
            double nearestY = segment.Start.Y + dy * position;
            double pointDx = point.X - nearestX;
            double pointDy = point.Y - nearestY;
            return pointDx * pointDx + pointDy * pointDy;
        }
    }

    /// <summary>Arc tolerance for round joins, in workspace units (mm).</summary>
    public const double DefaultArcTolerance = 0.25;

    /// <summary>Clipper2 miter limit, only consulted for <see cref="JoinType.Miter"/>.</summary>
    public const double DefaultMiterLimit = 10.0;

    // --- Scaling --------------------------------------------------------------

    private static Point64 ToPoint64(PointD p)
        // Math.Round rather than a (long)(v + 0.5) cast: the cast truncates toward zero
        // and therefore biases negative coordinates by a whole quantum.
        => new((long)Math.Round(p.x * PathUtils.Scale), (long)Math.Round(p.y * PathUtils.Scale));

    private static PointD ToPointD(Point64 p)
        => new(p.X / PathUtils.Scale, p.Y / PathUtils.Scale);

    private static Path64 ToPath64(IReadOnlyList<PointD> polygon)
    {
        var path = new Path64(polygon.Count);
        for (int i = 0; i < polygon.Count; i++)
            path.Add(ToPoint64(polygon[i]));
        return path;
    }

    private static Paths64 ToPaths64(IEnumerable<IReadOnlyList<PointD>> polygons, int minVertices = 3)
    {
        var paths = new Paths64();
        foreach (var polygon in polygons)
        {
            if (polygon.Count < minVertices)
                continue;
            paths.Add(ToPath64(polygon));
        }
        return paths;
    }

    private static List<List<PointD>> FromPaths64(Paths64 paths)
    {
        var output = new List<List<PointD>>(paths.Count);
        foreach (var path in paths)
        {
            if (path.Count < 3)
                continue;
            var polygon = new List<PointD>(path.Count);
            foreach (var pt in path)
                polygon.Add(ToPointD(pt));
            output.Add(polygon);
        }
        return output;
    }

    internal static List<List<PointD>> MaterializeIntegerPaths(Paths64 paths) => FromPaths64(paths);

    // Internal ordered unions keep their integer coordinates between calls.
    // Apply the exact floating normalization keys and List.Sort comparison used
    // by Union, so the next Boolean receives the same contour/vertex sequence.
    internal static Paths64 UnionNormalizedInteger(Paths64 subjects)
    {
        if (subjects.Count == 0) return new Paths64();
        var output = Clipper.Union(subjects, FillRule.NonZero);
        var keys = new Dictionary<Path64, (double Area, PointD Centroid)>(output.Count);
        foreach (var path in output)
        {
            const long maximumExactDoubleInteger = 1L << 53;
            bool needsLegacyRoundTrip = false;
            int smallest = 0;
            for (int i = 0; i < path.Count; i++)
            {
                Point64 point = path[i];
                needsLegacyRoundTrip |= point.X < -maximumExactDoubleInteger
                    || point.X > maximumExactDoubleInteger
                    || point.Y < -maximumExactDoubleInteger
                    || point.Y > maximumExactDoubleInteger;
                if (path[i].X < path[smallest].X
                    || (path[i].X == path[smallest].X && path[i].Y < path[smallest].Y)) smallest = i;
            }
            if (needsLegacyRoundTrip)
            {
                // Even exactly double-quantizable inputs can produce an odd
                // integer intersection beyond 2^53 (TopX adds a rounded delta
                // to an integer origin). Preserve the former materialization,
                // floating minimum-vertex ties and subsequent Boolean input.
                // Earlier contours were exact and already have their legacy
                // starts; this contour has not been rotated yet.
                long fallback = Interlocked.Increment(ref _integerUnionPrecisionFallbacks);
                if (fallback <= 4 || (fallback & (fallback - 1)) == 0)
                    Console.WriteLine($"[PolygonOps.IntegerUnionPrecision] fallback={fallback} "
                        + $"maximumExactDoubleInteger={maximumExactDoubleInteger} "
                        + $"legacyDoubleRoundTrip=True originalContourOrder=True");
                return ToPaths64(NormalizeOrder(FromPaths64(output)));
            }
            if (smallest != 0)
            {
                var rotated = new Path64(path.Count);
                for (int i = smallest; i < path.Count; i++) rotated.Add(path[i]);
                for (int i = 0; i < smallest; i++) rotated.Add(path[i]);
                path.Clear(); path.AddRange(rotated);
            }
            double area = 0, cx = 0, cy = 0;
            for (int i = 0, j = path.Count - 1; i < path.Count; j = i++)
            {
                var point = ToPointD(path[i]);
                var previous = ToPointD(path[j]);
                area += (previous.x - point.x) * (previous.y + point.y);
                cx += point.x; cy += point.y;
            }
            keys[path] = (Math.Abs(area * .5), path.Count == 0
                ? new PointD(0, 0) : new PointD(cx / path.Count, cy / path.Count));
        }
        output.Sort((first, second) =>
        {
            int comparison = keys[second].Area.CompareTo(keys[first].Area);
            if (comparison != 0) return comparison;
            comparison = keys[first].Centroid.x.CompareTo(keys[second].Centroid.x);
            return comparison != 0 ? comparison
                : keys[first].Centroid.y.CompareTo(keys[second].Centroid.y);
        });
        return output;
    }

    private static ClipperOffset CreateOffsetter(double arcTolerance)
        => new(
            miterLimit: DefaultMiterLimit,
            arcTolerance: arcTolerance * PathUtils.Scale,
            // Collinear vertices only inflate the point count; RDP would drop them anyway.
            preserveCollinear: false,
            reverseSolution: false);

    // The integer sites, canonical filled region and boundary hierarchy are
    // privately owned snapshots. Offsetters and per-execution certificates stay
    // in their own contexts; changing first-front tolerances shares only source
    // preparation and a compatible construction snapshot.
    private sealed class PreparedBoundarySource
    {
        private readonly object _holeErosionGate = new();
        private bool _holeErosionPrepared;
        private OriginalHoleErosionSource? _holeErosionSource;
        private string _holeErosionRefusal = "not-prepared";
        internal Paths64 Source { get; }
        internal Paths64 Region { get; }
        internal BoundarySegmentIndex Boundary { get; }
        internal int PointCount { get; }

        internal OriginalHoleErosionSource? OriginalHoleSource(CancellationToken token, out string reason)
        {
            token.ThrowIfCancellationRequested();
            lock (_holeErosionGate)
            {
                token.ThrowIfCancellationRequested();
                if (!_holeErosionPrepared)
                {
                    var prepared = OriginalHoleErosionSource.Capture(Region, token, out var refusal);
                    token.ThrowIfCancellationRequested();
                    _holeErosionSource = prepared;
                    _holeErosionRefusal = refusal;
                    _holeErosionPrepared = true;
                }
                reason = _holeErosionRefusal;
                return _holeErosionSource;
            }
        }

        internal PreparedBoundarySource(IReadOnlyList<PointD> outer,
            IEnumerable<IReadOnlyList<PointD>>? holes, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            Source = new Paths64();
            if (outer.Count >= 3)
            {
                var outerPath = ToPath64(outer);
                if (!Clipper.IsPositive(outerPath)) outerPath.Reverse();
                Source.Add(outerPath);
                if (holes is not null)
                    foreach (var hole in holes)
                    {
                        token.ThrowIfCancellationRequested();
                        if (hole.Count < 3) continue;
                        var path = ToPath64(hole);
                        if (Clipper.IsPositive(path)) path.Reverse();
                        Source.Add(path);
                    }
            }
            Region = Clipper.Union(Source, FillRule.NonZero);
            token.ThrowIfCancellationRequested();
            Boundary = new BoundarySegmentIndex(Region);
            PointCount = Source.Sum(path => path.Count);
        }
    }

    private sealed class PreparedOffsetConstruction
    {
        private readonly PreparedBoundarySource _geometry;
        private readonly CancellationToken _preparationToken;
        private readonly object _preparationGate = new();
        private Paths64 _source = null!, _region = null!;
        private int _pointCount;
        private double _guard;
        private volatile bool _prepared;
        internal Paths64 Source { get { EnsurePrepared(_preparationToken); return _source; } }
        internal Paths64 Region { get { EnsurePrepared(_preparationToken); return _region; } }
        internal int PointCount { get { EnsurePrepared(_preparationToken); return _pointCount; } }
        internal double Tolerance { get; }
        internal double Guard { get { EnsurePrepared(_preparationToken); return _guard; } }
        internal bool IsPrepared => _prepared;
        internal int BuildCount { get; private set; }
        internal double PreparationMilliseconds { get; private set; }

        internal PreparedOffsetConstruction(PreparedBoundarySource geometry,
            double tolerance, CancellationToken token)
        {
            Tolerance = tolerance; _geometry = geometry; _preparationToken = token;
            // An unreduced construction is already the original immutable
            // snapshot; it requires no RDP, union, XOR, or deferred work.
            if (tolerance == 0)
            {
                _source = geometry.Source; _region = geometry.Region;
                _pointCount = geometry.PointCount; _prepared = true;
            }
        }

        internal void EnsurePrepared(CancellationToken token)
        {
            if (_prepared) return;
            token.ThrowIfCancellationRequested();
            lock (_preparationGate)
            {
                if (_prepared) return;
                token.ThrowIfCancellationRequested();
                long started = Stopwatch.GetTimestamp();
                // Preserve the former constructor's complete path order, RDP,
                // conversion, signed-area fallback, union, and XOR exactly.
                // Only their evaluation time changes. Commit after all work
                // and cancellation checks, so a failed attempt publishes none
                // of a partial construction to shared tolerance contexts.
                var source = new Paths64(_geometry.Source.Select(path =>
                {
                    token.ThrowIfCancellationRequested();
                    var reduced = PolygonSimplify.RdpSegmentBounded(
                        FromPaths64(new Paths64 { path })[0], Tolerance);
                    token.ThrowIfCancellationRequested();
                    var candidate = reduced.Count >= 3 ? ToPath64(reduced) : path;
                    return Clipper.Area(candidate) * Clipper.Area(path) > 0 ? candidate : path;
                }));
                var region = ReferenceEquals(source, _geometry.Source)
                    ? _geometry.Region : Clipper.Union(source, FillRule.NonZero);
                token.ThrowIfCancellationRequested();
                int pointCount = source.Sum(path => path.Count);
                bool changed = pointCount < _geometry.PointCount
                    && Clipper.Xor(_geometry.Region, region, FillRule.NonZero)
                    .Any(path => Clipper.Area(path) != 0);
                token.ThrowIfCancellationRequested();
                _source = source; _region = region; _pointCount = pointCount;
                _guard = changed ? Tolerance : 0;
                PreparationMilliseconds = ElapsedMilliseconds(started); BuildCount++;
                _prepared = true;
                if (pointCount < _geometry.PointCount)
                    Console.WriteLine($"[PolygonOps.OffsetConstruction] originalPoints={_geometry.PointCount} "
                        + $"constructionPoints={pointCount} tolerance={Tolerance:R} "
                        + $"inwardGuard={_guard:R} identicalFilledGeometry={_guard == 0} "
                        + $"originalSourceCapsuleProof=True zeroOffsetExact=True");
                Console.WriteLine($"[PolygonOps.OffsetConstructionLazy] event=prepared buildCount={BuildCount} "
                    + $"originalPoints={_geometry.PointCount} constructionPoints={pointCount} "
                    + $"tolerance={Tolerance:R} elapsedMs={PreparationMilliseconds:F2} "
                    + $"pathOrder=original rdpUnionXor=unchanged publication=complete-snapshot");
            }
        }
    }

    internal sealed class PreparedBoundaryOffset
    {
        private readonly PreparedBoundarySource _geometry;
        private readonly PreparedOffsetConstruction _construction;
        private readonly Paths64 _source;
        private Paths64 _constructionSource => _construction.Source;
        private Paths64 _constructionRegion => _construction.Region;
        private readonly BoundarySegmentIndex _boundaryIndex;
        // Read-only original geometry authority; construction aids never replace it.
        internal BoundarySegmentIndex OriginalBoundary => _boundaryIndex;
        private readonly BoundarySegmentIndex.ClearanceCache _feasibilityCache = new();
        private ClipperOffset? _offsetter;
        private ClipperOffset? _outwardOffsetter;
        private readonly JoinType _joinType;
        private readonly double _arcTolerance;
        private readonly double _simplifyTolerance;
        private double _constructionGuard => _construction.Guard;
        private readonly double? _constructionSweepRadius;
        private readonly IReadOnlyList<PointD>? _constructionClipBoundary;
        private readonly CancellationToken _cancellationToken;
        private PreparedBoundaryOffset? _originalConstruction;
        private bool _constructionCoverageVerified;
        private double? _originalFirstDelta;
        private List<List<PointD>>? _originalFirstFront;
        private Paths64? _sourceRegion;
        private Paths64? _denseInsetBase;
        private double _denseInsetDelta;
        private readonly bool _twoOriginExperiment;
        private readonly bool _separateOriginalFirst;
        private Paths64? _shallowInsetBase;
        private double _shallowInsetDelta;

        internal PreparedBoundaryOffset(
            IReadOnlyList<PointD> outer,
            IEnumerable<IReadOnlyList<PointD>>? holes,
            double arcTolerance,
            double simplifyTolerance,
            JoinType joinType,
            double sourceConstructionTolerance,
            double? sourceConstructionSweepRadius,
            CancellationToken cancellationToken,
            IReadOnlyList<PointD>? sourceConstructionClipBoundary,
            bool sourceConstructionValidateFirstFootprint,
            bool? twoOriginConstruction = null,
            bool separateOriginalFirstConstruction = false)
            : this(outer, holes, arcTolerance, simplifyTolerance, joinType,
                sourceConstructionTolerance, sourceConstructionSweepRadius, cancellationToken,
                sourceConstructionClipBoundary, sourceConstructionValidateFirstFootprint, null, null,
                twoOriginConstruction ?? (Environment.GetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_EXPERIMENT") == "1"
                    && Environment.GetEnvironmentVariable("RW_TOOLPATHS_TWO_ORIGIN_REFERENCE") != "1"),
                separateOriginalFirstConstruction)
        { }

        private PreparedBoundaryOffset(
            IReadOnlyList<PointD>? outer,
            IEnumerable<IReadOnlyList<PointD>>? holes,
            double arcTolerance,
            double simplifyTolerance,
            JoinType joinType,
            double sourceConstructionTolerance,
            double? sourceConstructionSweepRadius,
            CancellationToken cancellationToken,
            IReadOnlyList<PointD>? sourceConstructionClipBoundary,
            bool sourceConstructionValidateFirstFootprint,
            PreparedBoundarySource? geometry,
            PreparedOffsetConstruction? construction,
            bool twoOriginConstruction,
            bool separateOriginalFirstConstruction)
        {
            cancellationToken.ThrowIfCancellationRequested();
            _cancellationToken = cancellationToken;
            _twoOriginExperiment = twoOriginConstruction;
            _separateOriginalFirst = separateOriginalFirstConstruction;
            if (!double.IsFinite(sourceConstructionTolerance) || sourceConstructionTolerance < 0)
                throw new ArgumentOutOfRangeException(nameof(sourceConstructionTolerance));
            if (sourceConstructionSweepRadius is { } sweep && (!double.IsFinite(sweep) || sweep <= 0))
                throw new ArgumentOutOfRangeException(nameof(sourceConstructionSweepRadius));
            _constructionSweepRadius = sourceConstructionSweepRadius;
            _constructionClipBoundary = sourceConstructionClipBoundary;
            _constructionCoverageVerified = !sourceConstructionValidateFirstFootprint;
            long prepareStart = Stopwatch.GetTimestamp();
            _arcTolerance = arcTolerance;
            _simplifyTolerance = simplifyTolerance;
            _joinType = joinType;
            _geometry = geometry ?? new PreparedBoundarySource(outer!, holes, cancellationToken);
            _construction = construction is not null && construction.Tolerance == sourceConstructionTolerance
                ? construction : new PreparedOffsetConstruction(_geometry, sourceConstructionTolerance, cancellationToken);
            _source = _geometry.Source;
            _sourceRegion = _geometry.Region;
            _boundaryIndex = _geometry.Boundary;
            SourcePointCount = _geometry.PointCount;
            SourcePreparationReused = geometry is not null;
            ConstructionPreparationReused = ReferenceEquals(construction, _construction);
            PreparationMilliseconds = ElapsedMilliseconds(prepareStart);
        }

        internal int SourcePathCount => _source.Count;
        internal int SourcePointCount { get; }
        internal bool SourcePreparationReused { get; }
        internal bool ConstructionPreparationReused { get; }
        internal int BoundaryPartitionCount => _boundaryIndex.PartitionCount;
        internal int BoundaryPartitionSortFallbacks => _boundaryIndex.PartitionSortFallbacks;
        internal bool LegacyWitnessIndexBuilt => _boundaryIndex.LegacyWitnessIndexBuilt;
        internal int OffsetCount { get; private set; }
        internal int FallbackCount { get; private set; }
        internal double PreparationMilliseconds { get; }
        internal bool ConstructionPrepared => _construction.IsPrepared;
        internal int ConstructionBuildCount => _construction.BuildCount;
        internal double ConstructionPreparationMilliseconds => _construction.PreparationMilliseconds;
        internal int ConstructionSkippedExecutions { get; private set; }
        internal int OrdinaryOffsetterBuildCount { get; private set; }
        internal int OutwardOffsetterBuildCount { get; private set; }
        internal int OrdinaryOffsetterSkippedExecutions { get; private set; }
        internal double OrdinaryOffsetterPreparationMilliseconds { get; private set; }
        internal double OffsetExecutionMilliseconds { get; private set; }
        internal double ValidationMilliseconds { get; private set; }
        internal double FallbackMilliseconds { get; private set; }
        internal double MaterializationMilliseconds { get; private set; }
        internal int CompactionCount { get; private set; }
        internal int DeferredCompactionCount { get; private set; }
        internal double CompactionMilliseconds { get; private set; }
        internal int CompactionProposalCount { get; private set; }
        internal int DenseInsetReuseCount { get; private set; }
        internal int DenseSourceConstructionCount { get; private set; }
        internal int TwoOriginShallowReuseCount { get; private set; }
        internal int TwoOriginDeepReuseCount { get; private set; }
        internal bool TwoOriginExperimentEnabled => _twoOriginExperiment;
        internal double TwoOriginShallowDistance => -_shallowInsetDelta / PathUtils.Scale;
        internal double TwoOriginDeepDistance => -_denseInsetDelta / PathUtils.Scale;
        internal int TwoOriginShallowAidPoints => _shallowInsetBase?.Sum(path => path.Count) ?? 0;
        internal int TwoOriginDeepAidPoints => _denseInsetBase?.Sum(path => path.Count) ?? 0;
        internal bool AcceptedInsetSeeded { get; private set; }
        internal double AcceptedInsetSeedMilliseconds { get; private set; }
        internal int OriginalFirstAuthorityCount { get; private set; }
        internal double OriginalFirstAuthorityMilliseconds { get; private set; }
        private int _originalHoleErosionCount;
        internal int OriginalHoleErosionCount => _originalConstruction?.OriginalHoleErosionCount
            ?? _originalHoleErosionCount;

        internal PreparedBoundaryOffset WithTolerances(double arcTolerance, double simplifyTolerance,
            bool validateFirstFootprint = true)
            => new(null, null, arcTolerance, simplifyTolerance, _joinType,
                _construction.Tolerance, _constructionSweepRadius, _cancellationToken,
                _constructionClipBoundary, validateFirstFootprint, _geometry, _construction, _twoOriginExperiment,
                _separateOriginalFirst);

        private void RecordLazySkip(string reason, double delta)
        {
            bool conditioning = !_construction.IsPrepared, ordinary = _offsetter is null;
            if (conditioning) ConstructionSkippedExecutions++;
            if (ordinary) OrdinaryOffsetterSkippedExecutions++;
            int count = Math.Max(ConstructionSkippedExecutions, OrdinaryOffsetterSkippedExecutions);
            if ((conditioning || ordinary) && (count <= 4 || (count & (count - 1)) == 0))
                Console.WriteLine($"[PolygonOps.OffsetConstructionLazy] event=skipped reason={reason} delta={delta:R} "
                    + $"sourcePoints={SourcePointCount} conditioningDeferred={conditioning} ordinaryOffsetterDeferred={ordinary} "
                    + $"conditioningSkips={ConstructionSkippedExecutions} ordinaryOffsetterSkips={OrdinaryOffsetterSkippedExecutions} "
                    + $"zeroFrontAndOriginalFeasibility=unchanged noConstructionOperandChanged=True");
        }

        private ClipperOffset GetOrdinaryOffsetter(CancellationToken token)
        {
            if (_offsetter is not null) return _offsetter;
            _construction.EnsurePrepared(token);
            long started = Stopwatch.GetTimestamp();
            var offsetter = CreateOffsetter(_arcTolerance);
            if (_source.Count > 0) offsetter.AddPaths(_constructionSource, _joinType, EndType.Polygon);
            token.ThrowIfCancellationRequested();
            _offsetter = offsetter; OrdinaryOffsetterBuildCount++;
            OrdinaryOffsetterPreparationMilliseconds += ElapsedMilliseconds(started);
            Console.WriteLine($"[PolygonOps.OffsetConstructionLazy] event=ordinary-offsetter-prepared "
                + $"buildCount={OrdinaryOffsetterBuildCount} sourcePoints={_construction.PointCount} "
                + $"elapsedMs={OrdinaryOffsetterPreparationMilliseconds:F2} originalPathOrder=True");
            return offsetter;
        }

        internal bool TrySeedAcceptedFirstInset(IReadOnlyList<List<PointD>> accepted,
            double delta, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            _construction.EnsurePrepared(token);
            bool dense = _construction.PointCount > 4096
                || (_construction.PointCount > 2048 && SourcePathCount > 32);
            if (!dense || !double.IsFinite(delta) || delta >= 0 || accepted.Count == 0
                || (_twoOriginExperiment ? _shallowInsetBase is not null || _denseInsetBase is not null
                    : _denseInsetBase is not null))
                return false;
            long started = Stopwatch.GetTimestamp();
            var source = Clipper.Union(ToPaths64(accepted), FillRule.NonZero);
            double required = Math.Max(0, -delta * PathUtils.Scale - 4);
            if (required <= 0 || Clipper.Difference(source, _sourceRegion!, FillRule.NonZero)
                .Any(path => Clipper.Area(path) != 0))
            {
                Console.WriteLine($"[PolygonOps.AcceptedInsetSeed] accepted=False delta={delta:R} "
                    + $"reason=complete-original-filled-source-containment authority=unchanged-source-construction");
                return false;
            }
            int chords = 0;
            foreach (var contour in source)
            for (int end = 0, start = contour.Count - 1; end < contour.Count; start = end++)
            {
                if ((++chords & 255) == 0) token.ThrowIfCancellationRequested();
                if (_boundaryIndex.IsInside(contour[start])
                    && _boundaryIndex.ClearanceViolation(contour[start], contour[end], required) is null)
                    continue;
                Console.WriteLine($"[PolygonOps.AcceptedInsetSeed] accepted=False delta={delta:R} "
                    + $"chords={chords} reason=original-source-chord-clearance authority=unchanged-source-construction");
                return false;
            }
            Paths64 seedCache;
            if (_twoOriginExperiment)
            {
                // The shallow seed never replaces the established first-later
                // cache. Both are fixed construction aids; their candidates
                // still need original walls and the collector's full shell.
                seedCache = CreateDenseInsetCache(source, delta * PathUtils.Scale, token,
                    certifyGuardedSeed: true);
                token.ThrowIfCancellationRequested();
                if (seedCache.Count > 0)
                {
                    _shallowInsetBase = seedCache;
                    _shallowInsetDelta = delta * PathUtils.Scale;
                }
            }
            else
            {
                CacheDenseInset(source, delta * PathUtils.Scale, token);
                seedCache = _denseInsetBase!;
            }
            AcceptedInsetSeeded = seedCache.Count > 0;
            AcceptedInsetSeedMilliseconds += ElapsedMilliseconds(started);
            int cachePoints = seedCache.Sum(path => path.Count);
            if (!AcceptedInsetSeeded && !_twoOriginExperiment) { _denseInsetBase = null; _denseInsetDelta = 0; }
            Console.WriteLine($"[PolygonOps.AcceptedInsetSeed] accepted={AcceptedInsetSeeded} delta={delta:R} "
                + $"chords={chords} cachePoints={cachePoints} "
                + $"twoOriginExperiment={_twoOriginExperiment} "
                + $"originalWalls=True constructionOnly=True futureShellProof=unchanged originalSourceModified=False");
            if (_twoOriginExperiment)
                LogTwoOrigin("publish-shallow", delta, "shallow", cachePoints);
            return AcceptedInsetSeeded;
        }

        private void CacheDenseInset(Paths64 solution, double scaledDelta, CancellationToken token)
        {
            var cache = CreateDenseInsetCache(solution, scaledDelta, token);
            token.ThrowIfCancellationRequested();
            _denseInsetBase = cache;
            _denseInsetDelta = scaledDelta;
            if (_twoOriginExperiment)
                LogTwoOrigin("publish-deep", scaledDelta / PathUtils.Scale, "deep", cache.Sum(path => path.Count));
        }

        private Paths64 CreateDenseInsetCache(Paths64 solution, double scaledDelta, CancellationToken token,
            bool certifyGuardedSeed = false)
        {
            var compact = new Paths64(solution.Select(path => {
                token.ThrowIfCancellationRequested();
                var reduced = PolygonSimplify.RdpSegmentBounded(FromPaths64(new Paths64 { path })[0], _arcTolerance);
                var reducedPath = reduced.Count >= 3 ? ToPath64(reduced) : new Path64(path);
                return Clipper.Area(reducedPath) * Clipper.Area(path) > 0 ? reducedPath : new Path64(path);
            }));
            // Fixed first-inset construction only. Guard inward before intersecting
            // so replacement chords cannot create alternating outward notches.
            var cacheOffsetter = CreateOffsetter(_arcTolerance / 4);
            cacheOffsetter.AddPaths(compact, JoinType.Round, EndType.Polygon);
            var guarded = new Paths64();
            cacheOffsetter.Execute(-Math.Max(8.0, _arcTolerance * PathUtils.Scale + 8), guarded);
            bool directGuarded = false;
            if (certifyGuardedSeed || (!_twoOriginExperiment
                && Environment.GetEnvironmentVariable("RW_TOOLPATHS_GUARDED_INSET_EXPERIMENT") == "1"))
            {
                // This is an ORIGINAL-source certified construction aid, not
                // an exact semigroup representation of the commanded front.
                // Avoid reintroducing the dense source's tiny intersection
                // notches only when the entire signed guarded domain and all
                // boundary chords independently retain first-inset authority.
                double required = Math.Max(0, -scaledDelta - 4);
                directGuarded = guarded.Count > 0 && required > 0
                    && Clipper.Difference(guarded, _sourceRegion!, FillRule.NonZero).Count == 0;
                int chords = 0;
                if (directGuarded)
                    foreach (var contour in guarded)
                    {
                        for (int end = 0, start = contour.Count - 1; end < contour.Count; start = end++)
                        {
                            if ((++chords & 255) == 0) token.ThrowIfCancellationRequested();
                            if (_boundaryIndex.IsInside(contour[start])
                                && _boundaryIndex.ClearanceViolation(contour[start], contour[end], required) is null)
                                continue;
                            directGuarded = false;
                            break;
                        }
                        if (!directGuarded) break;
                    }
                Console.WriteLine($"[PolygonOps.GuardedInsetExperiment] certified={directGuarded} "
                    + $"chords={chords} guardedPoints={guarded.Sum(path => path.Count)} "
                    + $"originalFirstClearance={required / PathUtils.Scale:R} "
                    + $"authority=complete-original-filled-domain-and-every-chord laterShellProof=unchanged "
                    + $"failure=established-intersected-cache commandedFirstFrontModified=False");
            }
            var cache = directGuarded ? guarded : Clipper.Intersect(solution, guarded, FillRule.NonZero);
            token.ThrowIfCancellationRequested();
            Console.WriteLine($"[PolygonOps.DenseInsetCache] validatedContours={solution.Count} " +
                $"cacheContours={cache.Count} validatedPoints={solution.Sum(path => path.Count)} " +
                $"cachePoints={cache.Sum(path => path.Count)} guard={_arcTolerance:R}");
            return cache;
        }

        private void LogTwoOrigin(string stage, double delta, string selected, int selectedPoints)
            => Console.WriteLine($"[PolygonOps.TwoOriginConstruction] event={stage} requestedDelta={delta:R} "
                + $"selected={selected} selectedAidPoints={selectedPoints} "
                + $"shallowOrigin={TwoOriginShallowDistance:R} shallowAidPoints={TwoOriginShallowAidPoints} "
                + $"deepOrigin={TwoOriginDeepDistance:R} deepAidPoints={TwoOriginDeepAidPoints} "
                + $"shallowReuses={TwoOriginShallowReuseCount} deepReuses={TwoOriginDeepReuseCount} "
                + $"mainOriginalSourceBuilds={DenseSourceConstructionCount} sharedOriginalPreparation=True "
                + $"preparationMs={PreparationMilliseconds:F2} seedMs={AcceptedInsetSeedMilliseconds:F2} "
                + $"originalWalls=True shellProof=unchanged stockAuthority=False accumulatedApproximation=False");

        internal List<List<PointD>> ExecuteOriginalFirstAuthority(double delta,
            CancellationToken cancellationToken = default)
        {
            if (!_constructionSweepRadius.HasValue || delta >= 0)
                return Execute(delta, cancellationToken);
            if (!cancellationToken.CanBeCanceled) cancellationToken = _cancellationToken;
            cancellationToken.ThrowIfCancellationRequested();
            // The original-first branch depends on the exact XOR-derived
            // guard. Preserve that decision; laziness cannot assume a positive
            // guard merely because a conditioning tolerance was requested.
            _construction.EnsurePrepared(cancellationToken);
            if (_constructionGuard <= 0) return Execute(delta, cancellationToken);
            if (_originalFirstDelta == delta) return Execute(delta, cancellationToken);
            long started = Stopwatch.GetTimestamp();
            OffsetCount++;
            var original = OriginalConstruction();
            double previousExecute = original.OffsetExecutionMilliseconds;
            double previousValidation = original.ValidationMilliseconds;
            double previousFallback = original.FallbackMilliseconds;
            double previousMaterialization = original.MaterializationMilliseconds;
            int previousFallbacks = original.FallbackCount;
            // Select the already established original-site first authority
            // directly. A discarded conditioned candidate cannot improve its
            // proof: the complete original source, native first tolerance and
            // inversion/chord checks still define these commands.
            var front = original.ExecuteCore(delta, cancellationToken, true, _separateOriginalFirst);
            OffsetExecutionMilliseconds += original.OffsetExecutionMilliseconds - previousExecute;
            ValidationMilliseconds += original.ValidationMilliseconds - previousValidation;
            FallbackMilliseconds += original.FallbackMilliseconds - previousFallback;
            FallbackCount += original.FallbackCount - previousFallbacks;
            MaterializationMilliseconds += original.MaterializationMilliseconds - previousMaterialization;
            _originalFirstDelta = delta;
            _originalFirstFront = front;
            _constructionCoverageVerified = true;
            long compactStarted = Stopwatch.GetTimestamp();
            var commanded = CompactCandidate(front, delta, cancellationToken);
            MaterializationMilliseconds += ElapsedMilliseconds(compactStarted);
            OriginalFirstAuthorityCount++;
            OriginalFirstAuthorityMilliseconds += ElapsedMilliseconds(started);
            Console.WriteLine($"[PolygonOps.OriginalFirstAuthority] inset={delta:R} "
                + $"originalPoints={SourcePointCount} constructionPoints={_construction.PointCount} "
                + $"nativeArcTolerance={_arcTolerance:R} commandedPoints={commanded.Sum(path => path.Count)} "
                + $"firstSourceCoverage=original conditionedCandidateBuilt=False differentialAlternativeRequired=False "
                + $"originalWallProof=unchanged physicalCommandCleanup=unchanged laterShellProof=unchanged");
            return commanded;
        }

        internal List<List<PointD>> Execute(double delta, CancellationToken cancellationToken = default,
            bool compactFront = true)
            => ExecuteCore(delta, cancellationToken, compactFront, false);

        private List<List<PointD>> ExecuteCore(double delta, CancellationToken cancellationToken,
            bool compactFront, bool separateOriginalFirst)
        {
            if (!cancellationToken.CanBeCanceled) cancellationToken = _cancellationToken;
            cancellationToken.ThrowIfCancellationRequested();
            OffsetCount++;
            if (_originalFirstDelta == delta)
            {
                Console.WriteLine($"[PolygonOps.OffsetConstructionBypass] delta={delta:R} "
                    + $"reason=certified-original-first-front cached=True laterFronts=conditioned-with-collector-shell-proof");
                if (!compactFront) { DeferredCompactionCount++; return _originalFirstFront!; }
                return CompactCandidate(_originalFirstFront!, delta, cancellationToken);
            }
            if (_source.Count == 0)
            {
                RecordLazySkip("empty-original-source", delta);
                return new List<List<PointD>>();
            }

            var solution = new Paths64();
            double scaledDelta = delta * PathUtils.Scale;
            if (scaledDelta < 0 && !_boundaryIndex.MayContainDisc(-scaledDelta,
                _arcTolerance * PathUtils.Scale, _feasibilityCache))
            {
                RecordLazySkip("original-source-infeasible", delta);
                return new List<List<PointD>>();
            }
            cancellationToken.ThrowIfCancellationRequested();
            if (scaledDelta < 0) _construction.EnsurePrepared(cancellationToken);

            long executeStart = Stopwatch.GetTimestamp();
            // Many short protected islands can make joined offsets expensive well
            // below the single-contour point threshold (e.g. 3,700 points / 53 holes).
            int constructionPoints = scaledDelta < 0 ? _construction.PointCount : SourcePointCount;
            bool usedBoundaryStroke = constructionPoints > 4096
                || (constructionPoints > 2048 && SourcePathCount > 32);
            Paths64? selectedInsetBase = _denseInsetBase;
            double selectedInsetDelta = _denseInsetDelta;
            if (scaledDelta == 0)
            {
                // A compensated V-carve layer already owns this canonical first
                // front. Source construction reduction applies only to offsets.
                solution = new Paths64(_sourceRegion!);
                RecordLazySkip("zero-original-front", delta);
            }
            else if (usedBoundaryStroke)
            {
                RecordLazySkip("dense-boundary-strokes", delta);
                // Dense image contours make one monolithic offset pathological. The existing
                // conservative boundary-band construction computes the same safe erosion/dilation.
                _sourceRegion ??= Clipper.Union(_source, FillRule.NonZero);
                bool reuseInset = scaledDelta < 0 && _denseInsetBase != null && scaledDelta < _denseInsetDelta;
                bool reuseShallow = _twoOriginExperiment && _shallowInsetBase != null
                    && _denseInsetBase != null && scaledDelta < _shallowInsetDelta
                    && scaledDelta > _denseInsetDelta;
                if (reuseShallow)
                {
                    selectedInsetBase = _shallowInsetBase;
                    selectedInsetDelta = _shallowInsetDelta;
                    reuseInset = true;
                }
                // Erosion by two disks equals erosion by the sum of their radii. Always reuse
                // the first inset (never the preceding ring), so rounding does not accumulate.
                if (reuseInset)
                {
                    DenseInsetReuseCount++;
                    if (_twoOriginExperiment)
                    {
                        if (reuseShallow) TwoOriginShallowReuseCount++;
                        else TwoOriginDeepReuseCount++;
                    }
                    // Even compacted contours can make a joined polygon offset spend
                    // seconds resolving inverted inner loops. The same round-capped
                    // boundary construction is cheap on this much smaller fixed inset.
                    // It reserves cap error and the original source still validates it.
                    solution = BuildConservativeOffset(selectedInsetBase!,
                        scaledDelta - selectedInsetDelta, _arcTolerance, sourceIsCanonical: true);
                }
                else
                {
                    DenseSourceConstructionCount++;
                    bool proposed = separateOriginalFirst && _separateOriginalFirst && scaledDelta < 0
                        && _construction.Tolerance == 0 && ReferenceEquals(_constructionRegion, _geometry.Region)
                        && SourcePointCount > 8192 && TryBuildOriginalHoleErosion(_geometry, scaledDelta,
                            _arcTolerance, cancellationToken, out solution);
                    if (!proposed)
                        solution = BuildConservativeOffset(scaledDelta < 0 ? _constructionRegion : _sourceRegion,
                            scaledDelta < 0 ? scaledDelta - _constructionGuard * PathUtils.Scale : scaledDelta,
                            _arcTolerance, sourceIsCanonical: true);
                    else _originalHoleErosionCount++;
                }
                FallbackCount++;
                FallbackMilliseconds += ElapsedMilliseconds(executeStart);
                Console.WriteLine($"[PolygonOps.DenseOffset] sourcePoints={SourcePointCount} delta={delta:R} paths={solution.Count} reusedFirstInset={reuseInset}");
                if (_twoOriginExperiment)
                    LogTwoOrigin("trial", delta, reuseInset ? reuseShallow ? "shallow" : "deep" : "original-source",
                        reuseInset ? selectedInsetBase!.Sum(path => path.Count) : constructionPoints);
            }
            else
            {
                _construction.EnsurePrepared(cancellationToken);
                double constructionDelta = scaledDelta < 0
                    ? scaledDelta - _constructionGuard * PathUtils.Scale : scaledDelta;
                ClipperOffset constructionOffsetter;
                if (scaledDelta > 0 && _construction.PointCount < SourcePointCount)
                {
                    if (_outwardOffsetter is null)
                    {
                        _outwardOffsetter = CreateOffsetter(_arcTolerance);
                        _outwardOffsetter.AddPaths(_source, _joinType, EndType.Polygon);
                        OutwardOffsetterBuildCount++;
                    }
                    constructionOffsetter = _outwardOffsetter;
                    Console.WriteLine($"[PolygonOps.OffsetConstructionBypass] delta={delta:R} "
                        + $"reason=outward-offset originalSites=True");
                }
                else constructionOffsetter = GetOrdinaryOffsetter(cancellationToken);
                constructionOffsetter.Execute(constructionDelta, solution);
                OffsetExecutionMilliseconds += ElapsedMilliseconds(executeStart);
            }

            int rawCount = solution.Count;
            // Validation may discard contours. Keep the complete candidate, including its holes,
            // until any significant clearance defect has been repaired as one region.
            var rawCandidate = new Paths64(solution);
            long validationStart = Stopwatch.GetTimestamp();
            int rejected = RemoveOffsetInversions(
                _boundaryIndex,
                solution,
                scaledDelta,
                _arcTolerance,
                out bool rejectedSignificantContour,
                out string details,
                out double significantDeficit);
            ValidationMilliseconds += ElapsedMilliseconds(validationStart);

            int fallbackPathCount = 0;
            int fallbackRejected = 0;
            if (!usedBoundaryStroke && rejected > 0 && (solution.Count == 0 || rejectedSignificantContour))
            {
                long fallbackStart = Stopwatch.GetTimestamp();
                _sourceRegion ??= Clipper.Union(_source, FillRule.NonZero);
                solution = BuildConservativeOffset(
                    _sourceRegion,
                    scaledDelta,
                    _arcTolerance,
                    sourceIsCanonical: true);
                FallbackMilliseconds += ElapsedMilliseconds(fallbackStart);
                FallbackCount++;
                fallbackPathCount = solution.Count;

                rawCandidate = new Paths64(solution);
                long fallbackValidationStart = Stopwatch.GetTimestamp();
                fallbackRejected = RemoveOffsetInversions(
                    _boundaryIndex,
                    solution,
                    scaledDelta,
                    _arcTolerance,
                    out rejectedSignificantContour,
                    out string fallbackDetails,
                    out significantDeficit);
                ValidationMilliseconds += ElapsedMilliseconds(fallbackValidationStart);
                if (fallbackRejected > 0)
                    details += $";fallbackRejected={fallbackRejected}:{fallbackDetails}";
            }

            if (rejectedSignificantContour)
            {
                long repairStart = Stopwatch.GetTimestamp();
                solution = RepairConservativeOffset(_boundaryIndex, _sourceRegion!, rawCandidate,
                    scaledDelta, _arcTolerance, significantDeficit, selectedInsetBase, selectedInsetDelta);
                FallbackMilliseconds += ElapsedMilliseconds(repairStart);
                FallbackCount++;
            }

            if (scaledDelta < 0 && (_constructionGuard > 0 || (_twoOriginExperiment && AcceptedInsetSeeded)))
            {
                // The reduced sites are a construction aid, never the clearance
                // authority. Prove every replacement command chord against all
                // ORIGINAL source walls at the original requested inset.
                double required = Math.Max(0, -scaledDelta - 4);
                int audited = 0;
                for (int attempt = 0; ; attempt++)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    var missing = new HashSet<(Point64 Start, Point64 End)>();
                    foreach (var contour in solution)
                    for (int end = 0, start = contour.Count - 1; end < contour.Count; start = end++)
                    {
                        if ((end & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                        audited++;
                        if (_twoOriginExperiment && AcceptedInsetSeeded && !_boundaryIndex.IsInside(contour[start]))
                            missing.Add(_boundaryIndex.NearestSegment(contour[start]));
                        else if (_boundaryIndex.ClearanceViolation(contour[start], contour[end], required) is { } wall)
                            missing.Add(wall);
                    }
                    if (missing.Count == 0)
                    {
                        Console.WriteLine($"[PolygonOps.OffsetConstructionProof] inset={delta:R} chords={audited} "
                            + $"repairs={attempt} requiredClearance={required / PathUtils.Scale:R} originalWalls=True");
                        break;
                    }
                    if (attempt == 3)
                        throw new InvalidOperationException("Reduced offset construction failed its original-source complete-chord proof.");
                    var capsules = new Paths64();
                    foreach (var wall in missing)
                    {
                        cancellationToken.ThrowIfCancellationRequested();
                        var guard = CreateOffsetter((_constructionGuard > 0
                            ? Math.Min(_arcTolerance, _constructionGuard) : _arcTolerance) / 4);
                        guard.AddPath(new Path64 { wall.Start, wall.End }, JoinType.Round, EndType.Round);
                        var capsule = new Paths64();
                        guard.Execute(-scaledDelta + _constructionGuard * PathUtils.Scale + 8, capsule);
                        capsules.AddRange(capsule);
                    }
                    solution = Clipper.Difference(solution, Clipper.Union(capsules, FillRule.NonZero), FillRule.NonZero);
                    cancellationToken.ThrowIfCancellationRequested();
                    Console.WriteLine($"[PolygonOps.OffsetConstructionRepair] inset={delta:R} attempt={attempt + 1} "
                        + $"originalWallCapsules={missing.Count} contours={solution.Count}");
                }
            }

            if (usedBoundaryStroke && scaledDelta < 0 && _denseInsetBase == null
                && (!_twoOriginExperiment || _shallowInsetBase is null || scaledDelta < _shallowInsetDelta))
            {
                CacheDenseInset(solution, scaledDelta, cancellationToken);
            }

            if (rejected > 0)
            {
                PerfLog.Stop(
                    "PolygonOps.OffsetBoundaryValidation",
                    validationStart,
                    $"delta={delta:F6} source={_source.Count} raw={rawCount} " +
                    $"rejected={rejected} fallback={fallbackPathCount} " +
                    $"fallbackRejected={fallbackRejected} " +
                    $"kept={solution.Count} {details}");
            }

            long materializationStart = Stopwatch.GetTimestamp();
            var output = _simplifyTolerance > 0
                ? PolygonSimplify.RdpAll(FromPaths64(solution), _simplifyTolerance)
                : FromPaths64(solution);
            NormalizeOrder(output);
            cancellationToken.ThrowIfCancellationRequested();
            if (scaledDelta < 0 && !_constructionCoverageVerified && _constructionGuard > 0
                && _constructionSweepRadius is { } sweepRadius)
            {
                // A site-error bound alone does not bound an inset intersection:
                // a near-tangent source corner can amplify a micron guard into a
                // much larger physical recession. Certify the FIRST physical
                // front against original-site construction. Later conditioned
                // fronts use the common collector's swept-shell proof against
                // that accepted first front; no raw-site mode is imposed on them.
                var original = OriginalConstruction().Execute(delta, cancellationToken);
                cancellationToken.ThrowIfCancellationRequested();
                var components = DifferenceToComponents(original, Array.Empty<IReadOnlyList<PointD>>());
                var originalFront = components.SelectMany(component =>
                    new[] { (IReadOnlyList<PointD>)component.Outer }.Concat(component.Holes));
                const double proofArc = .001;
                const double coverageError = .003 + 4 / PathUtils.Scale;
                var oldSweep = BufferCenterlines(originalFront, sweepRadius, proofArc);
                cancellationToken.ThrowIfCancellationRequested();
                var newSweep = BufferCenterlines(output, sweepRadius, proofArc);
                cancellationToken.ThrowIfCancellationRequested();
                var lost = Offset(Difference(oldSweep, newSweep), -coverageError, proofArc / 4, 0);
                double lostArea = Math.Abs(lost.Sum(SignedArea));
                cancellationToken.ThrowIfCancellationRequested();
                _constructionCoverageVerified = true;
                bool reverted = lostArea > 0;
                Console.WriteLine($"[PolygonOps.OffsetConstructionCoverage] inset={delta:R} "
                    + $"sweepRadius={sweepRadius:R} originalPoints={SourcePointCount} constructionPoints={constructionPoints} "
                    + $"lostFootprintArea={lostArea:R} coverageError={coverageError:R} "
                    + $"accepted={!reverted} firstFrontFixed=True laterFronts=conditioned-with-collector-shell-proof "
                    + $"reason={(reverted ? "original-front-footprint-recession" : "physical-covariance")}");
                if (reverted)
                {
                    _originalFirstDelta = delta;
                    _originalFirstFront = original;
                    output = original;
                }
            }
            if (compactFront) output = CompactCandidate(output, delta, cancellationToken);
            else DeferredCompactionCount++;
            MaterializationMilliseconds += ElapsedMilliseconds(materializationStart);
            return output;
        }

        internal List<List<PointD>> CompactCandidate(List<List<PointD>> front, double delta,
            CancellationToken cancellationToken, bool proposeForShellAudit = false)
        {
            cancellationToken.ThrowIfCancellationRequested();
            long started = Stopwatch.GetTimestamp();
            var compacted = CompactFront(front, delta, cancellationToken, proposeForShellAudit);
            cancellationToken.ThrowIfCancellationRequested();
            CompactionCount++;
            if (proposeForShellAudit) CompactionProposalCount++;
            CompactionMilliseconds += ElapsedMilliseconds(started);
            return compacted;
        }

        internal List<List<PointD>> CompactFirstCommands(List<List<PointD>> front, double delta,
            CancellationToken cancellationToken)
        {
            if (_constructionSweepRadius is not { } radius || _sourceRegion is null) return front;
            return OffsetBoundaryCleanup.CompactFirstCommands(front, _boundaryIndex, _sourceRegion,
                delta, radius, cancellationToken, _constructionClipBoundary);
        }

        private List<List<PointD>> CompactFront(List<List<PointD>> front, double delta,
            CancellationToken cancellationToken, bool proposeForShellAudit = false)
        {
            if (delta >= 0 || _constructionSweepRadius is not { } radius) return front;
            // Native offset decomposition cycles are not independent machining
            // boundaries. Recover the exact signed filled front before applying
            // bounded command-chord cleanup against ORIGINAL source walls.
            var components = DifferenceToComponents(front, Array.Empty<IReadOnlyList<PointD>>());
            var canonical = components.SelectMany(component =>
                new[] { component.Outer }.Concat(component.Holes)).ToList();
            cancellationToken.ThrowIfCancellationRequested();
            // This owner caches no compacted acceptance result. Existing filled
            // component evidence still proves the identical literal geometry;
            // proposal mode never changes its key or publishes a stock witness.
            return OffsetBoundaryCleanup.Compact(canonical, _boundaryIndex, delta, radius,
                .003, cancellationToken, _constructionClipBoundary, proposeForShellAudit);
        }

        private PreparedBoundaryOffset OriginalConstruction()
        {
            if (_originalConstruction is not null) return _originalConstruction;
            return _originalConstruction = new PreparedBoundaryOffset(null, null,
                _arcTolerance, _simplifyTolerance, _joinType, 0, null, _cancellationToken, null, true,
                _geometry, null, _twoOriginExperiment, _separateOriginalFirst);
        }
    }

    internal static PreparedBoundaryOffset PrepareBoundaryOffset(
        IReadOnlyList<PointD> outer,
        IEnumerable<IReadOnlyList<PointD>>? holes,
        double arcTolerance = DefaultArcTolerance,
        double simplifyTolerance = PolygonSimplify.DefaultTolerance,
        JoinType joinType = JoinType.Round,
        double sourceConstructionTolerance = 0,
        double? sourceConstructionSweepRadius = null,
        CancellationToken cancellationToken = default,
        IReadOnlyList<PointD>? sourceConstructionClipBoundary = null,
        bool sourceConstructionValidateFirstFootprint = true,
        bool? twoOriginConstruction = null,
        bool separateOriginalFirstConstruction = false)
        => new(outer, holes, arcTolerance, simplifyTolerance, joinType,
            sourceConstructionTolerance, sourceConstructionSweepRadius, cancellationToken,
            sourceConstructionClipBoundary, sourceConstructionValidateFirstFootprint, twoOriginConstruction,
            separateOriginalFirstConstruction);

    internal static bool HasInteriorPointWithClearance(
        IEnumerable<IReadOnlyList<PointD>> region, double radius, CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var canonical = Clipper.Union(ToPaths64(region), FillRule.NonZero);
        return new BoundarySegmentIndex(canonical)
            .HasInteriorPointWithClearance(radius * PathUtils.Scale, cancellationToken);
    }

    private static double ElapsedMilliseconds(long startTimestamp)
        => (Stopwatch.GetTimestamp() - startTimestamp) * 1000.0 / Stopwatch.Frequency;

    private static int RemoveOffsetInversions(
        BoundarySegmentIndex boundaryIndex,
        Paths64 solution,
        double scaledDelta,
        double arcTolerance,
        out bool rejectedSignificantContour,
        out string details)
        => RemoveOffsetInversions(boundaryIndex, solution, scaledDelta, arcTolerance,
            out rejectedSignificantContour, out details, out _);

    private static int RemoveOffsetInversions(
        BoundarySegmentIndex boundaryIndex,
        Paths64 solution,
        double scaledDelta,
        double arcTolerance,
        out bool rejectedSignificantContour,
        out string details,
        out double significantDeficit)
    {
        long t0 = PerfLog.Start();
        double clearanceTolerance = Math.Max(
            4.0,
            Math.Min(0.01, Math.Max(0, arcTolerance) * 0.05) * PathUtils.Scale);
        double requiredClearance = Math.Abs(scaledDelta) - clearanceTolerance;
        double significantArea = 0.01 * PathUtils.Scale * PathUtils.Scale;
        rejectedSignificantContour = false;
        significantDeficit = 0;
        details = string.Empty;
        if (requiredClearance <= 0 || solution.Count == 0)
            return 0;

        double requiredClearanceSquared = requiredClearance * requiredClearance;
        var decisions = new List<string>();
        var significantDecisions = new List<string>();
        int rejected = 0;
        int discardedUnreachableFragments = 0;
        int outputPoints = 0;
        for (int outputIndex = solution.Count - 1; outputIndex >= 0; outputIndex--)
        {
            Path64 contour = solution[outputIndex];
            outputPoints += contour.Count;
            double minimumClearanceSquared = double.MaxValue;
            Point64 minimumClearancePoint = default;
            bool hasBoundaryClearance = contour.Count >= 3;
            for (int pointIndex = 0; pointIndex < contour.Count; pointIndex++)
            {
                double clearanceSquared = boundaryIndex.DistanceSquared(contour[pointIndex]);
                if (clearanceSquared < minimumClearanceSquared)
                {
                    minimumClearanceSquared = clearanceSquared;
                    minimumClearancePoint = contour[pointIndex];
                }
                if (clearanceSquared < requiredClearanceSquared)
                    hasBoundaryClearance = false;
            }

            if (hasBoundaryClearance)
                continue;

            if (decisions.Count < 8)
            {
                decisions.Add(DescribeOffsetDecision(
                    outputIndex, contour, minimumClearanceSquared));
            }
            // The protected side reverses with the offset direction. During erosion,
            // holes protect material and a tiny outer fragment may be discarded. During
            // dilation, the filled region protects material and an unreachable tiny hole
            // may be filled. Rebuilding those seam holes just reproduces quantization
            // artifacts; keeping their boundary would send the cutter into the source.
            // Significant reachable regions still require a whole-region repair.
            double signedArea = Clipper.Area(contour);
            bool protectsMaterial = scaledDelta < 0 ? signedArea < 0 : signedArea > 0;
            if (Math.Abs(signedArea) > significantArea || protectsMaterial)
            {
                rejectedSignificantContour = true;
                significantDeficit = Math.Max(significantDeficit,
                    requiredClearance - Math.Sqrt(minimumClearanceSquared));
                if (significantDecisions.Count < 8)
                    significantDecisions.Add(DescribeOffsetDecision(outputIndex, contour, minimumClearanceSquared)
                        + $",witness=({minimumClearancePoint.X / PathUtils.Scale:R},{minimumClearancePoint.Y / PathUtils.Scale:R})"
                        + $",nearest={boundaryIndex.DescribeNearestSegment(minimumClearancePoint)}"
                        + $",bounds=({contour.Min(p => p.X) / PathUtils.Scale:R},{contour.Min(p => p.Y) / PathUtils.Scale:R})"
                        + $"-({contour.Max(p => p.X) / PathUtils.Scale:R},{contour.Max(p => p.Y) / PathUtils.Scale:R})");
            }
            else
            {
                discardedUnreachableFragments++;
            }
            solution.RemoveAt(outputIndex);
            rejected++;
        }

        details = string.Join(";", significantDecisions.Concat(decisions));
        if (significantDecisions.Count > 0)
            Console.WriteLine($"[PolygonOps.OffsetWitness] delta={scaledDelta / PathUtils.Scale:R} "
                + $"requiredClearance={requiredClearance / PathUtils.Scale:R} {string.Join(";", significantDecisions)}");
        if (rejected > 0)
        {
            PerfLog.Stop("PolygonOps.OffsetInversionCheck", t0,
                $"sourceSegments={boundaryIndex.SegmentCount} outputPoints={outputPoints} " +
                $"delta={scaledDelta / PathUtils.Scale:R} rejected={rejected} " +
                $"discardedUnreachableFragments={discardedUnreachableFragments} " +
                $"requiresRepair={rejectedSignificantContour}");
        }
        return rejected;
    }

    private static string DescribeOffsetDecision(
        int index,
        Path64 contour,
        double minimumClearanceSquared)
    {
        double scaleSquared = PathUtils.Scale * PathUtils.Scale;
        double clearance = minimumClearanceSquared == double.MaxValue
            ? double.NaN
            : Math.Sqrt(minimumClearanceSquared) / PathUtils.Scale;
        return $"reject[{index}]:n={contour.Count}," +
            $"area={Math.Abs(Clipper.Area(contour)) / scaleSquared:F4}," +
            $"signedArea={Clipper.Area(contour) / scaleSquared:F4},clear={clearance:F6}";
    }

    internal static Paths64 RepairConservativeOffset(
        BoundarySegmentIndex boundaryIndex,
        Paths64 sourceRegion,
        Paths64 candidate,
        double scaledDelta,
        double arcTolerance,
        double significantDeficit,
        Paths64? firstInset = null,
        double firstInsetDelta = 0)
    {
        // Repair the entire region before validation removes any contour: dropping an outer
        // loses a whole pocket, while dropping a hole changes which material is protected.
        // The original clearance threshold remains authoritative after every repair.
        double repairArcTolerance = Math.Min(Math.Max(0, arcTolerance), 0.01);
        double reserve = Math.Max(8.0, 2 * repairArcTolerance * PathUtils.Scale);
        double totalMargin = 0;
        int originalCount = candidate.Count;
        double originalArea = candidate.Sum(Clipper.Area) / (PathUtils.Scale * PathUtils.Scale);
        string details = string.Empty;
        // Dense boundary-band union can lose coverage already present in its individual
        // strokes. Repeating the same global union with a larger radius can preserve the
        // artifact. Repair only the proven missing source-segment capsules:
        // subtract their union during erosion (add it during dilation), retaining the
        // rest of the complete candidate, including every hole and reachable component.
        for (int attempt = 1; attempt <= 3; attempt++)
        {
            var repaired = RepairMissingBoundaryCapsules(boundaryIndex, candidate, scaledDelta,
                arcTolerance, repairArcTolerance, out int capsules);
            if (capsules == 0) break;
            candidate = repaired;
            var validated = new Paths64(candidate);
            int rejected = RemoveOffsetInversions(boundaryIndex, validated, scaledDelta, arcTolerance,
                out bool significantRejected, out details, out significantDeficit);
            Console.WriteLine($"[PolygonOps.OffsetRepair] delta={scaledDelta / PathUtils.Scale:R} "
                + $"attempt={attempt} method=source-segment-capsules capsules={capsules} "
                + $"originalContours={originalCount} recoveredContours={validated.Count} rejectedContours={rejected} "
                + $"originalSignedArea={originalArea:F4} retainedSignedArea={validated.Sum(Clipper.Area) / (PathUtils.Scale * PathUtils.Scale):F4} "
                + $"remainingDeficit={significantDeficit / PathUtils.Scale:R} significantRejected={significantRejected}");
            if (!significantRejected) return validated;
        }

        for (int attempt = 1; attempt <= 3; attempt++)
        {
            // A large deficit indicates an inversion or stroke artifact, not arc error.
            // Rebuild from the source instead of shrinking a useful pocket by that deficit.
            bool rebuild = attempt == 3 || significantDeficit > 4 * reserve;
            double margin = (rebuild ? 0 : Math.Max(0, significantDeficit)) + reserve;
            totalMargin += margin;
            string method;
            if (!rebuild)
            {
                var repair = CreateOffsetter(repairArcTolerance);
                repair.AddPaths(candidate, JoinType.Round, EndType.Polygon);
                var repaired = new Paths64();
                repair.Execute(Math.Sign(scaledDelta) * margin, repaired);
                candidate = repaired;
                method = "region-offset";
            }
            else
            {
                // The first validated inset has already removed source-scale teeth
                // and merged nearby islands. Erode that fixed region by the remaining
                // distance instead of restroking every original edge at the full radius.
                // Always validate against the original boundary; retain the original
                // rebuild for the final recovery attempt. Never chain previous rings.
                bool reuseInset = attempt < 3 && firstInset != null
                    && firstInsetDelta < 0 && scaledDelta < firstInsetDelta;
                candidate = BuildConservativeOffset(reuseInset ? firstInset! : sourceRegion,
                    scaledDelta - (reuseInset ? firstInsetDelta : 0) + Math.Sign(scaledDelta) * totalMargin,
                    repairArcTolerance, sourceIsCanonical: true);
                method = reuseInset ? "first-inset-boundary-rebuild" : "boundary-rebuild";
            }

            var validated = new Paths64(candidate);
            int rejected = RemoveOffsetInversions(boundaryIndex, validated, scaledDelta, arcTolerance,
                out bool significantRejected, out details, out significantDeficit);
            double retainedArea = validated.Sum(Clipper.Area) / (PathUtils.Scale * PathUtils.Scale);
            Console.WriteLine($"[PolygonOps.OffsetRepair] delta={scaledDelta / PathUtils.Scale:R} " +
                $"attempt={attempt} method={method} margin={margin / PathUtils.Scale:R} " +
                $"totalMargin={totalMargin / PathUtils.Scale:R} originalContours={originalCount} " +
                $"recoveredContours={validated.Count} rejectedContours={rejected} " +
                $"originalSignedArea={originalArea:F4} retainedSignedArea={retainedArea:F4} " +
                $"remainingDeficit={significantDeficit / PathUtils.Scale:R} significantRejected={significantRejected}");
            if (!significantRejected)
                return validated;
        }

        throw new InvalidOperationException(
            $"Polygon offset could not preserve cutter clearance after three conservative repairs " +
            $"(offset {scaledDelta / PathUtils.Scale:F6} mm, deficit {significantDeficit / PathUtils.Scale:F6} mm). {details}");
    }

    private static Paths64 RepairMissingBoundaryCapsules(BoundarySegmentIndex boundaryIndex,
        Paths64 candidate, double scaledDelta, double arcTolerance, double repairArcTolerance,
        out int capsuleCount)
    {
        double clearanceTolerance = Math.Max(4.0,
            Math.Min(0.01, Math.Max(0, arcTolerance) * 0.05) * PathUtils.Scale);
        double required = Math.Abs(scaledDelta) - clearanceTolerance;
        double requiredSquared = required * required;
        var segments = new HashSet<(long X0, long Y0, long X1, long Y1)>();
        foreach (var contour in candidate)
        foreach (var point in contour)
        {
            if (boundaryIndex.DistanceSquared(point) >= requiredSquared) continue;
            var segment = boundaryIndex.NearestSegment(point);
            segments.Add((segment.Start.X, segment.Start.Y, segment.End.X, segment.End.Y));
        }

        capsuleCount = segments.Count;
        if (capsuleCount == 0) return candidate;
        double radius = Math.Abs(scaledDelta) + Math.Max(8.0, 2 * repairArcTolerance * PathUtils.Scale);
        var capsules = new Paths64();
        foreach (var segment in segments)
        {
            var offsetter = CreateOffsetter(repairArcTolerance);
            offsetter.AddPath(new Path64 { new(segment.X0, segment.Y0), new(segment.X1, segment.Y1) },
                JoinType.Round, EndType.Round);
            var capsule = new Paths64();
            offsetter.Execute(radius, capsule);
            capsules.AddRange(capsule);
        }
        var band = Clipper.Union(capsules, FillRule.NonZero);
        return scaledDelta < 0
            ? Clipper.Difference(candidate, band, FillRule.NonZero)
            : Clipper.Union(new Paths64(candidate.Concat(band)), FillRule.NonZero);
    }

    private static Paths64 KeepInsetComponents(Paths64 source, double scaledDelta,
        double? feasibilityResolution = null)
    {
        // A component narrower than the inset diameter cannot contain a cutter center.
        // Protected holes are retained with their outer component; nested islands are independent.
        var tree = new PolyTree64();
        var topology = new Clipper64(); topology.AddSubject(source);
        topology.Execute(ClipType.Union, FillRule.NonZero, tree);
        var eligible = new Paths64();
        void Collect(PolyPath64 outer)
        {
            if (outer.Polygon is { Count: >= 3 } ring
                && ring.Max(p => p.X) - ring.Min(p => p.X) >= 2 * Math.Abs(scaledDelta)
                && ring.Max(p => p.Y) - ring.Min(p => p.Y) >= 2 * Math.Abs(scaledDelta))
            {
                var component = new Paths64 { ring };
                foreach (PolyPath64 hole in outer)
                    if (hole.Polygon != null) component.Add(hole.Polygon);
                if (feasibilityResolution == null || new BoundarySegmentIndex(component)
                    .MayContainDisc(Math.Abs(scaledDelta), feasibilityResolution.Value))
                    eligible.AddRange(component);
            }
            foreach (PolyPath64 hole in outer)
                foreach (PolyPath64 island in hole) Collect(island);
        }
        foreach (PolyPath64 outer in tree) Collect(outer);
        return eligible;
    }

    // Untrusted outer-offset aid: callers must prove its complete enclosure.
    internal static Paths64 BuildOuterCandidateForCertificate(Paths64 canonicalSource, double radius)
        => BuildConservativeOffset(canonicalSource, radius * PathUtils.Scale,
            1 / PathUtils.Scale, sourceIsCanonical: true);

    // An untrusted aid only. The caller must independently establish complete
    // original-site center support, physical enclosure and accepted-cut coverage.
    internal static Paths64 BuildOffsetCandidateForCertificate(Paths64 canonicalSource,
        double radius, double arcTolerance)
        => BuildConservativeOffset(canonicalSource, radius * PathUtils.Scale,
            arcTolerance, sourceIsCanonical: true);

    private static Paths64 BuildConservativeOffset(
        Paths64 source,
        double scaledDelta,
        double arcTolerance,
        bool sourceIsCanonical = false)
    {
        long t0 = PerfLog.Start();
        Paths64 sourceRegion = sourceIsCanonical
            ? source
            : Clipper.Union(source, FillRule.NonZero);
        if (sourceRegion.Count == 0 || Math.Abs(scaledDelta) <= 4.0)
            return sourceRegion;

        if (scaledDelta < 0 && sourceRegion.Sum(path => path.Count) > 8192)
        {
            sourceRegion = KeepInsetComponents(sourceRegion, scaledDelta);
            if (sourceRegion.Count == 0) return sourceRegion;
        }

        double fallbackArcTolerance = Math.Min(arcTolerance, 0.01);
        int sourcePoints = sourceRegion.Where(path => path.Count >= 2).Sum(path => path.Count), strokeChunks = 0;
        // Chunk seams have two independently tessellated caps; reserve both arc errors.
        double safetyMargin = Math.Max(4.0, 2 * fallbackArcTolerance * PathUtils.Scale);
        var boundaryBand = new Paths64();
        var chunks = new List<(Path64 Boundary, int Start, int EdgeCount)>();
        foreach (Path64 boundary in sourceRegion)
        {
            if (boundary.Count < 2)
                continue;
            // A round boundary stroke is the union of its segment capsules. Always split
            // a closed boundary into at least two genuinely open strokes: a Joined stroke
            // can invert its inner loop when its radius exceeds the enclosed feature.
            // Round caps cover every shared endpoint and the explicit closing segment.
            int edgesPerChunk = Math.Min(256, Math.Max(1, boundary.Count / 2));
            for (int start = 0; start < boundary.Count; start += edgesPerChunk)
            {
                int edgeCount = Math.Min(edgesPerChunk, boundary.Count - start);
                chunks.Add((boundary, start, edgeCount));
            }
        }
        var bands = new Paths64[chunks.Count];
        long strokeStarted = System.Diagnostics.Stopwatch.GetTimestamp();
        void StrokeChunk(int i)
        {
            var sourceChunk = chunks[i];
            var chunk = new Path64(sourceChunk.EdgeCount + 1);
            for (int edge = 0; edge <= sourceChunk.EdgeCount; edge++)
                chunk.Add(sourceChunk.Boundary[(sourceChunk.Start + edge) % sourceChunk.Boundary.Count]);
            var offsetter = CreateOffsetter(fallbackArcTolerance);
            offsetter.AddPath(chunk, JoinType.Round, EndType.Round);
            var band = new Paths64();
            offsetter.Execute(Math.Abs(scaledDelta) + safetyMargin, band);
            bands[i] = band;
        }
        if (chunks.Count <= 4)
            for (int i = 0; i < chunks.Count; i++) StrokeChunk(i);
        else
            Parallel.For(0, chunks.Count, new ParallelOptions {
                MaxDegreeOfParallelism = GeometryParallelism.WorkerLimit }, StrokeChunk);
        foreach (var band in bands) boundaryBand.AddRange(band);
        strokeChunks = chunks.Count;
        double strokeMs = System.Diagnostics.Stopwatch.GetElapsedTime(strokeStarted).TotalMilliseconds;
        if (boundaryBand.Count == 0)
            return new Paths64();

        long unionStarted = System.Diagnostics.Stopwatch.GetTimestamp();
        int unionBatches = 0;
        // Nearby stroke chunks overlap heavily. Collapse those overlaps locally
        // before the final union instead of intersecting thousands of duplicate
        // caps in one sweep. Each batch is a canonical union of complete bands,
        // including their holes; the original boundary still validates the inset.
        var groups = bands;
        while (groups.Length > 64)
        {
            const int batchSize = 32;
            var sourceGroups = groups;
            var combined = new Paths64[(sourceGroups.Length + batchSize - 1) / batchSize];
            Parallel.For(0, combined.Length, new ParallelOptions {
                MaxDegreeOfParallelism = GeometryParallelism.WorkerLimit }, batch =>
            {
                var paths = new Paths64();
                for (int i = batch * batchSize; i < Math.Min(sourceGroups.Length, (batch + 1) * batchSize); i++)
                    paths.AddRange(sourceGroups[i]);
                combined[batch] = Clipper.Union(paths, FillRule.NonZero);
            });
            unionBatches += combined.Length;
            groups = combined;
        }
        if (unionBatches > 0)
        {
            boundaryBand = new Paths64();
            foreach (var group in groups) boundaryBand.AddRange(group);
        }
        boundaryBand = Clipper.Union(boundaryBand, FillRule.NonZero);
        double unionMs = System.Diagnostics.Stopwatch.GetElapsedTime(unionStarted).TotalMilliseconds;

        Paths64 result;
        if (scaledDelta < 0)
        {
            result = Clipper.Difference(sourceRegion, boundaryBand, FillRule.NonZero);
        }
        else
        {
            var expanded = new Paths64();
            expanded.AddRange(sourceRegion);
            expanded.AddRange(boundaryBand);
            result = Clipper.Union(expanded, FillRule.NonZero);
        }

        PerfLog.Stop("PolygonOps.BuildConservativeOffset", t0,
            $"sourcePaths={sourceRegion.Count} sourcePoints={sourcePoints} " +
            $"strokeChunks={strokeChunks} bandPaths={boundaryBand.Count} resultPaths={result.Count} " +
            $"strokeMs={strokeMs:F2} unionMs={unionMs:F2} unionBatches={unionBatches}");
        return result;
    }

    // --- Offsetting -----------------------------------------------------------

    /// <summary>
    /// Cutter-center erosion with a complete-chord proof against the original
    /// source. Its reserve covers integer geometry, float command coordinates,
    /// and round-join chords; it is independent of stock and contour size.
    /// </summary>
    /// <param name="preferOriginalFirstConstruction">Opt into the existing
    /// complete-original first-front owner and its certified hole proposal.
    /// Uncertain proposals retain original boundary construction; caller-supplied
    /// candidates and the final actual-command proof keep their existing policy.</param>
    internal static List<List<PointD>> CutterInset(
        IEnumerable<IReadOnlyList<PointD>> polygons, double radius,
        IEnumerable<IReadOnlyList<PointD>>? candidateRings = null,
        CancellationToken cancellationToken = default,
        bool preferOriginalFirstConstruction = false, bool useOriginalHoleErosion = true)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var source = polygons.ToList();
        if (source.Count == 0) return new();
        double quantum = 1 / PathUtils.Scale;
        double coordinate = source.SelectMany(ring => ring)
            .Select(point => Math.Max(Math.Abs(point.x), Math.Abs(point.y))).DefaultIfEmpty().Max();
        float commandCoordinate = (float)coordinate;
        double commandError = Math.Sqrt(2) * (MathF.BitIncrement(commandCoordinate) - commandCoordinate) / 2;
        // F3 XY can move both coordinates by half a millimetre thousandth;
        // translation to the machine origin can change the rounding direction.
        double machineXYError = Math.Sqrt(2) * 0.0005;
        double reserve = Math.Max(8 * quantum, commandError + machineXYError + 6 * quantum);
        var sourceRegion = Clipper.Union(ToPaths64(source), FillRule.NonZero);
        var index = new BoundarySegmentIndex(sourceRegion);
        Paths64 candidate;
        if (candidateRings is not null)
        {
            // A caller-supplied finite-band core owns its authored cutoff.
            candidate = ToPaths64(candidateRings);
        }
        else if (radius > 0)
        {
            // A compensated layer becomes the exact zero front of V clearing.
            // Build that layer through the same original-source certified
            // prepared owner, then retain the stronger actual-command proof below.
            var components = DifferenceToComponents(FromPaths64(sourceRegion), Array.Empty<IReadOnlyList<PointD>>());
            int originalFirstComponents = 0, holeErosionComponents = 0;
            var fronts = components.SelectMany(component =>
            {
                var prepared = PrepareBoundaryOffset(component.Outer, component.Holes,
                    arcTolerance: quantum, simplifyTolerance: 0,
                    sourceConstructionTolerance: .001,
                    sourceConstructionSweepRadius: radius, cancellationToken: cancellationToken,
                    separateOriginalFirstConstruction: preferOriginalFirstConstruction && useOriginalHoleErosion);
                // This existing owner retains complete original-site admission,
                // compaction and physical proofs. An uncertain hole proposal
                // falls back to the same complete original boundary bands.
                // Small fronts keep their inexpensive established selection.
                // The operation's direct-original mode targets dense work;
                // the separately explicit hole experiment retains its scope.
                bool selectOriginalFirst = preferOriginalFirstConstruction
                    && (useOriginalHoleErosion || prepared.SourcePointCount > 8192);
                var front = selectOriginalFirst
                    ? prepared.ExecuteOriginalFirstAuthority(-(radius + reserve), cancellationToken)
                    : prepared.Execute(-(radius + reserve), cancellationToken);
                if (prepared.OriginalFirstAuthorityCount > 0) originalFirstComponents++;
                if (prepared.OriginalHoleErosionCount > 0) holeErosionComponents++;
                return front;
            }).ToList();
            if (preferOriginalFirstConstruction)
                Console.WriteLine($"[PolygonOps.CutterInsetConstruction] preferredOriginalFirst=True radius={radius:R} "
                    + $"holeErosionRequested={useOriginalHoleErosion} "
                    + $"components={components.Count} originalFirstComponents={originalFirstComponents} "
                    + $"holeErosionComponents={holeErosionComponents} "
                    + $"ordinaryFallbackComponents={components.Count - originalFirstComponents} "
                    + $"originalBoundaryComponents={originalFirstComponents - holeErosionComponents} "
                    + $"holeProposalRefusalComponents={(useOriginalHoleErosion ? originalFirstComponents - holeErosionComponents : 0)} "
                    + $"completeOriginalSource=True physicalCleanup=unchanged actualCommandProof=required");
            candidate = Clipper.Union(ToPaths64(fronts), FillRule.NonZero);
        }
        else
        {
            candidate = ToPaths64(Offset(source, -(radius + reserve), arcTolerance: quantum, simplifyTolerance: 0));
        }
        double required = (radius + machineXYError) * PathUtils.Scale + 2;

        for (int attempt = 0; attempt <= 3; attempt++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var missing = new HashSet<(Point64 Start, Point64 End)>();
            int violatingChords = 0;
            foreach (Path64 ring in candidate)
            for (int end = 0, start = ring.Count - 1; end < ring.Count; start = end++)
            {
                if ((end & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
                // The Shared emitter stores XY as float. Prove that command,
                // rather than just the double vertices returned by Clipper.
                Point64 Command(Point64 point) => new(
                    (long)Math.Round((float)(point.X / PathUtils.Scale) * PathUtils.Scale),
                    (long)Math.Round((float)(point.Y / PathUtils.Scale) * PathUtils.Scale));
                Point64 a = Command(ring[start]), b = Command(ring[end]);
                if (!index.IsInside(a))
                    throw new InvalidOperationException($"Cutter inset left its original region at ({a.X / PathUtils.Scale:R},{a.Y / PathUtils.Scale:R}).");
                if (index.ClearanceViolation(a, b, required) is not { } edge) continue;
                missing.Add(edge);
                violatingChords++;
            }
            if (missing.Count == 0)
            {
                Console.WriteLine($"[PolygonOps.CutterInset] radius={radius:R} reserve={reserve:R} "
                    + $"contours={candidate.Count} chords={candidate.Sum(ring => ring.Count)} "
                    + $"repairs={attempt} completeCommandChordProof=True");
                return FromPaths64(candidate);
            }
            var witness = missing.First();
            Console.WriteLine($"[PolygonOps.CutterInsetRepair] radius={radius:R} attempt={attempt + 1} "
                + $"violatingChords={violatingChords} capsules={missing.Count} "
                + $"source=({witness.Start.X / PathUtils.Scale:R},{witness.Start.Y / PathUtils.Scale:R})->"
                + $"({witness.End.X / PathUtils.Scale:R},{witness.End.Y / PathUtils.Scale:R})");
            if (attempt == 3)
                throw new InvalidOperationException("Cutter inset could not prove complete commanded chord clearance after three source-boundary repairs.");
            var capsules = new Paths64();
            foreach (var edge in missing)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var offsetter = CreateOffsetter(quantum);
                offsetter.AddPath(new Path64 { edge.Start, edge.End }, JoinType.Round, EndType.Round);
                var capsule = new Paths64();
                offsetter.Execute((radius + reserve) * PathUtils.Scale, capsule);
                capsules.AddRange(capsule);
            }
            // Repair the complete region, retaining its holes and connected rooms.
            // Removing a failing contour would silently discard reachable material.
            candidate = Clipper.Difference(candidate, Clipper.Union(capsules, FillRule.NonZero), FillRule.NonZero);
            cancellationToken.ThrowIfCancellationRequested();
        }
        throw new InvalidOperationException("Unreachable cutter inset repair state.");
    }

    /// <summary>
    /// Offsets a polygon-with-holes group by <paramref name="delta"/>.
    /// All polygons are added as one group so Clipper2 resolves outer/hole interaction
    /// in a single pass; winding must already be correct (outers CCW, holes CW).
    /// </summary>
    /// <param name="polygons">Pre-tessellated rings (outers CCW, holes CW).</param>
    /// <param name="delta">Negative shrinks inward, positive grows outward.</param>
    /// <param name="arcTolerance">Round-join chord tolerance in workspace units.</param>
    /// <param name="simplifyTolerance">RDP tolerance; pass 0 to skip simplification.</param>
    /// <param name="joinType">Corner treatment; round matches a cylindrical cutter.</param>
    public static List<List<PointD>> Offset(
        IEnumerable<IReadOnlyList<PointD>> polygons,
        double delta,
        double arcTolerance = DefaultArcTolerance,
        double simplifyTolerance = PolygonSimplify.DefaultTolerance,
        JoinType joinType = JoinType.Round,
        bool pruneCollapsedHoles = false)
    {
        var group = ToPaths64(polygons);
        if (group.Count == 0)
            return new List<List<PointD>>();


        double scaledDelta = delta * PathUtils.Scale;
        var originalGroup = group;
        if (pruneCollapsedHoles && scaledDelta > 0 && joinType == JoinType.Round
            && group.Sum(path => (long)path.Count) >= 4096 && group.Any(path => Clipper.Area(path) > 0))
        {
            // A positive filled-region offset erodes each clockwise hole. If
            // its complete signed-distance cover cannot contain even the smaller
            // reserved disk, that hole has no surviving offset operand. Keep all
            // original edges for the ordinary inversion/repair authority below.
            long pruneStarted = Stopwatch.GetTimestamp();
            double guard = 4 * Math.Max(0, arcTolerance) * PathUtils.Scale + 64;
            double required = scaledDelta - guard;
            int omitted = 0; long omittedPoints = 0;
            if (required > 0)
            {
                var topology = new Clipper64();
                var tree = new PolyTree64();
                topology.AddSubject(originalGroup);
                topology.Execute(ClipType.Union, FillRule.NonZero, tree);
                var holes = new List<Path64>();
                void CollectHoles(PolyPath64 node, bool isHole)
                {
                    if (isHole && node.Polygon is { } polygon) holes.Add(polygon);
                    foreach (PolyPath64 child in node) CollectHoles(child, !isHole);
                }
                CollectHoles(tree, true);
                static bool SameCycle(Path64 a, Path64 b)
                {
                    if (a.Count != b.Count) return false;
                    for (int start = 0; start < b.Count; start++)
                    {
                        if (a[0] != b[start]) continue;
                        bool same = true;
                        for (int i = 1; i < a.Count && same; i++) same = a[i] == b[(start + i) % b.Count];
                        if (same) return true;
                    }
                    return false;
                }
                var retained = new Paths64(group.Count);
                foreach (var path in group)
                {
                    bool collapsed = Clipper.Area(path) < 0 && path.Count >= 3
                        && holes.Any(hole => SameCycle(path, hole))
                        && !new BoundarySegmentIndex(new Paths64 { path }).MayContainDisc(required, guard);
                    if (collapsed) { omitted++; omittedPoints += path.Count; }
                    else retained.Add(path);
                }
                group = retained;
            }
            Console.WriteLine($"[PolygonOps.OffsetCollapsedHoles] omittedHoles={omitted} omittedPoints={omittedPoints} "
                +$"originalPoints={originalGroup.Sum(path=>(long)path.Count)} retainedPoints={group.Sum(path=>(long)path.Count)} "
                +$"delta={delta:R} radiusReserve={guard/PathUtils.Scale:R} elapsedMs={Stopwatch.GetElapsedTime(pruneStarted).TotalMilliseconds:F1} "
                +$"proof=complete-signed-distance-cover unresolvedRetainsLiteralHole=True originalValidationAndRepairEdgesRetained=True");
        }
        if (scaledDelta < 0)
        {
            // A collapsed source component can produce an inverted positive-area offset.
            // Exclude it before offsetting so its rejection never rebuilds a valid neighbor.
            group = KeepInsetComponents(group, scaledDelta, arcTolerance * PathUtils.Scale);
            originalGroup = group;
            if (group.Count == 0)
                return new List<List<PointD>>();
        }

        var offsetter = CreateOffsetter(arcTolerance);
        offsetter.AddPaths(group, joinType, EndType.Polygon);

        var solution = new Paths64();
        offsetter.Execute(scaledDelta, solution);

        int rawCount = solution.Count;
        long validationStart = PerfLog.Start();
        int rejected = RemoveOffsetInversions(
            new BoundarySegmentIndex(originalGroup), solution, scaledDelta, arcTolerance,
            out bool rejectedSignificantContour, out string details);
        int fallbackCount = 0;
        int fallbackRejected = 0;
        if (rejected > 0 && (solution.Count == 0 || rejectedSignificantContour))
        {
            solution = BuildConservativeOffset(originalGroup, scaledDelta, arcTolerance);
            fallbackCount = solution.Count;
            var rawCandidate = new Paths64(solution);
            fallbackRejected = RemoveOffsetInversions(
                new BoundarySegmentIndex(originalGroup),
                solution,
                scaledDelta,
                arcTolerance,
                out bool fallbackSignificant,
                out string fallbackDetails,
                out double significantDeficit);
            if (fallbackRejected > 0)
                details += $";fallbackRejected={fallbackRejected}:{fallbackDetails}";
            if (fallbackSignificant)
                solution = RepairConservativeOffset(new BoundarySegmentIndex(originalGroup),
                    Clipper.Union(originalGroup, FillRule.NonZero), rawCandidate, scaledDelta, arcTolerance, significantDeficit);
        }

        if (rejected > 0)
        {
            PerfLog.Stop(
                "PolygonOps.OffsetValidation",
                validationStart,
                $"delta={delta:F6} source={group.Count} raw={rawCount} " +
                $"rejected={rejected} fallback={fallbackCount} " +
                $"fallbackRejected={fallbackRejected} " +
                $"kept={solution.Count} {details}");
        }

        var output = simplifyTolerance > 0
            ? PolygonSimplify.RdpAll(FromPaths64(solution), simplifyTolerance)
            : FromPaths64(solution);

        NormalizeOrder(output);
        return output;
    }

    /// <summary>
    /// Offsets an outer boundary and its holes, forcing correct winding first.
    /// Use when the caller cannot guarantee the input orientation.
    /// </summary>
    public static List<List<PointD>> OffsetBoundary(
        IReadOnlyList<PointD> outer,
        IEnumerable<IReadOnlyList<PointD>>? holes,
        double delta,
        double arcTolerance = DefaultArcTolerance,
        double simplifyTolerance = PolygonSimplify.DefaultTolerance,
        JoinType joinType = JoinType.Round)
    {
        return PrepareBoundaryOffset(
            outer,
            holes,
            arcTolerance,
            simplifyTolerance,
            joinType).Execute(delta);
    }

    /// <summary>
    /// Expands filled polygons outward by <paramref name="radius"/> (Minkowski sum with a disk).
    /// </summary>
    public static List<List<PointD>> Buffer(
        IEnumerable<IReadOnlyList<PointD>> polygons,
        double radius,
        double arcTolerance = DefaultArcTolerance)
    {
        if(radius<.0001)return new();
        var source=polygons.ToList();
        if(!FilledFrontEvidenceCache.Enabled || source.Sum(p=>(long)p.Count)<=256)
            return BufferCore(source,radius,arcTolerance,forceBoundaryCapsules:false);
        long started=Stopwatch.GetTimestamp();
        var key=FilledFrontEvidenceCache.Key.Capture(source,radius,arcTolerance,0,12,null,default);
        var reused=FilledDilationCache.Find(key,default);
        if(reused is not null)
        {
            var copy=reused.Copy();
            Console.WriteLine($"[PolygonOps.FilledDilationEvidence] hit=True inputPoints={key!.Points} "
                +$"outputPoints={copy.Sum(p=>p.Count)} elapsedMs={ElapsedMilliseconds(started):F2} "
                +$"identity=complete-ordered-source-radius-arc resultOwnership=fresh-copy byteLimit=134217728");
            return copy;
        }
        var output=BufferCore(source,radius,arcTolerance,forceBoundaryCapsules:false);
        if(key is not null)
        {
            var evidence=new CenterlineBufferEvidence(output);
            FilledDilationCache.Publish(key,evidence,evidence.Bytes,default);
        }
        Console.WriteLine($"[PolygonOps.FilledDilationEvidence] hit=False inputPoints={source.Sum(p=>p.Count)} "
            +$"outputPoints={output.Sum(p=>p.Count)} elapsedMs={ElapsedMilliseconds(started):F2} "
            +$"identity=complete-ordered-source-radius-arc construction=unchanged byteLimit=134217728");
        return output;
    }

    internal static List<List<PointD>> BufferWithBoundaryCapsules(
        IEnumerable<IReadOnlyList<PointD>> polygons, double radius, double arcTolerance)
        => BufferCore(polygons, radius, arcTolerance, forceBoundaryCapsules:true);

    internal static List<List<PointD>> BufferWithLiteralSignedOffset(
        IEnumerable<IReadOnlyList<PointD>> polygons,double radius,double arcTolerance)
        => BufferCore(polygons,radius,arcTolerance,forceBoundaryCapsules:false,separateSignedClusters:false);

    private static List<List<PointD>> BufferCore(
        IEnumerable<IReadOnlyList<PointD>> polygons, double radius, double arcTolerance,
        bool forceBoundaryCapsules,bool separateSignedClusters=true)
    {
        if (radius < 0.0001)
            return new List<List<PointD>>();

        var group = ToPaths64(polygons);
        if (group.Count == 0)
            return new List<List<PointD>>();

        long inputPoints = group.Sum(path=>(long)path.Count);
        // Tiny clearance shells do not create the large overlapping join
        // arrangements which require chunking. Use the existing literal signed
        // round offset at its original radius/tolerance, retaining holes and all
        // source vertices. Larger physical sweeps keep their bounded builder.
        bool tinyShell = radius * PathUtils.Scale <= 256;
        if(inputPoints>256 && (!tinyShell || forceBoundaryCapsules))
        {
            // Positive dilation is the original signed fill plus the disk sweep
            // of every closed boundary (including holes). Bound each join
            // arrangement just as for executed open strokes; keep the interior
            // fill so this never turns a filled trial sweep into an annulus.
            long started=Stopwatch.GetTimestamp();
            var source=FromPaths64(group);
            var closed=source.Where(path=>path.Count>1).Select(path=>(IReadOnlyList<PointD>)
                (path[0]==path[^1]?path:path.Concat(new[]{path[0]}).ToList()));
            var bands=BufferOpenCenterlines(closed,radius,arcTolerance);
            var result=Union(source.Concat(bands));
            Console.WriteLine($"[PolygonOps.FilledDilation] sourcePaths={source.Count} sourcePoints={group.Sum(p=>p.Count)} "
                + $"resultPoints={result.Sum(p=>p.Count)} elapsedMs={ElapsedMilliseconds(started):F1} "
                + $"construction=signed-fill-plus-closed-boundary-capsules holesPreserved=True");
            return result;
        }

        long offsetStarted = Stopwatch.GetTimestamp();
        if(tinyShell && !forceBoundaryCapsules && separateSignedClusters && inputPoints>=8192
            && TryBufferSeparatedSignedClusters(group,radius,arcTolerance,out var separated))
        {
            var partitioned=FromPaths64(separated);NormalizeOrder(partitioned);return partitioned;
        }
        var offsetter = CreateOffsetter(arcTolerance);
        // An outer and its signed holes form one filled group. Adding each
        // contour separately makes a clockwise hole a reversed outer group.
        offsetter.AddPaths(group, JoinType.Round, EndType.Polygon);

        var solution = new Paths64();
        offsetter.Execute(radius * PathUtils.Scale, solution);

        // Execute already performs the offset cleanup union with collinear
        // preservation disabled. Its normally oriented result is canonical;
        // do not rebuild that same arrangement a second time. A reversed-only
        // input still uses the original NonZero orientation normalization.
        var largest=solution.MaxBy(path=>Math.Abs(Clipper.Area(path)));
        var canonical=forceBoundaryCapsules || largest is not null && !Clipper.IsPositive(largest)
            ? Clipper.Union(solution,FillRule.NonZero) : solution;
        var output = FromPaths64(canonical);
        NormalizeOrder(output);
        if (inputPoints > 256)
            Console.WriteLine($"[PolygonOps.FilledDilation] sourcePaths={group.Count} sourcePoints={inputPoints} "
                +$"resultPoints={output.Sum(p=>p.Count)} elapsedMs={ElapsedMilliseconds(offsetStarted):F1} "
                +$"radius={radius:R} arcTolerance={arcTolerance:R} maximumShellTicks=256 "
                +$"construction=literal-signed-round-offset sourceVerticesUnchanged=True holesPreserved=True");
        return output;
    }

    /// <summary>
    /// Buffers polylines as centrelines rather than filled regions: the swept area of a
    /// round cutter following the path. A closed centreline yields an annular band of
    /// width 2*<paramref name="radius"/> rather than a filled disc.
    /// </summary>
    public static List<List<PointD>> BufferCenterlines(
        IEnumerable<IReadOnlyList<PointD>> centrelines,
        double radius,
        double arcTolerance = DefaultArcTolerance)
    {
        if (radius < 0.0001)
            return new List<List<PointD>>();

        var offsetter = CreateOffsetter(arcTolerance);
        int added = 0;
        foreach (var line in centrelines)
        {
            if (line.Count < 2)
                continue;
            offsetter.AddPath(ToPath64(line), JoinType.Round, EndType.Joined);
            added++;
        }

        if (added == 0)
            return new List<List<PointD>>();

        var solution = new Paths64();
        offsetter.Execute(radius * PathUtils.Scale, solution);

        // Union so bands from neighbouring passes merge and enclosed voids invert winding.
        return FromPaths64(Clipper.Union(solution, FillRule.NonZero));
    }

    /// <summary>
    /// Buffers open polylines with round caps. This is used for swept-area accounting where a
    /// closed-looking source path may still represent an open cutter trace.
    /// </summary>
    public static List<List<PointD>> BufferOpenCenterlines(
        IEnumerable<IReadOnlyList<PointD>> centrelines,
        double radius,
        double arcTolerance = DefaultArcTolerance,
        CancellationToken cancellationToken = default)
    {
        if (radius < 0.0001)
            return new List<List<PointD>>();

        var lines = centrelines.ToList();
        // Dense, nearly coincident round joins can make one ClipperOffset
        // arrangement quadratic well below 8,192 vertices. A round-capped
        // polyline is the union of its segment capsules: overlapping chunks
        // retain exactly that swept geometry, including the shared vertex disk.
        const int maximumChunkChords = 32;
        long inputPoints = lines.Sum(line => (long)line.Count);
        if (inputPoints > 256)
        {
            long started = Stopwatch.GetTimestamp();
            var bands = new Paths64();
            var pendingBands = new Paths64();
            double batchUnionMs=0;
            int chunks = 0;
            bool parallelBatches=inputPoints>=8192&&GeometryParallelism.WorkerLimit>1;
            if(parallelBatches)
            {
                // Exactly the original 32-chunk operand groups and order. Only
                // independent group construction runs concurrently; final union
                // sees the same integer polygons in the same batch order.
                var work=new List<(IReadOnlyList<PointD> Line,int Start)>();
                foreach(var line in lines)
                    for(int at=0;at<line.Count-1;at+=maximumChunkChords)work.Add((line,at));
                chunks=work.Count;
                var completed=new Paths64[(chunks+31)/32];long unionTicks=0;
                Parallel.For(0,completed.Length,new ParallelOptions{
                    MaxDegreeOfParallelism=GeometryParallelism.WorkerLimit,CancellationToken=cancellationToken},batch=>
                {
                    var input=new Paths64();
                    for(int i=batch*32;i<Math.Min(chunks,(batch+1)*32);i++)
                    {
                        cancellationToken.ThrowIfCancellationRequested();
                        var item=work[i];var chunk=new List<PointD>(Math.Min(maximumChunkChords+1,item.Line.Count-item.Start));
                        for(int p=item.Start;p<Math.Min(item.Start+maximumChunkChords+1,item.Line.Count);p++)chunk.Add(item.Line[p]);
                        var offset=CreateOffsetter(arcTolerance);offset.AddPath(ToPath64(chunk),JoinType.Round,EndType.Round);
                        var band=new Paths64();offset.Execute(radius*PathUtils.Scale,band);input.AddRange(band);
                    }
                    long unionStart=Stopwatch.GetTimestamp();
                    completed[batch]=Clipper.Union(input,FillRule.NonZero);
                    Interlocked.Add(ref unionTicks,Stopwatch.GetTimestamp()-unionStart);
                });
                foreach(var batch in completed)bands.AddRange(batch);
                batchUnionMs=unionTicks*1000d/Stopwatch.Frequency;
            }
            void FlushBands()
            {
                cancellationToken.ThrowIfCancellationRequested();
                long batchStarted=Stopwatch.GetTimestamp();
                bands.AddRange(Clipper.Union(pendingBands,FillRule.NonZero));pendingBands.Clear();
                batchUnionMs+=ElapsedMilliseconds(batchStarted);
            }
            if(!parallelBatches)foreach (var line in lines)
            for (int start = 0; start < line.Count - 1; start += maximumChunkChords)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var chunk = new List<PointD>(Math.Min(maximumChunkChords + 1, line.Count - start));
                for (int i = start; i < Math.Min(start + maximumChunkChords + 1, line.Count); i++) chunk.Add(line[i]);
                var chunkOffset = CreateOffsetter(arcTolerance);
                chunkOffset.AddPath(ToPath64(chunk), JoinType.Round, EndType.Round);
                var band = new Paths64();
                chunkOffset.Execute(radius * PathUtils.Scale, band);
                pendingBands.AddRange(band); chunks++;
                if(chunks%32==0) FlushBands();
            }
            if(pendingBands.Count>0) FlushBands();
            cancellationToken.ThrowIfCancellationRequested();
            long unionStarted=Stopwatch.GetTimestamp();
            var buffered = FromPaths64(Clipper.Union(bands, FillRule.NonZero));
            double finalUnionMs=ElapsedMilliseconds(unionStarted);
            NormalizeOrder(buffered);
            Console.WriteLine($"[PolygonOps.OpenStroke] inputPaths={lines.Count} inputPoints={inputPoints} chunks={chunks} "
                + $"maximumChunkChords={maximumChunkChords} points={buffered.Sum(p => p.Count)} elapsedMs={ElapsedMilliseconds(started):F1} "
                + $"batchChunks=32 batchUnionMs={batchUnionMs:F1} finalUnionMs={finalUnionMs:F1} "
                + $"workers={(parallelBatches?GeometryParallelism.WorkerLimit:1)} batchUnionTiming=sum-worker-time orderedIntegerOperands=True "
                + $"geometry=round-capped-segment-union sourceVerticesUnchanged=True");
            return buffered;
        }

        var offsetter = CreateOffsetter(arcTolerance);
        int added = 0;
        foreach (var line in lines)
        {
            if (line.Count < 2)
                continue;

            offsetter.AddPath(ToPath64(line), JoinType.Round, EndType.Round);
            added++;
        }

        if (added == 0)
            return new List<List<PointD>>();

        var solution = new Paths64();
        offsetter.Execute(radius * PathUtils.Scale, solution);
        var output = FromPaths64(solution);
        if (output.Count > 50)
            output = FromPaths64(Clipper.Union(ToPaths64(output), FillRule.NonZero));

        NormalizeOrder(output);
        return output;
    }

    /// <summary>
    /// Morphological opening (erode then dilate). A non-empty result proves the region can
    /// contain a disc of <paramref name="radius"/> — the minimum-web test that rejects
    /// razor-thin holding tabs.
    /// </summary>
    public static List<List<PointD>> Open(
        IEnumerable<IReadOnlyList<PointD>> polygons,
        double radius,
        double arcTolerance = DefaultArcTolerance)
    {
        var source = polygons.ToList();
        if (source.Count == 0)
            return new List<List<PointD>>();
        if (radius <= 0.0001)
            return source.Select(p => new List<PointD>(p)).ToList();

        var eroded = Offset(source, -radius, arcTolerance, simplifyTolerance: 0);
        if (eroded.Count == 0)
            return new List<List<PointD>>();

        return Offset(eroded, radius, arcTolerance, simplifyTolerance: 0);
    }

    // --- Boolean operations ---------------------------------------------------

    /// <summary>Boolean union; overlapping and touching polygons merge into a minimal set.</summary>
    public static List<List<PointD>> Union(IEnumerable<IReadOnlyList<PointD>> polygons)
    {
        var subjects = ToPaths64(polygons);
        if (subjects.Count == 0)
            return new List<List<PointD>>();

        var output = FromPaths64(Clipper.Union(subjects, FillRule.NonZero));
        NormalizeOrder(output);
        return output;
    }

    /// <summary>Boolean difference (subjects minus clips).</summary>
    public static List<List<PointD>> Difference(
        IEnumerable<IReadOnlyList<PointD>> subjects,
        IEnumerable<IReadOnlyList<PointD>> clips)
    {
        var subjectPaths = ToPaths64(subjects);
        if (subjectPaths.Count == 0)
            return new List<List<PointD>>();

        var clipPaths = ToPaths64(clips);
        if (clipPaths.Count == 0)
            return FromPaths64(subjectPaths);

        var output = FromPaths64(Clipper.Difference(subjectPaths, clipPaths, FillRule.NonZero));
        NormalizeOrder(output);
        return output;
    }

    /// <summary>Boolean intersection; returns only regions covered by both sets.</summary>
    public static List<List<PointD>> Intersect(
        IEnumerable<IReadOnlyList<PointD>> subjects,
        IEnumerable<IReadOnlyList<PointD>> clips)
    {
        var subjectPaths = ToPaths64(subjects);
        var clipPaths = ToPaths64(clips);
        if (subjectPaths.Count == 0 || clipPaths.Count == 0)
            return new List<List<PointD>>();

        var output = FromPaths64(Clipper.Intersect(subjectPaths, clipPaths, FillRule.NonZero));
        NormalizeOrder(output);
        return output;
    }

    /// <summary>
    /// Clips polygons against a boundary, intersecting each one individually.
    /// Batching them would let NonZero filling merge nested concentric rings into a
    /// single filled region, destroying the ring structure the pocket engine depends on.
    /// </summary>
    public static List<List<PointD>> Clip(
        IEnumerable<IReadOnlyList<PointD>> polygons,
        IReadOnlyList<PointD> clipBoundary)
    {
        if (clipBoundary.Count < 3)
            return new List<List<PointD>>();

        var clipPath = ToPath64(clipBoundary);
        if (!Clipper.IsPositive(clipPath))
            clipPath.Reverse();
        var clips = new Paths64 { clipPath };

        var output = new List<List<PointD>>();
        foreach (var polygon in polygons)
        {
            if (polygon.Count < 3)
                continue;
            var subjects = new Paths64 { ToPath64(polygon) };
            output.AddRange(FromPaths64(Clipper.Intersect(subjects, clips, FillRule.NonZero)));
        }
        return output;
    }

    /// <summary>Clips polygons to an axis-aligned rectangle.</summary>
    public static List<List<PointD>> ClipToRect(
        IEnumerable<IReadOnlyList<PointD>> polygons,
        double minX, double minY, double maxX, double maxY)
    {
        var rect = new List<PointD>
        {
            new(minX, minY),
            new(maxX, minY),
            new(maxX, maxY),
            new(minX, maxY),
        };
        return Clip(polygons, rect);
    }

    // Unlike ClipToRect's independently FILLED polygon API, the integer
    // rectangle clipper retains each input contour's directed winding. Union
    // the clipped signed rings together, preserving enclosing holes/islands and
    // overlaps between the captured floor and the first following Cut.
    internal static List<List<PointD>> ClipSignedToRect(
        IEnumerable<IReadOnlyList<PointD>> polygons,long minX,long minY,long maxX,long maxY,
        CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        var clipped=Clipper.RectClip(new Rect64(minX,minY,maxX,maxY),ToPaths64(polygons));
        token.ThrowIfCancellationRequested();
        var result=FromPaths64(Clipper.Union(clipped,FillRule.NonZero));
        NormalizeOrder(result);
        return result;
    }

    /// <summary>
    /// Boolean difference resolved into connected components via Clipper2's PolyTree.
    /// Each top-level outer contour becomes one component carrying its own holes; islands
    /// nested inside a hole become separate components. This is the connected-component
    /// labelling that holding-tab planning relies on to tell floating waste from anchored stock.
    /// </summary>
    public static List<PolygonComponent> DifferenceToComponents(
        IEnumerable<IReadOnlyList<PointD>> subjects,
        IEnumerable<IReadOnlyList<PointD>> clips)
    {
        var components = new List<PolygonComponent>();

        var subjectPaths = ToPaths64(subjects);
        if (subjectPaths.Count == 0)
            return components;

        var clipper = new Clipper64();
        clipper.AddSubject(subjectPaths);

        var clipPaths = ToPaths64(clips);
        if (clipPaths.Count > 0)
            clipper.AddClip(clipPaths);

        var tree = new PolyTree64();
        clipper.Execute(ClipType.Difference, FillRule.NonZero, tree);

        return MaterializeComponents(tree);
    }

    /// <summary>Captures the complete ordered clip operand with the same minimum
    /// vertex filter and ties-to-even integer conversion as DifferenceToComponents.</summary>
    internal static PreparedResidualCoverage PrepareComponentDifference(
        IEnumerable<IReadOnlyList<PointD>> clips, CancellationToken cancellationToken)
        => new(ComponentDifferencePaths(clips, cancellationToken), cancellationToken);

    internal static Paths64 ComponentDifferencePaths(
        IEnumerable<IReadOnlyList<PointD>> polygons, CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var paths = ToPaths64(polygons);
        cancellationToken.ThrowIfCancellationRequested();
        return paths;
    }

    /// <summary>Literal PolyTree traversal. Component, hole and vertex order are
    /// intentionally preserved; this collector performs no canonical sorting.</summary>
    internal static List<PolygonComponent> MaterializeComponents(PolyTree64 tree,
        CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var components = new List<PolygonComponent>();

        List<PointD> Convert(Path64? path)
        {
            var points = new List<PointD>(path?.Count ?? 0);
            if (path is null)
                return points;
            foreach (var pt in path)
            {
                if ((points.Count & 1023) == 0) cancellationToken.ThrowIfCancellationRequested();
                points.Add(ToPointD(pt));
            }
            return points;
        }

        void Collect(PolyPath64 outerNode)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var outer = Convert(outerNode.Polygon);
            if (outer.Count >= 3)
            {
                var component = new PolygonComponent { Outer = outer };
                foreach (PolyPath64 holeNode in outerNode)
                {
                    var hole = Convert(holeNode.Polygon);
                    if (hole.Count >= 3)
                        component.Holes.Add(hole);
                }
                components.Add(component);
            }

            foreach (PolyPath64 holeNode in outerNode)
                foreach (PolyPath64 islandNode in holeNode)
                    Collect(islandNode);
        }

        foreach (PolyPath64 topLevel in tree)
            Collect(topLevel);

        cancellationToken.ThrowIfCancellationRequested();
        return components;
    }

    // --- Measurement & canonical ordering -------------------------------------

    /// <summary>Signed area; positive is counter-clockwise in Y-up space.</summary>
    public static double SignedArea(IReadOnlyList<PointD> polygon)
    {
        double area = 0;
        for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
            area += (polygon[j].x - polygon[i].x) * (polygon[j].y + polygon[i].y);
        return area * 0.5;
    }

    /// <summary>Arithmetic mean of the vertices.</summary>
    public static PointD Centroid(IReadOnlyList<PointD> polygon)
    {
        if (polygon.Count == 0)
            return new PointD(0, 0);

        double cx = 0, cy = 0;
        for (int i = 0; i < polygon.Count; i++)
        {
            cx += polygon[i].x;
            cy += polygon[i].y;
        }
        return new PointD(cx / polygon.Count, cy / polygon.Count);
    }

    /// <summary>Perimeter of a closed ring, including the implicit closing edge.</summary>
    public static double Perimeter(IReadOnlyList<PointD> polygon)
    {
        if (polygon.Count < 2)
            return 0;

        double total = 0;
        for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
        {
            double dx = polygon[i].x - polygon[j].x;
            double dy = polygon[i].y - polygon[j].y;
            total += Math.Sqrt(dx * dx + dy * dy);
        }
        return total;
    }

    /// <summary>
    /// Tests a point against a filled set whose holes wind opposite their outers, as produced
    /// by the boolean operations above: the point is inside when the enclosing windings do not
    /// cancel out.
    /// </summary>
    public static bool Contains(IReadOnlyList<IReadOnlyList<PointD>> polygons, double x, double y)
    {
        int winding = 0;
        for (int i = 0; i < polygons.Count; i++)
        {
            if (!PointInPolygon(polygons[i], x, y))
                continue;

            winding += SignedArea(polygons[i]) >= 0 ? 1 : -1;
        }
        return winding != 0;
    }

    private static bool PointInPolygon(IReadOnlyList<PointD> polygon, double x, double y)
    {
        bool inside = false;
        for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
        {
            var current = polygon[i];
            var previous = polygon[j];
            if ((current.y > y) != (previous.y > y)
                && x < (previous.x - current.x) * (y - current.y) / (previous.y - current.y) + current.x)
            {
                inside = !inside;
            }
        }
        return inside;
    }

    /// <summary>
    /// Imposes a deterministic order on Clipper2 output: each polygon starts at its
    /// lexicographically smallest vertex, and the list is sorted by descending area.
    /// Clipper2 makes no ordering guarantee, so without this the same input can produce
    /// different toolpath ordering between runs.
    /// </summary>
    public static List<List<PointD>> NormalizeOrder(List<List<PointD>> polygons)
    {
        foreach (var polygon in polygons)
        {
            if (polygon.Count > 0)
                RotateToMinVertex(polygon);
        }

        if (polygons.Count <= 1)
            return polygons;

        // List.Sort retains its original comparison and equal-key behavior, but
        // every contour measurement is now evaluated only once after rotation.
        var keys = new Dictionary<List<PointD>, (double Area, PointD Centroid)>(polygons.Count);
        foreach (var polygon in polygons)
            keys[polygon] = (Math.Abs(SignedArea(polygon)), Centroid(polygon));
        polygons.Sort((a, b) =>
        {
            int cmp = keys[b].Area.CompareTo(keys[a].Area);
            if (cmp != 0)
                return cmp;

            var ca = keys[a].Centroid;
            var cb = keys[b].Centroid;
            cmp = ca.x.CompareTo(cb.x);
            return cmp != 0 ? cmp : ca.y.CompareTo(cb.y);
        });

        return polygons;
    }

    private static void RotateToMinVertex(List<PointD> polygon)
    {
        int minIdx = 0;
        for (int i = 1; i < polygon.Count; i++)
        {
            if (polygon[i].x < polygon[minIdx].x ||
                (polygon[i].x == polygon[minIdx].x && polygon[i].y < polygon[minIdx].y))
                minIdx = i;
        }
        if (minIdx == 0)
            return;

        var rotated = new List<PointD>(polygon.Count);
        for (int i = minIdx; i < polygon.Count; i++)
            rotated.Add(polygon[i]);
        for (int i = 0; i < minIdx; i++)
            rotated.Add(polygon[i]);

        polygon.Clear();
        polygon.AddRange(rotated);
    }
}
