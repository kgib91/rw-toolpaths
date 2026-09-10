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
public static class PolygonOps
{
    private readonly record struct BoundarySegment(
        Point64 Start,
        Point64 End,
        long MinX,
        long MinY,
        long MaxX,
        long MaxY);

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
        private double _knownClearance;
        private double _emptyAtRadius = double.PositiveInfinity;

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
                        Math.Max(start.Y, end.Y)));
                }
            }

            _segments = segments.ToArray();
            _root = _segments.Length == 0 ? null : BuildNode(0, _segments.Length);
        }

        internal int SegmentCount => _segments.Length;

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

        internal bool MayContainDisc(double radius, double resolution)
        {
            if (_root == null || radius >= _emptyAtRadius) return false;
            if (radius <= _knownClearance) return true;
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
                _knownClearance = Math.Max(_knownClearance, distance);
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
            if (!possible) _emptyAtRadius = Math.Min(_emptyAtRadius, radius);
            Console.WriteLine($"[PolygonOps.InsetFeasibility] radius={radius / PathUtils.Scale:F4} possible={possible} probes={probes} elapsedMs={ElapsedMilliseconds(started):F2}");
            return possible;
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
            Array.Sort(
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

    private static ClipperOffset CreateOffsetter(double arcTolerance)
        => new(
            miterLimit: DefaultMiterLimit,
            arcTolerance: arcTolerance * PathUtils.Scale,
            // Collinear vertices only inflate the point count; RDP would drop them anyway.
            preserveCollinear: false,
            reverseSolution: false);

    internal sealed class PreparedBoundaryOffset
    {
        private readonly Paths64 _source;
        private readonly BoundarySegmentIndex _boundaryIndex;
        private readonly ClipperOffset _offsetter;
        private readonly double _arcTolerance;
        private readonly double _simplifyTolerance;
        private Paths64? _sourceRegion;
        private Paths64? _denseInsetBase;
        private double _denseInsetDelta;

        internal PreparedBoundaryOffset(
            IReadOnlyList<PointD> outer,
            IEnumerable<IReadOnlyList<PointD>>? holes,
            double arcTolerance,
            double simplifyTolerance,
            JoinType joinType)
        {
            long prepareStart = Stopwatch.GetTimestamp();
            _arcTolerance = arcTolerance;
            _simplifyTolerance = simplifyTolerance;
            _source = new Paths64();

            if (outer.Count >= 3)
            {
                var outerPath = ToPath64(outer);
                if (!Clipper.IsPositive(outerPath))
                    outerPath.Reverse();
                _source.Add(outerPath);

                if (holes is not null)
                {
                    foreach (var hole in holes)
                    {
                        if (hole.Count < 3)
                            continue;
                        var holePath = ToPath64(hole);
                        if (Clipper.IsPositive(holePath))
                            holePath.Reverse();
                        _source.Add(holePath);
                    }
                }
            }

            _sourceRegion = Clipper.Union(_source, FillRule.NonZero);
            _boundaryIndex = new BoundarySegmentIndex(_sourceRegion);
            _offsetter = CreateOffsetter(arcTolerance);
            if (_source.Count > 0)
                _offsetter.AddPaths(_source, joinType, EndType.Polygon);

            SourcePointCount = _source.Sum(path => path.Count);
            PreparationMilliseconds = ElapsedMilliseconds(prepareStart);
        }

        internal int SourcePathCount => _source.Count;
        internal int SourcePointCount { get; }
        internal int OffsetCount { get; private set; }
        internal int FallbackCount { get; private set; }
        internal double PreparationMilliseconds { get; }
        internal double OffsetExecutionMilliseconds { get; private set; }
        internal double ValidationMilliseconds { get; private set; }
        internal double FallbackMilliseconds { get; private set; }
        internal double MaterializationMilliseconds { get; private set; }

        internal List<List<PointD>> Execute(double delta)
        {
            if (_source.Count == 0)
                return new List<List<PointD>>();

            OffsetCount++;
            var solution = new Paths64();
            double scaledDelta = delta * PathUtils.Scale;
            if (scaledDelta < 0 && !_boundaryIndex.MayContainDisc(-scaledDelta, _arcTolerance * PathUtils.Scale))
                return new List<List<PointD>>();

            long executeStart = Stopwatch.GetTimestamp();
            bool usedBoundaryStroke = SourcePointCount > 8192;
            if (usedBoundaryStroke)
            {
                // Dense image contours make one monolithic offset pathological. The existing
                // conservative boundary-band construction computes the same safe erosion/dilation.
                _sourceRegion ??= Clipper.Union(_source, FillRule.NonZero);
                bool reuseInset = scaledDelta < 0 && _denseInsetBase != null && scaledDelta < _denseInsetDelta;
                // Erosion by two disks equals erosion by the sum of their radii. Always reuse
                // the first inset (never the preceding ring), so rounding does not accumulate.
                if (reuseInset)
                {
                    var insetOffsetter = CreateOffsetter(_arcTolerance);
                    insetOffsetter.AddPaths(KeepInsetComponents(_denseInsetBase!, scaledDelta - _denseInsetDelta), JoinType.Round, EndType.Polygon);
                    // A polygonal round join can fall short of its exact disk by one arc tolerance.
                    // Reserve that error when reusing the first proven inset, without accumulating it per ring.
                    double insetGuard = Math.Max(8.0, 2 * _arcTolerance * PathUtils.Scale);
                    insetOffsetter.Execute(scaledDelta - _denseInsetDelta - insetGuard, solution);
                }
                else
                {
                    solution = BuildConservativeOffset(_sourceRegion, scaledDelta, _arcTolerance, sourceIsCanonical: true);
                }
                FallbackCount++;
                FallbackMilliseconds += ElapsedMilliseconds(executeStart);
                Console.WriteLine($"[PolygonOps.DenseOffset] sourcePoints={SourcePointCount} delta={delta:R} paths={solution.Count} reusedFirstInset={reuseInset}");
            }
            else
            {
                _offsetter.Execute(scaledDelta, solution);
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
                    scaledDelta, _arcTolerance, significantDeficit);
                FallbackMilliseconds += ElapsedMilliseconds(repairStart);
                FallbackCount++;
            }

            if (usedBoundaryStroke && scaledDelta < 0 && _denseInsetBase == null)
            {
                var compact = new Paths64(solution.Select(path => {
                    var reduced = PolygonSimplify.RdpSegmentBounded(FromPaths64(new Paths64 { path })[0], _arcTolerance);
                    var reducedPath = reduced.Count >= 3 ? ToPath64(reduced) : new Path64(path);
                    return Clipper.Area(reducedPath) * Clipper.Area(path) > 0 ? reducedPath : new Path64(path);
                }));
                // As with heightmap slice compaction, first move simplified boundaries inward.
                // Intersecting unguarded chords directly creates narrow alternating notches that
                // fragment later offsets. The final intersection retains all protected islands.
                var cacheOffsetter = CreateOffsetter(_arcTolerance / 4);
                cacheOffsetter.AddPaths(compact, JoinType.Round, EndType.Polygon);
                var guarded = new Paths64();
                cacheOffsetter.Execute(-Math.Max(8.0, _arcTolerance * PathUtils.Scale + 8), guarded);
                _denseInsetBase = Clipper.Intersect(solution, guarded, FillRule.NonZero);
                Console.WriteLine($"[PolygonOps.DenseInsetCache] validatedContours={solution.Count} " +
                    $"cacheContours={_denseInsetBase.Count} validatedPoints={solution.Sum(path => path.Count)} " +
                    $"cachePoints={_denseInsetBase.Sum(path => path.Count)} guard={_arcTolerance:R}");
                _denseInsetDelta = scaledDelta;
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
            MaterializationMilliseconds += ElapsedMilliseconds(materializationStart);
            return output;
        }
    }

    internal static PreparedBoundaryOffset PrepareBoundaryOffset(
        IReadOnlyList<PointD> outer,
        IEnumerable<IReadOnlyList<PointD>>? holes,
        double arcTolerance = DefaultArcTolerance,
        double simplifyTolerance = PolygonSimplify.DefaultTolerance,
        JoinType joinType = JoinType.Round)
        => new(outer, holes, arcTolerance, simplifyTolerance, joinType);

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
        int rejected = 0;
        int outputPoints = 0;
        for (int outputIndex = solution.Count - 1; outputIndex >= 0; outputIndex--)
        {
            Path64 contour = solution[outputIndex];
            outputPoints += contour.Count;
            double minimumClearanceSquared = double.MaxValue;
            bool hasBoundaryClearance = contour.Count >= 3;
            for (int pointIndex = 0; pointIndex < contour.Count; pointIndex++)
            {
                double clearanceSquared = boundaryIndex.DistanceSquared(contour[pointIndex]);
                minimumClearanceSquared = Math.Min(minimumClearanceSquared, clearanceSquared);
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
            // A hole remains protected regardless of its area. Only tiny isolated outer
            // fragments may be discarded without repairing their candidate region.
            double signedArea = Clipper.Area(contour);
            if (Math.Abs(signedArea) > significantArea || signedArea < 0)
            {
                rejectedSignificantContour = true;
                significantDeficit = Math.Max(significantDeficit,
                    requiredClearance - Math.Sqrt(minimumClearanceSquared));
            }
            solution.RemoveAt(outputIndex);
            rejected++;
        }

        details = string.Join(";", decisions);
        if (rejected > 0)
        {
            PerfLog.Stop("PolygonOps.OffsetInversionCheck", t0,
                $"sourceSegments={boundaryIndex.SegmentCount} outputPoints={outputPoints} " +
                $"rejected={rejected}");
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

    private static Paths64 RepairConservativeOffset(
        BoundarySegmentIndex boundaryIndex,
        Paths64 sourceRegion,
        Paths64 candidate,
        double scaledDelta,
        double arcTolerance,
        double significantDeficit)
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
                candidate = BuildConservativeOffset(sourceRegion,
                    scaledDelta + Math.Sign(scaledDelta) * totalMargin,
                    repairArcTolerance, sourceIsCanonical: true);
                method = "boundary-rebuild";
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
            $"Pocket offset could not preserve cutter clearance after three conservative repairs " +
            $"(offset {scaledDelta / PathUtils.Scale:F6} mm, deficit {significantDeficit / PathUtils.Scale:F6} mm). {details}");
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
                MaxDegreeOfParallelism = Math.Min(4, Environment.ProcessorCount) }, StrokeChunk);
        foreach (var band in bands) boundaryBand.AddRange(band);
        strokeChunks = chunks.Count;
        if (boundaryBand.Count == 0)
            return new Paths64();

        boundaryBand = Clipper.Union(boundaryBand, FillRule.NonZero);

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
            $"strokeChunks={strokeChunks} bandPaths={boundaryBand.Count} resultPaths={result.Count}");
        return result;
    }

    // --- Offsetting -----------------------------------------------------------

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
        JoinType joinType = JoinType.Round)
    {
        var group = ToPaths64(polygons);
        if (group.Count == 0)
            return new List<List<PointD>>();

        double scaledDelta = delta * PathUtils.Scale;
        if (scaledDelta < 0)
        {
            // A collapsed source component can produce an inverted positive-area offset.
            // Exclude it before offsetting so its rejection never rebuilds a valid neighbor.
            group = KeepInsetComponents(group, scaledDelta, arcTolerance * PathUtils.Scale);
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
            new BoundarySegmentIndex(group), solution, scaledDelta, arcTolerance,
            out bool rejectedSignificantContour, out string details);
        int fallbackCount = 0;
        int fallbackRejected = 0;
        if (rejected > 0 && (solution.Count == 0 || rejectedSignificantContour))
        {
            solution = BuildConservativeOffset(group, scaledDelta, arcTolerance);
            fallbackCount = solution.Count;
            var rawCandidate = new Paths64(solution);
            fallbackRejected = RemoveOffsetInversions(
                new BoundarySegmentIndex(group),
                solution,
                scaledDelta,
                arcTolerance,
                out bool fallbackSignificant,
                out string fallbackDetails,
                out double significantDeficit);
            if (fallbackRejected > 0)
                details += $";fallbackRejected={fallbackRejected}:{fallbackDetails}";
            if (fallbackSignificant)
                solution = RepairConservativeOffset(new BoundarySegmentIndex(group),
                    Clipper.Union(group, FillRule.NonZero), rawCandidate, scaledDelta, arcTolerance, significantDeficit);
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
        if (radius < 0.0001)
            return new List<List<PointD>>();

        var group = ToPaths64(polygons);
        if (group.Count == 0)
            return new List<List<PointD>>();

        var offsetter = CreateOffsetter(arcTolerance);
        foreach (var path in group)
            offsetter.AddPath(path, JoinType.Round, EndType.Polygon);

        var solution = new Paths64();
        offsetter.Execute(radius * PathUtils.Scale, solution);

        var output = FromPaths64(solution);
        NormalizeOrder(output);
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
        if (lines.Sum(line => (long)line.Count) > 8192)
        {
            long started = Stopwatch.GetTimestamp();
            var bands = new Paths64();
            int chunks = 0;
            foreach (var line in lines)
            for (int start = 0; start < line.Count - 1; start += 256)
            {
                cancellationToken.ThrowIfCancellationRequested();
                var chunk = line.Skip(start).Take(Math.Min(257, line.Count - start)).ToList();
                var chunkOffset = CreateOffsetter(arcTolerance);
                chunkOffset.AddPath(ToPath64(chunk), JoinType.Round, EndType.Round);
                var band = new Paths64();
                chunkOffset.Execute(radius * PathUtils.Scale, band);
                bands.AddRange(band); chunks++;
            }
            cancellationToken.ThrowIfCancellationRequested();
            var buffered = FromPaths64(Clipper.Union(bands, FillRule.NonZero));
            NormalizeOrder(buffered);
            Console.WriteLine($"[PolygonOps.OpenStroke] chunks={chunks} points={buffered.Sum(p => p.Count)} elapsedMs={ElapsedMilliseconds(started):F1}");
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

        static List<PointD> Convert(Path64? path)
        {
            var points = new List<PointD>(path?.Count ?? 0);
            if (path is null)
                return points;
            foreach (var pt in path)
                points.Add(ToPointD(pt));
            return points;
        }

        void Collect(PolyPath64 outerNode)
        {
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

        polygons.Sort((a, b) =>
        {
            int cmp = Math.Abs(SignedArea(b)).CompareTo(Math.Abs(SignedArea(a)));
            if (cmp != 0)
                return cmp;

            var ca = Centroid(a);
            var cb = Centroid(b);
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
