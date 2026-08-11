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

    private sealed class BoundarySegmentIndex
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

        internal double DistanceSquared(Point64 point)
            => _root is null
                ? double.MaxValue
                : DistanceSquared(_root, point, double.MaxValue);

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

            _boundaryIndex = new BoundarySegmentIndex(_source);
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

            long executeStart = Stopwatch.GetTimestamp();
            _offsetter.Execute(scaledDelta, solution);
            OffsetExecutionMilliseconds += ElapsedMilliseconds(executeStart);

            int rawCount = solution.Count;
            long validationStart = Stopwatch.GetTimestamp();
            int rejected = RemoveOffsetInversions(
                _boundaryIndex,
                solution,
                scaledDelta,
                _arcTolerance,
                out bool rejectedSignificantContour,
                out string details);
            ValidationMilliseconds += ElapsedMilliseconds(validationStart);

            int fallbackPathCount = 0;
            int fallbackRejected = 0;
            if (rejected > 0 && (solution.Count == 0 || rejectedSignificantContour))
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

                long fallbackValidationStart = Stopwatch.GetTimestamp();
                fallbackRejected = RemoveOffsetInversions(
                    _boundaryIndex,
                    solution,
                    scaledDelta,
                    _arcTolerance,
                    out _,
                    out string fallbackDetails);
                ValidationMilliseconds += ElapsedMilliseconds(fallbackValidationStart);
                if (fallbackRejected > 0)
                    details += $";fallbackRejected={fallbackRejected}:{fallbackDetails}";
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
    {
        long t0 = PerfLog.Start();
        double clearanceTolerance = Math.Max(
            4.0,
            Math.Min(0.01, Math.Max(0, arcTolerance) * 0.05) * PathUtils.Scale);
        double requiredClearance = Math.Abs(scaledDelta) - clearanceTolerance;
        double significantArea = 0.01 * PathUtils.Scale * PathUtils.Scale;
        rejectedSignificantContour = false;
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
            if (Math.Abs(Clipper.Area(contour)) > significantArea)
                rejectedSignificantContour = true;
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
            $"area={Math.Abs(Clipper.Area(contour)) / scaleSquared:F4},clear={clearance:F6}";
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

        double fallbackArcTolerance = Math.Min(arcTolerance, 0.01);
        double safetyMargin = Math.Max(4.0, fallbackArcTolerance * PathUtils.Scale);
        var boundaryOffsetter = CreateOffsetter(fallbackArcTolerance);
        int sourcePoints = 0;
        foreach (Path64 boundary in sourceRegion)
        {
            if (boundary.Count < 2)
                continue;
            sourcePoints += boundary.Count;
            boundaryOffsetter.AddPath(boundary, JoinType.Round, EndType.Joined);
        }

        var boundaryBand = new Paths64();
        boundaryOffsetter.Execute(Math.Abs(scaledDelta) + safetyMargin, boundaryBand);
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
            $"bandPaths={boundaryBand.Count} resultPaths={result.Count}");
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

        var offsetter = CreateOffsetter(arcTolerance);
        offsetter.AddPaths(group, joinType, EndType.Polygon);

        var solution = new Paths64();
        double scaledDelta = delta * PathUtils.Scale;
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
            fallbackRejected = RemoveOffsetInversions(
                new BoundarySegmentIndex(group),
                solution,
                scaledDelta,
                arcTolerance,
                out _,
                out string fallbackDetails);
            if (fallbackRejected > 0)
                details += $";fallbackRejected={fallbackRejected}:{fallbackDetails}";
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
