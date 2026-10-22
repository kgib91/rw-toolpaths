// Frozen current-source oracle captured before the toolpath-scaling change.
// Retains legacy first-seen grid order and continuous cutter-profile equations.
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

internal sealed class LegacyWallClearanceGrid
    {
        private readonly IReadOnlyList<(PointD A, PointD B)> _segments;
        private readonly double _minX, _minY;
        private readonly double _cellSizeX, _cellSizeY;
        private readonly int _cols, _rows;
        private readonly List<int>[,] _grid;

        public LegacyWallClearanceGrid(IReadOnlyList<(PointD A, PointD B)> segments)
        {
            _segments = segments;
            if (segments.Count == 0)
            {
                _grid = new List<int>[0, 0];
                return;
            }

            double minX = double.MaxValue, minY = double.MaxValue;
            double maxX = -double.MaxValue, maxY = -double.MaxValue;
            foreach (var seg in segments)
            {
                minX = Math.Min(minX, Math.Min(seg.A.x, seg.B.x));
                maxX = Math.Max(maxX, Math.Max(seg.A.x, seg.B.x));
                minY = Math.Min(minY, Math.Min(seg.A.y, seg.B.y));
                maxY = Math.Max(maxY, Math.Max(seg.A.y, seg.B.y));
            }

            _minX = minX;
            _minY = minY;

            int n = segments.Count;
            int dims = Math.Clamp((int)Math.Sqrt(n), 8, 64);
            _cols = dims;
            _rows = dims;

            double width = Math.Max(1e-9, maxX - minX);
            double height = Math.Max(1e-9, maxY - minY);

            _cellSizeX = width / _cols;
            _cellSizeY = height / _rows;

            _grid = new List<int>[_cols, _rows];

            for (int i = 0; i < segments.Count; i++)
            {
                var seg = segments[i];
                double segMinX = Math.Min(seg.A.x, seg.B.x);
                double segMaxX = Math.Max(seg.A.x, seg.B.x);
                double segMinY = Math.Min(seg.A.y, seg.B.y);
                double segMaxY = Math.Max(seg.A.y, seg.B.y);

                int c1 = Math.Clamp((int)((segMinX - _minX) / _cellSizeX), 0, _cols - 1);
                int c2 = Math.Clamp((int)((segMaxX - _minX) / _cellSizeX), 0, _cols - 1);
                int r1 = Math.Clamp((int)((segMinY - _minY) / _cellSizeY), 0, _rows - 1);
                int r2 = Math.Clamp((int)((segMaxY - _minY) / _cellSizeY), 0, _rows - 1);

                for (int c = c1; c <= c2; c++)
                {
                    for (int r = r1; r <= r2; r++)
                    {
                        _grid[c, r] ??= new List<int>();
                        _grid[c, r].Add(i);
                    }
                }
            }
        }

        public double QueryDistance(double x, double y)
        {
            if (_segments.Count == 0) return 0.0;

            int targetC = Math.Clamp((int)((x - _minX) / _cellSizeX), 0, _cols - 1);
            int targetR = Math.Clamp((int)((y - _minY) / _cellSizeY), 0, _rows - 1);

            double bestSq = double.PositiveInfinity;

            // 1. First, check the target cell and its immediate 8 neighbors to get a good initial upper bound.
            for (int dc = -1; dc <= 1; dc++)
            {
                int c = targetC + dc;
                if (c < 0 || c >= _cols) continue;
                for (int dr = -1; dr <= 1; dr++)
                {
                    int r = targetR + dr;
                    if (r < 0 || r >= _rows) continue;
                    CheckCell(c, r, x, y, ref bestSq);
                }
            }

            // Expanding ring search
            int maxR = Math.Max(_cols, _rows);
            for (int ring = 2; ring <= maxR; ring++)
            {
                double minCellSize = Math.Min(_cellSizeX, _cellSizeY);
                double minDistToRing = (ring - 1) * minCellSize;
                if (minDistToRing * minDistToRing >= bestSq)
                {
                    break;
                }

                int cMin = targetC - ring;
                int cMax = targetC + ring;
                int rMin = targetR - ring;
                int rMax = targetR + ring;

                // Check cells on the boundary of the ring
                // Top & Bottom rows
                for (int c = cMin; c <= cMax; c++)
                {
                    if (c >= 0 && c < _cols)
                    {
                        if (rMin >= 0 && rMin < _rows) CheckCell(c, rMin, x, y, ref bestSq);
                        if (rMax >= 0 && rMax < _rows) CheckCell(c, rMax, x, y, ref bestSq);
                    }
                }
                // Left & Right columns
                for (int r = rMin + 1; r < rMax; r++)
                {
                    if (r >= 0 && r < _rows)
                    {
                        if (cMin >= 0 && cMin < _cols) CheckCell(cMin, r, x, y, ref bestSq);
                        if (cMax >= 0 && cMax < _cols) CheckCell(cMax, r, x, y, ref bestSq);
                    }
                }
            }

            // Fallback
            if (bestSq == double.PositiveInfinity)
            {
                for (int i = 0; i < _segments.Count; i++)
                {
                    double d2 = DistanceToSegmentSq(x, y, _segments[i]);
                    if (d2 < bestSq) bestSq = d2;
                }
            }

            return bestSq < double.PositiveInfinity ? Math.Sqrt(bestSq) : 0.0;
        }

        // Endpoints must already have analytical Z from this boundary. This permits the
        // endpoint-distance lower bound used by the Lipschitz shortcut below.
        internal bool TryFindDepthViolation(Point3D a, Point3D b, double startZ,
            double bottomRadius, double radiusSlope, double zTolerance, out double parameter,
            VCarveCutterProfile? cutterProfile = null)
        {
            if (cutterProfile is not null)
                return TryFindProfileDepthViolation(a, b, startZ, cutterProfile, zTolerance, out parameter);
            parameter = 0.5;
            double depthA = startZ - a.Z, depthB = startZ - b.Z;
            if (_segments.Count == 0 || Math.Max(depthA, depthB) <= zTolerance) return false;
            double dx = b.X - a.X, dy = b.Y - a.Y;
            double depthDelta = depthB - depthA;
            double length = Math.Sqrt(dx * dx + dy * dy);
            double endpointRadiusDelta = radiusSlope * depthDelta;
            // Boundary distance is 1-Lipschitz. Positive-depth endpoints provide lower
            // bounds even at the maximum-depth clamp; very short chords need no grid query.
            if (Math.Min(depthA, depthB) > zTolerance
                && length * length - endpointRadiusDelta * endpointRadiusDelta
                    <= 2 * length * radiusSlope * zTolerance) return false;

            double lo = 0, hi = 1;
            if (depthA <= zTolerance) lo = (zTolerance - depthA) / depthDelta;
            if (depthB <= zTolerance) hi = (zTolerance - depthA) / depthDelta;
            double radius0 = bottomRadius + radiusSlope * (depthA - zTolerance);
            double radiusDelta = radiusSlope * depthDelta;
            double radius = Math.Max(radius0 + radiusDelta * lo, radius0 + radiusDelta * hi);
            double x0 = a.X + lo * dx, x1 = a.X + hi * dx;
            double y0 = a.Y + lo * dy, y1 = a.Y + hi * dy;
            int c0 = Math.Clamp((int)Math.Floor((Math.Min(x0, x1) - radius - _minX) / _cellSizeX), 0, _cols - 1);
            int c1 = Math.Clamp((int)Math.Floor((Math.Max(x0, x1) + radius - _minX) / _cellSizeX), 0, _cols - 1);
            int r0 = Math.Clamp((int)Math.Floor((Math.Min(y0, y1) - radius - _minY) / _cellSizeY), 0, _rows - 1);
            int r1 = Math.Clamp((int)Math.Floor((Math.Max(y0, y1) + radius - _minY) / _cellSizeY), 0, _rows - 1);
            var candidates = new HashSet<int>();
            for (int c = c0; c <= c1; c++)
            for (int r = r0; r <= r1; r++)
                if (_grid[c, r] is { } cell) candidates.UnionWith(cell);

            double minimum = -1e-12;
            double witness = 0.5;
            bool violated = false;
            Span<double> cuts = stackalloc double[4];
            foreach (int candidate in candidates)
            {
                var edge = _segments[candidate];
                double ex = edge.B.x - edge.A.x, ey = edge.B.y - edge.A.y;
                double edgeLengthSquared = ex * ex + ey * ey;
                double ax = a.X - edge.A.x, ay = a.Y - edge.A.y;
                double projection0 = edgeLengthSquared > 0 ? (ax * ex + ay * ey) / edgeLengthSquared : 0;
                double projectionDelta = edgeLengthSquared > 0 ? (dx * ex + dy * ey) / edgeLengthSquared : 0;
                int count = 2;
                cuts[0] = lo; cuts[1] = hi;
                if (projectionDelta != 0)
                {
                    double first = -projection0 / projectionDelta;
                    double last = (1 - projection0) / projectionDelta;
                    if (first > lo && first < hi) cuts[count++] = first;
                    if (last > lo && last < hi) cuts[count++] = last;
                }
                cuts[..count].Sort();
                for (int i = 1; i < count; i++)
                {
                    double low = cuts[i - 1], high = cuts[i];
                    double projection = projection0 + projectionDelta * (low + high) / 2;
                    double qx = ax, qy = ay, vx = dx, vy = dy;
                    if (projection >= 1) { qx -= ex; qy -= ey; }
                    else if (projection > 0)
                    {
                        qx -= projection0 * ex; qy -= projection0 * ey;
                        vx -= projectionDelta * ex; vy -= projectionDelta * ey;
                    }
                    // distance^2 - requiredRadius^2 is quadratic on this projection
                    // interval. Its minimum is at an endpoint or its stationary point.
                    double quadratic = vx * vx + vy * vy - radiusDelta * radiusDelta;
                    double linear = 2 * (qx * vx + qy * vy - radius0 * radiusDelta);
                    void Check(double t)
                    {
                        double x = qx + vx * t, y = qy + vy * t;
                        double required = radius0 + radiusDelta * t;
                        double value = x * x + y * y - required * required;
                        if (value < minimum) { minimum = value; witness = t; violated = true; }
                    }
                    Check(low); Check(high);
                    if (quadratic > 0)
                    {
                        double stationary = -linear / (2 * quadratic);
                        if (stationary > low && stationary < high) Check(stationary);
                    }
                }
            }
            parameter = witness;
            return violated;
        }

        // On each physical section, radius squared is a quadratic of axial depth.
        // This lets the same all-chord proof cover a hemisphere, its taper and a
        // cylindrical plateau without sampling past a section transition.
        private readonly record struct RadiusInterval(double Low, double High,
            double Depth0, double DepthDelta, double BottomRadius, double Slope, double BallRadius)
        {
            public double Quadratic => BallRadius > 0 ? -DepthDelta * DepthDelta
                : Slope * Slope * DepthDelta * DepthDelta;
            public double Linear => BallRadius > 0 ? 2 * (BallRadius - Depth0) * DepthDelta
                : 2 * (BottomRadius + Slope * Depth0) * Slope * DepthDelta;
            public double RadiusSquared(double t)
            {
                double h = Depth0 + DepthDelta * t;
                if (BallRadius > 0) return h * (2 * BallRadius - h);
                double radius = BottomRadius + Slope * h;
                return radius * radius;
            }
        }

        private bool TryFindProfileDepthViolation(Point3D a, Point3D b, double startZ,
            VCarveCutterProfile profile, double zTolerance, out double parameter)
        {
            parameter = 0.5;
            double depthA = startZ - a.Z, depthB = startZ - b.Z;
            double minDepth = Math.Min(depthA, depthB), maxDepth = Math.Max(depthA, depthB);
            if (_segments.Count == 0 || maxDepth <= zTolerance) return false;

            // Retain the proven short-chord shortcut and exact cone arithmetic when
            // the entire depth interval (including its tolerance reserve) is linear.
            double sectionStart = 0;
            foreach (var section in profile.Sections)
            {
                double sectionEnd = Math.Min(profile.MaximumDepth, sectionStart + section.Height);
                if (section.Kind == CarvingCutterSectionKind.Linear
                    && minDepth - zTolerance >= sectionStart && maxDepth <= sectionEnd)
                {
                    double slope = (section.TopRadius - section.BottomRadius) / section.Height;
                    return TryFindDepthViolation(a, b, startZ,
                        section.BottomRadius - slope * sectionStart, slope, zTolerance, out parameter);
                }
                sectionStart = sectionEnd;
                if (sectionStart >= profile.MaximumDepth) break;
            }

            double depthDelta = depthB - depthA;
            double lo = 0, hi = 1;
            if (depthA <= zTolerance) lo = (zTolerance - depthA) / depthDelta;
            if (depthB <= zTolerance) hi = (zTolerance - depthA) / depthDelta;
            var intervals = new List<RadiusInterval>(profile.Sections.Count);
            sectionStart = 0;
            foreach (var section in profile.Sections)
            {
                double sectionEnd = Math.Min(profile.MaximumDepth, sectionStart + section.Height);
                double low = lo, high = hi;
                if (depthDelta == 0)
                {
                    if (depthA - zTolerance < sectionStart || depthA - zTolerance > sectionEnd)
                    {
                        sectionStart = sectionEnd;
                        if (sectionStart >= profile.MaximumDepth) break;
                        continue;
                    }
                }
                else
                {
                    double t0 = (sectionStart + zTolerance - depthA) / depthDelta;
                    double t1 = (sectionEnd + zTolerance - depthA) / depthDelta;
                    low = Math.Max(low, Math.Min(t0, t1));
                    high = Math.Min(high, Math.Max(t0, t1));
                }
                if (high >= low)
                    intervals.Add(new RadiusInterval(low, high, depthA - zTolerance - sectionStart,
                        depthDelta, section.BottomRadius,
                        (section.TopRadius - section.BottomRadius) / section.Height,
                        section.Kind == CarvingCutterSectionKind.Hemisphere ? section.TopRadius : 0));
                sectionStart = sectionEnd;
                if (sectionStart >= profile.MaximumDepth) break;
            }

            double dx = b.X - a.X, dy = b.Y - a.Y;
            double radius = profile.RadiusAtDepth(maxDepth - zTolerance);
            double x0 = a.X + lo * dx, x1 = a.X + hi * dx;
            double y0 = a.Y + lo * dy, y1 = a.Y + hi * dy;
            int c0 = Math.Clamp((int)Math.Floor((Math.Min(x0, x1) - radius - _minX) / _cellSizeX), 0, _cols - 1);
            int c1 = Math.Clamp((int)Math.Floor((Math.Max(x0, x1) + radius - _minX) / _cellSizeX), 0, _cols - 1);
            int r0 = Math.Clamp((int)Math.Floor((Math.Min(y0, y1) - radius - _minY) / _cellSizeY), 0, _rows - 1);
            int r1 = Math.Clamp((int)Math.Floor((Math.Max(y0, y1) + radius - _minY) / _cellSizeY), 0, _rows - 1);
            var candidates = new HashSet<int>();
            for (int c = c0; c <= c1; c++)
            for (int r = r0; r <= r1; r++)
                if (_grid[c, r] is { } cell) candidates.UnionWith(cell);

            double minimum = -1e-12, witness = 0.5;
            bool violated = false;
            Span<double> cuts = stackalloc double[4];
            foreach (int candidate in candidates)
            {
                var edge = _segments[candidate];
                double ex = edge.B.x - edge.A.x, ey = edge.B.y - edge.A.y;
                double edgeLengthSquared = ex * ex + ey * ey;
                double ax = a.X - edge.A.x, ay = a.Y - edge.A.y;
                double projection0 = edgeLengthSquared > 0 ? (ax * ex + ay * ey) / edgeLengthSquared : 0;
                double projectionDelta = edgeLengthSquared > 0 ? (dx * ex + dy * ey) / edgeLengthSquared : 0;
                foreach (var interval in intervals)
                {
                    int count = 2;
                    cuts[0] = interval.Low; cuts[1] = interval.High;
                    if (projectionDelta != 0)
                    {
                        double first = -projection0 / projectionDelta;
                        double last = (1 - projection0) / projectionDelta;
                        if (first > interval.Low && first < interval.High) cuts[count++] = first;
                        if (last > interval.Low && last < interval.High) cuts[count++] = last;
                    }
                    cuts[..count].Sort();
                    for (int i = 1; i < count; i++)
                    {
                        double low = cuts[i - 1], high = cuts[i];
                        double projection = projection0 + projectionDelta * (low + high) / 2;
                        double qx = ax, qy = ay, vx = dx, vy = dy;
                        if (projection >= 1) { qx -= ex; qy -= ey; }
                        else if (projection > 0)
                        {
                            qx -= projection0 * ex; qy -= projection0 * ey;
                            vx -= projectionDelta * ex; vy -= projectionDelta * ey;
                        }
                        double quadratic = vx * vx + vy * vy - interval.Quadratic;
                        double linear = 2 * (qx * vx + qy * vy) - interval.Linear;
                        void Check(double t)
                        {
                            double x = qx + vx * t, y = qy + vy * t;
                            double value = x * x + y * y - interval.RadiusSquared(t);
                            if (value < minimum) { minimum = value; witness = t; violated = true; }
                        }
                        Check(low); Check(high);
                        if (quadratic > 0)
                        {
                            double stationary = -linear / (2 * quadratic);
                            if (stationary > low && stationary < high) Check(stationary);
                        }
                    }
                }
            }
            parameter = witness;
            return violated;
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private void CheckCell(int c, int r, double x, double y, ref double bestSq)
        {
            var cellSegs = _grid[c, r];
            if (cellSegs == null) return;
            int count = cellSegs.Count;
            for (int i = 0; i < count; i++)
            {
                double d2 = DistanceToSegmentSq(x, y, _segments[cellSegs[i]]);
                if (d2 < bestSq) bestSq = d2;
            }
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private static double DistanceToSegmentSq(double x, double y, (PointD A, PointD B) seg)
        {
            var (a, b) = seg;
            double abx = b.x - a.x;
            double aby = b.y - a.y;
            double apx = x - a.x;
            double apy = y - a.y;
            double ab2 = abx * abx + aby * aby;
            double t = ab2 <= 1e-18 ? 0.0 : (apx * abx + apy * aby) / ab2;
            if (t < 0.0) t = 0.0;
            else if (t > 1.0) t = 1.0;
            double cx = a.x + abx * t;
            double cy = a.y + aby * t;
            double dx = x - cx;
            double dy = y - cy;
            return dx * dx + dy * dy;
        }
        internal int[] CandidateOrdinals(double x0, double y0, double x1, double y1, double radius)
        {
            if (_segments.Count == 0) return Array.Empty<int>();
            int c0 = Math.Clamp((int)Math.Floor((Math.Min(x0, x1) - radius - _minX) / _cellSizeX), 0, _cols - 1);
            int c1 = Math.Clamp((int)Math.Floor((Math.Max(x0, x1) + radius - _minX) / _cellSizeX), 0, _cols - 1);
            int r0 = Math.Clamp((int)Math.Floor((Math.Min(y0, y1) - radius - _minY) / _cellSizeY), 0, _rows - 1);
            int r1 = Math.Clamp((int)Math.Floor((Math.Max(y0, y1) + radius - _minY) / _cellSizeY), 0, _rows - 1);
            var candidates = new HashSet<int>();
            for (int c = c0; c <= c1; c++)
            for (int r = r0; r <= r1; r++)
                if (_grid[c, r] is { } cell) candidates.UnionWith(cell);
            return candidates.ToArray();
        }
    internal static List<Point3D> EnsureZFidelityFromBoundary(
        List<Point3D> path,
        LegacyWallClearanceGrid spatialIndex,
        double startDepth,
        double bottomRadius,
        double radiusSlope,
        double endZ,
        double zTolerance,
        VCarveCutterProfile? cutterProfile = null,
        Action<string>? diagnostic = null)
    {
        if (path.Count < 2 || zTolerance <= 0)
            return path;

        double startZ = -startDepth;
        double invSlope = 1.0 / radiusSlope;

        // Snap every retained vertex's Z to its analytical value first.
        var snapped = new List<Point3D>(path.Count);
        foreach (var p in path)
        {
            double z = AnalyticalZ(p.X, p.Y, spatialIndex, startZ, bottomRadius, invSlope, endZ, cutterProfile);
            snapped.Add(new Point3D(p.X, p.Y, z));
        }

        var refinement = cutterProfile?.RoundedTip == true && diagnostic is not null
            ? new ZRefinementDiagnostics() : null;
        var output = new List<Point3D>(snapped.Count * 2);
        output.Add(snapped[0]);
        for (int i = 0; i < snapped.Count - 1; i++)
        {
            try
            {
                BisectChordForZ(
                    snapped[i], snapped[i + 1], spatialIndex,
                    startZ, bottomRadius, invSlope, endZ, zTolerance,
                    output, depth: 0, cutterProfile, refinement);
            }
            catch (InvalidOperationException error)
            {
                throw new InvalidOperationException($"V-carve source chord {i}/{snapped.Count - 1}: " +
                    $"({snapped[i].X:R},{snapped[i].Y:R},{snapped[i].Z:R})->" +
                    $"({snapped[i + 1].X:R},{snapped[i + 1].Y:R},{snapped[i + 1].Z:R}). {error.Message}", error);
            }
        }
        if (refinement?.MaximumDepth > MaxZFidelityBisectDepth)
            diagnostic?.Invoke($"stage=depth-fidelity event=extended-refinement " +
                $"recursionDepth={refinement.MaximumDepth} budget={MaxRoundedZFidelityBisectDepth} " +
                $"extendedBranches={refinement.ExtendedBranches} sourcePoints={path.Count} outputPoints={output.Count} " +
                $"zTolerance={zTolerance:R} noseDiameter={cutterProfile!.ClearingDiameter:R}");
        return output;
    }

    private const int MaxZFidelityBisectDepth = 14;
    // At a hemisphere's equator its inverse depth has a square-root cusp. Halving
    // XY converges at half the cone's exponent, so permit twice the recursion
    // budget for rounded tools. Every accepted chord still needs the exact guard.
    private const int MaxRoundedZFidelityBisectDepth = 28;

    private sealed class ZRefinementDiagnostics
    {
        public int MaximumDepth;
        public int ExtendedBranches;
    }


    /// <summary>
    /// Splits any path segment whose |dz| exceeds the given cap, inserting
    /// midpoints with analytical Z. This bounds the error of the GPU swept-
    /// volume renderer, which projects the tool tip along the segment's XZ
    /// footprint only and otherwise leaves pillar-shaped gaps on V-carve ramps
    /// where dz is large. Flat (constant-Z) chords are left untouched because
    /// XZ projection is exact for them.
    /// </summary>
    internal static List<Point3D> EnforceMaxChordLength(
        List<Point3D> path,
        IReadOnlyList<(PointD A, PointD B)> boundarySegments,
        double startDepth,
        double bottomRadius,
        double radiusSlope,
        double endZ,
        double maxChordDz)
    {
        if (path.Count < 2 || boundarySegments.Count == 0 || maxChordDz <= 0)
            return path;

        double startZ = -startDepth;
        double invSlope = 1.0 / radiusSlope;

        var spatialIndex = new LegacyWallClearanceGrid(boundarySegments);

        var output = new List<Point3D>(path.Count * 2);
        output.Add(path[0]);
        for (int i = 1; i < path.Count; i++)
        {
            SubdivideChord(path[i - 1], path[i], spatialIndex,
                startZ, bottomRadius, invSlope, endZ,
                maxChordDz, output, depth: 0);
        }
        return output;
    }

    private const int MaxChordSubdivideDepth = 14;

    private static void SubdivideChord(
        Point3D a, Point3D b,
        LegacyWallClearanceGrid spatialIndex,
        double startZ, double bottomRadius, double invSlope, double endZ,
        double maxChordDz,
        List<Point3D> output, int depth)
    {
        double dz = b.Z - a.Z;
        if (Math.Abs(dz) <= maxChordDz || depth >= MaxChordSubdivideDepth)
        {
            output.Add(b);
            return;
        }

        double mx = 0.5 * (a.X + b.X);
        double my = 0.5 * (a.Y + b.Y);
        double mz = AnalyticalZ(mx, my, spatialIndex, startZ, bottomRadius, invSlope, endZ);
        var mid = new Point3D(mx, my, mz);
        SubdivideChord(a, mid, spatialIndex, startZ, bottomRadius, invSlope, endZ,
            maxChordDz, output, depth + 1);
        SubdivideChord(mid, b, spatialIndex, startZ, bottomRadius, invSlope, endZ,
            maxChordDz, output, depth + 1);
    }

    private static void BisectChordForZ(
        Point3D a,
        Point3D b,
        LegacyWallClearanceGrid spatialIndex,
        double startZ,
        double bottomRadius,
        double invSlope,
        double endZ,
        double zTolerance,
        List<Point3D> output,
        int depth,
        VCarveCutterProfile? cutterProfile = null,
        ZRefinementDiagnostics? refinement = null)
    {
        int maximumDepth = cutterProfile?.RoundedTip == true
            ? MaxRoundedZFidelityBisectDepth : MaxZFidelityBisectDepth;
        if (refinement is not null)
        {
            refinement.MaximumDepth = Math.Max(refinement.MaximumDepth, depth);
            if (depth == MaxZFidelityBisectDepth + 1) refinement.ExtendedBranches++;
        }
        double dx = b.X - a.X;
        double dy = b.Y - a.Y;
        if ((dx * dx + dy * dy) < 1e-18)
        {
            output.Add(b);
            return;
        }

        double mx = 0.5 * (a.X + b.X);
        double my = 0.5 * (a.Y + b.Y);
        double zChord = 0.5 * (a.Z + b.Z);
        double zAnalytical = AnalyticalZ(mx, my, spatialIndex, startZ, bottomRadius, invSlope, endZ, cutterProfile);

        bool midpointFits = Math.Abs(zChord - zAnalytical) <= zTolerance;
        double split = 0.5;
        // Both an off-center depth minimum and the surface clamp of a finite tip can
        // escape a midpoint test. Certify the entire cutting interval against the
        // original boundary before accepting it.
        bool overcuts = (midpointFits || depth >= maximumDepth)
            && spatialIndex.TryFindDepthViolation(a, b, startZ, bottomRadius,
                1.0 / invSlope, zTolerance, out split, cutterProfile);
        if (midpointFits && !overcuts)
        {
            output.Add(b);
            return;
        }

        if (depth >= maximumDepth)
        {
            if (overcuts)
                throw new InvalidOperationException($"V-carve depth refinement could not certify boundary clearance " +
                    $"within {zTolerance:R} mm at ({a.X:R},{a.Y:R},{a.Z:R})->({b.X:R},{b.Y:R},{b.Z:R}); " +
                    $"recursionDepth={depth} budget={maximumDepth} roundedTip={cutterProfile?.RoundedTip ?? false} " +
                    $"noseDiameter={cutterProfile?.ClearingDiameter ?? 0:R} profileDepth={cutterProfile?.MaximumDepth ?? 0:R}.");
            output.Add(b);
            return;
        }

        if (overcuts)
        {
            // Avoid repeatedly splitting at an endpoint when the minimum lies at the
            // open depth-tolerance boundary. Interior extrema are inserted directly.
            if (split < 0.01 || split > 0.99) split = 0.5;
            mx = a.X + split * dx;
            my = a.Y + split * dy;
            zAnalytical = AnalyticalZ(mx, my, spatialIndex, startZ, bottomRadius, invSlope, endZ, cutterProfile);
        }

        var mid = new Point3D(mx, my, zAnalytical);
        BisectChordForZ(a, mid, spatialIndex, startZ, bottomRadius, invSlope, endZ, zTolerance, output, depth + 1, cutterProfile, refinement);
        BisectChordForZ(mid, b, spatialIndex, startZ, bottomRadius, invSlope, endZ, zTolerance, output, depth + 1, cutterProfile, refinement);
    }

    private static double AnalyticalZ(
        double x,
        double y,
        LegacyWallClearanceGrid spatialIndex,
        double startZ,
        double bottomRadius,
        double invSlope,
        double endZ,
        VCarveCutterProfile? cutterProfile = null)
    {
        double r = spatialIndex.QueryDistance(x, y);
        double depthBelowStart = cutterProfile?.DepthAtRadius(r)
            ?? (Math.Max(0.0, r - bottomRadius) * invSlope);
        double z = startZ - depthBelowStart;
        if (z < endZ) z = endZ;
        return z;
    }


    }
