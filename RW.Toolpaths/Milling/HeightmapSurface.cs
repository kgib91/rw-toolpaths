using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Immutable scalar image. Samples are heights (white=1), never display pixels.</summary>
public sealed class HeightmapGrid
{
    private readonly float[] _samples;
    private readonly bool[] _mask;
    public int Width { get; }
    public int Height { get; }
    public HeightmapGrid(int width, int height, IReadOnlyList<float> samples, IReadOnlyList<bool>? mask = null)
    {
        if (width < 2 || height < 2 || (long)width * height > 16_777_216 || samples.Count != width * height)
            throw new ArgumentException("Heightmaps require 2..4096-scale dimensions and at most 16 million samples.");
        if (mask != null && mask.Count != samples.Count) throw new ArgumentException("Mask dimensions differ.");
        Width = width; Height = height;
        _samples = samples.ToArray();
        _mask = mask?.ToArray() ?? Enumerable.Repeat(true, samples.Count).ToArray();
        if (_samples.Any(v => !float.IsFinite(v) || v < 0 || v > 1))
            throw new ArgumentException("Height samples must be finite and normalized to [0,1].");
    }
    public float this[int x, int y] => _samples[y * Width + x];
    public bool IsIncluded(int x, int y) => _mask[y * Width + x];
}

/// <summary>Rigid placement of an already physically scaled image in the machining XY plane.</summary>
public sealed record HeightmapPlacement(double Width, double Height, double OriginX, double OriginY,
    double AngleDegrees = 0, bool MirrorX = false, bool MirrorY = false)
{
    public PointD ToWorld(double x, double y)
    {
        if (MirrorX) x = Width - x;
        if (MirrorY) y = Height - y;
        double a = AngleDegrees * Math.PI / 180, c = Math.Cos(a), s = Math.Sin(a);
        return new PointD(OriginX + c * x - s * y, OriginY + s * x + c * y);
    }
    public PointD ToLocal(double x, double y)
    {
        double a = AngleDegrees * Math.PI / 180, c = Math.Cos(a), s = Math.Sin(a);
        double lx = c * (x - OriginX) + s * (y - OriginY);
        double ly = -s * (x - OriginX) + c * (y - OriginY);
        return new PointD(MirrorX ? Width - lx : lx, MirrorY ? Height - ly : ly);
    }
}

/// <summary>Bilinear target surface with a maximum hierarchy for conservative cutter clearance.</summary>
public sealed class HeightmapSurface
{
    private sealed record Node(int X0, int Y0, int X1, int Y1, double Maximum, Node[] Children);
    private readonly double[] _heights;
    private readonly Node _root;
    public HeightmapGrid Grid { get; }
    public HeightmapPlacement Placement { get; }
    public double SurfaceZ { get; }
    public double PixelX { get; }
    public double PixelY { get; }
    public double Minimum { get; }

    public HeightmapSurface(HeightmapGrid grid, HeightmapPlacement placement, double surfaceZ, double depth, bool invert = false, CancellationToken cancellationToken = default)
    {
        if (!double.IsFinite(depth) || depth <= 0 || !double.IsFinite(surfaceZ)
            || !double.IsFinite(placement.Width) || !double.IsFinite(placement.Height)
            || !double.IsFinite(placement.OriginX) || !double.IsFinite(placement.OriginY) || !double.IsFinite(placement.AngleDegrees)
            || placement.Width <= 0 || placement.Height <= 0)
            throw new ArgumentException("Invalid heightmap physical dimensions or depth.");
        Grid = grid; Placement = placement; SurfaceZ = surfaceZ;
        PixelX = placement.Width / (grid.Width - 1); PixelY = placement.Height / (grid.Height - 1);
        _heights = new double[grid.Width * grid.Height];
        for (int y = 0; y < grid.Height; y++)
        {
        cancellationToken.ThrowIfCancellationRequested();
        for (int x = 0; x < grid.Width; x++)
            _heights[y * grid.Width + x] = grid.IsIncluded(x, y)
                ? surfaceZ - depth * (invert ? grid[x, y] : 1 - grid[x, y]) : surfaceZ;
        }
        Minimum = _heights.Min();
        _root = Build(0, 0, grid.Width - 1, grid.Height - 1, cancellationToken);
    }

    public double Sample(double x, double y)
    {
        if (x < -1e-9 || y < -1e-9 || x > Placement.Width + 1e-9 || y > Placement.Height + 1e-9) return SurfaceZ;
        x = Math.Clamp(x, 0, Placement.Width); y = Math.Clamp(y, 0, Placement.Height);
        int ix = Math.Min(Grid.Width - 2, (int)(x / PixelX));
        int iy = Math.Min(Grid.Height - 2, (int)(y / PixelY));
        if (!CellIncluded(ix, iy)) return SurfaceZ;
        double tx = x / PixelX - ix, ty = y / PixelY - iy;
        return Lerp(Lerp(H(ix, iy), H(ix + 1, iy), tx), Lerp(H(ix, iy + 1), H(ix + 1, iy + 1), tx), ty);
    }

    public bool CellIncluded(int x, int y) => Grid.IsIncluded(x, y) && Grid.IsIncluded(x + 1, y)
        && Grid.IsIncluded(x, y + 1) && Grid.IsIncluded(x + 1, y + 1);
    public double CellMaximum(int x, int y) => CellIncluded(x, y)
        ? Math.Max(Math.Max(H(x, y), H(x + 1, y)), Math.Max(H(x, y + 1), H(x + 1, y + 1))) : SurfaceZ;
    private double H(int x, int y) => _heights[y * Grid.Width + x];
    internal double VertexHeight(int x, int y) => H(x, y);
    private static double Lerp(double a, double b, double t) => a + (b - a) * t;

    private Node Build(int x0, int y0, int x1, int y1, CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        if (x1 - x0 <= 8 && y1 - y0 <= 8)
        {
            double maximum = double.NegativeInfinity;
            for (int y = y0; y < y1; y++) for (int x = x0; x < x1; x++) maximum = Math.Max(maximum, CellMaximum(x, y));
            return new Node(x0, y0, x1, y1, maximum, Array.Empty<Node>());
        }
        int xm = (x0 + x1) / 2, ym = (y0 + y1) / 2;
        var nodes = new List<Node>(4);
        int[] xs = x1 - x0 > 1 ? new[] { x0, xm, x1 } : new[] { x0, x1 };
        int[] ys = y1 - y0 > 1 ? new[] { y0, ym, y1 } : new[] { y0, y1 };
        for (int y = 1; y < ys.Length; y++)
        for (int x = 1; x < xs.Length; x++) nodes.Add(Build(xs[x - 1], ys[y - 1], xs[x], ys[y], cancellationToken));
        // Visit higher relief first so exact footprint samples tighten subsequent branch bounds.
        nodes.Sort((a, b) => b.Maximum.CompareTo(a.Maximum));
        return new Node(x0, y0, x1, y1, nodes[0].Maximum, nodes.ToArray());
    }

    /// <summary>Conservative vertical lift for a flat cutter following a sloped chord.
    /// Each surface patch bounds the range of cutter centers that can touch it. This
    /// retains the chord's slope instead of reducing every interval to a horizontal step.</summary>
    internal double FlatCutterChordLift(PointD a, double za, PointD b, double zb, double radius,
        double tolerance, CancellationToken cancellationToken, double horizontalMargin, CarvingCutterProfile? cutter = null)
    {
        double dx = b.x - a.x, dy = b.y - a.y, length = Math.Sqrt(dx * dx + dy * dy);
        if (length < 1e-12)
            return Math.Max(0, CutterUpperBound(a, b, radius, false, tolerance / 8,
                cancellationToken, horizontalMargin, cutter) - Math.Min(za, zb));
        dx /= length; dy /= length;
        double slope = (zb - za) / length;
        double margin = horizontalMargin * (Math.Abs(dx) + Math.Abs(dy));
        double result = 0;
        int visited = 0;
        double Bottom(double x0, double y0, double x1, double y1)
        {
            if (cutter == null) return 0;
            double rx = Math.Max(0, Math.Max(x0 - Math.Max(a.x, b.x) - horizontalMargin, Math.Min(a.x, b.x) - horizontalMargin - x1));
            double ry = Math.Max(0, Math.Max(y0 - Math.Max(a.y, b.y) - horizontalMargin, Math.Min(a.y, b.y) - horizontalMargin - y1));
            return cutter.Bottom(Math.Sqrt(rx * rx + ry * ry));
        }
        bool Range(double x0, double y0, double x1, double y1, double padding, out double minimumZ)
        {
            double mx = (x0 + x1) / 2 - a.x, my = (y0 + y1) / 2 - a.y;
            double hx = (x1 - x0) / 2, hy = (y1 - y0) / 2;
            double u = dx * mx + dy * my, v = -dy * mx + dx * my;
            double hu = Math.Abs(dx) * hx + Math.Abs(dy) * hy + padding;
            double hv = Math.Abs(dy) * hx + Math.Abs(dx) * hy + padding;
            double perpendicular = Math.Max(0, Math.Abs(v) - hv);
            minimumZ = 0;
            if (perpendicular > radius) return false;
            double reach = Math.Sqrt(Math.Max(0, radius * radius - perpendicular * perpendicular));
            double low = Math.Max(0, u - hu - reach), high = Math.Min(length, u + hu + reach);
            if (low > high) return false;
            minimumZ = za + slope * (slope >= 0 ? low : high);
            return true;
        }
        void Witness(double x, double y, double h)
        {
            if (cutter != null)
            {
                double t = Math.Clamp((x - a.x) * dx + (y - a.y) * dy, 0, length);
                double rx = x - a.x - t * dx, ry = y - a.y - t * dy;
                if (h - za - slope * t - cutter.Bottom(Math.Sqrt(rx * rx + ry * ry)) > tolerance)
                    result = double.PositiveInfinity;
                return;
            }
            // A real point under the unrounded cutter can reject a chord immediately.
            if (Range(x, y, x, y, 0, out double z) && h - z > tolerance)
                result = double.PositiveInfinity;
        }
        bool ExactRange(double x0, double y0, double x1, double y1, out double minimumZ)
        {
            // Intersect the center line with this axis-aligned patch expanded by the
            // rounding square and disk. Two strips and four disks describe that rounded
            // rectangle exactly, avoiding the looser box in rotated scan coordinates.
            x0 -= horizontalMargin; x1 += horizontalMargin;
            y0 -= horizontalMargin; y1 += horizontalMargin;
            double low = double.PositiveInfinity, high = double.NegativeInfinity;
            double safeRadius = radius + 1e-10;
            void Include(double lo, double hi)
            {
                lo = Math.Max(0, lo); hi = Math.Min(length, hi);
                if (lo <= hi) { low = Math.Min(low, lo); high = Math.Max(high, hi); }
            }
            void Disk(double x, double y)
            {
                double px = x - a.x, py = y - a.y;
                double along = px * dx + py * dy, perpendicular = -px * dy + py * dx;
                double square = safeRadius * safeRadius - perpendicular * perpendicular;
                if (square < 0) return;
                double reach = Math.Sqrt(square); Include(along - reach, along + reach);
            }
            void Box(double bx0, double by0, double bx1, double by1)
            {
                double lo = 0, hi = length;
                if (Math.Abs(dx) < 1e-15) { if (a.x < bx0 || a.x > bx1) return; }
                else { double u = (bx0 - a.x) / dx, v = (bx1 - a.x) / dx; lo = Math.Max(lo, Math.Min(u, v)); hi = Math.Min(hi, Math.Max(u, v)); }
                if (Math.Abs(dy) < 1e-15) { if (a.y < by0 || a.y > by1) return; }
                else { double u = (by0 - a.y) / dy, v = (by1 - a.y) / dy; lo = Math.Max(lo, Math.Min(u, v)); hi = Math.Min(hi, Math.Max(u, v)); }
                Include(lo, hi);
            }
            Box(x0 - safeRadius, y0, x1 + safeRadius, y1);
            Box(x0, y0 - safeRadius, x1, y1 + safeRadius);
            Disk(x0, y0); Disk(x1, y0); Disk(x0, y1); Disk(x1, y1);
            minimumZ = za + slope * (slope >= 0 ? low : high);
            return low <= high;
        }
        void Patch(double x0, double y0, double x1, double y1,
            double h00, double h10, double h01, double h11, int level)
        {
            if (result > tolerance) return;
            if ((++visited & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            // A conservative rejection is cheap and lets the caller split the chord.
            // Do not spend unbounded patch work trying to prove a loose projected bound.
            if (visited > 512) { result = double.PositiveInfinity; return; }
            if (!Range(x0, y0, x1, y1, margin, out double z)) return;
            double cutterBottom = Bottom(x0, y0, x1, y1);
            double upper = Math.Max(Math.Max(h00, h10), Math.Max(h01, h11)) - z - cutterBottom;
            if (cutter != null && upper > tolerance)
                upper = Math.Min(upper, cutter.PatchUpperBound(x0, y0, x1, y1, h00, h10, h01, h11,
                    a.x, a.y, za, b.x, b.y, zb, horizontalMargin));
            if (upper <= tolerance) { result = Math.Max(result, upper); return; }
            // Reject a real collision before evaluating tighter (more expensive) upper
            // bounds. Conversely, the simple bound above is sufficient when it passes.
            Witness(x0, y0, h00); Witness(x1, y0, h10);
            Witness(x0, y1, h01); Witness(x1, y1, h11);
            if (result > tolerance) return;
            // Subtracting the chord plane preserves bilinearity, whose extrema are at
            // corners. Keep height and along-chord position correlated on steep slopes.
            double Residual(double x, double y, double h) => h - za - slope * (dx * (x - a.x) + dy * (y - a.y));
            double residual = Math.Max(Math.Max(Residual(x0, y0, h00), Residual(x1, y0, h10)),
                Math.Max(Residual(x0, y1, h01), Residual(x1, y1, h11)));
            double v = -dy * ((x0 + x1) / 2 - a.x) + dx * ((y0 + y1) / 2 - a.y);
            double hv = Math.Abs(dy) * (x1 - x0) / 2 + Math.Abs(dx) * (y1 - y0) / 2 + margin;
            double perpendicular = Math.Max(0, Math.Abs(v) - hv);
            double reach = Math.Sqrt(Math.Max(0, radius * radius - perpendicular * perpendicular));
            upper = Math.Min(upper, residual + Math.Abs(slope) * (reach + margin) - cutterBottom);
            double gx = (h10 + h11 - h00 - h01) / (2 * (x1 - x0));
            double gy = (h01 + h11 - h00 - h10) / (2 * (y1 - y0));
            double saddle = Math.Abs(h00 - h10 - h01 + h11) / 4;
            // Support of the patch's tangent plane over the complete swept disk and
            // original axis-aligned rounding square. Keep both components of the relief
            // gradient: on a planar slope this is exact, including angled scanlines.
            double planeUpper = (h00 + h10 + h01 + h11) / 4 - za
                + gx * (a.x - (x0 + x1) / 2) + gy * (a.y - (y0 + y1) / 2)
                + Math.Max(0, (gx * dx + gy * dy - slope) * length)
                + (cutter?.Support(Math.Sqrt(gx * gx + gy * gy)) ?? radius * Math.Sqrt(gx * gx + gy * gy))
                + horizontalMargin * (Math.Abs(gx) + Math.Abs(gy)) + saddle;
            upper = Math.Min(upper, planeUpper);
            if (upper > tolerance && horizontalMargin > 0 && Math.Abs(dx * dy) > 1e-12)
            {
                if (!ExactRange(x0, y0, x1, y1, out double exactZ)) return;
                upper = Math.Min(upper, Math.Max(Math.Max(h00, h10), Math.Max(h01, h11)) - exactZ - cutterBottom);
            }
            if (upper <= tolerance || level >= 18) { result = Math.Max(result, upper); return; }
            double mx = (x0 + x1) / 2, my = (y0 + y1) / 2;
            double top = (h00 + h10) / 2, bottom = (h01 + h11) / 2;
            double left = (h00 + h01) / 2, right = (h10 + h11) / 2;
            double center = (top + bottom) / 2;
            Patch(x0, y0, mx, my, h00, top, left, center, level + 1);
            Patch(mx, y0, x1, my, top, h10, center, right, level + 1);
            Patch(x0, my, mx, y1, left, center, h01, bottom, level + 1);
            Patch(mx, my, x1, y1, center, right, bottom, h11, level + 1);
        }
        void Visit(Node node)
        {
            if (result > tolerance) return;
            if ((++visited & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            if (visited > 512) { result = double.PositiveInfinity; return; }
            if (!Range(node.X0 * PixelX, node.Y0 * PixelY, node.X1 * PixelX, node.Y1 * PixelY, margin, out double z)) return;
            double upper = node.Maximum - z - Bottom(node.X0 * PixelX, node.Y0 * PixelY, node.X1 * PixelX, node.Y1 * PixelY);
            if (upper <= tolerance) { result = Math.Max(result, upper); return; }
            if (node.Children.Length != 0) { foreach (var child in node.Children) Visit(child); return; }
            for (int y = node.Y0; y < node.Y1 && result <= tolerance; y++)
            for (int x = node.X0; x < node.X1 && result <= tolerance; x++)
            {
                if (CellIncluded(x, y))
                    Patch(x * PixelX, y * PixelY, (x + 1) * PixelX, (y + 1) * PixelY,
                        H(x, y), H(x + 1, y), H(x, y + 1), H(x + 1, y + 1), 0);
                else if (Range(x * PixelX, y * PixelY, (x + 1) * PixelX, (y + 1) * PixelY, margin, out z))
                    result = Math.Max(result, SurfaceZ - z - Bottom(x * PixelX, y * PixelY, (x + 1) * PixelX, (y + 1) * PixelY));
            }
        }
        Visit(_root);
        // Traversal stops as soon as a patch exceeds the requested bound. Remaining
        // patches are unvisited, so that result is a rejection, not a usable finite lift.
        // The image exterior remains protected stock. Bound all exterior portions
        // throughout the chord, including the wider taper near the image edge.
        if (cutter != null)
        {
            double edge = Math.Max(0, Math.Min(Math.Min(a.x, b.x), Math.Min(Math.Min(a.y, b.y),
                Math.Min(Placement.Width - Math.Max(a.x, b.x), Placement.Height - Math.Max(a.y, b.y)))) - horizontalMargin);
            if (edge <= radius)
            {
                double exterior = SurfaceZ - Math.Min(za, zb) - cutter.Bottom(edge);
                if (exterior > tolerance)
                {
                    // The image exterior is the union of four half-planes. The nearest
                    // radius to each half-plane varies linearly along this chord; keep
                    // that distance correlated with tip Z. Reuse exact planar support
                    // on a degenerate rectangle to include every physical profile section.
                    double Edge(double da, double db)
                    {
                        da -= horizontalMargin; db -= horizontalMargin;
                        double lo = Math.Min(da, db), hi = Math.Max(da, db);
                        if (lo > radius) return double.NegativeInfinity;
                        double delta = db - da;
                        if (Math.Abs(delta) < 1e-12)
                            return SurfaceZ - Math.Min(za, zb) - cutter.Bottom(Math.Max(0, lo));
                        double edgeSlope = (zb - za) / delta;
                        double bound = double.NegativeInfinity;
                        if (lo < 0)
                        {
                            double negativeHi = Math.Min(0, hi);
                            double d = edgeSlope >= 0 ? lo : negativeHi;
                            bound = SurfaceZ - za - edgeSlope * (d - da);
                        }
                        lo = Math.Max(0, lo); hi = Math.Min(radius, hi);
                        if (lo <= hi)
                        {
                            // Shift the support coordinates to the first endpoint so
                            // nearly equal edge distances do not subtract large products.
                            double support = cutter.ClippedPlaneSupport(-edgeSlope, 0,
                                lo - da, 0, hi - da, 0, -da, 0, -da, 0, out _, out _);
                            bound = Math.Max(bound, SurfaceZ - za + support + 1e-8);
                        }
                        return bound;
                    }
                    double correlated = Math.Max(Math.Max(Edge(a.x, b.x), Edge(a.y, b.y)),
                        Math.Max(Edge(Placement.Width - a.x, Placement.Width - b.x),
                            Edge(Placement.Height - a.y, Placement.Height - b.y)));
                    exterior = Math.Min(exterior, correlated);
                }
                result = Math.Max(result, exterior);
            }
        }
        return result <= tolerance ? result : double.PositiveInfinity;
    }

    /// <summary>Upper bound on required tip height everywhere in an XY segment's bounding box.
    /// Subdivision bounds the bilinear cells and cutter envelope, including narrow unsampled peaks.</summary>
    public double CutterUpperBound(PointD a, PointD b, double radius, bool ballNose, double tolerance,
        CancellationToken cancellationToken = default, double horizontalMargin = 0, CarvingCutterProfile? cutter = null)
    {
        if (!(radius > 0) || !(tolerance > 0) || horizontalMargin < 0 || !double.IsFinite(radius + tolerance + horizontalMargin))
            throw new ArgumentException("Invalid cutter clearance bounds.");
        double qx0 = Math.Min(a.x, b.x) - horizontalMargin, qx1 = Math.Max(a.x, b.x) + horizontalMargin;
        double qy0 = Math.Min(a.y, b.y) - horizontalMargin, qy1 = Math.Max(a.y, b.y) + horizontalMargin;
        if (cutter == null && (qx0 < radius - 1e-8 || qy0 < radius - 1e-8 || qx1 > Placement.Width - radius + 1e-8 || qy1 > Placement.Height - radius + 1e-8))
            return SurfaceZ;
        double best = Math.Max(Sample(a.x, a.y), Sample(b.x, b.y));
        double upperResult = best;
        int visited = 0;
        double Bottom(double r) => cutter?.Bottom(r) ?? (ballNose ? radius - Math.Sqrt(Math.Max(0, radius * radius - r * r)) : 0);
        if (cutter != null)
        {
            double edge = Math.Max(0, Math.Min(Math.Min(qx0, qy0), Math.Min(Placement.Width - qx1, Placement.Height - qy1)));
            if (edge <= radius) best = upperResult = Math.Max(best, SurfaceZ - Bottom(edge));
        }
        double Distance(double x0, double y0, double x1, double y1)
        {
            double dx = Math.Max(0, Math.Max(x0 - qx1, qx0 - x1));
            double dy = Math.Max(0, Math.Max(y0 - qy1, qy0 - y1));
            return Math.Sqrt(dx * dx + dy * dy);
        }
        void Patch(double x0, double y0, double x1, double y1, double h00, double h10, double h01, double h11, int level)
        {
            if ((++visited & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            double r = Distance(x0, y0, x1, y1);
            if (r > radius) return;
            double upper = Math.Max(Math.Max(h00, h10), Math.Max(h01, h11)) - Bottom(r);
            if (cutter != null && upper > best + tolerance)
                upper = Math.Min(upper, cutter.PatchUpperBound(x0, y0, x1, y1, h00, h10, h01, h11,
                    qx0, qy0, 0, qx1, qy1, 0, 0, queryRectangle: true));
            if (upper <= best + tolerance) { upperResult = Math.Max(upperResult, upper); return; }
            if (cutter != null)
            {
                double gx = (h10 + h11 - h00 - h01) / (2 * (x1 - x0));
                double gy = (h01 + h11 - h00 - h10) / (2 * (y1 - y0));
                double gradient = Math.Sqrt(gx * gx + gy * gy);
                double support = cutter.Support(gradient, out double supportRadius);
                double plane = (h00 + h10 + h01 + h11) / 4
                    + gx * ((gx >= 0 ? qx1 : qx0) - (x0 + x1) / 2)
                    + gy * ((gy >= 0 ? qy1 : qy0) - (y0 + y1) / 2)
                    + support + Math.Abs(h00 - h10 - h01 + h11) / 4;
                upper = Math.Min(upper, plane);
                void Witness(double x, double y)
                {
                    if (x < x0 || x > x1 || y < y0 || y > y1) return;
                    double radial = Distance(x, y, x, y);
                    if (radial > radius) return;
                    double tx = (x - x0) / (x1 - x0), ty = (y - y0) / (y1 - y0);
                    best = Math.Max(best, Lerp(Lerp(h00, h10, tx), Lerp(h01, h11, tx), ty) - Bottom(radial));
                }
                // Closest points matter on steep pixel edges; corner-only witnesses
                // miss a peak beside the middle of a cell and force needless subdivisions.
                Witness(Math.Clamp(qx0, x0, x1), Math.Clamp(qy0, y0, y1));
                Witness(Math.Clamp(qx1, x0, x1), Math.Clamp(qy1, y0, y1));
                if (gradient > 0)
                    Witness((gx >= 0 ? qx1 : qx0) + gx * supportRadius / gradient,
                        (gy >= 0 ? qy1 : qy0) + gy * supportRadius / gradient);
                if (upper > best + tolerance)
                {
                    double clipped = cutter.ClippedPlaneSupport(gx, gy, x0, y0, x1, y1,
                        qx0, qy0, qx1, qy1, out double wx, out double wy);
                    if (double.IsFinite(clipped))
                    {
                        upper = Math.Min(upper, (h00 + h10 + h01 + h11) / 4 - gx * (x0 + x1) / 2
                            - gy * (y0 + y1) / 2 + clipped + Math.Abs(h00 - h10 - h01 + h11) / 4 + 1e-9);
                        Witness(wx, wy);
                    }
                }
            }
            if (!ballNose && cutter == null)
            {
                // A bilinear patch is its tangent plane at the center plus a bounded
                // saddle term. The plane's support over the rounded query rectangle
                // is analytic; avoid subdividing planar slopes towards a cutter edge.
                double mx0 = (x0 + x1) / 2, my0 = (y0 + y1) / 2;
                double gx = (h10 + h11 - h00 - h01) / (2 * (x1 - x0));
                double gy = (h01 + h11 - h00 - h10) / (2 * (y1 - y0));
                double saddle = Math.Abs(h00 - h10 - h01 + h11) / 4;
                double centerHeight = (h00 + h10 + h01 + h11) / 4;
                double planeMaximum = double.NegativeInfinity;
                void Candidate(double x, double y)
                {
                    double plane = centerHeight + gx * (x - mx0) + gy * (y - my0);
                    planeMaximum = Math.Max(planeMaximum, plane);
                    // Every candidate is inside both the patch and cutter footprint.
                    // Its bilinear height is also a lower bound for branch pruning.
                    double tx = Math.Clamp((x - x0) / (x1 - x0), 0, 1);
                    double ty = Math.Clamp((y - y0) / (y1 - y0), 0, 1);
                    best = Math.Max(best, Lerp(Lerp(h00, h10, tx), Lerp(h01, h11, tx), ty));
                }
                void VerticalEdge(double x)
                {
                    double distance = Math.Max(0, Math.Max(qx0 - x, x - qx1));
                    if (distance > radius) return;
                    double reach = Math.Sqrt(Math.Max(0, radius * radius - distance * distance));
                    double lo = Math.Max(y0, qy0 - reach), hi = Math.Min(y1, qy1 + reach);
                    if (lo <= hi) Candidate(x, gy >= 0 ? hi : lo);
                }
                void HorizontalEdge(double y)
                {
                    double distance = Math.Max(0, Math.Max(qy0 - y, y - qy1));
                    if (distance > radius) return;
                    double reach = Math.Sqrt(Math.Max(0, radius * radius - distance * distance));
                    double lo = Math.Max(x0, qx0 - reach), hi = Math.Min(x1, qx1 + reach);
                    if (lo <= hi) Candidate(gx >= 0 ? hi : lo, y);
                }
                VerticalEdge(x0); VerticalEdge(x1); HorizontalEdge(y0); HorizontalEdge(y1);
                double gradient = Math.Sqrt(gx * gx + gy * gy);
                double supportX = gradient > 0 ? (gx >= 0 ? qx1 : qx0) + radius * gx / gradient : Math.Clamp(qx0, x0, x1);
                double supportY = gradient > 0 ? (gy >= 0 ? qy1 : qy0) + radius * gy / gradient : Math.Clamp(qy0, y0, y1);
                if (supportX >= x0 && supportX <= x1 && supportY >= y0 && supportY <= y1)
                    Candidate(supportX, supportY);
                // A linear maximum over the intersection is either the unconstrained
                // footprint support or on one of the four rectangle edges.
                if (double.IsFinite(planeMaximum)) upper = Math.Min(upper, planeMaximum + saddle);
            }
            if (upper <= best + tolerance) { upperResult = Math.Max(upperResult, upper); return; }
            // Corners are exact samples of the bilinear patch. Seeding the lower bound with
            // them avoids repeatedly subdividing toward a peak already represented by a pixel.
            void IncludeCorner(double x, double y, double h)
            {
                double distance = Distance(x, y, x, y);
                if (distance <= radius) best = Math.Max(best, h - Bottom(distance));
            }
            IncludeCorner(x0, y0, h00); IncludeCorner(x1, y0, h10);
            IncludeCorner(x0, y1, h01); IncludeCorner(x1, y1, h11);
            double mx = (x0 + x1) / 2, my = (y0 + y1) / 2;
            double center = (h00 + h10 + h01 + h11) / 4;
            double cr = Distance(mx, my, mx, my);
            if (cr <= radius) best = Math.Max(best, center - Bottom(cr));
            if (level >= 18 || upper - best <= tolerance)
            { upperResult = Math.Max(upperResult, upper); return; }
            double top = (h00 + h10) / 2, bottom = (h01 + h11) / 2;
            double left = (h00 + h01) / 2, right = (h10 + h11) / 2;
            Patch(x0, y0, mx, my, h00, top, left, center, level + 1);
            Patch(mx, y0, x1, my, top, h10, center, right, level + 1);
            Patch(x0, my, mx, y1, left, center, h01, bottom, level + 1);
            Patch(mx, my, x1, y1, center, right, bottom, h11, level + 1);
        }
        void Visit(Node node)
        {
            if ((++visited & 255) == 0) cancellationToken.ThrowIfCancellationRequested();
            double x0 = node.X0 * PixelX, x1 = node.X1 * PixelX;
            double y0 = node.Y0 * PixelY, y1 = node.Y1 * PixelY;
            double r = Distance(x0, y0, x1, y1);
            if (r > radius) return;
            double upper = node.Maximum - Bottom(r);
            if (upper <= best + tolerance) { upperResult = Math.Max(upperResult, upper); return; }
            if (node.Children.Length != 0) { foreach (var child in node.Children) Visit(child); return; }
            for (int y = node.Y0; y < node.Y1; y++)
            for (int x = node.X0; x < node.X1; x++)
            {
                double cellRadius = Distance(x * PixelX, y * PixelY, (x + 1) * PixelX, (y + 1) * PixelY);
                if (cellRadius > radius) continue;
                if (!CellIncluded(x, y)) { upperResult = Math.Max(upperResult, SurfaceZ - Bottom(cellRadius)); continue; }
                Patch(x * PixelX, y * PixelY, (x + 1) * PixelX, (y + 1) * PixelY,
                    H(x, y), H(x + 1, y), H(x, y + 1), H(x + 1, y + 1), 0);
            }
        }
        Visit(_root);
        return Math.Min(SurfaceZ, Math.Max(best, upperResult));
    }
}
