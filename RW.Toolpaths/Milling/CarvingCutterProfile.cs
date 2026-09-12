namespace RW.Toolpaths.Milling;

public enum CarvingCutterSectionKind { Linear, Hemisphere }

/// <summary>One physical section, ordered from the cutter tip upward.</summary>
public sealed record CarvingCutterSection(CarvingCutterSectionKind Kind, double BottomRadius, double TopRadius, double Height);

/// <summary>Analytic lower envelope of a revolved cutter. The depth limit excludes
/// sections that cannot reach material, without replacing the cutting tip by the shaft diameter.</summary>
public sealed class CarvingCutterProfile
{
    private readonly CarvingCutterSection[] _sections;
    public double TipRadius { get; }
    public double MaximumRadius { get; }
    public double MaximumContactHeight { get; }
    public bool IsFlatCylinder { get; }
    public bool IsStraightBall { get; }

    public CarvingCutterProfile(IEnumerable<CarvingCutterSection> sections, double maximumContactHeight)
    {
        _sections = sections.ToArray();
        if (_sections.Length == 0 || !double.IsFinite(maximumContactHeight) || maximumContactHeight <= 0)
            throw new ArgumentException("A carving cutter requires a physical profile and positive contact depth.");
        foreach (var section in _sections)
        {
            if (!Enum.IsDefined(typeof(CarvingCutterSectionKind), section.Kind)
                || !double.IsFinite(section.Height + section.BottomRadius + section.TopRadius)
                || section.Height <= 0 || section.BottomRadius < 0 || section.TopRadius < 0
                || Math.Max(section.BottomRadius, section.TopRadius) <= 0
                || (section.Kind == CarvingCutterSectionKind.Hemisphere
                    && (section.BottomRadius != 0 || section.Height != section.TopRadius)))
                throw new ArgumentException("Invalid carving cutter section dimensions.");
        }
        MaximumContactHeight = maximumContactHeight;
        var tip = _sections[0];
        TipRadius = tip.BottomRadius > 0 ? tip.BottomRadius : tip.TopRadius;
        MaximumRadius = MaxRadiusBelow(maximumContactHeight);
        IsFlatCylinder = tip.Kind == CarvingCutterSectionKind.Linear && tip.BottomRadius > 0
            && tip.BottomRadius == tip.TopRadius && MaximumRadius <= tip.BottomRadius;
        IsStraightBall = tip.Kind == CarvingCutterSectionKind.Hemisphere && MaximumRadius <= tip.TopRadius;
    }

    /// <summary>Lowest cutter material at radial distance r from its axis.</summary>
    public double Bottom(double radius)
    {
        double z = 0;
        foreach (var section in _sections)
        {
            if (radius <= section.BottomRadius) return z;
            if (radius <= section.TopRadius)
                return z + (section.Kind == CarvingCutterSectionKind.Hemisphere
                    ? section.TopRadius - Math.Sqrt(Math.Max(0, section.TopRadius * section.TopRadius - radius * radius))
                    : section.Height * (radius - section.BottomRadius) / (section.TopRadius - section.BottomRadius));
            z += section.Height;
        }
        return double.PositiveInfinity;
    }

    /// <summary>Largest physical radius anywhere between the tip and this axial height.</summary>
    public double MaxRadiusBelow(double height)
    {
        if (height < 0) return 0;
        double z = 0, radius = 0;
        foreach (var section in _sections)
        {
            if (z > height) break;
            double h = Math.Min(section.Height, height - z);
            double top = section.Kind == CarvingCutterSectionKind.Hemisphere
                ? Math.Sqrt(Math.Max(0, h * (2 * section.TopRadius - h)))
                : section.BottomRadius + (section.TopRadius - section.BottomRadius) * h / section.Height;
            radius = Math.Max(radius, Math.Max(section.BottomRadius, top));
            z += section.Height;
        }
        return radius;
    }

    /// <summary>Maximum of gradient*r - cutterHeight. Exact for every linear or
    /// hemispherical section; used to bound a sloped surface over the whole cutter.</summary>
    public double Support(double gradient)
        => Support(gradient, out _);

    /// <summary>Bounds a bilinear patch against supporting planes beneath each convex
    /// cutter section. The section domains may overlap; their maximum remains conservative.
    /// A swept center interval and its tip slope stay correlated in the same plane.</summary>
    internal double PatchUpperBound(double x0, double y0, double x1, double y1,
        double h00, double h10, double h01, double h11,
        double ax, double ay, double az, double bx, double by, double bz, double margin, bool queryRectangle = false)
    {
        double qx0 = Math.Min(ax, bx) - margin, qx1 = Math.Max(ax, bx) + margin;
        double qy0 = Math.Min(ay, by) - margin, qy1 = Math.Max(ay, by) + margin;
        double nearX = Math.Max(0, Math.Max(x0 - qx1, qx0 - x1));
        double nearY = Math.Max(0, Math.Max(y0 - qy1, qy0 - y1));
        double near = Math.Sqrt(nearX * nearX + nearY * nearY);
        if (!queryRectangle)
            near = SegmentBoxDistance(ax, ay, bx, by, x0 - margin, y0 - margin, x1 + margin, y1 + margin);
        double farX = Math.Max(Math.Abs(x0 - qx1), Math.Abs(x1 - qx0));
        double farY = Math.Max(Math.Abs(y0 - qy1), Math.Abs(y1 - qy0));
        double far = Math.Sqrt(farX * farX + farY * farY);
        double cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
        double vx = cx - Math.Clamp(cx, qx0, qx1), vy = cy - Math.Clamp(cy, qy0, qy1);
        double radial = Math.Sqrt(vx * vx + vy * vy);
        double sourceGx = (h10 + h11 - h00 - h01) / (2 * (x1 - x0));
        double sourceGy = (h01 + h11 - h00 - h10) / (2 * (y1 - y0));
        double lineX = bx - ax, lineY = by - ay, lineLength2 = lineX * lineX + lineY * lineY;
        double correction = !queryRectangle && lineLength2 > 1e-24
            ? (bz - az - sourceGx * lineX - sourceGy * lineY) / lineLength2 : 0;
        double lineGx = sourceGx + correction * lineX, lineGy = sourceGy + correction * lineY;
        double sweepLength = !queryRectangle ? Math.Sqrt(lineLength2) : 0;
        double sweepX = sweepLength > 1e-12 ? lineX / sweepLength : 0;
        double sweepY = sweepLength > 1e-12 ? lineY / sweepLength : 0;
        double sweepSlope = sweepLength > 1e-12 ? (bz - az) / sweepLength : 0;
        double sweepAlong = (cx - ax) * sweepX + (cy - ay) * sweepY;
        double sweepAcross = (cy - ay) * sweepX - (cx - ax) * sweepY;
        double maximum = double.NegativeInfinity, z = 0;
        double Plane(double gx, double gy, double intercept)
        {
            double patch = Math.Max(Math.Max(h00 - gx * x0 - gy * y0, h10 - gx * x1 - gy * y0),
                Math.Max(h01 - gx * x0 - gy * y1, h11 - gx * x1 - gy * y1));
            double centers = Math.Max(gx * ax + gy * ay - az, gx * bx + gy * by - bz);
            if (queryRectangle) centers = Math.Max(gx * ax, gx * bx) + Math.Max(gy * ay, gy * by) - Math.Min(az, bz);
            return patch + centers - intercept + margin * (Math.Abs(gx) + Math.Abs(gy));
        }
        foreach (var section in _sections)
        {
            if (z > MaximumContactHeight) break;
            double height = Math.Min(section.Height, MaximumContactHeight - z);
            double Dual(double gx, double gy) => Plane(gx, gy,
                -SectionSupport(section, z, height, Math.Sqrt(gx * gx + gy * gy), out _));
            // Rectangle queries have the same two dual planes. Only evaluate a
            // section's supports after its radial domain can reach this patch.
            double DualBound() => queryRectangle ? Dual(sourceGx, sourceGy)
                : Math.Min(Dual(sourceGx, sourceGy), Dual(lineGx, lineGy));
            if (section.Kind == CarvingCutterSectionKind.Hemisphere)
            {
                double r = section.TopRadius;
                double outer = Math.Sqrt(Math.Max(0, height * (2 * r - height)));
                if (near <= outer)
                {
                    double tangentRadius = Math.Min(radial, r * (1 - 1e-8));
                    double root = Math.Sqrt(r * r - tangentRadius * tangentRadius);
                    double scale = radial > 0 ? tangentRadius / (root * radial) : 0;
                    double bound = Math.Min(DualBound(), Plane(scale * vx, scale * vy, z + r - r * r / root));
                    if (sweepLength > 1e-12)
                    {
                        // At the source patch center, minimize tip Z plus this sphere's
                        // underside along the actual sloped segment. Its tangent remains
                        // a lower support for the full spherical section and entire patch.
                        double t = Math.Clamp(sweepAlong - sweepSlope * Math.Sqrt(
                            Math.Max(0, r * r - sweepAcross * sweepAcross) / (1 + sweepSlope * sweepSlope)), 0, sweepLength);
                        double rx = cx - ax - t * sweepX, ry = cy - ay - t * sweepY;
                        double radial2 = rx * rx + ry * ry;
                        double contact = Math.Min(Math.Sqrt(radial2), r * (1 - 1e-8));
                        double sr = Math.Sqrt(r * r - contact * contact);
                        double factor = radial2 > 0 ? contact / (sr * Math.Sqrt(radial2)) : 0;
                        bound = Math.Min(bound, Plane(factor * rx, factor * ry, z + r - r * r / sr));
                    }
                    maximum = Math.Max(maximum, bound);
                }
            }
            else
            {
                double outer = section.BottomRadius + (section.TopRadius - section.BottomRadius) * height / section.Height;
                if (near <= Math.Max(section.BottomRadius, outer))
                {
                    double bound = Math.Min(DualBound(), Plane(0, 0, z));
                    if (outer > section.BottomRadius && far >= section.BottomRadius)
                    {
                        double slope = section.Height / (section.TopRadius - section.BottomRadius);
                        double scale = radial > 0 ? slope / radial : 0;
                        bound = Math.Min(bound, Plane(scale * vx, scale * vy, z - slope * section.BottomRadius));
                        if (sweepLength > 1e-12)
                        {
                            double t = sweepSlope >= slope ? 0 : sweepSlope <= -slope ? sweepLength
                                : Math.Clamp(sweepAlong - sweepSlope * Math.Abs(sweepAcross)
                                    / Math.Sqrt(slope * slope - sweepSlope * sweepSlope), 0, sweepLength);
                            double rx = cx - ax - t * sweepX, ry = cy - ay - t * sweepY;
                            double distance = Math.Sqrt(rx * rx + ry * ry);
                            double gx = distance > 1e-14 ? slope * rx / distance : Math.Clamp(sweepSlope, -slope, slope) * sweepX;
                            double gy = distance > 1e-14 ? slope * ry / distance : Math.Clamp(sweepSlope, -slope, slope) * sweepY;
                            bound = Math.Min(bound, Plane(gx, gy, z - slope * section.BottomRadius));
                        }
                    }
                    maximum = Math.Max(maximum, bound);
                }
            }
            z += section.Height;
        }
        return maximum + 1e-8;
    }

    private static double SegmentBoxDistance(double ax, double ay, double bx, double by,
        double x0, double y0, double x1, double y1)
    {
        double dx = bx - ax, dy = by - ay, length2 = dx * dx + dy * dy;
        double lo = 0, hi = 1;
        bool Clip(double p, double d, double min, double max)
        {
            if (Math.Abs(d) < 1e-20) return p >= min && p <= max;
            double a = (min - p) / d, b = (max - p) / d;
            lo = Math.Max(lo, Math.Min(a, b)); hi = Math.Min(hi, Math.Max(a, b));
            return lo <= hi;
        }
        if (Clip(ax, dx, x0, x1) && Clip(ay, dy, y0, y1)) return 0;
        double Endpoint(double x, double y)
        {
            double ex = x - Math.Clamp(x, x0, x1), ey = y - Math.Clamp(y, y0, y1);
            return ex * ex + ey * ey;
        }
        double Corner(double x, double y)
        {
            double t = length2 > 1e-24 ? Math.Clamp(((x - ax) * dx + (y - ay) * dy) / length2, 0, 1) : 0;
            double ex = x - ax - t * dx, ey = y - ay - t * dy;
            return ex * ex + ey * ey;
        }
        double square = Math.Min(Math.Min(Endpoint(ax, ay), Endpoint(bx, by)),
            Math.Min(Math.Min(Corner(x0, y0), Corner(x1, y0)), Math.Min(Corner(x0, y1), Corner(x1, y1))));
        return Math.Max(0, Math.Sqrt(square) - 1e-10);
    }

    /// <summary>Planar support of one physical section, clipped at its reachable height.</summary>
    private static double SectionSupport(CarvingCutterSection section, double z, double height,
        double gradient, out double contactRadius)
    {
        if (section.Kind == CarvingCutterSectionKind.Hemisphere)
        {
            double contact = Math.Min(height, section.TopRadius * (1 - 1 / Math.Sqrt(1 + gradient * gradient)));
            contactRadius = Math.Sqrt(Math.Max(0, contact * (2 * section.TopRadius - contact)));
            return gradient * contactRadius - z - contact;
        }
        double top = section.BottomRadius + (section.TopRadius - section.BottomRadius) * height / section.Height;
        double lowerValue = gradient * section.BottomRadius - z;
        double upperValue = gradient * top - z - height;
        contactRadius = upperValue >= lowerValue ? top : section.BottomRadius;
        return Math.Max(lowerValue, upperValue);
    }

    internal double Support(double gradient, out double contactRadius)
    {
        double z = 0, result = 0;
        contactRadius = 0;
        foreach (var section in _sections)
        {
            if (z > MaximumContactHeight) break;
            double height = Math.Min(section.Height, MaximumContactHeight - z);
            double value = SectionSupport(section, z, height, gradient, out double radius);
            if (value >= result) { result = value; contactRadius = radius; }
            z += section.Height;
        }
        return result;
    }

    /// <summary>Exact planar support clipped to a surface patch. Interior extrema,
    /// patch edges, and every profile radius junction cover the piecewise linear/ball envelope.</summary>
    internal double ClippedPlaneSupport(double gx, double gy, double x0, double y0, double x1, double y1,
        double qx0, double qy0, double qx1, double qy1, out double witnessX, out double witnessY)
    {
        double maximum = double.NegativeInfinity, wx = 0, wy = 0;
        double gradient = Math.Sqrt(gx * gx + gy * gy);
        void Add(double x, double y)
        {
            if (x < x0 - 1e-12 || x > x1 + 1e-12 || y < y0 - 1e-12 || y > y1 + 1e-12) return;
            x = Math.Clamp(x, x0, x1); y = Math.Clamp(y, y0, y1);
            double dx = Math.Max(0, Math.Max(qx0 - x, x - qx1));
            double dy = Math.Max(0, Math.Max(qy0 - y, y - qy1));
            double radius = Math.Sqrt(dx * dx + dy * dy);
            if (radius > MaximumRadius + 1e-12) return;
            double value = gx * x + gy * y - Bottom(Math.Max(0, Math.Min(radius, MaximumRadius) - 1e-12));
            if (value > maximum) { maximum = value; wx = x; wy = y; }
        }
        void Radius(double radius)
        {
            if (gradient > 0) Add((gx >= 0 ? qx1 : qx0) + radius * gx / gradient,
                (gy >= 0 ? qy1 : qy0) + radius * gy / gradient);
            void Vertical(double x)
            {
                double dx = Math.Max(0, Math.Max(qx0 - x, x - qx1));
                if (dx > radius) return;
                double reach = Math.Sqrt(Math.Max(0, radius * radius - dx * dx));
                Add(x, qy0 - reach); Add(x, qy1 + reach);
            }
            void Horizontal(double y)
            {
                double dy = Math.Max(0, Math.Max(qy0 - y, y - qy1));
                if (dy > radius) return;
                double reach = Math.Sqrt(Math.Max(0, radius * radius - dy * dy));
                Add(qx0 - reach, y); Add(qx1 + reach, y);
            }
            Vertical(x0); Vertical(x1); Horizontal(y0); Horizontal(y1);
        }
        void EdgeExtrema(bool vertical, double fixedCoordinate, CarvingCutterSection section)
        {
            double delta = vertical ? Math.Max(0, Math.Max(qx0 - fixedCoordinate, fixedCoordinate - qx1))
                : Math.Max(0, Math.Max(qy0 - fixedCoordinate, fixedCoordinate - qy1));
            double g = vertical ? gy : gx, distance;
            if (section.Kind == CarvingCutterSectionKind.Hemisphere)
            {
                if (delta > section.TopRadius) return;
                distance = g * Math.Sqrt(Math.Max(0, section.TopRadius * section.TopRadius - delta * delta) / (1 + g * g));
            }
            else
            {
                if (section.TopRadius <= section.BottomRadius) return;
                double slope = section.Height / (section.TopRadius - section.BottomRadius);
                if (Math.Abs(g) >= slope) return; // Optimum is at an already included radius junction.
                distance = g * delta / Math.Sqrt(slope * slope - g * g);
            }
            if (vertical) Add(fixedCoordinate, (g >= 0 ? qy1 : qy0) + distance);
            else Add((g >= 0 ? qx1 : qx0) + distance, fixedCoordinate);
        }
        Add(x0, y0); Add(x0, y1); Add(x1, y0); Add(x1, y1);
        Add(qx0, qy0); Add(qx0, qy1); Add(qx1, qy0); Add(qx1, qy1);
        Add(x0, Math.Clamp(qy0, y0, y1)); Add(x0, Math.Clamp(qy1, y0, y1));
        Add(x1, Math.Clamp(qy0, y0, y1)); Add(x1, Math.Clamp(qy1, y0, y1));
        Add(Math.Clamp(qx0, x0, x1), y0); Add(Math.Clamp(qx1, x0, x1), y0);
        Add(Math.Clamp(qx0, x0, x1), y1); Add(Math.Clamp(qx1, x0, x1), y1);
        double z = 0;
        foreach (var section in _sections)
        {
            if (z > MaximumContactHeight) break;
            double h = Math.Min(section.Height, MaximumContactHeight - z);
            double radius = section.Kind == CarvingCutterSectionKind.Hemisphere
                ? Math.Sqrt(Math.Max(0, h * (2 * section.TopRadius - h)))
                : section.BottomRadius + (section.TopRadius - section.BottomRadius) * h / section.Height;
            Radius(section.BottomRadius); Radius(radius);
            if (section.Kind == CarvingCutterSectionKind.Hemisphere)
                Radius(Math.Min(radius, section.TopRadius * gradient / Math.Sqrt(1 + gradient * gradient)));
            EdgeExtrema(true, x0, section); EdgeExtrema(true, x1, section);
            EdgeExtrema(false, y0, section); EdgeExtrema(false, y1, section);
            z += section.Height;
        }
        witnessX = wx; witnessY = wy;
        return maximum;
    }
}
