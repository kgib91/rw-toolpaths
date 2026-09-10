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
        double farX = Math.Max(Math.Abs(x0 - qx1), Math.Abs(x1 - qx0));
        double farY = Math.Max(Math.Abs(y0 - qy1), Math.Abs(y1 - qy0));
        double far = Math.Sqrt(farX * farX + farY * farY);
        double cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
        double vx = cx - Math.Clamp(cx, qx0, qx1), vy = cy - Math.Clamp(cy, qy0, qy1);
        double radial = Math.Sqrt(vx * vx + vy * vy);
        double maximum = double.NegativeInfinity, z = 0;
        void Plane(double gx, double gy, double intercept)
        {
            double patch = Math.Max(Math.Max(h00 - gx * x0 - gy * y0, h10 - gx * x1 - gy * y0),
                Math.Max(h01 - gx * x0 - gy * y1, h11 - gx * x1 - gy * y1));
            double centers = Math.Max(gx * ax + gy * ay - az, gx * bx + gy * by - bz);
            if (queryRectangle) centers = Math.Max(gx * ax, gx * bx) + Math.Max(gy * ay, gy * by) - Math.Min(az, bz);
            maximum = Math.Max(maximum, patch + centers - intercept + margin * (Math.Abs(gx) + Math.Abs(gy)));
        }
        foreach (var section in _sections)
        {
            if (z > MaximumContactHeight) break;
            double height = Math.Min(section.Height, MaximumContactHeight - z);
            if (section.Kind == CarvingCutterSectionKind.Hemisphere)
            {
                double r = section.TopRadius;
                double outer = Math.Sqrt(Math.Max(0, height * (2 * r - height)));
                if (near <= outer)
                {
                    double tangentRadius = Math.Min(radial, r * (1 - 1e-8));
                    double root = Math.Sqrt(r * r - tangentRadius * tangentRadius);
                    double scale = radial > 0 ? tangentRadius / (root * radial) : 0;
                    Plane(scale * vx, scale * vy, z + r - r * r / root);
                }
            }
            else
            {
                if (near <= section.BottomRadius) Plane(0, 0, z);
                double outer = section.BottomRadius + (section.TopRadius - section.BottomRadius) * height / section.Height;
                if (outer > section.BottomRadius && near <= outer && far >= section.BottomRadius)
                {
                    double slope = section.Height / (section.TopRadius - section.BottomRadius);
                    double scale = radial > 0 ? slope / radial : 0;
                    Plane(scale * vx, scale * vy, z - slope * section.BottomRadius);
                }
            }
            z += section.Height;
        }
        return maximum + 1e-8;
    }

    internal double Support(double gradient, out double contactRadius)
    {
        double z = 0, result = 0;
        contactRadius = 0;
        foreach (var section in _sections)
        {
            if (z > MaximumContactHeight) break;
            double h = Math.Min(section.Height, MaximumContactHeight - z);
            if (section.Kind == CarvingCutterSectionKind.Hemisphere)
            {
                double contact = Math.Min(h, section.TopRadius * (1 - 1 / Math.Sqrt(1 + gradient * gradient)));
                double radius = Math.Sqrt(Math.Max(0, contact * (2 * section.TopRadius - contact)));
                double value = gradient * radius - z - contact;
                if (value >= result) { result = value; contactRadius = radius; }
            }
            else
            {
                double top = section.BottomRadius + (section.TopRadius - section.BottomRadius) * h / section.Height;
                double lowerValue = gradient * section.BottomRadius - z, upperValue = gradient * top - z - h;
                if (lowerValue >= result) { result = lowerValue; contactRadius = section.BottomRadius; }
                if (upperValue >= result) { result = upperValue; contactRadius = top; }
            }
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
