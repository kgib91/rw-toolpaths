namespace RW.Toolpaths.Milling;

/// <summary>Exact variable-depth physical cutter sweep against a vertical box
/// of protected material. Both squared box distance and section radius squared
/// are quadratic between XY projection and axial section transitions.</summary>
internal static class VCarveCutterBoxIntersection
{
    internal static bool Intersects(Point3D a, Point3D b, VCarveCutterProfile profile,
        double floorDepth, double left, double bottom, double right, double top)
    {
        if(a==b && double.IsFinite(a.X)&&double.IsFinite(a.Y)&&double.IsFinite(a.Z)
            &&double.IsFinite(floorDepth)&&double.IsFinite(left)&&double.IsFinite(bottom)
            &&double.IsFinite(right)&&double.IsFinite(top))
            return IntersectsPoint(a,profile,floorDepth,left,bottom,right,top);
        return IntersectsGeneral(a,b,profile,floorDepth,left,bottom,right,top);
    }

    private static bool IntersectsPoint(Point3D point,VCarveCutterProfile profile,
        double floorDepth,double left,double bottom,double right,double top)
    {
        double depth=-point.Z-floorDepth;
        if(!TryPointRadiusSquared(profile,depth,out double radiusSquared))
            return IntersectsGeneral(point,point,profile,floorDepth,left,bottom,right,top);
        return PointBoxDistanceSquared(point,left,bottom,right,top)-radiusSquared<-1e-12;
    }

    internal static double PointBoxDistanceSquared(Point3D point,double left,double bottom,double right,double top)
    {
        double x=point.X<left?point.X-left:point.X>right?point.X-right:0;
        double y=point.Y<bottom?point.Y-bottom:point.Y>top?point.Y-top:0;
        return x*x+y*y;
    }

    internal static bool TryPointRadiusSquared(VCarveCutterProfile profile,double depth,out double maximumRadiusSquared)
    {
        maximumRadiusSquared=double.NegativeInfinity;
        if(depth<=0)return true;
        double sectionStart=0;
        foreach(var section in profile.PhysicalProfile.Sections)
        {
            double sectionEnd=Math.Min(profile.MaximumDepth,sectionStart+section.Height);
            if(depth>=sectionStart&&depth<=sectionEnd)
            {
                // Exact constant-motion reduction of the general kernel:
                // same axial accumulator, radius coefficient, box-distance
                // arithmetic and strict tangency threshold; no radius square
                // root, parameter partition, sorting or heap enumerator.
                double h=depth-sectionStart,radiusSquared;
                if(section.Kind==CarvingCutterSectionKind.Hemisphere)
                    radiusSquared=h*(2*section.TopRadius-h);
                else
                {
                    double slope=(section.TopRadius-section.BottomRadius)/section.Height;
                    double r=section.BottomRadius+slope*h;
                    radiusSquared=r*r;
                }
                if(!double.IsFinite(radiusSquared))return false;
                maximumRadiusSquared=Math.Max(maximumRadiusSquared,radiusSquared);
                if(depth<sectionEnd)return true;
            }
            sectionStart=sectionEnd;
            if(sectionStart>=profile.MaximumDepth)break;
        }
        return true;
    }

    internal static bool IntersectsGeneral(Point3D a, Point3D b, VCarveCutterProfile profile,
        double floorDepth, double left, double bottom, double right, double top)
    {
        double da = -a.Z - floorDepth, dd = a.Z - b.Z;
        if (Math.Max(da, da + dd) <= 0) return false;
        double dx = b.X - a.X, dy = b.Y - a.Y, sectionStart = 0;
        Span<double> cuts = stackalloc double[6];
        foreach (var section in profile.PhysicalProfile.Sections)
        {
            double sectionEnd = Math.Min(profile.MaximumDepth, sectionStart + section.Height);
            double lo = 0, hi = 1;
            if (dd == 0)
            {
                if (da < sectionStart || da > sectionEnd)
                { sectionStart = sectionEnd; if (sectionStart >= profile.MaximumDepth) break; continue; }
            }
            else
            {
                double t0 = (sectionStart - da) / dd, t1 = (sectionEnd - da) / dd;
                lo = Math.Max(lo, Math.Min(t0, t1)); hi = Math.Min(hi, Math.Max(t0, t1));
            }
            if (lo <= hi)
            {
                int count = 2; cuts[0] = lo; cuts[1] = hi;
                if (dx != 0)
                {
                    double t0 = (left - a.X) / dx, t1 = (right - a.X) / dx;
                    if (t0 > lo && t0 < hi) cuts[count++] = t0;
                    if (t1 > lo && t1 < hi) cuts[count++] = t1;
                }
                if (dy != 0)
                {
                    double t0 = (bottom - a.Y) / dy, t1 = (top - a.Y) / dy;
                    if (t0 > lo && t0 < hi) cuts[count++] = t0;
                    if (t1 > lo && t1 < hi) cuts[count++] = t1;
                }
                cuts[..count].Sort();
                double h = da - sectionStart;
                double slope = (section.TopRadius - section.BottomRadius) / section.Height;
                double radiusA, radiusB, radiusC;
                if (section.Kind == CarvingCutterSectionKind.Hemisphere)
                {
                    radiusA = -dd * dd;
                    radiusB = 2 * (section.TopRadius - h) * dd;
                    radiusC = h * (2 * section.TopRadius - h);
                }
                else
                {
                    double r = section.BottomRadius + slope * h, rd = slope * dd;
                    radiusA = rd * rd; radiusB = 2 * r * rd; radiusC = r * r;
                }
                for (int i = 1; i < count; i++)
                {
                    double low = cuts[i - 1], high = cuts[i], mid = (low + high) / 2;
                    double x = a.X + dx * mid, y = a.Y + dy * mid;
                    double qx = x < left ? a.X - left : x > right ? a.X - right : 0;
                    double vx = x < left || x > right ? dx : 0;
                    double qy = y < bottom ? a.Y - bottom : y > top ? a.Y - top : 0;
                    double vy = y < bottom || y > top ? dy : 0;
                    double qa = vx * vx + vy * vy - radiusA;
                    double qb = 2 * (qx * vx + qy * vy) - radiusB;
                    double qc = qx * qx + qy * qy - radiusC;
                    double Value(double t) => (qa * t + qb) * t + qc;
                    if (Value(low) < -1e-12 || Value(high) < -1e-12) return true;
                    if (qa > 0)
                    {
                        double at = -qb / (2 * qa);
                        if (at > low && at < high && Value(at) < -1e-12) return true;
                    }
                }
            }
            sectionStart = sectionEnd;
            if (sectionStart >= profile.MaximumDepth) break;
        }
        return false;
    }
}
