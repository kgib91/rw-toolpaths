// Test-only literal pre-index separation oracle. Never used by production.
using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Milling;

// Specific to a design extruded below one fixed start plane. With a monotone
// cutter, the top-plane footprint bounds the entire body below that plane.
// On a linear move, distance squared to each polygon segment and hemisphere
// radius squared are piecewise quadratics. Taper radius squared is quadratic
// too. Checking their stationary points therefore checks the complete chord.
// This is not a stock model or a general 3D certificate engine.
internal sealed class VCarveNativeContainmentReference
{
    private readonly (PointD A, PointD B)[] edges;
    private readonly PreparedRegionDistance region;
    private readonly VCarveCutterProfile profile;
    internal CancellationToken Token {get;}
    private readonly Func<Point3D,Point3D,bool>? stockChord;
    private readonly Func<ToolMove,bool>? stockCurve;
    internal bool StockAware=>stockCurve is not null;
    internal bool CertifyStockCurve(ToolMove move)=>stockCurve?.Invoke(move)==true;
    internal VCarveNativeContainmentReference(IReadOnlyList<IReadOnlyList<PointD>> boundary, VCarveCutterProfile profile,CancellationToken token=default,Func<Point3D,Point3D,bool>? stockChord=null,Func<ToolMove,bool>? stockCurve=null)
    {
        if((stockChord is null)!=(stockCurve is null))throw new ArgumentException("Native stock context needs both chord and continuous curve authorities.");
        this.stockChord=stockChord;this.stockCurve=stockCurve;Token=token;
        edges = boundary.SelectMany(r => Enumerable.Range(0, r.Count).Select(i => (r[i], r[(i + 1) % r.Count]))).ToArray();
        region = new(boundary); this.profile = profile;
    }
    internal bool Legal(Point3D a, Point3D b) => MinimumSquaredSeparation(a, b) >= -1e-10;
    internal double MinimumSquaredSeparation(Point3D a, Point3D b)
    {
        // Stock-aware boolean proofs grant no fictitious clearance margin.
        // Continuous curves use their dedicated stock enclosure proof below;
        // zero here can only rank/reject poses, never inflate a flat certificate.
        if(stockChord is not null)return stockChord(a,b)?0:double.NegativeInfinity;
        if (Math.Min(a.Z, b.Z) >= 0) return double.PositiveInfinity;
        if ((a.Z < 0 && region.SignedDistance(new(a.X, a.Y)) < 0) || (b.Z < 0 && region.SignedDistance(new(b.X, b.Y)) < 0)) return double.NegativeInfinity;
        double da = -a.Z, db = a.Z - b.Z, vx = b.X - a.X, vy = b.Y - a.Y;
        double radius = profile.PhysicalRadiusAtDepth(Math.Max(da, da + db));
        double left = Math.Min(a.X, b.X) - radius, right = Math.Max(a.X, b.X) + radius,
            bottom = Math.Min(a.Y, b.Y) - radius, top = Math.Max(a.Y, b.Y) + radius;
        var depthCuts = new List<double> { 0, 1 }; double sectionHeight = 0;
        if (db != 0)
        {
            depthCuts.Add(-da / db);
            foreach (var s in profile.Sections) { sectionHeight += s.Height; depthCuts.Add((sectionHeight - da) / db); }
        }
        double minimum = double.PositiveInfinity;
        foreach (var edge in edges)
        {
            if (Math.Min(edge.A.x, edge.B.x) > right || Math.Max(edge.A.x, edge.B.x) < left
                || Math.Min(edge.A.y, edge.B.y) > top || Math.Max(edge.A.y, edge.B.y) < bottom)
            {
                // Pruning proves this edge cannot intersect the CHORD cutter.
                // It does not prove infinite clearance for an inflated CURVE.
                // Keep a finite conservative separation lower bound for every
                // omitted edge, using its enclosing box and maximum chord radius.
                double gx=Math.Max(0,Math.Max(Math.Min(edge.A.x,edge.B.x)-Math.Max(a.X,b.X),Math.Min(a.X,b.X)-Math.Max(edge.A.x,edge.B.x)));
                double gy=Math.Max(0,Math.Max(Math.Min(edge.A.y,edge.B.y)-Math.Max(a.Y,b.Y),Math.Min(a.Y,b.Y)-Math.Max(edge.A.y,edge.B.y)));
                minimum=Math.Min(minimum,gx*gx+gy*gy-radius*radius);
                continue;
            }
            double ex = edge.B.x - edge.A.x, ey = edge.B.y - edge.A.y, l2 = ex * ex + ey * ey;
            if (l2 == 0) continue;
            double ux = a.X - edge.A.x, uy = a.Y - edge.A.y;
            double projection = (ux * ex + uy * ey) / l2, projectionDelta = (vx * ex + vy * ey) / l2;
            var cuts = new List<double>(depthCuts);
            if (projectionDelta != 0) { cuts.Add(-projection / projectionDelta); cuts.Add((1 - projection) / projectionDelta); }
            var ordered = cuts.Where(t => t >= 0 && t <= 1).Order().Distinct().ToArray();
            for (int i = 1; i < ordered.Length; i++)
            {
                double lo = ordered[i - 1], hi = ordered[i], mid = (lo + hi) / 2;
                if (da + db * mid <= 0) continue;
                double u = projection + projectionDelta * mid, dx = ux, dy = uy, wx = vx, wy = vy;
                if (u >= 1) { dx -= ex; dy -= ey; }
                else if (u > 0) { dx -= ex * projection; dy -= ey * projection; wx -= ex * projectionDelta; wy -= ey * projectionDelta; }
                double height = 0; CarvingCutterSection section = profile.Sections[^1];
                foreach (var s in profile.Sections) { section = s; if (da + db * mid <= height + s.Height) break; height += s.Height; }
                double qa, qb, qc;
                if (section.Kind == CarvingCutterSectionKind.Hemisphere)
                {
                    double local = da - height; qa = -db * db; qb = 2 * db * (section.TopRadius - local); qc = local * (2 * section.TopRadius - local);
                }
                else
                {
                    double slope = (section.TopRadius - section.BottomRadius) / section.Height;
                    double r = section.BottomRadius + (da - height) * slope, delta = db * slope;
                    qa = delta * delta; qb = 2 * r * delta; qc = r * r;
                }
                double pa = wx * wx + wy * wy - qa, pb = 2 * (dx * wx + dy * wy) - qb, pc = dx * dx + dy * dy - qc;
                double Value(double t) => (pa * t + pb) * t + pc;
                minimum = Math.Min(minimum, Math.Min(Value(lo), Value(hi)));
                if (pa > 0) { double t = -pb / (2 * pa); if (t > lo && t < hi) minimum = Math.Min(minimum, Value(t)); }
            }
        }
        return minimum;
    }
}
