using RW.Toolpaths.Geometry;
using Clipper2Lib;
using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Contained columns owned by REAL earlier cutter poses. The same
/// whole-chord proof serves omission and continuation; tip disks are insufficient.</summary>
internal sealed class ClearedColumnStock
{
    internal readonly record struct Column(Point3D A, Point3D B, double Radius);
    private readonly List<(double Radius, PreparedPointIndex Index)> groups = new();
    private sealed class FloorUnion
    {
        internal double Radius, Floor;
        internal Column[] Source = Array.Empty<Column>();
        internal PolygonOps.BoundarySegmentIndex? Region;
        internal bool Built;
    }
    private readonly List<FloorUnion> floors = new();
    private sealed class UnionPreparation
    {
        internal int Builds, Chords;
        internal double Milliseconds;
    }
    private readonly UnionPreparation preparation = new();
    private readonly CancellationToken token;
    private const double Reserve = 1e-8;
    private const int MaximumPoses = 500_000;
    private readonly int maximumQueries;
    private readonly Dictionary<(Point3D,Point3D,double,double),bool> balls=new();
    private readonly Queue<(Point3D,Point3D,double,double)> ballOrder=new();
    internal long CachedQueries {get;private set;}
    internal int Poses { get; private set; }
    internal int Queries { get; private set; }
    internal bool PoseBudgetReached { get; private set; }
    internal bool QueryBudgetReached { get; private set; }
    internal long FloorUnionQueries { get; private set; }
    internal long FloorUnionAccepted { get; private set; }
    internal int FloorUnionBuilds => preparation.Builds;
    internal int FloorUnionChords => preparation.Chords;
    internal double FloorUnionMilliseconds => preparation.Milliseconds;

    // Pool immutable geometry and bounded proof memoization, not the callers'
    // original independent work budgets. Each nomination/selection stage keeps
    // its existing bound; an earlier region cannot consume a later region's proof.
    internal ClearedColumnStock CreateQuerySession(int? queryLimit = null) => new(this, queryLimit ?? maximumQueries);

    private ClearedColumnStock(ClearedColumnStock snapshot, int queryLimit)
    {
        groups = snapshot.groups; floors = snapshot.floors;
        balls = snapshot.balls; ballOrder = snapshot.ballOrder;
        preparation = snapshot.preparation; token = snapshot.token;
        maximumQueries = queryLimit; Poses = snapshot.Poses;
        PoseBudgetReached = snapshot.PoseBudgetReached;
    }

    internal ClearedColumnStock(IEnumerable<Column> source, CancellationToken token,int maximumQueries=2_000_000)
    {
        this.token = token;
        this.maximumQueries=maximumQueries;
        foreach (var group in source.GroupBy(c => c.Radius))
        {
            token.ThrowIfCancellationRequested();
            if (!(group.Key > 0) || !double.IsFinite(group.Key) || group.Key > 100) continue;
            var columns = group.Where(c => Finite(c.A) && Finite(c.B)).ToArray();
            var posted = columns.Where(c => Posted(c.A) && Posted(c.B)).ToArray();
            if (posted.Length > 0)
            {
                double floor = posted.Min(c => Math.Max(c.A.Z, c.B.Z));
                if (floor < 0) floors.Add(new FloorUnion { Radius = group.Key, Floor = floor,
                    Source = posted.Where(c => Math.Max(c.A.Z, c.B.Z) <= floor).ToArray() });
            }
            var sites = new Dictionary<(long, long), Point3D>();
            double pitch = Math.Min(.4, group.Key / 2);
            foreach (var chord in columns)
            {
                token.ThrowIfCancellationRequested();
                if (!Finite(chord.A) || !Finite(chord.B)) continue;
                double length = Math.Sqrt(Math.Pow(chord.B.X-chord.A.X, 2) + Math.Pow(chord.B.Y-chord.A.Y, 2));
                double pieces = Math.Max(1, Math.Ceiling(length / pitch));
                if (!double.IsFinite(pieces) || pieces > MaximumPoses) continue;
                for (int i = 0; i <= (int)pieces; i++)
                {
                    if ((i & 1023) == 0) token.ThrowIfCancellationRequested();
                    double t = i / pieces;
                    var p = new Point3D(chord.A.X + (chord.B.X-chord.A.X)*t,
                        chord.A.Y + (chord.B.Y-chord.A.Y)*t, chord.A.Z + (chord.B.Z-chord.A.Z)*t);
                    if (p.Z >= 0) continue;
                    var key = ((long)Math.Round(p.X*10), (long)Math.Round(p.Y*10));
                    if (!sites.TryGetValue(key, out var old))
                    {
                        if (Poses >= MaximumPoses) { PoseBudgetReached = true; break; }
                        sites.Add(key, p); Poses++;
                    }
                    else if (p.Z < old.Z) sites[key] = p;
                }
                if (PoseBudgetReached) break;
            }
            groups.Add((group.Key, new PreparedPointIndex(sites.Values.ToArray(), token)));
            if (PoseBudgetReached) break;
        }
    }

    internal bool Contains(Point3D a, Point3D b, Func<double, double> maximumRadiusAtDepth,
        double tolerance = 0) => Covered(a, b, 0, maximumRadiusAtDepth, tolerance);

    private bool Covered(Point3D a, Point3D b, int level, Func<double, double> radius, double tolerance)
    {
        token.ThrowIfCancellationRequested();
        if (!Finite(a) || !Finite(b)) return false;
        double depth = Math.Max(-a.Z, -b.Z) - tolerance;
        if (depth <= 0) return true;
        if (groups.Count == 0) return false;
        double requested = Math.Max(depth, .0005), reachRadius = radius(requested);
        if (!double.IsFinite(reachRadius) || reachRadius < 0) return false;
        double run = Math.Sqrt(Math.Pow(b.X-a.X, 2) + Math.Pow(b.Y-a.Y, 2));
        double x = (a.X+b.X)/2, y = (a.Y+b.Y)/2;
        // Cache the actual capsule and required floor, independent of a caller's
        // profile/delegate identity. The union can prove a long narrow sweep
        // without replacing it by a much larger enclosing ball or subdividing it.
        // A cached positive remains evidence after the new-query budget expires.
        var key=(a,b,requested,reachRadius);
        if(balls.TryGetValue(key,out bool contained))CachedQueries++;
        else
        {
            if(QueryBudgetReached)return false;
            if (++Queries > maximumQueries) { QueryBudgetReached = true; return false; }
            foreach (var floor in floors)
            {
                // These are closed, actual commanded solids. An equal posted
                // floor is already removed; do not demand fictitious extra depth.
                if (floor.Floor > -requested) continue;
                if (!floor.Built) BuildFloor(floor);
                if (floor.Region is null) continue;
                FloorUnionQueries++;
                if (floor.Region.ContainsCapsule(Quantize(a.X,a.Y), Quantize(b.X,b.Y), reachRadius * PathUtils.Scale))
                { contained = true; FloorUnionAccepted++; break; }
            }
            if (!contained)
            foreach (var group in groups)
            {
                double reach = group.Radius-reachRadius-run/2-Reserve;
                if(reach>=0 && group.Index.AnyWithin(x,y,reach,-requested,token))
                {contained=true;break;}
            }
            if(balls.Count>=262144){var old=ballOrder.Dequeue();balls.Remove(old);}
            balls.Add(key,contained);ballOrder.Enqueue(key);
        }
        if(contained)return true;
        if (level >= 8 || run <= .1) return false;
        // Every successful leaf would also contain its endpoint's complete
        // query cylinder. Refuse an impossible endpoint before subdividing.
        if(level==0 && (!Covered(a,a,8,radius,tolerance)||!Covered(b,b,8,radius,tolerance)))return false;
        var mid = new Point3D(x, y, (a.Z+b.Z)/2);
        return Covered(a, mid, level+1, radius, tolerance) && Covered(mid, b, level+1, radius, tolerance);
    }

    private void BuildFloor(FloorUnion floor)
    {
        long started = Stopwatch.GetTimestamp();
        floor.Built = true;
        var segments = floor.Source.Select(c => new PocketPostedChordChains.Segment(
            new((long)Math.Round(c.A.X * 1000), (long)Math.Round(c.A.Y * 1000)),
            new((long)Math.Round(c.B.X * 1000), (long)Math.Round(c.B.Y * 1000)))).ToArray();
        var chains = PocketPostedChordChains.Build(segments, token);
        var paths = new Paths64(chains.Paths.Select(path => new Path64(path.Select(p =>
            Quantize(p.X / 1000d, p.Y / 1000d)))));
        // Stationary real poses also own physical stock columns.
        paths.AddRange(floor.Source.Where(c => c.A.X == c.B.X && c.A.Y == c.B.Y)
            .Select(c => new Path64 { Quantize(c.A.X, c.A.Y) }));
        if (paths.Count > 0) floor.Region = PostedSweepInnerRegion.Build(paths, floor.Radius, token);
        preparation.Builds++; preparation.Chords += chains.UniqueEdges;
        preparation.Milliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds;
        floor.Source = Array.Empty<Column>();
    }

    private static Point64 Quantize(double x, double y) => new(
        (long)Math.Round(x * PathUtils.Scale), (long)Math.Round(y * PathUtils.Scale));
    private static bool Posted(Point3D p) => p.X == Math.Round(p.X * 1000) / 1000d
        && p.Y == Math.Round(p.Y * 1000) / 1000d && p.Z == Math.Round(p.Z * 1000) / 1000d;

    private static bool Finite(Point3D p) => double.IsFinite(p.X+p.Y+p.Z)
        && Math.Abs(p.X) <= 10000 && Math.Abs(p.Y) <= 10000 && Math.Abs(p.Z) <= 100;
}
