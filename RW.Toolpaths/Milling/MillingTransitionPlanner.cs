namespace RW.Toolpaths.Milling;

/// <summary>One transition in native milling axes (horizontal XY, vertical Z).</summary>
public readonly record struct MillingTransition(Point3D Start, Point3D End, bool ClearedTravel = false);

/// <summary>Common safe-plane transfer and controlled entry construction. Float
/// custody matches emitted machine trajectories; a stock proof authorizes each
/// entire fast descent prefix, never just its endpoint.</summary>
public static class MillingTransitionPlanner
{
    public const float ApproachClearance = .1f;
    // Whole-cutter positive reserve. Separate from the above-stock approach
    // height: equality of their current values does not make them one permission.
    public const double PositiveTravelClearance = .1;
    public const float ContinuityTolerance = 1e-4f;

    /// <summary>Emission continuity only. This must never merge stock evidence
    /// or authorize a new cutting chord between nearby native source points.</summary>
    public static bool IsContinuous(Point3D from, Point3D to)
        => MathF.Abs((float)from.X - (float)to.X) <= ContinuityTolerance
            && MathF.Abs((float)from.Y - (float)to.Y) <= ContinuityTolerance
            && MathF.Abs((float)from.Z - (float)to.Z) <= ContinuityTolerance;

    public static bool RequiresPlanarTraverse(Point3D from,Point3D to)
        =>MathF.Abs((float)from.X-(float)to.X)>ContinuityTolerance
            ||MathF.Abs((float)from.Y-(float)to.Y)>ContinuityTolerance;

    public static IReadOnlyList<MillingTransition> Build(Point3D from, Point3D to,
        float safeZ, float? surface = null, Func<Point3D, Point3D, bool>? clearedTravelGuard = null,
        MillingLocalTravel? localTravel = null,Point3D? postedFrom = null)
    {
        if(localTravel is not null&&surface is {} materialSurface&&localTravel.MatchesSurface(materialSurface)&&postedFrom is {} cursor){
            if(localTravel.Connect(from,to,cursor,safeZ) is {} local)return local;
            if(localTravel.ConnectAboveStock(from,to,cursor,safeZ,clearedTravelGuard) is {} air)return air;
        }
        float ax = (float)from.X, ay = (float)from.Y, az = (float)from.Z;
        float bx = (float)to.X, by = (float)to.Y, bz = (float)to.Z;
        var result = new List<MillingTransition>(5);
        if (az < safeZ) result.Add(new(new(ax, ay, az), new(ax, ay, safeZ)));
        if (RequiresPlanarTraverse(from,to))
            result.Add(new(new(ax, ay, safeZ), new(bx, by, safeZ)));
        float approach = safeZ;
        if (surface is { } top && float.IsFinite(top) && bz < top && top + ApproachClearance < safeZ)
        {
            approach = top + ApproachClearance;
            result.Add(new(new(bx, by, safeZ), new(bx, by, approach)));
        }
        if (surface is { } stockTop && float.IsFinite(stockTop) && approach >= stockTop
            && bz < stockTop && clearedTravelGuard is not null)
        {
            var start = new Point3D(bx, by, approach);
            bool Clear(float z) => clearedTravelGuard(start, new(bx, by, z));
            float unproved = bz, proved = approach;
            if (Clear(bz)) proved = bz;
            else for (int attempt = 0; attempt < 14; attempt++)
            {
                float candidate = (unproved + proved) / 2;
                if (Clear(candidate)) proved = candidate; else unproved = candidate;
            }
            if (approach - proved >= .05f)
            {
                result.Add(new(start, new(bx, by, proved), ClearedTravel: true));
                approach = proved;
            }
        }
        if (bz < approach) result.Add(new(new(bx, by, approach), new(bx, by, bz)));
        return result;
    }
}
