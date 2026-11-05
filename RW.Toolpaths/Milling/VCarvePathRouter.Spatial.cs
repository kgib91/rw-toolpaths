namespace RW.Toolpaths.Milling;

internal static partial class VCarvePathRouter
{
    // An explicit false is the same-source legacy scheduling control. This is
    // transient planning policy, never a change to configured Z-pass direction.
    internal static readonly AsyncLocal<bool?> AutomaticSpatialSweepOverride=new();

    internal static VCarveRingOrder.SpatialSweep? AutomaticSpatialSweep(
        IReadOnlyList<TaggedToolpath> paths,DepthSchedule depth,VCarveOptions options,CancellationToken token)
    {
        if(AutomaticSpatialSweepOverride.Value==false||!options.SmoothVCarving||!options.OptimizeContourConnections
            ||options.Motion is null||depth.DepthPerPass>0&&depth.DepthPerPass<depth.Depth-1e-9
            ||paths.Any(p=>p.DepthPassIndex is not (null or 0))
            ||paths.Count(p=>p.ConcentricSource is not null)<128||!paths.Any(p=>p.ConcentricSource is null))return null;
        // Small regions keep their cheap local scheduler. Broad regions with
        // many jobs benefit from a bounded board sweep; long source circuits
        // remain atomic and real offset/depth prerequisites may leave a window.
        const double window=64;
        double minX=double.PositiveInfinity,minY=minX,maxX=double.NegativeInfinity,maxY=maxX;
        long examined=0;
        foreach(var path in paths)foreach(var point in path.Points){
            if((++examined&1023)==0)token.ThrowIfCancellationRequested();
            if(!double.IsFinite(point.X)||!double.IsFinite(point.Y))return null;
            minX=Math.Min(minX,point.X);maxX=Math.Max(maxX,point.X);
            minY=Math.Min(minY,point.Y);maxY=Math.Max(maxY,point.Y);
        }
        token.ThrowIfCancellationRequested();
        double width=maxX-minX,height=maxY-minY;
        if(!double.IsFinite(width)||!double.IsFinite(height)||Math.Max(width,height)<2*window)return null;
        return new(height>width?VCarveRingOrder.SpatialAxis.Y:VCarveRingOrder.SpatialAxis.X,window);
    }
}
