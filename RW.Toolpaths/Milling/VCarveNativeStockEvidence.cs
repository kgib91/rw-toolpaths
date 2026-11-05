using Clipper2Lib;
namespace RW.Toolpaths.Milling;
public sealed record VCarveNativeStockEvidence(IReadOnlyList<IReadOnlyList<PointD>> Target,
    VCarveClearedStock Stock,VCarveCutterProfile Profile,double MaximumDepth,double SurfaceZ);
public static class VCarveNativeStockTrace
{
    // Diagnostic snapshot only, unset in the client; never grants authority.
    public static readonly AsyncLocal<Action<VCarveNativeStockEvidence>?> Observer=new();
}
