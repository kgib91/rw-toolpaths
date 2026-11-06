using Clipper2Lib;
namespace RW.Toolpaths.Milling;
public sealed record VCarveNativeStockEvidence(IReadOnlyList<IReadOnlyList<PointD>> Target,
    VCarveClearedStock Stock,VCarveCutterProfile Profile,double MaximumDepth,double SurfaceZ);
/// <summary>Exact immutable inputs to one primary continuous-stock index.</summary>
public sealed record VCarveNativeStockIndexEvidence(IReadOnlyList<IReadOnlyList<PointD>> Target,
    IReadOnlyList<VCarveRoughingChord> Columns,IReadOnlyList<VCarveClearedSweep> ProfileSweeps,
    IReadOnlyList<PointD> PartBoundary,VCarveCutterProfile Profile,double MaximumDepth,bool SpatialDepthTies,
    bool HasRemovalGauge=false,int IndexedChords=0);
/// <summary>Diagnostic result of one complete disk query, including literal seed identity.</summary>
public readonly record struct VCarveNativeStockQuery(double X,double Y,double Radius,bool InnerReserve,
    double ExpectedDepth,int SeedBefore,int SeedAfter);
public static class VCarveNativeStockTrace
{
    // Diagnostic snapshot only, unset in the client; never grants authority.
    public static readonly AsyncLocal<Action<VCarveNativeStockEvidence>?> Observer=new();
    // The factory is consulted only by primary owners. Private workers never
    // inherit a sink; diagnostic capture cannot supply geometry or a result.
    public static readonly AsyncLocal<Func<VCarveNativeStockIndexEvidence,Action<VCarveNativeStockQuery>?>?> QueryObserverFactory=new();
}
