using Clipper2Lib;
namespace RW.Toolpaths.Milling.ReviewedSource;
public sealed record ReviewedVCarveRegion(int Id,IReadOnlyList<IReadOnlyList<PointD>> Boundary,IReadOnlyList<IReadOnlyList<PointD>> PreparedBoundary);
public sealed record ReviewedVCarveSchedule(double Surface,double Depth,double Stepdown,double StartingDepth);
public sealed record ReviewedVCarveSourceSettings(double SourceCurveToleranceMm,double MedialCurveCentralAngleRadians,double GeneratingSiteCornerAngleRadians);
internal static class ReviewedVCarveMath
{
 internal static PointD Sub(PointD a,PointD b)=>new(a.x-b.x,a.y-b.y);
 internal static double Cross(PointD a,PointD b)=>a.x*b.y-a.y*b.x;
 internal static double DistanceSquared(PointD a,PointD b)=>(a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
}
