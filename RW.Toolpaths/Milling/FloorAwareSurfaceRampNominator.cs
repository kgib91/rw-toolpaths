using Clipper2Lib;
using RW.Toolpaths;

namespace RW.Toolpaths.Milling.Artifacts;

/// <summary>Bounded geometry nomination from an admitted actual endpoint and an original source direction.
/// The caller retains the original floor, both walls, grade, schedule, actual-command and stock proof owners.</summary>
public static class FloorAwareSurfaceRampNominator
{
    public sealed record Request(Point3D ActualEnd, PointD OriginalCenter, double Grade,
        double MaximumDepth, double DepthPerPass, int RegionIndex, string Category,
        int? DepthPassIndex, int? VisitIdentity = null);
    public sealed record Candidate(TaggedToolpath Path, double Factor, double Length,
        double MinimumLength, double F3RoundingReserve);

    // F3 XY rounding is <=.001 per coordinate under the captured ordinary FLOAT domain.
    // Two planar rounding errors plus four native ticks are charged BEFORE the caller's exact actual-grade check.
    public static bool TryNominate(Request request, Func<Point3D,Point3D> actualCommandPoint,
        CancellationToken token, out Candidate[] candidates, out string reason)
    {
        ArgumentNullException.ThrowIfNull(request);ArgumentNullException.ThrowIfNull(actualCommandPoint);
        token.ThrowIfCancellationRequested();candidates=Array.Empty<Candidate>();reason="unsupported-request";
        var end=request.ActualEnd;
        if(!new[]{end.X,end.Y,request.OriginalCenter.x,request.OriginalCenter.y}.All(v=>double.IsFinite(v)&&Math.Abs(v)<=10000)
            || !double.IsFinite(end.Z)||!(end.Z<0)||!double.IsFinite(request.Grade)||!(request.Grade>0)
            || !double.IsFinite(request.MaximumDepth)||!double.IsFinite(request.DepthPerPass)
            || -end.Z>request.MaximumDepth||-end.Z>request.DepthPerPass||string.IsNullOrEmpty(request.Category))return false;
        if(actualCommandPoint(end)!=end){reason="actual-end-not-command-fixed-point";return false;}
        double dx=request.OriginalCenter.x-end.X,dy=request.OriginalCenter.y-end.Y;
        double remaining=Math.Sqrt(dx*dx+dy*dy),reserve=2*Math.Sqrt(2)*.001+4/32768d;
        double minimum=-end.Z/request.Grade+reserve;
        if(!double.IsFinite(remaining)||!(remaining>=minimum)){reason="original-direction-too-short";return false;}
        var result=new List<Candidate>(4);var usedLengths=new HashSet<long>();
        foreach(double factor in new[]{1d,1.25,1.5,2})
        {
            token.ThrowIfCancellationRequested();double length=Math.Min(minimum*factor,remaining);
            if(!usedLengths.Add(BitConverter.DoubleToInt64Bits(length)))continue;
            var start=actualCommandPoint(new(end.X+dx*length/remaining,end.Y+dy*length/remaining,0));
            if(start.Z!=0||start.X==end.X&&start.Y==end.Y)continue;
            var points=Array.AsReadOnly(new[]{start,end});var spans=Array.AsReadOnly(new[]{new ToolpathSpan(0,1,ToolpathSpanKind.Ramp)});
            var path=new TaggedToolpath(points,spans,request.RegionIndex,request.Category,request.DepthPassIndex,request.VisitIdentity,detectCurves:false);
            result.Add(new(path,factor,length,minimum,reserve));
        }
        candidates=result.ToArray();reason=candidates.Length>0?"bounded-original-direction-ramp-nomination":"no-finite-command-ramp";
        return candidates.Length>0;
    }
}
