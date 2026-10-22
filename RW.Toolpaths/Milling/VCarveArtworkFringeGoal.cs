using System.Diagnostics;
using Clipper2Lib;

namespace RW.Toolpaths.Milling;

// Target policy only. This helper owns no cutter, floor, wall, grade or stock
// proof. The caller retains those complete original physical authorities.
internal static class VCarveArtworkFringeGoal
{
    internal const string SelectionEnvironment="RW_TOOLPATHS_VCARVE_ARTWORK_FRINGE_GOAL";
    internal const double NominalFringeMm=.008;
    // Subtract four explicit Native ticks from the requested inset so the
    // construction allowance keeps the ignored fringe within the existing
    // 8um XY policy. This is NOT a material-height or machining tolerance.
    internal const int DirectedConstructionReserveNative=4;
    private const long MaximumPoints=4_000_000;
    private const double MaximumSeconds=5;
    private const long MaximumAbsoluteNativeCoordinate=1L<<29;

    internal static List<List<PointD>> ApplyToFixedTarget(List<List<PointD>> originalGoal,
        IReadOnlyList<IReadOnlyList<PointD>> originalArtwork,bool enabled,
        CancellationToken token=default,Action<string>? diagnostic=null)
    {
        token.ThrowIfCancellationRequested();
        if(!enabled)
        {
            diagnostic?.Invoke("stage=artwork-fringe-goal event=selection enabled=False targetPolicy=literal-original-fixed-goal physicalGuardsUnchanged=True");
            return originalGoal;
        }
        long started=Stopwatch.GetTimestamp();string fallback="unsupported";long submitted=0;
        List<List<PointD>>? result=null;double oldArea=0,newArea=0,droppedArea=0;int oldRings=0,newRings=0;
        Rect64 oldBounds=default,newBounds=default;
        bool Timely(){token.ThrowIfCancellationRequested();return Stopwatch.GetElapsedTime(started).TotalSeconds<=MaximumSeconds;}
        bool Convert(IEnumerable<IReadOnlyList<PointD>> rings,out Paths64 native)
        {
            native=new();foreach(var ring in rings)
            {
                if(!Timely()||ring.Count<3||(submitted+=ring.Count)>MaximumPoints)return false;
                var path=new Path64(ring.Count);
                foreach(var point in ring)
                {
                    if(!double.IsFinite(point.x)||!double.IsFinite(point.y))return false;
                    double x=Math.Round(point.x*PathUtils.Scale),y=Math.Round(point.y*PathUtils.Scale);
                    if(Math.Abs(x)>=MaximumAbsoluteNativeCoordinate||Math.Abs(y)>=MaximumAbsoluteNativeCoordinate)return false;
                    path.Add(new((long)x,(long)y));
                }
                native.Add(path);
            }
            return native.Count>0;
        }
        try
        {
            if(!Convert(originalGoal,out var goal)||!Convert(originalArtwork,out var artwork))return Finish("unsupported-input");
            token.ThrowIfCancellationRequested();artwork=Clipper.Union(artwork,FillRule.NonZero);
            if(!Timely()||artwork.Count==0)return Finish("artwork-union-empty-or-deadline");
            double insetNative=NominalFringeMm*PathUtils.Scale-DirectedConstructionReserveNative;
            var offset=new ClipperOffset(miterLimit:2,arcTolerance:1);
            offset.AddPaths(artwork,JoinType.Round,EndType.Polygon);
            var interior=new Paths64();offset.Execute(-insetNative,interior);
            if(!Timely()||interior.Count==0)return Finish("whole-mask-empty-or-deadline");
            interior=Clipper.Union(interior,FillRule.NonZero);
            if(!Timely())return Finish("mask-union-deadline");
            var masked=Clipper.Intersect(goal,interior,FillRule.NonZero);
            if(!Timely()||masked.Count==0)return Finish("whole-goal-empty-or-deadline");
            // Intersection defines the new fixed goal. Reproving it by a
            // second rounded Difference can create Boolean debris and is not
            // a machining guard or a required path-matching experiment.
            oldArea=goal.Sum(Clipper.Area)/(PathUtils.Scale*PathUtils.Scale);
            newArea=masked.Sum(Clipper.Area)/(PathUtils.Scale*PathUtils.Scale);
            droppedArea=oldArea-newArea;
            oldRings=goal.Count;newRings=masked.Count;oldBounds=Clipper.GetBounds(goal);newBounds=Clipper.GetBounds(masked);
            result=masked.Select(r=>r.Select(p=>new PointD(p.X/PathUtils.Scale,p.Y/PathUtils.Scale)).ToList()).ToList();
            if(!Timely()){result=null;return Finish("conversion-deadline");}
            return Finish(null);
        }
        catch(OperationCanceledException){throw;}
        catch(Exception error) when(error is ArgumentException or InvalidOperationException or OverflowException)
        {return Finish("native-construction-"+error.GetType().Name);}

        List<List<PointD>> Finish(string? reason)
        {
            token.ThrowIfCancellationRequested();bool applied=reason is null&&result is not null;fallback=reason??"none";
            diagnostic?.Invoke($"stage=artwork-fringe-goal event=fixed-target enabled=True applied={applied} fallback={fallback} "
                +$"nominalArtworkFringeMm={NominalFringeMm:R} directedConstructionReserveNative={DirectedConstructionReserveNative} "
                +$"constructorInsetMm={NominalFringeMm-DirectedConstructionReserveNative/PathUtils.Scale:R} arcNative=1 droppedAreaOwner=signed-old-minus-new-area "
                +$"submittedPoints={submitted} originalGoalRings={oldRings} goalRings={newRings} "
                +$"originalGoalArea={oldArea:R} goalArea={newArea:R} droppedGoalArea={droppedArea:R} "
                +$"originalGoalBoundsNative=({oldBounds.left},{oldBounds.top})-({oldBounds.right},{oldBounds.bottom}) "
                +$"goalBoundsNative=({newBounds.left},{newBounds.top})-({newBounds.right},{newBounds.bottom}) "
                +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:R} maximumSeconds={MaximumSeconds:R} maximumPoints={MaximumPoints} "
                +"maskOwner=full-original-artwork fixedBeforeCorrections=True sameGoalForResidualAndWholeQ=True "
                +"originalReferenceSourceAndAllowedChannelRetained=True originalFloorArtworkWallsAndGradeRetained=True "
                +"wholeOldGoalOnUnsupported=True physicalStockCredit=False globalVisibilityAccepted=False");
            return applied?result!:originalGoal;
        }
    }
}
