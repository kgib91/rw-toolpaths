using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Bound the swept surface rather than preserve incidental subdivision
/// points. A finite 3D chord capsule contains every original linear segment;
/// continuity also bounds the reverse distance back to the original trace.
/// Cutter orientation is fixed, so translation sweeps inherit the same bound.
/// Nomination callers must still apply their floor, wall and posting guards.</summary>
public static class SurfaceToolpathSimplifier
{
    public static List<TaggedToolpath> Nominate(IReadOnlyList<TaggedToolpath> paths,
        double tolerance,CancellationToken token,Action<string>? diagnostic=null,string stage="nomination")
    {
        long started=Stopwatch.GetTimestamp(),before=0,after=0,beforeSpans=0,afterSpans=0;
        var result=new List<TaggedToolpath>(paths.Count);
        foreach(var path in paths)
        {
            token.ThrowIfCancellationRequested();before+=path.Points.Count;beforeSpans+=path.Spans.Count;
            var compact=Simplify(path,tolerance,token);after+=compact.Points.Count;afterSpans+=compact.Spans.Count;result.Add(compact);
        }
        diagnostic?.Invoke($"stage=surface-simplification event=complete phase={stage} paths={paths.Count} sourcePoints={before} retainedPoints={after} sourceSpans={beforeSpans} retainedSpans={afterSpans} "
            +$"chordBoundMm={tolerance:R} surfaceBudgetMm={MachiningAccuracy.SurfaceDeviation:R} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
            +"finite3DCapsules=True endpointsAndMotionKindBoundaries=True adjacentCutSpanSubdivisionRequired=False originalPathEqualityRequired=False downstreamPhysicalProtectionRequired=True");
        return result;
    }

    public static TaggedToolpath Simplify(TaggedToolpath path,double tolerance,CancellationToken token,
        Func<Point3D,Point3D,bool>? accept=null)
    {
        token.ThrowIfCancellationRequested();
        if(!(tolerance>0)||!double.IsFinite(tolerance))throw new ArgumentOutOfRangeException(nameof(tolerance));
        var p=path.Points;int count=p.Count;
        if(count<4 || path.DetectCurves || path.ContinuationCircuitDefinition is not null)return path;
        var keep=new bool[count];keep[0]=keep[1]=keep[^1]=true;
        // Keep the first explicit planar word, even when its XY change rounds
        // into the same F3 cell. A preceding vertical-only prefix cannot acquire
        // a different machine XY merely because its geometric trace is close.
        for(int i=1;i<count;i++)
            if(!ToolpathCommandRounding.IsVerticalOnly((float)p[i].X-(float)p[i-1].X,
                (float)p[i].Z-(float)p[i-1].Z,(float)p[i].Y-(float)p[i-1].Y))
            {keep[i-1]=keep[i]=true;break;}
        var semanticSpans=new List<ToolpathSpan>(path.Spans.Count);
        foreach(var span in path.Spans)
        {
            if(span.StartIndex<0||span.EndIndex>=count||span.EndIndex<span.StartIndex)return path;
            if(semanticSpans.Count>0&&span.Kind==ToolpathSpanKind.Cut
                &&semanticSpans[^1].Kind==ToolpathSpanKind.Cut&&semanticSpans[^1].EndIndex==span.StartIndex)
                semanticSpans[^1]=new ToolpathSpan(semanticSpans[^1].StartIndex,span.EndIndex,span.Kind);
            else semanticSpans.Add(span);
        }
        foreach(var span in semanticSpans)
        {
            keep[span.StartIndex]=keep[span.EndIndex]=true;
            // These points carry an independent air / holding contract.
            if(span.Kind is ToolpathSpanKind.ClearedTravel or ToolpathSpanKind.TabLift)
                for(int i=span.StartIndex;i<=span.EndIndex;i++)keep[i]=true;
        }
        // Keep a genuine tiny loop / out-and-back even when its entire trace
        // is smaller than the approximation budget. Do not erase a feature.
        if(p[0]==p[^1])
        {
            int far=1;double farDistance=0;
            for(int i=1;i<count-1;i++)
            {
                if((i&1023)==0)token.ThrowIfCancellationRequested();
                double d=DistanceSquared(p[i],p[0],p[0]);
                if(d>farDistance){farDistance=d;far=i;}
            }
            keep[far]=true;
        }
        double square=tolerance*tolerance;var pending=new Stack<(int,int)>();
        int first=0;
        for(int i=1;i<count;i++)if(keep[i]){pending.Push((first,i));first=i;}
        long visits=0;
        while(pending.TryPop(out var range))
        {
            token.ThrowIfCancellationRequested();int a=range.Item1,b=range.Item2;
            if(b-a<2)continue;
            double maximum=0;int split=a+1;
            for(int i=a+1;i<b;i++)
            {
                if((++visits&1023)==0)token.ThrowIfCancellationRequested();
                if(visits>64L*count)return path;
                double d=DistanceSquared(p[i],p[a],p[b]);
                if(!double.IsFinite(d))return path;
                if(d>maximum){maximum=d;split=i;}
            }
            if(maximum<=square && (accept is null || accept(p[a],p[b])))continue;
            if(maximum<=square)split=a+(b-a)/2;
            keep[split]=true;pending.Push((a,split));pending.Push((split,b));
        }
        int retained=keep.Count(v=>v);if(retained==count)return path;
        var selected=new List<Point3D>(retained);var map=new int[count];Array.Fill(map,-1);
        for(int i=0;i<count;i++)if(keep[i]){map[i]=selected.Count;selected.Add(p[i]);}
        var spans=semanticSpans.Select(s=>new ToolpathSpan(map[s.StartIndex],map[s.EndIndex],s.Kind)).ToArray();
        return new TaggedToolpath(selected,spans,path.RegionIndex,path.Category,
            path.DepthPassIndex,path.VisitIdentity,detectCurves:false)
            {ContinuationComponentIndex=path.ContinuationComponentIndex,ContinuationPlan=path.ContinuationPlan,
                ContinuationSourceOrdinal=path.ContinuationSourceOrdinal,ContinuationCircuit=path.ContinuationCircuit,
                ContinuationCircuitDefinition=path.ContinuationCircuitDefinition};
    }

    internal static double DistanceSquared(Point3D p,Point3D a,Point3D b)
    {
        double x=b.X-a.X,y=b.Y-a.Y,z=b.Z-a.Z,length=x*x+y*y+z*z;
        double t=length>0?Math.Clamp(((p.X-a.X)*x+(p.Y-a.Y)*y+(p.Z-a.Z)*z)/length,0,1):0;
        x=p.X-a.X-t*x;y=p.Y-a.Y-t*y;z=p.Z-a.Z-t*z;return x*x+y*y+z*z;
    }
}
