using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Match trimmed pieces to their constructed predecessor OFFSET ROW.
/// Empty gaps do not force unrelated pieces of a board-wide ring to finish
/// first. These links only order existing cuts; they grant no stock permission.</summary>
internal sealed class VCarveOffsetPieces
{
    internal readonly HashSet<(int First,int Next)> Links=new();
    internal long Queries,BoundedQueries;
    private sealed record Segment(int Owner,Point64 A,Point64 B);
    private sealed record Sites(PolygonOps.BoundarySegmentIndex Index,Segment[] Segments);
    internal VCarveOffsetPieces(IReadOnlyList<TaggedToolpath> paths,int[][] pieces,bool bottomUp,double reach,CancellationToken token)
    {
        (VCarveRingOrder.Plan Plan,int Ring) Key(int piece)
        {
            var v=paths[pieces[piece][0]].ConcentricSource!;
            return(v.Plan,v.Ring);
        }
        var levels=Enumerable.Range(0,pieces.Length).GroupBy(Key).ToDictionary(g=>g.Key,g=>g.ToArray());
        var indices=new Dictionary<(VCarveRingOrder.Plan,int),Sites>();var matches=new List<int>();
        long extent=double.IsFinite(reach)?(long)Math.Ceiling(reach*PathUtils.Scale)+2:0;
        double distance=double.IsFinite(reach)?Math.Pow(reach*PathUtils.Scale+2,2):double.PositiveInfinity;
        foreach(int child in Enumerable.Range(0,pieces.Length))
        {
            if(paths[pieces[child][0]].ConcentricSource!.Wall>=0)continue;
            token.ThrowIfCancellationRequested();var key=Key(child);
            int previous=key.Plan.Parents[key.Ring];
            while(previous>=0&&!levels.ContainsKey((key.Plan,previous)))previous=key.Plan.Parents[previous];
            if(previous<0)continue;
            var priorKey=(key.Plan,previous);var prior=levels[priorKey];var parents=new HashSet<int>();
            if(!double.IsFinite(reach))parents.UnionWith(prior);
            else
            {
                if(!indices.TryGetValue(priorKey,out var sites))
                {
                    var segments=new List<Segment>();var bounds=new Paths64();var seen=new HashSet<(int,Point64,Point64)>();
                    foreach(int owner in prior)foreach(int path in pieces[owner])
                    for(int i=1;i<paths[path].Points.Count;i++)
                    {
                        if((i&1023)==0)token.ThrowIfCancellationRequested();
                        var a=XY(paths[path].Points[i-1]);var b=XY(paths[path].Points[i]);
                        if(a==b||!seen.Add((owner,a,b)))continue;seen.Add((owner,b,a));
                        segments.Add(new(owner,a,b));bounds.Add(new(){a,b});
                    }
                    sites=new(new PolygonOps.BoundarySegmentIndex(bounds),segments.ToArray());indices.Add(priorKey,sites);
                }
                bool bounded=false;
                foreach(int path in pieces[child])
                {
                    for(int i=1;i<paths[path].Points.Count;i++)
                    {
                        token.ThrowIfCancellationRequested();Queries++;
                        var a=XY(paths[path].Points[i-1]);var b=XY(paths[path].Points[i]);
                        if(!sites.Index.QueryBounds(Math.Min(a.X,b.X)-extent,Math.Min(a.Y,b.Y)-extent,
                            Math.Max(a.X,b.X)+extent,Math.Max(a.Y,b.Y)+extent,matches,4096,token))
                        {parents.UnionWith(prior);BoundedQueries++;bounded=true;break;}
                        foreach(int ordinal in matches)
                        {
                            var s=sites.Segments[ordinal/2];
                            if(Separation(a,b,s.A,s.B)<=distance)parents.Add(s.Owner);
                        }
                    }
                    if(bounded)break;
                }
            }
            foreach(int parent in parents)Links.Add(bottomUp?(child,parent):(parent,child));
        }
        if(MillingGeometryTrace.Observer.Value is not null)
            MillingGeometryTrace.Capture("ring-offset-pieces",0,Links.Select(pair=>new TaggedToolpath(
                new[]{paths[pieces[pair.First][0]].Points[0],paths[pieces[pair.Next][0]].Points[0]},Array.Empty<ToolpathSpan>(),
                paths[pieces[pair.Next][0]].RegionIndex,"offset-level-order",null,paths[pieces[pair.Next][0]].ContinuationSourceOrdinal,detectCurves:false)
                {ContinuationSourceOrdinal=paths[pieces[pair.First][0]].ContinuationSourceOrdinal}).ToArray());
    }
    private static Point64 XY(Point3D p)=>new((long)Math.Round(p.X*PathUtils.Scale),(long)Math.Round(p.Y*PathUtils.Scale));
    private static double Separation(Point64 a,Point64 b,Point64 c,Point64 d)
    {
        double x=(double)b.X-a.X,y=(double)b.Y-a.Y,u=(double)d.X-c.X,v=(double)d.Y-c.Y;
        double qx=(double)c.X-a.X,qy=(double)c.Y-a.Y,den=x*v-y*u;
        if(den!=0){double t=(qx*v-qy*u)/den,s=(qx*y-qy*x)/den;if(t>=0&&t<=1&&s>=0&&s<=1)return 0;}
        static double Point(Point64 p,Point64 first,Point64 last)
        {
            double x=(double)last.X-first.X,y=(double)last.Y-first.Y,square=x*x+y*y;
            double t=square>0?Math.Clamp((((double)p.X-first.X)*x+((double)p.Y-first.Y)*y)/square,0,1):0;
            return Math.Pow(p.X-first.X-t*x,2)+Math.Pow(p.Y-first.Y-t*y,2);
        }
        return Math.Min(Math.Min(Point(a,c,d),Point(b,c,d)),Math.Min(Point(c,a,b),Point(d,a,b)));
    }
}
