using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Greedy visits measured from the exit actually produced by the previous
/// visit. A bounded sample index nominates paths; only the selected path's literal
/// vertices are scanned for its seam. It never synthesizes a cutting connector.</summary>
internal static class ToolpathVisitOrder
{
    internal static int[] Order(IReadOnlyList<TaggedToolpath> paths, IReadOnlyList<bool> movable,
        Point3D? start, CancellationToken token)
    {
        var used=new bool[paths.Count];var order=new int[paths.Count];
        var samples=new List<Point3D>();
        void Samples()
        {
            samples.Clear();
            for(int owner=0;owner<paths.Count;owner++)
            {
                if(used[owner])continue;
                var points=paths[owner].Points;
                bool closed=points[0]==points[^1];
                int count=movable[owner] ? closed ? Math.Max(1,Math.Min(16,points.Count-1)) : 2 : 1;
                for(int i=0;i<count;i++)
                {
                    int at=!movable[owner] ? 0 : closed ? (int)((long)i*(points.Count-1)/count) : i==0 ? 0 : points.Count-1;
                    // This index uses XY only. Z carries the path ordinal locally;
                    // physical coordinates always come from the original path.
                    samples.Add(new(points[at].X,points[at].Y,owner));
                }
            }
        }
        Samples();var index=new PreparedPointIndex(samples,token);
        Point3D? cursor=start;
        for(int visit=0;visit<paths.Count;visit++)
        {
            token.ThrowIfCancellationRequested();int next=-1;
            if(cursor is { } from)
            {
                foreach(var sample in index.Nearest(from.X,from.Y,256,token))
                    if(!used[(int)sample.Z]){next=(int)sample.Z;break;}
                if(next<0)
                {
                    // Bound stale-entry scans around completed large contours.
                    Samples();index=new PreparedPointIndex(samples,token);
                    next=(int)index.Nearest(from.X,from.Y,1,token).First().Z;
                }
            }
            else next=Array.FindIndex(used,p=>!p);
            used[next]=true;order[visit]=next;
            var path=paths[next];int seam=movable[next]&&cursor is {} p ? EntryIndex(path,p,token) : 0;
            cursor=path.Points[0]==path.Points[^1] ? path.Points[seam]
                : seam==path.Points.Count-1 ? path.Points[0] : path.Points[^1];
        }
        return order;
    }

    internal static int EntryIndex(TaggedToolpath path, Point3D from, CancellationToken token)
    {
        bool closed=path.Points[0]==path.Points[^1];int best=0;double distance=double.PositiveInfinity;
        for(int i=0;i<path.Points.Count;i+=closed ? 1 : Math.Max(1,path.Points.Count-1))
        {
            if((i&1023)==0)token.ThrowIfCancellationRequested();
            var p=path.Points[i];double dx=p.X-from.X,dy=p.Y-from.Y,d=dx*dx+dy*dy;
            if(d<distance){distance=d;best=i;}
        }
        return best;
    }
}
