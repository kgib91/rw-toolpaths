namespace RW.Toolpaths.Milling;

/// <summary>Local seam choices on the already selected constructed row. No
/// global neighbour index, intersection graph or reconstructed row order.</summary>
internal sealed class VCarveRingSeams
{
    internal readonly record struct Landing(int Owner,int End,double Parameter,Point3D Point,double DistanceSquared);
    private readonly IReadOnlyList<TaggedToolpath> paths;
    private readonly CancellationToken token;
    internal long SplitRefusals{get;private set;}
    internal long SplitLandings{get;private set;}
    internal VCarveRingSeams(IReadOnlyList<TaggedToolpath> paths,CancellationToken token)
    {this.paths=paths;this.token=token;}
    internal IEnumerable<Landing> Nearest(Point3D from,IEnumerable<int> owners,double reach)
    {
        var found=new List<Landing>();var seen=new HashSet<(int,Point3D)>();
        foreach(int owner in owners)
        for(int end=1;end<paths[owner].Points.Count;end++)
        {
            if((end&1023)==0)token.ThrowIfCancellationRequested();
            var a=paths[owner].Points[end-1];var b=paths[owner].Points[end];
            double dx=b.X-a.X,dy=b.Y-a.Y,square=dx*dx+dy*dy;if(square==0)continue;
            double t=Math.Clamp(((from.X-a.X)*dx+(from.Y-a.Y)*dy)/square,0,1);
            var point=t==0?a:t==1?b:new(a.X+t*dx,a.Y+t*dy,a.Z+t*(b.Z-a.Z));
            Add(t,point);Add(0,a);Add(1,b);
            void Add(double parameter,Point3D candidate)
            {
                double distance=Math.Pow(from.X-candidate.X,2)+Math.Pow(from.Y-candidate.Y,2);
                if(distance<=reach*reach&&seen.Add((owner,candidate)))found.Add(new(owner,end,parameter,candidate,distance));
            }
        }
        return found.OrderBy(p=>p.DistanceSquared).ThenBy(p=>p.Owner).ThenBy(p=>p.End);
    }
    internal TaggedToolpath? Rebase(Landing landing,Func<Point3D,Point3D,bool> guard)
    {
        var original=paths[landing.Owner];int end=landing.End;
        if(landing.Parameter==0)return VCarveContinuationAdapter.Rebase(original,end-1);
        if(landing.Parameter==1)return VCarveContinuationAdapter.Rebase(original,end);
        var a=original.Points[end-1];var b=original.Points[end];var p=landing.Point;
        if(!guard(a,p)||!guard(p,b)){SplitRefusals++;return null;}
        var points=original.Points.Take(end).Append(p).Concat(original.Points.Skip(end)).ToArray();
        var split=original with{Points=points,Moves=new LineToolMoves(points),
            Spans=new[]{new ToolpathSpan(0,points.Length-1,ToolpathSpanKind.Cut)}};
        SplitLandings++;return VCarveContinuationAdapter.Rebase(split,end);
    }
}
