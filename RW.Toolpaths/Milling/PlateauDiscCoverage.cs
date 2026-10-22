using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Subset certificates inside discs whose circumscribed polygons are already
/// published. A convex candidate with every vertex in one disc adds no geometry.</summary>
internal sealed class PlateauDiscCoverage
{
    internal readonly record struct Disc(Point64 Center, long Radius);
    private sealed record Node(long MinX,long MinY,long MaxX,long MaxY,int First,int Count,
        long CenterMinX,long CenterMinY,long CenterMaxX,long CenterMaxY,long MaximumRadius,Node? Left=null,Node? Right=null);
    private readonly Disc[] _discs;
    private readonly Node? _root;
    private readonly CancellationToken _token;
    private int _lastHullWinner=-1;
    internal long Queries { get; private set; }
    internal long Candidates { get; private set; }
    internal long PriorDiscTrials { get; private set; }
    internal long PriorDiscWins { get; private set; }

    internal PlateauDiscCoverage(IEnumerable<Disc> discs,CancellationToken token)
    {
        _discs=discs.Where(d=>d.Radius>0 && d.Radius<(1L<<60)
            && Math.Abs((double)d.Center.X)<(1L<<60) && Math.Abs((double)d.Center.Y)<(1L<<60)).ToArray();
        _token=token;
        if(_discs.Length>0) _root=Build(0,_discs.Length);
    }
    private Node Build(int first,int count)
    {
        _token.ThrowIfCancellationRequested();
        long minX=long.MaxValue,minY=long.MaxValue,maxX=long.MinValue,maxY=long.MinValue;
        long centerMinX=minX,centerMinY=minY,centerMaxX=maxX,centerMaxY=maxY,maximumRadius=0;
        for(int i=first;i<first+count;i++)
        {
            var d=_discs[i];minX=Math.Min(minX,d.Center.X-d.Radius);minY=Math.Min(minY,d.Center.Y-d.Radius);
            maxX=Math.Max(maxX,d.Center.X+d.Radius);maxY=Math.Max(maxY,d.Center.Y+d.Radius);
            centerMinX=Math.Min(centerMinX,d.Center.X);centerMinY=Math.Min(centerMinY,d.Center.Y);
            centerMaxX=Math.Max(centerMaxX,d.Center.X);centerMaxY=Math.Max(centerMaxY,d.Center.Y);
            maximumRadius=Math.Max(maximumRadius,d.Radius);
        }
        if(count<=12) return new(minX,minY,maxX,maxY,first,count,centerMinX,centerMinY,centerMaxX,centerMaxY,maximumRadius);
        bool x=maxX-minX>=maxY-minY;
        Array.Sort(_discs,first,count,Comparer<Disc>.Create((a,b)=>x?a.Center.X.CompareTo(b.Center.X):a.Center.Y.CompareTo(b.Center.Y)));
        int half=count/2;
        return new(minX,minY,maxX,maxY,first,0,centerMinX,centerMinY,centerMaxX,centerMaxY,maximumRadius,Build(first,half),Build(first+half,count-half));
    }
    internal bool Contains(Path64 polygon)
    {
        _token.ThrowIfCancellationRequested();Queries++;
        if(polygon.Count<3 || _root==null) return false;
        long minX=polygon.Min(p=>p.X),minY=polygon.Min(p=>p.Y),maxX=polygon.Max(p=>p.X),maxY=polygon.Max(p=>p.Y);
        return Visit(_root);
        bool Visit(Node node)
        {
            if(node.MinX>minX || node.MinY>minY || node.MaxX<maxX || node.MaxY<maxY) return false;
            if(node.Left!=null) return Visit(node.Left) || Visit(node.Right!);
            for(int i=node.First;i<node.First+node.Count;i++)
            {
                var d=_discs[i];
                if(d.Center.X-d.Radius>minX || d.Center.Y-d.Radius>minY
                    || d.Center.X+d.Radius<maxX || d.Center.Y+d.Radius<maxY) continue;
                Candidates++;
                Int128 square=(Int128)d.Radius*d.Radius;
                bool inside=true;
                foreach(var p in polygon)
                {
                    // Source coordinates were admitted to Clipper's integer range.
                    // Guard before subtraction so even an unsupported extreme declines.
                    if(Math.Abs((double)p.X)>=(1L<<60) || Math.Abs((double)p.Y)>=(1L<<60)) {inside=false;break;}
                    Int128 dx=(Int128)p.X-d.Center.X,dy=(Int128)p.Y-d.Center.Y;
                    if(dx*dx+dy*dy>square) {inside=false;break;}
                }
                if(inside) return true;
            }
            return false;
        }
    }

    internal bool ContainsHull(Point64 a,long ra,Point64 b,long rb)
    {
        _token.ThrowIfCancellationRequested();Queries++;
        if(_root==null || ra<0 || rb<0 || ra>=(1L<<60) || rb>=(1L<<60)
            || Math.Abs((double)a.X)>=(1L<<60) || Math.Abs((double)a.Y)>=(1L<<60)
            || Math.Abs((double)b.X)>=(1L<<60) || Math.Abs((double)b.Y)>=(1L<<60)) return false;
        long minX=Math.Min(a.X-ra,b.X-rb),minY=Math.Min(a.Y-ra,b.Y-rb);
        long maxX=Math.Max(a.X+ra,b.X+rb),maxY=Math.Max(a.Y+ra,b.Y+rb);
        // Nominate only the actual earlier containing disc. Re-evaluate both
        // COMPLETE new endpoint disks with the same exact integer predicate.
        // No previous query's permission or geometry result is transferred.
        if(_lastHullWinner>=0)
        {
            PriorDiscTrials++;
            if(FitsBoth(_discs[_lastHullWinner])){PriorDiscWins++;return true;}
        }
        return Visit(_root);
        bool FitsBoth(Disc d)
        {
            if(d.Radius<ra||d.Radius<rb)return false;
            bool Fits(Point64 p,long radius)
            {
                Int128 dx=(Int128)p.X-d.Center.X,dy=(Int128)p.Y-d.Center.Y;
                Int128 gap=d.Radius-radius;
                return dx*dx+dy*dy<=gap*gap;
            }
            return Fits(a,ra)&&Fits(b,rb);
        }
        bool Visit(Node node)
        {
            if(node.MinX>minX || node.MinY>minY || node.MaxX<maxX || node.MaxY<maxY) return false;
            // Exact integer lower distance from each candidate endpoint to
            // every source centre in this subtree. Combined with its maximum
            // radius this can only reject impossible containing discs.
            bool Possible(Point64 p,long radius)
            {
                if(node.MaximumRadius<radius)return false;
                Int128 dx= p.X<node.CenterMinX?(Int128)node.CenterMinX-p.X:
                    p.X>node.CenterMaxX?(Int128)p.X-node.CenterMaxX:0;
                Int128 dy= p.Y<node.CenterMinY?(Int128)node.CenterMinY-p.Y:
                    p.Y>node.CenterMaxY?(Int128)p.Y-node.CenterMaxY:0;
                Int128 gap=(Int128)node.MaximumRadius-radius;
                return dx*dx+dy*dy<=gap*gap;
            }
            if(!Possible(a,ra)||!Possible(b,rb))return false;
            if(node.Left!=null) return Visit(node.Left) || Visit(node.Right!);
            for(int i=node.First;i<node.First+node.Count;i++)
            {
                var d=_discs[i];
                if(d.Radius<ra || d.Radius<rb || d.Center.X-d.Radius>minX || d.Center.Y-d.Radius>minY
                    || d.Center.X+d.Radius<maxX || d.Center.Y+d.Radius<maxY) continue;
                Candidates++;
                if(FitsBoth(d)){_lastHullWinner=i;return true;}
            }
            return false;
        }
    }
}
