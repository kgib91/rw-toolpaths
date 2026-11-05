using RW.Toolpaths.Geometry;
using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>The active ring excursion stack. Returning reverses literal executed
/// entry chords and follows either arc of a completed ring. An explicit adapter
/// may prove a short cutting-feed step onto the actual stack parent. Source
/// work stays bounded; no subdivision graph or future stock is built.</summary>
internal sealed class VCarveRingReturns
{
    internal sealed class Node
    {
        internal readonly Node? Parent;
        internal readonly Point3D[] Prefix,Body;
        internal readonly PreparedPointIndex Index;
        internal readonly TaggedToolpath Source;
        internal readonly bool Closed;
        internal readonly bool Carrier;
        internal int Position;
        internal double PositionFraction;
        internal (PolygonOps.BoundarySegmentIndex? Index,int[] Edges,int[] Unindexed)? Segments;
        internal Node(Node? parent,Point3D[] prefix,Point3D[] body,bool closed,TaggedToolpath source,CancellationToken token,bool carrier=false)
        {
            Parent=parent;Prefix=prefix;Body=body;Closed=closed;Carrier=carrier;Source=source;Position=closed?0:body.Length-1;
            Index=new PreparedPointIndex(body.Select((p,i)=>new Point3D(p.X,p.Y,i)).ToArray(),token);
        }
    }
    internal sealed record Departure(Node Anchor,int Portal,int Seam,IReadOnlyList<Point3D> Points,double Seconds,int ProvedParentSteps=0,double PortalFraction=0);
    private readonly record struct Location(int Vertex,double Fraction=0);
    private Node? top;
    private readonly CancellationToken token;
    private readonly bool postedEquivalentReturns;
    private readonly Func<Point3D,Point3D,bool>? splitGuard;
    internal long BoundedSearches{get;private set;}
    internal long PartialAxisRefusals{get;private set;}
    internal long PostedEquivalentSteps{get;private set;}
    internal long ParentProjectionTrials{get;private set;}
    internal long ParentProjectionRefusals{get;private set;}
    internal long FractionalDepartures{get;private set;}
    internal long ParentProjectionEdgesExamined{get;private set;}
    internal VCarveRingReturns(CancellationToken token,bool postedEquivalentReturns=false,Func<Point3D,Point3D,bool>? splitGuard=null)
    {this.token=token;this.postedEquivalentReturns=postedEquivalentReturns;this.splitGuard=splitGuard;}
    internal void Clear()=>top=null;
    internal void RecordCarrier(TaggedToolpath source)
    {
        // The caller proves the entire outline from immutable original-material
        // clearance before recording it. Unlike cutting bodies, these explicit
        // ClearedTravel arcs need not first be executed in a redundant full lap.
        if(source.Points.Count<3||source.Points[0]!=source.Points[^1])throw new ArgumentException("A traversal carrier must be closed.");
        top=new(null,[],source.Points.Take(source.Points.Count-1).ToArray(),true,source,token,carrier:true);
    }

    internal void Record(TaggedToolpath path,IReadOnlyList<Point3D>? connector,Node? parent=null,int portal=0,double portalFraction=0)
    {
        if(parent is not null){parent.Position=portal;parent.PositionFraction=portalFraction;}
        else if(connector is not null)parent=top;
        var cut=path.Spans.LastOrDefault(s=>s.Kind==ToolpathSpanKind.Cut);
        bool closed=cut.MoveCount>0&&path.Points[cut.StartIndex]==path.Points[cut.EndIndex];
        int home=cut.MoveCount>0?cut.StartIndex:path.Points.Count-1;
        var body=cut.MoveCount>0?path.Points.Skip(home).Take(cut.MoveCount+(closed?0:1)).ToArray():new[]{path.Points[^1]};
        var prefix=connector is null?Array.Empty<Point3D>():connector.Take(connector.Count-1).Concat(path.Points.Take(home+1)).ToArray();
        top=new(parent,prefix,body,closed,path,token);
    }

    internal IEnumerable<Departure> Departures(TaggedToolpath target,ToolpathMotionCost cost,double budget,double reach,
        Func<Point3D,Point3D,bool>? allowsParentStep=null)
    {
        if(top is null||!(budget>0))yield break;
        var position=new Location(top.Position,top.PositionFraction);
        var route=new List<Point3D>{At(top,position)};double seconds=0;int ancestors=0,parentSteps=0;
        var source=target.Points;bool closed=source[0]==source[^1];
        var samples=closed?Enumerable.Range(0,Math.Min(24,source.Count-1))
            .Select(i=>(int)((long)i*(source.Count-1)/Math.Min(24,source.Count-1))).ToArray():new[]{0,source.Count-1};
        for(var node=top;node is not null;node=node.Parent)
        {
            token.ThrowIfCancellationRequested();if(++ancestors>256||route.Count>65536){BoundedSearches++;yield break;}
            var seen=new HashSet<(Location,int)>();
            if(position.Fraction!=0){
                FractionalDepartures++;
                yield return new(node,position.Vertex,0,route.ToArray(),seconds,parentSteps,position.Fraction);
            }
            foreach(int seam in samples)
            {
                var nearest=new Location((int)node.Index.Nearest(source[seam].X,source[seam].Y,1,token).First().Z);
                IEnumerable<Location> portals=new[]{nearest};
                if(splitGuard is not null)portals=portals.Concat(Projections(node,source[seam],reach,false).Take(1));
                foreach(var portal in portals){
                    if(!seen.Add((portal,seam)))continue;
                    var point=At(node,portal);
                    if(!node.Carrier&&Math.Pow(point.X-source[seam].X,2)+Math.Pow(point.Y-source[seam].Y,2)>reach*reach)continue;
                    var arc=Arc(node,position,portal,cost);
                    if(arc is null)continue;
                    var completed=WithoutLoops(route.Concat(arc.Value.Points.Skip(1)));
                    double completeSeconds=Seconds(completed,cost);if(completeSeconds>budget)continue;
                    if(portal.Fraction!=0)FractionalDepartures++;
                    yield return new(node,portal.Vertex,seam,completed,completeSeconds,parentSteps,portal.Fraction);
                }
            }
            if(node.Parent is null||node.Prefix.Length<2)yield break;
            // Optional adapter-proved step onto this actual stack parent's
            // completed body. This is a fresh cutting-feed chord, not an
            // invented literal return or cleared-stock permission.
            Location? parentPortal=null;double parentCost=double.PositiveInfinity;var current=At(node,position);
            if(allowsParentStep is not null)
            foreach(var site in node.Parent.Index.Nearest(current.X,current.Y,8,token))
            {
                int at=(int)site.Z;var next=node.Parent.Body[at];
                double candidate=cost.ChordSeconds(current,next);
                if(candidate<parentCost&&allowsParentStep(current,next)){parentPortal=new(at);parentCost=candidate;}
            }
            if(allowsParentStep is not null&&splitGuard is not null)
            foreach(var portal in Projections(node.Parent,current,reach,true).OrderBy(p=>cost.ChordSeconds(current,At(node.Parent,p)))){
                var next=At(node.Parent,portal);double candidate=cost.ChordSeconds(current,next);
                if(candidate>=parentCost)continue;ParentProjectionTrials++;
                if(!allowsParentStep(current,next)||!SplitLegal(node.Parent,portal)){ParentProjectionRefusals++;continue;}
                parentPortal=portal;parentCost=candidate;
            }
            if(parentPortal is {} nextPortal){
                route.Add(At(node.Parent,nextPortal));route=WithoutLoops(route).ToList();
                position=nextPortal;parentSteps++;seconds=Seconds(route,cost);
                if(seconds>budget)yield break;
                continue;
            }
            var home=Arc(node,position,new(0),cost);if(home is null)yield break;
            var reverse=node.Prefix.Reverse().ToArray();
            if(!Reversible(reverse))yield break;
            route.AddRange(home.Value.Points.Skip(1));route.AddRange(reverse.Skip(1));
            route=WithoutLoops(route).ToList();seconds=Seconds(route,cost);
            position=new(node.Parent.Position,node.Parent.PositionFraction);
            if(seconds>budget)yield break;
        }
    }
    // The current contour itself supplies departure sites. A long next row's
    // sparse target samples must not hide a short walk around a completed corner.
    internal IEnumerable<Departure> LocalDepartures(ToolpathMotionCost cost,double budget)
    {
        if(top is null||!(budget>0))yield break;
        var position=new Location(top.Position,top.PositionFraction);var origin=At(top,position);
        var portals=top.Body.Length<=32?Enumerable.Range(0,top.Body.Length)
            :top.Index.Nearest(origin.X,origin.Y,32,token).Select(p=>(int)p.Z);
        foreach(int portal in portals)
        {
            token.ThrowIfCancellationRequested();if(portal==position.Vertex&&position.Fraction==0)continue;
            var arc=Arc(top,position,new(portal),cost);
            if(arc is not {} route||route.Seconds>budget)continue;
            yield return new(top,portal,0,route.Points,route.Seconds);
        }
    }
    private static Point3D At(Node node,Location location)
    {
        var a=node.Body[location.Vertex];if(location.Fraction==0)return a;
        var b=node.Body[(location.Vertex+1)%node.Body.Length];double t=location.Fraction;
        return new(a.X+(b.X-a.X)*t,a.Y+(b.Y-a.Y)*t,a.Z+(b.Z-a.Z)*t);
    }
    private bool SplitLegal(Node node,Location location)
    {
        if(location.Fraction==0)return true;if(splitGuard is null)return false;
        var a=node.Body[location.Vertex];var b=node.Body[(location.Vertex+1)%node.Body.Length];var p=At(node,location);
        return splitGuard(a,p)&&splitGuard(p,a)&&splitGuard(p,b)&&splitGuard(b,p);
    }
    private IEnumerable<Location> Projections(Node node,Point3D point,double reach,bool offsets)
    {
        int count=node.Closed?node.Body.Length:node.Body.Length-1;
        if(count>8192){BoundedSearches++;return [];}
        IEnumerable<int> candidates=Enumerable.Range(0,count);
        if(double.IsFinite(reach)&&reach>=0&&count>32){
            if(node.Segments is not {} prepared){
                var paths=new Paths64();var ordinals=new List<int>();var unindexed=new List<int>();
                Point64 XY(Point3D p)=>new((long)Math.Round(p.X*PathUtils.Scale),(long)Math.Round(p.Y*PathUtils.Scale));
                for(int i=0;i<count;i++){
                    token.ThrowIfCancellationRequested();var a=XY(node.Body[i]);var b=XY(node.Body[(i+1)%node.Body.Length]);
                    if(a==b){unindexed.Add(i);continue;}paths.Add(new(){a,b});ordinals.Add(i);
                }
                prepared=(paths.Count==0?null:new(paths),ordinals.ToArray(),unindexed.ToArray());node.Segments=prepared;
            }
            long x=(long)Math.Round(point.X*PathUtils.Scale),y=(long)Math.Round(point.Y*PathUtils.Scale);
            long radius=(long)Math.Ceiling(reach*PathUtils.Scale)+2;var nearby=new List<int>();
            if(prepared.Index is null)candidates=prepared.Unindexed;
            else if(prepared.Index.QueryBounds(x-radius,y-radius,x+radius,y+radius,nearby,4096,token))
                candidates=nearby.Select(i=>prepared.Edges[i/2]).Distinct().Concat(prepared.Unindexed);
            else BoundedSearches++; // Retain the original finite scan on index saturation.
        }
        var edges=new List<(double Distance,int Edge,double T,double Length)>();
        foreach(int i in candidates){
            ParentProjectionEdgesExamined++;
            if((i&255)==0)token.ThrowIfCancellationRequested();
            var a=node.Body[i];var b=node.Body[(i+1)%node.Body.Length];double x=b.X-a.X,y=b.Y-a.Y,square=x*x+y*y;
            if(square==0)continue;double t=Math.Clamp(((point.X-a.X)*x+(point.Y-a.Y)*y)/square,0,1);
            double dx=a.X+x*t-point.X,dy=a.Y+y*t-point.Y;edges.Add((dx*dx+dy*dy,i,t,Math.Sqrt(square)));
        }
        var result=new HashSet<Location>();
        foreach(var edge in edges.OrderBy(e=>e.Distance).ThenBy(e=>e.Edge).Take(offsets?8:1)){
            var fractions=new List<double>{edge.T};
            if(offsets&&double.IsFinite(reach))foreach(double factor in new[]{.25,.5,1}){
                fractions.Add(edge.T-reach*factor/edge.Length);fractions.Add(edge.T+reach*factor/edge.Length);
            }
            foreach(double t in fractions)if(t>0&&t<1)result.Add(new(edge.Edge,t));
        }
        return result.OrderBy(p=>Math.Pow(At(node,p).X-point.X,2)+Math.Pow(At(node,p).Y-point.Y,2));
    }
    private (Point3D[] Points,double Seconds)? Arc(Node node,Location first,Location last,ToolpathMotionCost cost)
    {
        if(first==last)return(new[]{At(node,first)},0);
        double a=first.Vertex+first.Fraction,b=last.Vertex+last.Fraction;
        Point3D[] Direction(int step)
        {
            double end=b;
            if(node.Closed){if(step>0&&end<a)end+=node.Body.Length;if(step<0&&end>a)end-=node.Body.Length;}
            var result=new List<Point3D>{At(node,first)};
            int at=step>0?(int)Math.Floor(a)+1:(int)Math.Ceiling(a)-1;
            for(;step>0?at<end:at>end;at+=step)result.Add(node.Body[(at%node.Body.Length+node.Body.Length)%node.Body.Length]);
            result.Add(At(node,last));return WithoutLoops(result);
        }
        bool Allowed(Point3D[] points){
            if(!Reversible(points))return false;
            if(first.Fraction==0&&last.Fraction==0)return true;
            if(splitGuard is null)return false;
            return points.Length<2||splitGuard(points[0],points[1])&&splitGuard(points[1],points[0])
                &&splitGuard(points[^2],points[^1])&&splitGuard(points[^1],points[^2]);
        }
        if(!node.Closed){var points=Direction(b>a?1:-1);return Allowed(points)?(points,Seconds(points,cost)):null;}
        var forward=Direction(1);var reverse=Direction(-1);
        double forwardSeconds=Allowed(forward)?Seconds(forward,cost):double.PositiveInfinity;
        double reverseSeconds=Allowed(reverse)?Seconds(reverse,cost):double.PositiveInfinity;
        return !double.IsFinite(Math.Min(forwardSeconds,reverseSeconds))?null:forwardSeconds<=reverseSeconds?(forward,forwardSeconds):(reverse,reverseSeconds);
    }
    private static double Seconds(IReadOnlyList<Point3D> points,ToolpathMotionCost cost)
        =>Enumerable.Range(1,points.Count-1).Sum(i=>cost.ChordSeconds(points[i-1],points[i]));
    // These are optional returns over completed chords, never required source
    // work. Removing an exact XYZ loop joins two old edges at the SAME pose;
    // it cannot introduce an interpolated shortcut or new stock permission.
    private static Point3D[] WithoutLoops(IEnumerable<Point3D> source)
    {
        var result=new List<Point3D>();var positions=new Dictionary<Point3D,int>();
        foreach(var point in source)
        {
            if(positions.TryGetValue(point,out int previous))
            {
                for(int i=result.Count-1;i>previous;i--)positions.Remove(result[i]);
                result.RemoveRange(previous+1,result.Count-previous-1);
            }
            else{positions.Add(point,result.Count);result.Add(point);}
        }
        return result.ToArray();
    }
    private bool Reversible(IReadOnlyList<Point3D> points)
    {
        for(int i=1;i<points.Count;i++)
        {
            var a=points[i-1];var b=points[i];
            if(ToolpathCommandRounding.IsVerticalOnly((float)b.X-(float)a.X,(float)b.Z-(float)a.Z,(float)b.Y-(float)a.Y)
                &&(a.X!=b.X||a.Y!=b.Y))
            {
                // The upper-carrier adapter establishes a real commanded XY
                // anchor. A literal reverse with identical F3 XY words then
                // keeps that cursor; no new source point or stock permission
                // is inferred from the differing binary64 coordinates.
                if(postedEquivalentReturns&&ToolpathCommandRounding.Round((float)a.X)==ToolpathCommandRounding.Round((float)b.X)
                    &&ToolpathCommandRounding.Round((float)a.Y)==ToolpathCommandRounding.Round((float)b.Y))
                {PostedEquivalentSteps++;continue;}
                PartialAxisRefusals++;return false;
            }
        }
        return true;
    }
}
