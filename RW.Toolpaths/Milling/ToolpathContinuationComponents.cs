namespace RW.Toolpaths.Milling;

/// <summary>Finds certified first-layer components before expanding depth passes.
/// Portals are literal source vertices; unknown/bounded searches leave components
/// separate. The same component owns all of its later scheduled depth visits.</summary>
internal static class ToolpathContinuationComponents
{
    internal readonly record struct Portal(int Owner,Point3D Point);
    internal readonly record struct Edge(Portal A,Portal B);
    internal sealed class Plan
    {
        internal readonly IReadOnlyList<Edge> Edges;
        internal readonly int[] VisitOrder;
        internal Plan(IReadOnlyList<Edge> edges,IReadOnlyList<TaggedToolpath> source,CancellationToken token)
        {
            Edges=Array.AsReadOnly(edges.ToArray());VisitOrder=new int[source.Count];
            var neighbours=Enumerable.Range(0,source.Count).Select(_=>new List<int>()).ToArray();
            foreach(var edge in edges){neighbours[edge.A.Owner].Add(edge.B.Owner);neighbours[edge.B.Owner].Add(edge.A.Owner);}
            var used=new bool[source.Count];int rank=0;
            double Length(int owner)=>Enumerable.Range(1,Math.Max(0,source[owner].Points.Count-1))
                .Sum(i=>ToolpathContinuationPlanner.Distance(source[owner].Points[i-1],source[owner].Points[i]));
            foreach(int root in Enumerable.Range(0,source.Count).OrderByDescending(Length).ThenBy(i=>i))
            {
                if(used[root])continue;
                var stack=new Stack<int>();stack.Push(root);
                while(stack.Count>0)
                {
                    token.ThrowIfCancellationRequested();int owner=stack.Pop();if(used[owner])continue;
                    used[owner]=true;VisitOrder[owner]=rank++;
                    foreach(int next in neighbours[owner].OrderByDescending(i=>i))if(!used[next])stack.Push(next);
                }
            }
        }
        internal IEnumerable<(Portal Own,Portal Other)> Entries(int owner)
        {
            foreach(var edge in Edges)
            {
                if(edge.A.Owner==owner)yield return(edge.A,edge.B);
                else if(edge.B.Owner==owner)yield return(edge.B,edge.A);
            }
        }
    }
    internal static List<TaggedToolpath> Assign(IReadOnlyList<TaggedToolpath> source,
        Func<Point3D,Point3D> firstLayer, Func<Point3D,Point3D,bool> allows,
        double distance,CancellationToken token,Action<string>? log=null,Point3D? start=null,
        Func<Point3D,Point3D,bool>? candidateFilter=null,bool closeOpenStrokes=false)
    {
        var parent=Enumerable.Range(0,source.Count).ToArray();
        var grid=new Dictionary<(long,long),List<Portal>>();
        var edges=new List<Edge>();
        int proofs=0,joins=0,cyclePortals=0; bool bounded=false;
        for(int owner=0;owner<source.Count;owner++)
        {
            token.ThrowIfCancellationRequested();
            var portals=new List<Point3D>();
            var seen=new HashSet<(long,long,long)>();
            foreach(var original in source[owner].Points)
            {
                var point=firstLayer(original);
                var key=((long)Math.Floor(point.X/(distance/4)),(long)Math.Floor(point.Y/(distance/4)),
                    (long)Math.Floor(point.Z/(distance/4)));
                if(seen.Add(key))portals.Add(point);
            }
            var candidates=new Dictionary<int,List<(Portal A,Portal B,double Length)>>();
            foreach(var point in portals)
            {
                var cell=Cell(point);
                for(long x=cell.Item1-1;x<=cell.Item1+1;x++)
                for(long y=cell.Item2-1;y<=cell.Item2+1;y++)
                {
                    if(!grid.TryGetValue((x,y),out var found))continue;
                    foreach(var previous in found)
                    {
                        int root=Find(previous.Owner);
                        if(root==Find(owner))continue;
                        double length=ToolpathContinuationPlanner.Distance(previous.Point,point);
                        if(length>distance)continue;
                        // Necessary cheap conditions must be applied BEFORE
                        // retaining the nearest portal candidates. Otherwise a
                        // handful of impossible grades hides a legal doorway.
                        if(candidateFilter is not null && !candidateFilter(previous.Point,point))continue;
                        if(!candidates.TryGetValue(root,out var list))candidates[root]=list=new();
                        list.Add((previous,new Portal(owner,point),length));
                        list.Sort((a,b)=>a.Length.CompareTo(b.Length));
                        if(list.Count>4)list.RemoveAt(4);
                    }
                }
            }
            foreach(var candidate in candidates.OrderBy(p=>p.Value[0].Length).ThenBy(p=>p.Key))
            {
                if(Find(candidate.Key)==Find(owner))continue;
                foreach(var pair in candidate.Value)
                {
                    token.ThrowIfCancellationRequested();
                    if(++proofs>50_000){bounded=true;break;}
                    if(pair.A.Point==pair.B.Point || allows(pair.A.Point,pair.B.Point))
                    {parent[Find(owner)]=Find(candidate.Key);edges.Add(new(pair.A,pair.B));joins++;break;}
                }
                if(bounded)break;
            }
            foreach(var point in portals)
            {
                var cell=Cell(point);
                if(!grid.TryGetValue(cell,out var list))grid[cell]=list=new();
                list.Add(new(owner,point));
            }
        }
        if(closeOpenStrokes&&!bounded)
        {
            var existing=new HashSet<(Portal,Portal)>();
            foreach(var edge in edges){existing.Add((edge.A,edge.B));existing.Add((edge.B,edge.A));}
            for(int owner=0;owner<source.Count&&!bounded;owner++)
            {
                var points=source[owner].Points;
                if(points.Count<2||points[0]==points[^1])continue;
                double length=0;for(int i=1;i<points.Count;i++)length+=ToolpathContinuationPlanner.Distance(points[i-1],points[i]);
                if(length<=1e-9)continue;
                foreach(var end in new[]{points[0],points[^1]})
                {
                    token.ThrowIfCancellationRequested();var point=firstLayer(end);var cell=Cell(point);
                    var candidates=new List<(Portal Other,double Distance)>();var own=new Portal(owner,point);
                    for(long x=cell.Item1-1;x<=cell.Item1+1;x++)
                    for(long y=cell.Item2-1;y<=cell.Item2+1;y++)
                    {
                        if(!grid.TryGetValue((x,y),out var row))continue;
                        foreach(var other in row)
                        {
                            if(other.Owner==owner||Find(other.Owner)!=Find(owner)||other.Point==point
                                ||existing.Contains((own,other))||candidateFilter is not null&&!candidateFilter(point,other.Point))continue;
                            double d=ToolpathContinuationPlanner.Distance(point,other.Point);
                            if(d<=distance)candidates.Add((other,d));
                        }
                    }
                    foreach(var candidate in candidates.OrderBy(c=>c.Distance).ThenBy(c=>c.Other.Owner).Take(8))
                    {
                        if(2*candidate.Distance>=length)continue;
                        if(++proofs>50_000){bounded=true;break;}
                        if(!allows(point,candidate.Other.Point))continue;
                        var other=candidate.Other;edges.Add(new(own,other));existing.Add((own,other));existing.Add((other,own));
                        cyclePortals++;break;
                    }
                    if(bounded)break;
                }
            }
        }
        var ids=new Dictionary<int,int>();
        var plan=new Plan(edges,source,token);
        var result=new List<TaggedToolpath>(source.Count);
        for(int i=0;i<source.Count;i++)
        {
            int root=Find(i);
            if(!ids.TryGetValue(root,out int component))ids[root]=component=ids.Count;
            result.Add(source[i] with{ContinuationComponentIndex=component,ContinuationPlan=plan,ContinuationSourceOrdinal=i});
        }
        // Component-before-depth must not turn arbitrary source IDs into a
        // cross-job travel order. Reuse the shared island tour for these islands.
        var islands=result.GroupBy(p=>p.ContinuationComponentIndex!.Value).OrderBy(g=>g.Key).ToArray();
        var tour=TravelOptimizer.Order(islands.Select(g=>(IReadOnlyList<Clipper2Lib.PointD>)g
            .Where(p=>p.Points.Count>0).Select(p=>new Clipper2Lib.PointD(p.Points[0].X,p.Points[0].Y)).ToArray()).ToArray(),
            start is { } incoming ? new Clipper2Lib.PointD(incoming.X,incoming.Y) : null,
            "continuation-components",token);
        var ranks=new int[tour.Length];for(int i=0;i<tour.Length;i++)ranks[tour[i]]=i;
        for(int i=0;i<result.Count;i++)result[i]=result[i] with{ContinuationComponentIndex=ranks[result[i].ContinuationComponentIndex!.Value]};
        log?.Invoke($"stage=continuation-components event=complete sourceVisits={source.Count} components={ids.Count} "
            +$"certifiedJoins={joins} cyclePortals={cyclePortals} chordProofs={proofs} budgetReached={bounded} "
            +"sourceVerticesOnly=True futureStockCredit=False order=component-before-depth unknownSeparate=True");
        return result;
        int Find(int index){while(parent[index]!=index){parent[index]=parent[parent[index]];index=parent[index];}return index;}
        (long,long) Cell(Point3D p)=>((long)Math.Floor(p.X/distance),(long)Math.Floor(p.Y/distance));
    }
}
