using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Source-level route inspection. Polylines are macro edges, split only
/// at literal portals. Nearby odd endpoints are paired with certified chords;
/// the remaining parity is repaired on a tree, without an emitted-motion search.</summary>
internal static class ToolpathCircuitBuilder
{
    internal sealed record Edge(int A, int B, IReadOnlyList<Point3D> Points, double MaximumDepth, double Length);
    private sealed class Tour
    {
        internal readonly List<(Edge Edge, bool Forward, List<Tour> Children)> Steps = new();
        internal double MaximumDepth;
    }
    internal sealed class Definition
    {
        private readonly Tour tour;
        private readonly Point3D seam;
        private Definition(Tour tour, Point3D seam) { this.tour = tour; this.seam = seam; }
        internal IReadOnlyList<Point3D> AtDepth(double cap, double completedDepth, CancellationToken token)
        {
            var result = new List<Point3D>();
            var work = new Stack<(Tour? Tour, Edge? Edge, bool Forward)>();
            work.Push((tour,null,false));
            while(work.Count>0)
            {
                var frame=work.Pop();
                if(frame.Tour is { } current)
                {
                    if(current.MaximumDepth<=completedDepth+1e-10)continue;
                    var walk=Walk(current,completedDepth);
                    var seen=new HashSet<int>();
                    var emissions=new List<(Edge Edge,bool Forward,List<Tour> Children)>();
                    foreach(var (index,reverse) in walk)
                    {
                        var step=current.Steps[index];
                        int vertex=reverse ? (index+1)%current.Steps.Count : index;
                        emissions.Add((step.Edge,step.Forward != reverse,
                            seen.Add(vertex) ? current.Steps[vertex].Children : new()));
                    }
                    for(int i=emissions.Count-1;i>=0;i--)
                    {
                        var emission=emissions[i];work.Push((null,emission.Edge,emission.Forward));
                        for(int j=emission.Children.Count-1;j>=0;j--)work.Push((emission.Children[j],null,false));
                    }
                    if(walk.Count==0)
                        for(int j=current.Steps[0].Children.Count-1;j>=0;j--)work.Push((current.Steps[0].Children[j],null,false));
                    continue;
                }
                var row=frame.Edge!.Points;
                for(int i=0;i<row.Count;i++)
                {
                    if((result.Count&1023)==0)token.ThrowIfCancellationRequested();
                    var p=row[frame.Forward ? i : row.Count-1-i];
                    p=p with{Z=Math.Max(p.Z,-cap)};
                    if(result.Count==0 || result[^1]!=p)result.Add(p);
                }
                if(result.Count>2_000_000)throw new InvalidOperationException("A source circuit exceeds two million vertices; split the operation.");
            }
            if(result.Count==0)result.Add(seam with{Z=Math.Max(seam.Z,-cap)});
            if(result[0]!=result[^1])throw new InvalidOperationException("Source route inspection lost its closing seam.");
            return result;
        }

        // If this carrier is finished, it is now only a route to unfinished
        // child excursions. Remove its largest unused arc when out-and-back
        // travel over the remainder is shorter than another whole circuit.
        private static List<(int Index,bool Reverse)> Walk(Tour current,double completed)
        {
            int n=current.Steps.Count;
            var full=Enumerable.Range(0,n).Select(i=>(i,false)).ToList();
            if(current.Steps.Any(s=>s.Edge.MaximumDepth>completed+1e-10))return full;
            var stops=new List<int>{0};
            for(int i=1;i<n;i++)
                if(current.Steps[i].Children.Any(c=>c.MaximumDepth>completed+1e-10))stops.Add(i);
            if(stops.Count==1)return new();
            var prefix=new double[n+1];
            for(int i=0;i<n;i++)prefix[i+1]=prefix[i]+current.Steps[i].Edge.Length;
            double gap=-1;int lo=0,hi=n;
            for(int i=0;i<stops.Count;i++)
            {
                int a=stops[i],b=i+1==stops.Count ? n : stops[i+1];
                double length=prefix[b]-prefix[a];
                if(length>gap){gap=length;lo=a;hi=b;}
            }
            if(2*(prefix[n]-gap)>=prefix[n])return full;
            var walk=new List<(int,bool)>();
            for(int i=0;i<lo;i++)walk.Add((i,false));
            for(int i=lo-1;i>=0;i--)walk.Add((i,true));
            for(int i=n-1;i>=hi;i--)walk.Add((i,true));
            for(int i=hi;i<n;i++)walk.Add((i,false));
            return walk;
        }
        internal static Definition FromGraph(IReadOnlyList<Point3D> nodes,List<Edge> edges,
            List<int>[] adjacency,int root,CancellationToken token)
        {
            var consumed=new bool[edges.Count];var cursor=new int[nodes.Count];
            int Next(int node)
            {
                var row=adjacency[node];
                while(cursor[node]<row.Count && consumed[row[cursor[node]]])cursor[node]++;
                return cursor[node]<row.Count ? row[cursor[node]++] : -1;
            }
            Tour Cycle(int start)
            {
                var cycle=new Tour();int node=start;
                do
                {
                    token.ThrowIfCancellationRequested();int edgeId=Next(node);
                    if(edgeId<0)throw new InvalidOperationException("Source circuit has unpaired endpoint parity.");
                    consumed[edgeId]=true;var edge=edges[edgeId];bool forward=edge.A==node;
                    cycle.Steps.Add((edge,forward,new()));cycle.MaximumDepth=Math.Max(cycle.MaximumDepth,edge.MaximumDepth);
                    node=forward ? edge.B : edge.A;
                }while(node!=start);
                return cycle;
            }
            var initial=Cycle(root);
            var stack=new Stack<(Tour Tour,int Position)>();stack.Push((initial,0));
            while(stack.Count>0)
            {
                var (cycle,position)=stack.Pop();
                if(position==cycle.Steps.Count)continue;
                var step=cycle.Steps[position];int node=step.Forward ? step.Edge.A : step.Edge.B;
                int next=Next(node);
                if(next>=0)
                {
                    cursor[node]--;var child=Cycle(node);step.Children.Add(child);
                    stack.Push((cycle,position));stack.Push((child,0));
                }
                else stack.Push((cycle,position+1));
            }
            // Completed closed excursions can vanish from later passes. Every
            // surviving XY chord was physically executed at the preceding cap.
            var post=new Stack<(Tour Tour,bool Exit)>();post.Push((initial,false));
            while(post.Count>0)
            {
                var (cycle,exit)=post.Pop();
                if(exit)
                {
                    foreach(var step in cycle.Steps)
                    foreach(var child in step.Children)cycle.MaximumDepth=Math.Max(cycle.MaximumDepth,child.MaximumDepth);
                }
                else
                {
                    post.Push((cycle,true));
                    foreach(var step in cycle.Steps)
                    foreach(var child in step.Children)post.Push((child,false));
                }
            }
            if(consumed.Any(c=>!c))throw new InvalidOperationException("Source route inspection omitted a required chord component.");
            return new(initial,nodes[root]);
        }
    }

    internal static List<TaggedToolpath> Build(IReadOnlyList<TaggedToolpath> source,
        Func<Point3D,Point3D,bool> allowsConnector,double proximity,CancellationToken token,
        Action<string>? log=null,Point3D? start=null,double maximumGrade=double.PositiveInfinity)
    {
        long started=Stopwatch.GetTimestamp();
        bool Candidate(Point3D a,Point3D b)=>a==b || Math.Abs(a.Z-b.Z)<=maximumGrade*
            Math.Sqrt((a.X-b.X)*(a.X-b.X)+(a.Y-b.Y)*(a.Y-b.Y));
        var assigned=ToolpathContinuationComponents.Assign(source,p=>p,allowsConnector,proximity,token,log,start,Candidate,
            closeOpenStrokes:true);
        if(assigned.Count==0)return new();
        var plan=assigned[0].ContinuationPlan!;
        var vertexMaps=new Dictionary<Point3D,int>?[source.Count];
        void Need(ToolpathContinuationComponents.Portal portal)
        {
            var map=vertexMaps[portal.Owner]??=new();map.TryAdd(portal.Point,-1);
        }
        foreach(var edge in plan.Edges){Need(edge.A);Need(edge.B);}
        for(int owner=0;owner<source.Count;owner++)
        {
            if(vertexMaps[owner] is not { } map)continue;
            int missing=map.Count;var points=source[owner].Points;
            for(int i=0;i<points.Count&&missing>0;i++)
                if(map.TryGetValue(points[i],out int at)&&at<0){map[points[i]]=i;missing--;}
            if(missing!=0)throw new InvalidOperationException("A certified circuit portal has no literal source vertex.");
        }
        var portals=source.Select(path=>new SortedSet<int>{0,Math.Max(0,path.Points.Count-1)}).ToArray();
        foreach(var edge in plan.Edges)
        {portals[edge.A.Owner].Add(vertexMaps[edge.A.Owner]![edge.A.Point]);portals[edge.B.Owner].Add(vertexMaps[edge.B.Owner]![edge.B.Point]);}
        var componentEdges=plan.Edges.GroupBy(e=>assigned[e.A.Owner].ContinuationComponentIndex)
            .ToDictionary(g=>g.Key!.Value,g=>g.ToArray());
        var result=new List<TaggedToolpath>();long macroEdges=0,paired=0,duplicates=0,routeChords=0;int proofs=0;
        double requiredLength=0,duplicateLength=0;
        foreach(var component in assigned.Select((path,owner)=>(path,owner))
            .GroupBy(p=>p.path.ContinuationComponentIndex).OrderBy(g=>g.Key))
        {
            var owners=component.Select(p=>p.owner).ToHashSet();
            var nodes=new List<Point3D>();var ids=new Dictionary<Point3D,int>();var edges=new List<Edge>();
            var adjacency=new List<List<int>>();
            int Node(Point3D p)
            {if(!ids.TryGetValue(p,out int id)){ids[p]=id=nodes.Count;nodes.Add(p);adjacency.Add(new());}return id;}
            void Add(IReadOnlyList<Point3D> points)
            {
                if(points.Count<2)return;
                int a=Node(points[0]),b=Node(points[^1]);int id=edges.Count;
                double length=0;for(int i=1;i<points.Count;i++)length+=ToolpathContinuationPlanner.Distance(points[i-1],points[i]);
                edges.Add(new(a,b,points,points.Max(p=>-p.Z),length));adjacency[a].Add(id);adjacency[b].Add(id);
            }
            foreach(int owner in owners.Order())
            {
                int previous=0;
                foreach(int index in portals[owner])
                {
                    if(index>previous)Add(source[owner].Points.Skip(previous).Take(index-previous+1).ToArray());
                    previous=index;
                }
            }
            if(componentEdges.TryGetValue(component.Key!.Value,out var connections))
                foreach(var edge in connections)Add(new[]{edge.A.Point,edge.B.Point});
            if(edges.Count==0)continue;
            macroEdges+=edges.Count;
            requiredLength+=edges.Sum(e=>e.Length);
            var odd=Enumerable.Range(0,nodes.Count).Where(i=>(adjacency[i].Count&1)!=0).ToHashSet();
            var grid=new Dictionary<(long,long),List<int>>();
            (long,long) Cell(Point3D p)=>((long)Math.Floor(p.X/proximity),(long)Math.Floor(p.Y/proximity));
            foreach(int node in odd)
            {var key=Cell(nodes[node]);if(!grid.TryGetValue(key,out var row))grid[key]=row=new();row.Add(node);}
            foreach(int node in odd.Order().ToArray())
            {
                token.ThrowIfCancellationRequested();if(!odd.Contains(node))continue;
                var cell=Cell(nodes[node]);var candidates=new List<(int Node,double Distance)>();
                for(long x=cell.Item1-1;x<=cell.Item1+1;x++)
                for(long y=cell.Item2-1;y<=cell.Item2+1;y++)
                {
                    if(!grid.TryGetValue((x,y),out var row))continue;
                    foreach(int other in row)
                    {
                        if(other==node||!odd.Contains(other)||!Candidate(nodes[node],nodes[other]))continue;
                        double d=ToolpathContinuationPlanner.Distance(nodes[node],nodes[other]);
                        if(d<=proximity)candidates.Add((other,d));
                    }
                }
                foreach(var candidate in candidates.OrderBy(c=>c.Distance).ThenBy(c=>c.Node).Take(8))
                {
                    if(++proofs>50_000)break;
                    if(!allowsConnector(nodes[node],nodes[candidate.Node]))continue;
                    Add(new[]{nodes[node],nodes[candidate.Node]});odd.Remove(node);odd.Remove(candidate.Node);paired++;break;
                }
            }
            // Repair parity on a MINIMUM spanning tree of the source macro
            // graph. An arbitrary DFS tree can choose a long cutting stroke
            // where a short proved connection completes the same cycle.
            int root=Node(source[component.OrderBy(p=>plan.VisitOrder[p.owner]).First().owner].Points[0]);
            var parent=Enumerable.Repeat(-1,nodes.Count).ToArray();var parentEdge=new int[nodes.Count];
            var order=new List<int>();var frontier=new PriorityQueue<(int Node,int Parent,int Edge),double>();
            frontier.Enqueue((root,root,-1),0);
            while(frontier.TryDequeue(out var next,out _))
            {
                int node=next.Node;if(parent[node]>=0)continue;
                parent[node]=next.Parent;parentEdge[node]=next.Edge;order.Add(node);
                foreach(int e in adjacency[node])
                {
                    var edge=edges[e];int other=edge.A==node?edge.B:edge.A;
                    if(parent[other]<0)frontier.Enqueue((other,node,e),edge.Length);
                }
            }
            for(int i=order.Count-1;i>0;i--)
            {
                int node=order[i];if(!odd.Contains(node))continue;
                var duplicate=edges[parentEdge[node]];Add(duplicate.Points);duplicates++;duplicateLength+=duplicate.Length;
                odd.Remove(node);if(!odd.Add(parent[node]))odd.Remove(parent[node]);
            }
            if(odd.Count>0)throw new InvalidOperationException("Source parity repair did not terminate at its component root.");
            var definition=Definition.FromGraph(nodes,edges,adjacency.ToArray(),root,token);
            var points=definition.AtDepth(double.PositiveInfinity,-1,token);
            routeChords+=points.Count-1;
            var representative=source[component.First().owner];
            string category=component.Any(p=>p.path.Category=="final-carve")?"final-carve":representative.Category;
            result.Add(new TaggedToolpath(points,Array.Empty<ToolpathSpan>(),representative.RegionIndex,category,null,detectCurves:false)
                {ContinuationComponentIndex=component.Key,ContinuationCircuit=true,ContinuationCircuitDefinition=definition});
        }
        log?.Invoke($"stage=source-circuits event=complete sourceVisits={source.Count} circuits={result.Count} "
            +$"certifiedConnectors={plan.Edges.Count} macroEdges={macroEdges} pairedEndpoints={paired} parityDuplicates={duplicates} "
            +$"pairingProofs={proofs} routeChords={routeChords} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} "
            +$"requiredLength={requiredLength:R} duplicateLength={duplicateLength:R} parityTree=minimum-spanning "
            +"sourceChordGeometryRetained=True completedExcursionsPrunedBeforeDepthEmission=True emittedGraphSearch=False");
        return result;
    }
}
