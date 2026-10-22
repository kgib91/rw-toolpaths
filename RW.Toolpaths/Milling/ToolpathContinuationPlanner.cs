namespace RW.Toolpaths.Milling;

/// <summary>
/// The common execution policy: continue directly, retrace executed motion, then
/// request re-entry with a reason. Geometry adapters alone authorize NEW chords.
/// A ring, layer or source-visit boundary is not a reason to retract.
/// </summary>
internal sealed class ToolpathContinuationPlanner
{
    internal sealed record Connection(IReadOnlyList<Point3D> Points, int Target, string Kind,
        IReadOnlyList<bool>? ClearedTravel = null);
    private sealed class Node
    {
        internal Node(Point3D point) { Point = point; }
        internal readonly Point3D Point;
        internal readonly List<int> Neighbours = new();
    }
    private readonly List<Node> nodes = new();
    private readonly Dictionary<Point3D, int> ordinals = new();
    private readonly Dictionary<(double X, double Y), List<int>> columns = new();
    private readonly Dictionary<(long X, long Y), List<int>> grid = new();
    private readonly HashSet<(long X, long Y, long Z)> portalVoxels = new();
    private readonly HashSet<(int,int)> clearedEdges=new();
    private readonly Dictionary<(double,double,double,double),List<(Point3D A,Point3D B)>> executedProjections=new();
    private readonly double neighbourhood;
    private readonly CancellationToken token;
    private readonly Action<string>? diagnostic;
    private readonly string owner;
    private const int MaximumNodes = 2_000_000, MaximumSearch = 100_000;
    private const long MaximumTotalSearch = 20_000_000;
    private long direct, retraces, entries, examined, recordRefusals, boundedSearches, visibilityTests, priorAirEdges;
    private readonly Dictionary<string, long> reasons = new();
    internal string LastFailure {get;private set;}="no-certified-route";

    internal ToolpathContinuationPlanner(string owner, double neighbourhood,
        CancellationToken token, Action<string>? diagnostic = null)
    {
        if (!double.IsFinite(neighbourhood) || neighbourhood<=0) throw new ArgumentOutOfRangeException(nameof(neighbourhood));
        this.owner = owner; this.neighbourhood = Math.Max(.01, neighbourhood);
        this.token = token; this.diagnostic = diagnostic;
    }

    /// <summary>Only literal executed endpoints and chords enter the graph.
    /// No future paths, safe-height bridges, nominal stock or XY-only merges.</summary>
    internal bool Record(TaggedToolpath path, Func<Point3D, Point3D, bool>? allowRetrace = null)
    {
        bool anchored=true;
        var spans=path.Spans.Count>0 ? path.Spans : new[] {new ToolpathSpan(0,path.Points.Count-1,ToolpathSpanKind.Cut)};
        foreach (var span in spans)
        {
            for (int i = span.StartIndex + 1; i <= span.EndIndex; i++)
            {
                if ((i & 1023) == 0) token.ThrowIfCancellationRequested();
                var a = path.Points[i - 1]; var b = path.Points[i];
                float dx=(float)b.X-(float)a.X,dy=(float)b.Y-(float)a.Y,dz=(float)b.Z-(float)a.Z;
                bool yOnly=ToolpathCommandRounding.IsVerticalOnly(dx,dz,dy);
                bool partial=yOnly && (a.X!=b.X || a.Y!=b.Y) && span.Kind!=ToolpathSpanKind.TabLift;
                bool valid=anchored && !partial;
                if(partial)anchored=false;
                else if(!yOnly || span.Kind==ToolpathSpanKind.TabLift)anchored=true;
                if (!valid || span.Kind is not (ToolpathSpanKind.Cut or ToolpathSpanKind.Ramp or ToolpathSpanKind.Link or ToolpathSpanKind.ClearedTravel)
                    || !Finite(a) || !Finite(b) || allowRetrace is not null && !allowRetrace(a, b)) continue;
                int first = NodeAt(a), last = NodeAt(b);
                if (first < 0 || last < 0 || first == last) continue;
                // Repeated entry laps can contain the same physical chord many times.
                if (!nodes[first].Neighbours.Contains(last)) nodes[first].Neighbours.Add(last);
                if (!nodes[last].Neighbours.Contains(first)) nodes[last].Neighbours.Add(first);
                AddProjection(a,b);AddProjection(b,a);
                if(span.Kind==ToolpathSpanKind.ClearedTravel)clearedEdges.Add((Math.Min(first,last),Math.Max(first,last)));
            }
        }
        return anchored;
    }

    // A geometry adapter may deepen an already executed complete XY chord by
    // one physical step. Both endpoints must be dominated by ONE earlier chord
    // in native and posted space; independent endpoint minima are not evidence.
    internal bool HasStepdownPrecursor(Point3D a,Point3D b,double step,
        Func<Point3D,Point3D> command)
    {
        if(!executedProjections.TryGetValue((a.X,a.Y,b.X,b.Y),out var prior))return false;
        var ca=command(a);var cb=command(b);
        foreach(var edge in prior)
        {
            var pa=command(edge.A);var pb=command(edge.B);
            if(a.Z>=edge.A.Z-step && b.Z>=edge.B.Z-step
                && ca.X==pa.X && ca.Y==pa.Y && cb.X==pb.X && cb.Y==pb.Y
                && ca.Z>=pa.Z-step-1e-10 && cb.Z>=pb.Z-step-1e-10)return true;
        }
        return false;
    }

    private void AddProjection(Point3D a,Point3D b)
    {
        var key=(a.X,a.Y,b.X,b.Y);
        if(!executedProjections.TryGetValue(key,out var prior))executedProjections[key]=prior=new();
        if(prior.Any(p=>p.A.Z<=a.Z && p.B.Z<=b.Z))return;
        prior.RemoveAll(p=>a.Z<=p.A.Z && b.Z<=p.B.Z);
        // Bounded retention can discard authority, never manufacture it.
        if(prior.Count<16)prior.Add((a,b));
    }

    internal void RecordProvedAir(Point3D a, Point3D b, Func<Point3D,Point3D,bool> wholeVolumeProof,
        bool positiveTravelClearance=false)
    {
        token.ThrowIfCancellationRequested();
        if(!Finite(a) || !Finite(b))return;
        if (!wholeVolumeProof(a,b)) return;
        Record(new TaggedToolpath(new[] {a,b}, new[] {new ToolpathSpan(0,1,
            positiveTravelClearance ? ToolpathSpanKind.ClearedTravel : ToolpathSpanKind.Link)},
            0,"prior-proved-air",null,detectCurves:false));
        priorAirEdges++;
    }

    internal IReadOnlyList<Point3D> ClearedEntries(Point3D source, double surface,
        bool upwardProfileIsContained)
    {
        if (!upwardProfileIsContained || !columns.TryGetValue((source.X, source.Y), out var found)) return Array.Empty<Point3D>();
        // An earlier identical XY pose owns the complete cutter, not just its tip.
        // Raising is permitted only for a physically monotone profile and a caller
        // with unchanged placement/tool. Return literal Zs to preserve native and
        // posted endpoint identity; never snap nearby native points together.
        return found.Select(i => nodes[i].Point).Where(p => p.Z >= source.Z && p.Z < surface)
            .Distinct().OrderBy(p => p.Z).Take(12).ToArray();
    }

    internal Connection? Connect(Point3D from, IReadOnlyList<Point3D> targets,
        Func<Point3D, Point3D, bool> allowsNewChord,
        Func<Point3D,Point3D,bool>? positiveTravelClearance = null,double clearedTravelCostRatio=1,
        IReadOnlyList<double>? targetCosts=null,IReadOnlyList<Point3D>? preferredEntryPortals=null,
        double maximumCost=double.PositiveInfinity)
    {
        token.ThrowIfCancellationRequested();
        LastFailure="no-certified-route";
        if(!Finite(from) || targets.Count==0 || targets.Any(p=>!Finite(p)))
        {LastFailure="unsupported-command-domain";return null;}
        double ratio=positiveTravelClearance is null ? 1 : Math.Clamp(clearedTravelCostRatio,.001,1);
        var travelProofs=new Dictionary<(Point3D,Point3D),bool>();
        Connection? directFallback=null;double directCost=double.PositiveInfinity;
        for (int i = 0; i < targets.Count; i++)
        {
            if (from == targets[i] && TargetCost(i)<=maximumCost)
            { direct++; return Finish(new[] {from},i,"coincident"); }
            if (allowsNewChord(from, targets[i]))
            {
                var candidate=Finish(new[] {from,targets[i]},i,"certified-direct");
                double cost=Cost(from,targets[i])+TargetCost(i);
                if(cost>maximumCost)continue;
                if(ratio==1 || Fast(from,targets[i])){direct++;return candidate;}
                if(cost<directCost){directCost=cost;directFallback=candidate;}
            }
        }
        if (!ordinals.TryGetValue(from, out int start))
        {LastFailure=recordRefusals>0 ? "execution-graph-work-limit" : "exit-not-in-proved-network";return Fallback();}
        var goals = new Dictionary<int, (int Target, double Extra)>();
        for (int target = 0; target < targets.Count; target++)
        {
            var point = targets[target];
            if (ordinals.TryGetValue(point, out int exact)) AddGoal(exact, target, 0);
            if(preferredEntryPortals is not null)
            foreach(var portal in preferredEntryPortals)
                if(ordinals.TryGetValue(portal,out int node) && allowsNewChord(portal,point))
                    AddGoal(node,target,Cost(portal,point));
            var cell = Cell(point);
            var nearby = new List<(int Node, double Distance)>();
            for (long x = cell.X - 1; x <= cell.X + 1; x++)
            for (long y = cell.Y - 1; y <= cell.Y + 1; y++)
            {
                if (!grid.TryGetValue((x, y), out var list)) continue;
                foreach (int node in list)
                {
                    double distance = Distance(nodes[node].Point, point);
                    if (distance <= neighbourhood) nearby.Add((node, distance));
                }
            }
            // Bounds affect search only. Missing proof keeps the original entry.
            foreach (var proposal in nearby.OrderBy(p => p.Distance).ThenBy(p => p.Node).Take(48))
            {
                token.ThrowIfCancellationRequested();
                if (proposal.Distance > 0 && allowsNewChord(nodes[proposal.Node].Point, point))
                    AddGoal(proposal.Node,target,Cost(nodes[proposal.Node].Point,point));
            }
        }
        if (goals.Count == 0 || examined >= MaximumTotalSearch)
        {LastFailure=examined>=MaximumTotalSearch ? "search-work-limit" : "no-proved-entry-connection";return Fallback();}
        var costs = new Dictionary<int, double> { [start] = 0 };
        var parents = new Dictionary<int, int>();
        var visibility = new Dictionary<(int,int),bool>();
        var queue = new PriorityQueue<int, (double Cost, int Ordinal)>();
        queue.Enqueue(start, (Heuristic(start), start));
        int bestNode = -1, visited = 0;
        double bestCost = Math.Min(directCost,maximumCost+1e-9);
        while (queue.TryDequeue(out int node, out var priority))
        {
            token.ThrowIfCancellationRequested();
            double cost = costs[node];
            if (priority.Cost > cost + Heuristic(node) + 1e-10) continue;
            if (priority.Cost >= bestCost) break;
            if (++visited > MaximumSearch || ++examined > MaximumTotalSearch)
            { boundedSearches++; LastFailure="search-work-limit"; break; }
            if (goals.TryGetValue(node, out var goal) && cost + goal.Extra < bestCost)
            { bestCost = cost + goal.Extra; bestNode = node; }
            foreach (int next in nodes[node].Neighbours)
            {
                Relax(next);
            }
            // A blocked straight line need not force a long walk around an old
            // contour. Discover short certified visibility edges between native
            // graph portals. Search trials never become cleared-stock witnesses.
            if (visibilityTests < 100_000)
            {
                var cell = Cell(nodes[node].Point);
                var proposals = new List<(int Node,double Estimate)>();
                for (long x=cell.X-1;x<=cell.X+1;x++)
                for (long y=cell.Y-1;y<=cell.Y+1;y++)
                {
                    if (!grid.TryGetValue((x,y),out var list)) continue;
                    foreach (int next in list)
                    {
                        double length = Distance(nodes[node].Point,nodes[next].Point);
                        if (length < .25 || length > neighbourhood || nodes[node].Neighbours.Contains(next)) continue;
                        double value=cost+length*ratio;
                        if (costs.TryGetValue(next,out double old) && old<=value) continue;
                        proposals.Add((next,length+Heuristic(next)));
                    }
                }
                foreach (var proposal in proposals.OrderBy(p=>p.Estimate).ThenBy(p=>p.Node).Take(8))
                {
                    var key=(Math.Min(node,proposal.Node),Math.Max(node,proposal.Node));
                    if (!visibility.TryGetValue(key,out bool allowed))
                    {
                        visibilityTests++;
                        allowed=allowsNewChord(nodes[node].Point,nodes[proposal.Node].Point);
                        visibility[key]=allowed;
                    }
                    if (allowed) Relax(proposal.Node);
                }
            }
            void Relax(int next)
            {
                double value=cost+Cost(nodes[node].Point,nodes[next].Point);
                if (costs.TryGetValue(next,out double old) && value>=old) return;
                costs[next]=value; parents[next]=node;
                queue.Enqueue(next,(value+Heuristic(next),next));
            }
        }
        if (bestNode < 0)
        {if(LastFailure!="search-work-limit")LastFailure=double.IsFinite(maximumCost)
            ? "no-proved-route-within-replaced-entry-cost" : "captured-network-disconnected";return Fallback();}
        var walk = new List<Point3D>();
        for (int node = bestNode; ; node = parents[node])
        { walk.Add(nodes[node].Point); if (node == start) break; }
        walk.Reverse();
        int selected = goals[bestNode].Target;
        if (walk[^1] != targets[selected]) walk.Add(targets[selected]);
        retraces++;
        return Finish(walk,selected,"executed-native-retrace");

        Connection Finish(IReadOnlyList<Point3D> points,int target,string kind)
        {
            bool[]? travel=null;
            if(positiveTravelClearance is not null)
            {
                travel=new bool[Math.Max(0,points.Count-1)];
                for(int i=0;i<travel.Length;i++)travel[i]=Fast(points[i],points[i+1]);
            }
            return new(points,target,kind,travel);
        }

        bool Fast(Point3D a,Point3D b)
        {
            if(travelProofs.TryGetValue((a,b),out bool value))return value;
            bool known=ordinals.TryGetValue(a,out int x)&&ordinals.TryGetValue(b,out int y)
                &&clearedEdges.Contains((Math.Min(x,y),Math.Max(x,y)));
            value=known || positiveTravelClearance?.Invoke(a,b)==true;
            travelProofs[(a,b)]=value;travelProofs[(b,a)]=value;
            return value;
        }
        double Cost(Point3D a,Point3D b)=>Distance(a,b)*(ratio<1 && Fast(a,b) ? ratio : 1);
        Connection? Fallback(){if(directFallback is not null)direct++;return directFallback;}

        void AddGoal(int node, int target, double extra)
        {
            double total=extra+TargetCost(target);
            if (!goals.TryGetValue(node, out var old) || total < old.Extra)
                goals[node] = (target, total);
        }
        double TargetCost(int target)=>targetCosts is null ? 0 : targetCosts[target];
        double Heuristic(int node) => targets.Select((p,i)=>ratio*Distance(nodes[node].Point,p)+TargetCost(i)).Min();
    }

    internal void Reentry(string reason, Point3D? from, Point3D target)
    {
        entries++; reasons[reason] = reasons.GetValueOrDefault(reason) + 1;
        if (entries <= 8)
            diagnostic?.Invoke($"stage=continuation event=reentry owner={owner} reason={reason} "
                + $"from={from} target={target} physicalNecessityProved=False");
    }

    internal void Report() => diagnostic?.Invoke($"stage=continuation event=complete owner={owner} "
        + $"direct={direct} retraces={retraces} entries={entries} nodes={nodes.Count} "
        + $"searchNodes={examined} searchBudgetRefusals={boundedSearches} recordBudgetRefusals={recordRefusals} "
        + $"visibilityProofs={visibilityTests} priorProvedAirEdges={priorAirEdges} "
        + $"reasons={string.Join(',', reasons.OrderBy(p => p.Key).Select(p => p.Key + ':' + p.Value))} "
        + "policy=direct-then-executed-retrace-then-reasoned-entry futureStockCredit=False nativeXYZIdentity=True");

    /// <summary>Common final routing for generators that already nominate entries.
    /// Only a proved redundant Ramp prefix may be removed; every Cut/Tab survives.
    /// New-cut adapters must prove whole native/posted clearance and engagement.</summary>
    internal static List<TaggedToolpath> Reconnect(IReadOnlyList<TaggedToolpath> source,
        string owner, double neighbourhood, CancellationToken token,
        Func<TaggedToolpath, Point3D, Point3D, bool>? allowsNewChord = null,
        bool removeSubsumedEntries = false, Action<string>? diagnostic = null,
        Action<TaggedToolpath>? recordExecuted = null,
        Func<Point3D, Point3D, bool>? validateConnection = null)
    {
        diagnostic ??= Console.WriteLine;
        var planner = new ToolpathContinuationPlanner(owner, neighbourhood, token, diagnostic);
        var result = new List<TaggedToolpath>(source.Count);
        Point3D? previous = null; int? previousRegion = null; bool previousAnchored=true;
        foreach (var original in source)
        {
            token.ThrowIfCancellationRequested();
            if (original.Points.Count < 2) { result.Add(original); continue; }
            var path = original;
            if (previous is { } from && from != path.Points[0] && previousRegion == path.RegionIndex && previousAnchored)
            {
                var targets = new List<Point3D>();
                int cut = removeSubsumedEntries ? SubsumedEntryEnd(path) : 0;
                if (cut > 0) targets.Add(path.Points[cut]);
                targets.Add(path.Points[0]);
                var connection = planner.Connect(from, targets,
                    (a, b) => allowsNewChord?.Invoke(path, a, b) == true);
                bool rejectedConnection = false;
                if (connection is not null && validateConnection is not null)
                for (int i=1;i<connection.Points.Count;i++)
                {
                    if (validateConnection(connection.Points[i-1],connection.Points[i])) continue;
                    diagnostic($"[Toolpath.Continuation] owner={owner} event=connection-guard-refused "
                        + $"a={connection.Points[i-1]} b={connection.Points[i]} sourceEntryRetained=True");
                    connection=null; rejectedConnection=true; break;
                }
                if (connection is not null)
                    path = Prepend(path, connection.Points, connection.Target == 0 && cut > 0 ? cut : 0);
                else planner.Reentry(rejectedConnection ? "connection-guard-refused" : planner.LastFailure, from, path.Points[0]);
            }
            else if (previous is null || previous != path.Points[0])
                planner.Reentry(previous is null ? "first-entry" : "source-component-change", previous, path.Points[0]);
            result.Add(path); previousAnchored=planner.Record(path); recordExecuted?.Invoke(path);
            previous = path.Points[^1]; previousRegion = path.RegionIndex;
        }
        planner.Report();
        return result;
    }

    internal static TaggedToolpath Prepend(TaggedToolpath path, IReadOnlyList<Point3D> connection,
        int sourceStart = 0,IReadOnlyList<bool>? clearedTravel = null)
    {
        if (connection.Count == 0 || connection[^1] != path.Points[sourceStart])
            throw new InvalidOperationException("A certified continuation must meet its exact native source endpoint.");
        var points = connection.Take(connection.Count - 1).Concat(path.Points.Skip(sourceStart)).ToArray();
        int offset = connection.Count - 1;
        var spans = new List<ToolpathSpan>();
        if (offset > 0)
        {
            int start=0;
            for(int i=1;i<=offset;i++)
            {
                bool travel=clearedTravel?[start]==true;
                if(i<offset && (clearedTravel?[i]==true)==travel)continue;
                spans.Add(new(start,i,travel ? ToolpathSpanKind.ClearedTravel : ToolpathSpanKind.Link));
                start=i;
            }
        }
        var sourceSpans=path.Spans.Count>0 ? path.Spans : new[] {new ToolpathSpan(0,path.Points.Count-1,ToolpathSpanKind.Cut)};
        foreach (var span in sourceSpans)
        {
            int first = Math.Max(sourceStart, span.StartIndex);
            if (span.EndIndex > first)
                spans.Add(new(first - sourceStart + offset, span.EndIndex - sourceStart + offset, span.Kind));
        }
        return new(points, spans, path.RegionIndex, path.Category, path.DepthPassIndex,
            path.VisitIdentity, path.DetectCurves) {ContinuationComponentIndex=path.ContinuationComponentIndex,
                ContinuationPlan=path.ContinuationPlan,ContinuationSourceOrdinal=path.ContinuationSourceOrdinal};
    }

    private static int SubsumedEntryEnd(TaggedToolpath path)
    {
        int cut = 0;
        foreach (var span in path.Spans)
        {
            if (span.Kind != ToolpathSpanKind.Ramp || span.StartIndex != cut) break;
            cut = span.EndIndex;
        }
        if (cut == 0 || cut >= path.Points.Count - 1) return 0;
        var floors = new Dictionary<(double, double, double, double), (double A, double B)>();
        foreach (var span in path.Spans.Where(s => s.Kind == ToolpathSpanKind.Cut))
        for (int i = span.StartIndex + 1; i <= span.EndIndex; i++)
        {
            var a = path.Points[i - 1]; var b = path.Points[i];
            Add(a, b); Add(b, a);
        }
        for (int i = 1; i <= cut; i++)
        {
            var a = path.Points[i - 1]; var b = path.Points[i];
            if (!floors.TryGetValue((a.X, a.Y, b.X, b.Y), out var floor)
                || floor.A > a.Z || floor.B > b.Z) return 0;
        }
        return cut;
        void Add(Point3D a, Point3D b)
        {
            var key = (a.X, a.Y, b.X, b.Y);
            if (!floors.TryGetValue(key, out var old) || a.Z <= old.A && b.Z <= old.B)
                floors[key] = (a.Z, b.Z);
        }
    }

    private int NodeAt(Point3D point)
    {
        if (ordinals.TryGetValue(point, out int node)) return node;
        if (nodes.Count >= MaximumNodes) { recordRefusals++; return -1; }
        node = nodes.Count; nodes.Add(new(point)); ordinals.Add(point, node);
        var key = (point.X, point.Y);
        if (!columns.TryGetValue(key, out var list)) columns[key] = list = new();
        list.Add(node);
        // Portal sampling bounds nomination cost only; the exact execution graph
        // still retains every recorded endpoint and chord as the authority.
        var voxel=((long)Math.Floor(point.X/.5),(long)Math.Floor(point.Y/.5),(long)Math.Floor(point.Z/.5));
        if (portalVoxels.Add(voxel))
        {
            var cell = Cell(point);
            if (!grid.TryGetValue(cell, out list)) grid[cell] = list = new();
            list.Add(node);
        }
        return node;
    }
    private (long X, long Y) Cell(Point3D p) => ((long)Math.Floor(p.X / neighbourhood), (long)Math.Floor(p.Y / neighbourhood));
    private static bool Finite(Point3D p) => double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z)
        && Math.Abs(p.X)<=1e12 && Math.Abs(p.Y)<=1e12 && Math.Abs(p.Z)<=1e12;
    internal static double Distance(Point3D a, Point3D b) => Math.Sqrt((a.X-b.X)*(a.X-b.X) + (a.Y-b.Y)*(a.Y-b.Y) + (a.Z-b.Z)*(a.Z-b.Z));

    internal static double SafeHeightTransferCost(Point3D from,Point3D to,double safeZ,double cutToRapidRatio)
    {
        if(!double.IsFinite(safeZ)||!double.IsFinite(cutToRapidRatio)||cutToRapidRatio<=0)return 0;
        double planar=Math.Sqrt((from.X-to.X)*(from.X-to.X)+(from.Y-to.Y)*(from.Y-to.Y));
        return (Math.Max(0,safeZ-from.Z)+planar+Math.Max(0,safeZ-to.Z))*cutToRapidRatio;
    }
}
