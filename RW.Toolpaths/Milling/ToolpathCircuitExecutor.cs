namespace RW.Toolpaths.Milling;

/// <summary>Executes required circuits through a certified shallow region tree.
/// Complete each circuit's bounded depth passes, return over its actual descent
/// traces, and continue the carrier at its literal portal. No motion-graph search.</summary>
internal static class ToolpathCircuitExecutor
{
    private sealed class Frame
    {
        internal int Owner;
        internal RampedPath Entry=null!;
        internal Point3D? ParentPortal;
        internal int Position;
    }

    internal static List<TaggedToolpath> Execute(IReadOnlyList<TaggedToolpath> circuits,
        double transitDepth, double proximity, DepthSchedule schedule, RampSettings ramp,
        IReadOnlyList<(double Nominal,double Command)> caps,
        Func<Point3D,Point3D,bool> allowsTransit, CancellationToken token, Action<string>? log=null,
        double gradeLimit=double.PositiveInfinity,
        Func<int,Point3D,Point3D,bool>? clearedCarrier=null, Point3D? start=null)
    {
        var assigned=ToolpathContinuationComponents.Assign(circuits,
            p=>p with{Z=Math.Max(p.Z,schedule.SurfaceZ-transitDepth)},allowsTransit,proximity,token,log,start);
        if(assigned.Count==0)return new();
        var tree=assigned[0].ContinuationPlan!;
        var children=Enumerable.Range(0,circuits.Count).Select(_=>new Dictionary<Point3D,List<(int Owner,Point3D Point)>>()).ToArray();
        void Portal(ToolpathContinuationComponents.Portal a,ToolpathContinuationComponents.Portal b)
        {
            if(!children[a.Owner].TryGetValue(a.Point,out var row))children[a.Owner][a.Point]=row=new();
            row.Add((b.Owner,b.Point));
        }
        foreach(var edge in tree.Edges){Portal(edge.A,edge.B);Portal(edge.B,edge.A);}
        var used=new bool[circuits.Count];var result=new List<TaggedToolpath>();
        var exactPlanarCustody=Enumerable.Repeat(true,circuits.Count).ToArray();
        var stack=new Stack<Frame>();
        long descentPoints=0,ascentPoints=0,completedExcursionPoints=0,carrierPointsRemoved=0;
        double descentDistance=0,ascentDistance=0,depthPassDistance=0;
        double Length(IReadOnlyList<Point3D> points)
        {
            double length=0;
            for(int i=1;i<points.Count;i++)length+=ToolpathContinuationPlanner.Distance(points[i-1],points[i]);
            return length;
        }
        int entries=0,services=0,partialAxisFallbacks=0,portalRetainedPasses=0,serviceDepthPasses=0;
        var groups=assigned.Select((path,owner)=>(path,owner)).GroupBy(p=>p.path.ContinuationComponentIndex).OrderBy(g=>g.Key).ToArray();
        var visits=groups.Select(g=>
        {
            int root=g.OrderBy(p=>tree.VisitOrder[p.owner]).First().owner;
            var point=circuits[root].Points[0];
            var points=new[]{point,point};
            return new TaggedToolpath(points,Array.Empty<ToolpathSpan>(),0,"routing",null,detectCurves:false);
        }).ToArray();
        double Transfer(IEnumerable<int> order)
        {
            Point3D? previous=start;double length=0;
            foreach(int i in order)
            {
                var point=visits[i].Points[0];
                if(previous is {} p)length+=Math.Sqrt((point.X-p.X)*(point.X-p.X)+(point.Y-p.Y)*(point.Y-p.Y));
                previous=point;
            }
            return length;
        }
        var visitOrder=ToolpathVisitOrder.Order(visits,Enumerable.Repeat(false,visits.Length).ToArray(),start,token);
        double originalTransfer=Transfer(Enumerable.Range(0,groups.Length));
        if(Transfer(visitOrder)>originalTransfer+1e-8)visitOrder=Enumerable.Range(0,groups.Length).ToArray();
        double transferDistance=Transfer(visitOrder);
        foreach(int groupIndex in visitOrder)
        {
            var group=groups[groupIndex];
            int root=group.OrderBy(p=>tree.VisitOrder[p.owner]).First().owner;
            var seam=AtDepth(root,transitDepth,-1)[0];
            Enter(root,seam,null,true);
            while(stack.Count>0)
            {
                token.ThrowIfCancellationRequested();var frame=stack.Peek();var points=frame.Entry.Points;
                int next=frame.Position;
                (int Owner,Point3D Point)? child=null;
                while(next<points.Count)
                {
                    if(children[frame.Owner].TryGetValue(points[next],out var row))
                        foreach(var candidate in row)if(!used[candidate.Owner]){child=candidate;break;}
                    if(child.HasValue)break;next++;
                }
                int end=child.HasValue ? next : points.Count-1;
                EmitSlice(frame.Owner,frame.Entry,frame.Position,end);
                frame.Position=end;
                if(child is { } branch)
                {
                    Emit(frame.Owner,new[]{points[end],branch.Point},ToolpathSpanKind.Link);
                    Enter(branch.Owner,branch.Point,points[end],false);
                    continue;
                }
                if(frame.ParentPortal is { } parent)Emit(frame.Owner,new[]{frame.Entry.Points[^1],parent},ToolpathSpanKind.Link);
                stack.Pop();
            }
            entries++;
        }
        double maximumGrade=0;long checkedChords=0;int continuousBoundaries=0;
        for(int pathIndex=0;pathIndex<result.Count;pathIndex++)
        {
            var path=result[pathIndex];
            if(pathIndex>0 && result[pathIndex-1].ContinuationComponentIndex==path.ContinuationComponentIndex)
            {
                if(result[pathIndex-1].Points[^1]!=path.Points[0])
                    throw new InvalidOperationException("A region excursion lost its exact executed return pose.");
                continuousBoundaries++;
            }
            for(int i=1;i<path.Points.Count;i++)
            {
                if((++checkedChords&1023)==0)token.ThrowIfCancellationRequested();
                var a=path.Points[i-1];var b=path.Points[i];
                double run=Math.Sqrt((a.X-b.X)*(a.X-b.X)+(a.Y-b.Y)*(a.Y-b.Y)),rise=Math.Abs(a.Z-b.Z);
                if(rise>gradeLimit*run+1e-8)
                    throw new InvalidOperationException("A region excursion exceeded its authored ramp grade.");
                if(run>1e-12)maximumGrade=Math.Max(maximumGrade,rise/run);
            }
        }
        log?.Invoke($"stage=region-execution event=complete requiredCircuits={circuits.Count} connectedRegions={entries} "
            +$"surfaceEntries={entries} servicedCircuits={services} transitDepth={transitDepth:R} outputPaths={result.Count} "
            +$"descentPoints={descentPoints} ascentPoints={ascentPoints} depthPassPoints={completedExcursionPoints} partialAxisFallbacks={partialAxisFallbacks} "
            +$"descentDistance={descentDistance:R} ascentDistance={ascentDistance:R} depthPassDistance={depthPassDistance:R} axisPolicy=exact-planar-displacement "
            +$"continuousBoundaries={continuousBoundaries} checkedChords={checkedChords} maximumGrade={maximumGrade:R} "
            +$"carrierPointsRemoved={carrierPointsRemoved} "
            +$"portalRetainedPasses={portalRetainedPasses} serviceDepthPasses={serviceDepthPasses} "
            +$"transferBefore={originalTransfer:R} transferDistance={transferDistance:R} rootPolicy=source-circuit-seam "
            +"sourceTree=True exactDescentRetraces=True allDepthsBeforeRegionExit=True emittedGraphSearch=False");
        return result;

        IReadOnlyList<Point3D> AtDepth(int owner,double cap,double completed)
        {
            if(circuits[owner].ContinuationCircuitDefinition is { } definition)
                return definition.AtDepth(cap,completed,token).Select(p=>p with{Z=p.Z+schedule.SurfaceZ}).ToArray();
            return circuits[owner].Points.Select(p=>p with{Z=Math.Max(p.Z,schedule.SurfaceZ-cap)}).ToArray();
        }
        IReadOnlyList<Point3D> Rotate(IReadOnlyList<Point3D> points,Point3D entry)
        {
            int at=-1;
            for(int i=0;i<points.Count-1;i++)if(points[i].X==entry.X&&points[i].Y==entry.Y){at=i;break;}
            if(at<0)throw new InvalidOperationException("A required circuit lost its literal region portal.");
            if(at==0)return points;
            int n=points.Count-1;return Enumerable.Range(0,n+1).Select(i=>points[(at+i)%n]).ToArray();
        }
        RampedPath Entry(IReadOnlyList<Point3D> points,double fromZ,out List<Point3D> descent)
        {
            var entered=RampPlanner.PlanPathEntry(points,fromZ,ramp,token,null,true,false);
            var rampSpan=entered.Spans.FirstOrDefault(s=>s.Kind==ToolpathSpanKind.Ramp);
            descent=rampSpan.EndIndex>rampSpan.StartIndex ? entered.Points.Take(rampSpan.EndIndex+1).ToList() : new();
            int contact=-1;
            if(descent.Count>0)
                for(int i=0;i<points.Count;i++)if(points[i]==descent[^1]){contact=i;break;}
            if(entered.PartialAxisFallback)
            {
                partialAxisFallbacks++;
                // Retain literal commanded custody in the uncommon partial-axis
                // case, including its legacy repeated tail.
                descent=entered.Points.ToList();
            }
            if(entered.Points[^1]!=points[0])
            {
                if(contact<0)throw new InvalidOperationException("A source circuit entry missed its exact contact vertex.");
                int tail=entered.Points.Count-1;
                for(int i=contact-1;i>=0;i--)entered.Points.Add(points[i]);
                entered.Spans.Add(new(tail,entered.Points.Count-1,ToolpathSpanKind.Link));
            }
            if(descent.Count>0&&!entered.PartialAxisFallback)
                for(int i=contact-1;i>=0;i--)descent.Add(points[i]);
            descentPoints+=descent.Count;
            descentDistance+=Length(descent);
            return entered;
        }
        void Enter(int owner,Point3D entry,Point3D? parent,bool root)
        {
            used[owner]=true;services++;
            var completion=new List<TaggedToolpath>();var returns=new List<List<Point3D>>();
            Point3D cursor=root ? entry with{Z=schedule.SurfaceZ} : entry;double completed=0;
            double deepest=circuits[owner].Points.Max(p=>schedule.SurfaceZ-p.Z);
            var layers=caps.Where(c=>c.Command<deepest-1e-10)
                .Select(c=>c.Nominal).Append(double.PositiveInfinity);
            int pass=0;
            foreach(double cap in layers)
            {
                if(deepest<=completed+1e-10)break;
                var points=AtDepth(owner,cap,completed);
                if(!points.Any(p=>p.X==entry.X&&p.Y==entry.Y))
                {points=AtDepth(owner,cap,-1);portalRetainedPasses++;}
                points=Rotate(points,entry);
                var entered=Entry(points,cursor.Z,out var descent);
                if(entered.PartialAxisFallback)exactPlanarCustody[owner]=false;
                completion.Add(Tag(owner,entered.Points,entered.Spans) with{
                    Category=double.IsPositiveInfinity(cap) ? circuits[owner].Category : "clearing",
                    DepthPassIndex=double.IsPositiveInfinity(cap)&&circuits[owner].Category=="final-carve" ? null : pass});
                pass++;serviceDepthPasses++;
                completedExcursionPoints+=entered.Points.Count;
                depthPassDistance+=Length(entered.Points);
                if(descent.Count>0)returns.Add(descent);
                cursor=entered.Points[^1];completed=cap;
            }
            bool hasChildren=children[owner].Values.SelectMany(row=>row).Any(child=>!used[child.Owner]);
            if(!root||hasChildren)
                for(int i=returns.Count-1;i>=0;i--)
                {
                    var points=returns[i];points.Reverse();ascentPoints+=points.Count;ascentDistance+=Length(points);
                    completion.Add(Tag(owner,points,new[]{new ToolpathSpan(0,points.Count-1,ToolpathSpanKind.Link)}));
                }
            // Service the complete floor first. Its literal executed chords can
            // then prove positive clearance for the shallower carrier. The
            // carrier is routing only; it is not an additional cutting pass.
            result.AddRange(completion);
            IReadOnlyList<Point3D> carrier=hasChildren ? Rotate(AtDepth(owner,transitDepth,-1),entry)
                : new[]{root ? cursor : entry};
            if(hasChildren)
            {
                var required=children[owner].Where(pair=>pair.Value.Any(c=>!used[c.Owner])).Select(pair=>pair.Key).ToHashSet();
                int before=carrier.Count;carrier=TrimCarrier(carrier,required);carrierPointsRemoved+=before-carrier.Count;
                if(root&&carrier.Count<2&&entry.Z<schedule.SurfaceZ)
                {
                    var original=Rotate(AtDepth(owner,transitDepth,-1),entry);
                    carrier=new[]{original[0],original[1],original[0]};
                }
                if(root&&entry.Z<schedule.SurfaceZ)
                    carrier=Entry(carrier,schedule.SurfaceZ,out _).Points;
            }
            var spans=new List<ToolpathSpan>();
            for(int i=1;i<carrier.Count;i++)
            {
                var kind=exactPlanarCustody[owner] && clearedCarrier?.Invoke(owner,carrier[i-1],carrier[i])==true
                    ? ToolpathSpanKind.ClearedTravel : ToolpathSpanKind.Link;
                if(spans.Count>0&&spans[^1].Kind==kind)spans[^1]=spans[^1] with{EndIndex=i};
                else spans.Add(new(i-1,i,kind));
            }
            var initial=new RampedPath(carrier.ToList(),spans,RampStrategy.Linear,0);
            stack.Push(new Frame{Owner=owner,Entry=initial,ParentPortal=parent});
        }
        TaggedToolpath Tag(int owner,IReadOnlyList<Point3D> points,IReadOnlyList<ToolpathSpan> spans)
            =>new(points,spans,circuits[owner].RegionIndex,circuits[owner].Category,null,detectCurves:false)
            {ContinuationComponentIndex=assigned[owner].ContinuationComponentIndex,ContinuationCircuit=true};
        void Emit(int owner,IReadOnlyList<Point3D> points,ToolpathSpanKind kind)
        {if(points.Count>1&&points[0]!=points[^1])result.Add(Tag(owner,points,new[]{new ToolpathSpan(0,points.Count-1,kind)}));}
        void EmitSlice(int owner,RampedPath path,int first,int last)
        {
            if(last<=first)return;
            var spans=path.Spans.Select(s=>new ToolpathSpan(Math.Max(first,s.StartIndex)-first,
                Math.Min(last,s.EndIndex)-first,s.Kind)).Where(s=>s.EndIndex>s.StartIndex).ToArray();
            result.Add(Tag(owner,path.Points.Skip(first).Take(last-first+1).ToArray(),spans));
        }
    }

    // Every removed excursion is closed at a literal already-cut pose and has
    // no unserviced portal inside it. This is topology reduction, not a sampled
    // surface-contribution filter. Remaining arcs are followed or retraced whole.
    private static IReadOnlyList<Point3D> TrimCarrier(IReadOnlyList<Point3D> source,HashSet<Point3D> required)
    {
        var points=new List<Point3D>();var counts=new List<int>();var last=new Dictionary<Point3D,int>();
        foreach(var point in source)
        {
            if(last.TryGetValue(point,out int at)&&counts[^1]==counts[at])
            {
                for(int i=points.Count-1;i>at;i--)last.Remove(points[i]);
                points.RemoveRange(at+1,points.Count-at-1);counts.RemoveRange(at+1,counts.Count-at-1);
                last[point]=at;continue;
            }
            last[point]=points.Count;points.Add(point);
            counts.Add((counts.Count>0 ? counts[^1] : 0)+(required.Contains(point)?1:0));
        }
        if(points.Count<2)return points;
        var stops=new List<int>{0};var seen=new HashSet<Point3D>{points[0]};
        for(int i=1;i<points.Count-1;i++)if(required.Contains(points[i])&&seen.Add(points[i]))stops.Add(i);
        if(stops.Count==1)return new[]{points[0]};
        var length=new double[points.Count];
        for(int i=1;i<points.Count;i++)length[i]=length[i-1]+ToolpathContinuationPlanner.Distance(points[i-1],points[i]);
        double gap=-1;int lo=0,hi=points.Count-1;
        for(int i=0;i<stops.Count;i++)
        {
            int a=stops[i],b=i+1<stops.Count ? stops[i+1] : points.Count-1;
            if(length[b]-length[a]>gap){gap=length[b]-length[a];lo=a;hi=b;}
        }
        if(2*(length[^1]-gap)>=length[^1])return points;
        var walk=new List<Point3D>();
        for(int i=0;i<=lo;i++)walk.Add(points[i]);
        for(int i=lo-1;i>=0;i--)walk.Add(points[i]);
        for(int i=points.Count-2;i>=hi;i--)walk.Add(points[i]);
        for(int i=hi+1;i<points.Count;i++)walk.Add(points[i]);
        return walk;
    }
}
