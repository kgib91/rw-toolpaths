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
    internal long CandidateChordsExamined{get;private set;}
    internal long CandidateNominations{get;private set;}
    internal long CandidatesRetainedForSort{get;private set;}
    internal long CandidatesDiscardedBeforeSort{get;private set;}
    internal long BoundedQueries{get;private set;}
    internal long IndexBuilds{get;private set;}
    internal long IndexBuildRefusals{get;private set;}
    internal long IndexPathQueries{get;private set;}
    internal long BoundsNodeVisits{get;private set;}
    internal long BoundsPrunedRuns{get;private set;}
    internal long BoundsPrunedChords{get;private set;}
    internal long IndexRetainedBytes{get;private set;}
    private const int MinimumIndexedChords=128,LeafChords=16,MaximumTrackedPaths=65536;
    private const long MaximumIndexBytes=16L<<20;
    // Outside this finite arithmetic domain retain the literal scan. Bounds
    // inside it enclose the rounded interpolation itself, not an ideal segment.
    private const double IndexedCoordinateLimit=1e12;
    private readonly SegmentTree?[] trees;
    private readonly byte[] pathQueries,indexRefused;
    private readonly long[] lastPathQuery;
    private long querySequence;
    internal string IndexDiagnostics=>$"seamIndexBuilds={IndexBuilds} seamIndexBuildRefusals={IndexBuildRefusals} "
        +$"seamIndexPathQueries={IndexPathQueries} seamBoundsNodeVisits={BoundsNodeVisits} seamBoundsPrunedRuns={BoundsPrunedRuns} "
        +$"seamBoundsPrunedChords={BoundsPrunedChords} seamIndexRetainedBytes={IndexRetainedBytes} seamIndexMaximumBytes={MaximumIndexBytes} "
        +"seamIndexOrder=original-owner-chord-add seamBoundsAreNominationOnly=True";
    private readonly record struct BoundsNode(int First,int End,int Left,int Right,double MinX,double MinY,double MaxX,double MaxY);
    private sealed record SegmentTree(IReadOnlyList<Point3D> Points,int PointCount,BoundsNode[] Nodes,long RetainedBytes);
    private static readonly IComparer<(double Distance,int Owner,int End,long Sequence)> WorstFirst=
        Comparer<(double Distance,int Owner,int End,long Sequence)>.Create((a,b)=>b.CompareTo(a));
    internal VCarveRingSeams(IReadOnlyList<TaggedToolpath> paths,CancellationToken token)
    {
        this.paths=paths;this.token=token;
        int tracked=paths.Count<=MaximumTrackedPaths?paths.Count:0;
        trees=new SegmentTree?[tracked];pathQueries=new byte[tracked];indexRefused=new byte[tracked];lastPathQuery=new long[tracked];
        IndexRetainedBytes=1024+24L*tracked;
    }
    private SegmentTree? Index(int owner,long query,bool mayBuild)
    {
        if((uint)owner>=(uint)trees.Length)return null;
        var points=paths[owner].Points;
        if(trees[owner] is {} existing)
        {
            if(ReferenceEquals(existing.Points,points)&&existing.PointCount==points.Count)return existing;
            // A replacement source cannot use bounds belonging to the earlier
            // immutable point-list owner. Ordinary routing never mutates it.
            trees[owner]=null;IndexRetainedBytes-=existing.RetainedBytes;indexRefused[owner]=1;
            IndexBuildRefusals++;return null;
        }
        if(!mayBuild||indexRefused[owner]!=0||points.Count-1<MinimumIndexedChords)return null;
        if(lastPathQuery[owner]!=query)
        {lastPathQuery[owner]=query;if(pathQueries[owner]<2)pathQueries[owner]++;}
        if(pathQueries[owner]<2)return null;
        long slots=4*((points.Count-1L+LeafChords-1)/LeafChords),bytes=128+64*slots;
        if(bytes>MaximumIndexBytes-IndexRetainedBytes)
        {indexRefused[owner]=1;IndexBuildRefusals++;return null;}
        var nodes=new BoundsNode[(int)slots];int next=0;bool valid=true;
        int Build(int first,int end)
        {
            token.ThrowIfCancellationRequested();int node=next++;
            if(end-first<=LeafChords)
            {
                double minX=double.PositiveInfinity,minY=minX,maxX=double.NegativeInfinity,maxY=maxX;
                for(int i=first;i<end;i++)
                {
                    var a=points[i-1];var b=points[i];
                    if(!Finite(a)||!Finite(b))
                    {valid=false;return node;}
                    // For clamped finite t, rounded t*(b-a) lies between zero
                    // and the literal rounded difference. Rounded addition is
                    // monotone, so every actual a+t*(b-a) lies between a and
                    // a+(b-a). Include b separately for the t==1 endpoint arm.
                    double restoredX=a.X+(b.X-a.X),restoredY=a.Y+(b.Y-a.Y);
                    minX=Math.Min(minX,Math.Min(a.X,Math.Min(b.X,restoredX)));
                    minY=Math.Min(minY,Math.Min(a.Y,Math.Min(b.Y,restoredY)));
                    maxX=Math.Max(maxX,Math.Max(a.X,Math.Max(b.X,restoredX)));
                    maxY=Math.Max(maxY,Math.Max(a.Y,Math.Max(b.Y,restoredY)));
                }
                nodes[node]=new(first,end,-1,-1,minX,minY,maxX,maxY);return node;
            }
            int middle=first+(end-first)/2,left=Build(first,middle),right=Build(middle,end);
            if(valid){var a=nodes[left];var b=nodes[right];nodes[node]=new(first,end,left,right,
                Math.Min(a.MinX,b.MinX),Math.Min(a.MinY,b.MinY),Math.Max(a.MaxX,b.MaxX),Math.Max(a.MaxY,b.MaxY));}
            return node;
        }
        Build(1,points.Count);token.ThrowIfCancellationRequested();
        if(!valid){indexRefused[owner]=1;IndexBuildRefusals++;return null;}
        var built=new SegmentTree(points,points.Count,nodes,bytes);
        trees[owner]=built;IndexRetainedBytes+=bytes;IndexBuilds++;return built;
    }
    private static bool Finite(Point3D point)=>double.IsFinite(point.X)&&double.IsFinite(point.Y)
        &&Math.Abs(point.X)<=IndexedCoordinateLimit&&Math.Abs(point.Y)<=IndexedCoordinateLimit;
    private static double LowerSquaredGap(double from,double minimum,double maximum)
    {
        // Use the same subtraction and square as literal candidate pricing.
        // Each next-down fence lowers the gap/square; ties remain unpruned.
        double gap=Math.Abs(from<minimum?from-minimum:from>maximum?from-maximum:0);
        gap=Math.Max(0,Math.BitDecrement(gap));
        return Math.Max(0,Math.BitDecrement(Math.Pow(gap,2)));
    }
    internal IEnumerable<Landing> Nearest(Point3D from,IEnumerable<int> owners,double reach,int maximumCandidates=int.MaxValue)
    {
        if(maximumCandidates<0)throw new ArgumentOutOfRangeException(nameof(maximumCandidates));
        if(maximumCandidates==0)return Array.Empty<Landing>();
        // Callers consume only their fixed landing budget. Keep the exact
        // ordered prefix, including the original stable tie order, without
        // allocating and sorting every candidate on the constructed row.
        // A bounds-pruned run is strictly farther than the current retained
        // prefix/reach. Its omitted duplicate can never become competitive as
        // that threshold decreases. All surviving nominations retain Add order.
        token.ThrowIfCancellationRequested();long query=++querySequence;
        bool bounded=maximumCandidates!=int.MaxValue;
        PriorityQueue<Landing,(double Distance,int Owner,int End,long Sequence)>? nearest=null;
        if(bounded)BoundedQueries++;
        long sequence=0,nominated=0;double prefixWorst=0,reachSquared=reach*reach;
        var found=new List<Landing>();var seen=new HashSet<(int,Point3D)>();
        foreach(int owner in owners)
        {
            bool queryInDomain=double.IsFinite(from.X)&&double.IsFinite(from.Y)
                &&Math.Abs(from.X)<=IndexedCoordinateLimit&&Math.Abs(from.Y)<=IndexedCoordinateLimit;
            var tree=queryInDomain&&!double.IsNaN(reachSquared)&&(bounded||double.IsFinite(reachSquared))?Index(owner,query,true):null;
            if(tree is null)for(int end=1;end<paths[owner].Points.Count;end++)Examine(end);
            else{IndexPathQueries++;Visit(0);}
            void Visit(int ordinal)
            {
                if((BoundsNodeVisits++&255)==0)token.ThrowIfCancellationRequested();
                var node=tree!.Nodes[ordinal];double threshold=reachSquared;
                if(nearest is not null&&nearest.TryPeek(out _,out var worst))threshold=Math.Min(threshold,worst.Distance);
                else if(bounded&&found.Count==maximumCandidates)threshold=Math.Min(threshold,prefixWorst);
                double lower=LowerSquaredGap(from.X,node.MinX,node.MaxX)+LowerSquaredGap(from.Y,node.MinY,node.MaxY);
                lower=Math.Max(0,Math.BitDecrement(lower));
                if(lower>threshold)
                {BoundsPrunedRuns++;BoundsPrunedChords+=node.End-node.First;return;}
                if(node.Left<0)for(int end=node.First;end<node.End;end++)Examine(end);
                else{Visit(node.Left);Visit(node.Right);}
            }
            void Examine(int end)
            {
            CandidateChordsExamined++;
            if((end&1023)==0)token.ThrowIfCancellationRequested();
            var a=paths[owner].Points[end-1];var b=paths[owner].Points[end];
            double dx=b.X-a.X,dy=b.Y-a.Y,square=dx*dx+dy*dy;if(square==0)return;
            double t=Math.Clamp(((from.X-a.X)*dx+(from.Y-a.Y)*dy)/square,0,1);
            var point=t==0?a:t==1?b:new(a.X+t*dx,a.Y+t*dy,a.Z+t*(b.Z-a.Z));
            Add(t,point);Add(0,a);Add(1,b);
            void Add(double parameter,Point3D candidate)
            {
                double distance=Math.Pow(from.X-candidate.X,2)+Math.Pow(from.Y-candidate.Y,2);
                if(distance<=reach*reach&&seen.Add((owner,candidate)))
                {
                    nominated++;
                    var landing=new Landing(owner,end,parameter,candidate,distance);
                    if(!bounded){found.Add(landing);return;}
                    var rank=(distance,owner,end,sequence++);
                    if(nearest is null)
                    {
                        if(found.Count<maximumCandidates){found.Add(landing);prefixWorst=Math.Max(prefixWorst,distance);return;}
                        // Empty and short finite-reach queries retain the
                        // original small-list allocation and sorting behavior.
                        nearest=new(Math.Min(maximumCandidates,64),WorstFirst);
                        for(int i=0;i<found.Count;i++)
                        {
                            var retained=found[i];
                            nearest.Enqueue(retained,(retained.DistanceSquared,retained.Owner,retained.End,(long)i));
                        }
                        found.Clear();
                    }
                    if(nearest.TryPeek(out _,out var worst)&&rank.CompareTo(worst)<0)
                        nearest.DequeueEnqueue(landing,rank);
                }
            }
            }
        }
        token.ThrowIfCancellationRequested();
        CandidateNominations+=nominated;
        int retained=nearest?.Count??found.Count;
        CandidatesRetainedForSort+=retained;
        CandidatesDiscardedBeforeSort+=nominated-retained;
        if(nearest is not null)return nearest.UnorderedItems.OrderBy(item=>item.Priority).Select(item=>item.Element).ToArray();
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
