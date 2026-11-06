using Clipper2Lib;
namespace RW.Toolpaths.Milling.ReviewedSource;

/// <summary>Actual residual-stock contact construction, before detail/routing.
/// Uses the established material-surface kernel; never generates a legacy path.</summary>
public sealed partial class ReviewedVCarveStockContact
{
    readonly VCarveMaterialSurface? owner;
    readonly VCarveClearedStock? priorStock;
    readonly double tolerance;
    readonly IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> boundaries;
    readonly VCarveCutterProfile profile;
    readonly double maximumDepth;
    readonly double surface;
    readonly CancellationToken token;
    readonly Action<string>? diagnostic;
    public sealed record Receipt(int OriginalNodes,int ContactNodes,int OriginalEdges,int ContactEdges,
        int[] NodeMap,int[][] EdgeMap,bool HasActualStock,string Policy);
    public ReviewedVCarveStockContact(IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> boundaries,
        VCarveCutterProfile profile,DepthSchedule depth,VCarveClearedStock? stock,double tolerance,
        CancellationToken token=default,Action<string>? diagnostic=null)
    {
        if(!double.IsFinite(tolerance)||tolerance<=0)throw new ArgumentOutOfRangeException(nameof(tolerance));
        priorStock=stock;this.tolerance=tolerance;this.boundaries=boundaries;this.profile=profile;maximumDepth=depth.Depth;surface=depth.SurfaceZ;this.token=token;this.diagnostic=diagnostic;
        if(stock is not null)owner=new VCarveMaterialSurface(boundaries.SelectMany(x=>x),stock,profile,depth.Depth,depth.SurfaceZ,token,diagnostic);
    }

    /// <summary>Retain the immutable stock/target authority used to construct
    /// this source, including its already established full-cutter proofs.</summary>
    public ToolpathPlan AttachClearance(IReadOnlyList<TaggedToolpath> raw,Action<string>? diagnostic=null)
        =>ReviewedVCarvePlan.AttachClearance(raw,boundaries,profile,surface,owner,token,
            diagnostic??this.diagnostic,reusedAuthority:owner is not null);
    // One cap for the original graph, derived from the same full-cutter
    // stock layers used for finishing. No corner selection or appended visits.
    private Paths64? depthFront;
    public IReadOnlyList<IReadOnlyList<PointD>> DepthFront(ReviewedVCarveRegion region)
    {
        if(owner is null)throw new InvalidOperationException("STOCK_CONTACT_REQUIRED");
        if(depthFront is null)
        {
            Paths64? intersection=null;
            foreach(var band in owner.ContactReviewBands())
            {
                token.ThrowIfCancellationRequested();
                double radius=profile.PhysicalRadiusAtDepth(Math.Max(0,maximumDepth-band.Top))
                    +2*VCarveMaterialSurface.PlanarReserve;
                var offset=new ClipperOffset(2,.001*PathUtils.Scale);
                offset.AddPaths(PathUtils.ToClipper(band.Rings),JoinType.Round,EndType.Polygon);
                var feasible=new Paths64();offset.Execute(-radius*PathUtils.Scale,feasible);
                intersection=intersection is null?feasible:Clipper.Intersect(intersection,feasible,FillRule.NonZero);
            }
            depthFront=intersection??new();
        }
        var clipped=Clipper.Intersect(depthFront,PathUtils.ToClipper(region.Boundary),FillRule.NonZero);
        return PathUtils.FromClipper(clipped).Select(r=>(IReadOnlyList<PointD>)r).ToArray();
    }
    public (ReviewedVCarveConstructor.Result Graph,Receipt Evidence) Project(ReviewedVCarveConstructor.Result nominal)
    {
        // No prior-stock evidence means nominal original-stock contact, not a
        // fabricated cleared floor inferred from StartingDepth alone.
        if(owner is null)return(nominal,new(nominal.Nodes.Length,nominal.Nodes.Length,nominal.Edges.Length,nominal.Edges.Length,
            Enumerable.Range(0,nominal.Nodes.Length).ToArray(),Enumerable.Range(0,nominal.Edges.Length).Select(i=>new[]{i}).ToArray(),false,"original-stock-contact-no-prior-removal"));
        var nodes=new List<Point3D>();var unique=new Dictionary<Point3D,int>();
        int Node(Point3D point){if(unique.TryGetValue(point,out int known))return known;int index=nodes.Count;nodes.Add(point);unique.Add(point,index);return index;}
        // Fit a shared graph joint ONCE. Every return/branch uses the same pose.
        var map=nominal.Nodes.Select(p=>Node(owner.ProjectContactPose(p))).ToArray();
        var edges=new List<ReviewedVCarveConstructor.Edge>();var edgeMap=new List<int[]>();
        foreach(var edge in nominal.Edges)
        {
            IReadOnlyList<Point3D> chain;
            try{chain=owner.ProjectCanonicalContactEdge(nominal.Nodes[edge.A],nominal.Nodes[edge.B],nodes[map[edge.A]],nodes[map[edge.B]],tolerance);}
            catch(InvalidOperationException)
            {
                var a=nominal.Nodes[edge.A];var b=nominal.Nodes[edge.B];
                diagnostic?.Invoke($"stage=material-contact event=source-edge-refused region={nominal.Region} owner={edge.Owner} nodeA={edge.A} nodeB={edge.B} sourceA={a} sourceB={b} planarLengthMm={Math.Sqrt(Math.Pow(b.X-a.X,2)+Math.Pow(b.Y-a.Y,2)):R} poseLengthMm={Math.Sqrt(Math.Pow(b.X-a.X,2)+Math.Pow(b.Y-a.Y,2)+Math.Pow(b.Z-a.Z,2)):R}");
                throw;
            }
            var aliases=new List<int>();int previous=map[edge.A];
            foreach(var point in chain.Skip(1))
            {
                int next=Node(point);if(next==previous)continue;
                aliases.Add(edges.Count);edges.Add(new(previous,next,edge.Owner));previous=next;
            }
            if(previous!=map[edge.B])throw new InvalidOperationException("STOCK_CONTACT_JOINT_OWNERSHIP_MISMATCH");
            edgeMap.Add(aliases.ToArray());
        }
        return(nominal with{Nodes=nodes.ToArray(),Edges=edges.ToArray()},new(nominal.Nodes.Length,nodes.Count,nominal.Edges.Length,edges.Count,map,edgeMap.ToArray(),true,"production-top-down-stock-contact-v1"));
    }
}
