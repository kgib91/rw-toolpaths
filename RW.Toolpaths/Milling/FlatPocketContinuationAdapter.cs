namespace RW.Toolpaths.Milling;

/// <summary>Adapts the existing ordered pocket stock authority to the shared
/// routing policy. A legal pocket domain alone never proves cleared stock.</summary>
internal sealed class FlatPocketContinuationAdapter
{
    private readonly ToolGeometry tool;
    private readonly DepthSchedule depth;
    private readonly ILinkClearance clearance;
    private readonly CancellationToken token;
    private readonly PocketContinuationRouter.ClearedStock stock;
    private readonly PocketPostedStepdownStock posted;
    private readonly PocketContinuationRouter.FollowingSweepCache followingCache = new();
    private readonly PocketContinuationRouter.CoverageDiagnostics coverageDiagnostics = new();
    private PocketContinuationRouter.TemporalClearance? proof;
    private IReadOnlyList<Point3D>? proofPath;
    private double proofFloor;
    private readonly int maximumAdditionalWitnessSites;
    internal bool Supported { get; }
    internal FlatPocketContinuationAdapter(ToolGeometry tool, DepthSchedule depth, ILinkClearance clearance,
        CancellationToken token,IReadOnlyList<IReadOnlyList<Clipper2Lib.PointD>>? knownEmpty=null,
        double knownEmptyFloor=double.PositiveInfinity,int maximumAdditionalWitnessSites=29)
    {
        if(maximumAdditionalWitnessSites<0||maximumAdditionalWitnessSites>125)
            throw new ArgumentOutOfRangeException(nameof(maximumAdditionalWitnessSites));
        this.maximumAdditionalWitnessSites=maximumAdditionalWitnessSites;
        this.tool = tool; this.depth = depth; this.clearance = clearance;
        this.token = token;
        Supported = PocketContinuationRouter.TryFlatCylinderRadius(tool,
            PocketContinuationRouter.RequiredPostedCylinderHeight(depth), out _);
        stock = new(tool.Radius,token,knownEmpty:knownEmpty,knownEmptyFloor:knownEmptyFloor);
        posted = new(depth,tool.Radius,token,Supported);
    }
    internal bool AllowsNewChord(TaggedToolpath path, Point3D a, Point3D b)
    {
        if (!Supported || a.Z != b.Z || b.Z >= depth.SurfaceZ) return false;
        if(!clearance.IsTravelSafe(Posted(a),Posted(b),tool.Radius))return false;
        if (proof is null || !ReferenceEquals(proofPath,path.Points) || proofFloor != b.Z)
        {
            var first = path.Spans.FirstOrDefault(s => s.Kind == ToolpathSpanKind.Cut && s.MoveCount > 0);
            if (first.MoveCount <= 0) return false;
            var following = PocketContinuationRouter.BelowFloor(path.Points.Skip(first.StartIndex)
                .Take(first.PointCount).ToArray(),b.Z);
            proof = new(clearance,stock,b.Z,following,tool.Radius,token,
                diagnostics:coverageDiagnostics,followingCache:followingCache,priorStepdown:posted.Capture(),
                maximumAdditionalWitnessSites:maximumAdditionalWitnessSites);
            proofPath = path.Points; proofFloor = b.Z;
        }
        return proof.IsTravelSafe(a,b,tool.Radius);
    }
    internal void Record(TaggedToolpath path)
    {
        if (!Supported) return;
        foreach (var span in path.Spans)
        {
            if (span.Kind is not (ToolpathSpanKind.Cut or ToolpathSpanKind.Ramp or ToolpathSpanKind.Link)) continue;
            var stroke = path.Points.Skip(span.StartIndex).Take(span.PointCount).ToArray();
            stock.Record(stroke); posted.Record(stroke);
        }
        proof = null; proofPath = null;
    }
    private Point3D Posted(Point3D p)=>new(Math.Round((double)(float)p.X*1000,MidpointRounding.ToEven)/1000,
        Math.Round((double)(float)p.Y*1000,MidpointRounding.ToEven)/1000,
        depth.SurfaceZ+Math.Round(((float)p.Z-(float)depth.SurfaceZ)*1000d,MidpointRounding.ToEven)/1000);

    internal void Report()
        => Console.WriteLine($"[FlatPocketContinuation.Perf] recordedStrokes={stock.RecordedStrokes} floors={stock.FloorCount} "
            +$"floorQueries={stock.FloorQueries} floorSweepMs={stock.FloorSweepMilliseconds:F1} floorUnionMs={stock.FloorUnionMilliseconds:F1} floorUnionRetainedVertices={stock.FloorUnionRetainedVertices} floorUnionSubmittedVertices={stock.FloorUnionSubmittedVertices} "
            +$"{coverageDiagnostics.Describe()} priorStepdownMs={posted.ProofMilliseconds:F1} "
            +$"timing=accumulated-owned-query-wall sourceAndPostedPhysicalGuardsUnchanged=True");
}
