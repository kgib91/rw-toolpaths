using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

internal sealed partial class VCarveMaterialSurface
{
    internal sealed record ClearingLayer(List<List<PointD>> Rings, double MinimumRadius);
    private readonly Dictionary<(int Layer,double Radius,bool Original),List<List<PointD>>> clearingLayerInsets=new();

    /// <summary>Construct the clearing contact front from the same actual stock
    /// layers that bound medial fitting. This nominates centers, never a new target.
    /// The original complete native/F3 cutter checks still own every chord.</summary>
    internal Func<double,double,ClearingLayer?> CreateClearingInset(
        IReadOnlyList<IReadOnlyList<PointD>> boundary,int regionIndex,bool preferOriginalFirstConstruction=false)
    {
        var cache=new Dictionary<double,ClearingLayer?>();
        return (depth,originalRadius)=>
        {
            token.ThrowIfCancellationRequested();
            if(cache.TryGetValue(depth,out var cached))return cached;
            long started=Stopwatch.GetTimestamp();
            double commandedDepth=Math.Max(depth,-Command(new(0,0,-depth)).Z);
            var active=layers.Select((layer,index)=>(Layer:layer,Index:index))
                .Where(x=>x.Layer.TopDepth<commandedDepth).ToArray();
            double minimumRadius=active.Length==0?originalRadius:
                profile.PhysicalRadiusAtDepth(commandedDepth-active[^1].Layer.TopDepth);
            // No lowered layer participates: preserve the original construction
            // literally, including its existing rounding reserve and topology.
            if(active.Length<2||minimumRadius>=originalRadius)
            {
                cache.Add(depth,null);
                diagnostic?.Invoke($"stage=material-clearing region={regionIndex} depth={depth:R} originalRadius={originalRadius:R} "
                    +$"minimumRadius={originalRadius:R} layers={active.Length} policy=original-top nativeConstructorPreserved=True");
                return null;
            }

            long proofs=0;int reused=0;
            for(int attempt=0;attempt<4;attempt++)
            {
                // Intersections introduce new posted vertices. If their whole
                // cutter cannot be proved, reconstruct the complete front with
                // a small additional clearance; never delete individual chords.
                double reserve=attempt*PlanarReserve;
                var centers=PolygonOps.CutterInset(boundary,minimumRadius+reserve,cancellationToken:token,
                    preferOriginalFirstConstruction:preferOriginalFirstConstruction);
                foreach(var item in active)
                {
                    token.ThrowIfCancellationRequested();
                    double radius=profile.PhysicalRadiusAtDepth(commandedDepth-item.Layer.TopDepth)+reserve;
                    var key=(item.Index,radius,preferOriginalFirstConstruction);
                    if(!clearingLayerInsets.TryGetValue(key,out var inset))
                    {
                        inset=PolygonOps.CutterInset(item.Layer.FreeRegion,radius,cancellationToken:token,
                            preferOriginalFirstConstruction:preferOriginalFirstConstruction);
                        clearingLayerInsets.Add(key,inset);
                    }
                    else reused++;
                    centers=PolygonOps.Intersect(centers,inset);
                    if(centers.Count==0)break;
                }
                bool proved=true;
                foreach(var ring in centers)
                {
                    for(int i=0;i<ring.Count;i++)
                    {
                        token.ThrowIfCancellationRequested();proofs++;
                        var a=ring[i];var b=ring[(i+1)%ring.Count];
                        if(!IsLegalChord(new(a.x,a.y,-depth),new(b.x,b.y,-depth)))
                        {proved=false;break;}
                    }
                    if(!proved)break;
                }
                if(!proved)continue;
                var result=new ClearingLayer(centers,minimumRadius);
                cache.Add(depth,result);
                diagnostic?.Invoke($"stage=material-clearing region={regionIndex} depth={depth:R} commandedDepth={commandedDepth:R} "
                    +$"originalRadius={originalRadius:R} minimumRadius={minimumRadius:R} layers={active.Length} "
                    +$"rings={centers.Count} points={centers.Sum(r=>r.Count)} proofs={proofs} reusedLayerInsets={reused} "
                    +$"additionalReserve={reserve:R} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
                    +"policy=intersect-actual-stock-layer-insets nativeAndPostedProved=True declaredStartingDepthCredit=False targetUnchanged=True");
                return result;
            }
            throw new InvalidOperationException($"Could not certify the stock-aware V-carve perimeter: region={regionIndex}, depth={depth:R}.");
        };
    }
}
