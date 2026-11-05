using System.Diagnostics;

namespace RW.Toolpaths.Milling;

internal static partial class ToolpathSourceTraversal
{
    // Literal required junctions are small excursions from an existing row.
    // Do not give them an independent entry when a guarded out-and-back bridge
    // is available. All untouched carrier vertices and junction chords survive.
    internal static List<TaggedToolpath> JunctionDetours(IReadOnlyList<TaggedToolpath> carriers,
        IReadOnlyList<TaggedToolpath> junctions,double reach,
        Func<TaggedToolpath,TaggedToolpath,bool> sameRow,Func<Point3D,Point3D,bool> allows,
        CancellationToken token,Action<string>? log,out int attached)
    {
        var result=carriers.ToList();attached=0;double bridges=0;int trials=0;var watch=Stopwatch.StartNew();
        foreach(var branch in junctions){
            token.ThrowIfCancellationRequested();
            if(branch.Points.Count<2||branch.Spans.Any(s=>s.Kind!=ToolpathSpanKind.Cut))
            {result.Add(branch);continue;}
            // Reversing a literal cut still needs commanded-coordinate proof.
            bool reversible=true;
            for(int i=1;i<branch.Points.Count&&reversible;i++)
                reversible=allows(branch.Points[i-1],branch.Points[i])&&allows(branch.Points[i],branch.Points[i-1]);
            if(!reversible){result.Add(branch);continue;}
            var candidates=new List<(double Distance,int Host,int Vertex,bool Reverse)>();
            for(int host=0;host<result.Count;host++){
                var carrier=result[host];if(!sameRow(carrier,branch)||carrier.Spans.Any(s=>s.Kind!=ToolpathSpanKind.Cut))continue;
                for(int vertex=0;vertex<carrier.Points.Count;vertex++)for(int side=0;side<2;side++){
                    var from=carrier.Points[vertex];var to=branch.Points[side==0?0:branch.Points.Count-1];
                    double length=ToolpathContinuationPlanner.Distance(from,to);
                    if(length>reach)continue;
                    candidates.Add((length,host,vertex,side!=0));
                }
            }
            bool joined=false;
            foreach(var c in candidates.OrderBy(c=>c.Distance).Take(16)){
                token.ThrowIfCancellationRequested();trials++;
                var parent=result[c.Host];var from=parent.Points[c.Vertex];
                var walk=(c.Reverse?branch.Points.Reverse():branch.Points).ToArray();
                if(!allows(from,walk[0])||!allows(walk[0],from))continue;
                var points=parent.Points.Take(c.Vertex+1).Concat(walk).Concat(walk.Reverse().Skip(1))
                    .Append(from).Concat(parent.Points.Skip(c.Vertex+1)).ToArray();
                result[c.Host]=parent with{Points=points,Moves=new LineToolMoves(points),Spans=[new(0,points.Length-1,ToolpathSpanKind.Cut)]};
                attached++;bridges+=2*c.Distance;joined=true;break;
            }
            if(!joined)result.Add(branch);
        }
        log?.Invoke($"stage=junction-detours carriers={carriers.Count} requiredJunctions={junctions.Count} attached={attached} independent={junctions.Count-attached} bridgeLengthMm={bridges:R} trials={trials} elapsedMs={watch.Elapsed.TotalMilliseconds:F1} "
            +"literalCarrierAndJunctionChordsRetained=True newBridgeAndBothDirectionsProved=True noStockOrTravelCredit=True");
        return result;
    }
}
