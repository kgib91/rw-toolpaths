using System.Security.Cryptography;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling.ReviewedSource;

/// <summary>A derived contact source and its certified native geometry. The
/// original projected trace owns reference/routing intervals, not fit joins.</summary>
public sealed record ReviewedVCarveNativeBranch(string Identity,int Region,
    Point3D[] SourceReference,Point3D[] ContactGuide,ToolMove[] Moves,
    double MaximumFitErrorMm,int[] NominalEdges);

public sealed record ReviewedVCarveContactSearchLimits(int MaximumPoseTrials=50_000,
    int FamilyRefinementSteps=3,int LocalRefinementSteps=8,int? MaximumNativePieces=null,int MaximumNativeTrials=50_000,
    int MaximumNativeTrialsPerNomination=64);

public sealed partial class ReviewedVCarveStockContact
{
    public static readonly AsyncLocal<Action<ReviewedVCarveNativeBranch>?> BranchObserver=new();
    internal sealed record Chain(int[] Nodes,int[] Edges,int TipDegree,int RootDegree);
    internal static Chain[] MedialTerminalChains(ReviewedVCarveConstructor.Result graph)
    {
        var adjacent=Enumerable.Range(0,graph.Nodes.Length).Select(_=>new List<int>()).ToArray();
        for(int i=0;i<graph.Edges.Length;i++){var e=graph.Edges[i];adjacent[e.A].Add(i);adjacent[e.B].Add(i);}
        var claimed=new HashSet<int>();var result=new List<Chain>();
        for(int tip=0;tip<adjacent.Length;tip++)
        {
            if(adjacent[tip].Count!=1)continue;
            var nodes=new List<int>{tip};var edges=new List<int>();int at=tip,previous=-1;
            do
            {
                int edge=adjacent[at].Single(i=>i!=previous);var e=graph.Edges[edge];
                edges.Add(edge);at=e.A==at?e.B:e.A;nodes.Add(at);previous=edge;
            }while(adjacent[at].Count==2);
            if(edges.Any(claimed.Contains)||edges.Any(i=>!graph.Edges[i].Owner.StartsWith("medial:",StringComparison.Ordinal)))continue;
            foreach(int edge in edges)claimed.Add(edge);
            result.Add(new(nodes.ToArray(),edges.ToArray(),1,adjacent[at].Count));
        }
        return result.ToArray();
    }

    /// <summary>Construct terminal corner contact before native routing. Planar
    /// medial samples are derived, so asymmetric stock may change their XY.
    /// Graph ports, depth-cap tasks and authored state remain unchanged.</summary>
    public IReadOnlyList<ReviewedVCarveNativeBranch> ConstructNativeBranches(
        ReviewedVCarveConstructor.Result nominal,ReviewedVCarveConstructor.Result contact,
        Receipt projection,ReviewedVCarveConstructor.Result retained,VCarveNativeFitSettings settings,
        ReviewedVCarveContactCoverage wholeSource,ReviewedVCarveContactSearchLimits? limits=null)
    {
        settings.Validate();ArgumentNullException.ThrowIfNull(wholeSource);limits??=new();
        if(owner is null)return [];
        if(limits.MaximumPoseTrials<=0||limits.MaximumNativePieces is <=0||limits.MaximumNativeTrials<=0||limits.MaximumNativeTrialsPerNomination<=0)
            throw new ArgumentOutOfRangeException(nameof(limits));
        var active=retained.Edges.ToHashSet();
        var origins=projection.EdgeMap.SelectMany((aliases,source)=>aliases.Select(alias=>(Edge:contact.Edges[alias],Source:source)))
            .GroupBy(p=>p.Edge).ToDictionary(g=>g.Key,g=>g.Select(p=>p.Source).Distinct().Single());
        var nodeOrigins=projection.NodeMap.Select((node,source)=>(Node:node,Source:source)).GroupBy(p=>p.Node).ToDictionary(g=>g.Key,g=>g.First().Source);
        int omitted=MedialTerminalChains(nominal).Count(c=>c.Edges.SelectMany(e=>projection.EdgeMap[e]).Any(e=>!active.Contains(contact.Edges[e])));
        var output=new List<ReviewedVCarveNativeBranch>();int inspected=0,satisfied=0;
        // Detail omission can turn a former three-way node into an ordinary
        // degree-two interior point. The retained graph owns the real tasks.
        foreach(var chain in MedialTerminalChains(retained))
        {
            token.ThrowIfCancellationRequested();
            var reference=chain.Nodes.Select(i=>retained.Nodes[i]).ToList();
            if(reference.Count<2)continue;
            var nominalEdges=chain.Edges.Select(i=>origins[retained.Edges[i]]).Distinct().ToArray();
            var nominalPoints=chain.Nodes.Where((node,index)=>index==0||index==chain.Nodes.Length-1||nodeOrigins.ContainsKey(node))
                .Select(node=>nodeOrigins.TryGetValue(node,out int original)?nominal.Nodes[original]:retained.Nodes[node]).ToArray();
            var sites=VCarveContactLocus.ContactSites(nominalPoints,boundaries[nominal.Region],owner.ResidualSurfaceDepthBounds);
            double ContactGap(VCarveContactLocus.Pair pair,VCarveContactLocus.Site site)=>site.StockDepthLower>=maximumDepth?0:
                wholeSource.WitnessGapUpper(site.Point,site.StockDepthUpper);
            double gap=sites.Select(s=>Math.Max(ContactGap(s,s.Left),ContactGap(s,s.Right))).DefaultIfEmpty(0).Max();
            inspected++;
            // Selected geometry resolution nominates which corner sources need
            // reconstruction. This observation grants no cutter-clearance proof.
            if(gap<=settings.FitErrorMm){satisfied++;continue;}
            diagnostic?.Invoke($"stage=contact-branch event=construct region={nominal.Region} tip={chain.Nodes[0]} root={chain.Nodes[^1]} observedContactGapMm={gap:R} fitAllowanceMm={settings.FitErrorMm:R} poseBudget={limits.MaximumPoseTrials}");
            var selected=VCarveStockContactSelection.Construct(nominalPoints,reference.ToArray(),boundaries[nominal.Region],
                profile,priorStock!,settings,tolerance,owner,limits,token,diagnostic);
            var native=selected.Native;var guide=selected.Guide;
            string id=Convert.ToHexString(SHA256.HashData(JsonSerializer.SerializeToUtf8Bytes(new{reference,Edges=nominalEdges},new JsonSerializerOptions{IncludeFields=true})));
            var packet=new ReviewedVCarveNativeBranch(id,nominal.Region,reference.ToArray(),guide.ToArray(),native.Moves,native.ErrorBound,nominalEdges);
            output.Add(packet);
            BranchObserver.Value?.Invoke(packet with{SourceReference=packet.SourceReference.ToArray(),ContactGuide=packet.ContactGuide.ToArray(),Moves=packet.Moves.ToArray(),NominalEdges=packet.NominalEdges.ToArray()});
            diagnostic?.Invoke($"stage=contact-branch event=certified region={nominal.Region} identity={id} nativePrimitives={native.Moves.Length} fitBoundMm={native.ErrorBound:R} fullStock=True localRemovalDeviationCertified=True configuredRemovalAllowanceMm={settings.FitErrorMm:R} nominationDomain={selected.Domain} contactRankingIsNotFinishPolicy=True");
        }
        diagnostic?.Invoke($"stage=contact-branch event=complete region={nominal.Region} inspected={inspected} withinConfiguredResolution={satisfied} omittedByDetail={omitted} nativeBranches={output.Count}");
        return output;
    }
}
