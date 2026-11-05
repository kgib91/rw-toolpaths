using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class NativeNominationFamilyTests
{
    static readonly PointD[] Boundary=[new(0,-10),new(10,-10),new(10,10),new(0,10)];
    static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Linear,0,2,2)],2);
    // The near-port reference sample does not grant clearance. A fitted curve
    // may approximate it within the configured bound only if its WHOLE cutter
    // stays in the actual region, including at the exactly tangent start pose.
    static TaggedToolpath Source()=>new([new(1,0,-1),new(.98,.02,-1),new(1.5,.1,-.99),
        new(2.5,.15,-1),new(3,0,-1),new(2.98,.02,-1)], [new(0,5,ToolpathSpanKind.Cut)],0,"contact",0,0,detectCurves:false);

    static IReadOnlyList<VCarveNativeCurvePlan> Run(bool alternate,bool refuse=false,int maximum=50_000,Action<string>? log=null)
    {
        var source=Source();var guard=new VCarveNativeContainment([Boundary],Profile,default);
        bool Chord(Point3D a,Point3D b)=>!refuse&&guard.Legal(a,b);
        bool Curve(ToolMove move)=>!refuse&&VCarveNativeCutterProof.Certified(move,guard,Profile,out _);
        return VCarveNativeCurvePlanning.Nominate([source],new PointD[][][]{[Boundary]},Profile,0,
            limits:new(MaximumCandidateEdges:maximum),diagnostic:log,stockChord:Chord,stockCurve:Curve,
            selectedSourceOrdinals:[[0,1,2,3,4,5]],requiredSourceOrdinals:[[0,5]],
            sourceContract:VCarveNativeSourceContract.MedialPlanarWithStockContactDepth,
            nominationAlternatives:alternate?[new("longer-contact-proposal",new IReadOnlyList<int>[]{[0,2,3,5]})]:null);
    }

    [Fact] public void RejectedProposalDoesNotHideACertifiedAlternative()
    {
        var baseline=Run(false).Single();
        Assert.False(baseline.CandidateVerified,System.Text.Json.JsonSerializer.Serialize(baseline));
        var log=new List<string>();var plan=Run(true,log:log.Add).Single();
        Assert.True(plan.CandidateVerified);Assert.False(plan.BudgetExhausted);
        Assert.All(plan.Primitives,p=>Assert.True(p.LengthLowerMm>=.3));
        Assert.Equal(Source().Points[0],plan.Primitives[0].Move.Start);
        Assert.Equal(Source().Points[^1],plan.Primitives[^1].Move.End);
        Assert.Equal(0,plan.Primitives[0].SourceFirst);Assert.Equal(5,plan.Primitives[^1].SourceLast);
        for(int i=1;i<plan.Primitives.Count;i++)Assert.Equal(plan.Primitives[i-1].SourceLast,plan.Primitives[i].SourceFirst);
        Assert.Contains(log,s=>s.Contains("event=selected family=longer-contact-proposal"));
    }

    [Fact] public void AlternativeCannotBypassTheExistingWholeCutterOwner()
        =>Assert.False(Run(true,refuse:true).Single().CandidateVerified);

    [Fact] public void AlternativeDoesNotRenewTheSearchBudget()
    {
        var plan=Run(true,maximum:1).Single();
        Assert.True(plan.BudgetExhausted);Assert.False(plan.CandidateVerified);
    }

    [Fact] public void AuthoredSpatialTangentsCannotBeReplacedByNominationFamilies()
        =>Assert.Throws<ArgumentException>(()=>VCarveNativeCurvePlanning.Nominate([Source()],new PointD[][][]{[Boundary]},Profile,0,
            selectedSourceOrdinals:[[0,1,2,3,4,5]],nominationAlternatives:[new("other",new IReadOnlyList<int>[]{[0,2,3,5]})]));

    [Fact] public void AlternativeRebasesItsIntervalsInsideTheOriginalVisit()
    {
        var original=Source();var points=new[]{new Point3D(1,-1,-1)}.Concat(original.Points).ToArray();
        var main=new TaggedToolpath(points,[new(0,6,ToolpathSpanKind.Cut)],0,"main",0,0,detectCurves:false);
        var side=new TaggedToolpath([original.Points[0],new(2,-1,-1)],[new(0,1,ToolpathSpanKind.Cut)],0,"side",0,1,detectCurves:false);
        var guard=new VCarveNativeContainment([Boundary],Profile,default);
        var plans=VCarveNativeCurvePlanning.Nominate([main,side],new PointD[][][]{[Boundary]},Profile,0,
            stockChord:guard.Legal,stockCurve:m=>VCarveNativeCutterProof.Certified(m,guard,Profile,out _),
            selectedSourceOrdinals:[[0,1,2,3,4,5,6],[0,1]],requiredSourceOrdinals:[[0,1,6],[0,1]],
            sourceContract:VCarveNativeSourceContract.MedialPlanarWithStockContactDepth,
            nominationAlternatives:[new("branch",new IReadOnlyList<int>[] {[0,1,3,4,6],[0,1]})]);
        Assert.All(plans,p=>Assert.True(p.CandidateVerified));
        var first=plans[0];Assert.Equal(0,first.Primitives[0].SourceFirst);Assert.Equal(6,first.Primitives[^1].SourceLast);
        Assert.Equal(6,first.Primitives.Sum(p=>p.SourceLast-p.SourceFirst));
        for(int i=1;i<first.Primitives.Count;i++)Assert.Equal(first.Primitives[i-1].SourceLast,first.Primitives[i].SourceFirst);
    }

    [Fact] public void PrivateAlternativeDoesNotReplaceTheOperationEvidenceSnapshot()
    {
        int evidence=0,candidates=0,regions=0;
        var oldSource=VCarveNativeCurvePlanning.EvidenceObserver.Value;
        var oldCandidate=VCarveNativeCurvePlanning.CandidateObserver.Value;
        var oldRegion=VCarveNativeCurvePlanning.RegionObserver.Value;
        try
        {
            VCarveNativeCurvePlanning.EvidenceObserver.Value=(s,_,_,_)=>{evidence++;Assert.Equal(6,s.Single().Points.Count);};
            VCarveNativeCurvePlanning.CandidateObserver.Value=_=>candidates++;
            VCarveNativeCurvePlanning.RegionObserver.Value=(_,_)=>regions++;
            Assert.True(Run(true).Single().CandidateVerified);
            Assert.Equal(1,evidence);Assert.Equal(1,candidates);Assert.Equal(1,regions);
        }
        finally
        {
            VCarveNativeCurvePlanning.EvidenceObserver.Value=oldSource;
            VCarveNativeCurvePlanning.CandidateObserver.Value=oldCandidate;
            VCarveNativeCurvePlanning.RegionObserver.Value=oldRegion;
        }
    }
}
