using RW.Toolpaths;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.ReviewedSource;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class NativeTopologyAndTaskTests
{
    static TaggedToolpath Path(params Point3D[] p)=>new(p,[new(0,p.Length-1,ToolpathSpanKind.Cut)],0,"topology",0,detectCurves:false);
    [Fact] public void RemovingANativeBranchDoesNotChangeCanonicalJunctionOwnership()
    {
        Point3D a=new(-2,0,-1),joint=new(0,0,-1),b=new(2,0,-1),c=new(0,2,-1);
        var source=Path(a,joint,b,joint,c);var remainder=Path(a,joint,b);
        var full=VCarveNativeTopologyContext.Create([source]);var partial=VCarveNativeTopologyContext.Create([remainder]);
        Assert.Equal(3,full.Degree(joint));Assert.Equal(2,partial.Degree(joint));
        Assert.False(full.IsVisitPort(joint));Assert.Equal(5,full.SourcePointCount);
        Assert.Throws<ArgumentException>(()=>full.Degree(new(90,90,90)));
    }
    [Fact] public void CanonicalSourceBudgetCannotBeResetByFittingASubset()
    {
        var full=VCarveNativeTopologyContext.Create([Path(new(0,0,-1),new(1,0,-1),new(2,0,-1),new(3,0,-1),new(4,0,-1))]);
        var subset=Path(new(0,0,-1),new(1,0,-1));
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,1,1,10)],2);
        var plans=VCarveNativeCurvePlanning.Nominate([subset],[],profile,0,limits:new(MaximumSourcePoints:4),canonicalTopology:full);
        Assert.True(Assert.Single(plans).BudgetExhausted);Assert.False(plans[0].CandidateVerified);
    }
    static ReviewedVCarveConstructor.Result ThreeArms(double thirdLength)=>new(0,
        [new(0,0,-1),new(-2,0,-1),new(2,0,-1),new(0,thirdLength,-1)],
        [new(0,1,"medial:a"),new(0,2,"medial:b"),new(0,3,"medial:c")],3,0,0,0,0);
    [Fact] public void DetailOmissionMakesTheFormerJunctionAnInteriorPointOfOneTask()
    {
        var original=ThreeArms(.05);var retained=ReviewedVCarveDetailResolution.Apply(original,.3).Source;
        var task=Assert.Single(ReviewedVCarveStockContact.MedialTerminalChains(retained));
        Assert.Equal(3,original.Edges.Length);Assert.Equal(2,retained.Edges.Length);
        Assert.Equal(3,task.Nodes.Length);Assert.Equal(0,task.Nodes[1]);
        Assert.NotEqual(0,task.Nodes[0]);Assert.NotEqual(0,task.Nodes[^1]);Assert.Equal(1,task.RootDegree);
    }
    [Fact] public void ARealThreeWayJunctionAndDepthTasksRemainSeparate()
    {
        var original=ThreeArms(2);var tasks=ReviewedVCarveStockContact.MedialTerminalChains(original);
        Assert.Equal(3,tasks.Length);Assert.All(tasks,t=>{Assert.Equal(0,t.Nodes[^1]);Assert.Equal(3,t.RootDegree);});
        Assert.Equal(3,tasks.SelectMany(t=>t.Edges).Distinct().Count());
        var depthTask=original with{Edges=[new(0,1,"depth-cap:a")]};
        Assert.Empty(ReviewedVCarveStockContact.MedialTerminalChains(depthTask));
    }
}
