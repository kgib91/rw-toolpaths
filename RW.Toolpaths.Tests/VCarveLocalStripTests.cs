using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveLocalStripTests
{
    private static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Hemisphere,0,1,1),new(CarvingCutterSectionKind.Linear,1,2,5)],2);
    private static readonly IReadOnlyList<PointD>[] Walls=[new PointD[]{new(-10,-10),new(30,-10),new(30,30),new(-10,30)},
        new PointD[]{new(0,0),new(0,4),new(2,4),new(2,0)},new PointD[]{new(5,0),new(5,4),new(7,4),new(7,0)}];
    private static TaggedToolpath Path(VCarveRingOrder.Plan plan,int ring,double y,int pass=0)=>new(
        new Point3D[]{new(0,y,-1),new(2,y,-1),new(4,y,-1),new(6,y,-1)},[],0,"clearing",pass,detectCurves:false)
        {ConcentricSource=new(plan,ring),ContinuationSourceOrdinal=ring};
    [Theory]
    [InlineData(VCarveProfileDirection.TopDown)]
    [InlineData(VCarveProfileDirection.BottomUp)]
    public void LocalOffsetAncestryPreservesEveryDirectedChordWithoutWholeWallBatches(VCarveProfileDirection direction)
    {
        var plan=new VCarveRingOrder.Plan([-1,0],levels:[0,1],walls:Walls);
        var source=new[]{Path(plan,0,-.1),Path(plan,1,-.2)};
        var result=VCarveWallSources.LocalStrips(source,Profile,new DepthSchedule(2,2),new(){SmoothRampAngleDegrees=45,StepOver=.45,ProfileDirection=direction},default,Console.WriteLine);
        static (double,double,double,double,double,double)[] Edges(IEnumerable<TaggedToolpath> paths)=>paths.SelectMany(p=>p.Points.Zip(p.Points.Skip(1)))
            .Select(e=>(e.First.X,e.First.Y,e.First.Z,e.Second.X,e.Second.Y,e.Second.Z)).Order().ToArray();
        Assert.Equal(Edges(source),Edges(result));
        var localPlans=result.Select(p=>p.ConcentricSource!.Plan).Distinct().ToArray();Assert.Equal(2,localPlans.Length);
        Assert.All(localPlans,p=>{Assert.Equal(new[]{0,1},p.Levels);Assert.Equal(direction==VCarveProfileDirection.BottomUp,p.InsideOut);});
        Assert.All(result,p=>{Assert.Equal(-1,p.ConcentricSource!.Wall);Assert.Null(p.ContinuationSourceOrdinal);});
    }
    [Fact]
    public void MultipleDepthLayersKeepOriginalSourcePassOwnership()
    {
        var plan=new VCarveRingOrder.Plan([-1],walls:Walls);var source=new[]{Path(plan,0,-.1,0),Path(plan,0,-.1,1)};
        var result=VCarveWallSources.LocalStrips(source,Profile,new DepthSchedule(2,1),new(){SmoothRampAngleDegrees=45,StepOver=.45},default,Console.WriteLine);
        Assert.Equal(source[0],result[0]);Assert.Equal(source[1],result[1]);
    }
    [Fact]
    public void FloorCleanupRetainsItsOwnConstructionPlan()
    {
        var plan=new VCarveRingOrder.Plan([-1,0,-1],levels:[0,1,1000],walls:Walls);var floor=Path(plan,2,10);
        var result=VCarveWallSources.LocalStrips([Path(plan,0,-.1),Path(plan,1,-.2),floor],Profile,new DepthSchedule(2,2),new(){SmoothRampAngleDegrees=45,StepOver=.45},default,Console.WriteLine);
        Assert.Equal(floor,result[^1]);
    }
}
