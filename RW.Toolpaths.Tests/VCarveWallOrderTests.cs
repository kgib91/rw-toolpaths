using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveWallOrderTests
{
    private static TaggedToolpath Cut(params Point3D[] points)=>new(points,Array.Empty<ToolpathSpan>(),0,"clearing",0,detectCurves:false);
    [Fact]
    public void MergedContourRetainsBothOriginalWallsWithoutChangingAnyDirectedChord()
    {
        IReadOnlyList<PointD>[] walls=[new PointD[]{new(0,0),new(1,0),new(1,2),new(0,2)},
            new PointD[]{new(4,0),new(5,0),new(5,2),new(4,2)}];
        var plan=new VCarveRingOrder.Plan([-1],walls:walls);
        var original=Cut(new(.5,-.2,-1),new(1.5,-.2,-1),new(3.5,-.2,-1),new(4.5,-.2,-1),
            new(4.5,2.2,-1),new(3.5,2.2,-1),new(1.5,2.2,-1),new(.5,2.2,-1),new(.5,-.2,-1))
            with{ConcentricSource=new(plan,0)};
        var parts=VCarveWallSources.Assign([original],default,Console.WriteLine);
        Assert.Equal(new[]{0,1},parts.Select(p=>p.ConcentricSource!.Wall).Distinct().Order());
        static (Point3D,Point3D)[] Edges(IEnumerable<TaggedToolpath> paths)=>paths.SelectMany(p=>p.Points.Zip(p.Points.Skip(1)))
            .OrderBy(e=>e.First.X).ThenBy(e=>e.First.Y).ThenBy(e=>e.Second.X).ThenBy(e=>e.Second.Y).ToArray();
        Assert.Equal(Edges([original]),Edges(parts));Assert.All(parts,p=>Assert.Same(plan,p.ConcentricSource!.Plan));
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FinishEveryPieceOfTheCurrentWallLevelBeforeItsNextLevelOrAnotherWall(bool bottomUp)
    {
        var plan=new VCarveRingOrder.Plan([-1,0,-1,-1,3],levels:[0,1,0,0,1]);
        var paths=Enumerable.Range(0,5).Select(i=>Cut(new(i>=3?20:.1*i,0,-1),new(i>=3?21:1+.1*i,0,-1))
            with{ConcentricSource=new(plan,i,Wall:i==2?1:0),ContinuationSourceOrdinal=i}).ToArray();
        var order=new VCarveRingOrder(paths,bottomUp,default,.7);
        int[] expected=bottomUp?[1,4,3,0,2]:[0,3,4,1,2];
        for(int step=0;step<expected.Length;step++)
        {
            var ready=order.Candidates(paths[expected[Math.Max(0,step-1)]].Points[^1]);
            Assert.Contains(expected[step],ready);
            if(step>0&&step<4)Assert.DoesNotContain(2,order.IndependentCandidates());
            if(step==1)Assert.Equal(new[]{expected[1]},ready);
            order.Complete(expected[step]);
        }
        Assert.Empty(order.Candidates(null));Assert.Equal(2,order.WallCount);Assert.Equal(3,order.WallLevelBatches);
        Console.WriteLine($"[VCarve.WallOrder.Test] bottomUp={bottomUp} sequence={string.Join(',',expected)} completeWallLevels=True foreignWallDeferred=True");
    }
    [Fact]
    public void IndependentReentryCannotInterruptTheCurrentSourceDepthPasses()
    {
        var plan=new VCarveRingOrder.Plan([-1,0,-1]);
        var first=Cut(new(0,0,-1),new(1,0,-1)) with{ConcentricSource=new(plan,0,Wall:0),ContinuationSourceOrdinal=0};
        var paths=new[]{first,first with{DepthPassIndex=1},
            first with{ConcentricSource=new(plan,1,Wall:0),ContinuationSourceOrdinal=1},
            first with{ConcentricSource=new(plan,2,Wall:1),ContinuationSourceOrdinal=2}};
        var order=new VCarveRingOrder(paths,false,default,.7);
        order.Complete(0);Assert.Equal(new[]{1},order.Candidates(new(0,0,-1)));
        Assert.Equal(new[]{1},order.IndependentCandidates());
        order.Complete(1);Assert.Equal(new[]{2},order.Candidates(new(0,0,-1)));
        order.Complete(2);Assert.Equal(new[]{3},order.IndependentCandidates());
    }
}
