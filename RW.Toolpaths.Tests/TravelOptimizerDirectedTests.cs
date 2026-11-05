using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class TravelOptimizerDirectedTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(7)]
    [InlineData(13)]
    public void DirectedCostsAndPrerequisitesMatchExhaustiveSmallTours(int seed)
    {
        const int n=6;var random=new Random(seed);
        var costs=Enumerable.Range(0,n*n).Select(_=>(double)random.Next(1,100)).ToArray();
        var starts=Enumerable.Range(0,n).Select(_=>(double)random.Next(0,50)).ToArray();
        var ends=Enumerable.Range(0,n).Select(_=>(double)random.Next(0,50)).ToArray();
        int[] prerequisites=[0,0,1<<0,1<<1,(1<<0)|(1<<2),1<<3];
        double Score(int[] order)=>starts[order[0]]+ends[order[^1]]+
            order.Zip(order.Skip(1)).Sum(p=>costs[p.First*n+p.Second]);
        double minimum=double.PositiveInfinity;int tours=0;var path=new List<int>();
        void Enumerate(int mask){
            if(path.Count==n){minimum=Math.Min(minimum,Score(path.ToArray()));tours++;return;}
            for(int i=0;i<n;i++)if((mask&(1<<i))==0&&(mask&prerequisites[i])==prerequisites[i]){
                path.Add(i);Enumerate(mask|(1<<i));path.RemoveAt(path.Count-1);
            }
        }
        Enumerate(0);Assert.True(tours>1);
        var order=TravelOptimizer.OrderDirected(costs,starts,ends,prerequisites,default);
        Assert.Equal(minimum,Score(order),9);Assert.Equal(n,order.Distinct().Count());
        int seen=0;foreach(int i in order){Assert.Equal(prerequisites[i],seen&prerequisites[i]);seen|=1<<i;}
        Console.WriteLine($"[Travel.Directed.Test] seed={seed} exhaustiveTours={tours} minimum={minimum:R} actual={Score(order):R}");
    }

    [Fact]
    public void DirectionAndExitCostsCanDefeatNearestNeighbour()
    {
        double[] costs=[0,1,2,50,0,100,100,1,0];
        Assert.Equal(new[]{0,2,1},TravelOptimizer.OrderDirected(costs,[0,20,20],[0,0,0],[0,0,0],default));
        Assert.Equal(new[]{0,1,2},TravelOptimizer.OrderDirected(costs,[0,20,20],[0,200,0],[0,1<<0,1<<1],default));
    }

    [Fact]
    public void CyclesBoundsInvalidCostsAndCancellationAreExplicit()
    {
        Assert.Throws<InvalidOperationException>(()=>TravelOptimizer.OrderDirected([0,1,1,0],[0,0],[0,0],[2,1],default));
        Assert.Throws<ArgumentException>(()=>TravelOptimizer.OrderDirected(new double[17*17],new double[17],new double[17],new int[17],default));
        Assert.Throws<ArgumentException>(()=>TravelOptimizer.OrderDirected([double.NaN],[0],[0],[0],default));
        Assert.Throws<ArgumentException>(()=>TravelOptimizer.OrderDirected([0],[0],[0],[1],default));
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        Assert.Throws<OperationCanceledException>(()=>TravelOptimizer.OrderDirected([],[],[],[],cancel.Token));
    }

    [Fact]
    public void ChainTourKeepsLiteralCutsEntriesReturnsAndTheirDependencies()
    {
        TaggedToolpath Cut(params Point3D[] p)=>new(p,[new(0,p.Length-1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);
        var paths=new List<TaggedToolpath>{Cut(new(100,0,-1),new(100.5,0,-1)),
            Cut(new(100.5,0,-1),new(101,0,-1)),Cut(new(0,0,-1),new(.5,0,-1)),Cut(new(102,0,-1),new(102.5,0,-1))};
        var motion=new MillingMotionSettings(1000,500,5000,5,0);var cost=new ToolpathMotionCost(motion,0);
        var optimized=VCarveRingTour.Improve(paths,[0,1,2,3],new[]{(0,1),(1,3)},cost,motion,null,default,Console.WriteLine);
        Assert.True(cost.SequenceSeconds(null,optimized,null)<cost.SequenceSeconds(null,paths,null));
        Assert.Equal(paths.Count,optimized.Count);Assert.All(paths,p=>Assert.Contains(p,optimized));
        Assert.Equal(optimized.IndexOf(paths[0])+1,optimized.IndexOf(paths[1]));
        Assert.True(optimized.IndexOf(paths[1])<optimized.IndexOf(paths[3]));
        Assert.Equal(4,paths.Count);Assert.Equal(100,paths[0].Points[0].X);
    }
}
