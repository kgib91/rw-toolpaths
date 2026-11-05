using System.Text.Json;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativePlanarProgressTests
{
    sealed record Captured(Point3D[] Source,Point3D Start,Point3D Control1,Point3D Control2,Point3D End);

    [Fact] public void ControlPointsAreConstructedInsideTheForwardTravelDomain()
    {
        Point3D[] source=[new(0,0,0),new(.25,.5,-1.5),new(.5,1,-3)];
        var domain=VCarveNativePlanarProgress.Create(source,0,2);var random=new Random(1031);
        for(int trial=0;trial<500;trial++)
        {
            var u=VCarveNativeCurveMath.Unit(new(random.NextDouble()+.01,random.NextDouble()+.01,-random.NextDouble()));
            var v=VCarveNativeCurveMath.Unit(new(random.NextDouble()+.01,random.NextDouble()+.01,-random.NextDouble()));
            double alpha=Math.Pow(10,random.NextDouble()*4-2),beta=Math.Pow(10,random.NextDouble()*4-2);
            Assert.True(domain.TrySelectHandles(source[0],source[2],u,v,ref alpha,ref beta));
            var move=ToolMove.CubicBezier(source[0],VCarveNativeCurveMath.Add(source[0],VCarveNativeCurveMath.Mul(u,alpha)),
                VCarveNativeCurveMath.Sub(source[2],VCarveNativeCurveMath.Mul(v,beta)),source[2],0);
            Assert.True(domain.Preserved(move,out string reason),reason);
            var prior=move.Start;
            for(int i=1;i<=128;i++)
            {
                var next=VCarveNativeCurveMath.At(move,i/128d);
                Assert.True(next.X>=prior.X-1e-9&&next.Y>=prior.Y-1e-9);prior=next;
            }
        }
    }

    [Fact] public void FeasibleHandlesArePreservedExactly()
    {
        Point3D[] source=[new(0,0,0),new(1,0,-1)];
        double alpha=.2,beta=.2;var direction=VCarveNativeCurveMath.Unit(new(1,0,-1));
        Assert.True(VCarveNativePlanarProgress.Create(source,0,1).TrySelectHandles(source[0],source[1],direction,direction,ref alpha,ref beta));
        Assert.Equal(.2,alpha);Assert.Equal(.2,beta);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapturedSliverFoldsPassThreeDimensionalFitButFailPlanarOwnership(bool reverse)
    {
        using var stream=typeof(NativePlanarProgressTests).Assembly.GetManifestResourceStream("RW.Toolpaths.Tests.Fixtures.stock-contact-planar-fold.json")!;
        var cases=JsonSerializer.Deserialize<Captured[]>(stream,new JsonSerializerOptions{IncludeFields=true})!;
        Assert.Equal(2,cases.Length);
        foreach(var item in cases)
        {
            var move=ToolMove.CubicBezier(item.Start,item.Control1,item.Control2,item.End,0);
            var source=item.Source;
            if(reverse){move=VCarveNativeCurveMath.Reverse(move);source=source.Reverse().ToArray();}
            Assert.True(VCarveNativeGeometricProof.Prove(move,source,0,source.Length-1,.3).Certified);
            Assert.False(VCarveNativeCurvePlanning.PreservesPlanarProgress(move,source,0,source.Length-1,out var reason));
            Assert.StartsWith("source-planar-progress-reversal",reason);
        }
    }

    [Fact] public void InteriorReversalIsDetectedDespiteForwardEndpointDirections()
    {
        Point3D[] source=[new(0,0,0),new(.25,0,-1.5),new(.5,0,-3)];
        var move=ToolMove.CubicBezier(source[0],new(1,0,-1),new(-.5,0,-2),source[^1],0);
        Assert.True(move.Control1.X>move.Start.X);Assert.True(move.End.X>move.Control2.X);
        Assert.True(VCarveNativeGeometricProof.Prove(move,source,0,2,.3).Certified);
        Assert.False(VCarveNativeCurvePlanning.PreservesPlanarProgress(move,source,0,2,out _));
    }

    [Fact] public void GenuineSourceTurnsAreRetained()
    {
        var move=ToolMove.CubicBezier(new(0,0,-1),new(3,3,-1),new(-2,3,-1),new(1,0,-1),0);
        var source=Enumerable.Range(0,129).Select(i=>VCarveNativeCurveMath.At(move,i/128d)).ToArray();
        Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(move,source,0,128,out _));
    }

    [Fact] public void PureVerticalContactDoesNotNeedASidewaysBow()
    {
        Point3D[] source=[new(2,3,0),new(2,3,-4)];
        var vertical=ToolMove.CubicBezier(source[0],new(2,3,-1),new(2,3,-3),source[1],0);
        Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(vertical,source,0,1,out _));
        var bowed=ToolMove.CubicBezier(source[0],new(2.01,3,-1),new(2.01,3,-3),source[1],0);
        Assert.True(VCarveNativeGeometricProof.Prove(bowed,source,0,1,.3).Certified);
        Assert.False(VCarveNativeCurvePlanning.PreservesPlanarProgress(bowed,source,0,1,out var reason));
        Assert.Equal("source-constant-planar-axis-X",reason);
    }
}
