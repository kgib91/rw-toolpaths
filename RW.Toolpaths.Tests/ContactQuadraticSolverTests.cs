using System.Text.Json;
using RW.Toolpaths.Milling.ReviewedSource;
using Xunit;
namespace RW.Toolpaths.Tests;

public sealed class ContactQuadraticSolverTests
{
    [Fact] public void ActualDegenerateActiveSetsReachTheIndependentlyVerifiedOptimum()
    {
        var assembly=typeof(ContactQuadraticSolverTests).Assembly;
        using var input=assembly.GetManifestResourceStream(assembly.GetManifestResourceNames().Single(n=>n.EndsWith("contact-qp-degenerate.json")))!;
        using var document=JsonDocument.Parse(input);
        foreach(var item in document.RootElement.EnumerateArray())
        {
            var audit=item.GetProperty("audit");
            var inverse=audit.GetProperty("Inverse").EnumerateArray().Select(r=>r.EnumerateArray().Select(v=>v.GetDouble()).ToArray()).ToArray();
            var metric=new double[inverse.Length,inverse.Length];for(int i=0;i<inverse.Length;i++)for(int j=0;j<inverse.Length;j++)metric[i,j]=inverse[i][j];
            var optimum=audit.GetProperty("Optimum").EnumerateArray().Select(v=>v.GetDouble()).ToArray();
            var constraints=audit.GetProperty("Constraints").EnumerateArray().Select(r=>new VCarveContactNativeConstruction.LinearConstraint(r.GetProperty("A").EnumerateArray().Select(v=>v.GetDouble()).ToArray(),r.GetProperty("Bound").GetDouble())).ToArray();
            var result=VCarveContactNativeConstruction.SolveContactQp(metric,optimum,constraints);
            var expected=item.GetProperty("independent").GetProperty("values").EnumerateArray().Select(v=>v.GetDouble()).ToArray();
            Assert.True(result.Converged,$"region {item.GetProperty("region")}: {result.Iterations} iterations");
            Assert.InRange(result.MaximumViolation, -1e-8,1e-8);
            Assert.All(result.Values.Zip(expected),p=>Assert.InRange(Math.Abs(p.First-p.Second),0,1e-7));
        }
    }

    [Fact] public void DuplicatedContactConstraintsDoNotChangeTheNearestFeasiblePoint()
    {
        double[,] inverse={{1d,0},{0,1d}};
        var result=VCarveContactNativeConstruction.SolveContactQp(inverse,[-2,-3],
            [new([1,0],0),new([2,0],0),new([0,1],0),new([0,3],0),new([-1,0],-1)]);
        Assert.True(result.Converged);Assert.All(result.Values,v=>Assert.InRange(Math.Abs(v),0,1e-10));
    }

    [Fact] public void AnInfeasibleContactAndDepthPairStillRefuses()
    {
        double[,] inverse={{1d}};
        var result=VCarveContactNativeConstruction.SolveContactQp(inverse,[0],[new([1],1),new([-1],0)]);
        Assert.False(result.Converged);
    }
}
