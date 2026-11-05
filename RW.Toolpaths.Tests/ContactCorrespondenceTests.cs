using System.Text.Json;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class ContactCorrespondenceTests
{
    [Fact] public void ContinuousContactFitIsNotLimitedToItsFirstProjectionWitness()
    {
        using var input=typeof(ContactCorrespondenceTests).Assembly.GetManifestResourceStream(
            typeof(ContactCorrespondenceTests).Assembly.GetManifestResourceNames().Single(n=>n.EndsWith("contact-ordered-correspondence.json")))!;
        using var doc=JsonDocument.Parse(input);var json=new JsonSerializerOptions{IncludeFields=true};
        var move=JsonSerializer.Deserialize<ToolMove>(doc.RootElement.GetProperty("Move"),json);
        var source=JsonSerializer.Deserialize<Point3D[]>(doc.RootElement.GetProperty("Reference"),json)!;
        var proof=VCarveNativeGeometricProof.ProveFreeJoin(move,source,0,source.Length-1,.3);
        Assert.True(proof.Certified,$"{proof.Reason}: {proof.ErrorBound:R}");
        Assert.InRange(proof.ErrorBound,0,.3);
        Assert.Equal(0,proof.Parameters[0]);Assert.Equal(1,proof.Parameters[^1]);
        Assert.True(proof.Parameters.Zip(proof.Parameters.Skip(1)).All(p=>p.First<=p.Second));
        Assert.False(VCarveNativeGeometricProof.ProveFreeJoin(move,source,0,source.Length-1,.28).Certified);
        var otherBranch=source.Select(p=>p with{X=p.X+3}).ToArray();
        Assert.False(VCarveNativeGeometricProof.ProveFreeJoin(move,otherBranch,0,otherBranch.Length-1,.3).Certified);
    }
}
