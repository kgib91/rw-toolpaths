using System.Text.Json;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.ReviewedSource;
using Xunit;
namespace RW.Toolpaths.Tests;

public sealed class ContactFreeJoinTests
{
    static double[] Parameters(Point3D[] points)
    {
        var result=new double[points.Length];for(int i=1;i<points.Length;i++)result[i]=result[i-1]+VCarveNativeCurveMath.Distance(points[i-1],points[i]);
        double total=result[^1];for(int i=0;i<result.Length;i++)result[i]/=total;return result;
    }
    [Fact] public void AFreeJoinMayLieInsideASourceChordWithoutOwningItsSampleTime()
    {
        Point3D[] source=[new(0,0,0),new(2,0,0),new(8,0,0),new(10,0,0)];
        ToolMove[] moves=[ToolMove.Line(source[0],new(5,0,0),0),ToolMove.Line(new(5,0,0),source[^1],0)];
        var map=VCarveContactSourceCorrespondence.Nominate(moves,source,Parameters(source));
        Assert.Equal(new Point3D(5,0,0),map.Reference[map.Indices[1]]);
        Assert.All(source,p=>Assert.Contains(p,map.Reference));
        for(int i=0;i<moves.Length;i++)Assert.True(VCarveNativeGeometricProof.ProveFreeJoin(moves[i],map.Reference,map.Indices[i],map.Indices[i+1],1e-6).Certified);
    }
    [Fact] public void OrderedCorrespondenceCanPauseAcrossABoundedSourceReversal()
    {
        var move=ToolMove.CubicBezier(new(0,0,0),new(10d/3,0,0),new(20d/3,0,0),new(10,0,0),0);
        Point3D[] source=[new(0,0,0),new(6,0,0),new(4,0,0),new(10,0,0)];
        var valid=VCarveNativeGeometricProof.ProveFreeJoin(move,source,0,3,1.1);
        Assert.True(valid.Certified);Assert.InRange(valid.ErrorBound,1,1.1);
        Assert.False(VCarveNativeGeometricProof.ProveFreeJoin(move,source,0,3,.9).Certified);
    }
    [Fact] public void ActualLongBranchesHaveCompleteOrderedNativeCoverageWithoutShortPieces()
    {
        var assembly=typeof(ContactFreeJoinTests).Assembly;
        using var input=assembly.GetManifestResourceStream(assembly.GetManifestResourceNames().Single(n=>n.EndsWith("contact-free-joins.json")))!;
        using var doc=JsonDocument.Parse(input);var json=new JsonSerializerOptions{IncludeFields=true};
        foreach(var branch in doc.RootElement.EnumerateArray())
        {
            var source=JsonSerializer.Deserialize<Point3D[]>(branch.GetProperty("Guide"),json)!;
            var moves=JsonSerializer.Deserialize<ToolMove[]>(branch.GetProperty("Moves"),json)!;
            Assert.Equal(source[0],moves[0].Start);Assert.Equal(source[^1],moves[^1].End);
            var map=VCarveContactSourceCorrespondence.Nominate(moves,source,Parameters(source));
            Assert.All(source,p=>Assert.Contains(p,map.Reference));
            for(int i=0;i<moves.Length;i++)
            {
                Assert.True(VCarveNativeCurveMath.Length(moves[i]).Lower>=.3);
                if(i>0)Assert.Equal(moves[i-1].End,moves[i].Start);
                var proof=VCarveNativeGeometricProof.ProveFreeJoin(moves[i],map.Reference,map.Indices[i],map.Indices[i+1],.3);
                Assert.True(proof.Certified,$"{branch.GetProperty("Region")}, piece {i}: {proof.Reason}");
            }
        }
    }
    [Fact] public void ACornerShortcutAndDifferentBranchStillFail()
    {
        Point3D[] corner=[new(0,0,0),new(0,1,0),new(1,1,0)];
        var diagonal=ToolMove.Line(corner[0],corner[^1],0);
        Assert.False(VCarveNativeGeometricProof.ProveFreeJoin(diagonal,corner,0,2,.3).Certified);
        var other=corner.Select(p=>p with{Z=2}).ToArray();
        Assert.False(VCarveNativeGeometricProof.ProveFreeJoin(diagonal,other,0,2,.3).Certified);
    }
}
