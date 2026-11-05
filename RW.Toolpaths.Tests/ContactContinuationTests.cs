using System.Text.Json;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.ReviewedSource;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class ContactContinuationTests
{
    static VCarveContactLocus.Candidate Pose(int station,Point3D p,double gap)=>new(p,p,station,1,gap,2*Math.Max(0,gap));
    static double Length(Point3D start,Point3D end,IReadOnlyList<VCarveContactLocus.Candidate> path)
    {
        var points=new[]{start}.Concat(path.Select(c=>c.CertifiedPose)).Append(end).ToArray();
        return Enumerable.Range(1,points.Length-1).Sum(i=>VCarveNativeCurveMath.Distance(points[i-1],points[i]));
    }
    [Fact] public void LaterWorstGapDoesNotForceAnUnnecessaryLongEarlierPrefix()
    {
        var start=new Point3D(0,0,-1);var end=new Point3D(2,0,-1);
        var detour=Pose(0,new(0,10,-1),0);var direct=Pose(0,start,.1);var required=Pose(1,new(1,0,-1),.2);
        var result=VCarveContactLocus.ChooseCoherent(start,end,[detour,direct,required]);
        Assert.Equal(direct,result[0]);Assert.Equal(required,result[1]);
        Assert.Equal(.2,result.Max(c=>c.WorstContactGap));Assert.Equal(2,Length(start,end,result));
    }
    [Fact] public void ActualCatalogRetainsBestWorstContactWithoutItsArtificialDetour()
    {
        var assembly=typeof(ContactContinuationTests).Assembly;
        using var input=assembly.GetManifestResourceStream(assembly.GetManifestResourceNames().Single(n=>n.EndsWith("contact-coherent-continuation.json")))!;
        using var doc=JsonDocument.Parse(input);var json=new JsonSerializerOptions{IncludeFields=true};
        var start=JsonSerializer.Deserialize<Point3D>(doc.RootElement.GetProperty("Start"),json);
        var end=JsonSerializer.Deserialize<Point3D>(doc.RootElement.GetProperty("End"),json);
        var catalog=JsonSerializer.Deserialize<VCarveContactLocus.Candidate[]>(doc.RootElement.GetProperty("Catalog"),json)!;
        var old=JsonSerializer.Deserialize<VCarveContactLocus.Candidate[]>(doc.RootElement.GetProperty("OldChosen"),json)!;
        var chosen=VCarveContactLocus.ChooseCoherent(start,end,catalog);
        Assert.Equal(old.Max(c=>c.WorstContactGap),chosen.Max(c=>c.WorstContactGap));
        Assert.True(Length(start,end,chosen)<Length(start,end,old)-1);
        Assert.Equal(old.Select(c=>c.SourceStation),chosen.Select(c=>c.SourceStation));
        Point3D Mirror(Point3D p)=>p with{X=-p.X};
        var mirrored=VCarveContactLocus.ChooseCoherent(Mirror(start),Mirror(end),catalog.Select(c=>c with{Nomination=Mirror(c.Nomination),CertifiedPose=Mirror(c.CertifiedPose)}).ToArray());
        Assert.Equal(chosen.Select(c=>Mirror(c.CertifiedPose)),mirrored.Select(c=>c.CertifiedPose));
    }
}
