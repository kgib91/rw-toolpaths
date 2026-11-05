using System.IO.Compression;
using System.Reflection;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeContactDepthJoinTests
{
    [Theory]
    [InlineData(11)]
    [InlineData(19)]
    [InlineData(18)]
    public void SplitAxialContactLegHasACompleteMinimumLengthPathWithSharedJoins(int region)
    {
        using var stream=Assembly.GetExecutingAssembly().GetManifestResourceStream($"RW.Toolpaths.Tests.Fixtures.contact-depth-join-{region}.json.gz")!;
        using var gzip=new GZipStream(stream,CompressionMode.Decompress);
        using var document=JsonDocument.Parse(gzip);var data=document.RootElement;
        var json=new JsonSerializerOptions{IncludeFields=true};
        T Read<T>(JsonElement e)=>e.Deserialize<T>(json)!;
        VCarveCutterProfile Profile(JsonElement p)=>new(Read<CarvingCutterSection[]>(p.GetProperty("Sections")),p.GetProperty("MaximumDepth").GetDouble());
        double surface=data.GetProperty("SurfaceZ").GetDouble(),maximum=data.GetProperty("MaximumDepth").GetDouble();
        var profile=new VCarveCutterProfile(Read<CarvingCutterSection[]>(data.GetProperty("Profile")),maximum);
        var saved=data.GetProperty("Stock");
        var stock=new VCarveClearedStock(Read<VCarveRoughingChord[]>(saved.GetProperty("Chords")),Read<double[]>(saved.GetProperty("FloorDepths")),Read<PointD[]>(saved.GetProperty("PartBoundary")),
            saved.GetProperty("ProfileSweeps").EnumerateArray().Select(s=>new VCarveClearedSweep(Profile(s.GetProperty("Profile")),Read<VCarveRoughingChord[]>(s.GetProperty("Chords")))).ToArray());
        var target=Read<PointD[][]>(data.GetProperty("Target"));
        var source=Read<Point3D[]>(data.GetProperty("Points")).Select(p=>p with{Z=p.Z-surface}).ToArray();var original=source.ToArray();
        var knots=Read<int[]>(data.GetProperty("Selected"));
        using var deadline=new CancellationTokenSource(TimeSpan.FromSeconds(30));
        var owner=new VCarveMaterialSurface(target,stock,profile,maximum,surface,deadline.Token,null,false);
        var guard=new VCarveNativeContainment(target,profile,deadline.Token,stockChord:owner.IsLegalNativeChord,stockCurve:owner.IsLegalNativeCurve);
        var incumbent=Enumerable.Range(0,source.Length-1).Select(i=>{
            double length=VCarveNativeCurveMath.Distance(source[i],source[i+1]);
            return new VCarveNativeFit(ToolMove.Line(source[i],source[i+1]),i,i+1,length,length,length,0,0,true);
        }).ToArray();
        var settings=new VCarveNativeFitSettings(.3,.3){PreservePlanarSourceProgress=true};
        var budget=new VCarveNativeSearchBudget(deadline.Token,50000);
        var result=VCarveNativeMinimumPath.Fit(source,knots,incumbent,guard,profile,budget,settings,2048,deadline.Token,out bool exhausted,out _);
        Assert.False(exhausted);Assert.Equal(original,source);Assert.Equal(0,result[0].First);Assert.Equal(source.Length-1,result[^1].Last);
        // All stock support intersecting the source box plus4mm is retained as
        // complete original chords. A certified .3mm fit plus3mm cutter and
        // .002mm posting shell stays inside that captured domain.
        var independent=new VCarveMaterialSurface(target,stock,profile,maximum,surface,deadline.Token,null,false);
        foreach(var fit in result)
        {
            Assert.True(fit.LengthLower>=.3);Assert.False(fit.PreservedShortFeature);
            Assert.Equal(source[fit.First],fit.Move.Start);Assert.Equal(source[fit.Last],fit.Move.End);
            Assert.True(VCarveNativeGeometricProof.Prove(fit.Move,source,fit.First,fit.Last,.3).Certified);
            Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(fit.Move,source,fit.First,fit.Last,out _));
            Assert.True(independent.IsLegalNativeCurve(fit.Move));
        }
        foreach(var (a,b) in result.Zip(result.Skip(1)))
        {
            Assert.Equal(a.Last,b.First);
            var u=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(a.Move,1));
            var v=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(b.Move,0));
            Assert.True(VCarveNativeCurveMath.Norm(VCarveNativeCurveMath.Sub(u,v))<1e-7);
        }
    }
}
