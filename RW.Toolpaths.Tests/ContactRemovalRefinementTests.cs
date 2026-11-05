using System.IO.Compression;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Xunit;
namespace RW.Toolpaths.Tests;
public sealed class ContactRemovalRefinementTests
{
    [Fact] public void AGuideFitAndNoGougeDoNotReplaceTheIndependentRemovalRequirement()
    {
        var assembly=typeof(ContactRemovalRefinementTests).Assembly;
        using var file=assembly.GetManifestResourceStream(assembly.GetManifestResourceNames().Single(n=>n.EndsWith("contact-removal-refinement.json.gz")))!;
        using var compressed=new GZipStream(file,CompressionMode.Decompress);using var document=JsonDocument.Parse(compressed);
        var d=document.RootElement;var json=new JsonSerializerOptions{IncludeFields=true};
        T Read<T>(JsonElement value)=>JsonSerializer.Deserialize<T>(value,json)!;
        VCarveCutterProfile Profile(JsonElement value)=>new(Read<CarvingCutterSection[]>(value.GetProperty("Sections")),value.GetProperty("MaximumDepth").GetDouble());
        var profile=new VCarveCutterProfile(Read<CarvingCutterSection[]>(d.GetProperty("Profile")),d.GetProperty("MaximumDepth").GetDouble());
        var s=d.GetProperty("Stock");var stock=new VCarveClearedStock(Read<VCarveRoughingChord[]>(s.GetProperty("Chords")),Read<double[]>(s.GetProperty("FloorDepths")),Read<PointD[]>(s.GetProperty("PartBoundary")),
            s.GetProperty("ProfileSweeps").EnumerateArray().Select(v=>new VCarveClearedSweep(Profile(v.GetProperty("Profile")),Read<VCarveRoughingChord[]>(v.GetProperty("Chords")))).ToArray());
        var boundaries=Read<PointD[][][]>(d.GetProperty("Boundaries"));
        var authority=new VCarveMaterialSurface(boundaries.SelectMany(b=>b),stock,profile,profile.MaximumDepth,d.GetProperty("SurfaceZ").GetDouble(),default,null);
        var before=Read<ToolMove[]>(d.GetProperty("RefusedFourCubics"));var after=Read<ToolMove[]>(d.GetProperty("RefinedEightCubics"));var reference=Read<Point3D[]>(d.GetProperty("Reference"));
        double allowance=d.GetProperty("ConfiguredFitToleranceMm").GetDouble();Assert.Equal(.3,allowance);
        Assert.All(before,m=>Assert.True(authority.IsLegalNativeCurve(m)));
        Assert.All(after,m=>{Assert.True(authority.IsLegalNativeCurve(m));Assert.True(VCarveNativeCurvePlanning.MeasureLength(m).Lower>=.3);Assert.InRange(m.MaxError,0,allowance);});
        Assert.False(VCarveNativeCurvePlanning.VerifyRemovalDeviation(reference,before,stock,profile,allowance));
        Assert.True(VCarveNativeCurvePlanning.VerifyRemovalDeviation(reference,after,stock,profile,allowance));
        Assert.False(VCarveNativeCurvePlanning.VerifyRemovalDeviation(reference,[],stock,profile,allowance));
    }
}
