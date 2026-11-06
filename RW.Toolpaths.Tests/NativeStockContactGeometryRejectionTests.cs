using System.IO.Compression;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeStockContactGeometryRejectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AuthenticStockContactKeepsTheExactUnfilteredNativeFit(bool reverse)
    {
        var assembly=typeof(NativeStockContactGeometryRejectionTests).Assembly;
        using var stream=assembly.GetManifestResourceStream(assembly.GetManifestResourceNames().Single(n=>n.EndsWith("stock-contact-branch.json.gz")))!;
        using var gzip=new GZipStream(stream,CompressionMode.Decompress);
        using var document=JsonDocument.Parse(gzip);var data=document.RootElement;
        var json=new JsonSerializerOptions{IncludeFields=true};
        T Read<T>(JsonElement e)=>e.Deserialize<T>(json)!;
        VCarveCutterProfile Cutter(JsonElement e)=>new(Read<CarvingCutterSection[]>(e.GetProperty("Sections")),e.GetProperty("MaximumDepth").GetDouble());
        double surface=data.GetProperty("SurfaceZ").GetDouble(),depth=data.GetProperty("MaximumDepth").GetDouble();
        var profile=new VCarveCutterProfile(Read<CarvingCutterSection[]>(data.GetProperty("Profile")),depth);
        var boundaries=Read<PointD[][][]>(data.GetProperty("Boundaries"));var saved=data.GetProperty("Stock");
        var stock=new VCarveClearedStock(Read<VCarveRoughingChord[]>(saved.GetProperty("Chords")),Read<double[]>(saved.GetProperty("FloorDepths")),
            Read<PointD[]>(saved.GetProperty("PartBoundary")),saved.GetProperty("ProfileSweeps").EnumerateArray()
                .Select(e=>new VCarveClearedSweep(Cutter(e.GetProperty("Profile")),Read<VCarveRoughingChord[]>(e.GetProperty("Chords")))).ToArray());
        var points=Read<Point3D[]>(data.GetProperty("Paths")[0]).Select(p=>p with{Z=p.Z-surface}).ToArray();
        if(reverse)Array.Reverse(points);
        var original=points.ToArray();
        var start=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[1],points[0]));
        var end=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[^1],points[^2]));
        using var deadline=new CancellationTokenSource(TimeSpan.FromSeconds(30));
        VCarveNativeFit? Fit(bool filter)
        {
            var owner=new VCarveMaterialSurface(boundaries.SelectMany(b=>b),stock,profile,depth,surface,deadline.Token,null);
            var guard=new VCarveNativeContainment(boundaries[11],profile,deadline.Token,owner.IsLegalNativeChord,owner.IsLegalNativeCurve);
            return VCarveNativeStockContactFit.Find(points,start,end,guard,profile,new(.3,.3),deadline.Token,out _,useGeometryRejection:filter);
        }
        var unfiltered=Fit(false);var filtered=Fit(true);
        Assert.NotNull(unfiltered);Assert.Equal(unfiltered,filtered);Assert.Equal(original,points);
    }

    [Fact]
    public void ImpossibleSourceExcursionSkipsProofsWithoutChangingCandidatesOrRefusal()
    {
        Point3D[] points=[new(0,0,-.5),new(.2,1,-.5),new(.4,-1,-.5),new(.8,1,-.5),new(1,0,-.5)];
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.05,.05,2)],1);
        var guard=new VCarveNativeContainment([new PointD[]{new(-3,-3),new(3,-3),new(3,3),new(-3,3)}],profile,
            stockChord:(_,_)=>true,stockCurve:_=>true);
        var start=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[1],points[0]));
        var end=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(points[^1],points[^2]));
        var audits=new List<VCarveNativeStockContactFit.SearchAudit>();
        var previous=VCarveNativeStockContactFit.AuditObserver.Value;
        try
        {
            VCarveNativeStockContactFit.AuditObserver.Value=audits.Add;
            var unfiltered=VCarveNativeStockContactFit.Find(points,start,end,guard,profile,new(.01,.3),default,out var originalOutcome,useGeometryRejection:false);
            var filtered=VCarveNativeStockContactFit.Find(points,start,end,guard,profile,new(.01,.3),default,out var filteredOutcome);
            Assert.Null(unfiltered);Assert.Equal(unfiltered,filtered);Assert.Equal(originalOutcome,filteredOutcome);
        }
        finally{VCarveNativeStockContactFit.AuditObserver.Value=previous;}
        Assert.Equal(2,audits.Count);Assert.Equal(audits[0].Candidates,audits[1].Candidates);
        Assert.True(audits[1].GeometryExcluded>0);
        Assert.True(audits[1].GeometryProofs<audits[0].GeometryProofs);
    }
}
