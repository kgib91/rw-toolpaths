using System.Reflection;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class OutsideEdgePostedClipTests
{
    private static (MillingBoundaryAccess.Resolved Access,ToolGeometry Tool,DepthSchedule Depth,TaggedToolpath Source) Fixture()
    {
        using var stream=Assembly.GetExecutingAssembly().GetManifestResourceStream("RW.Toolpaths.Tests.Fixtures.outside-edge-clipped-ramp.json")!;
        using var json=JsonDocument.Parse(stream);var root=json.RootElement;
        PointD Point(JsonElement p)=>new(p[0].GetDouble(),p[1].GetDouble());
        IReadOnlyList<PointD>[] Rings(string key)=>root.GetProperty(key).EnumerateArray()
            .Select(r=>(IReadOnlyList<PointD>)r.EnumerateArray().Select(Point).ToArray()).ToArray();
        MillingRegion[] Regions(string key)=>PolygonOps.DifferenceToComponents(Rings(key),Array.Empty<IReadOnlyList<PointD>>())
            .Select(c=>new MillingRegion(c.Outer,c.Holes)).ToArray();
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),
            new(CarvingCutterSectionKind.Linear,1.5875,3,20)],21.5875);
        var tool=new ToolGeometry(3){VCarveProfile=profile};
        var depth=new DepthSchedule(6.35,2){SurfaceZ=root.GetProperty("SurfaceZ").GetDouble(),StartDepth=6.35};
        var part=root.GetProperty("part").EnumerateArray().Select(Point).ToArray();
        var access=new MillingBoundaryAccess([],part){AllowedRegions=Regions("allowed")}
            .Resolve(Regions("target"),tool,depth,true,50,0,default);
        var source=new TaggedToolpath(root.GetProperty("source").EnumerateArray()
            .Select(p=>new Point3D(p[0].GetDouble(),p[1].GetDouble(),p[2].GetDouble())).ToArray(),0,"clearing",null);
        return(access,tool,depth,source);
    }
    private static TaggedToolpath[] World(IEnumerable<TaggedToolpath> paths,double surface)
        =>paths.Select(p=>p with{Points=p.Points.Select(v=>v with{Z=v.Z+surface}).ToArray()}).ToArray();

    [Fact]
    public void AProtectedChordCanLoseItsPostedProofAfterContactInterpolation()
    {
        var f=Fixture();
        f.Access.Verify(World([f.Source],f.Depth.SurfaceZ),f.Tool,f.Depth,0,default);
        var clipped=f.Access.Trim([f.Source],f.Tool.VCarveProfile!.PhysicalRadiusAtDepth,default);
        Assert.NotEmpty(clipped);
        Assert.Contains(clipped.SelectMany(p=>p.Points),p=>!f.Source.Points.Contains(p));
        var failure=Assert.Throws<InvalidOperationException>(()=>f.Access.Verify(World(clipped,f.Depth.SurfaceZ),f.Tool,f.Depth,0,default));
        Assert.Contains("intersects protected material",failure.Message);
    }

    [Fact]
    public void WholeContactingProtectedChordsRetainTheOriginalCutAndItsPostedProof()
    {
        var f=Fixture();var inserted=new List<int>();
        var retained=f.Access.Trim([f.Source],f.Tool.VCarveProfile!.PhysicalRadiusAtDepth,default,
            retainContactingChords:true,interpolatedFragments:inserted);
        Assert.Single(retained);Assert.Equal(f.Source.Points,retained[0].Points);Assert.Empty(inserted);
        f.Access.Verify(World(retained,f.Depth.SurfaceZ),f.Tool,f.Depth,0,default);
    }

    [Fact]
    public void WholeChordTargetSelectionStillClipsCentresAndReportsEveryNewPlatformVertex()
    {
        PointD[] box=[new(0,0),new(12,0),new(12,12),new(0,12)];
        var access=new MillingBoundaryAccess([],box).Resolve([new MillingRegion(box)],ToolGeometry.Flat(.3),new(1,1),true,50,0,default);
        var source=new TaggedToolpath([new(-.5,6,-1),new(2,6,-1)],0,"clearing",null);
        var inserted=new List<int>();
        var retained=access.Trim([source],_=>.3,default,retainContactingChords:true,interpolatedFragments:inserted);
        Assert.Single(retained);Assert.Equal(new[]{0},inserted);
        Assert.All(retained.SelectMany(p=>p.Points),p=>Assert.InRange(p.X,0,12));
        Assert.Equal(source.Points[^1],retained[0].Points[^1]);
    }

    [Fact]
    public void CapturedProtectionRepairsInsertedVerticesWithoutDeletingTheCut()
    {
        var f=Fixture();Func<IReadOnlyList<TaggedToolpath>,List<TaggedToolpath>>? protect=null;
        var source=f.Source with{Category="final-carve"};
        var boundary=f.Access.Regions.SelectMany(r=>r.AsRingGroup()).ToArray();
        var options=new VCarveOptions{SmoothVCarving=true,SmoothRampAngleDegrees=50,StepOver=.45};
        VCarveSmoothing.ApplyWithContinuation([source],boundary,f.Tool,f.Depth,options,default,null,
            captureClippedPathProtection:p=>protect=p);
        Assert.NotNull(protect);
        var clipped=f.Access.Trim([f.Source],f.Tool.VCarveProfile!.PhysicalRadiusAtDepth,default);
        var repaired=protect!(clipped);
        Assert.Equal(clipped.Count,repaired.Count);
        foreach(var p in clipped.SelectMany(p=>p.Points))
            Assert.Contains(repaired.SelectMany(r=>r.Points),q=>q.X==p.X&&q.Y==p.Y&&q.Z>=p.Z);
        Assert.True(repaired.SelectMany(p=>p.Points).Min(p=>p.Z)<-.48);
        Assert.InRange(VCarveSmoothing.MaximumGrade(repaired),0,Math.Tan(50*Math.PI/180)+1e-8);
        f.Access.Verify(World(repaired,f.Depth.SurfaceZ),f.Tool,f.Depth,0,default);
    }
}
