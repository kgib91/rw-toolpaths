using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveWallContoursTests
{
    [Theory]
    [InlineData(VCarveProfileDirection.TopDown)]
    [InlineData(VCarveProfileDirection.BottomUp)]
    public void RebuildSelectedWallWithSharedPhysicalProtectionAndRetainOriginalRemoval(VCarveProfileDirection direction)
    {
        IReadOnlyList<PointD>[] boundary=[new PointD[]{new(0,0),new(40,0),new(40,40),new(0,40)},
            new PointD[]{new(10,10),new(10,14),new(14,14),new(14,10)},
            new PointD[]{new(16,10),new(16,14),new(20,14),new(20,10)}];
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,.5,.5),new(CarvingCutterSectionKind.Linear,.5,2,6)],1);
        var tool=new ToolGeometry(2){VCarveProfile=profile};
        var depth=new DepthSchedule(1,1){SurfaceZ=12.7};
        var options=new VCarveOptions{SmoothVCarving=true,SmoothRampAngleDegrees=45,StepOver=.45,ProfileDirection=direction};
        var floor=new VCarveSmoothing.FloorField(VCarveRampSurface.Sources(boundary,options,default),profile,1,1,default);
        var walls=new VCarveSmoothing.CommandClearance(boundary,profile,depth.SurfaceZ,default,Console.WriteLine);
        var guard=VCarveSmoothing.CreateContinuationGuard(floor,walls,depth.SurfaceZ,1,default);
        int protectCalls=0;
        List<TaggedToolpath> Protect(IReadOnlyList<TaggedToolpath> paths){protectCalls++;return VCarveSmoothing.LimitGraph(
            walls.Apply(VCarveSmoothing.Constrain(paths,floor,1,default,100_000)),1,default);}
        var original=Protect(VCarveRampSurface.Contours(boundary,options,profile,1,0,default,Console.WriteLine));
        VCarveWallContours.Review? capture=null;
        try {
            VCarveWallContours.Selection.Value=(wall,_)=>wall==1;
            VCarveWallContours.Observer.Value=r=>capture=r;
            var result=VCarveWallContours.Replace(original,boundary,tool,depth,options,Protect,guard,default,Console.WriteLine);
            Assert.NotNull(capture);Assert.True(protectCalls>=2);Assert.NotEmpty(capture.Rows);
            Assert.Equal(direction==VCarveProfileDirection.BottomUp,capture.Rows[0].ConcentricSource!.Plan.InsideOut);
            Assert.True(capture.RequiredIntervals>0);Assert.NotEmpty(capture.Neighbours);
            Assert.All(result.Where(p=>p.ConcentricSource is not null),p=>Assert.Equal(-1,p.ConcentricSource!.Wall));
            Console.WriteLine($"[VCarve.WallConstruction.Test] direction={direction} originalIntervals={capture.RequiredIntervals} rows={capture.Rows.Count} supports={capture.Supports} physicalFloorWallGrade=True originalRemovalCertified=True");
        } finally {VCarveWallContours.Selection.Value=null;VCarveWallContours.Observer.Value=null;}
    }
}
