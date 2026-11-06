using Clipper2Lib;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.ReviewedSource;

namespace RW.Toolpaths.Tests;

public sealed class ReviewedVCarveClearanceReuseTests
{
    static PointD[] Box(double left,double bottom,double right,double top)
        =>[new(left,bottom),new(right,bottom),new(right,top),new(left,top)];

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ReusedAuthorityPreservesFullTargetStockAndSurfaceGuards(bool hasStock)
    {
        IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> boundaries=
        [
            [Box(-3,-3,3,3),Box(1,-.5,2,.5).Reverse().ToArray()],
            [Box(4,-2,6,2)]
        ];
        var cutter=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,2,2)],2);
        var depth=new DepthSchedule(2,2){SurfaceZ=6};
        var stock=hasStock?new VCarveClearedStock(
            [new(new(-5,0,-1),new(-5,0,-1),1.4,1.4)],[1],Box(-8,-8,8,8)):null;
        var log=new List<string>();
        var contact=new ReviewedVCarveStockContact(boundaries,cutter,depth,stock,.01,diagnostic:log.Add);
        var graph=new ReviewedVCarveConstructor.Result(0,
            [new(-1,0,-.5),new(-1,.5,-.5)],[new(0,1,"original-source")],1,0,0,0,0);
        var projected=contact.Project(graph).Graph;
        var raw=new[] {new TaggedToolpath(projected.Nodes.Select(p=>p with{Z=p.Z+depth.SurfaceZ}).ToArray(),
            [new(0,projected.Nodes.Length-1,ToolpathSpanKind.Cut)],0,"standard-vcarve",null,detectCurves:false)};
        log.Clear();
        var reused=contact.AttachClearance(raw,log.Add);
        Assert.DoesNotContain(log,message=>message.Contains("event=prepared"));
        if(hasStock)
            Assert.Contains(log,message=>message.Contains("event=clearance-attached reusedAuthority=True")
                &&!message.Contains("previousChordChecks=0 "));
        var fresh=ReviewedVCarvePlan.AttachClearance(raw,boundaries,cutter,depth,stock);

        (Point3D A,Point3D B,bool Expected)[] chords=
        [
            (new(0,-.5,5.5),new(0,.5,5.5),true),
            (new(5,-.5,5.5),new(5,.5,5.5),true), // Complete second target, absent from raw.
            (new(-5,-.1,5.5),new(-5,.1,5.5),hasStock),
            (new(1.5,-.1,5.75),new(1.5,.1,5.75),false), // Protected hole.
            (new(7,0,5.5),new(7,.1,5.5),false),
            (new(9,0,5.5),new(9,.1,5.5),false) // Outside physical Part.
        ];
        foreach(var (a,b,expected) in chords)
        {
            Assert.Equal(expected,fresh.ReplacementChordGuard!(a,b));
            Assert.Equal(expected,reused.ReplacementChordGuard!(a,b));
            Assert.Equal(expected,fresh.NativeReplacementChordGuard!(a,b));
            Assert.Equal(expected,reused.NativeReplacementChordGuard!(a,b));
        }
        (ToolMove Move,bool Expected)[] curves=
        [
            (ToolMove.CubicBezier(new(0,-.5,5.5),new(.1,-.2,5.5),new(.1,.2,5.5),new(0,.5,5.5),0),true),
            (ToolMove.CubicBezier(new(5,-.5,5.5),new(5.1,-.2,5.5),new(5.1,.2,5.5),new(5,.5,5.5),0),true),
            (ToolMove.CubicBezier(new(1.5,-.1,5.75),new(1.5,-.05,5.75),new(1.5,.05,5.75),new(1.5,.1,5.75),0),false)
        ];
        foreach(var (move,expected) in curves)
        {
            Assert.Equal(expected,fresh.NativeReplacementCurveGuard!(move));
            Assert.Equal(expected,reused.NativeReplacementCurveGuard!(move));
        }
        Assert.Same(raw[0].Points,reused.Toolpaths[0].Points);
    }
}
