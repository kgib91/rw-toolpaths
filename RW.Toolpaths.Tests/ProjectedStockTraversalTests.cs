using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class ProjectedStockTraversalTests
{
    static PointD[] Rectangle(double extent)=>[new(-extent,-extent),new(extent,-extent),new(extent,extent),new(-extent,extent)];

    [Fact]
    public void EqualDepthSpatialOrderingMatchesIndependentEveryPoseEnvelope()
    {
        var cone=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,6)],6);
        var poses=Enumerable.Range(0,400).Select(i=>new Point3D((i%40-20)*.5,(i/40-5)*.5,-6)).ToArray();
        var sweeps=new[]{new VCarveClearedSweep(cone,poses.Select(p=>new VCarveRoughingChord(p,p,0)).ToArray())};
        var ordered=new VCarveProjectedStock([],[],Rectangle(40),cone,6,default,sweeps,spatialDepthTies:true);
        var control=new VCarveProjectedStock([],[],Rectangle(40),cone,6,default,sweeps,spatialDepthTies:false);
        var random=new Random(779211);
        foreach(bool reserved in new[]{false,true})
        for(int i=0;i<200;i++)
        {
            double x=random.NextDouble()*22-11,y=random.NextDouble()*8-4,radius=random.NextDouble()*.3;
            double extra=radius+(reserved?PostedSweepInnerRegion.Reserve:0);
            double expected=poses.Max(p=>Math.Max(0,6-2*(Math.Sqrt((x-p.X)*(x-p.X)+(y-p.Y)*(y-p.Y))+extra)));
            double actual=ordered.ClearedDepth(x,y,radius,reserved);
            Assert.InRange(actual,expected-1e-12,expected+1e-12);
            Assert.Equal(control.ClearedDepth(x,y,radius,reserved),actual);
        }
        Assert.True(ordered.SpatialDepthTieReorders>0);
        Assert.True(ordered.ColumnTests<control.ColumnTests,$"ordered={ordered.ColumnTests} control={control.ColumnTests}");
        Assert.True(ordered.ColumnNodeVisits<control.ColumnNodeVisits);
        var worker=ordered.CreateQueryWorker();
        Assert.Equal(ordered.ClearedDepth(-9,0,.2),worker.ClearedDepth(-9,0,.2));
        Assert.True(worker.SpatialDepthTieTrials>0);
    }

    [Fact]
    public void MixedDepthAndProfileTreesRetainBothLiteralQueryEnvelopes()
    {
        var ball=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),new(CarvingCutterSectionKind.Linear,1,1,9)],10);
        var cone=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,6)],6);
        var random=new Random(998172);
        Point3D Pose()=>new(Math.Round(random.NextDouble()*16-8,3),Math.Round(random.NextDouble()*16-8,3),-Math.Round(random.NextDouble()*5+.5,3));
        var sweeps=new[]{ball,cone}.Select(profile=>new VCarveClearedSweep(profile,
            Enumerable.Range(0,130).Select(_=>new VCarveRoughingChord(Pose(),Pose(),0)).ToArray())).ToArray();
        var columns=Enumerable.Range(0,30).Select(_=>new VCarveRoughingChord(Pose(),Pose(),.8,.8)).ToArray();
        var ordered=new VCarveProjectedStock(columns,[],Rectangle(40),ball,6,default,sweeps,spatialDepthTies:true);
        var control=new VCarveProjectedStock(columns,[],Rectangle(40),ball,6,default,sweeps,spatialDepthTies:false);
        foreach(bool reserved in new[]{false,true})
        for(int i=0;i<300;i++)
        {
            double x=random.NextDouble()*20-10,y=random.NextDouble()*20-10,radius=random.NextDouble()*.6;
            Assert.Equal(control.ClearedDepth(x,y,radius,reserved),ordered.ClearedDepth(x,y,radius,reserved));
        }
        Assert.Equal(3,ordered.ProfileForests);
    }
}
