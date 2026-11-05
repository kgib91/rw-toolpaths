using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveAutomaticSpatialSweepTests
{
    private static readonly VCarveOptions Options=new(){SmoothVCarving=true,OptimizeContourConnections=true,
        DetectCurves=false,Motion=new(1000,500,5000,5),SmoothRampAngleDegrees=50,StepOver=.45};
    private static TaggedToolpath[] Sources(bool rotate=false,int count=128)
    {
        var plan=new VCarveRingOrder.Plan(Enumerable.Repeat(-1,count));
        var rings=Enumerable.Range(0,count).Select(i=>new TaggedToolpath(
            rotate?[new(0,2*i,-1),new(0,2*i+.2,-1)]:[new(2*i,0,-1),new(2*i+.2,0,-1)],
            [new(0,1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false)
            {ConcentricSource=new(plan,i),ContinuationSourceOrdinal=i}).ToArray();
        return [..rings,rings[0] with{ConcentricSource=null,ContinuationSourceOrdinal=count}];
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LargeSinglePassRegionsSweepAlongTheirLongAxisInEitherDepthDirection(bool bottomUp)
    {
        foreach(bool rotate in new[]{false,true}){
            var options=Options with{ProfileDirection=bottomUp?VCarveProfileDirection.BottomUp:VCarveProfileDirection.TopDown};
            var selected=VCarvePathRouter.AutomaticSpatialSweep(Sources(rotate),new(2,2),options,default);
            Assert.NotNull(selected);Assert.Equal(64,selected.WindowMm);
            Assert.Equal(rotate?VCarveRingOrder.SpatialAxis.Y:VCarveRingOrder.SpatialAxis.X,selected.Axis);
            Assert.Equal(bottomUp?VCarveProfileDirection.BottomUp:VCarveProfileDirection.TopDown,options.ProfileDirection);
        }
    }

    [Fact]
    public void SmallFewJobMultipleLayerAndNonOptimizedRegionsKeepTheirExistingScheduler()
    {
        var source=Sources();
        Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(Sources(count:127),new(2,2),Options,default));
        var small=source.Select(p=>p with{Points=p.Points.Select(v=>v with{X=v.X/4}).ToArray()}).ToArray();
        Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(small,new(2,2),Options,default));
        Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(source,new(2,1),Options,default));
        Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(source.Select(p=>p with{DepthPassIndex=1}).ToArray(),new(2,2),Options,default));
        Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(source,new(2,2),Options with{SmoothVCarving=false},default));
        Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(source,new(2,2),Options with{OptimizeContourConnections=false},default));
        Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(source,new(2,2),Options with{Motion=null},default));
        Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(source.Select(p=>p with{ConcentricSource=null}).ToArray(),new(2,2),Options,default));
        Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(source.Where(p=>p.ConcentricSource is not null).ToArray(),new(2,2),Options,default));
    }

    [Fact]
    public void AutomaticSelectionRejectsNonfiniteBoundsAndPropagatesCancellation()
    {
        var source=Sources();source[0]=source[0] with{Points=[new(double.NaN,0,-1),new(1,0,-1)]};
        Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(source,new(2,2),Options,default));
        using var cancellation=new CancellationTokenSource();cancellation.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(()=>VCarvePathRouter.AutomaticSpatialSweep(Sources(),new(2,2),Options,cancellation.Token));
    }

    [Fact]
    public void ExplicitLegacyControlRetainsTheExistingScheduler()
    {
        var previous=VCarvePathRouter.AutomaticSpatialSweepOverride.Value;
        try{VCarvePathRouter.AutomaticSpatialSweepOverride.Value=false;
            Assert.Null(VCarvePathRouter.AutomaticSpatialSweep(Sources(),new(2,2),Options,default));
        }finally{VCarvePathRouter.AutomaticSpatialSweepOverride.Value=previous;}
    }

    private sealed class SelectionObserved:Exception;
    [Theory]
    [InlineData(null)]
    [InlineData(false)]
    public void AutomaticPolicyScopesMixedSourceDispatchAndRestoresOverridesOnFailure(bool? explicitPolicy)
    {
        var previousSweep=VCarveRingOrder.SpatialSweepOverride.Value;
        var previousMixed=VCarveRingRouter.SpatialMixedSourcesOverride.Value;
        var previousIndependent=VCarvePathRouter.SpatialIndependentSourcesOverride.Value;
        try{
            VCarveRingOrder.SpatialSweepOverride.Value=null;VCarveRingRouter.SpatialMixedSourcesOverride.Value=explicitPolicy;
            VCarvePathRouter.SpatialIndependentSourcesOverride.Value=explicitPolicy;
            Assert.Throws<SelectionObserved>(()=>VCarvePathRouter.Route(Sources(),Array.Empty<IReadOnlyList<PointD>>(),
                new(1),new(2,2),Options,0,null,diagnostic:line=>{
                    if(!line.Contains("stage=spatial-routing-policy event=selected"))return;
                    Assert.Equal(VCarveRingOrder.SpatialAxis.X,VCarveRingOrder.SpatialSweepOverride.Value!.Axis);
                    Assert.Equal(explicitPolicy??true,VCarveRingRouter.SpatialMixedSourcesOverride.Value);
                    Assert.Equal(explicitPolicy??true,VCarvePathRouter.SpatialIndependentSourcesOverride.Value);
                    throw new SelectionObserved();
                },continuationCutGuard:(_,_)=>false));
            Assert.Null(VCarveRingOrder.SpatialSweepOverride.Value);Assert.Equal(explicitPolicy,VCarveRingRouter.SpatialMixedSourcesOverride.Value);
            Assert.Equal(explicitPolicy,VCarvePathRouter.SpatialIndependentSourcesOverride.Value);
        }finally{
            VCarveRingOrder.SpatialSweepOverride.Value=previousSweep;VCarveRingRouter.SpatialMixedSourcesOverride.Value=previousMixed;
            VCarvePathRouter.SpatialIndependentSourcesOverride.Value=previousIndependent;
        }
    }
}
