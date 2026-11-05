using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveFeatureBandOrderTests
{
    private static TaggedToolpath Cut(VCarveRingOrder.Plan plan,int ring,int wall,int owner,double x,double y=0,int pass=0)
        =>new(new Point3D[]{new(x,y,-1),new(x+1,y,-1)},[],0,"clearing",pass,detectCurves:false)
            {ConcentricSource=new(plan,ring,Wall:wall),ContinuationSourceOrdinal=owner};

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EveryFragmentOfTheCurrentFeatureBandPrecedesItsNextBandAndOtherFeatures(bool bottomUp)
    {
        var feature=new VCarveRingOrder.Plan([-1,0],levels:[0,1]);
        var other=new VCarveRingOrder.Plan([-1],levels:[0]);
        var paths=new[]{Cut(feature,0,7,0,0),Cut(feature,0,7,1,20),
            Cut(feature,1,7,2,0,-.1),Cut(feature,1,7,3,20,-.1),Cut(other,0,8,4,.2)};
        var order=new VCarveRingOrder(paths,bottomUp,default,.7);
        int first=bottomUp?2:0,remaining=bottomUp?3:1,next=bottomUp?0:2,nextOther=bottomUp?1:3;

        order.Complete(first);
        // Being much closer to another feature cannot widen an independent
        // reentry beyond the unfinished current band's distant fragment.
        Assert.Equal(new[]{remaining},order.Candidates(paths[4].Points[0]));
        Assert.Equal(new[]{remaining},order.IndependentCandidates());
        Assert.False(order.IsReady(next));Assert.False(order.IsReady(nextOther));

        order.Complete(remaining);
        Assert.Equal(new[]{next,nextOther},order.Candidates(paths[4].Points[0]));
        Assert.Equal(new[]{next,nextOther},order.IndependentCandidates());
        order.Complete(next);Assert.Equal(new[]{nextOther},order.IndependentCandidates());
        order.Complete(nextOther);
        Assert.Equal(new[]{4},order.IndependentCandidates());
        Assert.Equal(2,order.WallCount);Assert.Equal(3,order.WallLevelBatches);
    }

    [Fact]
    public void AnUnclassifiedLocalStripMayAdvanceBesideOneFragmentBeforeAnotherFragmentOfItsBand()
    {
        var feature=new VCarveRingOrder.Plan([-1,0],levels:[0,1]);
        var other=new VCarveRingOrder.Plan([-1],levels:[0]);
        var paths=new[]{Cut(feature,0,-1,0,0),Cut(feature,0,-1,1,20),
            Cut(feature,1,-1,2,0,-.1),Cut(other,0,-1,3,.2)};
        var order=new VCarveRingOrder(paths,false,default,.7);
        order.Complete(0);

        // Legacy local progression follows the adjacent child even while a
        // spatially separate fragment of the shallower band remains unfinished.
        Assert.Equal(new[]{2},order.Candidates(paths[0].Points[^1]));
        Assert.Contains(1,order.IndependentCandidates());
        Assert.Contains(2,order.IndependentCandidates());
        Assert.Contains(3,order.IndependentCandidates());
        Assert.Equal(0,order.WallLevelBatches);
    }

    [Fact]
    public void ASeparateFloorPlanIsInitiallyIndependentButCannotInterruptAnActiveWallFamily()
    {
        var original=new VCarveRingOrder.Plan([-1,0,1],levels:[0,1,1000]);
        var local=new VCarveRingOrder.Plan([-1,0],levels:[0,1]);
        var paths=new[]{Cut(local,0,7,0,0),Cut(local,1,7,1,0,-.1),Cut(original,2,-1,2,0,-.2)};
        var order=new VCarveRingOrder(paths,false,default,.7);

        // Localizing slope rows alone does not invent a cross-plan predecessor
        // for floor cleanup. This explicitly limits the feature-band policy.
        Assert.True(order.IsReady(2));Assert.Contains(2,order.IndependentCandidates());
        order.Complete(0);
        Assert.Equal(new[]{1},order.IndependentCandidates());
        Assert.DoesNotContain(2,order.Candidates(paths[2].Points[0]));
        order.Complete(1);
        Assert.Equal(new[]{2},order.IndependentCandidates());
    }

    [Fact]
    public void FeatureBandReadinessCannotSkipTheCurrentSourcesPhysicalDepthPass()
    {
        var feature=new VCarveRingOrder.Plan([-1,0],levels:[0,1]);
        var first=Cut(feature,0,7,0,0);
        var paths=new[]{first,first with{DepthPassIndex=1},Cut(feature,1,7,1,0,-.1)};
        var order=new VCarveRingOrder(paths,false,default,.7);
        order.Complete(0);
        Assert.Equal(new[]{1},order.Candidates(paths[2].Points[0]));
        Assert.Equal(new[]{1},order.IndependentCandidates());Assert.False(order.IsReady(2));
        order.Complete(1);Assert.Equal(new[]{2},order.IndependentCandidates());
    }
}
