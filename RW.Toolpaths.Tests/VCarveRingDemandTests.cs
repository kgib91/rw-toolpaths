using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveRingDemandTests
{
    private static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Hemisphere,0,1,1),new(CarvingCutterSectionKind.Linear,1,1,10)],5);
    private static TaggedToolpath Cut(double z)=>new([new(0,0,z),new(10,0,z)],[],0,"clearing",0,detectCurves:false);

    [Theory]
    [InlineData(-.001,true)]
    [InlineData(-.021,true)]
    [InlineData(-.022,false)]
    [InlineData(-1,false)]
    public void OnlyEntireSweepsInsideTheExistingFinishAllowanceAreOptional(double z,bool optional)
    {
        var source=new[]{Cut(z)};var result=VCarveRingDemand.Required(source,Profile,0,0,.03,default,Console.WriteLine);
        Assert.Equal(optional?0:1,result.Length);Assert.Equal(z,source[0].Points[0].Z);
    }

    [Fact]
    public void PostedRoundingMustPassIndependentlyOfNominalDepth()
    {
        // .020505 is nominally inside .029/sqrt(2), but posts to .021.
        Assert.Single(VCarveRingDemand.Required([Cut(-.020505)],Profile,0,0,.029,default,null));
    }

    [Fact]
    public void AStartingFloorCannotReplaceTheOriginalMaterialTop()
    {
        Assert.Single(VCarveRingDemand.Required([Cut(-.001)],Profile,0,5,.03,default,null));
        Assert.Empty(VCarveRingDemand.Required([Cut(4.999)],Profile,0,5,.03,default,null));
    }

    [Fact]
    public void NoncutMotionOwnershipAndDeeperPartsAreRetained()
    {
        var link=Cut(-.001) with{Spans=[new(0,1,ToolpathSpanKind.Link)]};
        var deeper=new TaggedToolpath([new(0,0,-.001),new(1,0,-1),new(2,0,-.001)],[],0,"clearing",0,detectCurves:false);
        Assert.Equal(new[]{link,deeper},VCarveRingDemand.Required([link,deeper],Profile,0,0,.03,default,null));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AnEntirelyOptionalFamilyNeedsNoCarrierCircuitOrEntryOnlyUnderTheOptInPolicy(bool optimize)
    {
        var plan=new VCarveRingOrder.Plan([-1]);var path=Cut(-.001) with{ConcentricSource=new(plan,0)};
        var result=VCarvePathRouter.Route([path],[],new ToolGeometry(1){VCarveProfile=Profile},new DepthSchedule(5,1),
            new VCarveOptions{SmoothVCarving=true,OptimizeContourConnections=optimize,Motion=new(1000,500,5000,5,0)},0,null,
            continuationCutGuard:(_,_)=>true);
        if(optimize)Assert.Empty(result);else Assert.Single(result);
    }
}
