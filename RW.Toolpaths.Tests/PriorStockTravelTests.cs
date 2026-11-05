using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PriorStockTravelTests
{
    private static readonly VCarveCutterProfile Cylinder=new(
        [new(CarvingCutterSectionKind.Linear,.5,.5,5)],5);

    private static Func<Point3D,Point3D,bool> Guard(IEnumerable<VCarveRoughingChord> chords,
        VCarveCutterProfile? profile=null)
    {
        var options=new VCarveOptions{CompactCollinearCommands=true,RoughingRestChords=chords.ToArray()}
            .WithPriorStock(default);
        return VCarveContinuationAdapter.CreatePriorStockTravelGuard(options,profile??Cylinder,10,default,null)!;
    }

    [Fact]
    public void OrdinaryMaterialSurfaceStockSuppliesApproachProofWithoutRestRoutingInput()
    {
        var options=new VCarveOptions{CompactCollinearCommands=true,
            ClearedStock=new([new(new(0,0,-3),new(10,0,-3),2,2)],[3],
                [new(-5,-5),new(15,-5),new(15,5),new(-5,5)])};
        Assert.Empty(options.RoughingRestChords);Assert.Null(options.PriorStock);
        var guard=VCarveContinuationAdapter.CreatePriorStockTravelGuard(options,Cylinder,10,default,null);
        Assert.NotNull(guard);Assert.True(guard(new(2,0,10.1),new(2,0,8)));
        Assert.Empty(options.RoughingRestChords);Assert.Null(options.PriorStock);
    }

    [Fact]
    public void CompleteBodyWithPositiveRadialAndAxialReserveCanTravelThroughEarlierStock()
    {
        var guard=Guard([new(new(0,0,-3),new(10,0,-3),2,2)]);
        Assert.True(guard(new(2,0,9),new(8,0,8)));
        Assert.True(guard(new(8,0,8),new(2,0,9)));
        Assert.False(guard(new(2,0,7),new(8,0,7))); // Touching the old floor.
        Assert.False(guard(new(2,1.45,9),new(8,1.45,9))); // Insufficient radial reserve.
    }

    [Fact]
    public void TipDiskAndEndpointsCannotSubstituteForTheWholePhysicalChord()
    {
        var cone=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,5)],5);
        Assert.False(Guard([new(new(0,0,-3),new(10,0,-3),2,.2)],cone)
            (new(2,0,8),new(8,0,8)));
        var separated=Guard([new(new(0,0,-3),new(1,0,-3),1,1),
            new(new(9,0,-3),new(10,0,-3),1,1)]);
        Assert.False(separated(new(.5,0,9),new(9.5,0,9)));
    }

    [Fact]
    public void PostedTrajectoryMustAlsoHavePositiveClearance()
    {
        // At the rounded end of an earlier sweep, both nominal endpoints fit
        // inside radius 2 - (.5 + .1), but F3 (.990,.990) is outside it.
        var chord=new VCarveRoughingChord(new(0,0,-3),new(0,0,-3),2,2);
        var stock=new ClearedColumnStock([new(chord.A,chord.B,2)],default);
        Assert.True(stock.Contains(new(.9899,.9899,-1.1),new(.98991,.9899,-1.1),_=>.6));
        var guard=Guard([chord]);
        Assert.True(guard(new(.98,.98,9),new(.981,.98,9)));
        Assert.False(guard(new(.9899,.9899,9),new(.98991,.9899,9)));
    }

    [Fact]
    public void NoPriorStockOrNonfinalPlacementCannotAuthorizeTravel()
    {
        Assert.Null(VCarveContinuationAdapter.CreatePriorStockTravelGuard(new(),Cylinder,10,default,null));
        var options=new VCarveOptions{RoughingRestChords=[new(new(0,0,-3),new(10,0,-3),2,2)]}
            .WithPriorStock(default);
        Assert.Null(VCarveContinuationAdapter.CreatePriorStockTravelGuard(options,Cylinder,10,default,null));
    }
}
