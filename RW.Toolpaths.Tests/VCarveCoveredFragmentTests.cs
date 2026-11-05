using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveCoveredFragmentTests
{
    private static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Hemisphere,0,1,1),new(CarvingCutterSectionKind.Linear,1,1,10)],5);
    private static TaggedToolpath Cut(double x0,double x1,double z)=>new([new(x0,0,z),new(x1,0,z)],[],0,"clearing",0,detectCurves:false);
    private static TaggedToolpath[] Reduce(TaggedToolpath[] paths,TaggedToolpath[]? other=null,double top=0,double datum=0)
        =>VCarveRingDemand.WithoutCoveredFragments(paths,other??[],Profile,datum,top,.03,.7,default,Console.WriteLine);
    [Fact]
    public void DeepDebrisCoveredByRetainedLongWorkNeedsNoSeparateEntry()
    {
        var longCut=Cut(0,10,-2);var debris=Cut(4,4.05,-2);
        Assert.Equal(new[]{longCut},Reduce([longCut,debris]));
    }
    [Fact]
    public void DeeperRemovalOutsideTheDonorIsRequired()
    {
        var longCut=Cut(0,10,-1);var deeper=Cut(4,4.05,-2);
        Assert.Equal(new[]{longCut,deeper},Reduce([longCut,deeper]));
    }
    [Fact]
    public void TwoShortFragmentsCannotDeleteEachOther()
    {
        var a=Cut(4,4.05,-2);var b=Cut(4,4.05,-2);
        Assert.Equal(2,Reduce([a,b]).Length);
    }
    [Fact]
    public void NoncutPathsAreNeitherOmittedNorUsedAsRemovalDonors()
    {
        var longLink=Cut(0,10,-2) with{Spans=[new(0,1,ToolpathSpanKind.ClearedTravel)]};var debris=Cut(4,4.05,-2);
        Assert.Equal(2,Reduce([longLink,debris]).Length);
        Assert.Equal(new[]{longLink},Reduce([longLink], [Cut(0,10,-2)]));
    }
    [Fact]
    public void UseOriginalMaterialTopWhenTheOperationDatumIsAStartingFloor()
    {
        var donor=Cut(0,10,9);var deeper=Cut(4,4.05,8);
        Assert.Equal(2,Reduce([donor,deeper],top:10,datum:5).Length);
    }
    [Fact]
    public void APlannedFutureDonorCannotDeleteDepthPassPredecessors()
    {
        var donor=Cut(0,10,-2);var shallow=Cut(4,4.05,-1);var deep=Cut(4,4.05,-2) with{DepthPassIndex=1};
        Assert.Equal(new[]{donor,shallow,deep},Reduce([donor,shallow,deep]));
    }
}
