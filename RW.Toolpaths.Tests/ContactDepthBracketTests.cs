using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ContactDepthBracketTests
{
    private static PointD[] Box(double a,double b,double c,double d)
        =>[new(a,b),new(c,b),new(c,d),new(a,d)];
    private static readonly VCarveCutterProfile Cutter=new([
        new(CarvingCutterSectionKind.Hemisphere,0,.79375,.79375),
        new(CarvingCutterSectionKind.Linear,.79375,3,3.8213370941988356),
        new(CarvingCutterSectionKind.Linear,3,3,90.4)],12.7);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RejectedHighTrialStillSearchesCertifiedIntermediateDepths(bool witness)
    {
        var target=Box(-1,-20,1,20);
        var authority=new VCarveProjectedStock([new(new(0,-20,-6.35),new(0,20,-6.35),10,10)],
            [target],Box(-30,-30,30,30),Cutter,12.7,default);
        double known=4.61;
        Assert.True(authority.Prove(new(0,0,-7.49),new(0,0,-7.49)));
        Assert.False(authority.Prove(new(0,0,-12.7),new(0,0,-12.7)));
        // At the shank-radius plateau the unreserved layer ceiling can jump
        // to the operation cap while the reserved layer stays at the taper.
        // That high proposal is not a test of all the depths below it.
        double depth=authority.RefineDepth(0,0,known,12.7,false,witness?new PointD(1,0):null);
        Assert.InRange(depth,7.49,7.51);
        Assert.True(authority.Prove(new(0,0,-depth),new(0,0,-depth)));
        Assert.False(authority.Prove(new(0,0,-depth-.02),new(0,0,-depth-.02)));
    }

    [Fact]
    public void ContactConstructionDoesNotGrowASpikeAtTheFullShankClearancePlateau()
    {
        var target=Box(-1,-20,1,20);
        var priorProfile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,4,4,20)],12.7);
        var priorChord=new VCarveRoughingChord(new(0,-20,-6.35),new(0,20,-6.35),0);
        var stock=new VCarveClearedStock([new(new(0,-20,-6.35),new(0,20,-6.35),3.003,3.003)],
            [6.35],Box(-30,-30,30,30),[new(priorProfile,[priorChord])]);
        var owner=new VCarveMaterialSurface([target],stock,Cutter,12.7,12.7,default,Console.WriteLine,false);
        var poses=Enumerable.Range(-20,41).Select(i=>owner.ProjectContactPose(new(i*.0002,0,0))).ToArray();
        Assert.All(poses,p=>Assert.InRange(-p.Z,7.47,7.51));
        Assert.True(poses.Max(p=>p.Z)-poses.Min(p=>p.Z)<.02);
        Assert.All(poses.Zip(poses.Skip(1)),pair=>Assert.True(owner.IsLegalChord(pair.First,pair.Second)));
        var edge=owner.ProjectCanonicalContactEdge(new(-.004,0,0),new(.004,0,0),poses[0],poses[^1],.03);
        Assert.All(edge,p=>Assert.InRange(-p.Z,7.47,7.51));
        Assert.All(edge.Zip(edge.Skip(1)),pair=>Assert.True(owner.IsLegalChord(pair.First,pair.Second)));
    }

    [Fact]
    public void HighTrialRefusalDoesNotSupplyMissingShankClearance()
    {
        var authority=new VCarveProjectedStock([new(new(0,-20,-6.35),new(0,20,-6.35),3.003,3.003)],
            [Box(-1,-20,1,20)],Box(-30,-30,30,30),Cutter,12.7,default);
        Assert.False(authority.Prove(new(0,0,-7.49),new(0,0,-7.49)));
        double depth=authority.RefineDepth(0,0,4.61,12.7,false,null);
        Assert.InRange(depth,4.61,4.62);
        Assert.True(authority.Prove(new(0,0,-depth),new(0,0,-depth)));
    }
}
