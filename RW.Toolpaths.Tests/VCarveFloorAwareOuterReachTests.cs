using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveFloorAwareOuterReachTests
{
    private static readonly VCarveCutterProfile Profile=new(
        [new(CarvingCutterSectionKind.Linear,0,2,2)],2);

    [Fact]
    public void BoundedSolveRetainsItsBestInteriorPointInsteadOfRestoringAnIsolatedTooth()
    {
        Point3D[] ring=[new(2,0,-2),new(2,1,-2),new(2.01,1,-2),new(2.01,0,-2),new(2,0,-2)];
        var source=new TaggedToolpath(ring,[],0,VCarveInitialReachSelection.QuarterOuterMarkerCategory,null,detectCurves:false);
        var logs=new List<string>();
        double Cap(double x,double y)=>.2+1.6*(1-Math.Exp(-.5*x*x));
        var output=VCarveFloorAwareOuterReach.Apply([source],Profile,2,Cap,(x,y)=>x,
            (x,y,d)=>Profile.PhysicalRadiusAtDepth(d),default,logs.Add,samplingTolerance:.17);
        Assert.Single(output);
        Assert.Equal(output[0].Points[0],output[0].Points[^1]);
        Assert.Contains(logs,line=>line.Contains("event=partial-projection"));
        Assert.All(output[0].Points.Where(p=>p.Y==0||p.Y==1),p=>
        {
            Assert.True(p.X<1.8,$"Bounded projection restored old X={p.X:R}");
            Assert.Equal(-Cap(p.X,p.Y),p.Z,12);
            Assert.True(p.X>=Profile.PhysicalRadiusAtDepth(-p.Z));
        });
    }

    [Fact]
    public void SlopingFloorUsesTheChangingCutterFootprintToFindContactWithinTheBoundedSolve()
    {
        Point3D[] ring=[new(2,0,-2),new(2,1,-2),new(2.01,1,-2),new(2.01,0,-2),new(2,0,-2)];
        var source=new TaggedToolpath(ring,[],0,VCarveInitialReachSelection.QuarterOuterMarkerCategory,null,detectCurves:false);
        var output=VCarveFloorAwareOuterReach.Apply([source],Profile,2,(x,y)=>.25+.75*x,(x,y)=>x,
            (x,y,d)=>Profile.PhysicalRadiusAtDepth(d),default,samplingTolerance:.17);
        Assert.All(output[0].Points,p=>{Assert.Equal(1,p.X,8);Assert.Equal(-1,p.Z,8);});
    }

    [Fact]
    public void ConstantFloorStillConvergesToTheSameLiteralContactRing()
    {
        Point3D[] ring=[new(2,0,-2),new(2,1,-2),new(2.01,1,-2),new(2.01,0,-2),new(2,0,-2)];
        var source=new TaggedToolpath(ring,[],0,VCarveInitialReachSelection.QuarterOuterMarkerCategory,null,detectCurves:false);
        var output=VCarveFloorAwareOuterReach.Apply([source],Profile,2,(_,_)=>1,(x,y)=>x,
            (x,y,d)=>Profile.PhysicalRadiusAtDepth(d),default,samplingTolerance:.17);
        Assert.All(output[0].Points,p=>{Assert.Equal(1,p.X,9);Assert.Equal(-1,p.Z);});
    }
}
