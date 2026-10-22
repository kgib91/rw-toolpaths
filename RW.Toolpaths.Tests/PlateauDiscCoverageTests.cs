using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PlateauDiscCoverageTests
{
    [Fact]
    public void ConvexSubsetProofRefusesAChainedUnionAndTheFirstOutsideVertex()
    {
        var index=new PlateauDiscCoverage([new(new(0,0),10),new(new(15,0),10)],default);
        Assert.True(index.Contains(new Path64 {new(-2,-2),new(2,-2),new(2,2),new(-2,2)}));
        Assert.False(index.Contains(new Path64 {new(0,-1),new(15,-1),new(15,1),new(0,1)}));
        Assert.False(index.Contains(new Path64 {new(-2,-2),new(2,-2),new(11,0)}));
        Assert.True(index.ContainsHull(new(0,0),2,new(3,0),2));
        Assert.False(index.ContainsHull(new(0,0),2,new(15,0),2));
        Assert.False(index.ContainsHull(new(0,0),2,new(8,0),3));
        using var cancelled=new CancellationTokenSource();cancelled.Cancel();
        Assert.Throws<OperationCanceledException>(()=>new PlateauDiscCoverage([new(new(0,0),10)],cancelled.Token));
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void InteriorRoundedRampPrimitivesKeepTheCompleteReferenceExclusion(bool targetOnly)
    {
        const double floor=6,grade=1;
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new(CarvingCutterSectionKind.Linear,1,2,8)],8);
        var smooth=new List<TaggedToolpath> {new([new(0,0,-1),new(.01,0,-1.5)],[],0,"final-carve",null,detectCurves:false)};
        for(int i=0;i<100;i++) smooth.Add(new([new((i%10-5)*.1,(i/10-5)*.1,-5.99),
            new((i%10-5)*.1+.03,(i/10-5)*.1+.01,-5.9)],[],0,"final-carve",null,detectCurves:false));
        var original=smooth.Select(p=>p with {Points=p.Points.Select(v=>v with {Z=-floor}).ToArray()}).ToArray();
        IReadOnlyList<PointD>[] boundary=[[new(-10,-10),new(10,-10),new(10,10),new(-10,10)]];
        var reference=SmoothRoughingPlateau.Build(original,boundary,floor,grade,default,smooth,profile,targetOnly?boundary:null,false);
        var actual=SmoothRoughingPlateau.Build(original,boundary,floor,grade,default,smooth,profile,targetOnly?boundary:null);
        double gridReserve=8/PathUtils.Scale;
        Assert.Empty(PolygonOps.Difference(reference,PolygonOps.Offset(actual,gridReserve,.00001,0)));
        Assert.Empty(PolygonOps.Difference(actual,PolygonOps.Offset(reference,gridReserve,.00001,0)));
        Assert.True(PolygonOps.Contains(actual,4.9,0));
        Assert.False(PolygonOps.Contains(actual,5.5,0));
    }
}
