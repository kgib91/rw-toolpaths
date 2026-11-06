using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class RetainedRampEdgeProofTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void EdgeOnlyConsumerMatchesLiteralClipsAndRequestedProof(bool outsideWindow,bool entirelyRetained)
    {
        var part=Box(0,0,20,20);
        var target=Box(5,5,15,15);
        var clearance=Box(0,5,5,15);
        var access=new MillingBoundaryAccess([new MillingRegion(clearance)],part);
        var tool=ToolGeometry.Flat(1);
        var depth=new DepthSchedule(3,1);
        var prepared=access.Resolve([new MillingRegion(target)],tool,depth,true,30,0,default);
        var boundary=new List<IReadOnlyList<PointD>>();
        if(outsideWindow) boundary.Add(Box(-100,-100,100,100));
        if(!outsideWindow||!entirelyRetained)
            boundary.Add(entirelyRetained?Box(7,7,10,10):target);
        // For a fully retained interior source use an actual protected island.
        // The access recipe exposes this independent polygon as a retained wall.
        if(!outsideWindow&&entirelyRetained)
        {
            target=Box(5,5,15,15);
            var island=Box(7,7,10,10);
            access=new MillingBoundaryAccess([new MillingRegion(clearance)],part);
            prepared=access.Resolve([new MillingRegion(target,[island])],tool,depth,true,30,0,default);
        }
        var source=boundary.Select(p=>(IReadOnlyList<PointD>)p.Append(p[0]).ToArray()).ToArray();
        var allowed=PolygonOps.Union(new[]{(IReadOnlyList<PointD>)target,clearance});
        var protectedMaterial=PolygonOps.Difference(new[]{(IReadOnlyList<PointD>)part},allowed);
        if(!outsideWindow&&entirelyRetained)
            protectedMaterial=PolygonOps.Union(protectedMaterial.Concat(new[]{boundary[0]}));
        IReadOnlyList<IReadOnlyList<PointD>> clip;
        ClipType retainedOperation,proofOperation;
        if(!outsideWindow)
        {
            const double arc=.0001;
            double radius=2*VCarvePreparation.GeometricErrorBound+4*arc+32/PathUtils.Scale;
            clip=PolygonOps.Buffer(protectedMaterial,radius,arc);
            retainedOperation=ClipType.Intersection;proofOperation=ClipType.Difference;
        }
        else
        {
            double pad=2*tool.Radius+depth.Depth/Math.Tan(Math.PI/6)+1;
            var window=Box(5-pad,5-pad,15+pad,15+pad);
            var physical=PolygonOps.Difference(new[]{(IReadOnlyList<PointD>)window},protectedMaterial);
            clip=PolygonOps.Offset(physical,-2*VCarvePreparation.GeometricErrorBound,.0001,0);
            retainedOperation=ClipType.Difference;proofOperation=ClipType.Intersection;
        }
        var literal=CheckedOpenClip.Require(source,clip,retainedOperation,default,"ramp-edge-literal-oracle");
        bool proof=CheckedOpenClip.Require(source,clip,proofOperation,default,"ramp-edge-complement-oracle").Count==0;
        var expected=literal.SelectMany(p=>Enumerable.Range(1,p.Count-1).Select(i=>(p[i-1],p[i]))).ToArray();
        var edgesOnly=prepared.RetainedRampEdges(boundary,default);
        var edgesWithProof=prepared.RetainedRampEdges(boundary,default,out bool allRetained);
        Assert.Equal(expected,edgesOnly);
        Assert.Equal(expected,edgesWithProof);
        Assert.Equal(proof,allRetained);
        Assert.Equal(entirelyRetained,allRetained);
    }

    [Fact]
    public void BothConsumersHonorCancellationBeforeReturningAnyEdges()
    {
        var part=Box(0,0,20,20);
        var prepared=new MillingBoundaryAccess([],part).Resolve([new MillingRegion(part)],
            ToolGeometry.Flat(1),new(1,1),true,30,0,default);
        using var cancelled=new CancellationTokenSource();cancelled.Cancel();
        IReadOnlyList<IReadOnlyList<PointD>> source=[part];
        Assert.Throws<OperationCanceledException>(()=>prepared.RetainedRampEdges(source,cancelled.Token));
        Assert.Throws<OperationCanceledException>(()=>prepared.RetainedRampEdges(source,cancelled.Token,out _));
    }

    private static PointD[] Box(double left,double bottom,double right,double top)
        => [new(left,bottom),new(right,bottom),new(right,top),new(left,top)];
}
