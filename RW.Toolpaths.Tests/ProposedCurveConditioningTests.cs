namespace RW.Toolpaths.Tests;

public sealed class ProposedCurveConditioningTests
{
    [Fact]
    public void NominatedMicroCornersCanSmoothWithinAuthoredToleranceWhileOriginalCornersRemain()
    {
        List<Point3D> points=[new(0,0,0),new(.01,.01,0),new(.02,-.01,0),new(.03,.01,0),new(.04,0,0)];
        Assert.Equal(points,MedialAxisToolpaths.RdpSimplify3D(points,.015));
        Assert.Equal(new[]{points[0],points[^1]},MedialAxisToolpaths.RdpSimplify3D(points,.015,false));
    }

    [Fact]
    public void NominationConditioningDoesNotLoseAnExcursionBeyondAChordEndpoint()
    {
        List<Point3D> points=[new(0,0,0),new(0,0,2),new(0,0,1)];
        Assert.Equal(points,MedialAxisToolpaths.RdpSimplify3D(points,.01,false));
    }

    [Fact]
    public void LargeDesignCornersSurviveTheFiniteErrorBound()
    {
        List<Point3D> points=[new(0,0,0),new(1,1,0),new(2,0,0)];
        Assert.Equal(points,MedialAxisToolpaths.RdpSimplify3D(points,.03,false));
    }
}
