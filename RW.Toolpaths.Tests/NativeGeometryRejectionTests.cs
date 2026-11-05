using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class NativeGeometryRejectionTests
{
    [Fact] public void SubcurveBoxesRejectDistanceThatWholeCurveBoxCannotReject()
    {
        var m=ToolMove.CubicBezier(new(0,0,0),new(0,10,0),new(10,10,0),new(10,0,0),0);
        Point3D[] source=[m.Start,new(5,0,0),m.End];
        Assert.True(VCarveNativeGeometryRejection.Excluded(m,source,0,2,.3));
        Assert.False(VCarveNativeGeometricProof.Prove(m,source,0,2,.3).Certified);
    }

    [Fact] public void ConservativelyRetainsKnownContinuousCorrespondences()
    {
        var random=new Random(2819);
        for(int k=0;k<100;k++)
        {
            double offset=k%2==0?0:1000000;
            var m=ToolMove.CubicBezier(new(offset,0,0),new(offset+1,random.NextDouble(),0),new(offset+2,random.NextDouble(),0),new(offset+3,0,0),0);
            var source=Enumerable.Range(0,33).Select(i=>VCarveNativeCurveMath.At(m,i/32d)).ToArray();
            Assert.True(VCarveNativeGeometricProof.Prove(m,source,0,32,.01).Certified);
            Assert.False(VCarveNativeGeometryRejection.Excluded(m,source,0,32,.01));
        }
    }

    [Fact] public void FilterPassingDoesNotGrantOrderOrClearanceAuthority()
    {
        var m=ToolMove.CubicBezier(new(0,0,0),new(1,0,0),new(2,0,0),new(3,0,0),0);
        Point3D[] source=[m.Start,new(2,0,0),new(1,0,0),m.End];
        Assert.False(VCarveNativeGeometryRejection.Excluded(m,source,0,3,.01));
        Assert.False(VCarveNativeGeometricProof.Prove(m,source,0,3,.01).Certified);
    }

    [Fact] public void ArithmeticPaddingRetainsAThresholdPose()
    {
        var m=ToolMove.CubicBezier(new(0,0,0),new(1,0,0),new(2,0,0),new(3,0,0),0);
        Assert.False(VCarveNativeGeometryRejection.Excluded(m,[m.Start,new(1.5,.3,0),m.End],0,2,.3));
        Assert.True(VCarveNativeGeometryRejection.Excluded(m,[m.Start,new(1.5,.301,0),m.End],0,2,.3));
    }
}
