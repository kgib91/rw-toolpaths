using Clipper2Lib;
using RW.Toolpaths.Milling.Acceleration;
using Xunit;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class RawInitialClearingFrontConditioningTests
{
    [Fact]
    public void OptionalFourthCaptureIsDefaultOffImmutableAndRequiresEligibleRawSmoothOwner()
    {
        var original=RawInitialClearingInsetNomination.CaptureForSmoothGeneration(true,15,true)!;
        var requested=RawInitialClearingInsetNomination.CaptureForSmoothGeneration(true,15,true,true)!;
        Assert.False(original.FrontConditioningCaptured);Assert.True(requested.FrontConditioningCaptured);
        Assert.Null(RawInitialClearingInsetNomination.CaptureForSmoothGeneration(false,15,true,true));
        Assert.Null(RawInitialClearingInsetNomination.CaptureForSmoothGeneration(true,15,false,true));
        Assert.Null(RawInitialClearingInsetNomination.CaptureForSmoothGeneration(true,double.NaN,true,true));
        string? before=Environment.GetEnvironmentVariable(RawInitialClearingFrontConditioner.EnvironmentName);
        try
        {
            Environment.SetEnvironmentVariable(RawInitialClearingFrontConditioner.EnvironmentName,"0");
            Assert.True(requested.FrontConditioningCaptured);
            Environment.SetEnvironmentVariable(RawInitialClearingFrontConditioner.EnvironmentName,"1");
            Assert.False(original.FrontConditioningCaptured);
        }
        finally{Environment.SetEnvironmentVariable(RawInitialClearingFrontConditioner.EnvironmentName,before);}
    }

    [Fact]
    public void FrontConditioningRetainsEveryRingTinyLoopAndWholeFallbackWithCancellation()
    {
        var circle=new Path64();for(int i=0;i<8192;i++){double t=2*Math.PI*i/8192;circle.Add(new((long)Math.Round(32768*Math.Cos(t)),(long)Math.Round(32768*Math.Sin(t))));}
        var tiny=new Path64{new(0,0),new(1,0),new(0,1)};var raw=new Paths64{circle,tiny};var logs=new List<string>();
        Assert.True(RawInitialClearingFrontConditioner.TryCondition(raw,CancellationToken.None,logs.Add,out var candidate,out _));
        Assert.Equal(2,candidate!.Count);Assert.True(candidate[0].Count<circle.Count);Assert.Equal(tiny,candidate[1]);
        Assert.Equal(circle[0],candidate[0][0]);Assert.Equal(8192,raw[0].Count);
        Assert.Contains(logs,s=>s.Contains("maximumNativeMove=32")&&s.Contains("closedRingInventoryRetained=True")
            &&s.Contains("addedInsetPad=0")&&s.Contains("filledTopologyCredit=False")&&s.Contains("physicalCredit=False"));
        Assert.False(RawInitialClearingFrontConditioner.TryCondition(new Paths64{tiny},CancellationToken.None,null,out var small,out _));Assert.Null(small);
        Assert.False(RawInitialClearingFrontConditioner.TryCondition(new Paths64{new Path64{new(0,0),new(1,0)}},CancellationToken.None,null,out var unsupported,out _));Assert.Null(unsupported);
        using var canceled=new CancellationTokenSource();canceled.Cancel();
        Assert.Throws<OperationCanceledException>(()=>RawInitialClearingFrontConditioner.TryCondition(raw,canceled.Token,null,out _,out _));
        using var atPublish=new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(()=>RawInitialClearingFrontConditioner.TryCondition(raw,atPublish.Token,_=>atPublish.Cancel(),out _,out _));
    }
}
