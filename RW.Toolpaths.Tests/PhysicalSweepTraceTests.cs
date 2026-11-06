using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PhysicalSweepTraceTests
{
    private static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Linear,.1,.5,2)],2);
    private static VCarvePhysicalSweepTrace.Parameters Settings(bool upper=true)=>new(12.7,.002,.002,upper,true,true,false,false,false,1,false,32,256,2,4096,
        4/PathUtils.Scale,6/PathUtils.Scale,.25/PathUtils.Scale);
    private static TaggedToolpath Path(Point3D[] points)=>new(points,[new(0,points.Length-1,ToolpathSpanKind.Cut)],2,"trace-source",1,7,detectCurves:false);
    private static Paths64 Result()=>new(){new(){new(0,0),new(10,0),new(10,10),new(0,10)}};

    [Fact]
    public void SourceProfileAndCompletedIntegerOutputAreDetachedAndCaptureOnlyOnce()
    {
        Point3D[] points=[new(-0d,1,-.2),new(2,1,-.3)];var source=new[]{Path(points)};var result=Result();
        using var session=VCarvePhysicalSweepTrace.BeginCapture(0);
        using(var lease=VCarvePhysicalSweepTrace.Begin(source,Profile,Settings(),default,false)!)lease.Complete(result,default,new(2,2,0,1,1,0));
        var captured=Assert.IsType<VCarvePhysicalSweepTrace.Evidence>(session.Completed);
        points[0]=new(100,100,100);result[0][0]=new(999,999);
        Assert.Equal(BitConverter.DoubleToInt64Bits(-0d),BitConverter.DoubleToInt64Bits(captured.Source[0].Points[0].X));
        Assert.Equal(0,captured.CompletedResult[0][0].X);Assert.NotSame(Profile.Sections[0],captured.ProfileSections[0]);
        Assert.Equal(2,captured.SourcePoints);Assert.Equal(1,captured.SourceChords);Assert.Equal(2,captured.Source[0].RegionIndex);
        Assert.Equal(2,captured.Backend!.UnionCompleted);
        Assert.Null(VCarvePhysicalSweepTrace.Begin(source,Profile,Settings(),default,false));
        Assert.InRange(captured.ChargedBytes,1,VCarvePhysicalSweepTrace.MaximumOwnedBytes);
    }

    [Theory] [InlineData(false,false)] [InlineData(true,true)]
    public void LowerAndNominationRequestsCannotBecomeUpperSweepEvidence(bool upper,bool nomination)
    {
        using var session=VCarvePhysicalSweepTrace.BeginCapture(0);
        Assert.Null(VCarvePhysicalSweepTrace.Begin([Path([new(0,0,-.2),new(1,0,-.2)])],Profile,Settings(upper),default,nomination));
        Assert.Null(session.Completed);Assert.False(session.CaptureStatus.Selected);
    }

    [Fact]
    public void OversizedSourceAndResultAreRefusedWithoutPublishingPartialEvidence()
    {
        var source=new[]{Path([new(0,0,-.2),new(1,0,-.2)])};
        using(var session=VCarvePhysicalSweepTrace.BeginCapture(0,1))
        {
            Assert.Null(VCarvePhysicalSweepTrace.Begin(source,Profile,Settings(),default,false));
            Assert.Equal("source-byte-limit",session.CaptureStatus.Refusal);Assert.Equal(0,session.CaptureStatus.ChargedBytes);
        }
        using(var session=VCarvePhysicalSweepTrace.BeginCapture(0,1024))
        {
            using var lease=VCarvePhysicalSweepTrace.Begin(source,Profile,Settings(),default,false)!;Assert.NotNull(lease);
            lease.Complete(new(){new(Enumerable.Range(0,100).Select(i=>new Point64(i,i)))},default);
            Assert.Null(session.Completed);Assert.Equal("result-byte-limit",session.CaptureStatus.Refusal);Assert.Equal(0,session.CaptureStatus.ChargedBytes);
        }
    }

    [Fact]
    public void IncompleteAndCanceledBuildsNeverPublish()
    {
        var source=new[]{Path([new(0,0,-.2),new(1,0,-.2)])};
        using(var session=VCarvePhysicalSweepTrace.BeginCapture(0))
        {
            VCarvePhysicalSweepTrace.Begin(source,Profile,Settings(),default,false)!.Dispose();
            Assert.Equal(1,session.CaptureStatus.IncompleteRequests);Assert.Null(session.Completed);
        }
        using(var session=VCarvePhysicalSweepTrace.BeginCapture(0))using(var cancellation=new CancellationTokenSource())
        {
            using var lease=VCarvePhysicalSweepTrace.Begin(source,Profile,Settings(),default,false)!;cancellation.Cancel();
            Assert.Throws<OperationCanceledException>(()=>lease.Complete(Result(),cancellation.Token));Assert.Null(session.Completed);
        }
    }

    [Fact]
    public void NestedSessionRestoresTheOriginalCaptureAndDefaultOffDoesNotReadInputs()
    {
        Assert.False(VCarvePhysicalSweepTrace.IsEnabled);
        Assert.Null(VCarvePhysicalSweepTrace.Begin(null!,null!,null!,default,false));
        using(var outer=VCarvePhysicalSweepTrace.BeginCapture(0))
        {
            using(var inner=VCarvePhysicalSweepTrace.BeginCapture(0))
            {
                using var lease=VCarvePhysicalSweepTrace.Begin([Path([new(0,0,-.2),new(1,0,-.2)])],Profile,Settings(),default,false)!;
                lease.Complete(Result(),default);Assert.NotNull(inner.Completed);Assert.Null(outer.Completed);
            }
            using var outerLease=VCarvePhysicalSweepTrace.Begin([Path([new(0,0,-.2),new(1,0,-.2)])],Profile,Settings(),default,false)!;
            outerLease.Complete(Result(),default);Assert.NotNull(outer.Completed);
        }
        Assert.False(VCarvePhysicalSweepTrace.IsEnabled);
    }
}
