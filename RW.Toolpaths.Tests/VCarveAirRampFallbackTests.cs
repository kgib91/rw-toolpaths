using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveAirRampFallbackTests
{
    private static readonly VCarveCutterProfile Cone=new([new(CarvingCutterSectionKind.Linear,0,3,6)],5);
    private static readonly MillingMotionSettings Motion=new(1000,500,5000,10,0);

    private static TaggedToolpath Path(Point3D[] points,params ToolpathSpan[] spans)
        =>new(points,spans,0,"clearing",0,detectCurves:false);

    private static void GeometryAndContactKindsStayLiteral(IReadOnlyList<TaggedToolpath> before,IReadOnlyList<TaggedToolpath> after)
    {
        Assert.Equal(before.Count,after.Count);
        for(int p=0;p<before.Count;p++){
            Assert.Same(before[p].Points,after[p].Points);Assert.Same(before[p].Moves,after[p].Moves);
            Assert.Equal(before[p].RegionIndex,after[p].RegionIndex);
            Assert.Equal(before[p].Category,after[p].Category);Assert.Equal(before[p].DepthPassIndex,after[p].DepthPassIndex);
            Assert.Equal(before[p].VisitIdentity,after[p].VisitIdentity);
            var original=ToolpathSpanKinds.Expand(before[p]);var actual=ToolpathSpanKinds.Expand(after[p]);
            for(int i=0;i<original.Length;i++)if(original[i] is ToolpathSpanKind.Ramp or ToolpathSpanKind.Cut)
                Assert.Equal(original[i],actual[i]);
        }
    }

    [Fact]
    public void AnAmbiguousBridgeCannotRestoreStaleSourceFastTagsOnProofFailure()
    {
        var cut=Path([new(0,0,-4),new(20,0,-4)],new ToolpathSpan(0,1,ToolpathSpanKind.Cut));
        var stale=Path([new(20,5,-3),new(0,5,-3)],new ToolpathSpan(0,1,ToolpathSpanKind.ClearedTravel));
        var entry=Path([new(.00004,5,-3),new(1,5,-3.1),new(2,5,-3.1)],
            new(0,1,ToolpathSpanKind.Ramp),new(1,2,ToolpathSpanKind.Cut));
        var source=new[]{cut,stale,entry};var log=new List<string>();
        var result=VCarveToolpaths.OptimizeAirRampFeeds(source,Cone,0,Motion,diagnostic:log.Add);

        GeometryAndContactKindsStayLiteral(source,result);
        Assert.All(result,p=>Assert.DoesNotContain(p.Spans,s=>s.Kind==ToolpathSpanKind.ClearedTravel));
        Assert.Equal(ToolpathSpanKind.Link,ToolpathSpanKinds.Expand(result[1])[0]);
        Assert.Contains(log,s=>s.Contains("refused=ambiguous-bridge-XY-anchor"));
        Assert.Contains(log,s=>s.Contains("accepted=False reason=final-prefix-proof")
            &&s.Contains("fallback=controlled-source-feed")&&s.Contains("demotedFastChords=1"));
        // Source ownership is immutable even when the returned feed authority changes.
        Assert.Equal(ToolpathSpanKind.ClearedTravel,ToolpathSpanKinds.Expand(source[1])[0]);
    }

    [Fact]
    public void NonlinearRefusalRevokesAllFastPermissionsWithoutRewritingTheCurveOrContactKinds()
    {
        var points=new Point3D[]{new(0,5,-2),new(1,5,-2),new(2,5,-2),new(3,5,-2),new(4,5,-2)};
        var moves=new[]{ToolMove.CubicBezier(points[0],new(1,6,-2),new(3,6,-2),points[^1],.001)};
        var curved=new TaggedToolpath(points,moves,3,"clearing",7,42){Spans=[
            new(0,1,ToolpathSpanKind.Ramp),new(1,2,ToolpathSpanKind.Cut),new(2,4,ToolpathSpanKind.ClearedTravel)]};
        var log=new List<string>();
        var result=VCarveToolpaths.OptimizeAirRampFeeds([curved],Cone,0,Motion,diagnostic:log.Add);

        GeometryAndContactKindsStayLiteral([curved],result);
        Assert.Equal(new[]{ToolpathSpanKind.Ramp,ToolpathSpanKind.Cut,ToolpathSpanKind.Link,ToolpathSpanKind.Link},
            ToolpathSpanKinds.Expand(result[0]));
        Assert.Contains(log,s=>s.Contains("reason=nonlinear-motion")&&s.Contains("demotedFastChords=2")
            &&s.Contains("allFastFeedPermissionsRevoked=True"));
    }

    [Fact]
    public void AVerifiedLinearResultRemainsIdenticalToTheSuccessfulFeedPolicy()
    {
        var cut=Path([new(0,0,-4),new(20,0,-4)],new ToolpathSpan(0,1,ToolpathSpanKind.Cut));
        var entry=Path([new(0,0,-2),new(5,0,-2.5),new(10,0,-3),new(15,0,-4),new(20,0,-4)],
            new(0,3,ToolpathSpanKind.Ramp),new(3,4,ToolpathSpanKind.Cut));
        var source=new[]{cut,entry};
        var expected=VCarveAirRampFeed.Improve(source,Cone,0,Motion,default,null);
        var log=new List<string>();
        var result=VCarveToolpaths.OptimizeAirRampFeeds(source,Cone,0,Motion,diagnostic:log.Add);

        Assert.Contains(result[1].Spans,s=>s.Kind==ToolpathSpanKind.ClearedTravel);
        for(int p=0;p<result.Count;p++){
            Assert.Same(expected[p].Points,result[p].Points);Assert.Same(expected[p].Moves,result[p].Moves);
            Assert.Equal(expected[p].Spans,result[p].Spans);
        }
        Assert.DoesNotContain(log,s=>s.Contains("accepted=False"));
        VCarveReturnTravel.VerifyFinalTravel(result,Cone,0,default,Console.WriteLine,finalCompactionComplete:true);
    }

    [Fact]
    public void CancellationPropagatesInsteadOfReturningAnySourceFeedPermissions()
    {
        var source=Path([new(0,0,-2),new(1,0,-2)],new ToolpathSpan(0,1,ToolpathSpanKind.ClearedTravel));
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        Assert.Throws<OperationCanceledException>(()=>VCarveToolpaths.OptimizeAirRampFeeds([source],Cone,0,Motion,cancel.Token));
        Assert.Equal(ToolpathSpanKind.ClearedTravel,ToolpathSpanKinds.Expand(source)[0]);
    }
}
