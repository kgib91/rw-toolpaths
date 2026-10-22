using System.Globalization;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

// Adapter controls only. True synthetic guards exercise custody and publication;
// they supply no physical floor/wall/stock authority. Real owner facts are bound
// to the separately executed actual-floor consumer and later ApplyCore checks.
public sealed class ProductionF3CompactionControls
{
    static Point3D Command(Point3D p,double surface=0)
    {
        static double F3(float x)=>double.Parse(x.ToString("F3",CultureInfo.InvariantCulture),CultureInfo.InvariantCulture);
        return new(F3((float)p.X),F3((float)p.Y),F3((float)(surface+p.Z)-(float)surface));
    }
    static TaggedToolpath Path(Point3D[] p,ToolpathSpan[]? spans=null,bool curves=false)
        =>new(p,spans??Array.Empty<ToolpathSpan>(),7,"final-carve",3,11,detectCurves:curves);
    static ExactF3TrajectorySimplifier.GuardDecision Pass(TaggedToolpath path)=>new(true,true,true,true,true,"synthetic-adapter-custody-only");
    static VCarveActualF3TrajectoryCompaction Owner(Func<TaggedToolpath,ExactF3TrajectorySimplifier.GuardDecision>? proof=null,
        CancellationToken token=default)=>new(0,p=>Command(p),proof??Pass,_=>true,token,null);
    static Point3D[] Zigzag(int count)=>Enumerable.Range(0,count).Select(i=>new Point3D(i*.01,(i&1)*.001,-.1)).ToArray();

    [Fact]
    public void CompleteNativeEndpointOrdinalsAndEverySpanBoundaryRemainOwned()
    {
        var p=Zigzag(16);p[0]=new(p[0].X,BitConverter.Int64BitsToDouble(long.MinValue),p[0].Z);
        ToolpathSpan[] spans=[new(0,4,ToolpathSpanKind.Cut),new(4,8,ToolpathSpanKind.Ramp),
            new(8,12,ToolpathSpanKind.Link),new(12,13,ToolpathSpanKind.TabLift),new(13,15,ToolpathSpanKind.Retract)];
        var original=Path(p,spans);var owner=Owner();var candidate=Assert.Single(owner.Compact([original],"fixture"));
        Assert.True(candidate.Points.Count<original.Points.Count);Assert.Equal(7,candidate.RegionIndex);Assert.Equal(3,candidate.DepthPassIndex);Assert.Equal(11,candidate.VisitIdentity);
        Assert.Equal(original.Category,candidate.Category);Assert.Equal(spans.Length,candidate.Spans.Count);
        Assert.Equal(BitConverter.DoubleToInt64Bits(p[0].Y),BitConverter.DoubleToInt64Bits(candidate.Points[0].Y));
        Assert.Contains(p[1],candidate.Points);
        for(int i=0;i<spans.Length;i++)
        {
            Assert.Equal(spans[i].Kind,candidate.Spans[i].Kind);
            Assert.Equal(p[spans[i].StartIndex],candidate.Points[candidate.Spans[i].StartIndex]);
            Assert.Equal(p[spans[i].EndIndex],candidate.Points[candidate.Spans[i].EndIndex]);
        }
        foreach(var q in candidate.Points)Assert.Contains(q,p);
        Assert.Equal(p,ZigzagSnapshotExceptSignedZero(p)); // No caller geometry mutation.
        static Point3D[] ZigzagSnapshotExceptSignedZero(Point3D[] points)
        {var expected=Zigzag(points.Length);expected[0]=new(expected[0].X,BitConverter.Int64BitsToDouble(long.MinValue),expected[0].Z);return expected;}
    }

    [Fact]
    public void CurrentStrictSurfaceMotionAndPartialAxisArrivalAnchorsRemainLiteral()
    {
        Assert.Equal(0,VCarveActualF3TrajectoryCompaction.MotionMode(new(0,0,-.0005),new(.01,0,-.0005),12.7));
        Assert.Equal(1,VCarveActualF3TrajectoryCompaction.MotionMode(new(0,0,0),new(.01,0,0),12.7));
        Assert.Equal(4,VCarveActualF3TrajectoryCompaction.MotionMode(new(0,0,0),new(0,0,-.01),12.7));
        Assert.Equal(2,VCarveActualF3TrajectoryCompaction.MotionMode(new(0,0,-.01),new(0,0,0),12.7));
        var p=Zigzag(12);p[4]=new(p[3].X,p[3].Y,-.11);p[5]=new(.05,.001,-.11);
        var candidate=Assert.Single(Owner().Compact([Path(p)],"partial-axis-fixture"));
        Assert.Contains(p[4],candidate.Points);Assert.Contains(p[5],candidate.Points);
        Assert.Contains(p[1],candidate.Points);Assert.Equal(p[0],candidate.Points[0]);Assert.Equal(p[^1],candidate.Points[^1]);
    }

    [Fact]
    public void EveryFailedFreshOwnerKeepsOriginalChainWithoutReusingPassedEvidence()
    {
        foreach(int refused in Enumerable.Range(0,5))
        {
            int calls=0;var original=Path(Zigzag(24));
            var owner=Owner(_=>{calls++;return new(refused!=0,refused!=1,refused!=2,refused!=3,refused!=4,"independent-failed-owner");});
            var candidate=Assert.Single(owner.Compact([original],"failed-owner-fixture"));
            Assert.True(calls>1);Assert.Equal(original.Points,candidate.Points);Assert.Equal(original.Spans,candidate.Spans);Assert.Equal(0,owner.Guarded);
        }
    }

    [Fact]
    public void InvocationAndPathKeysCannotCombineForeignOrDifferentPathOwners()
    {
        var points=Enumerable.Range(0,9).Select(i=>new ExactF3TrajectorySimplifier.Tick3(i*10,i%2,0)).ToArray();
        var invocation=new object();var keys=new ExactF3TrajectorySimplifier.BoundaryKey[8];
        for(int e=0;e<8;e++)keys[e]=new(invocation,e<4?0:1,7,3,11,-1,ToolpathSpanKind.Cut,"final-carve",1,7);
        keys[2]=keys[2] with{InvocationOwner=new object()};
        var result=ExactF3TrajectorySimplifier.Simplify(points,keys,new bool[points.Length],proposal=>
        {Assert.Same(invocation,proposal.Boundary.InvocationOwner);return new(true,true,true,true,true,"synthetic-key-fixture");},default);
        Assert.Contains(2,result.OriginalOrdinals);Assert.Contains(3,result.OriginalOrdinals);Assert.Contains(4,result.OriginalOrdinals);
        Assert.True(result.Counts.BoundaryStops>0);
    }

    [Fact]
    public void CurvesOvercapMalformedSpansAndCancellationCannotPublishReplacement()
    {
        Assert.True(VCarveSmoothing.IsActualF3FloorProofBudgetRefusal(new InvalidOperationException("Smooth floor query exceeded its proof budget at (1,2), best=0, lower=0.")));
        Assert.False(VCarveSmoothing.IsActualF3FloorProofBudgetRefusal(new InvalidOperationException("Smooth V-carving floor proof exceeds 4 points.")));
        Assert.False(VCarveSmoothing.IsActualF3FloorProofBudgetRefusal(new InvalidOperationException("unexpected-original-field-error")));
        var curve=Path(Zigzag(12),curves:true);Assert.Equal(curve,Assert.Single(Owner().Compact([curve],"curve")));
        var overlap=Path(Zigzag(12),[new(0,8,ToolpathSpanKind.Cut),new(4,11,ToolpathSpanKind.Link)]);
        Assert.Equal(overlap,Assert.Single(Owner().Compact([overlap],"overlap")));
        var large=Path(Enumerable.Range(0,ExactF3TrajectorySimplifier.MaximumInputPoints+1).Select(i=>new Point3D(i*.001,0,-.1)).ToArray());
        var capped=Owner();Assert.Equal(large,Assert.Single(capped.Compact([large],"overcap")));Assert.Equal(0,capped.Proposals);
        using var cancel=new CancellationTokenSource();var canceled=Owner(_=>{cancel.Cancel();return Pass(default);},cancel.Token);
        Assert.Throws<OperationCanceledException>(()=>canceled.Compact([Path(Zigzag(24))],"callback-cancel"));
    }
}
