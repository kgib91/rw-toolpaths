using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativePreparedSourceProbeTests
{
    private static Point3D[] Points(int count)=>Enumerable.Range(0,count).Select(i=>new Point3D(i,0,-.5)).ToArray();
    private static VCarveNativeFit Edge(Point3D[] points,int a,int b,double? length=null)
        =>new(ToolMove.Line(points[a],points[b]),a,b,b-a,length??b-a,length??b-a,0,0);
    private static List<VCarveNativeFit>? Probe(Point3D[] points,Func<int,int,VCarveNativeFit?> find,
        out VCarveNativePreparedSourceProbe.Audit audit,CancellationToken token=default,int maximum=32)
        =>VCarveNativePreparedSourceProbe.Search(points,Enumerable.Range(0,points.Length).ToArray(),
            points.Select(p=>p.X).ToArray(),.3,.3,2048,find,()=>true,token,out audit,maximum);

    [Fact]
    public void CoarseEndpointsAreBoundedAndTheOriginalSkippedEndpointCanStillSolveTheCompletePath()
    {
        var points=Points(81);var knots=Enumerable.Range(0,81).ToArray();var cumulative=points.Select(p=>p.X).ToArray();
        Assert.Equal(new[]{48,24,12,6,1},VCarveNativePreparedSourceProbe.Endpoints(0,80,knots,cumulative,.3,2048));
        var allowed=new HashSet<(int,int)>{(0,5),(5,53),(53,80)};
        VCarveNativeFit? Find(int a,int b)=>allowed.Contains((a,b))?Edge(points,a,b):null;
        Assert.Null(Probe(points,Find,out var fast));Assert.Equal(5,fast.AttemptedEdges);
        var complete=VCarveNativeMinimumPath.Search(knots,cumulative,.3,2048,Find,default,out _);
        Assert.NotNull(complete);Assert.Equal(allowed.OrderBy(p=>p.Item1),complete.Select(f=>(f.First,f.Last)));
    }

    [Fact]
    public void RealNominationFallsBackToTheOriginalFortyEightEndpointsWithoutUnverifiedOutput()
    {
        var points=Points(81);var knots=Enumerable.Range(0,81).ToArray();
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
        PointD[][] boundary=[[new(-2,-2),new(83,-2),new(83,2),new(-2,2)]];
        var physical=new VCarveNativeContainment(boundary,profile);
        var allowed=new HashSet<(int,int)>{(0,5),(5,53),(53,80)};
        bool Chord(Point3D a,Point3D b)=>physical.Legal(a,b)&&(a==b||allowed.Contains(((int)a.X,(int)b.X)));
        bool Curve(ToolMove move)=>allowed.Contains(((int)move.Start.X,(int)move.End.X))
            &&VCarveNativeCutterProof.Certified(move,physical,profile,out _);
        var path=new TaggedToolpath(points,[],0,"coarse-fallback",0,detectCurves:false);
        var plan=Assert.Single(VCarveNativeCurvePlanning.Nominate([path],[boundary],profile,0,
            limits:new(MaximumCandidateEdges:1000),settings:new(.3,.3)
                {CandidateSelection=VCarveNativeCandidateSelection.FirstVerified,MaximumCandidateGeometryProofs=1},
            stockChord:Chord,stockCurve:Curve,selectedSourceOrdinals:[knots]));
        Assert.True(plan.CandidateVerified);Assert.False(plan.BudgetExhausted);
        Assert.Contains(plan.SearchRejections.Keys,key=>key.StartsWith("prepared-coarse:",StringComparison.Ordinal));
        Assert.Equal(new[]{(0,5),(5,53),(53,80)},plan.Primitives.Select(p=>(p.SourceFirst,p.SourceLast)));
        foreach(var primitive in plan.Primitives)
        {
            Assert.True(primitive.LengthLowerMm>=.3);
            Assert.True(VCarveNativeGeometricProof.Prove(primitive.Move,points,primitive.SourceFirst,primitive.SourceLast,.3).Certified);
            Assert.True(VCarveNativeCutterProof.Certified(primitive.Move,physical,profile,out _));
        }
    }

    [Fact]
    public void FailedSuffixDiscardsTheCompleteProvisionalPrefixBeforeOriginalSearchBacktracks()
    {
        var points=Points(97);var allowed=new HashSet<(int,int)>{(0,48),(0,32),(32,80),(80,96)};
        VCarveNativeFit? Find(int a,int b)=>allowed.Contains((a,b))?Edge(points,a,b):null;
        Assert.Null(Probe(points,Find,out var fast));Assert.Equal(1,fast.ProvisionalEdges);
        var complete=VCarveNativeMinimumPath.Search(Enumerable.Range(0,97).ToArray(),points.Select(p=>p.X).ToArray(),
            .3,2048,Find,default,out _);
        Assert.NotNull(complete);
        Assert.Equal(new[]{(0,32),(32,80),(80,96)},complete.Select(f=>(f.First,f.Last)));
    }

    [Fact]
    public void AShortUnsolvedTailCannotPublishAPartialFastPath()
    {
        Point3D[] points=[new(0,0,-.5),new(1,0,-.5),new(1.1,0,-.5)];
        Assert.Null(Probe(points,(a,b)=>a==0&&b==1?Edge(points,a,b):null,out var fast));
        Assert.Equal(0,fast.ProvisionalEdges);
        Assert.Equal("no-complete-coarse-path",fast.Outcome);
    }

    [Fact]
    public void InvalidMinimumLengthMetadataAndBlockAttemptLimitDiscardFastResults()
    {
        var points=Points(81);
        Assert.Null(Probe(points,(a,b)=>Edge(points,a,b,.1),out var invalid));
        Assert.Equal("certified-edge-contract-mismatch",invalid.Outcome);
        Assert.Null(Probe(points,(_,_)=>null,out var capped,maximum:2));
        Assert.Equal(2,capped.AttemptedEdges);Assert.Equal("block-fast-attempt-limit",capped.Outcome);
        Assert.Null(Probe(points,(a,b)=>Edge(points,a,b) with{
            Move=ToolMove.CubicBezier(points[a],new(double.NaN,0,-.5),points[b],points[b],0)},out var nonfinite));
        Assert.Equal("certified-edge-contract-mismatch",nonfinite.Outcome);
        Assert.Null(Probe(points,(a,b)=>Edge(points,a,b) with{Move=ToolMove.Line(points[a],points[b],-1)},out _));
    }

    [Fact]
    public void CancellationBeforePublicationStillThrowsForACompleteFastCandidate()
    {
        var points=Points(5);using var cancel=new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(()=>Probe(points,(a,b)=>{
            cancel.Cancel();return Edge(points,a,b);
        },out _,cancel.Token));
    }

    [Fact]
    public void FastWorkUsesTheOriginalGlobalBudgetAndKeepsFallbackHeadroom()
    {
        var budget=new VCarveNativeSearchBudget(default,100);
        Assert.Equal(5,budget.FastProbeLimit);
        for(int i=0;i<5;i++)Assert.True(budget.TryCheckFastCandidate());
        Assert.Equal(5,budget.Attempts);Assert.Equal(5,budget.FastProbeAttempts);
        Assert.False(budget.TryCheckFastCandidate());Assert.Equal(95,budget.Remaining);
        var late=new VCarveNativeSearchBudget(default,100);
        for(int i=0;i<51;i++)late.CheckCandidate();
        Assert.False(late.TryCheckFastCandidate());Assert.Equal(0,late.FastProbeAttempts);
        Assert.Equal(49,late.Remaining);
        var tiny=new VCarveNativeSearchBudget(default,1);
        Assert.False(tiny.TryCheckFastCandidate());tiny.CheckCandidate();
        Assert.Throws<VCarveNativeBudgetException>(tiny.CheckCandidate);
    }
}
