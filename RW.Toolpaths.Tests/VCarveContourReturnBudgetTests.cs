using System.Text.RegularExpressions;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveContourReturnBudgetTests
{
    [Fact]
    public void LibraryDefaultRetainsTheUnboundedNormalSearch()
        =>Assert.Equal(int.MaxValue,new VCarveOptions().MaximumContourReturnCandidates);

    [Theory]
    [InlineData(-1)]
    [InlineData(int.MinValue)]
    public void GenerationRejectsANegativeReturnBudget(int budget)
        =>Assert.Throws<ArgumentOutOfRangeException>(()=>VCarveToolpaths.Generate([],Tool(),new(2,2),
            new VCarveOptions{MaximumContourReturnCandidates=budget},BoostVoronoiProvider.CreateDefault()));

    [Fact]
    public void OneBudgetStopsEveryLazyFamilyBeforeAnotherExpensiveMoveNext()
    {
        var budget=new VCarveRingRouter.ReturnNominationBudget(4,default);
        int advanced=0,disposed=0;
        IEnumerable<int> ExpensiveAncestorFamily()
        {
            try{while(true){advanced++;yield return advanced;}}
            finally{disposed++;}
        }
        Assert.Equal(new[]{1,2},budget.Take(ExpensiveAncestorFamily()).Take(2));
        Assert.False(budget.Refused);Assert.Equal(2,budget.Used);
        Assert.Equal(new[]{3,4},budget.Take(ExpensiveAncestorFamily()));
        Assert.Empty(budget.Take(ExpensiveAncestorFamily()));
        Assert.Equal(4,advanced);Assert.Equal(2,disposed);
        Assert.True(budget.Refused);Assert.Equal(4,budget.Used);
    }

    [Fact]
    public void ZeroBudgetNeverStartsAnAncestorAndCancellationIsStillObserved()
    {
        using var cancellation=new CancellationTokenSource();
        var budget=new VCarveRingRouter.ReturnNominationBudget(0,cancellation.Token);
        int advanced=0;
        IEnumerable<int> Ancestor(){advanced++;yield return 1;}
        Assert.Empty(budget.Take(Ancestor()));Assert.Equal(0,advanced);
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(()=>budget.Take(Ancestor()).ToArray());
        Assert.Equal(0,advanced);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(4)]
    public void RefusedReturnSearchKeepsIndependentEntriesAndEveryRequiredSourceEdge(int budget)
    {
        var source=Rings();var log=new List<string>();int fullGuards=0;
        var routed=Route(source,budget,(_,_)=>{fullGuards++;return false;},log);
        Assert.Equal(source.Length,routed.Count);Assert.True(fullGuards>0);
        var report=Assert.Single(log.Where(l=>l.Contains("stage=ring-routing event=complete")));
        Assert.Contains($"maximumReturnNominationsPerVisit={budget} ",report);
        Assert.Contains("iteratorAdvanceAfterBudget=False ",report);
        Assert.Equal(budget,Counter(report,"maximumReturnNominationsObserved"));
        Assert.True(Counter(report,"returnNominationBudgetRefusals")>0);
        Assert.DoesNotContain(routed.SelectMany(p=>p.Spans),s=>s.Kind==ToolpathSpanKind.Link);
        AssertRequiredCuts(source,routed);
    }

    [Fact]
    public void AReducedReturnSearchStillProvesEveryIntroducedConnector()
    {
        var source=Rings();var log=new List<string>();
        bool Guard(Point3D a,Point3D b)=>new[]{a,b}.All(p=>Math.Abs(p.X)<=3&&Math.Abs(p.Y)<=3&&p.Z>=-2&&p.Z<=0)
            &&Math.Abs(a.Z-b.Z)<=Math.Sqrt(Math.Pow(a.X-b.X,2)+Math.Pow(a.Y-b.Y,2))+1e-9;
        var routed=Route(source,4,Guard,log);int links=0;
        AssertRequiredCuts(source,routed);
        foreach(var path in routed)
        foreach(var span in path.Spans.Where(s=>s.Kind==ToolpathSpanKind.Link))
        for(int i=span.StartIndex+1;i<=span.EndIndex;i++)
        {Assert.True(Guard(path.Points[i-1],path.Points[i]));links++;}
        Assert.True(links>0);
    }

    static long Counter(string report,string name)
        =>long.Parse(Regex.Match(report,$@"\b{name}=(\d+)").Groups[1].Value);
    static List<TaggedToolpath> Route(IReadOnlyList<TaggedToolpath> source,int budget,
        Func<Point3D,Point3D,bool> guard,List<string> log)
        =>VCarveRingRouter.Route(source,Tool(),new(2,2),new VCarveOptions
        {
            SmoothVCarving=true,SmoothRampAngleDegrees=45,StepOver=.45,Ramp=new(.1),
            MaximumContourSeamCandidates=1,MaximumContourReturnCandidates=budget,
            OptimizeContourConnections=false,Motion=new(1000,500,5000,5,0)
        },null,guard,p=>p.Z,default,log.Add,(_,_)=>[]);
    static ToolGeometry Tool()=>new(2){BottomRadius=0,TopRadius=2,ConeLength=4};
    static TaggedToolpath[] Rings()
    {
        var plan=new VCarveRingOrder.Plan([-1,0]);
        return Enumerable.Range(0,2).Select(owner=>
        {
            double radius=owner==0?1:.9,z=owner==0?-1:-1.1;
            var points=Enumerable.Range(0,513).Select(i=>i==512?new Point3D(radius,0,z):
                new Point3D(radius*Math.Cos(i*Math.Tau/512),radius*Math.Sin(i*Math.Tau/512),z)).ToArray();
            return new TaggedToolpath(points,[new(0,points.Length-1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false)
                {ConcentricSource=new(plan,owner),ContinuationSourceOrdinal=owner};
        }).ToArray();
    }
    static void AssertRequiredCuts(IReadOnlyList<TaggedToolpath> source,IReadOnlyList<TaggedToolpath> routed)
    {
        Assert.Equal(source.Select(p=>p.ContinuationSourceOrdinal),routed.Select(p=>p.ContinuationSourceOrdinal));
        foreach(var original in source)
        {
            var path=Assert.Single(routed.Where(p=>p.ContinuationSourceOrdinal==original.ContinuationSourceOrdinal));
            var kinds=ToolpathSpanKinds.Expand(path);
            for(int i=1;i<original.Points.Count;i++)
                Assert.Contains(Enumerable.Range(1,path.Points.Count-1),j=>kinds[j-1]==ToolpathSpanKind.Cut
                    &&path.Points[j-1]==original.Points[i-1]&&path.Points[j]==original.Points[i]);
        }
    }
}
