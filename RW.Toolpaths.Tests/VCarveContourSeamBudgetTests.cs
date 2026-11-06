using System.Text.RegularExpressions;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveContourSeamBudgetTests
{
    [Fact]
    public void LibraryDefaultKeepsTheFullReferenceSearchBudget()
        =>Assert.Equal(48,new VCarveOptions().MaximumContourSeamCandidates);

    [Theory]
    [InlineData(0)]
    [InlineData(49)]
    public void GenerationRejectsAnInvalidCandidateBudget(int budget)
        =>Assert.Throws<ArgumentOutOfRangeException>(()=>VCarveToolpaths.Generate([],Tool(),new(2,2),
            new VCarveOptions{MaximumContourSeamCandidates=budget},BoostVoronoiProvider.CreateDefault()));

    [Theory]
    [InlineData(1)]
    [InlineData(3)]
    [InlineData(12)]
    [InlineData(48)]
    public void RefusedConnectionsBoundActualWorkAndRetainEveryRequiredSourceEdge(int budget)
    {
        var source=Rings();var log=new List<string>();int guardQueries=0;
        var routed=Route(source,budget,(_,_)=>{guardQueries++;return false;},log);
        Assert.Equal(2,routed.Count);Assert.True(guardQueries>0);
        string report=Assert.Single(log.Where(l=>l.Contains("stage=ring-routing event=complete")));
        Assert.Contains($"maximumLandingTrialsPerDeparture={budget} ",report);
        Assert.Equal(budget,int.Parse(Regex.Match(report,@"maximumLandingTrialsObserved=(\d+)").Groups[1].Value));
        Assert.True(long.Parse(Regex.Match(report,@"landingWorkRefusals=(\d+)").Groups[1].Value)>0);
        foreach(var original in source)
        {
            var path=Assert.Single(routed.Where(p=>p.ContinuationSourceOrdinal==original.ContinuationSourceOrdinal));
            var kinds=ToolpathSpanKinds.Expand(path);
            for(int i=1;i<original.Points.Count;i++)
                Assert.Contains(Enumerable.Range(1,path.Points.Count-1),j=>kinds[j-1]==ToolpathSpanKind.Cut
                    &&path.Points[j-1]==original.Points[i-1]&&path.Points[j]==original.Points[i]);
        }
        Assert.DoesNotContain(routed.SelectMany(p=>p.Spans),s=>s.Kind==ToolpathSpanKind.Link);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(3)]
    [InlineData(12)]
    [InlineData(48)]
    public void AReducedSearchStillProvesEveryIntroducedCuttingConnector(int budget)
    {
        var source=Rings();var log=new List<string>();
        bool Guard(Point3D a,Point3D b)=>new[]{a,b}.All(p=>Math.Abs(p.X)<=3&&Math.Abs(p.Y)<=3&&p.Z>=-2&&p.Z<=0)
            &&Math.Abs(a.Z-b.Z)<=Math.Sqrt(Math.Pow(a.X-b.X,2)+Math.Pow(a.Y-b.Y,2))+1e-9;
        var routed=Route(source,budget,Guard,log);
        Assert.Equal(2,routed.Count);
        Assert.Equal(new[]{0,1},routed.Select(p=>p.ContinuationSourceOrdinal!.Value));
        int links=0;
        foreach(var path in routed)
        foreach(var span in path.Spans.Where(s=>s.Kind==ToolpathSpanKind.Link))
        for(int i=span.StartIndex+1;i<=span.EndIndex;i++)
        {Assert.True(Guard(path.Points[i-1],path.Points[i]));links++;}
        Assert.True(links>0);
    }

    static List<TaggedToolpath> Route(IReadOnlyList<TaggedToolpath> source,int budget,
        Func<Point3D,Point3D,bool> guard,List<string> log)
        =>VCarveRingRouter.Route(source,Tool(),new(2,2),new VCarveOptions
        {
            SmoothVCarving=true,SmoothRampAngleDegrees=45,StepOver=.45,Ramp=new(.1),
            MaximumContourSeamCandidates=budget,OptimizeContourConnections=false,
            Motion=new(1000,500,5000,5,0)
        },null,guard,p=>p.Z,default,log.Add,(_,_)=>[]);

    static ToolGeometry Tool()=>new(2){BottomRadius=0,TopRadius=2,ConeLength=4};
    static TaggedToolpath[] Rings()
    {
        var plan=new VCarveRingOrder.Plan([-1,0]);
        return Enumerable.Range(0,2).Select(owner=>
        {
            double radius=owner==0?1:.9,z=owner==0?-1:-1.1;
            // Supply more than 48 reachable nominations so both budgets bind.
            var points=Enumerable.Range(0,513).Select(i=>i==512?new Point3D(radius,0,z):
                new Point3D(radius*Math.Cos(i*Math.Tau/512),radius*Math.Sin(i*Math.Tau/512),z)).ToArray();
            return new TaggedToolpath(points,[new(0,points.Length-1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false)
            {ConcentricSource=new(plan,owner),ContinuationSourceOrdinal=owner};
        }).ToArray();
    }
}
