using System.Globalization;
using System.Text.RegularExpressions;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveRingCostPrecheckTests
{
    private static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Linear,0,3,6)],3);
    private static readonly PointD[][] Boundary=[[new(-40,-10),new(210,-10),new(210,210),new(-40,210)]];
    private sealed record Run(List<TaggedToolpath> Paths,string Summary,List<(Point3D A,Point3D B)> Guards,
        int EntryHeightQueries,int TravelQueries);

    private static TaggedToolpath Cut(int owner,VCarveRingOrder.Plan plan,int ring,params Point3D[] points)
        =>new(points,[new(0,points.Length-1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false)
        {ContinuationSourceOrdinal=owner,ConcentricSource=new(plan,ring)};

    private static TaggedToolpath[] Sources(double surface)
    {
        var first=new VCarveRingOrder.Plan([-1,0]);
        Point3D P(double x,double y,double z=-1)=>new(x,y,surface+z);
        return [
            Cut(0,first,0,P(0,0),P(2,0),P(2,2),P(0,2),P(0,0)),
            Cut(1,first,1,P(100,0,-1.1),P(102,0,-1.1),P(102,2,-1.1),P(100,2,-1.1),P(100,0,-1.1)),
            // Interior landings pay for the entire long open arm and its
            // recovery. Repeated nearby seams therefore lose on price.
            Cut(2,new([-1]),0,Enumerable.Range(0,61).Select(i=>P(i-30,.2)).ToArray()),
            Cut(3,new([-1]),0,P(2.2,0),P(2.2,2)),
            // Keep the unrelated chain-tour postpass outside this fixture.
            new([P(200,200),P(201,200)],[],0,"support",0,detectCurves:false)
        ];
    }

    private static Run Route(bool enabled,double surface,MillingMotionSettings motion,
        IReadOnlyList<TaggedToolpath> source,bool continuous=false)
    {
        bool? old=VCarveRingRouter.CostPrecheckOverride.Value;
        bool? oldWinner=VCarveRingRouter.IndependentEntryContinuationOverride.Value;
        bool? oldRetrace=VCarvePathRouter.CutRetraceOptimizationOverride.Value;
        try
        {
            VCarveRingRouter.CostPrecheckOverride.Value=enabled;
            VCarveRingRouter.IndependentEntryContinuationOverride.Value=true;
            VCarvePathRouter.CutRetraceOptimizationOverride.Value=false;
            var physical=new VCarveNativeContainment(Boundary,Profile);
            var guards=new List<(Point3D A,Point3D B)>();var logs=new List<string>();
            int heights=0,travels=0;
            bool Guard(Point3D a,Point3D b){guards.Add((a,b));return physical.Legal(a,b);}
            var paths=VCarveRingRouter.Route(source,new ToolGeometry(3){VCarveProfile=Profile},
                new(3,3){SurfaceZ=surface},new VCarveOptions{
                    SmoothVCarving=true,OptimizeContourConnections=true,DetectCurves=false,
                    MaximumContourSeamCandidates=12,PreferContinuousContourTraversal=continuous,
                    Motion=motion with{ClearedTravelGuard=(a,b)=>{travels++;return physical.Legal(
                        a with{Z=a.Z-surface},b with{Z=b.Z-surface});}},
                    Ramp=new(Math.Atan(.5)),StepOver=.45,SmoothRampAngleDegrees=50},
                null,Guard,_=>{heights++;return surface;},default,logs.Add,(other,_)=>other.ToList());
            return new(paths,Assert.Single(logs.Where(s=>s.Contains("stage=ring-routing event=complete"))),guards,heights,travels);
        }
        finally
        {
            VCarveRingRouter.CostPrecheckOverride.Value=old;
            VCarveRingRouter.IndependentEntryContinuationOverride.Value=oldWinner;
            VCarvePathRouter.CutRetraceOptimizationOverride.Value=oldRetrace;
        }
    }

    private static long Count(Run run,string metric)
        =>long.Parse(Regex.Match(run.Summary,$@"(?:^| ){metric}=(\d+)(?: |$)").Groups[1].Value,CultureInfo.InvariantCulture);

    private static void AssertSameTrace(Run control,Run actual)
    {
        Assert.Equal(control.Paths.Count,actual.Paths.Count);
        for(int i=0;i<control.Paths.Count;i++)
        {
            Assert.Equal(control.Paths[i].Points,actual.Paths[i].Points);
            Assert.Equal(control.Paths[i].Spans,actual.Paths[i].Spans);
            Assert.Equal(control.Paths[i].ContinuationSourceOrdinal,actual.Paths[i].ContinuationSourceOrdinal);
            Assert.Equal(control.Paths[i].DepthPassIndex,actual.Paths[i].DepthPassIndex);
            Assert.Equal(control.Paths[i].RegionIndex,actual.Paths[i].RegionIndex);
        }
        Assert.Equal(control.EntryHeightQueries,actual.EntryHeightQueries);
        Assert.Equal(control.TravelQueries,actual.TravelQueries);
    }

    private static void AssertSourceAndPhysicalProofs(Run run,IReadOnlyList<TaggedToolpath> source,double surface)
    {
        var physical=new VCarveNativeContainment(Boundary,Profile);int links=0;
        foreach(var path in run.Paths)
        {
            var kinds=ToolpathSpanKinds.Expand(path);
            for(int i=1;i<path.Points.Count;i++)
            {
                var a=path.Points[i-1] with{Z=path.Points[i-1].Z-surface};
                var b=path.Points[i] with{Z=path.Points[i].Z-surface};
                Assert.True(physical.Legal(a,b),$"Independent full cutter chord proof refused {a} -> {b}.");
                if(kinds[i-1]!=ToolpathSpanKind.Link)continue;
                links++;
                // Exact literal retraces of source chords need no new query.
                bool sourceChord=source.Any(p=>Enumerable.Range(1,p.Points.Count-1).Any(j=>
                    OnSegment(path.Points[i-1],p.Points[j-1],p.Points[j])&&OnSegment(path.Points[i],p.Points[j-1],p.Points[j])));
                Assert.True(sourceChord||run.Guards.Contains((a,b))||run.Guards.Contains((b,a)),
                    $"A new accepted link lacked its full physical callback: {a} -> {b}.");
            }
        }
        Assert.True(links>0,"Fixture must exercise a physically guarded accepted connector.");
        foreach(var original in source.Where(p=>p.ConcentricSource is not null))
        {
            var actual=Assert.Single(run.Paths.Where(p=>p.ContinuationSourceOrdinal==original.ContinuationSourceOrdinal));
            var kinds=ToolpathSpanKinds.Expand(actual);
            for(int i=1;i<original.Points.Count;i++)
            {
                var a=original.Points[i-1];var b=original.Points[i];
                Assert.Contains(Enumerable.Range(1,actual.Points.Count-1),j=>kinds[j-1]==ToolpathSpanKind.Cut
                    &&(actual.Points[j-1]==a&&actual.Points[j]==b||actual.Points[j-1]==b&&actual.Points[j]==a));
            }
        }
    }

    private static bool OnSegment(Point3D p,Point3D a,Point3D b)
    {
        double x=b.X-a.X,y=b.Y-a.Y,z=b.Z-a.Z,length=x*x+y*y+z*z;
        if(length==0)return p==a;
        double t=((p.X-a.X)*x+(p.Y-a.Y)*y+(p.Z-a.Z)*z)/length;
        return t>=0&&t<=1&&Math.Abs(p.X-a.X-t*x)<1e-10
            &&Math.Abs(p.Y-a.Y-t*y)<1e-10&&Math.Abs(p.Z-a.Z-t*z)<1e-10;
    }

    [Fact]
    public void UncompetitiveCachedComparisonsSkipFullGuardsWithoutChangingAcceptedSourceTrace()
    {
        var source=Sources(0);var motion=new MillingMotionSettings(1000,500,5000,5,0);
        var control=Route(false,0,motion,source);var actual=Route(true,0,motion,source);
        AssertSameTrace(control,actual);
        long skips=Count(actual,"costPrecheckGuardSkips");
        Assert.True(skips>0,actual.Summary);
        Assert.Equal(skips,control.Guards.Count-actual.Guards.Count);
        Assert.Equal(0,Count(actual,"additionalPureCosts"));
        Assert.Contains("costPrecheckOrdering=cached-independent-zero-transfer-price-before-full-physical-guard",actual.Summary);
        Assert.Contains("acceptedCandidatesRequireFullGuard=True",actual.Summary);
        AssertSourceAndPhysicalProofs(actual,source,0);
    }

    [Theory]
    [InlineData(0,20000,20000,50)]
    [InlineData(7.5,1000,500,5000)]
    public void PricesRemainOwnedByTheActualRatesStockFrameAndSpanSequence(double surface,double feed,double plunge,double travel)
    {
        var source=Sources(surface);var motion=new MillingMotionSettings(feed,plunge,travel,surface+5,surface);
        var control=Route(false,surface,motion,source);var actual=Route(true,surface,motion,source);
        AssertSameTrace(control,actual);
        var independentCost=new ToolpathMotionCost(motion,surface);
        Assert.Equal(independentCost.SequenceSeconds(null,control.Paths,null),independentCost.SequenceSeconds(null,actual.Paths,null));
        AssertSourceAndPhysicalProofs(actual,source,surface);
    }

    [Fact]
    public void AuthoredContinuousTraversalRetainsItsGuardBeforePricePolicy()
    {
        var source=Sources(0);var motion=new MillingMotionSettings(1000,500,5000,5,0);
        var control=Route(false,0,motion,source,continuous:true);
        var actual=Route(true,0,motion,source,continuous:true);
        AssertSameTrace(control,actual);
        Assert.Equal(control.Guards,actual.Guards);
        Assert.Equal(0,Count(actual,"costPrecheckComparisons"));
        Assert.Equal(0,Count(actual,"costPrecheckGuardSkips"));
        AssertSourceAndPhysicalProofs(actual,source,0);
    }
}
