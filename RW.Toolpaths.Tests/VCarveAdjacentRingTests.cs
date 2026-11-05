using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveAdjacentRingTests
{
    [Fact]
    public void AContactCannotMergeDifferentConstructedRowsAndLoseTheirOrder()
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1,0});
        var a=Cut(new(0,0,-1),new(2,0,-1)) with{ConcentricSource=new(plan,0)};
        var b=Cut(new(1,-1,-1),new(1,1,-1)) with{ConcentricSource=new(plan,1)};
        var log=new List<string>();
        var result=ToolpathSourceTraversal.Detours([a,b],default,log.Add,(_,_)=>true);
        Assert.Equal(2,result.Count);
        Assert.Equal(new[]{0,1},result.Select(p=>p.ConcentricSource!.Ring));
        var scheduler=new VCarveRingOrder(result,false,default);
        scheduler.Complete(0);scheduler.Complete(1);
    }
    [Fact]
    public void ContactAtAnInteriorRepeatOfTheTerminalPoseKeepsTheRemainingExcursion()
    {
        Point3D[] source=[new(0,0,-1),new(1,0,-1),new(2,0,-1),new(3,0,-1),new(2,0,-1),new(1,0,-1)];
        var result=RampPlanner.PlanPathEntry(source,0,new RampSettings(Math.PI/4),default,null,
            landAtSourceVertex:true,preserveClosedExit:false,preserveOpenExit:false);
        var cuts=result.Spans.Where(s=>s.Kind==ToolpathSpanKind.Cut)
            .SelectMany(s=>Enumerable.Range(s.StartIndex+1,s.MoveCount).Select(i=>(result.Points[i-1],result.Points[i]))).ToArray();
        for(int i=1;i<source.Length;i++)
            Assert.Contains(cuts,p=>p.Item1==source[i-1]&&p.Item2==source[i]||p.Item2==source[i-1]&&p.Item1==source[i]);
        Assert.Contains(result.Points,p=>p==source[3]);
        Console.WriteLine("[VCarve.OpenEntry.Test] repeatedTerminalPoseIsNotTerminalOrdinal=True remainingExcursionRetained=True");
    }
    private static TaggedToolpath Cut(params Point3D[] points) =>
        new(points, Array.Empty<ToolpathSpan>(), 0, "clearing", 0, detectCurves: false);

    [Fact]
    public void CompletedOpenFragmentCanReturnToAnInteriorLiteralDoorWithoutWrapping()
    {
        Point3D[] points=[new(0,0,-1),new(5,0,-1),new(10,0,-1),new(10,5,-1)];
        var path=Cut(points) with{Spans=new[]{new ToolpathSpan(0,3,ToolpathSpanKind.Cut)}};
        var returns=new VCarveRingReturns(default);returns.Record(path,null);
        var cost=new ToolpathMotionCost(new MillingMotionSettings(1000,500,1000,20),0);
        var target=Cut(new(5,.3,-1),new(5,1,-1));
        var departure=Assert.Single(returns.Departures(target,cost,10,.5));
        Assert.Equal(new[]{points[3],points[2],points[1]},departure.Points);
        Assert.Equal(.6,departure.Seconds,9);
        Assert.DoesNotContain(points[0],departure.Points);
        Assert.Equal(points,path.Points);
    }

    [Fact]
    public void AReturnOmitsAnAlreadyExecutedExactPoseLoopWithoutAddingAChord()
    {
        Point3D[] points=[new(0,0,-1),new(5,0,-1),new(5,5,-1),new(5,0,-1),new(10,0,-1)];
        var path=Cut(points) with{Spans=new[]{new ToolpathSpan(0,4,ToolpathSpanKind.Cut)}};
        var returns=new VCarveRingReturns(default);returns.Record(path,null);
        var cost=new ToolpathMotionCost(new MillingMotionSettings(1000,500,1000,20),0);
        var departure=Assert.Single(returns.Departures(Cut(new(0,.3,-1),new(0,1,-1)),cost,1,.5));
        Assert.Equal(new[]{points[4],points[3],points[0]},departure.Points);
        Assert.Equal(.6,departure.Seconds,9);
        foreach(var (a,b) in departure.Points.Zip(departure.Points.Skip(1)))
            Assert.Contains(Enumerable.Range(1,points.Length-1),i=>points[i-1]==b&&points[i]==a);
        Assert.Equal(5,path.Points.Count);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FrontierRetainsParentAndCompleteDepthDependencies(bool bottomUp)
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1,0,0,1});
        var paths=Enumerable.Range(0,4).SelectMany(r=>Enumerable.Range(0,2).Select(pass=>
            Cut(new(r*3,0,-pass-1),new(r*3+1,0,-pass-1)) with
            {ConcentricSource=new(plan,r),DepthPassIndex=pass,ContinuationSourceOrdinal=r})).ToArray();
        var scheduler=new VCarveRingOrder(paths,bottomUp,default);
        var done=new List<int>();
        for(int i=0;i<paths.Length;i++)
        {
            int next=scheduler.Candidates(new(20,0,0))[0];
            int ring=paths[next].ConcentricSource!.Ring,parent=plan.Parents[ring];
            if(!bottomUp&&parent>=0)Assert.All(Enumerable.Range(0,paths.Length).Where(j=>paths[j].ConcentricSource!.Ring==parent),j=>Assert.Contains(j,done));
            if(bottomUp)Assert.All(Enumerable.Range(0,paths.Length).Where(j=>plan.Parents[paths[j].ConcentricSource!.Ring]==ring),j=>Assert.Contains(j,done));
            if(paths[next].DepthPassIndex==1)Assert.Contains(next-1,done);
            done.Add(next);scheduler.Complete(next);
        }
        Assert.Equal(paths.Length,done.Distinct().Count());
        Console.WriteLine($"[VCarve.RingFrontier.Test] bottomUp={bottomUp} order={string.Join(',',done)} parentAndDepthDependencies=True");
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void RingRouterKeepsEveryCutAndOnlyConnectsAProvedNeighbour(bool corridor)
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1,0});
        var a=Cut(new(0,0,-1),new(5,0,-1),new(5,5,-1),new(0,5,-1),new(0,0,-1)) with{ConcentricSource=new(plan,0),ContinuationSourceOrdinal=0};
        var b=Cut(new(.3,.3,-1.2),new(4.7,.3,-1.2),new(4.7,4.7,-1.2),new(.3,4.7,-1.2),new(.3,.3,-1.2)) with{ConcentricSource=new(plan,1),ContinuationSourceOrdinal=1};
        var log=new List<string>();
        var result=VCarvePathRouter.Route([a,b],[new Clipper2Lib.PointD[]{new(-3,-3),new(8,-3),new(8,8),new(-3,8)}],
            new ToolGeometry(2){TipAngleRadians=Math.PI/3,BottomRadius=0,TopRadius=2,ConeLength=4},new DepthSchedule(2,1),
            new VCarveOptions{SmoothVCarving=true,DetectCurves=false,StepOver=.45,Ramp=new RampSettings(.1)},0,null,
            diagnostic:log.Add,continuationCutGuard:(_,_)=>corridor,materialEntryHeight:p=>p.Z);
        Assert.Equal(2,result.Count);
        Assert.Equal(corridor,result[0].Points[^1]==result[1].Points[0]);
        for(int owner=0;owner<2;owner++)
        {
            var original=owner==0?a:b;var routed=result.Single(p=>p.ContinuationSourceOrdinal==owner);
            for(int i=1;i<original.Points.Count;i++)
                Assert.Contains(Enumerable.Range(1,routed.Points.Count-1),j=>routed.Points[j-1]==original.Points[i-1]&&routed.Points[j]==original.Points[i]);
        }
        Assert.Contains(log,l=>l.Contains("stage=ring-routing")&&l.Contains("executedChordGraph=False"));
        foreach(var span in result[1].Spans.Where(s=>s.Kind==ToolpathSpanKind.Link))
        for(int i=span.StartIndex+1;i<=span.EndIndex;i++)
        {
            var first=result[1].Points[i-1];var last=result[1].Points[i];
            Assert.True(first.Z-last.Z<=Math.Tan(Math.PI/6)*Math.Sqrt(Math.Pow(first.X-last.X,2)+Math.Pow(first.Y-last.Y,2))+1e-9);
        }
    }

    [Fact]
    public void APartialFinalVerticalCannotInventAnExecutedXYDeparture()
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1,0});
        var first=Cut(new(100,0,-1),new(101,0,-1),new(101.000001,0,-.9999995)) with{ConcentricSource=new(plan,0)};
        var second=Cut(new(101.1,0,-1),new(102,0,-1)) with{ConcentricSource=new(plan,1)};
        var log=new List<string>();
        var result=VCarvePathRouter.Route([first,second],[],
            new ToolGeometry(2){TipAngleRadians=Math.PI/3,BottomRadius=0,TopRadius=2,ConeLength=4},
            new DepthSchedule(2,1),new VCarveOptions{SmoothVCarving=true,DetectCurves=false},0,null,
            diagnostic:log.Add,continuationCutGuard:(_,_)=>true,materialEntryHeight:p=>p.Z);
        Assert.NotEqual(result[0].Points[^1],result[1].Points[0]);
        Assert.Contains(log,l=>l.Contains("exitCustodyRefusals=1 "));
    }

    [Theory]
    [InlineData(20,true)]
    [InlineData(0.1,false)]
    public void CompletedParentArcsReplaceReentryOnlyWhenCompleteMotionIsCheaper(double safe,bool returnCheaper)
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1,0,0});
        var parent=Cut(new(0,0,-1),new(5,0,-1),new(5,5,-1),new(0,5,-1),new(0,0,-1)) with{ConcentricSource=new(plan,0),ContinuationSourceOrdinal=0};
        TaggedToolpath Child(int ring,double x)=>Cut(new(x,.3,-1.2),new(x+1,.3,-1.2),new(x+1,1.3,-1.2),new(x,1.3,-1.2),new(x,.3,-1.2))
            with{ConcentricSource=new(plan,ring),ContinuationSourceOrdinal=ring};
        var log=new List<string>();
        var result=VCarvePathRouter.Route([parent,Child(1,.3),Child(2,3.7)],[],
            new ToolGeometry(2){BottomRadius=0,TopRadius=2,ConeLength=4},new DepthSchedule(2,1),
            new VCarveOptions{SmoothVCarving=true,DetectCurves=false,StepOver=.45,Ramp=new RampSettings(.1),
                Motion=new MillingMotionSettings(1000,returnCheaper?500:5000,returnCheaper?1000:100000,safe)},0,null,
            diagnostic:log.Add,continuationCutGuard:(_,_)=>true,materialEntryHeight:p=>p.Z);
        Assert.Equal(3,result.Count);
        Assert.Equal(returnCheaper,result[1].Points[^1]==result[2].Points[0]);
        Assert.Contains(log,l=>l.Contains("returnedVisits="+(returnCheaper?"1":"0")+" "));
        Assert.Equal(new[]{0,1,2},result.Select(p=>p.ContinuationSourceOrdinal!.Value).Order());
        Console.WriteLine(string.Join(Environment.NewLine,log.Where(l=>l.Contains("stage=ring-routing"))));
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void OnlyAdjacentOwnedRowsUseTheAuthoredSurfaceGrade(bool related)
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1,0});
        var a=Cut(new(0,0,-1),new(5,0,-1),new(5,5,-1),new(0,5,-1),new(0,0,-1)) with{ConcentricSource=new(plan,0)};
        var b=Cut(new(.3,.3,-1.2),new(4.7,.3,-1.2),new(4.7,4.7,-1.2),new(.3,4.7,-1.2),new(.3,.3,-1.2))
            with{ConcentricSource=related?new(plan,1):new(new VCarveRingOrder.Plan(new[]{-1}),0)};
        var result=VCarvePathRouter.Route([a,b],[],new ToolGeometry(2){BottomRadius=0,TopRadius=2,ConeLength=4},
            new DepthSchedule(2,1),new VCarveOptions{SmoothVCarving=true,DetectCurves=false,StepOver=.45,
                SmoothRampAngleDegrees=30,Ramp=new RampSettings(.1)},0,null,
            continuationCutGuard:(_,_)=>true,materialEntryHeight:p=>p.Z);
        Assert.Equal(result[0].Points[^1],result[1].Points[0]);
        Assert.Equal(!related,result[1].Spans.Any(s=>s.Kind==ToolpathSpanKind.Ramp));
        Assert.All(result[1].Points,p=>Assert.True(p.Z>=-1.2));
    }

    [Theory]
    [InlineData(-1.05,true)]
    [InlineData(-.7,false)]
    public void AFailedDeepChordCanUseAProvedRaisedLandingButCannotCrossAnObstruction(double ridge,bool expected)
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1,0});
        var a=Cut(new(0,0,-1),new(5,0,-1),new(5,5,-1),new(0,5,-1),new(0,0,-1)) with{ConcentricSource=new(plan,0)};
        var b=Cut(new(.3,.3,-1.2),new(4.7,.3,-1.2),new(4.7,4.7,-1.2),new(.3,4.7,-1.2),new(.3,.3,-1.2)) with{ConcentricSource=new(plan,1)};
        bool Corridor(Point3D first,Point3D last)
        {
            if(first.X>=.15||last.X<=.15)return true;
            double t=(.15-first.X)/(last.X-first.X);
            return first.Z+t*(last.Z-first.Z)>=ridge;
        }
        var log=new List<string>();
        var result=VCarvePathRouter.Route([a,b],[],new ToolGeometry(2){BottomRadius=0,TopRadius=2,ConeLength=4},
            new DepthSchedule(2,1),new VCarveOptions{SmoothVCarving=true,DetectCurves=false,StepOver=.45,
                SmoothRampAngleDegrees=30,Ramp=new RampSettings(.1)},0,null,
            diagnostic:log.Add,continuationCutGuard:Corridor,materialEntryHeight:p=>p.Z);
        Assert.Equal(expected,result[0].Points[^1]==result[1].Points[0]);
        if(expected)
        {
            Assert.Contains(log,l=>l.Contains("raisedSteps=1 "));
            foreach(var span in result[1].Spans.Where(s=>s.Kind==ToolpathSpanKind.Link))
                Assert.True(Corridor(result[1].Points[span.StartIndex],result[1].Points[span.EndIndex]));
        }
        Assert.Contains(result[1].Spans,s=>s.Kind==ToolpathSpanKind.Cut);
    }

    [Fact]
    public void ASourceThatAlreadyMergedDifferentRowsIsDiagnosedBeforeRouting()
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1,0,1});
        var a=Cut(new(0,0,-1),new(1,0,-1)) with{ConcentricSource=new(plan,0,new[]{0,2})};
        var b=Cut(new(0,1,-1),new(1,1,-1)) with{ConcentricSource=new(plan,1)};
        Assert.Throws<InvalidOperationException>(()=>new VCarveRingOrder(new[]{a,b},false,default));
    }

    [Fact]
    public void ClippedPiecesFollowTheirNextLocalOffsetBeforeADistantPiece()
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1,0});
        var paths=new[]{
            Cut(new(0,0,-1),new(100,0,-1)) with{ConcentricSource=new(plan,0),ContinuationSourceOrdinal=0},
            Cut(new(0,100,-1),new(100,100,-1)) with{ConcentricSource=new(plan,0),ContinuationSourceOrdinal=1},
            Cut(new(50,.4,-1),new(51,.4,-1)) with{ConcentricSource=new(plan,1),ContinuationSourceOrdinal=2},
            Cut(new(50,100.4,-1),new(51,100.4,-1)) with{ConcentricSource=new(plan,1),ContinuationSourceOrdinal=3}};
        var scheduler=new VCarveRingOrder(paths,false,default,.6);
        scheduler.Complete(0);
        var ready=scheduler.Candidates(new(50,.4,-1));
        Assert.Equal(new[]{2},ready);
        scheduler.Complete(2);Assert.Equal(new[]{1},scheduler.Candidates(null));
        scheduler.Complete(1);Assert.Equal(new[]{3},scheduler.Candidates(null));scheduler.Complete(3);
        Console.WriteLine("[VCarve.ConstructedRow.Test] nextLocalOffsetBeforeDistantStrip=True");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NearbyCurrentOffsetFinishesBeforeACloserLaterOffsetDoor(bool floatSource)
    {
        var parents=Enumerable.Repeat(-1,14).ToArray();parents[13]=0;
        var ringPlan=new VCarveRingOrder.Plan(parents);
        var points=Enumerable.Range(0,1024).Select(i=>new Point3D(100*Math.Cos(i*Math.PI/512),100*Math.Sin(i*Math.PI/512),-1)).ToList();
        var door=points[17];points.Add(points[0]);
        if(floatSource)points=points.Select(p=>new Point3D((float)p.X,(float)p.Y,(float)p.Z)).ToList();
        var paths=new List<TaggedToolpath>{Cut(points.ToArray())};
        for(int i=1;i<14;i++)
        {
            double x=door.X+(i==13?.00001:.01*i),y=door.Y;
            paths.Add(Cut(new(x,y,-1),new(x+.001,y,-1),new(x,y+.001,-1),new(x,y,-1)));
        }
        var edge=new ToolpathContinuationComponents.Edge(new(0,door),new(1,paths[1].Points[0]));
        var portalPlan=new ToolpathContinuationComponents.Plan(new[]{edge},paths,default);
        for(int i=0;i<paths.Count;i++)paths[i]=paths[i] with{ConcentricSource=new(ringPlan,i),
            ContinuationSourceOrdinal=i,ContinuationPlan=portalPlan};
        var scheduler=new VCarveRingOrder(paths,false,default);
        Assert.Equal(0,scheduler.Candidates(door)[0]);
        scheduler.Complete(0);var currentOffset=scheduler.Candidates(door);
        Assert.Contains(1,currentOffset);Assert.DoesNotContain(13,currentOffset);
        Console.WriteLine($"[VCarve.RingDoor.Test] floatSource={floatSource} constructionOrderRetained=True");
    }
}
