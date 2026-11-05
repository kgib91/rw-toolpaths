using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveReturnTravelTests
{
    private static readonly VCarveCutterProfile Cone=new([new(CarvingCutterSectionKind.Linear,0,3,6)],5);
    private static readonly MillingMotionSettings Motion=new(1000,500,5000,5,0);
    private static TaggedToolpath Path(double y,ToolpathSpanKind kind,bool reverse=false)
    {
        var points=Enumerable.Range(0,21).Select(i=>new Point3D(reverse?20-i:i,y,-3)).ToArray();
        return new(points,[new(0,20,kind)],0,"clearing",0,detectCurves:false);
    }
    private static List<TaggedToolpath> Improve(IReadOnlyList<TaggedToolpath> source,Func<Point3D,Point3D,bool>? guard=null)
        =>VCarveReturnTravel.Improve(source,Cone,0,Motion,1.2,guard??((_,_)=>true),default,Console.WriteLine);

    [Fact]
    public void ACompletedLiteralReturnCanLiftAndTravelFasterWhileCutsStayExact()
    {
        var cut=Path(0,ToolpathSpanKind.Cut);var back=Path(0,ToolpathSpanKind.Link,true);var source=new[]{cut,back};
        var result=Improve(source);
        Assert.Equal(cut.Points,result[0].Points);Assert.Equal(back.Points[0],result[1].Points[0]);Assert.Equal(back.Points[^1],result[1].Points[^1]);
        Assert.Contains(result[1].Spans,s=>s.Kind==ToolpathSpanKind.ClearedTravel);
        Assert.True(new ToolpathMotionCost(Motion,0).SequenceSeconds(null,result,null)<new ToolpathMotionCost(Motion,0).SequenceSeconds(null,source,null));
        VCarveReturnTravel.VerifyFinalTravel(result,Cone,0,default,Console.WriteLine);
        Assert.All(back.Points,p=>Assert.Equal(-3,p.Z));
    }

    [Fact]
    public void FutureCutsAndFreshConnectorGeometryCannotAuthorizeARaisedReturn()
    {
        var back=Path(0,ToolpathSpanKind.Link,true);var future=Path(0,ToolpathSpanKind.Cut);
        Assert.Equal(back.Points,Improve(new[]{back,future})[0].Points);
        var fresh=Path(.01,ToolpathSpanKind.Link,true);
        Assert.Equal(fresh.Points,Improve(new[]{future,fresh})[1].Points);
    }

    [Fact]
    public void OriginalPhysicalGuardCanRefuseEveryLiftWithoutLosingRequiredMotion()
    {
        var cut=Path(0,ToolpathSpanKind.Cut);var back=Path(0,ToolpathSpanKind.Link,true);
        var result=Improve(new[]{cut,back},(_,_)=>false);
        Assert.Equal(back.Points,result[1].Points);Assert.DoesNotContain(result[1].Spans,s=>s.Kind==ToolpathSpanKind.ClearedTravel);
    }

    [Fact]
    public void FinalReplayRejectsAFalseFastTagAndDoesNotCreditItsFutureCut()
    {
        var falseFast=Path(0,ToolpathSpanKind.ClearedTravel);var future=Path(0,ToolpathSpanKind.Cut,true);
        Assert.Throws<InvalidOperationException>(()=>VCarveReturnTravel.VerifyFinalTravel([falseFast,future],Cone,0,default,Console.WriteLine));
    }

    [Fact]
    public void RepeatedContactAtTheSameDepthHasNoPositiveTravelReserve()
    {
        var cut=Path(0,ToolpathSpanKind.Cut);var contact=Path(0,ToolpathSpanKind.ClearedTravel,true);
        Assert.Throws<InvalidOperationException>(()=>VCarveReturnTravel.VerifyFinalTravel([cut,contact],Cone,0,default,Console.WriteLine));
    }

    [Fact]
    public void OriginalMaterialTopCannotBeReplacedByTheCommandDatum()
    {
        var fast=Path(0,ToolpathSpanKind.ClearedTravel) with{Points=[new(0,0,.2),new(1,0,.2)],Spans=[new(0,1,ToolpathSpanKind.ClearedTravel)]};
        Assert.Throws<InvalidOperationException>(()=>VCarveReturnTravel.VerifyFinalTravel([fast],Cone,0,default,Console.WriteLine,originalMaterialSurface:5));
        VCarveReturnTravel.VerifyFinalTravel([fast],Cone,0,default,Console.WriteLine);
    }

    [Fact]
    public void CancellationStopsTheReturnSearch()
    {
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        Assert.Throws<OperationCanceledException>(()=>VCarveReturnTravel.Improve([Path(0,ToolpathSpanKind.Link)],Cone,0,Motion,1.2,(_,_)=>true,cancel.Token,null));
    }

    [Fact]
    public void AnAmbiguousIncomingPartialAxisAnchorRetainsTheOriginalReturn()
    {
        var cut=Path(0,ToolpathSpanKind.Cut);var back=Path(0,ToolpathSpanKind.Link,true);var source=new[]{cut,back};
        var result=VCarveReturnTravel.Improve(source,Cone,0,Motion,1.2,(_,_)=>true,default,Console.WriteLine,incoming:new(.00004,0,-3));
        Assert.Equal(back.Points,result[1].Points);Assert.DoesNotContain(result[1].Spans,s=>s.Kind==ToolpathSpanKind.ClearedTravel);
    }

    [Fact]
    public void RegionReorderingCannotInvalidateTheFirstPostedXYAnchorOfAProvedFastBlock()
    {
        TaggedToolpath Row(int region,Point3D a,Point3D b,bool fast=false)=>new([a,b],[new(0,1,fast?ToolpathSpanKind.ClearedTravel:ToolpathSpanKind.Cut)],region,"clearing",0,detectCurves:false);
        var paths=new List<TaggedToolpath>{Row(0,new(100,0,-3),new(0,0,-3)),Row(1,new(50,0,-3),new(50,1,-3)),
            Row(2,new(.00004,0,-3),new(50,1,-3),true)};
        var result=VCarvePathRouter.OrderRegions(paths,default,Console.WriteLine);
        Assert.Equal(new[]{0,1,2},result.Select(p=>p.RegionIndex));
    }
    [Fact]
    public void AnExecutedCutRetraceCanLiftButItsFirstCutStaysExact()
    {
        var cut=Path(0,ToolpathSpanKind.Cut);var replay=Path(0,ToolpathSpanKind.Cut,true);
        var result=VCarveReturnTravel.Improve([cut,replay],Cone,0,Motion,1.2,(_,_)=>true,default,Console.WriteLine,optimizeCutRetraces:true);
        Assert.Equal(cut.Points,result[0].Points);Assert.Equal(cut.Spans,result[0].Spans);
        Assert.Equal(replay.Points[0],result[1].Points[0]);Assert.Equal(replay.Points[^1],result[1].Points[^1]);
        Assert.Contains(result[1].Spans,s=>s.Kind==ToolpathSpanKind.ClearedTravel);
        VCarveReturnTravel.VerifyFinalTravel(result,Cone,0,default,Console.WriteLine);
    }
    [Fact]
    public void FutureAndNearbyCutsCannotAuthorizeChangingACutRetrace()
    {
        var first=Path(0,ToolpathSpanKind.Cut,true);var future=Path(0,ToolpathSpanKind.Cut);
        var result=VCarveReturnTravel.Improve([first,future],Cone,0,Motion,1.2,(_,_)=>true,default,null,optimizeCutRetraces:true);
        Assert.Equal(first.Points,result[0].Points);Assert.Equal(first.Spans,result[0].Spans);
        var nearby=Path(.01,ToolpathSpanKind.Cut,true);
        var off=VCarveReturnTravel.Improve([future,nearby],Cone,0,Motion,1.2,(_,_)=>true,default,null,optimizeCutRetraces:true);
        Assert.Equal(nearby.Points,off[1].Points);
    }
    [Fact]
    public void APhysicalGuardRefusalKeepsARepeatedCutInItsOriginalMode()
    {
        var cut=Path(0,ToolpathSpanKind.Cut);var replay=Path(0,ToolpathSpanKind.Cut,true);
        var result=VCarveReturnTravel.Improve([cut,replay],Cone,0,Motion,1.2,(_,_)=>false,default,null,optimizeCutRetraces:true);
        Assert.Equal(replay.Points,result[1].Points);Assert.Equal(replay.Spans,result[1].Spans);
    }
    [Fact]
    public void AParityReturnInsideOneCutCircuitUsesOnlyItsExecutedForwardHalf()
    {
        var forward=Path(0,ToolpathSpanKind.Cut);
        var points=forward.Points.Concat(forward.Points.Reverse().Skip(1)).ToArray();
        var circuit=forward with{Points=points,Moves=new LineToolMoves(points),Spans=[new(0,points.Length-1,ToolpathSpanKind.Cut)]};
        var result=VCarveReturnTravel.Improve([circuit],Cone,0,Motion,1.2,(_,_)=>true,default,null,optimizeCutRetraces:true);
        Assert.Equal(forward.Points,result[0].Points.Take(forward.Points.Count));
        Assert.Contains(result[0].Spans,s=>s.Kind==ToolpathSpanKind.ClearedTravel);
        VCarveReturnTravel.VerifyFinalTravel(result,Cone,0,default,Console.WriteLine);
    }

    [Fact]
    public void ACompletedTailCanEndBeforeAnExistingIndependentReentry()
    {
        var forward=Path(0,ToolpathSpanKind.Cut);
        var points=forward.Points.Concat(forward.Points.Reverse().Skip(1)).ToArray();
        var circuit=forward with{Points=points,Moves=new LineToolMoves(points),Spans=[new(0,points.Length-1,ToolpathSpanKind.Cut)]};
        var next=Path(10,ToolpathSpanKind.Cut);
        var result=VCarveReturnTravel.Improve([circuit,next],Cone,0,Motion,1.2,(_,_)=>true,default,Console.WriteLine,
            optimizeCutRetraces:true,trimCompletedExits:true);
        Assert.Equal(forward.Points,result[0].Points);
        Assert.Equal(next.Points,result[1].Points);
        Assert.True(MillingTransitionPlanner.RequiresPlanarTraverse(result[0].Points[^1],next.Points[0]));
        VCarveReturnTravel.VerifyFinalTravel(result,Cone,0,default,Console.WriteLine);
    }

    [Fact]
    public void AContinuousHandoffCannotBeTrimmedAndItsFirstCutRemainsExact()
    {
        var forward=Path(0,ToolpathSpanKind.Cut);
        var points=forward.Points.Concat(forward.Points.Reverse().Skip(1)).ToArray();
        var circuit=forward with{Points=points,Moves=new LineToolMoves(points),Spans=[new(0,points.Length-1,ToolpathSpanKind.Cut)]};
        var next=new TaggedToolpath([points[^1],new(0,1,-3)],[new(0,1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);
        var result=VCarveReturnTravel.Improve([circuit,next],Cone,0,Motion,1.2,(_,_)=>true,default,null,
            optimizeCutRetraces:true,trimCompletedExits:true);
        Assert.Equal(points.Length,result[0].Points.Count);
        Assert.Equal(next.Points[0],result[0].Points[^1]);
        Assert.Equal(forward.Points,result[0].Points.Take(forward.Points.Count));
        VCarveReturnTravel.VerifyFinalTravel(result,Cone,0,default,null);
    }

    [Fact]
    public void AGuardRefusalCannotTrimACompletedFinalReturn()
    {
        var forward=Path(0,ToolpathSpanKind.Cut);
        var points=forward.Points.Concat(forward.Points.Reverse().Skip(1)).ToArray();
        var circuit=forward with{Points=points,Moves=new LineToolMoves(points),Spans=[new(0,points.Length-1,ToolpathSpanKind.Cut)]};
        var result=VCarveReturnTravel.Improve([circuit],Cone,0,Motion,1.2,(_,_)=>false,default,null,
            optimizeCutRetraces:true,trimCompletedExits:true);
        Assert.Equal(points,result[0].Points);
        Assert.Equal(circuit.Spans,result[0].Spans);
    }

    [Fact]
    public void TheLastRequiredCutCannotBeDiscardedAsAnExitReturn()
    {
        var forward=Path(0,ToolpathSpanKind.Cut);
        var points=forward.Points.Concat(forward.Points.Reverse().Skip(1)).Append(new Point3D(0,1,-3)).ToArray();
        var circuit=forward with{Points=points,Moves=new LineToolMoves(points),Spans=[new(0,points.Length-1,ToolpathSpanKind.Cut)]};
        var result=VCarveReturnTravel.Improve([circuit],Cone,0,Motion,1.2,(_,_)=>true,default,null,
            optimizeCutRetraces:true,trimCompletedExits:true);
        Assert.Equal(points.Length,result[0].Points.Count);
        Assert.Equal(points[^2],result[0].Points[^2]);Assert.Equal(points[^1],result[0].Points[^1]);
    }
}
