using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveCarrierReturnTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PostedEquivalentLiteralReturnRequiresAnExplicitAnchoredAdapter(bool anchored)
    {
        Point3D[] points=[new(101,0,-1),new(102,0,-1),new(102.000001,0,-1.0000005),new(103,0,-1)];
        var path=new TaggedToolpath(points,[new(0,3,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);
        var returns=new VCarveRingReturns(default,postedEquivalentReturns:anchored);returns.Record(path,null);
        var target=new TaggedToolpath([new(101,.3,-1),new(101,.4,-1)],[],0,"clearing",0,detectCurves:false);
        var cost=new ToolpathMotionCost(new(1000,500,1000,20),0);
        var choices=returns.Departures(target,cost,10,.5).ToArray();
        if(!anchored){Assert.Empty(choices);Assert.True(returns.PartialAxisRefusals>0);return;}
        Assert.NotEmpty(choices);Assert.All(choices,choice=>Assert.Equal(points.Reverse(),choice.Points));
        Assert.True(returns.PostedEquivalentSteps>0);
        Assert.Equal(points,path.Points);
    }

    [Fact]
    public void AParentShortcutRequiresTheAdapterProofAndKeepsBothOriginalBodies()
    {
        Point3D[] parentPoints=[new(0,0,-.5),new(5,0,-.5),new(5,5,-.5),new(0,5,-.5),new(0,0,-.5)];
        Point3D[] childPoints=[new(.3,0,-1),new(5.3,0,-1)];
        TaggedToolpath Cut(Point3D[] points)=>new(points,[new(0,points.Length-1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);
        var parent=Cut(parentPoints);var child=Cut(childPoints);var returns=new VCarveRingReturns(default);
        returns.Record(parent,null);returns.Record(child,[parentPoints[0],childPoints[0]]);
        var target=Cut([new(5,.1,-.5),new(5,.2,-.5)]);var cost=new ToolpathMotionCost(new(1000,500,1000,20),0);
        var literal=returns.Departures(target,cost,10,.5).ToArray();
        Assert.DoesNotContain(literal,p=>p.ProvedParentSteps>0);
        var shortcut=returns.Departures(target,cost,10,.5,(a,b)=>a==childPoints[^1]&&b==parentPoints[1]).ToArray();
        Assert.Contains(shortcut,p=>p.ProvedParentSteps==1&&p.Points.SequenceEqual(new[]{childPoints[^1],parentPoints[1]}));
        Assert.Equal(parentPoints,parent.Points);Assert.Equal(childPoints,child.Points);
    }

    [Theory]
    [InlineData(.099,0,false,false)]
    [InlineData(.102,5,false,false)]
    [InlineData(.099,0,true,false)]
    [InlineData(.102,5,true,false)]
    [InlineData(.099,0,false,true)]
    [InlineData(.102,5,false,true)]
    [InlineData(.099,0,true,true)]
    [InlineData(.102,5,true,true)]
    public void UpperCarrierCannotBorrowTheReserveOrTreatAStartingFloorAsOriginalStock(double carrierZ,double originalSurface,bool continuous,bool optimize)
    {
        var plan=new VCarveRingOrder.Plan([-1]);
        var cut=new TaggedToolpath([new(0,0,-1),new(1,0,-1)],[],0,"clearing",0,detectCurves:false){ConcentricSource=new(plan,0)};
        var carrier=new TaggedToolpath([new(0,0,carrierZ),new(1,0,carrierZ),new(0,0,carrierZ)],[new(0,2,ToolpathSpanKind.ClearedTravel)],0,"carrier",0,detectCurves:false);
        Assert.Throws<InvalidOperationException>(()=>VCarvePathRouter.Route([cut],[],
            new ToolGeometry(2){BottomRadius=0,TopRadius=2,ConeLength=4},new DepthSchedule(2,1),
            new VCarveOptions{SmoothVCarving=true,PreferContinuousContourTraversal=continuous,OptimizeContourConnections=optimize,UpperContourCarrier=carrier,Motion=new(1000,500,5000,10,originalSurface)},0,null,
            continuationCutGuard:(_,_)=>true,materialEntryHeight:_=>0));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CostAwareCarrierKeepsBeneficialAdjacentConnections(bool optimize)
    {
        var plan=new VCarveRingOrder.Plan([-1,0]);
        TaggedToolpath Cut(int row,params Point3D[] points)=>new(points,[],0,"clearing",0,detectCurves:false)
            {ConcentricSource=new(plan,row),ContinuationSourceOrdinal=row};
        var paths=new[]{Cut(0,new(0,0,-1),new(20,0,-1)),Cut(1,new(20,.2,-1),new(0,.2,-1))};
        var carrier=new TaggedToolpath([new(0,0,.102),new(20,0,.102),new(20,1,.102),new(0,1,.102),new(0,0,.102)],
            [new(0,4,ToolpathSpanKind.ClearedTravel)],0,"carrier",0,detectCurves:false);
        var motion=new MillingMotionSettings(100,100,5000,2,0);
        List<TaggedToolpath> Route(bool continuous)=>VCarvePathRouter.Route(paths,[],
            new ToolGeometry(2){BottomRadius=0,TopRadius=2,ConeLength=4},new DepthSchedule(2,1),
            new VCarveOptions{SmoothVCarving=true,DetectCurves=false,SmoothRampAngleDegrees=50,StepOver=.45,
                Ramp=RampSettings.FromRatio(2),PreferContinuousContourTraversal=continuous,OptimizeContourConnections=optimize,UpperContourCarrier=carrier,Motion=motion},
            0,null,diagnostic:Console.WriteLine,continuationCutGuard:(_,_)=>true,materialEntryHeight:_=>0);
        var hybrid=Route(false);var forced=Route(true);var cost=new ToolpathMotionCost(motion,0);
        Console.WriteLine($"[VCarve.CarrierCost.Test] hybridSeconds={cost.SequenceSeconds(null,hybrid,null):R} forcedSeconds={cost.SequenceSeconds(null,forced,null):R} hybridPaths={hybrid.Count} forcedPaths={forced.Count}");
        if(optimize)Assert.DoesNotContain(carrier,hybrid);else Assert.Equal(carrier,hybrid[0]);
        Assert.Equal(cost.SequenceSeconds(null,forced,null),cost.SequenceSeconds(null,hybrid,null),9);
        Assert.All(hybrid.Zip(hybrid.Skip(1)),pair=>Assert.Equal(pair.First.Points[^1],pair.Second.Points[0]));
        Assert.All(forced.Zip(forced.Skip(1)),pair=>Assert.Equal(pair.First.Points[^1],pair.Second.Points[0]));
        foreach(var source in paths)Assert.Contains(hybrid,p=>p.ConcentricSource==source.ConcentricSource
            &&source.Points.All(point=>p.Points.Contains(point)));
        Console.WriteLine($"[VCarve.CarrierCost.Test] hybridSeconds={cost.SequenceSeconds(null,hybrid,null):R} forcedSeconds={cost.SequenceSeconds(null,forced,null):R} carrierClearanceIndependentOfCostPreference=True");
    }
}
