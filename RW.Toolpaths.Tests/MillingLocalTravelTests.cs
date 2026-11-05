using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class MillingLocalTravelTests
{
    private static VCarveCutterProfile Cone=>new([new(CarvingCutterSectionKind.Linear,.05,3.175,10)],5);
    private static MillingLocalTravel Stock(VCarveCutterProfile? profile=null)
    {var s=new MillingLocalTravel(profile??Cone,10);s.BeginRegion(0);return s;}
    private static IReadOnlyList<MillingTransition> Connect(MillingLocalTravel s,Point3D from,Point3D to)
        =>MillingTransitionPlanner.Build(from,to,15,10,localTravel:s,
            postedFrom:new(ToolpathCommandRounding.Round((float)from.X),ToolpathCommandRounding.Round((float)from.Y),
                ToolpathCommandRounding.Round((float)from.Z-10)));

    [Fact]
    public void ActualDeeperConeSweepPermitsAMicrogapAtLocalHeight()
    {
        var s=Stock();s.RecordExecutedCut(new(-1,0,-5),new(1,0,-5));
        var moves=Connect(s,new(1,0,5),new(1.0004,0,5));
        Assert.Equal(3,moves.Count);Assert.Equal(6,moves[0].End.Z);
        Assert.True(moves[1].ClearedTravel);Assert.False(moves[2].ClearedTravel);
        Assert.Equal(new Point3D(1.0004f,0,5),moves[^1].End);
    }

    [Fact]
    public void CompletedEndpointPosesDoNotFillAnUncutHole()
    {
        var s=Stock();s.RecordExecutedCut(new(0,0,-5),new(1,0,-5));
        s.RecordExecutedCut(new(9,0,-5),new(10,0,-5));
        var moves=Connect(s,new(1,0,5),new(9,0,5));
        Assert.Contains(moves,m=>m.End.Z==15);Assert.DoesNotContain(moves,m=>m.ClearedTravel);
    }

    [Fact]
    public void FutureCutsAndAnotherRegionProvideNoTravelAuthority()
    {
        var s=Stock();s.RecordExecutedCut(new(0,0,-5),new(1,0,-5));
        Assert.Contains(Connect(s,new(1,0,5),new(4,0,5)),m=>m.End.Z==15);
        s.RecordExecutedCut(new(1,0,-5),new(4,0,-5));
        Assert.DoesNotContain(Connect(s,new(1,0,5),new(4,0,5)),m=>m.End.Z==15);
        s.BeginRegion(1);
        Assert.Contains(Connect(s,new(1,0,5),new(4,0,5)),m=>m.End.Z==15);
    }

    [Fact]
    public void FlatMillRequiresARealUnionOfExecutedLanes()
    {
        var s=Stock(new([new(CarvingCutterSectionKind.Linear,.5,.5,8)],5));
        s.RecordExecutedCut(new(-.4,-5,-3),new(-.4,5,-3));
        Assert.Contains(Connect(s,new(0,0,7),new(0,3,8)),m=>m.End.Z==15);
        s.RecordExecutedCut(new(.4,-5,-3),new(.4,5,-3));
        Assert.Contains(Connect(s,new(0,0,7),new(0,3,8)),m=>m.ClearedTravel);
        s.BeginRegion(1);
        Assert.DoesNotContain(Connect(s,new(0,0,7),new(0,3,8)),m=>m.ClearedTravel);
    }

    [Fact]
    public void BallNoseTipClearanceDoesNotAuthorizeAnUncutCylindricalFlank()
    {
        var s=Stock(new([new(CarvingCutterSectionKind.Hemisphere,0,1,1),new(CarvingCutterSectionKind.Linear,1,1,8)],5));
        s.RecordExecutedCut(new(-1,0,-5),new(1,0,-5));
        var local=Connect(s,new(1,0,5),new(1.001,0,5));
        Assert.DoesNotContain(local,m=>m.ClearedTravel&&m.Start.Z<=8);
    }

    [Fact]
    public void PostedRoundingRechecksTheWholeConnector()
    {
        var s=Stock();s.RecordExecutedCut(new(0,0,-5),new(0,0,-5));
        var moves=Connect(s,new(0,0,5),new(.14065,.1143,5));
        // The native first candidate fits a .18125 radial margin. F3 (.141,.114)
        // exceeds it, so the planner must choose a higher certified level.
        Assert.True(double.Hypot(.14065,.1143)<.18125);
        Assert.True(double.Hypot(.141,.114)>.18125);
        Assert.True(moves[0].End.Z>6);
    }

    [Fact]
    public void HeightOnlyTransfersKeepTheirOriginalControlledDescent()
    {
        var s=Stock();
        var moves=Connect(s,new(1.00049,0,7),new(1.00051,0,6));
        Assert.Single(moves);Assert.False(moves[0].ClearedTravel);Assert.Equal(6,moves[0].End.Z);
        Assert.Equal(moves[0].Start.X,moves[0].End.X);
    }

    [Fact]
    public void StockFromADifferentDatumCannotAuthorizeTravel()
    {
        var s=Stock();s.RecordExecutedCut(new(-1,0,-5),new(1,0,-5));
        var moves=MillingTransitionPlanner.Build(new(1,0,5),new(1.0004,0,5),15,12,
            localTravel:s,postedFrom:new(1,0,-5));
        Assert.Contains(moves,m=>m.End.Z==15);Assert.DoesNotContain(moves,m=>m.ClearedTravel);
    }

    [Fact]
    public void CancellationStopsProofAndRecording()
    {
        using var c=new CancellationTokenSource();var s=new MillingLocalTravel(Cone,10,c.Token);s.BeginRegion(0);c.Cancel();
        Assert.Throws<OperationCanceledException>(()=>s.RecordExecutedCut(new(0,0,-2),new(1,0,-2)));
        Assert.Throws<OperationCanceledException>(()=>Connect(s,new(0,0,8),new(1,0,8)));
    }
}
