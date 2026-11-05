using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class MillingAboveStockTransferTests
{
    private static VCarveCutterProfile Cone=>new([new(CarvingCutterSectionKind.Linear,0,3,10)],6);
    private static readonly PointD[] Boundary=[new(0,0),new(100,0),new(100,100),new(0,100)];

    private static MillingLocalTravel Stock(float surface=10,IReadOnlyList<PointD>? boundary=null,
        bool record=true,VCarveCutterProfile? profile=null,Action<string>? log=null)
    {
        var s=new MillingLocalTravel(profile??Cone,surface,default,log,boundary??Boundary);s.BeginRegion(0);
        // A real earlier cut establishes an ongoing visit, but deliberately
        // clears nothing beneath the requested transfer or approach.
        if(record)s.RecordExecutedCut(new(80,80,-3),new(81,80,-3));
        return s;
    }
    private static IReadOnlyList<MillingTransition> Build(MillingLocalTravel s,Point3D from,Point3D to,
        float surface=10,Func<Point3D,Point3D,bool>? guard=null,Point3D? posted=null)
        =>MillingTransitionPlanner.Build(from,to,surface+5,surface,guard,s,posted??new(
            ToolpathCommandRounding.Round((float)from.X),ToolpathCommandRounding.Round((float)from.Y),
            ToolpathCommandRounding.Round((float)from.Z-surface)));
    private static T WithPolicy<T>(bool? enabled,Func<T> action)
    {
        var previous=MillingLocalTravel.AirPlaneOptimizationOverride.Value;
        MillingLocalTravel.AirPlaneOptimizationOverride.Value=enabled;
        try{return action();}finally{MillingLocalTravel.AirPlaneOptimizationOverride.Value=previous;}
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(10f)]
    [InlineData(12.7f)]
    [InlineData(237.123f)]
    public void EntireProfileCanTraverseAboveOriginalStockWithoutACutChannel(float surface)
    {
        var from=new Point3D(20,20,(float)(surface-3));var to=new Point3D(35,20,(float)(surface-3));
        var moves=WithPolicy(true,()=>Build(Stock(surface),from,to,surface));
        var traverse=Assert.Single(moves.Where(m=>m.Start.X!=m.End.X||m.Start.Y!=m.End.Y));
        Assert.True(traverse.ClearedTravel);Assert.Equal(traverse.Start.Z,traverse.End.Z);
        Assert.InRange(traverse.Start.Z-surface,.101,.103);
        Assert.True(ToolpathCommandRounding.Round((float)traverse.Start.Z-surface)>=.1);
        Assert.DoesNotContain(moves,m=>m.Start.Z==surface+5||m.End.Z==surface+5);
        Assert.Equal(from,moves[0].Start);Assert.Equal(to,moves[^1].End);
        Assert.False(moves[^1].ClearedTravel);
    }

    [Fact]
    public void TheDisabledPolicyKeepsOriginalSafeHeight()
    {
        var moves=WithPolicy(false,()=>Build(Stock(),new(20,20,7),new(35,20,7)));
        Assert.Contains(moves,m=>m.End.Z==15);Assert.DoesNotContain(moves,m=>m.ClearedTravel);
    }

    [Fact]
    public void TheDefaultPolicyMatchesTheVerifiedAirPlanePolicy()
    {
        var expected=WithPolicy(true,()=>Build(Stock(),new(20,20,7),new(35,20,7)));
        var normal=WithPolicy(null,()=>Build(Stock(),new(20,20,7),new(35,20,7)));
        Assert.Equal(expected,normal);
    }

    [Fact]
    public void TheMasterConnectionControlRetainsTheOriginalSafeTransfer()
    {
        var previous=VCarvePathRouter.ConnectionOptimizationOverride.Value;
        try{
            VCarvePathRouter.ConnectionOptimizationOverride.Value=false;
            var moves=WithPolicy(null,()=>Build(Stock(),new(20,20,7),new(35,20,7)));
            Assert.Contains(moves,m=>m.End.Z==15);Assert.DoesNotContain(moves,m=>m.ClearedTravel);
        }finally{VCarvePathRouter.ConnectionOptimizationOverride.Value=previous;}
    }

    [Fact]
    public void MissingAndUnsupportedBoundariesCannotAuthorizeTheAirPlane()
    {
        var missing=new MillingLocalTravel(Cone,10);missing.BeginRegion(0);missing.RecordExecutedCut(new(80,80,-3),new(81,80,-3));
        var concave=new PointD[]{new(0,0),new(100,0),new(70,70),new(0,100)};
        foreach(var s in new[]{missing,Stock(boundary:concave)}){
            var moves=WithPolicy(true,()=>Build(s,new(20,20,7),new(35,20,7)));
            Assert.Contains(moves,m=>m.End.Z==15);Assert.DoesNotContain(moves,m=>m.ClearedTravel);
        }
    }

    [Fact]
    public void EntirePhysicalProfileAndActualRoundedCursorMustFitTheStockRectangle()
    {
        // The configured cutting depth sees radius1; the complete physical
        // profile still includes its radius3 shank.
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,1,2),
            new(CarvingCutterSectionKind.Linear,1,3,6)],2);
        var nearEdge=WithPolicy(true,()=>Build(Stock(profile:profile),new(2,20,9),new(10,20,9)));
        Assert.Contains(nearEdge,m=>m.End.Z==15);
        var roundedOutside=WithPolicy(true,()=>Build(Stock(),new(3.1002,20,7),new(20,20,7),
            posted:new(3.099,20,-3)));
        Assert.Contains(roundedOutside,m=>m.End.Z==15);
    }

    [Fact]
    public void OriginalBelowPlaneFastApproachAndControlledRemainderAreRetainedExactly()
    {
        var from=new Point3D(20,20,7);var to=new Point3D(35,20,7);
        bool EarlierStock(Point3D a,Point3D b)=>Math.Min(a.Z,b.Z)>=8;
        var before=MillingTransitionPlanner.Build(from,to,15,10,EarlierStock);
        var after=WithPolicy(true,()=>Build(Stock(),from,to,guard:EarlierStock));
        var oldBelow=before.Where(m=>m.Start.Z<=10.1f&&m.End.Z<m.Start.Z).ToArray();
        var newBelow=after.Where(m=>m.Start.Z<=10.1f&&m.End.Z<m.Start.Z).ToArray();
        Assert.Equal(oldBelow,newBelow);Assert.Contains(newBelow,m=>m.ClearedTravel);
        Assert.False(newBelow[^1].ClearedTravel);Assert.Equal(to,newBelow[^1].End);
    }

    [Fact]
    public void AProvedLowerExecutedPrefixRemainsPreferred()
    {
        var s=Stock();s.RecordExecutedCut(new(19,20,-6),new(36,20,-6));
        var moves=WithPolicy(true,()=>Build(s,new(20,20,7),new(35,20,7)));
        var traverse=Assert.Single(moves.Where(m=>m.Start.X!=m.End.X));
        Assert.True(traverse.ClearedTravel);Assert.True(traverse.End.Z<10);
    }

    [Fact]
    public void NewRegionsAndDifferentStockDatumsRetainGlobalSafeHeight()
    {
        var s=Stock();s.BeginRegion(1);
        var nextRegion=WithPolicy(true,()=>Build(s,new(20,20,7),new(35,20,7)));
        Assert.Contains(nextRegion,m=>m.End.Z==15);
        var differentDatum=WithPolicy(true,()=>Build(Stock(),new(20,20,9),new(35,20,9),surface:12));
        Assert.Contains(differentDatum,m=>m.End.Z==17);
    }

    [Fact]
    public void OriginalEntryAndFinalRetractionWithoutLocalContextAreUnchanged()
    {
        var from=new Point3D(20,20,15);var to=new Point3D(35,20,7);
        var expected=MillingTransitionPlanner.Build(from,to,15,10);
        var actual=WithPolicy(true,()=>MillingTransitionPlanner.Build(from,to,15,10));
        Assert.Equal(expected,actual);
        var noEntryContext=WithPolicy(true,()=>Build(Stock(record:false),new(20,20,7),to));
        Assert.Contains(noEntryContext,m=>m.End.Z==15);
    }

    [Fact]
    public void RetainedActualCursorHeightCannotBeReplacedWithNominalAirHeight()
    {
        var from=new Point3D(20,20,10.5);var to=new Point3D(35,20,10.5);
        var moves=WithPolicy(true,()=>Build(Stock(),from,to,posted:new(20,20,-.2)));
        Assert.Contains(moves,m=>m.End.Z==15);Assert.DoesNotContain(moves,m=>m.ClearedTravel);
    }

    [Fact]
    public void AcceptedAndRefusedAirPlanesExplainTheirAuthority()
    {
        var logs=new List<string>();var s=Stock(log:logs.Add);
        WithPolicy(true,()=>Build(s,new(20,20,7),new(35,20,7)));s.Report();
        Assert.Contains(logs,l=>l.Contains("authority=known-rectangular-stock-top")&&l.Contains("nominalAndActualF3=True"));
        Assert.Contains(logs,l=>l.Contains("airPlaneAccepted=1")&&l.Contains("sourceCutsPreserved=True"));
    }
}
