using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class MillingAirFirstTravelTests
{
    private static VCarveCutterProfile Cone=>new([new(CarvingCutterSectionKind.Linear,0,3,10)],6);
    private static readonly PointD[] Board=[new(0,0),new(100,0),new(100,100),new(0,100)];
    private static MillingLocalTravel Owner(bool prefer,float surface=10,List<string>? log=null,
        IReadOnlyList<PointD>? board=null,VCarveCutterProfile? profile=null,bool record=true)
    {
        var owner=new MillingLocalTravel(profile??Cone,surface,default,log is null?null:log.Add,board??Board)
            {PreferAboveStockTransfers=prefer};
        owner.BeginRegion(0);
        if(record)owner.RecordExecutedCut(new(40,20,-3),new(15,20,-3));
        return owner;
    }
    private static Point3D Posted(Point3D p,float surface)=>new(ToolpathCommandRounding.Round((float)p.X),
        ToolpathCommandRounding.Round((float)p.Y),ToolpathCommandRounding.Round((float)p.Z-surface));
    private static IReadOnlyList<MillingTransition> Connect(MillingLocalTravel owner,float surface=10,
        Func<Point3D,Point3D,bool>? guard=null,Point3D? from=null,Point3D? to=null,Point3D? posted=null)
    {
        var a=from??new(20,20,surface-3);var b=to??new(35,20,surface-3);
        return MillingTransitionPlanner.Build(a,b,surface+5,surface,guard,owner,posted??Posted(a,surface));
    }

    [Fact]
    public void ExplicitAirPreferenceSkipsPrefixProofEvenWhenItsLowerPlaneCouldBeCertified()
    {
        var control=Connect(Owner(false));var log=new List<string>();var owner=Owner(true,log:log);
        var actual=Connect(owner);owner.Report();
        Assert.True(control.Single(m=>m.ClearedTravel).Start.Z<10);
        var air=Assert.Single(actual.Where(m=>m.Start.X!=m.End.X));
        Assert.True(air.ClearedTravel);Assert.InRange(air.Start.Z,10.101,10.103);Assert.Equal(air.Start.Z,air.End.Z);
        Assert.Equal(control[^1].End,actual[^1].End);
        Assert.Contains(log,l=>l.Contains("event=air-first-accepted")&&l.Contains("prefixGeometryQueried=False"));
        string summary=Assert.Single(log.Where(l=>l.Contains("event=summary")));
        Assert.Contains("attempted=1 accepted=1",summary);Assert.Contains("retainedSafePlane=0",summary);
        Assert.Contains("stockQueries=0",summary);Assert.Contains("floorUnionBuilds=0",summary);
        Assert.Contains("airFirstAccepted=1",summary);Assert.Contains("prefixGeometryQueriesSkipped=1",summary);
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(12.7f)]
    [InlineData(237.123f)]
    public void OriginalNativeAndPostedTopReservesAreStillRequiredInTheActualFrame(float surface)
    {
        var owner=Owner(true,surface);var actual=Connect(owner,surface);
        var air=Assert.Single(actual.Where(m=>m.Start.X!=m.End.X));
        Assert.True(air.ClearedTravel);
        Assert.True((float)air.Start.Z-surface>=MillingLocalTravel.Clearance+.001);
        Assert.True(ToolpathCommandRounding.Round((float)air.Start.Z-surface)>=MillingLocalTravel.Clearance);
        Assert.Equal(new Point3D(35,20,surface-3),actual[^1].End);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NullOrExistingImmutableApproachCallbackRetainsTheExactBelowPlaneModes(bool withGuard)
    {
        var oldQueries=new List<(Point3D,Point3D)>();var actualQueries=new List<(Point3D,Point3D)>();
        bool Old(Point3D a,Point3D b){oldQueries.Add((a,b));return b.Z>=9;}
        bool Actual(Point3D a,Point3D b){actualQueries.Add((a,b));return b.Z>=9;}
        var from=new Point3D(20,20,7);var to=new Point3D(35,20,7);
        var original=MillingTransitionPlanner.Build(from,to,15,10,withGuard?Old:null);
        var next=Connect(Owner(true),guard:withGuard?Actual:null);
        Assert.Equal(oldQueries,actualQueries);
        Assert.Equal(original.Where(m=>m.Start.Z<=10.1f&&m.End.Z<m.Start.Z),
            next.Where(m=>m.Start.Z<=10.1f&&m.End.Z<m.Start.Z));
        Assert.False(next[^1].ClearedTravel);
        if(withGuard)Assert.Contains(next,m=>m.ClearedTravel&&m.End.Z<10);
        else Assert.DoesNotContain(next,m=>m.ClearedTravel&&m.End.Z<10);
    }

    [Fact]
    public void FullFlankEnvelopeAndRoundedDestinationCanRefuseThePreferredAirPlane()
    {
        // Tip and active shallow section fit, but the actual shoulder does not.
        var flank=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2),
            new(CarvingCutterSectionKind.Linear,.1,3,6)],2);
        var from=new Point3D(2,20,8);var to=new Point3D(2.0004,20,8);
        var log=new List<string>();var preferred=Owner(true,log:log,profile:flank);
        var actual=Connect(preferred,from:from,to:to);preferred.Report();
        Assert.Equal(Connect(Owner(false,profile:flank),from:from,to:to),actual);
        Assert.DoesNotContain(log,l=>l.Contains("event=air-first-accepted"));
        Assert.Contains(log,l=>l.Contains("event=air-first-refused"));
        // The nominal destination has positive shoulder margin; its actual F3
        // X coordinate rounds outside that same complete-profile margin.
        var fractional=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3.00025,10)],6);
        from=new(20,20,7);to=new(3.10049,20,7);log.Clear();
        preferred=Owner(true,log:log,profile:fractional);
        Connect(preferred,from:from,to:to);preferred.Report();
        Assert.True((float)to.X>3.10025);Assert.True(ToolpathCommandRounding.Round((float)to.X)<3.10025);
        Assert.DoesNotContain(log,l=>l.Contains("event=air-first-accepted"));
        Assert.Contains(log,l=>l.Contains("full-profile-envelope-outside-stock-boundary"));
    }

    [Fact]
    public void MissingBoundaryRegionPrefixOrMatchingSurfaceRetainsTheOriginalFallback()
    {
        foreach(string refusal in new[]{"missing","nonrectangle","no-prefix","new-region","different-surface"})
        {
            MillingLocalTravel Make(bool prefer)
            {
                var board=refusal=="missing"?null:refusal=="nonrectangle"?
                    new PointD[]{new(0,0),new(100,0),new(70,70),new(0,100)}:Board;
                var owner=new MillingLocalTravel(Cone,refusal=="different-surface"?12:10,default,null,board)
                    {PreferAboveStockTransfers=prefer};owner.BeginRegion(0);
                if(refusal!="no-prefix")owner.RecordExecutedCut(new(40,20,-3),new(15,20,-3));
                if(refusal=="new-region")owner.BeginRegion(1);
                return owner;
            }
            Assert.Equal(Connect(Make(false)),Connect(Make(true)));
        }
    }

    [Fact]
    public void CancellationInsideTheExistingApproachCallbackPreventsPreferredPublication()
    {
        using var cancellation=new CancellationTokenSource();
        var owner=new MillingLocalTravel(Cone,10,cancellation.Token,null,Board){PreferAboveStockTransfers=true};
        owner.BeginRegion(0);owner.RecordExecutedCut(new(40,20,-3),new(15,20,-3));
        Assert.ThrowsAny<OperationCanceledException>(()=>Connect(owner,guard:(_,_)=>{cancellation.Cancel();return false;}));
    }
}
