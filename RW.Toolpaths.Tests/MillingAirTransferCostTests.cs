using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class MillingAirTransferCostTests
{
    private static VCarveCutterProfile Profile=>new([new(CarvingCutterSectionKind.Linear,0,3,10)],6);
    private static readonly PointD[] Board=[new(0,0),new(100,0),new(100,100),new(0,100)];
    private static MillingMotionSettings Settings(float surface=10)=>new(1000,500,5000,surface+5,surface){
        AirTransferContext=MillingAirTransferContext.Create(Profile,surface,Board)};
    private static TaggedToolpath Cut(double x,int region=0)=>new([new(x,20,7),new(x+1,20,7)],
        [new(0,1,ToolpathSpanKind.Cut)],region,"clearing",0,detectCurves:false);
    private static T Policy<T>(bool? enabled,Func<T> run){
        var before=ToolpathMotionCost.AirTransferEstimateOverride.Value;
        try{ToolpathMotionCost.AirTransferEstimateOverride.Value=enabled;return run();}
        finally{ToolpathMotionCost.AirTransferEstimateOverride.Value=before;}
    }

    [Fact]
    public void TheExplicitDisabledControlPricesAreExactlyLegacy()
    {
        var motion=Settings();var from=new Point3D(20,20,7);var to=new Point3D(35,20,7);
        double legacy=new ToolpathMotionCost(motion with{AirTransferContext=null},10).TransferSeconds(from,to);
        double actual=Policy(false,()=>new ToolpathMotionCost(motion,10){LocalTransferEstimatesEnabled=true}.TransferSeconds(from,to));
        Assert.Equal(legacy,actual);
    }

    [Fact]
    public void TheDefaultPricesExactlyMatchTheVerifiedAlignedPolicy()
    {
        var motion=Settings();var from=new Point3D(20,20,7);var to=new Point3D(35,20,7);
        (double Transfer,double Sequence) Price(bool? setting)=>Policy(setting,()=>{
            var cost=new ToolpathMotionCost(motion,10){LocalTransferEstimatesEnabled=true};
            return(cost.TransferSeconds(from,to),cost.SequenceSeconds(from,[Cut(20),Cut(25)],to));
        });
        var verified=Price(true);var normal=Price(null);var legacy=Price(false);
        Assert.Equal(verified,normal);Assert.True(normal.Transfer<legacy.Transfer);Assert.True(normal.Sequence<legacy.Sequence);
    }

    [Fact]
    public void AContextAndSelectedPolicyCannotEnableUnscopedCostOwners()
    {
        var motion=Settings();var from=new Point3D(20,20,7);var to=new Point3D(35,20,7);
        double legacy=new ToolpathMotionCost(motion with{AirTransferContext=null},10).TransferSeconds(from,to);
        foreach(bool? setting in new bool?[]{null,true,false}){
            double actual=Policy(setting,()=>new ToolpathMotionCost(motion,10).TransferSeconds(from,to));
            Assert.Equal(legacy,actual);
        }
    }

    [Fact]
    public void TheConnectionMasterCanKeepLegacyTransferPricing()
    {
        var previous=VCarvePathRouter.ConnectionOptimizationOverride.Value;
        try{VCarvePathRouter.ConnectionOptimizationOverride.Value=false;
            Policy(true,()=>{
                var cost=new ToolpathMotionCost(Settings(),10){LocalTransferEstimatesEnabled=true};
                Assert.Equal(cost.GlobalTransferSeconds(new(20,20,7),new(35,20,7)),cost.TransferSeconds(new(20,20,7),new(35,20,7)));return 0;
            });
        }finally{VCarvePathRouter.ConnectionOptimizationOverride.Value=previous;}
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(10f)]
    [InlineData(12.7f)]
    [InlineData(237.123f)]
    public void EligibleRoutingPriceMatchesActualEmitterAirPlaneConstruction(float surface)
    {
        var motion=Settings(surface);var from=new Point3D(20,20,surface-3);var to=new Point3D(35,20,surface-3);
        Policy(true,()=>{
            var cost=new ToolpathMotionCost(motion,surface){LocalTransferEstimatesEnabled=true};
            var local=new MillingLocalTravel(Profile,surface,default,null,Board);local.BeginRegion(0);
            local.RecordExecutedCut(new(80,80,-3),new(81,80,-3));
            var posted=new Point3D(ToolpathCommandRounding.Round((float)from.X),ToolpathCommandRounding.Round((float)from.Y),
                ToolpathCommandRounding.Round((float)from.Z-surface));
            var moves=MillingTransitionPlanner.Build(from,to,surface+5,surface,null,local,posted);
            Assert.Equal(moves.Sum(m=>cost.ChordSeconds(m.Start,m.End,m.ClearedTravel)),cost.TransferSeconds(from,to));
            Assert.True(cost.TransferSeconds(from,to)<cost.GlobalTransferSeconds(from,to));return 0;
        });
    }

    [Fact]
    public void ExtractedPureConstructionRetainsTheExactEstablishedMoves()
    {
        var context=Settings().AirTransferContext!;var from=new Point3D(20,20,7);var to=new Point3D(35,20,7);
        var construction=context.TryBuild(from,to,new(20,20,-3),15,null,out string refusal);
        Assert.Equal("",refusal);Assert.NotNull(construction);
        float plane=10.102f,approach=10.1f;
        Assert.Equal(new MillingTransition[]{new(from,new(20,20,plane)),
            new(new(20,20,plane),new(35,20,plane),true),new(new(35,20,plane),new(35,20,approach)),
            new(new(35,20,approach),to)},construction.Value.Moves);
        Assert.Equal(MillingTransitionPlanner.Build(from,to,15,10),construction.Value.GlobalMoves);
    }

    [Fact]
    public void OriginalFastApproachAndControlledResidualPriceRemainIdentical()
    {
        bool Guard(Point3D a,Point3D b)=>Math.Min(a.Z,b.Z)>=8;
        var motion=Settings() with{ClearedTravelGuard=Guard};
        var from=new Point3D(20,20,7);var to=new Point3D(35,20,7);
        var construction=motion.AirTransferContext!.TryBuild(from,to,new(20,20,-3),15,Guard,out _)!;
        var original=MillingTransitionPlanner.Build(from,to,15,10,Guard);
        Assert.Equal(original.Where(m=>m.Start.Z<=10.1f&&m.End.Z<m.Start.Z),
            construction.Value.Moves.Where(m=>m.Start.Z<=10.1f&&m.End.Z<m.Start.Z));
        Policy(true,()=>{
            var cost=new ToolpathMotionCost(motion,10){LocalTransferEstimatesEnabled=true};
            Assert.Equal(construction.Value.Moves.Sum(m=>cost.ChordSeconds(m.Start,m.End,m.ClearedTravel)),cost.TransferSeconds(from,to));return 0;
        });
    }

    [Fact]
    public void FullPhysicalProfileAndImmutableBoardSnapshotControlEligibility()
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,1,2),new(CarvingCutterSectionKind.Linear,1,3,6)],2);
        var board=Board.ToArray();var context=MillingAirTransferContext.Create(profile,10,board)!;
        board[0]=new(-100,-100); // The scalar parsed snapshot is independent.
        var settings=Settings() with{AirTransferContext=context};
        Policy(true,()=>{
            var cost=new ToolpathMotionCost(settings,10){LocalTransferEstimatesEnabled=true};
            Assert.Equal(cost.GlobalTransferSeconds(new(2,20,9),new(10,20,9)),cost.TransferSeconds(new(2,20,9),new(10,20,9)));
            Assert.True(cost.TransferSeconds(new(20,20,9),new(30,20,9))<cost.GlobalTransferSeconds(new(20,20,9),new(30,20,9)));return 0;
        });
        Assert.Null(MillingAirTransferContext.Create(profile,10,null));
        Assert.Null(MillingAirTransferContext.Create(profile,10,[new(0,0),new(100,0),new(70,70),new(0,100)]));
    }

    [Fact]
    public void MissingOrDifferentMaterialDatumRetainsGlobalPricing()
    {
        Policy(true,()=>{
            foreach(var settings in new[]{Settings() with{AirTransferContext=null},Settings() with{MaterialSurfaceZ=12}}){
                var cost=new ToolpathMotionCost(settings,10){LocalTransferEstimatesEnabled=true};
                Assert.Equal(cost.GlobalTransferSeconds(new(20,20,7),new(35,20,7)),cost.TransferSeconds(new(20,20,7),new(35,20,7)));
            }return 0;
        });
    }

    [Fact]
    public void GlobalEndpointsAndSequenceOuterBoundariesStayGlobal()
    {
        Policy(true,()=>{
            var cost=new ToolpathMotionCost(Settings(),10){LocalTransferEstimatesEnabled=true};
            var first=Cut(20);var next=Cut(25);var incoming=new Point3D(15,20,7);var following=new Point3D(30,20,7);
            double expected=cost.GlobalTransferSeconds(incoming,first.Points[0])+cost.VisitSeconds(first)
                +cost.TransferSeconds(first.Points[^1],next.Points[0])+cost.VisitSeconds(next)+cost.GlobalTransferSeconds(next.Points[^1],following);
            Assert.Equal(expected,cost.SequenceSeconds(incoming,[first,next],following));
            Assert.Equal(cost.GlobalTransferSeconds(new(20,20,15),new(35,20,7)),cost.TransferSeconds(new(20,20,15),new(35,20,7)));
            Assert.Equal(cost.GlobalTransferSeconds(new(20,20,7),new(35,20,15)),cost.TransferSeconds(new(20,20,7),new(35,20,15)));
            return 0;
        });
    }

    [Fact]
    public void CrossRegionAndAllFastPredecessorSequencesKeepLegacyHops()
    {
        Policy(true,()=>{
            var settings=Settings();var local=new ToolpathMotionCost(settings,10){LocalTransferEstimatesEnabled=true};
            var legacy=new ToolpathMotionCost(settings,10);
            Assert.Equal(legacy.SequenceSeconds(null,[Cut(20),Cut(25,region:1)],null),local.SequenceSeconds(null,[Cut(20),Cut(25,region:1)],null));
            var air=Cut(20) with{Spans=[new(0,1,ToolpathSpanKind.ClearedTravel)]};
            Assert.False(ToolpathMotionCost.HasNonfastChord(air));
            Assert.Equal(legacy.SequenceSeconds(null,[air,Cut(25)],null),local.SequenceSeconds(null,[air,Cut(25)],null));
            var zero=new TaggedToolpath([new(20,20,7),new(20+1e-13,20,7)],[],0,"clearing",0,detectCurves:false);
            Assert.False(ToolpathMotionCost.HasNonfastChord(zero));return 0;
        });
    }

    [Fact]
    public void CostNominationCannotAuthorizeADifferentActualPostedCursor()
    {
        Policy(true,()=>{
            var motion=Settings();var from=new Point3D(20,20,10.5);var to=new Point3D(35,20,10.5);
            var price=new ToolpathMotionCost(motion,10){LocalTransferEstimatesEnabled=true};
            Assert.True(price.TransferSeconds(from,to)<price.GlobalTransferSeconds(from,to));
            Assert.Null(motion.AirTransferContext!.TryBuild(from,to,new(20,20,-.2),15,null,out var refusal));
            Assert.Equal("retained-posted-height-below-air-plane",refusal);
            var local=new MillingLocalTravel(Profile,10,default,null,Board);local.BeginRegion(0);local.RecordExecutedCut(new(80,80,-3),new(81,80,-3));
            var actual=MillingTransitionPlanner.Build(from,to,15,10,null,local,new(20,20,-.2));
            Assert.Contains(actual,m=>m.End.Z==15);return 0;
        });
    }

    [Fact]
    public void CorrectAirPlanePriceCanRejectAnOtherwiseAcceptedLongReturn()
    {
        var from=new Point3D(20,20,7);var target=new Point3D(20.5,20,7);var end=new Point3D(21.5,20,7);
        var independent=new TaggedToolpath([target,end],[new(0,1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);
        var detour=new TaggedToolpath([from,new(20,23.9,7),target,end],
            [new(0,2,ToolpathSpanKind.Link),new(2,3,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);
        var legacy=new ToolpathMotionCost(Settings(),10);
        Assert.True(ToolpathMotionCost.PreferContinuation(legacy,from,independent,detour,out _,out _));
        Policy(true,()=>{
            var local=new ToolpathMotionCost(Settings(),10){LocalTransferEstimatesEnabled=true};
            Assert.False(ToolpathMotionCost.PreferContinuation(local,from,independent,detour,out double before,out double after));
            Assert.True(before<after);Assert.Equal(legacy.VisitSeconds(detour),local.VisitSeconds(detour));return 0;
        });
    }

    [Fact]
    public void CostDiagnosticsSeparateSampledPricesFromActualSavings()
    {
        Policy(true,()=>{
            var log=new List<string>();var cost=new ToolpathMotionCost(Settings(),10){LocalTransferEstimatesEnabled=true};
            cost.TransferSeconds(new(20,20,7),new(35,20,7));cost.Report(log.Add,"test");
            Assert.Contains(log,l=>l.Contains("enabled=True ")&&l.Contains("localAccepted=1 ")&&l.Contains("sampledDifferenceIsNotRouteSaving=True")
                &&l.Contains("actualModalAuthority=False")&&l.Contains("finalEmitterProofRequired=True"));return 0;
        });
    }
}
