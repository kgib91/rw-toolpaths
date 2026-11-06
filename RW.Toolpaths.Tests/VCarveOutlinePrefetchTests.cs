using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[CollectionDefinition("Outline prefetch lifetime",DisableParallelization=true)]
public sealed class OutlinePrefetchCollection{}
[Collection("Outline prefetch lifetime")]
public sealed class VCarveOutlinePrefetchTests
{
    static readonly VCarveOptions Options=new(){SmoothVCarving=true,SmoothRampAngleDegrees=45,
        StepOver=.4,IncludeInteriorFill=false,EdgeFillWidth=2,ExcludeDiscSweep=true,DiscRadius=1,DetectCurves=false};
    static VCarveCutterProfile Profile()=>new([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
        new(CarvingCutterSectionKind.Linear,1,3,9)],10);
    static PointD[] Rectangle(double left,double bottom,double right,double top)
        =>[new(left,bottom),new(right,bottom),new(right,top),new(left,top)];
    static VCarvePreparation Prepare()=>VCarveToolpaths.Prepare([new(Rectangle(0,0,30,30),[Rectangle(10,10,15,15)])],
        new ToolGeometry(3){VCarveProfile=Profile()},new(2,2),Options,BoostVoronoiProvider.CreateDefault());
    static VCarveRampSurface.InitialOutline Request(VCarvePreparation owner,VCarveOptions? options=null,
        IReadOnlyList<IReadOnlyList<PointD>>? boundary=null)
        =>VCarveRampSurface.PrepareInitialOutline(boundary??owner.Regions[0].Boundary,options??Options,Profile(),2,default);
    static void Equal(IReadOnlyList<OffsetRing> expected,IReadOnlyList<OffsetRing> actual)
    {
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<expected.Count;i++)
        {
            var a=expected[i];var b=actual[i];Assert.Equal(a.Points,b.Points);
            Assert.Equal(a.Area,b.Area);Assert.Equal(a.Perimeter,b.Perimeter);Assert.Equal(a.Centroid,b.Centroid);
            Assert.Equal(a.Level,b.Level);Assert.Equal(a.RegionIndex,b.RegionIndex);Assert.Equal(a.IsClosed,b.IsClosed);
            Assert.Equal(a.IsHole,b.IsHole);Assert.Equal(a.OffsetDistance,b.OffsetDistance);
            Assert.Equal(a.Parent,b.Parent);Assert.Equal(a.Children,b.Children);
        }
    }

    [Fact]
    public void PrefetchMatchesLiteralGraphAndTransfersMutableOwnershipExactlyOnce()
    {
        var owner=Prepare();Assert.Null(owner.PrefetchInitialOutline(default));
        var request=Request(owner);var expected=Literal(request.Boundary,Options);
        using var lease=owner.PrefetchInitialOutline(default,0);Assert.NotNull(lease);
        var log=new List<string>();
        var result=owner.InitialOutlineRings(request,default,log.Add);
        Equal(expected,result);Assert.Equal(1,owner.InitialOutlineConstructions);
        Assert.Contains(log,l=>l.Contains("event=hit")&&l.Contains("consumedOnce=True"));
        Assert.Equal(0,owner.InitialOutlineCacheState.Entries);
        result[0].Points[0]=new(999,999);result[0].Children.Clear();result.Clear();
        Equal(expected,owner.InitialOutlineRings(request,default,null));
        Assert.Equal(2,owner.InitialOutlineConstructions);
    }

    [Fact]
    public async Task DifferentOrderedBoundaryOrSpacingCannotConsumeThePrefetchedBatch()
    {
        var owner=Prepare();var original=Request(owner);
        using var lease=owner.PrefetchInitialOutline(default,0);Assert.NotNull(lease);
        await lease!.Completion;Assert.Equal(1,owner.InitialOutlineCacheState.Entries);
        var changed=Request(owner,Options with{StepOver=.2});var log=new List<string>();
        Equal(Literal(changed.Boundary,Options with{StepOver=.2}),owner.InitialOutlineRings(changed,default,log.Add));
        Assert.Contains(log,l=>l.Contains("event=miss"));Assert.Equal(1,owner.InitialOutlineCacheState.Entries);
        var reversed=Request(owner,boundary:original.Boundary.Select(r=>(IReadOnlyList<PointD>)r.Reverse().ToArray()).ToArray());
        owner.InitialOutlineRings(reversed,default,null);Assert.Equal(1,owner.InitialOutlineCacheState.Entries);
        Equal(Literal(original.Boundary,Options),owner.InitialOutlineRings(original,default,null));
        Assert.Equal(0,owner.InitialOutlineCacheState.Entries);
    }

    [Fact]
    public void IndependentScopeConsumesTheSameExactBatchAndInputSnapshotsRemainStable()
    {
        var owner=Prepare();var boundary=owner.Regions[0].Boundary.Select(r=>(IReadOnlyList<PointD>)r.ToArray()).ToArray();
        var request=Request(owner,boundary:boundary);var expected=Literal(request.Boundary,Options);
        ((PointD[])boundary[0])[0]=new(-999,-999);
        using var lease=owner.PrefetchInitialOutline(default,0);Assert.NotNull(lease);
        var scope=owner.CreateIndependentFinishScope();
        Equal(expected,scope.InitialOutlineRings(request,default,null));
        Assert.Equal(1,owner.InitialOutlineConstructions);Assert.InRange(scope.InitialOutlineCacheState.Bytes,0,64L<<20);
    }

    [Fact]
    public void EmptyAndOverBudgetOutlinesKeepTheOriginalDiagnosticsAndFailure()
    {
        var tiny=new[]{(IReadOnlyList<PointD>)Rectangle(0,0,.001,.001)};var log=new List<string>();
        Assert.Empty(VCarveRampSurface.Contours(tiny,Options,Profile(),2,0,default,log.Add));
        Assert.Contains(log,l=>l.Contains("stage=outline-ramps event=empty"));
        var excessive=Options with{SmoothRampAngleDegrees=1e-8,StepOver=1e-10,ExcludeDiscSweep=false};
        log.Clear();
        var error=Assert.Throws<InvalidOperationException>(()=>VCarveRampSurface.Contours(
            [Rectangle(0,0,30,30)],excessive,Profile(),2,0,default,log.Add));
        Assert.Equal("Full-outline V-carve ramp exceeds the concentric ring inventory.",error.Message);
        Assert.Contains(log,l=>l.Contains("event=failed reason=ring-budget"));
    }

    [Fact]
    public void DisposalJoinsOptionalWorkAndKeepsCallerCancellationObservable()
    {
        var previousControl=VCarvePreparation.InitialOutlinePrefetchAmbientWorkers.Value;
        try
        {
            VCarvePreparation.InitialOutlinePrefetchAmbientWorkers.Value=false;
            var owner=Prepare();using var canceled=new CancellationTokenSource();canceled.Cancel();
            Assert.ThrowsAny<OperationCanceledException>(()=>owner.PrefetchInitialOutline(canceled.Token,0));
            Assert.Equal(0,owner.InitialOutlineCacheState.Publications);
            using var caller=new CancellationTokenSource();
            var lease=owner.PrefetchInitialOutline(caller.Token,0);Assert.NotNull(lease);caller.Cancel();
            Assert.ThrowsAny<OperationCanceledException>(()=>lease!.Dispose());Assert.True(lease!.Completion.IsCompleted);
            Assert.ThrowsAny<OperationCanceledException>(()=>owner.InitialOutlineRings(Request(owner),caller.Token,null));
            var budget=Assert.IsType<VCarvePreparation.OutlinePrefetchWorkerBudget>(owner.InitialOutlinePrefetchBudget);
            Assert.Equal(1,budget.EffectiveGeometryWorkers);
            Assert.Equal(budget.AmbientGeometryWorkers,budget.RestoredGeometryWorkers);
            Assert.Equal(budget.AmbientStockFitWorkers,budget.RestoredStockFitWorkers);
        }
        finally{VCarvePreparation.InitialOutlinePrefetchAmbientWorkers.Value=previousControl;}
    }

    [Theory]
    [InlineData(null)]
    [InlineData(true)]
    public async Task ExplicitSerialBuildersRetainAmbientExactKeysAndLiteralGraphAndRestGeometry(bool? ambientControl)
    {
        var limit=(AsyncLocal<int>)typeof(GeometryParallelism).GetField("Limit",
            System.Reflection.BindingFlags.Static|System.Reflection.BindingFlags.NonPublic)!.GetValue(null)!;
        int previousLimit=limit.Value;
        var previousControl=VCarvePreparation.InitialOutlinePrefetchAmbientWorkers.Value;
        try
        {
            limit.Value=4;
            VCarvePreparation.InitialOutlinePrefetchAmbientWorkers.Value=ambientControl;
            var original=Prepare();var originalRequest=Request(original);
            using var originalLease=original.PrefetchInitialOutline(default,0);
            Assert.NotNull(originalLease);await originalLease!.Completion;
            var originalBudget=Assert.IsType<VCarvePreparation.OutlinePrefetchWorkerBudget>(original.InitialOutlinePrefetchBudget);
            Assert.Equal(4,originalBudget.EffectiveGeometryWorkers);Assert.Equal(4,originalBudget.EffectiveStockFitWorkers);
            var originalRings=original.InitialOutlineRings(originalRequest,default,null,out var originalRest);
            Assert.NotNull(originalRest);

            VCarvePreparation.InitialOutlinePrefetchAmbientWorkers.Value=false;
            var scoped=Prepare();var scopedRequest=Request(scoped);
            using var scopedLease=scoped.PrefetchInitialOutline(default,0);
            Assert.NotNull(scopedLease);await scopedLease!.Completion;
            var scopedBudget=Assert.IsType<VCarvePreparation.OutlinePrefetchWorkerBudget>(scoped.InitialOutlinePrefetchBudget);
            Assert.Equal("rest-fill",scopedBudget.Phase);
            Assert.Equal(4,scopedBudget.AmbientGeometryWorkers);Assert.Equal(4,scopedBudget.AmbientStockFitWorkers);
            Assert.Equal(1,scopedBudget.EffectiveGeometryWorkers);Assert.Equal(1,scopedBudget.EffectiveStockFitWorkers);
            Assert.Equal(4,scopedBudget.RestoredGeometryWorkers);Assert.Equal(4,scopedBudget.RestoredStockFitWorkers);
            Assert.Equal(4,GeometryParallelism.WorkerLimit);Assert.Equal(4,GeometryParallelism.StockFitWorkerLimit);
            Assert.Equal(1,scoped.InitialOutlineCacheState.Publications);
            var log=new List<string>();
            var scopedRings=scoped.InitialOutlineRings(scopedRequest,default,log.Add,out var scopedRest);
            Equal(originalRings,scopedRings);Assert.NotNull(scopedRest);
            Assert.Contains(log,s=>s.Contains("event=hit")&&s.Contains("consumedOnce=True"));
            VCarveRampSurface.RestFillRequest Rest(VCarveRampSurface.InitialOutline request)
                =>VCarveRampSurface.CaptureRestFillRequest(request.Boundary,Options,2,request.Band.Grade,default);
            var a=originalRest!.Take(Rest(originalRequest),default,null);
            var b=scopedRest!.Take(Rest(scopedRequest),default,null);
            Assert.NotNull(a);Assert.NotNull(b);
            Assert.Equal(a!.MaterialFloor.Count,b!.MaterialFloor.Count);
            for(int i=0;i<a.MaterialFloor.Count;i++)Assert.Equal(a.MaterialFloor[i],b.MaterialFloor[i]);
            Assert.Equal(a.Disc.SweptRegion.Count,b.Disc.SweptRegion.Count);
            for(int i=0;i<a.Disc.SweptRegion.Count;i++)Assert.Equal(a.Disc.SweptRegion[i],b.Disc.SweptRegion[i]);
            Assert.Equal(0,scoped.InitialOutlineCacheState.Entries);
        }
        finally
        {
            VCarvePreparation.InitialOutlinePrefetchAmbientWorkers.Value=previousControl;
            limit.Value=previousLimit;
        }
    }

    // Retained pre-prefetch construction schedule and collector invocation.
    static List<OffsetRing> Literal(IReadOnlyList<IReadOnlyList<PointD>> boundary,VCarveOptions options)
    {
        var band=ConcentricBandSchedule.Ramp(Profile(),2,options);
        var regions=PolygonOps.DifferenceToComponents(boundary,Array.Empty<IReadOnlyList<PointD>>())
            .Select(c=>new MillingRegion(c.Outer,c.Holes)).ToArray();
        double geometric=regions.Select(r=>Math.Min(r.Outer.Max(p=>p.x)-r.Outer.Min(p=>p.x),
            r.Outer.Max(p=>p.y)-r.Outer.Min(p=>p.y))/2).DefaultIfEmpty(0).Max();
        double limit=options.IncludeInteriorFill?double.PositiveInfinity:options.EdgeFillWidth;
        bool rest=options.ExcludeDiscSweep&&(options.IncludeInteriorFill||limit>0);
        double reach=Math.Min(geometric,rest?band.Reach:options.IncludeInteriorFill?geometric:band.Reach+limit);
        if(reach<band.First)return [];
        double intervals=Math.Floor((reach-band.First)/band.Pitch+1e-10);
        return MedialAxisGeometry.BuildConcentricFill(regions,new RingOffsetOptions(band.First,band.Pitch)
        {MaximumInset=intervals*band.Pitch,RingLimit=(int)intervals+1,MinimumRingArea=0,
            Tolerances=GeometryTolerances.Default with{ArcTolerance=.001,SimplifyTolerance=0}},0,false);
    }
}
