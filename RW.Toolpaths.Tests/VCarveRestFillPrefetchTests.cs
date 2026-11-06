using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[Collection("Outline prefetch lifetime")]
public sealed class VCarveRestFillPrefetchTests
{
    static readonly VCarveOptions Options=new(){SmoothVCarving=true,SmoothRampAngleDegrees=45,
        StepOver=.4,IncludeInteriorFill=false,EdgeFillWidth=2,ExcludeDiscSweep=true,DiscRadius=1,DetectCurves=false};
    static VCarveCutterProfile Profile()=>new([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
        new(CarvingCutterSectionKind.Linear,1,3,9)],10);
    static PointD[] Rectangle(double a,double b,double c,double d)=>[new(a,b),new(c,b),new(c,d),new(a,d)];
    static VCarvePreparation Prepare()=>VCarveToolpaths.Prepare([new(Rectangle(0,0,30,30),[Rectangle(10,10,15,15)])],
        new ToolGeometry(3){VCarveProfile=Profile()},new(2,2),Options,BoostVoronoiProvider.CreateDefault());
    static VCarveRampSurface.InitialOutline Outline(VCarvePreparation owner)
        =>VCarveRampSurface.PrepareInitialOutline(owner.Regions[0].Boundary,Options,Profile(),2,default);
    static VCarveRampSurface.RestFillRequest Request(IReadOnlyList<IReadOnlyList<PointD>> boundary)
        =>VCarveRampSurface.CaptureRestFillRequest(boundary,Options,2,
            Math.Tan(Options.SmoothRampAngleDegrees*Math.PI/180),default);
    static void EqualRings(IReadOnlyList<IReadOnlyList<PointD>> expected,IReadOnlyList<IReadOnlyList<PointD>> actual)
    {Assert.Equal(expected.Count,actual.Count);for(int i=0;i<expected.Count;i++)Assert.Equal(expected[i],actual[i]);}

    [Fact]
    public async Task PrefetchedRestDomainAndFinalContoursMatchTheLiteralOriginalConstruction()
    {
        var owner=Prepare();var boundary=owner.Regions[0].Boundary;
        var expected=VCarveRampSurface.Contours(boundary,Options,Profile(),2,0,default,null);
        using var lease=owner.PrefetchInitialOutline(default,0);Assert.NotNull(lease);await lease!.Completion;
        var log=new List<string>();
        var actual=VCarveRampSurface.Contours(boundary,Options with{Preparation=owner},Profile(),2,0,default,log.Add);
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<expected.Count;i++)
        {
            Assert.Equal(expected[i].Points,actual[i].Points);Assert.Equal(expected[i].Spans,actual[i].Spans);
            Assert.Equal(expected[i].RegionIndex,actual[i].RegionIndex);Assert.Equal(expected[i].Category,actual[i].Category);
            Assert.Equal(expected[i].DepthPassIndex,actual[i].DepthPassIndex);Assert.Equal(expected[i].VisitIdentity,actual[i].VisitIdentity);
            Assert.Equal(expected[i].ConcentricSource!.Ring,actual[i].ConcentricSource!.Ring);
            Assert.Equal(expected[i].ConcentricSource!.Plan.Parents,actual[i].ConcentricSource!.Plan.Parents);
        }
        Assert.Contains(log,l=>l.Contains("stage=outline-prefetch event=hit"));
        Assert.Contains(log,l=>l.Contains("stage=rest-fill-prefetch event=hit")&&l.Contains("stockRead=False"));
        Assert.Equal(0,owner.InitialOutlineCacheState.Entries);
    }

    [Fact]
    public async Task HelperGeometryTransfersOnceAndCannotBeMutatedThroughAnotherConsumer()
    {
        var owner=Prepare();var outline=Outline(owner);var request=Request(outline.Boundary);
        // Independent literal expression retained from Contours: the plateau
        // owner, Difference and disc constructor run without the new helper.
        var protectedSlope=VCarveRampSurface.Plateau(outline.Boundary,Options,request.Depth,request.Grade,default);
        var literalFloor=PolygonOps.Difference(outline.Boundary,protectedSlope);
        var literalDisc=new VCarveDiscSweepExclusion(literalFloor,Options.DiscRadius,default,null,"literal-control");
        using var lease=owner.PrefetchInitialOutline(default,0);Assert.NotNull(lease);await lease!.Completion;
        owner.InitialOutlineRings(outline,default,null,out var prepared);Assert.NotNull(prepared);
        var result=prepared!.Take(request,default,null);Assert.NotNull(result);
        EqualRings(literalFloor,result!.MaterialFloor);EqualRings(literalDisc.SweptRegion,result.Disc.SweptRegion);
        Assert.Null(prepared.Take(request,default,null));Assert.NotEmpty(result.MaterialFloor);
        ((IList<PointD>)result.MaterialFloor[0])[0]=new(999,999);
        var fresh=VCarveRampSurface.BuildRestFillPreparation(request,default,null);
        EqualRings(literalFloor,fresh.MaterialFloor);
        Assert.InRange(owner.InitialOutlineCacheState.Bytes,0,64L<<20);
    }

    [Fact]
    public async Task ADifferentDiscRadiusKeepsRawReuseButCannotConsumeTheWrongRestDomain()
    {
        var owner=Prepare();var outline=Outline(owner);
        using var lease=owner.PrefetchInitialOutline(default,0);Assert.NotNull(lease);await lease!.Completion;
        var log=new List<string>();owner.InitialOutlineRings(outline,default,log.Add,out var prepared);Assert.NotNull(prepared);
        var request=Request(outline.Boundary) with{DiscRadius=1.1};
        Assert.Null(prepared!.Take(request,default,log.Add));
        Assert.Contains(log,l=>l.Contains("stage=outline-prefetch event=hit"));
        Assert.Contains(log,l=>l.Contains("stage=rest-fill-prefetch event=miss"));
    }

    [Theory]
    [InlineData("floor")]
    [InlineData("grade")]
    [InlineData("radius")]
    [InlineData("boundary-order")]
    [InlineData("source-order")]
    [InlineData("target-order")]
    [InlineData("target-null")]
    public void ExactHelperKeyRejectsEveryChangedConsumedInput(string change)
    {
        var original=Request([Rectangle(0,0,30,30)]) with{Target=[Rectangle(1,1,29,29)]};
        var changed=change switch
        {
            "floor"=>original with{Depth=Math.BitIncrement(original.Depth)},
            "grade"=>original with{Grade=Math.BitIncrement(original.Grade)},
            "radius"=>original with{DiscRadius=Math.BitIncrement(original.DiscRadius)},
            "boundary-order"=>original with{Boundary=original.Boundary.Select(r=>(IReadOnlyList<PointD>)r.Reverse().ToArray()).ToArray()},
            "source-order"=>original with{Crests=original.Crests.Reverse().ToArray()},
            "target-order"=>original with{Target=original.Target!.Select(r=>(IReadOnlyList<PointD>)r.Reverse().ToArray()).ToArray()},
            "target-null"=>original with{Target=null},
            _=>throw new ArgumentOutOfRangeException(nameof(change))
        };
        var key=VCarvePreparation.RestFillKey.Capture(original,default);Assert.NotNull(key);
        var other=VCarvePreparation.RestFillKey.Capture(changed,default);Assert.NotNull(other);
        Assert.False(key!.Same(other!));
    }

    [Fact]
    public void CapturedInputsAndKeysOwnTheirMutableInputSnapshots()
    {
        var input=Rectangle(0,0,30,30);var request=Request([input]);
        var key=VCarvePreparation.RestFillKey.Capture(request,default);Assert.NotNull(key);
        input[0]=new(999,999);
        Assert.True(key!.Same(VCarvePreparation.RestFillKey.Capture(request,default)!));
        Assert.DoesNotContain(request.Boundary.SelectMany(r=>r),p=>p.x==999);
        var changed=request with{Crests=[request.Crests[0] with{Points=[new(0,0,BitConverter.Int64BitsToDouble(long.MinValue)),new(1,1,0)]}]};
        // Unsupported signed/nonzero Z never gets an incomplete XY-only key.
        Assert.Null(VCarvePreparation.RestFillKey.Capture(changed,default));
    }

    [Fact]
    public void CancellationBeforeConstructionOrDuringPrefetchCannotLeaveADetachedHelper()
    {
        using var canceled=new CancellationTokenSource();canceled.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(()=>VCarveRampSurface.BuildRestFillPreparation(
            Request([Rectangle(0,0,30,30)]),canceled.Token,null));
        var owner=Prepare();using var cancellation=new CancellationTokenSource();
        var lease=owner.PrefetchInitialOutline(cancellation.Token,0);Assert.NotNull(lease);cancellation.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(()=>lease!.Dispose());
        Assert.True(lease!.Completion.IsCompleted);Assert.InRange(owner.InitialOutlineCacheState.Bytes,0,64L<<20);
    }
}
