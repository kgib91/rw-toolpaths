using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveMaterialClearingTests
{
    private static readonly VCarveCutterProfile Rounded=new([
        new(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),
        new(CarvingCutterSectionKind.Linear,1.5875,3,20),
        new(CarvingCutterSectionKind.Linear,3,3,90.4)],12.7);
    private static PointD[] Rectangle(double a,double b,double c,double d)=>[new(a,b),new(c,b),new(c,d),new(a,d)];

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void VerifiedLowerFloorAndEquivalentLowerStockHaveTheSameClearingFront(bool island)
    {
        PointD[] wedge=[new(8,8),new(32,8),new(20,36)];
        PointD[] hole=[new(14,10),new(20,32),new(26,10)];
        IReadOnlyList<IReadOnlyList<PointD>> boundary=island?[Rectangle(4,4,36,36),hole]:[wedge];
        var stock=new VCarveClearedStock([new(new(-20,20,-6.35),new(60,20,-6.35),50,50,50)], [6.35],Rectangle(0,0,40,40));
        var owner=new VCarveMaterialSurface(boundary,stock,Rounded,12.7,12.7,default,null);
        var construct=owner.CreateClearingInset(boundary,0);
        var front=construct(12.7,Rounded.PhysicalRadiusAtDepth(12.7))!;
        var equivalent=PolygonOps.CutterInset(boundary,Rounded.PhysicalRadiusAtDepth(6.35));
        Assert.Equal(Rounded.PhysicalRadiusAtDepth(6.35),front.MinimumRadius);
        Assert.Equal(equivalent.Sum(r=>r.Count),front.Rings.Sum(r=>r.Count));
        var actual=front.Rings.SelectMany(r=>r).Select(p=>(p.x,p.y)).Order().ToArray();
        var expected=equivalent.SelectMany(r=>r).Select(p=>(p.x,p.y)).Order().ToArray();
        Assert.Equal(expected,actual);
        Assert.Same(front,construct(12.7,Rounded.PhysicalRadiusAtDepth(12.7)));
        Console.WriteLine($"[VCarve.MaterialClearing] island={island} originalRadius={Rounded.PhysicalRadiusAtDepth(12.7):R} "
            +$"verifiedFloorRadius={front.MinimumRadius:R} equivalentFrontExact=True cachedFrontReused=True");
    }

    [Fact]
    public void PartialEarlierFacingAdvancesOnlyTheClearedPartOfThePerimeter()
    {
        var boundary=new[]{(IReadOnlyList<PointD>)Rectangle(8,8,32,32)};
        // A real flat sweep covers y<=20; the upper part remains at original top.
        var stock=new VCarveClearedStock([new(new(-10,0,-6.35),new(50,0,-6.35),20,20,20)], [6.35],Rectangle(0,0,40,40));
        var owner=new VCarveMaterialSurface(boundary,stock,Rounded,12.7,12.7,default,null);
        var front=owner.CreateClearingInset(boundary,0)(12.7,Rounded.PhysicalRadiusAtDepth(12.7))!;
        var points=front.Rings.SelectMany(r=>r).ToArray();
        Assert.Contains(points,p=>p.x<10&&p.x>9.92&&p.y<16);
        var upper=points.Where(p=>p.y>25&&p.x<20).ToArray();
        Assert.NotEmpty(upper);
        Assert.All(upper,p=>Assert.True(p.x-8>=Rounded.PhysicalRadiusAtDepth(12.7)-1e-6,$"Uncleared upper wall undercut at {p}"));
        foreach(var ring in front.Rings)
        for(int i=0;i<ring.Count;i++)
        {
            var a=ring[i];var b=ring[(i+1)%ring.Count];
            Assert.True(owner.IsLegalChord(new(a.x,a.y,-12.7),new(b.x,b.y,-12.7)));
        }
        Console.WriteLine($"[VCarve.MaterialClearing] partialFacing=True lowerFrontAdvanced=True upperWallRetained=True rings={front.Rings.Count}");
    }

    [Fact]
    public void NoVerifiedLowerLayerRetainsTheOriginalConstruction()
    {
        var boundary=new[]{(IReadOnlyList<PointD>)Rectangle(8,8,32,32)};
        var invalid=new VCarveClearedStock([new(new(-20.0001,20,-6.35),new(60,20,-6.35),50,0)],[],Rectangle(0,0,40,40));
        var owner=new VCarveMaterialSurface(boundary,invalid,Rounded,12.7,12.7,default,null);
        var front=owner.CreateClearingInset(boundary,0)(12.7,Rounded.PhysicalRadiusAtDepth(12.7));
        Assert.Null(front); // the native constructor remains the owner of this layer
    }

    [Fact]
    public void CancelledClearingConstructionDoesNotPublishAFront()
    {
        var boundary=new[]{(IReadOnlyList<PointD>)Rectangle(8,8,32,32)};
        using var cancellation=new CancellationTokenSource();
        var owner=new VCarveMaterialSurface(boundary,new([],[],Rectangle(0,0,40,40)),Rounded,12.7,12.7,cancellation.Token,null);
        var construct=owner.CreateClearingInset(boundary,0);cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(()=>construct(12.7,Rounded.PhysicalRadiusAtDepth(12.7)));
    }

    [Theory]
    [InlineData(false,0)]
    [InlineData(true,0)]
    [InlineData(false,5)]
    public void PublicClearingAndFlatFillConsumeTheStockAwareFront(bool interior,double band)
    {
        var boundary=Rectangle(8,8,32,32);
        var stock=new VCarveClearedStock([new(new(-20,20,-6.35),new(60,20,-6.35),50,50,50)], [6.35],Rectangle(0,0,40,40));
        var sources=new List<TaggedToolpath>();var previous=MillingGeometryTrace.Observer.Value;
        try
        {
            MillingGeometryTrace.Observer.Value=(stage,_,paths)=>{if(stage=="vcarve-original-sources")sources.AddRange(paths);};
            var plan=VCarveToolpaths.Generate([new MillingRegion(boundary)],new ToolGeometry(3){VCarveProfile=Rounded},
                new DepthSchedule(12.7,2){SurfaceZ=12.7,StartDepth=12.7},
                new VCarveOptions{IncludeInteriorFill=interior,EdgeFillWidth=band,DetectCurves=false,
                    CalculateSweptArea=false,ClearedStock=stock},BoostVoronoiProvider.CreateDefault());
            Assert.NotEmpty(plan.Toolpaths);
        }
        finally{MillingGeometryTrace.Observer.Value=previous;}
        var clearing=sources.Where(p=>p.Category=="clearing").SelectMany(p=>p.Points).ToArray();
        Assert.NotEmpty(clearing);
        Assert.InRange(clearing.Min(p=>p.X)-8,Rounded.PhysicalRadiusAtDepth(6.35),Rounded.PhysicalRadiusAtDepth(6.35)+.003);
        Assert.Contains(sources,p=>p.Category=="final-carve");
    }
}
