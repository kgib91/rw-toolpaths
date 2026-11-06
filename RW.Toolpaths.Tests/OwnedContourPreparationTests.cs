using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class OwnedContourPreparationTests
{
    static PointD[] Box(double x,double y,double z,double w)=>[new(x,y),new(z,y),new(z,w),new(x,w)];
    static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Linear,.1,1.1,2)],1);
    static T Mode<T>(bool enabled,Func<T> work)
    {
        var oldTrim=MillingBoundaryAccess.Resolved.ContourInteriorFirstEnabled.Value;
        var oldFill=VCarveDiscSweepExclusion.ParallelFillRegionsEnabled.Value;
        try
        {
            MillingBoundaryAccess.Resolved.ContourInteriorFirstEnabled.Value=enabled;
            VCarveDiscSweepExclusion.ParallelFillRegionsEnabled.Value=enabled;return work();
        }
        finally
        {
            MillingBoundaryAccess.Resolved.ContourInteriorFirstEnabled.Value=oldTrim;
            VCarveDiscSweepExclusion.ParallelFillRegionsEnabled.Value=oldFill;
        }
    }
    static (long,long)[] Bits(IReadOnlyList<(double Start,double End)> intervals)
        =>intervals.Select(p=>(BitConverter.DoubleToInt64Bits(p.Start),BitConverter.DoubleToInt64Bits(p.End))).ToArray();
    static string[] Words(IEnumerable<OffsetRing> rings)=>rings.Select(r=>
        $"{r.RegionIndex}/{r.Level}/{r.IsHole}/{r.IsClosed}/{r.OffsetDistance:R}/"+string.Join(';',r.Points.Select(p=>$"{p.x:R},{p.y:R}"))).ToArray();

    [Fact]
    public void InteriorFirstMatchesLiteralIntervalBitsAcrossHolesLongChordsTangenciesAndDegeneratePoses()
    {
        var outer=new Path64{new(-10000,-10000),new(10000,-10000),new(10000,10000),new(-10000,10000)};
        var hole=new Path64{new(-2000,-2000),new(-2000,2000),new(2000,2000),new(2000,-2000)};
        var index=new PolygonOps.BoundarySegmentIndex(new Paths64{outer,hole});
        var random=new Random(884319);int hits=0;
        var cases=new List<(Point64 A,Point64 B,double Radius)>{
            (new(-6000,-4000),new(-5900,-4000),1000),
            (new(0,0),new(0,0),1000), // The hole is not filled interior.
            (new(-10000,0),new(-10000,0),0),
            (new(-12000,0),new(-12000,0),2000),
            (new(-12000,0),new(-12000,0),Math.BitDecrement(2000d)),
            (new(-12000,0),new(-12000,0),Math.BitIncrement(2000d)),
            (new(-20000,0),new(20000,0),1000),
            (new(-1000000000000,-1000000000000),new(1000000000000,1000000000000),1),
            (new(9500,9500),new(10500,10500),Math.Sqrt(500000d)),
            (new(-5000,-5000),new(-5000,-5000),double.NaN),
            (new(-5000,-5000),new(-5000,-5000),double.PositiveInfinity)
        };
        for(int i=0;i<500;i++)
        {
            long x=random.Next(-15000,15001),y=random.Next(-15000,15001);
            cases.Add((new(x,y),new(x+random.Next(-5000,5001),y+random.Next(-5000,5001)),random.NextDouble()*6000));
        }
        foreach(var (a,b,radius) in cases)
        {
            var literal=index.ContactIntervals(a,b,radius,default,out _);
            var actual=index.ContactIntervalsInteriorFirst(a,b,radius,default,out _,out bool hit);
            Assert.Equal(Bits(literal),Bits(actual));
            if(hit){hits++;Assert.Equal(new[]{(0d,1d)},actual);}
        }
        Assert.True(hits>0);
        index.ContactIntervalsInteriorFirst(new(0,0),new(0,0),1000,default,out _,out bool holeHit);Assert.False(holeHit);
        var empty=new PolygonOps.BoundarySegmentIndex(new());
        Assert.Empty(empty.ContactIntervalsInteriorFirst(new(),new(),1,default,out _,out _));
    }

    [Fact]
    public void OwnedTrimKeepsGenericCallbackOrderAndExactFragmentsSpansMetadataAndInput()
    {
        var region=new MillingRegion(Box(-10,-10,10,10),[Box(-2,-2,2,2).Reverse().ToArray()]);
        var access=new MillingBoundaryAccess([],Box(-12,-12,12,12)).Resolve([region],new(.6){VCarveProfile=Profile},new(1,1),true,50,0,default);
        var plan=new VCarveRingOrder.Plan([-1]);
        TaggedToolpath Path(int visit,Point3D[] points)=>new(points,[new(0,points.Length/2,ToolpathSpanKind.Cut),
            new(points.Length/2,points.Length-1,ToolpathSpanKind.Link)],0,"contour",0,visit,detectCurves:false){ConcentricSource=new(plan,0)};
        var source=new[]{Path(1,Enumerable.Range(0,81).Select(i=>new Point3D(-8+i*.1,-7,-.6)).ToArray()),
            Path(2,[new(-8,0,-.5),new(-1,0,-.6),new(1,0,-.7),new(8,0,-.8)]),
            Path(3,[new(-14,7,-.5),new(-8,7,-.5),new(8,7,-.5),new(14,7,-.5)])};
        var raw=source.Select(p=>p.Points.ToArray()).ToArray();var radii=new List<double>();
        var literal=access.Trim(source,d=>{radii.Add(d);return Profile.PhysicalRadiusAtDepth(d);},default);
        var messages=new List<string>();var actual=Mode(true,()=>access.TrimOwnedContours(source,Profile,default,messages.Add));
        Assert.Equal(source.Sum(p=>p.Points.Count-1),radii.Count);AssertPaths(literal,actual);
        Assert.Contains(messages,m=>m.Contains("interiorShortcutHits=")&&!m.Contains("interiorShortcutHits=0 "));
        for(int i=0;i<source.Length;i++)Assert.Equal(raw[i],source[i].Points);
        AssertPaths(literal,Mode(false,()=>access.TrimOwnedContours(source,Profile,default)));
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void RestFillParallelCollectorKeepsLiteralGeometryRegionOrderNestingAndTraversal(bool insideOut)
    {
        IReadOnlyList<PointD>[] domain=[Box(0,0,10,10),Box(14,0,24,10),Box(28,0,38,10),Box(2,2,4,4).Reverse().ToArray()];
        IReadOnlyList<PointD>[] cleared=[Box(5,-1,19,7)];
        var disc=new VCarveDiscSweepExclusion(domain,.5,default,null,"test");var log=new List<string>();
        var literal=Mode(false,()=>disc.Fill(domain,cleared,.75,2,default,null));
        var actual=Mode(true,()=>disc.Fill(domain,cleared,.75,2,default,log.Add));
        Assert.NotEmpty(actual);Assert.Equal(Words(literal),Words(actual));
        Assert.All(actual,r=>Assert.InRange(r.Level,0,2));Assert.All(actual,r=>Assert.True(r.IsClosed));
        foreach(var rings in new[]{literal,actual})RingOffsetEngine.BuildNesting(rings);
        Assert.Equal(literal.Select(r=>r.Parent),actual.Select(r=>r.Parent));
        Assert.Equal(literal.Select(r=>string.Join(',',r.Children)),actual.Select(r=>string.Join(',',r.Children)));
        Assert.Equal(RingOffsetEngine.BuildTraversal(literal,insideOut).Order,RingOffsetEngine.BuildTraversal(actual,insideOut).Order);
        Assert.Contains(log,m=>m.Contains("stage=rest-fill-phases")&&m.Contains("completed=True")&&m.Contains("parallelRegions=True"));
    }

    [Fact]
    public void RestFillRetainsLiteralBudgetUnderNestedWorkerAndCancellationCannotPublish()
    {
        IReadOnlyList<PointD>[] domain=[Box(0,0,8,8),Box(12,0,20,8)];
        var disc=new VCarveDiscSweepExclusion(domain,.5,default,null,"test");
        var literal=Mode(false,()=>disc.Fill(domain,[],.75,0,default,null));
        using(GeometryParallelism.EnterWorker())
        {
            var nested=Mode(true,()=>disc.Fill(domain,[],.75,0,default,null));
            Assert.Equal(Words(literal),Words(nested));Assert.All(nested,r=>Assert.Equal(0,r.Level));
        }
        using var cancellation=new CancellationTokenSource();cancellation.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(()=>disc.Fill(domain,[],.75,1,cancellation.Token,null));
        using var afterCollection=new CancellationTokenSource();var messages=new List<string>();
        Assert.ThrowsAny<OperationCanceledException>(()=>Mode(true,()=>disc.Fill(domain,[],.75,1,afterCollection.Token,m=>
        {messages.Add(m);if(m.Contains("stage=rest-fill event=contoured"))afterCollection.Cancel();})));
        Assert.Contains(messages,m=>m.Contains("stage=rest-fill-phases")&&m.Contains("completed=False"));
    }

    [Theory]
    [InlineData(VCarveProfileDirection.TopDown)] [InlineData(VCarveProfileDirection.BottomUp)]
    public void CompleteOwnedContourNominationKeepsLiteralRingMetadataAndSourceGeometry(VCarveProfileDirection direction)
    {
        IReadOnlyList<PointD>[] boundary=[Box(0,0,20,20),Box(7,7,13,13).Reverse().ToArray()];
        var region=new MillingRegion(boundary[0],[boundary[1]]);
        var access=new MillingBoundaryAccess([],Box(-2,-2,22,22)).Resolve([region],new(.6){VCarveProfile=Profile},new(1,1),true,50,0,default);
        var options=new VCarveOptions{SmoothVCarving=true,SmoothRampAngleDegrees=50,StepOver=.5,
            IncludeInteriorFill=false,EdgeFillWidth=1,ExcludeDiscSweep=true,DiscRadius=.75,
            ProfileDirection=direction,AccessTarget=access,DetectCurves=false,
            RoughingRestChords=[new(new(3,3,-1),new(17,3,-1),.75)]};
        var literal=Mode(false,()=>VCarveRampSurface.Contours(boundary,options,Profile,1,0,default,null));
        var log=new List<string>();var actual=Mode(true,()=>VCarveRampSurface.Contours(boundary,options,Profile,1,0,default,log.Add));
        Assert.NotEmpty(actual);AssertPaths(literal,actual);
        Assert.Contains(log,m=>m.Contains("stage=outline-ramps-phases")&&m.Contains("completed=True")&&m.Contains("targetTrimMs="));
        for(int i=0;i<actual.Count;i++)
        {
            Assert.Equal(literal[i].ConcentricSource?.Ring,actual[i].ConcentricSource?.Ring);
            Assert.Equal(literal[i].ConcentricSource?.Plan.Parents,actual[i].ConcentricSource?.Plan.Parents);
        }
    }

    [Fact]
    public void InteriorFirstAndOwnedTrimHonorPreCancellation()
    {
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        var index=new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper([Box(-10,-10,10,10)]));
        Assert.ThrowsAny<OperationCanceledException>(()=>index.ContactIntervalsInteriorFirst(new(0,0),new(1,1),100,cancel.Token,out _,out _));
        var region=new MillingRegion(Box(-10,-10,10,10));
        var access=new MillingBoundaryAccess([],Box(-12,-12,12,12)).Resolve([region],new(.6){VCarveProfile=Profile},new(1,1),true,50,0,default);
        var path=new TaggedToolpath([new(0,0,-.5),new(.1,0,-.5)],[],0,"test",0,detectCurves:false);
        Assert.ThrowsAny<OperationCanceledException>(()=>access.TrimOwnedContours([path],Profile,cancel.Token));
    }

    static void AssertPaths(IReadOnlyList<TaggedToolpath> expected,IReadOnlyList<TaggedToolpath> actual)
    {
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<expected.Count;i++)
        {
            Assert.Equal(expected[i].Points,actual[i].Points);Assert.Equal(expected[i].Spans,actual[i].Spans);
            Assert.Equal(expected[i].RegionIndex,actual[i].RegionIndex);Assert.Equal(expected[i].Category,actual[i].Category);
            Assert.Equal(expected[i].DepthPassIndex,actual[i].DepthPassIndex);Assert.Equal(expected[i].VisitIdentity,actual[i].VisitIdentity);
        }
    }
}
