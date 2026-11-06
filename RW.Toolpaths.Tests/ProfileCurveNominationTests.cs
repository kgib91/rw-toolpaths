using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ProfileCurveNominationTests
{
    private static PointD[] Circle(double x,double y,double radius)=>Enumerable.Range(0,96)
        .Select(i=>new PointD(x+radius*Math.Cos(2*Math.PI*i/96),y+radius*Math.Sin(2*Math.PI*i/96))).ToArray();
    private static MillingRegion[] Regions()=>[new([new(-20,-15),new(20,-15),new(20,15),new(-20,15)],
        [Circle(-3.2,0,2),Circle(3.2,0,2)])];
    private static string[] Source(ToolpathPlan plan)=>Source(plan.Toolpaths);
    private static string[] Source(IEnumerable<TaggedToolpath> paths)=>paths.Select(path=>
        $"{path.RegionIndex}:{path.DepthPassIndex}:{path.VisitIdentity}:{path.Category}:"+
        string.Join(';',path.Points.Select(p=>$"{BitConverter.DoubleToInt64Bits(p.X):X16},{BitConverter.DoubleToInt64Bits(p.Y):X16},{BitConverter.DoubleToInt64Bits(p.Z):X16}"))+
        ":"+string.Join(';',path.Spans.Select(s=>$"{s.StartIndex},{s.EndIndex},{s.Kind}"))).ToArray();

    [Fact]
    public void DefaultCurveMetadataAndOptOutKeepTheIdenticalExecutablePointSpanSource()
    {
        Assert.True(new ProfileOptions().DetectCurves);
        // The detector certifies the original polygon chords at .00025 mm.
        // This circle's chord sagitta is below that bound; the coarser routing
        // fixture above intentionally does not promise a fitted arc.
        MillingRegion[] circle=[new(Enumerable.Range(0,720).Select(i=>new PointD(
            15*Math.Cos(2*Math.PI*i/720),15*Math.Sin(2*Math.PI*i/720))).ToArray())];
        var options=new ProfileOptions{Side=ProfileSide.OnLine,Ramp=RampSettings.Plunge,CalculateSweptArea=false};
        var original=ProfileToolpaths.Generate(circle,ToolGeometry.Flat(1),new(2,1),options);
        var linear=ProfileToolpaths.Generate(circle,ToolGeometry.Flat(1),new(2,1),options with{DetectCurves=false});
        Assert.NotEmpty(original.Toolpaths);Assert.Equal(Source(original),Source(linear));
        Assert.Contains(original.Toolpaths,p=>p.Moves.Any(m=>m.Kind!=ToolMoveKind.Line));
        Assert.All(linear.Toolpaths,p=>{Assert.False(p.DetectCurves);Assert.True(p.PreserveSourceTraversal);Assert.IsType<LineToolMoves>(p.Moves);});
        Assert.Equal(original.Diagnostics,linear.Diagnostics);
    }

    private sealed class EffectfulClearance(List<string> calls,Action? onCall=null):ILinkClearance
    {
        public bool IsTravelSafe(Point3D from,Point3D to,double radius)
        {
            calls.Add($"{from}:{to}:{radius:R}");onCall?.Invoke();return calls.Count%3!=0;
        }
    }

    [Fact]
    public void OpaqueClearanceCallsKeepTheirExactOrderAndEffects()
    {
        ToolpathPlan Run(bool detect,List<string> calls)=>ProfileToolpaths.Generate(Regions(),ToolGeometry.Flat(1),new(2,1),
            new(){Side=ProfileSide.Inside,CalculateSweptArea=false,DetectCurves=detect,LinkClearance=new EffectfulClearance(calls)});
        var oldCalls=new List<string>();var newCalls=new List<string>();
        var original=Run(true,oldCalls);var linear=Run(false,newCalls);
        Assert.NotEmpty(oldCalls);Assert.Equal(oldCalls,newCalls);Assert.Equal(Source(original),Source(linear));
    }

    [Fact]
    public void ScanlinePerimeterRetainsLinearMetadataLikeItsRasterSources()
    {
        var plan=ScanlinePocketToolpaths.Generate(Regions(),ToolGeometry.Flat(1),new(1,1),
            new(){StepOver=.5,CalculateSweptArea=false,DetectCurves=true});
        Assert.NotEmpty(plan.Toolpaths);
        Assert.Contains(plan.Toolpaths,p=>p.Category==ProfileToolpaths.Category);
        Assert.All(plan.Toolpaths,p=>{Assert.False(p.DetectCurves);Assert.IsType<LineToolMoves>(p.Moves);});
    }

    [Fact]
    public void CancellationFromAnOpaqueProofStillPreventsEitherModeFromPublishing()
    {
        List<string> Run(bool detect)
        {
            using var cancellation=new CancellationTokenSource();var calls=new List<string>();
            Assert.ThrowsAny<OperationCanceledException>(()=>ProfileToolpaths.Generate(Regions(),ToolGeometry.Flat(1),new(2,1),
                new(){Side=ProfileSide.Inside,CalculateSweptArea=false,DetectCurves=detect,
                    LinkClearance=new EffectfulClearance(calls,cancellation.Cancel)},cancellation.Token));
            return calls;
        }
        var original=Run(true);Assert.NotEmpty(original);Assert.Equal(original,Run(false));
    }

    [Fact]
    public void ClippedOptOutKeepsLinearMovesAndTheLiteralOriginalSourceAndTraversal()
    {
        MillingRegion[] regions=[new(Circle(0,0,15))];
        var options=new ProfileOptions{Side=ProfileSide.OnLine,Ramp=RampSettings.Plunge,CalculateSweptArea=false,
            ClipBoundary=[new(0,-20),new(20,-20),new(20,20),new(0,20)]};
        var original=ProfileToolpaths.Generate(regions,ToolGeometry.Flat(1),new(2,1),options);
        var linear=ProfileToolpaths.Generate(regions,ToolGeometry.Flat(1),new(2,1),options with{DetectCurves=false});
        Assert.NotEmpty(linear.Toolpaths);Assert.Equal(Source(original),Source(linear));
        Assert.All(linear.Toolpaths,p=>{Assert.False(p.DetectCurves);Assert.True(p.PreserveSourceTraversal);Assert.IsType<LineToolMoves>(p.Moves);});
    }

    [Fact]
    public void ExplicitProfileTraversalSurvivesPrependingAndBlocksPreviouslyIneligibleReductions()
    {
        var path=new TaggedToolpath(new Point3D[]{new(0,0,-1),new(1,0,-1),new(2,0,-1),new(3,0,-1)},
            [new(0,3,ToolpathSpanKind.Cut)],0,ProfileToolpaths.Category,0,detectCurves:false){PreserveSourceTraversal=true};
        Assert.True(path.RequiresSourcePreservation);
        Assert.Equal(path.Points,SurfaceToolpathSimplifier.Simplify(path,.01,default).Points);
        Assert.Equal(path.Points,ToolpathCollinearCompactor.Compact(path,0).Points);
        var joined=ToolpathContinuationPlanner.Prepend(path,[new(-1,0,-1),path.Points[0]]);
        Assert.True(joined.PreserveSourceTraversal);Assert.False(joined.DetectCurves);
        var ordinary=path with{PreserveSourceTraversal=false};Assert.False(ordinary.RequiresSourcePreservation);
    }

    [Fact]
    public void OwnedTrimKeepsItsOriginalIntentionalLinearTraversalBoundaryForBothSources()
    {
        PointD[] Box(double r)=>[new(-r,-r),new(r,-r),new(r,r),new(-r,r)];
        var access=new MillingBoundaryAccess([],Box(12)).Resolve([new MillingRegion(Box(10))],ToolGeometry.Flat(.6),
            new(1,1),false,30,0,default);
        Point3D[] points=[new(-8,0,-.5),new(-2,0,-.5),new(2,0,-.5),new(8,0,-.5)];
        ToolpathSpan[] spans=[new(0,3,ToolpathSpanKind.Cut)];
        var oldSource=new TaggedToolpath(points,spans,0,ProfileToolpaths.Category,0,7,detectCurves:true);
        var newSource=new TaggedToolpath(points,spans,0,ProfileToolpaths.Category,0,7,detectCurves:false){PreserveSourceTraversal=true};
        Assert.True(oldSource.RequiresSourcePreservation);Assert.True(newSource.RequiresSourcePreservation);
        var oldTrim=access.TrimOwnedFlatRadius([oldSource],.6,default);
        var newTrim=access.TrimOwnedFlatRadius([newSource],.6,default);
        Assert.NotEmpty(oldTrim);Assert.Equal(Source(oldTrim),Source(newTrim));
        Assert.All(oldTrim.Concat(newTrim),p=>{Assert.False(p.RequiresSourcePreservation);Assert.IsType<LineToolMoves>(p.Moves);});
    }
}
