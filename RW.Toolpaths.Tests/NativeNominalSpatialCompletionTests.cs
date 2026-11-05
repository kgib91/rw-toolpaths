using System.IO.Compression;
using System.Reflection;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeNominalSpatialCompletionTests
{
    [Theory]
    [InlineData(false,9)]
    [InlineData(true,9)]
    [InlineData(false,8)]
    [InlineData(true,8)]
    public void AuthenticNominalBranchCompletesWithoutContactAxialFreedom(bool reverse,int region)
    {
        var (points,knots,boundary,profile,surface)=Fixture(region);
        if(reverse){points=points.Reverse().ToArray();knots=knots.Reverse().Select(i=>points.Length-1-i).ToArray();}
        var path=Path(points);using var deadline=new CancellationTokenSource(TimeSpan.FromSeconds(30));
        var exact=new VCarveNativeContainment(boundary,profile,deadline.Token);
        var plan=Assert.Single(VCarveNativeCurvePlanning.Nominate([path],[boundary],profile,surface,deadline.Token,
            stockChord:exact.Legal,stockCurve:m=>VCarveNativeCutterProof.Certified(m,exact,profile,out _),
            selectedSourceOrdinals:[knots],requiredSourceOrdinals:[[0,points.Length-1]],
            sourceContract:VCarveNativeSourceContract.PreparedSpatialDirections));
        Assert.True(plan.CandidateVerified);Assert.Equal(0,plan.StockContactEnvelopeCurves);
        Assert.Equal(0,plan.MaximumSourceAxialDirectionChangeDegrees);
        Assert.Equal(points[0],plan.Primitives[0].Move.Start);Assert.Equal(points[^1],plan.Primitives[^1].Move.End);
        var raw=points.Select(p=>p with{Z=p.Z-surface}).ToArray();
        foreach(var primitive in plan.Primitives)
        {
            Assert.True(primitive.LengthLowerMm>=.3);Assert.Null(primitive.ShortSourceFeature);
            var relative=VCarveNativeCurvePlanning.TranslateZ(primitive.Move,-surface);
            Assert.True(VCarveNativeGeometricProof.Prove(relative,raw,primitive.SourceFirst,primitive.SourceLast,.3).Certified);
            Assert.True(VCarveNativeCutterProof.Certified(relative,exact,profile,out _));
            Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(primitive.Move,points,primitive.SourceFirst,primitive.SourceLast,out _));
        }
        Point3D Direction(Point3D a,Point3D b)=>VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(b,a));
        Assert.True(VCarveNativeCurveMath.Distance(Direction(points[0],points[knots[1]]),
            VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(plan.Primitives[0].Move,0)))<1e-8);
        Assert.True(VCarveNativeCurveMath.Distance(Direction(points[knots[^2]],points[^1]),
            VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(plan.Primitives[^1].Move,1)))<1e-8);
    }

    [Fact]
    public void SpatialSearchRetainsAnActualCutterRefusal()
    {
        var(points,knots,boundary,profile,surface)=Fixture();
        var plan=Assert.Single(VCarveNativeCurvePlanning.Nominate([Path(points)],[boundary],profile,surface,
            stockChord:(_,_)=>false,stockCurve:_=>false,selectedSourceOrdinals:[knots],
            requiredSourceOrdinals:[[0,points.Length-1]],sourceContract:VCarveNativeSourceContract.PreparedSpatialDirections));
        Assert.False(plan.CandidateVerified);Assert.False(plan.NewCurveChecksPassed);Assert.Equal(0,plan.StockContactEnvelopeCurves);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WholeShortNominalFeatureKeepsLiteralOwnershipAndFreshClearance(bool blocked)
    {
        Point3D[] points=[new(0,0,-1),new(.08,0,-1),new(.18,0,-1)];
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
        PointD[][] boundary=[[new(-2,-2),new(2,-2),new(2,2),new(-2,2)]];int checks=0;
        var plan=Assert.Single(VCarveNativeCurvePlanning.Nominate([Path(points)],[boundary],profile,0,
            stockChord:(_,_)=>{checks++;return !blocked;},stockCurve:_=>!blocked,
            selectedSourceOrdinals:[[0,1,2]],sourceContract:VCarveNativeSourceContract.PreparedSpatialDirections));
        Assert.Equal(!blocked,plan.CandidateVerified);Assert.True(checks>=2);Assert.Equal(2,plan.Primitives.Count);
        for(int i=0;i<2;i++)
        {
            var p=plan.Primitives[i];Assert.Equal(new VCarveNativeShortFeature(0,2),p.ShortSourceFeature);
            Assert.Equal(points[i],p.Move.Start);Assert.Equal(points[i+1],p.Move.End);Assert.Equal(0,p.Move.MaxError);
        }
    }

    [Fact]
    public void APreparationSplitInsideALongSourceCannotBecomeAWholeFeatureException()
    {
        Point3D[] points=[new(0,0,-1),new(.01,0,-1),new(1,0,-1)];
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
        PointD[][] boundary=[[new(-2,-2),new(2,-2),new(2,2),new(-2,2)]];
        var plan=Assert.Single(VCarveNativeCurvePlanning.Nominate([Path(points)],[boundary],profile,0,
            selectedSourceOrdinals:[[0,1,2]],requiredSourceOrdinals:[[0,1,2]],
            sourceContract:VCarveNativeSourceContract.PreparedSpatialDirections));
        Assert.False(plan.CandidateVerified);Assert.All(plan.Primitives,p=>Assert.Null(p.ShortSourceFeature));
    }

    static TaggedToolpath Path(Point3D[] points)=>new(points,[new(0,points.Length-1,ToolpathSpanKind.Cut)],0,"nominal-medial",1,0,detectCurves:false);
    static (Point3D[] Points,int[] Selected,PointD[][] Boundary,VCarveCutterProfile Profile,double Surface) Fixture(int region=9)
    {
        string suffix=region==9?"":"-8";
        using var stream=Assembly.GetExecutingAssembly().GetManifestResourceStream($"RW.Toolpaths.Tests.Fixtures.nominal-spatial-task-management{suffix}.json.gz")!;
        using var gzip=new GZipStream(stream,CompressionMode.Decompress);using var document=JsonDocument.Parse(gzip);
        var root=document.RootElement;var json=new JsonSerializerOptions{IncludeFields=true};T Read<T>(JsonElement e)=>e.Deserialize<T>(json)!;
        var profile=root.GetProperty("Profile");return(Read<Point3D[]>(root.GetProperty("Points")),Read<int[]>(root.GetProperty("Selected")),
            Read<PointD[][]>(root.GetProperty("Boundary")),new(Read<CarvingCutterSection[]>(profile.GetProperty("Sections")),profile.GetProperty("MaximumDepth").GetDouble()),root.GetProperty("Surface").GetDouble());
    }
}
