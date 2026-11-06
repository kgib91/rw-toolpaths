using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveCoveredRunDemandTests
{
    private static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Hemisphere,0,1,1),new(CarvingCutterSectionKind.Linear,1,1,10)],5);
    private static readonly VCarveRingOrder.Plan Plan=new([-1],levels:[1]);
    private static readonly VCarveOptions Options=new(){PathToleranceMm=.03,Motion=new(1000,500,5000,5,0)};
    private static TaggedToolpath Cut(double[] x,double y=0,double z=-2)=>new(x.Select(v=>new Point3D(v,y,z)).ToArray(),[],0,"clearing",0,detectCurves:false)
        {ConcentricSource=new(Plan,0),ContinuationSourceOrdinal=37};
    private static TaggedToolpath[] Apply(TaggedToolpath[] source,TaggedToolpath[]? donors=null,TaggedToolpath[]? original=null,
        DepthSchedule? depth=null,VCarveOptions? options=null,Action<string>? log=null,bool reuseFinalLiteralChords=true)
        =>VCarveCoveredRunDemand.Apply(source,donors??[],original??source,Profile,depth??new(2,2),options??Options,default,
            log??Console.WriteLine,reuseFinalLiteralChords);

    [Fact]
    public void AWholeLongSourceCoveredByPermanentMandatoryWorkCanDisappear()
    {
        var path=Cut([0,10]);var donor=Cut([-1,11]);
        Assert.Empty(Apply([path],[donor]));
        Assert.Equal(new Point3D(0,0,-2),path.Points[0]);
    }

    [Fact]
    public void ALongCoveredInteriorPaysForItsReentryAndPreservesEveryRequiredChordAndPlan()
    {
        var path=Cut([0,20,80,100]);var donor=Cut([20,80]);
        var result=Apply([path],[donor]);
        Assert.Equal(2,result.Length);
        Assert.Equal(new[]{path.Points[0],path.Points[1]},result[0].Points);
        Assert.Equal(new[]{path.Points[2],path.Points[3]},result[1].Points);
        Assert.All(result,p=>{Assert.Same(Plan,p.ConcentricSource!.Plan);Assert.Equal(0,p.ConcentricSource.Ring);Assert.Null(p.ContinuationSourceOrdinal);});
        Assert.Equal(4,path.Points.Count);
    }

    [Fact]
    public void AShortCoveredInteriorKeepsItsOriginalTraceWhenReentryCostsMore()
    {
        var path=Cut([0,1,1.1,2]);var donor=Cut([1,1.1]);
        Assert.Equal(path,Assert.Single(Apply([path],[donor])));
    }

    [Fact]
    public void IdenticalSourcesCannotMutuallyDeleteTheirOnlyRemoval()
    {
        var a=Cut([0,10]);var b=Cut([0,10]);
        var result=Apply([a,b]);
        Assert.Single(result);Assert.Equal(a.Points,result[0].Points);
    }

    [Fact]
    public void PreviouslyOmittedTargetsMustFitTheFinalKeepersWithoutCompoundingAllowances()
    {
        var original=Cut([0,10],.038);var intermediate=Cut([0,10],.018);var donor=Cut([-1,11],0);
        // Original -> intermediate and intermediate -> donor each fit .03,
        // but original -> final donor does not. The whole optimization refuses.
        Assert.Equal(intermediate,Assert.Single(Apply([intermediate],[donor],[original,intermediate])));
    }

    [Theory]
    [InlineData(0,1)]
    [InlineData(1,2)]
    public void StepdownPredecessorsAndLaterDepthPassesAreRetained(int pass,double perPass)
    {
        var path=Cut([0,10]) with{DepthPassIndex=pass};var donor=Cut([-1,11]);
        Assert.Equal(path,Assert.Single(Apply([path],[donor],depth:new(2,perPass))));
    }

    [Fact]
    public void OriginalMaterialPlaneCannotBeReplacedByAStartingFloorDatum()
    {
        var path=Cut([0,10],z:8);var shallower=Cut([-1,11],z:9);
        Assert.Equal(path,Assert.Single(Apply([path],[shallower],depth:new(2,2){SurfaceZ=5},
            options:Options with{Motion=new(1000,500,5000,15,10)})));
    }

    [Fact]
    public void MissingOriginalTargetsAndUnsupportedMotionRetainTheSource()
    {
        var path=Cut([0,10]);var donor=Cut([-1,11]);
        Assert.Equal(path,Assert.Single(Apply([path],[donor],original:[])));
        var ramp=path with{Spans=[new(0,1,ToolpathSpanKind.Ramp)]};
        Assert.Equal(ramp,Assert.Single(Apply([ramp],[donor])));
    }

    [Fact]
    public void CancellationPropagatesRatherThanSilentlyAcceptingOrRetrying()
    {
        using var cancel=new CancellationTokenSource();cancel.Cancel();var source=new[]{Cut([0,10])};
        Assert.Throws<OperationCanceledException>(()=>VCarveCoveredRunDemand.Apply(source,[],source,Profile,new(2,2),Options,cancel.Token,null));
    }

    [Fact]
    public void ACurveAwareSourceCannotBeSubstitutedByItsPointPolyline()
    {
        var line=Cut([0,10]);var curveAware=new TaggedToolpath(line.Points,[],0,"clearing",0,detectCurves:true);
        Assert.Equal(curveAware,Assert.Single(Apply([curveAware],[Cut([-1,11])])));
    }

    [Fact]
    public void NoOmissionOccursWhenLaterGeometryConsumesTheWholeFinishReserve()
    {
        var source=Cut([0,10]);
        Assert.Equal(source,Assert.Single(Apply([source],[Cut([-1,11])],options:Options with{OutputChordToleranceMm=.03})));
    }

    static long Counter(string report,string name)=>long.Parse(System.Text.RegularExpressions.Regex.Match(report,
        $@"\b{name}=(\d+)").Groups[1].Value);
    static string Accepted(IReadOnlyList<string> log)=>Assert.Single(log.Where(l=>l.Contains("accepted=True")));
    static void EqualGeometry(IReadOnlyList<TaggedToolpath> expected,IReadOnlyList<TaggedToolpath> actual)
    {
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<expected.Count;i++)
        {
            Assert.Equal(expected[i].Points,actual[i].Points);Assert.Equal(expected[i].Spans,actual[i].Spans);
            Assert.Equal(expected[i].ConcentricSource,actual[i].ConcentricSource);
            Assert.Equal(expected[i].RegionIndex,actual[i].RegionIndex);Assert.Equal(expected[i].DepthPassIndex,actual[i].DepthPassIndex);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FinalLiteralIdentityKeepsTheFullAuditGeometryAndAvoidsRedundantProofs(bool reverse)
    {
        var source=Cut([0,20,80,100]);var donor=Cut(reverse?[80,20]:[20,80]);
        var beforeLog=new List<string>();var afterLog=new List<string>();
        var before=Apply([source],[donor],log:beforeLog.Add,reuseFinalLiteralChords:false);
        var after=Apply([source],[donor],log:afterLog.Add);
        EqualGeometry(before,after);Assert.Equal(2,after.Length);
        var old=Accepted(beforeLog);var current=Accepted(afterLog);
        Assert.Equal(4,Counter(current,"originalTargetIntervalsVerified"));
        Assert.Equal(4,Counter(current,"finalNativeLiteralHits"));Assert.Equal(4,Counter(current,"finalPostedLiteralHits"));
        Assert.Equal(0,Counter(current,"finalNativeFullProofs"));Assert.Equal(0,Counter(current,"finalPostedFullProofs"));
        Assert.Equal(4,Counter(old,"finalNativeFullProofs"));Assert.Equal(4,Counter(old,"finalPostedFullProofs"));
        Assert.Contains("nominationMs=",current);Assert.Contains("finalStockBuildMs=",current);Assert.Contains("finalTargetAuditMs=",current);
    }

    [Fact]
    public void OmittedTargetsWithoutAnIdenticalFinalChordStillUseBothOriginalFullProofs()
    {
        var path=Cut([0,10]);var donor=Cut([-1,11]);var log=new List<string>();
        Assert.Empty(Apply([path],[donor],log:log.Add));var report=Accepted(log);
        Assert.Equal(2,Counter(report,"originalTargetIntervalsVerified"));
        Assert.Equal(1,Counter(report,"finalNativeLiteralHits"));Assert.Equal(1,Counter(report,"finalPostedLiteralHits"));
        Assert.Equal(1,Counter(report,"finalNativeFullProofs"));Assert.Equal(1,Counter(report,"finalPostedFullProofs"));
        var original=Cut([0,10],.038);var intermediate=Cut([0,10],.018);
        var refused=new List<string>();
        Assert.Equal(intermediate,Assert.Single(Apply([intermediate],[Cut([-1,11])],[original,intermediate],log:refused.Add)));
        Assert.Contains(refused,l=>l.Contains("accepted=False")&&l.Contains("reason=original-target-coverage-refused")
            &&l.Contains("finalNativeFullProofs=1"));
    }

    [Fact]
    public void PostedLiteralIdentityDoesNotBypassAnUnmatchedNativeProof()
    {
        var path=Cut([.0001,20.0001,80.0001,100.0001]);var donor=Cut([20.0002,80.0002]);
        var log=new List<string>();var result=Apply([path],[donor],log:log.Add);
        EqualGeometry(Apply([path],[donor],reuseFinalLiteralChords:false),result);
        var report=Accepted(log);Assert.Equal(1,Counter(report,"finalNativeFullProofs"));
        Assert.Equal(0,Counter(report,"finalPostedFullProofs"));Assert.Equal(4,Counter(report,"finalPostedLiteralHits"));
    }

    [Theory]
    [InlineData(0d,0d)]
    [InlineData(5.2,12.7)]
    public void ModalVerticalAndIndependentFirstCursorsRetainTheirLiteralFrames(double surface,double material)
    {
        var path=new TaggedToolpath(new Point3D[]{new(1000,50,material-1),new(1000+.000001,50+.00000001,material-2),
            new(1020,50,material-2),new(1080,50,material-2),new(1100,50,material-2)},[],0,"clearing",0,detectCurves:false)
            {ConcentricSource=new(Plan,0)};
        var donor=Cut([1020,1080],50,material-2);
        var depth=new DepthSchedule(2,2){SurfaceZ=surface};var options=Options with{Motion=new(1000,500,5000,material+5,material)};
        EqualGeometry(Apply([path],[donor],depth:depth,options:options,reuseFinalLiteralChords:false),
            Apply([path],[donor],depth:depth,options:options));
        var chords=VCarveCoveredRunDemand.ReconstructForAudit(path,surface,material).ToArray();
        var first=chords[0];Assert.Equal((float)path.Points[0].X,first.NativeA.X);Assert.Equal((float)path.Points[0].Y,first.NativeA.Y);
        Assert.Equal(first.NativeA.X,first.NativeB.X);Assert.Equal(first.NativeA.Y,first.NativeB.Y);
        Assert.Equal(first.PostedA.X,first.PostedB.X);Assert.Equal(first.PostedA.Y,first.PostedB.Y);
        double postedTop=(float)material-(float)surface;
        Assert.Equal(ToolpathCommandRounding.Round((float)path.Points[0].Z-(float)surface)-postedTop,first.PostedA.Z);
        Assert.Equal(first.NativeB,chords[1].NativeA);Assert.Equal(first.PostedB,chords[1].PostedA);
        var next=Cut([2,12],7,material-2);var start=VCarveCoveredRunDemand.ReconstructForAudit(next,surface,material).First();
        Assert.Equal(2,start.NativeA.X);Assert.Equal(7,start.NativeA.Y);Assert.NotEqual(chords[^1].NativeB,start.NativeA);
        Assert.Equal(5,path.Points.Count);Assert.Equal(new Point3D(1000,50,material-1),path.Points[0]);
    }
}
