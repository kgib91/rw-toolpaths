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
        DepthSchedule? depth=null,VCarveOptions? options=null)
        =>VCarveCoveredRunDemand.Apply(source,donors??[],original??source,Profile,depth??new(2,2),options??Options,default,Console.WriteLine);

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
}
