using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveAirRampFeedTests
{
    private static readonly VCarveCutterProfile Cone=new([new(CarvingCutterSectionKind.Linear,0,3,6)],5);
    private static readonly MillingMotionSettings Motion=new(1000,500,5000,10,0);

    private static TaggedToolpath EarlierCut()=>new(
        new Point3D[]{new(0,0,-4),new(10,0,-4),new(20,0,-4)},
        [new(0,2,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);

    private static TaggedToolpath Entry()=>new(
        new Point3D[]{new(0,0,-2),new(5,0,-2.25),new(10,0,-2.5),new(15,0,-3),new(20,0,-4),new(25,0,-4)},
        [new(0,4,ToolpathSpanKind.Ramp),new(4,5,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);

    private static List<TaggedToolpath> Improve(IReadOnlyList<TaggedToolpath> source,double? originalMaterialSurface=null,Point3D? incoming=null)
        =>VCarveAirRampFeed.Improve(source,Cone,0,Motion with{MaterialSurfaceZ=originalMaterialSurface??0},default,Console.WriteLine,
            originalMaterialSurface:originalMaterialSurface,incoming:incoming);

    private static void RequiredCutsStayExact(IReadOnlyList<TaggedToolpath> before,IReadOnlyList<TaggedToolpath> after)
    {
        Assert.Equal(before.Count,after.Count);
        for(int p=0;p<before.Count;p++){
            Assert.Equal(before[p].Points,after[p].Points);
            var original=ToolpathSpanKinds.Expand(before[p]);var actual=ToolpathSpanKinds.Expand(after[p]);
            for(int i=0;i<original.Length;i++)if(original[i]==ToolpathSpanKind.Cut)
                Assert.Equal(ToolpathSpanKind.Cut,actual[i]);
        }
    }

    [Fact]
    public void AnEarlierDeeperCutAcceleratesOnlyTheClearRampPrefixWithoutMovingAnyVertex()
    {
        var source=new[]{EarlierCut(),Entry()};var result=Improve(source);
        RequiredCutsStayExact(source,result);
        var kinds=ToolpathSpanKinds.Expand(result[1]);
        Assert.Equal(new[]{ToolpathSpanKind.ClearedTravel,ToolpathSpanKind.ClearedTravel,ToolpathSpanKind.ClearedTravel,
            ToolpathSpanKind.Ramp,ToolpathSpanKind.Cut},kinds);
        Assert.True(new ToolpathMotionCost(Motion,0).SequenceSeconds(null,result,null)
            <new ToolpathMotionCost(Motion,0).SequenceSeconds(null,source,null));
        VCarveReturnTravel.VerifyFinalTravel(result,Cone,0,default,Console.WriteLine);
    }

    [Fact]
    public void ContactWithAnEarlierCutHasNoPositiveClearanceForFastRampFeed()
    {
        var contact=new TaggedToolpath(new Point3D[]{new(0,0,-3.95),new(10,0,-4),new(20,0,-4)},
            [new(0,1,ToolpathSpanKind.Ramp),new(1,2,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);
        var source=new[]{EarlierCut(),contact};var result=Improve(source);
        RequiredCutsStayExact(source,result);
        Assert.Equal(ToolpathSpanKind.Ramp,ToolpathSpanKinds.Expand(result[1])[0]);
        Assert.DoesNotContain(result[1].Spans,s=>s.Kind==ToolpathSpanKind.ClearedTravel);
    }

    [Fact]
    public void ADeeperFutureCutCannotAuthorizeAnEarlierRamp()
    {
        var source=new[]{Entry(),EarlierCut()};var result=Improve(source);
        RequiredCutsStayExact(source,result);
        Assert.Equal(source[0].Spans,result[0].Spans);
        Assert.DoesNotContain(result[0].Spans,s=>s.Kind==ToolpathSpanKind.ClearedTravel);
    }

    [Fact]
    public void TheCommandDatumCannotReplaceTheOriginalMaterialTop()
    {
        var ramp=new TaggedToolpath(new Point3D[]{new(0,0,.3),new(1,0,.2)},
            [new(0,1,ToolpathSpanKind.Ramp)],0,"clearing",0,detectCurves:false);
        var result=Improve([ramp],originalMaterialSurface:5);
        Assert.Equal(ramp.Points,result[0].Points);Assert.Equal(ramp.Spans,result[0].Spans);
        Assert.DoesNotContain(result[0].Spans,s=>s.Kind==ToolpathSpanKind.ClearedTravel);
    }

    [Fact]
    public void TheMotionContextsOriginalTopIsRetainedWhenNoSeparateTopIsPassed()
    {
        var ramp=new TaggedToolpath(new Point3D[]{new(0,0,.3),new(1,0,.2)},
            [new(0,1,ToolpathSpanKind.Ramp)],0,"clearing",0,detectCurves:false);
        var result=VCarveAirRampFeed.Improve([ramp],Cone,0,Motion with{MaterialSurfaceZ=5},default,Console.WriteLine);
        Assert.Equal(ramp.Points,result[0].Points);Assert.Equal(ramp.Spans,result[0].Spans);
    }

    [Fact]
    public void NominalClearanceAloneCannotAuthorizeAFastRampWhenF3ConsumesTheReserve()
    {
        var ramp=new TaggedToolpath(new Point3D[]{new(0,0,-3.6986),new(10,0,-3.6996)},
            [new(0,1,ToolpathSpanKind.Ramp)],0,"clearing",0,detectCurves:false);
        Point3D N(Point3D p)=>new((float)p.X,(float)p.Y,(float)p.Z);
        Point3D Q(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),ToolpathCommandRounding.Round((float)p.Y),
            ToolpathCommandRounding.Round((float)p.Z));
        var native=new CutterSweepStock(Cone,p=>p,0,default);var posted=new CutterSweepStock(Cone,p=>p,0,default);
        var prior=EarlierCut();
        for(int i=1;i<prior.Points.Count;i++){
            native.RecordExecuted(N(prior.Points[i-1]),N(prior.Points[i]));
            posted.RecordExecuted(Q(prior.Points[i-1]),Q(prior.Points[i]));
        }
        native.Seal();posted.Seal();
        Assert.True(native.ContainsTravel(N(ramp.Points[0]),N(ramp.Points[1]),MillingLocalTravel.Clearance));
        Assert.False(posted.ContainsTravel(Q(ramp.Points[0]),Q(ramp.Points[1]),MillingLocalTravel.Clearance));
        var result=Improve([prior,ramp]);
        Assert.Equal(ramp.Points,result[1].Points);Assert.Equal(ramp.Spans,result[1].Spans);
    }

    [Fact]
    public void AnAmbiguousIncomingXYAnchorRetainsTheOriginalRampMode()
    {
        var source=new[]{EarlierCut(),Entry()};
        var result=Improve(source,incoming:new(.00004,0,-4));
        RequiredCutsStayExact(source,result);
        Assert.Equal(source[1].Spans,result[1].Spans);
    }

    [Fact]
    public void CutsInAnotherPhysicalRegionCannotAuthorizeAnAirRamp()
    {
        var source=new[]{EarlierCut() with{RegionIndex=7},Entry() with{RegionIndex=8}};
        var result=Improve(source);
        RequiredCutsStayExact(source,result);
        Assert.Equal(source[1].Spans,result[1].Spans);
    }

    [Fact]
    public void ACurvedMotionTemplateCannotUseStraightChordClearanceAsAuthority()
    {
        var entry=Entry();
        var curved=new TaggedToolpath(entry.Points,entry.Spans,entry.RegionIndex,entry.Category,entry.DepthPassIndex,detectCurves:true);
        var source=new[]{EarlierCut(),curved};var result=Improve(source);
        RequiredCutsStayExact(source,result);
        Assert.Equal(curved.Spans,result[1].Spans);
    }

    [Fact]
    public void AnInheritedFastPermissionWithoutFinalPrefixEvidenceReturnsToCuttingFeed()
    {
        var stale=new TaggedToolpath(new Point3D[]{new(0,0,-2),new(1,0,-2)},
            [new(0,1,ToolpathSpanKind.ClearedTravel)],0,"clearing",0,detectCurves:false);
        var result=Improve([stale]);
        Assert.Equal(stale.Points,result[0].Points);
        Assert.Equal(ToolpathSpanKind.Link,ToolpathSpanKinds.Expand(result[0])[0]);
        VCarveReturnTravel.VerifyFinalTravel(result,Cone,0,default,Console.WriteLine);
    }

    [Fact]
    public void AStillProvedExistingFastPermissionKeepsItsFeedAndCoordinates()
    {
        var clear=new TaggedToolpath(new Point3D[]{new(0,0,-2),new(10,0,-2)},
            [new(0,1,ToolpathSpanKind.ClearedTravel)],0,"clearing",0,detectCurves:false);
        var result=Improve([EarlierCut(),clear]);
        Assert.Equal(clear.Points,result[1].Points);Assert.Equal(clear.Spans,result[1].Spans);
        VCarveReturnTravel.VerifyFinalTravel(result,Cone,0,default,Console.WriteLine);
    }
}
