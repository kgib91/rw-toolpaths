using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveAirRampCompactionOrderTests
{
    [Fact]
    public void AFinalMergedRampMustBeReprovedInsteadOfInheritingItsSubdivisionsFeedPermission()
    {
        double surface=(double)(float)12.7;
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,.79375,.79375),
            new(CarvingCutterSectionKind.Linear,.79375,3,20),new(CarvingCutterSectionKind.Linear,3,3,90.4)],6.35);
        var motion=new MillingMotionSettings(1000,500,5000,surface+5,surface);
        var donor=new TaggedToolpath(new Point3D[]{
            new(180.1943359375,280.6305847167969,11.531267051973831),
            new(180.13821411132812,280.6802062988281,11.531311992786861)},
            [new(0,1,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false);
        var pad=new Point3D(180.1263061610888,280.6195143989556,12.41474836224381);
        var a=new Point3D(180.12721023194408,280.620042067599,12.414224965093304);
        var b=new Point3D(180.12901837365462,280.62109740488586,12.413178170792303);
        var c=new Point3D(180.13263465707567,280.6232080794596,12.411084582190314);
        var ramp=new TaggedToolpath(new[]{pad,a,b,c},[new(0,3,ToolpathSpanKind.Ramp)],0,"clearing",0,detectCurves:false);
        Point3D N(Point3D p)=>new((float)p.X,(float)p.Y,(float)p.Z-(float)surface);
        Point3D Q(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),ToolpathCommandRounding.Round((float)p.Y),
            ToolpathCommandRounding.Round((float)p.Z-(float)surface));
        var native=new CutterSweepStock(profile,p=>p,surface,default);
        var posted=new CutterSweepStock(profile,p=>p,surface,default);
        native.RecordExecuted(N(donor.Points[0]),N(donor.Points[1]));
        posted.RecordExecuted(Q(donor.Points[0]),Q(donor.Points[1]));native.Seal();posted.Seal();

        // This captured single-donor fixture demonstrates conservative proof
        // incompleteness, not a physical collision. The F3 midpoint is exactly
        // one third along AC in integer command coordinates. Each shorter
        // interval has a whole-cutter certificate, while the current combined
        // min-depth/max-distance bound cannot certify their merged interval.
        Assert.True(native.ContainsTravel(N(a),N(b),MillingLocalTravel.Clearance));
        Assert.True(native.ContainsTravel(N(b),N(c),MillingLocalTravel.Clearance));
        Assert.True(native.ContainsTravel(N(a),N(c),MillingLocalTravel.Clearance));
        Assert.True(posted.ContainsTravel(Q(a),Q(b),MillingLocalTravel.Clearance));
        Assert.True(posted.ContainsTravel(Q(b),Q(c),MillingLocalTravel.Clearance));
        Assert.False(posted.ContainsTravel(Q(a),Q(c),MillingLocalTravel.Clearance));

        var subdivided=VCarveAirRampFeed.Improve([donor,ramp],profile,surface,motion,default,Console.WriteLine);
        var smallerKinds=ToolpathSpanKinds.Expand(subdivided[1]);
        Assert.Equal(ToolpathSpanKind.ClearedTravel,smallerKinds[1]);
        Assert.Equal(ToolpathSpanKind.ClearedTravel,smallerKinds[2]);
        VCarveReturnTravel.VerifyFinalTravel(subdivided,profile,surface,default,Console.WriteLine);

        var incorrectlyInherited=subdivided.Select(p=>ToolpathCollinearCompactor.Compact(p,surface)).ToArray();
        Assert.DoesNotContain(b,incorrectlyInherited[1].Points);
        Assert.Throws<InvalidOperationException>(()=>VCarveReturnTravel.VerifyFinalTravel(
            incorrectlyInherited,profile,surface,default,Console.WriteLine,finalCompactionComplete:true));

        var compacted=ToolpathCollinearCompactor.Compact(ramp,surface);
        Assert.Equal(new[]{pad,a,c},compacted.Points);
        var final=VCarveAirRampFeed.Improve([donor,compacted],profile,surface,motion,default,Console.WriteLine);
        Assert.Equal(compacted.Points,final[1].Points);
        Assert.Equal(ToolpathSpanKind.Ramp,ToolpathSpanKinds.Expand(final[1])[^1]);
        Assert.Equal(donor.Points,final[0].Points);Assert.Equal(donor.Spans,final[0].Spans);
        VCarveReturnTravel.VerifyFinalTravel(final,profile,surface,default,Console.WriteLine,finalCompactionComplete:true);
    }
}
