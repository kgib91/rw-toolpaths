using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveRampSurfaceTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PreparedMedialSitesRetainEveryLiteralSerialDepth(bool rounded)
    {
        PointD[] boundary = [new(0,0),new(20,0),new(20,15),new(12,15),new(12,8),new(8,8),new(8,15),new(0,15)];
        var profile = rounded
            ? new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),new(CarvingCutterSectionKind.Linear,1,3,5)],6)
            : new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,3,6)],6);
        var points = Enumerable.Range(0,3072).Select(i => new Point3D(
            .25+19.5*(i%128)/127d, .25+14.5*(i/128)/23d, -2.5-.3*Math.Sin(i))).ToArray();
        var path = new TaggedToolpath(points,Array.Empty<ToolpathSpan>(),0,"final-carve",null,detectCurves:false);
        var upper = path with { Points = points.Select(p=>p with {Z=p.Z-.01}).ToArray() };
        var options = new VCarveOptions {SmoothRampAngleDegrees=35,ParallelFloorQueryPreparation=false};
        var serial = VCarveRampSurface.FitMedial([path],[upper],[boundary],options,profile,6,default);
        var prepared = VCarveRampSurface.FitMedial([path],[upper],[boundary],
            options with {ParallelFloorQueryPreparation=true},profile,6,default);
        Assert.Equal(serial.Limited[0].Points,prepared.Limited[0].Points);
        Assert.Equal(serial.Upper[0].Points,prepared.Upper[0].Points);
        Assert.Equal(points.Select(p=>(p.X,p.Y)),prepared.Limited[0].Points.Select(p=>(p.X,p.Y)));
    }
}
