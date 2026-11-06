using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketProtectionBatchTests
{
    private static PointD[] Box(double x0,double y0,double x1,double y1)
        => [new(x0,y0),new(x1,y0),new(x1,y1),new(x0,y1)];
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points,[new(0,points.Length-1,ToolpathSpanKind.Cut)],0,"batch-proof",0,detectCurves:false);
    private static Exception? Verify(TaggedToolpath path,double radius,IReadOnlyList<IReadOnlyList<PointD>> protection,bool batch)
        => Record.Exception(()=>PocketProtection.Verify([path],radius,0,protection,default,batch));
    private static void EqualToIndividual(TaggedToolpath path,double radius,IReadOnlyList<IReadOnlyList<PointD>> protection)
    {
        var original=Verify(path,radius,protection,false);var candidate=Verify(path,radius,protection,true);
        Assert.Equal(original?.GetType(),candidate?.GetType());
        Assert.Equal(original?.Message,candidate?.Message);
    }

    [Fact]
    public void ARefusedEnclosingShortcutStillVerifiesTheOriginalSafeCurve()
    {
        var path=Path(new(-2,0,-1),new(-2,2,-1),new(2,2,-1),new(2,0,-1));
        IReadOnlyList<PointD>[] protection=[Box(-.5,-.5,.5,.5)];
        Assert.Null(Verify(path,.1,protection,false));
        EqualToIndividual(path,.1,protection);
    }

    [Theory]
    [InlineData(0)] [InlineData(.1)] [InlineData(.5)]
    public void QuantizedTangencyAndUlpNeighboursRetainTheIndividualAnswer(double radius)
    {
        IReadOnlyList<PointD>[] protection=[Box(0,0,1,1)];
        foreach(double ticks in new[]{-1d,0,3,4,5,8,9})
        foreach(double y in new[]{1+radius+ticks/32768,Math.BitDecrement(1+radius+ticks/32768),Math.BitIncrement(1+radius+ticks/32768)})
        {
            var path=Path(Enumerable.Range(0,70).Select(i=>new Point3D(-1+3d*i/69,y,-1)).ToArray());
            EqualToIndividual(path,radius,protection);
        }
    }

    [Fact]
    public void EveryChordIsStillCheckedWhenACurveEntersProtectedMaterial()
    {
        var points=Enumerable.Range(0,97).Select(i=>new Point3D(-3+6d*i/96,2,-1)).ToArray();
        points[47]=new(0,0,-1);
        IReadOnlyList<PointD>[] protection=[Box(-.5,-.5,.5,.5)];
        Assert.IsType<InvalidOperationException>(Verify(Path(points),.1,protection,false));
        EqualToIndividual(Path(points),.1,protection);
    }

    [Fact]
    public void RandomCurvesAndSignedHolesMatchTheOriginalCompleteCapsuleOracle()
    {
        IReadOnlyList<PointD>[] protection=[Box(-3,-3,3,3),Box(-1,-1,1,1).Reverse().ToArray(),Box(5,-1,7,1)];
        var random=new Random(74821);
        for(int example=0;example<100;example++)
        {
            double x=-5+14*random.NextDouble(),y=-5+10*random.NextDouble();
            var points=Enumerable.Range(0,65).Select(i=>new Point3D(x+.4*Math.Sin(i*.1),y+.3*Math.Cos(i*.13),i%11==0?.2:-1)).ToArray();
            EqualToIndividual(Path(points),example%3*.05,protection);
        }
    }

    [Theory]
    [InlineData(-10000)] [InlineData(10000)] [InlineData(10001)]
    public void LargeCoordinatesAndFallbackDomainsRetainTheOriginalAnswer(double shift)
    {
        IReadOnlyList<PointD>[] protection=[Box(shift-1,shift-1,shift+1,shift+1)];
        var path=Path(Enumerable.Range(0,65).Select(i=>new Point3D(shift-2+4d*i/64,shift+2,-1)).ToArray());
        EqualToIndividual(path,.1,protection);
    }

    [Fact]
    public void InvalidValuesAndAirOnlyInputsKeepTheOriginalValidation()
    {
        IReadOnlyList<PointD>[] protection=[Box(-1,-1,1,1)];
        foreach(double radius in new[]{-1d,double.NaN,double.PositiveInfinity})
        foreach(double z in new[]{-1d,1d})
            EqualToIndividual(Path(Enumerable.Repeat(new Point3D(3,0,z),33).ToArray()),radius,protection);
        foreach(double z in new[]{-1d,1d})
            EqualToIndividual(Path(Enumerable.Repeat(new Point3D(double.NaN,0,z),33).ToArray()),.1,protection);
    }

    [Fact]
    public void StationaryVerticesAirAndCancellationRetainTheOriginalRules()
    {
        IReadOnlyList<PointD>[] protection=[Box(-1,-1,1,1)];
        EqualToIndividual(Path(Enumerable.Repeat(new Point3D(3,0,-1),65).ToArray()),.1,protection);
        EqualToIndividual(Path(Enumerable.Repeat(new Point3D(0,0,1),65).ToArray()),.1,protection);
        using var cancellation=new CancellationTokenSource();cancellation.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(()=>PocketProtection.Verify([Path(new(3,0,-1),new(4,0,-1))],.1,0,protection,cancellation.Token));
    }
}
