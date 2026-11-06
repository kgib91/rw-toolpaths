using System.IO.Compression;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ScanlinePreparedProtectionTests
{
    private static PointD[] Box(double x0,double y0,double x1,double y1)=>[new(x0,y0),new(x1,y0),new(x1,y1),new(x0,y1)];
    private static IReadOnlyList<PointD> Hole(PointD[] points)=>points.Reverse().ToArray();
    private static byte[] Encode(IReadOnlyList<MillingRegion> regions)
    {
        using var memory=new MemoryStream();using(var writer=new BinaryWriter(memory,System.Text.Encoding.UTF8,true))
        {
            writer.Write(regions.Count);
            foreach(var region in regions)
            {
                void Ring(IReadOnlyList<PointD> points)
                {writer.Write(points.Count);foreach(var point in points){writer.Write(point.x);writer.Write(point.y);}}
                Ring(region.Outer);writer.Write(region.Holes.Count);foreach(var hole in region.Holes)Ring(hole);
            }
        }
        return memory.ToArray();
    }
    private static void Oracle(IReadOnlyList<MillingRegion> regions,PocketOptions options)
    {
        var source=Encode(regions);var masks=options.ProtectedBoundary!.Select(p=>p.ToArray()).ToArray();
        var original=PocketProtection.Subtract(regions,options,default,preparedScanlineDifference:false);
        var prepared=PocketProtection.Subtract(regions,options,default,preparedScanlineDifference:true);
        Assert.Equal(Encode(original),Encode(prepared));Assert.Equal(source,Encode(regions));
        for(int i=0;i<masks.Length;i++)Assert.Equal(masks[i],options.ProtectedBoundary[i]);
        // Output mutation cannot poison the next call's private prepared input.
        foreach(var region in prepared)if(region.Outer is List<PointD> points)points.Clear();
        Assert.Equal(Encode(original),Encode(PocketProtection.Subtract(regions,options,default,preparedScanlineDifference:true)));
    }

    [Theory]
    [InlineData(0)] [InlineData(.25)] [InlineData(-.2)]
    public void HolesIslandsPlatformEdgesAndSeparatedRegionsKeepEveryOrderedCoordinate(double allowance)
    {
        MillingRegion[] regions=[new(Box(-20,-20,20,20),[Hole(Box(-17,-17,-12,-12))]),
            new(Box(30,-10,50,10)),new(Box(100,100,110,110))];
        IReadOnlyList<PointD>[] mask=[Box(-10,-10,10,10),Hole(Box(-7,-7,7,7)),Box(-3,-3,3,3),
            Box(38,-15,42,15),Box(1000,-10,1010,10)];
        Oracle(regions,new(){ProtectedBoundary=mask,StockToLeave=allowance});
    }

    [Fact]
    public void EnclosingMasksNestedHolesTouchingMinimaAndNegativeCoordinatesMatchTheFullClipOperand()
    {
        MillingRegion[] regions=[new(Box(-8,-8,8,8)),new(Box(24,-3,30,3)),new(Box(40,-3,46,3))];
        IReadOnlyList<PointD>[] mask=[Box(-20,-20,20,20),Hole(Box(-10,-10,10,10)),Box(-2,-2,2,2),
            Box(27,-3,30,0),Box(46,-3,50,0),Box(500,-3,510,0)];
        Oracle(regions,new(){ProtectedBoundary=mask});
    }

    [Fact]
    public void ActualDenseSleeveContoursKeepTheOriginalExpandedMaskAndRegionCycleOrder()
    {
        using var input=typeof(ScanlinePreparedProtectionTests).Assembly.GetManifestResourceStream(
            "RW.Toolpaths.Tests.Fixtures.vcarve-sleeve-pocket.json.gz")!;
        using var gzip=new GZipStream(input,CompressionMode.Decompress);using var document=JsonDocument.Parse(gzip);
        var mask=document.RootElement.GetProperty("boundary").EnumerateArray().Select(r=>(IReadOnlyList<PointD>)r.EnumerateArray()
            .Select(p=>new PointD(p[0].GetDouble(),p[1].GetDouble())).ToArray()).ToArray();
        Assert.NotEmpty(mask.SelectMany(r=>r));
        double left=mask.SelectMany(r=>r).Min(p=>p.x)-2,right=mask.SelectMany(r=>r).Max(p=>p.x)+2;
        double bottom=mask.SelectMany(r=>r).Min(p=>p.y)-2,top=mask.SelectMany(r=>r).Max(p=>p.y)+2,mid=(left+right)/2;
        MillingRegion[] regions=[new(Box(left,bottom,right,top)),new(Box(left,bottom,mid,top)),
            new(Box(mid,bottom,right,top)),new(Box(right+10,bottom,right+20,top))];
        Oracle(regions,new(){ProtectedBoundary=mask,Tolerances=new(){ArcTolerance=.001,SimplifyTolerance=.001}});
    }

    [Fact]
    public void EmptyProtectionAndCanceledRequestsRetainTheOriginalContracts()
    {
        MillingRegion[] regions=[new(Box(0,0,10,10))];var options=new PocketOptions();
        Assert.Same(regions,PocketProtection.Subtract(regions,options,default,preparedScanlineDifference:true));
        using var cancellation=new CancellationTokenSource();cancellation.Cancel();
        options=options with{ProtectedBoundary=[Box(4,4,6,6)]};
        Assert.ThrowsAny<OperationCanceledException>(()=>PocketProtection.Subtract(regions,options,cancellation.Token,preparedScanlineDifference:false));
        Assert.ThrowsAny<OperationCanceledException>(()=>PocketProtection.Subtract(regions,options,cancellation.Token,preparedScanlineDifference:true));
    }
}
