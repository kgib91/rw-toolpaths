using System.Collections;
using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class RefinedDepthMemoTests
{
    private const BindingFlags Fields=BindingFlags.Instance|BindingFlags.NonPublic;
    private static object? Field(object owner,string name)=>owner.GetType().GetField(name,Fields)!.GetValue(owner);
    private static PointD[] Box(double r)=>[new(-r,-r),new(r,-r),new(r,r),new(-r,r)];
    private static VCarveMaterialSurface Surface()
    {
        var current=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
        var prior=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,2,2)],2);
        var chord=new VCarveRoughingChord(new(0,0,-2),new(0,0,-2),0);
        var stock=new VCarveClearedStock([chord],[1d],Box(4),[new(prior,[chord])]);
        return new([Box(.04)],stock,current,1,0,default,null);
    }

    // Literal original AllowedDepth geometry/witness work, followed by its
    // original RefineDepth cache-hit branch, on the SAME immutable owner.
    private static double Original(VCarveMaterialSurface surface,double x,double y,bool clearedOnly)
    {
        double maximum=(double)Field(surface,"maximumDepth")!;
        var profile=(VCarveCutterProfile)Field(surface,"profile")!;
        var stock=(VCarveProjectedStock)Field(surface,"projectedStock")!;
        double result=maximum,unreserved=maximum,limitingClearance=0;
        PreparedRegionDistance? limiting=null;
        foreach(object layer in (IEnumerable)Field(surface,"layers")!)
        {
            double top=(double)layer.GetType().GetProperty("TopDepth")!.GetValue(layer)!;
            var distance=(PreparedRegionDistance)layer.GetType().GetProperty(clearedOnly?"ClearedDistance":"Distance")!.GetValue(layer)!;
            double clearance=distance.SignedDistance(new(x,y));
            double penetration=clearance<=VCarveMaterialSurface.PlanarReserve?0:profile.DepthAtRadius(clearance-VCarveMaterialSurface.PlanarReserve);
            result=Math.Min(result,top+penetration);
            double upper=top+(clearance<=0?0:profile.DepthAtRadius(clearance+PreparedRegionDistance.Uncertainty));
            if(upper<unreserved){unreserved=upper;limiting=distance;limitingClearance=clearance;}
        }
        PointD? witness=result<maximum&&limiting is not null
            ?limitingClearance<=0?new PointD(x,y):limiting.NearestBoundaryPoint(new(x,y)):null;
        return stock.RefineDepth(x,y,result,unreserved,clearedOnly,witness);
    }

    private static long[] Counters(VCarveProjectedStock stock)=>[stock.Queries,stock.Cells,stock.ColumnQueries,stock.ColumnTests,
        stock.CellGeometryCacheHits,stock.CellGeometryCacheMisses,stock.StockCellCacheHits,stock.StockCellCacheMisses];

    [Theory]
    [InlineData(.5)] [InlineData(1.25)]
    public void ActualAllowedDepthHitMatchesOriginalBitsWithoutStockQueriesOrHintChanges(double x)
    {
        var oldFactory=VCarveNativeStockTrace.QueryObserverFactory.Value;
        var captured=new List<VCarveNativeStockQuery>();
        try
        {
            VCarveNativeStockTrace.QueryObserverFactory.Value=_=>captured.Add;
            var surface=Surface();var stock=(VCarveProjectedStock)Field(surface,"projectedStock")!;
            Assert.False(stock.TryGetRefinedDepth(x,0,false,out _));
            double warm=surface.AllowedDepth(x,0);
            Assert.True(stock.TryGetRefinedDepth(x,0,false,out double cached));
            Assert.Equal(BitConverter.DoubleToInt64Bits(warm),BitConverter.DoubleToInt64Bits(cached));
            Assert.True(captured.Count>0);var querySequence=captured.ToArray();
            object? winner=Field(stock,"lastProfileWinner"),fitted=Field(stock,"lastFittedDepth");var counts=Counters(stock);
            double literal=Original(surface,x,0,false),fast=surface.AllowedDepth(x,0);
            Assert.Equal(BitConverter.DoubleToInt64Bits(literal),BitConverter.DoubleToInt64Bits(fast));
            Assert.Equal(BitConverter.DoubleToInt64Bits(cached),BitConverter.DoubleToInt64Bits(fast));
            Assert.Equal(winner,Field(stock,"lastProfileWinner"));Assert.Equal(fitted,Field(stock,"lastFittedDepth"));
            Assert.Equal(counts,Counters(stock));Assert.Equal(querySequence,captured);
            Assert.False(stock.TryGetRefinedDepth(x,0,true,out _));
            Assert.False(stock.TryGetRefinedDepth(x+.000001,0,false,out _));
            Assert.False(stock.TryGetRefinedDepth(double.NaN,0,false,out _));
            Assert.False(stock.CreateQueryWorker().TryGetRefinedDepth(x,0,false,out _));
        }
        finally{VCarveNativeStockTrace.QueryObserverFactory.Value=oldFactory;}
    }

    [Fact]
    public void ClearedOnlyKeysStayIndependentAndTheirHitsRetainTheSameHistory()
    {
        var surface=Surface();var stock=(VCarveProjectedStock)Field(surface,"projectedStock")!;
        var allowed=typeof(VCarveMaterialSurface).GetMethod("AllowedDepth",Fields,null,[typeof(double),typeof(double),typeof(bool)],null)!;
        double warm=(double)allowed.Invoke(surface,[.5,0d,true])!;
        Assert.True(stock.TryGetRefinedDepth(.5,0,true,out double cached));
        Assert.Equal(BitConverter.DoubleToInt64Bits(warm),BitConverter.DoubleToInt64Bits(cached));
        Assert.False(stock.TryGetRefinedDepth(.5,0,false,out _));
        object? winner=Field(stock,"lastProfileWinner"),fitted=Field(stock,"lastFittedDepth");var counts=Counters(stock);
        Assert.Equal(BitConverter.DoubleToInt64Bits(Original(surface,.5,0,true)),
            BitConverter.DoubleToInt64Bits((double)allowed.Invoke(surface,[.5,0d,true])!));
        Assert.Equal(winner,Field(stock,"lastProfileWinner"));Assert.Equal(fitted,Field(stock,"lastFittedDepth"));
        Assert.Equal(counts,Counters(stock));
    }
}
